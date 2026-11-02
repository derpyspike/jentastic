// Unit tests for the Nexus JenCore policy helpers (JenCorePolicy.h).
//
// What is pinned:
//   - TRACE flags bits 0-1 are the hash-size exponent (1/2/4/8), not the node
//     PATHSIZE. Only 1/2/4 may be originated; 3 and 8 are rejected on send.
//   - blocksTrace() treats a one-entry TRACE as a ping and everything larger
//     as a traceroute, with independent block bits.
//   - burner aliases stamp a random prefix and are claimed as a next hop only
//     when ROUTE is on; the real public-key prefix is always claimed.
//   - parseTraceSummary() reads tag/auth/flags, requested vs completed hops and
//     the endpoint condition from a TRACE payload.
//
// The regression guarded: using the node path hash size as the TRACE flags
// exponent produced a 4-byte trace for PATHSIZE 3 while writing 3-byte hashes,
// so 3-byte-pathsize traces were malformed and never completed.

#include "mesh/JenCorePolicy.h"
#include <Arduino.h> // declares setup()/loop() with C linkage
#include <cstring>
#include <unity.h>

using namespace nexus::jen_core_policy;

void setUp(void) {}
void tearDown(void) {}

static void test_trace_hash_size_mapping(void)
{
    TEST_ASSERT_EQUAL_UINT8(1, traceHashSize(0));
    TEST_ASSERT_EQUAL_UINT8(2, traceHashSize(1));
    TEST_ASSERT_EQUAL_UINT8(4, traceHashSize(2));
    TEST_ASSERT_EQUAL_UINT8(8, traceHashSize(3));
}

static void test_trace_sizes_allowed(void)
{
    TEST_ASSERT_TRUE(traceSizeSupported(1));
    TEST_ASSERT_TRUE(traceSizeSupported(2));
    TEST_ASSERT_TRUE(traceSizeSupported(4));
    TEST_ASSERT_FALSE(traceSizeSupported(3));
    TEST_ASSERT_FALSE(traceSizeSupported(8));

    TEST_ASSERT_EQUAL_UINT8(0, traceFlagsForSize(1));
    TEST_ASSERT_EQUAL_UINT8(1, traceFlagsForSize(2));
    TEST_ASSERT_EQUAL_UINT8(2, traceFlagsForSize(4));
    TEST_ASSERT_EQUAL_UINT8(0xFF, traceFlagsForSize(3));
    TEST_ASSERT_EQUAL_UINT8(0xFF, traceFlagsForSize(8));
}

static void test_blocks_trace_vs_ping(void)
{
    Policy p;
    TEST_ASSERT_FALSE(blocksTrace(p, 0));
    TEST_ASSERT_FALSE(blocksTrace(p, 1));
    TEST_ASSERT_FALSE(blocksTrace(p, 2));

    set(p, FEATURE_BLOCK_PING, true);
    TEST_ASSERT_TRUE(blocksTrace(p, 0));
    TEST_ASSERT_TRUE(blocksTrace(p, 1));
    TEST_ASSERT_FALSE(blocksTrace(p, 2));

    set(p, FEATURE_BLOCK_TRACE, true);
    TEST_ASSERT_TRUE(blocksTrace(p, 2));
    TEST_ASSERT_TRUE(blocksTrace(p, 9));
}

static void test_burner_stamp_and_truncate(void)
{
    Policy p;
    p.burners[0][0] = 0xAA;
    p.burners[0][1] = 0xBB;
    p.burners[0][2] = 0xCC;
    p.burners[0][3] = 0x00; // derpycore alias is 3 bytes + zero pad
    p.burner_count = 1;
    set(p, FEATURE_BURNER_ON, true);

    uint8_t identity[32];
    memset(identity, 0x11, sizeof(identity));

    uint8_t out[4] = {};
    stampEntry(p, identity, 1, 0, out);
    TEST_ASSERT_EQUAL_HEX8(0xAA, out[0]);
    stampEntry(p, identity, 2, 0, out);
    TEST_ASSERT_EQUAL_HEX8(0xAA, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0xBB, out[1]);
    stampEntry(p, identity, 4, 0, out);
    TEST_ASSERT_EQUAL_HEX8(0xAA, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0xBB, out[1]);
    TEST_ASSERT_EQUAL_HEX8(0xCC, out[2]);
    TEST_ASSERT_EQUAL_HEX8(0x00, out[3]);

    set(p, FEATURE_BURNER_ON, false);
    stampEntry(p, identity, 2, 0, out);
    TEST_ASSERT_EQUAL_HEX8(0x11, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0x11, out[1]);
}

static void test_burner_match_and_route_claim(void)
{
    Policy p;
    p.burners[0][0] = 0xAA;
    p.burners[0][1] = 0xBB;
    p.burners[0][2] = 0xCC;
    p.burner_count = 1;

    uint8_t identity[32];
    memset(identity, 0x11, sizeof(identity));
    const uint8_t alias3[3] = {0xAA, 0xBB, 0xCC};
    const uint8_t alias2[2] = {0xAA, 0xBB};
    const uint8_t alias1[1] = {0xAA};
    const uint8_t wrong[2] = {0xAA, 0x00};
    const uint8_t id2[2] = {0x11, 0x11};

    TEST_ASSERT_TRUE(isBurnerEntry(p, alias1, 1));
    TEST_ASSERT_TRUE(isBurnerEntry(p, alias2, 2));
    TEST_ASSERT_TRUE(isBurnerEntry(p, alias3, 3));
    TEST_ASSERT_FALSE(isBurnerEntry(p, wrong, 2));

    // Real identity always matches, even with ROUTE off.
    TEST_ASSERT_TRUE(matchesNextHop(p, id2, 2, identity));
    TEST_ASSERT_FALSE(matchesNextHop(p, alias2, 2, identity));

    set(p, FEATURE_ROUTE_INTERVENE, true);
    TEST_ASSERT_TRUE(matchesNextHop(p, alias2, 2, identity));
    TEST_ASSERT_TRUE(matchesNextHop(p, id2, 2, identity));
}

static void test_advert_name_override(void)
{
    Policy p;
    TEST_ASSERT_EQUAL_STRING("owner-name", advertName(p, "owner-name"));

    strcpy(p.name, "jen-name");
    TEST_ASSERT_EQUAL_STRING("owner-name", advertName(p, "owner-name")); // override bit still off

    set(p, FEATURE_NAME_SET, true);
    TEST_ASSERT_EQUAL_STRING("jen-name", advertName(p, "owner-name"));

    set(p, FEATURE_NAME_SET, false);
    TEST_ASSERT_EQUAL_STRING("owner-name", advertName(p, "owner-name"));
}

static void buildTrace(uint8_t *payload, size_t &len, uint8_t flags, uint32_t tag, const uint8_t *hashes, size_t hashBytes)
{
    memset(payload, 0, 64);
    memcpy(payload, &tag, 4);
    memset(payload + 4, 0x5A, 4); // auth
    payload[8] = flags;
    memcpy(payload + 9, hashes, hashBytes);
    len = 9 + hashBytes;
}

static void test_parse_trace_summary(void)
{
    const uint8_t hashes[8] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11};
    uint8_t payload[64];
    size_t len = 0;

    buildTrace(payload, len, 1 /* 2-byte */, 0x12345678, hashes, 6);
    TraceSummary s;
    TEST_ASSERT_TRUE(parseTraceSummary(payload, len, 1, s));
    TEST_ASSERT_EQUAL_HEX32(0x12345678, s.tag);
    TEST_ASSERT_EQUAL_HEX32(0x5A5A5A5A, s.auth);
    TEST_ASSERT_EQUAL_UINT8(2, s.hashSize);
    TEST_ASSERT_EQUAL_UINT8(3, s.requestedHops);
    TEST_ASSERT_EQUAL_UINT8(1, s.completedHops);
    TEST_ASSERT_FALSE(s.endpoint);

    TEST_ASSERT_TRUE(parseTraceSummary(payload, len, 3, s));
    TEST_ASSERT_TRUE(s.endpoint);

    // 4-byte trace: flags 2.
    buildTrace(payload, len, 2, 0x01020304, hashes, 8);
    TEST_ASSERT_TRUE(parseTraceSummary(payload, len, 2, s));
    TEST_ASSERT_EQUAL_UINT8(4, s.hashSize);
    TEST_ASSERT_EQUAL_UINT8(2, s.requestedHops);
    TEST_ASSERT_TRUE(s.endpoint);

    // Truncated payload / not a multiple of the hash size.
    TEST_ASSERT_FALSE(parseTraceSummary(payload, 8, 0, s));
    buildTrace(payload, len, 2, 0x01020304, hashes, 5);
    TEST_ASSERT_FALSE(parseTraceSummary(payload, len, 0, s));
}

static void test_policy_sanitize(void)
{
    Policy p;
    p.burner_count = 200;
    p.fake_fixed_count = 200;
    p.fake_hops = 200;
    p.features = ~0ull;
    memset(p.name, 'A', sizeof(p.name));

    sanitize(p);
    TEST_ASSERT_EQUAL_UINT8(BURNER_SLOTS, p.burner_count);
    TEST_ASSERT_EQUAL_UINT8(BURNER_SLOTS, p.fake_fixed_count);
    TEST_ASSERT_EQUAL_UINT8(8, p.fake_hops);
    TEST_ASSERT_EQUAL_UINT64(ALL_FEATURES_MASK, p.features);
    TEST_ASSERT_EQUAL_HEX8(0, p.name[NAME_LEN - 1]);
}

void setup()
{
    UNITY_BEGIN();
    RUN_TEST(test_trace_hash_size_mapping);
    RUN_TEST(test_trace_sizes_allowed);
    RUN_TEST(test_blocks_trace_vs_ping);
    RUN_TEST(test_burner_stamp_and_truncate);
    RUN_TEST(test_burner_match_and_route_claim);
    RUN_TEST(test_advert_name_override);
    RUN_TEST(test_parse_trace_summary);
    RUN_TEST(test_policy_sanitize);
    exit(UNITY_END());
}

void loop() {}
