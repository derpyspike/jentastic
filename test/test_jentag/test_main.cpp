// Unit tests for the JenTag (OpenHaystack-compatible) BLE advertisement.
//
// What is pinned:
//   - the exact 31-byte legacy advertising payload used by OpenHaystack's
//     ESP32 firmware: Apple company ID, offline-finding type/length, the LAST
//     22 bytes of the 28-byte public key, and the two high bits of key[0];
//   - the random static address derived from the first six key bytes.
//
// The regression guarded: an earlier implementation copied the FIRST 22 key
// bytes and zeroed the status nibble, so the advertisement was not recognized
// by the Find My network even though it had the right length.

#include "mesh/JenTag.h"
#include <Arduino.h> // declares setup()/loop() with C linkage
#include <cstring>
#include <unity.h>

using namespace nexus::jentag;

void setUp(void) {}
void tearDown(void) {}

static void test_advert_matches_openhaystack(void)
{
    uint8_t key[KEY_LEN];
    for (size_t i = 0; i < sizeof(key); i++)
        key[i] = (uint8_t)(0x10 + i);

    uint8_t out[ADVERT_LEN];
    buildAdvertisement(key, out);

    TEST_ASSERT_EQUAL_UINT8(0x1e, out[0]);
    TEST_ASSERT_EQUAL_UINT8(0xff, out[1]);
    TEST_ASSERT_EQUAL_UINT8(0x4c, out[2]);
    TEST_ASSERT_EQUAL_UINT8(0x00, out[3]);
    TEST_ASSERT_EQUAL_UINT8(0x12, out[4]);
    TEST_ASSERT_EQUAL_UINT8(0x19, out[5]);
    TEST_ASSERT_EQUAL_UINT8(0x00, out[6]);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(&key[6], &out[7], 22);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)(key[0] >> 6), out[29]);
    TEST_ASSERT_EQUAL_UINT8(0x00, out[30]);
}

static void test_advert_uses_key_tail_not_head(void)
{
    uint8_t key[KEY_LEN] = {};
    for (size_t i = 0; i < sizeof(key); i++)
        key[i] = (uint8_t)(0xA0 + i); // key[0] = 0xA0, top two bits are 0b10

    uint8_t out[ADVERT_LEN];
    buildAdvertisement(key, out);

    TEST_ASSERT_EQUAL_UINT8_ARRAY(&key[6], &out[7], 22);
    TEST_ASSERT_EQUAL_UINT8(0x02, out[29]);
    TEST_ASSERT_TRUE(memcmp(&key[0], &out[7], 22) != 0);
}

static void test_address_derivation(void)
{
    uint8_t key[KEY_LEN] = {};
    key[0] = 0x12;
    for (size_t i = 1; i < sizeof(key); i++)
        key[i] = (uint8_t)(0x20 + i);

    uint8_t addr[6];
    deriveAddress(key, addr);

    TEST_ASSERT_EQUAL_UINT8(0xD2, addr[0]); // 0x12 | 0xC0
    TEST_ASSERT_EQUAL_UINT8(0x21, addr[1]);
    TEST_ASSERT_EQUAL_UINT8(0x22, addr[2]);
    TEST_ASSERT_EQUAL_UINT8(0x23, addr[3]);
    TEST_ASSERT_EQUAL_UINT8(0x24, addr[4]);
    TEST_ASSERT_EQUAL_UINT8(0x25, addr[5]);
}

void setup()
{
    UNITY_BEGIN();
    RUN_TEST(test_advert_matches_openhaystack);
    RUN_TEST(test_advert_uses_key_tail_not_head);
    RUN_TEST(test_address_derivation);
    exit(UNITY_END());
}

void loop() {}
