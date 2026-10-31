// Unit tests for the Nexus-only JenCore metadata packed into MeshPacket
// (jc_flags / jc_relay_node / jc_path).
//
// What is pinned:
//   - the jc_flags bit layout (is_jencore, region_scoped, path_truncated,
//     route, payload type, hash size, region index), including the
//     "not JenCore => flags == 0" invariant that keeps the field absent;
//   - copyPathTail() keeps the *tail* of the path (the immediate relay) and
//     only ever returns whole hashes, flagging truncation;
//   - the fields survive a FromRadio encode/decode round-trip;
//   - a max payload + max signature + full jc_path still encodes inside the
//     512-byte FromRadio budget.
//
// The regression guarded: truncating the head instead of the tail loses the
// relay the client wants to display, and a half hash makes the path unusable.

#include "mesh/JenCoreMeta.h"
#include "mesh/mesh-pb-constants.h"
#include <Arduino.h> // declares setup()/loop() with C linkage
#include <cstring>
#include <pb_encode.h>
#include <unity.h>

using namespace nexus::jen_core_meta;

void setUp(void) {}
void tearDown(void) {}

static uint16_t flagField(uint16_t flags, uint16_t shift, uint16_t mask) { return (flags >> shift) & mask; }

static void test_flags_layout(void)
{
    const uint16_t f = packFlags(true, true, false, 2, 5, 3, 1);
    TEST_ASSERT_TRUE(f & FLAG_IS_JENCORE);
    TEST_ASSERT_TRUE(f & FLAG_REGION_SCOPED);
    TEST_ASSERT_FALSE(f & FLAG_PATH_TRUNCATED);
    TEST_ASSERT_EQUAL_UINT8(2, flagField(f, ROUTE_SHIFT, ROUTE_MASK));
    TEST_ASSERT_EQUAL_UINT8(5, flagField(f, PAYLOAD_SHIFT, PAYLOAD_MASK));
    TEST_ASSERT_EQUAL_UINT8(2, flagField(f, HASH_SIZE_SHIFT, HASH_SIZE_MASK)); // size-1
    TEST_ASSERT_EQUAL_UINT8(1, flagField(f, REGION_SHIFT, REGION_MASK));
}

static void test_flags_truncated_bit(void)
{
    const uint16_t f = packFlags(true, false, true, 0, 0, 1, 0);
    TEST_ASSERT_TRUE(f & FLAG_PATH_TRUNCATED);
    TEST_ASSERT_FALSE(f & FLAG_REGION_SCOPED);
}

static void test_flags_region_none(void)
{
    const uint16_t f = packFlags(true, false, false, 0, 0, 1, 0xFF);
    TEST_ASSERT_EQUAL_UINT8(REGION_NONE, flagField(f, REGION_SHIFT, REGION_MASK));
}

static void test_flags_zero_when_not_jencore(void)
{
    // Proto3 only encodes a singular field when it is non-zero; a zero flags
    // word is what keeps an ordinary Meshtastic packet free of this metadata.
    TEST_ASSERT_EQUAL_HEX16(0, packFlags(false, false, false, 0, 0, 0, 0xFF));
}

static void test_path_tail_keeps_last_hashes(void)
{
    uint8_t src[39];
    for (size_t i = 0; i < sizeof(src); i++)
        src[i] = (uint8_t)(0x40 + i);

    uint8_t out[PATH_CAP] = {};
    bool truncated = false;
    const size_t n = copyPathTail(src, sizeof(src), 3, out, sizeof(out), &truncated);

    TEST_ASSERT_EQUAL_UINT32(27, n); // 9 whole 3-byte hashes
    TEST_ASSERT_TRUE(truncated);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(src + sizeof(src) - n, out, n);
}

static void test_path_tail_short_path_not_truncated(void)
{
    uint8_t src[12];
    for (size_t i = 0; i < sizeof(src); i++)
        src[i] = (uint8_t)i;

    uint8_t out[PATH_CAP] = {};
    bool truncated = true;
    const size_t n = copyPathTail(src, sizeof(src), 3, out, sizeof(out), &truncated);

    TEST_ASSERT_EQUAL_UINT32(12, n);
    TEST_ASSERT_FALSE(truncated);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(src, out, 12);
}

static void test_path_tail_one_byte_hashes(void)
{
    uint8_t src[64];
    for (size_t i = 0; i < sizeof(src); i++)
        src[i] = (uint8_t)(i + 1);

    uint8_t out[PATH_CAP] = {};
    bool truncated = false;
    const size_t n = copyPathTail(src, sizeof(src), 1, out, sizeof(out), &truncated);

    TEST_ASSERT_EQUAL_UINT32(PATH_CAP, n);
    TEST_ASSERT_TRUE(truncated);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(src + sizeof(src) - PATH_CAP, out, PATH_CAP);
}

static void test_path_tail_invalid_inputs(void)
{
    uint8_t out[PATH_CAP] = {};
    bool truncated = true;
    TEST_ASSERT_EQUAL_UINT32(0, copyPathTail(nullptr, 10, 3, out, sizeof(out), &truncated));
    TEST_ASSERT_FALSE(truncated);
    TEST_ASSERT_EQUAL_UINT32(0, copyPathTail(out, 0, 3, out, sizeof(out), &truncated));
    TEST_ASSERT_EQUAL_UINT32(0, copyPathTail(out, 10, 0, out, sizeof(out), &truncated));
    TEST_ASSERT_EQUAL_UINT32(0, copyPathTail(out, 10, 3, out, 0, &truncated));
}

static void test_fromradio_roundtrip(void)
{
    uint8_t path[PATH_CAP];
    for (size_t i = 0; i < sizeof(path); i++)
        path[i] = (uint8_t)(0xA0 + i);

    meshtastic_MeshPacket p = meshtastic_MeshPacket_init_zero;
    p.from = 0x4D000001;
    p.to = 0xFFFFFFFF;
    p.id = 1234;
    p.which_payload_variant = meshtastic_MeshPacket_decoded_tag;
    p.decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;
    const char msg[] = "hello";
    p.decoded.payload.size = sizeof(msg) - 1;
    memcpy(p.decoded.payload.bytes, msg, p.decoded.payload.size);
    apply(p, packFlags(true, true, true, 0, 0x05, 3, 2), 0x4DAB, path, sizeof(path));

    meshtastic_FromRadio in = meshtastic_FromRadio_init_zero;
    in.which_payload_variant = meshtastic_FromRadio_packet_tag;
    in.packet = p;

    uint8_t buf[meshtastic_FromRadio_size] = {};
    const size_t len = pb_encode_to_bytes(buf, sizeof(buf), &meshtastic_FromRadio_msg, &in);
    TEST_ASSERT_GREATER_THAN(0, len);

    meshtastic_FromRadio out = meshtastic_FromRadio_init_zero;
    TEST_ASSERT_TRUE(pb_decode_from_bytes(buf, len, &meshtastic_FromRadio_msg, &out));
    TEST_ASSERT_EQUAL_HEX16(p.jc_flags, out.packet.jc_flags);
    TEST_ASSERT_EQUAL_HEX16(p.jc_relay_node, out.packet.jc_relay_node);
    TEST_ASSERT_EQUAL_UINT32(p.jc_path.size, out.packet.jc_path.size);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(p.jc_path.bytes, out.packet.jc_path.bytes, p.jc_path.size);
}

static void test_declared_meshpacket_size_bounds_all_fields(void)
{
    // Every field at its maximum, including the Nexus additions: the generated
    // size constant must still be the upper bound, or the FromRadio buffers it
    // sizes would truncate a real packet.
    meshtastic_MeshPacket p = meshtastic_MeshPacket_init_zero;
    p.from = 0xFFFFFFFF;
    p.to = 0xFFFFFFFF;
    p.channel = 0xFF;
    p.which_payload_variant = meshtastic_MeshPacket_decoded_tag;
    p.decoded.portnum = _meshtastic_PortNum_MAX;
    p.decoded.payload.size = sizeof(p.decoded.payload.bytes);
    memset(p.decoded.payload.bytes, 0xAA, p.decoded.payload.size);
    p.decoded.want_response = true;
    p.decoded.dest = 0xFFFFFFFF;
    p.decoded.source = 0xFFFFFFFF;
    p.decoded.request_id = 0xFFFFFFFF;
    p.decoded.reply_id = 0xFFFFFFFF;
    p.decoded.emoji = 0xFFFFFFFF;
    p.decoded.has_bitfield = true;
    p.decoded.bitfield = 0xFF;
    p.decoded.xeddsa_signature.size = sizeof(p.decoded.xeddsa_signature.bytes);
    memset(p.decoded.xeddsa_signature.bytes, 0xBB, p.decoded.xeddsa_signature.size);
    p.id = 0xFFFFFFFF;
    p.has_rx_time = true;
    p.rx_time = 0xFFFFFFFF;
    p.rx_snr = 1.0f;
    p.hop_limit = 0xFF;
    p.want_ack = true;
    p.priority = _meshtastic_MeshPacket_Priority_MAX;
    p.has_rx_rssi = true;
    p.rx_rssi = -1;
    p.delayed = _meshtastic_MeshPacket_Delayed_MAX;
    p.via_mqtt = true;
    p.hop_start = 0xFF;
    p.public_key.size = sizeof(p.public_key.bytes);
    memset(p.public_key.bytes, 0xCC, p.public_key.size);
    p.pki_encrypted = true;
    p.next_hop = 0xFF;
    p.relay_node = 0xFF;
    p.tx_after = 0xFFFFFFFF;
    p.transport_mechanism = _meshtastic_MeshPacket_TransportMechanism_MAX;
    p.xeddsa_signed = true;
    p.ack_proof_status = _meshtastic_MeshPacket_AckProofStatus_MAX;
    p.rx_frequency_error = 1.0f;
    p.rx_coding_rate = 0xFF;
    p.jc_flags = 0xFFFF;
    p.jc_relay_node = 0xFFFF;
    p.jc_path.size = sizeof(p.jc_path.bytes);
    memset(p.jc_path.bytes, 0xDD, p.jc_path.size);

    uint8_t buf[1024] = {};
    pb_ostream_t stream = pb_ostream_from_buffer(buf, sizeof(buf));
    TEST_ASSERT_TRUE(pb_encode(&stream, &meshtastic_MeshPacket_msg, &p));
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(meshtastic_MeshPacket_size, stream.bytes_written);
}

static void test_max_payload_and_path_fit_fromradio(void)
{
    meshtastic_MeshPacket p = meshtastic_MeshPacket_init_zero;
    p.from = 0xFFFFFFFF;
    p.to = 0xFFFFFFFF;
    p.id = 0xFFFFFFFF;
    p.has_rx_rssi = true;
    p.rx_rssi = -1;
    p.which_payload_variant = meshtastic_MeshPacket_decoded_tag;
    p.decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;
    p.decoded.payload.size = sizeof(p.decoded.payload.bytes);
    memset(p.decoded.payload.bytes, 'x', p.decoded.payload.size);
    p.decoded.xeddsa_signature.size = sizeof(p.decoded.xeddsa_signature.bytes);
    memset(p.decoded.xeddsa_signature.bytes, 's', p.decoded.xeddsa_signature.size);
    p.public_key.size = sizeof(p.public_key.bytes);
    memset(p.public_key.bytes, 'k', p.public_key.size);
    p.jc_flags = 0xFFFF;
    p.jc_relay_node = 0xFFFF;
    p.jc_path.size = sizeof(p.jc_path.bytes);
    memset(p.jc_path.bytes, 'p', p.jc_path.size);

    meshtastic_FromRadio in = meshtastic_FromRadio_init_zero;
    in.which_payload_variant = meshtastic_FromRadio_packet_tag;
    in.packet = p;

    uint8_t buf[meshtastic_FromRadio_size] = {};
    const size_t len = pb_encode_to_bytes(buf, sizeof(buf), &meshtastic_FromRadio_msg, &in);
    TEST_ASSERT_GREATER_THAN(0, len);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(512, len);
}

void setup()
{
    UNITY_BEGIN();
    RUN_TEST(test_flags_layout);
    RUN_TEST(test_flags_truncated_bit);
    RUN_TEST(test_flags_region_none);
    RUN_TEST(test_flags_zero_when_not_jencore);
    RUN_TEST(test_path_tail_keeps_last_hashes);
    RUN_TEST(test_path_tail_short_path_not_truncated);
    RUN_TEST(test_path_tail_one_byte_hashes);
    RUN_TEST(test_path_tail_invalid_inputs);
    RUN_TEST(test_fromradio_roundtrip);
    RUN_TEST(test_declared_meshpacket_size_bounds_all_fields);
    RUN_TEST(test_max_payload_and_path_fit_fromradio);
    exit(UNITY_END());
}

void loop() {}
