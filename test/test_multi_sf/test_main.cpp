// Unit tests for the pure multi-SF tier validator in
// src/modules/NexusMultiSF.h - the table used by the LR2021 Madrid layout
// (main SF5 RNode, side SF6/7 Meshtastic, side SF8 MeshCore).
//
// What is pinned:
//   - validateMultiSF() accepts the Madrid preset at 62.5 kHz and rejects a
//     side SF that is not above the main SF, duplicate SFs, spread > 4, and a
//     detection-factor budget over 32e6 (including the main detector, which
//     RadioLib's own check omits).
//   - makeMadridPreset() produces exactly that table.
//   - the shared tier types: DOUBLESHARED carries RNode + MeshCore on one detector,
//     TRIPLESHARED carries Meshtastic too, and only the latter can be the Meshtastic
//     TX tier by itself.
//
// The regression guarded: the LR2021 datasheet requires the main detector to
// use the smallest SF and the total detection-factor sum (main + sides) times
// the shared bandwidth to stay below 32e6. Accepting a table that violates
// either leaves the chip deaf or silently single-SF.

#include "modules/NexusMultiSF.h"
#include <Arduino.h> // declares setup()/loop() with C linkage
#include <cstdlib>
#include <unity.h>

using namespace meshtastic::nexus;

void setUp(void) {}
void tearDown(void) {}

static void test_madrid_preset_valid_at_62k5(void)
{
    MultiSFConfig cfg;
    makeMadridPreset(cfg);
    const char *reason = nullptr;
    TEST_ASSERT_TRUE(validateMultiSF(cfg, 5, 62.5f, &reason));
    TEST_ASSERT_EQUAL_STRING("OK", reason);
    TEST_ASSERT_EQUAL_UINT8(3, cfg.side_count);
    TEST_ASSERT_EQUAL_UINT8(TIER_RNODE, cfg.main_protocol);
    TEST_ASSERT_EQUAL_HEX8(0x12, cfg.main_syncword);
    TEST_ASSERT_EQUAL_UINT8(6, cfg.side[0].sf);
    TEST_ASSERT_EQUAL_UINT8(TIER_MESHTASTIC, cfg.side[0].protocol);
    TEST_ASSERT_EQUAL_UINT8(8, cfg.side[2].sf);
    TEST_ASSERT_EQUAL_UINT8(TIER_MESHCORE, cfg.side[2].protocol);
    TEST_ASSERT_TRUE(multiSFHasProtocol(cfg, TIER_RNODE));
    TEST_ASSERT_TRUE(multiSFHasProtocol(cfg, TIER_MESHCORE));
}

static void test_rejects_side_not_above_main(void)
{
    MultiSFConfig cfg;
    makeMadridPreset(cfg);
    cfg.side[0].sf = 5;
    const char *reason = nullptr;
    TEST_ASSERT_FALSE(validateMultiSF(cfg, 5, 62.5f, &reason));
    TEST_ASSERT_NOT_NULL(reason);
}

static void test_rejects_duplicate_sf(void)
{
    MultiSFConfig cfg;
    makeMadridPreset(cfg);
    cfg.side[1].sf = cfg.side[0].sf;
    const char *reason = nullptr;
    TEST_ASSERT_FALSE(validateMultiSF(cfg, 5, 62.5f, &reason));
}

static void test_rejects_spread_above_four(void)
{
    MultiSFConfig cfg;
    makeMadridPreset(cfg);
    cfg.side[2].sf = 10;
    const char *reason = nullptr;
    TEST_ASSERT_FALSE(validateMultiSF(cfg, 5, 62.5f, &reason));
}

static void test_rejects_detection_factor_over_budget(void)
{
    MultiSFConfig cfg = {};
    cfg.enabled = 1;
    cfg.main_protocol = TIER_MESHTASTIC;
    cfg.main_syncword = 0x2B;
    cfg.side_count = 2;
    cfg.side[0] = TierConfig{1, 10, 0x2B, TIER_MESHTASTIC, MULTISF_DEFAULT_POWER, 0};
    cfg.side[1] = TierConfig{1, 12, 0x2B, TIER_MESHTASTIC, MULTISF_DEFAULT_POWER, 0};
    const char *reason = nullptr;
    // 10 + 14 + 16 = 40 detection factors; 40 * 800 kHz = 32e6, not < 32e6.
    TEST_ASSERT_FALSE(validateMultiSF(cfg, 9, 800.0f, &reason));
    TEST_ASSERT_EQUAL_STRING("detection factor sum x BW must be < 32e6", reason);
}

static void test_rejects_three_sides_over_500k_bw(void)
{
    MultiSFConfig cfg;
    makeMadridPreset(cfg);
    const char *reason = nullptr;
    TEST_ASSERT_FALSE(validateMultiSF(cfg, 5, 800.0f, &reason));
    TEST_ASSERT_EQUAL_STRING("BW > 500 kHz allows only 2 side detectors", reason);
}

static void test_accepts_three_sides_at_500k_bw(void)
{
    MultiSFConfig cfg;
    makeMadridPreset(cfg);
    const char *reason = nullptr;
    // 44 * 500 kHz = 22e6, below the 32e6 budget.
    TEST_ASSERT_TRUE(validateMultiSF(cfg, 5, 500.0f, &reason));
}

static void test_protocol_names_and_parser(void)
{
    uint8_t protocol = TIER_NONE;
    TEST_ASSERT_TRUE(parseTierProtocol("MESHTASTIC", protocol));
    TEST_ASSERT_EQUAL_UINT8(TIER_MESHTASTIC, protocol);
    TEST_ASSERT_TRUE(parseTierProtocol("rnode", protocol));
    TEST_ASSERT_EQUAL_UINT8(TIER_RNODE, protocol);
    TEST_ASSERT_TRUE(parseTierProtocol("MeshCore", protocol));
    TEST_ASSERT_EQUAL_UINT8(TIER_MESHCORE, protocol);
    TEST_ASSERT_FALSE(parseTierProtocol("unknown", protocol));
    TEST_ASSERT_EQUAL_STRING("RNODE", tierProtocolName(TIER_RNODE));
}

static void test_shared_tier_protocols(void)
{
    TEST_ASSERT_TRUE(isSharedTier(TIER_DOUBLESHARED));
    TEST_ASSERT_TRUE(isSharedTier(TIER_TRIPLESHARED));
    TEST_ASSERT_FALSE(isSharedTier(TIER_RNODE));

    TEST_ASSERT_TRUE(tierCarriesProtocol(TIER_DOUBLESHARED, TIER_RNODE));
    TEST_ASSERT_TRUE(tierCarriesProtocol(TIER_DOUBLESHARED, TIER_MESHCORE));
    TEST_ASSERT_FALSE(tierCarriesProtocol(TIER_DOUBLESHARED, TIER_MESHTASTIC));

    TEST_ASSERT_TRUE(tierCarriesProtocol(TIER_TRIPLESHARED, TIER_MESHTASTIC));
    TEST_ASSERT_TRUE(tierCarriesProtocol(TIER_TRIPLESHARED, TIER_RNODE));
    TEST_ASSERT_TRUE(tierCarriesProtocol(TIER_TRIPLESHARED, TIER_MESHCORE));
    TEST_ASSERT_FALSE(tierCarriesProtocol(TIER_MESHTASTIC, TIER_RNODE));
}

static void test_parse_shared_names(void)
{
    uint8_t protocol = TIER_NONE;
    TEST_ASSERT_TRUE(parseTierProtocol("DOUBLESHARED", protocol));
    TEST_ASSERT_EQUAL_UINT8(TIER_DOUBLESHARED, protocol);
    TEST_ASSERT_TRUE(parseTierProtocol("ds", protocol));
    TEST_ASSERT_EQUAL_UINT8(TIER_DOUBLESHARED, protocol);
    TEST_ASSERT_TRUE(parseTierProtocol("TRIPLESHARED", protocol));
    TEST_ASSERT_EQUAL_UINT8(TIER_TRIPLESHARED, protocol);
    TEST_ASSERT_TRUE(parseTierProtocol("all", protocol));
    TEST_ASSERT_EQUAL_UINT8(TIER_TRIPLESHARED, protocol);
    TEST_ASSERT_EQUAL_STRING("DOUBLESHARED", tierProtocolName(TIER_DOUBLESHARED));
    TEST_ASSERT_EQUAL_STRING("TRIPLESHARED", tierProtocolName(TIER_TRIPLESHARED));
}

static void test_doubleshared_tier_valid(void)
{
    MultiSFConfig cfg = {};
    cfg.enabled = 1;
    cfg.main_protocol = TIER_DOUBLESHARED;
    cfg.main_syncword = 0x12;
    cfg.side_count = 1;
    cfg.meshtastic_tx_slot = 0;
    cfg.side[0] = TierConfig{1, 6, 0x2B, TIER_MESHTASTIC, MULTISF_DEFAULT_POWER, 0};
    const char *reason = nullptr;
    TEST_ASSERT_TRUE(validateMultiSF(cfg, 5, 62.5f, &reason));
    TEST_ASSERT_TRUE(multiSFHasProtocol(cfg, TIER_RNODE));
    TEST_ASSERT_TRUE(multiSFHasProtocol(cfg, TIER_MESHCORE));
    TEST_ASSERT_TRUE(multiSFHasProtocol(cfg, TIER_MESHTASTIC));
}

static void test_tripleshared_can_be_meshtastic_tx(void)
{
    MultiSFConfig cfg = {};
    cfg.enabled = 1;
    cfg.main_protocol = TIER_RNODE;
    cfg.main_syncword = 0x12;
    cfg.side_count = 1;
    cfg.meshtastic_tx_slot = 0;
    cfg.side[0] = TierConfig{1, 6, 0x12, TIER_TRIPLESHARED, MULTISF_DEFAULT_POWER, 0};
    const char *reason = nullptr;
    TEST_ASSERT_TRUE(validateMultiSF(cfg, 5, 62.5f, &reason));
    TEST_ASSERT_TRUE(multiSFHasProtocol(cfg, TIER_MESHTASTIC));
}

static void test_doubleshared_cannot_be_meshtastic_tx(void)
{
    MultiSFConfig cfg = {};
    cfg.enabled = 1;
    cfg.main_protocol = TIER_DOUBLESHARED;
    cfg.main_syncword = 0x12;
    cfg.side_count = 1;
    cfg.meshtastic_tx_slot = 0;
    cfg.side[0] = TierConfig{1, 6, 0x12, TIER_DOUBLESHARED, MULTISF_DEFAULT_POWER, 0};
    const char *reason = nullptr;
    TEST_ASSERT_FALSE(validateMultiSF(cfg, 5, 62.5f, &reason));
    TEST_ASSERT_EQUAL_STRING("Meshtastic TX slot is not a Meshtastic tier", reason);
}

void setup()
{
    UNITY_BEGIN();
    RUN_TEST(test_madrid_preset_valid_at_62k5);
    RUN_TEST(test_rejects_side_not_above_main);
    RUN_TEST(test_rejects_duplicate_sf);
    RUN_TEST(test_rejects_spread_above_four);
    RUN_TEST(test_rejects_detection_factor_over_budget);
    RUN_TEST(test_rejects_three_sides_over_500k_bw);
    RUN_TEST(test_accepts_three_sides_at_500k_bw);
    RUN_TEST(test_protocol_names_and_parser);
    RUN_TEST(test_shared_tier_protocols);
    RUN_TEST(test_parse_shared_names);
    RUN_TEST(test_doubleshared_tier_valid);
    RUN_TEST(test_tripleshared_can_be_meshtastic_tx);
    RUN_TEST(test_doubleshared_cannot_be_meshtastic_tx);
    exit(UNITY_END());
}

void loop() {}
