// Unit tests for JenWAN's CayenneLPP encoder and config defaults.
//
// What is pinned:
//   - the exact CayenneLPP wire bytes for the telemetry types JenWAN uplinks
//     (temperature, humidity, barometer, voltage, current, percentage, GPS),
//     including the 0.0001-degree / 0.01 m / 0.01 V scaling;
//   - that a record which does not fit the 51-byte LoRaWAN payload leaves the
//     buffer untouched;
//   - the all-off runtime defaults and region name round-trip.
//
// The regression guarded: an earlier draft wrote 1e-7-degree latitude straight
// into the 3-byte field and used volts instead of centivolts, which TTN's
// CayenneLPP decoder rendered as a position near the equator and 0 V. The
// byte-exact assertions below fail on either mistake.

#define NEXUS_ENABLE_JENWAN 1

#include <Arduino.h>
#include <cstring>
#include <unity.h>

#include "../../src/mesh/JenWANPayload.cpp"
#include "../../src/mesh/JenWANVault.cpp"

void setUp(void) {}
void tearDown(void) {}

static void test_temperature_scaling(void)
{
    uint8_t out[8] = {};
    const size_t n = jenwan::lppAddTemperature(out, sizeof(out), 0, 2, 22.5f);
    TEST_ASSERT_EQUAL_UINT32(4, n);
    TEST_ASSERT_EQUAL_UINT8(2, out[0]);
    TEST_ASSERT_EQUAL_UINT8(0x67, out[1]);
    TEST_ASSERT_EQUAL_UINT8(0x00, out[2]);
    TEST_ASSERT_EQUAL_UINT8(0xE1, out[3]); // 225 * 0.1 C
}

static void test_humidity_scaling(void)
{
    uint8_t out[8] = {};
    const size_t n = jenwan::lppAddHumidity(out, sizeof(out), 0, 3, 50.0f);
    TEST_ASSERT_EQUAL_UINT32(3, n);
    TEST_ASSERT_EQUAL_UINT8(0x68, out[1]);
    TEST_ASSERT_EQUAL_UINT8(100, out[2]); // 0.5 % steps
}

static void test_barometer_scaling(void)
{
    uint8_t out[8] = {};
    const size_t n = jenwan::lppAddBarometer(out, sizeof(out), 0, 3, 1013.25f);
    TEST_ASSERT_EQUAL_UINT32(4, n);
    TEST_ASSERT_EQUAL_UINT8(0x73, out[1]);
    TEST_ASSERT_EQUAL_UINT8(0x27, out[2]);
    TEST_ASSERT_EQUAL_UINT8(0x95, out[3]); // 10133 * 0.1 hPa (10132.5 rounds up)
}

static void test_voltage_and_current_scaling(void)
{
    uint8_t out[8] = {};
    size_t n = jenwan::lppAddVoltage(out, sizeof(out), 0, 2, 4.2f);
    TEST_ASSERT_EQUAL_UINT32(4, n);
    TEST_ASSERT_EQUAL_UINT8(0x74, out[1]);
    TEST_ASSERT_EQUAL_UINT8(0x01, out[2]);
    TEST_ASSERT_EQUAL_UINT8(0xA4, out[3]); // 420 * 0.01 V

    memset(out, 0, sizeof(out));
    n = jenwan::lppAddCurrent(out, sizeof(out), 0, 4, 0.125f);
    TEST_ASSERT_EQUAL_UINT32(4, n);
    TEST_ASSERT_EQUAL_UINT8(0x65, out[1]);
    TEST_ASSERT_EQUAL_UINT8(0x00, out[2]);
    TEST_ASSERT_EQUAL_UINT8(0x7D, out[3]); // 125 * 0.001 A
}

static void test_percentage(void)
{
    uint8_t out[8] = {};
    const size_t n = jenwan::lppAddPercentage(out, sizeof(out), 0, 2, 85);
    TEST_ASSERT_EQUAL_UINT32(3, n);
    TEST_ASSERT_EQUAL_UINT8(0x66, out[1]);
    TEST_ASSERT_EQUAL_UINT8(85, out[2]);
}

static void test_gps_scaling(void)
{
    uint8_t out[16] = {};
    // 52.37365 N, 4.88650 E, 12 m, 3 m accuracy (3000 mm).
    const size_t n = jenwan::lppAddGps(out, sizeof(out), 0, 1, 523736500, 48865000, 12, 3000);
    TEST_ASSERT_EQUAL_UINT32(11, n);
    TEST_ASSERT_EQUAL_UINT8(1, out[0]);
    TEST_ASSERT_EQUAL_UINT8(0x88, out[1]);
    TEST_ASSERT_EQUAL_UINT8(0x07, out[2]); // 523736 * 0.0001 deg
    TEST_ASSERT_EQUAL_UINT8(0xFD, out[3]);
    TEST_ASSERT_EQUAL_UINT8(0xD8, out[4]);
    TEST_ASSERT_EQUAL_UINT8(0x00, out[5]); // 48865 * 0.0001 deg
    TEST_ASSERT_EQUAL_UINT8(0xBE, out[6]);
    TEST_ASSERT_EQUAL_UINT8(0xE1, out[7]);
    TEST_ASSERT_EQUAL_UINT8(0x04, out[8]); // 1200 * 0.01 m
    TEST_ASSERT_EQUAL_UINT8(0xB0, out[9]);
    TEST_ASSERT_EQUAL_UINT8(0xFF, out[10]); // 300 cm clamped to one byte
}

static void test_record_does_not_split(void)
{
    uint8_t out[4] = {};
    memset(out, 0xAA, sizeof(out));
    const size_t n = jenwan::lppAddGps(out, sizeof(out), 0, 1, 523736500, 48865000, 12, 3000);
    TEST_ASSERT_EQUAL_UINT32(0, n);
    for (uint8_t b : out)
        TEST_ASSERT_EQUAL_UINT8(0xAA, b);
}

static void test_defaults_are_off(void)
{
    jenwan::Config c;
    jenwan::defaults(c);
    TEST_ASSERT_EQUAL_UINT8(0, c.enabled);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)jenwan::Band::UNSET, c.band);
    TEST_ASSERT_EQUAL_UINT8(0, c.sources);
    TEST_ASSERT_EQUAL_UINT8(1, c.port);
    TEST_ASSERT_TRUE(c.flags & (1u << jenwan::CFG_DUTY));
    TEST_ASSERT_TRUE(c.flags & (1u << jenwan::CFG_ADR));
    TEST_ASSERT_FALSE(c.flags & (1u << jenwan::CFG_CONFIRMED));
}

static void test_region_round_trip(void)
{
    jenwan::Band b = jenwan::Band::UNSET;
    TEST_ASSERT_TRUE(jenwan::bandFromName("EU868", b));
    TEST_ASSERT_EQUAL_UINT8((uint8_t)jenwan::Band::EU868, (uint8_t)b);
    TEST_ASSERT_EQUAL_STRING("EU868", jenwan::bandName(b));
    TEST_ASSERT_FALSE(jenwan::bandFromName("NOPE", b));
}

static void test_empty_sources(void)
{
    uint8_t out[jenwan::LPP_MAX_PAYLOAD] = {};
    TEST_ASSERT_EQUAL_UINT32(0, jenwan::encodeSources(0, out, sizeof(out)));
}

void setup()
{
    UNITY_BEGIN();
    RUN_TEST(test_temperature_scaling);
    RUN_TEST(test_humidity_scaling);
    RUN_TEST(test_barometer_scaling);
    RUN_TEST(test_voltage_and_current_scaling);
    RUN_TEST(test_percentage);
    RUN_TEST(test_gps_scaling);
    RUN_TEST(test_record_does_not_split);
    RUN_TEST(test_defaults_are_off);
    RUN_TEST(test_region_round_trip);
    RUN_TEST(test_empty_sources);
    UNITY_END();
}

void loop() {}
