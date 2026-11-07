// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) 2026 Jennifer Sanchez
//
// Licensed under the GNU Affero General Public License v3.
// See LICENSE-AGPL for the full license text.
//

#include "JenWANPayload.h"

#if NEXUS_ENABLE_JENWAN

#include "NodeDB.h"
#include "mesh/generated/meshtastic/telemetry.pb.h"
#include <cstring>

// Defined in NexusModule.cpp; forward-declared here so the payload encoder does
// not pull in the whole NexusModule header.
namespace meshtastic
{
bool getNexusDeviceTelemetry(meshtastic_Telemetry *m);
bool getNexusPowerTelemetry(meshtastic_Telemetry *m);
bool getNexusEnvironmentTelemetry(meshtastic_Telemetry *m);
} // namespace meshtastic

namespace jenwan
{
namespace
{
constexpr uint8_t LPP_GPS = 0x88;
constexpr uint8_t LPP_TEMPERATURE = 0x67;
constexpr uint8_t LPP_HUMIDITY = 0x68;
constexpr uint8_t LPP_BAROMETER = 0x73;
constexpr uint8_t LPP_VOLTAGE = 0x74;
constexpr uint8_t LPP_CURRENT = 0x65;
constexpr uint8_t LPP_PERCENTAGE = 0x66;

constexpr uint8_t CH_GPS = 1;
constexpr uint8_t CH_DEVICE = 2;
constexpr uint8_t CH_ENVIRONMENT = 3;
constexpr uint8_t CH_POWER = 4;

int32_t roundTo(float v)
{
    return (int32_t)(v >= 0 ? v + 0.5f : v - 0.5f);
}

size_t addRecord(uint8_t *out, size_t cap, size_t pos, uint8_t channel, uint8_t type, const uint8_t *data, size_t len)
{
    if (pos + 2 + len > cap)
        return pos;
    out[pos++] = channel;
    out[pos++] = type;
    memcpy(&out[pos], data, len);
    return pos + len;
}

void putU16(uint8_t *d, uint16_t v)
{
    d[0] = (uint8_t)(v >> 8);
    d[1] = (uint8_t)v;
}

void put24(uint8_t *d, int32_t v)
{
    d[0] = (uint8_t)((v >> 16) & 0xFF);
    d[1] = (uint8_t)((v >> 8) & 0xFF);
    d[2] = (uint8_t)(v & 0xFF);
}
} // namespace

size_t lppAddGps(uint8_t *out, size_t cap, size_t pos, uint8_t channel, int32_t lat_i, int32_t lon_i, int32_t altitude_m,
                 uint32_t accuracy_mm)
{
    uint8_t d[9];
    // Meshtastic stores 1e-7 degrees; CayenneLPP uses 0.0001 degrees.
    put24(&d[0], lat_i / 1000);
    put24(&d[3], lon_i / 1000);
    int32_t alt = altitude_m * 100; // 0.01 m
    if (alt > 32767)
        alt = 32767;
    if (alt < -32768)
        alt = -32768;
    putU16(&d[6], (uint16_t)(int16_t)alt);
    uint32_t acc = accuracy_mm / 10; // 0.01 m steps
    d[8] = (uint8_t)(acc > 255 ? 255 : acc);
    return addRecord(out, cap, pos, channel, LPP_GPS, d, sizeof(d));
}

size_t lppAddTemperature(uint8_t *out, size_t cap, size_t pos, uint8_t channel, float celsius)
{
    uint8_t d[2];
    putU16(d, (uint16_t)(int16_t)roundTo(celsius * 10.0f));
    return addRecord(out, cap, pos, channel, LPP_TEMPERATURE, d, sizeof(d));
}

size_t lppAddHumidity(uint8_t *out, size_t cap, size_t pos, uint8_t channel, float rh)
{
    int32_t v = roundTo(rh * 2.0f);
    if (v < 0)
        v = 0;
    if (v > 255)
        v = 255;
    uint8_t d = (uint8_t)v;
    return addRecord(out, cap, pos, channel, LPP_HUMIDITY, &d, 1);
}

size_t lppAddBarometer(uint8_t *out, size_t cap, size_t pos, uint8_t channel, float hpa)
{
    uint8_t d[2];
    int32_t v = roundTo(hpa * 10.0f);
    if (v < 0)
        v = 0;
    if (v > 65535)
        v = 65535;
    putU16(d, (uint16_t)v);
    return addRecord(out, cap, pos, channel, LPP_BAROMETER, d, sizeof(d));
}

size_t lppAddVoltage(uint8_t *out, size_t cap, size_t pos, uint8_t channel, float volts)
{
    uint8_t d[2];
    int32_t v = roundTo(volts * 100.0f);
    if (v < 0)
        v = 0;
    if (v > 65535)
        v = 65535;
    putU16(d, (uint16_t)v);
    return addRecord(out, cap, pos, channel, LPP_VOLTAGE, d, sizeof(d));
}

size_t lppAddCurrent(uint8_t *out, size_t cap, size_t pos, uint8_t channel, float amps)
{
    uint8_t d[2];
    int32_t v = roundTo(amps * 1000.0f);
    if (v < 0)
        v = 0;
    if (v > 65535)
        v = 65535;
    putU16(d, (uint16_t)v);
    return addRecord(out, cap, pos, channel, LPP_CURRENT, d, sizeof(d));
}

size_t lppAddPercentage(uint8_t *out, size_t cap, size_t pos, uint8_t channel, uint8_t percent)
{
    return addRecord(out, cap, pos, channel, LPP_PERCENTAGE, &percent, 1);
}

size_t encodeSources(uint8_t sourceMask, uint8_t *out, size_t cap)
{
    if (!out || cap == 0)
        return 0;
    size_t pos = 0;

    if (sourceMask & sourceBit(SRC_POS)) {
        if (localPosition.has_latitude_i && localPosition.has_longitude_i) {
            int32_t alt = localPosition.has_altitude ? localPosition.altitude : 0;
            pos = lppAddGps(out, cap, pos, CH_GPS, localPosition.latitude_i, localPosition.longitude_i, alt,
                            localPosition.gps_accuracy);
        }
    }

    if (sourceMask & sourceBit(SRC_DEVICE)) {
        meshtastic_Telemetry t = meshtastic_Telemetry_init_zero;
        if (meshtastic::getNexusDeviceTelemetry(&t) && t.which_variant == meshtastic_Telemetry_device_metrics_tag) {
            const auto &d = t.variant.device_metrics;
            if (d.has_battery_level && d.battery_level <= 100)
                pos = lppAddPercentage(out, cap, pos, CH_DEVICE, (uint8_t)d.battery_level);
            if (d.has_voltage && d.voltage > 0)
                pos = lppAddVoltage(out, cap, pos, CH_DEVICE, d.voltage);
        }
    }

    if (sourceMask & sourceBit(SRC_ENV)) {
        meshtastic_Telemetry t = meshtastic_Telemetry_init_zero;
        if (meshtastic::getNexusEnvironmentTelemetry(&t) && t.which_variant == meshtastic_Telemetry_environment_metrics_tag) {
            const auto &e = t.variant.environment_metrics;
            if (e.has_temperature)
                pos = lppAddTemperature(out, cap, pos, CH_ENVIRONMENT, e.temperature);
            if (e.has_relative_humidity)
                pos = lppAddHumidity(out, cap, pos, CH_ENVIRONMENT, e.relative_humidity);
            if (e.has_barometric_pressure)
                pos = lppAddBarometer(out, cap, pos, CH_ENVIRONMENT, e.barometric_pressure);
        }
    }

    if (sourceMask & sourceBit(SRC_POWER)) {
        meshtastic_Telemetry t = meshtastic_Telemetry_init_zero;
        if (meshtastic::getNexusPowerTelemetry(&t) && t.which_variant == meshtastic_Telemetry_power_metrics_tag) {
            const auto &p = t.variant.power_metrics;
            if (p.has_ch1_voltage)
                pos = lppAddVoltage(out, cap, pos, CH_POWER, p.ch1_voltage);
            if (p.has_ch1_current)
                pos = lppAddCurrent(out, cap, pos, CH_POWER, p.ch1_current);
        }
    }

    return pos;
}
} // namespace jenwan

#endif // NEXUS_ENABLE_JENWAN
