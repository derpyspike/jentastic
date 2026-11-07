// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) 2026 Jennifer Sanchez
//
// Licensed under the GNU Affero General Public License v3.
// See LICENSE-AGPL for the full license text.
//

#pragma once

#include "configuration.h"

#if NEXUS_ENABLE_JENWAN

#include "JenWANVault.h"
#include <stddef.h>
#include <stdint.h>

namespace jenwan
{
/// LoRaWAN application payload ceiling used by the encoder (EU868 SF12/125).
constexpr size_t LPP_MAX_PAYLOAD = 51;

// Pure CayenneLPP writers. Each returns the new write position, or `pos`
// unchanged when the record does not fit. They are unit-tested byte-exactly.
size_t lppAddGps(uint8_t *out, size_t cap, size_t pos, uint8_t channel, int32_t lat_i, int32_t lon_i, int32_t altitude_m,
                 uint32_t accuracy_mm);
size_t lppAddTemperature(uint8_t *out, size_t cap, size_t pos, uint8_t channel, float celsius);
size_t lppAddHumidity(uint8_t *out, size_t cap, size_t pos, uint8_t channel, float rh);
size_t lppAddBarometer(uint8_t *out, size_t cap, size_t pos, uint8_t channel, float hpa);
size_t lppAddVoltage(uint8_t *out, size_t cap, size_t pos, uint8_t channel, float volts);
size_t lppAddCurrent(uint8_t *out, size_t cap, size_t pos, uint8_t channel, float amps);
size_t lppAddPercentage(uint8_t *out, size_t cap, size_t pos, uint8_t channel, uint8_t percent);

/// Build one CayenneLPP payload from the sources in `sourceMask` (Source bits).
/// Returns 0 when no source produced data. Channel map: 1 GPS, 2 device,
/// 3 environment, 4 power.
size_t encodeSources(uint8_t sourceMask, uint8_t *out, size_t cap);
} // namespace jenwan

#endif // NEXUS_ENABLE_JENWAN
