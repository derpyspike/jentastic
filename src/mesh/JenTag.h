// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) 2026 Jennifer Sanchez
//
// JenTag — OpenHaystack-compatible Find My BLE advertisement.
//
// The advertisement layout follows OpenHaystack's ESP32 firmware:
// [0] length (0x1e), [1] manufacturer-specific (0xff), [2..3] Apple company ID,
// [4] offline-finding type (0x12), [5] length (0x19), [6] status,
// [7..28] the LAST 22 bytes of the 28-byte P-224 public key,
// [29] the top two bits of the key's first byte, [30] hint.
//
// The public key is not secret; the corresponding private key stays with the
// owner (OpenHaystack app). Keep this header free of platform dependencies so
// the payload can be unit-tested natively.

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace nexus::jentag
{

constexpr size_t KEY_LEN = 28;
constexpr size_t ADVERT_LEN = 31;

constexpr uint8_t ADVERT_LENGTH = 0x1e;
constexpr uint8_t ADVERT_TYPE = 0xff;
constexpr uint16_t APPLE_COMPANY_ID = 0x004c;
constexpr uint8_t OFFLINE_FINDING_TYPE = 0x12;
constexpr uint8_t OFFLINE_FINDING_LENGTH = 0x19;

/// Build the exact 31-byte legacy advertisement OpenHaystack's ESP32 firmware
/// emits. out[1] is the AD type, so this is a raw advertising payload (the
/// length byte is included), not a manufacturer-data-only blob.
inline void buildAdvertisement(const uint8_t key[KEY_LEN], uint8_t out[ADVERT_LEN])
{
    out[0] = ADVERT_LENGTH;
    out[1] = ADVERT_TYPE;
    out[2] = (uint8_t)(APPLE_COMPANY_ID & 0xff);
    out[3] = (uint8_t)(APPLE_COMPANY_ID >> 8);
    out[4] = OFFLINE_FINDING_TYPE;
    out[5] = OFFLINE_FINDING_LENGTH;
    out[6] = 0x00; // status
    memcpy(&out[7], &key[6], 22);
    out[29] = (uint8_t)(key[0] >> 6);
    out[30] = 0x00; // hint
}

/// BLE random static address derived from the public key, as OpenHaystack does.
inline void deriveAddress(const uint8_t key[KEY_LEN], uint8_t addr[6])
{
    addr[0] = (uint8_t)(key[0] | 0xC0);
    memcpy(&addr[1], &key[1], 5);
}

/// True when at least one key byte is non-zero. The all-zero buffer is the
/// "no key configured" sentinel; the key itself may legitimately start with 0.
inline bool keyIsSet(const uint8_t key[KEY_LEN])
{
    for (size_t i = 0; i < KEY_LEN; i++) {
        if (key[i] != 0)
            return true;
    }
    return false;
}

} // namespace nexus::jentag
