// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) 2026 Jennifer Sanchez
//
// Licensed under the GNU Affero General Public License v3.
// See LICENSE-AGPL for the full license text.
//

#pragma once

#include "configuration.h"

#if NEXUS_ENABLE_JENWAN

#include <stddef.h>
#include <stdint.h>

/**
 * JenWAN-only persistent state, kept in /jenwan.bin so it can grow without
 * bumping the Nexus vault. Holds the LoRaWAN provisioning (OTAA/ABP keys),
 * the user-facing schedule, and the opaque RadioLib nonces/session buffers.
 */
namespace jenwan
{
constexpr char MAGIC[4] = {'J', 'W', 'a', 'n'};
constexpr uint32_t VERSION = 1;

constexpr size_t EUI_LEN = 8;
constexpr size_t KEY_LEN = 16;
constexpr size_t NONCES_MAX = 32;
constexpr size_t SESSION_MAX = 320;
constexpr size_t NUM_SOURCES = 4;

enum class Band : uint8_t {
    UNSET = 0,
    EU868,
    US915,
    EU433,
    AU915,
    CN470,
    AS923,
    KR920,
    IN865,
};

enum class JoinMode : uint8_t { NONE = 0, OTAA, ABP };

enum class DownlinkMode : uint8_t { OFF = 0, STORE, TEXT };

enum class PayloadFormat : uint8_t { CAYENNE = 0 };

enum Source : uint8_t { SRC_DEVICE = 0, SRC_POWER = 1, SRC_ENV = 2, SRC_POS = 3 };

constexpr uint8_t sourceBit(uint8_t source)
{
    return (uint8_t)(1u << source);
}

enum ConfigFlag : uint8_t {
    CFG_CONFIRMED = 0, // confirmed uplinks (default off)
    CFG_DUTY = 1,      // enforce duty cycle (default on)
    CFG_ADR = 2,       // network-controlled data rate (default on)
    CFG_POS = 3,       // position part of SRC_POS (explicit privacy switch)
};

struct __attribute__((packed)) Config {
    uint8_t enabled;       // 0/1
    uint8_t band;          // Band
    uint8_t sub_band;      // 0 = band default, 1..8 for US915/AU915
    uint8_t join_mode;     // JoinMode
    uint8_t flags;         // ConfigFlag bits
    uint8_t sources;       // Source bits
    uint8_t format;        // PayloadFormat
    uint8_t downlink;      // DownlinkMode
    uint8_t downlink_chan; // Meshtastic channel for TEXT downlinks
    uint8_t port;          // uplink fPort (1..223)
    uint8_t reserved0[2];
    uint32_t intervals[NUM_SOURCES]; // seconds per source; 0 = off

    uint8_t join_eui[EUI_LEN];
    uint8_t dev_eui[EUI_LEN];
    uint8_t app_key[KEY_LEN];
    uint8_t nwk_key[KEY_LEN];
    uint8_t dev_addr[4];
    uint8_t nwk_s_key[KEY_LEN];
    uint8_t app_s_key[KEY_LEN];
    uint8_t key_flags; // bit0 OTAA keys present, bit1 ABP keys present
    uint8_t reserved1[3];

    uint16_t nonces_len;
    uint16_t session_len;
    uint8_t nonces[NONCES_MAX];
    uint8_t session[SESSION_MAX];
};

struct __attribute__((packed)) Vault {
    char magic[4];    // MAGIC
    uint32_t version; // VERSION
    Config config;
    uint32_t crc32; // CRC32 over everything before this field
};

static_assert(offsetof(Vault, crc32) == sizeof(Vault) - sizeof(uint32_t), "CRC must stay last");

/// Zero-initialise a Config and apply the runtime defaults (everything off).
void defaults(Config &out);

/// Load /jenwan.bin. A missing file, bad magic/version/size, or CRC mismatch
/// returns false, leaves `out` at defaults, and writes the reason to `*reason`.
bool load(Config &out, const char **reason = nullptr);

/// Fill in magic, version and CRC, then atomically replace /jenwan.bin.
bool save(const Config &in);

/// Region helpers used by the command parser.
const char *bandName(Band band);
bool bandFromName(const char *name, Band &out);
} // namespace jenwan

#endif // NEXUS_ENABLE_JENWAN
