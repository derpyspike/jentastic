// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) 2026 Jennifer Sanchez
//
// Licensed under the GNU Affero General Public License v3.
// See LICENSE-AGPL for the full license text.
//
// Pure JenCore policy helpers. Kept free of radio/NodeDB/LittleFS dependencies
// so the rules can be exercised by the native unit tests.

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace nexus::jen_core_policy
{

/// Policy feature bits, persisted in JenCoreVault. Append-only: never reuse a bit.
enum : uint64_t {
    FEATURE_TRACE_ALERT = 1ull << 0,     // phone-only text on every heard TRACE
    FEATURE_NAME_SET = 1ull << 1,        // name[] overrides the ADVERT name
    FEATURE_BLOCK_TRACE = 1ull << 2,     // do not participate in multi-hop traces
    FEATURE_BLOCK_PING = 1ull << 3,      // do not participate in one-entry traces
    FEATURE_BLOCK_DISC = 1ull << 4,      // do not answer neighbour discovery
    FEATURE_BURNER_ON = 1ull << 5,       // stamp a burner alias instead of our real hash
    FEATURE_ROUTE_INTERVENE = 1ull << 6, // claim direct next hops carrying any burner alias
    FEATURE_FAKE_RANDOM = 1ull << 7,     // reserved
    FEATURE_FAKE_FIXED = 1ull << 8,      // reserved
};

constexpr uint64_t ALL_FEATURES_MASK = (FEATURE_FAKE_FIXED << 1) - 1;

constexpr size_t NAME_LEN = 32;    // ADVERT app-data is 32 bytes: type + 31 name bytes
constexpr size_t BURNER_SLOTS = 8; // up to eight aliases
// Three bytes are the derpycore-compatible alias width. The fourth is kept at
// zero on disk so a future build can widen aliases without a vault layout change.
constexpr size_t BURNER_LEN = 4;

struct __attribute__((packed)) Policy {
    uint64_t features = 0;
    char name[NAME_LEN] = {};
    uint8_t burner_count = 0;     // 0..BURNER_SLOTS
    uint8_t fake_hops = 0;        // reserved
    uint8_t fake_fixed_count = 0; // reserved
    uint8_t reserved = 0;
    uint8_t burners[BURNER_SLOTS][BURNER_LEN] = {};
    uint8_t fake_fixed[BURNER_SLOTS][BURNER_LEN] = {};
};

static_assert(sizeof(Policy) == 108, "Policy layout changed; bump the vault version");

inline bool has(const Policy &p, uint64_t bit)
{
    return (p.features & bit) != 0;
}

inline void set(Policy &p, uint64_t bit, bool on)
{
    if (on)
        p.features |= bit;
    else
        p.features &= ~bit;
}

/// Keep a loaded policy inside the fields this build understands.
inline void sanitize(Policy &p)
{
    p.features &= ALL_FEATURES_MASK;
    p.name[NAME_LEN - 1] = '\0';
    if (p.burner_count > BURNER_SLOTS)
        p.burner_count = BURNER_SLOTS;
    if (p.fake_fixed_count > BURNER_SLOTS)
        p.fake_fixed_count = BURNER_SLOTS;
    if (p.fake_hops > 8)
        p.fake_hops = 8;
}

/// TRACE flags bits 0-1 are the path-hash exponent, not the size itself.
inline uint8_t traceHashSize(uint8_t flags)
{
    return (uint8_t)(1u << (flags & 0x03));
}

/// Sizes this build can originate. 8-byte traces are received but not offered.
inline bool traceSizeSupported(uint8_t size)
{
    return size == 1 || size == 2 || size == 4;
}

/// 0xFF when unsupported, so callers can reject before memset'ing a frame.
inline uint8_t traceFlagsForSize(uint8_t size)
{
    switch (size) {
    case 1:
        return 0;
    case 2:
        return 1;
    case 4:
        return 2;
    default:
        return 0xFF;
    }
}

/// One-entry TRACE is the zero-hop "ping" shape.
inline bool blocksTrace(const Policy &p, uint8_t entryCount)
{
    return entryCount <= 1 ? has(p, FEATURE_BLOCK_PING) : has(p, FEATURE_BLOCK_TRACE);
}

inline void burnerEntry(const Policy &p, uint8_t index, uint8_t *out, uint8_t size)
{
    const uint8_t *src = p.burners[index % BURNER_SLOTS];
    for (uint8_t i = 0; i < size && i < BURNER_LEN; i++)
        out[i] = src[i];
    for (uint8_t i = BURNER_LEN; i < size; i++)
        out[i] = 0;
}

/// An entry matches an alias when it is configured, independent of whether we
/// are currently stamping aliases: route claim remains useful even while off.
inline bool isBurnerEntry(const Policy &p, const uint8_t *entry, uint8_t size)
{
    if (!entry || p.burner_count == 0 || size == 0 || size > BURNER_LEN)
        return false;
    for (uint8_t i = 0; i < p.burner_count && i < BURNER_SLOTS; i++)
        if (memcmp(entry, p.burners[i], size) == 0)
            return true;
    return false;
}

inline void stampEntry(const Policy &p, const uint8_t *identityPub, uint8_t size, uint8_t index, uint8_t *out)
{
    if (has(p, FEATURE_BURNER_ON) && p.burner_count > 0)
        burnerEntry(p, index, out, size);
    else
        memcpy(out, identityPub, size);
}

/// Our real public-key prefix always matches; burner aliases only with ROUTE on.
inline bool matchesNextHop(const Policy &p, const uint8_t *entry, uint8_t size, const uint8_t *identityPub)
{
    if (!entry || !identityPub || size == 0 || size > 32)
        return false;
    if (memcmp(entry, identityPub, size) == 0)
        return true;
    return has(p, FEATURE_ROUTE_INTERVENE) && isBurnerEntry(p, entry, size);
}

inline const char *advertName(const Policy &p, const char *fallback)
{
    return (has(p, FEATURE_NAME_SET) && p.name[0]) ? p.name : fallback;
}

/// Parsed TRACE header. `requestedHops` is the hash list in the payload;
/// `completedHops` is the SNR byte count accumulated in the packet path.
struct TraceSummary {
    uint32_t tag = 0;
    uint32_t auth = 0;
    uint8_t flags = 0;
    uint8_t hashSize = 0;
    uint8_t requestedHops = 0;
    uint8_t completedHops = 0;
    bool endpoint = false;
};

inline bool parseTraceSummary(const uint8_t *payload, size_t payloadLen, uint8_t pathCount, TraceSummary &out)
{
    constexpr size_t HEAD = 4 + 4 + 1; // tag | auth | flags
    if (!payload || payloadLen < HEAD)
        return false;

    const uint8_t hashSize = traceHashSize(payload[8]);
    const size_t listBytes = payloadLen - HEAD;
    if (hashSize == 0 || listBytes == 0 || (listBytes % hashSize) != 0)
        return false;

    const size_t requested = listBytes / hashSize;
    if (requested > 255)
        return false;

    memcpy(&out.tag, payload, 4);
    memcpy(&out.auth, payload + 4, 4);
    out.flags = payload[8];
    out.hashSize = hashSize;
    out.requestedHops = (uint8_t)requested;
    out.completedHops = pathCount;
    out.endpoint = (size_t)pathCount * hashSize >= listBytes;
    return true;
}

} // namespace nexus::jen_core_policy
