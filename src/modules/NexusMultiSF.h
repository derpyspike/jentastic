// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) 2026 Jennifer Sanchez
//
// Nexus multi-SF tier configuration (LR2021). This is deliberately independent
// of NexusSplitConfig: SPLIT is the dual-preset/frequency system for every
// radio, while this table describes same-carrier protocol tiers (Madrid:
// SF5 RNode / SF6 Meshtastic / SF7 Meshtastic / SF8 MeshCore).

#pragma once

#include <cstddef>
#include <cstdint>
#include <strings.h>

namespace meshtastic::nexus {

enum TierProtocol : uint8_t {
    TIER_NONE = 0,
    TIER_MESHTASTIC = 1,
    TIER_RNODE = 2,
    TIER_MESHCORE = 3,
};

struct __attribute__((packed)) TierConfig {
    uint8_t enabled;
    uint8_t sf;       // 5..12; must be higher than the main SF
    uint8_t syncword; // 0x2B Meshtastic, 0x12 RNode/MeshCore, or operator value
    uint8_t protocol; // TierProtocol
    int8_t tx_power;  // 0x7F = use the radio default
    uint8_t flags;    // reserved
};

struct __attribute__((packed)) MultiSFConfig {
    uint8_t enabled;
    uint8_t main_protocol;    // protocol owned by the main detector
    uint8_t main_syncword;    // syncword for the main detector (e.g. 0x12 RNode)
    int8_t main_tx_power;     // 0x7F = use config.lora.tx_power
    uint8_t side_count;       // 0..3; side[0..side_count-1] are active
    uint8_t meshtastic_tx_slot; // side[] index used for client/relay TX, 0xFF = first Meshtastic
    uint8_t reserved[1];
    TierConfig side[3];
};

static constexpr uint8_t MULTISF_MAX_SIDE = 3;
static constexpr int8_t MULTISF_DEFAULT_POWER = 0x7F;
static constexpr uint8_t MULTISF_NO_TX_SLOT = 0xFF;

inline uint8_t tierDetectionFactor(uint8_t sf)
{
    if (sf < 5 || sf > 12)
        return 0;
    return static_cast<uint8_t>(10 + (((sf - 5) >> 1) << 1));
}

inline const char *tierProtocolName(uint8_t protocol)
{
    switch (protocol) {
    case TIER_MESHTASTIC:
        return "MESHTASTIC";
    case TIER_RNODE:
        return "RNODE";
    case TIER_MESHCORE:
        return "MESHCORE";
    default:
        return "NONE";
    }
}

inline bool parseTierProtocol(const char *value, uint8_t &protocol)
{
    if (!value)
        return false;
    if (!strcasecmp(value, "MESHTASTIC") || !strcasecmp(value, "MESH") || !strcasecmp(value, "MT")) {
        protocol = TIER_MESHTASTIC;
        return true;
    }
    if (!strcasecmp(value, "RNODE") || !strcasecmp(value, "RETICULUM") || !strcasecmp(value, "RNS")) {
        protocol = TIER_RNODE;
        return true;
    }
    if (!strcasecmp(value, "MESHCORE") || !strcasecmp(value, "JENCORE") || !strcasecmp(value, "MC")) {
        protocol = TIER_MESHCORE;
        return true;
    }
    return false;
}

/// Validate a multi-SF table against the LR2021 Rx rules and the current main
/// modulation. `mainSf` is the main detector's SF (config.lora.spread_factor)
/// and `bwKHz` is the shared bandwidth. Pure; no radio access.
inline bool validateMultiSF(const MultiSFConfig &cfg, uint8_t mainSf, float bwKHz, const char **reason)
{
    auto reject = [reason](const char *why) {
        if (reason)
            *reason = why;
        return false;
    };

    if (!cfg.enabled) {
        if (reason)
            *reason = "OK (disabled)";
        return true;
    }
    if (mainSf < 5 || mainSf > 12)
        return reject("main SF out of range [5..12]");
    if (cfg.main_protocol < TIER_MESHTASTIC || cfg.main_protocol > TIER_MESHCORE)
        return reject("main protocol invalid");
    if (cfg.main_syncword == 0)
        return reject("main syncword must be non-zero");
    if (cfg.side_count > MULTISF_MAX_SIDE)
        return reject("more than 3 side detectors");
    if (cfg.side_count >= 3 && bwKHz > 500.0f)
        return reject("BW > 500 kHz allows only 2 side detectors");
    if (cfg.side_count >= 3 && mainSf >= 10)
        return reject("main SF >= 10 allows only 2 side detectors");
    if (cfg.meshtastic_tx_slot != MULTISF_NO_TX_SLOT && cfg.meshtastic_tx_slot >= cfg.side_count)
        return reject("Meshtastic TX slot outside the side table");

    bool used[13] = {false};
    used[mainSf] = true;
    uint32_t detectionSum = tierDetectionFactor(mainSf);
    uint8_t largestSf = mainSf;
    bool hasMeshtastic = cfg.main_protocol == TIER_MESHTASTIC;

    for (uint8_t i = 0; i < cfg.side_count; ++i) {
        const TierConfig &tier = cfg.side[i];
        if (!tier.enabled)
            return reject("active side slot is not enabled");
        if (tier.sf < 5 || tier.sf > 12)
            return reject("side SF out of range [5..12]");
        if (tier.sf <= mainSf)
            return reject("side SF must be higher than the main SF");
        if (used[tier.sf])
            return reject("duplicate SF in the tier table");
        if (tier.protocol < TIER_MESHTASTIC || tier.protocol > TIER_MESHCORE)
            return reject("side protocol invalid");
        if (tier.syncword == 0)
            return reject("side syncword must be non-zero");
        used[tier.sf] = true;
        if (tier.sf > largestSf)
            largestSf = tier.sf;
        detectionSum += tierDetectionFactor(tier.sf);
        if (tier.protocol == TIER_MESHTASTIC)
            hasMeshtastic = true;
    }

    if (largestSf - mainSf > 4)
        return reject("SF spread must be <= 4");
    if (static_cast<float>(detectionSum) * bwKHz >= 32000.0f)
        return reject("detection factor sum x BW must be < 32e6");
    if (hasMeshtastic && cfg.meshtastic_tx_slot == MULTISF_NO_TX_SLOT) {
        // 0xFF means "first Meshtastic tier"; it is valid as long as one exists.
    } else if (cfg.meshtastic_tx_slot != MULTISF_NO_TX_SLOT &&
               cfg.side[cfg.meshtastic_tx_slot].protocol != TIER_MESHTASTIC) {
        return reject("Meshtastic TX slot is not a Meshtastic tier");
    }

    if (reason)
        *reason = "OK";
    return true;
}

/// Madrid: SF5 RNode main, SF6/7 Meshtastic, SF8 MeshCore. The caller still has
/// to put config.lora on SF5 / BW62.5 / CR5 / 869.618 MHz.
inline void makeMadridPreset(MultiSFConfig &cfg)
{
    cfg = MultiSFConfig{};
    cfg.enabled = 1;
    cfg.main_protocol = TIER_RNODE;
    cfg.main_syncword = 0x12;
    cfg.main_tx_power = MULTISF_DEFAULT_POWER;
    cfg.side_count = 3;
    cfg.meshtastic_tx_slot = 0;
    cfg.side[0] = TierConfig{1, 6, 0x2B, TIER_MESHTASTIC, MULTISF_DEFAULT_POWER, 0};
    cfg.side[1] = TierConfig{1, 7, 0x2B, TIER_MESHTASTIC, MULTISF_DEFAULT_POWER, 0};
    cfg.side[2] = TierConfig{1, 8, 0x12, TIER_MESHCORE, MULTISF_DEFAULT_POWER, 0};
}

inline bool multiSFHasProtocol(const MultiSFConfig &cfg, uint8_t protocol)
{
    if (!cfg.enabled)
        return false;
    if (cfg.main_protocol == protocol)
        return true;
    for (uint8_t i = 0; i < cfg.side_count; ++i)
        if (cfg.side[i].enabled && cfg.side[i].protocol == protocol)
            return true;
    return false;
}

} // namespace meshtastic::nexus
