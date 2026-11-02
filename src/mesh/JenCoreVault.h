// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) 2026 Jennifer Sanchez
//
// Licensed under the GNU Affero General Public License v3.
// See LICENSE-AGPL for the full license text.
//

#pragma once

#include "configuration.h"

#if NEXUS_ENABLE_JENCORE

#include "JenCorePolicy.h"
#include <stddef.h>
#include <stdint.h>

/**
 * JenCore-only persistent state, kept in /jencore.bin so it can grow without bumping
 * the Nexus vault. The older JenCore settings stay in the Nexus vault (v71-v75); this
 * vault is for JenCore policy that has no Meshtastic counterpart: the ADVERT name
 * override, trace alerts, trace/ping/discovery blocking and relay burner aliases.
 */
namespace jenCoreVault
{
constexpr char MAGIC[4] = {'J', 'C', 'o', 'r'};
constexpr uint32_t VERSION = 2;

struct __attribute__((packed)) Vault {
    char magic[4];                         // MAGIC
    uint32_t version;                      // VERSION
    nexus::jen_core_policy::Policy policy; // v2: JenCore-only policy
    uint32_t crc32;                        // CRC32 over everything before this field
};

static_assert(sizeof(Vault) == 4 + 4 + sizeof(nexus::jen_core_policy::Policy) + 4, "JenCoreVault layout changed; bump VERSION");
static_assert(offsetof(Vault, crc32) == sizeof(Vault) - sizeof(uint32_t), "CRC must stay last");

/// Process-wide policy. Zero-initialised defaults until loadPolicy() runs.
nexus::jen_core_policy::Policy &policy();

/// Load /jencore.bin into policy(). A missing file, bad magic/version/size, or CRC
/// mismatch returns false, leaves policy() at defaults, and writes the reason to
/// `*reason` when non-null.
bool loadPolicy(const char **reason = nullptr);

/// Fill in magic, version and CRC, then atomically replace /jencore.bin.
bool savePolicy();

/// Lower-level Policy codec: load from /jencore.bin (v1 files are accepted and
/// read as defaults) or write one. `out` is zeroed first on failure.
bool load(nexus::jen_core_policy::Policy &out, const char **reason = nullptr);
bool save(const nexus::jen_core_policy::Policy &in);
} // namespace jenCoreVault

#endif
