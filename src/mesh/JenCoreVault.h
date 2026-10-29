// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) 2026 Jennifer Sanchez
//
// Licensed under the GNU Affero General Public License v3.
// See LICENSE-AGPL for the full license text.
//

#pragma once

#include "configuration.h"

#if NEXUS_ENABLE_JENCORE

#include <stddef.h>
#include <stdint.h>

/**
 * JenCore-only persistent state, kept in /jencore.bin so it can grow without bumping
 * the Nexus vault. The older JenCore settings stay in the Nexus vault (v71-v75); this
 * vault is for new JenCore-specific state.
 */
namespace jenCoreVault
{
constexpr char MAGIC[4] = {'J', 'C', 'o', 'r'};
constexpr uint32_t VERSION = 1;

struct __attribute__((packed)) Vault {
    char magic[4];          // MAGIC
    uint32_t version;       // VERSION
    uint64_t features_mask; // v1: JenCore-only feature bits
    uint32_t crc32;         // CRC32 over everything before this field
};

static_assert(sizeof(Vault) == 20, "JenCoreVault layout changed; bump VERSION");
static_assert(offsetof(Vault, crc32) == sizeof(Vault) - sizeof(uint32_t), "CRC must stay last");

/// Load /jencore.bin into `out`. A missing file, bad magic/version/size, or CRC mismatch
/// returns false and writes the reason to `*reason` when non-null. `out` is zeroed first,
/// so a failed load leaves every field at its default.
bool load(Vault &out, const char **reason = nullptr);

/// Fill in magic, version and CRC, then atomically replace /jencore.bin.
bool save(Vault &in);
} // namespace jenCoreVault

#endif
