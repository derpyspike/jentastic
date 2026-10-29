// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) 2026 Jennifer Sanchez
//
// Licensed under the GNU Affero General Public License v3.
// See LICENSE-AGPL for the full license text.
//

#pragma once

#include "configuration.h"
#include <stddef.h>
#include <stdint.h>

class JenCoreCompanion;

/**
 * Flash persistence for the JenCore identity seed and the companion's saved contacts.
 *
 * This is storage policy, not radio code; RadioLibInterface only calls it. The identity seed is
 * created from the hardware RNG on first use, never from a weak source. Contacts are written at
 * most once per CONTACTS_SAVE_MIN_MS and only when their fingerprint changed.
 */
namespace jenCoreStore
{
constexpr size_t SEED_LEN = 32;

#if NEXUS_ENABLE_JENCORE_SNIFFER
/// Load the persisted 32-byte seed, or create and save a new one when missing/corrupt or forceNew.
/// Returns false only when the hardware RNG could not provide entropy.
bool loadOrCreateSeed(uint8_t seed[SEED_LEN], bool forceNew);
#endif

#if NEXUS_ENABLE_JENCORE_COMPANION
/// Restore the saved contacts once per boot (call after the companion has an identity).
void loadContactsOnce(JenCoreCompanion &companion);
/// Save the contacts when their fingerprint changed since the last save.
void saveContactsIfChanged(JenCoreCompanion &companion);
#endif
} // namespace jenCoreStore
