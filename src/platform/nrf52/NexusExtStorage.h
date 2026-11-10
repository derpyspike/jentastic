// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) 2026 Jennifer Sanchez
//
// Two LittleFS partitions on the external QSPI flash:
//   /ext       512 KB, first  (contacts/identity/small metadata)
//   /secondary rest of chip   (history/logs/archive)
//
// Mount is checked from runOnce(); a blank partition is provisioned in small
// erase steps across ticks so the node never blocks for a full format.

#pragma once

#include "configuration.h"

#if NEXUS_ENABLE_EXTERNAL_FLASH && defined(ARCH_NRF52)

#include <stddef.h>

namespace nexus::extstorage
{
/// One provisioning step. Returns true while work remains (caller should poll
/// again soon instead of the normal idle interval).
bool loop();

/// Compact status for both partitions (used/free/total or formatting progress).
void status(char *out, size_t cap);

/// Start re-provisioning: "ext", "secondary" (alias "sec"), or "all".
void format(const char *which, char *out, size_t cap);

/// Write/read/remove a small file. which: "ext" (default) or "secondary".
void selfTest(const char *which, char *out, size_t cap);
} // namespace nexus::extstorage

#endif // NEXUS_ENABLE_EXTERNAL_FLASH && ARCH_NRF52
