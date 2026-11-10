// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) 2026 Jennifer Sanchez
//
// Low-level external QSPI NOR access. NexusExtStorage owns the LittleFS
// partitions and serialises every call; this layer only talks to the chip.

#pragma once

#include "configuration.h"

#if NEXUS_ENABLE_EXTERNAL_FLASH && defined(ARCH_NRF52)

#include <stddef.h>
#include <stdint.h>

namespace nexus::extflash
{
/// Lazy probe (rail, QSPI init, JEDEC). Idempotent.
bool begin();

/// Re-probe after a failed boot-time attempt, without re-initialising QSPI.
bool retry();

bool ready();
uint32_t jedec();
uint32_t capacity();

/// Block-device primitives. Callers must hold the storage lock.
bool read(uint32_t addr, void *buf, size_t len);
bool program(uint32_t addr, const void *buf, size_t len);
bool erase4k(uint32_t addr);
} // namespace nexus::extflash

#endif // NEXUS_ENABLE_EXTERNAL_FLASH && ARCH_NRF52
