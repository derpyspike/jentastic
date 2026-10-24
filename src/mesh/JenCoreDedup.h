// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) 2026 Jennifer Sanchez
//
// Licensed under the GNU Affero General Public License v3.
// See LICENSE-AGPL for the full license text.
//

#pragma once

#include <stddef.h>
#include <stdint.h>

/** Fixed-size ring of recently seen JenCore frame hashes. 0 marks an empty slot and is never stored. */
template <size_t N> class JenCoreDedup
{
  public:
    bool contains(uint32_t hash) const
    {
        for (size_t i = 0; i < N; i++) {
            if (entries[i] == hash)
                return true;
        }
        return false;
    }

    void remember(uint32_t hash)
    {
        entries[next] = hash;
        next = (next + 1) % N;
    }

    /** Take a hash back out, so a copy of a frame we never got to forward is still eligible. */
    void forget(uint32_t hash)
    {
        for (size_t i = 0; i < N; i++) {
            if (entries[i] == hash)
                entries[i] = 0;
        }
    }

  private:
    uint32_t entries[N] = {};
    size_t next = 0;
};
