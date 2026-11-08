// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) 2026 Jennifer Sanchez
//
// Licensed under the GNU Affero General Public License v3.
// See LICENSE-AGPL for the full license text.
//
// Raw MeshCore / RNode tunnel over the phone API. Frames travel inside a
// MeshPacket with the encrypted payload variant, prefixed with a magic header,
// so upstream clients see an opaque encrypted packet and ignore it while a
// custom app can parse and inject raw frames.
//
// Wire format (MeshPacket.encrypted):
//   0..3  "JRAW"
//   4     version (1)
//   5     type (1 = MeshCore frame, 2 = RNode/Reticulum payload)
//   6     seq (1..255; groups the chunks of one frame)
//   7     chunk index (0..count-1)
//   8     chunk count (1..2)
//   9..   chunk bytes
//
// The raw frame is the concatenation of the chunks in index order. Upstream
// apps never see a decoded payload, so they treat it as encrypted traffic.

#pragma once

#include "configuration.h"
#include <stddef.h>
#include <stdint.h>

#if NEXUS_ENABLE_JENCORE || NEXUS_ENABLE_RNODE

struct _meshtastic_MeshPacket;

namespace nexus::rawtunnel
{
constexpr uint8_t VERSION = 1;
constexpr uint8_t MAGIC[4] = {'J', 'R', 'A', 'W'};
constexpr size_t HEADER_LEN = 9;
constexpr size_t MAX_CHUNKS = 2;
constexpr size_t CHUNK_SIZE = 256 - HEADER_LEN; // MeshPacket.encrypted capacity
constexpr size_t MAX_FRAME = CHUNK_SIZE * MAX_CHUNKS;

enum Type : uint8_t {
    MESHCORE = 1, // full JenCore frame, as heard on air
    RNODE = 2,    // RNode/Reticulum payload (no 1-byte RNode header)
};

/// Send a raw frame to the phone as one or two opaque packets (no-op when the
/// RAW_TUNNEL_TO_PHONE feature is off).
void forward(Type type, const uint8_t *data, size_t len, int32_t rssi, float snr, float freqErr, uint8_t codingRate);

/// Intercept a phone packet carrying the JRAW magic. Returns true when it was
/// consumed by the tunnel (including when the feature is disabled); false for
/// ordinary phone packets.
bool handleFromPhone(const struct _meshtastic_MeshPacket &p);
} // namespace nexus::rawtunnel

#else

struct _meshtastic_MeshPacket;
namespace nexus::rawtunnel
{
enum Type : uint8_t { MESHCORE = 1, RNODE = 2 };
inline void forward(Type, const uint8_t *, size_t, int32_t, float, float, uint8_t) {}
inline bool handleFromPhone(const struct _meshtastic_MeshPacket &)
{
    return false;
}
} // namespace nexus::rawtunnel

#endif
