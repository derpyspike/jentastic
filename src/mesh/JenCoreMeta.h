#pragma once

#include "mesh/generated/meshtastic/mesh.pb.h"
#include <cstddef>
#include <cstdint>
#include <cstring>

/**
 * Client-only metadata for packets synthesized from JenCore frames: the packed
 * MeshPacket.jc_flags word, the resolved relay and the path tail.
 *
 * It travels to the phone inside FromRadio and is never part of the 16-byte
 * LoRa header. Kept independent of the companion so the packing and truncation
 * rules can be exercised natively.
 */
namespace nexus::jen_core_meta
{

constexpr size_t PATH_CAP = sizeof(((meshtastic_MeshPacket *)nullptr)->jc_path.bytes);

enum : uint16_t {
    FLAG_IS_JENCORE = 1u << 0,
    FLAG_REGION_SCOPED = 1u << 1,
    FLAG_PATH_TRUNCATED = 1u << 2,

    ROUTE_SHIFT = 3,
    ROUTE_MASK = 0x3,
    PAYLOAD_SHIFT = 5,
    PAYLOAD_MASK = 0xF,
    HASH_SIZE_SHIFT = 9,
    HASH_SIZE_MASK = 0x3,
    REGION_SHIFT = 11,
    REGION_MASK = 0x7,

    REGION_NONE = 0x7,
};

constexpr uint16_t packFlags(bool isJenCore, bool regionScoped, bool pathTruncated, uint8_t routeType,
                             uint8_t payloadType, uint8_t pathHashSize, uint8_t regionIndex)
{
    if (!isJenCore)
        return 0; // keep the field absent on ordinary Meshtastic packets

    uint16_t f = FLAG_IS_JENCORE;
    if (regionScoped)
        f |= FLAG_REGION_SCOPED;
    if (pathTruncated)
        f |= FLAG_PATH_TRUNCATED;
    f |= (uint16_t)(routeType & ROUTE_MASK) << ROUTE_SHIFT;
    f |= (uint16_t)(payloadType & PAYLOAD_MASK) << PAYLOAD_SHIFT;
    f |= (uint16_t)((pathHashSize > 0 ? pathHashSize - 1 : 0) & HASH_SIZE_MASK) << HASH_SIZE_SHIFT;
    f |= (uint16_t)((regionIndex == 0xFF ? REGION_NONE : regionIndex) & REGION_MASK) << REGION_SHIFT;
    return f;
}

/**
 * Copy the tail of a path (the most recent hops, including the immediate relay)
 * into out, keeping only whole hashes. Returns the bytes written and reports
 * whether any hop was dropped.
 */
inline size_t copyPathTail(const uint8_t *path, size_t pathBytes, uint8_t hashSize, uint8_t *out, size_t cap,
                           bool *truncated)
{
    if (truncated)
        *truncated = false;
    if (!path || !out || pathBytes == 0 || hashSize == 0 || cap == 0)
        return 0;

    const size_t hashes = pathBytes / hashSize;
    size_t keep = cap / hashSize;
    if (keep > hashes)
        keep = hashes;

    const size_t keepBytes = keep * hashSize;
    if (keepBytes == 0) {
        if (truncated)
            *truncated = true;
        return 0;
    }

    memcpy(out, path + pathBytes - keepBytes, keepBytes);
    if (truncated)
        *truncated = keepBytes < pathBytes;
    return keepBytes;
}

inline void apply(meshtastic_MeshPacket &p, uint16_t flags, uint16_t relayNode, const uint8_t *pathTail, size_t pathLen)
{
    p.jc_flags = flags;
    p.jc_relay_node = relayNode;
    const size_t n = pathLen > PATH_CAP ? PATH_CAP : pathLen;
    p.jc_path.size = (pb_size_t)n;
    if (n)
        memcpy(p.jc_path.bytes, pathTail, n);
}

} // namespace nexus::jen_core_meta
