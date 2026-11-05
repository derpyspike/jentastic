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

/// Largest path hash width JenCore uses (1, 2 or 3 bytes).
constexpr uint8_t HASH_SIZE_MAX = 3;

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

/// Path hash width encoded in jc_flags (1..3 bytes).
inline uint8_t pathHashSize(uint16_t flags)
{
    const uint8_t size = (uint8_t)(((flags >> HASH_SIZE_SHIFT) & HASH_SIZE_MASK) + 1);
    return size > HASH_SIZE_MAX ? HASH_SIZE_MAX : size;
}

/// Wire path budget shared by every JenCore node (MAX_PATH_SIZE in JenCoreRepeater).
constexpr size_t PATH_BYTES_MAX = 64;

/// Maximum hops a path of this hash width can carry inside the 64-byte budget:
/// 64 for 1-byte, 32 for 2-byte, 21 for 3-byte hashes.
inline uint8_t pathHopLimit(uint8_t hashSize)
{
    if (hashSize == 0 || hashSize > HASH_SIZE_MAX)
        return 0;
    return (uint8_t)(PATH_BYTES_MAX / hashSize);
}

/// Decode the JenCore hop count and its byte-budget ceiling from a packet.
/// Returns false for ordinary Meshtastic packets (jc_flags absent).
inline bool packetHopInfo(const meshtastic_MeshPacket &p, uint8_t &hops, uint8_t &limit)
{
    hops = 0;
    limit = 0;
    if (!(p.jc_flags & FLAG_IS_JENCORE))
        return false;
    const uint8_t hashSize = pathHashSize(p.jc_flags);
    hops = hashSize ? (uint8_t)(p.jc_path.size / hashSize) : 0;
    limit = pathHopLimit(hashSize);
    return true;
}

/**
 * Render a path as "!aabb>!ccdd", one '!'-prefixed hex label per whole hash.
 * When the whole path does not fit in cap, keep the first and last hops and
 * mark the elided middle with "..", so the immediate relay is never lost. A
 * path whose head was ever cut (truncated) is prefixed "..>".
 * Returns the content bytes written, excluding the NUL.
 */
inline size_t formatPath(char *out, size_t cap, const uint8_t *path, size_t pathBytes, uint8_t hashSize, bool truncated)
{
    if (!out || cap == 0)
        return 0;
    out[0] = '\0';
    if (!path || hashSize == 0 || hashSize > HASH_SIZE_MAX)
        return 0;

    const size_t total = pathBytes / hashSize;
    if (total == 0)
        return 0;

    const size_t label = 1 + 2 * (size_t)hashSize;
    size_t pos = 0;
    auto putChar = [&](char c) {
        if (pos + 1 < cap)
            out[pos++] = c;
    };
    auto putLabel = [&](size_t idx) {
        const uint8_t *hash = path + idx * hashSize;
        static const char hexd[] = "0123456789abcdef";
        putChar('!');
        for (uint8_t i = 0; i < hashSize; i++) {
            putChar(hexd[hash[i] >> 4]);
            putChar(hexd[hash[i] & 0x0F]);
        }
    };
    auto room = [&]() { return cap - 1 - pos; };

    if (truncated) {
        putChar('.');
        putChar('.');
        putChar('>');
    }

    if (total == 1) {
        putLabel(0);
        out[pos] = '\0';
        return pos;
    }

    // Two hops plus the separator are the floor. If not even that fits, keep
    // the immediate relay (the hop the operator cares about) rather than none.
    if (room() < label + 1 + label) {
        putLabel(total - 1);
        out[pos] = '\0';
        return pos;
    }

    putLabel(0);
    size_t next = 1;
    const size_t reserveLast = 4 + label; // ">..>" + last label
    while (next + 1 < total) {
        if (room() < 1 + label + reserveLast)
            break;
        putChar('>');
        putLabel(next);
        next++;
    }

    if (next < total - 1) {
        putChar('>');
        putChar('.');
        putChar('.');
        putChar('>');
    } else {
        putChar('>');
    }
    putLabel(total - 1);
    out[pos] = '\0';
    return pos;
}

} // namespace nexus::jen_core_meta
