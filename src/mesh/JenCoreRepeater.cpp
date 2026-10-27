// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) 2026 Jennifer Sanchez
//
// Licensed under the GNU Affero General Public License v3.
// See LICENSE-AGPL for the full license text.
//

#include "JenCoreRepeater.h"

#include "Arduino.h"
#include "Ed25519.h"
#include "NodeDB.h"
#include "Throttle.h"
#include "UptimeClock.h"
#include "meshUtils.h"
#include <SHA256.h>
#include <string.h>
#include <strings.h>

#if NEXUS_ENABLE_JENCORE_SNIFFER

// JenCore Dispatcher::tryParsePacket payload types (see JenCore src/Packet.h)
#define MC_PAYLOAD_TYPE_REQ 0x00
#define MC_PAYLOAD_TYPE_RESPONSE 0x01
#define MC_PAYLOAD_TYPE_TXT_MSG 0x02
#define MC_PAYLOAD_TYPE_ACK 0x03
#define MC_PAYLOAD_TYPE_ADVERT 0x04
#define MC_PAYLOAD_TYPE_GRP_TXT 0x05
#define MC_PAYLOAD_TYPE_GRP_DATA 0x06
#define MC_PAYLOAD_TYPE_ANON_REQ 0x07
#define MC_PAYLOAD_TYPE_PATH 0x08
#define MC_PAYLOAD_TYPE_TRACE 0x09
#define MC_PAYLOAD_TYPE_MULTIPART 0x0A
#define MC_PAYLOAD_TYPE_CONTROL 0x0B
#define MC_PAYLOAD_TYPE_RAW_CUSTOM 0x0F

// JenCore route types (Packet.h PH_ROUTE_MASK)
#define MC_ROUTE_TRANSPORT_FLOOD 0x00
#define MC_ROUTE_FLOOD 0x01
#define MC_ROUTE_DIRECT 0x02
#define MC_ROUTE_TRANSPORT_DIRECT 0x03

// JenCore path_len byte (Packet.h): upper 2 bits = hash_size-1, lower 6 = count. Mode 3 reserved.
#define MC_PATH_MODE_MASK 0xC0
#define MC_PATH_MODE_RESERVED 0xC0
#define MC_PATH_COUNT_MASK 0x3F
#define MC_PATH_SIZE_SHIFT 6

// JenCore header byte 0: bits 6-7 are the payload version (must be PAYLOAD_VER_1 = 0).
#define MC_PAYLOAD_VER_SHIFT 6

// A full 16-byte LoRa packet header (to/from/id/flags/channel/next_hop/relay_node) must fit;
// spelled out to keep this file free of RadioInterface.h.
#define JC_LORA_HEADER_LEN 16

// JenCore ADVERT encoding (JenCore.h / AdvertDataHelpers.h)
#define MC_PUB_KEY_SIZE 32
#define MC_SIGNATURE_SIZE 64
#define MAX_ADVERT_DATA_SIZE 32
#define MC_ADV_TYPE_REPEATER 0x02
#define MC_ADV_NAME_MASK 0x80
#define MC_MAX_PACKET_PAYLOAD 184 // JenCore.h MAX_PACKET_PAYLOAD

// JenCore CONTROL subtypes (simple_repeater MyMesh.cpp)
#define MC_CTL_DISCOVER_REQ 0x80
#define MC_CTL_DISCOVER_RESP 0x90

static bool isKnownPayloadType(uint8_t type)
{
    return type == MC_PAYLOAD_TYPE_REQ || type == MC_PAYLOAD_TYPE_RESPONSE || type == MC_PAYLOAD_TYPE_TXT_MSG ||
           type == MC_PAYLOAD_TYPE_ACK || type == MC_PAYLOAD_TYPE_ADVERT || type == MC_PAYLOAD_TYPE_GRP_TXT ||
           type == MC_PAYLOAD_TYPE_GRP_DATA || type == MC_PAYLOAD_TYPE_ANON_REQ || type == MC_PAYLOAD_TYPE_PATH ||
           type == MC_PAYLOAD_TYPE_TRACE || type == MC_PAYLOAD_TYPE_MULTIPART || type == MC_PAYLOAD_TYPE_CONTROL ||
           type == MC_PAYLOAD_TYPE_RAW_CUSTOM;
}

// Mesh::onRecvPacket forwards an ADVERT only when its Ed25519 signature over pub_key || timestamp || app_data holds.
static bool advertSignatureValid(const uint8_t *payload, size_t payloadLen)
{
    constexpr size_t signedHeadLen = MC_PUB_KEY_SIZE + 4;
    constexpr size_t appDataAt = signedHeadLen + MC_SIGNATURE_SIZE;
    if (payloadLen < appDataAt)
        return false;
    size_t appLen = payloadLen - appDataAt;
    if (appLen > MAX_ADVERT_DATA_SIZE)
        appLen = MAX_ADVERT_DATA_SIZE;

    uint8_t message[signedHeadLen + MAX_ADVERT_DATA_SIZE];
    memcpy(message, payload, signedHeadLen);
    memcpy(&message[signedHeadLen], &payload[appDataAt], appLen);
    return Ed25519::verify(&payload[signedHeadLen], payload, message, signedHeadLen + appLen);
}

uint8_t JenCoreRepeater::nodeTypeForRole(int role)
{
    switch (role) {
    case meshtastic_Config_DeviceConfig_Role_ROUTER:
    case meshtastic_Config_DeviceConfig_Role_ROUTER_LATE:
    case meshtastic_Config_DeviceConfig_Role_REPEATER:
        return ADV_TYPE_REPEATER;
    default:
        return ADV_TYPE_CHAT;
    }
}

bool JenCoreRepeater::verifyAdvert(const uint8_t *payload, size_t payloadLen)
{
    return advertSignatureValid(payload, payloadLen);
}

// Every encrypted JenCore payload is [routing hashes][2-byte MAC][AES-128 blocks], and Utils::encrypt zero-pads the
// last block, so what follows the fixed part is a whole number of 16-byte blocks. A Meshtastic frame that parsed as
// JenCore by accident has this shape about one time in sixteen.
static bool payloadShapeValid(uint8_t payloadType, size_t payloadLen)
{
    constexpr size_t macSize = 2, blockSize = 16;
    size_t fixedLen;
    switch (payloadType) {
    case MC_PAYLOAD_TYPE_REQ:
    case MC_PAYLOAD_TYPE_RESPONSE:
    case MC_PAYLOAD_TYPE_TXT_MSG:
    case MC_PAYLOAD_TYPE_PATH:
        fixedLen = 1 + 1 + macSize; // dest hash, src hash
        break;
    case MC_PAYLOAD_TYPE_ANON_REQ:
        fixedLen = 1 + MC_PUB_KEY_SIZE + macSize; // dest hash, sender key
        break;
    case MC_PAYLOAD_TYPE_GRP_TXT:
    case MC_PAYLOAD_TYPE_GRP_DATA:
        fixedLen = 1 + macSize; // channel hash
        break;
    default:
        return true; // ADVERT is signed, the rest has no block structure we check
    }
    return payloadLen > fixedLen && (payloadLen - fixedLen) % blockSize == 0;
}

uint8_t JenCoreRepeater::ownPathLenByte()
{
    return static_cast<uint8_t>((getPathHashSize() - 1) << 6);
}

bool JenCoreRepeater::isJenCoreEnabled()
{
    return enabledStorage();
}

void JenCoreRepeater::setJenCoreEnabled(bool en)
{
    enabledStorage() = en;
}

bool JenCoreRepeater::isRelayEnabled()
{
    return relayStorage();
}

void JenCoreRepeater::setRelayEnabled(bool en)
{
    relayStorage() = en;
}

uint8_t JenCoreRepeater::getPathHashSize()
{
    return pathHashStorage();
}

void JenCoreRepeater::setPathHashSize(uint8_t size)
{
    pathHashStorage() = size < 1 ? 1 : (size > 3 ? 3 : size);
}

static bool normalizeRegionName(const char *input, char *out, size_t cap)
{
    if (!input || !out || cap < 2)
        return false;
    while (*input == ' ' || *input == '#')
        input++;
    size_t n = 0;
    for (; *input && *input != ' '; input++) {
        const char c = *input;
        if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '-' || c == '_' ||
              c == '.'))
            return false;
        if (n + 1 >= cap)
            return false;
        out[n++] = c;
    }
    out[n] = '\0';
    return n > 0;
}

void JenCoreRepeater::regionKey(const char *name, uint8_t key[16])
{
    char hashtag[REGION_NAME_LEN + 2];
    hashtag[0] = '#';
    strncpy(&hashtag[1], name, REGION_NAME_LEN);
    hashtag[REGION_NAME_LEN + 1] = '\0';

    SHA256 sha;
    sha.reset();
    sha.update((const uint8_t *)hashtag, strlen(hashtag));
    uint8_t digest[32];
    sha.finalize(digest, sizeof(digest));
    memcpy(key, digest, 16);
}

uint16_t JenCoreRepeater::transportCode(const uint8_t key[16], uint8_t payloadType, const uint8_t *payload, size_t len)
{
    SHA256 sha;
    uint16_t code = 0;
    sha.resetHMAC(key, 16);
    sha.update(&payloadType, 1);
    if (payload && len)
        sha.update(payload, len);
    sha.finalizeHMAC(key, 16, &code, sizeof(code));
    if (code == 0)
        code = 1;
    else if (code == 0xFFFF)
        code = 0xFFFE;
    return code;
}

bool JenCoreRepeater::isRegionAllowed(const uint8_t *raw, size_t len, const FrameInfo &info)
{
    const RegionStore &store = regionStore();
    if (store.count == 0)
        return true;

    if (info.route_type == MC_ROUTE_TRANSPORT_FLOOD) {
        if (len < 5)
            return false;
        const uint16_t code = (uint16_t)raw[1] | ((uint16_t)raw[2] << 8);
        if (code == 0)
            return store.allowUnscoped;
        const uint8_t *payload = raw + info.header_len;
        for (uint8_t i = 0; i < store.count; i++) {
            uint8_t key[16];
            regionKey(store.names[i], key);
            if (transportCode(key, info.payload_type, payload, info.payload_len) == code)
                return true;
        }
        return false;
    }
    if (info.route_type == MC_ROUTE_FLOOD)
        return store.allowUnscoped;
    return true; // MeshCore only scopes floods
}

bool JenCoreRepeater::isFrameRegionAllowed(const uint8_t *raw, size_t len)
{
    FrameInfo info;
    if (!parse(raw, len, &info))
        return true; // let the normal parser drop malformed frames
    return isRegionAllowed(raw, len, info);
}

bool JenCoreRepeater::addRegion(const char *name)
{
    RegionStore &store = regionStore();
    char normalized[REGION_NAME_LEN + 1];
    if (!normalizeRegionName(name, normalized, sizeof(normalized)))
        return false;
    for (uint8_t i = 0; i < store.count; i++)
        if (strcasecmp(store.names[i], normalized) == 0)
            return true;
    if (store.count >= MAX_REGIONS)
        return false;
    strncpy(store.names[store.count], normalized, REGION_NAME_LEN);
    store.names[store.count][REGION_NAME_LEN] = '\0';
    store.count++;
    return true;
}

bool JenCoreRepeater::removeRegion(const char *name)
{
    RegionStore &store = regionStore();
    char normalized[REGION_NAME_LEN + 1];
    if (!normalizeRegionName(name, normalized, sizeof(normalized)))
        return false;
    for (uint8_t i = 0; i < store.count; i++) {
        if (strcasecmp(store.names[i], normalized) != 0)
            continue;
        for (uint8_t j = i; j + 1 < store.count; j++)
            strncpy(store.names[j], store.names[j + 1], REGION_NAME_LEN + 1);
        store.count--;
        if (store.count == 0) {
            store.home = 0;
        } else if (store.home == i) {
            store.home = 0;
        } else if (store.home > i) {
            store.home--;
        }
        return true;
    }
    return false;
}

bool JenCoreRepeater::setHomeRegion(const char *name)
{
    RegionStore &store = regionStore();
    char normalized[REGION_NAME_LEN + 1];
    if (!normalizeRegionName(name, normalized, sizeof(normalized)))
        return false;
    for (uint8_t i = 0; i < store.count; i++) {
        if (strcasecmp(store.names[i], normalized) == 0) {
            store.home = i;
            return true;
        }
    }
    return false;
}

void JenCoreRepeater::clearRegions()
{
    RegionStore &store = regionStore();
    store.count = 0;
    store.home = 0;
    store.allowUnscoped = true;
}

void JenCoreRepeater::setAllowUnscoped(bool allow)
{
    regionStore().allowUnscoped = allow;
}

uint8_t JenCoreRepeater::getRegionCount()
{
    return regionStore().count;
}

const char *JenCoreRepeater::getRegionName(uint8_t index)
{
    const RegionStore &store = regionStore();
    return index < store.count ? store.names[index] : "";
}

uint8_t JenCoreRepeater::getHomeRegionIndex()
{
    const RegionStore &store = regionStore();
    return store.count ? store.home : 0;
}

bool JenCoreRepeater::getAllowUnscoped()
{
    return regionStore().allowUnscoped;
}

const uint8_t *JenCoreRepeater::scopeOwnFrame(const uint8_t *raw, size_t len, uint8_t *out, size_t outCap, size_t *outLen)
{
    *outLen = len;
    const RegionStore &store = regionStore();
    if (store.count == 0)
        return raw;

    FrameInfo info;
    if (!parse(raw, len, &info))
        return raw;
    if (info.route_type != MC_ROUTE_FLOOD)
        return raw;
    if (len + 4 > outCap)
        return nullptr;

    uint8_t key[16];
    regionKey(store.names[store.home], key);
    const uint16_t code = transportCode(key, info.payload_type, raw + info.header_len, info.payload_len);

    out[0] = (uint8_t)((raw[0] & ~0x03) | MC_ROUTE_TRANSPORT_FLOOD);
    out[1] = (uint8_t)(code & 0xFF);
    out[2] = (uint8_t)(code >> 8);
    out[3] = 0;
    out[4] = 0;
    memcpy(out + 5, raw + 1, len - 1);
    *outLen = len + 4;
    return out;
}

bool JenCoreRepeater::looksLikeJenCoreFrame(const uint8_t *raw, size_t len, uint32_t ourNodeNum, FrameInfo *out)
{
    if (!raw || len < 2 || len > MAX_FRAME_LEN)
        return false;

    FrameInfo info;
    if (!parse(raw, len, &info))
        return false;
    if (!payloadShapeValid(info.payload_type, info.payload_len))
        return false;

    // Meshtastic-header veto: bytes 0..3 and 4..7 of a real Meshtastic frame are its to/from
    // NodeNums. A frame addressed to or from us is Meshtastic with near certainty (our NodeNum is
    // crc32 of our public key; a JenCore header colliding with it is a 2^-32 accident), and it
    // must never be swallowed - or worse, flood-relayed - as foreign traffic. Broadcasts
    // (to == 0xffffffff) never reach here: their first byte fails the payload version check.
    if (ourNodeNum != 0 && len >= JC_LORA_HEADER_LEN) {
        uint32_t to = raw[0] | (raw[1] << 8) | (raw[2] << 16) | ((uint32_t)raw[3] << 24);
        uint32_t from = raw[4] | (raw[5] << 8) | (raw[6] << 16) | ((uint32_t)raw[7] << 24);
        if (to == ourNodeNum || from == ourNodeNum)
            return false;
    }

    if (out)
        *out = info;
    return true;
}

bool JenCoreRepeater::parse(const uint8_t *raw, size_t len, FrameInfo *out)
{
    if (!raw || len < 2)
        return false;

    const uint8_t header = raw[0];
    FrameInfo info;
    info.route_type = header & 0x03;
    info.payload_type = (header >> 2) & 0x0F;
    info.payload_ver = (header >> MC_PAYLOAD_VER_SHIFT) & 0x03;

    if (info.payload_ver != 0) // PAYLOAD_VER_1 only
        return false;
    if (!isKnownPayloadType(info.payload_type))
        return false;

    size_t i = 1;
    if (info.route_type == MC_ROUTE_TRANSPORT_FLOOD || info.route_type == MC_ROUTE_TRANSPORT_DIRECT) {
        i += 4; // two uint16 transport codes
    }

    if (i >= len)
        return false;

    const uint8_t path_len_byte = raw[i++];
    if ((path_len_byte & MC_PATH_MODE_MASK) == MC_PATH_MODE_RESERVED) // path mode 3 reserved
        return false;

    info.path_hash_size = (path_len_byte >> MC_PATH_SIZE_SHIFT) + 1;
    info.path_hash_count = path_len_byte & MC_PATH_COUNT_MASK;
    if (info.path_hash_size < 1 || info.path_hash_size > MAX_PATH_HASH_SIZE)
        return false;
    if (info.path_hash_count > MAX_PATH_HASH_COUNT)
        return false;

    // Dispatcher::checkRecv: a path over MAX_PATH_SIZE or a payload over MAX_PACKET_PAYLOAD is not a JenCore packet.
    size_t pathBytes = (size_t)info.path_hash_count * info.path_hash_size;
    if (pathBytes > MAX_PATH_SIZE || i + pathBytes > len)
        return false;
    i += pathBytes;
    if (len - i > MC_MAX_PACKET_PAYLOAD)
        return false;

    info.header_len = (uint8_t)i;
    info.payload_len = (uint8_t)(len - i);

    if (out)
        *out = info;
    return true;
}

void JenCoreRepeater::handleReceivedFrame(const uint8_t *raw, size_t len, float snr, uint32_t airtimeMs)
{
    (void)snr;       // only used by the relay half (discovery replies report it)
    (void)airtimeMs; // only used by the relay half
    FrameInfo info;
    if (!parse(raw, len, &info)) {
        rxDropped++;
        return;
    }

    rxSeen++;

    if (!isRegionAllowed(raw, len, info)) {
        LOG_DEBUG("JenCore: region scope mismatch, drop");
        rxDropped++;
        return;
    }

#if NEXUS_ENABLE_JENCORE_RELAY
    if (!isRelayEnabled()) {
        return;
    }
    const bool direct = info.route_type == MC_ROUTE_DIRECT || info.route_type == MC_ROUTE_TRANSPORT_DIRECT;
    if (direct) {
        // Zero-hop CONTROL is neighbour discovery (Mesh::onRecvPacket); any other direct frame is forwarded only
        // when we are its next hop, exactly like a JenCore repeater.
        if (info.payload_type == MC_PAYLOAD_TYPE_CONTROL) {
            if (info.path_hash_count == 0)
                handleControl(raw, len, info, snr, airtimeMs);
        } else {
            handleDirect(raw, len, info, snr, airtimeMs);
        }
        return;
    }
    // Only what JenCore itself routes on: anything else (TRACE, MULTIPART, CONTROL, RAW_CUSTOM, a truncated packet) is
    // not forwarded by a JenCore repeater - and is more likely a misparsed Meshtastic frame than real traffic.
    if (!isFloodForwardable(info)) {
        rxDropped++;
        return;
    }
    rxFlood++;

    // A flood that never got its turn is worthless once its dupe window is over: free its slot.
    expireStale();

    // JenCore's flood_max_advert default (NodePrefs.h): ADVERT floods are not forwarded past 8
    // hops. The general flood_max (64) is implied by the 6-bit hop count plus the path bound below.
    if (info.payload_type == MC_PAYLOAD_TYPE_ADVERT && info.path_hash_count >= MAX_FLOOD_HOPS_ADVERT) {
        LOG_DEBUG("JenCore: advert flood past %u hops, drop", info.path_hash_count);
        rxDropped++;
        return;
    }

    // JenCore appends its own hash to the path on each hop (Mesh::routeRecvPacket) and refuses
    // to forward when the path would overflow MAX_PATH_SIZE (64 bytes). We append ours too (once we
    // have an identity), so the same budget applies.
    size_t nextPathBytes = ((size_t)info.path_hash_count + 1) * info.path_hash_size;
    if (nextPathBytes > MAX_PATH_SIZE) {
        LOG_DEBUG("JenCore: flood path full (%u hops), drop", info.path_hash_count);
        rxDropped++;
        return;
    }

    // Our own ADVERT coming back (Mesh::onRecvPacket drops a SELF advert before anything else).
    if (info.payload_type == MC_PAYLOAD_TYPE_ADVERT && hasIdentityKey &&
        memcmp(&raw[info.header_len], identityPub, MC_PUB_KEY_SIZE) == 0)
        return;

    uint32_t hash = hashFrame(raw, len, info);
    if (wasRecentlySeen(hash)) {
        rxDup++;
        onDupeHeard(hash);
        return;
    }

    if (!canRelay())
        return;

    // With an identity, forward like a JenCore repeater: our hash goes on the path, so receivers can tell the
    // packet came through us (heard-by, return paths). Without one, the frame is relayed unchanged.
    uint8_t forwarded[MAX_FRAME_LEN];
    const uint8_t *out = raw;
    size_t outLen = len;
    if (hasIdentityKey) {
        outLen = rewritePath(raw, len, info, true, forwarded, sizeof(forwarded));
        if (outLen == 0) {
            rxDropped++;
            return;
        }
        out = forwarded;
    }

    // An ADVERT is only forwarded if its Ed25519 signature holds (Mesh::onRecvPacket). The check takes long enough to
    // deafen the radio, so it waits until the frame is about to go on air (takePendingFrame), with the radio listening.
    // Remembered only once queued: a flood dropped here was never forwarded, so a later copy of it must still be
    // eligible. A forged advert is remembered too, as JenCore marks it seen first: a forged copy is not checked twice.
    const uint8_t hops = info.path_hash_count < 200 ? info.path_hash_count : 200;
    if (!queueRelay(out, outLen, hash, airtimeMs, adaptiveDelayMs(airtimeMs), PRIORITY_FLOOD_BASE + hops,
                    info.payload_type == MC_PAYLOAD_TYPE_ADVERT))
        return;
    rememberFrame(hash);
#endif
}

#if NEXUS_ENABLE_JENCORE_RELAY
bool JenCoreRepeater::isNextHopForUs(const uint8_t *raw, const FrameInfo &info) const
{
    if (!hasIdentityKey || info.path_hash_count == 0)
        return false;
    const size_t pathStart = info.header_len - (size_t)info.path_hash_count * info.path_hash_size;
    return memcmp(&raw[pathStart], identityPub, info.path_hash_size) == 0;
}

size_t JenCoreRepeater::rewritePath(const uint8_t *raw, size_t len, const FrameInfo &info, bool appendSelf, uint8_t *out,
                                     size_t outLen) const
{
    const size_t hashSize = info.path_hash_size;
    const size_t pathBytes = (size_t)info.path_hash_count * hashSize;
    const size_t pathLenAt = info.header_len - pathBytes - 1; // the path_len byte sits just before the path
    const size_t payloadLen = len - info.header_len;
    const uint8_t newCount = appendSelf ? info.path_hash_count + 1 : info.path_hash_count - 1;
    const size_t newLen = appendSelf ? len + hashSize : len - hashSize;
    if (newLen > outLen || newLen > MAX_FRAME_LEN || (!appendSelf && info.path_hash_count == 0))
        return 0;

    memcpy(out, raw, pathLenAt); // header (+ transport codes)
    out[pathLenAt] = (uint8_t)(((hashSize - 1) << MC_PATH_SIZE_SHIFT) | newCount);
    size_t i = pathLenAt + 1;
    if (appendSelf) {
        memcpy(&out[i], &raw[pathLenAt + 1], pathBytes);
        i += pathBytes;
        memcpy(&out[i], identityPub, hashSize); // a path hash is the public key prefix (Identity::copyHashTo)
        i += hashSize;
    } else {
        memcpy(&out[i], &raw[pathLenAt + 1 + hashSize], pathBytes - hashSize); // drop the first hop: us
        i += pathBytes - hashSize;
    }
    memcpy(&out[i], &raw[info.header_len], payloadLen);
    return newLen;
}

void JenCoreRepeater::handleDirect(const uint8_t *raw, size_t len, const FrameInfo &info, float snr, uint32_t airtimeMs)
{
    // A TRACE carries its route in the payload; its path field collects SNRs, so the next-hop test below does not apply.
    if (info.payload_type == MC_PAYLOAD_TYPE_TRACE) {
        handleTrace(raw, len, info, snr, airtimeMs);
        return;
    }
    if (!isNextHopForUs(raw, info))
        return;
    if (info.payload_type == MC_PAYLOAD_TYPE_MULTIPART) {
        handleMultipartAck(raw, len, info, airtimeMs);
        return;
    }
    rxDirectForUs++;

    const uint32_t hash = hashFrame(raw, len, info);
    if (wasRecentlySeen(hash)) {
        rxDup++;
        return;
    }
    if (!canRelay())
        return;

    uint8_t forwarded[MAX_FRAME_LEN];
    const size_t outLen = rewritePath(raw, len, info, false, forwarded, sizeof(forwarded));
    if (outLen == 0)
        return;
    if (!queueRelay(forwarded, outLen, hash, airtimeMs, directDelayMs(airtimeMs), PRIORITY_DIRECT))
        return;
    rememberFrame(hash);
}

uint32_t JenCoreRepeater::directDelayMs(uint32_t airtimeMs)
{
    // JenCore getDirectRetransmitDelay: routed traffic goes out sooner than floods.
    return DELAY_FLOOR_MS + (uint32_t)random(0, (long)airtimeMs + 1);
}

void JenCoreRepeater::handleTrace(const uint8_t *raw, size_t len, const FrameInfo &info, float snr, uint32_t airtimeMs)
{
    // Mesh::onRecvPacket, TRACE: payload = tag(4) auth(4) flags(1) then the hashes to visit; the path field holds
    // one SNR byte (x4) per hop already taken, so path_len indexes the hash list. flags&3 is the hash size exponent.
    constexpr size_t traceHeadLen = 4 + 4 + 1;
    if (!hasIdentityKey || info.path_hash_size != 1 || info.path_hash_count >= MAX_PATH_SIZE || info.payload_len < traceHeadLen)
        return;

    const uint8_t *payload = &raw[info.header_len];
    const size_t hashSize = (size_t)1 << (payload[8] & 0x03);
    const size_t offset = (size_t)info.path_hash_count * hashSize;
    if (traceHeadLen + offset + hashSize > info.payload_len) // end of the list: we are the destination, nothing to forward
        return;
    if (memcmp(&payload[traceHeadLen + offset], identityPub, hashSize) != 0)
        return;
    if (len + 1 > MAX_FRAME_LEN)
        return;

    const uint32_t hash = hashFrame(raw, len, info);
    if (wasRecentlySeen(hash)) {
        rxDup++;
        return;
    }
    if (!canRelay())
        return;
    rxDirectForUs++;

    // Copy the frame with one more path byte: our receive SNR (not our hash) after the SNRs already collected.
    const size_t pathLenAt = info.header_len - info.path_hash_count - 1;
    const size_t snrAt = info.header_len;
    uint8_t forwarded[MAX_FRAME_LEN];
    memcpy(forwarded, raw, snrAt);
    forwarded[pathLenAt] = raw[pathLenAt] + 1;
    float snrX4 = snr * 4;
    snrX4 = snrX4 > 127.0f ? 127.0f : (snrX4 < -128.0f ? -128.0f : snrX4);
    forwarded[snrAt] = (uint8_t)(int8_t)snrX4;
    memcpy(&forwarded[snrAt + 1], &raw[snrAt], len - snrAt);

    if (!queueRelay(forwarded, len + 1, hash, airtimeMs, directDelayMs(airtimeMs), PRIORITY_DIRECT))
        return;
    rememberFrame(hash);
}

void JenCoreRepeater::handleMultipartAck(const uint8_t *raw, size_t len, const FrameInfo &info, uint32_t airtimeMs)
{
    // Mesh::forwardMultipartDirect: only a multipart ACK (type in the low nibble, packets still to come in the high
    // one) is forwarded, as a plain direct ACK with us removed from the path, delayed to follow the rest of the sequence.
    const uint8_t *payload = &raw[info.header_len];
    if (info.payload_len < 5 || (payload[0] & 0x0F) != MC_PAYLOAD_TYPE_ACK)
        return;
    const uint8_t remaining = payload[0] >> 4;

    FrameInfo ackInfo = info;
    ackInfo.header_len++; // dedupe on the ACK body, not on the sequence counter
    const uint32_t hash = hashFrame(raw, len, ackInfo);
    if (wasRecentlySeen(hash)) {
        rxDup++;
        return;
    }
    if (!canRelay())
        return;
    rxDirectForUs++;

    const size_t hashSize = info.path_hash_size;
    const size_t pathBytes = (size_t)info.path_hash_count * hashSize;
    const size_t bodyLen = (size_t)info.payload_len - 1;
    uint8_t forwarded[MAX_FRAME_LEN];
    size_t i = 0;
    forwarded[i++] = (MC_PAYLOAD_TYPE_ACK << 2) | MC_ROUTE_DIRECT;
    forwarded[i++] = (uint8_t)(((hashSize - 1) << MC_PATH_SIZE_SHIFT) | (info.path_hash_count - 1));
    memcpy(&forwarded[i], &raw[info.header_len - pathBytes + hashSize], pathBytes - hashSize); // drop the first hop: us
    i += pathBytes - hashSize;
    memcpy(&forwarded[i], &payload[1], bodyLen);
    i += bodyLen;

    if (!queueRelay(forwarded, i, hash, airtimeMs, directDelayMs(airtimeMs) + ((uint32_t)remaining + 1) * 300, PRIORITY_DIRECT))
        return;
    rememberFrame(hash);
}

void JenCoreRepeater::handleControl(const uint8_t *raw, size_t len, const FrameInfo &info, float snr, uint32_t airtimeMs)
{
    // NODE_DISCOVER_REQ (simple_repeater onControlDataRecv): type 0x8_, filter, 4-byte tag, optional 'since'.
    const uint8_t *req = &raw[info.header_len];
    if (!hasIdentityKey || info.payload_len < 6 || (req[0] & 0xF0) != MC_CTL_DISCOVER_REQ)
        return;
    if ((req[1] & (1 << nodeType)) == 0)
        return;
    uint32_t since = 0;
    if (info.payload_len >= 10)
        memcpy(&since, &req[6], 4);
    if (since != 0) // we keep no 'modified since' timestamp: answer only the plain "who is there"
        return;
    if (config.device.role == meshtastic_Config_DeviceConfig_Role_CLIENT_MUTE)
        return;

    const uint32_t now = Time::getMillis();
    if (!Throttle::isWithinTimespanMs(discoverWindowStartMs, DISCOVER_WINDOW_MS)) {
        discoverWindowStartMs = now;
        discoverRepliesInWindow = 0;
    }
    if (discoverRepliesInWindow >= DISCOVER_MAX_REPLIES)
        return;

    // NODE_DISCOVER_RESP: type|our node type, request SNR x4, the request tag, then our key (8-byte prefix if asked).
    const bool prefixOnly = (req[0] & 0x01) != 0;
    const size_t keyLen = prefixOnly ? 8 : MC_PUB_KEY_SIZE;
    uint8_t frame[2 + 6 + MC_PUB_KEY_SIZE];
    frame[0] = (MC_PAYLOAD_TYPE_CONTROL << 2) | MC_ROUTE_DIRECT;
    frame[1] = 0x00; // zero hop
    frame[2] = MC_CTL_DISCOVER_RESP | nodeType;
    frame[3] = (uint8_t)(int8_t)(snr * 4);
    memcpy(&frame[4], &req[2], 4);
    memcpy(&frame[8], identityPub, keyLen);

    // Several repeaters answer the same request: JenCore widens its random delay x4 to spread them out.
    uint32_t spread = 4 * (uint32_t)(JITTER_AIRTIME_MULT * 0.5f * (float)airtimeMs);
    if (spread > 4 * JITTER_CAP_MS)
        spread = 4 * JITTER_CAP_MS;
    if (queueOwnFrame(frame, 8 + keyLen, DELAY_FLOOR_MS + (uint32_t)random(0, (long)spread + 1))) {
        LOG_INFO("JenCore discover reply queued");
        discoverRepliesInWindow++;
        discoverReplies++;
    }
}
#endif

uint32_t JenCoreRepeater::hashFrame(const uint8_t *raw, size_t len, const FrameInfo &info)
{
    // FNV-1a over payload_type + payload ONLY. JenCore appends a hop hash to the path at every
    // forward (Mesh.cpp::routeRecvPacket), so a full-frame hash would make every path variant of
    // the same packet look new and the relay would re-broadcast each of them. Keying on the
    // payload dedupes variants, like JenCore's own Packet::calculatePacketHash.
    uint32_t hash = 2166136261u;
    hash = (hash ^ info.payload_type) * 16777619u;
    if (info.payload_type == MC_PAYLOAD_TYPE_TRACE) {
        // A TRACE grows its path with SNRs at every hop and may pass the same node again on the way back
        // (Packet::calculatePacketHash), so the path length is part of its identity.
        const size_t pathLenAt = info.header_len - (size_t)info.path_hash_count * info.path_hash_size - 1;
        hash = (hash ^ raw[pathLenAt]) * 16777619u;
    }
    for (size_t i = info.header_len; i < len; i++) {
        hash = (hash ^ raw[i]) * 16777619u;
    }
    return hash ? hash : 1; // 0 marks an empty dedupe slot
}

bool JenCoreRepeater::wasRecentlySeen(uint32_t hash) const
{
    return dedup.contains(hash);
}

void JenCoreRepeater::rememberFrame(uint32_t hash)
{
    dedup.remember(hash);
}

bool JenCoreRepeater::isFloodForwardable(const FrameInfo &info)
{
    switch (info.payload_type) {
    case MC_PAYLOAD_TYPE_ACK:
        return info.payload_len >= 4; // ack crc
    case MC_PAYLOAD_TYPE_REQ:
    case MC_PAYLOAD_TYPE_RESPONSE:
    case MC_PAYLOAD_TYPE_TXT_MSG:
    case MC_PAYLOAD_TYPE_PATH:
        return info.payload_len > 2 + 2; // dest + src hash, then more than the 2-byte MAC
    case MC_PAYLOAD_TYPE_ANON_REQ:
        return info.payload_len > 1 + MC_PUB_KEY_SIZE + 2; // dest hash + sender key, then more than the MAC
    case MC_PAYLOAD_TYPE_GRP_TXT:
    case MC_PAYLOAD_TYPE_GRP_DATA:
        return info.payload_len > 1 + 2; // channel hash, then more than the MAC
    case MC_PAYLOAD_TYPE_ADVERT:
        return info.payload_len >= MC_PUB_KEY_SIZE + 4 + MC_SIGNATURE_SIZE; // key, timestamp, signature
    default:
        return false;
    }
}

void JenCoreRepeater::onDupeHeard(uint32_t hash)
{
    // Someone else is retransmitting a frame we have queued: contention, measured on the air.
    // Postpone our copy (ZephCore's extendPendingRetransmit) and keep the dupe as the contention
    // sample this relay will fold into the estimate.
    for (QueuedFrame &f : queue) {
        if (f.len == 0 || f.own || f.hash32 != hash)
            continue;
        if (!Throttle::isWithinTimespanMs(f.firstSeenMs, DUPE_WINDOW_MS))
            return;

        if (f.dupeCount < 255)
            f.dupeCount++;

        uint32_t perDupe = (uint32_t)(REACTIVE_BACKOFF_AIRTIME * (float)f.airtimeMs);
        uint32_t cap = (uint32_t)(REACTIVE_AIRTIME_CAP * (float)f.airtimeMs);
        if (cap > REACTIVE_HARD_CAP_MS)
            cap = REACTIVE_HARD_CAP_MS;
        if (f.reactiveAddedMs >= cap)
            return;
        uint32_t remaining = cap - f.reactiveAddedMs;
        if (perDupe > remaining)
            perDupe = remaining;
        if (perDupe == 0)
            return;

        f.reactiveAddedMs += perDupe;
        f.deadlineMs += perDupe;
        return;
    }
}

bool JenCoreRepeater::canRelay() const
{
    // Same rebroadcast policy as Meshtastic's own relay (FloodingRouter::isRebroadcaster): a mute
    // node or a node with rebroadcast disabled relays nothing, JenCore floods included.
    return config.device.role != meshtastic_Config_DeviceConfig_Role_CLIENT_MUTE &&
           config.device.rebroadcast_mode != meshtastic_Config_DeviceConfig_RebroadcastMode_NONE;
}

int JenCoreRepeater::claimSlot(uint8_t priority)
{
    for (size_t i = 0; i < QUEUE_SIZE; i++) {
        if (queue[i].len == 0)
            return (int)i;
    }

    // Full: the worst queued flood gives way to a better frame (ZephCore: "if the new one is more urgent, expel the
    // worst"). Among equals the one due last goes. Routed traffic and our own frames are never evicted.
    int victim = -1;
    for (size_t i = 0; i < QUEUE_SIZE; i++) {
        const QueuedFrame &f = queue[i];
        if (f.own || f.priority <= priority)
            continue;
        if (victim < 0 || f.priority > queue[victim].priority ||
            (f.priority == queue[victim].priority && (int32_t)(f.deadlineMs - queue[victim].deadlineMs) > 0))
            victim = (int)i;
    }
    if (victim < 0)
        return -1;

    // It was remembered when queued, but never forwarded: a later copy must be eligible again.
    dedup.forget(queue[victim].hash32);
    release(victim);
    txDropped++;
    return victim;
}

bool JenCoreRepeater::queueRelay(const uint8_t *raw, size_t len, uint32_t hash32, uint32_t airtimeMs, uint32_t delayMs,
                                  uint8_t priority, bool needsVerify)
{
    if (len == 0 || len > MAX_FRAME_LEN)
        return false;
    const int slot = claimSlot(priority);
    if (slot < 0) {
        LOG_DEBUG("JenCore relay queue full, drop");
        txDropped++;
        return false;
    }

    QueuedFrame &f = queue[slot];
    memcpy(f.data, raw, len);
    f.len = (uint8_t)len; // MAX_FRAME_LEN is 255
    f.own = false;
    f.needsVerify = needsVerify;
    f.priority = priority;
    f.dupeCount = 0;
    f.reactiveAddedMs = 0;
    f.hash32 = hash32;
    f.airtimeMs = airtimeMs;
    f.firstSeenMs = Time::getMillis();
    f.deadlineMs = f.firstSeenMs + delayMs;
    LOG_DEBUG("JenCore relay queued: len=%u delay=%ums prio=%u", (unsigned)len, (unsigned)delayMs, (unsigned)priority);
    return true;
}

void JenCoreRepeater::release(int index)
{
    queue[index].len = 0;
    if (selected == index)
        selected = -1;
    if (!hasPendingFrame())
        busyAttempts = 0;
}

void JenCoreRepeater::expireStale()
{
    for (size_t i = 0; i < QUEUE_SIZE; i++) {
        const QueuedFrame &f = queue[i];
        if (f.len != 0 && !f.own && !Throttle::isWithinTimespanMs(f.firstSeenMs, DUPE_WINDOW_MS)) {
            release((int)i);
            txDropped++;
        }
    }
}

bool JenCoreRepeater::hasPendingFrame() const
{
    for (const QueuedFrame &f : queue) {
        if (f.len != 0)
            return true;
    }
    return false;
}

bool JenCoreRepeater::hasPendingOwnFrame() const
{
    for (const QueuedFrame &f : queue) {
        if (f.len != 0 && f.own)
            return true;
    }
    return false;
}

void JenCoreRepeater::markRelayed()
{
    lastRelayMs = Time::getMillis();
    txRelayed++;
}

void JenCoreRepeater::afterTransmit(uint32_t slotMs)
{
    const uint32_t slot = slotMs < MIN_SLOT_MS ? MIN_SLOT_MS : slotMs;
    holdoffUntilMs = Time::getMillis() + (uint32_t)random(1, HOLDOFF_MAX_SLOTS + 1) * slot;
    holdoffSet = true;
    busyAttempts = 0;
}

uint32_t JenCoreRepeater::busyBackoffMs(uint32_t slotMs)
{
    const uint32_t slot = slotMs < MIN_SLOT_MS ? MIN_SLOT_MS : slotMs;
    if (busyAttempts < BUSY_BACKOFF_MAX_EXP)
        busyAttempts++;
    const long window = 1L << busyAttempts; // 2, 4, 8, 16 slots
    return (uint32_t)random(1, window + 1) * slot;
}

int JenCoreRepeater::pickNext() const
{
    int best = -1;
    bool bestDue = false;
    for (size_t i = 0; i < QUEUE_SIZE; i++) {
        const QueuedFrame &f = queue[i];
        if (f.len == 0)
            continue;
        const bool due = Throttle::deadlinePassed(f.deadlineMs);
        bool better;
        if (best < 0)
            better = true;
        else if (due != bestDue)
            better = due; // a due frame beats one that is not
        else if (due && f.priority != queue[best].priority)
            better = f.priority < queue[best].priority;
        else
            better = (int32_t)(f.deadlineMs - queue[best].deadlineMs) < 0;
        if (better) {
            best = (int)i;
            bestDue = due;
        }
    }
    return best;
}

bool JenCoreRepeater::isRelayDue() const
{
    if (holdoffSet && !Throttle::deadlinePassed(holdoffUntilMs))
        return false;
    for (const QueuedFrame &f : queue) {
        if (f.len != 0 && Throttle::deadlinePassed(f.deadlineMs))
            return true;
    }
    return false;
}

uint32_t JenCoreRepeater::relayDueInMs() const
{
    const uint32_t now = Time::getMillis();
    bool any = false;
    uint32_t soonest = 0;
    for (const QueuedFrame &f : queue) {
        if (f.len == 0)
            continue;
        const uint32_t wait = Throttle::deadlinePassed(f.deadlineMs) ? 0 : f.deadlineMs - now;
        if (!any || wait < soonest)
            soonest = wait;
        any = true;
    }
    if (!any)
        return 0;
    if (holdoffSet && !Throttle::deadlinePassed(holdoffUntilMs)) {
        const uint32_t holdoff = holdoffUntilMs - now;
        if (holdoff > soonest)
            soonest = holdoff;
    }
    return soonest;
}

size_t JenCoreRepeater::takePendingFrame(uint8_t *buf, size_t bufLen)
{
    for (int pick = pickNext(); pick >= 0; pick = pickNext()) {
        QueuedFrame &f = queue[pick];
        if (f.needsVerify) {
            FrameInfo info;
            if (!parse(f.data, f.len, &info) || !advertSignatureValid(&f.data[info.header_len], info.payload_len)) {
                LOG_DEBUG("JenCore: advert with a bad signature, drop");
                release(pick);
                rxDropped++;
                continue;
            }
            f.needsVerify = false;
        }
        if (bufLen < f.len)
            return 0;
        memcpy(buf, f.data, f.len);
        selected = pick;
        selectedHash = f.hash32;
        return f.len;
    }
    return 0;
}

int JenCoreRepeater::selectedIndex() const
{
    if (selectedHash == 0)
        return pickNext(); // nothing taken: the frame that would be taken next
    if (selected >= 0 && queue[selected].len != 0 && queue[selected].hash32 == selectedHash)
        return selected;
    for (size_t i = 0; i < QUEUE_SIZE; i++) {
        if (queue[i].len != 0 && queue[i].hash32 == selectedHash)
            return (int)i;
    }
    return -1;
}

bool JenCoreRepeater::pendingIsOwn() const
{
    const int pick = selectedIndex();
    return pick >= 0 && queue[pick].own;
}

void JenCoreRepeater::clearPendingFrame()
{
    const int pick = selectedIndex();
    selected = -1;
    selectedHash = 0;
    if (pick < 0)
        return;
    if (!queue[pick].own)
        foldContentionSample(queue[pick]);
    release(pick);
}

void JenCoreRepeater::foldContentionSample(const QueuedFrame &f)
{
    // Same fold as ZephCore ContentionTracker::finalizeEntry: the dupe count of this relay is
    // one contention sample, folded into an EMA (alpha 1/8; fast convergence during warmup).
    uint32_t sampleX256 = (uint32_t)f.dupeCount << 8;
    int32_t diff = (int32_t)sampleX256 - (int32_t)emaX256;
    if (finalizedSamples < WARMUP_SAMPLES) {
        if (finalizedSamples == 0)
            emaX256 = sampleX256;
        else
            emaX256 = (uint32_t)((int32_t)emaX256 + (diff >> 1));
    } else {
        emaX256 = (uint32_t)((int32_t)emaX256 + (diff >> EMA_SHIFT));
    }
    finalizedSamples++;
}

void JenCoreRepeater::applyLazyDecay()
{
    // ZephCore decays its estimate when the mesh goes quiet (STALE_MS), paced by a maintenance
    // tick. Here the same steps are computed lazily from the timestamps: no timers at all.
    const uint32_t now = Time::getMillis();
    if (lastRelayMs == 0 || emaX256 == 0)
        return;
    if (!Throttle::hasElapsed(lastRelayMs, STALE_MS)) {
        lastDecayMs = 0;
        return;
    }
    if (lastDecayMs == 0)
        lastDecayMs = now - DECAY_PERIOD_MS; // arm so the first step lands on this call

    for (uint32_t n = 0; n < MAX_DECAY_CATCHUP && emaX256 != 0; n++) {
        if (!Throttle::hasElapsed(lastDecayMs, DECAY_PERIOD_MS))
            break;
        emaX256 -= emaX256 >> EMA_SHIFT;
        lastDecayMs += DECAY_PERIOD_MS;
    }
}

float JenCoreRepeater::getFloodDelayFactor()
{
    applyLazyDecay();

    if (finalizedSamples < WARMUP_SAMPLES)
        return 0.5f; // conservative default before the estimate is trusted

    float est = (float)emaX256 / 256.0f;
    if (est <= 0.0f)
        return MIN_FLOOD_FACTOR;

    // Factor rises smoothly from MIN toward MAX as observed contention grows (0.60 at est = 4).
    float factor = MIN_FLOOD_FACTOR + (MAX_FLOOD_FACTOR - MIN_FLOOD_FACTOR) * (est / (est + FLOOD_EST_HALFPOINT));
    if (factor > MAX_FLOOD_FACTOR)
        factor = MAX_FLOOD_FACTOR;
    return factor;
}

uint32_t JenCoreRepeater::adaptiveDelayMs(uint32_t airtimeMs)
{
    // ZephCore Mesh::computeAdaptiveFloodDelay: 20ms floor (neighbours are back in RX) plus a
    // random jitter of up to 5 x airtime x factor, capped at 6 x airtime and 2000ms.
    float factor = getFloodDelayFactor();
    uint32_t jitter = (uint32_t)(JITTER_AIRTIME_MULT * (float)airtimeMs * factor);
    uint32_t airtimeCap = (uint32_t)(JITTER_AIRTIME_CAP * (float)airtimeMs);
    if (jitter > airtimeCap)
        jitter = airtimeCap;
    if (jitter > JITTER_CAP_MS)
        jitter = JITTER_CAP_MS;
    return DELAY_FLOOR_MS + (uint32_t)random(0, (long)jitter + 1);
}

bool JenCoreRepeater::setIdentitySeed(const uint8_t seed[32])
{
    uint8_t pub[MC_PUB_KEY_SIZE];
    Ed25519::derivePublicKey(pub, seed);
    // 0x00 and 0xFF are reserved 1-byte path hashes: JenCore will not use such a key (Identity::validatePrivateKey).
    if (pub[0] == 0x00 || pub[0] == 0xFF)
        return false;
    memcpy(identitySeed, seed, sizeof(identitySeed));
    memcpy(identityPub, pub, sizeof(identityPub));
    hasIdentityKey = true;
    return true;
}

size_t JenCoreRepeater::buildAdvert(uint8_t *out, size_t outLen, const char *name, uint32_t timestamp, bool flood)
{
    if (!hasIdentityKey)
        return 0;

    // App data: type byte (our node type, with the name flag when there is a name), then the UTF-8 name.
    uint8_t appData[MAX_ADVERT_DATA_SIZE];
    size_t appLen = 1;
    appData[0] = nodeType;
    if (name && name[0] != 0) {
        size_t nameLen = strlen(name);
        if (nameLen > MAX_ADVERT_DATA_SIZE - 1)
            nameLen = MAX_ADVERT_DATA_SIZE - 1;
        // Never cut a multi-byte character in half: back off to a UTF-8 lead byte.
        while (nameLen > 0 && nameLen < strlen(name) && ((uint8_t)name[nameLen] & 0xC0) == 0x80)
            nameLen--;
        appData[0] |= MC_ADV_NAME_MASK;
        memcpy(&appData[appLen], name, nameLen);
        appLen += nameLen;
    }

    const size_t total = 2 + MC_PUB_KEY_SIZE + 4 + MC_SIGNATURE_SIZE + appLen;
    if (outLen < total)
        return 0;

    // Header: payload type ADVERT, version 0. Flood, or zero-hop (route DIRECT with an empty path, Mesh::sendZeroHop).
    out[0] = (MC_PAYLOAD_TYPE_ADVERT << 2) | (flood ? MC_ROUTE_FLOOD : MC_ROUTE_DIRECT);
    out[1] = flood ? ownPathLenByte() : 0x00;
    size_t i = 2;
    memcpy(&out[i], identityPub, MC_PUB_KEY_SIZE);
    i += MC_PUB_KEY_SIZE;
    memcpy(&out[i], &timestamp, 4); // little-endian, as JenCore writes it
    i += 4;
    uint8_t *signature = &out[i];
    i += MC_SIGNATURE_SIZE;
    memcpy(&out[i], appData, appLen);

    // Signed message: pub || timestamp || app_data, the same bytes JenCore verifies.
    uint8_t message[MC_PUB_KEY_SIZE + 4 + MAX_ADVERT_DATA_SIZE];
    size_t msgLen = 0;
    memcpy(&message[msgLen], identityPub, MC_PUB_KEY_SIZE);
    msgLen += MC_PUB_KEY_SIZE;
    memcpy(&message[msgLen], &timestamp, 4);
    msgLen += 4;
    memcpy(&message[msgLen], appData, appLen);
    msgLen += appLen;
    Ed25519::sign(signature, identitySeed, identityPub, message, msgLen);

    return total;
}

bool JenCoreRepeater::queueOwnFrame(const uint8_t *raw, size_t len, uint32_t delayMs)
{
    if (len == 0 || len > MAX_FRAME_LEN)
        return false;

    FrameInfo info;
    if (!parse(raw, len, &info))
        return false;

    uint8_t scoped[MAX_FRAME_LEN + 4];
    size_t framedLen = len;
    const uint8_t *framed = scopeOwnFrame(raw, len, scoped, sizeof(scoped), &framedLen);
    if (!framed || framedLen > MAX_FRAME_LEN)
        return false;
    if (framed != raw)
        parse(framed, framedLen, &info);
    const uint32_t hash = hashFrame(raw, len, info);

    const int slot = claimSlot(PRIORITY_OWN);
    if (slot < 0)
        return false;

    QueuedFrame &f = queue[slot];
    memcpy(f.data, framed, framedLen);
    f.len = (uint8_t)framedLen;
    f.own = true;
    f.needsVerify = false;
    f.priority = PRIORITY_OWN;
    f.dupeCount = 0;
    f.reactiveAddedMs = 0;
    f.hash32 = hash;
    f.airtimeMs = 0;
    f.firstSeenMs = Time::getMillis();
    f.deadlineMs = f.firstSeenMs + delayMs;
    rememberFrame(hash); // a neighbour re-flooding our own ADVERT must not make us relay it again
    return true;
}

#endif // NEXUS_ENABLE_JENCORE_SNIFFER
