// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) 2026 Jennifer Sanchez
//
// Licensed under the GNU Affero General Public License v3.
// See LICENSE-AGPL for the full license text.
//

#include "JenCoreCompanion.h"

#if NEXUS_ENABLE_JENCORE_COMPANION

#ifdef CRYPTO_AES_NO_DECRYPT
#error "JenCore needs AES decryption: add -UCRYPTO_AES_NO_DECRYPT to this env (the nRF52 base defines it)"
#endif

#include "Channels.h"
#include "JenCoreMeta.h"
#include "NodeDB.h"
#include "RadioInterface.h"
#include "Router.h"
#include "Throttle.h"
#include "TypeConversions.h"
#include "gps/RTC.h"
#include "mesh-pb-constants.h"
#include "meshUtils.h"
#include <AES.h>
#include <Curve25519.h>
#include <SHA256.h>
#include <SHA512.h>
#include <string.h>
#include <strings.h>

#if !(MESHTASTIC_EXCLUDE_XEDDSA)
#include "XEdDSA.h" // fe_* field arithmetic for the Ed25519 -> X25519 public key map
#endif

JenCoreCompanion jencoreCompanion;

static size_t utf8PrefixLen(const char *s, size_t maxBytes)
{
    if (!s)
        return 0;
    size_t len = strlen(s);
    if (len <= maxBytes)
        return len;
    size_t n = maxBytes;
    while (n > 0 && ((uint8_t)s[n] & 0xC0) == 0x80)
        n--; // do not split a multi-byte character
    return n;
}

static size_t aesEncrypt(const uint8_t secret[32], const uint8_t *src, size_t len, uint8_t *dst)
{
    AES128 aes;
    if (!aes.setKey(secret, 16))
        return 0;

    size_t i = 0;
    for (; i + 16 <= len; i += 16)
        aes.encryptBlock(&dst[i], &src[i]);
    if (i < len) {
        uint8_t block[16] = {};
        memcpy(block, &src[i], len - i);
        aes.encryptBlock(&dst[i], block);
        i += 16;
    }
    return i;
}

static size_t aesDecrypt(const uint8_t secret[32], const uint8_t *src, size_t len, uint8_t *dst)
{
    if (len == 0 || len % 16 != 0)
        return 0;
    AES128 aes;
    if (!aes.setKey(secret, 16))
        return 0;
    for (size_t i = 0; i < len; i += 16)
        aes.decryptBlock(&dst[i], &src[i]);
    return len;
}

static uint32_t readLe32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void writeLe32(uint8_t *p, uint32_t v)
{
    p[0] = v & 0xFF;
    p[1] = (v >> 8) & 0xFF;
    p[2] = (v >> 16) & 0xFF;
    p[3] = (v >> 24) & 0xFF;
}

size_t JenCoreCompanion::normalizedKeyLen(const uint8_t *key, size_t keyLen)
{
    if (!key || (keyLen != 16 && keyLen != 32))
        return 0;
    if (keyLen == 32) {
        uint8_t tail = 0;
        for (size_t i = 16; i < 32; i++)
            tail |= key[i];
        if (tail == 0)
            return 16;
    }
    return keyLen;
}

bool JenCoreCompanion::channelHash(const uint8_t *key, size_t keyLen, uint8_t *out)
{
    size_t len = normalizedKeyLen(key, keyLen);
    if (len == 0 || !out)
        return false;

    SHA256 sha;
    uint8_t digest[32];
    sha.reset();
    sha.update(key, len);
    sha.finalize(digest, sizeof(digest));
    *out = digest[0];
    return true;
}

size_t JenCoreCompanion::encodeGroupText(const uint8_t *key, size_t keyLen, const char *sender, uint32_t timestamp,
                                          const char *text, uint8_t *out, size_t outCap, bool *truncated)
{
    size_t len = normalizedKeyLen(key, keyLen);
    if (len == 0 || !out)
        return 0;

    uint8_t secret[32] = {};
    memcpy(secret, key, len);

    size_t senderLen = utf8PrefixLen(sender, MAX_TEXT_LEN - 2);
    size_t prefixLen = senderLen + 2; // ": "
    size_t textLen = utf8PrefixLen(text, MAX_TEXT_LEN - prefixLen);
    if (truncated)
        *truncated = text && textLen < strlen(text);

    uint8_t plain[5 + MAX_TEXT_LEN];
    writeLe32(plain, timestamp);
    plain[4] = 0; // TXT_TYPE_PLAIN
    if (senderLen)
        memcpy(&plain[5], sender, senderLen);
    plain[5 + senderLen] = ':';
    plain[5 + senderLen + 1] = ' ';
    if (textLen)
        memcpy(&plain[5 + prefixLen], text, textLen);

    const size_t plainLen = 5 + prefixLen + textLen;
    uint8_t cipher[MAX_CIPHER_LEN];
    const size_t cipherLen = aesEncrypt(secret, plain, plainLen, cipher);
    if (cipherLen == 0 || 5 + cipherLen > outCap)
        return 0;

    uint8_t hash;
    if (!channelHash(key, keyLen, &hash))
        return 0;

    out[0] = (0x05 << 2) | 0x01;                        // GRP_TXT, ROUTE_FLOOD
    out[1] = JenCoreRepeater::ownPathLenByte(); // zero hops, 2-byte hashes
    out[2] = hash;

    SHA256 hmac;
    hmac.resetHMAC(secret, sizeof(secret));
    hmac.update(cipher, cipherLen);
    hmac.finalizeHMAC(secret, sizeof(secret), &out[3], 2);

    memcpy(&out[5], cipher, cipherLen);
    return 5 + cipherLen;
}

bool JenCoreCompanion::decodeGroupText(const uint8_t *key, size_t keyLen, const uint8_t *frame, size_t len,
                                        const JenCoreRepeater::FrameInfo &info, char *textOut, size_t textOutCap,
                                        uint32_t *timestampOut)
{
    size_t keySize = normalizedKeyLen(key, keyLen);
    if (keySize == 0 || !frame || info.payload_type != 0x05 || info.header_len >= len)
        return false;
    if (info.payload_len < 1 + 2 + 16)
        return false;

    const uint8_t *payload = &frame[info.header_len];
    uint8_t hash;
    if (!channelHash(key, keyLen, &hash) || payload[0] != hash)
        return false;

    const size_t cipherLen = info.payload_len - 3;
    if (cipherLen % 16 != 0 || cipherLen > MAX_CIPHER_LEN)
        return false;

    uint8_t secret[32] = {};
    memcpy(secret, key, keySize);

    uint8_t mac[2];
    SHA256 hmac;
    hmac.resetHMAC(secret, sizeof(secret));
    hmac.update(&payload[3], cipherLen);
    hmac.finalizeHMAC(secret, sizeof(secret), mac, sizeof(mac));
    if (memcmp(mac, &payload[1], sizeof(mac)) != 0)
        return false;

    uint8_t plain[MAX_CIPHER_LEN];
    const size_t plainLen = aesDecrypt(secret, &payload[3], cipherLen, plain);
    if (plainLen < 5 || (plain[4] >> 2) != 0) // only TXT_TYPE_PLAIN
        return false;

    if (textOut && textOutCap) {
        size_t bodyLen = plainLen - 5;
        size_t end = 0;
        while (end < bodyLen && plain[5 + end] != 0)
            end++;
        size_t copyLen = end < textOutCap ? end : textOutCap - 1;
        memcpy(textOut, &plain[5], copyLen);
        textOut[copyLen] = 0;
        sanitizeUtf8(textOut, textOutCap);
    }
    if (timestampOut)
        *timestampOut = readLe32(plain);
    return true;
}

bool JenCoreCompanion::sharedSecret(const uint8_t seed[32], const uint8_t peerPub[32], uint8_t out[32])
{
#if !(MESHTASTIC_EXCLUDE_XEDDSA)
    if (!seed || !peerPub || !out)
        return false;

    // Peer: Montgomery u = (1 + y) / (1 - y) from the Edwards y in the public key.
    fe y, one, num, den;
    fe_frombytes(y, peerPub);
    fe_1(one);
    fe_add(num, one, y);
    fe_sub(den, one, y);
    fe_invert(den, den);
    fe_mul(num, num, den);
    uint8_t u[32];
    fe_tobytes(u, num);

    // Us: the Ed25519 scalar is the clamped first half of SHA-512(seed).
    SHA512 sha;
    uint8_t digest[64];
    sha.reset();
    sha.update(seed, 32);
    sha.finalize(digest, sizeof(digest));
    digest[0] &= 248;
    digest[31] &= 63;
    digest[31] |= 64;

    const bool ok = Curve25519::eval(out, digest, u) && !Curve25519::isWeakPoint(out);
    memset(digest, 0, sizeof(digest));
    return ok;
#else
    return false;
#endif
}

// JenCore datagram (TXT_MSG, PATH): dest hash | src hash | MAC(2) | AES-128 blocks, MAC = HMAC-SHA256 over the cipher.
static size_t sealDatagram(const uint8_t secret[32], uint8_t payloadType, uint8_t destHash, uint8_t srcHash, const uint8_t *plain,
                           size_t plainLen, uint8_t *out, size_t outCap)
{
    uint8_t cipher[JenCoreCompanion::MAX_CIPHER_LEN];
    if (plainLen > sizeof(cipher))
        return 0;
    const size_t cipherLen = aesEncrypt(secret, plain, plainLen, cipher);
    if (cipherLen == 0 || 6 + cipherLen > outCap)
        return 0;

    out[0] = (payloadType << 2) | 0x01;                 // ROUTE_FLOOD
    out[1] = JenCoreRepeater::ownPathLenByte(); // zero hops, 2-byte hashes
    out[2] = destHash;
    out[3] = srcHash;

    SHA256 hmac;
    hmac.resetHMAC(secret, 32);
    hmac.update(cipher, cipherLen);
    hmac.finalizeHMAC(secret, 32, &out[4], 2);

    memcpy(&out[6], cipher, cipherLen);
    return 6 + cipherLen;
}

/// Verify the MAC of a datagram payload and decrypt it into plain (MAX_CIPHER_LEN bytes). False on any mismatch.
static bool openDatagram(const uint8_t secret[32], uint8_t payloadType, const uint8_t *frame, size_t len,
                         const JenCoreRepeater::FrameInfo &info, uint8_t *plain, size_t *plainLen)
{
    if (!secret || !frame || info.payload_type != payloadType || info.header_len >= len)
        return false;
    if (info.payload_len < 1 + 1 + 2 + 16)
        return false;

    const uint8_t *payload = &frame[info.header_len];
    const size_t cipherLen = info.payload_len - 4;
    if (cipherLen % 16 != 0 || cipherLen > JenCoreCompanion::MAX_CIPHER_LEN)
        return false;

    uint8_t mac[2];
    SHA256 hmac;
    hmac.resetHMAC(secret, 32);
    hmac.update(&payload[4], cipherLen);
    hmac.finalizeHMAC(secret, 32, mac, sizeof(mac));
    if (memcmp(mac, &payload[2], sizeof(mac)) != 0)
        return false;

    *plainLen = aesDecrypt(secret, &payload[4], cipherLen, plain);
    return *plainLen != 0;
}

size_t JenCoreCompanion::encodeDirectText(const uint8_t secret[32], uint8_t destHash, uint8_t srcHash, uint32_t timestamp,
                                           uint8_t flags, const char *text, uint8_t *out, size_t outCap, bool *truncated)
{
    if (!secret || !out)
        return 0;

    const size_t textLen = utf8PrefixLen(text, MAX_TEXT_LEN);
    if (truncated)
        *truncated = text && textLen < strlen(text);

    uint8_t plain[5 + MAX_TEXT_LEN];
    writeLe32(plain, timestamp);
    plain[4] = flags;
    if (textLen)
        memcpy(&plain[5], text, textLen);

    return sealDatagram(secret, 0x02, destHash, srcHash, plain, 5 + textLen, out, outCap); // TXT_MSG
}

bool JenCoreCompanion::decodeDirectText(const uint8_t secret[32], const uint8_t *frame, size_t len,
                                         const JenCoreRepeater::FrameInfo &info, char *textOut, size_t textOutCap,
                                         uint32_t *timestampOut, uint8_t *flagsOut)
{
    uint8_t plain[MAX_DIRECT_CIPHER_LEN];
    size_t plainLen = 0;
    if (!openDatagram(secret, 0x02, frame, len, info, plain, &plainLen) || plainLen < 5 || (plain[4] >> 2) != TXT_FLAGS_PLAIN)
        return false;

    if (textOut && textOutCap) {
        size_t end = 0;
        while (end < plainLen - 5 && plain[5 + end] != 0)
            end++;
        size_t copyLen = end < textOutCap ? end : textOutCap - 1;
        memcpy(textOut, &plain[5], copyLen);
        textOut[copyLen] = 0;
        sanitizeUtf8(textOut, textOutCap);
    }
    if (timestampOut)
        *timestampOut = readLe32(plain);
    if (flagsOut)
        *flagsOut = plain[4];
    return true;
}

size_t JenCoreCompanion::encodePathAck(const uint8_t secret[32], uint8_t destHash, uint8_t srcHash, uint32_t hash, uint8_t *out,
                                        size_t outCap)
{
    if (!secret || !out)
        return 0;
    // Mesh::createPathReturn: the path walked so far (none), then an extra block: its type and the ACK hash.
    uint8_t plain[6];
    plain[0] = 0x00; // path_len: 1-byte hashes, no hops
    plain[1] = 0x03; // extra type: ACK
    writeLe32(&plain[2], hash);
    return sealDatagram(secret, 0x08, destHash, srcHash, plain, sizeof(plain), out, outCap); // PATH
}

bool JenCoreCompanion::decodePathAck(const uint8_t secret[32], const uint8_t *frame, size_t len,
                                      const JenCoreRepeater::FrameInfo &info, uint32_t *hashOut)
{
    uint8_t plain[MAX_CIPHER_LEN];
    size_t plainLen = 0;
    if (!hashOut || !openDatagram(secret, 0x08, frame, len, info, plain, &plainLen))
        return false;

    // path_len (hash size in the top two bits, hop count in the rest), the path, then the extra block.
    size_t at = 0;
    const uint8_t pathLenByte = plain[at++];
    at += (size_t)((pathLenByte >> 6) + 1) * (pathLenByte & 0x3F);
    if (at >= plainLen || (plain[at++] & 0x0F) != 0x03 || plainLen - at < 4) // the extra must be an ACK
        return false;
    *hashOut = readLe32(&plain[at]);
    return true;
}

uint32_t JenCoreCompanion::ackHash(uint32_t timestamp, uint8_t flags, const char *text, const uint8_t senderPub[32])
{
    uint8_t head[5];
    writeLe32(head, timestamp);
    head[4] = flags;

    SHA256 sha;
    uint8_t digest[32];
    sha.reset();
    sha.update(head, sizeof(head));
    if (text)
        sha.update(text, strlen(text));
    sha.update(senderPub, 32);
    sha.finalize(digest, sizeof(digest));
    return readLe32(digest);
}

size_t JenCoreCompanion::encodeAck(uint32_t hash, uint8_t *out, size_t outCap)
{
    if (!out || outCap < 6)
        return 0;
    out[0] = (0x03 << 2) | 0x01; // ACK, ROUTE_FLOOD
    out[1] = JenCoreRepeater::ownPathLenByte();
    writeLe32(&out[2], hash);
    return 6;
}

void JenCoreCompanion::announceUser(uint32_t nodeNum, const meshtastic_User &user)
{
    if (!router)
        return;
    meshtastic_MeshPacket *p = packetPool.allocZeroed();
    if (!p)
        return;

    p->from = nodeNum;
    p->to = NODENUM_BROADCAST;
    p->id = generatePacketId();
    p->channel = 0;
    p->which_payload_variant = meshtastic_MeshPacket_decoded_tag;
    p->decoded.portnum = meshtastic_PortNum_NODEINFO_APP;
    p->decoded.payload.size =
        pb_encode_to_bytes(p->decoded.payload.bytes, sizeof(p->decoded.payload.bytes), &meshtastic_User_msg, &user);
    // Local news for the client, never to be repeated over Meshtastic: a next hop that is not us closes the relay gate. The
    // router drops a packet whose hop_start is missing, so it carries the same hop fields as a bridged text one hop away.
    p->decoded.has_bitfield = true;
    p->decoded.bitfield = 0;
    hopFields(1, &p->hop_start, &p->hop_limit);
    const uint8_t ourRelay = nodeDB->getLastByteOfNodeNum(nodeDB->getNodeNum());
    p->next_hop = (uint8_t)(ourRelay + 1);
    if (p->next_hop == NO_NEXT_HOP_PREFERENCE)
        p->next_hop = 1;
    p->transport_mechanism = meshtastic_MeshPacket_TransportMechanism_TRANSPORT_LORA;
    router->enqueueReceivedMessage(p);
}

void JenCoreCompanion::announcePosition(uint32_t nodeNum, int32_t latitudeI, int32_t longitudeI)
{
    if (!router)
        return;
    meshtastic_MeshPacket *p = packetPool.allocZeroed();
    if (!p)
        return;

    meshtastic_Position pos = meshtastic_Position_init_default;
    pos.has_latitude_i = true;
    pos.latitude_i = latitudeI;
    pos.has_longitude_i = true;
    pos.longitude_i = longitudeI;
    pos.location_source = meshtastic_Position_LocSource_LOC_MANUAL;
    pos.time = getValidTime(RTCQualityFromNet);

    p->from = nodeNum;
    p->to = NODENUM_BROADCAST;
    p->id = generatePacketId();
    p->channel = 0;
    p->which_payload_variant = meshtastic_MeshPacket_decoded_tag;
    p->decoded.portnum = meshtastic_PortNum_POSITION_APP;
    p->decoded.payload.size =
        pb_encode_to_bytes(p->decoded.payload.bytes, sizeof(p->decoded.payload.bytes), &meshtastic_Position_msg, &pos);
    p->decoded.has_bitfield = true;
    p->decoded.bitfield = 0;
    hopFields(1, &p->hop_start, &p->hop_limit);
    const uint8_t ourRelay = nodeDB->getLastByteOfNodeNum(nodeDB->getNodeNum());
    p->next_hop = (uint8_t)(ourRelay + 1);
    if (p->next_hop == NO_NEXT_HOP_PREFERENCE)
        p->next_hop = 1;
    p->transport_mechanism = meshtastic_MeshPacket_TransportMechanism_TRANSPORT_LORA;
    router->enqueueReceivedMessage(p);
}

void JenCoreCompanion::updatePosition(uint32_t nodeNum, int32_t latMicro, int32_t lonMicro)
{
    if (!nodeDB || (latMicro == 0 && lonMicro == 0))
        return;
    // JenCore sends degrees x 1e6, Meshtastic keeps 1e-7.
    const int32_t latI = latMicro * 10, lonI = lonMicro * 10;
    meshtastic_PositionLite known;
    if (nodeDB->copyNodePosition(nodeNum, known) && known.latitude_i == latI && known.longitude_i == lonI)
        return; // adverts repeat every few minutes: only a move is news
    announcePosition(nodeNum, latI, lonI);
}

void JenCoreCompanion::ensurePseudoNode()
{
    if (!nodeDB)
        return;
    meshtastic_NodeInfoLite *lite = nodeDB->getOrCreateMeshNode(PSEUDO_NODE_NUM);
    if (!lite || nodeInfoLiteHasUser(lite))
        return;

    meshtastic_User user = {};
    strncpy(user.long_name, "JenCore", sizeof(user.long_name) - 1);
    strncpy(user.short_name, "JC", sizeof(user.short_name) - 1);
    user.hw_model = meshtastic_HardwareModel_UNSET;
    user.role = meshtastic_Config_DeviceConfig_Role_CLIENT;
    user.has_is_unmessagable = true;
    user.is_unmessagable = true;
    TypeConversions::CopyUserToNodeInfoLite(lite, user);
    announceUser(PSEUDO_NODE_NUM, user);
}

bool JenCoreCompanion::splitSender(const char *text, char *nameOut, size_t nameCap, const char **body)
{
    if (!text || !nameOut || nameCap == 0 || !body)
        return false;
    const char *sep = strstr(text, ": ");
    if (!sep || sep == text)
        return false;

    const size_t nameLen = sep - text;
    char name[MAX_TEXT_LEN + 1];
    memcpy(name, text, nameLen);
    name[nameLen] = 0;
    const size_t keep = utf8PrefixLen(name, nameCap - 1);
    memcpy(nameOut, name, keep);
    nameOut[keep] = 0;
    *body = sep + 2;
    return true;
}

uint32_t JenCoreCompanion::senderNodeNum(const char *name)
{
    const uint32_t num = JenCoreContacts<1>::nodeNumFor(name);
    return num == PSEUDO_NODE_NUM ? num + 1 : num;
}

void JenCoreCompanion::ensureSenderNode(uint32_t nodeNum, const char *name, bool messagable, const uint8_t *publicKey,
                                         meshtastic_Config_DeviceConfig_Role role)
{
    if (!nodeDB)
        return;
    meshtastic_NodeInfoLite *lite = nodeDB->getOrCreateMeshNode(nodeNum);
    // A real Meshtastic node that happens to share this number reports a hardware model: never rename it.
    if (lite && nodeInfoLiteHasUser(lite) && lite->hw_model != meshtastic_HardwareModel_UNSET)
        return;

    // A JenCore node shows up twice, as a Meshtastic node and as a JenCore one with the same name: the suffix tells them
    // apart.
    constexpr char suffix[] = " (JC)";
    char display[sizeof(((meshtastic_User *)nullptr)->long_name)];
    const size_t nameLen = utf8PrefixLen(name, sizeof(lite->long_name) - 1 - (sizeof(suffix) - 1));
    memcpy(display, name, nameLen);
    memcpy(&display[nameLen], suffix, sizeof(suffix));

    if (!lite ||
        (nodeInfoLiteHasUser(lite) && strncmp(lite->long_name, display, sizeof(lite->long_name) - 1) == 0 &&
         nodeInfoLiteIsUnmessagable(lite) == !messagable && lite->role == role && lite->public_key.size == (publicKey ? 32 : 0) &&
         (!publicKey || memcmp(lite->public_key.bytes, publicKey, 32) == 0)))
        return;

    meshtastic_User user = {};
    strncpy(user.long_name, display, sizeof(user.long_name) - 1);
    memcpy(user.short_name, name, utf8PrefixLen(name, 4));
    user.hw_model = meshtastic_HardwareModel_UNSET;
    user.role = role;
    user.has_is_unmessagable = true;
    user.is_unmessagable = !messagable;
    if (publicKey) {
        user.public_key.size = 32;
        memcpy(user.public_key.bytes, publicKey, 32);
    }
    TypeConversions::CopyUserToNodeInfoLite(lite, user);
    announceUser(nodeNum, user);
}

void JenCoreCompanion::hopFields(uint8_t pathHashCount, uint8_t *hopStart, uint8_t *hopLimit)
{
    const uint8_t hops = pathHashCount < HOP_START ? pathHashCount : HOP_START;
    *hopStart = HOP_START; // nonzero: a zero hop_start makes clients show '?' instead of the hop count
    *hopLimit = HOP_START - hops;
}

void JenCoreCompanion::noteRecentText(uint32_t packetId, const char *sender)
{
    RecentText &r = recent[recentNext];
    recentNext = (recentNext + 1) % RECENT_TEXTS;
    r.packetId = packetId;
    strncpy(r.sender, sender, sizeof(r.sender) - 1);
    r.sender[sizeof(r.sender) - 1] = 0;
}

const char *JenCoreCompanion::recentSender(uint32_t packetId) const
{
    for (const RecentText &r : recent)
        if (r.packetId == packetId && r.sender[0])
            return r.sender;
    return nullptr;
}

/// Length of name when text starts with it (ASCII case-insensitive) and the name ends there; 0 otherwise.
static size_t mentionMatchLen(const char *text, const char *name)
{
    const size_t n = strlen(name);
    if (n == 0 || strncasecmp(text, name, n) != 0)
        return 0;
    const uint8_t next = (uint8_t)text[n];
    const bool wordChar =
        (next >= '0' && next <= '9') || (next >= 'a' && next <= 'z') || (next >= 'A' && next <= 'Z') || next == '_';
    return wordChar ? 0 : n;
}

size_t JenCoreCompanion::prepareGroupText(const char *text, uint32_t replyId, char *out, size_t cap)
{
    if (!text || !out || cap == 0)
        return 0;

    size_t o = 0;
    auto put = [&](const char *s, size_t n) {
        const size_t room = cap - 1 - o;
        const size_t k = n < room ? n : room;
        memcpy(&out[o], s, k);
        o += k;
    };

    // JenCore shows a reply as "@[name] " in front of the text.
    const char *replied = replyId ? recentSender(replyId) : nullptr;
    if (replied) {
        put("@[", 2);
        put(replied, strlen(replied));
        put("] ", 2);
    }

    // JenCore marks a mention as "@[name]"; a bare "@name" is just text there. Names we know are matched whole (longest
    // first, so "Bob Builder" beats "Bob"); an unknown one is taken as a single word.
    for (const char *p = text; *p;) {
        const bool wordStart = p == text || p[-1] == ' ' || p[-1] == '\n';
        if (*p == '@' && p[1] != '[' && p[1] != ' ' && p[1] != 0 && wordStart) {
            char best[JenCoreContact::NAME_LEN] = {};
            size_t bestLen = 0;
            auto consider = [&](const char *name, size_t nameLen) {
                char known[JenCoreContact::NAME_LEN];
                if (nameLen == 0 || nameLen >= sizeof(known))
                    return;
                memcpy(known, name, nameLen);
                known[nameLen] = 0;
                const size_t m = mentionMatchLen(p + 1, known);
                if (m > bestLen) {
                    bestLen = m;
                    memcpy(best, known, nameLen + 1);
                }
            };
            contacts.forEach([&](JenCoreContact &c) { consider(c.name, strlen(c.name)); });
            for (const RecentText &r : recent)
                consider(r.sender, strlen(r.sender));
            // Nodes of JenCore people kept in the node DB (shown as "name (JC)" or legacy "name (MC)") outlive the recent-senders list.
            for (size_t n = 0; nodeDB && n < nodeDB->getNumMeshNodes(); n++) {
                const meshtastic_NodeInfoLite *lite = nodeDB->getMeshNodeByIndex(n);
                if ((lite->num >> 24) != 0x4D)
                    continue;
                const size_t len = strlen(lite->long_name);
                if (len > 5 && (strcmp(&lite->long_name[len - 5], " (JC)") == 0 ||
                                strcmp(&lite->long_name[len - 5], " (MC)") == 0))
                    consider(lite->long_name, len - 5);
            }
            if (bestLen > 0) {
                put("@[", 2);
                put(best, strlen(best));
                put("]", 1);
                p += 1 + bestLen;
                continue;
            }
            // Unknown: one word, without the punctuation that ends a sentence.
            size_t word = 0;
            while (p[1 + word] && p[1 + word] != ' ' && p[1 + word] != '\n')
                word++;
            while (word > 0 && strchr(".,;:!?)", p[word]))
                word--;
            if (word > 0) {
                put("@[", 2);
                put(p + 1, word);
                put("]", 1);
                p += 1 + word;
                continue;
            }
        }
        put(p, 1);
        p++;
    }
    LOG_DEBUG("JenCore text prepared: reply=%u known=%d", (unsigned)replyId, replied != nullptr);
    out[o] = 0;
    return o;
}

uint32_t JenCoreCompanion::nextTxTimestamp(uint32_t now)
{
    lastTxTimestamp = now > lastTxTimestamp ? now : lastTxTimestamp + 1;
    return lastTxTimestamp;
}

bool JenCoreCompanion::cachedChannelHash(ChannelIndex ch, const uint8_t *key, size_t keyLen, uint8_t *out)
{
    if (ch >= MAX_NUM_CHANNELS)
        return channelHash(key, keyLen, out);

    HashCacheEntry &entry = hashCache[ch];
    if (entry.keyLen != keyLen || memcmp(entry.key, key, keyLen) != 0) {
        if (!channelHash(key, keyLen, &entry.hash)) {
            entry.keyLen = 0;
            return false;
        }
        memcpy(entry.key, key, keyLen);
        entry.keyLen = keyLen;
    }
    *out = entry.hash;
    return true;
}

uint32_t JenCoreCompanion::resolveRelayNode(const uint8_t *hash, uint8_t hashSize)
{
    if (!hash || hashSize == 0 || hashSize > 3)
        return 0;

    uint32_t found = 0;
    contacts.forEach([&](JenCoreContact &c) {
        if (!found && memcmp(c.pub, hash, hashSize) == 0)
            found = c.nodeNum;
    });
    if (found || !nodeDB)
        return found;

    // A repeater we only heard an ADVERT from lives in the node DB, not the contact table.
    for (size_t i = 0; i < nodeDB->getNumMeshNodes(); i++) {
        const meshtastic_NodeInfoLite *lite = nodeDB->getMeshNodeByIndex(i);
        if (lite && lite->public_key.size == sizeof(lite->public_key.bytes) &&
            memcmp(lite->public_key.bytes, hash, hashSize) == 0)
            return lite->num;
    }
    return 0;
}

void JenCoreCompanion::handleFrame(const uint8_t *raw, size_t len, float snr, int32_t rssi, float freqErr, uint8_t codingRate)
{
    static_assert(FRAME_PATH_CAP == nexus::jen_core_meta::PATH_CAP, "path caps must agree");
    if (!raw || !router)
        return;
    lastRxFreqErr = freqErr;
    lastRxCodingRate = codingRate;
    lastRxSnr = snr;
    lastRxRssi = rssi;
    lastRxHasRssi = true;

    JenCoreRepeater::FrameInfo info;
    if (!JenCoreRepeater::parse(raw, len, &info))
        return;

    const size_t pathBytes = (size_t)info.path_hash_count * info.path_hash_size;
    const uint8_t *path = pathBytes ? raw + info.header_len - pathBytes : nullptr;
    bool pathTruncated = false;
    lastFramePathLen = (uint8_t)nexus::jen_core_meta::copyPathTail(path, pathBytes, info.path_hash_size, lastFramePath,
                                                                  sizeof(lastFramePath), &pathTruncated);
    lastFrameHops = info.path_hash_count;
    lastFrameRelayNode = 0;
    if (info.path_hash_count > 0 && path) {
        const uint32_t relay = resolveRelayNode(path + pathBytes - info.path_hash_size, info.path_hash_size);
        lastFrameRelayNode = (uint16_t)(relay & 0xFFFF);
    }
    bool regionScoped = false;
    uint8_t regionIndex = 0xFF;
    if (info.route_type == 0 && len >= 5) { // transport-flood carries the region code
        regionScoped = ((uint16_t)raw[1] | ((uint16_t)raw[2] << 8)) != 0;
        regionIndex = JenCoreRepeater::resolveRegionIndex(raw, len, info);
    }
    lastFrameFlags = nexus::jen_core_meta::packFlags(true, regionScoped, pathTruncated, info.route_type, info.payload_type,
                                                     info.path_hash_size, regionIndex);

    switch (info.payload_type) {
    case 0x05: // GRP_TXT
        handleGroupText(raw, len, info, snr, rssi);
        break;
    case 0x04: // ADVERT
        handleAdvert(raw, len, info);
        break;
    case 0x02: // TXT_MSG
        handleDirectText(raw, len, info, snr, rssi);
        break;
    case 0x03: // ACK
        handleAck(raw, len, info);
        break;
    case 0x08: // PATH: how JenCore answers a flood, with the ACK inside
        handlePathAck(raw, len, info);
        break;
    default:
        break;
    }
}

void JenCoreCompanion::handleGroupText(const uint8_t *raw, size_t len, const JenCoreRepeater::FrameInfo &info, float snr,
                                        int32_t rssi)
{
    rxSeen++;
    if (info.header_len >= len || info.payload_len < 1 + 2 + 16) {
        rxIgnored++;
        return;
    }
    const uint32_t hash = JenCoreRepeater::hashFrame(raw, len, info);
    // A repeater re-flooding a text of ours (it carries at least its own hash in the path) is the proof it left the room.
    if (info.path_hash_count > 0)
        settleEcho(hash);
    if (wasRecentlySeen(hash)) {
        rxDup++;
        return;
    }

    for (ChannelIndex i = 0; i < MAX_NUM_CHANNELS; i++) {
        uint8_t key[32];
        const size_t keyLen = channels.getJenCoreKey(i, key);
        if (keyLen == 0)
            continue;

        uint8_t hash8;
        if (!cachedChannelHash(i, key, keyLen, &hash8) || raw[info.header_len] != hash8)
            continue; // not this channel: skip the HMAC and AES work

        char text[MAX_TEXT_LEN + 1];
        uint32_t timestamp = 0;
        if (!decodeGroupText(key, keyLen, raw, len, info, text, sizeof(text), &timestamp))
            continue;

        meshtastic_MeshPacket *p = packetPool.allocZeroed();
        if (!p)
            return;

        rememberFrame(hash);
        rxDecoded++;

        // Texts read "name: body": the sender gets its own node and the client shows the body alone. Anything
        // without a prefix falls back to the generic "JenCore" node.
        char sender[sizeof(((meshtastic_User *)nullptr)->long_name)];
        const char *body = text;
        uint32_t fromNum = PSEUDO_NODE_NUM;
        if (splitSender(text, sender, sizeof(sender), &body)) {
            // Someone we can message shows up as that one node in channels too, instead of a second, unmessagable one.
            JenCoreContact *contact = contacts.findUniqueByName(sender);
            if (contact) {
                fromNum = contact->nodeNum;
                ensureSenderNode(fromNum, contact->name, true, contact->pub);
            } else {
                fromNum = senderNodeNum(sender);
                // Two contacts sharing this name leave it ambiguous: the number may belong to one of them, who stays as is.
                if (!contacts.findByNodeNum(fromNum))
                    ensureSenderNode(fromNum, sender);
            }
        } else {
            ensurePseudoNode(); // a node-DB reset or eviction may have removed it since boot
        }

        p->from = fromNum;
        p->to = NODENUM_BROADCAST;
        p->id = generatePacketId();
        if (fromNum != PSEUDO_NODE_NUM)
            noteRecentText(p->id, sender);
        p->channel = i;
        p->which_payload_variant = meshtastic_MeshPacket_decoded_tag;
        p->decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;
        const size_t textLen = strlen(body);
        p->decoded.payload.size = textLen <= sizeof(p->decoded.payload.bytes) ? textLen : sizeof(p->decoded.payload.bytes);
        memcpy(p->decoded.payload.bytes, body, p->decoded.payload.size);
        p->decoded.has_bitfield = true;
        p->decoded.bitfield = 0;
        // Hops are the repeaters the frame crossed (its path). A hop_limit below hop_start also keeps this
        // synthetic packet's relay gate closed, and a next hop that is not us closes it for direct frames.
        JenCoreCompanion::hopFields(info.path_hash_count, &p->hop_start, &p->hop_limit);
        const uint8_t ourRelay = nodeDB->getLastByteOfNodeNum(nodeDB->getNodeNum());
        p->next_hop = (uint8_t)(ourRelay + 1);
        if (p->next_hop == NO_NEXT_HOP_PREFERENCE)
            p->next_hop = 1;
        p->transport_mechanism = meshtastic_MeshPacket_TransportMechanism_TRANSPORT_LORA;
        p->rx_snr = snr;
        p->has_rx_rssi = true;
        p->rx_rssi = rssi;
        p->rx_frequency_error = lastRxFreqErr;
        p->rx_coding_rate = lastRxCodingRate;
        nexus::jen_core_meta::apply(*p, lastFrameFlags, lastFrameRelayNode, lastFramePath, lastFramePathLen);

        LOG_INFO("JenCore group text ch=%u ts=%u len=%u", (unsigned)i, timestamp, (unsigned)p->decoded.payload.size);
        router->enqueueReceivedMessage(p);
        return;
    }

    rxIgnored++;
}

void JenCoreCompanion::setIdentity(const uint8_t seed[32], const uint8_t pub[32])
{
    memcpy(identitySeed, seed, sizeof(identitySeed));
    memcpy(identityPub, pub, sizeof(identityPub));
    identityReady = true;
    contacts.forEach([](JenCoreContact &c) { c.hasSecret = false; }); // secrets were derived for another identity
}

bool JenCoreCompanion::ensureSecret(JenCoreContact &c)
{
    if (!c.hasSecret)
        c.hasSecret = identityReady && sharedSecret(identitySeed, c.pub, c.secret);
    return c.hasSecret;
}

void JenCoreCompanion::handleAdvert(const uint8_t *raw, size_t len, const JenCoreRepeater::FrameInfo &info)
{
    // ADVERT payload: public key (32), timestamp (4), signature (64), then app data: a flags byte (node type in the
    // low nibble, 0x10 position, 0x20/0x40 reserved features, 0x80 name) followed by those fields in that order.
    constexpr size_t appDataAt = 32 + 4 + 64;
    if (!identityReady || info.header_len >= len || info.payload_len <= appDataAt)
        return;

    const uint8_t *payload = &raw[info.header_len];
    if (memcmp(payload, identityPub, 32) == 0)
        return; // our own advert coming back

    const uint8_t flags = payload[appDataAt];
    const uint8_t type = flags & 0x0F;
    const bool repeater = type == JenCoreRepeater::ADV_TYPE_REPEATER;
    if ((type != JenCoreRepeater::ADV_TYPE_CHAT && !repeater) || !(flags & 0x80))
        return; // only named chat nodes (to message) and repeaters (to list): rooms and sensors are not shown

    size_t at = appDataAt + 1;
    int32_t latMicro = 0, lonMicro = 0;
    if (flags & 0x10) {
        if (at + 8 > info.payload_len)
            return;
        latMicro = (int32_t)readLe32(&payload[at]);
        lonMicro = (int32_t)readLe32(&payload[at + 4]);
        at += 8;
    }
    if (flags & 0x20)
        at += 2;
    if (flags & 0x40)
        at += 2;
    if (at >= info.payload_len)
        return;

    char name[JenCoreContact::NAME_LEN];
    size_t nameLen = info.payload_len - at;
    if (nameLen > sizeof(name) - 1)
        nameLen = sizeof(name) - 1;
    memcpy(name, &payload[at], nameLen);
    name[nameLen] = 0;
    sanitizeUtf8(name, sizeof(name));
    if (name[0] == 0)
        return;

    if (repeater) {
        // A repeater is not someone to message: it only gets a node, keyed by its public key, and no contact slot.
        uint32_t num = JenCoreContacts<1>::nodeNumForKey(payload);
        if (num == PSEUDO_NODE_NUM)
            num++;
        const meshtastic_NodeInfoLite *lite = nodeDB ? nodeDB->getMeshNode(num) : nullptr;
        const bool known = lite && nodeInfoLiteHasUser(lite) && lite->public_key.size == 32 &&
                           memcmp(lite->public_key.bytes, payload, 32) == 0 &&
                           strncmp(lite->long_name, name, utf8PrefixLen(name, sizeof(lite->long_name) - 6)) == 0;
        if (!known && !JenCoreRepeater::verifyAdvert(payload, info.payload_len))
            return;
        ensureSenderNode(num, name, false, payload, meshtastic_Config_DeviceConfig_Role_REPEATER);
        updatePosition(num, latMicro, lonMicro);
        return;
    }

    // A contact we already trust under the same name only needs its LRU stamp refreshed: skip the signature check.
    JenCoreContact *known = contacts.findByPub(payload);
    if (known && strcmp(known->name, name) == 0) {
        contacts.touch(known);
        ensureSenderNode(known->nodeNum, known->name, true, known->pub); // brings a node made before keys were set up along
        updatePosition(known->nodeNum, latMicro, lonMicro);
        return;
    }
    if (!JenCoreRepeater::verifyAdvert(payload, info.payload_len))
        return;

    JenCoreContact *c = contacts.upsert(payload, name);
    if (!known)
        contactsLearned++;
    ensureSenderNode(c->nodeNum, c->name, true, c->pub);
    retireGroupNode(c->name);
    retireKeyedNode(c->pub, c->nodeNum);
    updatePosition(c->nodeNum, latMicro, lonMicro);
}

void JenCoreCompanion::retireGroupNode(const char *name)
{
    // The node made for this name before it was a contact: ours (no hardware model, unmessagable), not a favourite.
    const uint32_t num = senderNodeNum(name);
    if (contacts.findByNodeNum(num))
        return; // the contact took over this very node: nothing left to retire
    const meshtastic_NodeInfoLite *lite = nodeDB ? nodeDB->getMeshNode(num) : nullptr;
    if (lite && nodeInfoLiteHasUser(lite) && lite->hw_model == meshtastic_HardwareModel_UNSET &&
        nodeInfoLiteIsUnmessagable(lite) && !nodeInfoLiteIsFavorite(lite))
        nodeDB->removeNodeByNum(num);
}

void JenCoreCompanion::retireKeyedNode(const uint8_t pub[32], uint32_t keepNum)
{
    // Earlier builds numbered a contact's node from its public key; that node carries the same key and no hardware model.
    const uint32_t num = JenCoreContacts<1>::nodeNumForKey(pub);
    if (num == keepNum || !nodeDB)
        return;
    const meshtastic_NodeInfoLite *lite = nodeDB->getMeshNode(num);
    if (lite && nodeInfoLiteHasUser(lite) && lite->hw_model == meshtastic_HardwareModel_UNSET && !nodeInfoLiteIsFavorite(lite) &&
        lite->public_key.size == 32 && memcmp(lite->public_key.bytes, pub, 32) == 0)
        nodeDB->removeNodeByNum(num);
}

void JenCoreCompanion::injectDirectText(const JenCoreContact &c, const char *text, int pathHashCount, float snr, int32_t rssi)
{
    meshtastic_MeshPacket *p = packetPool.allocZeroed();
    if (!p)
        return;

    ensureSenderNode(c.nodeNum, c.name, true, c.pub); // a node-DB reset or eviction may have removed it
    p->from = c.nodeNum;
    p->to = nodeDB->getNodeNum();
    p->id = generatePacketId();
    p->channel = 0;
    p->which_payload_variant = meshtastic_MeshPacket_decoded_tag;
    p->decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;
    const size_t textLen = strlen(text);
    p->decoded.payload.size = textLen <= sizeof(p->decoded.payload.bytes) ? textLen : sizeof(p->decoded.payload.bytes);
    memcpy(p->decoded.payload.bytes, text, p->decoded.payload.size);
    p->decoded.has_bitfield = true;
    p->decoded.bitfield = 0;
    hopFields(pathHashCount, &p->hop_start, &p->hop_limit);
    p->transport_mechanism = meshtastic_MeshPacket_TransportMechanism_TRANSPORT_LORA;
    p->rx_snr = snr;
    p->has_rx_rssi = true;
    p->rx_rssi = rssi;
    p->rx_frequency_error = lastRxFreqErr;
    p->rx_coding_rate = lastRxCodingRate;
    nexus::jen_core_meta::apply(*p, lastFrameFlags, lastFrameRelayNode, lastFramePath, lastFramePathLen);
    router->enqueueReceivedMessage(p);
}

void JenCoreCompanion::queueAck(uint32_t hash)
{
    uint8_t frame[8];
    const size_t n = encodeAck(hash, frame, sizeof(frame));
    RadioInterface *iface = router ? router->getRadioIface() : nullptr;
    if (n == 0 || !iface || !iface->queueJenCoreFrame(frame, n))
        LOG_WARN("JenCore ACK could not be queued");
}

void JenCoreCompanion::handleDirectText(const uint8_t *raw, size_t len, const JenCoreRepeater::FrameInfo &info, float snr,
                                         int32_t rssi)
{
    // TXT_MSG payload: dest hash (1), source hash (1), MAC (2), ciphertext.
    if (!identityReady || info.header_len >= len || info.payload_len < 1 + 1 + 2 + 16)
        return;
    const uint8_t *payload = &raw[info.header_len];
    if (payload[0] != identityPub[0])
        return; // not for us: the repeater relays it, we cannot read it anyway

    const uint32_t hash = JenCoreRepeater::hashFrame(raw, len, info);
    if (wasRecentlySeen(hash)) {
        rxDup++;
        return;
    }

    // The source hash is one byte, so several contacts can share it: the MAC tells which one wrote it.
    JenCoreContact *candidates[CONTACTS_MAX];
    const size_t n = contacts.findByHash(payload[1], candidates, CONTACTS_MAX);
    for (size_t i = 0; i < n; i++) {
        JenCoreContact *c = candidates[i];
        char text[MAX_TEXT_LEN + 1];
        uint32_t timestamp = 0;
        uint8_t flags = 0;
        if (!ensureSecret(*c) || !decodeDirectText(c->secret, raw, len, info, text, sizeof(text), &timestamp, &flags))
            continue;

        rememberFrame(hash);
        contacts.touch(c);
        dmRx++;
        LOG_INFO("JenCore direct text from %s ts=%u len=%u", c->name, timestamp, (unsigned)strlen(text));
        injectDirectText(*c, text, info.path_hash_count, snr, rssi);
        queueAck(ackHash(timestamp, flags, text, c->pub));
        return;
    }
}

void JenCoreCompanion::reportDelivery(uint32_t fromNum, uint32_t packetId, meshtastic_Routing_Error err, bool withMeta)
{
    meshtastic_Routing r = meshtastic_Routing_init_default;
    r.which_variant = meshtastic_Routing_error_reason_tag;
    r.error_reason = err;

    meshtastic_MeshPacket *p = packetPool.allocZeroed();
    if (!p)
        return;
    p->from = fromNum;
    p->to = nodeDB->getNodeNum();
    p->id = generatePacketId();
    p->channel = 0;
    p->priority = meshtastic_MeshPacket_Priority_ACK;
    p->which_payload_variant = meshtastic_MeshPacket_decoded_tag;
    p->decoded.portnum = meshtastic_PortNum_ROUTING_APP;
    p->decoded.request_id = packetId;
    p->decoded.payload.size =
        pb_encode_to_bytes(p->decoded.payload.bytes, sizeof(p->decoded.payload.bytes), &meshtastic_Routing_msg, &r);
    // The router drops a packet whose hop_start is missing, so the delivery report carries real hop fields.
    p->decoded.has_bitfield = true;
    p->decoded.bitfield = 0;
    if (withMeta) {
        p->rx_snr = lastRxSnr;
        p->has_rx_rssi = lastRxHasRssi;
        p->rx_rssi = lastRxRssi;
        p->rx_frequency_error = lastRxFreqErr;
        p->rx_coding_rate = lastRxCodingRate;
        nexus::jen_core_meta::apply(*p, lastFrameFlags, lastFrameRelayNode, lastFramePath, lastFramePathLen);
    }
    hopFields(withMeta ? lastFrameHops : 0, &p->hop_start, &p->hop_limit);
    p->transport_mechanism = meshtastic_MeshPacket_TransportMechanism_TRANSPORT_LORA;
    router->enqueueReceivedMessage(p);
}

void JenCoreCompanion::addPending(uint32_t ackHash, uint32_t packetId, uint32_t contactNum, bool group)
{
    PendingAck *slot = nullptr;
    for (auto &e : pending) {
        if (!e.used) {
            slot = &e;
            break;
        }
        if (!slot || (uint32_t)(millis() - e.sentMs) > (uint32_t)(millis() - slot->sentMs))
            slot = &e; // the oldest one gives way
    }
    if (slot->used)
        reportDelivery(slot->contactNum, slot->packetId, meshtastic_Routing_Error_MAX_RETRANSMIT);

    slot->ackHash = ackHash;
    slot->packetId = packetId;
    slot->contactNum = contactNum;
    slot->sentMs = millis();
    slot->group = group;
    slot->used = true;
}

bool JenCoreCompanion::trackGroupText(const uint8_t *frame, size_t len, uint32_t packetId)
{
    JenCoreRepeater::FrameInfo info;
    if (!frame || !JenCoreRepeater::parse(frame, len, &info))
        return false;
    addPending(JenCoreRepeater::hashFrame(frame, len, info), packetId, PSEUDO_NODE_NUM, true);
    return true;
}

void JenCoreCompanion::settleEcho(uint32_t hash)
{
    for (auto &e : pending) {
        if (e.used && e.group && e.ackHash == hash) {
            e.used = false;
            groupEchoed++;
            reportDelivery(e.contactNum, e.packetId, meshtastic_Routing_Error_NONE, /*withMeta=*/true);
            return;
        }
    }
}

bool JenCoreCompanion::sendDirectText(uint32_t nodeNum, uint32_t packetId, const char *text, uint32_t now, RadioInterface *iface)
{
    JenCoreContact *c = contacts.findByNodeNum(nodeNum);
    if (!c || !identityReady || !iface || now == 0 || !text || !ensureSecret(*c))
        return false;

    // The ACK hashes the text as the peer will read it, so cut it the way encodeDirectText does.
    char cut[MAX_TEXT_LEN + 1];
    const size_t cutLen = utf8PrefixLen(text, MAX_TEXT_LEN);
    memcpy(cut, text, cutLen);
    cut[cutLen] = 0;

    const uint32_t timestamp = nextTxTimestamp(now);
    uint8_t frame[JenCoreRepeater::MAX_FRAME_LEN];
    const size_t frameLen =
        encodeDirectText(c->secret, c->pub[0], identityPub[0], timestamp, TXT_FLAGS_PLAIN, cut, frame, sizeof(frame));
    if (frameLen == 0 || !iface->queueJenCoreFrame(frame, frameLen))
        return false;

    addPending(ackHash(timestamp, TXT_FLAGS_PLAIN, cut, identityPub), packetId, nodeNum);
    contacts.touch(c);
    dmTx++;
    return true;
}

void JenCoreCompanion::settleAck(uint32_t hash)
{
    for (auto &e : pending) {
        if (e.used && !e.group && e.ackHash == hash) {
            e.used = false;
            dmAcked++;
            reportDelivery(e.contactNum, e.packetId, meshtastic_Routing_Error_NONE, /*withMeta=*/true);
            return;
        }
    }
}

void JenCoreCompanion::handlePathAck(const uint8_t *raw, size_t len, const JenCoreRepeater::FrameInfo &info)
{
    // PATH payload: dest hash (1), source hash (1), MAC (2), ciphertext. Only ours, from a contact we hold a secret for.
    if (!identityReady || info.header_len >= len || info.payload_len < 1 + 1 + 2 + 16)
        return;
    const uint8_t *payload = &raw[info.header_len];
    if (payload[0] != identityPub[0])
        return;

    JenCoreContact *candidates[CONTACTS_MAX];
    const size_t n = contacts.findByHash(payload[1], candidates, CONTACTS_MAX);
    for (size_t i = 0; i < n; i++) {
        uint32_t hash = 0;
        if (ensureSecret(*candidates[i]) && decodePathAck(candidates[i]->secret, raw, len, info, &hash)) {
            settleAck(hash);
            return;
        }
    }
}

void JenCoreCompanion::handleAck(const uint8_t *raw, size_t len, const JenCoreRepeater::FrameInfo &info)
{
    if (info.payload_len != 4 || info.header_len + 4 > len)
        return;
    settleAck(readLe32(&raw[info.header_len]));
}

void JenCoreCompanion::service()
{
    for (auto &e : pending) {
        if (e.used && !Throttle::isWithinTimespanMs(e.sentMs, ACK_TIMEOUT_MS)) {
            e.used = false;
            (e.group ? groupTimedOut : dmTimedOut)++;
            reportDelivery(e.contactNum, e.packetId, meshtastic_Routing_Error_MAX_RETRANSMIT);
        }
    }
}

static constexpr uint8_t SAVED_MAGIC[3] = {'M', 'C', 'C'};
static constexpr uint8_t SAVED_VERSION = 1;

size_t JenCoreCompanion::exportContacts(uint8_t *buf, size_t cap)
{
    if (!buf || cap < SAVED_IMAGE_MAX)
        return 0;

    // The most recently used contacts, newest first: the table can hold more than the file.
    const JenCoreContact *picked[SAVED_MAX];
    size_t count = 0;
    for (; count < SAVED_MAX; count++) {
        const JenCoreContact *best = nullptr;
        contacts.forEach([&](JenCoreContact &c) {
            for (size_t i = 0; i < count; i++)
                if (picked[i] == &c)
                    return;
            if (!best || c.lastUsed > best->lastUsed)
                best = &c;
        });
        if (!best)
            break;
        picked[count] = best;
    }

    memcpy(buf, SAVED_MAGIC, sizeof(SAVED_MAGIC));
    buf[3] = SAVED_VERSION;
    buf[4] = (uint8_t)count;
    for (size_t i = 0; i < count; i++) {
        uint8_t *rec = &buf[5 + i * SAVED_RECORD_LEN];
        memcpy(rec, picked[i]->pub, 32);
        memcpy(&rec[32], picked[i]->name, JenCoreContact::NAME_LEN);
    }
    return 5 + count * SAVED_RECORD_LEN;
}

size_t JenCoreCompanion::importContacts(const uint8_t *buf, size_t len)
{
    if (!buf || len < 5 || memcmp(buf, SAVED_MAGIC, sizeof(SAVED_MAGIC)) != 0 || buf[3] != SAVED_VERSION)
        return 0;
    const size_t count = buf[4];
    if (count > SAVED_MAX || len != 5 + count * SAVED_RECORD_LEN)
        return 0;

    // Saved newest first: load oldest first so the newest ends up the most recently used.
    size_t loaded = 0;
    for (size_t i = count; i-- > 0;) {
        const uint8_t *rec = &buf[5 + i * SAVED_RECORD_LEN];
        if (identityReady && memcmp(rec, identityPub, 32) == 0)
            continue;
        char name[JenCoreContact::NAME_LEN];
        memcpy(name, &rec[32], sizeof(name));
        name[sizeof(name) - 1] = 0;
        JenCoreContact *c = contacts.upsert(rec, name);
        ensureSenderNode(c->nodeNum, c->name, true, c->pub);
        retireGroupNode(c->name);
        retireKeyedNode(c->pub, c->nodeNum);
        loaded++;
    }
    return loaded;
}

uint32_t JenCoreCompanion::contactsFingerprint()
{
    // Public keys and names only: the LRU order changes with every advert and is not worth a flash write.
    uint32_t h = 2166136261u; // FNV-1a
    auto mix = [&](const uint8_t *p, size_t n) {
        for (size_t i = 0; i < n; i++)
            h = (h ^ p[i]) * 16777619u;
    };
    uint32_t sum = 0, count = 0;
    contacts.forEach([&](JenCoreContact &c) {
        uint32_t ch = 2166136261u;
        for (size_t i = 0; i < 32; i++)
            ch = (ch ^ c.pub[i]) * 16777619u;
        for (const char *q = c.name; *q; q++)
            ch = (ch ^ (uint8_t)*q) * 16777619u;
        sum += ch; // order independent: a slot being reused must not look like a change
        count++;
    });
    mix((const uint8_t *)&sum, sizeof(sum));
    mix((const uint8_t *)&count, sizeof(count));
    return h;
}

void JenCoreCompanion::noteTransmitted(const uint8_t *raw, size_t len)
{
    JenCoreRepeater::FrameInfo info;
    if (raw && JenCoreRepeater::parse(raw, len, &info))
        rememberFrame(JenCoreRepeater::hashFrame(raw, len, info));
}

#endif // NEXUS_ENABLE_JENCORE_COMPANION
