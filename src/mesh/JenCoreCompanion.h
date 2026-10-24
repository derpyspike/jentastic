// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) 2026 Jennifer Sanchez
//
// Licensed under the GNU Affero General Public License v3.
// See LICENSE-AGPL for the full license text.
//

#pragma once

#include "configuration.h"

#if MESHTASTIC_ENABLE_JENCORE_COMPANION

#include "Channels.h"
#include "JenCoreContacts.h"
#include "JenCoreDedup.h"
#include "JenCoreRepeater.h"
#include <stddef.h>
#include <stdint.h>

class RadioInterface;

/**
 * JenCore group-text bridge for the Meshtastic client.
 *
 * Channels named "MC:..." carry a JenCore group PSK. Text sent on one is encoded as a JenCore
 * GRP_TXT flood (timestamp + "name: text", AES-128 + 2-byte HMAC); GRP_TXT frames we can decrypt
 * are injected into the Meshtastic router as a broadcast from PSEUDO_NODE_NUM, so the phone shows
 * them on that channel alongside Meshtastic traffic.
 */
class JenCoreCompanion
{
  public:
    /// Node number all bridged JenCore messages appear to come from.
    static constexpr uint32_t PSEUDO_NODE_NUM = 0x4D430001;

    /// JenCore MAX_TEXT_LEN: the encrypted body's limit, including the "name: " prefix.
    static constexpr size_t MAX_TEXT_LEN = 160;

    /// Largest ciphertext a valid GRP_TXT carries: 5-byte header + MAX_TEXT_LEN, padded to the AES block.
    static constexpr size_t MAX_CIPHER_LEN = 176;

    /// Effective secret length (16 or 32): a 32-byte key with an all-zero second half is a 16-byte secret.
    /// Returns 0 for any other key length.
    static size_t normalizedKeyLen(const uint8_t *key, size_t keyLen);

    /// First byte of SHA-256 over the effective secret, JenCore's GroupChannel::hash.
    static bool channelHash(const uint8_t *key, size_t keyLen, uint8_t *out);

    /**
     * Encode a GRP_TXT flood frame. sender is prefixed as "sender: "; text is truncated to
     * MAX_TEXT_LEN on a UTF-8 boundary; *truncated (optional) reports whether text lost its tail.
     * Returns the frame length, or 0 if it does not fit.
     */
    static size_t encodeGroupText(const uint8_t *key, size_t keyLen, const char *sender, uint32_t timestamp, const char *text,
                                  uint8_t *out, size_t outCap, bool *truncated = nullptr);

    /**
     * Decrypt and validate a GRP_TXT frame for key. textOut receives the decrypted "sender: text"
     * body (NUL-terminated, UTF-8 sanitised). Returns false on a wrong channel hash, bad MAC,
     * malformed ciphertext or unsupported text type.
     */
    static bool decodeGroupText(const uint8_t *key, size_t keyLen, const uint8_t *frame, size_t len,
                                const JenCoreRepeater::FrameInfo &info, char *textOut, size_t textOutCap,
                                uint32_t *timestampOut);

    /// JenCore ECDH: our Ed25519 seed and the peer's Ed25519 public key give the 32-byte secret both sides
    /// derive (orlp ed25519_key_exchange: X25519 over the clamped SHA-512 scalar and the key's Montgomery u).
    /// False for a malformed or weak peer key.
    static bool sharedSecret(const uint8_t seed[32], const uint8_t peerPub[32], uint8_t out[32]);

    /// Largest ciphertext a valid TXT_MSG carries: 5-byte header + MAX_TEXT_LEN, padded to the AES block.
    static constexpr size_t MAX_DIRECT_CIPHER_LEN = MAX_CIPHER_LEN;

    /// TXT_MSG flood flags byte: text type in bits 2-7 (0 = plain), send attempt in bits 0-1.
    static constexpr uint8_t TXT_FLAGS_PLAIN = 0;

    /**
     * Encode a TXT_MSG flood frame for a peer: dest_hash | src_hash | MAC | AES(timestamp, flags, text). hashes are the
     * first byte of each public key. text is truncated to MAX_TEXT_LEN on a UTF-8 boundary. Returns the frame length,
     * or 0 if it does not fit.
     */
    static size_t encodeDirectText(const uint8_t secret[32], uint8_t destHash, uint8_t srcHash, uint32_t timestamp, uint8_t flags,
                                   const char *text, uint8_t *out, size_t outCap, bool *truncated = nullptr);

    /**
     * Verify and decrypt a TXT_MSG for secret. textOut gets the NUL-terminated, sanitised text; the timestamp and flags
     * are returned as sent. False on a bad MAC, malformed ciphertext or a non-plain text type.
     */
    static bool decodeDirectText(const uint8_t secret[32], const uint8_t *frame, size_t len,
                                 const JenCoreRepeater::FrameInfo &info, char *textOut, size_t textOutCap,
                                 uint32_t *timestampOut, uint8_t *flagsOut);

    /// The 4-byte ACK a peer answers a TXT_MSG with: SHA-256(timestamp | flags | text | senderPub)[:4], where
    /// senderPub belongs to whoever wrote the text (BaseChatMesh::onPeerDataRecv).
    static uint32_t ackHash(uint32_t timestamp, uint8_t flags, const char *text, const uint8_t senderPub[32]);

    /// JenCore answers a flooded TXT_MSG with a PATH frame that carries the ACK inside (Mesh::createPathReturn): path_len,
    /// path, extra type (ACK), hash, all encrypted with the shared secret. These two build and read that frame.
    static size_t encodePathAck(const uint8_t secret[32], uint8_t destHash, uint8_t srcHash, uint32_t hash, uint8_t *out,
                                size_t outCap);
    static bool decodePathAck(const uint8_t secret[32], const uint8_t *frame, size_t len, const JenCoreRepeater::FrameInfo &info,
                              uint32_t *hashOut);

    /// Encode an ACK flood frame carrying hash. Returns the frame length, or 0 if it does not fit.
    static size_t encodeAck(uint32_t hash, uint8_t *out, size_t outCap);

    /// Meshtastic hop fields for a frame that crossed pathHashCount repeaters. Clients show hops as
    /// hop_start - hop_limit, so 0 hops stays "direct" and anything beyond HOP_START is capped there.
    static constexpr uint8_t HOP_START = 7;
    static void hopFields(uint8_t pathHashCount, uint8_t *hopStart, uint8_t *hopLimit);

    /// Create the synthetic "JenCore" node if it is missing, so the client has a sender name to render.
    /// It stands in for any text that carries no "name: " prefix.
    void ensurePseudoNode();

    /// Split a decoded "name: text" body at the first ": ". Returns false when there is no usable name
    /// (missing separator or empty name); otherwise nameOut holds the name, cut to nameCap-1 bytes on a
    /// UTF-8 boundary, and *body points at the text after the separator.
    static bool splitSender(const char *text, char *nameOut, size_t nameCap, const char **body);

    /// Node number standing in for a JenCore sender: 'M' in the top byte, 24 bits hashed from the name.
    static uint32_t senderNodeNum(const char *name);

    /// Create or refresh the node for a bridged sender: long name is the name, short name its first 4 bytes
    /// (never splitting a UTF-8 character).
    /// messagable=true marks a direct-message contact: the client lets the user write to it. publicKey (32 bytes, the
    /// contact's JenCore key) goes into the node's key field, because clients only offer a direct message to a node that
    /// has one. The text itself is encrypted by JenCore, not with that key.
    void ensureSenderNode(uint32_t nodeNum, const char *name, bool messagable = false, const uint8_t *publicKey = nullptr,
                          meshtastic_Config_DeviceConfig_Role role = meshtastic_Config_DeviceConfig_Role_CLIENT);

    /// Record a node's position from an ADVERT (degrees x 1e6, as JenCore sends it) and tell the client. Nothing is done
    /// for 0,0 (no position) or a position the node DB already holds.
    void updatePosition(uint32_t nodeNum, int32_t latMicro, int32_t lonMicro);

    /// Remember who wrote a bridged channel text (by the packet id the client sees), so a reply to it can name them.
    void noteRecentText(uint32_t packetId, const char *sender);
    const char *recentSender(uint32_t packetId) const;

    /// Rewrite an outgoing channel text the way JenCore apps expect: "@[name] " in front of a reply (replyId is the packet
    /// id of the text answered, 0 if none) and "@name" as "@[name]" (a known name whole, otherwise the next word). Returns the
    /// length.
    size_t prepareGroupText(const char *text, uint32_t replyId, char *out, size_t cap);

    /// Wait for a repeater to echo the channel text frame we just queued, then tell the client packetId was delivered; if
    /// nobody does within ACK_TIMEOUT_MS it is reported as failed. False if frame is not a JenCore frame.
    bool trackGroupText(const uint8_t *frame, size_t len, uint32_t packetId);

    /// Timestamp for the next outgoing text: strictly increasing, so two texts sent within the same
    /// second still encrypt differently and are not dropped as duplicates by repeaters.
    uint32_t nextTxTimestamp(uint32_t now);

    /// Consume a received JenCore frame: GRP_TXT on any MC: channel and TXT_MSG for us are injected into the
    /// router, an ACK settles a pending direct message, an ADVERT teaches us a contact.
    void handleFrame(const uint8_t *raw, size_t len, float snr, int32_t rssi);

    /// Install our JenCore identity (Ed25519 seed and public key); direct messages need it.
    void setIdentity(const uint8_t seed[32], const uint8_t pub[32]);
    bool hasIdentity() const { return identityReady; }

    /// True when node is a direct-message contact we hold a public key for.
    bool hasContact(uint32_t nodeNum) { return contacts.findByNodeNum(nodeNum) != nullptr; }
    size_t contactCount() const { return contacts.count(); }

    /// Queue a TXT_MSG for the contact behind nodeNum and wait for its ACK, answering packetId to the client.
    /// text is cut to MAX_TEXT_LEN on a UTF-8 boundary. False if it could not be queued.
    bool sendDirectText(uint32_t nodeNum, uint32_t packetId, const char *text, uint32_t now, RadioInterface *iface);

    /// How long a direct message waits for its ACK before the client is told it failed.
    static constexpr uint32_t ACK_TIMEOUT_MS = 30000;

    /// Run periodically: report direct messages whose ACK never came.
    void service();

    /// The most recently used contacts as a flash image: magic, count, then public key and name per contact. This is
    /// independent of the node DB's favourite flag, whose role-specific meaning in Meshtastic stays untouched.
    /// Returns the image length (an empty table is still a valid 5-byte image), 0 if cap is below SAVED_IMAGE_MAX.
    size_t exportContacts(uint8_t *buf, size_t cap);

    /// Restore contacts from an exportContacts() image. Returns how many were loaded.
    size_t importContacts(const uint8_t *buf, size_t len);

    static constexpr size_t SAVED_MAX = 16;
    static constexpr size_t SAVED_RECORD_LEN = 32 + JenCoreContact::NAME_LEN;
    static constexpr size_t SAVED_IMAGE_MAX = 5 + SAVED_MAX * SAVED_RECORD_LEN;

    /// Checksum of the contact set (keys and names, not recency), to save only when it changed.
    uint32_t contactsFingerprint();

    /// Mark one of our own transmitted frames seen, so hearing it back does not inject it.
    void noteTransmitted(const uint8_t *raw, size_t len);

    uint32_t rxSeen = 0, rxDecoded = 0, rxDup = 0, rxIgnored = 0, txSent = 0;
    uint32_t contactsLearned = 0, dmRx = 0, dmTx = 0, dmAcked = 0, dmTimedOut = 0, groupEchoed = 0, groupTimedOut = 0;

    static constexpr size_t CONTACTS_MAX = 32;
    JenCoreContacts<CONTACTS_MAX> contacts;

  private:
    static constexpr size_t DEDUP_ENTRIES = 64;

    /// Cached channelHash() per channel slot, recomputed when the key bytes change.
    struct HashCacheEntry {
        uint8_t key[32] = {};
        size_t keyLen = 0;
        uint8_t hash = 0;
    };

    JenCoreDedup<DEDUP_ENTRIES> dedup; // hashFrame() never returns 0, the empty-slot marker
    HashCacheEntry hashCache[MAX_NUM_CHANNELS];
    uint32_t lastTxTimestamp = 0;

    uint8_t identitySeed[32] = {};
    uint8_t identityPub[32] = {};
    bool identityReady = false;

    /// A direct message on air whose ACK we are waiting for.
    struct PendingAck {
        uint32_t ackHash = 0;
        uint32_t packetId = 0;
        uint32_t contactNum = 0;
        uint32_t sentMs = 0;
        bool used = false;
        bool group = false; // a channel text waiting to be heard back, not a direct message waiting for an ACK
    };
    static constexpr size_t PENDING_ACKS = 8;
    PendingAck pending[PENDING_ACKS];

    void handleGroupText(const uint8_t *raw, size_t len, const JenCoreRepeater::FrameInfo &info, float snr, int32_t rssi);
    void handleAdvert(const uint8_t *raw, size_t len, const JenCoreRepeater::FrameInfo &info);
    void handleDirectText(const uint8_t *raw, size_t len, const JenCoreRepeater::FrameInfo &info, float snr, int32_t rssi);
    void handleAck(const uint8_t *raw, size_t len, const JenCoreRepeater::FrameInfo &info);
    void handlePathAck(const uint8_t *raw, size_t len, const JenCoreRepeater::FrameInfo &info);
    /// An ACK with this hash arrived (plain or inside a PATH): report the matching direct message delivered.
    void settleAck(uint32_t hash);

    /// Drop the unmessagable node an earlier channel text created for this name, now that the name is a contact.
    void retireGroupNode(const char *name);
    /// Drop the node an earlier build made for this contact under a public-key-derived number.
    void retireKeyedNode(const uint8_t pub[32], uint32_t keepNum);

    bool ensureSecret(JenCoreContact &c);
    void injectDirectText(const JenCoreContact &c, const char *text, int pathHashCount, float snr, int32_t rssi);
    void queueAck(uint32_t hash);
    /// Tell the client how a direct message ended, as if the contact itself had answered.
    void reportDelivery(uint32_t fromNum, uint32_t packetId, meshtastic_Routing_Error err);
    void addPending(uint32_t ackHash, uint32_t packetId, uint32_t contactNum, bool group = false);
    /// A repeater re-flooded the channel text with this frame hash: report it delivered.
    void settleEcho(uint32_t hash);
    void announcePosition(uint32_t nodeNum, int32_t latitudeI, int32_t longitudeI);

    struct RecentText {
        uint32_t packetId = 0;
        char sender[JenCoreContact::NAME_LEN] = {};
    };
    static constexpr size_t RECENT_TEXTS = 32;
    RecentText recent[RECENT_TEXTS];
    size_t recentNext = 0;

    /// Tell the client about a node we just created or renamed. The node DB alone is not enough: a connected phone only
    /// learns a node's name from a NODEINFO packet (or on its next config download), and otherwise shows "Meshtacore xxxx".
    void announceUser(uint32_t nodeNum, const meshtastic_User &user);

    bool cachedChannelHash(ChannelIndex ch, const uint8_t *key, size_t keyLen, uint8_t *out);
    bool wasRecentlySeen(uint32_t hash) const { return dedup.contains(hash); }
    void rememberFrame(uint32_t hash) { dedup.remember(hash); }
};

extern JenCoreCompanion jencoreCompanion;

#endif // MESHTASTIC_ENABLE_JENCORE_COMPANION
