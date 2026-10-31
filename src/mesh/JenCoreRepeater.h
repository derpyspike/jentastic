// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) 2026 Jennifer Sanchez
//
// Licensed under the GNU Affero General Public License v3.
// See LICENSE-AGPL for the full license text.
//

#pragma once

#include "JenCoreDedup.h"
#include "configuration.h"
#include <stddef.h>
#include <stdint.h>

#if NEXUS_ENABLE_JENCORE_SNIFFER

/**
 * Minimal, protocol-agnostic relay for JenCore LoRa frames.
 *
 * JenCore frames carry no Meshtastic PacketHeader: the on-air bytes are exactly its own
 * 1-byte packet header (route type in bits 0-1, payload type in bits 2-5, version in bits 6-7),
 * optionally 4 bytes of transport codes, one path_len byte, the path, then the payload. Flood
 * frames are relayed transparently (never decrypted - JenCore does the same in its own
 * simple_repeater example), with dedup and a hop-count bound. This only makes sense when the
 * Meshtastic node is on the same frequency and spreading factor as the JenCore network.
 *
 * Known limitation: the two protocols are not fully distinguishable by header bytes alone (both
 * look random). A JenCore-looking frame that is NOT addressed to/from us could still be a
 * Meshtastic unicast between two other nodes (a few percent of overheard DMs by length); those
 * are sniffed instead of routed. Broadcasts and DMs to/from this node are never misclassified.
 *
 * Region note: MeshCore scopes floods with a 16-bit transport code computed over the payload with a
 * key derived from the region name. We support a small region list (RX filtering + TX scope) and
 * optionally accept unscoped floods. Direct traffic is never region-filtered, as in MeshCore.
 */
class JenCoreRepeater
{
  public:
    static constexpr size_t MAX_FRAME_LEN = 255; // JenCore MAX_TRANS_UNIT

    /**
     * path_len byte of a flood we originate: no hops yet and the configured path-hash size (upper bits = size - 1).
     * The sender picks the size and every repeater appends hashes of that size, so this is what clients show as the
     * path width. MeshCore defaults to 3 bytes; 1/2 remain selectable for older networks.
     */
    static uint8_t ownPathLenByte();

    /// Master gate for the whole JenCore/MeshCore persona (sniffing, relay, companion, adverts). Default OFF.
    static bool isJenCoreEnabled();
    static void setJenCoreEnabled(bool en);

    /// Path-hash size (1..3) used in frames we originate. Default 3.
    static uint8_t getPathHashSize();
    static void setPathHashSize(uint8_t size);

    /// Automatic ADVERT schedule, in minutes. 0 = disabled. Both default OFF.
    static uint16_t getAdvertLocalMinutes();
    static uint16_t getAdvertFloodMinutes();
    static void setAdvertIntervals(uint16_t localMinutes, uint16_t floodMinutes);

    /// MeshCore flood hop limits (MeshCore defaults: 64 / 64 / 8).
    static uint8_t getFloodMax();
    static uint8_t getFloodMaxUnscoped();
    static uint8_t getFloodMaxAdvert();
    static void setFloodLimits(uint8_t maxHops, uint8_t maxUnscoped, uint8_t maxAdvert);

    /// Repeater discovery + neighbours.
    static constexpr size_t MAX_NEIGHBORS = 8;
    struct NeighborInfo {
        uint8_t pub[32];
        uint8_t type;
        int8_t snr4;
        uint32_t seenMs;
    };
    static uint32_t makeDiscoveryTag();
    static size_t buildDiscoveryRequest(uint8_t *out, size_t outCap, uint32_t tag);
    static void startDiscovery(uint32_t tag);
    static bool discoveryActive();
    static uint8_t neighborCount();
    static const NeighborInfo *neighbor(uint8_t index);

    /// MeshCore-like region scoping. Names are stored without the leading '#', max 4 regions.
    static constexpr size_t MAX_REGIONS = 4;
    static constexpr size_t REGION_NAME_LEN = 23;

    static bool addRegion(const char *name);
    static bool removeRegion(const char *name);
    static bool setHomeRegion(const char *name);
    static void clearRegions();
    static void setAllowUnscoped(bool allow);
    static uint8_t getRegionCount();
    static const char *getRegionName(uint8_t index);
    static uint8_t getHomeRegionIndex();
    static bool getAllowUnscoped();

    /// RX gate: true when this frame is inside our configured region(s), or scoping is disabled.
    static bool isFrameRegionAllowed(const uint8_t *raw, size_t len);

    /** JenCore MAX_PATH_SIZE: the flood path budget in bytes, shared by every JenCore node. */
    static constexpr size_t MAX_PATH_SIZE = 64;

    /** Parsed view of a JenCore frame header (see JenCore Dispatcher::tryParsePacket). */
    struct FrameInfo {
        uint8_t route_type;   // 0x00 transport-flood, 0x01 flood, 0x02 direct, 0x03 transport-direct
        uint8_t payload_type; // PAYLOAD_TYPE_* (0x00-0x0B, 0x0F)
        uint8_t payload_ver;  // must be 0 (PAYLOAD_VER_1)
        uint8_t path_hash_size;
        uint8_t path_hash_count;
        uint8_t header_len;  // bytes consumed by header + transport codes + path
        uint8_t payload_len; // remaining bytes
    };

    /// Index of the configured region a transport-flood matches, or 0xFF when unscoped/unmatched.
    static uint8_t resolveRegionIndex(const uint8_t *raw, size_t len, const FrameInfo &info);

    /**
     * True if the raw LoRa frame looks like a valid JenCore packet. Conservative on purpose:
     * must be >= 2 bytes, <= 255, payload version 0, a known payload type, and a valid path_len
     * byte (path mode 3 is reserved). Encrypted payloads must also have JenCore's shape: a MAC and whole 16-byte AES
     * blocks, which every real JenCore packet has and a Meshtastic frame misread as one almost never does.
     * A frame whose Meshtastic to/from NodeNum equals ourNodeNum
     * is never JenCore (crc32(pubkey) collision odds are 2^-32), so a Meshtastic unicast to or
     * from this node is kept out of the sniffer even when its bytes happen to parse.
     */
    static bool looksLikeJenCoreFrame(const uint8_t *raw, size_t len, uint32_t ourNodeNum, FrameInfo *out);

    /** Parse a JenCore frame header. Returns false if the frame is malformed. */
    static bool parse(const uint8_t *raw, size_t len, FrameInfo *out);

    /** FNV-1a over payload_type + payload only (path variants hash equal). 0 is never returned. */
    static uint32_t hashFrame(const uint8_t *raw, size_t len, const FrameInfo &info);

    /**
     * Handle a received JenCore frame (raw bytes as demodulated). Sniffs always; relays floods
     * if enabled. airtimeMs is the frame's on-air time (RadioInterface::getPacketTime), used to
     * size the adaptive relay delay.
     */
    void handleReceivedFrame(const uint8_t *raw, size_t len, float snr, uint32_t airtimeMs);

    /** True when at least one raw JenCore frame is queued for transmission. */
    bool hasPendingFrame() const;

    /** True when one of our own frames (ADVERT, discovery reply, companion text) is still queued. */
    bool hasPendingOwnFrame() const;

    /** True when a queued frame's adaptive delay has elapsed (and the post-TX hold-off is over): it may go out now. */
    bool isRelayDue() const;

    /** Milliseconds until a queued frame can go out (0 when one is due or nothing is queued). */
    uint32_t relayDueInMs() const;

    /**
     * Pick the frame to send next and copy it into buf; returns its length (0 if none). Due frames go first, best
     * priority first (routed traffic, then our own frames, then floods with the fewest hops). An ADVERT is only
     * signature-checked here, as late as possible and with the radio listening; a bad one is dropped and the next
     * frame is tried. The pick stays selected for pendingIsOwn(), markRelayed() and clearPendingFrame().
     */
    size_t takePendingFrame(uint8_t *buf, size_t bufLen);

    /** True when the frame picked by takePendingFrame() is one of ours rather than a relayed flood. */
    bool pendingIsOwn() const;

    /** Remove the picked frame (sent or refused): folds a relayed flood's dupe sample into the contention estimate. */
    void clearPendingFrame();

    /** The picked frame went on air: counts it and keeps the contention estimate from going stale. */
    void markRelayed();

    /**
     * Something was transmitted: hold the next JenCore frame back a few random slots, like Meshtastic's own post-TX
     * delay, and forget the busy-channel backoff.
     */
    void afterTransmit(uint32_t slotMs);

    /**
     * The channel (or the radio) was busy for a due frame: milliseconds to wait before the next attempt. A random number
     * of slots whose range doubles with every consecutive deferral, so repeaters that heard the same packet do not
     * come back together the moment the channel frees up.
     */
    uint32_t busyBackoffMs(uint32_t slotMs);

    /**
     * Install this node's JenCore identity from its 32-byte Ed25519 seed (public key is derived). Returns false, and
     * installs nothing, for a key JenCore rejects: a public key starting 0x00 or 0xFF (Identity::validatePrivateKey).
     */
    bool setIdentitySeed(const uint8_t seed[32]);

    /// JenCore node types an ADVERT can announce (AdvertDataHelpers.h). A chat node is a companion people can message; a
    /// repeater is infrastructure that JenCore apps will not start a conversation with.
    static constexpr uint8_t ADV_TYPE_CHAT = 0x01;
    static constexpr uint8_t ADV_TYPE_REPEATER = 0x02;

    /// What a Meshtastic device role announces itself as: ROUTER, ROUTER_LATE and REPEATER are repeaters, every other role
    /// is a chat node. role is a meshtastic_Config_DeviceConfig_Role value.
    static uint8_t nodeTypeForRole(int role);

    /// Node type our ADVERTs and discovery replies announce. Repeater until set.
    void setNodeType(uint8_t type) { nodeType = type; }
    uint8_t getNodeType() const { return nodeType; }

    bool hasIdentity() const { return hasIdentityKey; }
    const uint8_t *getIdentityPub() const { return identityPub; }
    const uint8_t *getIdentitySeed() const { return identitySeed; }

    static bool isRelayEnabled();
    static void setRelayEnabled(bool en);

    /** Check an ADVERT payload's Ed25519 signature over pub_key || timestamp || app_data. */
    static bool verifyAdvert(const uint8_t *payload, size_t payloadLen);

    /**
     * Encode an ADVERT for this node: repeater type, signed over pub||timestamp||app_data exactly as JenCore's
     * Mesh::createAdvert does. flood=false gives a zero-hop advert (route DIRECT, empty path) that only direct
     * neighbours hear, like a JenCore repeater's local advert. Returns the frame length, or 0 if it does not fit.
     */
    size_t buildAdvert(uint8_t *out, size_t outLen, const char *name, uint32_t timestamp, bool flood);

    /**
     * Queue one of our own frames (ADVERT, discovery reply, companion text), due after delayMs. Not counted as a relay
     * and not folded into the contention estimate. When the queue is full it takes the place of the worst queued flood;
     * returns false if there is none to give way (the queue holds only routed traffic and our own frames).
     */
    bool queueOwnFrame(const uint8_t *raw, size_t len, uint32_t delayMs = 0);

    /** Frames the relay can hold at once (ZephCore queues 32; floods are redundant, so a handful is enough here). */
    static constexpr size_t QUEUE_SIZE = 8;

    /** Adaptive jitter factor in [0.40, 1.00] (0.5 before warmup), from the observed dupes. */
    float getFloodDelayFactor();

    // Counters for observability
    uint32_t rxSeen = 0, rxFlood = 0, rxDup = 0, rxDropped = 0, txRelayed = 0, txDropped = 0, rxDirectForUs = 0,
             discoverReplies = 0;

    // Contention measurements: attempts that found the radio or the channel busy, and the airtime of JenCore frames.
    uint32_t deferRadio = 0, deferChannel = 0, txAirtimeMs = 0, rxAirtimeMs = 0;

  private:
    static constexpr uint8_t MAX_PATH_HASH_SIZE = 3; // JenCore allows 1..3
    static constexpr uint8_t MAX_PATH_HASH_COUNT = 63;
    // JenCore SimpleMeshTables: a ring of the last 160 packet hashes, with no time expiry. A copy arriving late (slow SF,
    // many hops) must still be recognised, or it would be relayed a second time.
    static constexpr size_t DEDUP_ENTRIES = 160;

    // JenCore's flood_max_advert default (NodePrefs.h): ADVERT floods stop being forwarded
    // after this many hops. flood_max itself (64) is implied by the 6-bit hop count plus the
    // MAX_PATH_SIZE bound above.
    static constexpr uint8_t MAX_FLOOD_HOPS_ADVERT = 8;

    // Adaptive relay delay (design follows ZephCore's ContentionTracker / computeAdaptiveFloodDelay,
    // MIT): jitter is 20ms + rand(0, 5 x airtime x factor), capped at 6 x airtime and 2000ms;
    // hearing a duplicate of the pending frame postpones it by 0.2 x airtime per dupe, capped at
    // 12 x airtime / 2000ms cumulative.
    static constexpr uint32_t DELAY_FLOOR_MS = 20;
    static constexpr float JITTER_AIRTIME_MULT = 5.0f;
    static constexpr float JITTER_AIRTIME_CAP = 6.0f;
    static constexpr uint32_t JITTER_CAP_MS = 2000;
    static constexpr float REACTIVE_BACKOFF_AIRTIME = 0.2f;
    static constexpr float REACTIVE_AIRTIME_CAP = 12.0f;
    static constexpr uint32_t REACTIVE_HARD_CAP_MS = 2000;
    static constexpr uint32_t DUPE_WINDOW_MS = 10000;

    // Contention estimate: EMA (x256 fixed point) of the per-relay dupe counts.
    static constexpr int EMA_SHIFT = 3;      // alpha = 1/8
    static constexpr int WARMUP_SAMPLES = 4; // samples before the EMA is trusted
    static constexpr float MIN_FLOOD_FACTOR = 0.40f;
    static constexpr float MAX_FLOOD_FACTOR = 1.00f;
    static constexpr float FLOOD_EST_HALFPOINT = 4.0f; // factor = 0.60 at est = 4
    // The estimate decays when the relay goes idle: one 1/8 step per period once the relay has
    // been quiet for STALE_MS, applied lazily whenever the factor is read (no timer).
    static constexpr uint32_t STALE_MS = 300000;
    static constexpr uint32_t DECAY_PERIOD_MS = 5000;
    static constexpr uint32_t MAX_DECAY_CATCHUP = 8;

    // Queue order: routed traffic first, then our own frames, then floods by hop count (fewest first, as ZephCore).
    static constexpr uint8_t PRIORITY_DIRECT = 0;
    static constexpr uint8_t PRIORITY_OWN = 1;
    static constexpr uint8_t PRIORITY_FLOOD_BASE = 2;

    // Busy-channel backoff and post-TX hold-off are counted in slots; never less than MIN_SLOT_MS, because the fast
    // presets have sub-10 ms slots that would turn the retry into a spin.
    static constexpr uint32_t MIN_SLOT_MS = 20;
    static constexpr uint8_t BUSY_BACKOFF_MAX_EXP = 4; // widest window: 2^4 = 16 slots
    static constexpr uint8_t HOLDOFF_MAX_SLOTS = 3;

    /** One queued frame together with what the contention estimate needs from it: its dupes are the sample. */
    struct QueuedFrame {
        uint8_t data[MAX_FRAME_LEN];
        uint8_t len = 0; // 0 marks a free slot
        bool own = false;
        bool needsVerify = false; // ADVERT whose signature is still to be checked
        uint8_t priority = 0;
        uint8_t dupeCount = 0;
        uint16_t reactiveAddedMs = 0;
        uint32_t hash32 = 0;
        uint32_t firstSeenMs = 0;
        uint32_t deadlineMs = 0;
        uint32_t airtimeMs = 0;
    };

    JenCoreDedup<DEDUP_ENTRIES> dedup; // hashFrame() never returns 0, the empty-slot marker

    QueuedFrame queue[QUEUE_SIZE];
    int selected = -1;         // index picked by takePendingFrame(), -1 when none
    uint32_t selectedHash = 0; // identity of that pick: a radio event in between may evict it and reuse its slot
    uint32_t holdoffUntilMs = 0;
    bool holdoffSet = false;
    uint8_t busyAttempts = 0;

    // This node's JenCore identity (Ed25519). The seed is the secret; the public key is derived from it.
    uint8_t identitySeed[32] = {0};
    uint8_t identityPub[32] = {0};
    bool hasIdentityKey = false;
    uint8_t nodeType = ADV_TYPE_REPEATER;

    uint32_t emaX256 = 0;
    uint16_t finalizedSamples = 0;
    uint32_t lastRelayMs = 0;
    uint32_t lastDecayMs = 0;

    bool wasRecentlySeen(uint32_t hash) const;
    void rememberFrame(uint32_t hash);

    // A JenCore repeater answers neighbour discovery at most 4 times per 2 minutes (simple_repeater discover_limiter).
    static constexpr uint8_t DISCOVER_MAX_REPLIES = 4;
    static constexpr uint32_t DISCOVER_WINDOW_MS = 120000;
    uint32_t discoverWindowStartMs = 0;
    uint8_t discoverRepliesInWindow = 0;

    struct RegionStore {
        uint8_t count = 0;
        uint8_t home = 0;
        bool allowUnscoped = true;
        char names[MAX_REGIONS][REGION_NAME_LEN + 1] = {};
    };
    static RegionStore &regionStore() { static RegionStore store; return store; }
    static bool isRegionAllowed(const uint8_t *raw, size_t len, const FrameInfo &info);
    static void regionKey(const char *name, uint8_t key[16]);
    static uint16_t transportCode(const uint8_t key[16], uint8_t payloadType, const uint8_t *payload, size_t len);
    /// Rewrite a FLOOD of ours to TRANSPORT_FLOOD with the home region's code; returns the used buffer/length.
    static const uint8_t *scopeOwnFrame(const uint8_t *raw, size_t len, uint8_t *out, size_t outCap, size_t *outLen);

    static bool &enabledStorage() { static bool enabled = false; return enabled; }
    static bool &relayStorage() { static bool enabled = true; return enabled; }
    static uint8_t &pathHashStorage() { static uint8_t size = 3; return size; }
    static uint16_t &advertLocalStorage() { static uint16_t minutes = 0; return minutes; }
    static uint16_t &advertFloodStorage() { static uint16_t minutes = 0; return minutes; }
    static uint8_t &floodMaxStorage() { static uint8_t v = 64; return v; }
    static uint8_t &floodMaxUnscopedStorage() { static uint8_t v = 64; return v; }
    static uint8_t &floodMaxAdvertStorage() { static uint8_t v = 8; return v; }
    static uint32_t &discoveryTagStorage() { static uint32_t v = 0; return v; }
    static uint32_t &discoveryUntilStorage() { static uint32_t v = 0; return v; }
    static uint8_t &neighborCountStorage() { static uint8_t v = 0; return v; }
    static NeighborInfo *neighborStorage() { static NeighborInfo list[MAX_NEIGHBORS] = {}; return list; }

    /** Our path hash (prefix of the public key) is the first hop of this direct frame's path. */
    bool isNextHopForUs(const uint8_t *raw, const FrameInfo &info) const;

    /**
     * Copy raw into out with its path rewritten: append our path hash (flood forward, Mesh::routeRecvPacket) or
     * drop the first hop (direct forward, Mesh::removeSelfFromPath). Returns the new length, 0 if it does not fit.
     */
    size_t rewritePath(const uint8_t *raw, size_t len, const FrameInfo &info, bool appendSelf, uint8_t *out, size_t outLen) const;

    /** JenCore floods only these payload types, each with a minimum payload (Mesh::onRecvPacket). */
    static bool isFloodForwardable(const FrameInfo &info);

    /** Zero-hop CONTROL frame: answer a NODE_DISCOVER_REQ for repeaters with our public key. */
    void handleControl(const uint8_t *raw, size_t len, const FrameInfo &info, float snr, uint32_t airtimeMs);
    void handleDirect(const uint8_t *raw, size_t len, const FrameInfo &info, float snr, uint32_t airtimeMs);
    /** TRACE: when the next hash in its payload list is ours, append our receive SNR to the path and forward it. */
    void handleTrace(const uint8_t *raw, size_t len, const FrameInfo &info, float snr, uint32_t airtimeMs);
    /** MULTIPART ACK for us as next hop: forward it as a plain direct ACK. */
    void handleMultipartAck(const uint8_t *raw, size_t len, const FrameInfo &info, uint32_t airtimeMs);
    static uint32_t directDelayMs(uint32_t airtimeMs);
    void onDupeHeard(uint32_t hash);
    bool canRelay() const;

    /** Queue a frame to relay; false (and nothing remembered) when the queue is full of better frames. */
    bool queueRelay(const uint8_t *raw, size_t len, uint32_t hash32, uint32_t airtimeMs, uint32_t delayMs, uint8_t priority,
                    bool needsVerify = false);
    /** Index of a free slot, evicting the worst queued flood that ranks below `priority` if none is free; -1 if none. */
    int claimSlot(uint8_t priority);
    /** Index of the frame to send next: best priority among the due ones, else the soonest to become due. -1 if empty. */
    int pickNext() const;
    void release(int index);
    /** Index of the frame takePendingFrame() picked (or would pick), -1 if it is gone from the queue. */
    int selectedIndex() const;
    /** Drop floods that waited longer than a copy of them could still matter (they were never forwarded). */
    void expireStale();
    void foldContentionSample(const QueuedFrame &f);
    void applyLazyDecay();
    uint32_t adaptiveDelayMs(uint32_t airtimeMs);
};

#endif // NEXUS_ENABLE_JENCORE_SNIFFER
