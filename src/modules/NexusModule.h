// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) 2026 Jennifer Sanchez
//
// Licensed under the GNU Affero General Public License v3.
// See LICENSE-AGPL for the full license text.
//
// NexusModule — remote CLI module for Meshtastic firmware

#pragma once
#include "Channels.h"
#include "concurrency/OSThread.h"
#include "configuration.h"
#include "detect/ScanI2CConsumer.h"
#include "input/InputBroker.h"
#include "mesh/MeshModule.h"
#include "mesh/generated/meshtastic/mesh.pb.h"
#include "modules/NexusMultiSF.h"
#include "modules/NexusRuntime.h"
#include <vector>

namespace meshtastic {

static constexpr size_t NEXUS_MAX_REPLY = 200;
static constexpr size_t NEXUS_MAX_REPLY_BODY = NEXUS_MAX_REPLY - 1;

// Nexus v66: Feature toggles (uint64 bitmask, persisted in NexusVault::features_mask).
// Bit semantics — stable IDs; new bits append at the high end, never reuse a bit number.
//   bit0  FORWARD_OPAQUE_TO_PHONE       — forward encrypted packets we cannot decode
//                                         (private channels, foreign PKI we can't decrypt)
//                                         to the phone via service->handleFromRadio().
//                                         Restores 2.7 phone-visibility under v2.8's
//                                         OPAQUE_RELAY_ONLY short-circuit in Router.cpp.
//                                         Independent of relay decision (CLIENT_MUTE
//                                         still receives, just doesn't relay).
//   bit1  FORWARD_FOREIGN_PKI_TO_PHONE  — sub-feature of bit0 for un-decryptable PKI DMs
//                                         from other nodes (foreign sender, we lack
//                                         the key). Lets operators see "PKI traffic
//                                         exists" on the phone without seeing plaintext.
//   bit2  DISABLE_AGC_RESET             — no-op the periodic radio AGC reset
//                                         (RadioLibInterface::periodicRadioMaintenance
//                                         skips resetAGC() every 60 s tick; RX re-arm
//                                         recovery still runs). Off by default.
static constexpr uint64_t FEATURE_FORWARD_OPAQUE_TO_PHONE      = (1ULL << 0);
static constexpr uint64_t FEATURE_FORWARD_FOREIGN_PKI_TO_PHONE = (1ULL << 1);
static constexpr uint64_t FEATURE_DISABLE_AGC_RESET            = (1ULL << 2);
//   bit3  LATEBASE                      — CLIENT_BASE repeats with ROUTER_LATE
//                                         timing: restores the upstream v2.8.1
//                                         clampToLateRebroadcastWindow() block in
//                                         FloodingRouter::perhapsCancelDupe() for
//                                         favorited from/to packets. Off (default)
//                                         = JT behavior: client/router window.
static constexpr uint64_t FEATURE_LATEBASE                     = (1ULL << 3);

// Nexus v67 nexusVariantBits carve-out. Each variant that needs persisted
// state gets its own bit range from this 64-bit field. Layout is append-only:
// new variants carve from the high end so existing saved vaults stay valid.
//
// Today = station-g3 (a.k.a. G3) PA/LNA GPIO control. The two pins get three
// runtime states (DISABLED / OFF / ON) but only OFF and ON persist; DISABLED
// is re-detected every boot via raw digitalRead() before any pinMode() call.
//
//   [1:0]   PA       00=OFF,  01=ON,  10/11 reserved (saved as VAULT_PA_OFF / VAULT_PA_ON)
//   [3:2]   LNA      00=OFF,  01=ON,  10/11 reserved (saved as VAULT_LNA_OFF / VAULT_LNA_ON)
//   [5:4]   LNA2     reserved for future variants that have a second LNA slot
//                     (station-g3 is hard-wired without LNA2 — never read on this board)
//   [63:6]  future variants, must stay 0 on station-g3
//
// Off-the-shelf bit values stay 0 — a fresh vault or v66 → v67 migration boots
// into the safe default (PA off, LNA bypassed, no RF surprises).
static constexpr uint64_t VARIANT_PA_MASK        = 0x3ULL;   // bits [1:0]
static constexpr uint64_t VARIANT_PA_OFF         = 0x0ULL;
static constexpr uint64_t VARIANT_PA_ON          = 0x1ULL;
static constexpr uint64_t VARIANT_LNA_MASK       = 0xCULL;   // bits [3:2]
static constexpr uint64_t VARIANT_LNA_SHIFT      = 2;
static constexpr uint64_t VARIANT_LNA_OFF        = 0x0ULL;
static constexpr uint64_t VARIANT_LNA_ON         = 0x1ULL;
static constexpr uint64_t VARIANT_LNA2_MASK      = 0x30ULL;  // bits [5:4] reserved
static constexpr uint64_t VARIANT_LNA2_SHIFT     = 4;

/**
 * Nexus Module: Stealth-optimized coordinator for identity management,
 * tactical network analysis (NETSCAN), and high-granularity firewalling.
 */
class NexusModule : public MeshModule,
                    public concurrency::OSThread,
                    public ScanI2CConsumer {
public:
    NexusModule();

    virtual bool wantPacket(const meshtastic_MeshPacket *p) override;
    virtual ProcessMessage handleReceived(const meshtastic_MeshPacket &mp) override;
    ProcessMessage handleLocal(const meshtastic_MeshPacket &mp);
    virtual int32_t runOnce() override;

    /// Init-time side effects that the module framework does not reach: the framework never
    /// calls MeshModule::setup(), so the constructor calls this after loadVault().
    void initOnce();

    // Firewall/Drop Logic
    bool shouldDropPacket(const meshtastic_MeshPacket &mp);
    bool shouldInvalidatePacket(const meshtastic_MeshPacket &mp);
    bool shouldDropPrivate(const meshtastic_MeshPacket &mp);
    bool shouldDropOutput(const meshtastic_MeshPacket &mp);
    bool isNexusIgnored(uint32_t nodeID);
    // Wire-shape heuristic for foreign PKI packets we couldn't decode (no destination key).
    // The mp.pki_encrypted flag is only set on successful PKI decrypt (Router.cpp:1052), so for
    // un-decodable foreign PKI DMs the firewall must look at the wire shape (channel 0 on the
    // wire, encrypted, addressed to a specific non-us node) to apply /nexus FIREWALL PKI rules.
    // See looksLikeForeignPKI() in NexusModule.cpp for the full predicate. (2026-09-04.)
    bool looksLikeForeignPKI(const meshtastic_MeshPacket &mp) const;


    // Nexus-28003 (2026-08-24): bit definitions for the
    // nexus_security_bitmask (uint32_t, vault v65 slot at offset ~24 B
    // from start, see NexusVault struct definition below). All bits
    // default OFF. Toggled via /nexus SECURITY; persisted via the
    // existing /nexus SAVE / /nexus LOAD round-trip (no v66 vault bump
    // needed — the slot has been there since v65).
    //
    // Bit 0 — REQ_SIG: when ON, /nexus commands received on a Nexus
    //   channel (Nexus / Nexus2) must carry a valid XEdDSA signature
    //   from a trusted key (admin_key[0..2], secondary_keys[0..3], or
    //   NEXUS_MASTER_KEY_1..4). Verified via verifyTrustedSignature().
    //   Off-by-default to preserve Nexus-28001 channel-only semantics
    //   (anyone on the Nexus channel can issue /nexus).
    //
    // Bit 1 — ALLOW_DM: when ON, /nexus commands received as DMs
    //   (to == my_node_num) are accepted. DMs are NEVER signed in the
    //   Meshtastic PKI/admin model — they only carry the sender's
    //   public_key — so when this bit is set the sig rule is skipped
    //   for DMs and authorization falls back to isAuthorizedAdmin() on
    //   the sender's pubkey. Off-by-default to preserve the Nexus-28001
    //   "channel-only" rule that DMs are NEVER an authorized /nexus path.
    //
    // Bit 2 — SILENT_LOG: when ON, packets rejected by REQ_SIG /
    //   ALLOW_DM are silently logged (LOG_WARN) and dropped without
    //   any reply. When OFF, the operator receives a 'JT: SECURITY: ...'
    //   reply explaining why their command was rejected (useful for
    //   debugging, lets an attacker probe the bitmask state).
    //   Off-by-default to favor operator UX. Operators wanting
    //   stricter silence flip this ON.
    //
    // Bits 3..31 — reserved for future security flags. Reads via the
    //   bit constants (not magic numbers) so adding new bits is a
    //   single-file edit.
    static constexpr uint32_t NEXUS_SECURITY_REQUIRE_SIG    = (1u << 0);
    static constexpr uint32_t NEXUS_SECURITY_ALLOW_DM       = (1u << 1);
    static constexpr uint32_t NEXUS_SECURITY_SILENT_LOG    = (1u << 2);
    // bit 3 (Nexus-28004, 2026-08-30): when ON, any decoded packet
    //   seen on a Nexus channel from a non-self node is auto-favourited.
    //   No flash save — operator must run /nexus SAVE to persist.
    static constexpr uint32_t NEXUS_SECURITY_AUTO_FAV_NEXUS  = (1u << 3);
    // bit 4 (Nexus-28004, 2026-08-30): when ON, any decoded packet
    //   seen on a Nexus channel that carries a valid Nexus-trusted signature
    //   (verifyTrustedSignature() succeeds) auto-favourites the sender. Same
    //   no-flash-save policy as AUTO_FAV_NEXUS. verifyTrustedSignature() cost
    //   is bounded because the gate is still "Nexus channel only".
    static constexpr uint32_t NEXUS_SECURITY_AUTO_FAV_TRUSTED = (1u << 4);
    // bit 5 (Nexus-28005, 2026-09-06): Bypass_RP opt-out from replay
    // protection. INVERTED semantics: bit CLEAR = RP active (default, matches
    // "always on unless opted out"); bit SET = RP bypassed. Existing v65
    // vaults load with bit clear, so already-deployed nodes boot with
    // replay protection ON. When bypassed, the checkReplay() guard is
    // skipped entirely -- the ring is not searched AND not filled, so
    // toggling back off resumes from the exact ring state at bypass time.
    static constexpr uint32_t NEXUS_SECURITY_BYPASS_RP = (1u << 5);
    // bit 6+7-9 (2026-10-07): AUTO_FAV_EXTRA — like AUTO_FAV_NEXUS but for
    // ONE operator-chosen channel. Bit 6 = enable; bits 7-9 pack the channel
    // index (0-7) into the same bitmask ("store the channel as well as the
    // enable bit"). Persisted with /nexus SAVE like the other bits; old vaults
    // load with 0 = disabled / channel 0.
    static constexpr uint32_t NEXUS_SECURITY_AUTO_FAV_EXTRA = (1u << 6);
    static constexpr uint32_t NEXUS_SECURITY_AUTO_FAV_EXTRA_CH_SHIFT = 7;
    static constexpr uint32_t NEXUS_SECURITY_AUTO_FAV_EXTRA_CH_MASK = (0x7u << 7);

    // Accessors for the auth gate. Inline so the compiler folds the
    // mask check into a single load+test. Returns false when the bit
    // is clear (i.e., the pre-Nexus-28003 default is "no extra rule").
    bool securityRequiresSignature()  const { return (nexus_security_bitmask & NEXUS_SECURITY_REQUIRE_SIG)    != 0; }
    bool securityAllowsDM()           const { return (nexus_security_bitmask & NEXUS_SECURITY_ALLOW_DM)       != 0; }
    bool securitySilentLog()          const { return (nexus_security_bitmask & NEXUS_SECURITY_SILENT_LOG)    != 0; }
    bool securityAutoFavNexus()      const { return (nexus_security_bitmask & NEXUS_SECURITY_AUTO_FAV_NEXUS) != 0; }
    bool securityAutoFavTrusted()    const { return (nexus_security_bitmask & NEXUS_SECURITY_AUTO_FAV_TRUSTED) != 0; }
    bool securityBypassRP()          const { return (nexus_security_bitmask & NEXUS_SECURITY_BYPASS_RP)       != 0; }
    bool securityAutoFavExtra()      const { return (nexus_security_bitmask & NEXUS_SECURITY_AUTO_FAV_EXTRA)  != 0; }
    uint32_t securityAutoFavExtraChannel() const {
        return (nexus_security_bitmask & NEXUS_SECURITY_AUTO_FAV_EXTRA_CH_MASK) >> NEXUS_SECURITY_AUTO_FAV_EXTRA_CH_SHIFT;
    }

    bool checkMasterKeys(const uint8_t *senderPubKey);

    // Nexus-28002 (2026-08-24): single-call "is this sender
    // authorized to admin this node?" helper. Wraps the full set of
    // trusted admin keys into one pubkey-byte-compare so AdminModule
    // and any other PKC gate can call a single function instead of an
    // inline OR chain over the three sources. PKI doesn't carry
    // signatures, so we compare pubkey bytes directly — NOT a
    // signature verifier (see verifyTrustedSignature() for that).
    //
    // Returns true if senderPubKey matches ANY of:
    //   - config.security.admin_key[0..2]   (per-device, baked via
    //                                        USERPREFS_USE_ADMIN_KEY_*
    //                                        / build flags, 32-byte
    //                                        length = "loaded" gate)
    //   - NEXUS_MASTER_KEY_1..4             (build-time hardcoded,
    //                                        covered via checkMasterKeys)
    //   - nexus_secondary_keys[0..3]        (vault-loaded runtime,
    //                                        all-zero sentinel =
    //                                        uninitialized, covered
    //                                        via checkMasterKeys)
    //
    // Replaces the inline 4-line OR chain in AdminModule.cpp's PKC
    // admin auth gate. Same semantics as the Nexus-269 inline pattern,
    // plus the secondary_keys check that was added to checkMasterKeys
    // on the 28002 line.
    //
    // Null pubkey returns false (defensive — upstream callers should
    // never pass null, but the inline chain would have segfaulted).
    bool isAuthorizedAdmin(const uint8_t *senderPubKey);

    // Nexus-28002 (2026-08-20): verify a packet's XEdDSA
    // signature against the Nexus trust list (admin_key[0..2],
    // nexus_secondary_keys[0..3], NEXUS_MASTER_KEY_1..4) and report
    // which key verified it. First-match-wins. Intended as the
    // signature-based authentication path for future Nexus command
    // authorization (alongside checkMasterKeys() which is a plain
    // pubkey memcmp on the packet's carried public_key).
    //
    // Returns true if ANY of the candidate keys verifies the
    // packet's signature. On match, fills outMatch (4 bytes minimum
    // — pass 8 to be safe) with a shorthand label:
    //   "A<n>"  admin_key[n]                  (n = 0..2)
    //   "S<n>"  nexus_secondary_keys[n]       (n = 0..3)
    //   "M<n>"  NEXUS_MASTER_KEY_<n>          (n = 1..4, matches build-flag numbering)
    //
    // Returns false with outMatch="no_signature" if the packet has
    // no signature field (or a malformed one — anything other than
    // a full 64-byte XEdDSA signature). Returns false with
    // outMatch="none" if a signature is present but no candidate
    // key verifies.
    bool verifyTrustedSignature(const meshtastic_MeshPacket *p, char *outMatch, size_t outSize);

    uint64_t getNexusDropHex();
    void setNexusDropHex(uint64_t h);
    uint32_t getNexusInvalidHex();
    void setNexusInvalidHex(uint32_t h);
    uint64_t getNexusOutDropHex();
    void setNexusOutDropHex(uint64_t h);
    uint64_t getNexusFeaturesHex();
    void setNexusFeaturesHex(uint64_t h);

    // Public state for visibility
    bool public_ping = false;
    bool ping_rssi = false;
    bool ping_snr = false;
    bool ping_freqerr = false;
    bool ping_bitmask = true;
    bool ping_ignore_direct = false;
    bool ping_ignore_remote = false;
    bool ping_allow_dm = false;       // /ping DM toggle (v66, bit 15 of ping_mask)
    bool ping_channels[MAX_NUM_CHANNELS] = {true, false, false, false, false, false, false, false};

    // Firewall state
    bool drop_nodeinfo = false;
    bool drop_position = false;
    bool drop_telemetry = false;
    bool drop_neighbor = false;
    bool drop_traceroute = false;
    bool drop_traceroute_remote = false;
    bool drop_traceroute_local = false;
    bool drop_traceroute_direct = false;
    bool drop_traceroute_direct_remote = false;
    bool drop_traceroute_direct_local = false;
    bool drop_private = false;
    bool drop_hop6 = false;
    bool drop_hop7 = false;
    bool drop_req_nodeinfo = false;
    bool drop_req_telemetry = false;
    bool drop_req_position = false;
    bool drop_acks = false;
    bool drop_pki = false;
    bool drop_pki_remote = false;
    bool drop_pki_local = false;
    bool drop_prehop = false;
    bool drop_store_forward = false;
    bool drop_admin = false;
    bool drop_text = false;
    bool drop_keyver = false;
    bool drop_alert = false;
    bool drop_waypoint = false;
    bool drop_rl_text = false; // Bit 29
    bool drop_rl_trace = false; // Bit 30
    bool drop_rl_other = false; // Bit 31
    bool drop_rl_nodeinfo = false; // Bit 42
    bool drop_rl_position = false; // Bit 43
    bool drop_rl_telemetry = false; // Bit 44
    bool drop_rl_private = false; // Bit 45

    // Broadcast-specific request filters (Bits 46-48)
    // Drops the corresponding want_response packet only when sent to broadcast
    // (mp.to == NODENUM_BROADCAST). Non-broadcast (DM) requests pass through.
    bool drop_br_nodeinfo = false;   // Bit 46
    bool drop_br_telemetry = false;  // Bit 47
    bool drop_br_position = false;   // Bit 48

    // Ping emoji response mode (Bits 40-41 of drop_mask)
    // 0 = text (default), 1 = emoji if indirect, 2 = always emoji
    uint8_t ping_emoji_mode = 0;

    // v43: broadcast reply / paging max delay (seconds). Floor stays at 100ms;
    // jitter range is 100..N*1000 ms. Default 5, range [1, 300].
    uint16_t broadcast_delay_max_s = 5;

    // Telemetry Granular Drops (Bits 32-35)
    bool drop_tele_device = false;
    bool drop_tele_power = false;
    bool drop_tele_env = false;
    bool drop_tele_other = false;

    // MQTT Drops (Bit 36)
    bool drop_no_mqtt = false;

    // Position Granular Drops (Bits 37-39)
    bool drop_pos_manual = false;
    bool drop_pos_gps = false;
    bool drop_pos_other = false;
    bool fav_bypass = false;

    // Invalid Filters (v29)
    bool inv_null_island = false;
    bool inv_out_spain = false;
    bool inv_out_germany = false;
    bool inv_hw_unset = false;
    bool inv_id_mismatch = false;

    uint16_t rl_text_limit = 0;
    uint16_t rl_trace_limit = 0;
    uint16_t rl_other_limit = 0;
    uint16_t rl_nodeinfo_limit = 0;
    uint16_t rl_position_limit = 0;
    uint16_t rl_telemetry_limit = 0;
    uint16_t rl_private_limit = 0;
    float rl_tokens[7] = {0, 0, 0, 0, 0, 0, 0}; // 0:TX 1:TR 2:OT 3:NI 4:PO 5:TL 6:PR
    uint32_t rl_last_refill = 0;

    bool checkRateLimit(const meshtastic_MeshPacket &mp);

    // Output Firewall state
    bool drop_out_nodeinfo = false;
    bool drop_out_position = false;
    bool drop_out_telemetry = false;
    bool drop_out_traceroute = false;
    bool drop_out_admin = false;
    bool drop_out_routing = false; // Bit 5 (v31)

    bool trace_alert = false;
    uint8_t trace_alert_mode = 2; // 0=Direct, 1=Relay, 2=Both
    bool meta_alert = false;
    uint8_t meta_alert_mode = 2; // 0=Direct, 1=Relay, 2=Both
    bool skip_skip_handle = false; // Reversed toggle for skipHandle logic

    // NETSCAN state
    bool netscan_active = false;
    uint32_t netscan_start = 0;
    uint32_t netscan_reply_to = 0;
    uint32_t netscan_reply_ch = 0;

    // CLIENTSCAN state
    struct NexusClientScanEntry {
        uint32_t nodeID;
        uint32_t delay_ms;
        int8_t rssi;
        float snr;
        float freqErr;
        uint32_t relay;
        uint8_t hop_limit;
        uint8_t hop_start;
    };
    bool clientscan_active = false;
    uint32_t clientscan_start = 0;
    uint32_t clientscan_target_id = 0;
    uint32_t clientscan_duration_ms = 100000;
    NexusClientScanEntry clientscan_db[50];
    uint8_t clientscan_count = 0;

    void sniffPacket(const meshtastic_MeshPacket &mp);
    void updateTargetStats(const meshtastic_MeshPacket &mp);

    struct NexusTargetStats {
        uint32_t nodeID = 0;
        uint32_t packetCount = 0;
        uint32_t byteCount = 0;
        uint32_t duplicateCount = 0;
        float sumRssi = 0;
        float sumSnr = 0;
        float sumFreqErr = 0;
        float sumRssi2 = 0;
        float sumSnr2 = 0;
        float sumFreqErr2 = 0;
        int8_t minRssi = 0;
        int8_t maxRssi = -128;
        float minSnr = 100.0f;
        float maxSnr = -100.0f;
        float minFreqErr = 1000000.0f;
        float maxFreqErr = -1000000.0f;
        uint32_t lastRelay = 0;
        uint32_t primaryRelay = 0;
        uint32_t primaryRelayCount = 0;
        uint32_t lastSeen = 0;
        uint32_t lastPacketTime = 0;
        uint32_t minInterval = 0xFFFFFFFF;
        uint32_t maxInterval = 0;
        uint32_t hopDist[8] = {0};
        uint32_t portDist[8] = {0}; // 0:NI, 1:POS, 2:TEL, 3:TXT, 4:ADM, 5:RT, 6:SF, 7:OTH
    };
    NexusTargetStats targetWatchlist[8];
    uint8_t watchlistCount = 0;

    struct ReplyContext {
        uint32_t to = 0xFFFFFFFF;
        uint32_t channel = 0;
        uint32_t replyId = 0;
        bool phoneOnly = false;
        bool copyToPhone = true;
    };
    ReplyContext traceReply, netscanReply, clientscanReply, noiseReply;
    bool sendReply(const ReplyContext &context, const char *message, size_t length, bool reliable = true);

    // Trace context
    bool trace_active = false;
    bool input_hooked = false;
    uint32_t trace_start_time = 0;
    uint32_t trace_reply_to = 0;
    uint32_t trace_reply_channel = 0;

    // Hop Masking
    bool hop_mask_active = false;
    uint8_t hop_mask_start = 3;
    uint8_t hop_mask_limit = 3;

    // Internal state
    int8_t M_S = -1; //-1 = OFF

    bool client_always = false;
    bool base_texts = false;

    // Internal state
    uint8_t R_S_mode = 0;
    uint8_t R_S_id = 0;

    uint8_t T_S_mode = 0;
    uint32_t T_S_id = 0;

    struct __attribute__((packed)) NexusCronEntry {
        uint8_t  active;      // 0: Empty, 1: Active
        uint8_t  mode;        // 0: PER (Periodic), 1: SCH (Schedule), 2: AT (Run-Once)
        uint32_t value;       // PER: Interval (s), SCH: Sec-of-Day, AT: Unix TS
        uint32_t last_run;    // Timestamp of last execution
        char     command[64]; // Command string (Updated to 64 for complex commands)
    };
    NexusCronEntry cron_table[8] = {{0}};
    uint32_t cronLastRunMs[8] = {};

    struct __attribute__((packed)) NexusAliasEntry {
        uint32_t nodeID;
        char alias[5]; // 4 chars + null
    };

    struct __attribute__((packed)) NexusMacroEntry {
        char name[12];
        char command[128];
    };

    struct __attribute__((packed)) NexusHopMask {
        uint8_t active;
        uint8_t start;
        uint8_t limit;
    };

    struct __attribute__((packed)) NexusSignature {
        uint8_t bytes[16]; // Signature bytes (up to 16)
        uint8_t len;       // 0 = slot empty/disabled
        uint8_t type;      // 0=hex, 1=base64, 2=text (for display only)
    };

    enum NexusVoltAction : uint8_t {
        NEXUS_VOLT_ACT_NONE      = 0,
        NEXUS_VOLT_ACT_CMD       = 1,
        NEXUS_VOLT_ACT_HIBERNATE = 2,
        NEXUS_VOLT_ACT_WAKE      = 3
    };

    enum NexusVoltDir : uint8_t {
        NEXUS_VOLT_DIR_HIGH = 0, // >= target
        NEXUS_VOLT_DIR_LOW  = 1  // <= target
    };

    struct __attribute__((packed)) NexusVoltRule {
        uint16_t mv;          // Target millivolts
        uint8_t  action;      // NexusVoltAction
        uint8_t  direction;   // NexusVoltDir
        uint8_t  channel;     // 0 = default battery / INA CH1, 1 = INA CH2, 2 = INA CH3
        uint8_t  flags;       // Reserved / trigger tracking
        char     command[32]; // Command string
    };

    enum NexusSplitMode : uint8_t {
        NEXUS_SPLIT_OFF          = 0,
        NEXUS_SPLIT_ROOF_SPLIT   = 1, // Single-RX rooftop: RX on primary, downlink on altPreset, indoor 0-hop uplink on primary
        NEXUS_SPLIT_CLIENT_SPLIT = 2, // Indoor client: RX on primary, all TX on altPreset
        NEXUS_SPLIT_ROOF_MSF     = 3  // LR2021 concurrent RX: Primary=low SF (indoor), SideDet=high SF (mesh), cross-repeats
    };

    enum NexusSplitFlags : uint8_t {
        NEXUS_SPLIT_FLAG_USE_ZH_MASK = (1 << 0), // Only treat nodes in zero_hop_mask as indoor nodes (default in ROOF_SPLIT)
        NEXUS_SPLIT_FLAG_INDOOR_ECHO = (1 << 1), // Rooftop also echoes indoor->indoor packets on the indoor preset
        NEXUS_SPLIT_FLAG_MESH_RELAY  = (1 << 2), // In ROOF_MSF, also relay outside SF7->SF7 in addition to SF7->SF6 downlink
        NEXUS_SPLIT_FLAG_APC_EN      = (1 << 3)  // Enable SNR-based per-packet APC on unicast direct hops
    };

    struct __attribute__((packed)) NexusSplitConfig {
        uint8_t mode;            // NexusSplitMode (0..3)
        uint8_t flags;           // NexusSplitFlags bitmask
        uint8_t alt_sf;          // Secondary preset SF (5..12)
        uint8_t alt_cr;          // Secondary preset CR (5..8)
        uint16_t alt_bw;         // Secondary preset BW code (31, 62, 125, 250, 400, 500, 800, 1600)
        float alt_freq;          // Secondary preset frequency in MHz (e.g. 869.6180f, 867.5812f)
        int8_t alt_tx_power;     // TX power (dBm) when transmitting on secondary/indoor tier (e.g. -9)
        int8_t apc_min_power;    // Minimum TX power (dBm) for APC step-down (default 0)
    }; // Total: 12 bytes

    // v71: JenRNode (RNode) and JenCore/MeshCore persona state.
    struct __attribute__((packed)) NexusJenConfig {
        uint8_t rnode_mode;        // RNodeMode: 0 OFF, 1 BRIDGE, 2 STANDALONE
        uint8_t rnode_tcp;         // TCP KISS server enabled
        uint8_t rnode_ble;         // NUS BLE persona enabled
        uint8_t rnode_sf;          // Reticulum tier SF (5..12)
        uint8_t jencore_enabled;   // master JenCore switch
        uint8_t jencore_relay;     // flood relay
        uint8_t jencore_path_hash; // 1..3 bytes
        uint8_t reserved;
    }; // 8 bytes

    // v72: MeshCore/JenCore region scoping.
    struct __attribute__((packed)) NexusJenRegions {
        uint8_t count;         // 0..4
        uint8_t home;          // index used for TX scope
        uint8_t allow_unscoped; // accept plain ROUTE_FLOOD
        uint8_t reserved;
        char names[4][24];     // stored without leading '#'
    }; // 100 bytes

    // Nexus Storage Vault structure (Fixed-length for simplicity and speed)
    struct __attribute__((packed)) NexusVault {
        char magic[4];       // "JenT"
        uint32_t version;    // 40
        uint64_t drop_mask;
        uint16_t ping_mask;  // Bit 0: Global, Bits 1-8: Channels
        uint8_t  airtag_enabled;
        char     airtag_key[28];
        char     groups[4][8]; // 4 slots, 8 chars each
        uint8_t  hop_mask_active;
        uint8_t  hop_mask_start;
        uint8_t  hop_mask_limit;
        int8_t   M_S;
        uint8_t  client_always;
        uint8_t  base_texts;
        uint8_t  R_S_mode;
        uint8_t  R_S_id;
        uint8_t  secondary_keys[4][32]; // v8, expanded to 4 slots in v65
        uint32_t ch_whitelist[16];      // v14: Expanded to 16
        uint32_t ch_blacklist[16];      // v14: Expanded to 16
        uint8_t  dm_forward;            // Added in v10
        uint32_t v11_id;             // Added in v11
        uint8_t  v11_mode;           // Added in v11
        uint8_t  T_S_mode;      // Added in v12
        uint32_t T_S_id;        // Added in v12
        NexusCronEntry cron_table[8];   // Added in v13 (Increased to 8 in v23)
        uint8_t  custom_pping_active;   // Added in v15
        char     pping_header[64];      // Added in v15
        uint8_t  trace_alert;           // Added in v16
        uint8_t  trace_alert_mode;      // Added in v16
        uint8_t  trusted_ch_active[2];  // Added in v17
        char     trusted_ch_names[2][16]; // Added in v17
        uint8_t  meta_alert;            // Added in v18
        uint8_t  meta_alert_mode;      // Added in v18
        uint8_t  skip_skip_handle;      // Added in v19
        uint64_t out_drop_mask;         // Added in v20
        uint16_t nexus_ignore_count;    // Added in v21
        uint32_t nexus_ignore_list[64]; // Added in v21 (Resized to 64)
        uint8_t  zero_hop_config;       // Added in v22 (Bit 0: EN, Bit 1: TELE, Bit 2: POS, Bit 3: AUTO_ROUTER, Bit 4: AUTO_CLIENT)
        uint8_t  zero_hop_mask[32];     // Added in v22 (256 bits)
        uint8_t  relay_drop_mask[32];   // Added in v24 (256 bits)
        uint8_t  ignore_dupe_relay_mask[32]; // Added in v24 (256 bits)
        uint8_t  mqtt_override_mode;    // Added in v25 (0: Default, 1: Always Upload, 2: Strict Privacy)
        uint8_t  ina_legacy_math;       // Added in v26
        uint8_t  B_S_active;     // v27: active
        float    B_S_offset;     // v27: offset
        uint16_t rl_text_limit;        // v28: Rate limit pkts/hr
        uint16_t rl_trace_limit;       // v28: Rate limit pkts/hr
        uint16_t rl_other_limit;       // v28: Rate limit pkts/hr
        uint16_t rl_nodeinfo_limit;    // v38: Rate limit pkts/hr
        uint16_t rl_position_limit;    // v38: Rate limit pkts/hr
        uint16_t rl_telemetry_limit;   // v38: Rate limit pkts/hr
        uint16_t rl_private_limit;     // v38: Rate limit pkts/hr
        uint32_t invalid_mask;         // v29: Invalid filter mask
        NexusAliasEntry aliases[16];   // v30: Node aliases
        NexusMacroEntry macros[4];     // v30: Macro system
        NexusHopMask per_ch_hopmask[8];   // v31: Per-channel hopmask
        NexusHopMask per_port_hopmask[5]; // v31: Per-port hopmask (NI, TL, PO, TR, TX)
        uint8_t  t_m;             // v33 slot
        uint8_t  s_o_active;     // v34 slot
        float    s_o_value;      // v34 slot
        uint16_t preamble_override;       // v35: Custom Preamble Length
        uint64_t gpio_watch_mask;         // v37: GPIO Watch bitmask
        NexusSignature signatures[8]; // v39: Signature firewall (8 slots x 18 bytes)
        uint8_t  syncword_override;       // v40: Custom Sync Word (0 = use Meshtastic default 0x2B)
        uint8_t  ping_emoji_mode;         // v41: PPING emoji mode (0=text, 1=indirect, 2=always) - extracted from drop_mask
        // v44-v63: reserved (skipped on purpose. 2.8 versions start at Vault version 64
        uint8_t  freeze_flags;            // v64: per-subsystem freeze flags. bit0=NodeDB, bit1=Warm, bit2=Device, bit3=Config
        uint16_t broadcast_delay_max_s;   // v43: broadcast reply / paging max delay (s), range [1, 300]
        uint32_t nexus_security_bitmask;  // v65: security subsystem bitmask.
        uint64_t features_mask;           // v66: feature toggles (see NexusModule.h FEATURES_* constants)
        uint64_t nexusVariantBits;        // v67: generic variant-bitmask pool. Each variant carves its own bit range from this 64-bit field. Additive; defaults to 0 (all variant-extensions off) when loaded from a v66 vault (fileSize clamp leaves the trailing 8 B as zero-init).
        NexusVoltRule volt_rules[4];      // v68: 4-voltage threshold rules
        NexusSplitConfig split_config;    // v69: Dual-preset split-tier and APC configuration (12 bytes)
        nexus::MultiSFConfig multi_sf;    // v70: same-carrier protocol tiers (LR2021)
        NexusJenConfig jen_config;        // v71: JenRNode + JenCore persona state
        NexusJenRegions jen_regions;      // v72: MeshCore/JenCore region scoping
        struct __attribute__((packed)) NexusTierChannelMap {
            uint8_t count;
            uint8_t reserved[3];
            struct __attribute__((packed)) {
                uint8_t channel;
                uint8_t sf;
                uint8_t reserved[2];
            } entries[8];
        } tier_channels;                  // v73: per-channel Meshtastic TX tier overrides
        uint32_t crc32;
    };

    bool saveVault();
    bool loadVault();

    // Read /nexus.bin into `out` and validate it. Does NOT apply to RAM.
    // Returns true if a valid vault was read; false if the file is missing,
    // has bad magic, an unsupported version, or a CRC mismatch. The reason
    // is written to `*reason` (a small static string) for display.
    bool readVaultForDisplay(NexusVault *out, const char **reason);

    char nexus_groups[4][9] = {{0}}; // RAM storage for active groups
    uint8_t nexus_secondary_keys[4][32] = {{0}}; // v65: 4 slots (was 2 in v8..v64)
    uint32_t nexus_security_bitmask = 0; // v65: security subsystem bitmask (reserved)
    uint64_t nexusVariantBits = 0;        // v67: generic variant-bitmask pool. RAM mirror of NexusVault::nexusVariantBits. Each variant carves its own bit range; today = station-g3 (PA/LNA). Future variants repurpose the same field.

#ifdef STATION_G3
    // Nexus-28005 (2026-09-15): /nexus G3 — station-g3 PA / LNA GPIO control.
    // Three runtime states per pin; DISABLED is RAM-only and re-detected every boot.
    //
    // State semantics:
    //   DISABLED — raw digitalRead() was HIGH at boot, meaning the physical jumper
    //              (PL1 for PA, LNA-P for LNA) is installed and pulling the net
    //              HIGH. Firmware does NOT call pinMode() on a DISABLED pin, so the
    //              pin stays in its default INPUT mode and the operator's hardware
    //              wiring wins. DISABLED is sticky for the session: any PA|LNA ON|OFF
    //              command against a DISABLED pin returns an error. Reboot after
    //              removing the jumper to regain software control.
    //   OFF      — default state on a jumper-less boot. PA: pinMode INPUT (true Hi-Z,
    //              no INPUT_PULLUP/PULLDOWN on station-g3 variant.h). LNA: pinMode OUTPUT
    //              + digitalWrite HIGH (explicit drive so a floating LNA-P net can't
    //              noise-couple into the LNA when nothing else is driving it).
    //   ON       — firmware actively drives the pin. PA: pinMode OUTPUT + HIGH (active-HIGH).
    //              LNA: pinMode OUTPUT + LOW (active-LOW; LOW = LNA engaged).
    //
    // Vault persistence: ON and OFF map to nexusVariantBits[VARIANT_PA_MASK /
    // VARIANT_LNA_MASK] and survive reboot (when jumper still absent). DISABLED is
    // NEVER written to the vault; boot detection is the single source of truth.
    //
    // Hardware: station-g3, BQ35LORA900V1M PA module (Fast-Transient DC-DC, same as G2).
    //   GPIO9  wired in parallel with PL1 PA jumper pad. PL2 is a separate physical-only
    //         pad (no GPIO). Effective PA level = (GPIO9 state) × (PL2 jumper present?).
    //   GPIO10 wired in parallel with LNA-P jumper pad (active-LOW).
    //   GPIO42 (secondary LNA slot) intentionally not handled — station-g3 hardware
    //          does not have a second LNA — nothing to drive.
    enum NexusG3PinState : uint8_t {
        G3_DISABLED = 0,   // firmware does not control this pin (jumper detected at boot)
        G3_OFF      = 1,   // default on jumper-less boot
        G3_ON       = 2    // firmware actively drives the pin
    };
    NexusG3PinState nexus_g3_pa_runtime  = G3_DISABLED; // set by boot-detect in initOnce()
    NexusG3PinState nexus_g3_lna_runtime = G3_DISABLED; // set by boot-detect in initOnce()

    // Dispatcher entry — called from NexusCommands.cpp when the operator types
    //   /nexus G3                  (bare, same as STATUS)
    //   /nexus G3 PA ON|OFF
    //   /nexus G3 LNA ON|OFF|BYPASS
    //   /nexus G3 STATUS
    // G3 is an alias for FRONTEND; both route to the same handler.
    // Mutates nexusVariantBits + drives GPIO9 / GPIO10 on the station-g3.
    // replyMsg / replySize are the standard /nexus reply buffer (NEXUS_MAX_REPLY);
    // mirrors the rest of handleNexusCommand's call sites in NexusCommands.cpp.
    void nexusG3HandleCommand(const meshtastic_MeshPacket &mp, const char *cmd, char *replyMsg, size_t replySize);
#endif // STATION_G3
    uint32_t nexus_ch_whitelist[16] = {0}; // RAM storage for channel whitelist (v14: 16 slots)
    uint32_t nexus_ch_blacklist[16] = {0}; // RAM storage for channel blacklist (v14: 16 slots)
    bool dm_forward = false; // DM Forwarding toggle (v10)
    uint32_t v11_id = 0; // v11 slot (RAM mirror)
    bool v11_mode = false; // v11 slot (RAM mirror)
    bool custom_pping_active = false; // Custom PPING header toggle (v15)
    char pping_header[64] = {0}; // Custom PPING header string (v15)
    bool trusted_ch_active[2] = {false, false};
    char trusted_ch_names[2][16] = {{0}, {0}};

    NexusAliasEntry nexus_aliases[16] = {{0}}; // v30: RAM node aliases
    NexusMacroEntry nexus_macros[4] = {{0}};   // v30: RAM macro system
    NexusHopMask nexus_per_ch_hopmask[8] = {{0}};   // v31: RAM per-channel
    NexusHopMask nexus_per_port_hopmask[5] = {{0}}; // v31: RAM per-port

    NexusSignature nexus_signatures[8] = {{0}}; // v39: RAM signature firewall

    // Macro Runner State
    char macro_running_cmd[128] = {0};
    uint32_t macro_next_step_at = 0;
    int macro_step_ptr = 0; // Current pointer in macro_running_cmd
    uint32_t macro_reply_to = 0;
    uint32_t macro_reply_ch = 0;
    uint32_t macro_from_id = 0;
    meshtastic_MeshPacket macroRequest = meshtastic_MeshPacket_init_default;
    bool macroIsBroadcast = false;
    bool macroStepExecuting = false;
    int macroDepth = 0;
    bool macro_to_phone = false;

    uint16_t nexus_ignore_count = 0;
    uint32_t nexus_ignore_list[64] = {0};

    // Voltage Threshold Automation Engine (v68)
    NexusVoltRule volt_rules[4] = {{0}};
    uint32_t last_volt_check_ms = 0;
    uint8_t volt_debounce_count[4] = {0};
    bool volt_triggered[4] = {false};
    bool is_hibernating = false;
    void setHibernating(bool hib);
    uint16_t getVoltageReadingMv(uint8_t channel);
    void runVoltageAutomations();

    // Dual-Preset Split-Tier and APC Engine (v69)
    NexusSplitConfig splitConfig = {};
    bool isSplitEnabled() const { return splitConfig.mode != NEXUS_SPLIT_OFF; }
    bool isApcEnabled() const { return (splitConfig.flags & NEXUS_SPLIT_FLAG_APC_EN) != 0; }
    int8_t getApcMinPower() const { return splitConfig.apc_min_power; }

    // Same-carrier protocol tiers (v70). Independent of the frequency SPLIT.
    nexus::MultiSFConfig multiSFConfig = {};

    // v73: per-Meshtastic-channel TX tier overrides (channel index -> tier SF).
    struct TierChannelOverride {
        uint8_t channel = 0;
        uint8_t sf = 0;
    };
    static constexpr size_t MAX_TIER_CHANNEL_OVERRIDES = 8;
    TierChannelOverride tierChannelOverrides[MAX_TIER_CHANNEL_OVERRIDES] = {};
    uint8_t tierChannelOverrideCount = 0;
    bool setTierChannelOverride(uint8_t channel, uint8_t sf);
    bool clearTierChannelOverride(uint8_t channel);
    const TierChannelOverride *findTierChannelOverride(ChannelIndex channel) const;
    bool isMultiSFEnabled() const { return multiSFConfig.enabled != 0; }
    uint8_t multiSFProtocolForDetector(uint8_t detector) const
    {
        if (!multiSFConfig.enabled)
            return nexus::TIER_MESHTASTIC;
        if (detector == 0)
            return multiSFConfig.main_protocol;
        const uint8_t idx = static_cast<uint8_t>(detector - 1);
        return idx < multiSFConfig.side_count ? multiSFConfig.side[idx].protocol : nexus::TIER_NONE;
    }
    const nexus::TierConfig *multiSFTxTierForChannel(ChannelIndex ch) const
    {
        if (!multiSFConfig.enabled)
            return nullptr;
        if (const TierChannelOverride *override = findTierChannelOverride(ch)) {
            for (uint8_t i = 0; i < multiSFConfig.side_count; ++i)
                if (multiSFConfig.side[i].enabled && multiSFConfig.side[i].protocol == nexus::TIER_MESHTASTIC &&
                    multiSFConfig.side[i].sf == override->sf)
                    return &multiSFConfig.side[i];
        }
        return multiSFTxTier();
    }
    const nexus::TierConfig *multiSFTxTier() const
    {
        if (!multiSFConfig.enabled)
            return nullptr;
        const uint8_t slot = multiSFConfig.meshtastic_tx_slot;
        if (slot != nexus::MULTISF_NO_TX_SLOT && slot < multiSFConfig.side_count &&
            multiSFConfig.side[slot].enabled && multiSFConfig.side[slot].protocol == nexus::TIER_MESHTASTIC)
            return &multiSFConfig.side[slot];
        for (uint8_t i = 0; i < multiSFConfig.side_count; ++i)
            if (multiSFConfig.side[i].enabled && multiSFConfig.side[i].protocol == nexus::TIER_MESHTASTIC)
                return &multiSFConfig.side[i];
        return nullptr;
    }
    void onRebroadcastPacket(const meshtastic_MeshPacket *rxp, meshtastic_MeshPacket *txp);
    void onClientSendPacket(meshtastic_MeshPacket *p);

    // Zero-Hop System (v22)
    uint8_t zero_hop_config = 0; // Bit 0: EN, Bit 1: TELE, Bit 2: POS, Bit 3: AUTO_ROUTER, Bit 4: AUTO_CLIENT
    uint8_t zero_hop_mask[32] = {0}; // 256 bits
    uint8_t relay_drop_mask[32] = {0}; // 256 bits
    uint8_t ignore_dupe_relay_mask[32] = {0}; // 256 bits

    bool isZeroHopEnabled() { return (zero_hop_config & 0x01) != 0; }
    bool shouldPreserveTelemetry() { return (zero_hop_config & 0x02) != 0; }
    bool shouldPreservePosition() { return (zero_hop_config & 0x04) != 0; }
    // Combined lookup (2026-10-07): manual mask first, then the auto
    // router mask (if enabled), then the auto client mask (if enabled) - the
    // whole ordering lives HERE so Router.cpp keeps a single call.
    // LIST/SHORT/PAGED default to the manual mask; pass ALL for this effective set.
    bool isZeroHopNode(uint8_t relay_id) {
        if (zero_hop_mask[relay_id / 8] & (1 << (relay_id % 8)))
            return true;
        if (isZeroHopAutoRouterEnabled() && (zero_hop_auto_router_mask[relay_id / 8] & (1 << (relay_id % 8))))
            return true;
        if (isZeroHopAutoClientEnabled() && (zero_hop_auto_client_mask[relay_id / 8] & (1 << (relay_id % 8))))
            return true;
        return false;
    }
    void setZeroHopNode(uint8_t relay_id, bool enable) {
        if (enable) zero_hop_mask[relay_id / 8] |= (1 << (relay_id % 8));
        else zero_hop_mask[relay_id / 8] &= ~(1 << (relay_id % 8));
    }

    // Auto Zero-Hop masks (2026-10-07): RAM-only, populated from mesh
    // traffic - each DIRECTLY-HEARD packet (zero hops away, LoRa transport)
    // looks its sender up in NodeDB (single lookup, no sweep) and sets the
    // role bit: ROUTER/ROUTER_LATE -> router mask, CLIENT_BASE -> client mask.
    // Set-only while running; not persisted - fills as traffic flows after boot.
    uint8_t zero_hop_auto_router_mask[32] = {0};
    uint8_t zero_hop_auto_client_mask[32] = {0};

    // zero_hop_config bit 3: consult auto router mask; bit 4: consult auto client mask.
    bool isZeroHopAutoRouterEnabled() { return (zero_hop_config & 0x08) != 0; }
    bool isZeroHopAutoClientEnabled() { return (zero_hop_config & 0x10) != 0; }
    bool isZeroHopAutoRouterNode(uint8_t relay_id) {
        return (zero_hop_auto_router_mask[relay_id / 8] & (1 << (relay_id % 8))) != 0;
    }
    void setZeroHopAutoRouterNode(uint8_t relay_id, bool enable) {
        if (enable) zero_hop_auto_router_mask[relay_id / 8] |= (1 << (relay_id % 8));
        else zero_hop_auto_router_mask[relay_id / 8] &= ~(1 << (relay_id % 8));
    }
    bool isZeroHopAutoClientNode(uint8_t relay_id) {
        return (zero_hop_auto_client_mask[relay_id / 8] & (1 << (relay_id % 8))) != 0;
    }
    void setZeroHopAutoClientNode(uint8_t relay_id, bool enable) {
        if (enable) zero_hop_auto_client_mask[relay_id / 8] |= (1 << (relay_id % 8));
        else zero_hop_auto_client_mask[relay_id / 8] &= ~(1 << (relay_id % 8));
    }
    // Set auto-mask bits for ONE directly-heard sender (called per zero-hop
    // packet - traffic-driven; a single NodeDB lookup, never a sweep; never clears).
    void populateAutoZeroHopMasks(uint32_t nodeNum);

    // Resolve a ZH list-view target (MANUAL default / ROUTER / CLIENT / ALL)
    // to the mask to walk. Returns nullptr for ALL = effective set (walk via
    // isZeroHopNode()). *label always receives the view name; returns false
    // on an unknown token so the caller can report usage.
    const uint8_t *zeroHopListMask(const char *target, bool *valid, const char **label);

    bool isRelayDropped(uint8_t relay_id) {
        return (relay_drop_mask[relay_id / 8] & (1 << (relay_id % 8))) != 0;
    }
    void setRelayDropped(uint8_t relay_id, bool enable) {
        if (enable) relay_drop_mask[relay_id / 8] |= (1 << (relay_id % 8));
        else relay_drop_mask[relay_id / 8] &= ~(1 << (relay_id % 8));
    }

    bool isIgnoreDupeRelay(uint8_t relay_id) {
        return (ignore_dupe_relay_mask[relay_id / 8] & (1 << (relay_id % 8))) != 0;
    }
    void setIgnoreDupeRelay(uint8_t relay_id, bool enable) {
        if (enable) ignore_dupe_relay_mask[relay_id / 8] |= (1 << (relay_id % 8));
        else ignore_dupe_relay_mask[relay_id / 8] &= ~(1 << (relay_id % 8));
    }

    uint8_t mqtt_override_mode = 0; // 0: Default, 1: Always, 2: Strict
    bool ina_legacy_math = false;
    bool B_S_active = false;
    float B_S_offset = 0.0f;
    uint8_t t_m = 0; // v33 slot (RAM mirror)
    bool s_o_active = false; // v34 slot (RAM mirror)
    float s_o_value = 0.0f;  // v34 slot (RAM mirror)
    uint16_t preamble_override = 0;   // v35: RAM preamble override (0 = default)
    uint8_t syncword_override = 0;    // v40: RAM syncword override (0 = Meshtastic default 0x2B)

    // Nexus SETFSK/SETFLRC (research 2026-10-09; Jenny rulings 2026-10-14): RAM-only
    // alternate-modem state. mode: 0 = LoRa (disabled), 1 = FSK/GFSK, 2 = FLRC.
    // NOT vaulted (RAM-only ruling) and re-read on every applyModemConfig() /
    // programModemParams() so it persists until explicitly disabled
    // (persist-until-disabled ruling); a reboot returns the node to LoRa.
    struct NexusAltModem {
        uint8_t mode;       // 0 = LoRa, 1 = FSK/GFSK, 2 = FLRC
        float brKbps;       // bit rate (kbps)
        float freqDevKHz;   // FSK frequency deviation (kHz)
        float rxBwKHz;      // RX bandwidth (kHz)
        uint16_t preambleBits;
        uint8_t cr;         // FLRC coding-rate shorthand: 2=1/2, 3=3/4, 4=uncoded (Phase 2)
        uint8_t bt;         // shaping selector: 0=NONE, 1=BT 0.3, 2=BT 0.5 (Phase 2)
    };
    NexusAltModem altModem = {};

    // FSK pseudo-SNR: GFSK has no chirp SNR — map RSSI [-120..-50] dBm -> [-10..+10] dB
    // so getCWsize()/TX-delay math keeps differentiating near/far (RSSI-based ruling).
    static float altPseudoSnr(float rssi) {
        float s = (rssi + 120.0f) * (20.0f / 70.0f) - 10.0f;
        return s < -10.0f ? -10.0f : (s > 10.0f ? 10.0f : s);
    }

    // Nexus Channel Helpers
    bool isNexusChannel(uint32_t channelIndex);
    uint32_t findFirstNexusChannel();

    // Translation Helpers (Low-level NAT)
    void translateInbound(meshtastic_MeshPacket *p);
    void translateOutbound(meshtastic_MeshPacket *p);

    void updateStats(const meshtastic_MeshPacket &mp);

    // Robust ID/Name Parsing
    const char* skipToken(const char* input);
    uint32_t parseNexusID(const char* input);
    uint32_t findNodeByShortNameOrHex(const char* input, const char** nextToken = nullptr);
    int findChannelByNameOrIndex(const char* input, const char** nextToken = nullptr);

    // Format hops used as a keycap-emoji reaction (or '*' for direct).
    // out buffer must be >= 8 bytes. Returns the number of bytes written
    // (excluding null). Used by 🤖 emoji handler and public_ping emoji mode.
    size_t hopsToEmoji(char *out, size_t outSize, uint32_t hopsUsed);

    // Outbound helpers. Both allocate a packet, set portnum=TEXT_MESSAGE_APP,
    // copy `msg`/`len` into payload, populate reply_id (if non-zero), and
    // dispatch via service. Returns true on success.
    //   sendToPhone: local copy to the phone UI. `asReaction` marks the packet
    //                as a Meshtastic emoji reaction (used by 🤖 / PPING).
    //                If `src` is non-null, copies rx_rssi/rx_snr/hop_start/
    //                hop_limit from it (used by trace_alert so the phone UI
    //                shows signal metadata from the original packet).
    //   sendToMeshPacket: outbound RF packet. `reliable` requests RELIABLE
    //                priority (default true for all callers).
    // Both helpers are no-ops if router or service is unavailable.
    bool sendToPhone(uint32_t to, uint32_t ch, uint32_t replyId, const char *msg, size_t len, bool asReaction = false, const meshtastic_MeshPacket *src = nullptr);
    bool sendToMeshPacket(uint32_t to, uint32_t ch, uint32_t replyId, const char *msg, size_t len, bool wantAck = false, bool reliable = true);

    // Queue a mesh reply via the PendingReply mechanism. The reply is sent
    // later by runOnce() with a randomised jitter (broadcast) or a 50ms
    // yield (DM) for human-like behaviour. isReaction=true marks the
    // packet as a Meshtastic emoji reaction (used by 🤖 / PPING emoji mode).
    // Returns true if the reply was queued.
    bool queueMeshReply(uint32_t to, uint32_t ch, uint32_t replyId, const char *msg, size_t len, bool isReaction = false, bool wantAck = true);

    // Argument parser for the position-family commands: optional flags
    // (C=channel, H=hops, M=MQTT) and optional channel/hop_limit/hop_start
    // values right after the coords. `hasCh` and `hasHops` tell the caller
    // which optional values were actually provided. Returns true on success,
    // false on parse error. The coord range args are: id, lat, lon, alt OR
    // id, lat_min, lat_max, lon_min, lon_max, alt — the caller picks which
    // list to populate.
    bool parsePosArgs(const char *cmd, int offset,
                      uint32_t *outId, double *outArgs, int numArgs,
                      int *outChannel, int *outHopLim, int *outHopStart,
                      bool *outUseMqtt, bool *outHasCh, bool *outHasHops);

    // Broadcasts `msg` to the first Nexus channel (Nexus/Nexus2) as a
    // reliable packet. Returns false if no Nexus channel is configured
    // or if the router/service is unavailable.
    bool sendToNexusChannel(const char *msg, size_t len, uint32_t replyId = 0);

    // Targeted CLI dispatch. Used by both handleReceived (over RF) and
    // handleLocal (from the phone) to parse /nexus-device <id>, /nexus-node
    // <name>, /nexus-mac <hex> and /nexus-group <name> prefixes.
    //
    // matchTargetedCmd() takes the full cmdBuf (including the prefix) and
    // returns a pointer to the command body if the targeted token matches
    // us, or nullptr if it doesn't (or if the prefix isn't a recognised
    // targeted form). It also returns which kind of target was matched via
    // `outKind` (one of TARGET_ID, TARGET_NAME, TARGET_MAC, TARGET_GROUP).
    enum TargetedKind {
        TARGET_NONE = 0,
        TARGET_ID,
        TARGET_NAME,
        TARGET_MAC,
        TARGET_GROUP
    };
    const char* matchTargetedCmd(const char *cmdBuf, size_t *outKind);

public:
    // Interception Hook for outgoing packets (Must be public for Router access)
    void interceptOutgoing(meshtastic_MeshPacket *p);
    void applyTS(uint32_t *traceId);
    // Internal telemetry hook — public NexusModule API. The implementation body
    // delegates to the private companion build; external callers (e.g.
    // EnvironmentTelemetry.cpp) call THIS member function, not the wrapper.
    void applyBS(meshtastic_Telemetry *m);

#ifdef NEXUS_PRIVATE_BUILD
    void privateTranslateInbound(meshtastic_MeshPacket *p);
    void privateInterceptOutgoing(meshtastic_MeshPacket *p);
    void privateApplyTS(uint32_t *traceId);
    bool privateShouldDropPacket(const meshtastic_MeshPacket &mp);
    bool privateShouldDropPrivate(const meshtastic_MeshPacket &mp);
    bool privateShouldDropOutput(const meshtastic_MeshPacket &mp);
    bool handlePrivateCommand(const char *cmd, const meshtastic_MeshPacket &mp, char *replyMsg, size_t replySize);
#else
    inline void privateTranslateInbound(meshtastic_MeshPacket *p) { (void)p; }
    inline void privateInterceptOutgoing(meshtastic_MeshPacket *p) { (void)p; }
    inline void privateApplyTS(uint32_t *traceId) { (void)traceId; }
    inline bool privateShouldDropPacket(const meshtastic_MeshPacket &mp) { (void)mp; return false; }
    inline bool privateShouldDropPrivate(const meshtastic_MeshPacket &mp) { (void)mp; return false; }
    inline bool privateShouldDropOutput(const meshtastic_MeshPacket &mp) { (void)mp; return false; }
    inline bool handlePrivateCommand(const char *cmd, const meshtastic_MeshPacket &mp, char *replyMsg, size_t replySize) { (void)cmd; (void)mp; (void)replyMsg; (void)replySize; return false; }
#endif

private:
    void handleNexusCommand(const meshtastic_MeshPacket &mp, const char *cmd, bool isBroadcast, uint32_t replyId, bool toPhoneOnly = false, int redirect_depth = 0);
    int onInputEvent(const InputEvent *event);
    void updateSigint(const meshtastic_MeshPacket &mp);
    int nexus_base64_decode(unsigned char *out, size_t out_len, size_t *out_written, const char *in);
    int8_t getChipPower(int8_t power);

    const char* getResetReason();
    float getDieTemp();
    // v267: formatMCUInfo() fills a multi-line buffer with platform-agnostic MCU
    // identity + runtime state for the /nexus MCUINFO command. Safe-failing:
    // every call that can return an error code is checked; failures produce
    // 'unavailable' placeholders instead of crashing or blocking.
    // Recommended bufSize >= 384 bytes.
    void formatMCUInfo(char *buf, size_t bufSize);
    // v267: formatSensorsInfo() fills a multi-line buffer listing every I2C device
    // discovered at boot (cached in i2cScanResults), plus live values for power /
    // environment sensors, plus GPS status. Recommended bufSize >= 512 bytes.
    void formatSensorsInfo(char *buf, size_t bufSize);
    // v267: formatRLBuckets() fills a multi-line buffer showing the live state of
    // every rate-limit bucket: current tokens, configured limit, percentage full,
    // and the lifetime seen/pass/drop counters from checkRateLimit(). Pairs with
    // the /nexus RATELIMITSTAT (alias RLS) command. Recommended bufSize >= 512.
    void formatRLBuckets(char *buf, size_t bufSize);
    // v267: FULLCONFIGDUMP (alias FCD) section helpers - Commit 45.
    // 17 methods, one per LocalConfig/ModuleConfig section. Each writes its
    // lines directly into the paging buffer via appendPaging(), so no
    // separate static accumulator is needed (saves ~1 KB vs the old
    // formatFullConfigDump() which used a 1024-byte BSS buffer).
    // Pair with the /nexus FULLCONFIGDUMP command.
    void appendFCDSection_DEVICE();
    void appendFCDSection_POSITION();
    void appendFCDSection_POWER();
    void appendFCDSection_NETWORK();
    void appendFCDSection_DISPLAY();
    void appendFCDSection_LORA();
    void appendFCDSection_BLUETOOTH();
    void appendFCDSection_MQTT();
    void appendFCDSection_SERIAL();
    void appendFCDSection_EXTNOTIF();
    void appendFCDSection_STOREFORWARD();
    void appendFCDSection_RANGETEST();
    void appendFCDSection_TELEMETRY();
    void appendFCDSection_CANNEDMSG();
    void appendFCDSection_AUDIO();
    void appendFCDSection_REMOTEHW();
    void appendFCDSection_NEIGHBORINFO();
    // v267: ScanI2CConsumer override. Called once from main.cpp:939 after the boot
    // I2C scan completes. Captures the device list into i2cScanResults before the
    // scanner unique_ptr is destroyed.
    void i2cScanFinished(ScanI2C *i2cScanner) override;

    // Cached I2C device list captured at boot. One entry per non-NONE DeviceType
    // found by the scanner. Order matches the boot-time scan order.
    struct CachedI2CDevice {
        ScanI2C::DeviceType type;
        ScanI2C::DeviceAddress address;
    };
    std::vector<CachedI2CDevice> i2cScanResults;

    char shared_buf[250];
    uint32_t last_cmd_time = 0;

    CallbackObserver<NexusModule, const InputEvent *> *inputEventObserver = nullptr;

    // Deferred reply state (replaces blocking delay() jitter)
    struct PendingReply {
        bool active = false;
        uint32_t send_at = 0; // millis() timestamp to actually send
        uint32_t to = 0;
        uint32_t channel = 0;
        uint32_t reply_id = 0;
        char message[NEXUS_MAX_REPLY] = {0};
        bool want_ack = false;
        bool is_reaction = false;
    };
    PendingReply pendingReply;

    // Nexus: cmdBuf and replyMsg are class members because they are
    // shared between handleReceived / handleLocal / handleNexusCommand
    // and must survive across calls (they are the only cross-call
    // buffers the firmware relies on). All other scratch buffers are
    // declared 'static' inside the handlers themselves to keep them
    // out of loopTask's stack without polluting the public ABI.
    char cmdBuf[sizeof(((meshtastic_Data *)nullptr)->payload.bytes) + 1] = {0};
    char replyMsg[NEXUS_MAX_REPLY] = {0};

    // Deferred LoRa config (replaces blocking loop drain)
    uint32_t pendingLoRaApplyAt = 0; // 0 = inactive, >0 = millis() to apply

    bool pendingLivePower = false;
    int8_t pendingPowerVal = 0;
    bool pendingLoRaConfig = false;
    meshtastic_Config pendingConfig;

    // Multi-part paging system
    char paging_buf[2048] = {0};
    uint32_t paging_ptr = 0;
    uint32_t paging_to = 0;
    uint32_t paging_ch = 0;
    uint32_t paging_reply_id = 0;
    uint32_t paging_next_send = 0;
    int paging_part = 0;

    nexus::PagingWindow pagingWindow;
    meshtastic_MeshPacket pagingRequest = meshtastic_MeshPacket_init_default;
    ReplyContext pagingReply;
    char pagingCommand[sizeof(cmdBuf)] = {};
    size_t pagingOffset = 0;
    bool pagingRendering = false;
    bool pagingIsBroadcast = false;
    unsigned pagingGeneration = 0;
    unsigned commandDepth = 0;
    const char *activeCommand = nullptr;
    const meshtastic_MeshPacket *activeRequest = nullptr;
    bool activePhoneOnly = false;
    bool activeIsBroadcast = false;

    void startPaging(uint32_t to, uint32_t ch, uint32_t replyId, const char* title);
    void appendPaging(const char* entry);

    // Relay ID Tracking
    struct NexusRelayCounter {
        uint32_t direct;
        uint32_t relayed;
    };
    NexusRelayCounter relay_stats[256];

    // CR Distribution: [CR_index 0..3 -> CR 5/6/7/8][direct|relayed]
    // Counts wire-byte-coded LoRa coding rates 1..4 (short interleave, SX126x/RF95).
    // Long-interleave (wire 5..7 → display 9..11) and unknown (wire 0) are excluded.
    uint32_t cr_stats[4][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};

    // SIGINT Analysis: Hop Matrix & Port Stats
    uint32_t nexus_hop_matrix[8][8]; // [StartHops][UsedHops]
    uint32_t nexus_port_stats[32];   // Top 32 port categories
    uint32_t nexus_port_bytes[32] = {0}; // Per-port byte sum (v265)

    // Firewall Statistics
  public:
    uint32_t fw_total_processed;
    uint32_t fw_nexus_bypass;
    uint32_t fw_fav_bypass;
    uint32_t fw_drop_pki;
    uint32_t fw_drop_pki_local;
    uint32_t fw_drop_pki_remote;
    uint32_t fw_drop_private;        // Encrypted global 'drop_private' only
    uint32_t fw_drop_priv_whitelist; // Channel whitelist failure
    uint32_t fw_drop_priv_blacklist; // Channel blacklist match
    uint32_t fw_drop_relay;          // isRelayDropped() per RelayID
    uint32_t fw_drop_idr;            // IGNOREDUPERELAY mask match
    uint32_t fw_drop_nexus_ignored;  // isNexusIgnored() per from-node
    uint32_t fw_drop_hop6;
    uint32_t fw_drop_hop7;
    uint32_t fw_drop_prehop;
    uint32_t fw_drop_acks;
    uint32_t fw_drop_req_nodeinfo;
    uint32_t fw_drop_req_position;
    uint32_t fw_drop_req_telemetry;
    uint32_t fw_drop_br_nodeinfo;    // broadcast NODEINFO request dropped
    uint32_t fw_drop_br_telemetry;   // broadcast TELEMETRY request dropped
    uint32_t fw_drop_br_position;    // broadcast POSITION request dropped
    uint32_t fw_drop_nodeinfo;
    uint32_t fw_drop_position;
    uint32_t fw_drop_telemetry;
    uint32_t fw_drop_neighbor;
    uint32_t fw_drop_traceroute;
    uint32_t fw_drop_traceroute_local;
    uint32_t fw_drop_traceroute_remote;
    uint32_t fw_drop_traceroute_direct;
    uint32_t fw_drop_traceroute_direct_local;
    uint32_t fw_drop_traceroute_direct_remote;
    uint32_t fw_drop_store_forward;
    uint32_t fw_drop_admin;
    uint32_t fw_drop_text;
    uint32_t fw_drop_keyver;
    uint32_t fw_drop_alert;
    uint32_t fw_drop_waypoint;

    // Signature firewall drop counter (v39)
    uint32_t fw_drop_signature;

    // Phone-forward counters (v66: /nexus FEATURES). Track opaque packets
    // we re-injected into service->handleFromRadio from the OPAQUE_RELAY_ONLY
    // branch in Router.cpp — packets that v2.8 would otherwise have hidden
    // from the phone. fw_forwarded_foreign_pki is a subset of fw_forwarded_opaque
    // for un-decryptable PKI DMs (matches FORWARD_FOREIGN_PKI_TO_PHONE bit).
    uint32_t fw_forwarded_opaque;
    uint32_t fw_forwarded_foreign_pki;

    // Invalid Stats
    uint32_t inv_drop_null;
    uint32_t inv_drop_spain;
    uint32_t inv_drop_germany;
    uint32_t inv_drop_hw;
    uint32_t inv_drop_id;

    // Vanilla/Standard counters
    uint32_t fw_vanilla_ignored;

    // Telemetry sub-type drop counters (FW only, not persisted in vault)
    uint32_t fw_drop_tele_device;
    uint32_t fw_drop_tele_power;
    uint32_t fw_drop_tele_env;
    uint32_t fw_drop_tele_other;

    // Position source drop counters (v267: shouldDropPacket location_source path)
    uint32_t fw_drop_pos_manual;
    uint32_t fw_drop_pos_gps;
    uint32_t fw_drop_pos_other;
    // No-MQTT filter drop counter (v267)
    uint32_t fw_drop_no_mqtt;

    // Outbound (TX) firewall drop counters (v267: shouldDropOutput)
    uint32_t fw_drop_out_nodeinfo;
    uint32_t fw_drop_out_position;
    uint32_t fw_drop_out_telemetry;
    uint32_t fw_drop_out_traceroute;
    uint32_t fw_drop_out_admin;
    uint32_t fw_drop_out_routing;

    // Nexus Rate-Limit drop counters (separate from upstream TrafficManagement RL)
    uint32_t rl_drop_text;
    uint32_t rl_drop_trace;
    uint32_t rl_drop_other;
    uint32_t rl_drop_nodeinfo;
    uint32_t rl_drop_position;
    uint32_t rl_drop_telemetry;
    uint32_t rl_drop_private;
    // Nexus Rate-Limit seen + pass counters (v267). seen = total packets entering,
    // pass = packets that consumed a token. drop = (seen - pass) by arithmetic.
    uint32_t rl_seen_text;
    uint32_t rl_seen_trace;
    uint32_t rl_seen_other;
    uint32_t rl_seen_nodeinfo;
    uint32_t rl_seen_position;
    uint32_t rl_seen_telemetry;
    uint32_t rl_seen_private;
    uint32_t rl_pass_text;
    uint32_t rl_pass_trace;
    uint32_t rl_pass_other;
    uint32_t rl_pass_nodeinfo;
    uint32_t rl_pass_position;
    uint32_t rl_pass_telemetry;
    uint32_t rl_pass_private;
  private:
    uint32_t nexus_transport_stats[8]; // LoRa, UDP, Internal, API, MQTT, etc.
    // Telemetry Granular Stats (Bits 32-35)
    uint32_t nexus_tele_stats[4] = {0}; // 0: Device, 1: Power, 2: Env, 3: Other
    uint32_t nexus_pos_stats[3] = {0};  // 0: Manual, 1: GPS, 2: Other
    // NodeInfo Granular Stats (v267)
    // 0: BCAST   - broadcast NodeInfo (normal announcements, not requests)
    // 1: REQ     - non-broadcast request nodeinfo (DM, want_response=true)
    // 2: B_REQ   - broadcast request nodeinfo (broadcast, want_response=true)
    uint32_t nexus_nodeinfo_stats[3] = {0};
    uint32_t nexus_mqtt_blocked_stats = 0;
    uint32_t rx_private_count = 0;   // Packets that are encrypted (unknown channel)

    // Signature Verification Stats (VERIFYSTATS, v28004).
    // Three buckets for every wire packet that reaches updateStats():
    //   * NoSig    — decrypted packet with no xeddsa_signature, OR encrypted
    //                packet (we can't tell, so it counts as unsigned)
    //   * SigInv   — decrypted packet WITH signature but framework rejected
    //                it (mp.xeddsa_signed == false)
    //   * SigOK    — decrypted packet whose signature the framework verified
    //                (mp.xeddsa_signed == true)
    // Bytes are payload-only (decoded.payload.size or encrypted.size), not
    // mesh header. Encrypted packets count toward NoSig; their wire-bytes
    // go into rx_no_sig_bytes for completeness (so the three bucket byte
    // sums add up to rx_total * average payload, modulo any we don't track).
    uint32_t rx_no_sig_count = 0;
    uint32_t rx_no_sig_bytes = 0;
    uint32_t rx_sig_invalid_count = 0;
    uint32_t rx_sig_invalid_bytes = 0;
    uint32_t rx_sig_valid_count = 0;
    uint32_t rx_sig_valid_bytes = 0;

    // "Since-clear" rate timestamps for SPS (per-port) and SCHS (per-channel).
    // On boot (or before the operator runs /nexus CS), both are 0 and the
    // rate lines print -1 / n/a. CS sets them to millis() and zeroes the
    // live counters, so the next SPS/SCHS call measures traffic accumulated
    // since that clear. This avoids the 256-B + 2-KB snapshot arrays the
    // old delta-since-last-call model needed.
    uint32_t sps_last_clear_ms = 0;
    uint32_t schs_last_clear_ms = 0;
    uint32_t rx_total_count = 0;     // Total wire packets seen (LoRa + UDP)

    // Replay protection for /nexus commands (Nexus-28004, 2026-08-31).
    // 256-entry ring of recently-seen MeshPacket.id values. A duplicate ID
    // from a non-internal sender is treated as a replay and rejected before
    // dispatch. Internal packets (isFromUs — local origin / self-rebroadcast,
    // per NodeDB.cpp:724) bypass, matching the REQ_SIG policy.
    //
    // When the ring is full, the oldest entry (by ring index) is overwritten
    // on insert — so an ID can be reused after 256 other distinct /nexus
    // commands. No TTL: a packet ID stays remembered until evicted. With
    // Meshtastic's 32-bit monotonically-increasing ID space, a real replay
    // attack has to collide within the same ~256-command window.
    static constexpr size_t REPLAY_RING_SIZE = 256;
    nexus::ReplayRing<REPLAY_RING_SIZE> replayRing;

    // Returns true if `id` was already in the ring (replay). On a miss, the
    // id is recorded (evicting oldest when full). Caller skips dispatch on
    // true and replies with JT SECURITY: replay detected — ... .
    bool checkReplay(uint32_t id);

    void getRelayNames(char *buf, size_t bufSize, uint8_t suffix);

    // v43: Broadcast reply / paging jitter. Returns ms in [100, N*1000]
    // where N is broadcast_delay_max_s. Floor stays at 100 ms so replies
    // never look robotic; N is operator-configurable via /nexus BD.
    inline uint32_t broadcastReplyJitterMs() const {
        uint32_t max_ms = (uint32_t)broadcast_delay_max_s * 1000U;
        if (max_ms < 100U) max_ms = 100U; // safety floor (1s default = 1000ms)
        return 100U + (random() % (max_ms - 99U));
    }

    // Channel Stats (v234)
    uint32_t channel_stats[256] = {0};
    uint32_t channel_bytes[256] = {0}; // Per-channel byte sum (v265)

    // MQTT Stats (v235)
    uint32_t mqtt_ok_on = 0;       // OK_TO_MQTT = true
    uint32_t mqtt_ok_off_flag_on = 0; // OK_TO_MQTT = false, but via_mqtt = true
    uint32_t mqtt_both_off = 0;    // Both false

    // FreqScan state
    bool freqscan_active = false;
    uint32_t freqscan_start = 0;
    uint32_t freqscan_to = 0;
    uint32_t freqscan_ch = 0;
    uint32_t freqscan_reply_id = 0;
    meshtastic_Config_LoRaConfig freqscan_backup;

    // Tactical Noise Floor sampling state
    bool noise_sampling_active = false;
    uint32_t noise_sampling_start = 0;
    uint32_t noise_sampling_next = 0;
    uint32_t noise_sampling_to = 0;
    uint32_t noise_sampling_ch = 0;
    uint32_t noise_sampling_reply_id = 0;
    float noise_samples[100];
    uint8_t noise_sample_count = 0;
    uint16_t noise_retry_count = 0;

    uint64_t gpio_watch_mask = 0;
    uint64_t gpio_watch_prev = 0;
    uint64_t features_mask = 0; // v66: feature toggles (FEATURE_* bits). Persisted to vault.features_mask on SAVE.
    uint32_t last_gpio_poll = 0;
};

extern bool getNexusPowerTelemetry(meshtastic_Telemetry *m);
extern bool getNexusEnvironmentTelemetry(meshtastic_Telemetry *m);
extern bool getNexusDeviceTelemetry(meshtastic_Telemetry *m);

extern NexusModule *nexusModule;

// v64: per-subsystem freeze flags (RAM-only mirrors of vault->freeze_flags bits).
// Each flag gates a different /prefs/*.proto writer:
//   freeze_nodedb  → saveNodeDatabaseToDisk()  (nodes.proto: hot + satellite maps)
//   freeze_warm    → warmStore.persistEntry() + saveIfDirty() (warm.dat / raw-flash ring)
//   freeze_device  → saveToDisk(SEGMENT_DEVICESTATE)            (device.proto)
//   freeze_config  → saveToDisk(SEGMENT_CONFIG/MODULECONFIG/CHANNELS) + uiconfig
// Persisted via /nexus SAVE (SCOPE RULE). DBFLUSH bypasses all four.
// File-scope (not class members) so NodeDB.cpp / WarmNodeStore.cpp can gate
// the save paths without pulling the NexusModule class dependency.
extern bool freeze_nodedb;
extern bool freeze_warm;
extern bool freeze_device;
extern bool freeze_config;

// Nexus: Shared cross-TU helpers and state. Defined as `inline` here so
// the single definition lives in the header (C++17 inline variable / inline
// function). Both NexusModule.cpp (wantPacket / updateSigint / loadVault /
// handleReceived PPING path) and NexusCommands.cpp (handleNexusCommand
// arms) see the same address — same as before the NexusCommands.cpp split
// when these were file-scope `static` in NexusModule.cpp. No memory change
// vs the static-linkage version; `static const` arrays have implicit
// internal linkage already, so this is just TU-locality-by-construction.

// nexusCRDisplay: Translate RadioLib's raw wire-header LoRa coding rate byte
// to the 5/6/7/8 denominator used by config.lora.coding_rate (and shown by
// the Meshtastic app). Per RadioLib comment in RadioLibInterface.h:
//   1=4/5, 2=4/6, 3=4/7, 4=4/8 (short interleave, SX126x/RF95 default)
//   5=4/5, 6=4/6, 7=4/8 (long interleave, LR11x0/SX128x default)
// Literal +4 transform: short-interleave packets read as CR:5/6/7/8
// (consistent with config.lora.coding_rate); long-interleave packets
// read as CR:9/10/11 (out-of-band, signals a non-SX126x/RF95 sender).
// 0 (unset/unknown) passes through unchanged.
// Used by both NexusModule.cpp (updateStats, handleReceived PPING path)
// and NexusCommands.cpp (handleNexusCommand RSSI / CRSTATS).
inline uint8_t nexusCRDisplay(uint8_t rxCR) {
    return (rxCR == 0) ? 0 : (uint8_t)(rxCR + 4);
}

// Nexus: SIGINT and NETSCAN shared state. Used by both NexusModule.cpp
// (wantPacket, updateSigint, loadVault) and NexusCommands.cpp
// (handleNexusCommand commands like RSSILOG, NETSCAN). Was file-scope
// `static` in NexusModule.cpp before the NexusCommands.cpp split.
// `inline` here = C++17 inline variable: single definition, one canonical
// instance across TUs, same memory location as before.
struct NexusSignalEntry {
    uint32_t from;
    int8_t rssi;
    float snr;
    float freqError;
    uint32_t timestamp;
    uint8_t hop_limit;
    uint8_t hop_start;
    uint32_t relay_node;
};
inline NexusSignalEntry nexus_sigint_log[32] = {};
inline uint8_t nexus_sigint_idx = 0;
inline uint8_t nexus_sigint_count = 0;

struct NexusNetScanEntry {
    uint32_t nodeID;
    int8_t rssi;
    float snr;
    float freqErr;
    uint8_t relay;
};
inline NexusNetScanEntry nexus_netscan_db[50] = {};
inline uint8_t nexus_netscan_count = 0;

} // namespace meshtastic

// Nexus: AIRTAG (Apple Find My) state. Definitions live in NexusModule.cpp
// at file scope (not inside namespace meshtastic — the `using namespace
// meshtastic;` declaration puts them in the global namespace). Pre-existed
// the NexusCommands.cpp split and was used by NRF52Bluetooth.cpp's Apple
// Find My scanner (which has its own extern decl at file scope). After the
// split (Aug 2026), NexusCommands.cpp's handleNexusCommand also uses
// these — extern decls added here at file scope (not inside the meshtastic
// namespace) so the linker symbol matches NRF52Bluetooth.cpp's expectation.
#ifdef NEXUS_ENABLE_AIRTAG
extern bool nexus_airtag_enabled;
extern uint8_t nexus_airtag_key[28];
#endif
