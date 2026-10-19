// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) 2026 Jennifer Sanchez
//
// Licensed under the GNU Affero General Public License v3.
// See LICENSE-AGPL for the full license text.
//
// NexusModule — remote CLI module for Meshtastic firmware
//
// PROJECT-WIDE PRINTF NOTE (ESP32-S3, 2026-07-14):
// The Meshtastic framework builds against the ROM newlib-nano printf
// (see esp32s3.rom.newlib-nano.ld), which does NOT implement the 'll'
// length modifier. Specs like %llX / %llu / %lld are silently parsed
// as 4-byte %lX/%lu/%ld, leak the second 'l' plus the conversion
// character as literal text ('lX', 'llu', ...) into the output, AND
// leave the upper 4 bytes of the 8-byte argument unconsumed, so the
// next va_args slot reads garbage instead of the intended value.
//
// Workaround: never use %ll*. Split any 64-bit print into a high/low
// 32-bit pair with %08lX + %08lX (zero-padded hex) or %lu + %lu
// (decimal). Patterns already used by HEXDROPS / HEXOUTDROPS / VAULT
// DIFF / GPIO / etc. in this file. If full newlib is ever linked
// instead of nano, these can collapse back to %llX in one search-and-
// replace (look for "\%0[0-9]*l" in this file as the marker).

#include "UptimeClock.h"
#include "mesh/Throttle.h"
#include "modules/NexusModule.h"
// Decaf-split (Nexus-28006): NexusModule_private.h is only present
// in private builds. Gate the include so public builds compile cleanly
// when the header is absent (the public tree strips it via
// bin/nexus_strip_tactical_to_public.py). The matching call site at the
// top of handleNexusCommand is also gated, so the include + call are
// consistent: both exist on private builds, neither exists on public builds.
#ifdef NEXUS_PRIVATE_BUILD
#include "modules/NexusModule_private.h"
#endif
#include "Channels.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "PowerFSM.h"
#include "RTC.h"
#include "SPILock.h"
#include "input/InputBroker.h"
#include "meshUtils.h"
#include <FSCommon.h>
#include <ctype.h>
#include <climits>  // Nexus-28005: FEATURES hex parser uses ULLONG_MAX for strtoull range check
#if defined(ARCH_ESP32) && !MESHTASTIC_EXCLUDE_WIFI
#include "MeshtasticOTA.h"
#endif
#include "Router.h"
#include "airtime.h"
#include "configuration.h"
#include "UptimeClock.h"   // Time::getMillisMonotonic() — wrap-safe 64-bit uptime
#include "main.h"
#ifdef ARCH_NRF52
#include "main.h"
#endif
#ifdef ARCH_PORTDUINO
#include "unistd.h"
#endif

#include "Default.h"
#include "MeshRadio.h"
#include "TypeConversions.h"
#include "freertosinc.h"  // For uxTaskGetStackHighWaterMark / uxTaskGetSystemState (RAM command)

#if !MESHTASTIC_EXCLUDE_MQTT
#include "mqtt/MQTT.h"
#endif

#if !MESHTASTIC_EXCLUDE_GPS
#include "GPS.h"
#endif

#if MESHTASTIC_EXCLUDE_GPS
#include "modules/PositionModule.h"
#endif

// Nexus-28002 (2026-08-21): /nexus HOPSCALE reads live state from the
// HopScalingModule. The module is gated on HAS_VARIABLE_HOPS — mirror that
// gate so the include only fires when the module is actually compiled in.
#if HAS_VARIABLE_HOPS
#include "modules/HopScalingModule.h"
#endif

#if !defined(ARCH_STM32WL) && !MESHTASTIC_EXCLUDE_I2C
#include "motion/AccelerometerThread.h"
#endif
#include <pb_encode.h>
#include <pb_decode.h>
#include "mesh/generated/meshtastic/admin.pb.h"
#include "modules/TraceRouteModule.h"
#include "modules/ExternalNotificationModule.h"
#include "RadioLibInterface.h"
#if (defined(USE_LR2021) || defined(ARCH_PORTDUINO)) && !RADIOLIB_EXCLUDE_LR2021
// Nexus-28005: /nexus LR2021_SF handler uses LR2021Interface accessors
// (sideDetectorsCount, setSideDetectors, clearSideDetectors) for the
// multi-SF side-detector table. Guard mirrors src/mesh/LR2021Interface.cpp:3
// so the driver and the call sites pre-process together: SX126x-only builds
// (no USE_LR2021, no ARCH_PORTDUINO) drop both, avoiding undefined refs to
// LR2021Interface::setSideDetectors / configureSideDetectors at link time
// (the station-g3 Nexus-28005 link error).
#include "LR2021Interface.h"
#endif

#if (defined(ARCH_ESP32) || defined(ARCH_NRF52) || defined(ARCH_RP2040)) && !defined(CONFIG_IDF_TARGET_ESP32S2) &&               \
    !defined(CONFIG_IDF_TARGET_ESP32C3)
#include "SerialModule.h"
#endif

#include "modules/AdminModule.h"
#include "modules/TrafficManagementModule.h"
#include "modules/Telemetry/PowerTelemetry.h"
#include "modules/Telemetry/EnvironmentTelemetry.h"
#include "modules/Telemetry/DeviceTelemetry.h"
#include "gps/RTC.h"
#include "memGet.h"
#include "FSCommon.h"
#include "ErriezCRC32.h"

#ifdef ARCH_ESP32
#include "esp_system.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_heap_caps.h"
#include "esp_random.h"
#include "esp_efuse.h"
#include <WiFi.h>
#include "nimble/NimbleBluetooth.h"
extern NimbleBluetooth *nimbleBluetooth;
#if HAS_WIFI
#include "mesh/api/WiFiServerAPI.h"
extern bool hasWiFiClient();
extern String getWiFiClientInfo();
#endif
#endif

#ifdef ARCH_NRF52
#include <nrf.h>
#include <nrf_soc.h>
#include <Adafruit_LittleFS.h>
#include "NRF52Bluetooth.h"
extern NRF52Bluetooth *nrf52Bluetooth;
#endif

using namespace meshtastic;

// === END INCLUDES (from NexusModule.cpp:1-114) ===

// External helper needed by handleNexusCommand. Defined in main.cpp /
// NRF52 port (platform-specific; we just forward-declare here).
extern void getMacAddr(uint8_t *dmac);

// === BEGIN FILE-SCOPE HELPERS (TU-local, used only by handleNexusCommand) ===
// These were `static` in NexusModule.cpp before the NexusCommands.cpp split
// (Aug 2026). After the split they no longer needed to be visible across
// TUs (only handleNexusCommand uses them), so they're back to TU-local
// `static` here. The compiler is free to inline them aggressively within
// this TU, restoring the original per-command inlining budget.
// NOTE: nexusCRDisplay is NOT here — it lives in NexusModule.h as `inline`
// because it's used by both NexusModule.cpp (updateStats, handleReceived
// PPING path) and NexusCommands.cpp (handleNexusCommand RSSI / CRSTATS).
// Same for nexus_sigint_* / nexus_netscan_* — inline in NexusModule.h.

// === BEGIN SLIM NodeInfoLite COMPAT SHIM (Nexus-28001 Phase 2) ===
// Upstream 2.8.0 replaced the fat `meshtastic_NodeInfoLite` (with embedded user / position /
// device_metrics / is_favorite / is_ignored / has_user / has_position / has_device_metrics)
// with a slim struct that keeps only minimal fields. The full data lives in separate per-type
// `nodePositions` / `nodeTelemetry` / `nodeEnvironment` maps on NodeDB; the favorite/ignored/mqtt/
// has_user/has_is_unmessagable/etc. booleans are packed into a single `bitfield` field accessed
// via inline helpers. These shims give Nexus's existing command code (`n->short_name`,
// `nexus_is_favorite(n)`, `n->position.X`, ...) a single TU-local translation layer so the body
// of handleNexusCommand stays readable.
//
// Uses TypeConversions::ConvertToUser() (the upstream helper that fills a fresh meshtastic_User
// from a slim NodeInfoLite, including is_licensed via nodeInfoLiteIsLicensed()) for the rare
// spots that need the full User struct (e.g., is_licensed). Short name / long name / role /
// hw_model / public_key are direct field accesses on the slim struct now.
//
// Sized to 25 chars (slim) vs User's 40: Nexus only reads these, never writes. Callers that
// write to long_name through this shim are not present in NexusCommands; the upstream
// setters (handleSetOwner etc.) write through `owner` not through a NodeInfoLite.

// Nexus-28006 (2026-09-17) — REDIRECT recursion ceiling.
//
// Stack-overflow / crash prevention for the REDIRECT (R / REDIRECT) command
// chain in handleNexusCommand. Bug report 2026-09-17:
// "/nexus R CH <x> <cmd>" crashed the node — root cause was
// unbounded recursion via redirect_depth + 1, which would explode the
// stack on a chain like "/nexus R CH a R CH b R CH c ..." or on any
// operator-typo that triggered a sub-command which itself was a
// REDIRECT. No upper bound existed before — every recursive
// handleNexusCommand call grew the stack by the function's frame
// size, which on nRF52's 4 KB loopTask and ESP32's 8 KB loopTask
// is enough for ~50 levels before the stack pointer collides with
// BSS / heap and the node crashes.
//
// Cap at 3 — allows a sensible chain like
//   /nexus R CH a R DM b <leaf-command>
//   /nexus R DM a R CH b <leaf-command>
//   /nexus R CH a R CH b <leaf-command>
//   /nexus R DM a R DM b <leaf-command>
// before refusing, while still keeping the call stack at <1 KB on
// nRF52 and well within ESP32 budgets for any normal operator flow.
// Anything deeper is either operator error (recursion loop) or a
// malicious packet trying to DoS the node via stack exhaustion.
#define NEXUS_MAX_REDIRECT_DEPTH 3

// Booleans that used to be direct fields on fat NodeInfoLite:
static inline bool nexus_is_favorite(const meshtastic_NodeInfoLite *n) { return nodeInfoLiteIsFavorite(n); }
static inline bool nexus_is_ignored(const meshtastic_NodeInfoLite *n) { return nodeInfoLiteIsIgnored(n); }
static inline bool nexus_via_mqtt(const meshtastic_NodeInfoLite *n) { return nodeInfoLiteViaMqtt(n); }
static inline bool nexus_has_user(const meshtastic_NodeInfoLite *n) { return nodeInfoLiteHasUser(n); }
// Maps don't have a `has_*` field; presence is `.count() != 0`. Public maps via friend class
// declared in NodeDB.h (see Nexus-28001 commit 28f999c98).
static inline bool nexus_has_position(const meshtastic_NodeInfoLite *n) {
#if !MESHTASTIC_EXCLUDE_POSITIONDB
    return nodeDB->nodePositions.count(n->num) > 0;
#else
    return false;
#endif
}
static inline bool nexus_has_device_metrics(const meshtastic_NodeInfoLite *n) {
#if !MESHTASTIC_EXCLUDE_TELEMETRYDB
    return nodeDB->nodeTelemetry.count(n->num) > 0;
#else
    return false;
#endif
}
// Read-only views of the full Position / DeviceMetrics record. Returns nullptr if absent;
// callers should check nexus_has_* first or guard with `if (auto *p = nexus_pos(n)) { ... }`.
static inline const meshtastic_PositionLite *nexus_pos(const meshtastic_NodeInfoLite *n) {
#if !MESHTASTIC_EXCLUDE_POSITIONDB
    auto it = nodeDB->nodePositions.find(n->num);
    return it != nodeDB->nodePositions.end() ? &it->second : nullptr;
#else
    return nullptr;
#endif
}
static inline const meshtastic_DeviceMetrics *nexus_devmetrics(const meshtastic_NodeInfoLite *n) {
#if !MESHTASTIC_EXCLUDE_TELEMETRYDB
    auto it = nodeDB->nodeTelemetry.find(n->num);
    return it != nodeDB->nodeTelemetry.end() ? &it->second : nullptr;
#else
    return nullptr;
#endif
}
// is_licensed used to be `nexus_is_licensed(n)` (a User struct field). Upstream packed it into
// the bitfield; nodeInfoLiteIsLicensed() pulls it back out. The full User struct is needed
// in only a couple of spots (where Nexus reads is_licensed) — those use the inline shim below.
static inline bool nexus_is_licensed(const meshtastic_NodeInfoLite *n) { return nodeInfoLiteIsLicensed(n); }
// Write-side: the shims above return bool (not lvalue), so we need explicit setters for
// the fav/ignored flags. Uses upstream's nodeInfoLiteSetBit (NodeDB.h:875) under the hood.
static inline void nexus_set_favorite(meshtastic_NodeInfoLite *n, bool v) { nodeInfoLiteSetBit(n, NODEINFO_BITFIELD_IS_FAVORITE_MASK, v); }
static inline void nexus_set_ignored(meshtastic_NodeInfoLite *n, bool v) { nodeInfoLiteSetBit(n, NODEINFO_BITFIELD_IS_IGNORED_MASK, v); }
// Helper that fills a local meshtastic_User on demand (rarely needed — most call sites can use
// `n->short_name` / `n->role` / etc. directly). Kept here so we have one conversion point.
static inline meshtastic_User nexus_user_of(const meshtastic_NodeInfoLite *n) {
    return TypeConversions::ConvertToUser(n);
}
// === END SLIM NodeInfoLite COMPAT SHIM ===

static const char* const roleNames[] = {
    "CLIENT", "MUTE", "ROUTER", "ROUTER_CLIENT", "REPEATER", "TRACKER",
    "SENSOR", "TAK", "HIDDEN", "LOST", "TAK_TRACKER", "ROUTER_LATE", "BASE"
};
// Bound derived from the array, not a hardcoded literal. If a new role string is added
// here, the bound below tracks it automatically — no OOB drift on future enum additions.
// (OOB-SAFE HELPER RULE.)
static const size_t ROLE_NAMES_COUNT = sizeof(roleNames) / sizeof(roleNames[0]);
static_assert(ROLE_NAMES_COUNT > 0, "roleNames[] must not be empty");
static_assert(ROLE_NAMES_COUNT <= 32, "roleNames[] drift — Nexus role indices must fit in int8_t (M_S)");

int parseRole(const char *roleStr) {
    if (strcasecmp(roleStr, "CLIENT") == 0) return meshtastic_Config_DeviceConfig_Role_CLIENT;
    if (strcasecmp(roleStr, "SENSOR") == 0) return meshtastic_Config_DeviceConfig_Role_SENSOR;
    if (strcasecmp(roleStr, "TRACKER") == 0) return meshtastic_Config_DeviceConfig_Role_TRACKER;
    if (strcasecmp(roleStr, "MUTE") == 0) return meshtastic_Config_DeviceConfig_Role_CLIENT_MUTE;
    if (strcasecmp(roleStr, "TAK") == 0) return meshtastic_Config_DeviceConfig_Role_TAK;
    if (strcasecmp(roleStr, "ROUTER") == 0) return meshtastic_Config_DeviceConfig_Role_ROUTER;
    if (strcasecmp(roleStr, "ROUTER_CLIENT") == 0 || strcasecmp(roleStr, "RTR_CLIENT") == 0) return meshtastic_Config_DeviceConfig_Role_ROUTER_CLIENT;
    if (strcasecmp(roleStr, "REPEATER") == 0) return meshtastic_Config_DeviceConfig_Role_REPEATER;
    if (strcasecmp(roleStr, "HIDDEN") == 0) return meshtastic_Config_DeviceConfig_Role_CLIENT_HIDDEN;
    if (strcasecmp(roleStr, "LOST") == 0) return meshtastic_Config_DeviceConfig_Role_LOST_AND_FOUND;
    if (strcasecmp(roleStr, "TAK_TRACKER") == 0) return meshtastic_Config_DeviceConfig_Role_TAK_TRACKER;
    if (strcasecmp(roleStr, "ROUTER_LATE") == 0) return meshtastic_Config_DeviceConfig_Role_ROUTER_LATE;
    if (strcasecmp(roleStr, "BASE") == 0 || strcasecmp(roleStr, "CLIENT_BASE") == 0) return meshtastic_Config_DeviceConfig_Role_CLIENT_BASE;
    if (isdigit(roleStr[0])) return atoi(roleStr);
    return -1;
}

const char *getRoleName(int role) {
    if (role >= 0 && (size_t)role < ROLE_NAMES_COUNT) return roleNames[role];
    return "UNKNOWN";
}

static const char* const rebroadcastNames[] = {
    "ALL", "ALL_SKIP", "LOCAL_ONLY", "KNOWN_ONLY", "NONE", "CORE_ONLY"
};
// Bound derived from the array, not a hardcoded literal. (OOB-SAFE HELPER RULE.)
static const size_t REBROADCAST_NAMES_COUNT = sizeof(rebroadcastNames) / sizeof(rebroadcastNames[0]);
static_assert(REBROADCAST_NAMES_COUNT > 0, "rebroadcastNames[] must not be empty");

static const char* getRebroadcastModeName(int mode) {
    if (mode >= 0 && (size_t)mode < REBROADCAST_NAMES_COUNT) return rebroadcastNames[mode];
    return "UNKNOWN";
}

// Translate RadioLib's raw wire-header LoRa coding rate byte to the
// 5/6/7/8 denominator used by config.lora.coding_rate (and shown by
// the Meshtastic app). Per RadioLib comment in RadioLibInterface.h:
//   1=4/5, 2=4/6, 3=4/7, 4=4/8 (short interleave, SX126x/RF95 default)
//   5=4/5, 6=4/6, 7=4/8 (long interleave, LR11x0/SX128x default)
// Literal +4 transform: short-interleave packets read as CR:5/6/7/8
// (consistent with config.lora.coding_rate); long-interleave packets
// read as CR:9/10/11 (out-of-band, signals a non-SX126x/RF95 sender).
// 0 (unset/unknown) passes through unchanged.
// Used by both NexusModule.cpp (updateStats, handleReceived PPING path)
// and NexusCommands.cpp (handleNexusCommand RSSI / CRSTATS). Lives in
// NexusModule.h as `inline`.

// Internal tactical helper: case-insensitive substring search used by
// /nexus SEARCH / ALIAS resolution.
static const char* nexus_strcasestr(const char* haystack, const char* needle) {
    if (!haystack || !needle) return NULL;
    if (!*needle) return haystack;
    for (; *haystack; ++haystack) {
        if (tolower((unsigned char)*haystack) == tolower((unsigned char)*needle)) {
            const char *h, *n;
            for (h = haystack, n = needle; *h && *n; ++h, ++n) {
                if (tolower((unsigned char)*h) != tolower((unsigned char)*n)) break;
            }
            if (!*n) return haystack;
        }
    }
    return NULL;
}

// GNSS model name lookup (used by /nexus GPS command).
// Pulls in gps/GPS.h enum values — kept here as TU-local static rather
// than moving to the header, to avoid dragging GPS.h into every
// translation unit that includes NexusModule.h.
static const char* getGnssModelName(int model) {
    switch(model) {
        case GNSS_MODEL_ATGM336H: return "ATGM336H";
        case GNSS_MODEL_MTK: return "MTK";
        case GNSS_MODEL_UBLOX6: return "UBLOX6";
        case GNSS_MODEL_UBLOX7: return "UBLOX7";
        case GNSS_MODEL_UBLOX8: return "UBLOX8";
        case GNSS_MODEL_UBLOX9: return "UBLOX9";
        case GNSS_MODEL_UBLOX10: return "UBLOX10";
        case GNSS_MODEL_UC6580: return "UC6580";
        case GNSS_MODEL_MTK_L76B: return "L76B";
        case GNSS_MODEL_MTK_PA1010D: return "PA1010D";
        case GNSS_MODEL_MTK_PA1616S: return "PA1616S";
        case GNSS_MODEL_AG3335: return "AG3335";
        case GNSS_MODEL_AG3352: return "AG3352";
        case GNSS_MODEL_LS20031: return "LS20031";
        case GNSS_MODEL_CM121: return "CM121";
        default: return "GENERIC";
    }
}

// Base64 encoder for /nexus export paths (masks, keys, streams).
// Uses a local `table` for url_safe vs standard alphabet — does NOT depend
// on a file-scope b64_table.
static void nexus_base64_encode(char *out, const uint8_t *in, size_t in_len, bool url_safe = false) {
    const char *table = url_safe ? "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_" :
                                   "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    size_t j = 0;
    for (size_t i = 0; i < in_len; i += 3) {
        uint32_t val = (uint32_t)in[i] << 16;
        if (i + 1 < in_len) val |= (uint32_t)in[i + 1] << 8;
        if (i + 2 < in_len) val |= (uint32_t)in[i + 2];

        out[j++] = table[(val >> 18) & 0x3F];
        out[j++] = table[(val >> 12) & 0x3F];
        if (i + 1 < in_len) out[j++] = table[(val >> 6) & 0x3F];
        else if (!url_safe) out[j++] = '=';

        if (i + 2 < in_len) out[j++] = table[val & 0x3F];
        else if (!url_safe) out[j++] = '=';
    }
    out[j] = '\0';
}

// TraceRoute callback: receives a TRACEROUTE result and turns it into a
// mesh reply. Registered via traceRouteModule->setNexusTraceCallback().
// Uses 200-byte stack buffer (NEXUS_MAX_REPLY). Kept here (was
// NexusModule.cpp pre-split) because only handleNexusCommand registers it.
static void onNexusTraceResult(const String &result, int8_t rssi, float freqErr, uint32_t channel) {
    if (!nexusModule) return;
    nexusModule->trace_active = false; // Release the lock
    if (nexusModule->trace_reply_to == 0) return;

    float elapsed = (Time::getMillis() - nexusModule->trace_start_time) / 1000.0f;

    char replyMsg[NEXUS_MAX_REPLY]; // Stack reduction for nRF
    int l = snprintf(replyMsg, sizeof(replyMsg), "JT TRACE (%.2fs):\n%s\n", elapsed, result.c_str());
    if (l < 0 || (size_t)l >= sizeof(replyMsg)) l = sizeof(replyMsg) - 1;
    snprintf(replyMsg + l, sizeof(replyMsg) - l, "[Last: %ddBm, %.2fHz]", rssi, freqErr);

    nexusModule->sendReply(nexusModule->traceReply, replyMsg, strlen(replyMsg));
    nexusModule->trace_reply_to = 0; // Clear
}

// Nexus: Encode the given text into a meshtastic_StatusMessage and send it as
// a node-status broadcast (portnum NODE_STATUS_APP). Mirrors the upstream
// StatusMessageModule::runOnce logic at src/modules/StatusMessageModule.cpp:24
// but takes the text inline instead of pulling from moduleConfig. status[80]
// from the StatusMessage protobuf caps text at 79 chars + null; longer input
// is truncated silently. Caller is responsible for formatting the text.
//
// Priority BACKGROUND so it doesn't preempt normal node chatter. Channel
// defaults to 0 (StatusMessageModule always sends on Ch 0).
static void sendNexusStatusMessage(const char *text, uint8_t channel = 0)
{
    if (!router || !service || !text || *text == '\0') return;
    meshtastic_MeshPacket *p = router->allocForSending();
    if (!p) return;
    meshtastic_StatusMessage sm = meshtastic_StatusMessage_init_zero;
    strncpy(sm.status, text, sizeof(sm.status));
    sm.status[sizeof(sm.status) - 1] = '\0'; // ensure null termination
    p->decoded.payload.size = pb_encode_to_bytes(
        p->decoded.payload.bytes, sizeof(p->decoded.payload.bytes),
        meshtastic_StatusMessage_fields, &sm);
    p->decoded.portnum = meshtastic_PortNum_NODE_STATUS_APP;
    p->to = NODENUM_BROADCAST;
    p->decoded.want_response = false;
    p->priority = meshtastic_MeshPacket_Priority_BACKGROUND;
    p->channel = channel;
    service->sendToMesh(p);
}

// === END FILE-SCOPE HELPERS ===

// === BEGIN handleNexusCommand (from NexusModule.cpp:1651-8873) ===

void NexusModule::handleNexusCommand(const meshtastic_MeshPacket &mp, const char *cmd, bool isBroadcast, uint32_t replyId, bool toPhoneOnly, int redirect_depth) {
    if (!cmd) return;
    if (redirect_depth > NEXUS_MAX_REDIRECT_DEPTH || commandDepth > NEXUS_MAX_REDIRECT_DEPTH) {
        ReplyContext context{mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, toPhoneOnly,
                             toPhoneOnly || (mp.to != myNodeInfo.my_node_num)};
        const char *error = "JT: Command nesting limit exceeded (max 3).";
        sendReply(context, error, strlen(error));
        return;
    }
    const unsigned pagingGenerationBefore = pagingGeneration;
    struct CommandScope {
        NexusModule &module;
        const char *command;
        const meshtastic_MeshPacket *request;
        bool phoneOnly, broadcast;
        ~CommandScope()
        {
            --module.commandDepth;
            module.activeCommand = command;
            module.activeRequest = request;
            module.activePhoneOnly = phoneOnly;
            module.activeIsBroadcast = broadcast;
                    }
    } scope{*this, activeCommand, activeRequest, activePhoneOnly, activeIsBroadcast};
    ++commandDepth;
    activeCommand = cmd;
    activeRequest = &mp;
    activePhoneOnly = toPhoneOnly;
    activeIsBroadcast = isBroadcast;

    // Nexus-28006 (2026-09-18) — private command dispatcher gate.
    // On PRIVATE builds (NEXUS_PRIVATE_BUILD defined), the opaque
    // private-parser call below handles commands that are not part of
    // the public surface. On PUBLIC builds the #ifdef
    // removes the call entirely; see NexusModule_private.cpp.
    //
    // Bug fix 2026-09-23 (Nexus-28006): the prior implementation
    // did `return;` here on match, which SKIPPED the dispatcher tail
    // (auto-prefix + sendToPhone + queueMeshReply) at line ~9202. Result:
    // private-parser replies were built into replyMsg but NEVER sent.
    // Operator saw total silence for private-parser commands. Fix: on
    // match, send the reply inline via
    // the static sendTail helper, then return. Earlier `goto sendTail`
    // attempt failed to compile because C++ forbids jumping over the
    // C++ variable initializations between this point and the label
    // (CmdBuf static, replyMsg reference binding, memset of replyMsg).
#ifdef NEXUS_PRIVATE_BUILD
    if (handlePrivateCommand(cmd, mp, replyMsg, sizeof(replyMsg))) {
        // Bug fix 2026-09-23 (Nexus-28006): the prior implementation
        // did `return;` here on match, which SKIPPED the dispatcher tail
        // (auto-prefix + sendToPhone + queueMeshReply) at line ~9202. Result:
        // private-parser replies were built into replyMsg but NEVER sent.
        // Operator saw total silence for private-parser commands. Fix:
        // inline the send-tail logic here so the
        // private-parser reply actually reaches the wire. The public
        // dispatcher chain below MUST NOT run — it would clobber replyMsg
        // with "Unknown command" and send the wrong text. We operate on
        // the underlying member array (this->replyMsg) directly because
        // the replyMsg reference binding lives later in the function
        // (line ~579, after the binding we'd otherwise need to skip).
        //
        // Why not `goto sendTail`? C++ forbids jumping over variable
        // initializations (the CmdBuf static, the replyMsg reference
        // binding, the memset of replyMsg at line 580). An earlier
        // `goto sendTail` attempt failed compilation with "jump to
        // label 'sendTail' crosses initialization". The inline-block
        // approach is the only correct one.
        //
        // === BEGIN send-tail (mirrors dispatcher tail at line ~9202) ===
        // Auto-prefix: skip for CHECKSIG (emits its own inline emoji).
        //
        // Bug fix 2026-09-24 (Nexus-28006): previous inline-block used
        // hex escapes ("\xF0\x9F\x9F\xA1 ") for the green-circle (🟢) case
        // but the wrong byte sequence — `\xA1` is yellow-circle (🟡), not
        // green (🟢 = `\xA2`). Dispatcher tail at line ~9262 uses literal
        // UTF-8 source "🟢 " which the compiler emits as F0 9F 9F A2 (green).
        // Fix: use the SAME literal UTF-8 source as the dispatcher tail so
        // the byte sequences are guaranteed to match. No more hand-coded
        // hex escapes — the source is greppable too.
        const bool pp_isChecksigReply =
            strncmp(replyMsg, "✅ JT CHECKSIG", 15) == 0 ||
            strncmp(replyMsg, "🚫 JT CHECKSIG", 15) == 0 ||
            strncmp(replyMsg, "📄 JT CHECKSIG", 15) == 0;
        if (replyMsg[0] != '\0' && !pp_isChecksigReply) {
            const char *pp_sigEmoji;
            if (mp.decoded.xeddsa_signature.size == XEDDSA_SIGNATURE_SIZE) {
                if (mp.xeddsa_signed) {
                    char pp_matchLabel[24] = {0};
                    bool pp_trustedMatch = false;
                    if (nexusModule) {
                        pp_trustedMatch = nexusModule->verifyTrustedSignature(&mp, pp_matchLabel, sizeof(pp_matchLabel));
                    }
                    pp_sigEmoji = pp_trustedMatch ? "🔒 " : "🟢 ";
                } else {
                    pp_sigEmoji = "🟠 "; // signature present but xeddsa_verify() failed
                }
            } else {
                pp_sigEmoji = "🔴 "; // unsigned
            }
            size_t pp_curLen = strlen(replyMsg);
            size_t pp_preLen = strlen(pp_sigEmoji);
            if (pp_curLen + pp_preLen < NEXUS_MAX_REPLY) {
                memmove(replyMsg + pp_preLen, replyMsg, pp_curLen + 1);
                memcpy(replyMsg, pp_sigEmoji, pp_preLen);
            }
        }
        // Send-to-phone + queue-mesh-reply.
        if (strlen(replyMsg) > 0) {
            bool pp_isDM = (mp.to == myNodeInfo.my_node_num);
            bool pp_shouldSendLocal = (toPhoneOnly || !pp_isDM);
            if (pp_shouldSendLocal) {
                size_t pp_rLen = strlen(replyMsg);
                if (pp_rLen > NEXUS_MAX_REPLY_BODY) pp_rLen = NEXUS_MAX_REPLY_BODY;
                uint32_t pp_to = pp_isDM ? (mp.from ? mp.from : myNodeInfo.my_node_num) : 0xFFFFFFFF;
                sendToPhone(pp_to, mp.channel, replyId, replyMsg, pp_rLen);
                LOG_INFO("Nexus: Local reply copy sent.");
            }
            if (!toPhoneOnly) {
                size_t pp_rLen = strlen(replyMsg);
                if (pp_rLen > NEXUS_MAX_REPLY_BODY) pp_rLen = NEXUS_MAX_REPLY_BODY;
                bool pp_isDM2 = (mp.to == myNodeInfo.my_node_num);
                queueMeshReply(pp_isDM2 ? mp.from : 0xFFFFFFFF, mp.channel, replyId, replyMsg, pp_rLen, false, true);
            }
        }
        // === END send-tail ===
        return;
    }
#endif


    // Shared scratch region for CLI command format buffers. The dispatch
    // chain guarantees mutual exclusion: only one arm runs per call, so
    // all these typed-array views safely alias the same 1024 B. Access via
    // `cmdBuf.<member>` (e.g. `memset(cmdBuf.sensBuf, 0, sizeof(cmdBuf.sensBuf));`)
    // — each member is a real array, so `sizeof()` returns its declared size
    // and the compiler keeps the snprintf bounds checks honest.
    //
    // lBuf is 250 to cover both the 160-byte users (PRWHITELIST/RDROP/IDR)
    // and the 250-byte ZH user; the smaller consumers leave 90 bytes unused
    // in their window but it's still inside the 1024 B union allocation.
    union CmdBuf {
        char raw[1024];
        char sensBuf[768];
        char mcuBuf[512];
        char rlsBuf[512];
        char otaInfo[384];
        char b64[400];
        char pbuf[256];
        char envBuf[200];
        char mBuf[160];
        char lBuf[250];
    };
    static CmdBuf cmdBuf;

    // 0. Redirection Logic (Meta-command)
    if (strncasecmp(cmd, "R ", 2) == 0 || strncasecmp(cmd, "REDIRECT ", 9) == 0) {
        const char *p = skipToken(cmd);

        uint32_t targetTo = mp.from;
        uint32_t targetCh = mp.channel;
        bool foundTarget = false;
        const char *next = nullptr;

        if (strncasecmp(p, "CH ", 3) == 0 || strncasecmp(p, "CHAN ", 5) == 0) {
            int offset = (strncasecmp(p, "CH ", 3) == 0) ? 3 : 5;
            int lookedUpCh = findChannelByNameOrIndex(p + offset, &next);
            if (lookedUpCh >= 0 && lookedUpCh < MAX_NUM_CHANNELS && channels.getByIndex(lookedUpCh).role != meshtastic_Channel_Role_DISABLED) {
                targetCh = (uint32_t)lookedUpCh;
                targetTo = 0xFFFFFFFF; // Broadcast to that channel
                p = next;
                foundTarget = true;
            } else {
                foundTarget = false; // Invalid channel target
            }
        } else if (strncasecmp(p, "DM ", 3) == 0) {
            targetTo = findNodeByShortNameOrHex(p + 3, &next);
            if (targetTo != 0) {
                targetCh = 0; // standard DM channel
                p = next;
                foundTarget = true;
            } else {
                foundTarget = false; // Invalid DM target
            }
        }

        while (*p == ' ') p++;

        if (foundTarget && *p != '\0') {
            LOG_INFO("Nexus: Command REDIRECT to %08x Ch %d: %s", (unsigned int)targetTo, (int)targetCh, p);
            // Re-execute with overridden context
            meshtastic_MeshPacket mock = mp;
            mock.channel = targetCh;
            mock.from = (targetTo == 0xFFFFFFFF) ? mp.from : targetTo;
            mock.to = (targetTo == 0xFFFFFFFF) ? 0xFFFFFFFF : myNodeInfo.my_node_num;

            // Swap 'from' so replies target the new node if it's a DM
            if (targetTo != 0xFFFFFFFF) {
                mock.from = targetTo;
            }

            handleNexusCommand(mock, p, (targetTo == 0xFFFFFFFF), replyId, toPhoneOnly, redirect_depth + 1);
            return;
        }
    }

    // Nexus: Command Rate Limiting for nRF52 Stability (code review fix)
    // Only apply cooldown to broadcast commands (/nexus), not targeted ones (/nexus-device, /nexus-node, /nexus-mac)
    if (isBroadcast && commandDepth == 1 && redirect_depth == 0 && !pagingRendering) {
        if (last_cmd_time != 0 && Throttle::isWithinTimespanMs(last_cmd_time, 5000)) {
            LOG_WARN("Nexus: Broadcast rate limit triggered.");
            return;
        }
        last_cmd_time = Time::getMillis();
    }

    LOG_INFO("Nexus: command received: %.*s (Broadcast: %d)", (int)strcspn(cmd, " "), cmd, isBroadcast);
    // Bind replyMsg by REFERENCE to the persistent member array. This keeps
    // sizeof(replyMsg) == NEXUS_MAX_REPLY (not sizeof(char*)) so every
    // snprintf(replyMsg, sizeof(replyMsg), ...) below writes the full body.
    // Using a pointer here would silently truncate to 3-7 chars (the
    // previous 'Nex' bug). The reference binding keeps the array type.
    char (&replyMsg)[NEXUS_MAX_REPLY] = this->replyMsg;
    memset(replyMsg, 0, NEXUS_MAX_REPLY);

    // Safety: Critical identity commands MUST be targeted or DM
    bool isDM = (mp.to == myNodeInfo.my_node_num);
    bool isSensitive = (strncasecmp(cmd, "NAME ", 5) == 0 || strncasecmp(cmd, "OWNER ", 6) == 0);

    if (isSensitive && isBroadcast && !isDM) {
        LOG_WARN("Nexus: Blocking broadcast NAME/OWNER for safety.");
        return; // Silent ignore to prevent mass-wiping a network
    }

    if (strncasecmp(cmd, "HOPMASK ", 8) == 0) {
        const char *p = skipToken(cmd);
        if (strcasecmp(p, "OFF") == 0) {
            hop_mask_active = false;
            // saveVault();
            snprintf(replyMsg, sizeof(replyMsg), "JT: Hop Masking DISABLED.");
        } else {
            int start = atoi(p);
            while (*p && *p != ' ') p++;
            while (*p == ' ') p++;
            int limit = atoi(p);

            if (start >= 0 && start <= 7 && limit >= 0 && limit <= 7) {
                hop_mask_active = true;
                hop_mask_start = (uint8_t)start;
                hop_mask_limit = (uint8_t)limit;
                // saveVault();
                snprintf(replyMsg, sizeof(replyMsg), "JT: Hop Masking ACTIVE (Start:%d, Limit:%d)", start, limit);
            } else {
                snprintf(replyMsg, sizeof(replyMsg), "JT: Invalid Hop Mask (0-7)");
            }
        }
    }
    else if (strcasecmp(cmd, "HOPMASK") == 0) {
        snprintf(replyMsg, sizeof(replyMsg), "JT Hop Masking: %s (S:%d, L:%d)",
                 hop_mask_active ? "ACTIVE" : "OFF", hop_mask_start, hop_mask_limit);
    }
    else if (strncasecmp(cmd, "CH-HOPMASK ", 11) == 0 || strncasecmp(cmd, "CHM ", 4) == 0) {
        const char *p = skipToken(cmd);
        const char *next = NULL;
        int ch = findChannelByNameOrIndex(p, &next);

        if (ch >= 0 && ch < 8) {
            if (strcasecmp(next, "OFF") == 0) {
                nexus_per_ch_hopmask[ch].active = 0;
                snprintf(replyMsg, sizeof(replyMsg), "JT: Ch %d (%s) Hopmask DISABLED", ch, channels.getByIndex(ch).settings.name);
            } else {
                int start = atoi(next);
                next = skipToken(next);
                int limit = atoi(next);
                if (start >= 0 && start <= 7 && limit >= 0 && limit <= 7) {
                    nexus_per_ch_hopmask[ch].active = 1;
                    nexus_per_ch_hopmask[ch].start = (uint8_t)start;
                    nexus_per_ch_hopmask[ch].limit = (uint8_t)limit;
                    snprintf(replyMsg, sizeof(replyMsg), "JT: Ch %d (%s) Hopmask ACTIVE (S:%d, L:%d)", ch, channels.getByIndex(ch).settings.name, start, limit);
                } else snprintf(replyMsg, sizeof(replyMsg), "JT: Invalid Hop Mask (0-7)");
            }
        } else snprintf(replyMsg, sizeof(replyMsg), "JT: Invalid Channel");
    }
    else if (strcasecmp(cmd, "CH-HOPMASK") == 0 || strcasecmp(cmd, "CHM") == 0) {
        int l = snprintf(replyMsg, sizeof(replyMsg), "JT Ch Hopmasks:\n");
        for (int i = 0; i < 8; i++) {
            if (nexus_per_ch_hopmask[i].active) {
                if (l < (int)sizeof(replyMsg) - 40) {
                    l += snprintf(replyMsg + l, sizeof(replyMsg) - l, "Ch %d (%s): S:%d L:%d\n",
                                 i, channels.getByIndex(i).settings.name,
                                 nexus_per_ch_hopmask[i].start, nexus_per_ch_hopmask[i].limit);
                }
            }
        }
    }
    else if (strncasecmp(cmd, "PORT-HOPMASK ", 13) == 0 || strncasecmp(cmd, "PHM ", 4) == 0) {
        const char *p = skipToken(cmd);
        char portStr[16] = {0};
        int i = 0;
        while (*p && *p != ' ' && i < 15) portStr[i++] = *p++;

        // Skip current spaces to get to the first number (start)
        while (*p == ' ') p++;

        int portIdx = -1;
        if (strcasecmp(portStr, "NODEINFO") == 0 || strcasecmp(portStr, "NI") == 0) portIdx = 0;
        else if (strcasecmp(portStr, "TELEMETRY") == 0 || strcasecmp(portStr, "TL") == 0) portIdx = 1;
        else if (strcasecmp(portStr, "POSITION") == 0 || strcasecmp(portStr, "PO") == 0) portIdx = 2;
        else if (strcasecmp(portStr, "TRACEROUTE") == 0 || strcasecmp(portStr, "TR") == 0) portIdx = 3;
        else if (strcasecmp(portStr, "TEXT") == 0 || strcasecmp(portStr, "TX") == 0) portIdx = 4;

        if (portIdx != -1) {
            if (strcasecmp(p, "OFF") == 0) {
                nexus_per_port_hopmask[portIdx].active = 0;
                snprintf(replyMsg, sizeof(replyMsg), "JT: Port %s Hopmask DISABLED", portStr);
            } else {
                int start = atoi(p);
                const char *next = skipToken(p);
                int limit = atoi(next);
                if (start >= 0 && start <= 7 && limit >= 0 && limit <= 7) {
                    nexus_per_port_hopmask[portIdx].active = 1;
                    nexus_per_port_hopmask[portIdx].start = (uint8_t)start;
                    nexus_per_port_hopmask[portIdx].limit = (uint8_t)limit;
                    snprintf(replyMsg, sizeof(replyMsg), "JT: Port %s Hopmask ACTIVE (S:%d, L:%d)", portStr, start, limit);
                } else snprintf(replyMsg, sizeof(replyMsg), "JT: Invalid Hop Mask (0-7)");
            }
        } else snprintf(replyMsg, sizeof(replyMsg), "JT: Invalid Port (NI|TL|PO|TR)");
    }
    else if (strcasecmp(cmd, "PORT-HOPMASK") == 0 || strcasecmp(cmd, "PHM") == 0) {
        int l = snprintf(replyMsg, sizeof(replyMsg), "JT Port Hopmasks:\n");
        const char* pNames[] = {"NI", "TL", "PO", "TR", "TX"};
        for (int i = 0; i < 5; i++) {
            if (nexus_per_port_hopmask[i].active) {
                if (l < (int)sizeof(replyMsg) - 25) {
                    l += snprintf(replyMsg + l, sizeof(replyMsg) - l, "%s: S:%d L:%d\n",
                                 pNames[i], nexus_per_port_hopmask[i].start, nexus_per_port_hopmask[i].limit);
                }
            }
        }
    }
    else if (strncasecmp(cmd, "PRWHITELIST ", 12) == 0 || strncasecmp(cmd, "PRBLACKLIST ", 12) == 0 ||
             strncasecmp(cmd, "PRALLOW ", 8) == 0 || strncasecmp(cmd, "PRBLOCK ", 8) == 0 ||
             strcasecmp(cmd, "PRWHITELIST") == 0 || strcasecmp(cmd, "PRBLACKLIST") == 0 ||
             strcasecmp(cmd, "PRALLOW") == 0 || strcasecmp(cmd, "PRBLOCK") == 0) {
        // Bare forms accepted so FW PRALLOW / FW PRBLOCK delegation works
        // (they dispatch the bare word - A10 in nexus-commands-audit).
        bool isWhite = (strncasecmp(cmd, "PRWHITELIST", 11) == 0 || strncasecmp(cmd, "PRALLOW", 7) == 0);
        const char *p = skipToken(cmd);
        if (!p) p = "";
        while (*p == ' ') p++;

        if (strncasecmp(p, "ADD ", 4) == 0) {
            uint32_t ch = (uint32_t)strtoul(p + 4, NULL, 0);
            if (ch < 8) {
                snprintf(replyMsg, sizeof(replyMsg), "JT: %s - Only Ch 8+ allowed", isWhite ? "Whitelist" : "Blacklist");
            } else {
                uint32_t *list = isWhite ? nexus_ch_whitelist : nexus_ch_blacklist;
                int slot = -1;
                for(int i=0; i<16; i++) {
                    if (list[i] == ch) { slot = -2; break; }
                    if (list[i] == 0 && slot == -1) slot = i;
                }
                if (slot == -2) snprintf(replyMsg, sizeof(replyMsg), "JT: Ch %u (0x%X) already in list", (unsigned int)ch, (unsigned int)ch);
                else if (slot == -1) snprintf(replyMsg, sizeof(replyMsg), "JT: List full (max 16)");
                else {
                    list[slot] = ch;
                    // saveVault();
                    snprintf(replyMsg, sizeof(replyMsg), "JT: Ch %u (0x%X) added to %s", (unsigned int)ch, (unsigned int)ch, isWhite ? "Whitelist" : "Blacklist");
                }
            }
        }
        else if (strncasecmp(p, "REM ", 4) == 0) {
            uint32_t ch = (uint32_t)strtoul(p + 4, NULL, 0);
            uint32_t *list = isWhite ? nexus_ch_whitelist : nexus_ch_blacklist;
            bool found = false;
            for(int i=0; i<16; i++) {
                if (list[i] == ch) { list[i] = 0; found = true; break; }
            }
            if (found) {
                // saveVault();
                snprintf(replyMsg, sizeof(replyMsg), "JT: Ch %u (0x%X) removed from %s", (unsigned int)ch, (unsigned int)ch, isWhite ? "Whitelist" : "Blacklist");
            } else {
                snprintf(replyMsg, sizeof(replyMsg), "JT: Ch %u (0x%X) not found in %s", (unsigned int)ch, (unsigned int)ch, isWhite ? "Whitelist" : "Blacklist");
            }
        }
        else if (strcasecmp(p, "LIST") == 0 || *p == '\0') {
            uint32_t *list = isWhite ? nexus_ch_whitelist : nexus_ch_blacklist;
            memset(cmdBuf.lBuf, 0, sizeof(cmdBuf.lBuf));
            int lLen = snprintf(cmdBuf.lBuf, sizeof(cmdBuf.lBuf), "JT %s: ", isWhite ? "Whitelist" : "Blacklist");
            for(int i=0; i<16; i++) {
                if (list[i] != 0) {
                    if (lLen < (int)sizeof(cmdBuf.lBuf) - 15) {
                        lLen += snprintf(cmdBuf.lBuf + lLen, sizeof(cmdBuf.lBuf) - lLen, "%u(0x%X) ", (unsigned int)list[i], (unsigned int)list[i]);
                    }
                }
            }
            snprintf(replyMsg, sizeof(replyMsg), "%s", cmdBuf.lBuf);
            memset(cmdBuf.lBuf, 0, sizeof(cmdBuf.lBuf));
        }
    }
    else if (strncasecmp(cmd, "DROP ", 5) == 0 || strncasecmp(cmd, "UNDROP ", 7) == 0) {
        bool setDrop = strncasecmp(cmd, "DROP ", 5) == 0;
        const char *portStr = skipToken(cmd);
        bool handled = true;

        if (strcasecmp(portStr, "NODEINFO") == 0 || strcasecmp(portStr, "NI") == 0) drop_nodeinfo = setDrop;
        else if (strcasecmp(portStr, "POSITION") == 0 || strcasecmp(portStr, "PO") == 0) drop_position = setDrop;
        else if (strcasecmp(portStr, "TELEMETRY") == 0 || strcasecmp(portStr, "TL") == 0) drop_telemetry = setDrop;
        else if (strcasecmp(portStr, "NEIGHBOR") == 0 || strcasecmp(portStr, "NB") == 0) drop_neighbor = setDrop;
        else if (strcasecmp(portStr, "TRACEROUTE") == 0 || strcasecmp(portStr, "TR") == 0) drop_traceroute = setDrop;
        else if (strcasecmp(portStr, "TRACEROUTE_REMOTE") == 0 || strcasecmp(portStr, "TRR") == 0) drop_traceroute_remote = setDrop;
        else if (strcasecmp(portStr, "TRACEROUTE_LOCAL") == 0 || strcasecmp(portStr, "TRL") == 0) drop_traceroute_local = setDrop;
        else if (strcasecmp(portStr, "PRIVATE") == 0 || strcasecmp(portStr, "PR") == 0) drop_private = setDrop;
        else if (strcasecmp(portStr, "HOP6") == 0 || strcasecmp(portStr, "H6") == 0) drop_hop6 = setDrop;
        else if (strcasecmp(portStr, "HOP7") == 0 || strcasecmp(portStr, "H7") == 0) drop_hop7 = setDrop;
        else if (strcasecmp(portStr, "REQ_NODEINFO") == 0 || strcasecmp(portStr, "RNI") == 0) drop_req_nodeinfo = setDrop;
        else if (strcasecmp(portStr, "REQ_TELEMETRY") == 0 || strcasecmp(portStr, "RTL") == 0) drop_req_telemetry = setDrop;
        else if (strcasecmp(portStr, "REQ_POSITION") == 0 || strcasecmp(portStr, "RPO") == 0) drop_req_position = setDrop;
        else if (strcasecmp(portStr, "BR_NODEINFO") == 0 || strcasecmp(portStr, "BRNI") == 0) drop_br_nodeinfo = setDrop;
        else if (strcasecmp(portStr, "BR_TELEMETRY") == 0 || strcasecmp(portStr, "BRTL") == 0) drop_br_telemetry = setDrop;
        else if (strcasecmp(portStr, "BR_POSITION") == 0 || strcasecmp(portStr, "BRPO") == 0) drop_br_position = setDrop;
        else if (strcasecmp(portStr, "ACKS") == 0 || strcasecmp(portStr, "AK") == 0) drop_acks = setDrop;
        else if (strcasecmp(portStr, "PKI") == 0 || strcasecmp(portStr, "PK") == 0) drop_pki = setDrop;
        else if (strcasecmp(portStr, "PKI_REMOTE") == 0 || strcasecmp(portStr, "PKR") == 0) drop_pki_remote = setDrop;
        else if (strcasecmp(portStr, "PKI_LOCAL") == 0 || strcasecmp(portStr, "PKL") == 0) drop_pki_local = setDrop;
        else if (strcasecmp(portStr, "FAV_BYPASS") == 0 || strcasecmp(portStr, "FB") == 0) fav_bypass = setDrop;
        else if (strcasecmp(portStr, "PREHOP") == 0 || strcasecmp(portStr, "PH") == 0) drop_prehop = setDrop;
        else if (strcasecmp(portStr, "STORE_FORWARD") == 0 || strcasecmp(portStr, "SF") == 0) drop_store_forward = setDrop;
        else if (strcasecmp(portStr, "ADMIN") == 0 || strcasecmp(portStr, "AD") == 0) drop_admin = setDrop;
        else if (strcasecmp(portStr, "TEXT") == 0 || strcasecmp(portStr, "TX") == 0) drop_text = setDrop;
        else if (strcasecmp(portStr, "KEYVER") == 0 || strcasecmp(portStr, "KV") == 0) drop_keyver = setDrop;
        else if (strcasecmp(portStr, "ALERT") == 0 || strcasecmp(portStr, "AL") == 0) drop_alert = setDrop;
        else if (strcasecmp(portStr, "WAYPOINT") == 0 || strcasecmp(portStr, "WP") == 0) drop_waypoint = setDrop;
        else if (strcasecmp(portStr, "TRACEROUTE_DIRECT") == 0 || strcasecmp(portStr, "TRD") == 0 || strcasecmp(portStr, "TR_DIRECT") == 0) drop_traceroute_direct = setDrop;
        else if (strcasecmp(portStr, "TRACEROUTE_DIRECT_REMOTE") == 0 || strcasecmp(portStr, "TRDR") == 0 || strcasecmp(portStr, "TR_DIRECT_REMOTE") == 0) drop_traceroute_direct_remote = setDrop;
        else if (strcasecmp(portStr, "TRACEROUTE_DIRECT_LOCAL") == 0 || strcasecmp(portStr, "TRDL") == 0 || strcasecmp(portStr, "TR_DIRECT_LOCAL") == 0) drop_traceroute_direct_local = setDrop;
        else if (strcasecmp(portStr, "SKIP_SKIP") == 0 || strcasecmp(portStr, "SS") == 0) skip_skip_handle = setDrop;
        else if (strcasecmp(portStr, "RL_TEXT") == 0 || strcasecmp(portStr, "RLTX") == 0) drop_rl_text = setDrop;
        else if (strcasecmp(portStr, "RL_TRACE") == 0 || strcasecmp(portStr, "RLTR") == 0) drop_rl_trace = setDrop;
        else if (strcasecmp(portStr, "RL_OTHER") == 0 || strcasecmp(portStr, "RLOT") == 0) drop_rl_other = setDrop;
        else if (strcasecmp(portStr, "RL_NODEINFO") == 0 || strcasecmp(portStr, "RLNI") == 0) drop_rl_nodeinfo = setDrop;
        else if (strcasecmp(portStr, "RL_POSITION") == 0 || strcasecmp(portStr, "RLPO") == 0) drop_rl_position = setDrop;
        else if (strcasecmp(portStr, "RL_TELEMETRY") == 0 || strcasecmp(portStr, "RLTL") == 0) drop_rl_telemetry = setDrop;
        else if (strcasecmp(portStr, "RL_PRIVATE") == 0 || strcasecmp(portStr, "RLPR") == 0) drop_rl_private = setDrop;
        else if (strcasecmp(portStr, "TELE_DEVICE") == 0 || strcasecmp(portStr, "TD") == 0) drop_tele_device = setDrop;
        else if (strcasecmp(portStr, "TELE_POWER") == 0 || strcasecmp(portStr, "TP") == 0) drop_tele_power = setDrop;
        else if (strcasecmp(portStr, "TELE_ENV") == 0 || strcasecmp(portStr, "TE") == 0) drop_tele_env = setDrop;
        else if (strcasecmp(portStr, "TELE_OTHER") == 0 || strcasecmp(portStr, "TO") == 0) drop_tele_other = setDrop;
        else if (strcasecmp(portStr, "NO_MQTT") == 0 || strcasecmp(portStr, "NM") == 0) drop_no_mqtt = setDrop;
        else if (strcasecmp(portStr, "POS_MANUAL") == 0 || strcasecmp(portStr, "PM") == 0) drop_pos_manual = setDrop;
        else if (strcasecmp(portStr, "POS_GPS") == 0 || strcasecmp(portStr, "PG") == 0) drop_pos_gps = setDrop;
        else if (strcasecmp(portStr, "POS_OTHER") == 0 || strcasecmp(portStr, "POO") == 0) drop_pos_other = setDrop;
        else handled = false;

        if (handled) {
            snprintf(replyMsg, sizeof(replyMsg), "JT: %s filter %s", portStr, setDrop ? "ACTIVE" : "DISABLED");
        } else {
            snprintf(replyMsg, sizeof(replyMsg), "JT: Invalid filter target");
        }
    } else if (strncasecmp(cmd, "OUTDROP ", 8) == 0 || strncasecmp(cmd, "UNOUTDROP ", 10) == 0) {
        bool setDrop = (strncasecmp(cmd, "OUTDROP ", 8) == 0);
        const char *portStr = skipToken(cmd);
        bool handled = true;

        if (strcasecmp(portStr, "NODEINFO") == 0 || strcasecmp(portStr, "NI") == 0) drop_out_nodeinfo = setDrop;
        else if (strcasecmp(portStr, "POSITION") == 0 || strcasecmp(portStr, "PO") == 0) drop_out_position = setDrop;
        else if (strcasecmp(portStr, "TELEMETRY") == 0 || strcasecmp(portStr, "TL") == 0) drop_out_telemetry = setDrop;
        else if (strcasecmp(portStr, "TRACEROUTE") == 0 || strcasecmp(portStr, "TR") == 0) drop_out_traceroute = setDrop;
        else if (strcasecmp(portStr, "ADMIN") == 0 || strcasecmp(portStr, "AD") == 0) drop_out_admin = setDrop;
        else if (strcasecmp(portStr, "ROUTING") == 0 || strcasecmp(portStr, "RO") == 0) drop_out_routing = setDrop;
        else handled = false;

        if (handled) {
            snprintf(replyMsg, sizeof(replyMsg), "JT Output: %s filter %s", portStr, setDrop ? "ACTIVE" : "DISABLED");
        } else {
            snprintf(replyMsg, sizeof(replyMsg), "JT: Invalid output filter target");
        }
    }
    else if (strncasecmp(cmd, "ZH ", 3) == 0 || strcasecmp(cmd, "ZH") == 0) {
        const char *p = skipToken(cmd);

        if (strcasecmp(p, "ON") == 0) zero_hop_config |= 0x01;
        else if (strcasecmp(p, "OFF") == 0) zero_hop_config &= ~0x01;
        else if (strncasecmp(p, "POS ", 4) == 0) {
            if (strcasecmp(p+4, "ON") == 0) zero_hop_config |= 0x04;
            else if (strcasecmp(p+4, "OFF") == 0) zero_hop_config &= ~0x04;
        }
        else if (strncasecmp(p, "TELE ", 5) == 0) {
            if (strcasecmp(p+5, "ON") == 0) zero_hop_config |= 0x02;
            else if (strcasecmp(p+5, "OFF") == 0) zero_hop_config &= ~0x02;
        }
        else if (strncasecmp(p, "ROUTER ", 7) == 0 || strncasecmp(p, "RT ", 3) == 0) {
            // Auto-router mask consultation toggle (zero_hop_config bit 3).
            const char *verb = skipToken(p);
            if (strcasecmp(verb, "ON") == 0) zero_hop_config |= 0x08;
            else if (strcasecmp(verb, "OFF") == 0) zero_hop_config &= ~0x08;
        }
        else if (strncasecmp(p, "CLIENT ", 7) == 0 || strncasecmp(p, "CLIENT_BASE ", 13) == 0 ||
                 strncasecmp(p, "CB ", 3) == 0) {
            // Auto client_base mask consultation toggle (zero_hop_config bit 4).
            const char *verb = skipToken(p);
            if (strcasecmp(verb, "ON") == 0) zero_hop_config |= 0x10;
            else if (strcasecmp(verb, "OFF") == 0) zero_hop_config &= ~0x10;
        }
        else if (strncasecmp(p, "ADD ", 4) == 0) {
            uint8_t id = (uint8_t)strtoul(p+4, NULL, 16);
            setZeroHopNode(id, true);
            snprintf(replyMsg, sizeof(replyMsg), "JT: ZH ID 0x%02X ADDED", id);
        }
        else if (strncasecmp(p, "DEL ", 4) == 0) {
            uint8_t id = (uint8_t)strtoul(p+4, NULL, 16);
            setZeroHopNode(id, false);
            snprintf(replyMsg, sizeof(replyMsg), "JT: ZH ID 0x%02X REMOVED", id);
        }
        else if (strcasecmp(p, "SYNC") == 0) {
            int count = 0;
            for (size_t i = 0; i < nodeDB->getNumMeshNodes(); i++) {
                meshtastic_NodeInfoLite *node = nodeDB->getMeshNodeByIndex(i);
                if (node && nexus_is_favorite(node) && nexus_has_user(node) &&
                    IS_ONE_OF(node->role, meshtastic_Config_DeviceConfig_Role_ROUTER,
                              meshtastic_Config_DeviceConfig_Role_ROUTER_LATE,
                              meshtastic_Config_DeviceConfig_Role_CLIENT_BASE)) {
                    setZeroHopNode(nodeDB->getLastByteOfNodeNum(node->num), true);
                    count++;
                }
            }
            snprintf(replyMsg, sizeof(replyMsg), "JT: ZH SYNC OK (%d nodes)", count);
        }
        else if (strncasecmp(p, "CLEAR ", 6) == 0) {
            const char *target = skipToken(p);
            if (strcasecmp(target, "ROUTER") == 0 || strcasecmp(target, "RT") == 0) {
                memset(zero_hop_auto_router_mask, 0, sizeof(zero_hop_auto_router_mask));
                snprintf(replyMsg, sizeof(replyMsg), "JT: ZH ROUTER MASK CLEARED");
            } else if (strcasecmp(target, "CLIENT") == 0 || strcasecmp(target, "CLIENT_BASE") == 0 ||
                       strcasecmp(target, "CB") == 0) {
                memset(zero_hop_auto_client_mask, 0, sizeof(zero_hop_auto_client_mask));
                snprintf(replyMsg, sizeof(replyMsg), "JT: ZH CLIENT MASK CLEARED");
            } else {
                snprintf(replyMsg, sizeof(replyMsg), "JT: ZH CLEAR <ROUTER|CLIENT>");
            }
        }
        else if (strcasecmp(p, "CLEAR") == 0) {
            memset(zero_hop_mask, 0, sizeof(zero_hop_mask));
            snprintf(replyMsg, sizeof(replyMsg), "JT: ZH MASK CLEARED");
        }
        else if (strcasecmp(p, "EXPORT") == 0 || strcasecmp(p, "OUTPUT") == 0 ||
                 strcasecmp(p, "OUT") == 0 || strcasecmp(p, "B64") == 0) {
            // Emit the 32-byte zero_hop_mask as base64 (44 chars) on a single
            // line. Suitable for copy/paste between nodes or committing to
            // a notebook. Companion to ZH IMPORT.
            memset(cmdBuf.b64, 0, sizeof(cmdBuf.b64));
            nexus_base64_encode(cmdBuf.b64, zero_hop_mask, sizeof(zero_hop_mask), false);
            snprintf(replyMsg, sizeof(replyMsg), "JT ZH B64: %s", cmdBuf.b64);
        }
        else if (strncasecmp(p, "IMPORT ", 7) == 0 || strncasecmp(p, "INPUT ", 6) == 0 ||
                 strncasecmp(p, "IN ", 3) == 0) {
            // Parse a 44-char base64 blob (with or without padding) and
            // overwrite zero_hop_mask. Whitespace between chars tolerated;
            // decoder skips non-alphabet chars.
            const char *b64 = skipToken(p); // skip the keyword -> land on <b64>
            uint8_t decoded[32] = {0};
            size_t written = 0;
            int err = this->nexus_base64_decode(decoded, sizeof(decoded), &written, b64);
            if (err != 0 || written != sizeof(zero_hop_mask)) {
                snprintf(replyMsg, sizeof(replyMsg), "JT: ZH IMPORT failed (got %u bytes, need %u)", (unsigned)written, (unsigned)sizeof(zero_hop_mask));
            } else {
                memcpy(zero_hop_mask, decoded, sizeof(zero_hop_mask));
                snprintf(replyMsg, sizeof(replyMsg), "JT: ZH IMPORT OK (32 bytes / 256 relays)");
            }
        }
        else if (strcasecmp(p, "LIST") == 0 || strncasecmp(p, "LIST ", 5) == 0) {
            // Optional target (2026-10-07): MANUAL (default) | ROUTER | CLIENT | ALL.
            const char *tgt = skipToken(p);
            bool vok = true;
            const char *vlabel = "manual";
            const uint8_t *zhMask = zeroHopListMask(tgt, &vok, &vlabel);
            if (!vok) {
                snprintf(replyMsg, sizeof(replyMsg), "JT: ZH unknown list '%s' (MANUAL|ROUTER|CLIENT|ALL)", tgt);
            } else {
            memset(cmdBuf.lBuf, 0, sizeof(cmdBuf.lBuf));
            cmdBuf.lBuf[0] = '\0';
            int lLen = snprintf(cmdBuf.lBuf, sizeof(cmdBuf.lBuf), "ZH List (%s):\n", vlabel);
            int count = 0;
            static char names[32];
            for(int i=0; i<256; i++) {
                bool inList = zhMask ? ((zhMask[i / 8] & (1 << (i % 8))) != 0) : isZeroHopNode(i);
                if (inList) {
                    memset(names, 0, sizeof(names));
                    getRelayNames(names, sizeof(names), (uint8_t)i);
                    if (lLen < (int)sizeof(cmdBuf.lBuf) - 40) {
                        lLen += snprintf(cmdBuf.lBuf + lLen, sizeof(cmdBuf.lBuf) - lLen, "[%02X] %s\n", i, (names[0] != '\0') ? names : "?");
                        count++;
                    }
                }
            }
            if (count == 0) {
                snprintf(replyMsg, sizeof(replyMsg), "JT: ZH List (%s) is EMPTY", vlabel);
            } else {
                snprintf(replyMsg, sizeof(replyMsg), "%s", cmdBuf.lBuf);
            }
            memset(cmdBuf.lBuf, 0, sizeof(cmdBuf.lBuf));
            memset(names, 0, sizeof(names));
            }
        }
        else if (strcasecmp(p, "SHORT") == 0 || strncasecmp(p, "SHORT ", 6) == 0 ||
                 strcasecmp(p, "S") == 0 || strncasecmp(p, "S ", 2) == 0 ||
                 strcasecmp(p, "HEX") == 0 || strncasecmp(p, "HEX ", 4) == 0) {
            // Compact list: just the active hex relay IDs, space-separated,
            // no relay names. Fits in replyMsg even for ~150 entries.
            const char *tgt = skipToken(p);
            bool vok = true;
            const char *vlabel = "manual";
            const uint8_t *zhMask = zeroHopListMask(tgt, &vok, &vlabel);
            if (!vok) {
                snprintf(replyMsg, sizeof(replyMsg), "JT: ZH unknown list '%s' (MANUAL|ROUTER|CLIENT|ALL)", tgt);
            } else {
            char *dst = cmdBuf.lBuf;
            size_t left = sizeof(cmdBuf.lBuf);
            int count = 0;
            for (int i = 0; i < 256; i++) {
                bool inList = zhMask ? ((zhMask[i / 8] & (1 << (i % 8))) != 0) : isZeroHopNode(i);
                if (inList) {
                    int w = snprintf(dst, left, (count == 0) ? "%02X" : " %02X", (unsigned)i);
                    if (w <= 0 || (size_t)w >= left) break; // safety: stop on overflow
                    dst += w;
                    left -= (size_t)w;
                    count++;
                }
            }
            *dst = '\0';
            if (count == 0) {
                snprintf(replyMsg, sizeof(replyMsg), "JT: ZH [%s] (short) empty", vlabel);
            } else {
                snprintf(replyMsg, sizeof(replyMsg), "JT ZH [%s] (%u): %s", vlabel, (unsigned)count, cmdBuf.lBuf);
            }
            }
        }
        else if (strcasecmp(p, "PAGED") == 0 || strncasecmp(p, "PAGED ", 6) == 0 ||
                 strcasecmp(p, "PG") == 0 || strncasecmp(p, "PG ", 3) == 0 ||
                 strcasecmp(p, "P") == 0 || strncasecmp(p, "P ", 2) == 0) {
            // Same content as LIST but each entry appended via appendPaging,
            // so a large mask (>~25 entries) splits into multiple mesh
            // replies instead of being clipped by the single-reply buffer.
            const char *tgt = skipToken(p);
            bool vok = true;
            const char *vlabel = "manual";
            const uint8_t *zhMask = zeroHopListMask(tgt, &vok, &vlabel);
            if (!vok) {
                snprintf(replyMsg, sizeof(replyMsg), "JT: ZH unknown list '%s' (MANUAL|ROUTER|CLIENT|ALL)", tgt);
            } else {
            uint32_t to = (mp.to == myNodeInfo.my_node_num) ? mp.from : 0xFFFFFFFF;
            char ptitle[32];
            snprintf(ptitle, sizeof(ptitle), "ZH List (%s)", vlabel);
            startPaging(to, mp.channel, replyId, ptitle);
            static char names[32];
            int count = 0;
            for (int i = 0; i < 256; i++) {
                bool inList = zhMask ? ((zhMask[i / 8] & (1 << (i % 8))) != 0) : isZeroHopNode(i);
                if (inList) {
                    memset(names, 0, sizeof(names));
                    getRelayNames(names, sizeof(names), (uint8_t)i);
                    char entry[64];
                    snprintf(entry, sizeof(entry), "[%02X] %s\n", (unsigned)i, (names[0] != '\0') ? names : "?");
                    appendPaging(entry);
                    count++;
                }
            }
            if (count == 0) {
                appendPaging("(empty)");
            } else {
                char footer[32];
                snprintf(footer, sizeof(footer), "-- %u entries --", (unsigned)count);
                appendPaging(footer);
            }
            memset(names, 0, sizeof(names));
            snprintf(replyMsg, sizeof(replyMsg), "JT: Paging ZH [%s] list (%u entries)...", vlabel, (unsigned)count);
            }
        }
        else if (strcasecmp(p, "LISTS") == 0) {
            // Paged view of ALL THREE lists in one run (2026-10-07):
            // manual, then auto router/router_late, then auto client_base —
            // each with its own header so an operator can tell where every
            // zero-hop relay came from. Disabled auto lists get an [OFF] tag.
            uint32_t to = (mp.to == myNodeInfo.my_node_num) ? mp.from : 0xFFFFFFFF;
            startPaging(to, mp.channel, replyId, "ZH Lists");
            static char names[32];
            static char hdr[48];
            static char foot[48];
            const uint8_t *masks[3] = { zero_hop_mask, zero_hop_auto_router_mask, zero_hop_auto_client_mask };
            static const char *const labels[3] = { "manual", "router", "client" };
            const bool on[3] = { true, isZeroHopAutoRouterEnabled(), isZeroHopAutoClientEnabled() };
            int grand = 0;
            for (int m = 0; m < 3; m++) {
                int count = 0;
                for (int i = 0; i < 256; i++) {
                    if (masks[m][i / 8] & (1 << (i % 8))) count++;
                }
                snprintf(hdr, sizeof(hdr), "-- %s%s (%d) --", labels[m], on[m] ? "" : " [OFF]", count);
                appendPaging(hdr);
                grand += count;
                for (int i = 0; i < 256; i++) {
                    if (!(masks[m][i / 8] & (1 << (i % 8)))) continue;
                    memset(names, 0, sizeof(names));
                    getRelayNames(names, sizeof(names), (uint8_t)i);
                    char entry[64];
                    snprintf(entry, sizeof(entry), "[%02X] %s\n", (unsigned)i, (names[0] != '\0') ? names : "?");
                    appendPaging(entry);
                }
            }
            snprintf(foot, sizeof(foot), "-- all: %d entries --", grand);
            appendPaging(foot);
            memset(names, 0, sizeof(names));
            snprintf(replyMsg, sizeof(replyMsg), "JT: Paging ZH lists (manual/router/client, %d total)...", grand);
        }

        if (replyMsg[0] == '\0') {
            // Auto-mask counts in brackets make "populated all the time" visible
            // over the mesh (and show CLEAR re-filling within ~1 s).
            auto maskCount = [](const uint8_t *m) {
                int c = 0;
                for (int b = 0; b < 32; b++) c += __builtin_popcount(m[b]);
                return c;
            };
            snprintf(replyMsg, sizeof(replyMsg), "JT: ZH %s (POS:%s TELE:%s RTR:%s[%d] CLI:%s[%d])",
                isZeroHopEnabled() ? "ON" : "OFF",
                shouldPreservePosition() ? "ON" : "OFF",
                shouldPreserveTelemetry() ? "ON" : "OFF",
                isZeroHopAutoRouterEnabled() ? "ON" : "OFF",
                maskCount(zero_hop_auto_router_mask),
                isZeroHopAutoClientEnabled() ? "ON" : "OFF",
                maskCount(zero_hop_auto_client_mask));
        }
    }
    else if (strcasecmp(cmd, "PSRAM") == 0) {
        uint32_t total = memGet.getPsramSize();
        if (total == 0) {
            snprintf(replyMsg, sizeof(replyMsg), "JT: No PSRAM detected");
        } else {
            uint32_t free = memGet.getFreePsram();
            uint32_t used = total - free;
            float pUsed = (float)used / total * 100.0f;
            snprintf(replyMsg, sizeof(replyMsg), "JT PSRAM: %u/%u KB (%.1f%% used, %u KB free)",
                (unsigned int)(used / 1024), (unsigned int)(total / 1024), pUsed, (unsigned int)(free / 1024));
        }
    }
    else if (strncasecmp(cmd, "GPIO ", 5) == 0 || strcasecmp(cmd, "GPIO") == 0) {
        const char *p = skipToken(cmd);
        if (*p) {
            if (strncasecmp(p, "WATCH", 5) == 0) {
                const char *sub = skipToken(p);
                if (*sub) {
                    if (strcasecmp(sub, "OFF") == 0) {
                        gpio_watch_mask = 0;
                        snprintf(replyMsg, sizeof(replyMsg), "JT: GPIO Watching DISABLED");
                    } else {
                        int pin = atoi(sub);
                        if (pin >= 0 && pin <= 63) {
                            if (gpio_watch_mask & (1ULL << pin)) {
                                gpio_watch_mask &= ~(1ULL << pin);
                                snprintf(replyMsg, sizeof(replyMsg), "JT: Unwatched GPIO %d", pin);
                            } else {
                                gpio_watch_mask |= (1ULL << pin);
                                pinMode(pin, INPUT_PULLUP);
                                if (digitalRead(pin)) gpio_watch_prev |= (1ULL << pin);
                                else gpio_watch_prev &= ~(1ULL << pin);
                                snprintf(replyMsg, sizeof(replyMsg), "JT: Watching GPIO %d", pin);
                            }
                        } else snprintf(replyMsg, sizeof(replyMsg), "JT: Invalid Pin (0-63)");
                    }
                    // saveVault removed (SCOPE RULE: only SAVE persists in tactical)
                } else {
                    snprintf(replyMsg, sizeof(replyMsg), "JT GPIO Watch Mask: 0x%08lX%08lX",
                             (unsigned long)(gpio_watch_mask >> 32), (unsigned long)(gpio_watch_mask & 0xFFFFFFFFu));
                }
            } else {
                char op = toupper(*p);
                p = skipToken(p);
                if (*p) {
                    int pin = atoi(p);
                    if (pin >= 0 && pin <= 128) {
                        if (op == 'W') {
                            p = skipToken(p);
                            if (*p) {
                                int val = (strcasecmp(p, "H") == 0 || strcasecmp(p, "HIGH") == 0 || atoi(p) > 0) ? 1 : 0;
                                pinMode(pin, OUTPUT);
                                digitalWrite(pin, val);
                                snprintf(replyMsg, sizeof(replyMsg), "JT: GPIO %d set to %s", pin, val ? "HIGH" : "LOW");
                            } else snprintf(replyMsg, sizeof(replyMsg), "JT: Usage: GPIO W <pin> <H|L>");
                        } else if (op == 'R') {
                            pinMode(pin, INPUT_PULLUP);
                            int val = digitalRead(pin);
                            snprintf(replyMsg, sizeof(replyMsg), "JT: GPIO %d is %s", pin, val ? "HIGH" : "LOW");
                        } else if (op == 'M') {
                            p = skipToken(p);
                            if (*p) {
                                char mBuf[16];
                                int i = 0;
                                while(p[i] && p[i] != ' ' && i < 15) { mBuf[i] = p[i]; i++; }
                                mBuf[i] = '\0';

                                if (strcasecmp(mBuf, "IN") == 0) {
                                    pinMode(pin, INPUT);
                                    snprintf(replyMsg, sizeof(replyMsg), "JT: GPIO %d mode set to INPUT", pin);
                                } else if (strcasecmp(mBuf, "OUT") == 0) {
                                    pinMode(pin, OUTPUT);
                                    snprintf(replyMsg, sizeof(replyMsg), "JT: GPIO %d mode set to OUTPUT", pin);
                                } else if (strcasecmp(mBuf, "PU") == 0) {
                                    pinMode(pin, INPUT_PULLUP);
                                    snprintf(replyMsg, sizeof(replyMsg), "JT: GPIO %d mode set to INPUT_PULLUP", pin);
                                } else {
                                    snprintf(replyMsg, sizeof(replyMsg), "JT: GPIO M invalid mode (IN|OUT|PU)");
                                }
                            } else snprintf(replyMsg, sizeof(replyMsg), "JT: Usage: GPIO M <pin> <IN|OUT|PU>");
                        } else snprintf(replyMsg, sizeof(replyMsg), "JT: GPIO [W|R|M|WATCH]");
                    } else snprintf(replyMsg, sizeof(replyMsg), "JT: Invalid Pin (0-128)");
                } else snprintf(replyMsg, sizeof(replyMsg), "JT: GPIO <W|R|M|WATCH> <pin>");
            }
        } else snprintf(replyMsg, sizeof(replyMsg), "JT: GPIO <W|R|M|WATCH> <pin> [val]");
    }
    else if (strncasecmp(cmd, "TA ", 3) == 0 || strcasecmp(cmd, "TA") == 0 ||
             strncasecmp(cmd, "TR_ALERT ", 9) == 0 || strcasecmp(cmd, "TR_ALERT") == 0) {
        const char *p = skipToken(cmd);

        if (*p) {
            char val[16] = {0};
            int i = 0;
            while(p[i] && p[i] != ' ' && i < 15) { val[i] = p[i]; i++; }
            val[i] = '\0';

            if (strcasecmp(val, "ON") == 0) trace_alert = true;
            else if (strcasecmp(val, "OFF") == 0) trace_alert = false;

            const char *m = p + i;
            while(*m == ' ') m++;
            if (*m) trace_alert_mode = atoi(m);
        }
        snprintf(replyMsg, sizeof(replyMsg), "JT: Trace Alert %s (Mode: %u)", trace_alert ? "ON" : "OFF", (unsigned int)trace_alert_mode);
    }
    else if (strncasecmp(cmd, "TRUSTED ", 8) == 0 || strcasecmp(cmd, "TRUSTED") == 0) {
        const char* slotStr = (cmd[7] == ' ') ? cmd + 8 : nullptr;
        char slotBuf[8];
        const char* nameStr = nullptr;

        if (slotStr) {
            int i = 0;
            while(slotStr[i] && slotStr[i] != ' ' && i < 7) {
                slotBuf[i] = slotStr[i];
                i++;
            }
            slotBuf[i] = '\0';
            nameStr = slotStr + i;
            while(*nameStr == ' ') nameStr++;
            if (*nameStr == '\0') nameStr = nullptr;

            int slot = atoi(slotBuf);
            if (slot >= 0 && slot < 2) {
                if (nameStr) {
                    if (strcasecmp(nameStr, "OFF") == 0) {
                        trusted_ch_active[slot] = false;
                        snprintf(replyMsg, sizeof(replyMsg), "JT: Trusted Slot %d DISABLED", slot);
                    } else {
                        trusted_ch_active[slot] = true;
                        strncpy(trusted_ch_names[slot], nameStr, 15);
                        trusted_ch_names[slot][15] = '\0';
                        snprintf(replyMsg, sizeof(replyMsg), "JT: Trusted Slot %d -> '%s'", slot, trusted_ch_names[slot]);
                    }
                } else {
                     snprintf(replyMsg, sizeof(replyMsg), "JT TRUSTED [%d]: %s (%s)",
                              slot, trusted_ch_names[slot], trusted_ch_active[slot] ? "ON" : "OFF");
                }
            } else {
                snprintf(replyMsg, sizeof(replyMsg), "JT: Invalid slot (0-1)");
            }
        } else {
            snprintf(replyMsg, sizeof(replyMsg), "JT TRUSTED Slots:\n[0] %s (%s)\n[1] %s (%s)",
                     trusted_ch_names[0], trusted_ch_active[0] ? "ON" : "OFF",
                     trusted_ch_names[1], trusted_ch_active[1] ? "ON" : "OFF");
        }
    }
    else if (strcasecmp(cmd, "META_ALERT") == 0 || strcasecmp(cmd, "MA") == 0 ||
             strncasecmp(cmd, "META_ALERT ", 11) == 0 || strncasecmp(cmd, "MA ", 3) == 0) {
        const char *p = skipToken(cmd);

        if (*p) {
            char val[16] = {0};
            int i = 0;
            while(p[i] && p[i] != ' ' && i < 15) { val[i] = p[i]; i++; }
            val[i] = '\0';

            if (strcasecmp(val, "ON") == 0) meta_alert = true;
            else if (strcasecmp(val, "OFF") == 0) meta_alert = false;

            const char *m = p + i;
            while(*m == ' ') m++;
            if (*m) meta_alert_mode = atoi(m);
        }
        snprintf(replyMsg, sizeof(replyMsg), "JT: Meta Alert %s (Mode: %u)", meta_alert ? "ON" : "OFF", (unsigned int)meta_alert_mode);
    }
    else if (strcasecmp(cmd, "SETPREAMBLE") == 0 || strcasecmp(cmd, "SETPRE") == 0 || strncasecmp(cmd, "SETPREAMBLE ", 12) == 0 || strncasecmp(cmd, "SETPRE ", 7) == 0) {
        const char *p = skipToken(cmd);
        if (*p != '\0') {
            int val = atoi(p);
            if (val >= 6 && val <= 1024) {
                preamble_override = (uint16_t)val;
                snprintf(replyMsg, sizeof(replyMsg), "JT: Preamble Length set to %d. Applying in 5s...", val);

                // Trigger radio re-init
                pendingLoRaApplyAt = Time::getMillis() + 5000;
                pendingLoRaConfig = true;
                meshtastic_Config pc = meshtastic_Config_init_default;
                pc.which_payload_variant = meshtastic_Config_lora_tag;
                pc.payload_variant.lora = config.lora;
                pendingConfig = pc;
                setIntervalFromNow(50);
            } else {
                snprintf(replyMsg, sizeof(replyMsg), "JT: Invalid Preamble (6-1024)");
            }
        } else {
            snprintf(replyMsg, sizeof(replyMsg), "JT Preamble Override: %u", preamble_override);
        }
    }
    else if (strcasecmp(cmd, "SETSYNCWORD") == 0 || strcasecmp(cmd, "SETSW") == 0 || strncasecmp(cmd, "SETSYNCWORD ", 12) == 0 || strncasecmp(cmd, "SETSW ", 6) == 0) {
        // Nexus: SETSYNCWORD - runtime LoRa sync word override (v40).
        // 0 or RESET restores Meshtastic default 0x2B. Any byte 0x00-0xFF is accepted
        // (including 0x34 - caller is responsible for the implications, e.g. picking
        // a LoRaWAN syncword will make the node talk LoRaWAN, not Meshtastic).
        const char *p = skipToken(cmd);
        if (*p != '\0') {
            if (strcasecmp(p, "RESET") == 0 || strcasecmp(p, "DEFAULT") == 0 || strcasecmp(p, "DEF") == 0) {
                syncword_override = 0;
                snprintf(replyMsg, sizeof(replyMsg), "JT: Sync Word reset to default 0x2B. Applying in 5s...");

                pendingLoRaApplyAt = Time::getMillis() + 5000;
                pendingLoRaConfig = true;
                meshtastic_Config pc = meshtastic_Config_init_default;
                pc.which_payload_variant = meshtastic_Config_lora_tag;
                pc.payload_variant.lora = config.lora;
                pendingConfig = pc;
                setIntervalFromNow(50);
            } else {
                // Parse hex (0x..) or decimal. SyncWord is a single byte 0x00-0xFF.
                char *endptr = nullptr;
                unsigned long val = strtoul(p, &endptr, 0);
                if (endptr == p || val > 0xFF) {
                    snprintf(replyMsg, sizeof(replyMsg), "JT: Invalid SyncWord (0x00-0xFF, RESET)");
                } else {
                    syncword_override = (uint8_t)val;
                    snprintf(replyMsg, sizeof(replyMsg), "JT: Sync Word set to 0x%02X. Applying in 5s...", syncword_override);

                    pendingLoRaApplyAt = Time::getMillis() + 5000;
                    pendingLoRaConfig = true;
                    meshtastic_Config pc = meshtastic_Config_init_default;
                    pc.which_payload_variant = meshtastic_Config_lora_tag;
                    pc.payload_variant.lora = config.lora;
                    pendingConfig = pc;
                    setIntervalFromNow(50);
                }
            }
        } else {
            if (syncword_override == 0) {
                snprintf(replyMsg, sizeof(replyMsg), "JT Sync Word: 0x2B (Meshtastic default)");
            } else {
                snprintf(replyMsg, sizeof(replyMsg), "JT Sync Word: 0x%02X (override)", syncword_override);
            }
        }
    }
    else if (strncasecmp(cmd, "ALIAS ", 6) == 0 || strcasecmp(cmd, "ALIAS") == 0) {
        const char *sub = skipToken(cmd);
        if (strncasecmp(sub, "SET ", 4) == 0) {
            sub += 4;
            const char *aliasToken;
            uint32_t id = findNodeByShortNameOrHex(sub, &aliasToken);
            char alias[16] = {0};
            int i = 0;
            while (*aliasToken && *aliasToken != ' ' && i < 15) alias[i++] = *aliasToken++;
            alias[i] = '\0';

            if (id != 0 && i > 0 && i <= 4) {
                int slot = -1;
                for (int j = 0; j < 16; j++) {
                    if (nexus_aliases[j].nodeID == id || (nexus_aliases[j].nodeID != 0 && strcasecmp(nexus_aliases[j].alias, alias) == 0)) {
                        slot = j; break;
                    }
                }
                if (slot == -1) {
                    for (int j = 0; j < 16; j++) { if (nexus_aliases[j].nodeID == 0) { slot = j; break; } }
                }

                if (slot != -1) {
                    nexus_aliases[slot].nodeID = id;
                    strncpy(nexus_aliases[slot].alias, alias, 4);
                    nexus_aliases[slot].alias[4] = '\0';
                    snprintf(replyMsg, sizeof(replyMsg), "JT: Alias '%s' set for !%08x", alias, (unsigned int)id);
                } else snprintf(replyMsg, sizeof(replyMsg), "JT: Alias table full");
            } else snprintf(replyMsg, sizeof(replyMsg), "JT: Usage: ALIAS SET <node> <alias(4)>");
        } else if (strncasecmp(sub, "DEL ", 4) == 0) {
            sub += 4;
            char target[16] = {0};
            int i = 0;
            while (*sub && *sub != ' ' && i < 15) target[i++] = *sub++;
            bool found = false;
            for (int j = 0; j < 16; j++) {
                if (nexus_aliases[j].nodeID != 0 && strcasecmp(nexus_aliases[j].alias, target) == 0) {
                    nexus_aliases[j].nodeID = 0;
                    found = true; break;
                }
            }
            if (found) { snprintf(replyMsg, sizeof(replyMsg), "JT: Alias '%s' deleted", target); }
            else snprintf(replyMsg, sizeof(replyMsg), "JT: Alias not found");
        } else if (strcasecmp(sub, "LIST") == 0) {
            startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "Aliases");
            for (int j = 0; j < 16; j++) {
                if (nexus_aliases[j].nodeID != 0) {
                    char entry[32];
                    snprintf(entry, sizeof(entry), "%s -> !%08x\n", nexus_aliases[j].alias, (unsigned int)nexus_aliases[j].nodeID);
                    appendPaging(entry);
                }
            }
            snprintf(replyMsg, sizeof(replyMsg), "JT: Paging aliases to mesh...");
        } else snprintf(replyMsg, sizeof(replyMsg), "JT: ALIAS [SET|DEL|LIST]");
    }
    else if (strncasecmp(cmd, "MACRO ", 6) == 0 || strcasecmp(cmd, "MACRO") == 0 ||
             strncasecmp(cmd, "MC ", 3) == 0 || strcasecmp(cmd, "MC") == 0 ||
             strncasecmp(cmd, "MR ", 3) == 0 || strcasecmp(cmd, "MR") == 0) {
        bool isRun = (strncasecmp(cmd, "MR ", 3) == 0 || strcasecmp(cmd, "MR") == 0);
        const char *sub = skipToken(cmd);

        if (!isRun && strncasecmp(sub, "SET ", 4) == 0) {
            sub += 4;
            char name[16] = {0};
            int i = 0;
            while (*sub && *sub != ' ' && i < 15) name[i++] = *sub++;
            while (*sub == ' ') sub++; // Skip space before command

            if (i > 0 && i <= 11 && *sub != '\0' && strlen(sub) <= 127) {
                int slot = -1;
                for (int j = 0; j < 4; j++) { if (strcasecmp(nexus_macros[j].name, name) == 0) { slot = j; break; } }
                if (slot == -1) { for (int j = 0; j < 4; j++) { if (nexus_macros[j].name[0] == '\0') { slot = j; break; } } }

                if (slot != -1) {
                    strncpy(nexus_macros[slot].name, name, 11);
                    size_t commandLength = strlen(sub);
                    memmove(nexus_macros[slot].command, sub, commandLength);
                    memset(nexus_macros[slot].command + commandLength, 0, sizeof(nexus_macros[slot].command) - commandLength);
                    nexus_macros[slot].name[11] = '\0';
                    nexus_macros[slot].command[127] = '\0';
                    snprintf(replyMsg, sizeof(replyMsg), "JT: Macro '%s' stored", name);
                } else snprintf(replyMsg, sizeof(replyMsg), "JT: Macro slots full");
            } else
                snprintf(replyMsg, sizeof(replyMsg), "JT: Usage: MACRO SET <name> <cmd>");
        } else if (isRun || strncasecmp(sub, "RUN ", 4) == 0) {
            if (!isRun) sub += 4;
            char name[16] = {0};
            int i = 0;
            while (*sub && *sub != ' ' && i < 15) name[i++] = *sub++;

            int slot = -1;
            for (int j = 0; j < 4; j++) { if (strcasecmp(nexus_macros[j].name, name) == 0) { slot = j; break; } }

            if (slot != -1) {
                if (macroStepExecuting && strchr(nexus_macros[slot].command, ';')) {
                    snprintf(replyMsg, sizeof(replyMsg), "JT: Nested macro sequences are not supported; sequence unchanged.");
                } else if (strchr(nexus_macros[slot].command, ';')) {
                    // Sequence macro
                    strncpy(macro_running_cmd, nexus_macros[slot].command, 127);
                    macro_step_ptr = 0;
                    macro_next_step_at = Time::getMillis(); // Run first step immediately
                    macro_reply_to = replyId;
                    macro_reply_ch = mp.channel;
                    macro_from_id = mp.from;
                    macroRequest = mp;
                    macroIsBroadcast = isBroadcast;
                    macroDepth = redirect_depth + 1;
                    macro_to_phone = toPhoneOnly;
                                        snprintf(replyMsg, sizeof(replyMsg), "JT: Running macro sequence '%s'...", name);
                    setIntervalFromNow(50);
                } else {
                    // Direct macro
                    snprintf(replyMsg, sizeof(replyMsg), "JT: Executing macro '%s'...", name);
                    handleNexusCommand(mp, nexus_macros[slot].command, isBroadcast, replyId, toPhoneOnly, redirect_depth + 1);
                    return; // Prevent duplicate reply
                }
            } else snprintf(replyMsg, sizeof(replyMsg), "JT: Macro '%s' not found", name);
        } else if (!isRun && strncasecmp(sub, "DEL ", 4) == 0) {
            sub += 4;
            char name[16] = {0};
            int i = 0;
            while (*sub && *sub != ' ' && i < 15) name[i++] = *sub++;
            bool found = false;
            for (int j = 0; j < 4; j++) {
                if (strcasecmp(nexus_macros[j].name, name) == 0) {
                    nexus_macros[j].name[0] = '\0';
                    found = true; break;
                }
            }
            if (found) { snprintf(replyMsg, sizeof(replyMsg), "JT: Macro deleted"); }
            else snprintf(replyMsg, sizeof(replyMsg), "JT: Macro not found");
        } else if (!isRun && strcasecmp(sub, "LIST") == 0) {
            startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "Macros");
            static char entry[150];
            for (int j = 0; j < 4; j++) {
                if (nexus_macros[j].name[0] != '\0') {
                    memset(entry, 0, sizeof(entry));
                    snprintf(entry, sizeof(entry), "%s: %s\n", nexus_macros[j].name, nexus_macros[j].command);
                    appendPaging(entry);
                }
            }
            memset(entry, 0, sizeof(entry));
            snprintf(replyMsg, sizeof(replyMsg), "JT: Paging macros to mesh...");
        } else snprintf(replyMsg, sizeof(replyMsg), "JT: MACRO [SET|RUN|DEL|LIST]");
    }
    else if (strncasecmp(cmd, "FSIG", 4) == 0) {
        const char *p = cmd + 4;
        while (*p == ' ') p++;

        // Helper lambdas not used (C++11); use static functions instead via structs
        struct SigFmt {
            // Format signature for LIST display
            static void render(const NexusSignature *sig, char *out, size_t outSize) {
                if (sig->len == 0) { snprintf(out, outSize, "OFF"); return; }
                if (sig->type == 0) {
                    // hex
                    int l = 0;
                    for (int i = 0; i < sig->len && l < (int)outSize - 3; i++) {
                        l += snprintf(out + l, outSize - l, "%02X", sig->bytes[i]);
                    }
                } else if (sig->type == 1) {
                    // base64
		    // We already have a helper function for base64, do we need to duplicate this?
                    static const char b64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
                    uint8_t pad = (3 - (sig->len % 3)) % 3;
                    int inLen = sig->len + pad;
                    int l = 0;
                    for (int i = 0; i < inLen; i += 3) {
                        uint8_t b0 = (i < sig->len) ? sig->bytes[i] : 0;
                        uint8_t b1 = (i + 1 < sig->len) ? sig->bytes[i + 1] : 0;
                        uint8_t b2 = (i + 2 < sig->len) ? sig->bytes[i + 2] : 0;
                        uint32_t triple = (b0 << 16) | (b1 << 8) | b2;
                        l += snprintf(out + l, outSize - l, "%c%c",
                                      b64[(triple >> 18) & 0x3F], b64[(triple >> 12) & 0x3F]);
                        if (i + 1 < inLen)
                            l += snprintf(out + l, outSize - l, "%c", b64[(triple >> 6) & 0x3F]);
                        else if (pad >= 2) l += snprintf(out + l, outSize - l, "=");
                        if (i + 2 < inLen)
                            l += snprintf(out + l, outSize - l, "%c", b64[triple & 0x3F]);
                        else if (pad >= 1) l += snprintf(out + l, outSize - l, "=");
                    }
                } else {
                    // text
                    int l = 0;
                    for (int i = 0; i < sig->len && l < (int)outSize - 2; i++) {
                        char c = (char)sig->bytes[i];
                        if (c >= 0x20 && c < 0x7F) out[l++] = c;
                        else l += snprintf(out + l, outSize - l, "\\x%02X", (unsigned char)c);
                    }
                    out[l] = '\0';
                }
            }
            // Detect type from input string: returns 0=hex, 1=base64, 2=text
            static int detectType(const char *s) {
                // Skip 0x prefix for hex
                if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) s += 2;
                int hasHexOnly = 1, hexLen = 0;
                int hasB64Only = 1, b64Len = 0;
                int hasPrintable = 0, anyChar = 0;
                for (; *s; s++) {
                    unsigned char c = (unsigned char)*s;
                    anyChar = 1;
                    if (c >= '0' && c <= '9') { hexLen++; b64Len++; hasPrintable++; continue; }
                    if ((c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F')) { hexLen++; b64Len++; hasPrintable++; continue; }
                    if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z')) { hasB64Only = 0; hasPrintable++; hasHexOnly = 0; continue; }
                    if (c == '+' || c == '/' || c == '=') { hexLen = 0; b64Len++; hasHexOnly = 0; continue; }
                    // Other char -> text
                    hasHexOnly = 0; hasB64Only = 0;
                }
                (void)hasPrintable;
                if (!anyChar) return 2; // empty -> treat as text (will fail len check)
                // Prefer hex if all chars hex AND length is even AND >=2
                if (hasHexOnly && (hexLen % 2) == 0 && hexLen >= 2) return 0;
                // Otherwise prefer base64 if all chars b64 AND length is multiple of 4
                if (hasB64Only && (b64Len % 4) == 0) return 1;
                return 2; // fallback text
            }
            // Decode input string into bytes. Returns bytes written, or -1 on error.
            static int decode(const char *s, uint8_t type, uint8_t *out, uint8_t maxLen) {
                if (type == 0) {
                    // hex
                    if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) s += 2;
                    int len = strlen(s);
                    if ((len % 2) != 0) return -1;
                    int n = len / 2;
                    if (n > maxLen) return -1;
                    for (int i = 0; i < n; i++) {
                        char b[3] = {s[i*2], s[i*2+1], 0};
                        char *end;
                        long v = strtol(b, &end, 16);
                        if (*end != 0) return -1;
                        out[i] = (uint8_t)v;
                    }
                    return n;
                } else if (type == 1) {
                    // base64
                    struct B64 {
                        static int idx(unsigned char c) {
                            if (c >= 'A' && c <= 'Z') return c - 'A';
                            if (c >= 'a' && c <= 'z') return c - 'a' + 26;
                            if (c >= '0' && c <= '9') return c - '0' + 52;
                            if (c == '+') return 62;
                            if (c == '/') return 63;
                            return -1;
                        }
                    };
                    int len = strlen(s);
                    if ((len % 4) != 0) return -1;
                    int pad = 0;
                    if (len >= 1 && s[len-1] == '=') pad++;
                    if (len >= 2 && s[len-2] == '=') pad++;
                    int n = (len / 4) * 3 - pad;
                    if (n > maxLen) return -1;
                    int oi = 0;
                    for (int i = 0; i < len; i += 4) {
                        int v0 = B64::idx((unsigned char)s[i]);
                        int v1 = B64::idx((unsigned char)s[i+1]);
                        int v2 = (s[i+2] == '=') ? 0 : B64::idx((unsigned char)s[i+2]);
                        int v3 = (s[i+3] == '=') ? 0 : B64::idx((unsigned char)s[i+3]);
                        if (v0 < 0 || v1 < 0 || v2 < 0 || v3 < 0) return -1;
                        uint32_t triple = ((uint32_t)v0 << 18) | ((uint32_t)v1 << 12) | ((uint32_t)v2 << 6) | (uint32_t)v3;
                        if (oi < n) out[oi++] = (triple >> 16) & 0xFF;
                        if (oi < n) out[oi++] = (triple >> 8) & 0xFF;
                        if (oi < n) out[oi++] = triple & 0xFF;
                    }
                    return n;
                } else {
                    // text
                    int len = strlen(s);
                    if (len > maxLen) return -1;
                    memcpy(out, s, len);
                    return len;
                }
            }
        };

        if (*p == '\0' || strcasecmp(p, "LIST") == 0) {
            startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "Signatures");
            static char buf[64];
            static char line[96];
            for (int i = 0; i < 8; i++) {
                memset(buf, 0, sizeof(buf));
                SigFmt::render(&nexus_signatures[i], buf, sizeof(buf));
                const char *typeStr = (nexus_signatures[i].len == 0) ? "" :
                                     (nexus_signatures[i].type == 0) ? " (hex)" :
                                     (nexus_signatures[i].type == 1) ? " (b64)" : " (text)";
                memset(line, 0, sizeof(line));
                if (nexus_signatures[i].len == 0) {
                    snprintf(line, sizeof(line), "S%d: OFF\n", i + 1);
                } else {
                    snprintf(line, sizeof(line), "S%d: %s%s (%uB)\n", i + 1, buf, typeStr, nexus_signatures[i].len);
                }
                appendPaging(line);
            }
            memset(buf, 0, sizeof(buf));
            memset(line, 0, sizeof(line));
            snprintf(replyMsg, sizeof(replyMsg), "JT: Paging signatures...");
        }
        else if (strncasecmp(p, "SET ", 4) == 0) {
            const char *args = p + 4;
            while (*args == ' ') args++;
            int slot = atoi(args);
            if (slot < 1 || slot > 8) {
                snprintf(replyMsg, sizeof(replyMsg), "JT: Slot must be 1-8.");
            } else {
                while (*args >= '0' && *args <= '9') args++;
                while (*args == ' ') args++;
                if (*args == '\0') {
                    snprintf(replyMsg, sizeof(replyMsg), "JT: FSIG SET <1-8> <hex|base64|text>");
                } else {
                    int type = SigFmt::detectType(args);
                    uint8_t buf[16];
                    int n = SigFmt::decode(args, type, buf, 16);
                    if (n < 0) {
                        snprintf(replyMsg, sizeof(replyMsg), "JT: Invalid format or too long (max 16B).");
                    } else if (n == 0) {
                        snprintf(replyMsg, sizeof(replyMsg), "JT: Empty signature not allowed.");
                    } else {
                        memset(&nexus_signatures[slot - 1], 0, sizeof(NexusSignature));
                        memcpy(nexus_signatures[slot - 1].bytes, buf, n);
                        nexus_signatures[slot - 1].len = (uint8_t)n;
                        nexus_signatures[slot - 1].type = (uint8_t)type;
                        // saveVault removed (SCOPE RULE: only SAVE persists in tactical)
                        const char *typeStr = (type == 0) ? "hex" : (type == 1) ? "base64" : "text";
                        snprintf(replyMsg, sizeof(replyMsg), "JT: S%d set (%s, %uB).", slot, typeStr, n);
                    }
                }
            }
        }
        else if (strncasecmp(p, "OFF ", 4) == 0) {
            int slot = atoi(p + 4);
            if (slot < 1 || slot > 8) {
                snprintf(replyMsg, sizeof(replyMsg), "JT: Slot must be 1-8.");
            } else {
                memset(&nexus_signatures[slot - 1], 0, sizeof(NexusSignature));
                // saveVault removed (SCOPE RULE: only SAVE persists in tactical)
                snprintf(replyMsg, sizeof(replyMsg), "JT: S%d cleared.", slot);
            }
        }
        else {
            snprintf(replyMsg, sizeof(replyMsg), "JT: FSIG [LIST|SET <1-8> <value>|OFF <1-8>]");
        }
    }
    else if (strcasecmp(cmd, "FREEZE") == 0 || strncasecmp(cmd, "FREEZE ", 7) == 0) {
        // v64: /nexus FREEZE — per-subsystem freeze flags. Each flag gates one
        // /prefs/*.proto writer. `ON` sets all four; per-subsystem commands
        // (`NODEFREEZE`, `WARMFREEZE`, `DEVICEFREEZE`, `CONFIGFREEZE`) toggle
        // one. Persistence via /nexus SAVE (SCOPE RULE). DBFLUSH bypasses all.
        //
        // Backward compat aliases: `DISK` → all four on (legacy ON); `DB` →
        // nodedb on (legacy DB); naked `OFF` → all four off.
        const char *sub = (strlen(cmd) > 7) ? cmd + 7 : "";
        while (*sub == ' ') sub++;

        // Helper: parse optional [ON|OFF] suffix. Returns -1 if not found.
        auto parseOnOff = [](const char *s) -> int {
            while (*s == ' ') s++;
            if (*s == '\0') return -1;
            if (strcasecmp(s, "ON") == 0)  return 1;
            if (strcasecmp(s, "OFF") == 0) return 0;
            return -2; // garbage
        };

        if (*sub == '\0' || strcasecmp(sub, "STATUS") == 0) {
            snprintf(replyMsg, sizeof(replyMsg),
                     "JT FREEZE:\n"
                     "  NodeDB:  %s\n"
                     "  Warm:    %s\n"
                     "  Device:  %s\n"
                     "  Config:  %s\n"
                     "(DBFLUSH bypasses all four)",
                     freeze_nodedb ? "frozen" : "unfrozen",
                     freeze_warm   ? "frozen" : "unfrozen",
                     freeze_device ? "frozen" : "unfrozen",
                     freeze_config ? "frozen" : "unfrozen");
        } else if (strcasecmp(sub, "ON") == 0 || strcasecmp(sub, "DISK") == 0) {
            // Legacy `DISK` alias preserves old semantics: all four on.
            freeze_nodedb = freeze_warm = freeze_device = freeze_config = true;
            // saveVault removed (SCOPE RULE: only SAVE persists in tactical)
            snprintf(replyMsg, sizeof(replyMsg),
                     "JT: All freezes ACTIVE. saveToDisk() blocked on every segment.\n"
                     "Use 'FREEZE OFF' to resume or 'DBFLUSH' to force-flush.");
        } else if (strcasecmp(sub, "OFF") == 0) {
            freeze_nodedb = freeze_warm = freeze_device = freeze_config = false;
            // saveVault removed (SCOPE RULE: only SAVE persists in tactical)
            snprintf(replyMsg, sizeof(replyMsg),
                     "JT: All freezes cleared. Normal save resumes.");
        } else if (strncasecmp(sub, "NODEFREEZE", 10) == 0) {
            int v = parseOnOff(sub + 10);
            if (v < 0) {
                snprintf(replyMsg, sizeof(replyMsg),
                         "JT: FREEZE NODEFREEZE [ON|OFF] (current: %s)",
                         freeze_nodedb ? "ON" : "OFF");
            } else {
                freeze_nodedb = v;
                snprintf(replyMsg, sizeof(replyMsg),
                         "JT: NodeDB freeze %s. (Persist via /nexus SAVE.)",
                         freeze_nodedb ? "ON" : "OFF");
            }
        } else if (strncasecmp(sub, "WARMFREEZE", 10) == 0) {
            int v = parseOnOff(sub + 10);
            if (v < 0) {
                snprintf(replyMsg, sizeof(replyMsg),
                         "JT: FREEZE WARMFREEZE [ON|OFF] (current: %s)",
                         freeze_warm ? "ON" : "OFF");
            } else {
                freeze_warm = v;
                snprintf(replyMsg, sizeof(replyMsg),
                         "JT: Warm freeze %s. (Persist via /nexus SAVE.)",
                         freeze_warm ? "ON" : "OFF");
            }
        } else if (strncasecmp(sub, "DEVICEFREEZE", 12) == 0) {
            int v = parseOnOff(sub + 12);
            if (v < 0) {
                snprintf(replyMsg, sizeof(replyMsg),
                         "JT: FREEZE DEVICEFREEZE [ON|OFF] (current: %s)",
                         freeze_device ? "ON" : "OFF");
            } else {
                freeze_device = v;
                snprintf(replyMsg, sizeof(replyMsg),
                         "JT: Device freeze %s. (Persist via /nexus SAVE.)",
                         freeze_device ? "ON" : "OFF");
            }
        } else if (strncasecmp(sub, "CONFIGFREEZE", 12) == 0) {
            int v = parseOnOff(sub + 12);
            if (v < 0) {
                snprintf(replyMsg, sizeof(replyMsg),
                         "JT: FREEZE CONFIGFREEZE [ON|OFF] (current: %s)",
                         freeze_config ? "ON" : "OFF");
            } else {
                freeze_config = v;
                snprintf(replyMsg, sizeof(replyMsg),
                         "JT: Config freeze %s. (Persist via /nexus SAVE.)",
                         freeze_config ? "ON" : "OFF");
            }
        } else if (strcasecmp(sub, "DB") == 0 || strcasecmp(sub, "DBONLY") == 0) {
            // Legacy `DB` alias: just nodedb.
            if (!freeze_nodedb) {
                nodeDB->saveNodeDatabaseToDisk();
            }
            freeze_nodedb = true;
            // saveVault removed (SCOPE RULE: only SAVE persists in tactical)
            snprintf(replyMsg, sizeof(replyMsg),
                     "JT: NodeDB freeze ACTIVE. nodes.proto frozen at current RAM state.\n"
                     "Use 'FREEZE OFF' to resume or 'DBFLUSH' to force-flush.");
        } else {
            snprintf(replyMsg, sizeof(replyMsg),
                     "JT: FREEZE [ON|OFF|STATUS|NODEFREEZE|..]\n"
                     "  ON/OFF  = all four on/off\n"
                     "  NODEFREEZE/WARMFREEZE/DEVICEFREEZE/CONFIGFREEZE [ON|OFF]\n"
                     "Legacy: DISK == ON, DB == NODEFREEZE ON. DBFLUSH bypasses all.");
        }
    }
    else if (strcasecmp(cmd, "DBFLUSH") == 0) {
        // Manual save of NodeDB bypassing all four freeze flags.
        // forceWrite flag in saveToDiskNoRetry not exposed; we just call the
        // underlying saveProto path by temporarily clearing the flags.
        bool prevNodedb  = freeze_nodedb;
        bool prevWarm    = freeze_warm;
        bool prevDevice  = freeze_device;
        bool prevConfig  = freeze_config;
        freeze_nodedb = freeze_warm = freeze_device = freeze_config = false;
        bool ok = nodeDB->saveNodeDatabaseToDisk();
        freeze_nodedb = prevNodedb;
        freeze_warm   = prevWarm;
        freeze_device = prevDevice;
        freeze_config = prevConfig;
        snprintf(replyMsg, sizeof(replyMsg),
                 "JT: DBFLUSH %s (freeze flags unchanged: nodedb=%d warm=%d device=%d config=%d).",
                 ok ? "OK" : "FAILED", (int)freeze_nodedb, (int)freeze_warm, (int)freeze_device, (int)freeze_config);
    }
    else if (strcasecmp(cmd, "FIREWALL") == 0 || strcasecmp(cmd, "FW") == 0 ||
             strncasecmp(cmd, "FIREWALL ", 10) == 0 || strncasecmp(cmd, "FW ", 3) == 0) {
        // FIREWALL / FW: aggregated firewall status (single paging buffer)
        // FIREWALL <sub> / FW <sub>: delegate to existing handler (recursive dispatch).
        const char *sub = nullptr;
        if (strncasecmp(cmd, "FIREWALL ", 10) == 0) sub = cmd + 10;
        else if (strncasecmp(cmd, "FW ", 3) == 0) sub = cmd + 3;
        while (sub && *sub == ' ') sub++;

        if (sub && *sub != '\0') {
            // Anti-recursion: refuse nested FW/FIREWALL to avoid loops
            if (strncasecmp(sub, "FW ", 3) == 0 || strcasecmp(sub, "FW") == 0 ||
                strncasecmp(sub, "FIREWALL ", 10) == 0 || strcasecmp(sub, "FIREWALL") == 0) {
                snprintf(replyMsg, sizeof(replyMsg), "JT: FW nested call rejected (anti-recursion guard).");
            } else {
                // Delegate to existing handler with the sub-command as the new cmd.
                // Original reply_id is preserved; sub-handler fills replyMsg.
                handleNexusCommand(mp, sub, isBroadcast, replyId, toPhoneOnly, redirect_depth + 1);
                return;
            }
        } else {
            // No sub-command: build a single paged report with all sections.
            startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "FW STATUS");
            // Use persistent member buffers (zero stack; safe because FW
            // handler is terminal, not re-entered from another handler).
            static char line[160];
            static char sec[96];
            memset(line, 0, sizeof(line));
            memset(sec, 0, sizeof(sec));

            // === DROPS (active flags + WL/BL) ===
            appendPaging("== DROPS ==");
#define FW_APPEND_DROP(name, var)                                                                                                \
    if (var) {                                                                                                                   \
        appendPaging(name " ");                                                                                                  \
    }
            FW_APPEND_DROP("NI", drop_nodeinfo);
            FW_APPEND_DROP("PO", drop_position);
            FW_APPEND_DROP("POM", drop_pos_manual);
            FW_APPEND_DROP("POG", drop_pos_gps);
            FW_APPEND_DROP("POO", drop_pos_other);
            FW_APPEND_DROP("TL", drop_telemetry);
            FW_APPEND_DROP("TLD", drop_tele_device);
            FW_APPEND_DROP("TLP", drop_tele_power);
            FW_APPEND_DROP("TLE", drop_tele_env);
            FW_APPEND_DROP("NB", drop_neighbor);
            FW_APPEND_DROP("TR", drop_traceroute);
            FW_APPEND_DROP("RTR", drop_traceroute_remote);
            FW_APPEND_DROP("LTR", drop_traceroute_local);
            FW_APPEND_DROP("PV", drop_private);
            FW_APPEND_DROP("H6", drop_hop6);
            FW_APPEND_DROP("H7", drop_hop7);
            FW_APPEND_DROP("AK", drop_acks);
            FW_APPEND_DROP("FB", fav_bypass);
            FW_APPEND_DROP("PK", drop_pki);
            FW_APPEND_DROP("PKR", drop_pki_remote);
            FW_APPEND_DROP("PKL", drop_pki_local);
            FW_APPEND_DROP("PH", drop_prehop);
            FW_APPEND_DROP("SF", drop_store_forward);
            FW_APPEND_DROP("AD", drop_admin);
            FW_APPEND_DROP("TX", drop_text);
            FW_APPEND_DROP("KV", drop_keyver);
            FW_APPEND_DROP("AL", drop_alert);
            FW_APPEND_DROP("WP", drop_waypoint);
            FW_APPEND_DROP("TD", drop_traceroute_direct);
            FW_APPEND_DROP("TDR", drop_traceroute_direct_remote);
            FW_APPEND_DROP("TDL", drop_traceroute_direct_local);
            FW_APPEND_DROP("SS", skip_skip_handle);
#undef FW_APPEND_DROP
            appendPaging("\n");

            // White/Black channel lists (compact)
            bool hasWL = false;
            for (int i = 0; i < 16; i++) if (nexus_ch_whitelist[i] != 0) hasWL = true;
            if (hasWL) {
                int wl = snprintf(line, sizeof(line), "WL:");
                for (int i = 0; i < 16; i++) {
                    if (nexus_ch_whitelist[i] != 0 && (int)sizeof(line) - wl > 12) {
                        wl += snprintf(line + wl, sizeof(line) - wl, " 0x%X", (unsigned int)nexus_ch_whitelist[i]);
                    }
                }
                wl += snprintf(line + wl, sizeof(line) - wl, "\n");
                appendPaging(line);
            }
            bool hasBL = false;
            for (int i = 0; i < 16; i++) if (nexus_ch_blacklist[i] != 0) hasBL = true;
            if (hasBL) {
                int bl = snprintf(line, sizeof(line), "BL:");
                for (int i = 0; i < 16; i++) {
                    if (nexus_ch_blacklist[i] != 0 && (int)sizeof(line) - bl > 12) {
                        bl += snprintf(line + bl, sizeof(line) - bl, " 0x%X", (unsigned int)nexus_ch_blacklist[i]);
                    }
                }
                bl += snprintf(line + bl, sizeof(line) - bl, "\n");
                appendPaging(line);
            }
            appendPaging("\n");

            // === OUTDROPS ===
            appendPaging("== OUTDROPS ==");
#define FW_APPEND_OUT(name, var)                                                                                                 \
    if (var) {                                                                                                                   \
        appendPaging(name " ");                                                                                                  \
    }
            FW_APPEND_OUT("NI", drop_out_nodeinfo);
            FW_APPEND_OUT("PO", drop_out_position);
            FW_APPEND_OUT("TL", drop_out_telemetry);
            FW_APPEND_OUT("TR", drop_out_traceroute);
            FW_APPEND_OUT("AD", drop_out_admin);
            FW_APPEND_OUT("RO", drop_out_routing);
            FW_APPEND_OUT("FB", fav_bypass);
#undef FW_APPEND_OUT
            appendPaging("\n\n");

            // === HEXDROPS / HEXOUTDROPS ===
            {
                uint64_t dr = getNexusDropHex();
                uint64_t odr = getNexusOutDropHex();
                snprintf(line, sizeof(line), "== HEX ==\nDROPS:     0x%08lX%08lX\nOUTDROPS:  0x%08lX%08lX\n\n",
                         (unsigned long)(dr >> 32), (unsigned long)(dr & 0xFFFFFFFFu),
                         (unsigned long)(odr >> 32), (unsigned long)(odr & 0xFFFFFFFFu));
            }
            appendPaging(line);

            // === INVALID rules ===
            snprintf(line, sizeof(line),
                     "== INVALID ==\nNULL:%s(%u) SPAIN:%s(%u) GERM:%s(%u) HW:%s(%u) ID:%s(%u)\n\n",
                     inv_null_island ? "ON" : "OFF", (unsigned int)inv_drop_null,
                     inv_out_spain ? "ON" : "OFF", (unsigned int)inv_drop_spain,
                     inv_out_germany ? "ON" : "OFF", (unsigned int)inv_drop_germany,
                     inv_hw_unset ? "ON" : "OFF", (unsigned int)inv_drop_hw,
                     inv_id_mismatch ? "ON" : "OFF", (unsigned int)inv_drop_id);
            appendPaging(line);

            // === PRALLOW (Whitelist) ===
            int wl2 = snprintf(line, sizeof(line), "== PRALLOW ==");
            bool anyWL = false;
            for (int i = 0; i < 16; i++) {
                if (nexus_ch_whitelist[i] != 0 && (int)sizeof(line) - wl2 > 14) {
                    wl2 += snprintf(line + wl2, sizeof(line) - wl2, " %u(0x%X)", (unsigned int)nexus_ch_whitelist[i], (unsigned int)nexus_ch_whitelist[i]);
                    anyWL = true;
                }
            }
            if (!anyWL) wl2 += snprintf(line + wl2, sizeof(line) - wl2, " (empty)");
            wl2 += snprintf(line + wl2, sizeof(line) - wl2, "\n");
            appendPaging(line);

            // === PRBLOCK (Blacklist) ===
            int bl2 = snprintf(line, sizeof(line), "== PRBLOCK ==");
            bool anyBL = false;
            for (int i = 0; i < 16; i++) {
                if (nexus_ch_blacklist[i] != 0 && (int)sizeof(line) - bl2 > 14) {
                    bl2 += snprintf(line + bl2, sizeof(line) - bl2, " %u(0x%X)", (unsigned int)nexus_ch_blacklist[i], (unsigned int)nexus_ch_blacklist[i]);
                    anyBL = true;
                }
            }
            if (!anyBL) bl2 += snprintf(line + bl2, sizeof(line) - bl2, " (empty)");
            bl2 += snprintf(line + bl2, sizeof(line) - bl2, "\n");
            appendPaging(line);

            snprintf(replyMsg, sizeof(replyMsg), "JT: Paging firewall status...");
            // Clean up static scratch buffers so the next call starts fresh.
            memset(line, 0, sizeof(line));
            memset(sec, 0, sizeof(sec));
        }
    }
    // Nexus v66: /nexus FEATURES — toggle feature bits persisted in NexusVault::features_mask
    // (uint64). Four bits today: FORWARD_OPAQUE_TO_PHONE (0), FORWARD_FOREIGN_PKI_TO_PHONE (1),
    // DISABLE_AGC_RESET (2), LATEBASE (3).
    // RAM-only per SAVEVAULT SCOPE RULE — operators persist via /nexus SAVE. Mirrors the FIREWALL
    // command's RAM-only posture (only FEATURES persist).
    else if (strcasecmp(cmd, "FEATURES") == 0 || strncasecmp(cmd, "FEATURES ", 9) == 0) {
        const char *sub = nullptr;
        if (strncasecmp(cmd, "FEATURES ", 9) == 0) sub = cmd + 9;
        while (sub && *sub == ' ') sub++;

        if (!sub || *sub == '\0' || strcasecmp(sub, "LIST") == 0) {
            // FEATURES [LIST]: show current mask + each named bit ON/OFF.
            // Nexus-28006 (2026-09-16) — code review 2A fix.
            // %016llX is unsupported on the Meshtastic newlib-nano printf
            // (see file-header note at line ~20). Split the 64-bit mask into
            // two %08lX halves (high first, low second), mirroring the
            // HEXDROPS / JT GPIO Watch Mask pattern in this file.
            uint64_t m = getNexusFeaturesHex();
            int l = snprintf(replyMsg, sizeof(replyMsg), "JT FEATURES: 0x%08lX%08lX\n",
                             (unsigned long)(m >> 32),
                             (unsigned long)(m & 0xFFFFFFFFu));
            l += snprintf(replyMsg + l, sizeof(replyMsg) - l,
                          "FORWARD_OPAQUE_TO_PHONE:      %s\n",
                          (m & FEATURE_FORWARD_OPAQUE_TO_PHONE) ? "ON" : "off");
            l += snprintf(replyMsg + l, sizeof(replyMsg) - l,
                          "FORWARD_FOREIGN_PKI_TO_PHONE: %s\n",
                          (m & FEATURE_FORWARD_FOREIGN_PKI_TO_PHONE) ? "ON" : "off");
            l += snprintf(replyMsg + l, sizeof(replyMsg) - l,
                          "DISABLE_AGC_RESET:            %s\n",
                          (m & FEATURE_DISABLE_AGC_RESET) ? "ON" : "off");
            l += snprintf(replyMsg + l, sizeof(replyMsg) - l,
                          "LATEBASE:                     %s",
                          (m & FEATURE_LATEBASE) ? "ON" : "off");
        } else if (strcasecmp(sub, "OFF") == 0) {
            // FEATURES OFF: clear all bits.
            setNexusFeaturesHex(0);
            snprintf(replyMsg, sizeof(replyMsg), "JT FEATURES: off (mask=0x0)");
        } else if (strncasecmp(sub, "0x", 2) == 0) {
            // FEATURES 0x<hex>: set mask directly via 0x-prefixed hex.
            char *endp = nullptr;
            unsigned long long v = strtoull(sub + 2, &endp, 16);
            if (endp == sub + 2 || (v == ULLONG_MAX && errno == ERANGE)) {
                snprintf(replyMsg, sizeof(replyMsg), "JT FEATURES: bad hex '%s'", sub);
            } else {
                setNexusFeaturesHex((uint64_t)v);
                uint64_t m = getNexusFeaturesHex();
                // Nexus-28006 (2026-09-16) — code review 2A fix.
                // %016llX is unsupported on the Meshtastic newlib-nano printf.
                // Split into two %08lX halves. See file-header note line ~20.
                snprintf(replyMsg, sizeof(replyMsg), "JT FEATURES: 0x%08lX%08lX",
                         (unsigned long)(m >> 32),
                         (unsigned long)(m & 0xFFFFFFFFu));
            }
        } else {
            // FEATURES <bare-hex-or-name>: bare hex (1-16 hex digits, no spaces)
            // sets the mask; anything else is parsed as a NAME in the next branch.
            bool isBareHex = (strlen(sub) > 0 && strlen(sub) <= 16);
            for (const char *p = sub; *p && isBareHex; p++) {
                if (!((*p >= '0' && *p <= '9') ||
                      (*p >= 'a' && *p <= 'f') ||
                      (*p >= 'A' && *p <= 'F'))) {
                    isBareHex = false;
                }
            }
            if (isBareHex) {
                char *endp = nullptr;
                unsigned long long v = strtoull(sub, &endp, 16);
                if (endp != sub + strlen(sub) || (v == ULLONG_MAX && errno == ERANGE)) {
                    snprintf(replyMsg, sizeof(replyMsg), "JT FEATURES: bad hex '%s'", sub);
                } else {
                    setNexusFeaturesHex((uint64_t)v);
                    uint64_t m = getNexusFeaturesHex();
                    // Nexus-28006 (2026-09-16) — code review 2A fix.
                    // %016llX is unsupported on the Meshtastic newlib-nano printf.
                    // Split into two %08lX halves. See file-header note line ~20.
                    snprintf(replyMsg, sizeof(replyMsg), "JT FEATURES: 0x%08lX%08lX",
                             (unsigned long)(m >> 32),
                             (unsigned long)(m & 0xFFFFFFFFu));
                }
            } else {
                // FEATURES <NAME> [ON|OFF]: toggle named bit. NAME parsed case-insensitive,
                // whitespace-tolerant. Default verb is ON when missing.
                char name[48];
                const char *verb = nullptr;
                const char *sp = strchr(sub, ' ');
                if (sp) {
                    size_t nlen = (size_t)(sp - sub);
                    if (nlen >= sizeof(name)) nlen = sizeof(name) - 1;
                    memcpy(name, sub, nlen);
                    name[nlen] = '\0';
                    verb = sp + 1;
                    while (*verb == ' ') verb++;
                } else {
                    strncpy(name, sub, sizeof(name) - 1);
                    name[sizeof(name) - 1] = '\0';
                }
                uint64_t bit = 0;
                const char *displayName = nullptr;
                if (strcasecmp(name, "FORWARD_OPAQUE_TO_PHONE") == 0 ||
                    strcasecmp(name, "OPAQUE") == 0 || strcasecmp(name, "OPAQ") == 0) {
                    bit = FEATURE_FORWARD_OPAQUE_TO_PHONE;
                    displayName = "FORWARD_OPAQUE_TO_PHONE";
                } else if (strcasecmp(name, "FORWARD_FOREIGN_PKI_TO_PHONE") == 0 ||
                           strcasecmp(name, "FOREIGN_PKI") == 0 || strcasecmp(name, "PKI") == 0) {
                    bit = FEATURE_FORWARD_FOREIGN_PKI_TO_PHONE;
                    displayName = "FORWARD_FOREIGN_PKI_TO_PHONE";
                } else if (strcasecmp(name, "DISABLE_AGC_RESET") == 0 ||
                           strcasecmp(name, "DISABLEAGCRESET") == 0 ||
                           strcasecmp(name, "AGCRESET") == 0 || strcasecmp(name, "AGC") == 0) {
                    bit = FEATURE_DISABLE_AGC_RESET;
                    displayName = "DISABLE_AGC_RESET";
                } else if (strcasecmp(name, "LATEBASE") == 0 ||
                           strcasecmp(name, "LBASE") == 0 || strcasecmp(name, "LB") == 0) {
                    bit = FEATURE_LATEBASE;
                    displayName = "LATEBASE";
                }
                if (bit == 0) {
                    snprintf(replyMsg, sizeof(replyMsg),
                             "JT FEATURES: unknown name '%s' (try FORWARD_OPAQUE_TO_PHONE / FORWARD_FOREIGN_PKI_TO_PHONE / DISABLE_AGC_RESET / LATEBASE)",
                             name);
                } else {
                    bool turnOn = true;
                    if (verb && *verb) {
                        if (strcasecmp(verb, "ON") == 0) turnOn = true;
                        else if (strcasecmp(verb, "OFF") == 0) turnOn = false;
                        else {
                            snprintf(replyMsg, sizeof(replyMsg),
                                     "JT FEATURES: bad verb '%s' (ON|OFF)", verb);
                            return;
                        }
                    }
                    uint64_t m = getNexusFeaturesHex();
                    if (turnOn) m |= bit; else m &= ~bit;
                    setNexusFeaturesHex(m);
                    // Nexus-28006 (2026-09-16) — code review 2A fix.
                    // %016llX is unsupported on the Meshtastic newlib-nano printf.
                    // Split into two %08lX halves (high first, low second).
                    // See file-header note line ~20.
                    snprintf(replyMsg, sizeof(replyMsg), "JT FEATURES: %s %s (mask=0x%08lX%08lX)",
                             displayName, turnOn ? "ON" : "off",
                             (unsigned long)(m >> 32),
                             (unsigned long)(m & 0xFFFFFFFFu));
                }
            }
        }
    }
    // Nexus-28005 (2026-09-12): /nexus LR2021_SF — configure the LR2021
    // hardware's multi-SF side-detector table. RAM-only persistence: stored on
    // the LR2021Interface instance, applied on next radio init via the hook in
    // LR20x0Interface::init(). UX mirrors ZephCore's `config extra.sf <list>`.
    //
    // Forms:
    //   LR2021_SF                    -> show current table
    //   LR2021_SF 0                  -> clear all side detectors (disabled)
    //   LR2021_SF off                -> clear all side detectors (alias)
    //   LR2021_SF <sf> [<sf> [<sf>]] -> set up to 3 side-detector SFs
    //                                  (chip limit). Each SF must be in
    //                                  [5..12], distinct, spread <= 4.
    //
    // Returns a JT-formatted reply listing either the new table, the cleared
    // state, or a validation rejection reason. Operators on non-LR2021 hardware
    // (heltec-v4 with SX1262, etc.) get a "not supported" reply at runtime.
    //
    // Aliases (2026-09-13): MULTISF and MSF — same dispatch, kept short
    // for frequent operators. All six strcasecmp / strncasecmp matches route
    // to the same handler body.
    // Nexus-28005 (2026-09-14): the entire `else if (...)` block — not
    // just its body — is gated on LR2021 hardware. On non-LR2021 builds (e.g.
    // station-g3 with SX1262) the command simply does not exist: a /nexus
    // LR2021_SF falls through to the default `Unknown command` reply, the same
    // behavior as every other variant-gated command in this dispatcher.
    //
    // Previous shape wrapped ONLY the body inside #if/#else/#endif. That left
    // an orphan `}` on non-LR2021 builds, which prematurely closed
    // handleNexusCommand and triggered 115 cascading `expected unqualified-id
    // before 'else'` errors from the next `else if (SETHEX ...)` at line 2304
    // through the final `}` at line 9158. See station-g3 build log
    // /tmp/station-g3-fix.log for the full cascade.
#if (defined(USE_LR2021) || defined(ARCH_PORTDUINO)) && !RADIOLIB_EXCLUDE_LR2021
    else if (strcasecmp(cmd, "LR2021_SF") == 0 || strcasecmp(cmd, "MULTISF") == 0 ||
             strcasecmp(cmd, "MSF") == 0 || strncasecmp(cmd, "LR2021_SF ", 10) == 0 ||
             strncasecmp(cmd, "MULTISF ", 8) == 0 || strncasecmp(cmd, "MSF ", 4) == 0) {
        // Gate on actual hardware. We cast RadioLibInterface::instance to
        // LR2021Interface*; non-LR2021 boards (SX1262/LR1121/etc.) never
        // instantiate an LR2021Interface so the cast would be unsafe there.
        // We can detect that safely at compile time via the existing
        // RADIOLIB_EXCLUDE_LR2021 guard pattern (mirrors the header guard).
        // Nexus-28005 (2026-09-14): widened to also require USE_LR2021
        // or ARCH_PORTDUINO so station-g3 (SX1262, no LR2021 chip) doesn't
        // emit call sites to LR2021Interface::setSideDetectors /
        // configureSideDetectors when the driver .cpp pre-processes to
        // nothing — see src/mesh/LR2021Interface.cpp:3 for the matching
        // symbol-availability gate.
        LR2021Interface *lr2021 = static_cast<LR2021Interface *>(RadioLibInterface::instance);
        const char *sub = skipToken(cmd);
        if (!lr2021) {
            snprintf(replyMsg, sizeof(replyMsg), "JT LR2021_SF: radio interface unavailable");
        } else if (!*sub || strcasecmp(sub, "LIST") == 0) {
            uint8_t count = lr2021->sideDetectorsCount();
            int length = snprintf(replyMsg, sizeof(replyMsg), "JT LR2021_SF: side=%s", count ? "[" : "(empty)");
            for (uint8_t i = 0; i < count; ++i)
                length +=
                    snprintf(replyMsg + length, sizeof(replyMsg) - length, "%sSF%u", i ? "," : "", lr2021->sideDetectors()[i]);
            if (count)
                snprintf(replyMsg + length, sizeof(replyMsg) - length, "] (n=%u)", count);
        } else {
            uint8_t sfs[LR2021Interface::MAX_SF_TABLE] = {};
            uint8_t count = 0;
            if (!nexus::parseSideDetectors(sub, sfs, LR2021Interface::MAX_SF_TABLE, count)) {
                snprintf(replyMsg, sizeof(replyMsg), "JT LR2021_SF: use OFF, 0, or 1-3 distinct SFs [5..12], spread <= 4");
            } else if (lr2021->configureSideDetectors(sfs, count)) {
                lr2021->setSideDetectors(sfs, count);
                snprintf(replyMsg, sizeof(replyMsg),
                         count ? "JT LR2021_SF: stored and applied (n=%u)" : "JT LR2021_SF: cleared and applied (single-SF RX)",
                         count);
            } else {
                snprintf(replyMsg, sizeof(replyMsg), "JT LR2021_SF: chip rejected change; previous stored table retained");
            }
        }
    }
#endif
    else if (strcasecmp(cmd, "DROPS") == 0) {
        int l = snprintf(replyMsg, sizeof(replyMsg), "JT ACTIVE DROPS:\n");

        // Bounded appends: remaining space is checked before each write so a
        // long active-tag list truncates cleanly instead of underflowing the
        // size arithmetic (A12 in the audit; complete tag set incl. TO/NM/RL_*).
        #define APPEND_IF_DROP(name, var) if(var && l >= 0 && (size_t)l < sizeof(replyMsg) - 8) { l += snprintf(replyMsg + l, sizeof(replyMsg) - l, "%s ", name); }
        #define APPEND_IF_INV(name, var) if(!var && l >= 0 && (size_t)l < sizeof(replyMsg) - 8) { l += snprintf(replyMsg + l, sizeof(replyMsg) - l, "%s ", name); }

        APPEND_IF_DROP("NI", drop_nodeinfo);
        APPEND_IF_DROP("PO", drop_position);
        APPEND_IF_DROP("POM", drop_pos_manual);
        APPEND_IF_DROP("POG", drop_pos_gps);
        APPEND_IF_DROP("POO", drop_pos_other);
        APPEND_IF_DROP("TL", drop_telemetry);
        APPEND_IF_DROP("TLD", drop_tele_device);
        APPEND_IF_DROP("TLP", drop_tele_power);
        APPEND_IF_DROP("TLE", drop_tele_env);
        APPEND_IF_DROP("TO", drop_tele_other);
        APPEND_IF_DROP("NB", drop_neighbor);
        APPEND_IF_DROP("TR", drop_traceroute);
        APPEND_IF_DROP("RTR", drop_traceroute_remote);
        APPEND_IF_DROP("LTR", drop_traceroute_local);
        APPEND_IF_DROP("PV", drop_private);
        APPEND_IF_DROP("H6", drop_hop6);
        APPEND_IF_DROP("H7", drop_hop7);
        APPEND_IF_DROP("AK", drop_acks);
        APPEND_IF_DROP("FB", fav_bypass);
        APPEND_IF_DROP("PK", drop_pki);
        APPEND_IF_DROP("PKR", drop_pki_remote);
        APPEND_IF_DROP("PKL", drop_pki_local);
        APPEND_IF_DROP("PH", drop_prehop);
        APPEND_IF_DROP("SF", drop_store_forward);
        APPEND_IF_DROP("AD", drop_admin);
        APPEND_IF_DROP("TX", drop_text);
        APPEND_IF_DROP("KV", drop_keyver);
        APPEND_IF_DROP("AL", drop_alert);
        APPEND_IF_DROP("WP", drop_waypoint);
        APPEND_IF_DROP("TD", drop_traceroute_direct);
        APPEND_IF_DROP("TDR", drop_traceroute_direct_remote);
        APPEND_IF_DROP("TDL", drop_traceroute_direct_local);
        APPEND_IF_DROP("SS", skip_skip_handle);
        APPEND_IF_DROP("NM", drop_no_mqtt);
        APPEND_IF_DROP("RLTX", drop_rl_text);
        APPEND_IF_DROP("RLTR", drop_rl_trace);
        APPEND_IF_DROP("RLOT", drop_rl_other);
        APPEND_IF_DROP("RLNI", drop_rl_nodeinfo);
        APPEND_IF_DROP("RLPO", drop_rl_position);
        APPEND_IF_DROP("RLTL", drop_rl_telemetry);
        APPEND_IF_DROP("RLPR", drop_rl_private);
        if(trace_alert && l >= 0 && (size_t)l < sizeof(replyMsg) - 8) { l += snprintf(replyMsg + l, sizeof(replyMsg) - l, "TA:%d ", trace_alert_mode); }
        if(meta_alert && l >= 0 && (size_t)l < sizeof(replyMsg) - 8) { l += snprintf(replyMsg + l, sizeof(replyMsg) - l, "MA:%d ", meta_alert_mode); }
        APPEND_IF_DROP("RNI", drop_req_nodeinfo);
        APPEND_IF_DROP("RTL", drop_req_telemetry);
        APPEND_IF_DROP("RPO", drop_req_position);
        APPEND_IF_DROP("BRNI", drop_br_nodeinfo);
        APPEND_IF_DROP("BRTL", drop_br_telemetry);
        APPEND_IF_DROP("BRPO", drop_br_position);
        APPEND_IF_DROP("ZH", isZeroHopEnabled());
        APPEND_IF_INV("P_OFF", shouldPreservePosition());
        APPEND_IF_INV("T_OFF", shouldPreserveTelemetry());

        #undef APPEND_IF_DROP
        #undef APPEND_IF_INV

        if (l < 0 || (size_t)l >= sizeof(replyMsg)) l = sizeof(replyMsg) - 1;

        // Dynamic Whitelist Display (Expanded to 16) - Memory Safe
        bool hasWL = false;
        for(int i=0; i<16; i++) if(nexus_ch_whitelist[i] != 0) hasWL = true;
        if (hasWL && (sizeof(replyMsg) - l > 10)) {
            l += snprintf(replyMsg + l, sizeof(replyMsg) - l, "\nWL: ");
            for(int i=0; i<16; i++) {
                if(nexus_ch_whitelist[i] != 0 && (sizeof(replyMsg) - l > 12)) {
                    l += snprintf(replyMsg + l, sizeof(replyMsg) - l, "0x%X ", (unsigned int)nexus_ch_whitelist[i]);
                }
            }
        }

        if (l < 0 || (size_t)l >= sizeof(replyMsg)) l = sizeof(replyMsg) - 1;

        // Dynamic Blacklist Display (Expanded to 16) - Memory Safe
        bool hasBL = false;
        for(int i=0; i<16; i++) if(nexus_ch_blacklist[i] != 0) hasBL = true;
        if (hasBL && (sizeof(replyMsg) - l > 10)) {
            l += snprintf(replyMsg + l, sizeof(replyMsg) - l, "\nBL: ");
            for(int i=0; i<16; i++) {
                if(nexus_ch_blacklist[i] != 0 && (sizeof(replyMsg) - l > 12)) {
                    l += snprintf(replyMsg + l, sizeof(replyMsg) - l, "0x%X ", (unsigned int)nexus_ch_blacklist[i]);
                }
            }
        }
    } else if (strcasecmp(cmd, "OUTDROPS") == 0) {
        int l = snprintf(replyMsg, sizeof(replyMsg), "JT ACTIVE OUTDROPS:\n");
        #define APPEND_IF_OUT(name, var) if(var) { l += snprintf(replyMsg + l, sizeof(replyMsg) - l, "%s ", name); }
        APPEND_IF_OUT("NI", drop_out_nodeinfo);
        APPEND_IF_OUT("PO", drop_out_position);
        APPEND_IF_OUT("TL", drop_out_telemetry);
        APPEND_IF_OUT("TR", drop_out_traceroute);
        APPEND_IF_OUT("AD", drop_out_admin);
        APPEND_IF_OUT("RO", drop_out_routing);
        APPEND_IF_OUT("FB", fav_bypass);
        #undef APPEND_IF_OUT
    } else if (strcasecmp(cmd, "HEXDROPS") == 0) {
        uint64_t dr = getNexusDropHex();
        snprintf(replyMsg, sizeof(replyMsg), "JT HEXDROPS: 0x%08lX%08lX",
                 (unsigned long)(dr >> 32), (unsigned long)(dr & 0xFFFFFFFFu));
    } else if (strcasecmp(cmd, "HEXOUTDROPS") == 0) {
        uint64_t odr = getNexusOutDropHex();
        snprintf(replyMsg, sizeof(replyMsg), "JT HEXOUTDROPS: 0x%08lX%08lX",
                 (unsigned long)(odr >> 32), (unsigned long)(odr & 0xFFFFFFFFu));
    } else if (strncasecmp(cmd, "SETHEX ", 7) == 0) {
        uint64_t h = strtoull(cmd + 7, NULL, 16);
        setNexusDropHex(h);
        snprintf(replyMsg, sizeof(replyMsg), "JT: Drops updated to 0x%08lX%08lX",
                 (unsigned long)(h >> 32), (unsigned long)(h & 0xFFFFFFFFu));
    } else if (strncasecmp(cmd, "SETHEXOUT ", 10) == 0) {
        uint64_t h = strtoull(cmd + 10, NULL, 16);
        setNexusOutDropHex(h);
        snprintf(replyMsg, sizeof(replyMsg), "JT: Output Drops updated to 0x%08lX%08lX",
                 (unsigned long)(h >> 32), (unsigned long)(h & 0xFFFFFFFFu));
    }
    else if (strcasecmp(cmd, "UTIL") == 0) {
        float chUtil = (airTime != NULL) ? airTime->channelUtilizationPercent() : 0.0f;
        float txUtil = (airTime != NULL) ? airTime->utilizationTXPercent() : 0.0f;
        snprintf(replyMsg, sizeof(replyMsg), "JT UTIL:\nChUtil: %.2f%%\nTxUtil: %.2f%%", chUtil, txUtil);
    }
    else if (strcasecmp(cmd, "SYS") == 0) {
        float temp = getDieTemp();
        const char* reason = getResetReason();
        uint32_t freeHeap = memGet.getFreeHeap();
        uint32_t totalHeap = memGet.getHeapSize();

        snprintf(replyMsg, sizeof(replyMsg),
                 "JT SYS:\nDieTemp: %.1fC\nBoot: %s\nHeap: %u/%u",
                 temp, reason, freeHeap, totalHeap);
    }
    else if (strcasecmp(cmd, "SAVE") == 0) {
        snprintf(replyMsg, sizeof(replyMsg),
                 saveVault() ? "JT: Nexus vault saved and verified." : "JT: SAVE failed. Previous vault preserved.");
    }
    else if (strcasecmp(cmd, "LOAD") == 0) {
        snprintf(replyMsg, sizeof(replyMsg),
                 loadVault() ? "JT: Nexus vault loaded and validated."
                             : "JT: LOAD failed. RAM unchanged; check vault CRC, size and version.");
    }
    else if (strncasecmp(cmd, "BD ", 3) == 0 || strcasecmp(cmd, "BD") == 0) {
        // /nexus BD SET <1..300>   -> set broadcast reply / paging max delay (s)
        // /nexus BD GET            -> report current value
        // Floor stays at 100ms; range scales 100..N*1000 ms.
        // NOTE: BD SET no longer persists (public).
        const char *p = cmd + 2;
        while (*p == ' ') p++;
        if (*p == '\0' || strcasecmp(p, "GET") == 0) {
            snprintf(replyMsg, sizeof(replyMsg),
                     "JT BD: max delay = %us (range 100..%ums)",
                     (unsigned)broadcast_delay_max_s,
                     (unsigned)(broadcast_delay_max_s * 1000U));
        } else if (strncasecmp(p, "SET ", 4) == 0) {
            const char *arg = p + 4;
            while (*arg == ' ') arg++;
            char *endp = nullptr;
            long v = strtol(arg, &endp, 10);
            if (endp == arg || *endp != '\0' || v < 1 || v > 300) {
                snprintf(replyMsg, sizeof(replyMsg),
                         "JT BD: invalid value '%s' (expected integer 1..300 seconds)",
                         arg);
            } else {
                broadcast_delay_max_s = (uint16_t)v;
                // saveVault removed (SCOPE RULE: only SAVE persists in tactical)
                snprintf(replyMsg, sizeof(replyMsg),
                         "JT BD: max delay set to %us (jitter 100..%ums). (RAM-only; /nexus SAVE to persist.)",
                         (unsigned)broadcast_delay_max_s,
                         (unsigned)(broadcast_delay_max_s * 1000U));
            }
        } else {
            snprintf(replyMsg, sizeof(replyMsg),
                     "JT BD: usage BD SET <1..300> | BD GET");
        }
    }
    else if (strcasecmp(cmd, "LS") == 0) {
        auto files = getFiles("/", 1);
        startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "File List (LS)");
        static char entry[64];
        for (const auto& f : files) {
            memset(entry, 0, sizeof(entry));
            snprintf(entry, sizeof(entry), "%.24s:%uB\n", f.file_name, (unsigned int)f.size_bytes);
            appendPaging(entry);
        }
        memset(entry, 0, sizeof(entry));
        snprintf(replyMsg, sizeof(replyMsg), "JT: Paging file list to mesh...");
    }
    else if (strncasecmp(cmd, "RM ", 3) == 0) {
        const char *filename = cmd + 3;
        while (*filename == ' ') filename++;

        static char path[64];
        memset(path, 0, sizeof(path));
        if (filename[0] != '/') snprintf(path, sizeof(path), "/%s", filename);
        else strncpy(path, filename, sizeof(path));

        if (FSCom.exists(path)) {
            if (FSCom.remove(path)) {
                snprintf(replyMsg, sizeof(replyMsg), "JT: File '%s' deleted", filename);
            } else {
                snprintf(replyMsg, sizeof(replyMsg), "JT: Failed to delete '%s'", filename);
            }
        } else {
            snprintf(replyMsg, sizeof(replyMsg), "JT: File '%s' not found", filename);
        }
        memset(path, 0, sizeof(path));
    }
    else if (strncasecmp(cmd, "CAT ", 4) == 0) {
        const char *filename = cmd + 4;
        while (*filename == ' ') filename++;

        static char path[64];
        memset(path, 0, sizeof(path));
        if (filename[0] != '/') snprintf(path, sizeof(path), "/%s", filename);
        else strncpy(path, filename, sizeof(path));

        File f = FSCom.open(path, FILE_O_READ);
        if (f) {
            size_t s = f.size();
            if (s > 700) { // Safety limit for 1024B paging buffer (700 * 1.37 < 1024)
                snprintf(replyMsg, sizeof(replyMsg), "JT: File too large (%uB > 700B)", (unsigned int)s);
            } else {
                uint8_t *fBuf = (uint8_t*)malloc(s);
                if (fBuf) {
                    f.read(fBuf, s);
                    char *b64 = (char*)malloc(((s + 2) / 3) * 4 + 1);
                    if (b64) {
                        nexus_base64_encode(b64, fBuf, s);
                        startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "File Download (B64)");
                        appendPaging(b64);
                        free(b64);
                        snprintf(replyMsg, sizeof(replyMsg), "JT: Paging file '%s' (%uB) to mesh...", filename, (unsigned int)s);
                    } else {
                        snprintf(replyMsg, sizeof(replyMsg), "JT: Malloc failed (B64)");
                    }
                    free(fBuf);
                } else {
                    snprintf(replyMsg, sizeof(replyMsg), "JT: Malloc failed (Read)");
                }
            }
            f.close();
        } else {
            snprintf(replyMsg, sizeof(replyMsg), "JT: File '%s' not found", filename);
        }
    }
    else if (strcasecmp(cmd, "FS") == 0) {
#if defined(ARCH_ESP32)
        uint32_t total = FSCom.totalBytes();
        uint32_t used = FSCom.usedBytes();
        snprintf(replyMsg, sizeof(replyMsg), "JT FS:\nUsed: %uB\nTotal: %uB\nFree: %uB",
                 used, total, total - used);
#elif defined(ARCH_NRF52)
        // nRF52 / Adafruit_LittleFS: use the abstracted fsTotalBytes() /
        // fsUsedBytes() helpers from FSCommon.cpp so we get the same semantics
        // as the rest of the firmware (lfs_traverse-based used count, fail-safe
        // "full" on error). The 12 KB warm-store ring at 0xEA000-0xED000 is
        // outside LittleFS and reported separately so operators see the total
        // reserved flash budget for the node.
        uint32_t total = (uint32_t)fsTotalBytes();
        uint32_t used  = (uint32_t)fsUsedBytes();
        uint32_t free  = (total >= used) ? (total - used) : 0;
        snprintf(replyMsg, sizeof(replyMsg),
                 "JT FS (nRF52):\n"
                 "  LittleFS: %uB / %uB (%uB free)\n"
                 "  Warm ring: 12288B reserved (raw flash, 0xEA000-0xED000)",
                 used, total, free);
#else
        snprintf(replyMsg, sizeof(replyMsg), "JT FS: N/A");
#endif
    }
    else if (strcasecmp(cmd, "FAVS") == 0) {
        startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "Favorites");
        for (size_t i = 0; i < nodeDB->getNumMeshNodes(); i++) {
            auto n = nodeDB->getMeshNodeByIndex(i);
            if (n && nexus_is_favorite(n)) {
                char entry[40];
                snprintf(entry, sizeof(entry), "!%08x:%.4s ", (unsigned int)n->num, n->short_name);
                appendPaging(entry);
            }
        }
        snprintf(replyMsg, sizeof(replyMsg), "JT: Paging favorites to mesh...");
    }
    else if (strcasecmp(cmd, "IGNORED") == 0) {
        startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "Ignored");
        for (size_t i = 0; i < nodeDB->getNumMeshNodes(); i++) {
            auto n = nodeDB->getMeshNodeByIndex(i);
            if (n && nexus_is_ignored(n)) {
                char entry[40];
                snprintf(entry, sizeof(entry), "!%08x:%.4s ", (unsigned int)n->num, n->short_name);
                appendPaging(entry);
            }
        }
        snprintf(replyMsg, sizeof(replyMsg), "JT: Paging ignored to mesh...");
    }
    else if (strcasecmp(cmd, "ADMINS") == 0) {
        startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "Admins");
        for (size_t i = 0; i < nodeDB->getNumMeshNodes(); i++) {
            auto n = nodeDB->getMeshNodeByIndex(i);
            if (n && nexus_has_user(n)) {
                bool isAdmin = false;
                for(int j=0; j<3; j++) {
                    if (config.security.admin_key[j].size == 32 && memcmp(n->public_key.bytes, config.security.admin_key[j].bytes, 32) == 0)
                        isAdmin = true;
                }
                if (isAdmin) {
                    char entry[40];
                    snprintf(entry, sizeof(entry), "!%08x:%.4s ", (unsigned int)n->num, n->short_name);
                    appendPaging(entry);
                }
            }
        }
        snprintf(replyMsg, sizeof(replyMsg), "JT: Paging admins to mesh...");
    }
    else if (strcasecmp(cmd, "NEIGHBORS") == 0 || strcasecmp(cmd, "DIRECT") == 0) {
        startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "Neighbors (0-hop)");
        for (size_t i = 0; i < nodeDB->getNumMeshNodes(); i++) {
            auto n = nodeDB->getMeshNodeByIndex(i);
            if (n && n->has_hops_away && n->hops_away == 0) {
                char entry[40];
                snprintf(entry, sizeof(entry), "!%08x:%.4s(%.0f) ", (unsigned int)n->num, n->short_name, n->snr);
                appendPaging(entry);
            }
        }
        snprintf(replyMsg, sizeof(replyMsg), "JT: Paging neighbors to mesh...");
    }
    else if ((strncasecmp(cmd, "NODES", 5) == 0 && (cmd[5] == '\0' || cmd[5] == ' '))
             || strncasecmp(cmd, "FNODE", 5) == 0) {
        bool isFull = (strncasecmp(cmd, "FNODE", 5) == 0);
        const char *searchStr = cmd + 5;
        while (*searchStr == ' ') searchStr++;
        bool hasSearch = (strlen(searchStr) > 0);
        bool longSearch = (strlen(searchStr) > 4);

        if (isFull && !hasSearch) {
            snprintf(replyMsg, sizeof(replyMsg), "JT: Use FNODE <search/!id>");
            return;
        }

        startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId,
                    isFull ? "Node Detail" : "NodeDB");

        // Static buffers (BSS, not stack) to avoid 250B+64B stack frames inside
        // the loop. Both are mutually exclusive per iteration (isFull vs !isFull)
        // so they can share BSS without conflict.
        static char b[250];
        static char entry[64];

        for (size_t i = 0; i < nodeDB->getNumMeshNodes(); i++) {
            auto n = nodeDB->getMeshNodeByIndex(i);
            if (n && nexus_has_user(n)) {
                if (hasSearch) {
                    bool match = false;
                    // Format !id search
                    if (searchStr[0] == '!') {
                        uint32_t sid = strtoul(searchStr + 1, NULL, 16);
                        if (n->num == sid) match = true;
                    } else if (longSearch) {
                        if (nexus_strcasestr(n->long_name, searchStr)) match = true;
                    } else {
                        if (nexus_strcasestr(n->short_name, searchStr) || nexus_strcasestr(n->long_name, searchStr))
                            match = true;
                    }
                    if (!match) continue;
                }

                if (isFull) {
                    memset(b, 0, sizeof(b));
                    // Line 1: Identity
                    int l = snprintf(b, sizeof(b), "!%08x (%s)\nNames: %.32s / %s\n",
                                     (unsigned int)n->num, getRoleName(n->role), n->long_name, n->short_name);

                    // Line 2: HW & Security
                    l += snprintf(b + l, sizeof(b) - l, "HW:%d Lic:%d ", (int)n->hw_model, (int)nexus_is_licensed(n));
                    if (n->public_key.size == 32) {
                        l += snprintf(b + l, sizeof(b) - l, "Key:%02x%02x..%02x%02x(%s)\n",
                                     n->public_key.bytes[0], n->public_key.bytes[1],
                                     n->public_key.bytes[30], n->public_key.bytes[31],
                                     (n->bitfield & 0x01) ? "KV" : "K+");
                    } else {
                        l += snprintf(b + l, sizeof(b) - l, "Key:NONE\n");
                    }

                    // Line 3: Position
                    if (nexus_has_position(n)) {
                        l += snprintf(b + l, sizeof(b) - l, "Pos: %.4f,%.4f Alt:%dm\n",
                                     nexus_pos(n)->latitude_i / 1e7, nexus_pos(n)->longitude_i / 1e7, nexus_pos(n)->altitude);
                    }

                    // Line 4: Metrics & SIGINT
                    uint32_t ago = (getTime(false) > n->last_heard) ? (getTime(false) - n->last_heard) : 0;
                    l += snprintf(b + l, sizeof(b) - l, "SNR:%.1f Hops:%d via *%02x\n",
                                 n->snr, n->has_hops_away ? n->hops_away : -1, n->next_hop);

                    if (nexus_has_device_metrics(n)) {
                        l += snprintf(b + l, sizeof(b) - l, "Met: %.2fV(%d%%) Up:%ds\n",
                                     nexus_devmetrics(n)->voltage, nexus_devmetrics(n)->battery_level, nexus_devmetrics(n)->uptime_seconds);
                    }

                    l += snprintf(b + l, sizeof(b) - l, "Heard: %us ago (%s)\n---\n", ago, nexus_via_mqtt(n) ? "MQTT" : "LoRa");
                    appendPaging(b);
                } else {
                    // Standard compact view (per previous enhancement)
                    int hops = n->has_hops_away ? n->hops_away : -1;
                    const char* kStat = "K-";
                    if (n->public_key.size == 32) {
                        bool allZero = true;
                        for(int j=0; j<32; j++) { if(n->public_key.bytes[j] != 0) { allZero = false; break; } }
                        if (!allZero) kStat = (n->bitfield & 0x01) ? "KV" : "K+";
                    }
                    memset(entry, 0, sizeof(entry));
                    snprintf(entry, sizeof(entry), "!%08x:%s:%d:%.0f:%s ",
                             (unsigned int)n->num, n->short_name, hops, n->snr, kStat);
                    appendPaging(entry);
                }
            }
        }
        memset(b, 0, sizeof(b));
        memset(entry, 0, sizeof(entry));
        snprintf(replyMsg, sizeof(replyMsg), "JT: %s results paged...", isFull ? "Detail" : "NodeDB");
    }
    else if (strncasecmp(cmd, "MQTT ", 5) == 0 || strcasecmp(cmd, "MQTT") == 0) {
        const char *p = (strlen(cmd) > 5) ? cmd + 5 : "";
        while (*p == ' ') p++;

        if (*p == '\0') {
            const char* modes[] = {"DEFAULT", "ALWAYS-UPLOAD", "STRICT-PRIVACY"};
            const char* status = (moduleConfig.mqtt.enabled) ? "ENABLED" : "DISABLED";
            const char* conn = (mqtt && mqtt->isConnectedDirectly()) ? "CONNECTED" : (mqtt && moduleConfig.mqtt.proxy_to_client_enabled ? "PROXY" : "DISCONNECTED");

            snprintf(replyMsg, sizeof(replyMsg), "JT MQTT [%s]:\nStatus: %s\nMode: %s\nSrv: %s\nMap: %s",
                     conn, status,
                     mqtt_override_mode <= 2 ? modes[mqtt_override_mode] : "UNK",
                     moduleConfig.mqtt.address,
                     moduleConfig.mqtt.map_reporting_enabled ? "ON" : "OFF");
        } else if (strcasecmp(p, "DEFAULT") == 0 || strcasecmp(p, "0") == 0) {
            mqtt_override_mode = 0;
            snprintf(replyMsg, sizeof(replyMsg), "JT: MQTT Override set to DEFAULT.");
        } else if (strcasecmp(p, "ALWAYS") == 0 || strcasecmp(p, "1") == 0 || strcasecmp(p, "ALWAYS-UPLOAD") == 0) {
            mqtt_override_mode = 1;
            snprintf(replyMsg, sizeof(replyMsg), "JT: MQTT Override set to ALWAYS-UPLOAD.");
        } else if (strcasecmp(p, "STRICT") == 0 || strcasecmp(p, "2") == 0 || strcasecmp(p, "STRICT-PRIVACY") == 0) {
            mqtt_override_mode = 2;
            snprintf(replyMsg, sizeof(replyMsg), "JT: MQTT Override set to STRICT-PRIVACY.");
        } else {
             snprintf(replyMsg, sizeof(replyMsg), "JT: Invalid MQTT Mode (DEFAULT|ALWAYS|STRICT)");
        }
    }
    else if (strncasecmp(cmd, "INALM ", 6) == 0 || strcasecmp(cmd, "INALM") == 0) {
        const char *p = (strlen(cmd) > 6) ? cmd + 6 : "";
        while (*p == ' ') p++;

        if (*p == '\0') {
            snprintf(replyMsg, sizeof(replyMsg), "JT INA Legacy Math: %s", ina_legacy_math ? "ON" : "OFF");
        } else {
            bool enable = (strcasecmp(p, "ON") == 0 || strcasecmp(p, "1") == 0);
            ina_legacy_math = enable;
            snprintf(replyMsg, sizeof(replyMsg), "JT: INA Legacy Math set to %s.", enable ? "ON" : "OFF");
        }
    }
    else if (strncasecmp(cmd, "DU ", 3) == 0 || strncasecmp(cmd, "DECODEURL ", 10) == 0 ||
             strncasecmp(cmd, "ADDURL ", 7) == 0) {
        bool addNode = strncasecmp(cmd, "ADDURL ", 7) == 0;
        const char *url;
        if (addNode) url = cmd + 7;
        else url = (strncasecmp(cmd, "DU ", 3) == 0) ? cmd + 3 : cmd + 10;

        while (*url == ' ') url++;

        const char *hashPart = strchr(url, '#');
        if (hashPart) {
            url = hashPart + 1;
        }

        size_t dlen = 0;
        // Static (BSS, not stack). DU/ADDURL runs in handleNexusCommand,
        // 512 B on stack is OK on ESP32 (8 KB loopTask) but tight on nRF52
        // (4 KB loopTask). The base64 payload for a SharedContact protobuf
        // is bounded by the URL parser anyway, so this never re-enters.
        static unsigned char buffer[512];
        memset(buffer, 0, sizeof(buffer));
        if (nexus_base64_decode(buffer, sizeof(buffer), &dlen, url) == 0) {
            meshtastic_SharedContact sc = meshtastic_SharedContact_init_default;
            if (pb_decode_from_bytes(buffer, dlen, &meshtastic_SharedContact_msg, &sc)) {
                if (addNode) {
                    nodeDB->addFromContact(sc);
                    nodeDB->saveToDisk(SEGMENT_NODEDATABASE);
                    snprintf(replyMsg, sizeof(replyMsg), "JT: Added node !%08x (%s) from URL",
                             (unsigned int)sc.node_num, sc.user.short_name);
                } else {
                    snprintf(replyMsg, sizeof(replyMsg), "JT DU !%08x:\nLName: %.32s\nSName: %.8s\nHW: %d Role: %s\nPK: %02x%02x..%02x%02x",
                             (unsigned int)sc.node_num, sc.user.long_name, sc.user.short_name,
                             (int)sc.user.hw_model, getRoleName(sc.user.role),
                             sc.user.public_key.bytes[0], sc.user.public_key.bytes[1],
                             sc.user.public_key.bytes[30], sc.user.public_key.bytes[31]);
                }
            } else {
                snprintf(replyMsg, sizeof(replyMsg), "JT: %s Protobuf Decode Failed.", addNode ? "ADDURL" : "DU");
            }
        } else {
            snprintf(replyMsg, sizeof(replyMsg), "JT: %s Base64 Decode Failed.", addNode ? "ADDURL" : "DU");
        }
    }
    else if (strncasecmp(cmd, "RDROP ", 6) == 0 || strncasecmp(cmd, "RUNDROP ", 8) == 0 || strcasecmp(cmd, "RDROP") == 0) {
        bool setDrop = (strncasecmp(cmd, "RDROP ", 6) == 0);
        const char *p = (setDrop) ? cmd + 6 : cmd + 8;
        if (strcasecmp(cmd, "RDROP") == 0) p = "LIST"; // Handle bare command
        while (*p == ' ') p++;

        if (strcasecmp(p, "CLEAR") == 0) {
            memset(relay_drop_mask, 0, sizeof(relay_drop_mask));
            snprintf(replyMsg, sizeof(replyMsg), "JT: Relay Drop Mask CLEARED.");
        } else if (strcasecmp(p, "LIST") == 0) {
            memset(cmdBuf.lBuf, 0, sizeof(cmdBuf.lBuf));
            int lLen = snprintf(cmdBuf.lBuf, sizeof(cmdBuf.lBuf), "JT RDROP: ");
            for(int i=0; i<256; i++) {
                if (isRelayDropped(i)) {
                    if (lLen < (int)sizeof(cmdBuf.lBuf) - 10) {
                        lLen += snprintf(cmdBuf.lBuf + lLen, sizeof(cmdBuf.lBuf) - lLen, "*%02X ", i);
                    }
                }
            }
            snprintf(replyMsg, sizeof(replyMsg), "%s", cmdBuf.lBuf);
            memset(cmdBuf.lBuf, 0, sizeof(cmdBuf.lBuf));
        } else {
            if (*p == '*') p++; // accept the "*A4" display form
            uint8_t id = (uint8_t)strtoul(p, NULL, 16);
            setRelayDropped(id, setDrop);
            snprintf(replyMsg, sizeof(replyMsg), "JT: Relay *%02X %s", id, setDrop ? "DROPPED" : "ALLOWED");
        }
    }
    else if (strncasecmp(cmd, "IDR ", 4) == 0 || strncasecmp(cmd, "IGNORE-DUPE-RELAY ", 18) == 0 || strcasecmp(cmd, "IDR") == 0) {
        int off = (strncasecmp(cmd, "IDR ", 4) == 0) ? 4 : 18;
        if (strcasecmp(cmd, "IDR") == 0) off = 3;
        const char *p = cmd + off;
        if (strcasecmp(cmd, "IDR") == 0) p = "LIST";
        while (*p == ' ') p++;

        if (strcasecmp(p, "CLEAR") == 0) {
            memset(ignore_dupe_relay_mask, 0, sizeof(ignore_dupe_relay_mask));
            snprintf(replyMsg, sizeof(replyMsg), "JT: IDR Mask CLEARED.");
        } else if (strcasecmp(p, "LIST") == 0) {
            memset(cmdBuf.lBuf, 0, sizeof(cmdBuf.lBuf));
            int lLen = snprintf(cmdBuf.lBuf, sizeof(cmdBuf.lBuf), "JT IDR: ");
            for(int i=0; i<256; i++) {
                if (isIgnoreDupeRelay(i)) {
                    if (lLen < (int)sizeof(cmdBuf.lBuf) - 10) {
                        lLen += snprintf(cmdBuf.lBuf + lLen, sizeof(cmdBuf.lBuf) - lLen, "*%02X ", i);
                    }
                }
            }
            snprintf(replyMsg, sizeof(replyMsg), "%s", cmdBuf.lBuf);
            memset(cmdBuf.lBuf, 0, sizeof(cmdBuf.lBuf));
        } else {
            if (*p == '*') p++; // accept the "*A4" display form
            uint8_t id = (uint8_t)strtoul(p, NULL, 16);
            setIgnoreDupeRelay(id, true);
            snprintf(replyMsg, sizeof(replyMsg), "JT: Relay *%02X added to IDR", id);
        }
    }
    else if (strncasecmp(cmd, "UNIDR ", 6) == 0) {
        const char *p = cmd + 6;
        while (*p == ' ') p++;
        if (*p == '*') p++; // accept the "*A4" display form
        uint8_t id = (uint8_t)strtoul(p, NULL, 16);
        setIgnoreDupeRelay(id, false);
        snprintf(replyMsg, sizeof(replyMsg), "JT: Relay *%02X removed from IDR", id);
    }
    else if (strcasecmp(cmd, "CONFIG") == 0) {
        // Return key configuration values
        uint32_t niInt = Default::getConfiguredOrDefaultMs(config.device.node_info_broadcast_secs, default_node_info_broadcast_secs) / 1000;
        uint32_t telInt = Default::getConfiguredOrDefaultMs(moduleConfig.telemetry.device_update_interval, default_telemetry_broadcast_interval_secs) / 1000;
        uint32_t posInt = Default::getConfiguredOrDefaultMs(config.position.position_broadcast_secs, default_broadcast_interval_secs) / 1000;
        uint32_t envInt = Default::getConfiguredOrDefaultMs(moduleConfig.telemetry.environment_update_interval, default_telemetry_broadcast_interval_secs) / 1000;
        uint32_t powInt = Default::getConfiguredOrDefaultMs(moduleConfig.telemetry.power_update_interval, default_telemetry_broadcast_interval_secs) / 1000;

        auto ourNode = nodeDB->getMeshNode(nodeDB->getNodeNum());
        static char posBuf[64];
        memset(posBuf, 0, sizeof(posBuf));
        snprintf(posBuf, sizeof(posBuf), "None");
        if (ourNode && nexus_has_position(ourNode)) {
            snprintf(posBuf, sizeof(posBuf), "%.5f,%.5f", nexus_pos(ourNode)->latitude_i * 1e-7, nexus_pos(ourNode)->longitude_i * 1e-7);
        }

        snprintf(replyMsg, sizeof(replyMsg),
                 "JT CONFIG:\nRole: %s\nNI:%us\nTEL: D:%us E:%us P:%us\nPOS:%us(%d) Smart:%d\nGPS:%d Fixed:%d HM:%d RM:%d\nLoc: %s",
                 getRoleName(config.device.role),
                 niInt, telInt, envInt, powInt,
                 posInt, (int)config.position.position_flags, (int)config.position.position_broadcast_smart_enabled,
                 config.position.gps_enabled, config.position.fixed_position, hop_mask_active, (M_S != -1),
                 posBuf);
        memset(posBuf, 0, sizeof(posBuf));
    }
    else if (strncasecmp(cmd, "SETCONFIG ", 10) == 0 || strcasecmp(cmd, "SETCONFIG") == 0) {
        const char *p = skipToken(cmd);
        const char *valStr = skipToken(p);

        bool valid = true;
        bool setBase = false;
        bool setModule = false;

        meshtastic_Config pc = meshtastic_Config_init_default;
        meshtastic_ModuleConfig mc = meshtastic_ModuleConfig_init_default;

        if (strncasecmp(p, "NI", 2) == 0) {
            pc.which_payload_variant = meshtastic_Config_device_tag;
            pc.payload_variant.device = config.device;
            pc.payload_variant.device.node_info_broadcast_secs = atoi(valStr);
            setBase = true;
        }
        else if (strncasecmp(p, "TEL_D", 5) == 0) {
            mc.which_payload_variant = meshtastic_ModuleConfig_telemetry_tag;
            mc.payload_variant.telemetry = moduleConfig.telemetry;
            mc.payload_variant.telemetry.device_update_interval = atoi(valStr);
            setModule = true;
        }
        else if (strncasecmp(p, "TEL_E", 5) == 0) {
            mc.which_payload_variant = meshtastic_ModuleConfig_telemetry_tag;
            mc.payload_variant.telemetry = moduleConfig.telemetry;
            mc.payload_variant.telemetry.environment_update_interval = atoi(valStr);
            setModule = true;
        }
        else if (strncasecmp(p, "TEL_P", 5) == 0) {
            mc.which_payload_variant = meshtastic_ModuleConfig_telemetry_tag;
            mc.payload_variant.telemetry = moduleConfig.telemetry;
            mc.payload_variant.telemetry.power_update_interval = atoi(valStr);
            setModule = true;
        }
        else if (strncasecmp(p, "POS", 3) == 0) {
            pc.which_payload_variant = meshtastic_Config_position_tag;
            pc.payload_variant.position = config.position;
            pc.payload_variant.position.position_broadcast_secs = atoi(valStr);
            setBase = true;
        }
        else if (strncasecmp(p, "SMART", 5) == 0) {
            pc.which_payload_variant = meshtastic_Config_position_tag;
            pc.payload_variant.position = config.position;
            pc.payload_variant.position.position_broadcast_smart_enabled = (strcasecmp(valStr, "ON") == 0);
            setBase = true;
        }
        else if (strncasecmp(p, "FIXED", 5) == 0) {
            pc.which_payload_variant = meshtastic_Config_position_tag;
            pc.payload_variant.position = config.position;
            pc.payload_variant.position.fixed_position = (strcasecmp(valStr, "ON") == 0);
            setBase = true;
        }
        else if (strncasecmp(p, "GPS", 3) == 0) {
            pc.which_payload_variant = meshtastic_Config_position_tag;
            pc.payload_variant.position = config.position;
            pc.payload_variant.position.gps_enabled = (strcasecmp(valStr, "ON") == 0);
            setBase = true;
        }
        else if (strncasecmp(p, "LOC", 3) == 0) {
            const char *comma = strchr(valStr, ',');
            if (comma) {
                float lat = atof(valStr);
                float lon = atof(comma + 1);
                if (adminModule) {
                    meshtastic_AdminMessage am = meshtastic_AdminMessage_init_default;
                    am.which_payload_variant = meshtastic_AdminMessage_set_fixed_position_tag;
                    am.set_fixed_position.latitude_i = (int32_t)(lat * 1e7);
                    am.set_fixed_position.longitude_i = (int32_t)(lon * 1e7);
                    am.set_fixed_position.time = getTime();

                    meshtastic_MeshPacket dummyPacket = meshtastic_MeshPacket_init_default;
                    adminModule->handleReceivedProtobuf(dummyPacket, &am);

                    snprintf(replyMsg, sizeof(replyMsg), "JT: Manual location set to %.5f,%.5f", lat, lon);
                    valid = true;
                } else valid = false;
            } else {
                valid = false;
            }
        }
        else {
            valid = false;
        }

        if (valid) {
            if (setBase && adminModule) adminModule->handleSetConfig(pc, false);
            if (setModule && adminModule) adminModule->handleSetModuleConfig(mc);
            if (replyMsg[0] == '\0') {
                snprintf(replyMsg, sizeof(replyMsg), "JT: Config updated.");
            }
        } else {
            snprintf(replyMsg, sizeof(replyMsg), "JT SETCONFIG: NI|TEL_D|TEL_E|TEL_P|POS|SMART|FIXED|GPS <val> | LOC lat,lon");
        }
    }
    else if (strncasecmp(cmd, "SHARE ", 6) == 0) {
        const char *target = cmd + 6;
        uint32_t nodeID = findNodeByShortNameOrHex(target);
        if (nodeID == 0) {
            snprintf(replyMsg, sizeof(replyMsg), "JT: Share Error - Node '%s' not found.", target);
        } else {
            auto node = nodeDB->getMeshNode(nodeID);
            if (node) {
                // Construct SharedContact for serialization (Minimal)
                meshtastic_SharedContact sc = meshtastic_SharedContact_init_default;
                sc.node_num = node->num;
                sc.has_user = true;

                // Copy user details manually from Lite to standard User protobuf
                memset(&sc.user, 0, sizeof(sc.user));
                snprintf(sc.user.id, sizeof(sc.user.id), "!%08x", (unsigned int)node->num);
                strncpy(sc.user.long_name, node->long_name, sizeof(sc.user.long_name));
                strncpy(sc.user.short_name, node->short_name, sizeof(sc.user.short_name));
                sc.user.hw_model = node->hw_model;
                sc.user.role = node->role;
                sc.user.public_key.size = node->public_key.size;
                memcpy(sc.user.public_key.bytes, node->public_key.bytes, node->public_key.size);

                // Static (BSS) so 256B+400B don't stack-frame this handler.
                static uint8_t buffer[256];
                memset(buffer, 0, sizeof(buffer));
                memset(cmdBuf.b64, 0, sizeof(cmdBuf.b64));
                pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
                if (pb_encode(&stream, meshtastic_SharedContact_fields, &sc)) {
                    nexus_base64_encode(cmdBuf.b64, buffer, stream.bytes_written, true);

                    snprintf(replyMsg, sizeof(replyMsg), "JT SHARE (%s): https://meshtastic.org/v/#%s",
                             node->short_name[0] != '\0' ? node->short_name : target, cmdBuf.b64);
                } else {
                    snprintf(replyMsg, sizeof(replyMsg), "JT: Encode failed.");
                }
                memset(buffer, 0, sizeof(buffer));
                memset(cmdBuf.b64, 0, sizeof(cmdBuf.b64));
            } else {
                snprintf(replyMsg, sizeof(replyMsg), "JT: Internal NodeDB error.");
            }
        }
    }
    else if (strcasecmp(cmd, "INFRA") == 0) {
        size_t total = nodeDB->getNumMeshNodes();
        int infraCount = 0;
        startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "JT Infrastructure");

        static char line[64];
        for (size_t i = 0; i < total; i++) {
            auto n = nodeDB->getMeshNodeByIndex(i);
            if (n && nexus_has_user(n)) {
                int r = n->role;
                bool isInfra = (r == meshtastic_Config_DeviceConfig_Role_ROUTER ||
                                r == meshtastic_Config_DeviceConfig_Role_ROUTER_CLIENT ||
                                r == meshtastic_Config_DeviceConfig_Role_ROUTER_LATE ||
                                r == meshtastic_Config_DeviceConfig_Role_REPEATER);

                if (isInfra) {
                    infraCount++;
                    int hops = n->has_hops_away ? n->hops_away : -1;
                    memset(line, 0, sizeof(line));
                    snprintf(line, sizeof(line), "!%08x:%s:%s:H%d\n",
                             (unsigned int)n->num, n->short_name, getRoleName(r), hops);
                    appendPaging(line);
                }
            }
        }
        memset(line, 0, sizeof(line));

        if (infraCount == 0) {
            appendPaging("No infrastructure nodes found in NodeDB.\n");
        }

        snprintf(replyMsg, sizeof(replyMsg), "JT: Paging infrastructure (%d nodes)...", infraCount);
    }
    else if (strcasecmp(cmd, "DBINFO") == 0) {
        size_t total = nodeDB->getNumMeshNodes();
        size_t online = nodeDB->getNumOnlineMeshNodes();
        int direct = 0;
        int favs = 0;

        for (size_t i = 0; i < total; i++) {
            auto n = nodeDB->getMeshNodeByIndex(i);
            if (n) {
                if (n->has_hops_away && n->hops_away == 0) direct++;
                if (nexus_is_favorite(n)) favs++;
            }
        }

        snprintf(replyMsg, sizeof(replyMsg),
                 "JT DBINFO:\nNodes: %u/%d (%u%%)\nOnline: %u\nDirect: %d\nFavs: %d",
                 (unsigned int)total, MAX_NUM_NODES, (unsigned int)(total * 100 / MAX_NUM_NODES),
                 (unsigned int)online, direct, favs);
    }
    else if (strcasecmp(cmd, "MODULES") == 0 || strcasecmp(cmd, "MOD") == 0) {
        // List all loaded MeshModules, with their registered name. Paged because
        // a fully-loaded build has ~35 modules and a single mesh packet holds
        // only NEXUS_MAX_REPLY bytes per reply.
        const std::vector<MeshModule *> *all = MeshModule::getAllModules();
        if (!all || all->empty()) {
            snprintf(replyMsg, sizeof(replyMsg), "JT MODULES: none loaded");
        } else {
            size_t n = all->size();
            startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "Loaded Modules");
            for (size_t i = 0; i < n; i++) {
                MeshModule *m = (*all)[i];
                const char *nm = m ? m->getName() : "?";
                char entry[64];
                snprintf(entry, sizeof(entry), "[%02u/%02u] %s\n", (unsigned)(i + 1), (unsigned)n, nm);
                appendPaging(entry);
            }
            snprintf(replyMsg, sizeof(replyMsg), "JT: Paging %u loaded modules...", (unsigned)n);
        }
    }
    else if (strcasecmp(cmd, "TELEMETRY") == 0) {
        char metricsBuf[NEXUS_MAX_REPLY] = {0};
        int len = 0;

        // 1. Live Radio Metrics
        float liveChUtil = (airTime != NULL) ? airTime->channelUtilizationPercent() : 0.0f;
        float liveTxUtil = (airTime != NULL) ? airTime->utilizationTXPercent() : 0.0f;
        len += snprintf(metricsBuf + len, sizeof(metricsBuf) - len, "ChU:%.1f%% TxU:%.1f%%\n", liveChUtil, liveTxUtil);

        // 2. Device Metrics (battery / voltage / channel util / air util / uptime).
        // Read directly from the live DeviceTelemetryModule instead of the NodeDB
        // satellite map. The map (nodeDB->nodeTelemetry) is gated by
        // MESHTASTIC_EXCLUDE_TELEMETRYDB and is empty on memory-constrained variants
        // (promicro, rak3401, heltec_v3), so the previous shim-based lookup lost the
        // "DB:" line on those targets even though DeviceTelemetryModule was alive and
        // collecting data. getNexusDeviceTelemetry() reads the live snapshot directly.
        // Nexus-28006 (2026-09-17) — TELEMETRY stack-overflow fix.
        // Bug report 2026-09-17 19:10 UTC: /nexus R CH Channel
        // TELEMETRY crashed the node (reboot). Root cause: the three
        // meshtastic_Telemetry locals (dm/pm/em, ~300-500 bytes each,
        // protobuf-generated struct with device + environment + power +
        // health + air quality variants) plus the 200-byte metricsBuf
        // pushed the TELEMETRY handler's stack frame to ~1.5 KB on nRF52's
        // 4 KB loopTask. When called via REDIRECT recursion (depth 0→1
        // adds another meshtastic_MeshPacket mock = ~200 bytes to the
        // outer frame), the recursion's own TELEMETRY handler invocation
        // pushed the cumulative stack usage close to the loopTask limit.
        // In the VALIDATION data (bug report 2026-09-17 19:10 UTC),
        // the crash manifests as a node reboot (panic + reboot
        // path), which matches a stack overflow → FreeRTOS configASSERT
        // failure → ESP-IDF / nRF52 hard fault → bootloader reset.
        //
        // Fix: convert the three meshtastic_Telemetry locals from stack
        // to static (BSS). Mirrors the existing pattern at
        // NexusModule.cpp:636 and :1725 (`static meshtastic_Telemetry
        // tele;`). Static allocation is safe here because the TELEMETRY
        // handler is single-threaded (FreeRTOS task); the static buffers
        // are not shared with concurrent callers. Saves ~1-1.5 KB of
        // stack frame per TELEMETRY invocation — the recursion path is
        // now well within the 4 KB loopTask budget.
        static meshtastic_Telemetry dm = meshtastic_Telemetry_init_zero;
        if (meshtastic::getNexusDeviceTelemetry(&dm) && dm.which_variant == meshtastic_Telemetry_device_metrics_tag) {
            const auto &d = dm.variant.device_metrics;
            int vLen = snprintf(metricsBuf + len, sizeof(metricsBuf) - len, "DB:%.2fV(%u%%)",
                                d.has_voltage ? d.voltage : 0.0f, (uint32_t)d.battery_level);
            if (vLen > 0 && (size_t)vLen < (sizeof(metricsBuf) - len)) len += vLen;
            int nlLen = snprintf(metricsBuf + len, sizeof(metricsBuf) - len, "\n");
            if (nlLen > 0 && (size_t)nlLen < (sizeof(metricsBuf) - len)) len += nlLen;
        }

        // 3. Real-time Power (INA/MAX Hardware)
        // See the static-storage tombstone above the `dm` declaration for
        // why this is static instead of stack-allocated. Same pattern,
        // same Nexus-28006 stack-overflow fix (2026-09-17 19:10 UTC).
        static meshtastic_Telemetry pm = meshtastic_Telemetry_init_zero;
        if (meshtastic::getNexusPowerTelemetry(&pm) && pm.which_variant == meshtastic_Telemetry_power_metrics_tag) {
            auto &p = pm.variant.power_metrics;
            if (p.has_ch1_voltage) {
                int lTmp = snprintf(metricsBuf + len, sizeof(metricsBuf) - len, "V1:%.2fV ", p.ch1_voltage);
                if (lTmp > 0 && (size_t)lTmp < (sizeof(metricsBuf) - len)) len += lTmp;
            }
            if (p.has_ch1_current) {
                int lTmp = snprintf(metricsBuf + len, sizeof(metricsBuf) - len, "I1:%.0fmA\n", p.ch1_current);
                if (lTmp > 0 && (size_t)lTmp < (sizeof(metricsBuf) - len)) len += lTmp;
            }
            if (p.has_ch2_voltage) {
                int lTmp = snprintf(metricsBuf + len, sizeof(metricsBuf) - len, "V2:%.2fV ", p.ch2_voltage);
                if (lTmp > 0 && (size_t)lTmp < (sizeof(metricsBuf) - len)) len += lTmp;
            }
            if (p.has_ch2_current) {
                int lTmp = snprintf(metricsBuf + len, sizeof(metricsBuf) - len, "I2:%.0fmA\n", p.ch2_current);
                if (lTmp > 0 && (size_t)lTmp < (sizeof(metricsBuf) - len)) len += lTmp;
            }
            if (p.has_ch3_voltage) {
                int lTmp = snprintf(metricsBuf + len, sizeof(metricsBuf) - len, "V3:%.2fV ", p.ch3_voltage);
                if (lTmp > 0 && (size_t)lTmp < (sizeof(metricsBuf) - len)) len += lTmp;
            }
            if (p.has_ch3_current) {
                int lTmp = snprintf(metricsBuf + len, sizeof(metricsBuf) - len, "I3:%.0fmA\n", p.ch3_current);
                if (lTmp > 0 && (size_t)lTmp < (sizeof(metricsBuf) - len)) len += lTmp;
            }
        }

        // 4. Real-time Environment (BME/BMP/LTR Hardware)
        // See the static-storage tombstone above the `dm` declaration for
        // why this is static instead of stack-allocated. Same pattern,
        // same Nexus-28006 stack-overflow fix (2026-09-17 19:10 UTC).
        // Note: there is a SEPARATE `em` declaration in the /nexus ENV
        // handler below — that one stays stack-allocated because the
        // bug report was about TELEMETRY specifically. Don't convert
        // the ENV one's `em` without auditing that handler first.
        static meshtastic_Telemetry em = meshtastic_Telemetry_init_zero;
        if (meshtastic::getNexusEnvironmentTelemetry(&em) && em.which_variant == meshtastic_Telemetry_environment_metrics_tag) {
            auto &e = em.variant.environment_metrics;
            len += snprintf(metricsBuf + len, sizeof(metricsBuf) - len, "T:%.1fC H:%.0f%% P:%.2f\n",
                            e.temperature, e.relative_humidity, e.barometric_pressure);
            if (e.has_lux) {
                len += snprintf(metricsBuf + len, sizeof(metricsBuf) - len, "Lux:%.0f ", e.lux);
            }
            if (e.has_uv_lux) {
                len += snprintf(metricsBuf + len, sizeof(metricsBuf) - len, "UV:%.0f", e.uv_lux);
            }
        }

        snprintf(replyMsg, sizeof(replyMsg), "JT TELEMETRY:\n%s", metricsBuf);
    }
    else if (strcasecmp(cmd, "ENV") == 0) {
        meshtastic_Telemetry em = meshtastic_Telemetry_init_zero;
        if (meshtastic::getNexusEnvironmentTelemetry(&em) && em.which_variant == meshtastic_Telemetry_environment_metrics_tag) {
            auto &e = em.variant.environment_metrics;
            memset(cmdBuf.envBuf, 0, sizeof(cmdBuf.envBuf));
            int eLen = 0;
            if (e.has_temperature) eLen += snprintf(cmdBuf.envBuf + eLen, sizeof(cmdBuf.envBuf) - eLen, "Temp:%.1fC ", e.temperature);
            if (e.has_relative_humidity) eLen += snprintf(cmdBuf.envBuf + eLen, sizeof(cmdBuf.envBuf) - eLen, "Hum:%.0f%% ", e.relative_humidity);
            if (e.has_barometric_pressure) eLen += snprintf(cmdBuf.envBuf + eLen, sizeof(cmdBuf.envBuf) - eLen, "Pres:%.2fhPa\n", e.barometric_pressure); // Nexus-28006: 2 decimals (was %.0f), matching TELEMETRY cc5ec4be2. BME/BMP/LPS22HB sensors deliver float values with sub-hPa resolution; integer hPa was losing meaningful precision for weather-trend tracking and altitude calibration. envBuf is 200B so the extra ~3 chars worst-case fits comfortably.
            if (e.has_lux) eLen += snprintf(cmdBuf.envBuf + eLen, sizeof(cmdBuf.envBuf) - eLen, "Lux:%.0f ", e.lux);
            if (e.has_uv_lux) eLen += snprintf(cmdBuf.envBuf + eLen, sizeof(cmdBuf.envBuf) - eLen, "UV Index:%.0f", e.uv_lux);

            snprintf(replyMsg, sizeof(replyMsg), "JT ENV:\n%s", cmdBuf.envBuf);
            memset(cmdBuf.envBuf, 0, sizeof(cmdBuf.envBuf));
        } else {
            snprintf(replyMsg, sizeof(replyMsg), "JT ENV: No sensor data available");
        }
    }
    else if (strcasecmp(cmd, "POWER") == 0) {
        meshtastic_Telemetry m = meshtastic_Telemetry_init_zero;
        bool hasLive = meshtastic::getNexusPowerTelemetry(&m);

        float v1 = 0, i1 = 0, v2 = 0, i2 = 0, v3 = 0, i3 = 0;
        bool ch2 = false, ch3 = false;

        if (hasLive && m.which_variant == meshtastic_Telemetry_power_metrics_tag) {
            v1 = m.variant.power_metrics.ch1_voltage;
            i1 = m.variant.power_metrics.ch1_current;
            v2 = m.variant.power_metrics.ch2_voltage;
            i2 = m.variant.power_metrics.ch2_current;
            v3 = m.variant.power_metrics.ch3_voltage;
            i3 = m.variant.power_metrics.ch3_current;
            if (v2 > 0 || i2 != 0) ch2 = true;
            if (v3 > 0 || i3 != 0) ch3 = true;
        }

        float liveChUtil = (airTime != NULL) ? airTime->channelUtilizationPercent() : 0.0f;
        float liveTxUtil = (airTime != NULL) ? airTime->utilizationTXPercent() : 0.0f;

        int l = snprintf(replyMsg, sizeof(replyMsg), "JT POWER:\n");
        if (l < 0 || (size_t)l >= sizeof(replyMsg)) l = sizeof(replyMsg) - 1;

        if (v1 > 0 || i1 != 0) {
            int lTmp = snprintf(replyMsg + l, sizeof(replyMsg) - l, "V1:%.2fV I1:%.0fmA\n", v1, i1);
            if (lTmp > 0 && (size_t)lTmp < (sizeof(replyMsg) - l)) l += lTmp;
            if (ch2) {
                lTmp = snprintf(replyMsg + l, sizeof(replyMsg) - l, "V2:%.2fV I2:%.0fmA\n", v2, i2);
                if (lTmp > 0 && (size_t)lTmp < (sizeof(replyMsg) - l)) l += lTmp;
            }
            if (ch3) {
                lTmp = snprintf(replyMsg + l, sizeof(replyMsg) - l, "V3:%.2fV I3:%.0fmA\n", v3, i3);
                if (lTmp > 0 && (size_t)lTmp < (sizeof(replyMsg) - l)) l += lTmp;
            }
        } else {
            int lTmp = snprintf(replyMsg + l, sizeof(replyMsg) - l, "No Sensor Data\n");
            if (lTmp > 0 && (size_t)lTmp < (sizeof(replyMsg) - l)) l += lTmp;
        }
        snprintf(replyMsg + l, sizeof(replyMsg) - l, "ChU:%.1f%% TxU:%.1f%%", liveChUtil, liveTxUtil);
    }
    else if (strncasecmp(cmd, "PMGT", 4) == 0) {
        const char *p = cmd + 4;
        while (*p == ' ') p++;

        if (*p == '\0') {
            auto &c = config.power;
            snprintf(replyMsg, sizeof(replyMsg), "JT PMGT:\nPS:%d SD:%u BT:%u\nSDS:%u LS:%u MW:%u",
                     c.is_power_saving, (unsigned int)c.on_battery_shutdown_after_secs,
                     (unsigned int)c.wait_bluetooth_secs, (unsigned int)c.sds_secs,
                     (unsigned int)c.ls_secs, (unsigned int)c.min_wake_secs);
        } else {
            meshtastic_Config pc = meshtastic_Config_init_default;
            pc.which_payload_variant = meshtastic_Config_power_tag;
            pc.payload_variant.power = config.power;
            bool valid = true;

            if (strncasecmp(p, "SAVING ", 7) == 0) pc.payload_variant.power.is_power_saving = (strcasecmp(p + 7, "ON") == 0);
            else if (strncasecmp(p, "SHUTDOWN ", 9) == 0) pc.payload_variant.power.on_battery_shutdown_after_secs = atoi(p + 9);
            else if (strncasecmp(p, "BT_WAIT ", 8) == 0) pc.payload_variant.power.wait_bluetooth_secs = atoi(p + 8);
            else if (strncasecmp(p, "SDS ", 4) == 0) pc.payload_variant.power.sds_secs = atoi(p + 4);
            else if (strncasecmp(p, "LS ", 3) == 0) pc.payload_variant.power.ls_secs = atoi(p + 3);
            else if (strncasecmp(p, "MIN_WAKE ", 9) == 0) pc.payload_variant.power.min_wake_secs = atoi(p + 9);
            else if (strncasecmp(p, "ADC ", 4) == 0) pc.payload_variant.power.adc_multiplier_override = atof(p + 4);
            else valid = false;

            if (valid && adminModule) {
                adminModule->handleSetConfig(pc, false);
                snprintf(replyMsg, sizeof(replyMsg), "JT: Power Config Updated.");
            } else {
                snprintf(replyMsg, sizeof(replyMsg), "JT PMGT: Unknown sub-command.");
            }
        }
    }
    else if (strncasecmp(cmd, "WIFI ", 5) == 0) {
        const char *p = cmd + 5;
        while (*p == ' ') p++;

        if (strcasecmp(p, "STATUS") == 0) {
#if defined(ARCH_ESP32) && !MESHTASTIC_EXCLUDE_WIFI
            if (WiFi.status() == WL_CONNECTED) {
                snprintf(replyMsg, sizeof(replyMsg), "JT WIFI STATUS:\nSSID:%s\nPSK:%s\nRSSI:%d BSSID:%s\nIP:%s",
                         WiFi.SSID().c_str(), "[redacted]", WiFi.RSSI(), WiFi.BSSIDstr().c_str(),
                         WiFi.localIP().toString().c_str());
            } else {
                snprintf(replyMsg, sizeof(replyMsg), "JT WIFI: Not Connected.\nSSID:%s\nPSK:%s", config.network.wifi_ssid,
                         "[redacted]");
            }
#else
            snprintf(replyMsg, sizeof(replyMsg), "JT WIFI: N/A (Hardware)");
#endif
        } else {
            meshtastic_Config nc = meshtastic_Config_init_default;
            nc.which_payload_variant = meshtastic_Config_network_tag;
            nc.payload_variant.network = config.network;
            bool valid = true;

            if (strcasecmp(p, "ON") == 0) nc.payload_variant.network.wifi_enabled = true;
            else if (strcasecmp(p, "OFF") == 0) nc.payload_variant.network.wifi_enabled = false;
            else if (strncasecmp(p, "SSID ", 5) == 0) strncpy(nc.payload_variant.network.wifi_ssid, p + 5, 32);
            else if (strncasecmp(p, "PSK ", 4) == 0) strncpy(nc.payload_variant.network.wifi_psk, p + 4, 64);
            else valid = false;

            if (valid && adminModule) {
                adminModule->handleSetConfig(nc, false);
                snprintf(replyMsg, sizeof(replyMsg), "JT: WiFi Config Updated.");
            } else {
                snprintf(replyMsg, sizeof(replyMsg), "JT WIFI: Usage ON|OFF|SSID|PSK|STATUS");
            }
        }
    }
    else if (strcasecmp(cmd, "WIFI") == 0) {
#if defined(ARCH_ESP32) && !MESHTASTIC_EXCLUDE_WIFI
        if (WiFi.status() == WL_CONNECTED) {
            snprintf(replyMsg, sizeof(replyMsg),
                     "JT WIFI:\nSSID:%s\nRSSI:%d BSSID:%s\nIP:%s",
                     WiFi.SSID().c_str(), WiFi.RSSI(), WiFi.BSSIDstr().c_str(),
                     WiFi.localIP().toString().c_str());
        } else {
            snprintf(replyMsg, sizeof(replyMsg), "JT WIFI: Not Connected (Status:%d)", WiFi.status());
        }
#else
        snprintf(replyMsg, sizeof(replyMsg), "JT WIFI: N/A (No WiFi Hardware)");
#endif
    }
    else if (strncasecmp(cmd, "SETCHNAME ", 10) == 0 || strncasecmp(cmd, "USETCHNAME ", 11) == 0) {
        // Syntax: [U]SETCHNAME <idx|name> <new_name>
        bool unprotected = (strncasecmp(cmd, "U", 1) == 0);
        const char *p = cmd + (unprotected ? 11 : 10);
        const char *next = nullptr;
        int idx = findChannelByNameOrIndex(p, &next);
        p = next;

        if (idx < 0) {
            snprintf(replyMsg, sizeof(replyMsg), "JT: Invalid channel");
        } else if (!p || *p == '\0') {
            snprintf(replyMsg, sizeof(replyMsg), "JT: Missing new channel name");
        } else {
            meshtastic_Channel ch = channels.getByIndex(idx);

            // Protection Logic: Identify Trusted Channels (Nexus/Nexus2)
            const char *trusted1 =
#ifdef NEXUS_TRUSTED_CHANNEL_1
                NEXUS_TRUSTED_CHANNEL_1;
#else
                "Nexus";
#endif
            const char *trusted2 =
#ifdef NEXUS_TRUSTED_CHANNEL_2
                NEXUS_TRUSTED_CHANNEL_2;
#else
                "Nexus2";
#endif

            if (!unprotected && (strcasecmp(ch.settings.name, trusted1) == 0 || strcasecmp(ch.settings.name, trusted2) == 0)) {
                snprintf(replyMsg, sizeof(replyMsg), "JT: Error - Trusted channel protected.");
            } else {
                ch.index = idx; // Ensure index is correct for setChannel()
                strncpy(ch.settings.name, p, sizeof(ch.settings.name));
                ch.settings.name[sizeof(ch.settings.name) - 1] = '\0';

                // Nexus: Use standard setter, save, and notify
                channels.setChannel(ch);
                channels.onConfigChanged();
                nodeDB->saveToDisk(SEGMENT_CHANNELS);
                if (service) service->configChanged.notifyObservers(nullptr);

                snprintf(replyMsg, sizeof(replyMsg), "JT: Ch %d name set to '%s'", idx, p);
            }
        }
    }
    else if (strncasecmp(cmd, "SETCHPSK ", 9) == 0 || strncasecmp(cmd, "USETCHPSK ", 10) == 0) {
        // Syntax: [U]SETCHPSK <idx|name> <base64_psk>
        bool unprotected = (strncasecmp(cmd, "U", 1) == 0);
        const char *p = cmd + (unprotected ? 10 : 9);
        const char *next = nullptr;
        int idx = findChannelByNameOrIndex(p, &next);

        if (idx < 0) {
            snprintf(replyMsg, sizeof(replyMsg), "JT: Invalid channel");
        } else if (!next || *next == '\0') {
            snprintf(replyMsg, sizeof(replyMsg), "JT: Missing PSK");
        } else {
            meshtastic_Channel ch = channels.getByIndex(idx);

            // Protection Logic
            const char *trusted1 =
#ifdef NEXUS_TRUSTED_CHANNEL_1
                NEXUS_TRUSTED_CHANNEL_1;
#else
                "Nexus";
#endif
            const char *trusted2 =
#ifdef NEXUS_TRUSTED_CHANNEL_2
                NEXUS_TRUSTED_CHANNEL_2;
#else
                "Nexus2";
#endif

            if (!unprotected && (strcasecmp(ch.settings.name, trusted1) == 0 || strcasecmp(ch.settings.name, trusted2) == 0)) {
                snprintf(replyMsg, sizeof(replyMsg), "JT: Error - Trusted channel protected.");
            } else {
                size_t dlen = 0;
                unsigned char buffer[32];
                if (nexus_base64_decode(buffer, sizeof(buffer), &dlen, next) == 0) {
                    memcpy(ch.settings.psk.bytes, buffer, dlen);
                    ch.settings.psk.size = dlen;
                    ch.index = idx;
                    channels.setChannel(ch);
                    channels.onConfigChanged();
                    nodeDB->saveToDisk(SEGMENT_CHANNELS);
                    if (service) service->configChanged.notifyObservers(nullptr);
                    snprintf(replyMsg, sizeof(replyMsg), "JT: Ch %d PSK updated (%d bytes)", idx, (int)dlen);
                } else {
                    snprintf(replyMsg, sizeof(replyMsg), "JT: Invalid Base64 PSK");
                }
            }
        }
    }
    else if (strncasecmp(cmd, "ADDCH ", 6) == 0) {
        // Syntax: ADDCH <name> [base64_psk]
        const char *p = cmd + 6;
        while (*p == ' ') p++;

        char name[16];
        int i = 0;
        while (*p && *p != ' ' && i < 15) name[i++] = *p++;
        name[i] = '\0';
        while (*p == ' ') p++;

        if (strlen(name) == 0) {
            snprintf(replyMsg, sizeof(replyMsg), "JT: Missing channel name");
        } else {
            int freeIdx = -1;
            for (int j = 0; j < (int)MAX_NUM_CHANNELS; j++) {
                if (channels.getByIndex(j).role == meshtastic_Channel_Role_DISABLED) {
                    freeIdx = j;
                    break;
                }
            }

            if (freeIdx < 0) {
                snprintf(replyMsg, sizeof(replyMsg), "JT: No free channel slots");
            } else {
                meshtastic_Channel ch = channels.getByIndex(freeIdx);
                ch.index = freeIdx;
                ch.role = meshtastic_Channel_Role_SECONDARY;
                ch.has_settings = true;
                strncpy(ch.settings.name, name, sizeof(ch.settings.name));

                if (*p != '\0') {
                    size_t dlen = 0;
                    unsigned char buffer[32];
                    if (nexus_base64_decode(buffer, sizeof(buffer), &dlen, p) == 0) {
                        memcpy(ch.settings.psk.bytes, buffer, dlen);
                        ch.settings.psk.size = dlen;
                    }
                } else {
                    // Default to alias 1 (Default PSK)
                    ch.settings.psk.bytes[0] = 1;
                    ch.settings.psk.size = 1;
                }

                channels.setChannel(ch);
                channels.onConfigChanged();
                nodeDB->saveToDisk(SEGMENT_CHANNELS);
                if (service) service->configChanged.notifyObservers(nullptr);
                snprintf(replyMsg, sizeof(replyMsg), "JT: Added Ch %d '%s'", freeIdx, name);
            }
        }
    }
    else if (strncasecmp(cmd, "DELCH ", 6) == 0 || strncasecmp(cmd, "UDELCH ", 7) == 0) {
        // Syntax: [U]DELCH <idx|name>
        bool unprotected = (strncasecmp(cmd, "U", 1) == 0);
        const char *p = cmd + (unprotected ? 7 : 6);
        int idx = findChannelByNameOrIndex(p);

        if (idx < 0) {
            snprintf(replyMsg, sizeof(replyMsg), "JT: Invalid channel");
        } else if (idx == 0) {
             snprintf(replyMsg, sizeof(replyMsg), "JT: Error - Cannot delete Primary (Index 0)");
        } else {
            meshtastic_Channel ch = channels.getByIndex(idx);

            // Protection Logic
            const char *trusted1 =
#ifdef NEXUS_TRUSTED_CHANNEL_1
                NEXUS_TRUSTED_CHANNEL_1;
#else
                "Nexus";
#endif
            const char *trusted2 =
#ifdef NEXUS_TRUSTED_CHANNEL_2
                NEXUS_TRUSTED_CHANNEL_2;
#else
                "Nexus2";
#endif

            if (!unprotected && (strcasecmp(ch.settings.name, trusted1) == 0 || strcasecmp(ch.settings.name, trusted2) == 0)) {
                snprintf(replyMsg, sizeof(replyMsg), "JT: Error - Trusted channel protected.");
            } else {
                ch.index = idx;
                ch.role = meshtastic_Channel_Role_DISABLED;
                // Settings are kept but role disabled is the Meshtastic way of "deleting"
                channels.setChannel(ch);
                channels.onConfigChanged();
                nodeDB->saveToDisk(SEGMENT_CHANNELS);
                if (service) service->configChanged.notifyObservers(nullptr);
                snprintf(replyMsg, sizeof(replyMsg), "JT: Deleted Ch %d", idx);
            }
        }
    }
    else if (strcasecmp(cmd, "PHONE") == 0) {
        bool btConnected = false;
        int btRSSI = 0;

#ifdef ARCH_ESP32
        if (config.bluetooth.enabled && nimbleBluetooth) {
            btConnected = nimbleBluetooth->isConnected();
            btRSSI = nimbleBluetooth->getRssi();
        }
#elif defined(ARCH_NRF52)
        if (config.bluetooth.enabled && nrf52Bluetooth) {
            btConnected = nrf52Bluetooth->isConnected();
        }
#endif
        bool serConnected = (powerFSM.getState() == &stateSERIAL);
        bool wifiConnected = false;
        String wifiClientStr = "";
#if defined(ARCH_ESP32) && !MESHTASTIC_EXCLUDE_WIFI
        wifiConnected = (WiFi.status() == WL_CONNECTED);
#if HAS_WIFI
        if (hasWiFiClient()) {
            wifiClientStr = " App:YES (" + getWiFiClientInfo() + ")";
        } else {
            wifiClientStr = " App:NO";
        }
#endif
#endif

        snprintf(replyMsg, sizeof(replyMsg), "JT PHONE:\nBT:%s RSSI:%d\nSerial:%s\nWiFi:%s%s", btConnected ? "YES" : "NO", btRSSI,
                 serConnected ? "YES" : "NO", wifiConnected ? "YES" : "NO", wifiClientStr.c_str());
    }
    else if (strcasecmp(cmd, "POS") == 0) {
        // Source label reflects how the position was actually obtained on this node.
        //   MANUAL  -> user-configured fixed position
        //   INT     -> internal GPS hardware (live)
        //   EXT     -> external source (phone/app/peer)
        const char* source = "EXT"; // Default to External (Phone/App)
        if (config.position.fixed_position) source = "MANUAL";
#if !MESHTASTIC_EXCLUDE_GPS
        else if (gps && gps->isConnected()) source = "INT";
#endif

#if !MESHTASTIC_EXCLUDE_GPS
        // Primary path: live GPS read from the gpsStatus cache — the same source
        // /nexus SENSORS and the OLED screen use. nodePositions[our_num] only
        // populates on `updatePosition(LOCAL)` events (broadcast timer, NodeInfo
        // commits), so reading it directly causes "/nexus POS: No fix." replies
        // even when the GPS has a current lock. Mirroring the SENSORS read path
        // keeps the textual reply and the on-screen fix icon in agreement.
        if (gps && gps->isConnected() && gpsStatus) {
            const char* fix = "NoFix";
            bool hasLock = gpsStatus->getHasLock();
            if (hasLock) {
                fix = (gpsStatus->getAltitude() != 0) ? "3D" : "2D";
                int32_t lat = gpsStatus->getLatitude();
                int32_t lon = gpsStatus->getLongitude();
                int32_t alt = gpsStatus->getAltitude();
                snprintf(replyMsg, sizeof(replyMsg),
                         "JT POS [%s]:\nLat:%.5f Lon:%.5f\nAlt:%dm \nGPS:%s (%s)\nSats:%u PDOP:%.2f",
                         source,
                         lat / 1e7, lon / 1e7, (int)alt,
                         getGnssModelName(gps->p.location_source),
                         fix,
                         (unsigned)gpsStatus->getNumSatellites(),
                         (double)gpsStatus->getDOP() * 0.01);
            } else {
                // GPS online but no lock yet — report hardware status, skip Lat/Lon.
                snprintf(replyMsg, sizeof(replyMsg),
                         "JT POS [%s]:\nGPS:%s (%s)\nSats:%u PDOP:%.2f",
                         source,
                         getGnssModelName(gps->p.location_source),
                         fix,
                         (unsigned)gpsStatus->getNumSatellites(),
                         (double)gpsStatus->getDOP() * 0.01);
            }
        } else
#endif
        {
            // No GPS hardware on this variant — fall back to the PositionDB cache
            // for our own node (the T1000-E / nrf52_promicro_diy / etc. path).
            auto node = nodeDB->getMeshNode(nodeDB->getNodeNum());
            if (node && nexus_has_position(node)) {
                // Nexus-28001: slim NodeInfoLite doesn't have `position`; use the shim.
                const meshtastic_PositionLite *p = nexus_pos(node);
                snprintf(replyMsg, sizeof(replyMsg),
                         "JT POS [%s]:\nLat:%.5f Lon:%.5f\nAlt:%dm",
                         source,
                         p->latitude_i / 1e7, p->longitude_i / 1e7, (int)p->altitude);
            } else {
                snprintf(replyMsg, sizeof(replyMsg), "JT POS: No fix.");
            }
        }
    }
    else if (strncasecmp(cmd, "METRICS ", 8) == 0) {
        const char *idPtr = cmd + 8;
        uint32_t targetNum = strtoul(idPtr + (idPtr[0] == '!' ? 1 : 0), NULL, 16);
        auto node = nodeDB->getMeshNode(targetNum);
        if (node) {
            memset(cmdBuf.mBuf, 0, sizeof(cmdBuf.mBuf));
            int mLen = 0;
            if (nexus_has_device_metrics(node)) {
                mLen += snprintf(cmdBuf.mBuf + mLen, sizeof(cmdBuf.mBuf) - mLen, "Bat:%.2fV ChU:%.1f%% ",
                                 nexus_devmetrics(node)->voltage, nexus_devmetrics(node)->channel_utilization);
            }

            if (mLen > 0) {
                snprintf(replyMsg, sizeof(replyMsg), "JT METRICS %08x:\n%s", targetNum, cmdBuf.mBuf);
            } else {
                snprintf(replyMsg, sizeof(replyMsg), "JT: No metrics for %08x", targetNum);
            }
            memset(cmdBuf.mBuf, 0, sizeof(cmdBuf.mBuf));
        } else {
            snprintf(replyMsg, sizeof(replyMsg), "JT: Node %08x not found", targetNum);
        }
    }
    else if (strcasecmp(cmd, "BSAVE") == 0) {
        if (nodeDB->backupPreferences(meshtastic_AdminMessage_BackupLocation_FLASH)) {
            snprintf(replyMsg, sizeof(replyMsg), "JT: Config snapshot saved to Flash.");
        } else {
            snprintf(replyMsg, sizeof(replyMsg), "JT: BSAVE Error - Flash write failed.");
        }
    }
    else if (strcasecmp(cmd, "BLOAD") == 0) {
        if (nodeDB->restorePreferences(meshtastic_AdminMessage_BackupLocation_FLASH,
                                       SEGMENT_DEVICESTATE | SEGMENT_CONFIG | SEGMENT_MODULECONFIG | SEGMENT_CHANNELS)) {
            snprintf(replyMsg, sizeof(replyMsg), "JT: Config restored. Rebooting...");
            // AdminModule::reboot() takes SECONDS (was reboot(1000) = 16.7 min,
            // A2 in the audit). 5s leaves time for the reply to flush.
            if (adminModule) adminModule->reboot(5);
        } else {
            snprintf(replyMsg, sizeof(replyMsg), "JT: BLOAD Error - Backup not found or invalid.");
        }
    }
    else if (strcasecmp(cmd, "REBOOT") == 0) {
        LOG_INFO("Nexus: Reboot triggered");
        snprintf(replyMsg, sizeof(replyMsg), "JT: Rebooting in 20s...");
        if (adminModule) {
            adminModule->reboot(20);
        } else {
            snprintf(replyMsg, sizeof(replyMsg), "JT: Error - adminModule NULL");
        }
    }
    // Nexus LORACHIPSTATS / LCS — % of sampled seconds per LoRa chip state.
    // Counters live in RadioLibInterface, sampled at 1 Hz from NexusModule::runOnce
    // (RAM-only; zero with LCS RESET — CLRSTATS clears them too).
    else if (strcasecmp(cmd, "LORACHIPSTATS") == 0 || strcasecmp(cmd, "LCS") == 0 ||
             strncasecmp(cmd, "LORACHIPSTATS ", 14) == 0 || strncasecmp(cmd, "LCS ", 4) == 0) {
        RadioLibInterface *ri = RadioLibInterface::instance;
        if (!ri) {
            snprintf(replyMsg, sizeof(replyMsg), "JT LCS: no radio");
        } else {
            const char *sub = strchr(cmd, ' ');
            while (sub && *sub == ' ') sub++;
            if (sub && strcasecmp(sub, "RESET") == 0) {
                memset(ri->chipStateCount, 0, sizeof(ri->chipStateCount));
                ri->chipStateTotal = 0;
                snprintf(replyMsg, sizeof(replyMsg), "JT LCS: counters reset");
            } else if (ri->chipStateTotal == 0) {
                snprintf(replyMsg, sizeof(replyMsg), "JT LCS: no samples yet");
            } else {
                uint32_t total = ri->chipStateTotal;
                // tenths of a percent per bucket — integer math (newlib-nano has no %f),
                // uint64 intermediate so count*1000 can't overflow at long uptimes.
                uint32_t t[RadioLibInterface::CHIP_STATE_COUNT];
                for (int i = 0; i < RadioLibInterface::CHIP_STATE_COUNT; i++)
                    t[i] = (uint32_t)((uint64_t)ri->chipStateCount[i] * 1000 / total);
                int l = snprintf(replyMsg, sizeof(replyMsg), "JT LCS: n=%lu (~%lud)\n",
                                 (unsigned long)total, (unsigned long)(total / 86400));
                // One state per line (2026-10-07) with the raw sample count
                // next to the percentage. t[] holds tenths-of-a-percent; names[] must
                // match the ChipStateSample enum order (index 6 = CHIP_RXOFF = ERR).
                static const char *const names[RadioLibInterface::CHIP_STATE_COUNT] = {
                    "TX", "RX", "STBY", "FS", "SLP", "BUSY", "ERR", "UNK"
                };
                for (int i = 0; i < RadioLibInterface::CHIP_STATE_COUNT; i++) {
                    if (l >= (int)sizeof(replyMsg) - 1) break; // truncation guard
                    l += snprintf(replyMsg + l, sizeof(replyMsg) - l, "%s %u.%u%% (%lu)\n",
                                  names[i], t[i] / 10, t[i] % 10,
                                  (unsigned long)ri->chipStateCount[i]);
                    if (l > (int)sizeof(replyMsg) - 1) l = (int)sizeof(replyMsg) - 1; // snprintf returns the would-be length
                }
                if (l > 0 && replyMsg[l - 1] == '\n')
                    replyMsg[l - 1] = '\0'; // drop the trailing newline
            }
        }
    }
    else if (strcasecmp(cmd, "HEAP") == 0) {
        uint32_t free = memGet.getFreeHeap();
        uint32_t total = memGet.getHeapSize();
        uint32_t used = (total > free) ? (total - free) : 0;
        uint32_t maxBlock = memGet.getMinFreeBlock();

        int usedPerc = (total > 0) ? (int)(used * 100 / total) : 0;
        int fragPerc = (free > 0) ? (int)(maxBlock * 100 / free) : 100;

        snprintf(replyMsg, sizeof(replyMsg), "JT HEAP:\nU:%u(%d%%) F:%u\nMaxBlk:%u\nFrag:%d%% (Contig)",
                 used, usedPerc, free, maxBlock, fragPerc);
    }
    else if (strcasecmp(cmd, "RAM") == 0 || strcasecmp(cmd, "MEM") == 0) {
        // Compact one-message diagnostic. All values in bytes except PSRAM
// (bytes-only too) and uxTaskGetStackHighWaterMark (FreeRTOS words).
// Target: single reply under 200 B (NEXUS_MAX_REPLY).
        uint32_t hFree  = memGet.getFreeHeap();
        uint32_t hTotal = memGet.getHeapSize();
        uint32_t hUsed  = (hTotal > hFree) ? (hTotal - hFree) : 0;
        uint32_t hBlk   = memGet.getMinFreeBlock();
        uint32_t pFree  = memGet.getFreePsram();
        uint32_t pTotal = memGet.getPsramSize();

        // hBlk is intentionally read for completeness but not exposed in the
        // compact output (user feedback: 'W' field was confusing).

#ifdef HAS_FREE_RTOS
        UBaseType_t hereFree = uxTaskGetStackHighWaterMark(NULL);
        const char *taskName = pcTaskGetName(NULL);
        if (!taskName) taskName = "?";
        // Truncate task name to 12 chars to keep reply under 200 B.
        // Static (BSS) so we don't add 13B to every /nexus RAM call stack.
        static char tn[13];
        memset(tn, 0, sizeof(tn));
        strncpy(tn, taskName, sizeof(tn) - 1);
        tn[sizeof(tn) - 1] = '\0';
#else
        const char *tn = "?";
        uint32_t hereFree = 0;
#endif

        if (pTotal > 0) {
            uint32_t pUsed = (pTotal > pFree) ? (pTotal - pFree) : 0;
            snprintf(replyMsg, sizeof(replyMsg),
                     "HEAP U:%u F:%u T:%u MB:%u\n"
                     "PSRAM U:%lu F:%lu T:%lu\n"
                     "NEXUS bss:%u\n"
                     "STACK %s free:%u words",
                     (unsigned)hUsed, (unsigned)hFree, (unsigned)hTotal,
                     (unsigned)hBlk,
                     (unsigned long)pUsed, (unsigned long)pFree, (unsigned long)pTotal,
                     (unsigned)sizeof(NexusModule),
                     tn, (unsigned)hereFree);
        } else {
            snprintf(replyMsg, sizeof(replyMsg),
                     "HEAP U:%u F:%u T:%u MB:%u\n"
                     "PSRAM n/a\n"
                     "NEXUS bss:%u\n"
                     "STACK %s free:%u words",
                     (unsigned)hUsed, (unsigned)hFree, (unsigned)hTotal,
                     (unsigned)hBlk,
                     (unsigned)sizeof(NexusModule),
                     tn, (unsigned)hereFree);
        }
    }
    else if (strcasecmp(cmd, "MQTTSTATS") == 0 || strcasecmp(cmd, "MQS") == 0) {
        uint32_t total = mqtt_ok_on + mqtt_ok_off_flag_on + mqtt_both_off;
        int onPerc = (total > 0) ? (int)(mqtt_ok_on * 100 / total) : 0;
        int mqttPerc = (total > 0) ? (int)(mqtt_ok_off_flag_on * 100 / total) : 0;
        int offPerc = (total > 0) ? (int)(mqtt_both_off * 100 / total) : 0;

        snprintf(replyMsg, sizeof(replyMsg), "JT MQTT:\nON:%u(%d%%) MQTT:%u(%d%%)\nOFF:%u(%d%%) Tot:%u",
                 mqtt_ok_on, onPerc, mqtt_ok_off_flag_on, mqttPerc, mqtt_both_off, offPerc, total);
    }
    else if (strcasecmp(cmd, "STATS") == 0) {
        uint32_t tx = 0, rx = 0, rxBad = 0, txRelay = 0, txDrop = 0;
        if (RadioLibInterface::instance) {
            tx = RadioLibInterface::instance->txGood;
            rxBad = RadioLibInterface::instance->rxBad;
            rx = RadioLibInterface::instance->rxGood + rxBad;
            txRelay = RadioLibInterface::instance->txRelay;
            txDrop = RadioLibInterface::instance->txDrop;
        }
        uint32_t rxDupe = (router) ? router->rxDupe : 0;
        uint32_t txCanceled = (router) ? router->txRelayCanceled : 0;

        int32_t noise = (RadioLibInterface::instance) ? RadioLibInterface::instance->getAverageNoiseFloor() : -120;
        int rxBadPerc = (rx > 0) ? (int)(rxBad * 100 / rx) : 0;
        int dupePerc = (rx > 0) ? (int)(rxDupe * 100 / rx) : 0;

        snprintf(replyMsg, sizeof(replyMsg), "JT STATS:\nTX:%u RX:%u\nBad:%u(%d%%) Dp:%u(%d%%)\nRelay:%u Can:%u\nDrop:%u Noise:%d",
                         tx, rx, rxBad, rxBadPerc, rxDupe, dupePerc, txRelay, txCanceled, txDrop, noise);
    }
    else if (strncasecmp(cmd, "SEARCHKEY ", 10) == 0) {
        // Syntax: SEARCHKEY <base64_pubkey>
        const char *p = cmd + 10;
        while (*p == ' ') p++;

        size_t dlen = 0;
        unsigned char searchPK[32];
        if (nexus_base64_decode(searchPK, sizeof(searchPK), &dlen, p) != 0 || dlen != 32) {
            snprintf(replyMsg, sizeof(replyMsg), "JT: Invalid 32-byte Base64 key");
        } else {
            startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "SearchKey Results");
            int matches = 0;
            static char line[64];
            for (size_t i = 0; i < nodeDB->getNumMeshNodes(); i++) {
                auto n = nodeDB->getMeshNodeByIndex(i);
                if (n->public_key.size == 32 && memcmp(n->public_key.bytes, searchPK, 32) == 0) {
                    memset(line, 0, sizeof(line));
                    snprintf(line, sizeof(line), "!%08x: %s (%s)\n", (unsigned int)n->num, n->long_name, n->short_name);
                    appendPaging(line);
                    matches++;
                }
            }
            if (matches == 0) appendPaging("No matches found.\n");
            snprintf(replyMsg, sizeof(replyMsg), "JT: Found %d matching nodes", matches);
            memset(line, 0, sizeof(line));
        }
    }
    else if (strcasecmp(cmd, "SIGSTATS") == 0 || strcasecmp(cmd, "SS") == 0) {
        uint32_t decryptedCount = (this->rx_total_count >= this->rx_private_count) ? (this->rx_total_count - this->rx_private_count) : 0;
        uint32_t privPerc = (this->rx_total_count > 0) ? (this->rx_private_count * 100 / this->rx_total_count) : 0;
        uint32_t decPerc = (this->rx_total_count > 0) ? (decryptedCount * 100 / this->rx_total_count) : 0;
        int32_t noise = (RadioLibInterface::instance) ? RadioLibInterface::instance->getAverageNoiseFloor() : -120;

        snprintf(replyMsg, sizeof(replyMsg), "JT SIG-STATS:\nTotal Wire: %u\nPrivate: %u (%u%%)\nKnown/Dec: %u (%u%%)\nNoise Floor: %d dBm",
                 this->rx_total_count, this->rx_private_count, privPerc, decryptedCount, decPerc, noise);
    }
    else if (strcasecmp(cmd, "VERIFYSTATS") == 0 || strcasecmp(cmd, "VS") == 0) {
        // XEdDSA signature verification buckets. Each wire packet that reaches
        // updateStats() falls into exactly one of these three:
        //   * NoSig  — decrypted with no xeddsa_signature, OR encrypted
        //              (we can't see inside, so we treat as unsigned)
        //   * SigInv — decrypted with signature but framework rejected
        //              (mp.xeddsa_signed == false; see Router.cpp:checkXeddsa
        //              ReceivePolicy)
        //   * SigOK  — decrypted and verified (mp.xeddsa_signed == true)
        // Sum of bucket counts == rx_total_count.
        uint32_t total = this->rx_no_sig_count + this->rx_sig_invalid_count + this->rx_sig_valid_count;
        uint32_t totalBytes = this->rx_no_sig_bytes + this->rx_sig_invalid_bytes + this->rx_sig_valid_bytes;
        uint32_t noPct = (total > 0) ? (this->rx_no_sig_count * 100 / total) : 0;
        uint32_t invPct = (total > 0) ? (this->rx_sig_invalid_count * 100 / total) : 0;
        uint32_t okPct = (total > 0) ? (this->rx_sig_valid_count * 100 / total) : 0;

        snprintf(replyMsg, sizeof(replyMsg),
                 "JT VERIFY-STATS:\nNoSig:  %u pkts, %u B (%u%%)\nSigInv: %u pkts, %u B (%u%%)\nSigOK:  %u pkts, %u B (%u%%)\nTotal:  %u pkts, %u B",
                 this->rx_no_sig_count, this->rx_no_sig_bytes, noPct,
                 this->rx_sig_invalid_count, this->rx_sig_invalid_bytes, invPct,
                 this->rx_sig_valid_count, this->rx_sig_valid_bytes, okPct,
                 total, totalBytes);
    }
    else if (strcasecmp(cmd, "UPTIME") == 0 || strcasecmp(cmd, "UP") == 0) {
        // Use Time::getMillisMonotonic() (uint64_t) so the display never rolls over at the
        // 49.7-day Time::getMillis() boundary. Seconds fit in uint32_t (~136 years to overflow days).
        uint64_t ms = Time::getMillisMonotonic();
        uint32_t s  = (uint32_t)(ms / 1000ULL);
        uint32_t d  = s / 86400; s %= 86400;
        uint32_t h  = s / 3600;  s %= 3600;
        uint32_t m  = s / 60;    s %= 60;
        snprintf(replyMsg, sizeof(replyMsg), "JT UPTIME: %ud %uh %um %us", d, h, m, s);
    }
    else if (strcasecmp(cmd, "ROLE") == 0) {
        snprintf(replyMsg, sizeof(replyMsg), "JT ROLE: %s (%u)", getRoleName(config.device.role), (uint32_t)config.device.role);
    }
    else if (strncasecmp(cmd, "SETROLE ", 8) == 0) {
        const char *roleStr = cmd + 8;
        while (*roleStr == ' ') roleStr++;

        int role = -1;
        if (strcasecmp(roleStr, "CLIENT") == 0) role = meshtastic_Config_DeviceConfig_Role_CLIENT;
        else if (strcasecmp(roleStr, "SENS") == 0 || strcasecmp(roleStr, "SENSOR") == 0) role = meshtastic_Config_DeviceConfig_Role_SENSOR;
        else if (strcasecmp(roleStr, "RTR") == 0 || strcasecmp(roleStr, "ROUTER") == 0) role = meshtastic_Config_DeviceConfig_Role_ROUTER;
        else if (strcasecmp(roleStr, "RTR_CLIENT") == 0) role = meshtastic_Config_DeviceConfig_Role_ROUTER_CLIENT;
        else if (strcasecmp(roleStr, "BASE") == 0 || strcasecmp(roleStr, "CLIENT_BASE") == 0) role = meshtastic_Config_DeviceConfig_Role_CLIENT_BASE;
        else if (strcasecmp(roleStr, "MUTE") == 0 || strcasecmp(roleStr, "CLIENT_MUTE") == 0) role = meshtastic_Config_DeviceConfig_Role_CLIENT_MUTE;
        else if (strcasecmp(roleStr, "HIDDEN") == 0 || strcasecmp(roleStr, "CLIENT_HIDDEN") == 0) role = meshtastic_Config_DeviceConfig_Role_CLIENT_HIDDEN;
        else if (strcasecmp(roleStr, "LOST") == 0 || strcasecmp(roleStr, "LOST_AND_FOUND") == 0) role = meshtastic_Config_DeviceConfig_Role_LOST_AND_FOUND;
        else if (strcasecmp(roleStr, "TAK") == 0) role = meshtastic_Config_DeviceConfig_Role_TAK;
        else if (strcasecmp(roleStr, "TAK_TRACKER") == 0) role = meshtastic_Config_DeviceConfig_Role_TAK_TRACKER;
        else if (isdigit(roleStr[0])) role = atoi(roleStr);

        if (role >= 0 && role <= 13) {
            meshtastic_Config c = meshtastic_Config_init_default;
            c.which_payload_variant = meshtastic_Config_device_tag;
            c.payload_variant.device = config.device;
            c.payload_variant.device.role = (meshtastic_Config_DeviceConfig_Role)role;

            if (adminModule) adminModule->handleSetConfig(c, false);
            snprintf(replyMsg, sizeof(replyMsg), "JT: Role updated to %s (%d).", roleStr, role);
        } else {
            snprintf(replyMsg, sizeof(replyMsg), "JT: Invalid role '%s'", roleStr);
        }
    }
    else if (strcasecmp(cmd, "LNA") == 0) {
#if HAS_LORA_FEM
        if (loraFEMInterface.isLnaCanControl()) {
            bool lnaEnabled = (config.lora.fem_lna_mode == meshtastic_Config_LoRaConfig_FEM_LNA_Mode_ENABLED);
            snprintf(replyMsg, sizeof(replyMsg), "JT LNA: %s", lnaEnabled ? "ENABLED" : "BYPASS");
        } else {
            snprintf(replyMsg, sizeof(replyMsg), "JT LNA: Hardware does not support LNA control");
        }
#else
        snprintf(replyMsg, sizeof(replyMsg), "JT LNA: NO_FEM");
#endif
    }
    else if (strncasecmp(cmd, "SETLNA ", 7) == 0) {
#if HAS_LORA_FEM
        if (loraFEMInterface.isLnaCanControl()) {
            const char *p = cmd + 7;
            bool enable = false;
            bool valid = false;
            if (strcasecmp(p, "ON") == 0 || strcasecmp(p, "ENABLED") == 0 || strcasecmp(p, "1") == 0) {
                enable = true;
                valid = true;
            } else if (strcasecmp(p, "OFF") == 0 || strcasecmp(p, "BYPASS") == 0 || strcasecmp(p, "0") == 0) {
                enable = false;
                valid = true;
            }

            if (valid) {
                meshtastic_Config c = meshtastic_Config_init_default;
                c.which_payload_variant = meshtastic_Config_lora_tag;
                c.payload_variant.lora = config.lora;
                c.payload_variant.lora.fem_lna_mode = enable ? meshtastic_Config_LoRaConfig_FEM_LNA_Mode_ENABLED : meshtastic_Config_LoRaConfig_FEM_LNA_Mode_DISABLED;

                if (adminModule) {
                    adminModule->handleSetConfig(c, false);
                    snprintf(replyMsg, sizeof(replyMsg), "JT: LNA set to %s", enable ? "ENABLED" : "BYPASS");
                } else {
                    snprintf(replyMsg, sizeof(replyMsg), "JT: Error - adminModule NULL");
                }
            } else {
                snprintf(replyMsg, sizeof(replyMsg), "JT: Invalid LNA param (ON|OFF|BYPASS)");
            }
        } else {
            snprintf(replyMsg, sizeof(replyMsg), "JT LNA: Hardware does not support LNA control");
        }
#else
        snprintf(replyMsg, sizeof(replyMsg), "JT LNA: NO_FEM");
#endif
    }
    else if (strncasecmp(cmd, "SETPOWER ", 9) == 0 || strncasecmp(cmd, "SETTXPOWER ", 11) == 0 || strncasecmp(cmd, "SETPWR ", 7) == 0) {
        const char *p = (strncasecmp(cmd, "SETPOWER ", 9) == 0) ? cmd + 9 :
                        (strncasecmp(cmd, "SETTXPOWER ", 11) == 0) ? cmd + 11 : cmd + 7;
        int8_t pwr = atoi(p);
        if (pwr >= -9 && pwr <= 31) {
            meshtastic_Config c = meshtastic_Config_init_default;
            c.which_payload_variant = meshtastic_Config_lora_tag;
            c.payload_variant.lora = config.lora;
            c.payload_variant.lora.tx_power = pwr;

            if (adminModule) adminModule->handleSetConfig(c, false);
            snprintf(replyMsg, sizeof(replyMsg), "JT: TX Power set to %d dBm.", pwr);
        } else {
            snprintf(replyMsg, sizeof(replyMsg), "JT: Invalid Power (-9 to 31)");
        }
    }
    else if (strcasecmp(cmd, "GETTXPOWER") == 0) {
        int activePwr = (int)config.lora.tx_power;
        int sxPwr = (int)getChipPower(activePwr);
        snprintf(replyMsg, sizeof(replyMsg), "JT TX Power:\nActive: %d dBm\nChip: %d dBm",
                 activePwr, sxPwr);
    }
    else if (strncasecmp(cmd, "CLIENTALWAYS ", 13) == 0 || strcasecmp(cmd, "CLIENTALWAYS") == 0) {
        const char *p = (strlen(cmd) > 13) ? cmd + 13 : "";
        while (*p == ' ') p++;

        if (*p == '\0') {
            snprintf(replyMsg, sizeof(replyMsg), "JT: Client Always is %s", client_always ? "ON" : "OFF");
        } else {
            bool enable = (strcasecmp(p, "ON") == 0 || strcasecmp(p, "1") == 0);
            client_always = enable;
            // saveVault();
            snprintf(replyMsg, sizeof(replyMsg), "JT: Client Always set to %s", enable ? "ON" : "OFF");
        }
    }
    else if (strncasecmp(cmd, "BASETEXTS ", 10) == 0 || strcasecmp(cmd, "BASETEXTS") == 0) {
        const char *p = (strlen(cmd) > 10) ? cmd + 10 : "";
        while (*p == ' ') p++;

        if (*p == '\0') {
            snprintf(replyMsg, sizeof(replyMsg), "JT: Base Texts is %s", base_texts ? "ON" : "OFF");
        } else {
            bool enable = (strcasecmp(p, "ON") == 0 || strcasecmp(p, "1") == 0);
            base_texts = enable;
            // saveVault();
            snprintf(replyMsg, sizeof(replyMsg), "JT: Base Texts set to %s", enable ? "ON" : "OFF");
        }
    }
    else if (strcasecmp(cmd, "SETTINGS") == 0 || strcasecmp(cmd, "ST") == 0) {
        // Human-readable tactical overview
        char hMask[16];
        if (!hop_mask_active) strcpy(hMask, "OFF"); else snprintf(hMask, sizeof(hMask), "%d/%d", hop_mask_start, hop_mask_limit);

        uint32_t ppHex = public_ping ? 1 : 0;
        for(int i=0; i<8; i++) if (ping_channels[i]) ppHex |= (1 << (i+1));

        int skeyCount = 0;
        for(int i=0; i<4; i++) {
            bool allZero = true;
            for(int j=0; j<32; j++) { if(nexus_secondary_keys[i][j] != 0) { allZero = false; break; } }
            if(!allZero) skeyCount++;
        }

        // IMPORTANT: do NOT use %llX / %llx here. See the top-of-file comment
        // ("PROJECT-WIDE PRINTF NOTE") for the newlib-nano rationale. Split
        // getNexusDropHex() into two 32-bit halves instead; without the split,
        // %llX leaks literal 'lX' into the reply AND shifts va_args so DMF:%d
        // reads garbage from the unconsumed upper half instead of the bool.
        uint64_t dmask = getNexusDropHex();
        unsigned long dmask_hi = (unsigned long)(dmask >> 32);
        unsigned long dmask_lo = (unsigned long)(dmask & 0xFFFFFFFFu);

        snprintf(replyMsg, sizeof(replyMsg),
                 "JT SETTINGS:\nRole:%s Hops:%s\nCA:%d BT:%d DMF:%d\nDrops:0x%08lX%08lX\nPPing:0x%03X SK:%d/4 BD:%us",
                 getRoleName(config.device.role), hMask,
                 client_always, base_texts, dm_forward,
                 dmask_hi, dmask_lo,
                 (unsigned int)(ppHex & 0xFFF), skeyCount,
                 (unsigned)broadcast_delay_max_s);
    }
    else if (strncasecmp(cmd, "SECURITY ", 9) == 0 || strcasecmp(cmd, "SECURITY") == 0 ||
             strncasecmp(cmd, "SEC ",     4) == 0 || strcasecmp(cmd, "SEC")     == 0) {
        // /nexus SECURITY (or SEC) — inspect and toggle the nexus_security_bitmask
        // (Nexus-28003, 2026-08-24). The bitmask lives in the vault at v65;
        // no v66 bump needed — the slot has been there since v65.
        //
        // Forms:
        //   /nexus SECURITY                            → overview (like SETTINGS)
        //   /nexus SECURITY REQ_SIG [on|off]           → toggle bit 0
        //   /nexus SECURITY ALLOW_DM [on|off]          → toggle bit 1
        //   /nexus SECURITY SILENT_LOG [on|off]        → toggle bit 2
        //   /nexus SECURITY AUTO_FAV_NEXUS [on|off]    → toggle bit 3
        //   /nexus SECURITY AUTO_FAV_TRUSTED [on|off]  → toggle bit 4
        //   /nexus SECURITY BYPASS_RP [on|off]         → toggle bit 5 (inverted: on=bypass active)
        //   /nexus SECURITY AUTO_FAV_EXTRA [on|off|<0-7>] → bit 6 enable; a digit 0-7 sets bits 7-9 (channel) AND enables
        //   /nexus SECURITY [on|off]                   → shortcut to set/clear ALL bits
        //
        // Bit semantics (all default OFF):
        //   REQ_SIG    — when ON, /nexus commands on Nexus channels must carry
        //                a valid XEdDSA signature from a trusted key.
        //   ALLOW_DM   — when ON, /nexus commands received as DMs are accepted.
        //                DMs are never signed; auth falls back to isAuthorizedAdmin().
        //   SILENT_LOG — when ON, packets rejected by REQ_SIG/ALLOW_DM are
        //                silently logged and dropped. When OFF, the sender
        //                receives a 'JT SECURITY: rejected — ...' reply.
        //   AUTO_FAV_NEXUS   — when ON, any decoded packet (non-zero portnum) seen
        //                on a Nexus channel from a non-self sender auto-favourites
        //                the sender. No flash save.
        //   AUTO_FAV_TRUSTED — when ON, any decoded packet seen on a Nexus channel
        //                whose verifyTrustedSignature() succeeds (i.e. signed by a
        //                Nexus trusted key) auto-favourites the sender. No flash save.
        //
        // Aliases (Nexus-28004, 2026-08-30):
        //   REQ_SIG        → RSIG, RS
        //   ALLOW_DM       → AD, ADM
        //   SILENT_LOG     → SL, SIL
        //   AUTO_FAV_NEXUS → AFN, AN
        //   AUTO_FAV_TRUSTED → AFT, AT
        //   AUTO_FAV_EXTRA → AFX
        //
        // "Internal" packets (isFromUs — anonymous local OR self-rebroadcast, per
        // NodeDB.cpp:724) bypass ALL of these rules. The protobuf flag Meshtastic
        // sets on packets that arrive over LoRa/UDP/MQTT is not separately tracked
        // on the wire; isFromUs is the canonical proxy.

        // First, parse any arguments (only relevant when there IS something to parse).
        // The overview path (no args / unrecognised args) and the per-bit toggle
        // path share the tail via the isOverview flag.
        const char *targetName = nullptr;
        bool turnOn = false;
        bool turnOff = false;
        bool isOverview = true;
        // AUTO_FAV_EXTRA channel form: /nexus SECURITY AFX <0-7> sets bits 7-9
        // of the bitmask and enables bit 6 in the same round trip.
        bool hasExtraCh = false;
        int setExtraCh = 0;
        // Nexus-28006 (2026-09-16) — code review 1D fix.
        // Set to true on the unknown-bit path (operator typo, e.g.
        // /nexus SECURITY BOGUS). Gates the bit-toggle + overview blocks
        // below so they don't clobber replyMsg with their own snprintf.
        // After populating replyMsg, fall through to the dispatcher tail
        // at the end of handleNexusCommand (which sends the unknown-bit
        // error to the operator). DO NOT set this flag and continue the
        // handler — the bit-toggle and overview blocks below will run
        // their snprintf and overwrite the error message.
        bool securityReject = false;

        // Use skipToken() so both /nexus SECURITY REQ_SIG on and the /nexus SEC
        // REQ_SIG on shortcut advance past the command word correctly. The
        // previous implementation hardcoded cmd[8] which only worked for the
        // 8-char "SECURITY" form — the 3-char "SEC" form landed the parser in
        // the middle of the first arg and produced "JT SECURITY: unknown bit '...'".
        const char *p = skipToken(cmd);
        if (*p != '\0') {
            // Skip leading whitespace, then read the sub-token.
            char sub[24] = {0};
            int si = 0;
            while (*p && *p != ' ' && si < (int)sizeof(sub) - 1) sub[si++] = *p++;
            while (*p == ' ') p++;

            char arg[8] = {0};
            int ai = 0;
            while (*p && *p != ' ' && ai < (int)sizeof(arg) - 1) arg[ai++] = *p++;

            // Resolve which bit the operator wants to toggle. Aliases are
            // checked in addition to the canonical names so RSIG/RS, AD/ADM,
            // SL/SIL all map to their respective bit.
            if (strcasecmp(sub, "REQ_SIG") == 0 || strcasecmp(sub, "RSIG") == 0 || strcasecmp(sub, "RS") == 0) {
                targetName = "REQ_SIG";
            } else if (strcasecmp(sub, "ALLOW_DM") == 0 || strcasecmp(sub, "AD") == 0 || strcasecmp(sub, "ADM") == 0) {
                targetName = "ALLOW_DM";
            } else if (strcasecmp(sub, "SILENT_LOG") == 0 || strcasecmp(sub, "SL") == 0 || strcasecmp(sub, "SIL") == 0) {
                targetName = "SILENT_LOG";
            } else if (strcasecmp(sub, "AUTO_FAV_NEXUS") == 0 || strcasecmp(sub, "AFN") == 0 || strcasecmp(sub, "AN") == 0) {
                targetName = "AUTO_FAV_NEXUS";
            } else if (strcasecmp(sub, "AUTO_FAV_TRUSTED") == 0 || strcasecmp(sub, "AFT") == 0 || strcasecmp(sub, "AT") == 0) {
                targetName = "AUTO_FAV_TRUSTED";
            } else if (strcasecmp(sub, "AUTO_FAV_EXTRA") == 0 || strcasecmp(sub, "AFX") == 0) {
                targetName = "AUTO_FAV_EXTRA";
            } else if (strcasecmp(sub, "BYPASS_RP") == 0 || strcasecmp(sub, "BRP") == 0) {
                // Bypass_RP (Nexus-28005, 2026-09-06): opt-out from
                // replay protection. Inverted semantics — `on` means
                // "bypass IS active", `off` means "replay protection
                // active (default)". See NexusModule.h:NEXUS_SECURITY_BYPASS_RP.
                targetName = "BYPASS_RP";
            } else if (arg[0] == '\0' && (strcasecmp(sub, "on") == 0 || strcasecmp(sub, "off") == 0)) {
                // Bare "/nexus SECURITY on|off" — set ALL bits uniformly.
                targetName = "ALL";
            }

            if (targetName) {
                isOverview = false;
                if (strcasecmp(arg, "on") == 0) turnOn = true;
                else if (strcasecmp(arg, "off") == 0) turnOff = true;
                else if (strcmp(targetName, "AUTO_FAV_EXTRA") == 0 && arg[0] != '\0') {
                    // AUTO_FAV_EXTRA takes a channel number INSTEAD of on/off:
                    // /nexus SECURITY AFX 3 -> select channel 3 and enable.
                    if (arg[1] == '\0' && arg[0] >= '0' && arg[0] <= '7') {
                        setExtraCh = arg[0] - '0';
                        hasExtraCh = true;
                        turnOn = true;
                    } else {
                        snprintf(replyMsg, sizeof(replyMsg),
                                 "JT SECURITY: AUTO_FAV_EXTRA channel must be 0-7");
                        securityReject = true;
                    }
                } else {
                    // No on/off arg — leave the bit unchanged and treat as overview.
                    isOverview = true;
                }
            } else if (arg[0] == '\0' && sub[0] == '\0') {
                // Bare "/nexus SECURITY" — overview.
                isOverview = true;
            } else if (sub[0] != '\0') {
                snprintf(replyMsg, sizeof(replyMsg),
                         "JT SECURITY: unknown bit '%s' (valid: REQ_SIG/RSIG/RS, ALLOW_DM/AD/ADM, SILENT_LOG/SL/SIL, AUTO_FAV_NEXUS/AFN/AN, AUTO_FAV_TRUSTED/AFT/AT, AUTO_FAV_EXTRA/AFX, BYPASS_RP/BRP)", sub);
                // Nexus-28006 (2026-09-16) — code review 1D fix.
                // Previously: `return;` here dropped replyMsg before the
                // dispatcher tail at the end of handleNexusCommand could
                // sendToPhone / queueMeshReply it. Operator saw silence after
                // /nexus SECURITY BOGUS despite replyMsg being correctly populated.
                // Set the securityReject flag so the bit-toggle + overview
                // blocks below skip their work (which would clobber this
                // unknown-bit message with their own snprintf), then fall
                // through to the dispatcher tail at the end of the function.
                securityReject = true;
            }
        }

        if (!isOverview && !securityReject) {
            // Apply the bit change. "ALL" is a shortcut for "set every bit to
            // the same on/off value". Persistent via the in-memory bitmask;
            // operator must run /nexus SAVE to flush to /nexus.bin.
            uint32_t newMask = nexus_security_bitmask;
            if (targetName && strcmp(targetName, "ALL") == 0) {
                if (turnOn) newMask = NEXUS_SECURITY_REQUIRE_SIG | NEXUS_SECURITY_ALLOW_DM |
                                       NEXUS_SECURITY_SILENT_LOG | NEXUS_SECURITY_AUTO_FAV_NEXUS |
                                       NEXUS_SECURITY_AUTO_FAV_TRUSTED;
                else if (turnOff) newMask = 0;
            } else if (targetName) {
                uint32_t bit = 0;
                if (strcmp(targetName, "REQ_SIG") == 0) bit = NEXUS_SECURITY_REQUIRE_SIG;
                else if (strcmp(targetName, "ALLOW_DM") == 0) bit = NEXUS_SECURITY_ALLOW_DM;
                else if (strcmp(targetName, "SILENT_LOG") == 0) bit = NEXUS_SECURITY_SILENT_LOG;
                else if (strcmp(targetName, "AUTO_FAV_NEXUS") == 0) bit = NEXUS_SECURITY_AUTO_FAV_NEXUS;
                else if (strcmp(targetName, "AUTO_FAV_TRUSTED") == 0) bit = NEXUS_SECURITY_AUTO_FAV_TRUSTED;
                else if (strcmp(targetName, "BYPASS_RP") == 0)         bit = NEXUS_SECURITY_BYPASS_RP;
                else if (strcmp(targetName, "AUTO_FAV_EXTRA") == 0)    bit = NEXUS_SECURITY_AUTO_FAV_EXTRA;
                if (hasExtraCh) {
                    newMask &= ~NEXUS_SECURITY_AUTO_FAV_EXTRA_CH_MASK;
                    newMask |= ((uint32_t)setExtraCh << NEXUS_SECURITY_AUTO_FAV_EXTRA_CH_SHIFT);
                }
                if (turnOn) newMask |= bit;
                else if (turnOff) newMask &= ~bit;
            }
            nexus_security_bitmask = newMask;

            // Tiny ack (no full overview; operator can run /nexus SECURITY for that).
            // Nexus-28006 (2026-09-16) — code review 1D fix.
            // Previously: trailing `return;` dropped replyMsg before the
            // dispatcher tail at the end of handleNexusCommand could
            // sendToPhone / queueMeshReply it. Drop the return; — the
            // dispatcher tail picks up replyMsg and delivers the ack.
            if (hasExtraCh) {
                snprintf(replyMsg, sizeof(replyMsg),
                         "JT SECURITY: AUTO_FAV_EXTRA ON ch%u -> 0x%08lX (run /nexus SAVE to persist)",
                         (unsigned)setExtraCh, (unsigned long)newMask);
            } else {
                snprintf(replyMsg, sizeof(replyMsg),
                         "JT SECURITY: %s -> 0x%08lX (run /nexus SAVE to persist)",
                         targetName,
                         (unsigned long)newMask);
            }
            // (return; removed — see code review 1D comment above.)
        }

        if (!securityReject) {
        // Overview. Multiline, fits in NEXUS_MAX_REPLY (200 B). 6 bits + replay stats.
        // Mask line was removed (Nexus-28004, 2026-09-01) — per-bit on/off
        // below already encode the bitmask, no need to dump the raw hex too.
        //
        // Nexus-28005 (2026-09-13): trailing `return;` added to fix
        // silent-fail on bare /nexus SECURITY (and SECURITY LIST). Without
        // the return, control falls through to the next else-if block
        // (VAULT), then to the rest of the dispatcher chain, then to the
        // terminal `else` at line ~8979 which overwrites replyMsg with
        // `JT: Unknown command 'SECURITY'`. The phone app silently drops
        // that string (no JT-recognized command name) so operators see
        // nothing — same symptom as the LR2021_SF silent-fail we fixed
        // earlier. Bit-toggle path was unaffected because it already had
        // `return;` (line ~4160).
        //
        // Nexus-28006 (2026-09-16) — code review 1D fix.
        // The Nexus-28005 `return;` mentioned above skips the terminal-else
        // overwrite but ALSO skips the dispatcher tail at the very end of
        // handleNexusCommand (the one with sendToPhone + queueMeshReply).
        // So bare /nexus SECURITY was silent at the caller — replyMsg was
        // populated but never delivered.
        //
        // Fix: gate this entire overview block on `!securityReject` (so
        // the unknown-bit message from the earlier branch isn't clobbered
        // by THIS snprintf) AND drop the trailing `return;`. The dispatcher
        // tail at the end of handleNexusCommand now delivers the overview
        // reply. Mirrors the same fix in the bit-toggle block above.
        snprintf(
            replyMsg, sizeof(replyMsg),
            "JT SECURITY:\n REQ_SIG:%s\n ALLOW_DM:%s\n SILENT:%s\n FAV_NX:%s\n FAV_TR:%s\n FAV_XT:%s ch%u\n BYPASS_RP:%s (%u/%u)",
            (nexus_security_bitmask & NEXUS_SECURITY_REQUIRE_SIG) ? "ON " : "off",
            (nexus_security_bitmask & NEXUS_SECURITY_ALLOW_DM) ? "ON " : "off",
            (nexus_security_bitmask & NEXUS_SECURITY_SILENT_LOG) ? "ON " : "off",
            (nexus_security_bitmask & NEXUS_SECURITY_AUTO_FAV_NEXUS) ? "ON " : "off",
            (nexus_security_bitmask & NEXUS_SECURITY_AUTO_FAV_TRUSTED) ? "ON " : "off",
            (nexus_security_bitmask & NEXUS_SECURITY_AUTO_FAV_EXTRA) ? "ON" : "off", (unsigned)securityAutoFavExtraChannel(),
            securityBypassRP() ? "ON " : "off", (unsigned)replayRing.size(), (unsigned)REPLAY_RING_SIZE);
        // (return; removed — dispatcher tail at the end of handleNexusCommand
        //  now delivers the reply. See code review 1D comment above.)
        }
    }
    else if (strcasecmp(cmd, "VAULT") == 0 || strcasecmp(cmd, "VST") == 0) {
        // /nexus VAULT (or VST) - inspect the persistent vault without applying it.
        // Reports: file validity, version, struct size, key fields, and a summary
        // of whether the persistent vault matches the current RAM state.
        // Heap, not BSS: NexusVault is ~2.35 KB at v42 (was ~1.5 KB at v30).
        // Lazily allocated on first VAULT/VDF call. Same pattern as saveVault()
        // and loadVault() elsewhere in this file. Never freed during normal
        // operation; costs ~2.35 KB heap only if you actually call VST/VDF.
        static NexusVault *v = nullptr;
        if (!v) {
            v = new NexusVault();
            // Guard against OOM: ESP-IDF's default newlib new returns
            // nullptr rather than throwing. Without this, the memset below
            // would dereference null. On OOM we surface the failure to the
            // operator via replyMsg and bail; the trailing dispatch will
            // route the message back via replyId.
            if (!v) {
                snprintf(replyMsg, sizeof(replyMsg),
                         "JT VAULT: OOM allocating scratch vault (%u B)",
                         (unsigned)sizeof(NexusVault));
                return;
            }
            // Belt and suspenders: readVaultForDisplay does not memset the
            // output struct, and the heap is not zero-initialised like BSS
            // was. If the on-disk file is smaller than sizeof(NexusVault),
            // the trailing bytes would otherwise be uninitialised heap junk
            // and would leak into the VDF comparator / VST display.
            memset(v, 0, sizeof(NexusVault));
        }
        const char *reason = "unknown";
        bool valid = readVaultForDisplay(v, &reason);
        if (!valid) {
            snprintf(replyMsg, sizeof(replyMsg),
                     "JT VAULT:\nState: INVALID (%s)\nFile: /nexus.bin missing or unreadable",
                     reason);
        } else {
            // Field summary (subset of what's persisted)
            uint8_t groupsUsed = 0;
            for (int i = 0; i < 4; i++) if (v->groups[i][0] != '\0') groupsUsed++;
            uint8_t aliasesUsed = 0;
            for (int i = 0; i < 16; i++) if (v->aliases[i].alias[0] != '\0') aliasesUsed++;
            uint8_t macrosUsed = 0;
            for (int i = 0; i < 4; i++) if (v->macros[i].command[0] != '\0') macrosUsed++;
            uint8_t cronUsed = 0;
            for (int i = 0; i < 8; i++) if (v->cron_table[i].active) cronUsed++;
            snprintf(replyMsg, sizeof(replyMsg),
                     "JT VAULT:\nVer:v%u Size:%uB\nDrop:0x%08lX%08lX\nOutDr:0x%08lX%08lX\nInv:0x%08X\nGroups:%u/4\nAliases:%u/16\nMacros:%u/4\nCron:%u/8",
                     (unsigned int)v->version, (unsigned int)sizeof(NexusVault),
                     (unsigned long)(v->drop_mask >> 32), (unsigned long)(v->drop_mask & 0xFFFFFFFFu),
                     (unsigned long)(v->out_drop_mask >> 32), (unsigned long)(v->out_drop_mask & 0xFFFFFFFFu),
                     (unsigned int)v->invalid_mask,
                     (unsigned int)groupsUsed, (unsigned int)aliasesUsed,
                     (unsigned int)macrosUsed, (unsigned int)cronUsed);
        }
    }
    else if (strncasecmp(cmd, "VAULT DIFF", 10) == 0 || strcasecmp(cmd, "VDF") == 0) {
        // /nexus VAULT DIFF - compare the persistent vault to the current RAM
        // state. Reports which fields differ so the operator knows what
        // /nexus SAVE will actually persist.
        // Heap, not BSS: see VST block above for rationale (saves ~2.35 KB of
        // BSS, costs same in heap only if VST/VDF is ever called).
        static NexusVault *v = nullptr;
        if (!v) {
            v = new NexusVault();
            // Guard against OOM: see VST block above. Same pattern: surface
            // the failure via replyMsg and bail; the trailing dispatch will
            // route it back to the operator.
            if (!v) {
                snprintf(replyMsg, sizeof(replyMsg),
                         "JT VAULT DIFF: OOM allocating scratch vault (%u B)",
                         (unsigned)sizeof(NexusVault));
                return;
            }
            memset(v, 0, sizeof(NexusVault));
        }
        const char *reason = "unknown";
        bool valid = readVaultForDisplay(v, &reason);
        if (!valid) {
            snprintf(replyMsg, sizeof(replyMsg),
                     "JT VAULT DIFF:\nNo valid vault (%s) - any RAM state will be saved on next /nexus SAVE",
                     reason);
        } else {
            // Build a paginated diff. We compare every persisted field in
            // NexusVault (v43) against its RAM counterpart so the operator
            // can see exactly what /nexus SAVE will change. AirTag fields
            // are intentionally skipped: saveVault() never writes them, so
            // diffing would produce a constant spurious mismatch.
            startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "VAULT DIFF");
            int diffs = 0;

            // drop_mask (uint64) — ping_emoji_mode is a separate field now (v41).
            if ((uint64_t)getNexusDropHex() != v->drop_mask) {
                uint64_t ram_d = getNexusDropHex();
                char line[96];
                snprintf(line, sizeof(line), "drop_mask: RAM=0x%08lX%08lX Vault=0x%08lX%08lX",
                         (unsigned long)(ram_d >> 32), (unsigned long)(ram_d & 0xFFFFFFFFu),
                         (unsigned long)(v->drop_mask >> 32), (unsigned long)(v->drop_mask & 0xFFFFFFFFu));
                appendPaging(line);
                diffs++;
            }
            // out_drop_mask (uint64)
            if ((uint64_t)getNexusOutDropHex() != v->out_drop_mask) {
                uint64_t ram_od = getNexusOutDropHex();
                char line[96];
                snprintf(line, sizeof(line), "out_drop_mask: RAM=0x%08lX%08lX Vault=0x%08lX%08lX",
                         (unsigned long)(ram_od >> 32), (unsigned long)(ram_od & 0xFFFFFFFFu),
                         (unsigned long)(v->out_drop_mask >> 32), (unsigned long)(v->out_drop_mask & 0xFFFFFFFFu));
                appendPaging(line);
                diffs++;
            }
            // ping_emoji_mode (uint8) — carved out of drop_mask in v41
            if ((uint8_t)ping_emoji_mode != v->ping_emoji_mode) {
                char line[64];
                snprintf(line, sizeof(line), "ping_emoji_mode: RAM=%u Vault=%u",
                         (unsigned)ping_emoji_mode, (unsigned)v->ping_emoji_mode);
                appendPaging(line);
                diffs++;
            }
            // invalid_mask (uint32)
            uint32_t ramInv = getNexusInvalidHex();
            if (ramInv != v->invalid_mask) {
                char line[64];
                snprintf(line, sizeof(line), "invalid_mask: RAM=0x%08X Vault=0x%08X",
                         (unsigned int)ramInv, (unsigned int)v->invalid_mask);
                appendPaging(line);
                diffs++;
            }

            // ping_mask (uint16) — bitfield assembled from individual flags
            uint16_t ramPing = public_ping ? 1 : 0;
            for (int i = 0; i < 8; i++) if (ping_channels[i]) ramPing |= (1 << (i + 1));
            if (ping_rssi) ramPing |= (1 << 9);
            if (ping_snr) ramPing |= (1 << 10);
            if (ping_freqerr) ramPing |= (1 << 11);
            if (ping_bitmask) ramPing |= (1 << 12);
            if (ping_ignore_direct) ramPing |= (1 << 13);
            if (ping_ignore_remote) ramPing |= (1 << 14);
            if (ping_allow_dm) ramPing |= (1 << 15);
            if (ramPing != v->ping_mask) {
                char line[64];
                snprintf(line, sizeof(line), "ping_mask: RAM=0x%04X Vault=0x%04X",
                         (unsigned)ramPing, (unsigned)v->ping_mask);
                appendPaging(line);
                diffs++;
            }

            // hop_mask (active+start+limit combined)
            uint8_t ramHopAct = hop_mask_active ? 1 : 0;
            if (ramHopAct != v->hop_mask_active ||
                hop_mask_start != v->hop_mask_start ||
                hop_mask_limit != v->hop_mask_limit) {
                char line[64];
                snprintf(line, sizeof(line), "hop_mask: RAM=%d/%d/%d Vault=%d/%d/%d",
                         ramHopAct, hop_mask_start, hop_mask_limit,
                         v->hop_mask_active, v->hop_mask_start, v->hop_mask_limit);
                appendPaging(line);
                diffs++;
            }

            // M_S (int8)
            if ((int8_t)M_S != v->M_S) {
                char line[64];
                const char *ramRole = (M_S == -1) ? "OFF" : getRoleName(M_S);
                const char *vaultRole = (v->M_S == -1) ? "OFF" : getRoleName(v->M_S);
                snprintf(line, sizeof(line), "M_S: RAM=%s Vault=%s", ramRole, vaultRole);
                appendPaging(line);
                diffs++;
            }

            // Spoof / mask flags
            if ((uint8_t)client_always != v->client_always) {
                char line[48];
                snprintf(line, sizeof(line), "client_always: RAM=%d Vault=%d",
                         (int)client_always, (int)v->client_always);
                appendPaging(line);
                diffs++;
            }
            if ((uint8_t)base_texts != v->base_texts) {
                char line[48];
                snprintf(line, sizeof(line), "base_texts: RAM=%d Vault=%d",
                         (int)base_texts, (int)v->base_texts);
                appendPaging(line);
                diffs++;
            }
            if ((uint8_t)R_S_mode != v->R_S_mode ||
                (uint8_t)R_S_id != v->R_S_id) {
                char line[64];
                snprintf(line, sizeof(line), "R_S: RAM=%d/%02X Vault=%d/%02X",
                         (int)R_S_mode, (unsigned)R_S_id,
                         (int)v->R_S_mode, (unsigned)v->R_S_id);
                appendPaging(line);
                diffs++;
            }
            if ((uint8_t)T_S_mode != v->T_S_mode ||
                T_S_id != v->T_S_id) {
                char line[80];
                snprintf(line, sizeof(line), "T_S: RAM=%d/%08X Vault=%d/%08X",
                         (int)T_S_mode, (unsigned)T_S_id,
                         (int)v->T_S_mode, (unsigned)v->T_S_id);
                appendPaging(line);
                diffs++;
            }
            if ((uint8_t)v11_mode != v->v11_mode ||
                v11_id != v->v11_id) {
                char line[80];
                snprintf(line, sizeof(line), "v11: RAM=%d/!%08x Vault=%d/!%08x",
                         (int)v11_mode, (unsigned)v11_id,
                         (int)v->v11_mode, (unsigned)v->v11_id);
                appendPaging(line);
                diffs++;
            }
            if ((uint8_t)dm_forward != v->dm_forward) {
                char line[48];
                snprintf(line, sizeof(line), "dm_forward: RAM=%d Vault=%d",
                         (int)dm_forward, (int)v->dm_forward);
                appendPaging(line);
                diffs++;
            }

            // Alerts (mode+alert combined)
            if ((uint8_t)trace_alert != v->trace_alert ||
                (uint8_t)trace_alert_mode != v->trace_alert_mode) {
                char line[64];
                snprintf(line, sizeof(line), "trace_alert: RAM=%d/%d Vault=%d/%d",
                         (int)trace_alert, (int)trace_alert_mode,
                         (int)v->trace_alert, (int)v->trace_alert_mode);
                appendPaging(line);
                diffs++;
            }
            if ((uint8_t)meta_alert != v->meta_alert ||
                (uint8_t)meta_alert_mode != v->meta_alert_mode) {
                char line[64];
                snprintf(line, sizeof(line), "meta_alert: RAM=%d/%d Vault=%d/%d",
                         (int)meta_alert, (int)meta_alert_mode,
                         (int)v->meta_alert, (int)v->meta_alert_mode);
                appendPaging(line);
                diffs++;
            }
            // skip_skip_handle (uint8)
            if ((uint8_t)(skip_skip_handle ? 1 : 0) != v->skip_skip_handle) {
                char line[64];
                snprintf(line, sizeof(line), "skip_skip_handle: RAM=%d Vault=%d",
                         skip_skip_handle ? 1 : 0, (int)v->skip_skip_handle);
                appendPaging(line);
                diffs++;
            }

            // secondary_keys (4 × 32B) — encrypted channel PSKs (v65: 4 slots)
            for (int i = 0; i < 4; i++) {
                if (memcmp(nexus_secondary_keys[i], v->secondary_keys[i], 32) != 0) {
                    char line[64];
                    snprintf(line, sizeof(line), "secondary_keys[%d]: differs (32 B PSK)", i);
                    appendPaging(line);
                    diffs++;
                }
            }

            // ch_whitelist / ch_blacklist (16 × uint32 each)
            for (int i = 0; i < 16; i++) {
                if (nexus_ch_whitelist[i] != v->ch_whitelist[i]) {
                    char line[96];
                    snprintf(line, sizeof(line),
                             "ch_whitelist[%d]: RAM=0x%08X Vault=0x%08X",
                             i, (unsigned)nexus_ch_whitelist[i], (unsigned)v->ch_whitelist[i]);
                    appendPaging(line);
                    diffs++;
                }
                if (nexus_ch_blacklist[i] != v->ch_blacklist[i]) {
                    char line[96];
                    snprintf(line, sizeof(line),
                             "ch_blacklist[%d]: RAM=0x%08X Vault=0x%08X",
                             i, (unsigned)nexus_ch_blacklist[i], (unsigned)v->ch_blacklist[i]);
                    appendPaging(line);
                    diffs++;
                }
            }

            // Groups (4 × 8B strings)
            for (int i = 0; i < 4; i++) {
                if (strncmp(nexus_groups[i], v->groups[i], 8) != 0) {
                    char line[80];
                    snprintf(line, sizeof(line), "group[%d]: RAM='%.8s' Vault='%.8s'",
                             i, nexus_groups[i], v->groups[i]);
                    appendPaging(line);
                    diffs++;
                }
            }

            // cron_table (8 × NexusCronEntry) — meaningful field tuple per slot
            for (int i = 0; i < 8; i++) {
                bool ramActive = cron_table[i].active != 0;
                bool vaultActive = v->cron_table[i].active != 0;
                if (ramActive != vaultActive ||
                    cron_table[i].mode != v->cron_table[i].mode ||
                    cron_table[i].value != v->cron_table[i].value ||
                    strncmp(cron_table[i].command, v->cron_table[i].command, 64) != 0) {
                    char line[160];
                    snprintf(line, sizeof(line),
                             "cron[%d]: RAM=%d/m%d/v%u/cmd='%.32s' Vault=%d/m%d/v%u/cmd='%.32s'",
                             i,
                             ramActive ? 1 : 0, (int)cron_table[i].mode, (unsigned)cron_table[i].value,
                             cron_table[i].command,
                             vaultActive ? 1 : 0, (int)v->cron_table[i].mode, (unsigned)v->cron_table[i].value,
                             v->cron_table[i].command);
                    appendPaging(line);
                    diffs++;
                }
            }

            // custom_pping_active + pping_header (string content)
            if ((uint8_t)(custom_pping_active ? 1 : 0) != v->custom_pping_active) {
                char line[64];
                snprintf(line, sizeof(line), "custom_pping_active: RAM=%d Vault=%d",
                         custom_pping_active ? 1 : 0, (int)v->custom_pping_active);
                appendPaging(line);
                diffs++;
            }
            if (strncmp(pping_header, v->pping_header, 64) != 0) {
                char line[80];
                snprintf(line, sizeof(line), "pping_header: RAM='%.32s' Vault='%.32s'",
                         pping_header, v->pping_header);
                appendPaging(line);
                diffs++;
            }

            // trusted_ch (2 slots, active+name)
            for (int i = 0; i < 2; i++) {
                uint8_t ramA = trusted_ch_active[i] ? 1 : 0;
                if (ramA != v->trusted_ch_active[i] ||
                    strncmp(trusted_ch_names[i], v->trusted_ch_names[i], 16) != 0) {
                    char line[96];
                    snprintf(line, sizeof(line),
                             "trusted_ch[%d]: RAM=%d/'%.16s' Vault=%d/'%.16s'",
                             i, (int)ramA, trusted_ch_names[i],
                             (int)v->trusted_ch_active[i], v->trusted_ch_names[i]);
                    appendPaging(line);
                    diffs++;
                }
            }

            // nexus_ignore_count + nexus_ignore_list (compare used range)
            if ((uint16_t)nexus_ignore_count != v->nexus_ignore_count) {
                char line[64];
                snprintf(line, sizeof(line), "nexus_ignore_count: RAM=%u Vault=%u",
                         (unsigned)nexus_ignore_count, (unsigned)v->nexus_ignore_count);
                appendPaging(line);
                diffs++;
            }
            // Compare against max( RAM count, vault count ) so trailing
            // garbage in either side is surfaced.
            uint16_t cmpCount = nexus_ignore_count;
            if (v->nexus_ignore_count > cmpCount) cmpCount = v->nexus_ignore_count;
            if (cmpCount > 64) cmpCount = 64;
            for (int i = 0; i < cmpCount; i++) {
                if (nexus_ignore_list[i] != v->nexus_ignore_list[i]) {
                    char line[96];
                    snprintf(line, sizeof(line),
                             "nexus_ignore_list[%d]: RAM=0x%08X Vault=0x%08X",
                             i, (unsigned)nexus_ignore_list[i], (unsigned)v->nexus_ignore_list[i]);
                    appendPaging(line);
                    diffs++;
                }
            }

            // zero_hop_config + zero_hop_mask[32] (256 bits)
            if ((uint8_t)zero_hop_config != v->zero_hop_config) {
                char line[64];
                snprintf(line, sizeof(line), "zero_hop_config: RAM=0x%02X Vault=0x%02X",
                         (unsigned)zero_hop_config, (unsigned)v->zero_hop_config);
                appendPaging(line);
                diffs++;
            }
            if (memcmp(zero_hop_mask, v->zero_hop_mask, 32) != 0) {
                char line[80];
                snprintf(line, sizeof(line),
                         "zero_hop_mask[32]: differs (256-bit bitmap)");
                appendPaging(line);
                diffs++;
            }

            // relay_drop_mask[32] (256 bits)
            if (memcmp(relay_drop_mask, v->relay_drop_mask, 32) != 0) {
                appendPaging("relay_drop_mask[32]: differs (256-bit bitmap)");
                diffs++;
            }
            // ignore_dupe_relay_mask[32] (256 bits)
            if (memcmp(ignore_dupe_relay_mask, v->ignore_dupe_relay_mask, 32) != 0) {
                appendPaging("ignore_dupe_relay_mask[32]: differs (256-bit bitmap)");
                diffs++;
            }

            // mqtt_override_mode (uint8)
            if ((uint8_t)mqtt_override_mode != v->mqtt_override_mode) {
                char line[64];
                snprintf(line, sizeof(line), "mqtt_override_mode: RAM=%u Vault=%u",
                         (unsigned)mqtt_override_mode, (unsigned)v->mqtt_override_mode);
                appendPaging(line);
                diffs++;
            }

            // ina_legacy_math (uint8)
            if ((uint8_t)(ina_legacy_math ? 1 : 0) != v->ina_legacy_math) {
                char line[64];
                snprintf(line, sizeof(line), "ina_legacy_math: RAM=%d Vault=%d",
                         ina_legacy_math ? 1 : 0, (int)v->ina_legacy_math);
                appendPaging(line);
                diffs++;
            }

            // B_S_active + B_S_offset (float)
            if ((uint8_t)(B_S_active ? 1 : 0) != v->B_S_active) {
                char line[64];
                snprintf(line, sizeof(line), "B_S_active: RAM=%d Vault=%d",
                         B_S_active ? 1 : 0, (int)v->B_S_active);
                appendPaging(line);
                diffs++;
            }
            if (B_S_offset != v->B_S_offset) {
                char line[80];
                snprintf(line, sizeof(line), "B_S_offset: RAM=%.2f Vault=%.2f",
                         (double)B_S_offset, (double)v->B_S_offset);
                appendPaging(line);
                diffs++;
            }

            // Rate limits (7 × uint16)
            {
                uint16_t ramRL[7] = {
                    rl_text_limit, rl_trace_limit, rl_other_limit,
                    rl_nodeinfo_limit, rl_position_limit, rl_telemetry_limit, rl_private_limit
                };
                uint16_t vaultRL[7] = {
                    v->rl_text_limit, v->rl_trace_limit, v->rl_other_limit,
                    v->rl_nodeinfo_limit, v->rl_position_limit, v->rl_telemetry_limit, v->rl_private_limit
                };
                const char *names[7] = {
                    "rl_text", "rl_trace", "rl_other",
                    "rl_nodeinfo", "rl_position", "rl_telemetry", "rl_private"
                };
                for (int i = 0; i < 7; i++) {
                    if (ramRL[i] != vaultRL[i]) {
                        char line[64];
                        snprintf(line, sizeof(line), "%s_limit: RAM=%u Vault=%u",
                                 names[i], (unsigned)ramRL[i], (unsigned)vaultRL[i]);
                        appendPaging(line);
                        diffs++;
                    }
                }
            }

            // aliases (16 × NexusAliasEntry { nodeID, alias[5] })
            for (int i = 0; i < 16; i++) {
                if (nexus_aliases[i].nodeID != v->aliases[i].nodeID ||
                    strncmp(nexus_aliases[i].alias, v->aliases[i].alias, 5) != 0) {
                    char line[96];
                    snprintf(line, sizeof(line),
                             "alias[%d]: RAM=!%08x/'%s' Vault=!%08x/'%s'",
                             i,
                             (unsigned)nexus_aliases[i].nodeID, nexus_aliases[i].alias,
                             (unsigned)v->aliases[i].nodeID, v->aliases[i].alias);
                    appendPaging(line);
                    diffs++;
                }
            }

            // macros (4 × NexusMacroEntry { name[12], command[128] })
            for (int i = 0; i < 4; i++) {
                if (strncmp(nexus_macros[i].name, v->macros[i].name, 12) != 0 ||
                    strncmp(nexus_macros[i].command, v->macros[i].command, 128) != 0) {
                    char line[160];
                    snprintf(line, sizeof(line),
                             "macro[%d]: RAM='%.12s'/'%.32s' Vault='%.12s'/'%.32s'",
                             i, nexus_macros[i].name, nexus_macros[i].command,
                             v->macros[i].name, v->macros[i].command);
                    appendPaging(line);
                    diffs++;
                }
            }

            // per_ch_hopmask (8 × NexusHopMask { active, start, limit })
            for (int i = 0; i < 8; i++) {
                uint8_t ramA = nexus_per_ch_hopmask[i].active ? 1 : 0;
                if (ramA != (v->per_ch_hopmask[i].active ? 1 : 0) ||
                    nexus_per_ch_hopmask[i].start != v->per_ch_hopmask[i].start ||
                    nexus_per_ch_hopmask[i].limit != v->per_ch_hopmask[i].limit) {
                    char line[96];
                    snprintf(line, sizeof(line),
                             "per_ch_hopmask[%d]: RAM=%d/%u/%u Vault=%d/%u/%u",
                             i, (int)ramA,
                             (unsigned)nexus_per_ch_hopmask[i].start,
                             (unsigned)nexus_per_ch_hopmask[i].limit,
                             (int)(v->per_ch_hopmask[i].active ? 1 : 0),
                             (unsigned)v->per_ch_hopmask[i].start,
                             (unsigned)v->per_ch_hopmask[i].limit);
                    appendPaging(line);
                    diffs++;
                }
            }
            // per_port_hopmask (5 × NexusHopMask { active, start, limit })
            // Indices: 0=NI, 1=TL, 2=PO, 3=TR, 4=TX (matches code sites).
            {
                const char *portNames[5] = { "NI", "TL", "PO", "TR", "TX" };
                for (int i = 0; i < 5; i++) {
                    uint8_t ramA = nexus_per_port_hopmask[i].active ? 1 : 0;
                    if (ramA != (v->per_port_hopmask[i].active ? 1 : 0) ||
                        nexus_per_port_hopmask[i].start != v->per_port_hopmask[i].start ||
                        nexus_per_port_hopmask[i].limit != v->per_port_hopmask[i].limit) {
                        char line[96];
                        snprintf(line, sizeof(line),
                                 "per_port_hopmask[%d/%s]: RAM=%d/%u/%u Vault=%d/%u/%u",
                                 i, portNames[i], (int)ramA,
                                 (unsigned)nexus_per_port_hopmask[i].start,
                                 (unsigned)nexus_per_port_hopmask[i].limit,
                                 (int)(v->per_port_hopmask[i].active ? 1 : 0),
                                 (unsigned)v->per_port_hopmask[i].start,
                                 (unsigned)v->per_port_hopmask[i].limit);
                        appendPaging(line);
                        diffs++;
                    }
                }
            }

            // t_m (uint8)
            if ((uint8_t)t_m != v->t_m) {
                char line[64];
                snprintf(line, sizeof(line), "t_m: RAM=%u Vault=%u",
                         (unsigned)t_m, (unsigned)v->t_m);
                appendPaging(line);
                diffs++;
            }

            // s_o (active + float value)
            if ((uint8_t)(s_o_active ? 1 : 0) != v->s_o_active) {
                char line[64];
                snprintf(line, sizeof(line), "s_o_active: RAM=%d Vault=%d",
                         s_o_active ? 1 : 0, (int)v->s_o_active);
                appendPaging(line);
                diffs++;
            }
            if (s_o_value != v->s_o_value) {
                char line[80];
                snprintf(line, sizeof(line), "s_o_value: RAM=%.1f Vault=%.1f",
                         (double)s_o_value, (double)v->s_o_value);
                appendPaging(line);
                diffs++;
            }

            // preamble_override (uint16)
            if ((uint16_t)preamble_override != v->preamble_override) {
                char line[64];
                snprintf(line, sizeof(line), "preamble_override: RAM=%u Vault=%u",
                         (unsigned)preamble_override, (unsigned)v->preamble_override);
                appendPaging(line);
                diffs++;
            }
            // syncword_override (uint8)
            if ((uint8_t)syncword_override != v->syncword_override) {
                char line[64];
                snprintf(line, sizeof(line), "syncword_override: RAM=0x%02X Vault=0x%02X",
                         (unsigned)syncword_override, (unsigned)v->syncword_override);
                appendPaging(line);
                diffs++;
            }

            // gpio_watch_mask (uint64) — loadVault is default-off, so any
            // saved bits will be discarded on next boot. Always surface.
            if ((uint64_t)gpio_watch_mask != v->gpio_watch_mask) {
                uint64_t ramG = gpio_watch_mask;
                char line[96];
                if (v->gpio_watch_mask != 0 && gpio_watch_mask == 0) {
                    // Vault held bits, RAM is 0: loadVault discarded them
                    // (default-off directive). This is the operator's
                    // signal that the saved value won't survive reboot.
                    snprintf(line, sizeof(line),
                             "gpio_watch_mask: RAM=0x%08lX%08lX Vault=0x%08lX%08lX (saved bits DISCARDED on load)",
                             (unsigned long)(ramG >> 32), (unsigned long)(ramG & 0xFFFFFFFFu),
                             (unsigned long)(v->gpio_watch_mask >> 32),
                             (unsigned long)(v->gpio_watch_mask & 0xFFFFFFFFu));
                } else {
                    snprintf(line, sizeof(line),
                             "gpio_watch_mask: RAM=0x%08lX%08lX Vault=0x%08lX%08lX",
                             (unsigned long)(ramG >> 32), (unsigned long)(ramG & 0xFFFFFFFFu),
                             (unsigned long)(v->gpio_watch_mask >> 32),
                             (unsigned long)(v->gpio_watch_mask & 0xFFFFFFFFu));
                }
                appendPaging(line);
                diffs++;
            }

            // signatures (8 × NexusSignature { bytes[16], len, type })
            for (int i = 0; i < 8; i++) {
                if (nexus_signatures[i].len != v->signatures[i].len ||
                    nexus_signatures[i].type != v->signatures[i].type ||
                    memcmp(nexus_signatures[i].bytes, v->signatures[i].bytes, 16) != 0) {
                    char line[96];
                    snprintf(line, sizeof(line),
                             "signature[%d]: RAM=len%u/type%u Vault=len%u/type%u",
                             i, (unsigned)nexus_signatures[i].len, (unsigned)nexus_signatures[i].type,
                             (unsigned)v->signatures[i].len, (unsigned)v->signatures[i].type);
                    appendPaging(line);
                    diffs++;
                }
            }

            // freeze_flags (v64: bit0=NodeDB, bit1=Warm, bit2=Device, bit3=Config)
            {
                uint8_t ramF = 0;
                if (freeze_nodedb) ramF |= 0x01;
                if (freeze_warm)   ramF |= 0x02;
                if (freeze_device) ramF |= 0x04;
                if (freeze_config) ramF |= 0x08;
                if (ramF != v->freeze_flags) {
                    char line[96];
                    snprintf(line, sizeof(line),
                             "freeze_flags: RAM=0x%02X (nodedb=%d warm=%d device=%d config=%d) Vault=0x%02X",
                             (unsigned)ramF, freeze_nodedb ? 1 : 0, freeze_warm ? 1 : 0,
                             freeze_device ? 1 : 0, freeze_config ? 1 : 0,
                             (unsigned)v->freeze_flags);
                    appendPaging(line);
                    diffs++;
                }
            }

            // broadcast_delay_max_s (uint16)
            if ((uint16_t)broadcast_delay_max_s != v->broadcast_delay_max_s) {
                char line[64];
                snprintf(line, sizeof(line), "broadcast_delay_max_s: RAM=%u Vault=%u",
                         (unsigned)broadcast_delay_max_s, (unsigned)v->broadcast_delay_max_s);
                appendPaging(line);
                diffs++;
            }

            // Summary footer
            char summary[64];
            if (diffs == 0) {
                snprintf(summary, sizeof(summary), "✓ no diffs (vault mirrors RAM)");
            } else {
                snprintf(summary, sizeof(summary), "%d field(s) differ", diffs);
            }
            // Paging is FIFO; the diff lines were already pushed via
            // appendPaging above. Summary goes last so the operator sees
            // the per-field list before the count.
            appendPaging(summary);
            return;
        }
    }
    else if (strcasecmp(cmd, "LORA") == 0 || strcasecmp(cmd, "LR") == 0) {
        snprintf(replyMsg, sizeof(replyMsg),
                 "JT LORA:\nSF:%u BW:%u CR:%u\nPreset:%u (%s)\nFreq:%.4fMHz\nPower:%ddBm\nIgnoreMQTT:%d\nRebroadcast:%s",
                 config.lora.spread_factor, config.lora.bandwidth, config.lora.coding_rate,
                 config.lora.modem_preset, config.lora.use_preset ? "ACTIVE" : "OFF",
                 config.lora.override_frequency, (int)config.lora.tx_power,
                 config.lora.ignore_mqtt, getRebroadcastModeName(config.device.rebroadcast_mode));
    }
    else if (strncasecmp(cmd, "IGNORMQTT ", 10) == 0 || strcasecmp(cmd, "IGNORMQTT") == 0 ||
             strncasecmp(cmd, "IGNOREMQTT ", 11) == 0 || strcasecmp(cmd, "IGNOREMQTT") == 0) {
        bool isShort = (strncasecmp(cmd, "IGNORMQTT", 9) == 0);
        const char *p = (isShort) ? ((strlen(cmd) > 10) ? cmd + 10 : "") : ((strlen(cmd) > 11) ? cmd + 11 : "");
        while (*p == ' ') p++;

        if (*p == '\0') {
            snprintf(replyMsg, sizeof(replyMsg), "JT: LoRa Ignore MQTT is %s", config.lora.ignore_mqtt ? "ON" : "OFF");
        } else {
            bool enable = (strcasecmp(p, "ON") == 0 || strcasecmp(p, "1") == 0);
            meshtastic_Config c = meshtastic_Config_init_default;
            c.which_payload_variant = meshtastic_Config_lora_tag;
            c.payload_variant.lora = config.lora;
            c.payload_variant.lora.ignore_mqtt = enable;

            if (adminModule) adminModule->handleSetConfig(c, false);
            snprintf(replyMsg, sizeof(replyMsg), "JT: LoRa Ignore MQTT set to %s", enable ? "ON" : "OFF");
        }
    }
    else if (strncasecmp(cmd, "SETREBROADCAST ", 15) == 0 || strcasecmp(cmd, "REBROADCAST") == 0) {
        const char *p = (strlen(cmd) > 15) ? cmd + 15 : "";
        while (*p == ' ') p++;

        if (*p == '\0') {
            snprintf(replyMsg, sizeof(replyMsg), "JT: Rebroadcast Mode is %s (%u)",
                     getRebroadcastModeName(config.device.rebroadcast_mode), (uint32_t)config.device.rebroadcast_mode);
        } else {
            int mode = -1;
            if (strcasecmp(p, "ALL") == 0) mode = meshtastic_Config_DeviceConfig_RebroadcastMode_ALL;
            else if (strcasecmp(p, "ALL_SKIP") == 0) mode = meshtastic_Config_DeviceConfig_RebroadcastMode_ALL_SKIP_DECODING;
            else if (strcasecmp(p, "LOCAL") == 0 || strcasecmp(p, "LOCAL_ONLY") == 0) mode = meshtastic_Config_DeviceConfig_RebroadcastMode_LOCAL_ONLY;
            else if (strcasecmp(p, "KNOWN") == 0 || strcasecmp(p, "KNOWN_ONLY") == 0) mode = meshtastic_Config_DeviceConfig_RebroadcastMode_KNOWN_ONLY;
            else if (strcasecmp(p, "NONE") == 0) mode = meshtastic_Config_DeviceConfig_RebroadcastMode_NONE;
            else if (strcasecmp(p, "CORE") == 0 || strcasecmp(p, "CORE_ONLY") == 0) mode = meshtastic_Config_DeviceConfig_RebroadcastMode_CORE_PORTNUMS_ONLY;
            else if (isdigit(p[0])) mode = atoi(p);

            if (mode >= 0 && mode <= 5) {
                meshtastic_Config c = meshtastic_Config_init_default;
                c.which_payload_variant = meshtastic_Config_device_tag;
                c.payload_variant.device = config.device;
                c.payload_variant.device.rebroadcast_mode = (meshtastic_Config_DeviceConfig_RebroadcastMode)mode;

                if (adminModule) adminModule->handleSetConfig(c, false);
                snprintf(replyMsg, sizeof(replyMsg), "JT: Rebroadcast Mode set to %s (%d)", getRebroadcastModeName(mode), mode);
            } else {
                snprintf(replyMsg, sizeof(replyMsg), "JT: Invalid Mode (ALL|ALL_SKIP|LOCAL|KNOWN|NONE|CORE)");
            }
        }
    }
    else if (strncasecmp(cmd, "PRESET ", 7) == 0 || strncasecmp(cmd, "SETPRESET ", 10) == 0) {
        const char *p = (strncasecmp(cmd, "PRESET ", 7) == 0) ? cmd + 7 : cmd + 10;
        while (*p == ' ') p++;

        struct LoRaParams { uint8_t sf; uint16_t bw; uint8_t cr; float freq; };
        LoRaParams target = {0, 0, 0, 0.0f};
        bool found = true;

        // 1. Official Meshtastic Aliases (Aligned with Docs)
        if (strcasecmp(p, "LF") == 0 || strcasecmp(p, "LONGFAST") == 0) { target = {11, 250, 5, 869.525f}; }
        else if (strcasecmp(p, "LT") == 0 || strcasecmp(p, "LONGTURBO") == 0) { target = {11, 500, 8, 869.525f}; }
        else if (strcasecmp(p, "LM") == 0 || strcasecmp(p, "LONGMODERATE") == 0) { target = {11, 125, 8, 869.525f}; }
        else if (strcasecmp(p, "LS") == 0 || strcasecmp(p, "LONGSLOW") == 0) { target = {12, 125, 8, 869.525f}; }
        else if (strcasecmp(p, "MF") == 0 || strcasecmp(p, "MEDFAST") == 0) { target = {9, 250, 5, 869.525f}; }
        else if (strcasecmp(p, "MS") == 0 || strcasecmp(p, "MEDSLOW") == 0) { target = {10, 250, 5, 869.525f}; }
        else if (strcasecmp(p, "SS") == 0 || strcasecmp(p, "SHORTSLOW") == 0) { target = {8, 250, 5, 869.525f}; }
        else if (strcasecmp(p, "SF") == 0 || strcasecmp(p, "SHORTFAST") == 0) { target = {7, 250, 5, 869.525f}; }
        else if (strcasecmp(p, "ST") == 0 || strcasecmp(p, "SHORTTURBO") == 0) { target = {7, 500, 5, 869.525f}; }

        // 2. Nexus Tactical
        else if (strcasecmp(p, "P1") == 0 || strcasecmp(p, "SFNARROW") == 0) { target = {7, 62, 5, 869.618f}; }
        else if (strcasecmp(p, "P7") == 0 || strcasecmp(p, "NARROWPLUS") == 0) { target = {6, 62, 5, 869.618f}; }
        else if (strcasecmp(p, "NARROWPROMAX") == 0 || strcasecmp(p, "NARROWPREMIUM") == 0) { target = {5, 62, 5, 869.618f}; }
        else if (strcasecmp(p, "SP") == 0 || strcasecmp(p, "SHORTPLUS") == 0) { target = {6, 250, 5, 869.525f}; }
        else if (strcasecmp(p, "SPM") == 0 || strcasecmp(p, "SHORTPROMAX") == 0) { target = {5, 250, 5, 869.525f}; }

        // 3. NAP Series (BW 125)
        else if (strcasecmp(p, "NAP-1") == 0) { target = {7, 125, 5, 865.700f}; }
        else if (strcasecmp(p, "NAP-2") == 0) { target = {7, 125, 5, 866.300f}; }
        else if (strcasecmp(p, "NAP-3") == 0) { target = {7, 125, 5, 866.900f}; }
        else if (strcasecmp(p, "NAP-4") == 0) { target = {7, 125, 5, 867.500f}; }
        else if (strcasecmp(p, "NAP-5") == 0) { target = {7, 125, 5, 871.000f}; }

        // 4. NAPN Series (BW 62)
        else if (strcasecmp(p, "NAPN-1") == 0) { target = {7, 62, 5, 865.700f}; }
        else if (strcasecmp(p, "NAPN-2") == 0) { target = {7, 62, 5, 866.300f}; }
        else if (strcasecmp(p, "NAPN-3") == 0) { target = {7, 62, 5, 866.900f}; }
        else if (strcasecmp(p, "NAPN-4") == 0) { target = {7, 62, 5, 867.500f}; }
        else if (strcasecmp(p, "NAPN-5") == 0) { target = {7, 62, 5, 871.000f}; }
        else { found = false; }

        if (found) {
            memset(&pendingConfig, 0, sizeof(pendingConfig));
            pendingConfig.which_payload_variant = meshtastic_Config_lora_tag;
            pendingConfig.payload_variant.lora = config.lora;
            pendingConfig.payload_variant.lora.use_preset = false;
            pendingConfig.payload_variant.lora.spread_factor = target.sf;
            pendingConfig.payload_variant.lora.bandwidth = target.bw;
            pendingConfig.payload_variant.lora.coding_rate = target.cr;
            pendingConfig.payload_variant.lora.override_frequency = target.freq;

            snprintf(replyMsg, sizeof(replyMsg), "JT: Preset '%s' accepted. SF:%u BW:%u Freq:%.3fMHz. Applying LIVE in 5s.",
                     p, target.sf, target.bw, target.freq);
            pendingLoRaConfig = true;
            pendingLoRaApplyAt = Time::getMillis() + 5000;
        } else {
            snprintf(replyMsg, sizeof(replyMsg), "JT: Unknown Preset '%s'", p);
        }
    }
    else if (strncasecmp(cmd, "SETLORA ", 8) == 0) {
        // Nexus: Local + remote both allowed (mirrors SETPREAMBLE / SETSYNCWORD behavior).
        // Removed 2026-07-30: prior `if (mp.from == 0) reject` was inconsistent with the
        // sibling radio-reconfig commands and made CLI/serial debugging painful.
        // Format: SETLORA <SF> <BW> <CR> <FREQ_OVERRIDE>
        const char *p = cmd + 8;
        int sf = atoi(p); while (*p && *p != ' ') p++; while (*p == ' ') p++;
        int bw = atoi(p); while (*p && *p != ' ') p++; while (*p == ' ') p++;
        int cr = atoi(p); while (*p && *p != ' ') p++; while (*p == ' ') p++;
        float freq = atof(p);

        // Hardware Bounds Checks (Crash Prevention)
        bool sfOk = (sf >= 5 && sf <= 12);
        bool crOk = (cr >= 5 && cr <= 8);
        // Bandwidth validity depends on band:
        //   - Sub-GHz (SX126x, LR11x0 low-band, RF95):     31, 62, 125, 250, 500 kHz
        //   - 2.4 GHz  (SX128x, LR11x0 high-band, LR2021):  200, 400, 800, 1600 (= 203, 406, 812, 1625 kHz real)
        // BW codes are the Meshtastic proto codes (bwKHzToCode), not the real kHz values,
        // matching the existing sub-GHz convention (31/62/125/250/500).
        bool freqOk = (freq >= 863.0f && freq <= 873.0f) ||
                      (freq >= 2400.0f && freq <= 2500.0f);
        bool bwOk;
        if (freq >= 2400.0f && freq <= 2500.0f) {
            bwOk = (bw == 200 || bw == 400 || bw == 800 || bw == 1600);
        } else {
            bwOk = (bw == 31 || bw == 62 || bw == 125 || bw == 250 || bw == 500);
        }

        if (sfOk && crOk && bwOk && freqOk) {
            memset(&pendingConfig, 0, sizeof(pendingConfig));
            pendingConfig.which_payload_variant = meshtastic_Config_lora_tag;
            pendingConfig.payload_variant.lora = config.lora;

            pendingConfig.payload_variant.lora.use_preset = false;
            pendingConfig.payload_variant.lora.spread_factor = sf;
            pendingConfig.payload_variant.lora.bandwidth = bw;
            pendingConfig.payload_variant.lora.coding_rate = cr;
            pendingConfig.payload_variant.lora.override_frequency = freq;

            snprintf(replyMsg, sizeof(replyMsg), "JT: LoRa accepted. SF:%u BW:%u CR:%u Freq:%.4fMHz. Applying LIVE in 5s.", sf, bw, cr, freq);
            pendingLoRaConfig = true;
            pendingLoRaApplyAt = Time::getMillis() + 5000;
        } else {
            if (freq >= 2400.0f && freq <= 2500.0f) {
                snprintf(replyMsg, sizeof(replyMsg), "JT: Invalid 2.4GHz params (SF:5-12, CR:5-8, BW:200|400|800|1600, Freq:2400-2500)");
            } else {
                snprintf(replyMsg, sizeof(replyMsg), "JT: Invalid sub-GHz params (SF:5-12, CR:5-8, BW:31|62|125|250|500, Freq:863-873)");
            }
        }
    }
    else if (strncasecmp(cmd, "SETFSK ", 7) == 0 || strcasecmp(cmd, "SETFSK") == 0 ||
             strncasecmp(cmd, "SETFLRC ", 8) == 0 || strcasecmp(cmd, "SETFLRC") == 0) {
        // Nexus SETFSK/SETFLRC (research 2026-10-09; Jenny rulings 2026-10-14):
        //  public, RAM-only, persist-until-disabled, deferred 5s apply. Staging is
        //  identical to SETSYNCWORD: mutate the RAM state NOW, stage an unchanged
        //  LoRa config so the deferred apply re-enters applyModemConfig() and
        //  programModemParams() picks the FSK path via altModem.mode.
        bool isFlrc = (strncasecmp(cmd, "SETFLRC", 7) == 0);
        const char *sub = skipToken(cmd);
        RadioInterface *radioIf = (router) ? router->getInterface() : nullptr;
        uint8_t caps = radioIf ? radioIf->altModemCaps() : 0;
        uint8_t need = isFlrc ? 0x02 : 0x01;

        if (strcasecmp(sub, "OFF") == 0 || strcasecmp(sub, "RESET") == 0 || strcasecmp(sub, "DEFAULT") == 0) {
            if (altModem.mode == 0) {
                snprintf(replyMsg, sizeof(replyMsg), "JT: Alt modem already off (LoRa).");
            } else {
                altModem = NexusAltModem{0, 0.0f, 0.0f, 0.0f, 0};
                pendingLoRaConfig = true;
                pendingLoRaApplyAt = Time::getMillis() + 5000;
                meshtastic_Config pc = meshtastic_Config_init_default;
                pc.which_payload_variant = meshtastic_Config_lora_tag;
                pc.payload_variant.lora = config.lora;
                pendingConfig = pc;
                setIntervalFromNow(50);
                snprintf(replyMsg, sizeof(replyMsg), "JT: Alt modem disabled (back to LoRa). Applying in 5s...");
            }
        } else if (*sub == '\0') {
            static const char *modeName[] = {"LORA", "FSK", "FLRC"};
            snprintf(replyMsg, sizeof(replyMsg), "JT Alt Modem: %s BR:%.1fk dev:%.1fk bw:%.1fk pre:%u | caps:%s%s%s",
                     modeName[altModem.mode <= 2 ? altModem.mode : 0], altModem.brKbps, altModem.freqDevKHz,
                     altModem.rxBwKHz, (unsigned)altModem.preambleBits,
                     (caps & 0x01) ? "FSK " : "", (caps & 0x02) ? "FLRC " : "", (caps == 0) ? "none" : "");
        } else {
            if (!(caps & need)) {
                snprintf(replyMsg, sizeof(replyMsg), "JT: %s not supported on this radio", isFlrc ? "SETFLRC" : "SETFSK");
            } else if (isFlrc) {
                // SETFLRC <br_kbps> <cr 2|3|4> [bt 0|1|2] [preamble_bits]
                //   cr: 2 -> 1/2, 3 -> 3/4, 4 -> uncoded 1/1 (per-chip RadioLib constants)
                //   bt: 0 -> NONE/NRZ, 1 -> BT 0.3, 2 -> BT 0.5 (default 2 = RadioLib default)
                // FLRC is family-local by physics: 2.4 GHz SX128x pairs talk to SX128x peers,
                // sub-GHz LR2021 (X1) pairs talk to LR2021 peers — params are explicit so both
                // sides of a pair always match (Jenny ruling 2026-10-14: whatever it takes).
                const char *p = sub;
                float br = atof(p); while (*p && *p != ' ') p++; while (*p == ' ') p++;
                int crc = (*p) ? atoi(p) : 0; while (*p && *p != ' ') p++; while (*p == ' ') p++;
                int bt = (*p) ? atoi(p) : 2; while (*p && *p != ' ') p++; while (*p == ' ') p++;
                int pre = (*p) ? atoi(p) : 16;
                bool brOk = (br >= 1.0f && br <= 2600.0f); // broad — chip's beginFLRC range-checks
                bool crOk = (crc == 2 || crc == 3 || crc == 4);
                bool btOk = (bt >= 0 && bt <= 2);
                bool preOk = (pre >= 4 && pre <= 1024);
                if (brOk && crOk && btOk && preOk) {
                    altModem.mode = 2;
                    altModem.brKbps = br;
                    altModem.cr = (uint8_t)crc;
                    altModem.bt = (uint8_t)bt;
                    altModem.preambleBits = (uint16_t)pre;
                    altModem.freqDevKHz = 0.0f;
                    altModem.rxBwKHz = 0.0f;
                    pendingLoRaConfig = true;
                    pendingLoRaApplyAt = Time::getMillis() + 5000;
                    meshtastic_Config pc = meshtastic_Config_init_default;
                    pc.which_payload_variant = meshtastic_Config_lora_tag;
                    pc.payload_variant.lora = config.lora;
                    pendingConfig = pc;
                    setIntervalFromNow(50);
                    const char *crName = (crc == 2) ? "1/2" : (crc == 3) ? "3/4" : "1/1";
                    snprintf(replyMsg, sizeof(replyMsg),
                             "JT: FLRC accepted. BR:%.1fk CR:%s BT:%d PRE:%d. Applying LIVE in 5s.",
                             br, crName, bt, pre);
                } else {
                    snprintf(replyMsg, sizeof(replyMsg),
                             "JT: Invalid FLRC params (BR:1-2600k, CR:2|3|4 = 1/2|3/4|uncoded, BT:0|1|2, PRE:4-1024)");
                }
            } else {
                // SETFSK <br_kbps> [freqDev_kHz] [rxBw_kHz] [preamble_bits]
                // Per Carson's rule (Semtech SX1261-2 §6.2.1 / LR1110 §8.2.1), DSB receiver filter
                // bandwidth must satisfy rxBw >= br + 2*freqDev (e.g. 50 kbps + 2*25 kHz = 100 kHz -> 117.3 kHz step).
                const char *p = sub;
                float br = atof(p); while (*p && *p != ' ') p++; while (*p == ' ') p++;
                float dev = (*p) ? atof(p) : ((br >= 3.0f && br <= 300.0f) ? (br * 0.5f) : 25.0f);
                if (dev > 200.0f) dev = 200.0f;
                while (*p && *p != ' ') p++; while (*p == ' ') p++;
                float rbw = (*p) ? atof(p) : 0.0f;
                while (*p && *p != ' ') p++; while (*p == ' ') p++;
                int pre = (*p) ? atoi(p) : 16;
                float carsonMinBw = br + 2.0f * dev;
                if (rbw < carsonMinBw) {
                    static const float kSteps[] = {
                        4.8f, 5.8f, 7.3f, 9.7f, 11.7f, 14.6f, 19.5f, 23.4f, 29.3f, 39.0f,
                        46.9f, 58.6f, 78.2f, 93.8f, 117.3f, 156.2f, 187.2f, 234.3f, 312.0f, 373.6f, 467.0f
                    };
                    rbw = 467.0f;
                    for (size_t i = 0; i < sizeof(kSteps) / sizeof(kSteps[0]); ++i) {
                        if (kSteps[i] >= carsonMinBw) {
                            rbw = kSteps[i];
                            break;
                        }
                    }
                }
                bool brOk = (br >= 0.6f && br <= 300.0f);   // SX126x beginFSK range
                bool devOk = (dev >= 1.5f && dev <= 200.0f); // SX126x/GFSK deviation limit
                bool rbwOk = (rbw >= 4.8f && rbw <= 500.0f); // SX126x GFSK RX filter
                bool preOk = (pre >= 4 && pre <= 1024);
                if (brOk && devOk && rbwOk && preOk) {
                    altModem.mode = 1;
                    altModem.brKbps = br;
                    altModem.freqDevKHz = dev;
                    altModem.rxBwKHz = rbw;
                    altModem.preambleBits = (uint16_t)pre;
                    pendingLoRaConfig = true;
                    pendingLoRaApplyAt = Time::getMillis() + 5000;
                    meshtastic_Config pc = meshtastic_Config_init_default;
                    pc.which_payload_variant = meshtastic_Config_lora_tag;
                    pc.payload_variant.lora = config.lora;
                    pendingConfig = pc;
                    setIntervalFromNow(50);
                    snprintf(replyMsg, sizeof(replyMsg),
                             "JT: FSK accepted. BR:%.1fk DEV:%.1fk BW:%.1fk PRE:%d. Applying LIVE in 5s.",
                             br, dev, rbw, pre);
                } else {
                    snprintf(replyMsg, sizeof(replyMsg),
                             "JT: Invalid FSK params (BR:0.6-300k, DEV:1.5-200k, BW:4.8-500k, PRE:4-1024)");
                }
            }
        }
    }
    else if (strncasecmp(cmd, "SEARCHRELAY ", 12) == 0 || strncasecmp(cmd, "SRELAY ", 7) == 0) {
        const char *p = cmd + (strncasecmp(cmd, "SRELAY ", 7) == 0 ? 7 : 12);
        while (*p == ' ') p++;
        if (*p == '*') p++; // accept the documented "*A4" display form
        uint8_t suffix = (uint8_t)strtoul(p, NULL, 0);
        static char names[64];
        memset(names, 0, sizeof(names));
        getRelayNames(names, sizeof(names), suffix);
        if (names[0] != '\0') {
            snprintf(replyMsg, sizeof(replyMsg), "JT Relay *%02x Matches:\n%s", suffix, names);
        } else {
            snprintf(replyMsg, sizeof(replyMsg), "JT Relay *%02x: No matches in NodeDB", suffix);
        }
        memset(names, 0, sizeof(names));
    }
    else if (strncasecmp(cmd, "NODEDB ", 7) == 0 || strncasecmp(cmd, "DB ", 3) == 0) {
        const char *targetStr = cmd + (strncasecmp(cmd, "DB ", 3) == 0 ? 3 : 7);
        while (*targetStr == ' ') targetStr++;

        if (strlen(targetStr) < 2) {
            snprintf(replyMsg, sizeof(replyMsg), "JT: Invalid NODEDB target");
        } else {
            uint32_t targetNum = 0;
            // 1. Try to find by short name (case-insensitive)
            for (size_t i = 0; i < nodeDB->getNumMeshNodes(); i++) {
                auto n = nodeDB->getMeshNodeByIndex(i);
                if (n && nexus_has_user(n) && strcasecmp(n->short_name, targetStr) == 0) {
                    targetNum = n->num;
                    break;
                }
            }

            // 2. If not found by name, try parsing as hex
            if (targetNum == 0) {
                if (targetStr[0] == '!') targetNum = strtoul(targetStr + 1, NULL, 16);
                else {
                    char *endptr;
                    uint32_t val = strtoul(targetStr, &endptr, 16);
                    if (*endptr == '\0' || *endptr == ' ') targetNum = val;
                }
            }

            if (targetNum == 0) {
                snprintf(replyMsg, sizeof(replyMsg), "JT NODEDB: Target '%s' not found", targetStr);
            } else {
                auto node = nodeDB->getMeshNode(targetNum);
                if (node) {
                    bool isAdmin = false;
                    for(int i=0; i<3; i++) {
                        if (config.security.admin_key[i].size == 32 && memcmp(node->public_key.bytes, config.security.admin_key[i].bytes, 32) == 0)
                            isAdmin = true;
                    }
                    const char* name = (node->long_name[0] != '\0') ? node->long_name : "[No Name]";
                    const char* sname = (node->short_name[0] != '\0') ? node->short_name : "??";
                    const char* roleName = getRoleName(node->role);

                    // Nexus: Calculate metrics
                    uint32_t now_s = getTime();
                    uint32_t lastSeen = (node->last_heard != 0 && now_s >= node->last_heard) ? (now_s - node->last_heard) : 0;
                    float snr = node->snr;
                    int hops = node->has_hops_away ? node->hops_away : -1;
                    int ch = nodeDB->getMeshNodeChannel(targetNum);

                    snprintf(replyMsg, sizeof(replyMsg),
                             "JT NODEDB %08x:\nName: %.20s [%s]\nSeen: %us ago, Role:%s\nSNR: %.1f, Hop: %d, Ch: %d\nFav:%d Ign:%d Adm:%d",
                             (unsigned int)targetNum, name, sname, (unsigned int)lastSeen, roleName, snr, hops, ch, nexus_is_favorite(node), nexus_is_ignored(node), isAdmin);
                } else {
                    snprintf(replyMsg, sizeof(replyMsg), "JT NODEDB: Node %08x not found", (unsigned int)targetNum);
                }
            }
        }
    }
    else if (strncasecmp(cmd, "FAV ", 4) == 0 || strncasecmp(cmd, "UNFAV ", 6) == 0 ||
             strncasecmp(cmd, "IGNORE ", 7) == 0 || strncasecmp(cmd, "UNIGNORE ", 9) == 0 ||
             strncasecmp(cmd, "FFAV ", 5) == 0 || strncasecmp(cmd, "FUNFAV ", 7) == 0 ||
             strncasecmp(cmd, "FIGNORE ", 8) == 0 || strncasecmp(cmd, "FUNIGNORE ", 10) == 0 ||
             strncasecmp(cmd, "DELNODE ", 8) == 0) {

        bool delNode = (strncasecmp(cmd, "DELNODE ", 8) == 0);
        // Force forms are the base verb with one leading 'F':
        //   FFAV->FAV, FUNFAV->UNFAV, FIGNORE->IGNORE, FUNIGNORE->UNIGNORE.
        // (Previously FUN* kept sub=cmd so no predicate matched and FUNFAV fell
        // into the un-ignore branch - A1 in nexus-commands-audit.)
        bool force = (!delNode && (strncasecmp(cmd, "FFAV ", 5) == 0 || strncasecmp(cmd, "FUNFAV ", 7) == 0 ||
                                   strncasecmp(cmd, "FIGNORE ", 8) == 0 || strncasecmp(cmd, "FUNIGNORE ", 10) == 0));

        const char *sub = force ? cmd + 1 : cmd;
        bool setFav = !delNode && (strncasecmp(sub, "FAV ", 4) == 0);
        bool setUnfav = !delNode && (strncasecmp(sub, "UNFAV ", 6) == 0);
        bool setIgn = !delNode && (strncasecmp(sub, "IGNORE ", 7) == 0);
        // UNIGNORE (incl. FUNIGNORE) falls through to the un-ignore branch.

        // Arg pointer = end of the matched token incl. its trailing space, in
        // either the full command (non-force) or the F-stripped base (force).
        const char *p;
        if (delNode) p = cmd + 8;
        else p = sub + (setFav ? 4 : (setUnfav ? 6 : (setIgn ? 7 : 9)));

        while (*p == ' ') p++;

        // Fix: Save pointer to the start of the node name/ID for error reporting
        const char *targetStr = p;

        uint32_t targetNum = findNodeByShortNameOrHex(p);

        if (delNode) {
            if (targetNum != 0) {
                nodeDB->removeNodeByNum(targetNum);
                nodeDB->saveToDisk(SEGMENT_NODEDATABASE);
                snprintf(replyMsg, sizeof(replyMsg), "JT: Node !%08x removed from DB", (unsigned int)targetNum);
            } else {
                snprintf(replyMsg, sizeof(replyMsg), "JT: DELNODE Error - Node '%s' not found", targetStr);
            }
            // Nexus-28006 (2026-09-16) — code review 1D fix.
            // Previously: `return;` here dropped replyMsg before the
            // dispatcher tail at the end of handleNexusCommand could
            // sendToPhone / queueMeshReply it. Operator saw silence after
            // /nexus DELNODE despite replyMsg being correctly populated.
            // Drop the return; — the dispatcher tail picks up replyMsg.
            // Note: the rest of the DELNODE / FAV / UNFAV / IGNORE block
            // below this `if (delNode)` is gated by other conditions (setFav,
            // setUnfav, setIgn) that are all false when delNode is true,
            // so falling through does not run any of that logic.
        }

        if (!delNode) {
            // Fix: Only use getOrCreateMeshNode if 'force' is true (FFAV/FUNFAV/etc)
            auto node = force ? nodeDB->getOrCreateMeshNode(targetNum) : nodeDB->getMeshNode(targetNum);

            if (targetNum != 0 && node) {
                if (setFav)
                    nexus_set_favorite(node, true);
                else if (setUnfav)
                    nexus_set_favorite(node, false);
                else if (setIgn)
                    nexus_set_ignored(node, true);
                else
                    nexus_set_ignored(node, false);
                nodeDB->saveToDisk(SEGMENT_NODEDATABASE);
                snprintf(replyMsg, sizeof(replyMsg), "JT: %s node !%08x (%s)", force ? "Forced" : "Updated",
                         (unsigned int)targetNum, node->short_name[0] ? node->short_name : "UNK");
            } else {
                snprintf(replyMsg, sizeof(replyMsg), "JT: Node '%s' not found%s", targetStr,
                         force ? " (and could not create)" : "");
            }
        }
    }
    else if (strcasecmp(cmd, "RATELIMITSTAT") == 0 ||
             strncasecmp(cmd, "RATELIMITSTAT ", 13) == 0 ||
             strcasecmp(cmd, "RLS") == 0 || strncasecmp(cmd, "RLS ", 4) == 0) {
        // /nexus RATELIMITSTAT (alias RLS) - show live state of all 7 RL buckets.
        // Reports current tokens, configured limit, % full, and the lifetime
        // seen/pass/drop counters (the same numbers exposed in FWS but joined
        // with the live token state). The "RL" shortcut is taken by RATELIMIT
        // (see line 5174), so we use "RLS" for the new-command shortcut. (The
        // previous comment said RSSILOG owned 'RL' but that was wrong: RL
        // routing always went to RATELIMIT, making RSSILOG's RL arm dead.
        // Commit 42 renamed RSSILOG's shortcut to 'PL'.)
        memset(cmdBuf.rlsBuf, 0, sizeof(cmdBuf.rlsBuf));
        formatRLBuckets(cmdBuf.rlsBuf, sizeof(cmdBuf.rlsBuf));
        startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "RL Buckets");
        appendPaging(cmdBuf.rlsBuf);
        snprintf(replyMsg, sizeof(replyMsg), "JT: Paging RL buckets...");
    }
    else if (strcasecmp(cmd, "FULLCONFIGDUMP") == 0 ||
             strcasecmp(cmd, "FCD") == 0 ||
             strncasecmp(cmd, "FCD ", 4) == 0) {
        // /nexus FULLCONFIGDUMP (alias FCD) - dump every LocalConfig and
        // ModuleConfig field on the node. Excludes Security (per scope) and
        // Channels (already has a dedicated /nexus CHANNELS command). Paging
        // writes directly into paging_buf via per-section appendPaging() calls;
        // no static accumulator buffer is needed (saves ~1 KB of BSS).
        // Total output ~6 KB produces ~35 paging parts at 2s each.
        startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "Full Config Dump");
        appendFCDSection_DEVICE();
        appendFCDSection_POSITION();
        appendFCDSection_POWER();
        appendFCDSection_NETWORK();
        appendFCDSection_DISPLAY();
        appendFCDSection_LORA();
        appendFCDSection_BLUETOOTH();
        appendFCDSection_MQTT();
        appendFCDSection_SERIAL();
        appendFCDSection_EXTNOTIF();
        appendFCDSection_STOREFORWARD();
        appendFCDSection_RANGETEST();
        appendFCDSection_TELEMETRY();
        appendFCDSection_CANNEDMSG();
        appendFCDSection_AUDIO();
        appendFCDSection_REMOTEHW();
        appendFCDSection_NEIGHBORINFO();
        snprintf(replyMsg, sizeof(replyMsg), "JT: Paging full config dump...");
    }
    else if (strncasecmp(cmd, "RATELIMIT ", 10) == 0 || strcasecmp(cmd, "RATELIMIT") == 0 ||
             strncasecmp(cmd, "RL ", 3) == 0 || strcasecmp(cmd, "RL") == 0) {
        // Compute offset past the matched prefix. RL is 2 chars, RATELIMIT is 9 chars.
        // If the form had a trailing space, skip it too.
        size_t offset = (strncasecmp(cmd, "RL", 2) == 0) ? 2 : 9;
        if (cmd[offset] == ' ') offset++;
        const char *p = cmd + offset;
        while(*p == ' ') p++;

        char target[16] = {0};
        int i = 0;
        while(*p && *p != ' ' && i < 15) target[i++] = *p++;
        while(*p == ' ') p++;
        uint16_t val = (uint16_t)atoi(p);

        if (strcasecmp(target, "TEXT") == 0) {
            rl_text_limit = val;
            rl_tokens[0] = (float)val;
            snprintf(replyMsg, sizeof(replyMsg), "JT: RL TEXT set to %u pkts/hr", val);
        } else if (strcasecmp(target, "TRACE") == 0) {
            rl_trace_limit = val;
            rl_tokens[1] = (float)val;
            snprintf(replyMsg, sizeof(replyMsg), "JT: RL TRACE set to %u pkts/hr", val);
        } else if (strcasecmp(target, "OTHER") == 0) {
            rl_other_limit = val;
            rl_tokens[2] = (float)val;
            snprintf(replyMsg, sizeof(replyMsg), "JT: RL OTHER set to %u pkts/hr", val);
        } else if (strcasecmp(target, "NODEINFO") == 0 || strcasecmp(target, "NI") == 0) {
            rl_nodeinfo_limit = val;
            rl_tokens[3] = (float)val;
            snprintf(replyMsg, sizeof(replyMsg), "JT: RL NODEINFO set to %u pkts/hr", val);
        } else if (strcasecmp(target, "POSITION") == 0 || strcasecmp(target, "PO") == 0) {
            rl_position_limit = val;
            rl_tokens[4] = (float)val;
            snprintf(replyMsg, sizeof(replyMsg), "JT: RL POSITION set to %u pkts/hr", val);
        } else if (strcasecmp(target, "TELEMETRY") == 0 || strcasecmp(target, "TL") == 0) {
            rl_telemetry_limit = val;
            rl_tokens[5] = (float)val;
            snprintf(replyMsg, sizeof(replyMsg), "JT: RL TELEMETRY set to %u pkts/hr", val);
        } else if (strcasecmp(target, "PRIVATE") == 0 || strcasecmp(target, "PR") == 0) {
            rl_private_limit = val;
            rl_tokens[6] = (float)val;
            snprintf(replyMsg, sizeof(replyMsg), "JT: RL PRIVATE set to %u pkts/hr", val);
        } else if (strcasecmp(target, "STATUS") == 0 || strcasecmp(target, "STAT") == 0 ||
                   strcasecmp(target, "BUCKETS") == 0) {
            // /nexus RATELIMIT STATUS [alias STAT/BUCKETS] - delegate to the
            // bucket-state view (same output as /nexus RATELIMITSTAT / RLS).
            memset(cmdBuf.rlsBuf, 0, sizeof(cmdBuf.rlsBuf));
            formatRLBuckets(cmdBuf.rlsBuf, sizeof(cmdBuf.rlsBuf));
            startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "RL Buckets");
            appendPaging(cmdBuf.rlsBuf);
            snprintf(replyMsg, sizeof(replyMsg), "JT: Paging RL buckets...");
        } else {
            // No target: print the 7 limits (configuration view). Kept short
            // (under 200 bytes) so it fits in a single reply packet.
            snprintf(replyMsg, sizeof(replyMsg),
                "JT RL Limits: TX:%u TR:%u OT:%u NI:%u PO:%u TL:%u PR:%u",
                (unsigned)rl_text_limit, (unsigned)rl_trace_limit, (unsigned)rl_other_limit,
                (unsigned)rl_nodeinfo_limit, (unsigned)rl_position_limit,
                (unsigned)rl_telemetry_limit, (unsigned)rl_private_limit);
        }
    }
    else if (strcasecmp(cmd, "INVALIDS") == 0 || strcasecmp(cmd, "INV") == 0 || strcasecmp(cmd, "INVALID") == 0 ||
             strncasecmp(cmd, "INVALID ", 8) == 0 || strncasecmp(cmd, "INV ", 4) == 0 ||
             strncasecmp(cmd, "UNINVALID ", 10) == 0 || strncasecmp(cmd, "UNINV ", 6) == 0) {

        bool isCheck = (strcasecmp(cmd, "INVALIDS") == 0 || strcasecmp(cmd, "INV") == 0 || strcasecmp(cmd, "INVALID") == 0);
        bool setInv = (strncasecmp(cmd, "UN", 2) != 0);

        if (isCheck) {
            snprintf(replyMsg, sizeof(replyMsg), "JT INVALID Rules:\nNULL:%s (%u) SPAIN:%s (%u)\nGERM:%s (%u) HW:%s (%u)\nID:%s (%u)",
                     inv_null_island ? "ON" : "OFF", (unsigned int)inv_drop_null,
                     inv_out_spain ? "ON" : "OFF", (unsigned int)inv_drop_spain,
                     inv_out_germany ? "ON" : "OFF", (unsigned int)inv_drop_germany,
                     inv_hw_unset ? "ON" : "OFF", (unsigned int)inv_drop_hw,
                     inv_id_mismatch ? "ON" : "OFF", (unsigned int)inv_drop_id);
        } else {
            const char *p = cmd + (setInv ? (strncasecmp(cmd, "INV ", 4) == 0 ? 4 : 8) : (strncasecmp(cmd, "UNINV ", 6) == 0 ? 6 : 10));
            while(*p == ' ') p++;

            bool handled = true;
            if (strcasecmp(p, "NULL") == 0 || strcasecmp(p, "NULL_ISLAND") == 0) inv_null_island = setInv;
            else if (strcasecmp(p, "SPAIN") == 0) inv_out_spain = setInv;
            else if (strcasecmp(p, "GERMANY") == 0) inv_out_germany = setInv;
            else if (strcasecmp(p, "HW_UNSET") == 0 || strcasecmp(p, "HW") == 0) inv_hw_unset = setInv;
            else if (strcasecmp(p, "ID_MISMATCH") == 0 || strcasecmp(p, "ID") == 0) inv_id_mismatch = setInv;
            else handled = false;

            if (handled) {
                snprintf(replyMsg, sizeof(replyMsg), "JT: INVALID %s %s", p, setInv ? "ACTIVE" : "DISABLED");
            } else {
                snprintf(replyMsg, sizeof(replyMsg), "JT INVALID Rules:\nNULL:%s (%u) SPAIN:%s (%u)\nGERM:%s (%u) HW:%s (%u)\nID:%s (%u)",
                         inv_null_island ? "ON" : "OFF", (unsigned int)inv_drop_null,
                         inv_out_spain ? "ON" : "OFF", (unsigned int)inv_drop_spain,
                         inv_out_germany ? "ON" : "OFF", (unsigned int)inv_drop_germany,
                         inv_hw_unset ? "ON" : "OFF", (unsigned int)inv_drop_hw,
                         inv_id_mismatch ? "ON" : "OFF", (unsigned int)inv_drop_id);
            }
        }
    }
    else if (strcasecmp(cmd, "VERSION") == 0 || strcasecmp(cmd, "V") == 0) {
        auto metadata = getDeviceMetadata();
        snprintf(replyMsg, sizeof(replyMsg), "JT VERSION: %s", metadata.firmware_version);
    }
    else if (strcasecmp(cmd, "LORA-STATUS") == 0 || strcasecmp(cmd, "LRS") == 0) {
        // Nexus-28002 (2026-08-21): /nexus LRS sources ALL chip-side values from
        // the RadioInterface wrapper getters — NOT from config.lora or nexusModule state.
        // The wrapper's own fields (bw, sf, cr, preambleLength, preambleTimeMsec,
        // slotTimeMsec, power) are written by applyModemConfig() with all regional clamps
        // applied and then passed verbatim to RadioLib. They are the chip-actual truth.
        // syncWord lives on RadioLibInterface and reflects either the Meshtastic default
        // 0x2B or NexusModule::syncword_override (vault v40).
        //
        // Nexus-269 design (commit 86e833112): "/nexus LRS MUST source its data from
        // these getters, not from Nexus-side state." The previous Nexus-28001 form
        // (config.lora.tx_power, config.lora.spread_factor, etc.) reported operator intent,
        // not chip reality — could drift on regional clamps and setOutputPower() errors.
        //
        // Format mirrors Nexus-269 verbatim (Pre: + Slot: + CW: lines restored; SyncWr
        // byte shows the actual on-air byte 0x2B vs override 0xNN).
        RadioInterface *radio = (router) ? router->getInterface() : nullptr;
        if (!radio) {
            snprintf(replyMsg, sizeof(replyMsg), "JT: RadioInterface NULL!");
        } else {
            // Read every chip-side value through the wrapper getter set.
            // syncWord lives on RadioLibInterface, cast to access getSyncWord().
            float bw = radio->getBW();           // kHz (post clampBandwidthKHz)
            uint8_t sf = radio->getSF();
            uint8_t crDenom = radio->getCR();    // display-domain 4..8 (4/5 .. 4/8)
            uint16_t pre = radio->getPreambleLength();
            uint32_t preMs = radio->getPreambleTimeMsec();
            uint32_t slotMs = radio->getSlotTimeMsec();
            int8_t chipPwr = radio->getPower();  // post limitPower() + min-clamp
            float freqMHz = radio->getFreq();
            uint8_t syncByte = static_cast<RadioLibInterface *>(radio)->getSyncWord();

            // Alt-modem active (SETFSK/SETFLRC): bw/sf/cr/Pre/Slot/CW/SW/Speed are
            // the PARKED LoRa config — the modulation on air is FSK/FLRC, which has
            // no SF/BW/CR. Report alt-modem reality instead (Jenny, 2026-10-16).
            // Freq/ChipPwr stay valid: both alt paths reuse applyModemConfig's
            // freq/power pipeline. SW: FSK pins the 2-byte wire profile 0x12AD
            // explicitly; FLRC's word-sync lives inside beginFLRC's chip config.
            if (altModem.mode != 0) {
                const uint32_t altSpeed = (uint32_t)(altModem.brKbps * 1000.0f);
                if (altModem.mode == 2) {
                    const char *crName = (altModem.cr == 2) ? "1/2" : (altModem.cr == 3) ? "3/4" : "1/1";
                    snprintf(replyMsg, sizeof(replyMsg),
                        "JT LORA Status:\nMode: FLRC\nFreq: %.3fMHz\nBR:%.1fk CR:%s BT:%d\nPre:%u bits\nChipPwr:%ddBm\nSpeed:%u b/s",
                        freqMHz, altModem.brKbps, crName, (int)altModem.bt,
                        (unsigned)altModem.preambleBits, chipPwr, altSpeed);
                } else {
                    snprintf(replyMsg, sizeof(replyMsg),
                        "JT LORA Status:\nMode: FSK (GFSK)\nFreq: %.3fMHz\nBR:%.1fk dev:%.1fk RXBW:%.1fk\nPre:%u bits\nSW:0x12AD (wire profile)\nChipPwr:%ddBm\nSpeed:%u b/s",
                        freqMHz, altModem.brKbps, altModem.freqDevKHz, altModem.rxBwKHz,
                        (unsigned)altModem.preambleBits, chipPwr, altSpeed);
                }
            } else {
            // LoRa useful bit rate (Semtech formula, accounting for FEC).
            //   R_b = SF * BW_Hz * 4 / (2^SF * CR_denom)
            // BW is stored in kHz (float); multiply by 1000 to get Hz. CR_denom is 5..8.
            // Float math avoids integer truncation at typical configs (e.g. SF9/BW812.5/CR=5 → 11426 bps).
            const float bwHz = bw * 1000.0f;
            uint32_t speedBps = (uint32_t)((bwHz * (float)sf * 4.0f) / ((float)(1u << sf) * (float)crDenom));

            // CWmin/CWmax are static constexpr on RadioInterface. Access via the class
            // directly (no instance needed).
            snprintf(replyMsg, sizeof(replyMsg),
                "JT LORA Status:\nFreq: %.3fMHz\nBW:%.2fkHz SF:%d CR:%d\nPre:%u (%ums)\nSlot:%ums CW:%u-%u\nSW:0x%02X\nChipPwr:%ddBm\nSpeed:%u b/s",
                freqMHz, bw, sf, crDenom,
                (unsigned)pre, (unsigned)preMs,
                (unsigned)slotMs, (unsigned)RadioInterface::CWmin, (unsigned)RadioInterface::CWmax,
                syncByte, chipPwr,
                speedBps);
            }
        }
    }
    else if (strcasecmp(cmd, "HOPMATRIX") == 0 || strcasecmp(cmd, "HM") == 0) {
        startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "Hop Matrix");
        appendPaging("S\\U: 0 1 2 3 4 5 6 7\n");
        static char line[128];
        for (int s = 0; s < 8; s++) {
            memset(line, 0, sizeof(line));
            int l = snprintf(line, sizeof(line), "S%d: ", s);
            for (int u = 0; u <= s && u < 8; u++) { // u is used hops (cannot exceed start s)
                l += snprintf(line + l, sizeof(line) - l, "%u ", (unsigned int)nexus_hop_matrix[s][u]);
            }
            strcat(line, "\n");
            appendPaging(line);
        }
        memset(line, 0, sizeof(line));
        snprintf(replyMsg, sizeof(replyMsg), "JT: Paging hop matrix...");
    }
    else if (strcasecmp(cmd, "HOPSTATS") == 0 || strcasecmp(cmd, "HS") == 0) {
        uint32_t summary[8] = {0};
        uint32_t total = 0;
        for (int s = 0; s < 8; s++) {
            for (int u = 0; u < 8; u++) {
                summary[u] += nexus_hop_matrix[s][u];
                total += nexus_hop_matrix[s][u];
            }
        }
        int len = snprintf(shared_buf, sizeof(shared_buf), "JT HOPSTATS (T:%u):\n", (unsigned int)total);
        for (int i = 0; i < 8; i++) {
            if (summary[i] > 0 || i < 3) {
                len += snprintf(shared_buf + len, sizeof(shared_buf) - len, "H%d:%u ", i, (unsigned int)summary[i]);
                if (i == 3) len += snprintf(shared_buf + len, sizeof(shared_buf) - len, "\n");
            }
        }
        snprintf(replyMsg, sizeof(replyMsg), "%s", shared_buf);
    }
    else if (strcasecmp(cmd, "PACKETSTATS") == 0 || strcasecmp(cmd, "PS") == 0) {
        struct BytesFmt {
            static const char* b(uint32_t bytes, char *out, size_t outSize) {
                if (bytes < 1024) snprintf(out, outSize, "%uB", (unsigned int)bytes);
                else if (bytes < 1024ULL * 1024ULL) snprintf(out, outSize, "%.1fKB", bytes / 1024.0f);
                else snprintf(out, outSize, "%.2fMB", bytes / 1048576.0f);
                return out;
            }
        };
        static const char* portNames[] = {
            "TEXT", "POS", "INFO", "TELE", "NEIG", "ADM", "TRCE", "S&F",
            "RPLY", "REHW", "ATAK", "WAYP", "TUNN", "DETC", "PAX", "UNK",
            "AUDI", "RNGE", "RNST", "SERL", "ALRT", "KEYV", "SF++", "NSTA",
            "TAK2", "MAPR", "PSTR", "LWAB", "CAYN", "U29",  "U30",  "U31"
        };
        startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "Packet Stats");
        uint32_t total = 0;
        for (int i = 0; i < 32; i++) total += nexus_port_stats[i];
        uint32_t totalBytes = 0;
        for (int i = 0; i < 32; i++) totalBytes += nexus_port_bytes[i];

        // Static buffers (BSS, not stack) so loopTask pressure stays low.
        // Sized to match the original members. Cleared at start AND end to
        // prevent residual bytes from bleeding into the next call.
        static char tLine[96];
        static char bytesBuf[32];
        static char pLine160[160];
        memset(tLine, 0, sizeof(tLine));
        memset(bytesBuf, 0, sizeof(bytesBuf));
        snprintf(tLine, sizeof(tLine), "Total: %u pkts (%s)\n",
                 (unsigned int)total, BytesFmt::b(totalBytes, bytesBuf, sizeof(bytesBuf)));
        appendPaging(tLine);

        for (int i = 0; i < 32; i++) {
            if (nexus_port_stats[i] > 0) {
                memset(pLine160, 0, sizeof(pLine160));
                float pctP = (total > 0) ? (float)nexus_port_stats[i] * 100.0f / (float)total : 0.0f;
                float pctB = (totalBytes > 0) ? (float)nexus_port_bytes[i] * 100.0f / (float)totalBytes : 0.0f;
                int lLen = snprintf(pLine160, sizeof(pLine160), "%s: %u pkts (%.1f%%) %s (%.1f%%)",
                                    portNames[i], (unsigned int)nexus_port_stats[i], pctP,
                                    BytesFmt::b(nexus_port_bytes[i], bytesBuf, sizeof(bytesBuf)), pctB);

                if (i == 3) { // TELEMETRY
                    lLen += snprintf(pLine160 + lLen, sizeof(pLine160) - lLen, " [DEV:%u PWR:%u ENV:%u OTH:%u]",
                        (unsigned int)nexus_tele_stats[0], (unsigned int)nexus_tele_stats[1],
                        (unsigned int)nexus_tele_stats[2], (unsigned int)nexus_tele_stats[3]);
                }
                if (i == 1) { // POSITION
                    lLen += snprintf(pLine160 + lLen, sizeof(pLine160) - lLen, " [MAN:%u GPS:%u OTH:%u]",
                        (unsigned int)nexus_pos_stats[0], (unsigned int)nexus_pos_stats[1],
                        (unsigned int)nexus_pos_stats[2]);
                }
                if (i == 2) { // NODEINFO
                    lLen += snprintf(pLine160 + lLen, sizeof(pLine160) - lLen, " [BCAST:%u REQ:%u B_REQ:%u]",
                        (unsigned int)nexus_nodeinfo_stats[0], (unsigned int)nexus_nodeinfo_stats[1],
                        (unsigned int)nexus_nodeinfo_stats[2]);
                }

                strcat(pLine160, "\n");
                appendPaging(pLine160);
            }
        }

        snprintf(replyMsg, sizeof(replyMsg), "JT: Paging packet stats...");
        // Clean up static scratch buffers so the next call starts fresh.
        memset(tLine, 0, sizeof(tLine));
        memset(bytesBuf, 0, sizeof(bytesBuf));
        memset(pLine160, 0, sizeof(pLine160));
    }
    else if (strncasecmp(cmd, "SORTEDPACKETSTATS ", 18) == 0 || strcasecmp(cmd, "SORTEDPACKETSTATS") == 0 ||
             strncasecmp(cmd, "SPS ", 4) == 0 || strcasecmp(cmd, "SPS") == 0) {
        static const char* spsPortNames[] = {
            "TEXT", "POS", "INFO", "TELE", "NEIG", "ADM", "TRCE", "S&F",
            "RPLY", "REHW", "ATAK", "WAYP", "TUNN", "DETC", "PAX", "UNK",
            "AUDI", "RNGE", "RNST", "SERL", "ALRT", "KEYV", "SF++", "NSTA",
            "TAK2", "MAPR", "PSTR", "LWAB", "CAYN", "U29",  "U30",  "U31"
        };
        struct SPSBytesFmt {
            static const char* b(uint32_t bytes, char *out, size_t outSize) {
                if (bytes < 1024) snprintf(out, outSize, "%uB", (unsigned int)bytes);
                else if (bytes < 1024ULL * 1024ULL) snprintf(out, outSize, "%.1fKB", bytes / 1024.0f);
                else snprintf(out, outSize, "%.2fMB", bytes / 1048576.0f);
                return out;
            }
        };
        startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "Sorted Packet Stats");

        // Optional tail = top N limit (SPS 20 = top 20).
        //   /nexus SPS            -> all active ports, sorted
        //   /nexus SPS 20         -> top 20
        //   /nexus SPS 0          -> no rows (explicit top-0)
        //   /nexus SPS <garbage>  -> atoi() == 0 -> no rows
        int spsOff = (strncasecmp(cmd, "SPS", 3) == 0) ? 3 : 17;
        const char *spsP = cmd + spsOff;
        while (*spsP == ' ') spsP++;
        int spsRequested = 0x7FFFFFFF; // default: show all non-zero
        if (*spsP != '\0') {
            int v = atoi(spsP);
            if (v <= 0) spsRequested = 0;
            else spsRequested = v;
        }

        uint32_t total = 0;
        for (int i = 0; i < 32; i++) total += nexus_port_stats[i];
        uint32_t totalBytes = 0;
        for (int i = 0; i < 32; i++) totalBytes += nexus_port_bytes[i];

        // Rate window. Rates are "since-clear": measured from sps_last_clear_ms
        // (set by /nexus CS) to now. Before CS has run, sps_last_clear_ms is 0
        // and the rate lines print -1 / n/a so the operator sees the absence
        // of a reference window rather than a misleading number.
        uint32_t now_ms = Time::getMillis();
        bool have_window = (sps_last_clear_ms > 0 && now_ms > sps_last_clear_ms);
        float dt_s = have_window ? (float)(now_ms - sps_last_clear_ms) / 1000.0f : 0.0f;
        // Live counters are the per-port deltas since the clear (CS zeroes
        // them). No snapshot staging needed.
        auto pktsSinceClear  = [&](int i) -> uint32_t { return nexus_port_stats[i]; };
        auto bytesSinceClear = [&](int i) -> uint32_t { return nexus_port_bytes[i]; };

        // Static buffers (BSS, not stack).
        static char tLine[96];
        static char bytesBuf[32];
        static char rateBuf[48];
        static char pLine160[160];
        struct PortEntry { uint8_t id; uint32_t pkts; uint32_t bytes; };
        static PortEntry sortKey;  // single slot used as swap scratch
        memset(tLine, 0, sizeof(tLine));
        memset(bytesBuf, 0, sizeof(bytesBuf));
        if (have_window) {
            uint32_t dtPkt = 0, dtByt = 0;
            for (int i = 0; i < 32; i++) { dtPkt += pktsSinceClear(i); dtByt += bytesSinceClear(i); }
            // Rates reported per HOUR (LoRa mesh granularity is far too coarse
            // for per-second to be useful: typical nodes emit single-digit pkts
            // per minute). Internal dt_s stays in seconds; we scale at print.
            float aggPph = (dt_s > 0) ? (float)dtPkt * 3600.0f / dt_s : 0.0f;
            float aggBph = (dt_s > 0) ? ((float)dtByt * 8.0f) * 3600.0f / dt_s : 0.0f;
            snprintf(tLine, sizeof(tLine), "Total: %u pkts (%s)\nRates: %.1f pph, %.0f bph (%.1fs window)\n",
                     (unsigned int)total, SPSBytesFmt::b(totalBytes, bytesBuf, sizeof(bytesBuf)),
                     aggPph, aggBph, dt_s);
        } else {
            snprintf(tLine, sizeof(tLine), "Total: %u pkts (%s)\nRates: n/a (run CS to start a window)\n",
                     (unsigned int)total, SPSBytesFmt::b(totalBytes, bytesBuf, sizeof(bytesBuf)));
        }
        appendPaging(tLine);

        // Collect active ports into a sortable list
        // Heap-allocated with the EXACT active count (no waste, no 384B stack)
        int activeCount = 0;
        for (int i = 0; i < 32; i++) {
            if (nexus_port_stats[i] > 0) activeCount++;
        }
        // Hoist toShow so replyMsg at function scope can read it.
        uint32_t spsToShow = ((uint32_t)spsRequested < (uint32_t)activeCount)
                             ? (uint32_t)spsRequested : (uint32_t)activeCount;

        if (activeCount == 0) {
            appendPaging("No traffic recorded.");
        } else {
            PortEntry *entries = new PortEntry[activeCount];
            if (entries) {
                int idx = 0;
                for (int i = 0; i < 32; i++) {
                    if (nexus_port_stats[i] > 0) {
                        entries[idx].id = (uint8_t)i;
                        entries[idx].pkts = nexus_port_stats[i];
                        entries[idx].bytes = nexus_port_bytes[i];
                        idx++;
                    }
                }

                // Insertion sort: primary desc by pkts, secondary asc by id
                for (int i = 1; i < activeCount; i++) {
                    // Use a single static PortEntry as swap scratch.
                    memset(&sortKey, 0, sizeof(sortKey));
                    sortKey = entries[i];
                    int j = i - 1;
                    while (j >= 0 && (entries[j].pkts < sortKey.pkts ||
                                      (entries[j].pkts == sortKey.pkts && entries[j].id > sortKey.id))) {
                        entries[j + 1] = entries[j];
                        j--;
                    }
                    entries[j + 1] = sortKey;
                }

                for (uint32_t n = 0; n < spsToShow; n++) {
                    int i = entries[n].id;
                    memset(pLine160, 0, sizeof(pLine160));
                    memset(rateBuf, 0, sizeof(rateBuf));
                    float pctP = (total > 0) ? (float)entries[n].pkts * 100.0f / (float)total : 0.0f;
                    float pctB = (totalBytes > 0) ? (float)entries[n].bytes * 100.0f / (float)totalBytes : 0.0f;
                    // Per-port rate over the same window as the aggregate
                    // header line. Reported per HOUR (see SPS header comment).
                    // bph is rounded to integer to keep the line under 160
                    // chars with the breakdowns. No CS yet -> -1 placeholder
                    // so the columns stay aligned for paging readers.
                    float pph = 0.0f, bph = 0.0f;
                    if (have_window) {
                        pph = (float)pktsSinceClear(i)  * 3600.0f / dt_s;
                        bph = ((float)bytesSinceClear(i) * 8.0f) * 3600.0f / dt_s;
                    } else {
                        pph = -1.0f;
                        bph = -1.0f;
                    }
                    snprintf(rateBuf, sizeof(rateBuf), " | %.1f pph, %.0f bph", pph, bph);
                    int lLen = snprintf(pLine160, sizeof(pLine160), "%s: %u pkts (%.1f%%) %s (%.1f%%)%s",
                                        spsPortNames[i], (unsigned int)entries[n].pkts, pctP,
                                        SPSBytesFmt::b(entries[n].bytes, bytesBuf, sizeof(bytesBuf)), pctB,
                                        rateBuf);

                    if (i == 3) { // TELEMETRY
                        lLen += snprintf(pLine160 + lLen, sizeof(pLine160) - lLen, " [DEV:%u PWR:%u ENV:%u OTH:%u]",
                            (unsigned int)nexus_tele_stats[0], (unsigned int)nexus_tele_stats[1],
                            (unsigned int)nexus_tele_stats[2], (unsigned int)nexus_tele_stats[3]);
                    }
                    if (i == 1) { // POSITION
                        lLen += snprintf(pLine160 + lLen, sizeof(pLine160) - lLen, " [MAN:%u GPS:%u OTH:%u]",
                            (unsigned int)nexus_pos_stats[0], (unsigned int)nexus_pos_stats[1],
                            (unsigned int)nexus_pos_stats[2]);
                    }

                    if (i == 2) { // NODEINFO
                        lLen += snprintf(pLine160 + lLen, sizeof(pLine160) - lLen, " [BCAST:%u REQ:%u B_REQ:%u]",
                            (unsigned int)nexus_nodeinfo_stats[0], (unsigned int)nexus_nodeinfo_stats[1],
                            (unsigned int)nexus_nodeinfo_stats[2]);
                    }

                    strcat(pLine160, "\n");
                    appendPaging(pLine160);
                }
                delete[] entries;
            } else {
                appendPaging("JT: Out of memory for sorted packet stats.");
            }
        }
        snprintf(replyMsg, sizeof(replyMsg), "JT: Paging sorted packet stats (%u of %u active)...", (unsigned int)spsToShow, (unsigned int)activeCount);
        // Rate snapshot is now anchored to /nexus CS (sets sps_last_clear_ms
        // and zeroes the live counters). No per-call snapshot staging needed.
        // Clean up static scratch buffers so the next call starts fresh.
        memset(tLine, 0, sizeof(tLine));
        memset(bytesBuf, 0, sizeof(bytesBuf));
        memset(pLine160, 0, sizeof(pLine160));
        memset(rateBuf, 0, sizeof(rateBuf));
        memset(&sortKey, 0, sizeof(sortKey));
    }
    else if (strncasecmp(cmd, "SENDSTATUS ", 11) == 0) {
        // Nexus-28002 (2026-08-22): send a freeform text as a Node Status
        // message (portnum NODE_STATUS_APP). Status text is capped at 79 chars by
        // StatusMessage.status[80]; longer input is truncated silently. Always
        // broadcast on Ch 0 (matches StatusMessageModule::runOnce behavior).
        const char *p = cmd + 11;
        while (*p == ' ') p++;
        if (!p || *p == '\0') {
            snprintf(replyMsg, sizeof(replyMsg), "JT: Empty status text");
        } else {
            sendNexusStatusMessage(p);
            snprintf(replyMsg, sizeof(replyMsg), "JT: Status sent (%zu chars)", strlen(p));
        }
    }
    else if (strncasecmp(cmd, "STATUSPS ", 9) == 0 || strcasecmp(cmd, "STATUSPS") == 0) {
        // Nexus-28002 (2026-08-22): build the top-N SPS row (same data
        // /nexus SPS reads) and send it as a Node Status message. Default N=5.
        // Format (single-line, all on one status string, fits in 79 chars):
        //   "PS:INFO:1234(37%) POS:567(17%) TL:345(10%) NI:200(6%) NI2:99(3%)"
        // if more than ~5 active ports, only the top N fit; the rest are dropped.
        // Port names mirror the SPS handler's spsPortNames[] index order.
        static const char *spsPortNames[] = {
            "TEXT", "POS", "INFO", "TELE", "NEIG", "ADM", "TRCE", "S&F",
            "RPLY", "REHW", "ATAK", "WAYP", "TUNN", "DETC", "PAX", "UNK",
            "AUDI", "RNGE", "RNST", "SERL", "ALRT", "KEYV", "SF++", "NSTA",
            "TAK2", "MAPR", "PSTR", "LWAB", "CAYN", "U29",  "U30",  "U31"
        };
        int requested = 5; // default
        if (strncasecmp(cmd, "STATUSPS ", 9) == 0) {
            const char *sp = cmd + 9;
            while (*sp == ' ') sp++;
            if (*sp != '\0') {
                int v = atoi(sp);
                if (v > 0) requested = v;
            }
        }
        uint32_t total = 0;
        for (int i = 0; i < 32; i++) total += nexus_port_stats[i];

        int activeCount = 0;
        for (int i = 0; i < 32; i++) {
            if (nexus_port_stats[i] > 0) activeCount++;
        }
        if (activeCount == 0) {
            sendNexusStatusMessage("PS: no traffic recorded");
            snprintf(replyMsg, sizeof(replyMsg), "JT: Status sent (no traffic)");
        } else {
            // Collect active ports, sort desc by pkts (asc by id as tiebreak).
            struct PortEntry { uint8_t id; uint32_t pkts; };
            PortEntry *entries = new PortEntry[activeCount];
            if (!entries) {
                snprintf(replyMsg, sizeof(replyMsg), "JT: OOM (STATUSPS)");
            } else {
                int idx = 0;
                for (int i = 0; i < 32; i++) {
                    if (nexus_port_stats[i] > 0) {
                        entries[idx].id = (uint8_t)i;
                        entries[idx].pkts = nexus_port_stats[i];
                        idx++;
                    }
                }
                PortEntry sortKey;
                for (int i = 1; i < activeCount; i++) {
                    sortKey = entries[i];
                    int j = i - 1;
                    while (j >= 0 && (entries[j].pkts < sortKey.pkts ||
                                       (entries[j].pkts == sortKey.pkts && entries[j].id > sortKey.id))) {
                        entries[j + 1] = entries[j];
                        j--;
                    }
                    entries[j + 1] = sortKey;
                }
                uint32_t toShow = ((uint32_t)requested < (uint32_t)activeCount)
                                  ? (uint32_t)requested : (uint32_t)activeCount;

                // Build status text into a scratch buffer (128B; status string
                // caps at 80). Truncate gracefully if the requested top-N
                // would overflow — drop trailing rows rather than chop names.
                static char smsg[128];
                size_t off2 = 0;
                int n = snprintf(smsg + off2, sizeof(smsg) - off2, "PS:");
                if (n > 0 && (size_t)n < sizeof(smsg) - off2) off2 += n;
                bool overflowed = false;
                for (uint32_t k = 0; k < toShow && !overflowed; k++) {
                    int pct = (total > 0)
                              ? (int)((uint64_t)entries[k].pkts * 100u / total)
                              : 0;
                    int m = snprintf(smsg + off2, sizeof(smsg) - off2,
                                     " %s:%u(%d%%)",
                                     spsPortNames[entries[k].id],
                                     (unsigned)entries[k].pkts, pct);
                    if (m <= 0) break;
                    // Stop if next row would overflow 79-char status cap
                    // (we have 80-byte buffer; leave 1 byte for null).
                    if (off2 + m + 1 >= 80) {
                        overflowed = true;
                        break;
                    }
                    off2 += m;
                }
                smsg[off2] = '\0';
                sendNexusStatusMessage(smsg);
                // Count actual rows appended (loop ran k+1 times; subtract 1 if
                // the last appended row was rejected for overflow).
                uint32_t rowsAdded = overflowed ? (toShow > 0 ? toShow - 1 : 0) : toShow;
                snprintf(replyMsg, sizeof(replyMsg),
                         "JT: Status sent (%u of %u active ports, %s)",
                         (unsigned)rowsAdded,
                         (unsigned)activeCount, overflowed ? "truncated" : "exact");
                delete[] entries;
            }
        }
    }
    else if (strncasecmp(cmd, "NODECRC ", 8) == 0 || strcasecmp(cmd, "NODECRC") == 0) {
        // Nexus-28002 (2026-08-22): /nexus NODECRC <base64-pubkey>
        // Compute the 2.8 deterministic NodeNum = crc32(public_key). Mirrors
        // src/mesh/NodeDB.cpp:491 (my_node_num = crc32Buffer(public_key, 32))
        // and the upstream ErriezCRC32 library at .pio/libdeps/.../ErriezCRC32/,
        // which is the standard CRC-32 IEEE 802.3 (poly 0xEDB88320 reflected,
        // init 0xFFFFFFFF, final XOR 0xFFFFFFFF) — same as Python's
        // binascii.crc32(). Decodes the key via NexusModule::nexus_base64_decode.
        //
        // Companion Python tool at mesh-analysis-tools/pk2nodeid.py (same algo).
        const char *p = cmd + 8;
        while (*p == ' ') p++;
        if (!p || *p == '\0') {
            snprintf(replyMsg, sizeof(replyMsg), "JT: Usage: NODECRC <base64-pubkey>");
        } else if (!meshtastic::nexusModule) {
            snprintf(replyMsg, sizeof(replyMsg), "JT: NexusModule NULL");
        } else {
            uint8_t pk[32];
            size_t dlen = 0;
            if (meshtastic::nexusModule->nexus_base64_decode(pk, sizeof(pk), &dlen, p) != 0) {
                snprintf(replyMsg, sizeof(replyMsg), "JT: Base64 decode failed");
            } else if (dlen != 32) {
                snprintf(replyMsg, sizeof(replyMsg),
                         "JT: Expected 32-byte pubkey, got %u bytes", (unsigned)dlen);
            } else {
                uint32_t nodeNum = crc32Buffer(pk, dlen);
                snprintf(replyMsg, sizeof(replyMsg),
                         "JT: !%08x (%u)", (unsigned)nodeNum, (unsigned)nodeNum);
            }
        }
    }
    else if (strcasecmp(cmd, "CRSTATS") == 0 || strcasecmp(cmd, "CRS") == 0) {
        // /nexus CRSTATS (alias CRS): per-LoRa-coding-rate packet distribution.
        // Counts display-domain CR 5..8 only (short interleave, SX126x/RF95).
        // Long-interleave (display 9..11) and unknown (0) are excluded by design.
        // Layout: per-row Direct, Relayed, Total, % of grand total.
        uint32_t grandTotal = 0;
        for (int i = 0; i < 4; i++) grandTotal += cr_stats[i][0] + cr_stats[i][1];

        if (grandTotal == 0) {
            snprintf(replyMsg, sizeof(replyMsg),
                "JT CR Stats:\nNo packets recorded.\nReset with /nexus CS (CLRSTATS).");
        } else {
            static char line[64];
            size_t off = 0;
            int n = snprintf(replyMsg, sizeof(replyMsg), "JT CR Stats:\n");
            if (n > 0) off += (size_t)n;
            for (int i = 0; i < 4; i++) {
                uint32_t d = cr_stats[i][0];
                uint32_t r = cr_stats[i][1];
                uint32_t t = d + r;
                float pct = (grandTotal > 0) ? (float)t * 100.0f / (float)grandTotal : 0.0f;
                memset(line, 0, sizeof(line));
                snprintf(line, sizeof(line),
                    "CR:%u  D:%-6u R:%-6u T:%-6u %5.1f%%\n",
                    (unsigned)(i + 5), (unsigned)d, (unsigned)r, (unsigned)t, pct);
                size_t ll = strlen(line);
                if (off + ll < sizeof(replyMsg)) {
                    memcpy(replyMsg + off, line, ll);
                    off += ll;
                }
            }
            // Footer: grand total
            memset(line, 0, sizeof(line));
            snprintf(line, sizeof(line), "Total: %u packets", (unsigned)grandTotal);
            size_t ll = strlen(line);
            if (off + ll + 1 < sizeof(replyMsg)) {
                memcpy(replyMsg + off, line, ll);
                off += ll;
                replyMsg[off] = '\0';
            }
            memset(line, 0, sizeof(line));
        }
    }
    else if (strcasecmp(cmd, "FIXMYINFO") == 0 || strcasecmp(cmd, "FMI") == 0) {
        meshtastic_NodeInfoLite *us = nodeDB->getMeshNode(nodeDB->getNodeNum());
        if (!us) {
            snprintf(replyMsg, sizeof(replyMsg), "JT FMI Error: Local node not found in NodeDB!");
        } else {
            bool changed = false;
            static char fixes[128];
            memset(fixes, 0, sizeof(fixes));

            // Long Name
            if (strncmp(us->long_name, owner.long_name, sizeof(us->long_name)) != 0) {
                strncpy(us->long_name, owner.long_name, sizeof(us->long_name));
                us->long_name[sizeof(us->long_name)-1] = '\0';
                strcat(fixes, "LN ");
                changed = true;
            }

            // Short Name
            if (strncmp(us->short_name, owner.short_name, sizeof(us->short_name)) != 0) {
                strncpy(us->short_name, owner.short_name, sizeof(us->short_name));
                us->short_name[sizeof(us->short_name)-1] = '\0';
                strcat(fixes, "SN ");
                changed = true;
            }

            // HW Model
            if (us->hw_model != owner.hw_model) {
                us->hw_model = owner.hw_model;
                strcat(fixes, "HW ");
                changed = true;
            }

            // Role
            if (us->role != owner.role) {
                us->role = owner.role;
                strcat(fixes, "ROLE ");
                changed = true;
            }

            if (changed) {
                nodeDB->saveToDisk(SEGMENT_NODEDATABASE);
                snprintf(replyMsg, sizeof(replyMsg), "JT FMI: Fixed fields: %s", fixes);
            } else {
                snprintf(replyMsg, sizeof(replyMsg), "JT FMI: Info already in sync.");
            }
            memset(fixes, 0, sizeof(fixes));
        }
    }
    else if (strcasecmp(cmd, "CLRSTATS") == 0 || strcasecmp(cmd, "CS") == 0) {
        memset(nexus_hop_matrix, 0, sizeof(nexus_hop_matrix));
        memset(nexus_port_stats, 0, sizeof(nexus_port_stats));
        memset(nexus_port_bytes, 0, sizeof(nexus_port_bytes));
        memset(nexus_transport_stats, 0, sizeof(nexus_transport_stats));
        if (RadioLibInterface::instance) {
            memset(RadioLibInterface::instance->chipStateCount, 0, sizeof(RadioLibInterface::instance->chipStateCount));
            RadioLibInterface::instance->chipStateTotal = 0;
        }

        fw_total_processed = 0;
        fw_nexus_bypass = 0;
        fw_fav_bypass = 0;
        fw_drop_pki = 0;
        fw_drop_pki_local = 0;
        fw_drop_pki_remote = 0;
        fw_drop_private = 0;
        fw_drop_priv_whitelist = 0;
        fw_drop_priv_blacklist = 0;
        fw_drop_relay = 0;
        fw_drop_idr = 0;
        fw_drop_nexus_ignored = 0;
        fw_drop_hop6 = 0;
        fw_drop_hop7 = 0;
        fw_drop_prehop = 0;
        fw_drop_acks = 0;
        fw_drop_req_nodeinfo = 0;
        fw_drop_req_position = 0;
        fw_drop_req_telemetry = 0;
        fw_drop_br_nodeinfo = 0;
        fw_drop_br_position = 0;
        fw_drop_br_telemetry = 0;
        fw_drop_nodeinfo = 0;
        fw_drop_position = 0;
        fw_drop_telemetry = 0;
        fw_drop_neighbor = 0;
        fw_drop_traceroute = 0;
        fw_drop_traceroute_local = 0;
        fw_drop_traceroute_remote = 0;
        fw_drop_traceroute_direct = 0;
        fw_drop_traceroute_direct_local = 0;
        fw_drop_traceroute_direct_remote = 0;
        fw_drop_store_forward = 0;
        fw_drop_admin = 0;
        fw_drop_text = 0;
        fw_drop_keyver = 0;
        fw_drop_alert = 0;
        fw_drop_waypoint = 0;
        fw_drop_signature = 0;
        fw_vanilla_ignored = 0;
        fw_forwarded_opaque = 0;
        fw_forwarded_foreign_pki = 0;
        fw_drop_tele_device = 0;
        fw_drop_tele_power = 0;
        fw_drop_tele_env = 0;
        fw_drop_tele_other = 0;
        fw_drop_pos_manual = 0;
        fw_drop_pos_gps = 0;
        fw_drop_pos_other = 0;
        fw_drop_no_mqtt = 0;
        fw_drop_out_nodeinfo = 0;
        fw_drop_out_position = 0;
        fw_drop_out_telemetry = 0;
        fw_drop_out_traceroute = 0;
        fw_drop_out_admin = 0;
        fw_drop_out_routing = 0;
        rl_drop_text = 0;
        rl_drop_trace = 0;
        rl_drop_other = 0;
        rl_drop_nodeinfo = 0;
        rl_drop_position = 0;
        rl_drop_telemetry = 0;
        rl_drop_private = 0;
        rl_seen_text = 0;
        rl_seen_trace = 0;
        rl_seen_other = 0;
        rl_seen_nodeinfo = 0;
        rl_seen_position = 0;
        rl_seen_telemetry = 0;
        rl_seen_private = 0;
        rl_pass_text = 0;
        rl_pass_trace = 0;
        rl_pass_other = 0;
        rl_pass_nodeinfo = 0;
        rl_pass_position = 0;
        rl_pass_telemetry = 0;
        rl_pass_private = 0;
        memset(relay_stats, 0, sizeof(relay_stats));
        memset(channel_stats, 0, sizeof(channel_stats));
        memset(channel_bytes, 0, sizeof(channel_bytes));
        memset(cr_stats, 0, sizeof(cr_stats));
        mqtt_ok_on = 0;
        mqtt_ok_off_flag_on = 0;
        mqtt_both_off = 0;

        // Invalid-position drop counters (Null Island, Spain-out, Germany-out, HW, ID)
        inv_drop_null = 0;
        inv_drop_spain = 0;
        inv_drop_germany = 0;
        inv_drop_hw = 0;
        inv_drop_id = 0;
        // Telemetry / Position subtype counters
        memset(nexus_tele_stats, 0, sizeof(nexus_tele_stats));
        memset(nexus_pos_stats, 0, sizeof(nexus_pos_stats));
        memset(nexus_nodeinfo_stats, 0, sizeof(nexus_nodeinfo_stats));
        // MQTT blocked and RX wire counters
        nexus_mqtt_blocked_stats = 0;
        rx_private_count = 0;
        rx_total_count = 0;
        // Noise floor sampling data + counters
        memset(noise_samples, 0, sizeof(noise_samples));
        noise_sample_count = 0;
        noise_retry_count = 0;

        // Anchor SPS/SCHS "since-clear" rate windows to right now. CS zeroes
        // the live counters above; these timestamps tell the next SPS/SCHS
        // call when the window started so pph/bph are computed on the fly.
        sps_last_clear_ms = Time::getMillis();
        schs_last_clear_ms = sps_last_clear_ms;

        snprintf(replyMsg, sizeof(replyMsg), "JT: All tactical stats cleared.");
    }
    else if (strcasecmp(cmd, "CHANNELSTATS") == 0 || strcasecmp(cmd, "CHS") == 0) {
        struct ChBytesFmt {
            static const char* b(uint32_t bytes, char *out, size_t outSize) {
                if (bytes < 1024) snprintf(out, outSize, "%uB", (unsigned int)bytes);
                else if (bytes < 1024ULL * 1024ULL) snprintf(out, outSize, "%.1fKB", bytes / 1024.0f);
                else snprintf(out, outSize, "%.2fMB", bytes / 1048576.0f);
                return out;
            }
        };
        startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "Channel Stats");
        uint32_t total = 0;
        for (int i = 0; i < 256; i++) total += channel_stats[i];
        uint32_t totalBytes = 0;
        for (int i = 0; i < 256; i++) totalBytes += channel_bytes[i];

        // Static buffers (BSS, not stack).
        static char tLine[96];
        static char bytesBuf[32];
        static char pLine128[128];
        memset(tLine, 0, sizeof(tLine));
        memset(bytesBuf, 0, sizeof(bytesBuf));
        snprintf(tLine, sizeof(tLine), "Total: %u pkts (%s)\n",
                 (unsigned int)total, ChBytesFmt::b(totalBytes, bytesBuf, sizeof(bytesBuf)));
        appendPaging(tLine);

        for (int i = 0; i < 256; i++) {
            if (channel_stats[i] > 0) {
                memset(pLine128, 0, sizeof(pLine128));
                float pctP = (total > 0) ? (float)channel_stats[i] * 100.0f / (float)total : 0.0f;
                float pctB = (totalBytes > 0) ? (float)channel_bytes[i] * 100.0f / (float)totalBytes : 0.0f;
                // Try to get channel name if it's one of ours (0-7)
                const char* name = "??";
                if (i < MAX_NUM_CHANNELS) {
                    meshtastic_Channel *c = &channels.getByIndex(i);
                    if (c->role != meshtastic_Channel_Role_DISABLED) name = c->settings.name;
                }
                snprintf(pLine128, sizeof(pLine128), "Ch%d(%s): %u pkts (%.1f%%) %s (%.1f%%)\n",
                         i, name, (unsigned int)channel_stats[i], pctP,
                         ChBytesFmt::b(channel_bytes[i], bytesBuf, sizeof(bytesBuf)), pctB);
                appendPaging(pLine128);
            }
        }
        snprintf(replyMsg, sizeof(replyMsg), "JT: Paging channel stats...");
        // Clean up static scratch buffers so the next call starts fresh.
        memset(tLine, 0, sizeof(tLine));
        memset(bytesBuf, 0, sizeof(bytesBuf));
        memset(pLine128, 0, sizeof(pLine128));
    }
    else if (strncasecmp(cmd, "SORTEDCHANNELSTATS ", 19) == 0 || strcasecmp(cmd, "SORTEDCHANNELSTATS") == 0 ||
             strncasecmp(cmd, "SCHS ", 5) == 0 || strcasecmp(cmd, "SCHS") == 0) {
        struct SCHSBytesFmt {
            static const char* b(uint32_t bytes, char *out, size_t outSize) {
                if (bytes < 1024) snprintf(out, outSize, "%uB", (unsigned int)bytes);
                else if (bytes < 1024ULL * 1024ULL) snprintf(out, outSize, "%.1fKB", bytes / 1024.0f);
                else snprintf(out, outSize, "%.2fMB", bytes / 1048576.0f);
                return out;
            }
        };
        startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "Sorted Channel Stats");

        // Optional tail = top N limit (SCHS 10 = top 10).
        //   /nexus SCHS            -> all active channels, sorted
        //   /nexus SCHS 10         -> top 10
        //   /nexus SCHS 0          -> no rows (explicit top-0)
        //   /nexus SCHS <garbage>  -> atoi() == 0 -> no rows
        int schsOff = (strncasecmp(cmd, "SCHS", 4) == 0) ? 4 : 18;
        const char *schsP = cmd + schsOff;
        while (*schsP == ' ') schsP++;
        int schsRequested = 0x7FFFFFFF; // default: show all non-zero
        if (*schsP != '\0') {
            int v = atoi(schsP);
            if (v <= 0) schsRequested = 0;
            else schsRequested = v;
        }

        uint32_t total = 0;
        for (int i = 0; i < 256; i++) total += channel_stats[i];
        uint32_t totalBytes = 0;
        for (int i = 0; i < 256; i++) totalBytes += channel_bytes[i];

        // Rate window. Same "since-clear" semantic as SPS: rates are measured
        // from schs_last_clear_ms (set by /nexus CS) to now. Before CS has run,
        // schs_last_clear_ms is 0 and the rate lines print -1 / n/a so the
        // operator sees the absence of a reference window.
        uint32_t ch_now_ms = Time::getMillis();
        bool ch_have_window = (schs_last_clear_ms > 0 && ch_now_ms > schs_last_clear_ms);
        float ch_dt_s = ch_have_window ? (float)(ch_now_ms - schs_last_clear_ms) / 1000.0f : 0.0f;
        // Live counters are the per-channel deltas since the clear (CS zeroes
        // them). No snapshot staging needed.
        auto chPktSinceClear  = [&](int i) -> uint32_t { return channel_stats[i]; };
        auto chByteSinceClear = [&](int i) -> uint32_t { return channel_bytes[i]; };

        // Static buffers (BSS, not stack).
        static char tLine[96];
        static char bytesBuf[32];
        static char rateBuf[48];
        static char pLine128[128];
        struct ChEntry { uint8_t id; uint32_t pkts; uint32_t bytes; };
        static ChEntry sortChKey;  // single slot used as swap scratch
        memset(tLine, 0, sizeof(tLine));
        memset(bytesBuf, 0, sizeof(bytesBuf));
        if (ch_have_window) {
            uint32_t dtPkt = 0, dtByt = 0;
            for (int i = 0; i < 256; i++) { dtPkt += chPktSinceClear(i); dtByt += chByteSinceClear(i); }
            // Rates reported per HOUR (LoRa mesh granularity; see SPS comment).
            float aggPph = (ch_dt_s > 0) ? (float)dtPkt * 3600.0f / ch_dt_s : 0.0f;
            float aggBph = (ch_dt_s > 0) ? ((float)dtByt * 8.0f) * 3600.0f / ch_dt_s : 0.0f;
            snprintf(tLine, sizeof(tLine), "Total: %u pkts (%s)\nRates: %.1f pph, %.0f bph (%.1fs window)\n",
                     (unsigned int)total, SCHSBytesFmt::b(totalBytes, bytesBuf, sizeof(bytesBuf)),
                     aggPph, aggBph, ch_dt_s);
        } else {
            snprintf(tLine, sizeof(tLine), "Total: %u pkts (%s)\nRates: n/a (run CS to start a window)\n",
                     (unsigned int)total, SCHSBytesFmt::b(totalBytes, bytesBuf, sizeof(bytesBuf)));
        }
        appendPaging(tLine);

        // Collect active channels into a sortable list
        // Heap-allocated with the EXACT active count (no waste, no 3KB stack).
        int activeCount = 0;
        for (int i = 0; i < 256; i++) {
            if (channel_stats[i] > 0) activeCount++;
        }
        // Hoist toShow so replyMsg at function scope can read it.
        uint32_t schsToShow = ((uint32_t)schsRequested < (uint32_t)activeCount)
                              ? (uint32_t)schsRequested : (uint32_t)activeCount;

        if (activeCount == 0) {
            appendPaging("No traffic recorded.");
        } else {
            ChEntry *entries = new ChEntry[activeCount];
            if (entries) {
                int idx = 0;
                for (int i = 0; i < 256; i++) {
                    if (channel_stats[i] > 0) {
                        entries[idx].id = (uint8_t)i;
                        entries[idx].pkts = channel_stats[i];
                        entries[idx].bytes = channel_bytes[i];
                        idx++;
                    }
                }

                // Insertion sort: primary desc by pkts, secondary asc by id
                for (int i = 1; i < activeCount; i++) {
                    // Use a single static ChEntry as swap scratch.
                    sortChKey = entries[i];
                    int j = i - 1;
                    while (j >= 0 && (entries[j].pkts < sortChKey.pkts ||
                                      (entries[j].pkts == sortChKey.pkts && entries[j].id > sortChKey.id))) {
                        entries[j + 1] = entries[j];
                        j--;
                    }
                    entries[j + 1] = sortChKey;
                }

                for (uint32_t n = 0; n < schsToShow; n++) {
                    int i = entries[n].id;
                    memset(pLine128, 0, sizeof(pLine128));
                    memset(rateBuf, 0, sizeof(rateBuf));
                    float pctP = (total > 0) ? (float)entries[n].pkts * 100.0f / (float)total : 0.0f;
                    float pctB = (totalBytes > 0) ? (float)entries[n].bytes * 100.0f / (float)totalBytes : 0.0f;
                    // Per-channel rate (per HOUR, see SPS header comment).
                    // No CS yet -> -1 placeholder so columns stay aligned.
                    float pph = 0.0f, bph = 0.0f;
                    if (ch_have_window) {
                        pph = (float)chPktSinceClear(i)  * 3600.0f / ch_dt_s;
                        bph = ((float)chByteSinceClear(i) * 8.0f) * 3600.0f / ch_dt_s;
                    } else {
                        pph = -1.0f;
                        bph = -1.0f;
                    }
                    snprintf(rateBuf, sizeof(rateBuf), " | %.1f pph, %.0f bph", pph, bph);
                    const char* name = "??";
                    if (i < MAX_NUM_CHANNELS) {
                        meshtastic_Channel *c = &channels.getByIndex(i);
                        if (c->role != meshtastic_Channel_Role_DISABLED) name = c->settings.name;
                    }
                    snprintf(pLine128, sizeof(pLine128), "Ch%d(%s): %u pkts (%.1f%%) %s (%.1f%%)%s\n",
                             i, name, (unsigned int)entries[n].pkts, pctP,
                             SCHSBytesFmt::b(entries[n].bytes, bytesBuf, sizeof(bytesBuf)), pctB,
                             rateBuf);
                    appendPaging(pLine128);
                }
                delete[] entries;
            } else {
                appendPaging("JT: Out of memory for sorted channel stats.");
            }
        }
        snprintf(replyMsg, sizeof(replyMsg), "JT: Paging sorted channel stats (%u of %u active)...", (unsigned int)schsToShow, (unsigned int)activeCount);
        // Rate snapshot is now anchored to /nexus CS (sets schs_last_clear_ms
        // and zeroes the live counters). No per-call snapshot staging needed.
        // Clean up static scratch buffers so the next call starts fresh.
        memset(tLine, 0, sizeof(tLine));
        memset(bytesBuf, 0, sizeof(bytesBuf));
        memset(pLine128, 0, sizeof(pLine128));
        memset(rateBuf, 0, sizeof(rateBuf));
        memset(&sortChKey, 0, sizeof(sortChKey));
    }
    else if (strcasecmp(cmd, "TRANSSTATS") == 0 || strcasecmp(cmd, "TS") == 0) {
        static const char* transNames[] = {
            "INTL", "LORA", "ALT1", "ALT2", "ALT3", "MQTT", "UDP", "API"
        };
        uint32_t total = 0;
        for (int i = 0; i < 8; i++) total += nexus_transport_stats[i];

        startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "Transport Stats");
        // Static to avoid 64+64 = 128B on stack
        static char tLine[64];
        snprintf(tLine, sizeof(tLine), "Total Packets: %u\n", (unsigned int)total);
        appendPaging(tLine);

        for (int i = 0; i < 8; i++) {
            if (nexus_transport_stats[i] > 0) {
                // Static to avoid 64B on stack
                static char pLine[64];
                float pct = (total > 0) ? (float)nexus_transport_stats[i] * 100.0f / (float)total : 0.0f;
                snprintf(pLine, sizeof(pLine), "%s: %u (%.1f%%)\n", transNames[i], (unsigned int)nexus_transport_stats[i], pct);
                appendPaging(pLine);
            }
        }
        snprintf(replyMsg, sizeof(replyMsg), "JT: Paging transport stats...");
    }
    else if (strcasecmp(cmd, "FIREWALLSTATS") == 0 || strcasecmp(cmd, "FWS") == 0) {
        startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "Firewall Stats");
        static char line[96];

        // Aggregate totals for percentage calculations.
        // RL drops (rl_drop_*) are included here so the headline Drop%
        // reflects all operational rejections; per-bucket breakdown is
        // still exposed in the dedicated RL Seen/Pass/Drop rows below.
        uint32_t total_drops =
            fw_drop_pki + fw_drop_pki_local + fw_drop_pki_remote +
            fw_drop_private + fw_drop_priv_whitelist + fw_drop_priv_blacklist +
            fw_drop_relay + fw_drop_idr + fw_drop_nexus_ignored +
            fw_drop_hop6 + fw_drop_hop7 +
            fw_drop_prehop + fw_drop_acks +
            fw_drop_req_nodeinfo + fw_drop_req_position + fw_drop_req_telemetry +
            fw_drop_br_nodeinfo + fw_drop_br_position + fw_drop_br_telemetry +
            fw_drop_nodeinfo + fw_drop_position + fw_drop_telemetry +
            fw_drop_pos_manual + fw_drop_pos_gps + fw_drop_pos_other +
            fw_drop_no_mqtt +
            fw_drop_neighbor + fw_drop_traceroute +
            fw_drop_traceroute_local + fw_drop_traceroute_remote +
            fw_drop_traceroute_direct + fw_drop_traceroute_direct_local + fw_drop_traceroute_direct_remote +
            fw_drop_store_forward + fw_drop_admin + fw_drop_text +
            fw_drop_keyver + fw_drop_alert + fw_drop_waypoint +
            fw_drop_tele_device + fw_drop_tele_power + fw_drop_tele_env + fw_drop_tele_other +
            fw_drop_out_nodeinfo + fw_drop_out_position + fw_drop_out_telemetry +
            fw_drop_out_traceroute + fw_drop_out_admin + fw_drop_out_routing +
            fw_drop_signature +
            fw_vanilla_ignored +
            rl_drop_text + rl_drop_trace + rl_drop_other +
            rl_drop_nodeinfo + rl_drop_position + rl_drop_telemetry + rl_drop_private;

        uint32_t total_bypasses = fw_nexus_bypass + fw_fav_bypass;
        uint32_t accepted = (fw_total_processed > total_drops + total_bypasses) ?
                            (fw_total_processed - total_drops - total_bypasses) : 0;

        float pct_drops = (fw_total_processed > 0) ? (float)total_drops * 100.0f / (float)fw_total_processed : 0.0f;
        float pct_bypasses = (fw_total_processed > 0) ? (float)total_bypasses * 100.0f / (float)fw_total_processed : 0.0f;
        float pct_accepted = (fw_total_processed > 0) ? (float)accepted * 100.0f / (float)fw_total_processed : 0.0f;

        // Header summary
        snprintf(line, sizeof(line), "Processed: %u\n", (unsigned int)fw_total_processed);
        appendPaging(line);
        snprintf(line, sizeof(line), "Bypass: %u (%.1f%%)  NXS:%u FAV:%u\n",
                 (unsigned int)total_bypasses, pct_bypasses,
                 (unsigned int)fw_nexus_bypass, (unsigned int)fw_fav_bypass);
        appendPaging(line);
        snprintf(line, sizeof(line), "Drop:   %u (%.1f%%)\n", (unsigned int)total_drops, pct_drops);
        appendPaging(line);
        snprintf(line, sizeof(line), "Accept: %u (%.1f%%)\n", (unsigned int)accepted, pct_accepted);
        appendPaging(line);

        // Per-category drops
        snprintf(line, sizeof(line), "PKI: %u (L:%u R:%u)  PRIV: %u  WL: %u  BL: %u\n",
                 (unsigned int)fw_drop_pki, (unsigned int)fw_drop_pki_local, (unsigned int)fw_drop_pki_remote,
                 (unsigned int)fw_drop_private,
                 (unsigned int)fw_drop_priv_whitelist, (unsigned int)fw_drop_priv_blacklist);
        appendPaging(line);
        snprintf(line, sizeof(line), "Relay: %u  IDR: %u\n",
                 (unsigned int)fw_drop_relay, (unsigned int)fw_drop_idr);
        appendPaging(line);
        snprintf(line, sizeof(line), "HOP: 6:%u 7:%u  PRE: %u  ACKS: %u\n",
                 (unsigned int)fw_drop_hop6, (unsigned int)fw_drop_hop7,
                 (unsigned int)fw_drop_prehop, (unsigned int)fw_drop_acks);
        appendPaging(line);
        snprintf(line, sizeof(line), "Req: NI:%u PO:%u TL:%u\n",
                 (unsigned int)fw_drop_req_nodeinfo, (unsigned int)fw_drop_req_position, (unsigned int)fw_drop_req_telemetry);
        appendPaging(line);
        snprintf(line, sizeof(line), "BReq: NI:%u PO:%u TL:%u\n",
                 (unsigned int)fw_drop_br_nodeinfo, (unsigned int)fw_drop_br_position, (unsigned int)fw_drop_br_telemetry);
        appendPaging(line);
        snprintf(line, sizeof(line), "Port: NI:%u PO:%u TL:%u\n",
                 (unsigned int)fw_drop_nodeinfo, (unsigned int)fw_drop_position, (unsigned int)fw_drop_telemetry);
        appendPaging(line);
        snprintf(line, sizeof(line), "Port: NB:%u TR:%u SF:%u\n",
                 (unsigned int)fw_drop_neighbor, (unsigned int)fw_drop_traceroute, (unsigned int)fw_drop_store_forward);
        appendPaging(line);
        snprintf(line, sizeof(line), "Port: AD:%u TX:%u KV:%u AL:%u WP:%u\n",
                 (unsigned int)fw_drop_admin, (unsigned int)fw_drop_text, (unsigned int)fw_drop_keyver,
                 (unsigned int)fw_drop_alert, (unsigned int)fw_drop_waypoint);
        appendPaging(line);
        snprintf(line, sizeof(line), "TL Sub: Dev:%u Pwr:%u Env:%u Oth:%u\n",
                 (unsigned int)fw_drop_tele_device, (unsigned int)fw_drop_tele_power,
                 (unsigned int)fw_drop_tele_env, (unsigned int)fw_drop_tele_other);
        appendPaging(line);
        snprintf(line, sizeof(line), "POS Src: Man:%u GPS:%u Oth:%u  NoMQTT: %u\n",
                 (unsigned int)fw_drop_pos_manual, (unsigned int)fw_drop_pos_gps,
                 (unsigned int)fw_drop_pos_other, (unsigned int)fw_drop_no_mqtt);
        appendPaging(line);
        snprintf(line, sizeof(line), "Trace Loc:%u Rem:%u\n",
                 (unsigned int)fw_drop_traceroute_local, (unsigned int)fw_drop_traceroute_remote);
        appendPaging(line);
        snprintf(line, sizeof(line), "Trace Dir:%u (L:%u R:%u)\n",
                 (unsigned int)fw_drop_traceroute_direct, (unsigned int)fw_drop_traceroute_direct_local,
                 (unsigned int)fw_drop_traceroute_direct_remote);
        appendPaging(line);
        snprintf(line, sizeof(line), "Vanilla Ignore: %u  Nexus-Ignored: %u\n",
                 (unsigned int)fw_vanilla_ignored, (unsigned int)fw_drop_nexus_ignored);
        appendPaging(line);
        snprintf(line, sizeof(line), "OUT Drop: NI:%u PO:%u TL:%u TR:%u AD:%u RO:%u\n",
                 (unsigned int)fw_drop_out_nodeinfo, (unsigned int)fw_drop_out_position,
                 (unsigned int)fw_drop_out_telemetry, (unsigned int)fw_drop_out_traceroute,
                 (unsigned int)fw_drop_out_admin, (unsigned int)fw_drop_out_routing);
        appendPaging(line);
        snprintf(line, sizeof(line), "Sig Drop: %u\n", (unsigned int)fw_drop_signature);
        appendPaging(line);

        // Nexus v66: phone-forward counters from /nexus FEATURES. Forwards are
        // NOT drops — surfaced as their own row so the headline Drop% stays
        // accurate. Subset relation: ForeignPKI <= Opaque.
        snprintf(line, sizeof(line), "Fwd Phone: Opaque:%u ForeignPKI:%u\n",
                 (unsigned int)fw_forwarded_opaque, (unsigned int)fw_forwarded_foreign_pki);
        appendPaging(line);

        // Nexus Rate-Limit drops (separate from upstream TrafficManagement RL)
        // v267: also report seen (RL-engaged packets) and pass (tokens consumed). seen = pass + drop.
        snprintf(line, sizeof(line), "RL Seen: TX:%u TR:%u OT:%u\n",
                 (unsigned int)rl_seen_text, (unsigned int)rl_seen_trace, (unsigned int)rl_seen_other);
        appendPaging(line);
        snprintf(line, sizeof(line), "RL Seen: NI:%u PO:%u TL:%u PR:%u\n",
                 (unsigned int)rl_seen_nodeinfo, (unsigned int)rl_seen_position,
                 (unsigned int)rl_seen_telemetry, (unsigned int)rl_seen_private);
        appendPaging(line);
        snprintf(line, sizeof(line), "RL Pass: TX:%u TR:%u OT:%u\n",
                 (unsigned int)rl_pass_text, (unsigned int)rl_pass_trace, (unsigned int)rl_pass_other);
        appendPaging(line);
        snprintf(line, sizeof(line), "RL Pass: NI:%u PO:%u TL:%u PR:%u\n",
                 (unsigned int)rl_pass_nodeinfo, (unsigned int)rl_pass_position,
                 (unsigned int)rl_pass_telemetry, (unsigned int)rl_pass_private);
        appendPaging(line);
        snprintf(line, sizeof(line), "RL Drop: TX:%u TR:%u OT:%u\n",
                 (unsigned int)rl_drop_text, (unsigned int)rl_drop_trace, (unsigned int)rl_drop_other);
        appendPaging(line);
        snprintf(line, sizeof(line), "RL Drop: NI:%u PO:%u TL:%u PR:%u\n",
                 (unsigned int)rl_drop_nodeinfo, (unsigned int)rl_drop_position,
                 (unsigned int)rl_drop_telemetry, (unsigned int)rl_drop_private);
        appendPaging(line);

        snprintf(replyMsg, sizeof(replyMsg), "JT: Paging firewall stats...");
        memset(line, 0, sizeof(line));
    }
    else if (strncasecmp(cmd, "RELAYS ", 7) == 0 || strcasecmp(cmd, "RELAYS") == 0 ||
             strncasecmp(cmd, "RY ", 3) == 0 || strcasecmp(cmd, "RY") == 0) {
        int off = (strncasecmp(cmd, "RY", 2) == 0) ? 2 : 6;
        const char *p = cmd + off;
        while (*p == ' ') p++;

        if (*p != '\0') {
            // Specific ID requested
            uint32_t rid = strtoul(p, NULL, 0) & 0xFF;

            static char relayNames[64];
            memset(relayNames, 0, sizeof(relayNames));
            getRelayNames(relayNames, sizeof(relayNames), (uint8_t)rid);

            snprintf(replyMsg, sizeof(replyMsg), "Relay *%02x (%s) Stats:\nDirect: %u\nRelayed: %u\nTotal: %u",
                     (unsigned int)rid, relayNames[0] ? relayNames : "Unknown",
                     (unsigned int)relay_stats[rid].direct,
                     (unsigned int)relay_stats[rid].relayed,
                     (unsigned int)(relay_stats[rid].direct + relay_stats[rid].relayed));
            memset(relayNames, 0, sizeof(relayNames));
        } else {
            // Paginated list of non-zero relays
            startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "Relay Stats");
            uint16_t totalNonZero = 0;
            static char relayNames[32];
            static char entry[72];
            for (int i = 0; i < 256; i++) {
                if (relay_stats[i].direct > 0 || relay_stats[i].relayed > 0) {
                    memset(relayNames, 0, sizeof(relayNames));
                    getRelayNames(relayNames, sizeof(relayNames), (uint8_t)i);

                    memset(entry, 0, sizeof(entry));
                    snprintf(entry, sizeof(entry), "*%02x (%s): D:%u R:%u T:%u\n",
                             i, relayNames[0] ? relayNames : "??",
                             (unsigned int)relay_stats[i].direct,
                             (unsigned int)relay_stats[i].relayed,
                             (unsigned int)(relay_stats[i].direct + relay_stats[i].relayed));
                    appendPaging(entry);
                    totalNonZero++;
                }
            }
            memset(relayNames, 0, sizeof(relayNames));
            memset(entry, 0, sizeof(entry));
            if (totalNonZero == 0) {
                appendPaging("No traffic recorded.");
            }
            snprintf(replyMsg, sizeof(replyMsg), "JT: Paging RELAYS (%u active) to mesh...", totalNonZero);
        }
    }
    else if (strcasecmp(cmd, "WIFIOTA") == 0 || strcasecmp(cmd, "OTA") == 0 ||
             strcasecmp(cmd, "OTAINFO") == 0) {
        // ESP32-only: report the OTA loader installed at the ota_1 partition.
        // On other platforms (nRF52, Portduino, etc.) report a clear message.
#if defined(ARCH_ESP32) && !MESHTASTIC_EXCLUDE_WIFI
        memset(cmdBuf.otaInfo, 0, sizeof(cmdBuf.otaInfo));
        MeshtasticOTA::getInfo(cmdBuf.otaInfo, sizeof(cmdBuf.otaInfo));
        // getInfo() formats with embedded newlines. Use appendPaging so the
        // multi-line buffer pages through the operator's view cleanly.
        startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "WiFiOTA Info");
        appendPaging(cmdBuf.otaInfo);
        snprintf(replyMsg, sizeof(replyMsg), "JT: Paging WiFiOTA info...");
#else
        // Non-ESP32 platforms: short message (save airtime bytes).
        // The ota_1 partition is an ESP32 concept; on nRF52/Portduino/etc.
        // there is nothing equivalent to report.
        snprintf(replyMsg, sizeof(replyMsg),
            "JT WIFIOTA:\n"
            "Only available on ESP32");
#endif
    }
    else if (strcasecmp(cmd, "MCUINFO") == 0 || strcasecmp(cmd, "MCU") == 0) {
        // /nexus MCUINFO: platform-agnostic MCU identity + runtime state. v267.
        // Output covers chip model/revision, flash, IDF/SoftDevice version,
        // CPU freq, heap (free + min-ever + integrity flag), die temp, boot
        // reason, MAC/device ID, RNG sample, uptime. Every call that can
        // return an error is checked; failures degrade to 'unavailable'.
        memset(cmdBuf.mcuBuf, 0, sizeof(cmdBuf.mcuBuf));
        formatMCUInfo(cmdBuf.mcuBuf, sizeof(cmdBuf.mcuBuf));
        startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "MCU Info");
        appendPaging(cmdBuf.mcuBuf);
        snprintf(replyMsg, sizeof(replyMsg), "JT: Paging MCU info...");
    }
    else if (strcasecmp(cmd, "SENSORS") == 0) {
        // /nexus SENSORS: list I2C devices detected at boot + live power/env
        // telemetry + GPS. The I2C list is captured once in i2cScanFinished
        // (called from main.cpp:939 via the ScanI2CConsumer hook), not re-scanned
        // on demand. Power/env/GPS are read live via the existing getNexus*
        // helpers + globals.
        memset(cmdBuf.sensBuf, 0, sizeof(cmdBuf.sensBuf));
        formatSensorsInfo(cmdBuf.sensBuf, sizeof(cmdBuf.sensBuf));
        startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "Sensors");
        appendPaging(cmdBuf.sensBuf);
        snprintf(replyMsg, sizeof(replyMsg), "JT: Paging sensors...");
    }
    else if (strncasecmp(cmd, "SORTEDRELAYSTATS ", 17) == 0 || strcasecmp(cmd, "SORTEDRELAYSTATS") == 0 ||
             strncasecmp(cmd, "SRY ", 4) == 0 || strcasecmp(cmd, "SRY") == 0) {
        // Sorted counterpart of RY/RELAYS. Optional tail = top N limit.
        //   /nexus SRY          -> all relays with D+R > 0, sorted by total desc
        //   /nexus SRY 20       -> top 20 by total desc
        //   /nexus SRY 0        -> no rows (explicit top-0)
        //   /nexus SRY <garbage>-> atoi() == 0 -> no rows (matches "0" semantics)
        int off = (strncasecmp(cmd, "SRY", 3) == 0) ? 3 : 16;
        const char *p = cmd + off;
        while (*p == ' ') p++;
        int requestedLimit = 0x7FFFFFFF; // default: show all non-zero (avoid INT_MAX header dep)
        if (*p != '\0') {
            int v = atoi(p);
            if (v <= 0) requestedLimit = 0;   // top-0 or unparseable -> 0 rows
            else requestedLimit = v;
        }

        // Count active relays (D+R > 0) — same predicate as RY.
        uint32_t activeCount = 0;
        uint32_t grandTotal = 0;
        for (int i = 0; i < 256; i++) {
            uint32_t t = (uint32_t)relay_stats[i].direct + (uint32_t)relay_stats[i].relayed;
            if (t > 0) {
                activeCount++;
                grandTotal += t;
            }
        }

        startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "Sorted Relay Stats");

        if (activeCount == 0 || requestedLimit == 0) {
            appendPaging("No traffic recorded.");
            uint32_t shown = 0;
            snprintf(replyMsg, sizeof(replyMsg), "JT: Paging sorted relay stats (%u of %u active)...", shown, (unsigned int)activeCount);
        } else {
            // Heap-allocate with the EXACT active count (avoids 256-entry stack array).
            struct RyEntry { uint8_t id; uint32_t direct; uint32_t relayed; uint32_t total; };
            RyEntry *entries = new RyEntry[activeCount];
            if (entries) {
                int idx = 0;
                for (int i = 0; i < 256; i++) {
                    uint32_t d = (uint32_t)relay_stats[i].direct;
                    uint32_t r = (uint32_t)relay_stats[i].relayed;
                    uint32_t t = d + r;
                    if (t > 0) {
                        entries[idx].id = (uint8_t)i;
                        entries[idx].direct = d;
                        entries[idx].relayed = r;
                        entries[idx].total = t;
                        idx++;
                    }
                }

                // Insertion sort: primary desc by total, secondary asc by id (matches SCHS).
                static RyEntry sortRyKey; // BSS swap scratch (single slot, mirror SCHS)
                for (int i = 1; i < activeCount; i++) {
                    sortRyKey = entries[i];
                    int j = i - 1;
                    while (j >= 0 && (entries[j].total < sortRyKey.total ||
                                      (entries[j].total == sortRyKey.total && entries[j].id > sortRyKey.id))) {
                        entries[j + 1] = entries[j];
                        j--;
                    }
                    entries[j + 1] = sortRyKey;
                }

                // Static (BSS) so 96+32 = 128B doesn't frame-stack the paging loop.
                static char relayNames[32];
                static char entry[96];
                uint32_t toShow = (uint32_t)requestedLimit < activeCount
                                  ? (uint32_t)requestedLimit : activeCount;
                for (uint32_t n = 0; n < toShow; n++) {
                    uint8_t rid = entries[n].id;
                    memset(relayNames, 0, sizeof(relayNames));
                    getRelayNames(relayNames, sizeof(relayNames), rid);
                    memset(entry, 0, sizeof(entry));
                    float pct = (grandTotal > 0) ? (float)entries[n].total * 100.0f / (float)grandTotal : 0.0f;
                    snprintf(entry, sizeof(entry), "*%02x (%s): D:%u R:%u T:%u (%.1f%%)\n",
                             (unsigned int)rid,
                             relayNames[0] ? relayNames : "??",
                             (unsigned int)entries[n].direct,
                             (unsigned int)entries[n].relayed,
                             (unsigned int)entries[n].total,
                             pct);
                    appendPaging(entry);
                }
                memset(relayNames, 0, sizeof(relayNames));
                memset(entry, 0, sizeof(entry));
                memset(&sortRyKey, 0, sizeof(sortRyKey));
                delete[] entries;
                snprintf(replyMsg, sizeof(replyMsg), "JT: Paging sorted relay stats (%u of %u active)...", (unsigned int)toShow, (unsigned int)activeCount);
            } else {
                appendPaging("JT: Out of memory for sorted relay stats.");
                snprintf(replyMsg, sizeof(replyMsg), "JT: Paging sorted relay stats...");
            }
        }
    }
    else if (strncasecmp(cmd, "RSSILOG ", 8) == 0 || strcasecmp(cmd, "RSSILOG") == 0 ||
             strncasecmp(cmd, "PL ", 3) == 0 || strcasecmp(cmd, "PL") == 0) {
        // Shortcut 'PL' = "Ping Log" (RSSI history per packet). The previous
        // shortcut 'RL' was unreachable because RATELIMIT's 'RL' (line 5174)
        // wins dispatch first; PL is free. See Commit 42.
        int requestedCount = 5; // Default to last 5
        const char *pCount = (strncasecmp(cmd, "PL", 2) == 0) ?
                             ((strlen(cmd) > 3) ? cmd + 3 : "") :
                             ((strlen(cmd) > 8) ? cmd + 8 : "");
        while (*pCount == ' ') pCount++;
        if (*pCount != '\0') requestedCount = atoi(pCount);
        if (requestedCount > 32) requestedCount = 32;
        if (requestedCount > nexus_sigint_count) requestedCount = nexus_sigint_count;

        startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "RSSI Log");
        uint32_t now = Time::getMillis() / 1000;

        // Static (BSS) so 72B doesn't frame-stack the paging loop.
        static char entry[72];
        for (int i = 0; i < requestedCount; i++) {
            int pos = (nexus_sigint_idx - 1 - i + 32) % 32;
            auto &e = nexus_sigint_log[pos];
            if (e.from == 0) continue;

            uint32_t ago = now - e.timestamp;
            uint8_t usedHops = (e.hop_start >= e.hop_limit) ? (e.hop_start - e.hop_limit) : 0;

            memset(entry, 0, sizeof(entry));
            // Format: ID(4 hex),H/S,RSSI,SNR,Relay(2 hex),Age
            snprintf(entry, sizeof(entry), "%04x,%u/%u,%d,%.1f,%02x,%us\n",
                     (uint16_t)(e.from & 0xFFFF), usedHops, e.hop_start, (int)e.rssi, e.snr,
                     (uint8_t)(e.relay_node & 0xFF), (unsigned int)ago);
            appendPaging(entry);
        }
        memset(entry, 0, sizeof(entry));
        snprintf(replyMsg, sizeof(replyMsg), "JT: Paging RSSILOG (%d entries) to mesh...", requestedCount);
    }
    else if (strcasecmp(cmd, "CHASH") == 0 || strcasecmp(cmd, "CH") == 0 || strcasecmp(cmd, "CHANNELS") == 0 || strcasecmp(cmd, "CHHASHES") == 0) {
        bool showHashes = (strcasecmp(cmd, "CHASH") == 0 || strcasecmp(cmd, "CHANNELS") == 0 || strcasecmp(cmd, "CHHASHES") == 0);
        memset(shared_buf, 0, sizeof(shared_buf));
        int len = 0;
        for (uint8_t i = 0; i < MAX_NUM_CHANNELS; i++) {
            meshtastic_Channel *c = &channels.getByIndex(i);
            if (c->role != meshtastic_Channel_Role_DISABLED) {
                if (showHashes) {
                    int16_t h = channels.getHash(i);
                    uint8_t h8 = (uint8_t)(h & 0xFF);
                    len += snprintf(shared_buf + len, sizeof(shared_buf) - len, "%u:%s(0x%02X/%u) ",
                                    i, c->settings.name, h8, h8);
                } else {
                    len += snprintf(shared_buf + len, sizeof(shared_buf) - len, "%u:%s(%u) ",
                                    i, c->settings.name, (uint32_t)c->role);
                }
            }
        }
        snprintf(replyMsg, sizeof(replyMsg), "JT %s:\n%s", showHashes ? "CHASH" : "CHANNELS", shared_buf);
    }
    else if (strcasecmp(cmd, "SHORTRSSI") == 0 || strcasecmp(cmd, "SR") == 0) {
        // Self-echo guard: don't reply to loopback / uninitialized rx metrics
        if (isFromUs(&mp)) return;

        uint32_t hopsUsed = (mp.hop_start >= mp.hop_limit) ? (mp.hop_start - mp.hop_limit) : 0;
        uint8_t relayByte = (uint8_t)(mp.relay_node & 0xFF);

        if (hopsUsed == 0) {
            snprintf(replyMsg, sizeof(replyMsg),
                     "JT SR:\nH:%u/%u relay:none\nR:%d S:%.1f F:%.1f",
                     hopsUsed, mp.hop_start,
                     (int)mp.rx_rssi, mp.rx_snr, mp.rx_frequency_error);
        } else {
            snprintf(replyMsg, sizeof(replyMsg),
                     "JT SR:\nH:%u/%u relay:%02x\nR:%d S:%.1f F:%.1f",
                     hopsUsed, mp.hop_start, relayByte,
                     (int)mp.rx_rssi, mp.rx_snr, mp.rx_frequency_error);
        }
    }
    else if (strcasecmp(cmd, "RSSITEL") == 0 || strcasecmp(cmd, "RT") == 0) {
        // Self-echo guard: don't reply to loopback / uninitialized rx metrics
        if (isFromUs(&mp)) return;

        uint32_t hopsUsed = (mp.hop_start >= mp.hop_limit) ? (mp.hop_start - mp.hop_limit) : 0;
        uint8_t relayByte = (uint8_t)(mp.relay_node & 0xFF);

        float liveChUtil = (airTime != NULL) ? airTime->channelUtilizationPercent() : 0.0f;
        float liveTxUtil = (airTime != NULL) ? airTime->utilizationTXPercent() : 0.0f;

        // Device Metrics (B%, BV) - use static storage to protect loopTask stack
        static meshtastic_Telemetry dm = meshtastic_Telemetry_init_zero;
        uint32_t bPct = 0;
        float bVolt = 0.0f;
        if (meshtastic::getNexusDeviceTelemetry(&dm) && dm.which_variant == meshtastic_Telemetry_device_metrics_tag) {
            const auto &d = dm.variant.device_metrics;
            bPct = (uint32_t)d.battery_level;
            bVolt = d.has_voltage ? d.voltage : 0.0f;
        }

        // Power & Env Telemetry (single compact line if present)
        static meshtastic_Telemetry pm = meshtastic_Telemetry_init_zero;
        static meshtastic_Telemetry em = meshtastic_Telemetry_init_zero;
        char peBuf[96] = {0};
        int peLen = 0;

        if (meshtastic::getNexusPowerTelemetry(&pm) && pm.which_variant == meshtastic_Telemetry_power_metrics_tag) {
            auto &p = pm.variant.power_metrics;
            if (p.has_ch1_voltage && p.has_ch1_current) {
                peLen += snprintf(peBuf + peLen, sizeof(peBuf) - peLen, "V1:%.2fV I1:%.0fmA ", p.ch1_voltage, p.ch1_current);
            } else if (p.has_ch1_voltage) {
                peLen += snprintf(peBuf + peLen, sizeof(peBuf) - peLen, "V1:%.2fV ", p.ch1_voltage);
            }
        }

        if (meshtastic::getNexusEnvironmentTelemetry(&em) && em.which_variant == meshtastic_Telemetry_environment_metrics_tag) {
            auto &e = em.variant.environment_metrics;
            if (e.has_temperature) peLen += snprintf(peBuf + peLen, sizeof(peBuf) - peLen, "T:%.1fC ", e.temperature);
            if (e.has_relative_humidity) peLen += snprintf(peBuf + peLen, sizeof(peBuf) - peLen, "H:%.0f%% ", e.relative_humidity);
            if (e.has_barometric_pressure) peLen += snprintf(peBuf + peLen, sizeof(peBuf) - peLen, "P:%.2f ", e.barometric_pressure);
        }

        // Trim trailing space in peBuf if present
        if (peLen > 0 && peBuf[peLen - 1] == ' ') {
            peBuf[peLen - 1] = '\0';
        }

        float cput = getDieTemp();
        // Up: uptime in days as a float (2 decimals). Same monotonic clock as
        // the UPTIME command (Time::getMillisMonotonic — no 49.7-day rollover),
        // just flattened to one days figure instead of d/h/m/s.
        const float upDays = (float)(Time::getMillisMonotonic() / 86400000.0);

        // Format compact replyMsg
        if (hopsUsed == 0) {
            if (peLen > 0) {
                snprintf(replyMsg, sizeof(replyMsg),
                         "JT RT:\nH:%u/%u relay:none\nR:%d S:%.1f F:%.1f\nCU:%.1f%% AT:%.1f%%\nB%%:%u%% BV:%.2fV\n%s\nCPUT:%.1fC Up:%.2f",
                         hopsUsed, mp.hop_start,
                         (int)mp.rx_rssi, mp.rx_snr, mp.rx_frequency_error,
                         liveChUtil, liveTxUtil,
                         bPct, bVolt,
                         peBuf,
                         cput, upDays);
            } else {
                snprintf(replyMsg, sizeof(replyMsg),
                         "JT RT:\nH:%u/%u relay:none\nR:%d S:%.1f F:%.1f\nCU:%.1f%% AT:%.1f%%\nB%%:%u%% BV:%.2fV\nCPUT:%.1fC Up:%.2f",
                         hopsUsed, mp.hop_start,
                         (int)mp.rx_rssi, mp.rx_snr, mp.rx_frequency_error,
                         liveChUtil, liveTxUtil,
                         bPct, bVolt,
                         cput, upDays);
            }
        } else {
            if (peLen > 0) {
                snprintf(replyMsg, sizeof(replyMsg),
                         "JT RT:\nH:%u/%u relay:%02x\nR:%d S:%.1f F:%.1f\nCU:%.1f%% AT:%.1f%%\nB%%:%u%% BV:%.2fV\n%s\nCPUT:%.1fC Up:%.2f",
                         hopsUsed, mp.hop_start, relayByte,
                         (int)mp.rx_rssi, mp.rx_snr, mp.rx_frequency_error,
                         liveChUtil, liveTxUtil,
                         bPct, bVolt,
                         peBuf,
                         cput, upDays);
            } else {
                snprintf(replyMsg, sizeof(replyMsg),
                         "JT RT:\nH:%u/%u relay:%02x\nR:%d S:%.1f F:%.1f\nCU:%.1f%% AT:%.1f%%\nB%%:%u%% BV:%.2fV\nCPUT:%.1fC Up:%.2f",
                         hopsUsed, mp.hop_start, relayByte,
                         (int)mp.rx_rssi, mp.rx_snr, mp.rx_frequency_error,
                         liveChUtil, liveTxUtil,
                         bPct, bVolt,
                         cput, upDays);
            }
        }
    }
    else if (strcasecmp(cmd, "RSSI") == 0 || strcasecmp(cmd, "RS") == 0) {
        // (SELF-ECHO GUARD 2026-08-02) Don't reply to our own packet. A loopback
        // / MQTT-echo / retransmission reaches handleReceived with mp.from == our
        // node num, and rx_rssi / rx_snr / rx_frequency_error are uninitialized →
        // we'd report all-zeroed metrics. Skip silently.
        if (isFromUs(&mp)) return;
        // Report signal metrics of the command packet itself
        uint32_t hopsUsed = (mp.hop_start >= mp.hop_limit) ? (mp.hop_start - mp.hop_limit) : 0;

        int l = snprintf(replyMsg, sizeof(replyMsg), "JT RSSI:\n📡 Hops:%u/%u ", hopsUsed, mp.hop_start);
        if (l < 0 || (size_t)l >= sizeof(replyMsg)) l = sizeof(replyMsg) - 1;

        if (hopsUsed == 0) {
            l += snprintf(replyMsg + l, sizeof(replyMsg) - l, "(Direct)\n");
        } else {
            static char relayNames[64];
            memset(relayNames, 0, sizeof(relayNames));
            uint8_t targetSuffix = (uint8_t)(mp.relay_node & 0xFF);
            getRelayNames(relayNames, sizeof(relayNames), targetSuffix);

            if (relayNames[0] != '\0') {
                l += snprintf(replyMsg + l, sizeof(replyMsg) - l, "Relay:%s (*%02x)\n", relayNames, targetSuffix);
            } else {
                l += snprintf(replyMsg + l, sizeof(replyMsg) - l, "Relay:*%02x\n", targetSuffix);
            }
            memset(relayNames, 0, sizeof(relayNames));
        }

        if (l < 0 || (size_t)l >= sizeof(replyMsg)) l = sizeof(replyMsg) - 1;
        l += snprintf(replyMsg + l, sizeof(replyMsg) - l, "📶 RSSI:%d SNR:%.1f\n🔬 Δf: %.0fHz CR:%d\n", (int)mp.rx_rssi, mp.rx_snr, mp.rx_frequency_error, nexusCRDisplay(mp.rx_coding_rate));
        if (l < 0 || (size_t)l >= sizeof(replyMsg)) l = sizeof(replyMsg) - 1;
        bool om = (mp.decoded.has_bitfield && (mp.decoded.bitfield & BITFIELD_OK_TO_MQTT_MASK));
        int written = snprintf(replyMsg + l, sizeof(replyMsg) - l, "🌐 OM:%d%s MQTT:%d%s UDP:%d PKI:%d",
                 om ? 1 : 0, !om ? "⚠️" : "",
                 mp.via_mqtt ? 1 : 0, mp.via_mqtt ? "⚠️" : "",
                 (mp.transport_mechanism == meshtastic_MeshPacket_TransportMechanism_TRANSPORT_MULTICAST_UDP) ? 1 : 0,
                 mp.pki_encrypted ? 1 : 0);
        // (SIGNATURE FLAG 2026-08-23) Append S:<emoji> to the flags line. Same three-state
        // palette as PPING (NexusModule.cpp:1898); consistent with the auto-prefix 4-state
        // gate one block up so operators reading S: here and S: there see the same picture:
        //   🔴 — no signature field on the wire (decoded.xeddsa_signature.size == 0)
        //   🟠 — signature present but xeddsa_verify() failed (size != 0, !xeddsa_signed)
        //   🟢 — signature present AND verified (xeddsa_signed)
        // Note: PKI:%d above is encryption status, orthogonal to XEdDSA signature status —
        // both fields coexist intentionally. Auto-prefix (🔒/🟢/🟠/🔴) covers the
        // trust-list dimension that this surface doesn't (RSSI is a one-shot diagnostic,
        // not an auth gate).
        const char *sigEmoji;
        if (mp.decoded.xeddsa_signature.size == 0) {
            sigEmoji = "🔴";
        } else if (!mp.xeddsa_signed) {
            sigEmoji = "🟠";
        } else {
            sigEmoji = "🟢";
        }
        if (written > 0 && (size_t)written < sizeof(replyMsg) - l) {
            snprintf(replyMsg + l + written, sizeof(replyMsg) - l - written, " S:%s", sigEmoji);
        }
    }
    else if (strncasecmp(cmd, "TEXT ", 5) == 0) {
        const char *p = cmd + 5;
        while (*p == ' ') p++;
        if (!p || strlen(p) == 0) {
            snprintf(replyMsg, sizeof(replyMsg), "JT: Empty message");
        } else {
            meshtastic_MeshPacket *tp = (router) ? router->allocForSending() : nullptr;
            if (tp) {
                tp->decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;
                tp->to = 0xFFFFFFFF;
                tp->channel = 0; // Always Ch 0 for standard TEXT
                size_t msgLen = strlen(p);
                if (msgLen > sizeof(tp->decoded.payload.bytes)) msgLen = sizeof(tp->decoded.payload.bytes);
                memcpy(tp->decoded.payload.bytes, p, msgLen);
                tp->decoded.payload.size = msgLen;

                if (service) service->sendToMesh(tp);
                snprintf(replyMsg, sizeof(replyMsg), "JT: Text sent to Ch 0");
            }
        }
    }
    else if (strncasecmp(cmd, "TEXTC ", 6) == 0) {
        const char *p = cmd + 6;
        const char *msgStart = nullptr;
        int chIdx = findChannelByNameOrIndex(p, &msgStart);

        if (chIdx < 0) {
            snprintf(replyMsg, sizeof(replyMsg), "JT: Invalid channel");
        } else if (!msgStart || *msgStart == '\0') {
            snprintf(replyMsg, sizeof(replyMsg), "JT: Empty message");
        } else {
            meshtastic_MeshPacket *tp = (router) ? router->allocForSending() : nullptr;
            if (tp) {
                tp->decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;
                tp->to = 0xFFFFFFFF;
                tp->channel = chIdx;
                size_t msgLen = strlen(msgStart);
                if (msgLen > sizeof(tp->decoded.payload.bytes)) msgLen = sizeof(tp->decoded.payload.bytes);
                memcpy(tp->decoded.payload.bytes, msgStart, msgLen);
                tp->decoded.payload.size = msgLen;

                if (service) service->sendToMesh(tp);
                snprintf(replyMsg, sizeof(replyMsg), "JT: Text sent to Ch %d (%s)", chIdx, channels.getByIndex(chIdx).settings.name);
            }
        }
    }
    else if (strcasecmp(cmd, "DMTEST") == 0) {
        auto node = nodeDB->getMeshNode(mp.from);
        if (node && node->public_key.size > 0) {
            meshtastic_MeshPacket *dp = (router) ? router->allocForSending() : nullptr;
            if (dp) {
                dp->decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;
                dp->to = mp.from; // Always reply to the sender specifically
                dp->channel = 0;  // Explicitly use channel 0 for DM (or standard routing)
                strncpy((char*)dp->decoded.payload.bytes, "JT: Hello World (DM Test)", sizeof(dp->decoded.payload.bytes));
                dp->decoded.payload.size = strlen("JT: Hello World (DM Test)");
                if (service) service->sendToMesh(dp);
                snprintf(replyMsg, sizeof(replyMsg), "JT: DM Test sent to !%08x", mp.from);
            }
        } else {
            LOG_ERROR("Nexus: DMTEST Failed - !%08x has no Public Key in NodeDB", mp.from);
            snprintf(replyMsg, sizeof(replyMsg), "JT: DMTEST Error - !%08x Public Key missing", mp.from);
        }
    }
    else if (strcasecmp(cmd, "SENDPOS") == 0) {
        // Logic: Send a proper Position packet back to the commander.
        // If DM, send as PKI back to sender. If Channel, send to broadcast on that channel.
        if (positionModule) {
            bool isDM = (mp.to == myNodeInfo.my_node_num);
            positionModule->sendOurPosition(isDM ? mp.from : NODENUM_BROADCAST, false, mp.channel);
            snprintf(replyMsg, sizeof(replyMsg), "JT: Position injected to %s", isDM ? "DM" : "Channel");
        } else {
            snprintf(replyMsg, sizeof(replyMsg), "JT: PositionModule not available.");
        }
    }
    else if (strncasecmp(cmd, "SENDNODE", 8) == 0 || strncasecmp(cmd, "SN", 2) == 0 ||
             strncasecmp(cmd, "SENDMYNODE", 10) == 0 || strncasecmp(cmd, "SMN", 3) == 0 ||
             strncasecmp(cmd, "SENDMYINFO", 10) == 0 || strncasecmp(cmd, "SMI", 3) == 0) {

        bool isMyNode = strncasecmp(cmd, "SENDMYNODE", 10) == 0 || strncasecmp(cmd, "SMN", 3) == 0 ||
                        strncasecmp(cmd, "SENDMYINFO", 10) == 0 || strncasecmp(cmd, "SMI", 3) == 0;

        const char *cmdPtr = cmd;
        if (isMyNode) {
            if (strncasecmp(cmd, "SENDMYNODE", 10) == 0 || strncasecmp(cmd, "SENDMYINFO", 10) == 0) cmdPtr += 10;
            else cmdPtr += 3;
        } else {
            if (strncasecmp(cmd, "SENDNODE", 8) == 0) cmdPtr += 8;
            else cmdPtr += 2;
        }

        bool customChannel = false;
        bool customHops = false;
        bool isMqtt = false;

        while (*cmdPtr != ' ' && *cmdPtr != '\0') {
            if (toupper(*cmdPtr) == 'C') customChannel = true;
            if (toupper(*cmdPtr) == 'H') customHops = true;
            if (toupper(*cmdPtr) == 'M') isMqtt = true;
            cmdPtr++;
        }
        while (*cmdPtr == ' ') cmdPtr++;

        int targetCh = mp.channel;
        const char *next = nullptr;
        if (customChannel) {
            targetCh = findChannelByNameOrIndex(cmdPtr, &next);
            if (targetCh < 0) {
                snprintf(replyMsg, sizeof(replyMsg), "JT: Invalid channel");
                return;
            }
            cmdPtr = next;
        }

        int hopLim = 3, hopStart = 3;
        if (customHops) {
            hopLim = atoi(cmdPtr);
            while (*cmdPtr && *cmdPtr != ' ') cmdPtr++;
            while (*cmdPtr == ' ') cmdPtr++;
            hopStart = atoi(cmdPtr);
            while (*cmdPtr && *cmdPtr != ' ') cmdPtr++;
            while (*cmdPtr == ' ') cmdPtr++;
        }

        uint32_t targetId = isMyNode ? myNodeInfo.my_node_num : findNodeByShortNameOrHex(cmdPtr);

        auto node = nodeDB->getMeshNode(targetId);
        if (!node || node->long_name[0] == '\0') {
            snprintf(replyMsg, sizeof(replyMsg), "JT: Node %08x not in DB or invalid", targetId);
        } else {
            meshtastic_MeshPacket *np = (router) ? router->allocForSending() : nullptr;
            if (np) {
                np->decoded.portnum = meshtastic_PortNum_NODEINFO_APP;
                np->to = 0xFFFFFFFF;
                np->from = targetId;
                np->hop_limit = hopLim;
                np->hop_start = hopStart;
                np->channel = targetCh;
                if (isMqtt) {
                    np->decoded.has_bitfield = true;
                    np->decoded.bitfield |= BITFIELD_OK_TO_MQTT_MASK;
                }

                meshtastic_User user = meshtastic_User_init_default;
                user.hw_model = node->hw_model;
                user.is_licensed = nexus_is_licensed(node);
                user.role = node->role;
                snprintf(user.id, sizeof(user.id), "!%08x", targetId);
                strncpy(user.long_name, node->long_name, sizeof(user.long_name));
                strncpy(user.short_name, node->short_name, sizeof(user.short_name));
                memcpy(user.public_key.bytes, node->public_key.bytes, sizeof(user.public_key.bytes));
                user.public_key.size = node->public_key.size;

                np->decoded.payload.size = pb_encode_to_bytes(np->decoded.payload.bytes, sizeof(np->decoded.payload.bytes), &meshtastic_User_msg, &user);

                if (service) service->sendToMesh(np);
                snprintf(replyMsg, sizeof(replyMsg), "JT: NodeInfo injected for !%08x on Ch:%d (%s)%s",
                         targetId, targetCh, channels.getByIndex(targetCh).settings.name, isMqtt ? " [MQTT]" : "");
            }
        }
    }
    else if (strncasecmp(cmd, "NAME ", 5) == 0) {
        const char *newName = cmd + 5;
        LOG_INFO("Nexus: Name change to '%s'", newName);
        memset(owner.long_name, 0, sizeof(owner.long_name));
        strncpy(owner.long_name, newName, sizeof(owner.long_name) - 1);
        nodeDB->pickNewNodeNum();
        snprintf(replyMsg, sizeof(replyMsg), "JT: Name set. New NodeNum assigned.");
    }
    else if (strncasecmp(cmd, "OWNER ", 6) == 0) {
        // Syntax: /nexus OWNER <short> <long>
        const char *p = cmd + 6;
        while (*p == ' ') p++;

        char shortName[16] = {0};
        int i = 0;
        while (*p && *p != ' ' && i < 4) { // Meshtastic limits short names to 4 chars
            shortName[i++] = *p++;
        }
        shortName[i] = '\0';

        while (*p == ' ') p++;
        const char *longName = p;

        if (strlen(shortName) > 0 && strlen(longName) > 0) {
            LOG_INFO("Nexus: Owner change to %s / %s", shortName, longName);

            memset(owner.short_name, 0, sizeof(owner.short_name));
            strncpy(owner.short_name, shortName, sizeof(owner.short_name) - 1);

            memset(owner.long_name, 0, sizeof(owner.long_name));
            strncpy(owner.long_name, longName, sizeof(owner.long_name) - 1);

            nodeDB->pickNewNodeNum();
            snprintf(replyMsg, sizeof(replyMsg), "JT: Owner updated to %s / %s. New NodeNum assigned.", shortName, longName);
        } else {
            snprintf(replyMsg, sizeof(replyMsg), "JT: Usage - OWNER <short> <long>");
        }
    }
    else if (strncasecmp(cmd, "TRACE ", 6) == 0 || strncasecmp(cmd, "TR ", 3) == 0) {
        const char *target = cmd + (strncasecmp(cmd, "TR ", 3) == 0 ? 3 : 6);
        while (*target == ' ') target++;

        /*
        // New: Active Nexus TraceRoute
        uint32_t targetId = 0;
        // 1. Try to find by short name
        for (size_t i = 0; i < nodeDB->getNumMeshNodes(); i++) {
            auto n = nodeDB->getMeshNodeByIndex(i);
            if (n && nexus_has_user(n) && strcasecmp(n->short_name, target) == 0) {
                targetId = n->num;
                break;
            }
        }

        // 2. Try hex
        if (targetId == 0) {
            if (*target == '!') targetId = strtoul(target + 1, NULL, 16);
            else {
                char *endptr;
                uint32_t val = strtoul(target, &endptr, 16);
                if (*endptr == '\0' || *endptr == ' ') targetId = val;
            }
        }
        */
        uint32_t targetId = findNodeByShortNameOrHex(target);

        if (targetId != 0 && traceRouteModule) {
            if (trace_active) {
                snprintf(replyMsg, sizeof(replyMsg), "JT: Trace busy. Try again.");
            } else {
                // Nexus: Store context for the asynchronous reply.
                trace_active = true;
                trace_start_time = Time::getMillis(); // Watchdog reference
                trace_reply_to = isDM ? mp.from : 0xFFFFFFFF;
                trace_reply_channel = mp.channel;
                traceReply = {isDM ? mp.from : 0xFFFFFFFF, mp.channel, replyId, toPhoneOnly, toPhoneOnly || !isDM};
                traceRouteModule->setNexusTraceCallback(onNexusTraceResult);
                if (traceRouteModule->startTraceRoute(targetId)) {
                    snprintf(replyMsg, sizeof(replyMsg), "JT: Starting trace to !%08x...", targetId);
                } else {
                    trace_active = false;
                    snprintf(replyMsg, sizeof(replyMsg), "JT: Trace failed to start");
                    trace_reply_to = 0;
                }
            }
        } else {
            snprintf(replyMsg, sizeof(replyMsg), "JT: Invalid trace target '%s'", target);
        }
    }
    else if (strncasecmp(cmd, "GROUP ", 6) == 0) {
        const char *p = cmd + 6;
        while (*p == ' ') p++;

        if (strncasecmp(p, "ADD ", 4) == 0) {
            const char *name = p + 4;
            while (*name == ' ') name++;
            if (*name == '\0') {
                snprintf(replyMsg, sizeof(replyMsg), "JT: Usage GROUP ADD <name>");
            } else {
                // Trim trailing whitespace (\r, \n, space, tab) so that a
                // name stored with a stray newline can still be matched by
                // REM <bare-name>. The 8-char slot cap is preserved.
                const char *end = name + strlen(name);
                while (end > name && (end[-1] == ' ' || end[-1] == '\t' ||
                                       end[-1] == '\r' || end[-1] == '\n')) {
                    end--;
                }
                size_t nameLen = end - name;
                if (nameLen == 0) {
                    snprintf(replyMsg, sizeof(replyMsg), "JT: Usage GROUP ADD <name>");
                } else {
                    // Dedup: if the name (case-insensitive, exact length) is
                    // already in a slot, report the existing slot and skip
                    // the write. Prevents /nexus GROUP ADD foo from
                    // accidentally double-booking slot 0 and slot 1.
                    int existing = -1;
                    for (int i = 0; i < 4; i++) {
                        if (nexus_groups[i][0] == '\0') continue;
                        size_t storedLen = strlen(nexus_groups[i]);
                        if (storedLen == nameLen &&
                            strncasecmp(nexus_groups[i], name, nameLen) == 0) {
                            existing = i;
                            break;
                        }
                    }
                    if (existing != -1) {
                        snprintf(replyMsg, sizeof(replyMsg),
                                 "JT: Group '%.*s' already in slot %d",
                                 (int)nameLen, name, existing);
                    } else {
                        int slot = -1;
                        for (int i = 0; i < 4; i++) {
                            if (nexus_groups[i][0] == '\0') { slot = i; break; }
                        }
                        if (slot == -1) {
                            snprintf(replyMsg, sizeof(replyMsg), "JT: Group slots full (max 4)");
                        } else {
                            size_t copyLen = (nameLen < 8) ? nameLen : 8;
                            memcpy(nexus_groups[slot], name, copyLen);
                            nexus_groups[slot][copyLen] = '\0';
                            snprintf(replyMsg, sizeof(replyMsg), "JT: Group '%s' ADDED to slot %d", nexus_groups[slot], slot);
                        }
                    }
                }
            }
        }
        else if (strncasecmp(p, "REM ", 4) == 0) {
            const char *name = p + 4;
            while (*name == ' ') name++;
            int found = -1;
            for (int i=0; i<4; i++) {
                if (strcasecmp(nexus_groups[i], name) == 0) {
                    nexus_groups[i][0] = '\0';
                    found = i;
                    break;
                }
            }
            if (found != -1) {
                snprintf(replyMsg, sizeof(replyMsg), "JT: Group slot %d REMOVED", found);
            } else {
                snprintf(replyMsg, sizeof(replyMsg), "JT: Group '%s' not found", name);
            }
        }
        else if (strcasecmp(p, "CLEAR") == 0) {
            for (int i=0; i<4; i++) nexus_groups[i][0] = '\0';
            snprintf(replyMsg, sizeof(replyMsg), "JT: All groups CLEARED");
        }
        else if (strcasecmp(p, "LIST") == 0 || *p == '\0') {
            snprintf(replyMsg, sizeof(replyMsg), "JT Groups: [%s] [%s] [%s] [%s]",
                     nexus_groups[0], nexus_groups[1], nexus_groups[2], nexus_groups[3]);
        }
    }
    else if (strncasecmp(cmd, "CRON ", 5) == 0 || strcasecmp(cmd, "CRON") == 0) {
        const char *p = (strlen(cmd) > 5) ? cmd + 5 : "";
        while (*p == ' ') p++;

        if (strncasecmp(p, "ADD ", 4) == 0) {
            const char *modeStr = p + 4;
            while (*modeStr == ' ') modeStr++;

            uint8_t mode = 0;
            uint32_t value = 0;
            const char *command = nullptr;
            uint32_t wallClock = getTime(false);
            if (!nexus::parseCron(modeStr, wallClock, mode, value, command)) {
                snprintf(replyMsg, sizeof(replyMsg),
                         "JT: CRON expects PER >=60, SCH HH:MM UTC, or future AT timestamp; command 1-63 bytes");
            } else {
                int slot = -1;
                for (int i = 0; i < 8; ++i)
                    if (!cron_table[i].active) {
                        slot = i;
                        break;
                    }
                if (slot < 0) {
                    snprintf(replyMsg, sizeof(replyMsg), "JT: CRON table full (max 8)");
                } else {
                    auto &job = cron_table[slot];
                    job.active = 1;
                    job.mode = mode;
                    job.value = value;
                    job.last_run = mode == 0 ? static_cast<uint32_t>(Time::getUptimeSecs()) : wallClock;
                    cronLastRunMs[slot] = Time::getMillis();
                    strcpy(job.command, command);
                    snprintf(replyMsg, sizeof(replyMsg), "JT: CRON Job %d ADDED.", slot);
                }
            }
        }
        else if (strncasecmp(p, "REM ", 4) == 0) {
            int id = atoi(p + 4);
            if (id >= 0 && id < 8) {
                cron_table[id].active = 0;
                snprintf(replyMsg, sizeof(replyMsg), "JT: CRON Job %d REMOVED.", id);
            } else snprintf(replyMsg, sizeof(replyMsg), "JT: Invalid CRON ID");
        }
        else if (strcasecmp(p, "LIST") == 0 || *p == '\0') {
            startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "CRON Table");
            static char line[64];
            for (int i = 0; i < 8; i++) {
                if (!cron_table[i].active) continue;
                memset(line, 0, sizeof(line));
                const char* mNames[] = {"PER", "SCH", "AT"};
                snprintf(line, sizeof(line), "[%d] %s:%u -> %s\n", i, mNames[cron_table[i].mode], (unsigned int)cron_table[i].value, cron_table[i].command);
                appendPaging(line);
            }
            memset(line, 0, sizeof(line));
            snprintf(replyMsg, sizeof(replyMsg), "JT: Paging CRON list...");
        }
    }
    else if (strncasecmp(cmd, "NIGN ", 5) == 0 || strcasecmp(cmd, "NIGN") == 0) {
        const char *p = (strlen(cmd) > 5) ? cmd + 5 : "";
        while (*p == ' ') p++;

        if (strncasecmp(p, "ADD ", 4) == 0) {
            uint32_t targetNum = parseNexusID(p + 4);
            if (targetNum == 0) {
                snprintf(replyMsg, sizeof(replyMsg), "JT: Invalid node ID");
            } else if (nexus_ignore_count >= 64) {
                snprintf(replyMsg, sizeof(replyMsg), "JT: NIGN list full (64)");
            } else {
                bool exists = false;
                for (int i=0; i<nexus_ignore_count; i++) if (nexus_ignore_list[i] == targetNum) exists = true;
                if (exists) {
                    snprintf(replyMsg, sizeof(replyMsg), "JT: !%08x already ignored", (unsigned int)targetNum);
                } else {
                    nexus_ignore_list[nexus_ignore_count++] = targetNum;
                    snprintf(replyMsg, sizeof(replyMsg), "JT: !%08x ADDED to ignores", (unsigned int)targetNum);
                }
            }
        }
        else if (strncasecmp(p, "REM ", 4) == 0) {
            uint32_t targetNum = parseNexusID(p + 4);
            int idx = -1;
            for (int i=0; i<nexus_ignore_count; i++) if (nexus_ignore_list[i] == targetNum) { idx = i; break; }
            if (idx == -1) {
                snprintf(replyMsg, sizeof(replyMsg), "JT: !%08x not in list", (unsigned int)targetNum);
            } else {
                if (idx < nexus_ignore_count - 1) {
                    memmove(&nexus_ignore_list[idx], &nexus_ignore_list[idx + 1], (nexus_ignore_count - idx - 1) * sizeof(uint32_t));
                }
                nexus_ignore_count--;
                snprintf(replyMsg, sizeof(replyMsg), "JT: !%08x REMOVED", (unsigned int)targetNum);
            }
        }
        else if (strcasecmp(p, "LIST") == 0 || *p == '\0') {
            startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "JT Ignores");
            if (nexus_ignore_count == 0) {
                appendPaging("List empty.\n");
            } else {
                for (int i=0; i<nexus_ignore_count; i++) {
                    char line[48];
                    auto n = nodeDB->getMeshNode(nexus_ignore_list[i]);
                    snprintf(line, sizeof(line), "[%d] !%08x (%s)\n", i, (unsigned int)nexus_ignore_list[i],
                             (n && n->short_name[0]) ? n->short_name : "???");
                    appendPaging(line);
                }
            }
            snprintf(replyMsg, sizeof(replyMsg), "JT: Paging ignore list (%d nodes)...", nexus_ignore_count);
        }
    }
    else if (strncasecmp(cmd, "HELP ", 5) == 0 || strcasecmp(cmd, "HELP") == 0) {
        const char *sub = (strlen(cmd) > 5) ? cmd + 5 : "";
        while (*sub == ' ') sub++;

        if (*sub == '\0') {
            startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "Global Help");
            appendPaging("STEALTH: REBROADCAST, BASETEXTS\n");
            appendPaging("FIREWALL: DROPS, DROP, OUTDROP, PRALLOW, PRBLOCK, SS, SETHEX, SETHEXOUT, RL, FSIG\n");
            appendPaging("SIGINT: RSSILOG, NETSCAN, CLIENTSCAN, FREQSCAN, NOISE, PPING, CHASH, DBINFO, MODULES, INFRA, RELAYS, SRELAY, SEARCHKEY, HOPSTATS, TRANSSTATS, CHANNELSTATS, SORTEDCHANNELSTATS, SORTEDPACKETSTATS, LORACHIPSTATS\n");
            appendPaging("SEND: TEXTC, DM, REDIRECT (R), SENDPOS, SENDNODE\n");
            appendPaging("ADMIN: SKEY, DM-FORWARD, SAVE, LOAD, BD, SETCHNAME, SETCHPSK, ADDCH, DELCH, SETROLE, SETPOWER, OKTOMQTT, BELL, PLAY, STARWARS, FULLCONFIGDUMP\n");
            appendPaging("SYSTEM: SYS, HW, TELEMETRY, POS, INFO, COMPILATION, VAULT, STATS, SS, VS, HEAP, NODES, FNODE, GROUP, TIME, UTIL, PHONE, PMGT, SMI, GPIO, WIFIOTA, MCUINFO, SENSORS\n");
            appendPaging("TARGETS: FAV, UNFAV, IGNORE, UNIGNORE, NIGN\n");
            appendPaging("STORAGE: LS, RM, CAT, FS\n");
            appendPaging("Try: HELP <topic> (e.g. HELP DROP)");
            snprintf(replyMsg, sizeof(replyMsg), "JT: Paging help categories...");
        } else if (strcasecmp(sub, "DROPS") == 0) {
            startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "Help DROPS");
            appendPaging("Usage: DROP <tag> | UNDROP <tag>\nTags:\n");
            appendPaging("NI:NodeInfo PO:Pos TL:Tele\nNB:Neighbr TR:Trace (All)\nTRR:TR Rem TRL:TR Loc PV:Priv\nH6:H6 H7:H7 AK:ACKs FB:Fav\nPK:PKI PKR:Rem PKL:Loc PH:PHop\nSF:S&F AD:Admin TX:Text\nKV:KeyV AL:Alert WP:Waypoint\nTRD:TR Dir TDR:Rem TDL:Loc\nSS:SS-Handle TA:TraceAlt\nMA:MetaAlt REQ:Requests\nPos sub: PM:Manual PG:GPS POO:Other\nTele sub: TD:Dev TP:Pwr TE:Env TO:Oth\nRL: RLTX RLTR RLOT RLNI RLPO RLTL RLPR\nBcast REQ: BRNI BRTL BRPO (broadcast-only)");
            snprintf(replyMsg, sizeof(replyMsg), "JT: Paging DROPS list help...");
        } else if (strcasecmp(sub, "TEXT") == 0) {
            startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "Help TEXT/SEND");
            appendPaging("TEXTC <ch> <msg>: Standard on ch.\n");
            appendPaging("DM [!id|name] [msg]: Direct message.");
            snprintf(replyMsg, sizeof(replyMsg), "JT: Paging TEXT/SEND help...");
        } else if (strcasecmp(sub, "FW") == 0) {
            startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "Help FIREWALL");
            appendPaging("Usage: FIREWALL | FW   -> Aggregated status (DROPS, OUTDROPS, HEX, INVALID, PRALLOW, PRBLOCK)\n");
            appendPaging("Usage: FIREWALL <sec> | FW <sec>   -> Delegate to single section\n");
            appendPaging("Sections: DROPS, OUTDROPS, INVALIDS, PRALLOW, PRBLOCK\n");
            appendPaging("Output: paged (P1:, P2:, ...). Sub-sections run via internal handler dispatch.\n");
            appendPaging("Note: HEXDROPS / HEXOUTDROPS / FIREWALLSTATS are still standalone.");
            snprintf(replyMsg, sizeof(replyMsg), "JT: Paging FW help...");
        } else if (strcasecmp(sub, "DROP") == 0 || strcasecmp(sub, "FIREWALL") == 0 || strcasecmp(sub, "OUTDROP") == 0) {
            startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "Help FIREWALL");
            appendPaging("Usage: [UN]DROP <target> | [UN]OUTDROP <target>\n");
            appendPaging("Targets: NI, PO, TL, NB, TR, PV, AD, TX, KV, AL, WP, SF, ACKS, FB, SS, PK, RNI, RPO, RTL, BRNI, BRTL, BRPO, H6, H7, TO, RL_*\n");
            appendPaging("Trace: TRR, TRL, TRD, TRDR, TRDL\n");
            appendPaging("OutTargets: NI, PO, TL, TR, AD\n");
            appendPaging("Bulk: SETHEX [hex] | SETHEXOUT [hex]");
            snprintf(replyMsg, sizeof(replyMsg), "JT: Paging FIREWALL help...");
        } else if (strcasecmp(sub, "FEATURES") == 0) {
            // Nexus v66: FEATURES help. Four bits today (FORWARD_OPAQUE_TO_PHONE,
            // FORWARD_FOREIGN_PKI_TO_PHONE, DISABLE_AGC_RESET, LATEBASE). RAM-only — persist via /nexus SAVE.
            startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "Help FEATURES");
            appendPaging("Usage: FEATURES [LIST]                  -> Show mask + named bits ON/OFF\n");
            appendPaging("Usage: FEATURES OFF                     -> Clear all bits\n");
            appendPaging("Usage: FEATURES <hex>|0x<hex>           -> Set mask directly (uint64)\n");
            appendPaging("Usage: FEATURES <NAME> [ON|OFF]         -> Toggle named bit (default ON)\n");
            appendPaging("Names: FORWARD_OPAQUE_TO_PHONE (alias OPAQUE), FORWARD_FOREIGN_PKI_TO_PHONE (alias PKI), DISABLE_AGC_RESET (alias AGC), LATEBASE (alias LB)\n");
            appendPaging("Persistence: RAM-only. /nexus SAVE to write features_mask to /nexus.bin vault.");
            snprintf(replyMsg, sizeof(replyMsg), "JT: Paging FEATURES help...");
        } else if (strcasecmp(sub, "SIGINT") == 0 || strcasecmp(sub, "SCAN") == 0) {
            startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "Help SIGINT/SCAN");
            appendPaging("RSSILOG: Signal ring buffer.\nNETSCAN: Tactical survey.\nCLIENTSCAN [!id]: Repeat mapping.\n");
            appendPaging("FREQSCAN <MHz> <BW>: Spectrum test.\nNOISE: High-res floor snapshot.\n");
            appendPaging("RELAYS: Global relay ID stats.\nSRELAY <*hex>: Search relay ID names.\n");
            appendPaging("SEARCHKEY <base64>: Find nodes by PK.\n");
            appendPaging("HOPSTATS: Hop distribution.\nPPING: Public ping config (EMOJI <OFF|INDIRECT|ALWAYS>).\nMODULES: List loaded Meshtastic modules.");
            snprintf(replyMsg, sizeof(replyMsg), "JT: Paging SIGINT help...");
        } else if (strcasecmp(sub, "ADMIN") == 0 || strcasecmp(sub, "SYSTEM") == 0) {
            startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "Help ADMIN/SYS");
            appendPaging("SKEY [1|2] [Key]: Dynamic keys.\nSETROLE <role>.\nSETPOWER <dBm>.\n");
            appendPaging("SETCHNAME <idx> <name>.\nSETCHPSK <idx> <psk>.\nADDCH <name> [psk].\nDELCH <idx>.\nSAVE/LOAD: Vault management.\n");
            appendPaging("SYS: Core health.\nHW: Radio/Clock info.\nTELEMETRY: Power/Env report.\nUTIL: Ch/TX airtime.\nWIFIOTA: ESP32 OTA loader info (project, version, IDF, SHA256, built).\nMCUINFO: MCU identity + runtime state (chip, flash, heap, MAC/DeviceID, up-time).\nSENSORS: I2C devices detected at boot + live power/env/values + GPS.");
            snprintf(replyMsg, sizeof(replyMsg), "JT: Paging ADMIN/SYS help...");
        } else if (strcasecmp(sub, "BD") == 0) {
            startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "Help BD");
            appendPaging("BD SET <1..300>: Broadcast reply / paging max delay (seconds).\n");
            appendPaging("BD GET: Report current value.\n");
            appendPaging("Jitter floor stays at 100ms; range scales to N*1000ms.\n");
            appendPaging("Default 5. Persisted to vault on SAVE.");
            snprintf(replyMsg, sizeof(replyMsg), "JT: Paging ADMIN/SYS help...");
        } else if (strcasecmp(sub, "IDENTITY") == 0 || strcasecmp(sub, "NAME") == 0) {
            startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "Help IDENTITY");
            appendPaging("NAME <name>: Identity rotation + reboot.");
            snprintf(replyMsg, sizeof(replyMsg), "JT: Paging IDENTITY help...");
        } else if (strcasecmp(sub, "GPIO") == 0) {
            startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "Help GPIO");
            appendPaging("W <pin> <H|L>: Write digital value.\n");
            appendPaging("R <pin>: Read digital value (INPUT_PULLUP).\n");
            appendPaging("M <pin> <IN|OUT|PU>: Set pin mode.\n");
            appendPaging("WATCH <pin|OFF>: Alert on pin change.");
            snprintf(replyMsg, sizeof(replyMsg), "JT: Paging GPIO help...");
        } else if (strcasecmp(sub, "PPING") == 0) {
            startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "Help PPING");
            appendPaging("SubCmds: ON, OFF, RSSI, SNR, FREQERR, FLAGS, DIRECT IGNORE, REMOTE IGNORE, CHAN [ON/OFF] [ID], HEADER [text|OFF], EMOJI [OFF|INDIRECT|ALWAYS], SETHEX [hex]");
            appendPaging("EMOJI: OFF=texto completo, INDIRECT=emoji si hops>0, ALWAYS=emoji siempre.");
            snprintf(replyMsg, sizeof(replyMsg), "JT: Paging PPING help...");
        } else if (strcasecmp(sub, "SKEY") == 0) {
            startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "Help SKEY");
            appendPaging("Usage: SKEY [1|2|3|4] [Base64|Hex|OFF]\nSet secondary dynamic master keys for remote auth.");
            snprintf(replyMsg, sizeof(replyMsg), "JT: Paging SKEY help...");
        } else if (strcasecmp(sub, "LORA") == 0 || strcasecmp(sub, "PRESET") == 0 || strcasecmp(sub, "SETLORA") == 0) {
            startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "Help LORA");
            appendPaging("PRESET: SF, LF, MS, MF, SS, ST, LT, SPM, P1, P7, NAP-1..5, NAPN-1..5\n");
            appendPaging("SETLORA: [SF] [BW] [CR] [Freq]");
            snprintf(replyMsg, sizeof(replyMsg), "JT: Paging LORA help...");
        } else if (strcasecmp(sub, "TM") == 0) {
            startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "Help TM");
            appendPaging("Config Keys: ENABLE, DEDUP, DEDUP_PREC, DEDUP_INT, CACHE_REPLY, CACHE_HOPS, RATELIMIT, RATELIMIT_WIN, RATELIMIT_MAX, UNKNOWN, UNKNOWN_TH, EXHAUST_TELE, EXHAUST_POS, PRESERVE");
            snprintf(replyMsg, sizeof(replyMsg), "JT: Paging TM help...");
        } else if (strcasecmp(sub, "RL") == 0 || strcasecmp(sub, "RATELIMIT") == 0) {
            startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "Help RATELIMIT (Nexus)");
            appendPaging("Usage: RATELIMIT <TYPE> <pkts/hora>");
            appendPaging("Types: TEXT/TX, TRACE/TR, OTHER/OT, NODEINFO/NI, POSITION/PO, TELEMETRY/TL, PRIVATE/PR");
            appendPaging("Sin argumento: muestra los 7 límites actuales.");
            appendPaging("STATUS/STAT/BUCKETS: estado live de los buckets (igual que RATELIMITSTAT).");
            appendPaging("IMPORTANTE: Activar también DROP RL_<TYPE> (bits 29-31, 42-45) para que el filtro aplique.");
            snprintf(replyMsg, sizeof(replyMsg), "JT: Paging RATELIMIT help...");
        } else if (strcasecmp(sub, "RLS") == 0 || strcasecmp(sub, "RATELIMITSTAT") == 0) {
            startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "Help RATELIMITSTAT");
            appendPaging("Usage: RATELIMITSTAT (alias RLS)");
            appendPaging("Alias equivalente: RATELIMIT STATUS / RATELIMIT STAT / RATELIMIT BUCKETS");
            appendPaging("Muestra estado live de los 7 buckets: tokens actuales / límite / % lleno / En? / Seen / Pass / Drop / Drop%.");
            appendPaging("Une los contadores de FWS (Seen/Pass/Drop) con el estado de tokens en tiempo real.");
            appendPaging("El shortcut 'RL' pertenece a RATELIMIT (no a RSSILOG como se decía antes).");
            appendPaging("RLS = 3-char mnemonic para esta vista de estado (mismo prefijo RL pero específico).");
            snprintf(replyMsg, sizeof(replyMsg), "JT: Paging RATELIMITSTAT help...");
        } else if (strcasecmp(sub, "PL") == 0 || strcasecmp(sub, "RSSILOG") == 0) {
            startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "Help RSSILOG");
            appendPaging("Usage: RSSILOG (alias PL) [N]");
            appendPaging("Muestra los últimos N paquetes SIGINT escuchados (default 5, max 32).");
            appendPaging("Formato: ID,Hops,HopsTotal,RSSI,SNR,Relay,Age");
            appendPaging("El shortcut 'PL' (Ping Log) reemplaza al antiguo 'RL' que estaba en dead code.");
            appendPaging("El shortcut 'RL' actual pertenece a RATELIMIT (no a este comando).");
            snprintf(replyMsg, sizeof(replyMsg), "JT: Paging RSSILOG help...");
        } else if (strcasecmp(sub, "FSIG") == 0) {
            startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "Help FSIG");
            appendPaging("Signature firewall: drops decoded packets whose payload contains a configured byte sequence.");
            appendPaging("Usage: FSIG LIST");
            appendPaging("       FSIG SET <1-8> <hex|base64|text>");
            appendPaging("       FSIG OFF <1-8>");
            appendPaging("Detecta automáticamente el tipo (hex/base64/texto) y lo recuerda para el display.");
            appendPaging("Max 16 bytes por firma. Aplica a TODOS los canales decoded (excepto canales Nexus).");
            snprintf(replyMsg, sizeof(replyMsg), "JT: Paging FSIG help...");
        } else if (strcasecmp(sub, "CRON") == 0) {
            startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "Help CRON");
            appendPaging("PER <secs> <cmd>: Periodic interval.\nSCH <HH:MM> <cmd>: Daily wall-clock.\nAT <unix_ts> <cmd>: One-shot schedule.\nLIST: Show jobs.\nREM <id>: Delete job.");
            snprintf(replyMsg, sizeof(replyMsg), "JT: Paging CRON help...");
        } else if (strcasecmp(sub, "NIGN") == 0 || strcasecmp(sub, "IGNORE") == 0) {
            startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "Help NIGN/IGNORE");
            appendPaging("IGNORE <!id|name>: RAM-only ignore.\nNIGN ADD <!id>: Persistent ignore (Vault).\nNIGN REM <!id>: Remove from vault.\nNIGN LIST: Show all persistent ignores.");
            snprintf(replyMsg, sizeof(replyMsg), "JT: Paging NIGN help...");
        } else if (strcasecmp(sub, "NODE") == 0 || strcasecmp(sub, "TARGET") == 0) {
            startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "Help TARGETS");
            appendPaging("FAV <!id|name>: Set favorite.\nUNFAV <!id|name>: Unset favorite.\nIGNORE <!id|name>: RAM-only ignore.\nNIGN ADD <!id>: Persistent ignore.");
            snprintf(replyMsg, sizeof(replyMsg), "JT: Paging TARGET help...");
        } else {
            snprintf(replyMsg, sizeof(replyMsg), "JT: No help for '%s'", sub);
        }
    }
    else if (strcasecmp(cmd, "TIME") == 0) {
        uint32_t utc = getTime(false);
        uint32_t local = getTime(true);
        RTCQuality q = getRTCQuality();

        time_t t_local = (time_t)local;
        struct tm *tm_local = gmtime(&t_local);
        char timeStr[32];
        strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", tm_local);

        snprintf(replyMsg, sizeof(replyMsg), "JT TIME (%s):\nUTC: %u\nLOC: %s",
                 RtcName(q), (unsigned int)utc, timeStr);
    }
    else if (strncasecmp(cmd, "SETTIME ", 8) == 0) {
        uint32_t newTs = strtoul(cmd + 8, NULL, 10);
        if (newTs < 1700000000) { // Safety check (before 2023)
            snprintf(replyMsg, sizeof(replyMsg), "JT: Invalid timestamp");
        } else {
            struct timeval tv;
            tv.tv_sec = newTs;
            tv.tv_usec = 0;
            perhapsSetRTC(RTCQualityNTP, &tv, true);
            snprintf(replyMsg, sizeof(replyMsg), "JT: Time set to %u", (unsigned int)newTs);
        }
    }
    else if (strcasecmp(cmd, "INFO") == 0) {
        uint8_t realMac[6];
        getMacAddr(realMac);
        uint32_t activeID = myNodeInfo.my_node_num;
        if (activeID == 0) activeID = nodeDB->getNodeNum(); // Fallback to our own node num

        snprintf(replyMsg, sizeof(replyMsg),
                 "JT INFO:\n!%08x [%.8s]\nVer: %s\nRole: %s\nMAC: %02x:%02x:%02x:%02x:%02x:%02x",
                 (unsigned int)activeID, owner.short_name,
                 optstr(APP_VERSION),
                 getRoleName(config.device.role),
                 realMac[0], realMac[1], realMac[2], realMac[3], realMac[4], realMac[5]);
    }
    else if (strcasecmp(cmd, "COMPILATION") == 0 || strcasecmp(cmd, "COMP") == 0) {
        // Resolve MCU arch from compile-time Arduino defines.
#if defined(ARDUINO_ARCH_ESP32)
        const char *mcuArch = "esp32";
#elif defined(ARDUINO_ARCH_NRF52)
        const char *mcuArch = "nrf52";
#elif defined(ARDUINO_ARCH_RP2040)
        const char *mcuArch = "rp2040";
#elif defined(ARDUINO_ARCH_STM32)
        const char *mcuArch = "stm32";
#elif defined(ARDUINO_ARCH_PORTDUINO)
        const char *mcuArch = "portduino";
#else
        const char *mcuArch = "unknown";
#endif

        // Build flag summary from compile-time Nexus defines. Order-stable
        // (AIRTAG, ADV) for predictable diffs across rebuilds.
        static char buildFlags[32];
        buildFlags[0] = '\0';
        size_t flagLen = 0;
#if defined(NEXUS_ENABLE_AIRTAG)
        if (flagLen > 0 && flagLen < sizeof(buildFlags) - 1) {
            buildFlags[flagLen++] = '+';
            buildFlags[flagLen] = '\0';
        }
        strncpy(buildFlags + flagLen, "AIRTAG", sizeof(buildFlags) - flagLen - 1);
        flagLen = strlen(buildFlags);
#endif
#if defined(NEXUS_ENABLE_ADV)
        if (flagLen > 0 && flagLen < sizeof(buildFlags) - 1) {
            buildFlags[flagLen++] = '+';
            buildFlags[flagLen] = '\0';
        }
        strncpy(buildFlags + flagLen, "ADV", sizeof(buildFlags) - flagLen - 1);
#endif
        (void)flagLen; // referenced only when AIRTAG/ADV are compiled in
        if (buildFlags[0] == '\0') strncpy(buildFlags, "none", sizeof(buildFlags) - 1);

        // Format build date from BUILD_EPOCH (Unix seconds, midnight UTC,
        // injected by bin/platformio-custom.py). Fall back to "unknown"
        // for test/native builds where the define may be absent.
#ifdef BUILD_EPOCH
        time_t be = (time_t)BUILD_EPOCH;
        struct tm *btm = gmtime(&be);
        char buildDate[16];
        strftime(buildDate, sizeof(buildDate), "%Y-%m-%d", btm);
#else
        const char *buildDate = "unknown";
#endif

        snprintf(replyMsg, sizeof(replyMsg),
                 "JT COMPILATION:\nVer: %s\nTgt: %s (%s)\nRepo: %s\nBuilt: %s\nFlags: %s",
                 optstr(APP_VERSION),
                 optstr(APP_ENV), mcuArch,
                 optstr(APP_REPO),
                 buildDate,
                 buildFlags);
    }
    // Nexus-28006 (2026-09-16) — code review 3.2 fix: a dead
    // duplicate RELAYS block used to sit here (the live handler earlier in
    // the chain always caught it first); the empty block was deleted.
    else if (strcasecmp(cmd, "TX ON") == 0) {
        LOG_INFO("Nexus: Enabling LoRa TX");
        config.lora.tx_enabled = true;
        nodeDB->saveToDisk(SEGMENT_CONFIG);
        snprintf(replyMsg, sizeof(replyMsg), "JT: LoRa TX ENABLED");
    }
    else if (strcasecmp(cmd, "TX OFF") == 0) {
        LOG_INFO("Nexus: Disabling LoRa TX (Listen-Only)");
        config.lora.tx_enabled = false;
        nodeDB->saveToDisk(SEGMENT_CONFIG);
        snprintf(replyMsg, sizeof(replyMsg), "JT: LoRa TX DISABLED (Silent)");
    }
    else if (strcasecmp(cmd, "GOODPRACTICES") == 0 ||
             strcasecmp(cmd, "BUENASPRACTICAS") == 0 ||
             strcasecmp(cmd, "GP") == 0 ||
             strcasecmp(cmd, "BP") == 0) {
        // /nexus GOODPRACTICES (aliases BUENASPRACTICAS, GP, BP)
        // Applies airtime best practices:
        // - NodeInfo broadcast interval: 72 hours (259200s)
        // - Position broadcast interval: 72 hours (259200s)
        // - Smart position: disabled (false)
        // - Active telemetry intervals < 12h (43200s) bumped to 24h (86400s); disabled/0 left untouched.
        config.device.node_info_broadcast_secs = 259200;
        config.position.position_broadcast_secs = 259200;
        config.position.position_broadcast_smart_enabled = false;

        const uint32_t twelveHoursSecs = 43200;
        const uint32_t twentyFourHoursSecs = 86400;
        int telemBumped = 0;

        if (moduleConfig.telemetry.device_telemetry_enabled &&
            moduleConfig.telemetry.device_update_interval > 0 &&
            moduleConfig.telemetry.device_update_interval < twelveHoursSecs) {
            moduleConfig.telemetry.device_update_interval = twentyFourHoursSecs;
            telemBumped++;
        }
        if (moduleConfig.telemetry.environment_measurement_enabled &&
            moduleConfig.telemetry.environment_update_interval > 0 &&
            moduleConfig.telemetry.environment_update_interval < twelveHoursSecs) {
            moduleConfig.telemetry.environment_update_interval = twentyFourHoursSecs;
            telemBumped++;
        }
        if (moduleConfig.telemetry.air_quality_enabled &&
            moduleConfig.telemetry.air_quality_interval > 0 &&
            moduleConfig.telemetry.air_quality_interval < twelveHoursSecs) {
            moduleConfig.telemetry.air_quality_interval = twentyFourHoursSecs;
            telemBumped++;
        }
        if (moduleConfig.telemetry.power_measurement_enabled &&
            moduleConfig.telemetry.power_update_interval > 0 &&
            moduleConfig.telemetry.power_update_interval < twelveHoursSecs) {
            moduleConfig.telemetry.power_update_interval = twentyFourHoursSecs;
            telemBumped++;
        }
        if (moduleConfig.telemetry.health_measurement_enabled &&
            moduleConfig.telemetry.health_update_interval > 0 &&
            moduleConfig.telemetry.health_update_interval < twelveHoursSecs) {
            moduleConfig.telemetry.health_update_interval = twentyFourHoursSecs;
            telemBumped++;
        }

        nodeDB->saveToDisk(SEGMENT_CONFIG | SEGMENT_MODULECONFIG);
        if (service) service->configChanged.notifyObservers(nullptr);

        snprintf(replyMsg, sizeof(replyMsg),
                 "JT: Good Practices applied: NI=72h, POS=72h, SmartPos=OFF, TelemBumped=%d (to 24h)", telemBumped);
    }
    else if (strcasecmp(cmd, "OKTOMQTT") == 0 ||
             strncasecmp(cmd, "OKTOMQTT ", 9) == 0 ||
             strcasecmp(cmd, "OKMQTT") == 0 ||
             strncasecmp(cmd, "OKMQTT ", 7) == 0 ||
             strcasecmp(cmd, "OMQ") == 0 ||
             strncasecmp(cmd, "OMQ ", 4) == 0) {
        // /nexus OKTOMQTT (alias OKMQTT, OMQ) - read/write the LoRa config's
        // ok_to_mqtt bit (proto field config_ok_to_mqtt, bit 105 in the
        // LocalConfig.lora message). Router.cpp:652 reads this live each time
        // a packet is sent, so changes take effect immediately on the next
        // outgoing packet. Persisted via nodeDB->saveToDisk(SEGMENT_CONFIG),
        // same pattern as TX ON/OFF.
        size_t offset = 0;
        if (strncasecmp(cmd, "OKTOMQTT", 8) == 0) offset = 8;
        else if (strncasecmp(cmd, "OKMQTT", 6) == 0) offset = 6;
        else offset = 3; // OMQ
        if (cmd[offset] == ' ') offset++;
        const char *p = cmd + offset;
        while (*p == ' ') p++;

        if (*p == '\0' || strcasecmp(p, "STATUS") == 0 || strcasecmp(p, "STAT") == 0) {
            // Bare or explicit STATUS: print current state.
            snprintf(replyMsg, sizeof(replyMsg),
                "JT OKTOMQTT: %s (LoRa config bit 105, ok_to_mqtt on outgoing packets)",
                config.lora.config_ok_to_mqtt ? "ON" : "OFF");
        } else if (strcasecmp(p, "ON") == 0 || strcasecmp(p, "TRUE") == 0 ||
                   strcasecmp(p, "ENABLE") == 0 || strcasecmp(p, "1") == 0) {
            LOG_INFO("Nexus: OKTOMQTT set to ON");
            config.lora.config_ok_to_mqtt = true;
            nodeDB->saveToDisk(SEGMENT_CONFIG);
            snprintf(replyMsg, sizeof(replyMsg),
                "JT: OKTOMQTT set to ON (saved; next outgoing packet will carry ok_to_mqtt=1)");
        } else if (strcasecmp(p, "OFF") == 0 || strcasecmp(p, "FALSE") == 0 ||
                   strcasecmp(p, "DISABLE") == 0 || strcasecmp(p, "0") == 0) {
            LOG_INFO("Nexus: OKTOMQTT set to OFF");
            config.lora.config_ok_to_mqtt = false;
            nodeDB->saveToDisk(SEGMENT_CONFIG);
            snprintf(replyMsg, sizeof(replyMsg),
                "JT: OKTOMQTT set to OFF (saved; next outgoing packet will carry ok_to_mqtt=0)");
        } else {
            snprintf(replyMsg, sizeof(replyMsg),
                "JT OKTOMQTT: unknown arg '%s'\nUsage: OKTOMQTT [ON|OFF|STATUS]", p);
        }
    }
    else if (strcasecmp(cmd, "BELL") == 0) {
        if (externalNotificationModule) {
            externalNotificationModule->triggerNexusBell();
            snprintf(replyMsg, sizeof(replyMsg), "JT: Local BELL triggered");
        } else {
            snprintf(replyMsg, sizeof(replyMsg), "JT: ExternalNotificationModule unavailable");
        }
    }
    else if (strncasecmp(cmd, "PLAY ", 5) == 0) {
        const char *melody = cmd + 5;
        while (*melody == ' ') melody++;
        if (externalNotificationModule) {
            externalNotificationModule->playRTTTL(melody);
            snprintf(replyMsg, sizeof(replyMsg), "JT: Playing custom RTTTL...");
        } else {
            snprintf(replyMsg, sizeof(replyMsg), "JT: ExtNotification unavailable");
        }
    }
    else if (strcasecmp(cmd, "STARWARS") == 0) {
        const char *sw = "StarWars:d=4,o=5,b=45:32p,32f#,32f#,32f#,8b.,8f#.6,32e6,32d#6,32c#6,8b.6,16f#.6,32e6,32d#6,32c#6,8b.6,16f#.6,32e6,32d#6,32e6,8c#.6,32f#,32f#,32f#,8b.,8f#.6,32e6,32d#6,32c#6,8b.6,16f#.6,32e6,32d#6,32c#";
        if (externalNotificationModule) {
            externalNotificationModule->playRTTTL(sw);
            snprintf(replyMsg, sizeof(replyMsg), "JT: May the Force be with you...");
        } else {
            snprintf(replyMsg, sizeof(replyMsg), "JT: ExtNotification unavailable");
        }
    }
    else if (strncasecmp(cmd, "PPING", 5) == 0) {
        const char *p = cmd + 5;
        while (*p == ' ') p++;

        // Show status if no specific sub-command
        if (*p == '\0') {
            char chBuf[32] = {0};
            int chLen = 0;
            for(int i=0; i<8; i++) {
                if (ping_channels[i]) chLen += snprintf(chBuf + chLen, sizeof(chBuf) - chLen, "%d ", i);
            }
            snprintf(replyMsg, sizeof(replyMsg), "JT PPING:\nGlobal:%d RSSI:%d SNR:%d FE:%d Flags:%d DI:%d RI:%d DM:%d EM:%d\nChans:%s Custom:%d",
                     public_ping, ping_rssi, ping_snr, ping_freqerr, ping_bitmask, ping_ignore_direct, ping_ignore_remote, ping_allow_dm, ping_emoji_mode, chBuf, custom_pping_active);
        }
        else if (strncasecmp(p, "HEADER ", 7) == 0) {
            const char *h = p + 7;
            while (*h == ' ') h++;
            if (strcasecmp(h, "OFF") == 0) {
                custom_pping_active = false;
                snprintf(replyMsg, sizeof(replyMsg), "JT: Custom PPING Header DISABLED.");
            } else {
                memset(pping_header, 0, sizeof(pping_header));
                strncpy(pping_header, h, 63);
                custom_pping_active = true;
                snprintf(replyMsg, sizeof(replyMsg), "JT: Custom PPING Header set to '%s'", pping_header);
            }
        }
        else if (strncasecmp(p, "EMOJI ", 6) == 0) {
            const char *e = p + 6;
            while (*e == ' ') e++;
            if (strcasecmp(e, "OFF") == 0) {
                ping_emoji_mode = 0;
                snprintf(replyMsg, sizeof(replyMsg), "JT: Ping response in TEXT mode.");
            } else if (strcasecmp(e, "INDIRECT") == 0 || strcasecmp(e, "INDR") == 0) {
                ping_emoji_mode = 1;
                snprintf(replyMsg, sizeof(replyMsg), "JT: Ping response in EMOJI mode for INDIRECT pings.");
            } else if (strcasecmp(e, "ALWAYS") == 0 || strcasecmp(e, "ALW") == 0) {
                ping_emoji_mode = 2;
                snprintf(replyMsg, sizeof(replyMsg), "JT: Ping response in ALWAYS-EMOJI mode.");
            } else {
                snprintf(replyMsg, sizeof(replyMsg), "JT: PPING EMOJI <OFF|INDIRECT|ALWAYS>");
            }
        }
        else if (strcasecmp(p, "ON") == 0) {
            public_ping = true;
            snprintf(replyMsg, sizeof(replyMsg), "JT: Public Ping ENABLED");
        } else if (strcasecmp(p, "OFF") == 0) {
            public_ping = false;
            snprintf(replyMsg, sizeof(replyMsg), "JT: Public Ping DISABLED");
        } else if (strcasecmp(p, "RSSI ON") == 0) {
            ping_rssi = true;
            snprintf(replyMsg, sizeof(replyMsg), "JT: Ping RSSI reporting ENABLED");
        } else if (strcasecmp(p, "RSSI OFF") == 0) {
            ping_rssi = false;
            snprintf(replyMsg, sizeof(replyMsg), "JT: Ping RSSI reporting DISABLED");
        } else if (strcasecmp(p, "SNR ON") == 0) {
            ping_snr = true;
            snprintf(replyMsg, sizeof(replyMsg), "JT: Ping SNR reporting ENABLED");
        } else if (strcasecmp(p, "SNR OFF") == 0) {
            ping_snr = false;
            snprintf(replyMsg, sizeof(replyMsg), "JT: Ping SNR reporting DISABLED");
        } else if (strcasecmp(p, "FREQERR ON") == 0) {
            ping_freqerr = true;
            snprintf(replyMsg, sizeof(replyMsg), "JT: Ping FreqErr reporting ENABLED");
        } else if (strcasecmp(p, "FREQERR OFF") == 0) {
            ping_freqerr = false;
            snprintf(replyMsg, sizeof(replyMsg), "JT: Ping FreqErr reporting DISABLED");
        } else if (strcasecmp(p, "DIRECT IGNORE ON") == 0 || strcasecmp(p, "IGNORE DIRECT ON") == 0) {
            ping_ignore_direct = true;
            snprintf(replyMsg, sizeof(replyMsg), "JT: Public Ping Direct Ignore ENABLED");
        } else if (strcasecmp(p, "DIRECT IGNORE OFF") == 0 || strcasecmp(p, "IGNORE DIRECT OFF") == 0) {
            ping_ignore_direct = false;
            snprintf(replyMsg, sizeof(replyMsg), "JT: Public Ping Direct Ignore DISABLED");
        } else if (strcasecmp(p, "FLAGS ON") == 0) {
            ping_bitmask = true;
            snprintf(replyMsg, sizeof(replyMsg), "JT: Ping Flags reporting ENABLED");
        } else if (strcasecmp(p, "FLAGS OFF") == 0) {
            ping_bitmask = false;
            snprintf(replyMsg, sizeof(replyMsg), "JT: Ping Flags reporting DISABLED");
        } else if (strcasecmp(p, "ALLOWDM ON") == 0) {
            ping_allow_dm = true;
            snprintf(replyMsg, sizeof(replyMsg), "JT: Ping DM replies ENABLED");
        } else if (strcasecmp(p, "ALLOWDM OFF") == 0) {
            ping_allow_dm = false;
            snprintf(replyMsg, sizeof(replyMsg), "JT: Ping DM replies DISABLED");
        } else if (strcasecmp(p, "OM ON") == 0) {
            ping_bitmask = true; // OM is part of flags
            snprintf(replyMsg, sizeof(replyMsg), "JT: Ping OM reporting ENABLED (via FLAGS)");
        } else if (strcasecmp(p, "OM OFF") == 0) {
            snprintf(replyMsg, sizeof(replyMsg), "JT: Use 'PPING FLAGS OFF' to disable OM/MQTT reporting");
        } else if (strncasecmp(p, "CHAN ", 5) == 0) {
            const char *cPtr = p + 5;
            bool enable = true;
            if (strncasecmp(cPtr, "ON ", 3) == 0) { enable = true; cPtr += 3; }
            else if (strncasecmp(cPtr, "OFF ", 4) == 0) { enable = false; cPtr += 4; }

            int targetCh = findChannelByNameOrIndex(cPtr);

            if (targetCh >= 0) {
                ping_channels[targetCh] = enable;
                snprintf(replyMsg, sizeof(replyMsg), "JT: Ping on Ch %d (%s) set to %d", targetCh, channels.getByIndex(targetCh).settings.name, enable);
            } else {
                snprintf(replyMsg, sizeof(replyMsg), "JT: Channel '%s' not found", cPtr);
            }
        } else if (strcasecmp(p, "IGNORE REMOTE ON") == 0 || strcasecmp(p, "REMOTE IGNORE ON") == 0) {
            ping_ignore_remote = true;
            snprintf(replyMsg, sizeof(replyMsg), "JT: Ping IGNORE REMOTE ENABLED");
        } else if (strcasecmp(p, "IGNORE REMOTE OFF") == 0 || strcasecmp(p, "REMOTE IGNORE OFF") == 0) {
            ping_ignore_remote = false;
            snprintf(replyMsg, sizeof(replyMsg), "JT: Ping IGNORE REMOTE DISABLED");
        } else if (strncasecmp(p, "SETHEX ", 7) == 0) {
            uint32_t h = strtoul(p + 7, NULL, 16);
            public_ping = (h & (1 << 0));
            ping_rssi = (h & (1 << 1));
            ping_snr = (h & (1 << 2));
            ping_freqerr = (h & (1 << 3));
            ping_ignore_direct = (h & (1 << 4));
            ping_ignore_remote = (h & (1 << 5));
            ping_bitmask = (h & (1 << 6));
            ping_allow_dm = (h & (1 << 15));
            for (uint8_t i = 0; i < 8; i++) {
                ping_channels[i] = (h & (1 << (i + 7)));
            }
            snprintf(replyMsg, sizeof(replyMsg), "JT: PPING hex updated to 0x%04X", (unsigned int)(h & 0xFFFF));
        } else if (strcasecmp(p, "GETHEX") == 0) {
            uint32_t h = 0;
            if (public_ping) h |= (1 << 0);
            if (ping_rssi) h |= (1 << 1);
            if (ping_snr) h |= (1 << 2);
            if (ping_freqerr) h |= (1 << 3);
            if (ping_ignore_direct) h |= (1 << 4);
            if (ping_ignore_remote) h |= (1 << 5);
            if (ping_bitmask) h |= (1 << 6);
            if (ping_allow_dm) h |= (1 << 15);
            for (uint8_t i = 0; i < 8; i++) {
                if (ping_channels[i]) h |= (1 << (i + 7));
            }
            snprintf(replyMsg, sizeof(replyMsg), "JT: PPING hex mask: 0x%04X", (unsigned int)(h & 0xFFFF));
        } else {
            snprintf(replyMsg, sizeof(replyMsg), "JT PPING: ON|OFF|RSSI|SNR|FREQERR|FLAGS|OM|DIRECT IGNORE|IGNORE REMOTE|ALLOWDM|SETHEX|GETHEX");
        }
    }
    else if (strncasecmp(cmd, "WATCH ", 6) == 0 || strcasecmp(cmd, "WATCH") == 0) {
        const char *sub = cmd + 6;
        while (*sub == ' ') sub++;

        if (strncasecmp(sub, "ADD ", 4) == 0) {
            uint32_t targetId = findNodeByShortNameOrHex(sub + 4);
            if (targetId == 0) {
                snprintf(replyMsg, sizeof(replyMsg), "JT: Target node not found.");
            } else {
                bool exists = false;
                int freeIdx = -1;
                for (int i = 0; i < 8; i++) {
                    if (targetWatchlist[i].nodeID == targetId) { exists = true; break; }
                    if (targetWatchlist[i].nodeID == 0 && freeIdx == -1) freeIdx = i;
                }
                if (exists) {
                    snprintf(replyMsg, sizeof(replyMsg), "JT: Node %08x already in watchlist.", targetId);
                } else if (freeIdx == -1) {
                    snprintf(replyMsg, sizeof(replyMsg), "JT: Watchlist full (8 max).");
                } else {
                    memset(&targetWatchlist[freeIdx], 0, sizeof(NexusTargetStats));
                    targetWatchlist[freeIdx].nodeID = targetId;
                    targetWatchlist[freeIdx].minInterval = 0xFFFFFFFF;

                    // Explicitly initialize Min/Max for StDev
                    targetWatchlist[freeIdx].minSnr = 100.0f;
                    targetWatchlist[freeIdx].maxSnr = -100.0f;
                    targetWatchlist[freeIdx].minFreqErr = 1000000.0f;
                    targetWatchlist[freeIdx].maxFreqErr = -1000000.0f;
                    targetWatchlist[freeIdx].maxRssi = -128;

                    watchlistCount++;
                    snprintf(replyMsg, sizeof(replyMsg), "JT: Added %08x to watchlist.", targetId);
                }
            }
        }
        else if (strncasecmp(sub, "DEL ", 4) == 0) {
            uint32_t targetId = findNodeByShortNameOrHex(sub + 4);
            // An unresolvable name returns 0, which used to match the empty
            // slots and decrement watchlistCount spuriously (A13 in the audit).
            if (targetId == 0) {
                snprintf(replyMsg, sizeof(replyMsg), "JT: Node not in list.");
            } else {
                bool found = false;
                for (int i = 0; i < 8; i++) {
                    if (targetWatchlist[i].nodeID == targetId) {
                        targetWatchlist[i].nodeID = 0;
                        watchlistCount--;
                        found = true;
                        break;
                    }
                }
                snprintf(replyMsg, sizeof(replyMsg), found ? "JT: Removed %08x." : "JT: Node not in list.", targetId);
            }
        }
        else if (strcasecmp(sub, "CLEAR") == 0) {
            for (int i = 0; i < 8; i++) targetWatchlist[i].nodeID = 0;
            watchlistCount = 0;
            snprintf(replyMsg, sizeof(replyMsg), "JT: Watchlist cleared.");
        }
        else if (strcasecmp(sub, "RESET") == 0) {
            for (int i = 0; i < 8; i++) {
                if (targetWatchlist[i].nodeID != 0) {
                    uint32_t savedId = targetWatchlist[i].nodeID;
                    memset(&targetWatchlist[i], 0, sizeof(NexusTargetStats));
                    targetWatchlist[i].nodeID = savedId;
                    targetWatchlist[i].minInterval = 0xFFFFFFFF;
                    targetWatchlist[i].minSnr = 100.0f;
                    targetWatchlist[i].maxSnr = -100.0f;
                    targetWatchlist[i].minFreqErr = 1000000.0f;
                    targetWatchlist[i].maxFreqErr = -1000000.0f;
                    targetWatchlist[i].maxRssi = -128;
                }
            }
            snprintf(replyMsg, sizeof(replyMsg), "JT: Watchlist stats reset.");
        }
        else if (strcasecmp(sub, "LIST") == 0) {
            // Static buffers (BSS, not stack). Was 128B+32B stack-local.
            static char buf[128];
            memset(buf, 0, sizeof(buf));
            buf[0] = '\0';
            strcat(buf, "Watchlist:");
            static char entry[32];
            int count = 0;
            for (int i = 0; i < 8; i++) {
                if (targetWatchlist[i].nodeID != 0) {
                    memset(entry, 0, sizeof(entry));
                    const char *shortName = nullptr;
                    auto node = nodeDB->getMeshNode(targetWatchlist[i].nodeID);
                    if (node && nexus_has_user(node) && node->short_name[0] != '\0') {
                        shortName = node->short_name;
                    }

                    if (shortName) {
                        snprintf(entry, sizeof(entry), " %08x[%s]", targetWatchlist[i].nodeID, shortName);
                    } else {
                        snprintf(entry, sizeof(entry), " %08x", targetWatchlist[i].nodeID);
                    }
                    strncat(buf, entry, sizeof(buf) - strlen(buf) - 1);
                    count++;
                }
            }
            if (count == 0) snprintf(replyMsg, sizeof(replyMsg), "JT: Watchlist empty.");
            else snprintf(replyMsg, sizeof(replyMsg), "%s", buf);
            memset(buf, 0, sizeof(buf));
            memset(entry, 0, sizeof(entry));
        }
        else if (strncasecmp(sub, "STATS ", 6) == 0 || strcasecmp(sub, "ALL") == 0) {
            bool all = strcasecmp(sub, "ALL") == 0;
            uint32_t targetId = all ? 0 : findNodeByShortNameOrHex(sub + 6);
            int idx = -1;
            if (!all) {
                for (int i = 0; i < 8; i++) {
                    if (targetWatchlist[i].nodeID == targetId) { idx = i; break; }
                }
            }

            if (!all && idx == -1) {
                snprintf(replyMsg, sizeof(replyMsg), "JT: Node not in watchlist.");
            } else {
                bool headerStarted = false;
                // Format buffer is a member of cmdBuf (typed array view of the
                // 1024 B scratch union). Reusing it across iterations of the
                // watchlist loop avoids stack-framing a 256 B buffer per call.
                for (int i = 0; i < 8; i++) {
                    if (!all && i != idx) continue;
                    if (targetWatchlist[i].nodeID == 0) continue;

                    NexusTargetStats &s = targetWatchlist[i];
                    if (s.packetCount == 0 && !all) {
                        snprintf(replyMsg, sizeof(replyMsg), "JT: No packets seen from %08x.", targetWatchlist[i].nodeID);
                        continue;
                    }
                    if (s.packetCount == 0 && all) continue;

                    if (!headerStarted) {
                        startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, all ? "Watch All" : "Watch Stats");
                        headerStarted = true;
                    }

                    memset(cmdBuf.pbuf, 0, sizeof(cmdBuf.pbuf));
                    snprintf(cmdBuf.pbuf, sizeof(cmdBuf.pbuf), "Target: %08x\nPkts: %u Bytes: %u\n", s.nodeID, s.packetCount, s.byteCount);
                    appendPaging(cmdBuf.pbuf);

                    if (s.hopDist[0] > 0) {
                        float avgRssi = s.sumRssi / s.hopDist[0];
                        float avgSnr = s.sumSnr / s.hopDist[0];
                        float avgFe = s.sumFreqErr / s.hopDist[0];

                        float varRssi = (s.sumRssi2 / s.hopDist[0]) - (avgRssi * avgRssi);
                        float varSnr = (s.sumSnr2 / s.hopDist[0]) - (avgSnr * avgSnr);
                        float varFe = (s.sumFreqErr2 / s.hopDist[0]) - (avgFe * avgFe);

                        snprintf(cmdBuf.pbuf, sizeof(cmdBuf.pbuf), "RF (Direct Only):\nAvg:%ddB SNR:%.1f FE:%.0f\nSD: %.1f SNR:%.1f FE:%.0f\nMin:%ddB SNR:%.1f FE:%.0f\nMax:%ddB SNR:%.1f FE:%.0f\n",
                                 (int)avgRssi, avgSnr, avgFe,
                                 sqrt(varRssi > 0 ? varRssi : 0), sqrt(varSnr > 0 ? varSnr : 0), sqrt(varFe > 0 ? varFe : 0),
                                 s.minRssi, s.minSnr, s.minFreqErr,
                                 s.maxRssi, s.maxSnr, s.maxFreqErr);
                    } else {
                        snprintf(cmdBuf.pbuf, sizeof(cmdBuf.pbuf), "RF: No direct packets.\n");
                    }
                    appendPaging(cmdBuf.pbuf);

                    snprintf(cmdBuf.pbuf, sizeof(cmdBuf.pbuf), "Hops: h0:%u h1:%u h2:%u h3:%u h4:%u h5:%u h6:%u h7:%u\n",
                             s.hopDist[0], s.hopDist[1], s.hopDist[2], s.hopDist[3], s.hopDist[4], s.hopDist[5], s.hopDist[6], s.hopDist[7]);
                    appendPaging(cmdBuf.pbuf);

                    snprintf(cmdBuf.pbuf, sizeof(cmdBuf.pbuf), "Relay: Last:%02x Primary:%02x (%u)\n",
                             (unsigned int)(s.lastRelay & 0xFF), (unsigned int)(s.primaryRelay & 0xFF), s.primaryRelayCount);
                    appendPaging(cmdBuf.pbuf);

                    snprintf(cmdBuf.pbuf, sizeof(cmdBuf.pbuf), "Timing: Min:%us Max:%us Age:%us\n",
                             s.minInterval == 0xFFFFFFFF ? 0 : s.minInterval, s.maxInterval,
                             (uint32_t)((Time::getMillis() / 1000) - s.lastSeen));
                    appendPaging(cmdBuf.pbuf);

                    snprintf(cmdBuf.pbuf, sizeof(cmdBuf.pbuf), "Ports: NI:%u POS:%u TEL:%u TXT:%u ADM:%u RT:%u SF:%u OTH:%u\n",
                             s.portDist[0], s.portDist[1], s.portDist[2], s.portDist[3], s.portDist[4], s.portDist[5], s.portDist[6], s.portDist[7]);
                    appendPaging(cmdBuf.pbuf);

                    if (all) appendPaging("---\n");
                }
                memset(cmdBuf.pbuf, 0, sizeof(cmdBuf.pbuf));

                if (!headerStarted) {
                     snprintf(replyMsg, sizeof(replyMsg), all ? "JT: Watchlist empty or no packets." : "JT: Node not in watchlist.");
                } else {
                    snprintf(replyMsg, sizeof(replyMsg), "JT: Sending paged stats%s...", all ? " for all" : "");
                }
            }
        }
        else {
            snprintf(replyMsg, sizeof(replyMsg), "JT WATCH: ADD|DEL|LIST|CLEAR|RESET|STATS|ALL");
        }
    }
    else if (strcasecmp(cmd, "NETSCAN") == 0) {
        if (netscan_active) {
            snprintf(replyMsg, sizeof(replyMsg), "JT: Scan already in progress (%us left)",
                     60 - (uint32_t)((Time::getMillis() - netscan_start) / 1000));
        } else {
            netscan_active = true;
            netscan_start = Time::getMillis();
            nexus_netscan_count = 0;
            netscan_reply_to = mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF;
            netscan_reply_ch = mp.channel;
            netscanReply = {isDM ? mp.from : 0xFFFFFFFF, mp.channel, replyId, toPhoneOnly, toPhoneOnly || !isDM};

            // Broadcast a 0-hop RouteDiscovery to flush neighbors
            meshtastic_MeshPacket *tp = (router) ? router->allocForSending() : nullptr;
            if (tp) {
                tp->decoded.portnum = meshtastic_PortNum_TRACEROUTE_APP;
                tp->to = 0xFFFFFFFF; // Broadcast
                tp->hop_limit = 0;
                tp->hop_start = 0;
                tp->decoded.want_response = true;
                tp->decoded.request_id = generatePacketId();
                tp->decoded.payload.size = 0; // Standard Traceroute Start
                if (service) service->sendToMesh(tp);
                snprintf(replyMsg, sizeof(replyMsg), "JT: NETSCAN started. Collecting for 60s...");
            }
        }
    }
    else if (strncasecmp(cmd, "CLIENTSCAN", 10) == 0) {
        if (clientscan_active) {
            snprintf(replyMsg, sizeof(replyMsg), "JT: ClientScan in progress (%us left)",
                     (uint32_t)((clientscan_duration_ms - (Time::getMillis() - clientscan_start)) / 1000));
        } else {
            const char *p = cmd + 10;
            while (*p == ' ') p++;

            uint32_t duration = 100; // Default 100s
            bool valid = true;
            if (*p != '\0') {
                int d = atoi(p);
                if (d < 1 || d > 600) {
                    valid = false;
                } else {
                    duration = (uint32_t)d;
                }
            }

            if (valid) {
                // Initialize scan parameters
                clientscan_active = true;
                clientscan_count = 0;
                clientscan_duration_ms = duration * 1000;

                clientscanReply = {isDM ? mp.from : 0xFFFFFFFF, mp.channel, replyId, toPhoneOnly,
                                   toPhoneOnly || !isDM};

                // 1. Generate the confirmation message
                snprintf(replyMsg, sizeof(replyMsg), "JT: CLIENTSCAN Start (%us)", (unsigned int)duration);

                // 2. Allocate the confirmation packet
                meshtastic_MeshPacket *reply = (router) ? router->allocForSending() : nullptr;
                if (reply) {
                    // 3. Set the target ID to THIS reply packet's ID
                    clientscan_target_id = generatePacketId();
                    reply->id = clientscan_target_id;
                    clientscan_start = Time::getMillis(); // Precise start

                    reply->decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;
                    reply->to = isDM ? mp.from : 0xFFFFFFFF;
                    reply->channel = mp.channel;
                    reply->decoded.reply_id = replyId;

                    // Nexus: Explicitly set hop limit/start for the trigger packet so it repeats
                    reply->hop_limit = config.lora.hop_limit ? config.lora.hop_limit : 3;
                    reply->hop_start = reply->hop_limit;

                    size_t rLen = strlen(replyMsg);
                    memcpy(reply->decoded.payload.bytes, replyMsg, rLen);
                    reply->decoded.payload.size = rLen;

                    // 4. Send to mesh - neighbors will repeat this specific ID
                    if (service) service->sendToMesh(reply);

                    LOG_INFO("Nexus: ClientScan initialized for ID !%08x (%us)", (unsigned int)clientscan_target_id, (unsigned int)duration);

                    // We don't want handleNexusCommand to send another reply
                    replyMsg[0] = '\0';
                }
            } else {
                snprintf(replyMsg, sizeof(replyMsg), "JT: Invalid ClientScan duration (1-600s)");
            }
        }
    }
#ifdef NEXUS_ENABLE_AIRTAG
    else if (strncasecmp(cmd, "AIRTAG ", 7) == 0) {
        const char *p = cmd + 7;
        while (*p == ' ') p++;
        if (strcasecmp(p, "OFF") == 0) {
            nexus_airtag_enabled = false;
            snprintf(replyMsg, sizeof(replyMsg), "JT: AirTag Beacon DISABLED");
        } else if (strcasecmp(p, "ON") == 0) {
            if (nexus_airtag_key[0] == 0) {
                snprintf(replyMsg, sizeof(replyMsg), "JT: AirTag Error - No Key set (use AIRTAG !key)");
            } else {
                nexus_airtag_enabled = true;
                snprintf(replyMsg, sizeof(replyMsg), "JT: AirTag Beacon ENABLED (Key active)");
            }
        } else if (*p == '!') {
            // Decoding the 28-byte public key (OpenHaystack keys are base64 encoded strings)
            size_t written = 0;
            if (nexus_base64_decode(nexus_airtag_key, sizeof(nexus_airtag_key), &written, p + 1) == 0) {
                nexus_airtag_enabled = true;
                snprintf(replyMsg, sizeof(replyMsg), "JT: AirTag Key SET and Beacon ENABLED");
            } else {
                snprintf(replyMsg, sizeof(replyMsg), "JT: AirTag Error - Base64 Decode Failed");
            }
        } else {
            snprintf(replyMsg, sizeof(replyMsg), "JT: AirTag usage: AIRTAG ON|OFF|!<base64>");
        }
    }
#endif
    else if (strncasecmp(cmd, "FREQSCAN ", 9) == 0) {
        if (freqscan_active || noise_sampling_active) {
            snprintf(replyMsg, sizeof(replyMsg), "JT: Radio busy.");
        } else {
            // Syntax: FREQSCAN <freq_mhz> <bw_khz>
            const char *p = cmd + 9;
            float targetFreq = atof(p);
            while (*p && *p != ' ') p++;
            while (*p == ' ') p++;
            int targetBw = atoi(p);

            bool freqOk = (targetFreq >= 100.0f && targetFreq <= 2500.0f);
            bool bwOk = (targetBw == 31 || targetBw == 62 || targetBw == 125 || targetBw == 250 || targetBw == 500);

            if (freqOk && bwOk) {
                // 1. Backup current config
                freqscan_backup = config.lora;

                // 2. Prepare Context
                freqscan_active = true;
                freqscan_start = Time::getMillis();
                freqscan_to = isDM ? mp.from : 0xFFFFFFFF;
                freqscan_ch = mp.channel;
                freqscan_reply_id = replyId;

                // 3. Apply RAM-only shift
                config.lora.override_frequency = targetFreq;
                config.lora.bandwidth = targetBw;
                config.lora.spread_factor = 7;
                config.lora.tx_enabled = false;

                if (service) service->configChanged.notifyObservers(nullptr);

                snprintf(replyMsg, sizeof(replyMsg), "JT: FreqScan START (%.2fM/%uk). Deaf for 15s.", targetFreq, targetBw);

                // Trigger noise sampling
                noise_sampling_active = true;
                noise_sampling_start = Time::getMillis() + 5000; // 5s stabilization for FreqShift
                noise_sampling_next = noise_sampling_start;
                noise_sample_count = 0;
                noise_retry_count = 0;
                noise_sampling_to = freqscan_to;
                noise_sampling_ch = freqscan_ch;
                noise_sampling_reply_id = replyId;
                noiseReply = {isDM ? mp.from : 0xFFFFFFFF, mp.channel, replyId, toPhoneOnly, toPhoneOnly || !isDM};

                setIntervalFromNow(50);
            } else {
                snprintf(replyMsg, sizeof(replyMsg), "JT: Invalid Params (Freq: 100-2500, BW: 31|62|125|250|500)");
            }
        }
    }
    else if (strcasecmp(cmd, "NOISE") == 0) {
        if (noise_sampling_active) {
            snprintf(replyMsg, sizeof(replyMsg), "JT: Noise sampling already in progress.");
        } else {
            noise_sampling_active = true;
            noise_sampling_start = Time::getMillis();
            noise_sampling_next = Time::getMillis();
            noise_sample_count = 0;
            noise_retry_count = 0;
            noise_sampling_to = isDM ? mp.from : 0xFFFFFFFF;
            noise_sampling_ch = mp.channel;
            noise_sampling_reply_id = replyId;
            noiseReply = {isDM ? mp.from : 0xFFFFFFFF, mp.channel, replyId, toPhoneOnly, toPhoneOnly || !isDM};

            snprintf(replyMsg, sizeof(replyMsg), "JT: Noise snapshot started...");
            setIntervalFromNow(10); // High-speed snapshot
        }
    }
    else if (strncasecmp(cmd, "TM", 2) == 0) {
        const char *p = cmd + 2;
        while (*p == ' ') p++;

#if HAS_TRAFFIC_MANAGEMENT
        if (*p == '\0') {
            if (!trafficManagementModule) {
                snprintf(replyMsg, sizeof(replyMsg), "JT TM: Module compiled but DISABLED in config.");
            } else {
                auto stats = trafficManagementModule->getStats();
                snprintf(replyMsg, sizeof(replyMsg), "JT TM Stats:\nDP:%u RL:%u CH:%u\nRP:%u",
                         (unsigned int)stats.position_dedup_drops,
                         (unsigned int)stats.rate_limit_drops,
                         (unsigned int)stats.nodeinfo_cache_hits,
                         (unsigned int)stats.router_hops_preserved);
            }
        } else if (strcasecmp(p, "RESET") == 0) {
            // Nexus port (Nexus-28001, 2026-08-16): TrafficManagementModule::resetStats() was
            // removed during the slim-NodeInfoLite cherry-pick (commit 18ff3a7fe) along with
            // the cuckoo cache and tag system that upstream replaced with a flat-array + tiered
            // eviction design. The /nexus TM RESET command would have zero meaningful effect
            // anyway (no equivalent upstream API to reset the per-node stats). Commented out
            // and reply updated to reflect that the RESET action is a no-op on modern firmware.
            // if (trafficManagementModule) {
            //     trafficManagementModule->resetStats();
            // }
            snprintf(replyMsg, sizeof(replyMsg), "JT TM: RESET is a no-op (upstream removed resetStats).");
        } else if (strcasecmp(p, "CONFIG") == 0) {
            // Nexus port: upstream 2.8.0 slimmed the TrafficManagementConfig struct to just
            // the five runtime knobs (position dedup interval, direct-response max hops,
            // rate-limit window, rate-limit max packets, unknown-packet threshold). All
            // "enable" / "precision" / "exhaust hop" / "preserve hops" fields were removed.
            // "Enable" is now implicit (non-zero field => enabled). Dump only what's left.
            auto &c = moduleConfig.traffic_management;
            snprintf(replyMsg, sizeof(replyMsg), "JT TM Config:\nIN:%u CR:%d RLW:%u RLM:%u\nUK:%d",
                     (unsigned int)c.position_min_interval_secs, (int)c.nodeinfo_direct_response_max_hops,
                     (unsigned int)c.rate_limit_window_secs, (unsigned int)c.rate_limit_max_packets,
                     (unsigned int)c.unknown_packet_threshold);
        } else {
            // Configuration Setters
            meshtastic_ModuleConfig mc = meshtastic_ModuleConfig_init_default;
            mc.which_payload_variant = meshtastic_ModuleConfig_traffic_management_tag;
            mc.payload_variant.traffic_management = moduleConfig.traffic_management;
            bool valid = true;

            // Nexus port: upstream 2.8.0 dropped most Nexus-specific TM fields (enabled,
// position_dedup_enabled, position_precision_bits, rate_limit_enabled, drop_unknown_enabled,
// exhaust_hop_telemetry, exhaust_hop_position, router_preserve_hops). The corresponding
// /nexus TM setters (ENABLE, DEDUP, DEDUP_PREC, CACHE_REPLY, RATELIMIT, UNKNOWN, EXHAUST_TELE,
// EXHAUST_POS, PRESERVE) are removed too. Only the fields upstream retained can still be set:
// DEDUP_INT (position_min_interval_secs), CACHE_HOPS (nodeinfo_direct_response_max_hops),
// RATELIMIT_WIN (rate_limit_window_secs), RATELIMIT_MAX (rate_limit_max_packets),
// UNKNOWN_TH (unknown_packet_threshold).
if (strncasecmp(p, "DEDUP_INT ", 10) == 0) mc.payload_variant.traffic_management.position_min_interval_secs = atoi(p + 10);
            else if (strncasecmp(p, "CACHE_HOPS ", 11) == 0) mc.payload_variant.traffic_management.nodeinfo_direct_response_max_hops = atoi(p + 11);
            else if (strncasecmp(p, "RATELIMIT_WIN ", 14) == 0) mc.payload_variant.traffic_management.rate_limit_window_secs = atoi(p + 14);
            else if (strncasecmp(p, "RATELIMIT_MAX ", 14) == 0) mc.payload_variant.traffic_management.rate_limit_max_packets = atoi(p + 14);
            else if (strncasecmp(p, "UNKNOWN_TH ", 11) == 0) mc.payload_variant.traffic_management.unknown_packet_threshold = atoi(p + 11);
            else valid = false;

            if (valid && adminModule) {
                adminModule->handleSetModuleConfig(mc);
                snprintf(replyMsg, sizeof(replyMsg), "JT: TM Config Updated.");
            } else {
                snprintf(replyMsg, sizeof(replyMsg), "JT TM: Unknown sub-command or Admin disabled.");
            }
        }
#else
        snprintf(replyMsg, sizeof(replyMsg), "JT TM: Module NOT compiled in.");
#endif
    }
    else if (strncasecmp(cmd, "SKEY ", 5) == 0) {
        const char *p = cmd + 5;
        while (*p == ' ') p++;
        int idx = atoi(p);
        while (*p && *p != ' ') p++;
        while (*p == ' ') p++;

        if (idx < 1 || idx > 4) {
            snprintf(replyMsg, sizeof(replyMsg), "JT: Invalid SKEY index (1-4)");
        } else if (strcasecmp(p, "OFF") == 0 || strcasecmp(p, "WIPE") == 0) {
            memset(nexus_secondary_keys[idx - 1], 0, 32);
            // saveVault();
            snprintf(replyMsg, sizeof(replyMsg), "JT: SKEY %d wiped.", idx);
        } else {
            // Check if input is Base64 (starts with !) or Hex
            const char *input = p;
            if (input[0] == '!') input++;

            size_t len = strlen(input);
            bool success = false;

            if (len == 64) {
                // Hex path
                for (int i = 0; i < 32; i++) {
                    char byteStr[3] = { input[i*2], input[i*2+1], '\0' };
                    nexus_secondary_keys[idx - 1][i] = (uint8_t)strtoul(byteStr, NULL, 16);
                }
                success = true;
            } else {
                // Base64 path
                size_t written = 0;
                if (nexus_base64_decode(nexus_secondary_keys[idx - 1], 32, &written, input) == 0 && written == 32) {
                    success = true;
                }
            }

            if (success) {
                // saveVault();
                snprintf(replyMsg, sizeof(replyMsg), "JT: SKEY %d updated.", idx);
            } else {
                snprintf(replyMsg, sizeof(replyMsg), "JT: SKEY Error - Invalid Hex (64) or Base64 format");
            }
        }
    }
    else if (strncasecmp(cmd, "DM-FORWARD ", 11) == 0 || strcasecmp(cmd, "DM-FORWARD") == 0 ||
             strncasecmp(cmd, "DMFORWARD ", 10) == 0 || strcasecmp(cmd, "DMFORWARD") == 0 ||
             strncasecmp(cmd, "DMF ", 4) == 0 || strcasecmp(cmd, "DMF") == 0) {
        const char *p;
        if (strncasecmp(cmd, "DMF ", 4) == 0) p = cmd + 4;
        else if (strncasecmp(cmd, "DMFORWARD ", 10) == 0) p = cmd + 10;
        else p = (strlen(cmd) > 11) ? cmd + 11 : "";
        while (*p == ' ') p++;
        if (*p == '\0') {
            snprintf(replyMsg, sizeof(replyMsg), "JT DM Forwarding: %s", dm_forward ? "ENABLED" : "DISABLED");
        } else {
            dm_forward = (strcasecmp(p, "ON") == 0);
            // saveVault();
            snprintf(replyMsg, sizeof(replyMsg), "JT: DM Forwarding set to %s", dm_forward ? "ON" : "OFF");
        }
    }
    else if (strncasecmp(cmd, "DM ", 3) == 0) {
        const char *p = cmd + 3;
        while (*p == ' ') p++;
        /*
        uint32_t targetId = parseNexusID(p);

        // Skip past ID in string to get message
        while (*p && *p != ' ' && *p != '!') p++; // Skip ID/!
        while (*p && *p != ' ') p++;             // Skip rest of ID
        while (*p == ' ') p++;                   // Skip spaces
        */
        const char *msgStart = nullptr;
        uint32_t targetId = findNodeByShortNameOrHex(p, &msgStart);
        p = msgStart;

        if (targetId == 0 || !p || *p == '\0') {
            snprintf(replyMsg, sizeof(replyMsg), "JT: Usage DM !id|name message");
        } else {
            meshtastic_MeshPacket *tp = (router) ? router->allocForSending() : nullptr;
            if (tp) {
                tp->decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;
                tp->to = targetId;
                tp->want_ack = true;
                size_t mLen = strlen(p);
                if (mLen > sizeof(tp->decoded.payload.bytes)) mLen = sizeof(tp->decoded.payload.bytes);
                memcpy(tp->decoded.payload.bytes, p, mLen);
                tp->decoded.payload.size = mLen;
                if (service) service->sendToMesh(tp);
                snprintf(replyMsg, sizeof(replyMsg), "JT: DM sent to !%08x", (unsigned int)targetId);
            }
        }
    }
    else if (strcasecmp(cmd, "HW") == 0) {
        const char *board = optstr(APP_ENV);
        const char *mcu =
#ifdef ARCH_ESP32
            "ESP32";
#elif defined(ARCH_NRF52)
            "nRF52840";
#elif defined(ARCH_PORTDUINO)
            "Linux/Portduino";
#else
            "Unknown";
#endif

        const char *radio = "Unknown";
        switch (radioType) {
            case SX1262_RADIO: radio = "SX1262"; break;
            case SX1268_RADIO: radio = "SX1268"; break;
            case RF95_RADIO: radio = "SX127x/RF95"; break;
            case SX1280_RADIO: radio = "SX128x"; break;
            case LR1110_RADIO: radio = "LR1110"; break;
            case LR1120_RADIO: radio = "LR1120"; break;
            case LR1121_RADIO: radio = "LR1121"; break;
            // Nexus-28005: LR2021 driver exists upstream but had no HW command
            // case — operators on LR2021 hardware saw 'None/Unknown' here.
            // Adding the case is a string-only change; the driver-side CR
            // detection for LR2021 lands in a sibling commit on
            // LR20x0Interface::addReceiveMetadata.
            case LR2021_RADIO: radio = "LR2021"; break;
            case LLCC68_RADIO: radio = "LLCC68"; break;
            case STM32WLx_RADIO: radio = "STM32WLx"; break;
            case SIM_RADIO: radio = "Simulator"; break;
            default: radio = "None/Unknown"; break;
        }

        const char *clock = "XTAL";
        // Nexus-28002 (2026-08-21): restored the Nexus-269 runtime TCXO detection.
        // The Nexus-28001 rebase dropped it because upstream 2.8 removed RadioInterface::isTCXO;
        // we added the `bool isTCXO` member back on RadioInterface (see RadioInterface.h:257)
        // and set it from RadioInterface::initLoRa() at every SX126x + LR11x0 init-success
        // path (RadioInterface.cpp:492, 511, 524, 543, 556, 572, 589, 606, 623). This is the
        // runtime truth, including the TCXO_OPTIONAL XTAL-fallback case (L524). Nexus-269 used
        // the same pattern; the 269 hook was bit-for-bit this.
        if (router && router->getInterface() && router->getInterface()->isTCXO) {
            clock = "TCXO";
        }

        snprintf(replyMsg, sizeof(replyMsg), "JT HW:\nBoard: %s\nMCU: %s\nRadio: %s (%s)", board, mcu, radio, clock);
    }
    else if (strncasecmp(cmd, "HOPSCALE ", 9) == 0 || strncasecmp(cmd, "HSC ", 4) == 0) {
        // Nexus-28002 (2026-08-22): HSC WANTS prints the per-portnum hop-limit
        // that HOPSCALE would set on outgoing reporting broadcasts. Today the same
        // lastRequiredHop applies to all 4 reporting portnums (Router.cpp:559-569) —
        // per-portnum differentiation is the per-port hopmask's job. The cap is
        // min(user_hop_limit, lastRequiredHop): when user_hop_limit is smaller, the
        // hopscale "if (variableHopLimit < p->hop_limit)" guard at Router.cpp:563
        // means no shrink happens and the user value wins; otherwise hopscale
        // shrinks to lastRequiredHop.
        const char *sub = cmd + (strncasecmp(cmd, "HSC", 3) == 0 ? 4 : 9);
        while (*sub == ' ') sub++;
        if (strcasecmp(sub, "WANTS") == 0) {
            if (!hopScalingModule) {
                snprintf(replyMsg, sizeof(replyMsg), "JT HSC WANTS: module not active (HAS_VARIABLE_HOPS excluded?)");
            } else {
                const uint8_t userHop = config.lora.hop_limit ? config.lora.hop_limit : 3;
                const uint8_t want = hopScalingModule->getLastRequiredHop();
                const uint8_t cap = (userHop < want) ? userHop : want;
                // Per-portnum breakdown. Today all 4 share lastRequiredHop so each
                // line reads the same; the structure future-proofs per-port HOPSCALE.
                // Compact format — every appendPaging() is a LoRa packet, keep it tight.
                startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "HSC WANTS");
                char line[160];
                snprintf(line, sizeof(line), "JT HSC WANTS: user=%u want=%u cap=%u", (unsigned)userHop, (unsigned)want, (unsigned)cap);
                appendPaging(line);
                snprintf(line, sizeof(line), "POSITION:%u", (unsigned)cap); appendPaging(line);
                snprintf(line, sizeof(line), "TELEMETRY:%u", (unsigned)cap); appendPaging(line);
                snprintf(line, sizeof(line), "NODEINFO:%u", (unsigned)cap); appendPaging(line);
                snprintf(line, sizeof(line), "NEIGHBORINFO:%u", (unsigned)cap); appendPaging(line);
            }
        } else {
            snprintf(replyMsg, sizeof(replyMsg), "JT: Unknown HSC subcommand '%s' (try WANTS)", sub);
        }
    }
    else if (strcasecmp(cmd, "HOPSCALE") == 0 || strcasecmp(cmd, "HSC") == 0) {
        // Nexus-28002 (2026-08-21): read-only status of the HopScalingModule
        // (src/modules/HopScalingModule.h). The module is instantiated and live
        // (Modules.cpp:144) but had no Nexus command — its state was only visible
        // via [HOPSCALE] LOG_INFO lines from rollHour(). This handler exposes the
        // current live state on demand.
        //
        // Paginated output (5 pages max). Each line is a separate appendPaging() call
        // so the pager chunks naturally. Read-only, no setters (yet).
        if (!hopScalingModule) {
            snprintf(replyMsg, sizeof(replyMsg), "JT HOPSCALE: module not active (HAS_VARIABLE_HOPS excluded?)");
        } else {
            const auto &perHop = hopScalingModule->getLastPerHopCounts();
            const auto &trend = hopScalingModule->getLastTrendStats();
            const float politeness = hopScalingModule->getPoliteness();
            const char *polName = "DEFAULT";
            if (politeness <= 0.26f)      polName = "STRICT";
            else if (politeness >= 0.99f) polName = "GENEROUS";

            startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF, mp.channel, replyId, "HOPSCALE");

            char line[160];
            // 1. Header + status
            snprintf(line, sizeof(line),
                     "JT HOPSCALE: status=%s  entries=%u/128 (%u%%)  samp=1/%u  filt=1/%u",
                     hopScalingModule->getEntryCount() > 0 ? "ACTIVE" : "EMPTY",
                     (unsigned)hopScalingModule->getEntryCount(),
                     (unsigned)hopScalingModule->getFillPercentage(),
                     (unsigned)hopScalingModule->getSamplingDenominator(),
                     (unsigned)hopScalingModule->getFilteringDenominator());
            appendPaging(line);

            // 2. Suggested hop + politeness
            snprintf(line, sizeof(line),
                     "lastSuggestedHop: %u/7  lastRequiredHop: %u  politeness: %.2f (%s)",
                     (unsigned)hopScalingModule->getLastSuggestedHop(),
                     (unsigned)hopScalingModule->getLastRequiredHop(),
                     politeness, polName);
            appendPaging(line);

            // 3. Per-hop bucket (scaled by filteringDenominator)
            snprintf(line, sizeof(line),
                     "perHop [h0..h7]: [%u %u %u %u %u %u %u %u]  total=%u",
                     (unsigned)perHop.perHop[0], (unsigned)perHop.perHop[1],
                     (unsigned)perHop.perHop[2], (unsigned)perHop.perHop[3],
                     (unsigned)perHop.perHop[4], (unsigned)perHop.perHop[5],
                     (unsigned)perHop.perHop[6], (unsigned)perHop.perHop[7],
                     (unsigned)perHop.total);
            appendPaging(line);

            // 4. Trend stats
            snprintf(line, sizeof(line),
                     "trend: new=%u returning=%u lapsed=%u olderThan4h=%u agingOut=%u",
                     (unsigned)trend.newThisHour, (unsigned)trend.returningThisHour,
                     (unsigned)trend.lapsedSinceLastHour, (unsigned)trend.olderThan4h,
                     (unsigned)trend.agingOut);
            appendPaging(line);

            // 5. 13-hour scaled per-hour bitmap (h0 = just-completed hour)
            snprintf(line, sizeof(line),
                     "perHour [h0..h12] (scaled): [%u %u %u %u %u %u %u %u %u %u %u %u %u]",
                     (unsigned)trend.scaledPerHour[0],  (unsigned)trend.scaledPerHour[1],
                     (unsigned)trend.scaledPerHour[2],  (unsigned)trend.scaledPerHour[3],
                     (unsigned)trend.scaledPerHour[4],  (unsigned)trend.scaledPerHour[5],
                     (unsigned)trend.scaledPerHour[6],  (unsigned)trend.scaledPerHour[7],
                     (unsigned)trend.scaledPerHour[8],  (unsigned)trend.scaledPerHour[9],
                     (unsigned)trend.scaledPerHour[10], (unsigned)trend.scaledPerHour[11],
                     (unsigned)trend.scaledPerHour[12]);
            appendPaging(line);

            // 6. Per-portnum would-set (effective cap = min(user_hop_limit, lastRequiredHop)).
            //    Same lastRequiredHop applies to all 4 reporting portnums today
            //    (Router.cpp:559-569) — per-portnum differentiation is per-port hopmask's job.
            //    user + want are already in section 2 of this same output, so just show caps.
            {
                const uint8_t userHop = config.lora.hop_limit ? config.lora.hop_limit : 3;
                const uint8_t want = hopScalingModule->getLastRequiredHop();
                const uint8_t cap = (userHop < want) ? userHop : want;
                snprintf(line, sizeof(line), "hopscale wants: PO:%u TL:%u NI:%u NBI:%u",
                         (unsigned)cap, (unsigned)cap, (unsigned)cap, (unsigned)cap);
                appendPaging(line);
            }
        }
    }
    else if (strcasecmp(cmd, "NODES28") == 0 || strcasecmp(cmd, "N28") == 0) {
        // Nexus-28002 (2026-08-21): /nexus NODES28 lists nodeDB entries that
        // match at least one of two criteria (P or V). Both flags are shown per row.
        //
        // Dispatch ordering: the regular NODES handler at line ~2163 used to be
        // `strncasecmp(cmd, "NODES", 5)` (5-char prefix), which shadowed NODES28
        // and treated "28" as a search string — a node whose name contains "28"
        // was the only result ever returned. The NODES prefix check has been
        // tightened to (cmd[5] == '\0' || cmd[5] == ' ') so this exact-match
        // branch wins. (Fix: Nexus-28004, 2026-09-01.)
        //
        // P (Probable 2.8) — structural: 32-byte public key present AND
        //     num == crc32(key). 2.8+ assigns node_num from the hash of the
        //     node's public key (NodeDB.cpp:491 / Router.cpp:743). 2.7 and
        //     earlier used random/static node_nums with separately-stored keys.
        //
        // V (Verified) — cryptographic: NODEINFO_BITFIELD_HAS_XEDDSA_SIGNED_MASK
        //     set. Router.cpp:754,785 sets this when a valid XEdDSA signature
        //     is observed from this node — cryptographic evidence the node
        //     owns its claimed public key. Distinct from
        //     NODEINFO_BITFIELD_IS_KEY_MANUALLY_VERIFIED_MASK (operator's
        //     manual "trust this key" mark; not in scope here).
        //
        // Filter is "either P or V" so we don't dump every node in the DB.
        // Self is included (operator's own node will usually be P; useful
        // confirmation that the operator's config is healthy).
        size_t totalNodes = nodeDB->getNumMeshNodes();
        if (totalNodes == 0) {
            snprintf(replyMsg, sizeof(replyMsg), "JT NODES28: empty nodeDB");
        } else {
            startPaging(mp.to == myNodeInfo.my_node_num ? mp.from : 0xFFFFFFFF,
                        mp.channel, replyId, "NODES28");

            char line[128];
            size_t matched = 0;

            for (size_t i = 0; i < totalNodes; i++) {
                meshtastic_NodeInfoLite *n = nodeDB->getMeshNodeByIndex(i);
                if (!n) continue;

                // Probable-2.8: 32-byte key AND num == crc32(key).
                // Public key size guard: smaller-than-32 keys (legacy formats)
                // can't be CRC32-validated, so we exclude them from P but
                // still let V match.
                bool isP = false;
                if (n->public_key.size == 32) {
                    isP = (crc32Buffer(n->public_key.bytes, 32) == n->num);
                }

                // Verified: HAS_XEDDSA_SIGNED bit set by Router on signature observation.
                bool isV = (n->bitfield & NODEINFO_BITFIELD_HAS_XEDDSA_SIGNED_MASK) != 0;

                if (!isP && !isV) continue; // filter: only P or V matches
                matched++;

                // short_name is char[5] (4 chars + NUL). Show "--" when empty
                // so columns line up.
                const char *shortName = (n->short_name[0] == '\0') ? "--" : n->short_name;

                // Self marker (operator's own node). Purely cosmetic.
                const char *suffix = (n->num == myNodeInfo.my_node_num) ? " (self)" : "";

                snprintf(line, sizeof(line), "!%08x  %-5s %s%s%s",
                         (unsigned)n->num, shortName,
                         isP ? "P" : "-",
                         isV ? "V" : "-",
                         suffix);
                appendPaging(line);
            }

            if (matched == 0) {
                appendPaging("(no nodes matched P or V)");
            }

            // Send the count summary via the primary replyMsg (paging handles
            // the body). Mirrors the /nexus FAVS, IGNORED, ADMINS pattern.
            snprintf(replyMsg, sizeof(replyMsg),
                     "JT NODES28: %u/%u matched (P=structural2.8, V=XEdDSA-signed)",
                     (unsigned)matched, (unsigned)totalNodes);
        }
    }
    else if (strcasecmp(cmd, "ACC") == 0) {
#if !MESHTASTIC_EXCLUDE_ACCELEROMETER
        // Nexus port: upstream 2.8.0 removed AccelerometerThread::getMotionData() and
        // MotionSensor::getMotionData(). The replacement is the static
        // MotionSensor::getLatestCompassAccelSample() which reads the most recent
        // sample the accelerometer thread has cached. ageMs tells us how stale it is;
        // we use 10s as the "fresh enough" threshold for /nexus ACC.
        uint32_t ageMs = 0;
        float x = 0, y = 0, z = 0;
        if (MotionSensor::getLatestCompassAccelSample(x, y, z, ageMs) && ageMs < 10000) {
            snprintf(replyMsg, sizeof(replyMsg), "JT Motion (cached %ums):\nX: %.2fg\nY: %.2fg\nZ: %.2fg", (unsigned)ageMs, x/9.80665f, y/9.80665f, z/9.80665f);
        } else {
            snprintf(replyMsg, sizeof(replyMsg), "JT Motion: No recent sample (ageMs=%u, sensor present=%d)", (unsigned)ageMs, accelerometerThread ? 1 : 0);
        }
#else
        snprintf(replyMsg, sizeof(replyMsg), "JT Motion: Feature excluded.");
#endif
    }
    // Nexus-28002 (2026-08-20): /nexus CHECKSIG — inspect the
    // signature state of the command packet itself. Compact 4-line
    // output with inline emoji (same convention as the auto-prefix
    // below: ✅ verified, 🚫 not verified, 📄 no signature field):
    //   <emoji> JT CHECKSIG:
    //     <sig bytes>
    //     Upstream: <emoji>
    //     Trusted:  <emoji> [M<n>|S<n>|A<n>]
    // Self-contained — operates on the command packet itself, no args.
    // The auto-prefix block at line ~7830 is gated to skip this
    // command (we emit our own leading emoji); see the strncmp gate.
    else if (strcasecmp(cmd, "CHECKSIG") == 0) {
        const bool hasSig = (mp.decoded.xeddsa_signature.size == XEDDSA_SIGNATURE_SIZE);
        const bool hasAnySig = (mp.decoded.xeddsa_signature.size > 0);

        // Emoji convention (shared with the auto-prefix block below):
        //   ✅ verified / matched
        //   🚫 unverified / not matched
        //   📄 no signature field
        const char *upstreamEmoji;
        const char *trustedEmoji;
        if (!hasAnySig) {
            // No signature at all — upstream had nothing to verify,
            // verifyTrustedSignature returns false with "no_signature".
            upstreamEmoji = "📄";
            trustedEmoji = "📄";
        } else if (hasSig) {
            // Full 64-byte signature present. Upstream check is mp.xeddsa_signed
            // (set by Router::checkXeddsaReceivePolicy). Trusted check goes
            // through verifyTrustedSignature's admin_key / secondary_keys /
            // master_key list.
            upstreamEmoji = mp.xeddsa_signed ? "✅" : "🚫";
            trustedEmoji = "🚫";   // overwritten to "✅ <key>" below on hit
        } else {
            // Truncated / malformed (1..63 bytes) — upstream skips the
            // verify, verifyTrustedSignature also skips. Treat as unsigned.
            upstreamEmoji = "📄";
            trustedEmoji = "📄";
        }

        char matchLabel[24];
        bool trustedMatch = false;
        if (nexusModule)
            trustedMatch = nexusModule->verifyTrustedSignature(&mp, matchLabel, sizeof(matchLabel));
        if (matchLabel[0] == '\0')
            strcpy(matchLabel, trustedMatch ? "?" : "none");

        // Prefix emoji selected to match the upstream state. Convention
        // matches the dispatch chain's auto-prefix so an operator
        // scanning the chat log sees the same first-glyph signal
        // regardless of which command produced the reply.
        //
        // BUGFIX (Nexus-28002, 2026-08-20): do NOT compare
        // upstreamEmoji[0] against '✅' or '🚫' as a single char.
        // UTF-8 multi-byte emojis compile to multi-char literals in
        // C/C++ which evaluate to a 32-bit int (e.g. '✅' = 0x00e29c85)
        // — comparing against a single 8-bit char (sign-extended to
        // 32-bit, e.g. 0xFFFFFFE2) is ALWAYS false on platforms where
        // char is signed (x86, ARM). The previous logic therefore
        // ALWAYS fell through to the 📄 branch regardless of the actual
        // upstreamEmoji value, producing output like:
        //   📄 JT CHECKSIG:
        //     64 B
        //     Upstream: ✅    <- upstream is correctly ✅
        //     Trusted: ✅ M3
        // with a stale 📄 prefix that disagreed with the upstream line.
        // Fix: compare by full string equality (strcmp on UTF-8 byte
        // sequences). The strings are string-literal constants so the
        // lengths are known at compile time; strlen would work too.
        const char *prefixEmoji;
        if (strcmp(upstreamEmoji, "\xE2\x9C\x85") == 0) // ✅
            prefixEmoji = "\xE2\x9C\x85";                // ✅
        else if (strcmp(upstreamEmoji, "\xE2\x9D\x8C") == 0) // 🚫
            prefixEmoji = "\xE2\x9D\x8C";                // 🚫
        else
            prefixEmoji = "\xF0\x9F\x93\x84";             // 📄

        // Compose "Trusted:" line. The matched-key shorthand from
        // verifyTrustedSignature is one of "A<n>", "S<n>", "M<n>".
        // When a match is found the verified state is unambiguous
        // ("✅ <key>") regardless of upstream state — upstream may have
        // rejected the packet, but our trust-list crypto verification
        // succeeded independently. If no match, just the emoji.
        char trustedLine[40];
        if (trustedMatch && matchLabel[0] != '\0' && matchLabel[0] != '?')
            snprintf(trustedLine, sizeof(trustedLine), "✅ %s", matchLabel);
        else
            snprintf(trustedLine, sizeof(trustedLine), "%s", trustedEmoji);

        snprintf(replyMsg, sizeof(replyMsg),
                 "%s JT CHECKSIG:\n"
                 "  %u B\n"
                 "  Upstream: %s\n"
                 "  Trusted:  %s",
                 prefixEmoji, (unsigned)mp.decoded.xeddsa_signature.size,
                 upstreamEmoji, trustedLine);
    }
    else if (strcasecmp(cmd, "SETTIME") == 0) {
        snprintf(replyMsg, sizeof(replyMsg), "JT: Usage SETTIME <unix_ts>");
    }
    // Nexus-28005 (2026-09-15): /nexus G3 (alias: FRONTEND) — station-g3
    // PA / LNA GPIO control. Handler lives in NexusModule::nexusG3HandleCommand
    // (NexusModule.cpp) — no new TU. The
    // entire handler is gated on #ifdef STATION_G3 so the call resolves to a
    // no-op on non-station-g3 builds; the dispatcher here only routes when the
    // build *is* station-g3. Falls through to the default 'Unknown command'
    // reply on every other board, matching the LR2021_SF gate pattern above.
#ifdef STATION_G3
    else if (strcasecmp(cmd, "G3") == 0 || strcasecmp(cmd, "FRONTEND") == 0 ||
             strncasecmp(cmd, "G3 ", 3) == 0 || strncasecmp(cmd, "FRONTEND ", 9) == 0) {
        nexusModule->nexusG3HandleCommand(mp, cmd, replyMsg, sizeof(replyMsg));
    }
#endif
    else {
        if (!cmd || cmd[0] == '\0') {
            snprintf(replyMsg, sizeof(replyMsg), "JT: Ready. Send HELP for command list.");
        } else {
            snprintf(replyMsg, sizeof(replyMsg), "JT: Unknown command '%s'", cmd);
        }
    }

    if (pagingRendering || pagingGeneration != pagingGenerationBefore) {
        pagingWindow.finish(paging_buf);
        paging_ptr = pagingWindow.size();
    }
    if (pagingRendering)
        return;

    // 2026-08-17 (Nexus-28001): prepend a signature-state emoji to every reply as a debug
    // indicator. 2026-08-23 (Nexus-28002): expanded from 3 to 4 states — the 'signed & verified'
    // case splits on whether the signer is in our trust list (admin_key[0..2],
    // nexus_secondary_keys[0..3], or NEXUS_MASTER_KEY_<n> build-time keys). Palette now
    // intentionally overlaps with the PPING sig flag so operators see one consistent
    // 'signature state' picture across both surfaces:
    //   🔒 signed, verified, AND trusted — xeddsa_signed && verifyTrustedSignature() hit
    //   🟢 signed, verified, NOT in trust list — xeddsa_signed && verifyTrustedSignature() miss
    //                                   (signature is cryptographically valid but from an
    //                                    unknown signer; same color as PPING's verified state)
    //   🟠 signed but verify failed      — !xeddsa_signed && signature.size == 64
    //   🔴 unsigned                      — signature.size == 0
    //
    // Performance: verifyTrustedSignature is gated behind a `signature.size == 64` check so
    // it only runs on signed packets. On a signed packet, it does up to 11 XEdDSA verifications
    // (3 admin + 4 secondary + 4 master) and exits early on the first hit. Unsigned packets
    // (the common case) skip the helper entirely. CHECKSIG reply remains gated out below
    // to avoid double-prefixing with its own inline emoji.
    //
    // Skipped when replyMsg is empty (suppressed-reply case at line 7239 / unknown errors).
    // replyMsg is 200 bytes (NEXUS_MAX_REPLY); the longest emoji is 6 UTF-8 bytes (⚠️+space);
    // the existing NEXUS_MAX_REPLY_BODY clamp on rLen will truncate gracefully if needed.
    //
    // Nexus-28002: skip auto-prefix for CHECKSIG — it emits its own
    // leading emoji inline with the upstream/trusted state. Adding
    // a 🔒/🟢/🟠/🔴 on top would produce redundant double-emoji output
    // (e.g. "🔒 ✅ JT CHECKSIG:"). Detection: replyMsg starts with
    // one of three CHECKSIG-prefix emojis (UTF-8 byte counts: ✅=3,
    // 🚫=4, 📄=4) + " " (1 byte) + "JT CHECKSIG" (11 bytes). We
    // compare against 15 bytes (covers "✅ JT CHECKSIG" exactly,
    // and the first 15 bytes of "🚫/📄 JT CHECKSIG" which still
    // uniquely identifies the command).
    const bool isChecksigReply =
        strncmp(replyMsg, "✅ JT CHECKSIG", 15) == 0 ||
        strncmp(replyMsg, "🚫 JT CHECKSIG", 15) == 0 ||
        strncmp(replyMsg, "📄 JT CHECKSIG", 15) == 0;
    if (replyMsg[0] != '\0' && !isChecksigReply) {
        const char *sigEmoji;
        if (mp.decoded.xeddsa_signature.size == XEDDSA_SIGNATURE_SIZE) {
            if (mp.xeddsa_signed) {
                // Signature is valid. Now ask verifyTrustedSignature whether the
                // signer is in our admin/secondary/master key list. Gated by the
                // signature-size check above so unsigned packets never call the
                // helper (saves 11 XEdDSA verifications on the common case).
                char matchLabel[24] = {0};
                bool trustedMatch = false;
                if (nexusModule) {
                    trustedMatch = nexusModule->verifyTrustedSignature(&mp, matchLabel, sizeof(matchLabel));
                }
                sigEmoji = trustedMatch ? "🔒 " : "🟢 ";
            } else {
                sigEmoji = "🟠 "; // signature present but xeddsa_verify() failed
            }
        } else {
            sigEmoji = "🔴 "; // unsigned
        }
        // Shift replyMsg right by strlen(sigEmoji) bytes. memmove handles overlap.
        size_t curLen = strlen(replyMsg);
        size_t preLen = strlen(sigEmoji);
        if (curLen + preLen < NEXUS_MAX_REPLY) {
            memmove(replyMsg + preLen, replyMsg, curLen + 1); // +1 for the null terminator
            memcpy(replyMsg, sigEmoji, preLen);
        }
        // else: skip the prepend; reply is already at the buffer cap.
    }

    // Send the reply back to the sender via TEXT_MESSAGE_APP
    if (strlen(replyMsg) > 0) {
        // Nexus: Multi-Routing Logic.
        // Send a local copy if explicitly requested or if it's a non-DM command from a non-shadow node.
        bool shouldSendLocal = (toPhoneOnly || !isDM);

        if (shouldSendLocal) {
            size_t rLen = strlen(replyMsg);
            if (rLen > NEXUS_MAX_REPLY_BODY) rLen = NEXUS_MAX_REPLY_BODY;
            uint32_t to = isDM ? (mp.from ? mp.from : myNodeInfo.my_node_num) : 0xFFFFFFFF;
            sendToPhone(to, mp.channel, replyId, replyMsg, rLen);
            LOG_INFO("Nexus: Local reply copy sent.");
        }

        if (!toPhoneOnly) {
            // Nexus: Standard Mesh Reply (Non-blocking deferred reply).
            // queueMeshReply handles the jitter (broadcast: 100-5000 ms,
            // DM: 50 ms yield) and the PendingReply setup internally.
            size_t rLen = strlen(replyMsg);
            if (rLen > NEXUS_MAX_REPLY_BODY) rLen = NEXUS_MAX_REPLY_BODY;
            queueMeshReply(isDM ? mp.from : 0xFFFFFFFF, mp.channel, replyId, replyMsg, rLen, false, true);
        }
    }

    // Nexus: Move configuration application to runOnce() via timer
    if (pendingLoRaConfig) {
        // The mutation that staged pendingConfig owns its deadline.
        LOG_INFO("Nexus: LoRa shift scheduled in 5000ms");
        setIntervalFromNow(50);
    }
}
