// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) 2026 Jennifer Sanchez
//
// Licensed under the GNU Affero General Public License v3.
// See LICENSE-AGPL for the full license text.
//
// NexusModule — remote CLI module for Meshtastic firmware
//

#include "modules/NexusModule.h"
#include "modules/NexusTxHook.h"

/*
 * Nexus module implementation.
 * Internal development/testing helpers live behind NEXUS_PRIVATE_BUILD in the
 * private companion units; public builds compile only the hooks declared in
 * NexusModule.h (their private members become inline no-ops there).
 */
#ifdef NEXUS_PRIVATE_BUILD
#include "modules/NexusModule_private.h"
#endif
#include "Channels.h"
#include "MeshService.h"
#if NEXUS_ENABLE_RNODE
#include "mesh/RNodeEngine.h"
#endif
#if NEXUS_ENABLE_JENCORE_SNIFFER
#include "mesh/JenCoreRepeater.h"
#endif
#include "NodeDB.h"
#include "Power.h"
#include "PowerFSM.h"
#include "RTC.h"
#include "SPILock.h"
#include "sleep.h"
#include "input/InputBroker.h"
#include "meshUtils.h"
#include <FSCommon.h>
#include <ctype.h>
// Nexus-269: Sensor weight-reduction umbrella. Force-define
// MESHTASTIC_EXCLUDE_* macros from a single NEXUS_SENSOR_WEIGHTREDUCTION
// tier (1=LIGHT, 2=MEDIUM, 3=HEAVY) in the variant's build_flags. See
// src/NexusSensorBudget.h for the full mapping. Include this BEFORE the
// sensor module headers so the underlying #if guards pick up the umbrella
// flag. Also gates the Shadow CLI /nexus TELEMETRY / POWER / ENV helpers
// below to no-op when the corresponding module is not registered.
#include "NexusSensorBudget.h"
#if defined(ARCH_ESP32) && !MESHTASTIC_EXCLUDE_WIFI
#include "MeshtasticOTA.h"
#endif
#include "CryptoEngine.h" // for crypto->xeddsa_verify (Nexus-28002, verifyTrustedSignature)
#include "Router.h"
#include "UptimeClock.h" // Time::getMillisMonotonic() — wrap-safe 64-bit uptime
#include "airtime.h"
#include "configuration.h"
#include "main.h"
#include "mesh/Throttle.h"
#include <new>
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

#if defined(ARCH_ESP32)
RTC_DATA_ATTR static bool rtc_nexus_hibernating = false;
#else
static bool rtc_nexus_hibernating = false;
#endif

#if !MESHTASTIC_EXCLUDE_GPS
#include "GPS.h"
#endif

#if MESHTASTIC_EXCLUDE_GPS
#include "modules/PositionModule.h"
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

meshtastic::NexusModule *meshtastic::nexusModule;

static inline bool nexus_is_favorite(const meshtastic_NodeInfoLite *n) { return nodeInfoLiteIsFavorite(n); }
static inline bool nexus_is_ignored(const meshtastic_NodeInfoLite *n) { return nodeInfoLiteIsIgnored(n); }
static inline bool nexus_via_mqtt(const meshtastic_NodeInfoLite *n) { return nodeInfoLiteViaMqtt(n); }
static inline bool nexus_has_user(const meshtastic_NodeInfoLite *n) { return nodeInfoLiteHasUser(n); }

// Nexus-28005: A packet is internal only if isFromUs()
// returns true AND it did NOT arrive over LoRa. The protobuf field
// transport_mechanism IS tracked on the wire — RadioInterface.cpp:1559
// sets TRANSPORT_LORA, UdpMulticastHandler.h:87 sets TRANSPORT_MULTICAST_UDP,
// PacketAPI.cpp:73 sets TRANSPORT_API. Without this guard, a packet that
// loops back over LoRa with from==0 (anonymous) or from==us (a self-
// rebroadcast that came back over the air) would incorrectly satisfy
// isFromUs() and skip the REQ_SIG / replay gates. Local serial / API /
// UDP ingress always sets a non-LoRa mechanism, so legitimate internal
// packets still pass.
static inline bool nexusIsInternal(const meshtastic_MeshPacket &mp) {
    if (!isFromUs(&mp)) return false;
    return mp.transport_mechanism != meshtastic_MeshPacket_TransportMechanism_TRANSPORT_LORA;
}
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
static inline bool nexus_is_licensed(const meshtastic_NodeInfoLite *n) { return nodeInfoLiteIsLicensed(n); }
static inline meshtastic_User nexus_user_of(const meshtastic_NodeInfoLite *n) {
    return TypeConversions::ConvertToUser(n);
}
// === END SLIM NodeInfoLite COMPAT SHIM ===

#ifndef BITFIELD_OK_TO_MQTT_MASK
#define BITFIELD_OK_TO_MQTT_MASK (1 << 0)
#endif

#ifdef NEXUS_ENABLE_AIRTAG
bool nexus_airtag_enabled = false;
uint8_t nexus_airtag_key[28] = {0}; // Apple public key (Base64 decoded)
#endif

// v64: per-subsystem freeze flags. File-scope so NodeDB.cpp / WarmNodeStore.cpp
// can gate their save paths without re-introducing the NexusModule dependency
// in low-level TUs. Persisted via /nexus SAVE (SCOPE RULE). DBFLUSH bypasses.
// Bit layout (vault->freeze_flags): bit0=NodeDB, bit1=Warm, bit2=Device, bit3=Config.
namespace meshtastic {
bool freeze_nodedb = false;
bool freeze_warm = false;
bool freeze_device = false;
bool freeze_config = false;
} // namespace meshtastic

// Nexus: External helper for Shadow CLI
bool meshtastic::getNexusPowerTelemetry(meshtastic_Telemetry *m) {
#if HAS_TELEMETRY && !MESHTASTIC_EXCLUDE_POWER_TELEMETRY
    if (PowerTelemetryModule::instance) {
        return PowerTelemetryModule::instance->getPowerTelemetry(m);
    }
#endif
    return false;
}

// Nexus: External helper for Shadow CLI
bool meshtastic::getNexusEnvironmentTelemetry(meshtastic_Telemetry *m) {
#if HAS_TELEMETRY && !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR
    if (environmentTelemetryModule) {
        return environmentTelemetryModule->getEnvironmentTelemetry(m);
    }
#endif
    return false;
}

// Nexus: External helper for Shadow CLI — reads device telemetry (battery / voltage /
// channel util / air util / uptime) directly from the live DeviceTelemetryModule.
// Bypasses nodeDB->nodeTelemetry, which is gated by MESHTASTIC_EXCLUDE_TELEMETRYDB and
// is empty on memory-constrained variants (promicro, rak3401, heltec_v3). On those
// variants, /nexus TELEMETRY previously lost its "DB:" line because the shim helper
// returned nullptr unconditionally. This helper reads the live snapshot instead.
bool meshtastic::getNexusDeviceTelemetry(meshtastic_Telemetry *m) {
#if HAS_TELEMETRY
    if (DeviceTelemetryModule::instance) {
        return DeviceTelemetryModule::instance->getDeviceTelemetry(m);
    }
#endif
    return false;
}

extern void getMacAddr(uint8_t *dmac);

NexusModule::NexusModule()
    : MeshModule("Nexus"),
      concurrency::OSThread("Nexus"),
      ScanI2CConsumer() {
    setInterval(1000);
    inputEventObserver = new CallbackObserver<NexusModule, const InputEvent *>(this, &NexusModule::onInputEvent);

    // Initialize relay stats
    memset(relay_stats, 0, sizeof(relay_stats));
    memset(nexus_hop_matrix, 0, sizeof(nexus_hop_matrix));
    memset(nexus_port_stats, 0, sizeof(nexus_port_stats));
    memset(nexus_port_bytes, 0, sizeof(nexus_port_bytes));
    memset(nexus_transport_stats, 0, sizeof(nexus_transport_stats));
    mqtt_ok_on = 0;
    mqtt_ok_off_flag_on = 0;
    mqtt_both_off = 0;

    // Firewall Statistics
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
    fw_drop_br_telemetry = 0;
    fw_drop_br_position = 0;
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

    // Invalid-position drop counters
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

    // Watchlist SIGINT (RAM only)
    watchlistCount = 0;
    for(int i=0; i<8; i++) targetWatchlist[i].nodeID = 0;

    // Nexus: Capture boot parameters and restore vault
    loadVault();

    // The module framework never calls MeshModule::setup(), so run the init-time side
    // effects (TX hook registration, hibernation seed, G3 boot detect) from here.
    initOnce();
}

void NexusModule::initOnce() {
    is_hibernating = rtc_nexus_hibernating;
    if (!nexusTxHook) {
        nexusTxHook = new NexusTxHook();
    }

#ifdef STATION_G3
    // Nexus-28005 (2026-09-15): /nexus G3 boot-detect + vault apply.
    //
    // Runs from the constructor after loadVault(), so nexusVariantBits is already
    // populated from disk when we get here. The boot-detect sequence is:
    //
    //   1. Put each pin into INPUT mode with PULLS DISABLED before reading.
    //      ESP32-S3 cold-boot GPIO default has the internal pull-up enabled
    //      on most pads (including GPIO9/10) — a naive digitalRead() on a
    //      floating pin would return HIGH and we'd mis-classify it as a
    //      jumper. The Arduino-ESP32 2.x pinMode(INPUT) implementation
    //      explicitly sets .pull_up_en = GPIO_PULLUP_DISABLE and
    //      .pull_down_en = GPIO_PULLDOWN_DISABLE in gpio_config_t before
    //      configuring the pad, so calling pinMode(INPUT) first is the
    //      correct way to get a clean external-only reading.
    //      Fix verified by bug report 2026-09-15 18:29 UTC: with the
    //      original "read-before-pinMode" order, both jumpers uninstalled
    //      returned DISABLED on both pins (pull-up false-positive on the
    //      floating pads).
    //   2. HIGH = jumper installed = DISABLED. Firmware does not drive this
    //      pin afterwards — it stays in INPUT mode and the operator's
    //      hardware wiring wins. Vault bits for this pin are ignored
    //      (DISABLED always wins — the jumper is the source of truth).
    //   3. LOW = jumper absent = OFF. Apply the appropriate OFF drive so
    //      the pin is not floating: PA → INPUT (true Hi-Z, no pull),
    //      LNA → OUTPUT HIGH (explicit drive avoids a floating LNA-P net
    //      coupling noise into the LNA). Then if vault says ON for this
    //      pin, override OFF → ON by driving the active level. This is how
    //      /nexus PA ON (followed by /nexus SAVE) survives a reboot without
    //      re-issuing the command.
    //
    // Runtime state (nexus_g3_pa_runtime / nexus_g3_lna_runtime) is the
    // source of truth after initOnce() returns. nexusVariantBits is only what
    // gets persisted on the next SAVE.
    //
    // Hardware constants (B&Q BQ35LORA900V1M PA module, station-g3 variant):
    //   PIN_PA  = GPIO9   — wired in parallel with PL1 PA jumper pad
    //   PIN_LNA = GPIO10  — wired in parallel with LNA-P jumper pad (active-LOW)
    // GPIO42 (secondary LNA slot) intentionally not handled — station-g3 has
    // no LNA2 hardware.
    {
        const uint8_t PIN_PA  = 9;
        const uint8_t PIN_LNA = 10;

        // --- PA (GPIO9, active-HIGH) ---
        // Step 1: explicitly set INPUT mode (disables pull-up + pull-down).
        // Step 2: small settle delay (1 ms is generous; the IO_MUX + GPIO
        // matrix takes a few cycles to update after a mode change, and any
        // external capacitive load on the pad will need a moment to bleed).
        // Nexus-28006 (2026-09-17) — boot-detect latch-clear fix.
        // digitalWrite(LOW) BEFORE pinMode(INPUT) forces the per-pin
        // output latch LOW while the pin is still in OUTPUT mode. This
        // clears any stale HIGH leftover from a prior boot stage (SPI
        // flash access, prior /nexus G3 PA ON, deep-sleep state) that
        // would otherwise contaminate the subsequent digitalRead() with a
        // false-positive HIGH reading → mis-classify no-jumper pads as
        // DISABLED. Cost: microseconds (no delay added) → watchdog-safe
        // at boot. Verified by bug report 2026-09-17 06:07 UTC:
        // before this fix, boot-detect read both PA and LNA as DISABLED
        // when no jumpers were installed; after this fix, REINIT
        // correctly re-classifies both as OFF (because by the time
        // REINIT runs, the pin has already been through pinMode(INPUT)
        // and the latch is in a clean state). REINIT also gets the
        // same latch-clear prefix for defensive consistency.
        digitalWrite(PIN_PA, LOW);
        pinMode(PIN_PA, INPUT);
        delay(1);
        int raw_pa = digitalRead(PIN_PA);
        if (raw_pa == HIGH) {
            // Jumper installed (pulled HIGH through PL1 / PL2 net) → DISABLED.
            nexus_g3_pa_runtime = G3_DISABLED;
            LOG_INFO("Nexus/G3: PA (GPIO9) DISABLED — physical jumper detected at boot (PL1). Firmware will not drive this pin.");
        } else {
            // Jumper absent → start at OFF, then apply vault override if any.
            // Pin is already in INPUT (Hi-Z, no pull) from the setup above;
            // no second pinMode() call needed unless vault override drives
            // it to ON.
            nexus_g3_pa_runtime = G3_OFF;
            uint64_t pa_bits = nexusVariantBits & VARIANT_PA_MASK;
            if (pa_bits == VARIANT_PA_ON) {
                pinMode(PIN_PA, OUTPUT);
                digitalWrite(PIN_PA, HIGH); // active-HIGH
                nexus_g3_pa_runtime = G3_ON;
                LOG_INFO("Nexus/G3: PA (GPIO9) vault ON — applying on boot (OUTPUT HIGH).");
            } else {
                LOG_INFO("Nexus/G3: PA (GPIO9) OFF — vault default or freshly migrated (INPUT / Hi-Z).");
            }
        }

        // --- LNA (GPIO10, active-LOW) ---
        // Same pull-disable-then-read sequence as PA. Same latch-clear
        // prefix as PA (Nexus-28006, 2026-09-17 06:07 UTC) — see
        // the PA block above for the full rationale.
        digitalWrite(PIN_LNA, LOW);
        pinMode(PIN_LNA, INPUT);
        delay(1);
        int raw_lna = digitalRead(PIN_LNA);
        if (raw_lna == HIGH) {
            // Jumper installed (LNA-P pulls HIGH when present) → DISABLED.
            nexus_g3_lna_runtime = G3_DISABLED;
            LOG_INFO("Nexus/G3: LNA (GPIO10) DISABLED — physical jumper detected at boot (LNA-P). Firmware will not drive this pin.");
        } else {
            // Jumper absent → start at OFF (bypassed), then apply vault
            // override if any. Drive HIGH explicitly so the LNA-P net
            // isn't floating (a floating LNA-P can noise-couple into the
            // LNA when nothing else is driving it).
            nexus_g3_lna_runtime = G3_OFF;
            pinMode(PIN_LNA, OUTPUT);
            digitalWrite(PIN_LNA, HIGH);
            uint64_t lna_bits = (nexusVariantBits & VARIANT_LNA_MASK) >> VARIANT_LNA_SHIFT;
            if (lna_bits == VARIANT_LNA_ON) {
                digitalWrite(PIN_LNA, LOW); // active-LOW → LOW = LNA engaged
                nexus_g3_lna_runtime = G3_ON;
                LOG_INFO("Nexus/G3: LNA (GPIO10) vault ON — applying on boot (OUTPUT LOW).");
            } else {
                LOG_INFO("Nexus/G3: LNA (GPIO10) OFF — vault default or freshly migrated (OUTPUT HIGH / bypassed).");
            }
        }
    }
#endif // STATION_G3
}

void NexusModule::updateStats(const meshtastic_MeshPacket &mp) {
    // Nexus: Avoid double-counting if called from multiple firewall hooks
    static uint32_t last_stats_id = 0;
    static uint32_t last_stats_from = 0;
    if (mp.id == last_stats_id && mp.from == last_stats_from) return;
    last_stats_id = mp.id;
    last_stats_from = mp.from;

    // Filter for "Wire" packets only (LoRa or UDP) for stats/SIGINT
    bool isWirePacket = (mp.transport_mechanism == meshtastic_MeshPacket_TransportMechanism_TRANSPORT_LORA ||
                         mp.transport_mechanism == meshtastic_MeshPacket_TransportMechanism_TRANSPORT_MULTICAST_UDP ||
                         mp.transport_mechanism == meshtastic_MeshPacket_TransportMechanism_TRANSPORT_LORA_ALT1 ||
                         mp.transport_mechanism == meshtastic_MeshPacket_TransportMechanism_TRANSPORT_LORA_ALT2 ||
                         mp.transport_mechanism == meshtastic_MeshPacket_TransportMechanism_TRANSPORT_LORA_ALT3);

    if (isWirePacket) {
        rx_total_count++;

        if (mp.which_payload_variant == meshtastic_MeshPacket_encrypted_tag) {
            rx_private_count++;
        }

        // Signature verification bucketing. Only decrypted packets carry an
        // xeddsa_signature field; encrypted packets can't be inspected and
        // fall through to the NoSig bucket so the three sums cover every
        // wire packet. xeddsa_signed is reset to false by Router.cpp:check
        // XeddsaReceivePolicy() before the verify step, then set to the
        // crypto->xeddsa_verify() result — a packet that the framework
        // rejected keeps xeddsa_signed == false even if it had a signature.
        uint32_t wireBytes = 0;
        bool hasDecoded = (mp.which_payload_variant == meshtastic_MeshPacket_decoded_tag);
        if (hasDecoded) {
            wireBytes = mp.decoded.payload.size;
            if (mp.xeddsa_signed) {
                rx_sig_valid_count++;
                rx_sig_valid_bytes += wireBytes;
            } else if (mp.decoded.xeddsa_signature.size > 0) {
                rx_sig_invalid_count++;
                rx_sig_invalid_bytes += wireBytes;
            } else {
                rx_no_sig_count++;
                rx_no_sig_bytes += wireBytes;
            }
        } else {
            wireBytes = mp.encrypted.size;
            rx_no_sig_count++;
            rx_no_sig_bytes += wireBytes;
        }

        updateSigint(mp);
        updateTargetStats(mp);

        uint8_t rid = (uint8_t)(mp.relay_node & 0xFF);
        uint8_t usedHops = (mp.hop_start >= mp.hop_limit) ? (mp.hop_start - mp.hop_limit) : 0;

        if (usedHops == 0) relay_stats[rid].direct++;
        else relay_stats[rid].relayed++;

        // mp->rx_coding_rate is populated by the radio driver's addReceiveMetadata() via
        // lora.getLoRaRxHeaderInfo(&cr, &hasCRC). Counts display-domain CR 5..8 only (short
        // interleave). Long-interleave (display 9..11) and unknown (0) are excluded — those
        // packets still get counted in relay_stats / hop_matrix / port_stats above.
        // nexusCRDisplay transforms raw 1-7 -> display 5/6/7/8 (short) or 9/10/11 (long).
        uint8_t dispCR = nexusCRDisplay(mp.rx_coding_rate);
        if (dispCR >= 5 && dispCR <= 8) {
            uint8_t crIdx = dispCR - 5;
            if (usedHops == 0) cr_stats[crIdx][0]++; // direct
            else                cr_stats[crIdx][1]++; // relayed
        }

        if (usedHops < 8 && mp.hop_start < 8) {
            nexus_hop_matrix[mp.hop_start][usedHops]++;
        }

        channel_stats[mp.channel]++;
        uint32_t bytes = (mp.which_payload_variant == meshtastic_MeshPacket_decoded_tag)
                         ? mp.decoded.payload.size
                         : (mp.which_payload_variant == meshtastic_MeshPacket_encrypted_tag)
                           ? mp.encrypted.size : 0;
        channel_bytes[mp.channel] += bytes;
    }

    uint32_t t = mp.transport_mechanism;
    if (t < 8) nexus_transport_stats[t]++;

    // MQTT Stats (v235/v261-Fixed)
    bool okToMqtt = false;
    if (mp.which_payload_variant == meshtastic_MeshPacket_decoded_tag && mp.decoded.has_bitfield) {
        okToMqtt = (mp.decoded.bitfield & BITFIELD_OK_TO_MQTT_MASK);
    }

    if (mp.via_mqtt) {
        mqtt_ok_off_flag_on++; // This is the "via MQTT" counter
    } else if (okToMqtt) {
        mqtt_ok_on++;          // This is "OK to MQTT" but NOT via MQTT (RF packet with upload permission)
    } else {
        mqtt_both_off++;       // This is "OK to MQTT" OFF and NOT via MQTT (Pure RF Privacy packet)
    }

    bool isPortStatPacket = isWirePacket || (mp.transport_mechanism == meshtastic_MeshPacket_TransportMechanism_TRANSPORT_MQTT);
    if (isPortStatPacket && mp.which_payload_variant == meshtastic_MeshPacket_decoded_tag) {
        uint32_t port = mp.decoded.portnum;

        if (port == meshtastic_PortNum_TELEMETRY_APP) {
            // Static to keep stack pressure low — updateStats() runs on every
            // incoming packet and is shared by shouldDropPacket + shouldDropPrivate.
            static meshtastic_Telemetry tele;
            // init_zero == init_default for all nexus-used protos; memset is the
            // post-declaration equivalent. Used here because struct assignment from
            // a brace-enclosed initializer list isn't valid C++ (only at decl time).
            memset(&tele, 0, sizeof(tele));
            if (pb_decode_from_bytes(mp.decoded.payload.bytes, mp.decoded.payload.size, &meshtastic_Telemetry_msg, &tele)) {
                if (tele.which_variant == meshtastic_Telemetry_device_metrics_tag) nexus_tele_stats[0]++;
                else if (tele.which_variant == meshtastic_Telemetry_power_metrics_tag) nexus_tele_stats[1]++;
                else if (tele.which_variant == meshtastic_Telemetry_environment_metrics_tag) nexus_tele_stats[2]++;
                else nexus_tele_stats[3]++;
            }
        }
        if (port == meshtastic_PortNum_NODEINFO_APP) {
            // Sub-counter (v267):
            //   [0] BCAST  - normal broadcast nodeinfo announcement (not a request)
            //   [1] REQ    - non-broadcast request (DM, want_response=true)
            //   [2] B_REQ  - broadcast request (broadcast, want_response=true)
            bool isBcast = (mp.to == NODENUM_BROADCAST);
            bool isReq   = mp.decoded.want_response;
            if (isReq && isBcast)       nexus_nodeinfo_stats[2]++;
            else if (isReq)             nexus_nodeinfo_stats[1]++;
            else if (isBcast)           nexus_nodeinfo_stats[0]++;
            // else: targeted non-request nodeinfo (rare but possible). Not counted
            // in any sub-bucket to keep the three categories mutually exclusive.
        }
        if (port == meshtastic_PortNum_POSITION_APP) {
            static meshtastic_Position pos;  // ~180B; static to relieve stack pressure
            memset(&pos, 0, sizeof(pos));
            if (pb_decode_from_bytes(mp.decoded.payload.bytes, mp.decoded.payload.size, &meshtastic_Position_msg, &pos)) {
                if (pos.location_source == meshtastic_Position_LocSource_LOC_MANUAL) nexus_pos_stats[0]++;
                else if (pos.location_source == meshtastic_Position_LocSource_LOC_INTERNAL || pos.location_source == meshtastic_Position_LocSource_LOC_EXTERNAL) nexus_pos_stats[1]++;
                else nexus_pos_stats[2]++;
            }
        }
        if (mp.decoded.has_bitfield && !(mp.decoded.bitfield & BITFIELD_OK_TO_MQTT_MASK)) {
            nexus_mqtt_blocked_stats++;
        }

        int idx = -1;
        switch(port) {
            case meshtastic_PortNum_TEXT_MESSAGE_APP: idx = 0; break;
            case meshtastic_PortNum_POSITION_APP: idx = 1; break;
            case meshtastic_PortNum_NODEINFO_APP: idx = 2; break;
            case meshtastic_PortNum_TELEMETRY_APP: idx = 3; break;
            case meshtastic_PortNum_NEIGHBORINFO_APP: idx = 4; break;
            case meshtastic_PortNum_ADMIN_APP: idx = 5; break;
            case meshtastic_PortNum_TRACEROUTE_APP: idx = 6; break;
            case meshtastic_PortNum_STORE_FORWARD_APP: idx = 7; break;
            case meshtastic_PortNum_REPLY_APP: idx = 8; break;
            case meshtastic_PortNum_REMOTE_HARDWARE_APP: idx = 9; break;
            case meshtastic_PortNum_ATAK_FORWARDER: idx = 10; break;
            case meshtastic_PortNum_WAYPOINT_APP: idx = 11; break;
            case meshtastic_PortNum_IP_TUNNEL_APP: idx = 12; break;
            case meshtastic_PortNum_DETECTION_SENSOR_APP: idx = 13; break;
            case meshtastic_PortNum_PAXCOUNTER_APP: idx = 14; break;
            case meshtastic_PortNum_AUDIO_APP: idx = 16; break;
            case meshtastic_PortNum_RANGE_TEST_APP: idx = 17; break;
            case meshtastic_PortNum_RETICULUM_TUNNEL_APP: idx = 18; break;
            case meshtastic_PortNum_SERIAL_APP: idx = 19; break;
            case meshtastic_PortNum_ALERT_APP: idx = 20; break;
            case meshtastic_PortNum_KEY_VERIFICATION_APP: idx = 21; break;
            case meshtastic_PortNum_STORE_FORWARD_PLUSPLUS_APP: idx = 22; break;
            case meshtastic_PortNum_NODE_STATUS_APP: idx = 23; break;
            case meshtastic_PortNum_ATAK_PLUGIN_V2: idx = 24; break;
            case meshtastic_PortNum_MAP_REPORT_APP: idx = 25; break;
            case meshtastic_PortNum_POWERSTRESS_APP: idx = 26; break;
            case meshtastic_PortNum_LORAWAN_BRIDGE: idx = 27; break;
            case meshtastic_PortNum_CAYENNE_APP: idx = 28; break;
            case 0: idx = 15; break;
        }
        if (idx >= 0 && idx < 32) {
            nexus_port_stats[idx]++;
            uint32_t bytes = (mp.which_payload_variant == meshtastic_MeshPacket_decoded_tag)
                             ? mp.decoded.payload.size
                             : (mp.which_payload_variant == meshtastic_MeshPacket_encrypted_tag)
                               ? mp.encrypted.size : 0;
            nexus_port_bytes[idx] += bytes;
        }
    }
}

bool NexusModule::checkReplay(uint32_t id)
{
    return replayRing.check(id);
}

void NexusModule::sniffPacket(const meshtastic_MeshPacket &mp) {
    if (clientscan_active && mp.id == clientscan_target_id) {
        // Record repeat
        if (clientscan_count < 50) {
            uint32_t now = Time::getMillis();

            // Nexus: No deduplication - we want every instance of this packet ID seen on air.
            // Repetitions will have the same fromID and packetID, but unique signal data and relay node IDs.
            // Nexus port: changed from brace-init to field-by-field assignment. C++20 narrowing
            // conversion in designated-initializer-list rejected the implicit casts (e.g.
            // mp.hop_limit is uint8_t but was assigned to a uint8_t field from a larger
            // source). The explicit per-field static_cast below is C++20-clean.
            NexusClientScanEntry &e = clientscan_db[clientscan_count++];
            e.nodeID = mp.from;
            e.delay_ms = now - clientscan_start;
            e.rssi = (int8_t)mp.rx_rssi;
            e.snr = mp.rx_snr;
            // Nexus port (Nexus-28001, 2026-08-16): rx_frequency_error restored, populated by
            // the radio driver's addReceiveMetadata() (SX126x/SX128x/RF95/LR11x0).
            e.freqErr = mp.rx_frequency_error;
            e.relay = (uint8_t)(mp.relay_node & 0xFF);
            e.hop_limit = static_cast<uint8_t>(mp.hop_limit);
            e.hop_start = static_cast<uint8_t>(mp.hop_start);
            LOG_INFO("Nexus: ClientScan match from %08x via relay %02x (%ums)", mp.from, (unsigned int)(mp.relay_node & 0xFF), now - clientscan_start);
            setIntervalFromNow(100); // Wake up ASAP if we are active
        }
    }
}

bool NexusModule::wantPacket(const meshtastic_MeshPacket *p) {
    // Nexus: NETSCAN Collection (with deduplication)
    if (netscan_active && p->from != 0 && p->from != myNodeInfo.my_node_num) {
        bool found = false;
        for (int i = 0; i < nexus_netscan_count; i++) {
            if (nexus_netscan_db[i].nodeID == p->from) {
                // Update with better signal if found
                if (p->rx_rssi > nexus_netscan_db[i].rssi) {
                    nexus_netscan_db[i].rssi = (int8_t)p->rx_rssi;
                    nexus_netscan_db[i].snr = p->rx_snr;
                }
                found = true;
                break;
            }
        }
        if (!found && nexus_netscan_count < 50) {
            NexusNetScanEntry &e = nexus_netscan_db[nexus_netscan_count++];
            e.nodeID = p->from;
            e.rssi = (int8_t)p->rx_rssi;
            e.snr = p->rx_snr;
            // Nexus port (Nexus-28001, 2026-08-16): rx_frequency_error restored, populated by
            // the radio driver's addReceiveMetadata().
            e.freqErr = p->rx_frequency_error;
            e.relay = (uint8_t)(p->relay_node & 0xFF);
        }
    }

    // Nexus: Skip-Skip-Handle logic (Reverse skipHandle)
    if (skip_skip_handle) {
        LOG_DEBUG("Nexus: skip_skip_handle active, forcing wantPacket evaluation");
        return true;
    }

    if (p->decoded.portnum == meshtastic_PortNum_TEXT_MESSAGE_APP) return true;
    if (p->decoded.portnum == meshtastic_PortNum_ADMIN_APP) return true;
    if (p->decoded.portnum == meshtastic_PortNum_POSITION_APP) return true;
    if (p->decoded.portnum == meshtastic_PortNum_NODEINFO_APP) return true;
    if (p->decoded.portnum == meshtastic_PortNum_TELEMETRY_APP) return true;
    if (p->decoded.portnum == meshtastic_PortNum_TRACEROUTE_APP) return true;

    return false;
}

int NexusModule::onInputEvent(const InputEvent *event) {
    if (event->inputEvent == INPUT_BROKER_SEND_PING) {
        LOG_INFO("Nexus: Tactical Button Trigger detected!");

        // 1. Generate Tactical Report
        char report[200];
        int len = 0;

        auto node = nodeDB->getMeshNode(nodeDB->getNodeNum());
        if (node && nexus_has_position(node)) {
            len += snprintf(report + len, sizeof(report) - len, "L:%08x O:%08x H:%x\n",
                            (unsigned int)nexus_pos(node)->latitude_i, (unsigned int)nexus_pos(node)->longitude_i, (unsigned int)nexus_pos(node)->altitude);
        }

        float chUtil = (airTime != NULL) ? airTime->channelUtilizationPercent() : 0.0f;
        float txUtil = (airTime != NULL) ? airTime->utilizationTXPercent() : 0.0f;
        len += snprintf(report + len, sizeof(report) - len, "U:%.1f%% TX:%.1f%%", chUtil, txUtil);

        meshtastic_Telemetry em = meshtastic_Telemetry_init_zero;
        if (meshtastic::getNexusEnvironmentTelemetry(&em) && em.which_variant == meshtastic_Telemetry_environment_metrics_tag) {
            len += snprintf(report + len, sizeof(report) - len, " T:%.1fC", em.variant.environment_metrics.temperature);
        }

        // 2. Find Tactical Channel (Defined Trusted Channel)
        int targetCh = 0;
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

        for (uint8_t i = 0; i < MAX_NUM_CHANNELS; i++) {
            meshtastic_Channel *c = &channels.getByIndex(i);
            if (c->role != meshtastic_Channel_Role_DISABLED) {
                if (strcasecmp(c->settings.name, trusted1) == 0 || strcasecmp(c->settings.name, trusted2) == 0) {
                    targetCh = i;
                    break;
                }
            }
        }

        // 3. Queue Broadcast (Non-blocking)
        pendingReply.active = true;
        pendingReply.to = 0xFFFFFFFF;
        pendingReply.channel = targetCh;
        pendingReply.reply_id = 0;
        pendingReply.want_ack = false;
        pendingReply.send_at = Time::getMillis() + 500; // Debounce delay

        size_t rLen = strlen(report);
        if (rLen > NEXUS_MAX_REPLY_BODY) rLen = NEXUS_MAX_REPLY_BODY;
        memcpy(pendingReply.message, report, rLen);
        pendingReply.message[rLen] = '\0';

        setIntervalFromNow(50);
    }
    return 0;
}

void NexusModule::updateSigint(const meshtastic_MeshPacket &mp) {
    // Only log packets from other nodes
    if (mp.from == 0 || mp.from == myNodeInfo.my_node_num) return;

    NexusSignalEntry &e = nexus_sigint_log[nexus_sigint_idx];
    e.from = mp.from;
    e.rssi = (int8_t)mp.rx_rssi;
    e.snr = mp.rx_snr;
    // Nexus port (Nexus-28001, 2026-08-16): rx_frequency_error restored, populated by
    // the radio driver's addReceiveMetadata().
    e.freqError = mp.rx_frequency_error;
    e.timestamp = Time::getMillis() / 1000;
    e.hop_limit = static_cast<uint8_t>(mp.hop_limit);
    e.hop_start = static_cast<uint8_t>(mp.hop_start);
    e.relay_node = mp.relay_node;
    nexus_sigint_idx = (nexus_sigint_idx + 1) % 32;
    if (nexus_sigint_count < 32) nexus_sigint_count++;
}

void NexusModule::updateTargetStats(const meshtastic_MeshPacket &mp) {
    if (watchlistCount == 0) return;

    for (int i = 0; i < 8; i++) {
        if (targetWatchlist[i].nodeID != 0 && targetWatchlist[i].nodeID == mp.from) {
            NexusTargetStats &s = targetWatchlist[i];

            s.packetCount++;

            if (mp.which_payload_variant == meshtastic_MeshPacket_decoded_tag) {
                s.byteCount += mp.decoded.payload.size;
            } else if (mp.which_payload_variant == meshtastic_MeshPacket_encrypted_tag) {
                s.byteCount += mp.encrypted.size;
            }

            uint32_t usedHops = (mp.hop_start >= mp.hop_limit) ? (mp.hop_start - mp.hop_limit) : 0;

            if (usedHops == 0) {
                s.sumRssi += mp.rx_rssi;
                s.sumSnr += mp.rx_snr;
                // Nexus port (Nexus-28001, 2026-08-16): rx_frequency_error restored.
                s.sumFreqErr += mp.rx_frequency_error;
                s.sumRssi2 += (mp.rx_rssi * mp.rx_rssi);
                s.sumSnr2 += (mp.rx_snr * mp.rx_snr);
                s.sumFreqErr2 += (mp.rx_frequency_error * mp.rx_frequency_error);

                if (mp.rx_rssi < s.minRssi) s.minRssi = (int8_t)mp.rx_rssi;
                if (mp.rx_rssi > s.maxRssi) s.maxRssi = (int8_t)mp.rx_rssi;
                if (mp.rx_snr < s.minSnr) s.minSnr = mp.rx_snr;
                if (mp.rx_snr > s.maxSnr) s.maxSnr = mp.rx_snr;
                if (mp.rx_frequency_error < s.minFreqErr) s.minFreqErr = mp.rx_frequency_error;
                if (mp.rx_frequency_error > s.maxFreqErr) s.maxFreqErr = mp.rx_frequency_error;
            }

            if (usedHops < 8) s.hopDist[usedHops]++;

            uint8_t rid = (uint8_t)(mp.relay_node & 0xFF);
            s.lastRelay = rid;
            if (s.lastRelay != 0) {
                if (s.lastRelay == s.primaryRelay) {
                    s.primaryRelayCount++;
                } else if (s.primaryRelayCount == 0) {
                    s.primaryRelay = s.lastRelay;
                    s.primaryRelayCount = 1;
                } else {
                    s.primaryRelayCount--;
                }
            }

            uint32_t now = Time::getMillis();
            if (s.lastPacketTime != 0) {
                uint32_t interval = (now - s.lastPacketTime) / 1000;
                if (interval > 0) { // Avoid sub-second noise
                    if (interval < s.minInterval) s.minInterval = interval;
                    if (interval > s.maxInterval) s.maxInterval = interval;
                }
            }
            s.lastPacketTime = now;
            s.lastSeen = now / 1000;

            if (mp.which_payload_variant == meshtastic_MeshPacket_decoded_tag) {
                int pIdx = 7;
                switch(mp.decoded.portnum) {
                    case meshtastic_PortNum_NODEINFO_APP: pIdx = 0; break;
                    case meshtastic_PortNum_POSITION_APP: pIdx = 1; break;
                    case meshtastic_PortNum_TELEMETRY_APP: pIdx = 2; break;
                    case meshtastic_PortNum_TEXT_MESSAGE_APP: pIdx = 3; break;
                    case meshtastic_PortNum_ADMIN_APP: pIdx = 4; break;
                    case meshtastic_PortNum_ROUTING_APP: pIdx = 5; break;
                    case meshtastic_PortNum_STORE_FORWARD_APP: pIdx = 6; break;
                    // Nexus-28001: default catches portnums added upstream (PRIVATE_APP,
                    // ATAK_FORWARDER, TRACEROUTE_APP, etc.). Drops into the unspecified pIdx = 7 bucket.
                    default: pIdx = 7; break;
                }
                s.portDist[pIdx]++;
            }
            break;
        }
    }
}


/*
 * This function is an additional filter for invalid packets to drop
 * The origin of the function was when we started seeing nodes appearing in Null Island
 * So if you wanna completely drop this packets with this you can.
 */

bool NexusModule::shouldInvalidatePacket(const meshtastic_MeshPacket &mp) {
    if (mp.which_payload_variant != meshtastic_MeshPacket_decoded_tag) return false;

    uint32_t port = mp.decoded.portnum;

    // Rule 5: NodeID mismatch (From header vs NodeInfo payload)
    if (inv_id_mismatch && port == meshtastic_PortNum_NODEINFO_APP) {
        static meshtastic_User user;  // ~160B; static to relieve stack pressure
        memset(&user, 0, sizeof(user));
        if (pb_decode_from_bytes(mp.decoded.payload.bytes, mp.decoded.payload.size, &meshtastic_User_msg, &user)) {
            uint32_t payloadId = 0;
            if (user.id[0] == '!') {
                payloadId = strtoul(user.id + 1, NULL, 16);
            }
            if (payloadId != 0 && payloadId != mp.from) {
                LOG_WARN("Nexus: Invalid ID Mismatch - Header:!%08x vs Payload:!%08x", (unsigned int)mp.from, (unsigned int)payloadId);
                inv_drop_id++;
                return true;
            }
        }
    }

    // Rule 4: HW Unset
    if (inv_hw_unset && port == meshtastic_PortNum_NODEINFO_APP) {
        static meshtastic_User user;  // Reused across inv blocks; mutually exclusive here
        memset(&user, 0, sizeof(user));
        if (pb_decode_from_bytes(mp.decoded.payload.bytes, mp.decoded.payload.size, &meshtastic_User_msg, &user)) {
            if (user.hw_model == meshtastic_HardwareModel_UNSET) {
                LOG_WARN("Nexus: Invalid HW Unset from !%08x", (unsigned int)mp.from);
                inv_drop_hw++;
                return true;
            }
        }
    }

    // Position Rules (1, 2, 3)
    if (port == meshtastic_PortNum_POSITION_APP) {
        static meshtastic_Position pos;  // ~180B; static to relieve stack pressure
        memset(&pos, 0, sizeof(pos));
        if (pb_decode_from_bytes(mp.decoded.payload.bytes, mp.decoded.payload.size, &meshtastic_Position_msg, &pos)) {
            float lat = (float)pos.latitude_i / 1e7f;
            float lon = (float)pos.longitude_i / 1e7f;

            // Rule 1: Null Island drop (gated by inv_null_island).
            // Catches both exact (0,0) and near (0,0) coords (external bug injector uses 0,0 pure).
            // Note: legitimate "no fix" packets report lat=0 && lon=0, but they're rare and acceptable to drop when flag is ON.
            if (inv_null_island) {
                bool exactZero = (pos.latitude_i == 0 && pos.longitude_i == 0);
                bool nearZero = (fabs(lat) < 0.01f && fabs(lon) < 0.01f);
                if (exactZero || nearZero) {
                    LOG_WARN("Nexus: Invalid Null Island from !%08x (lat=%d, lon=%d)", (unsigned int)mp.from, pos.latitude_i, pos.longitude_i);
                    inv_drop_null++;
                    return true;
                }
            }

            // Rule 2: Outside Spain
            // Peninsula: 36N to 44N, 9.5W to 4.5E
            // Canary: 27N to 30N, 19W to 13W
            if (inv_out_spain) {
                bool inPeninsula = (lat >= 35.0f && lat <= 44.0f && lon >= -10.0f && lon <= 5.0f);
                bool inCanary = (lat >= 27.0f && lat <= 30.0f && lon >= -19.0f && lon <= -13.0f);
                if (!inPeninsula && !inCanary) {
                    LOG_WARN("Nexus: Invalid Outside Spain from !%08x (%.4f, %.4f)", (unsigned int)mp.from, lat, lon);
                    inv_drop_spain++;
                    return true;
                }
            }

            // Rule 3: Outside Germany
            // Approx: 47N to 55N, 5.5E to 15.5E
            if (inv_out_germany) {
                bool inGermany = (lat >= 47.0f && lat <= 55.5f && lon >= 5.5f && lon <= 15.5f);
                if (!inGermany) {
                    LOG_WARN("Nexus: Invalid Outside Germany from !%08x (%.4f, %.4f)", (unsigned int)mp.from, lat, lon);
                    inv_drop_germany++;
                    return true;
                }
            }
        }
    }

    return false;
}

bool NexusModule::checkRateLimit(const meshtastic_MeshPacket &mp) {
    // Tokens refill at limit/3600 per ms.
    // pkts/hr / 3600.0f = pkts/sec.
    // pkts/sec / 1000.0f = pkts/ms.
    // float refill_per_ms = (float)limit / 3600000.0f;

    uint32_t port = 0;
    bool isEncrypted = (mp.which_payload_variant == meshtastic_MeshPacket_encrypted_tag);
    if (isEncrypted) {
        if (!drop_rl_private) return true;
    } else if (mp.which_payload_variant == meshtastic_MeshPacket_decoded_tag) {
        port = mp.decoded.portnum;
    } else {
        return true; // Unknown variant, don't RL
    }

    int idx = -1;
    uint16_t limit = 0;
    if (isEncrypted) {
        idx = 6;
        limit = rl_private_limit;
    } else if (port == meshtastic_PortNum_TEXT_MESSAGE_APP || port == meshtastic_PortNum_TEXT_MESSAGE_COMPRESSED_APP) {
        if (!drop_rl_text) return true;
        idx = 0;
        limit = rl_text_limit;
    } else if (port == meshtastic_PortNum_TRACEROUTE_APP) {
        if (!drop_rl_trace) return true;
        idx = 1;
        limit = rl_trace_limit;
    } else if (port == meshtastic_PortNum_NODEINFO_APP) {
        if (!drop_rl_nodeinfo) return true;
        idx = 3;
        limit = rl_nodeinfo_limit;
    } else if (port == meshtastic_PortNum_POSITION_APP) {
        if (!drop_rl_position) return true;
        idx = 4;
        limit = rl_position_limit;
    } else if (port == meshtastic_PortNum_TELEMETRY_APP) {
        if (!drop_rl_telemetry) return true;
        idx = 5;
        limit = rl_telemetry_limit;
    } else {
        if (!drop_rl_other) return true;
        idx = 2;
        limit = rl_other_limit;
    }

    if (idx == -1 || limit == 0) return true;

    // v267: seen-counter increments only when the RL is actually engaged (idx resolved,
    // limit > 0). Bypasses (drop_rl_* = OFF) are not counted because the RL is not
    // regulating that bucket. seen = pass + drop by arithmetic.
    switch (idx) {
        case 0: rl_seen_text++; break;
        case 1: rl_seen_trace++; break;
        case 2: rl_seen_other++; break;
        case 3: rl_seen_nodeinfo++; break;
        case 4: rl_seen_position++; break;
        case 5: rl_seen_telemetry++; break;
        case 6: rl_seen_private++; break;
    }

    // Refill tokens for all buckets based on elapsed time
    uint32_t now = Time::getMillis();
    uint32_t elapsed = (now - rl_last_refill);
    rl_last_refill = now;
    float refill = (float)elapsed / 3600000.0f; // milliseconds to hours
    static const uint16_t* limits[7] = {
        &rl_text_limit, &rl_trace_limit, &rl_other_limit,
        &rl_nodeinfo_limit, &rl_position_limit, &rl_telemetry_limit, &rl_private_limit
    };
    for (int i=0; i<7; i++) {
        rl_tokens[i] += refill * (float)(*limits[i]);
        if (rl_tokens[i] > (float)(*limits[i])) rl_tokens[i] = (float)(*limits[i]); // Burst limit = 1hr worth
    }

    if (rl_tokens[idx] >= 1.0f) {
        rl_tokens[idx] -= 1.0f;
        switch (idx) {
            case 0: rl_pass_text++; break;
            case 1: rl_pass_trace++; break;
            case 2: rl_pass_other++; break;
            case 3: rl_pass_nodeinfo++; break;
            case 4: rl_pass_position++; break;
            case 5: rl_pass_telemetry++; break;
            case 6: rl_pass_private++; break;
        }
        return true; // Pass
    }

    LOG_WARN("Nexus: Rate limit exceeded (bucket %d, Limit: %u pkts/hr)", idx, (unsigned int)limit);
    switch (idx) {
        case 0: rl_drop_text++; break;
        case 1: rl_drop_trace++; break;
        case 2: rl_drop_other++; break;
        case 3: rl_drop_nodeinfo++; break;
        case 4: rl_drop_position++; break;
        case 5: rl_drop_telemetry++; break;
        case 6: rl_drop_private++; break;
    }
    return false; // Drop
}

bool NexusModule::checkMasterKeys(const uint8_t *senderPubKey) {
    static const uint8_t* masterKeys[] = {
#ifdef NEXUS_MASTER_KEY_1
        (const uint8_t[])NEXUS_MASTER_KEY_1,
#endif
#ifdef NEXUS_MASTER_KEY_2
        (const uint8_t[])NEXUS_MASTER_KEY_2,
#endif
#ifdef NEXUS_MASTER_KEY_3
        (const uint8_t[])NEXUS_MASTER_KEY_3,
#endif
#ifdef NEXUS_MASTER_KEY_4
        (const uint8_t[])NEXUS_MASTER_KEY_4,
#endif
    };

    const int numKeys = sizeof(masterKeys) / sizeof(masterKeys[0]);
    // Nexus-28006 (2026-09-16) — code review 1C fix.
    // Previously: if (numKeys == 0) return false;
    //
    // That early-return short-circuited the secondary-keys loop below on
    // community/community-flavored builds where none of
    // NEXUS_MASTER_KEY_{1..4} are #define'd. /nexus SKEY stores
    // runtime-mutable keys in the vault specifically because community
    // builds without baked-in master keys need a way to grant trust
    // without rebuilding firmware — but the early-return meant the
    // secondary-keys loop NEVER ran on those builds, making /nexus SKEY
    // effectively broken (the command accepted keys but they were never
    // consulted on incoming packets).
    //
    // Removing the early-return is safe: the for-loop below is already
    // a no-op when numKeys == 0, so behavior on master-key builds is
    // unchanged. On no-master-key builds, execution now falls through
    // to the secondary-keys loop as intended.
    //
    // DO NOT RE-ADD the early-return. The secondary-keys path is the
    // trust mechanism for community builds; short-circuiting it was a
    // silent regression that masked /nexus SKEY as broken.

    for (int i = 0; i < numKeys; i++) {
        if (memcmp(senderPubKey, masterKeys[i], 32) == 0) {
            return true;
        }
    }

    // Nexus: Check dynamic secondary keys from vault
    for (int i = 0; i < 4; i++) {
        // Only check if key is not all zeros
        bool allZero = true;
        for(int j=0; j<32; j++) { if(nexus_secondary_keys[i][j] != 0) { allZero = false; break; } }
        if (!allZero && memcmp(senderPubKey, nexus_secondary_keys[i], 32) == 0) {
            return true;
        }
    }

    return false;
}

bool NexusModule::isAuthorizedAdmin(const uint8_t *senderPubKey)
{
    // Defensive: inline OR chains in Nexus-269 / 28002 / 28001 would
    // segfault on null pubkey. Helper callers (AdminModule) pass
    // mp.public_key.bytes which is always 32 bytes, but a null guard
    // is cheap and makes the helper safe to reuse anywhere.
    if (!senderPubKey)
        return false;

    // 1. Per-device admin keys (USERPREFS_USE_ADMIN_KEY_0/1/2 ->
    //    config.security.admin_key[0..2]). size == 32 is the canonical
    //    "loaded" gate, mirrors NodeDB.cpp:1027/1036/1045.
    for (int i = 0; i < 3; i++) {
        const auto &k = config.security.admin_key[i];
        if (k.size == 32 && memcmp(senderPubKey, k.bytes, 32) == 0)
            return true;
    }

    // 2. Master keys (NEXUS_MASTER_KEY_1..4) + secondary_keys[0..3].
    //    checkMasterKeys() already covers both sources and is the same
    //    code path used by AdminModule in Nexus-269 and the /nexus DM
    //    tombstone path in Nexus-28001+. Reusing it here guarantees
    //    AdminModule and any future caller see the same trust list.
    return checkMasterKeys(senderPubKey);
}

bool NexusModule::verifyTrustedSignature(const meshtastic_MeshPacket *p, char *outMatch, size_t outSize)
{
    // Sentinel default. Callers must pass a non-null outMatch buffer;
    // we use snprintf into it so any size is safe but always NUL-terminated.
    if (outMatch && outSize > 0)
        outMatch[0] = '\0';
    if (!p)
        return false;

    // 1. Signature must be present and well-formed (full 64-byte XEdDSA
    //    signature). Anything else — 0 bytes (no signature) or a partial
    //    truncated field (malformed / hostile) — short-circuits to "no
    //    signature". Same policy as Router::checkXeddsaReceivePolicy.
    const auto &sig = p->decoded.xeddsa_signature;
    if (sig.size != XEDDSA_SIGNATURE_SIZE) {
        if (outMatch && outSize >= 13) strcpy(outMatch, "no_signature");
        return false;
    }

    // No crypto engine wired up (defensive — should never happen on a
    // production build, but matches the operator-friendly "trust me bro"
    // fallback in upstream verifyFirstContactNodeInfo).
    if (!crypto) {
        if (outMatch && outSize >= 5) strcpy(outMatch, "none");
        return false;
    }

    // 2. Try admin_key[0..2]. These are populated from
    //    USERPREFS_USE_ADMIN_KEY_0/1/2 build-time (NodeDB.cpp:92-98) and
    //    are 32 bytes when populated, 0-byte when not. size == 32 is the
    //    canonical "loaded" gate; mirrors NodeDB.cpp:1027/1036/1045.
    for (int i = 0; i < 3; i++) {
        const auto &k = config.security.admin_key[i];
        if (k.size == 32) {
            if (crypto->xeddsa_verify(k.bytes, p->from, p->id, p->to, &p->decoded, sig.bytes)) {
                if (outMatch && outSize >= 4) snprintf(outMatch, outSize, "A%d", i);
                LOG_DEBUG("verifyTrustedSignature: matched admin_key[%d] (A%d) for 0x%08x", i, i, p->from);
                return true;
            }
        }
    }

    // 3. Try nexus_secondary_keys[0..3] (vault-populated, dynamic). All-zero
    //    sentinel means "uninitialized" — same check checkMasterKeys uses.
    for (int i = 0; i < 4; i++) {
        bool allZero = true;
        for (int j = 0; j < 32; j++) {
            if (nexus_secondary_keys[i][j] != 0) { allZero = false; break; }
        }
        if (!allZero) {
            if (crypto->xeddsa_verify(nexus_secondary_keys[i], p->from, p->id, p->to, &p->decoded, sig.bytes)) {
                if (outMatch && outSize >= 4) snprintf(outMatch, outSize, "S%d", i);
                LOG_DEBUG("verifyTrustedSignature: matched secondary_keys[%d] (S%d) for 0x%08x", i, i, p->from);
                return true;
            }
        }
    }

    // 4. Try hardcoded master keys (build-time -DNEXUS_MASTER_KEY_1..4).
    //    Reuse the same table checkMasterKeys builds — guaranteed identical
    //    iteration order and compile-time-gated slots.
    static const uint8_t *masterKeys[] = {
#ifdef NEXUS_MASTER_KEY_1
        (const uint8_t[])NEXUS_MASTER_KEY_1,
#endif
#ifdef NEXUS_MASTER_KEY_2
        (const uint8_t[])NEXUS_MASTER_KEY_2,
#endif
#ifdef NEXUS_MASTER_KEY_3
        (const uint8_t[])NEXUS_MASTER_KEY_3,
#endif
#ifdef NEXUS_MASTER_KEY_4
        (const uint8_t[])NEXUS_MASTER_KEY_4,
#endif
    };
    const int numMasterKeys = sizeof(masterKeys) / sizeof(masterKeys[0]);
    for (int i = 0; i < numMasterKeys; i++) {
        if (crypto->xeddsa_verify(masterKeys[i], p->from, p->id, p->to, &p->decoded, sig.bytes)) {
            // Label index matches the -DNEXUS_MASTER_KEY_<N+1> build-flag
            // suffix so operators reading their build flags see the same
            // numbering as the reply. N+1 because build flags are 1-indexed.
            if (outMatch && outSize >= 4) snprintf(outMatch, outSize, "M%d", i + 1);
            LOG_DEBUG("verifyTrustedSignature: matched master_key[%d] (M%d) for 0x%08x", i + 1, i + 1, p->from);
            return true;
        }
    }

    // 5. No candidate verified. Report "none" so the caller can distinguish
    //    "no signature" (early-exit) from "signature present but no trust-
    //    list key verified it" (security audit log line).
    if (outMatch && outSize >= 5) strcpy(outMatch, "none");
    return false;
}

int NexusModule::nexus_base64_decode(unsigned char *out, size_t out_len, size_t *out_written, const char *in) {
    static const int b64_table[256] = {
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 62, -1, -1, -1, 63, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, -1, -1, -1, -1, -1, -1,
        -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, -1, -1, -1, -1, -1,
        -1, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, -1, -1, -1, -1, -1
    };
    int val = 0, valb = -8;
    size_t count = 0;
    for (const char *p = in; *p; p++) {
        if (b64_table[(int)*p] == -1) continue;
        val = (val << 6) + b64_table[(int)*p];
        valb += 6;
        if (valb >= 0) {
            if (count >= out_len) return -1;
            out[count++] = (uint8_t)((val >> valb) & 0xFF);
            valb -= 8;
        }
    }
    *out_written = count;
    return 0;
}

bool NexusModule::looksLikeForeignPKI(const meshtastic_MeshPacket &mp) const
{
    // Wire-shape predicate for an un-decoded PKI DM from a foreign node. Reached when the
    // firmware can't decrypt (no destination private key on this node) — mp.pki_encrypted is
    // only set on successful decrypt, so we have to look at the wire shape here.
    //
    // True iff: payload is still wire-encrypted, the PKI flag is NOT set (else the caller
    // handles it directly), the channel hash is 0 (PKI on the wire always uses channel 0, see
    // perhapsEncode at Router.cpp:1347 / 1372), and the destination is a specific non-us
    // node (PKI DMs aren't broadcast, and PKI DMs to us get decoded → handled by
    // shouldDropPacket's drop_pki_local rule instead).
    if (mp.pki_encrypted)
        return false;
    if (mp.which_payload_variant != meshtastic_MeshPacket_encrypted_tag)
        return false;
    if (mp.channel != 0)
        return false;
    if (isBroadcast(mp.to))
        return false;
    if (isToUs(&mp))
        return false;
    return true;
}

bool NexusModule::shouldDropPacket(const meshtastic_MeshPacket &mp) {
    fw_total_processed++;
    updateStats(mp);
    // 0. Alert Logic (Priority: fire before bypasses)
    if (mp.which_payload_variant == meshtastic_MeshPacket_decoded_tag) {
        // ... (skipping long trace alert block for brevity, I'll use the specific start/end markers)

        uint32_t port = mp.decoded.portnum;

        // Trace Alert Logic
        if (trace_alert && mp.to == myNodeInfo.my_node_num && port == meshtastic_PortNum_TRACEROUTE_APP) {
            uint8_t hopsUsed = (mp.hop_start >= mp.hop_limit) ? (mp.hop_start - mp.hop_limit) : 0;
            bool isDirect = (hopsUsed == 0);
            bool shouldAlert = false;
            if (trace_alert_mode == 2) shouldAlert = true;
            else if (trace_alert_mode == 0 && isDirect) shouldAlert = true;
            else if (trace_alert_mode == 1 && !isDirect) shouldAlert = true;

            if (shouldAlert) {
                // Static (BSS, not stack). shouldDropPacket() runs for every
                // incoming packet on the router task; 160B on stack adds up.
                // memset(0) at start keeps strlen() honest (snprintf-with-offset
                // accumulation depends on finding '\0' at the previous l).
                static char alertBuf[160];
                memset(alertBuf, 0, sizeof(alertBuf));
                auto fromNode = nodeDB->getMeshNode(mp.from);
                const char *name = (fromNode && fromNode->long_name[0] != '\0') ? fromNode->long_name : "Unknown";

                size_t l = 0;
                bool alertFull = false;
                const size_t alertCap = sizeof(alertBuf);

                // Safe-append: writes into alertBuf at offset l, advancing l on
                // success. If the format would overflow, sets alertFull and clamps
                // l to alertCap-1 (the null terminator). Without this clamp, the
                // snprintf(size_t)-l underflows when l > cap, which previously
                // caused unbounded writes past the BSS buffer on long routes.
                auto safe_append = [&](const char *fmt, ...) __attribute__((format(printf, 2, 3))) {
                    if (alertFull) return;
                    if (l + 1 >= alertCap) { l = alertCap - 1; alertBuf[l] = '\0'; alertFull = true; return; }
                    va_list ap; va_start(ap, fmt);
                    int w = vsnprintf(alertBuf + l, alertCap - l, fmt, ap);
                    va_end(ap);
                    if (w < 0) { l = alertCap - 1; alertBuf[l] = '\0'; alertFull = true; return; }
                    if ((size_t)w >= alertCap - l) {
                        // vsnprintf truncated; alertBuf is already null-terminated
                        // by vsnprintf itself. Clamp l to the last writable index.
                        l = alertCap - 1;
                        alertFull = true;
                    } else {
                        l += (size_t)w;
                    }
                };

                safe_append("⚠️ TRACE from %s (!%08x) [%u hops]: ",
                            name, (unsigned int)mp.from, (unsigned int)hopsUsed);

                // Probe/reply payload is encoded as RouteDiscovery directly (not the
                // Routing oneof wrapper). Decoding as Routing silently fails and the
                // alert prints only the header + footer with no path info.
                // See TraceRouteModule::startTraceRoute (line ~634) and allocReply
                // (line ~511), both pb_encode_to_bytes with meshtastic_RouteDiscovery_msg.
                static meshtastic_RouteDiscovery rd;  // ~80B; static to relieve stack pressure
                memset(&rd, 0, sizeof(rd));
                if (!alertFull && pb_decode_from_bytes(mp.decoded.payload.bytes, mp.decoded.payload.size, &meshtastic_RouteDiscovery_msg, &rd)) {
                    bool hasForward = rd.route_count > 0;
                    bool hasBack = rd.route_back_count > 0;
                    if (hasForward) {
                        safe_append("Fwd:[");
                        for (pb_size_t i = 0; i < rd.route_count && !alertFull; i++) {
                            // Prefer the relay's short_name from nodeDB; fall back
                            // to last 4 hex chars of the full node num if the entry
                            // is unknown or has no short_name yet. Match by FULL
                            // num (not low byte) so distinct relay nodes sharing a
                            // low-byte suffix aren't conflated.
                            const char *rname = nullptr;
                            for (size_t k = 0; k < nodeDB->getNumMeshNodes(); k++) {
                                auto n = nodeDB->getMeshNodeByIndex(k);
                                if (n && n->num == rd.route[i] && n->short_name[0] != '\0') {
                                    rname = n->short_name;
                                    break;
                                }
                            }
                            if (rname) safe_append("%s", rname);
                            else       safe_append("!%04x", (unsigned int)(rd.route[i] & 0xFFFF));
                            if (i < rd.snr_towards_count) safe_append("(%.1f)", (float)rd.snr_towards[i] / 4.0f);
                            if (i < rd.route_count - 1) safe_append(">");
                        }
                        safe_append(">ME] ");
                    }
                    if (!alertFull && hasBack) {
                        safe_append("Back:[ME>");
                        for (pb_size_t i = 0; i < rd.route_back_count && !alertFull; i++) {
                            if (i < rd.snr_back_count) safe_append("(%.1f)", (float)rd.snr_back[i] / 4.0f);
                            const char *rname = nullptr;
                            for (size_t k = 0; k < nodeDB->getNumMeshNodes(); k++) {
                                auto n = nodeDB->getMeshNodeByIndex(k);
                                if (n && n->num == rd.route_back[i] && n->short_name[0] != '\0') {
                                    rname = n->short_name;
                                    break;
                                }
                            }
                            if (rname) safe_append("%s", rname);
                            else       safe_append("!%04x", (unsigned int)(rd.route_back[i] & 0xFFFF));
                            if (i < rd.route_back_count - 1) safe_append(">");
                        }
                        safe_append("] ");
                    }
                    if (!alertFull && !hasForward && !hasBack) safe_append("[DIRECT] ");
                }
                if (!alertFull) {
                    // Nexus port (Nexus-28001, 2026-08-16): rx_frequency_error + rx_coding_rate
                    // both restored. FE comes from lora.getFrequencyError(); CR from
                    // lora.getLoRaRxHeaderInfo(&cr, &hasCRC). Both Nexus-only operator-data
                    // extensions, never serialized.
                    safe_append("%ddBm SNR:%.1f FE:%.0fHz CR:%d", (int)mp.rx_rssi, mp.rx_snr,
                                mp.rx_frequency_error, nexusCRDisplay(mp.rx_coding_rate));
                }
                if (alertFull && alertCap >= 4) {
                    // Truncation marker: replace last 3 chars with "...". vsnprintf
                    // already null-terminated the buffer at alertCap-1, so we only
                    // need to overwrite the three bytes before it.
                    alertBuf[alertCap - 4] = '.';
                    alertBuf[alertCap - 3] = '.';
                    alertBuf[alertCap - 2] = '.';
                }

                size_t aLen = strlen(alertBuf);
                // Local copy to phone UI (forward rx metadata from original packet)
                sendToPhone(0xFFFFFFFF, mp.channel, 0, alertBuf, aLen, false, &mp);
                // Broadcast to first Nexus channel
                sendToNexusChannel(alertBuf, aLen);
                memset(alertBuf, 0, sizeof(alertBuf));
            }
        }

        // Metadata Request Alert
        if (meta_alert && mp.to == myNodeInfo.my_node_num && port == meshtastic_PortNum_ADMIN_APP) {
            // AdminMessage union holds Config/ModuleConfig (~600B). Static to keep
            // stack usage bounded; router task has limited headroom.
            static meshtastic_AdminMessage admin;
            memset(&admin, 0, sizeof(admin));
            if (pb_decode_from_bytes(mp.decoded.payload.bytes, mp.decoded.payload.size, &meshtastic_AdminMessage_msg, &admin)) {
                if (admin.which_payload_variant == meshtastic_AdminMessage_get_device_metadata_request_tag) {
                    uint8_t hopsUsed = (mp.hop_start >= mp.hop_limit) ? (mp.hop_start - mp.hop_limit) : 0;
                    bool isDirect = (hopsUsed == 0);
                    bool shouldAlert = false;
                    if (meta_alert_mode == 2) shouldAlert = true;
                    else if (meta_alert_mode == 0 && isDirect) shouldAlert = true;
                    else if (meta_alert_mode == 1 && !isDirect) shouldAlert = true;

                    if (shouldAlert) {
                        // Static (BSS, not stack). Same rationale as the trace_alert
                        // block above. Mutually exclusive (different alert types)
                        // so they can share the name in different scopes.
                        static char alertBuf[160];
                        memset(alertBuf, 0, sizeof(alertBuf));
                        auto fromNode = nodeDB->getMeshNode(mp.from);
                        const char *name = (fromNode && fromNode->long_name[0] != '\0') ? fromNode->long_name : "Unknown";
                        static char relayStr[24];  // Below the 64B threshold but kept static for consistency
                        strcpy(relayStr, "DIRECT");
                        if (hopsUsed > 0) {
                            uint8_t relayId = (uint8_t)(mp.relay_node & 0xFF);
                            bool foundRelay = false;
                            for (size_t k = 0; k < nodeDB->getNumMeshNodes(); k++) {
                                auto n = nodeDB->getMeshNodeByIndex(k);
                                if (n && (uint8_t)(n->num & 0xFF) == relayId && n->short_name[0] != '\0') {
                                    snprintf(relayStr, sizeof(relayStr), "via %s(*%02x)", n->short_name, relayId);
                                    foundRelay = true;
                                    break;
                                }
                            }
                            if (!foundRelay) snprintf(relayStr, sizeof(relayStr), "via *%02x", relayId);
                        }
                        // Nexus port (Nexus-28001, 2026-08-16): rx_frequency_error + rx_coding_rate restored. See
                        // SX126xInterface.cpp for the per-driver hooks.
                        snprintf(alertBuf, sizeof(alertBuf), "⚠️ META REQ from %s (!%08x) [%u/%u] %s %ddBm SNR:%.1f FE:%.0fHz CR:%d",
                                 name, (unsigned int)mp.from, (unsigned int)hopsUsed, (unsigned int)mp.hop_start,
                                 relayStr, (int)mp.rx_rssi, mp.rx_snr, mp.rx_frequency_error,
                                 nexusCRDisplay(mp.rx_coding_rate));

                        size_t aLen = strlen(alertBuf);
                        sendToPhone(0xFFFFFFFF, mp.channel, 0, alertBuf, aLen, false, &mp);
                        sendToNexusChannel(alertBuf, aLen);
                        memset(alertBuf, 0, sizeof(alertBuf));
                    }
                }
            }
        }
    }

    bool isNexusChannel = this->isNexusChannel(mp.channel);

    // Nexus Trusted Channels bypass ALL firewall drops
    if (isNexusChannel) {
        fw_nexus_bypass++;
        return false;
    }

    // Nexus: Global Favorite Bypass
    // If active, any node marked as 'favorite' in NodeDB bypasses ALL subsequent firewall rules.
    // Checks both sender (mp.from) and recipient (mp.to) for maximum flexibility in 1:1 
    if (fav_bypass) {
        auto fromNode = nodeDB->getMeshNode(mp.from);
        if (fromNode && nexus_is_favorite(fromNode)) {
            fw_fav_bypass++;
            return false;
        }

        if (mp.to != 0xFFFFFFFF) {
            auto toNode = nodeDB->getMeshNode(mp.to);
            if (toNode && nexus_is_favorite(toNode)) {
                fw_fav_bypass++;
                return false;
            }
        }
    }

    // 0.5 Nexus Ignore List Check
    if (isNexusIgnored(mp.from)) {
        LOG_DEBUG("Nexus: Packet dropped from Nexus-Ignored node !%08x", (unsigned int)mp.from);
        fw_drop_nexus_ignored++;
        return true;
    }

    // Nexus: Relay Node Drop Filter (256-bit mask)
    uint8_t relayId = (uint8_t)(mp.relay_node & 0xFF);
    if (isRelayDropped(relayId)) {
        LOG_DEBUG("Nexus: Packet dropped from Relay-Dropped node *%02X", relayId);
        fw_drop_relay++;
        return true;
    }

    // 0. PreHop Filter (Validation of hop_start metrics)
    if (drop_prehop && !isFromUs(&mp)) {
        if (classifyHopStart(mp) != HopStartStatus::VALID) {
            fw_drop_prehop++;
            return true;
        }
    }

    // 0. PKI/DM Filter
    if (mp.pki_encrypted) {
        bool isLocalPKI = (mp.from == myNodeInfo.my_node_num || mp.to == myNodeInfo.my_node_num);
        // drop_pki_local auto-couples to drop_pki (global): an operator enabling global PKI drop
        // almost never wants their own PKI DMs through. Local drop fires first so the local
        // counter tracks local PKI specifically regardless of which toggle triggered it. Foreign
        // PKI handled by shouldDropPrivate() since un-decodable PKI DMs land on the opaque path.
        if ((drop_pki_local || drop_pki) && isLocalPKI) {
            fw_drop_pki_local++;
            return true;
        }
        if (drop_pki) {
            fw_drop_pki++;
            return true;
        }
        if (drop_pki_remote && !isLocalPKI) {
            fw_drop_pki_remote++;
            return true;
        }
        return false; // Bypass other rules if not explicitly dropped
    }

    // Whitelist/Blacklist filter is exclusively for encrypted packets (shouldDropPrivate).
    // shouldDropPacket() only sees decoded packets; decrypted wire-level channel filtering
    // is handled at the encrypted path. see shouldDropPrivate() for the actual checks.

    // 2. Mesh Congestion Filters (Hop-based)
    if (drop_hop6 && mp.hop_start == 6) {
        fw_drop_hop6++;
        return true;
    }
    if (drop_hop7 && mp.hop_start == 7) {
        fw_drop_hop7++;
        return true;
    }

    if (mp.which_payload_variant == meshtastic_MeshPacket_decoded_tag) {
        uint32_t port = mp.decoded.portnum;

        // 0. Stealth ACKs (Filter incoming acknowledgments if requested)
        if (drop_acks && port == meshtastic_PortNum_ROUTING_APP) {
            fw_drop_acks++;
            return true;
        }

        // 3. Block standard protocol requests (want_response = true)
        // This prevents the node from automatically formulating a reply to specific queries.
        if (mp.decoded.want_response) {
            if (port == meshtastic_PortNum_NODEINFO_APP && drop_req_nodeinfo) { fw_drop_req_nodeinfo++; return true; }
            if (port == meshtastic_PortNum_POSITION_APP && drop_req_position) { fw_drop_req_position++; return true; }
            if (port == meshtastic_PortNum_TELEMETRY_APP && drop_req_telemetry) { fw_drop_req_telemetry++; return true; }
        }

        // 3a. Broadcast-specific request filters (v267)
        // These only fire on requests sent to broadcast (mp.to == NODENUM_BROADCAST).
        // DM/targeted requests pass through regardless, so an operator can still
        // probe a single node directly even with these drops active.
        if (mp.decoded.want_response && mp.to == NODENUM_BROADCAST) {
            if (port == meshtastic_PortNum_NODEINFO_APP  && drop_br_nodeinfo)  { fw_drop_br_nodeinfo++;  return true; }
            if (port == meshtastic_PortNum_TELEMETRY_APP && drop_br_telemetry) { fw_drop_br_telemetry++; return true; }
            if (port == meshtastic_PortNum_POSITION_APP  && drop_br_position)  { fw_drop_br_position++;  return true; }
        }

        // 3b. Rate Limiting (Nexus v28)
        if (!checkRateLimit(mp)) return true;

        // 3c. Invalid Filters (Nexus v29)
        if (shouldInvalidatePacket(mp)) return true;

        // 4. Block specific ports from being repeated/processed
        if (port == meshtastic_PortNum_NODEINFO_APP && drop_nodeinfo) { fw_drop_nodeinfo++; return true; }
        if (port == meshtastic_PortNum_POSITION_APP) {
            if (drop_position) { fw_drop_position++; return true; }
            static meshtastic_Position pos;  // ~180B; static to relieve stack pressure
            memset(&pos, 0, sizeof(pos));
            if (pb_decode_from_bytes(mp.decoded.payload.bytes, mp.decoded.payload.size, &meshtastic_Position_msg, &pos)) {
                if (drop_pos_manual && pos.location_source == meshtastic_Position_LocSource_LOC_MANUAL) { fw_drop_pos_manual++; return true; }
                if (drop_pos_gps && (pos.location_source == meshtastic_Position_LocSource_LOC_INTERNAL || pos.location_source == meshtastic_Position_LocSource_LOC_EXTERNAL)) { fw_drop_pos_gps++; return true; }
                if (drop_pos_other && pos.location_source > meshtastic_Position_LocSource_LOC_EXTERNAL) { fw_drop_pos_other++; return true; }
            }
        }
        if (port == meshtastic_PortNum_TELEMETRY_APP) {
            if (drop_telemetry) { fw_drop_telemetry++; return true; }
            // Telemetry union holds the largest variant (LocalStats); ~200B total.
            static meshtastic_Telemetry tele;
            memset(&tele, 0, sizeof(tele));
            if (pb_decode_from_bytes(mp.decoded.payload.bytes, mp.decoded.payload.size, &meshtastic_Telemetry_msg, &tele)) {
                if (drop_tele_device && tele.which_variant == meshtastic_Telemetry_device_metrics_tag) { fw_drop_tele_device++; return true; }
                if (drop_tele_power && tele.which_variant == meshtastic_Telemetry_power_metrics_tag) { fw_drop_tele_power++; return true; }
                if (drop_tele_env && tele.which_variant == meshtastic_Telemetry_environment_metrics_tag) { fw_drop_tele_env++; return true; }
                if (drop_tele_other && tele.which_variant != meshtastic_Telemetry_device_metrics_tag &&
                    tele.which_variant != meshtastic_Telemetry_power_metrics_tag && tele.which_variant != meshtastic_Telemetry_environment_metrics_tag) { fw_drop_tele_other++; return true; }
            }
        }
        if (drop_no_mqtt && mp.decoded.has_bitfield && !(mp.decoded.bitfield & BITFIELD_OK_TO_MQTT_MASK)) { fw_drop_no_mqtt++; return true; }
        if (port == meshtastic_PortNum_NEIGHBORINFO_APP && drop_neighbor) { fw_drop_neighbor++; return true; }
        if (port == meshtastic_PortNum_TRACEROUTE_APP && drop_traceroute) { fw_drop_traceroute++; return true; }

        // 4b. Block Store and Forward
        if (drop_store_forward && (port == meshtastic_PortNum_STORE_FORWARD_APP || port == meshtastic_PortNum_STORE_FORWARD_PLUSPLUS_APP)) {
            fw_drop_store_forward++;
            return true;
        }

        // 4c. Block Admin, Text, KeyVer, Alert
        if (port == meshtastic_PortNum_ADMIN_APP && drop_admin) { fw_drop_admin++; return true; }
        if ((port == meshtastic_PortNum_TEXT_MESSAGE_APP || port == meshtastic_PortNum_TEXT_MESSAGE_COMPRESSED_APP) && drop_text) { fw_drop_text++; return true; }
        if (port == meshtastic_PortNum_KEY_VERIFICATION_APP && drop_keyver) { fw_drop_keyver++; return true; }
        if (port == meshtastic_PortNum_ALERT_APP && drop_alert) { fw_drop_alert++; return true; }
        if (port == meshtastic_PortNum_WAYPOINT_APP && drop_waypoint) { fw_drop_waypoint++; return true; }

        // 5. Granular TRACEROUTE filtering
        if (port == meshtastic_PortNum_TRACEROUTE_APP) {
            // Nexus: TEMPORARY BYPASS during active /nexus TRACE command
            if (trace_active) return false;

            bool isLocal = (mp.from == myNodeInfo.my_node_num || mp.to == myNodeInfo.my_node_num);
            uint8_t hopsUsed = (mp.hop_start >= mp.hop_limit) ? (mp.hop_start - mp.hop_limit) : 0;
            bool isDirect = (hopsUsed == 0);

            if (drop_traceroute) { fw_drop_traceroute++; return true; }
            if (drop_traceroute_local && isLocal) { fw_drop_traceroute_local++; return true; }
            if (drop_traceroute_remote && !isLocal) { fw_drop_traceroute_remote++; return true; }

            // 5b. Direct TraceRoute Filtering (0 hops used)
            if (isDirect) {
                if (drop_traceroute_direct) { fw_drop_traceroute_direct++; return true; }
                if (drop_traceroute_direct_local && isLocal) { fw_drop_traceroute_direct_local++; return true; }
                if (drop_traceroute_direct_remote && !isLocal) { fw_drop_traceroute_direct_remote++; return true; }
            }
        }
    }

    // Signature firewall (v39): scan payload bytes for any configured signature
    if (mp.which_payload_variant == meshtastic_MeshPacket_decoded_tag) {
        size_t plen = mp.decoded.payload.size;
        const uint8_t *pbuf = mp.decoded.payload.bytes;
        for (int s = 0; s < 8; s++) {
            if (nexus_signatures[s].len == 0) continue;
            if (plen < nexus_signatures[s].len) continue;
            // Naive substring match (O(n*m), m<=16, n<=240 => trivial)
            for (size_t off = 0; off <= plen - nexus_signatures[s].len; off++) {
                bool match = true;
                for (uint8_t i = 0; i < nexus_signatures[s].len; i++) {
                    if (pbuf[off + i] != nexus_signatures[s].bytes[i]) { match = false; break; }
                }
                if (match) {
                    fw_drop_signature++;
                    return true;
                }
            }
        }
    }

#ifdef NEXUS_PRIVATE_BUILD
    // Nexus-28006 Step C (2026-09-24) — private extension point.
    // Empty pass-through today; future private-only drop rules land here
    // without touching the public firewall body above.
    if (privateShouldDropPacket(mp)) {
        return true;
    }
#endif

    return false;
}

uint64_t NexusModule::getNexusDropHex() {
    uint64_t h = 0;
    if (drop_nodeinfo) h |= (1ULL << 0);
    if (drop_position) h |= (1ULL << 1);
    if (drop_telemetry) h |= (1ULL << 2);
    if (drop_neighbor) h |= (1ULL << 3);
    if (drop_traceroute) h |= (1ULL << 4);
    if (drop_traceroute_remote) h |= (1ULL << 5);
    if (drop_traceroute_local) h |= (1ULL << 6);
    if (drop_private) h |= (1ULL << 7);
    if (drop_hop6) h |= (1ULL << 8);
    if (drop_hop7) h |= (1ULL << 9);
    if (drop_req_nodeinfo) h |= (1ULL << 10);
    if (drop_req_telemetry) h |= (1ULL << 11);
    if (drop_req_position) h |= (1ULL << 12);
    if (drop_acks) h |= (1ULL << 13);
    if (drop_pki) h |= (1ULL << 14);
    if (drop_pki_remote) h |= (1ULL << 15);
    if (drop_pki_local) h |= (1ULL << 16);
    if (fav_bypass) h |= (1ULL << 17);
    if (drop_prehop) h |= (1ULL << 18);
    if (drop_store_forward) h |= (1ULL << 19);
    if (drop_admin) h |= (1ULL << 20);
    if (drop_text) h |= (1ULL << 21);
    if (drop_keyver) h |= (1ULL << 22);
    if (drop_alert) h |= (1ULL << 23);
    if (drop_waypoint) h |= (1ULL << 24);
    if (drop_traceroute_direct) h |= (1ULL << 25);
    if (drop_traceroute_direct_remote) h |= (1ULL << 26);
    if (drop_traceroute_direct_local) h |= (1ULL << 27);
    if (skip_skip_handle) h |= (1ULL << 28);
    if (drop_rl_text) h |= (1ULL << 29);
    if (drop_rl_trace) h |= (1ULL << 30);
    if (drop_rl_other) h |= (1ULL << 31);
    if (drop_tele_device) h |= (1ULL << 32);
    if (drop_tele_power) h |= (1ULL << 33);
    if (drop_tele_env) h |= (1ULL << 34);
    if (drop_tele_other) h |= (1ULL << 35);
    if (drop_no_mqtt) h |= (1ULL << 36);
    if (drop_pos_manual) h |= (1ULL << 37);
    if (drop_pos_gps) h |= (1ULL << 38);
    if (drop_pos_other) h |= (1ULL << 39);
    // Bits 40-41 reserved (was ping_emoji_mode, moved to dedicated vault field in v41)
    // Bits 42-45: rate-limit enable flags
    if (drop_rl_nodeinfo) h |= (1ULL << 42);
    if (drop_rl_position) h |= (1ULL << 43);
    if (drop_rl_telemetry) h |= (1ULL << 44);
    if (drop_rl_private) h |= (1ULL << 45);
    // v267: broadcast-specific request filters
    if (drop_br_nodeinfo) h |= (1ULL << 46);
    if (drop_br_telemetry) h |= (1ULL << 47);
    if (drop_br_position) h |= (1ULL << 48);
    return h;
}

uint32_t NexusModule::getNexusInvalidHex() {
    uint32_t h = 0;
    if (inv_null_island) h |= (1ULL << 0);
    if (inv_out_spain) h |= (1ULL << 1);
    if (inv_out_germany) h |= (1ULL << 2);
    if (inv_hw_unset) h |= (1ULL << 3);
    if (inv_id_mismatch) h |= (1ULL << 4);
    return h;
}

void NexusModule::setNexusInvalidHex(uint32_t h) {
    inv_null_island = (h & (1ULL << 0));
    inv_out_spain = (h & (1ULL << 1));
    inv_out_germany = (h & (1ULL << 2));
    inv_hw_unset = (h & (1ULL << 3));
    inv_id_mismatch = (h & (1ULL << 4));
}

uint64_t NexusModule::getNexusOutDropHex() {
    uint64_t h = 0;
    if (drop_out_nodeinfo) h |= (1ULL << 0);
    if (drop_out_position) h |= (1ULL << 1);
    if (drop_out_telemetry) h |= (1ULL << 2);
    if (drop_out_traceroute) h |= (1ULL << 3);
    if (drop_out_admin) h |= (1ULL << 4);
    if (drop_out_routing) h |= (1ULL << 5);
    return h;
}

void NexusModule::setNexusDropHex(uint64_t h) {
    drop_nodeinfo = (h & (1ULL << 0));
    drop_position = (h & (1ULL << 1));
    drop_telemetry = (h & (1ULL << 2));
    drop_neighbor = (h & (1ULL << 3));
    drop_traceroute = (h & (1ULL << 4));
    drop_traceroute_remote = (h & (1ULL << 5));
    drop_traceroute_local = (h & (1ULL << 6));
    drop_private = (h & (1ULL << 7));
    drop_hop6 = (h & (1ULL << 8));
    drop_hop7 = (h & (1ULL << 9));
    drop_req_nodeinfo = (h & (1ULL << 10));
    drop_req_telemetry = (h & (1ULL << 11));
    drop_req_position = (h & (1ULL << 12));
    drop_acks = (h & (1ULL << 13));
    drop_pki = (h & (1ULL << 14));
    drop_pki_remote = (h & (1ULL << 15));
    drop_pki_local = (h & (1ULL << 16));
    fav_bypass = (h & (1ULL << 17));
    drop_prehop = (h & (1ULL << 18));
    drop_store_forward = (h & (1ULL << 19));
    drop_admin = (h & (1ULL << 20));
    drop_text = (h & (1ULL << 21));
    drop_keyver = (h & (1ULL << 22));
    drop_alert = (h & (1ULL << 23));
    drop_waypoint = (h & (1ULL << 24));
    drop_traceroute_direct = (h & (1ULL << 25));
    drop_traceroute_direct_remote = (h & (1ULL << 26));
    drop_traceroute_direct_local = (h & (1ULL << 27));
    skip_skip_handle = (h & (1ULL << 28));
    drop_rl_text = (h & (1ULL << 29));
    drop_rl_trace = (h & (1ULL << 30));
    drop_rl_other = (h & (1ULL << 31));
    drop_tele_device = (h & (1ULL << 32));
    drop_tele_power = (h & (1ULL << 33));
    drop_tele_env = (h & (1ULL << 34));
    drop_tele_other = (h & (1ULL << 35));
    drop_no_mqtt = (h & (1ULL << 36));
    drop_pos_manual = (h & (1ULL << 37));
    drop_pos_gps = (h & (1ULL << 38));
    drop_pos_other = (h & (1ULL << 39));
    // Bits 40-41 reserved (was ping_emoji_mode, moved to dedicated vault field in v41)
    // Bits 42-45: rate-limit enable flags
    drop_rl_nodeinfo = (h & (1ULL << 42));
    drop_rl_position = (h & (1ULL << 43));
    drop_rl_telemetry = (h & (1ULL << 44));
    drop_rl_private = (h & (1ULL << 45));
    // v267: broadcast-specific request filters
    drop_br_nodeinfo  = (h & (1ULL << 46));
    drop_br_telemetry = (h & (1ULL << 47));
    drop_br_position  = (h & (1ULL << 48));
}

void NexusModule::setNexusOutDropHex(uint64_t h) {
    drop_out_nodeinfo = (h & (1ULL << 0));
    drop_out_position = (h & (1ULL << 1));
    drop_out_telemetry = (h & (1ULL << 2));
    drop_out_traceroute = (h & (1ULL << 3));
    drop_out_admin = (h & (1ULL << 4));
    drop_out_routing = (h & (1ULL << 5));
}

// Nexus v66: Feature toggles bitmask (persisted in NexusVault::features_mask).
// Plain pass-through accessors — the mask is opaque to the rest of the codebase,
// callers test individual bits via FEATURE_FORWARD_OPAQUE_TO_PHONE etc.
uint64_t NexusModule::getNexusFeaturesHex() {
    return features_mask;
}

void NexusModule::setNexusFeaturesHex(uint64_t h) {
    features_mask = h;
}

// Internal Nexus helpers nexus_strcasestr() and getGnssModelName() moved
// to NexusCommands.cpp as TU-local `static` (Aug 9, inline refactor).
// getGnssModelName pulls in gps/GPS.h enum values — kept in NexusCommands.cpp
// rather than the header to avoid dragging GPS.h into every TU that
// includes NexusModule.h.

void NexusModule::getRelayNames(char *buf, size_t bufSize, uint8_t suffix) {
    int foundCount = 0;
    buf[0] = '\0';
    for (size_t i = 0; i < nodeDB->getNumMeshNodes() && foundCount < 5; i++) {
        auto n = nodeDB->getMeshNodeByIndex(i);
        if (n && (uint8_t)(n->num & 0xFF) == suffix) {
            if (n->short_name[0] != '\0') {
                if (foundCount > 0) strncat(buf, ",", bufSize - strlen(buf) - 1);
                strncat(buf, n->short_name, bufSize - strlen(buf) - 1);
                foundCount++;
            }
        }
    }
}

void NexusModule::applyBS(meshtastic_Telemetry *m) {
    // Internal telemetry hook. Public builds: empty body so external callers
    // (EnvironmentTelemetry.cpp) compile and simply do nothing. Private builds:
    // delegates to the companion implementation.
#ifdef NEXUS_PRIVATE_BUILD
    if (B_S_active)
        ::applyBS(m, B_S_offset);
#endif
}


// Nexus: Targeted prefix roots (file-scope single source of truth). Shared by:
//   - handleReceived() cartesian loop below (mesh-radio dispatch)
//   - handleLocal() isLocalOnly/isBroadcast/isTargetedPrefix checks
//   - matchTargetedCmd() targeted dispatch helper
// Adding a new root here propagates to all three call sites. No other duplicates.
static const char* const NEXUS_PREFIX_ROOTS[] = {"/nexus", "/jt", "/jent", "/jentastic"};
static const size_t NEXUS_PREFIX_ROOTS_COUNT = sizeof(NEXUS_PREFIX_ROOTS) / sizeof(NEXUS_PREFIX_ROOTS[0]);

// Nexus: auto Zero-Hop masks - traffic-driven (2026-10-07 code review):
// called per DIRECTLY-HEARD packet (zero hops away, 2nd review) with the
// SENDER's NodeNum - one NodeDB lookup, no full sweep "at the start". Set-only:
// a demoted node's stale bit is removed only via ZH CLEAR ROUTER/CLIENT -
// never auto-cleared, because the masks are keyed on the last byte and an
// auto-clear keyed on one node could erase a colliding node's bit.
void NexusModule::populateAutoZeroHopMasks(uint32_t nodeNum)
{
    meshtastic_NodeInfoLite *node = nodeDB->getMeshNode(nodeNum);
    if (!node)
        return;
    uint8_t id = nodeDB->getLastByteOfNodeNum(nodeNum);
    if (node->role == meshtastic_Config_DeviceConfig_Role_ROUTER ||
        node->role == meshtastic_Config_DeviceConfig_Role_ROUTER_LATE) {
        setZeroHopAutoRouterNode(id, true);
    } else if (node->role == meshtastic_Config_DeviceConfig_Role_CLIENT_BASE) {
        setZeroHopAutoClientNode(id, true);
    }
}

// ZH list-view target -> mask (2026-10-07): lets LIST/SHORT/PAGED show
// each of the three zero-hop lists independently. MANUAL is the default so a
// bare `ZH LIST` keeps the historical meaning (the hand-built mask).
const uint8_t *NexusModule::zeroHopListMask(const char *target, bool *valid, const char **label)
{
    *valid = true;
    if (!target || !*target || strcasecmp(target, "MANUAL") == 0 ||
        strcasecmp(target, "M") == 0 || strcasecmp(target, "MAN") == 0) {
        *label = "manual";
        return zero_hop_mask;
    }
    if (strcasecmp(target, "ROUTER") == 0 || strcasecmp(target, "R") == 0 ||
        strcasecmp(target, "RT") == 0) {
        *label = "router";
        return zero_hop_auto_router_mask;
    }
    if (strcasecmp(target, "CLIENT") == 0 || strcasecmp(target, "C") == 0 ||
        strcasecmp(target, "CB") == 0 || strcasecmp(target, "CLIENT_BASE") == 0) {
        *label = "client";
        return zero_hop_auto_client_mask;
    }
    if (strcasecmp(target, "ALL") == 0 || strcasecmp(target, "A") == 0 ||
        strcasecmp(target, "EFFECTIVE") == 0) {
        *label = "all";
        return nullptr; // nullptr = effective set (manual + enabled autos)
    }
    *valid = false;
    *label = "?";
    return nullptr;
}

// Only DIRECT nodes join the auto masks (2026-10-07 2nd review): heard
// with zero hops away AND over an RF transport. getHopsAway() returns -1 when
// the hop count can't be determined -> skipped (never guessed). Transport must
// be a LoRa radio, or unstamped (0 - unstamped frames are deliberately
// OTA-equivalent); MQTT/UDP/API carriers are stamped at their
// own ingresses and excluded - their origin's zero hops don't prove RF
// neighbourhood with us.
static bool heardDirectZeroHop(const meshtastic_MeshPacket &mp)
{
    if (getHopsAway(mp) != 0)
        return false;
    switch (mp.transport_mechanism) {
    case meshtastic_MeshPacket_TransportMechanism_TRANSPORT_INTERNAL: // unstamped == OTA-equivalent
    case meshtastic_MeshPacket_TransportMechanism_TRANSPORT_LORA:
    case meshtastic_MeshPacket_TransportMechanism_TRANSPORT_LORA_ALT1:
    case meshtastic_MeshPacket_TransportMechanism_TRANSPORT_LORA_ALT2:
    case meshtastic_MeshPacket_TransportMechanism_TRANSPORT_LORA_ALT3:
        return true;
    default:
        return false;
    }
}

ProcessMessage NexusModule::handleReceived(const meshtastic_MeshPacket &mp) {
    // Nexus: auto Zero-Hop masks (2026-10-07) - traffic-driven AND direct
    // only: on a packet heard with zero hops from a LoRa neighbour, look the
    // SENDER up in NodeDB (single lookup, no sweep) and set its role bit in the
    // router/client auto-mask. Not channel-gated - roles are mesh-wide. Sender
    // absent from NodeDB yet (NODEINFO not ingested) is skipped; the next
    // direct packet from it catches up.
    if (mp.from != 0 && mp.from != myNodeInfo.my_node_num && heardDirectZeroHop(mp)) {
        populateAutoZeroHopMasks(mp.from);
    }

    // Nexus: AUTO_FAV_NEXUS / AUTO_FAV_TRUSTED — Nexus-28004, 2026-08-30.
    // Run before any portnum guard so this fires for every decoded packet
    // wantPacket() lets through (TEXT_MESSAGE, ADMIN, POSITION, NODEINFO,
    // TELEMETRY, TRACEROUTE, plus any new portnums we add later). No flash
    // save — the operator must /nexus SAVE to persist the new favourites.
    // Self-packets are skipped so we don't fav our own node.
    if (mp.which_payload_variant == meshtastic_MeshPacket_decoded_tag &&
        mp.decoded.portnum != 0 &&
        mp.from != 0 && mp.from != myNodeInfo.my_node_num &&
        this->isNexusChannel(mp.channel)) {
        bool alreadyFav = false;
        if (auto n = nodeDB->getMeshNode(mp.from)) {
            alreadyFav = nexus_is_favorite(n);
        }
        if (!alreadyFav && securityAutoFavNexus()) {
            auto n = nodeDB->getOrCreateMeshNode(mp.from);
            if (n) {
                // Set the favourite bit directly via the primitive (the
                // nexus_set_favorite() inline lives in NexusCommands.cpp's TU
                // and isn't visible here). Same call chain the inline uses.
                nodeInfoLiteSetBit(n, NODEINFO_BITFIELD_IS_FAVORITE_MASK, true);
                LOG_INFO("Nexus: AUTO_FAV_NEXUS fav'd !%08x on Nexus channel", (unsigned)mp.from);
            }
        }
        if (!alreadyFav && securityAutoFavTrusted()) {
            char matchBuf[32] = {0};
            if (verifyTrustedSignature(&mp, matchBuf, sizeof(matchBuf))) {
                auto n = nodeDB->getOrCreateMeshNode(mp.from);
                if (n) {
                    nodeInfoLiteSetBit(n, NODEINFO_BITFIELD_IS_FAVORITE_MASK, true);
                    LOG_INFO("Nexus: AUTO_FAV_TRUSTED fav'd !%08x (sig=%s)", (unsigned)mp.from, matchBuf);
                }
            }
        }
    }

    // Nexus: AUTO_FAV_EXTRA (2026-10-07) — like AUTO_FAV_NEXUS but for ONE
    // operator-chosen channel (security bitmask bit 6 = enable, bits 7-9 =
    // channel 0-7). Deliberately NOT gated on isNexusChannel(): the whole point
    // is faving a channel that is NOT one of the compile-time/hardcoded Nexus
    // trusted channels. Same no-flash-save policy — operator runs /nexus SAVE.
    // Self-packets skipped, same as AUTO_FAV_NEXUS.
    if (mp.which_payload_variant == meshtastic_MeshPacket_decoded_tag &&
        mp.decoded.portnum != 0 &&
        mp.from != 0 && mp.from != myNodeInfo.my_node_num &&
        this->securityAutoFavExtra() && mp.channel == this->securityAutoFavExtraChannel()) {
        bool alreadyFav = false;
        if (auto n = nodeDB->getMeshNode(mp.from)) {
            alreadyFav = nexus_is_favorite(n);
        }
        if (!alreadyFav) {
            auto n = nodeDB->getOrCreateMeshNode(mp.from);
            if (n) {
                nodeInfoLiteSetBit(n, NODEINFO_BITFIELD_IS_FAVORITE_MASK, true);
                LOG_INFO("Nexus: AUTO_FAV_EXTRA fav'd !%08x on channel %u",
                         (unsigned)mp.from, (unsigned)this->securityAutoFavExtraChannel());
            }
        }
    }

    if (mp.which_payload_variant == meshtastic_MeshPacket_decoded_tag &&
        mp.decoded.portnum == meshtastic_PortNum_TEXT_MESSAGE_APP &&
        mp.decoded.payload.size > 2) {

        // Use a persistent buffer to ensure we have a null-terminated string for case-insensitive comparison
        // Moved from stack to class member to reduce loopTask stack pressure
        char *cmdBuf = this->cmdBuf;
        // Nexus: Full memset (not just cmdBuf[0]) so residual bytes from previous calls cannot
        // bleed into strcasecmp/strlen reads and cause spurious 'Unknown command' mismatches.
        memset(cmdBuf, 0, sizeof(this->cmdBuf));
        size_t copyLen = std::min<size_t>(mp.decoded.payload.size, sizeof(this->cmdBuf) - 1);
        memcpy(cmdBuf, mp.decoded.payload.bytes, copyLen);

        // Nexus: Emoji Trigger (🐱 or 🙂 or 🤖 on Nexus Channel)
        if (this->isNexusChannel(mp.channel)) {
            bool isCat = (strncmp(cmdBuf, "\xF0\x9F\x90\xB1", 4) == 0);
            bool isSmile = (strncmp(cmdBuf, "\xF0\x9F\x99\x82", 4) == 0);
            bool isBot = (strncmp(cmdBuf, "\xF0\x9F\xA4\x96", 4) == 0);

            if (isCat || isSmile) {
                LOG_INFO("Nexus: Emoji trigger detected (🐱/🙂). Triggering RSSI.");
                handleNexusCommand(mp, "RSSI", false, mp.id);
            } else if (isBot) {
                LOG_INFO("Nexus: Emoji trigger detected (🤖). Triggering Hops Reaction.");
                uint32_t hopsUsed = (mp.hop_start >= mp.hop_limit) ? (mp.hop_start - mp.hop_limit) : 0;
                char hopEmoji[8] = {0};
                hopsToEmoji(hopEmoji, sizeof(hopEmoji), hopsUsed);

                bool isDM = (mp.to == myNodeInfo.my_node_num);
                size_t rLen = strlen(hopEmoji);

                // 1. Immediate Local Copy (No delay). asReaction=true marks
                // the packet as a Meshtastic emoji reaction (🤖 trigger only).
                sendToPhone(0xFFFFFFFF, mp.channel, mp.id, hopEmoji, rLen, true);

                // 2. Queued Mesh Response (Jittered)
                pendingReply.active = true;
                pendingReply.to = isDM ? mp.from : 0xFFFFFFFF;
                pendingReply.channel = mp.channel;
                pendingReply.reply_id = mp.id;
                pendingReply.want_ack = true;
                pendingReply.is_reaction = true; // Use reaction flag in PendingReply
                memcpy(pendingReply.message, hopEmoji, rLen);
                pendingReply.message[rLen] = '\0';

                uint32_t jitter = broadcastReplyJitterMs();
                pendingReply.send_at = Time::getMillis() + jitter;
                setIntervalFromNow(50);
            }
        }

        if (public_ping) {
            if (strcasecmp(cmdBuf, "/ping") == 0 || strcasecmp(cmdBuf, "/test") == 0 || strcasecmp(cmdBuf, "/prueba") == 0) {
                // (SELF-ECHO GUARD 2026-08-02) Same rationale as the /nexus RSSI guard:
                // a loopback / MQTT-echo / retransmission has uninitialized rx_rssi/SNR,
                // so replying produces zeroed telemetry. mp.from == 0 covers anonymous
                // phone-originated echoes; mp.from == our node num covers self-rebroadcast.
                // isFromUs() ORs both. CONTINUE (not STOP) so other modules can still see
                // the packet (phone mirror, etc.).
                if (isFromUs(&mp)) return ProcessMessage::CONTINUE;

                // (DM GATE 2026-08-23) PPING defaults to broadcast-only. When the /ping arrives
                // as a DM (mp.to == our node num), reply only if ping_allow_dm is enabled.
                // DMs bypass the ping_channels[mp.channel] gate below — a direct-to-me packet
                // is not a channel broadcast, and the operator who flipped ALLOWDM ON clearly
                // wants to receive DMs on the active channel regardless of the channel flag.
                bool isPingDM = (mp.to == myNodeInfo.my_node_num);
                if (isPingDM && !ping_allow_dm) {
                    LOG_INFO("Nexus: Public Ping DM ignored (ping_allow_dm disabled)");
                    return ProcessMessage::CONTINUE;
                }

                uint32_t hopsUsed = (mp.hop_start >= mp.hop_limit) ? (mp.hop_start - mp.hop_limit) : 0;

                // 1. Direct Filter: Only process if NOT a direct link, OR if direct ignore is disabled
                if (hopsUsed == 0 && ping_ignore_direct) {
                    LOG_INFO("Nexus: Public Ping direct ignored (ping_ignore_direct active)");
                }
                // 1b. Remote Filter: Only process if a direct link, OR if remote ignore is disabled
                else if (hopsUsed > 0 && ping_ignore_remote) {
                    LOG_INFO("Nexus: Public Ping remote ignored (ping_ignore_remote active)");
                }
                else {
                    // 2. Channel check: Only reply if PPING is enabled for this specific channel.
                    //    DMs (isPingDM) bypass this gate — a direct-to-me packet is not a
                    //    channel broadcast, and the operator who enabled ALLOWDM clearly wants
                    //    DMs regardless of the per-channel flag.
                    if ((isPingDM || (mp.channel < MAX_NUM_CHANNELS && ping_channels[mp.channel]))) {
                        LOG_INFO("Nexus: Public Ping triggered by !%08x on Ch %d (DM=%d)", mp.from, mp.channel, isPingDM ? 1 : 0);

                        // Emoji mode: 0=text, 1=emoji if indirect, 2=always emoji
                        bool useEmoji = (ping_emoji_mode == 2) ||
                                        (ping_emoji_mode == 1 && hopsUsed > 0);

                        if (useEmoji) {
                            // Reuse the 🤖 emoji path: send keycap number reaction
                            char hopEmoji[8] = {0};
                            hopsToEmoji(hopEmoji, sizeof(hopEmoji), hopsUsed);
                            bool isDM_e = (mp.to == myNodeInfo.my_node_num);
                            size_t rLen_e = strlen(hopEmoji);
                            sendToPhone(0xFFFFFFFF, mp.channel, mp.id, hopEmoji, rLen_e, true);
                            pendingReply.active = true;
                            pendingReply.to = isDM_e ? mp.from : 0xFFFFFFFF;
                            pendingReply.channel = mp.channel;
                            pendingReply.reply_id = mp.id;
                            pendingReply.want_ack = true;
                            pendingReply.is_reaction = true;
                            memcpy(pendingReply.message, hopEmoji, rLen_e);
                            pendingReply.message[rLen_e] = '\0';
                            uint32_t jitter = broadcastReplyJitterMs();
                            pendingReply.send_at = Time::getMillis() + jitter;
                            setIntervalFromNow(50);
                            return ProcessMessage::CONTINUE;
                        }

                        // Static buffer (BSS, not stack).
                        static char rMsg[200];
                        memset(rMsg, 0, sizeof(rMsg));

                int l;
                if (custom_pping_active && pping_header[0] != '\0') {
                    l = snprintf(rMsg, sizeof(rMsg), "%s\n", pping_header);
                } else {
                    l = snprintf(rMsg, sizeof(rMsg), "🤖Derpy Ping🤖\n");
                }
                l += snprintf(rMsg + l, sizeof(rMsg) - l, "📡 Hops:%u/%u ", hopsUsed, mp.hop_start);

                if (hopsUsed == 0) {
                    l += snprintf(rMsg + l, sizeof(rMsg) - l, "(Direct)\n");
                } else {
                    static char relayNames[64];
                    memset(relayNames, 0, sizeof(relayNames));
                    uint8_t targetSuffix = (uint8_t)(mp.relay_node & 0xFF);
                    getRelayNames(relayNames, sizeof(relayNames), targetSuffix);

                    if (relayNames[0] != '\0') {
                        l += snprintf(rMsg + l, sizeof(rMsg) - l, "Relay:%s (*%02x)\n", relayNames, targetSuffix);
                    } else {
                        l += snprintf(rMsg + l, sizeof(rMsg) - l, "Relay:*%02x\n", targetSuffix);
                    }
                    memset(relayNames, 0, sizeof(relayNames));
                }

                if (ping_rssi || ping_snr) {
                    l += snprintf(rMsg + l, sizeof(rMsg) - l, "📶 ");
                    if (ping_rssi) l += snprintf(rMsg + l, sizeof(rMsg) - l, "RSSI:%d ", (int)mp.rx_rssi);
                    if (ping_snr) l += snprintf(rMsg + l, sizeof(rMsg) - l, "SNR:%.1f", mp.rx_snr);
                    l += snprintf(rMsg + l, sizeof(rMsg) - l, "\n");
                }

                if (ping_freqerr) {
                    // Nexus port (Nexus-28001, 2026-08-16): rx_frequency_error + rx_coding_rate
                    // both restored. PPING FREQERR now reports the live frequency error in Hz and
                    // the recovered coding rate (raw 1-7 -> display 5/6/7/8 short, 9/10/11 long).
                    l += snprintf(rMsg + l, sizeof(rMsg) - l, "🔬 Δf: %.0fHz CR:%d\n",
                                  mp.rx_frequency_error, nexusCRDisplay(mp.rx_coding_rate));
                }

                if (ping_bitmask) {
                    if (l < 0 || (size_t)l >= sizeof(rMsg)) l = sizeof(rMsg) - 1;
                    bool om = (mp.decoded.has_bitfield && (mp.decoded.bitfield & BITFIELD_OK_TO_MQTT_MASK));
                    bool mqtt = mp.via_mqtt;

                    l += snprintf(rMsg + l, sizeof(rMsg) - l, "🌐 OM:%d%s MQTT:%d%s UDP:%d",
                                 om ? 1 : 0, !om ? "⚠️" : "",
                                 mqtt ? 1 : 0, mqtt ? "⚠️" : "",
                                 (mp.transport_mechanism == meshtastic_MeshPacket_TransportMechanism_TRANSPORT_MULTICAST_UDP) ? 1 : 0);

                    // (SIGNATURE FLAG 2026-08-23) Append S:<emoji> to the flags line.
                    // Three states, intentionally a different palette than the auto-prefix
                    // convention in NexusCommands.cpp:8134 (which uses ✅/🚫/📄):
                    //   🔴 — no signature field on the wire (decoded.xeddsa_signature.size == 0)
                    //   🟠 — signature present but xeddsa_verify() failed (size != 0, !xeddsa_signed)
                    //   🟢 — signature present AND verified by Router::checkXeddsaReceivePolicy
                    // Converging the two is a separate follow-up — for now PPING uses traffic-light
                    // colors and the auto-prefix keeps its checkmark/ban/document set.
                    const char *sigEmoji;
                    if (mp.decoded.xeddsa_signature.size == 0) {
                        sigEmoji = "🔴"; // F0 9F 94 B4 — not signed
                    } else if (!mp.xeddsa_signed) {
                        sigEmoji = "🟠"; // F0 9F 9F A0 — signed but unverified
                    } else {
                        sigEmoji = "🟢"; // F0 9F 9F A2 — signed & verified
                    }
                    l += snprintf(rMsg + l, sizeof(rMsg) - l, " S:%s", sigEmoji);
                }

                // Send the reply back (Multi-routing: immediate local + deferred mesh)
                bool isDM = (mp.to == myNodeInfo.my_node_num);
                size_t rLen = strlen(rMsg);
                if (rLen > NEXUS_MAX_REPLY_BODY) rLen = NEXUS_MAX_REPLY_BODY;

                // 1. Immediate Local Copy to phone UI
                sendToPhone(0xFFFFFFFF, mp.channel, mp.id, rMsg, rLen);
                LOG_INFO("Nexus: Local ping reply copy sent.");

                // 2. Queued Mesh Response
                pendingReply.active = true;
                pendingReply.to = isDM ? mp.from : 0xFFFFFFFF;
                pendingReply.channel = mp.channel;
                pendingReply.reply_id = mp.id;
                pendingReply.want_ack = true;
                memcpy(pendingReply.message, rMsg, rLen);
                pendingReply.message[rLen] = '\0';

                if (!isDM) {
                    uint32_t jitter = broadcastReplyJitterMs();
                    pendingReply.send_at = Time::getMillis() + jitter;
                    LOG_INFO("Nexus: Queuing broadcast ping with %ums jitter", jitter);
                } else {
                    pendingReply.send_at = Time::getMillis() + 50; // Minimal yield
                    LOG_INFO("Nexus: Queuing DM ping (50ms yield)");
                }
                setIntervalFromNow(50);

                // If it was a DM, we can STOP it here. If channel, CONTINUE so others can reply too.
                memset(rMsg, 0, sizeof(rMsg));
                return isDM ? ProcessMessage::STOP : ProcessMessage::CONTINUE;
                    }
                }
            }
        }

        // 2. Multi-prefix support: /nexus, /jt, /jent, /jentastic
        // Nexus v267: NEXUS_PREFIX_ROOTS (file scope, defined above this function) is the
        // single source of truth for the 4 valid prefix roots. The handleReceived cartesian
        // loop, handleLocal's isTargetedPrefix/isLocalOnly/isBroadcast checks, and
        // matchTargetedCmd() all read from this same table. Adding a root here propagates
        // to all three call sites automatically.
        int type = -1; // 0:local, 1:broadcast, 2:device, 3:node, 4:mac, 5:group
        size_t prefLen = 0;
        const char* suffixes[] = {"-local", "", "-device", "-node", "-mac", "-group"};

        for (size_t p=0; p < NEXUS_PREFIX_ROOTS_COUNT; p++) {
            for (int s=0; s<6; s++) {
                char full[32];
                snprintf(full, sizeof(full), "%s%s", NEXUS_PREFIX_ROOTS[p], suffixes[s]);
                size_t flen = strlen(full);
                if (strncasecmp(cmdBuf, full, flen) == 0) {
                    if (cmdBuf[flen] == ' ' || cmdBuf[flen] == '\0') {
                        prefLen = flen;
                        type = s;
                        break;
                    }
                }
            }
            if (type != -1) break;
        }

        if (type != -1) {
            bool isLocalOnly = (type == 0);
            bool isBroadcast = (type == 1);
            bool isTargetedID = (type == 2);
            bool isTargetedName = (type == 3);
            bool isTargetedMAC = (type == 4);
            bool isTargetedGroup = (type == 5);

            // 3. Context Check: 2026-08-17 (Nexus-28001) — /nexus command authorization
            // is now CHANNEL-ONLY by default. 2026-08-24 (Nexus-28003):
            // the gate is now controlled by nexus_security_bitmask (vault v65 slot):
            //   bit 0  NEXUS_SECURITY_REQUIRE_SIG  — sig required on Nexus-ch cmds
            //   bit 1  NEXUS_SECURITY_ALLOW_DM     — DMs accepted as an auth path
            //   bit 2  NEXUS_SECURITY_SILENT_LOG  — silent log+drop on reject (else reply)
            // All three bits default OFF. With everything off the gate is the
            // Nexus-28001 default: only Nexus-channel commands run, no sig required.
            //
            // "Internal" packets bypass ALL of these rules. nexusIsInternal()
            // (see shim block above) combines isFromUs() (anonymous local origin
            // OR self-rebroadcast) with a transport_mechanism != TRANSPORT_LORA
            // guard so that a packet which loops back over LoRa — even with
            // from==0 or from==us — is NOT treated as internal.
            // Rule (2026-09-02): don't apply the rule to packets that did not originate
            // locally; the on-wire transport_mechanism is the canonical witness.
            bool isDM = (mp.to == myNodeInfo.my_node_num);
            meshtastic_Channel *ch = &channels.getByIndex(mp.channel);
            bool isNexusChannel = this->isNexusChannel(mp.channel);
            bool isInternal = nexusIsInternal(mp);

            // Nexus-28005 (2026-09-02): Targeted-command suppression.
            // If the packet uses -device / -node / -mac / -group addressing but
            // the addressee is NOT us (matchTargetedCmd returns nullptr), we are
            // overhearing a packet meant for another node. REQ_SIG / replay
            // / authorization still apply — we don't run the command — but we
            // suppress the reject-reply unconditionally (same as SILENT_LOG
            // forced ON) so we don't leak our security state to the sender.
            // SILENT_LOG continues to suppress for packets addressed to us.
            // Computed eagerly here so both reject sites (replay + auth gate)
            // can share it without calling matchTargetedCmd() twice.
            bool isTargetedCommand = (isTargetedID || isTargetedName || isTargetedMAC || isTargetedGroup);
            bool addressedElsewhere = isTargetedCommand && (matchTargetedCmd(cmdBuf, nullptr) == nullptr);

            LOG_INFO("Nexus: Command context check - isDM: %d, Channel: '%s', isNexusChannel: %d, isInternal: %d", isDM, ch ? ch->settings.name : "NULL", isNexusChannel, isInternal);

            // Replay protection (Nexus-28004, 2026-08-31): a duplicate
            // MeshPacket.id from a non-internal sender is treated as a replay
            // and rejected before any dispatch. Internal packets (isFromUs)
            // bypass, matching the REQ_SIG policy. No TTL — ids stay in the
            // 256-entry ring until evicted by a newer one. The check records
            // the id regardless of accept/reject (so a malicious duplicate
            // doesn't unlock the original slot).
            //
            // Bypass_RP toggle (Nexus-28005, 2026-09-06): opt-out for
            // operators who want RP off. Per spec the bypass skips the check
            // AND skips the fill — when bypassed, checkReplay() is not called
            // at all, so the ring stays exactly as it was at bypass time and
            // picking back up resumes from there.
            if (!isInternal && !securityBypassRP()) {
                if (checkReplay(mp.id)) {
                    LOG_WARN("Nexus: SECURITY REJECTED /nexus from !%08x — replay detected (packet id %u)", (unsigned)mp.from, (unsigned)mp.id);
                    // Suppress the mesh reply if the packet was targeted to a
                    // different node — we never authorize it, but we also don't
                    // leak that we detected the replay to the sender. SILENT_LOG
                    // continues to suppress for packets addressed to us.
                    if (!securitySilentLog() && !addressedElsewhere) {
                        char repReply[NEXUS_MAX_REPLY];
                        snprintf(repReply, sizeof(repReply), "JT SECURITY: rejected — replay detected (packet id %u)", (unsigned)mp.id);
                        uint32_t to = (mp.to == myNodeInfo.my_node_num) ? mp.from : 0xFFFFFFFF;
                        queueMeshReply(to, mp.channel, mp.id, repReply, strlen(repReply), false, true);
                    }
                    return ProcessMessage::CONTINUE;
                }
            }

            // Authorization gate. Three outcomes:
            //   (a) Internal packet       → always authorized (own serial/local cmd)
            //   (b) Nexus-channel packet  → authorized if REQ_SIG is OFF, OR
            //                              verifyTrustedSignature() hits a
            //                              trusted key (admin/secondary/master)
            //   (c) DM packet             → authorized ONLY when ALLOW_DM is ON,
            //                              and pubkey isAuthorizedAdmin()
            //
            // On reject, behavior depends on SILENT_LOG:
            //   ON  → LOG_WARN + continue to the bottom (no reply)
            //   OFF → LOG_WARN + send 'JT: SECURITY: ...' reply + continue
            // Both paths drop through to the end of handleReceived so the
            // packet is still returned as CONTINUE for upstream visibility.
            bool authorized = false;
            const char *rejectReason = nullptr;

            if (isInternal) {
                authorized = true;
            } else if (isNexusChannel) {
                if (!securityRequiresSignature()) {
                    authorized = true;  // pre-Nexus-28003 default: open Nexus channel
                } else {
                    char sigLabel[8] = {0};
                    bool sigOk = verifyTrustedSignature(&mp, sigLabel, sizeof(sigLabel));
                    LOG_INFO("Nexus: SECURITY REQ_SIG check on Nexus-ch cmd — sig=%s", sigLabel);
                    if (sigOk) {
                        authorized = true;
                    } else {
                        rejectReason = "signature required (REQ_SIG)";
                    }
                }
            } else if (isDM && securityAllowsDM()) {
                // DMs accepted. Pubkey must be in the Nexus trust list.
                if (isAuthorizedAdmin(mp.public_key.bytes)) {
                    authorized = true;
                } else {
                    rejectReason = "DM not authorized (ALLOW_DM, pubkey not in trust list)";
                }
            } else if (isDM) {
                rejectReason = "DM not authorized (ALLOW_DM=OFF)";
            }

                if (authorized) {
                    // Nexus: Block /nexus-local if received over the mesh (Security Violation)
                    if (isLocalOnly) {
                        if (isInternal)
                            return handleLocal(mp);
                        LOG_WARN("Nexus: Security Violation - /nexus-local received over RF from !%08x", mp.from);
                        if (!securitySilentLog()) {
                            const char *secReply = "JT SECURITY: -local prefix is for local interface only. Use -node over mesh.";
                            uint32_t to = (mp.to == myNodeInfo.my_node_num) ? mp.from : 0xFFFFFFFF;
                            queueMeshReply(to, mp.channel, mp.id, secReply, strlen(secReply), false, true);
                        }
                        return ProcessMessage::STOP;
                    }

                    // Comparison for targeted commands
                    if (isTargetedID || isTargetedName || isTargetedMAC || isTargetedGroup) {
                        size_t kind = 0;
                        const char *cmdStart = matchTargetedCmd(cmdBuf, &kind);
                        if (cmdStart) {
                            // ID/Name/MAC are DMs (isBroadcast=false), group is broadcast.
                            bool isGroup = (kind == TARGET_GROUP);
                            LOG_INFO("Nexus: Targeted command for us (kind=%u): %s", (unsigned)kind, cmdStart);
                            handleNexusCommand(mp, cmdStart, isGroup, mp.id, false);
                        }
                        return ProcessMessage::CONTINUE;
                    }
                    else if (isBroadcast) {
                        const char *cmdStart = cmdBuf + prefLen;
                        while(*cmdStart == ' ') cmdStart++;
                        handleNexusCommand(mp, cmdStart, true, mp.id, false);
                        return ProcessMessage::CONTINUE;
                    }
                } else {
                    LOG_WARN("Nexus: SECURITY REJECTED /nexus from !%08x — %s", (unsigned)mp.from, rejectReason ? rejectReason : "not authorized");

                    // Reply with the security reason UNLESS the operator
                    // flipped SILENT_LOG ON. Replying helps debugging but
                    // also lets an attacker probe the bitmask state.
                    // Targeted commands addressed to a different node
                    // (addressedElsewhere) always stay silent — Nexus-28005.
                    if (!securitySilentLog() && !addressedElsewhere && rejectReason) {
                        char secReply[NEXUS_MAX_REPLY];
                        snprintf(secReply, sizeof(secReply), "JT SECURITY: rejected — %s", rejectReason);
                        // Reply on the same channel the command arrived on,
                        // back to the sender (or broadcast if no specific sender).
                        uint32_t to = (mp.to == myNodeInfo.my_node_num) ? mp.from : 0xFFFFFFFF;
                        queueMeshReply(to, mp.channel, mp.id, secReply, strlen(secReply), false, true);
                    }
                }
            }
        }

        // Nexus: DM Forwarding Logic (v10)
        // If enabled, forward non-CLI DMs to the Nexus channel with full SIGINT metadata
                if (dm_forward && mp.to == myNodeInfo.my_node_num) {
            LOG_INFO("Nexus: DM Forwarding triggered for DM from !%08x", mp.from);

            // 1. Prepare Tactical Report
            uint32_t hopsUsed = (mp.hop_start >= mp.hop_limit) ? (mp.hop_start - mp.hop_limit) : 0;
            char relayId[12] = "None";
            if (hopsUsed > 0) snprintf(relayId, sizeof(relayId), "*%02x", (uint8_t)(mp.relay_node & 0xFF));

            char report[NEXUS_MAX_REPLY];
            int rLen = snprintf(report, sizeof(report),
                                "JT FWDM [!%08x]:\nMsg:%.100s\nHops:%u/%u RY:%s\nSNR:%.2f RSSI:%d",
                                (unsigned int)mp.from, cmdBuf, hopsUsed, mp.hop_start, relayId, mp.rx_snr, (int)mp.rx_rssi);

            if (rLen < 0 || (size_t)rLen >= sizeof(report)) rLen = sizeof(report) - 1;

            // 2. Find Tactical Channel
            int targetCh = -1;
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
            for (uint8_t i = 0; i < MAX_NUM_CHANNELS; i++) {
                meshtastic_Channel *c = &channels.getByIndex(i);
                if (c->role != meshtastic_Channel_Role_DISABLED) {
                    if (strcasecmp(c->settings.name, trusted1) == 0 || strcasecmp(c->settings.name, trusted2) == 0) {
                        targetCh = i;
                        break;
                    }
                }
            }

            // 3. Send Report to Mesh (if channel found)
            if (targetCh != -1) {
                // If message is too long, use paging
                if (rLen > (int)NEXUS_MAX_REPLY_BODY - 10) {
                    startPaging(0xFFFFFFFF, targetCh, 0, "Forwarded DM");
                    appendPaging(report);
                } else {
                    meshtastic_MeshPacket *tp = (router) ? router->allocForSending() : nullptr;
                    if (tp) {
                        tp->decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;
                        tp->to = 0xFFFFFFFF; // Broadcast to channel
                        tp->channel = targetCh;
                        memcpy(tp->decoded.payload.bytes, report, rLen);
                        tp->decoded.payload.size = rLen;
                        if (service) service->sendToMesh(tp);
                    }
                }
            }
        }
    return ProcessMessage::CONTINUE;
}
ProcessMessage NexusModule::handleLocal(const meshtastic_MeshPacket &mp) {
    if (!nexusIsInternal(mp) || mp.which_payload_variant != meshtastic_MeshPacket_decoded_tag ||
        mp.decoded.portnum != meshtastic_PortNum_TEXT_MESSAGE_APP)
        return ProcessMessage::CONTINUE;
    memset(cmdBuf, 0, sizeof(cmdBuf));
    memcpy(cmdBuf, mp.decoded.payload.bytes, std::min<size_t>(mp.decoded.payload.size, sizeof(cmdBuf) - 1));
    for (size_t r = 0; r < NEXUS_PREFIX_ROOTS_COUNT; ++r) {
        size_t rootLength = strlen(NEXUS_PREFIX_ROOTS[r]);
        if (strncasecmp(cmdBuf, NEXUS_PREFIX_ROOTS[r], rootLength) != 0)
            continue;
        const char *tail = cmdBuf + rootLength;
        if (strncasecmp(tail, "-local", 6) != 0 || (tail[6] != ' ' && tail[6] != '\0'))
            continue;
        tail += 6;
        while (*tail == ' ')
            ++tail;
        meshtastic_MeshPacket local = mp;
        if (local.from == 0)
            local.from = nodeDB->getNodeNum();
        if (local.to == 0)
            local.to = NODENUM_BROADCAST;
        if (router && service) {
            meshtastic_QueueStatus qs = router->getQueueStatus();
            service->sendQueueStatusToPhone(qs, ERRNO_OK, mp.id);
        }
        // isBroadcast stays false for local ingress: -local never arrives over RF, so the
        // broadcast NAME/OWNER safety gate ("prevent mass-wiping a network") and the 5s
        // broadcast rate limit must not fire on local commands. Routing/paging read mp.to
        // directly, so nothing is lost by not passing the broadcast flag here.
        handleNexusCommand(local, tail, false, mp.id, true);
        return ProcessMessage::STOP;
    }
    return ProcessMessage::CONTINUE;
}


bool NexusModule::isNexusChannel(uint32_t channelIndex)
{
    if (channelIndex >= MAX_NUM_CHANNELS)
        return false;

    meshtastic_Channel *ch = &channels.getByIndex(channelIndex);
    if (!ch || ch->role == meshtastic_Channel_Role_DISABLED)
        return false;

    // 1. Check Compile-time Macros (Priority)
#ifdef NEXUS_TRUSTED_CHANNEL_1
    if (strcasecmp(ch->settings.name, NEXUS_TRUSTED_CHANNEL_1) == 0)
        return true;
#endif
#ifdef NEXUS_TRUSTED_CHANNEL_2
    if (strcasecmp(ch->settings.name, NEXUS_TRUSTED_CHANNEL_2) == 0)
        return true;
#endif

    // 2. Check Hardcoded fallback ONLY if macros are not defined
#if !defined(NEXUS_TRUSTED_CHANNEL_1) && !defined(NEXUS_TRUSTED_CHANNEL_2)
    if (strcasecmp(ch->settings.name, "Nexus") == 0 || strcasecmp(ch->settings.name, "Nexus2") == 0)
        return true;
#endif

    // 3. Check Vault-stored Secondary Channels
    for (int i = 0; i < 2; i++) {
        if (trusted_ch_active[i] && strcasecmp(ch->settings.name, trusted_ch_names[i]) == 0)
            return true;
    }

    return false;
}

uint32_t NexusModule::findFirstNexusChannel()
{
    for (uint32_t i = 0; i < MAX_NUM_CHANNELS; i++) {
        if (isNexusChannel(i))
            return i;
    }
    return channels.getPrimaryIndex(); // Fallback to primary if none found
}

void NexusModule::translateInbound(meshtastic_MeshPacket *p) {
    if (!p) return;
#ifdef NEXUS_PRIVATE_BUILD
    privateTranslateInbound(p);
#endif
}

void NexusModule::translateOutbound(meshtastic_MeshPacket *p) {
    (void)p;
}

void NexusModule::interceptOutgoing(meshtastic_MeshPacket *p) {
    if (!p) return;

    // 1. Hop Masking Logic
    if (hop_mask_active && p->from == myNodeInfo.my_node_num) {
        p->hop_start = hop_mask_start;
        p->hop_limit = hop_mask_limit;
    }

    // 1.1 Per-Channel Overwrite (v31)
    if (p->from == myNodeInfo.my_node_num && p->channel < 8) {
        if (nexus_per_ch_hopmask[p->channel].active) {
            p->hop_start = nexus_per_ch_hopmask[p->channel].start;
            p->hop_limit = nexus_per_ch_hopmask[p->channel].limit;
        }
    }

    // 1.2 Per-Port Overwrite (v31)
    if (p->from == myNodeInfo.my_node_num && p->which_payload_variant == meshtastic_MeshPacket_decoded_tag) {
        int portIdx = -1;
        switch(p->decoded.portnum) {
            case meshtastic_PortNum_NODEINFO_APP: portIdx = 0; break;
            case meshtastic_PortNum_TELEMETRY_APP: portIdx = 1; break;
            case meshtastic_PortNum_POSITION_APP: portIdx = 2; break;
            case meshtastic_PortNum_TRACEROUTE_APP: portIdx = 3; break;
            case meshtastic_PortNum_TEXT_MESSAGE_APP: portIdx = 4; break;
            default: break;
        }
        if (portIdx != -1 && nexus_per_port_hopmask[portIdx].active) {
            p->hop_start = nexus_per_port_hopmask[portIdx].start;
            p->hop_limit = nexus_per_port_hopmask[portIdx].limit;
        }
    }

#ifdef NEXUS_PRIVATE_BUILD
    privateInterceptOutgoing(p);
#endif

    // 5. Emoji Mangling Logic (v37)
    if (isFromUs(p) && p->which_payload_variant == meshtastic_MeshPacket_decoded_tag &&
        p->decoded.portnum == meshtastic_PortNum_TEXT_MESSAGE_APP &&
        p->decoded.payload.size > 7) {

        if (memcmp(p->decoded.payload.bytes, "/emoji ", 7) == 0) {
            size_t emojiLen = p->decoded.payload.size - 7;
            memmove(p->decoded.payload.bytes, p->decoded.payload.bytes + 7, emojiLen);
            p->decoded.payload.size = emojiLen;
            p->decoded.emoji = 1;
            p->decoded.bitfield |= (1 << 0); // REACTION flag
            p->decoded.has_bitfield = true;
            LOG_INFO("Nexus: Emoji Mangle applied for text message");
        }
    }
}

void NexusModule::applyTS(uint32_t *traceId) {
    if (!traceId) return;
#ifdef NEXUS_PRIVATE_BUILD
    privateApplyTS(traceId);
#endif
}

int32_t NexusModule::runOnce() {
    uint32_t now = Time::getMillis();

    // 0. Delayed InputBroker Hook (Ensure inputBroker is initialized)
    if (!input_hooked && inputBroker) {
        inputEventObserver->observe(inputBroker);
        input_hooked = true;
        LOG_INFO("Nexus: InputBroker hook established.");
    }

    // 1. Process Deferred Replies (Jitter)
    if (pendingReply.active && Throttle::deadlinePassedAt(now, pendingReply.send_at)) {
        meshtastic_MeshPacket *reply = (router) ? router->allocForSending() : nullptr;
        if (reply) {
            reply->decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;
            reply->to = pendingReply.to;
            reply->channel = pendingReply.channel;
            reply->decoded.reply_id = pendingReply.reply_id;
            reply->priority = meshtastic_MeshPacket_Priority_RELIABLE;
            reply->want_ack = pendingReply.want_ack;

            size_t len = strlen(pendingReply.message);
            memcpy(reply->decoded.payload.bytes, pendingReply.message, len);
            reply->decoded.payload.size = len;

            if (pendingReply.is_reaction) {
                reply->decoded.emoji = 1; // Mark as emoji reaction (🤖 trigger only)
                reply->decoded.bitfield |= (1 << 0); // Reaction flag
                reply->decoded.has_bitfield = true;
            }

            if (service) service->sendToMesh(reply);
            LOG_INFO("Nexus: Deferred reply sent to %08x", pendingReply.to);
        }
        pendingReply.active = false;
    }

    // 1.5 Nexus LORACHIPSTATS: 1 Hz chip-state sampling (self-throttled inside,
    // so the early setIntervalFromNow() wakeups don't skew the sample rate).
    if (RadioLibInterface::instance) {
        RadioLibInterface::instance->sampleChipState();
    }

    // 2. TRACE Watchdog (100s timeout)
    if (trace_active && Throttle::hasElapsed(trace_start_time, 100000)) {
        trace_active = false;
        trace_reply_to = 0;
        const char *message = "JT: Trace Timeout (100s)";
        sendReply(traceReply, message, strlen(message));
    }

    // 3. Process Deferred LoRa Changes
    if (pendingLoRaConfig && Throttle::deadlinePassedAt(now, pendingLoRaApplyAt)) {
        LOG_INFO("Nexus: Applying deferred LoRa configuration change (LIVE).");
        if (adminModule) adminModule->handleSetConfig(pendingConfig, false);
        pendingLoRaConfig = false;
        pendingLoRaApplyAt = 0;
    }

    // 4.5 Multi-step Macro Runner (v30)
    if (macro_running_cmd[0] != '\0' && Throttle::deadlinePassedAt(now, macro_next_step_at)) {
        // Find next command in sequence (delimited by ;)
        static char current_cmd[128];
        memset(current_cmd, 0, sizeof(current_cmd));
        int i = 0;
        bool found_semi = false;

        while (macro_running_cmd[macro_step_ptr] != '\0' && i < 127) {
            if (macro_running_cmd[macro_step_ptr] == ';') {
                found_semi = true;
                macro_step_ptr++;
                break;
            }
            current_cmd[i++] = macro_running_cmd[macro_step_ptr++];
        }
        current_cmd[i] = '\0';

        // Trim leading spaces
        char *cmdPtr = current_cmd;
        while (*cmdPtr == ' ') cmdPtr++;

        if (*cmdPtr != '\0') {
            LOG_INFO("Nexus: Macro executing step: %s", cmdPtr);
            // Execute the command (as a shadow command to reuse existing logic)
            macroStepExecuting = true;
            handleNexusCommand(macroRequest, cmdPtr, macroIsBroadcast, macro_reply_to, macro_to_phone, macroDepth);
            macroStepExecuting = false;

            if (found_semi) {
                macro_next_step_at = now + 10000; // 10s delay between steps
                setIntervalFromNow(100);
            } else {
                // Done
                macro_running_cmd[0] = '\0';
            }
        } else {
            if (found_semi) {
                macro_next_step_at = now;
                setIntervalFromNow(100);
            } else {
                macro_running_cmd[0] = '\0';
            }
        }
    }

    // Re-render read-only reports into successive bounded windows.
    if (paging_ptr == 0 && pagingWindow.hasMore() && pagingCommand[0] && !pagingRendering) {
        pagingOffset = pagingWindow.end();
        pagingRendering = true;
        handleNexusCommand(pagingRequest, pagingCommand, pagingIsBroadcast, paging_reply_id, pagingReply.phoneOnly);
        pagingRendering = false;
    }
    if (paging_ptr > 0 && Throttle::deadlinePassedAt(now, paging_next_send)) {
        char partBuf[NEXUS_MAX_REPLY];
        int headerLen = snprintf(partBuf, sizeof(partBuf), "P%d: ", paging_part + 1);
        size_t take = std::min<size_t>(paging_ptr, NEXUS_MAX_REPLY_BODY - headerLen);
        // Keep UTF-8 code points in a single transport packet.
        if (take < paging_ptr)
            while (take && (static_cast<uint8_t>(paging_buf[take]) & 0xC0) == 0x80)
                --take;
        memcpy(partBuf + headerLen, paging_buf, take);
        partBuf[headerLen + take] = '\0';
        if (sendReply(pagingReply, partBuf, headerLen + take)) {
            ++paging_part;
            memmove(paging_buf, paging_buf + take, paging_ptr - take);
            paging_ptr -= take;
        }
        paging_next_send = now + 2000;
    }
    if (netscan_active && (now - netscan_start > 60000)) {
        netscan_active = false; // Stop collection FIRST (ESP32 Atomic)

        if (nexus_netscan_count == 0) {
            // Send empty reply
            const char *msg = "JT NETSCAN: No direct neighbors heard.";
            size_t mLen = strlen(msg);
            if (mLen > NEXUS_MAX_REPLY_BODY) mLen = NEXUS_MAX_REPLY_BODY;
            sendReply(netscanReply, msg, mLen, false);
        } else {
            // Generate multi-part CSV reports
            char buf[NEXUS_MAX_REPLY];
            int part = 1;
            int len = snprintf(buf, sizeof(buf), "JT NETSCAN (%d):\nID,RSSI,SNR,FE,RY\n", nexus_netscan_count);

            // Nexus: Post-scan Deduplication Logic
            for(int i=0; i<nexus_netscan_count; i++) {
                // Check if this node has already been processed in this report
                bool alreadyProcessed = false;
                for(int k=0; k<i; k++) {
                    if (nexus_netscan_db[k].nodeID == nexus_netscan_db[i].nodeID) {
                        alreadyProcessed = true;
                        break;
                    }
                }
                if (alreadyProcessed) continue;

                auto &e = nexus_netscan_db[i];
                char idStr[12];
                auto node = nodeDB->getMeshNode(e.nodeID);
                if (node && nexus_has_user(node) && node->short_name[0] != '\0') {
                    strncpy(idStr, node->short_name, sizeof(idStr));
                } else {
                    snprintf(idStr, sizeof(idStr), "%08x", (unsigned int)e.nodeID);
                }

                char line[40];
                int lLen = snprintf(line, sizeof(line), "%s,%d,%.1f,%.0f,%02x\n",
                                   idStr, (int)e.rssi, e.snr, e.freqErr, e.relay);

                if (len + lLen >= (int)NEXUS_MAX_REPLY_BODY - 5) {
                    // Send current buffer
                    sendReply(netscanReply, buf, len, false);

                    // Reset buffer for next part
                    len = snprintf(buf, sizeof(buf), "NETSCAN P%d:\n", ++part);
                }
                memcpy(buf + len, line, lLen);
                len += lLen;
            }
            // Send final part
            sendReply(netscanReply, buf, len, false);
        }
    }

    if (clientscan_active && (now - clientscan_start > clientscan_duration_ms)) {
        clientscan_active = false; // Stop collection FIRST (ESP32 Atomic)

        if (clientscan_count == 0) {
            // The scan owns its context independently of command paging.
            const char *msg = "JT CLIENTSCAN: No repeats heard.";
            size_t mLen = strlen(msg);
            if (mLen > NEXUS_MAX_REPLY_BODY)
                mLen = NEXUS_MAX_REPLY_BODY;
            sendReply(clientscanReply, msg, mLen, false);
        } else {
            char buf[NEXUS_MAX_REPLY];
            int part = 1;
            int len = snprintf(buf, sizeof(buf), "JT CLIENTSCAN (!%08x):\nRY,Delay,RSSI,H/S,SNR\n", (unsigned int)clientscan_target_id);

            // Static (BSS) so 64B+32B+64B+200B don't all stack-frame here.
            static char relayNames[64];
            static char ryStr[32];
            static char line[64];
            for(int i=0; i<clientscan_count; i++) {
                auto &e = clientscan_db[i];
                memset(relayNames, 0, sizeof(relayNames));
                int foundCount = 0;
                uint8_t targetSuffix = e.relay;

                for (size_t k = 0; k < nodeDB->getNumMeshNodes() && foundCount < 5; k++) {
                    auto n = nodeDB->getMeshNodeByIndex(k);
                    if (n && (uint8_t)(n->num & 0xFF) == targetSuffix) {
                        if (n->short_name[0] != '\0') {
                            if (foundCount > 0) strncat(relayNames, ",", sizeof(relayNames) - strlen(relayNames) - 1);
                            strncat(relayNames, n->short_name, sizeof(relayNames) - strlen(relayNames) - 1);
                            foundCount++;
                        }
                    }
                }

                memset(ryStr, 0, sizeof(ryStr));
                if (foundCount > 0) {
                    snprintf(ryStr, sizeof(ryStr), "%s(%02x)", relayNames, targetSuffix);
                } else {
                    snprintf(ryStr, sizeof(ryStr), "*%02x", targetSuffix);
                }

                uint8_t usedHops = (e.hop_start >= e.hop_limit) ? (e.hop_start - e.hop_limit) : 0;
                memset(line, 0, sizeof(line));
                int lLen = snprintf(line, sizeof(line), "%s,%ums,%d,%u/%u,%.1f\n",
                                   ryStr, (unsigned int)e.delay_ms, (int)e.rssi, usedHops, e.hop_start, e.snr);

                if (len + lLen >= (int)NEXUS_MAX_REPLY_BODY - 5) {
                    sendReply(clientscanReply, buf, len, false);
                    len = snprintf(buf, sizeof(buf), "CLIENTSCAN P%d:\n", ++part);
                }
                memcpy(buf + len, line, lLen);
                len += lLen;
            }
            // Send final part
            sendReply(clientscanReply, buf, len, false);
        }
    }

    // 6. Tactical Noise Floor Sampling
    if (noise_sampling_active && Throttle::deadlinePassedAt(now, noise_sampling_next)) {
        // Safety: Timeout after 15s or too many retries (300)
        if (now - noise_sampling_start > 15000 || noise_retry_count > 300) {
            noise_sampling_active = false;
            char errBuf[NEXUS_MAX_REPLY];
            snprintf(errBuf, sizeof(errBuf), "JT: Noise Floor Error\nTimeout / Busy (R:%d)", noise_retry_count);
            size_t eLen = strlen(errBuf);
            sendReply(noiseReply, errBuf, eLen, false);
            if (freqscan_active) {
                config.lora = freqscan_backup;
                if (service) service->configChanged.notifyObservers(nullptr);
                freqscan_active = false;
            }
            return 1000;
        }

        if (noise_sample_count < 100) {
            // Fix: Check busy state AFTER reading to validate sample
            bool isSending = false;
            bool isReceiving = false;
            if (RadioLibInterface::instance) {
                isSending = RadioLibInterface::instance->isSending();
                isReceiving = RadioLibInterface::instance->isActivelyReceiving();
            }

            float instRSSI = RadioLibInterface::instance ? RadioLibInterface::instance->getInstantaneousRSSI() : -1000.0f;

            // Discard sample if radio is busy OR value is a hardware error code
            if (!isSending && !isReceiving && instRSSI < -5.0f && instRSSI > -160.0f) {
                noise_samples[noise_sample_count++] = instRSSI;
                noise_sampling_next = now + 10; // Fast 10ms sampling interval
            } else {
                noise_retry_count++;
                noise_sampling_next = now + 10; // Fast retry
            }
        } else {
            noise_sampling_active = false;

            float sum = 0, minR = 0, maxR = -200.0f;
            static char rawList[128];
            memset(rawList, 0, sizeof(rawList));
            for(int i=0; i<100; i++) {
                sum += noise_samples[i];
                if (i == 0 || noise_samples[i] < minR) minR = noise_samples[i];
                if (noise_samples[i] > maxR) maxR = noise_samples[i];

                // Append whole number to raw list (reporting 25 samples: every 4th)
                if (i % 4 == 0) {
                    char sBuf[8];
                    snprintf(sBuf, sizeof(sBuf), "%d,", (int)noise_samples[i]);
                    if (strlen(rawList) + strlen(sBuf) < sizeof(rawList)) strcat(rawList, sBuf);
                }
            }
            float avg = sum / 100.0f;

            // Determine Radio State and Params for diagnostic
            const char* rState = "UNK";
            float curFreq = config.lora.override_frequency;
            uint16_t curBw = config.lora.bandwidth;

            if (RadioLibInterface::instance) {
                if (RadioLibInterface::instance->isSending()) rState = "TX";
                else if (RadioLibInterface::instance->isActivelyReceiving()) rState = "BUSY";
                else rState = "RX";
            }

            char nBuf[NEXUS_MAX_REPLY];
            snprintf(nBuf, sizeof(nBuf), "Noise (%s,R:%d)\n%.2fM/%uk\n%s\nAvg:%.1f Min:%d Max:%d",
                     rState, noise_retry_count, curFreq, curBw, rawList, avg, (int)minR, (int)maxR);

            if (freqscan_active) {
                float targetF = config.lora.override_frequency;
                // 4. Restore original config (RAM only)
                config.lora = freqscan_backup;
                if (service) service->configChanged.notifyObservers(nullptr);

                char fBuf[NEXUS_MAX_REPLY];
                snprintf(fBuf, sizeof(fBuf), "JT FREQSCAN (%.2fM):\n%s", targetF, nBuf);
                strncpy(nBuf, fBuf, sizeof(nBuf));
                nBuf[sizeof(nBuf)-1] = '\0';
                freqscan_active = false;
            }

            size_t nLen = strlen(nBuf);
            if (nLen > NEXUS_MAX_REPLY_BODY) nLen = NEXUS_MAX_REPLY_BODY;
            sendReply(noiseReply, nBuf, nLen, false);
            setIntervalFromNow(1000);
        }
    }

    // Dynamic thread scheduling based on active tactical tasks!

    // 7. Nexus-Cron Scheduler
    static uint32_t last_cron_check = 0;
    uint32_t wall_clock = getTime(false);

    // Run scheduler check every 1s
    if (!freqscan_active && !noise_sampling_active && now - last_cron_check >= 1000) {
        last_cron_check = now;

        for (int i = 0; i < 8; i++) {
            if (!cron_table[i].active) continue;

            bool trigger = false;
            NexusCronEntry &job = cron_table[i];

            if (job.mode == 0) { // PER (Periodic)
                // Safety: Periodic jobs MUST have a value >= 60s and a non-empty command to trigger
                if (job.value >= 60 && job.command[0] != '\0' && (Throttle::hasElapsed(cronLastRunMs[i], job.value * 1000)))
                    trigger = true;
            }
            else if (wall_clock > 1700000000) { // Wall-clock modes (SCH, AT)
                if (job.mode == 1) { // SCH (Daily Schedule)
                    if (nexus::dailyDue(wall_clock, job.value, job.last_run))
                        trigger = true;
                }
                else if (job.mode == 2) { // AT (One-shot)
                    if (wall_clock >= job.value && job.last_run < job.value) trigger = true;
                }
            }

            if (trigger) {
                LOG_INFO("Nexus: Cron Triggering Job %d: %s", i, job.command);
                cronLastRunMs[i] = now;
                job.last_run = (job.mode == 0) ? static_cast<uint32_t>(Time::getUptimeSecs()) : wall_clock;
                if (job.mode == 2) job.active = 0;

                meshtastic_MeshPacket dummy = meshtastic_MeshPacket_init_default;
                dummy.from = myNodeInfo.my_node_num;
                dummy.to = myNodeInfo.my_node_num;
                handleNexusCommand(dummy, job.command, false, 0, true);
            }
        }
    }

    // 8. GPIO Watcher (v37)
    if (gpio_watch_mask != 0 && (now - last_gpio_poll > 2000)) {
        last_gpio_poll = now;
        uint64_t current_val = 0;
        for (int i = 0; i < 64; i++) {
            if (gpio_watch_mask & (1ULL << i)) {
                if (digitalRead(i)) current_val |= (1ULL << i);
            }
        }

        if (current_val != gpio_watch_prev) {
            uint64_t changed = current_val ^ gpio_watch_prev;
            gpio_watch_prev = current_val;

            for (int i = 0; i < 64; i++) {
                if (changed & (1ULL << i)) {
                    bool state = (current_val & (1ULL << i)) != 0;
                    char alert[NEXUS_MAX_REPLY];
                    snprintf(alert, sizeof(alert), "JT: GPIO %d changed to %s", i, state ? "HIGH" : "LOW");
                    LOG_INFO("%s", alert);
                    sendToNexusChannel(alert, strlen(alert));
                }
            }
        }
        setIntervalFromNow(100);
    }

    // 9. Voltage Threshold Automation Engine (v68)
    if (now - last_volt_check_ms >= 15000) {
        last_volt_check_ms = now;
        runVoltageAutomations();
    }

    if (noise_sampling_active || freqscan_active)
        return 50;
    if (netscan_active || clientscan_active || pendingReply.active || pendingLoRaConfig || macro_running_cmd[0] || paging_ptr ||
        pagingWindow.hasMore())
        return 100;
    return 1000; // Default idle sleep
}

void NexusModule::setHibernating(bool hib)
{
    is_hibernating = hib;
#if defined(ARCH_ESP32)
    rtc_nexus_hibernating = hib;
#endif
}

uint16_t NexusModule::getVoltageReadingMv(uint8_t channel)
{
    if (channel == 0) {
        uint16_t v = powerStatus ? powerStatus->getBatteryVoltageMv() : 0;
#if HAS_TELEMETRY && !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR && __has_include(<INA3221.h>)
        if (v == 0 && ina3221Sensor.hasSensor()) {
            int16_t raw = ina3221Sensor.getRawBusVoltage(0);
            if (raw > 0) v = (uint16_t)(raw * 8);
        }
#endif
        return v;
    }
#if HAS_TELEMETRY && !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR && __has_include(<INA3221.h>)
    if (ina3221Sensor.hasSensor() && channel <= 2) {
        int16_t raw = ina3221Sensor.getRawBusVoltage(channel);
        if (raw > 0) return (uint16_t)(raw * 8);
    }
#endif
    return 0;
}

void NexusModule::runVoltageAutomations()
{
    bool any_active = false;
    for (int i = 0; i < 4; i++) {
        if (volt_rules[i].action != NEXUS_VOLT_ACT_NONE && volt_rules[i].mv > 0) {
            any_active = true;
            break;
        }
    }
    if (!any_active && !is_hibernating) return;

    for (int i = 0; i < 4; i++) {
        NexusVoltRule &rule = volt_rules[i];
        if (rule.action == NEXUS_VOLT_ACT_NONE || rule.mv == 0) continue;

        uint16_t cur_mv = getVoltageReadingMv(rule.channel);
        if (cur_mv == 0) continue;

        bool match = false;
        if (rule.direction == NEXUS_VOLT_DIR_HIGH) {
            match = (cur_mv >= rule.mv);
        } else {
            match = (cur_mv <= rule.mv);
        }

        if (match) {
            if (volt_debounce_count[i] < 3) {
                volt_debounce_count[i]++;
            }
            if (volt_debounce_count[i] >= 3 && !volt_triggered[i]) {
                volt_triggered[i] = true;
                LOG_INFO("Nexus: Volt slot %d triggered (%u mV, dir %u, act %u, ch %u)",
                         i + 1, (unsigned)cur_mv, (unsigned)rule.direction, (unsigned)rule.action, (unsigned)rule.channel);

                if (rule.action == NEXUS_VOLT_ACT_CMD) {
                    if (rule.command[0] != '\0') {
                        meshtastic_MeshPacket dummy = meshtastic_MeshPacket_init_default;
                        dummy.from = myNodeInfo.my_node_num;
                        dummy.to = myNodeInfo.my_node_num;
                        handleNexusCommand(dummy, rule.command, false, 0, true);
                    }
                } else if (rule.action == NEXUS_VOLT_ACT_HIBERNATE) {
                    setHibernating(true);
                    if (rule.command[0] != '\0') {
                        meshtastic_MeshPacket dummy = meshtastic_MeshPacket_init_default;
                        dummy.from = myNodeInfo.my_node_num;
                        dummy.to = myNodeInfo.my_node_num;
                        handleNexusCommand(dummy, rule.command, false, 0, true);
                    }
                    LOG_INFO("Nexus: Entering voltage-induced hibernation (sniff interval 60s)...");
                    // Magnemite tweak at apply (Jenny 2026-10-21): skipSaveNodeDb=true —
                    // saveToDisk() has no dirty-check, so the default would write every
                    // NodeDB segment on each ~75s hibernate cycle: flash wear inside the
                    // anti-flash-wear feature. Nothing new was heard since pre-hibernation.
                    doDeepSleep(60000, true, true);
                } else if (rule.action == NEXUS_VOLT_ACT_WAKE) {
                    setHibernating(false);
                    if (rule.command[0] != '\0') {
                        meshtastic_MeshPacket dummy = meshtastic_MeshPacket_init_default;
                        dummy.from = myNodeInfo.my_node_num;
                        dummy.to = myNodeInfo.my_node_num;
                        handleNexusCommand(dummy, rule.command, false, 0, true);
                    }
                    LOG_INFO("Nexus: Woke from hibernation by voltage rule!");
                }
            }
        } else {
            volt_debounce_count[i] = 0;
            // 50mV hysteresis to re-arm
            if (rule.direction == NEXUS_VOLT_DIR_HIGH) {
                if (cur_mv + 50 <= rule.mv) {
                    volt_triggered[i] = false;
                }
            } else {
                if (cur_mv >= rule.mv + 50) {
                    volt_triggered[i] = false;
                }
            }
        }
    }

    if (is_hibernating) {
        bool has_wake_rule = false;
        for (int i = 0; i < 4; i++) {
            if (volt_rules[i].action == NEXUS_VOLT_ACT_WAKE && volt_rules[i].mv > 0) {
                has_wake_rule = true;
                break;
            }
        }
        if (!has_wake_rule) {
            uint16_t hib_mv = 0;
            uint8_t hib_ch = 0;
            for (int i = 0; i < 4; i++) {
                if (volt_rules[i].action == NEXUS_VOLT_ACT_HIBERNATE && volt_rules[i].mv > hib_mv) {
                    hib_mv = volt_rules[i].mv;
                    hib_ch = volt_rules[i].channel;
                }
            }
            uint16_t cur = getVoltageReadingMv(hib_ch);
            if (cur >= hib_mv + 100) {
                setHibernating(false);
                LOG_INFO("Nexus: Auto-woke from hibernation (ch %u cur %u mV >= %u mV)", (unsigned)hib_ch, (unsigned)cur, (unsigned)(hib_mv + 100));
                return;
            }
        }
        LOG_INFO("Nexus: Hibernating sniff complete, returning to deep sleep (60s)...");
        // Magnemite tweak at apply (Jenny 2026-10-21): skipSaveNodeDb=true (see HIB trigger above).
        doDeepSleep(60000, true, true);
    }
}

bool NexusModule::setTierChannelOverride(uint8_t channel, uint8_t sf)
{
    if (channel >= MAX_NUM_CHANNELS)
        return false;
    for (uint8_t i = 0; i < tierChannelOverrideCount; i++) {
        if (tierChannelOverrides[i].channel == channel) {
            tierChannelOverrides[i].sf = sf;
            return true;
        }
    }
    if (tierChannelOverrideCount >= MAX_TIER_CHANNEL_OVERRIDES)
        return false;
    tierChannelOverrides[tierChannelOverrideCount].channel = channel;
    tierChannelOverrides[tierChannelOverrideCount].sf = sf;
    tierChannelOverrideCount++;
    return true;
}

bool NexusModule::clearTierChannelOverride(uint8_t channel)
{
    for (uint8_t i = 0; i < tierChannelOverrideCount; i++) {
        if (tierChannelOverrides[i].channel != channel)
            continue;
        for (uint8_t j = i; j + 1 < tierChannelOverrideCount; j++)
            tierChannelOverrides[j] = tierChannelOverrides[j + 1];
        tierChannelOverrideCount--;
        return true;
    }
    return false;
}

const NexusModule::TierChannelOverride *NexusModule::findTierChannelOverride(ChannelIndex channel) const
{
    if (channel >= MAX_NUM_CHANNELS)
        return nullptr;
    for (uint8_t i = 0; i < tierChannelOverrideCount; i++)
        if (tierChannelOverrides[i].channel == channel)
            return &tierChannelOverrides[i];
    return nullptr;
}

void NexusModule::onRebroadcastPacket(const meshtastic_MeshPacket *rxp, meshtastic_MeshPacket *txp)
{
    if (!rxp || !txp)
        return;

    RadioInterface *rif = router ? router->getInterface() : nullptr;
    if (!rif)
        return;

    if (multiSFConfig.enabled) {
        // Multi-SF tiers are a same-carrier protocol system, not the frequency
        // split bridge. Meshtastic traffic uses the configured Meshtastic tier.
        const nexus::TierConfig *tier = multiSFTxTierForChannel(txp->channel);
        if (tier && (tier->sf != rif->getSF() || tier->syncword != rif->getSyncWord()))
            NexusTxHook::setOverride(txp->id, tier->sf, tier->tx_power, tier->syncword);
        return;
    }

    if (!isSplitEnabled())
        return;

    if (splitConfig.mode == NEXUS_SPLIT_ROOF_MSF) {
        // Mode 1: LR2021 Multi-SF Bridge
        uint8_t rxSF = rif->getLastRxSF();
        if (rxSF == splitConfig.alt_sf) {
            // Arrived on Side Detector (Outside Mesh e.g. SF7) -> Downlink into house on Primary SF (e.g. SF6)
            NexusTxHook::setOverride(txp->id, rif->getSF(), splitConfig.alt_tx_power);
            LOG_DEBUG("Nexus SPLIT (ROOF_MSF): downlink 0x%08x to indoor tier (SF%u @ %d dBm)",
                      txp->id, rif->getSF(), splitConfig.alt_tx_power);

            // If MESH_RELAY flag is set, also queue a copy on the outside mesh (SF7 @ normal power)
            if ((splitConfig.flags & NEXUS_SPLIT_FLAG_MESH_RELAY) != 0 && router) {
                if (meshtastic_MeshPacket *meshCopy = packetPool.allocCopy(*txp)) {
                    meshCopy->id = generatePacketId();
                    // F3 fix (Jenny 2026-10-23): tag the copy for the mesh side tier — without
                    // this the relay-to-mesh transmitted untagged on the baseline (indoor) SF,
                    // so outdoor nodes never heard it. Same mechanism the uplink branch uses.
                    NexusTxHook::setOverride(meshCopy->id, splitConfig.alt_sf, 0x7F);
                    router->send(meshCopy);
                }
            }
        } else if (rxSF == rif->getSF()) {
            // Arrived on Main Detector (Inside Node e.g. SF6) -> Uplink out to outside mesh on Side SF (e.g. SF7)
            bool isInside = true;
            if ((splitConfig.flags & NEXUS_SPLIT_FLAG_USE_ZH_MASK) != 0) {
                isInside = isZeroHopNode(rxp->relay_node) || (rxp->hop_start == rxp->hop_limit);
            }
            if (isInside) {
                NexusTxHook::setOverride(txp->id, splitConfig.alt_sf, 0x7F);
                LOG_DEBUG("Nexus SPLIT (ROOF_MSF): uplink 0x%08x to mesh tier (SF%u @ full power)",
                          txp->id, splitConfig.alt_sf);

                // If INDOOR_ECHO flag is set, also echo on SF6 so other indoor nodes hear it
                if ((splitConfig.flags & NEXUS_SPLIT_FLAG_INDOOR_ECHO) != 0 && router) {
                    if (meshtastic_MeshPacket *echoCopy = packetPool.allocCopy(*txp)) {
                        echoCopy->id = generatePacketId();
                        NexusTxHook::setOverride(echoCopy->id, rif->getSF(), splitConfig.alt_tx_power);
                        router->send(echoCopy);
                    }
                }
            }
        }
    } else if (splitConfig.mode == NEXUS_SPLIT_ROOF_SPLIT) {
        // Mode 2: Single-Radio Rooftop Split
        bool isInside = false;
        if ((splitConfig.flags & NEXUS_SPLIT_FLAG_USE_ZH_MASK) != 0) {
            isInside = isZeroHopNode(rxp->relay_node) || (nodeDB && nodeDB->isFromOrToFavoritedNode(*rxp));
        }

        if (isInside) {
            // Uplink from marked indoor node: repeat outward on Primary Preset at full power
            LOG_DEBUG("Nexus SPLIT (ROOF_SPLIT): uplink 0x%08x from indoor node -> mesh", txp->id);
            if ((splitConfig.flags & NEXUS_SPLIT_FLAG_INDOOR_ECHO) != 0 && router) {
                if (meshtastic_MeshPacket *echoCopy = packetPool.allocCopy(*txp)) {
                    echoCopy->id = generatePacketId();
                    bool isFast = (splitConfig.alt_freq == rif->getFreq() && splitConfig.alt_bw == bwKHzToCode(rif->getBW()) && splitConfig.alt_cr == rif->getCR());
                    if (isFast) {
                        NexusTxHook::setOverride(echoCopy->id, splitConfig.alt_sf, splitConfig.alt_tx_power);
                    } else {
                        NexusTxHook::setOverrideFull(echoCopy->id, splitConfig.alt_sf, splitConfig.alt_bw, splitConfig.alt_cr, splitConfig.alt_freq, splitConfig.alt_tx_power);
                    }
                    router->send(echoCopy);
                }
            }
        } else {
            // Outside mesh packet -> Downlink to indoor nodes on Alt Preset at indoor power
            bool isFast = (splitConfig.alt_freq == rif->getFreq() && splitConfig.alt_bw == bwKHzToCode(rif->getBW()) && splitConfig.alt_cr == rif->getCR());
            if (isFast) {
                NexusTxHook::setOverride(txp->id, splitConfig.alt_sf, splitConfig.alt_tx_power);
            } else {
                NexusTxHook::setOverrideFull(txp->id, splitConfig.alt_sf, splitConfig.alt_bw, splitConfig.alt_cr, splitConfig.alt_freq, splitConfig.alt_tx_power);
            }
            LOG_DEBUG("Nexus SPLIT (ROOF_SPLIT): downlink 0x%08x to indoor tier (@ %d dBm)", txp->id, splitConfig.alt_tx_power);
        }
    }
}

void NexusModule::onClientSendPacket(meshtastic_MeshPacket *p)
{
    if (!p)
        return;

    RadioInterface *rif = router ? router->getInterface() : nullptr;
    if (!rif)
        return;

    if (multiSFConfig.enabled) {
        const nexus::TierConfig *tier = multiSFTxTierForChannel(p->channel);
        if (tier && (tier->sf != rif->getSF() || tier->syncword != rif->getSyncWord()))
            NexusTxHook::setOverride(p->id, tier->sf, tier->tx_power, tier->syncword);
        return;
    }

    if (!isSplitEnabled())
        return;

    if (splitConfig.mode == NEXUS_SPLIT_CLIENT_SPLIT) {
        // Mode 3: Asymmetric Indoor Client -> switch all outgoing transmissions to Uplink Preset at alt_tx_power
        bool isFast = (splitConfig.alt_freq == rif->getFreq() && splitConfig.alt_bw == bwKHzToCode(rif->getBW()) && splitConfig.alt_cr == rif->getCR());
        if (isFast) {
            NexusTxHook::setOverride(p->id, splitConfig.alt_sf, splitConfig.alt_tx_power);
        } else {
            NexusTxHook::setOverrideFull(p->id, splitConfig.alt_sf, splitConfig.alt_bw, splitConfig.alt_cr, splitConfig.alt_freq, splitConfig.alt_tx_power);
        }
        LOG_DEBUG("Nexus SPLIT (CLIENT_SPLIT): outgoing packet 0x%08x tagged for uplink (@ %d dBm)", p->id, splitConfig.alt_tx_power);
    }
}

int8_t NexusModule::getChipPower(int8_t power) {
#if HAS_LORA_FEM
    if (!devicestate.owner.is_licensed) {
        return loraFEMInterface.powerConversion(power);
    }
#else
#ifdef ARCH_PORTDUINO
    size_t num_pa_points = portduino_config.num_pa_points;
    const uint16_t *tx_gain = portduino_config.tx_gain_lora;
#elif defined(NUM_PA_POINTS) && defined(TX_GAIN_LORA)
    size_t num_pa_points = NUM_PA_POINTS;
    const uint16_t tx_gain[NUM_PA_POINTS] = {TX_GAIN_LORA};
#else
    size_t num_pa_points = 0;
#endif
#if defined(NUM_PA_POINTS) && defined(TX_GAIN_LORA)
    if (num_pa_points == 1) {
        if (tx_gain[0] > 0 && !devicestate.owner.is_licensed) {
            return power - tx_gain[0];
        }
    } else if (!devicestate.owner.is_licensed) {
        for (int radio_dbm = 0; radio_dbm < (int)num_pa_points; radio_dbm++) {
            if (((radio_dbm + tx_gain[radio_dbm]) > power) ||
                ((radio_dbm == (int)(num_pa_points - 1)) && ((radio_dbm + tx_gain[radio_dbm]) <= power))) {
                return power - tx_gain[radio_dbm];
            }
        }
    }
#endif
#endif
    return power;
}


const char* NexusModule::getResetReason() {
#ifdef ARCH_ESP32
    esp_reset_reason_t reason = esp_reset_reason();
    switch (reason) {
        case ESP_RST_POWERON: return "POWER_ON";
        case ESP_RST_EXT:     return "EXT_PIN";
        case ESP_RST_SW:      return "SOFTWARE";
        case ESP_RST_PANIC:   return "CRASH/PANIC";
        case ESP_RST_INT_WDT: return "INT_WDT";
        case ESP_RST_TASK_WDT:return "TASK_WDT";
        case ESP_RST_WDT:     return "WDT";
        case ESP_RST_DEEPSLEEP: return "DEEP_SLEEP";
        case ESP_RST_BROWNOUT: return "BROWNOUT";
        case ESP_RST_SDIO:    return "SDIO";
        default:              return "UNKNOWN";
    }
#elif defined(ARCH_NRF52)
    uint32_t why = NRF_POWER->RESETREAS;
    if (why & 0x01) return "PIN";
    if (why & 0x02) return "WDT";
    if (why & 0x04) return "SREQ";
    if (why & 0x08) return "LOCKUP";
    if (why & 0x10) return "OFF";
    if (why & 0x20) return "LPCOMP";
    if (why & 0x40) return "DIF";
    if (why & 0x80) return "NFC";
    return "POR";
#else
    return "N/A";
#endif
}

float NexusModule::getDieTemp() {
#ifdef ARCH_ESP32
    return temperatureRead();
#elif defined(ARCH_NRF52)
    int32_t temp = 0;
    // Use SoftDevice API to avoid HardFault while BLE is active
    if (sd_temp_get(&temp) == 0) {
        return (float)temp / 4.0f;
    }
    return 0.0f;
#endif
    return 0.0f;
}

void NexusModule::formatMCUInfo(char *buf, size_t bufSize) {
    if (!buf || bufSize == 0) return;
    buf[0] = '\0';

    // Uptime formatter (shared by both paths) — ms -> 'Dd HHh MMm SSs' for >=1d, else 'Hh MMm SSs'.
    // Use Time::getMillisMonotonic() (uint64_t) so the display never rolls over at the
    // 49.7-day Time::getMillis() boundary. Days stay in uint32_t (~136 years to overflow).
    auto writeUptime = [](char *out, size_t outSize) {
        uint64_t ms = Time::getMillisMonotonic();
        uint32_t s  = (uint32_t)(ms / 1000ULL);
        uint32_t days = s / 86400; s %= 86400;
        uint32_t h = s / 3600; s %= 3600;
        uint32_t m = s / 60; s %= 60;
        if (days > 0) snprintf(out, outSize, "%ud %02uh %02um %02us", days, h, m, (unsigned)s);
        else          snprintf(out, outSize, "%uh %02um %02us", h, m, (unsigned)s);
    };

    char upStr[32];
    writeUptime(upStr, sizeof(upStr));

#if defined(ARCH_ESP32)
    // === ESP32 family (covers ESP32 / S2 / S3 / C3 / C6) ===
    esp_chip_info_t info;
    esp_chip_info(&info); // never fails in IDF; populates struct in place

    const char *modelStr = "ESP32";
    switch (info.model) {
        case CHIP_ESP32:   modelStr = "ESP32"; break;
        case CHIP_ESP32S2: modelStr = "ESP32-S2"; break;
        case CHIP_ESP32S3: modelStr = "ESP32-S3"; break;
        case CHIP_ESP32C3: modelStr = "ESP32-C3"; break;
        case CHIP_ESP32H2: modelStr = "ESP32-H2"; break;
        // Note: CHIP_ESP32C6 was added in IDF 5.2; bundled SDK for S3 may lack it.
        // If a newer chip ID appears, the default branch catches it.
        default:           modelStr = "Unknown"; break;
    }
    char featuresStr[80];
    int flen = 0;
    if (info.features & CHIP_FEATURE_WIFI_BGN)    flen += snprintf(featuresStr + flen, sizeof(featuresStr) - flen, "WiFi%s", flen ? "," : "");
    if (info.features & CHIP_FEATURE_BT)          flen += snprintf(featuresStr + flen, sizeof(featuresStr) - flen, "%sBT", flen ? "," : "");
    if (info.features & CHIP_FEATURE_BLE)         flen += snprintf(featuresStr + flen, sizeof(featuresStr) - flen, "%sBLE", flen ? "," : "");
    if (info.features & CHIP_FEATURE_IEEE802154)  flen += snprintf(featuresStr + flen, sizeof(featuresStr) - flen, "%s802.15.4", flen ? "," : "");
    if (info.features & CHIP_FEATURE_EMB_FLASH)   flen += snprintf(featuresStr + flen, sizeof(featuresStr) - flen, "%sEmbedFlash", flen ? "," : "");
    if (featuresStr[0] == '\0') strcpy(featuresStr, "none");

    uint32_t flashBytes = 0;
    if (esp_flash_get_size(NULL, &flashBytes) != ESP_OK) flashBytes = 0;

    uint32_t cpuMhz = ESP.getCpuFreqMHz();
    uint32_t heapFree = esp_get_free_heap_size();
    uint32_t heapMinEver = esp_get_minimum_free_heap_size();
    uint32_t heapMin8bit = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
    bool heapIntOk = heap_caps_check_integrity_all(true);
    uint32_t randSample = esp_random();
    // MAC for display (already in NodeInfo but useful here)
    // Nexus port: dropped esp_efuse_mac_get_default() (was not declared in the SDK the
    // heltec-v4 build was using). MAC is already visible via other upstream reporting
    // (/nexus STATS shows it from owner.macaddr), so we don't need it here.
    char devIdHex[20] = "n/a (sdk missing)";

    // Body only: the "JT MCU Info:" title is supplied by startPaging() at
    // the call site. Don't repeat it here. See Commit 42.
    // Nexus port: dropped full_revision field (not in this build's esp_chip_info_t).
    // Operators still get model, cores, features, flash, IDF, CPU, heap, temp,
    // reset reason, MAC, entropy, uptime.
    snprintf(buf, bufSize,
        "  Platform:  %s (%d cores)\n"
        "  Features:  %s\n"
        "  Flash:     %lu MB\n"
        "  IDF:       %s\n"
        "  CPU:       %lu MHz\n"
        "  Heap:      %lu B free (min-ever %lu, min 8-bit-cap %lu%s)\n"
        "  DieTemp:   %.1fC\n"
        "  Boot:      %s\n"
        "  MAC:       %s\n"
        "  RNG:       0x%08lX\n"
        "  Up-time:   %s",
        modelStr,
        (int)info.cores,
        featuresStr,
        (unsigned long)(flashBytes ? (flashBytes / (1024 * 1024)) : 0),
        esp_get_idf_version(),
        (unsigned long)cpuMhz,
        (unsigned long)heapFree,
        (unsigned long)heapMinEver,
        (unsigned long)heapMin8bit,
        heapIntOk ? "" : ", integrity FAIL",
        (double)getDieTemp(),
        getResetReason(),
        devIdHex,
        (unsigned long)randSample,
        upStr);

#elif defined(ARCH_NRF52)
    // === nRF52 family ===
    // FICR chip identity (register reads, no error path)
    uint32_t partRaw = NRF_FICR->INFO.PART;
    uint32_t variantRaw = NRF_FICR->INFO.VARIANT;
    uint32_t packageRaw = NRF_FICR->INFO.PACKAGE;
    uint32_t ramKb = NRF_FICR->INFO.RAM;
    uint32_t flashKb = NRF_FICR->INFO.FLASH;

    // nRF52 part number encoding: upper word encodes family.
    // Examples: 0x52840 = nRF52840, 0x52833 = nRF52833, 0x52832 = nRF52832, 0x52820 = nRF52820, 0x52811 = nRF52811, 0x52810 = nRF52810
    char partStr[12];
    if (partRaw == 0xFFFFFFFFu) {
        snprintf(partStr, sizeof(partStr), "Unknown");
    } else {
        // Display as 'nRF52xx' (high byte + low nibble of high byte)
        snprintf(partStr, sizeof(partStr), "nRF52%02x",
                 (unsigned)((partRaw >> 8) & 0xFF));
    }

    // Variant code is 4 ASCII chars packed as LE32.
    char variantStr[5] = {0};
    if (variantRaw != 0xFFFFFFFFu && variantRaw != 0) {
        variantStr[0] = (char)(variantRaw & 0xFF);
        variantStr[1] = (char)((variantRaw >> 8) & 0xFF);
        variantStr[2] = (char)((variantRaw >> 16) & 0xFF);
        variantStr[3] = (char)((variantRaw >> 24) & 0xFF);
    }

    // Package code: most nRF52 boards report QCAA / QIAA / QFAB etc.
    char packageStr[5] = {0};
    if (packageRaw != 0xFFFFFFFFu && packageRaw != 0) {
        packageStr[0] = (char)(packageRaw & 0xFF);
        packageStr[1] = (char)((packageRaw >> 8) & 0xFF);
        packageStr[2] = (char)((packageRaw >> 16) & 0xFF);
        packageStr[3] = (char)((packageRaw >> 24) & 0xFF);
    }

    // SoftDevice info removed: sd_ble_version_get() and sd_softdevice_is_enabled()
    // are SoftDevice SVCALLs that don't link on tracker-t1000-e (no SoftDevice
    // in the BSP). Use Adafruit Bluefruit running mode instead if needed.

    // 64-bit device ID
    uint32_t devIdLo = NRF_FICR->DEVICEID[0];
    uint32_t devIdHi = NRF_FICR->DEVICEID[1];

    // UICR addresses (0xFFFFFFFF means unset)
    uint32_t blAddr = NRF_UICR->NRFFW[0];
    uint32_t mbrParam = NRF_UICR->NRFFW[1];

    uint32_t heapFree = memGet.getFreeHeap();
    // nRF52 has no platform-API equivalent of esp_get_minimum_free_heap_size().
    // memGet::getMinFreeBlock() returns the smallest free block (a fragmentation
    // metric, not a historical watermark). Show both: 'XKB free, largest block YKB'.
    uint32_t heapLargestBlock = memGet.getMinFreeBlock();

    int n = snprintf(buf, bufSize,
        "  Platform:  %s%s%s / %s / %lu KB RAM / %lu KB Flash\n"
        "  Heap:       %lu B free, largest block %lu B\n"
        "  DieTemp:    %.1fC\n"
        "  Boot:       %s\n"
        "  DeviceID:   !%08X %08X\n"
        "  UICR BL:    0x%08lX\n"
        "  UICR MBR:   0x%08lX\n"
        "  Up-time:    %s",
        partStr,
        variantStr[0] ? " " : "", variantStr[0] ? variantStr : "",
        packageStr[0] ? packageStr : "?",
        (unsigned long)ramKb,
        (unsigned long)flashKb,
        (unsigned long)heapFree,
        (unsigned long)heapLargestBlock,
        (double)getDieTemp(),
        getResetReason(),
        (unsigned)devIdHi,
        (unsigned)devIdLo,
        (unsigned long)blAddr,
        (unsigned long)mbrParam,
        upStr);
    (void)n;

#else
    // Portduino / RP2040 / STM32WL - limited info
    snprintf(buf, bufSize,
        "  Platform:  %s\n"
        "  Heap:      %lu B free, largest block %lu B\n"
        "  DieTemp:   %.1fC\n"
        "  Boot:      %s\n"
        "  Up-time:   %s",
        "host/board",
        (unsigned long)memGet.getFreeHeap(),
        (unsigned long)memGet.getMinFreeBlock(),
        (double)getDieTemp(),
        getResetReason(),
        upStr);
#endif
}

void NexusModule::i2cScanFinished(ScanI2C *i2cScanner)
{
    // Called once from main.cpp:939 after the boot I2C scan completes. The
    // scanner unique_ptr is still alive at this point; capture all known
    // devices into i2cScanResults before the scanner is destroyed.
    if (!i2cScanner) return;
    i2cScanResults.clear();

    // Iterate the canonical DeviceType enum. 'NONE' is the first value; the
    // rest are non-sentinel and may match. We do one find() per type, store
    // results, then exit.
    const ScanI2C::DeviceType allTypes[] = {
        ScanI2C::DeviceType::SCREEN_SSD1306,
        ScanI2C::DeviceType::SCREEN_SH1106,
        ScanI2C::DeviceType::SCREEN_UNKNOWN,
        ScanI2C::DeviceType::SCREEN_ST7567,
        ScanI2C::DeviceType::RTC_RV3028,
        ScanI2C::DeviceType::RTC_PCF8563,
        ScanI2C::DeviceType::RTC_PCF85063,
        ScanI2C::DeviceType::RTC_RX8130CE,
        ScanI2C::DeviceType::CARDKB,
        ScanI2C::DeviceType::TDECKKB,
        ScanI2C::DeviceType::BBQ10KB,
        ScanI2C::DeviceType::RAK14004,
        ScanI2C::DeviceType::PMU_AXP192_AXP2101,
        ScanI2C::DeviceType::BME_680,
        ScanI2C::DeviceType::BME_280,
        ScanI2C::DeviceType::BMP_280,
        ScanI2C::DeviceType::BMP_085,
        ScanI2C::DeviceType::BMP_3XX,
        ScanI2C::DeviceType::INA260,
        ScanI2C::DeviceType::INA219,
        ScanI2C::DeviceType::INA3221,
        ScanI2C::DeviceType::MAX17048,
        ScanI2C::DeviceType::MCP9808,
        ScanI2C::DeviceType::SHT31,
        ScanI2C::DeviceType::SHT4X,
        ScanI2C::DeviceType::SHTC3,
        ScanI2C::DeviceType::SHTXX,
        ScanI2C::DeviceType::LPS22HB,
        ScanI2C::DeviceType::QMC6310U,
        ScanI2C::DeviceType::QMC6310N,
        ScanI2C::DeviceType::QMI8658,
        ScanI2C::DeviceType::QMC5883L,
        ScanI2C::DeviceType::HMC5883L,
        ScanI2C::DeviceType::MMC5983MA,
        ScanI2C::DeviceType::PMSA003I,
        ScanI2C::DeviceType::QMA6100P,
        ScanI2C::DeviceType::MPU6050,
        ScanI2C::DeviceType::LIS3DH,
        ScanI2C::DeviceType::BMA423,
        ScanI2C::DeviceType::BQ24295,
        ScanI2C::DeviceType::LSM6DS3,
        ScanI2C::DeviceType::TCA9535,
        ScanI2C::DeviceType::TCA9555,
        ScanI2C::DeviceType::VEML7700,
        ScanI2C::DeviceType::RCWL9620,
        ScanI2C::DeviceType::NCP5623,
        ScanI2C::DeviceType::LP5562,
        ScanI2C::DeviceType::TSL2591,
        ScanI2C::DeviceType::OPT3001,
        ScanI2C::DeviceType::MLX90632,
        ScanI2C::DeviceType::MLX90614,
        ScanI2C::DeviceType::AHT10,
        ScanI2C::DeviceType::BMX160,
        ScanI2C::DeviceType::DFROBOT_LARK,
        ScanI2C::DeviceType::NAU7802,
        ScanI2C::DeviceType::FT6336U,
        ScanI2C::DeviceType::STK8BAXX,
        ScanI2C::DeviceType::ICM20948,
        ScanI2C::DeviceType::ICM42607P,
        ScanI2C::DeviceType::SCD4X,
        ScanI2C::DeviceType::MAX30102,
        ScanI2C::DeviceType::TPS65233,
        ScanI2C::DeviceType::MPR121KB,
        ScanI2C::DeviceType::CGRADSENS,
        ScanI2C::DeviceType::INA226,
        ScanI2C::DeviceType::NXP_SE050,
        ScanI2C::DeviceType::DFROBOT_RAIN,
        ScanI2C::DeviceType::DPS310,
        ScanI2C::DeviceType::LTR390UV,
        ScanI2C::DeviceType::RAK12035,
        ScanI2C::DeviceType::TCA8418KB,
        ScanI2C::DeviceType::PCT2075,
        ScanI2C::DeviceType::CST328,
        ScanI2C::DeviceType::BQ25896,
        ScanI2C::DeviceType::BQ27220,
        ScanI2C::DeviceType::LTR553ALS,
        ScanI2C::DeviceType::BHI260AP,
        ScanI2C::DeviceType::BMM150,
        ScanI2C::DeviceType::TSL2561,
        ScanI2C::DeviceType::DRV2605,
        ScanI2C::DeviceType::BH1750,
        ScanI2C::DeviceType::DA217,
        ScanI2C::DeviceType::CHSC6X,
        ScanI2C::DeviceType::CST226SE,
        ScanI2C::DeviceType::CST3530,
        ScanI2C::DeviceType::BMI270,
        ScanI2C::DeviceType::SEN5X,
        ScanI2C::DeviceType::SFA30,
        ScanI2C::DeviceType::CW2015,
        ScanI2C::DeviceType::SCD30,
        ScanI2C::DeviceType::ADS1X15,
    };

    for (auto t : allTypes) {
        ScanI2C::FoundDevice fd = i2cScanner->find(t);
        if (fd.type != ScanI2C::DeviceType::NONE) {
            i2cScanResults.push_back({t, fd.address});
        }
    }
    LOG_INFO("Nexus: SENSORS command has %u cached I2C devices", (unsigned)i2cScanResults.size());
}

static const char *nexusDeviceTypeName(ScanI2C::DeviceType t)
{
    switch (t) {
        case ScanI2C::DeviceType::SCREEN_SSD1306:    return "SSD1306 (OLED)";
        case ScanI2C::DeviceType::SCREEN_SH1106:     return "SH1106 (OLED)";
        case ScanI2C::DeviceType::SCREEN_UNKNOWN:    return "OLED (unknown)";
        case ScanI2C::DeviceType::SCREEN_ST7567:     return "ST7567 (LCD)";
        case ScanI2C::DeviceType::RTC_RV3028:        return "RV3028 (RTC)";
        case ScanI2C::DeviceType::RTC_PCF8563:       return "PCF8563 (RTC)";
        case ScanI2C::DeviceType::RTC_PCF85063:      return "PCF85063 (RTC)";
        case ScanI2C::DeviceType::RTC_RX8130CE:      return "RX8130CE (RTC)";
        case ScanI2C::DeviceType::CARDKB:            return "CardKB";
        case ScanI2C::DeviceType::TDECKKB:           return "T-Deck KB";
        case ScanI2C::DeviceType::BBQ10KB:           return "BBQ10 KB";
        case ScanI2C::DeviceType::RAK14004:          return "RAK14004 KB";
        case ScanI2C::DeviceType::MPR121KB:          return "MPR121 KB";
        case ScanI2C::DeviceType::TCA8418KB:         return "TCA8418 KB";
        case ScanI2C::DeviceType::PMU_AXP192_AXP2101: return "AXP192/2101 (PMU)";
        case ScanI2C::DeviceType::BME_680:           return "BME680";
        case ScanI2C::DeviceType::BME_280:           return "BME280";
        case ScanI2C::DeviceType::BMP_280:           return "BMP280";
        case ScanI2C::DeviceType::BMP_085:           return "BMP085";
        case ScanI2C::DeviceType::BMP_3XX:           return "BMP3XX";
        case ScanI2C::DeviceType::DPS310:            return "DPS310";
        case ScanI2C::DeviceType::LPS22HB:           return "LPS22HB";
        case ScanI2C::DeviceType::AHT10:             return "AHT10";
        case ScanI2C::DeviceType::SHT31:             return "SHT31";
        case ScanI2C::DeviceType::SHT4X:             return "SHT4X";
        case ScanI2C::DeviceType::SHTC3:             return "SHTC3";
        case ScanI2C::DeviceType::SHTXX:             return "SHTxx";
        case ScanI2C::DeviceType::MCP9808:           return "MCP9808";
        case ScanI2C::DeviceType::PCT2075:           return "PCT2075";
        case ScanI2C::DeviceType::INA219:            return "INA219";
        case ScanI2C::DeviceType::INA226:            return "INA226";
        case ScanI2C::DeviceType::INA260:            return "INA260";
        case ScanI2C::DeviceType::INA3221:           return "INA3221";
        case ScanI2C::DeviceType::MAX17048:          return "MAX17048";
        case ScanI2C::DeviceType::CW2015:            return "CW2015";
        case ScanI2C::DeviceType::BQ27220:           return "BQ27220";
        case ScanI2C::DeviceType::BQ25896:           return "BQ25896";
        case ScanI2C::DeviceType::BQ24295:           return "BQ24295";
        case ScanI2C::DeviceType::TPS65233:          return "TPS65233";
        case ScanI2C::DeviceType::PMSA003I:          return "PMSA003I";
        case ScanI2C::DeviceType::SEN5X:             return "SEN5X";
        case ScanI2C::DeviceType::SFA30:             return "SFA30";
        case ScanI2C::DeviceType::SCD4X:             return "SCD4X";
        case ScanI2C::DeviceType::SCD30:             return "SCD30";
        case ScanI2C::DeviceType::MLX90614:          return "MLX90614";
        case ScanI2C::DeviceType::MLX90632:          return "MLX90632";
        case ScanI2C::DeviceType::OPT3001:           return "OPT3001";
        case ScanI2C::DeviceType::VEML7700:          return "VEML7700";
        case ScanI2C::DeviceType::TSL2561:           return "TSL2561";
        case ScanI2C::DeviceType::TSL2591:           return "TSL2591";
        case ScanI2C::DeviceType::LTR390UV:          return "LTR390UV";
        case ScanI2C::DeviceType::LTR553ALS:         return "LTR553ALS";
        case ScanI2C::DeviceType::BH1750:            return "BH1750";
        case ScanI2C::DeviceType::MPU6050:           return "MPU6050";
        case ScanI2C::DeviceType::LIS3DH:            return "LIS3DH";
        case ScanI2C::DeviceType::BMA423:            return "BMA423";
        case ScanI2C::DeviceType::LSM6DS3:           return "LSM6DS3";
        case ScanI2C::DeviceType::BMX160:            return "BMX160";
        case ScanI2C::DeviceType::BMI270:            return "BMI270";
        case ScanI2C::DeviceType::ICM20948:          return "ICM20948";
        case ScanI2C::DeviceType::ICM42607P:         return "ICM42607P";
        case ScanI2C::DeviceType::BHI260AP:          return "BHI260AP";
        case ScanI2C::DeviceType::QMC5883L:          return "QMC5883L";
        case ScanI2C::DeviceType::QMC6310U:          return "QMC6310U";
        case ScanI2C::DeviceType::QMC6310N:          return "QMC6310N";
        case ScanI2C::DeviceType::QMI8658:           return "QMI8658";
        case ScanI2C::DeviceType::QMA6100P:          return "QMA6100P";
        case ScanI2C::DeviceType::HMC5883L:          return "HMC5883L";
        case ScanI2C::DeviceType::MMC5983MA:         return "MMC5983MA";
        case ScanI2C::DeviceType::BMM150:            return "BMM150";
        case ScanI2C::DeviceType::STK8BAXX:          return "STK8BAXX";
        case ScanI2C::DeviceType::RCWL9620:          return "RCWL9620 (radar)";
        case ScanI2C::DeviceType::CGRADSENS:         return "CGRadSens (rad)";
        case ScanI2C::DeviceType::NAU7802:           return "NAU7802 (ADC)";
        case ScanI2C::DeviceType::NCP5623:           return "NCP5623 (RGB)";
        case ScanI2C::DeviceType::LP5562:            return "LP5562 (RGB)";
        case ScanI2C::DeviceType::TCA9535:           return "TCA9535 (I/O)";
        case ScanI2C::DeviceType::TCA9555:           return "TCA9555 (I/O)";
        case ScanI2C::DeviceType::DFROBOT_LARK:      return "DFRobot Lark";
        case ScanI2C::DeviceType::DFROBOT_RAIN:      return "DFRobot Rain";
        case ScanI2C::DeviceType::RAK12035:          return "RAK12035 (rain)";
        case ScanI2C::DeviceType::MAX30102:          return "MAX30102 (HR)";
        case ScanI2C::DeviceType::FT6336U:           return "FT6336U (touch)";
        case ScanI2C::DeviceType::CST328:            return "CST328 (touch)";
        case ScanI2C::DeviceType::CST226SE:          return "CST226SE (touch)";
        case ScanI2C::DeviceType::CST3530:           return "CST3530 (touch)";
        case ScanI2C::DeviceType::CHSC6X:            return "CHSC6X (touch)";
        case ScanI2C::DeviceType::DA217:             return "DA217 (touch)";
        case ScanI2C::DeviceType::DRV2605:           return "DRV2605 (haptic)";
        case ScanI2C::DeviceType::ADS1X15:           return "ADS1x15 (ADC, includes ADS1115)";
        case ScanI2C::DeviceType::NXP_SE050:         return "NXP SE050 (secure)";
        default:                                     return "Unknown";
    }
}

void NexusModule::formatSensorsInfo(char *buf, size_t bufSize)
{
    if (!buf || bufSize == 0) return;
    buf[0] = '\0';

    int n = 0; // bytes written

    // I2C count line (the body). The "JT SENSORS:" title is supplied by
    // startPaging() at the call site; we don't repeat it here to avoid a
    // duplicated header on the wire. See Commit 41.
    n += snprintf(buf + n, (n < (int)bufSize) ? bufSize - n : 0,
        "  I2C (%u detected):\n",
        (unsigned)i2cScanResults.size());

    // One line per detected device, formatted as '  - <name>  @0x<addr> (<port>)'
    for (const auto &d : i2cScanResults) {
        const char *port = (d.address.port == ScanI2C::I2CPort::WIRE1) ? "WIRE1" :
                           (d.address.port == ScanI2C::I2CPort::WIRE)  ? "WIRE"  : "?";
        n += snprintf(buf + n, (n < (int)bufSize) ? bufSize - n : 0,
            "  - %-22s @0x%02X (%s)\n", nexusDeviceTypeName(d.type), d.address.address, port);
    }

    // Live power telemetry
    meshtastic_Telemetry pm = meshtastic_Telemetry_init_zero;
    if (meshtastic::getNexusPowerTelemetry(&pm) && pm.which_variant == meshtastic_Telemetry_power_metrics_tag) {
        n += snprintf(buf + n, (n < (int)bufSize) ? bufSize - n : 0, "  Power (live): ");
        auto &p = pm.variant.power_metrics;
        if (p.has_ch1_voltage) n += snprintf(buf + n, (n < (int)bufSize) ? bufSize - n : 0, "V1:%.2fV ", (double)p.ch1_voltage);
        if (p.has_ch1_current) n += snprintf(buf + n, (n < (int)bufSize) ? bufSize - n : 0, "I1:%.0fmA ", (double)p.ch1_current);
        if (p.has_ch2_voltage) n += snprintf(buf + n, (n < (int)bufSize) ? bufSize - n : 0, "V2:%.2fV ", (double)p.ch2_voltage);
        if (p.has_ch2_current) n += snprintf(buf + n, (n < (int)bufSize) ? bufSize - n : 0, "I2:%.0fmA ", (double)p.ch2_current);
        if (p.has_ch3_voltage) n += snprintf(buf + n, (n < (int)bufSize) ? bufSize - n : 0, "V3:%.2fV ", (double)p.ch3_voltage);
        if (p.has_ch3_current) n += snprintf(buf + n, (n < (int)bufSize) ? bufSize - n : 0, "I3:%.0fmA ", (double)p.ch3_current);
        n += snprintf(buf + n, (n < (int)bufSize) ? bufSize - n : 0, "\n");
    } else {
        n += snprintf(buf + n, (n < (int)bufSize) ? bufSize - n : 0, "  Power (live): (none)\n");
    }

    // Live environment telemetry
    meshtastic_Telemetry em = meshtastic_Telemetry_init_zero;
    if (meshtastic::getNexusEnvironmentTelemetry(&em) && em.which_variant == meshtastic_Telemetry_environment_metrics_tag) {
        auto &e = em.variant.environment_metrics;
        n += snprintf(buf + n, (n < (int)bufSize) ? bufSize - n : 0,
            "  Env (live):   T:%.1fC H:%.0f%% P:%.0fhPa",
            (double)e.temperature, (double)e.relative_humidity, (double)e.barometric_pressure);
        if (e.has_lux)   n += snprintf(buf + n, (n < (int)bufSize) ? bufSize - n : 0, " Lux:%.0f", (double)e.lux);
        if (e.has_uv_lux) n += snprintf(buf + n, (n < (int)bufSize) ? bufSize - n : 0, " UV:%.0f", (double)e.uv_lux);
        n += snprintf(buf + n, (n < (int)bufSize) ? bufSize - n : 0, "\n");
    } else {
        n += snprintf(buf + n, (n < (int)bufSize) ? bufSize - n : 0, "  Env (live):   (none)\n");
    }

    // GPS status (uses global gpsStatus + gps instance)
    if (gpsStatus && gps && gps->isConnected()) {
        n += snprintf(buf + n, (n < (int)bufSize) ? bufSize - n : 0,
            "  GPS:          lock=%s sats=%u PDOP=%.2f",
            gpsStatus->getHasLock() ? "yes" : "no",
            (unsigned)gpsStatus->getNumSatellites(),
            (double)gpsStatus->getDOP() * 0.01);
        if (gpsStatus->getHasLock()) {
            int32_t lat = gpsStatus->getLatitude();
            int32_t lon = gpsStatus->getLongitude();
            int32_t alt = gpsStatus->getAltitude();
            n += snprintf(buf + n, (n < (int)bufSize) ? bufSize - n : 0,
                " lat=%d.%07d lon=%d.%07d alt=%d",
                (int)(lat / 10000000), abs(lat % 10000000),
                (int)(lon / 10000000), abs(lon % 10000000),
                (int)alt);
        }
        n += snprintf(buf + n, (n < (int)bufSize) ? bufSize - n : 0, "\n");
    } else {
        n += snprintf(buf + n, (n < (int)bufSize) ? bufSize - n : 0, "  GPS:          (none)\n");
    }
    (void)n;
}


void NexusModule::formatRLBuckets(char *buf, size_t bufSize)
{
    if (!buf || bufSize == 0) return;
    buf[0] = '\0';
    int n = 0;

    // Bucket table: 0:TX 1:TR 2:OT 3:NI 4:PO 5:TL 6:PR
    // Each row: <CODE>: <TOK>/<LIM> (<PCT>%) [ENABLED?] <seen> <pass> <drop> <drop%>%
    //   TOK = current float tokens (truncated to 1 decimal in the printed form,
    //         clamped to >=0 because refill could overshoot if a peer burst
    //         happens during a Time::getMillis() rollover)
    //   LIM = configured pkts/hr limit (0 = bucket off, no enforcement)
    //   PCT = percent of limit remaining (0 if limit=0; clamps to 0-999)
    //   ENABLED = whether drop_rl_<bucket> is true (only matters if LIM > 0)
    //   seen/pass/drop = lifetime counters from checkRateLimit()
    //   drop% = drop/seen*100 (0 if seen=0)
    //
    // Pairs with the /nexus RATELIMITSTAT (alias RLS) command. Output is sized
    // for 1 paging part in the common case (~470 bytes worst case at 6-digit
    // counts; appendPaging splits automatically if a future schema grows).
    const char *names[7]   = {"TX", "TR", "OT", "NI", "PO", "TL", "PR"};
    const uint16_t lims[7] = {rl_text_limit, rl_trace_limit, rl_other_limit,
                              rl_nodeinfo_limit, rl_position_limit,
                              rl_telemetry_limit, rl_private_limit};
    const bool enabled[7]  = {drop_rl_text, drop_rl_trace, drop_rl_other,
                              drop_rl_nodeinfo, drop_rl_position,
                              drop_rl_telemetry, drop_rl_private};
    const uint32_t seen[7] = {rl_seen_text, rl_seen_trace, rl_seen_other,
                              rl_seen_nodeinfo, rl_seen_position,
                              rl_seen_telemetry, rl_seen_private};
    const uint32_t pass[7] = {rl_pass_text, rl_pass_trace, rl_pass_other,
                              rl_pass_nodeinfo, rl_pass_position,
                              rl_pass_telemetry, rl_pass_private};
    const uint32_t drop[7] = {rl_drop_text, rl_drop_trace, rl_drop_other,
                              rl_drop_nodeinfo, rl_drop_position,
                              rl_drop_telemetry, rl_drop_private};

    // Header
    n += snprintf(buf + n, (n < (int)bufSize) ? bufSize - n : 0,
        "JT RL Buckets (live tokens / limit, since-boot counters):\n"
        " ID  Tokens/Limit  %%Full  En?  Seen  Pass  Drop  Drop%%\n");

    for (int i = 0; i < 7; i++) {
        float tok = rl_tokens[i];
        if (tok < 0.0f) tok = 0.0f;  // clamp
        int pct = 0;
        if (lims[i] > 0) {
            pct = (int)((tok * 100.0f) / (float)lims[i]);
            if (pct < 0) pct = 0;
            if (pct > 999) pct = 999;  // 3-digit cap so the column never wraps
        }
        int dropPct = 0;
        if (seen[i] > 0) {
            dropPct = (int)((drop[i] * 100.0f) / (float)seen[i]);
            if (dropPct > 100) dropPct = 100;
        }
        n += snprintf(buf + n, (n < (int)bufSize) ? bufSize - n : 0,
            " %s:  %5.1f/%-5u  %3d%%  %s  %5u %5u %5u  %3d%%\n",
            names[i],
            (double)tok, (unsigned)lims[i],
            pct,
            enabled[i] ? "Y" : "N",
            (unsigned)seen[i], (unsigned)pass[i], (unsigned)drop[i],
            dropPct);
    }
    (void)n;
}

// =========================================================================
// 17 appendFCDSection_* methods - Commit 44 + Commit 45 (refactor: wrote
// directly into paging_buf instead of a 1KB static accumulator).
// =========================================================================
// 17 sections of Meshtastic config, dumped field-by-field with human-readable
// enum names. Excludes Security (public/private/admin keys + secrets) and
// Channels (already exposed by /nexus CHANNELS). Each section is bracketed
// by `JT Full Config: <NAME>\n--\n` header and `--\n` footer so the paging
// output stays readable.
//
// Pattern (Commit 45): each appendFCDSection_*() writes its lines directly
// into the paging buffer via appendPaging(). No static accumulator buffer is
// used: saves ~1 KB of BSS vs the Commit 44 formatFullConfigDump() which used
// a 1024-byte static char[]. Total output is ~6 KB which produces ~30-40
// paging parts at the 179-byte body cap.

static const char *roleName(meshtastic_Config_DeviceConfig_Role r) {
    switch (r) {
        case meshtastic_Config_DeviceConfig_Role_CLIENT: return "CLIENT";
        case meshtastic_Config_DeviceConfig_Role_CLIENT_MUTE: return "CLIENT_MUTE";
        case meshtastic_Config_DeviceConfig_Role_ROUTER: return "ROUTER";
        case meshtastic_Config_DeviceConfig_Role_ROUTER_CLIENT: return "ROUTER_CLIENT";
        case meshtastic_Config_DeviceConfig_Role_REPEATER: return "REPEATER";
        case meshtastic_Config_DeviceConfig_Role_TRACKER: return "TRACKER";
        case meshtastic_Config_DeviceConfig_Role_SENSOR: return "SENSOR";
        case meshtastic_Config_DeviceConfig_Role_TAK: return "TAK";
        case meshtastic_Config_DeviceConfig_Role_CLIENT_HIDDEN: return "CLIENT_HIDDEN";
        case meshtastic_Config_DeviceConfig_Role_LOST_AND_FOUND: return "LOST_AND_FOUND";
        case meshtastic_Config_DeviceConfig_Role_TAK_TRACKER: return "TAK_TRACKER";
        case meshtastic_Config_DeviceConfig_Role_ROUTER_LATE: return "ROUTER_LATE";
        case meshtastic_Config_DeviceConfig_Role_CLIENT_BASE: return "CLIENT_BASE";
        default: return "UNKNOWN";
    }
}

static const char *rebroadcastName(meshtastic_Config_DeviceConfig_RebroadcastMode m) {
    switch (m) {
        case meshtastic_Config_DeviceConfig_RebroadcastMode_ALL: return "ALL";
        case meshtastic_Config_DeviceConfig_RebroadcastMode_ALL_SKIP_DECODING: return "ALL_SKIP_DECODING";
        case meshtastic_Config_DeviceConfig_RebroadcastMode_LOCAL_ONLY: return "LOCAL_ONLY";
        case meshtastic_Config_DeviceConfig_RebroadcastMode_KNOWN_ONLY: return "KNOWN_ONLY";
        default: return "UNKNOWN";
    }
}

static const char *buzzerModeName(meshtastic_Config_DeviceConfig_BuzzerMode m) {
    switch (m) {
        case meshtastic_Config_DeviceConfig_BuzzerMode_ALL_ENABLED: return "ALL_ENABLED";
        case meshtastic_Config_DeviceConfig_BuzzerMode_DISABLED: return "DISABLED";
        case meshtastic_Config_DeviceConfig_BuzzerMode_NOTIFICATIONS_ONLY: return "NOTIFICATIONS_ONLY";
        case meshtastic_Config_DeviceConfig_BuzzerMode_SYSTEM_ONLY: return "SYSTEM_ONLY";
        case meshtastic_Config_DeviceConfig_BuzzerMode_DIRECT_MSG_ONLY: return "DIRECT_MSG_ONLY";
        default: return "UNKNOWN";
    }
}

static const char *gpsModeName(meshtastic_Config_PositionConfig_GpsMode m) {
    switch (m) {
        case meshtastic_Config_PositionConfig_GpsMode_ENABLED: return "ENABLED";
        case meshtastic_Config_PositionConfig_GpsMode_DISABLED: return "DISABLED";
        case meshtastic_Config_PositionConfig_GpsMode_NOT_PRESENT: return "NOT_PRESENT";
        default: return "UNKNOWN";
    }
}

static const char *regionName(meshtastic_Config_LoRaConfig_RegionCode r) {
    switch (r) {
        case meshtastic_Config_LoRaConfig_RegionCode_UNSET: return "UNSET";
        case meshtastic_Config_LoRaConfig_RegionCode_US: return "US";
        case meshtastic_Config_LoRaConfig_RegionCode_EU_433: return "EU_433";
        case meshtastic_Config_LoRaConfig_RegionCode_EU_868: return "EU_868";
        case meshtastic_Config_LoRaConfig_RegionCode_CN: return "CN";
        case meshtastic_Config_LoRaConfig_RegionCode_JP: return "JP";
        case meshtastic_Config_LoRaConfig_RegionCode_ANZ: return "ANZ";
        case meshtastic_Config_LoRaConfig_RegionCode_KR: return "KR";
        case meshtastic_Config_LoRaConfig_RegionCode_TW: return "TW";
        case meshtastic_Config_LoRaConfig_RegionCode_RU: return "RU";
        case meshtastic_Config_LoRaConfig_RegionCode_IN: return "IN";
        case meshtastic_Config_LoRaConfig_RegionCode_NZ_865: return "NZ_865";
        case meshtastic_Config_LoRaConfig_RegionCode_TH: return "TH";
        case meshtastic_Config_LoRaConfig_RegionCode_LORA_24: return "LORA_24";
        case meshtastic_Config_LoRaConfig_RegionCode_UA_433: return "UA_433";
        case meshtastic_Config_LoRaConfig_RegionCode_UA_868: return "UA_868";
        case meshtastic_Config_LoRaConfig_RegionCode_MY_433: return "MY_433";
        case meshtastic_Config_LoRaConfig_RegionCode_MY_919: return "MY_919";
        case meshtastic_Config_LoRaConfig_RegionCode_SG_923: return "SG_923";
        case meshtastic_Config_LoRaConfig_RegionCode_PH_433: return "PH_433";
        case meshtastic_Config_LoRaConfig_RegionCode_PH_868: return "PH_868";
        case meshtastic_Config_LoRaConfig_RegionCode_PH_915: return "PH_915";
        case meshtastic_Config_LoRaConfig_RegionCode_ANZ_433: return "ANZ_433";
        case meshtastic_Config_LoRaConfig_RegionCode_KZ_433: return "KZ_433";
        case meshtastic_Config_LoRaConfig_RegionCode_KZ_863: return "KZ_863";
        case meshtastic_Config_LoRaConfig_RegionCode_NP_865: return "NP_865";
        case meshtastic_Config_LoRaConfig_RegionCode_BR_902: return "BR_902";
        case meshtastic_Config_LoRaConfig_RegionCode_ITU1_2M: return "ITU1_2M";
        case meshtastic_Config_LoRaConfig_RegionCode_ITU2_2M: return "ITU2_2M";
        case meshtastic_Config_LoRaConfig_RegionCode_EU_866: return "EU_866";
        case meshtastic_Config_LoRaConfig_RegionCode_EU_874: return "EU_874";
        case meshtastic_Config_LoRaConfig_RegionCode_EU_917: return "EU_917";
        case meshtastic_Config_LoRaConfig_RegionCode_EU_N_868: return "EU_N_868";
        case meshtastic_Config_LoRaConfig_RegionCode_ITU3_2M: return "ITU3_2M";
        default: return "UNKNOWN";
    }
}

static const char *modemPresetName(meshtastic_Config_LoRaConfig_ModemPreset p) {
    switch (p) {
        case meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST: return "LONG_FAST";
        case meshtastic_Config_LoRaConfig_ModemPreset_LONG_SLOW: return "LONG_SLOW";
        case meshtastic_Config_LoRaConfig_ModemPreset_VERY_LONG_SLOW: return "VERY_LONG_SLOW";
        case meshtastic_Config_LoRaConfig_ModemPreset_MEDIUM_SLOW: return "MEDIUM_SLOW";
        case meshtastic_Config_LoRaConfig_ModemPreset_MEDIUM_FAST: return "MEDIUM_FAST";
        case meshtastic_Config_LoRaConfig_ModemPreset_SHORT_SLOW: return "SHORT_SLOW";
        case meshtastic_Config_LoRaConfig_ModemPreset_SHORT_FAST: return "SHORT_FAST";
        case meshtastic_Config_LoRaConfig_ModemPreset_LONG_MODERATE: return "LONG_MODERATE";
        case meshtastic_Config_LoRaConfig_ModemPreset_SHORT_TURBO: return "SHORT_TURBO";
        case meshtastic_Config_LoRaConfig_ModemPreset_LONG_TURBO: return "LONG_TURBO";
        case meshtastic_Config_LoRaConfig_ModemPreset_LITE_FAST: return "LITE_FAST";
        case meshtastic_Config_LoRaConfig_ModemPreset_LITE_SLOW: return "LITE_SLOW";
        case meshtastic_Config_LoRaConfig_ModemPreset_NARROW_FAST: return "NARROW_FAST";
        case meshtastic_Config_LoRaConfig_ModemPreset_NARROW_SLOW: return "NARROW_SLOW";
        default: return "UNKNOWN";
    }
}

static const char *femModeName(meshtastic_Config_LoRaConfig_FEM_LNA_Mode m) {
    switch (m) {
        case meshtastic_Config_LoRaConfig_FEM_LNA_Mode_DISABLED: return "DISABLED";
        case meshtastic_Config_LoRaConfig_FEM_LNA_Mode_ENABLED: return "ENABLED";
        case meshtastic_Config_LoRaConfig_FEM_LNA_Mode_NOT_PRESENT: return "NOT_PRESENT";
        default: return "UNKNOWN";
    }
}

static const char *addressModeName(meshtastic_Config_NetworkConfig_AddressMode m) {
    switch (m) {
        case meshtastic_Config_NetworkConfig_AddressMode_DHCP: return "DHCP";
        case meshtastic_Config_NetworkConfig_AddressMode_STATIC: return "STATIC";
        default: return "UNKNOWN";
    }
}

static const char *displayModeName(meshtastic_Config_DisplayConfig_DisplayMode m) {
    switch (m) {
        case meshtastic_Config_DisplayConfig_DisplayMode_DEFAULT: return "DEFAULT";
        case meshtastic_Config_DisplayConfig_DisplayMode_TWOCOLOR: return "TWOCOLOR";
        case meshtastic_Config_DisplayConfig_DisplayMode_INVERTED: return "INVERTED";
        case meshtastic_Config_DisplayConfig_DisplayMode_COLOR: return "COLOR";
        default: return "UNKNOWN";
    }
}

static const char *oledTypeName(meshtastic_Config_DisplayConfig_OledType t) {
    switch (t) {
        case meshtastic_Config_DisplayConfig_OledType_OLED_AUTO: return "OLED_AUTO";
        case meshtastic_Config_DisplayConfig_OledType_OLED_SSD1306: return "OLED_SSD1306";
        case meshtastic_Config_DisplayConfig_OledType_OLED_SH1106: return "OLED_SH1106";
        case meshtastic_Config_DisplayConfig_OledType_OLED_SH1107: return "OLED_SH1107";
        case meshtastic_Config_DisplayConfig_OledType_OLED_SH1107_128_128: return "OLED_SH1107_128_128";
        case meshtastic_Config_DisplayConfig_OledType_OLED_SH1107_ROTATED: return "OLED_SH1107_ROTATED";
        default: return "UNKNOWN";
    }
}

static const char *unitsName(meshtastic_Config_DisplayConfig_DisplayUnits u) {
    switch (u) {
        case meshtastic_Config_DisplayConfig_DisplayUnits_METRIC: return "METRIC";
        case meshtastic_Config_DisplayConfig_DisplayUnits_IMPERIAL: return "IMPERIAL";
        default: return "UNKNOWN";
    }
}

static const char *pairingModeName(meshtastic_Config_BluetoothConfig_PairingMode p) {
    switch (p) {
        case meshtastic_Config_BluetoothConfig_PairingMode_RANDOM_PIN: return "RANDOM_PIN";
        case meshtastic_Config_BluetoothConfig_PairingMode_FIXED_PIN: return "FIXED_PIN";
        case meshtastic_Config_BluetoothConfig_PairingMode_NO_PIN: return "NO_PIN";
        default: return "UNKNOWN";
    }
}

static const char *serialBaudName(meshtastic_ModuleConfig_SerialConfig_Serial_Baud b) {
    switch (b) {
        case meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_DEFAULT: return "DEFAULT";
        case meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_110: return "110";
        case meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_300: return "300";
        case meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_600: return "600";
        case meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_1200: return "1200";
        case meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_2400: return "2400";
        case meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_4800: return "4800";
        case meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_9600: return "9600";
        case meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_19200: return "19200";
        case meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_38400: return "38400";
        case meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_57600: return "57600";
        case meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_115200: return "115200";
        case meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_230400: return "230400";
        case meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_460800: return "460800";
        case meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_576000: return "576000";
        case meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_921600: return "921600";
        default: return "UNKNOWN";
    }
}

static const char *serialModeName(meshtastic_ModuleConfig_SerialConfig_Serial_Mode m) {
    switch (m) {
        case meshtastic_ModuleConfig_SerialConfig_Serial_Mode_DEFAULT: return "DEFAULT";
        case meshtastic_ModuleConfig_SerialConfig_Serial_Mode_SIMPLE: return "SIMPLE";
        case meshtastic_ModuleConfig_SerialConfig_Serial_Mode_PROTO: return "PROTO";
        case meshtastic_ModuleConfig_SerialConfig_Serial_Mode_TEXTMSG: return "TEXTMSG";
        case meshtastic_ModuleConfig_SerialConfig_Serial_Mode_NMEA: return "NMEA";
        case meshtastic_ModuleConfig_SerialConfig_Serial_Mode_CALTOPO: return "CALTOPO";
        case meshtastic_ModuleConfig_SerialConfig_Serial_Mode_WS85: return "WS85";
        case meshtastic_ModuleConfig_SerialConfig_Serial_Mode_VE_DIRECT: return "VE_DIRECT";
        default: return "UNKNOWN";
    }
}

static const char *compassOrientName(meshtastic_Config_DisplayConfig_CompassOrientation o) {
    switch (o) {
        case meshtastic_Config_DisplayConfig_CompassOrientation_DEGREES_0: return "DEGREES_0";
        case meshtastic_Config_DisplayConfig_CompassOrientation_DEGREES_90: return "DEGREES_90";
        case meshtastic_Config_DisplayConfig_CompassOrientation_DEGREES_180: return "DEGREES_180";
        case meshtastic_Config_DisplayConfig_CompassOrientation_DEGREES_270: return "DEGREES_270";
        case meshtastic_Config_DisplayConfig_CompassOrientation_DEGREES_0_INVERTED: return "DEGREES_0_INVERTED";
        case meshtastic_Config_DisplayConfig_CompassOrientation_DEGREES_90_INVERTED: return "DEGREES_90_INVERTED";
        case meshtastic_Config_DisplayConfig_CompassOrientation_DEGREES_180_INVERTED: return "DEGREES_180_INVERTED";
        case meshtastic_Config_DisplayConfig_CompassOrientation_DEGREES_270_INVERTED: return "DEGREES_270_INVERTED";
        default: return "UNKNOWN";
    }
}

// FCD_LINE: format a single line into a stack buffer, then push it into
// the live paging buffer via appendPaging(). The macro is #undef-ed at the
// end of each appendFCDSection_*() method to keep the namespace clean and
// avoid leaking into unrelated code below.
//
// Why per-method: the paging buffer (paging_buf[2048]) is owned by the
// OSThread runOnce loop. We want FCD output to flow through it directly
// so we never need a separate static accumulator (saves ~1 KB of BSS).
// Each section helper is called serially from the dispatch site, writes
// one line at a time into paging_buf, and returns. runOnce drains
// paging_buf into mesh packets on its next tick (2s later).

void NexusModule::appendFCDSection_DEVICE()
{
    auto &d = config.device;
    #define FCD_LINE(...) do { char line[160]; snprintf(line, sizeof(line), __VA_ARGS__); appendPaging(line); } while(0)
    appendPaging("JT Full Config: DEVICE\n--\n");
    FCD_LINE("role: %s\n", roleName(d.role));
    FCD_LINE("serial_enabled: %s\n", d.serial_enabled ? "true" : "false");
    FCD_LINE("button_gpio: %u\n", (unsigned)d.button_gpio);
    FCD_LINE("buzzer_gpio: %u\n", (unsigned)d.buzzer_gpio);
    FCD_LINE("rebroadcast_mode: %s\n", rebroadcastName(d.rebroadcast_mode));
    FCD_LINE("node_info_broadcast_secs: %u\n", (unsigned)d.node_info_broadcast_secs);
    FCD_LINE("double_tap_as_button_press: %s\n", d.double_tap_as_button_press ? "true" : "false");
    FCD_LINE("is_managed: %s\n", d.is_managed ? "true" : "false");
    FCD_LINE("disable_triple_click: %s\n", d.disable_triple_click ? "true" : "false");
    FCD_LINE("tzdef: %s\n", d.tzdef);
    FCD_LINE("led_heartbeat_disabled: %s\n", d.led_heartbeat_disabled ? "true" : "false");
    FCD_LINE("buzzer_mode: %s\n", buzzerModeName(d.buzzer_mode));
    appendPaging("--\n");
    #undef FCD_LINE
}

void NexusModule::appendFCDSection_POSITION()
{
    auto &p = config.position;
    #define FCD_LINE(...) do { char line[160]; snprintf(line, sizeof(line), __VA_ARGS__); appendPaging(line); } while(0)
    appendPaging("JT Full Config: POSITION\n--\n");
    FCD_LINE("position_broadcast_secs: %u\n", (unsigned)p.position_broadcast_secs);
    FCD_LINE("position_broadcast_smart_enabled: %s\n", p.position_broadcast_smart_enabled ? "true" : "false");
    FCD_LINE("fixed_position: %s\n", p.fixed_position ? "true" : "false");
    FCD_LINE("gps_enabled: %s\n", p.gps_enabled ? "true" : "false");
    FCD_LINE("gps_update_interval: %u\n", (unsigned)p.gps_update_interval);
    FCD_LINE("gps_attempt_time: %u\n", (unsigned)p.gps_attempt_time);
    // position_flags is a bitfield. Expand known flags into a | separated
    // string. Up to ~100 chars, fits in line[160].
    {
        uint32_t f = p.position_flags;
        char flags[128];
        flags[0] = '\0';
        if (f) {
            #define APPEND_FLAG(bit, name) do { if (f & (1u << bit)) { if (flags[0]) strncat(flags, "|", sizeof(flags)-strlen(flags)-1); strncat(flags, name, sizeof(flags)-strlen(flags)-1); } } while(0)
            APPEND_FLAG(0, "ALTITUDE");
            APPEND_FLAG(1, "ALTITUDE_METERS");
            APPEND_FLAG(2, "GEOIDAL_SEPARATION");
            APPEND_FLAG(3, "DOP");
            APPEND_FLAG(4, "HEADING");
            APPEND_FLAG(5, "HEADING_DEG");
            APPEND_FLAG(6, "SATELLITES");
            APPEND_FLAG(7, "SEQUENCE_NUMBER");
            APPEND_FLAG(8, "TIMESTAMP");
            #undef APPEND_FLAG
        }
        FCD_LINE("position_flags: 0x%08x %s\n", (unsigned)f, flags[0] ? flags : "(none)");
    }
    FCD_LINE("rx_gpio: %u\n", (unsigned)p.rx_gpio);
    FCD_LINE("tx_gpio: %u\n", (unsigned)p.tx_gpio);
    FCD_LINE("broadcast_smart_minimum_distance: %u\n", (unsigned)p.broadcast_smart_minimum_distance);
    FCD_LINE("broadcast_smart_minimum_interval_secs: %u\n", (unsigned)p.broadcast_smart_minimum_interval_secs);
    FCD_LINE("gps_en_gpio: %u\n", (unsigned)p.gps_en_gpio);
    FCD_LINE("gps_mode: %s\n", gpsModeName(p.gps_mode));
    appendPaging("--\n");
    #undef FCD_LINE
}

void NexusModule::appendFCDSection_POWER()
{
    auto &pw = config.power;
    #define FCD_LINE(...) do { char line[160]; snprintf(line, sizeof(line), __VA_ARGS__); appendPaging(line); } while(0)
    appendPaging("JT Full Config: POWER\n--\n");
    FCD_LINE("is_power_saving: %s\n", pw.is_power_saving ? "true" : "false");
    FCD_LINE("on_battery_shutdown_after_secs: %u\n", (unsigned)pw.on_battery_shutdown_after_secs);
    FCD_LINE("adc_multiplier_override: %.3f\n", (double)pw.adc_multiplier_override);
    FCD_LINE("wait_bluetooth_secs: %u\n", (unsigned)pw.wait_bluetooth_secs);
    FCD_LINE("sds_secs: %u\n", (unsigned)pw.sds_secs);
    FCD_LINE("ls_secs: %u\n", (unsigned)pw.ls_secs);
    FCD_LINE("min_wake_secs: %u\n", (unsigned)pw.min_wake_secs);
    FCD_LINE("device_battery_ina_address: 0x%02X\n", (unsigned)pw.device_battery_ina_address);
    FCD_LINE("powermon_enables: 0x%016llx\n", (unsigned long long)pw.powermon_enables);
    appendPaging("--\n");
    #undef FCD_LINE
}

void NexusModule::appendFCDSection_NETWORK()
{
    auto &nw = config.network;
    #define FCD_LINE(...) do { char line[160]; snprintf(line, sizeof(line), __VA_ARGS__); appendPaging(line); } while(0)
    appendPaging("JT Full Config: NETWORK\n--\n");
    FCD_LINE("wifi_enabled: %s\n", nw.wifi_enabled ? "true" : "false");
    FCD_LINE("wifi_ssid: %s\n", nw.wifi_ssid);
    appendPaging("wifi_psk: ***\n");
    FCD_LINE("ntp_server: %s\n", nw.ntp_server);
    FCD_LINE("eth_enabled: %s\n", nw.eth_enabled ? "true" : "false");
    FCD_LINE("address_mode: %s\n", addressModeName(nw.address_mode));
    FCD_LINE("ipv4_config.ip: %u.%u.%u.%u\n",
        (unsigned)(nw.ipv4_config.ip & 0xFF), (unsigned)((nw.ipv4_config.ip >> 8) & 0xFF),
        (unsigned)((nw.ipv4_config.ip >> 16) & 0xFF), (unsigned)((nw.ipv4_config.ip >> 24) & 0xFF));
    FCD_LINE("ipv4_config.gateway: %u.%u.%u.%u\n",
        (unsigned)(nw.ipv4_config.gateway & 0xFF), (unsigned)((nw.ipv4_config.gateway >> 8) & 0xFF),
        (unsigned)((nw.ipv4_config.gateway >> 16) & 0xFF), (unsigned)((nw.ipv4_config.gateway >> 24) & 0xFF));
    FCD_LINE("ipv4_config.subnet: %u.%u.%u.%u\n",
        (unsigned)(nw.ipv4_config.subnet & 0xFF), (unsigned)((nw.ipv4_config.subnet >> 8) & 0xFF),
        (unsigned)((nw.ipv4_config.subnet >> 16) & 0xFF), (unsigned)((nw.ipv4_config.subnet >> 24) & 0xFF));
    FCD_LINE("ipv4_config.dns: %u.%u.%u.%u\n",
        (unsigned)(nw.ipv4_config.dns & 0xFF), (unsigned)((nw.ipv4_config.dns >> 8) & 0xFF),
        (unsigned)((nw.ipv4_config.dns >> 16) & 0xFF), (unsigned)((nw.ipv4_config.dns >> 24) & 0xFF));
    FCD_LINE("rsyslog_server: %s\n", nw.rsyslog_server);
    FCD_LINE("enabled_protocols: 0x%08x\n", (unsigned)nw.enabled_protocols);
    FCD_LINE("ipv6_enabled: %s\n", nw.ipv6_enabled ? "true" : "false");
    appendPaging("--\n");
    #undef FCD_LINE
}

void NexusModule::appendFCDSection_DISPLAY()
{
    auto &dp = config.display;
    #define FCD_LINE(...) do { char line[160]; snprintf(line, sizeof(line), __VA_ARGS__); appendPaging(line); } while(0)
    appendPaging("JT Full Config: DISPLAY\n--\n");
    FCD_LINE("screen_on_secs: %u\n", (unsigned)dp.screen_on_secs);
    FCD_LINE("auto_screen_carousel_secs: %u\n", (unsigned)dp.auto_screen_carousel_secs);
    FCD_LINE("compass_north_top: %s\n", dp.compass_north_top ? "true" : "false");
    FCD_LINE("flip_screen: %s\n", dp.flip_screen ? "true" : "false");
    FCD_LINE("units: %s\n", unitsName(dp.units));
    FCD_LINE("oled: %s\n", oledTypeName(dp.oled));
    FCD_LINE("displaymode: %s\n", displayModeName(dp.displaymode));
    FCD_LINE("heading_bold: %s\n", dp.heading_bold ? "true" : "false");
    FCD_LINE("wake_on_tap_or_motion: %s\n", dp.wake_on_tap_or_motion ? "true" : "false");
    FCD_LINE("compass_orientation: %s\n", compassOrientName(dp.compass_orientation));
    FCD_LINE("use_12h_clock: %s\n", dp.use_12h_clock ? "true" : "false");
    FCD_LINE("use_long_node_name: %s\n", dp.use_long_node_name ? "true" : "false");
    FCD_LINE("enable_message_bubbles: %s\n", dp.enable_message_bubbles ? "true" : "false");
    appendPaging("--\n");
    #undef FCD_LINE
}

void NexusModule::appendFCDSection_LORA()
{
    auto &lr = config.lora;
    #define FCD_LINE(...) do { char line[160]; snprintf(line, sizeof(line), __VA_ARGS__); appendPaging(line); } while(0)
    appendPaging("JT Full Config: LORA\n--\n");
    FCD_LINE("use_preset: %s\n", lr.use_preset ? "true" : "false");
    FCD_LINE("modem_preset: %s\n", modemPresetName(lr.modem_preset));
    FCD_LINE("bandwidth: %u\n", (unsigned)lr.bandwidth);
    FCD_LINE("spread_factor: %u\n", (unsigned)lr.spread_factor);
    FCD_LINE("coding_rate: %u\n", (unsigned)lr.coding_rate);
    FCD_LINE("frequency_offset: %.3f\n", (double)lr.frequency_offset);
    FCD_LINE("region: %s\n", regionName(lr.region));
    FCD_LINE("hop_limit: %u\n", (unsigned)lr.hop_limit);
    FCD_LINE("tx_enabled: %s\n", lr.tx_enabled ? "true" : "false");
    FCD_LINE("tx_power: %d\n", (int)lr.tx_power);
    FCD_LINE("channel_num: %u\n", (unsigned)lr.channel_num);
    FCD_LINE("override_duty_cycle: %s\n", lr.override_duty_cycle ? "true" : "false");
    FCD_LINE("sx126x_rx_boosted_gain: %s\n", lr.sx126x_rx_boosted_gain ? "true" : "false");
    FCD_LINE("override_frequency: %.3f\n", (double)lr.override_frequency);
    FCD_LINE("pa_fan_disabled: %s\n", lr.pa_fan_disabled ? "true" : "false");
    FCD_LINE("ignore_incoming_count: %u\n", (unsigned)lr.ignore_incoming_count);
    for (size_t i = 0; i < lr.ignore_incoming_count && i < 3; i++) {
        char line[64];
        snprintf(line, sizeof(line), "ignore_incoming[%u]: !%08x\n",
                 (unsigned)i, (unsigned)lr.ignore_incoming[i]);
        appendPaging(line);
    }
    FCD_LINE("ignore_mqtt: %s\n", lr.ignore_mqtt ? "true" : "false");
    FCD_LINE("config_ok_to_mqtt: %s\n", lr.config_ok_to_mqtt ? "true" : "false");
    FCD_LINE("fem_lna_mode: %s\n", femModeName(lr.fem_lna_mode));
    FCD_LINE("serial_hal_only: %s\n", lr.serial_hal_only ? "true" : "false");
    appendPaging("--\n");
    #undef FCD_LINE
}

void NexusModule::appendFCDSection_BLUETOOTH()
{
    auto &bt = config.bluetooth;
    #define FCD_LINE(...) do { char line[160]; snprintf(line, sizeof(line), __VA_ARGS__); appendPaging(line); } while(0)
    appendPaging("JT Full Config: BLUETOOTH\n--\n");
    FCD_LINE("enabled: %s\n", bt.enabled ? "true" : "false");
    FCD_LINE("mode: %s\n", pairingModeName(bt.mode));
    FCD_LINE("fixed_pin: [redacted]\n");
    appendPaging("--\n");
    #undef FCD_LINE
}

void NexusModule::appendFCDSection_MQTT()
{
    auto &mq = moduleConfig.mqtt;
    #define FCD_LINE(...) do { char line[160]; snprintf(line, sizeof(line), __VA_ARGS__); appendPaging(line); } while(0)
    appendPaging("JT Full Config: MQTT\n--\n");
    FCD_LINE("enabled: %s\n", mq.enabled ? "true" : "false");
    FCD_LINE("address: %s\n", mq.address);
    FCD_LINE("username: %s\n", mq.username);
    appendPaging("password: ***\n");
    FCD_LINE("encryption_enabled: %s\n", mq.encryption_enabled ? "true" : "false");
    FCD_LINE("tls_enabled: %s\n", mq.tls_enabled ? "true" : "false");
    FCD_LINE("root: %s\n", mq.root);
    FCD_LINE("proxy_to_client_enabled: %s\n", mq.proxy_to_client_enabled ? "true" : "false");
    FCD_LINE("map_reporting_enabled: %s\n", mq.map_reporting_enabled ? "true" : "false");
    if (mq.has_map_report_settings) {
        FCD_LINE("map_report_settings.publish_interval_secs: %u\n", (unsigned)mq.map_report_settings.publish_interval_secs);
        FCD_LINE("map_report_settings.position_precision: %u\n", (unsigned)mq.map_report_settings.position_precision);
    } else {
        appendPaging("map_report_settings: (not set)\n");
    }
    appendPaging("--\n");
    #undef FCD_LINE
}

void NexusModule::appendFCDSection_SERIAL()
{
    auto &se = moduleConfig.serial;
    #define FCD_LINE(...) do { char line[160]; snprintf(line, sizeof(line), __VA_ARGS__); appendPaging(line); } while(0)
    appendPaging("JT Full Config: SERIAL\n--\n");
    FCD_LINE("enabled: %s\n", se.enabled ? "true" : "false");
    FCD_LINE("echo: %s\n", se.echo ? "true" : "false");
    FCD_LINE("rxd: %u\n", (unsigned)se.rxd);
    FCD_LINE("txd: %u\n", (unsigned)se.txd);
    FCD_LINE("baud: %s\n", serialBaudName(se.baud));
    FCD_LINE("timeout: %u\n", (unsigned)se.timeout);
    FCD_LINE("mode: %s\n", serialModeName(se.mode));
    FCD_LINE("override_console_serial_port: %s\n", se.override_console_serial_port ? "true" : "false");
    appendPaging("--\n");
    #undef FCD_LINE
}

void NexusModule::appendFCDSection_EXTNOTIF()
{
    auto &en = moduleConfig.external_notification;
    #define FCD_LINE(...) do { char line[160]; snprintf(line, sizeof(line), __VA_ARGS__); appendPaging(line); } while(0)
    appendPaging("JT Full Config: EXTNOTIF\n--\n");
    FCD_LINE("enabled: %s\n", en.enabled ? "true" : "false");
    FCD_LINE("output_ms: %u\n", (unsigned)en.output_ms);
    FCD_LINE("output: %u\n", (unsigned)en.output);
    FCD_LINE("active: %s\n", en.active ? "true" : "false");
    FCD_LINE("alert_message: %s\n", en.alert_message ? "true" : "false");
    FCD_LINE("alert_bell: %s\n", en.alert_bell ? "true" : "false");
    FCD_LINE("use_pwm: %s\n", en.use_pwm ? "true" : "false");
    FCD_LINE("output_vibra: %u\n", (unsigned)en.output_vibra);
    FCD_LINE("output_buzzer: %u\n", (unsigned)en.output_buzzer);
    FCD_LINE("alert_message_vibra: %s\n", en.alert_message_vibra ? "true" : "false");
    FCD_LINE("alert_message_buzzer: %s\n", en.alert_message_buzzer ? "true" : "false");
    FCD_LINE("alert_bell_vibra: %s\n", en.alert_bell_vibra ? "true" : "false");
    FCD_LINE("alert_bell_buzzer: %s\n", en.alert_bell_buzzer ? "true" : "false");
    FCD_LINE("nag_timeout: %u\n", (unsigned)en.nag_timeout);
    FCD_LINE("use_i2s_as_buzzer: %s\n", en.use_i2s_as_buzzer ? "true" : "false");
    appendPaging("--\n");
    #undef FCD_LINE
}

void NexusModule::appendFCDSection_STOREFORWARD()
{
    auto &sf = moduleConfig.store_forward;
    #define FCD_LINE(...) do { char line[160]; snprintf(line, sizeof(line), __VA_ARGS__); appendPaging(line); } while(0)
    appendPaging("JT Full Config: STOREFORWARD\n--\n");
    FCD_LINE("enabled: %s\n", sf.enabled ? "true" : "false");
    FCD_LINE("heartbeat: %s\n", sf.heartbeat ? "true" : "false");
    FCD_LINE("records: %u\n", (unsigned)sf.records);
    FCD_LINE("history_return_max: %u\n", (unsigned)sf.history_return_max);
    FCD_LINE("history_return_window: %u\n", (unsigned)sf.history_return_window);
    FCD_LINE("is_server: %s\n", sf.is_server ? "true" : "false");
    appendPaging("--\n");
    #undef FCD_LINE
}

void NexusModule::appendFCDSection_RANGETEST()
{
    auto &rt = moduleConfig.range_test;
    #define FCD_LINE(...) do { char line[160]; snprintf(line, sizeof(line), __VA_ARGS__); appendPaging(line); } while(0)
    appendPaging("JT Full Config: RANGETEST\n--\n");
    FCD_LINE("enabled: %s\n", rt.enabled ? "true" : "false");
    FCD_LINE("sender: %u\n", (unsigned)rt.sender);
    FCD_LINE("save: %s\n", rt.save ? "true" : "false");
    FCD_LINE("clear_on_reboot: %s\n", rt.clear_on_reboot ? "true" : "false");
    appendPaging("--\n");
    #undef FCD_LINE
}

void NexusModule::appendFCDSection_TELEMETRY()
{
    auto &tl = moduleConfig.telemetry;
    #define FCD_LINE(...) do { char line[160]; snprintf(line, sizeof(line), __VA_ARGS__); appendPaging(line); } while(0)
    appendPaging("JT Full Config: TELEMETRY\n--\n");
    FCD_LINE("device_update_interval: %u\n", (unsigned)tl.device_update_interval);
    FCD_LINE("environment_update_interval: %u\n", (unsigned)tl.environment_update_interval);
    FCD_LINE("environment_measurement_enabled: %s\n", tl.environment_measurement_enabled ? "true" : "false");
    FCD_LINE("environment_screen_enabled: %s\n", tl.environment_screen_enabled ? "true" : "false");
    FCD_LINE("environment_display_fahrenheit: %s\n", tl.environment_display_fahrenheit ? "true" : "false");
    FCD_LINE("air_quality_enabled: %s\n", tl.air_quality_enabled ? "true" : "false");
    FCD_LINE("air_quality_interval: %u\n", (unsigned)tl.air_quality_interval);
    FCD_LINE("power_measurement_enabled: %s\n", tl.power_measurement_enabled ? "true" : "false");
    FCD_LINE("power_update_interval: %u\n", (unsigned)tl.power_update_interval);
    FCD_LINE("power_screen_enabled: %s\n", tl.power_screen_enabled ? "true" : "false");
    FCD_LINE("health_measurement_enabled: %s\n", tl.health_measurement_enabled ? "true" : "false");
    FCD_LINE("health_update_interval: %u\n", (unsigned)tl.health_update_interval);
    FCD_LINE("health_screen_enabled: %s\n", tl.health_screen_enabled ? "true" : "false");
    FCD_LINE("device_telemetry_enabled: %s\n", tl.device_telemetry_enabled ? "true" : "false");
    FCD_LINE("air_quality_screen_enabled: %s\n", tl.air_quality_screen_enabled ? "true" : "false");
    appendPaging("--\n");
    #undef FCD_LINE
}

void NexusModule::appendFCDSection_CANNEDMSG()
{
    auto &cm = moduleConfig.canned_message;
    #define FCD_LINE(...) do { char line[160]; snprintf(line, sizeof(line), __VA_ARGS__); appendPaging(line); } while(0)
    appendPaging("JT Full Config: CANNEDMSG\n--\n");
    FCD_LINE("rotary1_enabled: %s\n", cm.rotary1_enabled ? "true" : "false");
    FCD_LINE("inputbroker_pin_a: %u\n", (unsigned)cm.inputbroker_pin_a);
    FCD_LINE("inputbroker_pin_b: %u\n", (unsigned)cm.inputbroker_pin_b);
    FCD_LINE("inputbroker_pin_press: %u\n", (unsigned)cm.inputbroker_pin_press);
    FCD_LINE("inputbroker_event_cw: %u\n", (unsigned)cm.inputbroker_event_cw);
    FCD_LINE("inputbroker_event_ccw: %u\n", (unsigned)cm.inputbroker_event_ccw);
    FCD_LINE("inputbroker_event_press: %u\n", (unsigned)cm.inputbroker_event_press);
    FCD_LINE("updown1_enabled: %s\n", cm.updown1_enabled ? "true" : "false");
    FCD_LINE("enabled: %s\n", cm.enabled ? "true" : "false");
    FCD_LINE("allow_input_source: %s\n", cm.allow_input_source);
    FCD_LINE("send_bell: %s\n", cm.send_bell ? "true" : "false");
    appendPaging("--\n");
    #undef FCD_LINE
}

void NexusModule::appendFCDSection_AUDIO()
{
    auto &au = moduleConfig.audio;
    #define FCD_LINE(...) do { char line[160]; snprintf(line, sizeof(line), __VA_ARGS__); appendPaging(line); } while(0)
    appendPaging("JT Full Config: AUDIO\n--\n");
    FCD_LINE("codec2_enabled: %s\n", au.codec2_enabled ? "true" : "false");
    FCD_LINE("ptt_pin: %u\n", (unsigned)au.ptt_pin);
    FCD_LINE("bitrate: %u\n", (unsigned)au.bitrate);
    FCD_LINE("i2s_ws: %u\n", (unsigned)au.i2s_ws);
    FCD_LINE("i2s_sd: %u\n", (unsigned)au.i2s_sd);
    FCD_LINE("i2s_din: %u\n", (unsigned)au.i2s_din);
    FCD_LINE("i2s_sck: %u\n", (unsigned)au.i2s_sck);
    appendPaging("--\n");
    #undef FCD_LINE
}

void NexusModule::appendFCDSection_REMOTEHW()
{
    auto &rh = moduleConfig.remote_hardware;
    #define FCD_LINE(...) do { char line[160]; snprintf(line, sizeof(line), __VA_ARGS__); appendPaging(line); } while(0)
    appendPaging("JT Full Config: REMOTEHW\n--\n");
    FCD_LINE("enabled: %s\n", rh.enabled ? "true" : "false");
    FCD_LINE("allow_undefined_pin_access: %s\n", rh.allow_undefined_pin_access ? "true" : "false");
    FCD_LINE("available_pins_count: %u\n", (unsigned)rh.available_pins_count);
    // The available_pins[] array needs a bigger stack buffer because each
    // line includes a name string. 256 bytes covers full pin description.
    for (size_t i = 0; i < rh.available_pins_count && i < 4; i++) {
        char line[256];
        snprintf(line, sizeof(line), "available_pins[%u]: gpio=%u name=%s\n",
                 (unsigned)i, (unsigned)rh.available_pins[i].gpio_pin,
                 rh.available_pins[i].name);
        appendPaging(line);
    }
    appendPaging("--\n");
    #undef FCD_LINE
}

void NexusModule::appendFCDSection_NEIGHBORINFO()
{
    auto &nb = moduleConfig.neighbor_info;
    #define FCD_LINE(...) do { char line[160]; snprintf(line, sizeof(line), __VA_ARGS__); appendPaging(line); } while(0)
    appendPaging("JT Full Config: NEIGHBORINFO\n--\n");
    FCD_LINE("enabled: %s\n", nb.enabled ? "true" : "false");
    FCD_LINE("update_interval: %u\n", (unsigned)nb.update_interval);
    FCD_LINE("transmit_over_lora: %s\n", nb.transmit_over_lora ? "true" : "false");
    appendPaging("--\n");
    #undef FCD_LINE
}

const char* NexusModule::skipToken(const char* input) {
    if (!input || *input == '\0') return input;
    const char *p = input;
    while (*p == ' ') p++;
    while (*p != '\0' && *p != ' ') p++;
    while (*p == ' ') p++;
    return p;
}

uint32_t NexusModule::parseNexusID(const char* input) {
    if (!input || *input == '\0') return 0;
    const char *p = input;
    while (*p == ' ' || *p == '!') p++;
    return (uint32_t)strtoul(p, NULL, 16);
}

size_t NexusModule::hopsToEmoji(char *out, size_t outSize, uint32_t hopsUsed) {
    if (!out || outSize == 0) return 0;
    static const char *keycapMap[] = {
        "0️⃣", "1️⃣", "2️⃣", "3️⃣", "4️⃣",
        "5️⃣", "6️⃣", "7️⃣", "8️⃣", "9️⃣"
    };
    if (hopsUsed == 0) {
        // Direct link: '*' marker (matches legacy behaviour).
        return snprintf(out, outSize, "*");
    } else if (hopsUsed <= 9) {
        return snprintf(out, outSize, "%s", keycapMap[hopsUsed]);
    } else {
        // >9 hops: fall back to ASCII digits to keep UTF-8 budget tight.
        return snprintf(out, outSize, "%u", (unsigned int)hopsUsed);
    }
}

bool NexusModule::sendToPhone(uint32_t to, uint32_t ch, uint32_t replyId, const char *msg, size_t len, bool asReaction, const meshtastic_MeshPacket *src) {
    if (!msg || len == 0 || !router || !service)
        return false;
    if (len > NEXUS_MAX_REPLY_BODY) len = NEXUS_MAX_REPLY_BODY;
    meshtastic_MeshPacket *p = (router) ? router->allocForSending() : nullptr;
    if (!p) return false;
    p->decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;
    p->to = to;
    p->channel = ch;
    if (replyId) p->decoded.reply_id = replyId;
    if (asReaction) {
        p->decoded.emoji = 1;
        p->decoded.bitfield |= (1 << 0);
        p->decoded.has_bitfield = true;
    }
    if (src) {
        // Forward original signal metadata to the phone UI for trace_alert.
        p->from = src->from;
        p->rx_rssi = src->rx_rssi;
        p->rx_snr = src->rx_snr;
        p->hop_start = src->hop_start;
        p->hop_limit = src->hop_limit;
    }
    memcpy(p->decoded.payload.bytes, msg, len);
    p->decoded.payload.size = len;
    if (service) service->sendToPhone(p);
    return true;
}

bool NexusModule::sendToMeshPacket(uint32_t to, uint32_t ch, uint32_t replyId, const char *msg, size_t len, bool wantAck, bool reliable) {
    if (!msg || len == 0 || !router || !service)
        return false;
    if (len > NEXUS_MAX_REPLY_BODY) len = NEXUS_MAX_REPLY_BODY;
    meshtastic_MeshPacket *p = (router) ? router->allocForSending() : nullptr;
    if (!p) return false;
    p->decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;
    p->to = to;
    p->channel = ch;
    p->want_ack = wantAck;
    if (replyId) p->decoded.reply_id = replyId;
    if (reliable) p->priority = meshtastic_MeshPacket_Priority_RELIABLE;
    memcpy(p->decoded.payload.bytes, msg, len);
    p->decoded.payload.size = len;
    return service->sendToMesh(p) == ERRNO_OK;
}

bool NexusModule::queueMeshReply(uint32_t to, uint32_t ch, uint32_t replyId, const char *msg, size_t len, bool isReaction, bool wantAck) {
    if (!msg || len == 0) return false;
    if (len > NEXUS_MAX_REPLY_BODY) len = NEXUS_MAX_REPLY_BODY;
    // The isDM flag is implicit: broadcast targets (0xFFFFFFFF) get a 100-5000ms
    // jitter to look human; DM targets (specific node) get a 50ms yield.
    bool isBroadcast = (to == 0xFFFFFFFF);
    pendingReply.active = true;
    pendingReply.to = to;
    pendingReply.channel = ch;
    pendingReply.reply_id = replyId;
    pendingReply.want_ack = wantAck;
    pendingReply.is_reaction = isReaction;
    memcpy(pendingReply.message, msg, len);
    pendingReply.message[len] = '\0';
    if (isBroadcast) {
        uint32_t jitter = broadcastReplyJitterMs();
        pendingReply.send_at = Time::getMillis() + jitter;
        LOG_INFO("Nexus: Queuing broadcast reply with %ums jitter", jitter);
    } else {
        pendingReply.send_at = Time::getMillis() + 50;
        LOG_INFO("Nexus: Queuing DM reply (50ms yield)");
    }
    setIntervalFromNow(50);
    return true;
}

bool NexusModule::parsePosArgs(const char *cmd, int offset,
                                uint32_t *outId, double *outArgs, int numArgs,
                                int *outChannel, int *outHopLim, int *outHopStart,
                                bool *outUseMqtt, bool *outHasCh, bool *outHasHops) {
    if (!cmd || !outId || !outArgs) return false;
    const char *p = cmd + offset;
    while (*p == ' ') p++;

    // 1. Target ID (mandatory)
    if (*p == '!') p++;
    *outId = (uint32_t)strtoul(p, (char**)&p, 16);
    while (*p == ' ') p++;
    if (*outId == 0) return false;

    // 2. Positional double args (coord triples, or coord-range min/max
    //    quintuples for the random-position variant)
    for (int i = 0; i < numArgs; i++) {
        outArgs[i] = strtod(p, (char**)&p);
        while (*p == ' ') p++;
    }

    // 3. Optional flags C / H / M
    bool customCh = false, customHops = false, useMqtt = false;
    while (*p != '\0' && *p != ' ') {
        char c = toupper(*p);
        if (c == 'C') customCh = true;
        else if (c == 'H') customHops = true;
        else if (c == 'M') useMqtt = true;
        else return false; // Unknown flag char
        p++;
    }
    while (*p == ' ') p++;

    *outHasCh = customCh;
    *outHasHops = customHops;
    *outUseMqtt = useMqtt;

    // 4. Optional channel value
    if (customCh) {
        *outChannel = (int)strtol(p, (char**)&p, 10);
        while (*p == ' ') p++;
    } else {
        *outChannel = -1;
    }

    // 5. Optional hop values
    if (customHops) {
        *outHopLim = (int)strtol(p, (char**)&p, 10);
        while (*p == ' ') p++;
        *outHopStart = (int)strtol(p, (char**)&p, 10);
        while (*p == ' ') p++;
    } else {
        *outHopLim = -1;
        *outHopStart = -1;
    }

    return true;
}

bool NexusModule::sendToNexusChannel(const char *msg, size_t len, uint32_t replyId) {
    if (!msg || len == 0) return false;
    uint32_t ch = findFirstNexusChannel();
    if (ch == 0xFFFFFFFF) return false; // No Nexus channel configured
    return sendToMeshPacket(0xFFFFFFFF, ch, replyId, msg, len, false, true);
}

const char* NexusModule::matchTargetedCmd(const char *cmdBuf, size_t *outKind) {
    if (outKind) *outKind = TARGET_NONE;
    if (!cmdBuf) return nullptr;

    // Suffixes (the part appended after the root). Per-suffix lengths match the literal
    // "/nexus-" + suffix (e.g. "/nexus-device " is 14 chars). The root itself contributes
    // its own length, so we compute the total prefix length per attempt.
    static const char* const SUFFIXES[]    = {"-device ", "-node ", "-mac ", "-group "};
    static const size_t      SUFFIX_LENS[] = { 8,          6,        5,       7         }; // includes trailing space
    static const size_t      SUFFIX_COUNT  = sizeof(SUFFIXES) / sizeof(SUFFIXES[0]);

    for (size_t r = 0; r < NEXUS_PREFIX_ROOTS_COUNT; r++) {
        const char* root = NEXUS_PREFIX_ROOTS[r];
        size_t rootLen = strlen(root);
        for (size_t s = 0; s < SUFFIX_COUNT; s++) {
            size_t totalLen = rootLen + SUFFIX_LENS[s];
            if (strncasecmp(cmdBuf, root, rootLen) != 0) continue;
            // Reject a root that isn't followed by the suffix (e.g. "/nexusx" must not match "/nexus").
            if (strncasecmp(cmdBuf + rootLen, SUFFIXES[s], SUFFIX_LENS[s]) != 0) continue;

            const char *argPtr = cmdBuf + totalLen;
            switch (s) {
                case 0: { // -device <id>
                    char targetIdStr[12] = {0};
                    int i = 0;
                    while (argPtr[i] && argPtr[i] != ' ' && i < 11) {
                        targetIdStr[i] = argPtr[i];
                        i++;
                    }
                    if (strcasecmp(targetIdStr, owner.id) == 0) {
                        const char *cmdStart = argPtr + i;
                        while (*cmdStart == ' ') cmdStart++;
                        if (outKind) *outKind = TARGET_ID;
                        return cmdStart;
                    }
                    return nullptr;
                }
                case 1: { // -node <shortname>
                    char targetNameStr[12] = {0};
                    int i = 0;
                    while (argPtr[i] && argPtr[i] != ' ' && i < 11) {
                        targetNameStr[i] = argPtr[i];
                        i++;
                    }
                    if (strcasecmp(targetNameStr, owner.short_name) == 0) {
                        const char *cmdStart = argPtr + i;
                        while (*cmdStart == ' ') cmdStart++;
                        if (outKind) *outKind = TARGET_NAME;
                        return cmdStart;
                    }
                    return nullptr;
                }
                case 2: { // -mac <hex>
                    const char *macPtr = argPtr;
                    uint32_t targetMac = strtoul(macPtr, (char**)&macPtr, 16);
                    uint8_t realMac[6];
                    getMacAddr(realMac);
                    uint32_t ourMacSuffix = (realMac[4] << 8) | realMac[5];
                    if (targetMac == ourMacSuffix) {
                        while (*macPtr == ' ') macPtr++;
                        if (outKind) *outKind = TARGET_MAC;
                        return macPtr;
                    }
                    return nullptr;
                }
                case 3: { // -group <name>
                    const char *p = argPtr;
                    while (*p == ' ') p++; // defensive: tolerate multiple spaces after the prefix
                    char groupName[16] = {0};
                    int gIdx = 0;
                    while (*p && *p != ' ' && gIdx < 15) groupName[gIdx++] = *p++;
                    groupName[gIdx] = '\0';

                    bool inGroup = false;
                    for (int i = 0; i < 4; i++) {
                        if (nexus_groups[i][0] != '\0' && strcasecmp(nexus_groups[i], groupName) == 0) {
                            inGroup = true;
                            break;
                        }
                    }
                    if (inGroup) {
                        while (*p == ' ') p++;
                        if (outKind) *outKind = TARGET_GROUP;
                        return p;
                    }
                    return nullptr;
                }
            }
        }
    }

    return nullptr;
}

uint32_t NexusModule::findNodeByShortNameOrHex(const char* input, const char** nextToken) {
    if (!input || *input == '\0') return 0;
    const char *p = input;
    while (*p == ' ') p++;
    if (*p == '\0') return 0;

    char target[32] = {0};
    int i = 0;
    const char *scan = p;
    while (*scan && *scan != ' ' && i < 31) {
        target[i++] = *scan++;
    }
    target[i] = '\0';

    if (nextToken) {
        while (*scan == ' ') scan++;
        *nextToken = scan;
    }

    uint32_t targetId = 0;

    // 0. Try hex first IF it starts with 0x, X or !
    if (target[0] == '!' || target[0] == 'x' || target[0] == 'X') {
        const char *hexPtr = (target[0] == '!') ? target + 1 : ((target[0] == '0' && (target[1] == 'x' || target[1] == 'X')) ? target + 2 : target);
        targetId = (uint32_t)strtoul(hexPtr, NULL, 16);
        if (targetId != 0) return targetId;
    } else if (target[0] == '0' && (target[1] == 'x' || target[1] == 'X')) {
        targetId = (uint32_t)strtoul(target + 2, NULL, 16);
        if (targetId != 0) return targetId;
    }

    // 1. Try aliases
    for (int j = 0; j < 16; j++) {
        if (nexus_aliases[j].nodeID != 0 && strcasecmp(nexus_aliases[j].alias, target) == 0) {
            return nexus_aliases[j].nodeID;
        }
    }

    // 2. Try to find by short name in NodeDB
    for (size_t j = 0; j < nodeDB->getNumMeshNodes(); j++) {
        auto n = nodeDB->getMeshNodeByIndex(j);
        if (n && nexus_has_user(n) && strcasecmp(n->short_name, target) == 0) {
            return n->num;
        }
    }

    // 3. Last ditch: try plain hex without prefix
    char *endptr;
    uint32_t val = (uint32_t)strtoul(target, &endptr, 16);
    if (*endptr == '\0' || *endptr == ' ') return val;

    return 0;
}

int NexusModule::findChannelByNameOrIndex(const char* input, const char** nextToken) {
    if (!input || *input == '\0') return -1;
    const char *p = input;
    while (*p == ' ') p++;

    char target[16] = {0};
    int i = 0;
    const char *scan = p;
    while (*scan && *scan != ' ' && i < 15) {
        target[i++] = *scan++;
    }
    target[i] = '\0';

    if (nextToken) {
        while (*scan == ' ') scan++;
        *nextToken = scan;
    }

    // 1. Try numerical index
    if (isdigit(target[0])) {
        int idx = atoi(target);
        if (idx >= 0 && idx < (int)MAX_NUM_CHANNELS && channels.getByIndex(idx).role != meshtastic_Channel_Role_DISABLED) {
            return idx;
        }
    }

    // 2. Try name lookup
    for (uint8_t j = 0; j < MAX_NUM_CHANNELS; j++) {
        meshtastic_Channel *c = &channels.getByIndex(j);
        if (c->role != meshtastic_Channel_Role_DISABLED && strcasecmp(c->settings.name, target) == 0) {
            return j;
        }
    }

    return -1;
}

bool NexusModule::saveVault()
{
    NexusVault *v = new (std::nothrow) NexusVault();
    if (!v) {
        LOG_ERROR("Nexus: Heap allocation failed for saveVault!");
        return false;
    }
    memset(v, 0, sizeof(NexusVault));
    memcpy(v->magic, "JenT", 4);
    v->version = 74; // v74: jen_advert (automatic ADVERT intervals, default OFF)
    // - v69 was: split_config (dual-preset split-tier and APC)
    // - v68 was: volt_rules (4-voltage threshold power automation)
    // - v67 was: nexusVariantBits (generic variant-bitmask pool — station-g3 PA/LNA)
    // - v66 was: features_mask + secondary_keys[4] (4 slots)
    v->drop_mask = getNexusDropHex();
    v->ping_emoji_mode = ping_emoji_mode; // v41
    uint8_t flags = 0;
    if (freeze_nodedb) flags |= 0x01;
    if (freeze_warm) flags |= 0x02;
    if (freeze_device) flags |= 0x04;
    if (freeze_config) flags |= 0x08;
    v->freeze_flags = flags; // v64
    v->broadcast_delay_max_s = broadcast_delay_max_s; // v43

    // Ping mask: bit 0 = public_ping, bits 1-8 = channels, bits 9-15 = flags
    v->ping_mask = public_ping ? 1 : 0;
    for(int i=0; i<8; i++) {
        if (ping_channels[i]) v->ping_mask |= (1 << (i+1));
    }
    if (ping_rssi) v->ping_mask |= (1 << 9);
    if (ping_snr) v->ping_mask |= (1 << 10);
    if (ping_freqerr) v->ping_mask |= (1 << 11);
    if (ping_bitmask) v->ping_mask |= (1 << 12);
    if (ping_ignore_direct) v->ping_mask |= (1 << 13);
    if (ping_ignore_remote) v->ping_mask |= (1 << 14);
    if (ping_allow_dm) v->ping_mask |= (1 << 15);

    // Aliases (v30)
    for(int i=0; i<16; i++) {
        v->aliases[i] = nexus_aliases[i];
    }

    // Macros (v30)
    for(int i=0; i<4; i++) {
        v->macros[i] = nexus_macros[i];
    }

    // Groups
    for(int i=0; i<4; i++) {
        strncpy(v->groups[i], nexus_groups[i], 8);
    }

    // Masking support
    v->hop_mask_active = hop_mask_active ? 1 : 0;
    v->hop_mask_start = hop_mask_start;
    v->hop_mask_limit = hop_mask_limit;

    v->M_S = M_S;
    v->client_always = client_always ? 1 : 0;
    v->R_S_mode = R_S_mode;
    v->R_S_id = R_S_id;
    v->base_texts = base_texts ? 1 : 0;

    // Secondary Keys (v8, expanded to 4 slots in v65)
    for(int i=0; i<4; i++) {
        memcpy(v->secondary_keys[i], nexus_secondary_keys[i], 32);
    }
    // v65: security subsystem bitmask (reserved — bits TBD)
    v->nexus_security_bitmask = nexus_security_bitmask;

    // Channel Filtering (v14: Expanded to 16)
    for(int i=0; i<16; i++) {
        v->ch_whitelist[i] = nexus_ch_whitelist[i];
        v->ch_blacklist[i] = nexus_ch_blacklist[i];
    }

    v->dm_forward = dm_forward ? 1 : 0;
    v->v11_id = v11_id;
    v->v11_mode = v11_mode ? 1 : 0;
    v->T_S_mode = T_S_mode;
    v->T_S_id = T_S_id;

    // Cron (v13)
    for(int i=0; i<8; i++) {
        v->cron_table[i] = cron_table[i];
    }

    // Custom PPING Header (v15)
    v->custom_pping_active = custom_pping_active ? 1 : 0;
    strncpy(v->pping_header, pping_header, 64);
    v->pping_header[63] = '\0';

    // Trace Alert (v16)
    v->trace_alert = trace_alert ? 1 : 0;
    v->trace_alert_mode = trace_alert_mode;

    // Trusted Channels (v17)
    for(int i=0; i<2; i++) {
        v->trusted_ch_active[i] = trusted_ch_active[i] ? 1 : 0;
        strncpy(v->trusted_ch_names[i], trusted_ch_names[i], 16);
        v->trusted_ch_names[i][15] = '\0';
    }

    // Meta Alert (v18)
    v->meta_alert = meta_alert ? 1 : 0;
    v->meta_alert_mode = meta_alert_mode;

    // Skip Skip (v19)
    v->skip_skip_handle = skip_skip_handle ? 1 : 0;
    v->out_drop_mask = getNexusOutDropHex();
    v->features_mask = getNexusFeaturesHex(); // v66: feature toggles (FEATURE_* bits)
    v->nexusVariantBits = nexusVariantBits;  // v67: generic variant-bitmask pool (station-g3 PA/LNA carve-out lives here)

    // Volt rules (v68)
    for (int i = 0; i < 4; i++) {
        v->volt_rules[i] = volt_rules[i];
    }

    // Split tier config (v69)
    v->split_config = splitConfig;

    // Multi-SF protocol tiers (v70)
    v->multi_sf = multiSFConfig;

    // JenRNode + JenCore persona state (v71)
#if NEXUS_ENABLE_RNODE
    v->jen_config.rnode_mode = (uint8_t)rnodeEngine.getMode();
    v->jen_config.rnode_tcp = rnodeEngine.isTcpEnabled() ? 1 : 0;
    v->jen_config.rnode_ble = rnodeEngine.isBleEnabled() ? 1 : 0;
    v->jen_config.rnode_sf = rnodeEngine.getSf();
#endif
#if NEXUS_ENABLE_JENCORE_SNIFFER
    v->jen_config.jencore_enabled = JenCoreRepeater::isJenCoreEnabled() ? 1 : 0;
    v->jen_config.jencore_relay = JenCoreRepeater::isRelayEnabled() ? 1 : 0;
    v->jen_config.jencore_path_hash = JenCoreRepeater::getPathHashSize();

    // JenCore region scoping (v72)
    v->jen_regions.count = JenCoreRepeater::getRegionCount();
    v->jen_regions.home = JenCoreRepeater::getHomeRegionIndex();
    v->jen_regions.allow_unscoped = JenCoreRepeater::getAllowUnscoped() ? 1 : 0;
    for (uint8_t i = 0; i < v->jen_regions.count && i < 4; i++)
        strncpy(v->jen_regions.names[i], JenCoreRepeater::getRegionName(i), sizeof(v->jen_regions.names[i]) - 1);
#endif

    // Per-channel Meshtastic TX tier overrides (v73)
    v->tier_channels.count = tierChannelOverrideCount;
    for (uint8_t i = 0; i < tierChannelOverrideCount && i < 8; i++) {
        v->tier_channels.entries[i].channel = tierChannelOverrides[i].channel;
        v->tier_channels.entries[i].sf = tierChannelOverrides[i].sf;
    }

    // Automatic ADVERT intervals (v74); both default OFF.
#if NEXUS_ENABLE_JENCORE_SNIFFER
    v->jen_advert.local_min = JenCoreRepeater::getAdvertLocalMinutes();
    v->jen_advert.flood_min = JenCoreRepeater::getAdvertFloodMinutes();
#endif

    // Nexus Ignores (v21)
    v->nexus_ignore_count = nexus_ignore_count;
    memcpy(v->nexus_ignore_list, nexus_ignore_list, sizeof(nexus_ignore_list));

    // Zero-Hop (v22)
    v->zero_hop_config = zero_hop_config;
    memcpy(v->zero_hop_mask, zero_hop_mask, sizeof(zero_hop_mask));

    // Relay Drops and IDR (v24)
    memcpy(v->relay_drop_mask, relay_drop_mask, 32);
    memcpy(v->ignore_dupe_relay_mask, ignore_dupe_relay_mask, 32);

    // MQTT Override (v25)
    v->mqtt_override_mode = mqtt_override_mode;

    // INA Legacy Math (v26)
    v->ina_legacy_math = ina_legacy_math ? 1 : 0;

    // Barometric Spoofing (v27)
    v->B_S_active = B_S_active ? 1 : 0;
    v->B_S_offset = B_S_offset;

    // Rate Limits (v28)
    v->rl_text_limit = rl_text_limit;
    v->rl_trace_limit = rl_trace_limit;
    v->rl_other_limit = rl_other_limit;
    v->rl_nodeinfo_limit = rl_nodeinfo_limit;
    v->rl_position_limit = rl_position_limit;
    v->rl_telemetry_limit = rl_telemetry_limit;
    v->rl_private_limit = rl_private_limit;

    // Invalid Filters (v29)
    v->invalid_mask = getNexusInvalidHex();

    // Granular Hopmasks (v31)
    for(int i=0; i<8; i++) v->per_ch_hopmask[i] = nexus_per_ch_hopmask[i];
    for(int i=0; i<5; i++) v->per_port_hopmask[i] = nexus_per_port_hopmask[i];

    v->t_m = t_m;
    v->s_o_active = s_o_active ? 1 : 0;
    v->s_o_value = s_o_value;
    v->preamble_override = preamble_override;
    v->syncword_override = syncword_override; // v40
    v->gpio_watch_mask = gpio_watch_mask;

    // Signatures (v39)
    for (int i = 0; i < 8; i++) {
        v->signatures[i] = nexus_signatures[i];
    }

    // Integrity
    v->crc32 = crc32Buffer(v, sizeof(NexusVault) - 4);

    bool saved;
    {
        concurrency::LockGuard guard(spiLock);
        saved = nexus::writeVault(FSCom, v, sizeof(*v), FILE_O_READ, FILE_O_WRITE);
    }
    delete v;
    if (!saved)
        LOG_ERROR("Nexus: Vault save failed; previous file preserved.");
    return saved;
}

bool NexusModule::readVaultForDisplay(NexusVault *out, const char **reason) {
    if (reason)
        *reason = "no file";
    if (!out) return false;
    concurrency::LockGuard guard(spiLock);
    File file = FSCom.open("/nexus.bin", FILE_O_READ);
    if (!file)
        return false;
    size_t size = file.size();
    memset(out, 0, sizeof(*out));
    size_t read = file.read(reinterpret_cast<uint8_t *>(out), std::min(size, sizeof(*out)));
    file.close();
    return nexus::decodeVault(*out, size, read, reason);
}

bool NexusModule::loadVault()
{
    NexusVault *v = new (std::nothrow) NexusVault();
    if (!v)
        return false;
    const char *reason = nullptr;
    if (!readVaultForDisplay(v, &reason)) {
        LOG_WARN("Nexus: Vault not loaded: %s. RAM unchanged.", reason ? reason : "allocation failure");
        delete v;
        return false;
    }
    {
        // Small inline sanitizer: clamp uint16 to [lo, hi], default to lo on garbage.
        auto clamp_u16 = [](uint16_t val, uint16_t lo, uint16_t hi) -> uint16_t {
            return (val >= lo && val <= hi) ? val : lo;
        };

        // ─── Sanitized field application ────────────────────────────────

        setNexusDropHex(v->drop_mask);
        public_ping = (v->ping_mask & 0x01) != 0;
        for (int i = 0; i < 8; i++) {
            ping_channels[i] = (v->ping_mask & (1 << (i + 1))) != 0;
        }
        ping_rssi            = (v->ping_mask & (1 << 9))  != 0;
        ping_snr             = (v->ping_mask & (1 << 10)) != 0;
        ping_freqerr         = (v->ping_mask & (1 << 11)) != 0;
        ping_bitmask         = (v->ping_mask & (1 << 12)) != 0;
        ping_ignore_direct   = (v->ping_mask & (1 << 13)) != 0;
        ping_ignore_remote   = (v->ping_mask & (1 << 14)) != 0;
        ping_allow_dm        = (v->ping_mask & (1 << 15)) != 0;

        // Groups (string fields, bounded)
        for (int i = 0; i < 4; i++) {
            strncpy(nexus_groups[i], v->groups[i], 8);
            nexus_groups[i][8] = '\0';
        }

        // Masking: hop_mask range is [0..7], start must be <= limit.
        hop_mask_active = (v->hop_mask_active != 0);
        hop_mask_start  = (v->hop_mask_start <= 7) ? v->hop_mask_start : 0;
        hop_mask_limit  = (v->hop_mask_limit <= 7 && v->hop_mask_limit >= v->hop_mask_start)
                              ? v->hop_mask_limit : 7;
        // Role enum max is CLIENT_BASE (12). -1 = disabled, anything else invalid → CLIENT (0).
        if (v->M_S == -1 || (v->M_S >= 0 && v->M_S <= 12)) {
            M_S = v->M_S;
        } else {
            M_S = 0;
        }

        client_always = (v->client_always != 0);
        R_S_mode = (v->R_S_mode <= 2) ? v->R_S_mode : 0;
        R_S_id   = v->R_S_id; // any byte valid (uint32 relay ID)
        base_texts    = (v->base_texts != 0);

        for (int i = 0; i < 4; i++) {
            memcpy(nexus_secondary_keys[i], v->secondary_keys[i], 32);
        }
        // v65: security subsystem bitmask (reserved — bits TBD; no clamp needed)
        nexus_security_bitmask = v->nexus_security_bitmask;

        // Channel filters (uint32, any value technically valid)
        for (int i = 0; i < 16; i++) {
            nexus_ch_whitelist[i] = v->ch_whitelist[i];
            nexus_ch_blacklist[i] = v->ch_blacklist[i];
        }

        dm_forward = (v->dm_forward != 0);

        v11_id   = v->v11_id;
        v11_mode = (v->v11_mode != 0);

        T_S_mode = (v->T_S_mode <= 2) ? v->T_S_mode : 0;
        T_S_id   = v->T_S_id;

        for (int i = 0; i < 8; i++) {
            cron_table[i] = v->cron_table[i];
            auto &job = cron_table[i];
            job.command[sizeof(job.command) - 1] = '\0';
            if (job.mode > 2 || !job.command[0] || (job.mode == 0 && (job.value < 60 || job.value > INT32_MAX / 1000)) ||
                (job.mode == 1 && job.value >= 86400) || (job.mode == 2 && job.value <= 1700000000))
                job.active = 0;
            cronLastRunMs[i] = Time::getMillis();
        }

        custom_pping_active = (v->custom_pping_active != 0);
        strncpy(pping_header, v->pping_header, 64);
        pping_header[63] = '\0';

        trace_alert      = (v->trace_alert != 0);
        trace_alert_mode = (v->trace_alert_mode <= 2) ? v->trace_alert_mode : 0;

        for (int i = 0; i < 2; i++) {
            trusted_ch_active[i] = (v->trusted_ch_active[i] != 0);
            strncpy(trusted_ch_names[i], v->trusted_ch_names[i], 16);
            trusted_ch_names[i][15] = '\0';
        }

        meta_alert      = (v->meta_alert != 0);
        meta_alert_mode = (v->meta_alert_mode <= 2) ? v->meta_alert_mode : 0;

        skip_skip_handle = (v->skip_skip_handle != 0);
        setNexusOutDropHex(v->out_drop_mask);
        setNexusFeaturesHex(v->features_mask); // v66: feature toggles (FEATURE_* bits)
        nexusVariantBits = v->nexusVariantBits; // v67: variant-bitmask pool (zero-init default when loading a v66 vault — fileSize clamp leaves the trailing 8 B at zero)

        // Volt rules (v68)
        for (int i = 0; i < 4; i++) {
            volt_rules[i] = v->volt_rules[i];
            volt_rules[i].command[sizeof(volt_rules[i].command) - 1] = '\0';
            if (volt_rules[i].action > NEXUS_VOLT_ACT_WAKE || volt_rules[i].direction > NEXUS_VOLT_DIR_LOW || volt_rules[i].channel > 2) {
                volt_rules[i].action = NEXUS_VOLT_ACT_NONE;
                volt_rules[i].mv = 0;
            }
            volt_debounce_count[i] = 0;
            volt_triggered[i] = false;
        }

        // Split tier config (v69)
        splitConfig = v->split_config;
        if (splitConfig.mode > NEXUS_SPLIT_ROOF_MSF) {
            splitConfig.mode = NEXUS_SPLIT_OFF;
        }

        // Multi-SF protocol tiers (v70). Structural validation only; the radio
        // applies/validates the detector table once it exists.
        multiSFConfig = v->multi_sf;
        if (multiSFConfig.enabled) {
            // The two systems own the same per-packet TX hook; the same-carrier
            // tier table wins and the frequency split stays off.
            splitConfig.mode = NEXUS_SPLIT_OFF;
            const char *reason = nullptr;
            if (!nexus::validateMultiSF(multiSFConfig, config.lora.spread_factor,
                                        bwCodeToKHz(config.lora.bandwidth), &reason)) {
                LOG_WARN("Nexus: MultiSF disabled at load: %s", reason ? reason : "invalid");
                multiSFConfig.enabled = 0;
            }
        }

        // JenRNode + JenCore persona state (v71). Older vaults have jen_config zeroed by decodeVault:
        // keep the radio persona OFF and default the relay/path width to MeshCore's 3-byte norm.
        {
            const bool legacyJen = v->version < 71;
#if NEXUS_ENABLE_RNODE
            const uint8_t rmode = v->jen_config.rnode_mode;
            rnodeEngine.setMode(rmode <= 2 ? (RNodeMode)rmode : RNODE_MODE_OFF);
            rnodeEngine.setTcpEnabled(legacyJen ? true : (v->jen_config.rnode_tcp != 0));
            rnodeEngine.setBleEnabled(legacyJen ? true : (v->jen_config.rnode_ble != 0));
            if (v->jen_config.rnode_sf >= 5 && v->jen_config.rnode_sf <= 12)
                rnodeEngine.setSf(v->jen_config.rnode_sf);
#endif
#if NEXUS_ENABLE_JENCORE_SNIFFER
            JenCoreRepeater::setJenCoreEnabled(v->jen_config.jencore_enabled != 0);
            JenCoreRepeater::setRelayEnabled(legacyJen ? true : (v->jen_config.jencore_relay != 0));
            JenCoreRepeater::setPathHashSize(legacyJen ? 3 : v->jen_config.jencore_path_hash);

            // Region scoping (v72). Older vaults keep the previous unscoped behaviour.
            JenCoreRepeater::clearRegions();
            if (v->version >= 72) {
                const uint8_t count = v->jen_regions.count <= 4 ? v->jen_regions.count : 4;
                for (uint8_t i = 0; i < count; i++) {
                    char name[25];
                    memcpy(name, v->jen_regions.names[i], sizeof(v->jen_regions.names[i]));
                    name[24] = '\0';
                    if (name[0])
                        JenCoreRepeater::addRegion(name);
                }
                if (v->jen_regions.home < count) {
                    char homeName[25];
                    memcpy(homeName, v->jen_regions.names[v->jen_regions.home], sizeof(v->jen_regions.names[0]));
                    homeName[24] = '\0';
                    if (homeName[0])
                        JenCoreRepeater::setHomeRegion(homeName);
                }
                JenCoreRepeater::setAllowUnscoped(v->jen_regions.allow_unscoped != 0);
            }
#endif

            // Per-channel Meshtastic TX tier overrides (v73).
            tierChannelOverrideCount = 0;
            memset(tierChannelOverrides, 0, sizeof(tierChannelOverrides));
            if (v->version >= 73) {
                const uint8_t count = v->tier_channels.count <= MAX_TIER_CHANNEL_OVERRIDES ? v->tier_channels.count : MAX_TIER_CHANNEL_OVERRIDES;
                for (uint8_t i = 0; i < count; i++) {
                    const uint8_t channel = v->tier_channels.entries[i].channel;
                    const uint8_t sf = v->tier_channels.entries[i].sf;
                    if (channel < MAX_NUM_CHANNELS && sf >= 5 && sf <= 12)
                        setTierChannelOverride(channel, sf);
                }
            }

            // Automatic ADVERT intervals (v74); older vaults keep both OFF.
#if NEXUS_ENABLE_JENCORE_SNIFFER
            if (v->version >= 74)
                JenCoreRepeater::setAdvertIntervals(v->jen_advert.local_min, v->jen_advert.flood_min);
            else
                JenCoreRepeater::setAdvertIntervals(0, 0);
#endif
        }

        nexus_ignore_count = (v->nexus_ignore_count <= 64) ? v->nexus_ignore_count : 64;
        memcpy(nexus_ignore_list, v->nexus_ignore_list, sizeof(nexus_ignore_list));

        // zero_hop_config is a 5-bit bitmap (EN/TELE/POS/AUTO_ROUTER/AUTO_CLIENT).
        // Older vaults only have the low bits set -> auto masks power up OFF.
        zero_hop_config = (v->zero_hop_config & 0x1F);
        memcpy(zero_hop_mask, v->zero_hop_mask, sizeof(zero_hop_mask));

        memcpy(relay_drop_mask, v->relay_drop_mask, 32);
        memcpy(ignore_dupe_relay_mask, v->ignore_dupe_relay_mask, 32);

        mqtt_override_mode = (v->mqtt_override_mode <= 2) ? v->mqtt_override_mode : 0;
        ina_legacy_math    = (v->ina_legacy_math != 0);

        B_S_active = (v->B_S_active != 0);
        if (v->B_S_offset >= -200.0f && v->B_S_offset <= 200.0f) {
            B_S_offset = v->B_S_offset;
        } else {
            B_S_offset = 0.0f;
        }

        // Rate limits: clamp to sane per-hour range. Default 0 = unlimited (match pre-fix semantics).
        rl_text_limit      = clamp_u16(v->rl_text_limit,      0, 10000);
        rl_trace_limit     = clamp_u16(v->rl_trace_limit,     0, 10000);
        rl_other_limit     = clamp_u16(v->rl_other_limit,     0, 10000);
        rl_nodeinfo_limit  = clamp_u16(v->rl_nodeinfo_limit,  0, 10000);
        rl_position_limit  = clamp_u16(v->rl_position_limit,  0, 10000);
        rl_telemetry_limit = clamp_u16(v->rl_telemetry_limit, 0, 10000);
        rl_private_limit   = clamp_u16(v->rl_private_limit,   0, 10000);

        setNexusInvalidHex(v->invalid_mask);

        for (int i = 0; i < 16; i++) {
            nexus_aliases[i] = v->aliases[i];
            nexus_aliases[i].alias[sizeof(nexus_aliases[i].alias) - 1] = '\0';
        }
        for (int i = 0; i < 4; i++) {
            nexus_macros[i] = v->macros[i];
            nexus_macros[i].name[sizeof(nexus_macros[i].name) - 1] = '\0';
            nexus_macros[i].command[sizeof(nexus_macros[i].command) - 1] = '\0';
        }

        for (int i = 0; i < 8; i++) {
            nexus_per_ch_hopmask[i].active = (v->per_ch_hopmask[i].active != 0);
            uint8_t s = v->per_ch_hopmask[i].start;
            uint8_t l = v->per_ch_hopmask[i].limit;
            nexus_per_ch_hopmask[i].start = (s <= 7) ? s : 0;
            nexus_per_ch_hopmask[i].limit = (l <= 7 && l >= s) ? l : 7;
        }
        for (int i = 0; i < 5; i++) {
            nexus_per_port_hopmask[i].active = (v->per_port_hopmask[i].active != 0);
            uint8_t s = v->per_port_hopmask[i].start;
            uint8_t l = v->per_port_hopmask[i].limit;
            nexus_per_port_hopmask[i].start = (s <= 7) ? s : 0;
            nexus_per_port_hopmask[i].limit = (l <= 7 && l >= s) ? l : 7;
        }

        t_m = (v->t_m <= 2) ? v->t_m : 0;

        s_o_active = (v->s_o_active != 0);
        // Range check on load; the radio-init path rejects wild values too.
        if (v->s_o_value >= -128.0f && v->s_o_value <= 128.0f) {
            s_o_value = v->s_o_value;
        } else {
            s_o_value = 0.0f;
        }

        // Preamble: 0 = use default. Valid LoRa range is 6..65535; preamble_override is uint16_t
        // so the upper bound is always true by definition. Compare >= 6 only.
        preamble_override = (v->preamble_override >= 6) ? v->preamble_override : 0;

        // ─── GPIO WATCH: DEFAULT-OFF, OPT-IN ONLY ───────────────────────
        // Per safety directive (2026-07-16): GPIO watch must NEVER
        // be activated from vault data on boot. The only path to enable
        // is an explicit operator command (GPIO WATCH N). Any bits the
        // operator previously saved are logged for diagnostic visibility
        // and discarded. This breaks the bootloop caused by stray v37
        // GPIO bits sweeping constructor-time pinMode() calls on flash/
        // PSRAM strap pins.
        if (v->gpio_watch_mask != 0) {
            LOG_WARN("Nexus: Vault held GPIO watch mask 0x%08lX%08lX. Default-off: ignoring. "
                     "Re-enable with /nexus GPIO WATCH <pin>.",
                     (unsigned long)(v->gpio_watch_mask >> 32),
                     (unsigned long)(v->gpio_watch_mask & 0xFFFFFFFFu));
        }
        gpio_watch_mask = 0;
        gpio_watch_prev = 0;
        // ────────────────────────────────────────────────────────────────

        for (int i = 0; i < 8; i++) {
            nexus_signatures[i] = v->signatures[i];
            if (nexus_signatures[i].len > 16 || nexus_signatures[i].type > 2)
                nexus_signatures[i].len = 0;
        }

        // Syncword: 0 = use Meshtastic default (0x2B). Anything else is a custom override.
        syncword_override = (v->syncword_override != 0) ? v->syncword_override : 0;

        // PPING emoji mode: 0=text, 1=indirect, 2=always.
        ping_emoji_mode = (v->ping_emoji_mode <= 2) ? v->ping_emoji_mode : 0;

        // Freeze flags (v64): bit0=NodeDB, bit1=Warm, bit2=Device, bit3=Config.
        // Other bits (4..7) reserved / ignored.
        freeze_nodedb  = (v->freeze_flags & 0x01) != 0;
        freeze_warm    = (v->freeze_flags & 0x02) != 0;
        freeze_device  = (v->freeze_flags & 0x04) != 0;
        freeze_config  = (v->freeze_flags & 0x08) != 0;

        // Broadcast delay: clamp [1, 300] seconds. v43 write path already clamps,
        // but sanitize on load in case the vault was edited externally.
        if (v->broadcast_delay_max_s >= 1 && v->broadcast_delay_max_s <= 300) {
            broadcast_delay_max_s = v->broadcast_delay_max_s;
        } else {
            broadcast_delay_max_s = 5;
        }

        LOG_INFO("Nexus: Vault loaded successfully (v%u).", (unsigned int)v->version);
    }
    delete v;
    return true;
}

void NexusModule::startPaging(uint32_t to, uint32_t ch, uint32_t replyId, const char* title) {
    if (to == 0) to = 0xFFFFFFFF;
    if (!pagingRendering) {
        ++pagingGeneration;
        pagingOffset = 0;
        paging_part = 0;
        paging_next_send = Time::getMillis();
        pagingCommand[0] = '\0';
        if (activeCommand && activeRequest) {
            snprintf(pagingCommand, sizeof(pagingCommand), "%s", activeCommand);
            pagingRequest = *activeRequest;
            pagingIsBroadcast = activeIsBroadcast;
        }
        pagingReply = {to, ch, replyId, activePhoneOnly, activePhoneOnly || (to == 0xFFFFFFFF)};
    }
    paging_to = to;
    paging_ch = ch;
    paging_reply_id = replyId;
    memset(paging_buf, 0, sizeof(paging_buf));
    pagingWindow.reset(pagingOffset);
    paging_ptr = 0;
    char header[96];
    snprintf(header, sizeof(header), "JT %s:\n", title);
    appendPaging(header);
    setIntervalFromNow(50);
}

void NexusModule::appendPaging(const char* entry) {
    pagingWindow.append(paging_buf, sizeof(paging_buf) - 4, entry);
    paging_ptr = pagingWindow.size();
}

bool NexusModule::sendReply(const ReplyContext &context, const char *message, size_t length, bool reliable)
{
    if (context.phoneOnly)
        return sendToPhone(context.to, context.channel, context.replyId, message, length);
    if (!sendToMeshPacket(context.to, context.channel, context.replyId, message, length, false, reliable))
        return false;
    if (context.copyToPhone)
        sendToPhone(context.to, context.channel, context.replyId, message, length);
    return true;
}

bool NexusModule::shouldDropOutput(const meshtastic_MeshPacket &mp) {
    // 0. Trusted Channel Bypass (trusted channels always allowed out)
    for (uint8_t i = 0; i < MAX_NUM_CHANNELS; i++) {
        meshtastic_Channel *c = &channels.getByIndex(i);
        if (c->role != meshtastic_Channel_Role_DISABLED) {
            if (mp.channel == i) {
                if (strcasecmp(c->settings.name, "Nexus") == 0 || strcasecmp(c->settings.name, "Nexus2") == 0) return false;
#ifdef NEXUS_TRUSTED_CHANNEL_1
                if (strcasecmp(c->settings.name, NEXUS_TRUSTED_CHANNEL_1) == 0) return false;
#endif
#ifdef NEXUS_TRUSTED_CHANNEL_2
                if (strcasecmp(c->settings.name, NEXUS_TRUSTED_CHANNEL_2) == 0) return false;
#endif
                for (int j = 0; j < 2; j++) {
                    if (trusted_ch_active[j] && strcasecmp(c->settings.name, trusted_ch_names[j]) == 0) return false;
                }
            }
        }
    }

    // 1. Favorite Bypass (If we are sending to a friend, don't block it)
    if (fav_bypass) {
        if (mp.to != 0xFFFFFFFF) {
            auto toNode = nodeDB->getMeshNode(mp.to);
            if (toNode && nexus_is_favorite(toNode)) return false;
        }
    }

    // 2. Port-based Outbound Filters
    if (mp.which_payload_variant == meshtastic_MeshPacket_decoded_tag) {
        uint32_t port = mp.decoded.portnum;
        if (drop_out_nodeinfo && port == meshtastic_PortNum_NODEINFO_APP) { fw_drop_out_nodeinfo++; return true; }
        if (drop_out_position && port == meshtastic_PortNum_POSITION_APP) { fw_drop_out_position++; return true; }
        if (drop_out_telemetry && port == meshtastic_PortNum_TELEMETRY_APP) { fw_drop_out_telemetry++; return true; }
        if (drop_out_traceroute && port == meshtastic_PortNum_TRACEROUTE_APP) { fw_drop_out_traceroute++; return true; }
        if (drop_out_admin && port == meshtastic_PortNum_ADMIN_APP) { fw_drop_out_admin++; return true; }
        if (drop_out_routing && port == meshtastic_PortNum_ROUTING_APP) { fw_drop_out_routing++; return true; }
    }

#ifdef NEXUS_PRIVATE_BUILD
    // Nexus-28006 Step C (2026-09-24) — private extension point.
    if (privateShouldDropOutput(mp)) {
        return true;
    }
#endif

    return false;
}

bool NexusModule::isNexusIgnored(uint32_t nodeID) {
    if (nodeID == 0) return false;
    for (uint16_t i = 0; i < nexus_ignore_count; i++) {
        if (nexus_ignore_list[i] == nodeID) return true;
    }
    return false;
}

bool NexusModule::shouldDropPrivate(const meshtastic_MeshPacket &mp) {
    updateStats(mp);

    // Nexus: Relay Node Drop Filter (256-bit mask)
    uint8_t relayId = (uint8_t)(mp.relay_node & 0xFF);
    if (isRelayDropped(relayId)) {
        fw_drop_relay++;
        return true;
    }
    // 0. Favorite Bypass (Header-level info available for encrypted)
    if (fav_bypass) {
        auto fromNode = nodeDB->getMeshNode(mp.from);
        if (fromNode && nexus_is_favorite(fromNode)) return false;
        if (mp.to != 0xFFFFFFFF) {
            auto toNode = nodeDB->getMeshNode(mp.to);
            if (toNode && nexus_is_favorite(toNode)) return false;
        }
    }


    // 0.2 Nexus Ignore List Check
    if (isNexusIgnored(mp.from)) {
        LOG_DEBUG("Nexus: Private packet dropped from Nexus-Ignored node !%08x", (unsigned int)mp.from);
        fw_drop_nexus_ignored++;
        return true;
    }

    // 1. PreHop Filter — two branches: genuine mp.pki_encrypted (decoded path sees local PKI,
    // opaque path never does) and the foreign-PKI heuristic for un-decodable PKI DMs where
    // mp.pki_encrypted is false but the wire shape signals PKI from another node.
    if (mp.pki_encrypted) {
        // Genuine PKI on the opaque path: this fires only for already-decoded PKI packets that
        // came back through this path (rare; mostly internal/MQTT-uplink replays). drop_pki_local
        // deliberately omitted — a local PKI packet is decodable by us and lands in
        // shouldDropPacket instead, where its drop_pki_local rule (with the auto-couple to
        // drop_pki) handles it.
        if (drop_pki) { fw_drop_pki++; return true; }
        if (drop_pki_remote) { fw_drop_pki_remote++; return true; }
        return false; // Bypass other rules — mp.channel is 0 for PKI, blacklist hashes never match.
    }
    if (looksLikeForeignPKI(mp)) {
        // Foreign PKI we couldn't decode. mp.channel is 0 on the wire (PKI signature) so the
        // channel blacklist/whitelist below can catch it via PRBLOCK ADD 0; drop_private also
        // fires because which_payload_variant == encrypted_tag. drop_pki_local omitted (no
        // local PKI on this path by definition).
        if (drop_pki) { fw_drop_pki++; return true; }
        if (drop_pki_remote) { fw_drop_pki_remote++; return true; }
        // Fall through to channel rules: PRBLOCK ADD 0, whitelist, drop_private, hop6/7.
    }

    // 1. Whitelist/Blacklist
    for(int i=0; i<16; i++) {
        if (nexus_ch_blacklist[i] != 0 && mp.channel == nexus_ch_blacklist[i]) { fw_drop_priv_blacklist++; return true; }
    }
    bool whitelistActive = false;
    bool onWhitelist = false;
    for(int i=0; i<16; i++) {
        if (nexus_ch_whitelist[i] != 0) { whitelistActive = true; if (mp.channel == nexus_ch_whitelist[i]) onWhitelist = true; }
    }
    if (whitelistActive && !onWhitelist) { fw_drop_priv_whitelist++; return true; }

    // 2. Rate Limiting (Nexus v28). checkRateLimit() handles both the
    //    decoded and encrypted variants; in this function we always
    //    hit the encrypted/PR bucket (idx=6). Engaged iff drop_rl_private
    //    is on AND rl_private_limit > 0; otherwise it's a no-op that
    //    doesn't tick rl_seen_private. Sits BEFORE the global drop_private
    //    so RL can drop a packet that drop_private would also drop
    //    (they're independent rules — RL limits throughput, drop_private
    //    is a blanket firewall on/off).
    if (!checkRateLimit(mp)) return true;

    // 3. Global Drop Private (encrypted-only - fw_drop_private counter is reserved for this rule)
    if (mp.which_payload_variant == meshtastic_MeshPacket_encrypted_tag) {
        if (drop_private) { fw_drop_private++; return true; }
    }

    // 4. Hop Filter
    if (drop_hop6 && mp.hop_start == 6) { fw_drop_hop6++; return true; }
    if (drop_hop7 && mp.hop_start == 7) { fw_drop_hop7++; return true; }

#ifdef NEXUS_PRIVATE_BUILD
    // Nexus-28006 Step C (2026-09-24) — private extension point.
    if (privateShouldDropPrivate(mp)) {
        return true;
    }
#endif

    return false;
}

// Nexus-28005 (2026-09-15): /nexus G3 (alias: FRONTEND) dispatcher body.
// station-g3 PA / LNA GPIO control with three runtime states per pin
// (DISABLED / OFF / ON). Gated entirely on #ifdef STATION_G3 — other boards
// (heltec-v4, t-beam-1w, tracker-t1000-e, seeed_mesh_tracker_X1, …) don't
// get any of this code, no RAM, no flash.
//
// Command surface:
//   G3                       → STATUS (bare keyword = full pin state report)
//   G3 STATUS                → STATUS (explicit, same output as bare G3)
//   G3 PA  ON|OFF            → drive / float GPIO9 (active-HIGH)
//   G3 LNA ON                → drive GPIO10 LOW (LNA engaged)
//   G3 LNA OFF               → drive GPIO10 HIGH (LNA bypassed)
//   G3 LNA BYPASS            → synonym for LNA OFF (RF-native vocabulary)
//
// FRONTEND is an alias for G3; both route to the same handler.
//
// Behavior on DISABLED pins (jumper detected at boot):
//   - PA|LNA ON|OFF returns an error reply pointing at the physical jumper.
//   - DISABLED is sticky for the session: only a reboot after removing the
//     jumper restores software control. No force-override flag exists by
//     design (2026-09-15: keep it simple, don't fight hardware).
//
// Vault persistence:
//   - ON / OFF update nexusVariantBits immediately (RAM-side state mirrors).
//   - The new bits are written to disk only on /nexus SAVE (Golden Rule
//     saveVault scope rule). We do NOT auto-save
//     on every state change — flash wear concerns on daily-driven stations.
//   - DISABLED is never persisted; it is re-detected every boot.
#ifdef STATION_G3
void NexusModule::nexusG3HandleCommand(const meshtastic_MeshPacket &mp, const char *cmd, char *replyMsg, size_t replySize)
{
    (void)mp; // unused — handler doesn't read the inbound packet fields
    const uint8_t PIN_PA  = 9;
    const uint8_t PIN_LNA = 10;

    // Tokenize: first token after the keyword ("G3" or "FRONTEND") is the
    // subcommand (PA / LNA / STATUS), second is the action (ON / OFF / BYPASS).
    // Mirrors the strncasecmp prefix-then-tokenize pattern used by every other
    // /nexus dispatcher branch in NexusCommands.cpp.
    char sub[16] = {0};
    char arg[16] = {0};
    {
        const char *p = cmd;
        // Skip the command keyword ("G3" or "FRONTEND") and its trailing space.
        while (*p && *p != ' ') p++;
        while (*p == ' ') p++;
        // First token → sub.
        int i = 0;
        while (*p && *p != ' ' && i < (int)sizeof(sub) - 1) sub[i++] = *p++;
        sub[i] = '\0';
        // Skip whitespace between tokens.
        while (*p == ' ') p++;
        // Second token → arg.
        i = 0;
        while (*p && *p != ' ' && i < (int)sizeof(arg) - 1) arg[i++] = *p++;
        arg[i] = '\0';
    }

    // Bare /nexus G3 (or /nexus G3 STATUS) → full state report.
    auto format_status = [&](char *out, size_t outsz) {
        // Per-pin label: state (driven-level for OFF/ON, jumper note for DISABLED).
        auto pa_label  = [&](char *buf, size_t bufsz) {
            switch (nexus_g3_pa_runtime) {
                case G3_DISABLED:
                    snprintf(buf, bufsz, "DISABLED (jumper detected at boot — firmware does not control)");
                    break;
                case G3_OFF:
                    snprintf(buf, bufsz, "OFF (GPIO9 INPUT / Hi-Z)");
                    break;
                case G3_ON:
                    snprintf(buf, bufsz, "ON  (GPIO9 OUTPUT HIGH)");
                    break;
            }
        };
        auto lna_label = [&](char *buf, size_t bufsz) {
            switch (nexus_g3_lna_runtime) {
                case G3_DISABLED:
                    snprintf(buf, bufsz, "DISABLED (jumper detected at boot — firmware does not control)");
                    break;
                case G3_OFF:
                    snprintf(buf, bufsz, "OFF (GPIO10 OUTPUT HIGH / bypassed)");
                    break;
                case G3_ON:
                    snprintf(buf, bufsz, "ON  (GPIO10 OUTPUT LOW)");
                    break;
            }
        };
        char pa_buf[80];
        char lna_buf[80];
        pa_label(pa_buf, sizeof(pa_buf));
        lna_label(lna_buf, sizeof(lna_buf));
        // Vault-persisted view (the bits that the next SAVE will write).
        uint64_t pa_bits  = nexusVariantBits & VARIANT_PA_MASK;
        uint64_t lna_bits = (nexusVariantBits & VARIANT_LNA_MASK) >> VARIANT_LNA_SHIFT;
        snprintf(out, outsz,
                 "JT G3 STATUS\n"
                 "  PA  = %s\n"
                 "  LNA = %s\n"
                 "  vault: PA=%s LNA=%s (run /nexus SAVE to persist)",
                 pa_buf, lna_buf,
                 (pa_bits  == VARIANT_PA_ON)  ? "ON"  :
                 (pa_bits  == VARIANT_PA_OFF) ? "OFF" : "?",
                 (lna_bits == VARIANT_LNA_ON) ? "ON"  :
                 (lna_bits == VARIANT_LNA_OFF) ? "OFF" : "?");
    };

    // Sub=empty (bare /nexus G3) → STATUS.
    if (sub[0] == '\0') {
        format_status(replyMsg, replySize);
        return;
    }

    if (strcasecmp(sub, "STATUS") == 0) {
        format_status(replyMsg, replySize);
        return;
    }

    // Nexus-28005 (2026-09-16): /nexus G3 REINIT — re-probe both PA / LNA
    // jumper pads and restore from vault if jumper absent. Mirrors the boot-
    // detect sequence (pinMode(INPUT) → delay(1) → digitalRead) but uses the
    // CURRENT vault bits as the source of truth for the "jumper absent"
    // branch (boot-detect uses a hardcoded OFF default since no vault exists
    // yet on first boot). Refuses to drive a pin marked DISABLED — jumper
    // installed means firmware stays out of the way, regardless of vault.
    if (strcasecmp(sub, "REINIT") == 0) {
        // --- PA (GPIO9, active-HIGH) ---
        // Nexus-28006 (2026-09-17) — REINIT latch-clear prefix.
        // Same rationale as boot-detect: digitalWrite(LOW) while pin is
        // still OUTPUT forces the per-pin output latch LOW, clearing any
        // stale HIGH from prior /nexus G3 PA ON (or other prior output
        // state) so the subsequent digitalRead() reads the actual
        // external jumper state instead of the stale latch. Microseconds,
        // watchdog-safe. Same prefix added to LNA below and to boot-detect
        // (Nexus-28006, 2026-09-17 06:07 UTC).
        digitalWrite(PIN_PA, LOW);
        pinMode(PIN_PA, INPUT);
        delay(1);
        int raw_pa = digitalRead(PIN_PA);
        if (raw_pa == HIGH) {
            nexus_g3_pa_runtime = G3_DISABLED;
            LOG_INFO("Nexus/G3 REINIT: PA (GPIO9) DISABLED — physical jumper detected. Firmware will not drive this pin.");
        } else {
            uint64_t pa_bits = nexusVariantBits & VARIANT_PA_MASK;
            if (pa_bits == VARIANT_PA_ON) {
                pinMode(PIN_PA, OUTPUT);
                digitalWrite(PIN_PA, HIGH); // active-HIGH
                nexus_g3_pa_runtime = G3_ON;
                LOG_INFO("Nexus/G3 REINIT: PA (GPIO9) vault ON — OUTPUT HIGH.");
            } else {
                pinMode(PIN_PA, INPUT); // true Hi-Z, no pull
                nexus_g3_pa_runtime = G3_OFF;
                LOG_INFO("Nexus/G3 REINIT: PA (GPIO9) vault OFF — INPUT / Hi-Z.");
            }
        }

        // --- LNA (GPIO10, active-LOW) ---
        // Nexus-28006 (2026-09-17) — REINIT latch-clear prefix.
        // Same shape as PA above and as boot-detect: digitalWrite(LOW)
        // before pinMode(INPUT) clears any stale output latch so the
        // subsequent digitalRead() reads the external jumper state
        // instead of stale OUTPUT-HIGH. Microseconds, watchdog-safe.
        digitalWrite(PIN_LNA, LOW);
        pinMode(PIN_LNA, INPUT);
        delay(1);
        int raw_lna = digitalRead(PIN_LNA);
        if (raw_lna == HIGH) {
            nexus_g3_lna_runtime = G3_DISABLED;
            LOG_INFO("Nexus/G3 REINIT: LNA (GPIO10) DISABLED — physical jumper detected. Firmware will not drive this pin.");
        } else {
            uint64_t lna_bits = (nexusVariantBits & VARIANT_LNA_MASK) >> VARIANT_LNA_SHIFT;
            if (lna_bits == VARIANT_LNA_ON) {
                pinMode(PIN_LNA, OUTPUT);
                digitalWrite(PIN_LNA, LOW); // active-LOW → LOW = ON
                nexus_g3_lna_runtime = G3_ON;
                LOG_INFO("Nexus/G3 REINIT: LNA (GPIO10) vault ON — OUTPUT LOW.");
            } else {
                pinMode(PIN_LNA, OUTPUT);
                digitalWrite(PIN_LNA, HIGH); // bypassed
                nexus_g3_lna_runtime = G3_OFF;
                LOG_INFO("Nexus/G3 REINIT: LNA (GPIO10) vault OFF — OUTPUT HIGH / bypassed.");
            }
        }

        // Reply with the new states. Same labels as STATUS uses so the
        // operator can confirm REINIT did what they expected.
        auto pa_label  = [&](char *buf, size_t bufsz) {
            switch (nexus_g3_pa_runtime) {
                case G3_DISABLED:
                    snprintf(buf, bufsz, "DISABLED (jumper detected)");
                    break;
                case G3_OFF:
                    snprintf(buf, bufsz, "OFF (GPIO9 INPUT / Hi-Z)");
                    break;
                case G3_ON:
                    snprintf(buf, bufsz, "ON  (GPIO9 OUTPUT HIGH)");
                    break;
            }
        };
        auto lna_label = [&](char *buf, size_t bufsz) {
            switch (nexus_g3_lna_runtime) {
                case G3_DISABLED:
                    snprintf(buf, bufsz, "DISABLED (jumper detected)");
                    break;
                case G3_OFF:
                    snprintf(buf, bufsz, "OFF (GPIO10 OUTPUT HIGH / bypassed)");
                    break;
                case G3_ON:
                    snprintf(buf, bufsz, "ON  (GPIO10 OUTPUT LOW)");
                    break;
            }
        };
        char pa_buf[64];
        char lna_buf[64];
        pa_label(pa_buf, sizeof(pa_buf));
        lna_label(lna_buf, sizeof(lna_buf));
        snprintf(replyMsg, replySize,
                 "JT G3 REINIT\n"
                 "  PA  = %s\n"
                 "  LNA = %s",
                 pa_buf, lna_buf);
        return;
    }

    if (strcasecmp(sub, "PA") == 0) {
        if (strcasecmp(arg, "ON") == 0) {
            if (nexus_g3_pa_runtime == G3_DISABLED) {
                snprintf(replyMsg, replySize,
                         "JT G3: PA ON refused — pin is DISABLED (physical jumper PL1 detected at boot). "
                         "Remove the jumper and reboot to regain software control.");
                return;
            }
            pinMode(PIN_PA, OUTPUT);
            digitalWrite(PIN_PA, HIGH); // active-HIGH
            nexus_g3_pa_runtime = G3_ON;
            nexusVariantBits = (nexusVariantBits & ~VARIANT_PA_MASK) | VARIANT_PA_ON;
            snprintf(replyMsg, replySize,
                     "JT G3: PA=ON (GPIO9 OUTPUT HIGH). RAM-only — run /nexus SAVE to persist.");
        } else if (strcasecmp(arg, "OFF") == 0) {
            if (nexus_g3_pa_runtime == G3_DISABLED) {
                snprintf(replyMsg, replySize,
                         "JT G3: PA OFF refused — pin is DISABLED (physical jumper PL1 detected at boot). "
                         "Remove the jumper and reboot to regain software control.");
                return;
            }
            pinMode(PIN_PA, INPUT); // true Hi-Z, no pull
            nexus_g3_pa_runtime = G3_OFF;
            nexusVariantBits = (nexusVariantBits & ~VARIANT_PA_MASK) | VARIANT_PA_OFF;
            snprintf(replyMsg, replySize,
                     "JT G3: PA=OFF (GPIO9 INPUT / Hi-Z). RAM-only — run /nexus SAVE to persist.");
        } else {
            snprintf(replyMsg, replySize, "JT G3: PA expects ON|OFF (got '%s')", arg);
        }
        return;
    }

    if (strcasecmp(sub, "LNA") == 0) {
        // BYPASS is an alias for OFF (RF-native vocabulary).
        bool want_on  = (strcasecmp(arg, "ON")     == 0);
        bool want_off = (strcasecmp(arg, "OFF")    == 0) ||
                        (strcasecmp(arg, "BYPASS") == 0);
        if (want_on) {
            if (nexus_g3_lna_runtime == G3_DISABLED) {
                snprintf(replyMsg, replySize,
                         "JT G3: LNA ON refused — pin is DISABLED (physical jumper LNA-P detected at boot). "
                         "Remove the jumper and reboot to regain software control.");
                return;
            }
            pinMode(PIN_LNA, OUTPUT);
            digitalWrite(PIN_LNA, LOW); // active-LOW → LOW = ON
            nexus_g3_lna_runtime = G3_ON;
            nexusVariantBits = (nexusVariantBits & ~VARIANT_LNA_MASK) | (VARIANT_LNA_ON << VARIANT_LNA_SHIFT);
            snprintf(replyMsg, replySize,
                     "JT G3: LNA=ON (GPIO10 OUTPUT LOW). RAM-only — run /nexus SAVE to persist.");
        } else if (want_off) {
            if (nexus_g3_lna_runtime == G3_DISABLED) {
                snprintf(replyMsg, replySize,
                         "JT G3: LNA OFF refused — pin is DISABLED (physical jumper LNA-P detected at boot). "
                         "Remove the jumper and reboot to regain software control.");
                return;
            }
            pinMode(PIN_LNA, OUTPUT);
            digitalWrite(PIN_LNA, HIGH); // bypassed
            nexus_g3_lna_runtime = G3_OFF;
            nexusVariantBits = (nexusVariantBits & ~VARIANT_LNA_MASK) | (VARIANT_LNA_OFF << VARIANT_LNA_SHIFT);
            snprintf(replyMsg, replySize,
                     "JT G3: LNA=OFF (GPIO10 OUTPUT HIGH / bypassed). RAM-only — run /nexus SAVE to persist.");
        } else {
            snprintf(replyMsg, replySize, "JT G3: LNA expects ON|OFF|BYPASS (got '%s')", arg);
        }
        return;
    }

    snprintf(replyMsg, replySize,
             "JT G3: unknown sub '%s'. Use PA | LNA | STATUS.", sub);
}
#endif // STATION_G3
