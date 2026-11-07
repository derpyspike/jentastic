// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) 2026 Jennifer Sanchez
//
// Licensed under the GNU Affero General Public License v3.
// See LICENSE-AGPL for the full license text.
//
// Native LoRaWAN end-device engine (JenWAN). The blocking LoRaWAN join /
// sendReceive state machine runs in its own FreeRTOS task; the Meshtastic
// radio is handed over for the duration of a window and restored afterwards.
//
// Keep RadioLib's LoRaWAN types opaque here: including <LoRaWAN.h> from a
// header that NexusModule includes would pull the whole protocol into many
// translation units.

#pragma once

#include "configuration.h"

#if NEXUS_ENABLE_JENWAN

#include "JenWANVault.h"
#include "concurrency/Lock.h"
#include <stddef.h>
#include <stdint.h>

class LoRaWANNode;
struct LoRaWANBand_t;

namespace jenwan
{
class Engine
{
  public:
    /// Load /jenwan.bin and start the window task. Safe to call once at boot.
    bool begin();

    /// Called from NexusModule::runOnce(): schedule a window / drain state.
    void loop();

    /// Parse a /nexus JW (or LW/JENWAN) argument string. Always fills `reply`.
    bool handleCommand(const char *args, char *reply, size_t cap);

    /// Persist the current config + RadioLib session.
    bool save();

    const Config &config() const { return cfg; }

  private:
    enum class WindowKind : uint8_t { NONE = 0, JOIN, UPLINK };

    struct DownlinkEntry {
        uint8_t len;
        uint8_t data[48];
    };

    static constexpr size_t MAX_MANUAL = 51;
    static constexpr size_t MAX_DOWNLINK = 48;
    static constexpr size_t DOWNLINK_SLOTS = 4;

    Config cfg;
    LoRaWANNode *node = nullptr;
    const LoRaWANBand_t *band = nullptr;

    concurrency::Lock cfgLock;
    volatile bool windowActive = false;
    volatile bool windowRequested = false;
    volatile bool taskStarted = false;
    volatile bool activated = false;
    volatile bool needRestart = false;
    volatile WindowKind requestKind = WindowKind::NONE;
    uint8_t requestSources = 0;
    uint8_t requestPort = 1;
    uint8_t requestData[MAX_MANUAL];
    size_t requestLen = 0;

    void *taskHandle = nullptr;

    volatile uint32_t lastSentMs[NUM_SOURCES] = {};
    volatile uint32_t nextAttemptMs = 0;
    volatile uint32_t backoffMs = 0;
    volatile uint32_t lastUplinkMs = 0;
    volatile uint32_t uplinkCount = 0;
    volatile int16_t lastState = 0;
    volatile uint8_t lastPort = 0;
    volatile uint8_t lastLen = 0;

    DownlinkEntry downlinks[DOWNLINK_SLOTS] = {};
    volatile uint8_t downlinkHead = 0;
    volatile uint8_t downlinkCount = 0;
    char downlinkText[64] = {};
    volatile bool downlinkTextPending = false;

    bool ensureNode(const Config &local);
    void destroyNode();
    bool doJoin(const Config &local);
    void updateSession();
    bool runWindow(WindowKind kind, uint8_t sources, uint8_t port, const uint8_t *data, size_t len);
    void processRequest();
    bool requestWindow(WindowKind kind, uint8_t sources = 0, uint8_t port = 1, const uint8_t *data = nullptr, size_t len = 0);
    uint8_t dueSources(uint32_t nowMs) const;
    uint32_t nextDueMs(uint32_t nowMs) const;
    void handleDownlink(const uint8_t *data, size_t len);
    bool sendPendingDownlinkText();
    void appendStatus(char *reply, size_t cap);

    static void taskEntry(void *arg);
    void taskLoop();
};

extern Engine engine;
} // namespace jenwan

#endif // NEXUS_ENABLE_JENWAN
