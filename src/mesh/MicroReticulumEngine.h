// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) 2026 Jennifer Sanchez
//
// Native microReticulum transport-node engine. It shares the RNode framing and
// radio path with RNodeEngine, but feeds a real RNS stack instead of a KISS
// host. Enabled only on ESP32-S3 + PSRAM targets.
//
// Keep the RNS types opaque here: including microReticulum.h from a header that
// Meshtastic modules include pollutes the global namespace (the stack has its
// own Allocator type), so only the .cpp sees the full library.

#pragma once

#include "configuration.h"

#if NEXUS_ENABLE_MICRORETICULUM

#include <cstddef>
#include <cstdint>

namespace RNS {
class Reticulum;
class Interface;
class InterfaceImpl;
} // namespace RNS

namespace microStore {
class FileSystem;
} // namespace microStore

class MicroReticulumEngine
{
  public:
    MicroReticulumEngine();
    ~MicroReticulumEngine();

    bool start();
    void stop();
    void loop();
    bool isActive() const { return active; }

    /// Radio-thread entry point (RNode frame sink).
    void onRadioFrame(const uint8_t *payload, size_t len, float snr, int32_t rssi);

    uint32_t rxPackets = 0;
    uint32_t txPackets = 0;
    uint32_t dropped = 0;

  private:
    struct RadioInterface;

    static void frameSinkThunk(const uint8_t *payload, size_t len, float snr, int32_t rssi);
    bool queueToRadio(const uint8_t *data, size_t len);
    void drainRx();

    bool active = false;
    RNS::Reticulum *reticulum = nullptr;
    RNS::Interface *loraInterface = nullptr;
    RadioInterface *radioImpl = nullptr;
    microStore::FileSystem *filesystem = nullptr;
    uint32_t lastPersistMs = 0;

    static constexpr size_t RX_RING = 4;
    static constexpr size_t RX_FRAME_MAX = 512;
    struct RxFrame {
        uint16_t len;
        float snr;
        int32_t rssi;
        uint8_t data[RX_FRAME_MAX];
    };
    RxFrame rxRing[RX_RING];
    volatile uint8_t rxHead = 0;
    volatile uint8_t rxTail = 0;
};

extern MicroReticulumEngine microReticulumEngine;

#endif // NEXUS_ENABLE_MICRORETICULUM
