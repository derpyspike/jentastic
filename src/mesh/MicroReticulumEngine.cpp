// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) 2026 Jennifer Sanchez

#include "mesh/MicroReticulumEngine.h"

#if NEXUS_ENABLE_MICRORETICULUM

#include "DebugConfiguration.h"
#include "UptimeClock.h"
#include "mesh/RNodeEngine.h"
#include "mesh/Throttle.h"

#include <microReticulum.h>
#include <microStore/Adapters/UniversalFileSystem.h>

#include <cstring>
#include <exception>
#include <new>

MicroReticulumEngine microReticulumEngine;

/// RNS interface backed by the Nexus radio. TX goes through RNodeEngine's
/// RNode framing; RX is pushed in from the radio thread by onRadioFrame() and
/// drained on the module thread before being handed to the stack.
struct MicroReticulumEngine::RadioInterface : public RNS::InterfaceImpl {
    explicit RadioInterface(MicroReticulumEngine &owner) : RNS::InterfaceImpl("NexusLoRa"), owner(owner)
    {
        _IN = true;
        _OUT = true;
        _HW_MTU = 508;
        _bitrate = 10000;
    }

    bool start() override
    {
        _online = true;
        return true;
    }

    void stop() override { _online = false; }

    bool send_outgoing(const RNS::Bytes &data) override
    {
        return owner.queueToRadio(data.data(), data.size());
    }

    MicroReticulumEngine &owner;
};

MicroReticulumEngine::MicroReticulumEngine() = default;
MicroReticulumEngine::~MicroReticulumEngine() { stop(); }

bool MicroReticulumEngine::start()
{
    if (active)
        return true;

    try {
        RNS::loglevel(RNS::LOG_INFO);

        // PosixFileSystem on ESP32 goes through the VFS; on other ports
        // UniversalFileSystem selects the board filesystem. The adapter is
        // heap-owned because the stack keeps a reference to it.
        filesystem = new microStore::FileSystem{microStore::Adapters::UniversalFileSystem()};
        if (!filesystem)
            return false;
        filesystem->init();
        RNS::Utilities::OS::register_filesystem(*filesystem);

        loraInterface = new RNS::Interface{RNS::Type::NONE};
        reticulum = new RNS::Reticulum();
        if (!loraInterface || !reticulum)
            throw std::bad_alloc();

        radioImpl = new RadioInterface(*this);
        *loraInterface = radioImpl;
        loraInterface->mode(RNS::Type::Interface::MODE_GATEWAY);
        RNS::Transport::register_interface(*loraInterface);
        loraInterface->start();

        reticulum->transport_enabled(true); // this is a transport node, not just an endpoint
        reticulum->probe_destination_enabled(true);
        reticulum->remote_management_enabled(true);
        reticulum->start();

        rnodeEngine.setFrameSink(&MicroReticulumEngine::frameSinkThunk);
        active = true;
        lastPersistMs = Time::getMillis();
        LOG_INFO("microReticulum: transport node started");
        return true;
    } catch (const std::exception &e) {
        LOG_ERROR("microReticulum: start failed: %s", e.what());
        stop();
        return false;
    } catch (...) {
        LOG_ERROR("microReticulum: start failed (unknown exception)");
        stop();
        return false;
    }
}

void MicroReticulumEngine::stop()
{
    if (!active && !radioImpl && !filesystem)
        return;

    try {
        if (active) {
            RNS::Transport::persist_data();
            rnodeEngine.setFrameSink(nullptr);
            if (loraInterface)
                RNS::Transport::deregister_interface(*loraInterface);
        }
    } catch (const std::exception &e) {
        LOG_WARN("microReticulum: teardown exception: %s", e.what());
    } catch (...) {
        LOG_WARN("microReticulum: teardown exception");
    }

    if (loraInterface) {
        if (radioImpl) {
            loraInterface->stop();
            loraInterface->clear();
        }
        delete loraInterface;
        loraInterface = nullptr;
        radioImpl = nullptr;
    }
    if (reticulum) {
        delete reticulum;
        reticulum = nullptr;
    }
    if (filesystem) {
        delete filesystem;
        filesystem = nullptr;
    }
    active = false;
    rxHead = rxTail = 0;
    LOG_INFO("microReticulum: stopped");
}

void MicroReticulumEngine::loop()
{
    if (!active)
        return;

    drainRx();

    try {
        reticulum->loop();
    } catch (const std::exception &e) {
        LOG_WARN("microReticulum: loop exception: %s", e.what());
    } catch (...) {
        LOG_WARN("microReticulum: loop exception");
    }

    // Keep the path table reasonably fresh without writing flash every loop.
    if (Throttle::hasElapsed(lastPersistMs, 300000)) {
        lastPersistMs = Time::getMillis();
        try {
            RNS::Transport::persist_data();
        } catch (...) {
            LOG_WARN("microReticulum: path table persist failed");
        }
    }
}

void MicroReticulumEngine::onRadioFrame(const uint8_t *payload, size_t len, float snr, int32_t rssi)
{
    if (!active || !payload || len == 0 || len > RX_FRAME_MAX) {
        dropped++;
        return;
    }

    const uint8_t next = (uint8_t)((rxHead + 1) % RX_RING);
    if (next == rxTail) {
        dropped++;
        return;
    }

    RxFrame &f = rxRing[rxHead];
    f.len = (uint16_t)len;
    f.snr = snr;
    f.rssi = rssi;
    memcpy(f.data, payload, len);
    __asm__ __volatile__("" ::: "memory"); // payload before the published head
    rxHead = next;
    rxPackets++;
}

void MicroReticulumEngine::drainRx()
{
    while (rxTail != rxHead) {
        RxFrame &f = rxRing[rxTail];

        loraInterface->r_stat_rssi((float)f.rssi);
        loraInterface->r_stat_snr(f.snr);
        loraInterface->r_stat_q(0.0f);

        RNS::Bytes data(f.data, f.len);
        loraInterface->handle_incoming(data);

        __asm__ __volatile__("" ::: "memory");
        rxTail = (uint8_t)((rxTail + 1) % RX_RING);
    }
}

bool MicroReticulumEngine::queueToRadio(const uint8_t *data, size_t len)
{
    if (!active || !rnodeEngine.queueMicroReticulumFrame(data, len))
        return false;
    txPackets++;
    return true;
}

void MicroReticulumEngine::frameSinkThunk(const uint8_t *payload, size_t len, float snr, int32_t rssi)
{
    microReticulumEngine.onRadioFrame(payload, len, snr, rssi);
}

#endif // NEXUS_ENABLE_MICRORETICULUM
