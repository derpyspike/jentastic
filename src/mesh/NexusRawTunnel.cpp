// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) 2026 Jennifer Sanchez
//
// Licensed under the GNU Affero General Public License v3.
// See LICENSE-AGPL for the full license text.
//

#include "NexusRawTunnel.h"

#if NEXUS_ENABLE_JENCORE || NEXUS_ENABLE_RNODE

#include "MeshService.h"
#include "MeshTypes.h"
#include "NodeDB.h"
#include "RNodeEngine.h"
#include "RadioLibInterface.h"
#include "Router.h"
#include "Throttle.h"
#include "UptimeClock.h"
#include "gps/RTC.h"
#include "mesh/generated/meshtastic/mesh.pb.h"
#include "modules/NexusModule.h"
#include <cstring>

namespace nexus::rawtunnel
{
namespace
{
bool featureEnabled()
{
    return meshtastic::nexusModule && (meshtastic::nexusModule->getNexusFeaturesHex() & meshtastic::FEATURE_RAW_TUNNEL_TO_PHONE);
}

uint8_t nextSeq()
{
    static uint8_t seq = 0;
    if (++seq == 0)
        ++seq;
    return seq;
}

bool dispatch(Type type, const uint8_t *data, size_t len)
{
    if (!data || len == 0)
        return false;

    switch (type) {
    case MESHCORE: {
        RadioLibInterface *rli = RadioLibInterface::instance;
        if (rli && rli->queueJenCoreFrame(data, len))
            return true;
        LOG_WARN("raw tunnel: MeshCore TX rejected (len=%u)", (unsigned)len);
        return false;
    }
#if NEXUS_ENABLE_RNODE
    case RNODE:
        if (rnodeEngine.sendReticulumPayload(data, len))
            return true;
        LOG_WARN("raw tunnel: RNode TX rejected (len=%u)", (unsigned)len);
        return false;
#endif
    default:
        LOG_WARN("raw tunnel: unknown type %u", (unsigned)type);
        return false;
    }
}

void sendChunk(Type type, const uint8_t *data, size_t len, uint8_t seq, uint8_t index, uint8_t count, int32_t rssi, float snr,
               float freqErr, uint8_t codingRate)
{
    meshtastic_MeshPacket *p = packetPool.allocZeroed();
    if (!p)
        return;

    p->which_payload_variant = meshtastic_MeshPacket_encrypted_tag;
    p->from = nodeDB->getNodeNum();
    p->to = NODENUM_BROADCAST;
    p->id = generatePacketId();
    p->channel = 0;
    p->has_rx_time = true;
    p->rx_time = getTime();
    p->has_rx_rssi = true;
    p->rx_rssi = rssi;
    p->rx_snr = snr;
    p->rx_frequency_error = freqErr;
    p->rx_coding_rate = codingRate;

    uint8_t *out = p->encrypted.bytes;
    memcpy(out, MAGIC, sizeof(MAGIC));
    out[4] = VERSION;
    out[5] = (uint8_t)type;
    out[6] = seq;
    out[7] = index;
    out[8] = count;
    memcpy(out + HEADER_LEN, data, len);
    p->encrypted.size = (pb_size_t)(HEADER_LEN + len);

    if (service)
        service->sendOpaqueToPhone(p);
    else
        packetPool.release(p);
}
} // namespace

void forward(Type type, const uint8_t *data, size_t len, int32_t rssi, float snr, float freqErr, uint8_t codingRate)
{
    if (!featureEnabled() || !data || len == 0)
        return;
    if (len > MAX_FRAME) {
        LOG_WARN("raw tunnel: dropping %u-byte frame (max %u)", (unsigned)len, (unsigned)MAX_FRAME);
        return;
    }

    const size_t count = (len + CHUNK_SIZE - 1) / CHUNK_SIZE;
    if (count == 0 || count > MAX_CHUNKS)
        return;
    const uint8_t seq = nextSeq();
    for (size_t i = 0, off = 0; i < count; i++, off += CHUNK_SIZE) {
        const size_t n = (len - off > CHUNK_SIZE) ? CHUNK_SIZE : (len - off);
        sendChunk(type, data + off, n, seq, (uint8_t)i, (uint8_t)count, rssi, snr, freqErr, codingRate);
    }
}

bool handleFromPhone(const struct _meshtastic_MeshPacket &p)
{
    if (p.which_payload_variant != meshtastic_MeshPacket_encrypted_tag)
        return false;
    if (p.encrypted.size < HEADER_LEN || memcmp(p.encrypted.bytes, MAGIC, sizeof(MAGIC)) != 0)
        return false;
    if (!featureEnabled()) {
        LOG_WARN("raw tunnel: packet dropped (RAW_TUNNEL_TO_PHONE is off)");
        return true;
    }
    if (p.encrypted.bytes[4] != VERSION) {
        LOG_WARN("raw tunnel: unsupported version %u", (unsigned)p.encrypted.bytes[4]);
        return true;
    }

    const Type type = (Type)p.encrypted.bytes[5];
    const uint8_t seq = p.encrypted.bytes[6];
    const uint8_t index = p.encrypted.bytes[7];
    const uint8_t count = p.encrypted.bytes[8];
    if (count == 0 || count > MAX_CHUNKS || index >= count) {
        LOG_WARN("raw tunnel: bad chunk header (index=%u count=%u)", (unsigned)index, (unsigned)count);
        return true;
    }

    const uint8_t *payload = p.encrypted.bytes + HEADER_LEN;
    const size_t payloadLen = p.encrypted.size - HEADER_LEN;
    if (payloadLen == 0)
        return true;

    if (count == 1) {
        (void)dispatch(type, payload, payloadLen);
        return true;
    }

    struct Reassembly {
        bool active = false;
        uint8_t type = 0;
        uint8_t seq = 0;
        uint8_t count = 0;
        uint8_t got = 0;
        size_t len = 0;
        uint32_t lastMs = 0;
        uint8_t data[MAX_FRAME] = {};
    };
    static Reassembly rx;
    auto reset = [&rx]() {
        rx.active = false;
        rx.count = 0;
        rx.got = 0;
        rx.len = 0;
    };

    const uint32_t now = Time::getMillis();
    if (rx.active && rx.lastMs && Throttle::hasElapsed(rx.lastMs, 5000))
        reset();
    if (!rx.active || rx.type != (uint8_t)type || rx.seq != seq || rx.count != count) {
        reset();
        rx.active = true;
        rx.type = (uint8_t)type;
        rx.seq = seq;
        rx.count = count;
    }

    const size_t off = (size_t)index * CHUNK_SIZE;
    if (off + payloadLen > sizeof(rx.data)) {
        LOG_WARN("raw tunnel: reassembly overflow");
        reset();
        return true;
    }
    memcpy(rx.data + off, payload, payloadLen);
    if (off + payloadLen > rx.len)
        rx.len = off + payloadLen;
    rx.got |= (uint8_t)(1u << index);
    rx.lastMs = now;

    const uint8_t all = (uint8_t)((1u << count) - 1);
    if (rx.got == all) {
        (void)dispatch(type, rx.data, rx.len);
        reset();
    }
    return true;
}
} // namespace nexus::rawtunnel

#endif
