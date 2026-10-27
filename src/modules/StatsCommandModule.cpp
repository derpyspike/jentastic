// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) 2026 Jennifer Sanchez
//
// Licensed under the GNU Affero General Public License v3.
// See LICENSE-AGPL for the full license text.
//

#include "StatsCommandModule.h"
#if NEXUS_ENABLE_STATSBOT && NEXUS_ENABLE_JENCORE_SNIFFER
#include "ContentionStats.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "Router.h"
#include "mesh/Throttle.h"

static constexpr uint32_t STATS_REPLY_MIN_INTERVAL_MS = 30 * 1000; // every reply is a transmission: serve one per 30 s at most

bool StatsCommandModule::wantPacket(const meshtastic_MeshPacket *p)
{
    return p && p->decoded.portnum == ourPortNum && p->to == nodeDB->getNodeNum();
}

ProcessMessage StatsCommandModule::handleReceived(const meshtastic_MeshPacket &mp)
{
    char msg[48] = {0}; // a longer text cannot be exactly "/stats"
    const size_t n = mp.decoded.payload.size < sizeof(msg) - 1 ? mp.decoded.payload.size : sizeof(msg) - 1;
    memcpy(msg, mp.decoded.payload.bytes, n);
    if (!isStatsCommand(msg))
        return ProcessMessage::CONTINUE;

    static uint32_t lastReplyMs = 0;
    static bool replied = false;
    if (replied && Throttle::isWithinTimespanMs(lastReplyMs, STATS_REPLY_MIN_INTERVAL_MS))
        return ProcessMessage::CONTINUE;

    RadioInterface *iface = router ? router->getRadioIface() : nullptr;
    meshtastic_MeshPacket *reply = iface ? allocDataPacket() : nullptr;
    if (!reply)
        return ProcessMessage::CONTINUE;

    char text[161];
    const size_t len = iface->formatContentionStats(text, sizeof(text));
    if (len == 0) {
        packetPool.release(reply);
        return ProcessMessage::CONTINUE;
    }
    reply->to = mp.from;
    reply->channel = mp.channel;
    reply->want_ack = false;
    reply->decoded.want_response = false;
    reply->decoded.payload.size = len;
    memcpy(reply->decoded.payload.bytes, text, len);
    service->sendToMesh(reply);

    lastReplyMs = millis();
    replied = true;
    return ProcessMessage::CONTINUE;
}
#endif
