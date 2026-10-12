#include "FloodingRouter.h"
#include "MeshTypes.h"
#include "NodeDB.h"
#include "configuration.h"
#include "mesh-pb-constants.h"
#include "meshUtils.h"
#include "modules/TextMessageModule.h"
#include "modules/NexusModule.h"
#if !MESHTASTIC_EXCLUDE_TRACEROUTE
#include "modules/TraceRouteModule.h"
#endif

// Nexus: unified input-firewall gate for the shouldFilterReceived() retransmit
// paths. These paths run BEFORE handleReceived()/dispatchReceived() so the
// firewall hooks in Router.cpp (shouldDropPacket / shouldDropPrivate) never
// fire on reliable retransmits without this helper. Dispatches on
// which_payload_variant to match the dispatchReceived() semantics:
//   - decoded_tag      → shouldDropPacket()  (per-port / RL / out-of-policy)
//   - encrypted_tag    → shouldDropPrivate() (opaque channel / PRIVATE / hop6/7)
// Returns true if Nexus says drop; safe no-op (returns false) when NexusModule
// is not compiled in / not yet initialized.
// Note: this calls the same helpers as dispatchReceived() and will tick
// fw_total_processed / updateStats a second time on a retransmit. updateStats
// is internally dedup'd on (id, from); fw_total_processed counts firewall
// invocations accurately ("we processed the firewall for this packet twice").
// The per-rule drop counters (e.g. fw_drop_nodeinfo) may inflate on
// retransmits, which is semantically truthful: the operator asked for no
// NodeInfo, and we refused 2 instances of it.
static inline bool nexusFirewallDrops(const meshtastic_MeshPacket &p)
{
    if (!meshtastic::nexusModule)
        return false;
    return (p.which_payload_variant == meshtastic_MeshPacket_encrypted_tag)
               ? meshtastic::nexusModule->shouldDropPrivate(p)
               : meshtastic::nexusModule->shouldDropPacket(p);
}

FloodingRouter::FloodingRouter() {}

/**
 * Send a packet on a suitable interface.  This routine will
 * later free() the packet to pool.  This routine is not allowed to stall.
 * If the txmit queue is full it might return an error
 */
ErrorCode FloodingRouter::send(meshtastic_MeshPacket *p)
{
    // Add any messages _we_ send to the seen message list (so we will ignore all retransmissions we see)
    p->relay_node = nodeDB->getLastByteOfNodeNum(getNodeNum()); // First set the relayer to us
    wasSeenRecently(p);                                         // FIXME, move this to a sniffSent method

    return Router::send(p);
}

bool FloodingRouter::shouldFilterReceived(const meshtastic_MeshPacket *p)
{
    bool wasUpgraded = false;
    bool seenRecently =
        wasSeenRecently(p, true, nullptr, nullptr, &wasUpgraded); // Updates history; returns false when an upgrade is detected

    // Handle hop_limit upgrade scenario for rebroadcasters
    if (wasUpgraded && perhapsHandleUpgradedPacket(p)) {
        return true; // we handled it, so stop processing
    }

    if (!seenRecently && !wasUpgraded && textMessageModule) {
        seenRecently = textMessageModule->recentlySeen(p->id);
    }

    if (seenRecently) {
        printPacket("Ignore dupe incoming msg", p);
        rxDupe++;

        /* If the original transmitter is doing retransmissions (hopStart equals hopLimit) for a reliable transmission, e.g., when
        the ACK got lost, we will handle the packet again to make sure it gets an implicit ACK. */
        bool isRepeated = p->hop_start > 0 && p->hop_start == p->hop_limit;
        if (isRepeated) {
            LOG_DEBUG("Repeated reliable tx");
            // Check if it's still in the Tx queue, if not, we have to relay it again
            if (!findInTxQueue(p->from, p->id)) {
                if (reprocessPacket(p) && !nexusFirewallDrops(*p))
                    perhapsRebroadcast(p);
            }
        } else {
            perhapsCancelDupe(p);
        }

        return true;
    }

    return Router::shouldFilterReceived(p);
}

bool FloodingRouter::perhapsHandleUpgradedPacket(const meshtastic_MeshPacket *p)
{
    // isRebroadcaster() is duplicated in perhapsRebroadcast(), but this avoids confusing log messages
    if (isRebroadcaster() && iface && p->hop_limit > 0) {
        // Verify the replacement before deleting the valid lower-hop copy waiting in the TX queue.
        // This is intentionally redundant with ReliableRouter's ingress gate: it keeps this helper
        // safe if another caller is introduced later.
        if (passesRoutingAuthGate(const_cast<meshtastic_MeshPacket *>(p)) != RoutingAuthVerdict::ACCEPT)
            return true;

        // If we overhear a duplicate copy of the packet with more hops left than the one we are waiting to
        // rebroadcast, then remove the packet currently sitting in the TX queue and use this one instead.
        uint8_t dropThreshold = p->hop_limit; // remove queued packets that have fewer hops remaining
        if (iface->removePendingTXPacket(getFrom(p), p->id, dropThreshold)) {
            LOG_DEBUG("Processing upgraded packet 0x%08x for rebroadcast with hop limit %d (dropping queued < %d)", p->id,
                      p->hop_limit, dropThreshold);

            if (!reprocessPacket(p))
                return true;
            // Nexus: input firewall gate before the upgraded-packet rebroadcast.
            // This branch runs inside shouldFilterReceived() (pre-handleReceived),
            // so without this gate /nexus FIREWALL DROP would be bypassed for the
            // replacement copy that goes out on the wire.
            if (!nexusFirewallDrops(*p))
                perhapsRebroadcast(p);

            rxDupe++;
            // We already enqueued the improved copy, so make sure the incoming packet stops here.
            return true;
        }
    }

    return false;
}

bool FloodingRouter::reprocessPacket(const meshtastic_MeshPacket *p)
{
    if (p->which_payload_variant != meshtastic_MeshPacket_decoded_tag) {
        auto decodedState = perhapsDecode(const_cast<meshtastic_MeshPacket *>(p));
        if (decodedState != DecodeState::DECODE_SUCCESS && decodedState != DecodeState::DECODE_OPAQUE)
            return false;
    }

    if (nodeDB)
        nodeDB->updateFrom(*p);

#if !MESHTASTIC_EXCLUDE_TRACEROUTE
    if (traceRouteModule && p->which_payload_variant == meshtastic_MeshPacket_decoded_tag &&
        p->decoded.portnum == meshtastic_PortNum_TRACEROUTE_APP) {
        traceRouteModule->processUpgradedPacket(*p);
    }
#endif
    return true;
}

bool FloodingRouter::roleAllowsCancelingDupe(const meshtastic_MeshPacket *p)
{
#if HAS_NEXUS
    if (meshtastic::nexusModule) {
        if (meshtastic::nexusModule->client_always) {
            return false; // Force router behavior: NEVER cancel a relay
        }

        uint8_t rid = (uint8_t)(p->relay_node & 0xFF);
        if (meshtastic::nexusModule->isIgnoreDupeRelay(rid)) {
            // IDR forced a relay through that the upstream dupe-cancel would have
            // dropped. Count it so the FWS IDR counter reflects real activity.
            meshtastic::nexusModule->fw_drop_idr++;
            return false; // Force router behavior for IDR nodes
        }

        if (meshtastic::nexusModule->base_texts && 
            p->decoded.portnum == meshtastic_PortNum_TEXT_MESSAGE_APP) {
            return false; // Force router behavior for TEXT messages only
        }
    }
#endif

    if (config.device.role == meshtastic_Config_DeviceConfig_Role_ROUTER ||
        config.device.role == meshtastic_Config_DeviceConfig_Role_ROUTER_LATE) {
        // ROUTER, ROUTER_LATE should never cancel relaying a packet (i.e. we should always rebroadcast),
        // even if we've heard another station rebroadcast it already.
        return false;
    }

    if (config.device.role == meshtastic_Config_DeviceConfig_Role_CLIENT_BASE) {
        // CLIENT_BASE: if the packet is from or to a favorited node,
        // we should act like a ROUTER and should never cancel a rebroadcast (i.e. we should always rebroadcast),
        // even if we've heard another station rebroadcast it already.
        return !nodeDB->isFromOrToFavoritedNode(*p);
    }
   

    // All other roles (such as CLIENT) should cancel a rebroadcast if they hear another station's rebroadcast.
    return true;
}

void FloodingRouter::perhapsCancelDupe(const meshtastic_MeshPacket *p)
{
    if (p->transport_mechanism == meshtastic_MeshPacket_TransportMechanism_TRANSPORT_LORA && roleAllowsCancelingDupe(p)) {
        // cancel rebroadcast of this message *if* there was already one, unless we're a router!
        // But only LoRa packets should be able to trigger this.
        if (Router::cancelSending(p->from, p->id))
            txRelayCanceled++;
    }
    if (config.device.role == meshtastic_Config_DeviceConfig_Role_ROUTER_LATE && iface) {
        iface->clampToLateRebroadcastWindow(getFrom(p), p->id);
    }
#if HAS_NEXUS
    // Nexus LATEBASE (2026-10-07): upstream v2.8.1 also clamps CLIENT_BASE
    // (favorited from/to) into the late-rebroadcast window — i.e. CLIENT_BASE
    // repeats with ROUTER_LATE timing. JT removed that block so CLIENT_BASE
    // repeats in the client/router window instead; feature bit LATEBASE restores
    // the upstream behavior on demand (default OFF = current JT behavior).
    if (meshtastic::nexusModule &&
        (meshtastic::nexusModule->getNexusFeaturesHex() & meshtastic::FEATURE_LATEBASE) &&
        config.device.role == meshtastic_Config_DeviceConfig_Role_CLIENT_BASE && iface && nodeDB &&
        nodeDB->isFromOrToFavoritedNode(*p)) {
        iface->clampToLateRebroadcastWindow(getFrom(p), p->id);
    }
#endif
}

bool FloodingRouter::isRebroadcaster()
{
    return config.device.role != meshtastic_Config_DeviceConfig_Role_CLIENT_MUTE &&
           config.device.rebroadcast_mode != meshtastic_Config_DeviceConfig_RebroadcastMode_NONE;
}

void FloodingRouter::sniffReceived(const meshtastic_MeshPacket *p, const meshtastic_Routing *c)
{
    bool isAckorReply = (p->which_payload_variant == meshtastic_MeshPacket_decoded_tag) &&
                        (p->decoded.request_id != 0 || p->decoded.reply_id != 0);
    if (isAckorReply && !isToUs(p) && !isBroadcast(p->to)) {
        // do not flood direct message that is ACKed or replied to
        LOG_DEBUG("Rxd an ACK/reply not for me, cancel rebroadcast");
        Router::cancelSending(p->to, p->decoded.request_id); // cancel rebroadcast for this DM
    }

    perhapsRebroadcast(p);

    // handle the packet as normal
    Router::sniffReceived(p, c);
}
