#pragma once
#include "configuration.h"
#if MESHTASTIC_ENABLE_JENCORE_SNIFFER
#include "SinglePortModule.h"

/**
 * Answers a "/stats" direct message with the channel-contention measurements of both protocols. Only DMs are served, from
 * Meshtastic nodes and from JenCore contacts alike (the reply leaves through MeshService::sendToMesh, which picks the
 * protocol), and replies are rate limited because every one of them is a radio transmission.
 */
class StatsCommandModule : public SinglePortModule
{
  public:
    StatsCommandModule() : SinglePortModule("stats", meshtastic_PortNum_TEXT_MESSAGE_APP) {}

  protected:
    bool wantPacket(const meshtastic_MeshPacket *p) override;
    ProcessMessage handleReceived(const meshtastic_MeshPacket &mp) override;
};
#endif
