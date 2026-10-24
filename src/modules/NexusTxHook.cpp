// SPDX-License-Identifier: AGPL-3.0-only
#include "NexusTxHook.h"
#include "NexusModule.h"
#include "NodeDB.h"
#include "Default.h"
#include "configuration.h"
#include "RadioLibInterface.h"

NexusTxOverride NexusTxHook::overrides[MAX_OVERRIDES];
NexusTxHook *nexusTxHook = nullptr;

NexusTxHook::NexusTxHook()
{
}

void NexusTxHook::setOverride(PacketId id, uint8_t sf, int8_t power)
{
    if (id == 0) return;
    NexusTxOverride *slot = nullptr;
    for (auto &entry : overrides) {
        if (entry.inUse && entry.id == id) {
            slot = &entry;
            break;
        }
        if (!slot && !entry.inUse)
            slot = &entry;
    }
    if (!slot)
        slot = &overrides[0]; // evict oldest
    slot->inUse = true;
    slot->id = id;
    slot->sf = sf;
    slot->power = power;
    slot->isFastPath = true;
}

void NexusTxHook::setOverrideFull(PacketId id, uint8_t sf, uint16_t bw, uint8_t cr, float freq, int8_t power)
{
    if (id == 0) return;
    NexusTxOverride *slot = nullptr;
    for (auto &entry : overrides) {
        if (entry.inUse && entry.id == id) {
            slot = &entry;
            break;
        }
        if (!slot && !entry.inUse)
            slot = &entry;
    }
    if (!slot)
        slot = &overrides[0];
    slot->inUse = true;
    slot->id = id;
    slot->sf = sf;
    slot->bw = bw;
    slot->cr = cr;
    slot->freq = freq;
    slot->power = power;
    slot->isFastPath = false;
}

bool NexusTxHook::hasOverride(PacketId id)
{
    if (id == 0) return false;
    for (const auto &entry : overrides) {
        if (entry.inUse && entry.id == id)
            return true;
    }
    return false;
}

void NexusTxHook::clearOverride(PacketId id)
{
    if (id == 0) return;
    for (auto &entry : overrides) {
        if (entry.inUse && entry.id == id) {
            entry.inUse = false;
            return;
        }
    }
}

RadioTxHook::PreTxAction NexusTxHook::beforeTransmit(RadioInterface *iface, meshtastic_MeshPacket *p)
{
    if (!iface || !p)
        return PRETX_SEND;

    // Check if an override is registered for this packet
    for (const auto &entry : overrides) {
        if (entry.inUse && entry.id == p->id) {
            if (entry.isFastPath) {
                if (!fastSwitched) {
                    originalSf = iface->getSF();
                    originalPower = iface->getPower();
                    iface->setSpreadingFactorLive(entry.sf, entry.power);
                    fastSwitched = true;
                    activeSwitchedId = p->id;
                    LOG_DEBUG("NexusTxHook: fast-path switch for 0x%08x -> SF%u, pwr=%d dBm", p->id, entry.sf, entry.power);
                }
                return PRETX_SEND;
            } else {
                if (!fullSwitched) {
                    originalLora = config.lora;
                    originalPrimaryChannel = channels.getPrimary();
                    originalPower = iface->getPower();

                    // Apply the FULL secondary preset (F1 fix, Jenny 2026-10-23): switch every
                    // modulation setting, not just the carrier. use_preset=false so the sf/bw/cr
                    // fields are honored — a preset-mode baseline ignores them otherwise.
                    // Region and base channel stay: override_frequency governs the carrier and
                    // duty-cycle accounting stays on the baseline region.
                    config.lora.use_preset = false;
                    config.lora.spread_factor = entry.sf;
                    config.lora.bandwidth = entry.bw;
                    config.lora.coding_rate = entry.cr;
                    config.lora.override_frequency = entry.freq;
                    iface->reconfigure();
                    if (entry.power != 0x7F) {
                        iface->setOutputPowerLive(entry.power);
                    }
                    fullSwitched = true;
                    activeSwitchedId = p->id;
                    LOG_INFO("NexusTxHook: full-path switch for 0x%08x -> freq=%.3f, SF%u, pwr=%d dBm",
                             p->id, entry.freq, entry.sf, entry.power);
                    return PRETX_DEFER;
                }
                return PRETX_SEND;
            }
        }
    }

    // APC (Automatic Power Control): applies to unicast packets when enabled
    if (meshtastic::nexusModule && meshtastic::nexusModule->isApcEnabled() && !isBroadcast(p->to) && p->to != 0) {
        if (!powerSwitched) {
            if (const meshtastic_NodeInfoLite *node = nodeDB->getMeshNode(p->to)) {
                if (node->snr > 4.0f && node->hops_away == 0) {
                    int8_t defPwr = iface->getPower();
                    int8_t targetPwr = defPwr - (int8_t)(node->snr - 4.0f);
                    int8_t minPwr = meshtastic::nexusModule->getApcMinPower();
                    if (targetPwr < minPwr) targetPwr = minPwr;
                    if (targetPwr < defPwr) {
                        originalPower = defPwr;
                        iface->setOutputPowerLive(targetPwr);
                        powerSwitched = true;
                        activeSwitchedId = p->id;
                        LOG_DEBUG("NexusTxHook: APC step-down for 0x%08x (SNR=%.1f dB) -> %d dBm (was %d dBm)",
                                  p->id, node->snr, targetPwr, defPwr);
                    }
                }
            }
        }
    }

    return PRETX_SEND;
}

bool NexusTxHook::holdsRadio(const meshtastic_MeshPacket *p)
{
    return p && fullSwitched && activeSwitchedId == p->id;
}

void NexusTxHook::packetReleased(RadioInterface *iface, const meshtastic_MeshPacket *p)
{
    if (!iface || !p)
        return;

    if (fastSwitched && activeSwitchedId == p->id) {
        iface->setSpreadingFactorLive(originalSf, originalPower);
        fastSwitched = false;
        activeSwitchedId = 0;
        LOG_DEBUG("NexusTxHook: fast-path restore to SF%u, pwr=%d dBm", originalSf, originalPower);
    } else if (fullSwitched && activeSwitchedId == p->id) {
        // F1 fix: whole-struct restore — brings override_frequency back too (it was leaked,
        // leaving the radio on the alt carrier after every full-path packet).
        config.lora = originalLora;
        channels.getByIndex(channels.getPrimaryIndex()).settings = originalPrimaryChannel;
        channels.fixupChannel(channels.getPrimaryIndex());
        iface->reconfigure();
        fullSwitched = false;
        activeSwitchedId = 0;
        LOG_INFO("NexusTxHook: full-path restore complete");
    } else if (powerSwitched && activeSwitchedId == p->id) {
        iface->setOutputPowerLive(originalPower);
        powerSwitched = false;
        activeSwitchedId = 0;
        LOG_DEBUG("NexusTxHook: APC power restore to %d dBm", originalPower);
    }

    clearOverride(p->id);
}
