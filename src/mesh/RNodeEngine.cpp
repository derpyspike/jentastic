#include "RNodeEngine.h"

#if MESHTASTIC_ENABLE_RNODE

#include "RadioLibInterface.h"
#include <Arduino.h>
#include <math.h>
#include <string.h>

RNodeEngine rnodeEngine;

RNodeEngine::RNodeEngine() {}

void RNodeEngine::init()
{
    mode = RNODE_MODE_BRIDGE;
    tcpEnabled = true;
    bleEnabled = true;
    tcpConnected = false;
    bleConnected = false;
    rxPackets = 0;
    txPackets = 0;
    rxBytes = 0;
    txBytes = 0;
    lastRssi = -100;
    lastSnr = 0.0f;
    lastSnrRaw = 0;
    lora_sf = 5; // Default Madrid SF5 Reticulum tier
}

void RNodeEngine::feedBytes(const uint8_t *bytes, size_t len, RNodeClientType client)
{
    if (!bytes || len == 0 || mode == RNODE_MODE_OFF)
        return;
    for (size_t i = 0; i < len; ++i) {
        feedByte(bytes[i], client);
    }
}

void RNodeEngine::feedByte(uint8_t b, RNodeClientType client)
{
    ParserState *p = (client == RNODE_CLIENT_BLE) ? &bleParser : &tcpParser;

    if (b == RNODE_FEND) {
        if (p->in_frame && p->command != RNODE_CMD_UNKNOWN) {
            dispatchCommand(client);
        }
        p->in_frame = true;
        p->command = RNODE_CMD_UNKNOWN;
        p->len = 0;
        p->escape = false;
        return;
    }

    if (!p->in_frame) {
        return; // Ignore bytes outside FEND delimiter
    }

    if (p->command == RNODE_CMD_UNKNOWN) {
        p->command = b;
        return;
    }

    // Escape character handling
    if (b == RNODE_FESC) {
        p->escape = true;
        return;
    }

    if (p->escape) {
        if (b == RNODE_TFEND)
            b = RNODE_FEND;
        else if (b == RNODE_TFESC)
            b = RNODE_FESC;
        p->escape = false;
    }

    if (p->len < MAX_KISS_FRAME_LEN) {
        p->buf[p->len++] = b;
    }
}

void RNodeEngine::dispatchCommand(RNodeClientType client)
{
    ParserState *p = (client == RNODE_CLIENT_BLE) ? &bleParser : &tcpParser;

    switch (p->command) {
    case RNODE_CMD_DETECT: {
        uint8_t resp = RNODE_DETECT_RESP;
        sendKissFrame(RNODE_CMD_DETECT, &resp, 1, client);
        break;
    }
    case RNODE_CMD_FW_VERSION: {
        uint8_t ver[2] = {0x01, 0x56}; // v1.86
        sendKissFrame(RNODE_CMD_FW_VERSION, ver, sizeof(ver), client);
        break;
    }
    case RNODE_CMD_PLATFORM: {
#if defined(ARCH_ESP32)
        uint8_t plat = RNODE_PLATFORM_ESP32;
#else
        uint8_t plat = RNODE_PLATFORM_NRF52;
#endif
        sendKissFrame(RNODE_CMD_PLATFORM, &plat, 1, client);
        break;
    }
    case RNODE_CMD_MCU: {
#if defined(ARCH_ESP32)
        uint8_t mcu = RNODE_MCU_ESP32;
#else
        uint8_t mcu = RNODE_MCU_NRF52840;
#endif
        sendKissFrame(RNODE_CMD_MCU, &mcu, 1, client);
        break;
    }
    case RNODE_CMD_FREQUENCY: {
        if (p->len == 4) {
            uint32_t freq = ((uint32_t)p->buf[0] << 24) | ((uint32_t)p->buf[1] << 16) |
                            ((uint32_t)p->buf[2] << 8) | (uint32_t)p->buf[3];
            if (freq != 0) {
                lora_freq = freq;
            }
        }
        uint8_t fbuf[4] = {(uint8_t)(lora_freq >> 24), (uint8_t)(lora_freq >> 16),
                           (uint8_t)(lora_freq >> 8), (uint8_t)lora_freq};
        sendKissFrame(RNODE_CMD_FREQUENCY, fbuf, 4, client);
        break;
    }
    case RNODE_CMD_BANDWIDTH: {
        if (p->len == 4) {
            uint32_t bw = ((uint32_t)p->buf[0] << 24) | ((uint32_t)p->buf[1] << 16) |
                          ((uint32_t)p->buf[2] << 8) | (uint32_t)p->buf[3];
            if (bw != 0) {
                lora_bw = bw;
            }
        }
        uint8_t bwbuf[4] = {(uint8_t)(lora_bw >> 24), (uint8_t)(lora_bw >> 16),
                            (uint8_t)(lora_bw >> 8), (uint8_t)lora_bw};
        sendKissFrame(RNODE_CMD_BANDWIDTH, bwbuf, 4, client);
        break;
    }
    case RNODE_CMD_TXPOWER: {
        if (p->len >= 1) {
            lora_txp = p->buf[0];
        }
        sendKissFrame(RNODE_CMD_TXPOWER, &lora_txp, 1, client);
        break;
    }
    case RNODE_CMD_SF: {
        if (p->len >= 1) {
            lora_sf = p->buf[0];
            if (mode == RNODE_MODE_STANDALONE) {
                RadioLibInterface *rli = static_cast<RadioLibInterface *>(RadioLibInterface::instance);
                if (rli)
                    rli->setSpreadingFactorLive(lora_sf);
            }
        }
        sendKissFrame(RNODE_CMD_SF, &lora_sf, 1, client);
        break;
    }
    case RNODE_CMD_CR: {
        if (p->len >= 1) {
            lora_cr = p->buf[0];
        }
        sendKissFrame(RNODE_CMD_CR, &lora_cr, 1, client);
        break;
    }
    case RNODE_CMD_RADIO_STATE: {
        if (p->len >= 1) {
            radio_state = p->buf[0];
        }
        sendKissFrame(RNODE_CMD_RADIO_STATE, &radio_state, 1, client);
        break;
    }
    case RNODE_CMD_RADIO_LOCK: {
        uint8_t lock = 0x00;
        sendKissFrame(RNODE_CMD_RADIO_LOCK, &lock, 1, client);
        break;
    }
    case RNODE_CMD_PROMISC: {
        if (p->len >= 1) {
            promisc = (p->buf[0] == 0x01);
        }
        uint8_t pr = promisc ? 0x01 : 0x00;
        sendKissFrame(RNODE_CMD_PROMISC, &pr, 1, client);
        break;
    }
    case RNODE_CMD_READY: {
        uint8_t ready = 0x01;
        sendKissFrame(RNODE_CMD_READY, &ready, 1, client);
        break;
    }
    case RNODE_CMD_STAT_RX: {
        uint8_t s[4] = {(uint8_t)(rxPackets >> 24), (uint8_t)(rxPackets >> 16),
                        (uint8_t)(rxPackets >> 8), (uint8_t)rxPackets};
        sendKissFrame(RNODE_CMD_STAT_RX, s, 4, client);
        break;
    }
    case RNODE_CMD_STAT_TX: {
        uint8_t s[4] = {(uint8_t)(txPackets >> 24), (uint8_t)(txPackets >> 16),
                        (uint8_t)(txPackets >> 8), (uint8_t)txPackets};
        sendKissFrame(RNODE_CMD_STAT_TX, s, 4, client);
        break;
    }
    case RNODE_CMD_STAT_RSSI: {
        uint8_t r = (uint8_t)(lastRssi + RNODE_RSSI_OFFSET);
        sendKissFrame(RNODE_CMD_STAT_RSSI, &r, 1, client);
        break;
    }
    case RNODE_CMD_STAT_SNR: {
        sendKissFrame(RNODE_CMD_STAT_SNR, &lastSnrRaw, 1, client);
        break;
    }
    case RNODE_CMD_RANDOM: {
        uint8_t rnd = (uint8_t)random(256);
        sendKissFrame(RNODE_CMD_RANDOM, &rnd, 1, client);
        break;
    }
    case RNODE_CMD_DATA: {
        if (p->len > 0) {
            queueTxToRadio(p->buf, p->len);
        }
        break;
    }
    default:
        break;
    }
}

void RNodeEngine::queueTxToRadio(const uint8_t *data, size_t len)
{
    if (!data || len == 0 || mode == RNODE_MODE_OFF)
        return;

    RadioLibInterface *rli = static_cast<RadioLibInterface *>(RadioLibInterface::instance);
    if (!rli)
        return;

    if (!promisc) {
        // Standard RNode mode: prepend 1-byte radio header (sequence + split flag)
        uint8_t txBuf[MAX_RADIO_FRAME_LEN];
        if (len + 1 > MAX_RADIO_FRAME_LEN)
            return;
        txBuf[0] = (tx_seq++ << 4) & 0xF0;
        memcpy(&txBuf[1], data, len);
        rli->queueRNodeFrame(txBuf, len + 1);
    } else {
        if (len > MAX_RADIO_FRAME_LEN)
            return;
        rli->queueRNodeFrame(data, len);
    }
}

bool RNodeEngine::handleReceivedRadioPacket(const uint8_t *raw, size_t len, float snr, int32_t rssi)
{
    if (!raw || len == 0 || mode == RNODE_MODE_OFF)
        return false;

    if (mode == RNODE_MODE_BRIDGE && !looksLikeReticulumPacket(raw, len)) {
        return false;
    }

    lastSnr = snr;
    lastSnrRaw = (uint8_t)round(snr * 4.0f);
    lastRssi = rssi;
    rxPackets++;
    rxBytes += len;

    uint8_t rssiVal = (uint8_t)(lastRssi + RNODE_RSSI_OFFSET);
    sendKissFrame(RNODE_CMD_STAT_RSSI, &rssiVal, 1);
    sendKissFrame(RNODE_CMD_STAT_SNR, &lastSnrRaw, 1);

    if (!promisc && len >= 1) {
        // Strip the 1-byte radio header
        sendKissFrame(RNODE_CMD_DATA, &raw[1], len - 1);
    } else {
        sendKissFrame(RNODE_CMD_DATA, raw, len);
    }

    return true;
}

bool RNodeEngine::looksLikeReticulumPacket(const uint8_t *raw, size_t len)
{
    if (!raw)
        return false;

    // Case 1: Standard RNode radio frame with 1-byte header: total len >= 19
    if (len >= 19) {
        uint8_t rnode_hdr = raw[0];
        // Lower nibble: 0x00 (single) or 0x01 (split); bits 1..3 must be 0
        if ((rnode_hdr & 0x0E) == 0) {
            uint8_t flags = raw[1];
            uint8_t hops = raw[2];
            uint8_t prop_type = (flags >> 5) & 0x03;
            // Propagation type: 0 (broadcast), 1 (transport), 2 (relayed); 3 is reserved
            if (prop_type <= 2 && hops <= 64) {
                return true;
            }
        }
    }

    // Case 2: Promiscuous raw Reticulum frame (no RNode 1-byte header): len >= 18
    if (len >= 18) {
        uint8_t flags = raw[0];
        uint8_t hops = raw[1];
        uint8_t prop_type = (flags >> 5) & 0x03;
        if (prop_type <= 2 && hops <= 64) {
            return true;
        }
    }

    return false;
}

void RNodeEngine::sendKissFrame(uint8_t cmd, const uint8_t *payload, size_t len, RNodeClientType targetClient)
{
    uint8_t out[MAX_KISS_FRAME_LEN * 2];
    size_t idx = 0;

    out[idx++] = RNODE_FEND;
    out[idx++] = cmd;

    for (size_t i = 0; i < len; ++i) {
        if (idx >= sizeof(out) - 3)
            break;
        uint8_t b = payload[i];
        if (b == RNODE_FEND) {
            out[idx++] = RNODE_FESC;
            out[idx++] = RNODE_TFEND;
        } else if (b == RNODE_FESC) {
            out[idx++] = RNODE_FESC;
            out[idx++] = RNODE_TFESC;
        } else {
            out[idx++] = b;
        }
    }

    out[idx++] = RNODE_FEND;

    if (tcpWriteFn && (targetClient == RNODE_CLIENT_NONE || targetClient == RNODE_CLIENT_TCP)) {
        tcpWriteFn(out, idx);
    }
    if (bleWriteFn && (targetClient == RNODE_CLIENT_NONE || targetClient == RNODE_CLIENT_BLE)) {
        bleWriteFn(out, idx);
    }
}

#endif // MESHTASTIC_ENABLE_RNODE
