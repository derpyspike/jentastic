#pragma once

#include "configuration.h"

#if MESHTASTIC_ENABLE_RNODE

#include <stddef.h>
#include <stdint.h>

// KISS framing constants
#define RNODE_FEND            0xC0
#define RNODE_FESC            0xDB
#define RNODE_TFEND           0xDC
#define RNODE_TFESC           0xDD

// KISS command bytes
#define RNODE_CMD_DATA        0x00
#define RNODE_CMD_FREQUENCY   0x01
#define RNODE_CMD_BANDWIDTH   0x02
#define RNODE_CMD_TXPOWER     0x03
#define RNODE_CMD_SF          0x04
#define RNODE_CMD_CR          0x05
#define RNODE_CMD_RADIO_STATE 0x06
#define RNODE_CMD_RADIO_LOCK  0x07
#define RNODE_CMD_DETECT      0x08
#define RNODE_CMD_IMPLICIT    0x09
#define RNODE_CMD_LEAVE       0x0A
#define RNODE_CMD_ST_ALOCK    0x0B
#define RNODE_CMD_LT_ALOCK    0x0C
#define RNODE_CMD_PROMISC     0x0E
#define RNODE_CMD_READY       0x0F

#define RNODE_CMD_STAT_RX     0x21
#define RNODE_CMD_STAT_TX     0x22
#define RNODE_CMD_STAT_RSSI   0x23
#define RNODE_CMD_STAT_SNR    0x24
#define RNODE_CMD_STAT_CHTM   0x25
#define RNODE_CMD_STAT_PHYPRM 0x26
#define RNODE_CMD_STAT_BAT    0x27
#define RNODE_CMD_STAT_CSMA   0x28
#define RNODE_CMD_STAT_TEMP   0x29
#define RNODE_CMD_BLINK       0x30
#define RNODE_CMD_RANDOM      0x40

#define RNODE_CMD_BOARD       0x47
#define RNODE_CMD_PLATFORM    0x48
#define RNODE_CMD_MCU         0x49
#define RNODE_CMD_FW_VERSION  0x50

#define RNODE_CMD_ERROR       0x90
#define RNODE_CMD_UNKNOWN     0xFE

// Protocol magic values
#define RNODE_DETECT_REQ      0x73
#define RNODE_DETECT_RESP     0x46

#define RNODE_RADIO_STATE_OFF 0x00
#define RNODE_RADIO_STATE_ON  0x01

#define RNODE_FLAG_SPLIT      0x01
#define RNODE_RSSI_OFFSET     157

// Platform and MCU identifiers
#define RNODE_PLATFORM_ESP32  0x38
#define RNODE_PLATFORM_NRF52  0x70

#define RNODE_MCU_ESP32       0x30
#define RNODE_MCU_NRF52840    0x71

// Supported operating modes
enum RNodeMode : uint8_t {
    RNODE_MODE_OFF = 0,
    RNODE_MODE_BRIDGE = 1,       // Concurrent Meshtastic + JenCore + Reticulum (Default)
    RNODE_MODE_STANDALONE = 2    // Dedicated 100% Reticulum RNode
};

enum RNodeClientType : uint8_t {
    RNODE_CLIENT_NONE   = 0,
    RNODE_CLIENT_TCP    = 1,
    RNODE_CLIENT_BLE    = 2,
    RNODE_CLIENT_SERIAL = 3
};

typedef void (*RNodeWriteFn)(const uint8_t *data, size_t len);

class RNodeEngine
{
  public:
    static constexpr size_t MAX_KISS_FRAME_LEN = 512;
    static constexpr size_t MAX_RADIO_FRAME_LEN = 256;

    RNodeEngine();

    void init();

    RNodeMode getMode() const { return mode; }
    void setMode(RNodeMode m) { mode = m; }

    uint8_t getSf() const { return lora_sf; }
    void setSf(uint8_t sf) { lora_sf = sf; }

    uint32_t getFrequency() const { return lora_freq; }
    void setFrequency(uint32_t freq) { lora_freq = freq; }

    uint32_t getBandwidth() const { return lora_bw; }
    void setBandwidth(uint32_t bw) { lora_bw = bw; }

    uint8_t getCr() const { return lora_cr; }
    void setCr(uint8_t cr) { lora_cr = cr; }

    uint8_t getTxPower() const { return lora_txp; }
    void setTxPower(uint8_t txp) { lora_txp = txp; }

    bool isTcpEnabled() const { return tcpEnabled; }
    void setTcpEnabled(bool en) { tcpEnabled = en; }

    bool isBleEnabled() const { return bleEnabled; }
    void setBleEnabled(bool en) { bleEnabled = en; }

    bool isTcpConnected() const { return tcpConnected; }
    void setTcpConnected(bool c) { tcpConnected = c; }

    bool isBleConnected() const { return bleConnected; }
    void setBleConnected(bool c) { bleConnected = c; }

    void registerTcpWriteCallback(RNodeWriteFn fn) { tcpWriteFn = fn; }
    void registerBleWriteCallback(RNodeWriteFn fn) { bleWriteFn = fn; }

    /** Feed byte stream from connected clients (TCP, BLE, Serial) */
    void feedBytes(const uint8_t *bytes, size_t len, RNodeClientType client);
    void feedByte(uint8_t b, RNodeClientType client);

    /** Handle inbound packet received from LoRa radio */
    bool handleReceivedRadioPacket(const uint8_t *raw, size_t len, float snr, int32_t rssi);

    /** Packet inspection: does this raw LoRa payload match Reticulum / RNode? */
    static bool looksLikeReticulumPacket(const uint8_t *raw, size_t len);

    /** Encapsulate and send a KISS frame to clients */
    void sendKissFrame(uint8_t cmd, const uint8_t *payload, size_t len, RNodeClientType targetClient = RNODE_CLIENT_NONE);

    // Statistics & Counters
    uint32_t rxPackets = 0;
    uint32_t txPackets = 0;
    uint32_t rxBytes = 0;
    uint32_t txBytes = 0;
    int32_t lastRssi = -100;
    float lastSnr = 0.0f;
    uint8_t lastSnrRaw = 0;

  private:
    void dispatchCommand(RNodeClientType client);
    void queueTxToRadio(const uint8_t *data, size_t len);

    RNodeMode mode = RNODE_MODE_BRIDGE;
    bool tcpEnabled = true;
    bool bleEnabled = true;
    bool tcpConnected = false;
    bool bleConnected = false;

    RNodeWriteFn tcpWriteFn = nullptr;
    RNodeWriteFn bleWriteFn = nullptr;

    // Modem parameter state (echoed or set)
    uint32_t lora_freq = 869618000; // Default Madrid 869.618 MHz
    uint32_t lora_bw = 62500;       // 62.5 kHz
    uint8_t lora_sf = 5;            // Reticulum Madrid Tier SF5
    uint8_t lora_cr = 5;            // 4/5
    uint8_t lora_txp = 14;          // 14 dBm
    uint8_t radio_state = RNODE_RADIO_STATE_ON;
    bool promisc = false;
    uint8_t tx_seq = 0;

    // Parser state for stream reconstruction
    struct ParserState {
        bool in_frame = false;
        bool escape = false;
        uint8_t command = RNODE_CMD_UNKNOWN;
        uint8_t buf[MAX_KISS_FRAME_LEN] = {0};
        size_t len = 0;
    };

    ParserState tcpParser;
    ParserState bleParser;

    // Split packet reassembly buffer
    uint8_t splitBuf[MAX_KISS_FRAME_LEN] = {0};
    size_t splitLen = 0;
    uint8_t splitSeq = 0xFF;
};

extern RNodeEngine rnodeEngine;

#endif // MESHTASTIC_ENABLE_RNODE
