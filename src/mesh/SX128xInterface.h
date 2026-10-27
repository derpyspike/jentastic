#pragma once

#include "RadioLibInterface.h"

#ifndef RADIOLIB_SX128X_MAX_PACKET_LENGTH
#define RADIOLIB_SX128X_MAX_PACKET_LENGTH 255
#endif

/**
 * \brief Adapter for SX128x radio family. Implements common logic for child classes.
 * \tparam T RadioLib module type for SX128x: SX1280.
 */
template <class T> class SX128xInterface : public RadioLibInterface
{
  public:
    SX128xInterface(LockingArduinoHal *hal, RADIOLIB_PIN_TYPE cs, RADIOLIB_PIN_TYPE irq, RADIOLIB_PIN_TYPE rst,
                    RADIOLIB_PIN_TYPE busy);

    /// Initialise the Driver transport hardware and software.
    /// Make sure the Driver is properly configured before calling init().
    /// \return true if initialisation succeeded.
    virtual bool init() override;

    virtual bool wideLora() override;

    /// SX128x is a 2.4 GHz-only chip; it cannot tune sub-GHz regions
    virtual bool supportsSubGhz() override { return false; }

    /// Apply any radio provisioning changes
    /// Make sure the Driver is properly configured before calling init().
    /// \return true if initialisation succeeded.
    virtual bool reconfigure() override;

    /// Prepare hardware for sleep.  Call this _only_ for deep sleep, not needed for light sleep.
    virtual bool sleep() override;

    bool isIRQPending() override { return lora.getIrqFlags() != 0; }

    /** Nexus LCS: chip state via GetStatus SPI (mode bits 7:5) + busy bit. */
    uint8_t readChipState() override;

  protected:
    /**
     * Specific module instance
     */
    T lora;

    int16_t getCurrentRSSI() override;

    /**
     * Glue functions called from ISR land
     */
    virtual void clearRadioIsr() override;

    /**
     * Enable a particular ISR callback glue function
     */
    virtual void setRadioIsr(void (*callback)()) override { lora.setDio1Action(callback); }

    /** can we detect a LoRa preamble on the current channel? */
    virtual bool isChannelActive() override;

    /** are we actively receiving a packet (only called during receiving state) */
    virtual bool isActivelyReceiving() override;

    /**
     * Start waiting to receive a message
     */
    virtual void startReceive() override;

    /**
     *  We override to turn on transmitter power as needed.
     */
    virtual void configHardwareForSend() override;

    /**
     * Add SNR data to received messages
     */
    virtual void addReceiveMetadata(meshtastic_MeshPacket *mp) override;

    virtual void setStandby() override;

    virtual int16_t setOutputPowerLive(int8_t powerDb) override;
    virtual int16_t setSpreadingFactorLive(uint8_t targetSf, int8_t targetPwr = 0x7F) override;
    bool supportsHybridSync() const override { return true; }

    uint32_t getPacketTime(uint32_t pl, bool received) override
    {
        return usesFragments() ? flrcPacketTimeMsec(pl) : computePacketTime(lora, pl, received);
    }

    // FLRC frames carry at most 127 bytes, so FLRC packets get a 1-byte fragment header and are split when needed.
    int16_t startTransmitFrame(size_t numbytes) override;
    uint32_t nextFrameDelayMsec() override;
    bool sendNextFrame() override;
    int readFrame(size_t &length) override;

  private:
#if NEXUS_ENABLE_JENCORE_HYBRID_SYNC
    /** Write a raw 16-bit value into the LoRa sync word register (e.g. 0x14B4 hybrid). */
    void setSyncWordRegister(uint16_t registerValue);
#endif
#if NEXUS_ENABLE_JENCORE_SNIFFER
    /** The chip holds JenCore's preamble length for a raw frame; startReceive() puts ours back. */
    bool jencorePreambleSet = false;
#endif

    static const uint8_t FLRC_MAX_FRAME = 127;
    // GFSK and FLRC sync word (SX128x-only links): 4 bytes, no long runs of equal bits
    static constexpr uint8_t sx128xAltSyncWord[4] = {0x2D, 0x4B, 0xD4, 0x93};
    // The receiver reads each fragment from its main loop before the next one overwrites the chip's buffer
    static const uint8_t FRAG_GAP_MSEC = 10;

    uint8_t frameBuf[FLRC_MAX_FRAME];
    size_t txFragLen = 0;
    uint8_t txFragCount = 0, txFragNext = 0, txFragTag = 0;
    uint8_t rxFrag[sizeof(RadioBuffer)];
    size_t rxFragLen = 0;
    uint8_t rxFragTag = 0, rxFragCount = 0, rxFragNext = 0xFF;

    static bool usesFragments() { return meshtastic::nexusModule && meshtastic::nexusModule->altModem.mode == 2; }
    static size_t maxFrameLen() { return usesFragments() ? FLRC_MAX_FRAME : RADIOLIB_SX128X_MAX_PACKET_LENGTH; }
    static uint8_t fragmentCount(size_t len) { return (len + FLRC_MAX_FRAME - 2) / (FLRC_MAX_FRAME - 1); }
    int16_t transmitFragment();
    uint32_t flrcPacketTimeMsec(uint32_t pl);
    float altSensitivityDbm();
    static constexpr uint16_t FLRC_MIN_PREAMBLE_BITS = 32;

    /** Program all modem parameters into the chip; returns the first RadioLib error, or RADIOLIB_ERR_NONE */
    int16_t programModemParams();

    // Nexus SETFSK/SETFLRC (Phase 2): GFSK + FLRC programming for SX128x (2.4 GHz).
    int16_t programAltModemFSK();
    int16_t programAltModemFLRC();
    uint8_t altModemCaps() const override { return 0x03; } // FSK + FLRC

    /** begin() and chip-side setup, shared by init() and by reconfigure()'s recovery of a chip that lost its state */
    /** @param fromInit true only for the boot-time call, which may adjust region and reboot on a
     *  2.4GHz-only part; a runtime recovery must never reboot the node. */
    bool reinitChip(bool fromInit = false);

    /** setStandby()'s body, returning the standby error instead of asserting - for callers that can recover */
    int16_t trySetStandby();

    /** Recover a chip that lost its runtime state: hardware-reset via begin() and reprogram */
    bool recoverChipStateLoss() override { return reinitChip() && programModemParams() == RADIOLIB_ERR_NONE; }
};
