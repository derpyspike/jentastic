#pragma once
#if RADIOLIB_EXCLUDE_LR2021 != 1
#include "RadioLibInterface.h"

#ifndef RADIOLIB_LR2021_MAX_PACKET_LENGTH
#define RADIOLIB_LR2021_MAX_PACKET_LENGTH 255
#endif

/**
 * \brief Adapter for LR20x0 radio family. Implements common logic for child classes.
 * \tparam T RadioLib module type for LR20x0, e.g. LR2021.
 */
template <class T> class LR20x0Interface : public RadioLibInterface
{
  public:
    LR20x0Interface(LockingArduinoHal *hal, RADIOLIB_PIN_TYPE cs, RADIOLIB_PIN_TYPE irq, RADIOLIB_PIN_TYPE rst,
                    RADIOLIB_PIN_TYPE busy);

    /// Initialise the Driver transport hardware and software.
    /// Make sure the Driver is properly configured before calling init().
    /// \return true if initialisation succeeded.
    virtual bool init() override;

    /// Apply any radio provisioning changes
    /// Make sure the Driver is properly configured before calling init().
    /// \return true if initialisation succeeded.
    virtual bool reconfigure() override;

    // Nexus SETFSK/SETFLRC (Phase 2): GFSK + FLRC programming for LR2021 (X1, sub-GHz FLRC).
    int16_t programAltModemFSK();
    int16_t programAltModemFLRC();
    uint8_t altModemCaps() const override { return 0x03; } // FSK + FLRC
    float altSensitivityDbm();

    /// Prepare hardware for sleep.  Call this _only_ for deep sleep, not needed for light sleep.
    virtual bool sleep() override;

    bool isIRQPending() override { return lora.getIrqFlags() != 0; }

#ifdef LR20X0_AGC_RESET
    void resetAGC() override;
#endif

    /** Nexus LCS: busy pin + software RX view (LRxxxx::getStatus is protected, Stat1 mode bits unconfirmed). */
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
    virtual void setRadioIsr(void (*callback)()) override { lora.setIrqAction(callback); }

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
    virtual uint8_t getLastRxSF() override;
    virtual float getLastRxFrequencyError() override;
    virtual uint8_t getLastRxCodingRate() override;
    virtual int16_t setSyncWordLive(uint8_t syncword) override;
    virtual uint8_t getSyncWord() const override;
    virtual uint8_t readRxTierProtocol() override;

    /// Main-detector syncword: the multi-SF main tier when enabled, else the
    /// normal Meshtastic/operator syncword.
    uint8_t effectiveMainSyncword() const;

    uint8_t getLastRxDetector() const { return lastRxDetector; }

  protected:
    uint8_t lastRxDetector = 0;

    /**
     * Apply the Semtech DCDC sensitivity workaround (opt-in, godmode-only). Must be called after the LoRa
     * modulation parameters have been set - i.e. after lora.begin() in init(), or after the
     * setSpreadingFactor/setBandwidth/setCodingRate calls in reconfigure(), all of which re-run
     * setLoRaModulationParams() and thereby reset the DCDC configure state. No-op unless built with
     * -DLR2021_DCDC_WORKAROUND (and RADIOLIB_GODMODE). Logs success/failure; never fatal.
     */
    void applyDcdcWorkaround();

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
    static const uint8_t FLRC_MAX_FRAME = 127;
    // The receiver reads each fragment from its main loop before the next one overwrites the chip's buffer
    static const uint8_t FRAG_GAP_MSEC = 10;

    uint8_t frameBuf[FLRC_MAX_FRAME];
    size_t txFragLen = 0;
    uint8_t txFragCount = 0, txFragNext = 0, txFragTag = 0;
    uint8_t rxFrag[sizeof(RadioBuffer)];
    size_t rxFragLen = 0;
    uint8_t rxFragTag = 0, rxFragCount = 0, rxFragNext = 0xFF;

    static bool usesFragments() { return meshtastic::nexusModule && meshtastic::nexusModule->altModem.mode == 2; }
    static size_t maxFrameLen() { return usesFragments() ? FLRC_MAX_FRAME : RADIOLIB_LR2021_MAX_PACKET_LENGTH; }
    static uint8_t fragmentCount(size_t len) { return (len + FLRC_MAX_FRAME - 2) / (FLRC_MAX_FRAME - 1); }
    int16_t transmitFragment();
    uint32_t flrcPacketTimeMsec(uint32_t pl);
    static constexpr uint16_t FLRC_MIN_PREAMBLE_BITS = 32;

  private:
    /** Chip-side re-init shared by the band-hop and recovery paths: front-end GPIOs, begin(), CRC, RF switch, RX gain */
    bool fullBegin(float freq);

    /** Board LF PA table after begin(); HF keeps RadioLib default. Warn-only on setOutputPower miss. */
    void applyCustomLfPaTable(float freq);

    /** setStandby()'s body, returning the standby error instead of asserting - for callers that can recover */
    int16_t trySetStandby();

    /** Recover a chip that lost its runtime state via the same full begin() the band-hop path uses */
    bool recoverChipStateLoss() override
    {
        if (!fullBegin(getFreq()))
            return false;
        if (meshtastic::nexusModule && meshtastic::nexusModule->altModem.mode != 0) {
            int16_t err = (meshtastic::nexusModule->altModem.mode == 1) ? programAltModemFSK()
                                                                        : programAltModemFLRC();
            return err == RADIOLIB_ERR_NONE;
        }
        return true;
    }
};
#endif
