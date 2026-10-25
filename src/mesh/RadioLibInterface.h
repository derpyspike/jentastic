#pragma once

#include "MeshPacketQueue.h"
#include "RadioInterface.h"
#include "concurrency/NotifiedWorkerThread.h"

#if MESHTASTIC_ENABLE_JENCORE_SNIFFER
#include "JenCoreRepeater.h"
#endif
#include "modules/NexusModule.h"

#include <RadioLib.h>
#include <sys/types.h>

// ESP32 has special rules about ISR code
#ifdef ARDUINO_ARCH_ESP32
#define INTERRUPT_ATTR IRAM_ATTR
#else
#define INTERRUPT_ATTR
#endif

#define RADIOLIB_PIN_TYPE uint32_t

// In addition to the default Rx flags, we need the PREAMBLE_DETECTED flag to detect whether we are actively receiving
#define MESHTASTIC_RADIOLIB_IRQ_RX_FLAGS (RADIOLIB_IRQ_RX_DEFAULT_FLAGS | (1 << RADIOLIB_IRQ_PREAMBLE_DETECTED))

#define AGC_RESET_INTERVAL_MS (60 * 1000) // 60 seconds

// Nexus LORACHIPSTATS: chip-state sample cadence (internal throttle inside sampleChipState())
#define CHIP_STATE_SAMPLE_INTERVAL_MS (1000)

/**
 * We need to override the RadioLib ArduinoHal class to add mutex protection for SPI bus access
 */
class LockingArduinoHal : public ArduinoHal
{
  public:
    LockingArduinoHal(SPIClass &spi, SPISettings spiSettings) : ArduinoHal(spi, spiSettings) {};

    void spiBeginTransaction() override;
    void spiEndTransaction() override;
#if ARCH_PORTDUINO
    void spiTransfer(uint8_t *out, size_t len, uint8_t *in) override;

#endif
};

// TCXO_OPTIONAL (variant define) or Lora.TCXO_OPTIONAL (Portduino YAML): probe for a TCXO and
// fall back to the XTAL. LR11x0 tries XTAL first - TCXO-first hangs RadioLib's calibration wait.
#if ARCH_PORTDUINO
#define TCXO_OPTIONAL_ENABLED (portduino_config.tcxo_optional)
#elif defined(TCXO_OPTIONAL)
#define TCXO_OPTIONAL_ENABLED true
#else
#define TCXO_OPTIONAL_ENABLED false
#endif

// RadioLib's own default Vref, for a probe with no explicit voltage configured.
#define TCXO_OPTIONAL_DEFAULT_VOLTAGE 1.6f

#if MESHTASTIC_ENABLE_JENCORE_SNIFFER
class RadioLibInterface;

/**
 * Owns the wake-ups of the JenCore relay. The radio thread has a single notification slot, shared with the Meshtastic
 * TX timer: a relay timer armed there either displaces the TX timer or is displaced by it. A thread of its own keeps
 * the two schedules independent; both still serialise on the radio, which only the radio thread and this one touch.
 */
class JenCoreRelayThread : public concurrency::OSThread
{
  public:
    explicit JenCoreRelayThread(RadioLibInterface *radio);

    /** Run the relay step in ms (idle until then if nothing else wakes it). */
    void wakeIn(uint32_t ms);

    /** True while a wake-up is scheduled. */
    bool isArmed() const { return enabled; }

  protected:
    int32_t runOnce() override;

  private:
    RadioLibInterface *radio;
};
#endif

#if defined(USE_STM32WLx)
/**
 * A wrapper for the RadioLib STM32WLx_Module class, that doesn't connect any pins as they are virtual
 */
class STM32WLx_ModuleWrapper : public STM32WLx_Module
{
  public:
    STM32WLx_ModuleWrapper(LockingArduinoHal *hal, RADIOLIB_PIN_TYPE cs, RADIOLIB_PIN_TYPE irq, RADIOLIB_PIN_TYPE rst,
                           RADIOLIB_PIN_TYPE busy)
        : STM32WLx_Module() {};
};
#endif

// Nexus port (Nexus-28001): forward-declare NexusModule at file scope (see NodeDB.h for rationale).
namespace meshtastic { class NexusModule; }

class RadioLibInterface : public RadioInterface, protected concurrency::NotifiedWorkerThread
{
    // Nexus port: NexusModule reads getInstantaneousRSSI() (a protected member) via
    // RadioLibInterface::instance to power /nexus RSSI. Friend is preferred over moving
    // the method to public — the protected access keeps the API surface narrow.
    friend class meshtastic::NexusModule;

    MeshPacketQueue txQueue = MeshPacketQueue(MAX_TX_QUEUE);

  protected:
    /// Used as our notification from the ISR
    enum PendingISR { ISR_NONE = 0, ISR_RX, ISR_TX, TRANSMIT_DELAY_COMPLETED, ISR_POLL_TICK, NEXT_FRAME };

    /**
     * Raw ISR handler that just calls our polymorphic method
     */
    static void isrTxLevel0(), isrLevel0Common(PendingISR code);

    ModemType_t modemType = RADIOLIB_MODEM_LORA;
    DataRate_t getDataRate() const { return {.lora = {.spreadingFactor = sf, .bandwidth = bw, .codingRate = cr}}; }
    PacketConfig_t getPacketConfig() const
    {
        return {.lora = {.preambleLength = preambleLength,
                         .implicitHeader = false,
                         .crcEnabled = true,
                         // We use auto LDRO, meaning it is enabled if the symbol time is >= 16msec
                         .ldrOptimize = (1 << sf) / bw >= 16}};
    }

    /**
     * We use a meshtastic sync word, but hashed with the Channel name.  For releases before 1.2 we used 0x12 (or for very old
     * loads 0x14) Note: do not use 0x34 - that is reserved for lorawan
     *
     * We now use 0x2b (so that someday we can possibly use NOT 2b - because that would be funny pun).  We will be staying with
     * this code for a long time.
     *
     * Nexus port (Nexus-28001, 2026-08-16): the `const` was removed so NexusModule can override the
     * sync word at runtime via /nexus SETSYNCWORD. The override is applied at the top of
     * RadioInterface::applyModemConfig() (which runs at boot AND on every LoRa config change,
     * including the deferred apply triggered by SETSYNCWORD itself). Nexus persists the value
     * in vault v40 and loads it via NexusModule::loadVault() during setupModules() — which
     * runs before initLoRa() per main.cpp ordering, so the override is in place before
     * rIf->init() reads syncWord for the first time.
     */
    uint8_t syncWord = 0x2b;

    /** JenCore's sync word (RADIOLIB_SX126X_SYNC_WORD_PRIVATE), used to transmit relayed JenCore frames. */
    const uint8_t jencoreSyncWord = 0x12;

#if MESHTASTIC_ENABLE_JENCORE_HYBRID_SYNC
    bool hybridSyncEnabled = true;
#endif

    float currentLimit = 100; // 100mA OCP - Should be acceptable for RFM95/SX127x chipset.

#if !defined(USE_STM32WLx)
    Module module; // The HW interface to the radio
#else
    STM32WLx_ModuleWrapper module;
#endif

    /**
     * provides lowest common denominator RadioLib API
     */
    PhysicalLayer *iface;

    /// are _trying_ to receive a packet currently (note - we might just be waiting for one)
    bool isReceiving = false;

    /// has the radio IRQ ever been armed? latches true and is never cleared, so ISR context only reads it
    volatile bool isrEverArmed = false;

  protected:
    // Noise floor tracking - rolling window of samples.
    static const uint8_t NOISE_FLOOR_SAMPLES = 20;
    static const int32_t NOISE_FLOOR_DEFAULT = -120;
    static const int32_t NOISE_FLOOR_VALID_MIN = -127;
    static const int32_t NOISE_FLOOR_INVALID = -128;
    int32_t noiseFloorSamples[NOISE_FLOOR_SAMPLES];
    uint8_t currentSampleIndex = 0;
    bool isNoiseFloorBufferFull = false;
    uint32_t lastNoiseFloorUpdate = 0;
    static const uint32_t NOISE_FLOOR_UPDATE_INTERVAL_MS = 5000;
    int32_t currentNoiseFloor = NOISE_FLOOR_DEFAULT;

    /**
     * Pure virtual hook for derived radio interfaces to provide instantaneous RSSI.
     * Implementations should return dBm, or an invalid value that updateNoiseFloor()
     * can reject.
     */
    virtual int16_t getCurrentRSSI() = 0;

    // Nexus port: instantaneous RSSI as float dBm, kept for /nexus RSSI command which
    // surfaces the live reading rather than the noise-floor-compensated getCurrentRSSI.
    // The Nexus cherry-pick auto-merged the .cpp definition but lost this declaration
    // because RadioLibInterface.h was in the conflict set. Restoring here so the
    // SX126xInterface override below compiles cleanly. Default body in RadioLibInterface.cpp.
    // Made public (with re-declaration below in the public block) so NexusModule can read it
    // via RadioLibInterface::instance->getInstantaneousRSSI() — the protected access here
    // blocked NexusModule from seeing it without the friend declaration.
    virtual float getInstantaneousRSSI();

  public:
    /** Our ISR code currently needs this to find our active instance
     */
    static RadioLibInterface *instance;

    /** Clear instance on destruction so stale pointer checks in loop() are safe */
    virtual ~RadioLibInterface()
    {
        if (instance == this)
            instance = nullptr;
    }

    /**
     * Nexus-28002 (2026-08-21): syncword getter for /nexus LRS.
     * Returns the actual byte programmed into the LoRa chip — either the
     * Meshtastic default 0x2B or NexusModule::syncword_override (vault v40).
     * Source of truth is the `syncWord` member; /nexus LRS MUST read this
     * instead of NexusModule::syncword_override directly so the chip-actual
     * byte is reported (Nexus-269 design, commit 86e833112).
     */
    [[nodiscard]] uint8_t getSyncWord() const { return syncWord; }

    /**
     * Get the current calculated noise floor in dBm
     * Returns -120 dBm if not yet calibrated
     */
    int32_t getNoiseFloor();

    /**
     * Calculate the average noise floor from collected samples
     */
    int32_t getAverageNoiseFloor();

    /**
     * Glue functions called from ISR land
     *
     * Skip the detach until the IRQ has been armed once: the first setStandby() runs before any
     * enableInterrupt(), and ESP-IDF logs "GPIO isr service is not installed" for that call.
     */
    void disableInterrupt()
    {
        if (!isrEverArmed)
            return;
        clearRadioIsr();
    }

    /**
     * Enable a particular ISR callback glue function
     */
    void enableInterrupt(void (*callback)())
    {
        // Latch before arming: the ISR can fire the moment the handler is installed.
        isrEverArmed = true;
        setRadioIsr(callback);
    }

    /**
     * Poll as a backup to catch missed edge-triggered interrupts.
     */
    void pollMissedIrqs();

    /**
     * Reset AGC by power-cycling the analog frontend.
     * Subclasses override with chip-specific calibration sequences.
     * Safe to call periodically - skips if currently sending or receiving.
     */
    virtual void resetAGC();

    /** Periodic radio upkeep: re-arms RX if a failed startReceive() left it off, otherwise resets AGC. */
    void periodicRadioMaintenance();

    /** Chip-specific recovery of a chip that lost its state to a reset/brownout. Returns true if reprogrammed. */
    virtual bool recoverChipStateLoss() { return false; }

    /** Throttled recoverChipStateLoss(), so a dead chip can't stall the RX/TX hot paths with repeated begin(). */
    bool maybeRecoverChipStateLoss();

    uint32_t lastChipRecoveryMs = 0;

    /// Consecutive recovery attempts that never got RX armed again, before rebooting to re-run init()
    static constexpr uint8_t MAX_CHIP_RECOVERY_FAILURES = 5;
    uint8_t chipRecoveryFailures = 0;

    /// Set by a driver's startReceive() when it gives up and leaves RX off; cleared once RX is armed again.
    bool rxOffline = false;

    /**
     * Debugging counts
     */
    uint32_t rxBad = 0, rxGood = 0, txGood = 0, txRelay = 0;
    uint16_t txDrop = 0;

#if MESHTASTIC_ENABLE_JENCORE_SNIFFER
    // Meshtastic side of the contention measurements (JenCore's live in jencoreRepeater): CAD hits before a send, and airtime.
    uint32_t meshtasticCadBusy = 0, meshtasticTxAirtimeMs = 0, meshtasticRxAirtimeMs = 0;
#endif

    // Nexus LORACHIPSTATS (/nexus LCS): cumulative chip-state samples, one per
    // CHIP_STATE_SAMPLE_INTERVAL_MS (RAM-only, cleared on boot or LCS RESET).
    enum ChipStateSample : uint8_t {
        CHIP_TX = 0,   // sendingPacket in flight (software; no SPI touch during TX)
        CHIP_RX,       // chip status reports RX mode (or LR: RX armed in software)
        CHIP_STDBY,    // chip standby
        CHIP_FS,       // frequency synthesiser (FSRX/FSTX, between STDBY and RX/TX)
        CHIP_SLP,      // chip sleep (RF95 reads this from OP_MODE; others never sleep while running)
        CHIP_BUSY,     // BUSY pin high at sample time (chip mid-command)
        CHIP_RXOFF,    // rxOffline: RX re-arm failed, recovery ladder active
        CHIP_UNK,      // SPI read failed / mode not readable on this family
        CHIP_STATE_COUNT
    };
    uint32_t chipStateCount[CHIP_STATE_COUNT] = {0};
    uint32_t chipStateTotal = 0;
    uint32_t lastChipStateMs = 0;

    /** Take one throttled chip-state sample (called at ~1 Hz from NexusModule::runOnce()). */
    void sampleChipState();

    /** Read the chip's current state; returns a ChipStateSample. Base impl: CHIP_UNK. */
    virtual uint8_t readChipState();

  public:
    RadioLibInterface(LockingArduinoHal *hal, RADIOLIB_PIN_TYPE cs, RADIOLIB_PIN_TYPE irq, RADIOLIB_PIN_TYPE rst,
                      RADIOLIB_PIN_TYPE busy, PhysicalLayer *iface = NULL);

    virtual ErrorCode send(meshtastic_MeshPacket *p) override;

    /**
     * Return true if we think the board can go to sleep (i.e. our tx queue is empty, we are not sending or receiving)
     *
     * This method must be used before putting the CPU into deep or light sleep.
     * With deepSleep set, an in-flight transmission also vetoes sleep (see RadioInterface).
     */
    virtual bool canSleep(bool deepSleep) override;

    /**
     * Start waiting to receive a message
     *
     * External functions can call this method to wake the device from sleep.
     * Subclasses must override and call this base method
     */
    virtual void startReceive();

    /** can we detect a LoRa preamble on the current channel? */
    virtual bool isChannelActive() = 0;

    /** are we actively receiving a packet (only called during receiving state)
     *  This method is only public to facilitate debugging.  Do not call.
     */
    virtual bool isActivelyReceiving() = 0;

    /** Are we are currently sending a packet?
     * This method is public, intending to expose this information to other firmware components
     */
    virtual bool isSending();

    /** Attempt to cancel a previously sent packet.  Returns true if a packet was found we could cancel */
    virtual bool cancelSending(NodeNum from, PacketId id) override;

    /** Attempt to find a packet in the TxQueue. Returns true if the packet was found. */
    virtual bool findInTxQueue(NodeNum from, PacketId id) override;

    /**
     * Update the noise floor measurement by sampling RSSI from a slow path.
     * This should not be called from radio interrupt or TX/RX critical paths.
     */
    void updateNoiseFloor();

    /**
     * Check if we have collected any noise floor samples
     */
    bool hasNoiseFloorSamples();

    /**
     * Get the number of samples in the rolling window
     */
    uint8_t getNoiseFloorSampleCount();

    /**
     * Reset the noise floor calibration
     * Will automatically restart collection
     */
    void resetNoiseFloor();

    /**
     * Request randomness sourced from the LoRa modem, if supported by the active RadioLib interface.
     * @return true if len bytes were produced, false otherwise.
     */
    bool randomBytes(uint8_t *buffer, size_t length);

  private:
    uint8_t getNoiseFloorSampleCountInternal() const;
    int32_t getAverageNoiseFloorInternal() const;

    /** if we have something waiting to send, start a short (random) timer so we can come check for collision before actually
     * doing the transmit */
    void setTransmitDelay();

    /**
     * random timer with certain min. and max. settings
     * @return Timestamp after which the packet may be sent
     */
    void startTransmitTimer(bool withDelay = true);

    /**
     * timer scaled to SNR of to be flooded packet
     * @return Timestamp after which the packet may be sent
     */
    void startTransmitTimerRebroadcast(meshtastic_MeshPacket *p);

    void handleTransmitInterrupt();
    void handleReceiveInterrupt();

    /** Everything that follows the last frame of a transmission: release it, listen, schedule the next one */
    void onTransmitDone();

    static void timerCallback(void *p1, uint32_t p2);

    virtual void onNotify(uint32_t notification) override;

  public:
    /**
     * Provide a raw LoRa frame for transmission without the Meshtastic PacketHeader, for the
     * experimental JenCore relay. The frame is copied; TX is tracked by rawSending and
     * completed in handleTransmitInterrupt() without touching sendingPacket/radioBuffer.
     */
    bool sendRawFrame(const uint8_t *bytes, size_t len) override;

    /** Queue a raw JenCore frame for the next idle radio slot (see RadioInterface::queueJenCoreFrame). */
    bool queueJenCoreFrame(const uint8_t *bytes, size_t len) override;

#if MESHTASTIC_ENABLE_RNODE
    /** Queue a raw Reticulum RNode frame for transmission on the Reticulum tier. */
    bool queueRNodeFrame(const uint8_t *bytes, size_t len) override;
    void maybeSendRNodeFrame();
#endif

    size_t formatContentionStats(char *buf, size_t len) override;

    /** True if a JenCore frame may go on air now: radio enabled, tx_enabled and within the duty cycle. */
    bool jencoreTxAllowed();

#if MESHTASTIC_ENABLE_JENCORE_SNIFFER
    /** Dedup + optional relay for JenCore frames, fed from handleReceiveInterrupt. */
    JenCoreRepeater jencoreRepeater;

    /** True if the frame in radioBuffer decrypts as a Meshtastic packet on one of our channels. */
    bool isMeshtasticOnOurChannels(size_t length);
#endif

    /**
     * If a JenCore frame is due and the radio and channel are free, transmit it raw. Otherwise it stays queued for the
     * next jencoreRelayTick(). No-op when the sniffer is disabled.
     */
    void maybeSendJenCoreFrame();

#if MESHTASTIC_ENABLE_JENCORE_SNIFFER
    /** Owns the JenCore relay's wake-ups, apart from the radio thread's single notification slot. */
    JenCoreRelayThread *jencoreRelayThread = nullptr;

    /** Schedule the relay thread for when the next queued JenCore frame can go out. */
    void wakeJenCoreRelay();

    /** Owns the ADVERT schedule; the radio thread's single notification slot cannot hold a long timer. */
    concurrency::OSThread *jencoreAdvertThread = nullptr;
    // Next flood / zero-hop ADVERT. The zero-hop one is due at the first tick (JENCORE_ADVERT_FIRST_MS after boot).
    uint32_t nextJenCoreFloodAdvertMs = 0;
    uint32_t nextJenCoreLocalAdvertMs = 0;

  public:
    /**
     * One step of the JenCore relay, run from its own thread: sends what is due and returns ms until the next step,
     * or disables the thread when nothing is queued.
     */
    uint32_t jencoreRelayTick();

    /**
     * One step of the JenCore ADVERT schedule, run from its own thread: loads the identity, queues a signed
     * ADVERT when due, and nudges the radio thread until it is on air. Returns ms until the next step.
     */
    uint32_t jencoreAdvertTick();

    /**
     * Housekeeping around the ADVERT schedule, run from the same thread: hands the identity to the companion once it
     * exists, restores the saved contacts, saves them again when they changed, and times out unanswered direct
     * messages. Returns ms until the next step.
     */
    uint32_t jencoreServiceTick();

  private:
    uint32_t jencoreContactsFingerprint = 0; // of what is on flash, so a save happens only on a change
    uint32_t lastJenCoreContactsSaveMs = 0;
    bool jencoreContactsLoaded = false;

  public:
#endif

  protected:
    /** True if a raw (non-MeshPacket) frame is currently being transmitted. */
    bool isRawSending() const { return rawSending; }

    enum RawSendingType : uint8_t {
        RAW_SEND_NONE = 0,
        RAW_SEND_JENCORE = 1,
        RAW_SEND_RNODE = 2
    };
    RawSendingType rawSendingType = RAW_SEND_NONE;
    RawSendingType getRawSendingType() const { return rawSendingType; }

    uint8_t rawTxBuf[MAX_LORA_PAYLOAD_LEN];
    size_t rawTxLen = 0;
    bool rawSending = false;

#if MESHTASTIC_ENABLE_RNODE
    static constexpr size_t RNODE_TX_QUEUE_SIZE = 4;
    struct RNodeTxEntry {
        uint8_t buf[MAX_LORA_PAYLOAD_LEN];
        size_t len = 0;
    };
    RNodeTxEntry rnodeTxQueue[RNODE_TX_QUEUE_SIZE];
    uint8_t rnodeTxHead = 0;
    uint8_t rnodeTxTail = 0;
    uint8_t rnodeTxCount = 0;
#endif

    /**
     * start an immediate transmit
     *  This method is virtual so subclasses can hook as needed, subclasses should not call directly
     *  @return true if packet was sent
     */
    virtual bool startSend(meshtastic_MeshPacket *txp);

    meshtastic_QueueStatus getQueueStatus();

  protected:
    uint32_t activeReceiveStart = 0;

    bool receiveDetected(uint16_t irq, unsigned long syncWordHeaderValidFlag, unsigned long preambleDetectedFlag);

    /** Do any hardware setup needed on entry into send configuration for the radio.
     * Subclasses can customize, but must also call this base method */
    virtual void configHardwareForSend();

    /** Could we send right now (i.e. either not actively receiving or transmitting)? */
    virtual bool canSendImmediately();

    /**
     * Raw ISR handler that just calls our polymorphic method
     */
    static void isrRxLevel0();

    /**
     * If a send was in progress finish it and return the buffer to the pool */
    void completeSending();

    /** If a raw (non-MeshPacket) frame was being transmitted, mark it finished and log its airtime. */
    void completeRawSend();

    // Frame hooks for modems whose frames are shorter than a Meshtastic packet (SX128x FLRC: 127 bytes).
    // The default sends and reads the whole radioBuffer as one frame.
    static const int RX_FRAME_PARTIAL = 1;
    virtual int16_t startTransmitFrame(size_t numbytes) { return iface->startTransmit((uint8_t *)&radioBuffer, numbytes); }
    /** >0 while more frames of the current packet remain: wait this long, then sendNextFrame() */
    virtual uint32_t nextFrameDelayMsec() { return 0; }
    virtual bool sendNextFrame() { return false; }
    /** Read one received frame; RX_FRAME_PARTIAL means a fragment was stored and no packet is complete yet */
    virtual int readFrame(size_t &length) { return iface->readData((uint8_t *)&radioBuffer, length); }

    /**
     * Add SNR data to received messages
     */
    virtual void addReceiveMetadata(meshtastic_MeshPacket *mp) = 0;

    /** Chip specific arm/disarm of the radio IRQ; call enableInterrupt()/disableInterrupt() instead */
    virtual void setRadioIsr(void (*callback)()) = 0;
    virtual void clearRadioIsr() = 0;

    /**
     * Subclasses must override, implement and then call into this base class implementation
     */
    virtual void setStandby();

    /// RadioLib returns its negative RADIOLIB_ERR_* codes through the same unsigned microsecond count it
    /// returns durations in, so an error reads as 4294967ms of airtime for one packet and takes the node
    /// off the air until it reboots (#11935). The codes are int16_t, so they wrap to the top of the
    /// range; the slowest packet we can configure is ~229s, well clear of it.
    static bool isRadioLibTimeError(RadioLibTime_t usec) { return usec == 0 || usec >= (RadioLibTime_t)0 - 32768; }

    // Nexus port (Nexus-28001, 2026-08-16): override the virtual applyNexusSyncwordOverride() to
    // apply NexusModule::syncword_override (vault v40) to the subclass's syncWord member.
    // Called from RadioInterface::applyModemConfig() before any driver reads syncWord.
    // syncword_override == 0 means "use Meshtastic default 0x2b"; no write needed in that
    // case (the class member is already initialized to 0x2b).
    void applyNexusSyncwordOverride() override;

    /**
     * Derive packet time either for a received (using header info) or a transmitted packet
     */
    template <typename T> uint32_t computePacketTime(T &lora, uint32_t pl, bool received)
    {
        // Nexus SETFSK/SETFLRC: When alternate modem is active, calculate airtime directly
        // from the active modem driver parameters rather than applying LoRa DR math.
        if (meshtastic::nexusModule && meshtastic::nexusModule->altModem.mode != 0) {
            RadioLibTime_t reported = lora.getTimeOnAir(pl);
            if (!isRadioLibTimeError(reported)) {
                uint32_t ms = (reported + 999) / 1000;
                return ms > 0 ? ms : 1;
            }
            // Fallback when RadioLib's getTimeOnAir omits the active modem branch (e.g. SX128x::getTimeOnAir
            // in RadioLib 7.6.0 returns RADIOLIB_ERR_WRONG_MODEM for RADIOLIB_SX128X_PACKET_TYPE_FLRC):
            const auto &m = meshtastic::nexusModule->altModem;
            if (m.brKbps > 0.0f) {
                uint32_t totalBits = (uint32_t)m.preambleBits + 32 + ((pl + 3) * 8);
                if (m.mode == 2) {
                    uint32_t codedBits = ((pl + 3) * 8) + 6;
                    if (m.cr == 2)
                        codedBits *= 2;
                    else if (m.cr == 3)
                        codedBits = (codedBits * 4) / 3;
                    totalBits = (uint32_t)m.preambleBits + 21 + 32 + codedBits;
                }
                uint32_t ms = (uint32_t)((totalBits / m.brKbps) + 0.999f);
                return ms > 0 ? ms : 1;
            }
        }

        DataRate_t dr = getDataRate();
        PacketConfig_t pc = getPacketConfig();

        if (received) {
            // Received packet configuration must be the same as configured, except for coding rate and CRC
            uint8_t rxCR = 0;
            bool hasCRC = true;
            if (lora.getLoRaRxHeaderInfo(&rxCR, &hasCRC) == RADIOLIB_ERR_NONE) {
                // Raw 0 is reserved and >7 is either undefined or an LR2021-only convolutional rate no
                // Meshtastic peer can send. calculateTimeOnAir() would multiply by it unchecked.
                if (rxCR < 1 || rxCR > 7) {
                    LOG_WARN("Bogus RX coding rate %d from radio, use configured %d", rxCR, dr.lora.codingRate);
                } else {
                    // Go from raw header value to denominator
                    if (rxCR < 5) {
                        rxCR += 4;
                    } else if (rxCR == 7) {
                        rxCR = 8;
                    }

                    dr.lora.codingRate = rxCR;
                    pc.lora.crcEnabled = hasCRC;
                }
            }
        } else {
            // Reads the packet type back over SPI, so a chip that lost its config answers WRONG_MODEM.
            RadioLibTime_t reported = lora.getTimeOnAir(pl);
            if (!isRadioLibTimeError(reported))
                return reported / 1000;
            LOG_WARN("%s%d from getTimeOnAir, use configured modem", radioLibErr, (int)(int16_t)reported);
        }

        // Arithmetic on the config we asked for, with no readback to fail. Guarded too: once a code is
        // in milliseconds nothing downstream can tell it from a duration.
        RadioLibTime_t computed = lora.calculateTimeOnAir(modemType, dr, pc, pl);
        if (isRadioLibTimeError(computed)) {
            LOG_ERROR("%s%d from calculateTimeOnAir", radioLibErr, (int)(int16_t)computed);
            return 0;
        }

        return computed / 1000;
    }

    const char *radioLibErr = "RadioLib err=";

    /**
     * If the packet is not already in the late rebroadcast window, move it there
     */
    void clampToLateRebroadcastWindow(NodeNum from, PacketId id);

    /**
     * If there is a packet pending TX in the queue with a worse hop limit, remove it pending replacement with a better version
     * @return Whether a pending packet was removed
     */

    bool removePendingTXPacket(NodeNum from, PacketId id, uint32_t hop_limit_lt) override;

    void checkRxDoneIrqFlag();
    void checkTxDoneIrqFlag();

    /** Software-poll substitute for a hardware DIO interrupt, for radios whose IRQ line sits behind
     * an I2C IO expander with no INT routed to the MCU (e.g. Meshnology W10, LORA_DIO1_SOFTWARE_POLL).
     * The chip-specific subclass polls the radio's IRQ status register from the radio thread and
     * synthesizes ISR_TX/ISR_RX events equivalent to the hardware DIO1 interrupt. */
    void deliverPendingIrqFromPoll(PendingISR cause);
    void scheduleIrqPollTick();
    static bool isIsrTxCallback(void (*callback)());
    virtual void handleSoftwareLoraIrqPoll() {}
};
