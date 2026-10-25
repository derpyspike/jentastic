#pragma once

#include "MemoryPool.h"
#include "MeshTypes.h"
#include "Observer.h"
#include "PointerQueue.h"
#include "airtime.h"
#include "error.h"
#include <memory>

#if HAS_LORA_FEM
#include "LoRaFEMInterface.h"
#endif

// Forward decl to avoid a direct include of generated config headers / full LoRaConfig definition in this widely-included file.
typedef struct _meshtastic_Config_LoRaConfig meshtastic_Config_LoRaConfig;

#define MAX_TX_QUEUE 16 // max number of packets which can be waiting for transmission

#define MAX_LORA_PAYLOAD_LEN 255 // max length of 255 per Semtech's datasheets on SX12xx
#define MESHTASTIC_HEADER_LENGTH 16
#define MESHTASTIC_PKC_OVERHEAD 12
#define MESHTASTIC_AEAD_OVERHEAD 12

#define PACKET_FLAGS_HOP_LIMIT_MASK 0x07
#define PACKET_FLAGS_WANT_ACK_MASK 0x08
#define PACKET_FLAGS_VIA_MQTT_MASK 0x10
#define PACKET_FLAGS_HOP_START_MASK 0xE0
#define PACKET_FLAGS_HOP_START_SHIFT 5

/**
 * This structure has to exactly match the wire layout when sent over the radio link.  Used to keep compatibility
 * with the old radiohead implementation.
 */
typedef struct {
    NodeNum to, from; // can be 1 byte or four bytes

    PacketId id; // can be 1 byte or 4 bytes

    /**
     * Usage of flags:
     *
     * The bottom three bits of flags are use to store hop_limit when sent over the wire.
     **/
    uint8_t flags;

    /** The channel hash - used as a hint for the decoder to limit which channels we consider */
    uint8_t channel;

    // Last byte of the NodeNum of the next-hop for this packet
    uint8_t next_hop;

    // Last byte of the NodeNum of the node that will relay/relayed this packet
    uint8_t relay_node;
} PacketHeader;

/**
 * This structure represent the structured buffer : a PacketHeader then the payload. The whole is
 * MAX_LORA_PAYLOAD_LEN + 1 length
 * It makes the use of its data easier, and avoids manipulating pointers (and potential non aligned accesses)
 */
typedef struct {
    /** The header, as defined just before */
    PacketHeader header;

    /** The payload, of maximum length minus the header, aligned just to be sure */
    uint8_t payload[MAX_LORA_PAYLOAD_LEN + 1 - sizeof(PacketHeader)] __attribute__((__aligned__));

} RadioBuffer;

/// On-air ceiling for MeshPacket.encrypted. RadioBuffer holds one byte more, but the PHY caps a whole
/// frame at MAX_LORA_PAYLOAD_LEN, so the header comes out of the same budget (matches perhapsEncode).
constexpr size_t MAX_RADIO_PAYLOAD_LEN = MAX_LORA_PAYLOAD_LEN - sizeof(PacketHeader);
static_assert(MAX_RADIO_PAYLOAD_LEN < sizeof(RadioBuffer::payload), "payload ceiling must fit the buffer");

/**
 * Basic operations all radio chipsets must implement.
 *
 * This defines the SOLE API for talking to radios (because soon we will have alternate radio implementations)
 */
class RadioInterface
{
    friend class MeshRadio; // for debugging we let that class touch pool

    CallbackObserver<RadioInterface, void *> configChangedObserver =
        CallbackObserver<RadioInterface, void *>(this, &RadioInterface::reloadConfig);

    CallbackObserver<RadioInterface, void *> preflightSleepObserver =
        CallbackObserver<RadioInterface, void *>(this, &RadioInterface::preflightSleepCb);

    CallbackObserver<RadioInterface, void *> notifyDeepSleepObserver =
        CallbackObserver<RadioInterface, void *>(this, &RadioInterface::notifyDeepSleepCb);

  protected:
    bool disabled = false;

    float bw = 125;
    uint8_t sf = 9;
    uint8_t cr = 5;

    static constexpr uint8_t NUM_SYM_CAD =
        2; // Number of symbols used for CAD, 2 is the default since RadioLib 6.3.0 as per AN1200.48
    static constexpr uint8_t NUM_SYM_CAD_24GHZ =
        4; // Number of symbols used for CAD in 2.4 GHz, 4 is recommended in AN1200.22 of SX1280
    uint32_t slotTimeMsec = computeSlotTimeMsec();
    uint16_t preambleLength = 16; // 8 is default, but we use longer to increase the amount of sleep time when receiving
    static constexpr uint16_t preambleLengthDefault =
        16; // 8 is default, but we use longer to increase the amount of sleep time when receiving
    static constexpr uint16_t wideLoraPreambleLengthDefault = 12; // 12 is default for wide Lora
    uint32_t preambleTimeMsec = 165;                              // calculated on startup, this is the default for LongFast
    static constexpr uint32_t PROCESSING_TIME_MSEC =
        4500;                           // time to construct, process and construct a packet again (empirically determined)
    static constexpr uint8_t CWmin = 3; // minimum CWsize
    static constexpr uint8_t CWmax = 8; // maximum CWsize

    // SETFSK/SETFLRC timing: packets take 1-10 ms on air, so the LoRa-sized slots and processing allowance dominate
    static constexpr uint32_t ALT_MODEM_SLOT_TIME_MSEC = 1;
    static constexpr uint32_t ALT_MODEM_PROCESSING_TIME_MSEC = 1000;
    // a receiver must read each frame before the next overwrites the chip's buffer; its main loop can be busy ~10 ms
    static constexpr uint32_t ALT_MODEM_MIN_TX_GAP_MSEC = 10;
    static bool altModemActive();

    // Uniform Nexus high-speed GFSK & FLRC sync word across all transceivers (SX126x, SX128x, LR11x0, LR20x0):
    // 4 bytes: 0x2D, 0x4B, 0xD4, 0x93 (high autocorrelation, balanced 0/1, no bit runs, immune to noise locks)
    static constexpr uint8_t nexusAltSyncWord[4] = {0x2D, 0x4B, 0xD4, 0x93};

    meshtastic_MeshPacket *sendingPacket = NULL; // The packet we are currently sending
    uint32_t lastTxStart = 0L;

    uint32_t computeSlotTimeMsec();

    /**
     * A temporary buffer used for sending/receiving packets, sized to hold the biggest buffer we might need
     * */
    RadioBuffer radioBuffer __attribute__((__aligned__));
    /**
     * Enqueue a received packet for the registered receiver
     */
    void deliverToReceiver(meshtastic_MeshPacket *p);

  public:
    /** pool is the pool we will alloc our rx packets from
     */
    RadioInterface();

    virtual ~RadioInterface() {}

    /// Fires once per valid received LoRa packet (arg = sender NodeNum). Used e.g. to flash LED_LORA.
    static Observable<uint32_t> loraRxPacketObservable;

    /**
     * Coerce LoRa config fields (bandwidth/spread_factor) derived from presets.
     * This is used during early bootstrapping so UIs that display these fields directly remain consistent.
     */
    // static void bootstrapLoRaConfigFromPreset(meshtastic_Config_LoRaConfig &loraConfig); // maybe superseded?

    /**
     * Return true if we think the board can go to sleep (i.e. our tx queue is empty, we are not sending or receiving)
     *
     * This method must be used before putting the CPU into deep or light sleep.
     *
     * @param deepSleep true when the radio itself is about to be powered down (deep sleep or
     * shutdown) - an in-flight transmission then vetoes sleep, since it would be truncated on
     * air. false for a light sleep where the radio stays powered and finishes the TX on its own.
     */
    virtual bool canSleep(bool deepSleep = false) { return true; }

    virtual bool wideLora() { return false; }

    /// Whether the radio can tune sub-GHz bands. False for 2.4 GHz-only chips (SX128x);
    /// multiband chips like the LR1121 keep the default.
    virtual bool supportsSubGhz() { return true; }

    /// Prepare hardware for sleep.  Call this _only_ for deep sleep, not needed for light sleep.
    virtual bool sleep() { return true; }

    /// Disable this interface (while disabled, no packets can be sent or received)
    void disable()
    {
        disabled = true;
        sleep();
    }

    /**
     * Send a packet (possibly by enquing in a private fifo).  This routine will
     * later free() the packet to pool.  This routine is not allowed to stall.
     * If the txmit queue is full it might return an error
     */
    virtual ErrorCode send(meshtastic_MeshPacket *p) = 0;

    /**
     * Transmit a raw LoRa frame without any Meshtastic PacketHeader framing. Used by the
     * experimental JenCore relay to re-broadcast foreign-protocol frames byte-for-byte.
     * Implementations must own the buffer until TX completes (copy it). Returns false if the
     * radio is not idle or the frame could not be queued.
     */
    virtual bool sendRawFrame(const uint8_t *bytes, size_t len) { return false; }

    /**
     * Queue a raw JenCore frame for transmission through the radio's listen-before-talk path.
     * Unlike sendRawFrame(), this may defer until the channel is idle. Used by the JenCore
     * companion for GRP_TXT. The frame is copied; returns false if it could not be queued.
     */
    virtual bool queueJenCoreFrame(const uint8_t *bytes, size_t len)
    {
        (void)bytes;
        (void)len;
        return false;
    }

    /**
     * Queue a raw Reticulum RNode frame for transmission through the radio's listen-before-talk path.
     * The frame is copied; returns false if it could not be queued.
     */
    virtual bool queueRNodeFrame(const uint8_t *bytes, size_t len)
    {
        (void)bytes;
        (void)len;
        return false;
    }

    /** Contention measurements of both protocols as one short text line; returns its length, 0 if this radio keeps none. */
    virtual size_t formatContentionStats(char *buf, size_t len)
    {
        (void)buf;
        (void)len;
        return 0;
    }

    /** Return TX queue status */
    [[nodiscard]] virtual meshtastic_QueueStatus getQueueStatus()
    {
        meshtastic_QueueStatus qs;
        qs.res = qs.mesh_packet_id = qs.free = qs.maxlen = 0;
        return qs;
    }

    /** Attempt to cancel a previously sent packet.  Returns true if a packet was found we could cancel */
    virtual bool cancelSending(NodeNum from, PacketId id) { return false; }

    /** Attempt to find a packet in the TxQueue. Returns true if the packet was found. */
    virtual bool findInTxQueue(NodeNum from, PacketId id) { return false; }

    // methods from radiohead

    /// Initialise the Driver transport hardware and software.
    /// Make sure the Driver is properly configured before calling init().
    /// \return true if initialisation succeeded.
    virtual bool init();

    /// Apply any radio provisioning changes
    /// Make sure the Driver is properly configured before calling init().
    /// \return true if initialisation succeeded.
    virtual bool reconfigure();

    /** The delay to use for retransmitting dropped packets */
    [[nodiscard]] uint32_t getRetransmissionMsec(const meshtastic_MeshPacket *p);

    /** The delay to use when we want to send something */
    [[nodiscard]] uint32_t getTxDelayMsec();

    /** The CW to use when calculating SNR_based delays */
    [[nodiscard]] uint8_t getCWsize(float snr);

    /** The worst-case SNR_based packet delay */
    [[nodiscard]] uint32_t getTxDelayMsecWeightedWorst(float snr);

    /** Returns true if we should rebroadcast early like a ROUTER */
    [[nodiscard]] bool shouldRebroadcastEarlyLikeRouter(meshtastic_MeshPacket *p);

    /** The delay to use when we want to flood a message. Use a weighted scale based on SNR */
    [[nodiscard]] uint32_t getTxDelayMsecWeighted(meshtastic_MeshPacket *p);

    /** If the packet is not already in the late rebroadcast window, move it there */
    virtual void clampToLateRebroadcastWindow(NodeNum from, PacketId id) { return; }

    /**
     * If there is a packet pending TX in the queue with a worse hop limit, remove it pending replacement with a better version
     * @return Whether a pending packet was removed
     */
    virtual bool removePendingTXPacket(NodeNum from, PacketId id, uint32_t hop_limit_lt) { return false; }

    /**
     * Calculate airtime per
     * https://www.rs-online.com/designspark/rel-assets/ds-assets/uploads/knowledge-items/application-notes-for-the-internet-of-things/LoRa%20Design%20Guide.pdf
     * section 4
     *
     * @return num msecs for the packet
     */
    [[nodiscard]] uint32_t getPacketTime(const meshtastic_MeshPacket *p, bool received = false);
    [[nodiscard]] virtual uint32_t getPacketTime(uint32_t totalPacketLen, bool received = false) = 0;

    /**
     * Get the channel we saved.
     */
    [[nodiscard]] uint32_t getChannelNum();

    /**
     * Get the frequency we saved.
     */
    [[nodiscard]] virtual float getFreq();

    /**
     * Nexus-28002 (2026-08-21): rest of the Nexus-269 LRS getter set.
     * /nexus LRS uses these to surface the actual chip-side values written by
     * applyModemConfig() — NOT config.lora.* (operator intent). The wrapper's
     * own fields are the truth: configured values, then limitPower() clamps,
     * then passed verbatim to lora.setOutputPower() / lora.begin(). For
     * upstream 2.8 there is no RadioLib getPower()/getTxPower() getter, so
     * the wrapper is the only chip-actual source.
     *
     */
    [[nodiscard]] float getBW() { return bw; }
    [[nodiscard]] uint8_t getSF() { return sf; }
    [[nodiscard]] uint8_t getCR() { return cr; }
    [[nodiscard]] uint16_t getPreambleLength() { return preambleLength; }
    [[nodiscard]] uint32_t getPreambleTimeMsec() { return preambleTimeMsec; }
    [[nodiscard]] uint32_t getSlotTimeMsec() { return slotTimeMsec; }
    [[nodiscard]] int8_t getPower() { return power; }

    /**
     * Nexus (2.8.105): Live per-packet TX power control. Changes output power
     * without touching channel PLL frequency or receiver state.
     * Returns RADIOLIB_ERR_NONE (0) on success.
     */
    virtual int16_t setOutputPowerLive(int8_t powerDb) { return -1; }

    /**
     * Nexus (2.8.105): Fast-path Spreading Factor and TX power switch on the current carrier.
     * Used when presets share the same frequency, BW, and CR (e.g. SFNarrow SF7 <-> NarrowPlus SF6).
     * Returns RADIOLIB_ERR_NONE (0) on success.
     */
    virtual int16_t setSpreadingFactorLive(uint8_t targetSf, int8_t targetPwr = 0x7F) { return -1; }

    /**
     * Nexus (2.8.105): Returns the Spreading Factor of the last received packet.
     * On multi-SF hardware (LR2021), reports the actual detector SF that received the packet.
     */
    [[nodiscard]] virtual uint8_t getLastRxSF() { return sf; }

    /**
     * Nexus-28002 (2026-08-21): restored runtime TCXO/XTAL detection from Nexus-269.
     * Set by RadioInterface::initLoRa() at the per-driver init-success sites
     * (SX1262/68, LLCC68, LR1110/1120/1121). Read by /nexus HW to report the
     * actual clock source the radio is running on, not just the board's compile-time
     * possibility. Default false (XTAL) — overwritten to true on any TCXO path.
     *
     * Nexus-269 original comment:
     *   "Nexus: Detected clock source at runtime."
     */
    bool isTCXO = false;

    /// Some boards (1st gen Pinetab Lora module) have broken IRQ wires, so we need to poll via i2c registers
    virtual bool isIRQPending() { return false; }

    // Whether we use the default frequency slot given our LoRa config (region and modem preset)
    static bool uses_default_frequency_slot;

    // Whether we have a custom channel name
    static bool uses_custom_channel_name;

    static bool checkOrClampConfigLora(meshtastic_Config_LoRaConfig &loraConfig, bool clamp);

    // Check if a candidate region is compatible and valid, with no side effects (safe for
    // speculative UI checks). prospectiveLicensedOwner is for a UI flow that requires
    // confirmation before it sets the owner licensed. errBuf, if given, receives the failure reason.
    static bool checkConfigRegion(const meshtastic_Config_LoRaConfig &loraConfig, char *errBuf = nullptr, size_t errLen = 0,
                                  bool prospectiveLicensedOwner = false);

    // Check if a candidate region is compatible and valid. On failure, logs at ERROR,
    // records a critical error, and sends a client notification.
    static bool validateConfigRegion(const meshtastic_Config_LoRaConfig &loraConfig);

    // Check if a candidate radio configuration is valid.
    static bool validateConfigLora(const meshtastic_Config_LoRaConfig &loraConfig);

    // Make a candidate radio configuration valid, even if it isn't.
    static void clampConfigLora(meshtastic_Config_LoRaConfig &loraConfig);

    // If preset is locked to a sibling of currentRegion among the swappable EU regions
    // (EU_868/EU_866/EU_N_868), return the sibling region owning the preset, else nullptr.
    static const RegionInfo *regionSwapForPreset(meshtastic_Config_LoRaConfig_RegionCode currentRegion,
                                                 meshtastic_Config_LoRaConfig_ModemPreset preset);

  protected:
    int8_t power = 17; // Set by applyModemConfig()

    float savedFreq;
    uint32_t savedChannelNum;

    /***
     * given a packet set sendingPacket and decode the protobufs into radiobuf.  Returns # of bytes to send (including the
     * PacketHeader & payload).
     *
     * Used as the first step of
     */
    [[nodiscard]] size_t beginSending(meshtastic_MeshPacket *p);

    /**
     * Some regulatory regions limit xmit power.
     * This function should be called by subclasses after setting their desired power.  It might lower it
     */
    void limitPower(int8_t MAX_POWER);

    /**
     * Save the frequency we selected for later reuse.
     */
    virtual void saveFreq(float savedFreq);

    /**
     * Save the channel we selected for later reuse.
     */
    virtual void saveChannelNum(uint32_t savedChannelNum);

    /**
     * Get current RSSI reading from the radio.
     * Returns 0 if not available.
     */
    virtual int16_t getCurrentRSSI() { return 0; }

    // Nexus port (Nexus-28001, 2026-08-16): virtual hook so applyModemConfig() (defined on
    // this base class) can ask subclasses to apply NexusModule::syncword_override to their
    // own driver-specific syncWord member. Default is no-op for non-Nexus subclasses or
    // subclasses without a syncWord member. RadioLibInterface overrides to read from
    // NexusModule::syncword_override (vault v40) and write to its member.
    virtual void applyNexusSyncwordOverride() {}

  public:
    // Nexus SETFSK/SETFLRC (Jenny rulings 2026-10-14): capability bits for the
    // alternate modem so the command parser (NexusCommands.cpp) can reject
    // unsupported radios early: bit0 = FSK/GFSK, bit1 = FLRC. Default 0 = none;
    // subclasses override (SX126x returns FSK only — no FLRC hardware there).
    virtual uint8_t altModemCaps() const { return 0; }

  private:
    /**
     * Convert our modemConfig enum into wf, sf, etc...
     *
     * These parameters will be pull from the channelSettings global
     */
    void applyModemConfig();

    /// Return 0 if sleep is okay. A non-NULL argument means the radio is about to be powered
    /// down (deep sleep / shutdown), see doPreflightSleep()
    int preflightSleepCb(void *deepSleep = NULL) { return canSleep(deepSleep != NULL) ? 0 : 1; }

    int notifyDeepSleepCb(void *unused = NULL);

    int reloadConfig(void *unused)
    {
        reconfigure();
        return 0;
    }
};

std::unique_ptr<RadioInterface> initLoRa();

/// Debug printing for packets
void printPacket(const char *prefix, const meshtastic_MeshPacket *p);
