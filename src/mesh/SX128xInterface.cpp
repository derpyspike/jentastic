#if RADIOLIB_EXCLUDE_SX128X != 1
#include "SX128xInterface.h"
#include "Throttle.h"
#include "configuration.h"
#include "error.h"
#include "main.h"
#include "mesh/NodeDB.h"
#include "modules/NexusModule.h" // Nexus SETFSK/SETFLRC: altModem state (Phase 2: SX128x)

#if ARCH_PORTDUINO
#include "PortduinoGlue.h"
#endif

// Particular boards might define a different max power based on what their hardware can do
#if ARCH_PORTDUINO
#define SX128X_MAX_POWER portduino_config.sx128x_max_power
#endif
#ifndef SX128X_MAX_POWER
#define SX128X_MAX_POWER 13
#endif

template <typename T>
SX128xInterface<T>::SX128xInterface(LockingArduinoHal *hal, RADIOLIB_PIN_TYPE cs, RADIOLIB_PIN_TYPE irq, RADIOLIB_PIN_TYPE rst,
                                    RADIOLIB_PIN_TYPE busy)
    : RadioLibInterface(hal, cs, irq, rst, busy, &lora), lora(&module)
{
    LOG_DEBUG("SX128xInterface(cs=%d, irq=%d, rst=%d, busy=%d)", cs, irq, rst, busy);
}

/// Initialise the Driver transport hardware and software.
/// Make sure the Driver is properly configured before calling init().
/// \return true if initialisation succeeded.
template <typename T> bool SX128xInterface<T>::init()
{
#ifdef SX128X_POWER_EN
    pinMode(SX128X_POWER_EN, OUTPUT);
    digitalWrite(SX128X_POWER_EN, HIGH);
#endif

#ifdef RF95_FAN_EN
    pinMode(RF95_FAN_EN, OUTPUT);
    digitalWrite(RF95_FAN_EN, 1);
#endif

#if ARCH_PORTDUINO
    if (portduino_config.lora_rxen_pin.pin != RADIOLIB_NC) {
        pinMode(portduino_config.lora_rxen_pin.pin, OUTPUT);
        digitalWrite(portduino_config.lora_rxen_pin.pin, LOW); // Set low before becoming an output
    }
    if (portduino_config.lora_txen_pin.pin != RADIOLIB_NC) {
        pinMode(portduino_config.lora_txen_pin.pin, OUTPUT);
        digitalWrite(portduino_config.lora_txen_pin.pin, LOW); // Set low before becoming an output
    }
#else
#if defined(SX128X_RXEN) && (SX128X_RXEN != RADIOLIB_NC) // set not rx or tx mode
    pinMode(SX128X_RXEN, OUTPUT);
    digitalWrite(SX128X_RXEN, LOW); // Set low before becoming an output
#endif
#if defined(SX128X_TXEN) && (SX128X_TXEN != RADIOLIB_NC)
    pinMode(SX128X_TXEN, OUTPUT);
    digitalWrite(SX128X_TXEN, LOW);
#endif
#endif

    RadioLibInterface::init();

    if (!reinitChip(/*fromInit=*/true))
        return false;

    startReceive(); // start receiving

    return true;
}

// begin() and the chip-side setup that a reset chip loses. Shared by init() and by reconfigure()'s
// recovery of a chip that lost its state.
template <typename T> bool SX128xInterface<T>::reinitChip(bool fromInit)
{
    // Clamp here, not just in programModemParams(): applyModemConfig() resets `power` to the raw
    // config value, and the recovery path reaches begin() without passing through the params clamp
    limitPower(SX128X_MAX_POWER);

    preambleLength = 12; // 12 is the default for this chip, 32 does not RX at all

    int res = lora.begin(getFreq(), bw, sf, cr, syncWord, power, preambleLength);
    // \todo Display actual typename of the adapter, not just `SX128x`
    LOG_INFO("SX128x init result %d", res);
    if (res == RADIOLIB_ERR_CHIP_NOT_FOUND || res == RADIOLIB_ERR_SPI_CMD_FAILED)
        return false;

    if ((config.lora.region != meshtastic_Config_LoRaConfig_RegionCode_LORA_24) && (res == RADIOLIB_ERR_INVALID_FREQUENCY)) {
        // Boot-time only: rebooting out of a runtime recovery would reintroduce exactly the crash this
        // recovery path exists to avoid, and would do it while a config save is still pending.
        if (!fromInit) {
            LOG_ERROR("SX128x rejected the frequency during recovery; leaving region alone");
            return false;
        }
        LOG_WARN("Radio only supports 2.4GHz LoRa. Adjusting Region and rebooting");
        config.lora.region = meshtastic_Config_LoRaConfig_RegionCode_LORA_24;
        nodeDB->saveToDisk(SEGMENT_CONFIG);
        delay(2000);
#if defined(ARCH_ESP32)
        ESP.restart();
#elif defined(ARCH_NRF52)
        nrf52FlashQuiesce(); // reset with the flash layer quiesced, like every other nRF52 reset path
        NVIC_SystemReset();
#else
        LOG_ERROR("FIXME implement reboot for this platform. Skip for now");
#endif
    }

    LOG_INFO("Frequency set to %f", getFreq());
    LOG_INFO("Bandwidth set to %f", bw);
    LOG_INFO("Power output set to %d", power);

#if defined(SX128X_TXEN) && (SX128X_TXEN != RADIOLIB_NC) && defined(SX128X_RXEN) && (SX128X_RXEN != RADIOLIB_NC)
    if (res == RADIOLIB_ERR_NONE) {
        lora.setRfSwitchPins(SX128X_RXEN, SX128X_TXEN);
    }
#elif ARCH_PORTDUINO
    if (res == RADIOLIB_ERR_NONE && portduino_config.lora_rxen_pin.pin != RADIOLIB_NC &&
        portduino_config.lora_txen_pin.pin != RADIOLIB_NC) {
        lora.setRfSwitchPins(portduino_config.lora_rxen_pin.pin, portduino_config.lora_txen_pin.pin);
    }
#endif

    if (res == RADIOLIB_ERR_NONE)
        res = lora.setCRC(2);

    if (res != RADIOLIB_ERR_NONE)
        LOG_ERROR("SX128x re-init failed %s%d", radioLibErr, res);
    return res == RADIOLIB_ERR_NONE;
}

template <typename T> int16_t SX128xInterface<T>::programAltModemFSK()
{
    const meshtastic::NexusModule::NexusAltModem &m = meshtastic::nexusModule->altModem;
    limitPower(SX128X_MAX_POWER);
    // beginGFSK(freq MHz, br kbps, freqDev kHz, pwr dBm, preamble bits) — freq+pwr baked in.
    int16_t err = lora.beginGFSK(getFreq(), (uint16_t)m.brKbps, m.freqDevKHz, power, m.preambleBits);
    if (err != RADIOLIB_ERR_NONE) {
        LOG_ERROR("SX128X beginGFSK(%.3fM/%.1fk/%.1fk/%d/%u) %s%d", getFreq(), m.brKbps,
                  m.freqDevKHz, power, m.preambleBits, radioLibErr, err);
        return err;
    }
    // Uniform Nexus wire profile (Jenny ruling 2026-10-14): pin sync/whitening/NRZ/ CRC —
    // note §7: SX128x GFSK whitening defaults ON (silent killer), so OFF must be explicit.
    uint8_t wireSync[2] = {0x12, 0xAD};
    err = lora.setSyncWord(wireSync, sizeof(wireSync));
    if (err != RADIOLIB_ERR_NONE) {
        LOG_ERROR("SX128X GFSK setSyncWord %s%d", radioLibErr, err);
        return err;
    }
    err = lora.setCRC(2, 0x1D0F, 0x1021); // no inverted variant on SX128x
    if (err != RADIOLIB_ERR_NONE) {
        LOG_ERROR("SX128X GFSK setCRC %s%d", radioLibErr, err);
        return err;
    }
    err = lora.setWhitening(false);
    if (err != RADIOLIB_ERR_NONE) {
        LOG_ERROR("SX128X GFSK setWhitening %s%d", radioLibErr, err);
        return err;
    }
    err = lora.setDataShaping(RADIOLIB_SHAPING_NONE);
    if (err != RADIOLIB_ERR_NONE) {
        LOG_ERROR("SX128X GFSK setDataShaping %s%d", radioLibErr, err);
        return err;
    }
    LOG_INFO("SX128X GFSK armed: BR=%.1fk dev=%.1fk pre=%u freq=%.3f", m.brKbps, m.freqDevKHz,
             m.preambleBits, getFreq());
    return RADIOLIB_ERR_NONE;
}

template <typename T> int16_t SX128xInterface<T>::programAltModemFLRC()
{
    const meshtastic::NexusModule::NexusAltModem &m = meshtastic::nexusModule->altModem;
    limitPower(SX128X_MAX_POWER);
    uint8_t cr = (m.cr == 2) ? RADIOLIB_SX128X_FLRC_CR_1_2
               : (m.cr == 3) ? RADIOLIB_SX128X_FLRC_CR_3_4
                             : RADIOLIB_SX128X_FLRC_CR_1_0;
    uint8_t shaping = (m.bt == 0) ? RADIOLIB_SHAPING_NONE
                    : (m.bt == 1) ? RADIOLIB_SHAPING_0_3
                                  : RADIOLIB_SHAPING_0_5;
    // beginFLRC(freq MHz, br kbps, cr selector, pwr dBm, preamble bits, shaping) —
    // explicit params so both sides of an SX128x pair always match (Phase 2 ruling).
    int16_t err = lora.beginFLRC(getFreq(), (uint16_t)m.brKbps, cr, power, m.preambleBits, shaping);
    if (err != RADIOLIB_ERR_NONE) {
        LOG_ERROR("SX128X beginFLRC(%.3fM/%.1fk/cr=%u/%d/%u) %s%d", getFreq(), m.brKbps, cr,
                  power, m.preambleBits, radioLibErr, err);
        return err;
    }
    LOG_INFO("SX128X FLRC armed: BR=%.1fk cr=%u bt=%u pre=%u freq=%.3f", m.brKbps, cr, shaping,
             m.preambleBits, getFreq());
    return RADIOLIB_ERR_NONE;
}

template <typename T> int16_t SX128xInterface<T>::programModemParams()
{
    // Nexus SETFSK/SETFLRC (Phase 2): alt modem armed -> program GFSK/FLRC instead of
    // LoRa; re-read on every reconfigure (persist-until-disabled ruling, 2026-10-14).
    if (meshtastic::nexusModule && meshtastic::nexusModule->altModem.mode == 1) {
        return programAltModemFSK();
    }
    if (meshtastic::nexusModule && meshtastic::nexusModule->altModem.mode == 2) {
        return programAltModemFLRC();
    }

    // configure publicly accessible settings
    int16_t err = lora.setSpreadingFactor(sf);
    if (err != RADIOLIB_ERR_NONE) {
        LOG_ERROR("SX128X setSpreadingFactor(%u) %s%d", sf, radioLibErr, err);
        return err;
    }

    err = lora.setBandwidth(bw);
    if (err != RADIOLIB_ERR_NONE) {
        LOG_ERROR("SX128X setBandwidth(%.1f) %s%d", bw, radioLibErr, err);
        return err;
    }

    err = lora.setCodingRate(cr, cr != 7); // use long interleaving except if CR is 4/7 which doesn't support it
    if (err != RADIOLIB_ERR_NONE) {
        LOG_ERROR("SX128X setCodingRate(%u) %s%d", cr, radioLibErr, err);
        return err;
    }

    err = lora.setSyncWord(syncWord);
    if (err != RADIOLIB_ERR_NONE) {
        LOG_ERROR("SX128X setSyncWord %s%d", radioLibErr, err);
        return err;
    }

    err = lora.setPreambleLength(preambleLength);
    if (err != RADIOLIB_ERR_NONE) {
        LOG_ERROR("SX128X setPreambleLength(%u) %s%d", preambleLength, radioLibErr, err);
        return err;
    }

    err = lora.setFrequency(getFreq());
    if (err != RADIOLIB_ERR_NONE) {
        LOG_ERROR("SX128X setFrequency(%.3f) %s%d", getFreq(), radioLibErr, err);
        return err;
    }

    limitPower(SX128X_MAX_POWER);

    err = lora.setOutputPower(power);
    if (err != RADIOLIB_ERR_NONE) {
        LOG_ERROR("SX128X setOutputPower(%d) %s%d", power, radioLibErr, err);
        return err;
    }

    return RADIOLIB_ERR_NONE;
}

template <typename T> bool SX128xInterface<T>::reconfigure()
{
    RadioLibInterface::reconfigure();

    // set mode to standby - a chip that lost its state to a reset/brownout can time out here,
    // so don't let setStandby()'s assert fire before the recovery below gets a chance
    int16_t err = trySetStandby();
    if (err == RADIOLIB_ERR_NONE)
        err = programModemParams();

    if (err != RADIOLIB_ERR_NONE) {
        // A chip that fails standby or rejects parameter programming (typically WRONG_MODEM, -20) has
        // lost its runtime configuration - packet type included - to a chip-internal reset or brownout.
        // Recover in place: begin() hardware-resets the chip and restores the LoRa packet type. Crashing
        // here instead would reboot before MeshService persists the config change that triggered us.
        RECORD_CRITICALERROR(meshtastic_CriticalErrorCode_INVALID_RADIO_SETTING);
        LOG_ERROR("SX128x rejected modem params, chip state lost? Full re-init");
        if (!reinitChip() || (err = programModemParams()) != RADIOLIB_ERR_NONE) {
            LOG_ERROR("SX128x unrecoverable %s%d, radio down until reboot", radioLibErr, err);
            return false;
        }
        LOG_INFO("SX128x recovered after re-init");
    }

    startReceive(); // restart receiving

    return true;
}

template <typename T> void SX128xInterface<T>::clearRadioIsr()
{
    lora.clearDio1Action();
}

template <typename T> bool SX128xInterface<T>::wideLora()
{
    return true;
}

template <typename T> int16_t SX128xInterface<T>::trySetStandby()
{
    checkNotification(); // handle any pending interrupts before we force standby

    int16_t err = lora.standby();

    if (err != RADIOLIB_ERR_NONE)
        LOG_ERROR("SX128x standby %s%d", radioLibErr, err);
#if ARCH_PORTDUINO
    if (portduino_config.lora_rxen_pin.pin != RADIOLIB_NC) {
        digitalWrite(portduino_config.lora_rxen_pin.pin, LOW);
    }
    if (portduino_config.lora_txen_pin.pin != RADIOLIB_NC) {
        digitalWrite(portduino_config.lora_txen_pin.pin, LOW);
    }
#else
#if defined(SX128X_RXEN) && (SX128X_RXEN != RADIOLIB_NC) // we have RXEN/TXEN control - turn off RX and TX power
    digitalWrite(SX128X_RXEN, LOW);
#endif
#if defined(SX128X_TXEN) && (SX128X_TXEN != RADIOLIB_NC)
    digitalWrite(SX128X_TXEN, LOW);
#endif
#endif
    isReceiving = false; // If we were receiving, not any more
    activeReceiveStart = 0;
    disableInterrupt();
    completeSending(); // If we were sending, not anymore
    RadioLibInterface::setStandby();
    return err;
}

template <typename T> void SX128xInterface<T>::setStandby()
{
    int16_t err = trySetStandby();
    assert(err == RADIOLIB_ERR_NONE);
}

/**
 * Add SNR data to received messages
 */
template <typename T> uint8_t SX128xInterface<T>::readChipState()
{
    // Busy pin first: same wedged-chip protection as the SX126x path.
    if (module.getGpio() != RADIOLIB_NC && module.hal->digitalRead(module.getGpio()))
        return CHIP_BUSY;

    // Same vendored-getStatus fix as SX126x (SPIreadStream(...,0) copies nothing),
    // plus the SX1280 quirk from RadioLib master: the status byte is driven during
    // the command byte itself, so the command goes out via the NOP slot and the
    // payload is a single byte. parseStatusCb is parked because the default path
    // would read buffIn[statusPos=1] past the end of this 1-byte frame (OOB).
    const uint8_t cmd = RADIOLIB_SX128X_CMD_GET_STATUS;
    uint8_t data = 0;
    int16_t (*savedParse)(uint8_t) = module.spiConfig.parseStatusCb;
    module.spiConfig.parseStatusCb = nullptr;
    module.spiConfig.widths[RADIOLIB_MODULE_SPI_WIDTH_STATUS] = Module::BITS_0;
    module.spiConfig.cmds[RADIOLIB_MODULE_SPI_COMMAND_NOP] = RADIOLIB_SX128X_CMD_GET_STATUS;
    (void)module.SPItransferStream(NULL, 0, false, &cmd, &data, 1, true);
    module.spiConfig.cmds[RADIOLIB_MODULE_SPI_COMMAND_NOP] = RADIOLIB_SX128X_CMD_NOP;
    module.spiConfig.widths[RADIOLIB_MODULE_SPI_WIDTH_STATUS] = Module::BITS_8;
    module.spiConfig.parseStatusCb = savedParse;
    if (data == RADIOLIB_SX128X_STATUS_SPI_FAILED)
        return CHIP_UNK;
    if (data & RADIOLIB_SX128X_STATUS_BUSY)
        return CHIP_BUSY;

    // Mode lives in bits 7:5 (same raw GET_STATUS call as RadioLib's protected SX128x::getStatus).
    switch (data & 0b11100000) {
    case RADIOLIB_SX128X_STATUS_MODE_STDBY_RC:
    case RADIOLIB_SX128X_STATUS_MODE_STDBY_XOSC:
        return CHIP_STDBY;
    case RADIOLIB_SX128X_STATUS_MODE_FS:
        return CHIP_FS;
    case RADIOLIB_SX128X_STATUS_MODE_RX:
        return CHIP_RX;
    case RADIOLIB_SX128X_STATUS_MODE_TX:
        return CHIP_TX;
    default:
        return CHIP_UNK;
    }
}

template <typename T> void SX128xInterface<T>::addReceiveMetadata(meshtastic_MeshPacket *mp)
{
    // LOG_DEBUG("PacketStatus %x", lora.getPacketStatus());
    // Nexus SETFSK/SETFLRC: no chirp SNR in GFSK/FLRC — RSSI-based pseudo-SNR (2026-10-14).
    if (meshtastic::nexusModule && meshtastic::nexusModule->altModem.mode != 0) {
        mp->rx_snr = meshtastic::NexusModule::altPseudoSnr(lora.getRSSI());
    } else {
        mp->rx_snr = lora.getSNR();
    }
    mp->rx_rssi = lround(lora.getRSSI());
    mp->has_rx_rssi = true; // rx_rssi has explicit presence - a genuine reading must be marked present to survive encoding
    // Nexus port (Nexus-28001, 2026-08-16): see SX126xInterface.cpp for rationale.
    mp->rx_frequency_error = lora.getFrequencyError();
    LOG_DEBUG("Corrected frequency offset: %f", mp->rx_frequency_error);

    // Nexus port (Nexus-28001, 2026-08-16): CR recovery via LoRa header bits (RadioLib 7.6.0
    // exposes getLoRaRxHeaderInfo on every chip family). See SX126xInterface.cpp for rationale.
    {
        uint8_t rxCR = 0;
        bool hasCRC = true;
        if (lora.getLoRaRxHeaderInfo(&rxCR, &hasCRC) == RADIOLIB_ERR_NONE) {
            mp->rx_coding_rate = rxCR;
        }
    }
}

/** We override to turn on transmitter power as needed.
 */
template <typename T> void SX128xInterface<T>::configHardwareForSend()
{
#if ARCH_PORTDUINO
    if (portduino_config.lora_txen_pin.pin != RADIOLIB_NC) {
        digitalWrite(portduino_config.lora_txen_pin.pin, HIGH);
    }
    if (portduino_config.lora_rxen_pin.pin != RADIOLIB_NC) {
        digitalWrite(portduino_config.lora_rxen_pin.pin, LOW);
    }

#else
#if defined(SX128X_TXEN) && (SX128X_TXEN != RADIOLIB_NC) // we have RXEN/TXEN control - turn on TX power / off RX power
    digitalWrite(SX128X_TXEN, HIGH);
#endif
#if defined(SX128X_RXEN) && (SX128X_RXEN != RADIOLIB_NC)
    digitalWrite(SX128X_RXEN, LOW);
#endif
#endif

    RadioLibInterface::configHardwareForSend();
}

// For power draw measurements, helpful to force radio to stay sleeping
// #define SLEEP_ONLY

template <typename T> void SX128xInterface<T>::startReceive()
{
#ifdef SLEEP_ONLY
    sleep();
#else

#if ARCH_PORTDUINO
    if (portduino_config.lora_rxen_pin.pin != RADIOLIB_NC) {
        digitalWrite(portduino_config.lora_rxen_pin.pin, HIGH);
    }
    if (portduino_config.lora_txen_pin.pin != RADIOLIB_NC) {
        digitalWrite(portduino_config.lora_txen_pin.pin, LOW);
    }

#else
#if defined(SX128X_RXEN) && (SX128X_RXEN != RADIOLIB_NC) // we have RXEN/TXEN control - turn on RX power / off TX power
    digitalWrite(SX128X_RXEN, HIGH);
#endif
#if defined(SX128X_TXEN) && (SX128X_TXEN != RADIOLIB_NC)
    digitalWrite(SX128X_TXEN, LOW);
#endif
#endif

    int16_t err = trySetStandby();
    if (err == RADIOLIB_ERR_NONE)
        err = lora.startReceive(RADIOLIB_SX128X_RX_TIMEOUT_INF, MESHTASTIC_RADIOLIB_IRQ_RX_FLAGS);

    if (err != RADIOLIB_ERR_NONE) {
        LOG_ERROR("SX128X startReceive %s%d", radioLibErr, err);
        if (maybeRecoverChipStateLoss())
            err = lora.startReceive(RADIOLIB_SX128X_RX_TIMEOUT_INF, MESHTASTIC_RADIOLIB_IRQ_RX_FLAGS);
    }

    if (err != RADIOLIB_ERR_NONE) {
        // No assert: leave RX off rather than reboot; periodicRadioMaintenance() re-arms it, throttled
        LOG_ERROR("SX128X RX offline %s%d", radioLibErr, err);
        rxOffline = true;
        return;
    }

    RadioLibInterface::startReceive();

    // Must be done AFTER, starting transmit, because startTransmit clears (possibly stale) interrupt pending register bits
    enableInterrupt(isrRxLevel0);
    checkRxDoneIrqFlag();
#endif
}

/** Is the channel currently active? */
template <typename T> bool SX128xInterface<T>::isChannelActive()
{
    // check if we can detect a LoRa preamble on the current channel
    ChannelScanConfig_t cfg = {.cad = {.symNum = NUM_SYM_CAD_24GHZ,
                                       .detPeak = 0,
                                       .detMin = 0,
                                       .exitMode = 0,
                                       .timeout = 0,
                                       .irqFlags = RADIOLIB_IRQ_CAD_DEFAULT_FLAGS,
                                       .irqMask = RADIOLIB_IRQ_CAD_DEFAULT_MASK}};
    int16_t result = trySetStandby();
    if (result == RADIOLIB_ERR_NONE) {
        // Nexus SETFSK/SETFLRC: no LoRa CAD in GFSK/FLRC — RSSI energy sniff (2026-10-14).
        if (meshtastic::nexusModule && meshtastic::nexusModule->altModem.mode != 0) {
            float rssi = lora.getRSSI(false); // instantaneous, not last-packet status
            LOG_TRACE("SX128X alt-modem channel sniff RSSI=%.1f dBm", rssi);
            return rssi > -100.0f;
        }
        result = lora.scanChannel(cfg);
        if (result == RADIOLIB_LORA_DETECTED)
            return true;
        if (result != RADIOLIB_CHANNEL_FREE)
            LOG_ERROR("SX128X scanChannel %s%d", radioLibErr, result);
        if (result != RADIOLIB_ERR_WRONG_MODEM)
            return false;
    }

    // standby failed or the LoRa modem type is gone - the chip lost its runtime state
    maybeRecoverChipStateLoss();
    return false; // report the channel free: a recovered chip can TX, a dead one fails startSend safely
}

/** Could we send right now (i.e. either not actively receiving or transmitting)? */
template <typename T> bool SX128xInterface<T>::isActivelyReceiving()
{
    // Nexus SETFSK/SETFLRC: LoRa detects on HEADER_VALID; GFSK/FLRC detect on
    // SYNC_WORD_VALID (the GFSK packet-start event).
    if (meshtastic::nexusModule && meshtastic::nexusModule->altModem.mode != 0) {
        const uint16_t irq = lora.getIrqStatus();
        const uint16_t latch = RADIOLIB_SX128X_IRQ_SYNC_WORD_VALID | RADIOLIB_SX128X_IRQ_PREAMBLE_DETECTED;
        const bool detected = receiveDetected(irq, RADIOLIB_SX128X_IRQ_SYNC_WORD_VALID,
                                              RADIOLIB_SX128X_IRQ_PREAMBLE_DETECTED);
        // Stale-latch hazard (bench 2026-10-17, same as SX126x FSK): false noise
        // detections latch non-terminal IRQ bits that the interrupt path never clears
        // -> busyRx latches forever -> TX starved. Clear once receiveDetected() rules it stale.
        if (!detected && (irq & latch)) {
            lora.clearIrqFlags(latch);
            LOG_TRACE("SX128X alt-modem stale RX latch cleared (irq=0x%04X)", (unsigned)irq);
        }
        return detected;
    }
    return receiveDetected(lora.getIrqStatus(), RADIOLIB_SX128X_IRQ_HEADER_VALID, RADIOLIB_SX128X_IRQ_PREAMBLE_DETECTED);
}

template <typename T> bool SX128xInterface<T>::sleep()
{
    // Not keeping config is busted - next time nrf52 board boots lora sending fails  tcxo related? - see datasheet
    // \todo Display actual typename of the adapter, not just `SX128x`
    LOG_DEBUG("SX128x entering sleep mode"); // (FIXME, don't keep config)
    (void)trySetStandby(); // Stop any pending operations - the chip is being put to sleep, a failure must not crash

    // turn off TCXO if it was powered
    // FIXME - this isn't correct
    // lora.setTCXO(0);

    // put chipset into sleep mode (we've already disabled interrupts by now)
    bool keepConfig = true;
    lora.sleep(keepConfig); // Note: we do not keep the config, full reinit will be needed

#ifdef SX128X_POWER_EN
    digitalWrite(SX128X_POWER_EN, LOW);
#endif

    return true;
}

template <typename T> int16_t SX128xInterface<T>::getCurrentRSSI()
{
    float rssi = lora.getRSSI(false);
    return (int16_t)round(rssi);
}
#endif