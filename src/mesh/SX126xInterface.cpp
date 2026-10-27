#if RADIOLIB_EXCLUDE_SX126X != 1
#include "SX126xInterface.h"
#include "configuration.h"
#include "error.h"
#include "mesh/NodeDB.h"
#include "modules/NexusModule.h" // Nexus SETFSK: altModem state + pseudo-SNR helper
#if NEXUS_ENABLE_RNODE
#include "mesh/RNodeEngine.h"
#endif
#ifdef ARCH_PORTDUINO
#include "PortduinoGlue.h"
#endif
#if defined(ARCH_ESP32)
#include <driver/rtc_io.h>
#include <esp_sleep.h>
#endif

#include "Throttle.h"
#include "UptimeClock.h"

// Particular boards might define a different max power based on what their hardware can do, default to max power output if not
// specified (may be dangerous if using external PA and SX126x power config forgotten)
#if ARCH_PORTDUINO
#define SX126X_MAX_POWER portduino_config.sx126x_max_power
#endif
#ifndef SX126X_MAX_POWER
#define SX126X_MAX_POWER 22
#endif

template <typename T>
SX126xInterface<T>::SX126xInterface(LockingArduinoHal *hal, RADIOLIB_PIN_TYPE cs, RADIOLIB_PIN_TYPE irq, RADIOLIB_PIN_TYPE rst,
                                    RADIOLIB_PIN_TYPE busy)
    : RadioLibInterface(hal, cs, irq, rst, busy, &lora), lora(&module)
{
    LOG_DEBUG_RADIO("SX126xInterface(cs=%d, irq=%d, rst=%d, busy=%d)", cs, irq, rst, busy);
}

/// Initialise the Driver transport hardware and software.
/// Make sure the Driver is properly configured before calling init().
/// \return true if initialisation succeeded.
template <typename T> bool SX126xInterface<T>::init()
{

// Typically, the RF switch on SX126x boards is controlled by two signals, which are negations of each other (switched RFIO
// paths). The negation is usually performed in hardware, or (suboptimal design) TXEN and RXEN are the two inputs to this style of
// RF switch. On some boards, there is no hardware negation between CTRL and ¬CTRL, but CTRL is internally connected to DIO2, and
// DIO2's switching is done by the SX126X itself, so the MCU can't control ¬CTRL at exactly the same time. One solution would be
// to set ¬CTRL as SX126X_TXEN or SX126X_RXEN, but they may already be used for another purpose, such as controlling another
// PA/LNA. Keeping ¬CTRL high seems to work, as long CTRL=1, ¬CTRL=1 has the opposite and stable RF path effect as CTRL=0 and
// ¬CTRL=1, this depends on the RF switch, but it seems this usually works. Better hardware design, which is done most the time,
// means this workaround is not necessary.
#ifdef SX126X_ANT_SW // Perhaps add RADIOLIB_NC check, and beforehand define as such if it is undefined, but it is not commonly
                     // used and not part of the 'default' set of pin definitions.
    digitalWrite(SX126X_ANT_SW, HIGH);
    pinMode(SX126X_ANT_SW, OUTPUT);
#endif

#ifdef SX126X_POWER_EN // Perhaps add RADIOLIB_NC check, and beforehand define as such if it is undefined, but it is not commonly
                       // used and not part of the 'default' set of pin definitions.
    pinMode(SX126X_POWER_EN, OUTPUT);
    digitalWrite(SX126X_POWER_EN, HIGH);
#endif

#if HAS_LORA_FEM
    loraFEMInterface.init();
    // Apply saved FEM LNA mode from config
    if (loraFEMInterface.isLnaCanControl()) {
        loraFEMInterface.setLNAEnable(config.lora.fem_lna_mode != meshtastic_Config_LoRaConfig_FEM_LNA_Mode_DISABLED);
    }
#endif

#ifdef RF95_FAN_EN
    pinMode(RF95_FAN_EN, OUTPUT);
    digitalWrite(RF95_FAN_EN, HIGH);
#endif

#if ARCH_PORTDUINO
    // An explicit Vref wins; probing with none given tries the radio default first.
    bool tcxoVoltageExplicit = portduino_config.dio3_tcxo_voltage > 0;
    if (tcxoVoltageExplicit)
        tcxoVoltage = (float)portduino_config.dio3_tcxo_voltage / 1000;
    else if (TCXO_OPTIONAL_ENABLED)
        tcxoVoltage = TCXO_OPTIONAL_DEFAULT_VOLTAGE;
    else
        tcxoVoltage = 0;
    if (portduino_config.lora_sx126x_ant_sw_pin.pin != RADIOLIB_NC) {
        digitalWrite(portduino_config.lora_sx126x_ant_sw_pin.pin, HIGH);
        pinMode(portduino_config.lora_sx126x_ant_sw_pin.pin, OUTPUT);
    }
    // The knob here is the YAML key, not the variant define the other branch reports.
    if (tcxoVoltage == 0.0)
        LOG_DEBUG_RADIO("Lora.DIO3_TCXO_VOLTAGE not set, DIO3 not used as TCXO Vref");
    else if (!tcxoVoltageExplicit)
        LOG_DEBUG_RADIO("TCXO_OPTIONAL: no Vref configured, probing default TCXO Vref %f V on DIO3", tcxoVoltage);
    else
        LOG_DEBUG_RADIO("Lora.DIO3_TCXO_VOLTAGE set, DIO3 as TCXO Vref %f V", tcxoVoltage);
#endif
    setTransmitEnable(false);

    RadioLibInterface::init();

    if (!reinitChip())
        return false;
    boundBusyWait();

    startReceive(); // start receiving

    // Init-failure debug aid, disabled: a raw SPI read and parseStatusCb swap racing the RX readout task (needs a RadioSequence).
    // static uint8_t status;
    // uint8_t errors[2] = {0, 0};
    // static Module::SPIparseStatusCb_t parse;
    // parse = module.spiConfig.parseStatusCb;
    // module.spiConfig.parseStatusCb = [](uint8_t in) -> int16_t {
    //     status = in;
    //     return parse ? parse(in) : RADIOLIB_ERR_NONE;
    // };
    // const int16_t readRes = module.SPIreadStream(RADIOLIB_SX126X_CMD_GET_DEVICE_ERRORS, errors, 2);
    // module.spiConfig.parseStatusCb = parse;
    // if (readRes == RADIOLIB_ERR_NONE)
    //     LOG_DEBUG("SX126x status 0x%02x, device errors 0x%02x%02x", status, errors[0], errors[1]);

    return true;
}

// begin() and the chip-side setup that a reset chip loses: begin() hardware-resets the chip, then
// PA ramp, OCP limit, DIO2-as-RF-switch, RF switch pins, RX gain, the 0x8B5 RX patch, and CRC are
// reprogrammed. Shared by init() and by reconfigure()'s recovery path.
template <typename T> bool SX126xInterface<T>::reinitChip()
{
    // Clamp here, not just in programModemParams(): applyModemConfig() resets `power` to the raw
    // config value, and the recovery path reaches begin() without passing through the params clamp
    limitPower(SX126X_MAX_POWER);
    // Make sure we reach the minimum power supported to turn the chip on (-9dBm)
    if (power < -9)
        power = -9;

    // FIXME: May want to set depending on a definition, currently all SX126x variant files use the DC-DC regulator option
    bool useRegulatorLDO = false; // Seems to depend on the connection to pin 9/DCC_SW - if an inductor DCDC?

    int res = lora.begin(getFreq(), bw, sf, cr, syncWord, power, preambleLength, tcxoVoltage, useRegulatorLDO);
    // RadioLib >= 7.5.0 retries on the XTAL inside begin() when the TCXO will not start, and zeroes its own Vref;
    // follow it so logs and later begin() calls reflect the oscillator actually in use.
    if (res == RADIOLIB_ERR_NONE && lora.tcxoVoltage == 0)
        tcxoVoltage = 0;

    // Chip answered but would not start on the TCXO: retry on the XTAL. CHIP_NOT_FOUND is a
    // wiring or SPI fault, where a second attempt only hides it. Portduino only - an embedded
    // board gets this from the second interface instance in initLoRa()'s ladder.
    // TODO: consider deferring to RadioLib, which has autocorrected this itself since 7.5.0:
    // SX126x::modSetup() retries config() on the XTAL when begin() fails with SPI_CMD_FAILED and
    // XOSC_START_ERR, so the ordinary "TCXO configured, XTAL fitted" case never reaches here and
    // what does is mostly invalid settings. Narrowing this to SPI_CMD_TIMEOUT - the oscillator
    // symptom RadioLib's condition misses - would keep the cover and drop the misdiagnosis.
#if ARCH_PORTDUINO
    if (TCXO_OPTIONAL_ENABLED && res != RADIOLIB_ERR_NONE && res != RADIOLIB_ERR_CHIP_NOT_FOUND && tcxoVoltage > 0) {
        LOG_WARN("SX126x init failed with TCXO Vref %f V (err %d), retrying without TCXO", tcxoVoltage, res);
        tcxoVoltage = 0;
        res = lora.begin(getFreq(), bw, sf, cr, syncWord, power, preambleLength, tcxoVoltage, useRegulatorLDO);
        if (res == RADIOLIB_ERR_NONE)
            LOG_INFO("SX126x init success without TCXO (XTAL mode)");
    }
#endif
    if (res == RADIOLIB_ERR_NONE) {
        applyTcxoStartupDelay(lora, tcxoVoltage);
#ifdef SX126X_PA_RAMP_US
        // Set custom PA ramp time for boards requiring longer stabilization (e.g., T-Beam 1W needs >800us)
        lora.setPaRampTime(SX126X_PA_RAMP_US);
#endif
    }
    cadParamsValid = false; // begin() reset the chip
#ifdef ARCH_PORTDUINO
    txStagedByRadioLib = false; // begin() reset the chip, and the register fixes with it
    earlyStagedLen = 0;         // and the buffer with them
#endif
    // \todo Display actual typename of the adapter, not just `SX126x`
    LOG_INFO("SX126x init result %d", res);
    if (res == RADIOLIB_ERR_CHIP_NOT_FOUND || res == RADIOLIB_ERR_SPI_CMD_FAILED)
        return false;

    LOG_INFO("Frequency set to %f", getFreq());
    LOG_INFO("Bandwidth set to %f", bw);
    LOG_INFO("Power output set to %d", power);

    // Overriding current limit
    // (https://github.com/jgromes/RadioLib/blob/690a050ebb46e6097c5d00c371e961c1caa3b52e/src/modules/SX126x/SX126x.cpp#L85) using
    // value in SX126xInterface.h (currently 140 mA) It may or may not be necessary, depending on how RadioLib functions, from
    // SX1261/2 datasheet: OCP after setting DeviceSel with SetPaConfig(): SX1261 - 60 mA, SX1262 - 140 mA For the SX1268 the IC
    // defaults to 140mA no matter the set power level, but RadioLib set it lower, this would need further checking Default values
    // are: SX1262, SX1268: 0x38 (140 mA), SX1261: 0x18 (60 mA)
    // FIXME: Not ideal to increase SX1261 current limit above 60mA as it can only transmit max 15dBm, should probably only do it
    // if using SX1262 or SX1268
    res = lora.setCurrentLimit(currentLimit);
    LOG_DEBUG_RADIO("Current limit set to %f", currentLimit);
    LOG_DEBUG_RADIO("Current limit set result %d", res);

    if (res == RADIOLIB_ERR_NONE) {
#ifdef SX126X_DIO2_AS_RF_SWITCH
        bool dio2AsRfSwitch = true;
#elif defined(ARCH_PORTDUINO)
        bool dio2AsRfSwitch = false;
        if (portduino_config.dio2_as_rf_switch) {
            dio2AsRfSwitch = true;
        }
#else
        bool dio2AsRfSwitch = false;
#endif
        res = lora.setDio2AsRfSwitch(dio2AsRfSwitch);
        LOG_DEBUG_RADIO("Set DIO2 as %sRF switch, result: %d", dio2AsRfSwitch ? "" : "not ", res);
    }

    if (res == RADIOLIB_ERR_NONE && tcxoVoltage > 0) {
        // Standby and TX/RX fallback on STDBY_XOSC keep the TCXO powered: from STDBY_RC every SetRx and SetTx
        // first waits out the TCXO start-up delay (5 ms by RadioLib's default).
        const int16_t xoscRes = lora.setStandbyXOSC(true);
        LOG_DEBUG("Keep TCXO on in standby, result: %d", xoscRes);
    }

// If a pin isn't defined, we set it to RADIOLIB_NC, it is safe to always do external RF switching with RADIOLIB_NC as it has
// no effect
#if ARCH_PORTDUINO
    if (res == RADIOLIB_ERR_NONE) {
        LOG_DEBUG_RADIO("Use MCU pin %i as RXEN, pin %i as TXEN for RF switching", portduino_config.lora_rxen_pin.pin,
                        portduino_config.lora_txen_pin.pin);
        lora.setRfSwitchPins(portduino_config.lora_rxen_pin.pin, portduino_config.lora_txen_pin.pin);
    }
#else
#ifndef SX126X_RXEN
#define SX126X_RXEN RADIOLIB_NC
    LOG_DEBUG_RADIO("SX126X_RXEN not defined, default RADIOLIB_NC");
#endif
#ifndef SX126X_TXEN
#define SX126X_TXEN RADIOLIB_NC
    LOG_DEBUG_RADIO("SX126X_TXEN not defined, default RADIOLIB_NC");
#endif
    if (res == RADIOLIB_ERR_NONE) {
        LOG_DEBUG_RADIO("Use MCU pin %i as RXEN, pin %i as TXEN for RF switching", SX126X_RXEN, SX126X_TXEN);
        lora.setRfSwitchPins(SX126X_RXEN, SX126X_TXEN);
    }
#endif
    if (config.lora.sx126x_rx_boosted_gain) {
        uint16_t result = lora.setRxBoostedGainMode(true);
        LOG_INFO("Set RX gain to boosted mode; result: %d", result);
    } else {
        uint16_t result = lora.setRxBoostedGainMode(false);
        LOG_INFO("Set RX gain to power saving mode; result: %d", result);
    }

    // Undocumented SX1262 register patch recommended by Heltec/Semtech for improved RX sensitivity.
    // Sets bit 0 of register 0x8B5.
    if (module.SPIsetRegValue(0x8B5, 0x01, 0, 0) == RADIOLIB_ERR_NONE) {
        LOG_INFO("Applied SX1262 reg 0x8B5 RX patch");
    } else {
        LOG_WARN("Can't apply SX1262 reg 0x8B5 RX patch");
    }

    if (res == RADIOLIB_ERR_NONE)
        res = lora.setCRC(RADIOLIB_SX126X_LORA_CRC_ON);

#ifdef SX126X_NO_POWER_OPTIMIZATION_TABLE
    // begin() applied the optimization table; re-apply the fixed PA config.
    if (res == RADIOLIB_ERR_NONE)
        res = lora.setOutputPower(power, false);
#endif

    if (res != RADIOLIB_ERR_NONE)
        LOG_ERROR("SX126x re-init failed %s%d", radioLibErr, res);
    return res == RADIOLIB_ERR_NONE;
}

template <typename T> int16_t SX126xInterface<T>::programAltModemFSK()
{
    const meshtastic::NexusModule::NexusAltModem &m = meshtastic::nexusModule->altModem;
    // beginFSK switches packet type LoRa -> GFSK: (br kbps, freqDev kHz, rxBw kHz,
    // preamble bits). Frequency is programmed separately below.
    // QUALIFIED CALL — do NOT un-qualify: SX1262/SX1268/LLCC68 declare their own
    // beginFSK(freq, br, freqDev, rxBw, ...) chip-level overload with NO
    // `using SX126x::beginFSK`, so C++ name hiding binds an unqualified 4-arg call
    // to THAT overload with our bit rate read as the FREQUENCY — bench 2026-10-17:
    // SETFSK 50 25 39 16 became setFrequency(50.0 MHz) -> RADIOLIB_ERR_INVALID_
    // FREQUENCY (-12, band is 150-960) -> chip re-inited to LoRa -> retried -> same
    // -> "radio down until reboot". The base-class overload (SX126x.h:105) takes no
    // freq argument — this is that one; our tail below re-programs freq/power.
    // Floor preamble at 32 bits for reliable bit-sync lock at higher data rates
    uint16_t preambleBits = max(m.preambleBits, (uint16_t)32);
    int16_t err = lora.SX126x::beginFSK(m.brKbps, m.freqDevKHz, m.rxBwKHz, preambleBits);
    if (err != RADIOLIB_ERR_NONE) {
        LOG_ERROR("SX126X beginFSK(%.1fk/%.1fk/%.1fk/%u) %s%d", m.brKbps, m.freqDevKHz, m.rxBwKHz,
                  preambleBits, radioLibErr, err);
        return err;
    }

    // Uniform Nexus wire profile (research doc 2026-10-09 §7; Jenny ruling 2026-10-14):
    // 4-byte balanced sync word nexusAltSyncWord (0x2D, 0x4B, 0xD4, 0x93) across the entire fleet
    // to prevent receiver false-correlator locks on noise, whitening OFF, NRZ, variable packet mode,
    // and 2-byte CCITT inverted CRC (0x1D0F/0x1021).
    err = lora.setFrequency(getFreq());
    if (err != RADIOLIB_ERR_NONE) {
        LOG_ERROR("SX126X setFrequency(%.3f) %s%d", getFreq(), radioLibErr, err);
        return err;
    }
    uint8_t altSync[sizeof(nexusAltSyncWord)];
    memcpy(altSync, nexusAltSyncWord, sizeof(altSync));
    err = lora.setSyncWord(altSync, sizeof(altSync)); // GFSK multi-byte overload
    if (err != RADIOLIB_ERR_NONE) {
        LOG_ERROR("SX126X GFSK setSyncWord %s%d", radioLibErr, err);
        return err;
    }
    err = lora.setCRC(2, 0x1D0F, 0x1021, true); // len, init, poly, inverted — uniform CCITT
    if (err != RADIOLIB_ERR_NONE) {
        LOG_ERROR("SX126X GFSK setCRC %s%d", radioLibErr, err);
        return err;
    }

    // Power / current limit / RF switch / RX gain — same tail as reinitChip() since
    // beginFSK() hardware-resets the chip and unconditionally calls setDio2AsRfSwitch(true).
#ifdef SX126X_PA_RAMP_US
    lora.setPaRampTime(SX126X_PA_RAMP_US);
#endif
    err = lora.setCurrentLimit(currentLimit);
    if (err != RADIOLIB_ERR_NONE) {
        LOG_ERROR("SX126X setCurrentLimit %s%d", radioLibErr, err);
        return err;
    }
    {
#ifdef SX126X_DIO2_AS_RF_SWITCH
        bool dio2AsRfSwitch = true;
#elif defined(ARCH_PORTDUINO)
        bool dio2AsRfSwitch = portduino_config.dio2_as_rf_switch;
#else
        bool dio2AsRfSwitch = false;
#endif
        (void)lora.setDio2AsRfSwitch(dio2AsRfSwitch);
    }
#if ARCH_PORTDUINO
    lora.setRfSwitchPins(portduino_config.lora_rxen_pin.pin, portduino_config.lora_txen_pin.pin);
#else
    lora.setRfSwitchPins(SX126X_RXEN, SX126X_TXEN);
#endif
    limitPower(SX126X_MAX_POWER);
    if (power < -9)
        power = -9;
#ifdef SX126X_NO_POWER_OPTIMIZATION_TABLE
    err = lora.setOutputPower(power, false); // external PA: fixed PA config
#else
    err = lora.setOutputPower(power);
#endif
    if (err != RADIOLIB_ERR_NONE) {
        LOG_ERROR("SX126X setOutputPower %d dBm rejected (%s%d); keep previous Tx power", power, radioLibErr, err);
        RECORD_CRITICALERROR(meshtastic_CriticalErrorCode_INVALID_RADIO_SETTING);
    }
    err = lora.setRxBoostedGainMode(config.lora.sx126x_rx_boosted_gain);
    if (err != RADIOLIB_ERR_NONE)
        LOG_WARN("SX126X setRxBoostedGainMode %s%d", radioLibErr, err);

    (void)module.SPIsetRegValue(0x8B5, 0x01, 0, 0);

    LOG_INFO("SX126X GFSK armed: BR=%.1fk dev=%.1fk bw=%.1fk pre=%u freq=%.3f", m.brKbps, m.freqDevKHz,
             m.rxBwKHz, m.preambleBits, getFreq());
    return RADIOLIB_ERR_NONE;
}

template <typename T> int16_t SX126xInterface<T>::programModemParams()
{
    // Nexus SETFSK (Phase 1, Jenny rulings 2026-10-14): when the alternate modem is
    // armed, program GFSK instead of LoRa. Frequency/power/current-limit still come
    // from applyModemConfig()'s normal pipeline — only the modulation layer changes.
    // Persist-until-disabled: altModem is re-read on every reconfigure, so admin
    // pushes / channel changes don't silently fall back to LoRa mid-session.
    if (meshtastic::nexusModule && meshtastic::nexusModule->altModem.mode == 1) {
        return programAltModemFSK();
    }

    // configure publicly accessible settings
    int16_t err = lora.setSpreadingFactor(sf);
    // Nexus SETFSK: Returning to LoRa from FSK modem triggers RADIOLIB_ERR_WRONG_MODEM
    // because hardware packet type is still GFSK. Re-init chip cleanly to restore LoRa
    // packet mode without triggering a spurious critical error.
    if (err == RADIOLIB_ERR_WRONG_MODEM) {
        LOG_INFO("SX126x wrong modem (returning to LoRa), re-initializing");
        if (!reinitChip()) {
            return RADIOLIB_ERR_CHIP_NOT_FOUND;
        }
        err = lora.setSpreadingFactor(sf);
    }
    if (err != RADIOLIB_ERR_NONE) {
        LOG_ERROR("SX126X setSpreadingFactor(%u) %s%d", sf, radioLibErr, err);
        return err;
    }

    err = lora.setBandwidth(bw);
    if (err != RADIOLIB_ERR_NONE) {
        LOG_ERROR("SX126X setBandwidth(%.1f) %s%d", bw, radioLibErr, err);
        return err;
    }

    err = lora.setCodingRate(cr);
    if (err != RADIOLIB_ERR_NONE) {
        LOG_ERROR("SX126X setCodingRate(%u) %s%d", cr, radioLibErr, err);
        return err;
    }

    err = lora.setSyncWord(syncWord);
    if (err != RADIOLIB_ERR_NONE) {
        LOG_ERROR("SX126X setSyncWord %s%d", radioLibErr, err);
        return err;
    }

    err = lora.setCurrentLimit(currentLimit);
    if (err != RADIOLIB_ERR_NONE) {
        LOG_ERROR("SX126X setCurrentLimit %s%d", radioLibErr, err);
        return err;
    }

    err = lora.setPreambleLength(preambleLength);
    if (err != RADIOLIB_ERR_NONE) {
        LOG_ERROR("SX126X setPreambleLength(%u) %s%d", preambleLength, radioLibErr, err);
        return err;
    }

    err = lora.setFrequency(getFreq());
    if (err != RADIOLIB_ERR_NONE) {
        LOG_ERROR("SX126X setFrequency(%.3f) %s%d", getFreq(), radioLibErr, err);
        return err;
    }

    limitPower(SX126X_MAX_POWER);
    // Make sure we reach the minimum power supported to turn the chip on (-9dBm)
    if (power < -9)
        power = -9;

#ifdef SX126X_NO_POWER_OPTIMIZATION_TABLE
    err = lora.setOutputPower(power, false); // external PA: fixed PA config
#else
    err = lora.setOutputPower(power);
#endif
    if (err != RADIOLIB_ERR_NONE) {
        // Don't abort: this power is operator config (tx_power/SX126X_MAX_POWER); a value above the
        // driver's max would crash the daemon before reloadConfig() persists. Flag it and keep prior power.
        LOG_ERROR("SX126X setOutputPower %d dBm rejected (%s%d); keep previous Tx power", power, radioLibErr, err);
        RECORD_CRITICALERROR(meshtastic_CriticalErrorCode_INVALID_RADIO_SETTING);
    }

    // Apply RX gain mode - valid in STDBY (datasheet §9.6), matches resetAGC() pattern
    err = lora.setRxBoostedGainMode(config.lora.sx126x_rx_boosted_gain);
    if (err != RADIOLIB_ERR_NONE)
        LOG_WARN("SX126X setRxBoostedGainMode %s%d", radioLibErr, err);

    return RADIOLIB_ERR_NONE;
}

template <typename T> int16_t SX126xInterface<T>::setOutputPowerLive(int8_t powerDb)
{
    limitPower(SX126X_MAX_POWER);
    if (powerDb < -9)
        powerDb = -9;
    if (powerDb > SX126X_MAX_POWER)
        powerDb = SX126X_MAX_POWER;
#ifdef SX126X_NO_POWER_OPTIMIZATION_TABLE
    int16_t err = lora.setOutputPower(powerDb, false);
#else
    int16_t err = lora.setOutputPower(powerDb);
#endif
    if (err == RADIOLIB_ERR_NONE) {
        power = powerDb;
    }
    return err;
}

template <typename T> int16_t SX126xInterface<T>::setSpreadingFactorLive(uint8_t targetSf, int8_t targetPwr)
{
    int16_t err = lora.setSpreadingFactor(targetSf);
    if (err != RADIOLIB_ERR_NONE)
        return err;
    sf = targetSf;
    if (targetPwr != 0x7F) {
        return setOutputPowerLive(targetPwr);
    }
    return RADIOLIB_ERR_NONE;
}

template <typename T> bool SX126xInterface<T>::reconfigure()
{
    // A readout between these calls would clear the flags they set up, or move the chip out from under them
    RadioSequence seq(this);
    RadioLibInterface::reconfigure();

#ifdef MESHTASTIC_RX_READOUT_TASK
    // A reconfigure standbys the chip and owns the re-arm that follows, so an RX the readout task armed at TX_DONE is
    // void from here. Left set, adoptReceiveArmedFromIsr() would report armed for an RX that no longer exists and, via
    // the base startReceive() it calls, clear rxOffline and chipRecoveryFailures for a chip that never took one -
    // zeroing the ladder and the rxOffline that periodicRadioMaintenance() gates its retry on.
    rearmState = REARM_NONE;
#endif

    // set mode to standby - a chip that lost its state to a reset/brownout can time out here (-707),
    // so don't let setStandby()'s assert fire before the recovery below gets a chance
    int16_t err = trySetStandby();
    if (err == RADIOLIB_ERR_NONE)
        err = programModemParams();

    if (err != RADIOLIB_ERR_NONE) {
        // Chip likely lost its state (reset/brownout); recover in place rather than crash - see
        // RadioLibInterface::recoverChipStateLoss().
        RECORD_CRITICALERROR(meshtastic_CriticalErrorCode_INVALID_RADIO_SETTING);
        LOG_ERROR("SX126x rejected modem params, chip state lost? Full re-init");
        if (!reinitChip() || (err = programModemParams()) != RADIOLIB_ERR_NONE) {
            LOG_ERROR("SX126x unrecoverable %s%d, radio down until reboot", radioLibErr, err);
            return false;
        }
        LOG_INFO("SX126x recovered after re-init");
    }

#ifdef ARCH_PORTDUINO
    // RadioLib's TX staging ends in fixSensitivity(), which programs the sensitivity register from the configured
    // bandwidth (datasheet 15.1), and setBandwidth() does not redo it. The modem params just moved, so the next TX
    // has to be a full staging and not the launch that skips the register fixes.
    txStagedByRadioLib = false;
    prestagedLen = 0;
    earlyStagedLen = 0;
#endif

    startReceive(); // restart receiving

    return true;
}

template <typename T> int16_t SX126xInterface<T>::getCurrentRSSI()
{
    float rssi = lora.getRSSI(false);
    return (int16_t)round(rssi);
}

template <typename T> void SX126xInterface<T>::setRadioIsr(void (*callback)())
{
#ifdef LORA_DIO1_SOFTWARE_POLL
    irqPollingActive = true;
    pollTxMode = isIsrTxCallback(callback);
    scheduleIrqPollTick();
#else
    lora.setDio1Action(callback);
#endif
}

template <typename T> void SX126xInterface<T>::clearRadioIsr()
{
#ifdef LORA_DIO1_SOFTWARE_POLL
    irqPollingActive = false;
#else
    lora.clearDio1Action();
#endif
}

#ifdef LORA_DIO1_SOFTWARE_POLL
template <typename T> void SX126xInterface<T>::handleSoftwareLoraIrqPoll()
{
    if (!irqPollingActive)
        return;

    // getIrqFlags()/clearIrqFlags() both operate on the raw SX126x IRQ register, so use the
    // chip-specific RADIOLIB_SX126X_IRQ_* masks on both the read and the clear.
    uint16_t irq = lora.getIrqFlags();
    const uint16_t rxEventMask =
        RADIOLIB_SX126X_IRQ_RX_DONE | RADIOLIB_SX126X_IRQ_TIMEOUT | RADIOLIB_SX126X_IRQ_CRC_ERR | RADIOLIB_SX126X_IRQ_HEADER_ERR;
    const uint16_t noisyRxMask = RADIOLIB_SX126X_IRQ_PREAMBLE_DETECTED | RADIOLIB_SX126X_IRQ_HEADER_VALID;

    // A bare PREAMBLE is mid-reception, not an RX event: readData() here would run on nothing. With a TX
    // queued it goes through the same hold as the TX-path look; HEADER_VALID stays latched for readData().
    const bool preambleOnly = (irq & RADIOLIB_SX126X_IRQ_PREAMBLE_DETECTED) && !(irq & RADIOLIB_SX126X_IRQ_HEADER_VALID);
    if (!pollTxMode && hasQueuedTx() && preambleOnly && ((irq & ~noisyRxMask) == 0U)) {
        holdOnPreamble();
        scheduleIrqPollTick();
        return;
    }

    if (pollTxMode) {
        if (irq & (RADIOLIB_SX126X_IRQ_TX_DONE | RADIOLIB_SX126X_IRQ_TIMEOUT)) {
            deliverPendingIrqFromPoll(ISR_TX);
            return;
        }
    } else if (irq & rxEventMask) {
        deliverPendingIrqFromPoll(ISR_RX);
        return;
    }

    scheduleIrqPollTick();
}
#endif

template <typename T> int16_t SX126xInterface<T>::trySetStandby()
{
    // Handle any pending interrupts before we force standby, but not a TX whose backoff timer is pending: it would start
    // here, before its delay is up, and the standby below would cut it off with its payload still in the radio
    checkNotificationExcept(TRANSMIT_DELAY_COMPLETED);

    int16_t err = lora.standby();
    if (err == RADIOLIB_ERR_SPI_CMD_TIMEOUT) {
        // After a bounded RX times out, the status byte returned with SET_STANDBY still carries that timeout, and
        // RadioLib reports it as a failed command although the chip took it. The next command sees a fresh status.
        err = lora.standby();
        LOG_DEBUG("SX126x standby reported a stale command timeout, retry %s%d", radioLibErr, err);
    }

    if (err != RADIOLIB_ERR_NONE)
        LOG_DEBUG_RADIO("SX126x standby %s%d", radioLibErr, err);
#ifdef ARCH_PORTDUINO
    if (err != RADIOLIB_ERR_NONE)
        portduino_status.LoRa_in_error = true;
#endif
    isReceiving = false; // If we were receiving, not any more
    activeReceiveStart = 0;
    rxArmedContinuous = false;
    disableInterrupt();
    completeSending(); // If we were sending, not anymore
    RadioLibInterface::setStandby();
    return err;
}

template <typename T> void SX126xInterface<T>::setStandby()
{
    int16_t err = trySetStandby();
#ifdef ARCH_PORTDUINO
    (void)err;
#else
    assert(err == RADIOLIB_ERR_NONE);
#endif
}

/**
 * Add SNR data to received messages
 */
template <typename T> void SX126xInterface<T>::addReceiveMetadata(meshtastic_MeshPacket *mp)
{
    // LOG_DEBUG("PacketStatus %x", lora.getPacketStatus());
    // Nexus SETFSK: FSK has no chirp SNR — derive a comparable pseudo-SNR from RSSI
    // (Jenny ruling 2026-10-14: "RSSI based") so getCWsize()/TX-delay math keeps
    // separating near/far nodes.
    if (meshtastic::nexusModule && meshtastic::nexusModule->altModem.mode == 1) {
        mp->rx_snr = meshtastic::NexusModule::altPseudoSnr(lora.getRSSI());
    } else {
        mp->rx_snr = lora.getSNR();
    }
    mp->rx_rssi = lround(lora.getRSSI());
    mp->has_rx_rssi = true; // rx_rssi has explicit presence - a genuine reading must be marked present to survive encoding
    // Nexus port (Nexus-28001, 2026-08-16): re-populate mp->rx_frequency_error. Upstream dropped
    // this field in 2.8.0 (Nexus restored it as a Nexus-only operator-data extension). Captured
    // here so every radio driver contributes frequency-error data to NexusModule's SIGINT tooling
    // (/nexus RSSI, /nexus PPING FREQERR, /nexus ALERT META, /nexus CRSTATS) and TraceRouteModule::lastRxFreqErr.
    mp->rx_frequency_error = lora.getFrequencyError();
    LOG_TRACE("Corrected frequency offset: %f", mp->rx_frequency_error);

    // Nexus port (Nexus-28001, 2026-08-16): re-populate mp->rx_coding_rate. Raw LoRa header CR
    // (1-7), see upstream's LOG_WARN path in RadioLibInterface.h:648 for the same call.
    // Used by NexusModule's cr_stats[][] accumulator (/nexus CRSTATS) and display paths.
    // RadioLib has no getCodingRate() getter; this is the only way to recover the sender's CR
    // on receive. Failures fall back to 0 (raw 0 = reserved / unknown), Nexus nexusCRDisplay()
    // maps it to display 0.
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
template <typename T> void SX126xInterface<T>::configHardwareForSend()
{
#ifdef MESHTASTIC_RX_READOUT_TASK
    rearmState = REARM_NONE; // only this TX's TX_DONE may re-arm, never a stale one from a TX the poll completed
    rxArmedBeforeTxDone = false;
#endif
#if NEXUS_ENABLE_JENCORE_HYBRID_SYNC
    // Use the packet's own sync word: JenCore frames go out with 0x12, Meshtastic with our normal sync word (0x2B or override).
    int err = lora.setSyncWord(isRawSending() ? jencoreSyncWord : syncWord);
    if (err != RADIOLIB_ERR_NONE)
        LOG_ERROR("SX126X setSyncWord %s%d", radioLibErr, err);
#endif
#if NEXUS_ENABLE_JENCORE_SNIFFER
    // JenCore listens for a longer preamble at low SF (RadioLibWrapper::preambleLengthForSF: 32 at SF<=8, else 16).
    // Ours is restored in startReceive().
    const uint16_t jencorePreamble = sf <= 8 ? 32 : 16;
    if (isRawSending() && jencorePreamble != preambleLength) {
        if (lora.setPreambleLength(jencorePreamble) == RADIOLIB_ERR_NONE)
            jencorePreambleSet = true;
    }
#endif
#if NEXUS_ENABLE_RNODE
    if (getRawSendingType() == RAW_SEND_RNODE) {
        lora.setSyncWord(0x12);
        if (rnodeEngine.getSf() != this->sf) {
            lora.setSpreadingFactor(rnodeEngine.getSf());
        }
    }
#endif
    setTransmitEnable(true);
    RadioLibInterface::configHardwareForSend();
}

#if NEXUS_ENABLE_JENCORE_HYBRID_SYNC
template <typename T> void SX126xInterface<T>::setSyncWordRegister(uint16_t registerValue)
{
    // RadioLib packs (syncWord, controlBits) nibble-wise into the register: MSB = sw.hi|cb.hi, LSB = sw.lo|cb.lo
    const uint8_t msb = registerValue >> 8;
    const uint8_t lsb = registerValue & 0xFF;
    const uint8_t sw = (msb & 0xF0) | (lsb >> 4);
    const uint8_t controlBits = ((msb & 0x0F) << 4) | (lsb & 0x0F);
    int err = lora.setSyncWord(sw, controlBits);
    if (err != RADIOLIB_ERR_NONE)
        LOG_ERROR("SX126X setSyncWord 0x%04x %s%d", registerValue, radioLibErr, err);
}
#endif

// For power draw measurements, helpful to force radio to stay sleeping
// #define SLEEP_ONLY

template <typename T> void SX126xInterface<T>::startReceive()
{
#ifdef SLEEP_ONLY
    sleep();
#else

    setTransmitEnable(false);

#if NEXUS_ENABLE_JENCORE_HYBRID_SYNC
    if (this->hybridSyncEnabled) {
        setSyncWordRegister(NEXUS_JENCORE_RX_SYNCWORD);
    } else {
        lora.setSyncWord(this->syncWord);
    }
#endif
#if NEXUS_ENABLE_JENCORE_SNIFFER
    // Back to our preamble after a JenCore frame: the RX duty cycle below is sized from it, and normal TX uses it.
    if (jencorePreambleSet) {
        lora.setPreambleLength(preambleLength);
        jencorePreambleSet = false;
    }
#endif
#if NEXUS_ENABLE_RNODE
    if (rnodeEngine.getMode() == RNODE_MODE_STANDALONE) {
        lora.setSyncWord(0x12);
        lora.setSpreadingFactor(rnodeEngine.getSf());
        rnodeRadioDirty = false;
        rnodeWasStandalone = true;
    } else if (rnodeRadioDirty || rnodeWasStandalone) {
#if !NEXUS_ENABLE_JENCORE_HYBRID_SYNC
        lora.setSyncWord(this->syncWord);
#endif
        lora.setSpreadingFactor(this->sf);
        rnodeRadioDirty = false;
        rnodeWasStandalone = false;
    }
#endif

    auto tryStartRx = [&]() -> int16_t { return startRxCommand(); };

    int16_t err = trySetStandby();
    if (err == RADIOLIB_ERR_NONE)
        err = tryStartRx();

    if (err != RADIOLIB_ERR_NONE) {
        LOG_ERROR("SX126X startReceive %s%d", radioLibErr, err);
        if (maybeRecoverChipStateLoss())
            err = tryStartRx();
    }

    if (err != RADIOLIB_ERR_NONE) {
#ifdef ARCH_PORTDUINO
        portduino_status.LoRa_in_error = true;
#else
        // No assert: leave RX off rather than reboot; periodicRadioMaintenance() re-arms it, throttled
        LOG_ERROR("SX126X RX offline %s%d", radioLibErr, err);
        rxOffline = true;
        return;
#endif
    }

    RadioLibInterface::startReceive();
    rxArmedContinuous = true;
#ifdef ARCH_PORTDUINO
    rxWritePtr = 0;         // RX's base, which RadioLib's RX start always sets to 0
    rxClobberCheck = false; // the standby before it dropped the frame a stage was noted against
#endif

    // Must be done AFTER, starting transmit, because startTransmit clears (possibly stale) interrupt pending register bits
    enableInterrupt(isrRxLevel0);
    checkRxDoneIrqFlag();
#endif
}

template <typename T> bool SX126xInterface<T>::resumeRunningReceive()
{
    // Continuous RX survives RX_DONE and CRC or header errors: the chip is still listening. A restart is a standby
    // and ~10 bus transactions, deaf throughout on a CH341 host, so pick the RX back up instead.
    if (!rxArmedContinuous)
        return false;
    // RX_DONE and CRC_ERR are never cleared here: whoever read a frame out cleared its own flags, so anything still
    // latched belongs to a frame that has not been read, and checkRxDoneIrqFlag() below drives that readout. Clearing
    // them would erase a frame that finished while we got here - on a CH341 host the readout's own bus traffic, the
    // overlap check included, is several round trips wide. The early outs in handleReceiveInterrupt() that return
    // before readData(), and readData()'s own error returns, clear them where they give up. PREAMBLE/HEADER_VALID
    // stay: they may belong to the next frame, already arriving. A stale HEADER_ERR or TIMEOUT has no readout
    // waiting and would cost checkStaleRxFlags() a re-arm, so those two go unconditionally.
    lora.clearIrqFlags(RADIOLIB_SX126X_IRQ_HEADER_ERR | RADIOLIB_SX126X_IRQ_TIMEOUT);
    activeReceiveStart = 0; // as the standby this replaces would
    RadioLibInterface::startReceive();
    enableInterrupt(isrRxLevel0);
#ifdef ARCH_PORTDUINO
    // Name this look's own catch: "caught missed RX_DONE" alone cannot say which found it, as pollMissedIrqs(),
    // rearmReceive() and startReceive() all log it too, and the first fires on a timer.
    if (checkRxDoneIrqFlag()) // an RX_DONE the resume carried across
        LOG_DEBUG("RX resume kept RX_DONE for readout");
#else
    checkRxDoneIrqFlag(); // an RX_DONE the resume carried across
#endif
    return true;
}

// RadioLib's duty cycle listens for 2 x minSymbols and sleeps through whatever preamble is left
static constexpr uint16_t rxDutyCycleMinSymbols = 8;

template <typename T> int16_t SX126xInterface<T>::startRxCommand()
{
    // Every preamble we ask for is no longer than the window the duty cycle stays awake for, so it would sleep through
    // nothing and startReceiveDutyCycleAuto() falls back to this same command anyway. Sending it outright is what lets
    // rxArmedContinuous describe the RX the chip is really in, so resumeRunningReceive() can pick it back up after
    // RX_DONE instead of paying for a standby and a restart. A CH341 host and WASM need it for their own reasons: a
    // restart is deaf for several round trips over that bus, and duty-cycle sleep parks BUSY high between windows and
    // stalls the WebUSB link. If a preamble ever outgrows the wake window, restore the duty-cycled arm and a flag
    // saying which one was sent; this assert is here to say so.
    static_assert(preambleLengthDefault <= 2 * rxDutyCycleMinSymbols &&
                      wideLoraPreambleLengthDefault <= 2 * rxDutyCycleMinSymbols,
                  "a preamble longer than the duty cycle's wake window would really sleep, so this arm would no longer "
                  "be the continuous RX rxArmedContinuous claims");
    return lora.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF, MESHTASTIC_RADIOLIB_IRQ_RX_FLAGS);
}

#ifdef MESHTASTIC_RX_READOUT_TASK
template <typename T> bool INTERRUPT_ATTR SX126xInterface<T>::rearmReceiveFromIsr()
{
    // The interrupt cannot call RadioLib, so the readout task, above the main loop, re-arms as soon as it returns.
    rearmState = REARM_PENDING;
    if (requestRearmFromIsr())
        return true;
    rearmState = REARM_NONE;
    return false;
}

template <typename T> void SX126xInterface<T>::rearmReceiveFromTask()
{
    // Outside any radio-thread sequence. If the thread got there first, it took the re-arm over and left nothing to do.
    RadioSequence seq(this);
    if (rearmState != REARM_PENDING)
        return;
    // What startReceive() sends, less the standby: after TX_DONE the chip has already fallen back to standby.
    setTransmitEnable(false);
    const int16_t err = startRxCommand();
    rearmErr = err;
    if (err != RADIOLIB_ERR_NONE) {
        rearmState = REARM_FAILED;
        return;
    }
    rearmState = REARM_ARMED;
    // The TX_DONE interrupt detached the radio's interrupt. Attach the RX one now, so a frame that ends before the radio
    // thread runs wakes this task to read it, rather than waiting in the chip for the next frame to overwrite it.
    rxArmedBeforeTxDone = true;
    enableInterrupt(isrRxLevel0);
}

template <typename T> bool SX126xInterface<T>::adoptReceiveArmedFromIsr()
{
    // Called inside the post-TX sequence, so the task cannot be mid re-arm: it has either finished or not started, and
    // clearing the state here stops it starting.
    const uint8_t state = rearmState;
    rearmState = REARM_NONE;
    rxArmedBeforeTxDone = false; // this is the TX_DONE; frames the task read meanwhile were delivered ahead of it
    if (state == REARM_FAILED)
        LOG_WARN("SX126X RX re-arm at TX_DONE failed %s%d, restarting RX", radioLibErr, rearmErr);
    if (state != REARM_ARMED)
        return false;
    RadioLibInterface::startReceive();
    rxArmedContinuous = true;
    enableInterrupt(isrRxLevel0);
    checkRxDoneIrqFlag(); // an RX_DONE that completed before the interrupt was attached
    return true;
}
#endif

/** Is the channel currently active? */
template <typename T> bool SX126xInterface<T>::isChannelActive()
{
    // check if we can detect a LoRa preamble on the current channel.
    // NOTE: symNum is the *encoded* SET_CAD_PARAMS value, not a raw count - RADIOLIB_SX126X_CAD_ON_4_SYMB
    // (== raw 2) runs the 4-symbol scan getCadSymbolCountSubGhz() reports; keep the two in step.
    // Exit CAD straight into RX on detection (GOTO_RX) with the RX IRQs already mapped, so the chip's
    // own CAD->RX transition delivers RX_DONE with no library call in between. irqFlags is
    // the status-enable set (CAD verdict + full RX set incl. PREAMBLE/HEADER_VALID for isActivelyReceiving);
    // irqMask is the DIO-trigger set - CAD verdict + terminal RX events only, NOT PREAMBLE/HEADER_VALID,
    // which would fire the ISR mid-frame and run readData() on an incomplete packet.
    const uint32_t cadIrqFlags = RADIOLIB_IRQ_CAD_DEFAULT_FLAGS | MESHTASTIC_RADIOLIB_IRQ_RX_FLAGS;
    const uint32_t cadIrqMask = RADIOLIB_IRQ_CAD_DEFAULT_MASK | (1UL << RADIOLIB_IRQ_RX_DONE) | (1UL << RADIOLIB_IRQ_TIMEOUT) |
                                (1UL << RADIOLIB_IRQ_CRC_ERR) | (1UL << RADIOLIB_IRQ_HEADER_ERR);
    // cadTimeout bounds the RX the chip enters on detection: one max-length airtime, in microseconds. A
    // detection that delivers nothing then expires to standby and raises TIMEOUT, which re-arms us.
    const RadioLibTime_t cadRxTimeoutUsec =
        (RadioLibTime_t)getPacketTime(meshtastic_Constants_DATA_PAYLOAD_LEN + sizeof(PacketHeader), false) * 1000;
    ChannelScanConfig_t cfg = {.cad = {.symNum = RADIOLIB_SX126X_CAD_ON_4_SYMB,
                                       .detPeak = RADIOLIB_SX126X_CAD_PARAM_DEFAULT,
                                       .detMin = RADIOLIB_SX126X_CAD_PARAM_DEFAULT,
                                       .exitMode = RADIOLIB_SX126X_CAD_GOTO_RX,
                                       .timeout = cadRxTimeoutUsec,
                                       .irqFlags = cadIrqFlags,
                                       .irqMask = cadIrqMask}};
    // Nexus SETFSK: GFSK has no LoRa CAD — dynamic RSSI energy sniff instead:
    // busy = 10 dB over the larger of the mode's sensitivity and the noise floor.
    // Wide FSK filters (e.g. 467 kHz at 300k) have elevated noise floors that exceed -100 dBm.
    // MUST sample instantaneous RSSI while still in continuous RX mode: entering standby
    // shuts down the receiver/ADC, dropping getRSSI(false) to the standby noise floor (~-128 dBm)
    // and rendering channel activity detection completely blind.
    if (meshtastic::nexusModule && meshtastic::nexusModule->altModem.mode == 1) {
        float rssi = lora.getRSSI(false);
        float threshold = max(altSensitivityDbm(), (float)getNoiseFloor()) + 10.0f;
        LOG_TRACE("SX126X FSK channel sniff RSSI=%.1f dBm (busy above %.1f)", rssi, threshold);
        if (rssi > threshold)
            return true;
        setTransmitEnable(false);
        int16_t result = trySetStandby();
        if (result != RADIOLIB_ERR_NONE)
            LOG_ERROR("SX126X trySetStandby %s%d", radioLibErr, result);
        return false;
    }

#ifdef ARCH_PORTDUINO
    prestagedLen = 0; // only a clear verdict from this scan may launch what it stages
    prestagedInRx = false;
    if (!takeEarlyTxStage() && stageTxInRx())
        return true; // report busy without the standby, which would abort the frame; rearmReceive() keeps the RX
#endif

    setTransmitEnable(false);
    int16_t result = trySetStandby();
    if (result == RADIOLIB_ERR_NONE) {
#ifdef ARCH_PORTDUINO
        prestageTx();
#endif
        result = scanChannelForTx(cfg);
#ifdef ARCH_PORTDUINO
        if (result != RADIOLIB_CHANNEL_FREE)
            prestagedLen = 0; // no TX follows, and a detection's RX may overwrite the buffer
#endif
        if (result == RADIOLIB_LORA_DETECTED) {
            // The chip auto-entered RX (GOTO_RX). Drop the latched CAD verdict so the pin releases and the
            // coming RX_DONE is a clean edge.
            lora.clearIrqFlags(RADIOLIB_SX126X_IRQ_CAD_DONE | RADIOLIB_SX126X_IRQ_CAD_DETECTED);
#ifdef ARCH_PORTDUINO
            rxWritePtr = 0;         // assumed: entering RX from the CAD restarts the write point too; the readout corrects it
            rxClobberCheck = false; // the scan's standby dropped any frame a stage was noted against
#endif
            noteCadHandoffToRx();
            return true;
        }
        if (result != RADIOLIB_CHANNEL_FREE)
            LOG_ERROR("SX126X scanChannel %s%d", radioLibErr, result);
        if (result != RADIOLIB_ERR_WRONG_MODEM)
            return false;
    }
#ifdef ARCH_PORTDUINO
    portduino_status.LoRa_in_error = true;
#endif
    // standby failed or the LoRa modem type is gone - the chip lost its runtime state
    maybeRecoverChipStateLoss();
    return false; // report the channel free: a recovered chip can TX, a dead one fails startSend safely
}

template <typename T> int16_t SX126xInterface<T>::scanChannelForTx(const ChannelScanConfig_t &cfg)
{
    if (!irqPolledOverUsb())
        return lora.scanChannel(cfg);
    // lora.scanChannel(cfg) less its two packet-type reads and its standby, which trySetStandby() has just done, and
    // with the CAD parameters sent only when they change: three commands instead of six before the CAD starts.
    module.setRfSwitchState(Module::MODE_RX);
    const uint32_t irqs = lora.getIrqMapped(cfg.cad.irqFlags);
    const uint32_t dio1 = lora.getIrqMapped(cfg.cad.irqMask);
    const uint8_t dioIrqParams[] = {
        (uint8_t)(irqs >> 8), (uint8_t)(irqs & 0xFF), (uint8_t)(dio1 >> 8), (uint8_t)(dio1 & 0xFF), 0, 0, 0, 0};
    int16_t res = module.SPIwriteStream(RADIOLIB_SX126X_CMD_SET_DIO_IRQ_PARAMS, dioIrqParams, sizeof(dioIrqParams));
    if (res == RADIOLIB_ERR_NONE)
        res = lora.clearIrqFlags(RADIOLIB_SX126X_IRQ_ALL);
    // As RadioLib's setCad(): a default detection peak is SF + 13, and the timeout counts 15.625 us steps
    const uint32_t timeoutRaw = (uint32_t)((float)cfg.cad.timeout / 15.625f);
    const uint8_t cadParams[7] = {
        cfg.cad.symNum,
        cfg.cad.detPeak != RADIOLIB_SX126X_CAD_PARAM_DEFAULT ? cfg.cad.detPeak : (uint8_t)(sf + 13),
        cfg.cad.detMin != RADIOLIB_SX126X_CAD_PARAM_DEFAULT ? cfg.cad.detMin : (uint8_t)RADIOLIB_SX126X_CAD_PARAM_DET_MIN,
        cfg.cad.exitMode,
        (uint8_t)((timeoutRaw >> 16) & 0xFF),
        (uint8_t)((timeoutRaw >> 8) & 0xFF),
        (uint8_t)(timeoutRaw & 0xFF)};
    if (res == RADIOLIB_ERR_NONE && (!cadParamsValid || memcmp(cadParams, cadParamsSent, sizeof(cadParams)) != 0)) {
        res = module.SPIwriteStream(RADIOLIB_SX126X_CMD_SET_CAD_PARAMS, cadParams, sizeof(cadParams));
        cadParamsValid = res == RADIOLIB_ERR_NONE;
        if (cadParamsValid)
            memcpy(cadParamsSent, cadParams, sizeof(cadParams));
    }
    if (res == RADIOLIB_ERR_NONE)
        res = module.SPIwriteStream(RADIOLIB_SX126X_CMD_SET_CAD, nullptr, 0);
    if (res != RADIOLIB_ERR_NONE)
        return res;
    // As scanChannel(): wait for DIO1 to report the CAD finished, then read the verdict from the IRQ flags alone
    while (!module.hal->digitalRead(module.getIrq()))
        module.hal->yield();
    const uint32_t irq = lora.getIrqFlags();
    if (irq & RADIOLIB_SX126X_IRQ_CAD_DETECTED)
        return RADIOLIB_LORA_DETECTED;
    if (irq & RADIOLIB_SX126X_IRQ_CAD_DONE)
        return RADIOLIB_CHANNEL_FREE;
    return RADIOLIB_ERR_UNKNOWN;
}

#ifdef ARCH_PORTDUINO
template <typename T> void SX126xInterface<T>::prestageTx()
{
    // The payload goes in now, while nothing is listening anyway, rather than after the verdict. The CAD leaves the
    // buffer alone; a detection's RX may overwrite it, but then there is no TX and the next scan writes it again.
    if (prestagedLen || !scanForTx || !txStagedByRadioLib || !irqPolledOverUsb())
        return;
    const size_t numbytes = encodeRadioBuffer(scanForTx);
    if (numbytes == 0 || numbytes > RADIOLIB_SX126X_MAX_PACKET_LENGTH)
        return;
    const uint8_t writeBuffer[] = {RADIOLIB_SX126X_CMD_WRITE_BUFFER, 0x00}; // offset 0, RadioLib's TX base
    if (module.SPIwriteStream(writeBuffer, sizeof(writeBuffer), (uint8_t *)&radioBuffer, numbytes) == RADIOLIB_ERR_NONE) {
        prestagedLen = numbytes;
        prestagedId = scanForTx->id;
    }
}

template <typename T> int16_t SX126xInterface<T>::launchTransmit(size_t numbytes)
{
    const bool prestaged = prestagedLen != 0 && prestagedLen == numbytes && sendingPacket && sendingPacket->id == prestagedId;
    const bool inRx = prestagedInRx;
    prestagedLen = 0;
    prestagedInRx = false;
    earlyStagedLen = 0; // this TX takes the buffer, prestaged or not
    if (!prestaged) {
        const int16_t res = RadioLibInterface::launchTransmit(numbytes);
        if (res == RADIOLIB_ERR_NONE)
            txStagedByRadioLib = true;
        return res;
    }
    // What RadioLib's TX staging would send, less the buffer (already written), the buffer base (RadioLib only ever
    // uses 0/0), the register fixes (an earlier staging set them, and the chip keeps them until it loses its
    // registers) and the packet-type read. Packet params are the ones programModemParams() gives RadioLib.
    const uint8_t packetParams[] = {(uint8_t)(preambleLength >> 8),       (uint8_t)(preambleLength & 0xFF),
                                    RADIOLIB_SX126X_LORA_HEADER_EXPLICIT, (uint8_t)numbytes,
                                    RADIOLIB_SX126X_LORA_CRC_ON,          RADIOLIB_SX126X_LORA_IQ_STANDARD};
    const uint16_t irqMask = RADIOLIB_SX126X_IRQ_TX_DONE | RADIOLIB_SX126X_IRQ_TIMEOUT;
    const uint16_t dio1Mask = RADIOLIB_SX126X_IRQ_TX_DONE;
    const uint8_t dioIrqParams[] = {
        (uint8_t)(irqMask >> 8), (uint8_t)(irqMask & 0xFF), (uint8_t)(dio1Mask >> 8), (uint8_t)(dio1Mask & 0xFF), 0, 0, 0, 0};
    const uint8_t clearAll[] = {(uint8_t)(RADIOLIB_SX126X_IRQ_ALL >> 8), (uint8_t)(RADIOLIB_SX126X_IRQ_ALL & 0xFF)};
    int16_t res = RADIOLIB_ERR_NONE;
    if (inRx) {
        // Staged behind RX's write point: TX from there. RadioLib sets both bases back to 0 at the next RX start.
        const uint8_t bases[] = {prestagedBase, 0x00};
        res = module.SPIwriteStream(RADIOLIB_SX126X_CMD_SET_BUFFER_BASE_ADDRESS, bases, sizeof(bases));
    }
    if (res == RADIOLIB_ERR_NONE)
        res = module.SPIwriteStream(RADIOLIB_SX126X_CMD_SET_PACKET_PARAMS, packetParams, sizeof(packetParams));
    if (res == RADIOLIB_ERR_NONE)
        res = module.SPIwriteStream(RADIOLIB_SX126X_CMD_SET_DIO_IRQ_PARAMS, dioIrqParams, sizeof(dioIrqParams));
    if (res == RADIOLIB_ERR_NONE)
        res = module.SPIwriteStream(RADIOLIB_SX126X_CMD_CLEAR_IRQ_STATUS, clearAll, sizeof(clearAll));
    if (res != RADIOLIB_ERR_NONE)
        return res;
    module.setRfSwitchState(Module::MODE_TX);
    const uint8_t txTimeout[] = {0, 0, 0}; // RADIOLIB_SX126X_TX_TIMEOUT_NONE: single TX
    res = module.SPIwriteStream(RADIOLIB_SX126X_CMD_SET_TX, txTimeout, sizeof(txTimeout), false);
    if (res != RADIOLIB_ERR_NONE)
        return res;
    // As RadioLib's launchMode(): BUSY drops once the PA has ramped, after any oscillator start-up
    const uint32_t start = Time::getMillis();
    while (module.hal->digitalRead(module.getGpio())) {
        if (Throttle::hasElapsed(start, 100)) {
            LOG_WARN("Prestaged TX: BUSY still high after 100 ms");
            break;
        }
        module.hal->yield();
    }
    return RADIOLIB_ERR_NONE;
}

template <typename T> uint8_t SX126xInterface<T>::txStageBase(size_t numbytes) const
{
    // The bytes just behind the write point are the last RX reaches again. Kept clear of the wrap, which the chip's
    // TX would have to follow.
    return rxWritePtr >= numbytes ? (uint8_t)(rxWritePtr - numbytes) : (uint8_t)(256 - numbytes);
}

/** Whether two ranges of the chip's 256-byte buffer, which wraps, share a byte */
static bool bufferRangesOverlap(uint8_t a, size_t lenA, uint8_t b, size_t lenB)
{
    return lenA && lenB && ((uint8_t)(b - a) < lenA || (uint8_t)(a - b) < lenB);
}

template <typename T> void SX126xInterface<T>::noteStagedOverFrame(uint8_t base, size_t numbytes)
{
    rxClobberCheck = true;
    rxClobberBase = base;
    rxClobberLen = numbytes;
    memcpy(rxClobberBytes, &radioBuffer, numbytes); // still the payload just written
}

template <typename T> bool SX126xInterface<T>::stageTxInRx()
{
    if (!scanForTx || !txStagedByRadioLib || !irqPolledOverUsb())
        return false;
    const size_t numbytes = encodeRadioBuffer(scanForTx);
    if (numbytes == 0 || numbytes > RADIOLIB_SX126X_MAX_PACKET_LENGTH)
        return false;
    const uint8_t base = txStageBase(numbytes);
    const uint8_t writeBuffer[] = {RADIOLIB_SX126X_CMD_WRITE_BUFFER, base};
    if (module.SPIwriteStream(writeBuffer, sizeof(writeBuffer), (uint8_t *)&radioBuffer, numbytes) != RADIOLIB_ERR_NONE)
        return false;
    const uint32_t irq = lora.getIrqFlags();
    const uint32_t doneIrqs = RADIOLIB_SX126X_IRQ_RX_DONE | RADIOLIB_SX126X_IRQ_CRC_ERR;
    // A preamble shows ~2 ms into a frame, its header ~5 ms in: the scan's standby would abort either
    const uint32_t arrivingIrqs = RADIOLIB_SX126X_IRQ_HEADER_VALID | RADIOLIB_SX126X_IRQ_PREAMBLE_DETECTED;
    if (irq & (arrivingIrqs | doneIrqs)) {
        // Its readout checks whether our bytes can lie across it. A bare preamble counts: that frame can still
        // grow into the staged bytes, no later look records the check, and the record is spent on the next readout.
        noteStagedOverFrame(base, numbytes);
        // Name the bare-preamble record: with neither a header nor a terminal flag there is nothing else in the
        // log to tell it from a record the readout would have had anyway
        if (!(irq & (RADIOLIB_SX126X_IRQ_HEADER_VALID | doneIrqs)))
            LOG_DEBUG("TX stage recorded over a bare preamble");
        LOG_DEBUG("TX stage in RX: frame arriving or unread (irq 0x%04x), scan skipped", (unsigned)irq);
        return true;
    }
    prestagedLen = numbytes;
    prestagedId = scanForTx->id;
    prestagedBase = base;
    prestagedInRx = true;
    return false;
}

template <typename T> bool SX126xInterface<T>::rxFrameOverlapsTxStage(size_t length)
{
    const bool clobberCheck = rxClobberCheck;
    rxClobberCheck = false; // the frame flagged at the stage is the next one read out
    if (!irqPolledOverUsb())
        return false;
    uint8_t offset = 0;
    if (lora.getPacketLength(false, &offset) == 0 && length != 0)
        LOG_DEBUG("RX buffer status unreadable, frame placement unknown");
    rxWritePtr = (uint8_t)(offset + length);
    if (earlyStagedLen && bufferRangesOverlap(offset, length, earlyStagedBase, earlyStagedLen)) {
        earlyStagedLen = 0; // this frame's bytes went over the staged payload: stage it again
        LOG_DEBUG("TX staged early: overwritten by a %u-byte rx frame at 0x%02x, restage", (unsigned)length, (unsigned)offset);
    }
    if (!clobberCheck || !bufferRangesOverlap(offset, length, rxClobberBase, rxClobberLen))
        return false;
    // The chip writes a frame forward at ~0.37 ms a byte (SHORT_TURBO); our write sweeps forward far faster. So where
    // our bytes lie over the frame's, they do so from the first byte the two share: if that byte is ours the frame is
    // damaged, and if it is the frame's, so is every later shared byte. An intact frame matches ours there 1 in 256.
    const uint8_t shared = (uint8_t)(offset - rxClobberBase) < rxClobberLen ? offset : rxClobberBase;
    const uint8_t frameByte = ((const uint8_t *)&radioBuffer)[(uint8_t)(shared - offset)];
    const bool ours = frameByte == rxClobberBytes[(uint8_t)(shared - rxClobberBase)];
    LOG_DEBUG("RX frame at 0x%02x shares 0x%02x with a stage at 0x%02x: %s", (unsigned)offset, (unsigned)shared,
              (unsigned)rxClobberBase, ours ? "ours landed last, drop" : "the frame landed last, keep");
    return ours;
}

template <typename T> bool SX126xInterface<T>::wantsEarlyTxStage() const
{
    return txStagedByRadioLib && irqPolledOverUsb();
}

template <typename T> void SX126xInterface<T>::stageTxEarly(meshtastic_MeshPacket *p)
{
    // Only while RX runs: a TX in flight is using the buffer, and SPI would wake a sleeping chip
    if (!p || sendingPacket || !isReceiving || !wantsEarlyTxStage())
        return;
    const size_t numbytes = encodeRadioBuffer(p);
    if (numbytes == 0 || numbytes > RADIOLIB_SX126X_MAX_PACKET_LENGTH)
        return;
    if (earlyStagedLen == numbytes && memcmp(earlyStagedBytes, &radioBuffer, numbytes) == 0)
        return; // a redraw of the same packet: still in the buffer
    earlyStagedLen = 0;
    const uint32_t doneIrqs = RADIOLIB_SX126X_IRQ_RX_DONE | RADIOLIB_SX126X_IRQ_CRC_ERR;
    // With a frame in the buffer the write point is not known yet: wait for its readout, which redraws the backoff and
    // brings us back here
    const uint32_t irqBefore = lora.getIrqFlags();
    if (irqBefore & (RADIOLIB_SX126X_IRQ_HEADER_VALID | doneIrqs)) {
        LOG_DEBUG("TX staged early: deferred, a frame is in the buffer (irq 0x%04x)", (unsigned)irqBefore);
        return;
    }
    const uint8_t base = txStageBase(numbytes);
    const uint8_t writeBuffer[] = {RADIOLIB_SX126X_CMD_WRITE_BUFFER, base};
    if (module.SPIwriteStream(writeBuffer, sizeof(writeBuffer), (uint8_t *)&radioBuffer, numbytes) != RADIOLIB_ERR_NONE)
        return;
    // No standby follows, so a frame that began during the write is received as usual; its readout checks it
    if (lora.getIrqFlags() & (RADIOLIB_SX126X_IRQ_HEADER_VALID | doneIrqs))
        noteStagedOverFrame(base, numbytes);
    earlyStagedLen = numbytes;
    earlyStagedId = p->id;
    earlyStagedBase = base;
    memcpy(earlyStagedBytes, &radioBuffer, numbytes);
}

template <typename T> bool SX126xInterface<T>::takeEarlyTxStage()
{
    bool held = false;
    if (earlyStagedLen && scanForTx) {
        const size_t numbytes = encodeRadioBuffer(scanForTx); // CPU only, no bus traffic
        held = numbytes == earlyStagedLen && memcmp(earlyStagedBytes, &radioBuffer, numbytes) == 0;
    }
    if (!held) {
        if (earlyStagedLen)
            LOG_DEBUG("TX staged early: not the packet being scanned for, restage at the scan");
        earlyStagedLen = 0; // whatever this scan stages goes over it
        return false;
    }
    prestagedLen = earlyStagedLen;
    prestagedId = earlyStagedId;
    prestagedBase = earlyStagedBase;
    prestagedInRx = true;
    return true;
}
#endif

/** Could we send right now (i.e. either not actively receiving or transmitting)? */
template <typename T> bool SX126xInterface<T>::isActivelyReceiving()
{
    // The IRQ status will be cleared when we start our read operation. Check if we've started a header, but haven't yet
    // received and handled the interrupt for reading the packet/handling errors.
    // Nexus SETFSK: LoRa detects packet start on HEADER_VALID; GFSK detects on
    // SYNC_WORD_VALID (bit 3 — the slot the header flag occupies for LoRa).
    if (meshtastic::nexusModule && meshtastic::nexusModule->altModem.mode == 1) {
        const uint16_t irq = lora.getIrqFlags();
        const uint16_t latch = RADIOLIB_SX126X_IRQ_SYNC_WORD_VALID | RADIOLIB_SX126X_IRQ_PREAMBLE_DETECTED;
        const bool detected = receiveDetected(irq, RADIOLIB_SX126X_IRQ_SYNC_WORD_VALID,
                                              RADIOLIB_SX126X_IRQ_PREAMBLE_DETECTED);
        // Bench 2026-10-17 (heltec V3/V4 FSK): GFSK's pattern-matching preamble/sync
        // detector false-trips on noise, and on the DIO1 interrupt path nobody clears
        // those non-terminal bits (handleReceiveInterrupt's short-packet path returns
        // before readData; the edge was already consumed) -> latch sticks -> busyRx
        // forever -> TX starved ("Can not send yet, busyRx" spam). Clear it once
        // receiveDetected() has ruled it stale. A real packet reaches RX_DONE and is
        // cleared by readData() as usual.
        if (!detected && (irq & latch)) {
            lora.clearIrqFlags(latch);
            LOG_TRACE("SX126X FSK stale RX latch cleared (irq=0x%04X)", (unsigned)irq);
        }
        return detected;
    }
    return receiveDetected(lora.getIrqFlags(), RADIOLIB_SX126X_IRQ_HEADER_VALID, RADIOLIB_SX126X_IRQ_PREAMBLE_DETECTED);
}

template <typename T> bool SX126xInterface<T>::sleep()
{
    // Not keeping config is busted - next time nrf52 board boots lora sending fails  tcxo related? - see datasheet
    // \todo Display actual typename of the adapter, not just `SX126x`
    LOG_DEBUG_RADIO("SX126x entering sleep mode"); // (FIXME, don't keep config)
#ifdef ARCH_PORTDUINO
    txStagedByRadioLib = false; // sleep does not keep every register; let RadioLib stage the next TX in full
    earlyStagedLen = 0;         // nor the buffer
#endif
    cadParamsValid = false; // likewise the CAD parameters
    // trySetStandby() and the sleep command below are one sequence: a readout landing between them talks to a chip
    // that is on its way down, and the standby would be undone by the arm a readout's caller can follow it with.
    RadioSequence seq(this);
    (void)trySetStandby(); // Stop any pending operations - the chip is being put to sleep, a failure must not crash

    // turn off TCXO if it was powered
    // FIXME - this isn't correct
    // lora.setTCXO(0);

    // put chipset into sleep mode. Interrupts are not necessarily detached here - disable() and NodeDB's
    // config-reload park both reach this with the ISR still attached - so the lock above is what keeps
    // the readout task out of it rather than the absence of interrupts.
    bool keepConfig = true;
    lora.sleep(keepConfig); // Note: we do not keep the config, full reinit will be needed

#ifdef SX126X_POWER_EN
    digitalWrite(SX126X_POWER_EN, LOW);
#endif

#if HAS_LORA_FEM
    loraFEMInterface.setSleepModeEnable();
#endif

    return true;
}

template <typename T> bool SX126xInterface<T>::resetAGC()
{
    // Safety: don't reset mid-packet
    if (sendingPacket != NULL || (isReceiving && isActivelyReceiving()))
        return false;

    LOG_DEBUG_RADIO("SX126x AGC reset: warm sleep + Calibrate(0x7F)");
#ifdef ARCH_PORTDUINO
    txStagedByRadioLib = false; // as in sleep(): the next TX gets RadioLib's full staging
    earlyStagedLen = 0;
#endif
    cadParamsValid = false; // and the CAD parameters are sent again

    // 1. Warm sleep - powers down the entire analog frontend, resetting AGC state.
    //    A plain standby→startReceive cycle does NOT reset the AGC.
    lora.sleep(true);

    // 2. Wake to RC standby for stable calibration
    lora.standby(RADIOLIB_SX126X_STANDBY_RC, true);

    // 3. Calibrate all blocks (ADC, PLL, image, RC oscillators)
    uint8_t calData = RADIOLIB_SX126X_CALIBRATE_ALL;
    module.SPIwriteStream(RADIOLIB_SX126X_CMD_CALIBRATE, &calData, 1, true, false);

    // 4. Wait for calibration to complete (BUSY pin goes low)
    module.hal->delay(5);
    uint32_t start = millis();
    while (module.hal->digitalRead(module.getGpio())) {
        if (millis() - start > 50)
            break;
        module.hal->yield();
    }

    if (module.hal->digitalRead(module.getGpio())) {
        LOG_WARN("SX126x AGC reset: calibration not done in 50ms");
        startReceive();
        return false; // incomplete: retried on the next maintenance tick
    }

    // 5. Re-calibrate image rejection for actual operating frequency
    //    Calibrate(0x7F) defaults to 902-928 MHz which is wrong for other regions.
    lora.calibrateImage(getFreq());
    // 6. CalibrateImage keeps working internally after it returns, and BUSY does not
    //    stay asserted for it; a register write in that window fails write-verify
    //    (RADIOLIB_ERR_SPI_WRITE_FAILED) and stalls the chip.
    module.hal->delay(50);

    // Re-apply settings that calibration may have reset

    // DIO2 as RF switch
#ifdef SX126X_DIO2_AS_RF_SWITCH
    lora.setDio2AsRfSwitch(true);
#elif defined(ARCH_PORTDUINO)
    if (portduino_config.dio2_as_rf_switch)
        lora.setDio2AsRfSwitch(true);
#endif

    // RX boosted gain mode
    lora.setRxBoostedGainMode(config.lora.sx126x_rx_boosted_gain);

    // Re-apply the undocumented 0x8B5 RX sensitivity patch that was set in init().
    // The CALIBRATE_ALL (0x7F) command above clears bit 0 of register 0x8B5, which
    // silently removes the RX sensitivity improvement introduced in #9571 / #9777.
    // Without this re-apply, every SX1262 node loses its RX boost ~60s after boot
    // and never recovers until reboot. See empirical evidence in the PR description.
    const bool patched = module.SPIsetRegValue(0x8B5, 0x01, 0, 0) == RADIOLIB_ERR_NONE;
    if (!patched) {
        LOG_WARN("SX126x resetAGC: 0x8B5 RX patch re-apply failed");
    }

    // 7. Resume receiving
    startReceive();
    return patched; // without the patch the reset is incomplete: retried on the next maintenance tick
}

template <typename T> uint8_t SX126xInterface<T>::readChipState()
{
    // BUSY pin high = chip processing a command. Skip SPI this tick (RadioLib would
    // block on the busy-wait up to its 1 s timeout) and count the state instead.
    if (module.getGpio() != RADIOLIB_NC && module.hal->digitalRead(module.getGpio()))
        return CHIP_BUSY;

    // RadioLib's vendored SX126x::getStatus() calls SPIreadStream(..., numBytes=0),
    // which copies 0 bytes and always yields 0 (broken upstream; fixed in RadioLib
    // master by zeroing the status width first). Replicate the FIXED pattern through
    // the public Module API: with status width 0 the status byte lands in the 1-byte
    // payload. (Status parse callback stays armed: it reads buffIn[1] in-bounds here.)
    const uint8_t cmd = RADIOLIB_SX126X_CMD_GET_STATUS;
    uint8_t data = 0;
    module.spiConfig.widths[RADIOLIB_MODULE_SPI_WIDTH_STATUS] = Module::BITS_0;
    (void)module.SPItransferStream(&cmd, 1, false, NULL, &data, 1, true);
    module.spiConfig.widths[RADIOLIB_MODULE_SPI_WIDTH_STATUS] = Module::BITS_8;

    switch (data & 0b01110000) {
    case RADIOLIB_SX126X_STATUS_MODE_STDBY_RC:
    case RADIOLIB_SX126X_STATUS_MODE_STDBY_XOSC:
        return CHIP_STDBY;
    case RADIOLIB_SX126X_STATUS_MODE_FS:
        return CHIP_FS;
    case RADIOLIB_SX126X_STATUS_MODE_RX:
        return CHIP_RX;
    case RADIOLIB_SX126X_STATUS_MODE_TX:
        return CHIP_TX;
    default:
        return CHIP_UNK; // 0xFF SPI-failed pattern or undocumented mode
    }
}

/** Control PA mode for GC1109 FEM - CPS pin selects full PA (txon=true) or bypass mode (txon=false) */
template <typename T> void SX126xInterface<T>::setTransmitEnable(bool txon)
{
#if HAS_LORA_FEM
    if (txon) {
        loraFEMInterface.setTxModeEnable();
    } else {
        loraFEMInterface.setRxModeEnable();
    }
#endif
}

/// Approximate SX126x sensitivity (Semtech datasheet table 13-3) of the active FSK setting
template <typename T> float SX126xInterface<T>::altSensitivityDbm()
{
    const auto &m = meshtastic::nexusModule->altModem;
    return m.brKbps >= 250 ? -99.0f : m.brKbps >= 150 ? -103.0f : m.brKbps >= 50 ? -108.0f : m.brKbps >= 10 ? -114.0f : -118.0f;
}

#endif
