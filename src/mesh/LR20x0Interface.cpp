#include "configuration.h"

#if (defined(USE_LR2021) || defined(ARCH_PORTDUINO)) && RADIOLIB_EXCLUDE_LR2021 != 1
#include "LR20x0Band.h"
#include "LR20x0Interface.h"
#include "LR2021Interface.h" // Nexus-28005: apply hook for /nexus LR2021_SF uses LR2021Interface accessors
#include "error.h"
#include "mesh/NodeDB.h"
#include "modules/NexusModule.h" // Nexus SETFSK/SETFLRC: altModem state (Phase 2: LR2021/X1)
#if NEXUS_ENABLE_RNODE
#include "mesh/RNodeEngine.h"
#endif

#if defined(LR2021_DCDC_WORKAROUND) && RADIOLIB_GODMODE
// The DCDC sensitivity workaround pokes RadioLib-internal DCDC registers that are NOT exposed via the
// public LR2021.h, so pull in the internal register map explicitly. Opt-in only (see LR2021_DCDC_WORKAROUND).
#include <modules/LR2021/LR2021_registers.h>
#endif

// Keep LR20x0 naming while RadioLib exposes LR2021 symbols.
#ifndef LR20x0
#define LR20x0 LR2021
#endif

#ifdef LR2021_DIO_AS_RF_SWITCH
#include "rfswitch.h"
#elif ARCH_PORTDUINO
#include "PortduinoGlue.h"

// Switch-capable DIOs in slot order with this part's constants.
static const int8_t lr20x0_switch_dio_nums[] = {5, 6, 7, 8, 9, 10, 11};
static const uint32_t lr20x0_switch_dio_consts[] = {RADIOLIB_LR2021_DIO5, RADIOLIB_LR2021_DIO6, RADIOLIB_LR2021_DIO7,
                                                    RADIOLIB_LR2021_DIO8, RADIOLIB_LR2021_DIO9, RADIOLIB_LR2021_DIO10,
                                                    RADIOLIB_LR2021_DIO11};
static_assert(sizeof(lr20x0_switch_dio_nums) / sizeof(lr20x0_switch_dio_nums[0]) ==
                  sizeof(lr20x0_switch_dio_consts) / sizeof(lr20x0_switch_dio_consts[0]),
              "LR20x0 switch DIO numbers and constants must describe the same slots");

// This part has MODE_RX_HF and no MODE_TX_HP/MODE_GNSS/MODE_WIFI.
static const int32_t lr20x0_rfswitch_mode_map[RFSW_MODE_COUNT] = {
    LR20x0::MODE_STBY,  LR20x0::MODE_RX,    LR20x0::MODE_TX,       RFSW_MODE_UNSUPPORTED,
    LR20x0::MODE_TX_HF, LR20x0::MODE_RX_HF, RFSW_MODE_UNSUPPORTED, RFSW_MODE_UNSUPPORTED,
};

static uint32_t lr20x0_rfswitch_dio_pins[Module::RFSWITCH_MAX_PINS];
static Module::RfSwitchMode_t lr20x0_rfswitch_table[RFSW_MODE_COUNT + 1];
#else
static const uint32_t lr20x0_rfswitch_dio_pins[] = {RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC};
static const Module::RfSwitchMode_t lr20x0_rfswitch_table[] = {
    {LR20x0::MODE_STBY, {}},  {LR20x0::MODE_RX, {}},    {LR20x0::MODE_TX, {}},
    {LR20x0::MODE_RX_HF, {}}, {LR20x0::MODE_TX_HF, {}}, END_OF_MODE_TABLE,
};
#endif

#ifdef LR2021_CUSTOM_PA_TABLE
#include "pa_table.h"
#endif

// Particular boards might define a different max power based on what their hardware can do, default to max power output if not
// specified (may be dangerous if using external PA and LR20x0 power config forgotten)
#if ARCH_PORTDUINO
#define LR2021_MAX_POWER portduino_config.lr2021_max_power
#endif
#ifndef LR2021_MAX_POWER
#define LR2021_MAX_POWER 22
#endif

// the 2.4G part maxes at 12dBm

#if ARCH_PORTDUINO
#define LR2021_MAX_POWER_HF portduino_config.lr2021_max_power_hf
#endif
#ifndef LR2021_MAX_POWER_HF
#define LR2021_MAX_POWER_HF 12
#endif

// Last programmed carrier; LF/HF hops use full begin() (live setOutputPower returns -706).
static float lr20x0LastFreqMHz = 0;

// Unlike SX126x/LR11x0 (on/off bool), the LR2021 RX gain boost is a 0-7 level (0 = disabled, 7 = max boost).
// Map the historical on/off sx126x_rx_boosted_gain flag to max boost when enabled.
#ifndef LR2021_RX_GAIN_BOOST_LEVEL
#define LR2021_RX_GAIN_BOOST_LEVEL 7
#endif

template <typename T>
LR20x0Interface<T>::LR20x0Interface(LockingArduinoHal *hal, RADIOLIB_PIN_TYPE cs, RADIOLIB_PIN_TYPE irq, RADIOLIB_PIN_TYPE rst,
                                    RADIOLIB_PIN_TYPE busy)
    : RadioLibInterface(hal, cs, irq, rst, busy, &lora), lora(&module)
{
    LOG_WARN("LR20x0Interface(cs=%d, irq=%d, rst=%d, busy=%d)", cs, irq, rst, busy);
}

/// Initialise the Driver transport hardware and software.
/// Make sure the Driver is properly configured before calling init().
/// \return true if initialisation succeeded.
template <typename T> bool LR20x0Interface<T>::init()
{
#ifdef LR2021_POWER_EN
    pinMode(LR2021_POWER_EN, OUTPUT);
    digitalWrite(LR2021_POWER_EN, HIGH);
#endif

#if ARCH_PORTDUINO
    // An explicit Vref wins; probing with none given tries the radio default first.
    float tcxoVoltage;
    if (portduino_config.dio3_tcxo_voltage > 0)
        tcxoVoltage = (float)portduino_config.dio3_tcxo_voltage / 1000;
    else if (TCXO_OPTIONAL_ENABLED)
        tcxoVoltage = TCXO_OPTIONAL_DEFAULT_VOLTAGE;
    else
        tcxoVoltage = 0;
    if (portduino_config.dio3_tcxo_voltage <= 0 && TCXO_OPTIONAL_ENABLED)
        LOG_DEBUG("TCXO_OPTIONAL: no Lora.DIO3_TCXO_VOLTAGE set, trying default TCXO Vref %f V first", tcxoVoltage);
#elif defined(LR2021_DIO3_TCXO_VOLTAGE)
    float tcxoVoltage = LR2021_DIO3_TCXO_VOLTAGE;
    LOG_DEBUG("LR2021_DIO3_TCXO_VOLTAGE defined, DIO3 as TCXO Vref %f V", LR2021_DIO3_TCXO_VOLTAGE);
    // (DIO3 is not free to be used as an IRQ)
#elif defined(TCXO_OPTIONAL)
    float tcxoVoltage = 1.6f; // TCXO_OPTIONAL: try default 1.6 V first, fall back to XTAL on failure
    LOG_DEBUG("TCXO_OPTIONAL: no LR2021_DIO3_TCXO_VOLTAGE, try default TCXO Vref 1.6 V first");
#else
    float tcxoVoltage =
        0; // "TCXO reference voltage to be set on DIO3. Defaults to 1.6 V, set to 0 to skip." per
           // https://github.com/jgromes/RadioLib/blob/690a050ebb46e6097c5d00c371e961c1caa3b52e/src/modules/LR11x0/LR11x0.h#L471C26-L471C104
    // (DIO3 is free to be used as an IRQ)
    LOG_DEBUG("LR2021_DIO3_TCXO_VOLTAGE not defined, DIO3 not used as TCXO Vref");
#endif

    RadioLibInterface::init();

#ifdef LR2021_IRQ_DIO_NUM
    lora.irqDioNum = LR2021_IRQ_DIO_NUM;
    LOG_DEBUG("Set irqDioNum %d", lora.irqDioNum);
#elif defined(IRQ_DIO_NUM)
    lora.irqDioNum = IRQ_DIO_NUM;
    LOG_DEBUG("Set irqDioNum %d", lora.irqDioNum);
#elif defined(ARCH_PORTDUINO)
    // Unset keeps RadioLib's default of DIO5, which many carriers also drive as a switch line. The
    // range is checked again here because a DIO the radio cannot drive is a silently dead receiver.
    if (portduino_config.irq_dio_num < 0) {
        LOG_DEBUG("Use default irqDioNum %d", lora.irqDioNum);
    } else if (portduino_config.irq_dio_num >= kLr20x0IrqDioMin && portduino_config.irq_dio_num <= kLr20x0IrqDioMax) {
        lora.irqDioNum = portduino_config.irq_dio_num;
        LOG_DEBUG("Set irqDioNum %d from config", lora.irqDioNum);
    } else {
        LOG_WARN("Config irqDioNum %d outside DIO%d-DIO%d, using default irqDioNum %d", portduino_config.irq_dio_num,
                 kLr20x0IrqDioMin, kLr20x0IrqDioMax, lora.irqDioNum);
    }
#else
    LOG_DEBUG("Use default irqDioNum %d", lora.irqDioNum);
#endif

    if (config.lora.region == meshtastic_Config_LoRaConfig_RegionCode_LORA_24) { // clamp if wide freq range
        limitPower(LR2021_MAX_POWER_HF);
    } else {
        limitPower(LR2021_MAX_POWER); // default clamp for non-wide freq range
    }

#ifdef LR2021_RF_SWITCH_SUBGHZ
    pinMode(LR2021_RF_SWITCH_SUBGHZ, OUTPUT);
    digitalWrite(LR2021_RF_SWITCH_SUBGHZ, isLr20x0HighBand(getFreq()) ? LOW : HIGH);
    LOG_DEBUG("Set RF0 switch to %s", isLr20x0HighBand(getFreq()) ? "2.4GHz" : "SubGHz");
#endif

#ifdef LR2021_RF_SWITCH_2_4GHZ
    pinMode(LR2021_RF_SWITCH_2_4GHZ, OUTPUT);
    digitalWrite(LR2021_RF_SWITCH_2_4GHZ, isLr20x0HighBand(getFreq()) ? HIGH : LOW);
    LOG_DEBUG("Set RF1 switch to %s", isLr20x0HighBand(getFreq()) ? "2.4GHz" : "SubGHz");
#endif

    // Allow extra time for TCXO to stabilize after power-on
    delay(10);

    int res = lora.begin(getFreq(), bw, sf, cr, effectiveMainSyncword(), power, preambleLength, tcxoVoltage);

    // Retry if we get SPI command failed - some units need extra TCXO stabilization time
    if (res == RADIOLIB_ERR_SPI_CMD_FAILED) {
        LOG_WARN("LR20x0 init failed with %d (SPI_CMD_FAILED), retry after delay", res);
        delay(100);
        res = lora.begin(getFreq(), bw, sf, cr, effectiveMainSyncword(), power, preambleLength, tcxoVoltage);
    }

    // If init failed for any reason other than chip not found, retry without TCXO (XTAL mode)
    if (TCXO_OPTIONAL_ENABLED && res != RADIOLIB_ERR_NONE && res != RADIOLIB_ERR_CHIP_NOT_FOUND && tcxoVoltage > 0) {
        LOG_WARN("LR20x0 init failed with TCXO Vref %f V (err %d), retry without TCXO", tcxoVoltage, res);
        tcxoVoltage = 0;
        res = lora.begin(getFreq(), bw, sf, cr, effectiveMainSyncword(), power, preambleLength, tcxoVoltage);
        if (res == RADIOLIB_ERR_NONE)
            LOG_INFO("LR20x0 init success without TCXO (XTAL mode)");
    }

    // \todo Display actual typename of the adapter, not just `LR20x0`
    LOG_INFO("LR20x0 init result %d", res);
    if (res == RADIOLIB_ERR_CHIP_NOT_FOUND || res == RADIOLIB_ERR_SPI_CMD_FAILED)
        return false;

    // Some basic info about the module's explicit firmware version - no other info available
    // Currently requires radiolib godmode

#if RADIOLIB_GODMODE
    if (res == RADIOLIB_ERR_NONE) {
        uint8_t fwMajor = 0;
        uint8_t fwMinor = 0;
        int versionRes = lora.getVersion(&fwMajor, &fwMinor);
        if (versionRes == RADIOLIB_ERR_NONE)
            LOG_DEBUG("LR20x0 FW %d.%d", fwMajor, fwMinor);
    }
#endif

    // Semtech DCDC sensitivity workaround for sub-GHz operation - applied here after lora.begin() has set the
    // packet type and modulation params. reconfigure() reapplies it after its own modulation changes.
    if (res == RADIOLIB_ERR_NONE)
        applyDcdcWorkaround();

    applyCustomLfPaTable(getFreq());

    LOG_INFO("Frequency set to %f", getFreq());
    LOG_INFO("Bandwidth set to %f", bw);
    LOG_INFO("Power output set to %d", power);

    // Nexus-28005 (2026-09-13): boot-time safety-net apply for the
    // LR2021 multi-SF side-detector table. The /nexus LR2021_SF CLI handler
    // already applies immediately on the command (see NexusCommands.cpp), so
    // under normal operation this hook is a no-op (table is empty after
    // boot because storage is RAM-only). It catches two edge cases:
    //   1. Soft reset or fast-reboot that preserves RAM.
    //   2. Any future non-CLI path that writes sideSFs_[] without calling
    //      configureSideDetectors() — the hook re-establishes chip state
    //      from the stored table on next init().
    // No-op when the table is empty (configSideDetectors(nullptr, 0) sends
    // a clear-and-disable opcode to the chip). LR2021 hardware only — the
    // LR1121/LR1110/LR1120 derivatives don't have the side-detector command
    // (different opcode set), so we cast the pointer to LR2021Interface
    // first; the down-cast is safe because we're inside the
    // LR20x0Interface<LR2021> template instantiation.
    {
        LR2021Interface *lr2021 = static_cast<LR2021Interface *>(this);
        lr2021->applyStoredSideDetectors();
    }

    if (res == RADIOLIB_ERR_NONE)
        res = lora.setCRC(2);

    // Standard DCDC ramp timing from RadioLib workarounds (register 0x00F20024)
    // Currently requires radiolib godmode
#if RADIOLIB_GODMODE
    if (res == RADIOLIB_ERR_NONE) {
        uint8_t rampTimes[4] = {15, 15, 15, 15}; // Standard case for all conditions
        // godmode-only DCDC ramp tuning: log failures but don't fail init (radio is already up)
        int16_t rmRes = lora.setRegMode(RADIOLIB_LR2021_REG_MODE_SIMO_NORMAL, rampTimes);
        if (rmRes != RADIOLIB_ERR_NONE)
            LOG_WARN("LR2021 setRegMode failed: %d", rmRes);
    }
#endif

#ifdef LR2021_DIO_AS_RF_SWITCH
    bool dioAsRfSwitch = true;
#elif defined(ARCH_PORTDUINO)
    bool dioAsRfSwitch = portduino_config.has_rfswitch_table;
    if (dioAsRfSwitch)
        buildRfSwitchTable(lr20x0_rfswitch_dio_pins, lr20x0_rfswitch_table, RFSW_MODE_COUNT + 1, lr20x0_switch_dio_nums,
                           lr20x0_switch_dio_consts, sizeof(lr20x0_switch_dio_nums) / sizeof(lr20x0_switch_dio_nums[0]),
                           lr20x0_rfswitch_mode_map);
#else
    bool dioAsRfSwitch = false;
#endif

    if (dioAsRfSwitch) {
        lora.setRfSwitchTable(lr20x0_rfswitch_dio_pins, lr20x0_rfswitch_table);
        LOG_DEBUG("Set DIO RF switch");
    }

    if (res == RADIOLIB_ERR_NONE) {
        if (config.lora.sx126x_rx_boosted_gain) { // the name is unfortunate but historically accurate
            res = lora.setRxBoostedGainMode(LR2021_RX_GAIN_BOOST_LEVEL);
            LOG_INFO("Set RX gain to boosted mode (level %d); result: %d", LR2021_RX_GAIN_BOOST_LEVEL, res);
        } else {
            res = lora.setRxBoostedGainMode(0);
            LOG_INFO("Set RX gain to power saving mode (boosted mode off); result: %d", res);
        }
    }

    if (res == RADIOLIB_ERR_NONE)
        startReceive(); // start receiving

    lr20x0LastFreqMHz = getFreq();
    return res == RADIOLIB_ERR_NONE;
}

template <typename T> int16_t LR20x0Interface<T>::programAltModemFSK()
{
    const meshtastic::NexusModule::NexusAltModem &m = meshtastic::nexusModule->altModem;
    // fullBegin() already ran (RF-switch GPIOs + resolved TCXO + chip begin) — pass
    // lora.tcxoVoltage back through beginGFSK so the DIO3 reference fullBegin resolved
    // is preserved instead of RadioLib's 1.6 V default.
    // Floor preamble at 32 bits for reliable bit-sync lock at higher data rates
    uint16_t preambleBits = max(m.preambleBits, (uint16_t)32);
    int16_t err = lora.beginGFSK(getFreq(), m.brKbps, m.freqDevKHz, m.rxBwKHz, power,
                                 preambleBits, lora.tcxoVoltage);
    if (err != RADIOLIB_ERR_NONE) {
        LOG_ERROR("LR2021 beginGFSK(%.3fM/%.1fk/%.1fk/%.1fk/%d/%u) %s%d", getFreq(), m.brKbps,
                  m.freqDevKHz, m.rxBwKHz, power, preambleBits, radioLibErr, err);
        return err;
    }
    // Uniform Nexus wire profile: 4-byte balanced sync word nexusAltSyncWord (0x2D, 0x4B, 0xD4, 0x93)
    uint8_t altSync[sizeof(nexusAltSyncWord)];
    memcpy(altSync, nexusAltSyncWord, sizeof(altSync));
    err = lora.setSyncWord(altSync, sizeof(altSync));
    if (err != RADIOLIB_ERR_NONE) {
        LOG_ERROR("LR2021 GFSK setSyncWord %s%d", radioLibErr, err);
        return err;
    }
    err = lora.setCRC(2, 0x1D0F, 0x1021, true);
    if (err != RADIOLIB_ERR_NONE) {
        LOG_ERROR("LR2021 GFSK setCRC %s%d", radioLibErr, err);
        return err;
    }
    err = lora.setWhitening(false, 0x01FF);
    if (err != RADIOLIB_ERR_NONE) {
        LOG_ERROR("LR2021 GFSK setWhitening %s%d", radioLibErr, err);
        return err;
    }
    err = lora.setDataShaping(RADIOLIB_SHAPING_NONE);
    if (err != RADIOLIB_ERR_NONE) {
        LOG_ERROR("LR2021 GFSK setDataShaping %s%d", radioLibErr, err);
        return err;
    }

    // beginGFSK() internally calls modSetup() -> findChip() -> reset(), which hardware-resets
    // the LR2021 chip and wipes DIO RF switch config, custom PA table, RX boosted gain, and
    // DCDC regulator setup configured by fullBegin(). Re-apply them here.
    applyCustomLfPaTable(getFreq());
#ifdef LR2021_DIO_AS_RF_SWITCH
    lora.setRfSwitchTable(lr20x0_rfswitch_dio_pins, lr20x0_rfswitch_table);
#elif ARCH_PORTDUINO
    if (portduino_config.has_rfswitch_table)
        lora.setRfSwitchTable(lr20x0_rfswitch_dio_pins, lr20x0_rfswitch_table);
#endif
    (void)lora.setRxBoostedGainMode(config.lora.sx126x_rx_boosted_gain ? LR2021_RX_GAIN_BOOST_LEVEL : 0);
#if RADIOLIB_GODMODE
    {
        uint8_t rampTimes[4] = {15, 15, 15, 15};
        (void)lora.setRegMode(RADIOLIB_LR2021_REG_MODE_SIMO_NORMAL, rampTimes);
    }
#endif
    applyDcdcWorkaround();

    LOG_INFO("LR2021 GFSK armed: BR=%.1fk dev=%.1fk bw=%.1fk pre=%u freq=%.3f", m.brKbps,
             m.freqDevKHz, m.rxBwKHz, m.preambleBits, getFreq());
    return RADIOLIB_ERR_NONE;
}

template <typename T> int16_t LR20x0Interface<T>::programAltModemFLRC()
{
    const meshtastic::NexusModule::NexusAltModem &m = meshtastic::nexusModule->altModem;
    uint8_t cr = (m.cr == 2) ? RADIOLIB_LR2021_FLRC_CR_1_2
               : (m.cr == 3) ? RADIOLIB_LR2021_FLRC_CR_3_4
                             : RADIOLIB_LR2021_FLRC_CR_1_0;
    uint8_t shaping = (m.bt == 0) ? RADIOLIB_SHAPING_NONE
                    : (m.bt == 1) ? RADIOLIB_SHAPING_0_3
                                  : RADIOLIB_SHAPING_0_5;
    uint16_t preambleBits = max(m.preambleBits, (uint16_t)32);
    int16_t err = lora.beginFLRC(getFreq(), (uint16_t)m.brKbps, cr, power, preambleBits,
                                 shaping, lora.tcxoVoltage);
    if (err != RADIOLIB_ERR_NONE) {
        LOG_ERROR("LR2021 beginFLRC(%.3fM/%.1fk/cr=%u/%d/%u) %s%d", getFreq(), m.brKbps, cr,
                  power, preambleBits, radioLibErr, err);
        return err;
    }
    // Uniform Nexus wire profile: 4-byte balanced sync word nexusAltSyncWord (0x2D, 0x4B, 0xD4, 0x93)
    uint8_t altSync[sizeof(nexusAltSyncWord)];
    memcpy(altSync, nexusAltSyncWord, sizeof(altSync));
    err = lora.setSyncWord(altSync, sizeof(altSync));
    if (err != RADIOLIB_ERR_NONE) {
        LOG_ERROR("LR2021 FLRC setSyncWord %s%d", radioLibErr, err);
        return err;
    }

    // beginFLRC() internally calls modSetup() -> findChip() -> reset(), wiping DIO RF switch
    // config, custom PA table, RX boosted gain, and DCDC regulator setup. Re-apply them here.
    applyCustomLfPaTable(getFreq());
#ifdef LR2021_DIO_AS_RF_SWITCH
    lora.setRfSwitchTable(lr20x0_rfswitch_dio_pins, lr20x0_rfswitch_table);
#elif ARCH_PORTDUINO
    if (portduino_config.has_rfswitch_table)
        lora.setRfSwitchTable(lr20x0_rfswitch_dio_pins, lr20x0_rfswitch_table);
#endif
    (void)lora.setRxBoostedGainMode(config.lora.sx126x_rx_boosted_gain ? LR2021_RX_GAIN_BOOST_LEVEL : 0);
#if RADIOLIB_GODMODE
    {
        uint8_t rampTimes[4] = {15, 15, 15, 15};
        (void)lora.setRegMode(RADIOLIB_LR2021_REG_MODE_SIMO_NORMAL, rampTimes);
    }
#endif
    applyDcdcWorkaround();

    LOG_INFO("LR2021 FLRC armed: BR=%.1fk cr=%u bt=%u pre=%u freq=%.3f", m.brKbps, cr, shaping,
             m.preambleBits, getFreq());
    return RADIOLIB_ERR_NONE;
}

template <typename T> bool LR20x0Interface<T>::reconfigure()
{
    // Propagated to the return value below, separately from the chip-programming outcome, so a
    // base-class failure isn't masked as success.
    const bool reconfigureSuccess = RadioLibInterface::reconfigure();

    if (config.lora.region == meshtastic_Config_LoRaConfig_RegionCode_LORA_24) {
        limitPower(LR2021_MAX_POWER_HF);
    } else {
        limitPower(LR2021_MAX_POWER);
    }

    // Nexus SETFSK/SETFLRC (Phase 2): alt modem armed -> program GFSK/FLRC instead of
    // the incremental LoRa path below (persist-until-disabled ruling, 2026-10-14).
    // Mirrors the bandHop path: fullBegin() handles RF-switch GPIOs + resolved TCXO,
    // then the alt begin() switches packet type, then startReceive.
    if (meshtastic::nexusModule && meshtastic::nexusModule->altModem.mode != 0) {
        (void)trySetStandby();
        if (!fullBegin(getFreq()))
            return false;
        int16_t altErr = (meshtastic::nexusModule->altModem.mode == 1) ? programAltModemFSK()
                                                                      : programAltModemFLRC();
        if (altErr != RADIOLIB_ERR_NONE) {
            LOG_ERROR("LR2021 alt-modem programming failed %s%d", radioLibErr, altErr);
            RECORD_CRITICALERROR(meshtastic_CriticalErrorCode_INVALID_RADIO_SETTING);
            return false;
        }
        startReceive();
        return reconfigureSuccess;
    }

    const float freq = getFreq();
    const bool bandHop = lr20x0ReconfigurePath(lr20x0LastFreqMHz, freq) == Lr20x0ReconfigurePath::FullBegin;

    if (bandHop) {
        LOG_INFO("LR20x0 LF/HF band hop %.1f -> %.1f MHz, full begin()", lr20x0LastFreqMHz, freq);
        // fullBegin() hardware-resets the chip, so a standby failure is survivable here
        (void)trySetStandby();

        if (!fullBegin(freq))
            return false;

        startReceive();
        return reconfigureSuccess;
    }

    // Same-band reconfigure (previous incremental path)
    bool standbySuccess = true;
    int16_t standbyErr = trySetStandby();
    if (standbyErr != RADIOLIB_ERR_NONE)
        standbySuccess = false;

    if (standbyErr == RADIOLIB_ERR_NONE) {
        int err = lora.setFrequency(freq);
        if (err != RADIOLIB_ERR_NONE) {
            LOG_ERROR("LR20x0 setFrequency %.3f MHz %s%d", freq, radioLibErr, err);
            RECORD_CRITICALERROR(meshtastic_CriticalErrorCode_INVALID_RADIO_SETTING);
            standbySuccess = false;
        }

        err = lora.setSpreadingFactor(sf);
        // Nexus SETFSK/SETFLRC: Returning to LoRa from FSK/FLRC modem triggers RADIOLIB_ERR_WRONG_MODEM
        // because hardware packet type is still GFSK/FLRC. fullBegin() re-initializes the chip cleanly.
        if (err == RADIOLIB_ERR_WRONG_MODEM) {
            LOG_INFO("LR2021 wrong modem (returning to LoRa), full begin");
            (void)trySetStandby();
            if (!fullBegin(freq))
                return false;
            startReceive();
            return reconfigureSuccess;
        }
        if (err != RADIOLIB_ERR_NONE) {
            LOG_ERROR("LR20x0 setSpreadingFactor(%u) %s%d", sf, radioLibErr, err);
            RECORD_CRITICALERROR(meshtastic_CriticalErrorCode_INVALID_RADIO_SETTING);
            standbySuccess = false;
        }

        err = lora.setBandwidth(bw);
        if (err != RADIOLIB_ERR_NONE) {
            LOG_ERROR("LR20x0 setBandwidth(%.1f) %s%d", bw, radioLibErr, err);
            RECORD_CRITICALERROR(meshtastic_CriticalErrorCode_INVALID_RADIO_SETTING);
            standbySuccess = false;
        }

        err = lora.setCodingRate(cr, cr != 7);
        if (err != RADIOLIB_ERR_NONE) {
            LOG_ERROR("LR20x0 setCodingRate(%u) %s%d", cr, radioLibErr, err);
            RECORD_CRITICALERROR(meshtastic_CriticalErrorCode_INVALID_RADIO_SETTING);
            standbySuccess = false;
        }

        err = lora.setSyncWord(effectiveMainSyncword());
        if (err != RADIOLIB_ERR_NONE) {
            LOG_ERROR("LR20x0 setSyncWord %s%d", radioLibErr, err);
            RECORD_CRITICALERROR(meshtastic_CriticalErrorCode_INVALID_RADIO_SETTING);
            standbySuccess = false;
        }

        err = lora.setPreambleLength(preambleLength);
        if (err != RADIOLIB_ERR_NONE) {
            LOG_ERROR("LR20x0 setPreambleLength(%u) %s%d", preambleLength, radioLibErr, err);
            RECORD_CRITICALERROR(meshtastic_CriticalErrorCode_INVALID_RADIO_SETTING);
            standbySuccess = false;
        }

        err = lora.setOutputPower(power);
        if (err != RADIOLIB_ERR_NONE) {
            LOG_ERROR("LR20x0 setOutputPower %d dBm @ %.3f MHz %s%d", power, freq, radioLibErr, err);
            RECORD_CRITICALERROR(meshtastic_CriticalErrorCode_INVALID_RADIO_SETTING);
            standbySuccess = false;
        }

        // Warn-only, as in LR11x0: a rejected gain mode is cosmetic and not a lost-state signature, so
        // it must not drag reconfigure() into a full chip reset.
        err = lora.setRxBoostedGainMode(config.lora.sx126x_rx_boosted_gain ? LR2021_RX_GAIN_BOOST_LEVEL : 0);
        if (err != RADIOLIB_ERR_NONE)
            LOG_WARN("LR20x0 setRxBoostedGainMode %s%d", radioLibErr, err);
    }

    if (!standbySuccess) {
        // A chip that fails standby or rejects parameter programming (typically WRONG_MODEM, -20) has
        // lost its runtime configuration to a chip-internal reset or brownout. Recover in place with the
        // same full begin() the band-hop path uses - it hardware-resets the chip. Crashing here instead
        // would reboot before MeshService persists the config change that triggered us.
        LOG_ERROR("LR20x0 rejected modem params, chip state lost? Full re-init");
        if (!fullBegin(freq)) {
            LOG_ERROR("LR20x0 unrecoverable, radio down until reboot");
            return false;
        }
        LOG_INFO("LR20x0 recovered after re-init");
    }

    // Nexus-28005 (2026-09-13): re-apply the stored side detector
    // table on the same-band reconfigure path. setlora goes through
    // reconfigure() (this function), not init(), and the same-band path
    // reconfigures via per-parameter setters (setFrequency,
    // setSpreadingFactor, setBandwidth, etc.) — none of which touch the
    // side detector slots. Without this call, /nexus LR2021_SF 7 followed
    // by /nexus SETLORA leaves the chip in single-SF RX even though the
    // table is stored in RAM. The band-hop path (line 230) and recovery
    // path (line 297) both call fullBegin() which already applies; this
    // covers the fast per-parameter-setter path. Same source of truth
    // as the init() and fullBegin() hooks — see
    // LR2021Interface::applyStoredSideDetectors().
    {
        LR2021Interface *lr2021 = static_cast<LR2021Interface *>(this);
        lr2021->applyStoredSideDetectors();
    }

    // setSpreadingFactor/setBandwidth/setCodingRate each re-run setLoRaModulationParams(), which resets
    // the DCDC configure state, so reapply the workaround before we resume receiving.
    if (standbySuccess)
        applyDcdcWorkaround();

    startReceive();
    lr20x0LastFreqMHz = freq;
    return reconfigureSuccess;
}

// The chip-side re-init the band-hop and recovery paths share: front-end switch GPIOs for the target
// band, a fresh begin() (which hardware-resets the chip), CRC, DIO RF-switch table, and RX gain.
template <typename T> bool LR20x0Interface<T>::fullBegin(float freq)
{
    {
        // Match init(): external LF/HF front-end GPIOs (if board defines them).
#ifdef LR2021_RF_SWITCH_SUBGHZ
        pinMode(LR2021_RF_SWITCH_SUBGHZ, OUTPUT);
        digitalWrite(LR2021_RF_SWITCH_SUBGHZ, isLr20x0HighBand(freq) ? LOW : HIGH);
        LOG_DEBUG("Set RF0 switch to %s", isLr20x0HighBand(freq) ? "2.4GHz" : "SubGHz");
#endif
#ifdef LR2021_RF_SWITCH_2_4GHZ
        pinMode(LR2021_RF_SWITCH_2_4GHZ, OUTPUT);
        digitalWrite(LR2021_RF_SWITCH_2_4GHZ, isLr20x0HighBand(freq) ? HIGH : LOW);
        LOG_DEBUG("Set RF1 switch to %s", isLr20x0HighBand(freq) ? "2.4GHz" : "SubGHz");
#endif

#if ARCH_PORTDUINO
        float tcxoVoltage;
        if (portduino_config.dio3_tcxo_voltage > 0)
            tcxoVoltage = (float)portduino_config.dio3_tcxo_voltage / 1000;
        else if (TCXO_OPTIONAL_ENABLED)
            tcxoVoltage = TCXO_OPTIONAL_DEFAULT_VOLTAGE;
        else
            tcxoVoltage = 0;
#elif defined(LR2021_DIO3_TCXO_VOLTAGE)
        float tcxoVoltage = LR2021_DIO3_TCXO_VOLTAGE;
#elif defined(TCXO_OPTIONAL)
        float tcxoVoltage = TCXO_OPTIONAL_DEFAULT_VOLTAGE;
#else
        float tcxoVoltage = 0;
#endif

        delay(10); // same TCXO settle window as init()

        int res = lora.begin(freq, bw, sf, cr, effectiveMainSyncword(), power, preambleLength, tcxoVoltage);
        if (res == RADIOLIB_ERR_SPI_CMD_FAILED) {
            LOG_WARN("LR20x0 band-hop begin SPI_CMD_FAILED, retrying");
            delay(100);
            res = lora.begin(freq, bw, sf, cr, effectiveMainSyncword(), power, preambleLength, tcxoVoltage);
        }
        if (TCXO_OPTIONAL_ENABLED && res != RADIOLIB_ERR_NONE && res != RADIOLIB_ERR_CHIP_NOT_FOUND && tcxoVoltage > 0) {
            LOG_WARN("LR20x0 band-hop begin TCXO failed (%s%d), retry without TCXO", radioLibErr, res);
            tcxoVoltage = 0;
            res = lora.begin(freq, bw, sf, cr, effectiveMainSyncword(), power, preambleLength, tcxoVoltage);
        }
        if (res != RADIOLIB_ERR_NONE) {
            LOG_ERROR("LR20x0 band-hop begin %s%d", radioLibErr, res);
            RECORD_CRITICALERROR(meshtastic_CriticalErrorCode_INVALID_RADIO_SETTING);
            return false;
        }

        applyCustomLfPaTable(freq);

        lr20x0LastFreqMHz = freq;

        res = lora.setCRC(2);
        if (res != RADIOLIB_ERR_NONE) {
            LOG_ERROR("LR20x0 band-hop setCRC %s%d", radioLibErr, res);
            RECORD_CRITICALERROR(meshtastic_CriticalErrorCode_INVALID_RADIO_SETTING);
            return false;
        }

#ifdef LR2021_DIO_AS_RF_SWITCH
        lora.setRfSwitchTable(lr20x0_rfswitch_dio_pins, lr20x0_rfswitch_table);
#elif ARCH_PORTDUINO
        if (portduino_config.has_rfswitch_table)
            lora.setRfSwitchTable(lr20x0_rfswitch_dio_pins, lr20x0_rfswitch_table);
#endif

        res = lora.setRxBoostedGainMode(config.lora.sx126x_rx_boosted_gain ? LR2021_RX_GAIN_BOOST_LEVEL : 0);
        if (res != RADIOLIB_ERR_NONE) {
            LOG_ERROR("LR20x0 band-hop setRxBoostedGainMode %s%d", radioLibErr, res);
            RECORD_CRITICALERROR(meshtastic_CriticalErrorCode_INVALID_RADIO_SETTING);
            return false;
        }

        // Nexus-28005 (2026-09-13): re-apply the stored side detector
        // table after a band-hop begin(). fullBegin() hardware-resets the
        // chip, wiping any previously-programmed side detectors; without
        // this call, the side SF7 detector would silently vanish every time
        // an operator crossed the LF/HF band boundary. Same source of
        // truth as the init() hook — see LR2021Interface::applyStoredSideDetectors().
        {
            LR2021Interface *lr2021 = static_cast<LR2021Interface *>(this);
            lr2021->applyStoredSideDetectors();
        }

        // begin() above reprogrammed the modulation params, so the DCDC configure state is reset here too.
        applyDcdcWorkaround();

        return true;
    }
}

// Board LF PA table after begin(); pointer is retained. HF keeps the RadioLib default.
// Warn-only: a calibration miss must not fail init/fullBegin, keep the begin() PA config.
template <typename T> void LR20x0Interface<T>::applyCustomLfPaTable(float freq)
{
#ifdef LR2021_CUSTOM_PA_TABLE
    if (isLr20x0HighBand(freq))
        return;
    lora.setPaTable(lr2021_pa_table_lf, false);
    int16_t paRes = lora.setOutputPower(power);
    if (paRes != RADIOLIB_ERR_NONE)
        LOG_WARN("LR2021 custom LF PA table setOutputPower failed (%s%d)", radioLibErr, paRes);
    else
        LOG_DEBUG("LR2021 custom LF PA table installed");
#else
    (void)freq;
#endif
}

// Semtech DCDC sensitivity workaround for sub-GHz operation on engineering sample date code 2513.
// Worthy of note is that we tested this on non-engineering samples and it didn't make any difference, but
// we went to the trouble of writing this, so it can stay in, albeit gated behind a compile-time option.
// lr20xx_workarounds_dcdc_reset must follow setPacketType; lr20xx_workarounds_dcdc_configure must follow
// setModulationParams. In init() both hold once lora.begin() returns; in reconfigure() the caller invokes this
// after setSpreadingFactor/setBandwidth/setCodingRate, whose RadioLib implementations re-run
// setLoRaModulationParams() and reset the DCDC configure state. Only applies to sub-GHz; 2.4 GHz (LORA_24) is
// excluded. Opt-in only: requires -DLR2021_DCDC_WORKAROUND (and RADIOLIB_GODMODE for the internal register access).
template <typename T> void LR20x0Interface<T>::applyDcdcWorkaround()
{
#if defined(LR2021_DCDC_WORKAROUND) && RADIOLIB_GODMODE
    if (config.lora.region == meshtastic_Config_LoRaConfig_RegionCode_LORA_24)
        return;

    // Helper: set DCDC LF frequency register and re-apply the current RF frequency.
    auto dcdcSetFreq = [&](uint32_t freqHz) -> int16_t {
        const uint32_t freqLf = (uint32_t)((float)freqHz * 1.048576f);
        int16_t s = lora.writeRegMem32(RADIOLIB_LR2021_REG_DCDC_FREQ_LF, &freqLf, 1);
        if (s != RADIOLIB_ERR_NONE)
            return s;
        uint32_t rawRfFreq = 0;
        s = lora.readRegMem32(RADIOLIB_LR2021_REG_RTTOF_RF_FREQ, &rawRfFreq, 1);
        if (s != RADIOLIB_ERR_NONE)
            return s;
        // Convert PLL steps to Hz: (steps * 15625 + 16383) / 16384
        uint32_t rfHz = (uint32_t)(((uint64_t)rawRfFreq * 15625ULL + 16383ULL) / 16384ULL);
        return lora.setRfFrequency(rfHz);
    };

    // dcdc_reset: reset RISE/FALL ramp fields to conservative 15/15 at 2.8 MHz
    int16_t dcdcRes = lora.writeRegMemMask32(RADIOLIB_LR2021_REG_DCDC_SWITCHER, 0xFu << 20, 15u << 20);
    if (dcdcRes == RADIOLIB_ERR_NONE)
        dcdcRes = lora.writeRegMemMask32(RADIOLIB_LR2021_REG_DCDC_SWITCHER, 0xFu << 16, 15u << 16);
    if (dcdcRes == RADIOLIB_ERR_NONE)
        dcdcRes = dcdcSetFreq(2800000);

    // dcdc_configure: tune RISE/FALL and DC freq based on ADC decimation and RX path.
    // Matches lr20xx_workarounds_dcdc_configure() exactly.
    if (dcdcRes == RADIOLIB_ERR_NONE) {
        uint32_t adcCtrl = 0, rxPath = 0;
        dcdcRes = lora.readRegMem32(RADIOLIB_LR2021_REG_DCDC_ADC_CTRL, &adcCtrl, 1);
        if (dcdcRes == RADIOLIB_ERR_NONE)
            dcdcRes = lora.readRegMem32(RADIOLIB_LR2021_REG_DCDC_RX_PATH, &rxPath, 1);
        if (dcdcRes == RADIOLIB_ERR_NONE) {
            const uint32_t anaDec = (adcCtrl >> 8) & 0x7;
            const bool isRxHf = (rxPath & 0x3) == 1;
            // Narrowband sub-GHz path (ana_dec 1 or 2): use tighter RISE=11/FALL=13 timing
            const uint32_t rise = (!isRxHf && (anaDec == 1 || anaDec == 2)) ? 11u : 15u;
            const uint32_t fall = (!isRxHf && (anaDec == 1 || anaDec == 2)) ? 13u : 15u;
            dcdcRes = lora.writeRegMemMask32(RADIOLIB_LR2021_REG_DCDC_SWITCHER, 0xFu << 20, rise << 20);
            if (dcdcRes == RADIOLIB_ERR_NONE)
                dcdcRes = lora.writeRegMemMask32(RADIOLIB_LR2021_REG_DCDC_SWITCHER, 0xFu << 16, fall << 16);
            if (dcdcRes == RADIOLIB_ERR_NONE)
                dcdcRes = dcdcSetFreq(anaDec == 1 ? 4300000 : 2800000);
        }
    }
    if (dcdcRes != RADIOLIB_ERR_NONE)
        LOG_WARN("LR20x0 DCDC workaround failed: %d", dcdcRes);
    else
        LOG_DEBUG("LR20x0 DCDC workaround applied");
#endif
}

template <typename T> void LR20x0Interface<T>::clearRadioIsr()
{
    lora.clearIrqAction();
}

template <typename T> int16_t LR20x0Interface<T>::trySetStandby()
{
    checkNotification(); // handle any pending interrupts before we force standby

    int16_t err = lora.standby();

    if (err != RADIOLIB_ERR_NONE) {
        LOG_DEBUG("LR20x0 standby failed, err %d", err);
    }

    isReceiving = false; // If we were receiving, not any more
    activeReceiveStart = 0;
    disableInterrupt();
    completeSending(); // If we were sending, not anymore
    RadioLibInterface::setStandby();
    return err;
}

template <typename T> void LR20x0Interface<T>::setStandby()
{
    int16_t err = trySetStandby();
    assert(err == RADIOLIB_ERR_NONE);
}

template <typename T> int16_t LR20x0Interface<T>::setOutputPowerLive(int8_t powerDb)
{
    int8_t maxPwr = (config.lora.region == meshtastic_Config_LoRaConfig_RegionCode_LORA_24) ? LR2021_MAX_POWER_HF : LR2021_MAX_POWER;
    if (powerDb < -9)
        powerDb = -9;
    if (powerDb > maxPwr)
        powerDb = maxPwr;
    int16_t err = lora.setOutputPower(powerDb);
    if (err == RADIOLIB_ERR_NONE) {
        power = powerDb;
    }
    return err;
}

template <typename T> int16_t LR20x0Interface<T>::setSpreadingFactorLive(uint8_t targetSf, int8_t targetPwr)
{
    // SetLoraModulationParams disables every side detector on LR2021, so skip it when the SF is unchanged
    // and re-arm the stored table after a real change (the apply guard parks it while sf is above a side SF).
    if (targetSf != sf) {
        int16_t err = lora.setSpreadingFactor(targetSf);
        if (err != RADIOLIB_ERR_NONE)
            return err;
        sf = targetSf;
        applyDcdcWorkaround();

        bool sideDetectorsUsable = !meshtastic::nexusModule || meshtastic::nexusModule->altModem.mode == 0;
#if NEXUS_ENABLE_RNODE
        sideDetectorsUsable = sideDetectorsUsable && (rnodeEngine.getMode() != RNODE_MODE_STANDALONE);
#endif
        if (sideDetectorsUsable)
            static_cast<LR2021Interface *>(this)->applyStoredSideDetectors();
    }
    if (targetPwr != 0x7F) {
        return setOutputPowerLive(targetPwr);
    }
    return RADIOLIB_ERR_NONE;
}

template <typename T> float LR20x0Interface<T>::getLastRxFrequencyError()
{
    return 0.0f; // no public getFrequencyError() on LR2021 in the pinned RadioLib yet
}

template <typename T> uint8_t LR20x0Interface<T>::getLastRxCodingRate()
{
    uint8_t rxCR = 0;
    bool hasCRC = true;
    float snr = 0.0f;
    float rssi = 0.0f;
    if (lora.getLoRaPacketStatus(&rxCR, &hasCRC, nullptr, &snr, &rssi, nullptr, nullptr) != RADIOLIB_ERR_NONE)
        return 0;
    return rxCR;
}

template <typename T> uint8_t LR20x0Interface<T>::getLastRxSF()
{
#if (defined(USE_LR2021) || defined(ARCH_PORTDUINO)) && RADIOLIB_EXCLUDE_LR2021 != 1
    if (lastRxDetector > 0) {
        auto *lr2021 = static_cast<LR2021Interface *>(this);
        if (lr2021 && (lastRxDetector - 1) < lr2021->sideDetectorsCount()) {
            return lr2021->sideDetectors()[lastRxDetector - 1];
        }
    }
#endif
    return sf;
}

template <typename T> uint8_t LR20x0Interface<T>::effectiveMainSyncword() const
{
    if (meshtastic::nexusModule && meshtastic::nexusModule->multiSFConfig.enabled)
        return meshtastic::nexusModule->multiSFConfig.main_syncword;
    return syncWord;
}

template <typename T> int16_t LR20x0Interface<T>::setSyncWordLive(uint8_t syncword)
{
    return lora.setSyncWord(syncword);
}

template <typename T> uint8_t LR20x0Interface<T>::getSyncWord() const
{
    return effectiveMainSyncword();
}

template <typename T> uint8_t LR20x0Interface<T>::readRxTierProtocol()
{
    if (!meshtastic::nexusModule || !meshtastic::nexusModule->multiSFConfig.enabled)
        return meshtastic::nexus::TIER_MESHTASTIC;
#if (defined(USE_LR2021) || defined(ARCH_PORTDUINO)) && RADIOLIB_EXCLUDE_LR2021 != 1
    uint8_t rxCR = 0;
    bool hasCRC = true;
    float snr = 0.0f;
    float rssi = 0.0f;
    uint8_t detector = 0;
    if (lora.getLoRaPacketStatus(&rxCR, &hasCRC, nullptr, &snr, &rssi, nullptr, &detector) != RADIOLIB_ERR_NONE)
        return meshtastic::nexus::TIER_MESHTASTIC;
    lastRxDetector = detector;
    return meshtastic::nexusModule->multiSFProtocolForDetector(detector);
#else
    return meshtastic::nexus::TIER_MESHTASTIC;
#endif
}

/**
 * Add SNR data to received messages
 */
template <typename T> void LR20x0Interface<T>::addReceiveMetadata(meshtastic_MeshPacket *mp)
{
    // Single-SPI read of LoRa packet status: gets CR, CRC, SNR, RSSI, and multi-SF detector index!
    uint8_t rxCR = 0;
    bool hasCRC = true;
    float snr = 0.0f;
    float rssi = 0.0f;
    uint8_t detector = 0;

    int16_t statErr = lora.getLoRaPacketStatus(&rxCR, &hasCRC, nullptr, &snr, &rssi, nullptr, &detector);
    if (statErr == RADIOLIB_ERR_NONE) {
        if (meshtastic::nexusModule && meshtastic::nexusModule->altModem.mode != 0) {
            mp->rx_snr = meshtastic::NexusModule::altPseudoSnr(lora.getRSSI());
        } else {
            mp->rx_snr = snr;
        }
        mp->rx_rssi = lround(rssi);
        mp->has_rx_rssi = true;
        mp->rx_coding_rate = rxCR;
        lastRxDetector = detector;
    } else {
        if (meshtastic::nexusModule && meshtastic::nexusModule->altModem.mode != 0) {
            mp->rx_snr = meshtastic::NexusModule::altPseudoSnr(lora.getRSSI());
        } else {
            mp->rx_snr = lora.getSNR();
        }
        mp->rx_rssi = lround(lora.getRSSI());
        mp->has_rx_rssi = true;
        lastRxDetector = 0;
    }

    // TODO (Nexus-28005): re-populate rx_frequency_error once RadioLib exposes
    // getFrequencyError() on the LR2021 module. SX126xInterface.cpp:410 and
    // SX128xInterface.cpp:262 both call lora.getFrequencyError() today; that call
    // returns 0 (no-op) on LR20x0. The Semtech LR2021 chip exposes the modem-side
    // frequency-error correction register (RADIOLIB_LR2021_REG_LORA_EXT_FREQ_ERR_CTRL
    // at LR2021_registers.h:17), and a future RadioLib release will surface it as a
    // getter. Until then LR2021 reports rx_frequency_error == 0 here, mirroring the
    // pre-Nexus-28005 behavior across the other LR20x0 derivatives.
}

/** We override to turn on transmitter power as needed.
 */
template <typename T> void LR20x0Interface<T>::configHardwareForSend()
{
#if NEXUS_ENABLE_RNODE
    if (getRawSendingType() == RAW_SEND_RNODE) {
        const uint8_t sw = getRawTxSync() ? getRawTxSync() : 0x12;
        const uint8_t sfTarget = getRawTxSf() ? getRawTxSf() : rnodeEngine.getSf();
        lora.setSyncWord(sw);
        if (sfTarget != this->sf)
            lora.setSpreadingFactor(sfTarget);
    }
#endif
#if NEXUS_ENABLE_JENCORE_SNIFFER
    if (getRawSendingType() == RAW_SEND_JENCORE) {
        if (getRawTxSync())
            lora.setSyncWord(getRawTxSync());
        if (getRawTxSf() && getRawTxSf() != this->sf)
            lora.setSpreadingFactor(getRawTxSf());
    }
#endif
    RadioLibInterface::configHardwareForSend();
}

// For power draw measurements, helpful to force radio to stay sleeping
// #define SLEEP_ONLY

template <typename T> void LR20x0Interface<T>::startReceive()
{
#ifdef SLEEP_ONLY
    sleep();
#else

    int16_t err = trySetStandby();

    if (err == RADIOLIB_ERR_NONE) {
        if (!meshtastic::nexusModule || meshtastic::nexusModule->altModem.mode == 0) {
            lora.setPreambleLength(preambleLength); // Solve RX ack fail after direct message sent.  Not sure why this is needed.
#if NEXUS_ENABLE_RNODE
            if (rnodeEngine.getMode() == RNODE_MODE_STANDALONE) {
                lora.setSyncWord(0x12);
                lora.setSpreadingFactor(rnodeEngine.getSf());
                rnodeRadioDirty = false;
                rnodeWasStandalone = true;
            } else if (rnodeRadioDirty || rnodeWasStandalone) {
                lora.setSyncWord(effectiveMainSyncword());
                lora.setSpreadingFactor(this->sf);
                rnodeRadioDirty = false;
                rnodeWasStandalone = false;
                // SetLoraModulationParams above disabled every side detector; re-arm the stored table now
                // that the main SF is back (the apply guard parks it while sf is above a side SF).
                static_cast<LR2021Interface *>(this)->applyStoredSideDetectors();
            }
#elif NEXUS_ENABLE_JENCORE_SNIFFER
            if (rnodeRadioDirty) {
                lora.setSyncWord(effectiveMainSyncword());
                lora.setSpreadingFactor(this->sf);
                rnodeRadioDirty = false;
                static_cast<LR2021Interface *>(this)->applyStoredSideDetectors();
            }
#endif
        } else {
            (void)lora.variablePacketLengthMode(maxFrameLen());
        }

        // We use a 16 bit preamble so this should save some power by letting radio sit in standby mostly.
        err =
            lora.startReceive(RADIOLIB_LR2021_RX_TIMEOUT_INF, MESHTASTIC_RADIOLIB_IRQ_RX_FLAGS, RADIOLIB_IRQ_RX_DEFAULT_MASK, 0);
    }

    if (err != RADIOLIB_ERR_NONE) {
        LOG_ERROR("StartReceive error: %d", err);
        if (maybeRecoverChipStateLoss()) {
            if (!meshtastic::nexusModule || meshtastic::nexusModule->altModem.mode == 0) {
                lora.setPreambleLength(preambleLength);
            } else {
                (void)lora.variablePacketLengthMode(maxFrameLen());
            }
            err = lora.startReceive(RADIOLIB_LR2021_RX_TIMEOUT_INF, MESHTASTIC_RADIOLIB_IRQ_RX_FLAGS,
                                    RADIOLIB_IRQ_RX_DEFAULT_MASK, 0);
        }
    }

    if (err != RADIOLIB_ERR_NONE) {
        // No assert: leave RX off rather than reboot; periodicRadioMaintenance() re-arms it, throttled
        LOG_ERROR("LR20x0 RX offline %s%d", radioLibErr, err);
        rxOffline = true;
        return;
    }

    RadioLibInterface::startReceive();

    // Must be done AFTER starting receive, because startReceive clears (possibly stale) interrupt pending register bits
    enableInterrupt(isrRxLevel0);
    checkRxDoneIrqFlag();
#endif
}

/** Is the channel currently active? */
template <typename T> bool LR20x0Interface<T>::isChannelActive()
{
    // check if we can detect a LoRa preamble on the current channel
    ChannelScanConfig_t cfg = {.cad = {.symNum = NUM_SYM_CAD,
                                       .detPeak = RADIOLIB_LR2021_CAD_PARAM_DEFAULT,
                                       .detMin = RADIOLIB_LR2021_CAD_PARAM_DEFAULT,
                                       .exitMode = RADIOLIB_LR2021_CAD_PARAM_DEFAULT,
                                       .timeout = 0,
                                       .irqFlags = RADIOLIB_IRQ_CAD_DEFAULT_FLAGS,
                                       .irqMask = RADIOLIB_IRQ_CAD_DEFAULT_MASK}};
    // Nexus SETFSK/SETFLRC: no LoRa CAD in GFSK/FLRC — dynamic RSSI energy sniff:
    // busy = 10 dB over the larger of the mode's sensitivity and the noise floor.
    // MUST sample instantaneous RSSI while still in continuous RX mode: entering standby
    // drops getRSSI to the standby floor and makes channel activity detection blind.
    if (meshtastic::nexusModule && meshtastic::nexusModule->altModem.mode != 0) {
        float rssi = lora.getRSSI(false, true); // instantaneous, not last-packet status
        float threshold = max(altSensitivityDbm(), (float)getNoiseFloor()) + 10.0f;
        LOG_TRACE("LR2021 alt-modem channel sniff RSSI=%.1f dBm (busy above %.1f)", rssi, threshold);
        if (rssi > threshold)
            return true;
        int16_t result = trySetStandby();
        if (result != RADIOLIB_ERR_NONE) {
            maybeRecoverChipStateLoss();
        }
        return false;
    }

    int16_t result = trySetStandby();
    if (result == RADIOLIB_ERR_NONE) {
        result = lora.scanChannel(cfg);
        if (result == RADIOLIB_LORA_DETECTED)
            return true;
        if (result != RADIOLIB_ERR_WRONG_MODEM)
            return false;
    }

    // standby failed or the LoRa modem type is gone - the chip lost its runtime state
    maybeRecoverChipStateLoss();
    return false; // report the channel free: a recovered chip can TX, a dead one fails startSend safely
}

/** Could we send right now (i.e. either not actively receiving or transmitting)? */
template <typename T> bool LR20x0Interface<T>::isActivelyReceiving()
{
    // The IRQ status will be cleared when we start our read operation. Check if we've started a header, but haven't yet
    // received and handled the interrupt for reading the packet/handling errors.
    // Nexus SETFSK/SETFLRC: the shared header/sync bit also covers alt-modem packet start,
    // so the LoRa flags stay valid there — but the stale-latch hazard (bench 2026-10-17,
    // same as SX126x FSK) applies: clear false-detection latches in alt mode.
    if (meshtastic::nexusModule && meshtastic::nexusModule->altModem.mode != 0) {
        const uint16_t irq = lora.getIrqStatus();
        const uint16_t latch = RADIOLIB_LR2021_IRQ_LORA_HEADER_VALID | RADIOLIB_LR2021_IRQ_PREAMBLE_DETECTED;
        const bool detected = receiveDetected(irq, RADIOLIB_LR2021_IRQ_LORA_HEADER_VALID,
                                              RADIOLIB_LR2021_IRQ_PREAMBLE_DETECTED);
        if (!detected && (irq & latch)) {
            lora.clearIrqFlags(latch);
            LOG_TRACE("LR2021 alt-modem stale RX latch cleared (irq=0x%04X)", (unsigned)irq);
        }
        return detected;
    }
    return receiveDetected(lora.getIrqStatus(), RADIOLIB_LR2021_IRQ_LORA_HEADER_VALID, RADIOLIB_LR2021_IRQ_PREAMBLE_DETECTED);
}

#ifdef LR20X0_AGC_RESET
template <typename T> void LR20x0Interface<T>::resetAGC()
{
    // Safety: don't reset mid-packet
    if (isSending() || (isReceiving && isActivelyReceiving()))
        return;

    LOG_DEBUG("LR20x0 AGC reset: warm sleep + Calibrate(0x3F)");

    // 1. Warm sleep - powers down the analog frontend, resetting AGC state
    lora.sleep(true, 0);

    // 2. Wake to RC standby for stable calibration
    lora.standby(RADIOLIB_LR20X0_STANDBY_RC, true);

    // 3. Calibrate all blocks (PLL, ADC, image, RC oscillators)
    //    calibrate() is protected on LR20x0, so use raw SPI (same as internal implementation)
    uint8_t calData = RADIOLIB_LR20X0_CALIBRATE_ALL;
    module.SPIwriteStream(RADIOLIB_LR20X0_CMD_CALIBRATE, &calData, 1, true, true);

    // 4. Re-calibrate image rejection for actual operating frequency
    //    Calibrate(0x3F) defaults to 902-928 MHz which is wrong for other regions.
    lora.calibrateImageRejection(getFreq() - 4.0f, getFreq() + 4.0f);

    // 5. Re-apply RX boosted gain mode
    lora.setRxBoostedGainMode(config.lora.sx126x_rx_boosted_gain ? LR2021_RX_GAIN_BOOST_LEVEL : 0);

    // 6. Resume receiving
    startReceive();
}
#endif

template <typename T> uint8_t LR20x0Interface<T>::readChipState()
{
    // Busy pin first: same wedged-chip protection as the SX126x path.
    if (module.getGpio() != RADIOLIB_NC && module.hal->digitalRead(module.getGpio()))
        return CHIP_BUSY;

    // Same LR-status caveat as LR11x0Interface::readChipState - no decoded mode bits.
    if (isReceiving)
        return CHIP_RX;
    return CHIP_UNK;
}

template <typename T> bool LR20x0Interface<T>::sleep()
{
    // \todo Display actual typename of the adapter, not just `LR20x0`
    LOG_DEBUG("LR20x0 entering sleep mode");
    (void)trySetStandby(); // Stop any pending operations - the chip is being put to sleep, a failure must not crash

    // turn off TCXO if it was powered
    lora.setTCXO(0);

    // put chipset into sleep mode (we've already disabled interrupts by now)
    bool keepConfig = false;
    lora.sleep(keepConfig, 0); // Note: we do not keep the config, full reinit will be needed

#ifdef LR2021_POWER_EN
    digitalWrite(LR2021_POWER_EN, LOW);
#endif

    return true;
}

template <typename T> int16_t LR20x0Interface<T>::getCurrentRSSI()
{
    float rssi = lora.getRSSI(false, true);
    return (int16_t)round(rssi);
}

/// Approximate LR2021 sensitivity (Semtech datasheet) of the active GFSK/FLRC setting
template <typename T> float LR20x0Interface<T>::altSensitivityDbm()
{
    const auto &m = meshtastic::nexusModule->altModem;
    if (m.mode == 2) {
        float s = m.brKbps >= 1300 ? -99.0f : m.brKbps >= 1000 ? -100.0f : m.brKbps >= 520 ? -104.0f : -106.0f;
        return s + (m.cr == 3 ? 2.0f : m.cr == 4 ? 3.0f : 0.0f);
    }
    if (getFreq() > 1000.0f) {
        return m.brKbps >= 2000 ? -85.0f : m.brKbps >= 1000 ? -89.0f : m.brKbps >= 500 ? -92.0f : -95.0f;
    }
    return m.brKbps >= 250 ? -99.0f : m.brKbps >= 150 ? -103.0f : m.brKbps >= 50 ? -108.0f : m.brKbps >= 10 ? -114.0f : -118.0f;
}

/*
 * FLRC framing: byte 0 = 0x00 for an unfragmented packet, otherwise
 * 1 | index(2 bits) | count-1 (2 bits) | tag (3 bits), followed by an equal share of the packet so no
 * fragment falls under FLRC's 6-byte minimum.
 */
template <typename T> int16_t LR20x0Interface<T>::startTransmitFrame(size_t numbytes)
{
    if (!usesFragments())
        return RadioLibInterface::startTransmitFrame(numbytes);

    txFragLen = numbytes;
    txFragCount = fragmentCount(numbytes);
    txFragNext = 0;
    txFragTag = (txFragTag + 1) & 0x07;
    return transmitFragment();
}

template <typename T> int16_t LR20x0Interface<T>::transmitFragment()
{
    size_t per = (txFragLen + txFragCount - 1) / txFragCount;
    size_t off = txFragNext * per;
    size_t len = min(per, txFragLen - off);

    frameBuf[0] = txFragCount == 1 ? 0 : (0x80 | (txFragNext << 5) | ((txFragCount - 1) << 3) | txFragTag);
    memcpy(frameBuf + 1, ((uint8_t *)&radioBuffer) + off, len);
    txFragNext++;
    return lora.startTransmit(frameBuf, len + 1);
}

template <typename T> uint32_t LR20x0Interface<T>::nextFrameDelayMsec()
{
    return (usesFragments() && txFragNext < txFragCount) ? FRAG_GAP_MSEC : 0;
}

template <typename T> bool LR20x0Interface<T>::sendNextFrame()
{
    if (!usesFragments() || txFragNext >= txFragCount)
        return false;

    int16_t res = transmitFragment();
    if (res != RADIOLIB_ERR_NONE) {
        LOG_ERROR("FLRC fragment %u/%u failed, error=%d", txFragNext, txFragCount, res);
        txFragNext = txFragCount;
        return false;
    }
    return true;
}

template <typename T> int LR20x0Interface<T>::readFrame(size_t &length)
{
    if (!usesFragments())
        return RadioLibInterface::readFrame(length);

    if (length < 2 || length > FLRC_MAX_FRAME)
        return RADIOLIB_ERR_PACKET_TOO_LONG;
    int state = lora.readData(frameBuf, length);
    if (state != RADIOLIB_ERR_NONE)
        return state;

    uint8_t hdr = frameBuf[0];
    size_t len = length - 1;
    if (!(hdr & 0x80)) {
        memcpy(&radioBuffer, frameBuf + 1, len);
        length = len;
        return RADIOLIB_ERR_NONE;
    }

    uint8_t idx = (hdr >> 5) & 0x03, count = ((hdr >> 3) & 0x03) + 1, tag = hdr & 0x07;
    if (idx == 0) {
        rxFragTag = tag;
        rxFragCount = count;
        rxFragNext = 0;
        rxFragLen = 0;
    }
    if (idx != rxFragNext || tag != rxFragTag || count != rxFragCount || rxFragLen + len > sizeof(rxFrag)) {
        LOG_DEBUG("Drop FLRC fragment %u/%u", idx, count);
        rxFragNext = 0xFF;
        return RX_FRAME_PARTIAL;
    }
    memcpy(rxFrag + rxFragLen, frameBuf + 1, len);
    rxFragLen += len;
    if (++rxFragNext < rxFragCount)
        return RX_FRAME_PARTIAL;

    memcpy(&radioBuffer, rxFrag, rxFragLen);
    length = rxFragLen;
    rxFragNext = 0xFF;
    return RADIOLIB_ERR_NONE;
}

/// FLRC time on air from the datasheet, per fragment plus the gaps between fragments
template <typename T> uint32_t LR20x0Interface<T>::flrcPacketTimeMsec(uint32_t pl)
{
    const auto &m = meshtastic::nexusModule->altModem;
    uint32_t crNum = m.cr == 2 ? 1 : (m.cr == 3 ? 3 : 1), crDen = m.cr == 2 ? 2 : (m.cr == 3 ? 4 : 1);
    uint8_t count = fragmentCount(pl);
    uint32_t per = (pl + count - 1) / count;
    uint32_t us = 0;
    for (uint8_t i = 0; i < count; i++) {
        uint32_t len = min(per, pl - i * per) + 1;
        uint32_t uncoded = max(m.preambleBits, FLRC_MIN_PREAMBLE_BITS) + 21 + 32 + 16;
        uint32_t raw = (len + 2) * 8 + (crNum == crDen ? 0 : 6);
        us += (uint32_t)((uncoded + raw * crDen / crNum) * 1000 / m.brKbps);
    }
    us += (count - 1) * FRAG_GAP_MSEC * 1000;
    return max((uint32_t)1, (us + 999) / 1000);
}

// Don't leak the alias into the files InterfacesTemplates.cpp includes after this one.
#undef LR20x0
#endif
