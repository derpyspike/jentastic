#pragma once
#include "RadioLib.h"

// NiceRF LR1121 / LR2021 RF switch matrix (shared footprint, shared DIO mapping).
//
// The NiceRF Pro-Micro module exposes the radio's internal RF switch on
// DIO5..DIO8 (no external ANT SW GPIO; the LF/HF front-end select is done
// by the chip itself via MODE_TX_HF). Brad's reference for the LR2021 30 dB
// variant (NiceRF example code, TCXO 3.3 V) is the source of truth:
//
//   mode                DIO5  DIO6  DIO7  DIO8
//   STBY                LOW   LOW   LOW   LOW
//   sub-GHz RX (passive) LOW   LOW   LOW   LOW
//   sub-GHz TX (PA key) LOW   HIGH  LOW   LOW
//   2.4 GHz TX (FEM)    LOW   LOW   HIGH  HIGH
//
// Both LR11x0Interface (USE_LR1121) and LR20x0Interface (USE_LR2021) consume
// `rfswitch_dio_pins[]` / `rfswitch_table[]` via RadioLib's
// `setRfSwitchTable()`; `findRfSwitchMode()` matches by raw mode value, so a
// single table keyed on `Module::MODE_*` constants works for both drivers:
//   - LR11x0::MODE_TX_HF == LR2021::MODE_TX_HF (both numerically equal
//     Module::MODE_TX + 2), so the TX_HF row drives 2.4 GHz TX in both chips.
//   - The LR1121-only TX_HP / GNSS / WIFI rows are not needed for LoRa
//     operation and are dropped.
//   - The LR2021-only RX_HF row is identical to STBY (DIO5..DIO8 = LOW per
//     Brad) and is folded into MODE_IDLE.

static const uint32_t rfswitch_dio_pins[Module::RFSWITCH_MAX_PINS] = {
    RADIOLIB_LR11X0_DIO5,  // index 0 -> DIO5
    RADIOLIB_LR11X0_DIO6,  // index 1 -> DIO6
    RADIOLIB_LR11X0_DIO7,  // index 2 -> DIO7
    RADIOLIB_LR11X0_DIO8,  // index 3 -> DIO8
    RADIOLIB_NC            // index 4 -> unused
};

// Mode constants: both LR11x0 and LR2021 drivers consume this table via RadioLib's
// findRfSwitchMode(), which matches by raw uint8 value. The HF modes are
// defined by each driver's enum with identical numeric values:
//   LR11x0::MODE_TX_HF = Module::MODE_TX + 2 (and no MODE_RX_HF row needed;
//       RX_HF is omitted — both chips route HF RX through their internal
//       front-end and do not require DIO table entry).
//   LR2021::MODE_TX_HF = Module::MODE_TX + 2 (same numeric value).
// We use LR11x0::MODE_* constants here so the table compiles cleanly under
// USE_LR1121 (which is always defined for this variant); LR11x0Interface.cpp
// and LR20x0Interface.cpp both produce identical lookup keys because the
// numerical values coincide across drivers.
static const Module::RfSwitchMode_t rfswitch_table[] = {
    // clang-format off
    // mode                       DIO5  DIO6  DIO7  DIO8
    {LR11x0::MODE_STBY,   {LOW,  LOW,  LOW,  LOW}},  // standby
    {LR11x0::MODE_RX,     {LOW,  LOW,  LOW,  LOW}},  // sub-GHz RX (passive on the F33)
    {LR11x0::MODE_TX,     {LOW,  HIGH, LOW,  LOW}},  // sub-GHz TX (DIO6 = PA key)
    {LR11x0::MODE_TX_HF,  {LOW,  LOW,  HIGH, HIGH}}, // 2.4 GHz TX (DIO7+DIO8 = FEM active)
    END_OF_MODE_TABLE,
    // clang-format on
};

// LR2021 driver (LR20x0Interface.cpp) reads its switch table from symbols
// named `lr20x0_rfswitch_dio_pins[]` / `lr20x0_rfswitch_table[]`. Alias them
// to the shared table so the LR2021 build uses exactly the same data as the
// LR1121 build (no duplication, no drift).
#ifdef USE_LR2021
#define lr20x0_rfswitch_dio_pins rfswitch_dio_pins
#define lr20x0_rfswitch_table rfswitch_table
#endif
