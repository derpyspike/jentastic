#pragma once
#include "RadioLib.h"

// NiceRF LR2021 internal RF switch, DIO5..DIO8.
//
// Wiring confirmed by the user:
//   DIO5 = RXEN 2.4 GHz
//   DIO6 = TXEN 868 MHz
//   DIO7 = active for both RX and TX in 2.4 GHz
//   DIO8 = TXEN 2.4 GHz
//
// This is NOT the LR1121 table. The LR1121 uses DIO5 for RX and DIO8 for
// sub-GHz TX; the LR2021 uses the mapping below.
//
// The LR20x0Interface expects the lr20x0_* symbol names.
static const uint32_t rfswitch_dio_pins[Module::RFSWITCH_MAX_PINS] = {
    RADIOLIB_LR2021_DIO5,
    RADIOLIB_LR2021_DIO6,
    RADIOLIB_LR2021_DIO7,
    RADIOLIB_LR2021_DIO8,
    RADIOLIB_NC
};

static const Module::RfSwitchMode_t rfswitch_table[] = {
    // clang-format off
    // mode                  DIO5  DIO6  DIO7  DIO8
    //                       2.4RX 868TX 2.4EN 2.4TX
    {LR2021::MODE_STBY,  {LOW,  LOW,  LOW,  LOW}},
    {LR2021::MODE_RX,    {LOW,  LOW,  LOW,  LOW}},  // sub-GHz RX
    {LR2021::MODE_TX,    {LOW,  HIGH, LOW,  LOW}},  // 868 MHz TX
    {LR2021::MODE_RX_HF, {HIGH, LOW,  HIGH, LOW}},  // 2.4 GHz RX: DIO5+DIO7
    {LR2021::MODE_TX_HF, {LOW,  LOW,  HIGH, HIGH}}, // 2.4 GHz TX: DIO7+DIO8
    END_OF_MODE_TABLE,
    // clang-format on
};

#define lr20x0_rfswitch_dio_pins rfswitch_dio_pins
#define lr20x0_rfswitch_table rfswitch_table
