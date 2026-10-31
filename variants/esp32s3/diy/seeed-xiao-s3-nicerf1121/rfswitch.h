#pragma once
#include "RadioLib.h"

// NiceRF LoRa1121F33-2G4. Taken from the vendor demo (rf_switch_setup_F33_2G4 in
// LoRa1121F33_demo_code_V1.1/Core/Src/lr1121.c). RFSW0..RFSW3 are DIO5, DIO6, DIO7, DIO8:
//   sub-GHz TX   -> RFSW3 (DIO8)
//   2.4 GHz TX   -> RFSW1 | RFSW2 (DIO6 + DIO7)
//   RX / standby -> all low
//   RXEN         -> RFSW0 (DIO5)
// The module has no low-power route, so the build also forces the high-power PA.
static const uint32_t rfswitch_dio_pins[] = {RADIOLIB_LR11X0_DIO5, RADIOLIB_LR11X0_DIO6, RADIOLIB_LR11X0_DIO7,
                                             RADIOLIB_LR11X0_DIO8, RADIOLIB_NC};

static const Module::RfSwitchMode_t rfswitch_table[] = {
    // clang-format off
    // mode              DIO5  DIO6  DIO7  DIO8
    {LR11x0::MODE_STBY,  {LOW,  LOW,  LOW,  LOW}},
    {LR11x0::MODE_RX,    {HIGH, LOW,  LOW,  LOW}},
    {LR11x0::MODE_TX,    {LOW,  LOW,  LOW,  HIGH}},
    {LR11x0::MODE_TX_HP, {LOW,  LOW,  LOW,  HIGH}},
    {LR11x0::MODE_TX_HF, {LOW,  HIGH, HIGH, LOW}},
    {LR11x0::MODE_GNSS,  {LOW,  LOW,  LOW,  LOW}},
    {LR11x0::MODE_WIFI,  {HIGH, LOW,  LOW,  LOW}},
    END_OF_MODE_TABLE,
    // clang-format on
};
