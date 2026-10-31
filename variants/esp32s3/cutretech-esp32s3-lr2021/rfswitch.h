#pragma once
#include "RadioLib.h"

// NiceRF LR2021 RF switch on DIO5..DIO8. DIO5 is also the module's IRQ line,
// so it stays low in every switch state.
//
//   mode                DIO5  DIO6  DIO7  DIO8
//   STBY                LOW   LOW   LOW   LOW
//   sub-GHz RX          LOW   LOW   LOW   LOW
//   sub-GHz TX          LOW   HIGH  LOW   LOW
//   2.4 GHz TX (FEM)    LOW   LOW   HIGH  HIGH
static const uint32_t lr20x0_rfswitch_dio_pins[Module::RFSWITCH_MAX_PINS] = {
    RADIOLIB_LR2021_DIO5, RADIOLIB_LR2021_DIO6, RADIOLIB_LR2021_DIO7, RADIOLIB_LR2021_DIO8, RADIOLIB_NC};

static const Module::RfSwitchMode_t lr20x0_rfswitch_table[] = {
    {LR2021::MODE_STBY, {LOW, LOW, LOW, LOW}},
    {LR2021::MODE_RX, {LOW, LOW, LOW, LOW}},
    {LR2021::MODE_TX, {LOW, HIGH, LOW, LOW}},
    {LR2021::MODE_TX_HF, {LOW, LOW, HIGH, HIGH}},
    END_OF_MODE_TABLE,
};
