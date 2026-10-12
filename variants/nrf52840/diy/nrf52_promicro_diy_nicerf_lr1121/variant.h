#ifndef _VARIANT_PROMICRO_DIY_NICERF_LR1121_
#define _VARIANT_PROMICRO_DIY_NICERF_LR1121_

/** Master clock frequency */
#define VARIANT_MCK (64000000ul)

#define USE_LFRC // Board uses RC for LF

#define PROMICRO_DIY_NICERF_LR1121

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "WVariant.h"

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/*
NRF52 PRO MICRO PIN ASSIGNMENT - NiceRF LR1121 Edition

Based on promicro_DIY base with modifications for NiceRF LR1121 module pinout.
*/

// Number of pins defined in PinDescription array
#define PINS_COUNT (48)
#define NUM_DIGITAL_PINS (48)
#define NUM_ANALOG_INPUTS (1)
#define NUM_ANALOG_OUTPUTS (0)

// Pin 13 enables 3.3V periphery.
#define PIN_3V3_EN (0 + 13) // P0.13

// Analog pins
#define BATTERY_PIN (0 + 31) // P0.31 Battery ADC
#define ADC_CHANNEL ADC1_GPIO4_CHANNEL
#define ADC_RESOLUTION 14
#define BATTERY_SENSE_RESOLUTION_BITS 12
#define BATTERY_SENSE_RESOLUTION 4096.0
#define VBAT_MV_PER_LSB (0.73242188F)
#define VBAT_DIVIDER (0.6F)
#define VBAT_DIVIDER_COMP (1.73)
#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)
#undef AREF_VOLTAGE
#define AREF_VOLTAGE 3.0
#define VBAT_AR_INTERNAL AR_INTERNAL_3_0
#define ADC_MULTIPLIER VBAT_DIVIDER_COMP 
#define VBAT_RAW_TO_SCALED(x) (REAL_VBAT_MV_PER_LSB * x)

// WIRE IC AND IIC PINS
#define WIRE_INTERFACES_COUNT 1
#define PIN_WIRE_SDA (32 + 4) // P1.04
#define PIN_WIRE_SCL (0 + 11) // P0.11

// LED
#define PIN_LED1 (0 + 15) // P0.15
#define LED_BLUE PIN_LED1
#define LED_STATE_ON 1 // State when LED is lit

// Button
#define BUTTON_PIN (32 + 0) // P1.00

// GPS
#define GPS_TX_PIN (0 + 20) // P0.20
#define GPS_RX_PIN (0 + 22) // P0.22
#define PIN_GPS_EN (0 + 24) // P0.24
#define GPS_UBLOX

// UART interfaces
#define PIN_SERIAL1_TX GPS_TX_PIN
#define PIN_SERIAL1_RX GPS_RX_PIN
#define PIN_SERIAL2_RX (0 + 6) // P0.06
#define PIN_SERIAL2_TX (0 + 8) // P0.08

// SPI interfaces
#define SPI_INTERFACES_COUNT 1
#define PIN_SPI_MISO (0 + 2)   // P0.02
#define PIN_SPI_MOSI (32 + 15) // P1.15
#define PIN_SPI_SCK (32 + 11)  // P1.11

#define LORA_MISO PIN_SPI_MISO
#define LORA_MOSI PIN_SPI_MOSI
#define LORA_SCK PIN_SPI_SCK
#define LORA_CS (32 + 13) // P1.13

// Common LoRa pins for compatibility with other modules (though only LR1121 is used)
#define LORA_RESET (0 + 9) // P0.09 NRST
#define LORA_BUSY (0 + 29) // P0.29 BUSY
#define LORA_DIO1 (0 + 10) // P0.10 IRQ

// LORA MODULES - NiceRF LR1121 and NiceRF LR2021 share the same Pro-Micro
// pinout / footprint; only the LoRa IC differs. Both are enabled below and
// both consume the same RF switch table defined in rfswitch.h.
#define USE_LR1121
#define USE_LR2021

// LR1121 CONFIG
#ifdef USE_LR1121
#define LR1121_IRQ_PIN LORA_DIO1
#define LR1121_NRESET_PIN LORA_RESET
#define LR1121_BUSY_PIN LORA_BUSY
#define LR1121_SPI_NSS_PIN LORA_CS
#define LR1121_SPI_SCK_PIN LORA_SCK
#define LR1121_SPI_MOSI_PIN LORA_MOSI
#define LR1121_SPI_MISO_PIN LORA_MISO

// RF Switch / Antenna Switching for NiceRF LR1121
// DIO5..DIO8 are wired to the module's internal RF switch.
#define LR11X0_DIO_AS_RF_SWITCH
#endif

// LR2021 CONFIG
#ifdef USE_LR2021
#define LR2021_IRQ_PIN LORA_DIO1
#define LR2021_NRESET_PIN LORA_RESET
#define LR2021_BUSY_PIN LORA_BUSY
#define LR2021_SPI_NSS_PIN LORA_CS
#define LR2021_SPI_SCK_PIN LORA_SCK
#define LR2021_SPI_MOSI_PIN LORA_MOSI
#define LR2021_SPI_MISO_PIN LORA_MISO

// RF Switch / Antenna Switching for NiceRF LR2021
// DIO5..DIO8 are wired to the module's internal RF switch (same DIO mapping
// as the LR1121 — both modules share the Pro-Micro pinout).
#define LR2021_DIO_AS_RF_SWITCH
#endif

// Shared TCXO voltage rail (NiceRF LR1121 and LR2021 modules both run a
// 3.3 V TCXO on DIO3). The driver-side symbols differ by chip so we set
// both; only the one matching the active driver is consulted.
#define LR11X0_DIO3_TCXO_VOLTAGE 3.3
#define LR2021_DIO3_TCXO_VOLTAGE 3.3

#ifdef __cplusplus
}
#endif

#endif
