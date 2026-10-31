#ifndef _VARIANT_SEEED_XIAO_S3_NICERF1121_
#define _VARIANT_SEEED_XIAO_S3_NICERF1121_

// Seeed XIAO ESP32-S3 + NiceRF LoRa1121F33-2G4 (LR1121 with a 30 dBm FEM).
// Derived from variants/esp32s3/seeed_xiao_s3: no GPS, no screen, and every radio line moved onto
// the castellated pads (the stock variant uses GPIO38..42, which only exist on the back connector).

#define LED_POWER 48
#define LED_STATE_ON 1 // State when LED is lit

#define BUTTON_PIN 21 // This is the Program Button
#define BUTTON_NEED_PULLUP

// No BATTERY_PIN on purpose. PowerFSM treats a board with a battery sense as running on battery
// and light-sleeps whenever no USB host is present, which takes WiFi down with it. Without the
// define the node always behaves as externally powered. D10 is therefore free.
#define BATTERY_SENSE_RESOLUTION_BITS 12

// I2C on the two remaining free pads. D4 is the battery ADC, so Wire must not default to GPIO5.
#define I2C_SDA 7  // D8
#define I2C_SCL 8  // D9

#define USE_LR1121

// SPI keeps SCK on the XIAO's default bus pin; the rest is routed through the GPIO matrix.
#define LORA_SCK 44 // D7
#define LORA_MISO 4 // D3
#define LORA_MOSI 5  // D4

#define LORA_CS 43   // D6
#define LORA_RESET 2 // D1
#define LORA_IRQ 1   // D0
#define LORA_BUSY 6  // D5

#define LR1121_IRQ_PIN LORA_IRQ
#define LR1121_NRESET_PIN LORA_RESET
#define LR1121_BUSY_PIN LORA_BUSY
#define LR1121_SPI_NSS_PIN LORA_CS
#define LR1121_SPI_SCK_PIN LORA_SCK
#define LR1121_SPI_MOSI_PIN LORA_MOSI
#define LR1121_SPI_MISO_PIN LORA_MISO

// The vendor demo configures this module's TCXO at 3.0 V; 1.8 V underdrives the reference and the
// synthesiser will not settle at 2.4 GHz.
#define LR11X0_DIO3_TCXO_VOLTAGE 3.0
#define LR11X0_DIO_AS_RF_SWITCH

// Single high-power path through the FEM, so the low-power PA is never the right choice.
#define LR11X0_FORCE_HP_PA

#endif
