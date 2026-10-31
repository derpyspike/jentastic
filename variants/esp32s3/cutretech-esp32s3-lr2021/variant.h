#ifndef _VARIANT_CUTRETECH_ESP32S3_LR2021_
#define _VARIANT_CUTRETECH_ESP32S3_LR2021_

// CutreTech ESP32-S3 carrier with a NiceRF LR2021 module.
// Same physical pinout as cutretech-esp32s3; only the radio chip changes from
// SX1262/SX1268 to LR2021.

#define HAS_SCREEN 0
#define HAS_GPS 0

// UART0 not routed to the header: serial is USB CDC. D6/D7 are I2C/radio.
#define I2C_SDA 43 // D6
#define I2C_SCL 44 // D7

#define LED_POWER 21
#define LED_STATE_ON 0 // Active LOW on Seeed XIAO ESP32-S3

#define BATTERY_PIN 1
#define ADC_MULTIPLIER 2.0 // Adjust in configuration if needed, in my case ~2.025
#define ADC_CHANNEL ADC_CHANNEL_0
#define BATTERY_SENSE_RESOLUTION_BITS 12

#define USE_LR2021

// SPI pins for the LoRa module (same cutretech wiring).
#define LORA_SCK 7
#define LORA_MOSI 9
#define LORA_MISO 8
#define LORA_CS 5

// LR2021 radio control pins.
#define LORA_DIO1 2 // IRQ
#define LORA_RESET 3
#define LORA_BUSY 4

#define LR2021_IRQ_PIN LORA_DIO1
#define LR2021_NRESET_PIN LORA_RESET
#define LR2021_BUSY_PIN LORA_BUSY
#define LR2021_SPI_NSS_PIN LORA_CS
#define LR2021_SPI_SCK_PIN LORA_SCK
#define LR2021_SPI_MOSI_PIN LORA_MOSI
#define LR2021_SPI_MISO_PIN LORA_MISO

// NiceRF LR2021: IRQ on DIO5 and the module's RF switch on DIO5..DIO8.
#define LR2021_IRQ_DIO_NUM 5
#define LR2021_DIO_AS_RF_SWITCH
#define LR2021_DIO3_TCXO_VOLTAGE 3.0

#endif
