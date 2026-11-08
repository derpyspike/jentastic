#ifndef _VARIANT_SEEED_XIAO_S3_NICERF_LR2021_
#define _VARIANT_SEEED_XIAO_S3_NICERF_LR2021_

// Seeed XIAO ESP32-S3 + NiceRF LR2021.
// Same castellated wiring as the LR1121 variant; only the radio driver and
// the LR2021 internal RF-switch DIO table change.

#define LED_POWER 48
#define LED_STATE_ON 1

#define BUTTON_PIN 21
#define BUTTON_NEED_PULLUP

#define BATTERY_SENSE_RESOLUTION_BITS 12

#define I2C_SDA 7
#define I2C_SCL 8

#define USE_LR2021

// Same SPI/control pads as the XIAO LR1121 variant.
#define LORA_SCK 44   // D7
#define LORA_MISO 4   // D3
#define LORA_MOSI 5   // D4
#define LORA_CS 43    // D6
#define LORA_RESET 2  // D1
#define LORA_IRQ 1    // D0
#define LORA_BUSY 6   // D5

#define LR2021_IRQ_PIN LORA_IRQ
// NiceRF routes the module IRQ pin from DIO9; RadioLib defaults to DIO5, which
// is the 2.4 GHz LNA switch on this module.
#define LR2021_IRQ_DIO_NUM 9
#define LR2021_NRESET_PIN LORA_RESET
#define LR2021_BUSY_PIN LORA_BUSY
#define LR2021_SPI_NSS_PIN LORA_CS
#define LR2021_SPI_SCK_PIN LORA_SCK
#define LR2021_SPI_MOSI_PIN LORA_MOSI
#define LR2021_SPI_MISO_PIN LORA_MISO

// The NiceRF LR2021 module uses its internal RF switch on DIO5..DIO8.
#define LR2021_DIO_AS_RF_SWITCH
// The LR11x0 interface is still compiled into the same translation unit by
// InterfacesTemplates.cpp; without this it would define a second fallback
// rfswitch_table and clash with the LR2021 one. Both interfaces then share
// the same rfswitch.h, as on the ProMicro LR2021 variant.
#define LR11X0_DIO_AS_RF_SWITCH

// NiceRF demo code sets LR20XX_SYSTEM_TCXO_CTRL_3_3V for the LR2021 module.
#define LR2021_DIO3_TCXO_VOLTAGE 3.3

// Safe limits for the NiceRF module; adjust only with the module datasheet.
#ifndef LR2021_MAX_POWER
#define LR2021_MAX_POWER 22
#endif
#ifndef LR2021_MAX_POWER_HF
#define LR2021_MAX_POWER_HF 12
#endif

#endif
