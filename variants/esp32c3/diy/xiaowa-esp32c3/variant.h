#ifndef _VARIANT_XIAOWA_ESP32C3_H_
#define _VARIANT_XIAOWA_ESP32C3_H_

/*----------------------------------------------------------------------------
 * Headers
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

// I2C (Wire) & OLED
#define WIRE_INTERFACES_COUNT (1)
#define I2C_SDA (7)
#define I2C_SCL (8)

// Display: USE_SSD1306 was here historically, but Nexus-28002
// (2026-08-23) made this board headless via
// MESHTASTIC_EXCLUDE_SCREEN=1 in platformio.ini (the upstream
// esp8266-oled-ssd1306 fork at pinned commit ace0fcbc is missing
// the CONFIG_IDF_TARGET_ESP32C3 guard that upstream ThingPulse has,
// so any Wire1 reference crashes the C3 build). Leaving I2C_SDA/SCL
// defined because other peripherals on this board may still use Wire.

// GPS
#define HAS_GPS 0
#undef GPS_RX_PIN
#undef GPS_TX_PIN
#define GPS_RX_PIN (5)
#define GPS_TX_PIN (6)


// Button
#define BUTTON_PIN (9) // BOOT button

// LoRa
#define USE_LLCC68
#define USE_SX1262
// #define USE_RF95
#define USE_SX1268

#define LORA_DIO0 RADIOLIB_NC
#define LORA_RESET (0)
#define LORA_DIO1 (20)
#define LORA_RXEN (10)
#define LORA_BUSY (21)
#define LORA_SCK (3)
#define LORA_MISO (4)
#define LORA_MOSI (2)
#define LORA_CS (1)

#define SX126X_CS LORA_CS
#define SX126X_DIO1 LORA_DIO1
#define SX126X_BUSY LORA_BUSY
#define SX126X_RESET LORA_RESET
#define SX126X_RXEN LORA_RXEN
#define SX126X_TXEN RADIOLIB_NC //missing on official repo

#define SX126X_DIO2_AS_RF_SWITCH //missing on official repo
#define SX126X_DIO3_TCXO_VOLTAGE (1.8)
#define TCXO_OPTIONAL // make it so that the firmware can try both TCXO and XTAL

#define SX126X_MAX_POWER 22


#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 * Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#endif