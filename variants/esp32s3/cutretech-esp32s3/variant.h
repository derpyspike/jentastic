#define HAS_SCREEN 0
#define HAS_GPS 0

#define UART_TX 43
#define UART_RX 44

#define I2C_SDA 47
#define I2C_SCL 48

#define LED_POWER 21
#define LED_STATE_ON 1

#define BATTERY_PIN 1
#define ADC_MULTIPLIER 2.0 // Adjust in configuration if needed, in my case ~2.025
#define ADC_CHANNEL ADC_CHANNEL_0
#define BATTERY_SENSE_RESOLUTION_BITS 12

#define USE_SX1262 // E22-900, E22P-868, E22P-915 series
#define USE_SX1268 // E22-400 series

#define SX126X_MAX_POWER 22 // This doesn't take into account the power of E22 internal PA

#define SX126X_DIO3_TCXO_VOLTAGE 1.8 // E22 series TCXO reference voltage is 1.8V

#define SX126X_DIO2_AS_RF_SWITCH

// SPI pins for LoRa module (used SPI initialization)
#define LORA_SCK  7
#define LORA_MOSI 9
#define LORA_MISO 8
#define LORA_CS   5

// SX126X radio interface pins (used when creating SX1262Interface)
#define SX126X_CS    LORA_CS
#define SX126X_DIO1  2
#define SX126X_RESET 3
#define SX126X_BUSY  4
#define SX126X_RXEN  6
#define SX126X_TXEN  RADIOLIB_NC  // TXEN should be connected to DIO2 (internal to E22 module)

// Legacy LORA_* defines for compatibility (used by sleep.cpp)
#define LORA_RESET SX126X_RESET
#define LORA_DIO1  SX126X_DIO1