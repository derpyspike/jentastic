#define HAS_SCREEN 0
#define HAS_GPS 0
// GPS_EN PIN 8

#define UART_TX 43
#define UART_RX 44

#define I2C_SDA 1
#define I2C_SCL 2

// Neopixel and regular led shares the same pin, leaving neopixel bc it's cooler.
//#define LED_PIN 48
//#define LED_STATE_ON 1

#define HAS_NEOPIXEL                         // Enable the use of neopixels
#define NEOPIXEL_COUNT 1                     // How many neopixels are connected
#define NEOPIXEL_DATA 48                     // gpio pin used to send data to the neopixels
#define NEOPIXEL_TYPE (NEO_GRB + NEO_KHZ800) // type of neopixels in use

#define BATTERY_PIN 7 // Board uses pin3 but measures after the LDO
#define ADC_MULTIPLIER 2.0 // Adjust in configuration if needed, in my case ~2.025
#define ADC_CHANNEL ADC1_GPIO7_CHANNEL

#define USE_SX1262 // E22-900, E22P-868, E22P-915 series
#define USE_SX1268 // E22-400 series

#define SX126X_MAX_POWER 22 // This doesn't take into account the power of E22 internal PA

#define SX126X_DIO3_TCXO_VOLTAGE 1.8 // E22 series TCXO reference voltage is 1.8V

#define SX126X_DIO2_AS_RF_SWITCH

// SPI pins for LoRa module (used SPI initialization)
#define LORA_SCK  12
#define LORA_MOSI 11
#define LORA_MISO 13
#define LORA_CS   10

#define LORA_RESET 9
#define LORA_BUSY 6
#define LORA_DIO1 5

// SX126X radio interface pins (used when creating SX1262Interface)
#define SX126X_CS    LORA_CS
#define SX126X_DIO1  LORA_DIO1
#define SX126X_RESET LORA_RESET
#define SX126X_BUSY  LORA_BUSY
#define SX126X_RXEN  4
#define SX126X_TXEN  RADIOLIB_NC  // TXEN should be connected to DIO2 (internal to E22 module)

// LR1121
#ifdef USE_LR1121
#define LR1121_IRQ_PIN LORA_DIO1
#define LR1121_NRESET_PIN LORA_RESET
#define LR1121_BUSY_PIN LORA_BUSY
#define LR1121_SPI_NSS_PIN LORA_CS
#define LR1121_SPI_SCK_PIN LORA_SCK
#define LR1121_SPI_MOSI_PIN LORA_MOSI
#define LR1121_SPI_MISO_PIN LORA_MISO
#define LR11X0_DIO3_TCXO_VOLTAGE 1.8
#define LR11X0_DIO_AS_RF_SWITCH
#endif

#define TCXO_OPTIONAL // make it so that the firmware can try both TCXO and XTAL
