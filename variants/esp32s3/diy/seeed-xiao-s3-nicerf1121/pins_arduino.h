#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <stdint.h>

#define USB_VID 0x2886
#define USB_PID 0x0059

// UART0 would default to D6/D7 (GPIO43/44), which are radio CS/SCK on this board.
// Serial runs over native USB CDC, so keep UART0 off those pins (same as cutretech).
static const uint8_t TX = -1;
static const uint8_t RX = -1;

// GPIO48 Reference: https://github.com/espressif/arduino-esp32/pull/8600

// The default Wire will be mapped to Screen and Sensors
static const uint8_t SDA = 7;
static const uint8_t SCL = 8;

// Default SPI will be mapped to Radio
static const uint8_t MISO = 4;
static const uint8_t SCK = 44;
static const uint8_t MOSI = 5;
static const uint8_t SS = 43;

#endif /* Pins_Arduino_h */
