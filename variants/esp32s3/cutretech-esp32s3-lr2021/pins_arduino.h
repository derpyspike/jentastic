#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <stdint.h>

#define USB_VID 0x2886
#define USB_PID 0x0059

// UART0 disabled on this board: no GPS, serial console runs over native USB CDC,
// and D6/D7 belong to I2C/radio. -1 = no pin.
static const uint8_t TX = -1;
static const uint8_t RX = -1;

static const uint8_t SDA = 43;
static const uint8_t SCL = 44;

static const uint8_t SS = 5;
static const uint8_t SCK = 7;
static const uint8_t MOSI = 9;
static const uint8_t MISO = 8;

#endif /* Pins_Arduino_h */
