// Huntsman - hardware constants for the Brain Box
#pragma once

#include <Arduino.h>

namespace cfg {

constexpr uint32_t SERIAL_BAUD = 115200;

// I2C to the PCA9685 boards
constexpr uint8_t SDA_PIN = 8;
constexpr uint8_t SCL_PIN = 9;

// Board numbers match the prototype: board 1 = 0x40 (back legs), board 2 = 0x41 (front legs)
constexpr int BOARD_COUNT = 2;
constexpr uint8_t BOARD_ADDR[BOARD_COUNT] = {0x40, 0x41};

constexpr uint32_t DEFAULT_OSC_HZ = 25000000;  // PCA9685 nominal internal clock
constexpr float DEFAULT_FRAME_HZ = 50.0f;      // standard servo frame rate

// Absolute pulse limits for any output, regardless of joint calibration
constexpr int HARD_MIN_US = 400;
constexpr int HARD_MAX_US = 2600;

}  // namespace cfg
