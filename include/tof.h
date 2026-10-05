// Huntsman - forward-facing time-of-flight range sensor (VL53L0X on the I2C bus, docs/hardware.md)
#pragma once

#include <Arduino.h>

namespace tof {

bool begin();      // after pwm::begin() (shares its I2C bus); true if the sensor answered
void handle();     // call often: takes a new sample if one is ready, raises flags::CLIMB on an obstacle
bool found();
int distanceMm();  // last reading, -1 = nothing in range / no reading yet
bool obstacle();   // currently inside the climb band (with hysteresis, cfg::TOF_NEAR_MM / TOF_NEAR_CLEAR_MM)
void printStatus();

}  // namespace tof
