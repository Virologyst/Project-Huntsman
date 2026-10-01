// Huntsman - low-level PCA9685 control (boards numbered 1 and 2)
#pragma once

#include <Arduino.h>

namespace pwm {

bool begin();                          // init I2C + boards, all outputs off; true if every board answered
bool validBoard(int board);
bool boardFound(int board);

void setPulse(int board, int ch, int us);  // clamped to cfg::HARD_MIN_US..HARD_MAX_US
void setOff(int board, int ch);            // no pulses: servo goes limp
void allOff();
int pulse(int board, int ch);              // current pulse in us, 0 = off
int lastPulse(int board);                  // last pulse commanded on that board

uint32_t osc(int board);
void setOsc(int board, uint32_t hz);
uint32_t calibrate(int board, float measuredUs);  // corrects osc from a scope reading of lastPulse()
uint32_t calibrateFromFrame(int board, float measuredHz);  // corrects osc from the measured frame rate
float frameHz();
void setFrameHz(float hz);

void loadSettings();   // from flash, then re-applied to the boards
void saveSettings();
void resetSettings();  // defaults, not saved

}  // namespace pwm
