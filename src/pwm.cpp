#include "pwm.h"

#include <Adafruit_PWMServoDriver.h>
#include <Preferences.h>
#include <Wire.h>

#include "config.h"

namespace {

Adafruit_PWMServoDriver drivers[cfg::BOARD_COUNT] = {
    Adafruit_PWMServoDriver(cfg::BOARD_ADDR[0]),
    Adafruit_PWMServoDriver(cfg::BOARD_ADDR[1]),
};
bool found[cfg::BOARD_COUNT] = {};
uint32_t oscHz[cfg::BOARD_COUNT] = {cfg::DEFAULT_OSC_HZ, cfg::DEFAULT_OSC_HZ};
float frame = cfg::DEFAULT_FRAME_HZ;
int active[cfg::BOARD_COUNT][16] = {};  // 0 = off
int last[cfg::BOARD_COUNT] = {};

int idx(int board) { return board - 1; }

void readSettings() {
    Preferences prefs;
    prefs.begin("huntsman", true);
    oscHz[0] = prefs.getUInt("osc0", cfg::DEFAULT_OSC_HZ);
    oscHz[1] = prefs.getUInt("osc1", cfg::DEFAULT_OSC_HZ);
    frame = prefs.getFloat("freq", cfg::DEFAULT_FRAME_HZ);
    prefs.end();
}

// Re-send clock settings and restore active pulses (after osc/frame changes)
void apply(int i) {
    drivers[i].setOscillatorFrequency(oscHz[i]);
    drivers[i].setPWMFreq(frame);
    for (int ch = 0; ch < 16; ch++) {
        if (active[i][ch]) drivers[i].writeMicroseconds(ch, active[i][ch]);
    }
}

}  // namespace

namespace pwm {

bool validBoard(int board) { return board >= 1 && board <= cfg::BOARD_COUNT; }

bool boardFound(int board) { return validBoard(board) && found[idx(board)]; }

bool begin() {
    Wire.begin(cfg::SDA_PIN, cfg::SCL_PIN);
    Wire.setClock(400000);
    readSettings();

    bool all = true;
    for (int i = 0; i < cfg::BOARD_COUNT; i++) {
        Wire.beginTransmission(cfg::BOARD_ADDR[i]);
        found[i] = Wire.endTransmission() == 0;
        all &= found[i];
        drivers[i].begin();
        apply(i);
        for (int ch = 0; ch < 16; ch++) drivers[i].setPWM(ch, 0, 4096);  // full off
    }
    return all;
}

void setPulse(int board, int ch, int us) {
    if (!validBoard(board) || ch < 0 || ch > 15) return;
    us = constrain(us, cfg::HARD_MIN_US, cfg::HARD_MAX_US);
    int i = idx(board);
    drivers[i].writeMicroseconds(ch, us);
    active[i][ch] = us;
    last[i] = us;
}

void setOff(int board, int ch) {
    if (!validBoard(board) || ch < 0 || ch > 15) return;
    int i = idx(board);
    drivers[i].setPWM(ch, 0, 4096);
    active[i][ch] = 0;
}

void allOff() {
    for (int b = 1; b <= cfg::BOARD_COUNT; b++)
        for (int ch = 0; ch < 16; ch++) setOff(b, ch);
}

int pulse(int board, int ch) {
    if (!validBoard(board) || ch < 0 || ch > 15) return 0;
    return active[idx(board)][ch];
}

int lastPulse(int board) { return validBoard(board) ? last[idx(board)] : 0; }

uint32_t osc(int board) { return validBoard(board) ? oscHz[idx(board)] : 0; }

void setOsc(int board, uint32_t hz) {
    if (!validBoard(board)) return;
    oscHz[idx(board)] = hz;
    apply(idx(board));
}

uint32_t calibrate(int board, float measuredUs) {
    // Output pulse scales with assumed/actual clock, so actual = assumed * commanded / measured
    int i = idx(board);
    oscHz[i] = (uint32_t)((double)oscHz[i] * last[i] / measuredUs + 0.5);
    apply(i);
    return oscHz[i];
}

float frameHz() { return frame; }

void setFrameHz(float hz) {
    frame = hz;
    for (int i = 0; i < cfg::BOARD_COUNT; i++) apply(i);
}

void loadSettings() {
    readSettings();
    for (int i = 0; i < cfg::BOARD_COUNT; i++) apply(i);
}

void saveSettings() {
    Preferences prefs;
    prefs.begin("huntsman", false);
    prefs.putUInt("osc0", oscHz[0]);
    prefs.putUInt("osc1", oscHz[1]);
    prefs.putFloat("freq", frame);
    prefs.end();
}

void resetSettings() {
    oscHz[0] = oscHz[1] = cfg::DEFAULT_OSC_HZ;
    frame = cfg::DEFAULT_FRAME_HZ;
    for (int i = 0; i < cfg::BOARD_COUNT; i++) apply(i);
}

}  // namespace pwm
