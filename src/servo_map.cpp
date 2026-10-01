#include "servo_map.h"

#include <Preferences.h>

#include "config.h"
#include "pwm.h"

namespace {

// Default map from the Arduino UNO prototype.
// Prototype values were PCA9685 ticks at 50 Hz (1 tick = 20000/4096 = 4.883 us); converted to us
// and clamped to 500-2500 us. Limits are UNVERIFIED on the Brain Box - re-map each joint (docs/calibration.md).
const Joint DEFAULTS[servos::COUNT] = {
    // leg    J   brd ch   min   max  neutral dir     prototype ticks min/max/neutral
    {"FL",  'K', 2,  4,  732, 2500,  854, -1},  // 150/600/175
    {"FL",  'Y', 2,  5,  732, 2500, 1831, -1},  // 150/600/375
    {"FL",  'X', 2,  6,  977, 2500, 1709, -1},  // 200/550/350
    {"FML", 'K', 2,  0,  732, 2500,  854, -1},  // 150/600/175
    {"FML", 'Y', 2,  1,  732, 2500, 1831, -1},  // 150/600/375
    {"FML", 'X', 2,  2,  977, 2500, 1831, -1},  // 200/550/375
    {"BML", 'K', 1, 13,  732, 2500, 2500, +1},  // 150/600/575
    {"BML", 'Y', 1, 14,  732, 2500, 1831, +1},  // 150/600/375
    {"BML", 'X', 1, 15,  977, 2500, 1831, +1},  // 200/550/375
    {"BL",  'K', 1,  9,  732, 2500, 2500, +1},  // 150/600/575
    {"BL",  'Y', 1, 10,  732, 2500, 1831, +1},  // 150/600/375
    {"BL",  'X', 1, 11,  977, 2500, 1831, +1},  // 200/550/375
    {"FR",  'K', 2,  9,  732, 2500, 2500, +1},  // 150/600/575
    {"FR",  'Y', 2, 10,  732, 2500, 1831, +1},  // 150/600/375
    {"FR",  'X', 2, 11,  977, 2500, 1709, +1},  // 200/550/350
    {"FMR", 'K', 2, 13,  732, 2500, 2500, +1},  // 150/600/575
    {"FMR", 'Y', 2, 14,  732, 2500, 1831, +1},  // 150/600/375
    {"FMR", 'X', 2, 15,  977, 2500, 1953, +1},  // 200/550/400
    {"BMR", 'K', 1,  4,  732, 2500,  854, -1},  // 150/600/175
    {"BMR", 'Y', 1,  5,  732, 2500, 1831, -1},  // 150/600/375
    {"BMR", 'X', 1,  6,  977, 2500, 1831, -1},  // 200/550/375
    {"BR",  'K', 1,  0,  732, 2500,  854, -1},  // 150/600/175
    {"BR",  'Y', 1,  1,  732, 2500, 1831, -1},  // 150/600/375
    {"BR",  'X', 1,  2,  977, 2500, 1831, -1},  // 200/550/375
};

// Per-joint calibration stored in flash (leg/type stay fixed in code)
struct Cal {
    uint8_t board, channel;
    int16_t minUs, maxUs, neutralUs;
    int8_t dir;
    uint8_t wired;
} __attribute__((packed));

constexpr uint8_t CAL_VERSION = 3;  // v2 added board/channel, v3 added wired

}  // namespace

namespace servos {

Joint joints[COUNT];
const char *const LEGS[LEG_COUNT] = {"FL", "FML", "BML", "BL", "FR", "FMR", "BMR", "BR"};

bool isLeg(const char *name) {
    for (auto leg : LEGS)
        if (!strcasecmp(leg, name)) return true;
    return false;
}

int find(const char *leg, char type) {
    type = toupper(type);
    for (int i = 0; i < COUNT; i++)
        if (!strcasecmp(joints[i].leg, leg) && joints[i].type == type) return i;
    return -1;
}

int findByOutput(int board, int ch) {
    for (int i = 0; i < COUNT; i++)
        if (joints[i].board == board && joints[i].channel == ch) return i;
    return -1;
}

int position(int i) { return pwm::pulse(joints[i].board, joints[i].channel); }

int moveRaw(int i, int us) {
    const Joint &j = joints[i];
    us = constrain(us, j.minUs, j.maxUs);
    pwm::setPulse(j.board, j.channel, us);
    return us;
}

int moveOffset(int i, int offsetUs) { return moveRaw(i, joints[i].neutralUs + offsetUs * joints[i].dir); }

void off(int i) { pwm::setOff(joints[i].board, joints[i].channel); }

void allNeutral(uint16_t paceMs) {
    for (int i = 0; i < COUNT; i++) {
        moveRaw(i, joints[i].neutralUs);
        delay(paceMs);
    }
}

void resetDefaults() { memcpy(joints, DEFAULTS, sizeof(joints)); }

bool load() {
    resetDefaults();
    Preferences prefs;
    prefs.begin("huntsman", true);
    Cal cal[COUNT];
    bool ok = prefs.getUChar("jver", 0) == CAL_VERSION &&
              prefs.getBytesLength("joints") == sizeof(cal) &&
              prefs.getBytes("joints", cal, sizeof(cal)) == sizeof(cal);
    prefs.end();
    if (!ok) return false;
    for (int i = 0; i < COUNT; i++) {
        joints[i].board = cal[i].board;
        joints[i].channel = cal[i].channel;
        joints[i].minUs = cal[i].minUs;
        joints[i].maxUs = cal[i].maxUs;
        joints[i].neutralUs = cal[i].neutralUs;
        joints[i].dir = cal[i].dir;
        joints[i].wired = cal[i].wired;
    }
    return true;
}

void save() {
    Cal cal[COUNT];
    for (int i = 0; i < COUNT; i++)
        cal[i] = {joints[i].board, joints[i].channel, joints[i].minUs, joints[i].maxUs, joints[i].neutralUs,
                  joints[i].dir, joints[i].wired};
    Preferences prefs;
    prefs.begin("huntsman", false);
    prefs.putBytes("joints", cal, sizeof(cal));
    prefs.putUChar("jver", CAL_VERSION);
    prefs.end();
}

void printTable() {
    Serial.println("\n Leg  J  Brd Ch  Wired   Min  Neut   Max  Dir    Now");
    int wired = 0;
    for (int i = 0; i < COUNT; i++) {
        const Joint &j = joints[i];
        int now = position(i);
        wired += j.wired;
        Serial.printf(" %-4s %c   %d  %2d  %-5s  %4d  %4d  %4d  %+d   ", j.leg, j.type, j.board, j.channel,
                      j.wired ? "yes" : "-", j.minUs, j.neutralUs, j.maxUs, j.dir);
        if (now) Serial.printf("%4d\n", now);
        else Serial.println(" off");
    }
    Serial.printf("%d of %d joints confirmed on the harness\n", wired, COUNT);
    for (int i = 0; i < COUNT; i++)
        for (int k = i + 1; k < COUNT; k++)
            if (joints[i].board == joints[k].board && joints[i].channel == joints[k].channel)
                Serial.printf("WARNING: %s %c and %s %c share board %d ch %d\n", joints[i].leg, joints[i].type,
                              joints[k].leg, joints[k].type, joints[i].board, joints[i].channel);
}

void printExport() {
    Serial.println("\n// Paste into DEFAULTS in src/servo_map.cpp");
    for (int i = 0; i < COUNT; i++) {
        const Joint &j = joints[i];
        char leg[8];
        snprintf(leg, sizeof(leg), "\"%s\",", j.leg);
        Serial.printf("    {%-6s '%c', %d, %2d, %4d, %4d, %4d, %+d},\n", leg, j.type, j.board, j.channel,
                      j.minUs, j.maxUs, j.neutralUs, j.dir);
    }
}

}  // namespace servos
