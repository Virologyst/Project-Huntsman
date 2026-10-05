#include "servo_map.h"

#include <Preferences.h>

#include "config.h"
#include "term.h"
#include "pwm.h"

namespace {

// Default map. Board/channel and dir verified on the rebuilt robot with 'wiggle' (2026-10-03):
// board 1 = left legs, board 2 = right legs. dir convention: + = lift up, knee up, swing forward.
// neutral = centre for all motion (1500; FML X and FMR X +150 forward, 2026-10-03, so the middle
// legs clear BML/BMR when walking). min/max are still the
// Arduino UNO prototype values (ticks x 4.883 us, clamped to 500-2500) - UNVERIFIED.
const Joint DEFAULTS[servos::COUNT] = {
    // leg    J   brd ch   min   max  neutral dir  wired
    {"FL",  'K', 1,  2,  732, 2500, 1500, -1, true},
    {"FL",  'Y', 1,  1,  732, 2500, 1500, -1, true},
    {"FL",  'X', 1,  0,  977, 2500, 1500, +1, true},
    {"FML", 'K', 1,  5,  732, 2500, 1420, +1, true},  // trimmed 80 us toward the body (sat further out)
    {"FML", 'Y', 1,  3,  732, 2500, 1500, +1, true},
    {"FML", 'X', 1,  4,  977, 2500, 1650, +1, true},  // trimmed 150 us forward - clears BML
    {"BML", 'K', 1, 11,  732, 2500, 1500, -1, true},
    {"BML", 'Y', 1,  9,  732, 2500, 1500, -1, true},
    {"BML", 'X', 1, 10,  977, 2500, 1500, +1, true},
    {"BL",  'K', 1, 12,  732, 2500, 1500, +1, true},
    {"BL",  'Y', 1, 14,  732, 2500, 1500, +1, true},
    {"BL",  'X', 1, 13,  977, 2500, 1500, +1, true},
    {"FR",  'K', 2, 13,  732, 2500, 1500, +1, true},
    {"FR",  'Y', 2, 15,  732, 2500, 1500, +1, true},
    {"FR",  'X', 2, 14,  977, 2500, 1500, -1, true},
    {"FMR", 'K', 2, 10,  732, 2500, 1500, -1, true},
    {"FMR", 'Y', 2,  9,  732, 2500, 1500, -1, true},
    {"FMR", 'X', 2, 11,  977, 2500, 1350, -1, true},  // trimmed 150 us forward (dir -1) - clears BMR
    {"BMR", 'K', 2,  4,  732, 2500, 1500, +1, true},
    {"BMR", 'Y', 2,  6,  732, 2500, 1500, +1, true},
    {"BMR", 'X', 2,  5,  977, 2500, 1500, -1, true},
    {"BR",  'K', 2,  1,  732, 2500, 1500, -1, true},
    {"BR",  'Y', 2,  0,  732, 2500, 1500, -1, true},
    {"BR",  'X', 2,  2,  977, 2500, 1500, -1, true},
};

// Per-joint calibration stored in flash (leg/type stay fixed in code)
struct Cal {
    uint8_t board, channel;
    int16_t minUs, maxUs, neutralUs;
    int8_t dir;
    uint8_t wired;
} __attribute__((packed));

constexpr uint8_t CAL_VERSION = 5;  // v2 board/channel, v3 wired, v4 = reset to wiggle-verified defaults,
                                    // v5 = reset to defaults with FMR X trim 100 -> 150

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
    Term.println("\n Leg  J  Brd Ch  Wired   Min  Neut   Max  Dir    Now");
    int wired = 0;
    for (int i = 0; i < COUNT; i++) {
        const Joint &j = joints[i];
        int now = position(i);
        wired += j.wired;
        Term.printf(" %-4s %c   %d  %2d  %-5s  %4d  %4d  %4d  %+d   ", j.leg, j.type, j.board, j.channel,
                      j.wired ? "yes" : "-", j.minUs, j.neutralUs, j.maxUs, j.dir);
        if (now) Term.printf("%4d\n", now);
        else Term.println(" off");
    }
    Term.printf("%d of %d joints confirmed on the harness\n", wired, COUNT);
    for (int i = 0; i < COUNT; i++)
        for (int k = i + 1; k < COUNT; k++)
            if (joints[i].board == joints[k].board && joints[i].channel == joints[k].channel)
                Term.printf("WARNING: %s %c and %s %c share board %d ch %d\n", joints[i].leg, joints[i].type,
                              joints[k].leg, joints[k].type, joints[i].board, joints[i].channel);
}

void printExport() {
    Term.println("\n// Paste into DEFAULTS in src/servo_map.cpp");
    for (int i = 0; i < COUNT; i++) {
        const Joint &j = joints[i];
        char leg[8];
        snprintf(leg, sizeof(leg), "\"%s\",", j.leg);
        Term.printf("    {%-6s '%c', %d, %2d, %4d, %4d, %4d, %+d},\n", leg, j.type, j.board, j.channel,
                      j.minUs, j.maxUs, j.neutralUs, j.dir);
    }
}

}  // namespace servos
