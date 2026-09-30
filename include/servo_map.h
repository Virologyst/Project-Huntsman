// Huntsman - servo map: which board/channel drives each leg joint, and its calibration
#pragma once

#include <Arduino.h>

// Joint letters: 'K' = knee, 'Y' = lift (femur), 'X' = swing (coxa)
struct Joint {
    const char *leg;    // "FL", "FML", "BML", "BL", "FR", "FMR", "BMR", "BR"
    char type;          // 'K', 'Y' or 'X'
    uint8_t board;      // 1 = 0x40, 2 = 0x41  (changeable with 'assign', saved to flash)
    uint8_t channel;    // 0-15
    int16_t minUs;      // physical stop / safe limit
    int16_t maxUs;
    int16_t neutralUs;  // rest position
    int8_t dir;         // +1 / -1 so offsets mean the same motion on both sides
};

namespace servos {

constexpr int COUNT = 24;
constexpr int LEG_COUNT = 8;
extern Joint joints[COUNT];
extern const char *const LEGS[LEG_COUNT];

bool isLeg(const char *name);
int find(const char *leg, char type);  // index into joints[], -1 if none
int findByOutput(int board, int ch);   // joint wired to that output, -1 if none

int position(int i);                   // current pulse in us, 0 = off
int moveRaw(int i, int us);            // physical pulse, clamped to the joint's min/max; returns pulse sent
int moveOffset(int i, int offsetUs);   // neutral + offset * dir, clamped
void off(int i);
void allNeutral(uint16_t paceMs = 20); // paced to avoid a current spike

void resetDefaults();  // prototype map, not saved
bool load();           // from flash; false if nothing saved (defaults kept)
void save();

void printTable();
void printExport();    // C++ rows to paste into servo_map.cpp

}  // namespace servos
