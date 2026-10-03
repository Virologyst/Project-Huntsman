// Huntsman - event flags: raised by sensors / inputs, consumed (cleared) by whoever acts on them
#pragma once

#include <Arduino.h>

namespace flags {

enum : uint8_t {
    CLIMB = 1 << 0,  // ToF saw an obstacle closer than cfg::TOF_CLIMB_MM (tof.cpp); loop() runs motion::climb()
    // next: 1 << 1 ...
};

extern volatile uint8_t raised;

inline void set(uint8_t f) { raised |= f; }
inline void clear(uint8_t f) { raised &= (uint8_t)~f; }
inline bool test(uint8_t f) { return (raised & f) != 0; }

}  // namespace flags
