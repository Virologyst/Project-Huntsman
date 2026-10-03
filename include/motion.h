// Huntsman - coordinated joint motion (ramps, stand / sit)
#pragma once

#include <Arduino.h>

namespace motion {

// Ramp the listed joints from their current pulse to targets over ms, all together.
// Any serial input aborts and holds position. Returns false if aborted.
bool ramp(const int joints[], const int targets[], int count, uint16_t ms);

// Ramp one joint type ('K', 'Y', 'X') on all 8 legs to centre + offset * dir
bool rampType(char type, int offsetUs, uint16_t ms);

// All 24 joints to cfg::STAND_CENTER_US together
bool rampAllCenter(uint16_t ms);

// Stand-up sequence (see cfg::STAND_*). stepMode waits for Enter before each step.
bool standUp(bool stepMode);

// Lower the body and return every joint to centre
bool sitDown();

}  // namespace motion
