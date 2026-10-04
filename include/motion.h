// Huntsman - coordinated joint motion (ramps, stand / sit)
#pragma once

#include <Arduino.h>

namespace motion {

// Ramp the listed joints from their current pulse to targets over ms, all together.
// If abortable, any serial input aborts and holds position. Returns false if aborted.
// easeOut (optional, per joint): that joint decelerates to a stop instead of arriving at full speed.
bool ramp(const int joints[], const int targets[], int count, uint16_t ms, bool abortable = true,
          const bool easeOut[] = nullptr);

// Ramp one joint type ('K', 'Y', 'X') on all 8 legs to neutral + offset * dir
bool rampType(char type, int offsetUs, uint16_t ms);

// All 24 joints to their neutral together
bool rampAllCenter(uint16_t ms);

// Stand-up sequence (see cfg::STAND_*). stepMode waits for Enter before each step.
bool standUp(bool stepMode);

// Lower the body and return every joint to neutral
bool sitDown();

// True when Y and K are at the stand pose (X may be anywhere)
bool isStanding();

enum class Gait { Forward, Back, TurnLeft, TurnRight };

// Alternating tetrapod (see cfg::WALK_*). cycles = 0 walks until a key is pressed; a key always lets the
// current step finish, then both groups recentre and the robot ends in the stand pose.
// keepGoing (optional) is checked after each half-cycle like a key: returning false stops the same way.
bool walk(Gait g, int cycles, bool (*keepGoing)() = nullptr);

// Climb an obstacle ahead (flags::CLIMB from the ToF sensor, or 'climb'). Starts and ends in the stand
// pose. Sequence itself is still TODO (see motion.cpp / cfg::CLIMB_*).
bool climb();

}  // namespace motion
