// Huntsman - coordinated joint motion (ramps, stand / sit)
#pragma once

#include <Arduino.h>

namespace motion {

// Easing per joint in a ramp (bit flags). EASE_IN / EASE_OUT: short speed-up / slow-down over
// cfg::EASE_FRACTION of the ramp where the joint starts or stops; EASE_SOFT: full quadratic slow-down to a
// stop (gentle touchdown). Joints that keep moving the same way into the next ramp use EASE_NONE at the join.
enum : uint8_t { EASE_NONE = 0, EASE_IN = 1, EASE_OUT = 2, EASE_BOTH = 3, EASE_SOFT = 4 };

// Ramp the listed joints from their current pulse to targets over ms, all together.
// If abortable, any serial input aborts and holds position. Returns false if aborted.
// ease (optional, per joint, EASE_*); omitted = EASE_BOTH for every joint.
bool ramp(const int joints[], const int targets[], int count, uint16_t ms, bool abortable = true,
          const uint8_t ease[] = nullptr);

// Walk speed: every walk timing is divided by this (1 = cfg timings; the controller stick sets it from
// cfg::WALK_MIN_SPEED at the dead zone to 1 at full push)
void setSpeed(float s);

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

// With keepGoing (controller) a walk pauses mid-step when it returns false and stays in the walk pose;
// the next walk() carries on from there. These manage that state:
bool inWalkPose();  // paused / walking in the walk pose
bool canWalk();     // stand pose or walk pose
void endWalk();     // walk pose -> stand pose (each group lifts, places, lowers)
void resetWalk();   // forget the walk pose (outputs were turned off)

// Climb an obstacle ahead (flags::CLIMB from the ToF sensor, or 'climb'). Starts and ends in the stand
// pose. Sequence itself is still TODO (see motion.cpp / cfg::CLIMB_*).
bool climb();

// Climb button (controller LT / 'climb'): unlocks forward walking past the obstacle stop (TOF_STOP_MM) and
// runs climb() if standing. Walking forward from the walk pose then carries on.
void climbPressed();

// Walking this way is held at an obstacle (forward only, inside TOF_STOP_MM, climb not pressed)
bool blocked(Gait g);

}  // namespace motion
