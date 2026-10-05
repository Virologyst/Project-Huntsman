#include "motion.h"

#include "config.h"
#include "flags.h"
#include "term.h"
#include "pwm.h"
#include "servo_map.h"
#include "tof.h"

namespace motion {
float progress(float t, uint8_t ease);  // easing curve, defined below
}

namespace {

bool aborted() {
    if (!Term.available()) return false;
    while (Term.available()) Term.read();
    Term.println("\nAborted - holding position. 'limp' to release, 'sit' to lower.");
    return true;
}

// Step mode: wait for Enter (q cancels)
bool waitStep(const char *what) {
    Term.printf("Next: %s - Enter to go, q to stop: ", what);
    while (true) {
        if (!Term.available()) { delay(5); continue; }
        char c = Term.read();
        if (c == 'q' || c == 'Q') {
            while (Term.available()) Term.read();
            Term.println("\nStopped - holding position.");
            return false;
        }
        if (c == '\r' || c == '\n') {
            delay(20);
            while (Term.available()) Term.read();
            Term.println();
            return true;
        }
    }
}

bool step(bool stepMode, const char *what, char type, int offsetUs, uint16_t ms) {
    if (stepMode && !waitStep(what)) return false;
    Term.printf("%s ...\n", what);
    if (!motion::rampType(type, offsetUs, ms)) return false;
    if (!stepMode) delay(cfg::STAND_PAUSE_MS);
    return true;
}

// ---------- walking: joint-space alternating tetrapod ----------

// Alternate along each side and across, so 4 legs are always down
const char *const GROUP_A[] = {"FL", "BML", "FMR", "BR"};
const char *const GROUP_B[] = {"FML", "BL", "FR", "BMR"};

bool isLeft(const char *leg) { return leg[strlen(leg) - 1] == 'L'; }

// One coordinated ramp: joints and targets as offsets from centre, following each joint's dir
struct Pose {
    int joints[servos::COUNT];
    int targets[servos::COUNT];
    uint8_t ease[servos::COUNT];
    int n = 0;

    void add(const char *leg, char type, int offsetUs, uint8_t easing = motion::EASE_BOTH) {
        int i = servos::find(leg, type);
        addRaw(i, servos::joints[i].neutralUs + offsetUs * servos::joints[i].dir, easing);
    }
    void addRaw(int joint, int target, uint8_t easing = motion::EASE_BOTH) {
        joints[n] = joint;
        targets[n] = target;
        ease[n] = easing;
        n++;
    }
    int find(int joint) const {
        for (int k = 0; k < n; k++)
            if (joints[k] == joint) return k;
        return -1;
    }
    void run(uint16_t ms) { motion::ramp(joints, targets, n, ms, false, ease); }  // a step always completes
};

// +1 = this leg's swing goes forward when walking this way
int strideSign(const char *leg, motion::Gait g) {
    switch (g) {
        case motion::Gait::Forward: return +1;
        case motion::Gait::Back: return -1;
        case motion::Gait::TurnLeft: return isLeft(leg) ? -1 : +1;
        case motion::Gait::TurnRight: return isLeft(leg) ? +1 : -1;
    }
    return 0;
}

// Corner legs (FL FR BL BR) have 2-letter names; middle legs (FML FMR BML BMR) have 3
bool isCorner(const char *leg) { return strlen(leg) == 2; }

bool isTurn(motion::Gait g) { return g == motion::Gait::TurnLeft || g == motion::Gait::TurnRight; }

// Walk pose per leg: corners swing toward head / tail and straighten a little; middle legs reach out,
// with the back-middle pair set back a little so it stays clear of the front-middle pair.
// Turning, the back corners stride on X around a base shifted forward, so their tail-most point is the
// walking base (at base - stride they hit the battery).
int walkBaseX(const char *leg, motion::Gait g = motion::Gait::Forward) {
    bool front = leg[0] == 'F';
    if (isCorner(leg)) {
        if (front) return cfg::WALK_FRONT_SPREAD_US;
        return -cfg::WALK_BACK_SPREAD_US + (isTurn(g) ? cfg::WALK_TURN_REAR_SHIFT_US : 0);
    }
    return front ? 0 : -cfg::WALK_BACK_MID_SPREAD_US;
}
int walkKnee(const char *leg) { return isCorner(leg) ? cfg::WALK_CORNER_KNEE_US : cfg::WALK_MID_KNEE_US; }
int walkStride(const char *leg) { return isCorner(leg) ? cfg::WALK_STRIDE_CORNER_US : cfg::WALK_STRIDE_MID_US; }

// X at one end of a leg's step: s = +1 forward end, -1 back end (FML reaches a little further forward)
int strideEnd(const char *leg, int s, motion::Gait g) {
    int extra = (s > 0 && !strcmp(leg, "FML")) ? cfg::WALK_FML_EXTRA_REACH_US : 0;
    return walkBaseX(leg, g) + s * walkStride(leg) + extra;
}

bool isFront(const char *leg) { return leg[0] == 'F'; }

// Y lift above the stand pose while a leg is in the air: the front corner legs lift higher
// ... and higher still while the ToF sees something close in front (cfg::TOF_NEAR_MM)
int walkLift(const char *leg) {
    if (!isCorner(leg)) return cfg::WALK_MID_LIFT_US;
    if (!isFront(leg)) return cfg::WALK_LIFT_US;
    return cfg::WALK_FRONT_LIFT_US + (tof::obstacle() ? cfg::WALK_FRONT_OBSTACLE_LIFT_US : 0);
}

// Corner legs walking forward / back hold X and stride with K and Y instead: the front pair reaches out
// and pulls in, the back pair sets down tucked and pushes out. Turns still swing them on X.
bool kneeStroke(const char *leg, motion::Gait g) {
    return isCorner(leg) && (g == motion::Gait::Forward || g == motion::Gait::Back);
}

// Corner-leg K / ground-Y offsets at the extended or tucked end of its stroke
int strokeK(const char *leg, bool extended) {
    if (isFront(leg)) return extended ? cfg::WALK_FRONT_REACH_KNEE_US : cfg::WALK_FRONT_PULL_KNEE_US;
    return extended ? cfg::WALK_REAR_PUSH_KNEE_US : cfg::WALK_REAR_TUCK_KNEE_US;
}
int strokeY(const char *leg, bool extended) {
    if (isFront(leg)) return extended ? cfg::WALK_FRONT_REACH_Y_US : cfg::WALK_FRONT_PULL_Y_US;
    return extended ? cfg::WALK_REAR_PUSH_Y_US : cfg::WALK_REAR_TUCK_Y_US;
}

float speed = 1.0f;     // motion::setSpeed (controller stick)
float approach = 1.0f;  // obstacle slow-down for the current half-step (approachFactor)
int32_t scaled(uint16_t ms) { return max<int32_t>(cfg::FRAME_MS, (int32_t)(ms / (speed * approach))); }

// ---- obstacle approach (ToF, forward walking only) ----
bool climbCleared = false;  // climb pressed: forward walking allowed past TOF_STOP_MM until the range clears
bool obstacleHit = false;   // the last half-step stopped at the obstacle
bool blockWarned = false;   // obstacle message already printed for this stop

// Forward-walk speed factor from the range: 1 beyond TOF_SLOW_START_MM, then one step slower every
// TOF_SLOW_STEP_MM (400-300: 0.75, 300-200: 0.5, 200-100: 0.25 with the defaults)
float approachFactor(motion::Gait g) {
    int d = tof::distanceMm();
    if (g != motion::Gait::Forward || !tof::found() || d < 0 || d > cfg::TOF_SLOW_START_MM) return 1.0f;
    int bands = (cfg::TOF_SLOW_START_MM - cfg::TOF_STOP_MM) / cfg::TOF_SLOW_STEP_MM;
    int band = constrain((d - cfg::TOF_STOP_MM + cfg::TOF_SLOW_STEP_MM - 1) / cfg::TOF_SLOW_STEP_MM, 1, bands);
    return (float)band / (bands + 1);
}

// Forward walking must stop: obstacle at / inside TOF_STOP_MM and climb not pressed for it
bool obstacleStop(motion::Gait g) {
    int d = tof::distanceMm();
    if (d < 0 || d > cfg::TOF_STOP_MM + cfg::TOF_STOP_REARM_MM) climbCleared = false;  // re-arm
    return g == motion::Gait::Forward && tof::found() && d >= 0 && d <= cfg::TOF_STOP_MM && !climbCleared;
}

// Per-frame check while walking: the obstacle stop plus the caller's keep-going (controller)
motion::Gait walkGait = motion::Gait::Forward;
bool (*walkKeepGoing)() = nullptr;
bool walkContinue() {
    if (obstacleStop(walkGait)) {
        obstacleHit = true;
        return false;
    }
    return !walkKeepGoing || walkKeepGoing();
}

int startPos(int joint, int fallback) { return servos::position(joint) ? servos::position(joint) : fallback; }

// A walk half-cycle as a timeline: every joint gets its own timed segments (start ms, end ms, target,
// easing), all played together frame by frame. Lets the middle legs step at a different rate from the
// corner legs inside the same half-cycle.
struct Timeline {
    struct Seg {
        int joint;
        int32_t t0, t1;
        int target;
        uint8_t ease;
        int from;
        bool started, done;
    };
    Seg seg[96];
    int n = 0;

    // Segments for one joint must be added in time order (each starts from where the previous one ended)
    void addRaw(int joint, int32_t t0, int32_t t1, int target, uint8_t ease) {
        if (n < 96) seg[n++] = {joint, t0, max(t1, t0 + 1), target, ease, 0, false, false};
    }
    void add(const char *leg, char type, int32_t t0, int32_t t1, int offsetUs, uint8_t ease) {
        int i = servos::find(leg, type);
        addRaw(i, t0, t1, servos::joints[i].neutralUs + offsetUs * servos::joints[i].dir, ease);
    }
    // keepGoing (optional) is checked every frame: false = stop right there and hold (returns false)
    bool run(int32_t total, bool (*keepGoing)() = nullptr) {
        for (int32_t t = cfg::FRAME_MS;; t += cfg::FRAME_MS) {
            if (t > total) t = total;
            uint32_t frame = millis();
            for (int k = 0; k < n; k++) {
                Seg &sg = seg[k];
                if (sg.done || t <= sg.t0) continue;
                if (!sg.started) {
                    int now = servos::position(sg.joint);
                    sg.from = now ? now : sg.target;
                    sg.started = true;
                }
                float p = (float)(t - sg.t0) / (sg.t1 - sg.t0);
                if (p >= 1.0f) {
                    p = 1.0f;
                    sg.done = true;
                }
                servos::moveRaw(sg.joint, sg.from + (int)lroundf((sg.target - sg.from) * motion::progress(p, sg.ease)));
            }
            if (t >= total) break;
            tof::handle();  // frames block loop(), so sample here (drives the front legs' obstacle lift)
            if (keepGoing && !keepGoing()) return false;
            while (millis() - frame < cfg::FRAME_MS) {}
        }
        return true;
    }
};

// Middle legs step in their own pairs, alternating every middle sub-cycle across the whole walk
const char *const MID_1[] = {"BML", "FMR"};  // the middle legs of group A
const char *const MID_2[] = {"FML", "BMR"};  // the middle legs of group B
int midTurn = 0;                              // middle sub-cycles so far this walk (even = MID_1 swings)
bool inWalk = false;                          // legs are in the walk pose (paused or walking)
int nextHalf = 0;                             // 0 = group A swings next, 1 = group B

// ---- middle-leg IK (lengths and 1500 us joint angles in config.h, docs/hardware.md) ----
// Angles in degrees from physical pulses. yaw: leg direction from the head, outward (forward swing = smaller).
// femur: + up from level. knee: angle between femur and tibia (180 = straight out, 90 = tibia square to it).
struct MidLeg {
    int x, y, k;           // joint indexes
    float yaw0, knee0;     // at 1500 us
};

MidLeg midLeg(const char *leg) {
    bool front = leg[0] == 'F';
    return {servos::find(leg, 'X'), servos::find(leg, 'Y'), servos::find(leg, 'K'),
            front ? cfg::MID_FRONT_YAW_DEG : cfg::MID_BACK_YAW_DEG,
            front ? cfg::MID_FRONT_KNEE_DEG : cfg::MID_BACK_KNEE_DEG};
}

float pulseDeg(int joint, int us) { return (us - 1500) * servos::joints[joint].dir / cfg::US_PER_DEG; }
int degPulse(int joint, float deg) { return 1500 + (int)lroundf(deg * cfg::US_PER_DEG * servos::joints[joint].dir); }

float yawAt(const MidLeg &m, int xUs) { return m.yaw0 - pulseDeg(m.x, xUs); }

// Foot from pulses: reach r (mm, horizontal from the X axis) and drop h (mm below the Y axis)
void footAt(const MidLeg &m, int yUs, int kUs, float &r, float &h) {
    float femur = radians(pulseDeg(m.y, yUs));
    float tibia = femur - radians(180.0f - (m.knee0 + pulseDeg(m.k, kUs)));
    r = cfg::COXA_MM + cfg::FEMUR_MM * cosf(femur) + cfg::TIBIA_MM * cosf(tibia);
    h = -(cfg::FEMUR_MM * sinf(femur) + cfg::TIBIA_MM * sinf(tibia));
}

// Pulses that put the foot at reach r, drop h (knee above the hip-foot line)
void legFor(const MidLeg &m, float r, float h, int &yUs, int &kUs) {
    const float F = cfg::FEMUR_MM, T = cfg::TIBIA_MM;
    float ry = r - cfg::COXA_MM;
    float d = constrain(sqrtf(ry * ry + h * h), fabsf(F - T) + 1.0f, F + T - 1.0f);
    float knee = degrees(acosf(constrain((F * F + T * T - d * d) / (2 * F * T), -1.0f, 1.0f)));
    float femur = degrees(atan2f(-h, ry) + acosf(constrain((F * F + d * d - T * T) / (2 * F * d), -1.0f, 1.0f)));
    yUs = degPulse(m.y, femur);
    kUs = degPulse(m.k, knee - m.knee0);
}

// Straight-line reference from the walk pose: the foot's sideways distance and drop at the walk base
struct Line {
    float side, h;
    int yBase, kBase;  // walk-pose femur / knee pulses (the IK correction is scaled from these)
};
Line walkLine(const char *leg, motion::Gait g) {
    MidLeg m = midLeg(leg);
    const Joint &jx = servos::joints[m.x], &jy = servos::joints[m.y], &jk = servos::joints[m.k];
    int xUs = jx.neutralUs + walkBaseX(leg, g) * jx.dir;
    int yUs = jy.neutralUs + cfg::STAND_PUSH_US * jy.dir;
    int kUs = jk.neutralUs + walkKnee(leg) * jk.dir;
    float r, h;
    footAt(m, yUs, kUs, r, h);
    return {r * sinf(radians(yawAt(m, xUs))), h, yUs, kUs};
}

// Femur / knee pulses keeping the foot on its line for X pulse xUs
void onLine(const char *leg, const Line &line, int xUs, int &yUs, int &kUs) {
    MidLeg m = midLeg(leg);
    float yaw = constrain(yawAt(m, xUs), 20.0f, 160.0f);
    legFor(m, line.side / sinf(radians(yaw)), line.h, yUs, kUs);
    yUs = line.yBase + (int)lroundf((yUs - line.yBase) * cfg::WALK_MID_IK_GAIN);
    kUs = line.kBase + (int)lroundf((kUs - line.kBase) * cfg::WALK_MID_IK_GAIN);
}

// Half a gait cycle (one corner pair swings, the other pushes). Corner legs, three phases:
//   air   - swing corners lift and swing to the next stroke start (big knee strokes carry on into down)
//   down  - swing corners lower; front feet stop WALK_FRONT_APPROACH_US above the ground
//   touch - front feet set down slowly (servos can't follow a fast drop); the planted front foot eases up
//           a little (handover) so the body settles onto the new foot
// The stance corners push through all three (easing only at the start and end of their stroke).
// Middle legs run WALK_MID_CYCLES full step cycles in the same time with a stride scaled to match.
// Returns false if keepGoing stopped it part-way (every joint holds where it is).
// Swing legs set all three joints each step, so the gait settles back into shape within a step from any
// paused position or after a change of direction.
bool halfCycle(const char *const swing[], const char *const stance[], motion::Gait g,
               bool (*keepGoing)() = nullptr) {
    using namespace motion;
    approach = approachFactor(g);
    bool forward = g == Gait::Forward;
    bool hasTouch = false;
    for (int k = 0; k < 4; k++)
        if (isCorner(swing[k]) && isFront(swing[k])) hasTouch = true;

    // signed: an unsigned time turns a negative (end - start) into a huge value -> wild joint moves
    int32_t tAir = scaled(cfg::WALK_SWING_MS), tDown = scaled(cfg::WALK_LIFT_MS);
    int32_t tTouch = hasTouch ? scaled(cfg::WALK_FRONT_TOUCHDOWN_MS) : 0;
    int32_t tLanded = tAir + tDown, total = tLanded + tTouch;
    Timeline tl;

    // ---- corner legs ----
    for (int k = 0; k < 4; k++) {
        const char *leg = swing[k];
        if (!isCorner(leg)) continue;
        bool front = isFront(leg);
        int groundY = cfg::STAND_PUSH_US;
        tl.add(leg, 'Y', 0, tAir, cfg::STAND_PUSH_US + walkLift(leg), EASE_BOTH);
        if (kneeStroke(leg, g)) {
            bool extended = front == forward;
            int i = servos::find(leg, 'K');
            int end = servos::joints[i].neutralUs + strokeK(leg, extended) * servos::joints[i].dir;
            int start = startPos(i, end);
            tl.addRaw(i, 0, tAir, start + (end - start) * tAir / tLanded, EASE_IN);
            tl.addRaw(i, tAir, tLanded, end, EASE_OUT);
            groundY = strokeY(leg, extended);
            tl.add(leg, 'X', 0, tAir, walkBaseX(leg, g), EASE_BOTH);  // X holds at base for knee strokes
        } else {
            tl.add(leg, 'X', 0, tAir, strideEnd(leg, strideSign(leg, g), g), EASE_BOTH);
            tl.add(leg, 'K', 0, tAir, walkKnee(leg), EASE_BOTH);
        }
        if (front) {
            tl.add(leg, 'Y', tAir, tLanded, groundY + cfg::WALK_FRONT_APPROACH_US, EASE_IN);
            tl.add(leg, 'Y', tLanded, total, groundY, EASE_SOFT);
        } else {
            tl.add(leg, 'Y', tAir, tLanded, groundY, EASE_BOTH);
        }
    }
    for (int k = 0; k < 4; k++) {
        const char *leg = stance[k];
        if (!isCorner(leg)) continue;
        // stroke end offsets per joint, then spread over the phases in proportion to their time
        char types[2];
        int offs[2], count = 0;
        bool extended = false;
        if (kneeStroke(leg, g)) {
            extended = isFront(leg) != forward;
            types[count] = 'K'; offs[count++] = strokeK(leg, extended);
            types[count] = 'Y'; offs[count++] = strokeY(leg, extended);
        } else {
            types[count] = 'X'; offs[count++] = strideEnd(leg, -strideSign(leg, g), g);
        }
        for (int c = 0; c < count; c++) {
            int i = servos::find(leg, types[c]);
            int end = servos::joints[i].neutralUs + offs[c] * servos::joints[i].dir;
            int start = startPos(i, end);
            tl.addRaw(i, 0, tAir, start + (end - start) * tAir / total, EASE_IN);
            if (!hasTouch) {
                tl.addRaw(i, tAir, total, end, EASE_OUT);
                continue;
            }
            tl.addRaw(i, tAir, tLanded, start + (end - start) * tLanded / total, EASE_NONE);
            if (types[c] == 'Y' && isFront(leg)) {  // handover
                int h = end + cfg::WALK_FRONT_HANDOVER_US * servos::joints[i].dir;
                tl.addRaw(i, tLanded, total, h, EASE_SOFT);
            } else {
                tl.addRaw(i, tLanded, total, end, EASE_OUT);
            }
        }
    }

    // ---- middle legs: WALK_MID_CYCLES step cycles, pairs alternating ----
    // With IK (forward / back) the feet stay on straight lines: stance strokes get femur / knee waypoints
    // as X moves, and each swing lands exactly on the line.
    bool ik = cfg::WALK_MID_IK && (g == Gait::Forward || g == Gait::Back);
    const char *const midLegs[] = {"FML", "FMR", "BML", "BMR"};
    int plannedX[4];
    Line lines[4];
    for (int k = 0; k < 4; k++) {
        int xj = servos::find(midLegs[k], 'X');
        plannedX[k] = startPos(xj, servos::joints[xj].neutralUs);
        if (ik) lines[k] = walkLine(midLegs[k], g);
    }
    auto slot = [&](const char *leg) {
        for (int k = 0; k < 4; k++)
            if (!strcmp(midLegs[k], leg)) return k;
        return 0;
    };
    int cycles = max(1, cfg::WALK_MID_CYCLES);
    int32_t sub = total / cycles;
    int32_t subAir = sub * cfg::WALK_SWING_MS / (cfg::WALK_SWING_MS + cfg::WALK_LIFT_MS);
    for (int c = 0; c < cycles; c++, midTurn++) {
        int32_t s0 = c * sub, s1 = (c == cycles - 1) ? total : s0 + sub;
        const char *const *up = (midTurn % 2 == 0) ? MID_1 : MID_2;
        const char *const *down = (midTurn % 2 == 0) ? MID_2 : MID_1;
        for (int k = 0; k < 2; k++) {
            // swing: lift, swing X, land (on the line with IK)
            const char *leg = up[k];
            int xj = servos::find(leg, 'X'), yj = servos::find(leg, 'Y'), kj = servos::find(leg, 'K');
            int xEnd = servos::joints[xj].neutralUs + strideEnd(leg, strideSign(leg, g), g) * servos::joints[xj].dir;
            int yLand = servos::joints[yj].neutralUs + cfg::STAND_PUSH_US * servos::joints[yj].dir;
            int kLand = servos::joints[kj].neutralUs + walkKnee(leg) * servos::joints[kj].dir;
            if (ik) onLine(leg, lines[slot(leg)], xEnd, yLand, kLand);
            tl.addRaw(yj, s0, s0 + subAir, yLand + walkLift(leg) * servos::joints[yj].dir, EASE_BOTH);
            tl.addRaw(xj, s0, s0 + subAir, xEnd, EASE_BOTH);
            if (ik || c == 0) tl.addRaw(kj, s0, s0 + subAir, kLand, EASE_BOTH);
            tl.addRaw(yj, s0 + subAir, s1, yLand, EASE_BOTH);
            plannedX[slot(leg)] = xEnd;

            // stance: push X back (femur / knee waypoints keep the foot on its line with IK)
            leg = down[k];
            xj = servos::find(leg, 'X');
            yj = servos::find(leg, 'Y');
            kj = servos::find(leg, 'K');
            int xStart = plannedX[slot(leg)];
            xEnd = servos::joints[xj].neutralUs + strideEnd(leg, -strideSign(leg, g), g) * servos::joints[xj].dir;
            tl.addRaw(xj, s0, s1, xEnd, EASE_BOTH);
            if (ik) {
                const int knots = max(1, cfg::WALK_MID_IK_KNOTS);
                int32_t tPrev = s0;
                for (int n = 1; n <= knots; n++) {
                    int32_t t = s0 + (s1 - s0) * n / knots;
                    int x = xStart + (int)lroundf((xEnd - xStart) * progress((float)n / knots, EASE_BOTH));
                    int y, kk;
                    onLine(leg, lines[slot(leg)], x, y, kk);
                    tl.addRaw(yj, tPrev, t, y, EASE_NONE);
                    tl.addRaw(kj, tPrev, t, kk, EASE_NONE);
                    tPrev = t;
                }
            }
            plannedX[slot(leg)] = xEnd;
        }
    }
    return tl.run(total, keepGoing);
}

// Lift one group, move its X and K to the given pose, lower it (feet never drag)
void placeGroup(const char *const group[], bool walkPose, motion::Gait g = motion::Gait::Forward) {
    Pose lift, place, lower;
    for (int k = 0; k < 4; k++) {
        lift.add(group[k], 'Y', cfg::STAND_PUSH_US + walkLift(group[k]));
        place.add(group[k], 'X', walkPose ? walkBaseX(group[k], g) : 0);
        place.add(group[k], 'K', walkPose ? walkKnee(group[k]) : cfg::STAND_TUCK_US);
        lower.add(group[k], 'Y', cfg::STAND_PUSH_US);
    }
    lift.run(cfg::WALK_LIFT_MS);
    place.run(cfg::WALK_SWING_MS);
    lower.run(cfg::WALK_LIFT_MS);
}

// ---- climb gait (caterpillar wave, front to back) ----
// Legs in wave order; group = the phase in which the leg steps
const char *const CLIMB_LEGS[8] = {"FL", "FR", "FML", "FMR", "BML", "BMR", "BL", "BR"};
const int CLIMB_GROUP[8] = {0, 0, 1, 1, 2, 2, 3, 4};
constexpr int CLIMB_PHASES = 5;
bool inClimb = false;     // legs are in the climb wave (paused or climbing)
int climbPhase = 0;       // group stepping next
float climbP[8] = {};     // stroke position per leg: 1 = front of the stroke, 0 = back

float lerpf(float a, float b, float t) { return a + (b - a) * t; }

// Physical pulses for a climbing leg on the ground at stroke position p (0 = back, 1 = front)
void climbPose(const char *leg, float p, int &xUs, int &yUs, int &kUs) {
    const Joint &jx = servos::joints[servos::find(leg, 'X')];
    const Joint &jy = servos::joints[servos::find(leg, 'Y')];
    const Joint &jk = servos::joints[servos::find(leg, 'K')];
    float x, y, k;  // offsets from neutral
    if (isCorner(leg) && isFront(leg)) {  // reach (front of stroke) -> pull (back)
        x = walkBaseX(leg);
        k = lerpf(cfg::WALK_FRONT_PULL_KNEE_US, cfg::WALK_FRONT_REACH_KNEE_US, p);
        y = lerpf(cfg::WALK_FRONT_PULL_Y_US, cfg::WALK_FRONT_REACH_Y_US, p);
    } else if (isCorner(leg)) {           // tucked (front of stroke) -> pushed out (back)
        x = walkBaseX(leg);
        k = lerpf(cfg::WALK_REAR_PUSH_KNEE_US, cfg::WALK_REAR_TUCK_KNEE_US, p);
        y = lerpf(cfg::WALK_REAR_PUSH_Y_US, cfg::WALK_REAR_TUCK_Y_US, p);
    } else {                              // middles swing X; femur / knee from the straight-line IK
        x = walkBaseX(leg) + cfg::CLIMB_STRIDE_MID_US * (2.0f * p - 1.0f);
        y = cfg::STAND_PUSH_US;
        k = walkKnee(leg);
    }
    xUs = jx.neutralUs + (int)lroundf(x * jx.dir);
    yUs = jy.neutralUs + (int)lroundf(y * jy.dir);
    kUs = jk.neutralUs + (int)lroundf(k * jk.dir);
    if (!isCorner(leg) && cfg::WALK_MID_IK) onLine(leg, walkLine(leg, motion::Gait::Forward), xUs, yUs, kUs);
    if (leg[0] == 'F') yUs -= cfg::CLIMB_FRONT_PUSH_US * jy.dir;  // front four push the body up
}

// The four front legs (FL FR FML FMR) lift as high as their Y joints allow; the rest CLIMB_LIFT_US
int climbLift(const char *leg) { return leg[0] == 'F' ? cfg::CLIMB_FRONT_LIFT_US : cfg::CLIMB_LIFT_US; }

// One wave phase: group j steps (lift, reach to the front of its stroke, set down) while every other leg
// pushes back a quarter of its stroke. Returns false if keepGoing paused it (state not advanced: resume
// replays the phase from wherever the legs are).
bool climbPhaseRun(int j, bool (*keepGoing)()) {
    using namespace motion;
    bool frontSteps = (j == 0);
    int32_t tSwing = scaled(cfg::CLIMB_SWING_MS), tLower = scaled(cfg::CLIMB_LOWER_MS);
    int32_t tTouch = frontSteps ? scaled(cfg::WALK_FRONT_TOUCHDOWN_MS) : 0;
    int32_t total = tSwing + tLower + tTouch;
    float next[8];
    Timeline tl;
    for (int i = 0; i < 8; i++) {
        const char *leg = CLIMB_LEGS[i];
        int xj = servos::find(leg, 'X'), yj = servos::find(leg, 'Y'), kj = servos::find(leg, 'K');
        int x, y, k;
        if (CLIMB_GROUP[i] == j) {
            next[i] = 1.0f;
            climbPose(leg, 1.0f, x, y, k);
            tl.addRaw(yj, 0, tSwing, y + climbLift(leg) * servos::joints[yj].dir, EASE_BOTH);
            tl.addRaw(xj, 0, tSwing, x, EASE_BOTH);
            tl.addRaw(kj, 0, tSwing, k, EASE_BOTH);
            if (isCorner(leg) && isFront(leg)) {
                tl.addRaw(yj, tSwing, tSwing + tLower, y + cfg::WALK_FRONT_APPROACH_US * servos::joints[yj].dir, EASE_IN);
                tl.addRaw(yj, tSwing + tLower, total, y, EASE_SOFT);
            } else {
                tl.addRaw(yj, tSwing, total, y, EASE_BOTH);
            }
        } else {
            next[i] = max(0.0f, climbP[i] - 1.0f / (CLIMB_PHASES - 1));
            climbPose(leg, next[i], x, y, k);
            tl.addRaw(xj, 0, total, x, EASE_NONE);
            tl.addRaw(yj, 0, total, y, EASE_NONE);
            tl.addRaw(kj, 0, total, k, EASE_NONE);
        }
    }
    if (!tl.run(total, keepGoing)) return false;
    for (int i = 0; i < 8; i++) climbP[i] = next[i];
    climbPhase = (j + 1) % CLIMB_PHASES;
    return true;
}

// From the stand pose into the wave: each leg at the stroke position it needs for phase 0
// (FL FR at the back, ready to step; then 1/4, 1/2, 3/4 along; BR at the front). Two tetrapod groups:
// lift, place, lower, so no foot drags.
void enterClimb() {
    for (int i = 0; i < 8; i++) {
        int g = CLIMB_GROUP[i];
        climbP[i] = g == 0 ? 0.0f : 1.0f - (float)(CLIMB_PHASES - 1 - g) / (CLIMB_PHASES - 1);
    }
    for (const char *const *group : {GROUP_A, GROUP_B}) {
        Pose lift, place, lower;
        for (int k = 0; k < 4; k++) {
            const char *leg = group[k];
            int i = 0;
            while (strcmp(CLIMB_LEGS[i], leg)) i++;
            int x, y, kk;
            climbPose(leg, climbP[i], x, y, kk);
            int yj = servos::find(leg, 'Y');
            lift.addRaw(yj, y + climbLift(leg) * servos::joints[yj].dir);
            place.addRaw(servos::find(leg, 'X'), x);
            place.addRaw(servos::find(leg, 'K'), kk);
            lower.addRaw(yj, y);
        }
        lift.run(cfg::WALK_LIFT_MS);
        place.run(cfg::WALK_SWING_MS * 2);
        lower.run(cfg::WALK_LIFT_MS);
    }
    climbPhase = 0;
    inClimb = true;
}

bool keyPressed() {
    if (!Term.available()) return false;
    while (Term.available()) Term.read();
    return true;
}

// Between steps: a key, an obstacle (ToF raises flags::CLIMB - loop() then climbs) or the caller's check
bool stopRequested(bool (*keepGoing)()) {
    tof::handle();  // ramps block loop(), so sample here (it also drives the front legs' obstacle lift)
    return keyPressed() || (cfg::TOF_AUTO_CLIMB && flags::test(flags::CLIMB)) || (keepGoing && !keepGoing());
}

}  // namespace

namespace motion {

// Fraction (0-1) of the move done at time t (0-1). Trapezoid speed: ramps up over EASE_FRACTION at the
// start (EASE_IN) and down over it at the end (EASE_OUT); EASE_SOFT = quadratic slow-down to a stop.
float progress(float t, uint8_t ease) {
    if (ease & EASE_SOFT) return 1.0f - (1.0f - t) * (1.0f - t);
    float a = (ease & EASE_IN) ? cfg::EASE_FRACTION : 0.0f;
    float d = (ease & EASE_OUT) ? cfg::EASE_FRACTION : 0.0f;
    float v = 1.0f / (1.0f - a / 2 - d / 2);  // peak speed so the total is still 1
    if (t < a) return v * t * t / (2 * a);
    if (t <= 1.0f - d) return v * (t - a / 2);
    float u = t - (1.0f - d);
    return v * ((1.0f - d) - a / 2) + v * (u - u * u / (2 * d));
}

bool ramp(const int joints[], const int targets[], int count, uint16_t ms, bool abortable, const uint8_t ease[]) {
    int start[servos::COUNT];
    for (int k = 0; k < count; k++) {
        int now = servos::position(joints[k]);
        start[k] = now ? now : targets[k];  // an output that was off jumps straight to target
    }
    int steps = max(1, ms / cfg::FRAME_MS);
    for (int s = 1; s <= steps; s++) {
        uint32_t t = millis();
        float time = (float)s / steps;
        for (int k = 0; k < count; k++) {
            float done = progress(time, ease ? ease[k] : EASE_BOTH);
            servos::moveRaw(joints[k], start[k] + (int)lroundf((targets[k] - start[k]) * done));
        }
        if (abortable && aborted()) return false;
        while (millis() - t < cfg::FRAME_MS) {}
    }
    return true;
}

void setSpeed(float s) { speed = constrain(s, 0.1f, 2.0f); }

bool rampType(char type, int offsetUs, uint16_t ms) {
    int joints[servos::LEG_COUNT], targets[servos::LEG_COUNT], n = 0;
    for (int i = 0; i < servos::COUNT; i++) {
        if (servos::joints[i].type != type) continue;
        joints[n] = i;
        targets[n] = servos::joints[i].neutralUs + offsetUs * servos::joints[i].dir;
        n++;
    }
    return ramp(joints, targets, n, ms);
}

bool rampAllCenter(uint16_t ms) {
    int joints[servos::COUNT], targets[servos::COUNT];
    for (int i = 0; i < servos::COUNT; i++) {
        joints[i] = i;
        targets[i] = servos::joints[i].neutralUs;
    }
    return ramp(joints, targets, servos::COUNT, ms);
}

bool standUp(bool stepMode) {
    inWalk = false;
    inClimb = false;
    Term.println("Stand up (any key aborts and holds).");
    if (stepMode && !waitStep("all joints to centre")) return false;
    Term.println("all joints to centre ...");
    if (!rampAllCenter(cfg::STAND_RAMP_MS)) return false;
    if (!stepMode) delay(cfg::STAND_PAUSE_MS);
    return step(stepMode, "all Y (lift) up", 'Y', cfg::STAND_LIFT_US, cfg::STAND_RAMP_MS) &&
           step(stepMode, "all K (knee) tuck toward body", 'K', cfg::STAND_TUCK_US, cfg::STAND_RAMP_MS) &&
           step(stepMode, "all Y (lift) DOWN - lifting the body", 'Y', cfg::STAND_PUSH_US, cfg::STAND_PUSH_MS) &&
           (Term.println("Standing."), true);
}

bool isStanding() {
    for (int i = 0; i < servos::COUNT; i++) {
        const Joint &j = servos::joints[i];
        int offset = j.type == 'Y' ? cfg::STAND_PUSH_US : j.type == 'K' ? cfg::STAND_TUCK_US : 0;
        if (j.type != 'X' && servos::position(i) != j.neutralUs + offset * j.dir) return false;
    }
    return true;
}

bool inWalkPose() { return inWalk; }

bool canWalk() { return inWalk || inClimb || isStanding(); }

void endWalk() {
    if (!inWalk) return;
    Term.println("Back to the stand pose.");
    placeGroup(GROUP_A, false);
    placeGroup(GROUP_B, false);
    inWalk = false;
}

void resetWalk() {
    inWalk = false;
    inClimb = false;
}

bool walk(Gait g, int cycles, bool (*keepGoing)()) {
    if (inClimb) endClimb();  // climb pose -> stand pose -> walk pose
    if (!canWalk()) {
        Term.println("Not in the stand pose - run 'stand' first.");
        return false;
    }
    if (obstacleStop(g)) {  // still blocked: say so once, not on every retry
        if (!blockWarned)
            Term.printf("Obstacle at %d mm - holding. Press LT (or 'climb') to carry on; back / turn still work.\n",
                        tof::distanceMm());
        blockWarned = true;
        return false;
    }
    blockWarned = false;
    const char *name = g == Gait::Forward ? "forward" : g == Gait::Back ? "back"
                     : g == Gait::TurnLeft ? "turn left" : "turn right";
    Term.printf("Walking %s", name);
    if (cycles) Term.printf(", %d cycles", cycles);
    Term.println(keepGoing ? " - release to pause (everything holds where it is)."
                           : " - any key stops after the current step.");

    if (!inWalk) {
        midTurn = 0;      // middle pair 1 (BML FMR) steps first, as group A's middle legs
        nextHalf = 0;     // group A swings first
        placeGroup(GROUP_A, true, g);  // into the walk pose, one group at a time
        placeGroup(GROUP_B, true, g);
        inWalk = true;
    }

    // Controller: pause mid-step the moment the stick is released and stay in the walk pose.
    // Console: finish the current step on a key / after the cycles, then back to the stand pose.
    // Both: walking forward stops and holds at an obstacle (TOF_STOP_MM) until climb is pressed.
    walkGait = g;
    walkKeepGoing = keepGoing;
    for (int halves = 0; cycles == 0 || halves < cycles * 2; halves++) {
        const char *const *swing = nextHalf == 0 ? GROUP_A : GROUP_B;
        const char *const *stance = nextHalf == 0 ? GROUP_B : GROUP_A;
        int savedMid = midTurn;
        obstacleHit = false;
        if (obstacleStop(g) || !halfCycle(swing, stance, g, walkContinue)) {
            midTurn = savedMid;  // resume replays this half-step from wherever the legs are
            if (obstacleHit || obstacleStop(g)) {
                Term.printf("Obstacle at %d mm - holding. Press LT (or 'climb') to carry on; back / turn still work.\n",
                            tof::distanceMm());
                blockWarned = true;
            } else
                Term.println("Paused - holding. Push the stick to carry on, A = stand pose, B = sit.");
            return true;
        }
        nextHalf ^= 1;
        if (stopRequested(keepGoing)) {
            if (keepGoing) {
                Term.println("Paused - holding. Push the stick to carry on, A = stand pose, B = sit.");
                return true;
            }
            break;
        }
    }
    endWalk();
    Term.println(flags::test(flags::CLIMB) ? "Stopped - obstacle ahead, standing." : "Stopped - standing.");
    return true;
}

bool blocked(Gait g) { return obstacleStop(g); }

bool inClimbPose() { return inClimb; }

void endClimb() {
    if (!inClimb) return;
    Term.println("Back to the stand pose.");
    placeGroup(GROUP_A, false);
    placeGroup(GROUP_B, false);
    inClimb = false;
}

bool climb(int cycles, bool (*keepGoing)()) {
    climbCleared = true;  // also unlocks forward walking past the obstacle stop
    if (inWalk) endWalk();
    if (!inClimb) {
        if (!isStanding()) {
            Term.println("Not in the stand pose - run 'stand' first.");
            return false;
        }
        Term.println("Climb: staggering the legs into the wave.");
        enterClimb();
    }
    Term.printf("Climbing (obstacle %d mm)%s\n", tof::distanceMm(),
                keepGoing ? " - release LT to pause." : "");
    for (int n = 0; cycles == 0 || n < cycles * CLIMB_PHASES; n++) {
        if (!climbPhaseRun(climbPhase, keepGoing) || (keepGoing && !keepGoing())) {
            Term.println("Climb paused - holding. Hold LT to carry on, A = stand pose, B = sit.");
            return true;
        }
        if (!keepGoing && keyPressed()) break;
    }
    if (!keepGoing) endClimb();
    return true;
}

bool sitDown() {
    inWalk = false;
    inClimb = false;
    Term.println("Sit down (any key aborts and holds).");
    return step(false, "Y to centre - lowering the body", 'Y', 0, cfg::STAND_PUSH_MS) &&
           step(false, "K to centre", 'K', 0, cfg::STAND_RAMP_MS) &&
           step(false, "X to centre", 'X', 0, cfg::STAND_RAMP_MS) && (Term.println("Sitting."), true);
}

}  // namespace motion
