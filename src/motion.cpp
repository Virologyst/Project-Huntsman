#include "motion.h"

#include "config.h"
#include "flags.h"
#include "term.h"
#include "pwm.h"
#include "servo_map.h"
#include "tof.h"

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
    if (!(isCorner(leg) && isFront(leg))) return cfg::WALK_LIFT_US;
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

float speed = 1.0f;  // motion::setSpeed
int32_t scaled(uint16_t ms) { return max<int32_t>(cfg::FRAME_MS, (int32_t)(ms / speed)); }

int startPos(int joint, int fallback) { return servos::position(joint) ? servos::position(joint) : fallback; }

// Half a gait cycle in up to three ramps. The stance legs push through all of them (speeding up at the
// start of their stroke and slowing at the end, but not at the joins), so the body keeps moving:
//   air   - swing legs lift and swing X to the next stroke start
//   down  - swing legs lower; front feet stop WALK_FRONT_APPROACH_US above the ground
//   touch - front feet set down slowly (servos can't follow a fast drop), and the planted front foot eases
//           up a little (handover) so the body settles onto the new foot
// The big corner-leg knee strokes (750-1100 us) are spread over air + down: too far for the air ramp alone.
void halfCycle(const char *const swing[], const char *const stance[], motion::Gait g) {
    using namespace motion;
    Pose air, down, touch, swingK, stanceEnd, handover;
    bool forward = g == Gait::Forward;
    for (int k = 0; k < 4; k++) {
        // Swing leg (walking forward a front leg sets down extended and a back leg tucked; back, the reverse)
        air.add(swing[k], 'Y', cfg::STAND_PUSH_US + walkLift(swing[k]), EASE_BOTH);
        bool frontCorner = isCorner(swing[k]) && isFront(swing[k]);
        int groundY = cfg::STAND_PUSH_US;
        if (kneeStroke(swing[k], g)) {
            bool extended = isFront(swing[k]) == forward;
            swingK.add(swing[k], 'K', strokeK(swing[k], extended));
            groundY = strokeY(swing[k], extended);
        } else {
            air.add(swing[k], 'X', strideEnd(swing[k], strideSign(swing[k], g), g), EASE_BOTH);
        }
        if (frontCorner) {
            down.add(swing[k], 'Y', groundY + cfg::WALK_FRONT_APPROACH_US, EASE_IN);
            touch.add(swing[k], 'Y', groundY, EASE_SOFT);
        } else {
            down.add(swing[k], 'Y', groundY, EASE_BOTH);
        }
        // Stance leg: on the ground to the end of its stroke
        if (kneeStroke(stance[k], g)) {
            bool extended = isFront(stance[k]) != forward;
            stanceEnd.add(stance[k], 'K', strokeK(stance[k], extended));
            stanceEnd.add(stance[k], 'Y', strokeY(stance[k], extended));
            if (isFront(stance[k]))
                handover.add(stance[k], 'Y', strokeY(stance[k], extended) + cfg::WALK_FRONT_HANDOVER_US);
        } else {
            stanceEnd.add(stance[k], 'X', strideEnd(stance[k], -strideSign(stance[k], g), g));
        }
    }

    // signed: an unsigned time here turns a negative (end - start) into a huge value -> wild joint moves
    bool hasTouch = touch.n || handover.n;
    int32_t tAir = scaled(cfg::WALK_SWING_MS), tDown = scaled(cfg::WALK_LIFT_MS);
    int32_t tTouch = hasTouch ? scaled(cfg::WALK_FRONT_TOUCHDOWN_MS) : 0;
    int32_t total = tAir + tDown + tTouch;

    // Swing knees: speed up in air, slow down at the end of down
    for (int k = 0; k < swingK.n; k++) {
        int j = swingK.joints[k], end = swingK.targets[k], start = startPos(j, end);
        air.addRaw(j, start + (end - start) * tAir / (tAir + tDown), EASE_IN);
        down.addRaw(j, end, EASE_OUT);
    }
    // Stance: spread in proportion to the ramp times; ease only at the stroke's start and end
    for (int k = 0; k < stanceEnd.n; k++) {
        int j = stanceEnd.joints[k], end = stanceEnd.targets[k], start = startPos(j, end);
        air.addRaw(j, start + (end - start) * tAir / total, EASE_IN);
        if (hasTouch) {
            down.addRaw(j, start + (end - start) * (tAir + tDown) / total, EASE_NONE);
            int h = handover.find(j);
            if (h >= 0) touch.addRaw(j, handover.targets[h], EASE_SOFT);
            else touch.addRaw(j, end, EASE_OUT);
        } else {
            down.addRaw(j, end, EASE_OUT);
        }
    }
    air.run(tAir);
    down.run(tDown);
    if (hasTouch) touch.run(tTouch);
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

bool walk(Gait g, int cycles, bool (*keepGoing)()) {
    if (!isStanding()) {
        Term.println("Not in the stand pose - run 'stand' first.");
        return false;
    }
    const char *name = g == Gait::Forward ? "forward" : g == Gait::Back ? "back"
                     : g == Gait::TurnLeft ? "turn left" : "turn right";
    Term.printf("Walking %s", name);
    if (cycles) Term.printf(", %d cycles", cycles);
    Term.println(keepGoing ? " - release (or any key) stops after the current step."
                           : " - any key stops after the current step.");

    placeGroup(GROUP_A, true, g);  // into the walk pose, one group at a time
    placeGroup(GROUP_B, true, g);
    bool stop = stopRequested(keepGoing);
    for (int c = 0; !stop && (cycles == 0 || c < cycles); c++) {
        halfCycle(GROUP_A, GROUP_B, g);
        stop = stopRequested(keepGoing);
        if (stop) break;
        halfCycle(GROUP_B, GROUP_A, g);
        stop = stopRequested(keepGoing);
    }
    placeGroup(GROUP_A, false);  // back to the stand pose
    placeGroup(GROUP_B, false);
    Term.println(flags::test(flags::CLIMB) ? "Stopped - obstacle ahead, standing." : "Stopped - standing.");
    return true;
}

bool climb() {
    if (!isStanding()) {
        Term.println("Not in the stand pose - run 'stand' first.");
        return false;
    }
    Term.printf("Climb (obstacle %d mm) - any key aborts and holds.\n", tof::distanceMm());

    // TODO: climbing sequence. Building blocks (all in this file):
    //   Pose p; p.add("FL", 'Y', offset); ... p.run(ms);   - one coordinated ramp, offsets from neutral,
    //                                                         + = lift up / knee up / swing forward
    //   placeGroup(GROUP_A, true/false)                     - lift, place, lower a tetrapod group
    //   rampType('Y', offset, ms)                           - one joint type on all 8 legs
    //   cfg::CLIMB_LIFT_US, cfg::CLIMB_RAMP_MS              - tuning constants (config.h)
    // Must finish in the stand pose (isStanding() true) so walking can resume; return false if aborted.

    Term.println("Climb sequence not written yet - standing.");
    return true;
}

bool sitDown() {
    Term.println("Sit down (any key aborts and holds).");
    return step(false, "Y to centre - lowering the body", 'Y', 0, cfg::STAND_PUSH_MS) &&
           step(false, "K to centre", 'K', 0, cfg::STAND_RAMP_MS) &&
           step(false, "X to centre", 'X', 0, cfg::STAND_RAMP_MS) && (Term.println("Sitting."), true);
}

}  // namespace motion
