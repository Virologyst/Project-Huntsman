#include "motion.h"

#include "config.h"
#include "pwm.h"
#include "servo_map.h"

namespace {

bool aborted() {
    if (!Serial.available()) return false;
    while (Serial.available()) Serial.read();
    Serial.println("\nAborted - holding position. 'limp' to release, 'sit' to lower.");
    return true;
}

// Step mode: wait for Enter (q cancels)
bool waitStep(const char *what) {
    Serial.printf("Next: %s - Enter to go, q to stop: ", what);
    while (true) {
        if (!Serial.available()) { delay(5); continue; }
        char c = Serial.read();
        if (c == 'q' || c == 'Q') {
            while (Serial.available()) Serial.read();
            Serial.println("\nStopped - holding position.");
            return false;
        }
        if (c == '\r' || c == '\n') {
            delay(20);
            while (Serial.available()) Serial.read();
            Serial.println();
            return true;
        }
    }
}

bool step(bool stepMode, const char *what, char type, int offsetUs, uint16_t ms) {
    if (stepMode && !waitStep(what)) return false;
    Serial.printf("%s ...\n", what);
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
    int n = 0;

    void add(const char *leg, char type, int offsetUs) {
        int i = servos::find(leg, type);
        joints[n] = i;
        targets[n] = cfg::STAND_CENTER_US + offsetUs * servos::joints[i].dir;
        n++;
    }
    void run(uint16_t ms) { motion::ramp(joints, targets, n, ms, false); }  // a step always completes
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

// Swing group: lift, swing to +stride while the stance group pushes to -stride, lower
void halfCycle(const char *const swing[], const char *const stance[], motion::Gait g) {
    Pose lift, move, lower;
    for (int k = 0; k < 4; k++) {
        lift.add(swing[k], 'Y', cfg::STAND_PUSH_US + cfg::WALK_LIFT_US);
        move.add(swing[k], 'X', cfg::WALK_STRIDE_US * strideSign(swing[k], g));
        move.add(stance[k], 'X', -cfg::WALK_STRIDE_US * strideSign(stance[k], g));
        lower.add(swing[k], 'Y', cfg::STAND_PUSH_US);
    }
    lift.run(cfg::WALK_LIFT_MS);
    move.run(cfg::WALK_SWING_MS);
    lower.run(cfg::WALK_LIFT_MS);
}

// Bring each group's swing back to centre, one group at a time, ending in the stand pose
void recentre(const char *const group[]) {
    Pose lift, centre, lower;
    for (int k = 0; k < 4; k++) {
        lift.add(group[k], 'Y', cfg::STAND_PUSH_US + cfg::WALK_LIFT_US);
        centre.add(group[k], 'X', 0);
        lower.add(group[k], 'Y', cfg::STAND_PUSH_US);
    }
    lift.run(cfg::WALK_LIFT_MS);
    centre.run(cfg::WALK_SWING_MS / 2);
    lower.run(cfg::WALK_LIFT_MS);
}

bool keyPressed() {
    if (!Serial.available()) return false;
    while (Serial.available()) Serial.read();
    return true;
}

}  // namespace

namespace motion {

bool ramp(const int joints[], const int targets[], int count, uint16_t ms, bool abortable) {
    int start[servos::COUNT];
    for (int k = 0; k < count; k++) {
        int now = servos::position(joints[k]);
        start[k] = now ? now : targets[k];  // an output that was off jumps straight to target
    }
    int steps = max(1, ms / cfg::FRAME_MS);
    for (int s = 1; s <= steps; s++) {
        uint32_t t = millis();
        for (int k = 0; k < count; k++)
            servos::moveRaw(joints[k], start[k] + (targets[k] - start[k]) * s / steps);
        if (abortable && aborted()) return false;
        while (millis() - t < cfg::FRAME_MS) {}
    }
    return true;
}

bool rampType(char type, int offsetUs, uint16_t ms) {
    int joints[servos::LEG_COUNT], targets[servos::LEG_COUNT], n = 0;
    for (int i = 0; i < servos::COUNT; i++) {
        if (servos::joints[i].type != type) continue;
        joints[n] = i;
        targets[n] = cfg::STAND_CENTER_US + offsetUs * servos::joints[i].dir;
        n++;
    }
    return ramp(joints, targets, n, ms);
}

bool rampAllCenter(uint16_t ms) {
    int joints[servos::COUNT], targets[servos::COUNT];
    for (int i = 0; i < servos::COUNT; i++) {
        joints[i] = i;
        targets[i] = cfg::STAND_CENTER_US;
    }
    return ramp(joints, targets, servos::COUNT, ms);
}

bool standUp(bool stepMode) {
    Serial.println("Stand up (any key aborts and holds).");
    if (stepMode && !waitStep("all joints to centre")) return false;
    Serial.println("all joints to centre ...");
    if (!rampAllCenter(cfg::STAND_RAMP_MS)) return false;
    if (!stepMode) delay(cfg::STAND_PAUSE_MS);
    return step(stepMode, "all Y (lift) up", 'Y', cfg::STAND_LIFT_US, cfg::STAND_RAMP_MS) &&
           step(stepMode, "all K (knee) tuck toward body", 'K', cfg::STAND_TUCK_US, cfg::STAND_RAMP_MS) &&
           step(stepMode, "all Y (lift) DOWN - lifting the body", 'Y', cfg::STAND_PUSH_US, cfg::STAND_PUSH_MS) &&
           (Serial.println("Standing."), true);
}

bool isStanding() {
    for (int i = 0; i < servos::COUNT; i++) {
        const Joint &j = servos::joints[i];
        int offset = j.type == 'Y' ? cfg::STAND_PUSH_US : j.type == 'K' ? cfg::STAND_TUCK_US : 0;
        if (j.type != 'X' && servos::position(i) != cfg::STAND_CENTER_US + offset * j.dir) return false;
    }
    return true;
}

bool walk(Gait g, int cycles) {
    if (!isStanding()) {
        Serial.println("Not in the stand pose - run 'stand' first.");
        return false;
    }
    const char *name = g == Gait::Forward ? "forward" : g == Gait::Back ? "back"
                     : g == Gait::TurnLeft ? "turn left" : "turn right";
    Serial.printf("Walking %s", name);
    if (cycles) Serial.printf(", %d cycles", cycles);
    Serial.println(" - any key stops after the current step.");

    bool stop = false;
    for (int c = 0; !stop && (cycles == 0 || c < cycles); c++) {
        halfCycle(GROUP_A, GROUP_B, g);
        stop = keyPressed();
        if (stop) break;
        halfCycle(GROUP_B, GROUP_A, g);
        stop = keyPressed();
    }
    recentre(GROUP_A);
    recentre(GROUP_B);
    Serial.println("Stopped - standing.");
    return true;
}

bool sitDown() {
    Serial.println("Sit down (any key aborts and holds).");
    return step(false, "Y to centre - lowering the body", 'Y', 0, cfg::STAND_PUSH_MS) &&
           step(false, "K to centre", 'K', 0, cfg::STAND_RAMP_MS) &&
           step(false, "X to centre", 'X', 0, cfg::STAND_RAMP_MS) && (Serial.println("Sitting."), true);
}

}  // namespace motion
