#include "pad.h"

#include <Arduino.h>
// Defines static BLE globals - include it in this file only
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>

#include "config.h"
#include "motion.h"
#include "term.h"

namespace {

XboxSeriesXControllerESP32_asukiaaa::Core xbox(cfg::PAD_ADDRESS);

bool wasConnected = false;
bool prevA = false, prevB = false, prevLT = false;
bool ltWarned = false;  // 'needs the stand pose' already printed for this LT hold

enum class Dir { None, Forward, Back, Left, Right };

// Stick axis as -1..+1 (0 = centre). The controller reports 0 at top / left.
float axis(uint16_t raw) {
    const float half = XboxControllerNotificationParser::maxJoy / 2.0f;
    return (raw - half) / half;
}

// Direction asked for: D-pad first, then the left stick (dominant axis, past the dead zone)
Dir requested() {
    const auto &n = xbox.xboxNotif;
    if (n.btnDirUp) return Dir::Forward;
    if (n.btnDirDown) return Dir::Back;
    if (n.btnDirLeft) return Dir::Left;
    if (n.btnDirRight) return Dir::Right;

    float x = axis(n.joyLHori);
    float y = -axis(n.joyLVert) * cfg::PAD_STICK_Y_SIGN;  // + = stick pushed forward
    if (fabsf(x) < cfg::PAD_DEADZONE && fabsf(y) < cfg::PAD_DEADZONE) return Dir::None;
    if (fabsf(y) >= fabsf(x)) return y > 0 ? Dir::Forward : Dir::Back;
    return x > 0 ? Dir::Right : Dir::Left;
}

motion::Gait gaitFor(Dir d) {
    switch (d) {
        case Dir::Back: return motion::Gait::Back;
        case Dir::Left: return motion::Gait::TurnLeft;
        case Dir::Right: return motion::Gait::TurnRight;
        default: return motion::Gait::Forward;
    }
}

// Left trigger past a quarter travel; climb speed from how far it is pulled (WALK_MIN_SPEED .. 1)
bool ltPressed() { return xbox.xboxNotif.trigLT > XboxControllerNotificationParser::maxTrig / 4; }
float ltSpeed() {
    float f = (xbox.xboxNotif.trigLT - XboxControllerNotificationParser::maxTrig / 4.0f) /
              (XboxControllerNotificationParser::maxTrig * 0.75f);
    return cfg::WALK_MIN_SPEED + (1.0f - cfg::WALK_MIN_SPEED) * constrain(f, 0.0f, 1.0f);
}

// Walk speed from how far the stick is pushed: WALK_MIN_SPEED just past the dead zone, 1 at full.
// The D-pad always walks at full speed.
float stickSpeed() {
    const auto &n = xbox.xboxNotif;
    if (n.btnDirUp || n.btnDirDown || n.btnDirLeft || n.btnDirRight) return 1.0f;
    float m = max(fabsf(axis(n.joyLHori)), fabsf(axis(n.joyLVert)));
    float f = (m - cfg::PAD_DEADZONE) / (1.0f - cfg::PAD_DEADZONE);
    return cfg::WALK_MIN_SPEED + (1.0f - cfg::WALK_MIN_SPEED) * constrain(f, 0.0f, 1.0f);
}

bool ready() { return xbox.isConnected() && !xbox.isWaitingForFirstNotification(); }

// Climbing continues while LT is held
bool keepClimbing() {
    xbox.onLoop();
    if (!ready() || !ltPressed()) return false;
    motion::setSpeed(ltSpeed());
    return true;
}

// Walking continues while the same direction is held and the controller stays connected
Dir walking = Dir::None;
bool keepWalking() {
    xbox.onLoop();
    if (!ready() || requested() != walking) return false;
    motion::setSpeed(stickSpeed());  // checked every frame; the new speed applies from the next half step
    return true;
}

}  // namespace

namespace pad {

void begin() {
    if (!cfg::PAD_ENABLED) return;
    xbox.begin();
    Term.println("Controller: scanning - hold the pair button on the Xbox controller.");
}

bool connected() { return cfg::PAD_ENABLED && ready(); }

void handle() {
    if (!cfg::PAD_ENABLED) return;
    xbox.onLoop();

    bool now = ready();
    if (now != wasConnected) {
        wasConnected = now;
        if (now) {
            Term.printf("\nController connected (%s).\n> ", xbox.buildDeviceAddressStr().c_str());
            // Ignore buttons already held at connect
            prevA = xbox.xboxNotif.btnA;
            prevB = xbox.xboxNotif.btnB;
            prevLT = ltPressed();
        } else {
            Term.print("\nController disconnected.\n> ");
        }
    }
    if (!now) return;

    const auto &n = xbox.xboxNotif;
    bool pressA = n.btnA && !prevA, pressB = n.btnB && !prevB;
    bool lt = ltPressed();
    prevA = n.btnA;
    prevB = n.btnB;
    prevLT = lt;

    if (pressA) {
        if (motion::inClimbPose()) {
            Term.println("\n[pad] A: stand pose");
            motion::endClimb();
        } else if (motion::inWalkPose()) {
            Term.println("\n[pad] A: stand pose");
            motion::endWalk();
        } else {
            Term.println("\n[pad] A: stand");
            motion::standUp(false);
        }
        Term.print("> ");
    } else if (pressB) {
        Term.println("\n[pad] B: sit");
        motion::sitDown();
        Term.print("> ");
    } else if (lt && !motion::canWalk()) {
        if (!ltWarned) Term.print("\n[pad] LT: climb needs the stand pose (A)\n> ");
        ltWarned = true;
    } else if (lt) {
        ltWarned = false;
        Term.println("\n[pad] LT: climb (release to pause; pull harder = faster)");
        motion::setSpeed(ltSpeed());
        motion::climb(0, keepClimbing);
        motion::setSpeed(1.0f);
        prevA = n.btnA;
        prevB = n.btnB;
        prevLT = ltPressed();
        Term.print("> ");
    } else {
        Dir d = requested();
        if (d != Dir::None && motion::canWalk() && !motion::blocked(gaitFor(d))) {
            walking = d;
            Term.println("\n[pad] walk (release to pause; push further = faster)");
            motion::setSpeed(stickSpeed());
            motion::walk(gaitFor(d), 0, keepWalking);
            motion::setSpeed(1.0f);
            walking = Dir::None;
            prevA = n.btnA;  // presses during the walk don't queue a stand / sit
            prevB = n.btnB;
            Term.print("> ");
        }
    }
}

void printStatus() {
    if (!cfg::PAD_ENABLED) {
        Term.println("Controller: disabled (cfg::PAD_ENABLED)");
        return;
    }
    if (!xbox.isConnected()) {
        Term.println("Controller: not connected (scanning - hold the pair button)");
        return;
    }
    const auto &n = xbox.xboxNotif;
    Term.printf("Controller: %s, battery %u%%\n", xbox.buildDeviceAddressStr().c_str(), xbox.battery);
    Term.printf("  left stick x %+.2f  y %+.2f (speed %.2f)   A %d  B %d  LT %u   D-pad U%d D%d L%d R%d\n",
                axis(n.joyLHori), -axis(n.joyLVert) * cfg::PAD_STICK_Y_SIGN, stickSpeed(), n.btnA, n.btnB,
                n.trigLT, n.btnDirUp, n.btnDirDown, n.btnDirLeft, n.btnDirRight);
}

}  // namespace pad
