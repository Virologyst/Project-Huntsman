// Huntsman - hardware constants for the Brain Box
#pragma once

#include <Arduino.h>

namespace cfg {

constexpr uint32_t SERIAL_BAUD = 115200;

// Wi-Fi (credentials in include/secrets.h). OTA uploads and the console are reached at HOSTNAME.local
constexpr const char *HOSTNAME = "huntsman";
constexpr uint16_t CONSOLE_PORT = 23;  // raw TCP console (PlatformIO monitor: socket://huntsman.local:23)

// I2C to the PCA9685 boards
constexpr uint8_t SDA_PIN = 8;
constexpr uint8_t SCL_PIN = 9;

// Board 1 = 0x40 (left legs), board 2 = 0x41 (right legs) on the rebuilt robot
constexpr int BOARD_COUNT = 2;
constexpr uint8_t BOARD_ADDR[BOARD_COUNT] = {0x40, 0x41};

constexpr uint32_t DEFAULT_OSC_HZ = 25000000;  // PCA9685 nominal internal clock
constexpr float DEFAULT_FRAME_HZ = 50.0f;      // standard servo frame rate

// On boot, every output (all 32 channels) is driven to this pulse, paced to avoid a current spike.
// Set to 0 to boot with all outputs off (servos limp).
constexpr int BOOT_PULSE_US = 1500;
constexpr uint16_t BOOT_PACE_MS = 20;

// ---------- Stand-up sequence ----------
// Offsets are from each joint's neutral (servo_map, 'setneutral') and follow its dir:
// + = lift up, knee up, swing forward.
// Sequence: all joints centre -> all Y up -> all K tucked -> all Y down (lifts the body).
constexpr int STAND_LIFT_US = 200;    // Y up before tucking knees (doubled from 100)
constexpr int STAND_TUCK_US = -300;   // K toward the body (verified: + moves the knees outward)
constexpr int STAND_PUSH_US = -100;   // Y down: feet push the body off the ground
constexpr uint16_t STAND_RAMP_MS = 500;   // centre, lift and tuck ramps
constexpr uint16_t STAND_PUSH_MS = 750;   // the loaded push, slower
constexpr uint16_t STAND_PAUSE_MS = 500;  // between steps
constexpr uint16_t FRAME_MS = 20;         // ramp update interval (one servo frame)

// ---------- Walking (joint-space alternating tetrapod) ----------
// Group A = FL BML FMR BR, group B = FML BL FR BMR. A walk starts by moving each group from the stand
// pose into the walk pose (lift, place, lower) and ends by returning to the stand pose. Each half-cycle:
// swing group lifts, swings X to base + stride while the stance group pushes X to base - stride, lowers.
// Walk pose (offsets from neutral, + = knee up / swing forward):
constexpr int WALK_MID_KNEE_US = -50;       // middle legs: knee out from the stand's -300 -> feet reach out (was -100)
constexpr int WALK_CORNER_KNEE_US = -200;   // corner legs: straightened a little from -300
constexpr int WALK_FRONT_SPREAD_US = 400;   // FL FR X base toward the head (~54 deg; was 220, +~25 deg 2026-10-04)
// Front legs walking forward / back do not stride on X: X holds at the base above and the foot reaches
// straight out and pulls the body to it with K and Y. Swing: lift, knee out to REACH_KNEE, lower to
// REACH_Y. Stance: knee closes to PULL_KNEE while Y moves to PULL_Y so the foot stays on the ground.
// GUESSED values, no leg geometry yet - tune PULL_Y until the planted foot neither lifts nor jacks the body.
// Knee travel doubled 2026-10-03 (250 -> 500 us): the tucked end stays at the stand tuck, the reach end
// goes 200 past the knee's neutral; the Y change is doubled with it (40 -> 80).
constexpr int WALK_FRONT_REACH_KNEE_US = 350;   // knee out: foot far ahead (was 200; reaches further 2026-10-04)
constexpr int WALK_FRONT_REACH_Y_US = -250;     // Y when the reached foot is set down (was -165: foot stopped ~25 mm up)
constexpr int WALK_FRONT_PULL_KNEE_US = -750;   // knee closed at the end of the pull (was -550; stroke 750 -> 1100 us. FR knee min is -768)
constexpr int WALK_FRONT_PULL_Y_US = 40;        // Y at the end of the pull (was 10: lift back up slightly on the pull)
// Handover: as one front foot lowers onto the ground, the other (planted, end of its pull) eases Y up by
// this much at the same time, so the body is lowered onto the new foot instead of falling onto it.
constexpr int WALK_FRONT_HANDOVER_US = 60;
// Front touchdown: the lower ramp stops this far above the set-down Y, then the last part is a slow eased
// ramp (the handover runs in it too). A 650 us drop in 200 ms outruns the servos, so easing alone still
// landed at full speed and punched the toes.
constexpr int WALK_FRONT_APPROACH_US = 100;
constexpr uint16_t WALK_FRONT_TOUCHDOWN_MS = 300;
constexpr int WALK_BACK_SPREAD_US = 320;    // BL BR X base toward the tail (~43 deg; 400 scraped the battery sides)
// Back legs mirror the front legs: X holds, and they push the body forward with K and Y. Swing: lift,
// knee closes to TUCK_KNEE, lower to TUCK_Y (foot set down close in). Stance: knee opens to PUSH_KNEE
// while Y moves to PUSH_Y. GUESSED values, same starting numbers as the front - tune separately.
constexpr int WALK_REAR_TUCK_KNEE_US = -550;    // knee closed at the start of the push (was -300: stroke +50%, inward end only)
constexpr int WALK_REAR_TUCK_Y_US = -20;        // Y when the tucked foot is set down (was -60, scaled with the stroke)
constexpr int WALK_REAR_PUSH_KNEE_US = 200;     // knee out at the end of the push (was -50, travel doubled)
constexpr int WALK_REAR_PUSH_Y_US = -140;       // Y at the end of the push (was -100)
constexpr int WALK_BACK_MID_SPREAD_US = 60; // BML BMR X base toward the tail: clears FML/FMR at the longer stride
constexpr int WALK_LIFT_US = 350;           // Y up from the stand pose while swinging (was 150: ~10 mm clearance, too low to step over things)
constexpr int WALK_FRONT_LIFT_US = 500;      // FL FR lift higher than the rest (was 300; raised with WALK_LIFT_US)
constexpr int WALK_STRIDE_MID_US = 150;     // middle legs X each way from base (~20 deg)
constexpr int WALK_FML_EXTRA_REACH_US = 46;  // FML only: forward end of its step 30% further (150 -> 196; was 23)
constexpr int WALK_STRIDE_CORNER_US = 150;  // corner legs X each way from base (turns only)
// Turning, the back corners stride around a base shifted this far forward: their tail-most point is then
// the walking base (320). Unshifted (base - stride = 470 toward the tail) they hit the battery and the
// ESP32 browned out.
constexpr int WALK_TURN_REAR_SHIFT_US = 150;
constexpr uint16_t WALK_LIFT_MS = 200;   // 'down' ramp: swing legs lower (also lift/lower in and out of the walk pose)
constexpr uint16_t WALK_SWING_MS = 400;  // 'air' ramp: swing legs lift + swing together; stance legs push through every ramp

// Run the stand-up sequence automatically after boot (after BOOT_STAND_DELAY_MS; any key cancels)
constexpr bool BOOT_STAND = true;
constexpr uint16_t BOOT_STAND_DELAY_MS = 3000;

// ---------- Xbox controller (Bluetooth LE, docs/controller.md) ----------
constexpr bool PAD_ENABLED = true;
// "" = pair with the first Xbox controller found in pairing mode; or lock to one, e.g. "44:16:22:5e:b2:d4"
constexpr const char *PAD_ADDRESS = "";
constexpr float PAD_DEADZONE = 0.5f;   // stick must pass half travel to start walking
constexpr int PAD_STICK_Y_SIGN = 1;    // set to -1 if stick-forward walks backward ('pad' shows y)

// ---------- Time-of-flight range sensor (VL53L0X, forward-facing, own I2C bus on GPIO 17 / 18) ----------
constexpr bool TOF_ENABLED = true;
constexpr uint8_t TOF_ADDR = 0x29;            // VL53L0X default (fixed unless XSHUT is driven)
constexpr uint8_t TOF_SDA_PIN = 17;           // own I2C bus (Wire1), separate from the PCA boards on 8 / 9
constexpr uint8_t TOF_SCL_PIN = 18;
constexpr uint16_t TOF_PERIOD_MS = 50;        // continuous ranging interval
constexpr uint32_t TOF_BUDGET_US = 33000;     // per-sample timing budget (longer = more accurate, max ~PERIOD)
constexpr int TOF_CLIMB_MM = 300;             // closer than this raises flags::CLIMB ...
constexpr int TOF_CLEAR_MM = 400;             // ... and it can't raise again until the range passes this
constexpr int TOF_MAX_MM = 2000;              // beyond this = nothing in range (VL53L0X is ~1.2 m indoors)
constexpr bool TOF_AUTO_CLIMB = true;         // loop() runs motion::climb() on the flag (only while standing)

// ---------- Climb sequence (motion::climb, docs/motion.md) ----------
// TODO: sequence not written yet - add its offsets / ramps here as it takes shape
constexpr int CLIMB_LIFT_US = 300;            // Y up for a leg stepping onto the obstacle
constexpr uint16_t CLIMB_RAMP_MS = 400;

// Absolute pulse limits for any output, regardless of joint calibration
constexpr int HARD_MIN_US = 400;
constexpr int HARD_MAX_US = 2600;

}  // namespace cfg
