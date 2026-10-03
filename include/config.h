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
constexpr int WALK_FRONT_SPREAD_US = 220;   // FL FR X base toward the head (~30 deg): front feet come together
// Front legs walking forward / back do not stride on X: X holds at the base above and the foot reaches
// straight out and pulls the body to it with K and Y. Swing: lift, knee out to REACH_KNEE, lower to
// REACH_Y. Stance: knee closes to PULL_KNEE while Y moves to PULL_Y so the foot stays on the ground.
// GUESSED values, no leg geometry yet - tune PULL_Y until the planted foot neither lifts nor jacks the body.
// Knee travel doubled 2026-10-03 (250 -> 500 us): the tucked end stays at the stand tuck, the reach end
// goes 200 past the knee's neutral; the Y change is doubled with it (40 -> 80).
constexpr int WALK_FRONT_REACH_KNEE_US = 200;   // knee out: foot far ahead (was -50)
constexpr int WALK_FRONT_REACH_Y_US = -140;     // Y when the reached foot is set down (was -100; stand push = -100)
constexpr int WALK_FRONT_PULL_KNEE_US = -300;   // knee closed at the end of the pull (stand tuck = -300)
constexpr int WALK_FRONT_PULL_Y_US = -60;       // Y at the end of the pull (less down as the knee closes)
constexpr int WALK_BACK_SPREAD_US = 220;    // BL BR X base toward the tail (~30 deg): back feet come together
// Back legs mirror the front legs: X holds, and they push the body forward with K and Y. Swing: lift,
// knee closes to TUCK_KNEE, lower to TUCK_Y (foot set down close in). Stance: knee opens to PUSH_KNEE
// while Y moves to PUSH_Y. GUESSED values, same starting numbers as the front - tune separately.
constexpr int WALK_REAR_TUCK_KNEE_US = -300;    // knee closed at the start of the push (stand tuck = -300)
constexpr int WALK_REAR_TUCK_Y_US = -60;        // Y when the tucked foot is set down
constexpr int WALK_REAR_PUSH_KNEE_US = 200;     // knee out at the end of the push (was -50, travel doubled)
constexpr int WALK_REAR_PUSH_Y_US = -140;       // Y at the end of the push (was -100)
constexpr int WALK_BACK_MID_SPREAD_US = 60; // BML BMR X base toward the tail: clears FML/FMR at the longer stride
constexpr int WALK_LIFT_US = 150;           // Y up from the stand pose while swinging
constexpr int WALK_FRONT_LIFT_US = 300;      // FL FR lift higher (Y +200, same as the stand's lift): 150 left the feet dragging
constexpr int WALK_STRIDE_MID_US = 150;     // middle legs X each way from base (~20 deg)
constexpr int WALK_STRIDE_CORNER_US = 150;  // corner legs X each way from base (turns only)
constexpr uint16_t WALK_LIFT_MS = 200;   // lift and lower
constexpr uint16_t WALK_SWING_MS = 400;  // swing / push

// Run the stand-up sequence automatically after boot (after BOOT_STAND_DELAY_MS; any key cancels)
constexpr bool BOOT_STAND = true;
constexpr uint16_t BOOT_STAND_DELAY_MS = 3000;

// Absolute pulse limits for any output, regardless of joint calibration
constexpr int HARD_MIN_US = 400;
constexpr int HARD_MAX_US = 2600;

}  // namespace cfg
