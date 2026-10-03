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
constexpr int WALK_MID_KNEE_US = -100;      // middle legs: knee out from the stand's -300 -> feet reach out
constexpr int WALK_CORNER_KNEE_US = -200;   // corner legs: straightened a little from -300
constexpr int WALK_CORNER_SPREAD_US = 150;  // corner X base: front legs toward head, back legs toward tail
constexpr int WALK_LIFT_US = 150;           // Y up from the stand pose while swinging
constexpr int WALK_STRIDE_MID_US = 150;     // middle legs X each way from base (~20 deg)
constexpr int WALK_STRIDE_CORNER_US = 150;  // corner legs X each way from base
constexpr uint16_t WALK_LIFT_MS = 200;   // lift and lower
constexpr uint16_t WALK_SWING_MS = 400;  // swing / push

// Run the stand-up sequence automatically after boot (after BOOT_STAND_DELAY_MS; any key cancels)
constexpr bool BOOT_STAND = true;
constexpr uint16_t BOOT_STAND_DELAY_MS = 3000;

// ---------- Xbox controller (Bluetooth LE, docs/controller.md) ----------
constexpr bool PAD_ENABLED = true;
// "" = pair with the first Xbox controller found in pairing mode; or lock to one, e.g. "44:16:22:5e:b2:d4"
constexpr const char *PAD_ADDRESS = "";
constexpr float PAD_DEADZONE = 0.5f;   // stick must pass half travel to start walking
constexpr int PAD_STICK_Y_SIGN = 1;    // set to -1 if stick-forward walks backward ('pad' shows y)

// ---------- Time-of-flight range sensor (VL53L0X, forward-facing, on the PCA9685 I2C bus) ----------
constexpr bool TOF_ENABLED = true;
constexpr uint8_t TOF_ADDR = 0x29;            // VL53L0X default (fixed unless XSHUT is driven)
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
