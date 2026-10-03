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
constexpr int STAND_LIFT_US = 100;    // Y up before tucking knees
constexpr int STAND_TUCK_US = -300;   // K toward the body (verified: + moves the knees outward)
constexpr int STAND_PUSH_US = -100;   // Y down: feet push the body off the ground
constexpr uint16_t STAND_RAMP_MS = 500;   // centre, lift and tuck ramps
constexpr uint16_t STAND_PUSH_MS = 750;   // the loaded push, slower
constexpr uint16_t STAND_PAUSE_MS = 500;  // between steps
constexpr uint16_t FRAME_MS = 20;         // ramp update interval (one servo frame)

// ---------- Walking (joint-space alternating tetrapod, from the stand pose) ----------
// Group A = FL BML FMR BR, group B = FML BL FR BMR. Each half-cycle: swing group lifts, swings X to
// +stride while the stance group pushes X to -stride, then lowers.
constexpr int WALK_LIFT_US = 150;        // Y up from the stand pose while swinging
constexpr int WALK_STRIDE_US = 120;      // X each way from centre (+ = forward); ~16 deg
constexpr uint16_t WALK_LIFT_MS = 200;   // lift and lower
constexpr uint16_t WALK_SWING_MS = 400;  // swing / push

// Run the stand-up sequence automatically after boot (after BOOT_STAND_DELAY_MS; any key cancels)
constexpr bool BOOT_STAND = true;
constexpr uint16_t BOOT_STAND_DELAY_MS = 3000;

// Absolute pulse limits for any output, regardless of joint calibration
constexpr int HARD_MIN_US = 400;
constexpr int HARD_MAX_US = 2600;

}  // namespace cfg
