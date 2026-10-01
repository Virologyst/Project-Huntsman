// Huntsman - 8-legged spider robot
// Brain Box: ESP32-S3 + 2x PCA9685 (board 1 = 0x40 back legs, board 2 = 0x41 front legs)
//
// Current firmware: calibration console. On boot every output goes to cfg::BOOT_PULSE_US
// (0 = boot with outputs off). See docs/calibration.md.

#include <Arduino.h>

#include "config.h"
#include "console.h"
#include "pwm.h"
#include "servo_map.h"

void setup() {
    Serial.begin(cfg::SERIAL_BAUD);
    delay(1500);

    pwm::begin();
    bool saved = servos::load();

    Serial.println("\n=== Huntsman calibration console ===");
    for (int b = 1; b <= cfg::BOARD_COUNT; b++)
        Serial.printf("Board %d (0x%02X): %s, osc %lu Hz\n", b, cfg::BOARD_ADDR[b - 1],
                      pwm::boardFound(b) ? "OK" : "NOT FOUND", (unsigned long)pwm::osc(b));
    Serial.println(saved ? "Joint map: loaded from flash" : "Joint map: prototype defaults (UNVERIFIED)");

    if (cfg::BOOT_PULSE_US) {
        for (int b = 1; b <= cfg::BOARD_COUNT; b++) {
            for (int ch = 0; ch < 16; ch++) {
                pwm::setPulse(b, ch, cfg::BOOT_PULSE_US);
                delay(cfg::BOOT_PACE_MS);
            }
        }
        Serial.printf("All 32 outputs at %d us. Type 'limp' to release. Type help.\n", cfg::BOOT_PULSE_US);
    } else {
        Serial.println("All outputs OFF. Type help.");
    }
    Serial.print("> ");
}

void loop() {
    console::poll();
}
