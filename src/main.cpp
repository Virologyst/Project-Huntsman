// Huntsman - 8-legged spider robot
// Brain Box: ESP32-S3 + 2x PCA9685 (board 1 = 0x40 left legs, board 2 = 0x41 right legs)
//
// Current firmware: calibration console. On boot every output goes to cfg::BOOT_PULSE_US, then the
// robot stands up if cfg::BOOT_STAND (any key cancels). Console on USB and Wi-Fi (net.h);
// Xbox controller over BLE drives stand / sit / walk (pad.h). A forward ToF sensor (tof.h) raises
// flags::CLIMB below cfg::TOF_NEAR_MM, which stops any walk and runs motion::climb().
// See docs/calibration.md, docs/motion.md and docs/wifi.md.

#include <Arduino.h>

#include "config.h"
#include "console.h"
#include "flags.h"
#include "motion.h"
#include "net.h"
#include "pad.h"
#include "pwm.h"
#include "servo_map.h"
#include "term.h"
#include "tof.h"

void setup() {
    Serial.begin(cfg::SERIAL_BAUD);
    net::begin();  // connects in the background
    pad::begin();  // scans for the Xbox controller in the background
    delay(1500);

    pwm::begin();
    bool saved = servos::load();
    tof::begin();  // own I2C bus (Wire1) on GPIO 17 / 18

    Term.println("\n=== Huntsman calibration console ===");
    for (int b = 1; b <= cfg::BOARD_COUNT; b++)
        Term.printf("Board %d (0x%02X): %s, osc %lu Hz\n", b, cfg::BOARD_ADDR[b - 1],
                    pwm::boardFound(b) ? "OK" : "NOT FOUND", (unsigned long)pwm::osc(b));
    Term.println(saved ? "Joint map: loaded from flash" : "Joint map: code defaults (servo_map.cpp)");

    if (cfg::BOOT_PULSE_US) {
        for (int b = 1; b <= cfg::BOARD_COUNT; b++) {
            for (int ch = 0; ch < 16; ch++) {
                pwm::setPulse(b, ch, cfg::BOOT_PULSE_US);
                delay(cfg::BOOT_PACE_MS);
            }
        }
        Term.printf("All 32 outputs at %d us. Type 'limp' to release. Type help.\n", cfg::BOOT_PULSE_US);
    } else {
        Term.println("All outputs OFF. Type help.");
    }

    if (cfg::BOOT_STAND) {
        Term.printf("Standing up in %u ms - press any key to cancel.\n", cfg::BOOT_STAND_DELAY_MS);
        uint32_t t = millis();
        while (millis() - t < cfg::BOOT_STAND_DELAY_MS && !Term.available()) {
            net::handle();  // a Wi-Fi console can connect and cancel too
            delay(10);
        }
        if (Term.available()) {
            while (Term.available()) Term.read();
            Term.println("Stand cancelled.");
        } else {
            motion::standUp(false);
        }
    }
    Term.print("> ");
}

// ToF saw an obstacle: consume the flag and climb (only from the stand pose - sitting / mid-move ignores it)
void handleClimbFlag() {
    if (!cfg::TOF_AUTO_CLIMB || !flags::test(flags::CLIMB)) return;
    flags::clear(flags::CLIMB);
    Term.printf("\n[tof] obstacle at %d mm", tof::distanceMm());
    if (motion::isStanding()) {
        Term.println(" - climb");
        motion::climb();
    } else {
        Term.println(" - not standing, ignored");
    }
    Term.print("> ");
}

void loop() {
    net::handle();
    pad::handle();
    tof::handle();
    handleClimbFlag();
    console::poll();
}
