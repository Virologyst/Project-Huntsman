// Huntsman - 8-legged spider robot
// Brain Box: ESP32-S3 + 2x PCA9685 (board 1 = 0x40 left legs, board 2 = 0x41 right legs)
//
// On boot every output goes to cfg::BOOT_PULSE_US, then the robot stands up if cfg::BOOT_STAND (any key
// cancels). Console on USB and Wi-Fi (net.h); Xbox controller over BLE drives stand / sit / walk / climb
// (pad.h); a forward ToF sensor (tof.h) slows and stops forward walking at obstacles. diag.h logs why the
// last reset happened and drops into a safe mode after repeated crashes / brownouts.
// See docs/calibration.md, docs/motion.md, docs/wifi.md and docs/controller.md.

#include <Arduino.h>

#include "config.h"
#include "console.h"
#include "diag.h"
#include "flags.h"
#include "motion.h"
#include "net.h"
#include "pad.h"
#include "pwm.h"
#include "servo_map.h"
#include "term.h"
#include "tof.h"

// The walk / climb timelines and the IK live on the loop task's stack: the default 8 KB is too tight
SET_LOOP_TASK_STACK_SIZE(cfg::LOOP_STACK_BYTES);

void setup() {
    diag::begin();  // first: read the reset reason and the previous run's last stage
    Serial.begin(cfg::SERIAL_BAUD);
    diag::stage("setup:net");
    net::begin();  // connects in the background
    if (!diag::safeMode()) {
        diag::stage("setup:pad");
        pad::begin();  // scans for the Xbox controller in the background
    }
    delay(1500);

    diag::stage("setup:pwm");
    pwm::begin();
    bool saved = servos::load();
    diag::stage("setup:tof");
    tof::begin();  // own I2C bus (Wire1) on GPIO 17 / 18

    Term.println("\n=== Huntsman ===");
    diag::printBoot();
    for (int b = 1; b <= cfg::BOARD_COUNT; b++)
        Term.printf("Board %d (0x%02X): %s, osc %lu Hz\n", b, cfg::BOARD_ADDR[b - 1],
                    pwm::boardFound(b) ? "OK" : "NOT FOUND", (unsigned long)pwm::osc(b));
    Term.println(saved ? "Joint map: loaded from flash" : "Joint map: code defaults (servo_map.cpp)");

    if (diag::safeMode()) {
        Term.println("Safe mode: all outputs OFF. 'diag' for details. Type help.");
        Term.print("> ");
        diag::stage("idle (safe mode)");
        return;
    }

    if (cfg::BOOT_PULSE_US) {
        diag::stage("setup:boot pulse");
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
            diag::stage("setup:stand");
            motion::standUp(false);
        }
    }
    Term.print("> ");
    diag::stage("idle");
}

// ToF saw an obstacle: consume the flag and climb (only from the stand pose - sitting / mid-move ignores it)
void handleClimbFlag() {
    if (!cfg::TOF_AUTO_CLIMB || !flags::test(flags::CLIMB)) return;
    flags::clear(flags::CLIMB);
    Term.printf("\n[tof] obstacle at %d mm", tof::distanceMm());
    if (motion::isStanding()) {
        Term.println(" - climb");
        motion::climb(1);
    } else {
        Term.println(" - not standing, ignored");
    }
    Term.print("> ");
}

void loop() {
    diag::handle();
    net::handle();
    if (!diag::safeMode()) {
        pad::handle();
        tof::handle();
        handleClimbFlag();
    }
    console::poll();
}
