#include "tof.h"

#include <VL53L0X.h>
#include <Wire.h>

#include "config.h"
#include "flags.h"
#include "term.h"

namespace {

VL53L0X sensor;
bool present = false;
int lastMm = -1;    // -1 = out of range / no reading
bool inBand = false;  // hysteresis: true from < TOF_CLIMB_MM until > TOF_CLEAR_MM
uint32_t lastSampleMs = 0;

// True once the sensor has a fresh sample waiting (continuous mode sets this bit each period)
bool sampleReady() { return (sensor.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) != 0; }

}  // namespace

namespace tof {

bool begin() {
    if (!cfg::TOF_ENABLED) {
        Term.println("ToF: disabled (cfg::TOF_ENABLED)");
        return false;
    }
    // Wire is already up (pwm::begin) on cfg::SDA_PIN / SCL_PIN
    sensor.setTimeout(100);
    present = sensor.init();
    if (!present) {
        Term.printf("ToF: VL53L0X NOT FOUND at 0x%02X - climb flag off\n", cfg::TOF_ADDR);
        return false;
    }
    sensor.setMeasurementTimingBudget(cfg::TOF_BUDGET_US);
    sensor.startContinuous(cfg::TOF_PERIOD_MS);
    Term.printf("ToF: VL53L0X OK, every %u ms; climb flag below %d mm (clears above %d)\n",
                cfg::TOF_PERIOD_MS, cfg::TOF_CLIMB_MM, cfg::TOF_CLEAR_MM);
    return true;
}

bool found() { return present; }
int distanceMm() { return lastMm; }
bool obstacle() { return inBand; }

void handle() {
    if (!present || !sampleReady()) return;  // nothing new - costs one I2C register read

    uint16_t mm = sensor.readRangeContinuousMillimeters();  // returns at once: the sample is ready
    lastSampleMs = millis();
    lastMm = (sensor.timeoutOccurred() || mm > cfg::TOF_MAX_MM) ? -1 : (int)mm;

    bool wasNear = inBand;
    if (lastMm >= 0 && lastMm < cfg::TOF_CLIMB_MM) inBand = true;
    else if (lastMm < 0 || lastMm > cfg::TOF_CLEAR_MM) inBand = false;

    // Edge-triggered: raise once when something comes into the band; drop it if it goes away unhandled
    if (inBand && !wasNear) flags::set(flags::CLIMB);
    if (!inBand && wasNear) flags::clear(flags::CLIMB);
}

void printStatus() {
    if (!cfg::TOF_ENABLED) {
        Term.println("ToF: disabled (cfg::TOF_ENABLED)");
        return;
    }
    if (!present) {
        Term.println("ToF: not found at boot");
        return;
    }
    if (lastMm < 0) Term.print("ToF: out of range");
    else Term.printf("ToF: %d mm", lastMm);
    Term.printf("  (%lu ms ago)  obstacle %s  CLIMB flag %s  auto-climb %s\n",
                (unsigned long)(millis() - lastSampleMs), inBand ? "YES" : "no",
                flags::test(flags::CLIMB) ? "RAISED" : "clear", cfg::TOF_AUTO_CLIMB ? "on" : "off");
}

}  // namespace tof
