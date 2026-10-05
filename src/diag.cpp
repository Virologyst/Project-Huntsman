#include "diag.h"

#include <Arduino.h>
#include <esp_attr.h>
#include <esp_system.h>

#include "config.h"
#include "term.h"

namespace {

// RTC_NOINIT memory keeps its contents across crashes, watchdog and brownout resets (not power-off)
constexpr uint32_t MAGIC = 0x48554e54;  // "HUNT"
RTC_NOINIT_ATTR uint32_t magic;
RTC_NOINIT_ATTR uint32_t bootCount;
RTC_NOINIT_ATTR uint32_t quickBoots;  // crash / brownout / watchdog resets in a row
RTC_NOINIT_ATTR char lastStage[32];

esp_reset_reason_t reason = ESP_RST_UNKNOWN;
char prevStage[32] = "(none)";
bool safe = false;
bool cleared = false;

const char *reasonText(esp_reset_reason_t r) {
    switch (r) {
        case ESP_RST_POWERON: return "power on";
        case ESP_RST_SW: return "software restart (update / reboot)";
        case ESP_RST_EXT: return "reset pin";
        case ESP_RST_PANIC: return "CRASH (panic / exception)";
        case ESP_RST_INT_WDT: return "CRASH (interrupt watchdog)";
        case ESP_RST_TASK_WDT: return "CRASH (task watchdog - something blocked too long)";
        case ESP_RST_WDT: return "CRASH (watchdog)";
        case ESP_RST_BROWNOUT: return "BROWNOUT (supply voltage dipped)";
        case ESP_RST_DEEPSLEEP: return "deep sleep wake";
        default: return "unknown";
    }
}

bool isFault(esp_reset_reason_t r) {
    return r == ESP_RST_PANIC || r == ESP_RST_INT_WDT || r == ESP_RST_TASK_WDT || r == ESP_RST_WDT ||
           r == ESP_RST_BROWNOUT;
}

}  // namespace

namespace diag {

void begin() {
    reason = esp_reset_reason();
    if (magic != MAGIC || reason == ESP_RST_POWERON) {  // RTC memory is garbage after power-off
        magic = MAGIC;
        bootCount = 0;
        quickBoots = 0;
        lastStage[0] = 0;
    }
    if (lastStage[0]) {
        strncpy(prevStage, lastStage, sizeof(prevStage) - 1);
        prevStage[sizeof(prevStage) - 1] = 0;
    }
    bootCount++;
    quickBoots = isFault(reason) ? quickBoots + 1 : 0;
    safe = quickBoots >= (uint32_t)cfg::DIAG_SAFE_MODE_RESETS;
    stage("setup");
}

void stage(const char *s) {
    strncpy(lastStage, s, sizeof(lastStage) - 1);
    lastStage[sizeof(lastStage) - 1] = 0;
}

bool safeMode() { return safe; }

void handle() {
    if (!cleared && millis() > cfg::DIAG_STABLE_MS) {
        quickBoots = 0;  // up long enough: the next fault starts a new count
        cleared = true;
    }
}

void printBoot() {
    Term.printf("Boot #%lu - last reset: %s", (unsigned long)bootCount, reasonText(reason));
    if (isFault(reason)) Term.printf(" while '%s' (%lu in a row)", prevStage, (unsigned long)quickBoots);
    Term.println();
    if (safe)
        Term.printf("*** SAFE MODE: %lu crash / brownout resets in a row. Servos limp, no stand-up, no controller.\n"
                    "*** Wi-Fi + console stay up for a fix. Power off / on (or 'reboot') to leave safe mode.\n",
                    (unsigned long)quickBoots);
}

void print() {
    Term.printf("Uptime %lu s, boot #%lu\n", (unsigned long)(millis() / 1000), (unsigned long)bootCount);
    Term.printf("Last reset: %s\n", reasonText(reason));
    Term.printf("Previous run's last stage: %s\n", prevStage);
    Term.printf("Faults in a row: %lu%s\n", (unsigned long)quickBoots, safe ? "  (SAFE MODE)" : "");
    Term.printf("Free heap %lu bytes (lowest %lu), loop stack left %lu bytes (of %u)\n",
                (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getMinFreeHeap(),
                (unsigned long)uxTaskGetStackHighWaterMark(nullptr), (unsigned)cfg::LOOP_STACK_BYTES);
    Term.printf("Now at: %s\n", lastStage);
}

}  // namespace diag
