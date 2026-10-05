// Huntsman - restart diagnostics: why it last reset, what it was doing, crash-loop safe mode
#pragma once

namespace diag {

void begin();               // first thing in setup(): reads the reset reason and the previous run's last stage
void stage(const char *s);  // breadcrumb: kept in RTC memory, so it survives a crash / brownout reset
bool safeMode();            // several crash / brownout resets in a row: boot with servos limp, no controller
void handle();              // call from loop(): after a stable minute the quick-reboot count clears
void printBoot();           // one-line boot report
void print();               // 'diag' console command

}  // namespace diag
