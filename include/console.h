// Huntsman - serial command console (see docs/calibration.md for commands)
#pragma once

namespace console {

void printHelp();
void poll();  // call from loop(): reads serial, runs complete lines

}  // namespace console
