// Huntsman - Xbox Wireless Controller over Bluetooth LE (see docs/controller.md)
#pragma once

namespace pad {

void begin();     // start BLE and scan for a controller in pairing mode
void handle();    // call from loop(): reconnects, runs stand / sit / walk from the controller
bool connected(); // connected and sending input
void printStatus();

}  // namespace pad
