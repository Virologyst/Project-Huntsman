// Huntsman - Wi-Fi: OTA firmware updates and the network console
#pragma once

#include <WiFi.h>

namespace net {

void begin();           // start connecting (non-blocking)
void handle();          // call from loop(): OTA, console client, status messages
bool connected();
WiFiClient &client();   // current console client (may be disconnected)
void printNetworks();   // known networks, current one marked
bool useNetwork(int index, uint32_t holdMs);  // switch to network index (0-based), retrying it for holdMs

}  // namespace net
