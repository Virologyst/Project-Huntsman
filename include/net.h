// Huntsman - Wi-Fi: OTA firmware updates and the network console
#pragma once

#include <WiFi.h>

namespace net {

void begin();           // start connecting (non-blocking)
void handle();          // call from loop(): OTA, console client, status messages
bool connected();
WiFiClient &client();   // current console client (may be disconnected)

}  // namespace net
