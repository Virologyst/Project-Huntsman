#include "net.h"

#include <ArduinoOTA.h>

#include "config.h"
#include "motion.h"
#include "secrets.h"
#include "term.h"

namespace {

WiFiServer server(cfg::CONSOLE_PORT);
WiFiClient consoleClient;
// Known networks (secrets.h): home, and optionally a second one such as the Pi payload's hotspot
struct Network {
    const char *ssid, *pass;
};
const Network NETWORKS[] = {
    {WIFI_SSID, WIFI_PASSWORD},
#ifdef WIFI_SSID_2
    {WIFI_SSID_2, WIFI_PASSWORD_2},
#endif
};
constexpr int NETWORK_COUNT = sizeof(NETWORKS) / sizeof(NETWORKS[0]);
int current = 0;            // network being tried / used
uint32_t tryStarted = 0;    // when the current attempt began
int failedInRow = 0;        // attempts without a connection (a full round = NETWORK_COUNT)

void tryNetwork(int i) {
    current = i;
    tryStarted = millis();
    WiFi.disconnect();
    WiFi.begin(NETWORKS[i].ssid, NETWORKS[i].pass);
    Term.printf("Wi-Fi: trying %s\n", NETWORKS[i].ssid);
}

bool servicesStarted = false;  // OTA, mDNS and console server start on the first connection
bool reported = false;
int lastStatus = -1;

const char *statusText(wl_status_t s) {
    switch (s) {
        case WL_NO_SSID_AVAIL: return "network not found (2.4 GHz only - check SSID and antenna)";
        case WL_CONNECT_FAILED: return "connection failed (check password)";
        case WL_CONNECTION_LOST: return "connection lost - retrying";
        case WL_DISCONNECTED: return "disconnected - retrying";
        case WL_IDLE_STATUS: return "connecting";
        default: return "status changed";
    }
}

}  // namespace

namespace net {

WiFiClient &client() { return consoleClient; }

bool connected() { return WiFi.status() == WL_CONNECTED; }

void begin() {
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(cfg::HOSTNAME);
    WiFi.setAutoReconnect(NETWORK_COUNT == 1);  // with several networks, handle() rotates through them
    tryNetwork(0);

    ArduinoOTA.setHostname(cfg::HOSTNAME);
    ArduinoOTA.onStart([] {
        Term.println("\nOTA update starting.");
        if (motion::isStanding() || motion::inWalkPose() || motion::inClimbPose()) {
            Term.println("Sitting down first.");
            motion::sitDown();
        }
    });
    ArduinoOTA.onEnd([] { Term.println("OTA update done - rebooting."); });
    ArduinoOTA.onError([](ota_error_t e) { Term.printf("OTA error %u\n", e); });
}

void handle() {
    wl_status_t status = WiFi.status();
    if (status != WL_CONNECTED) {
        if (status != lastStatus) Term.printf("Wi-Fi: %s\n", statusText(status));
        lastStatus = status;
        reported = false;
        // Rotate through the known networks (WiFi.begin is non-blocking, so the gait never stalls)
        uint32_t wait = failedInRow >= NETWORK_COUNT ? cfg::WIFI_IDLE_TRY_MS : cfg::WIFI_TRY_MS;
        if (NETWORK_COUNT > 1 && millis() - tryStarted > wait) {
            failedInRow++;
            tryNetwork((current + 1) % NETWORK_COUNT);
        }
        return;
    }
    lastStatus = status;
    failedInRow = 0;
    if (!servicesStarted) {
        ArduinoOTA.begin();  // also starts mDNS as HOSTNAME.local
        server.begin();
        server.setNoDelay(true);
        servicesStarted = true;
    }
    if (!reported) {
        Term.printf("\nWi-Fi connected to %s: %s.local (%s), console on port %u\n", NETWORKS[current].ssid,
                    cfg::HOSTNAME, WiFi.localIP().toString().c_str(), cfg::CONSOLE_PORT);
        reported = true;
    }
    ArduinoOTA.handle();

    if (server.hasClient()) {
        if (consoleClient.connected()) consoleClient.stop();  // newest connection wins
        consoleClient = server.available();
        consoleClient.setNoDelay(true);
        consoleClient.println("Huntsman console - type help");
        consoleClient.print("> ");
    }
}

}  // namespace net
