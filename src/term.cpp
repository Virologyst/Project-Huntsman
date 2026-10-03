#include "term.h"

#include "net.h"

DualTerm Term;

namespace {

WiFiClient *netClient() {
    WiFiClient &c = net::client();
    return c.connected() ? &c : nullptr;
}

}  // namespace

int DualTerm::available() {
    WiFiClient *c = netClient();
    return Serial.available() + (c ? c->available() : 0);
}

int DualTerm::read() {
    if (Serial.available()) return Serial.read();
    WiFiClient *c = netClient();
    return c && c->available() ? c->read() : -1;
}

int DualTerm::peek() {
    if (Serial.available()) return Serial.peek();
    WiFiClient *c = netClient();
    return c && c->available() ? c->peek() : -1;
}

size_t DualTerm::write(uint8_t ch) {
    Serial.write(ch);
    if (WiFiClient *c = netClient()) c->write(ch);
    return 1;
}

size_t DualTerm::write(const uint8_t *buf, size_t size) {
    Serial.write(buf, size);
    if (WiFiClient *c = netClient()) c->write(buf, size);
    return size;
}

void DualTerm::flush() { Serial.flush(); }
