// Huntsman - console I/O on USB serial and the Wi-Fi console at the same time
#pragma once

#include <Arduino.h>

// Reads from whichever side has input (USB first); writes to both
class DualTerm : public Stream {
public:
    int available() override;
    int read() override;
    int peek() override;
    size_t write(uint8_t c) override;
    size_t write(const uint8_t *buf, size_t size) override;
    void flush() override;
};

extern DualTerm Term;
