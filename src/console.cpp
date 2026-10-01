#include "console.h"

#include <Arduino.h>

#include "config.h"
#include "pwm.h"
#include "servo_map.h"

namespace {

constexpr int MAX_TOKENS = 8;
char line[96];
size_t lineLen = 0;

bool eq(const char *a, const char *b) { return a && !strcasecmp(a, b); }

// Scope readings may be typed in us (1534) or ms (1.534)
float parseMeasuredUs(const char *s) {
    float v = atof(s);
    return v < 10.0f ? v * 1000.0f : v;
}

int tokenize(char *s, char *tok[], int max) {
    int n = 0;
    for (char *t = strtok(s, " \t"); t && n < max; t = strtok(nullptr, " \t")) tok[n++] = t;
    return n;
}

bool parseChannel(const char *s, int &ch) {
    if (!s || !isdigit(s[0])) return false;
    ch = atoi(s);
    return ch >= 0 && ch <= 15;
}

bool checkUs(int us) {
    if (us >= cfg::HARD_MIN_US && us <= cfg::HARD_MAX_US) return true;
    Serial.printf("Pulse must be %d-%d us\n", cfg::HARD_MIN_US, cfg::HARD_MAX_US);
    return false;
}

void printJoint(int i) {
    const Joint &j = servos::joints[i];
    int now = servos::position(i);
    Serial.printf("%s %c (board %d ch %d): min %d, neutral %d, max %d, dir %+d, now ", j.leg, j.type,
                  j.board, j.channel, j.minUs, j.neutralUs, j.maxUs, j.dir);
    if (now) Serial.printf("%d us\n", now);
    else Serial.println("off");
}

void reportMove(int i, int requested, int sent) {
    const Joint &j = servos::joints[i];
    Serial.printf("%s %c -> %d us (neutral %+d)", j.leg, j.type, sent, (sent - j.neutralUs) * j.dir);
    if (sent != requested) Serial.printf("  [clamped from %d to joint limit]", requested);
    Serial.println();
}

void printStatus() {
    Serial.printf("\nFrame rate: %.1f Hz (period %.3f ms)\n", pwm::frameHz(), 1000.0f / pwm::frameHz());
    for (int b = 1; b <= cfg::BOARD_COUNT; b++) {
        Serial.printf("Board %d (0x%02X): %s, osc %lu Hz, last pulse %d us\n", b, cfg::BOARD_ADDR[b - 1],
                      pwm::boardFound(b) ? "OK" : "NOT FOUND", (unsigned long)pwm::osc(b), pwm::lastPulse(b));
        for (int ch = 0; ch < 16; ch++)
            if (pwm::pulse(b, ch)) Serial.printf("   ch %2d: %d us\n", ch, pwm::pulse(b, ch));
    }
}

// ---------- joint commands ----------

// FL K 1600 | FL K +50 | FL K+50 | FL K off | FL K neutral | FL K | FL neutral | FL off
void cmdLeg(char *tok[], int n) {
    const char *leg = tok[0];
    if (n == 1) {
        for (int i = 0; i < servos::COUNT; i++)
            if (eq(servos::joints[i].leg, leg)) printJoint(i);
        return;
    }
    if (eq(tok[1], "neutral") || eq(tok[1], "off")) {
        bool neutral = eq(tok[1], "neutral");
        for (int i = 0; i < servos::COUNT; i++) {
            if (!eq(servos::joints[i].leg, leg)) continue;
            if (neutral) servos::moveRaw(i, servos::joints[i].neutralUs);
            else servos::off(i);
        }
        Serial.printf("%s %s\n", leg, neutral ? "to neutral" : "off");
        return;
    }

    int i = servos::find(leg, tok[1][0]);
    if (i < 0) { Serial.printf("No joint '%c' on %s (use K, Y or X)\n", tok[1][0], leg); return; }
    const char *val = tok[1][1] ? tok[1] + 1 : (n > 2 ? tok[2] : nullptr);

    if (!val) {
        printJoint(i);
    } else if (eq(val, "off")) {
        servos::off(i);
        Serial.printf("%s %c off\n", leg, servos::joints[i].type);
    } else if (eq(val, "neutral") || eq(val, "n")) {
        int us = servos::joints[i].neutralUs;
        reportMove(i, us, servos::moveRaw(i, us));
    } else if (val[0] == '+' || val[0] == '-') {
        int offset = atoi(val);
        int requested = servos::joints[i].neutralUs + offset * servos::joints[i].dir;
        reportMove(i, requested, servos::moveOffset(i, offset));
    } else if (isdigit(val[0])) {
        int us = atoi(val);
        if (!checkUs(us)) return;
        reportMove(i, us, servos::moveRaw(i, us));
    } else {
        Serial.println("Usage: <leg> <K|Y|X> <us | +N | -N | neutral | off>");
    }
}

// setmin|setmax|setneutral <leg> <joint> [us]   (no us = current position)
void cmdSetLimit(char *tok[], int n) {
    if (n < 3 || !servos::isLeg(tok[1])) { Serial.printf("Usage: %s <leg> <K|Y|X> [us]\n", tok[0]); return; }
    int i = servos::find(tok[1], tok[2][0]);
    if (i < 0) { Serial.println("No such joint (use K, Y or X)"); return; }
    Joint &j = servos::joints[i];

    int us = n > 3 ? atoi(tok[3]) : servos::position(i);
    if (!us) { Serial.println("Joint is off - give a value or move it first"); return; }
    if (!checkUs(us)) return;

    if (eq(tok[0], "setmin")) {
        if (us >= j.maxUs) { Serial.println("Min must be below max"); return; }
        j.minUs = us;
    } else if (eq(tok[0], "setmax")) {
        if (us <= j.minUs) { Serial.println("Max must be above min"); return; }
        j.maxUs = us;
    } else {
        j.neutralUs = us;
    }
    printJoint(i);
    if (j.neutralUs < j.minUs || j.neutralUs > j.maxUs) Serial.println("  WARNING: neutral is outside min/max");
    Serial.println("  (not saved - type 'save' to keep)");
}

// setdir <leg> <joint> <1|-1>
void cmdSetDir(char *tok[], int n) {
    if (n < 4 || !servos::isLeg(tok[1])) { Serial.println("Usage: setdir <leg> <K|Y|X> <1|-1>"); return; }
    int i = servos::find(tok[1], tok[2][0]);
    int dir = atoi(tok[3]);
    if (i < 0 || (dir != 1 && dir != -1)) { Serial.println("Usage: setdir <leg> <K|Y|X> <1|-1>"); return; }
    servos::joints[i].dir = dir;
    printJoint(i);
    Serial.println("  (not saved - type 'save' to keep)");
}

// assign <leg> <joint> <board> <ch>
void cmdAssign(char *tok[], int n) {
    int ch;
    int b = n > 3 ? atoi(tok[3]) : 0;
    if (n < 5 || !servos::isLeg(tok[1]) || !pwm::validBoard(b) || !parseChannel(tok[4], ch)) {
        Serial.println("Usage: assign <leg> <K|Y|X> <1|2> <ch>");
        return;
    }
    int i = servos::find(tok[1], tok[2][0]);
    if (i < 0) { Serial.println("No such joint (use K, Y or X)"); return; }
    int other = servos::findByOutput(b, ch);
    if (other >= 0 && other != i) {
        Serial.printf("Board %d ch %d is already %s %c - reassign that one too\n", b, ch,
                      servos::joints[other].leg, servos::joints[other].type);
    }
    servos::off(i);  // stop pulses on the old output
    servos::joints[i].board = b;
    servos::joints[i].channel = ch;
    servos::joints[i].wired = false;  // manual assignment is unconfirmed until 'find'
    printJoint(i);
    Serial.println("  (not saved - type 'save' to keep)");
}

// ---------- harness identification (NO SERVOS CONNECTED) ----------

constexpr int OUTPUT_COUNT = cfg::BOARD_COUNT * 16;  // output o = board (o / 16) + 1, channel o % 16
constexpr int FIND_PULSE_US = 1500;
bool servosUnplugged = false;  // asked once per boot

// Blocks until a y/n line arrives. Returns 'y', 'n' or 'q' (quit).
char ask(const char *question) {
    while (true) {
        Serial.printf("%s (y/n, q = quit): ", question);
        char buf[16];
        size_t len = 0;
        while (true) {
            if (!Serial.available()) { delay(5); continue; }
            char c = Serial.read();
            if (c == '\r' || c == '\n') {
                if (len) break;
                continue;
            }
            if (len < sizeof(buf) - 1) buf[len++] = c;
        }
        buf[len] = 0;
        char a = tolower(buf[0]);
        Serial.println();
        if (a == 'y' || a == 'n' || a == 'q') return a;
    }
}

// Pulses on outputs [from, to), off everywhere else
void showOutputs(int from, int to) {
    for (int o = 0; o < OUTPUT_COUNT; o++) {
        int b = o / 16 + 1, ch = o % 16;
        if (o >= from && o < to) pwm::setPulse(b, ch, FIND_PULSE_US);
        else pwm::setOff(b, ch);
    }
}

// find [leg joint] - probe a connector, answer y/n until its output is known (5 questions)
void cmdFind(char *tok[], int n) {
    int joint = -1;
    if (n >= 3) {
        if (!servos::isLeg(tok[1]) || (joint = servos::find(tok[1], tok[2][0])) < 0) {
            Serial.println("Usage: find [<leg> <K|Y|X>]");
            return;
        }
    }
    if (!servosUnplugged) {
        Serial.println("find drives ALL outputs - every servo must be UNPLUGGED.");
        if (ask("Are all servos unplugged?") != 'y') { Serial.println("Cancelled."); return; }
        servosUnplugged = true;
    }
    if (joint >= 0) Serial.printf("Probe the %s %c signal wire.\n", servos::joints[joint].leg, servos::joints[joint].type);
    else Serial.println("Probe the signal wire you want to identify.");

    char a;
    showOutputs(0, OUTPUT_COUNT);
    if ((a = ask("All outputs on - do you see pulses?")) != 'y') {
        pwm::allOff();
        Serial.println(a == 'q' ? "Cancelled." : "No pulses on that wire - check probe, ground clip and harness.");
        return;
    }

    int lo = 0, hi = OUTPUT_COUNT;
    for (int step = 1; hi - lo > 1; step++) {
        int mid = (lo + hi) / 2;
        showOutputs(lo, mid);
        char q[40];
        snprintf(q, sizeof(q), "Step %d of 5 - pulses?", step);
        if ((a = ask(q)) == 'q') { pwm::allOff(); Serial.println("Cancelled."); return; }
        if (a == 'y') hi = mid;
        else lo = mid;
    }

    int b = lo / 16 + 1, ch = lo % 16;
    showOutputs(lo, lo + 1);
    a = ask("Only that output on now - pulses?");
    pwm::allOff();
    if (a != 'y') {
        Serial.println(a == 'q' ? "Cancelled." : "Inconsistent answers - run find again.");
        return;
    }

    int mapped = servos::findByOutput(b, ch);
    Serial.printf("This wire is board %d ch %d", b, ch);
    if (mapped >= 0) Serial.printf(" (map says %s %c)", servos::joints[mapped].leg, servos::joints[mapped].type);
    Serial.println(".");
    if (joint < 0) return;

    Joint &j = servos::joints[joint];
    if (j.board == b && j.channel == ch) {
        Serial.printf("MATCH: %s %c is where the map says.\n", j.leg, j.type);
    } else {
        Serial.printf("CHANGED: %s %c was board %d ch %d, now board %d ch %d.\n", j.leg, j.type, j.board,
                      j.channel, b, ch);
        if (mapped >= 0 && mapped != joint)
            Serial.printf("  %s %c also points at this output - run find on it too.\n",
                          servos::joints[mapped].leg, servos::joints[mapped].type);
        j.board = b;
        j.channel = ch;
    }
    j.wired = true;
    Serial.println("  (not saved - type 'save' to keep)");
}

// Every output gets a unique width: board 1 = 1000 + 20*ch, board 2 = 1600 + 20*ch
constexpr int IDENT_BASE_US[cfg::BOARD_COUNT] = {1000, 1600};
constexpr int IDENT_STEP_US = 20;

void cmdIdent(char *tok[], int n) {
    if (n < 2 || !eq(tok[1], "confirm")) {
        Serial.println("ident drives ALL 32 outputs to arbitrary positions - servos must be DISCONNECTED.");
        Serial.println("Type 'ident confirm' to proceed, 'limp' to stop.");
        return;
    }
    for (int b = 1; b <= cfg::BOARD_COUNT; b++)
        for (int ch = 0; ch < 16; ch++) pwm::setPulse(b, ch, IDENT_BASE_US[b - 1] + ch * IDENT_STEP_US);
    Serial.println("Ident pattern on: board 1 = 1000 + 20*ch us (1000-1300), board 2 = 1600 + 20*ch us (1600-1900).");
    Serial.println("Probe a wire, then 'which <measured_us>'. 'limp' when done.");
}

// which <us> - decode an ident pulse width to board/channel
void cmdWhich(char *tok[], int n) {
    if (n < 2) { Serial.println("Usage: which <measured_us>"); return; }
    float us = parseMeasuredUs(tok[1]);
    for (int b = 1; b <= cfg::BOARD_COUNT; b++) {
        int ch = lroundf((us - IDENT_BASE_US[b - 1]) / IDENT_STEP_US);
        if (ch < 0 || ch > 15 || fabsf(us - (IDENT_BASE_US[b - 1] + ch * IDENT_STEP_US)) > 8) continue;
        int j = servos::findByOutput(b, ch);
        Serial.printf("%.0f us = board %d ch %d", us, b, ch);
        if (j >= 0) Serial.printf("  (map says %s %c)\n", servos::joints[j].leg, servos::joints[j].type);
        else Serial.println("  (not in map)");
        return;
    }
    Serial.println("Not an ident width - is 'ident confirm' running and the board clock calibrated?");
}

// ---------- board-level commands (bypass joint limits, keep hard limits) ----------

void cmdPulse(char *tok[], int n) {
    int b = n > 1 ? atoi(tok[1]) : 0, ch;
    if (n < 4 || !pwm::validBoard(b) || !parseChannel(tok[2], ch)) { Serial.println("Usage: p <1|2> <ch> <us>"); return; }
    int us = atoi(tok[3]);
    if (!checkUs(us)) return;
    pwm::setPulse(b, ch, us);
    Serial.printf("Board %d ch %d -> %d us\n", b, ch, us);
}

void cmdOff(char *tok[], int n) {
    int b = n > 1 ? atoi(tok[1]) : 0, ch;
    if (n < 3 || !pwm::validBoard(b)) { Serial.println("Usage: off <1|2> <ch|all>"); return; }
    if (eq(tok[2], "all")) {
        for (int c = 0; c < 16; c++) pwm::setOff(b, c);
        Serial.printf("Board %d all off\n", b);
    } else if (parseChannel(tok[2], ch)) {
        pwm::setOff(b, ch);
        Serial.printf("Board %d ch %d off\n", b, ch);
    } else {
        Serial.println("Channel must be 0-15 or all");
    }
}

void cmdSweep(char *tok[], int n) {
    int b = n > 1 ? atoi(tok[1]) : 0, ch;
    if (n < 7 || !pwm::validBoard(b) || !parseChannel(tok[2], ch)) {
        Serial.println("Usage: sweep <1|2> <ch> <from> <to> <step> <ms>");
        return;
    }
    int from = atoi(tok[3]), to = atoi(tok[4]), step = atoi(tok[5]), ms = atoi(tok[6]);
    if (!checkUs(from) || !checkUs(to)) return;
    if (step <= 0) step = 10;
    int dir = to >= from ? 1 : -1;

    Serial.printf("Sweep board %d ch %d: %d -> %d us, step %d, %d ms (any key aborts)\n", b, ch, from, to, step, ms);
    while (Serial.available()) Serial.read();
    for (int us = from; dir > 0 ? us <= to : us >= to; us += dir * step) {
        pwm::setPulse(b, ch, us);
        Serial.printf("  %d us\n", us);
        for (uint32_t t = millis(); millis() - t < (uint32_t)ms;) {
            if (Serial.available()) {
                while (Serial.available()) Serial.read();
                Serial.println("Aborted.");
                return;
            }
        }
    }
    Serial.println("Sweep done.");
}

void cmdCal(char *tok[], int n) {
    int b = n > 1 ? atoi(tok[1]) : 0;
    if (n < 3 || !pwm::validBoard(b)) { Serial.println("Usage: cal <1|2> <measured_us>"); return; }
    float measured = parseMeasuredUs(tok[2]);
    int commanded = pwm::lastPulse(b);
    if (!commanded) { Serial.println("Output a pulse on that board first (p ...)"); return; }
    if (measured < commanded * 0.8f || measured > commanded * 1.2f) {
        Serial.println("Measured value is >20% off the commanded pulse - check the reading");
        return;
    }
    uint32_t old = pwm::osc(b);
    uint32_t now = pwm::calibrate(b, measured);
    Serial.printf("Board %d osc %lu -> %lu Hz. Re-measure; repeat until it reads %d us, then 'save'.\n", b,
                  (unsigned long)old, (unsigned long)now, commanded);
}

// calf <b> <measured_hz> - calibrate from the scope's frequency reading (needs a pulse running)
void cmdCalFrame(char *tok[], int n) {
    int b = n > 1 ? atoi(tok[1]) : 0;
    if (n < 3 || !pwm::validBoard(b)) { Serial.println("Usage: calf <1|2> <measured_hz>"); return; }
    float hz = atof(tok[2]);
    if (!pwm::lastPulse(b)) { Serial.println("Output a pulse on that board first (p ...)"); return; }
    if (hz < pwm::frameHz() * 0.8f || hz > pwm::frameHz() * 1.2f) {
        Serial.println("Measured frequency is >20% off the frame rate - check the reading");
        return;
    }
    uint32_t old = pwm::osc(b);
    uint32_t now = pwm::calibrateFromFrame(b, hz);
    Serial.printf("Board %d osc %lu -> %lu Hz. Re-measure; frequency should now read %.1f Hz, then 'save'.\n", b,
                  (unsigned long)old, (unsigned long)now, pwm::frameHz());
}

void cmdOsc(char *tok[], int n) {
    int b = n > 1 ? atoi(tok[1]) : 0;
    if (n < 3 || !pwm::validBoard(b)) { Serial.println("Usage: osc <1|2> <hz>"); return; }
    uint32_t hz = strtoul(tok[2], nullptr, 10);
    if (hz < 20000000 || hz > 30000000) { Serial.println("Oscillator must be 20-30 MHz"); return; }
    pwm::setOsc(b, hz);
    Serial.printf("Board %d osc = %lu Hz\n", b, (unsigned long)hz);
}

void cmdFreq(char *tok[], int n) {
    float hz = n > 1 ? atof(tok[1]) : 0;
    if (hz < 40 || hz > 400) { Serial.println("Frame rate must be 40-400 Hz"); return; }
    pwm::setFrameHz(hz);
    Serial.printf("Frame rate = %.1f Hz\n", hz);
}

void handle(char *cmdLine) {
    char *tok[MAX_TOKENS];
    int n = tokenize(cmdLine, tok, MAX_TOKENS);
    if (!n) return;
    const char *c = tok[0];

    if (eq(c, "help") || eq(c, "?")) console::printHelp();
    else if (eq(c, "status")) printStatus();
    else if (eq(c, "map")) servos::printTable();
    else if (eq(c, "export")) servos::printExport();
    else if (eq(c, "all") || eq(c, "neutral")) { servos::allNeutral(); Serial.println("All joints to neutral."); }
    else if (eq(c, "limp")) { pwm::allOff(); Serial.println("All outputs off."); }
    else if (eq(c, "setmin") || eq(c, "setmax") || eq(c, "setneutral")) cmdSetLimit(tok, n);
    else if (eq(c, "setdir")) cmdSetDir(tok, n);
    else if (eq(c, "assign")) cmdAssign(tok, n);
    else if (eq(c, "find")) cmdFind(tok, n);
    else if (eq(c, "ident")) cmdIdent(tok, n);
    else if (eq(c, "which")) cmdWhich(tok, n);
    else if (eq(c, "p")) cmdPulse(tok, n);
    else if (eq(c, "off")) cmdOff(tok, n);
    else if (eq(c, "sweep")) cmdSweep(tok, n);
    else if (eq(c, "cal")) cmdCal(tok, n);
    else if (eq(c, "calf")) cmdCalFrame(tok, n);
    else if (eq(c, "osc")) cmdOsc(tok, n);
    else if (eq(c, "freq")) cmdFreq(tok, n);
    else if (eq(c, "save")) { pwm::saveSettings(); servos::save(); Serial.println("Clocks and joint map saved to flash."); }
    else if (eq(c, "load")) {
        pwm::loadSettings();
        Serial.println(servos::load() ? "Loaded from flash." : "Clocks loaded; no saved joint map - using defaults.");
    }
    else if (eq(c, "defaults")) {
        pwm::resetSettings();
        servos::resetDefaults();
        Serial.println("Defaults restored (not saved - type 'save' to keep).");
    }
    else if (servos::isLeg(c)) cmdLeg(tok, n);
    else Serial.printf("Unknown command '%s' - type help\n", c);
}

}  // namespace

namespace console {

void printHelp() {
    Serial.println(F(
        "\nJoints (legs FL FML BML BL FR FMR BMR BR, joints K Y X):\n"
        "  FR X 1600              move to a pulse (clamped to joint min/max)\n"
        "  FR X +50 | FML Y-20    offset from neutral, direction-corrected\n"
        "  FR X neutral | off     one joint;  FR neutral | FR off  whole leg;  FR  show leg\n"
        "  all | limp             every joint to neutral / every output off\n"
        "  setmin|setmax|setneutral <leg> <joint> [us]   (no us = current position)\n"
        "  setdir <leg> <joint> <1|-1>\n"
        "  assign <leg> <joint> <b> <ch>   rewire a joint to another output\n"
        "  map | export           show joint table / print it as C++ for servo_map.cpp\n"
        "Harness check (SERVOS UNPLUGGED):\n"
        "  find FL K              probe FL K's wire, answer y/n -> confirms or fixes its board/channel\n"
        "  find                   identify any wire without changing the map\n"
        "  ident confirm          unique pulse on every output;  which <us>  decode a precise scope reading\n"
        "Boards (1 = 0x40, 2 = 0x41; ignore joint limits):\n"
        "  p <b> <ch> <us>        raw pulse, e.g. p 1 0 1500\n"
        "  off <b> <ch|all>\n"
        "  sweep <b> <ch> <from> <to> <step> <ms>   (any key aborts)\n"
        "  cal <b> <measured>     correct board clock from scope pulse width (us, or ms e.g. 1.534)\n"
        "  calf <b> <hz>          correct board clock from scope frequency reading\n"
        "  osc <b> <hz> | freq <hz> | status\n"
        "Settings: save | load | defaults"));
}

void poll() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\r' || c == '\n') {
            if (lineLen) {
                line[lineLen] = 0;
                Serial.println();  // the serial monitor echoes typed text locally
                handle(line);
                lineLen = 0;
                Serial.print("> ");
            }
        } else if (c == 8 || c == 127) {  // backspace
            if (lineLen) lineLen--;
        } else if (lineLen < sizeof(line) - 1) {
            line[lineLen++] = c;
        }
    }
}

}  // namespace console
