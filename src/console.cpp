#include "console.h"

#include <Arduino.h>

#include "config.h"
#include "term.h"
#include "motion.h"
#include "net.h"
#include "pad.h"
#include "pwm.h"
#include "servo_map.h"
#include "tof.h"

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
    Term.printf("Pulse must be %d-%d us\n", cfg::HARD_MIN_US, cfg::HARD_MAX_US);
    return false;
}

void printJoint(int i) {
    const Joint &j = servos::joints[i];
    int now = servos::position(i);
    Term.printf("%s %c (board %d ch %d): min %d, neutral %d, max %d, dir %+d, now ", j.leg, j.type,
                  j.board, j.channel, j.minUs, j.neutralUs, j.maxUs, j.dir);
    if (now) Term.printf("%d us\n", now);
    else Term.println("off");
}

void reportMove(int i, int requested, int sent) {
    const Joint &j = servos::joints[i];
    Term.printf("%s %c -> %d us (neutral %+d)", j.leg, j.type, sent, (sent - j.neutralUs) * j.dir);
    if (sent != requested) Term.printf("  [clamped from %d to joint limit]", requested);
    Term.println();
}

void printStatus() {
    Term.printf("\nFrame rate: %.1f Hz (period %.3f ms)\n", pwm::frameHz(), 1000.0f / pwm::frameHz());
    for (int b = 1; b <= cfg::BOARD_COUNT; b++) {
        Term.printf("Board %d (0x%02X): %s, osc %lu Hz, last pulse %d us\n", b, cfg::BOARD_ADDR[b - 1],
                      pwm::boardFound(b) ? "OK" : "NOT FOUND", (unsigned long)pwm::osc(b), pwm::lastPulse(b));
        for (int ch = 0; ch < 16; ch++)
            if (pwm::pulse(b, ch)) Term.printf("   ch %2d: %d us\n", ch, pwm::pulse(b, ch));
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
        Term.printf("%s %s\n", leg, neutral ? "to neutral" : "off");
        return;
    }

    int i = servos::find(leg, tok[1][0]);
    if (i < 0) { Term.printf("No joint '%c' on %s (use K, Y or X)\n", tok[1][0], leg); return; }
    const char *val = tok[1][1] ? tok[1] + 1 : (n > 2 ? tok[2] : nullptr);

    if (!val) {
        printJoint(i);
    } else if (eq(val, "off")) {
        servos::off(i);
        Term.printf("%s %c off\n", leg, servos::joints[i].type);
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
        Term.println("Usage: <leg> <K|Y|X> <us | +N | -N | neutral | off>");
    }
}

// setmin|setmax|setneutral <leg> <joint> [us]   (no us = current position)
void cmdSetLimit(char *tok[], int n) {
    if (n < 3 || !servos::isLeg(tok[1])) { Term.printf("Usage: %s <leg> <K|Y|X> [us]\n", tok[0]); return; }
    int i = servos::find(tok[1], tok[2][0]);
    if (i < 0) { Term.println("No such joint (use K, Y or X)"); return; }
    Joint &j = servos::joints[i];

    int us = n > 3 ? atoi(tok[3]) : servos::position(i);
    if (!us) { Term.println("Joint is off - give a value or move it first"); return; }
    if (!checkUs(us)) return;

    if (eq(tok[0], "setmin")) {
        if (us >= j.maxUs) { Term.println("Min must be below max"); return; }
        j.minUs = us;
    } else if (eq(tok[0], "setmax")) {
        if (us <= j.minUs) { Term.println("Max must be above min"); return; }
        j.maxUs = us;
    } else {
        j.neutralUs = us;
    }
    printJoint(i);
    if (j.neutralUs < j.minUs || j.neutralUs > j.maxUs) Term.println("  WARNING: neutral is outside min/max");
    Term.println("  (not saved - type 'save' to keep)");
}

// setdir <leg> <joint> <1|-1>
void cmdSetDir(char *tok[], int n) {
    if (n < 4 || !servos::isLeg(tok[1])) { Term.println("Usage: setdir <leg> <K|Y|X> <1|-1>"); return; }
    int i = servos::find(tok[1], tok[2][0]);
    int dir = atoi(tok[3]);
    if (i < 0 || (dir != 1 && dir != -1)) { Term.println("Usage: setdir <leg> <K|Y|X> <1|-1>"); return; }
    servos::joints[i].dir = dir;
    printJoint(i);
    Term.println("  (not saved - type 'save' to keep)");
}

// assign <leg> <joint> <board> <ch>
void cmdAssign(char *tok[], int n) {
    int ch;
    int b = n > 3 ? atoi(tok[3]) : 0;
    if (n < 5 || !servos::isLeg(tok[1]) || !pwm::validBoard(b) || !parseChannel(tok[4], ch)) {
        Term.println("Usage: assign <leg> <K|Y|X> <1|2> <ch>");
        return;
    }
    int i = servos::find(tok[1], tok[2][0]);
    if (i < 0) { Term.println("No such joint (use K, Y or X)"); return; }
    int other = servos::findByOutput(b, ch);
    if (other >= 0 && other != i) {
        Term.printf("Board %d ch %d is already %s %c - reassign that one too\n", b, ch,
                      servos::joints[other].leg, servos::joints[other].type);
    }
    servos::off(i);  // stop pulses on the old output
    servos::joints[i].board = b;
    servos::joints[i].channel = ch;
    servos::joints[i].wired = false;  // manual assignment is unconfirmed until 'find'
    printJoint(i);
    Term.println("  (not saved - type 'save' to keep)");
}

// ---------- harness identification (NO SERVOS CONNECTED) ----------

constexpr int OUTPUT_COUNT = cfg::BOARD_COUNT * 16;  // output o = board (o / 16) + 1, channel o % 16
constexpr int FIND_PULSE_US = 1500;
bool servosUnplugged = false;  // asked once per boot

// Prints the prompt and blocks until a non-empty line arrives (trimmed into buf)
void readAnswer(const char *prompt, char *buf, size_t size) {
    Term.print(prompt);
    size_t len = 0;
    while (true) {
        if (!Term.available()) { delay(5); continue; }
        char c = Term.read();
        if (c == '\r' || c == '\n') {
            if (len) break;
            continue;
        }
        if (c == ' ' && !len) continue;
        if (len < size - 1) buf[len++] = c;
    }
    while (len && buf[len - 1] == ' ') len--;
    buf[len] = 0;
    Term.println();
}

// Blocks until a y/n line arrives. Returns 'y', 'n' or 'q' (quit).
char ask(const char *question) {
    char prompt[96], buf[16];
    snprintf(prompt, sizeof(prompt), "%s (y/n, q = quit): ", question);
    while (true) {
        readAnswer(prompt, buf, sizeof(buf));
        char a = tolower(buf[0]);
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

constexpr int LOCATE_QUIT = -1;  // user typed q
constexpr int LOCATE_NONE = -2;  // no pulses on the wire at all
constexpr int LOCATE_BAD = -3;   // answers didn't add up

bool confirmUnplugged() {
    if (servosUnplugged) return true;
    Term.println("This drives outputs with no limits - every servo must be UNPLUGGED.");
    if (ask("Are all servos unplugged?") != 'y') { Term.println("Cancelled."); return false; }
    servosUnplugged = true;
    return true;
}

int outputOf(int joint) { return (servos::joints[joint].board - 1) * 16 + servos::joints[joint].channel; }

// Bisect all outputs with y/n answers while the probe stays on one wire. Returns output index or LOCATE_*.
int locateOutput() {
    char a;
    showOutputs(0, OUTPUT_COUNT);
    if ((a = ask("All outputs on - do you see pulses?")) != 'y') {
        pwm::allOff();
        return a == 'q' ? LOCATE_QUIT : LOCATE_NONE;
    }
    int lo = 0, hi = OUTPUT_COUNT;
    for (int step = 1; hi - lo > 1; step++) {
        int mid = (lo + hi) / 2;
        showOutputs(lo, mid);
        char q[40];
        snprintf(q, sizeof(q), "Step %d of 5 - pulses?", step);
        if ((a = ask(q)) == 'q') { pwm::allOff(); return LOCATE_QUIT; }
        if (a == 'y') hi = mid;
        else lo = mid;
    }
    showOutputs(lo, lo + 1);
    a = ask("Only that output on now - pulses?");
    pwm::allOff();
    if (a == 'q') return LOCATE_QUIT;
    return a == 'y' ? lo : LOCATE_BAD;
}

void reportLocateFailure(int r) {
    if (r == LOCATE_QUIT) Term.println("Cancelled.");
    else if (r == LOCATE_NONE) Term.println("No pulses on that wire - check probe, ground clip and harness.");
    else Term.println("Inconsistent answers - try again.");
}

// Record that joint's wire is on output o; report MATCH / CHANGED
void recordJointOutput(int joint, int o) {
    int b = o / 16 + 1, ch = o % 16;
    Joint &j = servos::joints[joint];
    if (j.board == b && j.channel == ch) {
        Term.printf("MATCH: %s %c is on board %d ch %d.\n", j.leg, j.type, b, ch);
    } else {
        Term.printf("CHANGED: %s %c was board %d ch %d, now board %d ch %d.\n", j.leg, j.type, j.board,
                      j.channel, b, ch);
        int other = servos::findByOutput(b, ch);
        if (other >= 0 && other != joint)
            Term.printf("  %s %c also points at this output - check it too.\n", servos::joints[other].leg,
                          servos::joints[other].type);
        j.board = b;
        j.channel = ch;
    }
    j.wired = true;
}

// find [leg joint] - probe any wire, answer y/n until its output is known (5 questions)
void cmdFind(char *tok[], int n) {
    int joint = -1;
    if (n >= 3) {
        if (!servos::isLeg(tok[1]) || (joint = servos::find(tok[1], tok[2][0])) < 0) {
            Term.println("Usage: find [<leg> <K|Y|X>]");
            return;
        }
    }
    if (!confirmUnplugged()) return;
    if (joint >= 0) Term.printf("Probe the %s %c signal wire.\n", servos::joints[joint].leg, servos::joints[joint].type);
    else Term.println("Probe the signal wire you want to identify.");

    int o = locateOutput();
    if (o < 0) { reportLocateFailure(o); return; }

    int mapped = servos::findByOutput(o / 16 + 1, o % 16);
    Term.printf("This wire is board %d ch %d", o / 16 + 1, o % 16);
    if (mapped >= 0) Term.printf(" (map says %s %c)", servos::joints[mapped].leg, servos::joints[mapped].type);
    Term.println(".");
    if (joint < 0) return;
    recordJointOutput(joint, o);
    Term.println("  (not saved - type 'save' to keep)");
}

// check [leg] - per joint: pulse only the mapped output, user probes that wire. n -> locate it.
// No leg = all 8 legs in order.
void cmdCheck(char *tok[], int n) {
    if (n >= 2 && !servos::isLeg(tok[1])) { Term.println("Usage: check [<leg>]"); return; }
    if (!confirmUnplugged()) return;

    int first = 0, last = servos::LEG_COUNT - 1;
    for (int l = 0; n >= 2 && l < servos::LEG_COUNT; l++)
        if (eq(servos::LEGS[l], tok[1])) first = last = l;

    for (int l = first; l <= last; l++) {
        const char *leg = servos::LEGS[l];
        Term.printf("\n--- %s ---\n", leg);
        for (char type : {'K', 'Y', 'X'}) {
            int i = servos::find(leg, type);
            Joint &j = servos::joints[i];
            showOutputs(outputOf(i), outputOf(i) + 1);
            char q[64];
            snprintf(q, sizeof(q), "Probe the %s %c wire (board %d ch %d) - pulses?", leg, type, j.board, j.channel);
            char a = ask(q);
            if (a == 'q') { pwm::allOff(); Term.println("Stopped. 'save' to keep what was confirmed."); return; }
            if (a == 'y') {
                j.wired = true;
                Term.printf("OK: %s %c on board %d ch %d.\n", leg, type, j.board, j.channel);
                continue;
            }
            Term.printf("Not there. Keep the probe on the %s %c wire - locating it.\n", leg, type);
            int o = locateOutput();
            if (o == LOCATE_QUIT) { Term.println("Stopped. 'save' to keep what was confirmed."); return; }
            if (o < 0) { reportLocateFailure(o); Term.printf("%s %c left unconfirmed.\n", leg, type); continue; }
            recordJointOutput(i, o);
        }
    }
    pwm::allOff();
    Term.println("\nDone - type 'map' to review, then 'save'.");
}

// ---------- wiggle mapping (servos and legs connected) ----------

constexpr int WIGGLE_CENTER_US = 1500;
constexpr int WIGGLE_US = 50;          // 1550 -> 1500 -> 1450 -> 1500 (~7 deg on a 270 deg servo)
constexpr uint16_t WIGGLE_HOLD_MS = 600;

void wiggleOutput(int b, int ch) {
    if (pwm::pulse(b, ch) != WIGGLE_CENTER_US) {
        pwm::setPulse(b, ch, WIGGLE_CENTER_US);
        delay(WIGGLE_HOLD_MS);
    }
    const int seq[] = {WIGGLE_CENTER_US + WIGGLE_US, WIGGLE_CENTER_US, WIGGLE_CENTER_US - WIGGLE_US, WIGGLE_CENTER_US};
    for (int us : seq) {
        pwm::setPulse(b, ch, us);
        delay(WIGGLE_HOLD_MS);
    }
}

enum WiggleResult { WIGGLE_MAPPED, WIGGLE_UNUSED, WIGGLE_QUIT };

// Wiggle one output and ask what moved; records leg/joint and direction
WiggleResult wiggleAsk(int b, int ch) {
    char buf[16];
    int mapped = servos::findByOutput(b, ch);
    Term.printf("\n--- Board %d ch %d", b, ch);
    if (mapped >= 0) Term.printf(" (old map: %s %c)", servos::joints[mapped].leg, servos::joints[mapped].type);
    Term.println(" ---");
    wiggleOutput(b, ch);

    const char *leg = nullptr;
    while (!leg) {
        readAnswer("Which leg moved? (FL FML BML BL FR FMR BMR BR | none | r = repeat | q = quit): ", buf, sizeof(buf));
        if (eq(buf, "q")) return WIGGLE_QUIT;
        if (eq(buf, "r")) { wiggleOutput(b, ch); continue; }
        if (eq(buf, "none") || eq(buf, "n")) {
            Term.printf("Board %d ch %d: unused.\n", b, ch);
            return WIGGLE_UNUSED;
        }
        for (auto l : servos::LEGS)
            if (eq(buf, l)) leg = l;
        if (!leg) Term.println("Not a leg name.");
    }

    int i = -1;
    while (i < 0) {
        readAnswer("Which joint? (K = knee, Y = lift, X = swing | r = repeat | q = quit): ", buf, sizeof(buf));
        if (eq(buf, "q")) return WIGGLE_QUIT;
        if (eq(buf, "r")) { wiggleOutput(b, ch); continue; }
        if (strlen(buf) == 1) i = servos::find(leg, buf[0]);
        if (i < 0) Term.println("Use K, Y or X.");
    }
    Joint &j = servos::joints[i];

    bool swing = j.type == 'X';
    int dir = 0;
    while (!dir) {
        readAnswer(swing ? "First move: forward or back? (f/b | r = repeat | q = quit): "
                         : "First move: up or down? (u/d | r = repeat | q = quit): ",
                   buf, sizeof(buf));
        char a = tolower(buf[0]);
        if (a == 'q') return WIGGLE_QUIT;
        if (a == 'r') { wiggleOutput(b, ch); continue; }
        if (swing ? (a == 'f') : (a == 'u')) dir = +1;
        else if (swing ? (a == 'b') : (a == 'd')) dir = -1;
        else Term.println(swing ? "Use f or b." : "Use u or d.");
    }

    if (j.wired && (j.board != b || j.channel != ch))
        Term.printf("  Note: %s %c was already found on board %d ch %d this session - replacing.\n", j.leg, j.type,
                      j.board, j.channel);
    int other = servos::findByOutput(b, ch);
    if (other >= 0 && other != i && servos::joints[other].wired)
        Term.printf("  WARNING: %s %c was also recorded on this output - check it again.\n",
                      servos::joints[other].leg, servos::joints[other].type);

    bool flipped = j.dir != dir;
    j.board = b;
    j.channel = ch;
    j.dir = dir;
    j.wired = true;
    Term.printf("Board %d ch %d = %s %c, dir %+d (+ = %s)%s\n", b, ch, j.leg, j.type, dir, swing ? "forward" : "up",
                  flipped ? "  [direction changed]" : "");
    return WIGGLE_MAPPED;
}

// wiggle | wiggle <b> | wiggle <b> <ch>
void cmdWiggle(char *tok[], int n) {
    int b = n > 1 ? atoi(tok[1]) : 0, ch = -1;
    if ((n > 1 && !pwm::validBoard(b)) || (n > 2 && !parseChannel(tok[2], ch))) {
        Term.println("Usage: wiggle [<1|2> [<ch>]]");
        return;
    }
    int firstB = n > 1 ? b : 1, lastB = n > 1 ? b : cfg::BOARD_COUNT;
    if (n == 1)  // full remap: start with every joint unconfirmed
        for (int i = 0; i < servos::COUNT; i++) servos::joints[i].wired = false;
    Term.printf("Each output moves %d -> %d -> %d -> %d us. Watch the legs.\n", WIGGLE_CENTER_US + WIGGLE_US,
                  WIGGLE_CENTER_US, WIGGLE_CENTER_US - WIGGLE_US, WIGGLE_CENTER_US);
    Term.println("Convention: + = lift/knee UP, swing FORWARD. Answers set each joint's channel and direction.");

    int mapped = 0, unused = 0;
    for (int bb = firstB; bb <= lastB; bb++) {
        for (int c = (ch >= 0 ? ch : 0); c <= (ch >= 0 ? ch : 15); c++) {
            WiggleResult r = wiggleAsk(bb, c);
            if (r == WIGGLE_QUIT) {
                Term.printf("\nStopped. %d mapped, %d unused. 'save' to keep.\n", mapped, unused);
                return;
            }
            r == WIGGLE_MAPPED ? mapped++ : unused++;
        }
    }
    Term.printf("\nDone: %d mapped, %d unused.\n", mapped, unused);
    if (n == 1) {
        for (int i = 0; i < servos::COUNT; i++)
            if (!servos::joints[i].wired)
                Term.printf("  Not found: %s %c\n", servos::joints[i].leg, servos::joints[i].type);
    }
    Term.println("Type 'map' to review, then 'save'.");
}

// Every output gets a unique width: board 1 = 1000 + 20*ch, board 2 = 1600 + 20*ch
constexpr int IDENT_BASE_US[cfg::BOARD_COUNT] = {1000, 1600};
constexpr int IDENT_STEP_US = 20;

void cmdIdent(char *tok[], int n) {
    if (n < 2 || !eq(tok[1], "confirm")) {
        Term.println("ident drives ALL 32 outputs to arbitrary positions - servos must be DISCONNECTED.");
        Term.println("Type 'ident confirm' to proceed, 'limp' to stop.");
        return;
    }
    for (int b = 1; b <= cfg::BOARD_COUNT; b++)
        for (int ch = 0; ch < 16; ch++) pwm::setPulse(b, ch, IDENT_BASE_US[b - 1] + ch * IDENT_STEP_US);
    Term.println("Ident pattern on: board 1 = 1000 + 20*ch us (1000-1300), board 2 = 1600 + 20*ch us (1600-1900).");
    Term.println("Probe a wire, then 'which <measured_us>'. 'limp' when done.");
}

// which <us> - decode an ident pulse width to board/channel
void cmdWhich(char *tok[], int n) {
    if (n < 2) { Term.println("Usage: which <measured_us>"); return; }
    float us = parseMeasuredUs(tok[1]);
    for (int b = 1; b <= cfg::BOARD_COUNT; b++) {
        int ch = lroundf((us - IDENT_BASE_US[b - 1]) / IDENT_STEP_US);
        if (ch < 0 || ch > 15 || fabsf(us - (IDENT_BASE_US[b - 1] + ch * IDENT_STEP_US)) > 8) continue;
        int j = servos::findByOutput(b, ch);
        Term.printf("%.0f us = board %d ch %d", us, b, ch);
        if (j >= 0) Term.printf("  (map says %s %c)\n", servos::joints[j].leg, servos::joints[j].type);
        else Term.println("  (not in map)");
        return;
    }
    Term.println("Not an ident width - is 'ident confirm' running and the board clock calibrated?");
}

// ---------- board-level commands (bypass joint limits, keep hard limits) ----------

void cmdPulse(char *tok[], int n) {
    int b = n > 1 ? atoi(tok[1]) : 0, ch;
    if (n < 4 || !pwm::validBoard(b) || !parseChannel(tok[2], ch)) { Term.println("Usage: p <1|2> <ch> <us>"); return; }
    int us = atoi(tok[3]);
    if (!checkUs(us)) return;
    pwm::setPulse(b, ch, us);
    Term.printf("Board %d ch %d -> %d us\n", b, ch, us);
}

void cmdOff(char *tok[], int n) {
    int b = n > 1 ? atoi(tok[1]) : 0, ch;
    if (n < 3 || !pwm::validBoard(b)) { Term.println("Usage: off <1|2> <ch|all>"); return; }
    if (eq(tok[2], "all")) {
        for (int c = 0; c < 16; c++) pwm::setOff(b, c);
        Term.printf("Board %d all off\n", b);
    } else if (parseChannel(tok[2], ch)) {
        pwm::setOff(b, ch);
        Term.printf("Board %d ch %d off\n", b, ch);
    } else {
        Term.println("Channel must be 0-15 or all");
    }
}

void cmdSweep(char *tok[], int n) {
    int b = n > 1 ? atoi(tok[1]) : 0, ch;
    if (n < 7 || !pwm::validBoard(b) || !parseChannel(tok[2], ch)) {
        Term.println("Usage: sweep <1|2> <ch> <from> <to> <step> <ms>");
        return;
    }
    int from = atoi(tok[3]), to = atoi(tok[4]), step = atoi(tok[5]), ms = atoi(tok[6]);
    if (!checkUs(from) || !checkUs(to)) return;
    if (step <= 0) step = 10;
    int dir = to >= from ? 1 : -1;

    Term.printf("Sweep board %d ch %d: %d -> %d us, step %d, %d ms (any key aborts)\n", b, ch, from, to, step, ms);
    while (Term.available()) Term.read();
    for (int us = from; dir > 0 ? us <= to : us >= to; us += dir * step) {
        pwm::setPulse(b, ch, us);
        Term.printf("  %d us\n", us);
        for (uint32_t t = millis(); millis() - t < (uint32_t)ms;) {
            if (Term.available()) {
                while (Term.available()) Term.read();
                Term.println("Aborted.");
                return;
            }
        }
    }
    Term.println("Sweep done.");
}

void cmdCal(char *tok[], int n) {
    int b = n > 1 ? atoi(tok[1]) : 0;
    if (n < 3 || !pwm::validBoard(b)) { Term.println("Usage: cal <1|2> <measured_us>"); return; }
    float measured = parseMeasuredUs(tok[2]);
    int commanded = pwm::lastPulse(b);
    if (!commanded) { Term.println("Output a pulse on that board first (p ...)"); return; }
    if (measured < commanded * 0.8f || measured > commanded * 1.2f) {
        Term.println("Measured value is >20% off the commanded pulse - check the reading");
        return;
    }
    uint32_t old = pwm::osc(b);
    uint32_t now = pwm::calibrate(b, measured);
    Term.printf("Board %d osc %lu -> %lu Hz. Re-measure; repeat until it reads %d us, then 'save'.\n", b,
                  (unsigned long)old, (unsigned long)now, commanded);
}

// calf <b> <measured_hz> - calibrate from the scope's frequency reading (needs a pulse running)
void cmdCalFrame(char *tok[], int n) {
    int b = n > 1 ? atoi(tok[1]) : 0;
    if (n < 3 || !pwm::validBoard(b)) { Term.println("Usage: calf <1|2> <measured_hz>"); return; }
    float hz = atof(tok[2]);
    if (!pwm::lastPulse(b)) { Term.println("Output a pulse on that board first (p ...)"); return; }
    if (hz < pwm::frameHz() * 0.8f || hz > pwm::frameHz() * 1.2f) {
        Term.println("Measured frequency is >20% off the frame rate - check the reading");
        return;
    }
    uint32_t old = pwm::osc(b);
    uint32_t now = pwm::calibrateFromFrame(b, hz);
    Term.printf("Board %d osc %lu -> %lu Hz. Re-measure; frequency should now read %.1f Hz, then 'save'.\n", b,
                  (unsigned long)old, (unsigned long)now, pwm::frameHz());
}

void cmdOsc(char *tok[], int n) {
    int b = n > 1 ? atoi(tok[1]) : 0;
    if (n < 3 || !pwm::validBoard(b)) { Term.println("Usage: osc <1|2> <hz>"); return; }
    uint32_t hz = strtoul(tok[2], nullptr, 10);
    if (hz < 20000000 || hz > 30000000) { Term.println("Oscillator must be 20-30 MHz"); return; }
    pwm::setOsc(b, hz);
    Term.printf("Board %d osc = %lu Hz\n", b, (unsigned long)hz);
}

void cmdFreq(char *tok[], int n) {
    float hz = n > 1 ? atof(tok[1]) : 0;
    if (hz < 40 || hz > 400) { Term.println("Frame rate must be 40-400 Hz"); return; }
    pwm::setFrameHz(hz);
    Term.printf("Frame rate = %.1f Hz\n", hz);
}

void handle(char *cmdLine) {
    char *tok[MAX_TOKENS];
    int n = tokenize(cmdLine, tok, MAX_TOKENS);
    if (!n) return;
    const char *c = tok[0];

    if (eq(c, "help") || eq(c, "?")) console::printHelp();
    else if (eq(c, "status")) {
        printStatus();
        if (net::connected())
            Term.printf("Wi-Fi: %s.local (%s)\n", cfg::HOSTNAME, WiFi.localIP().toString().c_str());
        else
            Term.printf("Wi-Fi: not connected (status %d)\n", WiFi.status());
        pad::printStatus();
        tof::printStatus();
    }
    else if (eq(c, "pad")) pad::printStatus();
    else if (eq(c, "tof")) tof::printStatus();
    else if (eq(c, "map")) servos::printTable();
    else if (eq(c, "export")) servos::printExport();
    else if (eq(c, "all") || eq(c, "neutral")) { servos::allNeutral(); Term.println("All joints to neutral."); }
    else if (eq(c, "limp")) { pwm::allOff(); Term.println("All outputs off."); }
    else if (eq(c, "setmin") || eq(c, "setmax") || eq(c, "setneutral")) cmdSetLimit(tok, n);
    else if (eq(c, "setdir")) cmdSetDir(tok, n);
    else if (eq(c, "assign")) cmdAssign(tok, n);
    else if (eq(c, "find")) cmdFind(tok, n);
    else if (eq(c, "check")) cmdCheck(tok, n);
    else if (eq(c, "wiggle")) cmdWiggle(tok, n);
    else if (eq(c, "stand")) motion::standUp(n > 1 && eq(tok[1], "step"));
    else if (eq(c, "sit")) motion::sitDown();
    else if (eq(c, "walk")) motion::walk(motion::Gait::Forward, n > 1 ? atoi(tok[1]) : 0);
    else if (eq(c, "back")) motion::walk(motion::Gait::Back, n > 1 ? atoi(tok[1]) : 0);
    else if (eq(c, "turn")) {
        if (n < 2 || !(eq(tok[1], "left") || eq(tok[1], "right"))) Term.println("Usage: turn <left|right> [cycles]");
        else motion::walk(eq(tok[1], "left") ? motion::Gait::TurnLeft : motion::Gait::TurnRight,
                          n > 2 ? atoi(tok[2]) : 0);
    }
    else if (eq(c, "climb")) motion::climb();
    else if (eq(c, "ident")) cmdIdent(tok, n);
    else if (eq(c, "which")) cmdWhich(tok, n);
    else if (eq(c, "p")) cmdPulse(tok, n);
    else if (eq(c, "off")) cmdOff(tok, n);
    else if (eq(c, "sweep")) cmdSweep(tok, n);
    else if (eq(c, "cal")) cmdCal(tok, n);
    else if (eq(c, "calf")) cmdCalFrame(tok, n);
    else if (eq(c, "osc")) cmdOsc(tok, n);
    else if (eq(c, "freq")) cmdFreq(tok, n);
    else if (eq(c, "save")) { pwm::saveSettings(); servos::save(); Term.println("Clocks and joint map saved to flash."); }
    else if (eq(c, "load")) {
        pwm::loadSettings();
        Term.println(servos::load() ? "Loaded from flash." : "Clocks loaded; no saved joint map - using defaults.");
    }
    else if (eq(c, "defaults")) {
        pwm::resetSettings();
        servos::resetDefaults();
        Term.println("Defaults restored (not saved - type 'save' to keep).");
    }
    else if (servos::isLeg(c)) cmdLeg(tok, n);
    else Term.printf("Unknown command '%s' - type help\n", c);
}

}  // namespace

namespace console {

void printHelp() {
    Term.println(F(
        "\nMotion (any key aborts and holds):\n"
        "  stand | stand step     centre -> Y up -> K tuck -> Y down (step = Enter before each step)\n"
        "  sit                    lower the body, everything back to centre\n"
        "  walk [n] | back [n] | turn left|right [n]   tetrapod gait from the stand pose;\n"
        "                         n cycles or until a key (finishes the step, ends standing)\n"
        "  climb                  climb the obstacle ahead (auto when the ToF reads < TOF_CLIMB_MM while standing)\n"
        "  tof                    range sensor readout and CLIMB flag state\n"
        "Xbox controller (docs/controller.md): A stand, B sit, left stick / D-pad walk + turn\n"
        "  pad                    controller status, stick and button readout\n"
        "Joints (legs FL FML BML BL FR FMR BMR BR, joints K Y X):\n"
        "  FR X 1600              move to a pulse (clamped to joint min/max)\n"
        "  FR X +50 | FML Y-20    offset from neutral, direction-corrected\n"
        "  FR X neutral | off     one joint;  FR neutral | FR off  whole leg;  FR  show leg\n"
        "  all | limp             every joint to neutral / every output off\n"
        "  setmin|setmax|setneutral <leg> <joint> [us]   (no us = current position)\n"
        "  setdir <leg> <joint> <1|-1>\n"
        "  assign <leg> <joint> <b> <ch>   rewire a joint to another output\n"
        "  map | export           show joint table / print it as C++ for servo_map.cpp\n"
        "Wiggle mapping (servos connected):\n"
        "  wiggle | wiggle 1 | wiggle 1 5   wiggle each output 1550/1500/1450/1500, answer which leg,\n"
        "                                   joint and direction -> sets channel + dir (+ = up / forward)\n"
        "Harness check (SERVOS UNPLUGGED):\n"
        "  check FL | check       leg by leg: probe each wire, y/n; n -> locates it and fixes the map\n"
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
    while (Term.available()) {
        char c = Term.read();
        if (c == '\r' || c == '\n') {
            if (lineLen) {
                line[lineLen] = 0;
                Term.println();  // the serial monitor echoes typed text locally
                handle(line);
                lineLen = 0;
                Term.print("> ");
            }
        } else if (c == 8 || c == 127) {  // backspace
            if (lineLen) lineLen--;
        } else if (lineLen < sizeof(line) - 1) {
            line[lineLen++] = c;
        }
    }
}

}  // namespace console
