# Calibration

The firmware is currently a serial calibration console (115200 baud). Type a command and press Enter;
the console echoes the line back. Commands are case-insensitive.

## Commands

### Joints

Legs: `FL FML BML BL FR FMR BMR BR`. Joints: `K Y X`.

| Command | Effect |
|---|---|
| `FR X 1600` | Move to a physical pulse (us), clamped to that joint's min/max |
| `FR X +50`, `FML Y-20` | Offset from neutral, multiplied by `dir` so + means the same motion on both sides |
| `FR X neutral` / `FR X off` | One joint to neutral / limp |
| `FR neutral` / `FR off` | Whole leg |
| `FR`, `FR X` | Show leg / joint settings and current pulse |
| `all` (or `neutral`) | Every joint to neutral, 20 ms apart to avoid a current spike |
| `limp` | Every output off |
| `setmin FR X [us]` | Set limit (no value = current position). Same for `setmax`, `setneutral` |
| `setdir FR X -1` | Set direction |
| `assign FR X 2 11` | Rewire a joint to board 2 channel 11 (warns if another joint already uses it) |
| `map` | Table of all joints |
| `export` | Print the map as C++ rows for `DEFAULTS` in `src/servo_map.cpp` |

Joint moves are clamped to the joint's min/max. To explore beyond the current limit, widen it first
(`setmax FR X 2450`) or use the board-level `p` command.

### Boards (bypass joint limits; hard limit 400-2600 us always applies)

| Command | Effect |
|---|---|
| `p 1 0 1500` | Raw pulse on board 1 (0x40), channel 0 |
| `off 1 all`, `off 2 5` | Stop pulses |
| `sweep 1 0 1000 2000 100 1000` | Step 1000 -> 2000 us in 100 us steps, 1 s each; any key aborts |
| `cal 1 1523` | Correct board 1 clock from the scope reading of the last pulse |
| `osc 1 26500000` | Set board clock directly |
| `freq 50` | Frame rate for both boards |
| `status` | Clocks, frame rate, active outputs |

### Harness check (servos disconnected)

| Command | Effect |
|---|---|
| `ident` / `ident confirm` | Warns, then puts a unique width on all 32 outputs: board 1 = `1000 + 20*ch` us, board 2 = `1600 + 20*ch` us |
| `which 1180` | Decodes a scope reading: `board 1 ch 9 (map says BL K)` |

### Settings

`save` writes board clocks and the joint map (including board/channel assignments) to ESP32 flash (survives power cycles and firmware uploads).
`load` re-reads them. `defaults` restores 25 MHz clocks and the prototype map (not saved until `save`).

## 1. Board clock calibration (scope)

Each PCA9685's internal oscillator differs from the nominal 25 MHz, so pulse widths are off by a few percent
until corrected.

1. Scope on a board 1 output. `p 1 0 1500`.
2. Check: period ~20 ms, high level ~3.3 V, width ~1500 us.
3. If the width is off: `cal 1 <measured>`. Re-measure, repeat until 1500 +/-5 us (one PCA step is ~4.9 us).
4. Check `p 1 0 1000` and `p 1 0 2000` also read within +/-5 us.
5. Repeat for board 2, then `save`.

## 2. Harness check (servos disconnected, scope only)

The rebuild re-wired the harness, so confirm which output reaches each joint's connector before any servo
is plugged in. Do the clock calibration first so `which` decodes accurately.

1. **All servos unplugged.** `ident confirm`.
2. Probe the signal pin of each joint's connector on the chassis. `which <measured width>`.
3. If the reported board/channel differs from the map: `assign <leg> <joint> <board> <ch>`.
4. After all 24: `limp`, `map` to review (no duplicates), `save`.

## 3. Joint mapping (one servo at a time)

Do this with the leg free to move (robot on a stand) and a hand on the servo power switch.

1. Plug in one servo. `map` to find its leg/joint.
2. `FR X neutral` - confirm the **right joint** moves.
3. Jog in small steps (`FR X +20`, `+40` ...) and check the direction: + should mean the same physical
   motion as the matching joint on the other side. If not, `setdir FR X -1`.
4. Find the safe ends: jog toward each end and stop short of any hard stop or collision, then
   `setmin FR X` / `setmax FR X` at the current position. Widen limits first if the default stops you early.
5. Place the joint at its rest pose and `setneutral FR X`.
6. `save` after each joint (cheap, and nothing is lost on a reset).
7. When all 24 are done: `export`, paste into `DEFAULTS` in `src/servo_map.cpp`, commit.

## Prototype conversion

The UNO prototype stored PCA9685 **ticks** at 50 Hz on an uncalibrated board. Converted as
`us = ticks x 20000 / 4096` (4.883 us/tick) and clamped to 500-2500 us:

| Ticks | us | Notes |
|---|---|---|
| 150 | 732 | K/Y min |
| 175 | 854 | K neutral (legs with dir -1) |
| 200 | 977 | X min |
| 275 | 1343 | alternate Y neutral (commented-out prototype set, dir -1 legs) |
| 350 | 1709 | X neutral FL/FR |
| 375 | 1831 | Y neutral, most X neutrals |
| 400 | 1953 | X neutral FMR |
| 475 | 2319 | alternate Y neutral (dir +1 legs) |
| 550 | 2686 -> 2500 | X max (clamped) |
| 575 | 2808 -> 2500 | K neutral (legs with dir +1) (clamped) |
| 600 | 2930 -> 2500 | K/Y max (clamped) |

Values above 2500 us are outside the servo's spec, and the prototype board's real clock is unknown, so
**none of these are trusted** - they are starting points for the mapping above.
