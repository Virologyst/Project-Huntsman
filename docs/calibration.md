# Calibration

The firmware is currently a serial calibration console (115200 baud). Type a command and press Enter.
Commands are case-insensitive.

**On boot, all 32 outputs are driven to 1500 us** (one every 20 ms), so any connected servo moves to
centre. Type `limp` to release them. To boot with outputs off instead, set `BOOT_PULSE_US = 0` in
`include/config.h`.

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
| `cal 1 1523` / `cal 1 1.523` | Correct board 1 clock from the scope's pulse width (us or ms) |
| `calf 1 54.17` | Correct board 1 clock from the scope's frequency reading (more digits on most scopes) |
| `osc 1 26500000` | Set board clock directly |
| `freq 50` | Frame rate for both boards |
| `status` | Clocks, frame rate, active outputs |

### Wiggle mapping (servos and legs connected)

| Command | Effect |
|---|---|
| `wiggle` | Every output on both boards in turn: 1550 -> 1500 -> 1450 -> 1500 us (600 ms each). You answer which leg moved (or `none`), which joint, and whether the **first** move went up/down (K, Y) or forward/back (X). Sets the joint's board/channel and `dir`. Lists joints not found at the end |
| `wiggle 1` | Board 1 only |
| `wiggle 1 5` | One output (re-check a single channel) |

At any question: `r` repeats the wiggle, `q` stops. Direction convention: **+ = lift up, knee up, swing
forward**, so `dir` is +1 when the 1550 move went up/forward and -1 when it went down/back (inverted motor).

### Harness check (servos unplugged)

| Command | Effect |
|---|---|
| `check FL` / `check` | Leg by leg (or all 8 legs): pulses only the mapped output for K, then Y, then X while you probe that wire. `y` = confirmed; `n` = keeps the probe there and locates the real output (as `find`), fixing the map |
| `find FL K` | Probe FL K's signal wire and answer y/n ("pulses?") 5 times. Confirms or corrects its board/channel and marks it wired. Works with an uncalibrated clock and a coarse scope readout |
| `find` | Same, but only reports which output the probed wire is on |
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
is plugged in. `find` only needs the scope to show whether pulses are present, so clock calibration is
not required first.

Wires can't easily be traced inside the chassis, so the check works at each leg's connector, assuming
the leg's three wires keep their K, Y, X order (confirmed later with servos plugged in).

1. **All servos unplugged.** Scope ground clip on servo ground.
2. `check FL` (first time per boot it asks you to confirm servos are unplugged).
3. For each prompt, put the probe on that joint's signal wire at the leg connector and answer:
   - `y` - pulses present: confirmed.
   - `n` - keep the probe there and answer the y/n questions; it finds the real output and fixes the map.
   - `q` - stop.
4. `save`, then the next leg (`check FML`, ...), or run `check` alone to go through all 8 legs.
5. `map` - all 24 should show `Wired yes` and no duplicate-channel warnings. `save`.

`find FL K` does step 3's locate on its own for a single wire.

With a calibrated clock and a precise scope readout, `ident confirm` + `which <width>` is a faster alternative.

## 3. Wiggle mapping (all legs connected) - preferred

1. Robot on a stand, legs free to move. Boot puts every output at 1500 us.
2. `wiggle`. For each output: watch, then answer leg (or `none`), joint, first-move direction.
3. At the end it lists any joints not found. `map` - check 24 joints `Wired yes`, no duplicate warnings.
4. `save`.

This replaces the harness check and the channel/direction parts of joint mapping below; limits
(min/max/neutral) still need setting per joint.

## 4. Joint limits (one joint at a time)

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
