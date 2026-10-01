# Huntsman - Project Context

Living summary of what this project is, what has been decided and why, and where it is up to.
**Keep this file current** - update it whenever hardware, decisions, status or next steps change.

_Last updated: 2026-10-01_

## What it is

An 8-legged spider robot ("Huntsman") that walks while carrying a load.
24 high-torque servos (8 legs x 3 joints), controlled by an ESP32-S3 through two PCA9685 PWM boards.
This is the second build; the first prototype ran on an Arduino UNO (its servo map is carried over).

## Hardware (as built)

| Part | Detail |
|---|---|
| Controller | ESP32-S3-WROOM-1U dev board (external-antenna module; needs a U.FL 2.4 GHz antenna for Wi-Fi/BT) |
| USB | CH340K USB-serial on the UART port, enumerates as **COM4** on the dev PC |
| PWM | 2x PCA9685 16-ch boards on I2C: SDA = GPIO8, SCL = GPIO9 |
| Board 1 | `0x40` - back legs (BL, BML, BR, BMR) |
| Board 2 | `0x41` (A0 bridged) - front legs (FL, FML, FR, FMR) |
| Servos | 24x ~55 kg brushless HV servos (270 deg, 0.5-2.5 ms), fed 8.5 V directly |
| Brain Box | Enclosure with the ESP32 + both PCA boards; Dupont connectors to the chassis |
| Power | 8.5 V feed into Brain Box -> 5 V buck -> ESP32 `5V0` pin -> ESP32 3V3 -> PCA9685 VCC. Servos take 8.5 V directly, not through the PCA boards |
| Planned battery | Makita 18 V 6 Ah (needs external low-voltage cutoff - Makita packs rely on the tool) |

Full wiring and channel map: [docs/hardware.md](docs/hardware.md).

## Key decisions (and why)

- **2x PCA9685 over Lynxmotion SSC-32U / Pololu Maestro** - servo power has to bypass any controller board
  anyway (24 high-torque servos draw 15-25 A walking, far beyond on-board traces), so the cheaper boards
  already owned are sufficient. ESP32 does interpolation/IK itself.
- **Pulses in microseconds, not PCA ticks** - each PCA9685's internal clock varies (~5-10%), so each board's
  oscillator is calibrated against an oscilloscope and stored; joint limits in us then mean the same thing on
  any board.
- **Board numbering 1/2 kept from the prototype** (1 = 0x40, 2 = 0x41) so the old map transfers directly.
- **Outputs start OFF at boot** - no pulses until commanded, so unverified limits can't drive a joint into
  its stops on power-up. (The prototype drove everything to neutral at boot.)
- **Calibration lives in ESP32 flash (NVS)**, survives firmware uploads; `export` prints it as C++ so it can
  be committed back into `servo_map.cpp` defaults.
- **Joint letters: K = knee, Y = lift (femur), X = swing (coxa)** - confirmed by user.
- **Channel map is not trusted** - full rebuild of the harness, so board/channel per joint is stored in flash
  and corrected on the robot (`assign`), not only in code. PCA9685 OE is not wired (outputs always enabled).
- **Toolchain: CLion 2026 + bundled PlatformIO plugin** (user prefers JetBrains; no VS Code).

## Current status

- [x] PlatformIO project builds and uploads (CLion + `pio` CLI)
- [x] I2C scan: both boards found (0x40, 0x41, plus 0x70 all-call)
- [x] Calibration console firmware with prototype servo map (limits converted from ticks, UNVERIFIED)
- [x] Harness tools: `ident` / `which` to identify outputs by scope, `assign` to rewire joints (saved to flash)
- [x] `find` (y/n bisection) for harness check - works without clock calibration
- [x] `check <leg>`: leg-by-leg harness check at the leg connector (wires can't be traced in the chassis)
- [ ] Harness check: all 24 joints `Wired yes` via `check`, saved  <- doing first.
      So far: user's `find FL K` saved FL K = board 1 ch 2 (prototype had that as BR X) - re-verify with `check FL`
- [ ] K/Y/X order within each leg connector: assumed unchanged, confirm with servos plugged in
- [ ] Board clock calibration (`cal` / `calf`), then `save` - deferred: scope readout only gives 2 digits at
      the default timebase (read 1.3 ms for a 1500 us command); zoom in, use cursors, or use `calf`
- [ ] Per-joint mapping: direction, min/max/neutral for all 24 joints
- [ ] Commit calibrated map back into `servo_map.cpp` (`export`)
- [ ] Leg geometry (segment lengths) -> inverse kinematics
- [ ] Gait (alternating tetrapod first, wave gait for heavy loads)
- [ ] Wi-Fi control page, battery voltage monitor (ADC1 pin, e.g. GPIO1/2)

## Open questions

- Module flash/PSRAM code (N8 / N16R8 ...) - project assumes N8, no PSRAM.
- Servos are fed 8.5 V; confirm their rated max (the reference 55 kg listing said 7.4 V HV).
- Signal level is 3.3 V from the PCA boards - confirm the servos respond reliably.

## Next step

1. Harness check with servos unplugged: `check FL` ... `check BR` (or `check`), `save`.
2. Calibrate each board's oscillator (`cal` / `calf`, `save`).
3. Map joints one servo at a time.

Procedures in [docs/calibration.md](docs/calibration.md).
