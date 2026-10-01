# Huntsman - notes for Claude

Read [CONTEXT.md](CONTEXT.md) first: hardware, decisions, status, open questions.

## Keep docs current

After any change to hardware facts, wiring, pin use, console commands, calibration procedure, decisions or
project status, update in the same change:
- `CONTEXT.md` - status checklist, decisions, open questions, next step, "Last updated" date
- `docs/hardware.md` - wiring, pins, channel map
- `docs/calibration.md` - console commands and procedures
- `README.md` - only if build steps or layout change

## Build / upload

PlatformIO Core is at `%USERPROFILE%\.platformio\penv\Scripts\pio.exe` (user runs CLion 2026 + PlatformIO plugin).

```bash
pio run                                  # build
pio run -t upload --upload-port COM4     # upload (fails if a serial monitor has COM4 open)
```

## Safety

- Real 55 kg servos may be connected. Never command pulses, `all`, sweeps or joint moves to test code
  unless the user has said nothing is connected or asked for it. Read-only commands (`map`, `status`,
  joint queries, invalid input) are fine for testing.
- Boot behaviour is set by `cfg::BOOT_PULSE_US` (currently 1500 on all 32 outputs, at the user's request;
  0 = boot with outputs off). Don't change it without asking.
- Never upload while the user has the serial monitor open (it holds COM4) - ask them to close it first.
- Keep pulses in microseconds; hard limits live in `include/config.h`.

## Style

C++ (Arduino core on ESP32-S3), 4-space indent, namespaces per module (`pwm`, `servos`, `console`, `cfg`).
