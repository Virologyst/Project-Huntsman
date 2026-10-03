# Huntsman

Firmware for an 8-legged, load-carrying spider robot: ESP32-S3 + 2x PCA9685 driving 24 high-torque servos.

- **Project context, decisions and status:** [CONTEXT.md](CONTEXT.md)
- **Hardware, wiring and channel map:** [docs/hardware.md](docs/hardware.md)
- **Calibration console and procedures:** [docs/calibration.md](docs/calibration.md)
- **Motion (stand / sit):** [docs/motion.md](docs/motion.md)

## Current firmware

A serial **calibration console**: calibrate each PWM board's clock against an oscilloscope, then map and
tune every leg joint by name (`FR X 1600`, `FML Y+20`, `setmin BL K`, `save`). On boot every output is
driven to 1500 us (centre); `limp` releases them.

## Build and upload

Tooling: **CLion 2026** with the bundled PlatformIO plugin, plus PlatformIO Core
(`%USERPROFILE%\.platformio\penv\Scripts` on PATH).

In CLion: **Tools > PlatformIO > Upload**, then **PlatformIO Serial Monitor** (115200 baud).
Close the serial monitor before uploading, or the COM port is busy.

Command line:

```bash
pio run -t upload --upload-port COM4
```

```bash
pio device monitor -p COM4 -b 115200
```

If an upload won't start: hold **BOOT**, tap **RST**, release BOOT, retry.

## Layout

```
include/config.h       pins, I2C addresses, hard pulse limits
include/pwm.h          PCA9685 control, per-board clock calibration   (src/pwm.cpp)
include/servo_map.h    leg/joint -> board/channel map + calibration   (src/servo_map.cpp)
include/motion.h       ramps, stand / sit sequences                    (src/motion.cpp)
include/console.h      serial command console                          (src/console.cpp)
src/main.cpp           setup/loop
docs/                  hardware and calibration docs
```
