# Xbox controller

An Xbox Wireless Controller drives stand / sit / walk over **Bluetooth LE** (the ESP32-S3 has no Classic
Bluetooth). Works with Xbox Series X|S controllers and Xbox One controllers on current firmware (BLE
support); update an older controller in the Xbox Accessories app first. Code: `include/pad.h`,
`src/pad.cpp`; settings: `cfg::PAD_*` in `include/config.h`.

## Pairing

1. Power the robot. The boot log says `Controller: scanning`.
2. Turn the controller on and hold its **pair** button (top edge) until the Xbox logo flashes fast.
3. The console prints `Controller connected (<address>)`.

It reconnects automatically after a drop. To stop it pairing with a different controller in range, copy
the address into `cfg::PAD_ADDRESS`.

## Controls

| Input | Action |
|---|---|
| **A** | stand (same as `stand`); from a paused walk or climb, back to the stand pose |
| **B** | sit (same as `sit`) |
| Left stick / D-pad **up** | walk forward while held (release = pause in place) |
| Left stick / D-pad **down** | walk back while held |
| Left stick / D-pad **left / right** | turn left / right while held |
| **Left trigger** (LT), hold | climb wave (front to back) while held - pull harder = faster; release pauses in place. Also releases the 100 mm obstacle stop. Console: `climb [n]` |

- Walking starts from the stand pose (press **A** first) or carries on from a paused walk.
- **Releasing the stick pauses in place:** every joint stops mid-step and holds. Changing direction or the
  controller disconnecting does the same. Push the stick to carry on (any direction) - it resumes from
  where the legs are. **A** returns to the stand pose, **B** sits.
- The stick must pass half travel (`PAD_DEADZONE` = 0.5); the larger axis wins. D-pad overrides the stick.
- **Push further = faster:** walk speed goes from `WALK_MIN_SPEED` (0.4) just past the dead zone to full
  speed at full push, re-read every half step. The D-pad always walks at full speed.
- Stand and sit are still aborted by a console key, not the controller.

## Console

`pad` (also shown in `status`): connection, battery, left-stick x/y (-1..+1, + = right / forward) and the
walk speed it gives, A, B, LT (0-1023), D-pad. If pushing the stick forward shows a negative y, set `PAD_STICK_Y_SIGN = -1`.

## Build note

Wi-Fi + OTA + BLE need more than the default 1.25 MB app slot, so `platformio.ini` uses
`board_build.partitions = min_spiffs.csv` (1.9 MB app, OTA kept). **The first upload of this build must be
over USB** (`pio run -e usb -t upload --upload-port COM4`) - OTA can't change the partition table.
NVS (the calibration) sits at the same offset in both tables so it should survive, but run `export` and keep
the output before flashing just in case.
