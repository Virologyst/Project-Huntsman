# Hardware

## Brain Box

| Item | Detail |
|---|---|
| MCU | ESP32-S3-WROOM-1U dev board (board profile `esp32-s3-devkitc-1`, assumed N8 / no PSRAM) |
| Antenna | WROOM-**1U** = U.FL connector, no PCB antenna. Fit a 2.4 GHz antenna before using Wi-Fi/BT; mount it away from servos and battery |
| USB | UART port via CH340K (COM4 on the dev PC). The native "USB" port would need `-DARDUINO_USB_CDC_ON_BOOT=1` |
| PWM | 2x PCA9685 16-channel, 12-bit (~4.9 us steps at 50 Hz) |
| Range | VL53L0X time-of-flight, forward-facing at the front of the body, on its own I2C bus (Wire1, GPIO 17/18, 0x29) |

## Power

```
8.5 V feed ──┬── servos (direct, via chassis wiring - NOT through PCA V+)
             └── 5 V buck ── ESP32 5V0 pin ── ESP32 3V3 ── PCA9685 VCC (x2)
All grounds common.
```

- Never power the ESP32 from USB and the 5 V buck at the same time unless there is a diode in the buck
  line - many S3 boards join USB 5 V to `5V0` directly.
- Servo current: ~0.1-0.3 A idle, 1-3 A holding load, 6-9 A stall each. Walking with load: 15-25 A total.
- Source: Makita 18 V 6 Ah (~108 Wh), mounted centrally. Makita LXT packs rely on the tool for low-voltage cutoff, so an
  external cutoff (~15 V) or ESP32 monitoring + disconnect is required.

## ESP32-S3 pins

| GPIO | Use |
|---|---|
| 8 | I2C SDA (both PCA9685) |
| 9 | I2C SCL (both PCA9685) |
| 17 | I2C SDA, second bus (Wire1) - VL53L0X |
| 18 | I2C SCL, second bus (Wire1) - VL53L0X |
| 3V3 | PCA9685 VCC (both boards), with GND - on the first two header pins per the user |
| 4 | Optional future: PCA9685 OE (both boards) as a software kill. **Not wired** - OE floats low on the boards, so outputs are always enabled |
| 1 or 2 | Planned: battery voltage divider (must be ADC1; ADC2 = GPIO11-20 is unusable with Wi-Fi on) |

Avoid: GPIO 0, 3, 45, 46 (strapping), 19/20 (USB), 35-37 (reserved if octal PSRAM).
The S3 has no fixed I2C pins - 8/9 are the Arduino-core defaults.

## I2C

| Address | Device |
|---|---|
| 0x40 | Board 1 - left legs |
| 0x41 | Board 2 - right legs (A0 bridged) |
| 0x70 | PCA9685 all-call (every board answers; normal) |
| 0x29 | VL53L0X ToF range sensor - on the second bus (GPIO 17/18), not with the PCA boards |

## ToF range sensor (VL53L0X)

| Pin | To |
|---|---|
| VIN | ESP32 3V3 (board has its own regulator; 3.3 V or 5 V both fine) |
| GND | GND |
| SDA / SCL | GPIO 17 / 18 - own bus (Wire1), not shared with the PCA boards (breakout has its own pull-ups) |
| XSHUT, GPIO1 | not connected |

Mounted at the front, pointing forward, roughly at obstacle height. Range ~1.2 m indoors, poor in direct
sunlight; readings above `TOF_MAX_MM` (2000) count as "nothing in range". Below `TOF_NEAR_MM` (30) the front
legs lift higher while walking (docs/motion.md); `tof` on the console shows the live reading.

## Channel map

Verified on the rebuilt robot with `wiggle` (2026-10-03); matches `DEFAULTS` in `src/servo_map.cpp`.
The live map is in ESP32 flash (`map`). Joint letters: **K = knee, Y = lift (femur), X = swing (coxa)**.
Dir: + = lift up, knee up, swing forward. Neutral (centre) is 1500 for all joints except FML X = 1650 (150 us forward) and FMR X = 1350 (150 us forward).

| Leg | Board | K (ch / dir) | Y (ch / dir) | X (ch / dir) |
|---|---|---|---|---|
| FL  | 1 (0x40) | 2 / -1  | 1 / -1  | 0 / +1  |
| FML | 1 (0x40) | 5 / +1  | 3 / +1  | 4 / +1  |
| BML | 1 (0x40) | 11 / -1 | 9 / -1  | 10 / +1 |
| BL  | 1 (0x40) | 12 / +1 | 14 / +1 | 13 / +1 |
| FR  | 2 (0x41) | 13 / +1 | 15 / +1 | 14 / -1 |
| FMR | 2 (0x41) | 10 / -1 | 9 / -1  | 11 / -1 |
| BMR | 2 (0x41) | 4 / +1  | 6 / +1  | 5 / -1  |
| BR  | 2 (0x41) | 1 / -1  | 0 / -1  | 2 / -1  |

Unused channels: board 1 - 6, 7, 8, 15; board 2 - 3, 7, 8, 12.
K/Y/X order within a leg's channels is not consistent (e.g. FL is X,Y,K on 0,1,2) - always go by the map.
