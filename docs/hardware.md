# Hardware

## Brain Box

| Item | Detail |
|---|---|
| MCU | ESP32-S3-WROOM-1U dev board (board profile `esp32-s3-devkitc-1`, assumed N8 / no PSRAM) |
| Antenna | WROOM-**1U** = U.FL connector, no PCB antenna. Fit a 2.4 GHz antenna before using Wi-Fi/BT; mount it away from servos and battery |
| USB | UART port via CH340K (COM4 on the dev PC). The native "USB" port would need `-DARDUINO_USB_CDC_ON_BOOT=1` |
| PWM | 2x PCA9685 16-channel, 12-bit (~4.9 us steps at 50 Hz) |

## Power

```
8.5 V feed ──┬── servos (direct, via chassis wiring - NOT through PCA V+)
             └── 5 V buck ── ESP32 5V0 pin ── ESP32 3V3 ── PCA9685 VCC (x2)
All grounds common.
```

- Never power the ESP32 from USB and the 5 V buck at the same time unless there is a diode in the buck
  line - many S3 boards join USB 5 V to `5V0` directly.
- Servo current: ~0.1-0.3 A idle, 1-3 A holding load, 6-9 A stall each. Walking with load: 15-25 A total.
- Planned source: Makita 18 V 6 Ah (~108 Wh). Makita LXT packs rely on the tool for low-voltage cutoff, so an
  external cutoff (~15 V) or ESP32 monitoring + disconnect is required.

## ESP32-S3 pins

| GPIO | Use |
|---|---|
| 8 | I2C SDA (both PCA9685) |
| 9 | I2C SCL (both PCA9685) |
| 3V3 | PCA9685 VCC (both boards), with GND - on the first two header pins per the user |
| 4 | Optional future: PCA9685 OE (both boards) as a software kill. **Not wired** - OE floats low on the boards, so outputs are always enabled |
| 1 or 2 | Planned: battery voltage divider (must be ADC1; ADC2 = GPIO11-20 is unusable with Wi-Fi on) |

Avoid: GPIO 0, 3, 45, 46 (strapping), 19/20 (USB), 35-37 (reserved if octal PSRAM).
The S3 has no fixed I2C pins - 8/9 are the Arduino-core defaults.

## I2C

| Address | Device |
|---|---|
| 0x40 | Board 1 - back legs |
| 0x41 | Board 2 - front legs (A0 bridged) |
| 0x70 | PCA9685 all-call (every board answers; normal) |

## Channel map

From the Arduino UNO prototype. The robot is a **full rebuild**, so this is likely to have changed - verify
with the harness check (`ident` / `which`, see calibration.md) and correct with `assign`. The live map is in
ESP32 flash (`map`); update this table from `export` once verified.
Joint letters: **K = knee, Y = lift (femur), X = swing (coxa)**.

| Leg | Board | K | Y | X | Dir |
|---|---|---|---|---|---|
| FL  | 2 (0x41) | 4  | 5  | 6  | -1 |
| FML | 2 (0x41) | 0  | 1  | 2  | -1 |
| FR  | 2 (0x41) | 9  | 10 | 11 | +1 |
| FMR | 2 (0x41) | 13 | 14 | 15 | +1 |
| BL  | 1 (0x40) | 9  | 10 | 11 | +1 |
| BML | 1 (0x40) | 13 | 14 | 15 | +1 |
| BR  | 1 (0x40) | 0  | 1  | 2  | -1 |
| BMR | 1 (0x40) | 4  | 5  | 6  | -1 |

Unused channels on each board: 3, 7, 8, 12.
