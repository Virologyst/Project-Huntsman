# Wi-Fi: uploads and console

The firmware joins the network in `include/secrets.h` (git-ignored; template in `include/secrets.example.h`)
and appears as **`huntsman.local`**. The ESP32-S3-WROOM-**1U** needs its U.FL antenna fitted for usable range.

On connect the console prints: `Wi-Fi connected: huntsman.local (<ip>), console on port 23`.

## Uploads over Wi-Fi (OTA)

PlatformIO env **`wifi`** (in `platformio.ini`) uploads with `espota` to `huntsman.local`.

- **`wifi` is the default environment** (`default_envs` in platformio.ini), so CLion's normal
  Tools > PlatformIO > Upload and Serial Monitor go over Wi-Fi.
- For USB (recovery, or if Wi-Fi firmware is broken): set `default_envs = usb`, then
  Tools > PlatformIO > Reload PlatformIO Project.
- Command line: `pio run -e wifi -t upload`

The first Wi-Fi-capable firmware must go on over USB. After that, every build includes OTA, so Wi-Fi uploads
keep working. If the robot is **standing** when an update starts, it **sits down first**; after the update it
reboots and (with `BOOT_STAND`) stands up again.

If `huntsman.local` doesn't resolve, use the IP address printed at boot in `upload_port`.

Troubleshooting: the console prints Wi-Fi status (not found / failed / lost) on the UART port. The
first connection only worked after fitting the antenna and turning off the router's Smart Connect
(2.4/5 GHz band steering) so the ESP32 sees a 2.4 GHz VNet.

## Console over Wi-Fi

Raw TCP on port 23 - the same console as USB. Output goes to both; input is accepted from either.
One network client at a time (a new connection replaces the old one).

- CLion: PlatformIO Serial Monitor (default env `wifi`: `monitor_port = socket://huntsman.local:23`).
- Command line: `pio device monitor -e wifi`

Long-running commands (prompts in `wiggle`/`check`, walking) block the loop, so OTA uploads wait until they
finish.

## Security

No OTA password: anyone on the network can upload firmware. Fine on a trusted home network; add
`ArduinoOTA.setPassword()` (and `upload_flags = --auth=...` from a git-ignored file) if that changes.
