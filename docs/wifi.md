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
reboots and (with `BOOT_STAND`) stands up again. Same from a paused walk or climb pose.

CLion's PlatformIO plugin passes its COM-port switcher value (e.g. COM1) as the upload port even for Wi-Fi;
`scripts/ota_port.py` swaps any serial port back to `custom_ota_host` (huntsman.local) for the wifi env.

If `huntsman.local` doesn't resolve, use the IP address printed at boot in `upload_port`.

Troubleshooting: the console prints Wi-Fi status (not found / failed / lost) on the UART port. The
first connection only worked after fitting the antenna and turning off the router's Smart Connect
(2.4/5 GHz band steering) so the ESP32 sees a 2.4 GHz VNet.

## Second network: the Pi 5 payload's hotspot

The Pi 5 rides along as a payload (it does not control the robot). Away from home it can run a hotspot so a
laptop can still push updates:

1. On the Pi (Raspberry Pi OS, NetworkManager), once:
   `sudo nmcli device wifi hotspot ifname wlan0 ssid Huntsman password <password>` then make it start at
   boot: `sudo nmcli connection modify Hotspot connection.autoconnect yes`. Use 2.4 GHz if asked
   (`band bg`) - the ESP32 can't see 5 GHz.
2. In the git-ignored `include/secrets.h`, uncomment `WIFI_SSID_2` / `WIFI_PASSWORD_2` and fill them in.
   Upload once (at home, over VNet).
3. In the field: the ESP32 tries each known network in turn (`WIFI_TRY_MS` = 15 s each, then every
   `WIFI_IDLE_TRY_MS` = 60 s while none is found - non-blocking, so the gait never stalls, and the slower
   retries leave the radio to the controller). Join the laptop to the Pi's hotspot; CLion Upload and the
   Wi-Fi monitor reach `huntsman.local` as at home. The boot / connect line says which network it joined.

With only `WIFI_SSID` set it behaves as before (one network, auto-reconnect).

## Console over Wi-Fi

Raw TCP on port 23 - the same console as USB. Output goes to both; input is accepted from either.
One network client at a time (a new connection replaces the old one).

- CLion: PlatformIO Serial Monitor (default env `wifi`: `monitor_port = socket://huntsman.local:23`).
  After an upload, wait until the robot has rebooted and stood (~10 s) - "Upload and Monitor" connects
  too early and fails with `getaddrinfo failed`.
- Command line: `pio device monitor -e wifi`

Long-running commands (prompts in `wiggle`/`check`, walking) block the loop, so OTA uploads wait until they
finish.

## Security

No OTA password: anyone on the network can upload firmware. Fine on a trusted home network; add
`ArduinoOTA.setPassword()` (and `upload_flags = --auth=...` from a git-ignored file) if that changes.
