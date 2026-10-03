# Huntsman - force the Wi-Fi (OTA) upload address.
# CLion's PlatformIO plugin passes its COM-port switcher value (e.g. COM1) as the upload port for every
# environment, which would replace huntsman.local. For OTA, any serial-looking port is swapped for the host.
Import("env")

host = env.GetProjectOption("custom_ota_host")
port = env.subst("$UPLOAD_PORT")
if not port or port.upper().startswith("COM") or port.startswith("/dev/"):
    env.Replace(UPLOAD_PORT=host)
    print(f"OTA upload to {host} (ignoring serial port '{port}')")
