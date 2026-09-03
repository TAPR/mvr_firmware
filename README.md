# MVR A2 Firmware

## Introduction
The MVR is a low-cost, modest-performance GPS disciplined oscillator (GPSDO).
It uses a Seeed Studios XIAO-RP2040 [Seeed XIAO RP2040](https://www.seeedstudio.com/XIAO-RP2040-v1-0-p-5026.htmli) compute module for configuration and
monitoring during operation.

At startup, the firmware configures the MVR on-board GNSS module and Si5351
synthesizer chip.  Thereafter, it monitors system status, showing lock condition
and number of satellites being tracked.  A menu system allows the synthesized
output frequencies to be changed; changes are persistent across power cycles.

**Status:** Validated on Linux Mint 22.3 + Arduino IDE 2.3.10, 2026-08-31.

---

## Install the RP2040 board package

1. Open Arduino IDE (2.x). If you don't have it yet, get it from arduino.cc — see the Flatpak/Snap note in Troubleshooting if you're on Linux and installing through a software center rather than downloading directly.
2. Go to **File > Preferences** (Mac: **Arduino IDE > Settings**).
3. In the **"Additional Boards Manager URLs"** field, paste:
```
   https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
```
   If there's already something in that box, add a comma and paste this after it.
4. Click **OK**.
5. Go to **Tools > Board > Boards Manager**, search `RP2040`.
6. Install **"Raspberry Pi Pico/RP2040/RP2350"** by **Earle F. Philhower, III**.
   - Note: there are two different RP2040 cores available. We use this community one, not Arduino's official "Arduino Mbed OS RP2040 Boards" package — it's more actively maintained, generally faster, and is what includes Seeed's XIAO RP2040 board definition.
7. This one install includes everything needed — compiler, uploader, the works. No separate downloads.

---

## Select the board

**Tools > Board > Raspberry Pi Pico/RP2040/RP2350 > Seeed XIAO RP2040**

---

## Connect the board

1. Plug the XIAO RP2040 into your computer with a USB-C cable.
2. Check **Tools > Port**.
   - A **brand-new board** (or one that's never had an Arduino sketch on it) may show up as **"UF2 Board"** instead of a normal port name. This is expected — see the BOOTSEL explanation above — select it and move on.
   - If **nothing** shows up at all, see "Port never appears" in Troubleshooting.

---

## Linux only: serial port permissions

Linux systems may require additional permissions to access the USB serial port.
If you get permission errors, open a terminal and run:

```
sudo usermod -aG dialout $USER
```

Then **log all the way out and back in** — closing and reopening just the terminal or the IDE isn't enough.

You may also need to create a udev rule.  In a terminal do:

```
sudo tee /etc/udev/rules.d/99-pico-bootsel.rules > /dev/null << 'EOF'
# RP2040 in BOOTSEL mode (needed for picotool access without sudo)
SUBSYSTEM=="usb", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="0003", MODE:="0666"
# RP2350 in BOOTSEL mode too, in case a XIAO RP2350 ever shows up on this bench
SUBSYSTEM=="usb", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="000f", MODE:="0666"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Then **unplug the board and replug it holding BOOTSEL again** — the rule only applies to a fresh connection — and retry Upload.


## Troubleshooting

### "No drive to deploy" (Linux)

Full error looks like:
```
Converting to uf2, output size: ...
Scanning for RP2040 devices
No drive to deploy.
Failed uploading: uploading error: exit status 1
```

The board may be in a state where the default UF2 upload method may not work.
Try the following:

1. **Tools > Upload Method > Picotool**
2. Put the board back in BOOTSEL mode if needed (hold the **BOOT** button while plugging in USB).
3. Upload again.

### picotool: "...appears to be in BOOTSEL mode, but picotool was unable to connect" (Linux)

Full error looks like:
```
No accessible RP-series devices in BOOTSEL mode were found.
but:
RP2040 device at bus X, address Y appears to be in BOOTSEL mode, but picotool
    was unable to connect. Maybe try 'sudo' or check your permissions.
Failed uploading: uploading error: exit status 249
```

### Flatpak-installed Arduino IDE can't find the board (Linux)

If Arduino IDE was installed through your distro's software center (common on Mint, Pop!_OS, Fedora) rather than downloaded directly from arduino.cc, it may be running in a Flatpak sandbox that blocks access to where the board's drive gets mounted (often under `/media` or `/run`). Symptoms look the same as "No drive to deploy" above. Fix, then restart the IDE:

```
flatpak override --user --filesystem=host:ro cc.arduino.IDE2
```

If this comes up a lot, it's simpler to just download the IDE directly from arduino.cc (ZIP or AppImage) instead of using the software center.

### Port never appears at all

- Try a different USB cable — some are power-only and carry no data.
- Try a different USB port on your computer, ideally not through a hub.
- Confirm the board is actually in BOOTSEL mode: hold **BOOT**, plug in USB, then release. Check `Tools > Port` again.

### Windows / Mac

These steps are Linux-specific (udev doesn't exist on Windows or Mac). Windows/Mac users generally don't need any permission fix for serial ports, though very old Windows versions occasionally need a driver from Zadig for UF2 mode specifically. This hasn't been tested against this project — if you hit something here, please add notes to this page.

---

## Install the MVR firmware's library dependencies

The MVR sketch itself needs one external library beyond what the board package already includes:

- **Adafruit NeoPixel** — drives the onboard RGB status LED on the XIAO RP2040 (green = GNSS fix + loop lock both good, amber = one of the two, red = neither, blue = booting). Install it via:

  **Sketch > Include Library > Manage Libraries**, search `NeoPixel`, install **Adafruit NeoPixel** by Adafruit.

The other library the sketch uses (`Wire`, for I2C to the Si5351) is bundled with the RP2040 board package from Section 2 — nothing extra to install there.

Once that's in place, the `mvr_firmware` sketch should compile without errors.

**Sketch folder note:** the folder containing `mvr_firmware.ino` must be named exactly `mvr_firmware` (no extension) — the Arduino IDE requires the sketch folder and the main `.ino` file to share a name. Renaming the folder makes the IDE treat it as a new/unrecognized sketch, which resets board, port, and upload-method selections (see the Default (UF2) field note above) — worth knowing before doing that again.
