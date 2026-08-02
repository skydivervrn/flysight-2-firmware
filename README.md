# FlySight 2 firmware with ENGO 3 (ActiveLook) HUD

Unofficial [FlySight 2](https://www.flysight.ca/) firmware that connects
**directly** to ENGO 3 / ActiveLook smart glasses over BLE and shows live
flight data on the HUD — **no phone in the loop**.

Based on the official [flysight/flysight-2-firmware](https://github.com/flysight/flysight-2-firmware)
`develop` branch. Everything else (logging, USB, config) works as stock.

## What you see in the glasses

```
X A:87% F:76% N:21 v0.0.12     <- status: takeoff marker, glasses/FlySight
                                  battery, satellites, firmware version
143      52                    <- horizontal | vertical speed, km/h (GPS, raw)
2.75                           <- glide ratio
1043                           <- barometric altitude, m (zeroed at power-on)
```

Big digits (64–75 px), no labels, ~3 Hz refresh. Speeds are raw GPS ground
speed / velD in km/h ("+" vertical = down) with no air-density (SAS)
correction, so they match Skyderby's ground-speed charts. Details and layout
coordinates: [Docs/HUD_LAYOUT.md](Docs/HUD_LAYOUT.md).

Extra features over stock:

- **Auto-reconnect** — glasses can be powered on before or after the FlySight;
  the link self-heals after signal loss.
- **Glasses binding** — the HUD pins itself to the first pair of glasses it
  links with (serial stored in `/engo3.txt` on the device; delete the file to
  re-bind).
- **Takeoff detection** — the first header character flips `X` → `V` once a
  climb is detected.
- **Mac BLE bench** (`Tools/`) — iterate on HUD layouts against real glasses
  in ~15 seconds without reflashing the device.

## Requirements

- FlySight 2 on a reasonably recent official firmware (≥ v2024.x — older units
  predate this update mechanism; update once via [flysight.ca](https://www.flysight.ca/) first).
- ENGO 2 / ENGO 3 or other ActiveLook-based glasses (developed and tested on
  ENGO 3, glasses firmware 4.13.x).

## Install a prebuilt release (no tools needed)

**1. Find your device batch.** Plug the FlySight in as USB storage, open
`flysight.txt`, copy the `Pubkey_X:` value and match it here:

| Batch | `Pubkey_X` starts with |
|-------|------------------------|
| B2    | `486bee2d3dd60fd0`     |
| B3    | `211d721dbd9114a6`     |
| B4    | `dac40d2597e37139`     |
| B5    | `3157e02846ddfd2a`     |
| B6    | `8ee7870905b3e792`     |

Firmware is encrypted per batch — a file for the wrong batch is simply
rejected by the bootloader (harmless, but it won't install).

**2. Download** the `.sfb` for your batch from
[Releases](../../releases) and verify its checksum against `sha256sums.txt`.

**3. Back up your logs** (copy them off the USB drive).

**4. Enter the bootloader:** hold the FlySight button ~10 s until the LED
turns **orange** — it re-mounts as a USB drive. *(Copying the file in normal
USB mode does nothing — this step is required.)*

**5. Copy** the downloaded file to the device as `FW/APP.SFB`
(create the `FW` folder if missing, and rename the file to exactly `APP.SFB`).

**6. Eject, unplug, wait ~10 s.** The bootloader installs the update and
reboots.

**7. Verify:** power the glasses on, wait for the link (up to ~45 s) — the
status line should end with the release version (e.g. `v0.0.12`). On the USB
drive, `FW/APP.SFB` is gone (consumed) and `Firmware_Ver:` in `flysight.txt`
has changed.

## Connecting glasses & binding

**First connect:** just power the glasses on — before or after the FlySight,
order doesn't matter. The FlySight scans continuously in ACTIVE mode and links
within ~20–45 s of both being on. No pairing/bonding dialogs, no phone app.

**Binding (automatic):** on the first successful link the FlySight pins itself
to that pair of glasses — their 6-character serial (the tail of the advertised
name, e.g. `ENGO 3 123456` → `123456`) is written to `engo3.txt` in the root
of the FlySight USB drive. From then on it connects ONLY to those glasses —
important on a dropzone where several people run ActiveLook HUDs.

**Switch to different glasses / unbind:** delete `engo3.txt` from the USB
drive (normal USB mode is fine). Next power-up the FlySight is unbound,
links to the first ActiveLook glasses it finds, and re-binds to them.

**Pin specific glasses manually:** create `engo3.txt` containing the serial —
the last 6 characters of the glasses' Bluetooth name (shown in the ActiveLook
phone app, in a BLE scanner, or printed as `ENGO found: '...'` in the
FlySight's `EVENT.CSV`). Accepted formats: `123456`, `ID: 123456`, or the
full name `ENGO 3 123456` — the last 6 characters are used.

**Verify:** each session's `EVENT.CSV` logs the bind state at boot
(`ENGO bind: pinned to serial 123456` / `unbound -> connect to first`) and
`ENGO bind: linked to glasses serial 123456` once connected.

## Rollback to official firmware

The bootloader always survives, so you can never brick the device this way:

1. Download the official firmware for your device from
   [flysight.ca](https://www.flysight.ca/) (the site picks the right file for
   your unit), or use the firmware endpoint described in
   [Docs/firmware.md](Docs/firmware.md).
2. Repeat install steps 4–6 with the official file as `FW/APP.SFB`.

That's it — the device is back to stock, logs untouched.

## Build from source

See [Docs/BUILDING.md](Docs/BUILDING.md) — CLI build (no STM32CubeIDE), Arm
GNU 12.3, plus the ECIES encrypt + file-drop flash flow and every pitfall we
hit on the way.

## Feedback / issues

Please open a [GitHub issue](../../issues) — include your `flysight.txt`
`Firmware_Ver`, the HUD version from the glasses, and (for connection issues)
your glasses model and firmware version. Attaching the newest session's
`EVENT.CSV` from the device helps a lot — since v0.0.13 it records the whole
BLE link lifecycle.

Prefer email? Write to **flysight@domovionok.com**.

## Disclaimer

This is **unofficial** firmware for skydiving equipment, provided as-is,
without any warranty. A HUD is an aid, not an altimeter replacement — always
fly with (and trust) your certified instruments. Test the firmware on the
ground and on solo jumps before relying on it. Rollback to official firmware
is always available via the file-drop above.
