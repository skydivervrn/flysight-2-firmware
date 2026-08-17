# Building & flashing (CLI, no STM32CubeIDE, no hardware programmer)

This branch builds with a self-contained `Makefile` and deploys via the official
FlySight ECIES file-drop mechanism — **no private/signing keys and no SWD probe
needed**. Firmware is encrypted to the device's per-batch PUBLIC key
(`Deploy/Public_Keys/pub_key_bX.bin`, published by the maintainer).

## 0. One-time setup

```bash
# (a) Toolchain: official Arm GNU 12.3 tarball, extracted to ~/opt (no sudo).
#     Do NOT use GCC 14.x (firmware installs but does not boot) and do NOT use
#     Homebrew arm-none-eabi-gcc (ships without newlib -> no stdint.h).
mkdir -p ~/opt && cd ~/opt
curl -fSL -o armgnu12.tar.xz \
  https://developer.arm.com/-/media/Files/downloads/gnu/12.3.rel1/binrel/arm-gnu-toolchain-12.3.rel1-darwin-x86_64-arm-none-eabi.tar.xz
tar -xf armgnu12.tar.xz

# (b) Deploy venv (Deploy/.venv is gitignored — create it):
cd <repo>/Deploy
python3 -m venv .venv && . .venv/bin/activate
pip install coincurve pycryptodome
```

## 1. Build

```bash
make        # auto-detects ~/opt/arm-gnu-toolchain-12.3*; or pass PREFIX=.../bin/arm-none-eabi-
```

Expected: FLASH ~36%, `build/UserApp.bin` written. The Makefile mirrors the
CubeIDE Release config (cortex-m4, fpv4-sp-d16 hard, -Os) and already carries
the two MUST-HAVE flags (see Pitfalls).

## 2. Host unit tests

```bash
cd Tests && make run
```

## 3. Encrypt for your batch → APP.SFB

Find your batch in the device's `flysight.txt` (or match `Pubkey_X` against
`Deploy/Public_Keys/`). For batch B2:

```bash
cp build/UserApp.bin Deploy/Firmware_As_Built/UserApp.bin
cd Deploy && . .venv/bin/activate
python -c "import deploy_firmware as d; d.encrypt_firmware_with_key('Firmware_As_Built/UserApp.bin','UserApp','Public_Keys/pub_key_b2.bin','B2')"
```

Sanity check — the output must start with ASCII `SFU1`:

```bash
xxd Firmware_To_Deploy/B2_UserApp.sfb | head -1
```

## 4. Flash via the official file-drop

> ⚠️ `APP.SFB` is consumed ONLY by the bootloader. Copying it while the device
> is in normal USB mass-storage mode does nothing — the file just sits there.

1. Back up your logs.
2. Hold the FlySight button ~10 s until the LED turns **orange** — the
   bootloader mounts as a USB drive.
3. Copy the .sfb to the device as `FW/APP.SFB` (create `FW/` if missing).
4. Eject, unplug, wait ~10 s — the bootloader applies the update and reboots.
5. Verify: re-mount; `FW/APP.SFB` is gone and `Firmware_Ver` in `flysight.txt`
   reflects the new build. The HUD also shows its own version at the end of the
   info line (`HUD_VERSION` in `FlySight/activelook_mode0.c`).

Recovery: the bootloader always persists. If a build does not boot, re-flash an
official `APP.SFB` (flysight.ca) the same way — no programmer needed.

## 4b. Or flash it over Bluetooth, from the Mac

`Tools/flysight_ble.py` does step 4 without a cable or a tablet: `--info` reads
version/batch/mode/battery/MTU, `--upload` proves the transfer on a harmless
path, and `--flash` uploads to `/FW/APP.SFB` and — only with `--really-install`
— tells the bootloader to install it. Runbook, gates and the bonding dance:
**`Docs/FLASH_OVER_BLE.md`**.

```bash
~/.venvs/ble/bin/python Tools/flysight_ble.py --info   # macOS sandbox OFF
```

USB stays the only route for three things: the **first bond** with a new Mac,
firmware older than `v0.0.12` (no install-over-BLE command), and **recovery**
from a failed install — the bootloader speaks no Bluetooth.

## Pitfalls (each of these cost a debug cycle)

1. **GCC 14.x → app installs but won't boot.** Use Arm GNU **12.3** (matches
   ST's gnu-tools codegen). The Makefile auto-detect is pinned to `12.3*`.
2. **Homebrew arm-none-eabi-gcc → no newlib** (missing stdint.h). Use the tarball.
3. **`-fno-tree-loop-distribute-patterns` is required** — GCC 12+ otherwise
   rewrites startup init loops into memcpy/memset calls that run before the C
   runtime is ready → boot crash. Already in Makefile CFLAGS.
4. **`-u _printf_float` is required** — newlib-nano drops `%f` support by
   default; without it every float renders as an EMPTY string (labels show,
   numbers don't). Already in Makefile LDFLAGS.
5. **Old device firmware**: units shipped before ~v2024 predate the ECIES
   deploy scheme — update to an official recent build first.
6. Do not use the legacy `postbuild.bat` SBSFU signing path — it needs private
   keys and is superseded by `Deploy/deploy_firmware.py`.

## Key files

- `FlySight/activelook_mode0.c` — HUD render, line definitions, baro altitude.
- `FlySight/activelook.c` — ActiveLook app FSM (discovery → clear → update).
- `FlySight/activelook_proto.{c,h}` — pure protocol helpers (host-tested).
- `FlySight/engo_bind.c` — pin the HUD to one pair of glasses (`/engo3.txt`).
- `FlySight/flight_detect.c` — takeoff detection (drives the header marker).
- `STM32_WPAN/App/app_ble.c`, `activelook_client.c` — BLE central + flow control.
- `Tools/engo_mac_*.py` — Mac BLE bench: render mock HUD screens on the glasses
  in seconds, probe fonts, no reflash (see `Docs/HUD_LAYOUT.md`).
