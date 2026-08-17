# Flashing a FlySight 2 from a Mac, over Bluetooth

`Tools/flysight_ble.py` talks to a FlySight 2 directly over BLE: it reads the
device's identity, writes files to its card, and — if you insist, in words —
uploads a firmware image and tells the bootloader to install it. No Android
tablet, no USB cable, no bootloader button dance.

**It has never been run against a device.** Everything below was assembled from
the firmware in this repo, the Groundrush app's device layer, and 63 host tests
that model the firmware's own state machine. The first hardware run is the
proving run: do it with `--info`, then `--upload`, and only then `--flash`.

---

## When the Mac route works, and when it does not

| Situation | Route |
|---|---|
| Routine update of a working unit | **Mac, over BLE** |
| Reading version / batch / battery without unmounting anything | **Mac, `--info`** |
| The Mac has never been bonded to this FlySight | USB or tablet first — see *Bonding*, below |
| The device runs firmware older than `v0.0.12` | **USB only.** `DS_CMD_INSTALL_UPLOADED_FIRMWARE` (0x04) does not exist before commit `cff0b12`; the control point answers `not supported` |
| An install failed and the unit sits in its bootloader (orange LED, USB drive only) | **USB only.** The bootloader speaks no Bluetooth |
| The device is recording, in USB mode, or below 50 % battery | Neither — the tool refuses, and so should you |

The FlySight stops advertising while any central is connected
(`app_ble.c:585`), so close the app on the phone/tablet before scanning from
the Mac, and disconnect the Mac before going back to the tablet.

---

## Setup

```bash
# bleak lives in the same venv the ENGO bench tools use
~/.venvs/ble/bin/python -c "import bleak; print('ok')"
```

**Turn the macOS sandbox off for these runs.** CoreBluetooth is not reachable
from a sandboxed process — a sandboxed run finds no devices at all, which looks
exactly like a FlySight that is switched off.

### Bonding — the one manual step

Every user-data characteristic on the FlySight requires encryption, and
`bleak.pair()` **raises `NotImplementedError` on CoreBluetooth** (verified in
`bleak/backends/corebluetooth/client.py`: "Pairing is not available in Core
Bluetooth"). macOS pairs implicitly instead, when something reads a
characteristic that demands encryption. So the first time:

1. Put the FlySight in **PAIRING** mode: double-press the button from idle. The
   LED pulses green for 30 s.
2. Run `--info` from the Mac within that window.
3. Accept the macOS pairing dialog when it appears.

Afterwards the bond persists and plain `--info` works from idle.

**A stale bond has to be cleared on both sides**, or reconnects fail in ways
that look like a broken radio:

* on the Mac: System Settings → Bluetooth → the FlySight → *Forget This Device*;
* on the device: set `Reset_BLE: 1` in `FLYSIGHT.TXT` over USB (it clears the
  security database at the next boot).

Doing only one of the two is the classic "it used to connect and now it never
will" failure.

---

## 1. `--info` — reads only, writes nothing

```bash
~/.venvs/ble/bin/python Tools/flysight_ble.py --info
```

```
Scanning 10 s for a FlySight…
Found 'FlySight' at 9A1B…-…-… (-52 dBm)

  ATT MTU:       250
  Mode:          SLEEP
  Battery:       87 %
  Firmware:      v0.0.19-n.caf1   (BLE reports at most 16 characters of the tag)
  Device ID:     0123456789abcdef01234567

  Firmware_Ver:  v0.0.19-n.caf1a19   (complete tag, from /flysight.txt)
  Stack_Ver:     1.19.0
  FUS_Ver:       1.2.0
  Device_Name:   FlySight
  Pubkey_X:      486bee2d…  → batch B2
```

This is the command that replaces most of the tablet fiddling: version, batch,
mode, battery and MTU without mounting anything.

Two things it will tell you that are easy to misread:

* **`Battery: 0 % (never measured)`** is not a flat battery. The firmware only
  samples the ADC in ACTIVE (`FlySight/active_control.c:246-256` returns early
  in every other mode) and the characteristic starts at 0 on each boot
  (`custom_app.c:109,417`). Wake the device to ACTIVE for a few seconds, put it
  back to SLEEP, and read it again.
* **`/flysight.txt was NOT read`** means the device was not in SLEEP. Every CRS
  command claims the SD card (`crs.c:245`), and ACTIVE/START/CONFIG hold it for
  as long as they run — so the batch and the full version are unreadable then.
  This tool never changes the device's mode by itself; put it to sleep and run
  `--info` again.

---

## 2. `--upload` — prove the writer at zero risk

```bash
~/.venvs/ble/bin/python Tools/flysight_ble.py \
    --upload build/UserApp.bin --dest /TEST.BIN
```

Writes the file, reads it straight back off the card and compares sha256:

```
build/UserApp.bin: 158016 bytes, sha256 3f2a…
  up: 158016/158016 B (100 %) 31.2 kB/s
  down: 158016 B 33.0 kB/s
Round trip OK: 158016 bytes, sha256 3f2a… — the Go-Back-N uploader is sound on this link.
```

Do this **before** the first `--flash` on any Mac. It exercises the entire
transfer path — window, counters, retransmission, end-of-file — against a file
whose corruption costs nothing. A mismatch here is a bug to fix, not a warning
to click through. `/TEST.BIN` is left on the card; it is harmless and the next
upload overwrites it.

Expect roughly **28–33 kB/s** — measured from Android; nobody has measured it
from a Mac yet. A 160 kB image is therefore ~5 s each way.

---

## 3. `--flash` — the real thing

```bash
# dry run: uploads and verifies, does NOT install
~/.venvs/ble/bin/python Tools/flysight_ble.py \
    --flash Deploy/Firmware_To_Deploy/B2_UserApp.sfb

# and, once every line above says ok:
~/.venvs/ble/bin/python Tools/flysight_ble.py \
    --flash Deploy/Firmware_To_Deploy/B2_UserApp.sfb --really-install
```

**`--flash` is a dry run by default.** It does everything — gates, `mkdir /FW`,
upload to `/FW/APP.SFB`, read-back and digest comparison — except send
`DS_CMD_INSTALL_UPLOADED_FIRMWARE` (0x04). Until that opcode goes out, the
image is only a file on the memory card and the device still boots exactly what
it booted before. `--really-install` is the only thing that changes that.

### The gates, and what each refusal means

| Gate | Refuses when | Why it matters |
|---|---|---|
| SFU1 magic | the first four bytes are not `SFU1` | you are about to send a raw `.bin` or the wrong file entirely |
| sha256 | a digest was given (`--sha256`, or a `sha256sums.txt` beside the image) and disagrees | a truncated download installs and bricks |
| batch | the `B2`/`B3`/… in the file name is not the batch behind the device's `Pubkey_X` | the image is encrypted to one batch's key; another batch's bootloader cannot decrypt it |
| mode | the device is not in SLEEP | file writes are NAK'd in every other mode |
| battery | 0 % (unmeasured) or below 50 % | losing power between the trigger and the bootloader finishing needs a USB recovery |
| ATT MTU | below 247 | a 244-byte frame would not fit a write-without-response, and the image would silently truncate |

If **no** digest is available the run continues with a `WARN` and prints the
digest it computed. That is a deliberate judgement, not an oversight: an image
you have just built locally has nothing to be compared against, and the dry-run
default plus the on-device read-back are the real guards. Pass `--sha256` when
you have a published digest.

### Why it reads the image back before offering to install

`FS_CRS_State_Write` closes the file on **any** two-byte data packet, whatever
its counter says (`FlySight/crs.c:558-561` — the check sits outside the counter
test at `:547`). An end-of-file frame that slips out while a data frame is
still unacknowledged therefore leaves a **truncated** image on the card, with
no error anywhere: every retransmission afterwards comes back as `NAK 0x10` and
the file is already closed. Install that and the unit sits in its bootloader
until someone brings a USB cable.

Two things guard against it:

* the uploader holds the end-of-file frame back until every data frame has been
  acknowledged (`UploadWindow` in `Tools/flysight_ble.py`, and the test named
  `test_an_eager_uploader_would_truncate_the_image`, which demonstrates what
  the naive version does);
* `--flash` reads `/FW/APP.SFB` back off the card and compares sha256 before it
  will even offer to install.

### When it goes wrong

A refused install (`not permitted`, `busy`, `not supported`) leaves the device
running its old firmware with a stray file in `/FW`. Nothing is broken; delete
it over USB if you like, or upload again.

An install that is *accepted* and then fails leaves the unit in its bootloader:
orange LED, mounts as a USB drive, no Bluetooth. Recover exactly as in
`Docs/BUILDING.md` §4 — hold the button ~10 s for the bootloader, drop a known
good `B?_….sfb` in as `FW/APP.SFB`, eject, wait. **That path is the reason the
USB route can never be retired.**

---

## Tests

Everything the tool decides before it touches a device is pure and tested on
the Mac, with no radio and no bleak:

```bash
cd Tests && make run                 # C tests + these
cd Tests && python3 test_flysight_ble.py -v   # just these
```

63 tests: the CRS framing; the Go-Back-N window (loss, duplicate ACKs, counter
wrap past 256 frames, the end-of-file rule); the `flysight.txt` parse; the
batch table, re-derived from `Deploy/Public_Keys/pub_key_b*.bin` rather than
trusted; every flash gate; and each subcommand driven end to end against a
Python model of `FS_CRS_State_Idle/_Read/_Write`, including the assertion that
a dry run never sends the install opcode.

---

## What is verified, and what is not

Verified in source (file:line in the tool's comments): the UUIDs, every opcode,
the 242-byte frame, the 8-packet window and the device's matching receive ring,
the truncate-on-any-two-byte-packet behaviour, the mode gating, the battery
being sampled only in ACTIVE, the 16-character version truncation, the batch
prefixes, and `bleak.pair()` raising on CoreBluetooth.

**Inferred, not verified:** that the SBSFU bootloader picks the image up from
`/FW/APP.SFB` when it is delivered over BLE rather than over USB. The
bootloader is a prebuilt binary and names the path nowhere in these sources; it
is the path the USB file-drop uses, and the app assumes the same.

**Unmeasured:** BLE throughput from macOS, and whether CoreBluetooth's
fire-and-forget writes (bleak does not wait for
`peripheralIsReadyToSendWriteWithoutResponse`) drop frames at a full 8-frame
window. If uploads stall and recover repeatedly, lower `--window`.
