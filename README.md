# FlySight 2 firmware with ENGO 3 (ActiveLook) HUD

Unofficial [FlySight 2](https://www.flysight.ca/) firmware that connects
**directly** to ENGO 3 / ActiveLook smart glasses over BLE and shows live
flight data on the HUD — **no phone in the loop**.

Based on the official [flysight/flysight-2-firmware](https://github.com/flysight/flysight-2-firmware)
`develop` branch. Everything else (logging, USB, config) works as stock.

## What you see in the glasses

```
X A:87% F:76% N:21 v0.0.14     <- status: takeoff marker, glasses/FlySight
                                  battery, satellites, firmware version
143      52                    <- horizontal | vertical speed, km/h (GPS, raw)
2.75                           <- glide ratio
1043                           <- barometric altitude, m (zeroed at power-on)
```

Big digits (64–75 px), no labels, ~3 Hz refresh. Speeds are raw GPS ground
speed / velD in km/h ("+" vertical = down) with no air-density (SAS)
correction, so they match the ground-speed charts used for track analysis.
Details and layout
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

---

<a id="requirements"></a><a id="install"></a>

# Installing it — step by step

**No programming, no special tools.** You copy one file onto the FlySight, the
same way you copy a photo onto a USB stick. It takes about 10 minutes.

You don't need a GitHub account and you don't need to understand anything on
this page except the steps below.

### What you need

- Your FlySight 2 and its USB cable
- A computer (Windows or Mac)
- Your ENGO glasses (only for the final check)
- Your FlySight should already be running a recent official firmware. If you
  bought it in 2024 or later, or you have ever updated it, you're fine. If it
  is an old unit that has never been updated, update it once from
  [flysight.ca](https://www.flysight.ca/) first.

> **You cannot break the device with this.** If anything goes wrong, the
> FlySight can always be put back to the original firmware — see
> [Going back to the original firmware](#going-back-to-the-original-firmware)
> at the end. Nothing here touches your jump logs.

---

### Step 1 — Plug in the FlySight and save your jumps

Connect the FlySight to the computer with the USB cable. It appears like a USB
stick (a drive named **FLYSIGHT**).

Copy your jump folders off it somewhere safe now, just so you have them. (The
update does not delete anything, but it's a good habit.)

### Step 2 — Find out which file you need

FlySights are made in production batches, and each batch takes its own file.
You need to find out which one you have. It's one line in a text file:

1. On the **FLYSIGHT** drive, open the file **`flysight.txt`** (double-click —
   it opens in Notepad on Windows, TextEdit on a Mac).
2. Find the line that starts with **`Pubkey_X:`**
3. Look at the **first 8 characters** after it and find them in this table:

| If `Pubkey_X` starts with… | Your file is |
|----------------------------|--------------|
| `486bee2d`                 | **B2**       |
| `211d721d`                 | **B3**       |
| `dac40d25`                 | **B4**       |
| `3157e028`                 | **B5**       |
| `8ee78709`                 | **B6**       |

So if your line reads `Pubkey_X: 486bee2d3dd6...`, you need the **B2** file.

**Can't find the line, or not sure?** Just start with **B2** and continue. If
the update doesn't happen (step 7 shows nothing changed), simply repeat with
the next file. A file for the wrong batch is politely ignored by the device —
it cannot damage anything.

### Step 3 — Download the file

Open the download page: **[latest release](../../releases/latest)**

> There are two kinds of build. **Stable** is what the link above gives you —
> tested on real hardware, this is what you want. **Nightly** is built
> automatically from the newest code and may contain untested features; it lives
> on the [releases page](../../releases) marked *Pre-release*. Nightly builds
> show a version like `v0.0.14-n.a1b2c3d` in the glasses, so you can always tell
> which one you are running. Going back to stable is just installing the stable
> file the same way.

Scroll down to the list of files (it's called *Assets*) and click the one for
your batch, for example **`FlySight2-ENGO-HUD-B2.sfb`**. It downloads like any
other file — usually into your *Downloads* folder.

*(The other files there are for the remaining batches, plus a `sha256sums.txt`
which you can ignore — it's only for people who want to verify the download
cryptographically.)*

### Step 4 — Rename the file to `APP.SFB`

The FlySight only accepts the update if the file has exactly this name:

```
APP.SFB
```

Rename the downloaded file to that (right-click → Rename).

> ⚠️ **Windows users, important:** Windows usually hides file endings, so the
> file may look like just `FlySight2-ENGO-HUD-B2` while it is really
> `FlySight2-ENGO-HUD-B2.sfb`. If you rename it to `APP.SFB` while endings are
> hidden, you actually get `APP.SFB.sfb` and **the update will not work**.
>
> Fix: in Explorer open the **View** menu and tick **File name extensions**,
> then rename. Now what you see is the real full name.

### Step 5 — Put the FlySight into update mode (orange light)

This is the step people miss — copying the file normally does nothing.

**Press and hold the FlySight button for about 10 seconds**, until the light
turns **orange**. The device restarts and appears on the computer again as a
drive — now it is ready to receive the update.

### Step 6 — Copy the file onto the device

On the FlySight drive, open the folder named **`FW`**. If there is no such
folder, create one and name it exactly `FW`.

Drag your **`APP.SFB`** into that `FW` folder.

Then eject the drive safely (Windows: right-click the drive → *Eject*; Mac:
drag it to the Trash / click the ⏏ symbol next to it) and **unplug the cable**.

### Step 7 — Wait, then check that it worked

After unplugging, **wait about 10 seconds without touching it.** The light
blinks while the FlySight installs the update and restarts itself.

Now check:

- Plug it in again and open `flysight.txt` — the line `Firmware_Ver:` should
  have changed, and the `APP.SFB` file you copied is **gone** from the `FW`
  folder (the device used it up). That means it installed.
- Or simply switch on the glasses and the FlySight (see below) — the top line
  in the glasses ends with the version, e.g. `v0.0.14`.

If nothing changed, see [If something doesn't work](#if-something-doesnt-work).

---

## Using it — the glasses

**Just switch both on.** Turn on the glasses and turn on the FlySight — in any
order. Within roughly 20–45 seconds the display appears in the glasses.

There is no pairing, no PIN, no phone app, nothing to set up.

**Your FlySight remembers your glasses.** The first time it connects, it
memorises that specific pair and from then on connects only to them — so on a
busy dropzone it will not grab someone else's glasses.

**If you ever change glasses** (new pair, borrowed pair, sold yours): plug the
FlySight into the computer and delete the file **`engo3.txt`** from the drive.
Next time it will connect to your new glasses and remember those instead.

---

## If something doesn't work

**The version didn't change / the file is still sitting in the `FW` folder.**
The device was not in update mode. Repeat from step 5 and make sure the light
really turns **orange** before you copy the file.

**You're on Windows and it still doesn't install.** Check the file name really
is `APP.SFB` and not `APP.SFB.sfb` — see the warning in step 4.

**Still nothing after that.** You may have downloaded the file for the wrong
batch. Try the next one from the table in step 2 (it's harmless).

**The glasses stay blank.** Give it a minute — the connection can take up to
45 seconds. Make sure the glasses are actually switched on and charged; ENGO
glasses also switch themselves off after a few minutes of being idle. If you
recently used another pair, delete `engo3.txt` as described above.

**Something else, or it worked but behaves oddly.** Please tell us — see
[Feedback](#feedback--issues). If you can, copy the file `EVENT.CSV` from the
newest jump folder on the device and attach it: it records what the firmware
was doing and usually shows the problem straight away.

---

## Going back to the original firmware

You can always return to the official FlySight firmware — the update mechanism
itself can never be damaged.

1. Download the official firmware for your device from
   [flysight.ca](https://www.flysight.ca/).
2. Do exactly the same steps 4–7 as above with that file (rename to `APP.SFB`,
   orange light, copy into `FW`, eject, unplug, wait).

Your jump logs stay where they are.

---

## Technical notes on binding (for the curious)

The glasses are identified by the 6-character serial at the end of their
Bluetooth name (e.g. `ENGO 3 123456` → `123456`). On the first successful link
that serial is written to `engo3.txt` in the root of the FlySight drive, and
afterwards only glasses with that serial are accepted.

You can also pin a specific pair by creating `engo3.txt` yourself. Accepted
contents: `123456`, `ID: 123456`, or the full name `ENGO 3 123456` — the last
6 characters are used. Deleting the file returns the device to "connect to the
first ActiveLook glasses I see, then remember them".

Each session's `EVENT.CSV` logs the bind state at boot
(`ENGO bind: pinned to serial 123456` / `unbound -> connect to first`) and
`ENGO bind: linked to glasses serial 123456` once connected.

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
