# ENGO 3 HUD — layout and screen position

Since v0.0.16 the layout is **data, not code**: a list of elements in
`CONFIG.TXT`, each with a field, a position, a font and a precision, plus one
global offset that shifts the whole picture. The geometry lives in
`FlySight/hud_layout.{c,h}` (pure, host-tested in `Tests/test_hud_layout.c`);
`FlySight/activelook_mode0.c` draws it.

A `CONFIG.TXT` that positions nothing gets the built-in layout below, so an
untouched device looks exactly as it did before.

## Screen

304 x 256, text rotation 4.

- **X is mirrored**: `x=296` is the wearer's LEFT edge, `x=0` the right one,
  and a string grows from its anchor toward smaller x.
- **Y is not flipped**: larger y is higher, and glyphs hang DOWN from the
  anchor — so an element's y is its top edge.

## Built-in layout (tuned on hardware, approved pixel by pixel)

| element | field | x   | y   | font | px | content                              |
|---------|-------|-----|-----|------|----|--------------------------------------|
| status  | 100   | 296 | 232 | 0    | 24 | `X A:..% F:..% N:.. v<HUD_VER>`      |
| HSpd    | 0     | 268 | 208 | 3    | 64 | km/h, 0 dec, RAW GPS (no SAS)        |
| VSpd    | 1     | 134 | 208 | 3    | 64 | km/h, 0 dec, GNSS velD raw, + = down |
| GR      | 2     | 250 | 147 | 4    | 75 | 2 dec                                |
| Alt     | 14    | 250 |  73 | 4    | 75 | m baro, 0 dec                        |

No text labels — values only. HSpd and VSpd share one row (HSpd viewer-left,
VSpd viewer-right).

Speeds are raw GPS on purpose (they match skyderby's ground-speed charts; the
SAS air-density correction was removed from the HUD path — `Use_SAS` in
CONFIG.TXT still governs the audio tones). VSpd is GNSS velD rather than a
barometric estimate: real-jump log analysis showed baro altitude noise of
~11 m stddev in freefall (aerodynamic pressure on the body), which made a
baro-derived VSpd jump in 10-40 km/h steps, while velD stayed ~0.5 km/h stable.

## CONFIG.TXT keys

```
AL_Shift_X:    0   ; global offset, wearer's terms: + moves the picture right
AL_Shift_Y:    0   ; + moves it up. Both -120..120.

AL_Line:       0   ; opens an element; the keys below describe it
AL_Units:      0   ; 0 metric, 1 imperial, or one unit by name (see below)
AL_Dec:        0   ; decimal places, 0..3
AL_Unit_Show:  0   ; 0 = "148", 1 = "148 km/h"
AL_X:        268   ; 0..303
AL_Y:        208   ; 0..255
AL_Font:       3   ; 0..7
```

Repeat the block per element, up to `FS_HUD_MAX_ELEMENTS` (8). Any of
`AL_X` / `AL_Y` / `AL_Font` anywhere in the file switches the HUD from the
built-in layout to the file's; a block that omits a coordinate inherits the
built-in position for that slot. Out-of-range numbers are clamped, not
rejected — a hand-edited file cannot push an element off the panel.

`AL_Unit_Show` (since v0.0.17) appends the element's unit — `km/h`, `m/s`,
`mph`, `ft/s`, `m`, `ft`, `km`, `mi`, `deg` — after a single space. It defaults
to **0**, and the built-in layout above leaves it off: those coordinates were
approved on hardware with bare numbers, and whether a suffix at 64/75 px still
clears the neighbouring element has not been checked through the glasses. Glide
ratio and inverse glide ratio have no unit, so the key does nothing there. On
the split status pieces (101–103, below) it turns on the **prefix** instead —
`80%` becomes `A:80%` — and on 100, 104 and 105 it does nothing. Like `AL_Units`
and `AL_Dec`, it states no position, so it does not by itself switch the HUD
away from the built-in layout — a file that sets it must also state coordinates.

Fields: 0 HSpd, 1 VSpd, 2 GR, 3 1/GR, 4 total speed, 5 direction to
destination, 6 distance to destination, 7 direction to bearing, 11 dive angle,
12 GPS altitude above `DZ_Elev`, 13 course, **14 barometric altitude** (zeroed
at power-on), **100 status line**, **101–105 the pieces of the status line**.
None of 14 and 100–105 needs a GPS fix; everything else reads `----` until the
fix arrives.

## The status line, whole or in pieces (since v0.0.18)

Field `100` draws all five readings as one string in one font in one place:

```
V A:80% F:76% N:12 v0.0.18
```

Fields `101`–`105` each draw one of them as an ordinary element, with its own
`AL_X`, `AL_Y` and `AL_Font` — so the version can be dropped, or the satellite
count moved somewhere it does not crowd an altitude, or nothing kept at all.

| id  | piece               | bare      | with `AL_Unit_Show: 1` |
|-----|---------------------|-----------|------------------------|
| 101 | glasses battery     | `80%`     | `A:80%`                |
| 102 | FlySight battery    | `76%`     | `F:76%`                |
| 103 | satellites          | `12`      | `N:12`                 |
| 104 | HUD version         | `v0.0.18` | `v0.0.18`              |
| 105 | flight-detect mark  | `V` / `X` | same                   |

The `%` is part of the value — a battery reading without it is a number nobody
can place — so `AL_Unit_Show` adds the *prefix*, which is what tells two battery
percentages side by side apart. `AL_Units` and `AL_Dec` mean nothing to any of
the six and are ignored, not rejected. Satellites read `0` before the first fix,
never `----`; the glasses battery reads `??%` until the link reports one.
An element of this family that states no coordinates lands on the status line's
own defaults (296, 232, font 0) rather than on a 64 px data row, so a file that
splits the line without positioning the pieces stacks them rather than blowing
one battery percentage up to fill the panel.

All six are rebuilt from **one snapshot behind one shared 5-tick divider**: sat
and battery jitter would otherwise change one string or another nearly every
tick, and any single changed string forces a full clear+redraw of the panel.
Splitting the line must not multiply the chances of that. The cost is that the
takeoff marker can be up to five ticks (1.25 s at `AL_Rate: 250`) late — which
is what it has already been inside field 100 since v0.0.14.

Field `100` itself is untouched, byte for byte. Every card in the field has it.

## Units (`AL_Units`)

| value | unit | applies to |
|-------|------|------------|
| 0 | the quantity's metric default | every field |
| 1 | the quantity's imperial default | every field |
| 2 | km/h | speed |
| 3 | m/s  | speed |
| 4 | mph  | speed |
| 5 | ft/s | speed |
| 6 | m    | altitude, distance |
| 7 | ft   | altitude, distance |
| 8 | km   | distance |
| 9 | mi   | distance |

Since v0.0.18. `0` and `1` are what they always were, and a card written before
this release renders identically — the multipliers behind them are the same
constants bit for bit, checked string by string in `Tests/test_activelook.c`
against a copy of the v0.0.17 conversion function.

A value that does not apply to the field's quantity — `AL_Units: 8` on a
vertical speed — reads as that quantity's **metric** default (km/h here), not as
a distance. No app writes that; a hand-edited card can. Anything above 9 is
metric too. Angles are `deg` whatever is asked for, and the glide ratios stay
bare. The whole table is `FS_HudLayout_UnitConv` in `hud_layout.c`; it used to
live in the renderer, where no host test could reach it.

## Refresh rate

```
AL_Rate:     250   ; ms between HUD frames, minimum 100
```

**250 ms (4 Hz) since v0.0.17**, up from 500 ms — the owner asked for it, 2 Hz
lagged visibly. Honesty about what that number rests on: **333 ms is the
fastest rate ever field-tested on this hardware**, and 250 has not been flown
yet. It is inside the range the pacing was built for — the glasses throttle us
with a CB9 flow-control STOP, honoured between chunks in
`FS_ActiveLook_Mode0_DrainFrame`, and a frame that hits a full CPU2 TX pool
resumes on `ACI_GATT_TX_POOL_AVAILABLE` instead of being dropped — but the
known failure mode of over-driving the glasses is the display processor hanging
while BLE stays up. If the HUD freezes and only a power-cycle of the glasses
brings it back, raise `AL_Rate` to 333 or 500 before looking anywhere else.

## Fonts loaded on ENGO 3 (fontList 0x50, verified on hardware)

id/height px: 0/24, 1/24, 6/32, 2/38, 7/48, 3/64, 4/75, 5/82. All render.
Smallest is 24 px; no bold variants pre-installed. Smaller/bold text would
require a custom font upload via fontSave (0x51).
The `vers` command is 0x06 (0x04 returns an 0xE2 error frame).

## Iterating on the layout

The Groundrush app does this live: its HUD tabs connect the phone straight to
the glasses, so dragging an element or nudging the position shows up on the
glasses immediately, and only the final layout is written to `CONFIG.TXT`.
The FlySight must not be holding the glasses at the same time — one link only.

Without the app, over BLE from a Mac:

1. Edit the `DRAW` table in `Tools/engo_mac_hud_mock.py` (zeros as placeholders).
2. Run it with the glasses on — they show the mock in ~15 s, no reflash needed.
3. When happy, put the coordinates in `CONFIG.TXT`, or — to change the built-in
   default — in `s_defaultSlots[]` in `FlySight/hud_layout.c`, bump
   `HUD_VERSION`, and build + deploy per `Docs/BUILDING.md`.
