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
AL_Units:      0   ; 0 = km/h and m, 1 = mph and feet
AL_Dec:        0   ; decimal places, 0..3
AL_X:        268   ; 0..303
AL_Y:        208   ; 0..255
AL_Font:       3   ; 0..7
```

Repeat the block per element, up to `FS_HUD_MAX_ELEMENTS` (8). Any of
`AL_X` / `AL_Y` / `AL_Font` anywhere in the file switches the HUD from the
built-in layout to the file's; a block that omits a coordinate inherits the
built-in position for that slot. Out-of-range numbers are clamped, not
rejected — a hand-edited file cannot push an element off the panel.

Fields: 0 HSpd, 1 VSpd, 2 GR, 3 1/GR, 4 total speed, 5 direction to
destination, 6 distance to destination, 7 direction to bearing, 11 dive angle,
12 GPS altitude above `DZ_Elev`, 13 course, **14 barometric altitude** (zeroed
at power-on, the only field that works without a GPS fix), **100 status line**.

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
