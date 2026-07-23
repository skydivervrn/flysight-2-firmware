# ENGO 3 HUD — standard layout (v0.0.12)

Layout tuned live on ENGO 3 hardware via the Mac BLE bench
(`Tools/engo_mac_hud_mock.py`, a python venv with `bleak`).
Implemented in `FlySight/activelook_mode0.c` (`rowPos[]` + `HEADER_FONT`/
`SPEED_FONT`/`BIG_FONT`).

## Screen (304x256, rotation 4, X mirrored: x=296 = viewer left; glyphs draw
## downward from anchor y)

| element | x   | y   | font | px | content                        |
|---------|-----|-----|------|----|--------------------------------|
| header  | 296 | 232 | 0    | 24 | `X A:..% F:..% N:.. v<HUD_VER>` |
| HSpd    | 268 | 208 | 3    | 64 | km/h, 0 dec, RAW GPS (no SAS)  |
| VSpd    | 134 | 208 | 3    | 64 | km/h, 0 dec, GNSS velD raw, + = down |
| GR      | 250 | 147 | 4    | 75 | 2 dec                          |
| Alt     | 250 |  73 | 4    | 75 | m baro, 0 dec                  |

No text labels — values only. HSpd and VSpd share one row (HSpd viewer-left,
VSpd viewer-right).

Speeds are raw GPS on purpose (they match skyderby's ground-speed charts; the
SAS air-density correction was removed from the HUD path — `Use_SAS` in
CONFIG.TXT still governs the audio tones). VSpd is GNSS velD rather than a
barometric estimate: real-jump log analysis showed baro altitude noise of
~11 m stddev in freefall (aerodynamic pressure on the body), which made a
baro-derived VSpd jump in 10-40 km/h steps, while velD stayed ~0.5 km/h stable.

## Fonts loaded on ENGO 3 (fontList 0x50, verified on hardware)

id/height px: 0/24, 1/24, 6/32, 2/38, 7/48, 3/64, 4/75, 5/82. All render.
Smallest is 24 px; no bold variants pre-installed. Smaller/bold text would
require a custom font upload via fontSave (0x51).
The `vers` command is 0x06 (0x04 returns an 0xE2 error frame).

## Iterating on the layout

1. Edit the `DRAW` table in `Tools/engo_mac_hud_mock.py` (zeros as placeholders).
2. Run it with the glasses on — they show the mock in ~15 s, no reflash needed.
3. When happy, copy coords into `rowPos[]`/fonts in `activelook_mode0.c`,
   bump `HUD_VERSION`, build + deploy per `Docs/BUILDING.md`.
