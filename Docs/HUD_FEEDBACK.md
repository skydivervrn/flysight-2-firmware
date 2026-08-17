# Field feedback on 0.0.16, and what it turns into

The owner flew the config-driven HUD (firmware `0.0.16-n.384b589`, app branch
`hud-layout`) on 2026-08-17 and came back with ten remarks. This file is the
whole list, what has already been built for each, and — for the two that are
still open — the exact contract the firmware and the app must both implement,
written down here rather than agreed between them, because the two sides are
built in separate trees and only meet on the SD card.

## Where the work lives

| Tree | Branch | Base |
| --- | --- | --- |
| `/tmp/fs2fb` (firmware) | `fb-units-rate` | `hud-layout` @ `384b589` |
| `~/code/personal/groundrush-wt-fb-hud` (app) | `fb-hud` | `hud-layout` @ `6a4a6eb` |
| `~/code/personal/groundrush-wt-fb-device` (app) | `fb-device` | `hud-layout` @ `6a4a6eb` |

`fb-hud` and `fb-device` touch disjoint files and are meant to be merged
together. Nothing here is pushed or released until the owner has flown it.

## The ten remarks

1. **A "show units" tick in the unit setting.** — done: firmware `aad639f`
   (`AL_Unit_Show`), app `4aae3de`.
2. **A dropdown for the fonts.** — done: app `4aae3de`.
3. **Two menus, "HUD" and "Glasses", for what is one subject.** — done: one
   door named "HUD & glasses" (app `c67b564`) into a four-tab screen whose
   first tab is the binding (app `4aae3de`).
4. **Telemetry tiles above the settings, and far too large.** — done: app
   `c67b564`. Controls first, readings last, and the readings are a 2×2 grid of
   small tiles instead of a hero plus three.
5. **250 ms as the default refresh.** — done: firmware `aad639f`, app `4aae3de`.
6. **The red "did not answer within 5000 ms" stayed after the device did switch
   to ACTIVE.** — done: app `c67b564`. The control point's answer can time out
   while the write itself landed; the refusal is withdrawn as soon as `DS_Mode`
   reports the mode that was asked for.
7. **No way to stop a session from the app.** — done: app `c67b564`. A "Stop ·
   back to sleep" control, which the firmware really does accept —
   `validate_mode_request` returns accepted whenever the *target* is SLEEP and
   the current mode is not USB (`FlySight/device_state.c:388-400`).
8. **The status line should be separate, switchable pieces.** — OPEN, §A below.
   The wearer wants to drop the version, or keep nothing but an altitude across
   the whole panel.
9. **Altitude in feet; speed in km/h, m/s, mph or ft/s.** — OPEN, §B below.
10. **Every section switchable.** — covered by §A plus what already exists: any
    element can be removed in the layout tab. A layout of one element is
    allowed; an empty one is not, and is out of scope here — `al_layout_valid`
    would have to grow a way to say "positioned, and deliberately empty", which
    is a bigger change than the request needs.

---

# §A — the status line becomes five elements

Today field `100` draws one string:
`"^ A:80% F:76% N:12 v0.0.17"` — flight-detect marker, glasses battery,
FlySight battery, satellites, HUD version (`activelook_mode0.c`, the
`battLevels` block). It is all or nothing, and it is drawn at one font in one
place.

## Contract

Five new field ids, each drawing one piece, each an ordinary element with its
own `AL_X`, `AL_Y`, `AL_Font`:

| id | piece | bare text | with `AL_Unit_Show: 1` |
| --- | --- | --- | --- |
| 101 | glasses battery | `80%` | `A:80%` |
| 102 | FlySight battery | `76%` | `F:76%` |
| 103 | satellites | `12` | `N:12` |
| 104 | HUD version | `v0.0.18` | `v0.0.18` (flag ignored) |
| 105 | flight-detect marker | `^` / `-` | same (flag ignored) |

* Field `100` stays exactly as it is. Every card in the field has it, and an
  untouched device must look on 0.0.18 precisely as it did on 0.0.17.
* The prefix letters are the ones already on the panel today, so a wearer who
  splits the line and switches the prefixes on gets the same reading back.
* `%` is part of the value, not the unit suffix: a battery reading without it
  is a number nobody can place. `AL_Unit_Show` adds the *prefix*, which is what
  distinguishes two batteries sitting side by side.
* None of the five needs a GPS fix. Satellites read `0` before the first fix,
  never `----`; the glasses battery reads `??` when the link has not reported
  one, as it does today.
* `AL_Units` and `AL_Dec` are meaningless for all five and must be ignored, not
  rejected.
* Version text is `HUD_VERSION` with a leading `v`, the same string the status
  line ends with, because the point of it is to prove remotely which firmware
  is live on the glasses.
* The marker keeps `FD_MARK_DETECTED` / `FD_MARK_IDLE` verbatim.

The 5-tick rate limit that exists for field `100` (sat count and battery jitter
would otherwise force a full clear+redraw nearly every tick) must apply to
101–103 as well. 104 and 105 change rarely enough not to need it — but they
must not *defeat* it either, so keep one shared rebuild divider.

---

# §B — a unit per element, not a unit system per element

Today `AL_Units` is `0` metric / `1` imperial and the renderer maps that plus
the field's `FS_UnitType` onto one multiplier and one suffix
(`AL_GetUnitConversion`). Which is why a vertical speed can only be km/h or
mph, and neither is what a wingsuit pilot reads.

## Contract

`AL_Units` keeps its two old values and gains explicit ones:

| value | meaning | applies to |
| --- | --- | --- |
| 0 | metric — the type's metric default | every type |
| 1 | imperial — the type's imperial default | every type |
| 2 | km/h | speed |
| 3 | m/s | speed |
| 4 | mph | speed |
| 5 | ft/s | speed |
| 6 | m | altitude, distance |
| 7 | ft | altitude, distance |
| 8 | km | distance |
| 9 | mi | distance |

* Multipliers from the base units the line map already uses (speed m/s,
  altitude m, distance m): `km/h` ×3.6, `mph` ×2.236936, `ft/s` ×3.280840,
  `ft` ×3.280840, `km` ÷1000, `mi` ×0.000621371. Suffixes are exactly the
  strings in that table: `km/h`, `m/s`, `mph`, `ft/s`, `m`, `ft`, `km`, `mi`.
* Angles are `deg` at any value; unitless fields (glide ratio, inverse glide
  ratio) stay empty at any value.
* **A value that does not apply to the field's type falls back to that type's
  metric default** — `AL_Units: 8` on a vertical speed reads km/h, not a
  distance. It is a config the app will never write; the firmware must not
  render nonsense from a hand-edited card.
* Values above 9 clamp to 0, as `hud_layout.c` clamps everything else today.
* `0` and `1` must keep producing byte-identical output to 0.0.17 —
  `metric altitude` is `m` with multiplier 1, not `6`-through-a-new-path with a
  rounding difference. Prove it in the host tests.
* Old `FS_HUD_UNITS_METRIC` / `_IMPERIAL` stay as the names of 0 and 1.

## The app side of §B

The metric/imperial `SegmentedButton` becomes a dropdown listing the units that
apply to the selected field: a speed offers km/h, m/s, mph, ft/s; an altitude
or a distance offers m, ft (and km, mi where the field is a distance); an angle
and the ratios offer nothing and keep no picker. "Metric" and "Imperial" are
not shown as choices — a picker whose entries are `km/h` and `m/s` says more
than one that says `Metric` — but a file that arrives with `0` or `1` must be
shown as the unit it resolves to and written back **unchanged unless the user
picks something else**, so that opening the screen does not rewrite every card
in the field.

The preview sample must convert: 148 km/h is 41 m/s, and a preview that shows
`148 m/s` is worse than no preview.

---

# Common requirements

* **Firmware**: `Tests/test_hud_layout.c` covers every new parse and clamp;
  `Tests/test_activelook.c` covers the unit table if the conversion moves into
  a pure helper (preferred — the renderer is not host-testable today).
  `cd Tests && make run` must be green. Build with the GCC 12.3 toolchain in
  `~/opt` per `CLAUDE.md`; report FLASH/RAM. Bump `HUD_VERSION` to `0.0.18`,
  and update `Docs/HUD_LAYOUT.md` and the `CONFIG.TXT` template in `config.c`.
* **App**: `lib/model/hud_layout.dart` mirrors the firmware parser exactly,
  including the fallbacks above. `flutter analyze` clean, `flutter test` green
  — including the five cases the checkpoint commit left failing. The fixture
  `test/fixtures/config_template_hud.txt` is a copy of the template in
  `FlySight/config.c`; regenerate it from `/tmp/fs2fb` in the same commit.
* Neither side invents a key name. Both sides use `AL_Units`, `AL_Dec`,
  `AL_Unit_Show`, `AL_X`, `AL_Y`, `AL_Font`, and the field ids in this file.
* Nothing is pushed. The owner flashes and flies it first.
