# Divergence from the upstream FlySight firmware

Where this fork differs from `flysight/flysight-2-firmware`, why, and what must not be
changed without a coordinated migration. Keep this file current: every time an upstream
commit is examined and *not* taken, the reason belongs here, otherwise the next person
re-derives it from scratch.

Upstream remote: `https://github.com/flysight/flysight-2-firmware`, branch `develop`.

## How to check for new upstream work

```sh
git fetch upstream
git log --oneline $(git merge-base HEAD upstream/develop)..upstream/develop
```

Prefer a real `git merge upstream/develop`, resolving conflicts in our favour where the
table below says so, over cherry-picking. A merge records the merge base, so the next
comparison lists only genuinely new commits; cherry-picks leave those commits forever
"unmerged" and every future merge re-conflicts on the same hunks.

## State as of 2026-09-07

Merge base: `b11e597` (2026-05-14). Upstream commits since then, and what we did:

| Commit | Subject | Decision |
|---|---|---|
| `c17506a` | Search for ENGO 3 as well as ENGO 2 | **not taken.** Upstream matches an exact name `"ENGO 2/3 " + AL_ID` plus manufacturer data `0x08F2`. We match a case-insensitive name prefix or the service UUID, so we are model-agnostic and already find ENGO 3. |
| `541bbeb` | Include float option for printf | **not needed.** That patch edits `.cproject` for the CubeIDE build; our `Makefile` has carried `-u _printf_float` from the start. |
| `4fd8d08` | Fix issue connecting to Engo 3 | **taken** (`Address_Type & 0x01` before `aci_gap_create_connection`). |
| `7b57cc8` | Classify mode update task as using HCI | **to take.** `CFG_TASK_FS_MODE_UPDATE_ID` still sits in the no-HCI list in `Core/Inc/app_conf.h`; the mode-update task does issue BLE work, so the classification is wrong. |
| `98f15bb` | Add altitude at destination on ActiveLook | **not taken.** Predicted altitude over the destination is not information anyone uses in flight. See the field-number note below — this commit is the source of the collision at 14. |
| `bdab843` | Update public deployment keys | **already have it.** `pub_key_b7/b8/b9/dev.bin` here are byte-identical to upstream's. |

## Deliberate, permanent divergences

These are not "not yet merged" — they are decisions. When merging upstream, keep ours.

- **Glasses discovery** (`STM32_WPAN/App/app_ble.c`). Name-prefix or service-UUID match plus
  serial binding through `/engo3.txt`, instead of an exact `"ENGO <model> <id>"` string.
- **Flow control** (`STM32_WPAN/App/activelook_client.c`). We discover and subscribe CB8 and
  CB9 and refuse to bring the link up without CB9; upstream has no CB9 at all and cannot
  learn that the glasses asked it to stop sending.
- **Self-heal** (`FlySight/activelook.c`). Discovery timeout 10 s, STOP stuck > 6 s, and a
  streak of 8 failed writes each force a reconnect; rescan restarts on disconnect. Upstream
  clears the handle on disconnect and does nothing further.
- **Rendering** (`FlySight/activelook_mode0.c`). Direct draw wrapped in `holdFlush`, with
  change detection, instead of layouts plus `pageClearAndDisplay` on every tick.
- **Connection interval.** 15–30 ms, which is what ActiveLook asks for; upstream uses 7.5 ms.
- **Our own subsystems** with no upstream counterpart: `hud_layout`, `comp_corridor`, `nav`,
  `nav_arrow`, `flight_detect`, `engo_bind`, `activelook_adv`, `activelook_proto`, `ble_diag`.

## HUD field numbers — a real collision, migration deferred

Field ids are the contract between `CONFIG.TXT`, this firmware (`FlySight/hud_layout.h`)
and the phone app's layout editor (`lib/model/hud_layout.dart`, which also hard-codes
`isArrow => id == 15`). Ids 0–13 mean the same thing here and upstream.

| Id | Upstream | This fork | Status |
|---|---|---|---|
| 0–12 | HSpd, VSpd, GR, 1/GR, Spd, Dir, Dist, Brg, …, Dive, Alt | same | agreed |
| 13 | `MODE_COURSE` | `FIELD_HEADING` | same thing, different name |
| 14 | `MODE_ALTITUDE_AT_DESTINATION` (added in `98f15bb`) | `FIELD_BARO_ALT` — barometric altitude, zeroed at power-on | **collision** |
| 15 | free — the next id upstream will hand out | `FIELD_NAV_ARROW` | **latent collision** |
| 100–106 | — | status family: info line, batteries, sats, version, flight marker, flight time | safe, far outside upstream's range |

**Decision (2026-09-07): do not renumber yet.** Devices are already in the field with
configs written against these ids — at least two beyond the owner's own — so changing a
number silently draws the wrong value on somebody's HUD. The move to a private range must
happen as one migration, in the firmware and the app together, with the app translating
old ids to new when it reads a `CONFIG.TXT` it did not write.

Until then: do not accept upstream's `config.h` numbering in a merge, and do not implement
upstream field 14 under our number 14.
