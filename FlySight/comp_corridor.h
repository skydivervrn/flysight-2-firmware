/***************************************************************************
**                                                                        **
**  FlySight 2 firmware                                                   **
**  Copyright 2025 Bionic Avionics Inc.                                   **
**                                                                        **
**  This program is free software: you can redistribute it and/or modify  **
**  it under the terms of the GNU General Public License as published by  **
**  the Free Software Foundation, either version 3 of the License, or     **
**  (at your option) any later version.                                   **
**                                                                        **
****************************************************************************/

/*
 * Wingsuit-competition lane indicator — DISPLAY ONLY.
 *
 * FAI Competition Rules for Wingsuit Flying, 2026 Edition, as they apply to a
 * device that can only watch:
 *
 *   Validation Window   opens 9 s after vertical speed first reaches 10 m/s.
 *   Designated Flight   the straight GROUND track from where the competitor was
 *     Path              when that window opened, to a Ground Reference Point
 *                       the Chief Judge assigns (CONFIG.TXT Comp_Lat/Comp_Lon).
 *   Designated Lane     600 m wide, centred on that path: 300 m each side.
 *   Penalties           measured from the lane BOUNDARY, so a competitor who is
 *                       450 m from the centreline is 150 m outside and into the
 *                       -20 % band; 300..450 m from the centreline is -10 %.
 *
 * The competitor cannot know any of that in the air today: the lane exists only
 * in the judges' analysis afterwards. This module is what puts it on the glasses
 * while there is still time to steer.
 *
 * It changes NOTHING about the device. It does not touch logging, the GNSS rate,
 * the audio, or the power state — it reads the same samples the logger has
 * already written and answers questions about them. A competitor's track is
 * byte-for-byte the same whether this is drawn or not, which is the only
 * acceptable arrangement for a scored discipline.
 *
 * Pure + host-testable: no HAL, no RTOS, no config, only FS_GNSS_Data_t in and
 * numbers out. The two things that would be invisible until a competition day —
 * the instant the window opens, and which side of the lane the wearer is on —
 * are therefore checked on the host (Tests/test_comp_corridor.c).
 */

#ifndef COMP_CORRIDOR_H_
#define COMP_CORRIDOR_H_

#include <stdbool.h>
#include <stdint.h>
#include "gnss.h"

/* --------------------------------------------------------------------------
   The rules, as numbers
   -------------------------------------------------------------------------- */

/* Vertical speed that arms the window, and the delay from there to its start.
 * Both are quoted in the rule book; neither is a tuning parameter, and changing
 * one would put the origin of the lane somewhere the judges' does not. */
#define FS_COMP_TRIGGER_MPS      10.0
#define FS_COMP_WINDOW_DELAY_MS  9000u

/*
 * ...and the speed that says the 10 m/s was a FALL rather than an aircraft.
 *
 * 10 m/s down is 2000 ft/min, which a jump ship reaches easily. On 2026-09-05
 * a pilot took the lane run high and let down before the exit, and the descent
 * crossed 10 m/s twice — for 6.6 s peaking at 14.5 m/s, and for 10.6 s peaking
 * at 13.4. Either would have opened the window in the aircraft: the first one
 * put the origin 1111 m and four and a half minutes from where the exit was.
 *
 * Duration cannot tell the two apart — the second episode outlasted the nine
 * seconds. Speed can. Measured over three jumps from that day:
 *
 *   aircraft, letting down    peak 14.5, 13.4 m/s
 *   exits                     peak 30.8, 33.0, 32.2 m/s
 *
 * So the peak within the nine seconds has to reach this, checked at the moment
 * the origin would be latched — the confirmation arrives seconds before that
 * (10 to 20 m/s took 2.6 s on the exit measured), so nothing about WHERE the
 * origin falls changes. An arming that fails it is dropped and the detector
 * goes back to watching, which is the only way to catch the real exit after a
 * false one.
 *
 * NB the canopy also crosses 10 m/s, in spirals, peaking 19.5 m/s on one of
 * those tracks — closer to this number than the aircraft ever got. It cannot
 * arm anything: the window latches once and FS_CompCorridor_Update returns on
 * s_started for the rest of the flight. The exception is active mode being
 * re-entered mid-descent, which calls FS_CompCorridor_Init and clears that
 * latch; a spiral half a metre per second faster than the one measured would
 * then be indistinguishable from an exit.
 *
 * WHAT THIS COSTS, and it is not nothing. A rejected arming may not arm again
 * until the descent has come back under 10 m/s (see s_reArmBlock). So an exit
 * taken while the aircraft is ALREADY descending faster than that — no edge in
 * between — opens no window at all. Tests/test_comp_corridor.c section 12 pins
 * both that and the version where a fix dropout hides the edge.
 *
 * The trade was made knowingly: without the gate, that same profile times the
 * exit off the aircraft's stopwatch and draws a lane confidently in the wrong
 * place, which is the failure a competitor cannot see. Drawing nothing, he can.
 */
#define FS_COMP_CONFIRM_MPS      20.0

/* Half the Designated Lane. Beyond this the competitor is OUTSIDE and losing
 * percentage points, which is why it is the boundary the display changes at. */
#define FS_COMP_LANE_HALF_M      300.0f

/* 150 m outside the boundary is where the penalty doubles, from -10 % to -20 %.
 * Expressed from the CENTRELINE, because that is what the deviation measures. */
#define FS_COMP_PENALTY2_M       450.0f

/* One rung per 25 m, so a full ladder of 12 lands exactly on the lane edge.
 * 25 m is roughly one second of drift at a wingsuit's crosswind rates, i.e. the
 * finest step a competitor can actually fly to. */
#define FS_COMP_BAR_STEP_M       25.0f
#define FS_COMP_MAX_BARS         12

/* --------------------------------------------------------------------------
   The detector
   -------------------------------------------------------------------------- */

/* Reset (call once when active mode starts, beside FS_FlightDetect_Init). */
void FS_CompCorridor_Init(void);

/*
 * Feed one GNSS sample (call on every GNSS DataReady).
 *
 * Returns true EXACTLY ONCE — on the sample that opens the Validation Window and
 * latches its origin — so the caller can write a single line to EVENT.CSV. That
 * line is the only ground truth we will ever have for where this firmware
 * thought the window started, which matters when a score is disputed.
 *
 * ARMING NEEDS A 3D FIX and so does the latch: the origin is a POSITION, and one
 * cannot be latched from a sample that has none. If the fix is lost across the
 * 9 s delay the origin is taken on the first fixed sample after it, late rather
 * than invented. A dip back below 10 m/s does NOT re-arm: the rule says "first
 * reaches", and a wingsuit levelling off for a second in the first nine is still
 * in the same exit.
 */
bool FS_CompCorridor_Update(const FS_GNSS_Data_t *d);

/* True once the Validation Window has opened. This is what the HUD's centre bar
 * means: it appears at that instant and not before. */
bool FS_CompCorridor_Started(void);

/* The latched origin of the Designated Flight Path. Returns false and leaves the
 * outputs untouched until the window opens. */
bool FS_CompCorridor_Origin(int32_t *out_lat, int32_t *out_lon);

/*
 * Exactly 0,0 is how CONFIG.TXT says "no Ground Reference Point" — the same rule
 * the landing zone's Lat/Lon use. FS_Config_Init leaves the pair at zero and a
 * value the parser rejects leaves it there too, so zero means "nobody assigned
 * one", not "fly at the Gulf of Guinea".
 */
int FS_CompCorridor_RefSet(int32_t lat, int32_t lon);

/* --------------------------------------------------------------------------
   Which side of the lane, and how far

   ## THE SIGN CONVENTION

   POSITIVE means the flyer is to the RIGHT of the Designated Flight Path, as
   seen by someone flying along it from the origin towards the Ground Reference
   Point. Negative is to the left. It is the flyer's own left and right, not the
   map's, because that is the hand they will pull with.

   Getting this backwards is the failure that matters: a mirrored indicator is
   perfectly legible, perfectly steady, and tells a competitor to correct further
   out of the lane every single time. There is no tell in the air. So the sign is
   checked on the host, twice, on paths pointing in different directions —
   Tests/test_comp_corridor.c, "east of a northward path is RIGHT".
   -------------------------------------------------------------------------- */

/*
 * Signed lateral deviation in metres of `cur` from the path `start` -> `ref`.
 * All six coordinates are degrees x 10,000,000, as CONFIG.TXT and the u-blox
 * receiver both give them.
 *
 * Returns 1 on success. Returns 0 — and touches nothing — when the path is too
 * short to have a direction (see FS_COMP_MIN_PATH_M): a lane centred on the
 * bearing between two points 30 m apart is centred on GPS noise, and would swing
 * through 180 degrees while the competitor flew straight.
 *
 * The projection is flat-earth about `start`, which costs well under a metre
 * over the 5-10 km a competition run covers and needs no iteration.
 */
int FS_CompCorridor_Lateral(int32_t start_lat, int32_t start_lon,
                            int32_t ref_lat,   int32_t ref_lon,
                            int32_t cur_lat,   int32_t cur_lon,
                            float *out_m);

/* Shortest Designated Flight Path this will take a bearing from. */
#define FS_COMP_MIN_PATH_M  100.0f

/*
 * The same thing for the live state: deviation of the newest fixed sample fed to
 * Update() from the path latched at the window's start to (ref_lat, ref_lon).
 *
 * Returns 0 until the window has opened, when no fixed sample has arrived since
 * it did, or when the path is degenerate. Staleness is deliberately NOT checked
 * here — this module never sees a clock — so a caller with access to
 * FS_GNSS_IsStale() has to apply it, exactly as the HUD's other GPS-derived
 * readings do.
 */
int FS_CompCorridor_Deviation(int32_t ref_lat, int32_t ref_lon, float *out_m);

/* --------------------------------------------------------------------------
   What the HUD should draw
   -------------------------------------------------------------------------- */

typedef struct
{
	/* 0 = DRAW NOTHING AT ALL. Not an empty centre bar, not a placeholder: the
	 * appearance of the centre bar IS the signal that the Validation Window has
	 * opened, and a competitor watching for it must not have to tell a lit
	 * marker from a dim one at arm's length in daylight. */
	uint8_t active;

	int8_t  side;   /* +1 right of the path, -1 left, 0 exactly on it */
	uint8_t bars;   /* rungs to draw on that side, 0..FS_COMP_MAX_BARS */

	/* The side is one solid block rather than rungs: the competitor is OUTSIDE
	 * the lane and already losing 10 %. Counting rungs stops being useful there
	 * — the only fact left is "get back". */
	uint8_t solid;

	/* ...and that block blinks: 150 m or more outside, where the penalty is
	 * 20 %. A blink is the one thing on this panel that cannot be mistaken for
	 * a steady reading in peripheral vision. */
	uint8_t blink;
} FS_CompInd_t;

/*
 * Turn a deviation into the indicator's state.
 *
 *   started   the window has opened (FS_CompCorridor_Started)
 *   have_dev  a deviation was computed this tick; when 0, `dev_m` is ignored and
 *             a latched indicator shows its centre bar alone — the window is
 *             open, the position is not currently known, and saying so with a
 *             bare centre bar is more honest than freezing yesterday's rungs.
 */
void FS_CompCorridor_Indicator(int started, int have_dev, float dev_m,
                               FS_CompInd_t *out);

/* --------------------------------------------------------------------------
   Geometry (PURE)

   The panel MIRRORS X and does NOT flip Y — verified on the ENGO 3, written up
   in nav_arrow.h and hud_layout.h. A larger x is further to the wearer's LEFT;
   a larger y is higher. Everything below is worked out in the wearer's frame and
   mirrored in ONE place, at the bottom of comp_corridor.c, for the same reason
   the arrow is: so a future reader can check the mirror once instead of tracing
   it through every rung.
   -------------------------------------------------------------------------- */

typedef struct
{
	int16_t x0, y0;   /* normalised so x0 <= x1 and y0 <= y1 */
	int16_t x1, y1;

	/* 1 = the caller must draw a FILLED rectangle (ActiveLook rectf, 0x34);
	 * 0 = a plain line (0x32). Kept as a flag rather than an opcode so this
	 * module stays free of the protocol, exactly as nav_arrow.c is. */
	uint8_t solid;
} FS_CompShape_t;

/* Centre bar plus a full ladder on one side. Both sides are never drawn at
 * once — the competitor is on one side of the line or the other. */
#define FS_COMP_MAX_SHAPES  (1 + FS_COMP_MAX_BARS)

typedef struct
{
	FS_CompShape_t shape[FS_COMP_MAX_SHAPES];
	uint8_t        count;   /* shape[0] is ALWAYS the centre bar when count > 0 */
} FS_CompDraw_t;

/*
 * Build the shapes.
 *
 *   ax, ay    the element's anchor in PANEL coordinates — the wearer's TOP-LEFT
 *             corner, the same origin every HUD element grows from.
 *   size      the HEIGHT of the indicator, px; the HUD passes the element's font
 *             height so the bar and the rungs stand as tall as the clock they
 *             replace.
 *   span      the WIDTH the ladder may occupy, px.
 *
 *             Width used to follow from `size` — about three times it, 72 px at
 *             the 24 px status font, the footprint of the "--:--" clock. That
 *             made the instrument as wide as a piece of text: a rung every
 *             3 px, so 25 m of drift moved the reading by three pixels and a
 *             competitor flying the lane could not see the correction he was
 *             making. Flown 2026-09-05; the pilot's words were that the window
 *             is small, the size of the time.
 *
 *             The lane now gets the panel to itself. The clock sharing the slot
 *             is unchanged — it is text, and text is read, not measured.
 *             FS_CompCorridor_Width() says where the ladder will actually fall.
 *   ind       from FS_CompCorridor_Indicator.
 *   side_on   0 blanks the RUNGS/solid block for this frame and keeps the centre
 *             bar. That is how the blink is done: the caller flips it on the
 *             render tick it already has. The centre bar never blinks — it is
 *             the reference the competitor reads the rest against, and a
 *             reference that disappears half the time is not one.
 *
 * Returns the number of shapes, and 0 when there is nothing to draw at all — an
 * inactive indicator, a NULL argument, or a box too small to read.
 *
 * Coordinates are NOT clamped to the panel, for the reason spelled out in
 * nav_arrow.h: the glasses clip, and clamping would silently move a rung, which
 * on a ladder means changing the number the competitor counts.
 */
uint8_t FS_CompCorridor_Build(int16_t ax, int16_t ay, int16_t size,
                              int16_t span,
                              const FS_CompInd_t *ind, int side_on,
                              FS_CompDraw_t *out);

/*
 * How wide the ladder built with `span` really comes out, px.
 *
 * The rung pitch is a whole number of pixels — a ladder counted in fractions of
 * one is a ladder miscounted — so the drawn width is `span` rounded DOWN to a
 * multiple of twice the rung count. The caller centres with this, not with
 * `span`, or the instrument sits a few pixels off the middle of the panel and
 * every deviation reads biased to one side.
 */
int16_t FS_CompCorridor_Width(int16_t span);

/* Below this the rungs are under 6 px tall and 3 px apart, which reads as a
 * smudge rather than a count. Refused outright instead. */
#define FS_COMP_MIN_SIZE  12

#endif /* COMP_CORRIDOR_H_ */
