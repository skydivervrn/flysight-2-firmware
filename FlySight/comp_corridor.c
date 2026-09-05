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
 * See comp_corridor.h for the rules this implements, and for the two things
 * worth reading twice: the sign convention, and the panel's X mirror.
 */

#include "comp_corridor.h"

#include <math.h>
#include <stddef.h>               /* for NULL */

/* iTOW counts milliseconds into the GPS week and restarts at zero every Sunday
 * 00:00:00 GPS time. Same correction, for the same reason, as
 * FS_FlightDetect_ElapsedSec: a delta at or above a whole week is the unsigned
 * wrap of a backwards step, and one week is the only backwards step in that
 * number with a defined meaning. Without it, a run that armed at 604799000 and
 * crossed midnight Saturday would open its window on the very next sample. */
#define FS_COMP_WEEK_MS  604800000u

/* Earth radius as nav.c uses it, so a distance this module computes and one
 * calcDistance computes cannot disagree in the third digit and set two readings
 * on the same panel arguing. */
#define FS_COMP_EARTH_R_M   6371100.0
#define FS_COMP_DEG_TO_RAD  0.017453292519943295

/* Metres per degree of latitude, and of longitude at the equator: 111194.93 m.
 * Longitude is scaled by cos(latitude) at the path's origin. */
#define FS_COMP_M_PER_DEG   (FS_COMP_EARTH_R_M * FS_COMP_DEG_TO_RAD)

/* CONFIG.TXT and the u-blox receiver both carry coordinates as degrees x 1e7. */
#define FS_COMP_DEG_SCALE   1.0e7

static bool     s_armed;        /* 10 m/s has been seen                     */
static uint32_t s_armTOW;       /* iTOW (ms) of the sample that saw it      */
static double   s_armPeak;      /* fastest descent since, m/s: the evidence */
static bool     s_reArmBlock;   /* a rejected arming: wait for a real edge  */
static bool     s_started;      /* the Validation Window has opened         */
static int32_t  s_startLat;     /* its origin: the Designated Path starts   */
static int32_t  s_startLon;     /*   here and runs to the reference point   */
static bool     s_haveCur;      /* at least one fixed sample since Init     */
static int32_t  s_curLat;
static int32_t  s_curLon;

void FS_CompCorridor_Init(void)
{
	s_armed      = false;
	s_armTOW     = 0;
	s_armPeak    = 0.0;
	s_reArmBlock = false;
	s_started  = false;
	s_startLat = 0;
	s_startLon = 0;
	s_haveCur  = false;
	s_curLat   = 0;
	s_curLon   = 0;
}

bool FS_CompCorridor_Started(void)
{
	return s_started;
}

bool FS_CompCorridor_Origin(int32_t *out_lat, int32_t *out_lon)
{
	if (!s_started || out_lat == NULL || out_lon == NULL) return false;

	*out_lat = s_startLat;
	*out_lon = s_startLon;
	return true;
}

static uint32_t elapsed_ms(uint32_t now, uint32_t then)
{
	uint32_t ms = now - then;
	if (ms >= FS_COMP_WEEK_MS) ms += FS_COMP_WEEK_MS;
	return ms;
}

bool FS_CompCorridor_Update(const FS_GNSS_Data_t *d)
{
	/* A sample with no 3D fix carries neither a position to latch nor a vertical
	 * speed worth arming on. The ARMING already done is deliberately left
	 * standing across the dropout: the rule counts 9 s from when the competitor
	 * first reached 10 m/s, and the receiver losing sight of the sky for two of
	 * those seconds does not move that instant. */
	if (d == NULL || d->gpsFix != 3) return false;

	s_curLat  = d->lat;
	s_curLon  = d->lon;
	s_haveCur = true;

	if (s_started) return false;

	/* velD is mm/s, POSITIVE DOWN, so this is a descent rate. */
	const double vDown = (double)d->velD / 1000.0;

	if (!s_armed)
	{
		/* After a rejected arming, the descent has to come back UNDER the
		 * trigger before it may arm again — the next arming must be a real
		 * crossing, which is what "reaches" describes.
		 *
		 * Without this the aircraft re-arms on the very next sample, still at
		 * 12 m/s, and that arming's clock keeps running through the level
		 * flight that follows. An exit beginning inside its nine seconds would
		 * then be latched against the AIRCRAFT's stopwatch: replayed from the
		 * 2026-09-05 track, the window opened 3.2 s early, which is a lane
		 * origin several hundred metres up the run. */
		if (s_reArmBlock && vDown < FS_COMP_TRIGGER_MPS) s_reArmBlock = false;

		if (vDown >= FS_COMP_TRIGGER_MPS && !s_reArmBlock)
		{
			s_armed   = true;
			s_armTOW  = d->iTOW;
			s_armPeak = vDown;
		}

		/* Never latch on the arming sample itself, whatever the delay is set to:
		 * the window starts 9 s LATER, and its origin is a position the
		 * competitor has not flown to yet. */
		return false;
	}

	/* The evidence, gathered BEFORE the clock is read: a sample that both
	 * completes the nine seconds and carries the fastest descent of them has to
	 * count towards the verdict it is about to be judged by. Only fixed samples
	 * get here, so a dropout contributes nothing rather than a zero. */
	if (vDown > s_armPeak) s_armPeak = vDown;

	/* No re-arming on a dip back under 10 m/s. "First reaches" is what the rule
	 * says, and a wingsuit that levels off for a second inside the first nine is
	 * still in the same exit — restarting the count there would move the origin
	 * of the lane hundreds of metres down-track from the judges'. */
	if (elapsed_ms(d->iTOW, s_armTOW) < FS_COMP_WINDOW_DELAY_MS) return false;

	/* Nine seconds up. Was it a fall, or was it the aircraft letting down?
	 *
	 * An arming that never got near freefall is DROPPED and the detector goes
	 * back to watching. That is not a softening of "first reaches" — it is what
	 * the phrase means once the descent it referred to turns out to have been
	 * the ride to altitude. A judge reading the track afterwards would do the
	 * same, and the alternative is a lane drawn with total confidence about a
	 * kilometre from the one being scored. See FS_COMP_CONFIRM_MPS. */
	if (s_armPeak < FS_COMP_CONFIRM_MPS)
	{
		s_armed      = false;
		s_armPeak    = 0.0;
		s_reArmBlock = true;
		return false;
	}

	s_started  = true;
	s_startLat = d->lat;
	s_startLon = d->lon;
	return true;
}

int FS_CompCorridor_RefSet(int32_t lat, int32_t lon)
{
	return !(lat == 0 && lon == 0);
}

int FS_CompCorridor_Lateral(int32_t start_lat, int32_t start_lon,
                            int32_t ref_lat,   int32_t ref_lon,
                            int32_t cur_lat,   int32_t cur_lon,
                            float *out_m)
{
	if (out_m == NULL) return 0;

	/* Differences first, in double. A longitude near the date line is 1.8e9 in
	 * these units; converting it to metres before subtracting would spend the
	 * whole of a float's precision on the part that cancels, and leave the
	 * metres — the only part anyone cares about — quantised to tens. */
	const double coslat = cos((double)start_lat / FS_COMP_DEG_SCALE
	                          * FS_COMP_DEG_TO_RAD);

	const double de = ((double)ref_lon - (double)start_lon) / FS_COMP_DEG_SCALE
	                  * FS_COMP_M_PER_DEG * coslat;
	const double dn = ((double)ref_lat - (double)start_lat) / FS_COMP_DEG_SCALE
	                  * FS_COMP_M_PER_DEG;
	const double ce = ((double)cur_lon - (double)start_lon) / FS_COMP_DEG_SCALE
	                  * FS_COMP_M_PER_DEG * coslat;
	const double cn = ((double)cur_lat - (double)start_lat) / FS_COMP_DEG_SCALE
	                  * FS_COMP_M_PER_DEG;

	const double len = sqrt(de * de + dn * dn);
	if (len < (double)FS_COMP_MIN_PATH_M) return 0;

	/* Project the competitor's offset onto the RIGHT-HAND NORMAL of the path.
	 * With east as x and north as y, turning a direction (de, dn) ninety degrees
	 * clockwise — i.e. to the right of someone flying it — gives (dn, -de), and
	 * the dot product with (ce, cn) divided by the path's length is the signed
	 * distance to it.
	 *
	 * The whole convention is one sanity check: flying due north, (de, dn) is
	 * (0, +), the normal is (+, 0) = due east, and a competitor to the east comes
	 * out POSITIVE. East is on your right when you are flying north. */
	*out_m = (float)((ce * dn - cn * de) / len);
	return 1;
}

int FS_CompCorridor_Deviation(int32_t ref_lat, int32_t ref_lon, float *out_m)
{
	if (!s_started || !s_haveCur) return 0;

	return FS_CompCorridor_Lateral(s_startLat, s_startLon,
	                               ref_lat, ref_lon,
	                               s_curLat, s_curLon, out_m);
}

void FS_CompCorridor_Indicator(int started, int have_dev, float dev_m,
                               FS_CompInd_t *out)
{
	if (out == NULL) return;

	out->active = 0;
	out->side   = 0;
	out->bars   = 0;
	out->solid  = 0;
	out->blink  = 0;

	if (!started)  return;
	out->active = 1;
	if (!have_dev) return;

	const float mag = (dev_m < 0.0f) ? -dev_m : dev_m;

	out->side = (dev_m < 0.0f) ? -1 : ((mag > 0.0f) ? 1 : 0);

	/* STRICTLY greater. 300 m from the centreline is the lane BOUNDARY, and a
	 * competitor exactly on it is still inside — the penalty starts past it. So
	 * 300.0 draws a full ladder of twelve and 300.1 draws the solid block. */
	if (mag > FS_COMP_LANE_HALF_M)
	{
		out->solid = 1;
		/* At or beyond, because the rule band is "150 m or more outside". */
		out->blink = (mag >= FS_COMP_PENALTY2_M) ? 1 : 0;
		return;
	}

	uint32_t n = (uint32_t)(mag / FS_COMP_BAR_STEP_M);
	if (n > FS_COMP_MAX_BARS) n = FS_COMP_MAX_BARS;
	out->bars = (uint8_t)n;
}

/* --------------------------------------------------------------------------
   Geometry
   -------------------------------------------------------------------------- */

/*
 * The wearer's frame -> the panel. `u` is an offset to the wearer's RIGHT of the
 * anchor; the panel mirrors X, so moving right SUBTRACTS from x.
 *
 * This is the only place either axis is touched. If the ladder ever comes out on
 * the wrong side of the centre bar on real glasses, this function is the whole
 * of the bug — and a wrong side here is an indicator that reads perfectly and
 * steers every competitor further out of the lane.
 */
static int32_t panel_x(int16_t ax, int32_t u)
{
	return (int32_t)ax - u;
}

/* Corners in either order, stored normalised: the ActiveLook shape commands do
 * not care, but a test that has to guess which corner came first is a test
 * nobody re-reads. */
static void add_shape(FS_CompDraw_t *d, int32_t xa, int32_t ya,
                      int32_t xb, int32_t yb, uint8_t solid)
{
	if (d->count >= FS_COMP_MAX_SHAPES) return;

	FS_CompShape_t *s = &d->shape[d->count++];
	s->x0    = (int16_t)((xa < xb) ? xa : xb);
	s->x1    = (int16_t)((xa < xb) ? xb : xa);
	s->y0    = (int16_t)((ya < yb) ? ya : yb);
	s->y1    = (int16_t)((ya < yb) ? yb : ya);
	s->solid = solid;
}

/* The rung pitch for a given span, and the only place it is decided. Whole
 * pixels, never under 3: two rungs 2 px apart cannot be counted, and counting
 * them is the whole point. */
static int32_t pitch_for(int16_t span)
{
	int32_t p = (int32_t)span / (2 * FS_COMP_MAX_BARS);
	return (p < 3) ? 3 : p;
}

int16_t FS_CompCorridor_Width(int16_t span)
{
	return (int16_t)(2 * FS_COMP_MAX_BARS * pitch_for(span));
}

uint8_t FS_CompCorridor_Build(int16_t ax, int16_t ay, int16_t size,
                              int16_t span,
                              const FS_CompInd_t *ind, int side_on,
                              FS_CompDraw_t *out)
{
	if (out == NULL || ind == NULL) return 0;

	out->count = 0;

	if (!ind->active)             return 0;
	if (size < FS_COMP_MIN_SIZE)  return 0;

	/* Width comes from `span`, height from `size`. They were one number until
	 * v0.0.42, and tying the lane's width to a font size is what made it
	 * unreadable in the air: at the 24 px status font the whole instrument was
	 * 72 px, one rung every 3 px, so the full 300 m from the centreline to the
	 * lane edge fitted in the width of five digits.
	 *
	 *   pitch  the gap between rungs, whole pixels, from the span;
	 *   chw    half the centre bar's width, so the bar is 2*chw+1 px and always
	 *          odd, which is what lets it sit ON the centre column rather than
	 *          half a pixel off it;
	 *   rungh  half the height, leaving the centre bar standing proud of the
	 *          ladder by a quarter of the box at each end. That contrast is what
	 *          makes the centre bar readable at a glance against twelve rungs.
	 * Across the 304 px panel: pitch 12, ladder 288 px, one rung per 25 m of
	 * drift a full centimetre apart on the glass. */
	int32_t pitch = pitch_for(span);
	int32_t chw   = pitch / 2;          if (chw   < 1) chw   = 1;
	int32_t rungh = (int32_t)size / 2;  if (rungh < 4) rungh = 4;

	/* The centre column, measured to the wearer's right of the anchor. Putting
	 * it a full ladder in means the outermost LEFT rung lands exactly on the
	 * anchor and the outermost RIGHT one exactly on the far edge, so the element
	 * occupies precisely the box the layout gave it. */
	const int32_t uc = (int32_t)FS_COMP_MAX_BARS * pitch;

	/* Y is not mirrored: larger y is higher, and the anchor is the TOP edge. */
	const int32_t rung_top = (int32_t)ay - ((int32_t)size - rungh) / 2;
	const int32_t rung_bot = rung_top - rungh;

	add_shape(out, panel_x(ax, uc + chw), (int32_t)ay - (int32_t)size,
	               panel_x(ax, uc - chw), (int32_t)ay, 1);

	/* side_on is the blink: the caller drops the ladder on alternate frames and
	 * the centre bar above stays put. side 0 means dead on the centreline, where
	 * there is nothing to draw beside the bar. */
	if (!side_on || ind->side == 0) return out->count;

	const int32_t s = ind->side;

	if (ind->solid)
	{
		/* From the edge of the centre bar out to where the twelfth rung would
		 * be, so an outside-the-lane block covers exactly the footprint a full
		 * ladder does — the block reads as "the ladder ran out", which is what
		 * happened. */
		add_shape(out, panel_x(ax, uc + s * chw), rung_bot,
		               panel_x(ax, uc + s * (int32_t)FS_COMP_MAX_BARS * pitch),
		               rung_top, 1);
	}
	else
	{
		for (uint8_t k = 1; k <= ind->bars && k <= FS_COMP_MAX_BARS; k++)
		{
			const int32_t x = panel_x(ax, uc + s * (int32_t)k * pitch);
			add_shape(out, x, rung_bot, x, rung_top, 0);
		}
	}

	return out->count;
}
