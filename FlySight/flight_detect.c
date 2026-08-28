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

#include "flight_detect.h"
#include <stdint.h>
#include <stddef.h>               /* for NULL */

/* --------------------------------------------------------------------------
   Thresholds (GNSS-based, rate-agnostic — durations measured via iTOW ms).
   Tuned conservatively to avoid ground false-positives; refine after the
   first test jumps. Vertical reference: velD is mm/s, POSITIVE = downward.
   -------------------------------------------------------------------------- */

/* Takeoff = sustained CLIMB. A jump aircraft climbs several m/s for minutes;
 * GPS vertical-velocity noise on the ground does not hold >2.5 m/s up for 8 s. */
#define FS_FD_CLIMB_MPS         2.5      /* upward rate to count as climbing   */
#define FS_FD_CLIMB_HOLD_MS     8000u    /* sustained for this long => takeoff */

/* Fallback = FREEFALL (in case takeoff was missed, e.g. cold boot mid-jump).
 * 10 m/s down sustained 1 s is unambiguous (matches glidex-alt freefall entry). */
#define FS_FD_FREEFALL_MPS      10.0
#define FS_FD_FREEFALL_HOLD_MS  1000u

/* iTOW counts milliseconds into the GPS week and restarts at zero every
 * Sunday 00:00:00 GPS time. One week is the ONLY backwards step in that number
 * with a defined meaning, which is what makes the correction in ElapsedSec
 * exact rather than a heuristic. */
#define FS_FD_WEEK_MS           604800000u

static bool     s_inFlight;
static bool     s_climbActive;     /* currently accumulating a climb window  */
static uint32_t s_climbStartTOW;   /* iTOW (ms) when the climb window opened  */
static bool     s_ffActive;        /* currently accumulating a freefall window*/
static uint32_t s_ffStartTOW;
static uint32_t s_takeoffTOW;      /* iTOW of the sample that latched         */
static uint32_t s_lastTOW;         /* iTOW of the newest sample since then    */

void FS_FlightDetect_Init(void)
{
	s_inFlight      = false;
	s_climbActive   = false;
	s_climbStartTOW = 0;
	s_ffActive      = false;
	s_ffStartTOW    = 0;
	s_takeoffTOW    = 0;
	s_lastTOW       = 0;
}

bool FS_FlightDetect_InFlight(void)
{
	return s_inFlight;
}

bool FS_FlightDetect_ElapsedSec(uint32_t *out)
{
	if (out == NULL || !s_inFlight) return false;

	uint32_t ms = s_lastTOW - s_takeoffTOW;

	/* A forward delta is small — days, at the very worst — so anything that
	 * comes out at or above a full week is the unsigned wrap of a BACKWARDS
	 * step, and the week rollover is what puts one there. Adding a week back
	 * (in uint32, wrapping again) recovers the true interval exactly:
	 *   takeoff 604700000, now 100000 -> raw 3690367296, +week -> 200000 ms.
	 *
	 * A backwards step that is NOT a rollover — the receiver cold-starting
	 * mid-flight and re-deriving its clock — leaves a number close to a whole
	 * week, i.e. hundreds of hours on the panel. That is deliberate: it is
	 * visibly broken, where clamping it or silently re-anchoring would show a
	 * plausible time that is wrong, and a competitor timing a run off a
	 * plausible wrong clock is worse off than one who can see the fault. */
	if (ms >= FS_FD_WEEK_MS) ms += FS_FD_WEEK_MS;

	*out = ms / 1000u;
	return true;
}

bool FS_FlightDetect_Update(const FS_GNSS_Data_t *d)
{
	/* Already latched: nothing left to detect, but the elapsed clock still has
	 * to be fed. iTOW is stamped on EVERY complete sample (gnss.c
	 * FS_GNSS_ReceiveMessage sets it before dispatching the callback), fix or no
	 * fix, and the receiver keeps its time of week through a dropout — so the
	 * clock keeps running while the sky is obstructed instead of freezing over
	 * exactly the part of a jump the wearer is watching it for. */
	if (s_inFlight)
	{
		if (d != NULL) s_lastTOW = d->iTOW;
		return false;
	}

	/* No usable sample. */
	if (d == NULL || d->gpsFix != 3)
	{
		/* Reset sustain windows on fix loss so dropouts don't accumulate; the
		 * latch itself is unaffected (here it's still false). */
		s_climbActive = false;
		s_ffActive    = false;
		return false;
	}

	const double   vDown = (double)d->velD / 1000.0;   /* m/s, +down           */
	const uint32_t tow   = d->iTOW;                     /* GPS time of week, ms */

	/* --- Takeoff: sustained climb --- */
	if (-vDown >= FS_FD_CLIMB_MPS)
	{
		if (!s_climbActive)
		{
			s_climbActive   = true;
			s_climbStartTOW = tow;
		}
		else if ((uint32_t)(tow - s_climbStartTOW) >= FS_FD_CLIMB_HOLD_MS)
		{
			s_inFlight   = true;
			s_takeoffTOW = tow;                        /* clock epoch          */
			s_lastTOW    = tow;
			return true;                               /* rising edge          */
		}
	}
	else
	{
		s_climbActive = false;
	}

	/* --- Fallback: freefall --- */
	if (vDown >= FS_FD_FREEFALL_MPS)
	{
		if (!s_ffActive)
		{
			s_ffActive   = true;
			s_ffStartTOW = tow;
		}
		else if ((uint32_t)(tow - s_ffStartTOW) >= FS_FD_FREEFALL_HOLD_MS)
		{
			s_inFlight   = true;
			s_takeoffTOW = tow;                        /* clock epoch          */
			s_lastTOW    = tow;
			return true;                               /* rising edge          */
		}
	}
	else
	{
		s_ffActive = false;
	}

	return false;
}
