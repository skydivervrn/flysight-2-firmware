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

static bool     s_inFlight;
static bool     s_climbActive;     /* currently accumulating a climb window  */
static uint32_t s_climbStartTOW;   /* iTOW (ms) when the climb window opened  */
static bool     s_ffActive;        /* currently accumulating a freefall window*/
static uint32_t s_ffStartTOW;

void FS_FlightDetect_Init(void)
{
	s_inFlight      = false;
	s_climbActive   = false;
	s_climbStartTOW = 0;
	s_ffActive      = false;
	s_ffStartTOW    = 0;
}

bool FS_FlightDetect_InFlight(void)
{
	return s_inFlight;
}

bool FS_FlightDetect_Update(const FS_GNSS_Data_t *d)
{
	/* Already latched, or no usable sample: never re-fire. */
	if (s_inFlight) return false;
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
			s_inFlight = true;
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
			s_inFlight = true;
			return true;                               /* rising edge          */
		}
	}
	else
	{
		s_ffActive = false;
	}

	return false;
}
