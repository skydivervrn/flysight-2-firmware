/* Host tests for FlySight/flight_detect.c — pure, no HAL needed. */
#include <stdio.h>
#include <string.h>
#include "flight_detect.h"

static int g_checks = 0, g_fail = 0;
#define CHECK(cond) do { \
	g_checks++; \
	if (!(cond)) { g_fail++; printf("FAIL line %d: %s\n", __LINE__, #cond); } \
} while (0)

/* Build a GNSS sample: velD in mm/s (+down), iTOW in ms, gpsFix. */
static FS_GNSS_Data_t mk(int32_t velD, uint32_t itow, uint8_t fix)
{
	FS_GNSS_Data_t d;
	memset(&d, 0, sizeof(d));
	d.velD = velD;
	d.iTOW = itow;
	d.gpsFix = fix;
	d.numSV = 9;
	return d;
}

/* One sample, so a test can say exactly which iTOW the detector saw. */
static int step(int32_t velD, uint32_t itow, uint8_t fix)
{
	FS_GNSS_Data_t d = mk(velD, itow, fix);
	return FS_FlightDetect_Update(&d) ? 1 : 0;
}

/* Climb at 5 Hz from `t0` until the latch fires, and return the iTOW of the
 * sample that fired it — the epoch the elapsed clock is supposed to use. */
static uint32_t climb_until_detected(uint32_t t0)
{
	uint32_t t = t0;
	for (int i = 0; i < 1000; i++)
	{
		if (step(-3000, t, 3)) return t;
		t += 200;
	}
	return 0;   /* never fired: the CHECKs on the caller's side will say so */
}

#define WEEK_MS 604800000u

/* Like climb_until_detected, but iTOW wraps at the end of the GPS week the
 * way the receiver's does: returns the iTOW of the firing sample, or
 * 0xFFFFFFFF if it never fired within `n` samples. */
static uint32_t until_detected_wrapping(int32_t velD, uint32_t t0, int n)
{
	uint32_t t = t0;
	for (int i = 0; i < n; i++)
	{
		if (step(velD, t, 3)) return t;
		t = (t + 200u) % WEEK_MS;
	}
	return 0xFFFFFFFFu;
}

/* Feed N samples at 5 Hz (200 ms) with a constant velD, starting at t0.
 * Returns how many times Update() reported a rising edge. */
static int feed(int32_t velD, uint8_t fix, uint32_t t0, int n, uint32_t *t_out)
{
	int edges = 0;
	uint32_t t = t0;
	for (int i = 0; i < n; i++)
	{
		FS_GNSS_Data_t d = mk(velD, t, fix);
		if (FS_FlightDetect_Update(&d)) edges++;
		t += 200;
	}
	if (t_out) *t_out = t;
	return edges;
}

int main(void)
{
	/* 1. Fresh state: not in flight. */
	FS_FlightDetect_Init();
	CHECK(FS_FlightDetect_InFlight() == false);

	/* 2. Ground noise (small velD, fix) never triggers. */
	feed(300, 3, 1000, 100, NULL);    /* 0.3 m/s, 20 s */
	feed(-300, 3, 30000, 100, NULL);  /* -0.3 m/s, 20 s */
	CHECK(FS_FlightDetect_InFlight() == false);

	/* 3. Sustained climb -> takeoff after >= 8 s. */
	FS_FlightDetect_Init();
	uint32_t t;
	/* 3 m/s up for 7 s: not yet (7 s < 8 s hold). */
	int e1 = feed(-3000, 3, 1000, 35, &t);   /* 35*200ms = 7.0 s */
	CHECK(e1 == 0);
	CHECK(FS_FlightDetect_InFlight() == false);
	/* keep climbing past 8 s total -> exactly one rising edge. */
	int e2 = feed(-3000, 3, t, 20, NULL);
	CHECK(e2 == 1);
	CHECK(FS_FlightDetect_InFlight() == true);
	/* Latched: further updates never re-fire. */
	int e3 = feed(-3000, 3, 100000, 50, NULL);
	CHECK(e3 == 0);
	CHECK(FS_FlightDetect_InFlight() == true);

	/* 4. Freefall fallback: 12 m/s down for >= 1 s. */
	FS_FlightDetect_Init();
	int e4 = feed(12000, 3, 1000, 10, NULL);  /* 2 s */
	CHECK(e4 == 1);
	CHECK(FS_FlightDetect_InFlight() == true);

	/* 5. No 3D fix => never triggers, even with strong climb. */
	FS_FlightDetect_Init();
	feed(-5000, 2, 1000, 100, NULL);  /* fix=2, 20 s of climb */
	CHECK(FS_FlightDetect_InFlight() == false);

	/* 6. Fix dropout resets the sustain window. */
	FS_FlightDetect_Init();
	feed(-3000, 3, 1000, 30, &t);     /* 6 s climbing */
	CHECK(FS_FlightDetect_InFlight() == false);
	feed(-3000, 0, t, 5, &t);         /* fix lost 1 s -> window reset */
	CHECK(FS_FlightDetect_InFlight() == false);
	int e6 = feed(-3000, 3, t, 35, NULL); /* needs full 8 s again: 7 s -> no */
	CHECK(e6 == 0);
	CHECK(FS_FlightDetect_InFlight() == false);

	/* ---- The elapsed clock (HUD field 106) ---------------------------- */

	/* 7. Before a takeoff there is no reading AT ALL: the accessor says so and
	 *    does not touch the caller's variable, which is what lets the HUD draw
	 *    "--:--" instead of a zero that looks like a running clock. */
	FS_FlightDetect_Init();
	uint32_t sec = 0xDEADBEEFu;
	CHECK(FS_FlightDetect_ElapsedSec(&sec) == false);
	CHECK(sec == 0xDEADBEEFu);
	feed(300, 3, 1000, 100, NULL);        /* 20 s of ground noise */
	CHECK(FS_FlightDetect_ElapsedSec(&sec) == false);
	CHECK(sec == 0xDEADBEEFu);

	/* 8. The clock starts at the DETECTING sample. Climbing from iTOW 1000 at
	 *    5 Hz, the window opens on the first sample and the latch fires 8 s
	 *    later, at iTOW 9000 — which must read 0, not 8. */
	FS_FlightDetect_Init();
	uint32_t t0 = climb_until_detected(1000);
	CHECK(t0 == 9000);
	CHECK(FS_FlightDetect_ElapsedSec(&sec) == true);
	CHECK(sec == 0);

	/* ...and then counts the samples that follow. */
	step(-3000, t0 + 1000, 3);
	CHECK(FS_FlightDetect_ElapsedSec(&sec) == true);
	CHECK(sec == 1);
	feed(-3000, 3, t0 + 1200, 150, NULL); /* last sample at t0 + 31000 */
	CHECK(FS_FlightDetect_ElapsedSec(&sec) == true);
	CHECK(sec == 31);

	/* 9. It keeps running through a fix dropout. iTOW is stamped on every
	 *    complete sample whatever the fix, and a clock that stopped whenever the
	 *    sky was obstructed would be at its most wrong exactly when it is being
	 *    read. */
	step(0, t0 + 40000, 0);
	CHECK(FS_FlightDetect_ElapsedSec(&sec) == true);
	CHECK(sec == 40);

	/* 10. Init() puts the clock away again with the latch. */
	FS_FlightDetect_Init();
	sec = 0xDEADBEEFu;
	CHECK(FS_FlightDetect_ElapsedSec(&sec) == false);
	CHECK(sec == 0xDEADBEEFu);

	/* 11. The GPS week rollover. Take off 100 s before iTOW restarts
	 *     (604800000 ms = one week) and keep flying past it: the reading has to
	 *     cross the boundary as 100, 101, 102 s, not jump to seven weeks. */
	FS_FlightDetect_Init();
	uint32_t tw = climb_until_detected(604800000u - 108000u);
	CHECK(tw == 604800000u - 100000u);    /* 100 s left in the week */
	CHECK(FS_FlightDetect_ElapsedSec(&sec) == true);
	CHECK(sec == 0);

	step(-3000, 604800000u - 200u, 3);    /* last sample before the wrap */
	CHECK(FS_FlightDetect_ElapsedSec(&sec) == true);
	CHECK(sec == 99);

	step(-3000, 0, 3);                    /* the wrap itself */
	CHECK(FS_FlightDetect_ElapsedSec(&sec) == true);
	CHECK(sec == 100);

	step(-3000, 2000, 3);                 /* and on the far side of it */
	CHECK(FS_FlightDetect_ElapsedSec(&sec) == true);
	CHECK(sec == 102);

	/* ---- The week rollover in the sustain windows ---------------------- */

	/* 12. A climb that opens 4 s before iTOW restarts must still need its full
	 *     8 s: it fires at iTOW 4000 of the new week, not on the first sample
	 *     after the wrap (where tow - start, in uint32, is ~3.7 billion). */
	FS_FlightDetect_Init();
	CHECK(until_detected_wrapping(-3000, WEEK_MS - 4000u, 100) == 4000u);
	CHECK(FS_FlightDetect_InFlight() == true);

	/* 13. A rollover with no sustained climb triggers nothing: ground noise
	 *     across the wrap, then a 3 s climb burst straddling it (window opens
	 *     before, sample after the wrap must not satisfy the hold), then level. */
	FS_FlightDetect_Init();
	CHECK(until_detected_wrapping(300, WEEK_MS - 10000u, 100) == 0xFFFFFFFFu);
	FS_FlightDetect_Init();
	CHECK(until_detected_wrapping(-3000, WEEK_MS - 1000u, 15) == 0xFFFFFFFFu);
	CHECK(until_detected_wrapping(0, 2000u, 50) == 0xFFFFFFFFu);
	CHECK(FS_FlightDetect_InFlight() == false);

	/* 14. Same for the freefall fallback: a window opened 400 ms before the
	 *     wrap needs its 1 s, i.e. fires at iTOW 600, not at iTOW 0. */
	FS_FlightDetect_Init();
	CHECK(until_detected_wrapping(12000, WEEK_MS - 400u, 20) == 600u);

	printf("flight_detect: %d/%d checks passed\n", g_checks - g_fail, g_checks);
	return g_fail ? 1 : 0;
}
