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

	printf("flight_detect: %d/%d checks passed\n", g_checks - g_fail, g_checks);
	return g_fail ? 1 : 0;
}
