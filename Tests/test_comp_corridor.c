/* Host tests for FlySight/comp_corridor.c — the wingsuit-competition lane
 * indicator.
 *
 * Two of the things this module decides cannot be checked in the air:
 *
 *   WHEN the Validation Window opens. It is nine seconds after a moment nobody
 *   feels, and the origin it latches sets the whole lane. Off by a second is a
 *   lane rotated by a degree or two, which at 5 km is a hundred metres of
 *   corridor that is not there.
 *
 *   WHICH SIDE the competitor is on. A mirrored indicator is perfectly legible
 *   and perfectly steady, and tells them to correct further out of the lane
 *   every time. There is no tell at 200 km/h.
 *
 * So the checks below are written in the flyer's language ("east of a northward
 * path is on my right"), with the panel's mirror spelled out each time, rather
 * than as expected-value tables that a mirrored implementation would satisfy
 * just as happily as a correct one.
 *
 * The panel, VERIFIED on the ENGO 3 and recorded in CLAUDE.md:
 *   larger x  = further to the wearer's LEFT   (X is mirrored)
 *   larger y  = higher up                      (Y is not)
 */
#include <stdio.h>
#include <string.h>
#include "comp_corridor.h"

/* The panel width, spelled out rather than included: comp_corridor.c is pure
 * and knows nothing of hud_layout.h, and that is the point of it. If the two
 * ever disagree the HUD is drawing on a panel this test never checked. */
#define FS_HUD_PANEL_W_TEST 304

static int g_checks = 0, g_fail = 0;
#define CHECK(cond) do { \
	g_checks++; \
	if (!(cond)) { g_fail++; printf("FAIL line %d: %s\n", __LINE__, #cond); } \
} while (0)

#define NEAR(a, b, tol)  (((a) - (b) < (tol)) && ((b) - (a) < (tol)))

/* A DZ at 51 N, 4 E, in the degrees x 10,000,000 both CONFIG.TXT and the u-blox
 * receiver use. At this latitude a degree of longitude is 69978 m and a degree
 * of latitude 111197 m, which is where every metre quoted below comes from. */
#define LAT0   510000000
#define LON0    40000000

static FS_GNSS_Data_t mk(int32_t velD, uint32_t itow, uint8_t fix,
                         int32_t lat, int32_t lon)
{
	FS_GNSS_Data_t d;
	memset(&d, 0, sizeof(d));
	d.velD   = velD;
	d.iTOW   = itow;
	d.gpsFix = fix;
	d.lat    = lat;
	d.lon    = lon;
	return d;
}

/* One sample, so a test can say exactly which iTOW and which position the
 * detector saw. Returns 1 on the sample that opens the window. */
static int step(int32_t velD, uint32_t itow, uint8_t fix,
                int32_t lat, int32_t lon)
{
	FS_GNSS_Data_t d = mk(velD, itow, fix, lat, lon);
	return FS_CompCorridor_Update(&d) ? 1 : 0;
}

/* Feed 5 Hz samples over [t0, t1), all at the DZ, and count the openings. */
static int feed(int32_t velD, uint32_t t0, uint32_t t1)
{
	int opened = 0;
	for (uint32_t t = t0; t < t1; t += 200)
		opened += step(velD, t, 3, LAT0, LON0);
	return opened;
}

int main(void)
{
	FS_CompInd_t ind;
	FS_CompDraw_t d;
	float dev;
	int32_t olat, olon;

	/* ==== 1. The Validation Window opens 9 s after 10 m/s, and not before ==== */

	FS_CompCorridor_Init();
	CHECK(FS_CompCorridor_Started() == false);

	/* Under the threshold for twenty seconds: nothing arms. A jump aircraft's
	 * descent and a canopy flight both live here. */
	CHECK(feed(9999, 1000, 21000) == 0);
	CHECK(FS_CompCorridor_Started() == false);

	/* EXACTLY 10 m/s arms it — the rule says "reaches", not "exceeds". */
	CHECK(step(10000, 21000, 3, LAT0, LON0) == 0);

	/* Every sample of the next nine seconds is still before the window. The last
	 * one here is at 29800, 200 ms short. */
	CHECK(feed(20000, 21200, 30000) == 0);
	CHECK(FS_CompCorridor_Started() == false);

	/* And the sample at exactly +9000 ms opens it, once. */
	CHECK(step(20000, 30000, 3, LAT0 + 1000, LON0 + 2000) == 1);
	CHECK(FS_CompCorridor_Started() == true);

	/* The origin is that sample's position, which is what the lane is drawn
	 * from — not the position where 10 m/s was first seen, nine seconds and
	 * roughly a kilometre of track earlier. */
	CHECK(FS_CompCorridor_Origin(&olat, &olon) == true);
	CHECK(olat == LAT0 + 1000 && olon == LON0 + 2000);

	/* It never fires twice: the caller writes one line to EVENT.CSV from it. */
	CHECK(feed(20000, 30200, 40000) == 0);

	/* ==== 2. A dip below 10 m/s does not restart the count ==== */

	FS_CompCorridor_Init();
	CHECK(step(10000, 1000, 3, LAT0, LON0) == 0);

	/* Four seconds of level flight in the middle of the nine. "First reaches" is
	 * what the rule says; re-arming here would move the origin four seconds and
	 * some hundreds of metres down-track from where the judges put it. */
	CHECK(feed(2000, 1200, 5000) == 0);
	CHECK(feed(20000, 5000, 10000) == 0);
	CHECK(FS_CompCorridor_Started() == false);

	/* Still 9 s from the FIRST 10 m/s sample, at 1000. */
	CHECK(step(20000, 10000, 3, LAT0, LON0) == 1);

	/* ==== 3. A fix dropout does not move the instant either ==== */

	FS_CompCorridor_Init();
	CHECK(step(10000, 1000, 3, LAT0, LON0) == 0);
	/* Six seconds with no fix: no position to latch, no velocity to trust, and
	 * nothing about the moment the competitor reached 10 m/s has changed. */
	for (uint32_t t = 1200; t < 7000; t += 200)
		CHECK(step(20000, t, 0, LAT0, LON0) == 0);
	CHECK(feed(20000, 7000, 10000) == 0);
	CHECK(step(20000, 10000, 3, LAT0, LON0) == 1);

	/* 3b. A dropout ACROSS the nine-second mark makes the origin late rather
	 *     than invented — the first fixed sample after it. */
	FS_CompCorridor_Init();
	CHECK(step(10000, 1000, 3, LAT0, LON0) == 0);
	for (uint32_t t = 1200; t < 12000; t += 200)
		CHECK(step(20000, t, 0, LAT0, LON0) == 0);
	CHECK(FS_CompCorridor_Started() == false);
	CHECK(step(20000, 12000, 3, LAT0 + 500, LON0) == 1);
	CHECK(FS_CompCorridor_Origin(&olat, &olon) == true);
	CHECK(olat == LAT0 + 500);

	/* 3c. Nothing at all arms without a fix in the first place. */
	FS_CompCorridor_Init();
	for (uint32_t t = 1000; t < 30000; t += 200)
		CHECK(step(20000, t, 0, LAT0, LON0) == 0);
	CHECK(FS_CompCorridor_Started() == false);
	CHECK(FS_CompCorridor_Update(NULL) == false);

	/* ==== 4. THE SIGN. Positive is the flyer's RIGHT ==== */

	/* A path running due NORTH: the reference point is 0.05 deg of latitude up
	 * from the origin, 5560 m away. A competitor 0.001 deg of longitude EAST of
	 * it is 69.98 m to the east — and east is on your right when you fly north.
	 * Get this backwards and the ladder grows on the wrong side. */
	CHECK(FS_CompCorridor_Lateral(LAT0, LON0,
	                              LAT0 + 500000, LON0,
	                              LAT0 + 200000, LON0 + 10000, &dev) == 1);
	CHECK(NEAR(dev, 69.978f, 0.05f));

	/* The mirror image of it: west of the same path is negative. */
	CHECK(FS_CompCorridor_Lateral(LAT0, LON0,
	                              LAT0 + 500000, LON0,
	                              LAT0 + 200000, LON0 - 10000, &dev) == 1);
	CHECK(NEAR(dev, -69.978f, 0.05f));

	/* Turn the path through ninety degrees and the answer has to turn with it.
	 * A path running due EAST, with the competitor 0.001 deg of latitude NORTH
	 * of it: 111.20 m, and north is on your LEFT when you fly east. This is the
	 * check a "take the east offset and call it the deviation" implementation
	 * fails — that one passes the northward case perfectly. */
	CHECK(FS_CompCorridor_Lateral(LAT0, LON0,
	                              LAT0, LON0 + 500000,
	                              LAT0 + 10000, LON0 + 200000, &dev) == 1);
	CHECK(NEAR(dev, -111.197f, 0.05f));

	/* South of an eastward path is to the right. */
	CHECK(FS_CompCorridor_Lateral(LAT0, LON0,
	                              LAT0, LON0 + 500000,
	                              LAT0 - 10000, LON0 + 200000, &dev) == 1);
	CHECK(NEAR(dev, 111.197f, 0.05f));

	/* On the centreline, near and far along it, is zero either way. */
	CHECK(FS_CompCorridor_Lateral(LAT0, LON0, LAT0 + 500000, LON0,
	                              LAT0 + 250000, LON0, &dev) == 1);
	CHECK(NEAR(dev, 0.0f, 0.05f));
	/* Including behind the origin: the lane is a line, not a ray, and a
	 * competitor who drifts back up-track is still in it. */
	CHECK(FS_CompCorridor_Lateral(LAT0, LON0, LAT0 + 500000, LON0,
	                              LAT0 - 100000, LON0, &dev) == 1);
	CHECK(NEAR(dev, 0.0f, 0.05f));

	/* 4b. A path with no length has no direction. 30 m of it is GPS noise, and a
	 *     lane centred on noise would swing through a half-turn while the
	 *     competitor flew dead straight. Refused rather than guessed. */
	dev = 1234.0f;
	CHECK(FS_CompCorridor_Lateral(LAT0, LON0, LAT0, LON0,
	                              LAT0 + 10000, LON0, &dev) == 0);
	CHECK(dev == 1234.0f);
	CHECK(FS_CompCorridor_Lateral(LAT0, LON0, LAT0 + 2700, LON0,
	                              LAT0 + 10000, LON0, &dev) == 0);   /* ~30 m */
	/* Just over the 100 m floor it works again. */
	CHECK(FS_CompCorridor_Lateral(LAT0, LON0, LAT0 + 10000, LON0,
	                              LAT0 + 5000, LON0 + 10000, &dev) == 1);

	/* 4c. Zero is how CONFIG.TXT says nobody assigned a Ground Reference
	 *     Point — the same rule the landing zone's Lat/Lon use. */
	CHECK(FS_CompCorridor_RefSet(0, 0) == 0);
	CHECK(FS_CompCorridor_RefSet(LAT0, LON0) != 0);
	CHECK(FS_CompCorridor_RefSet(0, LON0) != 0);
	CHECK(FS_CompCorridor_RefSet(LAT0, 0) != 0);

	/* 4d. The live accessor says nothing before the window opens, whatever the
	 *     receiver is doing. */
	FS_CompCorridor_Init();
	step(0, 1000, 3, LAT0, LON0);
	dev = 1234.0f;
	CHECK(FS_CompCorridor_Deviation(LAT0 + 500000, LON0, &dev) == 0);
	CHECK(dev == 1234.0f);

	/* ...and reads off the newest fixed sample once it has. */
	CHECK(step(10000, 2000, 3, LAT0, LON0) == 0);
	CHECK(step(20000, 11000, 3, LAT0, LON0) == 1);
	CHECK(step(20000, 11200, 3, LAT0 + 200000, LON0 + 10000) == 0);
	CHECK(FS_CompCorridor_Deviation(LAT0 + 500000, LON0, &dev) == 1);
	CHECK(NEAR(dev, 69.978f, 0.05f));

	/* ==== 5. Quantisation: one rung per 25 m, twelve to the lane edge ==== */

	/* Inside the first 25 m there is nothing to say beyond the centre bar. */
	FS_CompCorridor_Indicator(1, 1, 0.0f, &ind);
	CHECK(ind.active == 1 && ind.bars == 0 && ind.solid == 0 && ind.side == 0);

	FS_CompCorridor_Indicator(1, 1, 24.999f, &ind);
	CHECK(ind.bars == 0 && ind.side == 1);

	/* EXACTLY 25 m is the first rung, not the last of none. */
	FS_CompCorridor_Indicator(1, 1, 25.0f, &ind);
	CHECK(ind.bars == 1 && ind.side == 1 && ind.solid == 0);

	FS_CompCorridor_Indicator(1, 1, 49.999f, &ind);
	CHECK(ind.bars == 1);
	FS_CompCorridor_Indicator(1, 1, 50.0f, &ind);
	CHECK(ind.bars == 2);

	/* The left side counts the same way; only the sign differs. */
	FS_CompCorridor_Indicator(1, 1, -50.0f, &ind);
	CHECK(ind.bars == 2 && ind.side == -1);
	FS_CompCorridor_Indicator(1, 1, -24.999f, &ind);
	CHECK(ind.bars == 0 && ind.side == -1);

	/* ==== 6. The lane edge at 300 m, and the penalty band at 450 ==== */

	FS_CompCorridor_Indicator(1, 1, 299.999f, &ind);
	CHECK(ind.bars == 11 && ind.solid == 0);

	/* 300 m from the centreline is the BOUNDARY of the Designated Lane. A
	 * competitor exactly on it is still inside, so it draws the full ladder of
	 * twelve rather than the solid block — the -10 % starts past it. */
	FS_CompCorridor_Indicator(1, 1, 300.0f, &ind);
	CHECK(ind.bars == FS_COMP_MAX_BARS && ind.solid == 0 && ind.blink == 0);

	FS_CompCorridor_Indicator(1, 1, 300.001f, &ind);
	CHECK(ind.solid == 1 && ind.bars == 0 && ind.blink == 0 && ind.side == 1);
	FS_CompCorridor_Indicator(1, 1, -300.001f, &ind);
	CHECK(ind.solid == 1 && ind.blink == 0 && ind.side == -1);

	/* 449 m from the centreline is 149 m outside: still the -10 % band. */
	FS_CompCorridor_Indicator(1, 1, 449.999f, &ind);
	CHECK(ind.solid == 1 && ind.blink == 0);

	/* 450 m is 150 m outside, where the penalty doubles to -20 %, and the block
	 * starts blinking. "150 m or more", so the boundary itself blinks. */
	FS_CompCorridor_Indicator(1, 1, 450.0f, &ind);
	CHECK(ind.solid == 1 && ind.blink == 1);
	FS_CompCorridor_Indicator(1, 1, -450.0f, &ind);
	CHECK(ind.solid == 1 && ind.blink == 1 && ind.side == -1);
	FS_CompCorridor_Indicator(1, 1, 5000.0f, &ind);
	CHECK(ind.solid == 1 && ind.blink == 1);

	/* ==== 7. Nothing at all before the latch ==== */

	/* Not an empty box, not a lone centre bar: NOTHING. The centre bar appearing
	 * is the signal the competitor is waiting for, and it cannot be a signal if
	 * something was already there. */
	FS_CompCorridor_Indicator(0, 1, 0.0f, &ind);
	CHECK(ind.active == 0 && ind.bars == 0 && ind.solid == 0 && ind.blink == 0);
	CHECK(FS_CompCorridor_Build(296, 232, 24, 72, &ind, 1, &d) == 0);
	CHECK(d.count == 0);

	/* Even a deviation that would otherwise fill the screen draws nothing. */
	FS_CompCorridor_Indicator(0, 1, 900.0f, &ind);
	CHECK(ind.active == 0);
	CHECK(FS_CompCorridor_Build(296, 232, 24, 72, &ind, 1, &d) == 0);

	/* Latched but with no position this tick: the centre bar alone. The window
	 * is open — that fact does not stop being true because the sky is blocked —
	 * and freezing the last ladder would be a reading, not a silence. */
	FS_CompCorridor_Indicator(1, 0, 900.0f, &ind);
	CHECK(ind.active == 1 && ind.bars == 0 && ind.solid == 0 && ind.blink == 0);
	CHECK(FS_CompCorridor_Build(296, 232, 24, 72, &ind, 1, &d) == 1);
	CHECK(d.shape[0].solid == 1);

	/* ==== 8. THE MIRROR ==== */

	/* The status-line anchor and font: x=296 is the wearer's LEFT edge, 24 px
	 * tall. The ladder is 72 px wide, its centre column 36 px to the wearer's
	 * right of the anchor — at panel x 260, since right is the FALLING x. */
	FS_CompCorridor_Indicator(1, 1, 100.0f, &ind);      /* 4 rungs, right side */
	CHECK(ind.bars == 4 && ind.side == 1);
	CHECK(FS_CompCorridor_Build(296, 232, 24, 72, &ind, 1, &d) == 5);

	/* shape[0] is always the centre bar: solid, three pixels wide about the
	 * centre column, and the full height of the element. */
	CHECK(d.shape[0].solid == 1);
	CHECK(d.shape[0].x0 == 259 && d.shape[0].x1 == 261);
	CHECK(d.shape[0].y0 == 208 && d.shape[0].y1 == 232);

	/* THE ONE THAT MATTERS: the competitor is RIGHT of the path, so every rung
	 * must be to their right of the centre bar — which on this panel is the
	 * SMALLER x. */
	for (uint8_t i = 1; i < d.count; i++)
	{
		CHECK(d.shape[i].x1 < d.shape[0].x0);
		CHECK(d.shape[i].solid == 0);
		CHECK(d.shape[i].x0 == d.shape[i].x1);       /* a vertical rung */
		CHECK(d.shape[i].y0 == 214 && d.shape[i].y1 == 226);
	}
	/* Evenly spaced, 3 px apart, the first just outside the centre bar. */
	CHECK(d.shape[1].x0 == 257 && d.shape[4].x0 == 248);

	/* And the mirror image: LEFT of the path puts the rungs at the LARGER x. */
	FS_CompCorridor_Indicator(1, 1, -100.0f, &ind);
	CHECK(FS_CompCorridor_Build(296, 232, 24, 72, &ind, 1, &d) == 5);
	for (uint8_t i = 1; i < d.count; i++)
		CHECK(d.shape[i].x0 > d.shape[0].x1);
	CHECK(d.shape[1].x0 == 263 && d.shape[4].x0 == 272);

	/* A full ladder ends exactly on the edges of the box the layout gave the
	 * element: the anchor itself on the left, one width in on the right. */
	FS_CompCorridor_Indicator(1, 1, 300.0f, &ind);
	CHECK(FS_CompCorridor_Build(296, 232, 24, 72, &ind, 1, &d) == 13);
	CHECK(d.shape[12].x0 == 296 - 72);
	FS_CompCorridor_Indicator(1, 1, -300.0f, &ind);
	CHECK(FS_CompCorridor_Build(296, 232, 24, 72, &ind, 1, &d) == 13);
	CHECK(d.shape[12].x0 == 296);

	/* ==== 9. Outside the lane: one solid block, on the correct side ==== */

	FS_CompCorridor_Indicator(1, 1, 400.0f, &ind);
	CHECK(FS_CompCorridor_Build(296, 232, 24, 72, &ind, 1, &d) == 2);
	CHECK(d.shape[1].solid == 1);
	/* It covers the footprint the twelve rungs would have, to the wearer's
	 * right — from just outside the centre bar out to the far edge. */
	CHECK(d.shape[1].x0 == 224 && d.shape[1].x1 == 259);
	CHECK(d.shape[1].y0 == 214 && d.shape[1].y1 == 226);
	/* ...and never crosses the centre bar onto the other side. */
	CHECK(d.shape[1].x1 < d.shape[0].x1);

	FS_CompCorridor_Indicator(1, 1, -400.0f, &ind);
	CHECK(FS_CompCorridor_Build(296, 232, 24, 72, &ind, 1, &d) == 2);
	CHECK(d.shape[1].x0 == 261 && d.shape[1].x1 == 296);
	CHECK(d.shape[1].x0 > d.shape[0].x0);

	/* ==== 10. The blink, and the size floor ==== */

	/* side_on 0 is the dark half of the blink: the block goes, the centre bar
	 * stays. A reference that disappears is not a reference. */
	FS_CompCorridor_Indicator(1, 1, 500.0f, &ind);
	CHECK(ind.blink == 1);
	CHECK(FS_CompCorridor_Build(296, 232, 24, 72, &ind, 0, &d) == 1);
	CHECK(d.shape[0].solid == 1 && d.shape[0].x0 == 259);

	/* The same switch blanks a ladder, which is what makes the caller's blink
	 * one line rather than two cases. */
	FS_CompCorridor_Indicator(1, 1, 100.0f, &ind);
	CHECK(FS_CompCorridor_Build(296, 232, 24, 72, &ind, 0, &d) == 1);

	/* A box too small to count rungs in is refused outright, rather than drawn
	 * as a smudge a competitor might try to read. */
	CHECK(FS_CompCorridor_Build(296, 232, FS_COMP_MIN_SIZE - 1, 72, &ind, 1, &d) == 0);
	CHECK(FS_CompCorridor_Build(296, 232, FS_COMP_MIN_SIZE, 72, &ind, 1, &d) > 0);
	CHECK(FS_CompCorridor_Build(296, 232, 24, 72, NULL, 1, &d) == 0);
	CHECK(FS_CompCorridor_Build(296, 232, 24, 72, &ind, 1, NULL) == 0);

	/* Nothing is clamped to the panel: an element parked at the wearer's right
	 * edge lets the glasses clip the ladder, because moving a rung inwards would
	 * change the number the competitor counts. */
	FS_CompCorridor_Indicator(1, 1, 300.0f, &ind);
	CHECK(FS_CompCorridor_Build(20, 232, 24, 72, &ind, 1, &d) == 13);
	CHECK(d.shape[12].x0 < 0);

	/* ==== 9. THE LANE GETS THE PANEL ====
	 *
	 * Flown 2026-09-05: the indicator worked and was useless, because its width
	 * came from the font. At the 24 px status font the rungs were 3 px apart, so
	 * the 300 m from centreline to lane edge was 36 px of glass and a correction
	 * a competitor actually flew moved nothing he could see.
	 *
	 * Width is now the caller's to give. Across the panel a rung is 12 px, so
	 * 25 m of drift is a centimetre on the glass — the thing being measured
	 * here is not the arithmetic but whether the instrument is legible. */
	CHECK(FS_CompCorridor_Width(FS_HUD_PANEL_W_TEST) == 288);

	/* Whole pixels only: 304 does not divide by 24, and a ladder counted in
	 * fractions of a pixel is a ladder miscounted. It rounds DOWN, which is why
	 * the caller must centre on the width and not on the span it asked for. */
	CHECK(288 % (2 * FS_COMP_MAX_BARS) == 0);
	CHECK(FS_CompCorridor_Width(FS_HUD_PANEL_W_TEST) < FS_HUD_PANEL_W_TEST);

	/* The old footprint still comes out where it always did, so a caller that
	 * wants a small ladder can still have one. */
	CHECK(FS_CompCorridor_Width(72) == 72);

	/* And the floor holds: a span too narrow for 3 px rungs does not silently
	 * draw a smudge, it widens to the smallest countable ladder. */
	CHECK(FS_CompCorridor_Width(24) == 72);

	/* Centred the way the HUD centres it, the full-width ladder lands inside
	 * the panel with an even margin, and its outermost rungs are the lane
	 * boundary — the two pixels a competitor is actually steering between. */
	{
		const int16_t w  = FS_CompCorridor_Width(FS_HUD_PANEL_W_TEST);
		const int16_t ax = (int16_t)((FS_HUD_PANEL_W_TEST + w) / 2);

		FS_CompCorridor_Indicator(1, 1, 300.0f, &ind);   /* full ladder, right */
		CHECK(ind.bars == FS_COMP_MAX_BARS && ind.solid == 0);
		CHECK(FS_CompCorridor_Build(ax, 232, 24, FS_HUD_PANEL_W_TEST,
		                            &ind, 1, &d) == 13);

		/* Both ends land ON the panel — the outermost rung is the lane edge,
		 * and a lane edge clipped by the glass is a boundary the competitor
		 * cannot see himself crossing. */
		CHECK(d.shape[12].x0 == ax - w);        /* far rung, wearer's right */
		CHECK(d.shape[12].x0 >= 0);
		CHECK(ax <= FS_HUD_PANEL_W_TEST - 1);

		/* Centred to within the odd pixel: 304 minus a 288 ladder leaves 16 to
		 * share, and the halves cannot both be 8. */
		{
			const int left  = ax - w;                        /* 8 */
			const int right = FS_HUD_PANEL_W_TEST - 1 - ax;  /* 7 */
			CHECK(left - right <= 1 && right - left <= 1);
		}

		/* The centre bar sits ON the middle of the panel, which is what the
		 * whole instrument is read against. */
		CHECK((d.shape[0].x0 + d.shape[0].x1) / 2 == FS_HUD_PANEL_W_TEST / 2);

		/* One rung per 25 m, twelve pixels apart rather than three. */
		CHECK(d.shape[2].x0 - d.shape[1].x0 == -12);
	}

	printf("%d checks, %d failures\n", g_checks, g_fail);
	return g_fail ? 1 : 0;
}
