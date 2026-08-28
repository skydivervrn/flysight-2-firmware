/***************************************************************************
**                                                                        **
**  FlySight 2 firmware — HUD navigation arrow geometry (PURE, testable)   **
**                                                                        **
**  See nav_arrow.h for the API and for the one thing worth reading twice: **
**  the panel mirrors X.                                                   **
**                                                                        **
****************************************************************************/

#include "nav_arrow.h"

#include <math.h>

/* Proportions of the square, chosen so the arrow reads at a glance in the
 * smallest font's box (24 px) as well as the largest (82 px):
 *   the shaft spans 78 % of the square, centred, leaving a margin the frame
 *   does not crowd;
 *   each barb is 38 % of the shaft and opens 35 degrees off it, which is about
 *   as narrow as ENGO's one-pixel lines survive at 24 px.
 */
#define SHAFT_FRACTION   0.78f
#define BARB_FRACTION    0.38f
#define BARB_ANGLE_DEG   35.0f

#define DEG_TO_RAD       0.017453292519943295f

static int16_t round16(float v)
{
	return (int16_t)((v < 0.0f) ? (v - 0.5f) : (v + 0.5f));
}

/*
 * The wearer's frame -> the panel.
 *
 * (u, v) is an offset in the frame the pilot thinks in: u to the RIGHT, v UP.
 * The panel mirrors X and does not flip Y, so moving right means SUBTRACTING
 * from x while up still adds to y.
 *
 * (bx, by) is the point the offset is measured from, already in PANEL
 * coordinates. That works because the mapping is affine: offsetting a panel
 * point by a mirrored offset lands where mapping the whole wearer-frame point
 * would. It is what lets the barbs be measured from the tip.
 *
 * This is the only place either axis is touched. If the arrow ever comes out
 * mirrored on real glasses, this function is the whole of the bug.
 */
static void to_panel(float bx, float by, float u, float v,
                     int16_t *out_x, int16_t *out_y)
{
	*out_x = round16(bx - u);
	*out_y = round16(by + v);
}

int FS_NavArrow_Build(int16_t ax, int16_t ay, int16_t size,
                      int have_dir, float deg, FS_NavArrow_t *out)
{
	if (out == 0) return 0;
	if (size < FS_NAV_ARROW_MIN_SIZE) return 0;

	/* The anchor is the wearer's top-left, and the HUD's elements all grow
	 * from there: to the wearer's right (falling x) and downwards (falling y).
	 * The frame follows the text it sits among rather than inventing its own
	 * convention. */
	out->box_x0 = (int16_t)(ax - size);
	out->box_y0 = (int16_t)(ay - size);
	out->box_x1 = ax;
	out->box_y1 = ay;
	out->seg_count = 0;

	if (!have_dir) return 1;

	const float cx = (float)ax - (float)size / 2.0f;
	const float cy = (float)ay - (float)size / 2.0f;

	/* Half the shaft: the tip and the tail sit either side of the centre, so
	 * the arrow turns about the middle of its box the way a compass needle
	 * does. An arrow that pivoted about its tail would swing its tip out of
	 * the frame at 90 degrees. */
	const float half = (float)size * SHAFT_FRACTION / 2.0f;

	/* Forward is UP, and the bearing turns clockwise from it: a destination to
	 * the right (positive degrees) leans the tip to the right. In the wearer's
	 * axes that is (sin, cos) — not the (cos, sin) of a maths-convention angle
	 * measured anticlockwise from the x axis. */
	const float rad = deg * DEG_TO_RAD;
	const float fu  = sinf(rad);
	const float fv  = cosf(rad);

	int16_t tip_x, tip_y, tail_x, tail_y;
	to_panel(cx, cy,  fu * half,  fv * half, &tip_x,  &tip_y);
	to_panel(cx, cy, -fu * half, -fv * half, &tail_x, &tail_y);

	out->seg[0].x0 = tail_x;
	out->seg[0].y0 = tail_y;
	out->seg[0].x1 = tip_x;
	out->seg[0].y1 = tip_y;

	/* The barbs run BACK from the tip, one turned each way off the shaft. */
	const float barb = half * 2.0f * BARB_FRACTION;
	for (int i = 0; i < 2; i++)
	{
		const float turn = (i == 0) ? BARB_ANGLE_DEG : -BARB_ANGLE_DEG;
		const float a    = (deg + 180.0f + turn) * DEG_TO_RAD;

		int16_t bx, by;
		to_panel((float)tip_x, (float)tip_y,
		         sinf(a) * barb, cosf(a) * barb, &bx, &by);

		out->seg[1 + i].x0 = tip_x;
		out->seg[1 + i].y0 = tip_y;
		out->seg[1 + i].x1 = bx;
		out->seg[1 + i].y1 = by;
	}

	out->seg_count = FS_NAV_ARROW_SEGS;
	return 1;
}

FS_NavArrowState_t FS_NavArrow_State(int dest_set, int fix_ok)
{
	/* The ORDER is the whole of this function; the reasoning is on the enum in
	 * nav_arrow.h and belongs there, not repeated here. */
	if (!dest_set) return FS_NAV_ARROW_NO_DEST;
	if (!fix_ok)   return FS_NAV_ARROW_NO_FIX;
	return FS_NAV_ARROW_OK;
}

const char *FS_NavArrow_Caption(FS_NavArrowState_t state)
{
	switch (state)
	{
	case FS_NAV_ARROW_NO_DEST: return "NO LZ";
	case FS_NAV_ARROW_NO_FIX:  return "NO FIX";
	default:                   return "";
	}
}
