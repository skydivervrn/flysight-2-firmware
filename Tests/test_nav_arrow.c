/* Host tests for FlySight/nav_arrow.c — the HUD's navigation arrow.
 *
 * Nobody can proofread an arrow by reading a coordinate list on a screen, and
 * the one mistake that matters here — mirroring — looks completely correct
 * until a pilot turns the wrong way at 300 m. So the checks below are written
 * in the wearer's language ("the tip is to my right"), with the panel's mirror
 * spelled out each time, rather than as expected-value tables that would agree
 * with a mirrored implementation just as happily as a correct one.
 *
 * The panel, VERIFIED on the ENGO 3 and recorded in CLAUDE.md:
 *   larger x  = further to the wearer's LEFT   (X is mirrored)
 *   larger y  = higher up                      (Y is not)
 */
#include <stdio.h>
#include "nav_arrow.h"

static int g_checks = 0, g_fail = 0;
#define CHECK(cond) do { \
	g_checks++; \
	if (!(cond)) { g_fail++; printf("FAIL line %d: %s\n", __LINE__, #cond); } \
} while (0)

/* The anchor used throughout: the wearer's top-left corner of a 64 px box, so
 * the box spans panel x 236..300 and panel y 136..200, and its centre is at
 * (268, 168). */
#define AX    300
#define AY    200
#define SIZE   64
#define CX    268
#define CY    168

int main(void)
{
	FS_NavArrow_t a;

	/* 1. The frame. Two opposite corners, growing from the anchor to the
	 *    wearer's right (falling x) and downwards (falling y) — the same
	 *    direction every text element grows. */
	CHECK(FS_NavArrow_Build(AX, AY, SIZE, 0, 0.0f, &a) == 1);
	CHECK(a.box_x0 == AX - SIZE && a.box_x1 == AX);
	CHECK(a.box_y0 == AY - SIZE && a.box_y1 == AY);

	/* 1b. With no direction the frame is still there and the arrow is not.
	 *     An empty box is the shape's version of four dashes. */
	CHECK(a.seg_count == 0);

	/* 2. Dead ahead points UP: the tip is higher than the tail, and neither
	 *    is off to a side. Segment 0 runs tail -> tip. */
	CHECK(FS_NavArrow_Build(AX, AY, SIZE, 1, 0.0f, &a) == 1);
	CHECK(a.seg_count == FS_NAV_ARROW_SEGS);
	CHECK(a.seg[0].y1 > a.seg[0].y0);          /* tip is higher            */
	CHECK(a.seg[0].x0 == CX && a.seg[0].x1 == CX);  /* dead centre, no lean */

	/* 2b. It pivots about the centre of the box, like a needle: the tail is
	 *     as far below the centre as the tip is above it. An arrow hinged at
	 *     its tail would swing its tip out of the frame at 90 degrees. */
	CHECK((CY - a.seg[0].y0) == (a.seg[0].y1 - CY));

	/* 2c. And it stays inside its box, so two arrows side by side cannot
	 *     draw over each other. */
	CHECK(a.seg[0].y1 <= AY && a.seg[0].y0 >= AY - SIZE);

	/* 3. THE MIRROR. A destination 90 degrees to the wearer's RIGHT must put
	 *    the tip to the wearer's right, which on this panel is the SMALLER x.
	 *    Get this backwards and every reading is a perfect, confident lie. */
	CHECK(FS_NavArrow_Build(AX, AY, SIZE, 1, 90.0f, &a) == 1);
	CHECK(a.seg[0].x1 < CX);                   /* tip to the wearer's right */
	CHECK(a.seg[0].x0 > CX);                   /* tail to the left          */
	CHECK(a.seg[0].y0 == CY && a.seg[0].y1 == CY);  /* level, no rise       */

	/* 3b. And the mirror image of that: 90 degrees LEFT puts the tip at the
	 *     larger x. */
	CHECK(FS_NavArrow_Build(AX, AY, SIZE, 1, -90.0f, &a) == 1);
	CHECK(a.seg[0].x1 > CX);
	CHECK(a.seg[0].x0 < CX);

	/* 4. Behind: the tip points DOWN. */
	CHECK(FS_NavArrow_Build(AX, AY, SIZE, 1, 180.0f, &a) == 1);
	CHECK(a.seg[0].y1 < a.seg[0].y0);

	/* 5. A destination off to the right-and-ahead leans both ways at once —
	 *    the case that catches an implementation which mirrors the angle
	 *    instead of the coordinate, because that one passes every check at
	 *    exactly 0, 90 and 180 degrees. */
	CHECK(FS_NavArrow_Build(AX, AY, SIZE, 1, 45.0f, &a) == 1);
	CHECK(a.seg[0].x1 < CX);                   /* tip to the wearer's right */
	CHECK(a.seg[0].y1 > CY);                   /* and ahead of them         */

	/* 6. The barbs. Both start at the tip and both fall BEHIND it — with the
	 *    arrow pointing up, "behind" is lower. They are also on opposite
	 *    sides of the shaft, which is what stops a "barb" being drawn on top
	 *    of the shaft itself. */
	CHECK(FS_NavArrow_Build(AX, AY, SIZE, 1, 0.0f, &a) == 1);
	const int16_t tip_x = a.seg[0].x1, tip_y = a.seg[0].y1;
	CHECK(a.seg[1].x0 == tip_x && a.seg[1].y0 == tip_y);
	CHECK(a.seg[2].x0 == tip_x && a.seg[2].y0 == tip_y);
	CHECK(a.seg[1].y1 < tip_y && a.seg[2].y1 < tip_y);
	CHECK((a.seg[1].x1 - tip_x) * (a.seg[2].x1 - tip_x) < 0);

	/* 6b. The barbs turn with the arrow rather than staying vertical: with
	 *     the arrow pointing right they fall to the wearer's LEFT of the tip,
	 *     i.e. at a larger x. */
	CHECK(FS_NavArrow_Build(AX, AY, SIZE, 1, 90.0f, &a) == 1);
	CHECK(a.seg[1].x1 > a.seg[1].x0 && a.seg[2].x1 > a.seg[2].x0);

	/* 7. A box too small to read as an arrow is refused outright, rather than
	 *    drawn as a smudge that a pilot might take for a direction. */
	CHECK(FS_NavArrow_Build(AX, AY, FS_NAV_ARROW_MIN_SIZE - 1, 1, 0.0f, &a) == 0);
	CHECK(FS_NavArrow_Build(AX, AY, FS_NAV_ARROW_MIN_SIZE, 1, 0.0f, &a) == 1);
	CHECK(FS_NavArrow_Build(AX, AY, SIZE, 1, 0.0f, 0) == 0);

	/* 8. Angles past a full turn are the same arrow, so a bearing that has
	 *    not been normalised cannot fling the tip somewhere absurd. */
	FS_NavArrow_t b;
	CHECK(FS_NavArrow_Build(AX, AY, SIZE, 1, 30.0f, &a) == 1);
	CHECK(FS_NavArrow_Build(AX, AY, SIZE, 1, 390.0f, &b) == 1);
	CHECK(a.seg[0].x1 == b.seg[0].x1 && a.seg[0].y1 == b.seg[0].y1);

	/* 9. Nothing is clamped to the panel: an arrow whose box hangs off the
	 *    edge keeps its true shape and lets the glasses clip it. Clamping
	 *    would BEND it, and a bent arrow still looks like a reading. */
	CHECK(FS_NavArrow_Build(20, 30, SIZE, 1, 0.0f, &a) == 1);
	CHECK(a.box_x0 < 0 && a.box_y0 < 0);

	printf("%d checks, %d failures\n", g_checks, g_fail);
	return g_fail ? 1 : 0;
}
