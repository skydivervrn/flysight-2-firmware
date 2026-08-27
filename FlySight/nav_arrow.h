/***************************************************************************
**                                                                        **
**  FlySight 2 firmware — HUD navigation arrow geometry (PURE, testable)   **
**                                                                        **
**  Turns "the destination is 37 degrees to my right" into the handful of  **
**  line segments the glasses draw. No BLE, no HAL, no floating window     **
**  into the rest of the firmware — so the shape can be checked on the     **
**  host, which matters here more than usual: nobody can read an arrow off **
**  a unit test, and getting the mirror wrong points every pilot the wrong **
**  way round with no obvious tell.                                        **
**                                                                        **
****************************************************************************/

#ifndef NAV_ARROW_H_
#define NAV_ARROW_H_

#include <stdint.h>

/* Shaft, plus one barb each side of the tip. */
#define FS_NAV_ARROW_SEGS  3

typedef struct
{
	int16_t x0, y0;
	int16_t x1, y1;
} FS_NavSeg_t;

typedef struct
{
	/* The frame, as ActiveLook rect (0x33) wants it: two opposite corners in
	 * PANEL coordinates. */
	int16_t     box_x0, box_y0;
	int16_t     box_x1, box_y1;

	/* The arrow, as ActiveLook line (0x32) wants it. `seg_count` is 0 when the
	 * caller had no direction to draw — the frame is still filled in, because
	 * an empty box is how the HUD says "this readout exists, it has nothing to
	 * say yet", the same job the four dashes do for a number. */
	FS_NavSeg_t seg[FS_NAV_ARROW_SEGS];
	uint8_t     seg_count;
} FS_NavArrow_t;

/*
 * Build the frame and the arrow inside it.
 *
 *   ax, ay   the element's anchor, PANEL coordinates — the corner the HUD
 *            already treats as an element's origin: the wearer's TOP-LEFT.
 *   size     the side of the square, px.
 *   deg      relative bearing to the destination: 0 = dead ahead, positive =
 *            to the wearer's RIGHT, negative = left, +/-180 = behind. This is
 *            exactly what nav.c's calcDirection() returns.
 *   have_dir 0 draws the empty frame and nothing else.
 *
 * Returns 0 (and touches nothing) on a NULL `out` or a size below the smallest
 * arrow worth drawing.
 *
 * ## Which way is which
 *
 * VERIFIED on the ENGO 3 and written down in CLAUDE.md: the panel's X axis is
 * MIRRORED — a LARGER x is further to the wearer's LEFT — while Y is not, so a
 * larger y is higher up. Everything here is worked out in the wearer's frame
 * (right and up positive, forward = up, which is what the pilot means by an
 * arrow) and mirrored on the way out, in one place, at the bottom of the file.
 * Do not "simplify" that by pre-negating the angle: the whole reason the
 * mapping lives in one function is so a future reader can check it once.
 *
 * Coordinates are NOT clamped to the panel. The protocol takes signed 16-bit
 * coordinates precisely so shapes can hang off the edge and be clipped by the
 * glasses; clamping an endpoint would silently BEND the arrow, which is worse
 * than a corner that is cut off — a bent arrow still looks like a reading.
 */
int FS_NavArrow_Build(int16_t ax, int16_t ay, int16_t size,
                      int have_dir, float deg, FS_NavArrow_t *out);

/* The smallest square this draws into. Below it the barbs collapse onto the
 * shaft and the result reads as a line, not an arrow. */
#define FS_NAV_ARROW_MIN_SIZE  16

#endif /* NAV_ARROW_H_ */
