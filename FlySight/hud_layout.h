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
 * ENGO 3 HUD layout — WHAT is drawn WHERE.
 *
 * Until v0.0.15 the layout was baked into activelook_mode0.c (a fixed rowPos[]
 * table). This module makes it data: a list of elements, each with a field, a
 * panel position, a font and a precision, plus one global offset applied to
 * every element (the "screen position" adjustment the native ActiveLook app
 * offers, for glasses whose optics sit slightly off-centre on the face).
 *
 * Deliberately free of every FlySight dependency (no HAL, no FatFs, no config)
 * so the geometry is unit-testable on the host — see Tests/test_hud_layout.c.
 *
 * PANEL GEOMETRY, verified on ENGO 3 hardware (Mac BLE bench, 2026-07-20):
 *   - 304 x 256 px, text drawn with rotation 4.
 *   - X IS MIRRORED: x=296 anchors at the VIEWER'S LEFT edge and the string
 *     grows toward smaller x (viewer-right). x=0 is the viewer's right edge.
 *   - Y is NOT flipped: larger y is higher on screen; glyphs draw DOWNWARD
 *     from the anchor, so an element's anchor is its TOP edge.
 * A consequence of the X mirror: "move the picture right, as the wearer sees
 * it" means DECREASING x. FS_HudLayout_Place takes the offset in VIEWER terms
 * and flips it, so the app can speak plain left/right/up/down.
 */

#ifndef HUD_LAYOUT_H_
#define HUD_LAYOUT_H_

#include <stdint.h>

/* Panel size in pixels (ENGO 2 and ENGO 3 alike). */
#define FS_HUD_PANEL_W  304
#define FS_HUD_PANEL_H  256

/* Room for the four data slots plus the info line, with spare capacity so a
 * future field does not force a config-format change. */
#define FS_HUD_MAX_ELEMENTS  8

/* Number of built-in data slots (the default layout, minus the info line). */
#define FS_HUD_DEFAULT_SLOTS  4

/* Field ids. 0..13 are the FS_CONFIG_MODE_* audio modes, reused verbatim so a
 * HUD element and an audio mode name the same quantity. 14 and 100+ are HUD-only
 * and start above the audio range on purpose. */
#define FS_HUD_FIELD_HSPEED     0   /* horizontal speed, raw GPS               */
#define FS_HUD_FIELD_VSPEED     1   /* vertical speed, GNSS velD, + = down     */
#define FS_HUD_FIELD_GLIDE      2   /* glide ratio                             */
#define FS_HUD_FIELD_INV_GLIDE  3
#define FS_HUD_FIELD_TOT_SPEED  4
#define FS_HUD_FIELD_DIR_DEST   5
#define FS_HUD_FIELD_DIST_DEST  6
#define FS_HUD_FIELD_DIR_BRG    7
#define FS_HUD_FIELD_DIVE       11
#define FS_HUD_FIELD_GPS_ALT    12  /* GPS altitude above DZ_Elev              */
#define FS_HUD_FIELD_HEADING    13
#define FS_HUD_FIELD_BARO_ALT   14  /* barometric altitude, zeroed at power-on */

/* The destination, drawn instead of read: a square frame with an arrow turning
 * inside it (forward is up) and the distance in small text underneath. Fields
 * 5 and 6 say the same two things as numbers; this says them as a picture,
 * which is what a pilot under canopy can take in without doing arithmetic.
 *
 * It deliberately IGNORES Max_Dist. Field 5 stops computing a direction beyond
 * that limit and returns 0.0, which renders as "dead ahead" — indistinguishable
 * from being on course, and the one failure mode you must not have in a shape
 * whose whole job is to point. The bearing is good at any range, so the arrow
 * draws at any range. */
#define FS_HUD_FIELD_NAV_ARROW  15

#define FS_HUD_FIELD_INFO       100 /* the status line (battery/sats/version)  */

/* The status line, taken apart (v0.0.18). Field 100 draws all five pieces as
 * one string in one font in one place; 101..105 each draw one piece as an
 * ordinary element, so a wearer can drop the version, move the satellite count
 * somewhere it does not crowd an altitude, or keep nothing at all. Field 100
 * stays exactly as it was — every card in the field has it. */
#define FS_HUD_FIELD_AL_BATT    101 /* glasses battery, "80%"  / "A:80%"       */
#define FS_HUD_FIELD_FS_BATT    102 /* FlySight battery, "76%" / "F:76%"       */
#define FS_HUD_FIELD_SATS       103 /* satellites, "12"        / "N:12"        */
#define FS_HUD_FIELD_VERSION    104 /* HUD version, "v0.0.18"                  */
#define FS_HUD_FIELD_FD_MARK    105 /* flight-detect marker                    */

/* Time since the takeoff that field 105 marks, "M:SS" (v0.0.38). It belongs to
 * the status family because it is built from device state rather than from the
 * line map, and because AL_Units and AL_Dec mean nothing to it — but it is NOT
 * part of the status LINE: field 100 renders exactly the five pieces it always
 * has. Every card in the field carries a 100, and slipping a clock into it
 * would move the version marker that tells us which firmware is live. */
#define FS_HUD_FIELD_FLT_TIME   106 /* time since takeoff, "M:SS" / "--:--"    */
#define FS_HUD_FIELD_STATUS_MAX 106 /* last of the status family               */

#define FS_HUD_FIELD_NONE       255 /* empty slot: draws nothing               */

/* Fonts loaded on ENGO 3, id -> height px (fontList 0x50, verified on HW):
 * 0/24 1/24 6/32 2/38 7/48 3/64 4/75 5/82. All eight render. */
#define FS_HUD_MAX_FONT  7

/* Largest global offset the wearer can dial in, in pixels. Generous enough to
 * recentre the image, small enough that a typo cannot push the whole HUD off
 * the panel. */
#define FS_HUD_MAX_SHIFT  120

/* Units. 0 and 1 are the two unit SYSTEMS the HUD started with (they resolve to
 * whatever the field's quantity calls metric or imperial); 2..9 name ONE unit
 * outright, because "imperial" is no answer for a wingsuit pilot who wants a
 * vertical speed in m/s and a horizontal one in km/h at the same time.
 * A value that does not apply to the element's quantity — say km on a speed —
 * falls back to that quantity's metric default rather than rendering nonsense;
 * see FS_HudLayout_UnitConv. */
#define FS_HUD_UNITS_METRIC    0   /* the quantity's metric default   */
#define FS_HUD_UNITS_IMPERIAL  1   /* the quantity's imperial default */
#define FS_HUD_UNITS_KMH       2   /* speed                           */
#define FS_HUD_UNITS_MS        3   /* speed                           */
#define FS_HUD_UNITS_MPH       4   /* speed                           */
#define FS_HUD_UNITS_FTS       5   /* speed                           */
#define FS_HUD_UNITS_M         6   /* altitude, distance              */
#define FS_HUD_UNITS_FT        7   /* altitude, distance              */
#define FS_HUD_UNITS_KM        8   /* distance                        */
#define FS_HUD_UNITS_MI        9   /* distance                        */
#define FS_HUD_UNITS_MAX       9

/* The physical quantity an element's field measures — what decides which of the
 * units above apply to it. Lives here rather than in the renderer so the whole
 * unit table is host-testable (Tests/test_activelook.c). */
typedef enum
{
	FS_HUD_QTY_SPEED,     /* base unit m/s     */
	FS_HUD_QTY_DISTANCE,  /* base unit m       */
	FS_HUD_QTY_ALTITUDE,  /* base unit m       */
	FS_HUD_QTY_ANGLE,     /* base unit degrees */
	FS_HUD_QTY_NONE       /* unitless (glide ratio and its inverse) */
} FS_HudQuantity_t;

typedef struct
{
	double      multiplier;  /* base unit -> displayed unit */
	const char *suffix;      /* "" when there is nothing to append */
} FS_HudUnitConv_t;

typedef struct
{
	uint8_t field;     /* FS_HUD_FIELD_*                                    */
	int16_t x;         /* panel X of the anchor, 0..303 (mirrored, see top) */
	int16_t y;         /* panel Y of the anchor, 0..255 (top edge of glyphs)*/
	uint8_t font;      /* 0..FS_HUD_MAX_FONT                                */
	uint8_t units;     /* FS_HUD_UNITS_*                                    */
	int8_t  decimals;  /* decimal places, 0..3                              */
	uint8_t show_units;/* 0 = bare value, 1 = value + unit suffix           */
} FS_HudElement_t;

/* show_units draws the element's unit beside the value, so a speed reads
 * "148 km/h" instead of "148". Since v0.0.19 the unit is its OWN txt in font 0
 * (24 px) rather than part of the value's string — see FS_HudLayout_UnitDraw at
 * the bottom of this file. DEFAULT 0 everywhere, deliberately: the layout in
 * Docs/HUD_LAYOUT.md was approved on hardware with bare numbers, and how much
 * room even a 24 px suffix takes beside a 75 px value is an estimate until
 * someone looks at it through the glasses — an untouched device must keep
 * looking exactly as it does today. The suffix comes from
 * FS_HudLayout_UnitConv below, which is empty for
 * the unitless fields (glide ratio, inverse glide ratio) and never consulted for
 * the status family, so the flag is a silent no-op there rather than a special
 * case anyone has to remember — with one deliberate exception: on the split
 * status pieces 101..103 the flag turns on the PREFIX ("A:", "F:", "N:"), which
 * is the only thing distinguishing two battery percentages side by side. */

typedef struct
{
	FS_HudElement_t el[FS_HUD_MAX_ELEMENTS];
	uint8_t         count;
	int16_t         shift_x;  /* global offset, VIEWER right is positive */
	int16_t         shift_y;  /* global offset, up is positive           */
} FS_HudLayout_t;

/* The layout tuned on hardware and approved pixel by pixel (see
 * Docs/HUD_LAYOUT.md): info line on top, HSpd and VSpd sharing a row, then GR
 * and Alt. Used whenever CONFIG.TXT carries no positioned layout, so the
 * glasses work out of the box with an untouched config. */
void FS_HudLayout_Default(FS_HudLayout_t *out);

/* Position, font and precision of built-in data slot `slot` (0..3), i.e. the
 * default layout without its info line. Used as the starting point for a
 * config-supplied element so a config that names fields but omits coordinates
 * still lands somewhere sensible. Slots at or beyond FS_HUD_DEFAULT_SLOTS get
 * the info line's own defaults for FS_HUD_FIELD_INFO and (0,0) otherwise. */
void FS_HudLayout_DefaultSlot(uint8_t slot, uint8_t field, FS_HudElement_t *out);

/* Clamp an element's members to the ranges this module accepts. Applied on
 * every element that comes in from CONFIG.TXT, so no downstream code has to
 * defend itself against a hand-edited file. */
void FS_HudLayout_ClampElement(FS_HudElement_t *el);

/* Clamp the whole layout: every element, plus the global offset. */
void FS_HudLayout_Clamp(FS_HudLayout_t *layout);

/* Where element `i` actually draws, once the global offset is applied.
 * Returns 1 and fills *out_x / *out_y when the element should be drawn;
 * returns 0 when the slot is empty (FS_HUD_FIELD_NONE) or `i` is out of range.
 * The result is clamped to the panel, so an offset can never push an element
 * to a coordinate the glasses would reject. */
int FS_HudLayout_Place(const FS_HudLayout_t *layout, uint8_t i,
                       int16_t *out_x, int16_t *out_y);

/* True if the field needs a 3D GPS fix to mean anything. Barometric altitude
 * and the status family (100..105) do not; everything else does. NB the
 * satellite count is precisely the reading a wearer wants BEFORE the fix
 * arrives, so it reads 0, never "----". */
int FS_HudLayout_FieldNeedsFix(uint8_t field);

/* True for the status line and the five pieces it was split into (100..105):
 * the fields the renderer builds from device state rather than from the line
 * map, and for which AL_Units / AL_Dec are meaningless. */
int FS_HudLayout_FieldIsStatus(uint8_t field);

/* Multiplier and suffix for `qty` displayed in `units` (an FS_HUD_UNITS_*
 * value, already clamped). A unit that does not apply to the quantity — km on
 * a speed, m/s on an altitude — resolves to the quantity's METRIC default, so
 * a hand-edited card cannot make the panel lie about which unit it is showing.
 * Angles are always "deg"; the unitless quantities always have an empty
 * suffix, which is what makes AL_Unit_Show a silent no-op there. */
FS_HudUnitConv_t FS_HudLayout_UnitConv(FS_HudQuantity_t qty, uint8_t units);

/* --------------------------------------------------------------------------
   The unit suffix as its own draw (v0.0.19)

   Until v0.0.18 AL_Unit_Show appended the suffix inside the value's own
   snprintf, so "148 km/h" was drawn entirely in the value's font — at 64 or
   75 px the unit alone is wider than a third of the panel. It is a separate
   txt now, in the SMALLEST loaded font (id 0, 24 px, the one the status line
   uses), bottom-aligned with the value.

   Geometry, both consequences of the panel facts at the top of this file:
     - glyphs hang DOWN from the anchor, so sharing a BASELINE means lowering
       the unit's anchor by the difference in font heights;
     - X is mirrored and a string grows toward SMALLER x, so the unit's anchor
       sits one gap PAST the end of the value: x_unit = x_val - w_val - gap.
   -------------------------------------------------------------------------- */

/* The unit is always drawn in font 0 — 24 px, the smallest loaded on ENGO 3.
 * Deliberately not a CONFIG.TXT key: the point of the feature is that the unit
 * stays out of the value's way, and a configurable unit font is one more thing
 * a card can get wrong for no gain anybody asked for. */
#define FS_HUD_UNIT_FONT  0

/* Blank columns between the value's reserved box and the unit's first glyph
 * cell. ENGO glyph cells are OPAQUE — a cell that lands on a digit erases part
 * of it — so this gap is what keeps a rounding error in the width estimate
 * below from eating a digit. */
#define FS_HUD_UNIT_GAP   5

typedef struct
{
	int16_t     x;
	int16_t     y;
	uint8_t     font;
	const char *text;   /* the suffix, e.g. "km/h"; never empty when returned */
} FS_HudUnitDraw_t;

/* Height in px of font `font`, from fontList (0x50) read off ENGO 3 hardware:
 * 0/24 1/24 6/32 2/38 7/48 3/64 4/75 5/82. MEASURED — the glasses report it. */
uint8_t FS_HudLayout_FontHeight(uint8_t font);

/* UPPER-BOUND horizontal advance of one character of `font`, in px.
 *
 * ESTIMATED, NOT MEASURED — see the table in hud_layout.c for how, and for the
 * bench experiment that would replace these numbers with real ones. The
 * protocol cannot help: fontList returns id and height only, there is no
 * text-extent command, and a font cannot be read back out of the glasses. */
uint8_t FS_HudLayout_FontAdvance(uint8_t font);

/* How many character cells the value is allowed to occupy, for the purpose of
 * placing the unit beside it. A RESERVATION, fixed by the element's quantity,
 * unit and precision — deliberately NOT the length of the live string, because
 * a unit whose position tracked the reading would slide left and right every
 * time the value crossed 100, which reads as a fault. */
uint8_t FS_HudLayout_ValueChars(FS_HudQuantity_t qty, uint8_t units,
                                int8_t decimals);

/* Where the unit of element `el` is drawn, given where its value was placed
 * (val_x/val_y as returned by FS_HudLayout_Place, and el->font).
 *
 * Returns 1 and fills *out when a second txt must be emitted; returns 0 — and
 * the renderer emits nothing extra, exactly as v0.0.18 does — when the element
 * draws no unit at all: AL_Unit_Show off, a unitless quantity (glide ratio and
 * its inverse), or one of the status family, where the flag means the prefix
 * instead. The result is clamped to the panel. */
int FS_HudLayout_UnitDraw(const FS_HudElement_t *el, FS_HudQuantity_t qty,
                          int16_t val_x, int16_t val_y, FS_HudUnitDraw_t *out);

#endif /* HUD_LAYOUT_H_ */
