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
 * HUD element and an audio mode name the same quantity. 14 and 100 are HUD-only
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
#define FS_HUD_FIELD_INFO       100 /* the status line (battery/sats/version)  */
#define FS_HUD_FIELD_NONE       255 /* empty slot: draws nothing               */

/* Fonts loaded on ENGO 3, id -> height px (fontList 0x50, verified on HW):
 * 0/24 1/24 6/32 2/38 7/48 3/64 4/75 5/82. All eight render. */
#define FS_HUD_MAX_FONT  7

/* Largest global offset the wearer can dial in, in pixels. Generous enough to
 * recentre the image, small enough that a typo cannot push the whole HUD off
 * the panel. */
#define FS_HUD_MAX_SHIFT  120

/* Unit systems, matching FS_Config_UnitSystem_t (metric = 0). */
#define FS_HUD_UNITS_METRIC    0
#define FS_HUD_UNITS_IMPERIAL  1

typedef struct
{
	uint8_t field;     /* FS_HUD_FIELD_*                                    */
	int16_t x;         /* panel X of the anchor, 0..303 (mirrored, see top) */
	int16_t y;         /* panel Y of the anchor, 0..255 (top edge of glyphs)*/
	uint8_t font;      /* 0..FS_HUD_MAX_FONT                                */
	uint8_t units;     /* FS_HUD_UNITS_*                                    */
	int8_t  decimals;  /* decimal places, 0..3                              */
} FS_HudElement_t;

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
 * and the info line do not; everything else does. */
int FS_HudLayout_FieldNeedsFix(uint8_t field);

#endif /* HUD_LAYOUT_H_ */
