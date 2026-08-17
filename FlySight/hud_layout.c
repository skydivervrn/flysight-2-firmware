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

#include "hud_layout.h"

#include <string.h>

/* The info line: small font, top of the panel, anchored at the viewer's left
 * edge (x=296 with the ENGO 3 X mirror). Its decimals field is unused. */
#define INFO_X     296
#define INFO_Y     232
#define INFO_FONT  0

/* The four data slots, in the order the HUD has drawn them since v0.0.11.
 * Fonts: 3 = 64 px (the two speeds, side by side), 4 = 75 px (GR and Alt).
 * Trailing 0 = show_units off. These positions were approved on hardware with
 * bare numbers only; whether a suffix still fits between HSpd (x=268) and VSpd
 * (x=134) at 64 px has NOT been measured, so the default does not risk it. */
static const FS_HudElement_t s_defaultSlots[FS_HUD_DEFAULT_SLOTS] =
{
	{ FS_HUD_FIELD_HSPEED,   268, 208, 3, FS_HUD_UNITS_METRIC, 0, 0 },
	{ FS_HUD_FIELD_VSPEED,   134, 208, 3, FS_HUD_UNITS_METRIC, 0, 0 },
	{ FS_HUD_FIELD_GLIDE,    250, 147, 4, FS_HUD_UNITS_METRIC, 2, 0 },
	{ FS_HUD_FIELD_BARO_ALT, 250,  73, 4, FS_HUD_UNITS_METRIC, 0, 0 },
};

static int16_t clamp16(int16_t v, int16_t lo, int16_t hi)
{
	if (v < lo) return lo;
	if (v > hi) return hi;
	return v;
}

void FS_HudLayout_Default(FS_HudLayout_t *out)
{
	if (out == 0) return;

	memset(out, 0, sizeof(*out));

	out->el[0].field      = FS_HUD_FIELD_INFO;
	out->el[0].x          = INFO_X;
	out->el[0].y          = INFO_Y;
	out->el[0].font       = INFO_FONT;
	out->el[0].units      = FS_HUD_UNITS_METRIC;
	out->el[0].decimals   = 0;
	out->el[0].show_units = 0;

	for (uint8_t i = 0; i < FS_HUD_DEFAULT_SLOTS; ++i)
	{
		out->el[1 + i] = s_defaultSlots[i];
	}

	out->count   = 1 + FS_HUD_DEFAULT_SLOTS;
	out->shift_x = 0;
	out->shift_y = 0;
}

void FS_HudLayout_DefaultSlot(uint8_t slot, uint8_t field, FS_HudElement_t *out)
{
	if (out == 0) return;

	if (FS_HudLayout_FieldIsStatus(field))
	{
		/* The info line — and each of the five pieces it splits into — keeps
		 * its own defaults wherever it appears in the list: one line of small
		 * text, never one of the 64/75 px data rows. A battery percentage
		 * inheriting slot 0's 64 px font would be absurd. show_units stays off,
		 * so a piece that lands here without an AL_Unit_Show of its own draws
		 * bare, exactly as the whole line does today. */
		out->field      = field;
		out->x          = INFO_X;
		out->y          = INFO_Y;
		out->font       = INFO_FONT;
		out->units      = FS_HUD_UNITS_METRIC;
		out->decimals   = 0;
		out->show_units = 0;
		return;
	}

	if (slot < FS_HUD_DEFAULT_SLOTS)
	{
		*out       = s_defaultSlots[slot];
		out->field = field;
	}
	else
	{
		/* Past the built-in rows there is no sensible default position: the
		 * config has to say where it wants the element. */
		out->field      = field;
		out->x          = 0;
		out->y          = 0;
		out->font       = INFO_FONT;
		out->units      = FS_HUD_UNITS_METRIC;
		out->decimals   = 0;
		out->show_units = 0;
	}
}

void FS_HudLayout_ClampElement(FS_HudElement_t *el)
{
	if (el == 0) return;

	el->x = clamp16(el->x, 0, FS_HUD_PANEL_W - 1);
	el->y = clamp16(el->y, 0, FS_HUD_PANEL_H - 1);

	if (el->font > FS_HUD_MAX_FONT)  el->font  = FS_HUD_MAX_FONT;

	/* Units are a set of names, not a range: a number nobody has defined means
	 * metric, not "the highest one we do have" — clamping 42 up to `mi` would
	 * put a distance on a speed. */
	if (el->units > FS_HUD_UNITS_MAX) el->units = FS_HUD_UNITS_METRIC;

	if (el->decimals < 0) el->decimals = 0;
	if (el->decimals > 3) el->decimals = 3;

	/* A flag, not a range: anything the file offers other than 1 means off. */
	if (el->show_units != 0) el->show_units = 1;
}

void FS_HudLayout_Clamp(FS_HudLayout_t *layout)
{
	if (layout == 0) return;

	if (layout->count > FS_HUD_MAX_ELEMENTS)
		layout->count = FS_HUD_MAX_ELEMENTS;

	for (uint8_t i = 0; i < layout->count; ++i)
		FS_HudLayout_ClampElement(&layout->el[i]);

	layout->shift_x = clamp16(layout->shift_x, -FS_HUD_MAX_SHIFT, FS_HUD_MAX_SHIFT);
	layout->shift_y = clamp16(layout->shift_y, -FS_HUD_MAX_SHIFT, FS_HUD_MAX_SHIFT);
}

int FS_HudLayout_Place(const FS_HudLayout_t *layout, uint8_t i,
                       int16_t *out_x, int16_t *out_y)
{
	if (layout == 0 || out_x == 0 || out_y == 0) return 0;
	if (i >= layout->count || i >= FS_HUD_MAX_ELEMENTS) return 0;

	const FS_HudElement_t *el = &layout->el[i];
	if (el->field == FS_HUD_FIELD_NONE) return 0;

	/* The offset arrives in viewer terms; panel X runs the other way, so
	 * "move right" subtracts. Y needs no flip (larger y is higher, and so is
	 * a positive offset). */
	int32_t x = (int32_t)el->x - (int32_t)layout->shift_x;
	int32_t y = (int32_t)el->y + (int32_t)layout->shift_y;

	if (x < 0) x = 0;
	if (x > FS_HUD_PANEL_W - 1) x = FS_HUD_PANEL_W - 1;
	if (y < 0) y = 0;
	if (y > FS_HUD_PANEL_H - 1) y = FS_HUD_PANEL_H - 1;

	*out_x = (int16_t)x;
	*out_y = (int16_t)y;
	return 1;
}

int FS_HudLayout_FieldIsStatus(uint8_t field)
{
	return (field >= FS_HUD_FIELD_INFO && field <= FS_HUD_FIELD_STATUS_MAX);
}

int FS_HudLayout_FieldNeedsFix(uint8_t field)
{
	if (FS_HudLayout_FieldIsStatus(field)) return 0;

	switch (field)
	{
	case FS_HUD_FIELD_BARO_ALT:
	case FS_HUD_FIELD_NONE:
		return 0;
	default:
		return 1;
	}
}

/* Conversion constants. Kept BIT-IDENTICAL to the ones activelook_mode0.c has
 * used since v0.0.11 (M_PER_S_TO_KMH etc.), because a card in the field that
 * says `AL_Units: 0` or `1` must render the same number on 0.0.18 as it did on
 * 0.0.17 — a "more correct" mph factor here would be a silent change to
 * everyone's panel. HUD_FEEDBACK.md quotes mph as 2.236936; 2.23694 is the same
 * figure to the precision the firmware already ships, and using one constant for
 * both `imperial` and the explicit `mph` is what keeps those two values from
 * disagreeing in the third decimal. */
#define M_PER_S_TO_KMH   3.6
#define M_PER_S_TO_MPH   2.23694
#define M_PER_S_TO_FT_S  3.28084
#define METERS_TO_KM     0.001
#define METERS_TO_MILES  0.000621371
#define METERS_TO_FEET   3.28084

FS_HudUnitConv_t FS_HudLayout_UnitConv(FS_HudQuantity_t qty, uint8_t units)
{
	FS_HudUnitConv_t info = { 1.0, "" };

	switch (qty)
	{
	case FS_HUD_QTY_SPEED:      /* base m/s */
		switch (units)
		{
		case FS_HUD_UNITS_MS:  info.multiplier = 1.0;             info.suffix = "m/s";  break;
		case FS_HUD_UNITS_MPH:
		case FS_HUD_UNITS_IMPERIAL:
		                       info.multiplier = M_PER_S_TO_MPH;  info.suffix = "mph";  break;
		case FS_HUD_UNITS_FTS: info.multiplier = M_PER_S_TO_FT_S; info.suffix = "ft/s"; break;
		default:               /* metric, km/h, and anything that is not a
		                        * speed unit at all */
		                       info.multiplier = M_PER_S_TO_KMH;  info.suffix = "km/h"; break;
		}
		break;

	case FS_HUD_QTY_DISTANCE:   /* base m */
		switch (units)
		{
		case FS_HUD_UNITS_M:   info.multiplier = 1.0;             info.suffix = "m";    break;
		case FS_HUD_UNITS_FT:  info.multiplier = METERS_TO_FEET;  info.suffix = "ft";   break;
		case FS_HUD_UNITS_MI:
		case FS_HUD_UNITS_IMPERIAL:
		                       info.multiplier = METERS_TO_MILES; info.suffix = "mi";   break;
		default:               /* metric, km, and anything that is not a
		                        * distance unit at all */
		                       info.multiplier = METERS_TO_KM;    info.suffix = "km";   break;
		}
		break;

	case FS_HUD_QTY_ALTITUDE:   /* base m */
		switch (units)
		{
		case FS_HUD_UNITS_FT:
		case FS_HUD_UNITS_IMPERIAL:
		                       info.multiplier = METERS_TO_FEET;  info.suffix = "ft";   break;
		default:               /* metric, m, and anything that is not an
		                        * altitude unit at all */
		                       info.multiplier = 1.0;             info.suffix = "m";    break;
		}
		break;

	case FS_HUD_QTY_ANGLE:      /* degrees, whatever the file asks for */
		info.suffix = "deg";
		break;

	case FS_HUD_QTY_NONE:       /* unitless: no multiplier, no suffix */
		break;
	}

	return info;
}
