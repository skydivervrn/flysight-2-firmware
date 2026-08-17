/* Host tests for FlySight/hud_layout.c — pure geometry, no HAL needed. */
#include <stdio.h>
#include <string.h>
#include "hud_layout.h"

static int g_checks = 0, g_fail = 0;
#define CHECK(cond) do { \
	g_checks++; \
	if (!(cond)) { g_fail++; printf("FAIL line %d: %s\n", __LINE__, #cond); } \
} while (0)

int main(void)
{
	FS_HudLayout_t l;
	int16_t x, y;

	/* 1. The built-in layout is the one approved on hardware: status line on
	 *    top, HSpd and VSpd sharing a row, then GR and barometric Alt. */
	FS_HudLayout_Default(&l);
	CHECK(l.count == 5);
	CHECK(l.shift_x == 0 && l.shift_y == 0);
	CHECK(l.el[0].field == FS_HUD_FIELD_INFO);
	CHECK(l.el[0].x == 296 && l.el[0].y == 232 && l.el[0].font == 0);
	CHECK(l.el[1].field == FS_HUD_FIELD_HSPEED);
	CHECK(l.el[1].x == 268 && l.el[1].y == 208 && l.el[1].font == 3);
	CHECK(l.el[2].field == FS_HUD_FIELD_VSPEED);
	CHECK(l.el[2].x == 134 && l.el[2].y == 208 && l.el[2].font == 3);
	CHECK(l.el[3].field == FS_HUD_FIELD_GLIDE);
	CHECK(l.el[3].x == 250 && l.el[3].y == 147 && l.el[3].decimals == 2);
	CHECK(l.el[4].field == FS_HUD_FIELD_BARO_ALT);
	CHECK(l.el[4].x == 250 && l.el[4].y == 73 && l.el[4].decimals == 0);

	/* 1b. Unit suffixes are OFF in the built-in layout, all five elements —
	 *     the positions above were approved on hardware with bare numbers, so
	 *     an untouched device must not suddenly grow "km/h" into its neighbour. */
	for (uint8_t i = 0; i < l.count; i++)
		CHECK(l.el[i].show_units == 0);

	/* 2. With no offset, an element draws exactly where it says. */
	CHECK(FS_HudLayout_Place(&l, 1, &x, &y) == 1);
	CHECK(x == 268 && y == 208);

	/* 3. The offset is in VIEWER terms and X is mirrored on the panel, so
	 *    "move right" must DECREASE the panel X. Y is not flipped. */
	l.shift_x = 20;
	l.shift_y = 10;
	CHECK(FS_HudLayout_Place(&l, 1, &x, &y) == 1);
	CHECK(x == 248 && y == 218);

	l.shift_x = -20;
	l.shift_y = -10;
	CHECK(FS_HudLayout_Place(&l, 1, &x, &y) == 1);
	CHECK(x == 288 && y == 198);

	/* 4. A large offset can never produce a coordinate off the panel. */
	FS_HudLayout_Default(&l);
	l.shift_x = FS_HUD_MAX_SHIFT;
	l.shift_y = FS_HUD_MAX_SHIFT;
	for (uint8_t i = 0; i < l.count; i++)
	{
		CHECK(FS_HudLayout_Place(&l, i, &x, &y) == 1);
		CHECK(x >= 0 && x < FS_HUD_PANEL_W);
		CHECK(y >= 0 && y < FS_HUD_PANEL_H);
	}
	l.shift_x = -FS_HUD_MAX_SHIFT;
	l.shift_y = -FS_HUD_MAX_SHIFT;
	for (uint8_t i = 0; i < l.count; i++)
	{
		CHECK(FS_HudLayout_Place(&l, i, &x, &y) == 1);
		CHECK(x >= 0 && x < FS_HUD_PANEL_W);
		CHECK(y >= 0 && y < FS_HUD_PANEL_H);
	}

	/* 5. An empty slot draws nothing, and so does an index past the end. */
	FS_HudLayout_Default(&l);
	l.el[2].field = FS_HUD_FIELD_NONE;
	CHECK(FS_HudLayout_Place(&l, 2, &x, &y) == 0);
	CHECK(FS_HudLayout_Place(&l, l.count, &x, &y) == 0);
	CHECK(FS_HudLayout_Place(&l, FS_HUD_MAX_ELEMENTS, &x, &y) == 0);

	/* 6. Clamping folds a hand-edited file into range instead of trusting it. */
	FS_HudLayout_Default(&l);
	l.el[0].x = 5000;
	l.el[0].y = -40;
	l.el[0].font = 99;
	l.el[0].decimals = 9;
	l.el[1].decimals = -3;
	l.el[1].units = 77;
	l.el[1].show_units = 200;
	l.el[2].show_units = 1;
	l.shift_x = 9000;
	l.shift_y = -9000;
	l.count = FS_HUD_MAX_ELEMENTS + 3;
	FS_HudLayout_Clamp(&l);
	CHECK(l.count == FS_HUD_MAX_ELEMENTS);
	CHECK(l.el[0].x == FS_HUD_PANEL_W - 1);
	CHECK(l.el[0].y == 0);
	CHECK(l.el[0].font == FS_HUD_MAX_FONT);
	CHECK(l.el[0].decimals == 3);
	CHECK(l.el[1].decimals == 0);
	CHECK(l.el[1].units == FS_HUD_UNITS_METRIC);
	/* show_units is a flag: any non-zero becomes 1, and a real 1 survives. */
	CHECK(l.el[1].show_units == 1);
	CHECK(l.el[2].show_units == 1);
	CHECK(l.el[0].show_units == 0);
	CHECK(l.shift_x == FS_HUD_MAX_SHIFT);
	CHECK(l.shift_y == -FS_HUD_MAX_SHIFT);

	/* 7. DefaultSlot seeds a config element that names a field but no
	 *    coordinates: slot N inherits row N's position and font. */
	/* Seeded with an element that already asks for suffixes, because that is
	 * exactly the situation in config.c: config.al_layout.el[] is reused from
	 * one FS_Config_Read to the next, so every branch of DefaultSlot must write
	 * show_units rather than let a previous file's 1 leak into a new element. */
	FS_HudElement_t el;
	memset(&el, 0xFF, sizeof(el));
	FS_HudLayout_DefaultSlot(0, FS_HUD_FIELD_HEADING, &el);
	CHECK(el.field == FS_HUD_FIELD_HEADING);
	CHECK(el.x == 268 && el.y == 208 && el.font == 3);
	CHECK(el.show_units == 0);
	el.show_units = 1;
	FS_HudLayout_DefaultSlot(3, FS_HUD_FIELD_GPS_ALT, &el);
	CHECK(el.field == FS_HUD_FIELD_GPS_ALT);
	CHECK(el.x == 250 && el.y == 73 && el.font == 4);
	CHECK(el.show_units == 0);

	/* The status line keeps its own small-font defaults wherever it lands. */
	el.show_units = 1;
	FS_HudLayout_DefaultSlot(2, FS_HUD_FIELD_INFO, &el);
	CHECK(el.x == 296 && el.y == 232 && el.font == 0);
	CHECK(el.show_units == 0);

	/* Past the built-in rows there is no default position to inherit. */
	el.show_units = 1;
	FS_HudLayout_DefaultSlot(FS_HUD_DEFAULT_SLOTS, FS_HUD_FIELD_DIVE, &el);
	CHECK(el.field == FS_HUD_FIELD_DIVE);
	CHECK(el.x == 0 && el.y == 0);
	CHECK(el.show_units == 0);

	/* 8. Only barometric altitude and the status family survive without a fix. */
	CHECK(FS_HudLayout_FieldNeedsFix(FS_HUD_FIELD_BARO_ALT) == 0);
	CHECK(FS_HudLayout_FieldNeedsFix(FS_HUD_FIELD_INFO) == 0);
	CHECK(FS_HudLayout_FieldNeedsFix(FS_HUD_FIELD_NONE) == 0);
	CHECK(FS_HudLayout_FieldNeedsFix(FS_HUD_FIELD_HSPEED) == 1);
	CHECK(FS_HudLayout_FieldNeedsFix(FS_HUD_FIELD_GPS_ALT) == 1);

	/* 9. The status line taken apart (v0.0.18): 101..105 are status fields, and
	 *    none of them waits for a fix — the satellite count in particular is the
	 *    reading a wearer wants BEFORE the fix arrives, so it must never be
	 *    treated as GPS-derived and blanked to "----". */
	for (uint8_t f = FS_HUD_FIELD_INFO; f <= FS_HUD_FIELD_STATUS_MAX; f++)
	{
		CHECK(FS_HudLayout_FieldIsStatus(f) == 1);
		CHECK(FS_HudLayout_FieldNeedsFix(f) == 0);
	}
	CHECK(FS_HUD_FIELD_STATUS_MAX == 105);
	/* Nothing outside 100..105 joins the family by accident — 14 is the
	 * barometric altitude, an ordinary element that happens to need no fix. */
	CHECK(FS_HudLayout_FieldIsStatus(99) == 0);
	CHECK(FS_HudLayout_FieldIsStatus(106) == 0);
	CHECK(FS_HudLayout_FieldIsStatus(FS_HUD_FIELD_BARO_ALT) == 0);
	CHECK(FS_HudLayout_FieldIsStatus(FS_HUD_FIELD_NONE) == 0);

	/* 9b. Each piece lands on the status line's own small-font defaults, not on
	 *     the 64 px defaults of whatever data slot it happens to occupy — a
	 *     battery percentage at 64 px would fill the panel. */
	for (uint8_t f = FS_HUD_FIELD_AL_BATT; f <= FS_HUD_FIELD_STATUS_MAX; f++)
	{
		memset(&el, 0xFF, sizeof(el));
		FS_HudLayout_DefaultSlot(0, f, &el);
		CHECK(el.field == f);
		CHECK(el.x == 296 && el.y == 232 && el.font == 0);
		CHECK(el.show_units == 0);
		CHECK(el.units == FS_HUD_UNITS_METRIC && el.decimals == 0);
	}

	/* 10. AL_Units became a set of names, not a two-valued flag. Out of range
	 *     means METRIC, not the nearest end of the range: clamping 42 up to
	 *     `mi` would put a distance unit on a speed. These are the same
	 *     semantics config.c applies to the raw value before it ever reaches
	 *     the struct (the AL_Units branch of FS_Config_Read). */
	CHECK(FS_HUD_UNITS_MAX == 9);
	FS_HudLayout_Default(&l);
	for (uint8_t u = 0; u <= FS_HUD_UNITS_MAX; u++)
	{
		l.el[1].units = u;
		FS_HudLayout_ClampElement(&l.el[1]);
		CHECK(l.el[1].units == u);          /* every named unit survives */
	}
	l.el[1].units = FS_HUD_UNITS_MAX + 1;
	FS_HudLayout_ClampElement(&l.el[1]);
	CHECK(l.el[1].units == FS_HUD_UNITS_METRIC);
	l.el[1].units = 255;
	FS_HudLayout_ClampElement(&l.el[1]);
	CHECK(l.el[1].units == FS_HUD_UNITS_METRIC);

	/* 11. Backwards compatibility, the whole point of the release: a card that
	 *     exists in the field today has no AL_Units above 1, and the built-in
	 *     layout it falls back to must be untouched. */
	FS_HudLayout_Default(&l);
	CHECK(l.count == 5);
	for (uint8_t i = 0; i < l.count; i++)
	{
		CHECK(l.el[i].units == FS_HUD_UNITS_METRIC);
		CHECK(l.el[i].show_units == 0);
	}
	CHECK(l.el[0].field == FS_HUD_FIELD_INFO);   /* field 100, unchanged */
	CHECK(l.el[0].x == 296 && l.el[0].y == 232 && l.el[0].font == 0);

	printf("%d checks, %d failures\n", g_checks, g_fail);
	return g_fail ? 1 : 0;
}
