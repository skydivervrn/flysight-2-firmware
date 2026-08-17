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
	CHECK(l.shift_x == FS_HUD_MAX_SHIFT);
	CHECK(l.shift_y == -FS_HUD_MAX_SHIFT);

	/* 7. DefaultSlot seeds a config element that names a field but no
	 *    coordinates: slot N inherits row N's position and font. */
	FS_HudElement_t el;
	FS_HudLayout_DefaultSlot(0, FS_HUD_FIELD_HEADING, &el);
	CHECK(el.field == FS_HUD_FIELD_HEADING);
	CHECK(el.x == 268 && el.y == 208 && el.font == 3);
	FS_HudLayout_DefaultSlot(3, FS_HUD_FIELD_GPS_ALT, &el);
	CHECK(el.field == FS_HUD_FIELD_GPS_ALT);
	CHECK(el.x == 250 && el.y == 73 && el.font == 4);

	/* The status line keeps its own small-font defaults wherever it lands. */
	FS_HudLayout_DefaultSlot(2, FS_HUD_FIELD_INFO, &el);
	CHECK(el.x == 296 && el.y == 232 && el.font == 0);

	/* Past the built-in rows there is no default position to inherit. */
	FS_HudLayout_DefaultSlot(FS_HUD_DEFAULT_SLOTS, FS_HUD_FIELD_DIVE, &el);
	CHECK(el.field == FS_HUD_FIELD_DIVE);
	CHECK(el.x == 0 && el.y == 0);

	/* 8. Only barometric altitude and the status line survive without a fix. */
	CHECK(FS_HudLayout_FieldNeedsFix(FS_HUD_FIELD_BARO_ALT) == 0);
	CHECK(FS_HudLayout_FieldNeedsFix(FS_HUD_FIELD_INFO) == 0);
	CHECK(FS_HudLayout_FieldNeedsFix(FS_HUD_FIELD_NONE) == 0);
	CHECK(FS_HudLayout_FieldNeedsFix(FS_HUD_FIELD_HSPEED) == 1);
	CHECK(FS_HudLayout_FieldNeedsFix(FS_HUD_FIELD_GPS_ALT) == 1);

	printf("%d checks, %d failures\n", g_checks, g_fail);
	return g_fail ? 1 : 0;
}
