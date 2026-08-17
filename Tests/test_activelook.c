// tests/test_activelook.c — host unit tests for the pure ActiveLook protocol helpers.
// Build: gcc -Wall -Wextra -std=c99 -o test_activelook test_activelook.c ../FlySight/activelook_proto.c
#include <stdio.h>
#include <string.h>
#include "../FlySight/activelook_proto.h"
#include "../FlySight/hud_layout.h"

static int g_fail = 0;
static int g_total = 0;
#define CHECK(cond) do { g_total++; if(!(cond)){ g_fail++; \
    printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);} } while(0)

static void test_build_clear(void)
{
    uint8_t out[16];
    size_t n = AL_BuildFrame(out, sizeof(out), AL_CMD_CLEAR, NULL, 0);
    // Expect: FF 01 00 05 AA  (total length 5, in the length field)
    CHECK(n == 5);
    CHECK(out[0] == 0xFF);
    CHECK(out[1] == 0x01);
    CHECK(out[2] == 0x00);     // format: 1-byte len, no QueryID
    CHECK(out[3] == 0x05);     // length = whole frame
    CHECK(out[4] == 0xAA);
}

static void test_build_with_data(void)
{
    uint8_t out[16];
    uint8_t data[2] = { 0xDE, 0xAD };
    size_t n = AL_BuildFrame(out, sizeof(out), AL_CMD_TXT, data, 2);
    // FF 37 00 07 DE AD AA  -> total length 7
    CHECK(n == 7);
    CHECK(out[0] == 0xFF);
    CHECK(out[1] == 0x37);
    CHECK(out[2] == 0x00);
    CHECK(out[3] == 0x07);     // 2 data + 5 overhead
    CHECK(out[4] == 0xDE);
    CHECK(out[5] == 0xAD);
    CHECK(out[6] == 0xAA);
}

static void test_build_overflow(void)
{
    uint8_t out[4];            // too small for any frame
    CHECK(AL_BuildFrame(out, sizeof(out), AL_CMD_CLEAR, NULL, 0) == 0);

    // datalen that would push total over the 1-byte length cap (255)
    static uint8_t big[300];
    uint8_t big_out[400];
    CHECK(AL_BuildFrame(big_out, sizeof(big_out), AL_CMD_TXT, big, 251) == 0); // 251+5=256 > 255
}

static void test_flow_gate(void)
{
    CHECK(AL_FlowCanSend(AL_FLOW_OK,   true)  == true);
    CHECK(AL_FlowCanSend(AL_FLOW_OK,   false) == false); // link down
    CHECK(AL_FlowCanSend(AL_FLOW_STOP, true)  == false); // buffer >=75%
    CHECK(AL_FlowCanSend(AL_FLOW_ERR_OVERFLOW, true) == false);
    CHECK(AL_FlowCanSend(AL_FLOW_ERR_NOCFG,    true) == false);
    CHECK(AL_FlowCanSend(0x00,          true) == false); // unknown/initial
}

static void test_build_text(void)
{
    uint8_t out[40];
    // x=261 (0x0105), y=213 (0x00D5), rot=4, font=1, color=15, "Hi"
    size_t n = AL_BuildText(out, sizeof(out), 261, 213, 4, 1, 15, "Hi");
    // FF 37 00 len 01 05 00 D5 04 01 0F 'H' 'i' 00 AA
    // payload = 7 + 2(str) + 1(nul) = 10 ; total = 10 + 5 = 15
    CHECK(n == 15);
    CHECK(out[0] == 0xFF);
    CHECK(out[1] == 0x37);      // txt
    CHECK(out[2] == 0x00);
    CHECK(out[3] == 15);        // total length
    CHECK(out[4] == 0x01);      // x hi
    CHECK(out[5] == 0x05);      // x lo  -> 261
    CHECK(out[6] == 0x00);      // y hi
    CHECK(out[7] == 0xD5);      // y lo  -> 213
    CHECK(out[8] == 4);         // rotation
    CHECK(out[9] == 1);         // font
    CHECK(out[10] == 15);       // color
    CHECK(out[11] == 'H');
    CHECK(out[12] == 'i');
    CHECK(out[13] == 0x00);     // NUL terminator (required by ActiveLook)
    CHECK(out[14] == 0xAA);

    // empty string still gets a NUL: payload = 7 + 0 + 1 = 8 ; total = 13
    size_t e = AL_BuildText(out, sizeof(out), 0, 0, 4, 1, 15, "");
    CHECK(e == 13);
    CHECK(out[11] == 0x00);     // NUL
    CHECK(out[12] == 0xAA);

    // NULL string == empty
    CHECK(AL_BuildText(out, sizeof(out), 0, 0, 4, 1, 15, NULL) == 13);

    // overflow: tiny buffer
    uint8_t small[8];
    CHECK(AL_BuildText(small, sizeof(small), 0, 0, 4, 1, 15, "toolong") == 0);
}

static void test_battery_pct(void)
{
    CHECK(AL_BatteryPct(4200) == 100);
    CHECK(AL_BatteryPct(4500) == 100);  // clamp high
    CHECK(AL_BatteryPct(3300) == 0);
    CHECK(AL_BatteryPct(3000) == 0);    // clamp low
    uint8_t mid = AL_BatteryPct(3750);  // midpoint -> ~50
    CHECK(mid >= 49 && mid <= 51);
}

/* --------------------------------------------------------------------------
   The unit table (FS_HudLayout_UnitConv). It lived inside activelook_mode0.c
   until v0.0.18, where nothing could reach it: that file pulls in the HAL, the
   GNSS driver and FatFs. It is pure now, and this is where it gets checked.
   -------------------------------------------------------------------------- */

#define SUFFIX_IS(c, s)  (strcmp((c).suffix, (s)) == 0)

/* v0.0.17's AL_GetUnitConversion, copied verbatim from the commit before this
 * one (aad639f), so "byte-identical output for AL_Units 0 and 1" is something
 * this test MEASURES rather than something a commit message claims. */
typedef struct { double multiplier; const char *suffix; } LegacyConv_t;

static LegacyConv_t legacy_conv(FS_HudQuantity_t type, int system)
{
    LegacyConv_t info = { 1.0, "" };
    switch (type) {
        case FS_HUD_QTY_SPEED:
            if (system == 0) { info.multiplier = 3.6;      info.suffix = "km/h"; }
            else             { info.multiplier = 2.23694;  info.suffix = "mph";  }
            break;
        case FS_HUD_QTY_DISTANCE:
            if (system == 0) { info.multiplier = 0.001;       info.suffix = "km"; }
            else             { info.multiplier = 0.000621371; info.suffix = "mi"; }
            break;
        case FS_HUD_QTY_ALTITUDE:
            if (system == 0) {                                info.suffix = "m";  }
            else             { info.multiplier = 3.28084;     info.suffix = "ft"; }
            break;
        case FS_HUD_QTY_ANGLE:
            info.suffix = "deg";
            break;
        case FS_HUD_QTY_NONE:
            break;
    }
    return info;
}

/* Render an element exactly the way activelook_mode0.c does, so the comparison
 * below is over the STRING that reaches the glasses, not over a double. */
static void render(char *out, size_t n, double base, double mult,
                   const char *suffix, int decimals, int show_units)
{
    if (show_units && suffix[0] != '\0')
        snprintf(out, n, "%.*f %s", decimals, base * mult, suffix);
    else
        snprintf(out, n, "%.*f", decimals, base * mult);
}

static void test_unit_table_legacy(void)
{
    /* Values a wingsuit actually produces, plus the edges that catch a rounding
     * difference: 0, a negative climb rate, and a speed that lands on .5 at
     * every precision. */
    static const double samples[] = {
        0.0, 0.5, 1.0, 2.5, -2.5, 5.0, 12.3456789, 41.0, 55.5,
        83.3333333, 100.0, -100.0, 1234.5, 3000.0, 4321.98765, 99999.0
    };
    static const FS_HudQuantity_t qtys[] = {
        FS_HUD_QTY_SPEED, FS_HUD_QTY_DISTANCE, FS_HUD_QTY_ALTITUDE,
        FS_HUD_QTY_ANGLE, FS_HUD_QTY_NONE
    };

    for (unsigned q = 0; q < sizeof(qtys)/sizeof(qtys[0]); q++) {
        for (int system = 0; system <= 1; system++) {
            FS_HudUnitConv_t now = FS_HudLayout_UnitConv(qtys[q], (uint8_t)system);
            LegacyConv_t     old = legacy_conv(qtys[q], system);

            /* Same multiplier bit for bit, and the same suffix string. */
            CHECK(now.multiplier == old.multiplier);
            CHECK(strcmp(now.suffix, old.suffix) == 0);

            for (unsigned s = 0; s < sizeof(samples)/sizeof(samples[0]); s++) {
                for (int dec = 0; dec <= 3; dec++) {
                    for (int show = 0; show <= 1; show++) {
                        char a[48], b[48];
                        render(a, sizeof(a), samples[s], now.multiplier, now.suffix, dec, show);
                        render(b, sizeof(b), samples[s], old.multiplier, old.suffix, dec, show);
                        CHECK(strcmp(a, b) == 0);
                    }
                }
            }
        }
    }
}

static void test_unit_table_named(void)
{
    FS_HudUnitConv_t c;

    /* Speeds, base m/s. 41 m/s is 148 km/h, the number in the layout docs. */
    c = FS_HudLayout_UnitConv(FS_HUD_QTY_SPEED, FS_HUD_UNITS_KMH);
    CHECK(c.multiplier == 3.6 && SUFFIX_IS(c, "km/h"));
    CHECK((int)(41.0 * c.multiplier + 0.5) == 148);
    c = FS_HudLayout_UnitConv(FS_HUD_QTY_SPEED, FS_HUD_UNITS_MS);
    CHECK(c.multiplier == 1.0 && SUFFIX_IS(c, "m/s"));
    c = FS_HudLayout_UnitConv(FS_HUD_QTY_SPEED, FS_HUD_UNITS_MPH);
    CHECK(c.multiplier == 2.23694 && SUFFIX_IS(c, "mph"));
    c = FS_HudLayout_UnitConv(FS_HUD_QTY_SPEED, FS_HUD_UNITS_FTS);
    CHECK(c.multiplier == 3.28084 && SUFFIX_IS(c, "ft/s"));

    /* `imperial` and the explicit `mph` must be the SAME entry, or a card
     * rewritten from 1 to 4 by the app would change the number on the panel. */
    CHECK(FS_HudLayout_UnitConv(FS_HUD_QTY_SPEED, FS_HUD_UNITS_IMPERIAL).multiplier
          == FS_HudLayout_UnitConv(FS_HUD_QTY_SPEED, FS_HUD_UNITS_MPH).multiplier);
    CHECK(FS_HudLayout_UnitConv(FS_HUD_QTY_SPEED, FS_HUD_UNITS_METRIC).multiplier
          == FS_HudLayout_UnitConv(FS_HUD_QTY_SPEED, FS_HUD_UNITS_KMH).multiplier);

    /* Altitudes, base m. */
    c = FS_HudLayout_UnitConv(FS_HUD_QTY_ALTITUDE, FS_HUD_UNITS_M);
    CHECK(c.multiplier == 1.0 && SUFFIX_IS(c, "m"));
    c = FS_HudLayout_UnitConv(FS_HUD_QTY_ALTITUDE, FS_HUD_UNITS_FT);
    CHECK(c.multiplier == 3.28084 && SUFFIX_IS(c, "ft"));
    CHECK(FS_HudLayout_UnitConv(FS_HUD_QTY_ALTITUDE, FS_HUD_UNITS_IMPERIAL).multiplier
          == FS_HudLayout_UnitConv(FS_HUD_QTY_ALTITUDE, FS_HUD_UNITS_FT).multiplier);
    /* 1200 m is 3937 ft — the altitude the wearer asked to see in feet. */
    CHECK((int)(1200.0 * c.multiplier) == 3937);

    /* Distances, base m — the one quantity where all four apply. */
    c = FS_HudLayout_UnitConv(FS_HUD_QTY_DISTANCE, FS_HUD_UNITS_M);
    CHECK(c.multiplier == 1.0 && SUFFIX_IS(c, "m"));
    c = FS_HudLayout_UnitConv(FS_HUD_QTY_DISTANCE, FS_HUD_UNITS_FT);
    CHECK(c.multiplier == 3.28084 && SUFFIX_IS(c, "ft"));
    c = FS_HudLayout_UnitConv(FS_HUD_QTY_DISTANCE, FS_HUD_UNITS_KM);
    CHECK(c.multiplier == 0.001 && SUFFIX_IS(c, "km"));
    c = FS_HudLayout_UnitConv(FS_HUD_QTY_DISTANCE, FS_HUD_UNITS_MI);
    CHECK(c.multiplier == 0.000621371 && SUFFIX_IS(c, "mi"));

    /* Angles are degrees whatever the file asks for, and the unitless
     * quantities stay bare — which is what makes AL_Unit_Show a no-op there
     * instead of a special case in the renderer. */
    for (uint8_t u = 0; u <= FS_HUD_UNITS_MAX; u++) {
        c = FS_HudLayout_UnitConv(FS_HUD_QTY_ANGLE, u);
        CHECK(c.multiplier == 1.0 && SUFFIX_IS(c, "deg"));
        c = FS_HudLayout_UnitConv(FS_HUD_QTY_NONE, u);
        CHECK(c.multiplier == 1.0 && SUFFIX_IS(c, ""));
    }
}

static void test_unit_table_mismatch(void)
{
    /* A unit that does not belong to the quantity falls back to that quantity's
     * METRIC default rather than rendering a distance as a speed. No app writes
     * these; a hand-edited card can. */
    static const uint8_t not_speed[]    = { 6, 7, 8, 9 };
    static const uint8_t not_altitude[] = { 2, 3, 4, 5, 8, 9 };
    static const uint8_t not_distance[] = { 2, 3, 4, 5 };
    FS_HudUnitConv_t c;
    unsigned i;

    for (i = 0; i < sizeof(not_speed); i++) {
        c = FS_HudLayout_UnitConv(FS_HUD_QTY_SPEED, not_speed[i]);
        CHECK(c.multiplier == 3.6 && SUFFIX_IS(c, "km/h"));
    }
    for (i = 0; i < sizeof(not_altitude); i++) {
        c = FS_HudLayout_UnitConv(FS_HUD_QTY_ALTITUDE, not_altitude[i]);
        CHECK(c.multiplier == 1.0 && SUFFIX_IS(c, "m"));
    }
    for (i = 0; i < sizeof(not_distance); i++) {
        c = FS_HudLayout_UnitConv(FS_HUD_QTY_DISTANCE, not_distance[i]);
        CHECK(c.multiplier == 0.001 && SUFFIX_IS(c, "km"));
    }

    /* Anything past the table is metric too. FS_HudLayout_ClampElement already
     * folds these to 0 before they get here; the table does not rely on it. */
    static const uint8_t junk[] = { 10, 42, 255 };
    for (i = 0; i < sizeof(junk); i++) {
        CHECK(SUFFIX_IS(FS_HudLayout_UnitConv(FS_HUD_QTY_SPEED,    junk[i]), "km/h"));
        CHECK(SUFFIX_IS(FS_HudLayout_UnitConv(FS_HUD_QTY_ALTITUDE, junk[i]), "m"));
        CHECK(SUFFIX_IS(FS_HudLayout_UnitConv(FS_HUD_QTY_DISTANCE, junk[i]), "km"));
    }
}

int main(void)
{
    test_build_clear();
    test_build_with_data();
    test_build_overflow();
    test_flow_gate();
    test_build_text();
    test_battery_pct();
    test_unit_table_legacy();
    test_unit_table_named();
    test_unit_table_mismatch();
    printf("activelook: %d/%d checks passed\n", g_total - g_fail, g_total);
    return g_fail ? 1 : 0;
}
