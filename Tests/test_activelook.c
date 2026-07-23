// tests/test_activelook.c — host unit tests for the pure ActiveLook protocol helpers.
// Build: gcc -Wall -Wextra -std=c99 -o test_activelook test_activelook.c ../FlySight/activelook_proto.c
#include <stdio.h>
#include <string.h>
#include "../FlySight/activelook_proto.h"

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

int main(void)
{
    test_build_clear();
    test_build_with_data();
    test_build_overflow();
    test_flow_gate();
    test_build_text();
    test_battery_pct();
    printf("activelook: %d/%d checks passed\n", g_total - g_fail, g_total);
    return g_fail ? 1 : 0;
}
