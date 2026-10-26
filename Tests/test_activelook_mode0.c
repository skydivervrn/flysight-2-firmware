/***************************************************************************
**  Host tests for the HUD renderer, FlySight/activelook_mode0.c.
**
**  WHY THIS EXISTS
**  The renderer was the one part of the HUD no host test could link: it pulls
**  in the HAL, the GNSS driver and the BLE client. Everything it reads is a
**  plain getter, though, so the same copy-next-to-mocks trick as
**  test_activelook_fsm works here: the real activelook_mode0.c is compiled
**  beside mocks of app_common.h (HAL_GetTick), activelook_client.h (the radio)
**  and log.h, and the pure modules it draws with (hud_layout, activelook_proto,
**  comp_corridor, nav, nav_arrow) are the real ones. The test then reads the
**  frame the way the glasses would — by decoding the packets.
**
**  What it pins down (review findings of 2026-10-09):
**   F1-a  a barometer that stops delivering must turn the altitude into "----",
**         with one "Baro silent" line, and back to a number with one
**         "Baro talking again" line — never a frozen height that looks live;
**   F1-b  the altitude zero point is the first sample after switch-on, taken
**         by the sample hook whether or not the glasses are connected — not
**         the pressure at the moment the glasses happened to link up;
**   F1-c  a tick must finish a half-sent frame before building another, so
**         the glasses never see HOLD, HOLD (a second hold inside an open one).
****************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "activelook_mode0.h"
#include "activelook_client.h"   /* mock */
#include "app_common.h"          /* mock */
#include "activelook_proto.h"
#include "config.h"
#include "baro.h"
#include "gnss.h"
#include "vbat.h"
#include "engo_bind.h"
#include "flight_detect.h"

/* ---------------- check harness ---------------- */
static int g_checks = 0, g_fail = 0;
#define CHECK(cond) do { \
	g_checks++; \
	if (!(cond)) { g_fail++; printf("FAIL line %d: %s\n", __LINE__, #cond); } \
} while (0)

/* ---------------- device-state mocks ---------------- */
static uint32_t         g_tick;
static FS_Config_Data_t g_cfg;
static FS_GNSS_Data_t   g_gnss;
static FS_VBAT_Data_t   g_vbat;
static FS_Baro_Data_t   g_baro;

uint32_t HAL_GetTick(void) { return g_tick; }
const FS_Config_Data_t *FS_Config_Get(void) { return &g_cfg; }
const FS_GNSS_Data_t   *FS_GNSS_GetData(void) { return &g_gnss; }
bool     FS_GNSS_IsStale(void) { return false; }
uint32_t FS_GNSS_MsSinceUpdate(void) { return 0; }
const FS_VBAT_Data_t   *FS_VBAT_GetData(void) { return &g_vbat; }
const FS_Baro_Data_t   *FS_Baro_GetData(void) { return &g_baro; }
void FS_EngoBind_LogLinkedOnce(void) { }
/* flight_detect.c is not linked: the renderer only asks two questions of it,
 * and "on the ground, no clock" is the state these tests want. */
bool FS_FlightDetect_InFlight(void) { return false; }
bool FS_FlightDetect_ElapsedSec(uint32_t *out) { (void)out; return false; }

/* ---------------- log mock ---------------- */
static char g_log[16][96];
static int  g_logN;

void FS_Log_WriteEventAsync(const char *format, ...)
{
	if (g_logN >= 16) return;
	va_list ap;
	va_start(ap, format);
	vsnprintf(g_log[g_logN++], sizeof(g_log[0]), format, ap);
	va_end(ap);
}

static int log_count(const char *prefix)
{
	int n = 0;
	for (int i = 0; i < g_logN; i++)
		if (strncmp(g_log[i], prefix, strlen(prefix)) == 0) n++;
	return n;
}

/* ---------------- radio mock ---------------- */
#define MAX_PKTS 256
static uint8_t  g_pkt[MAX_PKTS][80];
static uint16_t g_pktLen[MAX_PKTS];
static int      g_pktN;
static int      g_acceptLeft = -1;   /* -1 = unlimited; else writes until push-back */

tBleStatus FS_ActiveLook_Client_WriteWithoutResp(const uint8_t *data, uint16_t length)
{
	if (g_acceptLeft == 0) return BLE_STATUS_FAILED;   /* TX pool full */
	if (g_acceptLeft > 0) g_acceptLeft--;
	if (g_pktN < MAX_PKTS && length <= 80) {
		memcpy(g_pkt[g_pktN], data, length);
		g_pktLen[g_pktN] = length;
		g_pktN++;
	}
	return BLE_STATUS_SUCCESS;
}
bool    FS_ActiveLook_Client_CanSend(void) { return true; }
uint8_t FS_ActiveLook_Client_GetBatteryLevel(void) { return 80; }

/* The string of the LAST txt packet drawn at (x,y), or NULL. Frame layout:
 * FF cmd fmt len | x(2) y(2) rot font colour str NUL | AA. */
static const char *last_text_at(int16_t x, int16_t y)
{
	const char *found = NULL;
	for (int i = 0; i < g_pktN; i++) {
		const uint8_t *p = g_pkt[i];
		if (p[1] != AL_CMD_TXT || p[2] != 0x00) continue;
		int16_t px = (int16_t)((p[4] << 8) | p[5]);
		int16_t py = (int16_t)((p[6] << 8) | p[7]);
		if (px == x && py == y) found = (const char *)&p[11];
	}
	return found;
}

/* Every holdFlush in send order must alternate HOLD, FLUSH, HOLD, FLUSH. A HOLD
 * after a HOLD is a frame abandoned half-way with the glasses still held. */
static int holds_balanced(void)
{
	int open = 0;
	for (int i = 0; i < g_pktN; i++) {
		const uint8_t *p = g_pkt[i];
		if (p[1] != AL_CMD_HOLD_FLUSH) continue;
		if (p[4] == AL_HOLD)  { if (open) return 0; open = 1; }
		if (p[4] == AL_FLUSH) { if (!open) return 0; open = 0; }
	}
	return 1;
}

/* ---------------- fixtures ---------------- */
#define ALT_X  200
#define ALT_Y  100

static void reset_world(void)
{
	memset(&g_cfg, 0, sizeof(g_cfg));
	memset(&g_gnss, 0, sizeof(g_gnss));
	memset(&g_baro, 0, sizeof(g_baro));
	g_vbat.voltage = 4000;
	g_cfg.al_rate = 250;
	g_cfg.al_layout_valid = 1;
	g_cfg.al_layout.count = 1;
	g_cfg.al_layout.el[0].field      = FS_HUD_FIELD_BARO_ALT;
	g_cfg.al_layout.el[0].x          = ALT_X;
	g_cfg.al_layout.el[0].y          = ALT_Y;
	g_cfg.al_layout.el[0].font       = 3;
	g_cfg.al_layout.el[0].units      = FS_HUD_UNITS_M;
	g_cfg.al_layout.el[0].decimals   = 0;
	g_cfg.al_layout.el[0].show_units = 0;
	g_tick = 1000;
	g_pktN = 0;
	g_logN = 0;
	g_acceptLeft = -1;
}

/* What the barometer driver does for one good read: update the struct, then
 * FS_Baro_DataReady_Callback -> the hook. */
static void baro_sample(int32_t pa100)
{
	g_baro.pressure = pa100;
	FS_ActiveLook_Mode0_BaroSample(pa100);
}

/* One HUD tick: the timer advances by AL_Rate, then the update runs. */
static void hud_tick(void)
{
	g_tick += g_cfg.al_rate;
	FS_ActiveLook_Mode0_Update();
}

static int alt_is_number(const char *t, int lo, int hi)
{
	int v;
	if (t == NULL || sscanf(t, "%d", &v) != 1) return 0;
	return v >= lo && v <= hi;
}

#define P_GROUND   10132500   /* 101325 Pa, sea level            */
#define P_1000M     8987600   /*  89876 Pa, ~1000 m above that    */

/* ---------------- F1-b: zero point from the hook ---------------- */
static void test_zero_point_set_before_glasses(void)
{
	reset_world();
	FS_ActiveLook_Mode0_ResetBaroRef();          /* switch-on (ActiveControl_Init) */

	/* Samples arrive at the ground with no glasses — nothing renders. */
	baro_sample(P_GROUND);
	baro_sample(P_GROUND);

	/* The aircraft climbs; the glasses link up only now. */
	baro_sample(P_1000M);
	FS_ActiveLook_Mode0_Init();
	hud_tick();

	/* Until v0.0.46 this read 0: the reference was captured at the first
	 * render, i.e. at the aircraft's pressure. */
	const char *t = last_text_at(ALT_X, ALT_Y);
	CHECK(t != NULL);
	CHECK(alt_is_number(t, 990, 1010));

	/* A glasses reconnect must not re-zero either. */
	FS_ActiveLook_Mode0_Init();
	g_pktN = 0;
	baro_sample(P_1000M);
	hud_tick();
	t = last_text_at(ALT_X, ALT_Y);
	CHECK(alt_is_number(t, 990, 1010));

	/* A new ACTIVE session does: the next first sample is the new zero. */
	FS_ActiveLook_Mode0_ResetBaroRef();
	baro_sample(P_1000M);
	FS_ActiveLook_Mode0_Init();
	g_pktN = 0;
	hud_tick();
	t = last_text_at(ALT_X, ALT_Y);
	CHECK(alt_is_number(t, 0, 0) || (t && strcmp(t, "-0") == 0));
}

/* ---------------- F1-a: stale barometer -> dashes ---------------- */
static void test_stale_baro_shows_dashes(void)
{
	reset_world();
	FS_ActiveLook_Mode0_ResetBaroRef();
	baro_sample(P_GROUND);
	FS_ActiveLook_Mode0_Init();

	baro_sample(P_1000M);
	hud_tick();
	CHECK(alt_is_number(last_text_at(ALT_X, ALT_Y), 990, 1010));
	CHECK(log_count("Baro silent") == 0);

	/* The barometer stops. The struct keeps its last pressure — exactly what
	 * FS_Baro_GetData() does on the target — and the ticks go on. */
	for (int i = 0; i < 7; i++) hud_tick();      /* 1750 ms: still a number */
	CHECK(alt_is_number(last_text_at(ALT_X, ALT_Y), 990, 1010));
	CHECK(log_count("Baro silent") == 0);

	hud_tick();                                   /* 2000 ms: stale */
	const char *t = last_text_at(ALT_X, ALT_Y);
	CHECK(t != NULL && strcmp(t, "----") == 0);
	CHECK(log_count("Baro silent: no sample for 2000 ms") == 1);

	for (int i = 0; i < 8; i++) hud_tick();       /* still silent: one line only */
	CHECK(strcmp(last_text_at(ALT_X, ALT_Y), "----") == 0);
	CHECK(log_count("Baro silent") == 1);

	/* A fresh sample: a number again, and the partner line. */
	baro_sample(P_1000M);
	hud_tick();
	CHECK(alt_is_number(last_text_at(ALT_X, ALT_Y), 990, 1010));
	CHECK(log_count("Baro talking again") == 1);
	CHECK(log_count("Baro silent") == 1);
}

/* A barometer that never delivers at all: dashes, not "0" — from the first
 * frame. But the LOG waits for the real 2 s (F1-f): a render a few ms into the
 * session, before the first read has completed, is not a silent barometer. */
static void test_never_sampled_shows_dashes(void)
{
	reset_world();
	g_tick = 0;
	FS_ActiveLook_Mode0_ResetBaroRef();          /* switch-on at t = 0 */
	FS_ActiveLook_Mode0_Init();

	g_tick = 100;
	FS_ActiveLook_Mode0_Update();
	const char *t = last_text_at(ALT_X, ALT_Y);
	CHECK(t != NULL && strcmp(t, "----") == 0);
	CHECK(log_count("Baro silent") == 0);

	g_tick = 1999;
	FS_ActiveLook_Mode0_Update();
	CHECK(log_count("Baro silent") == 0);

	g_tick = 2000;
	FS_ActiveLook_Mode0_Update();
	CHECK(strcmp(last_text_at(ALT_X, ALT_Y), "----") == 0);
	CHECK(log_count("Baro silent: no sample for 2000 ms") == 1);

	g_tick = 3000;
	FS_ActiveLook_Mode0_Update();
	CHECK(log_count("Baro silent") == 1);
	CHECK(log_count("Baro talking again") == 0);
}

/* The first sample arriving inside the first 2 s: a number, and no log line
 * of either kind — nothing went silent, so nothing came back. */
static void test_first_sample_in_time_logs_nothing(void)
{
	reset_world();
	g_tick = 0;
	FS_ActiveLook_Mode0_ResetBaroRef();
	FS_ActiveLook_Mode0_Init();

	g_tick = 100;
	FS_ActiveLook_Mode0_Update();
	CHECK(strcmp(last_text_at(ALT_X, ALT_Y), "----") == 0);

	baro_sample(P_GROUND);
	g_tick = 350;
	FS_ActiveLook_Mode0_Update();
	CHECK(alt_is_number(last_text_at(ALT_X, ALT_Y), 0, 0));
	CHECK(g_logN == 0);
}

/* ---------------- F1-c: finish the pending frame first ---------------- */
static void test_pending_frame_finished_before_rebuild(void)
{
	reset_world();
	FS_ActiveLook_Mode0_ResetBaroRef();
	baro_sample(P_GROUND);
	FS_ActiveLook_Mode0_Init();

	/* Tick 1: the pool takes the HOLD and the clear, then pushes back. */
	g_acceptLeft = 2;
	hud_tick();
	CHECK(g_pktN == 2);
	CHECK(FS_ActiveLook_Mode0_HasPendingFrame());

	/* Tick 2: still full, and the value has changed. Nothing may be rebuilt —
	 * the old frame stays pending, intact. */
	baro_sample(P_1000M);
	g_acceptLeft = 0;
	hud_tick();
	CHECK(g_pktN == 2);
	CHECK(FS_ActiveLook_Mode0_HasPendingFrame());

	/* Tick 3: the pool has room. The old frame goes out to its FLUSH first;
	 * only then the new one, with the new value. */
	g_acceptLeft = -1;
	hud_tick();
	CHECK(!FS_ActiveLook_Mode0_HasPendingFrame());
	CHECK(holds_balanced());
	CHECK(alt_is_number(last_text_at(ALT_X, ALT_Y), 990, 1010));

	/* The frame sent on tick 3 must be the OLD one finished (its txt reads 0)
	 * followed by a complete new one. */
	int txtSeen = 0;
	for (int i = 0; i < g_pktN; i++)
		if (g_pkt[i][1] == AL_CMD_TXT) txtSeen++;
	CHECK(txtSeen == 2);
}

/* The same, with room on the very next tick: until v0.0.46 the rebuild threw
 * the half-sent frame away and opened a second HOLD inside the first. */
static void test_no_hold_inside_hold(void)
{
	reset_world();
	FS_ActiveLook_Mode0_ResetBaroRef();
	baro_sample(P_GROUND);
	FS_ActiveLook_Mode0_Init();

	g_acceptLeft = 2;                 /* HOLD + clear, then push-back */
	hud_tick();
	CHECK(FS_ActiveLook_Mode0_HasPendingFrame());

	baro_sample(P_1000M);             /* something to redraw */
	g_acceptLeft = -1;
	hud_tick();
	CHECK(holds_balanced());
	CHECK(!FS_ActiveLook_Mode0_HasPendingFrame());
	CHECK(alt_is_number(last_text_at(ALT_X, ALT_Y), 990, 1010));
}

int main(void)
{
	test_zero_point_set_before_glasses();
	test_stale_baro_shows_dashes();
	test_never_sampled_shows_dashes();
	test_first_sample_in_time_logs_nothing();
	test_pending_frame_finished_before_rebuild();
	test_no_hold_inside_hold();

	printf("%d checks, %d failures\n", g_checks, g_fail);
	return g_fail ? 1 : 0;
}
