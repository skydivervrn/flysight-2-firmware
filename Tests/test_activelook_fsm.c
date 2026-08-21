/***************************************************************************
**  Host tests for the ENGO HUD state machine in FlySight/activelook.c.
**
**  WHY THIS EXISTS
**  The FSM is driven entirely by two things it does not own: the sequencer
**  task and the repeating hardware timer. Both are registered through function
**  pointers (UTIL_SEQ_RegTask, HW_TS_Create), so mocking those two calls is
**  enough to step the machine here exactly as the scheduler steps it on the
**  target — no refactor of the production code, and s_state stays private.
**
**  The bug this pins down (FW-H1): the AL_STATE_UPDATE case used to `break`
**  out while still in UPDATE when flow control said "not now". Only
**  FS_ActiveLook_Timer moves the FSM into UPDATE, and only from READY, so the
**  timer then found the wrong state forever and stopped scheduling the task.
**  The STOP-stuck watchdog that should have dropped and re-established the
**  link lives INSIDE that same case, so it went deaf with everything else:
**  a frozen HUD on a link that was still up, and nothing in the event log.
****************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "activelook.h"          /* real  */
#include "activelook_client.h"   /* mock  */
#include "app_common.h"          /* mock  */
#include "config.h"              /* mock  */

/* ---------------- check harness ---------------- */
static int g_checks = 0, g_fail = 0;
#define CHECK(cond) do { \
	g_checks++; \
	if (!(cond)) { g_fail++; printf("FAIL line %d: %s\n", __LINE__, #cond); } \
} while (0)

/* ---------------- sequencer mock ---------------- */
static void (*g_task)(void);      /* the FSM task, captured at RegTask       */
static int   g_taskPending;       /* set by UTIL_SEQ_SetTask, cleared by pump */
static int   g_scanRequested;

void UTIL_SEQ_RegTask(uint32_t taskMask, uint32_t flags, void (*task)(void))
{
	(void)flags;
	if (taskMask == (1u << CFG_TASK_FS_ACTIVELOOK_ID)) g_task = task;
}

void UTIL_SEQ_SetTask(uint32_t taskMask, uint32_t prio)
{
	(void)prio;
	if (taskMask == (1u << CFG_TASK_FS_ACTIVELOOK_ID)) g_taskPending = 1;
	if (taskMask == (1u << CFG_TASK_START_SCAN_ID))    g_scanRequested = 1;
}

/* Run the task as long as it keeps re-scheduling itself, like the sequencer
 * does. Bounded so a self-scheduling loop shows up as a failed check rather
 * than a hung test. Returns the number of task invocations. */
static int pump(void)
{
	int runs = 0;
	while (g_taskPending && runs < 100)
	{
		g_taskPending = 0;
		g_task();
		runs++;
	}
	CHECK(runs < 100);
	return runs;
}

/* ---------------- timer server mock ---------------- */
static void   (*g_timerCb)(void);
static uint8_t *g_timerIdSlot;
static int      g_timerArmed;
static uint32_t g_timerTicks;
static int      g_timerStarts;

void HW_TS_Create(uint32_t procId, uint8_t *pId, HW_TS_Mode_t mode, HW_TS_pTimerCb_t cb)
{
	(void)procId;
	CHECK(mode == hw_ts_Repeated);   /* a single-shot here would stop the HUD */
	g_timerCb     = cb;
	g_timerIdSlot = pId;
	*pId = 7;
}
void HW_TS_Start(uint8_t id, uint32_t ticks) { (void)id; g_timerArmed = 1; g_timerTicks = ticks; g_timerStarts++; }
void HW_TS_Stop(uint8_t id)                  { (void)id; g_timerArmed = 0; }
void HW_TS_Delete(uint8_t id)                { (void)id; g_timerArmed = 0; }

/* One period of the repeating update timer, followed by whatever it scheduled. */
static int tick(void)
{
	if (!g_timerArmed) return 0;
	g_timerCb();
	return pump();
}

/* ---------------- config mock ---------------- */
static FS_Config_Data_t g_cfg = { .al_mode = 1, .al_rate = 250 };
const FS_Config_Data_t *FS_Config_Get(void) { return &g_cfg; }

/* ---------------- log mock ---------------- */
static char g_lastLog[160];
static int  g_logLines;
void FS_Log_WriteEventAsync(const char *format, ...)
{
	va_list ap;
	va_start(ap, format);
	vsnprintf(g_lastLog, sizeof(g_lastLog), format, ap);
	va_end(ap);
	g_logLines++;
}

/* ---------------- glasses (client) mock ---------------- */
static const FS_ActiveLook_ClientCb_t *g_clientCb;
static int      g_canSend = 1;
static uint32_t g_stopStuckMs;
static uint16_t g_writeFailStreak;
static int      g_writes;             /* successful WWR calls                */
static int      g_forceDisconnects;
static tBleStatus g_writeResult = BLE_STATUS_SUCCESS;

void FS_ActiveLook_Client_RegisterCb(const FS_ActiveLook_ClientCb_t *cb) { g_clientCb = cb; }

tBleStatus FS_ActiveLook_Client_WriteWithoutResp(const uint8_t *data, uint16_t length)
{
	(void)data; (void)length;
	if (g_writeResult == BLE_STATUS_SUCCESS) g_writes++;
	return g_writeResult;
}

uint32_t FS_ActiveLook_Client_StopStuckMs(void)    { return g_stopStuckMs; }
uint16_t FS_ActiveLook_Client_WriteFailStreak(void){ return g_writeFailStreak; }
void     FS_ActiveLook_Client_ForceDisconnect(void){ g_forceDisconnects++; }
tBleStatus FS_ActiveLook_Client_EnableBatteryNotifications(void) { return BLE_STATUS_SUCCESS; }
bool     FS_ActiveLook_Client_CanSend(void)        { return g_canSend != 0; }
uint16_t FS_ActiveLook_Client_GetMTU(void)         { return 247; }

/* ---------------- mode0 mock ---------------- */
static int  g_mode0Inits, g_mode0Updates, g_mode0Drains;
static int  g_pendingFrame;
static int  g_baroResets;

void FS_ActiveLook_Mode0_Init(void)   { g_mode0Inits++; }
void FS_ActiveLook_Mode0_Update(void) { g_mode0Updates++; }
FS_ActiveLook_SetupStatus_t FS_ActiveLook_Mode0_Setup(void) { return FS_AL_SETUP_DONE; }
void FS_ActiveLook_Mode0_ResetBaroRef(void) { g_baroResets++; }
const char *FS_ActiveLook_Mode0_HudVersion(void) { return "test"; }
bool FS_ActiveLook_Mode0_HasPendingFrame(void)   { return g_pendingFrame != 0; }
bool FS_ActiveLook_Mode0_DrainFrame(void)
{
	g_mode0Drains++;
	if (!g_canSend) return false;
	g_pendingFrame = 0;
	return true;
}

/* ---------------- scenario helpers ---------------- */

/* Fresh boot: init, then the glasses finish discovery and the FSM is pumped
 * through its one-shot CLEAR into the idle READY state with the timer armed. */
static void bring_up_link(void)
{
	g_task = NULL; g_taskPending = 0; g_scanRequested = 0;
	g_timerCb = NULL; g_timerIdSlot = NULL; g_timerArmed = 0; g_timerStarts = 0;
	g_clientCb = NULL;
	g_canSend = 1; g_stopStuckMs = 0; g_writeFailStreak = 0;
	g_writes = 0; g_forceDisconnects = 0; g_writeResult = BLE_STATUS_SUCCESS;
	g_mode0Inits = g_mode0Updates = g_mode0Drains = 0;
	g_pendingFrame = 0; g_baroResets = 0;
	g_logLines = 0; g_lastLog[0] = '\0';

	FS_ActiveLook_Init();
	CHECK(g_task != NULL);
	CHECK(g_timerCb != NULL);
	CHECK(g_scanRequested == 1);
	CHECK(g_baroResets == 1);

	CHECK(g_clientCb != NULL && g_clientCb->OnDiscoveryComplete != NULL);
	g_clientCb->OnDiscoveryComplete();
	pump();
}

int main(void)
{
	/* ------------------------------------------------------------------
	 * 1. The happy path: discovery -> CLEAR -> READY with the timer armed,
	 *    and every tick draws a frame.
	 * ---------------------------------------------------------------- */
	bring_up_link();
	CHECK(g_mode0Inits == 1);
	CHECK(g_writes == 1);            /* the one-shot clear went out       */
	CHECK(g_timerArmed == 1);
	CHECK(g_timerTicks == 250u * 1000u / CFG_TS_TICK_VAL);

	tick();
	CHECK(g_mode0Updates == 1);
	tick();
	CHECK(g_mode0Updates == 2);

	/* ------------------------------------------------------------------
	 * 2. FW-H1: STOP asserted. The FSM must keep taking ticks (deferring the
	 *    draw, not dying on it) and must fire the 6-second self-heal.
	 * ---------------------------------------------------------------- */
	bring_up_link();
	tick();
	CHECK(g_mode0Updates == 1);

	g_canSend = 0;                   /* glasses sent 0x02 on CB9          */
	g_stopStuckMs = 0;
	int updatesAtStop = g_mode0Updates;

	/* 5 s of ticks at 250 ms. Nothing may be drawn, but the task must be
	 * entered on every single tick — that is what "not deaf" means, and it is
	 * the precondition for the watchdog below ever running. */
	int ticksTaken = 0;
	for (int ms = 250; ms <= 5000; ms += 250)
	{
		g_stopStuckMs = (uint32_t)ms;
		if (tick() > 0) ticksTaken++;
	}
	CHECK(ticksTaken == 20);
	CHECK(g_mode0Updates == updatesAtStop);   /* STOP was honoured         */
	CHECK(g_forceDisconnects == 0);           /* too early to give up      */

	/* Past 6 s the watchdog drops the link and says so in the event log. */
	g_logLines = 0;
	g_stopStuckMs = 6250;
	tick();
	CHECK(g_forceDisconnects == 1);
	CHECK(g_logLines == 1);
	CHECK(strstr(g_lastLog, "STOP stuck") != NULL);

	/* ------------------------------------------------------------------
	 * 3. STOP -> OK: drawing resumes on the next tick.
	 * ---------------------------------------------------------------- */
	bring_up_link();
	tick();
	CHECK(g_mode0Updates == 1);

	g_canSend = 0;
	g_stopStuckMs = 250;
	tick();
	tick();
	CHECK(g_mode0Updates == 1);      /* still held                        */

	g_canSend = 1;                   /* glasses sent 0x01 on CB9          */
	g_stopStuckMs = 0;
	tick();
	CHECK(g_mode0Updates == 2);      /* and we are drawing again          */
	tick();
	CHECK(g_mode0Updates == 3);

	/* ------------------------------------------------------------------
	 * 4. FW-H1, the same wedge one state earlier: STOP (or a busy TX pool)
	 *    at the moment of the one-shot CLEAR. That state is entered exactly
	 *    once and it is what arms the timer, so failing there used to mean
	 *    the HUD never started at all. Losing the wipe is fine — every HUD
	 *    frame opens with its own clear.
	 * ---------------------------------------------------------------- */
	bring_up_link();               /* leaves a healthy link                */
	g_canSend = 0;                 /* ... now re-run discovery under STOP  */
	g_writes = 0;
	g_timerArmed = 0; g_timerStarts = 0;
	g_clientCb->OnDiscoveryComplete();
	pump();
	CHECK(g_writes == 0);          /* the wipe was skipped, as it should be */
	CHECK(g_timerStarts == 1);     /* but the update timer IS running       */

	g_canSend = 1;
	tick();
	CHECK(g_mode0Updates > 0);     /* and the HUD comes up on the next tick */

	/* Same again with the write itself failing (full CPU2 TX pool). */
	bring_up_link();
	g_writeResult = BLE_STATUS_FAILED;
	g_timerArmed = 0; g_timerStarts = 0;
	g_mode0Updates = 0;
	g_clientCb->OnDiscoveryComplete();
	pump();
	CHECK(g_timerStarts == 1);
	g_writeResult = BLE_STATUS_SUCCESS;
	tick();
	CHECK(g_mode0Updates == 1);

	/* ------------------------------------------------------------------
	 * 5. The other watchdog: a run of failed writes also drops the link.
	 * ---------------------------------------------------------------- */
	bring_up_link();
	tick();
	g_writeFailStreak = 8;
	g_logLines = 0;
	tick();
	CHECK(g_forceDisconnects == 1);
	CHECK(strstr(g_lastLog, "failed writes") != NULL);

	/* ------------------------------------------------------------------
	 * 6. A frame the glasses paused mid-way is finished as soon as flow
	 *    control reopens — FS_ActiveLook_OnFlowResume, called from the CB9
	 *    handler. Without it the rest of the frame waited for an update tick
	 *    that skips entirely when no displayed value changed, leaving the
	 *    glasses holding a stale screen (holdFlush HOLD sent, FLUSH not).
	 * ---------------------------------------------------------------- */
	bring_up_link();
	g_pendingFrame = 1;
	g_canSend = 0;
	g_mode0Drains = 0;
	FS_ActiveLook_OnFlowResume();   /* STOP still on: schedules, drains nothing */
	pump();
	CHECK(g_mode0Drains == 1);
	CHECK(g_pendingFrame == 1);

	g_canSend = 1;
	g_mode0Drains = 0;
	FS_ActiveLook_OnFlowResume();
	pump();
	CHECK(g_mode0Drains == 1);
	CHECK(g_pendingFrame == 0);     /* frame completed, glasses released   */

	/* With nothing pending, a resume must not wake the task at all. */
	g_mode0Drains = 0;
	FS_ActiveLook_OnFlowResume();
	CHECK(g_taskPending == 0);
	CHECK(g_mode0Drains == 0);

	/* ------------------------------------------------------------------
	 * 7. Disconnect disarms the timer and no tick can draw afterwards.
	 * ---------------------------------------------------------------- */
	bring_up_link();
	tick();
	int updatesBefore = g_mode0Updates;
	FS_ActiveLook_OnDisconnect();
	CHECK(g_timerArmed == 0);
	tick();
	CHECK(g_mode0Updates == updatesBefore);

	printf("activelook_fsm: %d/%d checks passed\n", g_checks - g_fail, g_checks);
	return g_fail ? 1 : 0;
}
