/***************************************************************************
**  Host tests for the Device State control point trace (FlySight/cp_log.c
**  and its wiring in FlySight/device_state.c).
**
**  WHY THIS EXISTS
**  The phone sometimes gets an error on "Switch to ACTIVE" and succeeds only
**  after several retries. To diagnose the next occurrence from the card, every
**  control-point request now leaves one EVENT.CSV line: opcode, the status
**  sent to the phone, the mode it arrived in, and what the mode machine said
**  to the deferred part. The catch is that EVENT.CSV is closed in SLEEP, which
**  is exactly where a "Switch to ACTIVE" arrives — so records wait in RAM and
**  are written once a log is open. This test compiles the real device_state.c
**  next to thin mocks (BLE TX queue, timer server, mode machine, log) and
**  checks that the lines that come out are the requests that went in.
****************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "main.h"                     /* mock */
#include "ble_tx_queue.h"             /* mock */
#include "custom_stm.h"               /* mock */
#include "state.h"                    /* mock */
#include "device_state.h"             /* real */
#include "cp_log.h"                   /* real */
#include "mode.h"                     /* real (enums only) */
#include "control_point_protocol.h"   /* real */

/* ---------------- check harness ---------------- */
static int g_checks = 0, g_fail = 0;
#define CHECK(cond) do { \
	g_checks++; \
	if (!(cond)) { g_fail++; printf("FAIL line %d: %s\n", __LINE__, #cond); } \
} while (0)

/* ---------------- PRIMASK / timer mocks ---------------- */
static int g_irqDisabled;
uint32_t __get_PRIMASK(void) { return (uint32_t)g_irqDisabled; }
void     __disable_irq(void) { g_irqDisabled = 1; }
void     __set_PRIMASK(uint32_t v) { g_irqDisabled = (int)v; }

static HW_TS_pTimerCb_t g_timerCb;
static int g_timerArmed;
void HW_TS_Create(uint32_t procId, uint8_t *pId, HW_TS_Mode_t mode, HW_TS_pTimerCb_t cb)
{ (void)procId; (void)mode; *pId = 3; g_timerCb = cb; }
void HW_TS_Start(uint8_t id, uint32_t t) { (void)id; (void)t; g_timerArmed = 1; }
void HW_TS_Stop(uint8_t id)             { (void)id; g_timerArmed = 0; }

/* The response fallback timer firing (interrupt context on the target). */
static void fire_timer(void)
{
	if (!g_timerArmed) return;
	g_timerArmed = 0;
	g_timerCb();
}

/* ---------------- BLE mocks ---------------- */
uint8_t SizeDs_Control_Point;
static uint8_t g_lastResp[3];
static BLE_TX_Queue_callback_t g_lastRespCb;
static int g_responses;

void BLE_TX_Queue_SendTxPacket(uint8_t char_id, uint8_t *data, uint8_t length,
                               uint8_t *size_ptr, BLE_TX_Queue_callback_t callback)
{
	(void)char_id; (void)size_ptr;
	memcpy(g_lastResp, data, length < 3 ? length : 3);
	if (callback) g_lastRespCb = callback;   /* the queue calls it on TX */
	g_responses++;
}
bool Custom_STM_IsDsControlPointNotificationComplete(uint16_t h) { return h == 0x42; }

static FS_State_Data_t g_state;
const FS_State_Data_t *FS_State_Get(void) { return &g_state; }

/* ---------------- mode machine mock ---------------- */
static FS_Mode_State_t g_mode = FS_MODE_STATE_SLEEP;
static FS_Mode_RequestResult_t g_modeAnswer = FS_MODE_REQUEST_ACCEPTED;
static int g_modeRequests;
FS_Mode_State_t FS_Mode_State(void) { return g_mode; }
FS_Mode_RequestResult_t FS_Mode_RequestState(FS_Mode_State_t t)
{ (void)t; g_modeRequests++; return g_modeAnswer; }
FS_Mode_RequestResult_t FS_Mode_RequestTerminalAction(FS_TerminalAction_t a)
{ (void)a; g_modeRequests++; return g_modeAnswer; }

/* ---------------- log mock ---------------- */
static int  g_logOpen;          /* an EVENT.CSV is open (ACTIVE/START)      */
static int  g_logFull;          /* the async event buffer has no free slot  */
static char g_lines[32][96];
static int  g_nLines;
bool FS_Log_EventSlotFree(void) { return g_logOpen && !g_logFull; }
void FS_Log_WriteEventAsync(const char *format, ...)
{
	CHECK(!g_irqDisabled);      /* never formatted/logged under PRIMASK     */
	if (!g_logOpen || g_logFull) return;   /* what log.c does: drop        */
	va_list ap;
	va_start(ap, format);
	if (g_nLines < 32) vsnprintf(g_lines[g_nLines++], sizeof(g_lines[0]), format, ap);
	va_end(ap);
}

/* ---------------- helpers ---------------- */
static void cp_write(uint8_t opcode, uint8_t indications)
{
	uint8_t p[1] = { opcode };
	DeviceState_Handle_DS_ControlPointWrite(p, 1, 0x0001, indications);
	CHECK(!g_irqDisabled);
}

/* The phone's indication of our response went out. */
static void response_delivered(void)
{
	if (g_lastRespCb) g_lastRespCb();
	g_lastRespCb = NULL;
	DeviceState_Handle_DS_ControlPointNotificationComplete(0x42);
}

static void drain(int calls)
{
	for (int i = 0; i < calls; ++i) DeviceState_FlushCpLog();
}

static void reset(void)
{
	FS_CpLog_Reset();
	g_nLines = 0; g_logOpen = 0; g_logFull = 0;
	g_mode = FS_MODE_STATE_SLEEP; g_modeAnswer = FS_MODE_REQUEST_ACCEPTED;
	g_modeRequests = 0; g_responses = 0; g_lastRespCb = NULL;
	fire_timer();               /* flush any request a previous case left */
	g_nLines = 0;
	FS_CpLog_Reset();
}

int main(void)
{
	DeviceState_Init();
	CHECK(g_timerCb != NULL);

	/* ------------------------------------------------------------------
	 * 1. The reported symptom, reconstructed: in SLEEP the phone asks for
	 *    ACTIVE, asks again while the first is still waiting for its
	 *    indication (BUSY), and the first one goes through. No line can be
	 *    written in SLEEP; all of them appear, in order, once the log opens.
	 * ---------------------------------------------------------------- */
	reset();
	cp_write(0x11, 1);
	CHECK(g_lastResp[1] == 0x11 && g_lastResp[2] == CP_STATUS_SUCCESS);
	cp_write(0x11, 1);
	CHECK(g_lastResp[2] == CP_STATUS_BUSY);
	response_delivered();
	CHECK(g_modeRequests == 1);
	CHECK(g_nLines == 0);                   /* SLEEP: nothing written       */

	g_mode = FS_MODE_STATE_ACTIVE;          /* the switch happened ...      */
	g_logOpen = 1;                          /* ... and opened EVENT.CSV     */
	drain(1);
	CHECK(g_nLines == 1);                   /* one line per flush call      */
	drain(5);
	CHECK(g_nLines == 2);
	CHECK(strcmp(g_lines[0], "CP#1 op 0x11 st 0x01 in SLEEP run ok") == 0);
	CHECK(strcmp(g_lines[1], "CP#2 op 0x11 st 0x06 in SLEEP run -") == 0);

	/* ------------------------------------------------------------------
	 * 2. "SUCCESS, but nothing happened": the phone is told OK, then the
	 *    mode machine refuses the deferred request. The line says so.
	 * ---------------------------------------------------------------- */
	reset();
	g_modeAnswer = FS_MODE_REQUEST_BUSY;
	cp_write(0x11, 1);
	CHECK(g_lastResp[2] == CP_STATUS_SUCCESS);
	response_delivered();
	g_logOpen = 1;
	drain(3);
	CHECK(g_nLines == 1);
	CHECK(strcmp(g_lines[0], "CP#1 op 0x11 st 0x01 in SLEEP run busy") == 0);

	/* ------------------------------------------------------------------
	 * 3. The indication never completes: the 200 ms fallback timer runs the
	 *    request (interrupt context on the target) and still fills the line.
	 *    Until then the record holds back later ones, so order is kept.
	 * ---------------------------------------------------------------- */
	reset();
	g_mode = FS_MODE_STATE_ACTIVE;
	g_logOpen = 1;
	cp_write(0x10, 1);                      /* SLEEP request, deferred     */
	cp_write(0x01, 1);                      /* FW version: immediate        */
	CHECK(g_nLines == 0);                   /* held behind the pending one  */
	fire_timer();
	CHECK(g_modeRequests == 1);
	drain(3);
	CHECK(g_nLines == 2);
	CHECK(strcmp(g_lines[0], "CP#1 op 0x10 st 0x01 in ACTIVE run ok") == 0);
	CHECK(strcmp(g_lines[1], "CP#2 op 0x01 st 0x01 in ACTIVE run -") == 0);

	/* ------------------------------------------------------------------
	 * 4. With the log open, an immediate request is written from the
	 *    handler itself; not-permitted is recorded with its status.
	 * ---------------------------------------------------------------- */
	reset();
	g_mode = FS_MODE_STATE_ACTIVE;
	g_logOpen = 1;
	cp_write(0x13, 1);                      /* CONFIG from ACTIVE           */
	CHECK(g_lastResp[2] == CP_STATUS_OPERATION_NOT_PERMITTED);
	CHECK(g_nLines == 1);
	CHECK(strcmp(g_lines[0], "CP#1 op 0x13 st 0x05 in ACTIVE run -") == 0);
	cp_write(0x7E, 1);                      /* unknown opcode               */
	CHECK(g_nLines == 2);
	CHECK(strcmp(g_lines[1], "CP#2 op 0x7E st 0x02 in ACTIVE run -") == 0);

	/* ------------------------------------------------------------------
	 * 5. A full async event buffer must not swallow a record: it stays in
	 *    RAM until a later call finds a free slot.
	 * ---------------------------------------------------------------- */
	reset();
	g_mode = FS_MODE_STATE_ACTIVE;
	g_logOpen = 1;
	g_logFull = 1;
	cp_write(0x01, 1);
	drain(3);
	CHECK(g_nLines == 0);
	g_logFull = 0;
	drain(1);
	CHECK(g_nLines == 1);
	CHECK(strcmp(g_lines[0], "CP#1 op 0x01 st 0x01 in ACTIVE run -") == 0);

	/* ------------------------------------------------------------------
	 * 6. More requests in SLEEP than the ring holds: the oldest are
	 *    dropped, counted, and the count is reported before the rest.
	 * ---------------------------------------------------------------- */
	reset();
	g_mode = FS_MODE_STATE_PAIRING;         /* every request: not permitted */
	for (int i = 0; i < 10; ++i) cp_write(0x11, 1);
	CHECK(g_lastResp[2] == CP_STATUS_OPERATION_NOT_PERMITTED);
	g_mode = FS_MODE_STATE_ACTIVE;
	g_logOpen = 1;
	drain(20);
	CHECK(g_nLines == 1 + (int)FS_CPLOG_DEPTH);
	CHECK(strcmp(g_lines[0], "CP log: 2 older requests dropped") == 0);
	CHECK(strcmp(g_lines[1], "CP#3 op 0x11 st 0x05 in PAIRING run -") == 0);
	CHECK(strcmp(g_lines[8], "CP#10 op 0x11 st 0x05 in PAIRING run -") == 0);

	/* ------------------------------------------------------------------
	 * 7. Without indications enabled the request is still run (deferred by
	 *    one timer tick) and still recorded; no response is sent.
	 * ---------------------------------------------------------------- */
	reset();
	cp_write(0x11, 0);
	CHECK(g_responses == 0);
	fire_timer();
	CHECK(g_modeRequests == 1);
	g_logOpen = 1;
	drain(2);
	CHECK(g_nLines == 1);
	CHECK(strcmp(g_lines[0], "CP#1 op 0x11 st 0x01 in SLEEP run ok") == 0);

	/* Every line fits the 80-byte EVENT.CSV message with room to spare. */
	{
		char line[96];
		FS_CpLog_Record_t r = { 0xFFFFFFFFu, 0xFF, 0xFF, 0xFF, 0xFD };
		FS_CpLog_Format(line, sizeof(line), &r, 0);
		CHECK(strlen(line) < 64);
		FS_CpLog_Format(line, sizeof(line), &r, 0xFFFFFFFFu);
		CHECK(strlen(line) < 64);
	}

	printf("device_state: %d/%d checks passed\n", g_checks - g_fail, g_checks);
	return g_fail ? 1 : 0;
}
