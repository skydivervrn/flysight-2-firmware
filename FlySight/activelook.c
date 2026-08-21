#include "main.h"
#include "activelook.h"
#include "activelook_client.h"
#include "ble_diag.h"
#include "activelook_mode0.h"
#include "activelook_proto.h"
#include "app_common.h"
#include "config.h"
#include "dbg_trace.h"
#include "log.h"
#include "stm32_seq.h"
#include <string.h>
#include <stdio.h>

/*----- State machine states -----*/
typedef enum
{
    AL_STATE_INIT = 0,       /* Not discovered yet */
    AL_STATE_CFG_WRITE,
    AL_STATE_SETUP,
    AL_STATE_CFG_SET,
    AL_STATE_CLEAR,          /* Need to send "clear" command */
    AL_STATE_READY,          /* Idle, waiting for GNSS data */
    AL_STATE_UPDATE,         /* GNSS arrived, update page */
} FS_ActiveLook_State_t;

/*----- Module-level static variables -----*/
static FS_ActiveLook_State_t s_state = AL_STATE_INIT;
static bool s_running = false;

static uint8_t timer_id;

typedef struct {
    /**
     * Called exactly once when this mode is selected, before setup() is called.
     * This resets any static variables so we start from scratch.
     */
    void (*init)(void);

    /**
     * Called once (or multiple times) per pass of the main SM, until
     * it returns FS_AL_SETUP_DONE.
     */
    FS_ActiveLook_SetupStatus_t (*setup)(void);

    /**
     * Called whenever we want the mode to redraw or update data.
     */
    void (*update)(void);
} FS_ActiveLook_ModeOps;

static const FS_ActiveLook_ModeOps s_modeTable[] =
{
   { // mode 0
        FS_ActiveLook_Mode0_Init,
        FS_ActiveLook_Mode0_Setup,
        FS_ActiveLook_Mode0_Update
   },
   // etc
};

#define FS_ACTIVELOOK_NUM_MODES (sizeof(s_modeTable)/sizeof(s_modeTable[0]))
static const FS_ActiveLook_ModeOps *s_currentMode = NULL;

/* Forward references */
static void FS_ActiveLook_Timer(void);
static void FS_ActiveLook_Task(void);
static void OnActiveLookDiscoveryComplete(void);

/* Callback struct for the ActiveLook client library */
static const FS_ActiveLook_ClientCb_t s_alk_cb =
{
    .OnDiscoveryComplete = OnActiveLookDiscoveryComplete
};

void AL_SelectMode(uint8_t modeId)
{
    // Suppose you have s_modeTable[] with one entry per mode
    if (modeId < FS_ACTIVELOOK_NUM_MODES)
        s_currentMode = &s_modeTable[modeId];
    else
        s_currentMode = &s_modeTable[0];

    // Call the mode’s init if it exists
    if (s_currentMode->init)
        s_currentMode->init();
}

/*******************************************************************************
 * Called by the ActiveLook client once the Rx characteristic is discovered.
 * We'll move the state to CLEAR, meaning "clear" next, and schedule the task.
 ******************************************************************************/
static void OnActiveLookDiscoveryComplete(void)
{
    /* A connection can finish after ACTIVE has already been left. Never let a
     * late GATT callback resurrect the HUD FSM or start a deleted timer. */
    if (!s_running)
    {
        FS_ActiveLook_Client_ForceDisconnect();
        return;
    }

    APP_DBG_MSG("ActiveLook: Discovery complete\n");

    FS_Log_WriteEventAsync("ENGO link ready (MTU %u)",
                           (unsigned)FS_ActiveLook_Client_GetMTU());

    /* Select the display mode (was previously done inside AL_STATE_CFG_WRITE). */
    AL_SelectMode(FS_Config_Get()->al_mode - 1);

    /* Direct-draw rendering: skip cfgWrite/layout-setup/cfgSet entirely (those save
     * to the glasses' persistent flash and proved unreliable). Go straight to CLEAR
     * then the periodic UPDATE, which draws with clear+txt every tick. */
    s_state = AL_STATE_CLEAR;

    // Begin updates
    UTIL_SEQ_SetTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, CFG_SCH_PRIO_0);

    // Start Battery notifications
    FS_ActiveLook_Client_EnableBatteryNotifications();
}

/*******************************************************************************
 * Our main sequencer task. We handle the state machine, sending WWR commands
 * without blocking. Each line is sent in a separate state so that we don't
 * bombard the BLE stack with multiple WWR calls in a single pass.
 ******************************************************************************/
static void FS_ActiveLook_Task(void)
{
    char tmp[32];

    switch (s_state)
    {
    case AL_STATE_INIT:
        /* Not discovered yet, do nothing */
        break;

    case AL_STATE_CFG_WRITE:
    {
        /* Gate on flow control — skip this tick if not ready */
        if (!FS_ActiveLook_Client_CanSend()) {
            break;
        }

        /* Build cfgWrite payload: 12-byte name (padded) + 4-byte version + 4-byte password */
        uint8_t data[20];
        uint8_t di = 0;

        snprintf(tmp, sizeof(tmp), "FLYSIGHT");
        size_t textLen = strlen(tmp);
        memcpy(&data[di], tmp, textLen);
        di += textLen;

        while (textLen < 12)
        {
            data[di++] = 0x00; // padding
            textLen++;
        }

        data[di++] = 0x00;  // version
        data[di++] = 0x00;
        data[di++] = 0x00;
        data[di++] = 0x01;

        data[di++] = 0x01;  // password
        data[di++] = 0x02;
        data[di++] = 0x03;
        data[di++] = 0x04;

        uint8_t packet[128];
        size_t plen = AL_BuildFrame(packet, sizeof(packet),
                                    AL_CMD_CFG_WRITE, data, di);
        if (plen == 0) {
            /* Frame build failed — retry next tick (don't advance) */
            break;
        }

        tBleStatus st = FS_ActiveLook_Client_WriteWithoutResp(packet, (uint16_t)plen);
        if (st != BLE_STATUS_SUCCESS) {
            /* Write failed — retry next tick (don't advance) */
            break;
        }

        /* Get mode from config file */
        AL_SelectMode(FS_Config_Get()->al_mode - 1);

        /* Next config step */
        s_state = AL_STATE_SETUP;
        UTIL_SEQ_SetTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, CFG_SCH_PRIO_0);
        break;
    }

    case AL_STATE_SETUP:
    {
        /* Gate on flow control — skip this tick if not ready */
        if (!FS_ActiveLook_Client_CanSend()) {
            break;
        }

        if (s_currentMode && s_currentMode->setup)
        {
            FS_ActiveLook_SetupStatus_t status = s_currentMode->setup();
            if (status == FS_AL_SETUP_DONE)
            {
                /* Once the mode is fully set up, move on */
                s_state = AL_STATE_CFG_SET;
            }
            /* else: mode still needs more calls — come back on next pass */
            UTIL_SEQ_SetTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, CFG_SCH_PRIO_0);
        }
        break;
    }

    case AL_STATE_CFG_SET:
    {
        /* Gate on flow control — skip this tick if not ready */
        if (!FS_ActiveLook_Client_CanSend()) {
            break;
        }

        /* Build cfgSet payload: 12-byte name (padded) */
        uint8_t data[12];
        uint8_t di = 0;

        snprintf(tmp, sizeof(tmp), "FLYSIGHT");
        size_t textLen = strlen(tmp);
        memcpy(&data[di], tmp, textLen);
        di += textLen;

        while (textLen < 12)
        {
            data[di++] = 0x00; // padding
            textLen++;
        }

        uint8_t packet[128];
        size_t plen = AL_BuildFrame(packet, sizeof(packet),
                                    AL_CMD_CFG_SET, data, di);
        if (plen == 0) {
            /* Frame build failed — retry next tick (don't advance) */
            break;
        }

        tBleStatus st = FS_ActiveLook_Client_WriteWithoutResp(packet, (uint16_t)plen);
        if (st != BLE_STATUS_SUCCESS) {
            /* Write failed — retry next tick (don't advance) */
            break;
        }

        /* Next config step */
        s_state = AL_STATE_CLEAR;
        UTIL_SEQ_SetTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, CFG_SCH_PRIO_0);
        break;
    }

    case AL_STATE_CLEAR:
    {
        /* One-shot wipe of whatever the glasses were showing before we linked.
         *
         * It is BEST EFFORT on purpose. This state is entered exactly once, from
         * OnActiveLookDiscoveryComplete, and the update timer is not running yet
         * — it is started right here. So any early return that left s_state at
         * CLEAR wedged the HUD before it ever drew a frame: no timer, no task,
         * no watchdog, and the link staying up the whole time. The three ways
         * out (glasses asserting STOP in the moment between the CCCD write and
         * this task running, a full CPU2 TX pool, a frame that would not build)
         * are all transient, and none of them is worth that.
         *
         * Losing the wipe costs nothing anyway: every Mode0 frame opens with its
         * own clear (AL_FrameAddClear), so the screen is wiped again within one
         * tick regardless. */
        if (FS_ActiveLook_Client_CanSend())
        {
            uint8_t packet[AL_FRAME_OVERHEAD];  /* clear has no payload: exactly 5 bytes */
            size_t plen = AL_BuildFrame(packet, sizeof(packet),
                                        AL_CMD_CLEAR, NULL, 0);
            if (plen != 0)
            {
                APP_DBG_MSG("ActiveLook: Clearing display...\n");
                (void)FS_ActiveLook_Client_WriteWithoutResp(packet, (uint16_t)plen);
            }
        }

        /* Now go idle. Wait for the update timer to fire */
        s_state = AL_STATE_READY;

        /* Start update timer */
        HW_TS_Start(timer_id, FS_Config_Get()->al_rate * 1000 / CFG_TS_TICK_VAL);
        break;
    }

    case AL_STATE_READY:
        /* Idle between ticks — but if a frame is still partially sent (TX pool
         * pushed back mid-frame), keep draining it. The task is (re)scheduled by
         * FS_ActiveLook_TxPoolAvailable() as pool space frees up. */
        if (FS_ActiveLook_Mode0_HasPendingFrame())
            FS_ActiveLook_Mode0_DrainFrame();
        break;

    case AL_STATE_UPDATE:
        /* Self-heal watchdogs (glidex + official-SDK-informed). A wedged glasses
         * link shows up as either STOP asserted forever (buffer never drains, or
         * the resume notification was lost) or writes failing back-to-back. In
         * both cases dropping the link and rescanning recovers without the user
         * power-cycling anything. */
        if (FS_ActiveLook_Client_StopStuckMs() > 6000u)
        {
            FS_Log_WriteEventAsync("ENGO self-heal: STOP stuck >6s -> reconnect");
            FS_ActiveLook_Client_ForceDisconnect();
            s_state = AL_STATE_READY; /* disconnection event will reset the FSM */
            break;
        }
        if (FS_ActiveLook_Client_WriteFailStreak() >= 8u)
        {
            FS_Log_WriteEventAsync("ENGO self-heal: %u failed writes -> reconnect",
                                   (unsigned)FS_ActiveLook_Client_WriteFailStreak());
            FS_ActiveLook_Client_ForceDisconnect();
            s_state = AL_STATE_READY;
            break;
        }
        /* Gate on flow control — defer this tick if not ready.
         *
         * Go back to READY before deferring. Nothing else re-enters this state:
         * FS_ActiveLook_Timer only promotes READY -> UPDATE, and
         * FS_ActiveLook_TxPoolAvailable only wakes the task when a frame is
         * half-sent. Breaking out while still in UPDATE therefore stopped the
         * timer from ever scheduling the task again — and since the STOP-stuck
         * watchdog above lives inside this very case, the one thing that could
         * have recovered the link went deaf with it. The HUD froze while BLE
         * stayed up, with no self-heal and nothing in the log. */
        if (!FS_ActiveLook_Client_CanSend()) {
            s_state = AL_STATE_READY;
            break;
        }
        if (s_currentMode && s_currentMode->update)
            s_currentMode->update();
        s_state = AL_STATE_READY;
        break;
    }
}

static void FS_ActiveLook_Timer(void)
{
    if (s_state == AL_STATE_READY)
    {
		/* Update the page */
		s_state = AL_STATE_UPDATE;
		UTIL_SEQ_SetTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, CFG_SCH_PRIO_0);
    }
}

/*******************************************************************************
 * Called from app_ble.c on ACI_GATT_TX_POOL_AVAILABLE: CPU2 TX buffers freed
 * up, so a frame that was paused mid-send can continue NOW (tens of ms after
 * the push-back) instead of waiting for the next 1 s update tick.
 ******************************************************************************/
void FS_ActiveLook_TxPoolAvailable(void)
{
    if (FS_ActiveLook_Mode0_HasPendingFrame())
        UTIL_SEQ_SetTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, CFG_SCH_PRIO_0);
}

/*******************************************************************************
 * Called from activelook_client.c when the glasses send 0x01 (resume) on CB9.
 *
 * A STOP that lands in the middle of a frame leaves the glasses HELD: the
 * opening holdFlush(HOLD) went out, the closing FLUSH did not, so the display
 * shows the previous frame until the rest is sent. Waiting for the next update
 * tick is not enough — that tick may find no changed values and return without
 * touching the pending frame, leaving the screen held indefinitely. So finish
 * the frame as soon as the glasses say they can take it again.
 *
 * Same shape as FS_ActiveLook_TxPoolAvailable above: the client layer notifies,
 * the app layer decides what to schedule.
 ******************************************************************************/
void FS_ActiveLook_OnFlowResume(void)
{
    if (FS_ActiveLook_Mode0_HasPendingFrame())
        UTIL_SEQ_SetTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, CFG_SCH_PRIO_0);
}

/*******************************************************************************
 * Called on BLE disconnect (from app_ble.c EndDevice handler).
 * Resets the app FSM to the idle/disconnected state and disarms the repeating
 * update timer so no further writes are attempted until re-discovery fires
 * OnActiveLookDiscoveryComplete again.
 ******************************************************************************/
void FS_ActiveLook_OnDisconnect(void)
{
    APP_DBG_MSG("ActiveLook: Disconnect — resetting FSM\n");

    /* Stop the periodic update timer (safe to call even when not running) */
    if (s_running)
        HW_TS_Stop(timer_id);

    /* Reset FSM to disconnected/idle — OnActiveLookDiscoveryComplete will
     * restart it cleanly when the glasses reconnect. */
    s_state = AL_STATE_INIT;
}

/*******************************************************************************
 * Standard init/deinit for ActiveLook
 ******************************************************************************/
/* Boot info for the event log. Called from active_mode.c AFTER FS_Log_Init
 * (FS_ActiveLook_Init itself runs before logging is up, so a log line from
 * there would be silently dropped — same rake as FS_EngoBind_LogStatus). */
void FS_ActiveLook_LogBootInfo(void)
{
    FS_Log_WriteEventAsync("ENGO HUD v%s, AL_Rate %u ms",
                           FS_ActiveLook_Mode0_HudVersion(),
                           (unsigned)FS_Config_Get()->al_rate);
}

void FS_ActiveLook_Init(void)
{
    s_running = true;
    FS_BleDiag_Log("ENGO lifecycle ACTIVE");

    /* Re-zero the barometric altitude at every ACTIVE entry ("switch-on"):
     * the device "off" is SLEEP with RAM retained, so without this the zero
     * point would survive across sessions. Deliberately NOT done on glasses
     * reconnect (Mode0_Init) — a mid-flight reconnect must not re-zero. */
    FS_ActiveLook_Mode0_ResetBaroRef();

    /* Register the callback for discovery */
    FS_ActiveLook_Client_RegisterCb(&s_alk_cb);

    /* Register our local "task" with the sequencer */
    UTIL_SEQ_RegTask(1 << CFG_TASK_FS_ACTIVELOOK_ID, UTIL_SEQ_RFU, FS_ActiveLook_Task);

    /* Start in the INIT state */
    s_state = AL_STATE_INIT;

    /* If we want to automatically scan/connect: */
    UTIL_SEQ_SetTask(1 << CFG_TASK_START_SCAN_ID, CFG_SCH_PRIO_0);

	/* Initialize update timer */
	HW_TS_Create(CFG_TIM_PROC_ID_ISR, &timer_id, hw_ts_Repeated, FS_ActiveLook_Timer);
}

void FS_ActiveLook_DeInit(void)
{
	/* Close the lifecycle gate before queuing asynchronous radio teardown.
	 * Otherwise its later disconnection event restarts scanning outside ACTIVE,
	 * allowing powered-on glasses to reconnect and receive retained sensor data. */
	s_running = false;
	FS_BleDiag_Log("ENGO lifecycle STOP");

	/* Reset state */
	s_state = AL_STATE_INIT;

	/* Delete update timer */
	HW_TS_Stop(timer_id);
	HW_TS_Delete(timer_id);

    /* Disconnect from the device if desired */
    UTIL_SEQ_SetTask(1 << CFG_TASK_DISCONN_DEV_1_ID, CFG_SCH_PRIO_0);
}

bool FS_ActiveLook_IsRunning(void)
{
    return s_running;
}
