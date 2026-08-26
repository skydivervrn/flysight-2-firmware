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
**  This program is distributed in the hope that it will be useful,       **
**  but WITHOUT ANY WARRANTY; without even the implied warranty of        **
**  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         **
**  GNU General Public License for more details.                          **
**                                                                        **
**  You should have received a copy of the GNU General Public License     **
**  along with this program.  If not, see <http://www.gnu.org/licenses/>. **
**                                                                        **
****************************************************************************
**  Contact: Bionic Avionics Inc.                                         **
**  Website: http://flysight.ca/                                          **
****************************************************************************/

#include "activelook_client.h"
#include "activelook.h"
#include "activelook_proto.h"
#include "app_common.h"
#include "log.h"
#include "dbg_trace.h"
#include "ble.h"
#include "tl.h"
#include <string.h>

#ifndef UNPACK_2_BYTE_PARAMETER
#define UNPACK_2_BYTE_PARAMETER(ptr) \
    (uint16_t)( ((uint16_t)(*((uint8_t *)(ptr)))) \
              | ((uint16_t)(*(((uint8_t *)(ptr))+1)) << 8U) )
#endif

/* ActiveLook Commands service UUID
 * 0783B03E-8535-B5A0-7140-A304D2495CB7 */
static const uint8_t ACTIVELK_SERVICE_UUID[16] =
{
  0xB7, 0x5C, 0x49, 0xD2,
  0x04, 0xA3, 0x40, 0x71,
  0xA0, 0xB5, 0x35, 0x85,
  0x3E, 0xB0, 0x83, 0x07
};

/* Rx characteristic UUID (write commands to glasses)
 * 0783B03E-8535-B5A0-7140-A304D2495CBA */
static const uint8_t ACTIVELK_RX_CHAR_UUID[16] =
{
  0xBA, 0x5C, 0x49, 0xD2,
  0x04, 0xA3, 0x40, 0x71,
  0xA0, 0xB5, 0x35, 0x85,
  0x3E, 0xB0, 0x83, 0x07
};

/* TX notify characteristic UUID (glasses → phone responses/acks)
 * 0783B03E-8535-B5A0-7140-A304D2495CB8 */
static const uint8_t ACTIVELK_TX_CHAR_UUID[16] =
{
  0xB8, 0x5C, 0x49, 0xD2,
  0x04, 0xA3, 0x40, 0x71,
  0xA0, 0xB5, 0x35, 0x85,
  0x3E, 0xB0, 0x83, 0x07
};

/* Control / flow-control characteristic UUID
 * 0783B03E-8535-B5A0-7140-A304D2495CB9 */
static const uint8_t ACTIVELK_CTRL_CHAR_UUID[16] =
{
  0xB9, 0x5C, 0x49, 0xD2,
  0x04, 0xA3, 0x40, 0x71,
  0xA0, 0xB5, 0x35, 0x85,
  0x3E, 0xB0, 0x83, 0x07
};

/* Discovery states — one state per descriptor-discovery or CCCD-write step.
 * Flow: EXCH_MTU → SVC → CHAR
 *       → BAT_DESC → BAT_CCCD_WRITE
 *       → TX_DESC  → TX_CCCD_WRITE
 *       → CTRL_DESC → CTRL_CCCD_WRITE
 *       → complete (IDLE, linkUp=true, callback)
 * Steps are skipped when the corresponding char was not found. */
typedef enum {
    DISC_STATE_IDLE = 0,
    DISC_STATE_EXCH_MTU,
    DISC_STATE_SVC_IN_PROGRESS,
    DISC_STATE_CHAR_IN_PROGRESS,
    DISC_STATE_DESC_IN_PROGRESS,        /* battery CCCD discovery  */
    DISC_STATE_BATTERY_NOTIFY_WRITE,    /* battery CCCD write      */
    DISC_STATE_TX_DESC_IN_PROGRESS,     /* TX (CB8) CCCD discovery */
    DISC_STATE_TX_NOTIFY_WRITE,         /* TX (CB8) CCCD write     */
    DISC_STATE_CTRL_DESC_IN_PROGRESS,   /* Ctrl (CB9) CCCD discov. */
    DISC_STATE_CTRL_NOTIFY_WRITE        /* Ctrl (CB9) CCCD write   */
} DiscoveryState_t;

/* We define a separate enum to track which service we are scanning for chars. */
typedef enum {
    SERVICE_NONE = 0,
    SERVICE_ACTIVELOOK,
    SERVICE_BATTERY
} WhichService_t;

typedef struct
{
    uint16_t connHandle;
    DiscoveryState_t discState;

    /* ActiveLook service discovery */
    uint8_t  serviceFound;
    uint16_t serviceStartHandle;
    uint16_t serviceEndHandle;

    uint8_t  rxCharFound;
    uint16_t rxCharHandle;

    /* Store battery service info */
    uint8_t  batteryServiceFound;
    uint16_t batteryServiceStartHandle;
    uint16_t batteryServiceEndHandle;

    uint8_t  batteryCharFound;
    uint16_t batteryCharHandle;
    uint8_t  lastBatteryPercent; /* store the last known battery percent */

    /* CCC descriptor for the battery char */
    uint16_t batteryCCCDHandle;

    /* TX notify characteristic (glasses → phone, ...CB8) */
    uint8_t  txCharFound;
    uint16_t txCharHandle;
    uint16_t txCCCDHandle;

    /* Control / flow-control characteristic (...CB9) */
    uint8_t  ctrlCharFound;
    uint8_t  ctrlSubscribeStarted; /* CB9 CCCD write was issued... */
    uint8_t  ctrlSubscribed;       /* ...and the procedure came back clean */
    uint16_t ctrlCharHandle;
    uint16_t ctrlCCCDHandle;

    /* Flow-control state */
    uint8_t  lastCtrlByte;  /* last FLOW byte on CB9 (0x01/0x02 only; error codes
                               0x03/0x04/0x06 are logged and do NOT change this) */
    bool     linkUp;        /* true after full discovery + CCCD writes complete */
    uint32_t stopSinceTick; /* HAL_GetTick() when STOP was asserted; 0 = not stopped */
    uint16_t writeFailStreak; /* consecutive failed WWR calls; reset on success */

    /* Negotiated ATT MTU (default BLE minimum until exchanged) */
    uint16_t negotiatedMTU;

    /* Keep track of which service we are currently discovering chars for */
    WhichService_t whichService;

    /* Track which handle we requested to read, so we know how to interpret the response. */
    uint16_t currentReadHandle;

    /* Callback interface if set */
    const FS_ActiveLook_ClientCb_t *cb;
} FS_ActiveLook_Client_Context_t;

static FS_ActiveLook_Client_Context_t g_ctx;

/******************************************************************************
 * Initialize
 ******************************************************************************/
void FS_ActiveLook_Client_Init(void)
{
    memset(&g_ctx, 0, sizeof(g_ctx));
    g_ctx.discState      = DISC_STATE_IDLE;
    g_ctx.negotiatedMTU  = 23;  /* BLE default ATT_MTU */
    g_ctx.lastBatteryPercent = 255;  /* 255 = unknown */
}

/******************************************************************************
 * Register optional callback interface
 ******************************************************************************/
void FS_ActiveLook_Client_RegisterCb(const FS_ActiveLook_ClientCb_t *cb)
{
    g_ctx.cb = cb;
}

/******************************************************************************
 * Start discovery (including MTU exchange) after connecting
 ******************************************************************************/
void FS_ActiveLook_Client_StartDiscovery(uint16_t connectionHandle)
{
    g_ctx.connHandle = connectionHandle;
    g_ctx.discState  = DISC_STATE_EXCH_MTU;  /* Start with MTU exchange */

    /* Clear everything so we re-discover from scratch */
    g_ctx.serviceFound          = 0;
    g_ctx.serviceStartHandle    = 0;
    g_ctx.serviceEndHandle      = 0;
    g_ctx.rxCharFound           = 0;
    g_ctx.rxCharHandle          = 0;

    g_ctx.batteryServiceFound   = 0;
    g_ctx.batteryServiceStartHandle = 0;
    g_ctx.batteryServiceEndHandle   = 0;
    g_ctx.batteryCharFound      = 0;
    g_ctx.batteryCharHandle     = 0;
    g_ctx.batteryCCCDHandle     = 0;
    g_ctx.lastBatteryPercent    = 255; /* 255 = unknown */

    g_ctx.txCharFound           = 0;
    g_ctx.txCharHandle          = 0;
    g_ctx.txCCCDHandle          = 0;

    g_ctx.ctrlCharFound         = 0;
    g_ctx.ctrlSubscribeStarted  = 0;
    g_ctx.ctrlSubscribed        = 0;
    g_ctx.ctrlCharHandle        = 0;
    g_ctx.ctrlCCCDHandle        = 0;

    /* Send-by-default once linkUp: ActiveLook glasses don't necessarily push an
     * initial 0x01 on CB9, so assume OK and only pause when they send 0x02 (STOP). */
    g_ctx.lastCtrlByte          = AL_FLOW_OK;
    g_ctx.linkUp                = false;
    g_ctx.stopSinceTick         = 0;
    g_ctx.writeFailStreak       = 0;
    g_ctx.negotiatedMTU         = 23;  /* reset to default; updated on MTU resp */

    g_ctx.whichService = SERVICE_NONE;

    /* Step 1: request a bigger ATT MTU from the peripheral */
    tBleStatus s = aci_gatt_exchange_config(connectionHandle);
    if (s == BLE_STATUS_SUCCESS)
    {
        APP_DBG_MSG("ActiveLook_Client: Requesting MTU exchange...\n");
    }
    else
    {
        APP_DBG_MSG("ActiveLook_Client: aci_gatt_exchange_config fail=0x%02X\n", s);
        FS_Log_WriteEventAsync("ENGO GATT setup failed: MTU req 0x%02X", s);
        g_ctx.discState = DISC_STATE_IDLE;  // stop
    }
}

/******************************************************************************
 * Event Handler
 ******************************************************************************/
void FS_ActiveLook_Client_EventHandler(void *p_blecore_evt, uint8_t hci_event_evt_code)
{
    evt_blecore_aci *blecore_evt = (evt_blecore_aci*) p_blecore_evt;

    switch (blecore_evt->ecode)
    {
        /**********************************************************************
         * The peripheral responded to MTU exchange:
         *********************************************************************/
        case ACI_ATT_EXCHANGE_MTU_RESP_VSEVT_CODE:
        {
            /* This event indicates the peripheral accepted some MTU.  */
            aci_att_exchange_mtu_resp_event_rp0 *mtu_resp =
                (aci_att_exchange_mtu_resp_event_rp0*) blecore_evt->data;
            /* Store the negotiated MTU so callers can avoid oversized writes. */
            g_ctx.negotiatedMTU = mtu_resp->Server_RX_MTU;
            APP_DBG_MSG("ActiveLook_Client: ACI_ATT_EXCHANGE_MTU_RESP, final MTU=%d\r\n",
                        mtu_resp->Server_RX_MTU);
            /* Wait for ACI_GATT_PROC_COMPLETE_VSEVT_CODE to know the procedure is done. */
        }
        break;

        /**********************************************************************
         * GATT procedure complete => check if we were in MTU exchange, or
         * discovering service, or discovering char, etc.
         *********************************************************************/
        case ACI_GATT_PROC_COMPLETE_VSEVT_CODE:
        {
            aci_gatt_proc_complete_event_rp0 *pc =
                (aci_gatt_proc_complete_event_rp0*) blecore_evt->data;

            if (pc->Connection_Handle != g_ctx.connHandle)
                break; /* Not for us */

            if (g_ctx.discState == DISC_STATE_EXCH_MTU)
            {
                /* Done exchanging MTU => discover all primary services */
                g_ctx.discState = DISC_STATE_SVC_IN_PROGRESS;
                tBleStatus s = aci_gatt_disc_all_primary_services(g_ctx.connHandle);
                if (s == BLE_STATUS_SUCCESS)
                {
                    APP_DBG_MSG("ActiveLook_Client: MTU ok, now discovering all services...\n");
                }
                else
                {
                    APP_DBG_MSG("ActiveLook_Client: disc_all_primary_services fail=0x%02X\n", s);
                    FS_Log_WriteEventAsync("ENGO GATT setup failed: svc disc 0x%02X", s);
                    g_ctx.discState = DISC_STATE_IDLE;
                }
            }
            else if (g_ctx.discState == DISC_STATE_SVC_IN_PROGRESS)
            {
                /* Done discovering all services. Next, discover chars. */
                if (g_ctx.serviceFound)
                {
                    g_ctx.discState    = DISC_STATE_CHAR_IN_PROGRESS;
                    g_ctx.whichService = SERVICE_ACTIVELOOK;
                    tBleStatus s = aci_gatt_disc_all_char_of_service(
                                       g_ctx.connHandle,
                                       g_ctx.serviceStartHandle,
                                       g_ctx.serviceEndHandle);
                    if (s == BLE_STATUS_SUCCESS)
                    {
                        APP_DBG_MSG("ActiveLook_Client: Discovering chars in ActiveLook service...\n");
                    }
                    else
                    {
                        APP_DBG_MSG("ActiveLook_Client: disc_all_char_of_service fail=0x%02X\n", s);
                        FS_Log_WriteEventAsync("ENGO GATT setup failed: char disc 0x%02X", s);
                        g_ctx.discState = DISC_STATE_IDLE;
                    }
                }
                else if (g_ctx.batteryServiceFound)
                {
                    g_ctx.discState    = DISC_STATE_CHAR_IN_PROGRESS;
                    g_ctx.whichService = SERVICE_BATTERY;
                    tBleStatus s = aci_gatt_disc_all_char_of_service(
                                       g_ctx.connHandle,
                                       g_ctx.batteryServiceStartHandle,
                                       g_ctx.batteryServiceEndHandle);
                    if (s == BLE_STATUS_SUCCESS)
                    {
                        APP_DBG_MSG("ActiveLook_Client: No AL service, but battery found; discovering battery char...\n");
                    }
                    else
                    {
                        APP_DBG_MSG("ActiveLook_Client: disc_all_char_of_service(battery) fail=0x%02X\n", s);
                        g_ctx.discState = DISC_STATE_IDLE;
                    }
                }
                else
                {
                    APP_DBG_MSG("ActiveLook_Client: No known services found.\n");
                    FS_Log_WriteEventAsync("ENGO GATT setup failed: ActiveLook service not found");
                    g_ctx.discState = DISC_STATE_IDLE;
                }
            }
            else if (g_ctx.discState == DISC_STATE_CHAR_IN_PROGRESS)
            {
                /* Done discovering chars for whichever service we were on. */
                if (g_ctx.whichService == SERVICE_ACTIVELOOK)
                {
                    /* If battery service also found, discover battery chars now */
                    if (g_ctx.batteryServiceFound)
                    {
                        g_ctx.whichService = SERVICE_BATTERY;
                        tBleStatus s = aci_gatt_disc_all_char_of_service(
                                           g_ctx.connHandle,
                                           g_ctx.batteryServiceStartHandle,
                                           g_ctx.batteryServiceEndHandle);
                        if (s == BLE_STATUS_SUCCESS)
                        {
                            APP_DBG_MSG("ActiveLook_Client: Now discovering battery char...\n");
                            return; /* Wait next event for the battery char discovery response */
                        }
                        else
                        {
                            APP_DBG_MSG("ActiveLook_Client: disc_all_char_of_service(battery) fail=0x%02X\n", s);
                        }
                    }
                }

                APP_DBG_MSG("ActiveLook_Client: Char discovery complete. Rx=0x%04X Tx=0x%04X Ctrl=0x%04X Bat=0x%04X\n",
                            g_ctx.rxCharHandle, g_ctx.txCharHandle,
                            g_ctx.ctrlCharHandle, g_ctx.batteryCharHandle);

                /* Step into descriptor discovery pipeline: battery first. */
                if (g_ctx.batteryCharFound && (g_ctx.batteryCharHandle != 0))
                {
                    g_ctx.discState = DISC_STATE_DESC_IN_PROGRESS;
                    tBleStatus s = aci_gatt_disc_all_char_desc(
                                       g_ctx.connHandle,
                                       g_ctx.batteryCharHandle,
                                       g_ctx.batteryCharHandle + 2);
                    if (s == BLE_STATUS_SUCCESS)
                    {
                        APP_DBG_MSG("ActiveLook_Client: Discovering battery descriptors...\n");
                        return;
                    }
                    APP_DBG_MSG("ActiveLook_Client: disc_all_char_desc(battery) fail=0x%02X\n", s);
                    g_ctx.discState = DISC_STATE_CHAR_IN_PROGRESS; /* fall through below */
                }

                /* No battery, or battery desc discovery failed — jump to TX desc step. */
                if (g_ctx.txCharFound && (g_ctx.txCharHandle != 0))
                {
                    g_ctx.discState = DISC_STATE_TX_DESC_IN_PROGRESS;
                    tBleStatus s = aci_gatt_disc_all_char_desc(
                                       g_ctx.connHandle,
                                       g_ctx.txCharHandle,
                                       g_ctx.txCharHandle + 2);
                    if (s == BLE_STATUS_SUCCESS)
                    {
                        APP_DBG_MSG("ActiveLook_Client: Discovering TX descriptors...\n");
                        return;
                    }
                    APP_DBG_MSG("ActiveLook_Client: disc_all_char_desc(tx) fail=0x%02X\n", s);
                }

                /* No TX, or TX desc discovery failed — jump to Ctrl desc step. */
                if (g_ctx.ctrlCharFound && (g_ctx.ctrlCharHandle != 0))
                {
                    g_ctx.discState = DISC_STATE_CTRL_DESC_IN_PROGRESS;
                    tBleStatus s = aci_gatt_disc_all_char_desc(
                                       g_ctx.connHandle,
                                       g_ctx.ctrlCharHandle,
                                       g_ctx.ctrlCharHandle + 2);
                    if (s == BLE_STATUS_SUCCESS)
                    {
                        APP_DBG_MSG("ActiveLook_Client: Discovering Ctrl descriptors...\n");
                        return;
                    }
                    APP_DBG_MSG("ActiveLook_Client: disc_all_char_desc(ctrl) fail=0x%02X\n", s);
                }

                /* Nothing to discover — complete now. */
                g_ctx.discState = DISC_STATE_IDLE;
                g_ctx.linkUp    = true;
                APP_DBG_MSG("ActiveLook_Client: Discovery complete (no CCCDs to write).\n");
                if (g_ctx.rxCharFound && g_ctx.cb && g_ctx.cb->OnDiscoveryComplete)
                {
                    g_ctx.cb->OnDiscoveryComplete();
                }
            }
            else if (g_ctx.discState == DISC_STATE_DESC_IN_PROGRESS)
            {
                /* Done discovering battery descriptors. Write battery CCCD if found. */
                APP_DBG_MSG("ActiveLook_Client: Battery descriptor discovery complete.\n");

                if (g_ctx.batteryCCCDHandle != 0)
                {
                    tBleStatus s2 = FS_ActiveLook_Client_EnableBatteryNotifications();
                    APP_DBG_MSG("ActiveLook_Client: EnableBatteryNotifications => 0x%02X\n", s2);
                    if (s2 == BLE_STATUS_SUCCESS)
                    {
                        g_ctx.discState = DISC_STATE_BATTERY_NOTIFY_WRITE;
                        return;
                    }
                }
                /* Battery CCCD not found or write failed — fall through to TX desc step
                 * by setting state and letting the shared advance logic below run. */
                g_ctx.discState = DISC_STATE_BATTERY_NOTIFY_WRITE; /* treated as already done */
            }

            /* --- Shared advance logic: runs when an individual step completes and we
             *     need to move to the next one.  Each branch either returns (step issued)
             *     or falls through to the next check. --- */

            if (g_ctx.discState == DISC_STATE_BATTERY_NOTIFY_WRITE)
            {
                /* Battery CCCD write complete (or was skipped). Issue immediate battery
                 * read if applicable, then advance to TX descriptor discovery. */
                if (g_ctx.batteryCharFound && (g_ctx.batteryCharHandle != 0)
                    && (g_ctx.batteryCCCDHandle != 0))
                {
                    /* Only read if we actually wrote the CCCD. */
                    g_ctx.currentReadHandle = g_ctx.batteryCharHandle;
                    tBleStatus rs = aci_gatt_read_char_value(g_ctx.connHandle,
                                                             g_ctx.batteryCharHandle);
                    if (rs == BLE_STATUS_SUCCESS)
                    {
                        APP_DBG_MSG("ActiveLook_Client: Requesting immediate battery read...\n");
                    }
                    else
                    {
                        APP_DBG_MSG("ActiveLook_Client: Battery read failed => 0x%02X\n", rs);
                        g_ctx.currentReadHandle = 0;
                    }
                }

                if (g_ctx.txCharFound && (g_ctx.txCharHandle != 0))
                {
                    tBleStatus s = aci_gatt_disc_all_char_desc(
                                       g_ctx.connHandle,
                                       g_ctx.txCharHandle,
                                       g_ctx.txCharHandle + 2);
                    if (s == BLE_STATUS_SUCCESS)
                    {
                        g_ctx.discState = DISC_STATE_TX_DESC_IN_PROGRESS;
                        APP_DBG_MSG("ActiveLook_Client: Discovering TX descriptors...\n");
                        return;
                    }
                    APP_DBG_MSG("ActiveLook_Client: disc_all_char_desc(tx) fail=0x%02X\n", s);
                }
                g_ctx.discState = DISC_STATE_TX_NOTIFY_WRITE; /* skip TX step */
            }

            if (g_ctx.discState == DISC_STATE_TX_DESC_IN_PROGRESS)
            {
                /* Done discovering TX descriptors. Write TX CCCD if found. */
                APP_DBG_MSG("ActiveLook_Client: TX descriptor discovery complete.\n");

                if (g_ctx.txCCCDHandle != 0)
                {
                    uint16_t enable = 0x0001;
                    tBleStatus s = aci_gatt_write_char_desc(
                                       g_ctx.connHandle,
                                       g_ctx.txCCCDHandle,
                                       2,
                                       (uint8_t*)&enable);
                    APP_DBG_MSG("ActiveLook_Client: Enable TX notify => 0x%02X\n", s);
                    if (s == BLE_STATUS_SUCCESS)
                    {
                        g_ctx.discState = DISC_STATE_TX_NOTIFY_WRITE;
                        return;
                    }
                }
                g_ctx.discState = DISC_STATE_TX_NOTIFY_WRITE; /* skip / write failed */
            }

            if (g_ctx.discState == DISC_STATE_TX_NOTIFY_WRITE)
            {
                /* TX CCCD write complete (or skipped). Advance to Ctrl desc discovery. */
                APP_DBG_MSG("ActiveLook_Client: TX step done, trying Ctrl desc...\n");

                if (g_ctx.ctrlCharFound && (g_ctx.ctrlCharHandle != 0))
                {
                    tBleStatus s = aci_gatt_disc_all_char_desc(
                                       g_ctx.connHandle,
                                       g_ctx.ctrlCharHandle,
                                       g_ctx.ctrlCharHandle + 2);
                    if (s == BLE_STATUS_SUCCESS)
                    {
                        g_ctx.discState = DISC_STATE_CTRL_DESC_IN_PROGRESS;
                        APP_DBG_MSG("ActiveLook_Client: Discovering Ctrl descriptors...\n");
                        return;
                    }
                    APP_DBG_MSG("ActiveLook_Client: disc_all_char_desc(ctrl) fail=0x%02X\n", s);
                }
                g_ctx.discState = DISC_STATE_CTRL_NOTIFY_WRITE; /* skip Ctrl step */
            }

            if (g_ctx.discState == DISC_STATE_CTRL_DESC_IN_PROGRESS)
            {
                /* Done discovering Ctrl descriptors. Write Ctrl CCCD if found. */
                APP_DBG_MSG("ActiveLook_Client: Ctrl descriptor discovery complete.\n");

                if (g_ctx.ctrlCCCDHandle != 0)
                {
                    uint16_t enable = 0x0001;
                    tBleStatus s = aci_gatt_write_char_desc(
                                       g_ctx.connHandle,
                                       g_ctx.ctrlCCCDHandle,
                                       2,
                                       (uint8_t*)&enable);
                    APP_DBG_MSG("ActiveLook_Client: Enable Ctrl notify => 0x%02X\n", s);
                    if (s == BLE_STATUS_SUCCESS)
                    {
                        g_ctx.ctrlSubscribeStarted = 1;
                        g_ctx.discState = DISC_STATE_CTRL_NOTIFY_WRITE;
                        return;
                    }
                }
                g_ctx.discState = DISC_STATE_CTRL_NOTIFY_WRITE; /* skip / write failed */
            }

            if (g_ctx.discState == DISC_STATE_CTRL_NOTIFY_WRITE)
            {
                /* CB9 carries the glasses' STOP. Without it we would push
                 * frames at a renderer that has no way of telling us to wait,
                 * and over-driving that buffer is exactly what freezes their
                 * display while BLE stays up — the failure this whole layer is
                 * built to avoid. So a link whose flow control did not
                 * subscribe is not a link we bring up: log it, terminate, and
                 * let the rescan try again.
                 *
                 * Deliberate behaviour change: glasses that expose no CB9
                 * descriptor at all now fail to connect instead of running
                 * blind. That is a visible, diagnosable failure rather than a
                 * display that freezes in the air. */
                if (g_ctx.ctrlSubscribeStarted && pc->Error_Code == 0)
                {
                    g_ctx.ctrlSubscribed = 1;
                }

                if (!g_ctx.ctrlSubscribed)
                {
                    FS_Log_WriteEventAsync(
                        "ENGO GATT setup failed: no CB9 flow control (started=%u err=0x%02X)",
                        (unsigned)g_ctx.ctrlSubscribeStarted,
                        (unsigned)pc->Error_Code);
                    APP_DBG_MSG("ActiveLook_Client: no CB9 subscription — dropping link\n");
                    g_ctx.discState = DISC_STATE_IDLE;
                    FS_ActiveLook_Client_ForceDisconnect();
                    return;
                }

                g_ctx.discState = DISC_STATE_IDLE;
                g_ctx.linkUp    = true;
                APP_DBG_MSG("ActiveLook_Client: Full discovery done. Rx=0x%04X linkUp=true\n",
                            g_ctx.rxCharHandle);
                if (g_ctx.rxCharFound && g_ctx.cb && g_ctx.cb->OnDiscoveryComplete)
                {
                    g_ctx.cb->OnDiscoveryComplete();
                }
            }
        }
        break;

        /**********************************************************************
         * "Read by group type" => primary service
         *********************************************************************/
        case ACI_ATT_READ_BY_GROUP_TYPE_RESP_VSEVT_CODE:
        {
            if (g_ctx.discState == DISC_STATE_SVC_IN_PROGRESS)
            {
                aci_att_read_by_group_type_resp_event_rp0 *pr =
                    (aci_att_read_by_group_type_resp_event_rp0*) blecore_evt->data;

                uint8_t idx = 0;
                while (idx < pr->Data_Length)
                {
                    uint16_t startHdl = UNPACK_2_BYTE_PARAMETER(&pr->Attribute_Data_List[idx]);
                    uint16_t endHdl   = UNPACK_2_BYTE_PARAMETER(&pr->Attribute_Data_List[idx+2]);
                    const uint8_t *uuid = &pr->Attribute_Data_List[idx+4];

                    /* The length is pr->Attribute_Data_Length for each record. */
                    /* If it’s 6, the UUID is 16 bits. If it’s 20, the UUID is 128 bits. */

                    if (pr->Attribute_Data_Length == 20)
                    {
                        /* 128-bit UUID */
                        if (memcmp(uuid, ACTIVELK_SERVICE_UUID, 16) == 0)
                        {
                            APP_DBG_MSG("ActiveLook_Client: Found ActiveLook Service 0x%04X–0x%04X\n", startHdl, endHdl);
                            g_ctx.serviceFound       = 1;
                            g_ctx.serviceStartHandle = startHdl;
                            g_ctx.serviceEndHandle   = endHdl;
                        }
                    }
                    else if (pr->Attribute_Data_Length == 6)
                    {
                        /* 16-bit UUID => check if 0x180F (Battery) */
                        uint16_t short_uuid = UNPACK_2_BYTE_PARAMETER(uuid);
                        if (short_uuid == BATTERY_SERVICE_UUID)
                        {
                            APP_DBG_MSG("ActiveLook_Client: Found Battery Service 0x%04X–0x%04X\n", startHdl, endHdl);
                            g_ctx.batteryServiceFound       = 1;
                            g_ctx.batteryServiceStartHandle = startHdl;
                            g_ctx.batteryServiceEndHandle   = endHdl;
                        }
                    }

                    idx += pr->Attribute_Data_Length;
                }
            }
        }
        break;

        /**********************************************************************
         * "Read by type response" => characteristics
         *********************************************************************/
        case ACI_ATT_READ_BY_TYPE_RESP_VSEVT_CODE:
        {
            if (g_ctx.discState == DISC_STATE_CHAR_IN_PROGRESS)
            {
                aci_att_read_by_type_resp_event_rp0 *pr =
                    (aci_att_read_by_type_resp_event_rp0*) blecore_evt->data;
                uint8_t idx = 0;
                while (idx < pr->Data_Length)
                {
                    /* typical format: 2B decl handle, 1B props, 2B value handle, then the UUID */
                    uint16_t declHandle = UNPACK_2_BYTE_PARAMETER(&pr->Handle_Value_Pair_Data[idx]);
                    idx += 2;

                    uint8_t properties = pr->Handle_Value_Pair_Data[idx++];
                    uint16_t valHandle = UNPACK_2_BYTE_PARAMETER(&pr->Handle_Value_Pair_Data[idx]);
                    idx += 2;

                    /* length of the UUID depends on pr->Handle_Value_Pair_Length */
                    uint8_t uuidLen = pr->Handle_Value_Pair_Length - 5; // we used up 5 bytes so far
                    const uint8_t *uuid = &pr->Handle_Value_Pair_Data[idx];
                    idx += uuidLen;

                    if (g_ctx.whichService == SERVICE_ACTIVELOOK)
                    {
                        if (uuidLen == 16)
                        {
                            /* Rx (write commands) ...CBA */
                            if (memcmp(uuid, ACTIVELK_RX_CHAR_UUID, 16) == 0)
                            {
                                APP_DBG_MSG("ActiveLook_Client: Found RxChar=0x%04X\n", valHandle);
                                g_ctx.rxCharFound  = 1;
                                g_ctx.rxCharHandle = valHandle;
                            }
                            /* TX notify (responses) ...CB8 */
                            else if (memcmp(uuid, ACTIVELK_TX_CHAR_UUID, 16) == 0)
                            {
                                APP_DBG_MSG("ActiveLook_Client: Found TxChar=0x%04X\n", valHandle);
                                g_ctx.txCharFound  = 1;
                                g_ctx.txCharHandle = valHandle;
                            }
                            /* Control / flow-control ...CB9 */
                            else if (memcmp(uuid, ACTIVELK_CTRL_CHAR_UUID, 16) == 0)
                            {
                                APP_DBG_MSG("ActiveLook_Client: Found CtrlChar=0x%04X\n", valHandle);
                                g_ctx.ctrlCharFound  = 1;
                                g_ctx.ctrlCharHandle = valHandle;
                            }
                        }
                    }
                    else if (g_ctx.whichService == SERVICE_BATTERY)
                    {
                        /* Compare to the known 16-bit Battery Level char UUID=0x2A19 */
                        if (uuidLen == 2)
                        {
                            uint16_t short_uuid = UNPACK_2_BYTE_PARAMETER(uuid);
                            if (short_uuid == BATTERY_LEVEL_CHAR_UUID)
                            {
                                APP_DBG_MSG("ActiveLook_Client: Found BatteryChar=0x%04X\n", valHandle);
                                g_ctx.batteryCharFound  = 1;
                                g_ctx.batteryCharHandle = valHandle;
                            }
                        }
                    }
                }
            }
        }
        break;

        /* Listen for notifications (including Battery Level) */
        case ACI_GATT_NOTIFICATION_VSEVT_CODE:
        {
            aci_gatt_notification_event_rp0 *pNotif =
                (aci_gatt_notification_event_rp0*) blecore_evt->data;

            if (pNotif->Connection_Handle == g_ctx.connHandle)
            {
                /* Battery Level characteristic (0x2A19) — 1-byte [0..100]. */
                if (pNotif->Attribute_Handle == g_ctx.batteryCharHandle)
                {
                    if (pNotif->Attribute_Value_Length >= 1)
                    {
                        g_ctx.lastBatteryPercent = pNotif->Attribute_Value[0];
                        APP_DBG_MSG("ActiveLook_Client: Battery notification => %d%%\n", g_ctx.lastBatteryPercent);
                    }
                }
                /* Control characteristic (...CB9). Per the ActiveLook API this
                 * carries BOTH flow control (0x01 resume / 0x02 stop) AND async
                 * error codes (0x03 corrupt cmd, 0x04 RX-queue overflow, 0x06
                 * missing cfgWrite). Only 0x01/0x02 may touch lastCtrlByte —
                 * an error byte previously clobbered it and, since CanSend()
                 * requires ==0x01, silenced ALL sends until the next 0x01
                 * (which may never come) => HUD frozen while BLE stays up. */
                else if (g_ctx.ctrlCharFound &&
                         (pNotif->Attribute_Handle == g_ctx.ctrlCharHandle) &&
                         (pNotif->Attribute_Value_Length >= 1))
                {
                    uint8_t v = pNotif->Attribute_Value[0];
                    APP_DBG_MSG("ActiveLook_Client: Ctrl notification => 0x%02X\n", v);
                    switch (v)
                    {
                    case AL_FLOW_OK:
                        g_ctx.lastCtrlByte  = AL_FLOW_OK;
                        g_ctx.stopSinceTick = 0;
                        /* Tell the app the gate opened. Without this the resume
                         * changed a variable and woke nothing, so a frame that
                         * STOP had paused mid-way waited for the next update
                         * tick — and that tick skips entirely when no value
                         * changed, leaving the glasses held on a stale screen.
                         * Same direction of call as OnDiscoveryComplete above:
                         * the client notifies, the app decides. */
                        FS_ActiveLook_OnFlowResume();
                        break;
                    case AL_FLOW_STOP:
                        if (g_ctx.lastCtrlByte != AL_FLOW_STOP)
                            g_ctx.stopSinceTick = HAL_GetTick();
                        g_ctx.lastCtrlByte = AL_FLOW_STOP;
                        break;
                    case 0x03:
                        FS_Log_WriteEventAsync("ENGO ctrl: corrupt command (0x03)");
                        break;
                    case 0x04:
                        FS_Log_WriteEventAsync("ENGO ctrl: RX queue overflow (0x04)");
                        break;
                    case 0x06:
                        FS_Log_WriteEventAsync("ENGO ctrl: missing cfgWrite (0x06)");
                        break;
                    default:
                        FS_Log_WriteEventAsync("ENGO ctrl: unknown code 0x%02X", v);
                        break;
                    }
                }
                /* TX notify characteristic (...CB8) — command responses/acks.
                   Logged here; upper layers may parse if needed. */
                else if (g_ctx.txCharFound &&
                         (pNotif->Attribute_Handle == g_ctx.txCharHandle))
                {
                    APP_DBG_MSG("ActiveLook_Client: TX notification len=%d\n",
                                pNotif->Attribute_Value_Length);
                }
            }
        }
        break;

        case ACI_ATT_FIND_INFO_RESP_VSEVT_CODE:
        {
            /* Triggered during aci_gatt_disc_all_char_desc — dispatch based on
             * which char's descriptors we are currently discovering. */
            DiscoveryState_t ds = g_ctx.discState;
            if (ds == DISC_STATE_DESC_IN_PROGRESS ||
                ds == DISC_STATE_TX_DESC_IN_PROGRESS ||
                ds == DISC_STATE_CTRL_DESC_IN_PROGRESS)
            {
                aci_att_find_info_resp_event_rp0 *resp =
                    (aci_att_find_info_resp_event_rp0 *)blecore_evt->data;

                /* Format=0x01 => 16-bit UUIDs (the common case for CCCD 0x2902) */
                if (resp->Format == 0x01)
                {
                    /* Each entry: 2B handle + 2B UUID */
                    uint8_t numDesc = (resp->Event_Data_Length - 1) / 4;
                    uint8_t *ptr    = resp->Handle_UUID_Pair;
                    for (uint8_t i = 0; i < numDesc; i++)
                    {
                        uint16_t descHandle = UNPACK_2_BYTE_PARAMETER(ptr);
                        ptr += 2;
                        uint16_t descUUID   = UNPACK_2_BYTE_PARAMETER(ptr);
                        ptr += 2;

                        if (descUUID == 0x2902) /* Client Characteristic Configuration */
                        {
                            if (ds == DISC_STATE_DESC_IN_PROGRESS)
                            {
                                g_ctx.batteryCCCDHandle = descHandle;
                                APP_DBG_MSG("ActiveLook_Client: Found Battery CCCD=0x%04X\n", descHandle);
                            }
                            else if (ds == DISC_STATE_TX_DESC_IN_PROGRESS)
                            {
                                g_ctx.txCCCDHandle = descHandle;
                                APP_DBG_MSG("ActiveLook_Client: Found TX CCCD=0x%04X\n", descHandle);
                            }
                            else if (ds == DISC_STATE_CTRL_DESC_IN_PROGRESS)
                            {
                                g_ctx.ctrlCCCDHandle = descHandle;
                                APP_DBG_MSG("ActiveLook_Client: Found Ctrl CCCD=0x%04X\n", descHandle);
                            }
                        }
                    }
                }
                /* Format=0x02 => 128-bit UUIDs — not typical for CCCD, ignore. */
            }
        }
        break;

        case ACI_ATT_READ_RESP_VSEVT_CODE:
        {
            aci_att_read_resp_event_rp0 *pRead =
                (aci_att_read_resp_event_rp0*) blecore_evt->data;

            /* Check if it's our connection */
            if (pRead->Connection_Handle == g_ctx.connHandle)
            {
                /* If we had queued up a read to batteryCharHandle, interpret it now */
                if (g_ctx.currentReadHandle == g_ctx.batteryCharHandle)
                {
                    if (pRead->Event_Data_Length >= 1)
                    {
                        g_ctx.lastBatteryPercent = pRead->Attribute_Value[0];
                        APP_DBG_MSG("ActiveLook_Client: Immediate battery read => %d%%\n",
                                    g_ctx.lastBatteryPercent);
                    }
                }

                /* Reset it so we don't confuse subsequent read responses. */
                g_ctx.currentReadHandle = 0;
            }
        }
        break;

        default:
            break;
    }
}

/******************************************************************************
 * Check if Rx handle is ready (ActiveLook commands)
 ******************************************************************************/
uint8_t FS_ActiveLook_Client_IsReady(void)
{
    return (g_ctx.rxCharFound && g_ctx.rxCharHandle != 0);
}

/******************************************************************************
 * Write data to Rx characteristic (WWR)
 ******************************************************************************/
tBleStatus FS_ActiveLook_Client_WriteWithoutResp(const uint8_t *data, uint16_t length)
{
    if (!FS_ActiveLook_Client_IsReady())
    {
        APP_DBG_MSG("ActiveLook_Client: Not ready, no Rx handle.\n");
        return BLE_STATUS_FAILED;
    }

    tBleStatus s = aci_gatt_write_without_resp(g_ctx.connHandle,
                                               g_ctx.rxCharHandle,
                                               length,
                                               (uint8_t*)data);
    if (s != BLE_STATUS_SUCCESS)
    {
        APP_DBG_MSG("ActiveLook_Client: WWR error=0x%02X\n", s);
        if (g_ctx.writeFailStreak < 0xFFFF)
            g_ctx.writeFailStreak++;
    }
    else
    {
        g_ctx.writeFailStreak = 0;
    }
    return s;
}

/******************************************************************************
 * Self-heal helpers (glidex/official-SDK-informed watchdog inputs)
 ******************************************************************************/

/* How long the glasses have been asserting STOP, in ms (0 = not stopped). */
uint32_t FS_ActiveLook_Client_StopStuckMs(void)
{
    if (!g_ctx.linkUp || g_ctx.lastCtrlByte != AL_FLOW_STOP || g_ctx.stopSinceTick == 0)
        return 0;
    return HAL_GetTick() - g_ctx.stopSinceTick;
}

/* Consecutive failed writes since the last successful one. */
uint16_t FS_ActiveLook_Client_WriteFailStreak(void)
{
    return g_ctx.writeFailStreak;
}

/* Drop the link deliberately; the HCI disconnection event then runs the normal
 * reset + rescan path (app_ble.c), which also self-heals a wedged glasses link. */
void FS_ActiveLook_Client_ForceDisconnect(void)
{
    if (g_ctx.connHandle != 0xFFFF)
        (void)aci_gap_terminate(g_ctx.connHandle, 0x13); /* remote user terminated */
}

/******************************************************************************
 * Enable battery notifications
 ******************************************************************************/
tBleStatus FS_ActiveLook_Client_EnableBatteryNotifications(void)
{
    if (!g_ctx.batteryCharFound || (g_ctx.batteryCharHandle == 0))
    {
        APP_DBG_MSG("ActiveLook_Client: No Battery char found!\n");
        return BLE_STATUS_FAILED;
    }

    if (g_ctx.batteryCCCDHandle == 0)
    {
        APP_DBG_MSG("ActiveLook_Client: No Battery CCCD handle discovered!\n");
        return BLE_STATUS_FAILED;
    }

    /* The BLE spec for notifications: 0x0001 in little-endian => 0x01,0x00 */
    uint16_t enable = 0x0001;
    tBleStatus s = aci_gatt_write_char_desc(
                       g_ctx.connHandle,
                       g_ctx.batteryCCCDHandle,
                       2,
                       (uint8_t*)&enable);
    if (s == BLE_STATUS_SUCCESS)
    {
        APP_DBG_MSG("ActiveLook_Client: Battery notifications enabled.\n");
    }
    else
    {
        APP_DBG_MSG("ActiveLook_Client: EnableBatteryNotifications fail=0x%02X\n", s);
    }
    return s;
}

/******************************************************************************
 * Get the last known battery level
 ******************************************************************************/
uint8_t FS_ActiveLook_Client_GetBatteryLevel(void)
{
    return g_ctx.lastBatteryPercent; /* 0..100, or 255 if unknown */
}

/******************************************************************************
 * Reset all client state on disconnect (idempotent)
 ******************************************************************************/
void FS_ActiveLook_Client_OnDisconnect(void)
{
    /* Preserve the callback pointer so it survives re-connection. */
    const FS_ActiveLook_ClientCb_t *savedCb = g_ctx.cb;

    memset(&g_ctx, 0, sizeof(g_ctx));

    g_ctx.cb             = savedCb;
    g_ctx.discState      = DISC_STATE_IDLE;
    g_ctx.connHandle     = 0xFFFF;   /* invalid / no connection */
    g_ctx.rxCharFound    = 0;
    g_ctx.rxCharHandle   = 0;
    g_ctx.linkUp         = false;
    g_ctx.lastCtrlByte   = 0;        /* not-ok until first CB9 notification */
    g_ctx.stopSinceTick  = 0;
    g_ctx.writeFailStreak = 0;
    g_ctx.negotiatedMTU  = 23;       /* reset to BLE default */
    g_ctx.lastBatteryPercent = 255;  /* unknown */

    APP_DBG_MSG("ActiveLook_Client: OnDisconnect — state cleared.\n");
}

/******************************************************************************
 * Flow-control gate
 ******************************************************************************/
bool FS_ActiveLook_Client_CanSend(void)
{
    return AL_FlowCanSend(g_ctx.lastCtrlByte, g_ctx.linkUp);
}

/******************************************************************************
 * Return the negotiated ATT MTU
 ******************************************************************************/
uint16_t FS_ActiveLook_Client_GetMTU(void)
{
    return g_ctx.negotiatedMTU;
}
