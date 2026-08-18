/* USER CODE BEGIN Header */
/***************************************************************************
**                                                                        **
**  FlySight 2 firmware                                                   **
**  Copyright 2023 Bionic Avionics Inc.                                   **
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "app_common.h"

#include "dbg_trace.h"
#include "ble.h"
#include "tl.h"
#include "app_ble.h"

#include "stm32_seq.h"
#include "shci.h"
#include "stm32_lpm.h"
#include "otp.h"

#include "custom_app.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "common.h"
#include "state.h"
#include "activelook_client.h"
#include "activelook.h"
#include "ble_diag.h"
#include "config.h"
#include "engo_bind.h"
#include "log.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/

/**
 * security parameters structure
 */
typedef struct _tSecurityParams
{
  /**
   * IO capability of the device
   */
  uint8_t ioCapability;

  /**
   * Authentication requirement of the device
   * Man In the Middle protection required?
   */
  uint8_t mitm_mode;

  /**
   * bonding mode of the device
   */
  uint8_t bonding_mode;

  /**
   * this variable indicates whether to use a fixed pin
   * during the pairing process or a passkey has to be
   * requested to the application during the pairing process
   * 0 implies use fixed pin and 1 implies request for passkey
   */
  uint8_t Use_Fixed_Pin;

  /**
   * minimum encryption key size requirement
   */
  uint8_t encryptionKeySizeMin;

  /**
   * maximum encryption key size requirement
   */
  uint8_t encryptionKeySizeMax;

  /**
   * fixed pin to be used in the pairing process if
   * Use_Fixed_Pin is set to 1
   */
  uint32_t Fixed_Pin;

  /**
   * this flag indicates whether the host has to initiate
   * the security, wait for pairing or does not have any security
   * requirements.
   * 0x00 : no security required
   * 0x01 : host should initiate security by sending the slave security
   *        request command
   * 0x02 : host need not send the clave security request but it
   * has to wait for paiirng to complete before doing any other
   * processing
   */
  uint8_t initiateSecurity;
  /* USER CODE BEGIN tSecurityParams*/

  /* USER CODE END tSecurityParams */
}tSecurityParams;

/**
 * global context
 * contains the variables common to all
 * services
 */
typedef struct _tBLEProfileGlobalContext
{
  /**
   * security requirements of the host
   */
  tSecurityParams bleSecurityParam;

  /**
   * gap service handle
   */
  uint16_t gapServiceHandle;

  /**
   * device name characteristic handle
   */
  uint16_t devNameCharHandle;

  /**
   * appearance characteristic handle
   */
  uint16_t appearanceCharHandle;

  /**
   * length of the UUID list to be used while advertising
   */
  uint8_t advtServUUIDlen;

  /**
   * the UUID list to be used while advertising
   */
  uint8_t advtServUUID[100];
  /* USER CODE BEGIN BleGlobalContext_t*/

  /* USER CODE END BleGlobalContext_t */
}BleGlobalContext_t;

typedef struct
{
  BleGlobalContext_t BleApplicationContext_legacy;
  /**
   * used to identify the GAP State
   */
  APP_BLE_ConnStatus_t SmartPhone_Connection_Status;

  /**
   * used to identify the GAP State
   */
  APP_BLE_ConnStatus_t EndDevice_Connection_Status[6];

  /**
   * connection handle with the Central connection (Smart Phone)
   * When not in connection, the handle is set to 0xFFFF
   */
  uint16_t connectionHandleCentral;

  /**
   * connection handle with the Server 1 connection (End Device 1)
   * When not in connection, the handle is set to 0xFFFF
   */
  uint16_t connectionHandleEndDevice1;

  /**
   * used when doing advertising to find end device 1
   */
  uint8_t EndDevice1Found;

  /**
   * address of end device 1
   */
  tBDAddr end_device_1_bdaddr;
} BleApplicationContext_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines -----------------------------------------------------------*/
#define FAST_ADV_TIMEOUT               (30*1000*1000/CFG_TS_TICK_VAL) /**< 30s */
#define INITIAL_ADV_TIMEOUT            (60*1000*1000/CFG_TS_TICK_VAL) /**< 60s */

#define BD_ADDR_SIZE_LOCAL    6

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
PLACE_IN_SECTION("MB_MEM1") ALIGN(4) static TL_CmdPacket_t BleCmdBuffer;

/**
 *   Identity root key used to derive IRK and DHK(Legacy)
 */
static const uint8_t a_BLE_CfgIrValue[16] = CFG_BLE_IR;

/**
 * Encryption root key used to derive LTK(Legacy) and CSRK
 */
static const uint8_t a_BLE_CfgErValue[16] = CFG_BLE_ER;

/**
 * These are the two tags used to manage a power failure during OTA
 * The MagicKeywordAdress shall be mapped @0x140 from start of the binary image
 * The MagicKeywordvalue is checked in the ble_ota application
 */
PLACE_IN_SECTION("TAG_OTA_END") const uint32_t MagicKeywordValue = 0x94448A29 ;
PLACE_IN_SECTION("TAG_OTA_START") const uint32_t MagicKeywordAddress = (uint32_t)&MagicKeywordValue;

static BleApplicationContext_t BleApplicationContext;

Custom_App_ConnHandle_Not_evt_t HandleNotification;

#if (L2CAP_REQUEST_NEW_CONN_PARAM != 0)
#define SIZE_TAB_CONN_INT            2
float a_ConnInterval[SIZE_TAB_CONN_INT] = {50, 1000}; /* ms */
uint8_t index_con_int, mutex;
#endif /* L2CAP_REQUEST_NEW_CONN_PARAM != 0 */

/**
 * Advertising Data
 */
uint8_t a_AdvData[15] =
{
  9, AD_TYPE_COMPLETE_LOCAL_NAME, 'F', 'l', 'y', 'S', 'i', 'g', 'h', 't',  /* Complete name */
  4, AD_TYPE_MANUFACTURER_SPECIFIC_DATA, 0xDB, 0x09, 0x00 /* Structure version */,
};

/* USER CODE BEGIN PV */
uint8_t Advertising_mgr_timer_Id;

/* Advertising callback */
void (*Adv_Callback)(void) = 0;
void (*Next_Adv_Callback)(void) = 0;

/* Pairing request flag */
uint8_t request_pairing = 0;

/* Poisoned-bond self-heal (peripheral role).
 *
 * Observed on the Mac (bluetoothd log, 2026-08-18): a central that still holds
 * an LTK from an old pairing connects fine through the whitelist, immediately
 * starts encryption with that stale key, fails, and tears the link down within
 * a second — over and over, and it never falls back to fresh pairing while its
 * side still holds a key. The device cannot fix the central, but it can stop
 * matching it: after two consecutive links from the same identity that die
 * without a successful PAIRING_COMPLETE, that identity's bond is removed here.
 * The peer then drops off the whitelist, its next attempt (via PAIRING mode)
 * finds no key on our side, gets "key missing" at the link layer, and the
 * central re-pairs from scratch — which is the recovery path every stack
 * implements.
 *
 * A healthy bonded reconnect re-encrypts within a few hundred ms and raises
 * PAIRING_COMPLETE status 0 (seen in BLEDIAG.TXT: CONN at 59496, PAIR_CPLT at
 * 59681), so it resets the counter and is never at risk. An unbonded stranger
 * browsing during a pairing window trips the counter, but removing a bond that
 * does not exist is a no-op. */
static uint8_t central_id_type;         /* identity of the connected central */
static uint8_t central_id_addr[6];
static uint8_t central_id_valid = 0;    /* identity above is populated */
static uint8_t central_pair_ok = 0;     /* PAIR_CPLT status 0 seen on this link */
static uint32_t central_conn_tick = 0;  /* HAL_GetTick at connection complete */
static uint8_t central_fail_count = 0;  /* consecutive dead links, same identity */
static uint8_t central_fail_type;
static uint8_t central_fail_addr[6];

/* The resolving list could not be updated (0x0C, command disallowed: the
 * controller refuses RL changes while scanning or advertising is enabled —
 * BLEDIAG.TXT showed every rebuild failing once the ENGO scan was running).
 * When that happens the scan is terminated, this flag is raised, and
 * Scan_Request retries the rebuild before it starts the next scan. */
static uint8_t resolving_list_dirty = 0;

/* A bond removal the controller refused, to be tried once more from
 * Scan_Request — the one context where the ENGO scan is known to be stopped.
 * Whether the refusal happens at all is what the BOND_EVICT/DISC_EVAL records
 * are there to establish; this is the retry, not a claim about the cause. */
static uint8_t pending_evict = 0;
static uint8_t pending_evict_type;
static uint8_t pending_evict_addr[6];

/* BD Address of device to be connected once discovered */
tBDAddr P2P_SERVER1_BDADDR;
/* Address type of device to be connected (0=public, 1=random, etc.) */
static uint8_t P2P_SERVER1_ADDR_TYPE = GAP_PUBLIC_ADDR;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void BLE_UserEvtRx(void *p_Payload);
static void BLE_StatusNot(HCI_TL_CmdStatus_t Status);
static void Ble_Tl_Init(void);
static void Ble_Hci_Gap_Gatt_Init(void);
static void Adv_Request(APP_BLE_ConnStatus_t NewStatus);
static void Adv_Cancel(void);
#if (L2CAP_REQUEST_NEW_CONN_PARAM != 0)
static void BLE_SVC_L2CAP_Conn_Update(uint16_t ConnectionHandle);
static void Connection_Interval_Update_Req(void);
#endif /* L2CAP_REQUEST_NEW_CONN_PARAM != 0 */

/* USER CODE BEGIN PFP */
static void LinkConfiguration(void);
static void FS_Adv_Request(APP_BLE_ConnStatus_t NewStatus);
static void Adv_Update_Req(void);
static void Adv_Update(void);
static void APP_BLE_UpdateAdvertisingData(APP_BLE_ConnStatus_t NewStatus);
static int8_t ble_count_bonded_devices(void);
static void Scan_Request(void);
static void Connect_Request(void);
static void Disconnect_Request(void);
/* USER CODE END PFP */

/* External variables --------------------------------------------------------*/
extern RNG_HandleTypeDef hrng;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Functions Definition ------------------------------------------------------*/
void APP_BLE_Init(void)
{
  SHCI_CmdStatus_t status;
#if (RADIO_ACTIVITY_EVENT != 0)
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
#endif /* RADIO_ACTIVITY_EVENT != 0 */
  /* USER CODE BEGIN APP_BLE_Init_1 */

  /* USER CODE END APP_BLE_Init_1 */
  SHCI_C2_Ble_Init_Cmd_Packet_t ble_init_cmd_packet =
  {
    {{0,0,0}},                          /**< Header unused */
    {0,                                 /** pBleBufferAddress not used */
     0,                                 /** BleBufferSize not used */
     CFG_BLE_NUM_GATT_ATTRIBUTES,
     CFG_BLE_NUM_GATT_SERVICES,
     CFG_BLE_ATT_VALUE_ARRAY_SIZE,
     CFG_BLE_NUM_LINK,
     CFG_BLE_DATA_LENGTH_EXTENSION,
     CFG_BLE_PREPARE_WRITE_LIST_SIZE,
     CFG_BLE_MBLOCK_COUNT,
     CFG_BLE_MAX_ATT_MTU,
     CFG_BLE_PERIPHERAL_SCA,
     CFG_BLE_CENTRAL_SCA,
     CFG_BLE_LS_SOURCE,
     CFG_BLE_MAX_CONN_EVENT_LENGTH,
     CFG_BLE_HSE_STARTUP_TIME,
     CFG_BLE_VITERBI_MODE,
     CFG_BLE_OPTIONS,
     0,
     CFG_BLE_MAX_COC_INITIATOR_NBR,
     CFG_BLE_MIN_TX_POWER,
     CFG_BLE_MAX_TX_POWER,
     CFG_BLE_RX_MODEL_CONFIG,
     CFG_BLE_MAX_ADV_SET_NBR,
     CFG_BLE_MAX_ADV_DATA_LEN,
     CFG_BLE_TX_PATH_COMPENS,
     CFG_BLE_RX_PATH_COMPENS,
     CFG_BLE_CORE_VERSION,
     CFG_BLE_OPTIONS_EXT
    }
  };

  /**
   * Initialize Ble Transport Layer
   */
  Ble_Tl_Init();

  /**
   * Do not allow standby in the application
   */
  UTIL_LPM_SetOffMode(1 << CFG_LPM_APP_BLE, UTIL_LPM_DISABLE);

  /**
   * Register the hci transport layer to handle BLE User Asynchronous Events
   */
  UTIL_SEQ_RegTask(1<<CFG_TASK_HCI_ASYNCH_EVT_ID, UTIL_SEQ_RFU, hci_user_evt_proc);

  /**
   * Starts the BLE Stack on CPU2
   */
  status = SHCI_C2_BLE_Init(&ble_init_cmd_packet);
  if (status != SHCI_Success)
  {
    APP_DBG_MSG("  Fail   : SHCI_C2_BLE_Init command, result: 0x%02x\n\r", status);
    /* if you are here, maybe CPU2 doesn't contain STM32WB_Copro_Wireless_Binaries, see Release_Notes.html */
    Error_Handler();
  }
  else
  {
    APP_DBG_MSG("  Success: SHCI_C2_BLE_Init command\n\r");
  }

  /**
   * Initialization of HCI & GATT & GAP layer
   */
  Ble_Hci_Gap_Gatt_Init();

  /**
   * Initialization of the BLE Services
   */
  SVCCTL_Init();

  /**
   * Initialization of the BLE App Context
   */
  BleApplicationContext.SmartPhone_Connection_Status = APP_BLE_IDLE;
  BleApplicationContext.EndDevice_Connection_Status[0] = APP_BLE_IDLE;
  BleApplicationContext.EndDevice1Found = 0x00;

  /**
   * From here, all initialization are BLE application specific
   */

  UTIL_SEQ_RegTask(1<<CFG_TASK_ADV_CANCEL_ID, UTIL_SEQ_RFU, Adv_Cancel);

  /* USER CODE BEGIN APP_BLE_Init_4 */
  UTIL_SEQ_RegTask(1<<CFG_TASK_LINK_CONFIG_ID, UTIL_SEQ_RFU, LinkConfiguration);
  UTIL_SEQ_RegTask(1<<CFG_TASK_ADV_UPDATE_ID, UTIL_SEQ_RFU, Adv_Update);
  UTIL_SEQ_RegTask(1<<CFG_TASK_START_SCAN_ID, UTIL_SEQ_RFU, Scan_Request);
  UTIL_SEQ_RegTask(1<<CFG_TASK_CONN_DEV_1_ID, UTIL_SEQ_RFU, Connect_Request);
  UTIL_SEQ_RegTask(1<<CFG_TASK_DISCONN_DEV_1_ID, UTIL_SEQ_RFU, Disconnect_Request);
  /* USER CODE END APP_BLE_Init_4 */

  /**
   * Initialization of ADV - Ad Manufacturer Element - Support OTA Bit Mask
   */
#if (RADIO_ACTIVITY_EVENT != 0)
  ret = aci_hal_set_radio_activity_mask(0x0006);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_hal_set_radio_activity_mask command, result: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_hal_set_radio_activity_mask command\n\r");
  }
#endif /* RADIO_ACTIVITY_EVENT != 0 */

#if (L2CAP_REQUEST_NEW_CONN_PARAM != 0)
  index_con_int = 0;
  mutex = 1;
#endif /* L2CAP_REQUEST_NEW_CONN_PARAM != 0 */

  /**
   * Initialize Custom Template Application
   */
  Custom_APP_Init();

  /* USER CODE BEGIN APP_BLE_Init_3 */
  /* Create timer to handle the connection state machine */
  HW_TS_Create(CFG_TIM_PROC_ID_ISR, &Advertising_mgr_timer_Id, hw_ts_SingleShot, Adv_Update_Req);
#if 0
  /* USER CODE END APP_BLE_Init_3 */

  /**
   * Make device discoverable
   */
  BleApplicationContext.BleApplicationContext_legacy.advtServUUID[0] = NULL;
  BleApplicationContext.BleApplicationContext_legacy.advtServUUIDlen = 0;

  /**
   * Start to Advertise to be connected by a Client
   */
  Adv_Request(APP_BLE_FAST_ADV);

  /* USER CODE BEGIN APP_BLE_Init_2 */
#endif
  /**
   * Make device discoverable
   */
  BleApplicationContext.BleApplicationContext_legacy.advtServUUID[0] = NULL;
  BleApplicationContext.BleApplicationContext_legacy.advtServUUIDlen = 0;

  /**
   * Start to Advertise to be connected by a Client
   */
  FS_Adv_Request(APP_BLE_FAST_ADV);
  /* USER CODE END APP_BLE_Init_2 */

  return;
}

SVCCTL_UserEvtFlowStatus_t SVCCTL_App_Notification(void *p_Pckt)
{
  hci_event_pckt    *p_event_pckt;
  evt_le_meta_event *p_meta_evt;
  evt_blecore_aci   *p_blecore_evt;
  tBleStatus        ret = BLE_STATUS_INVALID_PARAMS;
  hci_le_enhanced_connection_complete_event_rp0 *p_enhanced_connection_complete_event;
  hci_disconnection_complete_event_rp0        *p_disconnection_complete_event;
#if (CFG_DEBUG_APP_TRACE != 0)
  hci_le_connection_update_complete_event_rp0 *p_connection_update_complete_event;
#endif /* CFG_DEBUG_APP_TRACE != 0 */
  aci_l2cap_connection_update_req_event_rp0 *pr;
  hci_le_advertising_report_event_rp0 * le_advertising_event;
  uint8_t result;
  uint8_t role, event_type, event_data_size;
  int k = 0;
  uint8_t *adv_report_data;
  uint8_t adtype, adlength;
  uint16_t connection_handle;

  /* PAIRING */
  aci_gap_pairing_complete_event_rp0          *p_pairing_complete;
  /* PAIRING */

  /* USER CODE BEGIN SVCCTL_App_Notification */

  /* USER CODE END SVCCTL_App_Notification */

  p_event_pckt = (hci_event_pckt*) ((hci_uart_pckt *) p_Pckt)->data;

  switch (p_event_pckt->evt)
  {
    case HCI_DISCONNECTION_COMPLETE_EVT_CODE:
      p_disconnection_complete_event = (hci_disconnection_complete_event_rp0 *) p_event_pckt->data;

      /* USER CODE BEGIN EVT_DISCONN_COMPLETE */

      /* USER CODE END EVT_DISCONN_COMPLETE */

      /* Record 4, the second half. Reasons worth knowing by heart: 0x08
       * supervision timeout (walked out of range / powered off), 0x13 remote
       * user terminated, 0x16 local host terminated, 0x3D MIC failure and 0x05
       * authentication failure — the last two are what a broken bond looks like
       * from the link layer, as against a peer that was simply never let in. */
      FS_BleDiag_Log("DISC handle=0x%04X reason=0x%02X peer=%s status=0x%02X",
          (unsigned) p_disconnection_complete_event->Connection_Handle,
          (unsigned) p_disconnection_complete_event->Reason,
          (p_disconnection_complete_event->Connection_Handle
              == BleApplicationContext.connectionHandleEndDevice1) ? "engo" :
          (p_disconnection_complete_event->Connection_Handle
              == BleApplicationContext.connectionHandleCentral) ? "phone" : "other",
          (unsigned) p_disconnection_complete_event->Status);

      if (p_disconnection_complete_event->Connection_Handle == BleApplicationContext.connectionHandleEndDevice1)
      {
        APP_DBG_MSG("\r\n\r** DISCONNECTION EVENT OF END DEVICE 1 \n\r");
        BleApplicationContext.EndDevice_Connection_Status[0] = APP_BLE_IDLE;
        BleApplicationContext.connectionHandleEndDevice1 = 0xFFFF;

        /* Reason per BT spec: 0x08 supervision timeout (out of range/powered
         * off), 0x13 remote terminated, 0x16 local terminated (self-heal). */
        FS_Log_WriteEventAsync("ENGO disconnected: reason 0x%02X",
                               p_disconnection_complete_event->Reason);

        /* Reset GATT client and app FSM, then restart discovery */
        FS_ActiveLook_Client_OnDisconnect();
        FS_ActiveLook_OnDisconnect();
        UTIL_SEQ_SetTask(1 << CFG_TASK_START_SCAN_ID, CFG_SCH_PRIO_0);
      }

      if (p_disconnection_complete_event->Connection_Handle == BleApplicationContext.connectionHandleCentral)
      {
        APP_DBG_MSG("\r\n\r** DISCONNECTION EVENT OF SMART PHONE \n\r");
        BleApplicationContext.SmartPhone_Connection_Status = APP_BLE_IDLE;
        BleApplicationContext.connectionHandleCentral = 0xFFFF;

        /* Poisoned-bond self-heal. A link that ends without a successful
         * PAIRING_COMPLETE counts against its identity when it looks like a
         * broken bond: an explicit security failure (0x05 authentication,
         * 0x06 PIN/key missing, 0x3D MIC), or any death within 3 s of the
         * connect — a stale-key central connects, fails encryption and drops
         * in well under a second, while a human browsing and leaving takes
         * longer. Two in a row from the same identity and that bond is
         * removed, so the peer's next pairing attempt starts clean. */
        /* Why the eviction did or did not fire, on EVERY central disconnect.
         * The first attempt shipped only a BOND_EVICT line, which says nothing
         * when the branch is never reached — and on the hardware it was not
         * reached, or the removal failed, and the file could not tell the two
         * apart. This line names every input the decision is made from. */
        FS_BleDiag_Log("DISC_EVAL valid=%u pair_ok=%u reason=0x%02X dt=%u count=%u",
            (unsigned) central_id_valid, (unsigned) central_pair_ok,
            (unsigned) p_disconnection_complete_event->Reason,
            (unsigned) (HAL_GetTick() - central_conn_tick),
            (unsigned) central_fail_count);

        if (central_id_valid && !central_pair_ok)
        {
          uint8_t reason = p_disconnection_complete_event->Reason;
          uint8_t security_reason = (reason == 0x05) || (reason == 0x06)
                                 || (reason == 0x3D);
          uint8_t died_fast = (HAL_GetTick() - central_conn_tick) < 3000;

          if (security_reason || died_fast)
          {
            if (central_fail_count > 0
                && central_fail_type == central_id_type
                && memcmp(central_fail_addr, central_id_addr, 6) == 0)
            {
              central_fail_count++;
            }
            else
            {
              central_fail_count = 1;
              central_fail_type = central_id_type;
              memcpy(central_fail_addr, central_id_addr, 6);
            }

            if (central_fail_count >= 2)
            {
              tBleStatus evict_ret = aci_gap_remove_bonded_device(
                  central_fail_type, central_fail_addr);
              central_fail_count = 0;
#if FS_BLE_DIAG
              {
                char evictStr[18];
                FS_BleDiag_FormatAddr(evictStr, sizeof(evictStr),
                                      central_fail_addr);
                FS_BleDiag_Log("BOND_EVICT type=%u addr=%s remove=0x%02X"
                               " reason=0x%02X",
                    (unsigned) central_fail_type, evictStr,
                    (unsigned) evict_ret, (unsigned) reason);
              }
#endif
              if (evict_ret != BLE_STATUS_SUCCESS)
              {
                /* Try again from Scan_Request, with the ENGO scan stopped. */
                pending_evict = 1;
                pending_evict_type = central_fail_type;
                memcpy(pending_evict_addr, central_fail_addr, 6);
                (void) aci_gap_terminate_gap_proc(GAP_GENERAL_DISCOVERY_PROC);
              }
            }
          }
        }
        else
        {
          central_fail_count = 0;
        }
        central_id_valid = 0;
        central_pair_ok = 0;

        if (FS_State_Get()->enable_ble)
        {
          /* restart advertising */
          FS_Adv_Request(APP_BLE_FAST_ADV);
        }

        HandleNotification.Custom_Evt_Opcode = CUSTOM_DISCON_HANDLE_EVT;
        HandleNotification.ConnectionHandle = 0xFFFF;
        Custom_APP_Notification(&HandleNotification);
      }
      break; /* HCI_DISCONNECTION_COMPLETE_EVT_CODE */

    case HCI_LE_META_EVT_CODE:
    {
      p_meta_evt = (evt_le_meta_event*) p_event_pckt->data;
      /* USER CODE BEGIN EVT_LE_META_EVENT */

      /* USER CODE END EVT_LE_META_EVENT */
      switch (p_meta_evt->subevent)
      {
        case HCI_LE_CONNECTION_UPDATE_COMPLETE_SUBEVT_CODE:
#if (CFG_DEBUG_APP_TRACE != 0)
          p_connection_update_complete_event = (hci_le_connection_update_complete_event_rp0 *) p_meta_evt->data;
          APP_DBG_MSG(">>== HCI_LE_CONNECTION_UPDATE_COMPLETE_SUBEVT_CODE\n");
          APP_DBG_MSG("     - Connection Interval:   %.2f ms\n     - Connection latency:    %d\n     - Supervision Timeout: %d ms\n\r",
                       p_connection_update_complete_event->Conn_Interval*1.25,
                       p_connection_update_complete_event->Conn_Latency,
                       p_connection_update_complete_event->Supervision_Timeout*10);
#endif /* CFG_DEBUG_APP_TRACE != 0 */

          /* USER CODE BEGIN EVT_LE_CONN_UPDATE_COMPLETE */

          /* USER CODE END EVT_LE_CONN_UPDATE_COMPLETE */
          break;

        case HCI_LE_ENHANCED_CONNECTION_COMPLETE_SUBEVT_CODE:
          p_enhanced_connection_complete_event = (hci_le_enhanced_connection_complete_event_rp0 *) p_meta_evt->data;
          /**
           * The connection is done, there is no need anymore to schedule the LP ADV
           */

          connection_handle = p_enhanced_connection_complete_event->Connection_Handle;
          role = p_enhanced_connection_complete_event->Role;

#if FS_BLE_DIAG
          /* Record 4, the first half — and the single most informative line in
           * the file for this bug. role=1 is us as PERIPHERAL, i.e. a phone or
           * the Mac got past the whitelist filter. peer_type 0x00/0x01 is a plain
           * public/random identity; 0x02/0x03 mean the controller RESOLVED the
           * peer's private address against the resolving list, so peer_addr is
           * the identity behind it and peer_rpa is what was actually on the air.
           * A phone that only ever gets in after a double-press should show
           * peer_type 0x01 with a peer_rpa-shaped address here — that is the
           * signature of "known device, unresolvable identity". */
          {
            char idStr[18], rpaStr[18];
            FS_BleDiag_FormatAddr(idStr, sizeof(idStr),
                p_enhanced_connection_complete_event->Peer_Address);
            FS_BleDiag_FormatAddr(rpaStr, sizeof(rpaStr),
                p_enhanced_connection_complete_event->Peer_Resolvable_Private_Address);
            FS_BleDiag_Log("CONN st=0x%02X role=%u handle=0x%04X ptype=%u paddr=%s",
                (unsigned) p_enhanced_connection_complete_event->Status,
                (unsigned) role, (unsigned) connection_handle,
                (unsigned) p_enhanced_connection_complete_event->Peer_Address_Type,
                idStr);
            FS_BleDiag_Log("CONN handle=0x%04X peer_rpa=%s int=%u to=%u",
                (unsigned) connection_handle, rpaStr,
                (unsigned) p_enhanced_connection_complete_event->Conn_Interval,
                (unsigned) p_enhanced_connection_complete_event->Supervision_Timeout);
          }
#endif

          if (role == 0x00)
          { /* ROLE CENTRAL */
        	  APP_DBG_MSG("-- CONNECTION SUCCESS WITH END DEVICE 1\n\r");
        	  BleApplicationContext.EndDevice_Connection_Status[0] = APP_BLE_CONNECTED;
        	  BleApplicationContext.connectionHandleEndDevice1 = connection_handle;
        	  FS_ActiveLook_Client_StartDiscovery(connection_handle);
          }
          else
          {
            APP_DBG_MSG("-- CONNECTION SUCCESS WITH SMART PHONE\n\r");
            BleApplicationContext.SmartPhone_Connection_Status = APP_BLE_CONNECTED;
            BleApplicationContext.connectionHandleCentral = connection_handle;

            /* Remember who this is, for the poisoned-bond self-heal at
             * disconnect. Types 0x02/0x03 mean the controller already resolved
             * a private address: Peer_Address then IS the identity, of type
             * public (0x00) or static-random (0x01) respectively — the same
             * form the bonding table stores. */
            central_id_type = p_enhanced_connection_complete_event->Peer_Address_Type;
            if (central_id_type >= 0x02)
            {
              central_id_type -= 0x02;
            }
            memcpy(central_id_addr,
                   p_enhanced_connection_complete_event->Peer_Address, 6);
            central_id_valid = 1;
            central_pair_ok = 0;
            central_conn_tick = HAL_GetTick();
            HandleNotification.Custom_Evt_Opcode = CUSTOM_CONN_HANDLE_EVT;
            HandleNotification.ConnectionHandle = connection_handle;
            Custom_APP_Notification(&HandleNotification);

            /* Stop the timer */
            HW_TS_Stop(Advertising_mgr_timer_Id);

            /* Call advertising callback */
            if (Adv_Callback) Adv_Callback();

            /* Update advertising callback */
            Adv_Callback = Next_Adv_Callback;
            Next_Adv_Callback = 0;

            /* Configure the link */
            UTIL_SEQ_SetTask(1 << CFG_TASK_LINK_CONFIG_ID, CFG_SCH_PRIO_1);
          }
          break; /* HCI_LE_CONNECTION_COMPLETE_SUBEVT_CODE */

        case HCI_LE_ADVERTISING_REPORT_SUBEVT_CODE:
          /* USER CODE BEGIN EVT_LE_ADVERTISING_REPORT */

          /* USER CODE END EVT_LE_ADVERTISING_REPORT */
          le_advertising_event = (hci_le_advertising_report_event_rp0 *) p_meta_evt->data;

          event_type = le_advertising_event->Advertising_Report[0].Event_Type;

          event_data_size = le_advertising_event->Advertising_Report[0].Length_Data;

          /* WARNING: be careful when decoding advertising report as its raw format cannot be mapped on a C structure.
          The data and RSSI values could not be directly decoded from the RAM using the data and RSSI field from hci_le_advertising_report_event_rp0 structure.
          Instead they must be read by using offsets (please refer to BLE specification).
          RSSI = (int8_t)*(uint8_t*) (adv_report_data + le_advertising_event->Advertising_Report[0].Length_Data);
          */
          adv_report_data = (uint8_t*)(&le_advertising_event->Advertising_Report[0].Length_Data) + 1;
          k = 0;

          /*
           * ActiveLook command service UUID (little-endian):
           *   0783B03E-8535-B5A0-7140-A304D2495CB7
           * LE bytes: B7 5C 49 D2 04 A3 40 71 A0 B5 35 85 3E B0 83 07
           */
          static const uint8_t activelook_svc_uuid[16] = {
              0xB7, 0x5C, 0x49, 0xD2, 0x04, 0xA3, 0x40, 0x71,
              0xA0, 0xB5, 0x35, 0x85, 0x3E, 0xB0, 0x83, 0x07
          };

          /* Flags to track match within this advertising report */
          uint8_t foundUUID = 0;   /* ActiveLook service UUID found */
          uint8_t foundName = 0;   /* Name prefix "ENGO" found */
          char    candSerial[FS_ENGO_SERIAL_LEN + 1] = {0}; /* trailing 6 chars of name = ENGO serial */
          uint8_t haveSerial = 0;  /* candSerial is populated */
          char    candName[17] = {0};  /* advertised name, for the event log */

          /* Process both ADV_IND and SCAN_RSP so service UUID in scan response is caught */
          if (event_type == ADV_IND || event_type == SCAN_RSP)
          {
            while(k < event_data_size)
            {
              adlength = adv_report_data[k];
              if (adlength == 0) break; /* malformed — stop */
              adtype = adv_report_data[k + 1];

              if (adtype == AD_TYPE_128_BIT_SERV_UUID ||
                  adtype == AD_TYPE_128_BIT_SERV_UUID_CMPLT_LIST)
              {
                /* Each UUID is 16 bytes; there may be multiple */
                uint8_t uuid_data_len = adlength - 1;
                const uint8_t *uuid_data = &adv_report_data[k + 2];
                uint8_t ui;
                for (ui = 0; ui + 16 <= uuid_data_len; ui += 16)
                {
                  if (memcmp(&uuid_data[ui], activelook_svc_uuid, 16) == 0)
                  {
                    APP_DBG_MSG("-- Found ActiveLook service UUID in adv data\n\r");
                    foundUUID = 1;
                    break;
                  }
                }
              }
              else if (adtype == AD_TYPE_COMPLETE_LOCAL_NAME ||
                       adtype == AD_TYPE_SHORTENED_LOCAL_NAME)
              {
                const uint8_t *name_data = &adv_report_data[k + 2];
                uint8_t name_len = adlength - 1;

                /* Keep a copy for the event log (prefer the complete name). */
                if (candName[0] == '\0' || adtype == AD_TYPE_COMPLETE_LOCAL_NAME)
                {
                  uint8_t ci, cn = name_len < 16 ? name_len : 16;
                  for (ci = 0; ci < cn; ci++) candName[ci] = (char)name_data[ci];
                  candName[cn] = '\0';
                }

                APP_DBG_MSG("-- Found Device Name: '");
                {
                  uint8_t ni;
                  for (ni = 0; ni < name_len; ni++) { APP_DBG_MSG("%c", name_data[ni]); }
                }
                APP_DBG_MSG("'\n\r");

                /* Capture the trailing 6 chars of the name = ENGO Customer Serial
                 * (e.g. "ENGO 3 123456" -> "123456"). Per the ActiveLook API the
                 * serial is the last 6 chars of the COMPLETE 16-char name; a
                 * SHORTENED name is truncated at the tail, so its last 6 chars are
                 * NOT the serial — never capture from it (a wrong capture would be
                 * permanently auto-bound to engo3.txt). */
                if (adtype == AD_TYPE_COMPLETE_LOCAL_NAME &&
                    name_len >= FS_ENGO_SERIAL_LEN)
                {
                  uint8_t si;
                  for (si = 0; si < FS_ENGO_SERIAL_LEN; si++)
                    candSerial[si] = (char)name_data[name_len - FS_ENGO_SERIAL_LEN + si];
                  candSerial[FS_ENGO_SERIAL_LEN] = '\0';
                  haveSerial = 1;
                }

                /* Case-insensitive match of known ActiveLook/ENGO advertised-name
                 * prefixes. ENGO 2/3 advertise "ENGO ..."; other ActiveLook devices
                 * use "AL-"/"AL "/"ActiveLook"/"A.Look". The name may live only in the
                 * scan response, which is why we run an active scan and parse both. */
                {
                  static const char *al_prefixes[] = { "ENGO", "AL-", "AL ", "ACTIVELOOK", "A.LOOK" };
                  uint8_t pi;
                  for (pi = 0; pi < 5 && !foundName; pi++)
                  {
                    const char *pfx = al_prefixes[pi];
                    uint8_t plen = (uint8_t)strlen(pfx);
                    if (name_len >= plen)
                    {
                      uint8_t mi, ok = 1;
                      for (mi = 0; mi < plen; mi++)
                      {
                        uint8_t c = name_data[mi];
                        if (c >= 'a' && c <= 'z') c = (uint8_t)(c - 32); /* to upper */
                        if (c != (uint8_t)pfx[mi]) { ok = 0; break; }
                      }
                      if (ok) { foundName = 1; APP_DBG_MSG("-- Name prefix '%s' matched\n\r", pfx); }
                    }
                  }
                }
              }

              k += adlength + 1;
            } /* end while(k < event_data_size) */

            /* Decide whether to accept this device.
             *  - BOUND   (engo3.txt present): must be an ActiveLook device (UUID or
             *            name prefix) AND its serial (trailing 6 chars of the
             *            advertised name) must equal the saved one. The serial
             *            NARROWS the ActiveLook filter, it does not replace it —
             *            otherwise any BLE device whose name happens to end in
             *            those 6 chars would be connected to.
             *  - UNBOUND (no engo3.txt): first ActiveLook device wins (UUID or
             *            name prefix), and we remember its serial to persist once
             *            the link is up (auto-bind on first ever connect). */
            uint8_t accept = 0;
            if (BleApplicationContext.EndDevice1Found == 0x00)
            {
              if (FS_EngoBind_IsBound())
                accept = ((foundUUID || foundName) &&
                          haveSerial && FS_EngoBind_SerialMatches(candSerial));
              else
                accept = (foundUUID || foundName);
            }

            if (accept)
            {
              APP_DBG_MSG("-- Found matching ENGO/ActiveLook device!\n\r");
              BleApplicationContext.EndDevice1Found = 0x01;

              FS_Log_WriteEventAsync("ENGO found: '%s' (name=%u uuid=%u) addr %02X:%02X:%02X:%02X:%02X:%02X",
                  candName[0] ? candName : "?", foundName, foundUUID,
                  le_advertising_event->Advertising_Report[0].Address[5],
                  le_advertising_event->Advertising_Report[0].Address[4],
                  le_advertising_event->Advertising_Report[0].Address[3],
                  le_advertising_event->Advertising_Report[0].Address[2],
                  le_advertising_event->Advertising_Report[0].Address[1],
                  le_advertising_event->Advertising_Report[0].Address[0]);

              /* Unbound first connect: stash the serial to learn after link-up. */
              if (!FS_EngoBind_IsBound() && haveSerial)
                FS_EngoBind_NotePending(candSerial);

              /* Store BD Address and Address_Type from the advertising report */
              P2P_SERVER1_BDADDR[0] = le_advertising_event->Advertising_Report[0].Address[0];
              P2P_SERVER1_BDADDR[1] = le_advertising_event->Advertising_Report[0].Address[1];
              P2P_SERVER1_BDADDR[2] = le_advertising_event->Advertising_Report[0].Address[2];
              P2P_SERVER1_BDADDR[3] = le_advertising_event->Advertising_Report[0].Address[3];
              P2P_SERVER1_BDADDR[4] = le_advertising_event->Advertising_Report[0].Address[4];
              P2P_SERVER1_BDADDR[5] = le_advertising_event->Advertising_Report[0].Address[5];
              P2P_SERVER1_ADDR_TYPE  = le_advertising_event->Advertising_Report[0].Address_Type;

              APP_DBG_MSG("   Address (%s): %02X:%02X:%02X:%02X:%02X:%02X\n\r",
                          P2P_SERVER1_ADDR_TYPE == 0 ? "Public" : "Random",
                          P2P_SERVER1_BDADDR[5], P2P_SERVER1_BDADDR[4], P2P_SERVER1_BDADDR[3],
                          P2P_SERVER1_BDADDR[2], P2P_SERVER1_BDADDR[1], P2P_SERVER1_BDADDR[0]);
            }
          } /* end if (event_type == ADV_IND || event_type == SCAN_RSP) */
          break;

        default:
          /* USER CODE BEGIN SUBEVENT_DEFAULT */

          /* USER CODE END SUBEVENT_DEFAULT */
          break;
      }

      /* USER CODE BEGIN META_EVT */

      /* USER CODE END META_EVT */
      break; /* HCI_LE_META_EVT_CODE */
    }

    case HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE:
      p_blecore_evt = (evt_blecore_aci*) p_event_pckt->data;
      /* USER CODE BEGIN EVT_VENDOR */

      /* USER CODE END EVT_VENDOR */
      switch (p_blecore_evt->ecode)
      {
        /* USER CODE BEGIN ecode */

        /* USER CODE END ecode */

        /**
         * SPECIFIC to Custom Template APP
         */
        case ACI_L2CAP_CONNECTION_UPDATE_REQ_VSEVT_CODE:
          {
            /* USER CODE BEGIN EVT_BLUE_L2CAP_CONNECTION_UPDATE_REQ */

            /* USER CODE END EVT_BLUE_L2CAP_CONNECTION_UPDATE_REQ */
            pr = (aci_l2cap_connection_update_req_event_rp0 *) p_blecore_evt->data;
            result = aci_l2cap_connection_parameter_update_resp(pr->Connection_Handle,
                                                                pr->Interval_Min,
                                                                pr->Interval_Max,
                                                                pr->Latency,
                                                                pr->Timeout_Multiplier,
                                                                CONN_L1,
                                                                CONN_L2,
																pr->Identifier,
                                                                0x00);
            APP_DBG_MSG("\r\n\r** NO UPDATE \n\r");
            if(result != BLE_STATUS_SUCCESS)
            {
              /* USER CODE BEGIN BLE_STATUS_SUCCESS */

              /* USER CODE END BLE_STATUS_SUCCESS */
            }
          }
          break;

        case ACI_L2CAP_CONNECTION_UPDATE_RESP_VSEVT_CODE:
#if (L2CAP_REQUEST_NEW_CONN_PARAM != 0)
          mutex = 1;
#endif /* L2CAP_REQUEST_NEW_CONN_PARAM != 0 */
          /* USER CODE BEGIN EVT_BLUE_L2CAP_CONNECTION_UPDATE_RESP */

          /* USER CODE END EVT_BLUE_L2CAP_CONNECTION_UPDATE_RESP */
          break;

        case ACI_GAP_PROC_COMPLETE_VSEVT_CODE:
          {
            /* USER CODE BEGIN EVT_BLUE_GAP_PROCEDURE_COMPLETE */

            /* USER CODE END EVT_BLUE_GAP_PROCEDURE_COMPLETE */
            aci_gap_proc_complete_event_rp0 *gap_evt_proc_complete = (void*) p_blecore_evt->data;
            /* CHECK GAP GENERAL DISCOVERY PROCEDURE COMPLETED & SUCCEED */
            if (gap_evt_proc_complete->Procedure_Code == GAP_GENERAL_DISCOVERY_PROC
                && gap_evt_proc_complete->Status == 0x00)
            {
              /* USER CODE BEGIN GAP_GENERAL_DISCOVERY_PROC */

              /* USER CODE END GAP_GENERAL_DISCOVERY_PROC */

              APP_DBG_MSG("-- GAP GENERAL DISCOVERY PROCEDURE_COMPLETED\n\r");
              /*if a device found, connect to it, device 1 being chosen first if both found*/
              if (BleApplicationContext.EndDevice1Found == 0x01
                  && BleApplicationContext.EndDevice_Connection_Status[0] != APP_BLE_CONNECTED)
              {
                APP_DBG_MSG("-- Setting task CFG_TASK_CONN_DEV_1_ID\n\r");
                UTIL_SEQ_SetTask(1 << CFG_TASK_CONN_DEV_1_ID, CFG_SCH_PRIO_0);
              }
              else if (BleApplicationContext.EndDevice_Connection_Status[0] != APP_BLE_CONNECTED)
              {
                /* No matching device found — re-arm scanning so glasses found later */
                APP_DBG_MSG("-- No ENGO device found, re-scheduling scan\n\r");
                UTIL_SEQ_SetTask(1 << CFG_TASK_START_SCAN_ID, CFG_SCH_PRIO_0);
              }
#if (CFG_P2P_DEMO_MULTI != 0)
              /* USER CODE BEGIN EVT_BLUE_GAP_PROCEDURE_COMPLETE_Multi */

              /* USER CODE END EVT_BLUE_GAP_PROCEDURE_COMPLETE_Multi */
#endif
            }
            /* A discovery WE terminated to free the resolving list completes
             * with a NON-ZERO status, and the branch above only re-arms the
             * scan on 0x00 — so as written the ENGO scan would stop for good
             * at the first 0x0C, and the deferred rebuild in Scan_Request
             * would never run either. Re-arm it here. Gated on the flag, which
             * exactly one place sets, so an ordinary failed discovery is left
             * to behave as it always did. */
            else if (gap_evt_proc_complete->Procedure_Code == GAP_GENERAL_DISCOVERY_PROC
                     && resolving_list_dirty)
            {
              FS_BleDiag_Log("RL retry: scan ended status=0x%02X, rescheduling",
                  (unsigned) gap_evt_proc_complete->Status);
              UTIL_SEQ_SetTask(1 << CFG_TASK_START_SCAN_ID, CFG_SCH_PRIO_0);
            }
          }
          break; /* ACI_GAP_PAIRING_COMPLETE_VSEVT_CODE */

#if (RADIO_ACTIVITY_EVENT != 0)
        case ACI_HAL_END_OF_RADIO_ACTIVITY_VSEVT_CODE:
          /* USER CODE BEGIN RADIO_ACTIVITY_EVENT*/

          /* USER CODE END RADIO_ACTIVITY_EVENT*/
          break; /* ACI_HAL_END_OF_RADIO_ACTIVITY_VSEVT_CODE */
#endif /* RADIO_ACTIVITY_EVENT != 0 */

        /* PAIRING */
        case (ACI_GAP_KEYPRESS_NOTIFICATION_VSEVT_CODE):
          APP_DBG_MSG(">>== ACI_GAP_KEYPRESS_NOTIFICATION_VSEVT_CODE\n");
          /* USER CODE BEGIN ACI_GAP_KEYPRESS_NOTIFICATION_VSEVT_CODE*/

          /* USER CODE END ACI_GAP_KEYPRESS_NOTIFICATION_VSEVT_CODE*/
          break;

        case ACI_GAP_PASS_KEY_REQ_VSEVT_CODE:
          APP_DBG_MSG(">>== ACI_GAP_PASS_KEY_REQ_VSEVT_CODE \n");

          ret = aci_gap_pass_key_resp(BleApplicationContext.connectionHandleCentral, CFG_FIXED_PIN);
          if (ret != BLE_STATUS_SUCCESS)
          {
            APP_DBG_MSG("==>> aci_gap_pass_key_resp : Fail, reason: 0x%x\n", ret);
          }
          else
          {
            APP_DBG_MSG("==>> aci_gap_pass_key_resp : Success \n");
          }
          /* USER CODE BEGIN ACI_GAP_PASS_KEY_REQ_VSEVT_CODE*/

          /* USER CODE END ACI_GAP_PASS_KEY_REQ_VSEVT_CODE*/
          break;

        case ACI_GAP_NUMERIC_COMPARISON_VALUE_VSEVT_CODE:
          APP_DBG_MSG(">>== ACI_GAP_NUMERIC_COMPARISON_VALUE_VSEVT_CODE\n");
          APP_DBG_MSG("     - numeric_value = %ld\n",
                      ((aci_gap_numeric_comparison_value_event_rp0 *)(p_blecore_evt->data))->Numeric_Value);
          APP_DBG_MSG("     - Hex_value = %lx\n",
                      ((aci_gap_numeric_comparison_value_event_rp0 *)(p_blecore_evt->data))->Numeric_Value);
          ret = aci_gap_numeric_comparison_value_confirm_yesno(BleApplicationContext.connectionHandleCentral, YES);
          if (ret != BLE_STATUS_SUCCESS)
          {
            APP_DBG_MSG("==>> aci_gap_numeric_comparison_value_confirm_yesno-->YES : Fail, reason: 0x%x\n", ret);
          }
          else
          {
            APP_DBG_MSG("==>> aci_gap_numeric_comparison_value_confirm_yesno-->YES : Success \n");
          }
          /* USER CODE BEGIN ACI_GAP_NUMERIC_COMPARISON_VALUE_VSEVT_CODE*/

          /* USER CODE END ACI_GAP_NUMERIC_COMPARISON_VALUE_VSEVT_CODE*/
          break;

        case ACI_GAP_PAIRING_COMPLETE_VSEVT_CODE:
          p_pairing_complete = (aci_gap_pairing_complete_event_rp0*)p_blecore_evt->data;

          APP_DBG_MSG(">>== ACI_GAP_PAIRING_COMPLETE_VSEVT_CODE\n");
          if (p_pairing_complete->Status != 0)
          {
            APP_DBG_MSG("     - Pairing KO \n     - Status: 0x%x\n     - Reason: 0x%x\n", p_pairing_complete->Status, p_pairing_complete->Reason);
          }
          else
          {
            APP_DBG_MSG("     - Pairing Success\n");
          }
          APP_DBG_MSG("\n");

          /* USER CODE BEGIN ACI_GAP_PAIRING_COMPLETE_VSEVT_CODE*/
          /* Record 3. Status 0x00 success, 0x01 timeout, 0x02 failed; on 0x02
           * Reason carries the SMP pairing-failed code (0x03 authentication
           * requirements, 0x05 pairing not supported, 0x08 unspecified...). */
          FS_BleDiag_Log("PAIR_CPLT handle=0x%04X status=0x%02X reason=0x%02X",
              (unsigned) p_pairing_complete->Connection_Handle,
              (unsigned) p_pairing_complete->Status,
              (unsigned) p_pairing_complete->Reason);

          /* ...and the bond count immediately afterwards. This is a READ-ONLY
           * ACI query and nothing else: it does not rebuild the whitelist and
           * does not touch the resolving list, so the BLE path behaves exactly
           * as it did without this build. It answers the one question the
           * pairing-complete event does not — whether a "successful" pairing
           * actually left a bond behind for the next reconnect to match. */
#if FS_BLE_DIAG
          {
            uint8_t nb = 0;
            Bonded_Device_Entry_t nb_devices[16];
            tBleStatus nb_ret = aci_gap_get_bonded_devices(&nb, nb_devices);
            FS_BleDiag_Log("PAIR_BONDS get_bonded=0x%02X n=%u",
                (unsigned) nb_ret, (unsigned) nb);
          }
#endif
          /* A successful pairing OR re-encryption of a bonded reconnect both
           * land here with status 0 — either way this link's bond is proven
           * good, so it must not count towards eviction. */
          if (p_pairing_complete->Status == 0
              && p_pairing_complete->Connection_Handle
                  == BleApplicationContext.connectionHandleCentral)
          {
            central_pair_ok = 1;
            central_fail_count = 0;
          }
          /* USER CODE END ACI_GAP_PAIRING_COMPLETE_VSEVT_CODE*/
          break;

        /* USER CODE BEGIN ACI_GAP_SECURITY_VSEVT_CODES */
        case ACI_GAP_BOND_LOST_VSEVT_CODE:
          /* The peer is re-pairing over a bond we still hold — i.e. it threw
           * its key away and we did not. Per ble_gap_aci.h the application MUST
           * answer with aci_gap_allow_rebond or the pairing procedure times
           * out; nothing here ever called it, so a peer that lost its bond
           * could never get back in. Allow it: the double-press pairing window
           * is still the only way an unbonded central reaches pairing at all,
           * so this only lets a device the owner already trusted re-pair. */
          {
            tBleStatus rebond_ret = aci_gap_allow_rebond(
                BleApplicationContext.connectionHandleCentral);
            FS_BleDiag_Log("BOND_LOST allow_rebond=0x%02X", (unsigned) rebond_ret);
          }
          break;

        case ACI_GAP_ADDR_NOT_RESOLVED_VSEVT_CODE:
          /* The other half of the hypothesis, stated by the controller itself:
           * a peer turned up with a resolvable private address that no identity
           * in the resolving list could resolve. With privacy enabled and a
           * whitelist-only filter, this is precisely how a genuinely bonded
           * phone gets refused.
           *
           * READ THE ABSENCE OF THIS LINE WITH CARE. The firmware never calls
           * aci_gap_set_event_mask, so both this event (mask bit 0x0800) and
           * BOND_LOST (0x0080) arrive only if the stack's default mask has them
           * on. ble_gap_aci.h documents that default as 0xFFFF — every GAP event
           * enabled — and ACI_GAP_PAIRING_COMPLETE, bit 0x0001 of the same mask,
           * demonstrably does arrive here today without anyone setting it, which
           * is good evidence the default is indeed all-on. Good evidence, not
           * proof: it is NOT verified on this unit, and setting the mask
           * ourselves would be new behaviour on the BLE path, which this build
           * is not allowed. So a file with no ADDR_NOT_RESOLVED line is weak
           * evidence that resolution never failed, whereas one WITH the line is
           * conclusive that it did. */
          FS_BleDiag_Log("ADDR_NOT_RESOLVED handle=0x%04X",
              (unsigned) ((aci_gap_addr_not_resolved_event_rp0 *)(p_blecore_evt->data))->Connection_Handle);
          break;
        /* USER CODE END ACI_GAP_SECURITY_VSEVT_CODES */
        /* PAIRING */
        case ACI_GATT_INDICATION_VSEVT_CODE:
        {
          APP_DBG_MSG(">>== ACI_GATT_INDICATION_VSEVT_CODE \r");
          aci_gatt_confirm_indication(BleApplicationContext.connectionHandleCentral);
        }
        break;

        /* USER CODE BEGIN BLUE_EVT */
        case ACI_GATT_TX_POOL_AVAILABLE_VSEVT_CODE:
          Custom_APP_TxPoolAvailableNotification();
          /* Resume a half-sent ActiveLook HUD frame as soon as buffers free up */
          FS_ActiveLook_TxPoolAvailable();
          break;
        /* USER CODE END BLUE_EVT */
      }

      /* AFTER you handle your own cases, also pass the event to the ActiveLook client. */
      FS_ActiveLook_Client_EventHandler((void*)p_blecore_evt, HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE);

      break; /* HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE */

      /* USER CODE BEGIN EVENT_PCKT */

      /* USER CODE END EVENT_PCKT */

    default:
      /* USER CODE BEGIN ECODE_DEFAULT*/

      /* USER CODE END ECODE_DEFAULT*/
      break;
  }

  return (SVCCTL_UserEvtFlowEnable);
}

APP_BLE_ConnStatus_t APP_BLE_Get_Server_Connection_Status(void)
{
  return BleApplicationContext.SmartPhone_Connection_Status;
}

APP_BLE_ConnStatus_t APP_BLE_Get_Client_Connection_Status(uint16_t Connection_Handle)
{
  /* USER CODE BEGIN APP_BLE_Get_Client_Connection_Status_1 */

  /* USER CODE END APP_BLE_Get_Client_Connection_Status_1 */
  APP_BLE_ConnStatus_t return_value;

  if (BleApplicationContext.connectionHandleEndDevice1 == Connection_Handle)
  {
	return_value = BleApplicationContext.EndDevice_Connection_Status[0];
  }
  else
  {
	return_value = APP_BLE_IDLE;
  }
  /* USER CODE BEGIN APP_BLE_Get_Client_Connection_Status_2 */

  /* USER CODE END APP_BLE_Get_Client_Connection_Status_2 */

  return (return_value);
}

/* USER CODE BEGIN FD*/

/* USER CODE END FD*/

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/
static void Ble_Tl_Init(void)
{
  HCI_TL_HciInitConf_t Hci_Tl_Init_Conf;

  Hci_Tl_Init_Conf.p_cmdbuffer = (uint8_t*)&BleCmdBuffer;
  Hci_Tl_Init_Conf.StatusNotCallBack = BLE_StatusNot;
  hci_init(BLE_UserEvtRx, (void*) &Hci_Tl_Init_Conf);

  return;
}

static void Ble_Hci_Gap_Gatt_Init(void)
{
  uint8_t role;
  uint16_t gap_service_handle, gap_dev_name_char_handle, gap_appearance_char_handle;
  uint32_t a_srd_bd_addr[2] = {0,0};
  uint16_t a_appearance[1] = {BLE_CFG_GAP_APPEARANCE};
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  /* USER CODE BEGIN Ble_Hci_Gap_Gatt_Init*/

  /* USER CODE END Ble_Hci_Gap_Gatt_Init*/

  APP_DBG_MSG("==>> Start Ble_Hci_Gap_Gatt_Init function\n");

  /**
   * Initialize HCI layer
   */
  /*HCI Reset to synchronise BLE Stack*/
  ret = hci_reset();
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : hci_reset command, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: hci_reset command\n");
  }

  /**
   * Static random Address
   * The two upper bits shall be set to 1
   * The lowest 32bits is read from the UDN to differentiate between devices
   * The RNG may be used to provide a random number on each power on
   */
#if (CFG_IDENTITY_ADDRESS == GAP_STATIC_RANDOM_ADDR)
#if defined(CFG_STATIC_RANDOM_ADDRESS)
  a_srd_bd_addr[0] = CFG_STATIC_RANDOM_ADDRESS & 0xFFFFFFFF;
  a_srd_bd_addr[1] = (uint32_t)((uint64_t)CFG_STATIC_RANDOM_ADDRESS >> 32);
  a_srd_bd_addr[1] |= 0xC000; /* The two upper bits shall be set to 1 */
#else
  FS_Common_GetRandomBytes(a_srd_bd_addr, 2);
  a_srd_bd_addr[1] |= 0xC000; /* The two upper bits shall be set to 1 */
#endif /* CFG_STATIC_RANDOM_ADDRESS */
#endif

#if (CFG_BLE_ADDRESS_TYPE == GAP_STATIC_RANDOM_ADDR)
#endif

#if (CFG_BLE_ADDRESS_TYPE != PUBLIC_ADDR)
  ret = aci_hal_write_config_data(CONFIG_DATA_RANDOM_ADDRESS_OFFSET, CONFIG_DATA_RANDOM_ADDRESS_LEN, (uint8_t*)a_srd_bd_addr);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_hal_write_config_data command - CONFIG_DATA_RANDOM_ADDRESS_OFFSET, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_hal_write_config_data command - CONFIG_DATA_RANDOM_ADDRESS_OFFSET\n");
    APP_DBG_MSG("  Random Bluetooth Address: %02x:%02x:%02x:%02x:%02x:%02x\n", (uint8_t)(a_srd_bd_addr[1] >> 8),
                                                                               (uint8_t)(a_srd_bd_addr[1]),
                                                                               (uint8_t)(a_srd_bd_addr[0] >> 24),
                                                                               (uint8_t)(a_srd_bd_addr[0] >> 16),
                                                                               (uint8_t)(a_srd_bd_addr[0] >> 8),
                                                                               (uint8_t)(a_srd_bd_addr[0]));
  }
#endif /* CFG_BLE_ADDRESS_TYPE != GAP_PUBLIC_ADDR */

  /**
   * Write Identity root key used to derive IRK and DHK(Legacy)
   */
  ret = aci_hal_write_config_data(CONFIG_DATA_IR_OFFSET, CONFIG_DATA_IR_LEN, FS_State_Get()->ble_irk);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_hal_write_config_data command - CONFIG_DATA_IR_OFFSET, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_hal_write_config_data command - CONFIG_DATA_IR_OFFSET\n");
  }

  /**
   * Write Encryption root key used to derive LTK and CSRK
   */
  ret = aci_hal_write_config_data(CONFIG_DATA_ER_OFFSET, CONFIG_DATA_ER_LEN, FS_State_Get()->ble_erk);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_hal_write_config_data command - CONFIG_DATA_ER_OFFSET, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_hal_write_config_data command - CONFIG_DATA_ER_OFFSET\n");
  }

  /**
   * Set TX Power.
   */
  ret = aci_hal_set_tx_power_level(1, CFG_TX_POWER);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_hal_set_tx_power_level command, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_hal_set_tx_power_level command\n");
  }

  /**
   * Initialize GATT interface
   */
  ret = aci_gatt_init();
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_init command, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_init command\n");
  }

  /**
   * Initialize GAP interface
   */
  role = 0;

#if (BLE_CFG_PERIPHERAL == 1)
  role |= GAP_PERIPHERAL_ROLE;
#endif /* BLE_CFG_PERIPHERAL == 1 */

#if (BLE_CFG_CENTRAL == 1)
  role |= GAP_CENTRAL_ROLE;
#endif /* BLE_CFG_CENTRAL == 1 */

/* USER CODE BEGIN Role_Mngt*/

/* USER CODE END Role_Mngt */

  if (role > 0)
  {
    const char *name = CFG_GAP_DEVICE_NAME;
    ret = aci_gap_init(role,
                       CFG_PRIVACY,
                       CFG_GAP_DEVICE_NAME_LENGTH,
                       &gap_service_handle,
                       &gap_dev_name_char_handle,
                       &gap_appearance_char_handle);

    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("  Fail   : aci_gap_init command, result: 0x%x \n", ret);
    }
    else
    {
      APP_DBG_MSG("  Success: aci_gap_init command\n");
    }

    ret = aci_gatt_update_char_value(gap_service_handle, gap_dev_name_char_handle, 0, strlen(name), (uint8_t *) name);
    if (ret != BLE_STATUS_SUCCESS)
    {
      BLE_DBG_SVCCTL_MSG("  Fail   : aci_gatt_update_char_value - Device Name\n");
    }
    else
    {
      BLE_DBG_SVCCTL_MSG("  Success: aci_gatt_update_char_value - Device Name\n");
    }
  }

  ret = aci_gatt_update_char_value(gap_service_handle,
                                   gap_appearance_char_handle,
                                   0,
                                   2,
                                   (uint8_t *)&a_appearance);
  if (ret != BLE_STATUS_SUCCESS)
  {
    BLE_DBG_SVCCTL_MSG("  Fail   : aci_gatt_update_char_value - Appearance\n");
  }
  else
  {
    BLE_DBG_SVCCTL_MSG("  Success: aci_gatt_update_char_value - Appearance\n");
  }

  /**
   * Initialize Default PHY
   */
  ret = hci_le_set_default_phy(ALL_PHYS_PREFERENCE,TX_2M_PREFERRED,RX_2M_PREFERRED);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : hci_le_set_default_phy command, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: hci_le_set_default_phy command\n");
  }

  /**
   * Initialize IO capability
   */
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.ioCapability = CFG_IO_CAPABILITY;
  ret = aci_gap_set_io_capability(BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.ioCapability);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gap_set_io_capability command, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gap_set_io_capability command\n");
  }

  /**
   * Initialize authentication
   */
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.mitm_mode = CFG_MITM_PROTECTION;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMin = CFG_ENCRYPTION_KEY_SIZE_MIN;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMax = CFG_ENCRYPTION_KEY_SIZE_MAX;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Use_Fixed_Pin = CFG_USED_FIXED_PIN;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Fixed_Pin = CFG_FIXED_PIN;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode = CFG_BONDING_MODE;
  /* USER CODE BEGIN Ble_Hci_Gap_Gatt_Init_1*/
  BleApplicationContext.BleApplicationContext_legacy.gapServiceHandle = gap_service_handle;
  BleApplicationContext.BleApplicationContext_legacy.devNameCharHandle = gap_dev_name_char_handle;
  BleApplicationContext.BleApplicationContext_legacy.appearanceCharHandle = gap_appearance_char_handle;
  APP_BLE_UpdateDeviceName();
  /* USER CODE END Ble_Hci_Gap_Gatt_Init_1*/

  ret = aci_gap_set_authentication_requirement(BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode,
                                               BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.mitm_mode,
                                               CFG_SC_SUPPORT,
                                               CFG_KEYPRESS_NOTIFICATION_SUPPORT,
                                               BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMin,
                                               BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMax,
                                               BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Use_Fixed_Pin,
                                               BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Fixed_Pin,
                                               CFG_IDENTITY_ADDRESS);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gap_set_authentication_requirement command, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gap_set_authentication_requirement command\n");
  }

  /* Record 2, part one: the compile-time security posture, once per boot. Read
   * it together with the ADV lines below — privacy ENABLED plus a whitelist
   * filter is exactly the combination in which a bonded peer that is missing
   * from the RESOLVING list gets its connection request dropped by the
   * controller before the application ever hears about it. */
  FS_BleDiag_Log("CFG privacy=%u id_addr=%u own_addr=%u bonding=%u mitm=%u io=%u sc=%u"
                 " adv_filter=%u central=%u",
      (unsigned) CFG_PRIVACY, (unsigned) CFG_IDENTITY_ADDRESS,
      (unsigned) CFG_BLE_ADDRESS_TYPE, (unsigned) CFG_BONDING_MODE,
      (unsigned) CFG_MITM_PROTECTION, (unsigned) CFG_IO_CAPABILITY,
      (unsigned) CFG_SC_SUPPORT, (unsigned) ADV_FILTER, (unsigned) BLE_CFG_CENTRAL);
  FS_BleDiag_Log("AUTH set_authentication_requirement=0x%02X", (unsigned) ret);

  /**
   * Initialize whitelist
   */
  if (BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode)
  {
    ret = aci_gap_configure_whitelist();
    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("  Fail   : aci_gap_configure_whitelist command, result: 0x%x \n", ret);
    }
    else
    {
      APP_DBG_MSG("  Success: aci_gap_configure_whitelist command\n");
    }

    /* 0x00 = success; 0x47 (BLE_STATUS_FAILED) is what the controller returns
     * when the bond database is EMPTY, which is half of what we are here to
     * decide. */
    FS_BleDiag_Log("WL_INIT configure_whitelist=0x%02X", (unsigned) ret);
  }
  else
  {
    FS_BleDiag_Log("WL_INIT skipped (bonding_mode=0)");
  }
  APP_DBG_MSG("==>> End Ble_Hci_Gap_Gatt_Init function\n\r");
}

static void Adv_Request(APP_BLE_ConnStatus_t NewStatus)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;

  BleApplicationContext.SmartPhone_Connection_Status = NewStatus;
  /* Start Fast or Low Power Advertising */
  ret = aci_gap_set_discoverable(ADV_TYPE,
                                 CFG_FAST_CONN_ADV_INTERVAL_MIN,
                                 CFG_FAST_CONN_ADV_INTERVAL_MAX,
                                 CFG_BLE_ADDRESS_TYPE,
                                 ADV_FILTER,
                                 0,
                                 0,
                                 0,
                                 0,
                                 0,
                                 0);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("==>> aci_gap_set_discoverable - fail, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("==>> aci_gap_set_discoverable - Success\n");
  }

/* USER CODE BEGIN Adv_Request_1*/

/* USER CODE END Adv_Request_1*/

  /* Update Advertising data */
  ret = aci_gap_update_adv_data(sizeof(a_AdvData), (uint8_t*) a_AdvData);
  if (ret != BLE_STATUS_SUCCESS)
  {
      APP_DBG_MSG("==>> Start Fast Advertising Failed , result: %d \n\r", ret);
  }
  else
  {
      APP_DBG_MSG("==>> Success: Start Fast Advertising \n\r");
  }

  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTION */

/* USER CODE END FD_LOCAL_FUNCTION */

/*************************************************************
 *
 * SPECIFIC FUNCTIONS FOR CUSTOM
 *
 *************************************************************/
static void Adv_Cancel(void)
{
  /* USER CODE BEGIN Adv_Cancel_1 */

  /* USER CODE END Adv_Cancel_1 */

  if (BleApplicationContext.SmartPhone_Connection_Status != APP_BLE_CONNECTED)
  {
    tBleStatus ret = BLE_STATUS_INVALID_PARAMS;

    ret = aci_gap_set_non_discoverable();

    BleApplicationContext.SmartPhone_Connection_Status = APP_BLE_IDLE;
    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("** STOP ADVERTISING **  Failed \r\n\r");
    }
    else
    {
      APP_DBG_MSG("  \r\n\r");
      APP_DBG_MSG("** STOP ADVERTISING **  \r\n\r");
    }

    FS_BleDiag_Log("ADV path=stop set_non_discoverable=0x%02X", (unsigned) ret);
  }

  /* USER CODE BEGIN Adv_Cancel_2 */

  /* USER CODE END Adv_Cancel_2 */

  return;
}

#if (L2CAP_REQUEST_NEW_CONN_PARAM != 0)
void BLE_SVC_L2CAP_Conn_Update(uint16_t ConnectionHandle)
{
  /* USER CODE BEGIN BLE_SVC_L2CAP_Conn_Update_1 */

  /* USER CODE END BLE_SVC_L2CAP_Conn_Update_1 */

  if (mutex == 1)
  {
    mutex = 0;
    index_con_int = (index_con_int + 1)%SIZE_TAB_CONN_INT;
    uint16_t interval_min = CONN_P(a_ConnInterval[index_con_int]);
    uint16_t interval_max = CONN_P(a_ConnInterval[index_con_int]);
    uint16_t peripheral_latency = L2CAP_PERIPHERAL_LATENCY;
    uint16_t timeout_multiplier = L2CAP_TIMEOUT_MULTIPLIER;
    tBleStatus ret;

    ret = aci_l2cap_connection_parameter_update_req(BleApplicationContext.connectionHandleCentral,
                                                    interval_min, interval_max,
                                                    peripheral_latency, timeout_multiplier);
    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("BLE_SVC_L2CAP_Conn_Update(), Failed \r\n\r");
    }
    else
    {
      APP_DBG_MSG("BLE_SVC_L2CAP_Conn_Update(), Successfully \r\n\r");
    }
  }

  /* USER CODE BEGIN BLE_SVC_L2CAP_Conn_Update_2 */

  /* USER CODE END BLE_SVC_L2CAP_Conn_Update_2 */

  return;
}
#endif /* L2CAP_REQUEST_NEW_CONN_PARAM != 0 */

#if (L2CAP_REQUEST_NEW_CONN_PARAM != 0)
static void Connection_Interval_Update_Req(void)
{
  if (BleApplicationContext.SmartPhone_Connection_Status != APP_BLE_FAST_ADV && BleApplicationContext.SmartPhone_Connection_Status != APP_BLE_IDLE)
  {
    BLE_SVC_L2CAP_Conn_Update(BleApplicationContext.connectionHandleCentral);
  }

  return;
}
#endif /* L2CAP_REQUEST_NEW_CONN_PARAM != 0 */

/* USER CODE BEGIN FD_SPECIFIC_FUNCTIONS */
static void LinkConfiguration(void)
{
  tBleStatus status;

  /* See AN5289: How to maximize data throughput */
  APP_DBG_MSG("set data length \n");
  status = hci_le_set_data_length(BleApplicationContext.connectionHandleCentral,251,2120);
  if (status != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("set data length command error \n");
  }
}

static void FS_Adv_Request(APP_BLE_ConnStatus_t NewStatus)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  uint16_t Min_Inter, Max_Inter;
  int8_t bonded;

  /* ===== GATEKEEPER: Single point of enforcement ===== */
  if (!FS_State_Get()->enable_ble)
  {
    /* Ensure radio is off and state is consistent */
    if ((BleApplicationContext.SmartPhone_Connection_Status == APP_BLE_FAST_ADV)
        || (BleApplicationContext.SmartPhone_Connection_Status == APP_BLE_LP_ADV))
    {
      aci_gap_set_non_discoverable();
    }
    HW_TS_Stop(Advertising_mgr_timer_Id);
    BleApplicationContext.SmartPhone_Connection_Status = APP_BLE_IDLE;
    FS_BleDiag_Log("ADV path=none reason=enable_ble=0");
    return;
  }
  /* ===== END GATEKEEPER ===== */

  if (NewStatus == APP_BLE_FAST_ADV)
  {
    Min_Inter = CFG_FAST_CONN_ADV_INTERVAL_MIN;
    Max_Inter = CFG_FAST_CONN_ADV_INTERVAL_MAX;
  }
  else
  {
    Min_Inter = CFG_LP_CONN_ADV_INTERVAL_MIN;
    Max_Inter = CFG_LP_CONN_ADV_INTERVAL_MAX;
  }

  /**
   * Stop the timer, it will be restarted for a new shot
   * It does not hurt if the timer was not running
   */
  HW_TS_Stop(Advertising_mgr_timer_Id);

  if ((BleApplicationContext.SmartPhone_Connection_Status == APP_BLE_FAST_ADV)
      || (BleApplicationContext.SmartPhone_Connection_Status == APP_BLE_LP_ADV))
  {
    /* Connection in ADVERTISE mode have to stop the current advertising */
    ret = aci_gap_set_non_discoverable();
    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("==>> aci_gap_set_non_discoverable - Stop Advertising Failed , result: %d \n", ret);
    }
    else
    {
      APP_DBG_MSG("==>> aci_gap_set_non_discoverable - Successfully Stopped Advertising \n");
    }
  }

  /* Call advertising callback */
  if (Adv_Callback) Adv_Callback();

  /* Update advertising callback */
  Adv_Callback = Next_Adv_Callback;
  Next_Adv_Callback = 0;

  BleApplicationContext.SmartPhone_Connection_Status = NewStatus;

  /**
   * Prepare white list as described in PM0271 5.3.1
   */
  bonded = ble_count_bonded_devices();
  (void) bonded;  /* only read by the FS_BLE_DIAG records below */

  if (request_pairing)
  {
    /* Start Fast or Low Power Advertising */
    ret = aci_gap_set_limited_discoverable(ADV_IND,
                                           Min_Inter,
                                           Max_Inter,
                                           CFG_BLE_ADDRESS_TYPE,
                                           ADV_FILTER,
                                           0,
                                           0,
                                           0,
                                           0,
                                           0,
                                           0);
    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("==>> aci_gap_set_limited_discoverable - fail, result: 0x%x \n", ret);
    }
    else
    {
      APP_DBG_MSG("==>> aci_gap_set_limited_discoverable - Success\n");
    }

    /* Record 5, path A: the double-press path. ADV_FILTER is NO_WHITE_LIST_USE,
     * so this advertising is open to anyone — which is why a double-press always
     * gets in and is NOT evidence that the bond database is healthy. */
    FS_BleDiag_Log("ADV path=limited_discoverable filter=%u ret=0x%02X status=%u"
                   " bonds=%d int=%u-%u",
        (unsigned) ADV_FILTER, (unsigned) ret, (unsigned) NewStatus, (int) bonded,
        (unsigned) Min_Inter, (unsigned) Max_Inter);

    /* Update advertising data */
    APP_BLE_UpdateAdvertisingData(NewStatus);
  }
  else
  {
    /* Start Fast or Low Power Advertising */
    ret = aci_gap_set_undirected_connectable(Min_Inter,
                                             Max_Inter,
                                             CFG_BLE_ADDRESS_TYPE,
                                             HCI_ADV_FILTER_WHITELIST_SCAN_CONNECT);
    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("==>> aci_gap_set_undirected_connectable - fail, result: 0x%x \n", ret);
    }
    else
    {
      APP_DBG_MSG("==>> aci_gap_set_undirected_connectable - Success\n");
    }

    /* Record 5, path B: the ordinary path, and the suspect. The filter policy is
     * hard-coded HCI_ADV_FILTER_WHITELIST_SCAN_CONNECT (0x03) — scan requests AND
     * connection requests are accepted only from the whitelist. Compare bonds=
     * here against the WL/BOND/RL lines just above: bonds=0 means the whitelist
     * is empty and the radio will refuse every peer. */
    FS_BleDiag_Log("ADV path=undirected_connectable filter=0x%02X ret=0x%02X status=%u"
                   " bonds=%d int=%u-%u",
        (unsigned) HCI_ADV_FILTER_WHITELIST_SCAN_CONNECT, (unsigned) ret,
        (unsigned) NewStatus, (int) bonded, (unsigned) Min_Inter, (unsigned) Max_Inter);

    /* Update advertising data */
    APP_BLE_UpdateAdvertisingData(NewStatus);
  }

  if (NewStatus == APP_BLE_FAST_ADV)
  {
    HW_TS_Start(Advertising_mgr_timer_Id, FAST_ADV_TIMEOUT);
  }

  return;
}

static void Adv_Update_Req(void)
{
  /**
   * The code shall be executed in the background as an aci command may be sent
   * The background is the only place where the application can make sure a new aci command
   * is not sent if there is a pending one
   */
  UTIL_SEQ_SetTask(1 << CFG_TASK_ADV_UPDATE_ID, CFG_SCH_PRIO_1);
}

static void Adv_Update(void)
{
  FS_Adv_Request(APP_BLE_LP_ADV);
}

void APP_BLE_RequestPairing(void (*Callback)(void))
{
  /* Prepare for pairing request */
  Next_Adv_Callback = Callback;
  request_pairing = 1;

  /* Request fast advertising */
  FS_Adv_Request(APP_BLE_FAST_ADV);
}

void APP_BLE_CancelPairing(void)
{
  /* Prepare for pairing request */
  Next_Adv_Callback = 0;
  request_pairing = 0;

  if (BleApplicationContext.SmartPhone_Connection_Status == APP_BLE_FAST_ADV)
  {
    /* Request low power advertising */
    FS_Adv_Request(APP_BLE_LP_ADV);
  }
}

void APP_BLE_UpdateDeviceName(void)
{
  /**
   * For details of Bluetooth device name characteristic, see:
   * https://www.bluetooth.com/specifications/css-11/
   */

  const char *name = FS_State_Get()->device_name;
  const size_t name_len = strlen(name);
  uint8_t char_length;
  tBleStatus ret;

  if (BleApplicationContext.BleApplicationContext_legacy.gapServiceHandle)
  {
    /* Get device name length */
    char_length = MIN(name_len, 30);

    /* Update device name characteristic */
    ret = aci_gatt_update_char_value(
        BleApplicationContext.BleApplicationContext_legacy.gapServiceHandle,
        BleApplicationContext.BleApplicationContext_legacy.devNameCharHandle,
        0,
        char_length, (uint8_t *) name);
    if (ret != BLE_STATUS_SUCCESS)
    {
      BLE_DBG_SVCCTL_MSG("  Fail   : aci_gatt_update_char_value - Device Name\n");
    }
    else
    {
      BLE_DBG_SVCCTL_MSG("  Success: aci_gatt_update_char_value - Device Name\n");
    }
  }

  /* Update advertising data */
  APP_BLE_UpdateAdvertisingData(BleApplicationContext.SmartPhone_Connection_Status);
}

void APP_BLE_Reset(void)
{
  tBleStatus ret;

  /* Clear list of bonded devices */
  ret = aci_gap_clear_security_db();
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gap_clear_security_db command, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gap_clear_security_db command\n");
  }

  /* Record 1, the part that matters: this line IS the bond wipe. If it appears
   * without the owner having asked for it, that is the bug. */
  FS_BleDiag_Log("BLE_RESET clear_security_db=0x%02X (ALL BONDS ERASED)", (unsigned) ret);

  /* Re-initialize advertising */
  FS_Adv_Request(BleApplicationContext.SmartPhone_Connection_Status);
}
static void APP_BLE_UpdateAdvertisingData(APP_BLE_ConnStatus_t NewStatus)
{
  /**
   * For details of Bluetooth advertised local name, see:
   * https://www.bluetooth.com/specifications/css-11/
   */

  const char *name = FS_State_Get()->device_name;
  const size_t name_len = strlen(name);
  uint8_t mfg_data[] = { 4, AD_TYPE_MANUFACTURER_SPECIFIC_DATA, 0xDB, 0x09, 0x00 };
  uint8_t adv_data[31];
  uint8_t k = 0;
  uint8_t ad_length;
  uint8_t ad_type;
  tBleStatus ret;

  if ((NewStatus == APP_BLE_FAST_ADV) || (NewStatus == APP_BLE_LP_ADV))
  {
    /* Set flag in manufacturer specific data */
    mfg_data[4] = request_pairing;

    /* Copy manufacturer specific data */
    memcpy(&(adv_data[k]), mfg_data, sizeof(mfg_data));
    k += sizeof(mfg_data);

    /* Get short local name if needed */
    if (name_len > sizeof(adv_data) - k - 2)
    {
      ad_length = sizeof(adv_data) - k - 2;
      ad_type = AD_TYPE_SHORTENED_LOCAL_NAME;
    }
    else
    {
      ad_length = name_len;
      ad_type = AD_TYPE_COMPLETE_LOCAL_NAME;
    }

    /* Copy local name */
    adv_data[k++] = 1 + ad_length;
    adv_data[k++] = ad_type;
    memcpy(&(adv_data[k]), name, ad_length);
    k += ad_length;

    /* Update advertising data */
    ret = aci_gap_update_adv_data(k, adv_data);
    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("==>> aci_gap_update_adv_data - fail, result: 0x%x \n", ret);
    }
    else
    {
      APP_DBG_MSG("==>> aci_gap_update_adv_data - Success\n");
    }
  }
}

static int8_t ble_count_bonded_devices(void)
{
  uint8_t total = 0;
  tBleStatus ret;
  Bonded_Device_Entry_t devices[16];
  Identity_Entry_t rl_entries[16];
#if FS_BLE_DIAG
  char addrStr[18];
#endif

  // 1) Rebuild the whitelist from the bonding database (PM0271 5.3.1 step 3)
  ret = aci_gap_configure_whitelist();
  APP_DBG_MSG("\taci_gap_configure_whitelist = 0x%02X\r\n", ret);
  FS_BleDiag_Log("WL configure_whitelist=0x%02X", (unsigned) ret);

  // 2) Fetch bonded identities (PM0271 5.3.1 step 4)
  ret = aci_gap_get_bonded_devices(&total, devices);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("ACI_GAP_GET_BONDED_DEVICES error %x\r\n", ret);
    FS_BleDiag_Log("WL get_bonded=0x%02X FAILED", (unsigned) ret);
    return -1;
  }

  if (total == 0)
  {
    APP_DBG_MSG("No previous bonded devices\r\n");
  }

  /* Record 2's answer in one line: n=0 with the whitelist filter armed means
   * nothing on earth can connect without a double-press. */
  FS_BleDiag_Log("WL get_bonded=0x%02X n=%u", (unsigned) ret, (unsigned) total);

  for (uint8_t k = 0; k < total; k++)
  {
    // Log for visibility
    APP_DBG_MSG("Bonded %u/%u - %s %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                k + 1, total,
                devices[k].Address_Type == 0 ? "Public" : "Random",
                devices[k].Address[5], devices[k].Address[4], devices[k].Address[3],
                devices[k].Address[2], devices[k].Address[1], devices[k].Address[0]);

#if FS_BLE_DIAG
    /* Address TYPE matters as much as the address: the identity the controller
     * stored has to be the identity the peer presents behind its resolvable
     * private address, and a public/random mix-up resolves to nothing. */
    FS_BleDiag_FormatAddr(addrStr, sizeof(addrStr), devices[k].Address);
    FS_BleDiag_Log("BOND %u/%u type=%u addr=%s",
        (unsigned) (k + 1), (unsigned) total,
        (unsigned) devices[k].Address_Type, addrStr);
#endif

    rl_entries[k].Peer_Identity_Address_Type = devices[k].Address_Type;
    memcpy(rl_entries[k].Peer_Identity_Address, devices[k].Address, 6);
  }

  // 3) Add bonded identities to resolving list (PM0271 5.3.1 step 5)
  ret = aci_gap_add_devices_to_resolving_list(total, rl_entries, 1);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("aci_gap_add_devices_to_resolving_list fail %x\r\n", ret);
    FS_BleDiag_Log("RL add_devices=0x%02X n=%u FAILED", (unsigned) ret, (unsigned) total);

    /* 0x0C, command disallowed: the controller refuses resolving-list changes
     * while scanning is enabled (BT spec; adv is already stopped on this
     * path). Seen constantly in BLEDIAG.TXT once the ENGO scan was running —
     * which meant a bond made after boot never reached the RL, so that phone's
     * private address stopped resolving and its whitelist reconnects were
     * refused until the next reboot. Terminate the scan and let Scan_Request
     * redo the rebuild before it starts the next one. */
    if (ret == HCI_COMMAND_DISALLOWED_ERR_CODE && !resolving_list_dirty)
    {
      resolving_list_dirty = 1;
      ret = aci_gap_terminate_gap_proc(GAP_GENERAL_DISCOVERY_PROC);
      FS_BleDiag_Log("RL retry: terminate_scan=0x%02X", (unsigned) ret);
      if (ret != BLE_STATUS_SUCCESS)
      {
        /* No scan to stop after all — retry the rebuild on the spot. */
        resolving_list_dirty = 0;
        ret = aci_gap_add_devices_to_resolving_list(total, rl_entries, 1);
        FS_BleDiag_Log("RL add_devices=0x%02X n=%u (retry)",
                       (unsigned) ret, (unsigned) total);
        if (ret == BLE_STATUS_SUCCESS)
        {
          return (int8_t)total;
        }
      }
      /* else: ACI_GAP_PROC_COMPLETE restarts the scan task, and Scan_Request
       * finishes the rebuild first. */
    }
    return -1;
  }

  APP_DBG_MSG("Resolving List populated with %u bonded device(s)\r\n", total);
  FS_BleDiag_Log("RL add_devices=0x%02X n=%u", (unsigned) ret, (unsigned) total);
  return (int8_t)total;
}

/**
 * @brief  Scan Request
 * @param  None
 * @retval None
 */
static void Scan_Request(void)
{
  /* USER CODE BEGIN Scan_Request_1 */

  /* A resolving-list rebuild was refused with "command disallowed" because
   * this scan was running. Now — after the terminated scan's PROC_COMPLETE and
   * before the next one starts — is the moment it can succeed. When
   * advertising is up too (it also blocks RL changes), FS_Adv_Request redoes
   * the whole stop-rebuild-restart cycle; with the scan not yet running, the
   * rebuild inside it goes through. */
  /* A refused bond removal, retried here and once only: this runs before the
   * scan is started again, so if the controller was refusing because a GAP
   * discovery was running, it will not be now. Either way the return code is
   * recorded and the matter is settled rather than retried forever. */
  if (pending_evict)
  {
    tBleStatus evict_retry = aci_gap_remove_bonded_device(pending_evict_type,
                                                          pending_evict_addr);
    FS_BleDiag_Log("BOND_EVICT retry remove=0x%02X", (unsigned) evict_retry);
    pending_evict = 0;
    if (evict_retry == BLE_STATUS_SUCCESS)
    {
      /* The whitelist still holds the peer until it is rebuilt. */
      resolving_list_dirty = 1;
    }
  }

  if (resolving_list_dirty)
  {
    resolving_list_dirty = 0;
    if ((BleApplicationContext.SmartPhone_Connection_Status == APP_BLE_FAST_ADV)
        || (BleApplicationContext.SmartPhone_Connection_Status == APP_BLE_LP_ADV))
    {
      FS_Adv_Request(BleApplicationContext.SmartPhone_Connection_Status);
    }
    else
    {
      (void) ble_count_bonded_devices();
    }
  }

  /* USER CODE END Scan_Request_1 */
  tBleStatus result;

  if (BleApplicationContext.EndDevice_Connection_Status[0] != APP_BLE_CONNECTED)
  {
    /* USER CODE BEGIN APP_BLE_CONNECTED */
    BleApplicationContext.EndDevice1Found = 0x00;
    /* USER CODE END APP_BLE_CONNECTED */
    result = aci_gap_start_general_discovery_proc(SCAN_P,
                                                  SCAN_L,
                                                  CFG_BLE_ADDRESS_TYPE,
                                                  1);
    if (result == BLE_STATUS_SUCCESS)
    {
    /* USER CODE BEGIN BLE_SCAN_SUCCESS */

    /* USER CODE END BLE_SCAN_SUCCESS */
      APP_DBG_MSG(" \r\n\r** START GENERAL DISCOVERY (SCAN) **  \r\n\r");
    }
    else
    {
    /* USER CODE BEGIN BLE_SCAN_FAILED */

    /* USER CODE END BLE_SCAN_FAILED */
      APP_DBG_MSG("-- BLE_App_Start_Limited_Disc_Req, Failed %02x \r\n\r", result);
    }
  }
  /* USER CODE BEGIN Scan_Request_2 */

  /* USER CODE END Scan_Request_2 */

  return;
}

/**
 * @brief  Connection Establishement on SERVER 1
 * @param  None
 * @retval None
 */
static void Connect_Request(void)
{
  tBleStatus result;
  APP_DBG_MSG("\r\n\r** CREATE CONNECTION TO END DEVICE 1 **  \r\n\r");
  if (BleApplicationContext.EndDevice_Connection_Status[0] != APP_BLE_CONNECTED &&
      BleApplicationContext.EndDevice_Connection_Status[0] != APP_BLE_CONNECTING)
  {
    /* USER CODE BEGIN APP_BLE_CONNECTED_SUCCESS_END_DEVICE_1 */

    /* USER CODE END APP_BLE_CONNECTED_SUCCESS_END_DEVICE_1 */
    result = aci_gap_create_connection(SCAN_P,
                                       SCAN_L,
                                       P2P_SERVER1_ADDR_TYPE,
                                       P2P_SERVER1_BDADDR,
                                       CFG_BLE_ADDRESS_TYPE,
                                       0x000C,   /* conn interval min = 15 ms (ActiveLook prefers 15-30) */
                                       0x0018,   /* conn interval max = 30 ms */
                                       0,
                                       SUPERV_TIMEOUT,
                                       CONN_L1,
                                       CONN_L2);

    if (result == BLE_STATUS_SUCCESS)
    {
      /* USER CODE BEGIN BLE_STATUS_END_DEVICE_1_SUCCESS */

      /* USER CODE END BLE_STATUS_END_DEVICE_1_SUCCESS */
      BleApplicationContext.EndDevice_Connection_Status[0] = APP_BLE_CONNECTING;
      APP_DBG_MSG("==> Connect_Request Succeeded \n\r");
    }
    else
    {
      /* USER CODE BEGIN BLE_STATUS_END_DEVICE_1_FAILED */

      /* USER CODE END BLE_STATUS_END_DEVICE_1_FAILED */
      BleApplicationContext.EndDevice_Connection_Status[0] = APP_BLE_IDLE;
      APP_DBG_MSG("==> Connect_Request Failed, re-scheduling scan\n\r");
      /* Re-arm scanning so we retry when glasses become available */
      UTIL_SEQ_SetTask(1 << CFG_TASK_START_SCAN_ID, CFG_SCH_PRIO_0);
    }
  }

  return;
}

/**
 * @brief  Disconnect on SERVER 1
 * @param  None
 * @retval None
 */
static void Disconnect_Request(void)
{
  tBleStatus result;
  uint16_t connection_handle = BleApplicationContext.connectionHandleEndDevice1;

  APP_DBG_MSG("\r\n\r** DISCONNECT FROM END DEVICE 1 **  \r\n\r");
  if (BleApplicationContext.EndDevice_Connection_Status[0] == APP_BLE_CONNECTED)
  {
    result = aci_gap_terminate(connection_handle, 0x13);
    if (result == BLE_STATUS_SUCCESS)
    {
      BleApplicationContext.EndDevice_Connection_Status[0] = APP_BLE_IDLE;
      APP_DBG_MSG("Disconnection request sent successfully.\n");
    }
    else
    {
      BleApplicationContext.EndDevice_Connection_Status[0] = APP_BLE_IDLE;
      APP_DBG_MSG("Failed to send disconnection request. Error: 0x%02X\n", result);
    }
  }
}

/* USER CODE END FD_SPECIFIC_FUNCTIONS */
/*************************************************************
 *
 * WRAP FUNCTIONS
 *
 *************************************************************/
void hci_notify_asynch_evt(void* p_Data)
{
  UTIL_SEQ_SetTask(1 << CFG_TASK_HCI_ASYNCH_EVT_ID, CFG_SCH_PRIO_0);

  return;
}

void hci_cmd_resp_release(uint32_t Flag)
{
  UTIL_SEQ_SetEvt(1 << CFG_IDLEEVT_HCI_CMD_EVT_RSP_ID);

  return;
}

void hci_cmd_resp_wait(uint32_t Timeout)
{
  UTIL_SEQ_WaitEvt(1 << CFG_IDLEEVT_HCI_CMD_EVT_RSP_ID);

  return;
}

static void BLE_UserEvtRx(void *p_Payload)
{
  SVCCTL_UserEvtFlowStatus_t svctl_return_status;
  tHCI_UserEvtRxParam *p_param;

  p_param = (tHCI_UserEvtRxParam *)p_Payload;

  svctl_return_status = SVCCTL_UserEvtRx((void *)&(p_param->pckt->evtserial));
  if (svctl_return_status != SVCCTL_UserEvtFlowDisable)
  {
    p_param->status = HCI_TL_UserEventFlow_Enable;
  }
  else
  {
    p_param->status = HCI_TL_UserEventFlow_Disable;
  }

  return;
}

static void BLE_StatusNot(HCI_TL_CmdStatus_t Status)
{
  uint32_t task_id_list;
  switch (Status)
  {
    case HCI_TL_CmdBusy:
      /**
       * All tasks that may send an aci/hci commands shall be listed here
       * This is to prevent a new command is sent while one is already pending
       */
      task_id_list = (1 << CFG_LAST_TASK_ID_WITH_HCICMD) - 1;
      UTIL_SEQ_PauseTask(task_id_list);
      /* USER CODE BEGIN HCI_TL_CmdBusy */

      /* USER CODE END HCI_TL_CmdBusy */
      break;

    case HCI_TL_CmdAvailable:
      /**
       * All tasks that may send an aci/hci commands shall be listed here
       * This is to prevent a new command is sent while one is already pending
       */
      task_id_list = (1 << CFG_LAST_TASK_ID_WITH_HCICMD) - 1;
      UTIL_SEQ_ResumeTask(task_id_list);
      /* USER CODE BEGIN HCI_TL_CmdAvailable */

      /* USER CODE END HCI_TL_CmdAvailable */
      break;

    default:
      /* USER CODE BEGIN Status */

      /* USER CODE END Status */
      break;
  }

  return;
}

void SVCCTL_ResumeUserEventFlow(void)
{
  hci_resume_flow();

  return;
}

/* USER CODE BEGIN FD_WRAP_FUNCTIONS */

/* USER CODE END FD_WRAP_FUNCTIONS */
