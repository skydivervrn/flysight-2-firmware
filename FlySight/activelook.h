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

#ifndef ACTIVELOOK_H_
#define ACTIVELOOK_H_

typedef enum {
    FS_AL_SETUP_IN_PROGRESS = 0,
    FS_AL_SETUP_DONE
} FS_ActiveLook_SetupStatus_t;

void FS_ActiveLook_Init(void);
void FS_ActiveLook_DeInit(void);

/* Write the "ENGO HUD v<ver>, AL_Rate <ms>" boot line to the event log.
 * Call AFTER FS_Log_Init (FS_ActiveLook_Init runs before logging is up). */
void FS_ActiveLook_LogBootInfo(void);

/* Called on BLE disconnect: resets the app FSM to idle and disarms the
 * repeating update timer so no writes happen until re-discovery. */
void FS_ActiveLook_OnDisconnect(void);

/* Called from app_ble.c on ACI_GATT_TX_POOL_AVAILABLE — resumes a HUD frame
 * that was paused mid-send when the CPU2 TX pool was momentarily full. */
void FS_ActiveLook_TxPoolAvailable(void);

/* Called from activelook_client.c when CB9 carries 0x01 (flow resume) — sends
 * the rest of a frame the glasses paused mid-way, instead of leaving them held
 * until the next update tick (which may skip if no value changed). */
void FS_ActiveLook_OnFlowResume(void);

#endif /* ACTIVELOOK_H_ */
