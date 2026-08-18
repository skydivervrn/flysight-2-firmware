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

#ifndef STATE_H_
#define STATE_H_

#include "ble.h"
#include "charge.h"

typedef enum
{
	FS_ACTIVE_MODE_DEFAULT,
	FS_ACTIVE_MODE_START,
	FS_NUM_ACTIVE_MODES
} FS_State_ActiveMode_t;

typedef struct
{
	uint32_t device_id[3];
	uint32_t session_id[3];
	char     config_filename[13];
	uint32_t temp_folder;
	FS_Charge_Current_t charge_current;
	char     device_name[30];
	uint8_t  enable_ble;
	uint8_t  reset_ble;
	uint8_t  ble_irk[CONFIG_DATA_IR_LEN];
	uint8_t  ble_erk[CONFIG_DATA_ER_LEN];
	/* The static random address this device answers to — its identity as far as
	 * every bonded peer is concerned. Persisted for exactly the reason the IRK
	 * is: app_ble.c drew it from the RNG on every power-up, so the FlySight
	 * introduced itself under a new identity after every reboot. */
	uint8_t  ble_addr[6];
	/* True when the address above had to be generated, i.e. this is the first
	 * boot of a card that has never carried one. Not persisted: it describes
	 * this boot, not the device. The BLE layer uses it to clear the bonding
	 * table exactly once, because every key in it was agreed with the identity
	 * the device has just stopped using. */
	uint8_t  ble_addr_is_new;
	FS_State_ActiveMode_t active_mode;
} FS_State_Data_t;

void FS_State_Init(void);
void FS_State_Update(void);
const FS_State_Data_t *FS_State_Get(void);
void FS_State_NextSession(void);
void FS_State_SetConfigFilename(const char *filename);

#endif /* STATE_H_ */
