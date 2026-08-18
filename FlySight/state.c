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

#include <ctype.h>

#include "main.h"
#include "activelook_mode0.h"
#include "app_ble.h"
#include "ble_diag.h"
#include "common.h"
#include "ff.h"
#include "resource_manager.h"
#include "shci.h"
#include "state.h"
#include "version.h"

static FS_State_Data_t state;
static FIL stateFile;

static void FS_State_Write(void);

static void FS_State_WriteHex_8(FIL *file, const uint8_t *data, uint32_t count)
{
	uint32_t i;

	for (i = 0; i < count; ++i)
	{
		f_printf(file, "%02x", data[i]);
	}
}

static void FS_State_WriteHex_32(FIL *file, const uint32_t *data, uint32_t count)
{
	uint32_t i;

	for (i = 0; i < count; ++i)
	{
		f_printf(file, "%08x", data[i]);
	}
}

static uint32_t hexCharToUint(char c)
{
    if (c >= '0' && c <= '9')
    {
        return c - '0';
    }
    else if (c >= 'a' && c <= 'f')
    {
        return 10 + (c - 'a');
    }
    else if (c >= 'A' && c <= 'F')
    {
        return 10 + (c - 'A');
    }
    return 0; // Optionally handle invalid character error
}

static void FS_State_ReadHex_8(const char *str, uint8_t *data, uint32_t count)
{
    uint32_t i, j;
    uint8_t value;

    for (i = 0; i < count; ++i)
    {
        value = 0;
        for (j = 0; j < 2; ++j)
        {
            value = value << 4;
            value |= hexCharToUint(str[i * 2 + j]);
        }
        data[i] = value;
    }
}

static void FS_State_ReadHex_32(const char *str, uint32_t *data, uint32_t count)
{
    uint32_t i, j, value;

    for (i = 0; i < count; ++i)
    {
        value = 0;
        for (j = 0; j < 8; ++j)
        {
            value = value << 4;
            value |= hexCharToUint(str[i * 8 + j]);
        }
        data[i] = value;
    }
}

static char *trim(char *str) {
    char *start = str;
    char *end;

    // Trim leading whitespace by finding the first non-whitespace character
    while (isspace((unsigned char)*start)) start++;

    if (*start == 0) {  // All spaces?
        return start;
    }

    // Find the end of the string and step back to the last non-whitespace character
    end = start + strlen(start) - 1;
    while (end > start && isspace((unsigned char)*end)) end--;

    // Write new null terminator character
    *(end + 1) = '\0';

    return start;
}

uint8_t is_all_zeros(const void *buffer, size_t size) {
    const unsigned char *byte_buffer = (const unsigned char *)buffer;

    for (size_t i = 0; i < size; i++) {
        if (byte_buffer[i] != 0) {
            return 0;
        }
    }
    return 1;
}

void FS_State_Read(void)
{
	char    buffer[100];
	size_t  len;

	char    *name;
	char    *result;
	int32_t val;
	FRESULT fr;

	/* Initialize persistent state */
	state.config_filename[0] = 0;
	state.temp_folder = 0;
	state.charge_current = 2;
	strcpy(state.device_name, "FlySight");
	state.enable_ble = 1;
	/* 0, not 1. This default is what survives when /flysight.txt cannot be
	 * OPENED — the read below returns early and leaves these values standing —
	 * and reset_ble = 1 means FS_State_Init calls APP_BLE_Reset(), which is
	 * aci_gap_clear_security_db(): every bond on the device, gone. A card that
	 * is briefly busy at boot would therefore make every phone, tablet and Mac
	 * that ever paired with this FlySight pair again, with nothing said about
	 * why. Wiping the bonds is a deliberate act and now has to be asked for, by
	 * writing Reset_BLE: 1 into the file; the file is rewritten with 0 on every
	 * boot (FS_State_Write), so the request is a one-shot exactly as before. */
	state.reset_ble = 0;
	memset(state.session_id, 0, 4 * 3);
	memset(state.ble_irk, 0, CONFIG_DATA_IR_LEN);
	memset(state.ble_erk, 0, CONFIG_DATA_ER_LEN);
	memset(state.ble_addr, 0, sizeof(state.ble_addr));
	state.active_mode = FS_ACTIVE_MODE_DEFAULT;

	fr = f_open(&stateFile, "/flysight.txt", FA_READ);
	if (fr != FR_OK)
	{
		/* The whole point of record 1: this early return is what leaves the
		 * defaults standing, and before b0012cb the default was reset_ble = 1 —
		 * i.e. a card that was merely busy at boot wiped every bond. If this line
		 * ever appears in the file, the state read did NOT happen. */
		FS_BleDiag_Log("STATE open=FAIL fr=%u defaults reset_ble=%u enable_ble=%u",
				(unsigned) fr, (unsigned) state.reset_ble, (unsigned) state.enable_ble);
		return;
	}

	while (!f_eof(&stateFile))
	{
		f_gets(buffer, sizeof(buffer), &stateFile);

		len = strcspn(buffer, ";");
		buffer[len] = '\0';

		len = strcspn(buffer, ":");
		buffer[len] = '\0';

		name = trim(buffer);
		result = trim(buffer + len + 1);

		val = atol(result);

		if (!strcmp(name, "Session_ID") && (strlen(result) == 8 * 3))
		{
			FS_State_ReadHex_32(result, state.session_id, 3);
		}

		if (!strcmp(name, "Config_File"))
		{
			result[12] = '\0';
			strncpy(state.config_filename, result, sizeof(state.config_filename) - 1);
		}

		if (!strcmp(name, "Device_Name"))
		{
			result[30] = '\0';
			strncpy(state.device_name, result, sizeof(state.device_name) - 1);
		}

		#define HANDLE_VALUE(s,w,r,t) \
			if ((t) && !strcmp(name, (s))) { (w) = (r); }

		HANDLE_VALUE("Temp_Folder", state.temp_folder,    val, val >= 0);
		HANDLE_VALUE("Charging",    state.charge_current, val, val >= 0 && val <= 3);
		HANDLE_VALUE("Enable_BLE",  state.enable_ble,     val, val == 0 || val == 1);
		HANDLE_VALUE("Reset_BLE",   state.reset_ble,      val, val == 0 || val == 1);
		HANDLE_VALUE("Active_Mode", state.active_mode,    val, val >= 0 && val < FS_NUM_ACTIVE_MODES);

		if (!strcmp(name, "BLE_IRK") && (strlen(result) == 2 * CONFIG_DATA_IR_LEN))
		{
			FS_State_ReadHex_8(result, state.ble_irk, CONFIG_DATA_IR_LEN);
		}

		if (!strcmp(name, "BLE_ERK") && (strlen(result) == 2 * CONFIG_DATA_ER_LEN))
		{
			FS_State_ReadHex_8(result, state.ble_erk, CONFIG_DATA_ER_LEN);
		}

		if (!strcmp(name, "BLE_Addr") && (strlen(result) == 2 * sizeof(state.ble_addr)))
		{
			FS_State_ReadHex_8(result, state.ble_addr, sizeof(state.ble_addr));
		}

		#undef HANDLE_VALUE
	}

	f_close(&stateFile);

	/* Get device ID */
	state.device_id[0] = HAL_GetUIDw0();
	state.device_id[1] = HAL_GetUIDw1();
	state.device_id[2] = HAL_GetUIDw2();

	/* Initialize IRK if needed */
	while (is_all_zeros(state.ble_irk, CONFIG_DATA_IR_LEN))
	{
		FS_Common_GetRandomBytes((uint32_t *) state.ble_irk, CONFIG_DATA_IR_LEN / 4);
	}

	/* Initialize ERK if needed */
	while (is_all_zeros(state.ble_erk, CONFIG_DATA_ER_LEN))
	{
		FS_Common_GetRandomBytes((uint32_t *) state.ble_erk, CONFIG_DATA_ER_LEN / 4);
	}

	/* Initialize the identity address if needed.
	 *
	 * This is the fix for the reconnect bug. app_ble.c drew this address from
	 * the RNG on every power-up — the comment above that code claims it comes
	 * from the UDN, but CFG_STATIC_RANDOM_ADDRESS is not defined, so the else
	 * branch ran and the FlySight introduced itself under a NEW identity after
	 * every reboot. A central stores identity plus keys when it bonds; after the
	 * next power cycle the identity it stored no longer existed, which is why a
	 * pairing worked once and its reconnect did not, and why macOS ended up
	 * holding several records for one device. Generated once, kept in the state
	 * file beside the IRK, and stable for the life of the card from then on. */
	while (is_all_zeros(state.ble_addr, sizeof(state.ble_addr)))
	{
		uint32_t words[2];
		FS_Common_GetRandomBytes(words, 2);
		memcpy(state.ble_addr, words, sizeof(state.ble_addr));
		/* A static random address must have both top bits set (Core spec,
		 * Vol 6 Part B 1.3.2.1); the byte order here is the little-endian one
		 * the controller is given, so the most significant byte is the last. */
		state.ble_addr[5] |= 0xC0;
	}

	/* The IRK is what lets a bonded peer resolve OUR advertising address, and it
	 * is re-read from the card on every boot — so log its first bytes. If it ever
	 * changes between two boots, every phone's bond is stale on the phone's side
	 * regardless of what our own bond database says. */
	FS_BleDiag_Log("STATE open=OK reset_ble=%u enable_ble=%u name='%s' irk=%02X%02X%02X%02X",
			(unsigned) state.reset_ble, (unsigned) state.enable_ble, state.device_name,
			state.ble_irk[0], state.ble_irk[1], state.ble_irk[2], state.ble_irk[3]);
}

static void FS_State_Write(void)
{
	const uint8_t * const pubkey = (uint8_t *) 0x08018200;

	WirelessFwInfo_t WirelessInfo;

	// Read the firmware version of both the wireless stack and the FUS
	if (SHCI_GetWirelessFwInfo(&WirelessInfo) != SHCI_Success)
	{
		Error_Handler();
	}

	// Open FlySight info file
	if (f_open(&stateFile, "/flysight.txt", FA_WRITE|FA_CREATE_ALWAYS) != FR_OK)
	{
		Error_Handler();
	}

	// Write device info
	f_printf(&stateFile, "; FlySight - http://flysight.ca\n\n");

	f_printf(&stateFile, "; Firmware version\n\n");

	f_printf(&stateFile, "FUS_Ver:      %u.%u.%u\n",
			WirelessInfo.FusVersionMajor, WirelessInfo.FusVersionMinor, WirelessInfo.FusVersionSub);
	f_printf(&stateFile, "Stack_Ver:    %u.%u.%u\n",
			WirelessInfo.VersionMajor, WirelessInfo.VersionMinor, WirelessInfo.VersionSub);
	f_printf(&stateFile, "Firmware_Ver: %s\n\n", GIT_TAG);

	f_printf(&stateFile, "; Device information\n\n");

    f_printf(&stateFile, "Device_ID:    ");
	FS_State_WriteHex_32(&stateFile, state.device_id, 3);
	f_printf(&stateFile, "\n");

	f_printf(&stateFile, "Session_ID:   ");
	FS_State_WriteHex_32(&stateFile, state.session_id, 3);
	f_printf(&stateFile, "\n\n");

	f_printf(&stateFile, "; Persistent state\n\n");

	f_printf(&stateFile, "Config_File:  %s\n", state.config_filename);
	f_printf(&stateFile, "Temp_Folder:  %04lu\n\n", state.temp_folder);

	f_printf(&stateFile, "; Charging\n\n");

	f_printf(&stateFile, "Charging:     %u ; 0 = No charging\n", state.charge_current);
	f_printf(&stateFile, "                ; 1 = 100 mA\n");
	f_printf(&stateFile, "                ; 2 = 200 mA (recommended)\n");
	f_printf(&stateFile, "                ; 3 = 300 mA\n\n");

	f_printf(&stateFile, "; Bluetooth\n\n");

	f_printf(&stateFile, "Enable_BLE:   %u\n", state.enable_ble);
	f_printf(&stateFile, "Reset_BLE:    0\n");
	f_printf(&stateFile, "Device_Name:  %s\n\n", state.device_name);

	f_printf(&stateFile, "BLE_IRK:      ");
	FS_State_WriteHex_8(&stateFile, state.ble_irk, 16);
	f_printf(&stateFile, "\n");

	f_printf(&stateFile, "BLE_ERK:      ");
	FS_State_WriteHex_8(&stateFile, state.ble_erk, 16);
	f_printf(&stateFile, "\n");

	f_printf(&stateFile, "BLE_Addr:     ");
	FS_State_WriteHex_8(&stateFile, state.ble_addr, sizeof(state.ble_addr));
	f_printf(&stateFile, "\n\n");

	f_printf(&stateFile, "; Active mode\n\n");

	f_printf(&stateFile, "Active_Mode:  %u ; 0 = Default\n", state.active_mode);
	f_printf(&stateFile, "                ; 1 = Starter pistol\n\n");

	f_printf(&stateFile, "; Bootloader public key\n\n");

	f_printf(&stateFile, "Pubkey_X:     ");
	FS_State_WriteHex_8(&stateFile, pubkey, 32);
	f_printf(&stateFile, "\n");

	f_printf(&stateFile, "Pubkey_Y:     ");
	FS_State_WriteHex_8(&stateFile, pubkey + 32, 32);
	f_printf(&stateFile, "\n");

	// Close FlySight info file
	f_close(&stateFile);
}

void FS_State_Init(void)
{
	/* First record of a session. HAL_GetTick restarts at 0 on every boot and
	 * /BLEDIAG.TXT is appended to, so this line is what separates one power-up
	 * from the next when reading the file. */
	FS_BleDiag_Log("BOOT fw=%s hud=%s", GIT_TAG, FS_ActiveLook_Mode0_HudVersion());

	/* Initialize microSD */
	FS_ResourceManager_RequestResource(FS_RESOURCE_FATFS);

	/* Read persistent state */
	FS_State_Read();

	/* Write updated state */
	FS_State_Write();

	/* De-initialize microSD */
	FS_ResourceManager_ReleaseResource(FS_RESOURCE_FATFS);
}

void FS_State_Update(void)
{
	/* Initialize microSD */
	FS_ResourceManager_RequestResource(FS_RESOURCE_FATFS);

	/* Read persistent state */
	FS_State_Read();

	/* Write updated state */
	FS_State_Write();

	/* De-initialize microSD */
	FS_ResourceManager_ReleaseResource(FS_RESOURCE_FATFS);

	/* Record 1, second half. Note what the code actually does, which is not what
	 * the file name suggests: reset_ble is acted on HERE and nowhere else, and
	 * FS_State_Update is reached only from FS_USBMode_DeInit — i.e. the bond wipe
	 * happens when USB is UNPLUGGED, never at boot. Setting Reset_BLE: 1 and
	 * power-cycling therefore does nothing; FS_State_Init reads the flag and
	 * FS_State_Write immediately writes 0 back over it. */
	FS_BleDiag_Log("STATE_UPDATE reset_ble=%u ble_reset=%s",
			(unsigned) state.reset_ble, state.reset_ble ? "YES" : "no");

	if (state.reset_ble)
	{
		/* Clear list of bonded devices */
		APP_BLE_Reset();
	}

	/* Update device name */
	APP_BLE_UpdateDeviceName();
}

const FS_State_Data_t *FS_State_Get(void)
{
	return &state;
}

void FS_State_NextSession(void)
{
	/* Increment temporary folder number */
	state.temp_folder = (state.temp_folder + 1) % 10000;

	/* Get random session ID */
	do
	{
		FS_Common_GetRandomBytes(state.session_id, 3);
	}
	while (is_all_zeros(state.session_id, 4 * 3));

	/* Write updated state */
	FS_State_Write();
}

void FS_State_SetConfigFilename(const char *filename)
{
	/* Update config filename */
	strncpy(state.config_filename, filename, sizeof(state.config_filename));

	/* Write updated state */
	FS_State_Write();
}
