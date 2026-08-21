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
****************************************************************************/

#include "engo_bind.h"
#include "ff.h"
#include "log.h"
#include <string.h>

/* ABSOLUTE path: active_mode.c does f_chdir("/config") before Load(), and the
 * HUD tick (CommitIfPending) can run with CWD=/audio — a relative path would
 * read/write the wrong directory. */
#define ENGO_BIND_PATH   "/engo3.txt"

static bool s_bound;
static char s_serial[FS_ENGO_SERIAL_LEN + 1];
static bool s_pending;
static char s_pendingSerial[FS_ENGO_SERIAL_LEN + 1];
static bool s_linkedLogged;   /* per-session: "linked" line already emitted */

/* A serial is valid if all FS_ENGO_SERIAL_LEN chars are printable non-space. */
static bool serial_valid(const char *s)
{
	for (int i = 0; i < FS_ENGO_SERIAL_LEN; i++)
	{
		char c = s[i];
		if (c <= ' ' || c > '~') return false;
	}
	return true;
}

void FS_EngoBind_Load(void)
{
	FIL  f;
	char line[40];

	s_bound        = false;
	s_pending      = false;
	s_serial[0]    = '\0';
	s_pendingSerial[0] = '\0';
	s_linkedLogged = false;

	if (f_open(&f, ENGO_BIND_PATH, FA_READ) != FR_OK)
	{
		/* No file -> unbound -> first-boot (connect to first) behaviour. */
		return;
	}

	if (f_gets(line, sizeof(line), &f) != NULL)
	{
		/* Trim trailing newline/space, then take the LAST 6 chars. This accepts
		 * "123456", "ID: 123456" and even a full "ENGO 3 123456" line. */
		int n = (int)strlen(line);
		while (n > 0 && (line[n - 1] == '\n' || line[n - 1] == '\r' ||
		                 line[n - 1] == ' '  || line[n - 1] == '\t'))
		{
			n--;
		}
		if (n >= FS_ENGO_SERIAL_LEN)
		{
			memcpy(s_serial, &line[n - FS_ENGO_SERIAL_LEN], FS_ENGO_SERIAL_LEN);
			s_serial[FS_ENGO_SERIAL_LEN] = '\0';
			if (serial_valid(s_serial))
			{
				s_bound = true;
			}
			else
			{
				s_serial[0] = '\0';
			}
		}
	}

	f_close(&f);
	/* NB: do NOT log here — Load() runs before FS_Log_Init(), so any event would
	 * be dropped. Boot-time status is logged separately via FS_EngoBind_LogStatus()
	 * after logging is active. */
}

void FS_EngoBind_LogStatus(void)
{
	if (s_bound)
		FS_Log_WriteEventAsync("ENGO bind: pinned to serial %s (engo3.txt)", s_serial);
	else
		FS_Log_WriteEventAsync("ENGO bind: unbound (no/invalid engo3.txt) -> connect to first");
}

void FS_EngoBind_LogLinkedOnce(void)
{
	if (s_linkedLogged) return;
	s_linkedLogged = true;
	if (s_bound)
		FS_Log_WriteEventAsync("ENGO bind: linked to glasses serial %s", s_serial);
	else
		FS_Log_WriteEventAsync("ENGO bind: linked to glasses (unbound)");
}

bool FS_EngoBind_IsBound(void)
{
	return s_bound;
}

const char *FS_EngoBind_Serial(void)
{
	return s_serial;
}

bool FS_EngoBind_SerialMatches(const char *cand6)
{
	if (!s_bound || cand6 == NULL) return false;
	return memcmp(s_serial, cand6, FS_ENGO_SERIAL_LEN) == 0;
}

void FS_EngoBind_NotePending(const char *cand6)
{
	if (s_bound || cand6 == NULL) return;
	if (!serial_valid(cand6))     return;
	memcpy(s_pendingSerial, cand6, FS_ENGO_SERIAL_LEN);
	s_pendingSerial[FS_ENGO_SERIAL_LEN] = '\0';
	s_pending = true;
}

void FS_EngoBind_CommitIfPending(void)
{
	FIL     f;
	UINT    bw = 0;
	FRESULT fr;

	if (s_bound || !s_pending) return;

	/* FA_CREATE_NEW: never clobber an existing file (binding changes only when
	 * the user deletes /engo3.txt). */
	fr = f_open(&f, ENGO_BIND_PATH, FA_CREATE_NEW | FA_WRITE);
	if (fr != FR_OK)
	{
		FS_Log_WriteEventAsync("ENGO bind: engo3.txt open failed (%d), not bound", (int)fr);
	}
	else
	{
		/* Every result and every byte count is checked. A full card, a write
		 * error or a short write used to be invisible: s_bound went true, the
		 * session behaved as if it were pinned to these glasses, and only the
		 * next boot found out — an engo3.txt that fails to parse reads as
		 * "unbound", which means connect to whichever glasses answer first. */
		fr = f_write(&f, s_pendingSerial, FS_ENGO_SERIAL_LEN, &bw);
		if (fr == FR_OK && bw == FS_ENGO_SERIAL_LEN)
		{
			fr = f_write(&f, "\n", 1, &bw);
			if (fr == FR_OK && bw != 1) fr = FR_DISK_ERR;   /* short write */
		}
		else if (fr == FR_OK)
		{
			fr = FR_DISK_ERR;                                /* short write */
		}

		if (fr == FR_OK) fr = f_sync(&f);

		/* Close whatever happened above, and let a close error condemn the file
		 * too: FatFs only flushes the last sector and the directory entry here,
		 * so a close that fails means the bytes are not on the card. */
		{
			FRESULT cr = f_close(&f);
			if (fr == FR_OK) fr = cr;
		}

		if (fr == FR_OK)
		{
			memcpy(s_serial, s_pendingSerial, FS_ENGO_SERIAL_LEN + 1);
			s_bound = true;
			FS_Log_WriteEventAsync("ENGO bind: learned serial %s -> wrote engo3.txt", s_serial);
		}
		else
		{
			FS_Log_WriteEventAsync("ENGO bind: engo3.txt write failed (%d), not bound", (int)fr);
		}
	}

	/* Either we wrote it, or the file already exists / write failed — stop trying
	 * regardless so we don't hammer the FS every tick. */
	s_pending = false;
}
