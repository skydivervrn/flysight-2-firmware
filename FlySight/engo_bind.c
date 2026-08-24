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

/* What Load() found on the card. "Not bound" alone is NOT a licence to
 * overwrite: a card that would not answer looks exactly like a card holding
 * garbage, and treating the two alike would let a transient read error re-pin
 * the device to whatever glasses happen to be switched on nearby. Only a file
 * that was read AND closed cleanly, and still failed to parse, is known to be
 * rubbish and therefore safe to replace. */
typedef enum
{
	ENGO_FILE_IO_ERROR = 0,   /* the card did not tell us — never write       */
	ENGO_FILE_MISSING,        /* no file at all — create one                  */
	ENGO_FILE_VALID,          /* read cleanly, serial parsed                  */
	ENGO_FILE_INVALID         /* read AND closed cleanly, contents unusable   */
} EngoFileState;

static bool s_bound;
static char s_serial[FS_ENGO_SERIAL_LEN + 1];
static bool s_pending;
static char s_pendingSerial[FS_ENGO_SERIAL_LEN + 1];
static bool s_linkedLogged;   /* per-session: "linked" line already emitted */
static EngoFileState s_fileState;

static const char *file_state_name(EngoFileState st)
{
	switch (st)
	{
	case ENGO_FILE_MISSING: return "missing";
	case ENGO_FILE_VALID:   return "valid";
	case ENGO_FILE_INVALID: return "invalid";
	default:                return "io-error";
	}
}

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

	FRESULT fr;
	char   *got;
	bool    parsed = false;

	s_bound        = false;
	s_pending      = false;
	s_serial[0]    = '\0';
	s_pendingSerial[0] = '\0';
	s_linkedLogged = false;
	s_fileState    = ENGO_FILE_IO_ERROR;   /* pessimistic until proven otherwise */

	fr = f_open(&f, ENGO_BIND_PATH, FA_READ);
	if (fr != FR_OK)
	{
		/* FR_NO_FILE is the only "there is genuinely no binding" answer. Every
		 * other FRESULT means the card could not be asked, which is not the
		 * same thing and must not license a rewrite. */
		s_fileState = (fr == FR_NO_FILE) ? ENGO_FILE_MISSING : ENGO_FILE_IO_ERROR;
		return;
	}

	got = f_gets(line, sizeof(line), &f);

	/* Check the error flag BEFORE looking at the buffer, and regardless of what
	 * f_gets returned. FatFs ends with `return n ? buff : 0` (ff.c), so a read
	 * that dies halfway still hands back everything it managed to collect, with
	 * only f_error() to say so. Parsing that stump is how a good binding gets
	 * destroyed two different ways: a short prefix looks like a corrupt file and
	 * invites FA_CREATE_ALWAYS over a file that was fine, and a prefix of six or
	 * more printable characters can pass serial_valid() outright — "ID:123456"
	 * cut to "ID:123" pins the device to a serial that does not exist. */
	if (f_error(&f))
	{
		f_close(&f);
		s_fileState = ENGO_FILE_IO_ERROR;
		s_bound = false;
		s_serial[0] = '\0';
		return;
	}

	if (got != NULL)
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
				parsed  = true;
			}
			else
			{
				s_serial[0] = '\0';
			}
		}
	}

	fr = f_close(&f);
	if (fr != FR_OK && !parsed)
	{
		/* Only the "this file is rubbish" conclusion needs a clean close. If a
		 * good serial was already parsed we keep it — bytes in RAM do not stop
		 * being right because unmounting the handle went wrong — but we refuse
		 * to call an unparsed file invalid on the word of a card that just
		 * failed. */
		s_fileState = ENGO_FILE_IO_ERROR;
		return;
	}

	s_fileState = parsed ? ENGO_FILE_VALID : ENGO_FILE_INVALID;
	/* NB: do NOT log here — Load() runs before FS_Log_Init(), so any event would
	 * be dropped. Boot-time status is logged separately via FS_EngoBind_LogStatus()
	 * after logging is active. */
}

void FS_EngoBind_LogStatus(void)
{
	if (s_bound)
		FS_Log_WriteEventAsync("ENGO bind: pinned to serial %s (engo3.txt)", s_serial);
	else
		FS_Log_WriteEventAsync("ENGO bind: unbound (engo3.txt %s) -> connect to first",
		                       file_state_name(s_fileState));
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

	int mode;

	if (s_bound || !s_pending) return;

	/* What we are allowed to do depends on what Load() actually established.
	 *
	 * MISSING -> FA_CREATE_NEW, the original behaviour: a real binding is never
	 * clobbered, it changes only when the user deletes /engo3.txt.
	 *
	 * INVALID -> FA_CREATE_ALWAYS. This is the self-heal, and it is what stops
	 * a half-written file from bricking the binding forever: a write that dies
	 * (a full card, or the button held down mid-write) used to leave a stub
	 * that Load() rejects and that FA_CREATE_NEW then refuses to replace,
	 * FR_EXIST after FR_EXIST, until someone deleted it over USB. A stub is not
	 * a binding, so replacing it loses nothing.
	 *
	 * IO_ERROR -> refuse. We do not know what is on the card, and guessing
	 * would silently re-pin the device to whichever glasses answered first. */
	switch (s_fileState)
	{
	case ENGO_FILE_MISSING: mode = FA_CREATE_NEW;    break;
	case ENGO_FILE_INVALID: mode = FA_CREATE_ALWAYS; break;
	default:
		FS_Log_WriteEventAsync("ENGO bind: engo3.txt state %s, refusing to write",
		                       file_state_name(s_fileState));
		s_pending = false;
		return;
	}

	fr = f_open(&f, ENGO_BIND_PATH, mode | FA_WRITE);
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
			s_fileState = ENGO_FILE_VALID;
			FS_Log_WriteEventAsync("ENGO bind: learned serial %s -> wrote engo3.txt", s_serial);
		}
		else
		{
			/* We opened the file, so whatever is on the card now is our own
			 * half-written stub — known rubbish, and the next attempt (this
			 * session or after a reboot) is allowed to replace it. */
			s_fileState = ENGO_FILE_INVALID;
			FS_Log_WriteEventAsync("ENGO bind: engo3.txt write failed (%d), not bound", (int)fr);
		}
	}

	/* Either we wrote it, or the file already exists / write failed — stop trying
	 * regardless so we don't hammer the FS every tick. */
	s_pending = false;
}
