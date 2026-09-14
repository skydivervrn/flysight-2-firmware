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

/* Absolute path: active_mode.c changes the working directory before Load(),
 * so a relative path would read the wrong directory. */
#define ENGO_BIND_PATH   "/engo3.txt"

/* What Load() found on the card, retained for the boot status log. */
typedef enum
{
	ENGO_FILE_IO_ERROR = 0,   /* the card did not tell us                     */
	ENGO_FILE_MISSING,        /* no file at all                                */
	ENGO_FILE_VALID,          /* read cleanly, serial parsed                  */
	ENGO_FILE_INVALID         /* read AND closed cleanly, contents unusable   */
} EngoFileState;

static bool s_bound;
static char s_serial[FS_ENGO_SERIAL_LEN + 1];
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
	/* Deliberately far wider than any form we accept — the longest documented
	 * one, "ENGO 3 123456", is thirteen characters. That margin is what lets
	 * "the buffer ran out" below mean "this is not a binding file" instead of
	 * "we could not read all of a line that might have been fine". Narrow it
	 * and legitimate long-form lines start getting thrown away. */
	char line[64];

	FRESULT fr;
	char   *got;
	bool    parsed = false;

	s_bound        = false;
	s_serial[0]    = '\0';
	s_linkedLogged = false;
	s_fileState    = ENGO_FILE_IO_ERROR;   /* pessimistic until proven otherwise */

	fr = f_open(&f, ENGO_BIND_PATH, FA_READ);
	if (fr != FR_OK)
	{
		/* FR_NO_FILE means the file is missing. Other errors mean the card
		 * could not be read; both leave the device unbound. */
		s_fileState = (fr == FR_NO_FILE) ? ENGO_FILE_MISSING : ENGO_FILE_IO_ERROR;
		return;
	}

	got = f_gets(line, sizeof(line), &f);

	/* Check the error flag BEFORE looking at the buffer, and regardless of what
	 * f_gets returned. FatFs ends with `return n ? buff : 0` (ff.c), so a read
	 * that dies halfway still hands back everything it managed to collect, with
	 * only f_error() to say so. Parsing that stump can misread a good binding:
	 * a short prefix looks invalid, while six or more printable characters can
	 * pass serial_valid() outright — "ID:123456" cut to "ID:123" pins the
	 * device to a serial that does not exist. */
	if (f_error(&f))
	{
		f_close(&f);
		s_fileState = ENGO_FILE_IO_ERROR;
		s_bound = false;
		s_serial[0] = '\0';
		return;
	}

	/* Then prove we have a WHOLE line, which f_gets does not report either. It
	 * stops at sizeof(line)-1 characters and returns cleanly, so a line longer
	 * than the buffer arrives looking exactly like a short one — no error flag,
	 * nothing missing to the naked eye, and the rest of it, including the real
	 * serial in the long forms, still sitting on the card. Taking the last six
	 * characters of that prefix is the read-fault bug all over again without the
	 * fault: "AAAA…A123456" truncates to forty A's and binds the device to
	 * "AAAAAA". That false binding would recur on every boot.
	 *
	 * A line is whole if it ended with a newline, or if the file ended. Filling
	 * sixty-three characters without either means the file is many times longer
	 * than any binding we accept, so it is INVALID. */
	if (got != NULL)
	{
		size_t len = strlen(line);
		if (!(len > 0 && line[len - 1] == '\n') && !f_eof(&f))
		{
			got = NULL;                  /* buffer ran out: not a line we know */
		}
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
		/* Only the "this file is invalid" conclusion needs a clean close. If a
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
		FS_Log_WriteEventAsync("ENGO bind: unbound (engo3.txt %s) -> HUD off, choose glasses in the app",
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
