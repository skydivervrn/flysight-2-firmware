/***************************************************************************
**                                                                        **
**  FlySight 2 firmware — BLE reconnect diagnostic (THROWAWAY BUILD)       **
**                                                                        **
**  This program is free software: you can redistribute it and/or modify  **
**  it under the terms of the GNU General Public License as published by  **
**  the Free Software Foundation, either version 3 of the License, or     **
**  (at your option) any later version.                                   **
**                                                                        **
****************************************************************************/

/* The platform half of the diagnostic: the clock, and the card.
 *
 * Everything that touches FatFs lives here and runs in exactly two places, both
 * of them ordinary task/main context with no sequencer task on the stack:
 *   - Core/Src/app_entry.c, MX_APPE_Process, after UTIL_SEQ_Run has returned;
 *   - FlySight/mode.c, on VBUS_HIGH, just before USB mass storage takes the card.
 * Never from an interrupt, and never from a BLE callback. */

#include <stdio.h>
#include <string.h>

#include "main.h"
#include "app_common.h"
#include "ble_diag.h"
#include "ff.h"
#include "resource_manager.h"

#if FS_BLE_DIAG

#define FS_BLE_DIAG_PATH "/blediag.txt"

static FIL      s_file;
static uint8_t  s_inFlush;
static uint8_t  s_everFlushed;
static uint32_t s_lastFlushMs;

void FS_BleDiag_Log(const char *fmt, ...)
{
	char     line[FS_BLE_DIAG_LINE_LEN];
	va_list  ap;
	uint32_t primask_bit;

	/* Format on the stack first, with interrupts on: the caller's conversions are
	 * the expensive part of this and there is no reason to hold off interrupts
	 * through them. Passing the result on as "%s" also means a device name or an
	 * advertised string carrying a '%' cannot be re-read as a conversion. */
	va_start(ap, fmt);
	vsnprintf(line, sizeof(line), fmt, ap);
	va_end(ap);
	line[sizeof(line) - 1] = '\0';

	/* Guarded: the sequence number, the slot, and the copy — the same shape as
	 * the queue bookkeeping in mode.c. It is one prefix snprintf and a bounded
	 * copy of at most 128 bytes, so single-figure microseconds with interrupts
	 * off. Emits come from sequencer tasks, which cannot preempt one another, so
	 * this is belt-and-braces rather than load-bearing. */
	primask_bit = __get_PRIMASK();
	__disable_irq();
	FS_BleDiag_Emit(HAL_GetTick(), "%s", line);
	__set_PRIMASK(primask_bit);
}

void FS_BleDiag_Flush(void)
{
	const uint32_t now = HAL_GetTick();

	if (FS_BleDiag_Pending() == 0) return;

	/* Hold off unless it has been a while — or unless the ring is filling up, in
	 * which case waiting would start losing records, and a record lost is the
	 * whole exercise wasted. */
	if (s_everFlushed &&
			(FS_BleDiag_Pending() < FS_BLE_DIAG_LINES / 2) &&
			((now - s_lastFlushMs) < FS_BLE_DIAG_FLUSH_MIN_MS))
	{
		return;
	}

	FS_BleDiag_FlushNow();
}

void FS_BleDiag_FlushNow(void)
{
	const char *line;
	uint32_t dropped;

	/* HAL_Delay inside the SD driver spins on WFI and does not re-enter the
	 * sequencer, so this cannot nest today; the guard keeps that from becoming a
	 * silent assumption. */
	if (s_inFlush) return;
	if (FS_BleDiag_Pending() == 0) return;

	s_inFlush = 1;
	s_everFlushed = 1;
	s_lastFlushMs = HAL_GetTick();

	/* Acquire or skip. FatFs is a binary lock in resource_manager.c: the request
	 * fails outright if the log (ACTIVE) or USB mass storage (through MICROSD)
	 * already owns the card, and releasing something we did not acquire would
	 * unmount it under its owner. Skipping costs nothing — the ring keeps the
	 * lines until a later pass gets the card. */
	if (FS_ResourceManager_RequestResource(FS_RESOURCE_FATFS)
			== FS_RESOURCE_MANAGER_SUCCESS)
	{
		if (f_open(&s_file, FS_BLE_DIAG_PATH, FA_WRITE | FA_OPEN_APPEND) == FR_OK)
		{
			while ((line = FS_BleDiag_Peek()) != 0)
			{
				UINT bw = 0;
				UINT len = (UINT) strlen(line);

				if ((f_write(&s_file, line, len, &bw) != FR_OK) || (bw != len)) break;
				if ((f_write(&s_file, "\r\n", 2, &bw) != FR_OK) || (bw != 2)) break;

				/* Only now is the line safely on its way out: a write that failed
				 * leaves it in the ring for the next attempt. */
				FS_BleDiag_Drop();
			}

			/* Record losses AFTER the drain: with drop-newest, anything lost is
			 * younger than everything that made it into the file. */
			if (FS_BleDiag_Pending() == 0)
			{
				dropped = FS_BleDiag_TakeDropped();
				if (dropped != 0)
				{
					char note[48];
					UINT bw = 0;
					int n = snprintf(note, sizeof(note), "LOST n=%lu (ring full)\r\n",
							(unsigned long) dropped);
					if (n > 0) f_write(&s_file, note, (UINT) n, &bw);
				}
			}

			f_close(&s_file);
		}

		FS_ResourceManager_ReleaseResource(FS_RESOURCE_FATFS);
	}

	s_inFlush = 0;
}

#endif /* FS_BLE_DIAG */
