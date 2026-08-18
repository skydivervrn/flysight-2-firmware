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

/* The pure half of the diagnostic: a fixed ring of formatted lines and nothing
 * else. No HAL, no FatFs, no clock — the caller supplies the timestamp — so
 * Tests/test_ble_diag.c compiles this file straight from the host. */

#include <stdio.h>
#include <string.h>

#include "ble_diag.h"

#if FS_BLE_DIAG

static char     s_line[FS_BLE_DIAG_LINES][FS_BLE_DIAG_LINE_LEN];
static uint16_t s_head;     /* slot of the oldest buffered line */
static uint16_t s_count;    /* number of buffered lines */
static uint32_t s_seq;      /* records attempted, including dropped ones */
static uint32_t s_dropped;  /* records dropped since the last TakeDropped */

void FS_BleDiag_Reset(void)
{
	s_head = 0;
	s_count = 0;
	s_seq = 0;
	s_dropped = 0;
}

uint32_t FS_BleDiag_Pending(void)
{
	return s_count;
}

uint32_t FS_BleDiag_TakeDropped(void)
{
	uint32_t n = s_dropped;
	s_dropped = 0;
	return n;
}

const char *FS_BleDiag_Peek(void)
{
	if (s_count == 0) return 0;
	return s_line[s_head];
}

void FS_BleDiag_Drop(void)
{
	if (s_count == 0) return;
	s_head = (uint16_t)((s_head + 1) % FS_BLE_DIAG_LINES);
	--s_count;
}

uint32_t FS_BleDiag_FormatAddr(char *out, uint32_t outLen, const uint8_t *addr)
{
	static const char hex[] = "0123456789ABCDEF";
	uint32_t k = 0;
	int i;

	if (out == 0 || outLen < 18) return 0;

	/* addr[5] first: the BT spec prints an address most-significant byte first,
	 * and so does every scanner, so this is the form that can be compared by eye
	 * against nRF Connect and against the Mac's own log. */
	for (i = 5; i >= 0; --i)
	{
		out[k++] = hex[(addr[i] >> 4) & 0x0F];
		out[k++] = hex[addr[i] & 0x0F];
		if (i > 0) out[k++] = ':';
	}
	out[k] = '\0';
	return k;
}

void FS_BleDiag_EmitV(uint32_t ms, const char *fmt, va_list ap)
{
	uint16_t slot;
	char *dst;
	int n;
	uint32_t i;

	/* The sequence number is spent whether or not the record fits, so the gap it
	 * leaves in the file is the record that was lost. */
	const uint32_t seq = s_seq++;

	if (s_count >= FS_BLE_DIAG_LINES)
	{
		++s_dropped;
		return;
	}

	slot = (uint16_t)((s_head + s_count) % FS_BLE_DIAG_LINES);
	dst = s_line[slot];

	n = snprintf(dst, FS_BLE_DIAG_LINE_LEN, "%06lu %08lu ",
			(unsigned long) (seq % 1000000UL), (unsigned long) ms);
	if (n < 0) n = 0;
	if (n > FS_BLE_DIAG_LINE_LEN - 1) n = FS_BLE_DIAG_LINE_LEN - 1;

	vsnprintf(dst + n, (size_t) (FS_BLE_DIAG_LINE_LEN - n), fmt, ap);
	dst[FS_BLE_DIAG_LINE_LEN - 1] = '\0';

	/* The file is read over USB mass storage in a text editor, so keep it to
	 * printable ASCII: a device name straight off the air could hold anything,
	 * and one stray newline would silently split a record in two. */
	for (i = 0; dst[i] != '\0'; ++i)
	{
		if (dst[i] < 0x20 || dst[i] > 0x7E) dst[i] = '.';
	}

	++s_count;
}

void FS_BleDiag_Emit(uint32_t ms, const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	FS_BleDiag_EmitV(ms, fmt, ap);
	va_end(ap);
}

#endif /* FS_BLE_DIAG */
