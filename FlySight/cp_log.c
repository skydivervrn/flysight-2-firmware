/***************************************************************************
**                                                                        **
**  FlySight 2 firmware — control-point request trace for EVENT.CSV       **
**                                                                        **
**  This program is free software: you can redistribute it and/or modify  **
**  it under the terms of the GNU General Public License as published by  **
**  the Free Software Foundation, either version 3 of the License, or     **
**  (at your option) any later version.                                   **
**                                                                        **
****************************************************************************/

#include <stdio.h>
#include <string.h>

#include "cp_log.h"

static FS_CpLog_Record_t s_ring[FS_CPLOG_DEPTH];
static uint32_t s_head;      /* records added since reset (next write index) */
static uint32_t s_tail;      /* records taken or dropped (next read index)   */
static uint32_t s_seq;       /* last sequence number handed out              */
static uint32_t s_dropped;   /* dropped since the last notice                */

void FS_CpLog_Reset(void)
{
	memset(s_ring, 0, sizeof(s_ring));
	s_head = s_tail = s_seq = s_dropped = 0;
}

static FS_CpLog_Record_t *find(uint32_t seq)
{
	if (seq == 0) return NULL;
	for (uint32_t i = s_tail; i != s_head; ++i)
	{
		FS_CpLog_Record_t *r = &s_ring[i % FS_CPLOG_DEPTH];
		if (r->seq == seq) return r;
	}
	return NULL;
}

uint32_t FS_CpLog_Add(uint8_t opcode, uint8_t status, uint8_t mode, bool deferred)
{
	if (s_head - s_tail >= FS_CPLOG_DEPTH)
	{
		s_tail++;
		s_dropped++;
	}

	FS_CpLog_Record_t *r = &s_ring[s_head % FS_CPLOG_DEPTH];
	if (++s_seq == 0) s_seq = 1;          /* 0 means "empty" */
	r->seq    = s_seq;
	r->opcode = opcode;
	r->status = status;
	r->mode   = mode;
	r->run    = deferred ? FS_CPLOG_RUN_PENDING : FS_CPLOG_RUN_NONE;
	s_head++;
	return s_seq;
}

void FS_CpLog_Amend(uint32_t seq, uint8_t status)
{
	FS_CpLog_Record_t *r = find(seq);
	if (r)
	{
		r->status = status;
		r->run    = FS_CPLOG_RUN_NONE;
	}
}

void FS_CpLog_SetRun(uint32_t seq, uint8_t result)
{
	FS_CpLog_Record_t *r = find(seq);
	if (r && r->run == FS_CPLOG_RUN_PENDING)
		r->run = result;
}

bool FS_CpLog_Take(FS_CpLog_Record_t *rec, uint32_t *dropped)
{
	if (s_dropped)
	{
		*dropped = s_dropped;
		s_dropped = 0;
		return true;
	}
	*dropped = 0;

	if (s_head == s_tail) return false;

	FS_CpLog_Record_t *r = &s_ring[s_tail % FS_CPLOG_DEPTH];
	if (r->run == FS_CPLOG_RUN_PENDING) return false;   /* keep request order */

	*rec = *r;
	r->seq = 0;
	s_tail++;
	return true;
}

static const char *mode_name(uint8_t mode, char *buf, size_t n)
{
	/* FS_Mode_State_t order (mode.h). */
	static const char *const names[] = {
		"SLEEP", "ACTIVE", "CONFIG", "USB", "PAIRING", "START"
	};
	if (mode < sizeof(names) / sizeof(names[0])) return names[mode];
	snprintf(buf, n, "%u", (unsigned)mode);
	return buf;
}

static const char *run_name(uint8_t run, char *buf, size_t n)
{
	/* FS_Mode_RequestResult_t order (mode.h). */
	static const char *const names[] = { "ok", "invalid", "busy", "refused" };
	if (run == FS_CPLOG_RUN_NONE)    return "-";
	if (run == FS_CPLOG_RUN_PENDING) return "pending";
	if (run < sizeof(names) / sizeof(names[0])) return names[run];
	snprintf(buf, n, "%u", (unsigned)run);
	return buf;
}

void FS_CpLog_Format(char *out, size_t n, const FS_CpLog_Record_t *rec, uint32_t dropped)
{
	char mbuf[12], rbuf[12];

	if (dropped)
	{
		snprintf(out, n, "CP log: %lu older requests dropped", (unsigned long)dropped);
		return;
	}

	/* e.g. "CP#7 op 0x11 st 0x06 in SLEEP run -" — st is the CP status sent to
	 * the phone (0x01 success, 0x05 not permitted, 0x06 busy, ...); run is the
	 * mode machine's answer to the deferred request ("-" = nothing deferred). */
	snprintf(out, n, "CP#%lu op 0x%02X st 0x%02X in %s run %s",
	         (unsigned long)rec->seq, (unsigned)rec->opcode, (unsigned)rec->status,
	         mode_name(rec->mode, mbuf, sizeof(mbuf)),
	         run_name(rec->run, rbuf, sizeof(rbuf)));
}
