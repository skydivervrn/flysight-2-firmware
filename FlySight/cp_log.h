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

/*
 * WHY THIS EXISTS
 * ---------------
 * The phone sometimes gets an error on "Switch to ACTIVE" and succeeds only
 * after several retries. The firmware side of that exchange is the Device
 * State control point (device_state.c), and nothing about it was recorded.
 *
 * The event log is the wrong place to write to directly: EVENT.CSV is a
 * per-session file that is open only in ACTIVE (and START), while a "Switch to
 * ACTIVE" request by definition arrives in SLEEP, when every event line is
 * silently dropped. So each request is recorded here, in RAM (which survives
 * SLEEP), and written out as one EVENT.CSV line per request once a log is open
 * — typically into the very session the successful retry opened, preceded by
 * the attempts that failed.
 *
 * One record per request: a sequence number since boot (so dropped records
 * and the order of attempts are visible), the opcode, the status sent back to
 * the phone, the device mode when the request arrived, and — for requests the
 * firmware defers until the response has gone out (mode changes, reboot,
 * firmware install) — what the mode machine said when the deferred request
 * was finally handed to it. That last field is the one a "SUCCESS, but
 * nothing happened" failure would show up in.
 *
 * This module is pure: no HAL, no log, no locking. The caller (device_state.c)
 * serialises access — FS_CpLog_SetRun can be reached from the response
 * fallback timer, which runs in interrupt context.
 */

#ifndef CP_LOG_H_
#define CP_LOG_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define FS_CPLOG_DEPTH        8u     /* records kept while no log is open     */

#define FS_CPLOG_RUN_NONE     0xFFu  /* nothing was deferred                  */
#define FS_CPLOG_RUN_PENDING  0xFEu  /* deferred, not handed over yet         */

typedef struct
{
	uint32_t seq;      /* 1, 2, ... since boot; 0 = empty slot */
	uint8_t  opcode;
	uint8_t  status;   /* CP status sent (or due) to the phone */
	uint8_t  mode;     /* FS_Mode_State_t when the request arrived */
	uint8_t  run;      /* FS_Mode_RequestResult_t of the deferred request,
	                      or FS_CPLOG_RUN_NONE / FS_CPLOG_RUN_PENDING */
} FS_CpLog_Record_t;

void FS_CpLog_Reset(void);

/* Record a request; returns its sequence number. A full ring drops its oldest
 * record and counts it (reported once, before the next line taken). */
uint32_t FS_CpLog_Add(uint8_t opcode, uint8_t status, uint8_t mode, bool deferred);

/* The deferred part was not armed after all (pending slot busy): correct the
 * status and mark it as having nothing deferred. No-op if seq is gone. */
void FS_CpLog_Amend(uint32_t seq, uint8_t status);

/* The deferred request was handed to the mode machine with this result.
 * No-op if the record has already been dropped from the ring. */
void FS_CpLog_SetRun(uint32_t seq, uint8_t result);

/* Take the next line to write, if any. A record still waiting for its deferred
 * result holds back the records after it, so lines stay in request order.
 * Returns false when there is nothing ready. */
bool FS_CpLog_Take(FS_CpLog_Record_t *rec, uint32_t *dropped);

/* Format a taken record (or, when dropped != 0, the drop notice) into out.
 * Lines fit EVENT.CSV's 80-byte message comfortably. */
void FS_CpLog_Format(char *out, size_t n, const FS_CpLog_Record_t *rec, uint32_t dropped);

#endif /* CP_LOG_H_ */
