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

/*
 * WHY THIS EXISTS
 * ---------------
 * Since v0.0.19 an ordinary reconnect from the tablet and from the Mac is
 * refused; only a double-press (PAIRING mode) gets in. Ordinary advertising is
 * aci_gap_set_undirected_connectable(..., HCI_ADV_FILTER_WHITELIST_SCAN_CONNECT)
 * — whitelist-only — while PAIRING advertises with aci_gap_set_limited_discoverable,
 * open to everyone. So either the bond database is EMPTY when advertising starts,
 * or it holds identities the controller cannot match to the phone's resolvable
 * private address. This module does not fix either; it makes the device say
 * which one it is.
 *
 * The existing event log (log.c, $EVNT rows) is the wrong sink: it is a
 * per-session file that only exists in ACTIVE mode, and this bug happens in
 * SLEEP. So records go to a ring buffer in RAM and are appended to a plain file
 * on the card, /BLEDIAG.TXT, from a context where FatFs is legitimately free.
 *
 * CONTEXT RULES (verified in the sources, not assumed):
 *  - FatFs is NOT mounted in SLEEP. resource_manager.c hands it out as a binary
 *    lock, not a refcount: FatFS_Init returns FAILURE if the count is already 1,
 *    and FatFS_DeInit calls Error_Handler if it is not exactly 1. So the flush
 *    must only release what it successfully acquired — and "acquire or skip"
 *    gives the right gating for free: in ACTIVE the log owns FatFs, and in USB
 *    the host owns the card through FS_RESOURCE_MICROSD, and in both cases the
 *    acquire fails and the flush does nothing rather than fighting for the card.
 *  - Emitting is therefore split from writing. FS_BleDiag_Log() only formats
 *    into RAM and is safe to call from a BLE event callback; FS_BleDiag_Flush()
 *    touches the card and is called only from the main loop (MX_APPE_Process,
 *    after UTIL_SEQ_Run has returned, i.e. with no sequencer task running) and
 *    from the mode task just before USB mass storage takes the card.
 *  - Nothing here is ever called from an interrupt.
 *
 * Set FS_BLE_DIAG to 0 to compile the whole thing out; the call sites collapse
 * to nothing and the two .c files build to empty objects.
 */

#ifndef BLE_DIAG_H_
#define BLE_DIAG_H_

#ifndef FS_BLE_DIAG
#define FS_BLE_DIAG 0
#endif

#include <stdarg.h>
#include <stdint.h>

#if FS_BLE_DIAG

/* One event per line, plain ASCII, CRLF-terminated on the card. Anything longer
 * is TRUNCATED, never wrapped: two lines for one event would be read as two
 * events. 128 is not a round guess — the longest records are the CFG dump (~82
 * chars) and the STATE line with a 30-character device name (~106), on top of an
 * 18-character "<seq> <ms> " prefix once HAL_GetTick passes ten digits. 96 was
 * the first try and it clipped both; the host test pins the real lines. */
#define FS_BLE_DIAG_LINE_LEN 128

/* 48 lines = 6144 bytes of RAM1. The ring only has to bridge the gap between an
 * event and the next pass through the main loop, which is milliseconds, so this
 * is generous; it is bounded so a stuck card cannot grow it. */
#define FS_BLE_DIAG_LINES    48

/* ---- pure ring buffer + line formatter (ble_diag.c, host-tested) ---------- */

/* Discard everything and restart the sequence at 0. Tests only. */
void FS_BleDiag_Reset(void);

/* Append one record. The line is built as "<seq> <ms> <text>"; every byte
 * outside printable ASCII is replaced by '.' so a stray control character
 * cannot break the one-event-per-line rule. The sequence number counts every
 * ATTEMPTED record, so a gap in the numbers in the file is exactly the set of
 * records that were dropped. When the ring is full the NEWEST record is dropped,
 * never an older one: history already recorded is worth more than the record
 * that overflowed it. */
void FS_BleDiag_Emit(uint32_t ms, const char *fmt, ...);
void FS_BleDiag_EmitV(uint32_t ms, const char *fmt, va_list ap);

/* Oldest buffered line, NUL-terminated, or NULL when the ring is empty. Valid
 * until the next Drop/Emit. */
const char *FS_BleDiag_Peek(void);

/* Discard the oldest line — call only after it has been written out. */
void FS_BleDiag_Drop(void);

/* Number of lines waiting to be written. */
uint32_t FS_BleDiag_Pending(void);

/* Number of records dropped since the last call, and zero the counter. */
uint32_t FS_BleDiag_TakeDropped(void);

/* "XX:XX:XX:XX:XX:XX" in the usual display order (addr[5] first, the order the
 * BT spec prints and every scanner shows). Returns the number of characters
 * written, 0 if the buffer is too small. */
uint32_t FS_BleDiag_FormatAddr(char *out, uint32_t outLen, const uint8_t *addr);

/* ---- platform glue (ble_diag_sink.c) ------------------------------------- */

/* Timestamp with HAL_GetTick() and emit. Task/main context only.
 *
 * CAVEAT, and it matters when reading the file: HAL_GetTick counts SysTick,
 * which stops while the device is in STOP mode — and in SLEEP the sequencer
 * idles into STOP (app_entry.c UTIL_SEQ_Idle). The timestamp is therefore
 * monotonic and correct for ordering and for measuring how long a BLE exchange
 * took, but it is NOT wall-clock: a long idle between two records shows up as a
 * small gap. Use the ordering, not the deltas across an idle. */
void FS_BleDiag_Log(const char *fmt, ...);

/* Shortest interval between two flushes, in AWAKE milliseconds (HAL_GetTick
 * stops in STOP, so an idle period does not count towards it — which is what we
 * want: the first record after a quiet spell goes out at once). */
#define FS_BLE_DIAG_FLUSH_MIN_MS 2000

/* Append everything buffered to /BLEDIAG.TXT, at most once every
 * FS_BLE_DIAG_FLUSH_MIN_MS unless the ring is over half full. No-op when the
 * ring is empty or when FatFs is held by somebody else.
 *
 * The rate limit is not tidiness. Every acquire mounts the card, and
 * FatFS_Init calls Error_Handler() if f_mount fails — which disables interrupts
 * and spins, so the watchdog reboots the device. The firmware already takes that
 * risk at boot, on every ACTIVE entry and on every USB unplug; a diagnostic that
 * mounted on every pass of the main loop would take it hundreds of times more
 * often, and would keep the microSD spinning up in SLEEP for no good reason.
 * Two seconds is far shorter than anything that happens on the BLE path and far
 * longer than a burst of records from one event.
 *
 * Main-loop / task context only. */
void FS_BleDiag_Flush(void);

/* The same, ignoring the rate limit. For the one place that has no later
 * chance: the mode task on VBUS_HIGH, an instant before USB mass storage takes
 * the card away for as long as the cable is in. */
void FS_BleDiag_FlushNow(void);

#else /* !FS_BLE_DIAG */

#define FS_BleDiag_Log(...)   ((void)0)
#define FS_BleDiag_Flush()    ((void)0)
#define FS_BleDiag_FlushNow() ((void)0)

#endif /* FS_BLE_DIAG */

#endif /* BLE_DIAG_H_ */
