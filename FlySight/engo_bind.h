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

/*
 * ENGO/ActiveLook glasses binding — pin to ONE specific pair.
 *
 * Identifier = the 6-character Customer Serial Number, i.e. the trailing 6
 * characters of the advertised name (VERIFIED on HW: "ENGO 3 123456" in the
 * advertisement -> serial "123456"). This is the per-unit, stable, pre-connect
 * unique key per the ActiveLook API; the BLE address is static-random and NOT a
 * reliable key, so we deliberately do NOT pin by address.
 *
 * Persistence = a dedicated file "/engo3.txt" at the FAT root holding the
 * serial. Behaviour:
 *   - file present + valid  -> connect ONLY to that serial.
 *   - file absent / invalid -> connect to the first ActiveLook device found,
 *                              then create the file with its serial (auto-bind).
 *   - delete the file       -> back to first-boot (connect-to-first) behaviour.
 */

#ifndef ENGO_BIND_H_
#define ENGO_BIND_H_

#include <stdbool.h>

#define FS_ENGO_SERIAL_LEN  6

/* Read /engo3.txt. Call once at active-mode start, after FATFS is mounted and
 * before BLE discovery begins. Does NOT log (runs before logging is up). */
void FS_EngoBind_Load(void);

/* Log the boot-time bind status ("pinned to serial X" / "unbound"). Call AFTER
 * FS_Log_Init() so the line is actually recorded. */
void FS_EngoBind_LogStatus(void);

/* Log "linked to serial X" ONCE per session, on the first confirmed link.
 * Call from the HUD update tick (which only runs once connected). */
void FS_EngoBind_LogLinkedOnce(void);

/* True if a valid serial was loaded from /engo3.txt (i.e. we are pinned). */
bool FS_EngoBind_IsBound(void);

/* Bound serial as a NUL-terminated 6-char string (valid only when IsBound()). */
const char *FS_EngoBind_Serial(void);

/* True if cand (>= 6 chars) trailing-serial equals the bound serial.
 * cand must point to exactly FS_ENGO_SERIAL_LEN characters. */
bool FS_EngoBind_SerialMatches(const char *cand6);

/* While UNBOUND: remember the serial of the device we are about to connect to,
 * so it can be persisted once the link is confirmed. No-op when already bound. */
void FS_EngoBind_NotePending(const char *cand6);

/* While UNBOUND with a pending serial: create /engo3.txt and become bound.
 * Call from a normal task context AFTER the link is up (first HUD tick).
 * Idempotent: no-op when already bound or nothing pending. */
void FS_EngoBind_CommitIfPending(void);

#endif /* ENGO_BIND_H_ */
