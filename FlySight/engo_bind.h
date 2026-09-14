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
 * serial. The firmware never writes this file. A human chooses glasses by
 * creating it through the phone app or manually over USB. With a valid file,
 * the device connects only to that serial. If the file is missing, invalid,
 * or unreadable, the device is UNBOUND and does not scan or connect to any
 * glasses (HUD off). Deleting the file returns the device to that unbound state.
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

#endif /* ENGO_BIND_H_ */
