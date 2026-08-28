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
 * Flight (takeoff) detection — DISPLAY/LOG ONLY (phase 1).
 *
 * This module does NOT change any device behaviour: it only watches the GNSS
 * stream and latches an "in flight" flag once a takeoff (or, as a fallback,
 * freefall) is detected. The HUD shows a marker and, since v0.0.38, a clock
 * counting from that instant; the event is logged once. Power saving / mode
 * changes are a SEPARATE, later phase.
 *
 * Pure + host-testable: no HAL / RTOS dependencies, only FS_GNSS_Data_t in.
 */

#ifndef FLIGHT_DETECT_H_
#define FLIGHT_DETECT_H_

#include <stdbool.h>
#include <stdint.h>
#include "gnss.h"

/* Reset detector state (call once when active mode starts). */
void FS_FlightDetect_Init(void);

/*
 * Feed one GNSS sample (call on every GNSS DataReady).
 * Returns true EXACTLY ONCE — on the sample where flight is first detected
 * (rising edge) — so the caller can emit a single log entry. Returns false on
 * every other call (including once already latched).
 */
bool FS_FlightDetect_Update(const FS_GNSS_Data_t *d);

/* Latched flag: false until the first detection, then true until Init(). */
bool FS_FlightDetect_InFlight(void);

/*
 * Seconds since takeoff, for the HUD's elapsed-time field.
 *
 * Returns false and LEAVES *out UNTOUCHED until flight has been detected, so
 * the caller has to say "not started" rather than draw a zero — a clock reading
 * 0:00 on the ground is indistinguishable from one that is running, and the one
 * thing a stopwatch must never do is look like it started when it did not.
 *
 * THE EPOCH IS THE DETECTING SAMPLE, not the first climbing one. It therefore
 * reads about 8 s short of when the detector first saw a climb (FS_FD_CLIMB_
 * HOLD_MS) and rather more short of wheels-up, since an aircraft is rolling
 * before it climbs at 2.5 m/s. That is the honest anchor for a value derived
 * from a latch: the only instant this module actually knows is the one where
 * it made up its mind. Anything earlier would be a reconstruction, and it would
 * differ between the takeoff path and the freefall fallback, where "8 s ago"
 * means nothing at all.
 *
 * Time comes from the GNSS iTOW carried by the samples fed to Update(), never
 * from HAL_GetTick(): the module stays pure and host-testable, and the reading
 * cannot be pulled off by the app tick being stopped or reset.
 */
bool FS_FlightDetect_ElapsedSec(uint32_t *out);

#endif /* FLIGHT_DETECT_H_ */
