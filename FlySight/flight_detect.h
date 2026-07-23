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
 * freefall) is detected. The HUD shows a marker and the event is logged once.
 * Power saving / mode changes are a SEPARATE, later phase.
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

#endif /* FLIGHT_DETECT_H_ */
