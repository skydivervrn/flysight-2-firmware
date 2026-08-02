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
**  This program is distributed in the hope that it will be useful,       **
**  but WITHOUT ANY WARRANTY; without even the implied warranty of        **
**  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         **
**  GNU General Public License for more details.                          **
**                                                                        **
**  You should have received a copy of the GNU General Public License     **
**  along with this program.  If not, see <http://www.gnu.org/licenses/>. **
**                                                                        **
****************************************************************************
**  Contact: Bionic Avionics Inc.                                         **
**  Website: http://flysight.ca/                                          **
****************************************************************************/

#ifndef ACTIVELOOK_MODE0_H
#define ACTIVELOOK_MODE0_H

#include "activelook.h"
#include <stdbool.h>

/**
 * Called once when the mode is selected, to reset and load config data.
 */
void FS_ActiveLook_Mode0_Init(void);

/**
 * Called repeatedly to perform multi-step layout setup. Each call
 * sends exactly one BLE command. Returns FS_AL_SETUP_DONE when complete.
 */
FS_ActiveLook_SetupStatus_t FS_ActiveLook_Mode0_Setup(void);

/**
 * Called whenever we want to redraw or update the data on the display.
 * Uses the config-based line definitions from FS_ActiveLook_Mode0_Init().
 */
void FS_ActiveLook_Mode0_Update(void);

/**
 * Clear the barometric-altitude zero reference so it is re-captured from the
 * next valid baro sample. Call ONCE per ACTIVE-mode entry (device switch-on),
 * NOT on glasses reconnect — a mid-flight reconnect must not re-zero altitude.
 */
void FS_ActiveLook_Mode0_ResetBaroRef(void);

/**
 * Semantic HUD version string (e.g. "0.0.13") for logging/telemetry.
 */
const char *FS_ActiveLook_Mode0_HudVersion(void);

/**
 * Resumable-frame API: true while packets of the current HUD frame are still
 * unsent; DrainFrame sends until done or the CPU2 TX pool pushes back (then
 * returns false — call again on ACI_GATT_TX_POOL_AVAILABLE or the next tick).
 */
bool FS_ActiveLook_Mode0_HasPendingFrame(void);
bool FS_ActiveLook_Mode0_DrainFrame(void);

#endif /* ACTIVELOOK_MODE0_H */
