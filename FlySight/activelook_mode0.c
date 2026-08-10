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

#include "activelook_mode0.h"
#include "activelook_client.h"    // For FS_ActiveLook_Client_WriteWithoutResp
#include "activelook_proto.h"     // For AL_BuildFrame, AL_BatteryPct, AL_CMD_*, AL_HOLD/FLUSH
#include "config.h"               // For FS_Config_Get()
#include "engo_bind.h"            // For FS_EngoBind_CommitIfPending()
#include "flight_detect.h"        // For FS_FlightDetect_InFlight()
#include "flight_params.h"
#include "gnss.h"                 // For FS_GNSS_GetData()
#include "nav.h"                  // For calcDirection, calcDistance, calcRelBearing
#include "vbat.h"
#include "app_common.h"
#include "baro.h"                 // For FS_Baro_GetData() (needs HAL types from app_common.h)
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>                 // For atan2, fabs

/* --------------------------------------------------------------------------
   0. HUD versioning + fonts
   -------------------------------------------------------------------------- */

/* Semantic HUD version. Shown at the END of the info/status line so we can
 * confirm remotely WHICH firmware is live on the glasses. Bump on every HUD
 * change. Restarted at 0.0.1 (was the old ad-hoc "v15" build tag).
 *
 * Guarded by #ifndef so CI can stamp a nightly build with its commit, e.g.
 *   make EXTRA_CFLAGS='-DHUD_VERSION=\"0.0.15-n.abc1234\"'
 * Then the string on the glasses says exactly which build is running — the
 * whole point of this marker (Firmware_Ver in flysight.txt is unreliable). */
#ifndef HUD_VERSION
#define HUD_VERSION "0.0.14"
#endif

/* Fonts VERIFIED on ENGO 3 (this unit) via Mac BLE bench 2026-07-20 — fontList
 * (0x50) returned 8 loaded fonts (id/height px): 0/24 1/24 6/32 2/38 7/48 3/64
 * 4/75 5/82, and fonts 3 & 4 RENDER FINE (the old "3/4 = garbage" finding was a
 * clipping artefact, not a missing font). 24 px (f0/f1) is the smallest loaded;
 * no bold variants are pre-installed. */
#define HEADER_FONT 0    /* 24 px */
#define SPEED_FONT  3    /* 64 px — HSpd + VSpd side by side */
#define BIG_FONT    4    /* 75 px — GR and Alt rows */

const char *FS_ActiveLook_Mode0_HudVersion(void)
{
    return HUD_VERSION;
}

/* Takeoff/flight-detect marker, drawn at the LEFT end of the info line (with the
 * ENGO 3 X-mirror, the first character of the string is the left-most on screen).
 * ASCII glyphs so they are GUARANTEED to render (font glyph set for unicode
 * ✓/✗ is unverified on ENGO 3): 'X' = no takeoff yet, 'V' = takeoff detected.
 * Swap to a real check/cross once verified on hardware. */
#define FD_MARK_IDLE       "X"
#define FD_MARK_DETECTED   "V"

/* --------------------------------------------------------------------------
   1. Unit System Definitions
   -------------------------------------------------------------------------- */

// Enum to categorize the physical quantity a parameter represents
typedef enum {
    FS_UNIT_TYPE_SPEED,
    FS_UNIT_TYPE_DISTANCE,
    FS_UNIT_TYPE_ALTITUDE,
    FS_UNIT_TYPE_ANGLE,
    FS_UNIT_TYPE_NONE
} FS_ParamUnitType_t;

// Structure to hold conversion factor and unit suffix
typedef struct {
    double      multiplier;
    const char* suffix;
} UnitConversionInfo_t;

// --- Conversion Constants ---
#define M_PER_S_TO_KMH      3.6
#define M_PER_S_TO_MPH      2.23694
#define METERS_TO_KM        0.001
#define METERS_TO_MILES     0.000621371
#define METERS_TO_FEET      3.28084

// Minimum announced altitude (mm)
#define ALT_MIN_MM (1500L * 1000L)

/* --------------------------------------------------------------------------
   2. Data Structures for Each Line
   -------------------------------------------------------------------------- */

/**
 * A function pointer type that, given GNSS data, returns a double
 * representing the value we want displayed. e.g. heading, alt, etc.
 */
typedef double (*LineValueFn_t)(const FS_GNSS_Data_t*);

/* Simple "getter" functions for each known line type.
 * NB the HUD speed lines deliberately do NOT apply the SAS (air-density)
 * correction: user wants raw GPS telemetry matching skyderby's ground-speed
 * charts (decision 2026-07-22, from the 08-52-34 track analysis: HUD showed
 * 130 km/h SAS vs skyderby's 144 raw). Use_SAS in config still governs the
 * audio tones (audio_control.c) — only the glasses HUD is raw. */
static double LN_HSpeed(const FS_GNSS_Data_t *d) {
    return (double)d->gSpeed / 100.0; // cm/s to m/s, raw (no SAS)
}
static double LN_VSpeed(const FS_GNSS_Data_t *d) {
    return (double)d->velD / 1000.0; // mm/s to m/s, + = down, raw (no SAS)
}
static double LN_GlideRatio(const FS_GNSS_Data_t *d) {
    if (d->velD != 0) {
        return (double)(d->gSpeed / 100.0) / (double)(d->velD / 1000.0);
    }
    return 0.0; // Or some indicator for undefined glide ratio
}
static double LN_InvGlideRatio(const FS_GNSS_Data_t *d) {
    if (d->gSpeed != 0) {
        return (double)(d->velD / 1000.0) / (double)(d->gSpeed / 100.0);
    }
    return 0.0; // Or some indicator
}
static double LN_TotalSpeed(const FS_GNSS_Data_t *d) {
    double base_speed = (double)d->speed / 100.0; // cm/s to m/s
    double correction_factor = FS_FlightParams_GetSASCorrectionFactor(d->hMSL);
    return base_speed * correction_factor;
}
static double LN_DirToDest(const FS_GNSS_Data_t *d) {
    const FS_Config_Data_t *cfg = FS_Config_Get();
    // Check if destination is set and within reasonable range if needed
    if ((calcDistance(d->lat, d->lon, cfg->lat, cfg->lon) < cfg->max_dist) || (cfg->max_dist == 0)) {
         return (double)calcDirection(d->lat, d->lon, cfg->lat, cfg->lon, d->heading);
    }
    return 0.0; // Indicate no destination or out of range
}
static double LN_DistToDest(const FS_GNSS_Data_t *d) {
    const FS_Config_Data_t *cfg = FS_Config_Get();
    return (double)calcDistance(d->lat, d->lon, cfg->lat, cfg->lon); // in meters
}
static double LN_DirToBearing(const FS_GNSS_Data_t *d) {
    const FS_Config_Data_t *cfg = FS_Config_Get();
    // Assuming bearing is configured
    return (double)calcRelBearing(cfg->bearing, d->heading / 100000); // heading is 1e-5 deg
}
static double LN_DiveAngle(const FS_GNSS_Data_t *d) {
    if (d->gSpeed > 0) { // Avoid division by zero
        // velD is positive down, gSpeed is horizontal speed (always positive)
        // atan2(y, x) -> atan2(vertical_speed, horizontal_speed)
        // Need to convert cm/s to m/s or be consistent
        return atan2((double)(d->velD / 1000.0), (double)(d->gSpeed / 100.0)) * (180.0 / M_PI); // Result in degrees
    }
    return 0.0;
}
static double LN_Altitude(const FS_GNSS_Data_t *d) {
    const FS_Config_Data_t *cfg = FS_Config_Get();
    return ((double)d->hMSL - (double)cfg->dz_elev) / 1000.0; // mm to m, relative to DZ elev
}
static double LN_Heading(const FS_GNSS_Data_t *d) {
    return (double)d->heading / 100000.0; // 1e-5 deg to deg
}

/* Barometric altitude relative to the pressure captured at ACTIVE-mode start (so
 * it reads 0 where/when the device was switched ON, then height above that point
 * in metres). Independent of GPS. The reference is cleared on EVERY activation via
 * FS_ActiveLook_Mode0_ResetBaroRef() (called from FS_ActiveLook_Init, i.e. once
 * per ACTIVE entry) and re-captured on the first valid sample. It is NOT reset on
 * glasses reconnect (Mode0_Init runs per reconnect — a mid-flight reconnect must
 * not re-zero the altitude). NB "off" by button = SLEEP, RAM retained: a static
 * alone would keep last session's zero point forever. */
static int32_t s_baroRefPa100 = 0;   // reference pressure, units Pa*100 (as baro reports)

void FS_ActiveLook_Mode0_ResetBaroRef(void)
{
    s_baroRefPa100 = 0;
}
static double LN_BaroAltitude(const FS_GNSS_Data_t *d) {
    (void)d;
    const FS_Baro_Data_t *b = FS_Baro_GetData();
    if (b == NULL || b->pressure <= 0) {
        return 0.0;                    // no valid sample yet
    }
    if (s_baroRefPa100 == 0) {
        s_baroRefPa100 = b->pressure;  // capture power-on reference (zero point)
    }
    double P  = (double)b->pressure;
    double P0 = (double)s_baroRefPa100;
    // International barometric formula, relative form: h = 44330*(1-(P/P0)^(1/5.255))
    return 44330.0 * (1.0 - pow(P / P0, 0.190294957));
}

/* NOTE: the barometric vertical-speed estimator (LN_VSpeedBaro, EMA + ring
 * slope) was REMOVED 2026-07-22 after the 08-52-34 track analysis: baro
 * altitude noise in freefall is ~11 m stddev (aerodynamic pressure on the
 * body, 30x the ground noise), which made the displayed value jump in
 * 10-40 km/h steps, while GNSS velD over the same window was ~0.5 km/h
 * stable with 29+ sats. VSpd is GNSS velD again (raw, no SAS). If GPS-loss
 * robustness is ever needed, resurrect from git history as a FALLBACK only. */

/**
 * Updated table entry: includes unitType for conversion purposes.
 * 'units' field removed, as it's now dynamic based on user choice.
 */
typedef struct {
    uint8_t             typeId;   // ID from config (e.g., FS_CONFIG_MODE_HORIZONTAL_SPEED)
    const char         *label;    // Base text label (e.g., "HSpd:")
    FS_ParamUnitType_t  unitType; // Type of quantity (Speed, Distance, etc.)
    LineValueFn_t       fn;       // Function returning the value in base units (m/s, m, deg)
} AL_Mode0_LineMap_t;

/*
   A dictionary of possible line types based on FS_CONFIG_MODE_* in config.h
   and implementations in audio_control.c.
*/
static const AL_Mode0_LineMap_t s_lineMap[] =
{
    { FS_CONFIG_MODE_HORIZONTAL_SPEED,         "HSpd:", FS_UNIT_TYPE_SPEED,    LN_HSpeed       }, // 0
    { FS_CONFIG_MODE_VERTICAL_SPEED,           "VSpd:", FS_UNIT_TYPE_SPEED,    LN_VSpeed       }, // 1
    { FS_CONFIG_MODE_GLIDE_RATIO,              "GR:",   FS_UNIT_TYPE_NONE,     LN_GlideRatio   }, // 2
    { FS_CONFIG_MODE_INVERSE_GLIDE_RATIO,      "1/GR:", FS_UNIT_TYPE_NONE,     LN_InvGlideRatio}, // 3
    { FS_CONFIG_MODE_TOTAL_SPEED,              "Spd:",  FS_UNIT_TYPE_SPEED,    LN_TotalSpeed   }, // 4
    { FS_CONFIG_MODE_DIRECTION_TO_DESTINATION, "Dir:",  FS_UNIT_TYPE_ANGLE,    LN_DirToDest    }, // 5
    { FS_CONFIG_MODE_DISTANCE_TO_DESTINATION,  "Dist:", FS_UNIT_TYPE_DISTANCE, LN_DistToDest   }, // 6
    { FS_CONFIG_MODE_DIRECTION_TO_BEARING,     "Brg:",  FS_UNIT_TYPE_ANGLE,    LN_DirToBearing }, // 7
    // Mode 8, 9, 10?
    { FS_CONFIG_MODE_DIVE_ANGLE,               "Dive:", FS_UNIT_TYPE_ANGLE,    LN_DiveAngle    }, // 11
    { FS_CONFIG_MODE_ALTITUDE,                 "Alt:",  FS_UNIT_TYPE_ALTITUDE, LN_Altitude     }, // 12
    { 13,                                      "Hdg:",  FS_UNIT_TYPE_ANGLE,    LN_Heading      }, // Mode 13 (Heading)
};
static const unsigned s_lineMapCount = sizeof(s_lineMap) / sizeof(s_lineMap[0]);

/**
 * Updated line spec: stores only typeId and label.
 * Units and value calculation/conversion happen dynamically.
 */
typedef struct {
    const char    *label;      // e.g. "VSpd:"
    LineValueFn_t  fn;         // value getter (base units)
    double         multiplier; // applied to the base value before display
    int32_t        decimals;   // decimal places
    uint8_t        needsFix;   // 1 = requires 3D GPS fix; 0 = always valid (baro)
} AL_Mode0_LineSpec_t;

/* Maximum number of data lines in mode 0 (matches s_lineSpecs array bound). */
#define AL_MODE0_MAX_LINES  4

static AL_Mode0_LineSpec_t s_lineSpecs[AL_MODE0_MAX_LINES];
static uint8_t s_numLines = 0;   /* number of active lines (fixed default set) */

/* Default HUD lines, baked in so the glasses work WITHOUT any /config.txt editing.
 * Speeds are RAW GPS (no SAS) so the HUD matches skyderby's ground-speed charts;
 * Alt stays barometric (works without a fix, zeroed at power-on). */
static const AL_Mode0_LineSpec_t s_defaultLines[] = {
    { "HSpd:", LN_HSpeed,       3.6, 0, 1 },   /* horizontal speed, km/h, raw GPS */
    { "VSpd:", LN_VSpeed,       3.6, 0, 1 },   /* vertical speed, km/h, GNSS velD (+ = down),
                                                  raw — chosen over baro for stability
                                                  (track analysis 2026-07-22) */
    { "GR:",   LN_GlideRatio,   1.0, 2, 1 },
    { "Alt:",  LN_BaroAltitude, 1.0, 0, 0 },
};

/**
 * We'll also keep a "setup" step variable for multi-step layout creation.
 */
static int s_step = 0;

/*
 * Last-displayed strings for change-detection.
 * Index 0 = heading (battery/status), indices 1..4 = data lines.
 */
static char s_lastHeading[40];
static char s_lastLine[AL_MODE0_MAX_LINES][24];

/* Header (info line) cache + rebuild divider — the header string is rebuilt only
 * every 5th tick to keep sat/battery jitter from forcing a full redraw burst
 * every second (see the rate-limit comment in Mode0_Update). */
static char    battLevels[60];
static uint8_t s_hdrDivider;

/* --------------------------------------------------------------------------
   3. Unit Conversion Logic (to be factored out later)
   -------------------------------------------------------------------------- */

/**
 * Gets the conversion multiplier and unit suffix string based on parameter type
 * and the selected unit system.
 */
static UnitConversionInfo_t AL_GetUnitConversion(
        FS_ParamUnitType_t type,
        FS_Config_UnitSystem_t system) {
    UnitConversionInfo_t info = {1.0, ""}; // Default: no conversion, no suffix

    switch (type) {
        case FS_UNIT_TYPE_SPEED: // Base unit: m/s
            if (system == FS_UNIT_SYSTEM_METRIC) {
                info.multiplier = M_PER_S_TO_KMH;
                info.suffix = "km/h";
            } else { // Imperial
                info.multiplier = M_PER_S_TO_MPH;
                info.suffix = "mph";
            }
            break;
        case FS_UNIT_TYPE_DISTANCE: // Base unit: m
             if (system == FS_UNIT_SYSTEM_METRIC) {
                info.multiplier = METERS_TO_KM;
                info.suffix = "km";
            } else { // Imperial
                info.multiplier = METERS_TO_MILES;
                info.suffix = "mi";
            }
           break;
        case FS_UNIT_TYPE_ALTITUDE: // Base unit: m
            if (system == FS_UNIT_SYSTEM_METRIC) {
                // info.multiplier = 1.0; // Default is already 1.0
                info.suffix = "m";
            } else { // Imperial
                info.multiplier = METERS_TO_FEET;
                info.suffix = "ft";
            }
            break;
        case FS_UNIT_TYPE_ANGLE: // Base unit: degrees
            // No conversion needed between metric/imperial for angles
            info.suffix = "deg";
            break;
        case FS_UNIT_TYPE_NONE:  // Unitless
            // No conversion, no suffix (defaults are correct)
            break;
    }
    return info;
}

/* --------------------------------------------------------------------------
   4. Helpers for Sending Commands / Building Layout
   -------------------------------------------------------------------------- */

/* A helper to do a single WriteWithoutResp to the BLE client. */
static void AL_SendRaw(const uint8_t *data, uint16_t length)
{
    FS_ActiveLook_Client_WriteWithoutResp(data, length);
}

/* --------------------------------------------------------------------------
   Resumable frame: the HUD frame (~12 commands) is BUILT into a packet list,
   then DRAINED write-by-write. aci_gatt_write_without_resp fails with
   BUSY/INSUFFICIENT_RESOURCES when the CPU2 TX pool is momentarily full (it
   holds fewer packets than one full frame), so the drain simply stops there
   and RESUMES from the same packet on the ACI_GATT_TX_POOL_AVAILABLE event
   (tens of ms later) — the official ActiveLook SDK paces writes the same way.
   v0.0.7 aborted the WHOLE frame on the first busy write and waited for the
   next 1 s tick, which visibly slowed HUD updates.
   -------------------------------------------------------------------------- */

#define AL_FRAME_MAX_PKTS  16
static uint8_t  s_framePkt[AL_FRAME_MAX_PKTS][80];
static uint16_t s_framePktLen[AL_FRAME_MAX_PKTS];
static uint8_t  s_frameCount = 0;   /* packets in the current frame            */
static uint8_t  s_frameIdx   = 0;   /* next packet to send (== count when done)*/

static void AL_FrameReset(void)
{
    s_frameCount = 0;
    s_frameIdx   = 0;
}

/* Append a holdFlush command (AL_HOLD=0x00 or AL_FLUSH=0x01) to the frame. */
static void AL_FrameAddHoldFlush(uint8_t action)
{
    if (s_frameCount >= AL_FRAME_MAX_PKTS) return;
    size_t plen = AL_BuildFrame(s_framePkt[s_frameCount], 80,
                                AL_CMD_HOLD_FLUSH, &action, 1);
    if (plen) { s_framePktLen[s_frameCount] = (uint16_t)plen; s_frameCount++; }
}

/* Append a clear (0x01) command to the frame. */
static void AL_FrameAddClear(void)
{
    if (s_frameCount >= AL_FRAME_MAX_PKTS) return;
    size_t plen = AL_BuildFrame(s_framePkt[s_frameCount], 80,
                                AL_CMD_CLEAR, NULL, 0);
    if (plen) { s_framePktLen[s_frameCount] = (uint16_t)plen; s_frameCount++; }
}

/* Append a txt (0x37) draw to the frame. VERIFIED on ENGO 3 (v8 probe):
 * rotation must be 4; X is mirrored (large x = viewer-left); colour 15. */
static void AL_FrameAddText(int16_t x, int16_t y, uint8_t font, const char *str)
{
    if (s_frameCount >= AL_FRAME_MAX_PKTS) return;
    size_t plen = AL_BuildText(s_framePkt[s_frameCount], 80,
                               x, y, 4, font, 15, str);
    if (plen) { s_framePktLen[s_frameCount] = (uint16_t)plen; s_frameCount++; }
}

/* True while frame packets remain unsent. */
bool FS_ActiveLook_Mode0_HasPendingFrame(void)
{
    return s_frameIdx < s_frameCount;
}

/* Send remaining frame packets until done or the TX pool pushes back.
 * Returns true when the frame is fully sent. */
bool FS_ActiveLook_Mode0_DrainFrame(void)
{
    /* Drain calls are spread across task invocations, so a CB9 STOP may have
     * been processed since the last chunk — honor it between chunks. (Matters
     * at >=2 Hz refresh: STOP is how the glasses throttle us to what their
     * display processor can actually render.) */
    if (!FS_ActiveLook_Client_CanSend())
        return false;

    while (s_frameIdx < s_frameCount)
    {
        if (FS_ActiveLook_Client_WriteWithoutResp(s_framePkt[s_frameIdx],
                                                  s_framePktLen[s_frameIdx])
            != BLE_STATUS_SUCCESS)
        {
            return false;  /* resume on TX-pool-available or next tick */
        }
        s_frameIdx++;
    }
    return true;
}

/**
 * Build status layout
 */
static uint8_t AL_BuildStatus(uint8_t layoutId,
                              uint8_t *outBuf)
{
    uint8_t idx = 0;
    outBuf[idx++] = 0xFF;
    outBuf[idx++] = 0x60;  // "layoutSave"
    outBuf[idx++] = 0x00;  // 1B length
    uint8_t lenPos = idx++;

    // layout ID
    outBuf[idx++] = layoutId;

    // Additional commands size placeholder
    uint8_t addCmdSizePos = idx++;

    // X, Y, Width, Height, color, etc. Hard-coded example
    outBuf[idx++] = 0x00;
    outBuf[idx++] = 0x00;
    outBuf[idx++] = 0x00;
    outBuf[idx++] = 0x01;
    outBuf[idx++] = 0x30;
    outBuf[idx++] = 0x28;
    outBuf[idx++] = 15;
    outBuf[idx++] = 0;
    outBuf[idx++] = 1;
    outBuf[idx++] = 1;
    outBuf[idx++] = 1;
    outBuf[idx++] = 5;
    outBuf[idx++] = 35;
    outBuf[idx++] = 4;
    outBuf[idx++] = 1;

    // Build sub-commands for label & units
    uint8_t extra[64];
    uint8_t e = 0;

    // copy extras
    outBuf[addCmdSizePos] = e;
    memcpy(&outBuf[idx], extra, e);
    idx += e;

    outBuf[idx++] = 0xAA; // footer
    outBuf[lenPos] = idx; // fill length

    return idx;
}

/**
 * Build flight parameter layout
 */
static uint8_t AL_BuildLayout(uint8_t layoutId,
                              const char *headingText,
                              const char *unitsText,
                              uint8_t *outBuf)
{
    uint8_t idx = 0;
    outBuf[idx++] = 0xFF;
    outBuf[idx++] = 0x60;  // "layoutSave"
    outBuf[idx++] = 0x00;  // 1B length
    uint8_t lenPos = idx++;

    // layout ID
    outBuf[idx++] = layoutId;

    // Additional commands size placeholder
    uint8_t addCmdSizePos = idx++;

    // X, Y, Width, Height, color, etc. Hard-coded example
    outBuf[idx++] = 0x00;
    outBuf[idx++] = 0x00;
    outBuf[idx++] = 0x00;
    outBuf[idx++] = 0x01;
    outBuf[idx++] = 0x30;
    outBuf[idx++] = 0x28;
    outBuf[idx++] = 15;   // foreColor
    outBuf[idx++] = 0;    // backColor
    outBuf[idx++] = 1;    // font  (match the WORKING status layout)
    outBuf[idx++] = 1;    // textValid
    outBuf[idx++] = 1;    // textX hi
    outBuf[idx++] = 5;    // textX lo  -> 261 (right-anchored for ENGO 3 X-mirror)
    outBuf[idx++] = 35;   // textY  (inside clipH=40; was 40 = ON the clip edge -> invisible)
    outBuf[idx++] = 4;    // rotation = 4 (ENGO 3)
    outBuf[idx++] = 1;    // opacity

    /* No additional sub-commands. The layout "text" sub-command (0x09) encoding is
     * unreliable on ENGO 3 — a malformed sub-command makes the glasses reject the
     * whole layoutSave, so layouts 10-13 never existed and their page slots stayed
     * blank (only the sub-command-free status layout rendered). We now render the
     * data lines exactly like the status layout and prepend the label to the
     * dynamic value string in the page update instead. */
    (void)headingText; (void)unitsText;
    uint8_t e = 0;
    outBuf[addCmdSizePos] = e;

    outBuf[idx++] = 0xAA; // footer
    outBuf[lenPos] = idx; // fill length

    return idx;
}

/**
 * Build a page referencing layout #10..#14
 */
static uint8_t AL_BuildPage(uint8_t pageId, uint8_t *outBuf)
{
    // Read config
    const FS_Config_Data_t *cfg = FS_Config_Get();

    uint8_t idx = 0;
    outBuf[idx++] = 0xFF;
    outBuf[idx++] = 0x80;
    outBuf[idx++] = 0x00;
    uint8_t lenPos = idx++;

    outBuf[idx++] = pageId;

    outBuf[idx++] = 14;
    outBuf[idx++] = 0x00;
    outBuf[idx++] = 0x00;
    outBuf[idx++] = 138 + 40;

    for (int i = 0; i < cfg->num_al_lines; i++)
    {
        outBuf[idx++] = 10 + i;
        outBuf[idx++] = 0x00;
        outBuf[idx++] = 0x00;
        outBuf[idx++] = 133 - 40 * i;
    }

    outBuf[idx++] = 0xAA;
    outBuf[lenPos] = idx;
    return idx;
}

/**
 * Build a single pageUpdate command with 4 lines separated by '\0'.
 */
static uint8_t AL_BuildPageUpdate(uint8_t pageId,
                                  const char *heading,
                                  const char *line1,
                                  const char *line2,
                                  const char *line3,
                                  const char *line4,
                                  uint8_t *outBuf)
{
    uint8_t idx = 0;
    outBuf[idx++] = 0xFF;
    outBuf[idx++] = 0x86; // "pageClearAndDisplay"
    outBuf[idx++] = 0x00;
    uint8_t lenPos = idx++;

    outBuf[idx++] = pageId;

    // heading
    size_t lh = strlen(heading);
    memcpy(&outBuf[idx], heading, lh);
    idx += lh;
    outBuf[idx++] = 0;

    // line1
    size_t l1 = strlen(line1);
    memcpy(&outBuf[idx], line1, l1);
    idx += l1;
    outBuf[idx++] = 0;

    // line2
    size_t l2 = strlen(line2);
    memcpy(&outBuf[idx], line2, l2);
    idx += l2;
    outBuf[idx++] = 0;

    // line3
    size_t l3 = strlen(line3);
    memcpy(&outBuf[idx], line3, l3);
    idx += l3;
    outBuf[idx++] = 0;

    // line4
    size_t l4 = strlen(line4);
    memcpy(&outBuf[idx], line4, l4);
    idx += l4;
    outBuf[idx++] = 0;

    outBuf[idx++] = 0xAA;
    outBuf[lenPos] = idx;
    return idx;
}

/* --------------------------------------------------------------------------
   3. Mode0 Implementation
   -------------------------------------------------------------------------- */

// Helper to find the LineMap entry by typeId
static const AL_Mode0_LineMap_t* FindLineMapEntry(uint8_t typeId) {
    for (unsigned k = 0; k < s_lineMapCount; k++) {
        if (s_lineMap[k].typeId == typeId) {
            return &s_lineMap[k];
        }
    }
    return NULL;
}


/**
 * FS_ActiveLook_Mode0_Init()
 *
 *  - Reset our s_step to 0
 *  - Read the config => al_line_1..4
 *  - For each of the 4 lines, find a matching entry in s_lineMap
 *    (by typeId) and store it in s_lineSpecs[i].
 */
void FS_ActiveLook_Mode0_Init(void)
{
    s_step = 0;

    /* Use the baked-in default line set (VSpd, GR, Hdg, Alt) so the HUD works
     * out of the box without any /config.txt editing. */
    s_numLines = (uint8_t)(sizeof(s_defaultLines) / sizeof(s_defaultLines[0]));
    if (s_numLines > AL_MODE0_MAX_LINES)
        s_numLines = AL_MODE0_MAX_LINES;

    for (int i = 0; i < AL_MODE0_MAX_LINES; i++)
    {
        if (i < s_numLines) {
            s_lineSpecs[i] = s_defaultLines[i];
        } else {
            s_lineSpecs[i].label      = "";
            s_lineSpecs[i].fn         = NULL;
            s_lineSpecs[i].multiplier = 1.0;
            s_lineSpecs[i].decimals   = 0;
            s_lineSpecs[i].needsFix   = 0;
        }
    }

    /* Reset change-detection buffers so first update always sends */
    s_lastHeading[0] = '\0';
    for (int i = 0; i < AL_MODE0_MAX_LINES; i++)
        s_lastLine[i][0] = '\0';
    battLevels[0] = '\0';   /* header cache: rebuild immediately on next tick */
    s_hdrDivider  = 0;
    AL_FrameReset();        /* discard any frame left over from a dropped link */
}

/**
 * FS_ActiveLook_Mode0_Setup()
 *
 * Multi-step routine. In each step (0-3), builds a layout (#10-13).
 * It now dynamically determines the unit suffix based on the line's parameter type
 * and the chosen unit system.
 * Step 4 builds the page definition.
 */
FS_ActiveLook_SetupStatus_t FS_ActiveLook_Mode0_Setup(void)
{
    /* No-op: the old layout/page setup is replaced by direct clear+txt drawing
     * in FS_ActiveLook_Mode0_Update (the FSM goes discovery -> CLEAR -> UPDATE and
     * never enters AL_STATE_SETUP). Kept so the mode-ops table still resolves. */
    return FS_AL_SETUP_DONE;
}

/**
 * FS_ActiveLook_Mode0_Update()
 *
 * Called periodically to update the display.
 * - Gets current GNSS data.
 * - For each line:
 * - Finds the corresponding map entry using stored typeId.
 * - Calls the base value function (fn).
 * - Gets the correct unit conversion info (multiplier).
 * - Calculates the display value by applying the multiplier.
 * - Formats the display value into a string.
 * - Builds and sends the "pageClearAndDisplay" command with the 4 formatted value strings.
 */
void FS_ActiveLook_Mode0_Update(void)
{
    /* Link is up by the time we render: if we connected to a new (unbound) pair
     * of glasses, persist their serial to /engo3.txt now (once, task context). */
    FS_EngoBind_CommitIfPending();
    /* One-shot per-session proof that we actually connected (and to which serial). */
    FS_EngoBind_LogLinkedOnce();

    const FS_GNSS_Data_t *gnss = FS_GNSS_GetData();
    const FS_VBAT_Data_t *vbat = FS_VBAT_GetData();
    uint8_t alBatt = FS_ActiveLook_Client_GetBatteryLevel();

    /* Use the baked-in default line set (independent of /config.txt). */
    int numLines = s_numLines;
    if (numLines > AL_MODE0_MAX_LINES)
        numLines = AL_MODE0_MAX_LINES;

    char lineValueStr[AL_MODE0_MAX_LINES][24]; // value strings (label drawn separately)

    /* Zero-init all line strings — unused lines stay empty, not garbage */
    for (int i = 0; i < AL_MODE0_MAX_LINES; i++)
        lineValueStr[i][0] = '\0';

    // For each line, compute the value via its getter + multiplier
    for (int i = 0; i < numLines; i++)
    {
        bool display_invalid = false;

        if (s_lineSpecs[i].fn == NULL) {
            display_invalid = true;                       // unused/empty line
        } else if (s_lineSpecs[i].needsFix && gnss->gpsFix != 3) {
            display_invalid = true;                       // GPS-derived line, no 3D fix yet
        }

        if (!display_invalid) {
            double v = s_lineSpecs[i].fn(gnss) * s_lineSpecs[i].multiplier;
            snprintf(lineValueStr[i], sizeof(lineValueStr[i]), "%.*f",
                     (int)(s_lineSpecs[i].decimals), v);
        } else {
            snprintf(lineValueStr[i], sizeof(lineValueStr[i]), "----");
        }
    }

    // Build header text using AL_BatteryPct for consistent clamped math

    char alBattStr[5];
    if (alBatt == 255)
        snprintf(alBattStr, sizeof(alBattStr), "??");
    else
        snprintf(alBattStr, sizeof(alBattStr), "%d", alBatt);

    uint8_t fs_pct = AL_BatteryPct(vbat->voltage);

    /* Info/status line: takeoff marker (left-most), glasses batt, FlySight batt,
     * sat count, then the HUD version at the END (per request). The trailing
     * version also confirms REMOTELY which firmware is live on the glasses (if it
     * doesn't change after a flash, the glasses are frozen / the flash failed).
     *
     * Rate-limited: the string is REBUILT only every 5th tick. Sat count / battery
     * jitter otherwise changes the header nearly every tick, forcing a full
     * clear+redraw burst each second even when the data lines are static —
     * needless pressure on the glasses' command buffer (XCTrack: "not much
     * bandwidth available"). Takeoff-marker latency of <=5 ticks is fine. */
    if (s_hdrDivider == 0 || battLevels[0] == '\0') {
        const char *fdMark = FS_FlightDetect_InFlight() ? FD_MARK_DETECTED : FD_MARK_IDLE;
        snprintf(battLevels, sizeof(battLevels),
                 "%s A:%s%% F:%d%% N:%d v%s", fdMark, alBattStr, (int)fs_pct, gnss->numSV, HUD_VERSION);
    }
    s_hdrDivider = (uint8_t)((s_hdrDivider + 1) % 5);

    /* --- Change detection: skip the BLE write if nothing changed --- */
    bool changed = (strcmp(battLevels, s_lastHeading) != 0);
    if (!changed) {
        for (int i = 0; i < numLines; i++) {
            if (strcmp(lineValueStr[i], s_lastLine[i]) != 0) {
                changed = true;
                break;
            }
        }
    }

    if (!changed) {
        /* Nothing new to send — skip this tick entirely */
        return;
    }

    /* Update cached strings */
    strncpy(s_lastHeading, battLevels, sizeof(s_lastHeading) - 1);
    s_lastHeading[sizeof(s_lastHeading) - 1] = '\0';
    for (int i = 0; i < numLines; i++) {
        strncpy(s_lastLine[i], lineValueStr[i], sizeof(s_lastLine[i]) - 1);
        s_lastLine[i][sizeof(s_lastLine[i]) - 1] = '\0';
    }

    /* Layout tuned live on HW via the Mac BLE bench (Tools/engo_mac_hud_mock.py,
     * session 2026-07-20) and approved by the user pixel by pixel:
     *  - X is mirrored: x=296 anchors at the viewer's LEFT edge, text grows rightward
     *  - Y not flipped: larger Y = higher; glyphs draw DOWNWARD from the anchor
     *  - NO labels — values only (user request), positions must match s_defaultLines
     *    order: HSpd, VSpd (one row, side by side), GR, Alt. */
    static const struct { int16_t x, y; uint8_t font; } rowPos[AL_MODE0_MAX_LINES] = {
        { 268, 208, SPEED_FONT },   /* HSpd, viewer-left  */
        { 134, 208, SPEED_FONT },   /* VSpd, viewer-right */
        { 250, 147, BIG_FONT   },   /* GR                 */
        { 250,  73, BIG_FONT   },   /* Alt                */
    };

    /* Build the whole frame as a packet list (any still-pending older frame is
     * discarded — fresher data beats finishing a stale frame), then start
     * draining it. If the TX pool pushes back mid-frame, the remainder goes out
     * on the ACI_GATT_TX_POOL_AVAILABLE event via FS_ActiveLook_TxPoolAvailable()
     * — no full-second stall, no half-drawn frame left behind (the frame always
     * completes, ending with its FLUSH, unless the link drops entirely — and
     * that case is covered by the client watchdogs + reconnect). */
    AL_FrameReset();
    AL_FrameAddHoldFlush(AL_HOLD);
    AL_FrameAddClear();
    AL_FrameAddText(296, 232, HEADER_FONT, battLevels);            /* info line */
    for (int i = 0; i < numLines; i++) {
        AL_FrameAddText(rowPos[i].x, rowPos[i].y, rowPos[i].font, lineValueStr[i]);
    }
    AL_FrameAddHoldFlush(AL_FLUSH);
    FS_ActiveLook_Mode0_DrainFrame();
}
