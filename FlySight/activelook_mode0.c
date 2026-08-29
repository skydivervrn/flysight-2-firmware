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
#include "hud_layout.h"           // For the element list and the global offset
#include "engo_bind.h"            // For FS_EngoBind_CommitIfPending()
#include "comp_corridor.h"        // For the lane indicator (field 106, HUD_Mode 1)
#include "flight_detect.h"        // For FS_FlightDetect_InFlight()
#include "gnss.h"                 // For FS_GNSS_GetData()
#include "nav.h"                  // For calcDirection, calcDistance, calcRelBearing
#include "nav_arrow.h"            // For FS_NavArrow_Build() (field 15)
#include "vbat.h"
#include "app_common.h"
#include "baro.h"                 // For FS_Baro_GetData() (needs HAL types from app_common.h)
#include "log.h"                  // For FS_Log_WriteEventAsync() (same HAL dependency)
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
#define HUD_VERSION "0.0.40-diag"
#endif

/* Fonts VERIFIED on ENGO 3 (this unit) via Mac BLE bench 2026-07-20 — fontList
 * (0x50) returned 8 loaded fonts (id/height px): 0/24 1/24 6/32 2/38 7/48 3/64
 * 4/75 5/82, and fonts 3 & 4 RENDER FINE (the old "3/4 = garbage" finding was a
 * clipping artefact, not a missing font). 24 px (f0/f1) is the smallest loaded;
 * no bold variants are pre-installed.
 *
 * Which font each element uses is now part of the layout (hud_layout.h), not
 * baked in here. */

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

/* Field 106 before there is a takeoff to count from. It keeps a clock's SHAPE
 * rather than borrowing the "----" the data rows use for "no fix yet": at arm's
 * length the colon is what says this readout is a time, so the wearer can see
 * the clock is present and simply has not started. What it must never show is
 * 0:00 — a stopped clock and a clock one second into a flight would then look
 * the same, and the difference matters to whoever is being timed. */
#define FLT_TIME_IDLE      "--:--"

/* --------------------------------------------------------------------------
   1. Unit System Definitions
   -------------------------------------------------------------------------- */

/* The quantity each field measures (FS_HUD_QTY_*), the unit table that turns it
 * into a multiplier and a suffix (FS_HudLayout_UnitConv) and the unit ids the
 * config may ask for (FS_HUD_UNITS_*) all live in hud_layout.{c,h} now. They
 * moved out of this file in v0.0.18: nothing in this translation unit is
 * host-testable (it pulls in the HAL, the GNSS driver and FatFs), and the unit
 * table is exactly the part that has to be checked value by value — see
 * Tests/test_activelook.c. */

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
 * correction: user wants raw GPS telemetry matching the ground-speed charts
 * used for track analysis (decision 2026-07-22, from the 08-52-34 track
 * analysis: HUD showed 130 km/h SAS against 144 raw). Use_SAS in config still governs the
 * audio tones (audio_control.c) — only the glasses HUD is raw.
 *
 * Total speed was the one line that still applied it (owner, 2026-08-25:
 * "SAS убрать везде"). Three numbers side by side on one screen, two of them
 * raw and one corrected, differ by around 10 % at altitude and read as an
 * instrument fault. With that call gone nothing in the glasses path uses the
 * correction at all, which is why flight_params.c went with it. */
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
    return (double)d->speed / 100.0; // cm/s to m/s, raw (no SAS)
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

/* --- The navigation arrow (field 15) -------------------------------------
 *
 * Its own direction, deliberately NOT LN_DirToDest: that one answers 0.0
 * beyond Max_Dist, and an arrow cannot say "I don't know" by pointing
 * straight ahead. See FS_HUD_FIELD_NAV_ARROW in hud_layout.h. */
static int NavArrowDeg(const FS_GNSS_Data_t *d) {
    const FS_Config_Data_t *cfg = FS_Config_Get();
    return calcDirection(d->lat, d->lon, cfg->lat, cfg->lon, d->heading);
}

/* Exactly 0,0 is how the card says "no destination". FS_Config_Init leaves
 * Lat/Lon at zero, and a value the parser rejects leaves them there too — so
 * zero means "nobody has set one", not "steer for the Gulf of Guinea". */
static bool NavDestSet(void) {
    const FS_Config_Data_t *cfg = FS_Config_Get();
    return !(cfg->lat == 0 && cfg->lon == 0);
}

/* Quantise the arrow before comparing it against the last frame. Heading
 * wobbles by a degree constantly; without this the HUD would push a whole
 * frame to the glasses on nothing but noise, every tick, forever. Three
 * degrees is under two pixels at the tip of the largest box. */
#define NAV_ARROW_STEP_DEG  3

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
    FS_HudQuantity_t    qty;      // Quantity measured (speed, distance, ...)
    LineValueFn_t       fn;       // Function returning the value in base units (m/s, m, deg)
} AL_Mode0_LineMap_t;

/*
   A dictionary of possible line types based on FS_CONFIG_MODE_* in config.h
   and implementations in audio_control.c.
*/
static const AL_Mode0_LineMap_t s_lineMap[] =
{
    { FS_CONFIG_MODE_HORIZONTAL_SPEED,         "HSpd:", FS_HUD_QTY_SPEED,    LN_HSpeed       }, // 0
    { FS_CONFIG_MODE_VERTICAL_SPEED,           "VSpd:", FS_HUD_QTY_SPEED,    LN_VSpeed       }, // 1
    { FS_CONFIG_MODE_GLIDE_RATIO,              "GR:",   FS_HUD_QTY_NONE,     LN_GlideRatio   }, // 2
    { FS_CONFIG_MODE_INVERSE_GLIDE_RATIO,      "1/GR:", FS_HUD_QTY_NONE,     LN_InvGlideRatio}, // 3
    { FS_CONFIG_MODE_TOTAL_SPEED,              "Spd:",  FS_HUD_QTY_SPEED,    LN_TotalSpeed   }, // 4
    { FS_CONFIG_MODE_DIRECTION_TO_DESTINATION, "Dir:",  FS_HUD_QTY_ANGLE,    LN_DirToDest    }, // 5
    { FS_CONFIG_MODE_DISTANCE_TO_DESTINATION,  "Dist:", FS_HUD_QTY_DISTANCE, LN_DistToDest   }, // 6
    { FS_CONFIG_MODE_DIRECTION_TO_BEARING,     "Brg:",  FS_HUD_QTY_ANGLE,    LN_DirToBearing }, // 7
    // Mode 8, 9, 10?
    { FS_CONFIG_MODE_DIVE_ANGLE,               "Dive:", FS_HUD_QTY_ANGLE,    LN_DiveAngle    }, // 11
    { FS_CONFIG_MODE_ALTITUDE,                 "Alt:",  FS_HUD_QTY_ALTITUDE, LN_Altitude     }, // 12
    { FS_HUD_FIELD_HEADING,                    "Hdg:",  FS_HUD_QTY_ANGLE,    LN_Heading      }, // 13
    { FS_HUD_FIELD_BARO_ALT,                   "Alt:",  FS_HUD_QTY_ALTITUDE, LN_BaroAltitude }, // 14
};
static const unsigned s_lineMapCount = sizeof(s_lineMap) / sizeof(s_lineMap[0]);

/* The live layout: what is drawn, where, in which font. Taken from CONFIG.TXT
 * when the file positions anything, otherwise the tuned-on-hardware built-in
 * (see hud_layout.c). Resolved once per Init, i.e. per glasses connection. */
static FS_HudLayout_t s_layout;

/* Longest string an element can render. The status line is the long one
 * ("X A:100% F:100% N:12 v0.0.16-n.abc1234"); values are far shorter. */
#define AL_MODE0_MAX_TEXT  48

/* Last-displayed string per element, for change detection. */
static char s_lastText[FS_HUD_MAX_ELEMENTS][AL_MODE0_MAX_TEXT];

/* Same job for the navigation arrow, which is a shape and not a string: the
 * quantised bearing last drawn, and whether one was drawn at all. Without
 * these the arrow would only ever be redrawn when the distance caption under
 * it happened to change — an arrow that turns in whole degrees while standing
 * still at a fixed range would freeze. */
static int16_t s_lastArrowDeg[FS_HUD_MAX_ELEMENTS];
static uint8_t s_lastArrowHas[FS_HUD_MAX_ELEMENTS];

/* And for the competition lane indicator, which is a shape too. Packed into one
 * word so the comparison stays a single ==, and so the BLINK is part of what is
 * compared: the whole point of a blink is a frame that differs from the last one
 * when nothing about the flight has changed at all. */
static uint32_t s_lastComp[FS_HUD_MAX_ELEMENTS];

static uint32_t CompSig(const FS_CompInd_t *c, uint8_t is_comp, uint8_t side_on)
{
    return ((uint32_t)is_comp)
         | ((uint32_t)c->active           << 1)
         | ((uint32_t)(c->side + 1)       << 2)   /* 0, 1, 2 */
         | ((uint32_t)c->bars             << 4)
         | ((uint32_t)c->solid            << 9)
         | ((uint32_t)(side_on ? 1u : 0u) << 10);
}

/* The blink, on the render tick that already exists — no timer, no change to
 * AL_Rate. Two ticks lit, two dark: at the default 250 ms that is a 1 Hz square
 * wave, slow enough to read a ladder through and fast enough that it cannot be
 * mistaken for the HUD having stopped. It scales with AL_Rate, which is correct
 * — a wearer who slows the panel down to 500 ms wants everything slower, and a
 * blink pinned to wall-clock time would then be flashing between frames that no
 * longer arrive. */
static uint8_t s_blinkTick;
#define AL_BLINK_ON()  (s_blinkTick < 2u)

/* Status snapshot + ONE shared rebuild divider. The whole status family — the
 * old one-string info line (field 100) and the five pieces it was split into in
 * v0.0.18 (101..105) — is rebuilt only every 5th tick, from the same snapshot,
 * so the pieces always agree with each other and with the line. Sat count and
 * battery jitter would otherwise change one of them nearly every tick, and any
 * single changed string forces a full clear+redraw of the panel: splitting the
 * line must not multiply the chances of that happening (see the rate-limit
 * comment in Mode0_Update). */
static char    battLevels[AL_MODE0_MAX_TEXT];  /* field 100, whole line     */
static char    s_alBattStr[5];                 /* "??" or "0".."100"        */
static uint8_t s_fsBattPct;
static uint8_t s_numSV;
static const char *s_fdMark = FD_MARK_IDLE;
static uint8_t s_hdrDivider;

/* --------------------------------------------------------------------------
   4. Helpers for Sending Commands / Building Layout
   -------------------------------------------------------------------------- */

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

/* Worst case for one frame: holdFlush(HOLD) + clear, then every element, then
 * holdFlush(FLUSH). The cap MUST cover it: an overflow would drop the closing
 * FLUSH and leave the glasses held — the HUD frozen with the link still up,
 * which is exactly the failure this project has spent months chasing.
 *
 * The dearest element is now the competition lane indicator (field 106 under
 * HUD_Mode 1): a centre bar plus up to twelve rungs, 13 shapes. Only ONE is ever
 * built per frame — see where compTaken is set in Mode0_Update — so the most
 * expensive layout is seven navigation arrows (five apiece: a frame, three lines
 * and the distance caption) and one corridor. A text element costs two, value
 * plus unit. The grey-level command is one of the +4 outside the loop, alongside
 * the hold, the clear and the flush. */
#define AL_FRAME_WORST_PKTS  ((FS_HUD_MAX_ELEMENTS - 1) * 5 + FS_COMP_MAX_SHAPES + 4)
#define AL_FRAME_MAX_PKTS    56

#if AL_FRAME_MAX_PKTS < AL_FRAME_WORST_PKTS
#error "AL_FRAME_MAX_PKTS is smaller than a full HUD frame: the closing holdFlush(FLUSH) would be dropped and the glasses would stay held. Raise it."
#endif

static uint8_t  s_framePkt[AL_FRAME_MAX_PKTS][80];
static uint16_t s_framePktLen[AL_FRAME_MAX_PKTS];
static uint8_t  s_frameCount = 0;   /* packets in the current frame            */
static uint8_t  s_frameIdx   = 0;   /* next packet to send (== count when done)*/

/* One line in EVENT.CSV per boot if a frame ever fills up anyway. The #error
 * above says it cannot happen; if the layout ever outgrows the cap regardless,
 * the log is what tells us, instead of a silently truncated frame and a HUD
 * that stops. Latched for the whole session on purpose — the condition would
 * repeat every tick, and a log line 4 times a second would bury the card. */
static bool s_frameFullLogged = false;

static bool AL_FrameFull(void)
{
    if (s_frameCount < AL_FRAME_MAX_PKTS) return false;

    if (!s_frameFullLogged)
    {
        s_frameFullLogged = true;
        FS_Log_WriteEventAsync("ENGO frame full: %u packets, draw dropped",
                               (unsigned)s_frameCount);
    }
    return true;
}

static void AL_FrameReset(void)
{
    s_frameCount = 0;
    s_frameIdx   = 0;
}

/* Append a holdFlush command (AL_HOLD=0x00 or AL_FLUSH=0x01) to the frame. */
static void AL_FrameAddHoldFlush(uint8_t action)
{
    if (AL_FrameFull()) return;
    size_t plen = AL_BuildFrame(s_framePkt[s_frameCount], 80,
                                AL_CMD_HOLD_FLUSH, &action, 1);
    if (plen) { s_framePktLen[s_frameCount] = (uint16_t)plen; s_frameCount++; }
}

/* Append a clear (0x01) command to the frame. */
static void AL_FrameAddClear(void)
{
    if (AL_FrameFull()) return;
    size_t plen = AL_BuildFrame(s_framePkt[s_frameCount], 80,
                                AL_CMD_CLEAR, NULL, 0);
    if (plen) { s_framePktLen[s_frameCount] = (uint16_t)plen; s_frameCount++; }
}

/* Append a txt (0x37) draw to the frame. VERIFIED on ENGO 3 (v8 probe):
 * rotation must be 4; X is mirrored (large x = viewer-left); colour 15. */
static void AL_FrameAddText(int16_t x, int16_t y, uint8_t font, const char *str)
{
    if (AL_FrameFull()) return;
    size_t plen = AL_BuildText(s_framePkt[s_frameCount], 80,
                               x, y, 4, font, 15, str);
    if (plen) { s_framePktLen[s_frameCount] = (uint16_t)plen; s_frameCount++; }
}

/* Append a grey-level (0x30) command. Shapes carry no colour of their own — a
 * line is drawn in whatever level was last set, and that setting persists past
 * a clear and past the end of the frame. One at the top of the shapes is
 * cheaper than trusting what the previous frame left behind. */
static void AL_FrameAddColor(uint8_t grey)
{
    if (AL_FrameFull()) return;
    size_t plen = AL_BuildFrame(s_framePkt[s_frameCount], 80,
                                AL_CMD_COLOR, &grey, 1);
    if (plen) { s_framePktLen[s_frameCount] = (uint16_t)plen; s_frameCount++; }
}

/* Append a line (0x32) or rect (0x33) draw to the frame. */
static void AL_FrameAddShape(uint8_t cmd, int16_t x0, int16_t y0,
                             int16_t x1, int16_t y1)
{
    if (AL_FrameFull()) return;
    size_t plen = AL_BuildShape(s_framePkt[s_frameCount], 80, cmd,
                                x0, y0, x1, y1);
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
 * AL_StatusText()
 *
 * One element of the status family (100..105), rendered from the snapshot taken
 * under the rebuild divider. `showPrefix` is the element's AL_Unit_Show flag: on
 * the three readings it turns on the SAME one- or two-character prefix the whole
 * line has always used, so a wearer who splits the line and switches the
 * prefixes on gets back exactly the reading they had. It is not a unit — '%' is
 * part of the value, since a battery percentage without it is a number nobody
 * can place — but it is the same "tell me what this is" flag, and inventing a
 * second key for it would be a key the app would have to learn for nothing.
 * The version and the marker ignore the flag; AL_Units and AL_Dec are
 * meaningless for all six and are ignored rather than rejected.
 */
static void AL_StatusText(uint8_t field, uint8_t showPrefix, char *out, size_t n)
{
    switch (field) {
    case FS_HUD_FIELD_INFO:
        strncpy(out, battLevels, n - 1);
        out[n - 1] = '\0';
        break;
    case FS_HUD_FIELD_AL_BATT:
        snprintf(out, n, "%s%s%%", showPrefix ? "A:" : "", s_alBattStr);
        break;
    case FS_HUD_FIELD_FS_BATT:
        snprintf(out, n, "%s%d%%", showPrefix ? "F:" : "", (int)s_fsBattPct);
        break;
    case FS_HUD_FIELD_SATS:
        /* No fix needed and none implied: this reads 0 before the first fix,
         * which is precisely when a wearer looks at it. */
        snprintf(out, n, "%s%d", showPrefix ? "N:" : "", (int)s_numSV);
        break;
    case FS_HUD_FIELD_VERSION:
        /* Same string the whole line ends with — the point of it is to prove
         * remotely which firmware is live on the glasses. */
        snprintf(out, n, "v%s", HUD_VERSION);
        break;
    case FS_HUD_FIELD_FD_MARK:
        snprintf(out, n, "%s", s_fdMark);
        break;
    case FS_HUD_FIELD_FLT_TIME: {
        /* Read LIVE, not from the snapshot above, and the one member of the
         * family that is. The snapshot exists so battery and satellite jitter
         * cannot force a full clear+redraw every tick; a clock is the opposite
         * case — changing is its entire job, and sampling it on the 5-tick
         * divider (1.25 s at the default AL_Rate) would drop roughly one second
         * in five, which on a stopwatch reads as a fault rather than as
         * rounding. The redraw it costs is one per second at most, and only
         * once a takeoff has been detected: before that the string is fixed and
         * change detection sends nothing at all. */
        uint32_t sec;
        if (!FS_FlightDetect_ElapsedSec(&sec)) {
            snprintf(out, n, "%s", FLT_TIME_IDLE);
        } else {
            /* No leading zero on the minutes, so a jump run reads 7:12 the way
             * a watch shows it; three digits simply appear if the device is
             * left running long enough, rather than a truncated hour. */
            snprintf(out, n, "%u:%02u",
                     (unsigned)(sec / 60u), (unsigned)(sec % 60u));
        }
        break;
    }
    default:
        out[0] = '\0';
        break;
    }
}


/**
 * FS_ActiveLook_Mode0_Init()
 *
 * Resolve the layout to draw, and clear the change-detection state so the
 * first update after a connection always sends a full frame.
 */
void FS_ActiveLook_Mode0_Init(void)
{
    const FS_Config_Data_t *cfg = FS_Config_Get();

    if (cfg->al_layout_valid)
    {
        /* The file positions its own elements — use it as it stands. */
        s_layout = cfg->al_layout;
    }
    else
    {
        /* Nothing positioned in CONFIG.TXT: the built-in layout, tuned on
         * hardware and approved pixel by pixel (Docs/HUD_LAYOUT.md). The
         * global offset still applies — it is a fit adjustment for the
         * wearer's glasses, not part of the layout. */
        FS_HudLayout_Default(&s_layout);
        s_layout.shift_x = cfg->al_layout.shift_x;
        s_layout.shift_y = cfg->al_layout.shift_y;
    }
    FS_HudLayout_Clamp(&s_layout);

    /* Reset change-detection buffers so first update always sends */
    for (int i = 0; i < FS_HUD_MAX_ELEMENTS; i++) {
        s_lastText[i][0]  = '\0';
        s_lastArrowDeg[i] = 0;
        s_lastArrowHas[i] = 0;
        s_lastComp[i]     = 0;
    }
    battLevels[0] = '\0';   /* status snapshot: rebuild on the next tick */
    s_hdrDivider  = 0;
    s_blinkTick   = 0;
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
 * Called periodically to redraw the display. Walks the resolved layout: each
 * element is turned into a string (a formatted value, or the status line),
 * compared with what is already on the glasses, and — if anything at all
 * changed — the whole screen is redrawn as one clear+txt frame.
 */
void FS_ActiveLook_Mode0_Update(void)
{
    /* Link is up by the time we render: if we connected to a new (unbound) pair
     * of glasses, persist their serial to /engo3.txt now (once, task context). */
    FS_EngoBind_CommitIfPending();
    /* One-shot per-session proof that we actually connected (and to which serial). */
    FS_EngoBind_LogLinkedOnce();

    const FS_GNSS_Data_t   *gnss = FS_GNSS_GetData();
    const FS_VBAT_Data_t   *vbat = FS_VBAT_GetData();
    const FS_Config_Data_t *cfg  = FS_Config_Get();

    /* Advanced BEFORE the change-detection bail-out below: a blinking indicator
     * is precisely the case where nothing else on the panel is moving, so a
     * phase that only advanced on frames we already decided to send would stop
     * dead exactly when it is needed. */
    s_blinkTick = (uint8_t)((s_blinkTick + 1u) & 3u);

    /* Info/status line: takeoff marker (left-most), glasses batt, FlySight batt,
     * sat count, then the HUD version at the END (per request). The trailing
     * version also confirms REMOTELY which firmware is live on the glasses (if it
     * doesn't change after a flash, the glasses are frozen / the flash failed).
     * Since v0.0.18 each of those five pieces can also be drawn on its own, as
     * fields 101..105, in its own place and font — the snapshot below feeds both.
     *
     * Rate-limited: the snapshot is REBUILT only every 5th tick. Sat count /
     * battery jitter otherwise changes the header nearly every tick, forcing a
     * full clear+redraw burst each second even when the data lines are static —
     * needless pressure on the glasses' command buffer (XCTrack: "not much
     * bandwidth available"). Takeoff-marker latency of <=5 ticks is fine, and
     * has been the behaviour of the marker inside the line since v0.0.14. */
    if (s_hdrDivider == 0 || battLevels[0] == '\0') {
        uint8_t alBatt = FS_ActiveLook_Client_GetBatteryLevel();
        if (alBatt == 255)
            snprintf(s_alBattStr, sizeof(s_alBattStr), "??");
        else
            snprintf(s_alBattStr, sizeof(s_alBattStr), "%d", alBatt);

        /* AL_BatteryPct for consistent clamped math. */
        s_fsBattPct = AL_BatteryPct(vbat->voltage);
        s_numSV     = gnss->numSV;
        s_fdMark    = FS_FlightDetect_InFlight() ? FD_MARK_DETECTED : FD_MARK_IDLE;

        snprintf(battLevels, sizeof(battLevels),
                 "%s A:%s%% F:%d%% N:%d v%s", s_fdMark, s_alBattStr,
                 (int)s_fsBattPct, s_numSV, HUD_VERSION);
    }
    s_hdrDivider = (uint8_t)((s_hdrDivider + 1) % 5);

    /* --- Render each element of the layout to a string --- */
    /* Static on purpose: 8x48 = 384 bytes would otherwise land on the task
     * stack (the old fixed-layout code kept ~96 bytes there). The render tick
     * is single-threaded under the sequencer, so one buffer is safe. */
    static char text[FS_HUD_MAX_ELEMENTS][AL_MODE0_MAX_TEXT];
    /* The quantity each element ended up drawing, for the unit draw below. It
     * stays FS_HUD_QTY_NONE — i.e. no unit — for everything that is not a live
     * value: the status family, an empty or unknown field, and a GPS-derived
     * reading with no fix, which shows "----". A bare "km/h" beside four dashes
     * would say nothing; v0.0.18 printed no suffix there either. */
    static FS_HudQuantity_t qty[FS_HUD_MAX_ELEMENTS];
    /* The navigation arrow's shape, resolved in the same pass: the quantised
     * bearing, and whether there is one to draw at all. */
    static int16_t arrowDeg[FS_HUD_MAX_ELEMENTS];
    static uint8_t arrowHas[FS_HUD_MAX_ELEMENTS];
    /* The competition lane indicator, resolved in the same pass: its state, a
     * flag saying this element is one at all (so the frame builder knows not to
     * draw text there), and whether the ladder is lit this frame. */
    static FS_CompInd_t comp[FS_HUD_MAX_ELEMENTS];
    static uint8_t      compEl[FS_HUD_MAX_ELEMENTS];
    static uint8_t      compOn[FS_HUD_MAX_ELEMENTS];
    for (int i = 0; i < FS_HUD_MAX_ELEMENTS; i++)
    {
        text[i][0]  = '\0';
        qty[i]      = FS_HUD_QTY_NONE;
        arrowDeg[i] = 0;
        arrowHas[i] = 0;
        memset(&comp[i], 0, sizeof(comp[i]));
        compEl[i]   = 0;
        compOn[i]   = 0;
    }

    /* At most one lane indicator per frame. Twelve rungs and a bar are thirteen
     * packets, and a frame that outgrew AL_FRAME_MAX_PKTS would lose its closing
     * FLUSH and freeze the glasses with the link still up. Nothing is lost by
     * the cap: a second copy of the indicator is the same lane. A card that asks
     * for two gets the corridor in the first slot and keeps the stopwatch in the
     * rest, which at least says something the first one does not. */
    bool compTaken = false;

    for (int i = 0; i < s_layout.count; i++)
    {
        const FS_HudElement_t *el = &s_layout.el[i];

        /* Field 106 under HUD_Mode 1 is a picture rather than a reading: the
         * Designated Lane, as a ladder about a centre bar. See comp_corridor.h
         * for the rules and for what each part of it means.
         *
         * The deviation is held to the same standard as every other GPS-derived
         * reading on this panel — a 3D fix, a receiver that is still talking,
         * and a Ground Reference Point on the card. Short of any of those the
         * indicator falls back to its bare centre bar, which says "the window is
         * open and I cannot place you" rather than freezing the last ladder,
         * which would be a competitor steering on a number from a minute ago. */
        /* The top line has two jobs and only room for one at a time, so it
         * SWAPS at takeoff rather than showing both crammed together.
         *
         * On the ground the wearer needs the housekeeping: which firmware is
         * live, how much battery there is, how many satellites. In the air
         * none of that can be acted on — the aircraft door has closed on any
         * decision it could inform — and the strip is worth more as a clock
         * and a lane.
         *
         * So the status family blanks itself once flight is detected, and the
         * clock says nothing until then. One card carries both; the wearer
         * places each once and never touches the layout again. */
        if (FS_HudLayout_FieldIsStatus(el->field) && FS_FlightDetect_InFlight())
            continue;
        if (el->field == FS_HUD_FIELD_FLT_TIME && !FS_FlightDetect_InFlight())
            continue;

        /* The corridor only replaces the clock once it HAS a corridor. Between
         * takeoff and the Validation Window opening — the climb, the run-in,
         * the first nine seconds of the dive — there is no origin to draw a
         * lane about, and a blank strip through the whole ride up would be
         * worse than the clock that was already there. */
        if (el->field == FS_HUD_FIELD_FLT_TIME && cfg->hud_mode == 1 &&
            FS_CompCorridor_Started() && !compTaken)
        {
            float dev = 0.0f;
            const int haveDev =
                gnss->gpsFix == 3 && !FS_GNSS_IsStale() &&
                FS_CompCorridor_RefSet(cfg->comp_lat, cfg->comp_lon) &&
                FS_CompCorridor_Deviation(cfg->comp_lat, cfg->comp_lon, &dev);

            FS_CompCorridor_Indicator(FS_CompCorridor_Started(), haveDev, dev,
                                      &comp[i]);
            compEl[i]  = 1;
            compOn[i]  = (!comp[i].blink || AL_BLINK_ON()) ? 1 : 0;
            compTaken  = true;
            continue;   /* text[i] stays empty: nothing is written in this slot */
        }

        if (FS_HudLayout_FieldIsStatus(el->field)) {
            AL_StatusText(el->field, el->show_units, text[i], AL_MODE0_MAX_TEXT);
            continue;
        }
        if (el->field == FS_HUD_FIELD_NONE)
            continue;

        /* The arrow is not in the line map: it draws a shape, and its string is
         * only the caption underneath. Same honesty rules as every GPS-derived
         * reading — no arrow at all when there is no fresh fix or no
         * destination on the card. An arrow drawn from stale coordinates would
         * point somewhere with total confidence.
         *
         * Where the other elements can say nothing more than "----", this one
         * NAMES the reason: the two silences have different cures, and one of
         * them has to be applied before boarding. See FS_NavArrow_State. */
        if (el->field == FS_HUD_FIELD_NAV_ARROW)
        {
            const FS_NavArrowState_t nav = FS_NavArrow_State(
                NavDestSet(),
                gnss->gpsFix == 3 && !FS_GNSS_IsStale());

            if (nav != FS_NAV_ARROW_OK) {
                snprintf(text[i], AL_MODE0_MAX_TEXT, "%s",
                         FS_NavArrow_Caption(nav));
                continue;   /* arrowHas stays 0 — the box gets the bang below */
            }

            FS_HudUnitConv_t dconv =
                FS_HudLayout_UnitConv(FS_HUD_QTY_DISTANCE, el->units);
            double dist = LN_DistToDest(gnss) * dconv.multiplier;

            /* The unit goes INSIDE the caption here, unlike every other
             * element. FS_HudLayout_UnitDraw places a suffix past the width
             * RESERVED for a value — five digits and a sign for a distance —
             * which would push "m" clear of the box the caption belongs to. */
            if (el->show_units && dconv.suffix[0] != '\0')
                snprintf(text[i], AL_MODE0_MAX_TEXT, "%.*f %s",
                         (int)el->decimals, dist, dconv.suffix);
            else
                snprintf(text[i], AL_MODE0_MAX_TEXT, "%.*f",
                         (int)el->decimals, dist);

            int deg = NavArrowDeg(gnss);
            arrowDeg[i] = (int16_t)((deg / NAV_ARROW_STEP_DEG) * NAV_ARROW_STEP_DEG);
            arrowHas[i] = 1;
            continue;
        }

        const AL_Mode0_LineMap_t *entry = FindLineMapEntry(el->field);

        /* A GPS-derived value needs a fix AND a receiver that is still
         * talking. gpsFix only records what was true when the last sample
         * arrived: if the module, the UART or the message pairing stalls, the
         * whole struct keeps its values and its fix flag, and the HUD would go
         * on showing a speed and a glide from some minutes ago as though they
         * were now. Dashes are honest; a stale number is not. */
        if (entry == NULL ||
            (FS_HudLayout_FieldNeedsFix(el->field) &&
             (gnss->gpsFix != 3 || FS_GNSS_IsStale()))) {
            /* Unknown field, no 3D fix yet, or nothing fresh to draw. */
            snprintf(text[i], AL_MODE0_MAX_TEXT, "----");
            continue;
        }

        FS_HudUnitConv_t conv = FS_HudLayout_UnitConv(entry->qty, el->units);
        double v = entry->fn(gnss) * conv.multiplier;

        /* The value alone, always — byte for byte what v0.0.18 wrote with
         * AL_Unit_Show off. Since v0.0.19 the unit is no longer part of this
         * string: it is a second txt in font 0, placed by FS_HudLayout_UnitDraw
         * when the frame is built below. Drawn in the value's own font, "km/h"
         * at 64 or 75 px took more of the panel than the number did. */
        snprintf(text[i], AL_MODE0_MAX_TEXT, "%.*f", (int)el->decimals, v);
        qty[i] = entry->qty;
    }

    /* --- Change detection: skip the BLE write if nothing changed --- */
    bool changed = false;
    for (int i = 0; i < s_layout.count; i++) {
        if (strcmp(text[i], s_lastText[i]) != 0 ||
            arrowDeg[i] != s_lastArrowDeg[i] ||
            arrowHas[i] != s_lastArrowHas[i] ||
            CompSig(&comp[i], compEl[i], compOn[i]) != s_lastComp[i]) {
            changed = true;
            break;
        }
    }

    if (!changed) {
        /* Nothing new to send — skip this tick entirely */
        return;
    }

    /* Update cached strings */
    for (int i = 0; i < s_layout.count; i++) {
        strncpy(s_lastText[i], text[i], AL_MODE0_MAX_TEXT - 1);
        s_lastText[i][AL_MODE0_MAX_TEXT - 1] = '\0';
        s_lastArrowDeg[i] = arrowDeg[i];
        s_lastArrowHas[i] = arrowHas[i];
        s_lastComp[i]     = CompSig(&comp[i], compEl[i], compOn[i]);
    }

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
    bool greySent = false;
    for (int i = 0; i < s_layout.count; i++) {
        int16_t x, y;
        if (!FS_HudLayout_Place(&s_layout, (uint8_t)i, &x, &y))
            continue;

        if (compEl[i]) {
            /* Height from the element's own font, so the ladder scales with the
             * numbers beside it exactly as the arrow's box does. Width follows
             * from the height inside comp_corridor.c: 72 px at the 24 px status
             * font, which is what the "--:--" clock in this slot occupied.
             *
             * Nothing at all is emitted before the Validation Window opens — not
             * an empty box, not a placeholder. The centre bar APPEARING is the
             * signal the competitor is waiting for, and it cannot be a signal if
             * something was already sitting there. */
            FS_CompDraw_t lane;
            const int16_t size =
                (int16_t)FS_HudLayout_FontHeight(s_layout.el[i].font);

            if (FS_CompCorridor_Build(x, y, size, &comp[i], compOn[i], &lane)) {
                if (!greySent) { AL_FrameAddColor(15); greySent = true; }
                for (uint8_t s = 0; s < lane.count; s++)
                    AL_FrameAddShape(lane.shape[s].solid ? AL_CMD_RECTF
                                                         : AL_CMD_LINE,
                                     lane.shape[s].x0, lane.shape[s].y0,
                                     lane.shape[s].x1, lane.shape[s].y1);
            }
            continue;
        }

        if (s_layout.el[i].field == FS_HUD_FIELD_NAV_ARROW) {
            /* The box is a square as tall as a character of the element's own
             * font, so AL_Font sizes the arrow the same way it sizes a number
             * and a layout keeps its proportions when the wearer scales it. */
            const int16_t size =
                (int16_t)FS_HudLayout_FontHeight(s_layout.el[i].font);

            FS_NavArrow_t arrow;
            if (FS_NavArrow_Build(x, y, size, arrowHas[i],
                                  (float)arrowDeg[i], &arrow)) {
                if (!greySent) { AL_FrameAddColor(15); greySent = true; }
                AL_FrameAddShape(AL_CMD_RECT, arrow.box_x0, arrow.box_y0,
                                 arrow.box_x1, arrow.box_y1);
                for (uint8_t s = 0; s < arrow.seg_count; s++)
                    AL_FrameAddShape(AL_CMD_LINE,
                                     arrow.seg[s].x0, arrow.seg[s].y0,
                                     arrow.seg[s].x1, arrow.seg[s].y1);

                /* No arrow means a warning, and a warning has to carry inside
                 * the frame as well as under it: the caption is drawn in the
                 * smallest font, and at arm's length in daylight an empty box
                 * with small text beneath it still reads as "nothing yet".
                 * The bang goes in the element's own font so it fills the box
                 * at any scale, centred with the same mirrored arithmetic the
                 * caption uses. */
                if (!arrowHas[i]) {
                    const uint8_t wadv =
                        FS_HudLayout_FontAdvance(s_layout.el[i].font);
                    int32_t wslack = (int32_t)size - (int32_t)wadv;
                    if (wslack < 0) wslack = 0;
                    AL_FrameAddText((int16_t)(x - wslack / 2), y,
                                    s_layout.el[i].font,
                                    FS_NAV_ARROW_WARN_GLYPH);
                }
            }

            /* The caption, centred under the box in the smallest font. The
             * anchor is the string's LEFT edge as the wearer sees it and text
             * grows to the wearer's right, so centring SUBTRACTS half the
             * slack — the same mirrored arithmetic as everywhere else. */
            const uint8_t adv = FS_HudLayout_FontAdvance(FS_HUD_UNIT_FONT);
            const int32_t w   = (int32_t)strlen(text[i]) * (int32_t)adv;
            int32_t slack = (int32_t)size - w;
            if (slack < 0) slack = 0;

            AL_FrameAddText((int16_t)(x - slack / 2),
                            (int16_t)(y - size - FS_HUD_UNIT_GAP),
                            FS_HUD_UNIT_FONT, text[i]);
            continue;
        }

        AL_FrameAddText(x, y, s_layout.el[i].font, text[i]);

        /* AL_Unit_Show: the unit as its own draw, in the smallest font, sitting
         * one gap past the width RESERVED for the value (not past the width of
         * today's reading — a unit that moved whenever the value crossed 100
         * would read as a fault). Nothing is emitted at all when the element
         * draws no unit, so a layout with the flag off sends exactly the frame
         * v0.0.18 sent. */
        FS_HudUnitDraw_t u;
        if (FS_HudLayout_UnitDraw(&s_layout.el[i], qty[i], x, y, &u))
            AL_FrameAddText(u.x, u.y, u.font, u.text);
    }
    AL_FrameAddHoldFlush(AL_FLUSH);
    FS_ActiveLook_Mode0_DrainFrame();
}
