/***************************************************************************
**                                                                        **
**  FlySight 2 firmware                                                   **
**  Copyright 2023 Bionic Avionics Inc.                                   **
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

#include "main.h"
#include "app_common.h"
#include "config.h"
#include "ff.h"
#include "state.h"

#define CONFIG_FIRST_ALARM   0x01
#define CONFIG_FIRST_WINDOW  0x02
#define CONFIG_FIRST_SPEECH  0x04
#define CONFIG_FIRST_AL_LINE 0x08

static FS_Config_Data_t config;
static FIL configFile;

static const char defaultConfig[] =
		"; FlySight - http://flysight.ca\n"
		"\n"
		"; GPS settings\n"
		"\n"
		"Model:     7     ; Dynamic model\n"
		"                 ;   0 = Portable\n"
		"                 ;   2 = Stationary\n"
		"                 ;   3 = Pedestrian\n"
		"                 ;   4 = Automotive\n"
		"                 ;   5 = Sea\n"
		"                 ;   6 = Airborne with < 1 G acceleration\n"
		"                 ;   7 = Airborne with < 2 G acceleration\n"
		"                 ;   8 = Airborne with < 4 G acceleration\n"
		"Rate:      200   ; Measurement rate (ms)\n"
		"\n"
		"; Tone settings\n"
		"\n"
		"Mode:      2     ; Measurement mode\n"
		"                 ;   0 = Horizontal speed\n"
		"                 ;   1 = Vertical speed\n"
		"                 ;   2 = Glide ratio\n"
		"                 ;   3 = Inverse glide ratio\n"
		"                 ;   4 = Total speed\n"
		"                 ;   5 = Direction to destination\n"
		"                 ;   6 = Distance to destination\n"
		"                 ;   7 = Direction to bearing\n"
		"                 ;   11 = Dive angle\n"
		"Min:       0     ; Lowest pitch value\n"
		"                 ;   cm/s        in Mode 0, 1, or 4\n"
		"                 ;   ratio * 100 in Mode 2 or 3\n"
		"                 ;   degrees     in Mode 11\n"
		"Max:       300   ; Highest pitch value\n"
		"                 ;   cm/s        in Mode 0, 1, or 4\n"
		"                 ;   ratio * 100 in Mode 2 or 3\n"
		"                 ;   degrees     in Mode 11\n"
		"Limits:    1     ; Behaviour when outside bounds\n"
		"                 ;   0 = No tone\n"
		"                 ;   1 = Min/max tone\n"
		"                 ;   2 = Chirp up/down\n"
		"                 ;   3 = Chirp down/up\n"
		"Volume:    6     ; 0 (min) to 8 (max)\n"
		"\n"
		"; Rate settings\n"
		"\n"
		"Mode_2:    9     ; Determines tone rate\n"
		"                 ;   0 = Horizontal speed\n"
		"                 ;   1 = Vertical speed\n"
		"                 ;   2 = Glide ratio\n"
		"                 ;   3 = Inverse glide ratio\n"
		"                 ;   4 = Total speed\n"
		"                 ;   8 = Magnitude of Value 1\n"
		"                 ;   9 = Change in Value 1\n"
		"                 ;   11 = Dive angle\n"
		"Min_Val_2: 300   ; Lowest rate value\n"
		"                 ;   cm/s          when Mode 2 = 0, 1, or 4\n"
		"                 ;   ratio * 100   when Mode 2 = 2 or 3\n"
		"                 ;   percent * 100 when Mode 2 = 9\n"
		"                 ;   degrees       when Mode 2 = 11\n"
		"Max_Val_2: 1500  ; Highest rate value\n"
		"                 ;   cm/s          when Mode 2 = 0, 1, or 4\n"
		"                 ;   ratio * 100   when Mode 2 = 2 or 3\n"
		"                 ;   percent * 100 when Mode 2 = 9\n"
		"                 ;   degrees       when Mode 2 = 11\n"
		"Min_Rate:  100   ; Minimum rate (Hz * 100)\n"
		"Max_Rate:  500   ; Maximum rate (Hz * 100)\n"
		"Flatline:  0     ; Flatline at minimum rate\n"
		"                 ;   0 = No\n"
		"                 ;   1 = Yes\n"
		"\n"
		"; Speech settings\n"
		"\n"
		"Sp_Rate:   0     ; Speech rate (s)\n"
		"                 ;   0 = No speech\n"
		"Sp_Volume: 6     ; 0 (min) to 8 (max)\n"
		"\n"
		"Sp_Mode:   2     ; Speech mode\n"
		"                 ;   0 = Horizontal speed\n"
		"                 ;   1 = Vertical speed\n"
		"                 ;   2 = Glide ratio\n"
		"                 ;   3 = Inverse glide ratio\n"
		"                 ;   4 = Total speed\n"
		"                 ;   5 = Direction to destination\n"
		"                 ;   6 = Distance to destination\n"
		"                 ;   7 = Direction to bearing\n"
		"                 ;   11 = Dive angle\n"
		"                 ;   12 = Altitude above DZ_Elev\n"
		"Sp_Units:  1     ; Speech units\n"
		"                 ;   0 = km/h or m\n"
		"                 ;   1 = mph or feet\n"
		"Sp_Dec:    1     ; Speech precision\n"
		"                 ;   Altitude step in Mode 12\n"
		"                 ;   Decimal places in all other Modes\n"
		"\n"
		"; Thresholds\n"
		"\n"
		"V_Thresh:  1000  ; Minimum vertical speed for tone (cm/s)\n"
		"H_Thresh:  0     ; Minimum horizontal speed for tone (cm/s)\n"
		"\n"
		"; Miscellaneous\n"
		"\n"
		"Use_SAS:   1     ; Use skydiver's airspeed\n"
		"                 ;   0 = No\n"
		"                 ;   1 = Yes\n"
		"TZ_Offset: 0     ; Timezone offset of output files in seconds\n"
		"                 ;   -14400 = UTC-4 (EDT)\n"
		"                 ;   -18000 = UTC-5 (EST, CDT)\n"
		"                 ;   -21600 = UTC-6 (CST, MDT)\n"
		"                 ;   -25200 = UTC-7 (MST, PDT)\n"
		"                 ;   -28800 = UTC-8 (PST)\n"
		"\n"
		"; Initialization\n"
		"\n"
		"Init_Mode: 0     ; When the FlySight is powered on\n"
		"                 ;   0 = Do nothing\n"
		"                 ;   1 = Test speech mode\n"
		"                 ;   2 = Play file\n"
		"Init_File: 0     ; File to be played\n"
		"\n"
		"; Alarm settings\n"
		"\n"
		"; WARNING: GPS measurements depend on very weak signals\n"
		";          received from orbiting satellites. As such, they\n"
		";          are prone to interference, and should NEVER be\n"
		";          relied upon for life saving purposes.\n"
		"\n"
		";          UNDER NO CIRCUMSTANCES SHOULD THESE ALARMS BE\n"
		";          USED TO INDICATE DEPLOYMENT OR BREAKOFF ALTITUDE.\n"
		"\n"
		"; NOTE:    Alarm elevations are given in meters above ground\n"
		";          elevation, which is specified in DZ_Elev.\n"
		"\n"
		"Win_Above:     0 ; Window above each alarm (m)\n"
		"Win_Below:     0 ; Window below each alarm (m)\n"
		"DZ_Elev:       0 ; Ground elevation (m above sea level)\n"
		"\n"
		"Alarm_Elev:    0 ; Alarm elevation (m above ground level)\n"
		"Alarm_Type:    0 ; Alarm type\n"
		"                 ;   0 = No alarm\n"
		"                 ;   1 = Beep\n"
		"                 ;   2 = Chirp up\n"
		"                 ;   3 = Chirp down\n"
		"                 ;   4 = Play file\n"
		"Alarm_File:    0 ; File to be played\n"
		"\n"
		"; Altitude mode settings\n"
		"\n"
		"; WARNING: GPS measurements depend on very weak signals\n"
		";          received from orbiting satellites. As such, they\n"
		";          are prone to interference, and should NEVER be\n"
		";          relied upon for life saving purposes.\n"
		"\n"
		";          UNDER NO CIRCUMSTANCES SHOULD ALTITUDE MODE BE\n"
		";          USED TO INDICATE DEPLOYMENT OR BREAKOFF ALTITUDE.\n"
		"\n"
		"; NOTE:    Altitude is given relative to ground elevation,\n"
		";          which is specified in DZ_Elev. Altitude mode will\n"
		";          not function below 1500 m above ground.\n"
		"\n"
		"Alt_Units:     1 ; Altitude units\n"
		"                 ;   0 = m\n"
		"                 ;   1 = ft\n"
		"Alt_Step:      0 ; Altitude between announcements\n"
		"                 ;   0 = No altitude\n"
		"\n"
		"; Silence windows\n"
		"\n"
		"; NOTE:    Silence windows are given in meters above ground\n"
		";          elevation, which is specified in DZ_Elev. Tones\n"
		";          will be silenced during these windows and only\n"
		";          alarms will be audible.\n"
		"\n"
		"Win_Top:       0 ; Silence window top (m)\n"
		"Win_Bottom:    0 ; Silence window bottom (m)\n"
		"\n"
		"; Navigation settings\n"
		"\n"
		"; Activating navigation features requires the UNIQUE DEVICE ID\n"
		"; of this specific FlySight device to be entered below. This is\n"
		"; a deliberate step to ensure users understand the potential\n"
		"; implications of using navigation features.\n"
		"\n"
		"; BEFORE ENABLING THIS FEATURE, YOU MUST ACKNOWLEDGE THE\n"
		"; FOLLOWING:\n"
		"\n"
		"; 1. NAVIGATION DATA IS FOR SUPPLEMENTARY INFORMATION ONLY. It\n"
		";    should NEVER be your primary source of navigation or\n"
		";    situational awareness.\n"
		"\n"
		"; 2. DO NOT USE THIS FEATURE FOR SAFETY-CRITICAL DECISIONS.\n"
		";    Always prioritize visual confirmation, established safety\n"
		";    procedures, and communication.\n"
		"\n"
		"; 3. Be aware that other users may independently configure\n"
		";    similar navigation targets, potentially leading to\n"
		";    convergent flight paths. This system provides NO collision\n"
		";    avoidance.\n"
		"\n"
		"; 4. You are solely responsible for your flight path and safety.\n"
		"\n"
		"; ENTER YOUR FLYSIGHT'S UNIQUE DEVICE ID BELOW TO ACTIVATE\n"
		"; NAVIGATION FEATURES:\n"
		"\n"
		"Device_ID: 000000000000000000000000\n"
		"\n"
		"Lat:       510442700    ; Latitude (deg * 10,000,000)\n"
		"Lon:       -1140620190  ; Longitude (deg * 10,000,000)\n"
		"Bearing:   0            ; Bearing (deg)\n"
		"End_Nav:   1500         ; Minimum altitude (m)\n"
		"Max_Dist:  10000        ; Maximum distance (m)\n"
		"Min_Angle: 5            ; Minimum angle for direction (deg)\n"
		"\n"
		"; Wingsuit competition. The Ground Reference Point the Chief\n"
		"; Judge assigns: the far end of the Designated Flight Path.\n"
		"; The near end is where you are 9 s after first reaching 10 m/s\n"
		"; down, which the device works out for itself. Only used when\n"
		"; HUD_Mode is 1. Leave both at 0 if there is no assigned point.\n"
		"\n"
		"Comp_Lat:  0            ; Reference latitude (deg * 10,000,000)\n"
		"Comp_Lon:  0            ; Reference longitude (deg * 10,000,000)\n"
		"\n"
		"; ActiveLook interface\n"
		"\n"
		"AL_ID:    000000 ; ActiveLook device ID\n"
		"AL_Mode:       1 ; ActiveLook mode\n"
		"                     0 = Not active\n"
		"                     1 = Default mode\n"
		"AL_Rate:     250 ; ActiveLook rate (ms)\n"
		"                 ;   4 Hz. Raise it (333, 500) if the glasses\n"
		"                 ;   ever stop refreshing mid-flight.\n"
		"HUD_Mode:      0 ; What the HUD is for\n"
		"                 ;   0 = Default\n"
		"                 ;   1 = Wingsuit competition. Turns line 106\n"
		"                 ;       from a stopwatch into the Designated\n"
		"                 ;       Lane indicator: nothing until the\n"
		"                 ;       Validation Window opens, then a centre\n"
		"                 ;       bar plus one bar per 25 m off the path\n"
		"                 ;       to Comp_Lat / Comp_Lon. Needs those two.\n"
		"\n"
		"; HUD position. Shifts the whole picture, for glasses that sit a\n"
		"; little off-centre. Both are in pixels, as the WEARER sees it:\n"
		"; positive X moves the picture right, positive Y moves it up.\n"
		"\n"
		"AL_Shift_X:    0 ; -120 to 120\n"
		"AL_Shift_Y:    0 ; -120 to 120\n"
		"\n"
		"; HUD layout. Each AL_Line starts one element; the keys under it\n"
		"; describe that element. Screen is 304 x 256. X is MIRRORED: x=296\n"
		"; is the wearer's LEFT edge, x=0 the right one. Larger Y is higher,\n"
		"; and text hangs DOWN from its anchor.\n"
		"\n"
		"AL_Line:       0 ; ActiveLook line value\n"
		"                 ;   0 = Horizontal speed\n"
		"                 ;   1 = Vertical speed\n"
		"                 ;   2 = Glide ratio\n"
		"                 ;   3 = Inverse glide ratio\n"
		"                 ;   4 = Total speed\n"
		"                 ;   5 = Direction to destination\n"
		"                 ;   6 = Distance to destination\n"
		"                 ;   7 = Direction to bearing\n"
		"                 ;   11 = Dive angle\n"
		"                 ;   12 = Altitude above DZ_Elev\n"
		"                 ;   13 = Course\n"
		"                 ;   14 = Altitude, barometric (zeroed at power-on)\n"
		"                 ;   15 = Arrow to the destination, in a box, with\n"
		"                 ;        the distance under it. Forward is up. The\n"
		"                 ;        box is a square as tall as AL_Font, and\n"
		"                 ;        AL_Units / AL_Dec / AL_Unit_Show describe\n"
		"                 ;        the distance underneath. Unlike 5, it\n"
		"                 ;        ignores Max_Dist and points at any range.\n"
		"                 ;   100 = Status line (battery, satellites, version)\n"
		"                 ;   The status line, in five separate pieces:\n"
		"                 ;   101 = Glasses battery      102 = FlySight battery\n"
		"                 ;   103 = Satellites           104 = HUD version\n"
		"                 ;   105 = Takeoff marker\n"
		"                 ;   106 = Time since takeoff, M:SS. Reads\n"
		"                 ;         --:-- until a takeoff is detected.\n"
		"                 ;         Under HUD_Mode 1 this slot draws the\n"
		"                 ;         competition lane indicator instead.\n"
		"AL_Units:      0 ; ActiveLook units\n"
		"                 ;   0 = metric      1 = imperial\n"
		"                 ;   Or name one outright:\n"
		"                 ;   2 = km/h   3 = m/s   4 = mph   5 = ft/s\n"
		"                 ;   6 = m      7 = ft    8 = km    9 = mi\n"
		"                 ;   A unit that does not fit the line (km on a\n"
		"                 ;   speed) reads as the metric one.\n"
		"AL_Dec:        0 ; ActiveLook precision\n"
		"                 ;   Decimal places\n"
		"AL_Unit_Show:  0 ; Draw the unit after the value\n"
		"                 ;   0 = 148        1 = 148 km/h\n"
		"                 ;   The unit is drawn small (24 px) whatever\n"
		"                 ;   font the value uses, just past the width\n"
		"                 ;   the value reserves. It still costs panel\n"
		"                 ;   width; the positions below were tuned\n"
		"                 ;   with bare numbers.\n"
		"                 ;   On 101, 102 and 103 it draws the prefix\n"
		"                 ;   instead: 80% becomes A:80%.\n"
		"AL_X:        268 ; Position, 0 to 303\n"
		"AL_Y:        208 ; Position, 0 to 255\n"
		"AL_Font:       3 ; Text size\n"
		"                 ;   0, 1 = 24 px   6 = 32 px   2 = 38 px\n"
		"                 ;   7 = 48 px      3 = 64 px   4 = 75 px\n"
		"                 ;   5 = 82 px\n"
		"\n"
		"AL_Line:       1 ; Vertical speed\n"
		"AL_Units:      0 ; km/h\n"
		"AL_Dec:        0 ; Decimal places\n"
		"AL_Unit_Show:  0 ; No unit suffix\n"
		"AL_X:        134\n"
		"AL_Y:        208\n"
		"AL_Font:       3\n"
		"\n"
		"AL_Line:       2 ; Glide ratio\n"
		"AL_Units:      0 ; No units\n"
		"AL_Dec:        2 ; Decimal places\n"
		"AL_Unit_Show:  0 ; Glide ratio is unitless, so this does nothing\n"
		"AL_X:        250\n"
		"AL_Y:        147\n"
		"AL_Font:       4\n"
		"\n"
		"AL_Line:      14 ; Altitude, barometric\n"
		"AL_Units:      0 ; Metres\n"
		"AL_Dec:        0 ; Decimal places\n"
		"AL_Unit_Show:  0 ; Set to 1 for \"1200 m\"\n"
		"AL_X:        250\n"
		"AL_Y:         73\n"
		"AL_Font:       4\n"
		"\n"
		"AL_Line:     100 ; Status line\n"
		"AL_X:        296\n"
		"AL_Y:        232\n"
		"AL_Font:       0\n";

void FS_Config_Init(void)
{
	config.model         = FS_CONFIG_MODEL_AIRBORNE_2G;
	config.rate          = 200;

	config.mode          = FS_CONFIG_MODE_GLIDE_RATIO;
	config.min           = 0;
	config.max           = 300;
	config.limits        = 1;
	config.volume        = 2;

	config.mode_2        = FS_CONFIG_MODE_CHANGE_IN_VALUE_1;
	config.min_2         = 300;
	config.max_2         = 1500;
	config.min_rate      = FS_CONFIG_RATE_ONE_HZ;
	config.max_rate      = 5 * FS_CONFIG_RATE_ONE_HZ;
	config.flatline      = 0;

	config.sp_rate       = 0;
	config.sp_volume     = 0;
	config.num_speech    = 0;

	config.threshold     = 1000;
	config.hThreshold    = 0;

	config.use_sas       = 1;
	config.tz_offset     = 0;

	config.init_mode     = 0;
	*(config.init_filename) = '\0';

	config.alarm_window_above = 0;
	config.alarm_window_below = 0;
	config.dz_elev       = 0;

	config.num_alarms    = 0;

	config.alt_units     = FS_CONFIG_UNITS_FEET;
	config.alt_step      = 0;

	config.num_windows   = 0;

	config.enable_audio   = 1;
	config.enable_logging = 1;
	config.enable_vbat    = 1;
	config.enable_mic     = 1;
	config.enable_imu     = 1;
	config.enable_gnss    = 1;
	config.enable_baro    = 1;
	config.enable_hum     = 1;
	config.enable_mag     = 1;
	config.ble_tx_power   = 25;
	config.enable_raw     = 1;
	config.cold_start     = 0;

	config.baro_odr       = 2;
	config.hum_odr        = 1;
	config.mag_odr        = 0;
	config.accel_odr      = 1;
	config.accel_fs       = 1;
	config.gyro_odr       = 1;
	config.gyro_fs        = 3;

	config.lat            = 0;
	config.lon            = 0;

	/* Unassigned. HUD_Mode 1 with no Ground Reference Point draws the centre bar
	 * and nothing beside it — the window is open, the lane is not known — rather
	 * than centring the lane on the Gulf of Guinea. */
	config.comp_lat       = 0;
	config.comp_lon       = 0;

	config.bearing        = 0;
	config.end_nav        = 0;
	config.max_dist       = 10000;
	config.min_angle      = 5;

	*(config.al_id) = '\0';
	config.al_mode        = 1;
	/* 4 Hz HUD (was 500 = 2 Hz until v0.0.17). Asked for by the owner: at 2 Hz
	 * the numbers visibly lag under canopy. 250 ms is inside the range the CB9
	 * flow-control pacing was written for — the glasses throttle us with a STOP
	 * byte and the frame drain resumes on TX-pool-available (activelook_mode0.c
	 * DrainFrame) — but NOT yet flight-tested: 333 ms is the fastest rate ever
	 * flown on this hardware. If the display processor starts hanging (the known
	 * failure mode of over-driving the command buffer), put AL_Rate back to 333
	 * in CONFIG.TXT before blaming anything else. */
	config.al_rate        = 250;

	/* The general-purpose panel. An untouched card, and every card written
	 * before this key existed, must keep the behaviour it has today. */
	config.hud_mode       = 0;

	/* Start from the built-in layout so an element the file mentions without
	 * coordinates still has somewhere to land, and so a file with no AL_Line
	 * at all leaves the tuned layout untouched. */
	FS_HudLayout_Default(&config.al_layout);
	config.al_layout_valid = 0;

	// IMPORTANT: Navigation disabled by default
	config.enable_nav     = 0;
}

static void FS_Config_WriteHex_32(char *result, const uint32_t *data, uint32_t count)
{
	uint32_t i;

	for (i = 0; i < count; ++i)
	{
		sprintf(result + 8 * i, "%08lx", data[i]);
	}
}

FS_Config_Result_t FS_Config_Read(const char *filename)
{
	char    buffer[100];
	size_t  len;

	char    *name;
	char    *result;
	int32_t val;

	uint8_t flags = 0;

	/* True while the most recent AL_Line was ACCEPTED, i.e. there is an open
	 * HUD element for AL_Units/AL_Dec/AL_Unit_Show/AL_X/AL_Y/AL_Font to refine.
	 * Without this, keys under a rejected AL_Line (out-of-range id, or a 9th
	 * element) would clobber the previous element — and a stray AL_X with no
	 * AL_Line at all would edit the pre-filled default layout from
	 * FS_Config_Init. */
	uint8_t alOpen = 0;

	/* "No such file" is the only outcome that earns a default written over the
	 * top. FR_DISK_ERR, FR_NOT_READY and the rest mean the card could not be
	 * read, which is not the same as there being nothing to read — answering
	 * those with FA_CREATE_ALWAYS would replace a configuration that is
	 * probably fine. */
	{
		const FRESULT fr = f_open(&configFile, filename, FA_READ);

		if (fr == FR_NO_FILE || fr == FR_NO_PATH)
			return FS_CONFIG_ERR;
		if (fr != FR_OK)
			return FS_CONFIG_ERR_IO;
	}

	/* Everything below writes straight into the live `config`. If the read
	 * fails part way, the device would otherwise fly on a mixture of this
	 * file's first half and whatever was there before — so keep a copy and put
	 * it back. Static rather than on the stack: this struct is large enough
	 * that audio_control.c copying it per tick is a deliberate note in that
	 * file, and this runs on the mode-entry path. */
	static FS_Config_Data_t before;
	before = config;

	/* Driven by f_gets, not by f_eof: f_gets returns NULL at the end AND on a
	 * read fault, while f_eof only knows about the end. A card that errors
	 * mid-file leaves the position where it was, so the old loop re-parsed the
	 * same stale buffer for as long as the fault lasted — with the watchdog
	 * the only way out. Same rake as FS_EngoBind_Load; f_error is checked
	 * after the loop for the same reason. */
	while (f_gets(buffer, sizeof(buffer), &configFile) != 0)
	{

		len = strcspn(buffer, ";");
		buffer[len] = '\0';

		name = strtok(buffer, " \r\n\t:");
		if (name == 0) continue ;

		result = strtok(0, " \r\n\t:");
		if (result == 0) continue ;

		val = atol(result);

		#define HANDLE_VALUE(s,w,r,t) \
			if ((t) && !strcmp(name, (s))) { (w) = (r); }

		HANDLE_VALUE("Model",     config.model,        val, val >= 0 && val <= 8);
		HANDLE_VALUE("Rate",      config.rate,         val, val >= 40 && val <= 1000);
		HANDLE_VALUE("Mode",      config.mode,         val, (val >= 0 && val <= 7) || (val == 11));
		HANDLE_VALUE("Min",       config.min,          val, TRUE);
		HANDLE_VALUE("Max",       config.max,          val, TRUE);
		HANDLE_VALUE("Limits",    config.limits,       val, val >= 0 && val <= 3);
		HANDLE_VALUE("Volume",    config.volume,       8 - val, val >= 0 && val <= 8);
		HANDLE_VALUE("Mode_2",    config.mode_2,       val, (val >= 0 && val <= 9) || (val == 11));
		HANDLE_VALUE("Min_Val_2", config.min_2,        val, TRUE);
		HANDLE_VALUE("Max_Val_2", config.max_2,        val, TRUE);
		HANDLE_VALUE("Min_Rate",  config.min_rate,     val * FS_CONFIG_RATE_ONE_HZ / 100, val >= 0);
		HANDLE_VALUE("Max_Rate",  config.max_rate,     val * FS_CONFIG_RATE_ONE_HZ / 100, val >= 0);
		HANDLE_VALUE("Flatline",  config.flatline,     val, val == 0 || val == 1);
		HANDLE_VALUE("Sp_Rate",   config.sp_rate,      val * 1000, val >= 0 && val <= 32);
		HANDLE_VALUE("Sp_Volume", config.sp_volume,    8 - val, val >= 0 && val <= 8);
		HANDLE_VALUE("V_Thresh",  config.threshold,    val, TRUE);
		HANDLE_VALUE("H_Thresh",  config.hThreshold,   val, TRUE);
		HANDLE_VALUE("Use_SAS",   config.use_sas,      val, val == 0 || val == 1);
		HANDLE_VALUE("Window",    config.alarm_window_above, val * 1000, TRUE);
		HANDLE_VALUE("Window",    config.alarm_window_below, val * 1000, TRUE);
		HANDLE_VALUE("Win_Above", config.alarm_window_above, val * 1000, TRUE);
		HANDLE_VALUE("Win_Below", config.alarm_window_below, val * 1000, TRUE);
		HANDLE_VALUE("DZ_Elev",   config.dz_elev,      val * 1000, TRUE);
		HANDLE_VALUE("TZ_Offset", config.tz_offset,    val, TRUE);
		HANDLE_VALUE("Init_Mode", config.init_mode,    val, val >= 0 && val <= 2);
		HANDLE_VALUE("Alt_Units", config.alt_units,    val, val >= 0 && val <= 1);
		HANDLE_VALUE("Alt_Step",  config.alt_step,     val, val >= 0);

		HANDLE_VALUE("Enable_Audio",   config.enable_audio,   val, val == 0 || val == 1);
		HANDLE_VALUE("Enable_Logging", config.enable_logging, val, val == 0 || val == 1);
		HANDLE_VALUE("Enable_Vbat",    config.enable_vbat,    val, val == 0 || val == 1);
		HANDLE_VALUE("Enable_Mic",     config.enable_mic,     val, val == 0 || val == 1);
		HANDLE_VALUE("Enable_Imu",     config.enable_imu,     val, val == 0 || val == 1);
		HANDLE_VALUE("Enable_Gnss",    config.enable_gnss,    val, val == 0 || val == 1);
		HANDLE_VALUE("Enable_Baro",    config.enable_baro,    val, val == 0 || val == 1);
		HANDLE_VALUE("Enable_Hum",     config.enable_hum,     val, val == 0 || val == 1);
		HANDLE_VALUE("Enable_Mag",     config.enable_mag,     val, val == 0 || val == 1);
		HANDLE_VALUE("Ble_Tx_Power",   config.ble_tx_power,   val, val >= 0 && val <= 31);
		HANDLE_VALUE("Enable_Raw",     config.enable_raw,     val, val == 0 || val == 1);
		HANDLE_VALUE("Cold_Start",     config.cold_start,     val, val == 0 || val == 1);

		HANDLE_VALUE("Baro_ODR",  config.baro_odr,     val, val >= 0 && val <= 7);
		HANDLE_VALUE("Hum_ODR",   config.hum_odr,      val, val >= 0 && val <= 3);
		HANDLE_VALUE("Mag_ODR",   config.mag_odr,      val, val >= 0 && val <= 3);
		HANDLE_VALUE("Accel_ODR", config.accel_odr,    val, val >= 0 && val <= 11);
		HANDLE_VALUE("Accel_FS",  config.accel_fs,     val, val >= 0 && val <= 3);
		HANDLE_VALUE("Gyro_ODR",  config.gyro_odr,     val, val >= 0 && val <= 10);
		HANDLE_VALUE("Gyro_FS",   config.gyro_fs,      val, val >= 0 && val <= 3);

		HANDLE_VALUE("Lat",       config.lat,          val, val >= -900000000 && val <= 900000000);
		HANDLE_VALUE("Lon",       config.lon,          val, val >= -1800000000 && val <= 1800000000);

		/* Same limits as Lat/Lon: it is the same kind of number, and a card that
		 * gets one of them wrong should fail the same way. */
		HANDLE_VALUE("Comp_Lat",  config.comp_lat,     val, val >= -900000000 && val <= 900000000);
		HANDLE_VALUE("Comp_Lon",  config.comp_lon,     val, val >= -1800000000 && val <= 1800000000);

		HANDLE_VALUE("Bearing",   config.bearing,      val, val >= 0 && val <= 360);
		HANDLE_VALUE("End_Nav",   config.end_nav,      val * 1000, TRUE);
		HANDLE_VALUE("Max_Dist",  config.max_dist,     val, val >= 0 && val <= 10000);
		HANDLE_VALUE("Min_Angle", config.min_angle,    val, val >= 0 && val <= 360);

		HANDLE_VALUE("AL_Mode",   config.al_mode,      val, val >= 0 && val <= 1);
		HANDLE_VALUE("AL_Rate",   config.al_rate,      val, val >= 100);

		/* Out-of-range values are DROPPED, not folded to the nearest profile:
		 * a card asking for a mode this build has never heard of is one written
		 * by a newer app, and the default panel is the safe reading of it. */
		HANDLE_VALUE("HUD_Mode",  config.hud_mode,     val, val >= 0 && val <= 1);

		#undef HANDLE_VALUE

		if (!strcmp(name, "Init_File"))
		{
			result[8] = '\0';
			strncpy(config.init_filename, result, sizeof(config.init_filename));
		}

		/* Every "subordinate" key below — the ones that modify the entry its
		 * parent key opened — needs that parent to exist. The guards used to
		 * check only the upper bound, so a CONFIG.TXT that names Alarm_Type,
		 * Alarm_File, Win_Bottom, Sp_Units or Sp_Dec before its parent wrote
		 * to element [-1]: a store just below the array, into whatever the
		 * struct happens to keep there. The file comes off a card the user
		 * edits by hand, and Groundrush writes it too, so a reordering bug on
		 * either side reaches this. Sp_Dec only refuses negatives: it means two
		 * different things depending on the speech mode — decimal places in
		 * most of them, but the ALTITUDE STEP in mode 12, where "announce every
		 * 100 m" is a legitimate setting. An earlier version of this guard
		 * capped it at 3 and would have thrown that away. */
		if (!strcmp(name, "Alarm_Elev") && config.num_alarms < FS_CONFIG_MAX_ALARMS)
		{
			if (!(flags & CONFIG_FIRST_ALARM))
			{
				config.num_alarms = 0;
				flags |= CONFIG_FIRST_ALARM;
			}

			++config.num_alarms;
			config.alarms[config.num_alarms - 1].elev = val * 1000;
			config.alarms[config.num_alarms - 1].type = 0;
			config.alarms[config.num_alarms - 1].filename[0] = '\0';
		}
		if (!strcmp(name, "Alarm_Type") && config.num_alarms > 0 &&
				config.num_alarms <= FS_CONFIG_MAX_ALARMS)
		{
			config.alarms[config.num_alarms - 1].type = val;
		}
		if (!strcmp(name, "Alarm_File") && config.num_alarms > 0 &&
				config.num_alarms <= FS_CONFIG_MAX_ALARMS)
		{
			result[8] = '\0';
			strncpy(config.alarms[config.num_alarms - 1].filename, result,
					sizeof(config.alarms[config.num_alarms - 1].filename));
		}

		if (!strcmp(name, "Win_Top") && config.num_windows < FS_CONFIG_MAX_WINDOWS)
		{
			if (!(flags & CONFIG_FIRST_WINDOW))
			{
				config.num_windows = 0;
				flags |= CONFIG_FIRST_WINDOW;
			}

			++config.num_windows;
			config.windows[config.num_windows - 1].top = val * 1000;
		}
		if (!strcmp(name, "Win_Bottom") && config.num_windows > 0 &&
				config.num_windows <= FS_CONFIG_MAX_WINDOWS)
		{
			config.windows[config.num_windows - 1].bottom = val * 1000;
		}

		if (!strcmp(name, "Sp_Mode") && config.num_speech < FS_CONFIG_MAX_SPEECH)
		{
			if (!(flags & CONFIG_FIRST_SPEECH))
			{
				config.num_speech = 0;
				flags |= CONFIG_FIRST_SPEECH;
			}

			++config.num_speech;
			config.speech[config.num_speech - 1].mode = val;
			config.speech[config.num_speech - 1].units = FS_CONFIG_UNITS_MPH;
			config.speech[config.num_speech - 1].decimals = 1;
		}
		if (!strcmp(name, "Sp_Units") && config.num_speech > 0 &&
				config.num_speech <= FS_CONFIG_MAX_SPEECH)
		{
			config.speech[config.num_speech - 1].units = val;
		}
		if (!strcmp(name, "Sp_Dec") && config.num_speech > 0 &&
				config.num_speech <= FS_CONFIG_MAX_SPEECH &&
				val >= 0)
		{
			config.speech[config.num_speech - 1].decimals = val;
		}

		/* HUD elements. One AL_Line opens an element; the AL_Units / AL_Dec /
		 * AL_Unit_Show / AL_X / AL_Y / AL_Font keys that follow refine it, in
		 * any order, and each may be omitted — an unstated coordinate keeps
		 * the built-in position for that slot. Same shape as Alarm_* / Sp_*.
		 *
		 * Values are clamped from the RAW long here, before they touch the
		 * narrow struct fields: assigning first and clamping later would let
		 * e.g. 65836 wrap through int16 to a plausible 300 and sail past the
		 * clamp. The app's parser (hud_layout.dart) mirrors these semantics
		 * key for key — change one side only with the other in hand. */
		#define CFG_CLAMP(v, lo, hi) \
			((v) < (lo) ? (lo) : ((v) > (hi) ? (hi) : (v)))

		if (!strcmp(name, "AL_Line"))
		{
			if (!(flags & CONFIG_FIRST_AL_LINE))
			{
				/* This file authors its own element list: start it empty, and
				 * make the built-in-layout decision from THIS file's keys
				 * rather than inheriting it from a previously read file. */
				config.al_layout.count = 0;
				config.al_layout_valid = 0;
				flags |= CONFIG_FIRST_AL_LINE;
			}

			if (val >= 0 && val <= 255
					&& config.al_layout.count < FS_HUD_MAX_ELEMENTS)
			{
				FS_HudLayout_DefaultSlot(config.al_layout.count, (uint8_t)val,
						&config.al_layout.el[config.al_layout.count]);
				++config.al_layout.count;
				alOpen = 1;
			}
			else
			{
				alOpen = 0;
			}
		}
		if (alOpen && config.al_layout.count > 0)
		{
			FS_HudElement_t *el = &config.al_layout.el[config.al_layout.count - 1];

			/* AL_Units names ONE unit (2..9) or one of the two old unit
			 * SYSTEMS (0, 1). Out of range means metric, NOT the nearest end
			 * of the range: clamping 42 up to `mi` would put a distance on a
			 * speed. A unit that does not apply to the element's quantity —
			 * km on a speed — is not rejected here either; it resolves to
			 * that quantity's metric default when it is drawn
			 * (FS_HudLayout_UnitConv), which is the only place that knows
			 * what quantity the field measures. */
			if (!strcmp(name, "AL_Units"))
				el->units = (val >= 0 && val <= FS_HUD_UNITS_MAX)
						? (uint8_t)val : FS_HUD_UNITS_METRIC;
			if (!strcmp(name, "AL_Dec"))   el->decimals = CFG_CLAMP(val, 0, 3);

			/* Draw the unit after the value ("148 km/h"). Like AL_Units and
			 * AL_Dec — and unlike AL_X/AL_Y/AL_Font — it states no position,
			 * so it does NOT flip al_layout_valid. Consequence, same as for
			 * its two neighbours: a file that names fields and suffixes but
			 * never a coordinate is not a layout at all: Mode0_Init keeps the
			 * built-in one (activelook_mode0.c:432) and this element list —
			 * suffix flags with it — is dropped. Every writer of this file we
			 * know of (the app, and the template below) states coordinates. */
			if (!strcmp(name, "AL_Unit_Show")) el->show_units = (val == 1) ? 1 : 0;

			/* Any of these three means the file is describing positions, so
			 * the built-in layout steps aside in favour of the file's. */
			if (!strcmp(name, "AL_X")) {
				el->x = CFG_CLAMP(val, 0, FS_HUD_PANEL_W - 1);
				config.al_layout_valid = 1;
			}
			if (!strcmp(name, "AL_Y")) {
				el->y = CFG_CLAMP(val, 0, FS_HUD_PANEL_H - 1);
				config.al_layout_valid = 1;
			}
			if (!strcmp(name, "AL_Font")) {
				el->font = CFG_CLAMP(val, 0, FS_HUD_MAX_FONT);
				config.al_layout_valid = 1;
			}
		}

		/* Global HUD offset — the "screen position" adjustment, in viewer
		 * terms (positive X moves the image to the wearer's right, positive Y
		 * moves it up). Applies to every element, layout or built-in. */
		if (!strcmp(name, "AL_Shift_X"))
			config.al_layout.shift_x = CFG_CLAMP(val, -FS_HUD_MAX_SHIFT, FS_HUD_MAX_SHIFT);
		if (!strcmp(name, "AL_Shift_Y"))
			config.al_layout.shift_y = CFG_CLAMP(val, -FS_HUD_MAX_SHIFT, FS_HUD_MAX_SHIFT);

		#undef CFG_CLAMP

		if (!strcmp(name, "AL_ID"))
		{
			result[6] = '\0';
			strncpy(config.al_id, result, sizeof(config.al_id));
		}

		// Compare config Device_ID with actual hardware device ID.
		// If it matches, enable navigation.
		if (!strcmp(name, "Device_ID"))
		{
			// Build 24-hex-digit string from the hardware ID
			const uint32_t *hwId = FS_State_Get()->device_id; // 3 x 32-bit
			char hwIdString[25];  // 24 hex digits + null
			FS_Config_WriteHex_32(hwIdString, hwId, 3);

			if (!strcmp(result, hwIdString))
			{
				config.enable_nav = 1;
			}
		}
	}

	/* The loop above ends on NULL, which is both the end of the file and a read
	 * fault; only the error flag tells them apart. A configuration read half
	 * way is not one to fly with, so say so instead of running on whatever
	 * part of the file happened to arrive. */
	const int readFailed = f_error(&configFile);

	f_close(&configFile);

	/* Deliberately NOT FS_CONFIG_ERR: callers answer that by writing the
	 * default file over the top, which is right when there is no usable
	 * configuration and catastrophic when the card merely stumbled while
	 * reading a perfectly good one. A card that fails once takes the user's
	 * alarm altitudes, units and HUD layout with it. */
	if (readFailed)
	{
		/* Roll back to what was in force before this call: the defaults from
		 * FS_Config_Init on the first read, or the previously loaded file when
		 * an overlay fails. A configuration read half way is not one to fly
		 * with, and now nothing does. */
		config = before;
		return FS_CONFIG_ERR_IO;
	}

	/* A hand-edited file can name any number at all; fold it into range once,
	 * here, so nothing downstream has to. */
	FS_HudLayout_Clamp(&config.al_layout);

	return FS_CONFIG_OK;
}

FS_Config_Result_t FS_Config_Write(const char *filename)
{
	FRESULT res;

	res = f_open(&configFile, filename, FA_WRITE|FA_CREATE_ALWAYS);
	if (res != FR_OK) return FS_CONFIG_ERR;

	f_puts(defaultConfig, &configFile);

	f_close(&configFile);

	return FS_CONFIG_OK;
}

const FS_Config_Data_t *FS_Config_Get(void)
{
	return &config;
}
