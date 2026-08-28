# tests/Makefile
CC = gcc
CFLAGS = -Wall -Wextra -std=c99 -g

# The Mac-side BLE tools' host tests: the flasher (Tools/flysight_ble.py) and
# the long-lived app around it (Tools/flysight_app.py). Pure logic only — CRS
# framing, the Go-Back-N window, the flash gates, the app's state machine and
# its JSON API — so they need no radio and no bleak, and any python3 will do.
PYTHON = python3

TESTS = test_time test_activelook test_flight_detect test_engo_bind test_hud_layout \
        test_ble_diag test_activelook_adv test_activelook_fsm test_crs test_nav_arrow \
        test_comp_corridor

all: $(TESTS)

test_time: test_time.c
	$(CC) $(CFLAGS) -o test_time test_time.c

# hud_layout.c comes along for FS_HudLayout_UnitConv: the unit table moved out
# of activelook_mode0.c — which no host test can link, it pulls in the HAL —
# into the pure module in v0.0.18, and this is where it gets checked.
test_activelook: test_activelook.c ../FlySight/activelook_proto.c ../FlySight/activelook_proto.h ../FlySight/hud_layout.c ../FlySight/hud_layout.h
	$(CC) $(CFLAGS) -o test_activelook test_activelook.c ../FlySight/activelook_proto.c ../FlySight/hud_layout.c

test_flight_detect: test_flight_detect.c ../FlySight/flight_detect.c ../FlySight/flight_detect.h
	$(CC) $(CFLAGS) -I../FlySight -o test_flight_detect test_flight_detect.c ../FlySight/flight_detect.c

# engo_bind.c says #include "log.h"/"ff.h", and the quoted-include search looks
# in the includer's own directory first — so compiling it in place would pull
# the real FlySight/log.h instead of the mocks. Copy source + mocks into one
# directory and compile there.
test_engo_bind: test_engo_bind.c ../FlySight/engo_bind.c ../FlySight/engo_bind.h engo_mock/ff.h engo_mock/log.h
	rm -rf build_engo_bind && mkdir build_engo_bind
	cp test_engo_bind.c ../FlySight/engo_bind.c ../FlySight/engo_bind.h engo_mock/ff.h engo_mock/log.h build_engo_bind/
	$(CC) $(CFLAGS) -o test_engo_bind build_engo_bind/test_engo_bind.c build_engo_bind/engo_bind.c

test_hud_layout: test_hud_layout.c ../FlySight/hud_layout.c ../FlySight/hud_layout.h
	$(CC) $(CFLAGS) -I../FlySight -o test_hud_layout test_hud_layout.c ../FlySight/hud_layout.c

# The pure half of the BLE reconnect diagnostic — ring buffer and line
# formatter, no HAL and no FatFs, so it links straight from the host. The other
# half (ble_diag_sink.c) is a clock and a f_write and is not testable here.
# The HUD's navigation arrow. Pure geometry and one sinf/cosf, so -lm.
test_nav_arrow: test_nav_arrow.c ../FlySight/nav_arrow.c ../FlySight/nav_arrow.h
	$(CC) $(CFLAGS) -I../FlySight -o test_nav_arrow test_nav_arrow.c ../FlySight/nav_arrow.c -lm

# The competition lane indicator. Takes gnss.h for FS_GNSS_Data_t — a plain
# struct with no HAL behind it, the same way test_flight_detect does — plus cos
# and sqrt, so -lm.
test_comp_corridor: test_comp_corridor.c ../FlySight/comp_corridor.c ../FlySight/comp_corridor.h
	$(CC) $(CFLAGS) -I../FlySight -o test_comp_corridor test_comp_corridor.c ../FlySight/comp_corridor.c -lm

test_ble_diag: test_ble_diag.c ../FlySight/ble_diag.c ../FlySight/ble_diag.h
	$(CC) $(CFLAGS) -DFS_BLE_DIAG=1 -I../FlySight -o test_ble_diag test_ble_diag.c ../FlySight/ble_diag.c

test_activelook_adv: test_activelook_adv.c ../FlySight/activelook_adv.c ../FlySight/activelook_adv.h
	$(CC) $(CFLAGS) -o test_activelook_adv test_activelook_adv.c ../FlySight/activelook_adv.c

# The ENGO HUD state machine. Same copy-source-next-to-mocks trick as
# test_engo_bind: activelook.c says #include "main.h"/"app_common.h"/"config.h"
# etc., and quoted includes look in the includer's own directory first, so the
# real CubeMX/HAL headers would win if it were compiled in place. The mocks are
# only the sequencer, the timer server and the ActiveLook client — which is
# exactly what drives the FSM, so the FSM under test is the real one.
AL_FSM_MOCKS = al_mock/main.h al_mock/app_common.h al_mock/stm32_seq.h \
               al_mock/dbg_trace.h al_mock/log.h al_mock/config.h \
               al_mock/activelook_client.h al_mock/ble_diag.h al_mock/gnss.h

test_activelook_fsm: test_activelook_fsm.c ../FlySight/activelook.c ../FlySight/activelook.h \
                     ../FlySight/activelook_mode0.h ../FlySight/activelook_proto.c \
                     ../FlySight/activelook_proto.h $(AL_FSM_MOCKS)
	rm -rf build_al_fsm && mkdir build_al_fsm
	cp test_activelook_fsm.c ../FlySight/activelook.c ../FlySight/activelook.h \
	   ../FlySight/activelook_mode0.h ../FlySight/activelook_proto.c \
	   ../FlySight/activelook_proto.h $(AL_FSM_MOCKS) build_al_fsm/
	$(CC) $(CFLAGS) -o test_activelook_fsm build_al_fsm/test_activelook_fsm.c \
	   build_al_fsm/activelook.c build_al_fsm/activelook_proto.c

# Compile the real CRS state machine beside host-only definitions of its
# FatFs, BLE queue, scheduler, timer and resource-manager dependencies.
test_crs: test_crs.c crs_mock/crs_mock.h ../FlySight/crs.c ../FlySight/crs.h
	rm -rf build_crs && mkdir build_crs
	cp test_crs.c ../FlySight/crs.c ../FlySight/crs.h crs_mock/*.h build_crs/
	$(CC) $(CFLAGS) -Ibuild_crs -o test_crs build_crs/test_crs.c build_crs/crs.c

run: all
	@echo "=== test_time ===";          ./test_time
	@echo "=== test_activelook ===";    ./test_activelook
	@echo "=== test_flight_detect ==="; ./test_flight_detect
	@echo "=== test_engo_bind ===";     ./test_engo_bind
	@echo "=== test_hud_layout ===";   ./test_hud_layout
	@echo "=== test_ble_diag ===";     ./test_ble_diag
	@echo "=== test_nav_arrow ===";    ./test_nav_arrow
	@echo "=== test_comp_corridor ==="; ./test_comp_corridor
	@echo "=== test_activelook_adv ==="; ./test_activelook_adv
	@echo "=== test_activelook_fsm ==="; ./test_activelook_fsm
	@echo "=== test_crs ===";           ./test_crs
	@echo "=== test_flysight_ble ==="; $(PYTHON) test_flysight_ble.py
	@echo "=== test_flysight_app ==="; $(PYTHON) test_flysight_app.py

clean:
	rm -f $(TESTS)
	rm -rf build_engo_bind build_al_fsm build_crs $(addsuffix .dSYM,$(TESTS)) __pycache__

.PHONY: all run clean
