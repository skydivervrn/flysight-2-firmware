# tests/Makefile
CC = gcc
CFLAGS = -Wall -Wextra -std=c99 -g

TESTS = test_time test_activelook test_flight_detect test_engo_bind test_hud_layout

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

run: all
	@echo "=== test_time ===";          ./test_time
	@echo "=== test_activelook ===";    ./test_activelook
	@echo "=== test_flight_detect ==="; ./test_flight_detect
	@echo "=== test_engo_bind ===";     ./test_engo_bind
	@echo "=== test_hud_layout ===";   ./test_hud_layout

clean:
	rm -f $(TESTS)
	rm -rf build_engo_bind $(addsuffix .dSYM,$(TESTS))

.PHONY: all run clean
