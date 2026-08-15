# tests/Makefile
CC = gcc
CFLAGS = -Wall -Wextra -std=c99 -g

TESTS = test_time test_activelook test_flight_detect test_engo_bind

all: $(TESTS)

test_time: test_time.c
	$(CC) $(CFLAGS) -o test_time test_time.c

test_activelook: test_activelook.c ../FlySight/activelook_proto.c ../FlySight/activelook_proto.h
	$(CC) $(CFLAGS) -o test_activelook test_activelook.c ../FlySight/activelook_proto.c

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

run: all
	@echo "=== test_time ===";          ./test_time
	@echo "=== test_activelook ===";    ./test_activelook
	@echo "=== test_flight_detect ==="; ./test_flight_detect
	@echo "=== test_engo_bind ===";     ./test_engo_bind

clean:
	rm -f $(TESTS)
	rm -rf build_engo_bind $(addsuffix .dSYM,$(TESTS))

.PHONY: all run clean
