# tests/Makefile
CC = gcc
CFLAGS = -Wall -Wextra -std=c99 -g

TESTS = test_time test_activelook

all: $(TESTS)

test_time: test_time.c
	$(CC) $(CFLAGS) -o test_time test_time.c

test_activelook: test_activelook.c ../FlySight/activelook_proto.c ../FlySight/activelook_proto.h
	$(CC) $(CFLAGS) -o test_activelook test_activelook.c ../FlySight/activelook_proto.c

run: all
	@echo "=== test_time ===";       ./test_time
	@echo "=== test_activelook ==="; ./test_activelook

clean:
	rm -f $(TESTS)

.PHONY: all run clean
