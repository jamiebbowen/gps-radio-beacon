# Host-side unit tests for transmitter firmware logic modules.
#
# Targets:
#   make          - build and run all tests
#   make coverage - re-run with gcov instrumentation and print line coverage
#   make clean    - remove build artifacts
#
# Modules under test compile unmodified on the host (no Arduino deps):
#   ekf.cpp        - 6-state GPS/IMU EKF                (C++, g++)
#   nmea_fields.c  - NMEA field extraction / conversion (C,   gcc)
#
# The shared CHECK/TEST harness lives in receiver/tests/ (firmware-agnostic).

CC       ?= gcc
CXX      ?= g++
CFLAGS   := -std=gnu11 -Wall -Wextra -g -O0 -I../firmware -I../../receiver/tests
CXXFLAGS := -std=c++11 -Wall -Wextra -g -O0 -I../firmware -I../../receiver/tests
LDFLAGS  := -lm

BUILD    := build
BINS     := $(BUILD)/test_ekf $(BUILD)/test_nmea_fields

.PHONY: all test coverage clean

all: test

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/test_ekf: test_ekf.cpp ../firmware/ekf.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) $(COVFLAGS) -o $@ $^ $(LDFLAGS)

$(BUILD)/test_nmea_fields: test_nmea_fields.c ../firmware/nmea_fields.c | $(BUILD)
	$(CC) $(CFLAGS) $(COVFLAGS) -o $@ $^ $(LDFLAGS)

test: $(BINS)
	@rc=0; for b in $(BINS); do \
	    echo "== $$b =="; ./$$b || rc=1; \
	done; exit $$rc

coverage: COVFLAGS := --coverage
coverage: clean $(BINS)
	@for b in $(BINS); do ./$$b > /dev/null || exit 1; done
	@gcov -o $(BUILD) $(BUILD)/test_ekf-ekf.gcno 2>/dev/null \
	    | grep -A1 "File.*ekf.cpp"
	@gcov -o $(BUILD) $(BUILD)/test_nmea_fields-nmea_fields.gcno 2>/dev/null \
	    | grep -A1 "File.*nmea_fields.c"
	@mv -f *.gcov $(BUILD)/ 2>/dev/null || true

clean:
	rm -rf $(BUILD) *.gcov *.gcda *.gcno
