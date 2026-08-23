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
BINS     := $(BUILD)/test_ekf $(BUILD)/test_nmea_fields $(BUILD)/test_beacon \
            $(BUILD)/test_nav $(BUILD)/test_gps $(BUILD)/test_radio \
            $(BUILD)/test_launch_detect $(BUILD)/test_uart

.PHONY: all test coverage clean

all: test

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/test_ekf: test_ekf.cpp ../firmware/ekf.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) $(COVFLAGS) -o $@ $^ $(LDFLAGS)

$(BUILD)/test_nmea_fields: test_nmea_fields.c ../firmware/nmea_fields.c | $(BUILD)
	$(CC) $(CFLAGS) $(COVFLAGS) -o $@ $^ $(LDFLAGS)

# beacon.cpp is #included by the test (static access + Arduino stub);
# stubs/Arduino.h shadows the real core.
$(BUILD)/test_beacon: test_beacon.cpp ../firmware/beacon.cpp stubs/Arduino.h | $(BUILD)
	$(CXX) $(CXXFLAGS) $(COVFLAGS) -Istubs -o $@ $< $(LDFLAGS)

# nav.cpp is #included by the test; the real EKF is linked in.
$(BUILD)/test_nav: test_nav.cpp ../firmware/nav.cpp ../firmware/ekf.cpp \
                   stubs/Arduino.h | $(BUILD)
	$(CXX) $(CXXFLAGS) $(COVFLAGS) -Istubs -o $@ $< ../firmware/ekf.cpp $(LDFLAGS)

# gps.cpp is #included by the test; real nmea_fields.c is linked in.
$(BUILD)/test_gps: test_gps.cpp ../firmware/gps.cpp ../firmware/nmea_fields.c \
                   stubs/Arduino.h | $(BUILD)
	$(CXX) $(CXXFLAGS) $(COVFLAGS) -Istubs -o $@ $< ../firmware/nmea_fields.c $(LDFLAGS)

# radio.cpp / launch_detect.cpp are #included by their tests; RadioLib,
# Wire and the Adafruit BNO08x library are stubbed in stubs/.
$(BUILD)/test_radio: test_radio.cpp ../firmware/radio.cpp stubs/Arduino.h \
                     stubs/RadioLib.h | $(BUILD)
	$(CXX) $(CXXFLAGS) $(COVFLAGS) -Istubs -o $@ $< $(LDFLAGS)

$(BUILD)/test_launch_detect: test_launch_detect.cpp ../firmware/launch_detect.cpp \
                             stubs/Arduino.h stubs/Wire.h stubs/Adafruit_BNO08x.h | $(BUILD)
	$(CXX) $(CXXFLAGS) $(COVFLAGS) -Istubs -o $@ $< $(LDFLAGS)

$(BUILD)/test_uart: test_uart.cpp ../firmware/uart.cpp stubs/Arduino.h | $(BUILD)
	$(CXX) $(CXXFLAGS) $(COVFLAGS) -Istubs -o $@ $< $(LDFLAGS)

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
	@gcov -o $(BUILD) $(BUILD)/test_beacon*.gcno 2>/dev/null \
	    | grep -A1 "File.*firmware/beacon.cpp"
	@gcov -o $(BUILD) $(BUILD)/test_nav-test_nav.gcno 2>/dev/null \
	    | grep -A1 "File.*firmware/nav.cpp"
	@gcov -o $(BUILD) $(BUILD)/test_gps-test_gps.gcno 2>/dev/null \
	    | grep -A1 "File.*firmware/gps.cpp"
	@gcov -o $(BUILD) $(BUILD)/test_radio.gcno 2>/dev/null \
	    | grep -A1 "File.*firmware/radio.cpp"
	@gcov -o $(BUILD) $(BUILD)/test_launch_detect.gcno 2>/dev/null \
	    | grep -A1 "File.*firmware/launch_detect.cpp"
	@gcov -o $(BUILD) $(BUILD)/test_uart.gcno 2>/dev/null \
	    | grep -A1 "File.*firmware/uart.cpp"
	@mv -f *.gcov $(BUILD)/ 2>/dev/null || true

clean:
	rm -rf $(BUILD) *.gcov *.gcda *.gcno
