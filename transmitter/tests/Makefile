# Host-side unit tests for transmitter firmware logic modules.
#
# Targets:
#   make          - build and run all tests
#   make coverage - re-run with gcov instrumentation and print line, branch
#                   and call coverage for every firmware module
#   make sanitize - re-run every test under ASan + UBSan + float-cast-overflow
#   make clean    - remove build artifacts
#
# Firmware modules compile unmodified on the host: pure-logic modules link
# directly; hardware-facing ones (#included by their tests) run over the
# Arduino / RadioLib / BNO08x / Wire stand-ins in stubs/.
#
# The shared CHECK/TEST harness lives in receiver/tests/ (firmware-agnostic).

CC       ?= gcc
CXX      ?= g++
# -MMD -MP: header edits (packet_format.h, config.h, stubs) must rebuild
# the test binaries too - a stale binary is a silent false-green, the same
# trap the explicit source prerequisites below guard against.
CFLAGS   := -std=gnu11 -Wall -Wextra -g -O0 -I../firmware -I../../receiver/tests -MMD -MP
CXXFLAGS := -std=c++11 -Wall -Wextra -g -O0 -I../firmware -I../../receiver/tests -MMD -MP
LDFLAGS  := -lm

BUILD    := build
BINS     := $(BUILD)/test_ekf $(BUILD)/test_nmea_fields $(BUILD)/test_beacon \
            $(BUILD)/test_nav $(BUILD)/test_gps $(BUILD)/test_radio \
            $(BUILD)/test_launch_detect $(BUILD)/test_uart \
            $(BUILD)/test_flight_profile $(BUILD)/test_flight_cadence \
            $(BUILD)/test_flight_events $(BUILD)/test_flight_maxima \
            $(BUILD)/test_airlink_roundtrip $(BUILD)/test_mainloop \
            $(BUILD)/test_flight_log $(BUILD)/test_fuzz_nmea \
            $(BUILD)/test_fixed_point

.PHONY: all test coverage sanitize clean

all: test

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/test_ekf: test_ekf.cpp ../firmware/ekf.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) $(COVFLAGS) -o $@ $(filter %.cpp %.c %.ino,$^) $(LDFLAGS)

$(BUILD)/test_nmea_fields: test_nmea_fields.c ../firmware/nmea_fields.c | $(BUILD)
	$(CC) $(CFLAGS) $(COVFLAGS) -o $@ $(filter %.cpp %.c %.ino,$^) $(LDFLAGS)

# NMEA-helper fuzz: mutates real sentences through the field/checksum/coord
# parsers under ASan+UBSan. Skips COVFLAGS (gcov+sanitizer don't mix).
# Soak with FUZZ_ITERS=2000000.
FUZZ_SAN := -fsanitize=address,undefined,float-cast-overflow -fno-sanitize-recover=all
$(BUILD)/test_fuzz_nmea: test_fuzz_nmea.c ../firmware/nmea_fields.c | $(BUILD)
	$(CC) $(CFLAGS) $(FUZZ_SAN) -o $@ $(filter %.cpp %.c %.ino,$^) $(LDFLAGS)

# beacon.cpp is #included by the test (static access + Arduino stub);
# stubs/Arduino.h shadows the real core. flight_maxima.cpp is a real
# dependency of beacon_transmit_maxima().
$(BUILD)/test_beacon: test_beacon.cpp ../firmware/beacon.cpp \
                      ../firmware/flight_maxima.cpp ../firmware/nmea_fields.c \
                      stubs/Arduino.h | $(BUILD)
	$(CXX) $(CXXFLAGS) $(COVFLAGS) -Istubs -o $@ $< ../firmware/flight_maxima.cpp \
	    ../firmware/nmea_fields.c $(LDFLAGS)

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

# Simulated flight: nav + launch_detect + beacon chained over stubbed IMU
# events and scripted GPS fixes. IMPORTANT: tests that #include module
# sources must list them as prerequisites, or edits to the firmware do
# not rebuild the test binary (stale-pass trap that once masked a real
# CI failure).
$(BUILD)/test_flight_profile: test_flight_profile.cpp \
            ../firmware/launch_detect.cpp ../firmware/nav.cpp \
            ../firmware/ekf.cpp ../firmware/beacon.cpp \
            ../firmware/flight_maxima.cpp ../firmware/nmea_fields.c \
            stubs/Arduino.h stubs/Wire.h stubs/Adafruit_BNO08x.h | $(BUILD)
	$(CXX) $(CXXFLAGS) $(COVFLAGS) -Istubs -o $@ $< ../firmware/flight_maxima.cpp \
	    ../firmware/nmea_fields.c $(LDFLAGS)

# Pure state->timing policy; links the real module (Arduino stub headers
# only satisfy mpu_config.h's include chain).
$(BUILD)/test_flight_cadence: test_flight_cadence.cpp ../firmware/flight_cadence.cpp \
                              stubs/Arduino.h | $(BUILD)
	$(CXX) $(CXXFLAGS) $(COVFLAGS) -Istubs -o $@ $(filter %.cpp %.c %.ino,$^) $(LDFLAGS)

# Airborne event + anomaly detector: pure logic module, no Arduino deps.
$(BUILD)/test_flight_events: test_flight_events.cpp ../firmware/flight_events.cpp \
                             stubs/Arduino.h | $(BUILD)
	$(CXX) $(CXXFLAGS) $(COVFLAGS) -Istubs -o $@ $(filter %.cpp %.c %.ino,$^) $(LDFLAGS)

# Saturating fixed-point helpers (header-only).
$(BUILD)/test_fixed_point: test_fixed_point.cpp ../firmware/include/fixed_point.h | $(BUILD)
	$(CXX) $(CXXFLAGS) $(COVFLAGS) -o $@ $(filter %.cpp,$^) $(LDFLAGS)

# Running maxima tracker: pure logic module, no Arduino deps.
$(BUILD)/test_flight_maxima: test_flight_maxima.cpp ../firmware/flight_maxima.cpp \
                             stubs/Arduino.h | $(BUILD)
	$(CXX) $(CXXFLAGS) $(COVFLAGS) -Istubs -o $@ $(filter %.cpp %.c %.ino,$^) $(LDFLAGS)

# Airlink round-trip: TX encoder (beacon.cpp, C++) + RX parser
# (receiver's rf_parser.c, compiled as C) in one binary.
$(BUILD)/rf_parser_rx.o: ../../receiver/firmware/src/rf_parser.c | $(BUILD)
	$(CC) $(CFLAGS) -I../../receiver/tests/stubs \
	    -I../../receiver/firmware/inc -c -o $@ $<

$(BUILD)/test_airlink_roundtrip: test_airlink_roundtrip.cpp \
            ../firmware/beacon.cpp ../firmware/flight_maxima.cpp \
            ../firmware/nmea_fields.c \
            $(BUILD)/rf_parser_rx.o stubs/Arduino.h | $(BUILD)
	$(CXX) $(CXXFLAGS) $(COVFLAGS) -Istubs -I../../receiver/tests/stubs \
	    -I../../receiver/firmware/inc -o $@ $< ../firmware/flight_maxima.cpp \
	    ../firmware/nmea_fields.c $(BUILD)/rf_parser_rx.o $(LDFLAGS)

# Host sim of firmware.ino itself: the sketch runs over stubbed HAL with a
# scripted GPS fix stream and IMU event queue.
$(BUILD)/test_mainloop: test_mainloop.cpp ../firmware/flight_cadence.cpp \
            ../firmware/firmware.ino ../firmware/launch_detect.cpp \
            ../firmware/nav.cpp ../firmware/ekf.cpp ../firmware/beacon.cpp \
            ../firmware/flight_events.cpp ../firmware/flight_maxima.cpp \
            ../firmware/nmea_fields.c \
            stubs/Arduino.h stubs/Wire.h stubs/Adafruit_BNO08x.h | $(BUILD)
	$(CXX) $(CXXFLAGS) $(COVFLAGS) -Istubs -o $@ $< \
	    ../firmware/flight_cadence.cpp ../firmware/flight_events.cpp \
	    ../firmware/flight_maxima.cpp ../firmware/nmea_fields.c $(LDFLAGS)

# On-chip flight recorder: file #included by the test; the NVMCTRL register
# block and the FLOG_BASE flash region are faked in the test file.
$(BUILD)/test_flight_log: test_flight_log.cpp ../firmware/flight_log.cpp \
                          stubs/Arduino.h | $(BUILD)
	$(CXX) $(CXXFLAGS) $(COVFLAGS) -Istubs -o $@ $< $(LDFLAGS)

# Alternate-config compile check (compile-only: no link, no run).
# firmware.ino must build with IMU_FUSION_ENABLED=0 - the flag is
# #ifndef-wrapped in config.h so -D overrides it. Regression: now_ms was
# once declared inside `#if IMU_FUSION_ENABLED` yet used outside it, so
# the fusion-off build broke without anyone noticing.
$(BUILD)/altconfig_nofusion.o: test_mainloop.cpp ../firmware/firmware.ino \
            ../firmware/include/config.h stubs/Arduino.h stubs/Wire.h \
            stubs/Adafruit_BNO08x.h | $(BUILD)
	$(CXX) $(CXXFLAGS) -Istubs -DIMU_FUSION_ENABLED=0 -c -o $@ $<

# Same idea for the other field-used config variant: TESTING_MODE swaps
# the cadence/callsign constants - a build that only ever compiles the
# production branch could let that path rot unnoticed.
$(BUILD)/altconfig_testing.o: test_mainloop.cpp ../firmware/firmware.ino \
            ../firmware/include/config.h stubs/Arduino.h stubs/Wire.h \
            stubs/Adafruit_BNO08x.h | $(BUILD)
	$(CXX) $(CXXFLAGS) -Istubs -DTESTING_MODE=1 -c -o $@ $<

test: $(BINS) $(BUILD)/altconfig_nofusion.o $(BUILD)/altconfig_testing.o
	@rc=0; for b in $(BINS); do \
	    echo "== $$b =="; ./$$b || rc=1; \
	done; exit $$rc

# Every firmware module paired with the test binary that owns it
# (test:source). Each pair prints line, branch and call coverage; modules
# #included by their test report through that test's own .gcno.
COV_PAIRS := ekf:ekf.cpp nmea_fields:nmea_fields.c beacon:beacon.cpp \
             nav:nav.cpp gps:gps.cpp radio:radio.cpp \
             launch_detect:launch_detect.cpp uart:uart.cpp \
             flight_log:flight_log.cpp flight_cadence:flight_cadence.cpp \
             flight_events:flight_events.cpp flight_maxima:flight_maxima.cpp \
             mainloop:firmware.ino

coverage: COVFLAGS := --coverage
coverage: clean $(BINS)
	@for b in $(BINS); do ./$$b > /dev/null || exit 1; done
	@for p in $(COV_PAIRS); do t=$${p%%:*}; f=$${p##*:}; \
	    gcov -b -o $(BUILD) $(BUILD)/test_$$t*.gcno 2>/dev/null \
	        | grep -A4 "File '../firmware/$$f'" | grep -v '^--$$'; \
	done
	@mv -f *.gcov $(BUILD)/ 2>/dev/null || true

# Sanitizer run: every unit test rebuilt under ASan + UBSan, including
# float-cast-overflow (NOT part of -fsanitize=undefined in GCC). Separate
# build dir so it never mixes with plain/coverage objects.
SAN_FLAGS := -fsanitize=address,undefined,float-cast-overflow -fno-sanitize-recover=all
sanitize:
	@$(MAKE) --no-print-directory BUILD=build-san COVFLAGS="$(SAN_FLAGS)" test

clean:
	rm -rf $(BUILD) build-san *.gcov *.gcda *.gcno

# Header dependencies for every target. -MMD alone is not enough here:
# each test compiles several sources in ONE command, and every source
# rewrites the same .d file, so only the last source's headers survive
# (test_beacon.d once listed no beacon.h / packet_format.h at all). This
# rule appends the full header set after each target's own prerequisites,
# so $< and the source filters in the recipes above are unaffected.
TX_DEPS := $(wildcard ../firmware/include/*.h stubs/*.h) ../../receiver/tests/test_harness.h Makefile
RX_DEPS := $(wildcard ../../receiver/firmware/inc/*.h ../../receiver/tests/stubs/*.h)
$(BINS) $(BUILD)/altconfig_nofusion.o $(BUILD)/altconfig_testing.o: $(TX_DEPS)
$(BUILD)/rf_parser_rx.o $(BUILD)/test_airlink_roundtrip: $(RX_DEPS)

# Auto-generated header dependencies (see -MMD -MP above)
-include $(BINS:%=%.d) $(BUILD)/altconfig_nofusion.d $(BUILD)/altconfig_testing.d
