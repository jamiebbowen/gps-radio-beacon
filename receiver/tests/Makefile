# Host-side unit tests for receiver firmware logic modules.
#
# Targets:
#   make          - build and run all tests
#   make coverage - re-run with gcov instrumentation and print line, branch
#                   and call coverage for every firmware source under test
#   make sanitize - re-run every test under ASan + UBSan + float-cast-overflow
#   make clean    - remove build artifacts
#
# The stubs/ directory shadows the STM32 HAL headers so hardware-independent
# modules compile unmodified with the host compiler.
#
# Adding a new test: create test_<module>.c, add "<module>" to MODULES below.
# Each test #includes ../firmware/src/<module>.c directly (rather than
# linking it) so tests can reach static functions and state - required to
# exercise defensive guards on internal helpers and reach 100% coverage.

CC       ?= gcc
# No STM32CubeF4 -I: host CI never checks out the vendor tree (1+ GB), so
# FatFs headers are stubbed in stubs/ like the HAL ones - the test surface
# stays identical locally and in CI.
CFLAGS   := -std=gnu11 -Wall -Wextra -g -O0 \
            -Istubs -I. -I../firmware/inc
LDFLAGS  := -lm

BUILD    := build
SRC_DIR  := ../firmware/src

# Every test binary re-reads firmware headers (e.g. inc/<module>.h) via
# stubs/; add the whole header set AND the stub headers as dependencies so
# edits to constants/prototypes/stub internals always rebuild. (This is the
# same stale-binary trap that caused green-here/red-CI events on both
# 2026-09-14 (stale test_compass.c) and 2026-09-15 (stale stub header).)
# Defined BEFORE any rule: prerequisite lists expand immediately, so a
# later definition would leave every explicit rule below with no header
# dependencies at all.
HEADERS := $(wildcard ../firmware/inc/*.h ../firmware/inc/display_modes/*.h test_harness.h stubs/*.h) Makefile

# Modules with a matching test_<module>.c and $(SRC_DIR)/<module>.c
MODULES  := rf_parser gps_parser math_utils rf_receiver button display gps sd_card lora compass i2c_bus jogpad eink sd_diskio
BINS     := $(addprefix $(BUILD)/test_,$(MODULES)) \
            $(BUILD)/test_navigation_mode $(BUILD)/test_preflight_mode \
            $(BUILD)/test_display_modes $(BUILD)/test_fuzz_parsers

# navigation_mode.c lives in display_modes/ and needs the real math_utils
$(BUILD)/test_navigation_mode: test_navigation_mode.c \
        $(SRC_DIR)/display_modes/navigation_mode.c \
        $(SRC_DIR)/math_utils.c $(HEADERS) | $(BUILD)
	$(CC) $(CFLAGS) $(COVFLAGS) -o $@ $< \
	    $(SRC_DIR)/display_modes/navigation_mode.c $(SRC_DIR)/math_utils.c \
	    $(LDFLAGS)

# preflight_mode.c likewise lives in display_modes/ (RF layer stubbed in the
# test file itself; no math_utils dep)
$(BUILD)/test_preflight_mode: test_preflight_mode.c \
        $(SRC_DIR)/display_modes/preflight_mode.c $(HEADERS) | $(BUILD)
	$(CC) $(CFLAGS) $(COVFLAGS) -o $@ $< \
	    $(SRC_DIR)/display_modes/preflight_mode.c $(LDFLAGS)

# The small formatting pages: one test binary links all of them and stubs
# every sensor/RF getter they read (row/col text capture is the oracle).
MODE_SRCS := $(addprefix $(SRC_DIR)/display_modes/, \
    channel_mode.c rf_stats_mode.c gps_mode.c sd_card_mode.c rf_mode.c \
    compass_mode.c i2c_devices.c spi_test_mode.c imu_test_mode.c test_pattern.c)
# -Wno-format: the mode sources print int32_t with %ld, which is correct on
# the ARM ABI (int32_t = long) but warns on the host ABI.
$(BUILD)/test_display_modes: test_display_modes.c $(MODE_SRCS) $(HEADERS) | $(BUILD)
	$(CC) $(CFLAGS) $(COVFLAGS) -Wno-format -o $@ $< $(MODE_SRCS) $(LDFLAGS)

# rf_receiver.c calls into the real parser, so its test links rf_parser.c
# too (the LoRa driver + HAL are faked inside test_rf_receiver.c).
$(BUILD)/test_rf_receiver: test_rf_receiver.c $(SRC_DIR)/rf_receiver.c \
                           $(SRC_DIR)/rf_parser.c $(HEADERS) | $(BUILD)
	$(CC) $(CFLAGS) $(COVFLAGS) -o $@ $< $(SRC_DIR)/rf_parser.c $(LDFLAGS)

# Parser fuzz harness: mutates a seed corpus of real wire packets through
# the RF/ASCII/NMEA parsers under ASan+UBSan. Links the real parser sources
# (no include trick - no statics needed) and deliberately skips COVFLAGS:
# coverage+sanitizer instrumentation don't mix. Soak with FUZZ_ITERS=2000000.
FUZZ_SAN := -fsanitize=address,undefined -fno-sanitize-recover=all
$(BUILD)/test_fuzz_parsers: test_fuzz_parsers.c $(SRC_DIR)/rf_parser.c \
                            $(SRC_DIR)/gps_parser.c $(HEADERS) | $(BUILD)
	$(CC) $(CFLAGS) $(FUZZ_SAN) -o $@ $< \
	    $(SRC_DIR)/rf_parser.c $(SRC_DIR)/gps_parser.c $(LDFLAGS)

# gps.c calls into the real NMEA parser, so its test links gps_parser.c too
$(BUILD)/test_gps: test_gps.c $(SRC_DIR)/gps.c \
                   $(SRC_DIR)/gps_parser.c $(HEADERS) | $(BUILD)
	$(CC) $(CFLAGS) $(COVFLAGS) -o $@ $< $(SRC_DIR)/gps_parser.c $(LDFLAGS)

# compass.c speaks SH-2/SHTP through bno085.c; the I2C wire itself is
# faked inside test_compass.c (HAL master tx/rx), so link the real
# transport driver too. (Explicit rule shadows the generic test_% pattern
# for this module.)
$(BUILD)/test_compass: test_compass.c $(SRC_DIR)/compass.c $(SRC_DIR)/bno085.c \
                       $(HEADERS) | $(BUILD)
	$(CC) $(CFLAGS) $(COVFLAGS) -o $@ $< $(SRC_DIR)/bno085.c $(LDFLAGS)

# sd_card.c runs on the REAL LittleFS over a RAM block device in the test
#
# Note on LFS_NO_ERROR: several suites intentionally mount blank/corrupted
# images (that IS the test - the firmware's format-recovery path). LittleFS
# confirms each such mount with an expected "Corrupted dir pair" diagnostic
# on stderr; silencing it keeps real failures visible in the output.
LFS_DIR  := ../firmware/lib/littlefs
LFS_CFLAGS := -I$(LFS_DIR) -DLFS_NO_MALLOC -DLFS_NO_DEBUG -DLFS_NO_WARN -DLFS_NO_ERROR
$(BUILD)/test_sd_card: test_sd_card.c $(SRC_DIR)/sd_card.c \
                       $(LFS_DIR)/lfs.c $(LFS_DIR)/lfs_util.c $(HEADERS) | $(BUILD)
	$(CC) $(CFLAGS) $(COVFLAGS) $(LFS_CFLAGS) \
	    -o $@ $< $(LFS_DIR)/lfs.c $(LFS_DIR)/lfs_util.c $(LDFLAGS)

.PHONY: all test coverage sanitize clean

# Default goal must be the suite, not the first binary rule below.
.DEFAULT_GOAL := test

all: test

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/test_%: test_%.c $(SRC_DIR)/%.c $(HEADERS) | $(BUILD)
	$(CC) $(CFLAGS) $(COVFLAGS) -o $@ $< $(LDFLAGS)

test: $(BINS)
	@rc=0; for b in $(BINS); do \
	    echo "== $$b =="; ./$$b || rc=1; \
	done; exit $$rc

# Every firmware source paired with the test binary that exercises it
# (test:path-under-src). Each pair prints line, branch and call coverage.
COV_PAIRS := $(foreach m,$(MODULES),$(m):$(m).c) compass:bno085.c \
             navigation_mode:display_modes/navigation_mode.c \
             preflight_mode:display_modes/preflight_mode.c \
             $(addprefix display_modes:display_modes/,$(notdir $(MODE_SRCS)))

coverage: COVFLAGS := --coverage
coverage: clean $(BINS)
	@for b in $(BINS); do ./$$b > /dev/null || exit 1; done
	@for p in $(COV_PAIRS); do t=$${p%%:*}; f=$${p##*:}; \
	    gcov -b -o $(BUILD) $(BUILD)/test_$$t*.gcno 2>/dev/null \
	        | grep -A4 "File '$(SRC_DIR)/$$f'" | grep -v '^--$$'; \
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
