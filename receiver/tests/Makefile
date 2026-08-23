# Host-side unit tests for receiver firmware logic modules.
#
# Targets:
#   make          - build and run all tests
#   make coverage - re-run with gcov instrumentation and print a line-coverage
#                   summary for every module under test
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
CFLAGS   := -std=gnu11 -Wall -Wextra -g -O0 \
            -Istubs -I. -I../firmware/inc
LDFLAGS  := -lm

BUILD    := build
SRC_DIR  := ../firmware/src

# Modules with a matching test_<module>.c and $(SRC_DIR)/<module>.c
MODULES  := rf_parser gps_parser math_utils rf_receiver
BINS     := $(addprefix $(BUILD)/test_,$(MODULES))

# rf_receiver.c calls into the real parser, so its test links rf_parser.c
# too (the LoRa driver + HAL are faked inside test_rf_receiver.c).
$(BUILD)/test_rf_receiver: test_rf_receiver.c $(SRC_DIR)/rf_receiver.c \
                           $(SRC_DIR)/rf_parser.c test_harness.h | $(BUILD)
	$(CC) $(CFLAGS) $(COVFLAGS) -o $@ $< $(SRC_DIR)/rf_parser.c $(LDFLAGS)

.PHONY: all test coverage clean

all: test

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/test_%: test_%.c $(SRC_DIR)/%.c test_harness.h | $(BUILD)
	$(CC) $(CFLAGS) $(COVFLAGS) -o $@ $< $(LDFLAGS)

test: $(BINS)
	@rc=0; for b in $(BINS); do \
	    echo "== $$b =="; ./$$b || rc=1; \
	done; exit $$rc

coverage: COVFLAGS := --coverage
coverage: clean $(BINS)
	@for b in $(BINS); do ./$$b > /dev/null || exit 1; done
	@for m in $(MODULES); do \
	    gcov -o $(BUILD) $(BUILD)/test_$$m*.gcno 2>/dev/null \
	        | grep -A1 "File.*src/$$m.c"; \
	done
	@mv -f *.gcov $(BUILD)/ 2>/dev/null || true

clean:
	rm -rf $(BUILD) *.gcov *.gcda *.gcno
