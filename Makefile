# Top-level convenience Makefile for the GPS radio beacon project.
#
# Useful targets:
#   make receiver          - build receiver firmware
#   make transmitter       - build transmitter firmware
#   make tools             - build host-side lfs_extract + lfs_format
#   make test              - run host-side unit tests (receiver + transmitter)
#   make extract DEV=/dev/sdX [OUT=./flight_data]
#                          - copy every file off a LittleFS-formatted SD card
#                            into OUT/. Requires root (raw block-device read).
#   make format-card DEV=/dev/sdX
#                          - wipe the SD card and write a fresh LittleFS
#                            superblock with the firmware's geometry. Asks
#                            for YES confirmation; add FORCE=1 to skip.
#   make clean             - clean receiver, transmitter, and tools
#
# The SD card on this project is formatted as LittleFS (not FAT), which is
# what gives us crash-safe writes during rocket flights. The trade-off is
# that the card is not natively mountable by a desktop OS; use the tools
# above to read/format it.

RECEIVER_DIR     := receiver/firmware
TRANSMITTER_DIR  := transmitter/firmware
TOOLS_DIR        := tools/lfs_extract

OUT ?= ./flight_data

.PHONY: all receiver transmitter tools test tools-test tools-test-logs extract format-card clean help

all: receiver tools

help:
	@sed -n 's/^# //p' Makefile | head -n 20

receiver:
	$(MAKE) -C $(RECEIVER_DIR)

transmitter:
	$(MAKE) -C $(TRANSMITTER_DIR) || true  # transmitter uses Arduino build separately

tools:
	$(MAKE) -C $(TOOLS_DIR)

test: tools-test
	$(MAKE) -C receiver/tests test
	$(MAKE) -C transmitter/tests test

# Unit tests for the host-side analysis tools run on synthetic logs and
# work on any clone. The log-format regression smoke test below them
# needs real flight logs (flight_data/ is gitignored — populate it with
# `make extract DEV=/dev/sdX` after a flight); it fails fast if
# sd_card.c's log columns and the host tools ever drift apart again (an
# earlier format change silently broke analyze_flight.py - every row was
# skipped), so it runs whenever flight_data has logs, and skips
# otherwise.
tools-test:
	@echo "--- tools-test: unit tests ---"
	python3 -m unittest discover -s receiver/tools/tests
	@if ls flight_data/L*.TXT >/dev/null 2>&1; then \
		$(MAKE) -s tools-test-logs; \
	else \
		echo "--- tools-test: flight_data/ has no logs, skipping log-format smoke test ---"; \
	fi

tools-test-logs:
	@echo "--- tools-test: log-format compatibility ---"
	@python3 receiver/tools/tests/check_flight_data.py

# ---------------------------------------------------------------------------
# SD card I/O helpers
# ---------------------------------------------------------------------------

extract: tools
	@if [ -z "$(DEV)" ]; then \
		echo "Usage: make extract DEV=/dev/sdX [OUT=./flight_data]"; exit 1; \
	fi
	sudo $(TOOLS_DIR)/lfs_extract $(DEV) $(OUT)

format-card: tools
	@if [ -z "$(DEV)" ]; then \
		echo "Usage: make format-card DEV=/dev/sdX [FORCE=1]"; exit 1; \
	fi
	@if [ "$(FORCE)" = "1" ]; then \
		sudo $(TOOLS_DIR)/lfs_format -y $(DEV); \
	else \
		sudo $(TOOLS_DIR)/lfs_format $(DEV); \
	fi

clean:
	$(MAKE) -C $(RECEIVER_DIR) clean
	$(MAKE) -C $(TOOLS_DIR) clean
	$(MAKE) -C receiver/tests clean
	$(MAKE) -C transmitter/tests clean
