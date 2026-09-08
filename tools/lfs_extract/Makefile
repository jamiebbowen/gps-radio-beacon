# Host-side LittleFS extractor. Builds against the same vendored littlefs
# sources as the firmware so on-disk format is always in lock-step.

CC      ?= cc
CFLAGS  ?= -O2 -g -Wall -Wextra -Wno-unused-parameter
LFS_DIR := ../../receiver/firmware/lib/littlefs

# LittleFS is compiled here with malloc enabled for simplicity; the host has
# plenty of heap. LFS_NO_ASSERT keeps the output quieter.
CPPFLAGS := -I$(LFS_DIR) -DLFS_NO_ASSERT -DLFS_NO_DEBUG -DLFS_NO_WARN

LFS_CORE := $(LFS_DIR)/lfs.c $(LFS_DIR)/lfs_util.c

all: lfs_extract lfs_format

lfs_extract: lfs_extract.c $(LFS_CORE)
	$(CC) $(CFLAGS) $(CPPFLAGS) $^ -o $@

lfs_format: lfs_format.c $(LFS_CORE)
	$(CC) $(CFLAGS) $(CPPFLAGS) $^ -o $@

# Host test: builds synthetic LFS images (clean, mid-file CRC corruption,
# superblock corruption, claimed-overlap pair) and drives run_extract over
# them. This tool is the ONLY way flight data leaves the SD card - "works
# on the happy path" is not enough.
test_lfs_extract: test_lfs_extract.c lfs_extract.c $(LFS_CORE)
	$(CC) $(CFLAGS) $(CPPFLAGS) -DLFS_EXTRACT_NO_MAIN $^ -o $@

test: lfs_extract lfs_format test_lfs_extract
	./test_lfs_extract

clean:
	rm -f lfs_extract lfs_format test_lfs_extract

.PHONY: all clean test
