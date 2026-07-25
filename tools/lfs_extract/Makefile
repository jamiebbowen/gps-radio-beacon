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

clean:
	rm -f lfs_extract lfs_format

.PHONY: all clean
