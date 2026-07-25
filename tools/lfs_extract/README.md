# `lfs_extract` — pull flight data off the LittleFS SD card

The receiver writes `L####.TXT`, `BEACON.TXT`, and `COMPCAL.BIN` to the SD
card using **LittleFS**, not FAT. That choice gives us real power-fail
safety (the card can never be corrupted by a mid-write reset), but it means
Linux / macOS / Windows can no longer mount the card directly.

`lfs_extract` solves the read-back side: it opens the card as a raw block
device, mounts the LittleFS image in user-space, and copies every file out
to a normal host directory.

## Build

```sh
make -C tools/lfs_extract
```

Produces a single binary, `tools/lfs_extract/lfs_extract`.

## Use

```sh
# Identify the SD card device with `lsblk` or `dmesg | tail` first.
sudo tools/lfs_extract/lfs_extract /dev/sdb ./flight_data_2026_04_20
```

Root (or `disk` group membership) is required because the tool reads the
raw block device, not a mounted filesystem.

Typical output:

```
Device: /dev/sdb
Size:   29.72 GiB (7789184 LFS blocks)

Extracting:
  /BEACON.TXT                   30 bytes
  /COMPCAL.BIN                  25 bytes
  /L0001.TXT                14623 bytes
  /L0002.TXT               912442 bytes

Summary:
  files copied : 4
  bytes copied : 927120
  card used    : 342 / 7789184 blocks (0.00 %)
  output dir   : ./flight_data_2026_04_20
```

## Related top-level targets

From the repo root:

```sh
# Build this tool:
make tools

# One-shot extract:
make extract DEV=/dev/sdb OUT=./flight_data

# Wipe and re-format the card to LittleFS on the host (skips the firmware
# auto-format path, useful when re-using a card between flights):
make format-card DEV=/dev/sdb
```

## Constraint: geometry must match the firmware

The on-card layout is defined in `receiver/firmware/src/lfs_sd_bd.c`:

| Parameter   | Value  |
|-------------|--------|
| Sector size | 512 B  |
| Block size  | 4096 B |
| Cache size  | 512 B  |
| Lookahead   | 128 B  |

`lfs_extract.c` mirrors these constants. If you ever tune them on the
firmware side, update this file to match (and reformat existing cards).
