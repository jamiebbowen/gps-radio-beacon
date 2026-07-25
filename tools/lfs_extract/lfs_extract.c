/*
 * lfs_extract - Read a LittleFS image from a raw block device (SD card) and
 *               copy every file out to a host filesystem directory.
 *
 * Build:
 *     make -C tools/lfs_extract
 *
 * Usage (example):
 *     sudo ./lfs_extract /dev/sdX ./flight_data
 *
 * Notes:
 *   - Requires raw block-device read permission. On Linux this typically means
 *     sudo, unless the user is in the "disk" group.
 *   - The on-card geometry MUST match the firmware: 512 B sectors, 4 KiB
 *     LittleFS blocks (8 sectors per block). These are wired into lfs_sd_bd.c
 *     on the device side; any change there must be mirrored here.
 *   - The tool also prints a short summary at the end: file count, total
 *     bytes copied, and the percent of card space in use.
 */

#define _FILE_OFFSET_BITS 64
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "lfs.h"

/* Must match firmware/src/lfs_sd_bd.c */
#define SECTOR_SIZE        512U
#define LFS_BLOCK_SIZE     4096U
#define SECTORS_PER_BLOCK  (LFS_BLOCK_SIZE / SECTOR_SIZE)
#define CACHE_SIZE         512U
#define LOOKAHEAD          128U

static int dev_fd = -1;

/* -------------------------------------------------------------------------
 * Block-device callbacks (read-only extraction, but LittleFS mount insists
 * on prog/erase/sync pointers; we stub the write paths).
 * ------------------------------------------------------------------------- */

static int bd_read(const struct lfs_config *c, lfs_block_t block,
                   lfs_off_t off, void *buffer, lfs_size_t size) {
    (void)c;
    off_t byte_off = (off_t)block * LFS_BLOCK_SIZE + off;
    ssize_t n = pread(dev_fd, buffer, size, byte_off);
    if (n != (ssize_t)size) {
        fprintf(stderr, "pread(%zu @ %lld) = %zd (%s)\n",
                (size_t)size, (long long)byte_off, n, strerror(errno));
        return LFS_ERR_IO;
    }
    return 0;
}

static int bd_prog(const struct lfs_config *c, lfs_block_t block,
                   lfs_off_t off, const void *buffer, lfs_size_t size) {
    (void)c; (void)block; (void)off; (void)buffer; (void)size;
    fprintf(stderr, "refusing to prog (read-only extraction)\n");
    return LFS_ERR_IO;
}

static int bd_erase(const struct lfs_config *c, lfs_block_t block) {
    (void)c; (void)block;
    return 0;
}

static int bd_sync(const struct lfs_config *c) {
    (void)c;
    return 0;
}

/* -------------------------------------------------------------------------
 * File-system walk + copy
 * ------------------------------------------------------------------------- */

static int ensure_dir(const char *path) {
    struct stat st;
    if (stat(path, &st) == 0) {
        return S_ISDIR(st.st_mode) ? 0 : -1;
    }
    return mkdir(path, 0755);
}

static uint64_t total_bytes_copied;
static unsigned total_files;

static int copy_file(lfs_t *lfs, const char *lfs_path, const char *host_path) {
    lfs_file_t lfile;
    uint8_t fbuf[CACHE_SIZE];
    struct lfs_file_config fcfg = { .buffer = fbuf, .attrs = NULL, .attr_count = 0 };

    int err = lfs_file_opencfg(lfs, &lfile, lfs_path, LFS_O_RDONLY, &fcfg);
    if (err) {
        fprintf(stderr, "  ! open %s -> %d\n", lfs_path, err);
        return err;
    }

    FILE *out = fopen(host_path, "wb");
    if (!out) {
        fprintf(stderr, "  ! fopen %s: %s\n", host_path, strerror(errno));
        lfs_file_close(lfs, &lfile);
        return -1;
    }

    uint8_t buf[4096];
    lfs_ssize_t r;
    uint64_t bytes = 0;
    while ((r = lfs_file_read(lfs, &lfile, buf, sizeof(buf))) > 0) {
        if (fwrite(buf, 1, r, out) != (size_t)r) {
            fprintf(stderr, "  ! fwrite %s: %s\n", host_path, strerror(errno));
            fclose(out);
            lfs_file_close(lfs, &lfile);
            return -1;
        }
        bytes += r;
    }
    fclose(out);
    lfs_file_close(lfs, &lfile);

    printf("  %-24s  %8llu bytes\n", lfs_path, (unsigned long long)bytes);
    total_bytes_copied += bytes;
    total_files++;
    return 0;
}

static int walk_and_copy(lfs_t *lfs, const char *lfs_dir, const char *host_dir) {
    lfs_dir_t d;
    int err = lfs_dir_open(lfs, &d, lfs_dir);
    if (err) {
        fprintf(stderr, "lfs_dir_open(%s) -> %d\n", lfs_dir, err);
        return err;
    }

    struct lfs_info info;
    while ((err = lfs_dir_read(lfs, &d, &info)) > 0) {
        if (info.name[0] == '.') continue;

        char lfs_path[LFS_NAME_MAX + 8];
        char host_path[1024];
        snprintf(lfs_path, sizeof(lfs_path), "%s%s%s",
                 lfs_dir,
                 (lfs_dir[strlen(lfs_dir) - 1] == '/') ? "" : "/",
                 info.name);
        snprintf(host_path, sizeof(host_path), "%s/%s", host_dir, info.name);

        if (info.type == LFS_TYPE_DIR) {
            if (ensure_dir(host_path) != 0) {
                fprintf(stderr, "mkdir %s failed: %s\n", host_path, strerror(errno));
                continue;
            }
            walk_and_copy(lfs, lfs_path, host_path);
        } else if (info.type == LFS_TYPE_REG) {
            copy_file(lfs, lfs_path, host_path);
        }
    }
    lfs_dir_close(lfs, &d);
    return (err < 0) ? err : 0;
}

/* -------------------------------------------------------------------------
 * main
 * ------------------------------------------------------------------------- */

static off_t device_size(int fd) {
    off_t end = lseek(fd, 0, SEEK_END);
    lseek(fd, 0, SEEK_SET);
    return end;
}

int main(int argc, char **argv) {
    if (argc != 3) {
        fprintf(stderr, "usage: %s <device> <output-dir>\n", argv[0]);
        fprintf(stderr, "  e.g. %s /dev/sdb ./flight_data\n", argv[0]);
        return 1;
    }
    const char *dev = argv[1];
    const char *out = argv[2];

    dev_fd = open(dev, O_RDONLY);
    if (dev_fd < 0) {
        fprintf(stderr, "open %s: %s\n", dev, strerror(errno));
        return 1;
    }

    off_t dev_bytes = device_size(dev_fd);
    if (dev_bytes <= 0) {
        fprintf(stderr, "cannot determine size of %s\n", dev);
        return 1;
    }
    uint32_t block_count = (uint32_t)(dev_bytes / LFS_BLOCK_SIZE);
    printf("Device: %s\n", dev);
    printf("Size:   %.2f GiB (%u LFS blocks)\n",
           (double)dev_bytes / (1024.0 * 1024.0 * 1024.0), block_count);

    static uint8_t read_buf[CACHE_SIZE];
    static uint8_t prog_buf[CACHE_SIZE];
    static uint8_t lookahead_buf[LOOKAHEAD];

    struct lfs_config cfg = {
        .context = NULL,
        .read = bd_read, .prog = bd_prog, .erase = bd_erase, .sync = bd_sync,
        .read_size = SECTOR_SIZE,
        .prog_size = SECTOR_SIZE,
        .block_size = LFS_BLOCK_SIZE,
        .block_count = block_count,
        .block_cycles = 500,
        .cache_size = CACHE_SIZE,
        .lookahead_size = LOOKAHEAD,
        .read_buffer = read_buf,
        .prog_buffer = prog_buf,
        .lookahead_buffer = lookahead_buf,
    };

    lfs_t lfs;
    int err = lfs_mount(&lfs, &cfg);
    if (err) {
        fprintf(stderr, "lfs_mount failed: %d\n", err);
        fprintf(stderr, "  (wrong device? or not a LittleFS image?)\n");
        close(dev_fd);
        return 1;
    }

    if (ensure_dir(out) != 0) {
        fprintf(stderr, "cannot create %s: %s\n", out, strerror(errno));
        lfs_unmount(&lfs);
        close(dev_fd);
        return 1;
    }

    printf("\nExtracting:\n");
    walk_and_copy(&lfs, "/", out);

    lfs_ssize_t used_blocks = lfs_fs_size(&lfs);
    printf("\nSummary:\n");
    printf("  files copied : %u\n", total_files);
    printf("  bytes copied : %llu\n", (unsigned long long)total_bytes_copied);
    if (used_blocks >= 0) {
        printf("  card used    : %lld / %u blocks (%.2f %%)\n",
               (long long)used_blocks, block_count,
               100.0 * (double)used_blocks / (double)block_count);
    }
    printf("  output dir   : %s\n", out);

    lfs_unmount(&lfs);
    close(dev_fd);
    return 0;
}
