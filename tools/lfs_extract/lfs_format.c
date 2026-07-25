/*
 * lfs_format - Host-side formatter for the receiver's SD card.
 *
 * Writes a fresh LittleFS superblock pair with the exact geometry used by
 * the firmware (see receiver/firmware/src/lfs_sd_bd.c). Handy when re-using
 * a card between flights so you don't need to boot the receiver to trigger
 * the first-mount auto-format path.
 *
 * Safety: this ERASES the card. The program prompts unless -y is passed.
 */

#define _FILE_OFFSET_BITS 64
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "lfs.h"

#define SECTOR_SIZE        512U
#define LFS_BLOCK_SIZE     4096U
#define CACHE_SIZE         512U
#define LOOKAHEAD          128U

static int dev_fd = -1;

static int bd_read(const struct lfs_config *c, lfs_block_t block,
                   lfs_off_t off, void *buffer, lfs_size_t size) {
    (void)c;
    off_t o = (off_t)block * LFS_BLOCK_SIZE + off;
    return (pread(dev_fd, buffer, size, o) == (ssize_t)size) ? 0 : LFS_ERR_IO;
}
static int bd_prog(const struct lfs_config *c, lfs_block_t block,
                   lfs_off_t off, const void *buffer, lfs_size_t size) {
    (void)c;
    off_t o = (off_t)block * LFS_BLOCK_SIZE + off;
    return (pwrite(dev_fd, buffer, size, o) == (ssize_t)size) ? 0 : LFS_ERR_IO;
}
static int bd_erase(const struct lfs_config *c, lfs_block_t block) {
    (void)c; (void)block;
    return 0;
}
static int bd_sync(const struct lfs_config *c) {
    (void)c;
    fsync(dev_fd);
    return 0;
}

int main(int argc, char **argv) {
    int force = 0;
    const char *dev = NULL;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-y")) force = 1;
        else dev = argv[i];
    }
    if (!dev) {
        fprintf(stderr, "usage: %s [-y] <device>\n", argv[0]);
        fprintf(stderr, "  e.g. %s -y /dev/sdb\n", argv[0]);
        return 1;
    }

    dev_fd = open(dev, O_RDWR);
    if (dev_fd < 0) {
        fprintf(stderr, "open %s: %s\n", dev, strerror(errno));
        return 1;
    }

    off_t dev_bytes = lseek(dev_fd, 0, SEEK_END);
    lseek(dev_fd, 0, SEEK_SET);
    if (dev_bytes <= 0) {
        fprintf(stderr, "cannot size %s\n", dev);
        return 1;
    }
    uint32_t block_count = (uint32_t)(dev_bytes / LFS_BLOCK_SIZE);

    printf("Device : %s\n", dev);
    printf("Size   : %.2f GiB (%u LFS blocks)\n",
           (double)dev_bytes / (1024.0 * 1024.0 * 1024.0), block_count);

    if (!force) {
        printf("\nThis will ERASE all data on %s. Type YES to proceed: ", dev);
        fflush(stdout);
        char ans[8] = {0};
        if (!fgets(ans, sizeof(ans), stdin) || strncmp(ans, "YES", 3) != 0) {
            printf("Aborted.\n");
            return 1;
        }
    }

    static uint8_t read_buf[CACHE_SIZE];
    static uint8_t prog_buf[CACHE_SIZE];
    static uint8_t lookahead_buf[LOOKAHEAD];
    struct lfs_config cfg = {
        .read = bd_read, .prog = bd_prog, .erase = bd_erase, .sync = bd_sync,
        .read_size = SECTOR_SIZE,  .prog_size = SECTOR_SIZE,
        .block_size = LFS_BLOCK_SIZE, .block_count = block_count,
        .block_cycles = 500,
        .cache_size = CACHE_SIZE, .lookahead_size = LOOKAHEAD,
        .read_buffer = read_buf, .prog_buffer = prog_buf,
        .lookahead_buffer = lookahead_buf,
    };

    lfs_t lfs;
    int err = lfs_format(&lfs, &cfg);
    if (err) {
        fprintf(stderr, "lfs_format failed: %d\n", err);
        return 1;
    }
    fsync(dev_fd);
    close(dev_fd);
    printf("Done. Card is now a blank LittleFS image.\n");
    return 0;
}
