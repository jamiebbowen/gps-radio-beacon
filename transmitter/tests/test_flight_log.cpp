/**
 * @file test_flight_log.cpp
 * @brief Host-side unit tests for the transmitter on-chip flight recorder.
 *
 * Redirects the log region at a host buffer (FLOG_BASE override, guarded
 * in flight_log.cpp) and fakes the SAMD51 NVMCTRL register block so the
 * production erase/page-commit path runs unmodified:
 *   - arm: full-region block erase, erase address histogram, arm/rearm policy
 *   - append: every record is programmed at once as two 16-byte quad words
 *     (tail first, magic last: power loss costs at most the record in flight)
 *   - record layout: every payload field must survive the write stride
 *     (regression: the union's struct used to be 40 B wide while only 16 B
 *     per record was copied, so az/gyro/peak and the entire GPS payload
 *     (lat/lon/alt/sats/fix) never reached flash)
 *   - capacity: the region never wraps past its end
 *   - dump: CSV emission of flushed records, magic-gated skip of erased space
 *
 * Build & run:  make -C transmitter/tests
 */

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stddef.h>

#include <Arduino.h>
#include "test_harness.h"
#include "include/flight_log.h"

/* ------------------------------------------------------------------ */
/* Host flash backing store: 256 KB, word-aligned like real NVM. The   */
/* firmware addresses flash as uint32_t, so the buffer must live in    */
/* the low 2 GB of the 64-bit address space (MAP_32BIT).               */
/* ------------------------------------------------------------------ */

#include <sys/mman.h>

#define HOST_FLOG_BYTES (256u * 1024u)

/* 8 KB-aligned like FLOG_BASE (0x40000) on the chip: an erase block is
 * the 8 KB block containing ADDR, so a merely page-aligned buffer would
 * have its first erase land partly below it. */
static uint8_t *flash_alloc(void)
{
    uint8_t *raw = (uint8_t *)mmap(nullptr, HOST_FLOG_BYTES + 8192u, PROT_READ | PROT_WRITE,
                                   MAP_PRIVATE | MAP_ANONYMOUS | MAP_32BIT, -1, 0);
    return (uint8_t *)(((uintptr_t)raw + 8191u) & ~(uintptr_t)8191u);
}
static uint8_t *const g_flash = flash_alloc();
#define G_FLASH g_flash

#define FLOG_BASE ((uint32_t)(uintptr_t)G_FLASH)   /* picked up by flight_log.cpp */

/* ------------------------------------------------------------------ */
/* NVMCTRL register fake (SAMD51 command set, CMSIS values)            */
/*                                                                     */
/* Modeled on the SAMD51 datasheet and Adafruit's SAMD51 bootloader:   */
/*  - ADDR holds a BYTE address (the SAMD21 halfword convention does   */
/*    not apply - the bootloader writes NVMCTRL->ADDR = dst_addr).     */
/*  - CPU stores into flash land in the page buffer, not in flash;     */
/*    only WP (whole page) or WQW (16-byte quad word) at ADDR programs */
/*    them, and programming can only clear bits.                       */
/*  - EB erases the 8 KB block containing ADDR.                        */
/* The host flash buffer doubles as the CPU's view, so the fake        */
/* snapshots it at PBC and, at WP/WQW, moves whatever the CPU stored   */
/* since then into the page buffer before programming it at ADDR.      */
/* ------------------------------------------------------------------ */

#define NVMCTRL_CTRLB_CMDEX_KEY 0xA500u
#define NVMCTRL_CTRLB_CMD_EB    0x0001u   /* erase 8 KB block        */
#define NVMCTRL_CTRLB_CMD_WP    0x0003u   /* write page              */
#define NVMCTRL_CTRLB_CMD_WQW   0x0004u   /* write quad word (16 B)  */
#define NVMCTRL_CTRLB_CMD_PBC   0x0015u   /* page buffer clear       */

static uint32_t nvm_eb_count = 0;
static uint32_t nvm_wp_count = 0;
static uint32_t nvm_wqw_count = 0;
static uint32_t nvm_stray = 0;             /* programs aimed outside flash */
static uint8_t  nvm_snap[HOST_FLOG_BYTES]; /* flash as of the last PBC    */
static uint8_t  nvm_pb[512];               /* page buffer                 */
static uint32_t nvm_eb_hist[64];           /* byte addrs of first 64 erases */

/* The firmware writes NVMCTRL->ADDR.reg / ->CTRLB.reg directly, so the
 * command hook lives on the .reg member itself. */
struct CtrlbReg {
    uint32_t v;
    CtrlbReg &operator=(uint32_t x);
};

struct FakeNvmctrl {
    struct { struct { uint32_t READY; } bit; } STATUS;
    struct { uint32_t reg; } ADDR;
    struct { CtrlbReg reg; } CTRLB;
    struct { struct { uint32_t PROGE; } bit; } INTFLAG;
};

static FakeNvmctrl g_nvm;
#define NVMCTRL (&g_nvm)

/* Collect the CPU's stores since the last PBC into the page buffer and
 * put the flash view back to what is really programmed. */
static void nvm_collect_page_buffer(void)
{
    for (uint32_t pg = 0; pg < HOST_FLOG_BYTES; pg += 512) {
        if (memcmp(G_FLASH + pg, nvm_snap + pg, 512) == 0) continue;
        for (uint32_t k = 0; k < 512; k++) {
            if (G_FLASH[pg + k] != nvm_snap[pg + k]) nvm_pb[k] = G_FLASH[pg + k];
        }
        memcpy(G_FLASH + pg, nvm_snap + pg, 512);
    }
}

static void nvm_program(uintptr_t byte_addr, uint32_t len)
{
    uintptr_t off = byte_addr - (uintptr_t)G_FLASH;
    if (byte_addr < (uintptr_t)G_FLASH || off + len > HOST_FLOG_BYTES) {
        nvm_stray++;
        return;
    }
    for (uint32_t k = 0; k < len; k++) {
        G_FLASH[off + k] &= nvm_pb[(off + k) % 512];   /* bits only clear */
    }
}

CtrlbReg &CtrlbReg::operator=(uint32_t x)
{
    v = x;
    uintptr_t byte_addr = (uintptr_t)g_nvm.ADDR.reg;
    if (x == (NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD_EB)) {
        if (nvm_eb_count < 64) nvm_eb_hist[nvm_eb_count] = (uint32_t)byte_addr;
        nvm_eb_count++;
        uintptr_t blk = byte_addr & ~(uintptr_t)8191;
        uintptr_t off = blk - (uintptr_t)G_FLASH;
        if (blk >= (uintptr_t)G_FLASH && off + 8192 <= HOST_FLOG_BYTES)
            memset(G_FLASH + off, 0xFF, 8192);
        else
            nvm_stray++;
    } else if (x == (NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD_PBC)) {
        memcpy(nvm_snap, G_FLASH, HOST_FLOG_BYTES);
        memset(nvm_pb, 0xFF, sizeof(nvm_pb));
    } else if (x == (NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD_WP)) {
        nvm_wp_count++;
        nvm_collect_page_buffer();
        nvm_program(byte_addr & ~(uintptr_t)511, 512);
    } else if (x == (NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD_WQW)) {
        nvm_wqw_count++;
        nvm_collect_page_buffer();
        nvm_program(byte_addr & ~(uintptr_t)15, 16);
    }
    return *this;
}

static void nvm_reset(void)
{
    g_nvm = FakeNvmctrl();
    g_nvm.STATUS.bit.READY = 1;
    g_nvm.INTFLAG.bit.PROGE = 0;
    nvm_eb_count = 0;
    nvm_wp_count = 0;
    nvm_wqw_count = 0;
    nvm_stray = 0;
    memset(nvm_eb_hist, 0, sizeof(nvm_eb_hist));
    memcpy(nvm_snap, G_FLASH, HOST_FLOG_BYTES);
    memset(nvm_pb, 0xFF, sizeof(nvm_pb));
}

/* ------------------------------------------------------------------ */
/* Module under test (included for static state access).               */
/* Its raw-NVM address<->pointer casts are correct on the 32-bit       */
/* target; hush the (expected) 64-bit host warnings around the include */
/* only. MAP_32BIT above keeps the host buffer safely inside uint32.   */
/* ------------------------------------------------------------------ */

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wint-to-pointer-cast"
#include "../firmware/flight_log.cpp"
#pragma GCC diagnostic pop

/* ------------------------------------------------------------------ */
/* Arduino core fakes                                                  */
/* ------------------------------------------------------------------ */

FakeSerial Serial;
FakeUart Serial1;

unsigned long millis(void) { return 0; }
void delay(unsigned long ms) { (void)ms; }
void delayMicroseconds(unsigned int us) { (void)us; }
void pinMode(int pin, int mode) { (void)pin; (void)mode; }
int  digitalRead(int pin) { (void)pin; return LOW; }
void digitalWrite(int pin, int value) { (void)pin; (void)value; }

SPIClass SPI;

/* flight_log.cpp feeds the WDT inside the dump loop - a ~40 s printout
 * outlives the 16 s watchdog period and would otherwise reboot mid-dump.
 * Count the kicks here so the test can prove they happen. */
static uint32_t wdt_feeds = 0;
void watchdog_feed(void) { wdt_feeds++; }
void watchdog_init(void) {}

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

static const flog_rec_t *flash_rec(uint32_t idx)
{
    return (const flog_rec_t *)(G_FLASH + idx * FLOG_RECORD);
}

static void reset_all(void)
{
    nvm_reset();
    memset(G_FLASH, 0xFF, HOST_FLOG_BYTES);   /* blank NVM */
    flight_log_init();                         /* reinit: armed=0, wptr=0 */
    Serial.clear();
}

static int flash_all_erased(void)
{
    for (uint32_t i = 0; i < HOST_FLOG_BYTES; i++)
        if (G_FLASH[i] != 0xFF) return 0;
    return 1;
}

/* ------------------------------------------------------------------ */
/* Layout                                                              */
/* ------------------------------------------------------------------ */

TEST(record_struct_matches_write_stride)
{
    /* Only FLOG_RECORD bytes are copied per record; every field the API
     * promises to persist must fit inside them. This is the regression
     * guard for the 40-B-struct / 16-B-copy bug. */
    CHECK(sizeof(flog_rec_t) == FLOG_RECORD);
    CHECK(FLOG_PAGE % FLOG_RECORD == 0);      /* page commits are exact */
    CHECK(HOST_FLOG_BYTES % FLOG_RECORD == 0);
}

/* ------------------------------------------------------------------ */
/* Init / data-present notice                                          */
/* ------------------------------------------------------------------ */

TEST(init_blank_flash_reports_nothing)
{
    reset_all();
    CHECK(strstr(Serial.log, "[FLOG]") == nullptr);
}

TEST(init_with_previous_data_announces_dump_procedure)
{
    reset_all();
    uint32_t magic = FLOG_MAGIC;
    memcpy(G_FLASH, &magic, sizeof(magic));   /* preexisting record head */
    Serial.clear();
    flight_log_init();
    CHECK(strstr(Serial.log, "[FLOG] previous flight data present") != nullptr);
    CHECK(strstr(Serial.log, "DUMP") != nullptr);
}

/* ------------------------------------------------------------------ */
/* Arm / erase policy                                                  */
/* ------------------------------------------------------------------ */

TEST(unarmed_appends_are_dropped)
{
    reset_all();
    flight_log_imu(100, 1, 2, 3, 4, 5, 6, 7);
    CHECK(wptr == 0);
    CHECK(armed == 0);
    CHECK(nvm_wp_count == 0);
}

static int flash_data_area_erased(void)
{
    /* the append area - everything below the reserved state row */
    for (uint32_t i = 0; i < FLOG_DATA_BYTES; i++)
        if (G_FLASH[i] != 0xFF) return 0;
    return 1;
}

static const flog_rec_t *state_marker(void)
{
    return (const flog_rec_t *)(uintptr_t)FLOG_STATE_ADDR;
}

TEST(arm_erases_full_region_then_arms)
{
    reset_all();
    uint32_t magic = FLOG_MAGIC;
    memcpy(G_FLASH, &magic, sizeof(magic));   /* stale data to erase */
    flight_log_arm();
    CHECK(armed == 1);
    /* one EB per row, plus the state row's own rewrite erase */
    CHECK(nvm_eb_count == HOST_FLOG_BYTES / 8192u + 1);
    /* erase addresses sweep the whole region in 8 KB steps */
    for (uint32_t i = 0; i < HOST_FLOG_BYTES / 8192u; i++)
        CHECK(nvm_eb_hist[i] == (uint32_t)(uintptr_t)G_FLASH + i * 8192u);
    CHECK(nvm_eb_hist[HOST_FLOG_BYTES / 8192u] ==
          (uint32_t)(uintptr_t)FLOG_STATE_ADDR);   /* marker rewrite last */
    CHECK(flash_data_area_erased());
    /* Every erase and program hit the flight-log region itself. With the
     * SAMD21-style ADDR = addr >> 1 they landed at half the address:
     * nothing ever reached the log, and the arm erase swept the region
     * below it - where the sketch lives once it grows past 128 KB. */
    CHECK(nvm_stray == 0);
    /* ...and the FLYING marker is the ONE thing written at arm time */
    CHECK(nvm_wp_count == 1);
    CHECK(state_marker()->f.magic == FLOG_MAGIC);
    CHECK(state_marker()->f.kind == FLOG_KIND_STATE);
    CHECK(state_marker()->f.p.evt.code == FLOG_STATE_FLYING);
    CHECK(flight_log_prev_flying() == 1);
    CHECK(wptr == 0);
}

TEST(rearm_while_armed_is_noop)
{
    reset_all();
    flight_log_arm();
    uint32_t eb = nvm_eb_count;
    flight_log_arm();
    CHECK(nvm_eb_count == eb);
    CHECK(armed == 1);
}

/* ------------------------------------------------------------------ */
/* Append / page-commit behavior                                       */
/* ------------------------------------------------------------------ */

#define RECS_PER_PAGE (FLOG_PAGE / FLOG_RECORD)

TEST(records_reach_flash_as_they_are_written)
{
    /* A shredded airframe keeps every record written before the hit: each
     * is programmed at once as two 16-byte quad words (WQW), tail first so
     * the head - with the magic - lands last. */
    reset_all();
    flight_log_arm();
    const uint32_t wqw0 = nvm_wqw_count, wp0 = nvm_wp_count;
    flight_log_imu(1000, 10, 20, 30, 40, 50, 60, 70);
    CHECK(nvm_wqw_count == wqw0 + 2);
    CHECK(nvm_wp_count == wp0);                  /* no page writes */
    CHECK(wptr == FLOG_RECORD);
    const flog_rec_t *r0 = flash_rec(0);
    CHECK(r0->f.magic == FLOG_MAGIC);
    CHECK(r0->f.ms == 1000);
    CHECK(r0->f.kind == FLOG_KIND_IMU);
    CHECK(r0->f.p.imu.peak_mg == 70);
    CHECK(flash_rec(1)->f.magic != FLOG_MAGIC);  /* nothing beyond it */
    CHECK(nvm_stray == 0);
}

TEST(page_boundary_commit_is_byte_exact)
{
    reset_all();
    flight_log_arm();
    for (uint32_t i = 0; i < RECS_PER_PAGE; i++)
        flight_log_imu(2000 + i, 100, -200, 300,
                       4000, -5000, 6000, 5000 + (int16_t)i);
    CHECK(nvm_wp_count == 1);                    /* the marker; data by WQW */
    CHECK(flash_all_erased() == 0);
    /* first and last record of the page decode to exactly what was sent */
    const flog_rec_t *r0 = flash_rec(0);
    CHECK(r0->f.magic == FLOG_MAGIC);
    CHECK(r0->f.ms == 2000);
    CHECK(r0->f.kind == FLOG_KIND_IMU);
    CHECK(r0->f.p.imu.a_x_cg == 100);
    CHECK(r0->f.p.imu.a_y_cg == -200);
    CHECK(r0->f.p.imu.a_z_cg == 300);
    CHECK(r0->f.p.imu.g_x_dds == 4000);
    CHECK(r0->f.p.imu.g_y_dds == -5000);
    CHECK(r0->f.p.imu.g_z_dds == 6000);
    CHECK(r0->f.p.imu.peak_mg == 5000);
    CHECK(r0->f.rsv[0] == FLOG_IMU_FMT_V3);       /* units format byte */
    const flog_rec_t *rl = flash_rec(RECS_PER_PAGE - 1);
    CHECK(rl->f.ms == 2000 + RECS_PER_PAGE - 1);
    CHECK(rl->f.p.imu.peak_mg == 5000 + (int16_t)(RECS_PER_PAGE - 1));
}

TEST(gps_record_persists_full_payload)
{
    reset_all();
    flight_log_arm();
    for (uint32_t i = 0; i < RECS_PER_PAGE; i++)
        flight_log_gps(3000 + i,
                       398900000 + (int32_t)i, -1048851712 - (int32_t)i,
                       1655, 8, 1);
    CHECK(nvm_wp_count == 1);                    /* the marker; data by WQW */
    /* The fields past the 16th byte are the point of this test. */
    const flog_rec_t *r0 = flash_rec(0);
    CHECK(r0->f.kind == FLOG_KIND_GPS);
    CHECK(r0->f.p.gps.lat_e7 == 398900000);
    CHECK(r0->f.p.gps.lon_e7 == -1048851712);
    CHECK(r0->f.p.gps.alt_m == 1655);
    CHECK(r0->f.p.gps.sats == 8);
    CHECK(r0->f.p.gps.fix_q == 1);
    const flog_rec_t *rl = flash_rec(RECS_PER_PAGE - 1);
    CHECK(rl->f.p.gps.lat_e7 == 398900000 + (int32_t)(RECS_PER_PAGE - 1));
    CHECK(rl->f.p.gps.lon_e7 == -1048851712 - (int32_t)(RECS_PER_PAGE - 1));
}

TEST(event_record_persists_kind_and_time)
{
    reset_all();
    flight_log_arm();
    flight_log_event(45000, 3);         /* launch */
    for (uint32_t i = 1; i < RECS_PER_PAGE; i++)
        flight_log_event(45000 + i, 4); /* landed markers pad the page */
    const flog_rec_t *r0 = flash_rec(0);
    CHECK(r0->f.magic == FLOG_MAGIC);
    CHECK(r0->f.ms == 45000);
    CHECK(r0->f.kind == 3);
    CHECK(flash_rec(1)->f.kind == 4);
}

TEST(event_ex_persists_code_and_value)
{
    reset_all();
    flight_log_arm();
    /* kind 5 = flight event (e.g. ANOM_BALLISTIC at -15.5 m/s),
     * kind 8 = boot reset cause (RSTC byte) */
    flight_log_event_ex(46000, 5, 7, -1550);
    flight_log_event_ex(46001, 8, 0x12, 0);
    for (uint32_t i = 2; i < RECS_PER_PAGE; i++)
        flight_log_event(46000 + i, 6);
    const flog_rec_t *ev = flash_rec(0);
    CHECK(ev->f.kind == 5);
    CHECK(ev->f.p.evt.code == 7);
    CHECK(ev->f.p.evt.value == -1550);
    const flog_rec_t *boot = flash_rec(1);
    CHECK(boot->f.kind == 8);
    CHECK(boot->f.p.evt.code == 0x12);
    CHECK(boot->f.p.evt.value == 0);
    /* plain markers log through event_ex with code/value zeroed */
    const flog_rec_t *mk = flash_rec(2);
    CHECK(mk->f.kind == 6);
    CHECK(mk->f.p.evt.code == 0);
    CHECK(mk->f.p.evt.value == 0);
}

TEST(fused_record_persists_position_velocity_and_flags)
{
    reset_all();
    flight_log_arm();
    for (uint32_t i = 0; i < RECS_PER_PAGE; i++)
        flight_log_fused(47000 + i,
                         398812345, -1048987654, 1655,
                         1234, -321, (int16_t)-5500,
                         0x02 | 0x04);   /* DR, IMU healthy */
    const flog_rec_t *r0 = flash_rec(0);
    CHECK(r0->f.kind == FLOG_KIND_FUSED);
    CHECK(r0->f.p.fus.lat_e7 == 398812345);
    CHECK(r0->f.p.fus.lon_e7 == -1048987654);
    CHECK(r0->f.p.fus.alt_m == 1655);
    CHECK(r0->f.p.fus.vn_cms == 1234);
    CHECK(r0->f.p.fus.ve_cms == -321);
    CHECK(r0->f.p.fus.vd_cms == -5500);
    CHECK(r0->f.p.fus.flags == (0x02 | 0x04));
}

/* ------------------------------------------------------------------ */
/* Capacity                                                            */
/* ------------------------------------------------------------------ */

TEST(region_never_wraps_past_end)
{
    reset_all();
    flight_log_arm();
    /* data area only: the state row at the end is off-limits to appends */
    const uint32_t cap = FLOG_DATA_BYTES / FLOG_RECORD;
    for (uint32_t i = 0; i < cap; i++)
        flight_log_gps(i, 1000 + (int32_t)i, 2000, 300, 7, 1);
    CHECK(wptr == FLOG_DATA_BYTES);
    CHECK(nvm_wp_count == 1 /* marker */);
    CHECK(nvm_wqw_count == 2 * cap);              /* two quad words each */
    CHECK(nvm_stray == 0);
    /* overflow record is held, not wrapped over offset 0 */
    uint32_t wqws = nvm_wqw_count;
    flight_log_gps(99999, -1, -1, -1, 0, 0);
    CHECK(wptr == FLOG_DATA_BYTES);
    CHECK(nvm_wqw_count == wqws);
    CHECK(flash_rec(0)->f.p.gps.lat_e7 == 1000);
    /* last data slot carries the last written record */
    const flog_rec_t *last = flash_rec(cap - 1);
    CHECK(last->f.magic == FLOG_MAGIC);
    CHECK(last->f.ms == cap - 1);
    CHECK(last->f.p.gps.lat_e7 == 1000 + (int32_t)(cap - 1));
    /* ...and a packed-full data region never touched the marker row */
    CHECK(flight_log_prev_flying() == 1);
    CHECK(state_marker()->f.p.evt.code == FLOG_STATE_FLYING);
}

/* ------------------------------------------------------------------ */
/* Serial dump                                                         */
/* ------------------------------------------------------------------ */

TEST(dump_emits_csv_for_flushed_records_only)
{
    reset_all();
    flight_log_arm();
    for (uint32_t i = 0; i < RECS_PER_PAGE; i++) {
        if (i == 0)
            flight_log_imu(1234, 100, -200, 300, 4000, -5000, 6000, 5000);
        else if (i == 1)
            flight_log_gps(1235, 398900000, -1048851712, 1655, 8, 1);
        else if (i == 2)
            flight_log_event(1236, 3);
        else if (i == 3)
            flight_log_event_ex(1237, 5, 9, -550);
        else if (i == 4)
            flight_log_fused(1238, 398812345, -1048987654, 1655,
                             123, -45, (int16_t)-2200, 0x07);
        else
            flight_log_gps(1236 + i, 1, 1, 1, 1, 1);
    }
    /* a 17th record, mid-page: already in flash, so it is dumped too */
    flight_log_imu(7777, 9, 9, 9, 9, 9, 9, 9);

    Serial.clear();
    flight_log_dump_serial();

    CHECK(strstr(Serial.log, "ms,kind,ax,ay,az,gx,gy,gz,peak") != nullptr);
    CHECK(strstr(Serial.log, "# kind 1 (IMU)") != nullptr);  /* units note */
    CHECK(strstr(Serial.log, "1234,1,100,-200,300,4000,-5000,6000,5000,3")
          != nullptr);                                      /* IMU row + fmt */
    CHECK(strstr(Serial.log, "1235,2,398900000,-1048851712,1655,8,1")
          != nullptr);                                      /* GPS row  */
    CHECK(strstr(Serial.log, "1236,3") != nullptr);          /* marker row */
    CHECK(strstr(Serial.log, "1237,5,9,-550") != nullptr);    /* event+id  */
    CHECK(strstr(Serial.log,
                 "1238,9,398812345,-1048987654,1655,123,-45,-2200,7")
          != nullptr);                                        /* fused row */
    CHECK(strstr(Serial.log, "7777,1,9,9,9,9,9,9,9,3") != nullptr); /* mid-page */
    CHECK(strstr(Serial.log, "0x") == nullptr);
    /* exactly the 3-line units note + header + the 17 records + the
     * FLYING marker row: erased space is skipped, the marker is a real
     * record */
    int lines = 0;
    for (const char *p = Serial.log; *p; p++) if (*p == '\n') lines++;
    CHECK(lines == 3 + 1 + (int)RECS_PER_PAGE + 1 + 1);
    CHECK(strstr(Serial.log, "0,10,1,0") != nullptr);      /* state row */

    /* The dump feeds the WDT once per record slot - without that the
     * 16 s watchdog would reset the beacon ~halfway through the ~40 s
     * printout (the original reason DUMP was unreachable stayed hidden
     * because no caller existed at all). */
    CHECK(wdt_feeds == HOST_FLOG_BYTES / FLOG_RECORD);
}

/* ------------------------------------------------------------------ */
/* Crash-cycle round trip                                              */
/* ------------------------------------------------------------------ */

TEST(arm_commit_reboot_notice_rearm)
{
    reset_all();
    flight_log_arm();
    for (uint32_t i = 0; i < RECS_PER_PAGE; i++)
        flight_log_imu(100 + i, 1, 2, 3, 4, 5, 6, 7);

    /* "power loss" and reboot: statics stay but hardware boot re-inits */
    Serial.clear();
    flight_log_init();
    CHECK(armed == 0);
    CHECK(strstr(Serial.log, "[FLOG] previous flight data present") != nullptr);

    /* next flight arm wipes the record for the new flight (second full
     * erase cycle this boot, plus each arm's marker-row rewrite) */
    flight_log_arm();
    CHECK(nvm_eb_count == 2 * (HOST_FLOG_BYTES / 8192u + 1));
    CHECK(flash_data_area_erased());
    CHECK(flight_log_prev_flying() == 1);          /* fresh FLYING marker */
    CHECK(wptr == 0);
}

/* ------------------------------------------------------------------ */
/* Mid-flight reboot-resume (state marker row)                          */
/* ------------------------------------------------------------------ */

TEST(prev_flying_reads_the_marker_row)
{
    reset_all();
    CHECK(flight_log_prev_flying() == 0);          /* erased row = boot   */
    flight_log_arm();
    CHECK(flight_log_prev_flying() == 1);          /* FLYING written      */
    flight_log_mark_landed();
    CHECK(flight_log_prev_flying() == 0);          /* LANDED = power-on   */
    CHECK(state_marker()->f.p.evt.code == FLOG_STATE_LANDED);
}

TEST(resume_restores_append_point_without_erase)
{
    reset_all();
    flight_log_arm();
    /* three pages of GPS, then one more record mid-page, then "power
     * loss": every record written before the reset survives */
    for (uint32_t i = 0; i < 3 * RECS_PER_PAGE; i++)
        flight_log_gps(5000 + i, 1000 + (int32_t)i, 2000, 300, 7, 1);
    flight_log_gps(9999, -1, -1, -1, 0, 0);        /* mid-page, survives  */
    const uint32_t wp_before = nvm_wp_count;
    const uint32_t wqw_before = nvm_wqw_count;
    const uint32_t eb_before = nvm_eb_count;

    /* The reset: RAM state dies with the chip (init models it); flash and
     * the FLYING marker survive. */
    flight_log_init();
    CHECK(flight_log_prev_flying() == 1);
    CHECK(flight_log_resume() == 1);
    CHECK(armed == 1);
    CHECK(wptr == 3 * FLOG_PAGE + FLOG_RECORD);    /* first blank slot    */
    CHECK(nvm_eb_count == eb_before);              /* NO erase happened   */
    CHECK(nvm_wp_count == wp_before);              /* nothing rewritten   */
    CHECK(nvm_wqw_count == wqw_before);

    /* Appends continue exactly where the previous boot left off: the
     * next page commit lands at wptr and old records are untouched. */
    for (uint32_t i = 0; i < RECS_PER_PAGE; i++)
        flight_log_gps(6000 + i, -99 - (int32_t)i, 2000, 300, 7, 1);
    CHECK(flash_rec(3 * RECS_PER_PAGE)->f.ms == 9999);    /* survived */
    const flog_rec_t *res = flash_rec(3 * RECS_PER_PAGE + 1);
    CHECK(res->f.magic == FLOG_MAGIC);
    CHECK(res->f.ms == 6000);
    CHECK(res->f.p.gps.lat_e7 == -99);
    /* pre-reset records intact */
    CHECK(flash_rec(0)->f.ms == 5000);
    CHECK(flash_rec(0)->f.p.gps.lat_e7 == 1000);
    CHECK(flash_rec(3 * RECS_PER_PAGE - 1)->f.p.gps.lat_e7 ==
          1000 + (int32_t)(3 * RECS_PER_PAGE - 1));
}

TEST(resume_skips_a_torn_record)
{
    /* Power died between a record's two quad words: the tail is
     * programmed, the head (with the magic) never was. Programming only
     * clears bits, so that slot can't be reused - resume appends after
     * it - and DUMP skips it for lack of a magic. */
    reset_all();
    flight_log_arm();
    for (uint32_t i = 0; i < 5; i++) flight_log_gps(100 + i, 1, 2, 3, 7, 1);
    memset(G_FLASH + 5 * FLOG_RECORD + 16, 0x00, 16);   /* torn: tail only */
    flight_log_init();
    CHECK(flight_log_resume() == 1);
    CHECK(wptr == 6 * FLOG_RECORD);
    flight_log_gps(200, 9, 9, 9, 7, 1);
    CHECK(flash_rec(6)->f.ms == 200);
    CHECK(flash_rec(5)->f.magic != FLOG_MAGIC);
    CHECK(flash_rec(4)->f.ms == 104);
}

TEST(resume_requires_the_flying_marker)
{
    reset_all();
    CHECK(flight_log_resume() == 0);               /* blank flash: boot   */
    CHECK(armed == 0);

    flight_log_arm();
    flight_log_mark_landed();
    flight_log_init();
    CHECK(flight_log_resume() == 0);               /* landed flight: boot */
    CHECK(armed == 0);
}

/* ------------------------------------------------------------------ */

int main(void)
{
    run_record_struct_matches_write_stride();
    run_init_blank_flash_reports_nothing();
    run_init_with_previous_data_announces_dump_procedure();
    run_unarmed_appends_are_dropped();
    run_arm_erases_full_region_then_arms();
    run_rearm_while_armed_is_noop();
    run_records_reach_flash_as_they_are_written();
    run_page_boundary_commit_is_byte_exact();
    run_gps_record_persists_full_payload();
    run_event_record_persists_kind_and_time();
    run_event_ex_persists_code_and_value();
    run_fused_record_persists_position_velocity_and_flags();
    run_region_never_wraps_past_end();
    run_dump_emits_csv_for_flushed_records_only();
    run_arm_commit_reboot_notice_rearm();
    run_prev_flying_reads_the_marker_row();
    run_resume_restores_append_point_without_erase();
    run_resume_skips_a_torn_record();
    run_resume_requires_the_flying_marker();
    return TEST_SUMMARY();
}
