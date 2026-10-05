/**
 * @file test_flight_log.cpp
 * @brief Host-side unit tests for the transmitter on-chip flight recorder.
 *
 * Redirects the log region at a host buffer (FLOG_BASE override, guarded
 * in flight_log.cpp) and fakes the SAMD51 NVMCTRL register block so the
 * production erase/page-commit path runs unmodified:
 *   - arm: full-region block erase, erase address histogram, arm/rearm policy
 *   - append: records park in the page buffer until the 512 B page commits
 *     (power loss mid-page costs at most one page - the module's whole pitch)
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

static uint8_t *const g_flash =
    (uint8_t *)mmap(nullptr, HOST_FLOG_BYTES, PROT_READ | PROT_WRITE,
                    MAP_PRIVATE | MAP_ANONYMOUS | MAP_32BIT, -1, 0);
#define G_FLASH g_flash

#define FLOG_BASE ((uint32_t)(uintptr_t)G_FLASH)   /* picked up by flight_log.cpp */

/* ------------------------------------------------------------------ */
/* NVMCTRL register fake (SAMD51 rev-B command set)                    */
/* ------------------------------------------------------------------ */

#define NVMCTRL_CTRLB_CMDEX_KEY 0xA500u
#define NVMCTRL_CTRLB_CMD_EB    0x0001u   /* erase 8 KB block        */
#define NVMCTRL_CTRLB_CMD_PBC   0x0003u   /* page buffer clear       */
#define NVMCTRL_CTRLB_CMD_WP    0x0004u   /* write page              */

static uint32_t nvm_eb_count = 0;
static uint32_t nvm_wp_count = 0;
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

/* A real block erase fills the row with 0xFF; model that so the module
 * sees honest "blank flash" between arm cycles. */
CtrlbReg &CtrlbReg::operator=(uint32_t x)
{
    v = x;
    if (x == (NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD_EB)) {
        uintptr_t byte_addr = (uintptr_t)g_nvm.ADDR.reg << 1;
        if (nvm_eb_count < 64) nvm_eb_hist[nvm_eb_count] = (uint32_t)byte_addr;
        nvm_eb_count++;
        uintptr_t off = byte_addr - (uintptr_t)G_FLASH;
        if (byte_addr >= (uintptr_t)G_FLASH && off + 8192 <= HOST_FLOG_BYTES)
            memset(G_FLASH + off, 0xFF, 8192);
    } else if (x == (NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD_WP)) {
        nvm_wp_count++;
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
    memset(nvm_eb_hist, 0, sizeof(nvm_eb_hist));
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
    page_fill = 0;                             /* statics outlive init; on  */
    memset(page_buf, 0xFF, sizeof(page_buf));  /* target they die with RAM  */
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
    CHECK(flight_log_armed() == 0);
    CHECK(nvm_wp_count == 0);
}

TEST(arm_erases_full_region_then_arms)
{
    reset_all();
    uint32_t magic = FLOG_MAGIC;
    memcpy(G_FLASH, &magic, sizeof(magic));   /* stale data to erase */
    flight_log_arm();
    CHECK(flight_log_armed() == 1);
    CHECK(flight_log_busy() == 0);
    CHECK(nvm_eb_count == HOST_FLOG_BYTES / 8192u);   /* one EB per row */
    /* erase addresses sweep the whole region in 8 KB steps */
    for (uint32_t i = 0; i < nvm_eb_count && i < 64; i++)
        CHECK(nvm_eb_hist[i] == (uint32_t)(uintptr_t)G_FLASH + i * 8192u);
    CHECK(flash_all_erased());
    CHECK(wptr == 0);
}

TEST(rearm_while_armed_is_noop)
{
    reset_all();
    flight_log_arm();
    uint32_t eb = nvm_eb_count;
    flight_log_arm();
    CHECK(nvm_eb_count == eb);
    CHECK(flight_log_armed() == 1);
}

/* ------------------------------------------------------------------ */
/* Append / page-commit behavior                                       */
/* ------------------------------------------------------------------ */

#define RECS_PER_PAGE (FLOG_PAGE / FLOG_RECORD)

TEST(subpage_records_stay_in_ram_until_commit)
{
    reset_all();
    flight_log_arm();
    const uint32_t n = RECS_PER_PAGE - 1;
    for (uint32_t i = 0; i < n; i++)
        flight_log_imu(1000 + i, 10, 20, 30, 40, 50, 60, 70);
    CHECK(wptr == n * FLOG_RECORD);
    CHECK(nvm_wp_count == 0);                       /* nothing committed    */
    CHECK(flash_rec(0)->f.magic != FLOG_MAGIC);     /* power-loss envelope  */
    /* ...but the page buffer holds them, byte-exact */
    const flog_rec_t *buf0 = (const flog_rec_t *)page_buf;
    CHECK(buf0->f.magic == FLOG_MAGIC);
    CHECK(buf0->f.ms == 1000);
    CHECK(buf0->f.kind == FLOG_KIND_IMU);
}

TEST(page_boundary_commit_is_byte_exact)
{
    reset_all();
    flight_log_arm();
    for (uint32_t i = 0; i < RECS_PER_PAGE; i++)
        flight_log_imu(2000 + i, 100, -200, 300,
                       4000, -5000, 6000, 5000 + (int16_t)i);
    CHECK(nvm_wp_count == 1);
    CHECK(flash_all_erased() == 0);
    /* first and last record of the page decode to exactly what was sent */
    const flog_rec_t *r0 = flash_rec(0);
    CHECK(r0->f.magic == FLOG_MAGIC);
    CHECK(r0->f.ms == 2000);
    CHECK(r0->f.kind == FLOG_KIND_IMU);
    CHECK(r0->f.p.imu.a_x_cg == 100);
    CHECK(r0->f.p.imu.a_y_cg == -200);
    CHECK(r0->f.p.imu.a_z_cg == 300);
    CHECK(r0->f.p.imu.g_x_cds == 4000);
    CHECK(r0->f.p.imu.g_y_cds == -5000);
    CHECK(r0->f.p.imu.g_z_cds == 6000);
    CHECK(r0->f.p.imu.peak_mg == 5000);
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
    CHECK(nvm_wp_count == 1);
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
    const uint32_t cap = HOST_FLOG_BYTES / FLOG_RECORD;
    for (uint32_t i = 0; i < cap; i++)
        flight_log_gps(i, 1000 + (int32_t)i, 2000, 300, 7, 1);
    CHECK(wptr == HOST_FLOG_BYTES);
    CHECK(nvm_wp_count == HOST_FLOG_BYTES / FLOG_PAGE);
    /* overflow record is held, not wrapped over offset 0 */
    uint32_t wps = nvm_wp_count;
    flight_log_gps(99999, -1, -1, -1, 0, 0);
    CHECK(wptr == HOST_FLOG_BYTES);
    CHECK(nvm_wp_count == wps);
    CHECK(flash_rec(0)->f.p.gps.lat_e7 == 1000);
    /* last slot carries the last written record */
    const flog_rec_t *last = flash_rec(cap - 1);
    CHECK(last->f.magic == FLOG_MAGIC);
    CHECK(last->f.ms == cap - 1);
    CHECK(last->f.p.gps.lat_e7 == 1000 + (int32_t)(cap - 1));
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
    /* one more record sits in the (uncommitted) page buffer and must not
     * appear in the dump */
    flight_log_imu(7777, 9, 9, 9, 9, 9, 9, 9);

    Serial.clear();
    flight_log_dump_serial();

    CHECK(strstr(Serial.log, "ms,kind,ax,ay,az,gx,gy,gz,peak") != nullptr);
    CHECK(strstr(Serial.log, "1234,1,100,-200,300,4000,-5000,6000,5000")
          != nullptr);                                      /* IMU row  */
    CHECK(strstr(Serial.log, "1235,2,398900000,-1048851712,1655,8,1")
          != nullptr);                                      /* GPS row  */
    CHECK(strstr(Serial.log, "1236,3") != nullptr);          /* marker row */
    CHECK(strstr(Serial.log, "1237,5,9,-550") != nullptr);    /* event+id  */
    CHECK(strstr(Serial.log,
                 "1238,9,398812345,-1048987654,1655,123,-45,-2200,7")
          != nullptr);                                        /* fused row */
    CHECK(strstr(Serial.log, "7777") == nullptr);            /* unflushed */
    CHECK(strstr(Serial.log, "0x") == nullptr);
    /* exactly header + the 16 flushed records: erased space is skipped */
    int lines = 0;
    for (const char *p = Serial.log; *p; p++) if (*p == '\n') lines++;
    CHECK(lines == 1 + (int)RECS_PER_PAGE);
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
    CHECK(flight_log_armed() == 0);
    CHECK(strstr(Serial.log, "[FLOG] previous flight data present") != nullptr);

    /* next flight arm wipes the record for the new flight (second full
     * erase cycle this boot) */
    flight_log_arm();
    CHECK(nvm_eb_count == 2 * (HOST_FLOG_BYTES / 8192u));
    CHECK(flash_all_erased());
    CHECK(wptr == 0);
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
    run_subpage_records_stay_in_ram_until_commit();
    run_page_boundary_commit_is_byte_exact();
    run_gps_record_persists_full_payload();
    run_event_record_persists_kind_and_time();
    run_event_ex_persists_code_and_value();
    run_fused_record_persists_position_velocity_and_flags();
    run_region_never_wraps_past_end();
    run_dump_emits_csv_for_flushed_records_only();
    run_arm_commit_reboot_notice_rearm();
    return TEST_SUMMARY();
}
