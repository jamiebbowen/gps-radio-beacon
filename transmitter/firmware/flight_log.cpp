/**
 * @file flight_log.cpp
 * @brief On-beacon flight recorder in SAMD51 internal flash.
 *
 * Rationale (L0016 shredding): a beacon destroyed in flight never speaks
 * again over RF. The strongest surviving copy is the chip itself, found in
 * the wreck - if the airframe shreds, this is what a decomposition crew
 * reads over the debug UART.
 *
 * Memory model: a flat append-only log at the back of the 512 KB on-chip
 * flash, laid out as 32-byte fixed records. Pages are written when full
 * (page-register buffer), so a power loss mid-write costs at most the
 * current page. The record layout is a PER-KIND payload union: the header
 * (magic/ms/kind) plus the largest payload must fit inside FLOG_RECORD
 * bytes - flog_append_record only persists FLOG_RECORD per append, and the
 * host test static-asserts sizeof(flog_rec_t) == FLOG_RECORD so the
 * struct can't silently outgrow the write stride again.
 */

#include "include/flight_log.h"
#include "include/config.h"
#include "include/watchdog.h"  /* dump keeps the WDT fed mid-printout */
#include <Arduino.h>
#include <string.h>
#include <stdio.h>

/* =====================================================================
 * Region carve-up: 256 KB at the end of internal flash, far above the
 * ~80 KB sketch. At SAMD51 8 KB row-erase granularity that's 32 rows.
 * If the sketch ever approaches this base the linker's build output and
 * the arm-time delete get attention.
 * ===================================================================== */
/* FLOG_BASE is overridable so host tests can redirect the log region at a
 * real buffer (transmitter/tests/test_flight_log.cpp); on target it is the
 * fixed 0x00040000 carve-up above. */
#ifndef FLOG_BASE
#define FLOG_BASE        0x00040000u
#endif
#ifndef FLOG_BYTES
#define FLOG_BYTES       (256u * 1024u)
#endif
#define FLOG_PAGE        512u        /* SAMD51 NVM page */
#define FLOG_ROW         8192u       /* erase granularity (16 pages; SAMD51
                                        rev B calls this 'Erase Block') */
/* 16 records per 512 B page; 8192 records in the 256 KB region. */
#define FLOG_RECORD      32u
#define FLOG_MAGIC       0x464C4647u /* 'GFLF' - little-endian scalar   */


/* Flight-state marker codes (kind 10 evt.code). Lives on the reserved
 * last row, OUTSIDE the append area, so a wrap-to-full data region can
 * never overwrite it. */
#define FLOG_STATE_FLYING       1   /* armed and airborne                */
#define FLOG_STATE_LANDED       2   /* landing latch hit                 */

/* The last erase row of the region is reserved for the state marker;
 * normal records append only into [0, FLOG_DATA_BYTES). The marker is
 * what a mid-flight WDT/brown-out reboot uses to decide "resume the
 * running flight" vs "pad power-up" at boot. An erased row reads 0xFF
 * (kind byte 0xFF) -> no marker -> power-on. */
#define FLOG_DATA_BYTES   (FLOG_BYTES - FLOG_ROW)
#define FLOG_STATE_ADDR   (FLOG_BASE + FLOG_DATA_BYTES)

/* Fused-record health flags (kind 9 payload). Same meaning as the fused
 * wire packet's flags, kept separate so the crash copy can't drift with
 * future wire-format versions. */

typedef union {
    uint8_t  raw[FLOG_RECORD];
    struct {
        uint32_t magic;
        uint32_t ms;
        uint8_t  kind;
        uint8_t  rsv[3];
        union {
            struct {
                int16_t  a_x_cg, a_y_cg, a_z_cg;
                int16_t  g_x_dds, g_y_dds, g_z_dds;   /* IMU */
                int16_t  peak_mg;
            } imu;                                    /* 14 B */
            struct {
                int32_t  lat_e7, lon_e7;              /* GPS */
                int16_t  alt_m;
                uint8_t  sats;
                uint8_t  fix_q;
            } gps;                                    /* 12 B */
            struct {
                uint8_t  code;                        /* event id / RCAUSE */
                uint8_t  rsv1;
                int16_t  value;                       /* event magnitude   */
            } evt;                                    /* 4 B, kinds 5-8    */
            struct {
                int32_t  lat_e7, lon_e7;              /* fused EKF snapshot */
                int16_t  alt_m;
                int16_t  vn_cms, ve_cms, vd_cms;
                uint8_t  flags;
            } fus;                                    /* 15 B, kind 9      */
        } p;
    } f;
} flog_rec_t;

/* State */
static volatile uint8_t armed = 0;       /* reset-region armed           */
static uint32_t wptr = 0;                /* bytes written so far         */

/* ------------------------------------------------------------------ */
/* NVMCTRL helpers (SAMD51 Hardware Asynchronous Access pattern)       */
/* ------------------------------------------------------------------ */

static void nvm_wait_ready(void)
{
    while (!NVMCTRL->STATUS.bit.READY) { }
}

/* Erase one 8 KB block. SAMD51 NVM (rev B) is block-erased (8 KB blocks). */
static bool nvm_erase_block(uint32_t addr)
{
    nvm_wait_ready();
    NVMCTRL->ADDR.reg = addr;      /* SAMD51: byte address (SAMD21 took addr >> 1) */
    NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD_EB;
      nvm_wait_ready();
    return (NVMCTRL->INTFLAG.bit.PROGE == 0);
}

/* Write a 512-byte page from the provided buffer. */
static bool nvm_write_page(uint32_t addr, const uint8_t *buf)
{
    nvm_wait_ready();
    NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD_PBC;
    nvm_wait_ready();
    volatile uint32_t *dst = (volatile uint32_t *)addr;
    const uint32_t *src = (const uint32_t *)buf;
    for (int i = 0; i < (int)(FLOG_PAGE / 4); i++) {
        dst[i] = src[i];
    }
    NVMCTRL->ADDR.reg = addr;      /* SAMD51: byte address (SAMD21 took addr >> 1) */
    NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD_WP;
  nvm_wait_ready();
    return (NVMCTRL->INTFLAG.bit.PROGE == 0);
}

/* ------------------------------------------------------------------ */
/* Public API                                                           */
/* ------------------------------------------------------------------ */

void flight_log_init(void)
{
    armed = 0;
    wptr = 0;

    /* If a previous flight left data (magic at 0), print it over USB at
     * boot so a recovery crew just opens the serial monitor. */
    const flog_rec_t *first = (const flog_rec_t *)FLOG_BASE;
    if (first->f.magic == FLOG_MAGIC) {
        Serial.println(F("[FLOG] previous flight data present"));
        Serial.println(F("[FLOG] type 'DUMP' + Enter over USB CDC before rearm"));
    }
}

/* Page write buffer: records accumulate here and commit per NVM page.
 * Async-safety is deliberately NOT aimed at: the main loop calls the
 * append API on event edges only. */
static uint8_t page_buf[FLOG_PAGE];
static uint16_t page_fill = 0;

/* Write the flight-state marker (kind 10) onto its reserved row. The row
 * is always erased first because state writes (FLYING -> LANDED) flip
 * bits in both directions, which NAND cannot do in place. */
static void flog_write_state(uint8_t code)
{
    static uint8_t state_page[FLOG_PAGE];
    flog_rec_t r;
    memset(&r, 0, sizeof(r));
    memset(state_page, 0xFF, sizeof(state_page));
    r.f.magic       = FLOG_MAGIC;
    r.f.kind        = FLOG_KIND_STATE;
    r.f.p.evt.code  = code;
    memcpy(state_page, &r, sizeof(r));
    (void)nvm_erase_block(FLOG_STATE_ADDR);
    (void)nvm_write_page(FLOG_STATE_ADDR, state_page);
}

void flight_log_arm(void)
{
    if (armed) return;
    for (uint32_t row = FLOG_BASE; row < FLOG_BASE + FLOG_BYTES; row += FLOG_ROW) {
        (void)nvm_erase_block(row);
    }
    armed = 1;
    wptr = 0;
    flog_write_state(FLOG_STATE_FLYING);
}

uint8_t flight_log_prev_flying(void)
{
    const flog_rec_t *s = (const flog_rec_t *)FLOG_STATE_ADDR;
    return (s->f.magic == FLOG_MAGIC &&
            s->f.kind  == FLOG_KIND_STATE &&
            s->f.p.evt.code == FLOG_STATE_FLYING);
}

uint8_t flight_log_resume(void)
{
    if (armed || !flight_log_prev_flying()) return 0;

    /* Find the append point: first page whose FIRST record lacks the
     * magic. Power (or the reset) died with at most one partially-
     * buffered page in RAM - every earlier page was committed. */
    wptr = 0;
    while (wptr < FLOG_DATA_BYTES) {
        const flog_rec_t *r = (const flog_rec_t *)(FLOG_BASE + wptr);
        if (r->f.magic != FLOG_MAGIC) break;
        wptr += FLOG_PAGE;
    }
    page_fill = 0;
    armed = 1;
    return 1;
}

void flight_log_mark_landed(void)
{
    if (!armed) return;
    flog_write_state(FLOG_STATE_LANDED);
}

static void flog_append_record(const flog_rec_t *rec)
{
    if (!armed) return;
    if (wptr + FLOG_RECORD > FLOG_DATA_BYTES) return;  /* hold; state row
                                                        * is off-limits */

    memcpy(page_buf + page_fill, rec, FLOG_RECORD);
    page_fill += FLOG_RECORD;
    wptr += FLOG_RECORD;

    if (page_fill >= FLOG_PAGE) {
        (void)nvm_write_page(FLOG_BASE + wptr - FLOG_PAGE, page_buf);
        page_fill = 0;
    }
}

void flight_log_imu(uint32_t ms,
                    int16_t ax_cg, int16_t ay_cg, int16_t az_cg,
                    int16_t gx_dds, int16_t gy_dds, int16_t gz_dds,
                    int16_t peak_mg)
{
    flog_rec_t r;
    memset(&r, 0, sizeof(r));
    r.f.magic = FLOG_MAGIC;
    r.f.ms = ms;
    r.f.kind = FLOG_KIND_IMU;
    r.f.rsv[0] = FLOG_IMU_FMT_V3;   /* 0 on records from pre-V3 firmware */
    r.f.p.imu.a_x_cg = ax_cg; r.f.p.imu.a_y_cg = ay_cg; r.f.p.imu.a_z_cg = az_cg;
    r.f.p.imu.g_x_dds = gx_dds; r.f.p.imu.g_y_dds = gy_dds; r.f.p.imu.g_z_dds = gz_dds;
    r.f.p.imu.peak_mg = peak_mg;
    flog_append_record(&r);
}

void flight_log_gps(uint32_t ms,
                    int32_t lat_e7, int32_t lon_e7, int16_t alt_m,
                    uint8_t sats, uint8_t fix_q)
{
    flog_rec_t r;
    memset(&r, 0, sizeof(r));
    r.f.magic = FLOG_MAGIC;
    r.f.ms = ms;
    r.f.kind = FLOG_KIND_GPS;
    r.f.p.gps.lat_e7 = lat_e7;
    r.f.p.gps.lon_e7 = lon_e7;
    r.f.p.gps.alt_m = alt_m;
    r.f.p.gps.sats = sats;
    r.f.p.gps.fix_q = fix_q;
    flog_append_record(&r);
}

void flight_log_event(uint32_t ms, uint8_t kind)
{
    flight_log_event_ex(ms, kind, 0, 0);
}

void flight_log_event_ex(uint32_t ms, uint8_t kind, uint8_t code, int16_t value)
{
    flog_rec_t r;
    memset(&r, 0, sizeof(r));
    r.f.magic = FLOG_MAGIC;
    r.f.ms = ms;
    r.f.kind = kind;
    r.f.p.evt.code = code;
    r.f.p.evt.value = value;
    flog_append_record(&r);
}

void flight_log_fused(uint32_t ms, int32_t lat_e7, int32_t lon_e7,
                      int16_t alt_m, int16_t vn_cms, int16_t ve_cms,
                      int16_t vd_cms, uint8_t flags)
{
    flog_rec_t r;
    memset(&r, 0, sizeof(r));
    r.f.magic = FLOG_MAGIC;
    r.f.ms = ms;
    r.f.kind = FLOG_KIND_FUSED;
    r.f.p.fus.lat_e7 = lat_e7;
    r.f.p.fus.lon_e7 = lon_e7;
    r.f.p.fus.alt_m = alt_m;
    r.f.p.fus.vn_cms = vn_cms;
    r.f.p.fus.ve_cms = ve_cms;
    r.f.p.fus.vd_cms = vd_cms;
    r.f.p.fus.flags = flags;
    flog_append_record(&r);
}

void flight_log_dump_serial(void)
{
    /* Parse records linearly over the whole region; skip magic-only padding.
     * 8192 records at 115200 baud is a ~40 s printout - the 16 s WDT would
     * reboot mid-dump, so feed it on every record (a 2-op register poke). */
    Serial.println(F("# kind 1 (IMU): ax,ay,az centi-g; gx,gy,gz 0.1 deg/s; peak milli-g;"));
    Serial.println(F("#   last column fmt: 3 = these units, 0 = pre-V3 firmware (cm/s^2,"));
    Serial.println(F("#   0.01 deg/s, mm/s^2 - may have wrapped)"));
    Serial.println(F("ms,kind,ax,ay,az,gx,gy,gz,peak,stat_everything"));
    for (uint32_t off = 0; off < FLOG_BYTES; off += FLOG_RECORD) {
        watchdog_feed();
        const flog_rec_t *r = (const flog_rec_t *)(FLOG_BASE + off);
        if (r->f.magic != FLOG_MAGIC) continue;
        Serial.print(r->f.ms);         Serial.print(',');
        Serial.print(r->f.kind);       Serial.print(',');
        if (r->f.kind == FLOG_KIND_IMU) {
            Serial.print(r->f.p.imu.a_x_cg); Serial.print(',');
            Serial.print(r->f.p.imu.a_y_cg); Serial.print(',');
            Serial.print(r->f.p.imu.a_z_cg); Serial.print(',');
            Serial.print(r->f.p.imu.g_x_dds);Serial.print(',');
            Serial.print(r->f.p.imu.g_y_dds);Serial.print(',');
            Serial.print(r->f.p.imu.g_z_dds);Serial.print(',');
            Serial.print(r->f.p.imu.peak_mg);Serial.print(',');
            Serial.print(r->f.rsv[0]);
        } else if (r->f.kind == FLOG_KIND_GPS) {
            Serial.print(r->f.p.gps.lat_e7); Serial.print(',');
            Serial.print(r->f.p.gps.lon_e7); Serial.print(',');
            Serial.print(r->f.p.gps.alt_m);  Serial.print(',');
            Serial.print(r->f.p.gps.sats);   Serial.print(',');
            Serial.print(r->f.p.gps.fix_q);
        } else if (r->f.kind == FLOG_KIND_FUSED) {
            Serial.print(r->f.p.fus.lat_e7); Serial.print(',');
            Serial.print(r->f.p.fus.lon_e7); Serial.print(',');
            Serial.print(r->f.p.fus.alt_m);  Serial.print(',');
            Serial.print(r->f.p.fus.vn_cms); Serial.print(',');
            Serial.print(r->f.p.fus.ve_cms); Serial.print(',');
            Serial.print(r->f.p.fus.vd_cms); Serial.print(',');
            Serial.print(r->f.p.fus.flags);
        } else if (r->f.kind == FLOG_KIND_FLIGHT_EVENT ||
                   r->f.kind == FLOG_KIND_BOOT ||
                   r->f.kind == FLOG_KIND_STATE) {
            Serial.print(r->f.p.evt.code);   Serial.print(',');
            Serial.print(r->f.p.evt.value);
        }
        Serial.println();
    }
}
