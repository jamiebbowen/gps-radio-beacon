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
 * flash, laid out as 16-byte fixed records (the same 16-byte stride the
 * card's LittleFS rows use for IMU). Pages are written when full
 * (page-register buffer), so a power loss mid-write costs at most the
 * current page.
 */

#include "include/flight_log.h"
#include "include/config.h"
#include "include/radio.h"      /* nothing; same build-tree include chain */
#include <Arduino.h>
#include <string.h>
#include <stdio.h>

/* =====================================================================
 * Region carve-up: 256 KB at the end of internal flash, far above the
 * ~80 KB sketch. At SAMD51 8 KB row-erase granularity that's 32 rows.
 * If the sketch ever approaches this base the linker's build output and
 * the arm-time delete get attention.
 * ===================================================================== */
#define FLOG_BASE        0x00040000u
#define FLOG_BYTES       (256u * 1024u)
#define FLOG_PAGE        512u        /* SAMD51 NVM page */
#define FLOG_ROW         8192u       /* erase granularity (16 pages; SAMD51
                                        rev B calls this 'Erase Block') */
#define FLOG_RECORD      16u
#define FLOG_MAGIC       0x464C4647u /* 'GFLF' - little-endian scalar   */

#define FLOG_KIND_IMU     1
#define FLOG_KIND_GPS     2
#define FLOG_KIND_LAUNCH  3
#define FLOG_KIND_LANDED  4

typedef union {
    uint8_t  raw[FLOG_RECORD];
    struct {
        uint32_t magic;
        uint32_t ms;
        uint8_t  kind;
        uint8_t  rsv[3];
        int16_t  a_x_cg, a_y_cg, a_z_cg;
        int16_t  g_x_cds, g_y_cds, g_z_cds;   /* IMU */
        int32_t  lat_e7, lon_e7;              /* GPS (reuses a/g slots) */
        int16_t  alt_m;
        uint8_t  sats;
        uint8_t  fix_q;
        int16_t  peak_mg;
    } f;
} flog_rec_t;

/* State */
static volatile uint8_t armed = 0;       /* reset-region armed           */
static volatile uint8_t busy_erase = 0;  /* currently running erase      */
static uint32_t wptr = 0;                /* bytes written so far         */

/* ------------------------------------------------------------------ */
/* NVMCTRL helpers (SAMD51 Hardware Asynchronous Access pattern)       */
/* ------------------------------------------------------------------ */

#define NVMCTRL_NVM_REG  NVMCTRL

static void nvm_wait_ready(void)
{
    while (!NVMCTRL->STATUS.bit.READY) { }
}

/* Erase one 8 KB block. SAMD51 NVM (rev B) is block-erased (8 KB blocks). */
static bool nvm_erase_block(uint32_t addr)
{
    nvm_wait_ready();
    NVMCTRL->ADDR.reg = addr >> 1;
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
    NVMCTRL->ADDR.reg = addr >> 1;
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
    busy_erase = 0;
    wptr = 0;

    /* If a previous flight left data (magic at 0), print it over USB at
     * boot so a recovery crew just opens the serial monitor. */
    const flog_rec_t *first = (const flog_rec_t *)FLOG_BASE;
    if (first->f.magic == FLOG_MAGIC) {
        Serial.println(F("[FLOG] previous flight data present"));
        Serial.println(F("[FLOG] type 'DUMP' + Enter over USB CDC before rearm"));
    }
}

void flight_log_arm(void)
{
    if (armed || busy_erase) return;
    busy_erase = 1;
    for (uint32_t row = FLOG_BASE; row < FLOG_BASE + FLOG_BYTES; row += FLOG_ROW) {
        (void)nvm_erase_block(row);
    }
    busy_erase = 0;
    armed = 1;
    wptr = 0;
}

uint8_t flight_log_armed(void) { return armed; }
uint8_t flight_log_busy(void)  { return busy_erase; }

/* Append one record. Async-safety is deliberately NOT aimed at: the main
 * loop calls these on event edges only. */
static uint8_t page_buf[FLOG_PAGE];
static uint16_t page_fill = 0;

static void flog_append_record(const flog_rec_t *rec)
{
    if (!armed) return;
    if (wptr + FLOG_RECORD > FLOG_BYTES) return;  /* hold; never wrap past end */

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
                    int16_t gx_cds, int16_t gy_cds, int16_t gz_cds,
                    int16_t peak_mg)
{
    flog_rec_t r;
    memset(&r, 0, sizeof(r));
    r.f.magic = FLOG_MAGIC;
    r.f.ms = ms;
    r.f.kind = FLOG_KIND_IMU;
    r.f.a_x_cg = ax_cg; r.f.a_y_cg = ay_cg; r.f.a_z_cg = az_cg;
    r.f.g_x_cds = gx_cds; r.f.g_y_cds = gy_cds; r.f.g_z_cds = gz_cds;
    r.f.peak_mg = peak_mg;
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
    r.f.lat_e7 = lat_e7;
    r.f.lon_e7 = lon_e7;
    r.f.alt_m = alt_m;
    r.f.sats = sats;
    r.f.fix_q = fix_q;
    flog_append_record(&r);
}

void flight_log_event(uint32_t ms, uint8_t kind)
{
    flog_rec_t r;
    memset(&r, 0, sizeof(r));
    r.f.magic = FLOG_MAGIC;
    r.f.ms = ms;
    r.f.kind = kind;
    flog_append_record(&r);
}

void flight_log_dump_serial(void)
{
    /* Parse records linearly over the whole region; skip magic-only padding */
    Serial.println(F("ms,kind,ax,ay,az,gx,gy,gz,peak,stat_everything"));
    for (uint32_t off = 0; off < FLOG_BYTES; off += FLOG_RECORD) {
        const flog_rec_t *r = (const flog_rec_t *)(FLOG_BASE + off);
        if (r->f.magic != FLOG_MAGIC) continue;
        Serial.print(r->f.ms);         Serial.print(',');
        Serial.print(r->f.kind);       Serial.print(',');
        if (r->f.kind == FLOG_KIND_IMU) {
            Serial.print(r->f.a_x_cg); Serial.print(',');
            Serial.print(r->f.a_y_cg); Serial.print(',');
            Serial.print(r->f.a_z_cg); Serial.print(',');
            Serial.print(r->f.g_x_cds);Serial.print(',');
            Serial.print(r->f.g_y_cds);Serial.print(',');
            Serial.print(r->f.g_z_cds);Serial.print(',');
            Serial.print(r->f.peak_mg);
        } else if (r->f.kind == FLOG_KIND_GPS) {
            Serial.print(r->f.lat_e7); Serial.print(',');
            Serial.print(r->f.lon_e7); Serial.print(',');
            Serial.print(r->f.alt_m);  Serial.print(',');
            Serial.print(r->f.sats);   Serial.print(',');
            Serial.print(r->f.fix_q);
        }
        Serial.println();
    }
}
