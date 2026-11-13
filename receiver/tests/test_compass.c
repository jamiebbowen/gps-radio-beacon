/**
 * @file test_compass.c
 * @brief Host-side unit tests for the BNO085/SHTP compass driver
 *        (compass.c + bno085.c).
 *
 * A packet-level SHTP fake sits behind the I2C HAL master functions: the
 * hub queues reset-complete packets, honors SetFeature report enables,
 * and then streams synthetic sensor reports from test-settable knobs
 * (quaternion, per-sensor accuracy, raw accel/gyro/mag). Presence flags,
 * NAK knobs, a reset-kills-hub knob and a stream-stall knob cover the
 * failure modes the driver claims to handle.
 *
 * Covers: I2C scan + dual-address detection, SHTP reset handshake and its
 * timeout paths, feature enable -> fusion stream, heading math with
 * declination + mounting-offset normalization and wrap, the heading_valid
 * calibration latch (RV accuracy), vertical-orientation flagging, stale-
 * heading behavior on comm loss, error-streak recovery, calibrate status
 * messaging, self-test liveness, host-cal not-supported contract, and the
 * accessor/guard functions.
 *
 * Note on test ORDER: matches the BNO055 suite - compass_cal_proven is a
 * function-local static in Compass_Update, so tests that need it clear
 * must run before any test that lets an Update see RV accuracy >= 1.
 * fresh_init() uses rv_acc = 0 precisely for this.
 *
 * Build & run:  make -C receiver/tests
 * Coverage:     make -C receiver/tests coverage
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#include "compass.h"
#include "test_harness.h"

/* ------------------------------------------------------------------ */
/* SHTP packet-level BNO085 fake                                        */
/* ------------------------------------------------------------------ */

#define CH_EXECUTABLE     1
#define CH_CONTROL        2
#define CH_INPUT_REPORTS  3

#define EXE_CMD_RESET     0x01
#define SH2_SET_FEATURE   0xFD

#define RID_ACCEL   0x01
#define RID_GYRO    0x02
#define RID_MAG     0x03
#define RID_RV      0x05

static uint8_t present_28 = 1;
static uint8_t present_29 = 0;
static uint8_t present_50 = 0;      /* a non-IMU device on the bus */
static uint8_t extra_addrs[4];
static uint8_t extra_addr_count = 0;

static uint8_t nak_tx = 0;          /* Master_Transmit NAKs */
static uint8_t nak_rx = 0;          /* Master_Receive NAKs  */
static uint8_t nak_rx_once = 0;     /* NAK exactly one receive, then clear */
static uint8_t reset_dies = 0;      /* hub never confirms reset */
static uint8_t stream_off = 0;      /* features on, but no reports stream */

static uint8_t feat_rv = 0, feat_accel = 0, feat_gyro = 0, feat_mag = 0;

/* Stream content knobs */
static float   fq_w = 1.0f, fq_x = 0.0f, fq_y = 0.0f, fq_z = 0.0f;
static uint8_t rv_acc = 0;          /* SH-2 accuracy bits for the RV report */
static int16_t fa[3] = {0, 0, 0};
static int16_t fg[3] = {0, 0, 0};
static int16_t fm[3] = {0, 0, 0};
static uint8_t acc_acc = 0, gyro_acc = 0, mag_acc = 0;

/* Reset-complete packet queued by the reset command (single slot) */
static uint8_t q_buf[8];
static uint8_t q_len = 0;           /* payload length queued */

/* Current packet being clocked out over two-phase (header/body) reads */
static uint8_t  rpkt[64];           /* full packet incl. 4-byte header */
static uint16_t rpkt_len = 0;
static uint16_t rpkt_pos = 0;

static void fake_reset(void)
{
    present_28 = 1; present_29 = 0; present_50 = 0;
    extra_addr_count = 0;
    nak_tx = nak_rx = 0; nak_rx_once = 0;
    reset_dies = 0; stream_off = 0;
    feat_rv = feat_accel = feat_gyro = feat_mag = 0;
    fq_w = 1.0f; fq_x = fq_y = fq_z = 0.0f;
    rv_acc = 0;
    memset(fa, 0, sizeof(fa)); memset(fg, 0, sizeof(fg)); memset(fm, 0, sizeof(fm));
    acc_acc = gyro_acc = mag_acc = 0;
    q_len = 0; rpkt_len = 0; rpkt_pos = 0;
}

static uint8_t addr_present(uint16_t addr8)
{
    uint8_t a = (uint8_t)(addr8 >> 1);
    if ((a == 0x28 && present_28) || (a == 0x29 && present_29)) return 1;
    if (a == 0x50 && present_50) return 1;
    for (uint8_t i = 0; i < extra_addr_count; i++) {
        if (a == extra_addrs[i]) return 1;
    }
    return 0;
}

/* Build one payload and wrap it into rpkt as a full SHTP packet. */
static void rpkt_build(uint8_t channel, const uint8_t *payload, uint8_t plen)
{
    uint16_t len = (uint16_t)(4 + plen);
    static uint8_t hub_seq = 0;
    rpkt[0] = (uint8_t)(len & 0xFF);
    rpkt[1] = (uint8_t)(len >> 8);
    rpkt[2] = channel;
    rpkt[3] = hub_seq++;
    memcpy(&rpkt[4], payload, plen);
    rpkt_len = len;
    rpkt_pos = 0;
}

static void put_i16(uint8_t *p, int16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

/* Fill rpkt with the next packet the hub would emit, or 0 if none. */
static uint8_t gen_next_packet(void)
{
    if (q_len > 0) {
        uint8_t p[8];
        memcpy(p, q_buf, q_len);
        rpkt_build(CH_EXECUTABLE, p, q_len);
        q_len = 0;
        return 1;
    }
    if (stream_off || reset_dies) return 0;

    /* Rotate through the enabled reports: RV, accel, gyro, mag. */
    static uint8_t phase = 0;
    for (int n = 0; n < 4; n++, phase = (uint8_t)((phase + 1) & 3)) {
        if (phase == 0 && feat_rv) {
            uint8_t p[14];
            p[0] = RID_RV; p[1] = 0; p[2] = rv_acc; p[3] = 0;
            put_i16(&p[4],  (int16_t)(fq_x * 16384.0f));   /* i */
            put_i16(&p[6],  (int16_t)(fq_y * 16384.0f));   /* j */
            put_i16(&p[8],  (int16_t)(fq_z * 16384.0f));   /* k */
            put_i16(&p[10], (int16_t)(fq_w * 16384.0f));   /* real */
            put_i16(&p[12], 0);                            /* accuracy est */
            rpkt_build(CH_INPUT_REPORTS, p, sizeof(p));
            phase = (uint8_t)((phase + 1) & 3);
            return 1;
        }
        if (phase == 1 && feat_accel) {
            uint8_t p[10];
            p[0] = RID_ACCEL; p[1] = 0; p[2] = acc_acc; p[3] = 0;
            put_i16(&p[4], fa[0]); put_i16(&p[6], fa[1]); put_i16(&p[8], fa[2]);
            rpkt_build(CH_INPUT_REPORTS, p, sizeof(p));
            phase = (uint8_t)((phase + 1) & 3);
            return 1;
        }
        if (phase == 2 && feat_gyro) {
            uint8_t p[10];
            p[0] = RID_GYRO; p[1] = 0; p[2] = gyro_acc; p[3] = 0;
            put_i16(&p[4], fg[0]); put_i16(&p[6], fg[1]); put_i16(&p[8], fg[2]);
            rpkt_build(CH_INPUT_REPORTS, p, sizeof(p));
            phase = (uint8_t)((phase + 1) & 3);
            return 1;
        }
        if (phase == 3 && feat_mag) {
            uint8_t p[10];
            p[0] = RID_MAG; p[1] = 0; p[2] = mag_acc; p[3] = 0;
            put_i16(&p[4], fm[0]); put_i16(&p[6], fm[1]); put_i16(&p[8], fm[2]);
            rpkt_build(CH_INPUT_REPORTS, p, sizeof(p));
            phase = (uint8_t)((phase + 1) & 3);
            return 1;
        }
    }
    return 0;
}

/* The driver expects this symbol from the display module's TU */
I2C_HandleTypeDef hi2c_display;

HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c, uint16_t addr,
                                        uint32_t trials, uint32_t timeout)
{
    (void)hi2c; (void)trials; (void)timeout;
    return addr_present(addr) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t addr,
                                          uint8_t *data, uint16_t size, uint32_t timeout)
{
    (void)hi2c; (void)timeout;
    if (!addr_present(addr) || nak_tx) return HAL_ERROR;
    if (size < 5) return HAL_OK;          /* not an SHTP packet - ignore */

    uint8_t channel = data[2];
    const uint8_t *payload = &data[4];
    uint16_t plen = (uint16_t)(size - 4);

    if (channel == CH_EXECUTABLE && plen == 1 && payload[0] == EXE_CMD_RESET) {
        if (!reset_dies) {
            /* Queue the reset-complete response */
            q_buf[0] = EXE_CMD_RESET;     /* response id 0x01 */
            q_len = 1;
        }
        /* A real reset clears feature enables; the hub starts empty. */
        feat_rv = feat_accel = feat_gyro = feat_mag = 0;
    }
    if (channel == CH_CONTROL && plen >= 2 && payload[0] == SH2_SET_FEATURE) {
        switch (payload[1]) {
            case RID_RV:    feat_rv = 1;    break;
            case RID_ACCEL: feat_accel = 1; break;
            case RID_GYRO:  feat_gyro = 1;  break;
            case RID_MAG:   feat_mag = 1;   break;
            default: break;
        }
    }
    return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t addr,
                                         uint8_t *data, uint16_t size, uint32_t timeout)
{
    (void)hi2c; (void)timeout;
    if (!addr_present(addr) || nak_rx) return HAL_ERROR;
    if (nak_rx_once) { nak_rx_once = 0; return HAL_ERROR; }

    if (size == 4) {
        /* New packet request: if the previous one was fully clocked out,
         * generate the next. */
        if (rpkt_pos >= rpkt_len) {
            if (!gen_next_packet()) {
                memset(data, 0xFF, 4);    /* nothing queued */
                return HAL_OK;
            }
        }
        memcpy(data, rpkt, 4);
        rpkt_pos = 4;
        return HAL_OK;
    }

    /* Continuation read: serve the rest of the current packet */
    if (rpkt_pos < rpkt_len) {
        uint16_t avail = (uint16_t)(rpkt_len - rpkt_pos);
        uint16_t n = (size < avail) ? size : avail;
        memcpy(data, &rpkt[rpkt_pos], n);
        rpkt_pos = (uint16_t)(rpkt_pos + n);
        if (size > n) memset(data + n, 0xFF, size - n);
        return HAL_OK;
    }

    memset(data, 0xFF, size);
    return HAL_OK;
}

/* ------------------------------------------------------------------ */
/* Display fakes (capture the scan screen's text lines)                */
/* ------------------------------------------------------------------ */

static char disp_lines[8][32];
static int  disp_nlines = 0;

static void capture_line(int16_t y, const char *text)
{
    if (disp_nlines < 8) {
        snprintf(disp_lines[disp_nlines], sizeof(disp_lines[0]),
                 "y%d:%s", (int)y, text);
        disp_nlines++;
    }
}
void Display_Clear(void) { disp_nlines = 0; }
void Display_DrawText(uint8_t x, uint8_t y, const char *text)
{
    (void)x; capture_line(y, text);
}
void Display_Update(void) {}

/* ------------------------------------------------------------------ */
/* Misc HAL                                                            */
/* ------------------------------------------------------------------ */

void HAL_Delay(uint32_t ms) { Test_SetTick(HAL_GetTick() + ms); }

/* Include the module under test (statics reachable: compass_cal_proven via
 * rub of ordering, compass_cal_restored, heading_offset, etc.) */
#include "../firmware/src/compass.c"

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

static void set_quat(float w, float x, float y, float z)
{
    fq_w = w; fq_x = x; fq_y = y; fq_z = z;
}

static void fresh_init(void)
{
    fake_reset();
    /* Reset cross-test statics via direct access */
    compass_cal_restored = 0;
    heading_offset = 0.0f;
    decl_location_applied = 0;
    quat_conv_locked = 0;
    compass_i2c_error_streak = 0;
    compass_last_recover_ms = 0;
    compass_debug.error_code = 0;      /* error channel is latched: clear per test */
    compass_debug.error_message[0] = '\0';
    if (Compass_Init() != COMPASS_OK) CHECK(0);
}

/* ------------------------------------------------------------------ */
/* Init                                                                */
/* ------------------------------------------------------------------ */

TEST(test_init_success_default_address)
{
    fresh_init();
    CHECK(compass_debug.active_addr == 0x28);
    CHECK(quat_conv_locked == COMPASS_BNO085_DEFAULT_CONV);
    CHECK(Compass_GetFoundDeviceCount() >= 1);
    /* declination 8.5 + mounting 90.0 */
    CHECK_NEAR(heading_offset, 98.5, 0.01);
    CHECK(strstr(Compass_GetDebugMessageByIndex(2), "fusion running") != NULL);
    CHECK(Compass_GetRvSampleCount() >= 1);
}

TEST(test_init_alternate_address)
{
    fake_reset();
    present_28 = 0;
    present_29 = 1;
    compass_cal_restored = 0;
    CHECK(Compass_Init() == COMPASS_OK);
    CHECK(compass_debug.active_addr == 0x29);
}

TEST(test_init_no_devices)
{
    fake_reset();
    present_28 = 0;
    present_29 = 0;
    CHECK(Compass_Init() == COMPASS_ERROR);
    CHECK(Compass_GetLastErrorCode() == COMPASS_ERROR_I2C_INIT);
    CHECK(strstr(Compass_GetErrorMessage(), "No I2C devices") != NULL);
}

TEST(test_init_device_at_wrong_address_times_out)
{
    /* Bus has a device, but not at either strap address: detection must
     * time out with a distinct error (vs the empty-bus bail) */
    fake_reset();
    present_28 = 0; present_29 = 0; present_50 = 1;
    CHECK(Compass_Init() == COMPASS_ERROR);
    CHECK(Compass_GetErrorCode() == COMPASS_ERROR_I2C_INIT);
    CHECK(strstr(Compass_GetErrorMessage(), "not found after timeout") != NULL);
}

TEST(test_init_hub_never_confirms_reset)
{
    /* Hub ACKs and probes OK, but the reset-complete packet never comes */
    fake_reset();
    reset_dies = 1;
    compass_cal_restored = 0;
    CHECK(Compass_Init() == COMPASS_ERROR);
    CHECK(Compass_GetErrorCode() == COMPASS_ERROR_ID_CHECK);
    CHECK(strstr(Compass_GetErrorMessage(), "no fusion data") != NULL);
}

TEST(test_init_no_fusion_stream)
{
    /* Handshake completes, features accepted, but no rotation-vector data
     * ever arrives -> init must fail noisily, not 'succeed' silently. */
    fake_reset();
    stream_off = 1;
    compass_cal_restored = 0;
    Test_SetTickAutoAdvance(5);   /* Diagnostic loop spins on Poll() + tick */
    CHECK(Compass_Init() == COMPASS_ERROR);
    Test_SetTickAutoAdvance(0);
    CHECK(Compass_GetErrorCode() == COMPASS_ERROR_ID_CHECK);
    CHECK(strstr(Compass_GetErrorMessage(), "no fusion data") != NULL);
}

TEST(test_init_tx_failure)
{
    /* Every wire transaction NAKs from the start: SHTP handshake fails */
    fake_reset();
    nak_tx = 1;
    compass_cal_restored = 0;
    CHECK(Compass_Init() == COMPASS_ERROR);
    CHECK(Compass_GetErrorCode() == COMPASS_ERROR_I2C_INIT);
    nak_tx = 0;
}

/* ------------------------------------------------------------------ */
/* Update / heading math                                               */
/* ------------------------------------------------------------------ */

TEST(test_update_heading_offset_and_wrap)
{
    fresh_init();
    Compass_Data d = {0};

    /* ENU conv 5: raw heading = 90 - yaw.  Raw 300 deg = yaw 150 deg:
     * quat w=cos(75), z=sin(75).  300 + 98.5 offset -> wraps to 38.5 */
    set_quat(0.2588f, 0.0f, 0.0f, 0.9659f);
    CHECK(Compass_Update(&d) == COMPASS_OK);
    CHECK_NEAR(d.heading, 38.5, 1.5);
    CHECK_NEAR(d.roll, 0.0, 1.0);
    CHECK_NEAR(d.pitch, 0.0, 1.0);

    /* Negative wrap: offset override makes a small heading go negative.
     * Raw 10 deg = yaw 80 deg: w=cos(40), z=sin(40). */
    Compass_SetHeadingOffset(-200.0f);
    set_quat(0.7660f, 0.0f, 0.0f, 0.6428f);
    CHECK(Compass_Update(&d) == COMPASS_OK);
    CHECK_NEAR(d.heading, 170.0, 1.5);

    Compass_SetHeadingOffset(98.5f);   /* restore */
}

TEST(test_set_declination_overrides_site)
{
    fresh_init();
    /* Boot default: Denver declination 8.5 E + mounting 90 = 98.5 total */
    CHECK_NEAR(heading_offset, 98.5, 0.01);

    /* New site via SD DECLIN.TXT path: e.g. Nevada 11.9 E -> total 101.9 */
    Compass_SetDeclination(11.9f);
    CHECK_NEAR(heading_offset, 101.9, 0.01);

    Compass_Data d = {0};
    set_quat(0.2588f, 0.0f, 0.0f, 0.9659f);   /* raw 300 deg */
    CHECK(Compass_Update(&d) == COMPASS_OK);
    CHECK_NEAR(d.heading, 41.9, 1.5);   /* 300 + 101.9 -> wraps */

    Compass_SetDeclination(8.5f);        /* restore Denver */
    CHECK_NEAR(heading_offset, 98.5, 0.01);
}

TEST(test_location_table_and_precedence)
{
    fresh_init();
    /* Denver neighborhood (39.89N -105.1W) -> bilinear grid ~= +8.5 E */
    float decl = Compass_DeclinationFromLocation(39.89f, -105.1f);
    CHECK(decl > 8.0f && decl < 9.2f);

    /* Mid-Nevada / Reno area (39.5N -119.8W) -> about +13.4 */
    decl = Compass_DeclinationFromLocation(39.5f, -119.8f);
    CHECK(decl > 12.8f && decl < 14.2f);

    /* South Texas (29.4N -98.5W) interpolates to ~ +5.5 E */
    decl = Compass_DeclinationFromLocation(29.4f, -98.5f);
    CHECK(decl > 4.2f && decl < 6.6f);

    /* Edge clamp: outside-grid points saturate at the corner value. */
    decl = Compass_DeclinationFromLocation(50.0f, -130.0f);
    CHECK(decl == declin_grid[4][0]);     /* NW corner: +15.6 */

    /* Cross-state session: first credible fix overrides the Denver default
     * once, and later GPS updates do NOT pivot the arrow mid-flight. */
    Compass_UpdateLocation(39.89f, -105.1f);   /* home base (Denver) */
    CHECK_NEAR(heading_offset, 98.5f + (Compass_DeclinationFromLocation(39.89f, -105.1f) - 8.5f), 0.01);
    Compass_UpdateLocation(39.5f, -119.8f);   /* rocket flew to NV */
    CHECK_NEAR(heading_offset, 98.5f + (Compass_DeclinationFromLocation(39.89f, -105.1f) - 8.5f), 0.01);

    Compass_SetDeclination(8.5f);             /* restore */
}

TEST(test_location_update_rejects_impossible_fix)
{
    fresh_init();
    CHECK(Compass_UpdateLocation(0.0f, 0.0f) != COMPASS_OK);
    CHECK_NEAR(heading_offset, 98.5f, 0.01);  /* Denver default intact */
}

TEST(test_update_raw_sensor_stream)
{
    fresh_init();
    Compass_Data d = {0};

    /* Push distinct raw values through the three raw channels */
    const int16_t av[3] = {256, 512, -256};       /* 1.0, 2.0, -1.0 m/s2 */
    const int16_t gv[3] = {512, -512, 1024};      /* 1.0, -1.0, 2.0 rad/s */
    const int16_t mv[3] = {400, -160, 80};        /* 25.0, -10.0, 5.0 uT */
    memcpy(fa, av, sizeof(fa));
    memcpy(fg, gv, sizeof(fg));
    memcpy(fm, mv, sizeof(fm));

    /* Drain enough phases to pass the RV gate and see all channels */
    for (int i = 0; i < 6; i++) {
        Test_SetTick(HAL_GetTick() + 50);
        CHECK(Compass_Update(&d) == COMPASS_OK);
    }

    CHECK(d.accel_x == 256 && d.accel_z == -256);
    CHECK(d.mag_x == 400 && d.mag_y == -160);
    CHECK(d.gyro_z == 1024);
    CHECK_NEAR(d.cal_accel_y, 2.0, 0.01);     /* 512 / 256 */
    CHECK_NEAR(d.cal_gyro_x, 1.0, 0.01);      /* 512 / 512 */
    CHECK_NEAR(d.cal_accel_z, -1.0, 0.01);
}

TEST(test_heading_valid_latch)
{
    fresh_init();
    /* NOTE: compass_cal_proven is a function-local static in
     * Compass_Update (unreachable from here), so this test depends on
     * ordering: nothing before it may raise RV accuracy >= 1. */

    Compass_Data d = {0};
    rv_acc = 0;
    CHECK(Compass_Update(&d) == COMPASS_OK);
    CHECK(d.heading_valid == 0);
    CHECK(d.mag_cal == 0);

    /* RV accuracy reaches 1: valid latches on... */
    rv_acc = 1;
    CHECK(Compass_Update(&d) == COMPASS_OK);
    CHECK(d.heading_valid == 1);
    CHECK(d.mag_cal == 1);

    /* ...and STAYS on when the live level decays back to 0 */
    rv_acc = 0;
    CHECK(Compass_Update(&d) == COMPASS_OK);
    CHECK(d.heading_valid == 1);           /* latched */
    CHECK(d.mag_cal == 0);                 /* live level still visible */
}

TEST(test_heading_valid_via_restored_cal_flag)
{
    fresh_init();
    /* Runs while the proven latch is still clear, so validity can only
     * come from the restored-calibration flag (test static poke). */
    compass_cal_restored = 1;
    Compass_Data d = {0};
    rv_acc = 0;
    CHECK(Compass_Update(&d) == COMPASS_OK);
    CHECK(d.heading_valid == 1);
    compass_cal_restored = 0;
}

/* ------------------------------------------------------------------ */
/* Orientation / conventions                                           */
/* ------------------------------------------------------------------ */

TEST(test_vertical_orientation_invalid)
{
    fresh_init();
    compass_cal_restored = 1;              /* calibrated, so cal is NOT why */
    Compass_Data d = {0};

    /* 90 deg about Y: device +X points straight up - heading undefined */
    set_quat(0.7071f, 0.0f, 0.7071f, 0.0f);
    CHECK(Compass_Update(&d) == COMPASS_OK);
    CHECK(d.orientation_valid == 0);
    CHECK(d.heading_valid == 0);           /* UI shows VERT, not CAL */

    /* Level again: orientation recovers immediately */
    set_quat(1.0f, 0.0f, 0.0f, 0.0f);
    CHECK(Compass_Update(&d) == COMPASS_OK);
    CHECK(d.orientation_valid == 1);
    CHECK(d.heading_valid == 1);
    compass_cal_restored = 0;
}

TEST(test_quat_lock_get_set_for_persistence)
{
    fresh_init();
    quat_conv_locked = 0;
    CHECK(Compass_GetQuatLockConv() == 0);
    CHECK(Compass_SetQuatLockConv(0) == COMPASS_ERROR);   /* 0 = not a conv */
    CHECK(Compass_SetQuatLockConv(9) == COMPASS_ERROR);
    CHECK(Compass_SetQuatLockConv(5) == COMPASS_OK);
    CHECK(Compass_GetQuatLockConv() == 5);
    CHECK(quat_conv_locked == 5);                          /* same static drives heading */
}

TEST(test_fusion_stall_flagged)
{
    fresh_init();
    compass_cal_restored = 1;
    Compass_Data d = {0};

    /* Streamed fine... */
    CHECK(Compass_Update(&d) == COMPASS_OK);
    CHECK(d.heading_stale == 0);

    /* ...then the fusion pipeline stalls while the bus stays healthy */
    stream_off = 1;
    Test_SetTick(HAL_GetTick() + 800);    /* past the RV stall gate */
    CHECK(Compass_Update(&d) == COMPASS_ERROR);
    CHECK(strstr(Compass_GetErrorMessage(), "fusion stalled") != NULL);
    CHECK(d.heading_valid == 0);
    CHECK(d.heading_stale == 1);
    stream_off = 0;
    compass_cal_restored = 0;
}

TEST(test_i2c_error_streak_recovers)
{
    fresh_init();
    Compass_Data d = {0};

    /* The report reads NAK (cable noise): 12 polls at the 5 Hz cadence */
    nak_rx = 1;
    for (int i = 0; i < 12; i++) {
        Test_SetTick(HAL_GetTick() + 200);
        CHECK(Compass_Update(&d) == COMPASS_ERROR);
        CHECK(d.heading_valid == 0);
        CHECK(d.heading_stale == 1);   /* cal was proven in an earlier test */
    }

    /* One more single-shot NAK with the wire otherwise healthy: the poll
     * errors (streak hot, interval gate open) so recovery runs - and the
     * recovery itself succeeds (probe + soft reset + re-feature). The
     * update still reports the original fault, but the message is set. */
    nak_rx = 0;
    nak_rx_once = 1;
    Test_SetTick(HAL_GetTick() + 6000);   /* pass the recover interval gate */
    CHECK(Compass_Update(&d) == COMPASS_ERROR);  /* recovery attempt runs */
    CHECK(strstr(Compass_GetDebugMessageByIndex(2), "recovered") != NULL);
    /* Hub streams again */
    Test_SetTick(HAL_GetTick() + 200);
    CHECK(Compass_Update(&d) == COMPASS_OK);
}

TEST(test_stale_heading_flagged_on_comm_loss)
{
    fresh_init();
    compass_cal_restored = 1;              /* trustworthy heading exists */
    Compass_Data d = {0};
    set_quat(0.7071f, 0.0f, 0.0f, 0.7071f);    /* yaw 90 -> raw 0 deg */

    CHECK(Compass_Update(&d) == COMPASS_OK);
    CHECK(d.heading_valid == 1);
    CHECK(d.heading_stale == 0);
    const float live_heading = d.heading;

    nak_rx = 1;
    CHECK(Compass_Update(&d) == COMPASS_ERROR);
    CHECK(d.heading_valid == 0);
    CHECK(d.heading_stale == 1);
    CHECK_NEAR(d.heading, live_heading, 0.01f);   /* value retained */

    nak_rx = 0;
    Test_SetTick(HAL_GetTick() + 200);
    CHECK(Compass_Update(&d) == COMPASS_OK);
    CHECK(d.heading_valid == 1);
    CHECK(d.heading_stale == 0);
    compass_cal_restored = 0;
}

/* ------------------------------------------------------------------ */
/* Calibrate / self-test                                               */
/* ------------------------------------------------------------------ */

TEST(test_calibrate_fully_calibrated)
{
    fresh_init();
    rv_acc = 3; acc_acc = 3; gyro_acc = 3;
    Compass_Data d = {0};
    for (int i = 0; i < 4; i++) {          /* let all streams arrive */
        Test_SetTick(HAL_GetTick() + 50);
        CHECK(Compass_Update(&d) == COMPASS_OK);
    }
    CHECK(Compass_Calibrate() == COMPASS_OK);
    CHECK(strstr(Compass_GetDebugMessageByIndex(0), "SYS=3") != NULL);

    uint8_t s, g, a, m;
    Compass_GetCalibrationStatus(&s, &g, &a, &m);
    CHECK(s == 3 && g == 3 && a == 3 && m == 3);
    Compass_GetCalibrationStatus(NULL, NULL, NULL, NULL);  /* null-safe */
}

TEST(test_calibrate_partial_guidance)
{
    fresh_init();
    Compass_Data d = {0};
    rv_acc = 1; gyro_acc = 0; acc_acc = 0;
    Test_SetTick(HAL_GetTick() + 50);
    CHECK(Compass_Update(&d) == COMPASS_OK);

    CHECK(Compass_Calibrate() == COMPASS_ERROR);
    CHECK(strstr(Compass_GetDebugMessageByIndex(3), "MAG CAL") != NULL);
    CHECK(strstr(Compass_GetDebugMessageByIndex(6), "GYR CAL") != NULL);
    CHECK(strstr(Compass_GetDebugMessageByIndex(7), "ACC CAL") != NULL);
}

TEST(test_selftest)
{
    fresh_init();
    CHECK(Compass_SelfTest() == COMPASS_OK);   /* alive + fusion flowing */

    present_28 = 0;                      /* hub gone */
    CHECK(Compass_SelfTest() == COMPASS_ERROR);
    CHECK(Compass_GetErrorCode() == COMPASS_ERROR_I2C_INIT);
    present_28 = 1;
}

TEST(test_selftest_error_paths)
{
    /* Init with the fusion stream off fails, but not before memset'ing
     * s_bno - which leaves rv_count = 0 for the SelfTest check below.
     * (rv_count is cumulative, so this is the only deterministic route
     * to "alive but no fusion data" once any init has succeeded.) */
    fake_reset();
    stream_off = 1;
    compass_cal_restored = 0;
    Test_SetTickAutoAdvance(5);
    (void)Compass_Init();
    Test_SetTickAutoAdvance(0);

    /* Hub answers the probe and polls clean, yet no RVs were counted. */
    CHECK(Compass_SelfTest() == COMPASS_ERROR);
    CHECK(Compass_GetErrorCode() == COMPASS_ERROR_ID_CHECK);

    /* SHTP reads NAK while the address probe ACKs: read-data fault. */
    stream_off = 0;
    nak_rx = 1;
    CHECK(Compass_SelfTest() == COMPASS_ERROR);
    CHECK(Compass_GetErrorCode() == COMPASS_ERROR_READ_DATA);
    nak_rx = 0;
}

TEST(test_calibrate_mag_guidance_variants)
{
    /* The mag_cal digit tracks RV accuracy (SH-2 reports mag cal on the
     * RV report's accuracy byte). Drive the 0/3 and 2/3 guidance rows
     * the partial-calibration test skips. */
    fresh_init();
    Compass_Data d = {0};

    rv_acc = 0;
    for (int i = 0; i < 4; i++) {
        Test_SetTick(HAL_GetTick() + 50);
        (void)Compass_Update(&d);
    }
    CHECK(Compass_Calibrate() == COMPASS_ERROR);
    CHECK(strstr(Compass_GetDebugMessageByIndex(3), "figure 8") != NULL);

    rv_acc = 2;
    for (int i = 0; i < 4; i++) {
        Test_SetTick(HAL_GetTick() + 50);
        (void)Compass_Update(&d);
    }
    CHECK(Compass_Calibrate() == COMPASS_ERROR);
    CHECK(strstr(Compass_GetDebugMessageByIndex(3), "almost done") != NULL);
}

TEST(test_debug_getters_and_heading_failure)
{
    fresh_init();
    Compass_Data d = {0};
    Test_SetTick(HAL_GetTick() + 50);
    CHECK(Compass_Update(&d) == COMPASS_OK);

    /* debug message 0 carries the Cal: line; timestamp advances on update */
    CHECK(strstr(Compass_GetDebugMessage(), "Cal:") != NULL);
    CHECK(Compass_GetDebugTimestamp() == HAL_GetTick());

    /* Dead bus: GetHeading falls back to 0.0f */
    present_28 = 0;
    CHECK_NEAR(Compass_GetHeading(), 0.0f, 0.001);
    present_28 = 1;
}

TEST(test_display_i2c_scan_multi_device)
{
    fresh_init();

    /* Five devices on the bus exercises both display rows, the
     * devices_found>1 ternary, and all four debug address slots. */
    extra_addrs[0] = 0x40; extra_addrs[1] = 0x41;
    extra_addrs[2] = 0x42; extra_addrs[3] = 0x60;
    extra_addr_count = 4;

    disp_nlines = 0;
    CHECK(Compass_DisplayI2CScan() == 5);
    int saw_line3 = 0;
    for (int i = 0; i < disp_nlines; i++) {
        if (strstr(disp_lines[i], "0x60") != NULL) saw_line3 = 1;
    }
    CHECK(saw_line3);
    extra_addr_count = 0;
}

/* ------------------------------------------------------------------ */
/* Host calibration offsets: NOT SUPPORTED on the BNO085               */
/* ------------------------------------------------------------------ */

TEST(test_cal_data_unsupported_contract)
{
    fresh_init();
    uint8_t buf[BNO055_CAL_DATA_LEN];

    CHECK(Compass_HasHostCalOffsets() == 0);
    CHECK(Compass_GetCalibrationData(buf, sizeof(buf)) == COMPASS_ERROR);
    CHECK(Compass_SetCalibrationData(buf, sizeof(buf)) == COMPASS_ERROR);
    CHECK(Compass_WasCalRestored() == 0);
    CHECK(strstr(Compass_GetDebugMessageByIndex(3), "BNO085") != NULL);
}

/* ------------------------------------------------------------------ */
/* Accessors / misc                                                    */
/* ------------------------------------------------------------------ */

TEST(test_accessors)
{
    fresh_init();

    uint8_t st, self, err, mode, pwr;
    int8_t temp = -99;
    CHECK(Compass_GetSystemStatus(&st, &self, &err) == COMPASS_OK);
    CHECK(st == 5 && self == 0x0F && err == 0);   /* streaming, alive */
    CHECK(Compass_GetOperationMode(&mode) == COMPASS_OK && mode == 0x0C);
    CHECK(Compass_GetPowerMode(&pwr) == COMPASS_OK && pwr == 0);
    /* No temperature register on the BNO085: must fail, not fabricate */
    CHECK(Compass_GetTemperature(&temp) == COMPASS_ERROR);
    CHECK(Compass_GetOperationMode(NULL) == COMPASS_ERROR);
    CHECK(Compass_GetPowerMode(NULL) == COMPASS_ERROR);

    uint8_t addr = 0, raw[6], sreg = 0, cal = 0;
    Compass_GetDebugInfo(&addr, raw, &sreg, &cal);
    CHECK(addr == 0x28);
    Compass_GetDebugInfo(NULL, NULL, NULL, NULL);      /* null-safe */
    CHECK(Compass_GetDebugMessageByIndex(99)[0] == '\0');

    /* GetHeading returns the corrected value: identity quat = raw 90 deg
     * (conv 5: 90 - yaw=0) + 98.5 offset = 188.5 */
    set_quat(1.0f, 0.0f, 0.0f, 0.0f);
    CHECK_NEAR(Compass_GetHeading(), 188.5, 1.5);

    const uint8_t *found = Compass_GetFoundAddresses();
    CHECK(found != NULL);
}

TEST(test_display_i2c_scan)
{
    fresh_init();

    disp_nlines = 0;
    uint8_t count = Compass_DisplayI2CScan();
    CHECK(count >= 1);
    int saw_found = 0;
    for (int i = 0; i < disp_nlines; i++) {
        if (strstr(disp_lines[i], "Found")) saw_found = 1;
    }
    CHECK(saw_found);

    /* Empty bus: "No devices found" screen */
    present_28 = 0; present_29 = 0;
    disp_nlines = 0;
    CHECK(Compass_DisplayI2CScan() == 0);
    int saw_none = 0;
    for (int i = 0; i < disp_nlines; i++) {
        if (strstr(disp_lines[i], "No devices")) saw_none = 1;
    }
    CHECK(saw_none);
    present_28 = 1;
}

TEST(test_recover_failure_paths)
{
    fresh_init();
    Compass_Data d = {0};

    /* Hub disappears before the streak builds: the 10th consecutive
     * failure triggers a recovery attempt whose IsAlive probe fails (the
     * attempt interval gate is wide open right after init). */
    nak_rx = 1;
    present_28 = 0;
    for (int i = 0; i < 10; i++) {
        Test_SetTick(HAL_GetTick() + 200);
        (void)Compass_Update(&d);
    }

    /* Hub back, fault cleared: recovery succeeds, streaming resumes */
    present_28 = 1;
    nak_rx = 0;
    Test_SetTick(HAL_GetTick() + 6000);
    (void)Compass_Update(&d);                   /* streak still hot: recovers */
    Test_SetTick(HAL_GetTick() + 200);
    CHECK(Compass_Update(&d) == COMPASS_OK);
}

TEST(test_quat_heading_candidate_all_conventions)
{
    /* Every conversion in the lock-search table: yaw = atan2(vy,vx) with
     * vx=1,vy=0 => yaw=0. The table must cover 1..8 and stay normalized. */
    CHECK_NEAR(Compass_QuatHeadingCandidate(1, 1.0f, 0.0f),   0.0f, 0.01);
    CHECK_NEAR(Compass_QuatHeadingCandidate(2, 1.0f, 0.0f),   0.0f, 0.01);
    CHECK_NEAR(Compass_QuatHeadingCandidate(3, 1.0f, 0.0f), 180.0f, 0.01);
    CHECK_NEAR(Compass_QuatHeadingCandidate(4, 1.0f, 0.0f), 180.0f, 0.01);
    CHECK_NEAR(Compass_QuatHeadingCandidate(5, 1.0f, 0.0f),  90.0f, 0.01);
    CHECK_NEAR(Compass_QuatHeadingCandidate(6, 1.0f, 0.0f),  90.0f, 0.01);
    CHECK_NEAR(Compass_QuatHeadingCandidate(7, 1.0f, 0.0f), 270.0f, 0.01);
    CHECK_NEAR(Compass_QuatHeadingCandidate(8, 1.0f, 0.0f), 270.0f, 0.01);

    /* yaw=+90 (vx=0,vy=1): verify each formula across the full range */
    CHECK_NEAR(Compass_QuatHeadingCandidate(1, 0.0f, 1.0f),  90.0f, 0.01);
    CHECK_NEAR(Compass_QuatHeadingCandidate(2, 0.0f, 1.0f), 270.0f, 0.01);
    CHECK_NEAR(Compass_QuatHeadingCandidate(3, 0.0f, 1.0f), 270.0f, 0.01);
    CHECK_NEAR(Compass_QuatHeadingCandidate(4, 0.0f, 1.0f),  90.0f, 0.01);
    CHECK_NEAR(Compass_QuatHeadingCandidate(5, 0.0f, 1.0f),   0.0f, 0.01);
    CHECK_NEAR(Compass_QuatHeadingCandidate(6, 0.0f, 1.0f), 180.0f, 0.01);
    CHECK_NEAR(Compass_QuatHeadingCandidate(7, 0.0f, 1.0f), 180.0f, 0.01);
    CHECK_NEAR(Compass_QuatHeadingCandidate(8, 0.0f, 1.0f),   0.0f, 0.01);

    /* Out-of-range index falls to the default branch (conv 8 formula) */
    CHECK_NEAR(Compass_QuatHeadingCandidate(0, 0.0f, 1.0f),  0.0f, 0.01);
}

int main(void)
{
    run_test_quat_heading_candidate_all_conventions();
    run_test_init_success_default_address();
    run_test_init_alternate_address();
    run_test_init_no_devices();
    run_test_init_device_at_wrong_address_times_out();
    run_test_init_hub_never_confirms_reset();
    run_test_init_no_fusion_stream();
    run_test_init_tx_failure();

    run_test_update_heading_offset_and_wrap();
    run_test_set_declination_overrides_site();
    run_test_location_table_and_precedence();
    run_test_location_update_rejects_impossible_fix();
    run_test_update_raw_sensor_stream();

    run_test_heading_valid_via_restored_cal_flag();
    run_test_heading_valid_latch();

    run_test_vertical_orientation_invalid();
    run_test_quat_lock_get_set_for_persistence();
    run_test_fusion_stall_flagged();
    run_test_i2c_error_streak_recovers();
    run_test_stale_heading_flagged_on_comm_loss();

    run_test_calibrate_fully_calibrated();
    run_test_calibrate_partial_guidance();
    run_test_calibrate_mag_guidance_variants();
    run_test_selftest();
    run_test_selftest_error_paths();
    run_test_cal_data_unsupported_contract();
    run_test_accessors();
    run_test_debug_getters_and_heading_failure();
    run_test_display_i2c_scan();
    run_test_display_i2c_scan_multi_device();
    run_test_recover_failure_paths();

    return TEST_SUMMARY();
}
