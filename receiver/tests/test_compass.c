/**
 * @file test_compass.c
 * @brief Host-side unit tests for the BNO055 compass driver (compass.c).
 *
 * A register-level BNO055 fake sits behind the I2C HAL: a 256-byte
 * register file with presence flags per 7-bit address, NAK knobs for
 * every transfer type, and a calibration-read corruptor for the
 * restore-verify path. Display_* functions are faked with line capture
 * so Compass_DisplayI2CScan() output is assertable.
 *
 * Covers: I2C scan + dual-address detection, chip-ID checks and reset
 * recovery in init, NDOF verification, heading math with declination +
 * mounting offset normalization, the heading_valid calibration latch,
 * periodic calib/mode/sys-status checks with recovery writes, calibration
 * save/restore with read-back verification, self-test, calibrate status
 * messaging, and the accessor/guard functions.
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
/* BNO055 register fake                                                */
/* ------------------------------------------------------------------ */

#define REG_CHIP_ID    0x00
#define REG_EULER_H    0x1A
#define REG_ACCEL_X    0x08
#define REG_TEMP       0x34
#define REG_CALIB_STAT 0x35
#define REG_ST_RESULT  0x36
#define REG_SYS_STAT   0x39
#define REG_SYS_ERR    0x3A
#define REG_OPR_MODE   0x3D
#define REG_PWR_MODE   0x3E
#define REG_CAL_DATA   0x55

static uint8_t bno_regs[256];
static uint8_t present_28 = 1;
static uint8_t present_29 = 0;
static uint8_t nak_mem_read    = 0;   /* HAL_I2C_Mem_Read NAKs        */
static uint8_t nak_mem_write   = 0;   /* HAL_I2C_Mem_Write NAKs       */
static uint8_t nak_master_tx   = 0;   /* HAL_I2C_Master_Transmit NAKs */
static uint8_t nak_master_rx   = 0;   /* HAL_I2C_Master_Receive NAKs  */
static uint8_t corrupt_cal_read = 0;  /* read-back of cal data lies   */
static uint8_t last_reg_addr    = 0;  /* for Master_Transmit/Receive  */

static void bno_reset(void)
{
    memset(bno_regs, 0, sizeof(bno_regs));
    bno_regs[REG_CHIP_ID]    = 0xA0;
    bno_regs[REG_SYS_STAT]   = 5;      /* fusion running */
    bno_regs[REG_ST_RESULT]  = 0x0F;
    bno_regs[REG_OPR_MODE]   = 0x0C;   /* NDOF (init writes it anyway) */
    present_28 = 1;
    present_29 = 0;
    nak_mem_read = nak_mem_write = nak_master_tx = nak_master_rx = 0;
    corrupt_cal_read = 0;
}

static uint8_t addr_present(uint16_t addr8)
{
    uint8_t a = (uint8_t)(addr8 >> 1);
    return (a == 0x28 && present_28) || (a == 0x29 && present_29);
}

/* The driver expects this symbol from the display module's TU */
I2C_HandleTypeDef hi2c_display;

HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c, uint16_t addr,
                                        uint32_t trials, uint32_t timeout)
{
    (void)hi2c; (void)trials; (void)timeout;
    return addr_present(addr) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t addr,
                                   uint16_t mem_addr, uint16_t mem_size,
                                   uint8_t *data, uint16_t size, uint32_t timeout)
{
    (void)hi2c; (void)mem_size; (void)timeout;
    if (!addr_present(addr) || nak_mem_read) return HAL_ERROR;
    if (mem_addr + size > 256) return HAL_ERROR;
    memcpy(data, &bno_regs[mem_addr], size);
    if (corrupt_cal_read && mem_addr == REG_CAL_DATA) data[0] ^= 0xFF;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t addr,
                                    uint16_t mem_addr, uint16_t mem_size,
                                    uint8_t *data, uint16_t size, uint32_t timeout)
{
    (void)hi2c; (void)mem_size; (void)timeout;
    if (!addr_present(addr) || nak_mem_write) return HAL_ERROR;
    if (mem_addr + size > 256) return HAL_ERROR;
    memcpy(&bno_regs[mem_addr], data, size);
    return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t addr,
                                          uint8_t *data, uint16_t size, uint32_t timeout)
{
    (void)hi2c; (void)timeout;
    if (!addr_present(addr) || nak_master_tx) return HAL_ERROR;
    if (size == 1) {                       /* register-address pointer set */
        last_reg_addr = data[0];
    } else if (size >= 2) {                /* register write */
        bno_regs[data[0]] = data[1];
    }
    return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t addr,
                                         uint8_t *data, uint16_t size, uint32_t timeout)
{
    (void)hi2c; (void)timeout;
    if (!addr_present(addr) || nak_master_rx) return HAL_ERROR;
    for (uint16_t i = 0; i < size; i++) data[i] = bno_regs[last_reg_addr + i];
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

/* Include the module under test (statics reachable: compass_cal_proven,
 * compass_cal_restored, heading_offset, etc.) */
#include "../firmware/src/compass.c"

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

static void set_euler(float heading_deg, float roll_deg, float pitch_deg)
{
    int16_t h = (int16_t)(heading_deg * 16.0f);
    int16_t r = (int16_t)(roll_deg * 16.0f);
    int16_t p = (int16_t)(pitch_deg * 16.0f);
    bno_regs[REG_EULER_H + 0] = (uint8_t)h; bno_regs[REG_EULER_H + 1] = (uint8_t)(h >> 8);
    bno_regs[REG_EULER_H + 2] = (uint8_t)r; bno_regs[REG_EULER_H + 3] = (uint8_t)(r >> 8);
    bno_regs[REG_EULER_H + 4] = (uint8_t)p; bno_regs[REG_EULER_H + 5] = (uint8_t)(p >> 8);
}

static void fresh_init(void)
{
    bno_reset();
    /* Reset cross-test statics via direct access */
    compass_cal_restored = 0;
    heading_offset = 0.0f;
    if (Compass_Init() != COMPASS_OK) CHECK(0);
}

/* ------------------------------------------------------------------ */
/* Init                                                                */
/* ------------------------------------------------------------------ */

TEST(test_init_success_default_address)
{
    fresh_init();
    CHECK(compass_debug.active_addr == 0x28);
    CHECK(bno_regs[REG_OPR_MODE] == BNO055_OPR_MODE_NDOF);
    CHECK(bno_regs[BNO055_AXIS_MAP_CONFIG] == 0x24);
    CHECK(bno_regs[BNO055_AXIS_MAP_SIGN] == 0x00);
    CHECK(Compass_GetFoundDeviceCount() >= 1);
    /* declination 8.5 + mounting 90.0 */
    CHECK_NEAR(heading_offset, 98.5, 0.01);
    /* Slot 1 ends holding the system-status line; slot 2 the fusion one */
    CHECK(strstr(Compass_GetDebugMessageByIndex(1), "system status") != NULL);
    CHECK(strstr(Compass_GetDebugMessageByIndex(2), "running with fusion") != NULL);
}

TEST(test_init_alternate_address)
{
    bno_reset();
    present_28 = 0;
    present_29 = 1;
    compass_cal_restored = 0;
    CHECK(Compass_Init() == COMPASS_OK);
    CHECK(compass_debug.active_addr == 0x29);
}

TEST(test_init_no_devices)
{
    bno_reset();
    present_28 = 0;
    present_29 = 0;
    CHECK(Compass_Init() == COMPASS_ERROR);
    CHECK(Compass_GetLastErrorCode() == COMPASS_ERROR_I2C_INIT);
    CHECK(strstr(Compass_GetErrorMessage(), "No I2C devices") != NULL);
}

TEST(test_init_wrong_chip_id)
{
    bno_reset();
    bno_regs[REG_CHIP_ID] = 0x55;
    CHECK(Compass_Init() == COMPASS_ERROR);
    CHECK(Compass_GetErrorCode() == COMPASS_ERROR_ID_CHECK);
}

TEST(test_init_survives_nak_writes_and_bad_status)
{
    /* Every WriteRegister NAKs: init must press on ("continue anyway"),
     * then the NDOF verify fails (mode reads back CONFIG) and the
     * system-status check logs the not-running branch. */
    bno_reset();
    nak_mem_write = 1;                 /* Mem_Write (cal/NDOF verify path) */
    nak_master_tx = 1;                 /* WriteRegister (2-byte master tx) */
    bno_regs[REG_OPR_MODE] = 0x00;     /* stuck in CONFIG */
    bno_regs[REG_SYS_STAT] = 1;        /* SYS_STATUS_ERROR -> reads SYS_ERR */
    bno_regs[REG_SYS_ERR] = 0x0A;
    compass_cal_restored = 0;
    CHECK(Compass_Init() == COMPASS_OK);   /* degraded but returns OK */
    nak_master_tx = nak_mem_write = 0;
}

/* ------------------------------------------------------------------ */
/* Update / heading math                                               */
/* ------------------------------------------------------------------ */

TEST(test_update_heading_offset_and_wrap)
{
    fresh_init();
    Compass_Data d = {0};

    /* Raw 300 deg + 98.5 offset = 398.5 -> wraps to 38.5 */
    set_euler(300.0f, -45.0f, 22.5f);
    CHECK(Compass_Update(&d) == COMPASS_OK);
    CHECK_NEAR(d.heading, 38.5, 0.15);
    CHECK_NEAR(d.roll, -45.0, 0.15);
    CHECK_NEAR(d.pitch, 22.5, 0.15);

    /* Negative wrap: offset override makes a small heading go negative */
    Compass_SetHeadingOffset(-200.0f);
    set_euler(10.0f, 0.0f, 0.0f);
    CHECK(Compass_Update(&d) == COMPASS_OK);
    CHECK_NEAR(d.heading, 170.0, 0.15);

    Compass_SetHeadingOffset(98.5f);   /* restore */
}

TEST(test_update_raw_sensor_burst)
{
    fresh_init();
    Compass_Data d = {0};

    /* accel XYZ = 1,2,3 ; mag = -100,200,-300 ; gyro = 160 dps*16 on Z */
    static const int16_t vals[9] = {1, 2, 3, -100, 200, -300, 10, 20, 160};
    for (int i = 0; i < 9; i++) {
        bno_regs[REG_ACCEL_X + i * 2]     = (uint8_t)vals[i];
        bno_regs[REG_ACCEL_X + i * 2 + 1] = (uint8_t)(vals[i] >> 8);
    }
    CHECK(Compass_Update(&d) == COMPASS_OK);
    CHECK(d.accel_x == 1 && d.accel_y == 2 && d.accel_z == 3);
    CHECK(d.mag_x == -100 && d.mag_y == 200 && d.mag_z == -300);
    CHECK(d.gyro_x == 10 && d.gyro_y == 20 && d.gyro_z == 160);
    CHECK_NEAR(d.cal_gyro_z, 10.0, 0.01);    /* 160/16 dps */

    /* Burst read NAK: update still OK, gyro rate zeroed */
    nak_mem_read = 0; /* euler read must work; fail only the burst via size? */
    /* (burst read shares the same path; simulate by NAKing everything and
     *  checking the euler-failure error return instead) */
    nak_mem_read = 1;
    CHECK(Compass_Update(&d) == COMPASS_ERROR);
    CHECK(Compass_GetErrorCode() == COMPASS_ERROR_READ_REG);
    nak_mem_read = 0;
}

TEST(test_heading_valid_latch)
{
    fresh_init();
    /* NOTE: compass_cal_proven is a function-local static in
     * Compass_Update (unreachable from here), so this test depends on
     * ordering: nothing before it may raise mag_cal >= 1. */

    Compass_Data d = {0};
    bno_regs[REG_CALIB_STAT] = 0x00;   /* mag_cal = 0 */
    CHECK(Compass_Update(&d) == COMPASS_OK);
    CHECK(d.heading_valid == 0);
    CHECK(d.mag_cal == 0);

    /* mag_cal reaches 1: valid latches on... */
    bno_regs[REG_CALIB_STAT] = 0x01;
    Test_SetTick(HAL_GetTick() + 1100);    /* pass the 1 s calib poll */
    CHECK(Compass_Update(&d) == COMPASS_OK);
    CHECK(d.heading_valid == 1);
    CHECK(d.mag_cal == 1);

    /* ...and STAYS on when the live level decays back to 0 */
    bno_regs[REG_CALIB_STAT] = 0x00;
    Test_SetTick(HAL_GetTick() + 1100);
    CHECK(Compass_Update(&d) == COMPASS_OK);
    CHECK(d.heading_valid == 1);           /* latched */
    CHECK(d.mag_cal == 0);                 /* live level still visible */
}

TEST(test_heading_valid_via_restored_cal)
{
    fresh_init();
    /* Runs BEFORE the latch test so the proven latch is still clear and
     * validity can only come from the restored-calibration flag. */
    compass_cal_restored = 1;              /* as if SetCalibrationData ran */
    Compass_Data d = {0};
    bno_regs[REG_CALIB_STAT] = 0x00;
    CHECK(Compass_Update(&d) == COMPASS_OK);
    CHECK(d.heading_valid == 1);
    compass_cal_restored = 0;
}

TEST(test_periodic_mode_recovery)
{
    fresh_init();
    Compass_Data d = {0};

    /* Trip the 5 s status check with the chip dropped out of NDOF:
     * the driver must rewrite CONFIG then NDOF. */
    bno_regs[REG_OPR_MODE] = 0x00;
    Test_SetTick(HAL_GetTick() + 5100);
    CHECK(Compass_Update(&d) == COMPASS_OK);
    CHECK(bno_regs[REG_OPR_MODE] == BNO055_OPR_MODE_NDOF);
    CHECK(strstr(Compass_GetDebugMessageByIndex(1), "H") != NULL);
}

/* ------------------------------------------------------------------ */
/* Calibrate / self-test                                               */
/* ------------------------------------------------------------------ */

TEST(test_calibrate_fully_calibrated)
{
    fresh_init();
    bno_regs[REG_CALIB_STAT] = 0xFF;   /* S3 G3 A3 M3 */
    bno_regs[REG_SYS_STAT]   = 5;
    CHECK(Compass_Calibrate() == COMPASS_OK);
    CHECK(strstr(Compass_GetDebugMessageByIndex(3), "fully calibrated") != NULL);
    /* GetCalibrationStatus reads the cached calib_status, refreshed by
     * Compass_Update's 1 s poll - run one past the poll interval. */
    Compass_Data d = {0};
    Test_SetTick(HAL_GetTick() + 1100);
    CHECK(Compass_Update(&d) == COMPASS_OK);
    uint8_t s, g, a, m;
    Compass_GetCalibrationStatus(&s, &g, &a, &m);
    CHECK(s == 3 && g == 3 && a == 3 && m == 3);
    Compass_GetCalibrationStatus(NULL, NULL, NULL, NULL);  /* null-safe */
}

TEST(test_calibrate_partial_guidance)
{
    fresh_init();
    bno_regs[REG_CALIB_STAT] = 0x01;   /* mag_cal=1, others 0 */
    bno_regs[REG_SYS_STAT]   = 6;      /* running without fusion */
    CHECK(Compass_Calibrate() == COMPASS_ERROR_CALIB);
    CHECK(strstr(Compass_GetDebugMessageByIndex(3), "MAG CAL") != NULL);
    CHECK(strstr(Compass_GetDebugMessageByIndex(6), "GYR CAL") != NULL);
    CHECK(strstr(Compass_GetDebugMessageByIndex(7), "ACC CAL") != NULL);

    /* mag done, others not */
    bno_regs[REG_CALIB_STAT] = 0x03;
    CHECK(Compass_Calibrate() == COMPASS_ERROR_CALIB);
    CHECK(strstr(Compass_GetDebugMessageByIndex(3), "Complete") != NULL);

    /* I2C failure */
    nak_mem_read = 1;
    CHECK(Compass_Calibrate() == COMPASS_ERROR);
    nak_mem_read = 0;
}

TEST(test_selftest)
{
    fresh_init();
    bno_regs[REG_SYS_STAT] = 5;
    bno_regs[REG_SYS_ERR]  = 0;
    set_euler(90.0f, 1.0f, 2.0f);
    CHECK(Compass_SelfTest() == COMPASS_OK);

    bno_regs[REG_SYS_ERR] = 0x03;      /* fusion algorithm error */
    CHECK(Compass_SelfTest() == COMPASS_ERROR);
    bno_regs[REG_SYS_ERR] = 0;

    bno_regs[REG_SYS_STAT] = 0;        /* not running */
    CHECK(Compass_SelfTest() == COMPASS_ERROR_SYS_STATUS);

    bno_regs[REG_SYS_STAT] = 5;
    nak_mem_read = 1;
    CHECK(Compass_SelfTest() == COMPASS_ERROR);
    nak_mem_read = 0;
}

/* ------------------------------------------------------------------ */
/* Calibration save / restore                                          */
/* ------------------------------------------------------------------ */

TEST(test_cal_data_roundtrip)
{
    fresh_init();

    for (int i = 0; i < BNO055_CAL_DATA_LEN; i++) {
        bno_regs[REG_CAL_DATA + i] = (uint8_t)(0xA0 + i);
    }

    uint8_t buf[BNO055_CAL_DATA_LEN];
    memset(buf, 0, sizeof(buf));
    CHECK(Compass_GetCalibrationData(buf, sizeof(buf)) == COMPASS_OK);
    CHECK(buf[0] == 0xA0 && buf[21] == 0xA0 + 21);
    CHECK(bno_regs[REG_OPR_MODE] == BNO055_OPR_MODE_NDOF);  /* mode restored */

    /* Restore: verified by read-back -> cal_restored flag set */
    uint8_t saved[BNO055_CAL_DATA_LEN];
    for (int i = 0; i < BNO055_CAL_DATA_LEN; i++) saved[i] = (uint8_t)(i * 3);
    CHECK(Compass_SetCalibrationData(saved, sizeof(saved)) == COMPASS_OK);
    CHECK(Compass_WasCalRestored() == 1);

    /* Corrupted read-back: verify must fail, flag must stay clear */
    compass_cal_restored = 0;
    corrupt_cal_read = 1;
    CHECK(Compass_SetCalibrationData(saved, sizeof(saved)) == COMPASS_ERROR);
    CHECK(Compass_WasCalRestored() == 0);
    CHECK(Compass_GetErrorCode() == COMPASS_ERROR_CALIB);
    corrupt_cal_read = 0;

    /* Guards */
    CHECK(Compass_GetCalibrationData(NULL, 22) == COMPASS_ERROR);
    CHECK(Compass_GetCalibrationData(buf, 21) == COMPASS_ERROR);
    CHECK(Compass_SetCalibrationData(NULL, 22) == COMPASS_ERROR);
}

/* ------------------------------------------------------------------ */
/* Accessors / misc                                                    */
/* ------------------------------------------------------------------ */

TEST(test_accessors)
{
    fresh_init();

    bno_regs[REG_SYS_STAT]  = 5;
    bno_regs[REG_ST_RESULT] = 0x0F;
    bno_regs[REG_SYS_ERR]   = 0;
    bno_regs[REG_OPR_MODE]  = BNO055_OPR_MODE_NDOF;
    bno_regs[REG_PWR_MODE]  = 0;
    bno_regs[REG_TEMP]      = (uint8_t)-5;

    uint8_t st, self, err, mode, pwr;
    int8_t temp;
    CHECK(Compass_GetSystemStatus(&st, &self, &err) == COMPASS_OK);
    CHECK(st == 5 && self == 0x0F && err == 0);
    CHECK(Compass_GetOperationMode(&mode) == COMPASS_OK && mode == BNO055_OPR_MODE_NDOF);
    CHECK(Compass_GetPowerMode(&pwr) == COMPASS_OK && pwr == 0);
    CHECK(Compass_GetTemperature(&temp) == COMPASS_OK && temp == -5);

    /* ReadRegister NAK paths (master tx and rx) */
    nak_master_rx = 1;
    CHECK(Compass_GetOperationMode(&mode) == COMPASS_ERROR);
    nak_master_rx = 0;
    nak_master_tx = 1;
    CHECK(Compass_GetPowerMode(&pwr) == COMPASS_ERROR);
    nak_master_tx = 0;

    /* Debug info + timestamps */
    uint8_t addr = 0, raw[6], sreg = 0, cal = 0;
    Compass_GetDebugInfo(&addr, raw, &sreg, &cal);
    CHECK(addr == 0x28);
    Compass_GetDebugInfo(NULL, NULL, NULL, NULL);      /* null-safe */
    CHECK(Compass_GetDebugTimestamp() == 0);           /* never set */
    CHECK(Compass_GetDebugMessageByIndex(99)[0] == '\0');

    /* GetHeading returns the corrected value */
    set_euler(0.0f, 0.0f, 0.0f);
    CHECK_NEAR(Compass_GetHeading(), 98.5, 0.15);

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

/* ------------------------------------------------------------------ */

int main(void)
{
    Test_SetTick(100000);

    run_test_init_success_default_address();
    run_test_init_alternate_address();
    run_test_init_no_devices();
    run_test_init_wrong_chip_id();
    run_test_init_survives_nak_writes_and_bad_status();
    run_test_update_heading_offset_and_wrap();
    run_test_update_raw_sensor_burst();
    run_test_heading_valid_via_restored_cal();
    run_test_heading_valid_latch();
    run_test_periodic_mode_recovery();
    run_test_calibrate_fully_calibrated();
    run_test_calibrate_partial_guidance();
    run_test_selftest();
    run_test_cal_data_roundtrip();
    run_test_accessors();
    run_test_display_i2c_scan();

    return TEST_SUMMARY();
}
