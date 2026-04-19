/**
 * @file imu_test_mode.h
 * @brief IMU self-test display mode header
 */

#ifndef __IMU_TEST_MODE_H
#define __IMU_TEST_MODE_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Display IMU self-test screen
 * 
 * Shows the results of the BNO055 IMU self-test and calibration status
 */
void DisplayMode_IMUTest(void);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_TEST_MODE_H */
