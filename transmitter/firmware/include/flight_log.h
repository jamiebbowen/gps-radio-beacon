#ifndef FLIGHT_LOG_H
#define FLIGHT_LOG_H

#include <stdint.h>

/**
 * On-beacon flight log in the SAMD51's own internal flash.
 *
 * Sits next to nav's fused estimator in memory only, and wakes up armed at
 * every launch-detect trip: the airframe shredder in L0016 never got the
 * beacon back (and therefore its radio-packed telemetry) - this log is the
 * copy a "good summer" would hand you on a post-crash NAND dump.
 *
 * Format: fixed 32-byte records, all little-endian:
 *   u32 magic 'FLOG'      (0x4C464C46 = "FLF" then a kind byte)
 *   u32 ms since TX boot
 *   u8  kind (1=imu, 2=gps, 3=launch marker, 4=landed marker)
 *   u8  reserved[3]
 *   payload x20 (kind-dependent: 7x i16 accel/gyro/peak for IMU,
 *                lat_e7/lon_e7/alt/sats/fix for GPS)
 *
 * Erase policy: full-region erase at next launch. The space spans twelve
 * 16KB erase rows; that takes NVMCTL row-erase time per row (~20 ms each,
 * ~250 ms total), which happens once at T0 - on the pad the PAYOFF is
 * worth it. In flight the writes append with a 512B page commit; if power
 * dies mid-page, only that page is lost.
 */

void flight_log_init(void);
/** Called when launch_detect confirms; erases the region (~250 ms) and
 *  arms the recorder. Pre-launched again before erase completes -> caller
 *  must wait for flight_log_armed(). */
void flight_log_arm(void);
uint8_t flight_log_armed(void);
uint8_t flight_log_busy(void);          /* erase in progress */

/** Raw IMU sample: centi-g and centi-deg/s, same units as the IMU packet. */
void flight_log_imu(uint32_t ms,
                    int16_t ax_cg, int16_t ay_cg, int16_t az_cg,
                    int16_t gx_cds, int16_t gy_cds, int16_t gz_cds,
                    int16_t peak_mg);
void flight_log_gps(uint32_t ms,
                    int32_t lat_e7, int32_t lon_e7, int16_t alt_m,
                    uint8_t sats, uint8_t fix_quality);
void flight_log_event(uint32_t ms, uint8_t kind);   /* 3=launch, 4=landed */

/** Serial dump over USB while powered (post-recovery forensics):
 * Emits CSV: ms,kind,ax_cg,ay_cg,az_cg,gx_cds,gy_cds,gz_cds,peak_mg
 * or       ms,kind,lat_e7,lon_e7,alt_m,sats,fix
 * via Serial.print. Holds the loop; call on command. */
void flight_log_dump_serial(void);

#endif
