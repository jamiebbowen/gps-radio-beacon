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
 *   u8  kind
 *   u8  reserved[3]
 *   payload x20 (kind-dependent)
 *
 * Kinds:
 *   1 imu          accel/gyro/peak payload
 *   2 gps          lat/lon/alt/sats/fix payload
 *   3 launch       marker only
 *   4 landed       marker only
 *   5 flight event code=FLIGHT_EVENT_*, value=event magnitude (anomaly /
 *                  life-cycle edges from flight_events.cpp)
 *   6 post-launch  marker only (cadence transition)
 *   7 battery-save marker only (cadence transition)
 *   8 boot         code=SAMD51 RCAUSE byte (brown-out / watchdog reboot
 *                  evidence: without it an uptime restart is invisible)
 *   9 fused        lat/lon/alt + vn/ve/vd + flags (EKF snapshot taken
 *                  while GPS is stale - the only position data the
 *                  beacon has during a fix hole)
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
void flight_log_event(uint32_t ms, uint8_t kind);   /* 3=launch, 4=landed, ... */
/** Event with an identity: kind 5 (flight event code + magnitude) or
 *  kind 8 (RCAUSE byte at rearm). flight_log_event() is the code/value=0
 *  shorthand used by the plain markers. */
void flight_log_event_ex(uint32_t ms, uint8_t kind, uint8_t code, int16_t value);
/** Fused EKF snapshot (kind 9): position + NED velocity at cm/s plus
 *  health flags (bit0 GPS fresh, bit1 dead reckoning, bit2 IMU healthy).
 *  Call sites gate on a fix hole so region space goes to the moments the
 *  raw GPS stream has nothing to say. */
void flight_log_fused(uint32_t ms, int32_t lat_e7, int32_t lon_e7,
                      int16_t alt_m, int16_t vn_cms, int16_t ve_cms,
                      int16_t vd_cms, uint8_t flags);

/** Serial dump over USB while powered (post-recovery forensics):
 * Emits CSV: ms,1,ax_cg,ay_cg,az_cg,gx_cds,gy_cds,gz_cds,peak_mg   (imu)
 *            ms,2,lat_e7,lon_e7,alt_m,sats,fix                    (gps)
 *            ms,kind                                              (3/4/6/7/8, code/value 0)
 *            ms,kind,code,value                                  (events 5/8)
 *            ms,9,lat_e7,lon_e7,alt_m,vn,ve,vd,flags             (fused)
 * via Serial.print. Holds the loop; call on command. */
void flight_log_dump_serial(void);

#endif
