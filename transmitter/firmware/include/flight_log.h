#ifndef FLIGHT_LOG_H
#define FLIGHT_LOG_H

#include <stdint.h>

/* Record kinds (the DUMP 'kind' column) */
#define FLOG_KIND_IMU           1
#define FLOG_KIND_GPS           2
#define FLOG_KIND_LAUNCH        3
#define FLOG_KIND_LANDED        4
#define FLOG_KIND_FLIGHT_EVENT  5
#define FLOG_KIND_POST_LAUNCH   6
#define FLOG_KIND_BATTERY_SAVE  7
#define FLOG_KIND_BOOT          8
#define FLOG_KIND_FUSED         9
#define FLOG_KIND_STATE        10   /* persistent flight-state marker    */

/* Fused-record flags byte */
#define FLOG_FUS_GPS_FRESH   0x01
#define FLOG_FUS_DR          0x02
#define FLOG_FUS_IMU_HEALTHY 0x04

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
 *  10 state        flight-state marker on the RESERVED last 8KB row:
 *                  code 1=FLYING (written at arm), 2=LANDED (landing
 *                  latch). This is the mid-flight reboot-resume evidence:
 *                  a boot that finds the marker still FLYING knows the
 *                  chip reset in mid-air and calls flight_log_resume()
 *                  instead of erasing the surviving log.
 *
 * Erase policy: full-region erase at next launch, INCLUDING the marker
 * row (the arm then rewrites it FLYING). Normal records append only into
 * the first FLOG_BYTES-8KB so a full data region can never clobber the
 * marker. The erase takes NVMCTL row-erase time per row (~20 ms each,
 * ~500 ms total), which happens once at T0 - on the pad the PAYOFF is
 * worth it. In flight the writes append with a 512B page commit; if power
 * dies mid-page, only that page is lost.
 */

void flight_log_init(void);
/** Called when launch_detect confirms: synchronously erases the whole
 *  region (32 block erases) and arms the recorder. A second call while
 *  armed is a no-op. */
void flight_log_arm(void);

/** Nonzero when the state row still says FLYING - i.e. the previous run
 *  was airborne when the chip reset. Checked at boot before arming. */
uint8_t flight_log_prev_flying(void);
/** Mid-flight reboot-resume: restores the append point by scanning for
 *  the first uncommitted page and re-arms WITHOUT erasing prior records.
 *  Returns nonzero when it resumed (marker was FLYING). */
uint8_t flight_log_resume(void);
/** Rewrites the state marker LANDED at the landing latch so the next
 *  boot reads it as a power-on, not a mid-flight reset. */
void flight_log_mark_landed(void);

/** IMU record format byte (rsv[0]): V3 units - centi-g, 0.1 deg/s,
 *  milli-g, the same as the IMU trace packet. Pre-V3 records read 0. */
#define FLOG_IMU_FMT_V3  3

/** Raw IMU sample in V3 packet units (the encoded packet's own fields). */
void flight_log_imu(uint32_t ms,
                    int16_t ax_cg, int16_t ay_cg, int16_t az_cg,
                    int16_t gx_dds, int16_t gy_dds, int16_t gz_dds,
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
