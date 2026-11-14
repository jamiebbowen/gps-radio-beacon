/**
 * @file    log_policy.h
 * @brief   main.c's SD breadcrumb decisions as pure functions of their
 *          inputs, so the host tests can pin them (main.c has no host
 *          build, and the host tools parse these rows).
 */
#ifndef LOG_POLICY_H
#define LOG_POLICY_H

#include <stdint.h>
#include <stddef.h>

/* Link-edge breadcrumbs: mark the walk-out / walk-back edge the moment it
 * is crossed, without post-hoc RSSI archaeology. The -85 / -80 dBm
 * hysteresis keeps flutter out. */
#define LOG_LINK_WEAK_DBM   (-85)
#define LOG_LINK_OK_DBM     (-80)

typedef struct {
  uint8_t weak;
} LogPolicy_LinkEdge_t;

/** Per received packet. Returns 1 and writes the row ("LINK WEAK ..." or
 *  "LINK OK ...") when this packet crosses an edge. */
uint8_t LogPolicy_LinkEdge(LogPolicy_LinkEdge_t *st, int16_t rssi_dbm,
                           int8_t snr_db, char *out, size_t n);

/* Contact-loss session marker: one "RF LOST" row per silence longer than
 * LOG_CONTACT_LOST_MS (matches the auto re-scan threshold, far beyond the
 * slowest beacon cadence), so review can tell "signal lost at T" from
 * "operator stopped logging". The "RF REGAINED after Ns" row reports the
 * whole silence, last packet to first packet - forensic data (beacon
 * recovery vs receiver wedge vs geometry). */
#define LOG_CONTACT_LOST_MS 300000u

typedef struct {
  uint8_t  lost;
  uint32_t silence_start_ms;   /* last packet before the silence */
} LogPolicy_Contact_t;

/** Every loop. last_pkt_ms: arrival time of the newest packet, 0 = none
 *  yet. Returns 1 and writes the row when one is due. */
uint8_t LogPolicy_Contact(LogPolicy_Contact_t *st, uint32_t last_pkt_ms,
                          uint32_t now_ms, char *out, size_t n);

/* The once-a-minute RFSTATS breadcrumb */
typedef struct {
  uint32_t pkts, irqs, crc, wedges;
  uint8_t  nf_valid;   int16_t nf_dbm;    /* alert noise floor   */
  uint16_t vdd_mv;                        /* pack voltage        */
  uint32_t loop_ms, sd_ms, syncs;
  uint8_t  spidiv;
  uint32_t rf_ms, gps_ms, disp_ms, cmp_ms;
  uint32_t iters, sderr;
  uint8_t  nfl_valid;  int16_t nfl_dbm;   /* live-link floor     */
} LogPolicy_RfStats_t;

/** Format the RFSTATS row; returns snprintf's result. */
int LogPolicy_FormatRfStats(const LogPolicy_RfStats_t *s, char *out, size_t n);

#endif /* LOG_POLICY_H */
