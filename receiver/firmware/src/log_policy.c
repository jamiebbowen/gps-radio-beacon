/**
 * @file    log_policy.c
 * @brief   main.c's SD breadcrumb decisions (see log_policy.h).
 */
#include "log_policy.h"
#include <stdio.h>

uint8_t LogPolicy_LinkEdge(LogPolicy_LinkEdge_t *st, int16_t rssi_dbm,
                           int8_t snr_db, char *out, size_t n)
{
  if (!st->weak && rssi_dbm <= LOG_LINK_WEAK_DBM) {
    st->weak = 1;
    snprintf(out, n, "LINK WEAK rssi=%ddBm snr=%d", (int)rssi_dbm, (int)snr_db);
    return 1;
  }
  if (st->weak && rssi_dbm >= LOG_LINK_OK_DBM) {
    st->weak = 0;
    snprintf(out, n, "LINK OK rssi=%ddBm snr=%d", (int)rssi_dbm, (int)snr_db);
    return 1;
  }
  return 0;
}

uint8_t LogPolicy_Contact(LogPolicy_Contact_t *st, uint32_t last_pkt_ms,
                          uint32_t now_ms, char *out, size_t n)
{
  if (last_pkt_ms == 0) return 0;
  if (now_ms - last_pkt_ms > LOG_CONTACT_LOST_MS) {
    if (st->lost) return 0;
    st->lost = 1;
    st->silence_start_ms = last_pkt_ms;
    snprintf(out, n, "RF LOST (>5min silence)");
    return 1;
  }
  if (!st->lost) return 0;
  /* A new packet ended the silence: report all of it, from the last packet
   * before it (counting from the LOST row came out 5 min short). */
  st->lost = 0;
  snprintf(out, n, "RF REGAINED after %lus",
           (unsigned long)((last_pkt_ms - st->silence_start_ms) / 1000u));
  return 1;
}

int LogPolicy_FormatRfStats(const LogPolicy_RfStats_t *s, char *out, size_t n)
{
  char nf_field[12] = "n/a";
  char nfl_field[24] = "";
  if (s->nf_valid) snprintf(nf_field, sizeof(nf_field), "%ddBm", (int)s->nf_dbm);
  if (s->nfl_valid) snprintf(nfl_field, sizeof(nfl_field), " nfl=%ddBm", (int)s->nfl_dbm);
  return snprintf(out, n,
                  "RFSTATS pkts=%lu irq=%lu crc=%lu wedges=%lu nf=%s vdd=%umV loop=%lums sd=%lums syncs=%lu spidiv=%u rf=%lums gps=%lums disp=%lums cmp=%lums iters=%lu sderr=%lu%s",
                  (unsigned long)s->pkts, (unsigned long)s->irqs,
                  (unsigned long)s->crc, (unsigned long)s->wedges, nf_field,
                  (unsigned)s->vdd_mv, (unsigned long)s->loop_ms, (unsigned long)s->sd_ms,
                  (unsigned long)s->syncs, (unsigned)s->spidiv,
                  (unsigned long)s->rf_ms, (unsigned long)s->gps_ms,
                  (unsigned long)s->disp_ms, (unsigned long)s->cmp_ms,
                  (unsigned long)s->iters, (unsigned long)s->sderr, nfl_field);
}
