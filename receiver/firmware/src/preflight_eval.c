/**
 * @file preflight_eval.c
 * @brief Shared go/no-go evaluation (see preflight_eval.h)
 */
#include "preflight_eval.h"
#include <string.h>

void Preflight_Evaluate(const Preflight_Inputs_t *in, Preflight_Result_t *out)
{
  if (out == NULL) return;
  memset(out, 0, sizeof(*out));
  if (in == NULL) {
    out->verdict = PF_NOT_READY;
    return;
  }

  out->link_ok = (in->link_heard && in->link_age_s <= PF_LINK_FRESH_S) ? 1 : 0;

  /* The beacon's GPS from the newer of its two reports: on the pad the
   * heartbeat (every minute) is fresher than the position audit (every 10
   * minutes) and carries the same sats/fix quality. */
  if (in->hb_valid && (!in->pos_valid || in->hb_age_s < in->pos_age_s)) {
    out->tx_fix  = in->hb_fix;
    out->tx_sats = in->hb_sats;
  } else if (in->pos_valid) {
    out->tx_fix  = in->pos_fix;
    out->tx_sats = in->pos_sats;
  }
  out->tx_gps_ok  = (out->tx_fix >= 1 && out->tx_sats >= 4) ? 1 : 0;
  out->rx_gps_ok  = in->rx_fix_ok ? 1 : 0;
  out->compass_ok = in->compass_ok ? 1 : 0;
  out->low_power  = (in->rx_vdd_mv > 0 && in->rx_vdd_mv < PF_LOW_POWER_MV) ? 1 : 0;

  /* Hardware health is a gate. A TESTING-build beacon (judged while its
   * link is up) blocks readiness outright: a pad-passed but wrong-firmware
   * launch must never look ready. The battery rail is the next gate. */
  if (out->link_ok && in->testing_suspect) {
    out->verdict = PF_VETO_TEST_BUILD;
  } else if (out->low_power) {
    out->verdict = PF_VETO_LOW_POWER;
  } else if (out->link_ok && out->tx_gps_ok && out->rx_gps_ok && out->compass_ok) {
    out->verdict = PF_READY;
  } else {
    out->verdict = PF_NOT_READY;
  }
}
