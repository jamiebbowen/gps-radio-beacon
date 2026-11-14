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

  out->link_ok    = in->link_ok ? 1 : 0;
  out->tx_gps_ok  = (in->tx_fix >= 1 && in->tx_sats >= 4) ? 1 : 0;
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
