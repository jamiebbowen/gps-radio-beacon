/**
 * @file preflight_eval.h
 * @brief One go/no-go evaluation shared by every READY TO FLY display.
 *
 * The PRE-FLIGHT page and the navigation page's ready-to-fly row used to
 * compute their verdicts separately, and the navigation page skipped the
 * TESTING-build and low-battery vetoes: the default page could say READY
 * while the checklist said NOT READY. main.c evaluates once per loop and
 * hands the same result to both pages.
 */
#ifndef PREFLIGHT_EVAL_H
#define PREFLIGHT_EVAL_H

#include <stdint.h>

/* Below this pack voltage (1S LiPo, mV) the receiver may brown out
 * mid-session and scramble SD state: a hard veto. 0 = not measured. */
#define PF_LOW_POWER_MV  3200u

typedef struct {
  uint8_t  link_ok;          /* beacon heard within the 35 s freshness gate */
  uint8_t  tx_fix;           /* beacon fix quality (last position packet)   */
  uint8_t  tx_sats;          /* beacon satellites                           */
  uint8_t  rx_fix_ok;        /* receiver has its own fix                    */
  uint8_t  compass_ok;       /* heading valid and live                      */
  uint16_t rx_vdd_mv;        /* receiver pack voltage, 0 = not measured     */
  uint8_t  testing_suspect;  /* beacon looks like a TESTING_MODE build      */
} Preflight_Inputs_t;

typedef enum {
  PF_READY = 0,
  PF_NOT_READY,              /* one or more checks failing                  */
  PF_VETO_TEST_BUILD,        /* the beacon runs a bench (TESTING) build     */
  PF_VETO_LOW_POWER          /* receiver pack below PF_LOW_POWER_MV         */
} Preflight_Verdict_t;

typedef struct {
  uint8_t link_ok;
  uint8_t tx_gps_ok;         /* fix >= 1 and >= 4 satellites                */
  uint8_t rx_gps_ok;
  uint8_t compass_ok;
  uint8_t low_power;
  uint8_t verdict;           /* Preflight_Verdict_t                         */
} Preflight_Result_t;

/** Evaluate the go/no-go checks. Vetoes outrank the individual checks:
 *  a test-build beacon (while its link is up) first, then low power. */
void Preflight_Evaluate(const Preflight_Inputs_t *in, Preflight_Result_t *out);

#endif /* PREFLIGHT_EVAL_H */
