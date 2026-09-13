/**
 * @file test_flight_cadence.cpp
 * @brief Pins the beacon's state->timing policy (flight_cadence.cpp), then
 *        replays a scripted launch-day timeline through the policy the same
 *        way loop() + timer_isr() consume it.
 *
 * Why this exists: the cadence state machine was previously embedded in
 * firmware.ino, host-untested. A regression there is silent and expensive:
 * pad cadence too fast desenses the beacon's own GPS, flight cadence too
 * fast overheats the PA, battery-save too fast kills the pack during a
 * long walk, and the callsign interval is an FCC compliance item
 * (97.119: ID at least every 10 minutes).
 *
 * Build & run:  make -C transmitter/tests
 */

#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "../firmware/include/flight_cadence.h"
#include "../firmware/include/config.h"
#include "../firmware/include/mpu_config.h"
#include "../firmware/include/packet_format.h"
#include "test_harness.h"

/* SX126x time-on-air (Semtech RM, explicit header + CRC16 + LDRO auto per
 * symbol length). Kept here - the pin is what matters, not the plumbing. */
static double lora_airtime_ms(unsigned payload_bytes)
{
    double sf = LORA_SPREADING;
    double tsym_ms = ldexp(1.0, (int)sf) / (LORA_BANDWIDTH * 1000.0) * 1000.0;
    int de = (tsym_ms > 16.0) ? 1 : 0;          /* LDRO rule */
    double num = 8.0 * payload_bytes - 4.0 * sf + 28.0 + 16.0;
    double den = 4.0 * (sf - 2 * de);
    int pay_syms = 8 + (int)ceil(num / den) * LORA_CODING_RATE;
    return (LORA_PREAMBLE + 4.25 + pay_syms) * tsym_ms;
}

/* These tests pin PRODUCTION pacing. A TESTING_MODE build would need its
 * own expectations; the suite always builds the flight config. */
#if TESTING_MODE
#error "test_flight_cadence pins production pacing - build with TESTING_MODE=0"
#endif

TEST(test_cadence_table_pins)
{
    /* Seconds between raw GPS beacons */
    CHECK(flight_cadence_beacon_interval_s(BEACON_STATE_PRE_LAUNCH)  == 5);
    CHECK(flight_cadence_beacon_interval_s(BEACON_STATE_LAUNCH)      == 0);
    CHECK(flight_cadence_beacon_interval_s(BEACON_STATE_POST_LAUNCH) == 2);
    CHECK(flight_cadence_beacon_interval_s(BEACON_STATE_BATTERY_SAVE)== 60);

    /* Milliseconds between fused packets */
    CHECK(flight_cadence_fused_interval_ms(BEACON_STATE_PRE_LAUNCH)  == 5000);
    CHECK(flight_cadence_fused_interval_ms(BEACON_STATE_LAUNCH)      == 1500);
    CHECK(flight_cadence_fused_interval_ms(BEACON_STATE_POST_LAUNCH) == 1500);
    CHECK(flight_cadence_fused_interval_ms(BEACON_STATE_BATTERY_SAVE)== 5000);

    /* FCC 97.119: station ID at least every 10 minutes in flight. We do
     * 5 minutes in every non-flight state (loop() gates callsigns out of
     * the fast LAUNCH phase, which is seconds long). */
    CHECK(flight_cadence_callsign_interval_s() == 300);
    CHECK(flight_cadence_callsign_interval_s() <= 600);
}

TEST(test_cadence_phase_exits)
{
    /* LAUNCH -> POST_LAUNCH after POST_LAUNCH_DURATION_SEC */
    CHECK(!flight_cadence_should_leave_launch(POST_LAUNCH_DURATION_SEC - 1));
    CHECK(flight_cadence_should_leave_launch(POST_LAUNCH_DURATION_SEC));

    /* POST_LAUNCH -> BATTERY_SAVE after the 10-minute recovery window */
    CHECK(!flight_cadence_should_leave_post_launch(POST_LAUNCH_RECOVERY_DURATION_SEC - 1));
    CHECK(flight_cadence_should_leave_post_launch(POST_LAUNCH_RECOVERY_DURATION_SEC));
}

TEST(test_cadence_pa_airtime_margin)
{
    /* The fused interval must clear the packet's computed time-on-air with
     * real gap - otherwise "cadence" degenerates into back-to-back
     * blocking transmits with the PA at ~100% duty. This is a formula
     * guardrail against every future format/config drift: packet-size
     * growth, CR/BW/SF changes, preamble changes. */
    double fused_toa = lora_airtime_ms(FUSED_PACKET_SIZE);
    /* Formula sanity: V2 20B at SF10/62.5k/CR4-8/8pre = 68.25 sym * 16.384 ms */
    CHECK_NEAR(fused_toa, 1118.2, 2.0);
    /* Legacy 19B packet = 987 ms - the step the rocket_id byte crossed */
    CHECK_NEAR(lora_airtime_ms(19), 987.1, 2.0);
    /* The 14-byte V2 GPS packet did NOT cross a step (same as legacy 13B) */
    CHECK_NEAR(lora_airtime_ms(GPS_PACKET_SIZE), lora_airtime_ms(13), 1.0);

    double iv = (double)flight_cadence_fused_interval_ms(BEACON_STATE_LAUNCH);
    CHECK(iv >= fused_toa * 1.1);        /* >10% radio-quiet gap per cycle */
    CHECK(iv <= fused_toa * 2.0);        /* don't relax into uselessness */
}

TEST(test_scripted_flight_timeline)
{
    /* Drive the policy exactly the way firmware.ino does: a 1 Hz tick
     * governs raw-beacon/callsign flags, a per-loop check governs fused.
     * Script: pad sit 47 s, launch, 1 s LAUNCH, 600 s recovery, then
     * battery-save. The counters below pin production behavior end-to-end. */
    const uint32_t T_END_S   = 720;
    const uint32_t LAUNCH_S  = 47;
    const uint32_t LANDED_S  = 648;   /* policy-level battery-save entry */

    beacon_state_t st = BEACON_STATE_PRE_LAUNCH;
    uint32_t last_raw_s = 0, last_callsign_s = 0;
    uint32_t last_fused_ms = 0;
    bool immediate_raw = true;        /* boot + every state entry force a TX */

    uint32_t pad_raws = 0, launch_raws = 0, post_raws = 0, save_raws = 0;
    uint32_t launch_fused = 0, save_fused = 0;
    uint32_t callsigns = 1;           /* setup() transmits one at boot */

    for (uint32_t t_ms = 0; t_ms <= T_END_S * 1000; t_ms += 200) {
        uint32_t s = t_ms / 1000;

        /* Script events (stand-ins for launch_detect / landing latch).
         * Evaluate once per second, like the loop reacting to the tick. */
        if (t_ms % 1000 == 0) {
            if (s == LAUNCH_S) { st = BEACON_STATE_LAUNCH;       immediate_raw = true; }
            if (s == LAUNCH_S + POST_LAUNCH_DURATION_SEC) {
                st = BEACON_STATE_POST_LAUNCH; immediate_raw = true;
            }
            if (s == LANDED_S) { st = BEACON_STATE_BATTERY_SAVE; immediate_raw = true; }
        }

        /* 1 Hz tick: raw beacon + callsign (loop() blocks callsigns in
         * the fast phase) */
        if (t_ms % 1000 == 0) {
            uint32_t iv = flight_cadence_beacon_interval_s(st);
            if (immediate_raw || iv == 0 || (s - last_raw_s) >= iv) {
                last_raw_s = s;
                immediate_raw = false;
                switch (st) {
                    case BEACON_STATE_PRE_LAUNCH:   pad_raws++;    break;
                    case BEACON_STATE_LAUNCH:       launch_raws++; break;
                    case BEACON_STATE_POST_LAUNCH:  post_raws++;   break;
                    case BEACON_STATE_BATTERY_SAVE: save_raws++;   break;
                }
            }
            if (st != BEACON_STATE_LAUNCH &&
                (s - last_callsign_s) >= flight_cadence_callsign_interval_s()) {
                last_callsign_s = s;
                callsigns++;
            }
        }

        /* Per-loop fused check */
        if (t_ms - last_fused_ms >= flight_cadence_fused_interval_ms(st)) {
            last_fused_ms = t_ms;
            if (st == BEACON_STATE_LAUNCH || st == BEACON_STATE_POST_LAUNCH) {
                launch_fused++;
            } else if (st == BEACON_STATE_BATTERY_SAVE) {
                save_fused++;
            }
        }
    }

    /* Pad: one raw every 5 s over 47 s, incl. boot TX: t = 0,5,...,45 */
    CHECK(pad_raws == 10);

    /* Launch phase: the LAUNCH interval is 0 s (continuous) and the entry
     * forces a TX; this 1 Hz model captures exactly one. The real loop
     * additionally transmits back-to-back off the fast flag - the paced
     * phases are where exact counts matter. */
    CHECK(launch_raws == 1);

    /* Recovery window: 600 s / 2 s pacing = 300 raw beacons */
    CHECK(post_raws == 300 || post_raws == 301);

    /* Battery-save tail (648..720 s): entry TX plus one at the 60 s mark */
    CHECK(save_raws == 2);

    /* Fused stream in flight phases: 601 s at 1.5 s nominal. The harness
     * steps 200 ms so the effective spacing quantizes to 1.4/1.6 s -> the
     * deterministic count for this script is 376. */
    CHECK(launch_fused == 376);
    /* ...then 5 s spacing on the ground */
    CHECK(save_fused >= 13 && save_fused <= 15);

    /* FCC ID cadence: boot ID plus t = 300, 600 */
    CHECK(callsigns == 3);
}

int main(void)
{
    run_test_cadence_table_pins();
    run_test_cadence_phase_exits();
    run_test_cadence_pa_airtime_margin();
    run_test_scripted_flight_timeline();
    return TEST_SUMMARY();
}
