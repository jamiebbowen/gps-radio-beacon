# Receiver Bench Checklist

A short power-up checklist for the receiver, focused on things that can only
be verified on hardware. Run it after flashing new firmware and before every
launch weekend. Total time: ~5 minutes with the beacon on the bench.
Beacon-side checks live in `bench-checklist-tx.md`.

Firmware verified by this checklist: display geometry fixes, compass error
reporting, SD self-test, LED packet pulse, LoRa link.

## 0. Transmitter USB banner (after every flash)

Connect the beacon over USB and watch the first seconds of boot:

- [ ] `[Radio] Fused time-on-air: NNNN ms, cadence 1500 ms` — the on-air
      time must clear the cadence with a real gap; a `WARNING` line here
      means a packet-format or radio-config change ate the PA duty margin.
- [ ] The callsign line shows the intended `KE0MZS-<id> CH<n>` for this
      airframe.
- [ ] If you INTENDED a testing flash: confirm the receiver's PRE-FLIGHT
      page shows the starred callsign (`TST *KE0MZS-n CHn*`) within ~60 s -
      that path verifies both the beacon and the bench-build detector.
      If you intended production: starred callsign = you flashed a
      TESTING_MODE build; reflash before the pad.

## 1. Boot sequence (just watch the screen)

- [ ] **Test pattern**: `TL` top-left, `TR` top-right, `BL` bottom-left,
      `BR` bottom-right, `CENTER` centered. All four corners must be visible
      and in the right corners — anything shifted or missing means the
      display rotation/geometry regressed.
- [ ] **RF Init**: shows `RF Init / OK / LoRa 433MHz`. An error screen here
      shows `RF Init Err: 0xNN` — note the code.
- [ ] **SD Card**: `SD Card / Ready` then `SD Self-Test / PASS`.
      On FAIL the screen shows `S:<step> FR:<result>` plus
      `CMD:xx DT:xx T:n` — photograph it; see
      `sd-card-troubleshooting.md` for decoding.
- [ ] **Last Beacon**: if a previous flight saved a position, its lat/lon
      shows for 1.5 s and the nav screen has a bearing immediately.
- [ ] **BNO085 Initialized**, with `Cal: dynamic (sens)` — the BNO085 keeps
      calibration in-sensor (the `Cal: Restored`/`None saved` messages only
      existed on the old BNO055 host-offset path).

## 2. LED (PC13)

- [ ] LED is **off** when no packets are arriving (it used to be solid on —
      solid on now means something is wrong).
- [ ] With the beacon transmitting, LED gives a **short blink on every
      packet** (~100 ms). Blink cadence should match the beacon's send rate.

## 3. RF link (beacon on, a few metres away)

- [ ] RF display mode (button-cycle to it): `P: n/n` parse counters climbing,
      `Lat/Lon/Alt` matching the beacon's position, `Fix: Y`.
- [ ] Rows 6–7 show the raw packet ASCII (two 15-char lines).
- [ ] Navigation mode: distance is plausibly small, arrow tracks as you
      rotate the receiver.

## 4. Compass / IMU

- [ ] IMU test mode: real values updating; **deliberately induce an error**
      once (e.g. hold BOOT and disconnect the BNO085 I2C wire briefly) and
      confirm the error line shows an actual message, not `No error` —
      this verifies the error-reporting path end-to-end.

      NOTE: only do the wire-pull test on the bench, never before a flight.
- [ ] Compass heading mode: `Cal S_ G_ A_ M_` — wave figure-8 until `M3`.
      Power-cycle and confirm boot shows `Cal: dynamic (sens)` — there is
      no SD-backed restore on the BNO085; `M` re-climbs after remotion.

      IMPORTANT: after boot the BNO085 still reports `S0 G0 A0 M0` until
      fresh motion re-verifies — that is normal, NOT a lost calibration.
      The BNO085 calibrates dynamically in-sensor; there is no host restore
      step and nothing stored on the SD card. Just keep the figure-8 habit:
      accuracy digits climb back to `M3` within a minute of motion.
- [ ] Heading sanity: point the antenna north, heading reads ~0°
      (declination is compiled for the Denver area — see
      `COMPASS_MAGNETIC_DECLINATION_DEG` in `compass.c` before travelling).

## 5. SD logging round-trip (weekend-before check)

- [ ] Receive ≥10 packets from the beacon, power off, pull the card.
- [ ] `make extract DEV=/dev/sdX OUT=/tmp/bench` — expect `BEACON.TXT`,
      `COMPCAL.BIN`, and a new `Lnnnn.TXT`.
- [ ] `python3 receiver/tools/analyze_flight.py /tmp/bench/Lnnnn.TXT` —
      must print packet counts (a bench log will say "No launch detected",
      which is correct). If it says "Not enough NAV rows", the firmware
      log format and the tools have drifted — run `make tools-test`.

## Field notes

- Button cycles: Navigation → GPS → RF → Compass Visual → Compass Heading →
  SD Card → Rocket Select (long-press cycles channel) → RF Stats →
  Preflight → back. Any screen auto-returns to Navigation after 120 s.
- Remote position goes stale (nav arrow frozen, `has_valid_remote_gps`
  dropped) after **180 s** without a packet.
- BEACON.TXT persists at most every **10 s**, so a power cut can lose up to
  10 s of last-known-position; the per-packet NAV log is synced every row
  and loses nothing.
