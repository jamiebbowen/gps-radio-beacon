# Transmitter Bench Checklist

Power-up and fault-injection checks for the beacon (ItsyBitsy M4), focused on
things that can only be verified on hardware. Run after flashing new firmware
and before every launch weekend. Total time: ~10 minutes with the beacon on
the bench and USB serial open at 115200 baud.

Companion docs: receiver checks live in `bench-checklist.md`; the field
runbook is `launch-day.md`.

## 0. USB boot banner (watch the first ~20 s)

- [ ] `[Launch] Initializing BNO085 IMU...` -> `BNO08x Found!`.
      `400 kHz failed, retrying 100 kHz` is the intended I2C speed fallback
      (long-wire safety) - fine once, a flag if it repeats every boot.
- [ ] `✓ Linear acceleration enabled`, `✓ Rotation vector enabled`,
      `✓ Gyroscope enabled`, `✓ Launch detection ready`. A `⚠` line means
      the IMU answered but a report failed to enable.
- [ ] `[Radio] radio.begin() returned: 0` then `✓ Initialized successfully`.
      `CHIP_NOT_FOUND` (-2) = SPI wiring/power; other codes are listed in
      the banner itself.
- [ ] `[Radio] Fused time-on-air: NNNN ms, cadence 1500 ms` - the on-air
      time must clear the cadence with a real gap. `WARNING: fused cadence
      gap under 100 ms` means a packet-format or radio-config change ate the
      PA duty margin; fix before flight.
- [ ] `[Beacon] HELLO fw=0x...` shows the expected firmware hash.
- [ ] No `[Boot] *** Recovered from WATCHDOG RESET` / `BROWN-OUT` banner on
      a clean plug-in (those lines are the reset-cause reporter working as
      intended - just not expected now).

## 1. Flight log state + DUMP command

- [ ] `[FLOG] previous flight data present` + the `type 'DUMP'` hint on a
      used flash; absent on a freshly formatted one.
- [ ] Type `DUMP` + Enter over USB: CSV rows print (`ms,kind,...` header).
      This proves the on-chip flight log + the serial command channel
      end-to-end.
- [ ] While flying (LAUNCH/POST_LAUNCH states), `DUMP` answers
      `[FLOG] airborne - dump after landing` and prints nothing else - the
      refusal path protects in-flight GPS ticks.
- [ ] On-air evidence: the heartbeat `reset_info` field carries the boot
      cause. Trigger a button reset once while armed and check the receiver
      SD log records a reset-cause transition line.

- [ ] DUMP output starts with the IMU units note; IMU rows written by this
      firmware end with `,3` (V3 units).

## 2. GPS path

- [ ] `[GPS] ✓ Fix:` lines appear once per second after lock; sats climb to
      plausible counts. The heartbeat `sats=` transmit lines match.
- [ ] `[Beacon] Rejecting GPS -` lines appear ONLY while genuinely without
      fix (cold start, indoors). Their reasons are self-describing:
      `insufficient satellites`, `no fix`, `bad coords`, `unreasonable
      altitude`.

## 3. GPS watchdog (deliberate fault - bench only)

- [ ] Unplug the GPS antenna (or shade it under metal): within ~30 s
      `[GPS] Watchdog: No UART data` or within ~60 s `No valid fix` prints,
      then `Recovery: UBX-CFG-RST hot start` and `Recovery attempt
      complete`. Leave it out: the next attempts print `warm start`, then
      `cold start` (the heartbeat's `rst=` count climbs 1-3).
- [ ] Reconnect: fixes resume without a reboot.
- [ ] If you leave it unplugged long enough to see
      `Recovery failed after 3 attempts - cooling down`, that is the
      three-strikes cooldown; `cooldown over, re-arming` should follow.
      Persistent failure here = module/wiring fault, not firmware.
- [ ] In flight the watchdog never resets the module: during a GPS-altitude
      fallback launch (section 6) a fault prints `settings re-send only (in
      flight)` instead of a UBX-CFG-RST line.

## 4. IMU launch path (shake test)

- [ ] Shake the beacon sharply: `[Launch] Acceleration detected:` prints.
- [ ] A single shake must produce `False alarm, acceleration dropped` -
      never `LAUNCH CONFIRMED`. The confirm path needs SUSTAINED thrust;
      if a bench shake latches launch, the thresholds regressed and the
      flight profile is unsafe.
- [ ] `Sensor was reset, re-enabling reports` may appear after violent
      shaking - that is the BNO085 self-recovery working.

## 5. Landing + descent evidence (post-flight review)

These can't be forced on the bench, but know what the log will show:

- `[Launch] GPS-alt fallback: climb of NNN m sustained - LAUNCH CONFIRMED`
  = the GPS-altitude backup fired because the IMU path didn't (a dead IMU
  no longer means no launch detect).
- `[Launch] LANDING DETECTED (quiet + stable alt)` = the 60 s
  quiet-accel + stable-altitude window latched; the beacon then drops to
  battery-save cadence.
- `[Nav] GPS REJECTED: nis=...` lines = the EKF innovation gate dropping
  multipath/glitch fixes; `[Nav] RESCUE:` lines = the starved-state
  re-anchor after long outages. Both normal in noisy conditions.

## 6. Mid-flight reboot resume

The FLYING state marker is only written at *launch confirmation*
(`flight_log_arm`), so a power-cycle while merely armed is a normal
cold boot - that is correct, not a missed resume.

- [ ] **Flight-time evidence**: after a real launch, any reset prints
      `[Boot] *** mid-flight reboot: resuming flight ***` and the
      receiver's SD log gains an anomaly event - the flight record
      continues appending into the surviving log instead of erasing it.
- [ ] After the resume banner the beacon leaves continuous TX within a few
      seconds (POST_LAUNCH) and the callsign airs again within 5 minutes.
      (A resumed flight used to stay in LAUNCH for good, never airing the
      callsign.)
- [ ] **Optional field verification without flying**: take the beacon up
      ~50 m of real elevation (a hill drive or 15+ flights of stairs with
      GPS view) - the GPS-altitude fallback prints `GPS-alt fallback:
      climb of NNN m sustained - LAUNCH CONFIRMED`, the marker is written,
      and a power-cycle then demonstrates the resume banner + retained
      log (DUMP it afterward to confirm both halves survived).
- [ ] If a confirmed launch + reset shows a plain cold-start banner, the
      marker didn't survive - investigate before flying.

## Field notes

- Serial prints are the primary health surface on the bench; on the air,
  the equivalent evidence is the heartbeat `sats`/`gps_health`/`reset_info`
  fields, fused packets, flight events, and maxima recaps on the receiver.
- `[Radio] Reinitialize attempt N` / `TX failure streak` lines = the radio
  self-recovery ladder running; a healthy bench session shows none.
- `[2way] CMD answered` only appears when a command was received from the
  receiver's command channel - expected during paired two-way tests.
- The beacon's USB CDC is also the recovery crew's door: leaving `DUMP`
  working is part of pre-flight readiness.
