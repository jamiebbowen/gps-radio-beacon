# Launch Day Runbook

Field sequence, built around what the receiver actually shows. Assumes the
bench checklist (`bench-checklist.md`) passed this weekend.

## Arriving at the site

1. **Power the receiver.** Watch boot: RF init OK, SD ready, compass
   calibration restored. Last Beacon position showing is fine (it's the
   previous flight's, loaded from the card) - unless the arrow points at
   another state (see cross-state sanity below).
2. **Watch the boot NF sweep.** The receiver sweeps all 8 channels at boot
   and logs them. `-70 dBm` on some channel = that channel is hot on this
   day; cleanest channels hover around `-80`. If you must pick (see
   "Crowded channel"), pick from the quietest ones, don't just flip a coin.

### Magnetic declination

The BNO085 measures *magnetic* north; the arrow points along *true* north.
The receiver now self-corrects: after your local GPS first fixes, the
CONUS declination grid (WMM2025) gives the site value (Denver +7.5 deg,
Reno +12.7 deg, Philadelphia -11.8 deg, ...) and applies it once. An explicit `DECLIN.TXT` on the card
(produced by the user) still beats the table.

### Cross-state boot sanity

If the card's saved last-beacon position is more than ~100 km from where
the receiver's own GPS says YOU are (the normal "drove to another state"
case), the saved anchor is discarded on boot: the arrow would otherwise
point at last-flight landing coordinates, which is wrong and dangerous
now. The display drops to "no target" until fresh link data arrives.
No action needed; just know it can take a boot or two for RF position rows
to repopulate the target.
2. **RF stats page** (button-cycle): note `Bnd:` — should be `Bnd:--`
   after a cold boot. If it already shows an ID and rows of `FGN:` counts,
   you're hearing someone else's beacon; that's fine, see *Crowded channel*
   below.

## On the pad, before arming

3. Power/arm the beacon. Within ~30 s the receiver locks its channel and
   shows the callsign `KE0MZS-<id> CH<n>` (RF mode). That holds during the
   beacon's first minute (a heartbeat every 5 s); after that its quiet pad
   sends one packet a minute, so a receiver booted later starts on the
   channel it last locked and catches the beacon there within ~62 s - on
   any other channel, pick it on Rocket Select. Confirm `<id>` and
   `<n>` match the labels on the airframe and the jumper you mean to fly.
4. **PRE-FLIGHT page** — every row means something you can fix now:
   - `TX LINK OK Ns` counts every packet from the beacon. On the quiet pad
     that is mostly its once-a-minute heartbeat (a position goes out only
     every 10 minutes), so N climbs to ~60 between heartbeats; the row
     drops to `CHK no pkts` after 75 s of silence.
   - `TX LINK WEAK xx dBm` → antenna/IPEX problem: fix before flight.
     (A healthy beacon on the rail is way above -75 dBm.)
   - `TST *KE0MZS-3 CH0*` → the beacon's callsign, starred - it was flashed
     with a TESTING_MODE build (30 s ID cadence). Reflash production before
     the pad unless you did that deliberately. The RF mode page stars the
     callsign the same way. Power-cycling a production beacon does not
     trigger it (its boot ID is followed by HELLO; the flag waits 5 s for
     one).
   - `TX GPS OK n sat` is judged from the newer of the beacon's last
     heartbeat and its last position packet.
   - `TX GPS ACQ n sat` → normal cold start, wait; sats climbing = healthy.
   - `TX GPS CHK wiring!` / `CHK garbled` → GPS UART fault; open the rocket.
   - `TX IMU CHK dead!` → advisory only; the flight still works on GPS
     with the altitude-climb fallback, but you'll lose fused smoothing.
   - `PWR LOW 3.1xV` → the receiver's own supply is sagging (this is the
     exact brown-out scenario that cost us a session once). SWAP THE PACK
     BEFORE FLYING. `READY TO FLY` is blocked while the rail is low.
   - `COMPASS OK 3.3V` → rail healthy, the voltage reads with the name.
   - `SD OK   ALT J5 J6` → the card logs, and the beacon's heartbeat says it
     recognises a commercial altimeter on J5 and J6 (advisory, the verdict
     ignores it). `ALT none` with an altimeter wired → it isn't streaming
     telemetry on the pad (RRC3: pre-launch telemetry; StratoLogger: OnPad)
     or the jack isn't seated; `?` → one stopped talking; `--` → no fresh
     heartbeat yet. LOG.TXT gets an `ALTIM pad` row when it changes.
   - Wait for `** READY TO FLY **`.
5. Quick arrow sanity: walk 10 m away, arrow should track the pad.

## Flight

6. Watch the nav page: `Pkts:` counter climbing, `B:3D`, speed live.
   The row-5 range countdown reads like a fuel gauge for the link —
   when it approaches zero, expect dropouts.
7. Dropouts during boost/flip are normal. The arrow/Dist keep pointing at
   the last good fix; `Last:` age tells you how stale that is.

## Recovery

8. On `LANDED` (row 7 chip), the position is the touchdown anchor. Row 5
   becomes recovery intel:
   - `LANDED` — down, holding position, walk on.
   - Row 7 `LANDED DR` — down, and the beacon has lost its GPS too: the
     target is its last real fix and stays put until the beacon
     re-acquires (dead-reckoned positions never move it).
   - `MOVED +Nm!` — it dragged after touchdown (wind/slope): what you're
     walking to is moving; keep an eye out visually.
   - `CLS N m/m` — closing rate. `AWY N m/m` — you're walking away from it
     (normal on road/terrain detours; it should flip back when you head in).
9. Over a hill we lose it entirely: `Last:` age just climbs — the arrow
   still points to the last known spot, which is where you should walk.
   Once the receiver has heard this beacon launch it stays on the channel
   however long the silence (the SD log notes `FLIGHT CONTACT launched -
   auto-rescan OFF`, then `AUTO-RESCAN suppressed xN`); when you crest the
   rise and packets return, the position snaps to current. Without flight
   contact (pad, a beacon that never flew), 5 minutes of silence starts a
   re-scan for it.
10. If the beacon's GPS dies under cover (heartbeat-only: `HB R<id>` with
    sats low), the position also stays at last-known — walk it, then use
    eyes and the RF strength trend. Its fused packets turn `DR` (dead
    reckoning): they are logged but never move the target, `Last:` keeps
    counting from the last real fix, and BEACON.TXT keeps that fix.

## Crowded channel (someone else on our frequency)

11. `FGN:` counters on RF stats mean foreign packets are being dropped —
    your position is protected by the airframe binding. If *you* become
    the foreign rocket (two airframes, one receiver, channel shared), the
    receiver lets go after ~20 min of only-foreign traffic and adopts the
    live one; to force it sooner, just step the channel manually.

## After recovery

12. Power the beacon down, then read the receiver log:
    - `RX FAULT...` rows on boot = last-boot crash dumps (PC/LR/registers);
      pair the PC against `gps_radio_beacon_receiver.elf` to locate the trap.
    - `TX reset: uptime N -> M cause=...` rows tell you what reset the beacon
      took (POR = fresh battery, WDT = firmware hang, BOD = rail dip).
    - `SD bus stepped down ...` = write trouble detected and bus slowed.
    - `TX reset` count + `RF wedge` count in `tools/log_summary.py <card>`
      remains the headline scorecard; anything non-OK there is a
      hardware report card, not noise.
13. Your own walk is logged too: the receiver writes its own GPS as BASE
    rows every 30 s (including through beacon blackouts). `analyze_flight.py
    --kml` now overlays the operator track on the flight replay, so you can
    see where contact was lost and regained relative to terrain - that is
    the raw material for tuning antenna placement and the re-acquire dwell.
14. Reusing the SD card between launches is now a tested path: boot appends
    a new `L####.TXT` (skipping existing ones) instead of formatting.
    `BEACON.TXT` / `QLOCK.BIN` / `COMPCAL.BIN` persist across sessions by
    design. No field format needed - just extract (`make extract DEV=...`)
    and hand the card back to the beacon, or keep the card seated and live
    in one long history.
