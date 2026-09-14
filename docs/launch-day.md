# Launch Day Runbook

Field sequence, built around what the receiver actually shows. Assumes the
bench checklist (`bench-checklist.md`) passed this weekend.

## Arriving at the site

1. **Power the receiver.** Watch boot: RF init OK, SD ready, compass
   calibration restored. Last Beacon position showing is fine (it's the
   previous flight's, loaded from the card).
2. **RF stats page** (button-cycle): note `Bnd:` — should be `Bnd:--`
   after a cold boot. If it already shows an ID and rows of `FGN:` counts,
   you're hearing someone else's beacon; that's fine, see *Crowded channel*
   below.

## On the pad, before arming

3. Power/arm the beacon. Within ~30 s the receiver locks its channel and
   shows the callsign `KE0MZS-<id> CH<n>` (RF mode). Confirm `<id>` and
   `<n>` match the labels on the airframe and the jumper you mean to fly.
4. **PRE-FLIGHT page** — every row means something you can fix now:
   - `TX LINK WEAK xx dBm` → antenna/IPEX problem: fix before flight.
     (A healthy beacon on the rail is way above -75 dBm.)
   - `TST *KE0MZS-3 CH0*` → the beacon's callsign, starred - it was flashed
     with a TESTING_MODE build (30 s ID cadence). Reflash production before
     the pad unless you did that deliberately. The RF mode page stars the
     callsign the same way.
   - `TX GPS ACQ n sat` → normal cold start, wait; sats climbing = healthy.
   - `TX GPS CHK wiring!` / `CHK garbled` → GPS UART fault; open the rocket.
   - `TX IMU CHK dead!` → advisory only; the flight still works on GPS
     with the altitude-climb fallback, but you'll lose fused smoothing.
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
   - `MOVED +Nm!` — it dragged after touchdown (wind/slope): what you're
     walking to is moving; keep an eye out visually.
   - `CLS N m/m` — closing rate. `AWY N m/m` — you're walking away from it
     (normal on road/terrain detours; it should flip back when you head in).
9. Over a hill we lose it entirely: `Last:` age just climbs — the arrow
   still points to the last known spot, which is where you should walk.
   Past ~5 minutes of silence the receiver re-scans for the beacon on its
   own; when you crest the rise and packets return, the position snaps to
   current.
10. If the beacon's GPS dies under cover (heartbeat-only: `HB R<id>` with
    sats low), the position also stays at last-known — walk it, then use
    eyes and the RF strength trend.

## Crowded channel (someone else on our frequency)

11. `FGN:` counters on RF stats mean foreign packets are being dropped —
    your position is protected by the airframe binding. If *you* become
    the foreign rocket (two airframes, one receiver, channel shared), the
    receiver lets go after ~20 min of only-foreign traffic and adopts the
    live one; to force it sooner, just step the channel manually.

## After recovery

12. Power the beacon down, note `TX reset` events and `RF wedge` counts in
    `tools/log_summary.py <card>` output; anything non-OK there is a
    hardware report card, not noise.
