# Arrow-bias measurement procedure

How to reproduce and quantify the "arrow points some degrees off true"
complaint seen in the field (reported ~12:30–1 o'clock at ~1 km).

## Why a procedure and not a bench calibration

The arrow is `compass_heading - bearing_to_last_known`: any bias comes from
one of three layers:

1. **Magnetic declination** at the site. (Now handled automatically by the
   CONUS declination grid, applied at first local GPS fix - this procedure
   measures what remains.)
2. **Fixed mounting offset** of the BNO055 relative to the antenna face.
3. **Soft/hard-iron anomalies** around the case.

Procedure is designed so residual #2 and #3 cancel their own signs over a
quarter-circle sample, leaving a good estimate of #2 alone.

## Procedure (field)

**Critical**: the receiver needs to log rows where you **deliberately point
at the target** — scatter from ordinary walking-around-then-looking-down
rows, which dominate a normal field log, aren't usable. (The existing launch
log already gives false-precision: a whole-file median against the pad sits
at ~76° error even though the arrow was healthy, because most of a walk is
look-at-the-feet time.)

1. Place the beacon (or any GPS-reported target) at a known point. Don't
   move it for the duration.
2. Make the card non-stale: boot the receiver fresh, allow a position fix,
   let scan lock on to the beacon.
3. Open the navigation page — confirm a live target (not the card-restore
   state).
4. Stand ~15-25 m from it; deliberate facing protocol:
   - Face the target, hold the receiver at chest height, square shoulders,
     5 seconds minimum. Walk 10-15° along a circle around the target.
     Repeat at ~8 stations evenly spread.
   - No sideways drift counting: stop, face, then resume.
   - Toss midway: a couple of "trap" walks where you rotate the receiver
     upside-down between facings (to know if roll corruption shows).
5. Stop the recording, pull the card, extract: `make extract DEV=/dev/sdX`.

## Analysis

```
python3 receiver/tools/heading_residuals.py flight_data/<dump>/L000N.TXT
```

Reads the NAV rows and prints a median of `(heading - bearing)` over the
walk. That number is the arrow's persistent bias, in degrees, with sign.

- |median| < 5 deg → within noise, leave compensation alone.
- Systematic negative → arc-length of the receiver's magnetic compass sits
  behind true; turn left by the median systematically (correct by rotating
  the compass-mount offset, not touching declination).
- Systematic positive → converse.

Record the counter clearly in the docs for the next flight day.

## What this tool does NOT do

- It does not explain scatter; single-frame outliers are normal.
- It does not cover pitch/roll effects (compass under tilt) - a vertical
  walk would be a separate procedure.
