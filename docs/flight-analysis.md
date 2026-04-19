# Flight analysis

`receiver/tools/analyze_flight.py` turns a receiver SD-card log into:

1. A printed summary of altitude, velocity, horizontal drift, and RF link stats
2. A free-space-path-loss prediction of the maximum usable range
3. (Optional) A Google-Earth KML with a time-animated flight replay
4. (Optional) A headless MP4 3-D flight replay rendered via matplotlib + ffmpeg

The log is the plain-text CSV the receiver firmware writes to the SD card.
Only `NAV` rows (with GPS + RSSI) are analysed; other row types are ignored.

## Requirements

- Python 3.8+ (standard library only for analysis + KML)
- `matplotlib` and a working `ffmpeg` on `PATH` **only if** you want MP4 output

```bash
pip install matplotlib
sudo apt install ffmpeg        # or the equivalent on your OS
```

## Usage

```bash
python3 receiver/tools/analyze_flight.py /path/to/Lxxxxxxx.TXT [OPTIONS]
```

### Options

| Flag | Default | Purpose |
|---|---|---|
| `--sensitivity DBM` | `-123` | Receiver sensitivity floor for range prediction (LoRa SF7 / 125 kHz). Use `-134` for SF10. |
| `--kml PATH` | off | Write a Google-Earth KML of the flight (trajectory + animated rocket + pad/apogee/landing placemarks). |
| `--mp4 PATH` | off | Render a 3-D animated MP4 directly. Requires `matplotlib` + `ffmpeg`. |
| `--fps N` | 30 | MP4 frame rate. |
| `--speedup N` | 1.0 | MP4 playback speed (1.0 = real-time, 4.0 = 4x faster). |
| `--duration N` | auto | MP4 total duration in seconds; overrides `--speedup`. |
| `--pre-launch N` | 2.0 | Seconds of pad lead-in shown in the MP4 before T+0. |

## Output sections

The printed summary has six sections:

```
FLIGHT ANALYSIS  (Lxxxxxxx.TXT)
--- ALTITUDE                        apogee MSL/AGL, final altitude
--- VELOCITY / ACCEL                peak ascent/descent rate, peak |a| in G
--- HORIZONTAL / RECOVERY           max drift, ground track, landing offset
--- RF LINK                         pre/post-launch cadence, RSSI range
--- PREDICTED RANGE                 FSPL extrapolation from worst RSSI
--- LINK CONTINUITY                 in-flight packet gaps
```

### Altitude

Ground level is taken from the **first pre-launch sample**. Launch is
declared when altitude rises >10 m above ground. AGL values are relative
to that reference.

### Velocity / acceleration

Derivatives are computed by central difference over the packet cadence
(~1.7 s during flight with the continuous-send firmware). These peaks are
**always underestimates** of the true instantaneous values -- a real Mach
1 rocket reaches ~340 m/s in under a second, so with 1.7 s sampling the
averaged peak can be much lower than reality.

### Horizontal / recovery

Uses a flat-Earth ENU projection centered on the pad -- good to ~0.1 m for
the few-km scales typical of an amateur flight. Ground track is the
integrated path length, not straight-line displacement.

### RF link

Pre-launch cadence is typically 5 s (battery save); post-launch is ~1.7 s
with continuous-send enabled. The "TX rate boost" column shows how much
faster the in-flight data comes in.

### Predicted range

Takes the **worst in-flight RSSI** paired with the **slant range at that
moment** (i.e., includes altitude as well as horizontal distance). Applies
free-space path loss (20 * log10(d)) to extrapolate to the sensitivity
floor.

Caveats:

- Assumes free space -- real horizon, terrain, and antenna patterns will
  limit range well before this number.
- Uses a single RSSI/range pair; noisier fits may give larger predictions
  than the real link can deliver.
- Override `--sensitivity` to match your actual LoRa configuration.

### Link continuity

Reports in-flight packet gaps exceeding the expected cadence + 1.5 s. A
flight with zero gaps means signal was never lost -- the log ending does
not imply signal loss (usually the receiver was powered off after recovery).

## Pad-reference robustness

Two subtle-but-important fixes live in the MP4 renderer:

- The **vertical reference** uses the first pre-launch sample's altitude,
  not `rows[launch_idx]["alt"]`. The launch-detect sample can already be
  hundreds of metres up because of the 1.7 s cadence, and using it would
  place the descent below the ground plane.
- The **horizontal reference** uses the median of pre-launch samples
  captured within the last 60 s before launch. This handles the common
  case where the beacon/RX is powered on in the prep tent and walked to
  the pad -- using all pre-launch samples would drift the pad marker.

If your log shows the rocket still airborne at the last packet (log ended
mid-flight), the "Landing offset" in the summary is really the
last-known-position offset, not the actual landing point.

## Google-Earth replay (`--kml`)

Produces a KML with:

- A static red LineString of the full trajectory (altitude-accurate)
- A `<gx:Track>` with per-packet timestamps that Google Earth's time
  slider replays in 3-D
- Styled placemarks for launch pad (green), apogee (yellow star), and the
  last packet (red)

Open the file in **Google Earth Pro** (free). To export video:
`Tools > Movie Maker`.

The web version of Google Earth renders the trajectory but does not have
Movie Maker.

## Headless MP4 (`--mp4`)

Fully scripted; no GUI needed. Produces:

- 3-D plot in local ENU metres around the pad
- Cyan trailing line showing the traversed path
- Yellow moving rocket marker with ground-shadow trail
- Static placemarks (launch pad, apogee, last packet)
- Live HUD: T+, altitude AGL, east/north offset, RSSI
- Slowly orbiting camera
- Progress indicator during render

Matplotlib 3-D rendering is slow (~0.3-0.5 s per frame at 100 dpi). A
real-time render of a 2-minute flight takes ~15 minutes. For quick
previews use `--speedup 8` or `--duration 15`.

## Example

```bash
$ python3 receiver/tools/analyze_flight.py /media/you/SD/L000314A.TXT \
      --kml flight.kml --mp4 flight.mp4 --duration 20

FLIGHT ANALYSIS  (L000314A.TXT)
Launch pad:      38.156120, -104.808723
Ground (MSL):    1667 m (5469 ft)
Flight duration: 125.1 s
Time to apogee:  16.1 s

--- ALTITUDE ---
  Apogee AGL:   2041 m  (6697 ft)

--- VELOCITY / ACCEL ---
  Max ascent rate:  339.1 m/s (Mach 0.99)

--- RF LINK ---
  Worst RSSI: -86 dBm at T+20s

--- PREDICTED RANGE ---
  Link margin at anchor: 37.0 dB
  Predicted max range:   150 km  (FSPL, LOS)
  Reached this flight:   2.1 km  (1.4% of predicted max)

--- LINK CONTINUITY ---
  No in-flight packet gaps > expected cadence + 1.5s

KML written: flight.kml
Rendering MP4 to flight.mp4 ...
  Frames: 600  (20.0s @ 30fps)  ~estimated render: 4.0 min
  rendering:  600/600  (100.0%)  elapsed 245.1s  ETA   0.0s
MP4 written: flight.mp4
```
