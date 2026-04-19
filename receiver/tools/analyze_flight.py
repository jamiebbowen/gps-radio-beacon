#!/usr/bin/env python3
"""
analyze_flight.py - Compute flight statistics from a GPS-radio-beacon NAV log.

Usage:
    python3 analyze_flight.py <path/to/Lxxxxxxx.TXT>
    python3 analyze_flight.py /media/<user>/<sd>/L000314A.TXT

    # Also emit a Google Earth KML for a 3-D animated flight replay:
    python3 analyze_flight.py L000314A.TXT --kml flight.kml
    # Then: open flight.kml in Google Earth Pro -> press the time-slider play
    # button -> Tools > Movie Maker to export MP4.

The NAV log is the CSV file written by the receiver firmware with header:
    Timestamp,Type,BeaconLat,BeaconLon,BeaconAlt_m,BeaconSats,
    BaseLat,BaseLon,Distance_km,Bearing_deg,Heading_deg,RSSI_dBm

Only NAV rows are used. Pre-launch GPS-drift is filtered out by treating
the first sample as ground level and looking for >10 m AGL to detect launch.

All velocity and acceleration numbers are GPS-derived, so they are averaged
over the packet interval (~1.7 s post-launch with continuous-send firmware).
True instantaneous peaks are higher than what GPS sampling can resolve.
"""

import argparse
import datetime as dt
import math
import sys
from pathlib import Path


EARTH_RADIUS_M = 6_371_000.0
LAUNCH_DETECT_AGL_M = 10.0   # altitude above ground to register launch
SOUND_SPEED_MPS = 343.0      # for Mach number (nominal, sea-level 20 C)

# LoRa SX1276 sensitivity floor (dBm) for common spreading factors @ 125 kHz BW.
# Override with --sensitivity if your radio config differs.
DEFAULT_SENSITIVITY_DBM = -123.0  # SF7, 125 kHz (matches current receiver config)


def haversine_m(lat1, lon1, lat2, lon2):
    """Great-circle distance in metres."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(a))


def load_nav_rows(path):
    """Parse the log file, returning a list of NAV row dicts sorted by time."""
    rows = []
    with open(path) as f:
        for line in f:
            parts = line.strip().split(",")
            if len(parts) < 12 or parts[1] != "NAV":
                continue
            try:
                rows.append({
                    "t":        float(parts[0]),
                    "lat":      float(parts[2]),
                    "lon":      float(parts[3]),
                    "alt":      float(parts[4]),
                    "sats":     int(parts[5]),
                    "base_lat": float(parts[6]),
                    "base_lon": float(parts[7]),
                    "dist_km":  float(parts[8]),
                    "rssi":     int(parts[11]),
                })
            except ValueError:
                continue
    rows.sort(key=lambda r: r["t"])
    return rows


def central_diff(rows, key):
    """Central-difference derivative of <key> w.r.t. time, padded at ends."""
    n = len(rows)
    out = [0.0] * n
    for i in range(1, n - 1):
        dt = rows[i + 1]["t"] - rows[i - 1]["t"]
        if dt > 0:
            out[i] = (rows[i + 1][key] - rows[i - 1][key]) / dt
    return out


def find_launch_idx(rows, ground_alt):
    for i, r in enumerate(rows):
        if r["alt"] - ground_alt > LAUNCH_DETECT_AGL_M:
            return i
    return None


def detect_gaps(rows, expected_dt, tolerance=2.0):
    """Return list of (t, gap_seconds) where gap exceeds expected_dt+tolerance."""
    gaps = []
    for i in range(1, len(rows)):
        dt = rows[i]["t"] - rows[i - 1]["t"]
        if dt > expected_dt + tolerance:
            gaps.append((rows[i]["t"], dt))
    return gaps


def cadence_stats(rows):
    if len(rows) < 2:
        return (0.0, 0.0, 0.0)
    dts = [rows[i]["t"] - rows[i - 1]["t"] for i in range(1, len(rows))]
    return (sum(dts) / len(dts), min(dts), max(dts))


def predict_max_range_m(anchor_rssi_dbm, anchor_range_m, sensitivity_dbm):
    """Free-space-path-loss extrapolation of max usable range.

    FSPL scales as 20*log10(d), so a drop of dR dB means a factor of
    10^(dR/20) in distance. This assumes the anchor point is line-of-sight
    with no obstructions, which is the best available model from a single
    RSSI/range pair during flight.
    """
    if anchor_range_m <= 0 or anchor_rssi_dbm >= 0:
        return None
    margin_db = anchor_rssi_dbm - sensitivity_dbm  # positive = headroom
    return anchor_range_m * (10 ** (margin_db / 20.0))


def write_kml(rows, launch_idx, apogee_idx, out_path, base_time=None):
    """Emit a Google-Earth KML with a time-animated 3-D flight replay.

    Produces three things inside the document:
      1. A static coloured LineString of the full trajectory (always visible).
      2. A <gx:Track> with per-sample <when> timestamps so Google Earth's
         time slider animates the rocket position through the flight.
      3. Styled placemarks for Launch Pad, Apogee, and Landing.

    The log's "Timestamp" column is seconds-since-boot, not wall-clock, so we
    synthesize an absolute base time (defaults to today 12:00 UTC). Google
    Earth only cares about relative ordering for animation.
    """
    if base_time is None:
        base_time = dt.datetime.now(dt.timezone.utc).replace(
            hour=12, minute=0, second=0, microsecond=0, tzinfo=None)

    t0 = rows[0]["t"]

    def iso(t_rel):
        return (base_time + dt.timedelta(seconds=t_rel - t0)).strftime(
            "%Y-%m-%dT%H:%M:%SZ")

    pad     = rows[launch_idx]
    apogee  = rows[apogee_idx]
    landing = rows[-1]

    def pm(name, row, icon, colour, description=""):
        return (
            f'<Placemark><name>{name}</name>'
            f'<description><![CDATA[{description}]]></description>'
            f'<Style><IconStyle><color>{colour}</color>'
            f'<Icon><href>{icon}</href></Icon></IconStyle></Style>'
            f'<Point><altitudeMode>absolute</altitudeMode>'
            f'<coordinates>{row["lon"]:.7f},{row["lat"]:.7f},{row["alt"]:.1f}'
            f'</coordinates></Point></Placemark>'
        )

    # Static trajectory line (red, 3 px, extruded to ground for altitude feel)
    coord_str = " ".join(
        f"{r['lon']:.7f},{r['lat']:.7f},{r['alt']:.1f}" for r in rows)

    # gx:Track for time-based animation
    when_lines  = "\n        ".join(f"<when>{iso(r['t'])}</when>" for r in rows)
    coord_lines = "\n        ".join(
        f"<gx:coord>{r['lon']:.7f} {r['lat']:.7f} {r['alt']:.1f}</gx:coord>"
        for r in rows)

    flight_duration = rows[-1]["t"] - pad["t"]
    apogee_agl = apogee["alt"] - rows[0]["alt"]

    doc = f"""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:gx="http://www.google.com/kml/ext/2.2">
<Document>
  <name>Flight Replay - {Path(out_path).stem}</name>
  <description><![CDATA[
    Launch time: T+{pad['t']:.1f}s (log time)<br/>
    Flight duration: {flight_duration:.1f}s<br/>
    Apogee: {apogee['alt']:.0f} m MSL ({apogee_agl:.0f} m AGL)<br/>
    Packets: {len(rows)}
  ]]></description>

  <Style id="trajectory">
    <LineStyle><color>ff0000ff</color><width>3</width></LineStyle>
    <PolyStyle><color>4000ffff</color></PolyStyle>
  </Style>

  <Style id="rocketTrack">
    <IconStyle>
      <color>ff00ffff</color>
      <Icon><href>http://maps.google.com/mapfiles/kml/shapes/rocket.png</href></Icon>
    </IconStyle>
    <LineStyle><color>ff00ffff</color><width>4</width></LineStyle>
  </Style>

  <Placemark>
    <name>Flight Path</name>
    <styleUrl>#trajectory</styleUrl>
    <LineString>
      <extrude>1</extrude>
      <tessellate>1</tessellate>
      <altitudeMode>absolute</altitudeMode>
      <coordinates>{coord_str}</coordinates>
    </LineString>
  </Placemark>

  <Placemark>
    <name>Rocket (animated)</name>
    <styleUrl>#rocketTrack</styleUrl>
    <gx:Track>
      <altitudeMode>absolute</altitudeMode>
      <extrude>1</extrude>
        {when_lines}
        {coord_lines}
    </gx:Track>
  </Placemark>

  {pm("Launch Pad", pad,
      "http://maps.google.com/mapfiles/kml/paddle/grn-blank.png",
      "ff00ff00",
      f"Lat {pad['lat']:.6f}, Lon {pad['lon']:.6f}<br/>Ground: {pad['alt']:.0f} m MSL")}
  {pm("Apogee", apogee,
      "http://maps.google.com/mapfiles/kml/paddle/ylw-stars.png",
      "ff00ffff",
      f"{apogee['alt']:.0f} m MSL ({apogee_agl:.0f} m AGL)<br/>"
      f"T+{apogee['t'] - pad['t']:.1f}s after launch")}
  {pm("Landing", landing,
      "http://maps.google.com/mapfiles/kml/paddle/red-blank.png",
      "ff0000ff",
      f"Last packet: {landing['alt']:.0f} m MSL<br/>"
      f"Lat {landing['lat']:.6f}, Lon {landing['lon']:.6f}")}
</Document>
</kml>
"""
    Path(out_path).write_text(doc)


def write_mp4(rows, launch_idx, apogee_idx, out_path,
              fps=30, speedup=1.0, duration_s=None, pre_launch_s=2.0):
    """Render a 3-D animated flight replay directly to MP4 via ffmpeg.

    Fully headless (no Google Earth needed). Requires matplotlib + numpy and
    a working ``ffmpeg`` binary on PATH.

    Parameters
    ----------
    fps          : output video frame rate.
    speedup      : playback speed multiplier; 1.0 = real time, 4.0 = 4x.
    duration_s   : if set, overrides speedup to fit the whole flight into this
                   many seconds of video.
    pre_launch_s : seconds of pre-launch pad time to include as lead-in.
                   Set to 0 to start exactly at launch.
    """
    try:
        import numpy as np
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation, FFMpegWriter
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    except ImportError as e:
        print(f"MP4 export needs matplotlib + numpy: {e}", file=sys.stderr)
        print("  pip install matplotlib", file=sys.stderr)
        return False

    # --- Pick the launch window: from pre_launch_s before launch through the
    # last sample. This avoids hundreds of seconds of pad-idle time that
    # would otherwise dominate the video.
    pad = rows[launch_idx]
    cutoff_t = pad["t"] - max(pre_launch_s, 0.0)
    start_idx = launch_idx
    while start_idx > 0 and rows[start_idx - 1]["t"] >= cutoff_t:
        start_idx -= 1
    flight_rows = rows[start_idx:]
    new_apogee_idx = apogee_idx - start_idx
    new_launch_idx = launch_idx - start_idx

    # --- Choose the true launch-pad reference.
    # rows[launch_idx] is the first packet AFTER launch detection; at ~1.7 s
    # cadence and hundreds of m/s ascent velocity, the rocket may already be
    # ~900 m up and several hundred metres downrange in that sample. So we
    # can't use it as the ENU origin.
    #
    # Using ALL pre-launch samples is also wrong if the beacon/RX is powered
    # on before the rocket reaches the pad (prep tent, RV, check-in): those
    # early packets are logged from wherever the rocket was at the time, and
    # would pull the median away from the true pad.
    #
    # Instead, use samples in the final PAD_WINDOW_S seconds before launch -
    # these are captured while the rocket is stationary on the rail waiting
    # for ignition. Falls back to all pre-launch samples if the window is
    # empty (very short pad time).
    PAD_WINDOW_S = 60.0
    launch_t = rows[launch_idx]["t"]
    pad_samples = [r for r in rows[:launch_idx]
                   if launch_t - r["t"] <= PAD_WINDOW_S]
    if len(pad_samples) < 3:
        pad_samples = rows[:launch_idx] if launch_idx > 0 else [rows[0]]

    pad_ref_lat = float(np.median([r["lat"] for r in pad_samples]))
    pad_ref_lon = float(np.median([r["lon"] for r in pad_samples]))
    ground_alt  = float(np.median([r["alt"] for r in pad_samples]))

    # --- Project lat/lon/alt into local East-North-Up metres around the pad.
    pad_lat_rad = math.radians(pad_ref_lat)
    m_per_deg_lat = 111_320.0
    m_per_deg_lon = 111_320.0 * math.cos(pad_lat_rad)

    t_arr = np.array([r["t"] - pad["t"] for r in flight_rows])  # s since launch
    east  = np.array([(r["lon"] - pad_ref_lon) * m_per_deg_lon for r in flight_rows])
    north = np.array([(r["lat"] - pad_ref_lat) * m_per_deg_lat for r in flight_rows])
    up    = np.array([r["alt"] - ground_alt                    for r in flight_rows])
    # Clip tiny GPS jitter below 0 so the pre-launch lead-in sits on the plane
    up    = np.maximum(up, 0.0)
    rssi  = np.array([r["rssi"] for r in flight_rows])

    # --- Resample onto an evenly-spaced timeline for smooth playback
    flight_t0 = float(t_arr[0])     # negative if pre-launch lead-in included
    flight_tN = float(t_arr[-1])
    flight_secs = flight_tN - flight_t0
    if duration_s is not None and duration_s > 0:
        video_secs = duration_s
    else:
        video_secs = flight_secs / max(speedup, 0.01)
    n_frames = max(int(video_secs * fps), 2)

    t_video = np.linspace(flight_t0, flight_tN, n_frames)
    x_f = np.interp(t_video, t_arr, east)
    y_f = np.interp(t_video, t_arr, north)
    z_f = np.interp(t_video, t_arr, up)
    r_f = np.interp(t_video, t_arr, rssi)
    apogee_idx = new_apogee_idx
    launch_idx = new_launch_idx

    # Render-time heads-up. Matplotlib 3D is roughly 0.3-0.5 s/frame at this
    # resolution, so real-time renders of multi-minute flights take 15+ min.
    est_min = n_frames * 0.4 / 60.0
    print(f"  Frames: {n_frames}  ({video_secs:.1f}s @ {fps}fps)  "
          f"~estimated render: {est_min:.1f} min")
    if video_secs > 45 and duration_s is None:
        print("  TIP: use --duration 30 or --speedup 4 for a faster render.")

    # --- Figure + axes
    fig = plt.figure(figsize=(12, 8), facecolor="black")
    ax = fig.add_subplot(111, projection="3d", facecolor="black")

    pad_m = 100.0
    x_range = (float(min(east.min(),  -pad_m)), float(max(east.max(),   pad_m)))
    y_range = (float(min(north.min(), -pad_m)), float(max(north.max(),  pad_m)))
    z_max = float(max(up.max() * 1.05, 100.0))

    ax.set_xlim(x_range); ax.set_ylim(y_range); ax.set_zlim(0, z_max)
    ax.set_xlabel("East (m)",  color="white")
    ax.set_ylabel("North (m)", color="white")
    ax.set_zlabel("Alt AGL (m)", color="white")
    ax.tick_params(colors="white")
    for pane in (ax.xaxis, ax.yaxis, ax.zaxis):
        pane.pane.set_facecolor((0.02, 0.02, 0.05, 1.0))
        pane.pane.set_edgecolor((0.2, 0.2, 0.3, 1.0))

    # Ground plane mesh (simple translucent)
    gx, gy = np.meshgrid(
        np.linspace(x_range[0], x_range[1], 8),
        np.linspace(y_range[0], y_range[1], 8))
    ax.plot_surface(gx, gy, np.zeros_like(gx),
                    color=(0.15, 0.25, 0.15), alpha=0.35, linewidth=0)

    # Full trajectory (faint, always visible) for context
    ax.plot(east, north, up, color=(1, 0.4, 0.2), linewidth=0.8, alpha=0.35)

    # Animated elements
    trail_line,  = ax.plot([], [], [], color="cyan",   linewidth=2.2)
    rocket_pt,   = ax.plot([], [], [], "o",            color="yellow",
                           markersize=8, markeredgecolor="white")
    ground_shadow, = ax.plot([], [], [], color="white", linewidth=0.6, alpha=0.4)

    # Static markers
    ax.scatter([0], [0], [0], color="lime",   s=60,
               marker="^", label="Launch pad")
    ax.scatter([east[apogee_idx]], [north[apogee_idx]], [up[apogee_idx]],
               color="gold", s=80, marker="*", label="Apogee")
    ax.scatter([east[-1]], [north[-1]], [up[-1]],
               color="red", s=60, marker="v", label="Last packet")
    ax.legend(loc="upper right", facecolor="black", edgecolor="white",
              labelcolor="white", framealpha=0.7, fontsize=9,
              borderpad=0.6)

    # HUD text
    hud = ax.text2D(0.02, 0.95, "", transform=ax.transAxes,
                    color="white", fontsize=11, family="monospace",
                    verticalalignment="top",
                    bbox=dict(facecolor="black", edgecolor="white",
                              alpha=0.55, boxstyle="round,pad=0.4"))
    title = ax.text2D(0.5, 0.02, f"Flight Replay  -  {Path(out_path).stem}",
                      transform=ax.transAxes, color="white", fontsize=13,
                      ha="center")
    _ = title  # keep reference

    # Camera orbits slowly around the flight volume
    def azim_at(frame):
        return -60.0 + 30.0 * frame / n_frames

    def init():
        trail_line.set_data([], []); trail_line.set_3d_properties([])
        rocket_pt.set_data([], []);  rocket_pt.set_3d_properties([])
        ground_shadow.set_data([], []); ground_shadow.set_3d_properties([])
        return trail_line, rocket_pt, ground_shadow, hud

    def update(frame):
        i = frame + 1  # number of samples to include
        trail_line.set_data(x_f[:i], y_f[:i])
        trail_line.set_3d_properties(z_f[:i])
        rocket_pt.set_data([x_f[frame]], [y_f[frame]])
        rocket_pt.set_3d_properties([z_f[frame]])
        ground_shadow.set_data(x_f[:i], y_f[:i])
        ground_shadow.set_3d_properties(np.zeros(i))

        hud.set_text(
            f"T+{t_video[frame]:6.1f} s\n"
            f"Alt:  {z_f[frame]:6.0f} m AGL\n"
            f"East: {x_f[frame]:+6.0f} m\n"
            f"North:{y_f[frame]:+6.0f} m\n"
            f"RSSI: {r_f[frame]:+5.0f} dBm"
        )
        ax.view_init(elev=22, azim=azim_at(frame))
        return trail_line, rocket_pt, ground_shadow, hud

    anim = FuncAnimation(fig, update, frames=n_frames,
                         init_func=init, interval=1000 / fps, blit=False)

    import time
    _t_start = time.monotonic()

    def _progress(frame, total):
        # Print every ~5% so we don't spam; also always first/last.
        step = max(total // 20, 1)
        if frame == 0 or frame == total - 1 or frame % step == 0:
            elapsed = time.monotonic() - _t_start
            frac = (frame + 1) / total
            eta = (elapsed / frac) - elapsed if frac > 0 else 0
            sys.stdout.write(
                f"\r  rendering: {frame + 1:4d}/{total}  "
                f"({100 * frac:5.1f}%)  elapsed {elapsed:5.1f}s  "
                f"ETA {eta:5.1f}s   "
            )
            sys.stdout.flush()

    try:
        writer = FFMpegWriter(fps=fps, bitrate=2500,
                              codec="libx264",
                              extra_args=["-pix_fmt", "yuv420p"])
        anim.save(out_path, writer=writer, dpi=100,
                  savefig_kwargs={"facecolor": "black"},
                  progress_callback=_progress)
        sys.stdout.write("\n")
    except (RuntimeError, FileNotFoundError) as e:
        print(f"ffmpeg failed: {e}", file=sys.stderr)
        print("  Install ffmpeg (e.g. 'sudo apt install ffmpeg') and retry.",
              file=sys.stderr)
        plt.close(fig)
        return False

    plt.close(fig)
    return True


def analyze(log_path, sensitivity_dbm=DEFAULT_SENSITIVITY_DBM,
            kml_path=None, mp4_path=None, mp4_fps=30,
            mp4_speedup=1.0, mp4_duration=None, mp4_pre_launch_s=2.0):
    rows = load_nav_rows(log_path)
    if len(rows) < 5:
        print(f"Not enough NAV rows in {log_path} ({len(rows)} found)")
        return 1

    ground_alt = rows[0]["alt"]
    pad_lat, pad_lon = rows[0]["lat"], rows[0]["lon"]

    launch_idx = find_launch_idx(rows, ground_alt)
    if launch_idx is None:
        print("No launch detected (altitude never rose >10 m above ground).")
        print(f"Log spans {rows[-1]['t'] - rows[0]['t']:.1f}s, "
              f"{len(rows)} packets, altitude {rows[0]['alt']:.0f}-"
              f"{max(r['alt'] for r in rows):.0f} m")
        return 0

    launch_t = rows[launch_idx]["t"]
    pre  = [r for r in rows if r["t"] <  launch_t]
    post = [r for r in rows if r["t"] >= launch_t]

    apogee = max(rows, key=lambda r: r["alt"])
    apogee_idx = rows.index(apogee)
    apogee_agl = apogee["alt"] - ground_alt

    # Distance from pad, ground-track length
    for r in rows:
        r["r_pad_m"] = haversine_m(pad_lat, pad_lon, r["lat"], r["lon"])
    gt_m = sum(
        haversine_m(rows[i - 1]["lat"], rows[i - 1]["lon"],
                    rows[i]["lat"],     rows[i]["lon"])
        for i in range(1, len(rows))
    )
    max_drift = max(rows, key=lambda r: r["r_pad_m"])

    # Vertical velocity/acceleration
    vz = central_diff(rows, "alt")
    max_ascent = max(vz[launch_idx:apogee_idx + 1], default=0.0)
    max_descent = min(vz[apogee_idx:], default=0.0)

    max_acc_g = 0.0
    max_acc_t = 0.0
    for i in range(1, len(vz)):
        dt = rows[i]["t"] - rows[i - 1]["t"]
        if dt > 0:
            a = (vz[i] - vz[i - 1]) / dt
            if abs(a) > abs(max_acc_g):
                max_acc_g = a
                max_acc_t = rows[i]["t"]

    # Slant range (3-D distance from pad) for every row + peak
    for r in rows:
        r["slant_m"] = math.sqrt(
            r["r_pad_m"] ** 2 + max(0.0, r["alt"] - ground_alt) ** 2)
    max_slant_m = max(r["slant_m"] for r in rows)

    # Cadence & link
    pre_mean, pre_min, pre_max = cadence_stats(pre)
    post_mean, post_min, post_max = cadence_stats(post)
    rssi_all = [r["rssi"] for r in rows]
    rssi_post = [r["rssi"] for r in post]
    worst_rssi = min(rows, key=lambda r: r["rssi"])

    # Range-prediction anchor: worst RSSI during flight paired with its
    # actual slant range at that moment (not horizontal range).
    if post:
        anchor = min(post, key=lambda r: r["rssi"])
        max_range_m = predict_max_range_m(
            anchor["rssi"], anchor["slant_m"], sensitivity_dbm)
        link_margin_db = anchor["rssi"] - sensitivity_dbm
    else:
        anchor = None
        max_range_m = None
        link_margin_db = None

    # Flight-phase gaps (>3 s while transmitter is in continuous-send)
    flight_gaps = detect_gaps(post, post_mean, tolerance=1.5) if post else []

    # ---------- output ----------
    print("=" * 62)
    print(f"FLIGHT ANALYSIS  ({Path(log_path).name})")
    print("=" * 62)
    print(f"Launch pad:      {pad_lat:.6f}, {pad_lon:.6f}")
    print(f"Ground (MSL):    {ground_alt:.0f} m ({ground_alt * 3.281:.0f} ft)")
    print(f"Packets total:   {len(rows)}  (pre: {len(pre)}, post: {len(post)})")
    print(f"Launch detected: log-time T+{launch_t:.2f}s  (>10 m AGL)")
    print(f"Flight duration: {rows[-1]['t'] - launch_t:.1f} s")
    print(f"Time to apogee:  {apogee['t'] - launch_t:.1f} s")
    print()
    print("--- ALTITUDE ---")
    print(f"  Apogee MSL:       {apogee['alt']:7.0f} m  ({apogee['alt'] * 3.281:.0f} ft)")
    print(f"  Apogee AGL:       {apogee_agl:7.0f} m  ({apogee_agl * 3.281:.0f} ft)")
    print(f"  Final altitude:   {rows[-1]['alt']:7.0f} m MSL  "
          f"({rows[-1]['alt'] - ground_alt:.0f} m AGL)")
    print()
    print("--- VELOCITY / ACCEL  (GPS @ ~{:.1f}s sampling; peaks underestimated) ---".format(post_mean))
    print(f"  Max ascent rate:  {max_ascent:7.1f} m/s  "
          f"({max_ascent * 3.281:.0f} ft/s, Mach {max_ascent / SOUND_SPEED_MPS:.2f})")
    print(f"  Max descent rate: {max_descent:7.1f} m/s  "
          f"({abs(max_descent) * 3.281:.0f} ft/s)")
    print(f"  Peak |accel| est: {max_acc_g:7.1f} m/s^2  "
          f"({abs(max_acc_g) / 9.81:.1f} G)  at T+{max_acc_t - launch_t:.1f}s")
    print()
    print("--- HORIZONTAL / RECOVERY ---")
    print(f"  Max drift (pad):  {max_drift['r_pad_m']:7.0f} m  "
          f"at T+{max_drift['t'] - launch_t:.0f}s")
    print(f"  Ground track:     {gt_m:7.0f} m  ({gt_m / 1000:.2f} km)")
    print(f"  Max slant range:  {max_slant_m:7.0f} m")
    print(f"  Last position:    {rows[-1]['lat']:.6f}, {rows[-1]['lon']:.6f}")
    print(f"  Landing offset:   {rows[-1]['r_pad_m']:7.0f} m from pad")
    print()
    print("--- RF LINK ---")
    print(f"  Pre-launch cadence:  {pre_mean:4.2f}s "
          f"(min {pre_min:.2f}, max {pre_max:.2f})")
    print(f"  Post-launch cadence: {post_mean:4.2f}s "
          f"(min {post_min:.2f}, max {post_max:.2f})")
    if pre_mean > 0 and post_mean > 0:
        print(f"  TX rate boost:       {pre_mean / post_mean:.1f}x")
    print(f"  RSSI range:          {min(rssi_all)} to {max(rssi_all)} dBm  "
          f"(mean {sum(rssi_all) / len(rssi_all):.1f})")
    if rssi_post:
        print(f"  RSSI mean in flight: {sum(rssi_post) / len(rssi_post):.1f} dBm")
    print(f"  Worst RSSI:          {worst_rssi['rssi']} dBm at T+"
          f"{worst_rssi['t'] - launch_t:.0f}s  "
          f"(alt {worst_rssi['alt']:.0f}m, range {worst_rssi['r_pad_m']:.0f}m)")
    print()
    print("--- PREDICTED RANGE  (FSPL extrapolation, line-of-sight) ---")
    print(f"  Assumed sensitivity:  {sensitivity_dbm:.0f} dBm  "
          f"(LoRa SF7 / 125 kHz default; override with --sensitivity)")
    if anchor and max_range_m is not None:
        print(f"  Anchor sample:        RSSI {anchor['rssi']} dBm at "
              f"slant range {anchor['slant_m']:.0f} m  "
              f"(T+{anchor['t'] - launch_t:.0f}s)")
        print(f"  Link margin at anchor: {link_margin_db:.1f} dB")
        print(f"  Predicted max range:  {max_range_m:7.0f} m  "
              f"({max_range_m / 1000:.1f} km, {max_range_m * 3.281 / 5280:.1f} mi)")
        reached_pct = (max_slant_m / max_range_m * 100.0) if max_range_m > 0 else 0
        print(f"  Reached this flight:   {max_slant_m:.0f} m  "
              f"({reached_pct:.1f}% of predicted max)")
    else:
        print("  Insufficient in-flight data to estimate range.")
    print()
    print("--- LINK CONTINUITY (in-flight gaps) ---")
    if not flight_gaps:
        print("  No in-flight packet gaps > expected cadence + 1.5s")
        print("  -> Signal was never lost due to range.")
    else:
        print(f"  {len(flight_gaps)} gap(s) detected during flight:")
        for t, gdt in flight_gaps[:20]:
            row = next(r for r in rows if r["t"] == t)
            print(f"    T+{t - launch_t:6.1f}s  gap={gdt:5.2f}s  "
                  f"alt={row['alt']:.0f}m  range={row['r_pad_m']:.0f}m")

    if kml_path:
        write_kml(rows, launch_idx, apogee_idx, kml_path)
        print()
        print(f"KML written: {kml_path}")
        print("  Open in Google Earth Pro; use the time slider to replay the")
        print("  flight. To export MP4: Tools -> Movie Maker -> record tour.")

    if mp4_path:
        print()
        print(f"Rendering MP4 to {mp4_path} ...")
        ok = write_mp4(rows, launch_idx, apogee_idx, mp4_path,
                       fps=mp4_fps, speedup=mp4_speedup,
                       duration_s=mp4_duration,
                       pre_launch_s=mp4_pre_launch_s)
        if ok:
            print(f"MP4 written: {mp4_path}")
        else:
            print("MP4 render failed (see error above).")
            return 3
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("log", help="Path to NAV log file (Lxxxxxxx.TXT)")
    ap.add_argument("--sensitivity", type=float, default=DEFAULT_SENSITIVITY_DBM,
                    help=f"Receiver sensitivity floor in dBm for range prediction "
                         f"(default {DEFAULT_SENSITIVITY_DBM} = LoRa SF7 @ 125 kHz)")
    ap.add_argument("--kml", metavar="PATH",
                    help="Write a Google-Earth KML of the flight to PATH "
                         "(includes time-animated rocket track for Movie Maker)")
    ap.add_argument("--mp4", metavar="PATH",
                    help="Render a 3-D animated replay directly to MP4. "
                         "Requires matplotlib + ffmpeg.")
    ap.add_argument("--fps", type=int, default=30,
                    help="MP4 frame rate (default 30)")
    ap.add_argument("--speedup", type=float, default=1.0,
                    help="MP4 playback speed multiplier: 1=real time, "
                         "4=4x faster (default 1.0)")
    ap.add_argument("--duration", type=float, default=None,
                    help="MP4 total duration in seconds; overrides --speedup "
                         "when set")
    ap.add_argument("--pre-launch", type=float, default=2.0,
                    help="Seconds of pre-launch pad time to show as lead-in "
                         "(default 2.0; use 0 to start exactly at launch)")
    args = ap.parse_args()
    if not Path(args.log).is_file():
        print(f"File not found: {args.log}", file=sys.stderr)
        return 2
    return analyze(args.log,
                   sensitivity_dbm=args.sensitivity,
                   kml_path=args.kml,
                   mp4_path=args.mp4,
                   mp4_fps=args.fps,
                   mp4_speedup=args.speedup,
                   mp4_duration=args.duration,
                   mp4_pre_launch_s=args.pre_launch)


if __name__ == "__main__":
    sys.exit(main())
