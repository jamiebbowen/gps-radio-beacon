#!/usr/bin/env python3
"""Host-side unit tests for the GPS-radio-beacon analysis tools.

Covers analyze_flight.py and calibrate_rf.py with SYNTHETIC logs so the
suite runs on any fresh clone (the Makefile's tools-test smoke check
needs flight_data/*.TXT, which is gitignored and only exists on a
machine that has pulled a real SD card).

Run:  python3 -m unittest discover -s receiver/tools/tests -v
(No third-party deps — the matplotlib/numpy paths are not exercised.)
"""
from __future__ import annotations

import contextlib
import io
import math
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(TOOLS_DIR))

import analyze_flight as af  # noqa: E402
import calibrate_rf as crf  # noqa: E402


# ── Synthetic-log builders ────────────────────────────────────────────

HEADER = ("Timestamp,Type,PktSrc,BeaconLat,BeaconLon,BeaconAlt_m,BeaconSats,"
          "BaseLat,BaseLon,BaseAlt_m,Distance_km,Bearing_deg,Heading_deg,"
          "VelE_mps,VelN_mps,VelU_mps,Spd2D_mps,RSSI_dBm,SNR_dB")

PAD_LAT, PAD_LON, PAD_ALT = 45.0, -75.0, 100.0


def nav_row(t, lat, lon, alt, rssi=-60, snr=8, dist_km=None, base_alt=PAD_ALT):
    if dist_km is None:
        dist_km = af.haversine_m(PAD_LAT, PAD_LON, lat, lon) / 1000.0
    return (f"{t:.2f},NAV,L, {lat:.7f},{lon:.7f},{alt:.1f},9,"
            f"{PAD_LAT:.7f},{PAD_LON:.7f},{base_alt:.1f},{dist_km:.4f},"
            f"0,0,0,0,0,0,{rssi},{snr}")


def synthetic_flight() -> str:
    """Pad (3 samples) → launch → apogee 500 m AGL → descent → landing."""
    lines = [HEADER]
    t = 0.0
    # Pre-launch: sitting on the pad, GPS idle.
    for _ in range(3):
        lines.append(nav_row(t, PAD_LAT, PAD_LON, PAD_ALT, rssi=-60))
        t += 2.0
    # Boost: 10 rows climbing 60 m each, drifting ~11 m north per row.
    alt = PAD_ALT
    lat = PAD_LAT
    for i in range(10):
        alt += 60.0
        lat += 0.0001
        lines.append(nav_row(t, lat, PAD_LON, alt, rssi=-60 - i * 3))
        t += 1.7
    # Descent: 8 rows dropping 60 m each.
    for i in range(8):
        alt -= 60.0
        lat += 0.0001
        lines.append(nav_row(t, lat, PAD_LON, alt, rssi=-90 - i))
        t += 1.7
    return "\n".join(lines) + "\n"


def write_log(text: str, directory: Path, name="L0000001.TXT") -> Path:
    p = directory / name
    p.write_text(text)
    return p


# ── analyze_flight.py ─────────────────────────────────────────────────

class TestHaversine(unittest.TestCase):
    def test_zero_distance(self):
        self.assertAlmostEqual(af.haversine_m(45, -75, 45, -75), 0.0)

    def test_one_degree_latitude_is_about_111km(self):
        d = af.haversine_m(45.0, -75.0, 46.0, -75.0)
        self.assertAlmostEqual(d / 1000.0, 111.195, places=1)

    def test_antipodal_symmetry(self):
        a = af.haversine_m(10, 20, 11, 21)
        b = af.haversine_m(11, 21, 10, 20)
        self.assertAlmostEqual(a, b, places=6)


class TestLoadNavRows(unittest.TestCase):
    def test_parses_nav_rows_sorted_by_time(self):
        with tempfile.TemporaryDirectory() as td:
            # One out-of-order timestamp and one junk row mixed in.
            body = "\n".join([
                HEADER,
                nav_row(4.0, PAD_LAT, PAD_LON, PAD_ALT),
                nav_row(1.0, PAD_LAT, PAD_LON, PAD_ALT),
                "2.0,STAT,L,hello",           # non-NAV row: skipped
                "3.0,NAV,L,not-a-number,x,y,z,w,v,u,1.0,0,0,0,0,0,0,-90,8",  # malformed
            ])
            rows = af.load_nav_rows(write_log(body + "\n", Path(td)))
            self.assertEqual([r["t"] for r in rows], [1.0, 4.0])
            self.assertEqual(rows[0]["rssi"], -60)

    def test_legacy_header_without_pktsrc_or_snr(self):
        legacy_header = ("Timestamp,Type,BeaconLat,BeaconLon,BeaconAlt_m,"
                         "BeaconSats,BaseLat,BaseLon,Distance_km,RSSI_dBm")
        legacy_row = "7.5,NAV,45.0,-75.0,100.0,8,45.0,-75.0,0.0,-70"
        with tempfile.TemporaryDirectory() as td:
            rows = af.load_nav_rows(write_log(legacy_header + "\n" + legacy_row + "\n", Path(td)))
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["t"], 7.5)
            self.assertEqual(rows[0]["sats"], 8)


class TestDerivedMath(unittest.TestCase):
    def test_central_diff_of_linear_ramp(self):
        rows = [{"t": float(i), "alt": 2.0 * i} for i in range(5)]
        vz = af.central_diff(rows, "alt")
        for v in vz[1:-1]:
            self.assertAlmostEqual(v, 2.0)
        self.assertEqual(vz[0], 0.0)  # padded ends

    def test_central_diff_handles_zero_dt(self):
        # Identical timestamps on both neighbours → dt=0 → derivative stays 0.
        rows = [{"t": 1.0, "alt": 0.0}, {"t": 1.0, "alt": 5.0}, {"t": 1.0, "alt": 10.0}]
        self.assertEqual(af.central_diff(rows, "alt")[1], 0.0)

    def test_find_launch_idx_threshold(self):
        rows = [{"alt": 100.0}, {"alt": 109.9}, {"alt": 111.0}]
        self.assertEqual(af.find_launch_idx(rows, 100.0), 2)
        self.assertIsNone(af.find_launch_idx([{"alt": 100.0}, {"alt": 105.0}], 100.0))

    def test_detect_gaps(self):
        rows = [{"t": 0.0}, {"t": 1.7}, {"t": 3.4}, {"t": 10.0}]
        gaps = af.detect_gaps(rows, expected_dt=1.7, tolerance=1.5)
        self.assertEqual(gaps, [(10.0, 6.6)])

    def test_cadence_stats(self):
        rows = [{"t": 0.0}, {"t": 1.0}, {"t": 3.0}]
        mean, mn, mx = af.cadence_stats(rows)
        self.assertEqual((mean, mn, mx), (1.5, 1.0, 2.0))
        self.assertEqual(af.cadence_stats([{"t": 0.0}]), (0.0, 0.0, 0.0))


class TestRangePrediction(unittest.TestCase):
    def test_fspl_extrapolation(self):
        # 24 dB of headroom at 1 km → 10^(24/20) ≈ 15.85 km usable range.
        r = af.predict_max_range_m(-100.0, 1000.0, -124.0)
        self.assertAlmostEqual(r, 1000.0 * 10 ** (24 / 20), places=3)

    def test_guards(self):
        self.assertIsNone(af.predict_max_range_m(-100.0, 0.0, -124.0))
        self.assertIsNone(af.predict_max_range_m(5.0, 1000.0, -124.0))


class TestKml(unittest.TestCase):
    def test_kml_is_well_formed_with_track_and_placemarks(self):
        with tempfile.TemporaryDirectory() as td:
            log = write_log(synthetic_flight(), Path(td))
            rows = af.load_nav_rows(log)
            launch_idx = af.find_launch_idx(rows, rows[0]["alt"])
            apogee_idx = max(range(len(rows)), key=lambda i: rows[i]["alt"])
            out = Path(td) / "flight.kml"
            af.write_kml(rows, launch_idx, apogee_idx, out,
                         base_time=af.dt.datetime(2026, 8, 1, 12, 0, 0))

            root = ET.parse(out).getroot()  # raises if malformed
            text = out.read_text()
            self.assertEqual(root.tag, "{http://www.opengis.net/kml/2.2}kml")
            self.assertIn("gx:Track", text)
            self.assertIn("Launch Pad", text)
            self.assertIn("Apogee", text)
            self.assertIn("Landing", text)
            # One <when> per sample so the GE time slider can animate.
            self.assertEqual(text.count("<when>"), len(rows))


class TestAnalyzeEndToEnd(unittest.TestCase):
    def test_synthetic_flight_summary(self):
        with tempfile.TemporaryDirectory() as td:
            log = write_log(synthetic_flight(), Path(td))
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                rc = af.analyze(str(log))
            out = buf.getvalue()
            self.assertEqual(rc, 0)
            self.assertIn("Launch detected", out)
            self.assertIn("Apogee AGL:           600 m", out)
            self.assertIn("Packets total:   21", out)

    def test_no_launch_returns_zero_with_explanation(self):
        with tempfile.TemporaryDirectory() as td:
            lines = [HEADER] + [
                nav_row(float(i), PAD_LAT, PAD_LON, PAD_ALT + (i % 2)) for i in range(6)
            ]
            log = write_log("\n".join(lines) + "\n", Path(td))
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                rc = af.analyze(str(log))
            self.assertEqual(rc, 0)
            self.assertIn("No launch detected", buf.getvalue())

    def test_too_few_rows_returns_one(self):
        with tempfile.TemporaryDirectory() as td:
            log = write_log(HEADER + "\n" + nav_row(0.0, PAD_LAT, PAD_LON, PAD_ALT) + "\n", Path(td))
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(af.analyze(str(log)), 1)


# ── calibrate_rf.py ───────────────────────────────────────────────────

class TestCalibrateRf(unittest.TestCase):
    def _row(self, dist_m, rssi, alt=100.0, base_alt=100.0):
        return {
            "BeaconLat": 45.001, "BeaconLon": -75.0, "BeaconAlt_m": alt,
            "BaseLat": 45.0, "BaseLon": -75.0, "BaseAlt_m": base_alt,
            "Distance_km": dist_m / 1000.0, "RSSI_dBm": float(rssi), "SNR_dB": 5.0,
        }

    def test_slant_range_includes_altitude_delta(self):
        r = self._row(300.0, -80, alt=500.0, base_alt=100.0)
        self.assertAlmostEqual(crf.slant_range_m(r), 500.0)  # 3-4-5 triangle

    def test_clean_drops_near_field_and_garbage(self):
        rows = [
            self._row(100.0, -80),                    # keep
            self._row(5.0, -80),                      # near-field: drop
            self._row(100.0, -200),                   # RSSI out of range: drop
            {**self._row(100.0, -80), "BeaconLat": 0.0},  # no GPS lock: drop
        ]
        with contextlib.redirect_stdout(io.StringIO()):
            kept = crf.clean(rows, min_distance_m=10.0)
        self.assertEqual(len(kept), 1)
        self.assertAlmostEqual(kept[0]["slant_m"], 100.0)

    def test_fit_recovers_free_space_exponent(self):
        # Synthetic log-distance data with n=2 exactly: RSSI = -40 - 20log10(d)
        with contextlib.redirect_stdout(io.StringIO()):
            rows = crf.clean(
                [self._row(d, -40.0 - 20.0 * math.log10(d))
                 for d in (20, 50, 100, 200, 500, 1000, 2000)],
                min_distance_m=10.0,
            )
        n, intercept, r2 = crf.fit_path_loss(rows)
        self.assertAlmostEqual(n, 2.0, places=6)
        self.assertAlmostEqual(intercept, -40.0, places=6)
        self.assertAlmostEqual(r2, 1.0, places=9)

    def test_fit_degenerate_single_distance(self):
        rows = [dict(self._row(100.0, -80), slant_m=100.0),
                dict(self._row(100.0, -81), slant_m=100.0)]
        n, intercept, r2 = crf.fit_path_loss(rows)
        self.assertTrue(math.isnan(n) and math.isnan(intercept) and math.isnan(r2))

    def test_percentile_and_floor(self):
        self.assertEqual(crf._percentile([1.0], 50), 1.0)
        self.assertEqual(crf._percentile([1.0, 3.0], 50), 2.0)
        self.assertTrue(math.isnan(crf._percentile([], 50)))
        rows = [self._row(100.0, r) for r in (-80, -85, -90, -120)]
        observed, recommended = crf.recommended_floor(rows, percentile=25.0)
        self.assertEqual(observed, -120)
        self.assertEqual(recommended, -98)  # 25th pct of [-120,-90,-85,-80] = -97.5 → floor

    def test_load_logs_skips_bad_files_and_requires_rows(self):
        with tempfile.TemporaryDirectory() as td:
            good = write_log(synthetic_flight(), Path(td), "good.TXT")
            bad = write_log("a,b,c\n1,2,3\n", Path(td), "bad.TXT")
            missing = Path(td) / "missing.TXT"
            with contextlib.redirect_stderr(io.StringIO()):
                rows = crf.load_logs([good, bad, missing])
            self.assertEqual(len(rows), 21)

            with self.assertRaises(SystemExit) as cm, \
                    contextlib.redirect_stderr(io.StringIO()):
                crf.load_logs([bad])
            self.assertEqual(cm.exception.code, 2)

    def test_main_prints_loaded_row_count(self):
        # Mirrors the Makefile smoke check, but on a synthetic log so it
        # works on a fresh clone without flight_data/.
        with tempfile.TemporaryDirectory() as td:
            log = write_log(synthetic_flight(), Path(td))
            argv = sys.argv
            sys.argv = ["calibrate_rf.py", str(log)]
            try:
                buf = io.StringIO()
                with contextlib.redirect_stdout(buf):
                    rc = crf.main()
            finally:
                sys.argv = argv
            self.assertEqual(rc, 0)
            self.assertIn("loaded 21 NAV rows", buf.getvalue())


if __name__ == "__main__":
    unittest.main()
