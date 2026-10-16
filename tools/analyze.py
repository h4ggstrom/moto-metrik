#!/usr/bin/env python3
"""Analyse a moto-metrik NMEA log: lap splitting, lap times, traces.

Usage:
    python tools/analyze.py LOG.nmea                       # known track auto-detected, else gate = max speed
    python tools/analyze.py LOG.nmea --gate 47.2950,-0.5870  # explicit start/finish line
    python tools/analyze.py LOG.nmea --no-laps             # just plot the session

Outputs go to out/<log name>/: track.png, laps.png, laps.csv, session.gpx.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from imu import imu_dynamics, load_imu  # noqa: E402

EARTH_R = 6371000.0
KNOT_KMH = 1.852


# ---------------------------------------------------------------------------
# NMEA parsing
# ---------------------------------------------------------------------------


def nmea_checksum_ok(line: str) -> bool:
    if not line.startswith("$") or "*" not in line:
        return False
    body, _, ck = line[1:].partition("*")
    calc = 0
    for ch in body:
        calc ^= ord(ch)
    try:
        return calc == int(ck[:2], 16)
    except ValueError:
        return False


def nmea_coord(value: str, hemi: str) -> float | None:
    if not value:
        return None
    dot = value.index(".")
    deg = float(value[: dot - 2])
    minutes = float(value[dot - 2 :])
    v = deg + minutes / 60.0
    return -v if hemi in ("S", "W") else v


@dataclass
class Fix:
    t: float  # seconds since epoch (UTC)
    lat: float
    lon: float
    speed_kmh: float
    course: float
    sats: int = 0
    hdop: float = float("nan")
    alt: float = float("nan")


def parse_nmea(path: Path) -> list[Fix]:
    fixes: list[Fix] = []
    gga: dict[str, tuple[int, float, float]] = {}  # hhmmss.ss -> (sats, hdop, alt)
    rmc_keys: list[str] = []
    bad = 0
    with open(path, "r", encoding="ascii", errors="replace") as f:
        for raw in f:
            # A power cut can splice two sentences together; keep the last '$'.
            line = raw[raw.rfind("$") :].strip() if "$" in raw else ""
            if not line:
                continue
            if not nmea_checksum_ok(line):
                bad += 1
                continue
            fields = line[1:].split("*")[0].split(",")
            kind = fields[0][2:]
            try:
                if kind == "GGA" and fields[6] not in ("", "0"):
                    gga[fields[1]] = (
                        int(fields[7] or 0),
                        float(fields[8] or "nan"),
                        float(fields[9] or "nan"),
                    )
                elif kind == "RMC" and fields[2] == "A":
                    hhmmss, ddmmyy = fields[1], fields[9]
                    t = dt.datetime.strptime(ddmmyy + hhmmss.split(".")[0], "%d%m%y%H%M%S")
                    frac = float("0." + hhmmss.split(".")[1]) if "." in hhmmss else 0.0
                    ts = t.replace(tzinfo=dt.timezone.utc).timestamp() + frac
                    fix = Fix(
                        t=ts,
                        lat=nmea_coord(fields[3], fields[4]),
                        lon=nmea_coord(fields[5], fields[6]),
                        speed_kmh=float(fields[7] or 0) * KNOT_KMH,
                        course=float(fields[8] or "nan"),
                    )
                    fixes.append(fix)
                    rmc_keys.append(hhmmss)
            except (ValueError, IndexError):
                bad += 1
    # GGA may come before or after its RMC within an epoch: merge once all is read.
    for fix, key in zip(fixes, rmc_keys):
        if key in gga:
            fix.sats, fix.hdop, fix.alt = gga[key]
    if bad:
        print(f"[parse] {bad} corrupt sentences skipped", file=sys.stderr)
    # Drop duplicate / non-monotonic timestamps.
    out: list[Fix] = []
    for fx in fixes:
        if not out or fx.t > out[-1].t:
            out.append(fx)
    return out


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------


class Projection:
    """Local equirectangular projection in metres; plenty accurate at track scale."""

    def __init__(self, lat0: float, lon0: float):
        self.lat0, self.lon0 = lat0, lon0
        self.kx = math.radians(1) * EARTH_R * math.cos(math.radians(lat0))
        self.ky = math.radians(1) * EARTH_R

    def __call__(self, lat, lon):
        return (np.asarray(lon) - self.lon0) * self.kx, (np.asarray(lat) - self.lat0) * self.ky


def seg_intersect(p1, p2, q1, q2):
    """Return u in [0,1] along p1->p2 where it crosses segment q1-q2, else None."""
    d = (p2[0] - p1[0]) * (q2[1] - q1[1]) - (p2[1] - p1[1]) * (q2[0] - q1[0])
    if abs(d) < 1e-12:
        return None
    u = ((q1[0] - p1[0]) * (q2[1] - q1[1]) - (q1[1] - p1[1]) * (q2[0] - q1[0])) / d
    v = ((q1[0] - p1[0]) * (p2[1] - p1[1]) - (q1[1] - p1[1]) * (p2[0] - p1[0])) / d
    return u if 0 <= u <= 1 and 0 <= v <= 1 else None


G = 9.81


def smooth(a, n: int):
    """Centered moving average over n samples (odd), edges padded."""
    if n <= 1:
        return a
    k = n // 2
    return np.convolve(np.pad(a, k, mode="edge"), np.ones(n) / n, mode="valid")


def median(a, n: int):
    """Centered running median over n samples (odd), edges padded."""
    if n <= 1:
        return a
    k = n // 2
    return np.median(np.lib.stride_tricks.sliding_window_view(np.pad(a, k, mode="edge"), n), axis=1)


def dynamics(t, speed_kmh, course_deg, min_speed_kmh: float = 15.0, window: int = 5):
    """Lean angle (deg, + = right), lateral and longitudinal acceleration (g).

    Steady-turn model: a_lat = v * yaw_rate, tan(lean) = a_lat / g. Yaw rate
    comes from the GPS course over ground (Doppler-based, far less noisy than
    differentiating positions). This is the lean of an ideal thin-tyre bike
    with the rider in line: real bike lean is a few degrees more (tyre width),
    less if the rider hangs off.
    """
    v = speed_kmh / 3.6
    course = np.asarray(course_deg, dtype=float)
    ok = np.isfinite(course)
    if ok.sum() < 2:
        nan = np.full_like(v, np.nan)
        return nan, nan, nan
    course = np.interp(t, t[ok], course[ok])
    heading = smooth(np.unwrap(np.radians(course)), window)
    yaw_rate = median(np.gradient(heading, t), window)  # kills single-epoch course spikes
    a_lat = smooth(v * yaw_rate, window)
    a_lon = smooth(np.gradient(smooth(v, window), t), window)
    lean = np.degrees(np.arctan2(a_lat, G))
    slow = speed_kmh < min_speed_kmh  # course is meaningless when (nearly) stopped
    lean[slow] = np.nan
    a_lat[slow] = np.nan
    return lean, a_lat / G, a_lon / G


@dataclass
class Lap:
    number: int
    t_start: float
    t_end: float
    i_start: int  # first fix index inside the lap
    i_end: int  # last fix index inside the lap
    sectors: list[float | None] = field(default_factory=list)  # sector times, None if a gate was missed

    @property
    def time(self) -> float:
        return self.t_end - self.t_start


@dataclass
class Gate:
    """A virtual line across the track, crossed in direction (hx, hy)."""

    name: str
    x: float
    y: float
    hx: float
    hy: float
    width: float

    @property
    def ends(self):
        half = self.width / 2
        return ((self.x - self.hy * half, self.y + self.hx * half),
                (self.x + self.hy * half, self.y - self.hx * half))


def gate_from_trace(name: str, x, y, gx: float, gy: float, width: float) -> Gate:
    """Gate at (gx, gy), perpendicular to the recorded trace near that point."""
    i0 = int(np.argmin((x - gx) ** 2 + (y - gy) ** 2))
    a, b = max(i0 - 2, 0), min(i0 + 2, len(x) - 1)
    hx, hy = x[b] - x[a], y[b] - y[a]
    n = math.hypot(hx, hy) or 1.0
    return Gate(name, gx, gy, hx / n, hy / n, width)


def gate_on_centerline(name: str, cx, cy, dist_m: float, width: float) -> Gate:
    """Gate at dist_m along a closed centerline, perpendicular to it."""
    xs, ys = np.append(cx, cx[0]), np.append(cy, cy[0])
    seg = np.hypot(np.diff(xs), np.diff(ys))
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    d = dist_m % cum[-1]
    i = min(int(np.searchsorted(cum, d, side="right")) - 1, len(seg) - 1)
    u = (d - cum[i]) / seg[i]
    return Gate(name, xs[i] + u * (xs[i + 1] - xs[i]), ys[i] + u * (ys[i + 1] - ys[i]),
                (xs[i + 1] - xs[i]) / seg[i], (ys[i + 1] - ys[i]) / seg[i], width)


def gate_crossings(x, y, t, gate: Gate, min_interval_s: float = 0.0) -> list[float]:
    """Interpolated times at which the trace crosses the gate in its direction."""
    q1, q2 = gate.ends
    crossings: list[float] = []
    for i in range(len(x) - 1):
        p1, p2 = (x[i], y[i]), (x[i + 1], y[i + 1])
        u = seg_intersect(p1, p2, q1, q2)
        if u is None:
            continue
        if (p2[0] - p1[0]) * gate.hx + (p2[1] - p1[1]) * gate.hy <= 0:
            continue  # crossing backwards
        tc = t[i] + u * (t[i + 1] - t[i])
        if crossings and tc - crossings[-1] < min_interval_s:
            continue
        crossings.append(tc)
    return crossings


# ---------------------------------------------------------------------------
# Track library
# ---------------------------------------------------------------------------

TRACKS_DIR = Path(__file__).parent / "tracks"


def load_track(name: str) -> dict:
    return json.loads((TRACKS_DIR / f"{name}.json").read_text())


def detect_track(lat, lon, max_dist_m: float = 1000.0) -> str | None:
    """Name of the library track the session's median position is closest to."""
    mlat, mlon = float(np.median(lat)), float(np.median(lon))
    best, best_d = None, max_dist_m
    for path in sorted(TRACKS_DIR.glob("*.json")):
        cl = np.array(json.loads(path.read_text())["centerline"])
        proj = Projection(mlat, mlon)
        cx, cy = proj(cl[:, 0], cl[:, 1])
        d = float(np.min(np.hypot(cx, cy)))
        if d < best_d:
            best, best_d = path.stem, d
    return best



# ---------------------------------------------------------------------------
# Corners
# ---------------------------------------------------------------------------


def track_progress(x, y, cx, cy, offset_m: float):
    """Distance along the closed centerline (0 = start/finish) of each fix."""
    xs, ys = np.append(cx, cx[0]), np.append(cy, cy[0])
    ax, ay = xs[:-1], ys[:-1]
    dx, dy = np.diff(xs), np.diff(ys)
    seg = np.hypot(dx, dy)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    length = cum[-1]
    px, py = np.asarray(x)[:, None], np.asarray(y)[:, None]
    u = np.clip(((px - ax) * dx + (py - ay) * dy) / seg**2, 0, 1)
    dist2 = (ax + u * dx - px) ** 2 + (ay + u * dy - py) ** 2
    k = np.argmin(dist2, axis=1)
    rows = np.arange(len(k))
    s = cum[k] + u[rows, k] * seg[k]
    return (s - offset_m) % length, length


@dataclass
class CornerPass:
    corner: str
    lap: int
    t_in: float
    time: float
    v_in: float
    v_min: float
    v_min_at: float  # metres after the corner start
    v_out: float
    lean_max: float
    brake_at: float | None  # metres before the corner start where braking began
    brake_g: float


def corner_passes(t, s, speed, lean, a_lon, corners, sf_times, track_length: float) -> list[CornerPass]:
    out = []
    ds = np.diff(s)
    fwd = (ds > 0) & (ds < 60)  # moving forward, not wrapping at the line
    for ci, c in enumerate(corners):
        a, b = c["start_m"], c["end_m"]
        # Braking can only start after the previous corner (straight before this one).
        prev_end = corners[ci - 1]["end_m"]
        window = min(250.0, (a - prev_end) % track_length)
        starts = np.where(fwd & (s[:-1] < a) & (s[1:] >= a))[0]
        for i in starts:
            ends = np.where(fwd[i:] & (s[i:-1] < b) & (s[i + 1:] >= b))[0]
            if not len(ends):
                continue
            j = i + ends[0]
            if t[j] - t[i] > 60:
                continue
            t_in = t[i] + (a - s[i]) / (s[i + 1] - s[i]) * (t[i + 1] - t[i])
            t_out = t[j] + (b - s[j]) / (s[j + 1] - s[j]) * (t[j + 1] - t[j])
            sl = slice(i + 1, j + 1)
            k = i + 1 + int(np.argmin(speed[sl]))
            # Braking point: the speed peak before the slowest point, searched
            # back to 250 m before the corner start.
            brake_at, brake_g = None, 0.0
            m0 = i
            while m0 > 0 and 0 <= a - s[m0 - 1] < window and t[i] - t[m0 - 1] < 20:
                m0 -= 1
            m = m0 + int(np.argmax(speed[m0 : k + 1]))
            if speed[m] - speed[k] > 10:  # a real slow-down, not a lift
                brake_at = a - s[m]
                brake_g = float(-np.nanmin(a_lon[m : k + 1])) if np.isfinite(a_lon[m : k + 1]).any() else 0.0
            lap = int(np.searchsorted(sf_times, t_in))
            seg_lean = lean[sl]
            out.append(CornerPass(
                c["name"], lap, t_in, t_out - t_in, float(np.interp(t_in, t, speed)), float(speed[k]),
                float(s[k] - a), float(np.interp(t_out, t, speed)),
                float(np.nanmax(np.abs(seg_lean))) if np.isfinite(seg_lean).any() else float("nan"),
                brake_at, brake_g))
    return sorted(out, key=lambda p: p.t_in)

# ---------------------------------------------------------------------------
# Output
# ---------------------------------------------------------------------------


def fmt_lap(s: float) -> str:
    m, s = divmod(s, 60)
    return f"{int(m)}:{s:06.3f}"


def write_gpx(path: Path, fixes: list[Fix]) -> None:
    with open(path, "w") as f:
        f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        f.write('<gpx version="1.1" creator="moto-metrik" xmlns="http://www.topografix.com/GPX/1/1">\n')
        f.write("<trk><name>moto-metrik</name><trkseg>\n")
        for fx in fixes:
            ts = dt.datetime.fromtimestamp(fx.t, dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%fZ")
            ele = "" if math.isnan(fx.alt) else f"<ele>{fx.alt:.1f}</ele>"
            f.write(f'<trkpt lat="{fx.lat:.7f}" lon="{fx.lon:.7f}">{ele}<time>{ts}</time></trkpt>\n')
        f.write("</trkseg></trk></gpx>\n")


def plot_track(path: Path, x, y, speed, best: Lap | None, gates: list[Gate], outline, title: str,
               corner_labels=()) -> None:
    fig, ax = plt.subplots(figsize=(10, 10))
    if outline is not None:
        ox, oy = outline
        ax.plot(np.append(ox, ox[0]), np.append(oy, oy[0]), color="0.6", lw=10, alpha=0.25, zorder=0,
                solid_capstyle="round")
    ax.plot(x, y, color="0.85", lw=1, zorder=1)
    sel = slice(best.i_start, best.i_end + 1) if best else slice(None)
    sc = ax.scatter(x[sel], y[sel], c=speed[sel], cmap="turbo", s=6, zorder=2)
    fig.colorbar(sc, ax=ax, shrink=0.7, label="vitesse (km/h)")
    for g in gates:
        (x1, y1), (x2, y2) = g.ends
        ax.plot([x1, x2], [y1, y2], color="k", lw=2, zorder=3)
        ax.annotate(g.name, (x2, y2), fontsize=9, fontweight="bold", zorder=3)
    for name, (lx, ly) in corner_labels:
        ax.annotate(name, (lx, ly), fontsize=10, color="0.25", ha="center", va="center", zorder=4,
                    bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="0.6", alpha=0.85))
    if best:
        title += f" — meilleur tour T{best.number} {fmt_lap(best.time)}"
    ax.set_title(title)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def lap_profile(lap: Lap, x, y, t, speed):
    """Distance from lap start, time from lap start, speed - all per fix inside the lap."""
    sl = slice(lap.i_start, lap.i_end + 1)
    lx, ly, lt, lv = x[sl], y[sl], t[sl], speed[sl]
    d = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(lx), np.diff(ly)))])
    return d, lt - lap.t_start, lv


def plot_laps(path: Path, laps: list[Lap], best: Lap, x, y, t, speed, lean) -> None:
    fig, (ax_v, ax_l, ax_d) = plt.subplots(3, 1, figsize=(14, 12), sharex=True,
                                           gridspec_kw={"height_ratios": [2, 1.3, 1]})
    bd, bt, _ = lap_profile(best, x, y, t, speed)
    cmap = plt.get_cmap("tab10")
    for k, lap in enumerate(laps):
        d, lt, v = lap_profile(lap, x, y, t, speed)
        is_best = lap is best
        style = dict(lw=2.2 if is_best else 1, alpha=1 if is_best else 0.7, color="k" if is_best else cmap(k % 10))
        label = f"T{lap.number} {fmt_lap(lap.time)}" + (" ★" if is_best else "")
        ax_v.plot(d, v, label=label, **style)
        ax_l.plot(d, lean[lap.i_start : lap.i_end + 1], **style)
        if not is_best:
            # Delta vs best lap at equal distance (positive = slower than best).
            ax_d.plot(d, lt - np.interp(d, bd, bt), **style)
    ax_d.axhline(0, color="k", lw=1)
    ax_l.axhline(0, color="k", lw=1)
    ax_l.set_ylabel("angle estimé (°)\n← gauche | droite →")
    ax_v.set_ylabel("vitesse (km/h)")
    ax_d.set_ylabel("delta vs meilleur (s)")
    ax_d.set_xlabel("distance depuis la ligne (m)")
    ax_v.legend(ncol=4, fontsize=8)
    for ax in (ax_v, ax_l, ax_d):
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def plot_lean(path: Path, x, y, lean, best: Lap | None, outline, title: str) -> None:
    fig, ax = plt.subplots(figsize=(10, 10))
    if outline is not None:
        ox, oy = outline
        ax.plot(np.append(ox, ox[0]), np.append(oy, oy[0]), color="0.6", lw=10, alpha=0.25, zorder=0)
    sel = slice(best.i_start, best.i_end + 1) if best else slice(None)
    finite = np.isfinite(lean[sel])
    lim = max(10.0, float(np.percentile(np.abs(lean[sel][finite]), 99))) if finite.any() else 10.0
    sc = ax.scatter(x[sel], y[sel], c=lean[sel], cmap="RdBu_r", vmin=-lim, vmax=lim, s=6, zorder=2)
    fig.colorbar(sc, ax=ax, shrink=0.7, label="angle estimé (°)  ← gauche | droite →")
    ax.set_title(title + (f" — angle, tour T{best.number}" if best else " — angle"))
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def write_session_csv(path: Path, fixes: list[Fix], laps: list[Lap], lean, a_lat, a_lon) -> None:
    lap_of = np.zeros(len(fixes), dtype=int)
    for lap in laps:
        lap_of[lap.i_start : lap.i_end + 1] = lap.number
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["utc", "lap", "lat", "lon", "speed_kmh", "course_deg", "lean_deg", "a_lat_g", "a_lon_g",
                    "sats", "hdop"])
        for i, fx in enumerate(fixes):
            ts = dt.datetime.fromtimestamp(fx.t, dt.timezone.utc).strftime("%H:%M:%S.%f")[:-4]
            w.writerow([ts, lap_of[i], f"{fx.lat:.7f}", f"{fx.lon:.7f}", f"{fx.speed_kmh:.2f}",
                        f"{fx.course:.2f}", f"{lean[i]:.1f}", f"{a_lat[i]:.3f}", f"{a_lon[i]:.3f}",
                        fx.sats, fx.hdop])


# ---------------------------------------------------------------------------


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", type=Path)
    ap.add_argument("--track", help="track from tools/tracks/ (default: auto-detect, 'none' to disable)")
    ap.add_argument("--gate", help="start/finish line centre 'LAT,LON' (overrides the track; default: point of max speed)")
    ap.add_argument("--gate-width", type=float, default=40.0, help="gate width in metres (default 40)")
    ap.add_argument("--min-lap", type=float, default=20.0, help="ignore crossings closer than N s (default 20)")
    ap.add_argument("--no-laps", action="store_true", help="skip lap detection")
    ap.add_argument("--out", type=Path, default=Path("out"))
    args = ap.parse_args()

    fixes = parse_nmea(args.log)
    # Receiver config written by the firmware at boot ($PMMK sentences).
    with open(args.log, "r", encoding="ascii", errors="replace") as f:
        for line in f:
            if line.startswith("$PMMK,GNSS,") and nmea_checksum_ok(line.strip()):
                print(f"[gps] {line.strip()[11:].split('*')[0]}")
                break
    if len(fixes) < 10:
        print(f"Only {len(fixes)} valid fixes in {args.log} - nothing to analyse.", file=sys.stderr)
        return 1

    t = np.array([f.t for f in fixes])
    lat = np.array([f.lat for f in fixes])
    lon = np.array([f.lon for f in fixes])
    speed = np.array([f.speed_kmh for f in fixes])
    proj = Projection(lat.mean(), lon.mean())
    x, y = proj(lat, lon)
    lean, a_lat, a_lon = dynamics(t, speed, np.array([f.course for f in fixes]))
    # Poor signal (few satellites, high HDOP, data gap) gives position/course
    # jumps and absurd lean/g values: blank the dynamics around those fixes.
    sats = np.array([f.sats for f in fixes])
    hdop = np.array([f.hdop for f in fixes])
    bad = (sats < 5) | (hdop > 2.0)
    bad[1:] |= np.diff(t) > 0.3
    bad = np.convolve(bad, np.ones(11), mode="same") > 0  # +-0.5 s
    lean[bad] = np.nan
    a_lat[bad] = np.nan
    a_lon[bad] = np.nan
    if bad.any():
        print(f"[dyn] {bad.sum() / 10:.1f} s of poor signal excluded from lean/g")
    # IMU, when logged and rigidly mounted: replaces the GPS-derived dynamics.
    lean_gps = lean.copy()
    imu = load_imu(args.log)
    if imu is not None:
        course = np.array([f.course for f in fixes], dtype=float)
        ok_c = np.isfinite(course)
        yaw_gps = (np.degrees(np.gradient(np.unwrap(np.radians(np.interp(t, t[ok_c], course[ok_c]))), t))
                   if ok_c.sum() > 2 else np.zeros_like(t))
        dyn = imu_dynamics(imu, t, speed, a_lon, yaw_gps)
        if dyn is None:
            print("[imu] not enough stops/straights/braking to calibrate the mounting: GPS dynamics kept")
        else:
            print(f"[imu] {dyn.info}")
            both = np.isfinite(lean_gps) & (speed > 30)
            if both.any():
                print(f"[imu] lean IMU vs GPS: median |diff| {np.median(np.abs(dyn.lean[both] - lean_gps[both])):.1f} deg")
            lean, a_lat, a_lon = dyn.lean, dyn.a_lat, dyn.a_lon
            lean[speed < 15] = np.nan
    # Beyond what track-day tyres allow: GPS course lag/catch-up in tight hairpins.
    implausible = np.abs(lean) > 62
    if implausible.any():
        print(f"[dyn] {implausible.sum() / 10:.1f} s of implausible lean (>62 deg) excluded")
        lean[implausible] = np.nan
        a_lat[implausible] = np.nan

    dts = np.diff(t)
    period = np.median(dts)
    skipped = int(np.round(dts[dts > 1.5 * period] / period).sum() - (dts > 1.5 * period).sum())
    expected = len(fixes) + skipped
    print(f"[session] {len(fixes)} fixes, {fmt_lap(t[-1] - t[0])} long, "
          f"rate ~{1 / period:.1f} Hz, skipped epochs: {skipped} ({100 * skipped / expected:.1f}%), "
          f"gaps >1s: {int((dts > 1).sum())}, vmax {speed.max():.1f} km/h")

    out_dir = args.out / args.log.stem
    out_dir.mkdir(parents=True, exist_ok=True)
    write_gpx(out_dir / "session.gpx", fixes)

    laps: list[Lap] = []
    gates: list[Gate] = []  # gates[0] = start/finish, then sector gates
    outline = None
    title = "Session complète"
    track_name = None if (args.gate or args.no_laps) else (args.track or detect_track(lat, lon))
    if track_name == "none":
        track_name = None
    if track_name:
        track = load_track(track_name)
        cl = np.array(track["centerline"])
        outline = proj(cl[:, 0], cl[:, 1])
        width = track.get("gate_width_m", args.gate_width)
        gates.append(gate_on_centerline("S/F", *outline, track["finish_m"], width))
        for k, d in enumerate(track.get("sectors_m", []), start=1):
            gates.append(gate_on_centerline(f"S{k}|S{k + 1}", *outline, track["finish_m"] + d, width))
        title = track["name"]
        print(f"[track] {track['name']} ({track['length_m']:.0f} m, {len(gates)} secteurs)")
    elif args.gate:
        glat, glon = (float(v) for v in args.gate.split(","))
        gx, gy = proj(glat, glon)
        gates.append(gate_from_trace("S/F", x, y, float(gx), float(gy), args.gate_width))
    elif args.no_laps:
        pass
    elif speed.max() < 15:
        print("[laps] vmax < 15 km/h: static/walking session, skipping auto lap detection (use --gate)")
    else:
        i = int(np.argmax(speed))
        gates.append(gate_from_trace("S/F", x, y, float(x[i]), float(y[i]), args.gate_width))
        print(f"[laps] auto gate at max speed point {lat[i]:.6f},{lon[i]:.6f} "
              f"(pass --gate for the real start/finish line)")
    if args.no_laps:
        gates = []

    if gates:
        crossings = gate_crossings(x, y, t, gates[0], args.min_lap)
        sector_cross = [gate_crossings(x, y, t, g) for g in gates[1:]]
        for n, (a, b) in enumerate(zip(crossings, crossings[1:]), start=1):
            ia = int(np.searchsorted(t, a))
            ib = int(np.searchsorted(t, b)) - 1
            lap = Lap(n, a, b, ia, ib)
            if sector_cross:
                # One crossing of each sector gate, in order, inside the lap.
                marks = [a]
                for sc in sector_cross:
                    inside = [c for c in sc if marks[-1] < c < b]
                    marks.append(inside[0] if inside else None)
                marks.append(b)
                if None in marks:
                    lap.sectors = [None] * (len(marks) - 1)
                else:
                    lap.sectors = list(np.diff(marks))
            laps.append(lap)

    if gates and len(gates) > 1:
        # Every sector timed between two consecutive gates, including those of
        # laps that are not complete (out-lap, session cut short).
        events = sorted([(c, 0) for c in crossings] +
                        [(c, k + 1) for k, sc in enumerate(sector_cross) for c in sc])
        n_g = len(gates)
        partial: dict[int, list] = {}
        lap_no = 0
        for (ta, ga), (tb, gb) in zip(events, events[1:]):
            if ga == 0:
                lap_no += 1
            if gb == (ga + 1) % n_g and tb - ta < 300:
                partial.setdefault(lap_no, [None] * n_g)[ga] = tb - ta
        complete = {l.number for l in laps}
        rows = {k: v for k, v in partial.items() if k not in complete}
        if rows:
            print("  tours incomplets (temps par secteur) :")
            for k, secs in sorted(rows.items()):
                name = "sortie" if k == 0 else f"T{k}"
                txt = "  ".join("   --   " if v is None else f"{v:7.3f} " for v in secs)
                print(f"  {name:<6}            {txt}")

    best = min(laps, key=lambda l: l.time) if laps else None
    corner_labels, passes = [], []
    if track_name and track.get("corners"):
        ox, oy = outline
        for c in track["corners"]:
            g = gate_on_centerline(c["name"], ox, oy, track["finish_m"] + (c["start_m"] + c["end_m"]) / 2, 0)
            # label pushed 35 m to the outside of the turn
            side = -1 if c["dir"].startswith("G") else 1
            corner_labels.append((c["name"], (g.x + side * g.hy * 35, g.y - side * g.hx * 35)))
        prog, track_len = track_progress(x, y, ox, oy, track["finish_m"])
        sf_times = np.array(crossings) if gates else np.array([])
        passes = corner_passes(t, prog, speed, lean, a_lon, track["corners"], sf_times, track_len)
        if passes:
            print("  virages :  tour  freine à   v entrée   v mini (à)    v sortie  angle   temps")
            for p_ in passes:
                lap_txt = "sortie" if p_.lap == 0 else f"T{p_.lap}"
                brake = f"{p_.brake_at:5.0f} m" if p_.brake_at is not None else "   --  "
                print(f"  {p_.corner:<8} {lap_txt:>6}  {brake}  {p_.v_in:6.1f}    {p_.v_min:6.1f} ({p_.v_min_at:3.0f} m)"
                      f"  {p_.v_out:6.1f}   {p_.lean_max:4.1f}°  {p_.time:6.2f}s")
            with open(out_dir / "corners.csv", "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["corner", "lap", "brake_before_m", "brake_g", "v_in_kmh", "v_min_kmh", "v_min_after_m",
                            "v_out_kmh", "lean_max_deg", "time_s"])
                for p_ in passes:
                    w.writerow([p_.corner, p_.lap, "" if p_.brake_at is None else f"{p_.brake_at:.0f}",
                                f"{p_.brake_g:.2f}", f"{p_.v_in:.1f}", f"{p_.v_min:.1f}", f"{p_.v_min_at:.0f}",
                                f"{p_.v_out:.1f}", f"{p_.lean_max:.1f}", f"{p_.time:.3f}"])
    plot_track(out_dir / "track.png", x, y, speed, best, gates, outline, title, corner_labels)
    plot_lean(out_dir / "lean.png", x, y, lean, best, outline, title)
    write_session_csv(out_dir / "session.csv", fixes, laps, lean, a_lat, a_lon)

    if laps:
        n_sec = len(laps[0].sectors)
        best_sec = [min((l.sectors[k] for l in laps if l.sectors[k] is not None), default=None)
                    for k in range(n_sec)]
        with open(out_dir / "laps.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["lap", "time_s", "time", *[f"S{k + 1}_s" for k in range(n_sec)], "vmax_kmh",
                        "lean_left_deg", "lean_right_deg", "brake_max_g", "gap_to_best_s"])
            for lap in laps:
                sl = slice(lap.i_start, lap.i_end + 1)
                vmax = speed[sl].max()
                lean_l, lean_r = -np.nanmin(lean[sl]), np.nanmax(lean[sl])
                brake = -np.nanmin(a_lon[sl])
                secs = ["" if s_ is None else f"{s_:.3f}" for s_ in lap.sectors]
                w.writerow([lap.number, f"{lap.time:.3f}", fmt_lap(lap.time), *secs, f"{vmax:.1f}",
                            f"{lean_l:.1f}", f"{lean_r:.1f}", f"{brake:.2f}", f"{lap.time - best.time:.3f}"])
                sec_txt = "  ".join(
                    "   --   " if s_ is None else f"{s_:7.3f}{'*' if s_ == best_sec[k] else ' '}"
                    for k, s_ in enumerate(lap.sectors))
                mark = "  ★" if lap is best else f"  +{lap.time - best.time:.3f}"
                print(f"  T{lap.number:<3} {fmt_lap(lap.time)}  {sec_txt}  vmax {vmax:5.1f} km/h  "
                      f"angle G/D {lean_l:4.1f}°/{lean_r:4.1f}°  frein {brake:.2f}g{mark}")
        if n_sec and None not in best_sec:
            print(f"  tour idéal (meilleurs secteurs) : {fmt_lap(sum(best_sec))}  "
                  f"(-{best.time - sum(best_sec):.3f} vs meilleur tour)")
        plot_laps(out_dir / "laps.png", laps, best, x, y, t, speed, lean)
    elif gates:
        print("[laps] no complete lap found")

    if np.isfinite(lean).any():
        print(f"[dyn] angle max G/D {-np.nanmin(lean):.1f}°/{np.nanmax(lean):.1f}°, "
              f"a_lat max {np.nanmax(np.abs(a_lat)):.2f} g, freinage max {-np.nanmin(a_lon):.2f} g, "
              f"accel max {np.nanmax(a_lon):.2f} g")
    print(f"[out] {out_dir}/")
    return 0


if __name__ == "__main__":
    sys.exit(main())
