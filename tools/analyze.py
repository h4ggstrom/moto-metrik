#!/usr/bin/env python3
"""Analyse a moto-metrik NMEA log: lap splitting, lap times, traces.

Usage:
    python tools/analyze.py LOG.nmea                       # gate = point of max speed
    python tools/analyze.py LOG.nmea --gate 47.2950,-0.5870  # explicit start/finish line
    python tools/analyze.py LOG.nmea --no-laps             # just plot the session

Outputs go to out/<log name>/: track.png, laps.png, laps.csv, session.gpx.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

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


@dataclass
class Lap:
    number: int
    t_start: float
    t_end: float
    i_start: int  # first fix index inside the lap
    i_end: int  # last fix index inside the lap

    @property
    def time(self) -> float:
        return self.t_end - self.t_start


def find_laps(x, y, t, gate_xy, gate_width: float, min_lap_s: float) -> list[float]:
    """Return interpolated gate-crossing times (same direction as racing line)."""
    gx, gy = gate_xy
    # Racing direction at the gate = heading of the track near the closest fix.
    i0 = int(np.argmin((x - gx) ** 2 + (y - gy) ** 2))
    a, b = max(i0 - 2, 0), min(i0 + 2, len(x) - 1)
    hx, hy = x[b] - x[a], y[b] - y[a]
    n = math.hypot(hx, hy) or 1.0
    hx, hy = hx / n, hy / n
    # Gate segment is perpendicular to the heading.
    half = gate_width / 2
    q1 = (gx - hy * half, gy + hx * half)
    q2 = (gx + hy * half, gy - hx * half)

    crossings: list[float] = []
    for i in range(len(x) - 1):
        p1, p2 = (x[i], y[i]), (x[i + 1], y[i + 1])
        u = seg_intersect(p1, p2, q1, q2)
        if u is None:
            continue
        if (p2[0] - p1[0]) * hx + (p2[1] - p1[1]) * hy <= 0:
            continue  # crossing backwards
        tc = t[i] + u * (t[i + 1] - t[i])
        if crossings and tc - crossings[-1] < min_lap_s:
            continue
        crossings.append(tc)
    return crossings


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


def plot_track(path: Path, x, y, speed, laps: list[Lap], best: Lap | None, gate_xy) -> None:
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.plot(x, y, color="0.85", lw=1, zorder=1)
    sel = slice(best.i_start, best.i_end + 1) if best else slice(None)
    sc = ax.scatter(x[sel], y[sel], c=speed[sel], cmap="turbo", s=6, zorder=2)
    fig.colorbar(sc, ax=ax, shrink=0.7, label="vitesse (km/h)")
    if gate_xy is not None:
        ax.plot(*gate_xy, marker="s", color="k", ms=8, zorder=3, label="ligne départ/arrivée")
        ax.legend(loc="upper right")
    title = f"Meilleur tour #{best.number} — {fmt_lap(best.time)}" if best else "Session complète"
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


def plot_laps(path: Path, laps: list[Lap], best: Lap, x, y, t, speed) -> None:
    fig, (ax_v, ax_d) = plt.subplots(2, 1, figsize=(14, 9), sharex=True, gridspec_kw={"height_ratios": [2, 1]})
    bd, bt, _ = lap_profile(best, x, y, t, speed)
    cmap = plt.get_cmap("tab10")
    for k, lap in enumerate(laps):
        d, lt, v = lap_profile(lap, x, y, t, speed)
        is_best = lap is best
        style = dict(lw=2.2 if is_best else 1, alpha=1 if is_best else 0.7, color="k" if is_best else cmap(k % 10))
        label = f"T{lap.number} {fmt_lap(lap.time)}" + (" ★" if is_best else "")
        ax_v.plot(d, v, label=label, **style)
        if not is_best:
            # Delta vs best lap at equal distance (positive = slower than best).
            ax_d.plot(d, lt - np.interp(d, bd, bt), **style)
    ax_d.axhline(0, color="k", lw=1)
    ax_v.set_ylabel("vitesse (km/h)")
    ax_d.set_ylabel("delta vs meilleur (s)")
    ax_d.set_xlabel("distance depuis la ligne (m)")
    ax_v.legend(ncol=4, fontsize=8)
    for ax in (ax_v, ax_d):
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


# ---------------------------------------------------------------------------


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", type=Path)
    ap.add_argument("--gate", help="start/finish line centre 'LAT,LON' (default: point of max speed)")
    ap.add_argument("--gate-width", type=float, default=40.0, help="gate width in metres (default 40)")
    ap.add_argument("--min-lap", type=float, default=20.0, help="ignore crossings closer than N s (default 20)")
    ap.add_argument("--no-laps", action="store_true", help="skip lap detection")
    ap.add_argument("--out", type=Path, default=Path("out"))
    args = ap.parse_args()

    fixes = parse_nmea(args.log)
    if len(fixes) < 10:
        print(f"Only {len(fixes)} valid fixes in {args.log} - nothing to analyse.", file=sys.stderr)
        return 1

    t = np.array([f.t for f in fixes])
    lat = np.array([f.lat for f in fixes])
    lon = np.array([f.lon for f in fixes])
    speed = np.array([f.speed_kmh for f in fixes])
    proj = Projection(lat.mean(), lon.mean())
    x, y = proj(lat, lon)

    dts = np.diff(t)
    print(f"[session] {len(fixes)} fixes, {fmt_lap(t[-1] - t[0])} long, "
          f"rate ~{1 / np.median(dts):.1f} Hz, gaps >1s: {int((dts > 1).sum())}, vmax {speed.max():.1f} km/h")

    out_dir = args.out / args.log.stem
    out_dir.mkdir(parents=True, exist_ok=True)
    write_gpx(out_dir / "session.gpx", fixes)

    laps: list[Lap] = []
    gate_xy = None
    if not args.gate and speed.max() < 15:
        print("[laps] vmax < 15 km/h: static/walking session, skipping auto lap detection (use --gate)")
        args.no_laps = True
    if not args.no_laps:
        if args.gate:
            glat, glon = (float(v) for v in args.gate.split(","))
            gx, gy = proj(glat, glon)
            gate_xy = (float(gx), float(gy))
        else:
            i = int(np.argmax(speed))
            gate_xy = (float(x[i]), float(y[i]))
            print(f"[laps] auto gate at max speed point {lat[i]:.6f},{lon[i]:.6f} "
                  f"(pass --gate for the real start/finish line)")
        crossings = find_laps(x, y, t, gate_xy, args.gate_width, args.min_lap)
        for n, (a, b) in enumerate(zip(crossings, crossings[1:]), start=1):
            ia = int(np.searchsorted(t, a))
            ib = int(np.searchsorted(t, b)) - 1
            laps.append(Lap(n, a, b, ia, ib))

    best = min(laps, key=lambda l: l.time) if laps else None
    plot_track(out_dir / "track.png", x, y, speed, laps, best, gate_xy)

    if laps:
        with open(out_dir / "laps.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["lap", "time_s", "time", "vmax_kmh", "gap_to_best_s"])
            for lap in laps:
                vmax = speed[lap.i_start : lap.i_end + 1].max()
                w.writerow([lap.number, f"{lap.time:.3f}", fmt_lap(lap.time), f"{vmax:.1f}",
                            f"{lap.time - best.time:.3f}"])
                mark = "  ★" if lap is best else f"  +{lap.time - best.time:.3f}"
                print(f"  T{lap.number:<3} {fmt_lap(lap.time)}  vmax {vmax:5.1f} km/h{mark}")
        plot_laps(out_dir / "laps.png", laps, best, x, y, t, speed)
    elif not args.no_laps:
        print("[laps] no complete lap found")

    print(f"[out] {out_dir}/")
    return 0


if __name__ == "__main__":
    sys.exit(main())
