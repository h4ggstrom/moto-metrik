#!/usr/bin/env python3
"""Build a track file (tools/tracks/<name>.json) from one recorded lap.

The lap runs between the first two passes at the start/finish point.
The trace is smoothed and resampled into a closed centerline.
Sector lines sit at the given points. Corners are found from curvature
and named V1, V2, ... (rename them in the JSON afterwards).

Usage:
    python tools/make_track.py LOG.nmea [LOG2.nmea ...] --name boucle \\
        --sf 49.0595,2.2539 --sector 49.0702,2.2620 --sector 49.0724,2.2427

Several logs are merged in time order (a lap split by an SD recovery).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

from analyze import EARTH_R, TRACKS_DIR, Projection, parse_nmea, track_progress


def latlon(s: str) -> tuple[float, float]:
    a, b = s.split(",")
    return float(a), float(b)


def passes(x, y, t, px, py, radius: float, min_gap_s: float) -> list[int]:
    """Indices of the closest fix of each pass within radius of (px, py)."""
    d = np.hypot(x - px, y - py)
    out, i = [], 0
    while i < len(d):
        if d[i] < radius:
            j = i
            while j < len(d) and d[j] < radius:
                j += 1
            k = i + int(np.argmin(d[i:j]))
            if not out or t[k] - t[out[-1]] > min_gap_s:
                out.append(k)
            i = j
        else:
            i += 1
    return out


def resample_closed(x, y, step: float):
    """Closed polyline resampled every `step` metres along its length."""
    xs, ys = np.append(x, x[0]), np.append(y, y[0])
    cum = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(xs), np.diff(ys)))])
    n = max(int(round(cum[-1] / step)), 8)
    s = np.linspace(0, cum[-1], n, endpoint=False)
    return np.interp(s, cum, xs), np.interp(s, cum, ys)


def find_corners(cx, cy, step: float, max_radius: float, min_turn_deg: float, merge_m: float):
    """Corners as (start, end, dir) in metres along the centerline (0 = point 0)."""
    n = len(cx)
    hx, hy = np.roll(cx, -1) - cx, np.roll(cy, -1) - cy
    heading = np.unwrap(np.arctan2(hy, hx))
    dtheta = np.diff(np.append(heading, heading[0] + 2 * np.pi * round((heading[-1] - heading[0]) / (2 * np.pi))))
    dtheta = (dtheta + np.pi) % (2 * np.pi) - np.pi
    w = max(int(round(30 / step)), 1)  # curvature averaged over ~30 m
    kern = np.ones(w) / w
    curv = np.convolve(np.concatenate([dtheta[-w:], dtheta, dtheta[:w]]), kern, "same")[w:-w] / step
    turning = np.abs(curv) > 1 / max_radius
    if turning.all():
        return []
    # Start scanning on a straight so no corner wraps around index 0.
    start = int(np.argmin(turning))
    order = (np.arange(n) + start) % n
    raw = []  # [first, last, sign, turn] in unrolled indices
    for k, i in enumerate(order):
        if not turning[i]:
            continue
        sign = 1 if curv[i] > 0 else -1
        if raw and raw[-1][1] == k - 1 and raw[-1][2] == sign:
            raw[-1][1] = k
            raw[-1][3] += dtheta[i]
        else:
            raw.append([k, k, sign, dtheta[i]])
    # Same-direction pieces close together are one corner (e.g. double apex).
    merged = []
    for r in raw:
        if merged and merged[-1][2] == r[2] and (r[0] - merged[-1][1]) * step <= merge_m:
            merged[-1][1] = r[1]
            merged[-1][3] += r[3]
        else:
            merged.append(r)
    corners = []
    for a, b, sign, turn in merged:
        if abs(math.degrees(turn)) < min_turn_deg:
            continue
        s0 = ((a + start) % n) * step - step  # one sample of margin each side
        s1 = ((b + start) % n) * step + 2 * step
        corners.append((s0, s1, "G" if sign > 0 else "D"))  # counter-clockwise turn = left
    # Opposite-direction corners back to back (chicane, roundabout) become one.
    out = []
    for c in corners:
        if out and c[0] - out[-1][1] <= merge_m / 2 and out[-1][2][-1] != c[2]:
            out[-1] = (out[-1][0], c[1], out[-1][2] + c[2])
        else:
            out.append(c)
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", type=Path, nargs="+")
    ap.add_argument("--name", required=True, help="file name in tools/tracks/ (without .json)")
    ap.add_argument("--title", help="display name (default: --name)")
    ap.add_argument("--location", default="")
    ap.add_argument("--sf", type=latlon, required=True, help="start/finish point 'LAT,LON' on the road centre")
    ap.add_argument("--sector", type=latlon, action="append", default=[], help="sector line point, in lap order")
    ap.add_argument("--lap", type=int, default=1, help="which complete lap to use (default 1)")
    ap.add_argument("--gate-width", type=float, default=40.0)
    ap.add_argument("--step", type=float, default=10.0, help="centerline spacing in metres")
    ap.add_argument("--corner-radius", type=float, default=120.0, help="tighter than this is a corner (m)")
    ap.add_argument("--corner-turn", type=float, default=30.0, help="minimum heading change of a corner (deg)")
    args = ap.parse_args()

    fixes = sorted((f for log in args.logs for f in parse_nmea(log)), key=lambda f: f.t)
    t = np.array([f.t for f in fixes])
    keep = np.concatenate([[True], np.diff(t) > 0])
    fixes = [f for f, k in zip(fixes, keep) if k]
    t = t[keep]
    proj = Projection(*args.sf)
    x, y = proj([f.lat for f in fixes], [f.lon for f in fixes])

    sf = passes(x, y, t, 0.0, 0.0, args.gate_width / 2, 30)
    if len(sf) < args.lap + 1:
        print(f"[track] {len(sf)} passages au départ: pas de tour complet n°{args.lap}", file=sys.stderr)
        return 1
    i0, i1 = sf[args.lap - 1], sf[args.lap]
    print(f"[track] tour {args.lap}: {t[i1] - t[i0]:.1f} s, {i1 - i0} points")

    lx, ly = x[i0:i1], y[i0:i1]
    k = 3  # light smoothing (0.3 s at 10 Hz), wrapping around the closed lap
    lx = np.convolve(np.concatenate([lx[-k:], lx, lx[:k]]), np.ones(k) / k, "same")[k:-k]
    ly = np.convolve(np.concatenate([ly[-k:], ly, ly[:k]]), np.ones(k) / k, "same")[k:-k]
    cx, cy = resample_closed(lx, ly, args.step)
    step = float(np.mean(np.hypot(np.diff(np.append(cx, cx[0])), np.diff(np.append(cy, cy[0])))))
    length = step * len(cx)

    def along(lat, lon):
        px, py = proj(lat, lon)
        s, _ = track_progress(np.array([px]), np.array([py]), cx, cy, 0.0)
        return float(s[0])

    finish = along(*args.sf)
    sectors = [round((along(*p) - finish) % length, 1) for p in args.sector]
    if sectors != sorted(sectors):
        print(f"[track] attention: secteurs hors ordre {sectors}", file=sys.stderr)

    # Another stretch of road near a line would trigger it: warn.
    for name, d in [("S/F", 0.0)] + [(f"S{i + 1}|S{i + 2}", s) for i, s in enumerate(sectors)]:
        g = int(round(((finish + d) % length) / step)) % len(cx)
        dist = np.hypot(cx - cx[g], cy - cy[g])
        sep = np.minimum(np.abs(np.arange(len(cx)) - g), len(cx) - np.abs(np.arange(len(cx)) - g)) * step
        close = dist[sep > 100]
        if close.size and close.min() < args.gate_width / 2 + 10:
            print(f"[track] attention: ligne {name} à {close.min():.0f} m d'une autre portion du tracé")

    corners = []
    for n, (s0, s1, d) in enumerate(find_corners(cx, cy, step, args.corner_radius, args.corner_turn, 40), start=1):
        a, b = (s0 - finish) % length, (s1 - finish) % length
        if b < a:  # corner straddles the line: clip at it
            a, b = (a, length) if length - a > b else (0.0, b)
        corners.append({"name": f"V{n}", "dir": d, "start_m": round(a), "end_m": round(b)})
    corners.sort(key=lambda c: c["start_m"])
    for n, c in enumerate(corners, start=1):
        c["name"] = f"V{n}"

    lat = proj.lat0 + cy / (math.radians(1) * EARTH_R)
    lon = proj.lon0 + cx / proj.kx
    track = {
        "name": args.title or args.name,
        "location": args.location,
        "source": f"Centerline from recorded lap {args.lap} of {', '.join(p.name for p in args.logs)}",
        "length_m": round(length, 1),
        "gate_width_m": args.gate_width,
        "finish_m": round(finish, 1),
        "sectors_m": sectors,
        "corners": corners,
        "centerline": [[round(float(a), 7), round(float(b), 7)] for a, b in zip(lat, lon)],
    }
    out = TRACKS_DIR / f"{args.name}.json"
    out.write_text(json.dumps(track, indent=1) + "\n")
    print(f"[track] {out}: {length:.0f} m, {len(cx)} points, secteurs {sectors}, {len(corners)} virages")
    for c in corners:
        print(f"  {c['name']:4} {c['dir']:4} {c['start_m']:5}-{c['end_m']:5} m")
    return 0


if __name__ == "__main__":
    sys.exit(main())
