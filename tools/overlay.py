#!/usr/bin/env python3
"""Burn moto-metrik data into GoPro onboard footage.

The video is aligned on the log by its own GPS: GoPro GPSU time gives a first
guess, the GoPro and logger speed traces are cross-correlated, then the GoPro
and logger gyroscopes refine it (GPS speeds carry their own delays). The overlay shows, live:
  - lap timer: lap number, running time, live delta to the best lap so far,
    sector times, last and best lap;
  - track map with the bike's position;
  - lean angle gauge (with the lap's max left/right), speed, longitudinal g.

Usage:
    python tools/overlay.py LOG.nmea GX010325.MP4 GX020325.MP4 [--start 300 --duration 90]

Chapters of one GoPro recording (GX01xxxx, GX02xxxx...) are given in order and
joined. Output: out/<log name>/overlay.mp4 (or --out).
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
import time
from pathlib import Path

import imageio_ffmpeg
import numpy as np
from PIL import Image, ImageDraw, ImageFont

import gopro
from imu import load_imu
from analyze import (Projection, detect_track, fmt_lap, gate_crossings, gate_on_centerline, load_track, parse_nmea,
                     session_dynamics, track_progress)

FPS = 30000 / 1001  # overlay refresh; the video keeps its own frame rate
MARGIN = 40

FONT_DIR = Path("/usr/share/fonts/truetype")
BOLD = FONT_DIR / "liberation/LiberationSansNarrow-Bold.ttf"
REG = FONT_DIR / "liberation/LiberationSansNarrow-Regular.ttf"
MONO = FONT_DIR / "dejavu/DejaVuSansMono-Bold.ttf"

PANEL = (12, 16, 22, 165)
WHITE = (245, 247, 250, 255)
MUTED = (170, 180, 192, 255)
GREEN = (60, 220, 110, 255)
RED = (255, 75, 60, 255)
PURPLE = (190, 110, 255, 255)
YELLOW = (255, 210, 40, 255)
TRACK = (235, 238, 242, 255)


def font(path: Path, size: int):
    try:
        return ImageFont.truetype(str(path), size)
    except OSError:
        return ImageFont.load_default(size)


def fmt_delta(d: float) -> str:
    return f"{'+' if d >= 0 else '−'}{abs(d):.2f}"


# ---------------------------------------------------------------------------
# Video <-> log time


def display_size(video: Path) -> tuple[int, int]:
    """Width, height as played: GoPro mounted sideways stores a rotation."""
    ff = imageio_ffmpeg.get_ffmpeg_exe()
    info = subprocess.run([ff, "-hide_banner", "-i", str(video)], capture_output=True, text=True).stderr
    m = re.search(r"Video:.*?(\d{3,5})x(\d{3,5})", info)
    w, h = int(m.group(1)), int(m.group(2))
    rot = re.search(r"rotation of (-?[\d.]+)", info)
    if rot and abs(round(float(rot.group(1)))) % 180 == 90:
        w, h = h, w
    return w, h


def chapter_clock(tel: gopro.Telemetry):
    """Robust linear fit video time -> GoPro UTC (GPSU) for one chapter."""
    u = np.array(tel.gps_utc)
    keep = np.ones(len(u), bool)
    for _ in range(3):
        a, b = np.polyfit(u[keep, 0], u[keep, 1], 1)
        keep = np.abs(u[:, 1] - (a * u[:, 0] + b)) < 0.2
    return a, b


def find_lag(t_log, v_log, utc_gp, v_gp) -> float:
    """Seconds to add to GoPro UTC to get logger UTC (speed cross-correlation)."""
    grid = np.arange(max(t_log[0], utc_gp[0]) + 5, min(t_log[-1], utc_gp[-1]) - 5, 0.02)
    if len(grid) < 500:
        raise SystemExit("[sync] la vidéo et le log ne se recouvrent pas dans le temps")
    vo = np.interp(grid, t_log, v_log)

    def err(lag):
        return np.mean((vo - np.interp(grid - lag, utc_gp, v_gp)) ** 2)

    coarse = min(np.arange(-5, 5, 0.05), key=err)
    fine = min(np.arange(coarse - 0.1, coarse + 0.1, 0.005), key=err)
    print(f"[sync] décalage GoPro -> log {fine:+.3f} s, écart vitesse {err(fine) ** .5:.1f} km/h")
    return float(fine)


def refine_lag_gyro(imu_t, imu_gyro_dps, gp_t, gp_gyro_rads, lag: float) -> float:
    """Refine the offset on angular rate: GoPro gyro samples are tied to the
    video frames at 200 Hz, while both GPS speeds carry their own processing
    delay (about 0.1 s apart). Rotation norms are mount-independent."""
    from imu import lowpass
    wn = np.linalg.norm(imu_gyro_dps, axis=1)
    gn = np.degrees(np.linalg.norm(gp_gyro_rads, axis=1))
    grid = np.arange(max(imu_t[0], gp_t[0]) + 5, min(imu_t[-1], gp_t[-1]) - 5, 0.01)
    if len(grid) < 3000:
        return lag
    o = lowpass(np.interp(grid, imu_t, wn), 100, 3)
    g = lowpass(np.interp(grid, gp_t, gn), 100, 3)
    lags = np.arange(lag - 0.3, lag + 0.3, 0.005)
    cs = [np.corrcoef(o, np.interp(grid - l, grid, g))[0, 1] for l in lags]
    i = int(np.argmax(cs))
    if cs[i] < 0.3:
        print(f"[sync] gyroscopes trop peu corrélés ({cs[i]:.2f}) : calage sur la vitesse gardé")
        return lag
    print(f"[sync] affiné sur les gyroscopes : {lags[i]:+.3f} s (corrélation {cs[i]:.2f})")
    return float(lags[i])


# ---------------------------------------------------------------------------
# Session model: everything the overlay needs at a given log time


class Session:
    def __init__(self, log: Path):
        fixes = parse_nmea(log)
        self.t = np.array([f.t for f in fixes])
        lat = np.array([f.lat for f in fixes])
        lon = np.array([f.lon for f in fixes])
        self.speed = np.array([f.speed_kmh for f in fixes])
        self.lean, _, self.a_lon = session_dynamics(log, fixes)
        name = detect_track(lat, lon)
        if not name:
            raise SystemExit("[track] circuit non reconnu")
        self.track = load_track(name)
        proj = Projection(lat.mean(), lon.mean())
        self.x, self.y = proj(lat, lon)
        cl = np.array(self.track["centerline"])
        self.ox, self.oy = proj(cl[:, 0], cl[:, 1])
        width = self.track.get("gate_width_m", 40)
        fin = self.track["finish_m"]
        self.gates = [gate_on_centerline("S/F", self.ox, self.oy, fin, width)]
        self.gates += [gate_on_centerline(f"S{k}", self.ox, self.oy, fin + d, width)
                       for k, d in enumerate(self.track.get("sectors_m", []), start=1)]
        self.sf = np.array(gate_crossings(self.x, self.y, self.t, self.gates[0], 20.0))
        self.sec = [np.array(gate_crossings(self.x, self.y, self.t, g)) for g in self.gates[1:]]
        self.prog, self.length = track_progress(self.x, self.y, self.ox, self.oy, fin)
        self.n_sec = len(self.gates)
        # Completed laps: (start, end, sector times or None)
        self.laps = []
        for a, b in zip(self.sf, self.sf[1:]):
            marks = [a]
            for sc in self.sec:
                inside = sc[(sc > marks[-1]) & (sc < b)]
                marks.append(inside[0] if len(inside) else None)
            marks.append(b)
            secs = None if None in marks else list(np.diff(marks))
            self.laps.append((a, b, secs))
        self.ref = [self._lap_curve(a, b) for a, b, _ in self.laps]
        # Lean drawn with a little smoothing; NaN (slow) shown as upright.
        self.lean_f = np.nan_to_num(self.lean)
        self.lean_max = []  # per completed/ongoing lap: (max left, max right)
        print(f"[session] {len(self.laps)} tours complets, circuit {self.track['name']}")

    def _lap_curve(self, a, b):
        """(distance along the lap, elapsed time) for the fixes of a lap."""
        m = (self.t >= a) & (self.t <= b)
        s = self.prog[m].copy()
        el = self.t[m] - a
        s[(el < 10) & (s > self.length * 0.8)] -= self.length
        s[(el > (b - a) - 10) & (s < self.length * 0.2)] += self.length
        return np.maximum.accumulate(s), el

    def state(self, tl: float) -> dict | None:
        if tl < self.t[0] or tl > self.t[-1]:
            return None
        st = {
            "speed": float(np.interp(tl, self.t, self.speed)),
            "lean": float(np.interp(tl, self.t, self.lean_f)),
            "g": float(np.interp(tl, self.t, np.nan_to_num(self.a_lon))),
            "x": float(np.interp(tl, self.t, self.x)),
            "y": float(np.interp(tl, self.t, self.y)),
        }
        k = int(np.searchsorted(self.sf, tl))  # laps started so far
        done = [l for l in self.laps if l[1] <= tl]
        st["best"] = min((b - a for a, b, _ in done), default=None)
        st["last"] = (done[-1][1] - done[-1][0]) if done else None
        st["lap_no"] = k
        if k == 0:
            st["elapsed"] = None
            start = self.t[0]
        else:
            start = self.sf[k - 1]
            st["elapsed"] = tl - start
        # Sector times of the current lap, and best sector times so far.
        best_sec = [min((l[2][i] for l in done if l[2]), default=None) for i in range(self.n_sec)]
        cur, marks = [], [start]
        nxt = self.sf[k] if k < len(self.sf) else np.inf
        for sc in self.sec:
            inside = sc[(sc > marks[-1]) & (sc < min(tl, nxt))]
            if not len(inside):
                break
            cur.append(inside[0] - marks[-1])
            marks.append(inside[0])
        st["sectors"], st["best_sec"] = (cur if k else []), best_sec
        st["sec_start"] = marks[-1]
        # Live delta against the best completed lap.
        st["delta"] = None
        if k and done:
            bi = min(range(len(done)), key=lambda i: done[i][1] - done[i][0])
            s_ref, t_ref = self.ref[bi]
            s_now = float(np.interp(tl, self.t, self.prog))
            if st["elapsed"] < 10 and s_now > self.length * 0.8:
                s_now -= self.length
            if 30 < s_now < s_ref[-1]:
                st["delta"] = st["elapsed"] - float(np.interp(s_now, s_ref, t_ref))
        # Max lean of the current lap so far.
        m = (self.t >= start) & (self.t <= tl)
        ln = self.lean_f[m]
        st["lmax"] = (float(-ln.min()) if ln.size and ln.min() < 0 else 0.0, float(ln.max()) if ln.size and ln.max() > 0 else 0.0)
        return st


# ---------------------------------------------------------------------------
# Drawing


class Overlay:
    """Landscape: lap panel top left, map bottom left, gauges bottom right.
    Portrait (camera mounted sideways): lap panel across the top, map and
    gauges side by side at the bottom."""

    def __init__(self, s: Session, w: int, h: int):
        self.s, self.W, self.H = s, w, h
        W, H = w, h
        portrait = h > w
        self.f_time = font(BOLD, 86)
        self.f_big = font(BOLD, 64)
        self.f_mid = font(BOLD, 34)
        self.f_small = font(REG, 26)
        self.f_label = font(BOLD, 22)
        self.f_mono = font(MONO, 26)
        self.base = Image.new("RGBA", (W, H), (0, 0, 0, 0))
        d = ImageDraw.Draw(self.base)
        # Lap panel (top left)
        self.lap_box = (MARGIN, MARGIN, (W - MARGIN) if portrait else MARGIN + 560, MARGIN + 250)
        d.rounded_rectangle(self.lap_box, 18, fill=PANEL)
        # Map panel (bottom left)
        size = 400 if portrait else 380
        self.map_box = (MARGIN, H - MARGIN - size, MARGIN + size, H - MARGIN)
        d.rounded_rectangle(self.map_box, 18, fill=PANEL)
        self._map_transform(size - 50)
        pts = [self.to_map(x, y) for x, y in zip(np.append(s.ox, s.ox[0]), np.append(s.oy, s.oy[0]))]
        d.line(pts, fill=(0, 0, 0, 200), width=13, joint="curve")
        d.line(pts, fill=TRACK, width=7, joint="curve")
        for i, g in enumerate(s.gates):
            p, q = self.to_map(g.x + g.hx * 14, g.y + g.hy * 14), self.to_map(g.x - g.hx * 14, g.y - g.hy * 14)
            d.line([p, q], fill=YELLOW if i == 0 else MUTED, width=4)
        placed = []  # corner initials, outside the turn, skipped when they would collide
        for c in s.track.get("corners", []):
            g = gate_on_centerline("", s.ox, s.oy, s.track["finish_m"] + (c["start_m"] + c["end_m"]) / 2, 0)
            side = -1 if c["dir"].startswith("G") else 1
            px, py = self.to_map(g.x + side * g.hy * 75, g.y - side * g.hx * 75)
            if all(abs(px - qx) > 22 or abs(py - qy) > 22 for qx, qy in placed):
                d.text((px, py), c["name"][0], font=self.f_label, fill=MUTED, anchor="mm")
                placed.append((px, py))
        # Gauge panel (bottom right)
        self.g_box = ((MARGIN + size + 30) if portrait else (W - MARGIN - 500), H - MARGIN - size, W - MARGIN, H - MARGIN)
        d.rounded_rectangle(self.g_box, 18, fill=PANEL)
        gx0, gy0, gx1, _ = self.g_box
        self.gc = ((gx0 + gx1) // 2, gy0 + 195 + (10 if portrait else 0))
        self.gr = 165
        cx, cy = self.gc
        for a in range(-60, 61, 15):
            r0 = self.gr - (26 if a % 30 == 0 else 16)
            p0 = self._polar(a, r0)
            p1 = self._polar(a, self.gr)
            col = RED if abs(a) >= 60 else (YELLOW if abs(a) >= 45 else MUTED)
            d.line([p0, p1], fill=col, width=4 if a % 30 == 0 else 2)
            if a and a % 30 == 0:
                d.text(self._polar(a, self.gr + 22), f"{abs(a)}", font=self.f_label, fill=MUTED, anchor="mm")
        d.arc([cx - self.gr, cy - self.gr, cx + self.gr, cy + self.gr], 210, 330, fill=MUTED, width=2)
        d.text((gx0 + 24, gy0 + 18), "ANGLE", font=self.f_label, fill=MUTED)

    # map coordinates
    def _map_transform(self, inner):
        s = self.s
        x0, x1, y0, y1 = s.ox.min(), s.ox.max(), s.oy.min(), s.oy.max()
        self.k = inner / max(x1 - x0, y1 - y0)
        bx0, by0, bx1, by1 = self.map_box
        self.mcx, self.mcy = (bx0 + bx1) / 2, (by0 + by1) / 2
        self.xc, self.yc = (x0 + x1) / 2, (y0 + y1) / 2

    def to_map(self, x, y):
        return (self.mcx + (x - self.xc) * self.k, self.mcy - (y - self.yc) * self.k)

    def _polar(self, lean_deg, r):
        a = np.radians(lean_deg)
        cx, cy = self.gc
        return (cx + r * np.sin(a), cy - r * np.cos(a))

    def frame(self, st: dict | None) -> Image.Image:
        img = self.base.copy()
        if st is None:
            return Image.new("RGBA", (self.W, self.H), (0, 0, 0, 0))
        d = ImageDraw.Draw(img)
        self._lap(d, st)
        # position on the map
        px, py = self.to_map(st["x"], st["y"])
        d.ellipse([px - 11, py - 11, px + 11, py + 11], fill=(0, 0, 0, 230))
        d.ellipse([px - 8, py - 8, px + 8, py + 8], fill=YELLOW)
        self._gauge(d, st)
        return img

    def _lap(self, d, st):
        x0, y0, x1, _ = self.lap_box
        if st["lap_no"] == 0:
            d.text((x0 + 28, y0 + 24), "TOUR DE SORTIE", font=self.f_mid, fill=MUTED)
        else:
            d.text((x0 + 28, y0 + 22), f"TOUR {st['lap_no']}", font=self.f_label, fill=MUTED)
            d.text((x0 + 26, y0 + 44), fmt_lap(st["elapsed"])[:-2], font=self.f_time, fill=WHITE)
        if st["delta"] is not None:
            col = GREEN if st["delta"] < 0 else RED
            d.text((x1 - 28, y0 + 60), fmt_delta(st["delta"]), font=self.f_big, fill=col, anchor="ra")
            d.text((x1 - 28, y0 + 30), "vs meilleur", font=self.f_small, fill=MUTED, anchor="ra")
        # sectors
        sy = y0 + 150
        w = (x1 - x0 - 56) / self.s.n_sec
        for i in range(self.s.n_sec):
            sx = x0 + 28 + i * w
            label = f"S{i + 1}"
            if i < len(st["sectors"]):
                v = st["sectors"][i]
                b = st["best_sec"][i]
                col = PURPLE if b is None or v <= b + 1e-6 else WHITE
                txt = f"{v:.2f}"
            elif i == len(st["sectors"]) and st["lap_no"]:
                col, txt = MUTED, "…"
            else:
                col, txt = MUTED, "--"
            d.text((sx, sy), label, font=self.f_label, fill=MUTED)
            d.text((sx + 34, sy - 6), txt, font=self.f_mid, fill=col)
        line = []
        if st["best"] is not None:
            line.append(f"Meilleur {fmt_lap(st['best'])}")
        if st["last"] is not None:
            line.append(f"Dernier {fmt_lap(st['last'])}")
        d.text((x0 + 28, y0 + 200), "    ".join(line), font=self.f_small, fill=MUTED)

    def _gauge(self, d, st):
        cx, cy = self.gc
        lean = max(-65.0, min(65.0, st["lean"]))
        # lap max marks
        for side, v in ((-1, st["lmax"][0]), (1, st["lmax"][1])):
            if v > 1:
                a = side * min(v, 65)
                p = self._polar(a, self.gr + 2)
                q1, q2 = self._polar(a - 3, self.gr + 16), self._polar(a + 3, self.gr + 16)
                d.polygon([p, q1, q2], fill=YELLOW)
        tip = self._polar(lean, self.gr - 8)
        d.line([(cx, cy), tip], fill=(0, 0, 0, 220), width=12)
        d.line([(cx, cy), tip], fill=WHITE, width=6)
        d.ellipse([cx - 12, cy - 12, cx + 12, cy + 12], fill=WHITE)
        side = "G" if lean < -0.5 else ("D" if lean > 0.5 else "")
        d.text((cx, cy + 52), f"{abs(lean):.0f}°", font=self.f_big, fill=WHITE, anchor="mm")
        if side:
            d.text((cx + (-1 if side == "G" else 1) * 78, cy + 56), side, font=self.f_mid, fill=MUTED, anchor="mm")
        # speed
        gx0, gy0, gx1, gy1 = self.g_box
        d.text((gx0 + 28, gy1 - 30), f"{st['speed']:.0f}", font=self.f_big, fill=WHITE, anchor="ls")
        d.text((gx0 + 28 + d.textlength(f"{st['speed']:.0f}", font=self.f_big) + 8, gy1 - 32), "km/h",
               font=self.f_small, fill=MUTED, anchor="ls")
        # longitudinal g bar: braking left (red), throttle right (green)
        bx0, bx1, by = gx0 + 240, gx1 - 28, gy1 - 62
        mid = (bx0 + bx1) / 2
        d.rounded_rectangle([bx0, by - 9, bx1, by + 9], 9, fill=(255, 255, 255, 40))
        g = max(-1.4, min(1.4, st["g"]))
        span = (bx1 - bx0) / 2 / 1.4
        if g < 0:
            d.rounded_rectangle([mid + g * span, by - 9, mid, by + 9], 9, fill=RED)
        elif g > 0:
            d.rounded_rectangle([mid, by - 9, mid + g * span, by + 9], 9, fill=GREEN)
        d.line([(mid, by - 14), (mid, by + 14)], fill=WHITE, width=2)
        d.text((mid, by - 18), f"{g:+.2f} g".replace("-", "−"), font=self.f_label, fill=WHITE if abs(g) > 0.05 else MUTED, anchor="ms")
        d.text((bx0, by + 36), "FREIN", font=self.f_label, fill=MUTED, anchor="ls")
        d.text((bx1, by + 36), "GAZ", font=self.f_label, fill=MUTED, anchor="rs")


# ---------------------------------------------------------------------------


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", type=Path)
    ap.add_argument("videos", type=Path, nargs="+", help="GoPro chapters in order")
    ap.add_argument("--start", type=float, default=0.0, help="start of the render in the video (s)")
    ap.add_argument("--duration", type=float, help="length of the render (s, default: to the end)")
    ap.add_argument("--lag", type=float, help="force the GoPro -> log time offset (s)")
    ap.add_argument("--still", type=float, action="append", default=[], help="only write a PNG of the overlay at video time T")
    ap.add_argument("--crf", type=int, default=22, help="x264 quality (lower = better, bigger)")
    ap.add_argument("--out", type=Path)
    args = ap.parse_args()

    # Video time of each chapter start, and its clock.
    chapters, offset = [], 0.0
    gp_utc, gp_speed, gp_gt, gp_gyro = [], [], [], []
    for v in args.videos:
        tel = gopro.read_telemetry(v)
        _, _, fps, dur = gopro.video_info(v)
        a, b = chapter_clock(tel)
        chapters.append((offset, dur, a, b))
        ok = tel.gps_fix >= 2
        gp_utc.append(a * tel.gps_t[ok] + b)
        gp_speed.append(tel.speed_kmh[ok])
        gp_gt.append(a * tel.gyro_t + b)
        gp_gyro.append(tel.gyro)
        offset += dur
    total = offset
    sess = Session(args.log)
    if args.lag is not None:
        lag = args.lag
    else:
        lag = find_lag(sess.t, sess.speed, np.concatenate(gp_utc), np.concatenate(gp_speed))
        im = load_imu(args.log)
        if im is not None and sum(len(g) for g in gp_gyro):
            lag = refine_lag_gyro(im.t, im.gyro, np.concatenate(gp_gt), np.concatenate(gp_gyro), lag)

    def log_time(T):
        for off, dur, a, b in chapters:
            if T < off + dur or (off, dur, a, b) == chapters[-1]:
                return a * (T - off) + b + lag

    W, H = display_size(args.videos[0])
    print(f"[video] {W}x{H}{' (vertical)' if H > W else ''}")
    ov = Overlay(sess, W, H)
    out = args.out or Path("out") / args.log.stem / "overlay.mp4"
    out.parent.mkdir(parents=True, exist_ok=True)

    if args.still:
        for T in args.still:
            p = out.with_name(f"overlay_{T:.0f}s.png")
            frame = ov.frame(sess.state(log_time(T)))
            ff = imageio_ffmpeg.get_ffmpeg_exe()
            # composite on the actual video frame for review
            ch = max(i for i, c in enumerate(chapters) if c[0] <= T)
            raw = subprocess.run([ff, "-v", "error", "-ss", f"{T - chapters[ch][0]:.3f}", "-i", str(args.videos[ch]),
                                  "-frames:v", "1", "-f", "rawvideo", "-pix_fmt", "rgba", "-"],
                                 capture_output=True, check=True).stdout
            bg = Image.frombytes("RGBA", (W, H), raw)
            Image.alpha_composite(bg, frame).convert("RGB").save(p)
            print(f"[still] {p}")
        return 0

    dur = args.duration if args.duration is not None else total - args.start
    # Concat list starting at the chapter holding --start, seeked with inpoint
    # (an input -ss on the concat demuxer would decode everything before it).
    concat = out.with_suffix(".txt")
    first = max(i for i, c in enumerate(chapters) if c[0] <= args.start)
    lines = []
    for i, v in enumerate(args.videos[first:], start=first):
        lines.append(f"file '{v.resolve()}'\n")
        if i == first and args.start > chapters[i][0]:
            lines.append(f"inpoint {args.start - chapters[i][0]:.3f}\n")
    concat.write_text("".join(lines))
    ff = imageio_ffmpeg.get_ffmpeg_exe()
    cmd = [ff, "-v", "error", "-stats", "-y",
           "-f", "concat", "-safe", "0", "-i", str(concat),
           "-f", "rawvideo", "-pix_fmt", "rgba", "-s", f"{W}x{H}", "-framerate", "30000/1001", "-i", "-",
           "-filter_complex", "[0:v][1:v]overlay=0:0:eof_action=pass:format=auto[v]",
           "-map", "[v]", "-map", "0:a:0?", "-t", f"{dur:.3f}",
           "-c:v", "libx264", "-preset", "veryfast", "-crf", str(args.crf), "-pix_fmt", "yuv420p",
           "-c:a", "copy", "-movflags", "+faststart", str(out)]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    n = int(dur * FPS) + 1
    t0 = time.time()
    try:
        for i in range(n):
            T = args.start + i / FPS
            proc.stdin.write(ov.frame(sess.state(log_time(T))).tobytes())
            if i % 300 == 0 and i:
                el = time.time() - t0
                print(f"[render] {i}/{n} images, reste ~{el / i * (n - i) / 60:.1f} min", file=sys.stderr)
    except BrokenPipeError:
        pass
    proc.stdin.close()
    proc.wait()
    print(f"[out] {out}")
    return proc.returncode


if __name__ == "__main__":
    sys.exit(main())
