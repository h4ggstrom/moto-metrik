#!/usr/bin/env python3
"""Sim-racing style HUD for onboard videos, drawn with Skia (anti-aliased).

Widgets, in the spirit of broadcast / sim-racing HUDs:
  - delta bar: a bar filling from the centre (green ahead, red behind) and
    the live delta;
  - sectors: current (lap number) / best / last / ideal lap, sector cells and
    micro-sectors coloured purple (session best), green (personal best),
    grey (no improvement);
  - map (--no-map hides it): track with micro-sectors coloured the same way,
    sector lines, bike;
  - dash: speed, longitudinal g bar (braking / accelerating) and a
    steering-wheel style lean ring (marker at the lean angle, value inside).

Design iteration: `python tools/hud.py --demo BG.png OUT.png` draws made-up
values over a video frame.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import skia

FONTS = Path(__file__).parent / "fonts"


def tf(name: str) -> skia.Typeface:
    return skia.Typeface.MakeFromFile(str(FONTS / name))


WIDE = tf("Michroma-Regular.ttf")
LABEL = tf("Barlow-SemiBold.ttf")
LABEL_B = tf("Barlow-Bold.ttf")


def rgba(hexstr: str, a: float = 1.0) -> int:
    h = hexstr.lstrip("#")
    return skia.Color(int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16), int(a * 255))


PANEL = rgba("#0a1424", 0.80)
PANEL_EDGE = rgba("#ffffff", 0.10)
WHITE = rgba("#f5f7fa")
MUTED = rgba("#9aa6b4")
DIM = rgba("#5c6878")
PURPLE = rgba("#b44cff")
GREEN = rgba("#1fd16b")
YELLOW = rgba("#ffcc1a")
RED = rgba("#ff3b3b")
GREY = rgba("#4a5566")
STATUS = {"purple": PURPLE, "green": GREEN, "grey": GREY, None: rgba("#ffffff", 0.08)}


def fmt_lap(s: float | None, tenths: bool = False) -> str:
    if s is None:
        return "-:--.---"
    m, sec = divmod(s, 60)
    return f"{int(m)}:{sec:04.1f}" if tenths else f"{int(m)}:{sec:06.3f}"


def fmt_delta(d: float) -> str:
    return f"{'+' if d >= 0 else '−'}{abs(d):.2f}"


@dataclass
class HudState:
    lap: int
    lap_time: float  # running time of the current lap
    delta: float | None  # live delta to the best lap, s (negative = ahead)
    best: float | None
    last: float | None
    ideal: float | None  # sum of the best sectors so far
    sectors: list  # per sector: time or None
    sector_status: list  # per sector: purple / green / grey / None
    micro: list  # per micro-sector: purple / green / grey / None (not yet)
    speed: float  # km/h
    lean: float  # deg, + = right
    lean_max: tuple  # (left, right) max of the lap
    g_lon: float  # + = throttle, - = brake
    progress: float  # 0..1 along the lap
    pos: tuple | None = None  # bike position, metres, same frame as the centerline


class Hud:
    def __init__(self, w: int, h: int, centerline_xy: np.ndarray, sector_frac: list, micro_frac: list | None = None,
                 finish_frac: float = 0.0, show_map: bool = True):
        """centerline_xy: (n, 2) metres, closed. Lap fractions (0 = finish line):
        sector_frac for the sector lines, micro_frac for the micro-sector
        boundaries (n + 1 values, default: even split). finish_frac: where the
        finish line sits along the centerline, as a fraction of its length."""
        self.w, self.h = w, h
        self.cl = centerline_xy
        self.sector_frac = sector_frac
        self.micro_frac = micro_frac
        self.finish_frac = finish_frac
        seg = np.hypot(*np.diff(np.vstack([self.cl, self.cl[:1]]), axis=0).T)
        self.cum = np.concatenate([[0], np.cumsum(seg)]) / seg.sum()
        self.portrait = h > w
        self.show_map = show_map

    # -- primitives ---------------------------------------------------------
    @staticmethod
    def paint(color, stroke: float | None = None, **kw) -> skia.Paint:
        p = skia.Paint(AntiAlias=True, Color=color, **kw)
        if stroke is not None:
            p.setStyle(skia.Paint.kStroke_Style)
            p.setStrokeWidth(stroke)
            p.setStrokeCap(skia.Paint.kRound_Cap)
            p.setStrokeJoin(skia.Paint.kRound_Join)
        return p

    def text(self, c, s, x, y, face, size, color, align="left", spacing=0.0):
        font = skia.Font(face, size)
        font.setEdging(skia.Font.Edging.kAntiAlias)
        width = font.measureText(s) + spacing * (len(s) - 1)
        if align == "center":
            x -= width / 2
        elif align == "right":
            x -= width
        if spacing:
            for ch in s:
                c.drawString(ch, x, y, font, self.paint(color))
                x += font.measureText(ch) + spacing
        else:
            c.drawString(s, x, y, font, self.paint(color))
        return width

    def panel(self, c, x, y, w, h, r=16, stripes=True):
        rect = skia.Rect.MakeXYWH(x, y, w, h)
        shadow = self.paint(rgba("#000000", 0.45))
        shadow.setMaskFilter(skia.MaskFilter.MakeBlur(skia.kNormal_BlurStyle, 10))
        c.drawRoundRect(rect.makeOffset(0, 6), r, r, shadow)
        c.drawRoundRect(rect, r, r, self.paint(PANEL))
        if stripes:  # faint diagonal texture
            c.save()
            c.clipRRect(skia.RRect.MakeRectXY(rect, r, r), doAntiAlias=True)
            p = self.paint(rgba("#ffffff", 0.025), 10)
            for k in range(-int(h), int(w), 34):
                c.drawLine(x + k, y + h, x + k + h, y, p)
            c.restore()
        c.drawRoundRect(rect, r, r, self.paint(PANEL_EDGE, 1.5))

    def pill(self, c, cx, y, w, h, color=PANEL):
        rect = skia.Rect.MakeXYWH(cx - w / 2, y, w, h)
        shadow = self.paint(rgba("#000000", 0.4))
        shadow.setMaskFilter(skia.MaskFilter.MakeBlur(skia.kNormal_BlurStyle, 8))
        c.drawRoundRect(rect.makeOffset(0, 4), h / 2, h / 2, shadow)
        c.drawRoundRect(rect, h / 2, h / 2, self.paint(color))
        c.drawRoundRect(rect, h / 2, h / 2, self.paint(PANEL_EDGE, 1.5))

    # -- widgets ------------------------------------------------------------
    def deltabar(self, c, st: HudState, cx, y, w):
        """Bar filling from the centre (green ahead, red behind), then the live delta."""
        by, bh = y, 46
        bx = cx - w / 2
        rect = skia.Rect.MakeXYWH(bx, by, w, bh)
        c.drawRoundRect(rect, bh / 2, bh / 2, self.paint(rgba("#0a1424", 0.72)))
        c.save()
        c.clipRRect(skia.RRect.MakeRectXY(rect, bh / 2, bh / 2), doAntiAlias=True)
        dot = self.paint(rgba("#ffffff", 0.10))
        for gx in range(int(bx) + 8, int(bx + w), 10):
            for gy in range(int(by) + 7, int(by + bh), 10):
                c.drawCircle(gx, gy, 1.4, dot)
        if st.delta is not None:
            span = max(-1.0, min(1.0, st.delta / 2.0))  # full scale = 2 s
            col = GREEN if st.delta < 0 else RED
            x1 = cx + span * w / 2
            grad = skia.GradientShader.MakeLinear(
                [skia.Point(cx, 0), skia.Point(x1, 0)], [rgba("#000000", 0.0) & 0x00FFFFFF | (col & 0x00FFFFFF), col])
            fill = skia.Paint(AntiAlias=True, Shader=grad)
            c.drawRect(skia.Rect.MakeLTRB(min(cx, x1), by, max(cx, x1), by + bh), fill)
            c.drawRoundRect(skia.Rect.MakeXYWH(x1 - 3, by + 6, 6, bh - 12), 3, 3, self.paint(WHITE))
        c.restore()
        c.drawLine(cx, by + 4, cx, by + bh - 4, self.paint(rgba("#ffffff", 0.35), 2))
        c.drawRoundRect(rect, bh / 2, bh / 2, self.paint(PANEL_EDGE, 1.5))
        # live delta pill
        if st.delta is not None:
            dy = by + bh + 14
            self.pill(c, cx, dy, 260, 84)
            self.text(c, fmt_delta(st.delta), cx, dy + 60, WIDE, 46, GREEN if st.delta < 0 else RED, "center")
        return by + bh + 14 + 84

    def sectors(self, c, st: HudState, x, y, w):
        h = 196
        self.panel(c, x, y, w, h)
        rows = [(f"Tour {st.lap}", fmt_lap(st.lap_time, True)), ("Meilleur", fmt_lap(st.best)),
                ("Dernier", fmt_lap(st.last)), ("Idéal", fmt_lap(st.ideal))]
        colw = (w - 48) / 2
        for i, (lab, val) in enumerate(rows):
            cx0 = x + 24 + (i % 2) * colw
            cy = y + 44 + (i // 2) * 46
            self.text(c, lab, cx0, cy, LABEL, 26, MUTED)
            self.text(c, val, cx0 + colw - 20, cy, WIDE, 24, PURPLE if lab == "Meilleur" else WHITE, "right")
        # sector cells
        sy, sh = y + 116, 40
        n = len(st.sectors)
        cw = (w - 48) / n
        per = len(st.micro) // n
        for k in range(n):
            sx = x + 24 + k * cw
            rect = skia.Rect.MakeXYWH(sx + 2, sy, cw - 4, sh)
            col = STATUS.get(st.sector_status[k]) if st.sectors[k] is not None else rgba("#ffffff", 0.06)
            c.drawRoundRect(rect, 6, 6, self.paint(col))
            label = f"S{k + 1}" if st.sectors[k] is None else f"{st.sectors[k]:.2f}"
            self.text(c, label, sx + cw / 2, sy + 29, LABEL_B, 26, WHITE, "center")
            # micro-sectors under the cell
            mw = (cw - 4) / per
            for j in range(per):
                s = st.micro[k * per + j]
                mr = skia.Rect.MakeXYWH(sx + 2 + j * mw + 1.5, sy + sh + 10, mw - 3, 12)
                c.drawRoundRect(mr, 2, 2, self.paint(STATUS[s]))
        return y + h

    def track_map(self, c, st: HudState, x, y, w, h):
        self.panel(c, x, y, w, h, stripes=False)
        pts = self.cl
        x0, y0 = pts.min(axis=0)
        x1, y1 = pts.max(axis=0)
        k = min((w - 70) / (x1 - x0), (h - 100) / (y1 - y0))  # room for the title
        ox = x + w / 2 - (x0 + x1) / 2 * k
        oy = y + h / 2 + 15 + (y0 + y1) / 2 * k
        P = [(ox + px * k, oy - py * k) for px, py in pts]

        def path(i0, i1):
            p = skia.Path()
            p.moveTo(*P[i0 % len(P)])
            for i in range(i0 + 1, i1 + 1):
                p.lineTo(*P[i % len(P)])
            return p

        full = path(0, len(P))
        c.drawPath(full, self.paint(rgba("#000000", 0.85), 16))
        c.drawPath(full, self.paint(rgba("#ffffff", 0.92), 10))
        def idx(lap_frac):  # centerline index of a lap fraction
            return int(np.searchsorted(self.cum, (lap_frac + self.finish_frac) % 1.0))

        # micro-sectors done this lap, coloured
        n = len(st.micro)
        bounds = self.micro_frac or [m / n for m in range(n + 1)]
        for m, s in enumerate(st.micro):
            if s is None:
                continue
            i0, i1 = idx(bounds[m]), idx(bounds[m + 1] - 1e-9)
            if i1 < i0:
                i1 += len(P)
            c.drawPath(path(max(i0 - 1, 0), i1), self.paint(STATUS[s], 6))
        # sector lines and finish
        for j, f in enumerate([0.0] + list(self.sector_frac)):
            i = idx(f) % len(P)
            a, b = np.array(P[i]), np.array(P[(i + 1) % len(P)])
            d = (b - a) / (np.linalg.norm(b - a) + 1e-9)
            nrm = np.array([-d[1], d[0]]) * 14
            c.drawLine(*(a - nrm), *(a + nrm), self.paint(RED if j == 0 else WHITE, 4))
        # bike: real position when known, else interpolated along the centerline
        # (centerline vertices are ~13 m apart: snapping to them stutters)
        if st.pos is not None:
            bx, by = ox + st.pos[0] * k, oy - st.pos[1] * k
        else:
            f = (st.progress + self.finish_frac) % 1.0
            i = min(int(np.searchsorted(self.cum, f)), len(P) - 1)
            i0 = max(i - 1, 0)
            u = (f - self.cum[i0]) / max(self.cum[i] - self.cum[i0], 1e-9)
            bx = P[i0][0] + u * (P[i % len(P)][0] - P[i0][0])
            by = P[i0][1] + u * (P[i % len(P)][1] - P[i0][1])
        glow = self.paint(rgba("#ffcc1a", 0.45))
        glow.setMaskFilter(skia.MaskFilter.MakeBlur(skia.kNormal_BlurStyle, 8))
        c.drawCircle(bx, by, 16, glow)
        c.drawCircle(bx, by, 13, self.paint(rgba("#000000", 0.9)))
        c.drawCircle(bx, by, 10, self.paint(YELLOW))
        self.text(c, "CARTE", x + 20, y + 34, LABEL_B, 22, MUTED, spacing=2)

    def lean_ring(self, c, st: HudState, cx, cy, r):
        ring = skia.Rect.MakeXYWH(cx - r, cy - r, 2 * r, 2 * r)
        c.drawOval(ring, self.paint(rgba("#05080e", 0.75)))
        c.drawArc(ring, 0, 360, False, self.paint(rgba("#ffffff", 0.12), 14))
        # max lean of the lap, both sides (0 deg = top)
        for side, v in ((-1, st.lean_max[0]), (1, st.lean_max[1])):
            if v > 1:
                start = -90.0 if side > 0 else -90.0 - v
                c.drawArc(ring, start, v, False, self.paint(rgba("#ffcc1a", 0.55), 14))
        # current lean arc and marker
        lean = max(-70.0, min(70.0, st.lean))
        if abs(lean) > 0.5:
            start = -90.0 if lean > 0 else -90.0 + lean
            c.drawArc(ring, start, abs(lean), False, self.paint(WHITE, 14))
        a = math.radians(lean - 90)
        mx, my = cx + r * math.cos(a), cy + r * math.sin(a)
        c.drawRoundRect(skia.Rect.MakeXYWH(mx - 9, my - 9, 18, 18), 4, 4, self.paint(WHITE))
        c.drawLine(cx, cy - r - 12, cx, cy - r + 12, self.paint(rgba("#ffffff", 0.6), 2))
        side = "GAUCHE" if lean < -0.5 else ("DROITE" if lean > 0.5 else "")
        num = f"{abs(lean):.0f}"
        nw = skia.Font(WIDE, 54).measureText(num)
        self.text(c, num, cx - 8, cy + 20, WIDE, 54, WHITE, "center")
        self.text(c, "°", cx - 8 + nw / 2 + 2, cy - 2, LABEL_B, 40, WHITE)
        if side:
            self.text(c, side, cx, cy + 62, LABEL_B, 24, MUTED, "center", spacing=2)
        self.text(c, "ANGLE", cx, cy - 44, LABEL_B, 22, MUTED, "center", spacing=2)

    def g_bar(self, c, st: HudState, x, y, w):
        """Longitudinal g, centred on zero: braking fills left (red), accelerating right (green)."""
        bh, full = 22, 1.3  # bar height, g at full scale
        g = max(-full, min(full, st.g_lon))
        self.text(c, "G LONG.", x, y - 16, LABEL_B, 20, MUTED, spacing=2)
        self.text(c, f"{abs(g):.2f} g", x + w, y - 14, WIDE, 24, RED if g < -0.05 else (GREEN if g > 0.05 else WHITE), "right")
        mid = x + w / 2
        c.drawRoundRect(skia.Rect.MakeXYWH(x, y, w, bh), bh / 2, bh / 2, self.paint(rgba("#ffffff", 0.08)))
        for gv in (-1.0, -0.5, 0.5, 1.0):  # graduations
            gx = mid + gv / full * w / 2
            c.drawLine(gx, y + 5, gx, y + bh - 5, self.paint(rgba("#ffffff", 0.18), 1.5))
        x1 = mid + g / full * w / 2
        if abs(x1 - mid) > 1:
            c.drawRoundRect(skia.Rect.MakeLTRB(min(mid, x1), y, max(mid, x1), y + bh), bh / 2, bh / 2,
                            self.paint(RED if g < 0 else GREEN))
        c.drawLine(mid, y - 6, mid, y + bh + 6, self.paint(WHITE, 2))
        ty = y + bh + 30
        self.text(c, "FREINAGE", x, ty, LABEL_B, 17, RED if g < -0.05 else MUTED, spacing=1)
        self.text(c, "ACCÉLÉRATION", x + w, ty, LABEL_B, 17, GREEN if g > 0.05 else MUTED, "right", spacing=1)

    def dash(self, c, st: HudState, x, y, w, h):
        """Speed and longitudinal g on the left, lean ring on the right."""
        self.panel(c, x, y, w, h)
        r = (h - 64) / 2
        cx = x + w - 30 - r
        lx, lw = x + 28, cx - r - 36 - (x + 28)
        self.text(c, "VITESSE", lx, y + 44, LABEL_B, 20, MUTED, spacing=2)
        sw = self.text(c, f"{st.speed:.0f}", lx, y + 128, WIDE, 70, WHITE)
        self.text(c, "km/h", lx + sw + 12, y + 128, LABEL, 26, MUTED)
        self.g_bar(c, st, lx, y + 200, lw)
        self.lean_ring(c, st, cx, y + h / 2, r)

    # -- frame --------------------------------------------------------------
    def draw(self, c, st: HudState):
        W, H = self.w, self.h
        m = 40
        th = 330  # bottom row height: map (square) + dash
        if self.portrait:
            bottom = self.sectors(c, st, m, m, W - 2 * m)
            self.deltabar(c, st, W / 2, bottom + 20, W - 2 * m)
            row_x, row_w = m, W - 2 * m
        else:
            bottom = self.sectors(c, st, (W - 900) / 2, m, 900)
            self.deltabar(c, st, W / 2, bottom + 20, 900)
            row_x, row_w = W - m - 1000, 1000
        dash_x = row_x
        if self.show_map:
            self.track_map(c, st, row_x, H - m - th, th, th)
            dash_x += th + 20
        self.dash(c, st, dash_x, H - m - th, row_x + row_w - dash_x, th)

    def render(self, st: HudState) -> np.ndarray:
        """RGBA frame (straight alpha), h x w x 4."""
        surf = skia.Surface(self.w, self.h)
        c = surf.getCanvas()
        c.clear(skia.ColorTRANSPARENT)
        self.draw(c, st)
        return surf.makeImageSnapshot().toarray(colorType=skia.kRGBA_8888_ColorType, alphaType=skia.kUnpremul_AlphaType)


def demo_state() -> HudState:
    return HudState(
        lap=5, lap_time=62.418, delta=-0.19, best=74.888, last=79.054, ideal=74.417,
        sectors=[16.62, 25.97, None], sector_status=["purple", "green", None],
        micro=["purple", "green", "green", "grey", "purple", "green", "grey", "green",
               "green", "purple", "grey", "green", "green", "purple", "green", "grey",
               "green", "grey", None, None, None, None, None, None],
        speed=187, lean=-41, lean_max=(49.7, 51.8), g_lon=-0.62, progress=0.80,
    )


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--demo", nargs=2, metavar=("BG", "OUT"), required=True, help="background frame, output PNG")
    ap.add_argument("--track", default="carole")
    ap.add_argument("--no-map", action="store_true", help="hide the track map")
    args = ap.parse_args()
    import json
    from analyze import Projection, TRACKS_DIR
    tr = json.loads((TRACKS_DIR / f"{args.track}.json").read_text())
    cl = np.array(tr["centerline"])
    proj = Projection(cl[:, 0].mean(), cl[:, 1].mean())
    xs, ys = proj(cl[:, 0], cl[:, 1])
    xy = np.column_stack([xs, ys])
    L = tr["length_m"]
    frac = [d / L for d in tr.get("sectors_m", [])]
    bg = skia.Image.open(args.demo[0])
    hud = Hud(bg.width(), bg.height(), xy, frac, show_map=not args.no_map)
    surf = skia.Surface(bg.width(), bg.height())
    c = surf.getCanvas()
    c.drawImage(bg, 0, 0)
    hud.draw(c, demo_state())
    surf.makeImageSnapshot().save(args.demo[1], skia.kPNG)
    print(f"[demo] {args.demo[1]}")


if __name__ == "__main__":
    main()
