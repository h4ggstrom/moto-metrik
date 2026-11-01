#!/usr/bin/env python3
"""Explore the CAN frames ($PCAN) of a moto-metrik log, to decode the bike's bus.

The bus is undocumented: this lists what is on it and ranks every byte and
16-bit word (both byte orders) by how well it follows what we already know
from GPS/IMU (speed, longitudinal g, lean). A wheel speed shows up as a
near-perfect speed correlation, with its scale factor.

Usage:
    python tools/can_sniff.py LOG.nmea                 # IDs + candidate signals
    python tools/can_sniff.py LOG.nmea --plot 1A0      # bytes of one ID vs speed

Outputs go to out/<log name>/: can_ids.csv, can_candidates.csv, can_<ID>.png.
"""

from __future__ import annotations

import argparse
import csv
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from analyze import dynamics, parse_nmea  # noqa: E402
from imu import clock_fit, read_log  # noqa: E402


def corr(a, b) -> float:
    if np.std(a) == 0 or np.std(b) == 0:
        return 0.0
    return float(np.corrcoef(a, b)[0, 1])


def fields(frames: np.ndarray, dlc: int):
    """(name, values) for each byte and each 16-bit word of an ID's payloads."""
    for i in range(dlc):
        yield f"b{i}", frames[:, i].astype(float)
    for i in range(dlc - 1):
        hi, lo = frames[:, i].astype(float), frames[:, i + 1].astype(float)
        yield f"b{i}{i + 1}be", hi * 256 + lo
        yield f"b{i}{i + 1}le", lo * 256 + hi


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", type=Path)
    ap.add_argument("--plot", action="append", default=[], help="CAN ID (hex) to plot against speed")
    ap.add_argument("--top", type=int, default=12, help="candidates shown per reference signal")
    ap.add_argument("--out", type=Path, default=Path("out"))
    args = ap.parse_args()

    _, pairs, can, _, _ = read_log(args.log)
    if not can:
        print("[can] aucune trame $PCAN dans ce fichier")
        return 1
    if len(pairs) < 50:
        print("[can] pas assez de positions GPS pour dater les trames")
        return 1
    a, b, resid = clock_fit(pairs)
    fixes = parse_nmea(args.log)
    t_gps = np.array([f.t for f in fixes])
    speed = np.array([f.speed_kmh for f in fixes])
    lean, _, a_lon = dynamics(t_gps, speed, np.array([f.course for f in fixes]))
    refs = {
        "vitesse": speed,
        "accel long.": np.nan_to_num(a_lon),
        "|angle|": np.abs(np.nan_to_num(lean)),
    }

    by_id: dict[int, list] = defaultdict(list)
    for ms, cid, data in can:
        by_id[cid].append((a * ms + b, data))
    span = (a * can[-1][0] + b) - (a * can[0][0] + b)
    print(f"[can] {len(can)} trames, {len(by_id)} identifiants, {span / 60:.1f} min, synchro {resid:.0f} ms")

    out_dir = args.out / args.log.stem
    out_dir.mkdir(parents=True, exist_ok=True)
    cands = []
    with open(out_dir / "can_ids.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["id", "frames", "hz", "dlc", "constant_bytes", "changing_bytes"])
        print("  ID         trames    Hz  DLC  octets qui varient")
        for cid in sorted(by_id):
            rows = by_id[cid]
            t = np.array([r[0] for r in rows])
            dlc = max(len(r[1]) for r in rows)
            frames = np.array([list(r[1].ljust(dlc, b"\0")) for r in rows], dtype=np.uint8)
            hz = (len(t) - 1) / (t[-1] - t[0]) if len(t) > 1 and t[-1] > t[0] else 0.0
            changing = [i for i in range(dlc) if len(np.unique(frames[:, i])) > 1]
            constant = [i for i in range(dlc) if i not in changing]
            name = f"{cid:08X}" if cid > 0x7FF else f"{cid:03X}"
            w.writerow([name, len(t), f"{hz:.1f}", dlc, " ".join(map(str, constant)), " ".join(map(str, changing))])
            print(f"  {name:9} {len(t):7} {hz:5.1f}  {dlc}    {' '.join(map(str, changing)) or '-'}")
            inside = (t >= t_gps[0]) & (t <= t_gps[-1])
            if inside.sum() < 50:
                continue
            for fname, vals in fields(frames[inside], dlc):
                if np.std(vals) == 0:
                    continue
                for rname, ref in refs.items():
                    r_at = np.interp(t[inside], t_gps, ref)
                    c = corr(vals, r_at)
                    k, c0 = np.polyfit(vals, r_at, 1)
                    cands.append((rname, abs(c), c, name, fname, k, c0))

    with open(out_dir / "can_candidates.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["reference", "corr", "id", "field", "scale", "offset"])
        for rname in refs:
            best = sorted((c for c in cands if c[0] == rname), key=lambda c: -c[1])
            print(f"\n  candidats « {rname} » (corrélation, échelle : ref = brut x échelle + décalage)")
            for _, _, c, name, fname, k, c0 in best[: args.top]:
                print(f"    {name:9} {fname:7} r={c:+.3f}  x{k:.5g} {c0:+.3g}")
            for _, _, c, name, fname, k, c0 in best:
                w.writerow([rname, f"{c:.4f}", name, fname, f"{k:.6g}", f"{c0:.6g}"])

    for want in args.plot:
        cid = int(want, 16)
        if cid not in by_id:
            print(f"[plot] ID {want} absent")
            continue
        rows = by_id[cid]
        t = np.array([r[0] for r in rows])
        dlc = max(len(r[1]) for r in rows)
        frames = np.array([list(r[1].ljust(dlc, b"\0")) for r in rows], dtype=np.uint8)
        fig, axes = plt.subplots(dlc + 1, 1, figsize=(14, 1.6 * (dlc + 1)), sharex=True)
        axes[0].plot(t_gps - t[0], speed, "k", lw=1)
        axes[0].set_ylabel("km/h")
        for i in range(dlc):
            axes[i + 1].plot(t - t[0], frames[:, i], lw=0.8)
            axes[i + 1].set_ylabel(f"b{i}")
        axes[-1].set_xlabel("s")
        fig.suptitle(f"CAN {want.upper()} : octets vs vitesse GPS")
        fig.tight_layout()
        path = out_dir / f"can_{want.upper()}.png"
        fig.savefig(path, dpi=110)
        plt.close(fig)
        print(f"[plot] {path}")
    print(f"[out] {out_dir}/")
    return 0


if __name__ == "__main__":
    sys.exit(main())
