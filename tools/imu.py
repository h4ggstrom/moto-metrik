"""IMU processing for moto-metrik logs ($PIMU samples, $PTRX timestamps).

The IMU can be mounted in any orientation, as long as it is rigidly fixed to
the bike. Everything is derived from the ride itself:

1. clock sync: $PTRX gives the ESP32 millis() at which each RMC arrived;
   a linear fit maps IMU timestamps to GPS (UTC) time.
2. gyro bias: median gyro while stopped (GPS speed < 1 km/h).
3. mounting: "up" = mean specific force on steady straights (bike upright on
   average), "forward" = direction of the extra acceleration when GPS says
   the bike brakes or accelerates in a straight line.
4. lean: gyro roll rate integrated, pulled towards the steady-turn lean
   atan(v * yaw_rate / g), with yaw rate from the gyro (100 Hz, far smoother
   than the GPS course). Longitudinal g from the accelerometer, lateral g
   from v * yaw_rate.
"""

from __future__ import annotations

import datetime as dt
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

G = 9.81


@dataclass
class ImuData:
    t: np.ndarray  # UTC epoch seconds (after sync), same base as the GPS fixes
    acc: np.ndarray  # (n, 3) g, sensor frame
    gyro: np.ndarray  # (n, 3) deg/s, sensor frame
    sync_residual_ms: float  # p95 of |fit residual| of the clock sync


def nmea_ok(line: str) -> bool:
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


def load_imu(path: Path) -> ImuData | None:
    acc_lsb, gyro_lsb = 4096.0, 32.8
    raw, pairs = [], []
    pending_ms = None
    with open(path, "r", encoding="ascii", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not nmea_ok(line):
                continue
            body = line[1:].split("*")[0]
            fields = body.split(",")
            if fields[0] == "PIMU" and len(fields) == 9:
                raw.append([int(v) for v in fields[1:]])
            elif fields[0] == "PTRX":
                pending_ms = int(fields[1])
            elif fields[0].endswith("RMC"):
                if pending_ms is not None and fields[1] and len(fields) > 9 and fields[9]:
                    hh, dd = fields[1], fields[9]
                    day = dt.datetime.strptime(dd, "%d%m%y").replace(tzinfo=dt.timezone.utc).timestamp()
                    pairs.append((pending_ms, day + int(hh[:2]) * 3600 + int(hh[2:4]) * 60 + float(hh[4:])))
                pending_ms = None
            elif fields[0] == "PMMK" and len(fields) > 2 and fields[1] == "IMU":
                kv = dict(x.split("=") for x in fields[3:] if "=" in x)
                acc_lsb = float(kv.get("accel_lsb_per_g", acc_lsb))
                gyro_lsb = float(kv.get("gyro_lsb_per_dps", gyro_lsb))
    if len(raw) < 1000 or len(pairs) < 50:
        return None
    raw = np.array(raw, dtype=float)
    p = np.array(pairs, dtype=float)
    # Reception = epoch + output delay + small jitter: robust linear fit.
    keep = np.ones(len(p), bool)
    for _ in range(4):
        a, b = np.polyfit(p[keep, 0], p[keep, 1], 1)
        res = p[:, 1] - (a * p[:, 0] + b)
        keep = np.abs(res - np.median(res[keep])) < max(3 * np.std(res[keep]), 0.005)
    resid = np.percentile(np.abs(res[keep] - np.median(res[keep])), 95) * 1000
    t = a * raw[:, 0] + b
    return ImuData(t, raw[:, 1:4] / acc_lsb, raw[:, 4:7] / gyro_lsb, float(resid))


def lowpass(a, fs: float, fc: float):
    """Zero-phase first-order low-pass (forward + backward), per column."""
    alpha = 1 - math.exp(-2 * math.pi * fc / fs)
    out = np.array(a, dtype=float)
    for rng in (range(1, len(out)), range(len(out) - 2, -1, -1)):
        for i in rng:
            j = i - 1 if rng.step == 1 else i + 1
            out[i] = out[j] + alpha * (out[i] - out[j])
    return out


@dataclass
class ImuDynamics:
    lean: np.ndarray  # deg, + = right, at GPS fix times
    a_lon: np.ndarray  # g, + = accelerating
    a_lat: np.ndarray  # g, + = right turn
    info: str


def imu_dynamics(imu: ImuData, t_gps, speed_kmh, a_lon_gps, yaw_gps_dps) -> ImuDynamics | None:
    fs = 1 / np.median(np.diff(imu.t))
    acc = lowpass(imu.acc, fs, 5.0)  # engine/road vibration out
    gyro = imu.gyro.copy()
    v = np.interp(imu.t, t_gps, speed_kmh) / 3.6
    a_gps = np.interp(imu.t, t_gps, np.nan_to_num(a_lon_gps))
    yaw_g = np.interp(imu.t, t_gps, np.nan_to_num(yaw_gps_dps))

    still = v < 0.3
    if still.sum() < fs * 2:
        return None
    gyro -= np.median(gyro[still], axis=0)
    gyro = lowpass(gyro, fs, 8.0)

    # Straight line = little yaw. The bike is upright there on average, but it
    # may be accelerating: estimate "up", then "forward", then refine "up" by
    # removing the forward acceleration.
    straight = (v > 30 / 3.6) & (np.abs(yaw_g) < 4)
    if straight.sum() < fs * 3:
        return None
    long_ = (v > 15 / 3.6) & (np.abs(yaw_g) < 4) & (np.abs(a_gps) > 0.15)
    if long_.sum() < fs * 1:
        return None
    up = acc[straight].mean(axis=0)
    up /= np.linalg.norm(up)
    for _ in range(3):
        horiz = acc[long_] - np.outer(acc[long_] @ up, up)
        fwd = (horiz * a_gps[long_, None]).sum(axis=0)
        fwd -= (fwd @ up) * up
        fwd /= np.linalg.norm(fwd)
        up = (acc[straight] - np.outer(a_gps[straight], fwd)).mean(axis=0)
        up /= np.linalg.norm(up)
    fwd -= (fwd @ up) * up
    fwd /= np.linalg.norm(fwd)
    left = np.cross(up, fwd)
    R = np.vstack([fwd, left, up])  # sensor -> bike (x fwd, y left, z up)

    ab = acc @ R.T
    wb = gyro @ R.T  # deg/s about fwd (roll), left (pitch), up (yaw)

    # Yaw rate about the true vertical: with lean phi the turn rate splits
    # between the bike's z and y axes. Iterate the lean estimate twice.
    phi = np.zeros(len(v))
    dt = np.diff(imu.t, prepend=imu.t[0])
    tau = 1.0  # s: gyro short-term, steady-turn model long-term
    k = 1 - np.exp(-dt / tau)
    for _ in range(2):
        yaw = wb[:, 2] * np.cos(np.radians(phi)) - wb[:, 1] * np.sin(np.radians(phi))
        ref = np.degrees(np.arctan2(v * np.radians(yaw), G))
        ref[v < 15 / 3.6] = 0.0
        roll_rate = -wb[:, 0]  # + = rolling to the right
        out = np.empty(len(v))
        cur = 0.0
        for i in range(len(v)):
            cur = cur + roll_rate[i] * dt[i]
            cur += k[i] * (ref[i] - cur)
            out[i] = cur
        phi = out

    # g values smoothed to 2 Hz: what the rider feels, not the chassis buzz.
    a_lon = lowpass(ab[:, 0], fs, 2.0)  # flat track: pitch is small
    a_lat = lowpass(v * np.radians(yaw) / G, fs, 2.0)

    # Sanity: the IMU lean must agree in sign with the GPS one in turns.
    yaw_ok = np.abs(yaw_g) > 10
    corr = np.corrcoef(np.radians(yaw[yaw_ok]), np.radians(yaw_g[yaw_ok]))[0, 1] if yaw_ok.sum() > 50 else float("nan")
    if np.isfinite(corr) and corr < 0:  # mounting sign ambiguity: flip left/right
        phi, a_lat = -phi, -a_lat

    on_gps = lambda a: np.interp(t_gps, imu.t, a)  # noqa: E731
    info = (f"sync p95 {imu.sync_residual_ms:.0f} ms, gyro yaw vs GPS yaw corr {abs(corr):.2f}, "
            f"up={np.round(up, 2)}, fwd={np.round(fwd, 2)}")
    return ImuDynamics(on_gps(phi), on_gps(a_lon), on_gps(a_lat), info)
