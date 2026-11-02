"""Read a GoPro MP4: video properties and the GPMF telemetry track.

GoPro cameras (HERO5 and later) store their sensors in a 'gpmd' metadata track,
one payload per second in KLV format (key, type, struct size, repeat). We only
need the GPS (GPS5 + GPSU for UTC time) to align a video with a moto-metrik log,
plus ACCL/GYRO when a finer alignment is wanted.

Reference: https://github.com/gopro/gpmf-parser (GPMF format, MIT/Apache).
"""

from __future__ import annotations

import datetime as dt
import struct
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

CONTAINERS = {b"moov", b"trak", b"mdia", b"minf", b"stbl", b"edts", b"dinf", b"udta"}


def _boxes(f, start: int, end: int):
    """(type, payload start, payload end) of the boxes between start and end."""
    pos = start
    while pos + 8 <= end:
        f.seek(pos)
        size, kind = struct.unpack(">I4s", f.read(8))
        hdr = 8
        if size == 1:
            size = struct.unpack(">Q", f.read(8))[0]
            hdr = 16
        elif size == 0:
            size = end - pos
        if size < hdr:
            break
        yield kind, pos + hdr, pos + size
        pos += size


def _find(f, start, end, path):
    for kind, a, b in _boxes(f, start, end):
        if kind == path[0]:
            return (a, b) if len(path) == 1 else _find(f, a, b, path[1:])
    return None


@dataclass
class Track:
    handler: bytes
    fmt: bytes
    timescale: int
    duration: float  # s
    width: int = 0
    height: int = 0
    times: np.ndarray = field(default_factory=lambda: np.zeros(0))  # sample start, s
    durs: np.ndarray = field(default_factory=lambda: np.zeros(0))  # sample duration, s
    offsets: list = field(default_factory=list)
    sizes: list = field(default_factory=list)


def _read_track(f, a, b) -> Track | None:
    mdia = _find(f, a, b, [b"mdia"])
    if not mdia:
        return None
    hdlr = _find(f, *mdia, [b"hdlr"])
    f.seek(hdlr[0] + 8)
    handler = f.read(4)
    mdhd = _find(f, *mdia, [b"mdhd"])
    f.seek(mdhd[0])
    ver = f.read(1)[0]
    if ver == 1:
        f.seek(mdhd[0] + 20)
        timescale, dur = struct.unpack(">IQ", f.read(12))
    else:
        f.seek(mdhd[0] + 12)
        timescale, dur = struct.unpack(">II", f.read(8))
    stbl = _find(f, *mdia, [b"minf", b"stbl"])
    stsd = _find(f, *stbl, [b"stsd"])
    f.seek(stsd[0] + 12)
    fmt = f.read(4)
    t = Track(handler, fmt, timescale, dur / timescale)
    if handler == b"vide":
        f.seek(stsd[0] + 8 + 8 + 24)
        t.width, t.height = struct.unpack(">HH", f.read(4))

    # Sample table: times (stts), sizes (stsz), chunk offsets (stco/co64), samples per chunk (stsc).
    def table(kind, fmt_, n_fields):
        box = _find(f, *stbl, [kind])
        if not box:
            return None
        f.seek(box[0] + 4)
        n = struct.unpack(">I", f.read(4))[0]
        raw = f.read(n * struct.calcsize(">" + fmt_ * n_fields))
        return np.frombuffer(raw, dtype=">" + {"I": "u4", "Q": "u8"}[fmt_]).reshape(n, n_fields).astype(np.int64)

    stts = table(b"stts", "I", 2)
    durs = np.repeat(stts[:, 1], stts[:, 0]) / timescale
    t.durs = durs
    t.times = np.concatenate([[0], np.cumsum(durs)[:-1]])
    stsz = _find(f, *stbl, [b"stsz"])
    f.seek(stsz[0] + 4)
    fixed, n = struct.unpack(">II", f.read(8))
    sizes = [fixed] * n if fixed else list(np.frombuffer(f.read(4 * n), dtype=">u4").astype(np.int64))
    co = table(b"stco", "I", 1)
    if co is None:
        co = table(b"co64", "Q", 1)
    chunk_off = co[:, 0]
    stsc = table(b"stsc", "I", 3)  # first_chunk, samples_per_chunk, desc
    offsets, s = [], 0
    for ci, off in enumerate(chunk_off, start=1):
        per = stsc[np.searchsorted(stsc[:, 0], ci, side="right") - 1, 1]
        for _ in range(per):
            if s >= n:
                break
            offsets.append(int(off))
            off += sizes[s]
            s += 1
    t.offsets, t.sizes = offsets, sizes
    return t


def read_tracks(path: Path) -> list[Track]:
    with open(path, "rb") as f:
        f.seek(0, 2)
        end = f.tell()
        moov = _find(f, 0, end, [b"moov"])
        if not moov:
            raise ValueError(f"{path}: no moov box")
        return [t for kind, a, b in _boxes(f, *moov) if kind == b"trak" and (t := _read_track(f, a, b))]


# ---------------------------------------------------------------------------
# GPMF KLV

_TYPES = {ord("b"): "b", ord("B"): "B", ord("c"): "c", ord("s"): "h", ord("S"): "H", ord("l"): "i",
          ord("L"): "I", ord("f"): "f", ord("d"): "d", ord("j"): "q", ord("J"): "Q", ord("U"): "U"}


def _klv(buf: bytes, pos: int = 0, end: int | None = None):
    end = len(buf) if end is None else end
    while pos + 8 <= end:
        key = buf[pos:pos + 4]
        typ, ssize, rep = buf[pos + 4], buf[pos + 5], struct.unpack(">H", buf[pos + 6:pos + 8])[0]
        n = ssize * rep
        data_end = pos + 8 + n
        yield key, typ, ssize, rep, pos + 8, data_end
        pos = pos + 8 + ((n + 3) & ~3)


def _values(buf, typ, ssize, rep, a):
    t = _TYPES.get(typ)
    if t in (None, "c", "U"):
        return buf[a:a + ssize * rep]
    width = struct.calcsize(">" + t)
    per = ssize // width
    vals = struct.unpack(">" + t * (per * rep), buf[a:a + ssize * rep])
    return np.array(vals, dtype=float).reshape(rep, per)


@dataclass
class Telemetry:
    gps_t: np.ndarray  # video time of each GPS sample, s
    lat: np.ndarray
    lon: np.ndarray
    speed_kmh: np.ndarray  # 2D speed
    gps_fix: np.ndarray  # per sample, 0/2/3
    gps_utc: list  # (video time of payload start, UTC epoch from GPSU)
    acc_t: np.ndarray  # video time of each accelerometer sample, s
    acc: np.ndarray  # (n, 3) m/s^2, camera frame


def read_telemetry(path: Path) -> Telemetry:
    tracks = read_tracks(path)
    meta = next((t for t in tracks if t.fmt == b"gpmd"), None)
    if meta is None:
        raise ValueError(f"{path}: no GPMF track")
    gps_t, gps, fix, utc, acc_t, acc = [], [], [], [], [], []
    with open(path, "rb") as f:
        for t0, dur, off, size in zip(meta.times, meta.durs, meta.offsets, meta.sizes):
            f.seek(off)
            buf = f.read(size)
            for key, *_r, a, b in _klv(buf):
                if key != b"DEVC":
                    continue
                for skey, *_s, sa, sb in _klv(buf, a, b):
                    if skey != b"STRM":
                        continue
                    scal, gpsf, rows, kind, gpsu = None, 0, None, None, None
                    for k, typ, ssize, rep, da, db in _klv(buf, sa, sb):
                        if k == b"SCAL":
                            scal = _values(buf, typ, ssize, rep, da).ravel()
                        elif k == b"GPSF":
                            gpsf = int(_values(buf, typ, ssize, rep, da).ravel()[0])
                        elif k == b"GPSU":
                            gpsu = _values(buf, typ, ssize, rep, da).decode("ascii", "replace")
                        elif k in (b"GPS5", b"ACCL"):
                            rows, kind = _values(buf, typ, ssize, rep, da), k
                    if rows is None or scal is None:
                        continue
                    vals = rows / (scal if len(scal) > 1 else scal[0])
                    ts = t0 + dur * np.arange(len(vals)) / len(vals)
                    if kind == b"GPS5":
                        gps_t.append(ts)
                        gps.append(vals)
                        fix.append(np.full(len(vals), gpsf))
                        if gpsu:
                            d = dt.datetime.strptime(gpsu[:16], "%y%m%d%H%M%S.%f").replace(tzinfo=dt.timezone.utc)
                            utc.append((t0, d.timestamp()))
                    else:
                        acc_t.append(ts)
                        acc.append(vals)
    g = np.concatenate(gps) if gps else np.zeros((0, 5))
    return Telemetry(np.concatenate(gps_t) if gps_t else np.zeros(0), g[:, 0], g[:, 1], g[:, 3] * 3.6,
                     np.concatenate(fix) if fix else np.zeros(0), utc,
                     np.concatenate(acc_t) if acc_t else np.zeros(0), np.concatenate(acc) if acc else np.zeros((0, 3)))


def video_info(path: Path):
    """(width, height, fps, duration s) of the video track."""
    v = next(t for t in read_tracks(path) if t.handler == b"vide")
    return v.width, v.height, 1 / np.median(v.durs), v.duration
