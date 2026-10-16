// moto-metrik session analysis, JavaScript port of the core of tools/analyze.py
// (GPS part: laps, sectors, corners, GPS-derived lean/g). Runs in the phone
// browser; also loadable from node for testing against the Python version.

"use strict";

const EARTH_R = 6371000;
const KNOT_KMH = 1.852;
const G = 9.81;

function nmeaOk(line) {
  if (line[0] !== "$") return false;
  const star = line.indexOf("*");
  if (star < 0) return false;
  let cs = 0;
  for (let i = 1; i < star; i++) cs ^= line.charCodeAt(i);
  return cs === parseInt(line.substr(star + 1, 2), 16);
}

function coord(value, hemi) {
  if (!value) return NaN;
  const dot = value.indexOf(".");
  const v = parseFloat(value.slice(0, dot - 2)) + parseFloat(value.slice(dot - 2)) / 60;
  return hemi === "S" || hemi === "W" ? -v : v;
}

// -> {fixes: [{t, lat, lon, speed, course, sats, hdop}], info: {key: text}}
function parseLog(text) {
  const fixes = [];
  const gga = new Map();
  const info = {};
  let lastT = -Infinity;
  for (const raw of text.split("\n")) {
    const line = raw.trim();
    if (line.length < 6 || !nmeaOk(line)) continue;
    const f = line.slice(1, line.indexOf("*")).split(",");
    const kind = f[0].slice(2);
    if (f[0] === "PMMK") {
      info[f[1]] = f.slice(2).join(",");
    } else if (kind === "GGA" && f[6] && f[6] !== "0") {
      gga.set(f[1], [parseInt(f[7] || "0"), parseFloat(f[8] || "NaN")]);
    } else if (kind === "RMC" && f[2] === "A" && f[9]) {
      const hh = f[1], dd = f[9];
      const t = Date.UTC(2000 + +dd.slice(4, 6), +dd.slice(2, 4) - 1, +dd.slice(0, 2),
                         +hh.slice(0, 2), +hh.slice(2, 4), 0) / 1000 + parseFloat(hh.slice(4));
      if (t <= lastT) continue;
      lastT = t;
      fixes.push({ t, key: hh, lat: coord(f[3], f[4]), lon: coord(f[5], f[6]),
                   speed: parseFloat(f[7] || "0") * KNOT_KMH, course: parseFloat(f[8] || "NaN"),
                   sats: 0, hdop: NaN });
    }
  }
  for (const fx of fixes) {  // GGA can come after its RMC: merge at the end
    const g = gga.get(fx.key);
    if (g) [fx.sats, fx.hdop] = g;
  }
  return { fixes, info };
}

function projection(lat0, lon0) {
  const kx = (Math.PI / 180) * EARTH_R * Math.cos(lat0 * Math.PI / 180);
  const ky = (Math.PI / 180) * EARTH_R;
  return (lat, lon) => [(lon - lon0) * kx, (lat - lat0) * ky];
}

// ---------------------------------------------------------------------------
// signal helpers

function smooth(a, n) {
  const k = n >> 1, out = new Float64Array(a.length);
  for (let i = 0; i < a.length; i++) {
    let s = 0;
    for (let j = -k; j <= k; j++) s += a[Math.min(a.length - 1, Math.max(0, i + j))];
    out[i] = s / n;
  }
  return out;
}

function median(a, n) {
  const k = n >> 1, out = new Float64Array(a.length), w = new Array(n);
  for (let i = 0; i < a.length; i++) {
    for (let j = -k; j <= k; j++) w[j + k] = a[Math.min(a.length - 1, Math.max(0, i + j))];
    w.sort((x, y) => x - y);
    out[i] = w[k];
  }
  return out;
}

function gradient(y, x) {
  const n = y.length, g = new Float64Array(n);
  for (let i = 0; i < n; i++) {
    const a = Math.max(0, i - 1), b = Math.min(n - 1, i + 1);
    g[i] = (y[b] - y[a]) / (x[b] - x[a]);
  }
  return g;
}

function interp(x, xp, fp) {  // xp ascending
  let lo = 0, hi = xp.length - 1;
  if (x <= xp[0]) return fp[0];
  if (x >= xp[hi]) return fp[hi];
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (xp[mid] <= x) lo = mid; else hi = mid;
  }
  return fp[lo] + (fp[hi] - fp[lo]) * (x - xp[lo]) / (xp[hi] - xp[lo]);
}

// GPS-derived lean (deg, + = right), lateral and longitudinal g.
function dynamics(t, speed, course, sats, hdop) {
  const n = t.length, w = 5;
  const okT = [], okC = [];
  for (let i = 0; i < n; i++) if (isFinite(course[i])) { okT.push(t[i]); okC.push(course[i]); }
  const lean = new Float64Array(n).fill(NaN), aLat = new Float64Array(n).fill(NaN);
  const v = speed.map(s => s / 3.6);
  const aLon = smooth(gradient(smooth(v, w), t), w).map(a => a / G);
  if (okT.length < 2) return { lean, aLat, aLon };
  const heading = new Float64Array(n);
  let prev = 0, acc = 0;
  for (let i = 0; i < n; i++) {
    const c = interp(t[i], okT, okC) * Math.PI / 180;
    if (i) { let d = c - prev; d -= 2 * Math.PI * Math.round(d / (2 * Math.PI)); acc += d; } else acc = c;
    prev = c;
    heading[i] = acc;
  }
  const yaw = median(gradient(smooth(heading, w), t), w);
  const lat = smooth(v.map((vv, i) => vv * yaw[i]), w);
  for (let i = 0; i < n; i++) {
    lean[i] = Math.atan2(lat[i], G) * 180 / Math.PI;
    aLat[i] = lat[i] / G;
  }
  // poor signal / slow / gaps / implausible -> NaN (same rules as analyze.py)
  const bad = new Uint8Array(n);
  for (let i = 0; i < n; i++) {
    if (sats[i] < 5 || hdop[i] > 2 || (i && t[i] - t[i - 1] > 0.3)) bad[i] = 1;
  }
  for (let i = 0; i < n; i++) {
    let b = speed[i] < 15 || Math.abs(lean[i]) > 62;
    for (let j = Math.max(0, i - 5); j <= Math.min(n - 1, i + 5) && !b; j++) b = bad[j] === 1;
    if (b) { lean[i] = NaN; aLat[i] = NaN; }
    if (bad[i]) aLon[i] = NaN;
  }
  return { lean, aLat, aLon };
}

// ---------------------------------------------------------------------------
// track geometry

function gateOnCenterline(name, cx, cy, dist, width) {
  const n = cx.length;
  let cum = 0;
  const total = (() => {
    let s = 0;
    for (let i = 0; i < n; i++) s += Math.hypot(cx[(i + 1) % n] - cx[i], cy[(i + 1) % n] - cy[i]);
    return s;
  })();
  const d = ((dist % total) + total) % total;
  for (let i = 0; i < n; i++) {
    const x2 = cx[(i + 1) % n], y2 = cy[(i + 1) % n];
    const seg = Math.hypot(x2 - cx[i], y2 - cy[i]);
    if (cum + seg >= d) {
      const u = (d - cum) / seg;
      return { name, x: cx[i] + u * (x2 - cx[i]), y: cy[i] + u * (y2 - cy[i]),
               hx: (x2 - cx[i]) / seg, hy: (y2 - cy[i]) / seg, width };
    }
    cum += seg;
  }
}

function gateEnds(g) {
  const h = g.width / 2;
  return [[g.x - g.hy * h, g.y + g.hx * h], [g.x + g.hy * h, g.y - g.hx * h]];
}

function segIntersect(p1, p2, q1, q2) {
  const d = (p2[0] - p1[0]) * (q2[1] - q1[1]) - (p2[1] - p1[1]) * (q2[0] - q1[0]);
  if (Math.abs(d) < 1e-12) return null;
  const u = ((q1[0] - p1[0]) * (q2[1] - q1[1]) - (q1[1] - p1[1]) * (q2[0] - q1[0])) / d;
  const v = ((q1[0] - p1[0]) * (p2[1] - p1[1]) - (q1[1] - p1[1]) * (p2[0] - p1[0])) / d;
  return u >= 0 && u <= 1 && v >= 0 && v <= 1 ? u : null;
}

function gateCrossings(x, y, t, g, minInterval = 0) {
  const [q1, q2] = gateEnds(g), out = [];
  for (let i = 0; i < x.length - 1; i++) {
    const p1 = [x[i], y[i]], p2 = [x[i + 1], y[i + 1]];
    const u = segIntersect(p1, p2, q1, q2);
    if (u === null) continue;
    if ((p2[0] - p1[0]) * g.hx + (p2[1] - p1[1]) * g.hy <= 0) continue;
    const tc = t[i] + u * (t[i + 1] - t[i]);
    if (out.length && tc - out[out.length - 1] < minInterval) continue;
    out.push(tc);
  }
  return out;
}

function trackProgress(x, y, cx, cy, offset) {
  const n = cx.length, cum = [0];
  for (let i = 0; i < n; i++) cum.push(cum[i] + Math.hypot(cx[(i + 1) % n] - cx[i], cy[(i + 1) % n] - cy[i]));
  const L = cum[n], s = new Float64Array(x.length);
  for (let k = 0; k < x.length; k++) {
    let best = Infinity, bs = 0;
    for (let i = 0; i < n; i++) {
      const ax = cx[i], ay = cy[i], dx = cx[(i + 1) % n] - ax, dy = cy[(i + 1) % n] - ay;
      const seg2 = dx * dx + dy * dy;
      const u = Math.min(1, Math.max(0, ((x[k] - ax) * dx + (y[k] - ay) * dy) / seg2));
      const ex = ax + u * dx - x[k], ey = ay + u * dy - y[k], d2 = ex * ex + ey * ey;
      if (d2 < best) { best = d2; bs = cum[i] + u * Math.sqrt(seg2); }
    }
    s[k] = (((bs - offset) % L) + L) % L;
  }
  return { s, length: L };
}

function detectTrack(fixes, tracks, maxDist = 1000) {
  const lats = fixes.map(f => f.lat).sort((a, b) => a - b), lons = fixes.map(f => f.lon).sort((a, b) => a - b);
  const mlat = lats[lats.length >> 1], mlon = lons[lons.length >> 1];
  const proj = projection(mlat, mlon);
  let best = null, bestD = maxDist;
  for (const tr of tracks) {
    for (const [la, lo] of tr.centerline) {
      const [px, py] = proj(la, lo), d = Math.hypot(px, py);
      if (d < bestD) { bestD = d; best = tr; }
    }
  }
  return best;
}

// ---------------------------------------------------------------------------
// session analysis

function analyze(text, tracks) {
  const { fixes, info } = parseLog(text);
  if (fixes.length < 10) return { error: `Seulement ${fixes.length} positions valides`, info };
  const n = fixes.length;
  const t = fixes.map(f => f.t), speed = fixes.map(f => f.speed);
  const meanLat = fixes.reduce((s, f) => s + f.lat, 0) / n, meanLon = fixes.reduce((s, f) => s + f.lon, 0) / n;
  const proj = projection(meanLat, meanLon);
  const x = new Float64Array(n), y = new Float64Array(n);
  fixes.forEach((f, i) => { [x[i], y[i]] = proj(f.lat, f.lon); });
  const dyn = dynamics(t, speed, fixes.map(f => f.course), fixes.map(f => f.sats), fixes.map(f => f.hdop));

  const dts = t.slice(1).map((v, i) => v - t[i]).sort((a, b) => a - b);
  const period = dts[dts.length >> 1];
  let skipped = 0;
  for (let i = 1; i < n; i++) { const d = t[i] - t[i - 1]; if (d > 1.5 * period) skipped += Math.round(d / period) - 1; }

  const res = {
    info, n, t, x, y, speed, ...dyn,
    duration: t[n - 1] - t[0], start: t[0], rateHz: 1 / period, skipped,
    skippedPct: 100 * skipped / (n + skipped), vmax: Math.max(...speed),
    track: null, gates: [], outline: null, laps: [], partial: [], corners: [], ideal: null, best: null,
  };

  const track = detectTrack(fixes, tracks);
  if (!track) return res;
  res.track = track;
  const cx = track.centerline.map(p => proj(p[0], p[1])[0]), cy = track.centerline.map(p => proj(p[0], p[1])[1]);
  res.outline = [cx, cy];
  const width = track.gate_width_m || 40;
  const gates = [gateOnCenterline("S/F", cx, cy, track.finish_m, width)];
  (track.sectors_m || []).forEach((d, k) => gates.push(gateOnCenterline(`S${k + 1}|S${k + 2}`, cx, cy, track.finish_m + d, width)));
  res.gates = gates;

  const sf = gateCrossings(x, y, t, gates[0], 20);
  const secCross = gates.slice(1).map(g => gateCrossings(x, y, t, g));
  for (let k = 0; k + 1 < sf.length; k++) {
    const a = sf[k], b = sf[k + 1];
    const marks = [a];
    for (const sc of secCross) {
      const inside = sc.filter(c => c > marks[marks.length - 1] && c < b);
      marks.push(inside.length ? inside[0] : null);
    }
    marks.push(b);
    const sectors = marks.includes(null) ? marks.slice(1).map(() => null) : marks.slice(1).map((m, i) => m - marks[i]);
    const i0 = t.findIndex(v => v >= a), i1 = t.findIndex(v => v >= b) - 1;
    let vmax = 0;
    for (let i = i0; i <= i1; i++) vmax = Math.max(vmax, speed[i]);
    res.laps.push({ number: k + 1, tStart: a, tEnd: b, time: b - a, sectors, i0, i1, vmax });
  }
  if (res.laps.length) {
    res.best = res.laps.reduce((m, l) => (l.time < m.time ? l : m));
    const nSec = res.laps[0].sectors.length;
    const bestSec = [];
    for (let k = 0; k < nSec; k++) {
      const vals = res.laps.map(l => l.sectors[k]).filter(v => v !== null);
      bestSec.push(vals.length ? Math.min(...vals) : null);
    }
    res.bestSectors = bestSec;
    if (nSec && !bestSec.includes(null)) res.ideal = bestSec.reduce((s, v) => s + v, 0);
  }

  // sectors of incomplete laps
  const events = sf.map(c => [c, 0]);
  secCross.forEach((sc, k) => sc.forEach(c => events.push([c, k + 1])));
  events.sort((a, b) => a[0] - b[0]);
  const partial = new Map();
  let lapNo = 0;
  for (let i = 0; i + 1 < events.length; i++) {
    const [ta, ga] = events[i], [tb, gb] = events[i + 1];
    if (ga === 0) lapNo++;
    if (gb === (ga + 1) % gates.length && tb - ta < 300) {
      if (!partial.has(lapNo)) partial.set(lapNo, new Array(gates.length).fill(null));
      partial.get(lapNo)[ga] = tb - ta;
    }
  }
  const complete = new Set(res.laps.map(l => l.number));
  for (const [k, secs] of [...partial.entries()].sort((a, b) => a[0] - b[0])) {
    if (!complete.has(k)) res.partial.push({ lap: k, sectors: secs });
  }

  // corners
  if (track.corners) {
    const { s, length } = trackProgress(x, y, cx, cy, track.finish_m);
    track.corners.forEach((c, ci) => {
      const a = c.start_m, b = c.end_m;
      const prevEnd = track.corners[(ci - 1 + track.corners.length) % track.corners.length].end_m;
      const window = Math.min(250, (((a - prevEnd) % length) + length) % length);
      for (let i = 0; i + 1 < n; i++) {
        const ds = s[i + 1] - s[i];
        if (!(ds > 0 && ds < 60 && s[i] < a && s[i + 1] >= a)) continue;
        let j = -1;
        for (let m = i; m + 1 < n && t[m] - t[i] < 60; m++) {
          const d2 = s[m + 1] - s[m];
          if (d2 > 0 && d2 < 60 && s[m] < b && s[m + 1] >= b) { j = m; break; }
        }
        if (j < 0) continue;
        const tIn = t[i] + (a - s[i]) / (s[i + 1] - s[i]) * (t[i + 1] - t[i]);
        const tOut = t[j] + (b - s[j]) / (s[j + 1] - s[j]) * (t[j + 1] - t[j]);
        let k = i + 1;
        for (let m = i + 1; m <= j; m++) if (speed[m] < speed[k]) k = m;
        let m0 = i;
        while (m0 > 0 && a - s[m0 - 1] >= 0 && a - s[m0 - 1] < window && t[i] - t[m0 - 1] < 20) m0--;
        let mPeak = m0;
        for (let m = m0; m <= k; m++) if (speed[m] > speed[mPeak]) mPeak = m;
        let brakeAt = null, brakeG = 0;
        if (speed[mPeak] - speed[k] > 10) {
          brakeAt = a - s[mPeak];
          for (let m = mPeak; m <= k; m++) if (isFinite(dyn.aLon[m])) brakeG = Math.max(brakeG, -dyn.aLon[m]);
        }
        let leanMax = NaN;
        for (let m = i + 1; m <= j; m++) if (isFinite(dyn.lean[m])) leanMax = isFinite(leanMax) ? Math.max(leanMax, Math.abs(dyn.lean[m])) : Math.abs(dyn.lean[m]);
        let lap = 0;
        while (lap < sf.length && sf[lap] < tIn) lap++;
        res.corners.push({ corner: c.name, lap, tIn, time: tOut - tIn, vIn: interp(tIn, t, speed),
                           vMin: speed[k], vMinAt: s[k] - a, vOut: interp(tOut, t, speed), leanMax, brakeAt, brakeG });
      }
    });
    res.corners.sort((p, q) => p.tIn - q.tIn);
  }
  return res;
}

function fmtLap(sec) {
  if (sec === null || !isFinite(sec)) return "--";
  const m = Math.floor(sec / 60), s = sec - 60 * m;
  return `${m}:${s.toFixed(3).padStart(6, "0")}`;
}

if (typeof module !== "undefined") module.exports = { parseLog, analyze, fmtLap, nmeaOk };
