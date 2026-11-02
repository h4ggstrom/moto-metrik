// moto-metrik session analysis, JavaScript port of the core of tools/analyze.py
// and tools/imu.py (laps, sectors, corners, lean/g from the IMU when logged,
// else from the GPS). Runs in the phone
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

// -> {fixes: [{t, lat, lon, speed, course, sats, hdop}], info: {key: text},
//     imuRaw: [ms, ax, ay, az, gx, gy, gz, ...] flat, pairs: [[ms, utc], ...]}
function parseLog(text) {
  const fixes = [];
  const gga = new Map();
  const info = {};
  const imuRaw = [], pairs = [];
  let pendingMs = null;
  let lastT = -Infinity;
  for (const raw of text.split("\n")) {
    const line = raw.trim();
    if (line.length < 6 || !nmeaOk(line)) continue;
    const f = line.slice(1, line.indexOf("*")).split(",");
    const kind = f[0].slice(2);
    if (f[0] === "PIMU") {
      if (f.length === 9) for (let k = 1; k < 8; k++) imuRaw.push(+f[k]);
    } else if (f[0] === "PTRX") {
      pendingMs = +f[1];
    } else if (f[0] === "PMMK") {
      info[f[1]] = f.slice(2).join(",");
    } else if (kind === "GGA" && f[6] && f[6] !== "0") {
      gga.set(f[1], [parseInt(f[7] || "0"), parseFloat(f[8] || "NaN")]);
    } else if (kind === "RMC") {
      const hh = f[1], dd = f[9];
      const ms = pendingMs;
      pendingMs = null;
      if (!hh || !dd) continue;
      const t = Date.UTC(2000 + +dd.slice(4, 6), +dd.slice(2, 4) - 1, +dd.slice(0, 2),
                         +hh.slice(0, 2), +hh.slice(2, 4), 0) / 1000 + parseFloat(hh.slice(4));
      if (ms !== null) pairs.push([ms, t]);  // clock sync, fix or not
      if (f[2] !== "A" || t <= lastT) continue;
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
  return { fixes, info, imuRaw, pairs };
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
// IMU (port of tools/imu.py): clock sync, mounting calibration, lean

function interpSorted(xs, xp, fp) {  // xs and xp ascending: one linear walk
  const out = new Float64Array(xs.length), n = xp.length;
  let j = 0;
  for (let i = 0; i < xs.length; i++) {
    const x = xs[i];
    if (x <= xp[0]) { out[i] = fp[0]; continue; }
    if (x >= xp[n - 1]) { out[i] = fp[n - 1]; continue; }
    while (xp[j + 1] < x) j++;
    out[i] = fp[j] + (fp[j + 1] - fp[j]) * (x - xp[j]) / (xp[j + 1] - xp[j]);
  }
  return out;
}

function lowpass(a, fs, fc) {  // zero-phase first-order (forward + backward)
  const alpha = 1 - Math.exp(-2 * Math.PI * fc / fs), out = Float64Array.from(a);
  for (let i = 1; i < out.length; i++) out[i] = out[i - 1] + alpha * (out[i] - out[i - 1]);
  for (let i = out.length - 2; i >= 0; i--) out[i] = out[i + 1] + alpha * (out[i] - out[i + 1]);
  return out;
}

function medianOf(vals) {
  const s = Float64Array.from(vals).sort();
  return s.length ? s[s.length >> 1] : 0;
}

// -> {t, acc: [x, y, z], gyro: [x, y, z], syncMs} or null
function loadImu(imuRaw, pairs, info) {
  const n = imuRaw.length / 7;
  if (n < 1000 || pairs.length < 50) return null;
  let accLsb = 4096, gyroLsb = 32.8;
  for (const kv of (info.IMU || "").split(",")) {
    const [k, v] = kv.split("=");
    if (k === "accel_lsb_per_g") accLsb = +v;
    if (k === "gyro_lsb_per_dps") gyroLsb = +v;
  }
  // Reception = epoch + output delay + jitter: robust linear fit ms -> UTC.
  const m0 = pairs[0][0], t0 = pairs[0][1];
  const px = pairs.map(p => p[0] - m0), py = pairs.map(p => p[1] - t0);
  let keep = px.map(() => true), a = 0, b = 0, res = [];
  for (let it = 0; it < 4; it++) {
    let sx = 0, sy = 0, sxx = 0, sxy = 0, k = 0;
    px.forEach((x, i) => { if (keep[i]) { sx += x; sy += py[i]; sxx += x * x; sxy += x * py[i]; k++; } });
    a = (k * sxy - sx * sy) / (k * sxx - sx * sx);
    b = (sy - a * sx) / k;
    res = px.map((x, i) => py[i] - (a * x + b));
    const kept = res.filter((_, i) => keep[i]), med = medianOf(kept);
    const mean = kept.reduce((s, r) => s + r, 0) / kept.length;
    const std = Math.sqrt(kept.reduce((s, r) => s + (r - mean) ** 2, 0) / kept.length);
    keep = res.map(r => Math.abs(r - med) < Math.max(3 * std, 0.005));
  }
  const kept = res.filter((_, i) => keep[i]), med = medianOf(kept);
  const dev = Float64Array.from(kept.map(r => Math.abs(r - med))).sort();
  const syncMs = dev[Math.min(dev.length - 1, Math.floor(0.95 * dev.length))] * 1000;
  const t = new Float64Array(n), acc = [0, 1, 2].map(() => new Float64Array(n)),
        gyro = [0, 1, 2].map(() => new Float64Array(n));
  for (let i = 0; i < n; i++) {
    const o = 7 * i;
    t[i] = t0 + a * (imuRaw[o] - m0) + b;
    for (let c = 0; c < 3; c++) { acc[c][i] = imuRaw[o + 1 + c] / accLsb; gyro[c][i] = imuRaw[o + 4 + c] / gyroLsb; }
  }
  return { t, acc, gyro, syncMs };
}

const dot3 = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const norm3 = a => { const l = Math.hypot(...a); return a.map(x => x / l); };

// Lean (deg, + = right) and lateral g at the GPS fix times, or null when the
// ride does not hold enough stops/straights/braking to calibrate the mounting.
function imuDynamics(imu, tGps, speedKmh, aLonGps, yawGpsDps) {
  const n = imu.t.length;
  const dts = []; for (let i = 1; i < n; i += 97) dts.push(imu.t[i] - imu.t[i - 1]);
  const fs = 1 / medianOf(dts);
  const acc = imu.acc.map(c => lowpass(c, fs, 5.0));  // engine/road vibration out
  const v = interpSorted(imu.t, tGps, speedKmh.map(s => s / 3.6));
  const aGps = interpSorted(imu.t, tGps, aLonGps.map(a => (isFinite(a) ? a : 0)));
  const yawG = interpSorted(imu.t, tGps, yawGpsDps.map(a => (isFinite(a) ? a : 0)));

  // Gyro bias from stops, else from straights.
  let still = Array.from(v, x => x < 0.3);
  let cnt = still.filter(Boolean).length;
  if (cnt < fs * 2) {
    still = Array.from(v, (x, i) => x > 30 / 3.6 && Math.abs(yawG[i]) < 2);
    if (still.filter(Boolean).length < fs * 10) return null;
  }
  const gyro = imu.gyro.map(c => {
    const bias = medianOf(c.filter((_, i) => still[i]));
    return lowpass(c.map(x => x - bias), fs, 8.0);
  });

  const straight = [], long_ = [];
  for (let i = 0; i < n; i++) {
    if (v[i] > 30 / 3.6 && Math.abs(yawG[i]) < 4) straight.push(i);
    if (v[i] > 15 / 3.6 && Math.abs(yawG[i]) < 4 && Math.abs(aGps[i]) > 0.15) long_.push(i);
  }
  if (straight.length < fs * 3 || long_.length < fs) return null;
  const A = i => [acc[0][i], acc[1][i], acc[2][i]];
  const meanOf = (idx, f) => {
    const s = [0, 0, 0];
    for (const i of idx) { const a = f(i); s[0] += a[0]; s[1] += a[1]; s[2] += a[2]; }
    return s;
  };
  let up = norm3(meanOf(straight, A)), fwd = [1, 0, 0];
  for (let it = 0; it < 3; it++) {
    fwd = [0, 0, 0];
    for (const i of long_) {
      const a = A(i), u = dot3(a, up);
      for (let c = 0; c < 3; c++) fwd[c] += (a[c] - u * up[c]) * aGps[i];
    }
    const fu = dot3(fwd, up);
    fwd = norm3(fwd.map((f, c) => f - fu * up[c]));
    up = norm3(meanOf(straight, i => A(i).map((a, c) => a - aGps[i] * fwd[c])));
  }
  const fu = dot3(fwd, up);
  fwd = norm3(fwd.map((f, c) => f - fu * up[c]));
  const left = [up[1] * fwd[2] - up[2] * fwd[1], up[2] * fwd[0] - up[0] * fwd[2], up[0] * fwd[1] - up[1] * fwd[0]];
  const W = i => [gyro[0][i], gyro[1][i], gyro[2][i]];
  const wRoll = new Float64Array(n), wPitch = new Float64Array(n), wYaw = new Float64Array(n);
  for (let i = 0; i < n; i++) { const w = W(i); wRoll[i] = dot3(fwd, w); wPitch[i] = dot3(left, w); wYaw[i] = dot3(up, w); }

  // Complementary filter: roll rate short-term, steady-turn lean long-term.
  const tau = 1.0, D = Math.PI / 180;
  let phi = new Float64Array(n);
  const yaw = new Float64Array(n);
  for (let it = 0; it < 2; it++) {
    const out = new Float64Array(n);
    let cur = 0;
    for (let i = 0; i < n; i++) {
      yaw[i] = wYaw[i] * Math.cos(phi[i] * D) - wPitch[i] * Math.sin(phi[i] * D);
      const ref = v[i] < 15 / 3.6 ? 0 : Math.atan2(v[i] * yaw[i] * D, G) / D;
      const dt = i ? imu.t[i] - imu.t[i - 1] : 0;
      cur += -wRoll[i] * dt;
      cur += (1 - Math.exp(-dt / tau)) * (ref - cur);
      out[i] = cur;
    }
    phi = out;
  }
  let aLat = lowpass(Array.from(v, (x, i) => x * yaw[i] * D / G), fs, 2.0);
  const aLon = lowpass(Array.from(v, (_, i) => dot3(fwd, A(i))), fs, 2.0);

  // Mounting sign ambiguity: the IMU yaw must agree with the GPS one in turns.
  let sx = 0, sy = 0, sxx = 0, syy = 0, sxy = 0, k = 0;
  for (let i = 0; i < n; i++) {
    if (Math.abs(yawG[i]) <= 10) continue;
    const x = yaw[i], y = yawG[i];
    sx += x; sy += y; sxx += x * x; syy += y * y; sxy += x * y; k++;
  }
  const corr = k > 50 ? (k * sxy - sx * sy) / Math.sqrt((k * sxx - sx * sx) * (k * syy - sy * sy)) : NaN;
  if (corr < 0) { phi = phi.map(x => -x); aLat = aLat.map(x => -x); }

  return { lean: interpSorted(tGps, imu.t, phi), aLat: interpSorted(tGps, imu.t, aLat), aLon: interpSorted(tGps, imu.t, aLon),
           syncMs: imu.syncMs, corr: Math.abs(corr) };
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
  const { fixes, info, imuRaw, pairs } = parseLog(text);
  if (fixes.length < 10) return { error: `Seulement ${fixes.length} positions valides`, info };
  const n = fixes.length;
  const t = fixes.map(f => f.t), speed = fixes.map(f => f.speed);
  const meanLat = fixes.reduce((s, f) => s + f.lat, 0) / n, meanLon = fixes.reduce((s, f) => s + f.lon, 0) / n;
  const proj = projection(meanLat, meanLon);
  const x = new Float64Array(n), y = new Float64Array(n);
  fixes.forEach((f, i) => { [x[i], y[i]] = proj(f.lat, f.lon); });
  const dyn = dynamics(t, speed, fixes.map(f => f.course), fixes.map(f => f.sats), fixes.map(f => f.hdop));
  dyn.leanSource = "GPS";
  const imu = loadImu(imuRaw, pairs, info);
  if (imu) {
    const course = fixes.map(f => f.course), okT = [], okC = [];
    course.forEach((c, i) => { if (isFinite(c)) { okT.push(t[i]); okC.push(c); } });
    const heading = new Float64Array(n);
    for (let i = 0, acc = 0, prev = 0; i < n && okT.length > 1; i++) {
      const c = interp(t[i], okT, okC) * Math.PI / 180;
      if (i) { let d = c - prev; d -= 2 * Math.PI * Math.round(d / (2 * Math.PI)); acc += d; } else acc = c;
      prev = c;
      heading[i] = acc;
    }
    const yawGps = gradient(heading, t).map(r => r * 180 / Math.PI);
    const d = imuDynamics(imu, t, speed, dyn.aLon, yawGps);
    if (d) {
      const aLat = median(d.aLat, 5);
      // IMU a_lon only when it tracks the GPS one (logs from before the IMU
      // low-pass fix carry aliased vibration); then it beats GPS speed jumps.
      let sx = 0, sy = 0, sxx = 0, syy = 0, sxy = 0, k = 0;
      for (let i = 0; i < n; i++) {
        if (!(speed[i] > 30) || !isFinite(dyn.aLon[i])) continue;
        const x = dyn.aLon[i], y = d.aLon[i];
        sx += x; sy += y; sxx += x * x; syy += y * y; sxy += x * y; k++;
      }
      const c = k > 100 ? (k * sxy - sx * sy) / Math.sqrt((k * sxx - sx * sx) * (k * syy - sy * sy)) : 0;
      if (c >= 0.95) dyn.aLon = d.aLon;
      for (let i = 0; i < n; i++) {
        const bad = speed[i] < 15 || Math.abs(d.lean[i]) > 62;
        dyn.lean[i] = bad ? NaN : d.lean[i];
        dyn.aLat[i] = bad ? NaN : aLat[i];
      }
      Object.assign(dyn, { leanSource: "IMU", imuSyncMs: d.syncMs, imuCorr: d.corr });
    }
  }

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
