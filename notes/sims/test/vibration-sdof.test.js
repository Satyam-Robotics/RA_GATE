// Model tests for vibration-sdof. Run: node notes/sims/test/vibration-sdof.test.js
'use strict';
const assert = require('assert');
const RASim = require('./load.js')('vibration-sdof');
const ID = 'vibration-sdof';
const m = RASim.sims[ID].model;
const M = RASim.math;
const PI = Math.PI;

let failures = 0;
function group(name, fn) {
  try { const extra = fn(); console.log('ok   ' + name + (extra ? '  (' + extra + ')' : '')); }
  catch (e) { failures++; console.log('FAIL ' + name + ': ' + (e && e.message || e)); }
}
// a value printed in the specification: |actual − expected| ≤ max(half a unit of the last printed digit, 5e-4·|expected|)
function printed(actual, expected, dec, what) {
  const tol = Math.max(0.5 * Math.pow(10, -dec), 5e-4 * Math.abs(expected)) + 1e-12;
  assert.ok(Number.isFinite(actual) && Math.abs(actual - expected) <= tol, what + ': ' + actual + ' vs ' + expected + ' (tol ' + tol + ')');
}
function near(actual, expected, tol, what) {
  assert.ok(Number.isFinite(actual) && Math.abs(actual - expected) <= tol, what + ': ' + actual + ' vs ' + expected + ' (tol ' + tol + ')');
}
function rel(actual, expected, tol, what) {
  const d = Math.abs(actual - expected) / Math.max(Math.abs(expected), 1e-300);
  assert.ok(Number.isFinite(actual) && d <= tol, what + ': ' + actual + ' vs ' + expected + ' (rel ' + d + ' > ' + tol + ')');
}
function isNaNField(v, what) { assert.ok(Number.isNaN(v), what + ' should be NaN, got ' + v); }
// deterministic PRNG (mulberry32)
function rng(seed) {
  let a = seed >>> 0;
  return function () {
    a = (a + 0x6D2B79F5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}
function uni(R, a, b) { return a + (b - a) * R(); }

group('API shape and title', function () {
  assert.strictEqual(RASim.sims[ID].title, 'Single-DOF vibration: free, forced and isolation');
  ['natural', 'damping', 'free', 'freeVel', 'peaks', 'envelope', 'MF', 'phase', 'TR', 'unbalance', 'peakMF', 'freeTEnd'].forEach(function (k) {
    assert.strictEqual(typeof m[k], 'function', 'model.' + k);
  });
  const n = m.natural(1, 100);
  ['wn', 'fn', 'Tn', 'cc', 'dst'].forEach(k => assert.ok(k in n, 'natural.' + k));
  const d = m.damping(1, 100, 0.1);
  ['c', 'kind', 'wd', 'Td', 'delta', 'ratio', 'halfCycles'].forEach(k => assert.ok(k in d, 'damping.' + k));
  const p = m.peaks(1, 100, 0.1, 0.05, 0, 2);
  assert.ok(Array.isArray(p) && p.length > 0 && 't' in p[0] && 'x' in p[0], 'peaks shape');
  const pk = m.peakMF(0.1);
  assert.ok(pk && 'r' in pk && 'MF' in pk, 'peakMF shape');
});

group('check: m = 1, k = 100 (natural)', function () {
  const n = m.natural(1, 100);
  printed(n.wn, 10, 0, 'wn');
  printed(n.fn, 1.59155, 5, 'fn');
  printed(n.Tn, 2 * PI / 10, 6, 'Tn');
  printed(n.cc, 20, 0, 'cc');
  printed(1000 * n.dst, 98.1, 1, 'dst mm');
  near(Math.sqrt(9.81 / n.dst), n.wn, 1e-12, 'wn = sqrt(g/dst)');
  near(2 * 1 * n.wn, n.cc, 1e-12, 'cc = 2 m wn');
});

group('check: ζ = 0.1 (damping)', function () {
  const d = m.damping(1, 100, 0.1);
  assert.strictEqual(d.kind, 'under');
  printed(d.c, 2, 0, 'c');
  printed(d.wd, 9.94987, 5, 'wd');
  printed(d.Td, 0.631484, 6, 'Td');
  printed(d.delta, 0.631484, 6, 'delta');
  printed(d.ratio, 1.88040, 5, 'ratio');
  printed(d.halfCycles, 1.0976, 4, 'halfCycles');
});

group('damping: kinds and NaN where not applicable', function () {
  assert.strictEqual(m.damping(1, 100, 0).kind, 'undamped');
  assert.strictEqual(m.damping(1, 100, 0.5).kind, 'under');
  assert.strictEqual(m.damping(1, 100, 1).kind, 'critical');
  assert.strictEqual(m.damping(1, 100, 1 + 5e-7).kind, 'critical');
  assert.strictEqual(m.damping(1, 100, 1 - 5e-7).kind, 'critical');
  assert.strictEqual(m.damping(1, 100, 1 + 2e-6).kind, 'over');
  assert.strictEqual(m.damping(1, 100, 1 - 2e-6).kind, 'under');
  assert.strictEqual(m.damping(1, 100, 2).kind, 'over');
  ['critical', 'over'].forEach(function (k, i) {
    const d = m.damping(1, 100, [1, 2][i]);
    ['wd', 'Td', 'delta', 'ratio', 'halfCycles'].forEach(f => isNaNField(d[f], k + '.' + f));
  });
  const u = m.damping(1, 100, 0);
  near(u.wd, 10, 1e-12, 'undamped wd = wn'); near(u.delta, 0, 0, 'undamped delta 0'); near(u.ratio, 1, 0, 'undamped ratio 1');
  assert.strictEqual(u.halfCycles, Infinity, 'undamped never halves');
  printed(m.damping(2, 800, 0.5).c, 0.5 * 2 * Math.sqrt(1600), 9, 'c = ζ c_c');
});

group('check: free() values', function () {
  printed(m.free(1, 100, 0.1, 0.05, 0, 0.631484), 0.026590, 6, 'under, t = T_d');
  printed(m.free(1, 100, 1, 0.05, 0, 0.1), 0.036788, 6, 'critical');
  printed(m.free(1, 100, 2, 0.05, 0, 0.1), 0.041113, 6, 'over');
  printed(m.free(1, 100, 0, 0.05, 0.5, PI / 20), 0.05, 6, 'undamped');
  // the case formulas of the specification, evaluated independently
  near(m.free(1, 100, 1, 0.05, 0, 0.1), (0.05 + 0.5 * 0.1) * Math.exp(-1), 1e-15, 'critical formula');
  const s1 = -10 * (2 - Math.sqrt(3)), s2 = -10 * (2 + Math.sqrt(3));
  const A = (0 - s2 * 0.05) / (s1 - s2), B = (s1 * 0.05 - 0) / (s1 - s2);
  near(m.free(1, 100, 2, 0.05, 0, 0.1), A * Math.exp(s1 * 0.1) + B * Math.exp(s2 * 0.1), 1e-14, 'over formula');
});

// central finite differences
function fd1(f, t, h) { return (f(t + h) - f(t - h)) / (2 * h); }
function fd2(f, t, h) { return (f(t + h) - 2 * f(t) + f(t - h)) / (h * h); }
function randomCase(R, i) {
  const mm = uni(R, 0.5, 10), kk = uni(R, 10, 2000);
  const kind = i % 4;
  const zeta = kind === 0 ? 0 : kind === 1 ? uni(R, 0.01, 0.99) : kind === 2 ? 1 : uni(R, 1.01, 2);
  return { m: mm, k: kk, zeta: zeta, x0: uni(R, -0.1, 0.1), v0: uni(R, -1, 1) };
}

group('invariant: free() satisfies m x″ + c x′ + k x = 0 and the initial conditions (300 random sets, all four cases)', function () {
  const R = rng(12345);
  let worst = 0;
  for (let i = 0; i < 300; i++) {
    const q = randomCase(R, i);
    const f = t => m.free(q.m, q.k, q.zeta, q.x0, q.v0, t);
    const wn = Math.sqrt(q.k / q.m), c = q.zeta * 2 * Math.sqrt(q.k * q.m);
    // initial conditions
    near(f(0), q.x0, 1e-15, 'x(0) case ' + i);
    near(m.freeVel(q.m, q.k, q.zeta, q.x0, q.v0, 0), q.v0, 1e-14, 'v(0) case ' + i);
    near(fd1(f, 1e-7 / wn, 1e-7 / wn), m.freeVel(q.m, q.k, q.zeta, q.x0, q.v0, 1e-7 / wn), 1e-6 * (Math.abs(q.v0) + wn * Math.abs(q.x0)), 'x′(0) by differences, case ' + i);
    const T = m.freeTEnd(q.m, q.k, q.zeta);
    for (let j = 0; j < 4; j++) {
      const t = uni(R, 0.05, 1) * T, h = 5e-4 / wn;
      const x = f(t), v = fd1(f, t, h), a = fd2(f, t, h);
      const res = q.m * a + c * v + q.k * x;
      const scale = Math.abs(q.m * a) + Math.abs(c * v) + Math.abs(q.k * x);
      if (scale < 1e-280) continue;
      const e = Math.abs(res) / scale;
      worst = Math.max(worst, e);
      assert.ok(e <= 1e-5, 'ODE residual ' + e + ' case ' + i + ' ' + JSON.stringify(q) + ' t = ' + t);
    }
  }
  return 'worst relative residual ' + worst.toExponential(2);
});

group('invariant: free() is continuous across ζ = 1 and the critical band', function () {
  const R = rng(777);
  for (let i = 0; i < 100; i++) {
    const q = randomCase(R, 2);
    const wn = Math.sqrt(q.k / q.m);
    const t = uni(R, 0, 8) / wn;
    const xc = m.free(q.m, q.k, 1, q.x0, q.v0, t);
    const sc = Math.abs(q.x0) + Math.abs(q.v0) / wn;
    [1e-9, 5e-7, 1.5e-6, 1e-5].forEach(function (e) {
      const tol = 20 * e * sc + 1e-15;
      near(m.free(q.m, q.k, 1 - e, q.x0, q.v0, t), xc, tol, 'below 1 by ' + e);
      near(m.free(q.m, q.k, 1 + e, q.x0, q.v0, t), xc, tol, 'above 1 by ' + e);
      near(m.freeVel(q.m, q.k, 1 - e, q.x0, q.v0, t), m.freeVel(q.m, q.k, 1, q.x0, q.v0, t), tol * wn, 'v below 1 by ' + e);
      near(m.freeVel(q.m, q.k, 1 + e, q.x0, q.v0, t), m.freeVel(q.m, q.k, 1, q.x0, q.v0, t), tol * wn, 'v above 1 by ' + e);
    });
    // the over-damped branch switch (cosh/sinh form ↔ exponentials at βt = 1) is continuous as well
    const z = uni(R, 1.01, 2), b = wn * Math.sqrt(z * z - 1), tb = 1 / b;
    near(m.free(q.m, q.k, z, q.x0, q.v0, tb * (1 - 1e-12)), m.free(q.m, q.k, z, q.x0, q.v0, tb * (1 + 1e-12)), 1e-12 * sc, 'branch switch');
  }
});

group('invariant: freeVel is the derivative of free (300 random sets)', function () {
  const R = rng(4242);
  let worst = 0;
  for (let i = 0; i < 300; i++) {
    const q = randomCase(R, i);
    const wn = Math.sqrt(q.k / q.m);
    const T = m.freeTEnd(q.m, q.k, q.zeta);
    const t = uni(R, 0.02, 1) * T, h = 1e-4 / wn;
    const v = m.freeVel(q.m, q.k, q.zeta, q.x0, q.v0, t);
    const vd = fd1(t2 => m.free(q.m, q.k, q.zeta, q.x0, q.v0, t2), t, h);
    const sc = Math.abs(v) + 1e-9 * wn * (Math.abs(q.x0) + Math.abs(q.v0) / wn);
    const e = Math.abs(v - vd) / sc;
    worst = Math.max(worst, e);
    assert.ok(e <= 1e-5, 'freeVel vs difference: ' + v + ' vs ' + vd + ' case ' + JSON.stringify(q) + ' t ' + t);
  }
  return 'worst ' + worst.toExponential(2);
});

group('peaks(): successive ratios = e^δ, spacing T_d, true positive maxima', function () {
  const R = rng(99);
  let count = 0;
  for (let i = 0; i < 200; i++) {
    const q = randomCase(R, 1);
    if (Math.abs(q.x0) < 1e-4 && Math.abs(q.v0) < 1e-3) continue;
    const d = m.damping(q.m, q.k, q.zeta);
    const T = m.freeTEnd(q.m, q.k, q.zeta);
    const pk = m.peaks(q.m, q.k, q.zeta, q.x0, q.v0, T);
    assert.ok(pk.length >= 2, 'at least two peaks in [0, freeTEnd] (got ' + pk.length + ')');
    pk.forEach(function (p, j) {
      assert.ok(p.t >= 0 && p.t <= T * (1 + 1e-12), 'inside [0, tEnd]');
      assert.ok(p.x > 0, 'positive');
      near(p.x, m.free(q.m, q.k, q.zeta, q.x0, q.v0, p.t), 1e-15, 'x = free(t)');
      if (p.t > 0) near(m.freeVel(q.m, q.k, q.zeta, q.x0, q.v0, p.t), 0, 1e-9 * (Math.abs(q.v0) + Math.abs(q.x0) * d.wd / (1 - q.zeta)), 'x′ = 0 at a peak');
      if (j > 0) {
        if (pk[j].x > 1e-250) rel(pk[j - 1].x / pk[j].x, d.ratio, 1e-9, 'ratio of successive peaks');
        rel(pk[j].t - pk[j - 1].t, d.Td, 1e-9, 'peaks T_d apart');
      }
    });
    // nothing missed: the first peak lies within one T_d of t = 0, the next one after the last lies beyond T
    assert.ok(pk[0].t < d.Td * (1 + 1e-9), 'first peak within one period');
    assert.ok(pk[pk.length - 1].t + d.Td > T, 'no peak missing at the end');
    // brute force: count sign changes + → − of x′ with x > 0 on a fine grid
    let n = 0; const N = 4000;
    let vp = m.freeVel(q.m, q.k, q.zeta, q.x0, q.v0, 0);
    if (vp === 0 && q.x0 > 0) n++;
    for (let s = 1; s <= N; s++) {
      const t = T * s / N, v = m.freeVel(q.m, q.k, q.zeta, q.x0, q.v0, t);
      if (vp > 0 && v <= 0 && m.free(q.m, q.k, q.zeta, q.x0, q.v0, t) > 0) n++;
      vp = v;
    }
    assert.ok(Math.abs(n - pk.length) <= 1, 'brute-force count ' + n + ' vs ' + pk.length);
    count++;
  }
  // a maximum at t = 0 counts when v₀ = 0 and x₀ > 0
  const p0 = m.peaks(1, 100, 0.1, 0.05, 0, 2);
  assert.strictEqual(p0[0].t, 0, 'peak at t = 0');
  near(p0[0].x, 0.05, 1e-15, 'x₁ = x₀');
  printed(p0[1].t, 0.631484, 6, 'second peak at T_d');
  printed(p0[1].x, 0.026590, 6, 'second peak value');
  printed(p0[0].x / p0[1].x, 1.88040, 5, 'x₁/x₂');
  // x₀ < 0, v₀ = 0: the first positive maximum is half a period later
  const pn = m.peaks(1, 100, 0.1, -0.05, 0, 2);
  rel(pn[0].t, m.damping(1, 100, 0.1).Td / 2, 1e-9, 'x₀ < 0: first peak at T_d/2');
  // undamped peaks are equal; critical / over-damped responses have no listed peaks
  const pu = m.peaks(1, 100, 0, 0.05, 0, 3);
  pu.forEach(p => near(p.x, 0.05, 1e-15, 'undamped peak'));
  assert.deepStrictEqual(m.peaks(1, 100, 1, 0.05, 0, 3), []);
  assert.deepStrictEqual(m.peaks(1, 100, 2, 0, 1, 3), []);
  assert.deepStrictEqual(m.peaks(1, 100, 0.1, 0, 0, 3), []);
  return count + ' random sets';
});

group('envelope(): |x| ≤ X·e^(−ζωₙt), touching at the peaks', function () {
  const R = rng(2024);
  for (let i = 0; i < 100; i++) {
    const q = randomCase(R, 1);
    const X = m.envelope(q.m, q.k, q.zeta, q.x0, q.v0);
    const wn = Math.sqrt(q.k / q.m), T = m.freeTEnd(q.m, q.k, q.zeta);
    const o = (q.v0 + q.zeta * wn * q.x0) / (wn * Math.sqrt(1 - q.zeta * q.zeta));
    near(X, Math.hypot(q.x0, o), 1e-15, 'formula');
    for (let s = 0; s <= 200; s++) {
      const t = T * s / 200;
      assert.ok(Math.abs(m.free(q.m, q.k, q.zeta, q.x0, q.v0, t)) <= X * Math.exp(-q.zeta * wn * t) * (1 + 1e-12) + 1e-300, 'bounded');
    }
  }
  near(m.envelope(1, 100, 0.1, 0.05, 0), 0.05 / Math.sqrt(0.99), 1e-15, 'v₀ = 0: X = x₀/√(1 − ζ²)');
  isNaNField(m.envelope(1, 100, 1, 0.05, 0), 'critical envelope');
  isNaNField(m.envelope(1, 100, 1.5, 0.05, 0), 'over envelope');
});

group('freeTEnd()', function () {
  const d = m.damping(1, 100, 0.1);
  near(m.freeTEnd(1, 100, 0.1), Math.max(Math.min(6 / 1, 12 * d.Td), 3 * d.Td), 1e-12, 'under ζ = 0.1');
  printed(m.freeTEnd(1, 100, 0.1), 6, 9, 'under ζ = 0.1 is 6/(ζωₙ) = 6 s');
  near(m.freeTEnd(1, 100, 0.01), 12 * m.damping(1, 100, 0.01).Td, 1e-12, 'light damping: 12 T_d');
  near(m.freeTEnd(1, 100, 0.9), 3 * m.damping(1, 100, 0.9).Td, 1e-12, 'heavy damping: at least 3 T_d');
  near(m.freeTEnd(1, 100, 0), 6 * 2 * PI / 10, 1e-12, 'undamped: 6 Tₙ');
  near(m.freeTEnd(1, 100, 1), 0.8, 1e-12, 'critical: 8/ωₙ');
  near(m.freeTEnd(1, 100, 2), 6 / (10 * (2 - Math.sqrt(3))), 1e-9, 'over: 6/|s₁|');
});

group('check: MF, phase, TR, unbalance, peakMF', function () {
  printed(m.MF(1, 0.1), 5, 4, 'MF(1, 0.1)');
  [0, 0.05, 0.3, 1, 2].forEach(z => printed(m.MF(0, z), 1, 9, 'MF(0, ' + z + ')'));
  printed(m.MF(Math.SQRT2, 0.1), 0.96225, 5, 'MF(√2, 0.1)');
  [0.01, 0.1, 0.5, 1, 2].forEach(z => printed(M.deg(m.phase(1, z)), 90, 9, 'phase(1, ' + z + ')'));
  printed(M.deg(m.phase(2, 0.1)), 172.405, 3, 'phase(2, 0.1)');
  [0.05, 0.3, 1].forEach(z => printed(m.TR(Math.SQRT2, z), 1, 9, 'TR(√2, ' + z + ')'));
  printed(m.TR(3, 0.1), 0.14537, 5, 'TR(3, 0.1)');
  printed(m.TR(3, 0.5), 0.37012, 5, 'TR(3, 0.5)');
  const pk = m.peakMF(0.1);
  printed(pk.r, 0.98995, 5, 'peakMF(0.1).r');
  printed(pk.MF, 5.0252, 4, 'peakMF(0.1).MF');
  assert.strictEqual(m.peakMF(0.8), null, 'peakMF(0.8)');
  assert.strictEqual(m.peakMF(Math.SQRT1_2), null, 'peakMF(1/√2)');
  assert.ok(m.peakMF(0.7) !== null, 'peakMF(0.7) exists');
  printed(m.unbalance(1, 0.1), 5, 4, 'unbalance(1, 0.1)');
  [0.05, 0.3, 1].forEach(function (z) {
    near(m.unbalance(1e3, z), 1, 1e-5, 'unbalance → 1, ζ = ' + z);
    assert.ok(Math.abs(m.unbalance(40, z) - 1) < Math.abs(m.unbalance(10, z) - 1), 'approaches 1 as r grows, ζ = ' + z);
  });
  // r = 1 gives 1/(2ζ); the peak formulas; ζ = 0 at r = 1 is infinite
  [0.05, 0.2, 0.6].forEach(z => near(m.MF(1, z), 1 / (2 * z), 1e-12, 'MF(1) = 1/2ζ'));
  assert.strictEqual(m.MF(1, 0), Infinity);
  assert.strictEqual(m.TR(1, 0), Infinity);
  assert.strictEqual(m.unbalance(1, 0), Infinity);
  near(m.phase(0.5, 0), 0, 0, 'ζ = 0 below resonance: in phase');
  near(m.phase(2, 0), PI, 0, 'ζ = 0 above resonance: anti-phase');
  near(m.phase(0, 0.3), 0, 0, 'r = 0');
  near(m.phase(1, 0), PI / 2, 0, 'ζ = 0, r = 1: π/2 (limit / secular solution)');
});

group('peakMF is the maximum of MF over r', function () {
  const R = rng(5);
  for (let i = 0; i < 100; i++) {
    const z = uni(R, 0.02, 0.70);
    const pk = m.peakMF(z);
    near(m.MF(pk.r, z), pk.MF, 1e-9 * pk.MF, 'MF(r_peak) = MF_max');
    for (let s = 0; s <= 400; s++) assert.ok(m.MF(4 * s / 400, z) <= pk.MF * (1 + 1e-12), 'nothing higher');
  }
  for (let i = 0; i < 50; i++) {
    const z = uni(R, 0.7072, 2);
    for (let s = 1; s <= 200; s++) assert.ok(m.MF(4 * s / 200, z) <= 1 + 1e-15, 'ζ ≥ 1/√2: MF ≤ 1, decreasing from r = 0');
  }
});

group('invariant: MF, phase, TR agree with the complex frequency response (500 random sets)', function () {
  const R = rng(31337);
  for (let i = 0; i < 500; i++) {
    const mm = uni(R, 0.5, 10), kk = uni(R, 10, 2000), z = i % 10 === 0 ? 0 : uni(R, 0, 2), r = uni(R, 0, 4);
    if (z === 0 && Math.abs(r - 1) < 1e-6) continue;
    const wn = Math.sqrt(kk / mm), w = r * wn, c = 2 * z * Math.sqrt(kk * mm);
    const den = M.cx(kk - mm * w * w, c * w);
    const H = M.cdiv(1, den);                                    // X/F₀
    const T = M.cdiv(M.cx(kk, c * w), den);                      // F_T/F₀
    rel(m.MF(r, z), M.cabs(H) * kk, 1e-9, 'MF = |X|k/F₀');
    rel(m.TR(r, z), M.cabs(T), 1e-9, 'TR = |F_T/F₀|');
    near(Math.abs(M.wrapPi(m.phase(r, z) + M.carg(H))), 0, 1e-9, 'φ = −arg(X/F₀) (mod 2π; −0 imaginary parts give ±π)');
    const ph = m.phase(r, z);
    assert.ok(ph >= 0 && ph <= PI, 'φ ∈ [0, π]');
    rel(m.unbalance(r, z), r * r * M.cabs(H) * kk, 1e-9, 'unbalance = r²·MF');
  }
});

group('invariant: TR < 1 exactly when r > √2; in the isolation region TR increases with ζ (random)', function () {
  const R = rng(8080);
  for (let i = 0; i < 2000; i++) {
    const r = uni(R, 0, 4), z = uni(R, 0, 2);
    if (Math.abs(r - Math.SQRT2) < 1e-9) continue;
    assert.strictEqual(m.TR(r, z) < 1, r > Math.SQRT2, 'TR(' + r + ', ' + z + ') = ' + m.TR(r, z));
    if (r > Math.SQRT2 + 1e-6) {
      const z2 = z + uni(R, 0.001, 0.5);
      assert.ok(m.TR(r, z2) > m.TR(r, z), 'TR increases with ζ at r = ' + r);
    } else if (r < Math.SQRT2 - 1e-6) {
      const z2 = z + uni(R, 0.001, 0.5);
      assert.ok(m.TR(r, z2) < m.TR(r, z), 'below √2 more damping lowers TR, r = ' + r);
    }
  }
});

if (failures) { console.log(failures + ' group(s) FAILED'); process.exit(1); }
console.log('all groups passed');
