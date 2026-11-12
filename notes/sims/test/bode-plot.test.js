// Model tests for bode-plot. Run: node notes/sims/test/bode-plot.test.js
'use strict';
const assert = require('assert');
const RASim = require('./load.js')('bode-plot');
const ID = 'bode-plot';
const m = RASim.sims[ID].model;
const M = RASim.math;

let failures = 0;
function group(name, fn) {
  try { const extra = fn(); console.log('ok   ' + name + (extra ? '  (' + extra + ')' : '')); }
  catch (e) { failures++; console.log('FAIL ' + name + ': ' + (e && e.message || e)); }
}
// tolerance: half a unit of the last printed digit or 5e-4 relative, whichever is larger
function printed(actual, expected, dec, what) {
  const tol = Math.max(0.5 * Math.pow(10, -dec), 5e-4 * Math.abs(expected)) + 1e-12;
  assert.ok(Number.isFinite(actual) && Math.abs(actual - expected) <= tol, what + ': ' + actual + ' vs ' + expected + ' (tol ' + tol + ')');
}
function near(actual, expected, tol, what) {
  assert.ok(Number.isFinite(actual) && Math.abs(actual - expected) <= tol, what + ': ' + actual + ' vs ' + expected + ' (tol ' + tol + ')');
}
function rel(actual, expected, tol, what) {
  if (!Number.isFinite(expected)) {
    assert.ok(Object.is(actual, expected) || (Number.isNaN(actual) && Number.isNaN(expected)), what + ': ' + actual + ' vs ' + expected);
    return;
  }
  near(actual, expected, tol * Math.max(1, Math.abs(expected)), what);
}
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
const R2D = 180 / Math.PI;
function wrap180(d) { return d - 360 * Math.round(d / 360); }

// presets from the specification (independent of the simulation file)
const P = {
  A: { K: 1, N: 1, factors: [{ type: 'pole', w: 1 }, { type: 'pole', w: 2 }], delay: 0 },
  B: { K: 100, N: 0, factors: [{ type: 'pole', w: 1 }, { type: 'pole', w: 10 }, { type: 'pole', w: 100 }], delay: 0 },
  C: { K: 1, N: 0, factors: [{ type: 'quad', w: 10, zeta: 0.2 }], delay: 0 },
  D: { K: 1, N: 0, factors: [{ type: 'zero', w: 1 }, { type: 'pole', w: 10 }], delay: 0 },
  E: { K: 1, N: 0, factors: [{ type: 'zero', w: 10 }, { type: 'pole', w: 1 }], delay: 0 },
  F: { K: 1, N: 0, factors: [{ type: 'rhpzero', w: 10 }, { type: 'pole', w: 10 }], delay: 0 },
  G: { K: 1, N: 0, factors: [{ type: 'pole', w: 1 }], delay: 0.5 }
};
function withK(sys, K) { return Object.assign({}, sys, { K: K }); }

// ---------------------------------------------------------------- independent reference (complex arithmetic)
// G(jω) of the delay-free part, factor by factor, as a complex number
function cmul(a, b) { return { re: a.re * b.re - a.im * b.im, im: a.re * b.im + a.im * b.re }; }
function cinv(a) { const d = a.re * a.re + a.im * a.im; return { re: a.re / d, im: -a.im / d }; }
function Grat(sys, w) {
  let g = { re: sys.K, im: 0 };
  const sN = sys.N >= 0 ? cinv(Math.pow(w, sys.N) === 0 ? { re: 1, im: 0 } : jPow(w, sys.N)) : jPow(w, -sys.N);
  g = cmul(g, sN);
  sys.factors.forEach(function (f) {
    if (!f || f.type === 'off') return;
    const x = w / f.w;
    if (f.type === 'pole') g = cmul(g, cinv({ re: 1, im: x }));
    else if (f.type === 'zero') g = cmul(g, { re: 1, im: x });
    else if (f.type === 'rhpzero') g = cmul(g, { re: 1, im: -x });
    else if (f.type === 'quad') g = cmul(g, cinv({ re: 1 - x * x, im: 2 * f.zeta * x }));
  });
  return g;
}
function jPow(w, n) { let z = { re: 1, im: 0 }; for (let i = 0; i < n; i++) z = cmul(z, { re: 0, im: w }); return z; }

// dense brute-force scan: 20000 samples per decade over 1e-4 … 1e6, phase of the rational part from atan2 of the
// complex product, unwrapped from its low-frequency value −90N, plus the delay's −ωT added analytically
function bruteMargins(sys) {
  const n = 200001, lw0 = -4, lw1 = 6;
  const lw = new Float64Array(n), mag = new Float64Array(n), ph = new Float64Array(n);
  let prev = null;
  for (let i = 0; i < n; i++) {
    lw[i] = lw0 + (lw1 - lw0) * i / (n - 1);
    const w = Math.pow(10, lw[i]);
    const g = Grat(sys, w);
    mag[i] = 20 * Math.log10(Math.hypot(g.re, g.im));
    let a = Math.atan2(g.im, g.re) * R2D;
    if (prev === null) a = a + 360 * Math.round((-90 * sys.N - a) / 360);
    else a = a + 360 * Math.round((prev - a) / 360);
    prev = a;
    ph[i] = a - w * (sys.delay || 0) * R2D;
  }
  function interp(i, f0, f1) { const t = f0 / (f0 - f1); return Math.pow(10, lw[i] + t * (lw[i + 1] - lw[i])); }
  const gcs = [];
  for (let i = 0; i + 1 < n; i++) if ((mag[i] > 1e-9 && mag[i + 1] < -1e-9) || (mag[i] < -1e-9 && mag[i + 1] > 1e-9)) gcs.push(interp(i, mag[i], mag[i + 1]));
  let pm = Infinity, wgc = NaN;
  gcs.forEach(function (w) {
    const i = Math.min(n - 2, Math.floor((Math.log10(w) - lw0) / (lw1 - lw0) * (n - 1)));
    const t = (Math.log10(w) - lw[i]) / (lw[i + 1] - lw[i]);
    const p = 180 + ph[i] + t * (ph[i + 1] - ph[i]);
    if (p < pm) { pm = p; wgc = w; }
  });
  if (!gcs.length) {
    let allZero = true, allNeg = true;
    for (let i = 0; i < n; i++) { if (Math.abs(mag[i]) > 1e-9) allZero = false; if (mag[i] >= -1e-9) allNeg = false; }
    pm = allZero ? NaN : allNeg ? Infinity : NaN;
  }
  const pcs = [];
  let gm = Infinity, wpc = NaN;
  for (let i = 0; i + 1 < n; i++) {
    const h0 = (ph[i] + 180) / 360, h1 = (ph[i + 1] + 180) / 360;
    const lo = Math.min(h0, h1), hi = Math.max(h0, h1);
    for (let k = Math.floor(lo) + 1; k <= Math.min(0, Math.floor(hi)); k++) {
      const w = interp(i, h0 - k, h1 - k);
      const t = (h0 - k) / (h0 - h1);
      const md = mag[i] + t * (mag[i + 1] - mag[i]);
      pcs.push(w);
      if (-md < gm) { gm = -md; wpc = w; }
    }
  }
  return { gcs: gcs, pcs: pcs, pm: pm, wgc: wgc, gm: gm, wpc: wpc };
}

// ---------------------------------------------------------------- random systems
function randomSys(R, opts) {
  opts = opts || {};
  const types = ['off', 'pole', 'zero', 'rhpzero', 'quad'];
  const f = [];
  for (let i = 0; i < 4; i++) {
    f.push({ type: types[Math.floor(R() * 5)], w: Math.pow(10, -2 + 5 * R()), zeta: 0.05 + 0.95 * R() });
  }
  return {
    K: Math.pow(10, -2 + 5 * R()), N: [-1, 0, 1, 2][Math.floor(R() * 4)], factors: f,
    delay: opts.noDelay ? 0 : (R() < 0.5 ? 0 : Math.round(200 * R()) / 100)
  };
}

// ================================================================ check values
group('one pole at w = 10: exact and asymptotic values', function () {
  const s = { K: 1, N: 0, factors: [{ type: 'pole', w: 10 }], delay: 0 };
  let r = m.response(s, 10);
  printed(r.magDb, -3.0103, 4, '|G(j10)| dB'); printed(r.phaseDeg, -45, 3, '∠G(j10)');
  r = m.response(s, 100);
  printed(r.magDb, -20.043, 3, '|G(j100)| dB'); printed(r.phaseDeg, -84.289, 3, '∠G(j100)');
  printed(m.asymMagDb(s, 100), -20, 3, 'asymptote at 100');
  printed(m.asymPhaseDeg(s, 1), 0, 3, 'asym phase at 1');
  printed(m.asymPhaseDeg(s, 10), -45, 3, 'asym phase at 10');
  printed(m.asymPhaseDeg(s, 100), -90, 3, 'asym phase at 100');
  const f = m.factorResponse({ type: 'pole', w: 10 }, 10);
  printed(f.magDb, -3.0103, 4, 'factorResponse mag'); printed(f.phaseDeg, -45, 3, 'factorResponse phase');
});

group('preset A: margins, stability, slopes; K = 3 marginal; K = 10 unstable', function () {
  const mg = m.margins(P.A);
  printed(mg.wpc, 1.41421, 5, 'ω_pc'); printed(mg.gmDb, 9.542, 3, 'GM');
  printed(Math.pow(10, mg.gmDb / 20), 3, 4, 'GM ratio');
  printed(mg.wgc, 0.7494, 4, 'ω_gc'); printed(mg.pmDeg, 32.61, 2, 'PM');
  assert.deepStrictEqual(mg.gainCrossovers.length, 1); assert.deepStrictEqual(mg.phaseCrossovers.length, 1);
  assert.strictEqual(m.closedLoopStable(P.A), true, 'A stable');
  const sl = m.slopes(P.A);
  assert.strictEqual(sl.low, -20); assert.strictEqual(sl.high, -60);
  const m3 = m.margins(withK(P.A, 3));
  near(m3.gmDb, 0, 5e-4, 'GM at K = 3');
  assert.strictEqual(m.closedLoopStable(withK(P.A, 3)), false, 'K = 3 is not asymptotically stable');
  assert.strictEqual(m.closedLoopMarginal(withK(P.A, 3)), true, 'K = 3 marginal');
  const p3 = m.closedLoopPoles(withK(P.A, 3));
  assert.ok(p3.some(function (p) { return Math.abs(p.re) < 1e-6 && Math.abs(Math.abs(p.im) - Math.SQRT2) < 1e-6; }), 'poles ±j√2');
  const m10 = m.margins(withK(P.A, 10));
  printed(m10.gmDb, -10.46, 2, 'GM at K = 10');
  assert.ok(m10.pmDeg < 0, 'PM < 0 at K = 10');
  assert.strictEqual(m.closedLoopStable(withK(P.A, 10)), false, 'K = 10 unstable');
  assert.strictEqual(m.closedLoopMarginal(withK(P.A, 10)), false, 'K = 10 not marginal');
  const fr = m.freqRange(P.A);
  near(fr.wmin, 0.01, 1e-15, 'wmin'); near(fr.wmax, 100, 1e-12, 'wmax');
});

group('preset D (lead): max phase, magnitude there, high-frequency gain', function () {
  // golden-section search for the maximum phase
  let a = -1, b = 2;
  const g = (Math.sqrt(5) - 1) / 2;
  for (let i = 0; i < 200; i++) {
    const c = b - g * (b - a), d = a + g * (b - a);
    if (m.response(P.D, Math.pow(10, c)).phaseDeg > m.response(P.D, Math.pow(10, d)).phaseDeg) b = d; else a = c;
  }
  const wmax = Math.pow(10, 0.5 * (a + b));
  printed(wmax, 3.1623, 4, 'ω of maximum phase');
  printed(m.response(P.D, wmax).phaseDeg, 54.903, 3, 'maximum phase');
  printed(m.response(P.D, Math.sqrt(10)).magDb, 10.000, 3, 'magnitude at √10');
  printed(m.response(P.D, 1e6).magDb, 20, 3, 'high-frequency gain');
  printed(m.asymMagDb(P.D, 1e6), 20, 3, 'high-frequency asymptote');
});

group('preset C (resonant pair): values at ωₙ and the peak', function () {
  const r = m.response(P.C, 10);
  printed(r.magDb, 7.959, 3, '|G(j10)|'); printed(r.phaseDeg, -90, 3, '∠G(j10)');
  const res = m.resonance(P.C);
  assert.strictEqual(res.length, 1);
  printed(res[0].MrDb, 8.136, 3, 'Mr dB'); printed(res[0].wr, 9.5917, 4, 'ωr');
  printed(res[0].Mr, 1 / (0.4 * Math.sqrt(0.96)), 4, 'Mr');
  // the formula is the true maximum of the exact curve
  let best = -Infinity, wb = 0;
  for (let i = 0; i <= 200000; i++) { const w = 9 + i * 1e-5; const v = m.response(P.C, w).magDb; if (v > best) { best = v; wb = w; } }
  printed(best, 8.136, 3, 'scanned peak'); printed(wb, 9.5917, 4, 'scanned ωr');
  assert.strictEqual(m.resonance({ K: 1, N: 0, factors: [{ type: 'quad', w: 10, zeta: 0.71 }] }).length, 0, 'no peak for ζ ≥ 0.7071');
  assert.strictEqual(m.resonance({ K: 1, N: 0, factors: [{ type: 'quad', w: 10, zeta: 0.70 }] }).length, 1, 'peak for ζ = 0.70');
});

group('preset F (non-minimum phase): 0 dB everywhere, phase −90° at 10 → −180°', function () {
  for (let lw = -4; lw <= 6; lw += 0.25) near(m.response(P.F, Math.pow(10, lw)).magDb, 0, 1e-9, '|G| at 10^' + lw);
  printed(m.response(P.F, 10).phaseDeg, -90, 3, '∠G(j10)');
  near(m.response(P.F, 1e6).phaseDeg, -180, 0.01, '∠G(j1e6)');
  assert.ok(m.response(P.F, 1e6).phaseDeg > -180, 'phase tends to −180 from above');
  const mg = m.margins(P.F);
  assert.ok(mg.unity && Number.isNaN(mg.pmDeg) && mg.gmDb === Infinity, 'unity loop: PM undefined, GM ∞');
  assert.strictEqual(m.closedLoopStable(P.F), true, 'closed loop (1 − s/10)/2 has no poles');
});

group('preset G (delay): values at ω = 2', function () {
  const r = m.response(P.G, 2);
  printed(r.magDb, -6.990, 3, '|G(j2)|'); printed(r.phaseDeg, -120.731, 3, '∠G(j2)');
  assert.strictEqual(m.closedLoopStable(P.G), null, 'delay → null');
  const mg = m.margins(P.G);
  assert.strictEqual(mg.pmDeg, Infinity, '|G| < 1 everywhere → PM ∞');
  assert.ok(mg.gmDb > 0 && mg.truncated && mg.phaseCrossovers.length === 200, 'GM > 0, crossover list truncated');
});

group('preset B: margins against an independent brute-force scan', function () {
  const mg = m.margins(P.B), bf = bruteMargins(P.B);
  rel(mg.wgc, bf.wgc, 1e-3, 'ω_gc'); rel(mg.pmDeg, bf.pm, 1e-3, 'PM');
  rel(mg.wpc, bf.wpc, 1e-3, 'ω_pc'); rel(mg.gmDb, bf.gm, 1e-3, 'GM');
  // closed form: phase crossover from atan sums, K_crit from Routh (0.111·1.11 = 0.001·(1 + K))
  const Kc = 0.111 * 1.11 / 0.001 - 1;
  printed(mg.gmDb, 20 * Math.log10(Kc / 100), 4, 'GM vs Routh');
  assert.strictEqual(m.closedLoopStable(P.B), true);
  const sl = m.slopes(P.B);
  assert.strictEqual(sl.low, 0); assert.strictEqual(sl.high, -60);
  const fr = m.freqRange(P.B);
  near(fr.wmin, 0.1, 1e-15, 'wmin'); near(fr.wmax, 1000, 1e-12, 'wmax');
  return 'ω_gc ' + mg.wgc.toFixed(4) + ', PM ' + mg.pmDeg.toFixed(3) + '°, ω_pc ' + mg.wpc.toFixed(4) + ', GM ' + mg.gmDb.toFixed(4) + ' dB';
});

// ================================================================ invariants
group('invariant: K × 10 adds exactly 20 dB, phase unchanged (200 systems × 20 ω)', function () {
  const R = rng(11);
  for (let t = 0; t < 200; t++) {
    const s = randomSys(R), s10 = withK(s, s.K * 10);
    for (let j = 0; j < 20; j++) {
      const w = Math.pow(10, -3 + 8 * R());
      const a = m.response(s, w), b = m.response(s10, w);
      near(b.magDb - a.magDb, 20, 1e-9, 'Δmag');
      near(b.phaseDeg, a.phaseDeg, 1e-12 * Math.max(1, Math.abs(a.phaseDeg)), 'phase');
    }
  }
});

group('invariant: response() = 20 log|G(jω)| and ∠G (mod 360°) from polys() and the delay (200 × 20)', function () {
  const R = rng(22);
  let worstM = 0, worstP = 0;
  for (let t = 0; t < 200; t++) {
    const s = randomSys(R);
    const P2 = m.polys(s);
    for (let j = 0; j < 20; j++) {
      const w = Math.pow(10, -3 + 8 * R());
      const jw = { re: 0, im: w };
      const g0 = M.cdiv(M.polyEval(P2.num, jw), M.polyEval(P2.den, jw));
      const g = M.cmul(g0, { re: Math.cos(-w * s.delay), im: Math.sin(-w * s.delay) });
      const r = m.response(s, w);
      const md = Math.abs(r.magDb - 20 * Math.log10(M.cabs(g)));
      const pd = Math.abs(wrap180(r.phaseDeg - Math.atan2(g.im, g.re) * R2D));
      worstM = Math.max(worstM, md); worstP = Math.max(worstP, pd);
      assert.ok(md < 1e-7, 'magnitude mismatch ' + md + ' dB');
      assert.ok(pd < 1e-6, 'phase mismatch ' + pd + '°');
      // and against the independent factor-by-factor product
      const gr = Grat(s, w);
      assert.ok(Math.abs(20 * Math.log10(Math.hypot(gr.re, gr.im)) - 20 * Math.log10(M.cabs(g0))) < 1e-7, 'polys() vs factor product');
    }
  }
  return 'worst ' + worstM.toExponential(1) + ' dB, ' + worstP.toExponential(1) + '°';
});

group('invariant: at an isolated real corner exact − asymptote = ∓3.0103 dB and the factor phase is ∓45°', function () {
  const R = rng(33);
  for (let t = 0; t < 200; t++) {
    // corners at least 2.5 decades apart
    const n = 1 + Math.floor(R() * 3), base = -2 + R();
    const f = [];
    for (let i = 0; i < n; i++) {
      const tp = ['pole', 'zero', 'rhpzero', 'quad'][Math.floor(R() * 4)];
      f.push({ type: tp, w: Math.pow(10, base + 2.5 * i + 0.3 * R()), zeta: 0.3 + 0.7 * R() });
    }
    const s = { K: Math.pow(10, -2 + 5 * R()), N: [-1, 0, 1, 2][Math.floor(R() * 4)], factors: f, delay: 0 };
    f.forEach(function (fi) {
      if (fi.type === 'quad') return;
      const d = m.response(s, fi.w).magDb - m.asymMagDb(s, fi.w);
      const sign = fi.type === 'pole' ? -1 : 1;
      printed(d, sign * 3.0103, 4, fi.type + ' corner difference');
      near(m.factorResponse(fi, fi.w).phaseDeg, fi.type === 'zero' ? 45 : -45, 1e-12, fi.type + ' phase at corner');
      near(m.factorResponse(fi, fi.w).magDb, sign * 10 * Math.log10(2), 1e-12, fi.type + ' magnitude at corner');
    });
  }
});

group('invariant: delay-free asymptotes equal the exact curves for ω → 0 and ω → ∞', function () {
  const R = rng(44);
  for (let t = 0; t < 200; t++) {
    const s = randomSys(R, { noDelay: true });
    const cs = m.corners(s);
    const lo = (cs.length ? cs[0] : 1) * 1e-7, hi = (cs.length ? cs[cs.length - 1] : 1) * 1e7;
    [lo, hi].forEach(function (w) {
      const r = m.response(s, w);
      near(r.magDb, m.asymMagDb(s, w), 1e-4, 'magnitude at ' + w);
      near(r.phaseDeg, m.asymPhaseDeg(s, w), 1e-3, 'phase at ' + w);
    });
  }
});

group('invariant: margins() agrees with a brute-force dense scan (1e-3 relative) for all presets', function () {
  const out = [];
  Object.keys(P).forEach(function (k) {
    [1, 0.5, 2].forEach(function (scale) {
      const s = withK(P[k], P[k].K * scale);
      const mg = m.margins(s), bf = bruteMargins(s);
      const tag = k + ' (K×' + scale + ')';
      rel(mg.pmDeg, bf.pm, 1e-3, tag + ' PM'); rel(mg.wgc, bf.wgc, 1e-3, tag + ' ω_gc');
      rel(mg.gmDb, bf.gm, 1e-3, tag + ' GM'); rel(mg.wpc, bf.wpc, 1e-3, tag + ' ω_pc');
      assert.strictEqual(mg.gainCrossovers.length, bf.gcs.length, tag + ' number of gain crossovers');
      mg.gainCrossovers.forEach(function (w, i) { rel(w, bf.gcs[i], 1e-3, tag + ' gain crossover ' + i); });
      if (!s.delay) {
        assert.strictEqual(mg.phaseCrossovers.length, bf.pcs.length, tag + ' number of phase crossovers');
        mg.phaseCrossovers.forEach(function (w, i) { rel(w, bf.pcs[i], 1e-3, tag + ' phase crossover ' + i); });
      } else {
        assert.strictEqual(mg.phaseCrossoverCount, bf.pcs.length, tag + ' number of phase crossovers');
        mg.phaseCrossovers.forEach(function (w, i) { rel(w, bf.pcs[i], 1e-3, tag + ' phase crossover ' + i); });
      }
      if (scale === 1) out.push(k + ': PM ' + (isFinite(mg.pmDeg) ? mg.pmDeg.toFixed(2) : mg.pmDeg) + ', GM ' + (isFinite(mg.gmDb) ? mg.gmDb.toFixed(2) : mg.gmDb));
    });
  });
  return out.join('; ');
});

group('invariant: margins() against the brute-force scan for 30 random systems', function () {
  const R = rng(55);
  for (let t = 0; t < 30; t++) {
    const s = randomSys(R);
    const mg = m.margins(s), bf = bruteMargins(s);
    rel(mg.pmDeg, bf.pm, 1e-3, 'PM #' + t); rel(mg.gmDb, bf.gm, 1e-3, 'GM #' + t);
    if (isFinite(bf.wgc)) rel(mg.wgc, bf.wgc, 1e-3, 'ω_gc #' + t);
    if (isFinite(bf.wpc)) rel(mg.wpc, bf.wpc, 1e-3, 'ω_pc #' + t);
  }
});

group('invariant: presets A and B at several K — closedLoopStable = (GM > 0 and PM > 0)', function () {
  const cases = [['A', [0.1, 0.5, 1, 2, 2.9, 3.1, 5, 10, 100]], ['B', [1, 10, 50, 100, 120, 125, 200, 1000]]];
  cases.forEach(function (c) {
    c[1].forEach(function (K) {
      const s = withK(P[c[0]], K), mg = m.margins(s);
      const byMargins = mg.gmDb > 0 && mg.pmDeg > 0;
      assert.strictEqual(m.closedLoopStable(s), byMargins, c[0] + ' K = ' + K);
    });
  });
});

// ================================================================ the rest of the API
group('polys(), closedLoopPoles() residuals, corners(), factorResponse() special types', function () {
  const p = m.polys(P.A);
  assert.deepStrictEqual(p.num, [1]);
  assert.deepStrictEqual(p.den.map(function (v) { return +v.toFixed(12); }), [0.5, 1.5, 1, 0]);
  const pd = m.polys({ K: 2, N: -1, factors: [{ type: 'rhpzero', w: 4 }, { type: 'quad', w: 2, zeta: 0.5 }] });
  assert.deepStrictEqual(pd.num, [-0.5, 2, 0]);
  assert.deepStrictEqual(pd.den, [0.25, 0.5, 1]);
  const R = rng(66);
  let worst = 0;
  for (let t = 0; t < 200; t++) {
    const s = randomSys(R, { noDelay: true }), q = m.polys(s);
    const poles = m.closedLoopPoles(s);
    // backward error: |D(p) + N(p)| relative to Σ|coefficient|·|p|^k of both polynomials
    poles.forEach(function (z) {
      const r = M.cabs(z);
      const absSum = function (c) { return c.reduce(function (acc, v) { return acc * r + Math.abs(v); }, 0); };
      const res = M.cabs(M.cadd(M.polyEval(q.den, z), M.polyEval(q.num, z))) / (absSum(q.den) + absSum(q.num));
      worst = Math.max(worst, res);
      assert.ok(res <= 1e-9, 'residual at ' + RASim.fmtComplex(z) + ': ' + res);
    });
  }
  assert.deepStrictEqual(m.corners({ K: 1, N: 0, factors: [{ type: 'pole', w: 5 }, { type: 'off', w: 1 }, { type: 'quad', w: 0.5, zeta: 0.3 }, { type: 'zero', w: 50 }] }), [0.5, 5, 50]);
  near(m.factorResponse({ type: 'gain', K: 100 }, 3).magDb, 40, 1e-12, 'gain factor');
  assert.strictEqual(m.factorResponse({ type: 'gain', K: 100 }, 3).phaseDeg, 0);
  near(m.factorResponse({ type: 'integrator', N: 2 }, 10).magDb, -40, 1e-12, 'integrator 1/s²');
  near(m.factorResponse({ type: 'integrator', N: 2 }, 10).phaseDeg, -180, 1e-12, 'integrator phase');
  near(m.factorResponse({ type: 'integrator', N: -1 }, 10).magDb, 20, 1e-12, 'differentiator');
  near(m.factorResponse({ type: 'delay', T: 0.5 }, 2).phaseDeg, -R2D, 1e-12, 'delay phase −ωT');
  assert.strictEqual(m.factorResponse({ type: 'delay', T: 0.5 }, 2).magDb, 0);
  // quad phase runs continuously 0 → −180°
  let last = 0;
  for (let lw = -3; lw <= 3; lw += 0.01) { const ph = m.factorResponse({ type: 'quad', w: 1, zeta: 0.05 }, Math.pow(10, lw)).phaseDeg; assert.ok(ph <= last + 1e-12 && ph > -180, 'quad phase'); last = ph; }
  // quad asymptotic phase: 0 below ωₙ·10^−ζ, −180 above ωₙ·10^ζ, −90/ζ °/dec in between
  const q = { K: 1, N: 0, factors: [{ type: 'quad', w: 10, zeta: 0.5 }] };
  near(m.asymPhaseDeg(q, 10 * Math.pow(10, -0.5)), 0, 1e-9, 'quad asym start');
  near(m.asymPhaseDeg(q, 10), -90, 1e-9, 'quad asym at ωₙ');
  near(m.asymPhaseDeg(q, 10 * Math.pow(10, 0.5)), -180, 1e-9, 'quad asym end');
  near(m.asymPhaseDeg(q, 10 * Math.pow(10, 0.1)) - m.asymPhaseDeg(q, 10), -18, 1e-9, 'quad asym slope');
  near(m.asymMagDb(q, 1000), -80, 1e-9, 'quad asym magnitude −40 dB/dec');
  return 'worst closed-loop pole backward error ' + worst.toExponential(1);
});

group('slopes and freqRange edge cases', function () {
  assert.deepStrictEqual(m.slopes({ K: 1, N: -1, factors: [{ type: 'zero', w: 1 }, { type: 'quad', w: 1, zeta: 0.5 }, { type: 'rhpzero', w: 3 }] }), { low: 20, high: 20 });
  assert.deepStrictEqual(m.slopes({ K: 1, N: 2, factors: [{ type: 'quad', w: 1, zeta: 0.5 }, { type: 'off', w: 1 }] }), { low: -40, high: -80 });
  const fr0 = m.freqRange({ K: 0.5, N: 0, factors: [], delay: 0 });
  near(fr0.wmin, 0.01, 1e-15, 'no corners wmin'); near(fr0.wmax, 100, 1e-12, 'no corners wmax');
  const frI = m.freqRange({ K: 0.01, N: 1, factors: [], delay: 0 });   // ω_gc = 0.01 → one decade inside
  near(frI.wmin, 0.001, 1e-15, 'extended wmin');
  const frC = m.freqRange({ K: 1, N: 0, factors: [{ type: 'pole', w: 0.01 }, { type: 'pole', w: 1000 }], delay: 0 });
  near(frC.wmin, 0.001, 1e-15, 'corner wmin'); near(frC.wmax, 1e4, 1e-9, 'corner wmax');
  const frL = m.freqRange({ K: 1000, N: -1, factors: [], delay: 0 });     // ω_gc = 1e-3 wants 1e-4: clamped
  near(frL.wmin, 0.001, 1e-15, 'clamped wmin');
  const frH = m.freqRange({ K: 0.01, N: 0, factors: [{ type: 'zero', w: 1000 }], delay: 0 });   // ω_gc ≈ 1e5 wants 1e6
  near(frH.wmax, 1e5, 1e-9, 'clamped wmax');
});

// ================================================================ root refinement (verifier)
// margins() refines every crossing with Illinois false position (replacing ~45-step bisection for speed). Each
// reported crossing must still sit exactly on its level: |G| = 0 dB, phase = −180° − 360°·k.
group('every reported crossing lies on its level (300 random systems, with and without delay)', function () {
  const R = rng(77);
  let worstG = 0, worstP = 0;
  for (let t = 0; t < 300; t++) {
    const s = randomSys(R);
    const mg = m.margins(s);
    mg.gainCrossovers.forEach(function (w) {
      const d = Math.abs(m.response(s, w).magDb);
      worstG = Math.max(worstG, d);
      assert.ok(d < 1e-9, 'gain crossover off 0 dB by ' + d + ' (#' + t + ')');
    });
    mg.phaseCrossovers.forEach(function (w) {
      const p = m.response(s, w).phaseDeg, k = Math.round((-180 - p) / 360);
      const d = Math.abs(p + 180 + 360 * k);
      worstP = Math.max(worstP, d / Math.max(1, Math.abs(p)));
      assert.ok(k >= 0 && d < 1e-9 * Math.max(1, Math.abs(p)), 'phase crossover off −180° − 360°k by ' + d + ' (#' + t + ')');
    });
    if (isFinite(mg.wgc)) near(mg.pmDeg, 180 + m.response(s, mg.wgc).phaseDeg, 1e-9, 'PM at ω_gc');
    if (isFinite(mg.wpc)) near(mg.gmDb, -m.response(s, mg.wpc).magDb, 1e-9, 'GM at ω_pc');
  }
  return 'worst ' + worstG.toExponential(1) + ' dB, ' + worstP.toExponential(1) + ' (relative phase)';
});

group('margins() for the heaviest delay case stays fast (guard against slow root refinement)', function () {
  const heavy = { K: 1000, N: -1, delay: 2, factors: [{ type: 'quad', w: 0.01, zeta: 0.05 }, { type: 'quad', w: 1000, zeta: 0.05 }, { type: 'rhpzero', w: 3 }, { type: 'zero', w: 30 }] };
  for (let i = 0; i < 10; i++) m.margins(heavy);
  const t0 = process.hrtime.bigint();
  for (let i = 0; i < 20; i++) m.margins(Object.assign({}, heavy, { K: 1000 - i }));
  const ms = Number(process.hrtime.bigint() - t0) / 20e6;
  assert.ok(ms < 30, 'margins() took ' + ms.toFixed(2) + ' ms');
  return ms.toFixed(2) + ' ms per call';
});

if (failures) { console.log(failures + ' group(s) FAILED'); process.exit(1); }
console.log('all groups passed');
