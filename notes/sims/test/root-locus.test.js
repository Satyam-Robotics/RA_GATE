// Model tests for root-locus. Run: node notes/sims/test/root-locus.test.js
'use strict';
const assert = require('assert');
const RASim = require('./load.js')('root-locus');
const ID = 'root-locus';
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
const r = (re, im) => ({ re: re, im: im || 0 });
const cabs = z => Math.hypot(z.re, z.im);

const PRESETS = {
  'p2': [[r(0), r(-2)], []],
  'p3': [[r(0), r(-1), r(-2)], []],
  'p2z': [[r(0), r(-2)], [r(-3)]],
  'p3c': [[r(0), r(-2, 2), r(-2, -2)], []],
  'p00z': [[r(0), r(0)], [r(-1)]],
  'p4': [[r(-1), r(-2), r(-3), r(-4)], []],
  'p3zz': [[r(0), r(-1), r(-2)], [r(-1, 1), r(-1, -1)]]
};

// random configuration on the 0.1 grid (as the panel makes them): n poles (1…6), m ≤ n zeros, conjugate pairs
function randomConfig(R) {
  function grid(v) { return Math.round(v * 10) / 10; }
  function roots(count) {
    const out = [];
    while (out.length < count) {
      if (count - out.length >= 2 && R() < 0.4) {
        const re = grid(-5 + 6 * R()), im = Math.max(0.1, grid(0.1 + 4 * R()));
        out.push(r(re, im), r(re, -im));
      } else out.push(r(grid(-6 + 7 * R())));
    }
    return out;
  }
  const n = 1 + Math.floor(R() * 6);
  const mm = Math.floor(R() * (n + 1));
  return [roots(n), roots(mm)];
}

function sortedCl(list) { return list.slice().sort((a, b) => (b.re - a.re) || (b.im - a.im)); }
function relResidual(poles, zeros, s, K) {
  const P = m.polys(poles, zeros);
  const len = P.den.length, num = new Array(len - P.num.length).fill(0).concat(P.num);
  const c = P.den.map((d, i) => d + K * num[i]);
  const v = M.polyEval(c, s);
  let mag = 0, pw = 1;
  const rr = cabs(s);
  for (let i = c.length - 1; i >= 0; i--) { mag += Math.abs(c[i]) * pw; pw *= rr; }
  return cabs(v) / Math.max(mag, 1e-300);
}
function viewHalf(v) { return Math.max(v.xmax - v.xmin, v.ymax - v.ymin) / 2; }

group('API shape and title', function () {
  assert.strictEqual(RASim.sims[ID].title, 'Root locus explorer');
  ['polys', 'closedLoopPoles', 'asymptotes', 'breakPoints', 'jwCrossings', 'stableRanges', 'departureAngles',
    'arrivalAngles', 'locus', 'kAt', 'dominant'].forEach(k => assert.strictEqual(typeof m[k], 'function', 'model.' + k));
  const [p, z] = PRESETS.p2z;
  const P = m.polys(p, z);
  assert.deepStrictEqual(P.num, [1, 3]);
  assert.deepStrictEqual(P.den, [1, 2, 0]);
  const a = m.asymptotes([r(-1)], [r(-2)]);
  assert.strictEqual(a.count, 0); assert.deepStrictEqual(a.anglesDeg, []); assert.ok(Number.isNaN(a.centroid));
  const cl = m.closedLoopPoles(p, z, 1);
  assert.ok(Array.isArray(cl) && cl.length === 2 && 're' in cl[0] && 'im' in cl[0]);
  const L = m.locus(p, z);
  assert.ok(Array.isArray(L) && L.length === 2 && ['re', 'im', 'K'].every(k => k in L[0][0]));
  const d = m.dominant(cl);
  assert.ok(d && 'zeta' in d && 'wn' in d);
  assert.strictEqual(m.dominant([r(1), r(0, 2), r(0, -2)]), null, 'no pole in the open LHP -> null');
});

group('1/[s(s+1)(s+2)]: asymptotes, break point, jω crossing, stability', function () {
  const [p, z] = PRESETS.p3;
  const a = m.asymptotes(p, z);
  assert.strictEqual(a.count, 3);
  [-60, 60, 180].forEach((v, i) => near(a.anglesDeg[i], v, 1e-9, 'angle ' + i));
  printed(a.centroid, -1, 5, 'centroid');
  const b = m.breakPoints(p, z);
  assert.strictEqual(b.length, 1, 'exactly one break point (−1.57735 has K < 0), got ' + JSON.stringify(b));
  printed(b[0].s.re, -0.42265, 5, 'break s'); near(b[0].s.im, 0, 1e-12, 'break im');
  printed(b[0].K, 0.38490, 5, 'break K');
  // the excluded stationary point really has negative K
  const s2 = -1 - 1 / Math.sqrt(3);
  assert.ok(-(s2 * (s2 + 1) * (s2 + 2)) < 0, 'K(−1.57735) < 0');
  const c = m.jwCrossings(p, z);
  assert.strictEqual(c.length, 1, JSON.stringify(c));
  printed(c[0].K, 6, 5, 'crossing K'); printed(c[0].w, 1.41421, 5, 'crossing ω');
  const sr = m.stableRanges(p, z);
  assert.strictEqual(sr.length, 1); assert.strictEqual(sr[0][0], 0); printed(sr[0][1], 6, 5, 'K upper');
  const cl = sortedCl(m.closedLoopPoles(p, z, 6));
  near(cl[0].re, 0, 1e-6, 'Re'); printed(cl[0].im, 1.41421, 5, '+jω');
  near(cl[1].re, 0, 1e-6, 'Re'); printed(cl[1].im, -1.41421, 5, '−jω');
  printed(cl[2].re, -3, 5, 'real pole'); near(cl[2].im, 0, 1e-9, 'real pole im');
  assert.strictEqual(m.stability(cl), 'marginal');
  assert.strictEqual(m.stability(m.closedLoopPoles(p, z, 8)), 'unstable');
  assert.strictEqual(m.stability(m.closedLoopPoles(p, z, 1)), 'stable');
  // Routh for s³ + 3s² + 2s + K: s¹ row (6 − K)/3 → K = 6; auxiliary 3s² + 6 = 0 → ω = √2
  near(c[0].K, 6, 1e-6, 'Routh K'); near(c[0].w, Math.SQRT2, 1e-6, 'Routh ω');
});

group('1/[s(s+2)]: break-away, asymptotes, all K stable, dominant pole', function () {
  const [p, z] = PRESETS.p2;
  const b = m.breakPoints(p, z);
  assert.strictEqual(b.length, 1);
  printed(b[0].s.re, -1, 5, 'break s'); printed(b[0].K, 1, 5, 'break K');
  const a = m.asymptotes(p, z);
  assert.deepStrictEqual(a.anglesDeg.map(v => Math.round(v * 1e9) / 1e9), [-90, 90]);
  printed(a.centroid, -1, 5, 'centroid');
  assert.deepStrictEqual(m.jwCrossings(p, z), []);
  assert.deepStrictEqual(m.stableRanges(p, z), [[0, Infinity]]);
  const cl = sortedCl(m.closedLoopPoles(p, z, 5));
  printed(cl[0].re, -1, 5, 'Re'); printed(cl[0].im, 2, 5, 'Im'); printed(cl[1].im, -2, 5, 'Im conj');
  const d = m.dominant(cl);
  printed(d.zeta, 0.44721, 5, 'ζ'); printed(d.wn, 2.23607, 5, 'ωn');
});

group('(s+3)/[s(s+2)]: break-away and break-in, circle |s + 3| = √3', function () {
  const [p, z] = PRESETS.p2z;
  const b = m.breakPoints(p, z);
  assert.strictEqual(b.length, 2, JSON.stringify(b));
  printed(b[0].s.re, -1.26795, 5, 'break-away s'); printed(b[0].K, 0.53590, 5, 'break-away K');
  printed(b[1].s.re, -4.73205, 5, 'break-in s'); printed(b[1].K, 7.46410, 5, 'break-in K');
  const a = m.asymptotes(p, z);
  assert.strictEqual(a.count, 1); near(a.anglesDeg[0], 180, 1e-12, 'angle');
  assert.deepStrictEqual(m.stableRanges(p, z), [[0, Infinity]]);
  let nc = 0;
  m.locus(p, z).forEach(br => br.forEach(q => {
    if (Math.abs(q.im) > 1e-9) { nc++; near(Math.hypot(q.re + 3, q.im), 1.73205, 1e-3, 'circle radius at K = ' + q.K); }
  }));
  assert.ok(nc > 100, 'complex samples: ' + nc);
  return nc + ' complex samples on the circle';
});

group('1/[s(s²+4s+8)]: departure −45°, crossing K = 32', function () {
  const [p, z] = PRESETS.p3c;
  const a = m.asymptotes(p, z);
  [-60, 60, 180].forEach((v, i) => near(a.anglesDeg[i], v, 1e-9, 'angle ' + i));
  printed(a.centroid, -1.33333, 5, 'centroid');
  const d = m.departureAngles(p, z);
  assert.strictEqual(d.length, 1);
  near(d[0].at.re, -2, 0, 'at re'); near(d[0].at.im, 2, 0, 'at im');
  printed(d[0].deg, -45, 5, 'departure');
  const c = m.jwCrossings(p, z);
  assert.strictEqual(c.length, 1);
  printed(c[0].K, 32, 5, 'K'); printed(c[0].w, 2.82843, 5, 'ω');
  const sr = m.stableRanges(p, z);
  assert.strictEqual(sr.length, 1); assert.strictEqual(sr[0][0], 0); printed(sr[0][1], 32, 5, 'K upper');
  assert.deepStrictEqual(m.breakPoints(p, z), []);
  assert.deepStrictEqual(m.arrivalAngles(p, z), []);
});

group('(s+1)/s²: break-in at −2, circle |s + 1| = 1', function () {
  const [p, z] = PRESETS.p00z;
  const b = m.breakPoints(p, z);
  assert.strictEqual(b.length, 1, JSON.stringify(b));
  printed(b[0].s.re, -2, 5, 'break s'); printed(b[0].K, 4, 5, 'break K');
  assert.deepStrictEqual(m.stableRanges(p, z), [[0, Infinity]]);
  const cl = sortedCl(m.closedLoopPoles(p, z, 2));
  printed(cl[0].re, -1, 5, 'Re'); printed(cl[0].im, 1, 5, 'Im'); printed(cl[1].im, -1, 5, 'Im conj');
  let nc = 0;
  m.locus(p, z).forEach(br => br.forEach(q => {
    if (Math.abs(q.im) > 1e-9) { nc++; near(Math.hypot(q.re + 1, q.im), 1, 1e-3, 'circle radius at K = ' + q.K); }
  }));
  assert.ok(nc > 100, 'complex samples: ' + nc);
  return nc + ' complex samples';
});

group('1/[(s+1)(s+2)(s+3)(s+4)]: asymptotes and the Routh marginal gain', function () {
  const [p, z] = PRESETS.p4;
  const a = m.asymptotes(p, z);
  [-135, -45, 45, 135].forEach((v, i) => near(a.anglesDeg[i], v, 1e-9, 'angle ' + i));
  printed(a.centroid, -2.5, 5, 'centroid');
  // Routh array of s⁴ + 10s³ + 35s² + 50s + (24 + K):
  //   s⁴: 1, 35, 24 + K;  s³: 10, 50;  s²: (10·35 − 50)/10 = 30, 24 + K;  s¹: (30·50 − 10(24 + K))/30 = (1260 − 10K)/30
  //   → K_u = 126; auxiliary 30s² + (24 + 126) = 0 → ω² = 5, ω = √5 = 2.23607
  const b1 = 10, b2 = 50, a0 = 1, a2 = 35;
  const c1 = (b1 * a2 - a0 * b2) / b1;                // 30
  const Ku = (c1 * b2 / b1) - 24;                     // s¹ row zero: c1·50 − 10(24 + K) = 0
  const wu = Math.sqrt((24 + Ku) / c1);
  near(c1, 30, 1e-12, 'Routh s² row'); near(Ku, 126, 1e-12, 'Routh K'); near(wu, Math.sqrt(5), 1e-12, 'Routh ω');
  const c = m.jwCrossings(p, z);
  assert.strictEqual(c.length, 1, JSON.stringify(c));
  printed(c[0].K, Ku, 5, 'crossing K'); printed(c[0].w, wu, 5, 'crossing ω');
  const sr = m.stableRanges(p, z);
  assert.strictEqual(sr.length, 1); assert.strictEqual(sr[0][0], 0); printed(sr[0][1], Ku, 5, 'K upper');
  const b = m.breakPoints(p, z);       // (−5 ± √5)/2 with K = 1; −2.5 has K < 0
  assert.strictEqual(b.length, 2, JSON.stringify(b));
  printed(b[0].s.re, (-5 + Math.sqrt(5)) / 2, 5, 'break 1'); printed(b[0].K, 1, 5, 'K 1');
  printed(b[1].s.re, (-5 - Math.sqrt(5)) / 2, 5, 'break 2'); printed(b[1].K, 1, 5, 'K 2');
});

group('(s²+2s+2)/[s(s+1)(s+2)]: arrival angle', function () {
  const [p, z] = PRESETS.p3zz;
  const ar = m.arrivalAngles(p, z);
  assert.strictEqual(ar.length, 1);
  // 180 + (135 + 90 + 45) − 90 = 360 → 0°
  near(ar[0].deg, 0, 1e-9, 'arrival');
  const a = m.asymptotes(p, z);
  assert.strictEqual(a.count, 1); printed(a.centroid, -1, 5, 'centroid');
});

// ---------------------------------------------------------------- invariants
function checkLocus(p, z, label) {
  const v = m.view(p, z);
  const half = viewHalf(v), maxMove = half / 100, cx = (v.xmin + v.xmax) / 2;
  const L = m.locus(p, z);
  assert.strictEqual(L.length, p.length, label + ': branch count');
  const starts = p.map(q => ({ re: q.re, im: q.im, used: false }));
  let pts = 0, worst = 0;
  L.forEach((br, bi) => {
    const s0 = br[0];
    assert.strictEqual(s0.K, 0, label + ': starts at K = 0');
    const st = starts.find(q => !q.used && q.re === s0.re && q.im === s0.im);
    assert.ok(st, label + ': branch ' + bi + ' starts at an open-loop pole');
    st.used = true;
    for (let i = 0; i < br.length; i++) {
      const q = br[i];
      pts++;
      if (i > 0) {
        assert.ok(q.K >= br[i - 1].K, label + ': K non-decreasing');
        const inV = Math.hypot(q.re - cx, q.im) < 2.5 * half || Math.hypot(br[i - 1].re - cx, br[i - 1].im) < 2.5 * half;
        if (inV) {
          const d = Math.hypot(q.re - br[i - 1].re, q.im - br[i - 1].im);
          assert.ok(d <= maxMove * (1 + 1e-9), label + ': continuity, step ' + d + ' > ' + maxMove + ' at K = ' + q.K);
        }
        const res = relResidual(p, z, q, q.K);
        worst = Math.max(worst, res);
        assert.ok(res <= 1e-6, label + ': residual ' + res + ' at K = ' + q.K);
        if (q.K > 0) {
          const kk = m.kAt(p, z, q);
          assert.ok(Math.abs(kk - q.K) <= 1e-4 * q.K, label + ': kAt ' + kk + ' vs K ' + q.K);
        }
      }
    }
  });
  // every branch has the same K samples
  L.forEach(br => assert.strictEqual(br.length, L[0].length, label + ': equal sample counts'));
  // termination: m branches end at the zeros, n − m have left the view
  let atZero = 0, out = 0;
  L.forEach(br => {
    const q = br[br.length - 1];
    if (z.some(zz => Math.hypot(q.re - zz.re, q.im - zz.im) < 0.2 * maxMove)) atZero++;
    else if (q.re < v.xmin || q.re > v.xmax || q.im < v.ymin || q.im > v.ymax) out++;
  });
  assert.strictEqual(atZero, z.length, label + ': branches ending at zeros');
  assert.strictEqual(out, p.length - z.length, label + ': branches leaving the view');
  return { pts: pts, worst: worst, L: L };
}

group('locus invariants on every preset (residual, continuity, starts, kAt, termination)', function () {
  let worst = 0, pts = 0;
  for (const k in PRESETS) { const res = checkLocus(PRESETS[k][0], PRESETS[k][1], k); worst = Math.max(worst, res.worst); pts += res.pts; }
  return pts + ' samples, worst relative residual ' + worst.toExponential(1);
});

group('branches never swap at break points (conjugate after the break-away)', function () {
  ['p3', 'p2z'].forEach(k => {
    const [p, z] = PRESETS[k];
    const L = m.locus(p, z);
    const bK = m.breakPoints(p, z)[0].K;
    // the two branches that start at 0 and the pole next to it
    const i0 = L.findIndex(br => br[0].re === 0), i1 = L.findIndex(br => br[0].re === (k === 'p3' ? -1 : -2));
    const A = L[i0], B = L[i1];
    const end = k === 'p2z' ? m.breakPoints(p, z)[1].K : Infinity;
    let checked = 0, sideA = 0;
    for (let i = 0; i < A.length; i++) {
      if (A[i].K <= bK * (1 + 1e-6) || A[i].K >= end * (1 - 1e-6)) continue;
      assert.strictEqual(A[i].K, B[i].K);
      if (Math.abs(A[i].im) < 1e-6) continue;
      near(A[i].re, B[i].re, 1e-9, k + ' conj re at K = ' + A[i].K);
      near(A[i].im, -B[i].im, 1e-9, k + ' conj im at K = ' + A[i].K);
      const sg = Math.sign(A[i].im);
      if (!sideA) sideA = sg;
      assert.strictEqual(sg, sideA, k + ': branch changed half-plane (swap) at K = ' + A[i].K);
      checked++;
    }
    assert.ok(checked > 50, k + ': checked ' + checked);
  });
});

group('locus invariants on 60 random configurations (seeded)', function () {
  const R = rng(20270101);
  let pts = 0, worst = 0;
  for (let t = 0; t < 60; t++) {
    const [p, z] = randomConfig(R);
    const res = checkLocus(p, z, 'random #' + t + ' ' + JSON.stringify([p, z]));
    pts += res.pts; worst = Math.max(worst, res.worst);
  }
  return pts + ' samples, worst residual ' + worst.toExponential(1);
});

group('stableRanges agrees with a brute-force scan (presets and random sets)', function () {
  const R = rng(777);
  const sets = Object.keys(PRESETS).map(k => PRESETS[k]);
  for (let t = 0; t < 40; t++) sets.push(randomConfig(R));
  let checked = 0;
  sets.forEach((cfg, si) => {
    const [p, z] = cfg;
    const sr = m.stableRanges(p, z);
    const bounds = [];
    sr.forEach(iv => { bounds.push(iv[0], iv[1]); });
    m.jwCrossings(p, z).forEach(c => bounds.push(c.K));
    M.logspace(1e-3, 1e5, 1500).forEach(K => {
      if (bounds.some(b => b > 0 && isFinite(b) && Math.abs(K - b) < 2e-3 * b)) return;
      const cl = m.closedLoopPoles(p, z, K);
      const mx = Math.max.apply(null, cl.map(q => q.re));
      const scale = Math.max(1e-9, Math.max.apply(null, cl.map(cabs)));
      if (Math.abs(mx) < 1e-6 * scale) return;           // on the axis for this K: not decidable by sign
      const inside = sr.some(iv => K > iv[0] && K < iv[1]);
      assert.strictEqual(inside, mx < 0, 'set ' + si + ' ' + JSON.stringify(cfg) + ' K = ' + K + ' max Re = ' + mx + ' ranges ' + JSON.stringify(sr));
      checked++;
    });
  });
  return checked + ' (set, K) pairs';
});

group('jω crossings: a closed-loop pole sits on the axis at each reported K (random sets)', function () {
  const R = rng(4242);
  let n = 0;
  for (let t = 0; t < 80; t++) {
    const [p, z] = randomConfig(R);
    m.jwCrossings(p, z).forEach(c => {
      assert.ok(c.K > 0 && c.w >= 0);
      const cl = m.closedLoopPoles(p, z, c.K);
      const scale = Math.max(1, Math.max.apply(null, cl.map(cabs)));
      const hit = cl.some(q => Math.abs(q.re) < 1e-5 * scale && Math.abs(Math.abs(q.im) - c.w) < 1e-5 * scale);
      assert.ok(hit, JSON.stringify([p, z, c, cl]));
      n++;
    });
  }
  return n + ' crossings verified';
});

group('asymptote angles and centroid for random sets', function () {
  const R = rng(99);
  for (let t = 0; t < 200; t++) {
    const [p, z] = randomConfig(R);
    const a = m.asymptotes(p, z);
    const q = p.length - z.length;
    assert.strictEqual(a.count, q);
    if (q === 0) { assert.deepStrictEqual(a.anglesDeg, []); assert.ok(Number.isNaN(a.centroid)); continue; }
    const exp = [];
    for (let k = 0; k < q; k++) { let ang = (2 * k + 1) * 180 / q; while (ang > 180) ang -= 360; exp.push(ang); }
    exp.sort((x, y) => x - y);
    assert.strictEqual(a.anglesDeg.length, q);
    exp.forEach((e, i) => near(a.anglesDeg[i], e, 1e-9, 'angle'));
    a.anglesDeg.forEach(v => assert.ok(v > -180 && v <= 180));
    const sp = p.reduce((s, x) => s + x.re, 0), sz = z.reduce((s, x) => s + x.re, 0);
    near(a.centroid, (sp - sz) / q, 1e-12, 'centroid');
  }
});

group('break points: on the locus (K > 0, angle condition), negative-K points excluded', function () {
  const R = rng(31337);
  let n = 0;
  for (let t = 0; t < 80; t++) {
    const [p, z] = randomConfig(R);
    const bs = m.breakPoints(p, z);
    for (let i = 1; i < bs.length; i++) assert.ok(bs[i - 1].s.re >= bs[i].s.re, 'sorted by descending re');
    bs.forEach(b => {
      assert.ok(b.K > 1e-9);
      // the closed loop has a repeated root at s for this K
      const cl = m.closedLoopPoles(p, z, b.K);
      const close = cl.filter(q => Math.hypot(q.re - b.s.re, q.im - b.s.im) < 2e-3 * Math.max(1, cabs(b.s)));
      assert.ok(close.length >= 2, 'double root at the break point ' + JSON.stringify([p, z, b, cl]));
      n++;
    });
  }
  return n + ' break points verified';
});

group('departure / arrival angles satisfy the angle condition (random sets)', function () {
  const R = rng(55);
  let n = 0;
  for (let t = 0; t < 120; t++) {
    const [p, z] = randomConfig(R);
    [['dep', m.departureAngles(p, z), p], ['arr', m.arrivalAngles(p, z), z]].forEach(item => {
      const list = item[1], own = item[2];
      assert.strictEqual(list.length, own.filter(q => q.im > 0).length);
      list.forEach(a => {
        if (!isFinite(a.deg)) return;
        assert.ok(a.deg > -180 && a.deg <= 180);
        const mult = own.filter(q => q.re === a.at.re && q.im === a.at.im).length;
        // a point a small distance away in that direction: ∠G(s) = ±180° (on the locus, K > 0)
        const eps = 1e-5;
        const s = { re: a.at.re + eps * Math.cos(a.deg * Math.PI / 180), im: a.at.im + eps * Math.sin(a.deg * Math.PI / 180) };
        const P = m.polys(p, z);
        const G = M.tfEval(P.num, P.den, s);
        const ang = Math.abs(M.wrapDeg(M.deg(M.carg(G))));
        assert.ok(Math.abs(ang - 180) < 1e-2 * mult + 1e-3, item[0] + ' angle ' + a.deg + ' gives ∠G = ' + ang);
        n++;
      });
    });
  }
  // the 1/[s(s²+4s+8)] locus really leaves −2 + j2 at −45°
  const [p, z] = PRESETS.p3c;
  const L = m.locus(p, z);
  const br = L.find(b => b[0].re === -2 && b[0].im === 2);
  const q = br.find(x => Math.hypot(x.re + 2, x.im - 2) > 0.02);
  near(M.deg(Math.atan2(q.im - 2, q.re + 2)), -45, 1.5, 'traced departure');
  return n + ' angles verified';
});

group('kAt (magnitude condition) and dominant', function () {
  const [p, z] = PRESETS.p3;
  const cl = m.closedLoopPoles(p, z, 2.5);
  cl.forEach(q => near(m.kAt(p, z, q), 2.5, 1e-8, 'kAt'));
  near(m.kAt(PRESETS.p2z[0], PRESETS.p2z[1], r(-1, 1)), Math.sqrt(2) * Math.sqrt(2) / Math.sqrt(5), 1e-12, 'kAt complex');
  const d = m.dominant([r(-3), r(-0.5, 2), r(-0.5, -2)]);
  near(d.wn, Math.hypot(0.5, 2), 1e-12, 'ωn'); near(d.zeta, 0.5 / Math.hypot(0.5, 2), 1e-12, 'ζ');
  const d2 = m.dominant([r(-1), r(-4)]);
  near(d2.zeta, 1, 0, 'real ζ'); near(d2.wn, 1, 0, 'real ωn');
  assert.strictEqual(m.dominant([r(0, 1), r(0, -1)]), null);
  // closed-loop poles at K = 0 are the open-loop poles
  const c0 = sortedCl(m.closedLoopPoles(PRESETS.p3c[0], [], 0));
  near(c0[1].re, -2, 1e-12, 'K = 0'); near(c0[1].im, 2, 1e-12, 'K = 0');
});

group('step response: window and final value', function () {
  const [p, z] = PRESETS.p2;
  const s = m.stepResponse(p, z, 5);
  // poles −1 ± j2: σ_min = 1 → 6 s; T(0) = 5/5 = 1
  near(s.tEnd, 6, 1e-9, 'window'); near(s.final, 1, 1e-12, 'final'); near(s.y[s.y.length - 1], 1, 0.02, 'settled');
  const u = m.stepResponse(PRESETS.p3[0], [], 8);
  assert.strictEqual(u.kind, 'unstable'); assert.ok(Number.isNaN(u.final));
  const cl = m.closedLoopPoles(PRESETS.p3[0], [], 8);
  const wmax = Math.max.apply(null, cl.filter(q => q.re >= 0).map(q => Math.abs(q.im)));
  near(u.tEnd, M.clamp(10 * Math.PI / wmax, 2, 30), 1e-9, 'unstable window');
  near(m.stepWindow([r(-100)]), 2, 0, 'clamp low'); near(m.stepWindow([r(-0.01)]), 60, 0, 'clamp high');
  near(m.stepWindow([r(0), r(-1)]), 10, 0, 'ω_max = 0');
  const z2 = m.stepResponse(PRESETS.p2z[0], PRESETS.p2z[1], 2);
  near(z2.final, 1, 1e-12, 'type-1 final value');
});

// ---------------------------------------------------------------------------------------------
// Browser regression tests (need Playwright; skipped with a message when it is not available or
// when run with --no-browser). They guard display bugs found in verification.
// ---------------------------------------------------------------------------------------------
async function agroup(name, fn) {
  try { const extra = await fn(); console.log('ok   ' + name + (extra ? '  (' + extra + ')' : '')); }
  catch (e) { failures++; console.log('FAIL ' + name + ': ' + (e && e.message || e)); }
}
async function browserTests() {
  let playwrightOk = true;
  try { require.resolve('playwright'); } catch (e) { playwrightOk = false; }
  if (!playwrightOk || process.argv.indexOf('--no-browser') >= 0) { console.log('skip browser regression tests'); return; }
  const path = require('path'), os = require('os');
  const { openSim } = require('./harness.js');
  const outDir = path.join(os.tmpdir(), 'rasim-root-locus-test');
  const P = k => ({ poles: PRESETS[k][0], zeros: PRESETS[k][1] });
  for (const width of [390, 1100]) {
    const s = await openSim(ID, { width: width, outDir: outDir });
    const page = s.page;
    const frames = () => page.evaluate(() => new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r))));
    const setS = async function (st) { await page.evaluate(st => RASim.instances['root-locus'].setState(st), st); await frames(); };
    const W = ' @' + width + ' px';
    // colour distance of the canvas pixel at data point (re, im) to a hex colour
    const colourAt = (re, im, hex) => page.evaluate(([re, im, hex]) => {
      const c = document.querySelector('[data-sim="root-locus"] [data-canvas="splane"]'), g = c.rlGeom;
      const ctx = c.getContext('2d'), dpr = c.width / c.clientWidth;
      const x = g.x + (re - g.xmin) / (g.xmax - g.xmin) * g.w, y = g.y + (g.ymax - im) / (g.ymax - g.ymin) * g.h;
      const d = ctx.getImageData(Math.round(x * dpr), Math.round(y * dpr), 1, 1).data;
      const h = [1, 3, 5].map(i => parseInt(hex.substr(i, 2), 16));
      return Math.abs(d[0] - h[0]) + Math.abs(d[1] - h[1]) + Math.abs(d[2] - h[2]);
    }, [re, im, hex]);
    try {
      await agroup('browser: break points and jω crossings stay visible when a closed-loop pole sits on them (regression)' + W, async function () {
        const purple = RASim.colors.purple, gold = RASim.colors.gold;
        await setS(Object.assign(P('p00z'), { K: 4 }));                       // closed-loop double pole at the break-in −2
        assert.ok(await colourAt(-2, 0, purple) < 60, '(s+1)/s² at K = 4: break-point diamond at −2 hidden');
        await setS(Object.assign(P('p4'), { K: 1 }));                         // both break points at K = 1
        const b = m.breakPoints(PRESETS.p4[0], []);
        for (const q of b) assert.ok(await colourAt(q.s.re, 0, purple) < 60, 'four-pole preset at K = 1: diamond at ' + q.s.re + ' hidden');
        await setS(Object.assign(P('p3'), { K: 6 }));                         // closed-loop poles on the crossings ±j√2
        for (const w of [Math.SQRT2, -Math.SQRT2]) assert.ok(await colourAt(0, w, gold) < 60, '1/[s(s+1)(s+2)] at K = 6: crossing dot at j' + w + ' hidden');
      });
      await agroup('browser: the centroid label σₐ is drawn and no canvas labels overlap in the screenshot states (regression)' + W, async function () {
        const states = [['p3', 1, '−1'], ['p3', 8, '−1'], ['p2z', 2, '1'], ['p3c', 10, '−1.333'], ['p00z', 4, '1'], ['p4', 1, '−2.5'], ['p2', 5, '−1'], ['p3zz', 1, '−1']];
        let n = 0;
        for (const [k, K, cent] of states) {
          await setS(Object.assign(P(k), { K: K }));
          const info = await page.evaluate(() => { const c = document.querySelector('[data-sim="root-locus"] [data-canvas="splane"]'); return { L: c.rlLabels, w: c.clientWidth, h: c.clientHeight }; });
          const L = info.L;
          assert.ok(L.some(l => l.text === 'σₐ = ' + cent), k + ' K = ' + K + ': centroid label σₐ = ' + cent + ' missing; labels ' + JSON.stringify(L.map(l => l.text)));
          for (let i = 0; i < L.length; i++) {
            assert.ok(L[i].l >= 0 && L[i].t >= 0 && L[i].r <= info.w && L[i].b <= info.h, k + ': label "' + L[i].text + '" leaves the canvas');
            for (let j = 0; j < i; j++) {
              const a = L[i], c = L[j];
              assert.ok(!(Math.min(a.r, c.r) - Math.max(a.l, c.l) > 0.5 && Math.min(a.b, c.b) - Math.max(a.t, c.t) > 0.5), k + ' K = ' + K + ': labels "' + a.text + '" and "' + c.text + '" overlap');
            }
          }
          n += L.length;
        }
        return n + ' labels';
      });
      await agroup('browser: the step-response caption tells oscillation from a ramp (pole at s = 0) (regression)' + W, async function () {
        const cap = () => page.evaluate(() => document.querySelector('[data-sim="root-locus"] [data-canvas="step"]').rlCaption);
        await setS({ poles: [r(1), r(-2), r(-3)], zeros: [r(-1)], K: 6 });     // closed loop s(s² + 4s + 7): pole at the origin
        assert.strictEqual(await page.textContent('[data-sim="root-locus"] [data-tag="stability"]'), 'MARGINALLY STABLE');
        assert.strictEqual(await cap(), 'marginally stable: y(t) ramps (pole at s = 0)');
        await setS(Object.assign(P('p3'), { K: 6 }));
        assert.strictEqual(await cap(), 'marginally stable: sustained oscillation');
        await setS({ K: 8 });
        assert.strictEqual(await cap(), 'unstable: y(t) grows without bound');
        await setS({ K: 1 });
        assert.strictEqual(await cap(), 'final value T(0) = 1');
      });
      if (width === 1100) await agroup('browser: read-outs show the check values; Reset restores every default', async function () {
        const read = () => page.evaluate(() => {
          const o = {};
          document.querySelectorAll('[data-sim="root-locus"] [data-stat]').forEach(e => { o[e.getAttribute('data-stat')] = e.querySelector('.sim-stat-value').textContent; });
          o.tag = document.querySelector('[data-sim="root-locus"] [data-tag="stability"]').textContent;
          o.tf = document.querySelector('[data-sim="root-locus"] [data-key="tf"]').textContent;
          o.charEq = document.querySelector('[data-sim="root-locus"] [data-key="charEq"]').textContent;
          return o;
        });
        const expect = [
          [Object.assign(P('p3'), { K: 1 }), { asym: '3 at −60°, 60°, 180°', centroid: '−1', breaks: '−0.4226 (K = 0.3849)', crossings: '±j1.414 at K = 6', stable: '0 < K < 6', tag: 'STABLE' }],
          [{ K: 6 }, { clPoles: '±j1.414, −3', tag: 'MARGINALLY STABLE', zeta: '—' }],
          [{ K: 8 }, { tag: 'UNSTABLE', clPoles: '0.08316 ± j1.587, −3.166' }],
          [Object.assign(P('p2'), { K: 5 }), { breaks: '−1 (K = 1)', asym: '2 at −90°, 90°', crossings: 'none', stable: 'all K > 0', clPoles: '−1 ± j2', zeta: '0.4472', wn: '2.236 rad/s' }],
          [Object.assign(P('p2z'), { K: 2 }), { breaks: '−1.268 (K = 0.5359), −4.732 (K = 7.464)', asym: '1 at 180°', stable: 'all K > 0', tf: 'Open loop: G(s) = (s + 3) / (s² + 2s)' }],
          [Object.assign(P('p3c'), { K: 10 }), { asym: '3 at −60°, 60°, 180°', centroid: '−1.333', breaks: 'none', crossings: '±j2.828 at K = 32', stable: '0 < K < 32' }],
          [Object.assign(P('p00z'), { K: 2 }), { breaks: '−2 (K = 4)', stable: 'all K > 0', clPoles: '−1 ± j1' }],
          [Object.assign(P('p4'), { K: 1 }), { asym: '4 at −135°, −45°, 45°, 135°', centroid: '−2.5', crossings: '±j2.236 at K = 126', stable: '0 < K < 126',
            charEq: '1 + K·G(s) = 0 at K = 1: s⁴ + 10s³ + 35s² + 50s + 25 = 0' }]
        ];
        for (const [st, want] of expect) {
          await setS(st);
          const got = await read();
          Object.keys(want).forEach(k => assert.strictEqual(got[k].trim(), want[k], JSON.stringify(st) + ' ' + k));
        }
        const init = { poles: PRESETS.p3[0], zeros: [], K: 1, tool: 'move', showAsymptotes: true, showBreaks: true, showAngles: true, showStep: true };
        await setS({ poles: [r(-1, 2), r(-1, -2), r(1)], zeros: [r(-0.5)], K: 77, tool: 'delete', showAsymptotes: false, showBreaks: false, showAngles: false, showStep: false });
        await page.click('[data-sim="root-locus"] [data-control="reset"]');
        await frames();
        assert.deepStrictEqual(await page.evaluate(() => RASim.instances['root-locus'].getState()), init);
      });
      assert.deepStrictEqual(s.errors, [], 'console errors');
    } finally { await s.browser.close(); }
  }
}

browserTests().then(function () {
  if (failures) { console.log(failures + ' group(s) FAILED'); process.exit(1); }
  console.log('all root-locus tests passed');
}, function (e) { console.log('FAIL browser tests: ' + (e && e.stack || e)); process.exit(1); });
