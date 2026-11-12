// Model tests for four-bar ("Four-bar linkage and Grashof's law").
// Run: node notes/sims/test/four-bar.test.js
'use strict';
const assert = require('assert');
const fileArg = process.argv.indexOf('--file') >= 0 ? process.argv[process.argv.indexOf('--file') + 1] : undefined;
const RASim = require('./load.js')('four-bar', fileArg);
const m = RASim.sims['four-bar'].model;

const D2R = Math.PI / 180, R2D = 180 / Math.PI;
let failures = 0;
function group(name, fn) {
  try { const extra = fn(); console.log('ok   ' + name + (extra ? '  (' + extra + ')' : '')); }
  catch (e) { failures++; console.log('FAIL ' + name + ': ' + (e && e.message)); }
}
// tolerance: given, else max(5e-4 relative, half a unit of the last printed digit)
function near(actual, expected, what, tol) {
  const dec = (String(expected).split('.')[1] || '').length;
  const t = tol != null ? tol : Math.max(5e-4 * Math.abs(expected), 0.5 * Math.pow(10, -dec));
  assert.ok(Math.abs(actual - expected) <= t, what + ': got ' + actual + ', expected ' + expected + ' ± ' + t);
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

group('classify check values', function () {
  let c = m.classify(2, 7, 5, 6);
  assert.strictEqual(c.assemblable, true); assert.strictEqual(c.grashof, 'grashof');
  assert.strictEqual(c.type, 'crank-rocker'); assert.strictEqual(c.shortest, 'a');
  assert.strictEqual(m.classify(2, 6, 5, 6).type, 'crank-rocker');
  c = m.classify(4, 5, 6, 2); assert.strictEqual(c.type, 'double-crank'); assert.strictEqual(c.shortest, 'd');
  c = m.classify(5, 2, 6, 4); assert.strictEqual(c.type, 'double-rocker'); assert.strictEqual(c.shortest, 'b');
  c = m.classify(5, 7, 2, 6); assert.strictEqual(c.type, 'rocker-crank'); assert.strictEqual(c.shortest, 'c');
  c = m.classify(4, 6, 5, 8); assert.strictEqual(c.grashof, 'non-grashof'); assert.strictEqual(c.type, 'triple-rocker');
  c = m.classify(3, 6, 3, 6); assert.strictEqual(c.grashof, 'change-point'); assert.strictEqual(c.type, 'change-point');
  assert.strictEqual(c.shortest, null);
  [[1, 1, 1, 5], [1, 2, 3, 6]].forEach(function (L) {
    const q = m.classify(L[0], L[1], L[2], L[3]);
    assert.strictEqual(q.assemblable, false, 'not assemblable ' + L); assert.strictEqual(q.type, 'none');
    assert.strictEqual(q.grashof, null);
  });
  c = m.classify(2.1, 4.2, 3.3, 3);
  assert.strictEqual(c.type, 'change-point', 'floating-point trap 2.1 + 4.2 = 3.3 + 3');
  assert.strictEqual(c.sumSL, 6.3); assert.strictEqual(c.sumPQ, 6.3);
  c = m.classify(2, 6, 5, 6);
  assert.strictEqual(c.s, 2); assert.strictEqual(c.l, 6); assert.strictEqual(c.sumSL, 8); assert.strictEqual(c.sumPQ, 11);
  assert.ok(/Crank–rocker/.test(c.label) && /s \+ l < p \+ q/.test(c.label), c.label);
});

group('inputRange check values', function () {
  let r = m.inputRange(2, 7, 5, 6);
  assert.strictEqual(r.full, true); assert.deepStrictEqual(r.rangesDeg, [[0, 360]]);
  r = m.inputRange(5, 2, 6, 4);
  assert.strictEqual(r.full, false); assert.strictEqual(r.rangesDeg.length, 2);
  near(r.rangesDeg[0][0], 51.318, 'lo'); near(r.rangesDeg[0][1], 125.100, 'hi');
  near(r.rangesDeg[1][0], -125.100, '-hi'); near(r.rangesDeg[1][1], -51.318, '-lo');
  r = m.inputRange(4, 6, 5, 8);
  assert.strictEqual(r.full, false); assert.strictEqual(r.rangesDeg.length, 1);
  near(r.rangesDeg[0][0], -129.838, 'lo'); near(r.rangesDeg[0][1], 129.838, 'hi');
  assert.strictEqual(m.inputRange(3, 6, 3, 6).full, true, 'parallelogram counts as full rotation');
  assert.strictEqual(m.inputRange(2.1, 4.2, 3.3, 3).full, true, 'change-point with a shortest: full');
});

group('solve check values', function () {
  const s = m.solve(2, 7, 5, 6, 90 * D2R, +1);
  near(s.A[0], 0, 'Ax', 1e-12); near(s.A[1], 2, 'Ay', 1e-12);
  near(s.B[0], 6.3297, 'Bx'); near(s.B[1], 4.9891, 'By');
  near(s.theta3 * R2D, 25.278, 'theta3'); near(s.theta4 * R2D, 86.219, 'theta4');
  near(s.mu * R2D, 60.941, 'mu');
  near(s.w4, 0.41379, 'w4'); near(s.w3, 0.02155, 'w3');
  // I24 consistent with w4
  near(s.w4, s.I24[0] / (s.I24[0] - 6), 'I24 ratio', 1e-9);
  assert.strictEqual(m.solve(4, 6, 5, 8, 180 * D2R, 1), null, 'cannot close');
  assert.strictEqual(m.solve(1, 1, 1, 5, 0, 1), null);
  // crossed is the mirror image of open at −θ₂
  const o = m.solve(2, 7, 5, 6, -40 * D2R, 1), x = m.solve(2, 7, 5, 6, 40 * D2R, -1);
  near(x.B[0], o.B[0], 'mirror Bx', 1e-12); near(x.B[1], -o.B[1], 'mirror By', 1e-12);
  // string mode alias
  near(m.solve(2, 7, 5, 6, 1, 'crossed').B[1], x && m.solve(2, 7, 5, 6, 1, -1).B[1], 'mode alias', 0);
});

group('couplerPoint', function () {
  const P = m.couplerPoint([1, 1], [4, 5], 0.5, 2);       // |AB| = 5, ê = (0.6, 0.8), n̂ = (−0.8, 0.6)
  near(P[0], 1 + 1.5 - 1.6, 'Px', 1e-12); near(P[1], 1 + 2 + 1.2, 'Py', 1e-12);
  const Q = m.couplerPoint([0, 0], [2, 0], 1.5, -1);
  near(Q[0], 3, 'Qx', 1e-12); near(Q[1], -1, 'Qy', 1e-12);
});

group('rockerLimits check values', function () {
  const r = m.rockerLimits(2, 7, 5, 6);
  near(r.theta4MinDeg, 70.529, 'theta4 min'); near(r.theta4MaxDeg, 126.870, 'theta4 max');
  near(r.swingDeg, 56.341, 'swing');
  near(r.theta2ExtendedDeg, 31.586, 'extended'); near(r.theta2FoldedDeg, 233.130, 'folded');
  near(r.forwardDeg, 201.544, 'forward'); near(r.returnDeg, 158.456, 'return');
  near(r.timeRatio, 1.2719, 'time ratio');
  assert.strictEqual(m.rockerLimits(4, 5, 6, 2), null); assert.strictEqual(m.rockerLimits(4, 6, 5, 8), null);
  assert.strictEqual(m.rockerLimits(5, 7, 2, 6), null);
  // at the limits the crank and coupler are collinear: ω₄ = 0
  const se = m.solve(2, 7, 5, 6, r.theta2ExtendedDeg * D2R, 1), sf = m.solve(2, 7, 5, 6, r.theta2FoldedDeg * D2R, 1);
  near(se.theta4 * R2D, 70.529, 'theta4 at extended'); near(sf.theta4 * R2D, 126.870, 'theta4 at folded');
  assert.ok(Math.abs(se.w4) < 1e-9 && Math.abs(sf.w4) < 1e-9, 'w4 = 0 at the rocker limits');
});

group('transmissionRange check values', function () {
  let t = m.transmissionRange(2, 7, 5, 6);
  near(t.minDeg, 34.048, 'min poor'); near(t.maxDeg, 81.787, 'max poor');
  t = m.transmissionRange(2, 6, 5, 6);
  near(t.minDeg, 41.410, 'min'); near(t.maxDeg, 92.866, 'max');
});

group('parallelogram (3, 6, 3, 6) open', function () {
  for (let t = 0.5; t < 180; t += 0.5) {
    const s = m.solve(3, 6, 3, 6, t * D2R, 1);
    near(s.theta4 * R2D, t, 'theta4 at ' + t, 1e-7);
    near(s.theta3 * R2D, 0, 'theta3 at ' + t, 1e-7);
    near(s.w4, 1, 'w4 at ' + t, 1e-7);
  }
});

group('couplerCurve', function () {
  const cc = m.couplerCurve(2, 7, 5, 6, 1, 0.5, 2, 360);
  assert.strictEqual(cc.length, 1); assert.strictEqual(cc[0].length, 361);
  const s = m.solve(2, 7, 5, 6, 0, 1), P = m.couplerPoint(s.A, s.B, 0.5, 2);
  near(cc[0][0][0], P[0], 'first x', 1e-12); near(cc[0][0][1], P[1], 'first y', 1e-12);
  const c2 = m.couplerCurve(5, 2, 6, 4, -1, 0.3, -1);
  assert.strictEqual(c2.length, 2);
  c2.forEach(function (line) { assert.ok(line.length > 10); line.forEach(function (q) { assert.ok(isFinite(q[0]) && isFinite(q[1])); }); });
  return 'ranges ' + c2.length;
});

// ---------------------------------------------------------------- invariants
function randomLinkage(R, tenthsOnly) {
  for (;;) {
    const L = [0, 0, 0, 0].map(function () { const x = 1 + 9 * R(); return tenthsOnly ? Math.round(10 * x) / 10 : x; });
    const c = m.classify(L[0], L[1], L[2], L[3]);
    if (c.assemblable) return L;
  }
}
function randomValidTheta(R, L) {
  const ir = m.inputRange(L[0], L[1], L[2], L[3]);
  const r = ir.rangesDeg[Math.floor(R() * ir.rangesDeg.length)];
  return (r[0] + (r[1] - r[0]) * (0.02 + 0.96 * R())) * D2R;
}

group('invariants: closure, lengths, mu, I24, I13 (500 random linkages, both modes)', function () {
  const R = rng(20271);
  let n = 0;
  for (let i = 0; i < 500; i++) {
    const L = randomLinkage(R, false), a = L[0], b = L[1], c = L[2], d = L[3];
    const t2 = randomValidTheta(R, L);
    [1, -1].forEach(function (mode) {
      const s = m.solve(a, b, c, d, t2, mode);
      assert.ok(s, 'solvable inside the input range ' + L + ' ' + t2 * R2D);
      near(Math.hypot(s.A[0], s.A[1]), a, '|A|', 1e-9);
      near(Math.hypot(s.B[0] - s.A[0], s.B[1] - s.A[1]), b, '|B − A|', 1e-9);
      near(Math.hypot(s.B[0] - d, s.B[1]), c, '|B − O4|', 1e-9);
      // loop closure a e^{jθ₂} + b e^{jθ₃} = d + c e^{jθ₄}
      near(a * Math.cos(t2) + b * Math.cos(s.theta3), d + c * Math.cos(s.theta4), 'closure re', 1e-9);
      near(a * Math.sin(t2) + b * Math.sin(s.theta3), c * Math.sin(s.theta4), 'closure im', 1e-9);
      // open: B left of A→O4
      const cr = (d - s.A[0]) * (s.B[1] - s.A[1]) - (-s.A[1]) * (s.B[0] - s.A[0]);
      assert.ok(mode * cr >= -1e-9, 'configuration side');
      // μ = angle between B→A and B→O4
      const ux = s.A[0] - s.B[0], uy = s.A[1] - s.B[1], vx = d - s.B[0], vy = -s.B[1];
      const ang = Math.acos(Math.max(-1, Math.min(1, (ux * vx + uy * vy) / (Math.hypot(ux, uy) * Math.hypot(vx, vy)))));
      near(s.mu, ang, 'mu', 1e-7);
      // I24 on line AB and ω₄/ω₂ = x/(x − d)
      if (s.I24 && Math.abs(s.I24[0] - d) > 1e-3 * d) {
        const rel = Math.abs(s.w4 - s.I24[0] / (s.I24[0] - d)) / Math.max(1, Math.abs(s.w4));
        assert.ok(rel < 1e-7, 'I24 ratio ' + s.w4 + ' vs ' + s.I24[0] / (s.I24[0] - d));
        const cr2 = (s.B[0] - s.A[0]) * (0 - s.A[1]) - (s.B[1] - s.A[1]) * (s.I24[0] - s.A[0]);
        assert.ok(Math.abs(cr2) < 1e-7 * (1 + Math.abs(s.I24[0])) * b, 'I24 on AB');
      }
      if (s.I13) {
        const c1 = Math.cos(t2) * s.I13[1] - Math.sin(t2) * s.I13[0];
        const c2 = (s.B[0] - d) * s.I13[1] - s.B[1] * (s.I13[0] - d);
        const sc = 1 + Math.hypot(s.I13[0], s.I13[1]);
        assert.ok(Math.abs(c1) < 1e-8 * sc && Math.abs(c2) < 1e-8 * sc * c, 'I13 on O2A and O4B');
      }
      n++;
    });
  }
  return n + ' solutions';
});

group('invariants: w3, w4 = finite differences of θ₃(θ₂), θ₄(θ₂)', function () {
  const R = rng(777);
  let n = 0, skipped = 0;
  while (n < 500) {
    const L = randomLinkage(R, false), a = L[0], b = L[1], c = L[2], d = L[3];
    const t2 = randomValidTheta(R, L), mode = R() < 0.5 ? 1 : -1;
    const s = m.solve(a, b, c, d, t2, mode);
    const hh = 1e-6;
    const sp = m.solve(a, b, c, d, t2 + hh, mode), sm = m.solve(a, b, c, d, t2 - hh, mode);
    // away from toggles: μ not near 0° / 180°
    if (!sp || !sm || Math.sin(s.mu) < 0.05) { skipped++; continue; }
    const d3 = M_wrap(sp.theta3 - sm.theta3) / (2 * hh), d4 = M_wrap(sp.theta4 - sm.theta4) / (2 * hh);
    assert.ok(Math.abs(d4 - s.w4) <= 1e-5 * Math.max(1, Math.abs(s.w4)), 'w4 ' + s.w4 + ' vs ' + d4);
    assert.ok(Math.abs(d3 - s.w3) <= 1e-5 * Math.max(1, Math.abs(s.w3)), 'w3 ' + s.w3 + ' vs ' + d3);
    n++;
  }
  return n + ' points, ' + skipped + ' near toggles skipped';
});
function M_wrap(x) { return Math.atan2(Math.sin(x), Math.cos(x)); }

group('inputRange agrees with a brute-force scan (0.01° steps)', function () {
  const R = rng(4242);
  let n = 0;
  const cases = [[2, 7, 5, 6], [5, 2, 6, 4], [4, 6, 5, 8], [4, 5, 6, 2], [5, 7, 2, 6]];
  while (cases.length < 500) cases.push(randomLinkage(R, cases.length % 2 === 0));
  cases.forEach(function (L) {
    const ir = m.inputRange(L[0], L[1], L[2], L[3]);
    for (let k = 0; k < 36000; k++) {
      const t = k / 100;
      const ok = m.solve(L[0], L[1], L[2], L[3], t * D2R, 1) !== null;
      let inside = false, nearEdge = false;
      ir.rangesDeg.forEach(function (r) {
        [-360, 0, 360].forEach(function (sh) {
          const u = t + sh;
          if (u >= r[0] && u <= r[1]) inside = true;
          if (Math.abs(u - r[0]) < 1e-6 || Math.abs(u - r[1]) < 1e-6) nearEdge = true;
        });
      });
      if (!nearEdge) assert.strictEqual(ok, inside, 'L = ' + L + ' θ₂ = ' + t + ' solvable ' + ok + ' in range ' + inside);
    }
    n++;
  });
  return n + ' linkages';
});

group('classify agrees with brute force (input / output full rotation)', function () {
  const R = rng(99);
  let n = 0, counts = {};
  function rotatesFully(a, b, c, d) {
    for (let k = 0; k < 36000; k++) {
      const t = (k / 100) * D2R;
      // a = d, θ₂ = 0: A falls on O₄ (f = 0), the singular point of a kite — a change-point limit case
      if (Math.hypot(d - a * Math.cos(t), a * Math.sin(t)) < 1e-9) continue;
      if (m.solve(a, b, c, d, t, 1) === null) return false;
    }
    return true;
  }
  const cases = [[2, 7, 5, 6], [4, 5, 6, 2], [5, 2, 6, 4], [5, 7, 2, 6], [4, 6, 5, 8], [3, 6, 3, 6], [2.1, 4.2, 3.3, 3], [5, 7, 2, 4], [2, 5, 5, 2], [5, 2, 6, 3]];
  while (cases.length < 500) cases.push(randomLinkage(R, true));
  cases.forEach(function (L) {
    const a = L[0], b = L[1], c = L[2], d = L[3];
    const cl = m.classify(a, b, c, d);
    const inFull = rotatesFully(a, b, c, d);
    if (cl.type === 'change-point') {
      // limit case: a link adjacent to the frame (or the frame) that is among the shortest still turns fully
      const T = L.map(function (x) { return Math.round(10 * x); }), mn = Math.min.apply(null, T);
      assert.strictEqual(inFull, T[0] === mn || T[3] === mn, 'change-point input ' + L);
      assert.strictEqual(m.inputRange(a, b, c, d).full, inFull, 'inputRange.full ' + L);
      counts[cl.type] = (counts[cl.type] || 0) + 1; n++;
      return;
    }
    const outFull = rotatesFully(c, b, a, d);   // the output, by swapping a and c
    const expectIn = cl.type === 'crank-rocker' || cl.type === 'double-crank';
    const expectOut = cl.type === 'rocker-crank' || cl.type === 'double-crank';
    assert.strictEqual(inFull, expectIn, 'input ' + L + ' ' + cl.type);
    assert.strictEqual(outFull, expectOut, 'output ' + L + ' ' + cl.type);
    assert.strictEqual(m.inputRange(a, b, c, d).full, inFull, 'inputRange.full ' + L);
    counts[cl.type] = (counts[cl.type] || 0) + 1;
    n++;
  });
  return n + ' linkages: ' + JSON.stringify(counts);
});

group('rockerLimits agrees with the numerical extremes of θ₄ (open)', function () {
  const R = rng(5150);
  let n = 0;
  const cases = [[2, 7, 5, 6], [2, 6, 5, 6]];
  while (cases.length < 500) {
    const L = randomLinkage(R, true);   // classify works in tenths, like the sliders
    if (m.classify(L[0], L[1], L[2], L[3]).type === 'crank-rocker') cases.push(L);
  }
  cases.forEach(function (L) {
    const r = m.rockerLimits(L[0], L[1], L[2], L[3]);
    let lo = Infinity, hi = -Infinity, tlo = 0, thi = 0;
    for (let k = 0; k < 36000; k++) {
      const s = m.solve(L[0], L[1], L[2], L[3], (k / 100) * D2R, 1);
      const t4 = s.theta4 * R2D;
      if (t4 < lo) { lo = t4; tlo = k / 100; }
      if (t4 > hi) { hi = t4; thi = k / 100; }
    }
    near(r.theta4MinDeg, lo, 'min ' + L, 1e-3); near(r.theta4MaxDeg, hi, 'max ' + L, 1e-3);
    near(r.swingDeg, hi - lo, 'swing ' + L, 2e-3);
    // the crank angles of the limits (θ₄ is flat there, so only to ~0.5°)
    const te = r.theta2ExtendedDeg, tf = r.theta2FoldedDeg;
    const ok1 = Math.min(Math.abs(te - tlo), Math.abs(te - thi)) < 0.5, ok2 = Math.min(Math.abs(tf - tlo), Math.abs(tf - thi)) < 0.5;
    assert.ok(ok1 && ok2, 'limit crank angles ' + L + ': ' + te + ', ' + tf + ' vs ' + tlo + ', ' + thi);
    near(r.forwardDeg + r.returnDeg, 360, 'forward + return', 1e-9);
    assert.ok(r.forwardDeg >= r.returnDeg);
    n++;
  });
  return n + ' crank-rockers';
});

group('transmissionRange agrees with a scan of μ', function () {
  const R = rng(31337);
  for (let i = 0; i < 60; i++) {
    const L = randomLinkage(R, false);
    const t = m.transmissionRange(L[0], L[1], L[2], L[3]);
    let lo = Infinity, hi = -Infinity;
    m.inputRange(L[0], L[1], L[2], L[3]).rangesDeg.forEach(function (r) {
      for (let k = 0; k <= 2000; k++) {
        const s = m.solve(L[0], L[1], L[2], L[3], (r[0] + (r[1] - r[0]) * k / 2000) * D2R, 1);
        if (s) { lo = Math.min(lo, s.mu * R2D); hi = Math.max(hi, s.mu * R2D); }
      }
    });
    near(t.minDeg, lo, 'min ' + L, 0.05); near(t.maxDeg, hi, 'max ' + L, 0.05);
  }
});

// ---------------------------------------------------------------------------------------------
// Browser regression tests (need Playwright; skipped with a message when it is not available or
// when run with --no-browser). They guard display bugs found in verification.
// Run against another copy of the sim with: node four-bar.test.js --file <path>
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
  const fi = process.argv.indexOf('--file'), file = fi >= 0 ? process.argv[fi + 1] : undefined;
  const outDir = path.join(os.tmpdir(), 'rasim-four-bar-test');
  const num = t => Number(String(t).replace(/−/g, '-').replace(/[°\s]/g, ''));
  for (const width of [1100, 390]) {
    const s = await openSim('four-bar', { width: width, outDir: outDir, file: file });
    const page = s.page;
    const frames = () => page.evaluate(() => new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r))));
    const setS = async st => { await page.evaluate(st => RASim.instances['four-bar'].setState(st), st); await frames(); };
    const getS = () => page.evaluate(() => RASim.instances['four-bar'].getState());
    const stats = () => page.evaluate(() => { const o = {}; document.querySelectorAll('[data-stat]').forEach(e => { o[e.getAttribute('data-stat')] = e.querySelector('.sim-stat-value').textContent; }); o.status = document.querySelector('[data-key="status"]').textContent; return o; });
    try {
      if (width === 1100) {
        await agroup('browser: read-outs equal the model at the check-value states', async function () {
          await setS({ a: 2, b: 7, c: 5, d: 6, theta2: 90, mode: 'open' });
          let r = await stats();
          assert.strictEqual(r.theta3, '25.28°'); assert.strictEqual(r.theta4, '86.22°'); assert.strictEqual(r.mu, '60.94°');
          assert.strictEqual(r.w4, '0.4138'); assert.strictEqual(r.w3, '0.02155'); assert.strictEqual(r.MA, '2.417');
          assert.strictEqual(r.swing, '56.34°'); assert.strictEqual(r.timeRatio, '1.272'); assert.strictEqual(r.muRange, '34.05° … 81.79°');
          assert.strictEqual(r.range, 'full 360°');
          await setS({ a: 5, b: 2, c: 6, d: 4, theta2: 90 });
          r = await stats(); assert.strictEqual(r.range, '51.3° … 125.1° and −125.1° … −51.3°'); assert.strictEqual(r.swing, '—');
          await setS({ a: 4, b: 6, c: 5, d: 8, theta2: 0 });
          r = await stats(); assert.strictEqual(r.range, '−129.8° … 129.8°');
          await setS({ a: 2, b: 6, c: 5, d: 6, theta2: 60 });
          r = await stats(); assert.strictEqual(r.muRange, '41.41° … 92.87°');
          await setS({ a: 3, b: 6, c: 3, d: 6, theta2: 60, mode: 'open' });
          r = await stats(); assert.strictEqual(r.theta4, '60.00°'); assert.strictEqual(r.theta3, '0.00°'); assert.strictEqual(r.w4, '1');
        });
        await agroup('browser: a typed out-of-range θ₂ stays snapped and keeps its message (regression)', async function () {
          await setS({ a: 4, b: 6, c: 5, d: 8, theta2: 0, mode: 'open' });
          const box = await page.$('[data-control-num="theta2"]');
          await box.fill('170'); await box.press('Enter'); await frames();
          const g = await getS(), r = await stats();
          near(g.theta2, m.inputRange(4, 6, 5, 8).rangesDeg[0][1], 'snapped θ₂', 1e-9);
          assert.ok(/170° cannot be reached/.test(r.status), 'snap message kept: ' + r.status);
          assert.strictEqual(r.mu, '180.00°', 'μ at the toggle');
        });
        await agroup('browser: setState takes θ₂ modulo 360° and names the snapped angle in range form (regression)', async function () {
          await setS({ a: 4, b: 6, c: 5, d: 8, theta2: -60 });
          near((await getS()).theta2, 300, 'θ₂ = −60° → 300°', 1e-9);
          assert.strictEqual((await stats()).theta4, RASim.fmtFixed(m.solve(4, 6, 5, 8, -60 * D2R, 1).theta4 * R2D, 2) + '°');
          await setS({ theta2: 200 });
          assert.ok(/230\.2° \(= −129\.8°\)/.test((await stats()).status), (await stats()).status);
        });
      }
      await agroup('browser: no overlapping or clipped canvas labels at ' + width + ' px (regression)', async function () {
        await page.evaluate(() => {
          const orig = RASim.draw.text;
          window.__labels = [];
          RASim.draw.text = function (ctx, text, x, y, o) {
            if (ctx.canvas.getAttribute('data-canvas') === 'mechanism') {
              ctx.save(); ctx.font = (o && o.font) || '12px sans-serif'; const w = ctx.measureText(text).width; ctx.restore();
              const al = (o && o.align) || 'left', l = al === 'center' ? x - w / 2 : al === 'right' ? x - w : x;
              const r = ctx.canvas.getBoundingClientRect();
              window.__labels.push({ text: text, l: l, r: l + w, t: y - 6, b: y + 6, W: r.width, H: r.height });
            }
            return orig.apply(this, arguments);
          };
        });
        // states where the old layout put a link letter on top of the μ / θ₂ label or of a point label, plus the preset states
        const cases = [
          { a: 5, b: 2, c: 6, d: 4, theta2: 29.5, mode: 'open', showIC: false }, { a: 4, b: 5, c: 6, d: 2, theta2: 204, mode: 'open', showIC: false },
          { a: 5, b: 2, c: 6, d: 4, theta2: 90, mode: 'crossed', showIC: false }, { a: 5, b: 2, c: 6, d: 4, theta2: 105, mode: 'crossed', showIC: true },
          { a: 3, b: 6, c: 3, d: 6, theta2: 201, mode: 'open', showIC: true }, { a: 1.1, b: 1.6, c: 9.8, d: 7.3, theta2: 187.5, mode: 'open', u: 0.35, v: 0.5 },
          { a: 7.9, b: 5.7, c: 2.8, d: 4.3, theta2: 105.5, mode: 'crossed', showIC: true, u: 1.45, v: -2.5 },
          { a: 2, b: 6, c: 5, d: 6, theta2: 60, mode: 'open', showIC: false, u: 0.5, v: 2 }, { a: 2, b: 7, c: 5, d: 6, theta2: 0 },
          { a: 4, b: 5, c: 6, d: 2, theta2: 120 }, { a: 5, b: 2, c: 6, d: 4, theta2: 60 }, { a: 4, b: 6, c: 5, d: 8, theta2: 129.838 },
          { a: 3, b: 6, c: 3, d: 6, theta2: 60 }, { a: 2, b: 6, c: 5, d: 6, theta2: 60, mode: 'crossed' }, { mode: 'open', showIC: true }
        ];
        const bad = [];
        for (const st of cases) {
          await page.evaluate(() => { window.__labels = []; });
          await setS(Object.assign({ u: 0.5, v: 2 }, st));
          const L = await page.evaluate(() => window.__labels);
          for (let i = 0; i < L.length; i++) {
            const A = L[i];
            if (A.l < 0 || A.r > A.W || A.t < 0 || A.b > A.H) bad.push(JSON.stringify(st) + ' clipped ' + A.text);
            for (let j = i + 1; j < L.length; j++) { const B = L[j]; if (A.l < B.r && B.l < A.r && A.t < B.b && B.t < A.b) bad.push(JSON.stringify(st) + ' ' + A.text + ' × ' + B.text); }
          }
        }
        assert.strictEqual(bad.length, 0, bad.join('; '));
        return cases.length + ' states';
      });
      assert.ok(s.errors.length === 0 && s.warnings.length === 0, 'console: ' + s.errors.concat(s.warnings).join('; '));
    } finally { await s.browser.close(); }
  }
}

(async function () {
  try { await browserTests(); } catch (e) { failures++; console.log('FAIL browser: ' + (e && e.message || e)); }
  if (failures) { console.log(failures + ' group(s) failed'); process.exit(1); }
  console.log('all four-bar tests passed');
})();
