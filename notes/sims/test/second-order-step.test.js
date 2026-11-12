// Model tests for second-order-step. Run: node notes/sims/test/second-order-step.test.js
'use strict';
const assert = require('assert');
const RASim = require('./load.js')('second-order-step');
const ID = 'second-order-step';
const m = RASim.sims[ID].model;
const M = RASim.math;
const PI = Math.PI;

let failures = 0;
function group(name, fn) {
  try { const extra = fn(); console.log('ok   ' + name + (extra ? '  (' + extra + ')' : '')); }
  catch (e) { failures++; console.log('FAIL ' + name + ': ' + (e && e.message || e)); }
}
// a value printed with `dec` decimals in the specification must round to it: |actual − expected| ≤ half a unit
// of the last printed digit (plus 1e-12 for round-off)
function printed(actual, expected, dec, what) {
  const tol = 0.5 * Math.pow(10, -dec) + 1e-12;
  assert.ok(Number.isFinite(actual) && Math.abs(actual - expected) <= tol, what + ': ' + actual + ' vs ' + expected + ' (tol ' + tol + ')');
}
function near(actual, expected, tol, what) {
  assert.ok(Number.isFinite(actual) && Math.abs(actual - expected) <= tol, what + ': ' + actual + ' vs ' + expected + ' (tol ' + tol + ')');
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

group('API shape and title', function () {
  ['classify', 'poles', 'step', 'specs', 'settleActual', 'rise1090', 'tEnd', 'fromPole'].forEach(function (k) {
    assert.strictEqual(typeof m[k], 'function', 'model.' + k);
  });
  assert.strictEqual(RASim.sims[ID].title, 'Second-order step response and pole location');
  const s = m.specs(0.5, 2);
  ['kind', 'wd', 'thetaDeg', 'tr', 'tp', 'Mp', 'ts2', 'ts5', 'N2', 'Mr', 'wr', 'wb'].forEach(k => assert.ok(k in s, 'specs.' + k));
  const p = m.poles(0.5, 2);
  assert.ok(Array.isArray(p) && p.length === 2 && 're' in p[0] && 'im' in p[0], 'poles shape');
  const f = m.fromPole(-1, 1);
  assert.ok('zeta' in f && 'wn' in f, 'fromPole shape');
});

group('classify', function () {
  assert.strictEqual(m.classify(0), 'undamped');
  assert.strictEqual(m.classify(0.01), 'under');
  assert.strictEqual(m.classify(0.5), 'under');
  assert.strictEqual(m.classify(0.999), 'under');
  assert.strictEqual(m.classify(1), 'critical');
  assert.strictEqual(m.classify(1 - 5e-7), 'critical');
  assert.strictEqual(m.classify(1 + 5e-7), 'critical');
  assert.strictEqual(m.classify(1 + 2e-6), 'over');
  assert.strictEqual(m.classify(1 - 2e-6), 'under');
  assert.strictEqual(m.classify(2.5), 'over');
});

group('check: ζ = 0.5, ωₙ = 2', function () {
  const p = m.poles(0.5, 2);
  printed(p[0].re, -1, 4, 'pole re'); printed(p[0].im, 1.7321, 4, 'pole im');
  printed(p[1].re, -1, 4, 'lower pole re'); printed(p[1].im, -1.7321, 4, 'lower pole im');
  const s = m.specs(0.5, 2);
  assert.strictEqual(s.kind, 'under');
  printed(s.wd, 1.7321, 4, 'wd');
  printed(s.thetaDeg, 60, 0, 'theta');
  printed(s.tr, 1.2092, 4, 'tr');
  printed(s.tp, 1.8138, 4, 'tp');
  printed(100 * s.Mp, 16.303, 3, 'Mp %');
  printed(s.ts2, 4, 0, 'ts2'); printed(s.ts5, 3, 0, 'ts5');
  printed(m.step(0.5, 2, s.tp), 1.16303, 5, 'step(tp)');
  printed(m.settleActual(0.5, 2, 0.02), 4.038, 3, 'actual settle 2 %');
  printed(m.settleActual(0.5, 2, 0.05), 2.645, 3, 'actual settle 5 %');
  printed(m.rise1090(0.5, 2), 0.8188, 4, 'rise 10-90');
  printed(s.Mr, 1.1547, 4, 'Mr'); printed(s.wr, 1.4142, 4, 'wr'); printed(s.wb, 2.5440, 4, 'wb');
  assert.strictEqual(m.tEnd(0.5, 2), 10, 'tEnd');
  near(s.N2, s.ts2 * s.wd / (2 * PI), 1e-12, 'N2');
});

group('check: ζ = 0.70711, ωₙ = 1 (M_p) and ζ = 0.7 (actual 2 % settling)', function () {
  // The specification's 4.3214 % is e^(−π) = M_p at ζ = 1/√2 exactly; at ζ = 0.70711 exactly M_p = 4.32127 %,
  // which prints as 4.3213 (checked against an RK4 integration of the ODE in verification). Both are asserted.
  printed(100 * m.specs(Math.SQRT1_2, 1).Mp, 4.3214, 4, 'Mp % at ζ = 1/√2');
  printed(100 * m.specs(0.70711, 1).Mp, 4.3213, 4, 'Mp % at ζ = 0.70711');
  printed(m.settleActual(0.7, 1, 0.02), 5.979, 3, 'actual settle');
  const s = m.specs(0.70711, 1);
  isNaNField(s.Mr, 'Mr (ζ ≥ 1/√2)'); isNaNField(s.wr, 'wr (ζ ≥ 1/√2)');
  assert.ok(Number.isFinite(m.specs(0.7071, 1).Mr), 'Mr exists for ζ = 0.7071 < 1/√2');
});

group('check: ζ = 1, ωₙ = 1', function () {
  printed(m.step(1, 1, 1), 0.26424, 5, 'step(1)');
  near(m.step(1, 1, 1), 1 - 2 * Math.exp(-1), 1e-15, 'step(1) exact');
  printed(m.settleActual(1, 1, 0.02), 5.834, 3, 'actual settle');
  printed(m.rise1090(1, 1), 3.358, 3, 'rise 10-90');
  const p = m.poles(1, 1);
  assert.deepStrictEqual(p.map(q => [q.re, q.im]), [[-1, 0], [-1, 0]], 'double pole');
  const s = m.specs(1, 1);
  assert.strictEqual(s.kind, 'critical');
  ['wd', 'tr', 'tp', 'ts2', 'ts5', 'N2', 'Mr', 'wr'].forEach(k => isNaNField(s[k], k));
  assert.strictEqual(s.Mp, 0, 'no overshoot');
  assert.strictEqual(s.thetaDeg, 0, 'θ = 0');
});

group('check: ζ = 2, ωₙ = 1', function () {
  const p = m.poles(2, 1);
  printed(p[0].re, -0.26795, 5, 's1'); printed(p[1].re, -3.73205, 5, 's2');
  assert.ok(p[0].im === 0 && p[1].im === 0, 'real poles');
  printed(m.step(2, 1, 1), 0.17774, 5, 'step(1)');
  printed(m.settleActual(2, 1, 0.02), 14.88, 2, 'actual settle');
  printed(m.rise1090(2, 1), 8.229, 3, 'rise 10-90');
  const s = m.specs(2, 1);
  assert.strictEqual(s.kind, 'over');
  ['wd', 'thetaDeg', 'tr', 'tp', 'ts2', 'ts5', 'N2', 'Mr', 'wr'].forEach(k => isNaNField(s[k], k));
  assert.strictEqual(s.Mp, 0, 'no overshoot');
  assert.ok(s.wb > 0 && Number.isFinite(s.wb), 'wb');
  // over-damped spec formula y = 1 − (s₂e^(s₁t) − s₁e^(s₂t))/(s₂ − s₁)
  const s1 = p[0].re, s2 = p[1].re;
  [0.1, 0.5, 1, 3, 10].forEach(t => near(m.step(2, 1, t), 1 - (s2 * Math.exp(s1 * t) - s1 * Math.exp(s2 * t)) / (s2 - s1), 1e-14, 'formula t=' + t));
});

group('check: ζ = 0, ωₙ = 2', function () {
  near(m.step(0, 2, PI / 2), 2, 1e-12, 'step(π/2)');
  const s = m.specs(0, 2);
  assert.strictEqual(s.kind, 'undamped');
  assert.strictEqual(s.Mp, 1, 'Mp = 1');
  assert.strictEqual(s.ts2, Infinity); assert.strictEqual(s.ts5, Infinity);
  assert.strictEqual(m.settleActual(0, 2, 0.02), Infinity); assert.strictEqual(m.settleActual(0, 2, 0.05), Infinity);
  const p = m.poles(0, 2);
  assert.ok(p[0].re === 0 && p[0].im === 2 && p[1].re === 0 && p[1].im === -2, 'poles ±j2');
  [0.3, 1, 2.7, 9].forEach(t => near(m.step(0, 2, t), 1 - Math.cos(2 * t), 1e-14, '1 − cos ωₙt at ' + t));
  near(s.tp, PI / 2, 1e-15, 'tp'); near(s.thetaDeg, 90, 1e-12, 'θ');
});

group('check: ζ = 0.2, ωₙ = 5', function () {
  printed(m.settleActual(0.2, 5, 0.02), 3.920, 3, 'actual settle');
  printed(m.rise1090(0.2, 5), 0.2407, 4, 'rise 10-90');
});

group('check: fromPole(−1, 1.7320508) -> ζ = 0.5, ωₙ = 2', function () {
  const r = m.fromPole(-1, 1.7320508);
  printed(r.zeta, 0.5, 4, 'zeta'); printed(r.wn, 2, 4, 'wn');
  const a = m.fromPole(-3, 0); near(a.zeta, 1, 0, 'real axis ζ'); near(a.wn, 3, 0, 'real axis ωₙ');
  const b = m.fromPole(0, 2); assert.ok(b.zeta === 0 && !Object.is(b.zeta, -0) && b.wn === 2, 'jω axis');
  // round trip with poles()
  const r2 = rng(7);
  for (let i = 0; i < 100; i++) {
    const z = 0.999 * r2(), wn = 0.5 + 9.5 * r2();
    const f = m.fromPole(m.poles(z, wn)[0].re, m.poles(z, wn)[0].im);
    near(f.zeta, z, 1e-12, 'round trip ζ'); near(f.wn, wn, 1e-12 * wn, 'round trip ωₙ');
  }
});

group('spec formulas: under-damped closed form, critical form, tEnd rule', function () {
  const r = rng(11);
  for (let i = 0; i < 200; i++) {
    const z = 0.01 + 0.98 * r(), wn = 0.5 + 9.5 * r(), t = 3 * r() / (z * wn);
    const w = Math.sqrt(1 - z * z), th = Math.acos(z);
    near(m.step(z, wn, t), 1 - Math.exp(-z * wn * t) * Math.sin(wn * w * t + th) / w, 1e-12, 'closed form');
    const s = m.specs(z, wn);
    near(s.tr, (PI - th) / (wn * w), 1e-12, 'tr'); near(s.Mp, Math.exp(-PI * z / w), 1e-15, 'Mp');
    near(m.step(z, wn, s.tr), 1, 1e-12, 'y(t_r) = 1');
  }
  [0.3, 1, 2.5, 7].forEach(t => near(m.step(1, 1.7, t), 1 - (1 + 1.7 * t) * Math.exp(-1.7 * t), 1e-15, 'critical t=' + t));
  for (let i = 0; i < 200; i++) {
    const z = 2.5 * r(), wn = 0.5 + 9.5 * r();
    const k = m.classify(z);
    const sg = k === 'over' ? Math.abs(m.poles(z, wn)[0].re) : z * wn;
    const raw = sg > 0 ? M.clamp(6 / sg, 8 / wn, 40 / wn) : 40 / wn;
    assert.strictEqual(m.tEnd(z, wn), M.niceCeil(raw), 'tEnd');
  }
  assert.strictEqual(m.tEnd(0, 2), 20, 'tEnd ζ = 0');
  // bandwidth: |G(jω_b)| = 1/√2
  for (let i = 0; i < 100; i++) {
    const z = 2.5 * r(), wn = 0.5 + 9.5 * r();
    const wb = m.specs(z, wn).wb;
    near(M.cabs(M.tfEval([wn * wn], [1, 2 * z * wn, wn * wn], M.cx(0, wb))), Math.SQRT1_2, 1e-9, '|G(jωb)| ζ=' + z);
  }
  // resonant peak: |G(jω_r)| = M_r and it is a maximum
  for (let i = 0; i < 50; i++) {
    const z = 0.05 + 0.6 * r(), wn = 0.5 + 9.5 * r(), s = m.specs(z, wn);
    const g = w2 => M.cabs(M.tfEval([wn * wn], [1, 2 * z * wn, wn * wn], M.cx(0, w2)));
    near(g(s.wr), s.Mr, 1e-9 * s.Mr, 'Mr');
    assert.ok(g(s.wr * 1.01) < s.Mr && g(s.wr * 0.99) < s.Mr, 'max at ωr');
  }
});

group('invariant: step() agrees with RASim.math.tfStep to 1e-8 (200 random + special ζ)', function () {
  const r = rng(12345);
  const cases = [[0, 2], [0, 0.5], [0, 10], [1, 1], [1, 7.3], [1 - 1e-7, 2], [1 + 1e-7, 2], [1 - 5e-8, 0.9], [1 + 3e-8, 4.4],
    [1 - 9e-7, 3], [1 + 9e-7, 3], [1 + 2e-6, 1], [2.5, 0.5], [2.5, 10]];
  while (cases.length < 214) cases.push([2.5 * r(), 0.5 + 9.5 * r()]);
  let worst = 0;
  cases.forEach(function (c) {
    const z = c[0], wn = c[1], T = m.tEnd(z, wn);
    const ref = M.tfStep([wn * wn], [1, 2 * z * wn, wn * wn], T, 400);
    for (let i = 0; i <= 400; i++) {
      const d = Math.abs(m.step(z, wn, ref.t[i]) - ref.y[i]);
      if (d > worst) worst = d;
      assert.ok(d <= 1e-8, 'ζ=' + z + ' ωₙ=' + wn + ' t=' + ref.t[i] + ': ' + m.step(z, wn, ref.t[i]) + ' vs ' + ref.y[i]);
    }
  });
  return cases.length + ' cases, worst ' + worst.toExponential(2);
});

group('invariant: step(0) = 0, continuity across ζ = 1, time scaling', function () {
  const r = rng(99);
  for (let i = 0; i < 200; i++) {
    const z = 2.5 * r(), wn = 0.5 + 9.5 * r();
    assert.strictEqual(m.step(z, wn, 0), 0, 'step(0) ζ=' + z);
    const t = m.tEnd(z, wn) * r();
    near(m.step(z, wn, t), m.step(z, 1, wn * t), 1e-13, 'time scaling');
  }
  // |∂y/∂ζ| at ζ = 1 is e^(−x)x³/3 ≤ 0.45 (x = ωₙt): a step of ε in ζ moves y by at most ~0.45ε
  [1e-3, 1e-5, 1e-7, 1e-9, 1e-12].forEach(function (e) {
    for (let x = 0.05; x < 40; x += 0.37) {
      const y0 = m.step(1, 1, x);
      assert.ok(Math.abs(m.step(1 + e, 1, x) - y0) <= 0.5 * e + 1e-15, 'above ε=' + e + ' x=' + x);
      assert.ok(Math.abs(m.step(1 - e, 1, x) - y0) <= 0.5 * e + 1e-15, 'below ε=' + e + ' x=' + x);
    }
  });
});

group('invariant: max of finely sampled under-damped response = 1 + M_p at t_p', function () {
  const r = rng(2024);
  for (let i = 0; i < 60; i++) {
    const z = 0.02 + 0.95 * r(), wn = 0.5 + 9.5 * r();
    const s = m.specs(z, wn), T = Math.max(m.tEnd(z, wn), 1.5 * s.tp), n = 200000;
    let best = -Infinity, tb = 0;
    for (let k = 0; k <= n; k++) { const t = T * k / n, y = m.step(z, wn, t); if (y > best) { best = y; tb = t; } }
    near(best, 1 + s.Mp, 1e-8, 'max ζ=' + z);
    near(tb, s.tp, 1.5 * T / n + 1e-6 * s.tp, 'argmax ζ=' + z);
  }
  // ζ = 0: maximum 2 at t_p = π/ωₙ
  near(m.step(0, 3, m.specs(0, 3).tp), 2, 1e-14, 'ζ = 0 peak');
});

group('independent: step() agrees with an RK4 integration of y″ + 2ζωₙy′ + ωₙ²y = ωₙ² (60 random + edge cases)', function () {
  const r = rng(4711);
  const cases = [[0, 2], [0.5, 2], [1, 1], [1 - 1e-7, 3], [1 + 1e-7, 3], [2, 1], [2.5, 10], [0.01, 0.5]];
  while (cases.length < 68) cases.push([2.5 * r(), 0.5 + 9.5 * r()]);
  let worst = 0;
  cases.forEach(function (c) {
    const z = c[0], wn = c[1], T = m.tEnd(z, wn), n = Math.ceil(T * wn / 2e-3), h = T / n;
    const f = (y, v) => wn * wn * (1 - y) - 2 * z * wn * v;
    let y = 0, v = 0;
    for (let i = 1; i <= n; i++) {
      const k1y = v, k1v = f(y, v), k2y = v + h / 2 * k1v, k2v = f(y + h / 2 * k1y, v + h / 2 * k1v);
      const k3y = v + h / 2 * k2v, k3v = f(y + h / 2 * k2y, v + h / 2 * k2v), k4y = v + h * k3v, k4v = f(y + h * k3y, v + h * k3v);
      y += h / 6 * (k1y + 2 * k2y + 2 * k3y + k4y); v += h / 6 * (k1v + 2 * k2v + 2 * k3v + k4v);
      if (i % 97 === 0 || i === n) {
        const d = Math.abs(m.step(z, wn, i * h) - y);
        if (d > worst) worst = d;
        assert.ok(d <= 1e-9, 'ζ=' + z + ' ωₙ=' + wn + ' t=' + i * h + ': ' + m.step(z, wn, i * h) + ' vs RK4 ' + y);
      }
    }
  });
  return cases.length + ' cases, worst ' + worst.toExponential(2);
});

group('invariant: settleActual and rise1090 agree with a brute-force scan', function () {
  const r = rng(31337);
  const cases = [[0.5, 2], [0.7, 1], [1, 1], [2, 1], [0.2, 5], [0.05, 1], [0.69, 3], [1.4, 0.7], [2.5, 0.5], [0.995, 2]];
  while (cases.length < 40) cases.push([0.04 + 2.46 * r(), 0.5 + 9.5 * r()]);
  cases.forEach(function (c) {
    const z = c[0], wn = c[1];
    [0.02, 0.05].forEach(function (band) {
      const ts = m.settleActual(z, wn, band);
      const H = 1.5 * ts + 1, n = 300000, dt = H / n;
      let last = 0;
      for (let k = 0; k <= n; k++) { const t = k * dt; if (Math.abs(m.step(z, wn, t) - 1) > band) last = t; }
      near(ts, last, 1.01 * dt, 'settle ζ=' + z + ' ωₙ=' + wn + ' band=' + band);
    });
    const T = m.tEnd(z, wn), n = 300000, dt = T / n;
    let t10 = NaN, t90 = NaN;
    for (let k = 0; k <= n; k++) {
      const t = k * dt, y = m.step(z, wn, t);
      if (isNaN(t10) && y >= 0.1) t10 = t;
      if (isNaN(t90) && y >= 0.9) { t90 = t; break; }
    }
    near(m.rise1090(z, wn), t90 - t10, 2.01 * dt, 'rise ζ=' + z + ' ωₙ=' + wn);
  });
  return cases.length + ' cases × 2 bands';
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
  const outDir = path.join(os.tmpdir(), 'rasim-second-order-step-test');
  const DEF = { zeta: 0.5, wn: 2, band: '2', showEnvelope: true, showMarkers: true, held: false };
  const STATES = [[{}], [{ zeta: 0 }], [{ zeta: 1 }], [{ zeta: 2 }], [{ held: true }, { zeta: 0.2, wn: 5 }], [{ band: '5' }],
    [{ zeta: 0.05, wn: 0.5 }], [{ zeta: 0.05, wn: 2 }], [{ zeta: 0.1, wn: 1 }], [{ zeta: 0.99, wn: 10 }], [{ zeta: 2.5, wn: 10 }],
    [{ zeta: 0.707, wn: 8 }], [{ zeta: 0.2, wn: 5 }, { held: true }, { zeta: 0.5, wn: 2 }], [{ zeta: 0.3, wn: 10, band: '5' }]];
  for (const width of [390, 1100]) {
    const s = await openSim(ID, { width: width, outDir: outDir });
    const page = s.page;
    const frames = () => page.evaluate(() => new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r))));
    const setS = async function (st) { await page.evaluate(st => RASim.instances['second-order-step'].setState(st), st); await frames(); };
    const W = ' @' + width + ' px';
    try {
      await agroup('browser: canvas labels never overlap each other and stay inside the canvas (regression)' + W, async function () {
        let n = 0;
        for (const seq of STATES) {
          await setS(DEF);
          for (const st of seq) await setS(st);
          const info = await page.evaluate(() => ['step', 'splane'].map(k => {
            const c = document.querySelector('[data-sim="second-order-step"] [data-canvas="' + k + '"]');
            return { k: k, w: c.clientWidth, h: c.clientHeight, labels: c.sosLabels || null };
          }));
          info.forEach(function (cv) {
            assert.ok(Array.isArray(cv.labels), cv.k + ': label boxes not exposed');
            const L = cv.labels;
            for (let i = 0; i < L.length; i++) {
              assert.ok(L[i].l >= 0 && L[i].t >= 0 && L[i].r <= cv.w && L[i].b <= cv.h, cv.k + ' ' + JSON.stringify(seq) + ': label "' + L[i].text + '" leaves the canvas');
              for (let j = 0; j < i; j++) {
                const a = L[i], b = L[j], ov = Math.min(a.r, b.r) - Math.max(a.l, b.l) > 0.5 && Math.min(a.b, b.b) - Math.max(a.t, b.t) > 0.5;
                assert.ok(!ov, cv.k + ' ' + JSON.stringify(seq) + ': labels "' + a.text + '" and "' + b.text + '" overlap');
              }
            }
            n += L.length;
          });
        }
        return n + ' labels in ' + STATES.length + ' states';
      });
      await agroup('browser: every pole × is visible (no label background hides a pole) (regression)' + W, async function () {
        for (const seq of STATES) {
          await setS(DEF);
          for (const st of seq) await setS(st);
          const res = await page.evaluate(() => {
            const c = document.querySelector('[data-sim="second-order-step"] [data-canvas="splane"]');
            const ctx = c.getContext('2d'), dpr = c.width / c.clientWidth;
            return (c.sosPoles || []).map(function (q) {
              const d = ctx.getImageData(Math.round((q.x - 1) * dpr), Math.round((q.y - 1) * dpr), Math.round(2 * dpr) + 1, Math.round(2 * dpr) + 1).data;
              let best = 1e9;   // colour distance to RASim.colors.red (#c0392b) near the centre of the ×
              for (let i = 0; i < d.length; i += 4) best = Math.min(best, Math.abs(d[i] - 0xc0) + Math.abs(d[i + 1] - 0x39) + Math.abs(d[i + 2] - 0x2b));
              return best;
            });
          });
          assert.ok(res.length === 2, JSON.stringify(seq) + ': pole positions not exposed');
          res.forEach((d, i) => assert.ok(d < 40, JSON.stringify(seq) + ': pole ' + (i + 1) + ' is hidden (colour distance ' + d + ')'));
        }
      });
      await agroup('browser: the s-plane does not jump when the legend changes during a pole drag (regression)' + W, async function () {
        await setS(DEF);
        const top = () => page.evaluate(() => { const r = document.querySelector('[data-sim="second-order-step"] [data-canvas="splane"]').getBoundingClientRect(); return r.top + window.scrollY; });
        const t0 = await top();
        for (const st of [{ zeta: 0.659, wn: 2.277 }, { zeta: 1 }, { zeta: 0, wn: 10 }, { zeta: 0.123, wn: 0.5 }, { zeta: 0.9987, wn: 9.876 }, { showMarkers: false }, { held: true, showEnvelope: false }, { band: '5', showMarkers: true }]) {
          await setS(st);
          const t1 = await top();
          assert.ok(Math.abs(t1 - t0) < 0.5, 'after ' + JSON.stringify(st) + ' the s-plane moved from ' + t0 + ' to ' + t1);
        }
        // a real mouse drag: the dragged pole ends exactly under the pointer
        await setS(DEF);
        await page.evaluate(() => document.querySelector('[data-sim="second-order-step"] [data-canvas="splane"]').scrollIntoView({ behavior: 'instant', block: 'center' }));
        await frames();
        const g = await page.evaluate(() => {
          const c = document.querySelector('[data-sim="second-order-step"] [data-canvas="splane"]'), r = c.getBoundingClientRect();
          return { x0: r.left + 1, y0: r.top + 1, pole: c.sosPoles[0] };
        });
        // from (−1, 1.732) at ωₙ = 2: move by (−0.5, −0.232) data units = to (−1.5, 1.5); 1 unit = |pole| / 2 in px
        const ppu = (g.pole.y - (await page.evaluate(() => document.querySelector('[data-sim="second-order-step"] [data-canvas="splane"]').sosPoles[1].y))) / (2 * Math.sqrt(3)) * -1;
        await page.mouse.move(g.x0 + g.pole.x, g.y0 + g.pole.y);
        await page.mouse.down();
        await page.mouse.move(g.x0 + g.pole.x - 0.5 * ppu, g.y0 + g.pole.y + (Math.sqrt(3) - 1.5) * ppu, { steps: 6 });
        await page.mouse.up();
        const st = await page.evaluate(() => RASim.instances['second-order-step'].getState());
        near(-st.zeta * st.wn, -1.5, 0.02, 'dragged pole re'); near(st.wn * Math.sqrt(1 - st.zeta * st.zeta), 1.5, 0.02, 'dragged pole im');
      });
      if (width === 1100) await agroup('browser: read-outs show the check values; Reset restores every default', async function () {
        const read = () => page.evaluate(() => {
          const o = {};
          document.querySelectorAll('[data-sim="second-order-step"] [data-stat]').forEach(e => { o[e.getAttribute('data-stat')] = e.querySelector('.sim-stat-value').textContent; });
          o.kind = document.querySelector('[data-tag="kind"]').textContent; o.tf = document.querySelector('[data-key="tf"]').textContent;
          return o;
        });
        const expect = [
          [DEF, { poles: '−1 ± j1.732', wd: '1.732 rad/s', theta: '60°', tr: '1.209 s', tp: '1.814 s', Mp: '16.3 %', ts: '4 s', tsActual: '4.038 s', tr1090: '0.8188 s', Mr: '1.155', wr: '1.414 rad/s', wb: '2.544 rad/s', kind: 'UNDERDAMPED', tf: 'G(s) = 4 / (s² + 2s + 4)' }],
          [{ band: '5' }, { ts: '3 s', tsActual: '2.645 s' }],
          [{ band: '2', zeta: 0.7, wn: 1 }, { tsActual: '5.979 s', Mp: '4.599 %' }],
          [{ zeta: 1, wn: 1 }, { poles: '−1 (double)', tsActual: '5.834 s', tr1090: '3.358 s', Mp: '0 %', tr: '—', kind: 'CRITICALLY DAMPED' }],
          [{ zeta: 2, wn: 1 }, { poles: '−0.2679, −3.732', tsActual: '14.88 s', tr1090: '8.229 s', theta: '—', kind: 'OVERDAMPED', tf: 'G(s) = 1 / (s² + 4s + 1)' }],
          [{ zeta: 0, wn: 2 }, { poles: '± j2', Mp: '100 %', ts: '∞', tsActual: '∞', kind: 'UNDAMPED', tf: 'G(s) = 4 / (s² + 4)' }],
          [{ zeta: 0.2, wn: 5 }, { tsActual: '3.92 s', tr1090: '0.2407 s', Mp: '52.66 %', poles: '−1 ± j4.899' }]
        ];
        for (const [st, want] of expect) {
          await setS(st);
          const got = await read();
          Object.keys(want).forEach(k => assert.strictEqual(got[k].trim(), want[k], JSON.stringify(st) + ' ' + k));
        }
        await setS({ zeta: 1.37, wn: 7.7, band: '5', showEnvelope: false, showMarkers: false, held: true });
        await page.click('[data-sim="second-order-step"] [data-control="reset"]');
        await frames();
        assert.deepStrictEqual(await page.evaluate(() => RASim.instances['second-order-step'].getState()), DEF);
      });
      assert.deepStrictEqual(s.errors, [], 'console errors');
    } finally { await s.browser.close(); }
  }
}

browserTests().then(function () {
  if (failures) { console.log(failures + ' group(s) FAILED'); process.exit(1); }
  console.log('all groups passed');
}, function (e) { console.log('FAIL browser tests: ' + (e && e.stack || e)); process.exit(1); });
