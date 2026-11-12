// Model tests for notes/sims/pid-tuning.js. Run: node notes/sims/test/pid-tuning.test.js
'use strict';
const assert = require('assert');
const RASim = require('./load.js')('pid-tuning');
const m = RASim.sims['pid-tuning'].model;
const M = RASim.math;

let failures = 0;
function group(name, fn) {
  try { const extra = fn(); console.log('ok   ' + name + (extra ? '  (' + extra + ')' : '')); } catch (e) {
    failures++;
    console.log('FAIL ' + name + ': ' + (e && e.message || e));
  }
}
// tolerance: as stated, otherwise max(5e-4 relative, half a unit of the last printed digit)
function near(actual, expected, what, tol) {
  if (tol == null) {
    const s = String(expected);
    const dec = s.indexOf('.') >= 0 ? s.length - s.indexOf('.') - 1 : 0;
    tol = Math.max(5e-4 * Math.abs(expected), 0.5 * Math.pow(10, -dec));
  }
  assert.ok(Math.abs(actual - expected) <= tol, what + ': got ' + actual + ', expected ' + expected + ' ± ' + tol);
}
function polyNear(p, q, what, tol) {
  assert.strictEqual(p.length, q.length, what + ': length ' + JSON.stringify(p) + ' vs ' + JSON.stringify(q));
  p.forEach(function (v, i) { near(v, q[i], what + '[' + i + ']', tol == null ? 1e-12 * Math.max(1, Math.abs(q[i])) : tol); });
}
function tfEq(tf, num, den, what) {
  // compare as normalised rational functions (leading denominator coefficient 1)
  const a = tf.den[0], b = den[0];
  polyNear(tf.den.map(v => v / a), den.map(v => v / b), what + ' den', 1e-12);
  polyNear(M.polyTrim(tf.num).map(v => v / a), num.map(v => v / b), what + ' num', 1e-12);
}
// seeded generator (mulberry32)
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
function sample(plantId, Kp, Ki, Kd, opts) {
  const L = m.loop(plantId, Kp, Ki, Kd);
  const s = m.simulate(plantId, Kp, Ki, Kd, opts);
  return { L: L, s: s, mt: m.metrics(s.t, s.yRef, L.yFinal) };
}
function at(s, t) { const k = s.t.findIndex(v => Math.abs(v - t) < 1e-12); assert.ok(k >= 0, 'no sample at t = ' + t); return s.y[k]; }
function maxRe(L) { return Math.max.apply(null, L.poles.map(p => p.re)); }
function hasPole(L, re, im, tol) { return L.poles.some(p => Math.abs(p.re - re) <= tol && Math.abs(p.im - im) <= tol); }

group('API shape and constants', function () {
  assert.strictEqual(m.TAU_F, 0.02);
  ['first', 'second', 'third', 'motor'].forEach(function (id) {
    const P = m.plants[id];
    ['label', 'num', 'den', 'Ku', 'Pu'].forEach(k => assert.ok(k in P, id + '.' + k));
  });
  polyNear(m.plants.first.den, [1, 1], 'first den'); polyNear(m.plants.first.num, [1], 'first num');
  polyNear(m.plants.second.den, [1, 3, 2], 'second den'); polyNear(m.plants.second.num, [2], 'second num');
  polyNear(m.plants.third.den, [1, 3, 3, 1], 'third den'); polyNear(m.plants.motor.den, [1, 1, 0], 'motor den');
  assert.strictEqual(m.plants.first.Ku, null); assert.strictEqual(m.plants.second.Pu, null); assert.strictEqual(m.plants.motor.Ku, null);
  near(m.plants.third.Ku, 8, 'third Ku'); near(m.plants.third.Pu, 3.62760, 'third Pu');
  near(m.plants.third.Pu, 2 * Math.PI / Math.sqrt(3), 'Pu = 2π/√3', 1e-12);
  // controller forms
  const c1 = m.controller(3, 0, 0); polyNear(c1.num, [3]); polyNear(c1.den, [1]);
  const c2 = m.controller(3, 0, 2); polyNear(c2.num, [3 * 0.02 + 2, 3]); polyNear(c2.den, [0.02, 1]);
  const c3 = m.controller(3, 4, 0); polyNear(c3.num, [3, 4]); polyNear(c3.den, [1, 0]);
  const c4 = m.controller(3, 4, 2); polyNear(c4.num, [3 * 0.02 + 2, 3 + 4 * 0.02, 4]); polyNear(c4.den, [0.02, 1, 0]);
  // C(jω) agrees with K_p + K_i/s + K_d s/(τ_f s + 1) at a few points for every form
  const r = rng(7);
  for (let i = 0; i < 40; i++) {
    const Kp = r() * 20, Ki = i % 4 < 2 ? 0 : r() * 20, Kd = i % 2 ? 0 : r() * 10;
    const c = m.controller(Kp, Ki, Kd);
    const s = M.cx(-0.3 + r(), 0.2 + 3 * r());
    const lhs = M.tfEval(c.num, c.den, s);
    let rhs = M.cx(Kp);
    if (Ki) rhs = M.cadd(rhs, M.cdiv(M.cx(Ki), s));
    if (Kd) rhs = M.cadd(rhs, M.cdiv(M.cscale(s, Kd), M.cadd(M.cscale(s, 0.02), M.cx(1))));
    assert.ok(M.cabs(M.csub(lhs, rhs)) < 1e-10 * Math.max(1, M.cabs(rhs)), 'controller form ' + i);
  }
  const L = m.loop('third', 1, 0, 0);
  ['T', 'U', 'Yd', 'Ud'].forEach(k => assert.ok(Array.isArray(L[k].num) && Array.isArray(L[k].den), k));
  ['charPoly', 'poles', 'stability', 'type', 'ess', 'yFinal', 'distOffsetPerUnit'].forEach(k => assert.ok(k in L, k));
  const s = m.simulate('third', 1, 0, 0);
  assert.strictEqual(s.t.length, 601); assert.strictEqual(s.y.length, 601); assert.strictEqual(s.u.length, 601); assert.strictEqual(s.yRef.length, 601);
  near(s.t[600], 15, 'default tEnd', 0);
  const mt = m.metrics(s.t, s.yRef, L.yFinal);
  ['overshootPct', 'tPeak', 'tRise', 'tSettle'].forEach(k => assert.ok(k in mt, k));
  const g = m.zn(8, 3.6276, 'PI');
  assert.deepStrictEqual(Object.keys(g).sort(), ['Kd', 'Ki', 'Kp']);
});

group('loop transfer functions (Δ = D_c·D_p + N_c·N_p)', function () {
  const L = m.loop('second', 2, 3, 0.5);
  const c = m.controller(2, 3, 0.5), P = m.plants.second;
  const Dl = M.polyAdd(M.polyMul(c.den, P.den), M.polyMul(c.num, P.num));
  polyNear(L.charPoly, Dl, 'charPoly');
  tfEq(L.T, M.polyMul(c.num, P.num), Dl, 'T');
  tfEq(L.U, M.polyMul(c.num, P.den), Dl, 'U');
  tfEq(L.Yd, M.polyMul(P.num, c.den), Dl, 'Yd');
  tfEq(L.Ud, M.polyScale(M.polyMul(c.num, P.num), -1), Dl, 'Ud');
  // all gains zero: T = 0, y stays 0
  ['first', 'second', 'third', 'motor'].forEach(function (id) {
    const s = m.simulate(id, 0, 0, 0, { tEnd: 10 });
    assert.ok(s.y.every(v => v === 0), id + ': y stays 0');
    assert.ok(s.u.every(v => v === 0), id + ': u stays 0');
  });
  assert.strictEqual(m.loop('motor', 0, 0, 0).stability, 'marginal');
  assert.strictEqual(m.loop('first', 0, 0, 0).stability, 'stable');
});

group('check values: first order', function () {
  const a = sample('first', 4, 0, 0);
  tfEq(a.L.T, [4], [1, 5], 'T = 4/(s + 5)');
  near(a.L.yFinal, 0.8, 'y(∞)'); near(a.L.ess, 0.2, 'e_ss');
  near(at(a.s, 0.2), 0.50570, 'y(0.2)');
  assert.strictEqual(a.L.type, 0); assert.strictEqual(a.L.stability, 'stable');
  const b = sample('first', 1, 1, 0);
  near(at(b.s, 1), 0.63212, 'y(1)');
  for (let k = 0; k < b.s.t.length; k++) near(b.s.y[k], 1 - Math.exp(-b.s.t[k]), 'y = 1 − e^(−t) at ' + b.s.t[k], 1e-9);
  near(b.L.ess, 0, 'e_ss', 1e-15); assert.strictEqual(b.L.type, 1); assert.strictEqual(b.L.stability, 'stable');
  // P only: e_ss = 1/(1 + K_p), never zero
  [0.5, 1, 2, 10, 20].forEach(Kp => near(m.loop('first', Kp, 0, 0).ess, 1 / (1 + Kp), 'e_ss(K_p = ' + Kp + ')', 1e-12));
});

group('check values: second order', function () {
  const a = sample('second', 5, 0, 0);
  near(a.L.yFinal, 0.83333, 'y(∞)');
  near(a.mt.overshootPct, 22.11, 'overshoot %');
  assert.ok(hasPole(a.L, -1.5, 3.1225, 5e-5) && hasPole(a.L, -1.5, -3.1225, 5e-5), 'poles −1.5 ± j3.1225');
  assert.strictEqual(a.L.poles.length, 2);
});

group('check values: third order', function () {
  const a = sample('third', 8, 0, 0);
  assert.strictEqual(a.L.stability, 'marginal');
  assert.ok(hasPole(a.L, 0, 1.73205, 5e-6) && hasPole(a.L, 0, -1.73205, 5e-6) && hasPole(a.L, -3, 0, 5e-5), 'poles ±j1.73205, −3');
  near(m.loop('third', 1, 0, 0).ess, 0.5, 'e_ss at K_p = 1');
  assert.strictEqual(m.loop('third', 9, 0, 0).stability, 'unstable');
  // K_u = 8 is the boundary
  assert.strictEqual(m.loop('third', 7.95, 0, 0).stability, 'stable');
  assert.strictEqual(m.loop('third', 8.05, 0, 0).stability, 'unstable');
  // sustained oscillation at K_u: period P_u (zero crossings of y − 8/9 half a period apart)
  const s = m.simulate('third', 8, 0, 0, { tEnd: 60, n: 6000 });
  const yc = 8 / 9, cr = [];
  for (let k = 3000; k < 6000; k++) {
    const e0 = s.yRef[k] - yc, e1 = s.yRef[k + 1] - yc;
    if (e0 < 0 && e1 >= 0) cr.push(s.t[k] + (-e0) / (e1 - e0) * (s.t[k + 1] - s.t[k]));
  }
  near((cr[cr.length - 1] - cr[0]) / (cr.length - 1), 3.6276, 'oscillation period = P_u');
});

group('check values: Ziegler–Nichols', function () {
  let g = m.zn(8, 3.62760, 'PID');
  near(g.Kp, 4.8, 'PID Kp'); near(g.Ki, 2.6464, 'PID Ki'); near(g.Kd, 2.1766, 'PID Kd');
  g = m.zn(8, 3.62760, 'PI');
  near(g.Kp, 3.6, 'PI Kp'); near(g.Ki, 1.1909, 'PI Ki'); near(g.Kd, 0, 'PI Kd', 0);
  g = m.zn(8, 3.62760, 'P');
  near(g.Kp, 4, 'P Kp'); near(g.Ki, 0, 'P Ki', 0); near(g.Kd, 0, 'P Kd', 0);
  const P = m.plants.third;
  const zd = m.zn(P.Ku, P.Pu, 'PID');
  const a = sample('third', zd.Kp, zd.Ki, zd.Kd);
  assert.strictEqual(a.L.stability, 'stable'); near(a.L.ess, 0, 'Z–N PID e_ss', 1e-15);
  near(a.mt.overshootPct, 41.5, 'Z–N PID overshoot', 0.5);
  near(a.mt.tPeak, 2.19, 'Z–N PID peak time', 0.02);
  near(a.mt.tSettle, 9.3, 'Z–N PID settling', 0.2);
  const zi = m.zn(P.Ku, P.Pu, 'PI');
  near(sample('third', zi.Kp, zi.Ki, zi.Kd).mt.overshootPct, 56.1, 'Z–N PI overshoot', 0.5);
  const zp = m.zn(P.Ku, P.Pu, 'P');
  const c = sample('third', zp.Kp, zp.Ki, zp.Kd);
  near(c.L.yFinal, 0.8, 'Z–N P final value'); near(c.mt.overshootPct, 54.3, 'Z–N P overshoot', 0.5);
});

group('check values: motor', function () {
  const a = sample('motor', 1, 0, 0);
  tfEq(a.L.T, [1], [1, 1, 1], 'T = 1/(s² + s + 1)');
  near(a.mt.overshootPct, 16.30, 'overshoot'); near(a.mt.tPeak, 3.628, 'peak time');
  near(a.L.ess, 0, 'e_ss', 1e-15); assert.strictEqual(a.L.type, 1);
  near(sample('motor', 4, 0, 0).mt.overshootPct, 44.43, 'K_p = 4 overshoot');
  const b = sample('motor', 4, 0, 3);
  near(b.mt.overshootPct, 2.4, 'K_p = 4, K_d = 3 overshoot', 0.3);
  [-1.601, -2.673, -46.73].forEach(r => assert.ok(hasPole(b.L, r, 0, 5e-4 * Math.abs(r) + 5e-4), 'pole near ' + r));
  assert.strictEqual(b.L.poles.length, 3);
  assert.strictEqual(m.loop('motor', 1, 1, 0).type, 2);
});

group('check values: load disturbance', function () {
  near(m.loop('third', 4, 0, 0).distOffsetPerUnit * 1, 0.2, 'third K_p = 4 offset');
  const r = rng(11);
  let n = 0;
  ['first', 'second', 'third', 'motor'].forEach(function (id) {
    for (let i = 0; i < 30; i++) {
      const L = m.loop(id, r() * 20, 0.05 + r() * 19.95, r() < 0.5 ? 0 : r() * 10);
      if (L.stability !== 'stable') continue;
      near(L.distOffsetPerUnit, 0, id + ': offset with K_i > 0', 1e-15); n++;
    }
  });
  near(m.loop('motor', 2, 0, 0).distOffsetPerUnit, 0.5, 'motor P K_p = 2 offset');
  // the simulated output really settles at y_∞ + d₀·Y_d(0)
  const s = m.simulate('third', 4, 0, 0, { tEnd: 160, d0: 1 });
  near(s.y[600], 0.8 + 0.2, 'third K_p = 4, d₀ = 1: final y', 1e-4);
  const s2 = m.simulate('motor', 2, 0, 0, { tEnd: 100, d0: 1 });
  near(s2.y[600], 1.5, 'motor K_p = 2, d₀ = 1: final y', 1e-4);
  const s3 = m.simulate('motor', 2, 1, 0, { tEnd: 100, d0: 1 });
  const L3 = m.loop('motor', 2, 1, 0);
  assert.strictEqual(L3.stability, 'stable');
  near(s3.y[600], 1, 'motor PI: disturbance rejected', 1e-3);
  return n + ' stable PI/PID loops';
});

group('metrics on analytic responses', function () {
  // first order 1 − e^(−t): rise 10–90 % = ln 9, 2 % settling = ln 50, no overshoot
  const t = M.linspace(0, 15, 601), y = t.map(v => 1 - Math.exp(-v));
  const mt = m.metrics(t, y, 1);
  near(mt.tRise, Math.log(9), 'rise', 1e-3); near(mt.tSettle, Math.log(50), 'settle', 1e-3);
  assert.strictEqual(mt.overshootPct, 0); assert.ok(Number.isNaN(mt.tPeak), 'no peak time without overshoot');
  // not settled in the window → Infinity
  const t2 = M.linspace(0, 2, 101), y2 = t2.map(v => 1 - Math.exp(-v));
  assert.strictEqual(m.metrics(t2, y2, 1).tSettle, Infinity);
  // second order ζ = 0.5, ωₙ = 2: exact peak
  const z = 0.5, wn = 2, wd = wn * Math.sqrt(1 - z * z);
  const t3 = M.linspace(0, 15, 601), y3 = t3.map(v => 1 - Math.exp(-z * wn * v) * (Math.cos(wd * v) + z / Math.sqrt(1 - z * z) * Math.sin(wd * v)));
  const m3 = m.metrics(t3, y3, 1);
  near(m3.overshootPct, 100 * Math.exp(-Math.PI * z / Math.sqrt(1 - z * z)), 'overshoot ζ = 0.5', 5e-3);
  near(m3.tPeak, Math.PI / wd, 'peak time ζ = 0.5', 1e-3);
  // non-positive or NaN final value → NaN metrics
  assert.ok(Number.isNaN(m.metrics(t, y, NaN).overshootPct));
  assert.ok(Number.isNaN(m.metrics(t, y, 0).tRise));
});

// ---------------------------------------------------------------- independent RK4 simulation
// plant in controllable canonical form + integrator state z (ż = e) + derivative filter w (τ_f ẇ = e − w):
// u = K_p e + K_i z + (K_d/τ_f)(e − w), plant input v = u + d, e = 1 − y.
function rk4Sim(plantId, Kp, Ki, Kd, tEnd, n, d0, lamMax) {
  const P = m.plants[plantId], tau = 0.02;
  const a = P.den.map(v => v / P.den[0]), np = a.length - 1;
  const b = new Array(np).fill(0);                 // y = Σ b_k x_k, b_k = coefficient of s^k in num/den0
  P.num.slice().reverse().forEach((c, k) => { b[k] = c / P.den[0]; });
  const useD = Kd !== 0, N = np + 2;
  const dt = tEnd / n;
  const steps = Math.max(4, Math.ceil(dt / Math.min(dt / 4, 0.05 / Math.max(1, lamMax))));
  const h = dt / steps;
  let x = new Float64Array(N);
  const k1 = new Float64Array(N), k2 = new Float64Array(N), k3 = new Float64Array(N), k4 = new Float64Array(N), tmp = new Float64Array(N);
  function out(s) {
    let y = 0; for (let i = 0; i < np; i++) y += b[i] * s[i];
    const e = 1 - y;
    return { y: y, u: Kp * e + Ki * s[np] + (useD ? Kd / tau * (e - s[np + 1]) : 0), e: e };
  }
  function f(s, d, dst) {
    const o = out(s), v = o.u + d;
    for (let i = 0; i < np - 1; i++) dst[i] = s[i + 1];
    let acc = v; for (let i = 0; i < np; i++) acc -= a[np - i] * s[i];
    dst[np - 1] = acc;
    dst[np] = o.e;
    dst[np + 1] = useD ? (o.e - s[np + 1]) / tau : 0;
  }
  const ys = [], us = [];
  const kd = n / 2;                               // n even: t_d = tEnd/2 is a sample
  for (let k = 0; k <= n; k++) {
    const o = out(x); ys.push(o.y); us.push(o.u);
    if (k === n) break;
    const d = k >= kd ? d0 : 0;
    for (let j = 0; j < steps; j++) {
      f(x, d, k1); for (let i = 0; i < N; i++) tmp[i] = x[i] + h / 2 * k1[i];
      f(tmp, d, k2); for (let i = 0; i < N; i++) tmp[i] = x[i] + h / 2 * k2[i];
      f(tmp, d, k3); for (let i = 0; i < N; i++) tmp[i] = x[i] + h * k3[i];
      f(tmp, d, k4); for (let i = 0; i < N; i++) x[i] += h / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
    }
  }
  return { y: ys, u: us };
}

group('invariant: simulate() = independent RK4 state-space simulation (1e-4)', function () {
  const r = rng(2027);
  let worst = 0, count = 0;
  ['first', 'second', 'third', 'motor'].forEach(function (id) {
    let got = 0, guard = 0;
    while (got < 50 && guard++ < 2000) {
      const form = guard % 4;                     // cycle through P, PD, PI and PID forms
      const Kp = Math.round(r() * 400) / 20, Ki = form === 0 || form === 1 ? 0 : Math.round(r() * 400) / 20;
      const Kd = form === 0 || form === 2 ? 0 : Math.round(r() * 200) / 20;
      const L = m.loop(id, Kp, Ki, Kd);
      if (L.stability !== 'stable') continue;
      const tEnd = 2 + Math.round(r() * 16), n = 600, d0 = got % 2 ? Math.round(r() * 20) / 20 : 0;
      const lam = Math.max.apply(null, L.poles.map(p => Math.hypot(p.re, p.im)));
      const s = m.simulate(id, Kp, Ki, Kd, { tEnd: tEnd, n: n, d0: d0 });
      const q = rk4Sim(id, Kp, Ki, Kd, tEnd, n, d0, lam);
      for (let k = 0; k <= n; k++) {
        const ey = Math.abs(s.y[k] - q.y[k]), eu = Math.abs(s.u[k] - q.u[k]);
        worst = Math.max(worst, ey, eu);
        assert.ok(ey <= 1e-4 && eu <= 1e-4, id + ' Kp=' + Kp + ' Ki=' + Ki + ' Kd=' + Kd + ' d0=' + d0 + ' tEnd=' + tEnd +
          ' k=' + k + ': |Δy| = ' + ey + ', |Δu| = ' + eu);
      }
      got++; count++;
    }
    assert.strictEqual(got, 50, id + ': 50 stable gain sets');
  });
  // odd n: the disturbance step falls half-way between samples
  const s = m.simulate('second', 3, 2, 0.5, { tEnd: 10, n: 201, d0: 0.7 });
  const f = m.simulate('second', 3, 2, 0.5, { tEnd: 10, n: 402, d0: 0.7 });
  for (let k = 0; k <= 201; k++) near(s.y[k], f.y[2 * k], 'odd n grid k = ' + k, 1e-9);
  return count + ' sets, worst |Δ| = ' + worst.toExponential(2);
});

group('invariants: y(0) = 0, y∞ = 1 − e_ss, K_i > 0 ⇒ e_ss = 0, stability ↔ response', function () {
  const r = rng(99);
  let nStable = 0, nUnstable = 0;
  ['first', 'second', 'third', 'motor'].forEach(function (id) {
    for (let i = 0; i < 60; i++) {
      const Kp = r() * 20, Ki = i % 3 === 0 ? 0 : r() * 20, Kd = i % 2 ? 0 : r() * 10;
      const L = m.loop(id, Kp, Ki, Kd);
      const s0 = m.simulate(id, Kp, Ki, Kd, { tEnd: 5, n: 50, d0: 0.5 });
      assert.strictEqual(s0.y[0], 0, 'y(0) = 0');
      const sig = maxRe(L);
      if (L.stability === 'stable') {
        near(L.yFinal, 1 - L.ess, 'y∞ = 1 − e_ss', 1e-12);
        if (Ki > 0) near(L.ess, 0, 'K_i > 0 ⇒ e_ss = 0', 1e-15);
        if (id === 'motor') near(L.ess, 0, 'type 1 plant ⇒ e_ss = 0', 1e-15);
        // the response converges to y∞ (bounded)
        const T = Math.min(4000, 16 / -sig);
        const s = m.simulate(id, Kp, Ki, Kd, { tEnd: T, n: 2000 });
        near(s.yRef[2000], L.yFinal, id + ' converges to y∞', 1e-3 * Math.max(1, Math.abs(L.yFinal)));
        nStable++;
      } else if (L.stability === 'unstable' && sig > 0.01) {
        const T = 14 / sig;
        const s = m.simulate(id, Kp, Ki, Kd, { tEnd: T, n: 2000 });
        let early = 0, late = 0;
        for (let k = 0; k <= 2000; k++) {
          const v = Math.abs(s.yRef[k]);
          if (k <= 500) early = Math.max(early, v); else if (k >= 1000) late = Math.max(late, isFinite(v) ? v : Infinity);
        }
        assert.ok(late > 100 * Math.max(1, early), id + ' Kp=' + Kp + ' Ki=' + Ki + ' Kd=' + Kd + ': unstable loop grows (' + early + ' → ' + late + ')');
        nUnstable++;
      }
    }
  });
  // marginal: bounded sustained oscillation
  const s = m.simulate('third', 8, 0, 0, { tEnd: 60, n: 3000 });
  const amp = Math.max.apply(null, s.yRef.slice(1500).map(v => Math.abs(v - 8 / 9)));
  assert.ok(amp > 0.1 && amp < 2, 'marginal: sustained bounded oscillation');
  assert.ok(nStable > 50 && nUnstable > 10, 'enough samples: ' + nStable + ' stable, ' + nUnstable + ' unstable');
  return nStable + ' stable, ' + nUnstable + ' unstable checked';
});

// ---------------------------------------------------------------- regressions found by the verifier
// Reference values below come from an independent residue (partial-fraction) evaluation of y_r(t) with
// Durand–Kerner roots on a two-scale brute-force grid (step 0.002/|p_max| where fast poles act).
group('regression: sampled metrics never invent overshoot (fast filtered-derivative spike)', function () {
  // first, K_p = 5.8, K_d = 7.85, tEnd = 11: true overshoot 4.0426 % at t = 0.0215 s; the old parabola through
  // (0, 0) and the first two samples extrapolated to 17.02 %
  const L = m.loop('first', 5.8, 0, 7.85), s = m.simulate('first', 5.8, 0, 7.85, { tEnd: 11 });
  const mt = m.metrics(s.t, s.yRef, L.yFinal);
  assert.ok(mt.overshootPct <= 4.0427, 'sampled overshoot ' + mt.overshootPct + ' > true 4.0426');
  // first, K_p = 1.1, K_d = 3.9, tEnd = 14: true 51.107 %, old 69.71 %
  const L2 = m.loop('first', 1.1, 0, 3.9), s2 = m.simulate('first', 1.1, 0, 3.9, { tEnd: 14 });
  assert.ok(m.metrics(s2.t, s2.yRef, L2.yFinal).overshootPct <= 51.11, 'sampled overshoot (2)');
});

group('regression: panel metrics (refMetrics) resolve fast spikes, near touches and coarse grids', function () {
  let r = m.refMetrics('first', 5.8, 0, 7.85, 11, 600);
  near(r.overshootPct, 4.04263, 'spike overshoot', 2e-4); near(r.tPeak, 0.021470, 'spike peak time', 3e-5);
  near(r.tRise, 0.0042319, 'spike rise', 1e-6); near(r.tSettle, 0.952116, 'spike settling', 1e-5);
  // the response touches 90 % in a 0.02 s spike long before the slow climb (old panel: 2.955 s)
  r = m.refMetrics('first', 1.5, 3.35, 9.4, 39, 600);
  near(r.tRise, 0.0113598, 'rise time through the spike', 1e-6); near(r.overshootPct, 7.33340, 'overshoot', 1e-4);
  // the error leaves the 2 % band by only 0.002 % between two samples near t = 15.3 s (old panel: 14.41 s)
  r = m.refMetrics('third', 16.3, 0, 5.45, 44, 600);
  near(r.tSettle, 15.31754, 'settling with a near touch of the band', 1e-4);
  // coarse grid (dt = 0.077 s, ω_d = 5.9 rad/s): old panel 31.54 %
  r = m.refMetrics('second', 18.05, 0.85, 0.35, 46, 600);
  near(r.overshootPct, 31.66446, 'overshoot on a coarse grid', 1e-3); near(r.tPeak, 0.509910, 'peak time', 1e-4);
  near(r.tSettle, 20.19616, 'settling', 1e-3);
  // not stable or y∞ ≤ 0 → NaN
  assert.ok(Number.isNaN(m.refMetrics('third', 8, 0, 0, 15).overshootPct));
  assert.ok(Number.isNaN(m.refMetrics('third', 0, 0, 10, 15).tRise));
});

group('regression: poles listed closest to the jω-axis first', function () {
  // third, K_p = 10, K_i = 1: poles 0.112 ± j…, −0.093, −3.13 — the old order put the RHP pair first
  ['first', 'second', 'third', 'motor'].forEach(function (id) {
    [[10, 1, 0], [12, 2, 0], [4, 1, 2], [20, 20, 10], [1, 0, 0]].forEach(function (g) {
      const p = m.loop(id, g[0], g[1], g[2]).poles;
      for (let i = 1; i < p.length; i++) assert.ok(Math.abs(p[i].re) >= Math.abs(p[i - 1].re) - 1e-12, id + ' ' + g + ': order');
    });
  });
  const p = m.loop('third', 10, 1, 0).poles;
  near(p[0].re, -0.093, 'third K_p = 10, K_i = 1: first pole', 1e-3);
});

group('check values through the panel metrics (refMetrics)', function () {
  const P = m.plants.third, zd = m.zn(P.Ku, P.Pu, 'PID'), zi = m.zn(P.Ku, P.Pu, 'PI'), zp = m.zn(P.Ku, P.Pu, 'P');
  let r = m.refMetrics('second', 5, 0, 0); near(r.overshootPct, 22.11, 'second K_p = 5 overshoot');
  r = m.refMetrics('third', zd.Kp, zd.Ki, zd.Kd);
  near(r.overshootPct, 41.5, 'Z–N PID overshoot', 0.5); near(r.tPeak, 2.19, 'Z–N PID peak', 0.02); near(r.tSettle, 9.3, 'Z–N PID settling', 0.2);
  near(m.refMetrics('third', zi.Kp, zi.Ki, zi.Kd).overshootPct, 56.1, 'Z–N PI overshoot', 0.5);
  near(m.refMetrics('third', zp.Kp, zp.Ki, zp.Kd).overshootPct, 54.3, 'Z–N P overshoot', 0.5);
  r = m.refMetrics('motor', 1, 0, 0);
  near(r.overshootPct, 100 * Math.exp(-Math.PI / Math.sqrt(3)), 'motor K_p = 1 overshoot (exact)', 1e-6);
  near(r.tPeak, 2 * Math.PI / Math.sqrt(3), 'motor K_p = 1 peak time π/ω_d (exact)', 1e-6);
  near(m.refMetrics('motor', 4, 0, 0).overshootPct, 44.43, 'motor K_p = 4');
  near(m.refMetrics('motor', 4, 0, 3).overshootPct, 2.4, 'motor K_p = 4, K_d = 3', 0.3);
  r = m.refMetrics('first', 4, 0, 0);
  near(r.tRise, Math.log(9) / 5, 'first K_p = 4: rise ln 9/5', 1e-7); near(r.tSettle, Math.log(50) / 5, 'settling ln 50/5', 1e-7);
  assert.strictEqual(r.overshootPct, 0);
});

// independent: y_r(t) by residues, y(t) = T(0) + Σ Res[T(s)e^{st}/s] at simple poles, on a two-scale brute grid
group('invariant: refMetrics = brute-force metrics of the residue solution (60 random stable sets)', function () {
  const r = rng(31337);
  let got = 0, worst = { os: 0, tr: 0, ts: 0 };
  for (let guard = 0; got < 60 && guard < 3000; guard++) {
    const id = ['first', 'second', 'third', 'motor'][guard % 4];
    const Kp = Math.round(r() * 400) / 20, Ki = r() < 0.4 ? 0 : Math.round(r() * 60) / 20, Kd = r() < 0.4 ? 0 : Math.round(r() * 200) / 20;
    const tEnd = 2 + Math.round(r() * 58);
    const L = m.loop(id, Kp, Ki, Kd);
    if (L.stability !== 'stable' || !(L.yFinal > 0)) continue;
    const ps = L.poles;
    let sep = Infinity;
    ps.forEach((p, i) => { sep = Math.min(sep, Math.hypot(p.re, p.im)); ps.forEach((q, j) => { if (j > i) sep = Math.min(sep, Math.hypot(p.re - q.re, p.im - q.im)); }); });
    if (sep < 1e-3) continue;
    const dD = M.polyDeriv(L.T.den);
    const res = ps.map(p => M.cdiv(M.polyEval(L.T.num, p), M.cmul(p, M.polyEval(dD, p))));
    const f = t => { let y = L.yFinal; ps.forEach((p, i) => { y += M.cmul(res[i], M.cexp(M.cscale(p, t))).re; }); return y; };
    const lam = Math.max.apply(null, ps.map(p => Math.hypot(p.re, p.im)));
    const fast = ps.filter(p => Math.hypot(p.re, p.im) > 5).map(p => -p.re);
    const tF = Math.min(tEnd, fast.length ? 20 / Math.min.apply(null, fast) : 0);
    const ts = [];
    if (tF > 0) { const h = Math.min(0.002 / lam, tF / 1000); for (let t = 0; t < tF; t += h) ts.push(t); }
    for (let k = 0; k <= 200000; k++) { const t = k * tEnd / 200000; if (t >= tF) ts.push(t); }
    ts[ts.length - 1] = tEnd;
    const ys = ts.map(f), yF = L.yFinal;
    let im = 0; for (let k = 1; k < ys.length; k++) if (ys[k] > ys[im]) im = k;
    const cross = lv => { for (let k = 1; k < ys.length; k++) if (ys[k] >= lv) return ts[k - 1] + (lv - ys[k - 1]) / (ys[k] - ys[k - 1]) * (ts[k] - ts[k - 1]); return Infinity; };
    let last = -1; for (let k = ys.length - 1; k >= 0; k--) if (Math.abs(ys[k] - yF) > 0.02 * yF) { last = k; break; }
    const os = Math.max(0, (ys[im] - yF) / yF * 100);
    const t90 = cross(0.9 * yF), tr = isFinite(t90) ? t90 - cross(0.1 * yF) : Infinity;
    let tsl = Infinity;
    if (last < ys.length - 1) { const e0 = Math.abs(ys[last] - yF), e1 = Math.abs(ys[last + 1] - yF); tsl = ts[last] + (e0 - 0.02 * yF) / (e0 - e1) * (ts[last + 1] - ts[last]); }
    const q = m.refMetrics(id, Kp, Ki, Kd, tEnd, 600);
    const tag = id + ' ' + Kp + ',' + Ki + ',' + Kd + ' T=' + tEnd;
    near(q.overshootPct, os, tag + ' overshoot', 5e-3 + 1e-4 * os); worst.os = Math.max(worst.os, Math.abs(q.overshootPct - os));
    assert.strictEqual(isFinite(q.tRise), isFinite(tr), tag + ' rise finite');
    if (isFinite(tr)) { near(q.tRise, tr, tag + ' rise', 1e-4 * tr + 1e-5); worst.tr = Math.max(worst.tr, Math.abs(q.tRise - tr) / tr); }
    assert.strictEqual(isFinite(q.tSettle), isFinite(tsl), tag + ' settle finite');
    if (isFinite(tsl)) { near(q.tSettle, tsl, tag + ' settle', 1e-4 * tsl + 1e-5); worst.ts = Math.max(worst.ts, Math.abs(q.tSettle - tsl) / tsl); }
    got++;
  }
  assert.strictEqual(got, 60);
  return 'worst |Δos| = ' + worst.os.toExponential(1) + ' pts, rel Δt_r = ' + worst.tr.toExponential(1) + ', rel Δt_s = ' + worst.ts.toExponential(1);
});

group('loop type', function () {
  assert.strictEqual(m.loop('first', 2, 0, 0).type, 0);
  assert.strictEqual(m.loop('first', 2, 0, 1).type, 0);
  assert.strictEqual(m.loop('second', 2, 1, 0).type, 1);
  assert.strictEqual(m.loop('motor', 2, 0, 0).type, 1);
  assert.strictEqual(m.loop('motor', 2, 1, 1).type, 2);
});

if (failures) { console.log(failures + ' group(s) failed'); process.exit(1); }
console.log('all groups passed');
