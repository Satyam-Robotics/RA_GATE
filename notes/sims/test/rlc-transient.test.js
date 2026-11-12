// Model tests for rlc-transient. Run: node notes/sims/test/rlc-transient.test.js
'use strict';
const assert = require('assert');
const RASim = require('./load.js')('rlc-transient');
const m = RASim.sims['rlc-transient'].model;

let failures = 0;
function group(name, fn) {
  try { const extra = fn(); console.log('ok   ' + name + (extra ? '  (' + extra + ')' : '')); }
  catch (e) { failures++; console.log('FAIL ' + name + ': ' + (e && e.message || e)); }
}
// value printed with `dec` decimals in the specification: tolerance max(5e-4 relative, half a unit of the last digit)
function printed(actual, expected, dec, what) {
  const tol = Math.max(5e-4 * Math.abs(expected), 0.5 * Math.pow(10, -dec));
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
function logUniform(r, lo, hi) { return Math.pow(10, Math.log10(lo) + (Math.log10(hi) - Math.log10(lo)) * r()); }
// random parameter set over the slider ranges: V 1…20 V (step 1), R 1 Ω…10 kΩ, L 1…1000 mH, C 0.1…1000 µF (log)
function randomP(r) {
  return { V: 1 + Math.floor(20 * r()), R: logUniform(r, 1, 1e4), L: logUniform(r, 1e-3, 1), C: logUniform(r, 1e-7, 1e-3) };
}
const CIRCUITS = ['RC', 'RL', 'RLC'];
const P_RC = { V: 10, R: 1000, L: 0.1, C: 100e-6 };
const P_RL = { V: 10, R: 100, L: 0.1, C: 100e-6 };
const P_RLC = { V: 10, R: 20, L: 0.1, C: 100e-6 };

group('API shape and title', function () {
  assert.strictEqual(RASim.sims['rlc-transient'].title, 'RC / RL / RLC transient response');
  ['firstOrder', 'rlc', 'response', 'tEnd', 'peak'].forEach(k => assert.strictEqual(typeof m[k], 'function', 'model.' + k));
  assert.deepStrictEqual(Object.keys(m.firstOrder('RC', 1, 1, 1)), ['tau']);
  const r = m.rlc(20, 0.1, 100e-6);
  ['alpha', 'w0', 'f0', 'zeta', 'kind', 'wd', 's1', 's2', 'Rcrit', 'Q'].forEach(k => assert.ok(k in r, 'rlc.' + k));
  ['re', 'im'].forEach(k => { assert.ok(k in r.s1, 's1.' + k); assert.ok(k in r.s2, 's2.' + k); });
  CIRCUITS.forEach(function (c) {
    const q = m.response(c, 'charge', P_RLC, 0.001);
    ['vC', 'vR', 'vL', 'i'].forEach(k => assert.ok(k in q, c + ' response.' + k));
  });
  assert.strictEqual(m.response('RC', 'charge', P_RC, 0.01).vL, undefined, 'RC has no v_L');
  assert.strictEqual(m.response('RL', 'charge', P_RL, 0.001).vC, undefined, 'RL has no v_C');
  assert.strictEqual(m.response('RC', 'discharge', P_RC, 0.01).vL, undefined, 'RC discharge has no v_L');
  assert.strictEqual(m.response('RL', 'discharge', P_RL, 0.001).vC, undefined, 'RL discharge has no v_C');
  ['vC', 'vR', 'vL', 'i'].forEach(k => assert.ok(Number.isFinite(m.response('RLC', 'charge', P_RLC, 0.01)[k]), 'RLC ' + k + ' finite'));
  assert.ok(Number.isNaN(m.rlc(200, 0.1, 100e-6).wd), 'wd NaN unless under');
  assert.ok(Number.isNaN(m.rlc(2 * Math.sqrt(0.1 / 100e-6), 0.1, 100e-6).wd), 'wd NaN when critical');
  assert.strictEqual(m.peak({ V: 10, R: 200, L: 0.1, C: 100e-6 }), null, 'peak null when over');
  assert.strictEqual(m.peak({ V: 10, R: 2 * Math.sqrt(0.1 / 100e-6), L: 0.1, C: 100e-6 }), null, 'peak null when critical');
});

group('check: RC, R = 1000, C = 100 µF, V = 10 (charge and discharge)', function () {
  printed(m.firstOrder('RC', 1000, 0.1, 100e-6).tau, 0.1, 1, 'τ');
  printed(m.response('RC', 'charge', P_RC, 0.1).vC, 6.3212, 4, 'v_C(0.1)');
  printed(m.response('RC', 'charge', P_RC, 0.5).vC, 9.9326, 4, 'v_C(0.5)');
  printed(m.response('RC', 'charge', P_RC, 0).i, 0.010, 3, 'i(0)');
  printed(m.response('RC', 'discharge', P_RC, 0.1).vC, 3.6788, 4, 'discharge v_C(0.1)');
  printed(m.response('RC', 'discharge', P_RC, 0).i, -0.010, 3, 'discharge i(0⁺)');
  printed(m.tEnd('RC', P_RC), 0.6, 1, 'tEnd = 6τ');
  // 63.2 % at τ, more than 99 % at 5τ
  printed(m.response('RC', 'charge', P_RC, 0.1).vC / 10, 0.632, 3, '63.2 % at τ');
  assert.ok(m.response('RC', 'charge', P_RC, 0.5).vC / 10 > 0.99, '> 99 % at 5τ');
  printed(m.response('RC', 'charge', P_RC, Infinity).vC, 10, 6, 'v_C(∞)');
  printed(m.response('RC', 'discharge', P_RC, Infinity).vC, 0, 6, 'discharge v_C(∞)');
});

group('check: RL, R = 100, L = 0.1, V = 10 (charge and discharge)', function () {
  printed(m.firstOrder('RL', 100, 0.1, 100e-6).tau, 0.001, 4, 'τ');
  printed(m.response('RL', 'charge', P_RL, 0.001).i, 0.063212, 6, 'i(0.001)');
  printed(m.response('RL', 'charge', P_RL, Infinity).i, 0.1, 1, 'i(∞)');
  printed(m.response('RL', 'charge', P_RL, 0.05).i, 0.1, 6, 'i(50τ)');
  printed(m.response('RL', 'charge', P_RL, 0).vL, 10, 0, 'v_L(0)');
  printed(m.response('RL', 'discharge', P_RL, 0.001).i, 0.036788, 6, 'discharge i(0.001)');
  printed(m.response('RL', 'discharge', P_RL, 0).vL, -10, 0, 'discharge v_L(0⁺)');
  printed(m.tEnd('RL', P_RL), 0.006, 3, 'tEnd = 6τ');
});

group('check: RLC, R = 20, L = 0.1, C = 100 µF (under-damped)', function () {
  const r = m.rlc(20, 0.1, 100e-6);
  assert.strictEqual(r.kind, 'under');
  printed(r.alpha, 100, 0, 'α');
  printed(r.w0, 316.228, 3, 'ω₀');
  printed(r.f0, 316.228 / (2 * Math.PI), 4, 'f₀');
  printed(r.zeta, 0.31623, 5, 'ζ');
  near(r.wd, 300, 1e-9, 'ω_d = 300 exactly');
  near(r.s1.re, -100, 1e-9, 's1.re'); near(r.s1.im, 300, 1e-9, 's1.im');
  near(r.s2.re, -100, 1e-9, 's2.re'); near(r.s2.im, -300, 1e-9, 's2.im');
  printed(r.Rcrit, 63.2456, 4, 'R_crit');
  printed(r.Q, 1.5811, 4, 'Q');
  const pk = m.peak(P_RLC);
  printed(pk.tp, 0.010472, 6, 't_p');
  printed(pk.vCmax, 13.5092, 4, 'v_C,max');
  printed(pk.overshootPct, 35.09, 2, 'overshoot %');
  printed(m.response('RLC', 'charge', P_RLC, pk.tp).vC, 13.5092, 4, 'v_C(t_p) from response');
  // the current maximum, found from response() alone (fine grid, then golden-section refinement)
  const T = m.tEnd('RLC', P_RLC);
  const f = t => m.response('RLC', 'charge', P_RLC, t).i;
  let kb = 0, best = -Infinity;
  for (let k = 0; k <= 20000; k++) { const v = f(T * k / 20000); if (v > best) { best = v; kb = k; } }
  let a = T * (kb - 1) / 20000, b = T * (kb + 1) / 20000;
  const gr = (Math.sqrt(5) - 1) / 2;
  for (let n = 0; n < 100; n++) { const c = b - gr * (b - a), d = a + gr * (b - a); if (f(c) > f(d)) b = d; else a = c; }
  const tm = (a + b) / 2;
  printed(f(tm), 0.20854, 5, 'i_max');
  printed(tm, 0.004163, 6, 't at i_max');
  // the v_C maximum from response() equals peak()
  let vmax = -Infinity;
  for (let k = 0; k <= 20000; k++) vmax = Math.max(vmax, m.response('RLC', 'charge', P_RLC, T * k / 20000).vC);
  near(vmax, pk.vCmax, 1e-5, 'sampled max v_C = peak().vCmax');
  near(m.tEnd('RLC', P_RLC), Math.min(6 / 100, 20 * 2 * Math.PI / 300), 1e-12, 'tEnd under = min(6/α, 20·2π/ω_d)');
});

group('check: RLC, R = 200 (over-damped)', function () {
  const r = m.rlc(200, 0.1, 100e-6);
  assert.strictEqual(r.kind, 'over');
  printed(r.zeta, 3.1623, 4, 'ζ');
  printed(r.s1.re, -51.317, 3, 's₁');
  printed(r.s2.re, -1948.683, 3, 's₂');
  assert.strictEqual(r.s1.im, 0); assert.strictEqual(r.s2.im, 0);
  assert.ok(Math.abs(r.s1.re) < Math.abs(r.s2.re), 's₁ nearer the origin');
  near(r.s1.re * r.s2.re, r.w0 * r.w0, 1e-9 * r.w0 * r.w0, 's₁s₂ = ω₀²');
  near(r.s1.re + r.s2.re, -2 * r.alpha, 1e-9 * r.alpha, 's₁ + s₂ = −2α');
  assert.strictEqual(m.peak({ V: 10, R: 200, L: 0.1, C: 100e-6 }), null);
  near(m.tEnd('RLC', { V: 10, R: 200, L: 0.1, C: 100e-6 }), 6 / 51.317, 1e-4 * 6 / 51.317, 'tEnd over = 6/|s₁|');
});

group('check: RLC, R = 63.2455532 (critical)', function () {
  const p = { V: 10, R: 63.2455532, L: 0.1, C: 100e-6 };
  const r = m.rlc(p.R, p.L, p.C);
  assert.strictEqual(r.kind, 'critical');
  printed(r.alpha, 316.228, 3, 'α');
  near(r.s1.re, -r.alpha, 0, 's₁ = −α'); near(r.s2.re, -r.alpha, 0, 's₂ = −α');
  assert.strictEqual(r.s1.im, 0); assert.strictEqual(r.s2.im, 0);
  printed(m.response('RLC', 'charge', p, 1 / r.alpha).vC, 2.6424, 4, 'v_C(1/α)');
  near(m.tEnd('RLC', p), 8 / r.w0, 1e-15, 'tEnd critical = 8/ω₀');
  // classification thresholds
  assert.strictEqual(m.rlc(p.R * (1 + 5e-7), p.L, p.C).kind, 'critical');
  assert.strictEqual(m.rlc(p.R * (1 + 2e-6), p.L, p.C).kind, 'over');
  assert.strictEqual(m.rlc(p.R * (1 - 2e-6), p.L, p.C).kind, 'under');
});

// ------------------------------------------------------------------ invariants
const R0 = rng(20270101);
const SETS = [];
for (let n = 0; n < 300; n++) SETS.push(randomP(R0));
const FRACS = [0, 0.003, 0.05, 0.2, 0.5, 0.9, 1];

group('invariant: charge starts from zero energy and settles (300 random sets)', function () {
  let n = 0;
  SETS.forEach(function (p) {
    CIRCUITS.forEach(function (c) {
      const q0 = m.response(c, 'charge', p, 0);
      if (c !== 'RL') near(q0.vC, 0, 1e-12 * p.V, c + ' v_C(0) = 0');
      if (c !== 'RC') near(q0.i, 0, 1e-15, c + ' i(0) = 0');
      // slowest decay rate → wait 40 time constants
      let rate;
      if (c === 'RC') rate = 1 / (p.R * p.C);
      else if (c === 'RL') rate = p.R / p.L;
      else { const r = m.rlc(p.R, p.L, p.C); rate = r.kind === 'over' ? Math.abs(r.s1.re) : r.alpha; }
      const qf = m.response(c, 'charge', p, 40 / rate);
      if (c !== 'RL') near(qf.vC, p.V, 1e-6 * p.V, c + ' v_C → V');
      else near(qf.i, p.V / p.R, 1e-6 * p.V / p.R, 'RL i → V/R');
      if (c === 'RLC') near(qf.i, 0, 1e-6 * p.V / p.R + 1e-6 * p.V * Math.sqrt(p.C / p.L), 'RLC i → 0');
      n++;
    });
  });
  return n + ' cases';
});

group('invariant: KVL v_R + v_L + v_C = V (charge) and 0 (discharge) to 1e-9·V', function () {
  let n = 0;
  SETS.forEach(function (p) {
    CIRCUITS.forEach(function (c) {
      const T = m.tEnd(c, p);
      FRACS.forEach(function (fr) {
        ['charge', 'discharge'].forEach(function (mode) {
          const q = m.response(c, mode, p, fr * T);
          const sum = q.vR + (q.vL === undefined ? 0 : q.vL) + (q.vC === undefined ? 0 : q.vC);
          near(sum, mode === 'charge' ? p.V : 0, 1e-9 * p.V, c + ' ' + mode + ' KVL at t = ' + fr + '·tEnd');
          near(q.vR, q.i * p.R, 1e-12 * p.V, c + ' v_R = iR');
          n++;
        });
      });
    });
  });
  return n + ' checks';
});

group('invariant: i = C·dv_C/dt and v_L = L·di/dt (central differences, relative 1e-4)', function () {
  let n = 0;
  SETS.forEach(function (p) {
    CIRCUITS.forEach(function (c) {
      const T = m.tEnd(c, p);
      ['charge', 'discharge'].forEach(function (mode) {
        // scales: largest |i| and |v_L| on a grid over the window
        let iS = 0, vS = 0;
        for (let k = 0; k <= 400; k++) {
          const q = m.response(c, mode, p, T * k / 400);
          iS = Math.max(iS, Math.abs(q.i));
          if (q.vL !== undefined) vS = Math.max(vS, Math.abs(q.vL));
        }
        [0.003, 0.05, 0.2, 0.5, 0.9].forEach(function (fr) {
          const t = fr * T, h = 1e-5 * t;
          const a = m.response(c, mode, p, t - h), b = m.response(c, mode, p, t + h), q = m.response(c, mode, p, t);
          if (c !== 'RL') {
            const d = p.C * (b.vC - a.vC) / (2 * h);
            near(d, q.i, 1e-4 * Math.max(Math.abs(q.i), iS), c + ' ' + mode + ' i = C dv_C/dt at ' + fr + '·tEnd');
            n++;
          }
          if (c !== 'RC') {
            const d = p.L * (b.i - a.i) / (2 * h);
            near(d, q.vL, 1e-4 * Math.max(Math.abs(q.vL), vS), c + ' ' + mode + ' v_L = L di/dt at ' + fr + '·tEnd');
            n++;
          }
        });
      });
    });
  });
  return n + ' checks';
});

group('invariant: continuity across the damping classes (ζ = 1 ± 1e-5 vs ζ = 1, v_C to 1e-4·V)', function () {
  const r = rng(77);
  let n = 0;
  for (let s = 0; s < 300; s++) {
    const p = randomP(r);
    const Rc = 2 * Math.sqrt(p.L / p.C);
    const pc = Object.assign({}, p, { R: Rc }), pu = Object.assign({}, p, { R: Rc * (1 - 1e-5) }), po = Object.assign({}, p, { R: Rc * (1 + 1e-5) });
    assert.strictEqual(m.rlc(pc.R, p.L, p.C).kind, 'critical');
    assert.strictEqual(m.rlc(pu.R, p.L, p.C).kind, 'under');
    assert.strictEqual(m.rlc(po.R, p.L, p.C).kind, 'over');
    const T = 8 / m.rlc(Rc, p.L, p.C).w0;
    for (let k = 0; k <= 20; k++) {
      const t = T * k / 20;
      ['charge', 'discharge'].forEach(function (mode) {
        const vc = m.response('RLC', mode, pc, t).vC;
        near(m.response('RLC', mode, pu, t).vC, vc, 1e-4 * p.V, 'under vs critical at k = ' + k);
        near(m.response('RLC', mode, po, t).vC, vc, 1e-4 * p.V, 'over vs critical at k = ' + k);
        n += 2;
      });
    }
  }
  return n + ' checks';
});

group('invariant: discharge = steady − charge (every quantity)', function () {
  let n = 0;
  SETS.forEach(function (p) {
    CIRCUITS.forEach(function (c) {
      const T = m.tEnd(c, p);
      const st = c === 'RC' ? { vC: p.V, vR: 0, vL: undefined, i: 0 }
        : c === 'RL' ? { vC: undefined, vR: p.V, vL: 0, i: p.V / p.R } : { vC: p.V, vR: 0, vL: 0, i: 0 };
      FRACS.forEach(function (fr) {
        const ch = m.response(c, 'charge', p, fr * T), di = m.response(c, 'discharge', p, fr * T);
        ['vC', 'vR', 'vL', 'i'].forEach(function (k) {
          if (st[k] === undefined) { assert.strictEqual(di[k], undefined, c + ' ' + k + ' absent'); return; }
          const scale = k === 'i' ? p.V / p.R + p.V * Math.sqrt(p.C / p.L) : p.V;
          near(di[k], st[k] - ch[k], 1e-12 * scale, c + ' ' + k + ' at ' + fr + '·tEnd');
          n++;
        });
      });
      // before the switch: steady state (discharge) / zero (charge)
      const pre = m.response(c, 'discharge', p, -1);
      ['vC', 'vR', 'vL', 'i'].forEach(function (k) { assert.strictEqual(pre[k], st[k], c + ' discharge t < 0: ' + k); });
    });
  });
  return n + ' checks';
});

group('invariant: no jump of v_C and inductor current at t = 0 (charge and discharge)', function () {
  SETS.slice(0, 100).forEach(function (p) {
    CIRCUITS.forEach(function (c) {
      ['charge', 'discharge'].forEach(function (mode) {
        const before = m.response(c, mode, p, -1e-12), after = m.response(c, mode, p, 0);
        if (c !== 'RL') near(after.vC, before.vC, 1e-12 * p.V, c + ' ' + mode + ' v_C continuous');
        if (c !== 'RC') near(after.i, before.i, 1e-12 * p.V / p.R, c + ' ' + mode + ' i continuous');
      });
    });
  });
});

group('invariant: rlc() relations, peak() and tEnd() over random sets', function () {
  let under = 0, over = 0;
  SETS.forEach(function (p) {
    const r = m.rlc(p.R, p.L, p.C);
    near(r.zeta, r.alpha / r.w0, 1e-12 * r.zeta, 'ζ = α/ω₀');
    near(r.Q, 1 / (2 * r.zeta), 1e-12 * r.Q, 'Q = 1/2ζ');
    near(r.Q, Math.sqrt(p.L / p.C) / p.R, 1e-12 * r.Q, 'Q = (1/R)√(L/C)');
    near(r.f0, r.w0 / (2 * Math.PI), 1e-12 * r.f0, 'f₀');
    near(r.Rcrit, 2 * Math.sqrt(p.L / p.C), 1e-12 * r.Rcrit, 'R_crit');
    const T = m.tEnd('RLC', p);
    assert.ok(T > 0 && Number.isFinite(T), 'tEnd finite');
    if (r.kind === 'under') {
      under++;
      near(Math.hypot(r.s1.re, r.s1.im), r.w0, 1e-9 * r.w0, 'under poles on the circle |s| = ω₀');
      near(T, Math.min(6 / r.alpha, 40 * Math.PI / r.wd), 1e-12 * T, 'tEnd under');
      const pk = m.peak(p);
      near(m.response('RLC', 'charge', p, pk.tp).vC, pk.vCmax, 1e-9 * p.V, 'v_C(t_p) = v_C,max');
      near(pk.overshootPct, 100 * (pk.vCmax - p.V) / p.V, 1e-9, 'overshoot');
      near(pk.overshootPct, 100 * Math.exp(-Math.PI * r.zeta / Math.sqrt(1 - r.zeta * r.zeta)), 1e-9 * 100, 'overshoot from ζ');
    } else if (r.kind === 'over') {
      over++;
      near(T, 6 / Math.abs(r.s1.re), 1e-12 * T, 'tEnd over');
      near(r.s1.re * r.s2.re, r.w0 * r.w0, 1e-9 * r.w0 * r.w0, 's₁s₂ = ω₀²');
      assert.ok(r.s2.re <= r.s1.re && r.s1.re < 0, 'order of the real poles');
    }
    ['RC', 'RL'].forEach(function (c) {
      const tau = m.firstOrder(c, p.R, p.L, p.C).tau;
      near(tau, c === 'RC' ? p.R * p.C : p.L / p.R, 0, c + ' τ');
      near(m.tEnd(c, p), 6 * tau, 1e-15 * tau, c + ' tEnd = 6τ');
      const x = c === 'RC' ? 'vC' : 'i', xi = c === 'RC' ? p.V : p.V / p.R;
      near(m.response(c, 'charge', p, tau)[x], (1 - Math.exp(-1)) * xi, 1e-12 * xi, c + ' 63.2 % at τ');
      near(m.response(c, 'discharge', p, tau)[x], Math.exp(-1) * xi, 1e-12 * xi, c + ' 36.8 % at τ');
    });
  });
  return under + ' under, ' + over + ' over';
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
  const s = await openSim('rlc-transient', { width: 1100, outDir: path.join(os.tmpdir(), 'rasim-rlc-transient-test') });
  const page = s.page;
  async function frames() { await page.evaluate(() => new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r)))); }
  async function setS(st) { await page.evaluate(st => RASim.instances['rlc-transient'].setState(st), st); await frames(); }
  const stat = k => page.textContent('[data-stat="' + k + '"] .sim-stat-value');
  try {
    await agroup('browser: pole-map locus arrows are arcs along |s| = ω₀ ending in heads towards 180° (regression)', async function () {
      // The first version drew lone, nearly equilateral heads at 140° and 220° that read as pointing to the right.
      await setS({ circuit: 'RLC', R: 20, LmH: 100, CuF: 100 });
      const res = await page.evaluate(() => {
        const cv = document.querySelector('[data-canvas="poles"]');
        const r = cv.getBoundingClientRect(), W = r.width - 2, H = r.height - 2, w0 = 1 / Math.sqrt(0.1 * 100e-6);
        const pp = RASim.plot(document.createElement('canvas').getContext('2d'), { box: { x: 0, y: 0, w: W, h: H },
          margin: { left: 48, right: 10, top: 10, bottom: 34 }, xmin: -2.4 * w0, xmax: 1.15 * w0, ymin: -1.3 * w0, ymax: 1.3 * w0, equal: true });
        const dpr = cv.width / W, img = cv.getContext('2d').getImageData(0, 0, cv.width, cv.height);
        function teal(x, y) {
          const i = 4 * (Math.round(y * dpr) * cv.width + Math.round(x * dpr)), d = img.data;
          return d[i + 1] - d[i] > 30 && d[i + 2] - d[i] > 25;    // teal (0x0f766e) on white: green and blue well above red
        }
        function count(deg, rad) {   // teal pixels in a (2·rad+1)² box at a circle angle (degrees, as seen)
          const th = deg * Math.PI / 180, cx = pp.X(w0 * Math.cos(th)), cy = pp.Y(w0 * Math.sin(th));
          let n = 0;
          for (let dx = -rad; dx <= rad; dx++) for (let dy = -rad; dy <= rad; dy++) if (teal(cx + dx, cy + dy)) n++;
          return n;
        }
        return { up: [100, 120, 130, 140, 147].map(d => count(d, 2)), lo: [260, 240, 230, 220, 213].map(d => count(d, 2)),
          upEnds: [count(114, 5), count(147, 5)], loEnds: [count(246, 5), count(213, 5)] };
      });
      assert.strictEqual(res.up[0], 0, 'no arc before 112°: ' + res.up);
      assert.ok(res.up.slice(1).every(n => n > 0), 'upper arc drawn from 112° to 148°: ' + res.up);
      assert.strictEqual(res.lo[0], 0, 'no arc beyond 248°: ' + res.lo);
      assert.ok(res.lo.slice(1).every(n => n > 0), 'lower arc drawn from 248° to 212°: ' + res.lo);
      assert.ok(res.upEnds[1] > 1.5 * res.upEnds[0], 'upper head at the 148° end (towards 180°): ' + res.upEnds);
      assert.ok(res.loEnds[1] > 1.5 * res.loEnds[0], 'lower head at the 212° end (towards 180°): ' + res.loEnds);
    });
    await agroup('browser: a small overshoot stays visible in the peak read-out (regression)', async function () {
      // R = 60 Ω: v_C,max = 10.000807 V was shown as "10 V" next to "(0.00807 %)".
      await setS({ circuit: 'RLC', mode: 'charge', V: 10, R: 60, LmH: 100, CuF: 100 });
      const pk = m.peak({ V: 10, R: 60, L: 0.1, C: 100e-6 });
      const t = await stat('peak');
      assert.ok(t.indexOf('10.00081 V at ') === 0, 'peak read-out: ' + t);
      assert.ok(t.indexOf('(' + RASim.fmt(pk.overshootPct, 4) + ' %)') > 0, 'overshoot: ' + t);
      await setS({ R: 20 });
      assert.strictEqual(await stat('peak'), '13.51 V at 10.47 ms (35.09 %)');
    });
    await agroup('browser: an overshoot below the formatter cut-off reads as negligible, not 0 % (integration regression)', async function () {
      // R = 63 Ω (just under R_crit = 63.246 Ω): the true overshoot ≈ 4e-14 % printed as "(0 %)" next to UNDERDAMPED.
      await setS({ circuit: 'RLC', mode: 'charge', V: 10, R: 63, LmH: 100, CuF: 100 });
      const pk = m.peak({ V: 10, R: 63, L: 0.1, C: 100e-6 });
      assert.ok(pk.overshootPct > 0 && pk.overshootPct < 1e-12, 'precondition: tiny positive overshoot ' + pk.overshootPct);
      const t = await stat('peak');
      assert.ok(t.indexOf('(< 10⁻¹² %: negligible)') > 0 && t.indexOf('(0 %)') < 0, 'peak read-out: ' + t);
      assert.strictEqual(await page.textContent('[data-tag="kind"]'), 'UNDERDAMPED');
      // review regression: closer still (typed R = 63.245 Ω) e^(−απ/ω_d) underflows to exactly 0 while still under-damped
      await setS({ R: 63.245 });
      const pk0 = m.peak({ V: 10, R: 63.245, L: 0.1, C: 100e-6 });
      assert.ok(pk0 && pk0.overshootPct === 0, 'precondition: under-damped with an overshoot that underflows to 0');
      const t0 = await stat('peak');
      assert.ok(t0.indexOf('(< 10⁻¹² %: negligible)') > 0 && t0.indexOf('(0 %)') < 0, 'peak read-out at 63.245 Ω: ' + t0);
      assert.strictEqual(await page.textContent('[data-tag="kind"]'), 'UNDERDAMPED');
      await setS({ R: 60 });
      assert.ok((await stat('peak')).indexOf('(' + RASim.fmt(m.peak({ V: 10, R: 60, L: 0.1, C: 100e-6 }).overshootPct, 4) + ' %)') > 0, 'normal overshoot unchanged');
    });
    await agroup('browser: read-outs show the check values', async function () {
      await setS({ circuit: 'RLC', mode: 'charge', V: 10, R: 20, LmH: 100, CuF: 100 });
      assert.strictEqual(await stat('w0'), '316.23 rad/s'); assert.strictEqual(await stat('alpha'), '100 s⁻¹');
      assert.strictEqual(await stat('zeta'), '0.31623'); assert.strictEqual(await stat('wd'), '300 rad/s');
      assert.strictEqual(await stat('poles'), '−100 ± j300'); assert.strictEqual(await stat('Rcrit'), '63.246 Ω');
      assert.strictEqual(await stat('Q'), '1.5811'); assert.strictEqual(await page.textContent('[data-tag="kind"]'), 'UNDERDAMPED');
      await setS({ R: 200 });
      assert.strictEqual(await stat('zeta'), '3.1623'); assert.strictEqual(await stat('poles'), '−51.317,  −1948.7');
      assert.strictEqual(await page.textContent('[data-tag="kind"]'), 'OVERDAMPED'); assert.strictEqual(await stat('peak'), '—');
      await setS({ R: 63.2455532 });
      assert.strictEqual(await page.textContent('[data-tag="kind"]'), 'CRITICALLY DAMPED'); assert.strictEqual(await stat('zeta'), '1');
      await setS({ circuit: 'RC', mode: 'charge', R: 1000, CuF: 100 });
      assert.strictEqual(await stat('tau'), '100 ms'); assert.strictEqual(await stat('atTau'), 'vC(τ) = 6.321 V (63.2 %)');
      await setS({ circuit: 'RL', mode: 'discharge', R: 100, LmH: 100 });
      assert.strictEqual(await stat('tau'), '1 ms'); assert.strictEqual(await stat('atTau'), 'i(τ) = 36.79 mA (36.8 %)');
      assert.strictEqual(await stat('x0'), 'i(0⁺) = 100 mA');
    });
    assert.ok(s.errors.length === 0 && s.warnings.length === 0, 'console: ' + s.errors.concat(s.warnings).join('; '));
  } finally { await s.browser.close(); }
}

(async function () {
  try { await browserTests(); } catch (e) { failures++; console.log('FAIL browser: ' + (e && e.message || e)); }
  if (failures) { console.log(failures + ' group(s) FAILED'); process.exit(1); }
  console.log('all groups passed');
})();
