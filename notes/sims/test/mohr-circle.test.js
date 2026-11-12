// Tests for mohr-circle. Run: node notes/sims/test/mohr-circle.test.js [--no-browser] [--file <sim file>]
'use strict';
const assert = require('assert');
// optional: --file <path> runs the same tests against another copy of the simulation file
const FILE_ARG = process.argv.indexOf('--file') >= 0 ? require('path').resolve(process.argv[process.argv.indexOf('--file') + 1]) : undefined;
const RASim = require('./load.js')('mohr-circle', FILE_ARG);
const m = RASim.sims['mohr-circle'].model;
const D2R = Math.PI / 180;

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
const deg = r => r / D2R;

group('API shape and title', function () {
  assert.strictEqual(RASim.sims['mohr-circle'].title, "Mohr's circle for plane stress");
  assert.deepStrictEqual(Object.keys(m).sort(), ['circlePoints', 'principal', 'transform']);
  const t = m.transform(50, -10, 40, 0);
  assert.deepStrictEqual(Object.keys(t).sort(), ['sx1', 'sy1', 'txy1']);
  const p = m.principal(50, -10, 40);
  ['avg', 'R', 's1', 's2', 'thetaP', 'thetaS', 'tauMax', 'tauAbs', 'vonMises', 'tresca', 'isotropic'].forEach(k => assert.ok(k in p, 'principal.' + k));
  assert.strictEqual(typeof p.isotropic, 'boolean');
  const c = m.circlePoints(50, -10, 40, 0);
  ['C', 'X', 'Y', 'X1', 'Y1'].forEach(k => assert.ok(Array.isArray(c[k]) && c[k].length === 2, 'circlePoints.' + k));
});

group('general state (50, −10, 40)', function () {
  const p = m.principal(50, -10, 40);
  printed(p.avg, 20, 0, 'σ_avg'); printed(p.R, 50, 0, 'R');
  printed(p.s1, 70, 0, 'σ₁'); printed(p.s2, -30, 0, 'σ₂');
  printed(deg(p.thetaP), 26.565, 3, 'θ_p'); printed(deg(p.thetaS), -18.435, 3, 'θ_s');
  printed(p.tauMax, 50, 0, 'τ_max'); printed(p.tauAbs, 50, 0, 'τ_abs');
  printed(p.vonMises, 88.882, 3, 'von Mises'); printed(p.tresca, 100, 0, 'Tresca');
  assert.strictEqual(p.isotropic, false);
  let t = m.transform(50, -10, 40, p.thetaP);
  printed(t.sx1, 70, 0, 'σ_x′ at θ_p'); printed(t.sy1, -30, 0, 'σ_y′ at θ_p'); printed(t.txy1, 0, 0, 'τ at θ_p');
  t = m.transform(50, -10, 40, 45 * D2R);
  printed(t.sx1, 60, 0, 'σ_x′ at 45°'); printed(t.sy1, -20, 0, 'σ_y′ at 45°'); printed(t.txy1, -30, 0, 'τ at 45°');
  t = m.transform(50, -10, 40, p.thetaS);
  printed(t.sx1, 20, 0, 'σ_x′ at θ_s'); printed(t.sy1, 20, 0, 'σ_y′ at θ_s'); printed(t.txy1, 50, 0, 'τ at θ_s (+R)');
  t = m.transform(50, -10, 40, 0);
  near(t.sx1, 50, 1e-12, 'σ_x′ at 0'); near(t.sy1, -10, 1e-12, 'σ_y′ at 0'); near(t.txy1, 40, 1e-12, 'τ at 0');
});

group('pure shear (0, 0, 50)', function () {
  const p = m.principal(0, 0, 50);
  printed(p.s1, 50, 0, 'σ₁'); printed(p.s2, -50, 0, 'σ₂');
  printed(deg(p.thetaP), 45, 0, 'θ_p'); printed(deg(p.thetaS), 0, 0, 'θ_s');
});

group('uniaxial (80, 0, 0)', function () {
  const p = m.principal(80, 0, 0);
  printed(p.s1, 80, 0, 'σ₁'); printed(p.s2, 0, 0, 'σ₂'); printed(deg(p.thetaP), 0, 0, 'θ_p');
  printed(p.tauMax, 40, 0, 'τ_max'); printed(deg(p.thetaS), -45, 0, 'θ_s');
  const t = m.transform(80, 0, 0, p.thetaS);
  printed(t.sx1, 40, 0, 'σ_x′ at θ_s'); printed(t.sy1, 40, 0, 'σ_y′ at θ_s'); printed(t.txy1, 40, 0, 'τ at θ_s');
});

group('thin cylinder (80, 40, 0)', function () {
  const p = m.principal(80, 40, 0);
  printed(p.tauMax, 20, 0, 'in-plane τ_max'); printed(p.tauAbs, 40, 0, 'τ_abs');
  printed(p.tresca, 80, 0, 'Tresca'); printed(p.vonMises, 69.282, 3, 'von Mises');
});

group('equal biaxial (60, 60, 0)', function () {
  const p = m.principal(60, 60, 0);
  printed(p.R, 0, 0, 'R'); assert.strictEqual(p.isotropic, true, 'isotropic');
  assert.strictEqual(p.thetaP, 0, 'θ_p reported as 0'); printed(p.tauAbs, 30, 0, 'τ_abs');
  assert.strictEqual(m.principal(50, -10, 40).isotropic, false);
  assert.strictEqual(m.principal(60, 60, 1e-7).isotropic, false, 'R = 1e-7 is not isotropic');
});

group('biaxial compression (−40, −80, 0)', function () {
  const p = m.principal(-40, -80, 0);
  printed(p.s1, -40, 0, 'σ₁'); printed(p.s2, -80, 0, 'σ₂');
  printed(p.tauMax, 20, 0, 'τ_max'); printed(p.tauAbs, 40, 0, 'τ_abs');
});

group('(20, 80, −30)', function () {
  const p = m.principal(20, 80, -30);
  printed(p.avg, 50, 0, 'σ_avg'); printed(p.R, 42.426, 3, 'R');
  printed(p.s1, 92.426, 3, 'σ₁'); printed(p.s2, 7.574, 3, 'σ₂'); printed(deg(p.thetaP), -67.5, 1, 'θ_p');
});

group('θ_p range (−90°, 90°] and its edge cases', function () {
  near(deg(m.principal(0, 80, 0).thetaP), 90, 1e-12, 'σ_y > σ_x, τ = 0 → θ_p = 90°');
  near(deg(m.principal(0, 80, -0).thetaP), 90, 1e-12, 'τ = −0 → still 90°');
  near(deg(m.principal(80, 0, -0).thetaP), 0, 1e-12, 'σ_x > σ_y → 0');
  near(deg(m.principal(0, 0, -50).thetaP), -45, 1e-12, 'pure negative shear → −45°');
  const r = rng(7);
  for (let i = 0; i < 1000; i++) {
    const sx = 200 * r() - 100, sy = 200 * r() - 100, t = 200 * r() - 100;
    const p = m.principal(sx, sy, t);
    assert.ok(p.thetaP > -Math.PI / 2 && p.thetaP <= Math.PI / 2, 'θ_p in range: ' + p.thetaP);
    near(p.thetaS, p.thetaP - Math.PI / 4, 1e-15, 'θ_s = θ_p − 45°');
    assert.ok(p.s1 >= p.s2, 'σ₁ ≥ σ₂');
  }
});

group('circlePoints definition', function () {
  const r = rng(11);
  for (let i = 0; i < 1000; i++) {
    const sx = 200 * r() - 100, sy = 200 * r() - 100, t = 200 * r() - 100, th = Math.PI * (r() - 0.5);
    const c = m.circlePoints(sx, sy, t, th), T = m.transform(sx, sy, t, th);
    assert.deepStrictEqual(c.C, [(sx + sy) / 2, 0]);
    assert.deepStrictEqual(c.X, [sx, t]); assert.deepStrictEqual(c.Y, [sy, -t]);
    assert.deepStrictEqual(c.X1, [T.sx1, T.txy1]); assert.deepStrictEqual(c.Y1, [T.sy1, -T.txy1]);
  }
  assert.ok(!Object.is(m.circlePoints(1, 2, 0, 0).Y[1], -0), 'no −0');
});

group('invariants (1000 random states and angles)', function () {
  const r = rng(2027);
  let worst = 0;
  for (let i = 0; i < 1000; i++) {
    const sx = 200 * r() - 100, sy = 200 * r() - 100, t = 200 * r() - 100;
    const th = 2 * Math.PI * (r() - 0.5);
    const T = m.transform(sx, sy, t, th);
    const tol = 1e-9 * 300;
    near(T.sx1 + T.sy1, sx + sy, tol, 'σ_x′ + σ_y′ invariant');
    near(T.sx1 * T.sy1 - T.txy1 * T.txy1, sx * sy - t * t, 1e-9 * 9e4, 'σ_x′σ_y′ − τ² invariant');
    const T180 = m.transform(sx, sy, t, th + Math.PI);
    near(T180.sx1, T.sx1, tol, 'θ+180 σ_x′'); near(T180.sy1, T.sy1, tol, 'θ+180 σ_y′'); near(T180.txy1, T.txy1, tol, 'θ+180 τ');
    const T90 = m.transform(sx, sy, t, th + Math.PI / 2);
    near(T90.sx1, T.sy1, tol, 'θ+90 swaps σ'); near(T90.sy1, T.sx1, tol, 'θ+90 swaps σ'); near(T90.txy1, -T.txy1, tol, 'θ+90 reverses τ');
    const p = m.principal(sx, sy, t);
    const Tp = m.transform(sx, sy, t, p.thetaP);
    near(Tp.sx1, p.s1, tol, 'at θ_p σ_x′ = σ₁'); near(Tp.sy1, p.s2, tol, 'at θ_p σ_y′ = σ₂'); near(Tp.txy1, 0, tol, 'at θ_p τ = 0');
    const Ts = m.transform(sx, sy, t, p.thetaS);
    near(Ts.sx1, p.avg, tol, 'at θ_s σ_x′ = σ_avg'); near(Ts.sy1, p.avg, tol, 'at θ_s σ_y′ = σ_avg'); near(Ts.txy1, p.R, tol, 'at θ_s τ = +R');
    // tensor rotation σ′ = Q σ Qᵀ with Q = [[cos θ, sin θ], [−sin θ, cos θ]]
    const c = Math.cos(th), s = Math.sin(th);
    const Q = [[c, s], [-s, c]], S = [[sx, t], [t, sy]];
    const QS = [[Q[0][0] * S[0][0] + Q[0][1] * S[1][0], Q[0][0] * S[0][1] + Q[0][1] * S[1][1]],
      [Q[1][0] * S[0][0] + Q[1][1] * S[1][0], Q[1][0] * S[0][1] + Q[1][1] * S[1][1]]];
    const R2 = [[QS[0][0] * Q[0][0] + QS[0][1] * Q[0][1], QS[0][0] * Q[1][0] + QS[0][1] * Q[1][1]],
      [QS[1][0] * Q[0][0] + QS[1][1] * Q[0][1], QS[1][0] * Q[1][0] + QS[1][1] * Q[1][1]]];
    near(T.sx1, R2[0][0], tol, 'QσQᵀ[0][0]'); near(T.sy1, R2[1][1], tol, 'QσQᵀ[1][1]');
    near(T.txy1, R2[0][1], tol, 'QσQᵀ[0][1]'); near(T.txy1, R2[1][0], tol, 'QσQᵀ[1][0]');
    // screen frame (σ right, −τ up): X1 at distance R from C, at the angle of CX + 2θ counter-clockwise
    const cp = m.circlePoints(sx, sy, t, th);
    const vx = cp.X1[0] - cp.C[0], vy = -cp.X1[1];
    near(Math.hypot(vx, vy), p.R, tol, '|CX′| = R');
    if (p.R > 1e-6) {
      const a0 = Math.atan2(-cp.X[1], cp.X[0] - cp.C[0]), a1 = Math.atan2(vy, vx);
      const d = Math.abs(RASim.math.wrapPi(a1 - a0 - 2 * th));
      worst = Math.max(worst, d);
      assert.ok(d < 1e-9, 'X′ at angle(CX) + 2θ ccw: off by ' + d);
    }
    // equivalent stresses
    near(p.tresca, 2 * p.tauAbs, 1e-12, 'Tresca = 2τ_abs');
    near(p.vonMises, Math.sqrt(p.s1 * p.s1 - p.s1 * p.s2 + p.s2 * p.s2), 1e-9, 'von Mises');
    near(p.tauAbs, Math.max(Math.abs(p.s1 - p.s2), Math.abs(p.s1), Math.abs(p.s2)) / 2, 1e-12, 'τ_abs');
    assert.ok(p.tauAbs >= p.tauMax - 1e-12, 'τ_abs ≥ τ_max');
  }
  return 'worst angle error ' + worst.toExponential(1) + ' rad';
});

group('screen sense: increasing θ moves X′ counter-clockwise (−τ up)', function () {
  // default state: X below the σ-axis (τ = +40 > 0, plotted at −τ < 0)
  const c0 = m.circlePoints(50, -10, 40, 0);
  assert.ok(-c0.X[1] < 0, 'X drawn below the σ-axis');
  const c1 = m.circlePoints(50, -10, 40, 5 * D2R);
  const a0 = Math.atan2(-c0.X1[1], c0.X1[0] - 20), a1 = Math.atan2(-c1.X1[1], c1.X1[0] - 20);
  near(RASim.math.wrapPi(a1 - a0), 10 * D2R, 1e-12, '+5° on the element = +10° ccw on screen');
});

// ---------------------------------------------------------------------------------------------
// Independent reference (verification): the stresses on the rotated faces from traction vectors
// t = σ·n, the principal stresses and θ_p by brute-force search of σ_x′(θ), the in-plane maximum
// shear by brute-force search of τ_x′y′(θ), τ_abs by a search over 3-D unit normals with principal
// stresses (σ₁, σ₂, 0), and von Mises from the components. No formula is shared with the model.
// ---------------------------------------------------------------------------------------------
function refTraction(sx, sy, txy, th) {
  const n = [Math.cos(th), Math.sin(th)], k = [-Math.sin(th), Math.cos(th)];
  const tn = [sx * n[0] + txy * n[1], txy * n[0] + sy * n[1]], tk = [sx * k[0] + txy * k[1], txy * k[0] + sy * k[1]];
  return { sx1: n[0] * tn[0] + n[1] * tn[1], sy1: k[0] * tk[0] + k[1] * tk[1], txy1: k[0] * tn[0] + k[1] * tn[1] };
}
function refArgmax(f) {       // max of f over (−90°, 90°]: grid of 0.05° then golden section
  let best = -Infinity, bt = 0;
  for (let i = 1; i <= 3600; i++) { const t = -Math.PI / 2 + Math.PI * i / 3600; const v = f(t); if (v > best) { best = v; bt = t; } }
  let a = bt - Math.PI / 3600, b = bt + Math.PI / 3600; const g = (Math.sqrt(5) - 1) / 2;
  for (let i = 0; i < 80; i++) { const c = b - g * (b - a), d = a + g * (b - a); if (f(c) > f(d)) b = d; else a = c; }
  return (a + b) / 2;
}
function refPrincipal(sx, sy, txy) {
  const fx = t => refTraction(sx, sy, txy, t).sx1, ft = t => refTraction(sx, sy, txy, t).txy1;
  const tp = refArgmax(fx), s1 = fx(tp), s2 = fx(tp + Math.PI / 2), tauMax = ft(refArgmax(ft));
  let tauAbs = 0;
  for (let i = 0; i <= 180; i++) for (let j = 0; j <= 90; j++) {
    const ph = 2 * Math.PI * i / 180, ps = Math.PI * j / 90;
    const n = [Math.sin(ps) * Math.cos(ph), Math.sin(ps) * Math.sin(ph), Math.cos(ps)];
    const sn = s1 * n[0] * n[0] + s2 * n[1] * n[1], tt = s1 * s1 * n[0] * n[0] + s2 * s2 * n[1] * n[1];
    tauAbs = Math.max(tauAbs, Math.sqrt(Math.max(0, tt - sn * sn)));
  }
  return { s1: s1, s2: s2, thetaP: tp, tauMax: tauMax, tauAbs: tauAbs, vonMises: Math.sqrt(sx * sx - sx * sy + sy * sy + 3 * txy * txy),
    tresca: Math.max(s1, s2, 0) - Math.min(s1, s2, 0) };
}
group('independent reference: 300 random states + slider extremes', function () {
  const r = rng(99), cases = [];
  for (let i = 0; i < 300; i++) cases.push([Math.round(200 * r() - 100), Math.round(200 * r() - 100), Math.round(200 * r() - 100), 0.5 * Math.round(360 * r() - 180)]);
  [-100, -1, 0, 1, 100].forEach(a => [-100, 0, 100].forEach(b => [-100, -1, 0, 1, 100].forEach(c => cases.push([a, b, c, 90], [a, b, c, -89.5]))));
  cases.forEach(function (cs) {
    const sx = cs[0], sy = cs[1], txy = cs[2], th = cs[3] * D2R, what = cs.join(',');
    const T = m.transform(sx, sy, txy, th), Q = refTraction(sx, sy, txy, th);
    near(T.sx1, Q.sx1, 1e-9, what + ' σ_x′'); near(T.sy1, Q.sy1, 1e-9, what + ' σ_y′'); near(T.txy1, Q.txy1, 1e-9, what + ' τ_x′y′');
    const p = m.principal(sx, sy, txy), q = refPrincipal(sx, sy, txy);
    near(p.s1, q.s1, 1e-7, what + ' σ₁'); near(p.s2, q.s2, 1e-7, what + ' σ₂'); near(p.tauMax, q.tauMax, 1e-7, what + ' τ_max');
    near(p.tauAbs, q.tauAbs, 2e-3 * Math.max(1, q.tauAbs), what + ' τ_abs'); near(p.vonMises, q.vonMises, 1e-9, what + ' von Mises');
    near(p.tresca, q.tresca, 1e-7, what + ' Tresca');
    if (p.R > 1e-6) {     // same plane modulo 180°, and θ_p in (−90°, 90°]
      let d = (p.thetaP - q.thetaP) / D2R; d -= 180 * Math.round(d / 180);
      near(d, 0, 1e-3, what + ' θ_p'); assert.ok(p.thetaP > -Math.PI / 2 && p.thetaP <= Math.PI / 2, what + ' θ_p range');
    }
  });
  return cases.length + ' cases';
});

// ---------------------------------------------------------------------------------------------
// Browser tests (need Playwright; skipped with a message when it is not available or when run
// with --no-browser): the read-outs, the drawing conventions and the display bugs found in verification.
// ---------------------------------------------------------------------------------------------
async function agroup(name, fn) {
  try { await fn(); console.log('ok   ' + name); }
  catch (e) { failures++; console.log('FAIL ' + name + ': ' + (e && e.message || e)); }
}
async function browserTests() {
  let playwrightOk = true;
  try { require.resolve('playwright'); } catch (e) { playwrightOk = false; }
  if (!playwrightOk || process.argv.indexOf('--no-browser') >= 0) { console.log('skip browser regression tests'); return; }
  const path = require('path'), os = require('os');
  const { openSim } = require('./harness.js');
  const s = await openSim('mohr-circle', { width: 1100, outDir: path.join(os.tmpdir(), 'rasim-mohr-circle-test'), file: FILE_ARG });
  const page = s.page, ID = 'mohr-circle';
  const r1 = v => RASim.fmt(Math.round(v * 10) / 10, 7);
  async function frames() { await page.evaluate(() => new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r)))); }
  async function setS(st) { await page.evaluate(st => RASim.instances['mohr-circle'].setState(Object.assign({ show3D: false }, st)), st); await frames(); }
  async function getS() { return page.evaluate(() => RASim.instances['mohr-circle'].getState()); }
  async function stats() {
    return page.evaluate(() => { const o = {}; document.querySelectorAll('[data-stat]').forEach(e => { o[e.getAttribute('data-stat')] = e.querySelector('.sim-stat-value').textContent; });
      o.tag = document.querySelector('[data-tag="state"]').textContent; return o; });
  }
  // texts and label backgrounds drawn on each canvas during the next frames
  await page.evaluate(() => {
    window.__txt = { element: [], mohr: [] }; window.__bg = { element: [], mohr: [] };
    const P = CanvasRenderingContext2D.prototype, ft = P.fillText, rr = P.roundRect;
    P.fillText = function (t) { const k = this.canvas.getAttribute('data-canvas'); if (window.__txt[k]) window.__txt[k].push(String(t)); return ft.apply(this, arguments); };
    if (rr) P.roundRect = function (x, y, w, h) { const k = this.canvas.getAttribute('data-canvas'); if (window.__bg[k]) window.__bg[k].push([x, y, w, h]); return rr.apply(this, arguments); };
  });
  async function drawn(st) {
    await page.evaluate(() => { ['element', 'mohr'].forEach(k => { window.__txt[k] = []; window.__bg[k] = []; }); });
    await setS(st);
    return page.evaluate(() => ({ txt: window.__txt, bg: window.__bg }));
  }
  // pixel position of a model point [σ, τ] (τ positive downward), rebuilt from the same plot window as the simulation
  async function px(pt) {
    return page.evaluate(pt => {
      const st = RASim.instances['mohr-circle'].getState(), m = RASim.sims['mohr-circle'].model, P = m.principal(st.sx, st.sy, st.txy);
      const cv = document.querySelector('[data-canvas="mohr"]'), w = cv.clientWidth, h = cv.clientHeight;
      const W = Math.max(P.R, Math.abs(P.avg), 5) * 1.3, H = Math.max(P.R, st.show3D ? P.tauAbs : 0, 5) * 1.3;
      const p = RASim.plot(document.createElement('canvas').getContext('2d'), { box: { x: 0, y: 0, w: w, h: h }, xmin: P.avg - W, xmax: P.avg + W,
        ymin: -H, ymax: H, equal: true, margin: { left: 36, right: 10, top: 22, bottom: 34 } });
      const x = p.X(pt[0]), y = p.Y(-pt[1]), d = cv.getContext('2d').getImageData(Math.round(x * cv.width / w), Math.round(y * cv.width / w), 1, 1).data;
      const rc = cv.getBoundingClientRect();
      return { x: x, y: y, axisY: p.Y(0), rgb: [d[0], d[1], d[2]], pageX: rc.left + cv.clientLeft + x, pageY: rc.top + cv.clientTop + y,
        cPageX: rc.left + cv.clientLeft + p.X(P.avg), cPageY: rc.top + cv.clientTop + p.Y(0), Rpx: P.R * (p.X(1) - p.X(0)) };
    }, pt);
  }
  const isRed = c => c[0] > 150 && c[1] < 90 && c[2] < 90;
  try {
    await agroup('browser: read-outs equal the independent reference (check-value states)', async function () {
      const states = [[50, -10, 40, 0], [50, -10, 40, 26.56505117707799], [50, -10, 40, -18.43494882292201], [50, -10, 40, 45], [50, -10, 40, -60],
        [0, 0, 50, 45], [80, 0, 0, -45], [80, 40, 0, 0], [60, 60, 0, 10], [-40, -80, 0, 0], [20, 80, -30, -67.5], [-100, 100, -100, 90]];
      for (const c of states) {
        await setS({ sx: c[0], sy: c[1], txy: c[2], theta: c[3] });
        const o = await stats(), T = refTraction(c[0], c[1], c[2], c[3] * D2R), q = refPrincipal(c[0], c[1], c[2]), w = c.join(',');
        assert.strictEqual(o.sx1, RASim.fmtFixed(T.sx1, 1), w + ' σ_x′'); assert.strictEqual(o.sy1, RASim.fmtFixed(T.sy1, 1), w + ' σ_y′');
        assert.strictEqual(o.txy1, RASim.fmtFixed(T.txy1, 1), w + ' τ_x′y′');
        assert.strictEqual(o.s1, RASim.fmtFixed(q.s1, 2), w + ' σ₁'); assert.strictEqual(o.s2, RASim.fmtFixed(q.s2, 2), w + ' σ₂');
        assert.strictEqual(o.avg, RASim.fmtFixed((c[0] + c[1]) / 2, 2), w + ' σ_avg'); assert.strictEqual(o.R, RASim.fmtFixed(q.tauMax, 2), w + ' R');
        assert.strictEqual(o.tauMax, RASim.fmtFixed(q.tauMax, 2), w + ' τ_max'); near(Number(o.tauAbs.replace('−', '-')), q.tauAbs, 0.005 + 2e-3 * q.tauAbs, w + ' τ_abs');   // the 3-D search is on a 1° grid
        assert.strictEqual(o.vonMises, RASim.fmtFixed(q.vonMises, 2), w + ' von Mises'); assert.strictEqual(o.tresca, RASim.fmtFixed(q.tresca, 2), w + ' Tresca');
        const iso = q.tauMax < 1e-6;
        if (!iso) assert.ok(o.thetaP.indexOf(RASim.fmtFixed(q.thetaP / D2R, 2) + '°') === 0, w + ' θ_p ' + o.thetaP);
        const tag = iso ? 'HYDROSTATIC — NO SHEAR ON ANY PLANE' : Math.abs(T.txy1) < 0.05 ? 'PRINCIPAL PLANES' : Math.abs(Math.abs(T.txy1) - q.tauMax) < 0.05 ? 'MAX SHEAR PLANES' : '';
        assert.strictEqual(o.tag, tag, w + ' tag');
      }
    });
    await agroup('browser: X drawn below the σ-axis at θ = 0; increasing θ moves X′ counter-clockwise on screen', async function () {
      await setS({ sx: 50, sy: -10, txy: 40, theta: 0 });
      const X = await px([50, 40]);
      assert.ok(X.y > X.axisY + 20 && isRed(X.rgb), 'X′ = X (50, 40) drawn red below the axis: ' + JSON.stringify(X));
      let prev = null;
      for (const t of [-80, -40, 0, 20, 50, 85]) {
        await setS({ sx: 50, sy: -10, txy: 40, theta: t });
        const T = m.transform(50, -10, 40, t * D2R), q = await px([T.sx1, T.txy1]);
        assert.ok(isRed(q.rgb), 'X′ drawn at θ = ' + t + ': ' + q.rgb);
        const a = Math.atan2(q.cPageY - q.pageY, q.pageX - q.cPageX);
        if (prev != null) assert.ok(RASim.math.wrapPi(a - prev) > 0, 'counter-clockwise step to θ = ' + t);
        prev = a;
      }
    });
    await agroup('browser: buttons set θ exactly; presets; Reset restores every default', async function () {
      await setS({ sx: 20, sy: 80, txy: -30, theta: 0 });
      await page.click('[data-control="principal"]'); near((await getS()).theta, -67.5, 1e-9, 'θ_p button');
      await page.click('[data-control="maxShear"]'); near((await getS()).theta, 67.5, 1e-9, 'θ_s button (−112.5° ≡ 67.5°)');
      assert.strictEqual((await stats()).txy1, '42.4', 'τ = +R on the max-shear plane');
      await page.selectOption('[data-control="preset"]', 'thinCylinder');
      let st = await getS(); assert.deepStrictEqual([st.sx, st.sy, st.txy], [80, 40, 0], 'thin cylinder preset');
      await page.check('[data-control="show3D"]');
      const o = await stats(); assert.strictEqual(o.tauMax, '20.00'); assert.strictEqual(o.tauAbs, '40.00');
      await page.evaluate(() => RASim.instances['mohr-circle'].setState({ sx: -77, sy: 33, txy: -12, theta: 61.5, show3D: true }));
      await page.click('[data-control="reset"]');
      st = await getS(); assert.deepStrictEqual(st, { sx: 50, sy: -10, txy: 40, theta: 0, show3D: false }, 'Reset');
    });
    await agroup('browser: a circle too small to draw keeps σ₁ ≠ σ₂ and X′ ≠ Y′ apart in its labels (regression)', async function () {
      const d = await drawn({ sx: 100, sy: 96, txy: 0, theta: 20 });       // R = 2 MPa: under 3 px on the canvas
      const t = d.txt.mohr.join(' | ');
      assert.ok(t.indexOf('σ₁ = σ₂') < 0 && t.indexOf('X′ = Y′') < 0, 'no false equality: ' + t);
      assert.ok(t.indexOf('σ₁ = 100, σ₂ = 96') >= 0 && t.indexOf('X′ (99.5, −1.3)') >= 0 && t.indexOf('Y′ (96.5, 1.3)') >= 0, 'both values drawn: ' + t);
      const e = await drawn({ sx: 60, sy: 60, txy: 0, theta: 20 });         // hydrostatic: the equality is true
      assert.ok(e.txt.mohr.join(' | ').indexOf('σ₁ = σ₂ = 60') >= 0, 'hydrostatic label: ' + e.txt.mohr.join(' | '));
    });
    await agroup('browser: σ₁, σ₂, σ_avg, 2θ and X′ values are always on the canvas, and labels never overlap (regression)', async function () {
      const states = [{ sx: 38, sy: 55, txy: -56, theta: 22, show3D: true }, { sx: -40, sy: 79, txy: -42, theta: 46 }, { sx: -75, sy: -6, txy: 98, theta: -46, show3D: true },
        { sx: -23, sy: -57, txy: 74, theta: 40 }, { sx: 23, sy: 37, txy: 54, theta: 3.5, show3D: true }, { sx: 36, sy: -1, txy: -38, theta: -70.5, show3D: true },
        { sx: 50, sy: -10, txy: 40, theta: 30 }, { sx: 54, sy: 68, txy: -12, theta: -40.5, show3D: true }];
      for (const st of states) {
        const d = await drawn(st), t = d.txt.mohr, j = t.join(' | '), w = JSON.stringify(st);
        const P = m.principal(st.sx, st.sy, st.txy), T = m.transform(st.sx, st.sy, st.txy, st.theta * D2R);
        ['σ₁ = ' + r1(P.s1), 'σ₂ = ' + r1(P.s2), '2θ = ' + r1(2 * st.theta) + '°', 'X′ (' + RASim.fmtFixed(T.sx1, 1) + ', ' + RASim.fmtFixed(T.txy1, 1) + ')'].forEach(function (need) {
          assert.ok(j.indexOf(need) >= 0, w + ': "' + need + '" drawn: ' + j);
        });
        assert.ok(t.some((x, i) => x === 'avg' && t[i + 1] === ' = ' + r1(P.avg)), w + ': σ_avg value drawn: ' + j);
        const b = d.bg.mohr;               // label backgrounds: no two overlap by more than 3 px
        for (let i = 0; i < b.length; i++) for (let k = i + 1; k < b.length; k++) {
          const ox = Math.min(b[i][0] + b[i][2], b[k][0] + b[k][2]) - Math.max(b[i][0], b[k][0]);
          const oy = Math.min(b[i][1] + b[i][3], b[k][1] + b[k][3]) - Math.max(b[i][1], b[k][1]);
          assert.ok(!(ox > 3 && oy > 3), w + ': label backgrounds ' + i + ' and ' + k + ' overlap (' + ox.toFixed(1) + ' × ' + oy.toFixed(1) + ' px)');
        }
      }
    });
    await agroup('browser: dragging X′ follows the pointer without a jump when grabbed off centre (regression)', async function () {
      await setS({ sx: 50, sy: -10, txy: 40, theta: 0 });
      await page.evaluate(() => document.querySelector('[data-canvas="mohr"]').scrollIntoView({ behavior: 'instant', block: 'center' }));
      let q = await px([50, 40]);
      await page.mouse.move(q.pageX + 10, q.pageY - 10); await page.mouse.down();
      await page.mouse.move(q.pageX + 10, q.pageY - 11);
      const t1 = (await getS()).theta;
      assert.ok(Math.abs(t1) < 1, 'θ after a 1 px move from an off-centre grab: ' + t1);
      // then 60° counter-clockwise on screen around C: θ = +30°
      const a0 = Math.atan2(q.cPageY - (q.pageY - 10), q.pageX + 10 - q.cPageX), rr = Math.hypot(q.pageX + 10 - q.cPageX, q.pageY - 10 - q.cPageY);
      for (let i = 1; i <= 12; i++) { const a = a0 + i * (Math.PI / 3) / 12; await page.mouse.move(q.cPageX + rr * Math.cos(a), q.cPageY - rr * Math.sin(a)); }
      await page.mouse.up();
      near((await getS()).theta, 30, 0.5, 'θ after a 60° counter-clockwise drag');
    });
    assert.deepStrictEqual(s.errors, [], 'console errors');
    // (the getImageData read-backs of this test itself make Chrome suggest willReadFrequently)
    assert.deepStrictEqual(s.warnings.filter(x => x.indexOf('willReadFrequently') < 0), [], 'console warnings');
  } catch (e) { failures++; console.log('FAIL browser: ' + (e && e.message || e)); }
  finally { await s.browser.close(); }
}

browserTests().then(function () {
  if (failures) { console.log(failures + ' group(s) failed'); process.exit(1); }
  console.log('all mohr-circle tests passed');
});
