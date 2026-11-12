// Model tests for eigen-2x2. Run: node notes/sims/test/eigen-2x2.test.js
'use strict';
const assert = require('assert');
const RASim = require('./load.js')('eigen-2x2');
const m = RASim.sims['eigen-2x2'].model;
const PI = Math.PI;

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
function vecPrinted(v, e, dec, what) {
  assert.ok(Array.isArray(v) && v.length === 2, what + ' is a 2-vector: ' + JSON.stringify(v));
  printed(v[0], e[0], dec, what + '[0]'); printed(v[1], e[1], dec, what + '[1]');
}
function realValues(r, l1, l2, dec, what) {
  printed(r.values[0].re, l1, dec, what + ' λ₁'); printed(r.values[1].re, l2, dec, what + ' λ₂');
  assert.strictEqual(r.values[0].im, 0, what + ' λ₁ real'); assert.strictEqual(r.values[1].im, 0, what + ' λ₂ real');
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

group('API shape and title', function () {
  assert.strictEqual(RASim.sims['eigen-2x2'].title, 'Eigenvalues and eigenvectors of a 2×2 matrix');
  ['eig', 'apply', 'morph', 'props', 'angleBetween', 'charEq'].forEach(k => assert.strictEqual(typeof m[k], 'function', 'model.' + k));
  const r = m.eig(2, 1, 1, 2);
  ['trace', 'det', 'disc', 'type', 'values', 'vectors'].forEach(k => assert.ok(k in r, 'eig.' + k));
  assert.strictEqual(r.values.length, 2); assert.strictEqual(r.vectors.length, 2);
  ['re', 'im'].forEach(k => assert.ok(k in r.values[0], 'values[0].' + k));
  const p = m.props(2, 1, 1, 2);
  ['symmetric', 'singular', 'orthogonal', 'diagonalizable', 'rank', 'areaScale', 'flips'].forEach(k => assert.ok(k in p, 'props.' + k));
  assert.deepStrictEqual(m.apply(1, 2, 3, 4, 5, 6), [17, 39]);
  assert.deepStrictEqual(m.morph(2, 3, 4, 5, 0.5), [1.5, 1.5, 2, 3]);
});

group('check: eig(2, 1, 1, 2) real-distinct, λ = 3, 1, v₁ = (0.70711, 0.70711), v₂ = (0.70711, −0.70711), trace 4, det 3', function () {
  const r = m.eig(2, 1, 1, 2);
  assert.strictEqual(r.type, 'real-distinct');
  realValues(r, 3, 1, 0, 'eig(2,1,1,2)');
  vecPrinted(r.vectors[0], [0.70711, 0.70711], 5, 'v₁');
  vecPrinted(r.vectors[1], [0.70711, -0.70711], 5, 'v₂');
  printed(r.trace, 4, 0, 'trace'); printed(r.det, 3, 0, 'det');
  printed(r.disc, 4, 0, 'disc = tr² − 4det');
});

group('check: eig(1, 1, 0, 1) repeated-defective, λ = 1, v = (1, 0)', function () {
  const r = m.eig(1, 1, 0, 1);
  assert.strictEqual(r.type, 'repeated-defective');
  realValues(r, 1, 1, 0, 'eig(1,1,0,1)');
  vecPrinted(r.vectors[0], [1, 0], 0, 'v');
  assert.strictEqual(r.vectors[1], null, 'second vector null');
  const p = m.props(1, 1, 0, 1);
  assert.strictEqual(p.diagonalizable, false, 'not diagonalizable');
});

group('check: eig(0, −1, 1, 0) complex, λ = ±j (+imaginary first)', function () {
  const r = m.eig(0, -1, 1, 0);
  assert.strictEqual(r.type, 'complex');
  printed(r.values[0].re, 0, 0, 'Re λ₁'); printed(r.values[0].im, 1, 0, 'Im λ₁');
  printed(r.values[1].re, 0, 0, 'Re λ₂'); printed(r.values[1].im, -1, 0, 'Im λ₂');
  assert.deepStrictEqual(r.vectors, [null, null]);
  const p = m.props(0, -1, 1, 0);
  assert.strictEqual(p.orthogonal, true, 'rotation is orthogonal');
  assert.strictEqual(p.flips, false); assert.strictEqual(p.symmetric, false);
});

group('check: eig(1, 2, 2, 4) λ = 5, 0, v₁ = (0.44721, 0.89443), v₂ = (0.89443, −0.44721), det 0, rank 1', function () {
  const r = m.eig(1, 2, 2, 4);
  assert.strictEqual(r.type, 'real-distinct');
  realValues(r, 5, 0, 0, 'eig(1,2,2,4)');
  vecPrinted(r.vectors[0], [0.44721, 0.89443], 5, 'v₁');
  vecPrinted(r.vectors[1], [0.89443, -0.44721], 5, 'v₂');
  printed(r.det, 0, 0, 'det');
  const p = m.props(1, 2, 2, 4);
  assert.strictEqual(p.rank, 1); assert.strictEqual(p.singular, true); assert.strictEqual(p.symmetric, true);
  assert.strictEqual(p.flips, false); printed(p.areaScale, 0, 0, 'areaScale');
});

group('check: eig(4, 1, 2, 3) λ = 5, 2, v₁ = (0.70711, 0.70711), v₂ = (0.44721, −0.89443)', function () {
  const r = m.eig(4, 1, 2, 3);
  assert.strictEqual(r.type, 'real-distinct');
  realValues(r, 5, 2, 0, 'eig(4,1,2,3)');
  vecPrinted(r.vectors[0], [0.70711, 0.70711], 5, 'v₁');
  vecPrinted(r.vectors[1], [0.44721, -0.89443], 5, 'v₂');
});

group('check: eig(2, 0, 0, 2) repeated-scalar; eig(0, 0, 0, 0) repeated-scalar, λ = 0, rank 0', function () {
  let r = m.eig(2, 0, 0, 2);
  assert.strictEqual(r.type, 'repeated-scalar');
  realValues(r, 2, 2, 0, 'eig(2,0,0,2)');
  assert.deepStrictEqual(r.vectors, [[1, 0], [0, 1]]);
  r = m.eig(0, 0, 0, 0);
  assert.strictEqual(r.type, 'repeated-scalar');
  realValues(r, 0, 0, 0, 'eig(0,0,0,0)');
  assert.deepStrictEqual(r.vectors, [[1, 0], [0, 1]]);
  assert.strictEqual(m.props(0, 0, 0, 0).rank, 0);
  assert.strictEqual(m.props(2, 0, 0, 2).rank, 2);
  assert.strictEqual(m.props(2, 0, 0, 2).diagonalizable, true);
});

group('check: diagonal and triangular eigenvector ordering', function () {
  let r = m.eig(3, 0, 0, 1);
  realValues(r, 3, 1, 0, 'eig(3,0,0,1)');
  vecPrinted(r.vectors[0], [1, 0], 0, 'v₁'); vecPrinted(r.vectors[1], [0, 1], 0, 'v₂');
  r = m.eig(1, 0, 0, 3);
  realValues(r, 3, 1, 0, 'eig(1,0,0,3)');
  vecPrinted(r.vectors[0], [0, 1], 0, 'v₁'); vecPrinted(r.vectors[1], [1, 0], 0, 'v₂');
  r = m.eig(1, 0, 2, 3);
  realValues(r, 3, 1, 0, 'eig(1,0,2,3)');
  vecPrinted(r.vectors[0], [0, 1], 0, 'v₁'); vecPrinted(r.vectors[1], [0.70711, -0.70711], 5, 'v₂');
});

group('check: charEq strings', function () {
  assert.strictEqual(m.charEq(2, 1, 1, 2), 'λ² − 4λ + 3 = 0');
  assert.strictEqual(m.charEq(0, -1, 1, 0), 'λ² + 1 = 0');
  assert.strictEqual(m.charEq(1, 2, 2, 4), 'λ² − 5λ = 0');
  assert.strictEqual(m.charEq(-1, 0, 0, -2), 'λ² + 3λ + 2 = 0');
  assert.strictEqual(m.charEq(1, 1, 0, 1), 'λ² − 2λ + 1 = 0');
  assert.strictEqual(m.charEq(0.5, 0, 0, 0.5), 'λ² − λ + 0.25 = 0');
  assert.strictEqual(m.charEq(0, 0, 0, 0), 'λ² = 0');
  assert.strictEqual(m.charEq(0, 1, 1, 0), 'λ² − 1 = 0');
  assert.ok(!/-/.test(m.charEq(-2.3, 1.7, -0.4, 1.1)), 'no ASCII hyphen-minus');
});

group('props: presets (symmetric, orthogonal, flips, rank)', function () {
  let p = m.props(0, 1, 1, 0);
  assert.strictEqual(p.symmetric, true); assert.strictEqual(p.orthogonal, true); assert.strictEqual(p.flips, true);
  assert.strictEqual(p.rank, 2); printed(p.areaScale, 1, 0, 'reflection area');
  p = m.props(1, 0, 0, 0);
  assert.strictEqual(p.singular, true); assert.strictEqual(p.rank, 1); assert.strictEqual(p.flips, false);
  p = m.props(2, 0, 0, 0.5);
  assert.strictEqual(p.orthogonal, false); printed(p.areaScale, 1, 0, 'scaling area');
  p = m.props(Math.cos(0.3), -Math.sin(0.3), Math.sin(0.3), Math.cos(0.3));
  assert.strictEqual(p.orthogonal, true); assert.strictEqual(p.symmetric, false);
  p = m.props(0.1, 0.2, 0.3, 0.6);   // det is round-off on the 0.1 grid
  assert.strictEqual(p.singular, true); assert.strictEqual(p.flips, false); assert.strictEqual(p.rank, 1);
});

group('angleBetween in [0, π]', function () {
  near(m.angleBetween(1, 0, 0, 1), PI / 2, 1e-12, '90°');
  near(m.angleBetween(1, 0, -1, 0), PI, 1e-12, '180°');
  near(m.angleBetween(1, 1, 2, 2), 0, 1e-12, '0°');
  near(m.angleBetween(1, 0, 1, -1), PI / 4, 1e-12, '45° (clockwise is still positive)');
  assert.ok(Number.isNaN(m.angleBetween(0, 0, 1, 0)), 'zero vector → NaN');
  const r = rng(7);
  for (let i = 0; i < 500; i++) {
    const a = m.angleBetween(r() * 8 - 4, r() * 8 - 4, r() * 8 - 4, r() * 8 - 4);
    assert.ok(a >= 0 && a <= PI, 'range');
  }
});

function invariants(A, what) {
  const a = A[0], b = A[1], c = A[2], d = A[3];
  const r = m.eig(a, b, c, d);
  const tr = a + d, det = a * d - b * c;
  near(r.trace, tr, 1e-12, what + ' trace');
  near(r.det, det, 1e-12, what + ' det');
  near(r.disc, tr * tr - 4 * det, 1e-9, what + ' disc');
  const l1 = r.values[0], l2 = r.values[1];
  near(l1.re + l2.re, tr, 1e-9, what + ' λ₁ + λ₂ = trace');
  if (r.type === 'complex') {
    near(l1.re * l1.re + l1.im * l1.im, det, 1e-9, what + ' |λ|² = det');
    assert.ok(l1.im > 0 && l2.im < 0 && l1.im === -l2.im && l1.re === l2.re, what + ' conjugate pair, +im first');
    assert.deepStrictEqual(r.vectors, [null, null]);
  } else {
    near(l1.re * l2.re, det, 1e-9, what + ' λ₁·λ₂ = det');
    assert.ok(l1.im === 0 && l2.im === 0, what + ' real');
    if (r.type === 'real-distinct') assert.ok(l1.re > l2.re, what + ' λ₁ > λ₂');
    else assert.strictEqual(l1.re, l2.re, what + ' repeated equal');
    if (r.type === 'repeated-defective') assert.ok(r.vectors[0] && r.vectors[1] === null, what + ' defective vectors');
    if (r.type === 'repeated-scalar') assert.deepStrictEqual(r.vectors, [[1, 0], [0, 1]]);
    r.vectors.forEach(function (v, i) {
      if (!v) return;
      const L = r.values[i].re;
      near(Math.hypot(v[0], v[1]), 1, 1e-12, what + ' |v' + (i + 1) + '| = 1');
      const Av = m.apply(a, b, c, d, v[0], v[1]);
      near(Math.hypot(Av[0] - L * v[0], Av[1] - L * v[1]), 0, 1e-9, what + ' A·v' + (i + 1) + ' = λ·v' + (i + 1));
      const first = Math.abs(v[0]) > 1e-12 ? v[0] : v[1];
      assert.ok(first > 0, what + ' first non-zero component positive');
    });
    if (r.type === 'real-distinct' && Math.abs(b - c) <= 1e-12) {
      near(r.vectors[0][0] * r.vectors[1][0] + r.vectors[0][1] * r.vectors[1][1], 0, 1e-9, what + ' symmetric ⇒ v₁·v₂ = 0');
    }
  }
  const M0 = m.morph(a, b, c, d, 0), M1 = m.morph(a, b, c, d, 1);
  [1, 0, 0, 1].forEach((v, i) => assert.ok(M0[i] === v && !Object.is(M0[i], -0), what + ' morph(0) = I: ' + JSON.stringify(M0)));
  [a, b, c, d].forEach((v, i) => assert.ok(M1[i] === v, what + ' morph(1) = A: ' + JSON.stringify(M1)));
  const p = m.props(a, b, c, d);
  near(p.areaScale, Math.abs(det), 1e-12, what + ' areaScale');
  assert.strictEqual(p.diagonalizable, r.type !== 'repeated-defective');
  return r.type;
}

group('invariants: 1000 random matrices on the 0.1 grid in [−3, 3]', function () {
  const r = rng(12345), count = {};
  for (let i = 0; i < 1000; i++) {
    const A = [0, 1, 2, 3].map(() => Math.round(r() * 60 - 30) / 10);
    const t = invariants(A, 'grid ' + JSON.stringify(A));
    count[t] = (count[t] || 0) + 1;
  }
  return JSON.stringify(count);
});

group('invariants: 1000 random continuous matrices in [−3, 3]', function () {
  const r = rng(987), count = {};
  for (let i = 0; i < 1000; i++) {
    const A = [0, 1, 2, 3].map(() => r() * 6 - 3);
    const t = invariants(A, 'cont ' + JSON.stringify(A));
    count[t] = (count[t] || 0) + 1;
  }
  return JSON.stringify(count);
});

group('invariants: 1000 random symmetric grid matrices (b = c) give perpendicular eigenvectors', function () {
  const r = rng(4242);
  let distinct = 0;
  for (let i = 0; i < 1000; i++) {
    const a = Math.round(r() * 60 - 30) / 10, b = Math.round(r() * 60 - 30) / 10, d = Math.round(r() * 60 - 30) / 10;
    const t = invariants([a, b, b, d], 'sym');
    assert.notStrictEqual(t, 'complex', 'symmetric ⇒ real λ');
    assert.notStrictEqual(t, 'repeated-defective', 'symmetric ⇒ diagonalizable');
    if (t === 'real-distinct') distinct++;
  }
  return distinct + ' distinct';
});

group('invariants: repeated / defective constructions on the grid', function () {
  // [[λ + p, q], [r, λ − p]] with p² + qr = 0 → disc = 0
  const cases = [[1, 1, 0, 1], [2, 1, -1, 0], [0, 1, 0, 0], [0.3, 0.1, -0.1, 0.1], [1.5, -0.3, 0.3, 0.9], [-2, 0, 1.3, -2]];
  cases.forEach(function (A) { assert.strictEqual(invariants(A, JSON.stringify(A)), 'repeated-defective', JSON.stringify(A)); });
  const morphMid = m.morph(2, 1, 1, 2, 0.5);
  near(morphMid[0], 1.5, 1e-15, 'morph mid a'); near(morphMid[1], 0.5, 1e-15, 'morph mid b');
});

group('robustness: tiny non-zero b (classified repeated within eps) still returns a true eigenvector', function () {
  // [[1, 1e-10], [3, 1]]: disc = 1.2e-9 ≤ eps = 4e-9 → repeated-defective, λ = 1. (b, λ − a) = (1e-10, 0) would give
  // (1, 0), which is not an eigenvector (A·(1, 0) = (1, 3)); the larger candidate (λ − d, c) = (0, 3) gives (0, 1).
  const r = m.eig(1, 1e-10, 3, 1);
  assert.strictEqual(r.type, 'repeated-defective');
  const v = r.vectors[0], Av = m.apply(1, 1e-10, 3, 1, v[0], v[1]);
  near(Math.hypot(Av[0] - v[0], Av[1] - v[1]), 0, 1e-9, 'A·v = λ·v');
});

group('check: λ₁ + λ₂ = trace and λ₁·λ₂ = det for every check-value matrix', function () {
  [[2, 1, 1, 2], [1, 1, 0, 1], [0, -1, 1, 0], [1, 2, 2, 4], [4, 1, 2, 3], [2, 0, 0, 2], [0, 0, 0, 0], [3, 0, 0, 1], [1, 0, 0, 3], [1, 0, 2, 3]]
    .forEach(function (A) { invariants(A, JSON.stringify(A)); });
  const r = m.eig(0, -1, 1, 0);
  near(r.values[0].re * r.values[0].re + r.values[0].im * r.values[0].im, 1, 1e-12, 'rotation |λ|² = det = 1');
  assert.strictEqual(m.props(0, -1, 1, 0).diagonalizable, true, 'complex distinct λ: diagonalizable over ℂ');
  assert.strictEqual(m.props(4, 1, 2, 3).rank, 2);
});

// ---------------------------------------------------------------------------------------------
// Browser regression tests (need Playwright; skipped with a message when it is not available or
// when run with --no-browser). They guard display bugs found in verification.
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
  const s = await openSim('eigen-2x2', { width: 1100, outDir: path.join(os.tmpdir(), 'rasim-eigen-2x2-test') });
  const page = s.page;
  const DEF = { a: 2, b: 1, c: 1, d: 2, t: 1, vx: 1, vy: 0.5, showGrid: true, showCircle: true, showEigen: true };
  async function frames() { await page.evaluate(() => new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r)))); }
  async function setS(st) { await page.evaluate(st => RASim.instances['eigen-2x2'].setState(st), st); await frames(); }
  const getS = () => page.evaluate(() => RASim.instances['eigen-2x2'].getState());
  const stats = () => page.evaluate(() => { const o = {}; document.querySelectorAll('[data-stat]').forEach(e => { o[e.getAttribute('data-stat')] = e.querySelector('.sim-stat-value').textContent; }); return o; });
  // boxes of every canvas label (tick labels are 11px and excluded) drawn during the next draw
  async function labelBoxes(st) {
    await page.evaluate(() => {
      if (!window.__lbl) {
        const orig = CanvasRenderingContext2D.prototype.fillText;
        CanvasRenderingContext2D.prototype.fillText = function (t, x, y) {
          if (this.canvas.dataset.canvas === 'plane' && this.font.indexOf('11px') < 0) { const w = this.measureText(t).width; window.__lbl.push({ t: String(t), l: x - w / 2, r: x + w / 2, tp: y - 7, b: y + 7 }); }
          return orig.apply(this, arguments);
        };
      }
      window.__lbl = [];
    });
    await setS(Object.assign({}, DEF, st));
    return page.evaluate(() => { const c = document.querySelector('[data-canvas="plane"]'); return { boxes: window.__lbl.splice(0), w: c.clientWidth, h: c.clientHeight }; });
  }
  function noOverlap(L, what) {
    assert.ok(L.boxes.length >= 3, what + ': labels drawn: ' + L.boxes.map(b => b.t).join(' | '));
    L.boxes.forEach(function (A, i) {
      assert.ok(A.l >= 24 && A.r <= L.w - 8 && A.tp >= 8 && A.b <= L.h - 20, what + ': label clipped: ' + A.t);
      L.boxes.slice(i + 1).forEach(function (B) {
        assert.ok(!(A.l < B.r && A.r > B.l && A.tp < B.b && A.b > B.tp), what + ': "' + A.t + '" overlaps "' + B.t + '"');
      });
    });
  }
  try {
    await agroup('browser: read-outs of the check-value states', async function () {
      const exp = [
        [[2, 1, 1, 2], 'REAL, DISTINCT', '3', '1', '(0.707, 0.707)', '(0.707, −0.707)', '4', '3', 'λ² − 4λ + 3 = 0'],
        [[1, 1, 0, 1], 'REPEATED — DEFECTIVE (1 EIGENVECTOR)', '1', '1', '(1.000, 0.000)', '—', '2', '1', 'λ² − 2λ + 1 = 0'],
        [[0, -1, 1, 0], 'COMPLEX — NO REAL EIGENVECTOR', 'j1', '−j1', '—', '—', '0', '1', 'λ² + 1 = 0'],
        [[1, 2, 2, 4], 'REAL, DISTINCT', '5', '0', '(0.447, 0.894)', '(0.894, −0.447)', '5', '0', 'λ² − 5λ = 0'],
        [[4, 1, 2, 3], 'REAL, DISTINCT', '5', '2', '(0.707, 0.707)', '(0.447, −0.894)', '7', '10', 'λ² − 7λ + 10 = 0'],
        [[2, 0, 0, 2], 'REPEATED — EVERY VECTOR', '2', '2', 'any direction', 'any direction', '4', '4', 'λ² − 4λ + 4 = 0'],
        [[0, 0, 0, 0], 'REPEATED — EVERY VECTOR', '0', '0', 'any direction', 'any direction', '0', '0', 'λ² = 0'],
        [[3, 0, 0, 1], 'REAL, DISTINCT', '3', '1', '(1.000, 0.000)', '(0.000, 1.000)', '4', '3', 'λ² − 4λ + 3 = 0'],
        [[1, 0, 0, 3], 'REAL, DISTINCT', '3', '1', '(0.000, 1.000)', '(1.000, 0.000)', '4', '3', 'λ² − 4λ + 3 = 0'],
        [[1, 0, 2, 3], 'REAL, DISTINCT', '3', '1', '(0.000, 1.000)', '(0.707, −0.707)', '4', '3', 'λ² − 4λ + 3 = 0'],
        [[-1, 0, 0, -2], 'REAL, DISTINCT', '−1', '−2', '(1.000, 0.000)', '(0.000, 1.000)', '−3', '2', 'λ² + 3λ + 2 = 0']
      ];
      for (const e of exp) {
        await setS({ a: e[0][0], b: e[0][1], c: e[0][2], d: e[0][3] });
        const r = await stats();
        const tag = await page.textContent('[data-tag="type"]');
        assert.strictEqual(tag.toUpperCase(), e[1], JSON.stringify(e[0]) + ' tag');
        assert.deepStrictEqual([r.lambda1, r.lambda2, r.v1, r.v2, r.trace, r.det, r.charEq], e.slice(2), JSON.stringify(e[0]));
        assert.strictEqual(r.sum, e[6], JSON.stringify(e[0]) + ' λ₁ + λ₂ = trace');
        assert.strictEqual(r.prod, e[7], JSON.stringify(e[0]) + ' λ₁·λ₂ = det (|λ|² for complex)');
      }
      await setS(DEF);
      const r = await stats();
      assert.strictEqual(r.angle, '12.1°'); assert.strictEqual(r.stretch, '2.864');   // x = (1, 0.5), Ax = (2.5, 2)
    });
    await agroup('browser: canvas labels never overlap or leave the plot (regression: singular A at 360 px)', async function () {
      for (const w of [390, 360]) {
        await page.setViewportSize({ width: w, height: 900 }); await page.waitForTimeout(300);
        noOverlap(await labelBoxes({ a: 1, b: 2, c: 2, d: 4 }), 'singular at ' + w + ' px');   // x, Ax, A e₂ and the λ₁ line coincide
        noOverlap(await labelBoxes({ a: 2.2, b: 1.9, c: 0.7, d: 2.8, vx: 1.66, vy: 2.56 }), 'Ax off view at ' + w + ' px');
      }
      await page.setViewportSize({ width: 1100, height: 900 }); await page.waitForTimeout(300);
      noOverlap(await labelBoxes({}), 'default at 1100 px');
    });
    await agroup('browser: typing t while Play runs is not overwritten by the animation (regression)', async function () {
      await setS(DEF);
      await page.click('[data-control="play"]');
      await page.waitForTimeout(250);
      await page.fill('[data-control-num="t"]', '0.5');
      await page.waitForTimeout(400);           // the animation would have overwritten the box meanwhile
      await page.press('[data-control-num="t"]', 'Enter');
      await page.waitForTimeout(300);
      assert.strictEqual((await getS()).t, 0.5, 't after typing 0.5 during Play');
    });
    await agroup('browser: λ = 0 with x near (not on) the null line says Ax ≈ 0 (regression)', async function () {
      await setS({ a: 1, b: 2, c: 2, d: 4, vx: 2, vy: -1 });   // exactly on the null line: Ax = 0
      let msg = await page.textContent('[data-key="eigMsg"]');
      assert.ok(/λ = 0 \(Ax = 0/.test(msg), msg);
      assert.strictEqual((await stats()).stretch, '0');
      await setS({ a: 1, b: 2, c: 2, d: 4, vx: 2, vy: -0.97 });   // 0.77° off the null line: Ax = (0.06, 0.12) ≠ 0
      msg = await page.textContent('[data-key="eigMsg"]');
      assert.ok(/λ = 0 \(Ax ≈ 0/.test(msg), msg);
      assert.strictEqual((await stats()).angle, '—');
    });
    await agroup('browser: Reset restores every default', async function () {
      await setS({ a: -3, b: 2.5, c: 4, d: 0.1, t: 0.2, vx: -3, vy: 2, showGrid: false, showCircle: false, showEigen: false });
      await page.click('[data-control="reset"]'); await frames();
      assert.deepStrictEqual(await getS(), DEF);
      assert.strictEqual(await page.getAttribute('[data-control="c"]', 'max'), '3', 'widened slider back to −3…3');
      assert.strictEqual(await page.inputValue('[data-control="preset"]'), 'symmetric');
    });
    assert.ok(s.errors.length === 0 && s.warnings.length === 0, 'console: ' + s.errors.concat(s.warnings).join('; '));
  } finally { await s.browser.close(); }
}

(async function () {
  try { await browserTests(); } catch (e) { failures++; console.log('FAIL browser: ' + (e && e.message || e)); }
  if (failures) { console.log(failures + ' group(s) FAILED'); process.exit(1); }
  console.log('all groups passed');
})();
