// Model tests for rotation-transforms. Run: node notes/sims/test/rotation-transforms.test.js [--no-browser]
'use strict';
const assert = require('assert');
const RASim = require('./load.js')('rotation-transforms');
const m = RASim.sims['rotation-transforms'].model;
const PI = Math.PI, D = PI / 180;

let failures = 0;
function group(name, fn) {
  try { const extra = fn(); console.log('ok   ' + name + (extra ? '  (' + extra + ')' : '')); }
  catch (e) { failures++; console.log('FAIL ' + name + ': ' + (e && e.message || e)); }
}
function near(actual, expected, tol, what) {
  assert.ok(Number.isFinite(actual) && Math.abs(actual - expected) <= tol, what + ': ' + actual + ' vs ' + expected + ' (tol ' + tol + ')');
}
function matNear(A, B, tol, what) {
  assert.strictEqual(A.length, B.length, what + ': rows');
  for (let i = 0; i < A.length; i++) {
    assert.strictEqual(A[i].length, B[i].length, what + ': columns of row ' + i);
    for (let j = 0; j < A[i].length; j++) near(A[i][j], B[i][j], tol, what + '[' + i + '][' + j + ']');
  }
}
function vecNear(a, b, tol, what) { assert.strictEqual(a.length, b.length, what + ': length'); a.forEach((x, i) => near(x, b[i], tol, what + '[' + i + ']')); }
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

// ---- independent reference implementations (written from the notes, not from the model) ----
function refR(axis, a) {
  const c = Math.cos(a), s = Math.sin(a);
  if (axis === 'x') return [[1, 0, 0], [0, c, -s], [0, s, c]];
  if (axis === 'y') return [[c, 0, s], [0, 1, 0], [-s, 0, c]];
  return [[c, -s, 0], [s, c, 0], [0, 0, 1]];
}
function refMul(A, B) {
  return A.map(r => B[0].map((_, j) => r.reduce((s, v, k) => s + v * B[k][j], 0)));
}
const I3 = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
const I4 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]];
function skew(k) { return [[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]]; }
function rodrigues(k, th) {
  const K = skew(k), K2 = refMul(K, K);
  return I3.map((r, i) => r.map((v, j) => v + Math.sin(th) * K[i][j] + (1 - Math.cos(th)) * K2[i][j]));
}
const AX = ['x', 'y', 'z'];
function randomSteps(r) {
  return [0, 1, 2].map(() => ({ axis: AX[Math.floor(3 * r())], angle: (-180 + 360 * r()) * D }));
}
const N = 300;

group('API shape and title', function () {
  ['Rx', 'Ry', 'Rz', 'rot', 'mul', 'transpose', 'det3', 'trace', 'compose', 'partial', 'homogeneous', 'invHomogeneous', 'apply', 'axisAngle', 'formula']
    .forEach(k => assert.strictEqual(typeof m[k], 'function', 'model.' + k));
  assert.strictEqual(RASim.sims['rotation-transforms'].title, '3D rotations and homogeneous transforms');
  const aa = m.axisAngle(m.Rz(0.3));
  assert.ok('angle' in aa && 'axis' in aa, 'axisAngle returns { angle, axis }');
  assert.strictEqual(m.apply(m.homogeneous(I3, [1, 2, 3]), [0, 0, 0]).length, 3, 'apply returns [x, y, z]');
});

group('elementary matrices match the notes; rot(axis) dispatch', function () {
  const r = rng(1);
  for (let i = 0; i < 50; i++) {
    const a = (-360 + 720 * r()) * D;
    matNear(m.Rx(a), refR('x', a), 0, 'Rx'); matNear(m.Ry(a), refR('y', a), 0, 'Ry'); matNear(m.Rz(a), refR('z', a), 0, 'Rz');
    AX.forEach(ax => matNear(m.rot(ax, a), refR(ax, a), 0, 'rot ' + ax));
  }
  // explicit signs at 90°
  matNear(m.Rx(PI / 2), [[1, 0, 0], [0, 0, -1], [0, 1, 0]], 1e-15, 'Rx(90°)');
  matNear(m.Ry(PI / 2), [[0, 0, 1], [0, 1, 0], [-1, 0, 0]], 1e-15, 'Ry(90°)');
  matNear(m.Rz(PI / 2), [[0, -1, 0], [1, 0, 0], [0, 0, 1]], 1e-15, 'Rz(90°)');
  assert.throws(() => m.rot('w', 1));
});

group('mul / transpose / det3 / trace', function () {
  const A = [[1, 2, 3], [4, 5, 6], [7, 8, 10]], B = [[2, 0, 1], [1, 3, 0], [0, 1, 4]];
  matNear(m.mul(A, B), [[4, 9, 13], [13, 21, 28], [22, 34, 47]], 0, 'mul');
  matNear(m.transpose(A), [[1, 4, 7], [2, 5, 8], [3, 6, 10]], 0, 'transpose');
  near(m.det3(A), -3, 1e-12, 'det3'); near(m.trace(A), 16, 0, 'trace');
  matNear(m.mul(I4, m.homogeneous(A, [1, 2, 3])), m.homogeneous(A, [1, 2, 3]), 0, 'mul 4×4');
});

group('check: compose([X 90°, Z 90°], fixed) = Rz(90°)·Rx(90°) = [[0,0,1],[1,0,0],[0,1,0]]', function () {
  const s = [{ axis: 'x', angle: 90 * D }, { axis: 'z', angle: 90 * D }];
  const R = m.compose(s, 'fixed');
  matNear(R, [[0, 0, 1], [1, 0, 0], [0, 1, 0]], 1e-15, 'R');
  matNear(R, refMul(refR('z', PI / 2), refR('x', PI / 2)), 1e-15, 'Rz·Rx');
});

group('check: compose([X 90°, Z 90°], current) = Rx(90°)·Rz(90°) = [[0,−1,0],[0,0,−1],[1,0,0]]', function () {
  const s = [{ axis: 'x', angle: 90 * D }, { axis: 'z', angle: 90 * D }];
  const R = m.compose(s, 'current');
  matNear(R, [[0, -1, 0], [0, 0, -1], [1, 0, 0]], 1e-15, 'R');
  matNear(R, refMul(refR('x', PI / 2), refR('z', PI / 2)), 1e-15, 'Rx·Rz');
});

group('check: axisAngle of those: 120° about (1, 1, 1)/√3 and 120° about (1, −1, 1)/√3', function () {
  const s = [{ axis: 'x', angle: 90 * D }, { axis: 'z', angle: 90 * D }];
  const a = m.axisAngle(m.compose(s, 'fixed')), b = m.axisAngle(m.compose(s, 'current'));
  const q = 1 / Math.sqrt(3);
  near(a.angle, 120 * D, 1e-12, 'fixed θ'); vecNear(a.axis, [q, q, q], 1e-12, 'fixed k');
  near(b.angle, 120 * D, 1e-12, 'current θ'); vecNear(b.axis, [q, -q, q], 1e-12, 'current k');
});

group('check: presets — RPY = Rz(60°)·Ry(45°)·Rx(30°), ZYZ = Rz(30°)·Ry(45°)·Rz(60°)', function () {
  const rpy = m.compose([{ axis: 'x', angle: 30 * D }, { axis: 'y', angle: 45 * D }, { axis: 'z', angle: 60 * D }], 'fixed');
  matNear(rpy, refMul(refMul(refR('z', 60 * D), refR('y', 45 * D)), refR('x', 30 * D)), 1e-15, 'RPY');
  const zyz = m.compose([{ axis: 'z', angle: 30 * D }, { axis: 'y', angle: 45 * D }, { axis: 'z', angle: 60 * D }], 'current');
  matNear(zyz, refMul(refMul(refR('z', 30 * D), refR('y', 45 * D)), refR('z', 60 * D)), 1e-15, 'ZYZ');
  // closed form of the RPY matrix, entry R31 = −sin β and R11 = cos α cos β
  near(rpy[2][0], -Math.sin(45 * D), 1e-15, 'RPY R31'); near(rpy[0][0], Math.cos(60 * D) * Math.cos(45 * D), 1e-15, 'RPY R11');
  // ZYZ: R33 = cos θ
  near(zyz[2][2], Math.cos(45 * D), 1e-15, 'ZYZ R33');
});

group('check: R = Rz(90°), d = (1, 2, 3): T⁻¹ translation (−2, 1, −3); apply(T, [1,0,0]) = (1, 3, 3); T·T⁻¹ = I', function () {
  const T = m.homogeneous(m.Rz(PI / 2), [1, 2, 3]);
  matNear(T, [[0, -1, 0, 1], [1, 0, 0, 2], [0, 0, 1, 3], [0, 0, 0, 1]], 1e-15, 'T');
  const Ti = m.invHomogeneous(T);
  vecNear([Ti[0][3], Ti[1][3], Ti[2][3]], [-2, 1, -3], 1e-15, 'T⁻¹ translation');
  matNear(Ti, [[0, 1, 0, -2], [-1, 0, 0, 1], [0, 0, 1, -3], [0, 0, 0, 1]], 1e-15, 'T⁻¹');
  vecNear(m.apply(T, [1, 0, 0]), [1, 3, 3], 1e-15, 'apply');
  matNear(m.mul(T, Ti), I4, 1e-15, 'T·T⁻¹');
});

group('check: axisAngle(Rx(180°)) = 180° about ±(1,0,0); axisAngle(I) = 0, null; axisAngle(Rz(−90°)) = 90° about (0,0,−1)', function () {
  const a = m.axisAngle(m.Rx(PI));
  near(a.angle, PI, 1e-12, 'Rx(180) θ');
  vecNear(a.axis.map(Math.abs), [1, 0, 0], 1e-12, 'Rx(180) |k|');
  const b = m.axisAngle(I3);
  near(b.angle, 0, 0, 'I θ'); assert.strictEqual(b.axis, null, 'I axis null');
  const c = m.axisAngle(m.Rz(-PI / 2));
  near(c.angle, PI / 2, 1e-12, 'Rz(−90) θ'); vecNear(c.axis, [0, 0, -1], 1e-12, 'Rz(−90) k');
});

group('axisAngle near π: diagonal formula, sign rule, Rodrigues still holds', function () {
  const r = rng(5);
  for (let i = 0; i < 200; i++) {
    const k = (function () { const v = [r() - 0.5, r() - 0.5, r() - 0.5]; const n = Math.hypot(...v); return v.map(x => x / n); })();
    [PI, PI - 1e-7, PI - 5e-7].forEach(function (th) {
      const R = rodrigues(k, th);
      const aa = m.axisAngle(R);
      near(aa.angle, th, 1e-7, 'θ');
      near(Math.hypot(...aa.axis), 1, 1e-12, '|k|');
      const dotk = aa.axis[0] * k[0] + aa.axis[1] * k[1] + aa.axis[2] * k[2];
      near(Math.abs(dotk), 1, 1e-9, 'axis ∥ k');
      // θ comes from acos((trace R − 1)/2) as specified; near π acos amplifies the ~2e-16 round-off of the trace
      // by 1/sin θ (≈ 1e7 here), so θ itself is only good to ~1e-8 rad: the reconstruction is checked to 1e-7.
      matNear(rodrigues(aa.axis, aa.angle), R, 1e-7, 'Rodrigues near π');
      if (th === PI) {   // exactly π: largest component positive
        const j = [0, 1, 2].reduce((b, q) => Math.abs(aa.axis[q]) > Math.abs(aa.axis[b]) ? q : b, 0);
        assert.ok(aa.axis[j] > 0, 'largest component positive');
      }
    });
  }
  // tiny angles: axis undefined
  assert.strictEqual(m.axisAngle(m.Rx(1e-8)).axis, null, 'θ < 1e-6 → null');
  return '600 matrices';
});

group('formula(): non-zero steps in multiplication order; all zero → R = I', function () {
  const s = [{ axis: 'x', angle: 90 * D }, { axis: 'z', angle: 90 * D }, { axis: 'y', angle: 0 }];
  assert.strictEqual(m.formula(s, 'fixed'), 'R = Rz(90°) · Rx(90°)');
  assert.strictEqual(m.formula(s, 'current'), 'R = Rx(90°) · Rz(90°)');
  const t = [{ axis: 'x', angle: 30 * D }, { axis: 'y', angle: -45 * D }, { axis: 'z', angle: 60 * D }];
  assert.strictEqual(m.formula(t, 'fixed'), 'R = Rz(60°) · Ry(−45°) · Rx(30°)');
  assert.strictEqual(m.formula(t, 'current'), 'R = Rx(30°) · Ry(−45°) · Rz(60°)');
  assert.strictEqual(m.formula([{ axis: 'x', angle: 0 }, { axis: 'y', angle: 0 }, { axis: 'z', angle: 0 }], 'fixed'), 'R = I');
  assert.strictEqual(m.formula([{ axis: 'z', angle: 90 * D }, { axis: 'x', angle: 0 }, { axis: 'y', angle: 0 }], 'current'), 'R = Rz(90°)');
});

group('invariants over ' + N + ' random step sets in both modes', function () {
  const r = rng(20270101);
  let maxRod = 0;
  for (let i = 0; i < N; i++) {
    const s = randomSteps(r);
    ['fixed', 'current'].forEach(function (mode) {
      const R = m.compose(s, mode);
      // independent product
      const Ms = s.map(q => refR(q.axis, q.angle));
      const ref = mode === 'fixed' ? refMul(Ms[2], refMul(Ms[1], Ms[0])) : refMul(refMul(Ms[0], Ms[1]), Ms[2]);
      matNear(R, ref, 1e-12, 'compose #' + i + ' ' + mode);
      near(m.det3(R), 1, 1e-12, 'det #' + i);
      matNear(m.mul(m.transpose(R), R), I3, 1e-12, 'RᵀR #' + i);
      const aa = m.axisAngle(R);
      near(Math.cos(aa.angle), (m.trace(R) - 1) / 2, 1e-12, 'trace = 1 + 2cos θ #' + i);
      assert.ok(aa.angle >= 0 && aa.angle <= PI, 'θ in [0, π]');
      if (aa.axis) {
        const Rr = rodrigues(aa.axis, aa.angle);
        matNear(Rr, R, 1e-9, 'Rodrigues #' + i + ' ' + mode);
        for (let a = 0; a < 3; a++) for (let b = 0; b < 3; b++) maxRod = Math.max(maxRod, Math.abs(Rr[a][b] - R[a][b]));
        const Rk = R.map(row => row[0] * aa.axis[0] + row[1] * aa.axis[1] + row[2] * aa.axis[2]);
        vecNear(Rk, aa.axis, 1e-9, 'R·k = k #' + i);
        near(Math.hypot(...aa.axis), 1, 1e-12, '|k| = 1');
      } else matNear(R, I3, 1e-6, 'null axis only for R ≈ I');
      matNear(m.partial(s, mode, 3, 1), R, 1e-12, 'partial(3, 1) = compose #' + i);
      for (let k = 1; k <= 3; k++) matNear(m.partial(s, mode, k, 0), m.partial(s, mode, k - 1, 1), 1e-12, 'partial(k, 0) = partial(k−1, 1) k=' + k);
      // partial is the composition of the truncated list with step k at f·θ_k
      const f = r(), k = 1 + Math.floor(3 * r());
      const tr = s.map((q, j) => j + 1 < k ? q : { axis: q.axis, angle: j + 1 === k ? f * q.angle : 0 });
      matNear(m.partial(s, mode, k, f), m.compose(tr, mode), 1e-12, 'partial = compose(truncated)');
      // homogeneous transform with a random d
      const d = [-2 + 4 * r(), -2 + 4 * r(), -2 + 4 * r()];
      const T = m.homogeneous(R, d), Ti = m.invHomogeneous(T);
      matNear(m.mul(Ti, T), I4, 1e-12, 'T⁻¹·T #' + i);
      matNear(m.mul(T, Ti), I4, 1e-12, 'T·T⁻¹ #' + i);
      const Rt = m.transpose(R);
      vecNear([Ti[0][3], Ti[1][3], Ti[2][3]], Rt.map(row => -(row[0] * d[0] + row[1] * d[1] + row[2] * d[2])), 1e-12, '−Rᵀd');
      const p = [-1.5 + 3 * r(), -1.5 + 3 * r(), -1.5 + 3 * r()];
      const pA = m.apply(T, p);
      vecNear(pA, R.map((row, a) => row[0] * p[0] + row[1] * p[1] + row[2] * p[2] + d[a]), 1e-12, 'p_A = R p_B + d');
      vecNear(m.apply(Ti, pA), p, 1e-12, 'T⁻¹ p_A = p_B');
    });
    // fixed (s1, s2, s3) = current (s3, s2, s1)
    matNear(m.compose(s, 'fixed'), m.compose(s.slice().reverse(), 'current'), 1e-12, 'fixed = reversed current #' + i);
  }
  return N + ' sets × 2 modes, max Rodrigues error ' + maxRod.toExponential(1);
});

// ---------------------------------------------------------------------------------------------
// Browser checks (need Playwright; skipped when it is not available or with --no-browser)
// ---------------------------------------------------------------------------------------------
async function agroup(name, fn) {
  try { await fn(); console.log('ok   ' + name); }
  catch (e) { failures++; console.log('FAIL ' + name + ': ' + (e && e.message || e)); }
}
async function browserTests() {
  let ok = true;
  try { require.resolve('playwright'); } catch (e) { ok = false; }
  if (!ok || process.argv.indexOf('--no-browser') >= 0) { console.log('skip browser checks'); return; }
  const path = require('path'), os = require('os');
  const { openSim } = require('./harness.js');
  const s = await openSim('rotation-transforms', { width: 1100, outDir: path.join(os.tmpdir(), 'rasim-rotation-transforms-test') });
  const page = s.page;
  const num = t => Number(String(t).replace(/−/g, '-'));
  async function frames() { await page.evaluate(() => new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r)))); }
  async function setS(x) { await page.evaluate(x => RASim.instances['rotation-transforms'].setState(x), x); await frames(); }
  async function matrix(key) {
    return page.evaluate(k => Array.from(document.querySelectorAll('[data-matrix="' + k + '"] tr')).map(tr => Array.from(tr.querySelectorAll('td')).map(td => td.textContent)), key);
  }
  try {
    await agroup('browser: default view is the textbook picture and right-handed', async function () {
      await page.evaluate(() => {
        window.__seg = [];
        const P = CanvasRenderingContext2D.prototype;
        const mt = P.moveTo, lt = P.lineTo;
        P.moveTo = function (x, y) { this.__m = [x, y]; return mt.apply(this, arguments); };
        P.lineTo = function (x, y) { if (this.__m) window.__seg.push({ c: String(this.strokeStyle), w: this.lineWidth, a: this.__m, b: [x, y] }); this.__m = [x, y]; return lt.apply(this, arguments); };
        window.__txt = [];
        const ft = P.fillText;
        P.fillText = function (t, x, y) { window.__txt.push({ t: String(t), c: String(this.fillStyle), x: x, y: y }); return ft.apply(this, arguments); };
      });
      await setS({ az: 30, el: 25 });
      const seg = await page.evaluate(() => window.__seg.splice(0));
      const txt = await page.evaluate(() => window.__txt.splice(0));
      const ax = seg.filter(q => q.c === RASim.colors.muted && Math.abs(q.w - 1.5) < 1e-9);
      assert.strictEqual(ax.length, 3, 'three frame-A axis strokes, got ' + ax.length);
      const O = ax[0].a;
      ax.forEach(q => { near(q.a[0], O[0], 1e-6, 'common origin x'); near(q.a[1], O[1], 1e-6, 'common origin y'); });
      // name each stroke by its nearest label X / Y / Z (drawn in the muted colour); screen y down → flip
      const v = ['X', 'Y', 'Z'].map(function (n) {
        const lb = txt.filter(q => q.t === n && q.c === RASim.colors.muted);
        assert.strictEqual(lb.length, 1, 'one label ' + n + '_A');
        const q = ax.reduce((b, a) => Math.hypot(a.b[0] - lb[0].x, a.b[1] - lb[0].y) < Math.hypot(b.b[0] - lb[0].x, b.b[1] - lb[0].y) ? a : b);
        return [q.b[0] - O[0], O[1] - q.b[1]];
      });
      assert.ok(new Set(v.map(String)).size === 3, 'three different axes');
      assert.ok(v[2][1] > 0 && Math.abs(v[2][0]) < 1e-6, 'Z_A points straight up');
      assert.ok(v[0][0] < 0 && v[0][1] < 0, 'X_A towards the lower left');
      assert.ok(v[1][0] > 0 && v[1][1] < 0, 'Y_A towards the lower right');
      // seen from above (Z towards the viewer) X → Y is counter-clockwise: cross(X, Y) > 0 in a y-up screen
      assert.ok(v[0][0] * v[1][1] - v[0][1] * v[1][0] > 0, 'X_A → Y_A counter-clockwise');
    });
    await agroup('browser: displayed R for the check states and presets equals the model', async function () {
      const cases = [
        [{ mode: 'fixed', axis1: 'x', angle1: 90, axis2: 'z', angle2: 90, axis3: 'y', angle3: 0 }, [[0, 0, 1], [1, 0, 0], [0, 1, 0]]],
        [{ mode: 'current' }, [[0, -1, 0], [0, 0, -1], [1, 0, 0]]]
      ];
      for (const c of cases) {
        await setS(c[0]);
        const R = await matrix('R');
        R.forEach((row, i) => row.forEach((t, j) => near(num(t), c[1][i][j], 5e-4, 'R[' + i + '][' + j + ']')));
      }
      for (const p of [['rpy', 'fixed', [['z', 60], ['y', 45], ['x', 30]]], ['zyz', 'current', [['z', 30], ['y', 45], ['z', 60]]]]) {
        await page.selectOption('[data-control="preset"]', p[0]); await frames();
        const st = await page.evaluate(() => RASim.instances['rotation-transforms'].getState());
        assert.strictEqual(st.mode, p[1], 'preset mode');
        const ref = refMul(refMul(refR(p[2][0][0], p[2][0][1] * D), refR(p[2][1][0], p[2][1][1] * D)), refR(p[2][2][0], p[2][2][1] * D));
        const R = await matrix('R');
        R.forEach((row, i) => row.forEach((t, j) => near(num(t), ref[i][j], 5e-4, p[0] + ' R[' + i + '][' + j + ']')));
      }
    });
    await agroup('browser: stats and T⁻¹ read-outs for check states (Rz(90°), d = (1, 2, 2); hint 3 d = (1, 2, 0))', async function () {
      const stat = k => page.textContent('[data-stat="' + k + '"] .sim-stat-value');
      const vec = t => String(t).replace(/[()]/g, '').split(',').map(num);
      await setS({ mode: 'fixed', axis1: 'z', angle1: 90, axis2: 'x', angle2: 0, axis3: 'y', angle3: 0, dx: 1, dy: 2, dz: 2, px: 1, py: 0, pz: 0 });
      let Ti = await matrix('Tinv');
      vecNear([0, 1, 2].map(i => num(Ti[i][3])), [-2, 1, -2], 5e-4, 'T⁻¹ translation');
      vecNear(vec(await stat('pA')), [1, 3, 2], 5e-4, 'p_A');
      near(num((await stat('eqAngle')).replace('°', '')), 90, 0.05, 'θ'); vecNear(vec(await stat('eqAxis')), [0, 0, 1], 5e-4, 'k');
      assert.strictEqual(await page.textContent('[data-key="formula"]'), 'R = Rz(90°)');
      await setS({ axis1: 'x', angle1: 90, axis2: 'z', angle2: 90, dz: 0, px: 1, py: 1 });
      Ti = await matrix('Tinv');
      vecNear([0, 1, 2].map(i => num(Ti[i][3])), [-2, 0, -1], 5e-4, 'hint 3: −Rᵀd');
      near(num(await stat('trace')), 0, 5e-4, 'trace'); near(num(await stat('detR')), 1, 5e-4, 'det');
      near(num((await stat('eqAngle')).replace('°', '')), 120, 0.05, 'θ = 120°');
      await setS({ dx: 0, dy: 0 });
    });
    await agroup('browser: legend describes the ghost as it is drawn (faded coloured dashes, not grey)', async function () {
      // regression: the legend showed a grey swatch "ghost: other order" while the ghost is drawn in faded x/y/z colours
      const t = await page.evaluate(() => Array.from(document.querySelectorAll('.sim-legend-item')).map(e => e.textContent).filter(x => /ghost/.test(x))[0]);
      assert.ok(/faded/.test(t) && /xB/.test(t) && /yB/.test(t) && /zB/.test(t), 'ghost legend: ' + t);
    });
    assert.ok(s.errors.length === 0 && s.warnings.length === 0, 'console: ' + s.errors.concat(s.warnings).join('; '));
  } finally { await s.browser.close(); }
}

(async function () {
  try { await browserTests(); } catch (e) { failures++; console.log('FAIL browser: ' + (e && e.message || e)); }
  if (failures) { console.log(failures + ' group(s) FAILED'); process.exit(1); }
  console.log('all groups passed');
})();
