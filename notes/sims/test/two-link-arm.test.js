// Model tests for two-link-arm. Run: node notes/sims/test/two-link-arm.test.js
'use strict';
const assert = require('assert');
const RASim = require('./load.js')('two-link-arm');
const m = RASim.sims['two-link-arm'].model;
const PI = Math.PI, D = PI / 180;

let failures = 0;
function group(name, fn) {
  try { const extra = fn(); console.log('ok   ' + name + (extra ? '  (' + extra + ')' : '')); }
  catch (e) { failures++; console.log('FAIL ' + name + ': ' + (e && e.message || e)); }
}
// tolerance for a value printed with `dec` decimals in the specification:
// max(5e-4 relative, half a unit of the last printed digit)
function printed(actual, expected, dec, what) {
  const tol = Math.max(5e-4 * Math.abs(expected), 0.5 * Math.pow(10, -dec));
  assert.ok(Math.abs(actual - expected) <= tol, what + ': ' + actual + ' vs ' + expected + ' (tol ' + tol + ')');
}
function near(actual, expected, tol, what) {
  assert.ok(Number.isFinite(actual) && Math.abs(actual - expected) <= tol, what + ': ' + actual + ' vs ' + expected + ' (tol ' + tol + ')');
}
function angNear(a, b, tol, what) { // equal modulo 2π
  const d = Math.abs(RASim.math.wrapPi(a - b));
  assert.ok(d <= tol, what + ': ' + a + ' vs ' + b + ' (mod 2π diff ' + d + ')');
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
const N = 500;
function sample(r) {
  return { L1: 0.5 + 1.5 * r(), L2: 0.5 + 1.5 * r(), t1: -PI + 2 * PI * r(), t2: -PI + 2 * PI * r(), w1: -2 + 4 * r(), w2: -2 + 4 * r() };
}

group('API shape', function () {
  ['fk', 'ik', 'other', 'jacobian', 'detJ', 'tipVelocity', 'workspace', 'singular'].forEach(function (k) {
    assert.strictEqual(typeof m[k], 'function', 'model.' + k);
  });
  assert.strictEqual(RASim.sims['two-link-arm'].title, '2-link planar arm: forward and inverse kinematics');
  const f = m.fk(1, 0.8, 0.3, 0.4);
  ['x', 'y', 'phi', 'ex', 'ey'].forEach(k => assert.ok(typeof f[k] === 'number', 'fk.' + k));
  const s = m.ik(1, 0.8, 1, 1, 1);
  ['t1', 't2', 'reachable', 'x', 'y'].forEach(k => assert.ok(k in s, 'ik.' + k));
});

group('check: fk(1, 1, 0, π/2) -> (1, 1), φ = 90°, det J = 1', function () {
  const f = m.fk(1, 1, 0, PI / 2);
  near(f.x, 1, 1e-12, 'x'); near(f.y, 1, 1e-12, 'y'); near(f.phi, PI / 2, 1e-12, 'phi');
  near(f.ex, 1, 1e-12, 'ex'); near(f.ey, 0, 1e-12, 'ey');
  near(m.detJ(1, 1, PI / 2), 1, 1e-12, 'detJ');
});

group('check: fk(1, 0.8, 30°, 60°) -> x = 0.86603, y = 1.30000, det J = 0.69282', function () {
  const f = m.fk(1, 0.8, 30 * D, 60 * D);
  printed(f.x, 0.86603, 5, 'x'); printed(f.y, 1.30000, 5, 'y');
  printed(m.detJ(1, 0.8, 60 * D), 0.69282, 5, 'detJ');
  near(f.x, Math.sqrt(3) / 2, 1e-12, 'x exact'); near(f.y, 1.3, 1e-12, 'y exact');
  near(m.detJ(1, 0.8, 60 * D), 0.4 * Math.sqrt(3), 1e-12, 'detJ exact');
  near(f.phi, PI / 2, 1e-12, 'phi');
});

group('check: ik(1, 1, 1, 1, ±1)', function () {
  const a = m.ik(1, 1, 1, 1, 1);
  near(a.t1, 0, 1e-12, 'down t1'); near(a.t2, PI / 2, 1e-12, 'down t2'); assert.strictEqual(a.reachable, true);
  near(a.x, 1, 0, 'down x'); near(a.y, 1, 0, 'down y');
  const b = m.ik(1, 1, 1, 1, -1);
  near(b.t1, PI / 2, 1e-12, 'up t1'); near(b.t2, -PI / 2, 1e-12, 'up t2'); assert.strictEqual(b.reachable, true);
});

group('check: ik(2, 1, 3, 0, ±1) -> θ₁ = θ₂ = 0, reachable', function () {
  [1, -1].forEach(function (e) {
    const s = m.ik(2, 1, 3, 0, e);
    near(s.t1, 0, 1e-12, 't1 e=' + e); near(s.t2, 0, 1e-12, 't2 e=' + e);
    assert.ok(!Object.is(s.t2, -0), 't2 is not −0');
    assert.strictEqual(s.reachable, true, 'reachable e=' + e);
    near(s.x, 3, 1e-12, 'x'); near(s.y, 0, 1e-12, 'y');
  });
});

group('check: ik(1, 1, 3, 0, +1) -> unreachable, solved (2, 0), θ₁ = θ₂ = 0', function () {
  const s = m.ik(1, 1, 3, 0, 1);
  assert.strictEqual(s.reachable, false);
  near(s.x, 2, 1e-12, 'x'); near(s.y, 0, 1e-12, 'y');
  near(s.t1, 0, 1e-12, 't1'); near(s.t2, 0, 1e-12, 't2');
});

group('check: ik(2, 1, 0.5, 0, +1) -> unreachable, solved (1, 0), θ₂ = 180°', function () {
  const s = m.ik(2, 1, 0.5, 0, 1);
  assert.strictEqual(s.reachable, false);
  near(s.x, 1, 1e-12, 'x'); near(s.y, 0, 1e-12, 'y');
  near(s.t2, PI, 1e-12, 't2'); angNear(s.t1, 0, 1e-12, 't1');
  const f = m.fk(2, 1, s.t1, s.t2);
  near(f.x, 1, 1e-12, 'fk x'); near(f.y, 0, 1e-12, 'fk y');
  const u = m.ik(2, 1, 0.5, 0, -1);
  near(u.t2, -PI, 1e-12, 'elbow up t2 = −180°');
});

group('check: jacobian(1, 1, 0, π/2) = [[−1, −1], [1, 0]], tipVelocity -> (−1.5, 1)', function () {
  const J = m.jacobian(1, 1, 0, PI / 2);
  near(J[0][0], -1, 1e-12, 'J00'); near(J[0][1], -1, 1e-12, 'J01');
  near(J[1][0], 1, 1e-12, 'J10'); near(J[1][1], 0, 1e-12, 'J11');
  const v = m.tipVelocity(1, 1, 0, PI / 2, 1, 0.5);
  near(v.vx, -1.5, 1e-12, 'vx'); near(v.vy, 1, 1e-12, 'vy'); near(v.speed, Math.hypot(1.5, 1), 1e-12, 'speed');
});

group('check: other(1, 1, 0, π/2) -> (π/2, −π/2)', function () {
  const o = m.other(1, 1, 0, PI / 2);
  near(o.t1, PI / 2, 1e-12, 't1'); near(o.t2, -PI / 2, 1e-12, 't2');
});

group('edge cases: unreachable directions, origin, degenerate L₁ = L₂', function () {
  // outside: direction kept
  let s = m.ik(1, 0.8, 3, 4, 1);
  assert.strictEqual(s.reachable, false);
  near(s.x, 1.8 * 0.6, 1e-12, 'outer x'); near(s.y, 1.8 * 0.8, 1e-12, 'outer y'); near(s.t2, 0, 1e-12, 'outer t2');
  near(s.t1, Math.atan2(4, 3), 1e-12, 'outer t1');
  // inside the hole, L1 < L2
  s = m.ik(1, 2, 0, 0.5, -1);
  assert.strictEqual(s.reachable, false);
  near(s.x, 0, 1e-12, 'hole x'); near(s.y, 1, 1e-12, 'hole y'); near(s.t2, -PI, 1e-12, 'hole t2');
  let f = m.fk(1, 2, s.t1, s.t2); near(f.x, 0, 1e-12, 'hole fk x'); near(f.y, 1, 1e-12, 'hole fk y');
  // target exactly at the origin, L1 ≠ L2: +x direction
  s = m.ik(2, 1, 0, 0, 1);
  assert.strictEqual(s.reachable, false);
  near(s.x, 1, 1e-12, 'origin x'); near(s.y, 0, 1e-12, 'origin y'); near(s.t2, PI, 1e-12, 'origin t2');
  // L1 = L2, origin: reachable, θ₂ = ±π, θ₁ = caller's
  s = m.ik(1, 1, 0, 0, 1, 0.7);
  assert.strictEqual(s.reachable, true);
  near(s.t2, PI, 1e-12, 'degenerate t2'); near(s.t1, 0.7, 1e-12, 'degenerate t1');
  s = m.ik(1.5, 1.5, 0, 0, -1, -2.5);
  near(s.t2, -PI, 1e-12, 'degenerate up t2'); near(s.t1, -2.5, 1e-12, 'degenerate up t1');
  s = m.ik(1, 1, 0, 0, 1, 4);  // wrapped
  near(s.t1, 4 - 2 * PI, 1e-12, 'degenerate t1 wrapped');
  s = m.ik(1, 1, 0, 0, 1);     // default t1Current = 0
  near(s.t1, 0, 0, 'default t1Current');
  // phi wrap to (−π, π]
  near(m.fk(1, 1, PI, PI).phi, 0, 1e-12, 'phi wrap 2π');
  near(m.fk(1, 1, -PI / 2, -PI / 2).phi, PI, 1e-12, 'phi −π → π');
});

group('singular()', function () {
  assert.strictEqual(m.singular(0), 'stretched');
  assert.strictEqual(m.singular(0.04), 'stretched');
  assert.strictEqual(m.singular(-0.04), 'stretched');
  assert.strictEqual(m.singular(0.06), '');
  assert.strictEqual(m.singular(PI), 'folded');
  assert.strictEqual(m.singular(-PI), 'folded');
  assert.strictEqual(m.singular(PI - 0.04), 'folded');
  assert.strictEqual(m.singular(PI / 2), '');
  assert.strictEqual(m.singular(-2), '');
});

group('workspace radii', function () {
  const r = rng(11);
  near(m.workspace(1, 0.8).rMin, 0.2, 1e-12, 'rMin'); near(m.workspace(1, 0.8).rMax, 1.8, 1e-12, 'rMax');
  near(m.workspace(0.5, 2).rMin, 1.5, 1e-12, 'rMin L2 > L1');
  near(m.workspace(1, 1).rMin, 0, 0, 'rMin L1 = L2');
  for (let i = 0; i < N; i++) {
    const s = sample(r);
    const w = m.workspace(s.L1, s.L2);
    near(w.rMin, Math.abs(s.L1 - s.L2), 1e-15, 'rMin'); near(w.rMax, s.L1 + s.L2, 1e-15, 'rMax');
    const f = m.fk(s.L1, s.L2, s.t1, s.t2);
    const rr = Math.hypot(f.x, f.y);
    assert.ok(rr >= w.rMin - 1e-12 && rr <= w.rMax + 1e-12, 'fk tip inside the annulus');
  }
  return N + ' samples';
});

group('invariant: ik(fk(θ)) with elbow = sign(θ₂) reproduces θ (mod 2π) and the point', function () {
  const r = rng(12345);
  for (let i = 0; i < N; i++) {
    const s = sample(r);
    const f = m.fk(s.L1, s.L2, s.t1, s.t2);
    const e = s.t2 >= 0 ? 1 : -1;
    const q = m.ik(s.L1, s.L2, f.x, f.y, e);
    assert.strictEqual(q.reachable, true, 'reachable #' + i);
    angNear(q.t1, s.t1, 1e-9, 't1 #' + i); angNear(q.t2, s.t2, 1e-9, 't2 #' + i);
    near(q.x, f.x, 1e-9, 'x #' + i); near(q.y, f.y, 1e-9, 'y #' + i);
    const g = m.fk(s.L1, s.L2, q.t1, q.t2);
    near(g.x, f.x, 1e-9, 'fk x #' + i); near(g.y, f.y, 1e-9, 'fk y #' + i);
  }
  return N + ' samples';
});

group('invariant: fk(other(θ)) gives the same tip; opposite-branch ik equals other()', function () {
  const r = rng(777);
  for (let i = 0; i < N; i++) {
    const s = sample(r);
    const f = m.fk(s.L1, s.L2, s.t1, s.t2);
    const o = m.other(s.L1, s.L2, s.t1, s.t2);
    near(o.t2, -s.t2, 0, 't2′ = −t2 #' + i);
    assert.ok(o.t1 > -PI && o.t1 <= PI, 't1′ wrapped #' + i);
    const g = m.fk(s.L1, s.L2, o.t1, o.t2);
    near(g.x, f.x, 1e-9, 'x #' + i); near(g.y, f.y, 1e-9, 'y #' + i);
    const q = m.ik(s.L1, s.L2, f.x, f.y, s.t2 >= 0 ? -1 : 1);
    angNear(q.t1, o.t1, 1e-9, 'ik other-branch t1 #' + i); angNear(q.t2, o.t2, 1e-9, 'ik other-branch t2 #' + i);
  }
  return N + ' samples';
});

group('invariant: det(jacobian()) = detJ()', function () {
  const r = rng(4242);
  for (let i = 0; i < N; i++) {
    const s = sample(r);
    const J = m.jacobian(s.L1, s.L2, s.t1, s.t2);
    near(J[0][0] * J[1][1] - J[0][1] * J[1][0], m.detJ(s.L1, s.L2, s.t2), 1e-12, 'det #' + i);
  }
  return N + ' samples';
});

group('invariant: jacobian() = central finite difference of fk (1e-6); tipVelocity = J·ω', function () {
  const r = rng(99);
  const h = 1e-6;
  for (let i = 0; i < N; i++) {
    const s = sample(r);
    const J = m.jacobian(s.L1, s.L2, s.t1, s.t2);
    const a = m.fk(s.L1, s.L2, s.t1 + h, s.t2), b = m.fk(s.L1, s.L2, s.t1 - h, s.t2);
    const c = m.fk(s.L1, s.L2, s.t1, s.t2 + h), d = m.fk(s.L1, s.L2, s.t1, s.t2 - h);
    near(J[0][0], (a.x - b.x) / (2 * h), 1e-6, 'dx/dt1 #' + i);
    near(J[1][0], (a.y - b.y) / (2 * h), 1e-6, 'dy/dt1 #' + i);
    near(J[0][1], (c.x - d.x) / (2 * h), 1e-6, 'dx/dt2 #' + i);
    near(J[1][1], (c.y - d.y) / (2 * h), 1e-6, 'dy/dt2 #' + i);
    const v = m.tipVelocity(s.L1, s.L2, s.t1, s.t2, s.w1, s.w2);
    near(v.vx, J[0][0] * s.w1 + J[0][1] * s.w2, 1e-12, 'vx #' + i);
    near(v.vy, J[1][0] * s.w1 + J[1][1] * s.w2, 1e-12, 'vy #' + i);
    near(v.speed, Math.hypot(v.vx, v.vy), 1e-12, 'speed #' + i);
  }
  return N + ' samples';
});

group('invariant: at θ₂ = 0 the tip velocity is perpendicular to the arm', function () {
  const r = rng(2027);
  for (let i = 0; i < N; i++) {
    const s = sample(r);
    const v = m.tipVelocity(s.L1, s.L2, s.t1, 0, s.w1, s.w2);
    const dot = v.vx * Math.cos(s.t1) + v.vy * Math.sin(s.t1);
    near(dot, 0, 1e-12, 'v·u #' + i);
    near(m.detJ(s.L1, s.L2, 0), 0, 0, 'detJ = 0');
  }
  return N + ' samples';
});

group('invariant: unreachable targets are projected radially onto the nearest boundary', function () {
  const r = rng(31337);
  let n = 0;
  for (let i = 0; i < N; i++) {
    const s = sample(r);
    const w = m.workspace(s.L1, s.L2);
    const rad = 3 * w.rMax * r(), a = -PI + 2 * PI * r();
    const x = rad * Math.cos(a), y = rad * Math.sin(a);
    const q = m.ik(s.L1, s.L2, x, y, r() < 0.5 ? 1 : -1, s.t1);
    const inside = rad >= w.rMin && rad <= w.rMax;
    assert.strictEqual(q.reachable, inside, 'reachable flag #' + i);
    if (!inside) {
      n++;
      const rc = Math.min(Math.max(rad, w.rMin), w.rMax);
      near(Math.hypot(q.x, q.y), rc, 1e-12, 'clamped radius #' + i);
      if (rc > 0) angNear(Math.atan2(q.y, q.x), a, 1e-12, 'direction kept #' + i);
    }
    const f = m.fk(s.L1, s.L2, q.t1, q.t2);
    near(f.x, q.x, 1e-9, 'fk x #' + i); near(f.y, q.y, 1e-9, 'fk y #' + i);
  }
  return N + ' samples, ' + n + ' unreachable';
});

if (failures) { console.log(failures + ' group(s) FAILED'); process.exit(1); }
console.log('all groups passed');
