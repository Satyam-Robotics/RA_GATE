// Node tests for notes/sims/core.js (pure functions only).
// Run: node notes/sims/test/core.test.js   (exits non-zero on failure)
'use strict';
const assert = require('assert');
const RASim = require('./load.js')();
const M = RASim.math;

let failed = 0;
function group(name, fn) {
  try { fn(); console.log('ok   ' + name); } catch (e) { failed++; console.log('FAIL ' + name + '\n     ' + (e && e.stack || e)); }
}
function near(a, b, tol, msg) {
  const ok = Math.abs(a - b) <= tol * Math.max(1, Math.abs(b));
  assert.ok(ok, (msg || '') + ` expected ${b}, got ${a} (tol ${tol})`);
}
function nearAbs(a, b, tol, msg) {
  assert.ok(Math.abs(a - b) <= tol, (msg || '') + ` expected ${b}, got ${a} (abs tol ${tol})`);
}
function rootsMatch(got, want, tol, msg) {
  assert.strictEqual(got.length, want.length, msg + ': root count');
  const used = want.map(() => false);
  got.forEach(g => {
    let best = -1, bd = Infinity;
    want.forEach((w, i) => {
      if (used[i]) return;
      const wc = typeof w === 'number' ? { re: w, im: 0 } : w;
      const d = Math.hypot(g.re - wc.re, g.im - wc.im) / Math.max(1, Math.hypot(wc.re, wc.im));
      if (d < bd) { bd = d; best = i; }
    });
    assert.ok(bd <= tol, `${msg}: root ${JSON.stringify(g)} not within ${tol} of any expected (best ${bd})`);
    used[best] = true;
  });
}
function checkOrderAndConj(r, msg) {
  for (let i = 1; i < r.length; i++) {
    const d = r[i - 1].re - r[i].re;
    assert.ok(d > 1e-9 || (Math.abs(d) <= 1e-9 && r[i - 1].im >= r[i].im), msg + ': ordering');
  }
  r.forEach(x => {
    if (x.im !== 0) assert.ok(r.some(y => y.re === x.re && y.im === -x.im), msg + ': exact conjugate of ' + JSON.stringify(x));
  });
}

group('polyRoots', () => {
  rootsMatch(M.polyRoots([1, 6, 11, 6]), [-1, -2, -3], 1e-9, 'cubic');
  const r1 = M.polyRoots([1, 6, 11, 6]);
  assert.deepStrictEqual(r1.map(r => r.im), [0, 0, 0]);
  near(r1[0].re, -1, 1e-9); near(r1[1].re, -2, 1e-9); near(r1[2].re, -3, 1e-9);
  const r2 = M.polyRoots([1, 2, 5]);
  assert.strictEqual(r2.length, 2);
  near(r2[0].re, -1, 1e-12); near(r2[0].im, 2, 1e-12); near(r2[1].im, -2, 1e-12);
  assert.strictEqual(r2[0].re, r2[1].re); assert.strictEqual(r2[0].im, -r2[1].im);
  rootsMatch(M.polyRoots([1, 3, 3, 1]), [-1, -1, -1], 1e-3, 'triple');
  const r4 = M.polyRoots([1, 3, 2, 0]);
  rootsMatch(r4, [0, -1, -2], 1e-9, 'origin');
  assert.strictEqual(r4[0].re, 0); assert.strictEqual(r4[0].im, 0);
  assert.ok(!Object.is(r4[0].re, -0), 'no -0 at origin');
  const h = Math.SQRT1_2;
  const r5 = M.polyRoots([1, 0, 0, 0, 1]);
  rootsMatch(r5, [{ re: h, im: h }, { re: h, im: -h }, { re: -h, im: h }, { re: -h, im: -h }], 1e-9, 's^4+1');
  checkOrderAndConj(r5, 's^4+1');
  const known = [-0.5, -1, -2.5, -4, { re: -1, im: 2 }, { re: -1, im: -2 }, { re: -3, im: 0.75 }, { re: -3, im: -0.75 }];
  const p8 = M.polyFromRoots(known);
  assert.strictEqual(p8.length, 9);
  const r8 = M.polyRoots(p8);
  rootsMatch(r8, known, 1e-7, 'degree 8 round trip');
  checkOrderAndConj(r8, 'degree 8');
  // round trip through polyFromRoots again
  const p8b = M.polyFromRoots(r8);
  p8.forEach((c, i) => near(p8b[i], c, 1e-7, 'coeff ' + i));
  // leading zeros
  rootsMatch(M.polyRoots([0, 0, 1, 3, 2]), [-1, -2], 1e-12, 'leading zeros');
  assert.deepStrictEqual(M.polyRoots([0, 5]), []);
  assert.deepStrictEqual(M.polyRoots([0, 0]), []);
  // ordering: descending re then descending im
  const r6 = M.polyRoots(M.polyFromRoots([-5, 2, { re: 2, im: 1 }, { re: 2, im: -1 }, -1]));
  assert.deepStrictEqual(r6.map(r => [Math.round(r.re * 1e6) / 1e6, Math.round(r.im * 1e6) / 1e6]), [[2, 1], [2, 0], [2, -1], [-1, 0], [-5, 0]]);
  checkOrderAndConj(r6, 'ordering');
  // higher degree, unstable and widely spread roots
  const wide = [-100, -10, -1, -0.1, 3, { re: -2, im: 50 }, { re: -2, im: -50 }, 0.5, -7, -20];
  rootsMatch(M.polyRoots(M.polyFromRoots(wide)), wide, 1e-6, 'degree 10 wide');
  // double root at origin among others
  rootsMatch(M.polyRoots([1, 2, 0, 0]), [0, 0, -2], 1e-12, 'double origin');
  // repeated complex pair
  const rc = M.polyRoots(M.polyFromRoots([{ re: -1, im: 1 }, { re: -1, im: -1 }, { re: -1, im: 1 }, { re: -1, im: -1 }]));
  rootsMatch(rc, [{ re: -1, im: 1 }, { re: -1, im: 1 }, { re: -1, im: -1 }, { re: -1, im: -1 }], 1e-5, 'repeated complex pair');
  // regression (review): a double real root used to be "paired" with an unrelated complex root,
  // averaging both into garbage (-0.083 ± 0.018j here)
  const dbl = [-0.034367297753639364, -0.034367297753639364, { re: -0.13157050211705879, im: 0.03672165956158923 }, { re: -0.13157050211705879, im: -0.03672165956158923 },
    { re: -0.2412262666526187, im: 0.09654060676588878 }, { re: -0.2412262666526187, im: -0.09654060676588878 }, -0.1666630607138702, -0.1256434490631893];
  const rd = M.polyRoots(M.polyFromRoots(dbl));
  rootsMatch(rd, dbl, 1e-6, 'double root among complex pairs');
  checkOrderAndConj(rd, 'double root among complex pairs');
  // regression (review): members of a triple real root must not come back as unpaired complex roots
  let seed = 3;
  const rnd = () => { seed = (seed * 16807) % 2147483647; return seed / 2147483647; };
  for (let k = 0; k < 150; k++) {
    const r0 = -10 * rnd(), mult = 2 + (k % 3);
    const rs = []; for (let i = 0; i < mult; i++) rs.push(r0);
    const re = -5 * rnd(), im = 0.1 + 5 * rnd();
    rs.push({ re, im }, { re, im: -im }, -3 * rnd());
    const sc = 0.01 + 100 * rnd();
    const got = M.polyRoots(M.polyFromRoots(rs).map(c => c * sc));
    assert.strictEqual(got.length, rs.length);
    checkOrderAndConj(got, 'repeated root fuzz ' + k);
    assert.strictEqual(got.filter(z => z.im > 0).length, got.filter(z => z.im < 0).length, 'conjugate symmetry ' + k);
    rootsMatch(got, rs, mult === 4 ? 2e-3 : 1e-3, 'repeated root fuzz ' + k);
  }
});

group('polynomial arithmetic', () => {
  assert.deepStrictEqual(M.polyMul([1, 1], [1, 2]), [1, 3, 2]);
  assert.deepStrictEqual(M.polyMul([2], [1, 0, -1]), [2, 0, -2]);
  assert.deepStrictEqual(M.polyAdd([1, 2, 3], [1, 1]), [1, 3, 4]);
  assert.deepStrictEqual(M.polyAdd([1, 2], [-1, 0]), [2]);
  assert.deepStrictEqual(M.polySub([1, 2, 3], [1, 2, 3]), [0]);
  assert.deepStrictEqual(M.polySub([3, 1], [1]), [3, 0]);
  assert.deepStrictEqual(M.polyScale([1, -2], 3), [3, -6]);
  assert.deepStrictEqual(M.polyDeriv([1, 3, 3, 1]), [3, 6, 3]);
  assert.deepStrictEqual(M.polyDeriv([5]), [0]);
  assert.deepStrictEqual(M.polyTrim([0, 0, 1, 2]), [1, 2]);
  assert.deepStrictEqual(M.polyTrim([0, 0]), [0]);
  assert.deepStrictEqual(M.polyTrim([]), [0]);
  const v = M.polyEval([1, 2, 5], { re: -1, im: 2 });
  nearAbs(v.re, 0, 1e-12); nearAbs(v.im, 0, 1e-12);
  assert.deepStrictEqual(M.polyEval([1, 3, 2], 2), { re: 12, im: 0 });
  const w = M.polyEval([1, 0, 1], { re: 0, im: 1 }); nearAbs(w.re, 0, 1e-15); nearAbs(w.im, 0, 1e-15);
  assert.deepStrictEqual(M.polyFromRoots([]), [1]);
  assert.deepStrictEqual(M.polyFromRoots([-1, -2]), [1, 3, 2]);
  assert.deepStrictEqual(M.polyFromRoots([{ re: -1, im: 2 }, { re: -1, im: -2 }]), [1, 2, 5]);
  assert.throws(() => M.polyFromRoots([{ re: -1, im: 2 }]));
  // product rule identity d(ab) = a'b + ab'
  const a = [1, -2, 3], b = [2, 0, 1, 5];
  assert.deepStrictEqual(M.polyDeriv(M.polyMul(a, b)), M.polyAdd(M.polyMul(M.polyDeriv(a), b), M.polyMul(a, M.polyDeriv(b))));
  const t = M.tfEval([1], [1, 1], { re: 0, im: 1 });
  near(t.re, 0.5, 1e-15); near(t.im, -0.5, 1e-15);
});

group('tfStep', () => {
  let r = M.tfStep([1], [1, 1], 5, 500);
  assert.strictEqual(r.t.length, 501); assert.strictEqual(r.y.length, 501);
  assert.strictEqual(r.t[0], 0); assert.strictEqual(r.t[500], 5);
  near(r.y[100], 1 - Math.exp(-1), 1e-12, '1/(s+1) at t=1');
  near(r.y[100], 0.632121, 1e-6);
  r.t.forEach((t, i) => near(r.y[i], 1 - Math.exp(-t), 1e-10));
  assert.strictEqual(M.tfStep([1], [1, 1], 2).t.length, 401);
  // second order
  const z = 0.5, wn = 2, wd = wn * Math.sqrt(1 - z * z);
  r = M.tfStep([wn * wn], [1, 2 * z * wn, wn * wn], 10, 50);
  r.t.forEach((t, i) => {
    const ex = 1 - Math.exp(-z * wn * t) * Math.sin(wd * t + Math.acos(z)) / Math.sqrt(1 - z * z);
    nearAbs(r.y[i], ex, 1e-9, 'second order t=' + t);
  });
  r = M.tfStep([1], [1, 0], 3, 300);
  r.t.forEach((t, i) => nearAbs(r.y[i], t, 1e-12, 'ramp'));
  r = M.tfStep([1], [1, 0, 0], 3, 300);
  r.t.forEach((t, i) => nearAbs(r.y[i], t * t / 2, 1e-11, 't^2/2'));
  r = M.tfStep([1, 2], [1, 1], 8, 400);
  assert.strictEqual(r.y[0], 1);
  r.t.forEach((t, i) => nearAbs(r.y[i], 2 - Math.exp(-t), 1e-11, '(s+2)/(s+1)'));
  near(r.y[400], 2, 1e-3);
  r = M.tfStep([1000], M.polyMul([1, 1], [1, 1000]), 10, 400);
  r.t.forEach((t, i) => nearAbs(r.y[i], 1 - (1000 / 999) * Math.exp(-t) + Math.exp(-1000 * t) / 999, 1e-9, 'stiff t=' + t));
  r = M.tfStep([1], [1, -1], 5, 400);
  r.t.forEach((t, i) => near(r.y[i], Math.exp(t) - 1, 1e-9, 'unstable t=' + t));
  r = M.tfStep([1], [1, 3, 3, 1], 12, 400);
  r.t.forEach((t, i) => nearAbs(r.y[i], 1 - Math.exp(-t) * (1 + t + t * t / 2), 1e-10, 'triple pole'));
  r = M.tfStep([3], [2], 1, 10);
  assert.strictEqual(r.y.length, 11); r.y.forEach(y => assert.strictEqual(y, 1.5));
  assert.throws(() => M.tfStep([1, 0, 0], [1, 1], 1), /deg/);
  // leading zeros in num are not "improper"
  r = M.tfStep([0, 0, 1], [1, 1], 1, 10);
  near(r.y[10], 1 - Math.exp(-1), 1e-12);
  // undamped oscillator
  r = M.tfStep([4], [1, 0, 4], 20, 400);
  r.t.forEach((t, i) => nearAbs(r.y[i], 1 - Math.cos(2 * t), 1e-9, 'undamped'));
});

group('expm', () => {
  const I = M.expm([[0, 0], [0, 0]]);
  assert.deepStrictEqual(I, [[1, 0], [0, 1]]);
  [0.3, 1, 2.5, 10, 100].forEach(t => {
    const R = M.expm([[0, t], [-t, 0]]);
    nearAbs(R[0][0], Math.cos(t), 1e-11 * Math.max(1, t)); nearAbs(R[0][1], Math.sin(t), 1e-11 * Math.max(1, t));
    nearAbs(R[1][0], -Math.sin(t), 1e-11 * Math.max(1, t)); nearAbs(R[1][1], Math.cos(t), 1e-11 * Math.max(1, t));
  });
  const N = M.expm([[0, 1, 0], [0, 0, 1], [0, 0, 0]]);
  const ex = [[1, 1, 0.5], [0, 1, 1], [0, 0, 1]];
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) nearAbs(N[i][j], ex[i][j], 1e-14);
  let seed = 7;
  const rnd = () => { seed = (seed * 16807) % 2147483647; return seed / 2147483647 - 0.5; };
  const A = [0, 1, 2, 3].map(() => [0, 1, 2, 3].map(() => 4 * rnd()));
  const P = M.matMul(M.expm(A), M.expm(A.map(r => r.map(v => -v))));
  for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++) nearAbs(P[i][j], i === j ? 1 : 0, 1e-10);
  const D = M.expm([[-1000, 0], [0, 2]]);
  near(D[1][1], Math.exp(2), 1e-12); nearAbs(D[0][0], 0, 1e-300);
  const S = M.expm([[1]]); near(S[0][0], Math.E, 1e-14);
});

group('bisect, zeroCrossings', () => {
  near(M.bisect(x => x * x - 2, 0, 2), Math.SQRT2, 1e-12);
  assert.strictEqual(M.bisect(x => x - 1, 1, 3), 1);
  assert.strictEqual(M.bisect(x => x - 3, 1, 3), 3);
  near(M.bisect(Math.cos, 0, 3, 1e-6), Math.PI / 2, 1e-6);
  const zc = M.zeroCrossings(Math.sin, 0.5, 10);
  assert.strictEqual(zc.length, 3);
  [Math.PI, 2 * Math.PI, 3 * Math.PI].forEach((v, i) => near(zc[i], v, 1e-10));
  const zl = M.zeroCrossings(w => Math.log10(w) - 1.5, 0.01, 1e4, 200, { log: true });
  assert.strictEqual(zl.length, 1); near(zl[0], Math.pow(10, 1.5), 1e-10);
  const z0 = M.zeroCrossings(x => x, -1, 1, 4);   // samples hit 0 exactly
  assert.deepStrictEqual(z0, [0]);
  const zinf = M.zeroCrossings(x => 1 / x, -1, 1, 401);   // pole: no crossing across non-finite / sign flip of a pole is reported?
  assert.ok(zinf.length <= 1);
  const znan = M.zeroCrossings(x => (Math.abs(x) < 0.3 ? NaN : x), -1, 1, 40);
  assert.deepStrictEqual(znan, []);
  assert.deepStrictEqual(M.zeroCrossings(x => x * x + 1, -3, 3), []);
  // regression (review): ascending even when a > b
  const zr = M.zeroCrossings(Math.sin, 10, 0.5);
  assert.strictEqual(zr.length, 3); assert.ok(zr[0] < zr[1] && zr[1] < zr[2], 'ascending for a > b');
});

group('ticks, spacing, angles', () => {
  assert.strictEqual(M.niceCeil(0.3), 0.5);
  assert.strictEqual(M.niceCeil(2), 2);
  assert.strictEqual(M.niceCeil(2.1), 2.5);
  assert.strictEqual(M.niceCeil(7), 10);
  assert.strictEqual(M.niceCeil(1), 1);
  assert.strictEqual(M.niceCeil(0.0011), 0.002);
  assert.strictEqual(M.niceCeil(3e5), 5e5);
  assert.strictEqual(M.niceCeil(0.30000000000000004), 0.5);
  assert.deepStrictEqual(M.niceTicks(0, 10), [0, 2, 4, 6, 8, 10]);
  assert.deepStrictEqual(M.niceTicks(-0.37, 1.23), [0, 0.5, 1]);
  const nt = M.niceTicks(-0.37, 1.23, 8);
  assert.deepStrictEqual(nt, [-0.2, 0, 0.2, 0.4, 0.6, 0.8, 1, 1.2]);
  assert.deepStrictEqual(M.niceTicks(-1.1, 1.1, 10), [-1, -0.75, -0.5, -0.25, 0, 0.25, 0.5, 0.75, 1]);
  const nt2 = M.niceTicks(0.1, 0.7, 6);
  assert.deepStrictEqual(nt2, [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]);
  nt2.concat(nt).forEach(v => assert.ok(String(v).length < 8, 'clean tick ' + v));
  M.niceTicks(-3.3, 17.9, 5).forEach(v => assert.ok(v >= -3.3 && v <= 17.9));
  assert.deepStrictEqual(M.linspace(0, 1, 5), [0, 0.25, 0.5, 0.75, 1]);
  assert.strictEqual(M.linspace(0, 0.3, 7)[6], 0.3);
  const ls = M.logspace(0.1, 1000, 5);
  assert.strictEqual(ls[0], 0.1); assert.strictEqual(ls[4], 1000);
  near(ls[1], 1, 1e-12); near(ls[2], 10, 1e-12); near(ls[3], 100, 1e-12);
  near(M.wrapPi(3 * Math.PI), Math.PI, 1e-12);
  near(M.wrapPi(-Math.PI), Math.PI, 1e-12);
  near(M.wrapPi(Math.PI), Math.PI, 1e-12);
  near(M.wrapPi(-3 * Math.PI / 2), Math.PI / 2, 1e-12);
  near(M.wrapPi(0.5), 0.5, 1e-15);
  assert.strictEqual(M.wrapDeg(180), 180);
  assert.strictEqual(M.wrapDeg(-180), 180);
  assert.strictEqual(M.wrapDeg(190), -170);
  assert.strictEqual(M.wrapDeg(540), 180);
  assert.strictEqual(M.wrapDeg(-190), 170);
  assert.strictEqual(M.wrapDeg(30), 30);
  assert.deepStrictEqual(M.unwrapDeg([170, -170, -150, 170, 10]), [170, 190, 210, 170, 10]);
  assert.deepStrictEqual(M.unwrapDeg([-170, 170, 150]), [-170, -190, -210]);
  assert.deepStrictEqual(M.unwrapDeg([0, -90, -180, 90, 0]), [0, -90, -180, -270, -360]);
  near(M.rad(180), Math.PI, 1e-15); near(M.deg(Math.PI / 2), 90, 1e-12);
  assert.strictEqual(M.clamp(5, 0, 3), 3); assert.strictEqual(M.clamp(-1, 0, 3), 0); assert.strictEqual(M.clamp(2, 0, 3), 2);
});

group('complex helpers', () => {
  assert.deepStrictEqual(M.cx(1), { re: 1, im: 0 });
  assert.deepStrictEqual(M.cadd(M.cx(1, 2), 3), { re: 4, im: 2 });
  assert.deepStrictEqual(M.csub(M.cx(1, 2), M.cx(1, 1)), { re: 0, im: 1 });
  assert.deepStrictEqual(M.cmul(M.cx(1, 2), M.cx(3, -1)), { re: 5, im: 5 });
  const q = M.cdiv(M.cx(5, 5), M.cx(3, -1)); near(q.re, 1, 1e-15); near(q.im, 2, 1e-15);
  const q2 = M.cdiv(1, M.cx(0, 2)); near(q2.re, 0, 1e-15); near(q2.im, -0.5, 1e-15);
  const q3 = M.cdiv(M.cx(1e300, 1e300), M.cx(1e300, 1e300)); near(q3.re, 1, 1e-15); nearAbs(q3.im, 0, 1e-15);
  assert.deepStrictEqual(M.csqrt(-4), { re: 0, im: 2 });
  const s = M.csqrt(M.cx(3, 4)); near(s.re, 2, 1e-15); near(s.im, 1, 1e-15);
  const s2 = M.csqrt(M.cx(3, -4)); near(s2.re, 2, 1e-15); near(s2.im, -1, 1e-15);
  const e = M.cexp(M.cx(0, Math.PI)); near(e.re, -1, 1e-15); nearAbs(e.im, 0, 1e-15);
  assert.deepStrictEqual(M.cconj(M.cx(1, 2)), { re: 1, im: -2 });
  assert.strictEqual(M.cabs(M.cx(3, 4)), 5);
  near(M.carg(M.cx(-1, 0)), Math.PI, 1e-15);
  assert.deepStrictEqual(M.cscale(M.cx(1, -2), 2), { re: 2, im: -4 });
});

group('rk4', () => {
  let y = [1], t = 0;
  const h = 0.01;
  for (let i = 0; i < 100; i++) { y = M.rk4((tt, yy) => [-yy[0]], t, y, h); t += h; }
  near(y[0], Math.exp(-1), 1e-9);
  let z = [1, 0];
  for (let i = 0; i < 628; i++) z = M.rk4((tt, v) => [v[1], -v[0]], 0, z, 0.01);
  near(z[0], Math.cos(6.28), 1e-8);
});

group('fmt / fmtFixed / fmtComplex / fmtPoly', () => {
  const MINUS = '−';
  assert.strictEqual(RASim.fmt(NaN), '—');
  assert.strictEqual(RASim.fmt(Infinity), '∞');
  assert.strictEqual(RASim.fmt(-Infinity), MINUS + '∞');
  assert.strictEqual(RASim.fmt(0), '0');
  assert.strictEqual(RASim.fmt(-0), '0');
  assert.strictEqual(RASim.fmt(-1e-13), '0');
  assert.strictEqual(RASim.fmt(2.5), '2.5');
  assert.strictEqual(RASim.fmt(3), '3');
  assert.strictEqual(RASim.fmt(2.500), '2.5');
  assert.ok(['1235', '1234'].includes(RASim.fmt(1234.5)));
  assert.strictEqual(RASim.fmt(1234.56), '1235');
  assert.strictEqual(RASim.fmt(-1.5), MINUS + '1.5');
  assert.strictEqual(RASim.fmt(0.000123456), '0.0001235');
  assert.strictEqual(RASim.fmt(1.2e-5), '1.2×10⁻⁵');
  assert.strictEqual(RASim.fmt(-1.2e-5), MINUS + '1.2×10⁻⁵');
  assert.strictEqual(RASim.fmt(1.23456e7), '1.235×10⁷');
  assert.strictEqual(RASim.fmt(9999999), '1×10⁷');
  assert.strictEqual(RASim.fmt(1e7), '1×10⁷');
  assert.strictEqual(RASim.fmt(6543210), '6543000');
  assert.strictEqual(RASim.fmt(1e-4), '0.0001');
  assert.strictEqual(RASim.fmt(1e21), '1×10²¹');
  assert.strictEqual(RASim.fmt(Math.PI, 3), '3.14');
  assert.strictEqual(RASim.fmt(Math.PI, 6), '3.14159');
  assert.strictEqual(RASim.fmt(100), '100');
  assert.strictEqual(RASim.fmt(0.1 + 0.2), '0.3');
  assert.strictEqual(RASim.fmt(-0.00001), MINUS + '1×10⁻⁵');
  assert.strictEqual(RASim.fmtFixed(1.005, 1), '1.0');
  assert.strictEqual(RASim.fmtFixed(2), '2.00');
  assert.strictEqual(RASim.fmtFixed(-2.345, 1), MINUS + '2.3');
  assert.strictEqual(RASim.fmtFixed(-0.001), '0.00');
  assert.strictEqual(RASim.fmtFixed(-0), '0.00');
  assert.strictEqual(RASim.fmtFixed(-1e-13, 3), '0.000');
  assert.strictEqual(RASim.fmtFixed(NaN), '—');
  assert.strictEqual(RASim.fmtFixed(-Infinity), MINUS + '∞');
  assert.strictEqual(RASim.fmtFixed(Infinity), '∞');
  assert.strictEqual(RASim.fmtFixed(3.14159, 0), '3');
  // regression (review): toFixed switches to exponent notation at 1e21
  assert.strictEqual(RASim.fmtFixed(-2e22), MINUS + '2×10²²');
  assert.strictEqual(RASim.fmtComplex({ re: 1, im: Infinity }), '1 + j∞');
  assert.strictEqual(RASim.fmtComplex({ re: -1, im: 1.7320508 }), MINUS + '1 + j1.732');
  assert.strictEqual(RASim.fmtComplex({ re: -1, im: -1.7320508 }), MINUS + '1 ' + MINUS + ' j1.732');
  assert.strictEqual(RASim.fmtComplex({ re: 2.5, im: 0 }), '2.5');
  assert.strictEqual(RASim.fmtComplex(-3), MINUS + '3');
  assert.strictEqual(RASim.fmtComplex({ re: 0, im: 1.7320508 }), 'j1.732');
  assert.strictEqual(RASim.fmtComplex({ re: 0, im: -1.7320508 }), MINUS + 'j1.732');
  assert.strictEqual(RASim.fmtComplex({ re: 0, im: 0 }), '0');
  assert.strictEqual(RASim.fmtComplex({ re: 1, im: 2 }, 2), '1 + j2');
  assert.strictEqual(RASim.fmtPoly([1, 3, 2, 0]), 's<sup>3</sup> + 3s<sup>2</sup> + 2s');
  assert.strictEqual(RASim.fmtPoly([1, 3, 2]), 's<sup>2</sup> + 3s + 2');
  assert.strictEqual(RASim.fmtPoly([-1, 0, -4]), MINUS + 's<sup>2</sup> ' + MINUS + ' 4');
  assert.strictEqual(RASim.fmtPoly([2, -1, 1]), '2s<sup>2</sup> ' + MINUS + ' s + 1');
  assert.strictEqual(RASim.fmtPoly([0, 0, 0]), '0');
  assert.strictEqual(RASim.fmtPoly([1]), '1');
  assert.strictEqual(RASim.fmtPoly([-1]), MINUS + '1');
  assert.strictEqual(RASim.fmtPoly([0, 1, 0.5], 'z'), 'z + 0.5');
  assert.strictEqual(RASim.fmtPoly([1, 0]), 's');
  // never an ASCII hyphen for negative inputs
  [-1, -0.5, -1234.5, -1e-5, -1e9, -Infinity, -3.3333333].forEach(v => {
    assert.ok(!RASim.fmt(v).includes('-'), 'fmt ' + v);
    assert.ok(!RASim.fmtFixed(v, 3).includes('-'), 'fmtFixed ' + v);
  });
  [{ re: -1, im: -2 }, { re: -1e-6, im: -3e8 }, { re: 0, im: -1 }].forEach(z => assert.ok(!RASim.fmtComplex(z).includes('-'), 'fmtComplex'));
  assert.ok(!RASim.fmtPoly([-1, -2e-6, -3]).includes('-'));
});

group('colors / alpha', () => {
  assert.strictEqual(RASim.colors.maroon, '#6b1d1d');
  assert.strictEqual(Object.keys(RASim.colors).length, 18);
  assert.strictEqual(RASim.alpha('#1f4e79', 0.5), 'rgba(31, 78, 121, 0.5)');
});

group('register', () => {
  const def = { title: 'Test', model: { twice: x => 2 * x }, mount: () => ({}) };
  RASim.register('unit-test-sim', def);
  assert.strictEqual(RASim.sims['unit-test-sim'].model.twice(4), 8);
  assert.throws(() => RASim.register('unit-test-sim', def), /duplicate/);
  assert.throws(() => RASim.register('no-mount', { title: 'x' }));
  // loader for sims in other locations
  const load = require('./load.js');
  const R2 = load('demo', require('path').join(__dirname, 'demo-sim.js'));
  assert.strictEqual(R2, RASim);
  assert.strictEqual(typeof RASim.sims.demo.model.secondOrder, 'function');
  RASim.mountAll();   // no-op in Node
});

if (failed) { console.log(`\n${failed} test group(s) FAILED`); process.exit(1); }
console.log('\nAll core tests passed');
