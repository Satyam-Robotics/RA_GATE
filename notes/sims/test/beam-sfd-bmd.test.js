// Tests for beam-sfd-bmd. Run: node notes/sims/test/beam-sfd-bmd.test.js [--no-browser] [--file <sim file>]
'use strict';
const assert = require('assert');
const FILE_ARG = process.argv.indexOf('--file') >= 0 ? require('path').resolve(process.argv[process.argv.indexOf('--file') + 1]) : undefined;
const RASim = require('./load.js')('beam-sfd-bmd', FILE_ARG);
const m = RASim.sims['beam-sfd-bmd'].model;

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
const EI = 10000;
const ss = (L, loads, a, b) => ({ type: 'ss', L: L, a: a == null ? 0 : a, b: b == null ? L : b, EI: EI, loads: loads });
const cant = (L, loads) => ({ type: 'cantilever', L: L, a: 0, b: L, EI: EI, loads: loads });

group('API shape and title', function () {
  assert.strictEqual(RASim.sims['beam-sfd-bmd'].title, 'Beam: shear force, bending moment and deflection');
  assert.deepStrictEqual(Object.keys(m).sort(), ['equilibrium', 'sample', 'solve']);
  const r = m.solve(ss(6, [{ kind: 'point', P: 20, x: 3 }]));
  ['reactions', 'V', 'M', 'slope', 'y', 'criticalX', 'maxV', 'maxSag', 'maxHog', 'contraflexure', 'maxDeflection'].forEach(k => assert.ok(k in r, 'result.' + k));
  assert.deepStrictEqual(Object.keys(r.reactions).sort(), ['Ra', 'Rb']);
  assert.deepStrictEqual(Object.keys(m.solve(cant(4, [])).reactions).sort(), ['Mwall', 'R']);
  ['V', 'M', 'slope', 'y'].forEach(k => assert.strictEqual(typeof r[k], 'function'));
  const s = m.sample(r, 50);
  assert.deepStrictEqual(Object.keys(s).sort(), ['M', 'V', 'x', 'y']);
  const e = m.equilibrium(ss(6, [{ kind: 'point', P: 20, x: 3 }]), r);
  assert.deepStrictEqual(Object.keys(e).sort(), ['sumF', 'sumM']);
  assert.deepStrictEqual(Object.keys(r.maxDeflection).sort(), ['value', 'x']);
});

group('ss, L = 6, P = 20 at 3 (PL/4, PL³/48EI, PL²/16EI)', function () {
  const r = m.solve(ss(6, [{ kind: 'point', P: 20, x: 3 }]));
  printed(r.reactions.Ra, 10, 0, 'R_A'); printed(r.reactions.Rb, 10, 0, 'R_B');
  [0, 0.5, 1, 2, 2.99].forEach(x => printed(r.V(x), 10, 0, 'V(' + x + ')'));
  [3.01, 4, 5, 5.99, 6].forEach(x => printed(r.V(x), -10, 0, 'V(' + x + ')'));
  printed(r.V(3, -1), 10, 0, 'V(3⁻)'); printed(r.V(3, 1), -10, 0, 'V(3⁺)');
  printed(r.V(6, 1), 0, 9, 'V(6⁺) closure');
  printed(r.M(3), 30, 0, 'M(3)'); printed(r.M(1.5), 15, 0, 'M(1.5)');
  printed(r.y(3), -0.009, 3, 'y(3)');
  printed(r.slope(0), -0.0045, 4, 'slope at A'); printed(r.slope(6), 0.0045, 4, 'slope at B');
  assert.strictEqual(r.contraflexure.length, 0, 'no contraflexure');
  printed(r.maxSag.value, 30, 0, 'max sag'); printed(r.maxSag.x, 3, 1, 'max sag x');
  assert.strictEqual(r.maxHog, null, 'no hogging');
  printed(r.maxV.value, 10, 0, 'max |V|');
  printed(r.maxDeflection.value, -0.009, 3, 'max deflection'); printed(r.maxDeflection.x, 3, 3, 'max deflection x');
});

group('ss, L = 6, full UDL w = 5 (wL²/8, 5wL⁴/384EI, wL³/24EI)', function () {
  const r = m.solve(ss(6, [{ kind: 'udl', w: 5, x1: 0, x2: 6 }]));
  printed(r.reactions.Ra, 15, 0, 'R_A'); printed(r.reactions.Rb, 15, 0, 'R_B');
  printed(r.M(3), 22.5, 1, 'M(3)');
  printed(r.y(3), -0.0084375, 7, 'y(3)');
  printed(r.slope(0), -0.0045, 4, 'slope at A'); printed(r.slope(6), 0.0045, 4, 'slope at B');
  printed(r.maxSag.value, 22.5, 1, 'max sag'); printed(r.maxSag.x, 3, 3, 'max sag at V = 0');
  assert.ok(r.criticalX.some(x => Math.abs(x - 3) < 1e-12), 'V = 0 at x = 3 is a critical point');
  printed(r.maxDeflection.value, -0.0084375, 7, 'max deflection'); printed(r.maxDeflection.x, 3, 3, 'max deflection x');
});

group('cantilever, L = 4, P = 10 at 4 (PL, PL³/3EI, PL²/2EI)', function () {
  const r = m.solve(cant(4, [{ kind: 'point', P: 10, x: 4 }]));
  printed(r.reactions.R, 10, 0, 'R'); printed(r.reactions.Mwall, -40, 0, 'M_wall');
  printed(r.M(0), -40, 0, 'M(0⁺)');
  [0.01, 1, 2, 3, 3.99].forEach(x => { printed(r.V(x), 10, 0, 'V(' + x + ')'); printed(r.M(x), -10 * (4 - x), 6, 'M(' + x + ')'); });
  printed(r.y(4), -0.0213333, 7, 'y(4)'); printed(r.slope(4), -0.008, 3, 'slope(4)');
  printed(r.maxHog.value, -40, 0, 'max hog'); printed(r.maxHog.x, 0, 3, 'max hog x');
  assert.strictEqual(r.maxSag, null, 'no sagging');
  printed(r.maxDeflection.value, -0.0213333, 7, 'max deflection'); printed(r.maxDeflection.x, 4, 3, 'max deflection x');
});

group('cantilever, L = 4, full UDL w = 5 (wL²/2, wL⁴/8EI, wL³/6EI)', function () {
  const r = m.solve(cant(4, [{ kind: 'udl', w: 5, x1: 0, x2: 4 }]));
  printed(r.reactions.R, 20, 0, 'R'); printed(r.reactions.Mwall, -40, 0, 'M_wall');
  printed(r.y(4), -0.016, 3, 'y(4)'); printed(r.slope(4), -0.0053333, 7, 'slope(4)');
});

group('cantilever, L = 4, end couple M₀ = 10 (ML²/2EI, ML/EI)', function () {
  const r = m.solve(cant(4, [{ kind: 'moment', M: 10, x: 4 }]));
  printed(r.reactions.R, 0, 9, 'R'); printed(r.reactions.Mwall, -10, 0, 'M_wall');
  [0, 0.01, 1, 2, 3.99].forEach(x => { printed(r.M(x), -10, 0, 'M(' + x + ')'); printed(r.V(x), 0, 9, 'V(' + x + ')'); });
  printed(r.M(4, -1), -10, 0, 'M(4⁻)'); printed(r.M(4, 1), 0, 9, 'M(4⁺)');
  printed(r.y(4), -0.008, 3, 'y(4)'); printed(r.slope(4), -0.004, 3, 'slope(4)');
});

group('ss, L = 6, couple M₀ = 12 at 3', function () {
  const r = m.solve(ss(6, [{ kind: 'moment', M: 12, x: 3 }]));
  printed(r.reactions.Ra, -2, 0, 'R_A'); printed(r.reactions.Rb, 2, 0, 'R_B');
  printed(r.M(3, -1), -6, 0, 'M(3⁻)'); printed(r.M(3, 1), 6, 0, 'M(3⁺)');
  printed(r.V(1), -2, 0, 'V(1)'); printed(r.V(3, -1), -2, 0, 'V(3⁻)'); printed(r.V(3, 1), -2, 0, 'V(3⁺)');
  assert.strictEqual(r.contraflexure.length, 1, 'one contraflexure point');
  printed(r.contraflexure[0].x, 3, 4, 'contraflexure x'); assert.strictEqual(r.contraflexure[0].atCouple, true, 'atCouple');
  printed(r.y(1.5) * 1000, 0.3375, 4, 'y(1.5) mm'); near(r.y(3), 0, 1e-12, 'y(3)'); printed(r.y(4.5) * 1000, -0.3375, 4, 'y(4.5) mm');
  printed(r.slope(0), 0.0003, 4, 'slope at A');
  printed(r.maxSag.value, 6, 0, 'max sag'); printed(r.maxHog.value, -6, 0, 'max hog');
});

group('overhang: ss, L = 8, a = 0, b = 6, w = 4 over 0 … 8', function () {
  const r = m.solve(ss(8, [{ kind: 'udl', w: 4, x1: 0, x2: 8 }], 0, 6));
  printed(r.reactions.Ra, 10.6667, 4, 'R_A'); printed(r.reactions.Rb, 21.3333, 4, 'R_B');
  assert.ok(r.criticalX.some(x => Math.abs(x - 8 / 3) < 1e-12), 'V = 0 at x = 8/3 (exact)');
  printed(r.V(8 / 3), 0, 9, 'V(2.6667)');
  printed(r.M(8 / 3), 14.2222, 4, 'M(2.6667)');
  printed(r.maxSag.value, 14.2222, 4, 'max sag'); printed(r.maxSag.x, 2.6667, 4, 'max sag x');
  printed(r.M(6), -8, 0, 'M(6)');
  printed(r.maxHog.value, -8, 0, 'max hog'); printed(r.maxHog.x, 6, 0, 'max hog x');
  assert.strictEqual(r.contraflexure.length, 1, 'one contraflexure point');
  printed(r.contraflexure[0].x, 5.3333, 4, 'contraflexure x'); assert.strictEqual(r.contraflexure[0].atCouple, false, 'not at a couple');
  printed(r.M(8), 0, 9, 'M(8)');
  printed(r.y(3) * 1000, -4.95, 2, 'y(3) mm'); printed(r.y(8) * 1000, 3.2, 1, 'tip y(8) mm');
});

group('two point loads: ss, L = 6, P = 10 at 2 and 4 (pure bending)', function () {
  const r = m.solve(ss(6, [{ kind: 'point', P: 10, x: 2 }, { kind: 'point', P: 10, x: 4 }]));
  [2.01, 2.5, 3, 3.5, 3.99].forEach(x => { printed(r.V(x), 0, 9, 'V(' + x + ')'); printed(r.M(x), 20, 0, 'M(' + x + ')'); });
  printed(r.V(2, 1), 0, 9, 'V(2⁺)'); printed(r.V(4, -1), 0, 9, 'V(4⁻)');
  assert.strictEqual(r.contraflexure.length, 0, 'no contraflexure');
});

group('sides, defaults and sample()', function () {
  const r = m.solve(ss(6, [{ kind: 'point', P: 20, x: 3 }, { kind: 'moment', M: 6, x: 4 }]));
  assert.strictEqual(r.V(3), r.V(3, 1), 'default side +1 inside');
  assert.strictEqual(r.V(6), r.V(6, -1), 'default side −1 at x = L');
  assert.strictEqual(r.M(6), r.M(6, -1), 'default side −1 at x = L (M)');
  near(r.V(0, -1), 0, 1e-12, 'V(0⁻) = 0'); near(r.M(0, -1), 0, 1e-12, 'M(0⁻) = 0');
  const s = m.sample(r, 100);
  for (let i = 1; i < s.x.length; i++) assert.ok(s.x[i] >= s.x[i - 1], 'x sorted');
  [3, 4].forEach(xc => {
    const idx = s.x.map((x, i) => (Math.abs(x - xc) < 1e-12 ? i : -1)).filter(i => i >= 0);
    assert.ok(idx.length >= 2, 'both sides at ' + xc);
  });
  // R_A = 10 − 6/6 = 9, R_B = 11: V = 9 left of the load, −11 right of it
  const i3 = s.x.indexOf(3);
  near(s.V[i3], 9, 1e-9, 'sample V(3⁻)'); near(s.V[i3 + 1], -11, 1e-9, 'sample V(3⁺)');
  const i4 = s.x.indexOf(4);
  near(s.M[i4 + 1] - s.M[i4], 6, 1e-9, 'sample M jump at the couple');
  assert.strictEqual(s.x[0], 0); assert.strictEqual(s.x[s.x.length - 1], 6);
  assert.ok(s.x.length >= 101, 'at least n + 1 samples');
  [s.V, s.M, s.y].forEach(arr => arr.forEach(v => assert.ok(Number.isFinite(v))));
});

// ---------------------------------------------------------------- invariants on random beams
function q1(v) { return Math.round(v * 10) / 10; }
function randBeam(rnd) {
  const type = rnd() < 0.5 ? 'ss' : 'cantilever';
  const L = 2 + Math.floor(rnd() * 17) * 0.5;
  const grid = rnd() < 0.6;                                    // positions on the 0.1 m grid, or arbitrary
  const pos = (lo, hi) => { const v = lo + rnd() * (hi - lo); return grid ? Math.min(hi, Math.max(lo, q1(v))) : v; };
  let a = 0, b = L;
  if (type === 'ss' && rnd() < 0.7) { a = pos(0, L - 0.5); b = Math.min(L, a + 0.5 + pos(0, L - a - 0.5)); if (b - a < 0.5) b = Math.min(L, a + 0.5); }
  const EIr = Math.pow(10, 3 + 2 * rnd());
  const n = 1 + Math.floor(rnd() * 5);
  const loads = [];
  const special = () => { const r = rnd(); return r < 0.08 ? 0 : r < 0.16 ? L : r < 0.22 ? a : r < 0.28 ? b : pos(0, L); };
  for (let i = 0; i < n; i++) {
    const k = rnd();
    if (k < 0.4) loads.push({ kind: 'point', P: Math.round(-100 + 200 * rnd()), x: special() });
    else if (k < 0.7) {
      let x1 = special(), x2 = special();
      if (x1 > x2) { const t = x1; x1 = x2; x2 = t; }
      if (x2 - x1 < 0.1) { x2 = Math.min(L, x1 + 0.1); x1 = x2 - 0.1; }
      loads.push({ kind: 'udl', w: Math.round(-50 + 100 * rnd()), x1: x1, x2: x2 });
    } else loads.push({ kind: 'moment', M: Math.round(-100 + 200 * rnd()), x: special() });
  }
  return { type: type, L: L, a: a, b: b, EI: EIr, loads: loads };
}
function wAt(beam, x) { let w = 0; beam.loads.forEach(l => { if (l.kind === 'udl' && x > l.x1 && x < l.x2) w += l.w; }); return w; }
function scaleOf(beam) {
  let s = 1;
  beam.loads.forEach(l => { s += l.kind === 'point' ? Math.abs(l.P) : l.kind === 'udl' ? Math.abs(l.w) * (l.x2 - l.x1) : Math.abs(l.M) / beam.L; });
  return s;
}
function coupleAtX(beam, x) { return beam.loads.some(l => l.kind === 'moment' && l.M !== 0 && Math.abs(l.x - x) < 1e-12); }
// random interior points away from every critical point
function freePoints(rnd, beam, r, k, gap) {
  const out = [];
  for (let t = 0; t < 200 && out.length < k; t++) {
    const x = beam.L * rnd();
    if (r.criticalX.every(c => Math.abs(c - x) > gap)) out.push(x);
  }
  return out;
}
const N_RANDOM = 300;
const beams = [];
{ const rnd = rng(20270101); for (let i = 0; i < N_RANDOM; i++) beams.push(randBeam(rnd)); }

group('invariants: closure, zero end moments, equilibrium (' + N_RANDOM + ' random beams)', function () {
  let nEndChecks = 0;
  beams.forEach((beam, i) => {
    const r = m.solve(beam), S = scaleOf(beam), L = beam.L;
    const tol = 1e-9 * S * Math.max(1, L);
    near(r.V(L, 1), 0, tol, '#' + i + ' V(L⁺)'); near(r.M(L, 1), 0, tol, '#' + i + ' M(L⁺)');
    near(r.V(0, -1), 0, tol, '#' + i + ' V(0⁻)'); near(r.M(0, -1), 0, tol, '#' + i + ' M(0⁻)');
    // free ends and simply supported ends carry no moment (unless a couple is applied exactly there)
    if (!coupleAtX(beam, L)) { near(r.M(L, -1), 0, tol, '#' + i + ' M(L⁻)'); nEndChecks++; }
    if (beam.type === 'ss' && !coupleAtX(beam, 0)) { near(r.M(0, 1), 0, tol, '#' + i + ' M(0⁺)'); nEndChecks++; }
    const e = m.equilibrium(beam, r);
    near(e.sumF, 0, tol, '#' + i + ' ΣF'); near(e.sumM, 0, tol, '#' + i + ' ΣM');
  });
  return nEndChecks + ' end-moment checks';
});

group('invariants: boundary conditions to 1e-9', function () {
  beams.forEach((beam, i) => {
    const r = m.solve(beam);
    if (beam.type === 'ss') { near(r.y(beam.a), 0, 1e-9, '#' + i + ' y(a)'); near(r.y(beam.b), 0, 1e-9, '#' + i + ' y(b)'); }
    else { near(r.y(0), 0, 1e-9, '#' + i + ' y(0)'); near(r.slope(0), 0, 1e-9, '#' + i + ' θ(0)'); }
  });
});

group('invariants: dM/dx = V, dV/dx = −w, EI·y″ = M, dy/dx = θ (finite differences)', function () {
  const rnd = rng(77);
  let n = 0;
  beams.forEach((beam, i) => {
    const r = m.solve(beam), L = beam.L, S = scaleOf(beam) * Math.max(1, L);
    const h1 = 1e-5 * L, h2 = 4e-3 * L;
    let ymax = 0; for (let k = 0; k <= 200; k++) ymax = Math.max(ymax, Math.abs(r.y(L * k / 200)));
    freePoints(rnd, beam, r, 6, 3 * h2).forEach(x => {
      near((r.M(x + h1) - r.M(x - h1)) / (2 * h1), r.V(x), 1e-6 * S, '#' + i + ' dM/dx at ' + x);
      near((r.V(x + h1) - r.V(x - h1)) / (2 * h1), -wAt(beam, x), 1e-6 * S, '#' + i + ' dV/dx at ' + x);
      // second difference with Richardson extrapolation (removes the h² term, exact for the quartic pieces of y)
      const d2 = h => (r.y(x + h) - 2 * r.y(x) + r.y(x - h)) / (h * h);
      const ypp = (4 * d2(h2 / 2) - d2(h2)) / 3;
      near(beam.EI * ypp, r.M(x), 1e-6 * S + 1e-12 * beam.EI * ymax / (h2 * h2), '#' + i + ' EI·y″ at ' + x);
      near((r.y(x + h1) - r.y(x - h1)) / (2 * h1), r.slope(x), 1e-7 * Math.max(1e-6, Math.abs(r.slope(x))) + 1e-9 * ymax / h1, '#' + i + ' dy/dx at ' + x);
      n++;
    });
  });
  return n + ' points';
});

group('invariants: superposition', function () {
  const rnd = rng(4242);
  beams.forEach((beam, i) => {
    const A = [], B = [];
    beam.loads.forEach(l => (rnd() < 0.5 ? A : B).push(l));
    const r = m.solve(beam), ra = m.solve(Object.assign({}, beam, { loads: A })), rb = m.solve(Object.assign({}, beam, { loads: B }));
    const S = scaleOf(beam) * Math.max(1, beam.L);
    Object.keys(r.reactions).forEach(k => near(r.reactions[k], ra.reactions[k] + rb.reactions[k], 1e-9 * S, '#' + i + ' reaction ' + k));
    const xs = r.criticalX.concat([0.1, 0.37, 0.5, 0.81].map(f => f * beam.L));
    let ys = 0; xs.forEach(x => { ys = Math.max(ys, Math.abs(r.y(x))); });
    xs.forEach(x => {
      [-1, 1].forEach(s => {
        near(r.V(x, s), ra.V(x, s) + rb.V(x, s), 1e-9 * S, '#' + i + ' V');
        near(r.M(x, s), ra.M(x, s) + rb.M(x, s), 1e-9 * S, '#' + i + ' M');
      });
      near(r.y(x), ra.y(x) + rb.y(x), 1e-9 * Math.max(ys, 1e-6), '#' + i + ' y');
      near(r.slope(x), ra.slope(x) + rb.slope(x), 1e-9 * Math.max(1e-6, ys / beam.L) * 10, '#' + i + ' slope');
    });
  });
});

group('invariants: mirror symmetry of simply supported beams', function () {
  let n = 0;
  beams.filter(b => b.type === 'ss').forEach((beam, i) => {
    const L = beam.L;
    const mir = {
      type: 'ss', L: L, a: L - beam.b, b: L - beam.a, EI: beam.EI,
      loads: beam.loads.map(l => l.kind === 'point' ? { kind: 'point', P: l.P, x: L - l.x }
        : l.kind === 'udl' ? { kind: 'udl', w: l.w, x1: L - l.x2, x2: L - l.x1 } : { kind: 'moment', M: -l.M, x: L - l.x })
    };
    const r = m.solve(beam), q = m.solve(mir), S = scaleOf(beam) * Math.max(1, L);
    const xs = r.criticalX.concat([0.13, 0.42, 0.66, 0.9].map(f => f * L));
    let ys = 0; xs.forEach(x => { ys = Math.max(ys, Math.abs(r.y(x))); });
    xs.forEach(x => {
      [-1, 1].forEach(s => {
        near(q.M(L - x, -s), r.M(x, s), 1e-9 * S, '#' + i + ' M mirrored at ' + x);
        near(q.V(L - x, -s), -r.V(x, s), 1e-9 * S, '#' + i + ' V mirrored at ' + x);
      });
      near(q.y(L - x), r.y(x), 1e-9 * Math.max(ys, 1e-6), '#' + i + ' y mirrored at ' + x);
    });
    n++;
  });
  return n + ' beams';
});

// independent numerical double integration of M/EI (composite Simpson between critical points)
function numericY(beam, r) {
  const L = beam.L, cx = r.criticalX, K = 64;
  function Mside(x, lo, hi) { return x <= lo ? r.M(lo, 1) : x >= hi ? r.M(hi, -1) : r.M(x); }
  // I1(x) = ∫0^x M ds, I2(x) = ∫0^x (x − s) M ds, evaluated piecewise
  function integrals(x) {
    let I1 = 0, I2 = 0;
    for (let i = 0; i + 1 < cx.length; i++) {
      const lo = cx[i], hi = Math.min(cx[i + 1], x);
      if (hi <= lo) break;
      const h = (hi - lo) / K;
      let s1 = 0, s2 = 0;
      for (let k = 0; k <= K; k++) {
        const s = lo + k * h, wgt = (k === 0 || k === K) ? 1 : (k % 2 ? 4 : 2);
        const mv = Mside(s, lo, cx[i + 1] <= x ? cx[i + 1] : Infinity);
        s1 += wgt * mv; s2 += wgt * (x - s) * mv;
      }
      I1 += s1 * h / 3; I2 += s2 * h / 3;
    }
    return { I1: I1, I2: I2 };
  }
  let C1, C2;
  if (beam.type === 'ss') {
    const fa = integrals(beam.a).I2, fb = integrals(beam.b).I2;
    C1 = -(fb - fa) / (beam.b - beam.a); C2 = -fa - C1 * beam.a;
  } else { C1 = 0; C2 = 0; }
  return function (x) { return (C2 + C1 * x + integrals(x).I2) / beam.EI; };
}
group('invariants: deflection = independent numerical double integration of M/EI (relative 1e-5)', function () {
  let worst = 0;
  beams.forEach((beam, i) => {
    const r = m.solve(beam), yn = numericY(beam, r), L = beam.L;
    const xs = [0, 0.11, 0.29, 0.5, 0.73, 0.95, 1].map(f => f * L);
    // reference size: the maximum deflection, or the round-off level S·L³/EI when the loads cancel exactly
    const ref = Math.max(Math.abs(r.maxDeflection.value), 1e-12 * scaleOf(beam) * L * L * L / beam.EI);
    xs.forEach(x => {
      const d = Math.abs(r.y(x) - yn(x)) / ref;
      worst = Math.max(worst, d);
      assert.ok(d <= 1e-5, '#' + i + ' y(' + x + ') = ' + r.y(x) + ' vs numerical ' + yn(x));
    });
  });
  return 'worst relative difference ' + worst.toExponential(2);
});

group('invariants: extremes, maximum deflection and contraflexure against a fine scan', function () {
  beams.forEach((beam, i) => {
    const r = m.solve(beam), L = beam.L, N = 20000, S = scaleOf(beam) * Math.max(1, L);
    let ymax = 0, mmax = -Infinity, mmin = Infinity, vmax = 0;
    for (let k = 0; k <= N; k++) {
      const x = L * k / N;
      ymax = Math.max(ymax, Math.abs(r.y(x)));
      const mv = r.M(x); mmax = Math.max(mmax, mv); mmin = Math.min(mmin, mv);
      vmax = Math.max(vmax, Math.abs(r.V(x)));
    }
    assert.ok(Math.abs(r.maxDeflection.value) >= ymax * (1 - 1e-12) - 1e-15, '#' + i + ' max deflection ' + r.maxDeflection.value + ' < scan ' + ymax);
    near(r.y(r.maxDeflection.x), r.maxDeflection.value, 1e-15, '#' + i + ' maxDeflection consistent');
    assert.ok(r.maxV.value >= vmax * (1 - 1e-12) - 1e-12, '#' + i + ' max |V|');
    if (mmax > 1e-6 * S) assert.ok(r.maxSag && r.maxSag.value >= mmax * (1 - 1e-12), '#' + i + ' max sag');
    if (mmin < -1e-6 * S) assert.ok(r.maxHog && r.maxHog.value <= mmin * (1 - 1e-12), '#' + i + ' max hog');
    if (r.maxSag) near(Math.max(r.M(r.maxSag.x, -1), r.M(r.maxSag.x, 1)), r.maxSag.value, 1e-12 * S, '#' + i + ' maxSag consistent');
    r.contraflexure.forEach(c => {
      assert.ok(c.x > 0 && c.x < L, '#' + i + ' contraflexure inside');
      if (c.atCouple) assert.ok(r.M(c.x, -1) * r.M(c.x, 1) <= 0 && coupleAtX(beam, c.x), '#' + i + ' couple jump at ' + c.x);
      else near(r.M(c.x), 0, 1e-9 * S, '#' + i + ' M = 0 at contraflexure ' + c.x);
      const d = 1e-6 * L;
      assert.ok(r.M(c.x - d) * r.M(c.x + d) < 0, '#' + i + ' M changes sign at ' + c.x);
    });
  });
});

group('equilibrium() detects a wrong reaction', function () {
  const beam = ss(6, [{ kind: 'point', P: 20, x: 3 }]);
  const r = m.solve(beam);
  const bad = Object.assign({}, r, { reactions: { Ra: 11, Rb: 10 } });
  const e = m.equilibrium(beam, bad);
  assert.ok(Math.abs(e.sumF) > 0.5, 'ΣF residual');
});

// ---------------------------------------------------------------- browser regression tests (Playwright; skipped without it
// or with --no-browser): read-outs, the hover read-out and Reset.
async function agroup(name, fn) {
  try { const extra = await fn(); console.log('ok   ' + name + (extra ? '  (' + extra + ')' : '')); }
  catch (e) { failures++; console.log('FAIL ' + name + ': ' + (e && e.message || e)); }
}
function nums(t) { return (t.replace(/\u2212/g, '-').match(/-?\d+(\.\d+)?/g) || []).map(Number); }
function fmt4(v) { return Number(RASim.fmt(v, 4).replace(/\u2212/g, '-')); }
async function browserTests() {
  let playwrightOk = true;
  try { require.resolve('playwright'); } catch (e) { playwrightOk = false; }
  if (!playwrightOk || process.argv.indexOf('--no-browser') >= 0) { console.log('skip browser regression tests'); return; }
  const path = require('path'), os = require('os');
  const { openSim } = require('./harness.js');
  const s = await openSim('beam-sfd-bmd', { width: 1100, outDir: path.join(os.tmpdir(), 'rasim-beam-sfd-bmd-test'), file: FILE_ARG });
  const page = s.page;
  const ID = 'beam-sfd-bmd';
  async function frames() { await page.evaluate(() => new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r)))); }
  async function setS(st) { await page.evaluate(([id, st]) => RASim.instances[id].setState(st), [ID, st]); await frames(); }
  async function getS() { return page.evaluate(id => RASim.instances[id].getState(), ID); }
  async function stat(k) { return page.textContent('[data-stat="' + k + '"] .sim-stat-value'); }
  // hover the BMD canvas at data x (same plot margins as the simulation: left 46, right 16)
  async function hover(x, L) {
    const b = await page.evaluate(() => { const r = document.querySelector('[data-canvas="bmd"]').getBoundingClientRect(); return { l: r.left, t: r.top, w: r.width }; });
    const w = b.w - 2;
    await page.mouse.move(b.l + 1 + 46 + (w - 62) * x / L, b.t + 60);
    await frames();
    return (await page.textContent('[data-key="cursor"]')).replace(/\u00a0/g, ' ');
  }
  try {
    await agroup('browser: read-outs of the check-value states', async function () {
      await setS({ type: 'ss', L: 6, a: 0, b: 6, EI: 10000, loads: [{ kind: 'point', P: 20, x: 3 }] });
      assert.ok(/RA = 10 kN ↑,\s+RB = 10 kN ↑/.test((await stat('reactions')).replace(/\u00a0/g, ' ')), await stat('reactions'));
      assert.ok(/^9 mm ↓/.test(await stat('maxDefl')), await stat('maxDefl'));
      assert.ok(/θA = −0\.0045 rad,\s+θB = 0\.0045 rad/.test((await stat('slopes')).replace(/\u00a0/g, ' ')), await stat('slopes'));
      assert.strictEqual(await stat('check'), 'ΣF = 0, ΣM = 0 ✓');
      await setS({ type: 'cantilever', L: 4, EI: 10000, loads: [{ kind: 'point', P: 10, x: 4 }] });
      assert.ok(/R = 10 kN ↑,\s+Mwall = −40 kN·m/.test((await stat('reactions')).replace(/\u00a0/g, ' ')), await stat('reactions'));
      assert.ok(/^21\.33 mm ↓/.test(await stat('maxDefl')) && /−0\.008 rad/.test(await stat('slopes')));
      await setS({ type: 'ss', L: 6, a: 0, b: 6, EI: 10000, loads: [{ kind: 'moment', M: 12, x: 3 }] });
      assert.ok(/RA = 2 kN ↓,\s+RB = 2 kN ↑/.test((await stat('reactions')).replace(/\u00a0/g, ' ')), await stat('reactions'));
      assert.ok(/x = 3 m \(jump at the couple\)/.test(await stat('contra')), await stat('contra'));
      await setS({ type: 'ss', L: 8, a: 0, b: 6, EI: 10000, loads: [{ kind: 'udl', w: 4, x1: 0, x2: 8 }] });
      assert.ok(/^14\.22 kN·m\s*at x = 2\.667 m/.test(await stat('maxSag')), await stat('maxSag'));
      assert.ok(/^−8 kN·m\s*at x = 6 m/.test(await stat('maxHog')), await stat('maxHog'));
      assert.strictEqual(await stat('contra'), 'x = 5.333 m');
    });
    await agroup('browser: the hover read-out gives the values at the x it prints, both sides at a jump (regression)', async function () {
      const beam = { type: 'cantilever', L: 7.5, a: 0, b: 7.5, EI: 47000, loads: [{ kind: 'point', P: -30, x: 2 }, { kind: 'moment', M: 55, x: 5 }, { kind: 'udl', w: 8, x1: 3, x2: 7.5 }] };
      await setS(beam);
      const r = m.solve(beam);
      let n = 0;
      for (const xd of [0.953, 1.27, 3.751, 6.2317]) {
        const t = await hover(xd, beam.L), parts = t.split(/V =|M =|y =/);
        const x = nums(parts[0])[0];
        assert.ok(Math.abs(x * 100 - Math.round(x * 100)) < 1e-9, 'x printed with 2 decimals: ' + t);
        assert.strictEqual(nums(parts[1])[0], fmt4(r.V(x)), 'V at the printed x: ' + t);
        assert.strictEqual(nums(parts[2])[0], fmt4(r.M(x)), 'M at the printed x: ' + t);
        assert.strictEqual(nums(parts[3])[0], fmt4(r.y(x) * 1000), 'y at the printed x: ' + t);
        n++;
      }
      // exactly at the point load of the default beam: V = 10 → −10
      await setS({ type: 'ss', L: 6, a: 0, b: 6, EI: 10000, loads: [{ kind: 'point', P: 20, x: 3 }] });
      const t = await hover(3, 6);
      assert.ok(/x = 3\.00 m:\s+V = 10 → −10 kN,\s+M = 30 kN·m,\s+y = −9 mm/.test(t), t);
      await setS({ type: 'ss', L: 6, a: 0, b: 6, EI: 10000, loads: [{ kind: 'moment', M: 12, x: 3 }] });
      const t2 = await hover(3, 6);
      assert.ok(/V = −2 kN,\s+M = −6 → 6 kN·m/.test(t2), t2);
      await page.mouse.move(2, 2);
      return n + 2 + ' positions';
    });
    await agroup('browser: Reset restores every default from a dirty state', async function () {
      const init = { type: 'ss', L: 6, a: 0, b: 6, EI: 10000, showDeflection: true, loads: [{ kind: 'point', P: 20, x: 3 }] };
      await setS({ type: 'cantilever', L: 9, EI: 2000, showDeflection: false, loads: [{ kind: 'udl', w: -7, x1: 1, x2: 2 }] });
      await page.click('[data-control="reset"]'); await frames();
      assert.deepStrictEqual(await getS(), init);
      assert.strictEqual(await page.$eval('[data-control="preset"]', e => e.value), 'ssPoint');
    });
    assert.deepStrictEqual(s.errors, [], 'console errors'); assert.deepStrictEqual(s.warnings, [], 'console warnings');
  } catch (e) { failures++; console.log('FAIL browser: ' + (e && e.message || e)); }
  finally { await s.browser.close(); }
}

browserTests().then(function () {
  if (failures) { console.log(failures + ' group(s) FAILED'); process.exit(1); }
  console.log('all tests passed');
});
