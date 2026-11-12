// beam-sfd-bmd — "Beam: shear force, bending moment and deflection"
// Sits beside notes/src/05_mechanical.html, sections "Shear force and bending moment diagrams" and
// "Deflection of beams" (B2.1 Mechanics of Materials).
//
// SIGN CONVENTIONS
//   x from the left end, 0 ≤ x ≤ L. Units: kN, m, kN·m; EI in kN·m²; the model returns deflections in metres,
//   the UI shows them in mm and slopes in rad.
//   Loads: point load P (kN) positive DOWNWARD; distributed load w (kN/m) positive DOWNWARD over [x1, x2];
//   applied couple M₀ (kN·m) positive CLOCKWISE.
//   Shear force V(x) = sum of the UPWARD forces to the left of the section. Bending moment M(x) positive when
//   SAGGING, from everything to the left of the section (= clockwise moment of the left-hand forces about the
//   section). Hence dM/dx = V, dV/dx = −w; a downward P makes V drop by P; a clockwise M₀ makes M jump up by M₀.
//   Deflection y positive UPWARD; slope θ = dy/dx; EI·y″ = M.
//   Beam types: 'ss' = pin at x = a, roller at x = b (overhangs when a > 0 or b < L);
//               'cantilever' = fixed at x = 0, free at x = L (a and b ignored).
//   Reactions (upward +): ss R_b = [Σ P_i(x_i − a) + Σ W_j(x̄_j − a) + Σ M₀_k]/(b − a), R_a = Σ P + Σ W − R_b;
//   cantilever R = Σ P + Σ W, M_wall = M(0⁺) = −[Σ P_i x_i + Σ W_j x̄_j + Σ M₀_k] (also the clockwise couple that
//   the wall applies to the beam).
//   Macaulay: M(x) = Σ c·⟨x − x₀⟩ⁿ (n = 0 couples / wall moment, 1 forces, 2 UDL ends); V = dM/dx;
//   EI·θ = C₁ + Σ c⟨u⟩ⁿ⁺¹/(n + 1); EI·y = C₂ + C₁x + Σ c⟨u⟩ⁿ⁺²/((n + 1)(n + 2)) — exact.
//   BCs: ss y(a) = y(b) = 0; cantilever y(0) = θ(0) = 0.
//   Discontinuities: V(x, side), M(x, side) with side −1 = left limit, +1 = right limit; default +1, except at
//   x = L where it is −1. maxV.value is the largest |V| (≥ 0).
//
// STATE KEYS (getState / setState): type ('ss' | 'cantilever'), L, a, b, EI,
//   loads ([{ kind: 'point', P, x } | { kind: 'udl', w, x1, x2 } | { kind: 'moment', M, x }], at most 5),
//   showDeflection (boolean)
// STAT KEYS:   reactions, maxV, maxSag, maxHog, contra, maxDefl, slopes, check
// TAG KEYS:    (none)
// MATRIX KEYS: (none)
// DATA-KEY:    cursor (hover read-out line), conventions (sign-convention note)
// CANVAS KEYS: beam, sfd, bmd, defl
// CONTROL KEYS: preset, type, L, EI, a, b,
//   loadNtype, loadNmag, loadNx, loadNx2, loadNremove (N = 1 … 5),
//   addPoint, addUDL, addCouple, showDeflection, reset
(function () {
  'use strict';
  const RASim = globalThis.RASim;
  const M = RASim.math;
  const ID = 'beam-sfd-bmd';
  const SC = '.sim[data-sim="' + ID + '"]';

  // ================================================================ model (pure)
  function numOr(v, d) { v = Number(v); return isFinite(v) ? v : d; }

  function normalize(beam) {
    beam = beam || {};
    const type = beam.type === 'cantilever' ? 'cantilever' : 'ss';
    const L = numOr(beam.L, 6);
    const a = type === 'ss' ? numOr(beam.a, 0) : 0;
    const b = type === 'ss' ? numOr(beam.b, L) : L;
    const EI = numOr(beam.EI, 10000);
    const loads = [];
    (beam.loads || []).forEach(function (ld) {
      if (!ld) return;
      if (ld.kind === 'point') loads.push({ kind: 'point', P: numOr(ld.P, 0), x: numOr(ld.x, 0) });
      else if (ld.kind === 'udl') {
        const p = numOr(ld.x1, 0), q = numOr(ld.x2, 0);
        loads.push({ kind: 'udl', w: numOr(ld.w, 0), x1: Math.min(p, q), x2: Math.max(p, q) });
      } else if (ld.kind === 'moment' || ld.kind === 'couple') loads.push({ kind: 'moment', M: numOr(ld.M, 0), x: numOr(ld.x, 0) });
    });
    return { type: type, L: L, a: a, b: b, EI: EI, loads: loads };
  }

  // load totals: downward force, its moment about x = a (clockwise +) and about x = 0, and a size scale
  function totals(nb) {
    const a = nb.a;
    let F = 0, momA = 0, mom0 = 0, scale = 0;
    nb.loads.forEach(function (ld) {
      if (ld.kind === 'point') { F += ld.P; momA += ld.P * (ld.x - a); mom0 += ld.P * ld.x; scale += Math.abs(ld.P); }
      else if (ld.kind === 'udl') {
        const W = ld.w * (ld.x2 - ld.x1), xb = (ld.x1 + ld.x2) / 2;
        F += W; momA += W * (xb - a); mom0 += W * xb; scale += Math.abs(W);
      } else { momA += ld.M; mom0 += ld.M; scale += Math.abs(ld.M) / Math.max(1e-9, nb.L); }
    });
    return { F: F, momA: momA, mom0: mom0, scale: scale };
  }

  function solve(beam) {
    const nb = normalize(beam);
    const L = nb.L, EI = nb.EI, ss = nb.type === 'ss', a = nb.a, b = nb.b;
    const epsX = 1e-12 * Math.max(1, L);          // positions closer than this are the same point
    const tc = [], tx = [], tn = [];              // Macaulay terms of M: c·<x − x0>^n
    function term(c, x0, n) { if (c !== 0) { tc.push(c); tx.push(x0); tn.push(n); } }
    nb.loads.forEach(function (ld) {
      if (ld.kind === 'point') term(-ld.P, ld.x, 1);
      else if (ld.kind === 'udl') { term(-ld.w / 2, ld.x1, 2); term(ld.w / 2, ld.x2, 2); }
      else term(ld.M, ld.x, 0);
    });
    const T = totals(nb);
    let reactions, Rscale;
    if (ss) {
      const Rb = T.momA / (b - a), Ra = T.F - Rb;
      term(Ra, a, 1); term(Rb, b, 1);
      reactions = { Ra: Ra, Rb: Rb };
      Rscale = Math.abs(Ra) + Math.abs(Rb);
    } else {
      const R = T.F, Mwall = -T.mom0;
      term(Mwall, 0, 0); term(R, 0, 1);
      reactions = { R: R, Mwall: Mwall };
      Rscale = Math.abs(R) + Math.abs(Mwall) / Math.max(1e-9, L);
    }
    const nT = tc.length;
    const scaleF = Math.max(1, T.scale + Rscale);
    const tolV = 1e-10 * scaleF, tolM = 1e-10 * scaleF * Math.max(1, L);

    function sideOf(x, side) {
      if (side == null || !isFinite(side) || side === 0) return x >= L - epsX ? -1 : 1;
      return side < 0 ? -1 : 1;
    }
    function step(u, s) { return u > epsX ? 1 : u < -epsX ? 0 : (s > 0 ? 1 : 0); }
    function V(x, side) {
      const s = sideOf(x, side);
      let v = 0;
      for (let i = 0; i < nT; i++) {
        const n = tn[i], u = x - tx[i];
        if (n === 1) v += tc[i] * step(u, s);
        else if (n === 2 && u > 0) v += 2 * tc[i] * u;
      }
      return v;
    }
    function Mf(x, side) {
      const s = sideOf(x, side);
      let m = 0;
      for (let i = 0; i < nT; i++) {
        const n = tn[i], u = x - tx[i];
        if (n === 0) m += tc[i] * step(u, s);
        else if (u > 0) m += n === 1 ? tc[i] * u : tc[i] * u * u;
      }
      return m;
    }
    // first and second integrals of the Macaulay sum (continuous, no side needed)
    function G(x) {
      let g = 0;
      for (let i = 0; i < nT; i++) {
        const u = x - tx[i];
        if (u > 0) { const n = tn[i]; g += n === 0 ? tc[i] * u : n === 1 ? tc[i] * u * u / 2 : tc[i] * u * u * u / 3; }
      }
      return g;
    }
    function F(x) {
      let f = 0;
      for (let i = 0; i < nT; i++) {
        const u = x - tx[i];
        if (u > 0) { const n = tn[i]; f += n === 0 ? tc[i] * u * u / 2 : n === 1 ? tc[i] * u * u * u / 6 : tc[i] * u * u * u * u / 12; }
      }
      return f;
    }
    let C1, C2;
    if (ss) { C1 = -(F(b) - F(a)) / (b - a); C2 = -F(a) - C1 * a; }
    else { C1 = -G(0); C2 = -F(0); }
    function slope(x) { return (C1 + G(x)) / EI; }
    function y(x) { return (C2 + C1 * x + F(x)) / EI; }

    // ---- critical points: ends, every load position / UDL end, supports, interior zeros of V ----
    const epsC = 1e-9 * Math.max(1, L);
    function uniq(arr) {
      arr.sort(function (p, q) { return p - q; });
      const out = [];
      arr.forEach(function (v) { if (!out.length || v - out[out.length - 1] > epsC) out.push(v); });
      return out;
    }
    const raw = [0, L];
    nb.loads.forEach(function (ld) { if (ld.kind === 'udl') raw.push(ld.x1, ld.x2); else raw.push(ld.x); });
    if (ss) raw.push(a, b);
    const brk = uniq(raw.map(function (v) { return M.clamp(v, 0, L); }));
    const zerosV = [];
    for (let i = 0; i + 1 < brk.length; i++) {
      const p = brk[i], q = brk[i + 1];
      const va = V(p, 1), vb = V(q, -1);                            // V is linear on (p, q)
      if ((va > tolV && vb < -tolV) || (va < -tolV && vb > tolV)) zerosV.push(p + (q - p) * va / (va - vb));
    }
    const criticalX = uniq(brk.concat(zerosV));

    // values at both sides of every critical point (only the inside of the beam at the two ends)
    const ent = [];
    criticalX.forEach(function (x) {
      if (x <= epsC) ent.push({ x: x, s: 1 });
      else if (x >= L - epsC) ent.push({ x: x, s: -1 });
      else { ent.push({ x: x, s: -1 }); ent.push({ x: x, s: 1 }); }
    });
    ent.forEach(function (e) { e.V = V(e.x, e.s); e.M = Mf(e.x, e.s); });

    let maxV = { value: 0, x: 0 };
    ent.forEach(function (e) { if (Math.abs(e.V) > maxV.value * (1 + 1e-12) + 1e-15) maxV = { value: Math.abs(e.V), x: e.x }; });
    let sag = null, hog = null;
    ent.forEach(function (e) {
      if (e.M > tolM && (!sag || e.M > sag.value * (1 + 1e-12))) sag = { value: e.M, x: e.x };
      if (e.M < -tolM && (!hog || e.M < hog.value * (1 + 1e-12))) hog = { value: e.M, x: e.x };
    });

    // ---- contraflexure: sign changes of M between consecutive non-zero values ----
    function coupleAt(x) { return nb.loads.some(function (ld) { return ld.kind === 'moment' && ld.M !== 0 && Math.abs(ld.x - x) <= epsC; }); }
    const contra = [];
    let last = null, zeros = [];
    ent.forEach(function (e) {
      if (Math.abs(e.M) <= tolM) { zeros.push(e); return; }
      if (last && (e.M > 0) !== (last.M > 0)) {
        let xc, atC;
        if (zeros.length) {
          const z0 = zeros[0].x, z1 = zeros[zeros.length - 1].x;
          xc = (z0 + z1) / 2;
          atC = coupleAt(xc) && Math.abs(Mf(xc, 1) - Mf(xc, -1)) > tolM;
        } else if (Math.abs(e.x - last.x) <= epsC) { xc = e.x; atC = true; }           // jump through a couple
        else {                                                                          // M continuous, monotonic on (last.x, e.x)
          let lo = last.x, hi = e.x;
          const sLo = last.M > 0;
          for (let k = 0; k < 200 && hi - lo > 1e-15 * Math.max(1, L); k++) {
            const mid = (lo + hi) / 2;
            if (mid <= lo || mid >= hi) break;
            const mm = Mf(mid);
            if (mm === 0) { lo = hi = mid; break; }
            if ((mm > 0) === sLo) lo = mid; else hi = mid;
          }
          xc = (lo + hi) / 2; atC = false;
        }
        if (xc > epsC && xc < L - epsC) contra.push({ x: xc, atCouple: atC });
      }
      last = e; zeros = [];
    });

    // ---- maximum |deflection|: 2000-interval grid + refinement where the slope changes sign + the ends ----
    const N = 2000;
    let best = { value: y(0), x: 0 };
    let sPrev = slope(0);
    function consider(x) { const v = y(x); if (Math.abs(v) > Math.abs(best.value) * (1 + 1e-12) + 1e-18) best = { value: v, x: x }; }
    for (let i = 1; i <= N; i++) {
      const xi = L * i / N, si = slope(xi);
      if ((sPrev > 0 && si < 0) || (sPrev < 0 && si > 0)) {
        let lo = L * (i - 1) / N, hi = xi;
        const sLo = sPrev > 0;
        for (let k = 0; k < 100 && hi - lo > 1e-15 * Math.max(1, L); k++) {
          const mid = (lo + hi) / 2;
          if (mid <= lo || mid >= hi) break;
          if ((slope(mid) > 0) === sLo) lo = mid; else hi = mid;
        }
        consider((lo + hi) / 2);
      }
      consider(xi);
      sPrev = si;
    }

    return {
      reactions: reactions,
      V: V, M: Mf, slope: slope, y: y,
      criticalX: criticalX,
      maxV: maxV, maxSag: sag, maxHog: hog,
      contraflexure: contra,
      maxDeflection: best
    };
  }

  // arrays for plotting; both sides of every critical point so that steps are drawn vertical
  function sample(result, n) {
    n = Math.max(1, Math.round(numOr(n, 400)));
    const cx = result.criticalX, L = cx[cx.length - 1], eps = 1e-9 * Math.max(1, L);
    const xs = [], sides = [];
    function crit(x) {
      if (x <= eps) { xs.push(x); sides.push(1); }
      else if (x >= L - eps) { xs.push(x); sides.push(-1); }
      else { xs.push(x, x); sides.push(-1, 1); }
    }
    let j = 0;
    for (let i = 0; i <= n; i++) {
      const xg = L * i / n;
      while (j < cx.length && cx[j] < xg - eps) crit(cx[j++]);
      if (j < cx.length && Math.abs(cx[j] - xg) <= eps) crit(cx[j++]);
      else { xs.push(xg); sides.push(i === n ? -1 : 1); }
    }
    while (j < cx.length) crit(cx[j++]);
    const out = { x: xs, V: [], M: [], y: [] };
    for (let i = 0; i < xs.length; i++) {
      out.V.push(result.V(xs[i], sides[i]));
      out.M.push(result.M(xs[i], sides[i]));
      out.y.push(result.y(xs[i]));
    }
    return out;
  }

  // statics residuals of the whole beam: Σ vertical forces (up +) and Σ moments about x = 0 (clockwise +)
  function equilibrium(beam, result) {
    const nb = normalize(beam);
    const T = totals(nb), r = result.reactions;
    if (nb.type === 'ss') {
      return { sumF: r.Ra + r.Rb - T.F, sumM: T.mom0 - r.Ra * nb.a - r.Rb * nb.b };
    }
    return { sumF: r.R - T.F, sumM: r.Mwall + T.mom0 };
  }

  const model = { solve: solve, sample: sample, equilibrium: equilibrium };

  // ================================================================ presets and defaults
  const PRESETS = [
    { value: 'ssPoint', label: 'Simply supported, central point load', s: { type: 'ss', L: 6, loads: [{ kind: 'point', P: 20, x: 3 }] } },
    { value: 'ssUdl', label: 'Simply supported, full UDL', s: { type: 'ss', L: 6, loads: [{ kind: 'udl', w: 5, x1: 0, x2: 6 }] } },
    { value: 'cPoint', label: 'Cantilever, end point load', s: { type: 'cantilever', L: 4, loads: [{ kind: 'point', P: 10, x: 4 }] } },
    { value: 'cUdl', label: 'Cantilever, full UDL', s: { type: 'cantilever', L: 4, loads: [{ kind: 'udl', w: 5, x1: 0, x2: 4 }] } },
    { value: 'cCouple', label: 'Cantilever, end couple', s: { type: 'cantilever', L: 4, loads: [{ kind: 'moment', M: 10, x: 4 }] } },
    { value: 'overhang', label: 'Overhang with contraflexure', s: { type: 'ss', L: 8, a: 0, b: 6, loads: [{ kind: 'udl', w: 4, x1: 0, x2: 8 }] } },
    { value: 'coupleMid', label: 'Couple at mid-span', s: { type: 'ss', L: 6, loads: [{ kind: 'moment', M: 12, x: 3 }] } },
    { value: 'twoPoint', label: 'Two point loads — pure bending between them', s: { type: 'ss', L: 6, loads: [{ kind: 'point', P: 10, x: 2 }, { kind: 'point', P: 10, x: 4 }] } }
  ];
  function presetState(p) {
    const s = p.s;
    return { type: s.type, L: s.L, a: s.a != null ? s.a : 0, b: s.b != null ? s.b : s.L, EI: 10000, loads: s.loads.map(function (l) { return Object.assign({}, l); }) };
  }
  const DEFAULTS = Object.assign(presetState(PRESETS[0]), { showDeflection: true });
  const MAXLOADS = 5;
  const MAG = { point: { min: -100, max: 100, field: 'P' }, udl: { min: -50, max: 50, field: 'w' }, moment: { min: -100, max: 100, field: 'M' } };

  RASim.register(ID, {
    title: 'Beam: shear force, bending moment and deflection',
    model: model,
    mount: function (api) {
      const ui = RASim.ui, C = RASim.colors, D = RASim.draw;
      const fmt = RASim.fmt, fmtFixed = RASim.fmtFixed;
      const alpha = RASim.alpha;
      const st = cloneState(DEFAULTS);

      RASim.css(ID,
        SC + ' .bm-nt { text-transform: none; letter-spacing: 0; font-size: 11.5px; }\n' +
        SC + ' .bm-case { text-transform: none; letter-spacing: 0; }\n' +
        SC + ' .bm-wide { grid-column: 1 / -1; }\n' +
        SC + ' .bm-wide .sim-stat-value { font-size: 14px; }\n' +
        SC + ' .bm-at { display: block; font-size: 11.5px; font-weight: normal; color: ' + C.muted + '; }\n' +
        SC + ' .bm-stack { display: flex; flex-direction: column; gap: 4px; min-width: 0; }\n' +
        SC + ' .bm-readout { font-size: 12.5px; margin: 0; color: ' + C.ink + '; min-height: 1.5em; font-variant-numeric: tabular-nums; }\n' +
        SC + ' .bm-load { border: 1px solid ' + C.grid + '; border-radius: 4px; background: #ffffff; padding: 4px 8px 6px; display: flex; flex-direction: column; gap: 2px; min-width: 0; }\n' +
        SC + ' .bm-load-head { display: flex; flex-wrap: wrap; align-items: center; gap: 4px 10px; min-width: 0; }\n' +
        SC + ' .bm-load-title { font-size: 12.5px; font-weight: bold; color: ' + C.maroon + '; }\n' +
        SC + ' .bm-load-head .sim-btn { margin-left: auto; }\n' +
        SC + ' .bm-btns .sim-btn { flex: 1 1 auto; }\n' +
        SC + ' .bm-preset { display: flex; width: 100%; }\n' +
        SC + ' .bm-preset select { flex: 1 1 auto; min-width: 0; width: 100%; }\n');
      function lab(html) { return '<span class="bm-nt">' + html + '</span>'; }

      ui.note(api.root, 'Pick a preset or add loads, drag loads and supports along the beam, and watch the reactions, the shear-force and bending-moment diagrams and the deflected shape change together.');
      const sp = ui.split(api.root, { ratio: 1.7 });

      // ---- main column ----
      const stack = ui.el('div', { 'class': 'bm-stack' }, sp.main);
      const H_BEAM = 214, H_SFD = 150, H_BMD = 166, H_DEF = 150;
      const cvB = RASim.canvas(stack, { aspect: 3, minHeight: H_BEAM, maxHeight: H_BEAM, key: 'beam' });
      const cvV = RASim.canvas(stack, { aspect: 3, minHeight: H_SFD, maxHeight: H_SFD, key: 'sfd' });
      const cvM = RASim.canvas(stack, { aspect: 3, minHeight: H_BMD, maxHeight: H_BMD, key: 'bmd' });
      const cvY = RASim.canvas(stack, { aspect: 3, minHeight: H_DEF, maxHeight: H_DEF, key: 'defl' });
      const readout = ui.el('p', { 'class': 'bm-readout' }, sp.main);
      readout.setAttribute('data-key', 'cursor');
      ui.legend(sp.main, [
        { color: C.red, label: 'point load P' },
        { color: C.orange, label: 'UDL w' },
        { color: C.purple, label: 'couple M₀' },
        { color: C.green, label: 'reactions' },
        { color: C.teal, label: 'deflected shape / y(x)', dash: true },
        { color: C.blue, label: 'shear force V' },
        { color: C.maroon, label: 'sagging M (above the axis)' },
        { color: C.gold, label: 'hogging M (below)' }
      ]);
      const conv = ui.note(sp.main, '<b>Conventions.</b> x from the left end; kN, m, kN·m; EI in kN·m²; deflection in mm. ' +
        'P and w positive <b>downward</b>, M₀ positive <b>clockwise</b>. V = sum of the upward forces to the left of the section; ' +
        'M positive when <b>sagging</b> (compression on top), from everything to the left: dM/dx = V and dV/dx = −w, so a downward P makes V drop by P ' +
        'and a clockwise M₀ makes M jump up by M₀. y positive upward, θ = dy/dx, EI·y″ = M.');
      conv.setAttribute('data-key', 'conventions');

      // ---- side column: controls ----
      const gP = ui.group(sp.side, 'Preset');
      const selP = ui.select(gP, {
        label: '', key: 'preset',
        options: PRESETS.map(function (p) { return { value: p.value, label: p.label }; }).concat([{ value: 'custom', label: 'Custom (edited)' }]),
        value: PRESETS[0].value,
        onChange: function (v) {
          const p = PRESETS.filter(function (q) { return q.value === v; })[0];
          if (p) setState(presetState(p)); else update();
        }
      });
      selP.el.classList.add('bm-preset');

      const gB = ui.group(sp.side, 'Beam');
      const segT = ui.segmented(gB, { label: '', key: 'type', value: st.type,
        options: [{ value: 'ss', label: 'Simply supported' }, { value: 'cantilever', label: 'Cantilever' }],
        onChange: function (v) { st.type = v; syncControls(); update(); } });
      const sL = ui.slider(gB, { label: 'Length L', min: 2, max: 10, step: 0.5, value: st.L, unit: 'm', key: 'L',
        onInput: function (v) { applyL(v); syncControls(); update(); } });
      const sEI = ui.slider(gB, { label: 'Flexural rigidity EI', min: 1000, max: 100000, scale: 'log', value: st.EI, unit: 'kN·m²', key: 'EI',
        onInput: function (v) { st.EI = v; update(); } });

      const gS = ui.group(sp.side, 'Supports');
      const gSbox = gS.parentNode;
      const sA = ui.slider(gS, { label: 'a: pin A at x =', min: 0, max: st.L, step: 0.1, value: st.a, unit: 'm', key: 'a',
        onInput: function (v) { moveSupport('a', v); syncControls(); update(); } });
      const sB = ui.slider(gS, { label: 'b: roller B at x =', min: 0, max: st.L, step: 0.1, value: st.b, unit: 'm', key: 'b',
        onInput: function (v) { moveSupport('b', v); syncControls(); update(); } });

      const gL = ui.group(sp.side, 'Loads (at most 5)');
      const rows = [];
      for (let i = 0; i < MAXLOADS; i++) rows.push(buildRow(i));
      const rowAdd = ui.row(gL); rowAdd.classList.add('bm-btns');
      const bAddP = ui.button(rowAdd, { label: '+ Point load', key: 'addPoint', onClick: function () { addLoad({ kind: 'point', P: 10, x: st.L / 2 }); } });
      const bAddU = ui.button(rowAdd, { label: '+ UDL', key: 'addUDL', onClick: function () { addLoad({ kind: 'udl', w: 5, x1: 0, x2: st.L }); } });
      const bAddC = ui.button(rowAdd, { label: '+ Couple', key: 'addCouple', onClick: function () { addLoad({ kind: 'moment', M: 10, x: st.L / 2 }); } });

      const gD = ui.group(sp.side, 'Display');
      const rowD = ui.row(gD);
      const chkD = ui.checkbox(rowD, { label: 'Deflection diagram', checked: st.showDeflection, key: 'showDeflection',
        onChange: function (v) { st.showDeflection = v; update(); } });
      ui.button(rowD, { label: 'Reset', key: 'reset', title: 'Restore every default', onClick: function () { hoverX = null; setState(cloneState(DEFAULTS)); } });

      const stats = ui.stats(sp.side);
      const tR = ui.stat(stats, { label: lab('reactions (upward +)'), key: 'reactions' });
      tR.el.classList.add('bm-wide');
      const tV = ui.stat(stats, { label: lab('max |V|'), key: 'maxV' });
      const tDf = ui.stat(stats, { label: lab('max deflection |y|'), key: 'maxDefl' });
      const tSag = ui.stat(stats, { label: lab('max sagging M'), key: 'maxSag' });
      const tHog = ui.stat(stats, { label: lab('max hogging M'), key: 'maxHog' });
      const tCf = ui.stat(stats, { label: lab('contraflexure (M changes sign)'), key: 'contra' });
      tCf.el.classList.add('bm-wide');
      const tSl = ui.stat(stats, { label: lab('slopes θ = dy/dx'), key: 'slopes' });
      tSl.el.classList.add('bm-wide');
      const tCk = ui.stat(stats, { label: lab('equilibrium of the whole beam'), key: 'check' });
      tCk.el.classList.add('bm-wide');

      ui.hints(api.root, [
        'The bending moment is largest where the shear force crosses zero — check it with the full UDL and with the overhang preset.',
        'A point load makes a step in the SFD and a kink in the BMD; a UDL gives a sloping SFD and a parabolic BMD; a couple makes a jump in the BMD and nothing in the SFD.',
        'Overhang preset: slide support B inward — the overhang produces a hogging moment over the support and a point of contraflexure where M = 0.',
        'Compare with the table in the notes: central load PL/4 and PL³/48EI; full UDL wL²/8 and 5wL⁴/384EI; cantilever end load PL and PL³/3EI.',
        'Two equal point loads: between them V = 0 and M is constant — pure bending.'
      ]);

      // ================================================================ state helpers
      function cloneState(s) {
        return { type: s.type, L: s.L, a: s.a, b: s.b, EI: s.EI, showDeflection: s.showDeflection,
          loads: s.loads.map(function (l) { return Object.assign({}, l); }) };
      }
      function r9(v) { return Math.round(v * 1e9) / 1e9; }
      function snap(v) { return Math.round(v * 10) / 10; }
      function beamOf() { return { type: st.type, L: st.L, a: st.a, b: st.b, EI: st.EI, loads: st.loads }; }

      // supports: keep 0 ≤ a, a + 0.5 ≤ b ≤ L by pushing the other one
      function moveSupport(which, v) {
        const L = st.L;
        if (which === 'a') {
          st.a = r9(M.clamp(v, 0, L));
          if (st.b - st.a < 0.5 - 1e-9) { st.b = r9(st.a + 0.5); if (st.b > L) { st.b = L; st.a = r9(L - 0.5); } }
        } else {
          st.b = r9(M.clamp(v, 0, L));
          if (st.b - st.a < 0.5 - 1e-9) { st.a = r9(st.b - 0.5); if (st.a < 0) { st.a = 0; st.b = 0.5; } }
        }
      }
      function fixSupports() {
        st.a = r9(M.clamp(st.a, 0, st.L)); st.b = r9(M.clamp(st.b, 0, st.L));
        if (st.b - st.a < 0.5 - 1e-9) { st.b = r9(Math.min(st.L, st.a + 0.5)); st.a = r9(Math.min(st.a, st.b - 0.5)); }
      }
      function clampLoad(ld) {
        const L = st.L, mg = MAG[ld.kind];
        ld[mg.field] = M.clamp(numOr(ld[mg.field], 0), mg.min, mg.max);
        if (ld.kind === 'udl') {
          let p = M.clamp(numOr(ld.x1, 0), 0, L), q = M.clamp(numOr(ld.x2, L), 0, L);
          if (p > q) { const t = p; p = q; q = t; }
          if (q - p < 0.1 - 1e-9) { q = r9(p + 0.1); if (q > L) { q = L; p = r9(L - 0.1); } }
          ld.x1 = p; ld.x2 = q;
        } else ld.x = M.clamp(numOr(ld.x, 0), 0, L);
        return ld;
      }
      // a support that was at the end stays at the end; everything else is clamped into [0, L]
      function applyL(v) {
        const L0 = st.L;
        const aEnd = Math.abs(st.a - L0) < 1e-9, bEnd = Math.abs(st.b - L0) < 1e-9;
        st.L = v;
        if (aEnd) st.a = v;
        if (bEnd) st.b = v;
        fixSupports();
        st.loads.forEach(clampLoad);
      }
      function moveUdlEnd(ld, end, v) {
        const L = st.L;
        v = r9(M.clamp(v, 0, L));
        if (end === 'x1') {
          ld.x1 = v;
          if (ld.x2 - ld.x1 < 0.1 - 1e-9) { ld.x2 = r9(ld.x1 + 0.1); if (ld.x2 > L) { ld.x2 = L; ld.x1 = r9(L - 0.1); } }
        } else {
          ld.x2 = v;
          if (ld.x2 - ld.x1 < 0.1 - 1e-9) { ld.x1 = r9(ld.x2 - 0.1); if (ld.x1 < 0) { ld.x1 = 0; ld.x2 = 0.1; } }
        }
      }
      function convertLoad(ld, kind) {
        if (ld.kind === kind) return ld;
        const mag = ld[MAG[ld.kind].field];
        const xc = ld.kind === 'udl' ? (ld.x1 + ld.x2) / 2 : ld.x;
        const out = { kind: kind };
        out[MAG[kind].field] = M.clamp(mag, MAG[kind].min, MAG[kind].max);
        if (kind === 'udl') { out.x1 = r9(Math.max(0, xc - 1)); out.x2 = r9(Math.min(st.L, xc + 1)); }
        else out.x = r9(xc);
        return clampLoad(out);
      }
      function addLoad(ld) {
        if (st.loads.length >= MAXLOADS) return;
        st.loads.push(clampLoad(ld));
        syncControls(); update();
      }

      // ---- load rows (built once; shown / hidden and relabelled by kind) ----
      function buildRow(i) {
        const k = 'load' + (i + 1);
        const box = ui.el('div', { 'class': 'bm-load' }, gL);
        const head = ui.el('div', { 'class': 'bm-load-head' }, box);
        ui.el('span', { 'class': 'bm-load-title', text: 'Load ' + (i + 1) }, head);
        const sel = ui.select(head, { label: '', key: k + 'type', value: 'point',
          options: [{ value: 'point', label: 'Point' }, { value: 'udl', label: 'UDL' }, { value: 'moment', label: 'Couple' }],
          onChange: function (v) { st.loads[i] = convertLoad(st.loads[i], v); syncControls(); update(); } });
        sel.el.querySelector('select').setAttribute('aria-label', 'Load ' + (i + 1) + ' type');
        const rm = ui.button(head, { label: '✕ Remove', key: k + 'remove', title: 'Remove load ' + (i + 1),
          onClick: function () { st.loads.splice(i, 1); syncControls(); update(); } });
        const mag = ui.slider(box, { label: 'P', min: -100, max: 100, step: 1, value: 10, unit: 'kN', key: k + 'mag',
          onInput: function (v) { const ld = st.loads[i]; ld[MAG[ld.kind].field] = v; update(); } });
        const sx = ui.slider(box, { label: 'x', min: 0, max: st.L, step: 0.1, value: 0, unit: 'm', key: k + 'x',
          onInput: function (v) {
            const ld = st.loads[i];
            if (ld.kind === 'udl') { moveUdlEnd(ld, 'x1', v); syncRow(i); } else ld.x = v;
            update();
          } });
        const sx2 = ui.slider(box, { label: 'x<sub>2</sub>', min: 0, max: st.L, step: 0.1, value: 0, unit: 'm', key: k + 'x2',
          onInput: function (v) { moveUdlEnd(st.loads[i], 'x2', v); syncRow(i); update(); } });
        return { box: box, sel: sel, rm: rm, mag: mag, sx: sx, sx2: sx2, kind: null };
      }
      function syncRow(i) {
        const r = rows[i], ld = st.loads[i];
        if (!ld) { r.box.style.display = 'none'; return; }
        r.box.style.display = '';
        if (r.kind !== ld.kind) {
          r.kind = ld.kind;
          r.sel.set(ld.kind);
          const mg = MAG[ld.kind];
          r.mag.setRange(mg.min, mg.max);
          if (ld.kind === 'point') { r.mag.setLabel('P (+ downward)'); r.sx.setLabel('position x'); }
          else if (ld.kind === 'udl') { r.mag.setLabel('w (+ downward)'); r.sx.setLabel('from x<sub>1</sub>'); r.sx2.setLabel('to x<sub>2</sub>'); }
          else { r.mag.setLabel('M₀ (+ clockwise)'); r.sx.setLabel('position x'); }
          r.mag.el.querySelector('.sim-unit').innerHTML = ld.kind === 'point' ? 'kN' : ld.kind === 'udl' ? 'kN/m' : 'kN·m';
          r.sx2.el.style.display = ld.kind === 'udl' ? '' : 'none';
        }
        r.mag.set(ld[MAG[ld.kind].field]);
        r.sx.setRange(0, st.L); r.sx2.setRange(0, st.L);
        if (ld.kind === 'udl') { r.sx.set(ld.x1); r.sx2.set(ld.x2); } else r.sx.set(ld.x);
      }
      function syncControls() {
        segT.set(st.type);
        sL.set(st.L);
        sEI.set(st.EI);
        sA.setRange(0, st.L); sB.setRange(0, st.L);
        sA.set(st.a); sB.set(st.b);
        gSbox.style.display = st.type === 'ss' ? '' : 'none';
        for (let i = 0; i < MAXLOADS; i++) syncRow(i);
        const full = st.loads.length >= MAXLOADS;
        bAddP.disabled = full; bAddU.disabled = full; bAddC.disabled = full;
        chkD.set(st.showDeflection);
      }

      function presetOf() {
        for (let i = 0; i < PRESETS.length; i++) {
          const p = presetState(PRESETS[i]);
          if (p.type !== st.type || p.L !== st.L || st.EI !== 10000) continue;
          if (st.type === 'ss' && (p.a !== st.a || p.b !== st.b)) continue;
          if (p.loads.length !== st.loads.length) continue;
          let same = true;
          p.loads.forEach(function (l, j) {
            const q = st.loads[j];
            Object.keys(l).forEach(function (key) { if (l[key] !== q[key]) same = false; });
            if (l.kind !== q.kind) same = false;
          });
          if (same) return PRESETS[i].value;
        }
        return 'custom';
      }

      // ================================================================ computation cache
      let R = null, S = null, EQ = null, beamNb = null, hoverX = null;
      let derived = null;
      function compute() {
        const beam = beamOf();
        beamNb = normalize(beam);
        R = model.solve(beam);
        S = model.sample(R, 400);
        EQ = model.equilibrium(beam, R);
        derived = derive();
      }
      // plotting helpers derived from the model's values (no new numbers)
      function derive() {
        const L = st.L, cx = R.criticalX;
        let vmin = 0, vmax = 0, mmin = 0, mmax = 0, ymin = 0, ymax = 0;
        for (let i = 0; i < S.x.length; i++) {
          vmin = Math.min(vmin, S.V[i]); vmax = Math.max(vmax, S.V[i]);
          mmin = Math.min(mmin, S.M[i]); mmax = Math.max(mmax, S.M[i]);
          ymin = Math.min(ymin, S.y[i]); ymax = Math.max(ymax, S.y[i]);
        }
        const tolV = 1e-9 * Math.max(1, vmax - vmin), tolM = 1e-9 * Math.max(1, mmax - mmin);
        const eps = 1e-9 * Math.max(1, L);
        // segments between consecutive critical points: start / end values of V and M
        const segs = [];
        for (let i = 0; i + 1 < cx.length; i++) {
          const p = cx[i], q = cx[i + 1];
          segs.push({ x0: p, x1: q, v0: R.V(p, 1), v1: R.V(q, -1), m0: R.M(p, 1), m1: R.M(q, -1) });
        }
        // V sign changes (isolated): inside a segment (a zero of V is a critical point) or through a jump
        const vCross = [];
        const ent = [];
        cx.forEach(function (x) {
          if (x <= eps) ent.push({ x: x, v: R.V(x, 1) });
          else if (x >= L - eps) ent.push({ x: x, v: R.V(x, -1) });
          else { ent.push({ x: x, v: R.V(x, -1) }); ent.push({ x: x, v: R.V(x, 1) }); }
        });
        let last = null, zs = [];
        ent.forEach(function (e) {
          if (Math.abs(e.v) <= tolV) { zs.push(e); return; }
          if (last && (e.v > 0) !== (last.v > 0)) {
            if (zs.length && zs[zs.length - 1].x - zs[0].x <= eps) vCross.push(zs[0].x);
            else if (!zs.length && Math.abs(e.x - last.x) <= eps) vCross.push(e.x);
          }
          last = e; zs = [];
        });
        return { vmin: vmin, vmax: vmax, mmin: mmin, mmax: mmax, ymin: ymin, ymax: ymax, tolV: tolV, tolM: tolM, segs: segs, vCross: vCross };
      }

      // ================================================================ drawing
      const ML = 46, MR = 16;
      const FONT = '12px sans-serif', FONT_S = '11px sans-serif', FONT_B = 'bold 12px sans-serif';
      const BG = alpha('#ffffff', 0.85);
      function xPlot(ctx, w, h, ymin, ymax, top, bottom, xlabel) {
        return RASim.plot(ctx, { box: { x: 0, y: 0, w: w, h: h }, margin: { left: ML, right: MR, top: top, bottom: bottom },
          xmin: 0, xmax: st.L, ymin: ymin, ymax: ymax, xlabel: xlabel || '' });
      }
      // y-range [lo, hi] widened so that padTop / padBot PIXELS stay free above / below the data (room for labels)
      function yRange(lo, hi, H, padTop, padBot) {
        if (hi - lo < 1e-12) { lo = -1; hi = 1; }
        const k = Math.max(10, H - padTop - padBot) / (hi - lo);
        return [lo - padBot / k, hi + padTop / k];
      }
      // greedy label placement without overlaps
      function Placer(ctx, bounds) {
        const boxes = [];
        function boxOf(t, x, y, align, font) {
          ctx.save(); ctx.font = font; const tw = ctx.measureText(t).width; ctx.restore();
          const x0 = align === 'left' ? x : align === 'right' ? x - tw : x - tw / 2;
          return { x0: x0 - 3, x1: x0 + tw + 3, y0: y - 8, y1: y + 8 };
        }
        const pts = [];                       // pixel points of plotted curves that labels must not cover
        function free(bx) {
          if (bounds && (bx.x0 < bounds.x0 || bx.x1 > bounds.x1 || bx.y0 < bounds.y0 || bx.y1 > bounds.y1)) return false;
          for (let i = 0; i < boxes.length; i++) {
            const q = boxes[i];
            if (bx.x0 < q.x1 && bx.x1 > q.x0 && bx.y0 < q.y1 && bx.y1 > q.y0) return false;
          }
          for (let i = 0; i < pts.length; i += 2) {
            if (pts[i] > bx.x0 && pts[i] < bx.x1 && pts[i + 1] > bx.y0 + 1 && pts[i + 1] < bx.y1 - 1) return false;
          }
          return true;
        }
        return {
          block: function (x0, y0, x1, y1) { boxes.push({ x0: x0, y0: y0, x1: x1, y1: y1 }); },
          // a plotted polyline (data arrays through the plot p) that labels must avoid; long segments are subdivided
          avoid: function (p, xs, ys) {
            for (let i = 0; i < xs.length; i++) {
              const X = p.X(xs[i]), Y = M.clamp(p.Y(ys[i]), -1e4, 1e4);
              if (i > 0) {
                const X0 = p.X(xs[i - 1]), Y0 = M.clamp(p.Y(ys[i - 1]), -1e4, 1e4);
                const n = Math.min(400, Math.ceil(Math.max(Math.abs(X - X0), Math.abs(Y - Y0)) / 3));
                for (let k = 1; k < n; k++) pts.push(X0 + (X - X0) * k / n, Y0 + (Y - Y0) * k / n);
              }
              pts.push(X, Y);
            }
          },
          // tries each candidate { x, y, align }; draws the first free one (or, with force, the first)
          place: function (t, cands, o) {
            o = o || {};
            const font = o.font || FONT;
            let pick = null;
            for (let i = 0; i < cands.length && !pick; i++) {
              const c = cands[i];
              let x = c.x;
              if (bounds && o.clampX) {
                const bx = boxOf(t, x, c.y, c.align, font), wdt = bx.x1 - bx.x0;
                if (bx.x0 < bounds.x0) x += bounds.x0 - bx.x0; else if (bx.x1 > bounds.x1) x -= bx.x1 - bounds.x1;
                if (wdt > bounds.x1 - bounds.x0) x = c.x;
              }
              const b2 = boxOf(t, x, c.y, c.align, font);
              if (free(b2)) pick = { x: x, y: c.y, align: c.align, box: b2 };
            }
            if (!pick && o.force) {
              const c = cands[0];
              pick = { x: c.x, y: c.y, align: c.align, box: boxOf(t, c.x, c.y, c.align, font) };
            }
            if (!pick) return false;
            boxes.push(pick.box);
            D.text(ctx, t, pick.x, pick.y, { color: o.color || C.ink, font: font, align: pick.align, bg: o.bg === undefined ? BG : o.bg });
            return true;
          }
        };
      }
      function cursorLine(ctx, p, y0, y1) {
        if (hoverX == null) return;
        const px = p.X(hoverX);
        ctx.save();
        ctx.strokeStyle = alpha(C.ink, 0.55); ctx.lineWidth = 1; ctx.setLineDash([4, 3]);
        ctx.beginPath(); ctx.moveTo(Math.round(px) + 0.5, y0); ctx.lineTo(Math.round(px) + 0.5, y1); ctx.stroke();
        ctx.restore();
      }
      function fillSigned(ctx, p, xs, ys, fPos, fNeg) {
        const r = p.rect, y0 = M.clamp(p.Y(0), r.y, r.y + r.h);
        ctx.save(); ctx.beginPath(); ctx.rect(r.x, r.y, r.w, y0 - r.y); ctx.clip(); p.area(xs, ys, { fill: fPos }); ctx.restore();
        ctx.save(); ctx.beginPath(); ctx.rect(r.x, y0, r.w, r.y + r.h - y0); ctx.clip(); p.area(xs, ys, { fill: fNeg }); ctx.restore();
      }
      function title(ctx, p, t) { D.text(ctx, t, p.rect.x, 10, { font: FONT_B, color: C.ink }); }
      const n4 = function (v) { return fmt(v, 4); };

      // ---------------------------------------------------------------- beam picture
      const BEAM_Y = 100, BT = 4;            // centre line and half thickness of the bar
      function beamGeom(w, h) {
        const p = xPlot(cvB.ctx, w, h, 0, 1, 0, 0);
        const L = st.L;
        let pmax = 0, wmax = 0;
        st.loads.forEach(function (ld) { if (ld.kind === 'point') pmax = Math.max(pmax, Math.abs(ld.P)); if (ld.kind === 'udl') wmax = Math.max(wmax, Math.abs(ld.w)); });
        // UDL levels: overlapping UDLs are stacked
        const levels = [];
        const geo = st.loads.map(function (ld, i) {
          if (ld.kind === 'point') {
            const len = 22 + 26 * (pmax > 0 ? Math.abs(ld.P) / pmax : 0);
            return { i: i, kind: 'point', X: p.X(ld.x), len: len, top: BEAM_Y - BT - len };
          }
          if (ld.kind === 'moment') return { i: i, kind: 'moment', X: p.X(ld.x), r: 13 };
          let lv = 0;
          while (levels.some(function (q) { return q.lv === lv && q.x1 > ld.x1 + 1e-9 && q.x0 < ld.x2 - 1e-9; })) lv++;
          levels.push({ lv: lv, x0: ld.x1, x1: ld.x2 });
          const hU = 16 + 8 * (wmax > 0 ? Math.abs(ld.w) / wmax : 0);
          return { i: i, kind: 'udl', X1: p.X(ld.x1), X2: p.X(ld.x2), yLine: BEAM_Y - BT - hU - lv * 30 };
        });
        return { p: p, L: L, geo: geo };
      }

      function drawCouple(ctx, cx, cy, r, cw, color) {
        // 270° arc around (cx, cy); clockwise on screen when cw
        const a0 = cw ? -2.2 : -0.94, sweep = 4.7;
        const a1 = cw ? a0 + sweep : a0 - sweep;
        ctx.save();
        ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.setLineDash([]);
        ctx.beginPath(); ctx.arc(cx, cy, r, a0, a1, !cw); ctx.stroke();
        ctx.restore();
        const ex = cx + r * Math.cos(a1), ey = cy + r * Math.sin(a1);
        const tx = cw ? -Math.sin(a1) : Math.sin(a1), ty = cw ? Math.cos(a1) : -Math.cos(a1);
        D.arrow(ctx, ex - tx * 7, ey - ty * 7, ex + tx * 1.5, ey + ty * 1.5, { color: color, width: 2, head: 8 });
      }

      cvB.draw = function (ctx, w, h) {
        if (!R) return;
        const G = beamGeom(w, h), p = G.p, L = st.L;
        const X0 = p.X(0), XL = p.X(L);
        const pl = Placer(ctx, { x0: 1, y0: 1, x1: w - 1, y1: h - 1 });
        // length scale
        const yS = BEAM_Y + 78;
        ctx.save();
        ctx.strokeStyle = C.axis; ctx.lineWidth = 1;
        ctx.beginPath(); ctx.moveTo(X0, yS + 0.5); ctx.lineTo(XL, yS + 0.5); ctx.stroke();
        const ticks = M.niceTicks(0, L, Math.max(3, Math.floor((XL - X0) / 46)));
        ctx.font = FONT_S; ctx.fillStyle = C.muted; ctx.textAlign = 'center'; ctx.textBaseline = 'top';
        ticks.forEach(function (t) {
          const px = Math.round(p.X(t)) + 0.5;
          ctx.beginPath(); ctx.moveTo(px, yS - 3); ctx.lineTo(px, yS + 4); ctx.stroke();
          ctx.fillText(fmt(t, 4), M.clamp(px, 8, w - 8), yS + 6);
        });
        ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
        ctx.fillText('x (m)', X0 - 8, yS + 1);
        ctx.restore();
        pl.block(0, yS - 4, w, yS + 20);

        // supports / wall
        const r = R.reactions;
        if (st.type === 'ss') {
          [['a', st.a, 'A', r.Ra], ['b', st.b, 'B', r.Rb]].forEach(function (s) {
            const X = p.X(s[1]), top = BEAM_Y + BT;
            ctx.save();
            ctx.fillStyle = '#ffffff'; ctx.strokeStyle = C.ink; ctx.lineWidth = 1.5;
            const th = s[0] === 'a' ? 15 : 11;
            ctx.beginPath(); ctx.moveTo(X, top); ctx.lineTo(X - 8, top + th); ctx.lineTo(X + 8, top + th); ctx.closePath(); ctx.fill(); ctx.stroke();
            let gy = top + th + 1;
            if (s[0] === 'b') {
              ctx.beginPath(); ctx.arc(X - 4, top + th + 3.5, 2.6, 0, Math.PI * 2); ctx.stroke();
              ctx.beginPath(); ctx.arc(X + 4, top + th + 3.5, 2.6, 0, Math.PI * 2); ctx.stroke();
              gy = top + th + 7;
            }
            ctx.restore();
            D.hatch(ctx, X - 11, gy, X + 11, gy, { side: 1, length: 6, spacing: 5, color: C.ink });
            D.text(ctx, s[2], X - 12, BEAM_Y + 14, { font: FONT_B, align: 'right', color: C.ink });
            pl.block(X - 22, BEAM_Y + 6, X + 12, gy + 6);
            // reaction arrow (green), pointing the way the reaction acts
            const val = s[3], yA = BEAM_Y + 32, yB = BEAM_Y + 62;
            if (Math.abs(val) > 1e-9) {
              if (val > 0) D.arrow(ctx, X, yB, X, yA, { color: C.green, width: 2.5, head: 9 });
              else D.arrow(ctx, X, yA, X, yB, { color: C.green, width: 2.5, head: 9 });
            }
            const txt = n4(Math.abs(val)) + ' kN';
            const right = X + 6 + 70 < w;
            pl.place(txt, [{ x: right ? X + 6 : X - 6, y: BEAM_Y + 50, align: right ? 'left' : 'right' },
              { x: right ? X - 6 : X + 6, y: BEAM_Y + 50, align: right ? 'right' : 'left' },
              { x: X, y: BEAM_Y + 70, align: 'center' }], { color: C.green, force: true, clampX: true });
          });
        } else {
          // hatched wall at x = 0
          D.hatch(ctx, X0, BEAM_Y - 30, X0, BEAM_Y + 30, { side: 1, length: 8, spacing: 6, color: C.ink });
          pl.block(X0 - 12, BEAM_Y - 32, X0 + 2, BEAM_Y + 32);
          // wall force R
          if (Math.abs(r.R) > 1e-9) {
            if (r.R > 0) D.arrow(ctx, X0 + 6, BEAM_Y + 56, X0 + 6, BEAM_Y + BT + 1, { color: C.green, width: 2.5, head: 9 });
            else D.arrow(ctx, X0 + 6, BEAM_Y + BT + 1, X0 + 6, BEAM_Y + 56, { color: C.green, width: 2.5, head: 9 });
          }
          pl.place('R = ' + n4(Math.abs(r.R)) + ' kN', [{ x: X0 + 12, y: BEAM_Y + 50, align: 'left' }], { color: C.green, force: true });
          // wall moment: the couple the wall applies to the beam is M_wall (clockwise +)
          if (Math.abs(r.Mwall) > 1e-9) {
            const cw = r.Mwall > 0, rr = 24;
            const a0 = cw ? -1.05 : 1.05, a1 = -a0;
            ctx.save(); ctx.strokeStyle = C.green; ctx.lineWidth = 2;
            ctx.beginPath(); ctx.arc(X0, BEAM_Y, rr, a0, a1, !cw); ctx.stroke(); ctx.restore();
            const ex = X0 + rr * Math.cos(a1), ey = BEAM_Y + rr * Math.sin(a1);
            const tx = cw ? -Math.sin(a1) : Math.sin(a1), ty = cw ? Math.cos(a1) : -Math.cos(a1);
            D.arrow(ctx, ex - tx * 7, ey - ty * 7, ex + tx * 1.5, ey + ty * 1.5, { color: C.green, width: 2, head: 8 });
            pl.place('M = ' + n4(Math.abs(r.Mwall)) + ' kN·m', [{ x: X0 + 30, y: BEAM_Y + 24, align: 'left' }], { color: C.green, force: true });
          }
        }

        // the beam bar
        ctx.save();
        ctx.fillStyle = C.fill; ctx.strokeStyle = C.ink; ctx.lineWidth = 1.5;
        ctx.fillRect(X0, BEAM_Y - BT, XL - X0, 2 * BT); ctx.strokeRect(X0, BEAM_Y - BT, XL - X0, 2 * BT);
        ctx.restore();
        pl.block(X0, BEAM_Y - BT - 1, XL, BEAM_Y + BT + 1);

        // deflected shape (exaggerated: largest |y| ≈ 18 px)
        const ymx = Math.max(Math.abs(derived.ymin), Math.abs(derived.ymax));
        if (ymx > 1e-15) {
          const k = 18 / ymx;
          ctx.save();
          ctx.strokeStyle = C.teal; ctx.lineWidth = 2; ctx.setLineDash([6, 4]);
          ctx.beginPath();
          for (let i = 0; i < S.x.length; i++) {
            const px = p.X(S.x[i]), py = BEAM_Y - k * S.y[i];
            if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
          }
          ctx.stroke(); ctx.restore();
          D.text(ctx, 'deflection not to scale', w - 4, h - 8, { font: FONT_S, color: C.teal, align: 'right' });
          pl.block(w - 150, h - 16, w, h);
        }

        // loads
        const labels = [];
        G.geo.forEach(function (g) {
          const ld = st.loads[g.i];
          if (g.kind === 'point') {
            const tip = BEAM_Y - BT - 1, tail = g.top;
            if (ld.P > 0) D.arrow(ctx, g.X, tail, g.X, tip, { color: C.red, width: 2.5, head: 10 });
            else if (ld.P < 0) D.arrow(ctx, g.X, tip, g.X, tail, { color: C.red, width: 2.5, head: 10 });
            labels.push({ t: n4(Math.abs(ld.P)) + ' kN', x: g.X, y: tail - 9, color: C.red });
          } else if (g.kind === 'udl') {
            const yb = BEAM_Y - BT - 1, yl = g.yLine;
            ctx.save();
            ctx.fillStyle = alpha(C.orange, 0.08); ctx.fillRect(g.X1, yl, g.X2 - g.X1, yb - yl);
            ctx.strokeStyle = C.orange; ctx.lineWidth = 2;
            ctx.beginPath(); ctx.moveTo(g.X1, yl); ctx.lineTo(g.X2, yl); ctx.stroke();
            ctx.restore();
            const n = Math.max(2, Math.round((g.X2 - g.X1) / 14) + 1);
            for (let j = 0; j < n; j++) {
              const ax = g.X1 + (g.X2 - g.X1) * j / (n - 1);
              if (ld.w > 0) D.arrow(ctx, ax, yl, ax, yb, { color: C.orange, width: 1.5, head: 6 });
              else if (ld.w < 0) D.arrow(ctx, ax, yb, ax, yl, { color: C.orange, width: 1.5, head: 6 });
            }
            labels.push({ t: n4(Math.abs(ld.w)) + ' kN/m', x: (g.X1 + g.X2) / 2, y: yl - 9, color: C.orange });
          } else {
            if (ld.M !== 0) drawCouple(ctx, g.X, BEAM_Y, g.r, ld.M > 0, C.purple);
            labels.push({ t: n4(Math.abs(ld.M)) + ' kN·m', x: g.X, y: BEAM_Y - g.r - 10, color: C.purple });
          }
        });
        labels.sort(function (u, v) { return v.y - u.y; });
        labels.forEach(function (lb) {
          const c = [];
          for (let k = 0; k < 6; k++) c.push({ x: lb.x, y: lb.y - 15 * k, align: 'center' });
          pl.place(lb.t, c, { color: lb.color, force: true, clampX: true, font: FONT });
        });
        cursorLine(ctx, p, 4, BEAM_Y + 66);
      };

      // ---------------------------------------------------------------- SFD
      function sfdPlot(ctx, w, h) {
        const d = derived, yr = yRange(d.vmin, d.vmax, h - 40, d.vmax > d.tolV ? 24 : 10, d.vmin < -d.tolV ? 24 : 10);
        return xPlot(ctx, w, h, yr[0], yr[1], 20, 20);
      }
      cvV.draw = function (ctx, w, h) {
        if (!R) return;
        const p = sfdPlot(ctx, w, h);
        p.axes();
        fillSigned(ctx, p, S.x, S.V, alpha(C.blue, 0.18), alpha(C.blue, 0.18));
        p.hline(0, { color: C.ink, width: 1 });
        p.line(S.x, S.V, { color: C.blue, width: 2 });
        title(ctx, p, 'SFD — V (kN)');
        const r = p.rect;
        const pl = Placer(ctx, { x0: r.x + 1, y0: r.y + 1, x1: r.x + r.w - 1, y1: r.y + r.h - 1 });
        pl.avoid(p, S.x, S.V);
        derived.vCross.forEach(function (x) { p.ring(x, 0, { color: C.blue, r: 4, width: 1.5 }); pl.block(p.X(x) - 5, p.Y(0) - 5, p.X(x) + 5, p.Y(0) + 5); });
        // labels: one per run of constant V (merged across continuous joints), otherwise the values at both
        // sides of each joint (one label where V is continuous); largest first, skipped when there is no room
        const tv = derived.tolV, sg = derived.segs;
        const isConst = function (q) { return Math.abs(q.v0 - q.v1) <= tv; };
        const cands = [];
        for (let i = 0; i < sg.length;) {
          if (isConst(sg[i])) {
            let j = i;
            while (j + 1 < sg.length && isConst(sg[j + 1]) && Math.abs(sg[j + 1].v0 - sg[j].v1) <= tv) j++;
            if (Math.abs(sg[i].v0) > tv) cands.push({ v: sg[i].v0, x: (sg[i].x0 + sg[j].x1) / 2, align: 'center', dx: 0 });
            i = j + 1;
          } else i++;
        }
        for (let i = 0; i <= sg.length; i++) {
          const left = i > 0 ? sg[i - 1] : null, right = i < sg.length ? sg[i] : null;
          const x = right ? right.x0 : left.x1;
          const vl = left ? left.v1 : 0, vr = right ? right.v0 : 0;
          const lc = left && isConst(left), rc = right && isConst(right);
          if (left && right && Math.abs(vl - vr) <= tv) {
            if (!lc && !rc && Math.abs(vl) > tv) cands.push({ v: vl, x: x, align: 'center', dx: 0 });
          } else {
            if (left && !lc && Math.abs(vl) > tv) cands.push({ v: vl, x: x, align: 'right', dx: -3 });
            if (right && !rc && Math.abs(vr) > tv) cands.push({ v: vr, x: x, align: 'left', dx: 3 });
          }
        }
        cands.sort(function (u, v) { return Math.abs(v.v) - Math.abs(u.v); });
        const done = [];
        cands.forEach(function (c) {
          const t = n4(c.v), px = p.X(c.x) + c.dx, py = p.Y(c.v);
          if (done.some(function (d) { return d.t === t && Math.abs(d.py - py) < 3 && Math.abs(d.px - px) < 40; })) return;
          const dy = c.v > 0 ? -9 : 9;
          if (pl.place(t, [{ x: px, y: py + dy, align: c.align }, { x: px, y: py + 2.2 * dy, align: c.align }], { color: C.blue, font: FONT_S, clampX: true })) done.push({ t: t, px: px, py: py });
        });
        cursorLine(ctx, p, r.y, r.y + r.h);
        if (hoverX != null) p.point(hoverX, R.V(hoverX), { color: C.blue, r: 3.5 });
      };

      // ---------------------------------------------------------------- BMD
      function bmdPlot(ctx, w, h) {
        const d = derived, bot = st.showDeflection ? 20 : 34;
        const yr = yRange(d.mmin, d.mmax, h - 20 - bot, d.mmax > d.tolM ? 28 : 10, d.mmin < -d.tolM ? 28 : 10);
        return xPlot(ctx, w, h, yr[0], yr[1], 20, bot, st.showDeflection ? '' : 'x (m)');
      }
      cvM.draw = function (ctx, w, h) {
        if (!R) return;
        const p = bmdPlot(ctx, w, h);
        p.axes();
        fillSigned(ctx, p, S.x, S.M, alpha(C.maroon, 0.16), alpha(C.gold, 0.28));
        p.hline(0, { color: C.ink, width: 1 });
        p.line(S.x, S.M, { color: C.maroon, width: 2 });
        title(ctx, p, 'BMD — M (kN·m), sagging +');
        const r = p.rect;
        const pl = Placer(ctx, { x0: r.x + 1, y0: r.y + 1, x1: r.x + r.w - 1, y1: r.y + r.h - 1 });
        pl.avoid(p, S.x, S.M);
        R.contraflexure.forEach(function (c) { pl.block(p.X(c.x) - 5, p.Y(0) - 5, p.X(c.x) + 5, p.Y(0) + 5); });
        const ext = [];
        if (R.maxSag) ext.push({ e: R.maxSag, dy: -13, color: C.maroon });
        if (R.maxHog) ext.push({ e: R.maxHog, dy: 13, color: C.ink });
        ext.forEach(function (q) {
          p.point(q.e.x, q.e.value, { color: q.color, r: 3.5 });
          pl.block(p.X(q.e.x) - 4, p.Y(q.e.value) - 4, p.X(q.e.x) + 4, p.Y(q.e.value) + 4);
        });
        ext.forEach(function (q) {
          const t = n4(q.e.value) + ' at x = ' + n4(q.e.x), px = p.X(q.e.x), py = p.Y(q.e.value);
          pl.place(t, [{ x: px, y: py + q.dy, align: 'center' }, { x: px + 6, y: py + q.dy, align: 'left' }, { x: px - 6, y: py + q.dy, align: 'right' },
            { x: px + 7, y: py, align: 'left' }, { x: px - 7, y: py, align: 'right' }], { color: q.color, font: FONT, clampX: true, force: true });
        });
        R.contraflexure.forEach(function (c) {
          p.ring(c.x, 0, { color: C.purple, r: 4.5, width: 2 });
          const px = p.X(c.x), py = p.Y(0);
          pl.place('M = 0', [{ x: px, y: py - 13, align: 'center' }, { x: px, y: py + 13, align: 'center' }, { x: px + 8, y: py - 11, align: 'left' }, { x: px - 8, y: py + 11, align: 'right' }],
            { color: C.purple, font: FONT_S, clampX: true });
        });
        // other values at the piece ends (local extremes, jumps), smaller and only where there is room
        const cands = [];
        derived.segs.forEach(function (s) {
          [[s.m0, s.x0, 'left', 3], [s.m1, s.x1, 'right', -3]].forEach(function (q) { if (Math.abs(q[0]) > derived.tolM) cands.push({ v: q[0], x: q[1], align: q[2], dx: q[3] }); });
        });
        const isExt = function (c) {
          const lm = R.M(c.x, -1), rm = R.M(c.x, 1);
          const vl = R.V(c.x, -1), vr = R.V(c.x, 1);
          const jump = Math.abs(lm - rm) > derived.tolM;
          const turn = (vl > derived.tolV && vr < -derived.tolV) || (vl < -derived.tolV && vr > derived.tolV) || Math.abs(vr) <= derived.tolV || Math.abs(vl) <= derived.tolV;
          return jump || turn;
        };
        const shown = ext.map(function (q) { return n4(q.e.value); });
        cands.filter(function (c) { return c.x > 1e-9 && c.x < st.L - 1e-9 && isExt(c); })
          .sort(function (u, v) { return Math.abs(v.v) - Math.abs(u.v); })
          .forEach(function (c) {
            if ((R.maxSag && Math.abs(c.v - R.maxSag.value) < derived.tolM && Math.abs(c.x - R.maxSag.x) < 1e-9) ||
                (R.maxHog && Math.abs(c.v - R.maxHog.value) < derived.tolM && Math.abs(c.x - R.maxHog.x) < 1e-9)) return;
            const t = n4(c.v);
            if (shown.indexOf(t) >= 0) return;                    // e.g. the far end of a constant-M stretch
            const dy = c.v > 0 ? -9 : 9;
            if (pl.place(t, [{ x: p.X(c.x) + c.dx, y: p.Y(c.v) + dy, align: c.align }, { x: p.X(c.x) + c.dx, y: p.Y(c.v) + 2 * dy, align: c.align },
              { x: p.X(c.x) - c.dx, y: p.Y(c.v) + dy, align: c.align === 'left' ? 'right' : 'left' }], { color: C.muted, font: FONT_S, clampX: true })) shown.push(t);
          });
        cursorLine(ctx, p, r.y, r.y + r.h);
        if (hoverX != null) p.point(hoverX, R.M(hoverX), { color: C.maroon, r: 3.5 });
      };

      // ---------------------------------------------------------------- deflection
      function deflPlot(ctx, w, h) {
        // the label of the maximum deflection goes beyond the extreme: 30 px there, 14 px on the other side
        const lo = derived.ymin * 1000, hi = derived.ymax * 1000, down = -lo >= hi;
        const yr = yRange(lo, hi, h - 54, down ? (hi > 0 ? 14 : 10) : 30, down ? 30 : (lo < 0 ? 14 : 10));
        return xPlot(ctx, w, h, yr[0], yr[1], 20, 34, 'x (m)');
      }
      cvY.draw = function (ctx, w, h) {
        if (!R) return;
        const p = deflPlot(ctx, w, h);
        p.axes();
        const ymm = S.y.map(function (v) { return v * 1000; });
        p.area(S.x, ymm, { fill: alpha(C.teal, 0.12) });
        p.hline(0, { color: C.ink, width: 1 });
        p.line(S.x, ymm, { color: C.teal, width: 2 });
        title(ctx, p, 'Deflection — y (mm), upward +');
        const r = p.rect;
        const pl = Placer(ctx, { x0: r.x + 1, y0: r.y + 1, x1: r.x + r.w - 1, y1: r.y + r.h - 1 });
        pl.avoid(p, S.x, ymm);
        if (st.type === 'ss') {
          [st.a, st.b].forEach(function (s) {
            const px = p.X(s), py = p.Y(0);
            ctx.save(); ctx.fillStyle = C.ink;
            ctx.beginPath(); ctx.moveTo(px, py); ctx.lineTo(px - 5, py + 8); ctx.lineTo(px + 5, py + 8); ctx.closePath(); ctx.fill(); ctx.restore();
            pl.block(px - 6, py - 2, px + 6, py + 9);
          });
        }
        const md = R.maxDeflection;
        if (Math.abs(md.value) > 1e-15) {
          const v = md.value * 1000;
          p.point(md.x, v, { color: C.teal, r: 3.5 });
          pl.block(p.X(md.x) - 4, p.Y(v) - 4, p.X(md.x) + 4, p.Y(v) + 4);
          const t = 'max ' + n4(Math.abs(v)) + ' mm ' + (v < 0 ? '↓' : '↑') + ' at x = ' + n4(md.x);
          const dy = v < 0 ? 13 : -13, px = p.X(md.x), py = p.Y(v);
          pl.place(t, [{ x: px, y: py + dy, align: 'center' }, { x: px - 7, y: py + dy, align: 'right' }, { x: px + 7, y: py + dy, align: 'left' },
            { x: px - 8, y: py, align: 'right' }, { x: px + 8, y: py, align: 'left' }], { color: C.teal, font: FONT, clampX: true, force: true });
        }
        cursorLine(ctx, p, r.y, r.y + r.h);
        if (hoverX != null) p.point(hoverX, R.y(hoverX) * 1000, { color: C.teal, r: 3.5 });
      };

      // ---------------------------------------------------------------- hover (all canvases)
      function setHover(px, cv) {
        const p = xPlot(cv.ctx, cv.width, cv.height, 0, 1, 0, 0);
        const x = p.invX(px);
        // snapped to the 0.01 m shown in the read-out, so that the values printed belong to the x printed
        const nx = (x < -0.02 * st.L || x > st.L * 1.02) ? null : M.clamp(Math.round(x * 100) / 100, 0, st.L);
        if (nx === hoverX) return;
        hoverX = nx;
        showCursor(); redrawAll();
      }
      function showCursor() {
        if (hoverX == null || !R) { readout.innerHTML = 'Hover over the diagrams to read V, M and y at any section.'; return; }
        const x = hoverX, inside = x > 1e-9 && x < st.L - 1e-9;
        // at a discontinuity show the values just left → just right of the section
        function two(f, tol) {
          const l = f(x, -1), r = f(x, 1);
          return inside && Math.abs(l - r) > tol ? n4(l) + ' → ' + n4(r) : n4(f(x));
        }
        readout.innerHTML = 'x = ' + fmtFixed(x, 2) + ' m: &nbsp;V = <b>' + two(R.V, derived.tolV) + '</b> kN, &nbsp;M = <b>' + two(R.M, derived.tolM) +
          '</b> kN·m, &nbsp;y = <b>' + n4(R.y(x) * 1000) + '</b> mm';
      }
      [cvB, cvV, cvM, cvY].forEach(function (cv) {
        cv.onHover({ move: function (x) { setHover(x, cv); }, leave: function () { if (hoverX != null) { hoverX = null; showCursor(); redrawAll(); } } });
      });
      function redrawAll() { cvB.redraw(); cvV.redraw(); cvM.redraw(); if (st.showDeflection) cvY.redraw(); }

      // ---------------------------------------------------------------- drag on the beam picture
      function handles() {
        const G = beamGeom(cvB.width, cvB.height), p = G.p, hs = [];
        G.geo.forEach(function (g) {
          if (g.kind === 'point') hs.push({ X: g.X, y0: g.top - 18, y1: BEAM_Y + BT + 2, tok: { load: g.i, end: 'x' }, pri: 0 });
          else if (g.kind === 'moment') hs.push({ X: g.X, y0: BEAM_Y - g.r - 18, y1: BEAM_Y + g.r + 3, tok: { load: g.i, end: 'x' }, pri: 0, wx: 16 });
          else {
            hs.push({ X: g.X1, y0: g.yLine - 8, y1: BEAM_Y - BT, tok: { load: g.i, end: 'x1' }, pri: 1 });
            hs.push({ X: g.X2, y0: g.yLine - 8, y1: BEAM_Y - BT, tok: { load: g.i, end: 'x2' }, pri: 1 });
          }
        });
        if (st.type === 'ss') {
          hs.push({ X: p.X(st.a), y0: BEAM_Y + BT, y1: BEAM_Y + 64, tok: { support: 'a' }, pri: 2, wx: 13 });
          hs.push({ X: p.X(st.b), y0: BEAM_Y + BT, y1: BEAM_Y + 64, tok: { support: 'b' }, pri: 2, wx: 13 });
        }
        return { p: p, hs: hs };
      }
      function findHandle(x, y) {
        const H = handles();
        let best = null, bd = Infinity;
        H.hs.forEach(function (h) {
          const d = Math.abs(x - h.X);
          if (d <= (h.wx || 10) && y >= h.y0 && y <= h.y1) {
            const score = d + h.pri * 0.01;
            if (score < bd) { bd = score; best = h; }
          }
        });
        return best ? Object.assign({ p: H.p }, best.tok) : null;
      }
      cvB.onDrag({
        hit: function (x, y) { return findHandle(x, y); },
        move: function (x, y, tok) {
          const v = snap(M.clamp(tok.p.invX(x), 0, st.L));
          if (tok.support) { moveSupport(tok.support, v); sA.set(st.a); sB.set(st.b); }
          else {
            const ld = st.loads[tok.load];
            if (!ld) return;
            if (ld.kind === 'udl') moveUdlEnd(ld, tok.end === 'x2' ? 'x2' : 'x1', v);
            else ld.x = v;
            syncRow(tok.load);
          }
          update();
        },
        cursor: function (x, y) { return findHandle(x, y) ? 'ew-resize' : null; }
      });

      // ================================================================ read-outs
      function at(x) { return '<span class="bm-at">at x = ' + n4(x) + ' m</span>'; }
      function update() {
        compute();
        const r = R.reactions;
        const arrow = function (v) { return Math.abs(v) < 1e-12 ? '' : v > 0 ? ' ↑' : ' ↓'; };
        if (st.type === 'ss') {
          tR.set('R<sub>A</sub> = ' + n4(Math.abs(r.Ra)) + ' kN' + arrow(r.Ra) + ', &nbsp;R<sub>B</sub> = ' + n4(Math.abs(r.Rb)) + ' kN' + arrow(r.Rb));
        } else {
          tR.set('R = ' + n4(Math.abs(r.R)) + ' kN' + arrow(r.R) + ', &nbsp;M<sub>wall</sub> = ' + n4(r.Mwall) + ' kN·m');
        }
        tV.set(n4(R.maxV.value) + ' kN' + at(R.maxV.x));
        tSag.set(R.maxSag ? n4(R.maxSag.value) + ' kN·m' + at(R.maxSag.x) : '—');
        tHog.set(R.maxHog ? n4(R.maxHog.value) + ' kN·m' + at(R.maxHog.x) : '—');
        tCf.set(R.contraflexure.length ? R.contraflexure.map(function (c) { return 'x = ' + n4(c.x) + ' m' + (c.atCouple ? ' (jump at the couple)' : ''); }).join(', ') : 'none');
        const md = R.maxDeflection, mm = md.value * 1000;
        tDf.set(n4(Math.abs(mm)) + ' mm' + (Math.abs(mm) < 1e-12 ? '' : mm < 0 ? ' ↓' : ' ↑') + at(md.x));
        if (st.type === 'ss') tSl.set('θ<sub>A</sub> = ' + n4(R.slope(st.a)) + ' rad, &nbsp;θ<sub>B</sub> = ' + n4(R.slope(st.b)) + ' rad');
        else tSl.set('θ at the free end (x = ' + n4(st.L) + ' m) = ' + n4(R.slope(st.L)) + ' rad');
        const good = Math.abs(EQ.sumF) <= 1e-6 && Math.abs(EQ.sumM) <= 1e-6;
        tCk.set(good ? 'ΣF = 0, ΣM = 0 ✓' : 'ΣF = ' + n4(EQ.sumF) + ', ΣM = ' + n4(EQ.sumM) + ' ✗');
        tCk.setState(good ? 'ok' : 'bad');
        selP.set(presetOf());
        cvY.canvas.style.display = st.showDeflection ? '' : 'none';
        showCursor();
        redrawAll();
      }

      // ================================================================ state
      function getState() {
        return { type: st.type, L: st.L, a: st.a, b: st.b, EI: st.EI, showDeflection: st.showDeflection,
          loads: st.loads.map(function (l) { return Object.assign({}, l); }) };
      }
      function setState(s) {
        s = s || {};
        if (s.type === 'ss' || s.type === 'cantilever') st.type = s.type;
        if (s.L != null && isFinite(Number(s.L))) { sL.set(Number(s.L)); applyL(sL.get()); }
        if (s.EI != null && isFinite(Number(s.EI))) { sEI.set(Number(s.EI)); st.EI = sEI.get(); }
        if (s.a != null && isFinite(Number(s.a))) st.a = Number(s.a);
        if (s.b != null && isFinite(Number(s.b))) st.b = Number(s.b);
        if (s.a != null || s.b != null) fixSupports();
        if (Array.isArray(s.loads)) {
          st.loads = [];
          s.loads.slice(0, MAXLOADS).forEach(function (l) {
            if (!l) return;
            const kind = l.kind === 'couple' ? 'moment' : l.kind;
            if (!MAG[kind]) return;
            const ld = Object.assign({}, l, { kind: kind });
            const out = { kind: kind };
            out[MAG[kind].field] = ld[MAG[kind].field];
            if (kind === 'udl') { out.x1 = ld.x1; out.x2 = ld.x2; } else out.x = ld.x;
            st.loads.push(clampLoad(out));
          });
        }
        if (s.showDeflection != null) st.showDeflection = !!s.showDeflection;
        syncControls();
        update();
      }

      syncControls();
      update();
      return { getState: getState, setState: setState };
    }
  });
})();
