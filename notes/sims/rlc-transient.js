// rlc-transient — "RC / RL / RLC transient response"
// Sits beside notes/src/02_mechatronics.html, section "Transient response" (1. Network Elements and Analysis).
//
// SIGN CONVENTIONS AND MODEL
//   All circuits are series loops driven by a DC source V switched at t = 0 (SI units: V, Ω, H, F, s, A).
//   i is the loop current, positive out of the source's + terminal (clockwise in the schematic).
//   Element voltages use the passive sign convention with that i: v_R = R·i, v_L = L·di/dt, i = C·dv_C/dt,
//   KVL: v_R + v_L + v_C = source voltage (V in charge mode, 0 after the switch in discharge mode).
//   'charge':    zero initial energy, the source steps 0 → V at t = 0.
//   'discharge': DC steady state under V, then the source is replaced by a short at t = 0.
//                q_discharge(t) = q_steady − q_charge(t) (superposition) with the steady values under V:
//                RC: v_C = V, i = 0, v_R = 0.  RL: i = V/R, v_R = V, v_L = 0.  RLC: v_C = V, i = v_R = v_L = 0.
//   First order: τ = RC (RC) or L/R (RL); x(t) = x(∞) + [x(0⁺) − x(∞)]·e^(−t/τ).
//   Series RLC: s² + (R/L)s + 1/LC = 0, α = R/2L, ω₀ = 1/√(LC), ζ = (R/2)√(C/L) = α/ω₀, R_crit = 2√(L/C),
//   Q = 1/(2ζ). kind = 'critical' if |ζ − 1| < 1e-6, else 'under' (ζ < 1) or 'over'.
//   Poles: under −α ± jω_d (ω_d = ω₀√(1 − ζ²)); critical −α (double); over s₁ = −α + √(α² − ω₀²) (nearer the
//   origin; evaluated as −ω₀²/(α + β) to avoid cancellation) and s₂ = −α − √(α² − ω₀²).
//   response() for t < 0 returns the values just before the switch; for t = ∞ the final values.
//   A quantity whose element is absent is undefined (RC has no v_L, RL has no v_C).
//   The plotted "envelope" V(1 ± e^(−αt)) passes through every extremum of v_C (it is not tangent to the curve:
//   the exact envelope has the extra factor 1/√(1 − ζ²)), so it is labelled "through the peaks".
//
// STATE KEYS (getState / setState)
//   circuit ('RC' | 'RL' | 'RLC'), mode ('charge' | 'discharge'), V (V), R (Ω), LmH (mH), CuF (µF),
//   showVC, showVR, showVL, showI (booleans; a trace whose element is absent is always false).
//   setState applies a circuit first (a CHANGED circuit loads its default R / L / C and traces), then the rest.
//
// STAT KEYS:   tau, fiveTau, atTau, x0, xInf            (RC and RL)
//              w0, f0, alpha, zeta, wd, Rcrit, Q, poles, peak   (RLC)
// TAG KEYS:    kind   (UNDERDAMPED / CRITICALLY DAMPED / OVERDAMPED / FIRST ORDER)
// MATRIX KEYS: (none)
// DATA-KEY:    cursor (hover read-out under the plot)
// CANVAS KEYS: schematic, plot, poles
// CONTROL KEYS: circuit, mode, V, R, L, C, setRcrit, showVC, showVR, showVL, showI, reset
(function () {
  'use strict';
  const RASim = globalThis.RASim;
  const M = RASim.math;
  const ID = 'rlc-transient';

  const KIND_TOL = 1e-6;

  // ---------------------------------------------------------------- model (pure)
  function firstOrder(circuit, R, L, C) {
    if (circuit === 'RC') return { tau: R * C };
    if (circuit === 'RL') return { tau: L / R };
    return { tau: NaN };
  }

  function rlc(R, L, C) {
    const alpha = R / (2 * L);
    const w0 = 1 / Math.sqrt(L * C);
    const zeta = (R / 2) * Math.sqrt(C / L);
    const kind = Math.abs(zeta - 1) < KIND_TOL ? 'critical' : (zeta < 1 ? 'under' : 'over');
    let wd = NaN, s1, s2;
    if (kind === 'under') {
      wd = Math.sqrt((w0 - alpha) * (w0 + alpha));
      s1 = { re: -alpha, im: wd };
      s2 = { re: -alpha, im: -wd };
    } else if (kind === 'critical') {
      s1 = { re: -alpha, im: 0 };
      s2 = { re: -alpha, im: 0 };
    } else {
      const beta = Math.sqrt((alpha - w0) * (alpha + w0));
      s1 = { re: -(w0 * w0) / (alpha + beta), im: 0 };
      s2 = { re: -(alpha + beta), im: 0 };
    }
    return { alpha: alpha, w0: w0, f0: w0 / (2 * Math.PI), zeta: zeta, kind: kind, wd: wd, s1: s1, s2: s2,
      Rcrit: 2 * Math.sqrt(L / C), Q: 1 / (2 * zeta) };
  }

  // charge-mode values of the series RLC at t >= 0 (finite)
  function rlcCharge(V, R, L, r, t) {
    const a = r.alpha;
    let vC, i;
    if (r.kind === 'under') {
      const wd = r.wd, e = Math.exp(-a * t), c = Math.cos(wd * t), s = Math.sin(wd * t);
      vC = V * (1 - e * (c + (a / wd) * s));
      i = V / (wd * L) * e * s;
    } else if (r.kind === 'critical') {
      const e = Math.exp(-a * t);
      vC = V * (1 - (1 + a * t) * e);
      i = V / L * t * e;
    } else {
      const beta = Math.sqrt((a - r.w0) * (a + r.w0)), bt = beta * t;
      if (bt < 1) {
        // e^(−αt)[cosh βt + (α/β) sinh βt] = (s₂e^(s₁t) − s₁e^(s₂t))/(s₂ − s₁), stable near ζ = 1
        const e = Math.exp(-a * t), sh = Math.sinh(bt), ch = Math.cosh(bt);
        vC = V * (1 - e * (ch + a * sh / beta));
        i = V / L * e * sh / beta;
      } else {
        const s1 = r.s1.re, s2 = r.s2.re, e1 = Math.exp(s1 * t), e2 = Math.exp(s2 * t);
        vC = V * (1 - (s2 * e1 - s1 * e2) / (s2 - s1));
        i = V / L * (e1 - e2) / (s1 - s2);
      }
    }
    const vR = i * R;
    return { vC: vC, vR: vR, vL: V - vR - vC, i: i };
  }

  function chargeValues(circuit, p, t) {
    const V = p.V, R = p.R;
    if (t < 0) return circuit === 'RC' ? { vC: 0, vR: 0, vL: undefined, i: 0 }
      : circuit === 'RL' ? { vC: undefined, vR: 0, vL: 0, i: 0 } : { vC: 0, vR: 0, vL: 0, i: 0 };
    if (circuit === 'RC') {
      const tau = R * p.C;
      const e = t === Infinity ? 0 : Math.exp(-t / tau);
      const vC = t === Infinity ? V : -V * Math.expm1(-t / tau);
      return { vC: vC, vR: V * e, vL: undefined, i: V / R * e };
    }
    if (circuit === 'RL') {
      const tau = p.L / R;
      const e = t === Infinity ? 0 : Math.exp(-t / tau);
      const g = t === Infinity ? 1 : -Math.expm1(-t / tau);
      return { vC: undefined, vR: V * g, vL: V * e, i: V / R * g };
    }
    if (t === Infinity) return { vC: V, vR: 0, vL: 0, i: 0 };
    return rlcCharge(V, R, p.L, rlc(R, p.L, p.C), t);
  }

  function steady(circuit, p) {
    if (circuit === 'RC') return { vC: p.V, vR: 0, vL: undefined, i: 0 };
    if (circuit === 'RL') return { vC: undefined, vR: p.V, vL: 0, i: p.V / p.R };
    return { vC: p.V, vR: 0, vL: 0, i: 0 };
  }

  function response(circuit, mode, p, t) {
    if (mode !== 'discharge') return chargeValues(circuit, p, t);
    const s = steady(circuit, p);
    if (t < 0) return s;
    const c = chargeValues(circuit, p, t);
    function d(k) { return s[k] === undefined ? undefined : s[k] - c[k]; }
    return { vC: d('vC'), vR: d('vR'), vL: d('vL'), i: d('i') };
  }

  function tEnd(circuit, p) {
    if (circuit === 'RC' || circuit === 'RL') return 6 * firstOrder(circuit, p.R, p.L, p.C).tau;
    const r = rlc(p.R, p.L, p.C);
    if (r.kind === 'under') return Math.min(6 / r.alpha, 20 * (2 * Math.PI / r.wd));
    if (r.kind === 'critical') return 8 / r.w0;
    return 6 / Math.abs(r.s1.re);
  }

  function peak(p) {
    const r = rlc(p.R, p.L, p.C);
    if (r.kind !== 'under') return null;
    const k = Math.exp(-r.alpha * Math.PI / r.wd);
    return { tp: Math.PI / r.wd, vCmax: p.V * (1 + k), overshootPct: 100 * k };
  }

  const model = { firstOrder: firstOrder, rlc: rlc, response: response, tEnd: tEnd, peak: peak };

  // ---------------------------------------------------------------- UI constants
  const CIRCUIT_DEFAULTS = {
    RC: { R: 1000, CuF: 100, showVC: true, showVR: false, showVL: false, showI: true },
    RL: { R: 100, LmH: 100, showVC: false, showVR: false, showVL: true, showI: true },
    RLC: { R: 20, LmH: 100, CuF: 100, showVC: true, showVR: false, showVL: false, showI: true }
  };
  const DEFAULTS = { circuit: 'RLC', mode: 'charge', V: 10, R: 20, LmH: 100, CuF: 100,
    showVC: true, showVR: false, showVL: false, showI: true };
  const HAS = { RC: { C: true, L: false }, RL: { C: false, L: true }, RLC: { C: true, L: true } };
  const N = 600;   // plot samples over [0, tEnd]

  RASim.register(ID, {
    title: 'RC / RL / RLC transient response',
    model: model,
    mount: function (api) {
      const ui = RASim.ui, C = RASim.colors, D = RASim.draw;
      const fmt = RASim.fmt, fmtFixed = RASim.fmtFixed;
      const SC = '.sim[data-sim="' + ID + '"]';
      const st = Object.assign({}, DEFAULTS);
      const TC = { vC: C.blue, vR: C.green, vL: C.purple, i: C.orange };
      const F11 = '11px sans-serif', F12 = '12px sans-serif', B12 = 'bold 12px sans-serif';
      const LBG = RASim.alpha('#ffffff', 0.88);

      RASim.css(ID,
        SC + ' .rlc-nt { text-transform: none; letter-spacing: 0; font-size: 11.5px; }\n' +
        SC + ' .rlc-wide { grid-column: 1 / -1; }\n' +
        SC + ' .rlc-stats { grid-template-columns: repeat(2, minmax(0, 1fr)); }\n' +
        SC + ' .rlc-cursor { font-size: 12.5px; color: ' + C.ink + '; min-height: 1.45em; font-variant-numeric: tabular-nums; }\n' +
        SC + ' .rlc-seg { display: flex; width: 100%; }\n' +
        SC + ' .rlc-seg .sim-seg { display: flex; flex-direction: column; width: 100%; }\n' +
        SC + ' .rlc-seg .sim-seg > button { margin: -1px 0 0 0; border-radius: 0; text-align: left; }\n' +
        SC + ' .rlc-seg .sim-seg > button:first-child { margin-top: 0; border-radius: 4px 4px 0 0; }\n' +
        SC + ' .rlc-seg .sim-seg > button:last-child { border-radius: 0 0 4px 4px; }\n');
      function lab(html) { return '<span class="rlc-nt">' + html + '</span>'; }

      ui.note(api.root, 'Pick a circuit and a mode, change V, R, L and C, and watch how τ (first order) or ζ and the poles (RLC) shape the response; hover over the plot to read the values at any instant.');
      const sp = ui.split(api.root, { ratio: 1.5 });

      // ---- main column ----
      const cvS = RASim.canvas(sp.main, { aspect: 2.4, minHeight: 136, maxHeight: 180, maxWidth: 460, key: 'schematic' });
      const cvP = RASim.canvas(sp.main, { aspect: 1.3, minHeight: 330, maxHeight: 470, key: 'plot' });
      const legend = ui.legend(sp.main, []);
      const cursorLine = ui.note(sp.main, '');
      cursorLine.setAttribute('data-key', 'cursor');
      cursorLine.classList.add('rlc-cursor');
      const poleBox = ui.el('div', { style: { display: 'flex', flexDirection: 'column', gap: '4px', minWidth: '0' } }, sp.main);
      const cvZ = RASim.canvas(poleBox, { aspect: 1.3, minHeight: 210, maxHeight: 300, maxWidth: 380, key: 'poles' });
      ui.note(poleBox, 's-plane: the poles × of s² + (R/L)s + 1/LC. As R grows they slide along the dashed circle |s| = ω₀ (teal arrows), meet at s = −ω₀ when ζ = 1, then split along the real axis.');

      // ---- side column ----
      const gC = ui.group(sp.side, 'Circuit');
      const segCircuit = ui.segmented(gC, { key: 'circuit', value: st.circuit,
        options: [{ value: 'RC', label: 'RC' }, { value: 'RL', label: 'RL' }, { value: 'RLC', label: 'Series RLC' }],
        onChange: function (v) { applyCircuit(v); update(); } });
      const segMode = ui.segmented(gC, { key: 'mode', value: st.mode,
        options: [{ value: 'charge', label: 'Charge (source 0 → V)' }, { value: 'discharge', label: 'Discharge (source V → 0)' }],
        onChange: function (v) { st.mode = v === 'discharge' ? 'discharge' : 'charge'; update(); } });
      segMode.el.classList.add('rlc-seg');

      const gV = ui.group(sp.side, 'Values');
      const sV = ui.slider(gV, { label: 'Source V', min: 1, max: 20, step: 1, value: st.V, unit: 'V', key: 'V',
        onInput: function (v) { st.V = v; update(); } });
      const sR = ui.slider(gV, { label: 'Resistance R', min: 1, max: 10000, scale: 'log', value: st.R, unit: 'Ω', key: 'R',
        onInput: function (v) { st.R = v; update(); } });
      const sL = ui.slider(gV, { label: 'Inductance L', min: 1, max: 1000, scale: 'log', value: st.LmH, unit: 'mH', key: 'L',
        onInput: function (v) { st.LmH = v; update(); } });
      const sC = ui.slider(gV, { label: 'Capacitance C', min: 0.1, max: 1000, scale: 'log', value: st.CuF, unit: 'µF', key: 'C',
        onInput: function (v) { st.CuF = v; update(); } });
      const rowCrit = ui.row(gV);
      ui.button(rowCrit, { label: 'Set R = R<sub>crit</sub>', key: 'setRcrit', title: 'R = 2√(L/C): critical damping, ζ = 1',
        onClick: function () { setRcrit(); update(); } });

      const gT = ui.group(sp.side, 'Traces');
      const rowT = ui.row(gT);
      function tlab(color, html) { return '<span style="color:' + color + '; font-weight: bold;">' + html + '</span>'; }
      const cVC = ui.checkbox(rowT, { label: tlab(TC.vC, 'v<sub>C</sub>'), checked: st.showVC, key: 'showVC', onChange: function (v) { st.showVC = v; update(); } });
      const cVR = ui.checkbox(rowT, { label: tlab(TC.vR, 'v<sub>R</sub>'), checked: st.showVR, key: 'showVR', onChange: function (v) { st.showVR = v; update(); } });
      const cVL = ui.checkbox(rowT, { label: tlab(TC.vL, 'v<sub>L</sub>'), checked: st.showVL, key: 'showVL', onChange: function (v) { st.showVL = v; update(); } });
      const cI = ui.checkbox(rowT, { label: tlab(TC.i, 'i'), checked: st.showI, key: 'showI', onChange: function (v) { st.showI = v; update(); } });
      const rowReset = ui.row(gT);
      ui.button(rowReset, { label: 'Reset', key: 'reset', title: 'Restore every default',
        onClick: function () { hoverT = null; setState(Object.assign({}, DEFAULTS)); } });

      const rowTag = ui.row(sp.side);
      const tag = ui.tag(rowTag, { text: '', key: 'kind' });

      const stats = ui.stats(sp.side);
      stats.classList.add('rlc-stats');
      const tTau = ui.stat(stats, { label: lab('τ'), key: 'tau' });
      const tFive = ui.stat(stats, { label: lab('5τ (&gt; 99 % settled)'), key: 'fiveTau' });
      const tAt = ui.stat(stats, { label: lab('level at t = τ'), key: 'atTau' });
      const tX0 = ui.stat(stats, { label: lab('initial value'), key: 'x0' });
      const tXi = ui.stat(stats, { label: lab('final value'), key: 'xInf' });
      const tW0 = ui.stat(stats, { label: lab('ω₀ = 1/√(LC)'), key: 'w0' });
      const tF0 = ui.stat(stats, { label: lab('f₀ = ω₀/2π'), key: 'f0' });
      const tAl = ui.stat(stats, { label: lab('α = R/2L'), key: 'alpha' });
      const tZe = ui.stat(stats, { label: lab('ζ = (R/2)√(C/L)'), key: 'zeta' });
      const tWd = ui.stat(stats, { label: lab('ω<sub>d</sub> = ω₀√(1 − ζ²)'), key: 'wd' });
      const tRc = ui.stat(stats, { label: lab('R<sub>crit</sub> = 2√(L/C)'), key: 'Rcrit' });
      const tQ = ui.stat(stats, { label: lab('Q = 1/(2ζ)'), key: 'Q' });
      const tPo = ui.stat(stats, { label: lab('poles s₁, s₂'), key: 'poles' });
      const tPk = ui.stat(stats, { label: lab('peak v<sub>C</sub> (overshoot)'), key: 'peak' });
      [tAt, tPo, tPk].forEach(function (s) { s.el.classList.add('rlc-wide'); });
      const tauLabel = tTau.el.querySelector('.sim-stat-label');
      const FIRST_STATS = [tTau, tFive, tAt, tX0, tXi], RLC_STATS = [tW0, tF0, tAl, tZe, tWd, tRc, tQ, tPo, tPk];

      ui.hints(api.root, [
        'RC: at t = τ the capacitor has reached 63.2 % of V; after 5τ it is above 99 %. Double R or C and τ doubles.',
        'Series RLC: raise R until the oscillation just disappears — that is R = 2√(L/C) (ζ = 1), the fastest response with no overshoot. Press <b>Set R = R<sub>crit</sub></b>.',
        'Watch the pole map: a complex pair (oscillation at ω<sub>d</sub>) moves along the circle of radius ω₀, meets on the real axis at ζ = 1, then splits into two real poles.',
        'The capacitor voltage and the inductor current start exactly where they were: they cannot jump at t = 0. v<sub>R</sub>, v<sub>L</sub> and (for RC) i can.'
      ]);

      // ---------------------------------------------------------------- helpers
      function par() { return { V: st.V, R: st.R, L: st.LmH * 1e-3, C: st.CuF * 1e-6 }; }
      function timeUnit(T) { return T < 1e-3 ? { k: 1e6, u: 'µs' } : T < 1 ? { k: 1e3, u: 'ms' } : { k: 1, u: 's' }; }
      function fmtTime(t, sig) { const u = timeUnit(Math.abs(t)); return fmt(t * u.k, sig || 4) + ' ' + u.u; }
      function curUnit(ref) { return Math.abs(ref) >= 1 ? { k: 1, u: 'A' } : { k: 1e3, u: 'mA' }; }
      function fmtI(v, ref, sig) { const u = curUnit(ref); return fmt(v * u.k, sig || 4) + ' ' + u.u; }
      function fmtOhm(R, sig) { return Number(R.toPrecision(sig || 4)) >= 1000 ? fmt(R / 1000, sig || 4) + ' kΩ' : fmt(R, sig || 4) + ' Ω'; }
      function fmtHenry(L) { return L >= 1 ? fmt(L, 4) + ' H' : fmt(L * 1e3, 4) + ' mH'; }
      // v_C,max with enough digits that a small overshoot stays visible (10.00081 V, not "10 V")
      // overshoot in %: below RASim.fmt's 1e-12 cut-off (R just under R_crit, e.g. R = 63 Ω) fmt would print "0 %"
      // next to the UNDERDAMPED tag; say instead that it is positive but negligible. peak() exists only when
      // under-damped, so the true overshoot is always > 0 here, even when e^(−απ/ω_d) underflows to exactly 0
      // (e.g. a typed R = 63.245 Ω)
      function fmtOvershoot(pct) { return !(pct >= 1e-12) ? '< 10⁻¹² %: negligible' : fmt(pct, 4) + ' %'; }
      function fmtPeakV(pk) { return fmt(pk.vCmax, Math.min(7, 4 + Math.max(0, Math.ceil(-Math.log10(pk.overshootPct / 100)) - 1))); }
      function fmtFarad(Cf) { return fmt(Cf * 1e6, 4) + ' µF'; }
      const QN = { vC: 'v<sub>C</sub>', vR: 'v<sub>R</sub>', vL: 'v<sub>L</sub>', i: 'i' };
      function visible() {
        const out = [];
        if (st.showVC && HAS[st.circuit].C) out.push('vC');
        if (st.showVR) out.push('vR');
        if (st.showVL && HAS[st.circuit].L) out.push('vL');
        if (st.showI) out.push('i');
        return out;
      }
      function normalizeTraces() {
        if (!HAS[st.circuit].C) st.showVC = false;
        if (!HAS[st.circuit].L) st.showVL = false;
        cVC.set(st.showVC); cVR.set(st.showVR); cVL.set(st.showVL); cI.set(st.showI);
      }
      function applyCircuit(v) {
        if (!CIRCUIT_DEFAULTS[v]) return;
        st.circuit = v;
        segCircuit.set(v);
        const d = CIRCUIT_DEFAULTS[v];
        sR.set(d.R); st.R = sR.get();
        if (d.LmH != null) { sL.set(d.LmH); st.LmH = sL.get(); }
        if (d.CuF != null) { sC.set(d.CuF); st.CuF = sC.get(); }
        st.showVC = d.showVC; st.showVR = d.showVR; st.showVL = d.showVL; st.showI = d.showI;
        hoverT = null;
        normalizeTraces();
      }
      function setRcrit() {
        if (st.circuit !== 'RLC') return;
        const r = model.rlc(st.R, st.LmH * 1e-3, st.CuF * 1e-6);
        sR.set(M.clamp(r.Rcrit, 1, 10000)); st.R = sR.get();
      }

      // ---------------------------------------------------------------- cached samples
      let G = null;          // { key, p, T, tu, ts[], q: { vC, vR, vL, i }, iRef, cu, ... }
      let hoverT = null;     // seconds, or null
      function computeSamples() {
        const p = par();
        const key = [st.circuit, st.mode, p.V, p.R, p.L, p.C].join('|');
        if (G && G.key === key) return G;
        const T = model.tEnd(st.circuit, p);
        const tu = timeUnit(T);
        const ts = new Array(N + 1), q = { vC: [], vR: [], vL: [], i: [] };
        for (let k = 0; k <= N; k++) {
          const t = T * k / N;
          ts[k] = t * tu.k;
          const r = model.response(st.circuit, st.mode, p, t);
          q.vC.push(r.vC); q.vR.push(r.vR); q.vL.push(r.vL); q.i.push(r.i);
        }
        const ext = {};
        Object.keys(q).forEach(function (k) {
          let lo = Infinity, hi = -Infinity;
          q[k].forEach(function (v) { if (v !== undefined) { if (v < lo) lo = v; if (v > hi) hi = v; } });
          ext[k] = [lo, hi];
        });
        const iRef = Math.max(Math.abs(ext.i[0]), Math.abs(ext.i[1]));
        const cu = curUnit(iRef);
        G = { key: key, p: p, T: T, tu: tu, ts: ts, q: q, ext: ext, iRef: iRef, cu: cu,
          iS: q.i.map(function (v) { return v * cu.k; }) };
        return G;
      }

      // ---------------------------------------------------------------- plot geometry (draw + hover)
      function plotRanges(g) {
        const vis = visible();
        let lo = 0, hi = g.p.V;
        vis.forEach(function (k) { if (k !== 'i') { lo = Math.min(lo, g.ext[k][0]); hi = Math.max(hi, g.ext[k][1]); } });
        const pk = peakShown(g);
        const span = hi - lo;
        const vr = [lo - 0.06 * span, hi + (pk ? 0.17 : 0.1) * span];
        const ilo = Math.min(0, g.ext.i[0]) * g.cu.k, ihi = Math.max(0, g.ext.i[1]) * g.cu.k;
        const ispan = (ihi - ilo) || 1;
        return { v: vr, i: [ilo - 0.08 * ispan, ihi + 0.1 * ispan] };
      }
      function peakShown(g) {
        if (st.circuit !== 'RLC' || st.mode !== 'charge' || !st.showVC) return null;
        const pk = model.peak(g.p);
        return pk && pk.tp <= g.T ? pk : null;
      }
      function plots(ctx, w, h) {
        const g = computeSamples();
        const rg = plotRanges(g);
        const hTop = Math.round((h - 4) * 0.55);
        const xmax = g.T * g.tu.k;
        // first order: a row between the two plots carries the τ, 2τ … 5τ labels
        const m1 = { left: 54, right: 12, top: 10, bottom: st.circuit === 'RLC' ? 6 : 20 }, m2 = { left: 54, right: 12, top: 8, bottom: 36 };
        const pv = RASim.plot(ctx, { box: { x: 0, y: 0, w: w, h: hTop }, margin: m1, xmin: 0, xmax: xmax,
          ymin: rg.v[0], ymax: rg.v[1], ylabel: 'v (V)' });
        const pi = RASim.plot(ctx, { box: { x: 0, y: hTop, w: w, h: h - hTop }, margin: m2, xmin: 0, xmax: xmax,
          ymin: rg.i[0], ymax: rg.i[1], xlabel: 't (' + g.tu.u + ')', ylabel: 'i (' + g.cu.u + ')' });
        return { g: g, pv: pv, pi: pi };
      }

      cvP.draw = function (ctx, w, h) {
        const P = plots(ctx, w, h), g = P.g, pv = P.pv, pi = P.pi;
        const vis = visible();
        const p = g.p, k = g.tu.k;
        pv.axes({ xFormat: function () { return ''; } });
        pi.axes();
        const first = st.circuit !== 'RLC';
        const tau = first ? model.firstOrder(st.circuit, p.R, p.L, p.C).tau : NaN;

        // τ markers
        if (first) {
          for (let n = 1; n <= 5; n++) {
            [pv, pi].forEach(function (pp) { pp.vline(n * tau * k, { color: C.muted, width: 1, dash: [4, 4] }); });
          }
        }
        // source level V
        pv.hline(p.V, { color: C.faint, width: 1.5, dash: [6, 4] });
        // envelope through the peaks (under-damped, v_C shown, charge)
        const r = st.circuit === 'RLC' ? model.rlc(p.R, p.L, p.C) : null;
        const envOn = !!(r && r.kind === 'under' && st.mode === 'charge' && st.showVC);
        if (envOn) {
          const envStyle = { color: RASim.alpha(C.blue, 0.55), width: 1.3, dash: [5, 4], n: 300 };
          pv.fn(function (x) { return p.V * (1 + Math.exp(-r.alpha * x / k)); }, envStyle);
          pv.fn(function (x) { return p.V * (1 - Math.exp(-r.alpha * x / k)); }, envStyle);
        }
        // traces (current first so that voltages are on top within their own plot anyway)
        ['vR', 'vL', 'vC'].forEach(function (q) {
          if (vis.indexOf(q) >= 0) pv.line(g.ts, g.q[q], { color: TC[q], width: q === 'vC' ? 2.5 : 2 });
        });
        if (st.showI) pi.line(g.ts, g.iS, { color: TC.i, width: 2.5 });
        else D.text(ctx, 'current trace hidden (tick i to show it)', pi.rect.x + pi.rect.w / 2, pi.rect.y + pi.rect.h / 2,
          { align: 'center', color: C.muted, font: F12, bg: LBG });

        // source-level label (after the traces so that it stays readable)
        pv.text(pv.xmax, p.V, 'V = ' + fmt(p.V, 4) + ' V' + (st.mode === 'discharge' ? ' (before t = 0)' : ''), { dx: -4, dy: -10, align: 'right', color: C.muted, font: F11, bg: LBG });
        // τ labels in the row between the two plots
        if (first) {
          for (let n = 1; n <= 5; n++) {
            D.text(ctx, (n === 1 ? '' : n) + 'τ', pv.X(n * tau * k), pv.rect.y + pv.rect.h + 10, { align: 'center', color: C.ink, font: F12 });
          }
          // marker at t = τ on the main trace
          const mainQ = st.circuit === 'RC' ? 'vC' : 'i';
          if (vis.indexOf(mainQ) >= 0) {
            const x0 = model.response(st.circuit, st.mode, p, 0)[mainQ];
            const xi = model.response(st.circuit, st.mode, p, Infinity)[mainQ];
            const xt = model.response(st.circuit, st.mode, p, tau)[mainQ];
            const frac = st.mode === 'charge' ? (xt - x0) / (xi - x0) : xt / x0;
            const pp = mainQ === 'vC' ? pv : pi;
            const yv = mainQ === 'vC' ? xt : xt * g.cu.k;
            const rising = xi > x0;
            pp.point(tau * k, yv, { color: TC[mainQ], r: 4.5 });
            pp.text(tau * k, yv, fmtFixed(100 * frac, 1) + ' %', { dx: 8, dy: rising ? 11 : -11, color: TC[mainQ], font: B12, bg: LBG });
          }
        }
        // first peak of v_C
        const pk = peakShown(g);
        if (pk) {
          pv.point(pk.tp * k, pk.vCmax, { color: TC.vC, r: 4.5 });
          pv.text(pk.tp * k, pk.vCmax, 'peak ' + fmtPeakV(pk) + ' V', { dx: 7, dy: -12, color: TC.vC, font: B12, bg: LBG });
        }
        // hover cursor
        if (hoverT !== null && hoverT >= 0 && hoverT <= g.T) {
          const x = hoverT * k;
          const rv = model.response(st.circuit, st.mode, p, hoverT);
          [pv, pi].forEach(function (pp) { pp.vline(x, { color: C.ink, width: 1 }); });
          vis.forEach(function (q) {
            if (q === 'i') pi.point(x, rv.i * g.cu.k, { color: TC.i, r: 3.5 });
            else pv.point(x, rv[q], { color: TC[q], r: 3.5 });
          });
        }
      };

      function cursorText() {
        const g = computeSamples();
        if (hoverT === null || hoverT < 0 || hoverT > g.T) return 'Hover over the plot to read the values at any instant.';
        const rv = model.response(st.circuit, st.mode, g.p, hoverT);
        const parts = visible().map(function (q) {
          return QN[q] + ' = <b>' + (q === 'i' ? fmtI(rv.i, g.iRef) : fmt(rv[q], 4) + ' V') + '</b>';
        });
        return 't = ' + fmt(hoverT * g.tu.k, 4) + ' ' + g.tu.u + (parts.length ? ':  ' + parts.join(',  ') : '');
      }
      cvP.onHover({
        move: function (x, y) {
          const P = plots(cvP.ctx, cvP.width, cvP.height);
          const r = P.pv.rect;
          let t = null;
          if (x >= r.x && x <= r.x + r.w && y >= P.pv.rect.y && y <= P.pi.rect.y + P.pi.rect.h) t = P.pv.invX(x) / P.g.tu.k;
          if (t !== null) t = M.clamp(t, 0, P.g.T);
          if (t === hoverT) return;
          hoverT = t;
          cursorLine.innerHTML = cursorText();
          cvP.redraw();
        },
        leave: function () { if (hoverT === null) return; hoverT = null; cursorLine.innerHTML = cursorText(); cvP.redraw(); }
      });

      // ---------------------------------------------------------------- rich canvas labels (subscripts)
      // a label is a string or an array of parts: 'jω' or ['jω', ['d']] (an inner array is a subscript)
      function partsOf(lbl) { return typeof lbl === 'string' ? [lbl] : lbl; }
      function subFont(font) { return font.replace(/(\d+(?:\.\d+)?)px/, function (m0, n) { return Math.round(Number(n) * 0.78) + 'px'; }); }
      function richWidth(ctx, lbl, font) {
        let wsum = 0;
        partsOf(lbl).forEach(function (q) {
          ctx.font = Array.isArray(q) ? subFont(font) : font;
          wsum += ctx.measureText(Array.isArray(q) ? q[0] : q).width;
        });
        return wsum;
      }
      function richText(ctx, lbl, x, y, o) {
        const font = o.font || F12, tw = richWidth(ctx, lbl, font);
        let l = o.align === 'center' ? x - tw / 2 : o.align === 'right' ? x - tw : x;
        if (o.bg) {
          ctx.save(); ctx.fillStyle = o.bg; ctx.beginPath();
          ctx.rect(l - 3, y - 9, tw + 6, 18); ctx.fill(); ctx.restore();
        }
        ctx.save(); ctx.fillStyle = o.color || C.ink; ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
        partsOf(lbl).forEach(function (q) {
          const sub = Array.isArray(q), str = sub ? q[0] : q;
          ctx.font = sub ? subFont(font) : font;
          ctx.fillText(str, l, sub ? y + 4 : y);
          l += ctx.measureText(str).width;
        });
        ctx.restore();
      }

      // ---------------------------------------------------------------- label placement helper
      function placer(ctx, frame) {
        const boxes = [];
        function ov(a, b) { return a.l < b.r && a.r > b.l && a.t < b.b && a.b > b.t; }
        return {
          block: function (x, y, rx, ry) { boxes.push({ l: x - rx, r: x + rx, t: y - ry, b: y + ry }); },
          // cands: [{ x, y, align }] (y = middle); returns the chosen candidate (cheapest: no overlap, inside)
          place: function (lbl, font, cands) {
            const tw = richWidth(ctx, lbl, font);
            let best = null;
            cands.forEach(function (c, n) {
              const l = c.align === 'left' ? c.x : c.align === 'right' ? c.x - tw : c.x - tw / 2;
              const b = { l: l - 3, r: l + tw + 3, t: c.y - 9, b: c.y + 9 };
              let cost = n;
              boxes.forEach(function (o) { if (ov(b, o)) cost += 100; });
              if (b.l < frame.x || b.r > frame.x + frame.w || b.t < frame.y || b.b > frame.y + frame.h) cost += 1000;
              if (!best || cost < best.cost) best = { cost: cost, c: c, b: b };
            });
            boxes.push(best.b);
            return best.c;
          }
        };
      }

      // ---------------------------------------------------------------- pole map
      function polePlot(ctx, w, h, W) {
        const tf = function (v) { return Math.abs(v) >= 1e4 ? fmt(v / 1000, 3) + 'k' : fmt(v, 3); };
        const pp = RASim.plot(ctx, { box: { x: 0, y: 0, w: w, h: h }, margin: { left: 48, right: 10, top: 10, bottom: 34 },
          xmin: -2.4 * W, xmax: 1.15 * W, ymin: -1.3 * W, ymax: 1.3 * W, equal: true, xlabel: 'σ (1/s)', ylabel: 'jω (rad/s)' });
        pp.tf = tf;
        return pp;
      }
      function arrowHeadAt(ctx, x, y, dx, dy, color) {   // small filled head at (x, y) pointing along (dx, dy) (screen)
        const L = Math.hypot(dx, dy); if (!(L > 0)) return;
        // slender head (length 9, half-width 3.5) so that its direction cannot be misread
        const ux = dx / L, uy = dy / L, s = 9, wv = 3.5;
        ctx.fillStyle = color;
        ctx.beginPath();
        ctx.moveTo(x + ux * s * 0.6, y + uy * s * 0.6);
        ctx.lineTo(x - ux * s * 0.4 - uy * wv, y - uy * s * 0.4 + ux * wv);
        ctx.lineTo(x - ux * s * 0.4 + uy * wv, y - uy * s * 0.4 - ux * wv);
        ctx.closePath(); ctx.fill();
      }
      cvZ.draw = function (ctx, w, h) {
        if (st.circuit !== 'RLC') return;
        const p = par();
        const r = model.rlc(p.R, p.L, p.C);
        const W = r.w0;
        const pp = polePlot(ctx, w, h, W);
        pp.axes({ xFormat: pp.tf, yFormat: pp.tf });
        const rc = pp.rect;
        const lay = placer(ctx, rc);
        const teal = RASim.alpha(C.teal, 0.75);
        // circle |s| = ω₀ and locus arrows (direction of travel as R grows)
        pp.circle(0, 0, W, { color: C.muted, width: 1.2, dash: [5, 4] });
        const ox = pp.X(0), oy = pp.Y(0), Rp = pp.X(W) - pp.X(0);
        // each locus arrow is a short teal arc along the circle ending in a head, from angle a[0] to a[1]
        // (degrees, counter-clockwise from +σ as seen): upper pole 112° → 148°, lower pole 248° → 212°,
        // i.e. both towards 180° (the meeting point s = −ω₀) as R grows
        const scr = function (deg) { const th = M.rad(deg); return [ox + Rp * Math.cos(th), oy - Rp * Math.sin(th)]; };
        ctx.save(); ctx.strokeStyle = teal; ctx.lineWidth = 2; ctx.lineCap = 'round';
        [[112, 148], [248, 212]].forEach(function (a) {
          const sgn = a[1] > a[0] ? 1 : -1, nSeg = 12;
          ctx.beginPath();
          for (let k = 0; k <= nSeg; k++) {
            const q = scr(a[0] + (a[1] - a[0] - sgn * 4) * k / nSeg);
            if (k === 0) ctx.moveTo(q[0], q[1]); else ctx.lineTo(q[0], q[1]);
            if (k % 3 === 0) lay.block(q[0], q[1], 5, 5);
          }
          ctx.stroke();
          const th = M.rad(a[1]), e = scr(a[1]);
          // screen tangent in the direction of increasing angle is (−sin θ, −cos θ); flip it for a decreasing angle
          arrowHeadAt(ctx, e[0], e[1], -Math.sin(th) * sgn, -Math.cos(th) * sgn, teal);
          lay.block(e[0], e[1], 6, 6);
        });
        // real-axis arrows (s₁ towards the origin, s₂ away from it as R grows; skipped where a real pole sits on them)
        const realPoles = r.kind === 'under' ? [] : [r.s1.re, r.s2.re];
        [[-0.5 * W, 1], [-1.75 * W, -1]].forEach(function (a) {
          const x = pp.X(a[0]);
          if (realPoles.some(function (q) { return Math.abs(pp.X(q) - x) < 18; })) return;
          ctx.beginPath(); ctx.moveTo(x - a[1] * 16, oy); ctx.lineTo(x - a[1] * 3, oy); ctx.stroke();
          arrowHeadAt(ctx, x, oy, a[1], 0, teal); lay.block(x - a[1] * 8, oy, 12, 6);
        });
        ctx.restore();

        const red = C.red;
        const crossAt = function (x, y) {
          pp.cross(x, y, { color: red, size: 6, width: 2.5 });
          lay.block(pp.X(x), pp.Y(y), 8, 8);
        };
        const labels = [];
        function put(text, font, color, cands) { const c = lay.place(text, font, cands); labels.push([text, c, font, color]); }

        if (r.kind === 'under') {
          const a = r.alpha, wd = r.wd;
          // guides: vertical through −α, horizontals to the jω axis
          pp.segment(-a, wd, -a, -wd, { color: C.muted, width: 1, dash: [3, 3] });
          pp.segment(-a, wd, 0, wd, { color: C.muted, width: 1, dash: [3, 3] });
          pp.segment(-a, -wd, 0, -wd, { color: C.muted, width: 1, dash: [3, 3] });
          crossAt(-a, wd); crossAt(-a, -wd);
          const xa = pp.X(-a), yu = pp.Y(wd), yl = pp.Y(-wd);
          // jω_d labels just right of the jω axis
          if (yl - yu > 30) {
            put(['jω', ['d']], F12, C.ink, [{ x: ox + 5, y: yu, align: 'left' }, { x: ox + 5, y: yu - 10, align: 'left' }]);
            put(['−jω', ['d']], F12, C.ink, [{ x: ox + 5, y: yl, align: 'left' }, { x: ox + 5, y: yl + 10, align: 'left' }]);
          } else {
            put(['±jω', ['d']], F12, C.ink, [{ x: ox + 5, y: yu - 10, align: 'left' }, { x: ox + 5, y: yl + 12, align: 'left' }]);
          }
          put('−α', F12, C.ink, [{ x: xa - 4, y: oy + 10, align: 'right' }, { x: xa + 4, y: oy + 10, align: 'left' },
            { x: xa - 4, y: oy - 10, align: 'right' }, { x: xa + 4, y: oy - 10, align: 'left' }]);
          put('s₁', B12, red, [{ x: xa - 9, y: yu - 10, align: 'right' }, { x: xa + 9, y: yu - 10, align: 'left' }, { x: xa - 10, y: yu, align: 'right' }]);
          put('s₂', B12, red, [{ x: xa - 9, y: yl + 10, align: 'right' }, { x: xa + 9, y: yl + 10, align: 'left' }, { x: xa - 10, y: yl, align: 'right' }]);
        } else if (r.kind === 'critical') {
          crossAt(-r.alpha, 0);
          const xa = pp.X(-r.alpha);
          put('s₁ = s₂ = −α (double pole)', B12, red, [{ x: xa, y: oy - 16, align: 'center' }, { x: xa + 8, y: oy - 16, align: 'left' }, { x: xa, y: oy + 16, align: 'center' }]);
        } else {
          const s1 = r.s1.re, s2 = r.s2.re;
          crossAt(s1, 0);
          put('s₁', B12, red, [{ x: pp.X(s1), y: oy - 15, align: 'center' }, { x: pp.X(s1), y: oy + 15, align: 'center' }, { x: pp.X(s1) - 10, y: oy - 15, align: 'right' }]);
          if (pp.X(s2) >= rc.x + 8) {
            crossAt(s2, 0);
            put('s₂', B12, red, [{ x: pp.X(s2), y: oy - 15, align: 'center' }, { x: pp.X(s2), y: oy + 15, align: 'center' }]);
          } else {
            // off scale to the left: arrow at the edge
            D.arrow(ctx, rc.x + 30, oy, rc.x + 3, oy, { color: red, width: 2.5, head: 9 });
            lay.block(rc.x + 16, oy, 15, 6);
            const tx = 's₂ = ' + fmt(s2, 4) + ' (off scale)';
            put(tx, B12, red, [{ x: rc.x + 4, y: oy - 15, align: 'left' }, { x: rc.x + 4, y: oy + 15, align: 'left' }]);
          }
        }
        // ω₀ label on the circle, away from everything else
        const cands = [];
        [150, 210, 125, 235, 60, 300].forEach(function (d) {
          const th = M.rad(d);
          const x = ox + (Rp + 6) * Math.cos(th), y = oy - (Rp + 6) * Math.sin(th);
          cands.push({ x: x, y: y + (Math.sin(th) > 0 ? -8 : 8), align: Math.cos(th) > 0.2 ? 'left' : Math.cos(th) < -0.2 ? 'right' : 'center' });
        });
        put('|s| = ω₀', F12, C.muted, cands);
        labels.forEach(function (L) { richText(ctx, L[0], L[1].x, L[1].y, { align: L[1].align, color: L[3], font: L[2], bg: LBG }); });
      };

      // ---------------------------------------------------------------- schematic
      function zigzag(ctx, x1, x2, y) { D.spring(ctx, x1, y, x2, y, { coils: 3, amplitude: 6, color: C.ink, width: 2 }); }
      function line(ctx, pts) {
        ctx.beginPath(); ctx.moveTo(pts[0][0], pts[0][1]);
        for (let n = 1; n < pts.length; n++) ctx.lineTo(pts[n][0], pts[n][1]);
        ctx.stroke();
      }
      cvS.draw = function (ctx, w, h) {
        const p = par(), has = HAS[st.circuit];
        const rlcMode = st.circuit === 'RLC';
        ctx.font = F12;
        const long = {
          V: 'V = ' + fmt(p.V, 4) + ' V', R: 'R = ' + fmtOhm(p.R), L: 'L = ' + fmtHenry(p.L), C: 'C = ' + fmtFarad(p.C)
        };
        const short = { V: fmt(p.V, 4) + ' V', R: fmtOhm(p.R), L: fmtHenry(p.L), C: fmtFarad(p.C) };
        function wOf(s) { return ctx.measureText(s).width; }
        function geom(T) {
          const rightQ = has.L ? 'L' : 'C';
          const x0 = Math.round(10 + wOf(T.V) + 22);
          const x1 = Math.round(w - 14 - wOf(T[rightQ]) - (has.L ? 14 : 18));
          return { x0: x0, x1: x1 };
        }
        let T = long, gm = geom(long);
        if (gm.x1 - gm.x0 < 190) { T = short; gm = geom(short); }
        const x0 = gm.x0, x1 = gm.x1;
        const y0 = 40, y1 = rlcMode ? h - 34 : h - 18;
        const ya = y0 - 11, yk = y0 + 11;                 // contact a (source side) above, contact b (short) below
        const xa = x0 + 26, xp = xa + 34;                 // contacts, pivot
        const xr1 = Math.round((xp + 34 + x1) / 2 - 22), xr2 = xr1 + 44;
        const yb = (ya + y1) / 2;
        const ink = C.ink;
        ctx.strokeStyle = ink; ctx.lineWidth = 2; ctx.lineCap = 'round'; ctx.lineJoin = 'round';

        // battery on the left side (+ plate on top), connected to contact a
        line(ctx, [[xa, ya], [x0, ya], [x0, yb - 5]]);
        line(ctx, [[x0, yb + 5], [x0, y1]]);
        ctx.lineWidth = 2;
        line(ctx, [[x0 - 13, yb - 5], [x0 + 13, yb - 5]]);
        ctx.lineWidth = 4;
        line(ctx, [[x0 - 7, yb + 5], [x0 + 7, yb + 5]]);
        ctx.lineWidth = 2;
        D.text(ctx, '+', x0 - 17, yb - 11, { align: 'center', color: ink, font: B12 });
        D.text(ctx, T.V, x0 - 22, yb + 2, { align: 'right', color: ink, font: F12 });

        // change-over switch: pivot on the loop, contact a → source, contact b → short circuit to the bottom wire
        line(ctx, [[xa, yk], [xa, y1]]);
        ctx.fillStyle = ink;
        [[xa, ya], [xa, yk], [xp, y0], [xa, y1]].forEach(function (q) { ctx.beginPath(); ctx.arc(q[0], q[1], 2.7, 0, 2 * Math.PI); ctx.fill(); });
        const toA = st.mode === 'charge';                                       // charge: b → a; discharge: a → b
        const pre = toA ? [xa, yk] : [xa, ya], post = toA ? [xa, ya] : [xa, yk];
        line(ctx, [[xp, y0], [pre[0] + 1, pre[1]]]);                            // arm before t = 0
        ctx.save(); ctx.strokeStyle = C.faint; ctx.lineWidth = 1.5; ctx.setLineDash([3, 3]);
        line(ctx, [[xp, y0], [post[0] + 3, post[1]]]); ctx.restore();          // arm after t = 0
        // curved arrow around the pivot, from the old to the new position
        const aPre = Math.atan2(pre[1] - y0, pre[0] - xp), aPost = Math.atan2(post[1] - y0, post[0] - xp);
        const ccw = !toA;              // a (above) → b (below) through 180°: decreasing canvas angle
        const rr = 24;
        ctx.save(); ctx.strokeStyle = C.maroon; ctx.lineWidth = 1.5;
        ctx.beginPath(); ctx.arc(xp, y0, rr, aPre, aPost - (ccw ? -0.12 : 0.12), ccw); ctx.stroke();
        const hx = xp + rr * Math.cos(aPost), hy = y0 + rr * Math.sin(aPost);
        const sg = ccw ? -1 : 1;
        arrowHeadAt(ctx, hx, hy, -Math.sin(aPost) * sg, Math.cos(aPost) * sg, C.maroon);
        ctx.restore();
        D.text(ctx, 't = 0', (xa + xp) / 2 + 2, ya - 13, { align: 'center', color: C.maroon, font: F12 });

        // top wire: pivot → current arrow → R → corner
        line(ctx, [[xp, y0], [xr1, y0]]);
        D.arrow(ctx, xp + 8, y0, xp + 28, y0, { color: TC.i, width: 2.5, head: 9 });
        D.text(ctx, 'i', xp + 18, y0 + 11, { align: 'center', color: TC.i, font: 'bold italic 13px sans-serif' });
        zigzag(ctx, xr1, xr2, y0);
        D.text(ctx, T.R, (xr1 + xr2) / 2, y0 - 15, { align: 'center', color: TC.vR, font: B12 });
        line(ctx, [[xr2, y0], [x1, y0]]);

        // right side
        const ym = (y0 + y1) / 2;
        if (has.L) {
          const coil = Math.min(40, (y1 - y0) - 18), d = coil / 4;
          const yA = ym - coil / 2;
          line(ctx, [[x1, y0], [x1, yA]]);
          ctx.beginPath();
          for (let n = 0; n < 4; n++) ctx.arc(x1, yA + d * (n + 0.5), d / 2, -Math.PI / 2, Math.PI / 2, false);
          ctx.stroke();
          line(ctx, [[x1, yA + coil], [x1, y1]]);
          D.text(ctx, T.L, x1 + d / 2 + 9, ym, { align: 'left', color: TC.vL, font: B12 });
        } else {
          line(ctx, [[x1, y0], [x1, ym - 5]]);
          line(ctx, [[x1, ym + 5], [x1, y1]]);
          ctx.lineWidth = 2.5;
          line(ctx, [[x1 - 13, ym - 5], [x1 + 13, ym - 5]]);
          line(ctx, [[x1 - 13, ym + 5], [x1 + 13, ym + 5]]);
          ctx.lineWidth = 2;
          D.text(ctx, T.C, x1 + 18, ym, { align: 'left', color: TC.vC, font: B12 });
        }
        // bottom wire (with C for the series RLC)
        if (rlcMode) {
          const xc = Math.round((xa + x1) / 2);
          line(ctx, [[x1, y1], [xc + 5, y1]]);
          line(ctx, [[xc - 5, y1], [x0, y1]]);
          ctx.lineWidth = 2.5;
          line(ctx, [[xc - 5, y1 - 13], [xc - 5, y1 + 13]]);
          line(ctx, [[xc + 5, y1 - 13], [xc + 5, y1 + 13]]);
          ctx.lineWidth = 2;
          D.text(ctx, T.C, xc, y1 + 23, { align: 'center', color: TC.vC, font: B12 });
        } else {
          line(ctx, [[x1, y1], [x0, y1]]);
        }
      };

      // ---------------------------------------------------------------- read-outs
      let legendKey = '';
      function update() {
        normalizeTraces();
        const p = par(), has = HAS[st.circuit], first = st.circuit !== 'RLC';
        const g = computeSamples();

        // control visibility
        sL.el.style.display = has.L ? '' : 'none';
        sC.el.style.display = has.C ? '' : 'none';
        rowCrit.style.display = first ? 'none' : '';
        cVC.el.style.display = has.C ? '' : 'none';
        cVL.el.style.display = has.L ? '' : 'none';
        FIRST_STATS.forEach(function (s) { s.el.style.display = first ? '' : 'none'; });
        RLC_STATS.forEach(function (s) { s.el.style.display = first ? 'none' : ''; });
        poleBox.style.display = first ? 'none' : '';

        if (first) {
          tag.set('FIRST ORDER', '');
          const tau = model.firstOrder(st.circuit, p.R, p.L, p.C).tau;
          tauLabel.innerHTML = lab(st.circuit === 'RC' ? 'τ = RC' : 'τ = L/R');
          tTau.set(fmtTime(tau));
          tFive.set(fmtTime(5 * tau));
          const q = st.circuit === 'RC' ? 'vC' : 'i';
          const v0 = model.response(st.circuit, st.mode, p, 0)[q];
          const vi = model.response(st.circuit, st.mode, p, Infinity)[q];
          const vt = model.response(st.circuit, st.mode, p, tau)[q];
          const frac = st.mode === 'charge' ? (vt - v0) / (vi - v0) : vt / v0;
          const ref = p.V / p.R;
          const f = function (v) { return q === 'i' ? fmtI(v, ref) : fmt(v, 4) + ' V'; };
          tAt.set(QN[q] + '(τ) = ' + f(vt) + ' (' + fmtFixed(100 * frac, 1) + ' %)');
          tX0.set(QN[q] + '(0⁺) = ' + f(v0));
          tXi.set(QN[q] + '(∞) = ' + f(vi));
        } else {
          const r = model.rlc(p.R, p.L, p.C);
          tag.set(r.kind === 'under' ? 'UNDERDAMPED' : r.kind === 'critical' ? 'CRITICALLY DAMPED' : 'OVERDAMPED',
            r.kind === 'under' ? 'warn' : r.kind === 'critical' ? 'ok' : '');
          tW0.set(fmt(r.w0, 5) + ' rad/s');
          tF0.set(fmt(r.f0, 5) + ' Hz');
          tAl.set(fmt(r.alpha, 5) + ' s⁻¹');
          tZe.set(fmt(r.zeta, 5));
          tWd.set(r.kind === 'under' ? fmt(r.wd, 5) + ' rad/s' : '— (no oscillation)');
          tRc.set(fmtOhm(r.Rcrit, 5));
          tQ.set(fmt(r.Q, 5));
          if (r.kind === 'under') tPo.set(fmt(r.s1.re, 5) + ' ± j' + fmt(r.wd, 5));
          else if (r.kind === 'critical') tPo.set(fmt(r.s1.re, 5) + ' (double)');
          else tPo.set(fmt(r.s1.re, 5) + ',  ' + fmt(r.s2.re, 5));
          const pk = st.mode === 'charge' ? model.peak(p) : null;
          tPk.set(pk ? fmtPeakV(pk) + ' V at ' + fmtTime(pk.tp) + ' (' + fmtOvershoot(pk.overshootPct) + ')' : '—');
        }

        // legend
        const items = [];
        visible().forEach(function (q) { items.push({ color: TC[q], label: QN[q] + '(t)' }); });
        items.push({ color: C.faint, label: st.mode === 'discharge' ? 'source level V (before t = 0)' : 'source level V', dash: true });
        if (first) items.push({ color: C.muted, label: 't = τ, 2τ … 5τ', dash: true });
        const r2 = first ? null : model.rlc(p.R, p.L, p.C);
        if (r2 && r2.kind === 'under' && st.mode === 'charge' && st.showVC) {
          items.push({ color: RASim.alpha(C.blue, 0.55), label: 'V(1 ± e<sup>−αt</sup>) through the peaks', dash: true });
        }
        const lk = JSON.stringify(items);
        if (lk !== legendKey) { legend.set(items); legendKey = lk; }

        if (hoverT !== null && hoverT > g.T) hoverT = null;
        cursorLine.innerHTML = cursorText();
        cvS.redraw(); cvP.redraw();
        if (!first) cvZ.redraw();
      }

      function getState() {
        return { circuit: st.circuit, mode: st.mode, V: st.V, R: st.R, LmH: st.LmH, CuF: st.CuF,
          showVC: st.showVC, showVR: st.showVR, showVL: st.showVL, showI: st.showI };
      }
      function setState(s) {
        s = s || {};
        if (s.circuit != null && CIRCUIT_DEFAULTS[String(s.circuit)] && String(s.circuit) !== st.circuit) applyCircuit(String(s.circuit));
        if (s.mode != null) { st.mode = s.mode === 'discharge' ? 'discharge' : 'charge'; segMode.set(st.mode); }
        if (s.V != null) { sV.set(Number(s.V)); st.V = sV.get(); }
        if (s.R != null) { sR.set(Number(s.R)); st.R = sR.get(); }
        if (s.LmH != null) { sL.set(Number(s.LmH)); st.LmH = sL.get(); }
        if (s.CuF != null) { sC.set(Number(s.CuF)); st.CuF = sC.get(); }
        if (s.showVC != null) st.showVC = !!s.showVC;
        if (s.showVR != null) st.showVR = !!s.showVR;
        if (s.showVL != null) st.showVL = !!s.showVL;
        if (s.showI != null) st.showI = !!s.showI;
        update();
      }

      update();
      return { getState: getState, setState: setState };
    }
  });
})();
