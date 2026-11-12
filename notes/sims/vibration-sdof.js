// vibration-sdof — "Single-DOF vibration: free, forced and isolation"
// Sits beside notes/src/05_mechanical.html, sections "Vibrations of single DOF systems" and
// "Effect of damping; vibration isolation" (B2.2 Kinematics and Dynamics of Machines).
//
// SIGN CONVENTIONS AND MODEL (SI units inside the model: kg, N/m, N·s/m, m, m/s, s, rad)
//   m·x″ + c·x′ + k·x = F(t). x is measured from the static-equilibrium position, positive to the RIGHT
//   (away from the wall); the force F and every arrow in the drawings are positive to the right as well.
//   ωₙ = √(k/m), fₙ = ωₙ/2π, Tₙ = 2π/ωₙ, c_c = 2√(km) = 2mωₙ, ζ = c/c_c, δ_st = mg/k with g = 9.81 m/s².
//   kind: 'undamped' (ζ = 0), 'critical' (|ζ − 1| < 1e-6), 'under' (0 < ζ < 1), 'over' (ζ > 1).
//   Free response from x(0) = x₀, x′(0) = v₀, σ = ζωₙ:
//     x = e^(−σt)[x₀·C + (v₀ + σx₀)·t·S],  x′ = e^(−σt)[v₀·C − (σv₀ + ωₙ²x₀)·t·S]
//     with C = cos ω_d t, S = sin(ω_d t)/(ω_d t) for ζ < 1 (ω_d = ωₙ√(1 − ζ²)); C = 1, S = 1 for ζ = 1;
//     C = cosh βt, S = sinh(βt)/(βt) for ζ > 1 (β = ωₙ√(ζ² − 1)), and for βt ≥ 1 the equivalent
//     x = A·e^(s₁t) + B·e^(s₂t), s₁,₂ = −ωₙ(ζ ∓ √(ζ² − 1)), A = (v₀ − s₂x₀)/(s₁ − s₂), B = (s₁x₀ − v₀)/(s₁ − s₂).
//     This is algebraically identical to each case formula of the notes (the critical one
//     [x₀ + (v₀ + ωₙx₀)t]e^(−ωₙt) is the ζ = 1 case) and is continuous in ζ; it is used for every ζ, also
//     inside the critical band, so x is the exact solution for the ζ actually set.
//   Logarithmic decrement (0 ≤ ζ < 1): δ = ln(xₙ/xₙ₊₁) = 2πζ/√(1 − ζ²), x₁/x₂ = e^δ, T_d = 2π/ω_d,
//     cycles to halve = ln 2/δ. For ζ = 0 these are ω_d = ωₙ, δ = 0, ratio 1, cycles to halve ∞ (no decay);
//     critical and over-damped: NaN (shown as "—").
//   peaks(): positive local maxima (x′ = 0, x > 0) at ω_d t = ψ − asin ζ + 2πn, ψ = atan2((v₀ + σx₀)/ω_d, x₀).
//   Forced response to F = F₀ sin ωt, r = ω/ωₙ, steady state x = X sin(ωt − φ):
//     MF = X/(F₀/k) = 1/√[(1 − r²)² + (2ζr)²]; φ = atan2(2ζr, 1 − r²) ∈ [0, π] is the LAG of x behind F
//     (φ = π/2 at r = 1 for every ζ; for ζ = 0 and r = 1 the secular solution −(F₀ωₙt/2k)cos ωₙt also lags by
//     π/2, so phase(1, 0) = π/2); TR = F_T/F₀ = √[1 + (2ζr)²]·MF; unbalance M·X/(m₀e) = r²·MF.
//     peakMF(ζ) = { r: √(1 − 2ζ²), MF: 1/(2ζ√(1 − ζ²)) } for ζ < 1/√2, else null. ζ = 0, r = 1: MF = TR = ∞.
//   Phasor diagram (forced tab): phasors rotate counter-clockwise; F₀ is drawn at angle ωt − 90° and X at
//   ωt − 90° − φ, so their HORIZONTAL projections are F₀ sin ωt and X sin(ωt − φ), matching the horizontal
//   force arrow and mass motion. The displayed cycle lasts 2 s whatever ω (display phase = π·time).
//   Units in the UI: x₀ and X, x(t) and δ_st in mm; v₀ in m/s; φ in degrees; N in rpm.
//
// STATE KEYS (getState / setState)
//   tab ('free' | 'forced'), m (kg), k (N/m), zeta, x0 (mm), v0 (m/s), speed ('1' | '0.5' | '0.25'),
//   r, F0 (N), curve ('force' | 'unbalance'), playing (boolean), time (s of animation time).
//   setState applies tab and the parameters first, then playing, then time.
//
// STAT KEYS:   wn, fn, cc, c, Tn, dst                   (both tabs)
//              wd, Td, delta, ratio, half              (free tab)
//              w, rpm, MF, X, phase, TR, FT, isolation, peak   (forced tab)
// TAG KEYS:    kind   (UNDAMPED / UNDERDAMPED / CRITICALLY DAMPED / OVERDAMPED)
// MATRIX KEYS: (none)
// DATA-KEY:    (none)
// CANVAS KEYS: freeAnim, freePlot (free tab); ampPlot, phasePlot, trPlot, forcedAnim, phasor (forced tab)
// CONTROL KEYS: tab, m, k, zeta, x0, v0, play, restart, speed, r, F0, curve,
//               presetRes, presetSqrt2, presetIso, playForced, reset
// DRAG:        the time cursor in freePlot (scrub the time); the operating-point dot in ampPlot, phasePlot and
//              trPlot (sets r). Nothing else on the canvases captures the pointer.
(function () {
  'use strict';
  const RASim = globalThis.RASim;
  const M = RASim.math;
  const ID = 'vibration-sdof';
  const PI = Math.PI, TWO_PI = 2 * Math.PI;
  const G = 9.81;
  const KIND_TOL = 1e-6;

  // ---------------------------------------------------------------- model (pure)
  function classify(zeta) {
    if (!(zeta > 0)) return 'undamped';
    if (Math.abs(zeta - 1) < KIND_TOL) return 'critical';
    return zeta < 1 ? 'under' : 'over';
  }

  function natural(m, k) {
    const wn = Math.sqrt(k / m);
    return { wn: wn, fn: wn / TWO_PI, Tn: TWO_PI / wn, cc: 2 * Math.sqrt(k * m), dst: m * G / k };
  }

  function damping(m, k, zeta) {
    const n = natural(m, k), kind = classify(zeta);
    const out = { c: zeta * n.cc, kind: kind, wd: NaN, Td: NaN, delta: NaN, ratio: NaN, halfCycles: NaN };
    if (kind === 'under' || kind === 'undamped') {
      const z = kind === 'undamped' ? 0 : zeta;
      const s = Math.sqrt((1 - z) * (1 + z));
      out.wd = n.wn * s;
      out.Td = TWO_PI / out.wd;
      out.delta = TWO_PI * z / s;
      out.ratio = Math.exp(out.delta);
      out.halfCycles = Math.LN2 / out.delta;          // ∞ for ζ = 0
    }
    return out;
  }

  function sinc(a) { return a === 0 ? 1 : Math.sin(a) / a; }
  function sinhc(a) { return a === 0 ? 1 : Math.sinh(a) / a; }

  // [x, x′] of the free response (exact for every ζ ≥ 0, continuous in ζ)
  function freeState(m, k, zeta, x0, v0, t) {
    const wn = Math.sqrt(k / m), sg = zeta * wn;
    let C, S;                                           // S = t·sinc(…)
    if (zeta < 1) {
      const a = wn * Math.sqrt((1 - zeta) * (1 + zeta)) * t;
      C = Math.cos(a); S = t * sinc(a);
    } else if (zeta === 1) {
      C = 1; S = t;
    } else {
      const q = Math.sqrt((zeta - 1) * (zeta + 1)), a = wn * q * t;
      if (Math.abs(a) >= 1) {
        const s1 = -wn / (zeta + q), s2 = -wn * (zeta + q), d = 2 * wn * q;   // d = s₁ − s₂
        const A = (v0 - s2 * x0) / d, B = (s1 * x0 - v0) / d;
        const e1 = Math.exp(s1 * t), e2 = Math.exp(s2 * t);
        return [A * e1 + B * e2, A * s1 * e1 + B * s2 * e2];
      }
      C = Math.cosh(a); S = t * sinhc(a);
    }
    const E = Math.exp(-sg * t);
    return [E * (x0 * C + (v0 + sg * x0) * S), E * (v0 * C - (sg * v0 + wn * wn * x0) * S)];
  }
  function free(m, k, zeta, x0, v0, t) { return freeState(m, k, zeta, x0, v0, t)[0]; }
  function freeVel(m, k, zeta, x0, v0, t) { return freeState(m, k, zeta, x0, v0, t)[1]; }

  function oscParams(m, k, zeta) {
    const wn = Math.sqrt(k / m), z = Math.max(0, zeta);
    return { wn: wn, z: z, sg: z * wn, wd: wn * Math.sqrt((1 - z) * (1 + z)) };
  }

  function peaks(m, k, zeta, x0, v0, tEnd) {
    const kind = classify(zeta);
    if (kind !== 'under' && kind !== 'undamped') return [];
    const o = oscParams(m, k, zeta);
    const A = x0, B = (v0 + o.sg * x0) / o.wd;
    if (!(Math.hypot(A, B) > 0)) return [];
    const base = Math.atan2(B, A) - Math.atan2(o.sg, o.wd);       // ω_d t of the maxima: base + 2πn
    let n = Math.ceil(-base / TWO_PI - 1e-9);
    const out = [];
    for (let i = 0; i < 100000; i++, n++) {
      const t = Math.max(0, (base + TWO_PI * n) / o.wd);
      if (!(t <= tEnd * (1 + 1e-12))) break;
      out.push({ t: t, x: free(m, k, zeta, x0, v0, t) });
    }
    return out;
  }

  function envelope(m, k, zeta, x0, v0) {
    const kind = classify(zeta);
    if (kind !== 'under' && kind !== 'undamped') return NaN;
    const o = oscParams(m, k, zeta);
    return Math.hypot(x0, (v0 + o.sg * x0) / o.wd);
  }

  function freeTEnd(m, k, zeta) {
    const kind = classify(zeta), wn = Math.sqrt(k / m);
    if (kind === 'undamped') return 6 * TWO_PI / wn;
    if (kind === 'critical') return 8 / wn;
    if (kind === 'over') return 6 * (zeta + Math.sqrt((zeta - 1) * (zeta + 1))) / wn;   // 6/|s₁|
    const Td = TWO_PI / (wn * Math.sqrt((1 - zeta) * (1 + zeta)));
    return Math.max(Math.min(6 / (zeta * wn), 12 * Td), 3 * Td);
  }

  function den2(r, z) { const a = (1 - r) * (1 + r), b = 2 * z * r; return a * a + b * b; }
  function MF(r, zeta) { return 1 / Math.sqrt(den2(r, zeta)); }
  function phase(r, zeta) {
    const a = (1 - r) * (1 + r), b = 2 * zeta * r;
    if (a === 0 && b === 0) return PI / 2;
    return Math.atan2(b, a);
  }
  function TR(r, zeta) { const b = 2 * zeta * r; return Math.sqrt(1 + b * b) / Math.sqrt(den2(r, zeta)); }
  function unbalance(r, zeta) { return r * r * MF(r, zeta); }
  function peakMF(zeta) {
    if (!(zeta < Math.SQRT1_2)) return null;
    const z = Math.max(0, zeta);
    return { r: Math.sqrt(1 - 2 * z * z), MF: z > 0 ? 1 / (2 * z * Math.sqrt((1 - z) * (1 + z))) : Infinity };
  }

  const model = { classify: classify, natural: natural, damping: damping, free: free, freeVel: freeVel, peaks: peaks,
    envelope: envelope, freeTEnd: freeTEnd, MF: MF, phase: phase, TR: TR, unbalance: unbalance, peakMF: peakMF };

  // ---------------------------------------------------------------- UI constants
  const DEFAULTS = { tab: 'free', m: 1, k: 100, zeta: 0.1, x0: 50, v0: 0, speed: '0.5', r: 1, F0: 10, curve: 'force',
    playing: false, time: 0 };
  const FAMILY = [0.05, 0.1, 0.2, 0.3, 0.5, 0.707, 1];
  const NFREE = 600;                 // samples of x(t)
  const FORCED_PERIOD = 2;           // s per displayed cycle in the forced animation
  const SQRT2 = Math.SQRT2;

  RASim.register(ID, {
    title: 'Single-DOF vibration: free, forced and isolation',
    model: model,
    mount: function (api) {
      const ui = RASim.ui, C = RASim.colors, D = RASim.draw;
      const fmt = RASim.fmt, fmtFixed = RASim.fmtFixed;
      const SC = '.sim[data-sim="' + ID + '"]';
      const st = Object.assign({}, DEFAULTS);
      const F11 = '11px sans-serif', F12 = '12px sans-serif', B12 = 'bold 12px sans-serif';
      const LBG = RASim.alpha('#ffffff', 0.88);
      const COL = { x: C.blue, env: C.orange, peak: C.maroon, cursor: C.purple, cur: C.blue, fam: C.faint,
        op: C.purple, res: C.maroon, iso: C.green, force: C.red, mass: C.maroon };

      RASim.css(ID,
        SC + ' .vs-nt { text-transform: none; letter-spacing: 0; font-size: 11.5px; }\n' +
        SC + ' .vs-case { text-transform: none; letter-spacing: 0; }\n' +
        SC + ' .vs-stats { grid-template-columns: repeat(2, minmax(0, 1fr)); }\n' +
        SC + ' .vs-wide { grid-column: 1 / -1; }\n' +
        SC + ' .vs-box { display: flex; flex-direction: column; gap: 10px; min-width: 0; }\n' +
        SC + ' .vs-tabs { margin: 2px 0 10px; }\n' +
        SC + ' .vs-tabs .sim-seg > button { font-weight: bold; }\n' +
        SC + ' .vs-pre { gap: 6px 6px; }\n' +
        SC + ' .vs-pre .sim-btn { min-width: 0; padding-left: 9px; padding-right: 9px; }\n' +
        SC + ' .vs-tagrow { gap: 8px 10px; }\n' +
        SC + ' .sim-legend { align-content: flex-start; }\n');
      function lab(html) { return '<span class="vs-nt">' + html + '</span>'; }
      const WN = 'ω<sub>n</sub>';

      ui.note(api.root, 'Choose a tab: in <b>Free vibration</b> press Play and compare successive peaks of the decaying motion; ' +
        'in <b>Forced response</b> move r (or drag the purple operating point on a curve) and read the magnification, the phase lag and the transmissibility.');
      const tabsWrap = ui.el('div', { 'class': 'vs-tabs' }, api.root);
      const segTab = ui.segmented(tabsWrap, { label: '', key: 'tab', value: st.tab,
        options: [{ value: 'free', label: 'Free vibration' }, { value: 'forced', label: 'Forced response &amp; isolation' }],
        onChange: function (v) { setTab(v); update(); } });
      const sp = ui.split(api.root, { ratio: 1.5 });

      // ---- main column ----
      const freeBox = ui.el('div', { 'class': 'vs-box' }, sp.main);
      const cvFA = RASim.canvas(freeBox, { aspect: 3, minHeight: 150, maxHeight: 200, key: 'freeAnim' });
      const cvFX = RASim.canvas(freeBox, { aspect: 1.7, minHeight: 240, maxHeight: 360, key: 'freePlot' });
      const legFree = ui.legend(freeBox, []);
      const forcedBox = ui.el('div', { 'class': 'vs-box' }, sp.main);
      const cvA = RASim.canvas(forcedBox, { aspect: 1.9, minHeight: 230, maxHeight: 320, key: 'ampPlot' });
      const cvB = RASim.canvas(forcedBox, { aspect: 2.5, minHeight: 190, maxHeight: 250, key: 'phasePlot' });
      const cvC = RASim.canvas(forcedBox, { aspect: 2.1, minHeight: 210, maxHeight: 290, key: 'trPlot' });
      const animRow = ui.row(forcedBox);
      animRow.style.alignItems = 'stretch';
      const mechBox = ui.el('div', { style: { flex: '1.5 1 250px', minWidth: '0' } }, animRow);
      const phBox = ui.el('div', { style: { flex: '1 1 190px', minWidth: '0' } }, animRow);
      const cvAn = RASim.canvas(mechBox, { aspect: 1.55, minHeight: 180, maxHeight: 220, key: 'forcedAnim' });
      const cvPh = RASim.canvas(phBox, { aspect: 1.15, minHeight: 180, maxHeight: 220, key: 'phasor' });
      const legForced = ui.legend(forcedBox, []);

      // ---- side column: controls ----
      const gS = ui.group(sp.side, 'System');
      const sM = ui.slider(gS, { label: 'Mass m', min: 0.5, max: 10, step: 0.1, value: st.m, unit: 'kg', key: 'm',
        onInput: function (v) { st.m = v; update(); } });
      const sK = ui.slider(gS, { label: 'Stiffness k', min: 10, max: 2000, step: 10, value: st.k, unit: 'N/m', key: 'k',
        onInput: function (v) { st.k = v; update(); } });
      const sZ = ui.slider(gS, { label: 'Damping ratio ζ', min: 0, max: 2, step: 0.01, value: st.zeta, key: 'zeta',
        onInput: function (v) { st.zeta = v; update(); } });

      const gF = ui.group(sp.side, 'Free vibration: start and play');
      const sX0 = ui.slider(gF, { label: 'Initial displacement x<sub>0</sub>', min: -100, max: 100, step: 1, value: st.x0, unit: 'mm', key: 'x0',
        onInput: function (v) { st.x0 = v; update(); } });
      const sV0 = ui.slider(gF, { label: 'Initial velocity v<sub>0</sub>', min: -1, max: 1, step: 0.05, value: st.v0, unit: 'm/s', key: 'v0',
        onInput: function (v) { st.v0 = v; update(); } });
      const rowFP = ui.row(gF);
      const bPlay = ui.button(rowFP, { label: 'Play', primary: true, key: 'play', title: 'Play / pause the animation',
        onClick: function () { setPlaying(!anim.running); } });
      ui.button(rowFP, { label: 'Restart', key: 'restart', title: 'Back to t = 0 and play',
        onClick: function () { st.time = 0; setPlaying(true); drawAnimated(); } });
      const segSpeed = ui.segmented(gF, { label: 'Speed', key: 'speed', value: st.speed,
        options: [{ value: '1', label: '1×' }, { value: '0.5', label: '0.5×' }, { value: '0.25', label: '0.25×' }],
        onChange: function (v) { st.speed = normSpeed(v); } });

      const gD = ui.group(sp.side, 'Forcing <span class="vs-case">F<sub>0</sub> sin ωt</span>');
      const sR = ui.slider(gD, { label: 'Frequency ratio r = ω/' + WN, min: 0, max: 4, step: 0.01, value: st.r, key: 'r',
        onInput: function (v) { st.r = v; update(); } });
      const sF0 = ui.slider(gD, { label: 'Force amplitude F<sub>0</sub>', min: 1, max: 100, step: 1, value: st.F0, unit: 'N', key: 'F0',
        onInput: function (v) { st.F0 = v; update(); } });
      const selCurve = ui.select(gD, { label: 'First plot', key: 'curve', value: st.curve,
        options: [{ value: 'force', label: 'Constant force amplitude: X/(F₀/k)' }, { value: 'unbalance', label: 'Rotating unbalance: MX/(m₀e)' }],
        onChange: function (v) { st.curve = v === 'unbalance' ? 'unbalance' : 'force'; update(); } });
      const rowPre = ui.row(gD);
      rowPre.classList.add('vs-pre');
      ui.button(rowPre, { label: 'Resonance r = 1', key: 'presetRes', title: 'Set r = 1', onClick: function () { setR(1); } });
      ui.button(rowPre, { label: 'r = √2', key: 'presetSqrt2', title: 'Set r = √2 = 1.41421…', onClick: function () { setR(SQRT2); } });
      ui.button(rowPre, { label: 'Isolation r = 3', key: 'presetIso', title: 'Set r = 3', onClick: function () { setR(3); } });
      const rowDP = ui.row(gD);
      const bPlayF = ui.button(rowDP, { label: 'Play', primary: true, key: 'playForced', title: 'Play / pause the steady-state animation',
        onClick: function () { setPlaying(!anim.running); } });

      const rowTag = ui.row(sp.side);
      rowTag.classList.add('vs-tagrow');
      const tag = ui.tag(rowTag, { text: '', key: 'kind' });
      ui.button(rowTag, { label: 'Reset', key: 'reset', title: 'Restore every default', onClick: function () { setState(DEFAULTS); } });

      // ---- side column: read-outs ----
      const statsS = ui.stats(sp.side);
      statsS.classList.add('vs-stats');
      const tWn = ui.stat(statsS, { label: lab(WN + ' = √(k/m)'), key: 'wn' });
      const tFn = ui.stat(statsS, { label: lab('f<sub>n</sub> = ' + WN + '/2π'), key: 'fn' });
      const tCc = ui.stat(statsS, { label: lab('c<sub>c</sub> = 2√(km)'), key: 'cc', title: 'Critical damping coefficient' });
      const tC = ui.stat(statsS, { label: lab('c = ζ·c<sub>c</sub>'), key: 'c', title: 'Damping coefficient' });
      const tTn = ui.stat(statsS, { label: lab('T<sub>n</sub> = 2π/' + WN), key: 'Tn', title: 'Natural period' });
      const tDst = ui.stat(statsS, { label: lab('δ<sub>st</sub> = mg/k'), key: 'dst', title: 'Static deflection, g = 9.81 m/s²' });

      const statsF = ui.stats(sp.side);
      statsF.classList.add('vs-stats');
      const tWd = ui.stat(statsF, { label: lab('ω<sub>d</sub> = ' + WN + '√(1 − ζ²)'), key: 'wd' });
      const tTd = ui.stat(statsF, { label: lab('T<sub>d</sub> = 2π/ω<sub>d</sub>'), key: 'Td' });
      const tDelta = ui.stat(statsF, { label: lab('δ = 2πζ/√(1 − ζ²)'), key: 'delta', title: 'Logarithmic decrement ln(xₙ/xₙ₊₁)' });
      const tRatio = ui.stat(statsF, { label: lab('x<sub>1</sub>/x<sub>2</sub> = e<sup>δ</sup>'), key: 'ratio', title: 'Ratio of successive peaks' });
      const tHalf = ui.stat(statsF, { label: lab('cycles to halve = ln 2/δ'), key: 'half' });
      tHalf.el.classList.add('vs-wide');

      const statsD = ui.stats(sp.side);
      statsD.classList.add('vs-stats');
      const tW = ui.stat(statsD, { label: lab('ω = r·' + WN), key: 'w', title: 'Forcing frequency' });
      const tRpm = ui.stat(statsD, { label: lab('N = 60ω/2π'), key: 'rpm', title: 'Forcing frequency as a machine speed' });
      const tMF = ui.stat(statsD, { label: lab('MF = X/(F<sub>0</sub>/k)'), key: 'MF', title: 'Magnification factor 1/√[(1 − r²)² + (2ζr)²]' });
      const tX = ui.stat(statsD, { label: lab('X = MF·F<sub>0</sub>/k'), key: 'X', title: 'Steady-state amplitude' });
      const tPh = ui.stat(statsD, { label: lab('phase lag φ'), key: 'phase', title: 'φ = atan2(2ζr, 1 − r²): x lags F by φ' });
      const tTR = ui.stat(statsD, { label: lab('TR = F<sub>T</sub>/F<sub>0</sub>'), key: 'TR', title: 'Transmissibility √[1 + (2ζr)²]/√[(1 − r²)² + (2ζr)²]' });
      const tFT = ui.stat(statsD, { label: lab('F<sub>T</sub> = TR·F<sub>0</sub>'), key: 'FT', title: 'Force transmitted to the foundation (amplitude)' });
      const tIso = ui.stat(statsD, { label: lab('isolation (1 − TR)·100 %'), key: 'isolation' });
      const tPeak = ui.stat(statsD, { label: lab('peak of MF at r = √(1 − 2ζ²)'), key: 'peak' });
      tFT.el.classList.add('vs-wide');
      tIso.el.classList.add('vs-wide');
      tPeak.el.classList.add('vs-wide');

      ui.hints(api.root, [
        'Free vibration: read two successive peaks — ln(x<sub>1</sub>/x<sub>2</sub>) is the logarithmic decrement δ, about 2πζ for light damping.',
        'Raise ζ to 1: the fastest return to rest with no oscillation (critical damping, c = c<sub>c</sub> = 2√(km)).',
        'Forced: at r = 1 the magnification is 1/(2ζ) and the displacement lags the force by exactly 90°, whatever the damping.',
        'Every transmissibility curve passes through TR = 1 at r = √2. Beyond that the mount isolates — and there LESS damping gives better isolation.',
        'To isolate a machine running at a fixed speed, soften the mounts: a lower k lowers ω<sub>n</sub> and raises r.'
      ]);

      // ---------------------------------------------------------------- helpers
      function normSpeed(v) { v = String(v); return v === '1' || v === '0.25' ? v : '0.5'; }
      function setR(v) { sR.set(v); st.r = sR.get(); update(); }
      function setTab(v) {
        st.tab = v === 'forced' ? 'forced' : 'free';
        segTab.set(st.tab);
        wrapTime();
      }
      function period() { return st.tab === 'free' ? freeData().T : FORCED_PERIOD; }
      function wrapTime() {
        const T = period();
        if (!(st.time >= 0) || !isFinite(st.time)) st.time = 0;
        if (st.time >= T) st.time = st.time % T;
      }

      // ---------------------------------------------------------------- animation
      let scrubbing = false;
      const anim = api.animate(function (dt) {
        if (!scrubbing) {
          if (st.tab === 'free') {
            st.time += dt * Number(st.speed);
            const T = freeData().T;
            if (st.time >= T) st.time = st.time % T;
          } else st.time = (st.time + dt) % FORCED_PERIOD;
        }
        drawAnimated(true);
      });
      function drawAnimated(now) {
        if (st.tab === 'free') {
          if (now) { cvFA.drawNow(); cvFX.drawNow(); } else { cvFA.redraw(); cvFX.redraw(); }
        } else if (now) { cvAn.drawNow(); cvPh.drawNow(); } else { cvAn.redraw(); cvPh.redraw(); }
      }
      function setPlaying(on) {
        if (on && !anim.running) anim.start();
        else if (!on && anim.running) anim.stop();
        const lbl = anim.running ? 'Pause' : 'Play';
        bPlay.textContent = lbl; bPlayF.textContent = lbl;
      }

      // ---------------------------------------------------------------- cached free response
      let fCache = null;
      function freeData() {
        const key = st.m + '|' + st.k + '|' + st.zeta + '|' + st.x0 + '|' + st.v0;
        if (fCache && fCache.key === key) return fCache;
        const x0 = st.x0 / 1000;
        const T = model.freeTEnd(st.m, st.k, st.zeta);
        const ts = new Array(NFREE + 1), xs = new Array(NFREE + 1);
        let maxAbs = 0;
        for (let i = 0; i <= NFREE; i++) {
          const t = T * i / NFREE;
          ts[i] = t; xs[i] = 1000 * model.free(st.m, st.k, st.zeta, x0, st.v0, t);
          if (Math.abs(xs[i]) > maxAbs) maxAbs = Math.abs(xs[i]);
        }
        const kind = model.classify(st.zeta);
        const env = kind === 'under' ? 1000 * model.envelope(st.m, st.k, st.zeta, x0, st.v0) : NaN;
        const pk = kind === 'under' ? model.peaks(st.m, st.k, st.zeta, x0, st.v0, T) : [];
        fCache = { key: key, T: T, ts: ts, xs: xs, maxAbs: maxAbs, env: env, peaks: pk, kind: kind,
          sg: st.zeta * Math.sqrt(st.k / st.m) };
        return fCache;
      }
      function xAt(t) { return 1000 * model.free(st.m, st.k, st.zeta, st.x0 / 1000, st.v0, t); }

      // ---------------------------------------------------------------- label placement
      // Candidates are scored by overlaps with blocked boxes (weight w) and by leaving the frame; the cheapest wins.
      // Boxes are kept in a 24 px grid so that a candidate is only compared with the boxes near it.
      function placer(ctx, frame, el) {
        const CELL = 24, grid = new Map(), out = [];
        const wcache = {};
        let stamp = 0;
        function width(s, font) { const k = font + '|' + s; if (!(k in wcache)) { ctx.font = font; wcache[k] = ctx.measureText(s).width; } return wcache[k]; }
        function ov(a, b) { return a.l < b.r && a.r > b.l && a.t < b.b && a.b > b.t; }
        function cells(b, fn) {
          const i0 = Math.floor(b.l / CELL), i1 = Math.floor(b.r / CELL), j0 = Math.floor(b.t / CELL), j1 = Math.floor(b.b / CELL);
          for (let i = i0; i <= i1; i++) for (let j = j0; j <= j1; j++) fn(i * 4096 + j);
        }
        function add(b) {
          // boxes far outside the frame can never meet a label
          if (b.r < frame.x - 20 || b.l > frame.x + frame.w + 20 || b.b < frame.y - 20 || b.t > frame.y + frame.h + 20) return;
          b.s = 0;
          cells(b, function (key) { let l = grid.get(key); if (!l) { l = []; grid.set(key, l); } l.push(b); });
        }
        return {
          block: function (x, y, rx, ry, w, owner) { add({ l: x - rx, r: x + rx, t: y - ry, b: y + ry, w: w == null ? 100 : w, owner: owner }); },
          // a polyline in pixels: small boxes every ~4 px
          path: function (pts, w, owner) {
            let px = null, py = null;
            for (let i = 0; i < pts.length; i++) {
              const q = pts[i];
              if (!q) { px = null; continue; }
              if (px !== null) {
                const d = Math.hypot(q[0] - px, q[1] - py);
                if (d < 4) continue;
                const n = Math.ceil(d / 4);
                for (let j = 1; j < n; j++) add(bx(px + (q[0] - px) * j / n, py + (q[1] - py) * j / n));
              }
              add(bx(q[0], q[1]));
              px = q[0]; py = q[1];
            }
            function bx(x, y) { return { l: x - 2, r: x + 2, t: y - 2, b: y + 2, w: w, owner: owner }; }
          },
          // cands: [{ x, y, align, cost }]; opts: { optional: max cost, ignore: owner or predicate(owner) }
          put: function (text, font, color, cands, opts) {
            opts = opts || {};
            const tw = width(text, font);
            let best = null;
            cands.forEach(function (c, n) {
              const l = c.align === 'left' ? c.x : c.align === 'right' ? c.x - tw : c.x - tw / 2;
              const b = { l: l - 3, r: l + tw + 3, t: c.y - 8, b: c.y + 8 };
              let cost = (c.cost != null ? c.cost : n * 0.5);
              if (b.l < frame.x || b.r > frame.x + frame.w || b.t < frame.y || b.b > frame.y + frame.h) cost += 1000;
              stamp++;
              const hit = {};                    // a curve counts once per label, whatever its length under it
              cells(b, function (key) {
                const l2 = grid.get(key);
                if (!l2) return;
                for (let i = 0; i < l2.length; i++) {
                  const o = l2[i];
                  if (o.s === stamp) continue;
                  o.s = stamp;
                  if (opts.ignore != null && (typeof opts.ignore === 'function' ? opts.ignore(o.owner) : o.owner === opts.ignore)) continue;
                  if (!ov(b, o)) continue;
                  if (o.owner != null) { if (hit[o.owner]) continue; hit[o.owner] = 1; }
                  cost += o.w;
                }
              });
              if (!best || cost < best.cost) best = { cost: cost, c: c, b: b };
            });
            if (!best || (opts.optional != null && best.cost > opts.optional)) return null;
            best.b.w = 1000;
            add(best.b);
            out.push({ text: text, font: font, color: color, c: best.c, b: best.b });
            return best.c;
          },
          flush: function () {
            out.forEach(function (o) {
              ctx.save();
              ctx.fillStyle = LBG;
              ctx.fillRect(o.b.l, o.b.t + 1, o.b.r - o.b.l, o.b.b - o.b.t - 2);
              ctx.fillStyle = o.color; ctx.font = o.font; ctx.textBaseline = 'middle'; ctx.textAlign = 'left';
              ctx.fillText(o.text, o.b.l + 3, o.c.y);
              ctx.restore();
            });
            if (el) el.vsLabels = out.map(function (o) { return { text: o.text, l: o.b.l, r: o.b.r, t: o.b.t, b: o.b.b }; });
          }
        };
      }

      // ---------------------------------------------------------------- mechanism drawing
      // Layout of wall + spring + dash-pot + block on rollers inside box bx. aTarget: wanted amplitude (px);
      // the dash-pot must keep its piston inside the cylinder over the whole stroke 2A, which limits A.
      function mechLayout(bx, aTarget, rightReserve, topReserve, bFrac) {
        const xW = bx.x + 14;
        const B = M.clamp((bFrac || 0.12) * bx.w, 40, 64);
        const yF = bx.y + bx.h - 16;
        const top = bx.y + topReserve;
        const Hb = M.clamp(Math.min(0.9 * B, yF - 8 - top), 24, 58);
        const right = bx.x + bx.w - rightReserve;
        const A = Math.max(4, Math.min(aTarget, (right - B - xW - 40) / 4));
        const xEq = Math.max(xW + 40 + 3 * A, right - B - A - 4);   // block's left face at equilibrium
        const yb = yF - 8, ytop = yb - Hb;
        return { xW: xW, B: B, yF: yF, Hb: Hb, A: A, xEq: xEq, yb: yb, ytop: ytop,
          ySpring: ytop + Hb * 0.3, yDamp: ytop + Hb * 0.74, wallTop: Math.min(ytop - 6, bx.y + topReserve - 4), right: right };
      }
      function drawMech(ctx, bx, L, u) {
        const xm = L.xEq + u;                              // block's left face now
        // floor and wall
        D.hatch(ctx, L.xW, L.yF, bx.x + bx.w - 4, L.yF, { side: 1, color: C.ink, length: 7, spacing: 9 });
        D.hatch(ctx, L.xW, L.yF, L.xW, L.wallTop, { side: 1, color: C.ink, length: 7, spacing: 9 });
        // spring (top)
        D.spring(ctx, L.xW, L.ySpring, xm, L.ySpring, { coils: 7, amplitude: Math.min(7, L.Hb * 0.16), color: C.teal, width: 1.8 });
        // dash-pot (bottom): cylinder fixed to the wall, piston rod fixed to the block
        const y = L.yDamp, hc = Math.min(7, L.Hb * 0.16);
        const cyl0 = L.xW + 8, cyl1 = L.xEq - L.A - 6;
        const rod = 2 * L.A + 10, piston = xm - rod;
        ctx.save();
        ctx.strokeStyle = C.ink; ctx.lineWidth = 1.8; ctx.lineCap = 'butt';
        ctx.beginPath();
        ctx.moveTo(L.xW, y); ctx.lineTo(cyl0, y);                                  // wall → cylinder bottom
        ctx.moveTo(cyl1, y - hc); ctx.lineTo(cyl0, y - hc); ctx.lineTo(cyl0, y + hc); ctx.lineTo(cyl1, y + hc);  // open cylinder
        ctx.stroke();
        ctx.fillStyle = RASim.alpha(C.lightBlue, 0.35);
        ctx.fillRect(cyl0, y - hc, piston - cyl0, 2 * hc);                          // fluid behind the piston
        ctx.lineWidth = 2.4;
        ctx.beginPath(); ctx.moveTo(piston, y - hc + 1.5); ctx.lineTo(piston, y + hc - 1.5); ctx.stroke();   // piston
        ctx.lineWidth = 1.8;
        ctx.beginPath(); ctx.moveTo(piston, y); ctx.lineTo(xm, y); ctx.stroke();       // rod
        // block on rollers
        ctx.fillStyle = C.fill; ctx.strokeStyle = COL.mass; ctx.lineWidth = 2;
        ctx.fillRect(xm, L.ytop, L.B, L.Hb); ctx.strokeRect(xm, L.ytop, L.B, L.Hb);
        ctx.fillStyle = C.panel; ctx.strokeStyle = C.ink; ctx.lineWidth = 1.3;
        [xm + L.B * 0.25, xm + L.B * 0.75].forEach(function (cx) { ctx.beginPath(); ctx.arc(cx, L.yF - 4, 3.6, 0, TWO_PI); ctx.fill(); ctx.stroke(); });
        ctx.restore();
        D.text(ctx, 'm', xm + L.B / 2, L.ytop + L.Hb / 2, { align: 'center', font: 'bold 14px sans-serif', color: COL.mass });
        return xm;
      }
      // equilibrium marker (dashed line through the block centre at x = 0)
      function eqMarker(ctx, L, yTop) {
        const xc = L.xEq + L.B / 2;
        ctx.save();
        ctx.strokeStyle = C.muted; ctx.lineWidth = 1; ctx.setLineDash([4, 3]);
        ctx.beginPath(); ctx.moveTo(xc, yTop); ctx.lineTo(xc, L.yF); ctx.stroke();
        ctx.restore();
        return xc;
      }

      // ---------------------------------------------------------------- free tab: animation
      cvFA.draw = function (ctx, w, h) {
        const fd = freeData();
        const t = M.clamp(st.time, 0, fd.T);
        const xmm = xAt(t);
        const bx = { x: 0, y: 0, w: w, h: h };
        const L = mechLayout(bx, 0.3 * w, 10, 44);
        const scale = fd.maxAbs > 0 ? L.A / fd.maxAbs : 0;
        const u = M.clamp(xmm * scale, -L.A, L.A);
        const xc = eqMarker(ctx, L, L.ytop - 26);
        drawMech(ctx, bx, L, u);
        // displacement arrow and value
        const ya = L.ytop - 13;
        if (Math.abs(u) >= 2) D.arrow(ctx, xc, ya, xc + u, ya, { color: COL.x, width: 2, head: Math.min(8, Math.abs(u)) });
        ctx.save(); ctx.fillStyle = C.muted; ctx.fillRect(xc - 1, ya - 4, 2, 8); ctx.restore();
        const txt = 'x = ' + fmt(xmm, 3) + ' mm';
        const tt = 't = ' + fmtFixed(t, 3) + ' s';
        ctx.font = B12; const tw = ctx.measureText(txt).width;
        ctx.font = F11; const ttw = ctx.measureText(tt).width;
        const ly = Math.max(12, ya - 16);
        const lx = M.clamp(xc + u / 2, L.xW + 6 + tw / 2, (ly < 26 ? w - ttw - 18 : w - 4) - tw / 2);
        D.text(ctx, txt, lx, ly, { align: 'center', font: B12, color: COL.x, bg: LBG });
        D.text(ctx, tt, w - 6, 12, { align: 'right', font: F11, color: C.muted });
        D.text(ctx, 'x = 0', xc + 4, L.yF + 9, { align: 'left', font: F11, color: C.muted, bg: LBG });
      };

      // ---------------------------------------------------------------- free tab: x(t) plot
      function freePlot(ctx, w, h) {
        const fd = freeData();
        let ym = fd.maxAbs;
        if (isFinite(fd.env)) ym = Math.max(ym, fd.env);
        ym = ym > 0 ? ym * 1.12 : 1;
        return RASim.plot(ctx, { box: { x: 0, y: 0, w: w, h: h }, margin: { left: 48, right: 12, top: 12, bottom: 36 },
          xmin: 0, xmax: fd.T, ymin: -ym, ymax: ym, xlabel: 't (s)', ylabel: 'x (mm)' });
      }
      cvFX.draw = function (ctx, w, h) {
        const fd = freeData();
        const p = freePlot(ctx, w, h);
        p.axes();
        const rc = p.rect;
        const lay = placer(ctx, rc, cvFX.canvas);
        const under = fd.kind === 'under';
        if (under) {
          const es = { color: COL.env, width: 1.5, dash: [6, 4], n: 240 };
          p.fn(function (t) { return fd.env * Math.exp(-fd.sg * t); }, es);
          p.fn(function (t) { return -fd.env * Math.exp(-fd.sg * t); }, es);
        }
        p.line(fd.ts, fd.xs, { color: COL.x, width: 2.4 });
        lay.path(fd.ts.map(function (t, i) { return [p.X(t), p.Y(fd.xs[i])]; }), 30, 'x');
        // time cursor
        const t = M.clamp(st.time, 0, fd.T), xc = p.X(t);
        p.vline(t, { color: COL.cursor, width: 1.5 });
        p.point(t, xAt(t), { color: COL.cursor, r: 4, clip: false });
        lay.block(xc, rc.y + rc.h / 2, 2, rc.h / 2, 20);
        if (under) {
          const pk = fd.peaks;
          pk.forEach(function (q) { p.point(q.t, 1000 * q.x, { color: COL.peak, r: 3.8, clip: false }); lay.block(p.X(q.t), p.Y(1000 * q.x), 5, 5, 200); });
          ['x₁', 'x₂', 'x₃'].forEach(function (s, i) {
            if (!pk[i]) return;
            const px = p.X(pk[i].t), py = p.Y(1000 * pk[i].x);
            lay.put(s, B12, COL.peak, [
              { x: px, y: py - 13, align: 'center' }, { x: px + 7, y: py - 11, align: 'left' }, { x: px - 7, y: py - 11, align: 'right' },
              { x: px + 8, y: py + 4, align: 'left' }, { x: px - 8, y: py + 4, align: 'right' }, { x: px, y: py + 14, align: 'center' }]);
          });
          if (pk.length >= 2) {
            const dl = 'ln(x₁/x₂) = δ = ' + fmt(model.damping(st.m, st.k, st.zeta).delta, 3);
            const x2 = p.X(pk[1].t), y1 = p.Y(1000 * pk[0].x);
            lay.put(dl, B12, COL.peak, [
              { x: rc.x + rc.w - 6, y: rc.y + 12, align: 'right' }, { x: x2 + 10, y: rc.y + 12, align: 'left' },
              { x: rc.x + rc.w - 6, y: rc.y + rc.h - 12, align: 'right' }, { x: rc.x + rc.w - 6, y: rc.y + 30, align: 'right' },
              { x: x2 + 10, y: y1, align: 'left' }, { x: rc.x + 6, y: rc.y + rc.h - 12, align: 'left' }]);
          }
        }
        lay.flush();
        cvFX.canvas.vsPlot = { x: xc, top: rc.y, bottom: rc.y + rc.h };
      };
      cvFX.onDrag({
        hit: function (x, y) {
          const p = freePlot(cvFX.ctx, cvFX.width, cvFX.height);
          const xc = p.X(M.clamp(st.time, 0, freeData().T));
          return Math.abs(x - xc) <= 10 && y >= p.rect.y - 4 && y <= p.rect.y + p.rect.h + 4 ? { p: p } : null;
        },
        move: function (x, y, tok) {
          scrubbing = true;
          st.time = M.clamp(tok.p.invX(x), 0, freeData().T);
          cvFA.drawNow(); cvFX.drawNow();
        },
        end: function () { scrubbing = false; },
        cursor: function (x, y) {
          const p = freePlot(cvFX.ctx, cvFX.width, cvFX.height);
          const xc = p.X(M.clamp(st.time, 0, freeData().T));
          return Math.abs(x - xc) <= 10 && p.contains(M.clamp(x, p.rect.x, p.rect.x + p.rect.w), y) ? 'ew-resize' : null;
        }
      });

      // ---------------------------------------------------------------- forced tab: the three plots
      function ampFn() { return st.curve === 'unbalance' ? model.unbalance : model.MF; }
      const PLOTS = {
        A: { cv: cvA, ymax: 6, yTicks: [0, 1, 2, 3, 4, 5, 6], labelFamily: 'all',
          f: function (r, z) { return ampFn()(r, z); },
          title: function () { return st.curve === 'unbalance' ? 'Rotating unbalance M·X/(m₀e) = r²·MF' : 'Magnification factor MF = X/(F₀/k)'; },
          ylabel: function () { return st.curve === 'unbalance' ? 'MX/(m₀e)' : 'MF'; } },
        B: { cv: cvB, ymax: 180, yTicks: [0, 45, 90, 135, 180], yFormat: function (v) { return fmt(v, 3) + '°'; }, labelFamily: 'none',
          f: function (r, z) { return M.deg(model.phase(r, z)); },
          title: function () { return 'Phase lag φ of x behind F'; }, ylabel: function () { return 'φ (°)'; } },
        C: { cv: cvC, ymax: 4, yTicks: [0, 1, 2, 3, 4], labelFamily: 'some',
          f: function (r, z) { return model.TR(r, z); },
          title: function () { return 'Transmissibility TR = transmitted force / F₀'; }, ylabel: function () { return 'TR'; } }
      };
      function plotOf(which, ctx, w, h) {
        const P = PLOTS[which];
        return RASim.plot(ctx, { box: { x: 0, y: 0, w: w, h: h }, margin: { left: 46, right: 12, top: 26, bottom: 34 },
          xmin: 0, xmax: 4, ymin: 0, ymax: P.ymax, xlabel: 'r = ω/ωₙ', ylabel: P.ylabel(), title: P.title() });
      }
      // samples of a curve, clamped for drawing (an infinite peak is drawn as a spike through the top)
      const famCache = {};
      function curveSamples(which, z, n) {
        const key = which + '|' + st.curve + '|' + z + '|' + n;
        if (famCache[key]) return famCache[key];
        const P = PLOTS[which], rs = new Array(n + 1), ys = new Array(n + 1);
        for (let i = 0; i <= n; i++) {
          const r = 4 * i / n, y = P.f(r, z);
          rs[i] = r; ys[i] = y > 3 * P.ymax || !(y === y) ? 3 * P.ymax : y;
        }
        const out = { rs: rs, ys: ys };
        if (Object.keys(famCache).length > 60) Object.keys(famCache).forEach(function (k2) { delete famCache[k2]; });
        famCache[key] = out;
        return out;
      }
      function opValue(which) { return PLOTS[which].f(st.r, st.zeta); }
      function opPx(p, which) {
        const v = opValue(which), P = PLOTS[which];
        const clipped = !(v <= P.ymax);
        return { x: p.X(st.r), y: clipped ? p.rect.y + 1 : p.Y(v), v: v, clipped: clipped };
      }
      function drawForcedPlot(which, ctx, w, h) {
        const P = PLOTS[which];
        const p = plotOf(which, ctx, w, h);
        p.axes({ yTicks: P.yTicks, yFormat: P.yFormat });
        const rc = p.rect, narrow = rc.w < 330;
        const lay = placer(ctx, { x: rc.x, y: rc.y, w: rc.w, h: rc.h }, P.cv.canvas);
        // isolation region and reference lines
        if (which === 'C') {
          p.vband(SQRT2, 4, { fill: RASim.alpha(C.green, 0.10) });
          p.hline(1, { color: C.ink, width: 1.2, dash: [5, 4] });
          p.vline(SQRT2, { color: COL.iso, width: 1.5, dash: [5, 4] });
          lay.block(p.X(SQRT2), rc.y + rc.h / 2, 2, rc.h / 2, 15);
          lay.block(rc.x + rc.w / 2, p.Y(1), rc.w / 2, 2, 15);
        }
        if (which === 'B') {
          p.hline(90, { color: COL.res, width: 1.2, dash: [5, 4] });
          lay.block(rc.x + rc.w / 2, p.Y(90), rc.w / 2, 2, 15);
        }
        if (which === 'A' || which === 'B') {
          p.vline(1, { color: COL.res, width: 1.3, dash: [5, 4] });
          lay.block(p.X(1), rc.y + rc.h / 2, 2, rc.h / 2, 15);
        }
        // family
        const fam = FAMILY.map(function (z) { return curveSamples(which, z, 320); });
        fam.forEach(function (s, i) {
          p.line(s.rs, s.ys, { color: RASim.alpha(COL.fam, 0.85), width: 1.3 });
          lay.path(s.rs.map(function (r, j) { return [p.X(r), p.Y(s.ys[j])]; }), 25, 'f' + i);
        });
        // current ζ
        const cur = curveSamples(which, st.zeta, 800);
        p.line(cur.rs, cur.ys, { color: COL.cur, width: 2.6 });
        lay.path(cur.rs.map(function (r, j) { return [p.X(r), p.Y(cur.ys[j])]; }), 300, 'cur');
        // operating point with drop lines
        const op = opPx(p, which);
        ctx.save();
        ctx.strokeStyle = COL.op; ctx.lineWidth = 1.2; ctx.setLineDash([3, 3]);
        ctx.beginPath(); ctx.moveTo(op.x, op.y); ctx.lineTo(op.x, rc.y + rc.h); ctx.moveTo(op.x, op.y); ctx.lineTo(rc.x, op.y); ctx.stroke();
        ctx.restore();
        lay.block(op.x, (op.y + rc.y + rc.h) / 2, 2, (rc.y + rc.h - op.y) / 2, 40);
        lay.block((rc.x + op.x) / 2, op.y, (op.x - rc.x) / 2, 2, 40);
        if (op.clipped) {
          ctx.save(); ctx.fillStyle = COL.op;
          ctx.beginPath(); ctx.moveTo(op.x, rc.y + 1); ctx.lineTo(op.x - 6, rc.y + 11); ctx.lineTo(op.x + 6, rc.y + 11); ctx.closePath(); ctx.fill();
          ctx.restore();
          op.y = rc.y + 7;
        } else p.point(st.r, op.v, { color: COL.op, r: 5.5, clip: false });
        lay.block(op.x, op.y, 6.5, 6.5, 1000);
        // labels: the operating point first, then the reference lines, then the family
        const name = which === 'A' ? (st.curve === 'unbalance' ? 'MX/m₀e' : 'MF') : which === 'B' ? 'φ' : 'TR';
        const val = which === 'B' ? fmt(op.v, 4) + '°' : isFinite(op.v) ? fmt(op.v, 4) : '∞';
        const ol = name + ' = ' + val;
        const ox = op.x, oy = op.y;
        lay.put(ol, B12, COL.op, [
          { x: ox + 10, y: oy - 12, align: 'left' }, { x: ox - 10, y: oy - 12, align: 'right' }, { x: ox + 11, y: oy, align: 'left' },
          { x: ox - 11, y: oy, align: 'right' }, { x: ox + 10, y: oy + 13, align: 'left' }, { x: ox - 10, y: oy + 13, align: 'right' },
          { x: ox + 10, y: oy - 28, align: 'left' }, { x: ox - 10, y: oy - 28, align: 'right' }, { x: ox, y: oy + 22, align: 'center' },
          { x: ox + 26, y: oy + 14, align: 'left', cost: 40 }, { x: ox - 26, y: oy + 14, align: 'right', cost: 40 },
          { x: ox + 40, y: oy + 30, align: 'left', cost: 60 }, { x: ox - 40, y: oy + 30, align: 'right', cost: 60 }],
          // the label belongs to the bold curve and has a white background: it may cross the curves
          { ignore: function (ow) { return ow === 'cur' || (typeof ow === 'string' && ow.charAt(0) === 'f'); } });
        if (which === 'A' || which === 'B') {
          lay.put('resonance', F11, COL.res, [{ x: p.X(1) + 4, y: rc.y + 10, align: 'left', cost: 0 }, { x: p.X(1) - 4, y: rc.y + 10, align: 'right', cost: 1 },
            { x: p.X(1) + 4, y: rc.y + rc.h - 10, align: 'left', cost: 3 }, { x: p.X(1) - 4, y: rc.y + rc.h - 10, align: 'right', cost: 3.5 }]);
        }
        if (which === 'C') {
          lay.put('r = √2', F11, COL.iso, [{ x: p.X(SQRT2) - 4, y: rc.y + 10, align: 'right', cost: 0 }, { x: p.X(SQRT2) + 4, y: rc.y + 10, align: 'left', cost: 1 },
            { x: p.X(SQRT2) - 4, y: rc.y + 28, align: 'right', cost: 2 }, { x: p.X(SQRT2) + 4, y: rc.y + 28, align: 'left', cost: 2.5 },
            { x: p.X(SQRT2) + 4, y: rc.y + rc.h - 10, align: 'left', cost: 4 }, { x: p.X(SQRT2) - 4, y: rc.y + rc.h - 10, align: 'right', cost: 4.5 }]);
          const ix = (p.X(SQRT2) + rc.x + rc.w) / 2;
          lay.put('isolation (TR < 1)', B12, COL.iso, [{ x: ix, y: rc.y + 0.4 * rc.h, align: 'center', cost: 0 }, { x: ix, y: rc.y + 0.25 * rc.h, align: 'center', cost: 0.5 },
            { x: ix, y: rc.y + 0.55 * rc.h, align: 'center', cost: 1 }, { x: ix, y: rc.y + 12, align: 'center', cost: 1.5 }]);
          lay.put('TR = 1', F11, C.ink, [{ x: rc.x + 4, y: p.Y(1) - 10, align: 'left', cost: 0 }, { x: rc.x + rc.w - 4, y: p.Y(1) - 10, align: 'right', cost: 1 },
            { x: rc.x + 4, y: p.Y(1) + 10, align: 'left', cost: 2 }]);
        }
        if (which === 'B') {
          lay.put('90°', F11, COL.res, [{ x: rc.x + rc.w - 4, y: p.Y(90) - 10, align: 'right', cost: 0 }, { x: rc.x + 4, y: p.Y(90) - 10, align: 'left', cost: 1 },
            { x: rc.x + rc.w - 4, y: p.Y(90) + 10, align: 'right', cost: 2 }]);
        }
        if (P.labelFamily !== 'none') {
          const rsC = [];
          for (let r = 0.3; r <= 3.85; r += 0.05) rsC.push(r);
          FAMILY.forEach(function (z, i) {
            const txt = (narrow ? '' : 'ζ=') + fmt(z, 3);
            const cands = [];
            rsC.forEach(function (r) {
              const y = P.f(r, z);
              if (!(y < P.ymax * 0.95) || !(y > P.ymax * 0.02)) return;
              cands.push({ x: p.X(r), y: p.Y(y), align: 'center', cost: Math.abs(r - prefR(which, z)) * 3 });
            });
            if (!cands.length) return;
            // the family member that equals the current ζ lies under the bold curve: label it in blue there
            const same = Math.abs(z - st.zeta) < 0.005;
            if (same) lay.put(txt, B12, COL.cur, cands, { ignore: 'cur', optional: P.labelFamily === 'all' ? null : 60 });
            else lay.put(txt, F11, C.muted, cands, { ignore: 'f' + i, optional: P.labelFamily === 'all' ? null : 60 });
          });
        }
        lay.flush();
        P.cv.canvas.vsOp = { x: op.x, y: op.y };
      }
      // preferred r for the family labels (spread along the curves)
      function prefR(which, z) {
        if (which === 'A') return z < 0.07 ? 1.06 : z < 0.15 ? 1.12 : z < 0.25 ? 1.2 : z < 0.4 ? 0.8 : z < 0.6 ? 0.6 : z < 0.8 ? 0.45 : 0.3;
        return z < 0.15 ? 1.25 : z < 0.4 ? 1.1 : z < 0.6 ? 2.6 : z < 0.8 ? 3.1 : 3.6;
      }
      cvA.draw = function (ctx, w, h) { drawForcedPlot('A', ctx, w, h); };
      cvB.draw = function (ctx, w, h) { drawForcedPlot('B', ctx, w, h); };
      cvC.draw = function (ctx, w, h) { drawForcedPlot('C', ctx, w, h); };
      ['A', 'B', 'C'].forEach(function (which) {
        const cv = PLOTS[which].cv;
        function near(x, y) {
          const p = plotOf(which, cv.ctx, cv.width, cv.height);
          const op = opPx(p, which);
          if (op.clipped) op.y = p.rect.y + 7;
          return Math.hypot(op.x - x, op.y - y) <= 14 ? p : null;
        }
        cv.onDrag({
          hit: function (x, y) { const p = near(x, y); return p ? { p: p } : null; },
          move: function (x, y, tok) {
            const r = Math.round(M.clamp(tok.p.invX(x), 0, 4) * 100) / 100;
            if (r !== st.r) { sR.set(r); st.r = sR.get(); update(true); }
          },
          cursor: function (x, y) { return near(x, y) ? 'grab' : null; }
        });
      });

      // ---------------------------------------------------------------- forced tab: animation + phasors
      cvAn.draw = function (ctx, w, h) {
        const th = PI * st.time;                     // display phase: one cycle per 2 s
        const ph = model.phase(st.r, st.zeta);
        const mb = { x: 0, y: 0, w: w, h: h };
        const fLen = M.clamp(0.13 * w, 28, 48);
        const L = mechLayout(mb, M.clamp(0.08 * w, 12, 26), fLen + 16, 46, 0.17);
        const u = L.A * Math.sin(th - ph);
        eqMarker(ctx, L, L.ytop - 8);
        const xm = drawMech(ctx, mb, L, u);
        const fv = Math.sin(th);
        const fy = L.ytop + L.Hb / 2;
        const fx0 = fv >= 0 ? xm + L.B : xm;            // the force acts on the block: push from its side
        if (Math.abs(fv) > 0.04) D.arrow(ctx, fx0, fy, fx0 + fLen * fv, fy, { color: COL.force, width: 2.4, head: Math.min(9, fLen * Math.abs(fv)) });
        const fxL = fv >= 0 ? Math.max(xm + L.B + fLen * fv, xm + L.B) + 4 : Math.min(xm + fLen * fv, xm) - 4;
        D.text(ctx, 'F', fxL, fy - 12, { align: fv >= 0 ? 'left' : 'right', font: B12, color: COL.force, bg: LBG });
        const X = model.MF(st.r, st.zeta) * st.F0 / st.k * 1000;
        const xTxt = 'X = ' + (isFinite(X) ? fmt(X, 4) + ' mm' : '∞ (resonance)') + (w < 200 ? '' : ' (not to scale)');
        const fTxt = 'F = F₀ sin ωt, F₀ = ' + fmt(st.F0, 3) + ' N';
        D.text(ctx, fTxt, 8, 12, { font: F12, color: COL.force });
        D.text(ctx, xTxt, 8, 29, { font: F12, color: COL.x });
      };
      cvPh.draw = function (ctx, w, h) {
        const th = PI * st.time;
        const ph = model.phase(st.r, st.zeta);
        const top = 22, bot = 24;
        const R = Math.max(18, Math.min(w / 2 - 30, (h - top - bot) / 2 - 16));
        const cx = w / 2, cy = top + (h - top - bot) / 2;
        ctx.save();
        ctx.strokeStyle = C.grid; ctx.lineWidth = 1;
        ctx.beginPath(); ctx.arc(cx, cy, R, 0, TWO_PI); ctx.stroke();
        ctx.strokeStyle = C.axis; ctx.setLineDash([4, 3]);
        ctx.beginPath(); ctx.moveTo(cx - R - 10, cy); ctx.lineTo(cx + R + 10, cy); ctx.stroke();
        ctx.restore();
        const aF = th - PI / 2, aX = th - PI / 2 - ph, rX = 0.72 * R;
        const tipF = [cx + R * Math.cos(aF), cy - R * Math.sin(aF)], tipX = [cx + rX * Math.cos(aX), cy - rX * Math.sin(aX)];
        // projections on the horizontal axis = F(t) and x(t)
        ctx.save();
        ctx.lineWidth = 1; ctx.setLineDash([2, 3]);
        ctx.strokeStyle = COL.force; ctx.beginPath(); ctx.moveTo(tipF[0], tipF[1]); ctx.lineTo(tipF[0], cy); ctx.stroke();
        ctx.strokeStyle = COL.x; ctx.beginPath(); ctx.moveTo(tipX[0], tipX[1]); ctx.lineTo(tipX[0], cy); ctx.stroke();
        ctx.restore();
        if (ph > 0.02) D.angleArc(ctx, cx, cy, Math.min(0.38 * R, 22), aX, aF, { color: C.purple, width: 1.6 });
        D.arrow(ctx, cx, cy, tipF[0], tipF[1], { color: COL.force, width: 2.4, head: 9 });
        D.arrow(ctx, cx, cy, tipX[0], tipX[1], { color: COL.x, width: 2.4, head: 9 });
        ctx.save();
        ctx.fillStyle = COL.force; ctx.beginPath(); ctx.arc(tipF[0], cy, 3, 0, TWO_PI); ctx.fill();
        ctx.fillStyle = COL.x; ctx.beginPath(); ctx.arc(tipX[0], cy, 3, 0, TWO_PI); ctx.fill();
        ctx.restore();
        // labels beyond the tips; φ outside the circle at the middle of the angle
        D.text(ctx, 'F₀', cx + (R + 12) * Math.cos(aF), cy - (R + 12) * Math.sin(aF), { align: 'center', font: B12, color: COL.force, bg: LBG });
        const off = ph < 0.35 ? -0.35 : 0;              // nearly in phase: move the X label clockwise off the F phasor
        D.text(ctx, 'X', cx + (rX + 12) * Math.cos(aX + off), cy - (rX + 12) * Math.sin(aX + off), { align: 'center', font: B12, color: COL.x, bg: LBG });
        if (ph > 0.26) {                               // small angles: the arc alone (the value is written below)
          const am = aF - ph / 2;
          D.text(ctx, 'φ', cx + (R + 12) * Math.cos(am), cy - (R + 12) * Math.sin(am), { align: 'center', font: B12, color: C.purple, bg: LBG });
        }
        D.text(ctx, 'phasors, rotating ↺', w / 2, 11, { align: 'center', font: F11, color: C.muted });
        D.text(ctx, 'φ = ' + fmt(M.deg(ph), 4) + '°: x lags F', w / 2, h - 11, { align: 'center', font: B12, color: C.purple });
      };

      // ---------------------------------------------------------------- read-outs
      const KIND_TAG = { undamped: ['UNDAMPED', 'bad'], under: ['UNDERDAMPED', 'warn'], critical: ['CRITICALLY DAMPED', 'ok'], over: ['OVERDAMPED', ''] };
      function unit(v, u, sig) { return isFinite(v) ? fmt(v, sig || 4) + (u ? ' ' + u : '') : fmt(v, sig || 4); }
      let legKeyF = '', legKeyD = '';
      function update(fromDrag) {
        wrapTime();
        const n = model.natural(st.m, st.k), d = model.damping(st.m, st.k, st.zeta);
        const kt = KIND_TAG[d.kind];
        tag.set(kt[0], kt[1]);
        tWn.set(unit(n.wn, 'rad/s'));
        tFn.set(unit(n.fn, 'Hz'));
        tCc.set(unit(n.cc, 'N·s/m'));
        tC.set(unit(d.c, 'N·s/m'));
        tTn.set(unit(n.Tn, 's'));
        tDst.set(unit(1000 * n.dst, 'mm'));
        const free = st.tab === 'free';
        freeBox.style.display = free ? '' : 'none';
        forcedBox.style.display = free ? 'none' : '';
        gF.parentNode.style.display = free ? '' : 'none';
        gD.parentNode.style.display = free ? 'none' : '';
        statsF.style.display = free ? '' : 'none';
        statsD.style.display = free ? 'none' : '';
        if (free) {
          tWd.set(unit(d.wd, 'rad/s'));
          tTd.set(unit(d.Td, 's'));
          tDelta.set(fmt(d.delta, 4));
          tRatio.set(fmt(d.ratio, 4));
          tHalf.set(isFinite(d.halfCycles) ? fmt(d.halfCycles, 4) + ' cycles' : d.kind === 'undamped' ? '∞ (no decay)' : '—');
          const fd = freeData();
          const items = [{ color: COL.x, label: 'x(t)' }];
          if (fd.kind === 'under') {
            items.push({ color: COL.env, dash: true, label: 'envelope ±X·e<sup>−ζω<sub>n</sub>t</sup>' });
            items.push({ color: COL.peak, label: 'positive peaks x<sub>1</sub>, x<sub>2</sub>, x<sub>3</sub> …' });
          }
          items.push({ color: COL.cursor, label: 'time cursor (drag it)' });
          const k1 = JSON.stringify(items);
          if (k1 !== legKeyF) { legFree.set(items); legKeyF = k1; }
          cvFA.redraw(); cvFX.redraw();
        } else {
          const r = st.r, z = st.zeta;
          const w = r * n.wn;
          tW.set(unit(w, 'rad/s'));
          tRpm.set(unit(60 * w / TWO_PI, 'rpm'));
          const mf = model.MF(r, z), tr = model.TR(r, z);
          tMF.set(isFinite(mf) ? fmt(mf, 4) : '∞ (resonance)');
          tMF.setState(isFinite(mf) ? (mf > 1 ? 'warn' : '') : 'bad');
          const X = mf * st.F0 / st.k * 1000;
          tX.set(isFinite(X) ? fmt(X, 4) + ' mm' : '∞');
          tPh.set(fmt(M.deg(model.phase(r, z)), 4) + '°');
          tTR.set(isFinite(tr) ? fmt(tr, 4) : '∞ (resonance)');
          tFT.set(isFinite(tr) ? fmt(tr * st.F0, 4) + ' N' : '∞');
          if (r > SQRT2 && tr < 1) { tIso.set(fmt((1 - tr) * 100, 3) + ' % isolated'); tIso.setState('ok'); }
          else { tIso.set('no isolation — TR ≥ 1'); tIso.setState('warn'); }
          const pk = model.peakMF(z);
          tPeak.set(pk ? 'MF<sub>max</sub> = ' + (isFinite(pk.MF) ? fmt(pk.MF, 3) : '∞') + ' at r = ' + fmtFixed(pk.r, 3) : 'no peak (ζ ≥ 0.707)');
          const items = [
            { color: COL.cur, label: 'current ζ = ' + fmt(z, 3) },
            { color: COL.fam, label: 'ζ = 0.05, 0.1, 0.2, 0.3, 0.5, 0.707, 1' },
            { color: COL.op, label: 'operating point r = ' + fmtFixed(r, 3) + ' (drag it)' },
            { color: COL.res, dash: true, label: 'resonance r = 1' },
            { color: RASim.alpha(COL.iso, 0.6), label: 'isolation r &gt; √2' }];
          const k2 = JSON.stringify(items);
          if (k2 !== legKeyD) { legForced.set(items); legKeyD = k2; }
          if (fromDrag) { cvA.drawNow(); cvB.drawNow(); cvC.drawNow(); } else { cvA.redraw(); cvB.redraw(); cvC.redraw(); }
          cvAn.redraw(); cvPh.redraw();
        }
      }

      function getState() {
        return { tab: st.tab, m: st.m, k: st.k, zeta: st.zeta, x0: st.x0, v0: st.v0, speed: st.speed, r: st.r, F0: st.F0,
          curve: st.curve, playing: anim.running, time: st.time };
      }
      function setState(s) {
        s = s || {};
        if (s.tab != null) setTab(String(s.tab));
        if (s.m != null) { sM.set(Number(s.m)); st.m = sM.get(); }
        if (s.k != null) { sK.set(Number(s.k)); st.k = sK.get(); }
        if (s.zeta != null) { sZ.set(Number(s.zeta)); st.zeta = sZ.get(); }
        if (s.x0 != null) { sX0.set(Number(s.x0)); st.x0 = sX0.get(); }
        if (s.v0 != null) { sV0.set(Number(s.v0)); st.v0 = sV0.get(); }
        if (s.speed != null) { st.speed = normSpeed(s.speed); segSpeed.set(st.speed); }
        if (s.r != null) { sR.set(Number(s.r)); st.r = sR.get(); }
        if (s.F0 != null) { sF0.set(Number(s.F0)); st.F0 = sF0.get(); }
        if (s.curve != null) { st.curve = s.curve === 'unbalance' ? 'unbalance' : 'force'; selCurve.set(st.curve); }
        if (s.playing != null) setPlaying(!!s.playing);
        if (s.time != null && isFinite(Number(s.time))) st.time = Math.max(0, Number(s.time));
        update();
      }

      setPlaying(false);
      update();
      return { getState: getState, setState: setState };
    }
  });
})();
