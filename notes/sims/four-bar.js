// four-bar — "Four-bar linkage and Grashof's law"
// Sits beside notes/src/05_mechanical.html, "B2.2 Kinematics and Dynamics of Machines":
// "Mechanisms basics" (Grashof's law, inversions, transmission angle, toggle, quick return) and
// "Displacement, velocity and acceleration analysis" (instantaneous centres, Kennedy's theorem).
//
// SIGN CONVENTIONS
//   Fixed pivots O₂ = (0, 0) and O₄ = (d, 0); x to the right (direction O₂→O₄), y up.
//   a = O₂A (input, "crank"), b = AB (coupler), c = O₄B (output, "rocker"), d = O₂O₄ (frame).
//   θ₂, θ₃, θ₄ = angles of O₂→A, A→B, O₄→B, counter-clockwise positive from +x.
//   A = (a cos θ₂, a sin θ₂);  f = |O₄ − A| = √(a² + d² − 2ad cos θ₂); closes iff |b − c| ≤ f ≤ b + c and f > 0.
//   e = (O₄ − A)/f, n = (−e_y, e_x), α = (b² − c² + f²)/(2f), h = √max(0, b² − α²),
//   B = A + α e + mode·h n: mode = +1 OPEN (B left of the directed line A→O₄), −1 CROSSED.
//   θ₃ = atan2(B_y − A_y, B_x − A_x), θ₄ = atan2(B_y, B_x − d).
//   μ = angle ABO₄ ∈ [0°, 180°], cos μ = (b² + c² − f²)/(2bc); "poor" when μ < 40° or μ > 140°.
//   ω₄/ω₂ = a sin(θ₂ − θ₃)/(c sin(θ₄ − θ₃)); ω₃/ω₂ = a sin(θ₄ − θ₂)/(b sin(θ₃ − θ₄)); MA = T₄/T₂ = ω₂/ω₄.
//   I₂₄ = line AB ∩ x-axis (ω₄/ω₂ = x_I/(x_I − d)); I₁₃ = line O₂A ∩ line O₄B.
//   Coupler point P = A + (u·b) ê + v n̂, ê = (B − A)/b, n̂ = (−ê_y, ê_x): u is a fraction of b, v a length (left +).
//   Classification compares lengths in integer tenths (Math.round(10x)).
//   Model angles are radians unless the name ends in Deg; the UI and the state use degrees; θ₂ in the state is in [0, 360).
//
// STATE KEYS (getState / setState)
//   a, b, c, d, theta2 (degrees), mode ('open' | 'crossed'), playing, speed (°/s), u, v,
//   showCoupler, showPaths, showIC, showMu.
//   setState snaps θ₂ to the nearest valid input angle when the requested one cannot be reached.
//
// STAT KEYS:   range, theta3, theta4, mu, muRange, w4, w3, MA, swing, timeRatio, I24
// TAG KEYS:    type
// MATRIX KEYS: (none)
// DATA KEYS:   grashof (Grashof line), status (status line), kennedy (live I₂₄ formula, when IC are shown)
// CANVAS KEYS: mechanism, plot
// CONTROL KEYS: preset, a, b, c, d, theta2, mode, play, reset, speed, u, v, showCoupler, showPaths, showIC, showMu
(function () {
  'use strict';
  const RASim = globalThis.RASim;
  const M = RASim.math;
  const D2R = Math.PI / 180, R2D = 180 / Math.PI;

  // ================================================================ model (pure)
  function tenths(x) { return Math.round(10 * x); }
  function acosDeg(x) { return Math.acos(Math.max(-1, Math.min(1, x))) * R2D; }
  function modeSign(mode) { return (mode === -1 || mode === 'crossed') ? -1 : 1; }

  const NAMES = ['a', 'b', 'c', 'd'];
  function classify(a, b, c, d) {
    const L = [a, b, c, d], T = L.map(tenths);
    const idx = [0, 1, 2, 3].sort(function (i, j) { return T[i] - T[j] || i - j; });
    const iS = idx[0], iL = idx[3], iP = idx[1], iQ = idx[2];
    const S = T[iS], Lg = T[iL], P = T[iP], Q = T[iQ];
    const out = {
      assemblable: Lg < S + P + Q,
      grashof: null, type: 'none',
      shortest: S < T[idx[1]] ? NAMES[iS] : null,
      s: L[iS], l: L[iL], sumSL: (S + Lg) / 10, sumPQ: (P + Q) / 10,
      // extras for the read-out line
      p: L[iP], q: L[iQ], sumSPQ: (S + P + Q) / 10,
      links: { s: NAMES[iS], l: NAMES[iL], p: NAMES[iP], q: NAMES[iQ] },
      label: ''
    };
    if (!out.assemblable) {
      out.label = 'Cannot be assembled (l ≥ s + p + q)';
      return out;
    }
    if (S + Lg < P + Q) {
      out.grashof = 'grashof';
      out.type = { a: 'crank-rocker', b: 'double-rocker', c: 'rocker-crank', d: 'double-crank' }[out.shortest];
      out.label = { 'crank-rocker': 'Crank–rocker', 'double-rocker': 'Double-rocker', 'rocker-crank': 'Rocker–crank',
        'double-crank': 'Double-crank (drag link)' }[out.type] + ' (Grashof: s + l < p + q)';
    } else if (S + Lg === P + Q) {
      out.grashof = 'change-point'; out.type = 'change-point';
      out.label = 'Change-point (s + l = p + q)';
    } else {
      out.grashof = 'non-grashof'; out.type = 'triple-rocker';
      out.label = 'Triple-rocker (non-Grashof: s + l > p + q)';
    }
    return out;
  }

  const K_EPS = 1e-9;
  function kLimits(a, b, c, d) {
    return { kmax: (a * a + d * d - (b - c) * (b - c)) / (2 * a * d), kmin: (a * a + d * d - (b + c) * (b + c)) / (2 * a * d) };
  }
  function inputRange(a, b, c, d) {
    const k = kLimits(a, b, c, d);
    if (k.kmax < -1 - K_EPS || k.kmin > 1 + K_EPS) return { full: false, rangesDeg: [] };
    const lo = k.kmax >= 1 - K_EPS ? 0 : acosDeg(k.kmax);
    const hi = k.kmin <= -1 + K_EPS ? 180 : acosDeg(k.kmin);
    if (lo === 0 && hi === 180) return { full: true, rangesDeg: [[0, 360]] };
    if (lo === 0) return { full: false, rangesDeg: [[-hi, hi]] };
    if (hi === 180) return { full: false, rangesDeg: [[lo, 360 - lo]] };
    return { full: false, rangesDeg: [[lo, hi], [-hi, -lo]] };
  }

  function solve(a, b, c, d, theta2, mode) {
    const m = modeSign(mode);
    const Ax = a * Math.cos(theta2), Ay = a * Math.sin(theta2);
    const gx = d - Ax, gy = -Ay;
    const f = Math.hypot(gx, gy);
    const tol = 1e-9 * (a + b + c + d);
    if (!(f > 1e-12 * (a + d))) return null;
    if (f > b + c + tol || f < Math.abs(b - c) - tol) return null;
    const ex = gx / f, ey = gy / f, nx = -ey, ny = ex;
    const al = (b * b - c * c + f * f) / (2 * f);
    const h = Math.sqrt(Math.max(0, b * b - al * al));
    const Bx = Ax + al * ex + m * h * nx, By = Ay + al * ey + m * h * ny;
    const t3 = Math.atan2(By - Ay, Bx - Ax), t4 = Math.atan2(By, Bx - d);
    const mu = Math.acos(Math.max(-1, Math.min(1, (b * b + c * c - f * f) / (2 * b * c))));
    let den4 = c * Math.sin(t4 - t3), den3 = b * Math.sin(t3 - t4);
    if (Math.abs(Math.sin(t4 - t3)) < 1e-12) { den4 = 0; den3 = 0; }      // toggle: coupler and output collinear
    const w4 = a * Math.sin(theta2 - t3) / den4;
    const w3 = a * Math.sin(t4 - theta2) / den3;
    // I₂₄: line AB meets the x-axis
    let I24 = null;
    const dyAB = By - Ay;
    if (Math.abs(dyAB) > 1e-12 * b) I24 = [Ax - Ay * (Bx - Ax) / dyAB, 0];
    // I₁₃: line O₂A meets line O₄B
    let I13 = null;
    const ux = Math.cos(theta2), uy = Math.sin(theta2), wx = (Bx - d) / c, wy = By / c;
    const det = uy * wx - ux * wy;
    if (Math.abs(det) > 1e-12) { const s = -d * wy / det; I13 = [s * ux, s * uy]; }
    return { A: [Ax, Ay], B: [Bx, By], theta3: t3, theta4: t4, mu: mu, w3: w3, w4: w4, I24: I24, I13: I13 };
  }

  function couplerPoint(A, B, u, v) {
    const dx = B[0] - A[0], dy = B[1] - A[1], L = Math.hypot(dx, dy);
    const ex = dx / L, ey = dy / L;
    return [A[0] + u * L * ex - v * ey, A[1] + u * L * ey + v * ex];
  }

  function rockerLimits(a, b, c, d) {
    if (classify(a, b, c, d).type !== 'crank-rocker') return null;
    const be1 = acosDeg((c * c + d * d - (b + a) * (b + a)) / (2 * c * d));
    const ps1 = acosDeg(((a + b) * (a + b) + d * d - c * c) / (2 * (a + b) * d));
    const be2 = acosDeg((c * c + d * d - (b - a) * (b - a)) / (2 * c * d));
    const ps2 = acosDeg(((b - a) * (b - a) + d * d - c * c) / (2 * (b - a) * d));
    const t4e = 180 - be1, t4f = 180 - be2;
    const one = 180 + (ps2 - ps1), other = 180 - (ps2 - ps1);
    const fw = Math.max(one, other), rt = Math.min(one, other);
    return { theta4MinDeg: Math.min(t4e, t4f), theta4MaxDeg: Math.max(t4e, t4f), swingDeg: Math.abs(be1 - be2),
      theta2ExtendedDeg: ps1, theta2FoldedDeg: 180 + ps2, forwardDeg: fw, returnDeg: rt, timeRatio: fw / rt };
  }

  function transmissionRange(a, b, c, d) {
    const k = kLimits(a, b, c, d);
    if (inputRange(a, b, c, d).rangesDeg.length === 0) return { minDeg: NaN, maxDeg: NaN };
    const cmax = Math.min(1, k.kmax), cmin = Math.max(-1, k.kmin);
    const f1 = Math.sqrt(Math.max(0, a * a + d * d - 2 * a * d * cmax));
    const f2 = Math.sqrt(Math.max(0, a * a + d * d - 2 * a * d * cmin));
    const mu = function (f) { return acosDeg((b * b + c * c - f * f) / (2 * b * c)); };
    return { minDeg: mu(f1), maxDeg: mu(f2) };
  }

  function couplerCurve(a, b, c, d, mode, u, v, n) {
    n = n || 360;
    const out = [];
    inputRange(a, b, c, d).rangesDeg.forEach(function (r) {
      const span = r[1] - r[0];
      const N = Math.max(2, Math.ceil(n * span / 360));
      const line = [];
      for (let i = 0; i <= N; i++) {
        const s = solve(a, b, c, d, (r[0] + span * i / N) * D2R, mode);
        if (s) line.push(couplerPoint(s.A, s.B, u, v));
      }
      out.push(line);
    });
    return out;
  }

  const model = { classify: classify, inputRange: inputRange, solve: solve, couplerPoint: couplerPoint,
    rockerLimits: rockerLimits, transmissionRange: transmissionRange, couplerCurve: couplerCurve };

  // ================================================================ user interface
  const DEFAULTS = { a: 2, b: 6, c: 5, d: 6, theta2: 60, mode: 'open', playing: false, speed: 60, u: 0.5, v: 2,
    showCoupler: true, showPaths: true, showIC: false, showMu: true };
  const PRESETS = [
    { value: 'crank-rocker', label: 'Crank–rocker (2, 6, 5, 6)', L: [2, 6, 5, 6] },
    { value: 'poor', label: 'Crank–rocker with poor transmission (2, 7, 5, 6)', L: [2, 7, 5, 6] },
    { value: 'drag-link', label: 'Double-crank / drag link (4, 5, 6, 2)', L: [4, 5, 6, 2] },
    { value: 'double-rocker', label: 'Double-rocker, Grashof (5, 2, 6, 4)', L: [5, 2, 6, 4] },
    { value: 'triple-rocker', label: 'Triple-rocker, non-Grashof (4, 6, 5, 8)', L: [4, 6, 5, 8] },
    { value: 'parallelogram', label: 'Parallelogram, change-point (3, 6, 3, 6)', L: [3, 6, 3, 6] },
    { value: 'rocker-crank', label: 'Rocker–crank (5, 7, 2, 6)', L: [5, 7, 2, 6] }
  ];
  const CUSTOM = 'custom';
  const TAGS = {
    'crank-rocker': ['CRANK–ROCKER', 'ok'], 'double-crank': ['DOUBLE-CRANK (DRAG LINK)', 'ok'],
    'double-rocker': ['DOUBLE-ROCKER', 'warn'], 'rocker-crank': ['ROCKER–CRANK', 'warn'],
    'change-point': ['CHANGE-POINT', 'warn'], 'triple-rocker': ['TRIPLE-ROCKER (NON-GRASHOF)', 'bad'],
    'none': ['CANNOT BE ASSEMBLED', 'bad']
  };
  const MU_LO = 40, MU_HI = 140;
  const FAR_VIEWS = 3;   // an IC further than this many view-widths from the view centre is not drawn

  RASim.register('four-bar', {
    title: "Four-bar linkage and Grashof's law",
    model: model,
    mount: function (api) {
      const ui = RASim.ui, C = RASim.colors, D = RASim.draw;
      const fmt = RASim.fmt, fmtFixed = RASim.fmtFixed;
      const SC = '.sim[data-sim="four-bar"]';
      RASim.css('four-bar',
        SC + ' .fb-nt { text-transform: none; letter-spacing: 0; font-size: 11.5px; }\n' +
        SC + ' .fb-case { text-transform: none; letter-spacing: 0; }\n' +
        SC + ' .fb-line { margin: 2px 0 0; font-size: 12.5px; line-height: 1.45; color: #1a1a1a; }\n' +
        SC + ' .fb-select select { width: 100%; }\n' +
        SC + ' .fb-select { display: flex; width: 100%; }\n');
      function lab(html) { return '<span class="fb-nt">' + html + '</span>'; }

      const st = Object.assign({}, DEFAULTS);
      let dir = 1;            // sweep direction while playing a rocking input
      let snapMsg = '';       // set when the last requested θ₂ was out of range

      ui.note(api.root, 'Pick a preset or set the four lengths, then press Play or drag the pin A: watch the Grashof test, the motion, the transmission angle μ and the velocity ratio.');
      const sp = ui.split(api.root, { ratio: 1.5 });

      // ---- main column ----
      const cv = RASim.canvas(sp.main, { aspect: 1.5, minHeight: 250, maxHeight: 460, key: 'mechanism' });
      ui.legend(sp.main, [
        { color: C.maroon, label: 'crank a' }, { color: C.blue, label: 'coupler b' }, { color: C.green, label: 'rocker c' },
        { color: C.orange, label: 'coupler curve of P' }, { color: C.faint, label: 'paths of A and B' },
        { color: C.teal, label: 'μ (red when poor)' }, { color: C.purple, label: 'instantaneous centres', dash: true }
      ]);
      const statusEl = ui.note(sp.main, '');
      statusEl.setAttribute('data-key', 'status');
      const kenEl = ui.note(sp.main, '');
      kenEl.setAttribute('data-key', 'kennedy');
      const cvPlot = RASim.canvas(sp.main, { aspect: 2.2, minHeight: 200, maxHeight: 300, key: 'plot' });
      ui.legend(sp.main, [
        { color: C.green, label: 'θ₄ (°)' }, { color: C.red, label: 'μ (°)' },
        { color: C.lightGreen, label: 'good transmission 40°–140°' },
        { color: C.green, label: 'rocker limits (crank–rocker)', dash: true }
      ]);

      // ---- side column ----
      const rowTag = ui.row(sp.side);
      const tag = ui.tag(rowTag, { text: '', key: 'type' });
      const grEl = ui.el('p', { 'class': 'fb-line' }, sp.side);
      grEl.setAttribute('data-key', 'grashof');

      const gL = ui.group(sp.side, 'Links');
      const selP = ui.select(gL, { label: 'Preset', key: 'preset', value: PRESETS[0].value,
        options: PRESETS.map(function (p) { return { value: p.value, label: p.label }; }).concat([{ value: CUSTOM, label: 'Custom lengths' }]),
        onChange: function (v) {
          const p = PRESETS.filter(function (q) { return q.value === v; })[0];
          if (p) setState({ a: p.L[0], b: p.L[1], c: p.L[2], d: p.L[3] });
        } });
      selP.el.classList.add('fb-select');
      function lenSlider(label, key) {
        return ui.slider(gL, { label: label, min: 1, max: 10, step: 0.1, value: st[key], key: key,
          onInput: function (v) { st[key] = v; snapTheta(st.theta2); update(); } });
      }
      const sA = lenSlider('crank <i>a</i> (O₂A)', 'a');
      const sB = lenSlider('coupler <i>b</i> (AB)', 'b');
      const sC = lenSlider('rocker <i>c</i> (O₄B)', 'c');
      const sD = lenSlider('frame <i>d</i> (O₂O₄)', 'd');

      const gM = ui.group(sp.side, 'Input <span class="fb-case">θ₂</span> and motion');
      const sT = ui.slider(gM, { label: 'θ₂', min: 0, max: 360, step: 0.5, value: st.theta2, unit: '°', key: 'theta2',
        onInput: function (v) { snapTheta(v); update(); } });
      const seg = ui.segmented(gM, { label: 'Configuration', key: 'mode', value: st.mode,
        options: [{ value: 'open', label: 'Open' }, { value: 'crossed', label: 'Crossed' }],
        onChange: function (v) { st.mode = v === 'crossed' ? 'crossed' : 'open'; update(); } });
      const rowBtn = ui.row(gM);
      const bPlay = ui.button(rowBtn, { label: 'Play', primary: true, key: 'play', title: 'Play / pause the motion',
        onClick: function () { setPlaying(!anim.running); } });
      ui.button(rowBtn, { label: 'Reset', key: 'reset', title: 'Restore every default',
        onClick: function () { setState(Object.assign({}, DEFAULTS)); } });
      const sSpeed = ui.slider(gM, { label: 'speed', min: 10, max: 180, step: 5, value: st.speed, unit: '°/s', key: 'speed',
        onInput: function (v) { st.speed = v; } });

      const gP = ui.group(sp.side, 'Coupler point P');
      const sU = ui.slider(gP, { label: '<i>u</i> (fraction of <i>b</i> along AB)', min: 0, max: 1.5, step: 0.05, value: st.u, key: 'u',
        onInput: function (v) { st.u = v; update(); } });
      const sV = ui.slider(gP, { label: '<i>v</i> (offset, left +)', min: -5, max: 5, step: 0.1, value: st.v, key: 'v',
        onInput: function (v) { st.v = v; update(); } });

      // Core quirk workaround: after a track drag the number box ignores a typed value equal to the last typed text
      // (its "unchanged" check compares with the last committed text). Re-apply such a value here. The core's own
      // handler runs first; when it did commit, the box already shows the (possibly snapped) current value rounded
      // to the step, so a value within half a step of the current one is left alone (otherwise a snapped θ₂ would be
      // replaced by its rounded display text and the snap message lost).
      function numBoxFix(sl, apply, step) {
        const box = sl.el.querySelector('input.sim-num');
        function chk() {
          const v = parseFloat(box.value);
          if (!isFinite(v)) return;
          const c = M.clamp(v, Number(box.min), Number(box.max));
          if (Math.abs(c - sl.get()) > step / 2 + 1e-9) { sl.set(c); apply(sl.get()); }
        }
        box.addEventListener('change', chk);
        box.addEventListener('keydown', function (e) { if (e.key === 'Enter') chk(); });
      }
      [[sA, 'a'], [sB, 'b'], [sC, 'c'], [sD, 'd']].forEach(function (q) {
        numBoxFix(q[0], function (v) { st[q[1]] = v; snapTheta(st.theta2); update(); }, 0.1);
      });
      numBoxFix(sT, function (v) { snapTheta(v); update(); }, 0.5);
      numBoxFix(sSpeed, function (v) { st.speed = v; }, 5);
      numBoxFix(sU, function (v) { st.u = v; update(); }, 0.05);
      numBoxFix(sV, function (v) { st.v = v; update(); }, 0.1);

      const gS = ui.group(sp.side, 'Show');
      const rowS = ui.row(gS);
      function chk(label, key) {
        return ui.checkbox(rowS, { label: label, checked: st[key], key: key, onChange: function (v) { st[key] = v; update(); } });
      }
      const cCoup = chk('Coupler curve', 'showCoupler');
      const cPath = chk('Paths of A and B', 'showPaths');
      const cIC = chk('Instantaneous centres', 'showIC');
      const cMu = chk('Transmission angle', 'showMu');

      const stats = ui.stats(sp.side);
      const tRange = ui.stat(stats, { label: lab('input range of θ₂'), key: 'range' });
      tRange.el.style.gridColumn = '1 / -1';
      const tMuR = ui.stat(stats, { label: lab('μ min … max'), key: 'muRange', title: 'over all valid θ₂' });
      tMuR.el.style.gridColumn = 'span 2';
      const tT3 = ui.stat(stats, { label: lab('θ₃'), key: 'theta3' });
      const tT4 = ui.stat(stats, { label: lab('θ₄'), key: 'theta4' });
      const tMu = ui.stat(stats, { label: lab('μ (transmission)'), key: 'mu' });
      const tW4 = ui.stat(stats, { label: lab('ω₄/ω₂'), key: 'w4' });
      const tW3 = ui.stat(stats, { label: lab('ω₃/ω₂'), key: 'w3' });
      const tMA = ui.stat(stats, { label: lab('MA = T₄/T₂'), key: 'MA', title: 'ideal mechanical advantage = ω₂/ω₄' });
      const tI24 = ui.stat(stats, { label: lab('x of I₂₄'), key: 'I24' });
      const tSwing = ui.stat(stats, { label: lab('rocker swing'), key: 'swing' });
      const tTR = ui.stat(stats, { label: lab('time ratio'), key: 'timeRatio', title: 'quick-return ratio = forward / return crank angle' });

      ui.hints(api.root, [
        'Grashof test: s + l ≤ p + q. With the default lengths the shortest link is the crank → crank-rocker. Choose the drag-link preset (the shortest link is the frame) → both cranks rotate. Make the coupler the shortest → double-rocker.',
        'Triple-rocker preset: s + l &gt; p + q — nothing can rotate fully; press Play and watch the input swing between its two toggle positions.',
        'Watch μ: the "poor transmission" preset dips to 34° — below about 40° the coupler pushes the rocker badly.',
        'Crank-rocker: at the rocker\'s two extreme positions the crank and the coupler are collinear; the two strokes take different crank angles, so one stroke is quicker (time ratio &gt; 1).',
        'Turn on Instantaneous centres: I₂₄ lies where the coupler line meets the frame line, and ω₄/ω₂ = I₂₄O₂ / I₂₄O₄ (Kennedy\'s theorem).'
      ]);

      // ---- helpers ----
      function fmtDeg(x, dec) { return fmtFixed(x, dec == null ? 2 : dec) + '°'; }
      function norm360(x) { x = x % 360; if (x < 0) x += 360; return x >= 360 ? 0 : x + 0; }
      // unwrapped angle t (≡ th mod 360) inside range r, or null
      function inRange(th, r) {
        const e = 1e-9;
        for (let k = -1; k <= 1; k++) { const t = th + 360 * k; if (t >= r[0] - e && t <= r[1] + e) return t; }
        return null;
      }
      function circDist(x, y) { const dd = Math.abs(norm360(x) - norm360(y)); return Math.min(dd, 360 - dd); }

      // ---- geometry cache (lengths, mode, u, v) ----
      let G = null;
      function geom() {
        const key = [st.a, st.b, st.c, st.d, st.mode, st.u, st.v].join(',');
        if (G && G.key === key) return G;
        const a = st.a, b = st.b, c = st.c, d = st.d, m = st.mode;
        const g = { key: key, cl: classify(a, b, c, d) };
        g.ir = inputRange(a, b, c, d);
        g.ok = g.cl.assemblable && g.ir.rangesDeg.length > 0;
        if (g.ok) {
          g.tr = transmissionRange(a, b, c, d);
          g.rl = rockerLimits(a, b, c, d);
          g.curve = couplerCurve(a, b, c, d, m, st.u, st.v, 360);
          g.runs = g.ir.rangesDeg.map(function (r) {     // samples for paths and the plot
            const span = r[1] - r[0], N = Math.max(2, Math.ceil(span));
            const pts = [];
            for (let i = 0; i <= N; i++) {
              const t = r[0] + span * i / N;
              const s = solve(a, b, c, d, t * D2R, m);
              if (s) pts.push({ t: t, s: s });
            }
            return pts;
          });
          let x0 = Math.min(0, d), x1 = Math.max(0, d), y0 = 0, y1 = 0;
          function inc(q) { if (q[0] < x0) x0 = q[0]; if (q[0] > x1) x1 = q[0]; if (q[1] < y0) y0 = q[1]; if (q[1] > y1) y1 = q[1]; }
          g.runs.forEach(function (run) { run.forEach(function (q) { inc(q.s.A); inc(q.s.B); }); });
          g.curve.forEach(function (line) { line.forEach(inc); });
          const W = x1 - x0, H = y1 - y0, mg = 0.12;
          g.box = { x0: x0 - mg * W, x1: x1 + mg * W, y0: y0 - mg * H, y1: y1 + mg * H };
          // plot data θ₄(θ₂), μ(θ₂) on a 0…360 axis with breaks at wraps
          const xs = [], y4 = [], ym = [];
          let all0to180 = true;
          g.runs.forEach(function (run) {
            let px = null, p4 = null;
            run.forEach(function (q) {
              const x = q.t === 360 ? 360 : norm360(q.t), t4 = q.s.theta4 * R2D;
              if (px !== null && (Math.abs(x - px) > 90 || Math.abs(t4 - p4) > 180)) { xs.push(NaN); y4.push(NaN); ym.push(NaN); }
              xs.push(x); y4.push(t4); ym.push(q.s.mu * R2D);
              if (t4 < -1e-6) all0to180 = false;
              px = x; p4 = t4;
            });
            xs.push(NaN); y4.push(NaN); ym.push(NaN);
          });
          g.plot = { xs: xs, y4: y4, ym: ym, ymin: all0to180 ? 0 : -180 };
        }
        G = g;
        return g;
      }

      // θ₂ (degrees) → valid θ₂ in [0, 360); sets snapMsg
      function snapTheta(req) {
        req = Number(req);
        const g = geom();
        snapMsg = '';
        let th = norm360(req === 360 ? 0 : req);
        if (req === 360) th = 0;
        if (g.ok && !g.ir.full && !g.ir.rangesDeg.some(function (r) { return inRange(th, r) !== null; })) {
          let best = null, bd = Infinity;
          g.ir.rangesDeg.forEach(function (r) {
            [r[0], r[1]].forEach(function (e) { const dd = circDist(th, e); if (dd < bd) { bd = dd; best = e; } });
          });
          // name the snapped angle also as it appears in the allowed ranges (e.g. 230.2° = −129.8°)
          const alias = Math.abs(norm360(best) - best) > 1e-9 ? ' (= ' + fmtDeg(best, 1) + ')' : '';
          snapMsg = 'θ₂ = ' + fmt(Math.round(req * 10) / 10, 6) + '° cannot be reached; snapped to the nearest valid angle ' +
            fmtDeg(norm360(best), 1) + alias + '.';
          th = norm360(best);
        }
        st.theta2 = th;
        sT.set(th);
      }
      // the solution shown; at f = 0 (a = d, θ₂ = 0) the coupler direction is undefined: nudge by 1e-4°
      function current() {
        const g = geom();
        if (!g.ok) return null;
        const t = st.theta2 * D2R;
        return solve(st.a, st.b, st.c, st.d, t, st.mode) || solve(st.a, st.b, st.c, st.d, t + 1e-4 * D2R, st.mode) ||
          solve(st.a, st.b, st.c, st.d, t - 1e-4 * D2R, st.mode);
      }
      function rangeText(g) {
        if (g.ir.full) return 'full 360°';
        return g.ir.rangesDeg.map(function (r) { return fmtDeg(r[0], 1) + ' … ' + fmtDeg(r[1], 1); }).join(' and ');
      }

      // ---- label layout (thick-segment obstacles, cheapest candidate wins) ----
      function segHitsRect(x1, y1, x2, y2, l, t, r, b) {
        const dx = x2 - x1, dy = y2 - y1;
        const P = [-dx, dx, -dy, dy], Q = [x1 - l, r - x1, y1 - t, b - y1];
        let u0 = 0, u1 = 1;
        for (let i = 0; i < 4; i++) {
          if (P[i] === 0) { if (Q[i] < 0) return false; }
          else {
            const u = Q[i] / P[i];
            if (P[i] < 0) { if (u > u1) return false; if (u > u0) u0 = u; }
            else { if (u < u0) return false; if (u < u1) u1 = u; }
          }
        }
        return true;
      }
      function makeLayout(ctx, frame) {
        const segs = [], boxes = [];
        const L = {
          seg: function (x1, y1, x2, y2, hw) { segs.push([x1, y1, x2, y2, hw]); },
          dot: function (x, y, rr) { segs.push([x, y, x, y, rr]); },
          poly: function (pts, hw) { for (let i = 1; i < pts.length; i++) segs.push([pts[i - 1][0], pts[i - 1][1], pts[i][0], pts[i][1], hw]); },
          hits: function (bx) {
            let n = 0;
            for (let i = 0; i < segs.length; i++) {
              const s = segs[i];
              if (segHitsRect(s[0], s[1], s[2], s[3], bx.l - s[4], bx.t - s[4], bx.r + s[4], bx.b + s[4])) n++;
            }
            boxes.forEach(function (o) { if (bx.l < o.r + 2 && bx.r > o.l - 2 && bx.t < o.b + 2 && bx.b > o.t - 2) n += 12; });   // text on text is unreadable: far worse than text on a line
            return n;
          },
          place: function (text, font, color, cands) {
            ctx.font = font;
            const a = ctx.measureText(text).width / 2 + 3, b = 8;
            let best = null;
            cands.forEach(function (c) {
              const bx = { l: c.x - a, r: c.x + a, t: c.y - b, b: c.y + b };
              let cost = c.cost;
              if (bx.l < frame.x + 1 || bx.r > frame.x + frame.w - 1 || bx.t < frame.y + 1 || bx.b > frame.y + frame.h - 1) cost += 8000;
              if (best && cost >= best.cost) return;
              cost += 1000 * L.hits(bx);
              if (!best || cost < best.cost) best = { cost: cost, c: c, bx: bx };
            });
            boxes.push(best.bx);
            return { x: best.c.x, y: best.c.y, text: text, font: font, color: color, box: best.bx };
          },
          around: function (cx, cy, r, mid, step, n, text, font) {
            ctx.font = font;
            const a = ctx.measureText(text).width / 2 + 3, b = 8;
            const out = [];
            for (let i = 0; i <= n; i++) {
              [i, -i].forEach(function (j, s) {
                if (i === 0 && s) return;
                const u = mid + j * step, ux = Math.cos(u), uy = Math.sin(u);
                [0, 10, 22, 36, 52].forEach(function (extra, ring) {
                  const dd = r + 3 + a * Math.abs(ux) + b * Math.abs(uy) + extra;
                  out.push({ x: cx + dd * ux, y: cy - dd * uy, cost: 5 * i + 9 * ring });
                });
              });
            }
            return out;
          }
        };
        return L;
      }

      function viewPlot(ctx, w, h, g) {
        return RASim.plot(ctx, { box: { x: 0, y: 0, w: w, h: h }, margin: { left: 4, right: 4, top: 4, bottom: 4 },
          xmin: g.box.x0, xmax: g.box.x1, ymin: g.box.y0, ymax: g.box.y1, equal: true });
      }
      function wrapText(ctx, text, maxW) {
        const words = text.split(' '), lines = [];
        let cur = '';
        words.forEach(function (wd) {
          const t = cur ? cur + ' ' + wd : wd;
          if (ctx.measureText(t).width > maxW && cur) { lines.push(cur); cur = wd; } else cur = t;
        });
        if (cur) lines.push(cur);
        return lines;
      }

      // where each IC is relative to the visible view p: 'none' (at infinity), 'far' (> FAR_VIEWS view-widths from the
      // view centre: not drawn), 'out' (outside the canvas: an edge arrow points to it) or 'in'
      function icInfo(s, p) {
        const W = p.xmax - p.xmin, cx = (p.xmin + p.xmax) / 2, cy = (p.ymin + p.ymax) / 2;
        function one(P) {
          if (!P) return { state: 'none' };
          const dist = Math.hypot(P[0] - cx, P[1] - cy) / W;
          if (dist > FAR_VIEWS) return { state: 'far', dist: dist };
          const x = p.X(P[0]), y = p.Y(P[1]), r = p.rect;
          const inside = x >= r.x + 8 && x <= r.x + r.w - 8 && y >= r.y + 8 && y <= r.y + r.h - 8;
          return { state: inside ? 'in' : 'out', dist: dist, px: [x, y] };
        }
        return { I24: one(s.I24), I13: one(s.I13) };
      }

      // ---- mechanism canvas ----
      cv.draw = function (ctx, w, h) {
        const g = geom();
        if (!g.ok) {
          ctx.font = 'bold 13px sans-serif';
          const msg = 'These lengths cannot form a closed chain: the longest link must be shorter than the sum of the other three.';
          const lines = wrapText(ctx, msg, Math.min(w - 32, 420));
          const lh = 19, y0 = h / 2 - (lines.length - 1) * lh / 2;
          lines.forEach(function (ln, i) { D.text(ctx, ln, w / 2, y0 + i * lh, { align: 'center', color: C.red, font: 'bold 13px sans-serif' }); });
          return;
        }
        const s = current();
        if (!s) return;
        const p = viewPlot(ctx, w, h, g);
        const k = p.X(1) - p.X(0);
        const X = function (q) { return [p.X(q[0]), p.Y(q[1])]; };
        const O2 = X([0, 0]), O4 = X([st.d, 0]), A = X(s.A), B = X(s.B);
        const Pw = couplerPoint(s.A, s.B, st.u, st.v), P = X(Pw);
        const r = p.rect;
        const lay = makeLayout(ctx, r);
        const LBG = 'rgba(255,255,255,0.85)';
        const F12 = '12px sans-serif', B12 = 'bold 12px sans-serif', I13 = 'italic bold 13px serif';

        // 1. frame line (dashed grey) and the IC construction lines
        ctx.strokeStyle = C.axis; ctx.lineWidth = 2; ctx.setLineDash([7, 5]);
        ctx.beginPath(); ctx.moveTo(O2[0], O2[1]); ctx.lineTo(O4[0], O4[1]); ctx.stroke(); ctx.setLineDash([]);
        lay.seg(O2[0], O2[1], O4[0], O4[1], 2);

        // 2. paths of A and B
        if (st.showPaths) {
          ctx.lineWidth = 1.5;
          g.runs.forEach(function (run) {
            [['A', C.maroon], ['B', C.green]].forEach(function (pc) {
              ctx.strokeStyle = RASim.alpha(pc[1], 0.35);
              ctx.beginPath();
              run.forEach(function (q, i) { const z = X(q.s[pc[0]]); if (i) ctx.lineTo(z[0], z[1]); else ctx.moveTo(z[0], z[1]); });
              ctx.stroke();
            });
          });
        }
        // 3. coupler curve
        if (st.showCoupler) {
          ctx.strokeStyle = C.orange; ctx.lineWidth = 2;
          g.curve.forEach(function (line) {
            ctx.beginPath();
            line.forEach(function (q, i) { const z = X(q); if (i) ctx.lineTo(z[0], z[1]); else ctx.moveTo(z[0], z[1]); });
            ctx.stroke();
          });
        }
        // 4. instantaneous centres: construction lines
        const ici = icInfo(s, p);
        let I24p = null, I13p = null;
        if (st.showIC) {
          ctx.lineWidth = 1; ctx.setLineDash([4, 4]);
          ctx.save(); ctx.beginPath(); ctx.rect(r.x, r.y, r.w, r.h); ctx.clip();
          function cline(P1, P2, col) {
            ctx.strokeStyle = RASim.alpha(col, 0.75);
            ctx.beginPath(); ctx.moveTo(P1[0], P1[1]); ctx.lineTo(P2[0], P2[1]); ctx.stroke();
          }
          if (ici.I24.state === 'in' || ici.I24.state === 'out') {
            I24p = ici.I24.px;
            cline(A, I24p, C.blue); cline(B, I24p, C.blue);
            cline(O2, I24p, C.muted); cline(O4, I24p, C.muted);
          }
          if (ici.I13.state === 'in' || ici.I13.state === 'out') {
            I13p = ici.I13.px;
            cline(O2, I13p, C.maroon); cline(A, I13p, C.maroon);
            cline(O4, I13p, C.green); cline(B, I13p, C.green);
          }
          ctx.restore();
          ctx.setLineDash([]);
        }

        // 5. ground pivots
        [O2, O4].forEach(function (q) {
          ctx.fillStyle = C.fill; ctx.strokeStyle = C.ink; ctx.lineWidth = 1.5;
          ctx.beginPath(); ctx.moveTo(q[0], q[1]); ctx.lineTo(q[0] - 9, q[1] + 14); ctx.lineTo(q[0] + 9, q[1] + 14); ctx.closePath(); ctx.fill(); ctx.stroke();
          D.hatch(ctx, q[0] - 13, q[1] + 14, q[0] + 13, q[1] + 14, { side: 1, length: 6, spacing: 6, color: C.ink });
          lay.seg(q[0] - 13, q[1] + 17, q[0] + 13, q[1] + 17, 5); lay.seg(q[0], q[1], q[0], q[1] + 12, 9);
        });

        // 6. coupler triangle A–B–P
        ctx.fillStyle = RASim.alpha(C.lightBlue, 0.35); ctx.strokeStyle = RASim.alpha(C.blue, 0.5); ctx.lineWidth = 1;
        ctx.beginPath(); ctx.moveTo(A[0], A[1]); ctx.lineTo(B[0], B[1]); ctx.lineTo(P[0], P[1]); ctx.closePath(); ctx.fill(); ctx.stroke();
        lay.seg(A[0], A[1], P[0], P[1], 1); lay.seg(B[0], B[1], P[0], P[1], 1);

        // 7. links
        ctx.lineCap = 'round';
        [[O4, B, C.green], [O2, A, C.maroon], [A, B, C.blue]].forEach(function (lk) {
          ctx.strokeStyle = lk[2]; ctx.lineWidth = 8;
          ctx.beginPath(); ctx.moveTo(lk[0][0], lk[0][1]); ctx.lineTo(lk[1][0], lk[1][1]); ctx.stroke();
          lay.seg(lk[0][0], lk[0][1], lk[1][0], lk[1][1], 5);
        });
        ctx.lineCap = 'butt';
        [O2, O4, A, B].forEach(function (q) {
          ctx.fillStyle = '#ffffff'; ctx.strokeStyle = C.ink; ctx.lineWidth = 2;
          ctx.beginPath(); ctx.arc(q[0], q[1], 4.5, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
          lay.dot(q[0], q[1], 6);
        });
        // drag handle ring on A
        ctx.strokeStyle = C.maroon; ctx.lineWidth = 1.5;
        ctx.beginPath(); ctx.arc(A[0], A[1], 9, 0, 2 * Math.PI); ctx.stroke();
        ctx.fillStyle = C.orange; ctx.strokeStyle = '#ffffff'; ctx.lineWidth = 1.5;
        ctx.beginPath(); ctx.arc(P[0], P[1], 5, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
        lay.dot(P[0], P[1], 6);

        // 8. θ₂ arc at O₂
        const t2 = st.theta2 * D2R;
        const r2 = M.clamp(0.4 * st.a * k, 14, 30);
        if (st.theta2 > 0.05) {
          D.angleArc(ctx, O2[0], O2[1], r2, 0, t2, { color: C.maroon, width: 1.5 });
          const n = 12;
          for (let i = 0; i < n; i++) {
            const q0 = t2 * i / n, q1 = t2 * (i + 1) / n;
            lay.seg(O2[0] + r2 * Math.cos(q0), O2[1] - r2 * Math.sin(q0), O2[0] + r2 * Math.cos(q1), O2[1] - r2 * Math.sin(q1), 2);
          }
        }
        // 9. μ arc at B between B→A and B→O₄
        const poor = s.mu * R2D < MU_LO || s.mu * R2D > MU_HI;
        const muCol = poor ? C.red : C.teal;
        const phA = Math.atan2(s.A[1] - s.B[1], s.A[0] - s.B[0]), phO = Math.atan2(-s.B[1], st.d - s.B[0]);
        const dph = M.wrapPi(phO - phA);
        const rm = M.clamp(0.3 * Math.min(st.b, st.c) * k, 12, 26);
        if (st.showMu && Math.abs(dph) > 1e-3) {
          D.angleArc(ctx, B[0], B[1], rm, phA, phA + dph, { ccw: dph >= 0, color: muCol, width: 2 });
          const n = 10;
          for (let i = 0; i < n; i++) {
            const q0 = phA + dph * i / n, q1 = phA + dph * (i + 1) / n;
            lay.seg(B[0] + rm * Math.cos(q0), B[1] - rm * Math.sin(q0), B[0] + rm * Math.cos(q1), B[1] - rm * Math.sin(q1), 2);
          }
        }
        // 10. IC points
        function icDot(q) {
          ctx.fillStyle = '#ffffff'; ctx.strokeStyle = C.purple; ctx.lineWidth = 2;
          ctx.beginPath(); ctx.arc(q[0], q[1], 5, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
          ctx.fillStyle = C.purple; ctx.beginPath(); ctx.arc(q[0], q[1], 2, 0, 2 * Math.PI); ctx.fill();
          lay.dot(q[0], q[1], 7);
        }
        // an IC outside the canvas: an arrow at the edge, pointing towards it
        function icEdge(q) {
          const cx = r.x + r.w / 2, cy = r.y + r.h / 2, dx = q[0] - cx, dy = q[1] - cy;
          const t = Math.min((r.w / 2 - 10) / Math.max(1e-9, Math.abs(dx)), (r.h / 2 - 10) / Math.max(1e-9, Math.abs(dy)));
          const L = Math.hypot(dx, dy), ex = cx + dx * t, ey = cy + dy * t;
          D.arrow(ctx, ex - 26 * dx / L, ey - 26 * dy / L, ex, ey, { color: C.purple, width: 2, head: 8 });
          lay.seg(ex - 26 * dx / L, ey - 26 * dy / L, ex, ey, 4);
          return [ex - 26 * dx / L, ey - 26 * dy / L, Math.atan2(-dy, dx) + Math.PI];   // label behind the arrow tail
        }
        let I24lab = null, I13lab = null;
        if (I24p) I24lab = ici.I24.state === 'in' ? (icDot(I24p), [I24p[0], I24p[1], -Math.PI / 2]) : icEdge(I24p);
        if (I13p) I13lab = ici.I13.state === 'in' ? (icDot(I13p), [I13p[0], I13p[1], Math.PI / 2]) : icEdge(I13p);
        if (st.showCoupler) g.curve.forEach(function (line) { lay.poly(line.map(X), 1.5); });

        // 11. labels (most important first)
        const labels = [];
        function linkLabel(P1, P2, text, color) {
          const mx = (P1[0] + P2[0]) / 2, my = (P1[1] + P2[1]) / 2, L = Math.hypot(P2[0] - P1[0], P2[1] - P1[1]) || 1;
          const nx = -(P2[1] - P1[1]) / L, ny = (P2[0] - P1[0]) / L;
          const tx = (P2[0] - P1[0]) / L, ty = (P2[1] - P1[1]) / L;
          const cands = [];
          [0, 0.18, -0.18, 0.32, -0.32].forEach(function (al, i) {
            [12, 20, 30, 42, 56].forEach(function (off, j) {
              [1, -1].forEach(function (sg, kk) {
                cands.push({ x: mx + al * L * tx + sg * off * nx, y: my + al * L * ty + sg * off * ny, cost: 4 * i + 6 * j + kk });
              });
            });
          });
          labels.push(lay.place(text, I13, color, cands));
        }
        function pointLabel(q, text, color, mid, font) {
          labels.push(lay.place(text, font || B12, color, lay.around(q[0], q[1], 7, mid, Math.PI / 8, 8, text, font || B12)));
        }
        // a label that ended up away from its arc gets a thin leader line to the middle of the arc
        const leaders = [];
        function leader(lb, ax, ay) {
          const b = lb.box, nx = M.clamp(ax, b.l, b.r), ny = M.clamp(ay, b.t, b.b);
          if (Math.hypot(nx - ax, ny - ay) > 10) leaders.push([ax, ay, nx, ny, lb.color]);
        }
        const muTxt = 'μ = ' + fmtDeg(s.mu * R2D, 1);
        if (st.showMu) {
          const midm = phA + dph / 2;
          const lm = lay.place(muTxt, B12, muCol, lay.around(B[0], B[1], rm, midm, Math.PI / 14, 10, muTxt, B12));
          labels.push(lm);
          leader(lm, B[0] + rm * Math.cos(midm), B[1] - rm * Math.sin(midm));
        }
        const t2Txt = 'θ₂ = ' + fmtDeg(st.theta2, 1);
        const mid2 = st.theta2 > 0.05 ? t2 / 2 : -Math.PI / 4;
        const l2 = lay.place(t2Txt, F12, C.maroon, lay.around(O2[0], O2[1], r2, mid2, Math.PI / 12, 12, t2Txt, F12));
        labels.push(l2);
        if (st.theta2 > 0.05) leader(l2, O2[0] + r2 * Math.cos(mid2), O2[1] - r2 * Math.sin(mid2));
        pointLabel(O2, 'O₂', C.ink, -Math.PI * 0.75);
        pointLabel(O4, 'O₄', C.ink, -Math.PI * 0.25);
        pointLabel(A, 'A', C.ink, Math.atan2(s.A[1], s.A[0]));
        pointLabel(B, 'B', C.ink, Math.atan2(s.B[1], s.B[0] - st.d));
        pointLabel(P, 'P', C.orange, Math.atan2(Pw[1] - (s.A[1] + s.B[1]) / 2, Pw[0] - (s.A[0] + s.B[0]) / 2));
        linkLabel(O2, A, 'a', C.maroon);
        linkLabel(A, B, 'b', C.blue);
        linkLabel(O4, B, 'c', C.green);
        linkLabel(O2, O4, 'd', C.muted);
        if (I24lab) pointLabel(I24lab, ici.I24.state === 'in' ? 'I₂₄' : 'to I₂₄', C.purple, I24lab[2]);
        if (I13lab) pointLabel(I13lab, ici.I13.state === 'in' ? 'I₁₃' : 'to I₁₃', C.purple, I13lab[2]);
        ctx.lineWidth = 1; ctx.setLineDash([]);
        leaders.forEach(function (q) {
          ctx.strokeStyle = RASim.alpha(q[4], 0.7);
          ctx.beginPath(); ctx.moveTo(q[0], q[1]); ctx.lineTo(q[2], q[3]); ctx.stroke();
        });
        labels.forEach(function (lb) { D.text(ctx, lb.text, lb.x, lb.y, { align: 'center', color: lb.color, font: lb.font, bg: LBG }); });
      };

      // ---- drag the pin A to set θ₂ ----
      function hitA(x, y) {
        const g = geom();
        if (!g.ok) return null;
        const s = current();
        if (!s) return null;
        const p = viewPlot(cv.ctx, cv.width, cv.height, g);
        return Math.hypot(p.X(s.A[0]) - x, p.Y(s.A[1]) - y) <= 16 ? { p: p } : null;
      }
      cv.onDrag({
        hit: hitA,
        move: function (x, y, token) {
          const p = token.p;
          const wx = p.invX(x), wy = p.invY(y);
          if (Math.hypot(wx, wy) < 1e-9) return;
          if (anim.running) setPlaying(false);
          snapTheta(Math.atan2(wy, wx) * R2D);
          update(true);
        },
        end: function () { update(); },
        cursor: function (x, y) { return hitA(x, y) ? 'grab' : null; }
      });

      // ---- plot canvas ----
      cvPlot.draw = function (ctx, w, h) {
        const g = geom();
        const narrow = w < 380;
        const ymin = g.ok ? g.plot.ymin : 0;
        const pad = ymin < 0 ? 14 : 8;     // a little headroom so curves at ±180° / 0° stay off the frame
        const p = RASim.plot(ctx, { box: { x: 0, y: 0, w: w, h: h }, xmin: 0, xmax: 360, ymin: ymin - pad, ymax: 180 + pad,
          margin: { left: 46, right: 12, top: 10, bottom: 36 }, xlabel: 'θ₂ (°)', ylabel: 'angle (°)' });
        p.axes({ xTicks: narrow ? [0, 90, 180, 270, 360] : [0, 45, 90, 135, 180, 225, 270, 315, 360],
          yTicks: ymin < 0 ? [-180, -90, 0, 90, 180] : [0, 45, 90, 135, 180] });
        p.band(MU_LO, MU_HI, { fill: RASim.alpha(C.green, 0.08) });
        p.hline(MU_LO, { color: RASim.alpha(C.green, 0.4), width: 1, dash: [3, 3] });
        p.hline(MU_HI, { color: RASim.alpha(C.green, 0.4), width: 1, dash: [3, 3] });
        if (!g.ok) {
          p.text(180, (ymin + 180) / 2, 'no valid input angle', { align: 'center', color: C.red, font: 'bold 12px sans-serif', bg: 'rgba(255,255,255,0.9)' });
          return;
        }
        // rocker limits of the crank-rocker: the crossed configuration is the mirror image (θ₂ → −θ₂, θ₄ → −θ₄)
        const crossed = st.mode === 'crossed';
        const lim = g.rl ? [g.rl.theta2ExtendedDeg, g.rl.theta2FoldedDeg].map(function (t) {
          const x = norm360(crossed ? -t : t);
          const s = solve(st.a, st.b, st.c, st.d, x * D2R, st.mode);
          return s ? [x, s.theta4 * R2D] : null;
        }) : [];
        lim.forEach(function (q) { if (q) p.vline(q[0], { color: C.green, width: 1.2, dash: [5, 4] }); });
        p.line(g.plot.xs, g.plot.ym, { color: C.red, width: 2 });
        p.line(g.plot.xs, g.plot.y4, { color: C.green, width: 2 });
        lim.forEach(function (q) { if (q) p.point(q[0], q[1], { color: C.green, r: 3.5 }); });
        // band label: the first candidate spot (inside the band, at its top or bottom edge) that no curve passes through
        ctx.font = '11px sans-serif';
        const bw = ctx.measureText('good transmission').width + 8, bh = 16;
        const spots = [[1, 'right', 1], [1, 'left', 1], [0, 'right', 1], [0, 'left', 1], [1, 'center', 1], [0, 'center', 1], [1, 'right', 0]];
        let spot = spots[spots.length - 1];
        for (let i = 0; i < spots.length; i++) {
          const sp2 = spots[i], top = sp2[0] === 1;
          const px = sp2[1] === 'right' ? p.rect.x + p.rect.w - 4 - bw : sp2[1] === 'left' ? p.rect.x + 4 : p.rect.x + p.rect.w / 2 - bw / 2;
          const py = top ? p.Y(MU_HI) + 1 : p.Y(MU_LO) - 1 - bh;
          let hit = false;
          for (let j = 0; j < g.plot.xs.length && !hit; j++) {
            const X0 = p.X(g.plot.xs[j]);
            if (!(X0 >= px - 2 && X0 <= px + bw + 2)) continue;
            [g.plot.y4[j], g.plot.ym[j]].forEach(function (yv) { const Y0 = p.Y(yv); if (Y0 >= py - 3 && Y0 <= py + bh + 3) hit = true; });
          }
          if (!hit) { spot = sp2; spot.px = px; spot.py = py; break; }
          spot = sp2; spot.px = px; spot.py = py;
        }
        D.text(ctx, 'good transmission', spot.px + 4, spot.py + bh / 2, { color: C.green, font: '11px sans-serif', bg: 'rgba(255,255,255,0.8)' });
        const s = current();
        if (s) {
          p.vline(st.theta2, { color: C.muted, width: 1 });
          p.point(st.theta2, s.theta4 * R2D, { color: C.green, r: 5 });
          p.point(st.theta2, s.mu * R2D, { color: C.red, r: 5 });
        }
      };

      // ---- animation ----
      const anim = api.animate(function (dt) {
        const g = geom();
        if (!g.ok) return;
        const stepDeg = st.speed * dt;
        let th = st.theta2;
        if (g.ir.full) th = norm360(th + stepDeg);
        else {
          let r = null, t = null;
          g.ir.rangesDeg.forEach(function (q) { if (r === null) { const u = inRange(th, q); if (u !== null) { r = q; t = u; } } });
          if (r === null) { snapTheta(th); update(true); return; }
          t += dir * stepDeg;
          if (t >= r[1]) { t = r[1]; dir = -1; }
          if (t <= r[0]) { t = r[0]; dir = 1; }
          th = norm360(t);
        }
        snapMsg = '';
        st.theta2 = th; sT.set(th);
        update(true);
      });
      function setPlaying(on) {
        if (on && !anim.running) { snapMsg = ''; anim.start(); }
        if (!on && anim.running) anim.stop();
        st.playing = anim.running;
        bPlay.textContent = anim.running ? 'Pause' : 'Play';
      }

      // ---- read-outs ----
      function syncPreset() {
        const p = PRESETS.filter(function (q) {
          return tenths(q.L[0]) === tenths(st.a) && tenths(q.L[1]) === tenths(st.b) && tenths(q.L[2]) === tenths(st.c) && tenths(q.L[3]) === tenths(st.d);
        })[0];
        selP.set(p ? p.value : CUSTOM);
      }
      function update(now) {
        const g = geom();
        const cl = g.cl;
        syncPreset();
        const tg = TAGS[cl.type];
        tag.set(tg[0], tg[1]);
        const nm = function (x, link) { return fmt(x, 4) + ' (<i>' + link + '</i>)'; };
        if (!cl.assemblable) {
          grEl.innerHTML = 'l = ' + nm(cl.l, cl.links.l) + ' ≥ s + p + q = ' + fmt(cl.sumSPQ, 4) + '  ⇒  cannot be assembled';
        } else {
          const rel = cl.grashof === 'grashof' ? '&lt;' : cl.grashof === 'change-point' ? '=' : '&gt;';
          const concl = cl.grashof === 'grashof' ? 'Grashof' : cl.grashof === 'change-point' ? 'change-point (limit of Grashof)' : 'non-Grashof';
          grEl.innerHTML = 's + l = ' + nm(cl.s, cl.links.s) + ' + ' + nm(cl.l, cl.links.l) + ' = ' + fmt(cl.sumSL, 4) +
            ' &nbsp;' + rel + '&nbsp; p + q = ' + nm(cl.p, cl.links.p) + ' + ' + nm(cl.q, cl.links.q) + ' = ' + fmt(cl.sumPQ, 4) +
            ' &nbsp;⇒&nbsp; <b>' + concl + '</b>';
        }
        const s = current();
        const status = [];
        if (!g.ok || !s) {
          [tT3, tT4, tMu, tMuR, tW4, tW3, tMA, tI24, tSwing, tTR].forEach(function (t) { t.set('—'); t.setState(''); });
          tRange.set('none');
          status.push('Not assemblable: the longest link l = ' + fmt(cl.l, 4) + ' is not shorter than s + p + q = ' + fmt(cl.sumSPQ, 4) + '.');
          kenEl.style.display = 'none';
        } else {
          tRange.set(rangeText(g));
          tT3.set(fmtDeg(s.theta3 * R2D));
          tT4.set(fmtDeg(s.theta4 * R2D));
          const muD = s.mu * R2D, poor = muD < MU_LO || muD > MU_HI;
          tMu.set(fmtDeg(muD)); tMu.setState(poor ? 'bad' : 'ok');
          tMuR.set(fmtDeg(g.tr.minDeg) + ' … ' + fmtDeg(g.tr.maxDeg));
          tMuR.setState(g.tr.minDeg < MU_LO || g.tr.maxDeg > MU_HI ? 'bad' : 'ok');
          const toggleOut = !isFinite(s.w4) || Math.abs(s.w4) > 1e6;
          tW4.set(toggleOut ? '∞ (toggle)' : fmt(s.w4, 4));
          tW3.set(!isFinite(s.w3) || Math.abs(s.w3) > 1e6 ? '∞ (toggle)' : fmt(s.w3, 4));
          if (toggleOut) tMA.set('0 (toggle)');
          else if (Math.abs(s.w4) < 1e-6) tMA.set('∞ (toggle)');
          else tMA.set(fmt(1 / s.w4, 4));
          tMA.setState(toggleOut || Math.abs(s.w4) < 1e-6 ? 'warn' : '');
          tI24.set(s.I24 ? fmtFixed(s.I24[0], 3) : '∞ (AB ∥ O₂O₄)');
          if (g.rl) { tSwing.set(fmtDeg(g.rl.swingDeg)); tTR.set(fmt(g.rl.timeRatio, 4)); }
          else { tSwing.set('—'); tTR.set('—'); }

          // status line
          if (snapMsg) status.push(snapMsg + ' Allowed: ' + rangeText(g) + '.');
          else if (g.ir.full) status.push('The input a rotates fully (θ₂ from 0° to 360°).');
          else status.push('The input a only rocks: allowed θ₂ = ' + rangeText(g) + '.');
          if (toggleOut) status.push('Toggle: coupler and output are collinear (μ = ' + fmtDeg(muD, 0) + '), ω₄/ω₂ is unbounded.');
          else if (Math.abs(s.w4) < 1e-6) status.push('Crank and coupler are collinear: the output is at a limit (dead centre, MA → ∞).');
          if (cl.type === 'change-point') status.push('Change-point linkage: where all links become collinear the motion is indeterminate (open and crossed meet).');
          if (st.showIC) {
            const ici = icInfo(s, viewPlot(cv.ctx, cv.width, cv.height, g));
            [['I₂₄', ici.I24], ['I₁₃', ici.I13]].forEach(function (q) {
              if (q[1].state === 'far') status.push(q[0] + ' lies ' + fmt(q[1].dist, 2) + ' view-widths away (not drawn).');
              if (q[1].state === 'out') status.push(q[0] + ' is just outside the view: the purple arrow points to it.');
            });
            if (!s.I24) status.push('I₂₄ is at infinity (AB ∥ O₂O₄): ω₄/ω₂ = 1.');
            if (!s.I13) status.push('I₁₃ is at infinity (O₂A ∥ O₄B).');
            // live Kennedy formula, printed so that the arithmetic redone from the printed numbers gives the result
            if (s.I24 && !toggleOut) {
              const target = fmt(s.w4, 4), x = s.I24[0], dd = st.d;
              let dec = 3, xr = x;
              for (; dec <= 10; dec++) { xr = Number(x.toFixed(dec)); if (fmt(xr / (xr - dd), 4) === target) break; }
              const xs = fmtFixed(xr, dec);
              kenEl.innerHTML = 'Kennedy: I₂₄ is on the frame line (through I₁₂ = O₂ and I₁₄ = O₄) and on the coupler line (through I₂₃ = A and I₃₄ = B). ' +
                'ω₄/ω₂ = x<sub>I</sub>/(x<sub>I</sub> − d) = ' + xs + '/(' + xs + ' − ' + fmt(dd, 4) + ') = <b>' + target + '</b>. ' +
                'Six centres in all: I₁₂, I₁₃, I₁₄, I₂₃, I₂₄, I₃₄.';
            } else {
              kenEl.innerHTML = 'Kennedy: I₂₄ is on the frame line (I₁₂I₁₄) and on the coupler line (I₂₃I₃₄). Six centres in all: I₁₂, I₁₃, I₁₄, I₂₃, I₂₄, I₃₄.';
            }
            kenEl.style.display = '';
          } else kenEl.style.display = 'none';
        }
        statusEl.innerHTML = status.join(' ');
        if (now === true) { cv.drawNow(); cvPlot.drawNow(); }
        else { cv.redraw(); cvPlot.redraw(); }
      }

      function getState() {
        return { a: st.a, b: st.b, c: st.c, d: st.d, theta2: st.theta2, mode: st.mode, playing: anim.running, speed: st.speed,
          u: st.u, v: st.v, showCoupler: st.showCoupler, showPaths: st.showPaths, showIC: st.showIC, showMu: st.showMu };
      }
      function setState(s) {
        s = s || {};
        const num = function (v) { return v != null && isFinite(Number(v)); };
        if (num(s.a)) { sA.set(Number(s.a)); st.a = sA.get(); }
        if (num(s.b)) { sB.set(Number(s.b)); st.b = sB.get(); }
        if (num(s.c)) { sC.set(Number(s.c)); st.c = sC.get(); }
        if (num(s.d)) { sD.set(Number(s.d)); st.d = sD.get(); }
        if (num(s.speed)) { sSpeed.set(Number(s.speed)); st.speed = sSpeed.get(); }
        if (num(s.u)) { sU.set(Number(s.u)); st.u = sU.get(); }
        if (num(s.v)) { sV.set(Number(s.v)); st.v = sV.get(); }
        if (s.mode != null) { st.mode = s.mode === 'crossed' ? 'crossed' : 'open'; seg.set(st.mode); }
        if (s.showCoupler != null) { st.showCoupler = !!s.showCoupler; cCoup.set(st.showCoupler); }
        if (s.showPaths != null) { st.showPaths = !!s.showPaths; cPath.set(st.showPaths); }
        if (s.showIC != null) { st.showIC = !!s.showIC; cIC.set(st.showIC); }
        if (s.showMu != null) { st.showMu = !!s.showMu; cMu.set(st.showMu); }
        const lengths = num(s.a) || num(s.b) || num(s.c) || num(s.d);
        if (num(s.theta2)) snapTheta(Number(s.theta2));   // any angle in degrees, taken modulo 360°
        else if (lengths) snapTheta(st.theta2);
        if (s.playing != null) { if (s.playing) dir = 1; setPlaying(!!s.playing); }
        update();
      }

      update();
      return { getState: getState, setState: setState };
    }
  });
})();
