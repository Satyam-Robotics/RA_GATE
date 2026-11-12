// rotation-transforms — "3D rotations and homogeneous transforms"
// Sits beside notes/src/03_robotics.html, sections "3. Rotation Matrices" and "4. Homogeneous Transformations".
//
// SIGN CONVENTIONS
//   Right-handed frames. Frame A is fixed (world, Z_A up); frame B moves and starts coincident with A.
//   Elementary rotations exactly as in the notes (c = cos θ, s = sin θ, θ counter-clockwise positive about the axis
//   by the right-hand rule):
//     Rx(θ) = [[1,0,0],[0,c,−s],[0,s,c]],  Ry(θ) = [[c,0,s],[0,1,0],[−s,0,c]],  Rz(θ) = [[c,−s,0],[s,c,0],[0,0,1]]
//   R = ᴬR_B: its COLUMNS are B's unit axes x_B, y_B, z_B written in A.
//   Three steps applied in the order 1, 2, 3 with step matrices R₁, R₂, R₃:
//     mode 'fixed'   (about the FIXED axes of A):    R = R₃·R₂·R₁  (each new rotation PRE-multiplies)
//     mode 'current' (about B's own CURRENT axes):   R = R₁·R₂·R₃  (each new rotation POST-multiplies)
//   d = (dx, dy, dz): position of B's origin in A.  T = [[R, d], [0, 0, 0, 1]].  p_A = R·p_B + d.
//   T⁻¹ = [[Rᵀ, −Rᵀd], [0, 0, 0, 1]].
//   Axis–angle: θ = acos(clamp((trace R − 1)/2, −1, 1)) ∈ [0, π]; trace R = 1 + 2 cos θ.
//     θ < 1e-6: axis undefined (null).
//     π − θ < 1e-6: kᵢ = sqrt(max(0, (Rᵢᵢ + 1)/2)), largest component positive, the others take the sign of
//       (R_ij + R_ji) with respect to it; then normalised. Refinement (see the report): when the antisymmetric part
//       v = [R₃₂ − R₂₃, R₁₃ − R₃₁, R₂₁ − R₁₂] is not round-off (|v| > 1e-12) and v·k < 0, k is negated, because then
//       θ < π exactly and only +k reproduces R (at θ = π both signs are correct).
//     otherwise: k = [R₃₂ − R₂₃, R₁₃ − R₃₁, R₂₁ − R₁₂] / (2 sin θ), normalised.
//   Model angles are radians; the UI and the state use degrees (angle1…3, az, el).
//   View: orthographic, world Z up. Camera direction c = (cos el·cos az, cos el·sin az, sin el) (from the origin
//   towards the viewer); screen right = (−sin az, cos az, 0); screen up = c × right = (−sin el·cos az,
//   −sin el·sin az, cos el), so right × up = c (a right-handed picture). Default az = 30°, el = 25°: Z_A up, X_A towards the lower left, Y_A towards the
//   lower right. Dragging right decreases az, dragging down increases el (clamped to ±85°).
//   Canvas label background is the canvas' own white.
//
// STATE KEYS (getState / setState)
//   mode ('fixed' | 'current'), axis1, axis2, axis3 ('x' | 'y' | 'z'), angle1, angle2, angle3 (degrees),
//   dx, dy, dz, px, py, pz, showGhost, showAxis, showBox (booleans), az, el (degrees),
//   animStep (0 = not animating, else 1…3), animFrac (0…1).
//   setState({ animStep, animFrac }) freezes the picture at that moment of the animation; any other key in a
//   setState call without animStep clears the freeze. Every setState stops a running animation.
//
// STAT KEYS:   detR, trace, eqAngle, eqAxis, pA, orth
// TAG KEYS:    rule   (FIXED AXES → R = R₃·R₂·R₁ / CURRENT AXES → R = R₁·R₂·R₃)
// MATRIX KEYS: R, T, Tinv
// DATA-KEY:    formula (the multiplication-order line), Rcaption, animNote (what the animation is doing)
// CANVAS KEY:  scene
// CONTROL KEYS: mode, axis1, angle1, axis2, angle2, axis3, angle3, preset, dx, dy, dz, px, py, pz,
//               showGhost, showAxis, showBox, play, reset, touchOrbit (touch screens only; not part of the state:
//               a device preference. Off: a one-finger touch on the picture scrolls the page; on: it orbits)
(function () {
  'use strict';
  const RASim = globalThis.RASim;
  const M = RASim.math;

  // ------------------------------------------------------------------ model (pure) ----
  function Rx(a) { const c = Math.cos(a), s = Math.sin(a); return [[1, 0, 0], [0, c, -s], [0, s, c]]; }
  function Ry(a) { const c = Math.cos(a), s = Math.sin(a); return [[c, 0, s], [0, 1, 0], [-s, 0, c]]; }
  function Rz(a) { const c = Math.cos(a), s = Math.sin(a); return [[c, -s, 0], [s, c, 0], [0, 0, 1]]; }
  function rot(axis, a) {
    const ax = String(axis).toLowerCase();
    if (ax === 'x') return Rx(a);
    if (ax === 'y') return Ry(a);
    if (ax === 'z') return Rz(a);
    throw new Error('rot: axis must be x, y or z (got ' + axis + ')');
  }
  function eye(n) { const A = []; for (let i = 0; i < n; i++) { const r = []; for (let j = 0; j < n; j++) r.push(i === j ? 1 : 0); A.push(r); } return A; }
  function mul(A, B) {
    const n = A.length, p = B.length, m = B[0].length, C = [];
    for (let i = 0; i < n; i++) {
      const r = [];
      for (let j = 0; j < m; j++) { let s = 0; for (let k = 0; k < p; k++) s += A[i][k] * B[k][j]; r.push(s); }
      C.push(r);
    }
    return C;
  }
  function transpose(A) { return A[0].map(function (_, j) { return A.map(function (r) { return r[j]; }); }); }
  function det3(A) {
    return A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
      - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
      + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
  }
  function trace(A) { let s = 0; for (let i = 0; i < A.length; i++) s += A[i][i]; return s; }
  function isCurrent(mode) { return mode === 'current'; }

  // steps = [{ axis, angle (rad) }] in order of application
  function compose(steps, mode) {
    let R = eye(3);
    const cur = isCurrent(mode);
    for (let i = 0; i < steps.length; i++) {
      const Ri = rot(steps[i].axis, steps[i].angle);
      R = cur ? mul(R, Ri) : mul(Ri, R);
    }
    return R;
  }
  // steps 1 … k−1 complete, step k (1-based) at fraction f of its angle; later steps not applied
  function partial(steps, mode, k, f) {
    const cur = isCurrent(mode);
    const n = Math.min(Math.max(0, Math.round(k)), steps.length);
    const ff = M.clamp(Number(f), 0, 1);
    let R = eye(3);
    for (let i = 0; i < n; i++) {
      const a = i === n - 1 && n === Math.round(k) ? ff * steps[i].angle : steps[i].angle;
      const Ri = rot(steps[i].axis, a);
      R = cur ? mul(R, Ri) : mul(Ri, R);
    }
    return R;
  }
  function homogeneous(R, d) {
    return [
      [R[0][0], R[0][1], R[0][2], d[0]],
      [R[1][0], R[1][1], R[1][2], d[1]],
      [R[2][0], R[2][1], R[2][2], d[2]],
      [0, 0, 0, 1]
    ];
  }
  function invHomogeneous(T) {
    const Rt = [[T[0][0], T[1][0], T[2][0]], [T[0][1], T[1][1], T[2][1]], [T[0][2], T[1][2], T[2][2]]];
    const d = [T[0][3], T[1][3], T[2][3]];
    const t = Rt.map(function (r) { return -(r[0] * d[0] + r[1] * d[1] + r[2] * d[2]); });
    return homogeneous(Rt, t);
  }
  function apply(T, p) {
    const out = [];
    for (let i = 0; i < 3; i++) out.push(T[i][0] * p[0] + T[i][1] * p[1] + T[i][2] * p[2] + T[i][3]);
    return out;
  }
  function unit(v) { const n = Math.hypot(v[0], v[1], v[2]); return [v[0] / n, v[1] / n, v[2] / n]; }
  function axisAngle(R) {
    const th = Math.acos(M.clamp((trace(R) - 1) / 2, -1, 1));
    if (th < 1e-6) return { angle: th, axis: null };
    const v = [R[2][1] - R[1][2], R[0][2] - R[2][0], R[1][0] - R[0][1]];
    if (Math.PI - th < 1e-6) {
      const k = [0, 1, 2].map(function (i) { return Math.sqrt(Math.max(0, (R[i][i] + 1) / 2)); });
      let j = 0;
      if (k[1] > k[j]) j = 1;
      if (k[2] > k[j]) j = 2;
      for (let i = 0; i < 3; i++) if (i !== j && R[i][j] + R[j][i] < 0) k[i] = -k[i];
      let u = unit(k);
      // refinement: θ slightly below π — only +k (not −k) reproduces R; decided by the antisymmetric part
      if (Math.hypot(v[0], v[1], v[2]) > 1e-12 && v[0] * u[0] + v[1] * u[1] + v[2] * u[2] < 0) u = [-u[0], -u[1], -u[2]];
      return { angle: th, axis: u.map(function (x) { return x + 0; }) };
    }
    const s2 = 2 * Math.sin(th);
    return { angle: th, axis: unit([v[0] / s2, v[1] / s2, v[2] / s2]) };
  }
  function degTxt(a) { return RASim.fmt(Math.round(M.deg(a) * 100) / 100, 7) + '°'; }
  function formula(steps, mode) {
    const terms = [];
    steps.forEach(function (s) {
      if (Math.round(M.deg(s.angle) * 100) / 100 === 0) return;
      terms.push('R' + String(s.axis).toLowerCase() + '(' + degTxt(s.angle) + ')');
    });
    if (!terms.length) return 'R = I';
    if (!isCurrent(mode)) terms.reverse();
    return 'R = ' + terms.join(' · ');
  }

  const model = {
    Rx: Rx, Ry: Ry, Rz: Rz, rot: rot, mul: mul, transpose: transpose, det3: det3, trace: trace,
    compose: compose, partial: partial, homogeneous: homogeneous, invHomogeneous: invHomogeneous, apply: apply,
    axisAngle: axisAngle, formula: formula
  };

  // ------------------------------------------------------------------ user interface ----
  const DEFAULTS = {
    mode: 'fixed', axis1: 'x', angle1: 90, axis2: 'z', angle2: 90, axis3: 'y', angle3: 0,
    dx: 0, dy: 0, dz: 0, px: 1, py: 1, pz: 0, showGhost: true, showAxis: true, showBox: true,
    az: 30, el: 25, animStep: 0, animFrac: 0
  };
  const PRESETS = {
    order: { label: 'X 90° then Z 90° — order matters', s: { axis1: 'x', angle1: 90, axis2: 'z', angle2: 90, axis3: 'y', angle3: 0 } },
    rpy: { label: 'Roll–pitch–yaw, fixed XYZ: Rz(60°)·Ry(45°)·Rx(30°)', s: { mode: 'fixed', axis1: 'x', angle1: 30, axis2: 'y', angle2: 45, axis3: 'z', angle3: 60 } },
    zyz: { label: 'ZYZ Euler angles: Rz(30°)·Ry(45°)·Rz(60°)', s: { mode: 'current', axis1: 'z', angle1: 30, axis2: 'y', angle2: 45, axis3: 'z', angle3: 60 } },
    rz90: { label: 'Single rotation Rz(90°)', s: { axis1: 'z', angle1: 90, axis2: 'x', angle2: 0, axis3: 'y', angle3: 0 } },
    identity: { label: 'Identity', s: { axis1: 'x', angle1: 0, axis2: 'z', angle2: 0, axis3: 'y', angle3: 0 } }
  };
  const STEP_T = 1.2;          // s per animated step
  const BOX = [1, 0.6, 0.3];   // body box along x_B, y_B, z_B
  const WHITE = '#ffffff';     // the canvas background (core.css .sim-canvas)
  const ID = 'rotation-transforms';
  const SC = '.sim[data-sim="' + ID + '"]';

  RASim.register(ID, {
    title: '3D rotations and homogeneous transforms',
    model: model,
    mount: function (api) {
      const ui = RASim.ui, C = RASim.colors, D = RASim.draw;
      const fmt = RASim.fmt, fmtFixed = RASim.fmtFixed;
      const rad = M.rad;
      const AXC = [C.red, C.green, C.blue];
      const st = Object.assign({}, DEFAULTS);

      RASim.css(ID,
        SC + ' .rt-nt { text-transform: none; letter-spacing: 0; font-size: 11.5px; }\n' +
        SC + ' .rt-step { border-left: 3px solid transparent; border-radius: 3px; padding: 3px 0 3px 7px; margin-left: -10px; }\n' +
        SC + ' .rt-step.rt-active { border-left-color: ' + C.maroon + '; background: ' + RASim.alpha(C.maroon, 0.08) + '; }\n' +
        SC + ' .rt-formula { font-size: 13.5px; color: ' + C.ink + '; font-weight: bold; }\n' +
        SC + ' .rt-wide { grid-column: 1 / -1; }\n' +
        SC + ' .rt-case { text-transform: none; letter-spacing: 0; }\n' +
        SC + ' .rt-stats { grid-template-columns: repeat(2, minmax(0, 1fr)); }\n' +
        SC + ' .rt-mode .sim-seg { display: flex; flex-direction: column; width: 100%; }\n' +
        SC + ' .rt-mode .sim-seg > button { margin: -1px 0 0 0; border-radius: 0; text-align: left; }\n' +
        SC + ' .rt-mode .sim-seg > button:first-child { margin-top: 0; border-radius: 4px 4px 0 0; }\n' +
        SC + ' .rt-mode .sim-seg > button:last-child { border-radius: 0 0 4px 4px; }\n' +
        SC + ' .rt-mats { display: flex; flex-direction: column; gap: 6px; min-width: 0; }\n' +
        '@container (min-width: 715px) { ' + SC + ' .sim-split-main { position: sticky; top: 8px; } }\n' +
        SC + '.sim-narrow .sim-split-main { position: static; }\n');
      function lab(html) { return '<span class="rt-nt">' + html + '</span>'; }
      function col(c, html) { return '<span style="color:' + c + '">' + html + '</span>'; }

      ui.note(api.root, 'Choose three rotation steps and the mode, then compare the bold frame B with the dashed ghost of the other order and read R, T and T⁻¹; drag the picture to orbit the view.');
      const sp = ui.split(api.root, { ratio: 1.5 });

      // ---- main column ----
      const cv = RASim.canvas(sp.main, { aspect: 1.55, minHeight: 250, maxHeight: 560, key: 'scene' });
      // Touch screens: the whole picture is an orbit surface, so a finger on it would never scroll the page. Orbit by
      // touch only when this box is ticked (shown only on devices with a touch screen); mouse and pen always orbit.
      const hasTouch = (typeof navigator !== 'undefined' && navigator.maxTouchPoints > 0) ||
        (typeof window.matchMedia === 'function' && window.matchMedia('(any-pointer: coarse)').matches);
      let touchOrbit = false;
      const rowTouch = ui.row(sp.main);
      rowTouch.classList.add('rt-touch');
      ui.checkbox(rowTouch, { label: 'Orbit by touch (off: a finger on the picture scrolls the page)', checked: false, key: 'touchOrbit',
        onChange: function (v) { touchOrbit = v; update(); } });
      if (!hasTouch) rowTouch.style.display = 'none';
      const animNote = ui.note(sp.main, '');
      animNote.setAttribute('data-key', 'animNote');
      ui.legend(sp.main, [
        { color: C.red, label: 'x<sub>B</sub>' },
        { color: C.green, label: 'y<sub>B</sub>' },
        { color: C.blue, label: 'z<sub>B</sub>' },
        { color: C.muted, label: 'frame A' },
        { color: RASim.alpha(C.ink, 0.45), label: 'ghost of the other order: faded dashed ' + col(RASim.alpha(C.red, 0.75), 'x<sub>B</sub>') + ' ' +
          col(RASim.alpha(C.green, 0.75), 'y<sub>B</sub>') + ' ' + col(RASim.alpha(C.blue, 0.75), 'z<sub>B</sub>'), dash: true },
        { color: C.teal, label: 'd', dash: true },
        { color: C.orange, label: 'point P' },
        { color: C.purple, label: 'equivalent axis k', dash: true },
        { color: C.maroon, label: 'axis being turned about', dash: true }
      ]);

      // ---- side column: controls ----
      const gSteps = ui.group(sp.side, 'Rotation steps (applied 1 → 2 → 3)');
      const segMode = ui.segmented(gSteps, { label: 'Mode', key: 'mode', value: st.mode,
        options: [{ value: 'fixed', label: 'About FIXED axes (pre-multiply)' }, { value: 'current', label: 'About CURRENT axes (post-multiply)' }],
        onChange: function (v) { st.mode = v === 'current' ? 'current' : 'fixed'; userChange(); } });
      segMode.el.classList.add('rt-mode');
      segMode.el.style.width = '100%';
      const stepCtl = [1, 2, 3].map(function (i) {
        const box = ui.el('div', { 'class': 'rt-step' }, gSteps);
        const row = ui.row(box);
        const seg = ui.segmented(row, { label: 'Step ' + i + ' about', key: 'axis' + i, value: st['axis' + i],
          options: [{ value: 'x', label: 'X' }, { value: 'y', label: 'Y' }, { value: 'z', label: 'Z' }],
          onChange: function (v) { st['axis' + i] = v; userChange(); } });
        const sl = ui.slider(row, { label: 'θ<sub>' + i + '</sub>', min: -180, max: 180, step: 1, value: st['angle' + i], unit: '°', key: 'angle' + i,
          onInput: function (v) { st['angle' + i] = v; userChange(); } });
        return { box: box, seg: seg, sl: sl };
      });
      const presetOpts = [{ value: '', label: 'Choose a preset…' }].concat(Object.keys(PRESETS).map(function (k) { return { value: k, label: PRESETS[k].label }; }));
      const selPreset = ui.select(gSteps, { label: 'Preset', key: 'preset', value: '', options: presetOpts,
        onChange: function (v) { applyPreset(v); } });

      const gD = ui.group(sp.side, 'Translation <span class="rt-case">d</span> (origin of B in A)');
      const sD = ['x', 'y', 'z'].map(function (c) {
        return ui.slider(gD, { label: 'd<sub>' + c + '</sub>', min: -2, max: 2, step: 0.1, value: st['d' + c], key: 'd' + c,
          onInput: function (v) { st['d' + c] = v; userChange(); } });
      });
      const gP = ui.group(sp.side, 'Point P fixed in B: <span class="rt-case">p<sub>B</sub></span>');
      const sP = ['x', 'y', 'z'].map(function (c) {
        return ui.slider(gP, { label: 'p<sub>B' + c + '</sub>', min: -1.5, max: 1.5, step: 0.1, value: st['p' + c], key: 'p' + c,
          onInput: function (v) { st['p' + c] = v; userChange(); } });
      });

      const gShow = ui.group(sp.side, 'Show and play');
      const rowShow = ui.row(gShow);
      const cGhost = ui.checkbox(rowShow, { label: 'Ghost of the other order', checked: st.showGhost, key: 'showGhost',
        onChange: function (v) { st.showGhost = v; userChange(); } });
      const cAxis = ui.checkbox(rowShow, { label: 'Equivalent axis', checked: st.showAxis, key: 'showAxis',
        onChange: function (v) { st.showAxis = v; userChange(); } });
      const cBox = ui.checkbox(rowShow, { label: 'Body box', checked: st.showBox, key: 'showBox',
        onChange: function (v) { st.showBox = v; userChange(); } });
      const rowBtn = ui.row(gShow);
      ui.button(rowBtn, { label: 'Play steps', primary: true, key: 'play', title: 'Animate step 1, then 2, then 3',
        onClick: function () { play(); } });
      ui.button(rowBtn, { label: 'Reset', key: 'reset', title: 'Restore every default',
        onClick: function () { reset(); } });

      // ---- side column: read-outs ----
      const rowTag = ui.row(sp.side);
      const tag = ui.tag(rowTag, { text: '', key: 'rule' });
      const fLine = ui.note(sp.side, '');
      fLine.setAttribute('data-key', 'formula');
      fLine.classList.add('rt-formula');

      const stats = ui.stats(sp.side);
      stats.classList.add('rt-stats');
      const tDet = ui.stat(stats, { label: lab('det R'), key: 'detR', title: 'det R = +1 for every rotation matrix' });
      const tTr = ui.stat(stats, { label: lab('trace R = 1 + 2cos θ'), key: 'trace' });
      const tAng = ui.stat(stats, { label: lab('equivalent angle θ'), key: 'eqAngle', title: 'θ = acos((trace R − 1)/2), 0° … 180°' });
      const tOrth = ui.stat(stats, { label: lab('max |RᵀR − I|'), key: 'orth', title: 'largest entry of |RᵀR − I|: zero up to round-off (below 10⁻¹² shows as 0)' });
      const tAx = ui.stat(stats, { label: lab('axis k = (k<sub>x</sub>, k<sub>y</sub>, k<sub>z</sub>)'), key: 'eqAxis' });
      tAx.el.classList.add('rt-wide');
      const tPA = ui.stat(stats, { label: lab('p<sub>A</sub> = R·p<sub>B</sub> + d'), key: 'pA' });
      tPA.el.classList.add('rt-wide');

      const mats = ui.el('div', { 'class': 'rt-mats' }, sp.side);
      const mR = ui.matrix(mats, { label: '<sup>A</sup>R<sub>B</sub> =', key: 'R' });
      const cap = ui.note(mats, 'columns = ' + col(C.red, 'x<sub>B</sub>') + ', ' + col(C.green, 'y<sub>B</sub>') + ', ' + col(C.blue, 'z<sub>B</sub>') + ' expressed in A');
      cap.setAttribute('data-key', 'Rcaption');
      const mT = ui.matrix(mats, { label: 'T =', key: 'T' });
      const mTi = ui.matrix(mats, { label: 'T<sup>−1</sup> =', key: 'Tinv' });

      ui.hints(api.root, [
        'With steps X 90° then Z 90°, switch between FIXED and CURRENT axes: the final frames differ — rotations do not commute. The ghost shows the other answer.',
        'Read the columns of R: column 1 is where ' + col(C.red, 'x<sub>B</sub>') + ' points in frame A, column 2 is ' + col(C.green, 'y<sub>B</sub>') + ', column 3 is ' + col(C.blue, 'z<sub>B</sub>') + '. Compare them with the drawn axes.',
        'Set d = (1, 2, 0): T<sup>−1</sup> is not T<sup>T</sup> — its translation part is −R<sup>T</sup>d.',
        'Whatever the three steps, R is one rotation by θ about a single axis k (purple): trace R = 1 + 2 cos θ.',
        'Press <b>Play steps</b> in each mode and watch which axis each step turns about.'
      ]);

      // ---- state helpers ----
      const anim = api.animate(function (dt) {
        if (!st.animStep) { anim.stop(); update(); return; }
        st.animFrac += dt / STEP_T;
        if (st.animFrac >= 1) {
          const n = nextStep(st.animStep);
          if (n) { st.animStep = n; st.animFrac = 0; } else { anim.stop(); st.animStep = 0; st.animFrac = 0; }
        }
        update();
        cv.drawNow();
      });
      function stepsFull() {
        return [1, 2, 3].map(function (i) { return { axis: st['axis' + i], angle: rad(st['angle' + i]) }; });
      }
      function stepsShown(full) {
        if (!st.animStep) return full;
        return full.map(function (q, i) {
          if (i + 1 < st.animStep) return q;
          return { axis: q.axis, angle: i + 1 === st.animStep ? q.angle * st.animFrac : 0 };
        });
      }
      function nextStep(k) {
        for (let i = k + 1; i <= 3; i++) if (Math.abs(st['angle' + i]) > 1e-9) return i;
        return 0;
      }
      function stopAnim() { if (anim.running) anim.stop(); }
      function userChange() {
        stopAnim();
        st.animStep = 0; st.animFrac = 0;
        selPreset.set('');
        update();
      }
      function play() {
        stopAnim();
        const first = nextStep(0);
        st.animStep = first; st.animFrac = 0;
        update();
        if (first) anim.start();
      }
      function applyPreset(v) {
        const p = PRESETS[v];
        if (!p) { selPreset.set(''); return; }
        stopAnim();
        setControls(p.s);
        st.animStep = 0; st.animFrac = 0;
        selPreset.set(v);
        update();
      }
      function reset() {
        stopAnim();
        setControls(DEFAULTS);
        st.az = DEFAULTS.az; st.el = DEFAULTS.el;
        st.animStep = 0; st.animFrac = 0;
        selPreset.set('');
        update();
      }
      function normAxis(a) { const s = String(a).toLowerCase(); return s === 'x' || s === 'y' || s === 'z' ? s : null; }
      // applies the control-backed keys of s (silently) and returns true if any was present
      function setControls(s) {
        let any = false;
        if (s.mode != null) { st.mode = s.mode === 'current' ? 'current' : 'fixed'; segMode.set(st.mode); any = true; }
        stepCtl.forEach(function (c, i) {
          const ax = s['axis' + (i + 1)], an = s['angle' + (i + 1)];
          if (ax != null) { const a = normAxis(ax); if (a) { st['axis' + (i + 1)] = a; c.seg.set(a); } any = true; }
          if (an != null) { c.sl.set(Number(an)); st['angle' + (i + 1)] = c.sl.get(); any = true; }
        });
        ['x', 'y', 'z'].forEach(function (c, i) {
          if (s['d' + c] != null) { sD[i].set(Number(s['d' + c])); st['d' + c] = sD[i].get(); any = true; }
          if (s['p' + c] != null) { sP[i].set(Number(s['p' + c])); st['p' + c] = sP[i].get(); any = true; }
        });
        if (s.showGhost != null) { st.showGhost = !!s.showGhost; cGhost.set(st.showGhost); any = true; }
        if (s.showAxis != null) { st.showAxis = !!s.showAxis; cAxis.set(st.showAxis); any = true; }
        if (s.showBox != null) { st.showBox = !!s.showBox; cBox.set(st.showBox); any = true; }
        return any;
      }

      // ---- computation (one source of truth for the picture and the read-outs) ----
      let cur = null;
      function compute() {
        const full = stepsFull();
        const shown = stepsShown(full);
        const R = st.animStep ? model.partial(full, st.mode, st.animStep, st.animFrac) : model.compose(full, st.mode);
        const G = model.compose(full, st.mode === 'fixed' ? 'current' : 'fixed');
        const d = [st.dx, st.dy, st.dz];
        const T = model.homogeneous(R, d);
        const Ti = model.invHomogeneous(T);
        const pA = model.apply(T, [st.px, st.py, st.pz]);
        const aa = model.axisAngle(R);
        const RtR = model.mul(model.transpose(R), R);
        let orth = 0;
        for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) orth = Math.max(orth, Math.abs(RtR[i][j] - (i === j ? 1 : 0)));
        let turn = null;
        if (st.animStep) {
          const q = full[st.animStep - 1];
          const idx = 'xyz'.indexOf(q.axis);
          let dir;
          if (st.mode === 'fixed') dir = [0, 1, 2].map(function (i) { return i === idx ? 1 : 0; });
          else { const Rb = model.partial(full, st.mode, st.animStep, 0); dir = [Rb[0][idx], Rb[1][idx], Rb[2][idx]]; }
          turn = { dir: dir, idx: idx, sign: q.angle >= 0 ? 1 : -1, step: st.animStep, angle: q.angle, now: q.angle * st.animFrac };
        }
        return { full: full, shown: shown, R: R, G: G, d: d, T: T, Ti: Ti, pA: pA, aa: aa, orth: orth, turn: turn,
          formula: model.formula(shown, st.mode) };
      }
      function fmtDeg(r) { return fmt(Math.round(M.deg(r) * 10) / 10, 6) + '°'; }
      function vec(v) { return '(' + v.map(function (x) { return fmtFixed(x, 3); }).join(', ') + ')'; }
      function axisName(t) {
        const L = 'xyz'.charAt(t.idx);
        return st.mode === 'fixed' ? L.toUpperCase() + '<sub>A</sub> (fixed axis of A)' : L + '<sub>B</sub> (current axis of B)';
      }

      function update() {
        cur = compute();
        const R = cur.R;
        fLine.textContent = cur.formula;
        tag.set(st.mode === 'fixed' ? 'Fixed axes → R = R₃·R₂·R₁' : 'Current axes → R = R₁·R₂·R₃', '');
        mR.set(R, { decimals: 3, colColors: AXC });
        mT.set(cur.T, { decimals: 3, colColors: [C.red, C.green, C.blue, C.teal] });
        mTi.set(cur.Ti, { decimals: 3 });
        tDet.set(fmtFixed(model.det3(R), 3));
        tTr.set(fmtFixed(model.trace(R), 3));
        tAng.set(fmtDeg(cur.aa.angle));
        tAx.set(cur.aa.axis ? vec(cur.aa.axis) : '—');
        tPA.set(vec(cur.pA));
        tOrth.set(fmt(cur.orth, 2));
        tOrth.setState(cur.orth < 1e-9 ? 'ok' : 'bad');
        stepCtl.forEach(function (c, i) { c.box.classList.toggle('rt-active', st.animStep === i + 1); });
        if (cur.turn) {
          const t = cur.turn;
          animNote.innerHTML = '<b>Step ' + t.step + '</b>: R' + 'xyz'.charAt(t.idx) + '(' + fmtDeg(t.angle) + ') about ' + axisName(t) +
            ' — now at ' + fmtDeg(t.now) + '.';
        } else {
          animNote.innerHTML = (hasTouch && !touchOrbit ? 'Drag the picture with the mouse (or tick <b>Orbit by touch</b>) to orbit the view. ' :
            'Drag the picture to orbit the view. ') + '<b>Play steps</b> shows each step turning frame B.';
        }
        cv.redraw();
      }

      // ---- view and drawing ----
      function dot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
      function add(a, b, s) { return [a[0] + s * b[0], a[1] + s * b[1], a[2] + s * b[2]]; }
      function makeView(w, h) {
        const a = rad(st.az), e = rad(st.el);
        const ca = Math.cos(a), sa = Math.sin(a), ce = Math.cos(e), se = Math.sin(e);
        const er = [-sa, ca, 0], eu = [-se * ca, -se * sa, ce], ec = [ce * ca, ce * sa, se];
        let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
        function box(p, rx, ry) {
          const u = dot(p, er), v = dot(p, eu);
          x0 = Math.min(x0, u - rx); x1 = Math.max(x1, u + rx); y0 = Math.min(y0, v - ry); y1 = Math.max(y1, v + ry);
        }
        // the ground grid's circumscribed circle (az-independent fit); on a phone-width canvas the grid's far corners
        // may run off the edges so that the frames get a usable size
        const G = w < 420 ? 2 : 2 * Math.SQRT2;
        box([0, 0, 0], G, G * Math.abs(se));
        box([1.6, 0, 0], 0, 0); box([0, 1.6, 0], 0, 0); box([0, 0, 1.6], 0, 0);
        const rb = Math.max(1.3, Math.hypot(st.px, st.py, st.pz));   // everything attached to B (rotation-independent)
        box([st.dx, st.dy, st.dz], rb, rb);
        const m = w < 420 ? 18 : 24;
        const sc = Math.min((w - 2 * m) / (x1 - x0), (h - 2 * m) / (y1 - y0));
        const cx = w / 2 - sc * (x0 + x1) / 2, cy = h / 2 + sc * (y0 + y1) / 2;
        return {
          sc: sc, c: ec,
          P: function (p) { return [cx + sc * dot(p, er), cy - sc * dot(p, eu)]; },
          depth: function (p) { return dot(p, ec); }
        };
      }

      // label placement: obstacles are thick segments and already placed boxes (as in two-link-arm)
      function segHitsRect(x1, y1, x2, y2, l, t, r, b) {   // Liang–Barsky
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
      function makeLayout(w, h) {
        const segs = [], boxes = [];
        return {
          seg: function (a, b, hw) { segs.push([a[0], a[1], b[0], b[1], hw]); },
          dot: function (a, r) { segs.push([a[0], a[1], a[0], a[1], r]); },
          place: function (hw, hh, cands) {
            let best = null;
            cands.forEach(function (c) {
              const bx = { l: c.x - hw, r: c.x + hw, t: c.y - hh, b: c.y + hh };
              let n = 0;
              segs.forEach(function (s) { if (segHitsRect(s[0], s[1], s[2], s[3], bx.l - s[4], bx.t - s[4], bx.r + s[4], bx.b + s[4])) n++; });
              boxes.forEach(function (o) { if (bx.l < o.r + 2 && bx.r > o.l - 2 && bx.t < o.b + 2 && bx.b > o.t - 2) n += 3; });
              let cost = c.cost + 1000 * n;
              if (bx.l < 2 || bx.r > w - 2 || bx.t < 2 || bx.b > h - 2) cost += 100000;
              if (!best || cost < best.cost) best = { cost: cost, x: c.x, y: c.y, bx: bx };
            });
            return best;
          },
          keep: function (bx) { boxes.push(bx); }
        };
      }
      // candidate label centres around (cx, cy) in directions mid ± i·step (radians, y up), just clear of radius r
      function around(cx, cy, r, mid, step, n, hw, hh) {
        const out = [];
        for (let i = 0; i <= n; i++) {
          [i, -i].forEach(function (j, s) {
            if (i === 0 && s) return;
            const u = mid + j * step, ux = Math.cos(u), uy = Math.sin(u);
            [0, 8, 18, 32].forEach(function (extra, ring) {
              const dd = r + 3 + hw * Math.abs(ux) + hh * Math.abs(uy) + extra;
              out.push({ x: cx + dd * ux, y: cy - dd * uy, cost: 5 * i + 9 * ring });
            });
          });
        }
        return out;
      }
      // a label is a list of parts [text, font, dy]; measured once per draw
      const F_MAIN = 'bold 13px sans-serif', F_SUB = 'bold 11px sans-serif', F_TXT = '12px sans-serif', F_A = '13px sans-serif', F_AS = '11px sans-serif';
      function measure(ctx, parts) {
        let wsum = 0;
        parts.forEach(function (p) { ctx.font = p[1]; p.w = ctx.measureText(p[0]).width; wsum += p.w; });
        return wsum;
      }
      function drawLabel(ctx, lb) {
        const x0 = lb.x - lb.w / 2;
        ctx.fillStyle = RASim.alpha(WHITE, 0.86);
        ctx.beginPath();
        if (ctx.roundRect) ctx.roundRect(x0 - 3, lb.y - 9, lb.w + 6, 18, 3); else ctx.rect(x0 - 3, lb.y - 9, lb.w + 6, 18);
        ctx.fill();
        let x = x0;
        ctx.textAlign = 'left'; ctx.textBaseline = 'middle'; ctx.fillStyle = lb.color;
        lb.parts.forEach(function (p) { ctx.font = p[1]; ctx.fillText(p[0], x, lb.y + p[2]); x += p.w; });
      }

      cv.draw = function (ctx, w, h) {
        const V = makeView(w, h), P = V.P;
        const R = cur.R, d = cur.d;
        const O = [0, 0, 0];
        const lay = makeLayout(w, h);
        const prims = [];
        function prim(depth, f) { prims.push({ z: depth, f: f }); }
        function line(a, b, color, width, dash) {
          ctx.strokeStyle = color; ctx.lineWidth = width; ctx.setLineDash(dash || []);
          ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke(); ctx.setLineDash([]);
        }
        const colR = function (j) { return [R[0][j], R[1][j], R[2][j]]; };
        const Od = P(d);

        // 1. ground grid (always at the back)
        ctx.lineCap = 'butt';
        for (let i = -4; i <= 4; i++) {
          const v = i * 0.5, edge = Math.abs(i) === 4;
          line(P([v, -2, 0]), P([v, 2, 0]), edge ? C.faint : C.grid, edge ? 1 : 1, null);
          line(P([-2, v, 0]), P([2, v, 0]), edge ? C.faint : C.grid, edge ? 1 : 1, null);
        }

        // 2. frame A
        const tipsA = [];
        for (let j = 0; j < 3; j++) {
          const e = [0, 0, 0]; e[j] = 1.6;
          const a2 = P(O), b2 = P(e);
          tipsA.push(b2);
          lay.seg(a2, b2, 2);
          prim(V.depth(add(O, e, 0.5)), function () { D.arrow(ctx, a2[0], a2[1], b2[0], b2[1], { color: C.muted, width: 1.5, head: 8 }); });
        }

        // 3. d vector
        const dLen = Math.hypot(d[0], d[1], d[2]);
        if (dLen > 1e-9) {
          const a2 = P(O);
          lay.seg(a2, Od, 2);
          prim(V.depth(add(O, d, 0.5)), function () { D.arrow(ctx, a2[0], a2[1], Od[0], Od[1], { color: C.teal, width: 1.75, head: 9, dash: [6, 4] }); });
        }

        // 4. body box: translucent faces + edges
        if (st.showBox) {
          const corner = function (a, b, c) { return add(add(add(d, colR(0), a * BOX[0]), colR(1), b * BOX[1]), colR(2), c * BOX[2]); };
          const faces = [
            [[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]], [[0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]],
            [[0, 0, 0], [1, 0, 0], [1, 0, 1], [0, 0, 1]], [[0, 1, 0], [1, 1, 0], [1, 1, 1], [0, 1, 1]],
            [[0, 0, 0], [0, 1, 0], [0, 1, 1], [0, 0, 1]], [[1, 0, 0], [1, 1, 0], [1, 1, 1], [1, 0, 1]]
          ];
          faces.forEach(function (f) {
            const p3 = f.map(function (q) { return corner(q[0], q[1], q[2]); });
            const p2 = p3.map(P);
            const cz = (V.depth(p3[0]) + V.depth(p3[2])) / 2;
            prim(cz - 0.05, function () {
              ctx.fillStyle = RASim.alpha(C.gold, 0.10);
              ctx.beginPath(); ctx.moveTo(p2[0][0], p2[0][1]);
              for (let i = 1; i < 4; i++) ctx.lineTo(p2[i][0], p2[i][1]);
              ctx.closePath(); ctx.fill();
            });
          });
          for (let a = 0; a < 2; a++) for (let b = 0; b < 2; b++) {
            [[[0, a, b], [1, a, b]], [[a, 0, b], [a, 1, b]], [[a, b, 0], [a, b, 1]]].forEach(function (e) {
              const p = corner(e[0][0], e[0][1], e[0][2]), q = corner(e[1][0], e[1][1], e[1][2]);
              const p2 = P(p), q2 = P(q);
              lay.seg(p2, q2, 1);
              prim((V.depth(p) + V.depth(q)) / 2, function () { line(p2, q2, RASim.alpha(C.ink, 0.45), 1.25, null); });
            });
          }
        }

        // 5. ghost frame (other order)
        if (st.showGhost) {
          for (let j = 0; j < 3; j++) {
            const g = [cur.G[0][j], cur.G[1][j], cur.G[2][j]];
            const t2 = P(add(d, g, 1));
            lay.seg(Od, t2, 2);
            prim(V.depth(add(d, g, 0.5)), function () { D.arrow(ctx, Od[0], Od[1], t2[0], t2[1], { color: RASim.alpha(AXC[j], 0.6), width: 2, head: 9, dash: [5, 4] }); });
          }
        }

        // 6. frame B
        const tipsB = [];
        for (let j = 0; j < 3; j++) {
          const cj = colR(j), t2 = P(add(d, cj, 1));
          tipsB.push(t2);
          lay.seg(Od, t2, 3);
          prim(V.depth(add(d, cj, 0.5)) + 0.02, function () { D.arrow(ctx, Od[0], Od[1], t2[0], t2[1], { color: AXC[j], width: 3, head: 11 }); });
        }

        // 7. equivalent axis through B's origin
        let kTip = null;
        if (st.showAxis && cur.aa.axis) {
          const k = cur.aa.axis;
          const a2 = P(add(d, k, -1.3)), b2 = P(add(d, k, 1.3));
          kTip = { tip: b2, from: a2 };
          lay.seg(a2, b2, 2);
          prim(V.depth(d) + 0.01, function () { D.arrow(ctx, a2[0], a2[1], b2[0], b2[1], { color: C.purple, width: 2, head: 10, dash: [7, 4] }); });
        }

        // 8. the axis being turned about (animation) with a right-hand sense arrow
        let turnTip = null;
        if (cur.turn) {
          const t = cur.turn, u = t.dir;
          const a2 = P(add(d, u, -1.45)), b2 = P(add(d, u, 1.45));
          turnTip = { tip: b2, from: a2 };
          lay.seg(a2, b2, 3);
          prim(V.depth(d) + 0.015, function () { line(a2, b2, RASim.alpha(C.maroon, 0.85), 4, [10, 6]); });
          // circular arrow around the axis near its + end, in the direction of the step's angle
          const ref = Math.abs(u[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0];
          let e1 = [u[1] * ref[2] - u[2] * ref[1], u[2] * ref[0] - u[0] * ref[2], u[0] * ref[1] - u[1] * ref[0]];
          const n1 = Math.hypot(e1[0], e1[1], e1[2]); e1 = [e1[0] / n1, e1[1] / n1, e1[2] / n1];
          const e2 = [u[1] * e1[2] - u[2] * e1[1], u[2] * e1[0] - u[0] * e1[2], u[0] * e1[1] - u[1] * e1[0]];   // u × e1
          const ctr = add(d, u, 1.15), rr = 0.22, pts = [];
          for (let i = 0; i <= 24; i++) {
            const th = t.sign * 1.5 * Math.PI * i / 24;
            pts.push(P(add(add(ctr, e1, rr * Math.cos(th)), e2, rr * Math.sin(th))));
          }
          prim(V.depth(ctr) + 0.03, function () {
            ctx.strokeStyle = C.maroon; ctx.lineWidth = 2; ctx.setLineDash([]);
            ctx.beginPath(); ctx.moveTo(pts[0][0], pts[0][1]);
            for (let i = 1; i < pts.length - 1; i++) ctx.lineTo(pts[i][0], pts[i][1]);
            ctx.stroke();
            const p = pts[pts.length - 2], q = pts[pts.length - 1];
            D.arrow(ctx, p[0], p[1], q[0], q[1], { color: C.maroon, width: 2, head: 8 });
          });
          for (let i = 0; i < pts.length - 1; i += 4) lay.seg(pts[i], pts[Math.min(pts.length - 1, i + 4)], 2);
        }

        // 9. point P
        const pA2 = P(cur.pA);
        lay.seg(Od, pA2, 1); lay.dot(pA2, 6);
        prim(V.depth(cur.pA), function () {
          line(Od, pA2, C.orange, 1.25, null);
          ctx.fillStyle = C.orange; ctx.strokeStyle = WHITE; ctx.lineWidth = 1.5;
          ctx.beginPath(); ctx.arc(pA2[0], pA2[1], 4.5, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
        });
        lay.dot(Od, 5);

        prims.sort(function (a, b) { return a.z - b.z; });
        prims.forEach(function (p) { p.f(); });
        ctx.setLineDash([]);
        // origin of B
        ctx.fillStyle = C.ink;
        ctx.beginPath(); ctx.arc(Od[0], Od[1], 3, 0, 2 * Math.PI); ctx.fill();

        // 10. labels, most important first; optional ones are dropped instead of overlapping
        const labels = [], leaders = [];
        function addLabel(parts, color, anchor, from, r, optional) {
          const wsum = measure(ctx, parts);
          const hw = wsum / 2 + 3, hh = 9;
          const vx = anchor[0] - from[0], vy = from[1] - anchor[1];
          const ang = Math.hypot(vx, vy) > 6 ? Math.atan2(vy, vx) : Math.PI / 4;
          const best = lay.place(hw, hh, around(anchor[0], anchor[1], r, ang, Math.PI / 10, 10, hw, hh));
          if (optional && best.cost >= 1000) return;
          lay.keep(best.bx);
          labels.push({ parts: parts, color: color, x: best.x, y: best.y, w: wsum });
          // a label that ended up away from what it names gets a thin leader line to it
          const nx = M.clamp(anchor[0], best.bx.l, best.bx.r), ny = M.clamp(anchor[1], best.bx.t, best.bx.b);
          const dl = Math.hypot(nx - anchor[0], ny - anchor[1]);
          if (dl > r + 14) leaders.push([anchor[0] + (nx - anchor[0]) * (r + 3) / dl, anchor[1] + (ny - anchor[1]) * (r + 3) / dl, nx, ny, color]);
        }
        const names = ['x', 'y', 'z'];
        for (let j = 0; j < 3; j++) addLabel([[names[j], F_MAIN, 0], ['B', F_SUB, 4]], AXC[j], tipsB[j], Od, 2, false);
        addLabel([['P', F_MAIN, 0]], C.orange, pA2, Od, 4, false);
        for (let j = 0; j < 3; j++) addLabel([[names[j].toUpperCase(), F_A, 0], ['A', F_AS, 4]], C.muted, tipsA[j], P(O), 2, false);
        if (turnTip) {
          const t = cur.turn, L = 'xyz'.charAt(t.idx);
          const parts = st.mode === 'fixed' ? [['about ', F_TXT, 0], [L.toUpperCase(), F_MAIN, 0], ['A', F_SUB, 4]] : [['about ', F_TXT, 0], [L, F_MAIN, 0], ['B', F_SUB, 4]];
          addLabel(parts, C.maroon, turnTip.tip, turnTip.from, 2, w < 420);
        }
        if (kTip) addLabel([['k, θ = ' + fmtDeg(cur.aa.angle), 'bold 12px sans-serif', 0]], C.purple, kTip.tip, kTip.from, 3, w < 420);
        if (dLen > 1e-9) {
          const m2 = P(add(O, d, 0.5)), a0 = P(O);
          const L2 = Math.hypot(Od[0] - a0[0], Od[1] - a0[1]);
          if (L2 > 24) {
            const nx = -(Od[1] - a0[1]) / L2, ny = (Od[0] - a0[0]) / L2;    // screen normal
            addLabel([['d', F_MAIN, 0]], C.teal, m2, [m2[0] - nx, m2[1] - ny], 2, true);
          }
        }
        leaders.forEach(function (q) { line([q[0], q[1]], [q[2], q[3]], RASim.alpha(q[4], 0.7), 1, null); });
        labels.forEach(function (lb) { drawLabel(ctx, lb); });
      };

      cv.onDrag({
        hit: function (x, y, info) {
          if (info && info.pointerType === 'touch' && !touchOrbit) return null;   // let the finger scroll the page
          return { x0: x, y0: y, az0: st.az, el0: st.el };
        },
        move: function (x, y, t) {
          st.az = M.wrapDeg(t.az0 - (x - t.x0) * 0.5);
          st.el = M.clamp(t.el0 + (y - t.y0) * 0.5, -85, 85);
          cv.drawNow();
        },
        cursor: function () { return 'grab'; }
      });

      function getState() {
        return { mode: st.mode, axis1: st.axis1, angle1: st.angle1, axis2: st.axis2, angle2: st.angle2, axis3: st.axis3, angle3: st.angle3,
          dx: st.dx, dy: st.dy, dz: st.dz, px: st.px, py: st.py, pz: st.pz,
          showGhost: st.showGhost, showAxis: st.showAxis, showBox: st.showBox,
          az: st.az, el: st.el, animStep: st.animStep, animFrac: st.animFrac };
      }
      function setState(s) {
        s = s || {};
        stopAnim();
        let other = setControls(s);
        if (s.az != null && isFinite(Number(s.az))) { st.az = M.wrapDeg(Number(s.az)); other = true; }
        if (s.el != null && isFinite(Number(s.el))) { st.el = M.clamp(Number(s.el), -85, 85); other = true; }
        if (other) selPreset.set('');
        if (s.animStep != null) {
          const k = Math.round(Number(s.animStep));
          st.animStep = k >= 1 && k <= 3 ? k : 0;
          if (!st.animStep) st.animFrac = 0;
        } else if (other) { st.animStep = 0; st.animFrac = 0; }
        if (s.animFrac != null && st.animStep && isFinite(Number(s.animFrac))) st.animFrac = M.clamp(Number(s.animFrac), 0, 1);
        update();
        cv.drawNow();
      }

      update();
      return { getState: getState, setState: setState };
    }
  });
})();
