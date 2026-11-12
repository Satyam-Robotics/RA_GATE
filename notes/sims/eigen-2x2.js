// eigen-2x2 — "Eigenvalues and eigenvectors of a 2×2 matrix"
// Sits beside notes/src/01_math.html, section "Eigenvalues and eigenvectors" (under "1. Linear Algebra").
//
// SIGN CONVENTIONS
//   A = [[a, b], [c, d]] acts on column vectors: A·(x, y) = (a·x + b·y, c·x + d·y). x to the right, y up, equal axes.
//   Columns of A are the images of the basis vectors: A e₁ = (a, c), A e₂ = (b, d).
//   tr = a + d, det = ad − bc, characteristic equation det(A − λI) = λ² − (tr)λ + det = 0,
//   disc = tr² − 4·det (computed as the identical (a − d)² + 4bc, which has no cancellation for symmetric A).
//   Classification with eps = 1e-9·max(1, tr², |det|):
//     disc > eps    'real-distinct'       λ₁ = (tr + √disc)/2 > λ₂ = (tr − √disc)/2 (the smaller-magnitude root is
//                                         taken as det/λ to avoid cancellation; same value)
//     |disc| ≤ eps  'repeated-scalar'     λ = tr/2, |b| ≤ 1e-12 and |c| ≤ 1e-12: A = λI, every vector is an eigenvector
//                   'repeated-defective'  λ = tr/2, one eigenvector direction only (alg. mult. 2, geom. mult. 1)
//     disc < −eps   'complex'             λ = tr/2 ± j√(−disc)/2 (+imaginary part first); no real eigenvector
//   Eigenvector of a real λ: (b, λ − a) and (λ − d, c) are both solutions of (A − λI)v = 0 (parallel whenever both are
//   non-zero); the one with the larger norm is used (this is the specification's rule "(b, λ − a) if |b| > 1e-12,
//   else (λ − d, c)" made robust for tiny b, see the report). Diagonal A (|b|, |c| ≤ 1e-12): (1, 0) for λ = a and
//   (0, 1) for λ = d. Vectors are unit length, sign fixed so that the first non-zero component (|·| > 1e-12) is > 0.
//   Morph: M(t) = (1 − t)·I + t·A, t ∈ [0, 1]. The grid, unit square, unit circle and basis arrows show M(t);
//   the eigen-directions and A·x always use the full A.
//   Angles: the model returns radians; the UI shows degrees. x is an eigenvector when it lies within 2° of the LINE
//   of an eigenvector (v or −v); for 'repeated-scalar' every non-zero x qualifies.
//   flips (orientation reversed) = det < 0 and not singular (|det| < 1e-12 counts as det = 0: round-off on the
//   0.1 grid must not report a flip for a singular matrix).
//
// STATE KEYS (getState / setState)
//   a, b, c, d (matrix entries; sliders −3…3, presets / setState up to ±10 widen that slider while needed), t (morph, 0…1), vx, vy (test vector x, each clamped to −4…4),
//   showGrid, showCircle, showEigen (booleans). Every setState stops a running morph animation.
//
// STAT KEYS:   trace, sum, det, prod, charEq, lambda1, lambda2, v1, v2, angle, stretch
// TAG KEYS:    type   (REAL, DISTINCT / REPEATED — DEFECTIVE (1 EIGENVECTOR) / REPEATED — EVERY VECTOR /
//                      COMPLEX — NO REAL EIGENVECTOR)
// MATRIX KEYS: A
// DATA-KEY:    props (property line), eigMsg (eigenvector detection message)
// CANVAS KEY:  plane
// CONTROL KEYS: a, b, c, d, preset, t, play, snapV1, snapV2, showGrid, showCircle, showEigen, reset
(function () {
  'use strict';
  const RASim = globalThis.RASim;
  const M = RASim.math;

  const ID = 'eigen-2x2';
  const ZERO = 1e-12;

  // unit vector, sign fixed: first non-zero component positive; −0 cleaned
  function unitFixed(x, y) {
    const n = Math.hypot(x, y);
    let u = x / n, v = y / n;
    if (Math.abs(u) > ZERO ? u < 0 : v < 0) { u = -u; v = -v; }
    return [u + 0, v + 0];
  }

  // eigenvector of a real eigenvalue l of a non-diagonal matrix
  function eigvec(a, b, c, d, l) {
    const u0 = b, u1 = l - a, w0 = l - d, w1 = c;
    if (u0 * u0 + u1 * u1 >= w0 * w0 + w1 * w1) return unitFixed(u0, u1);
    return unitFixed(w0, w1);
  }

  function eig(a, b, c, d) {
    a = Number(a); b = Number(b); c = Number(c); d = Number(d);
    const trace = a + d, det = a * d - b * c;
    const disc = (a - d) * (a - d) + 4 * b * c;
    const eps = 1e-9 * Math.max(1, trace * trace, Math.abs(det));
    const diag = Math.abs(b) <= ZERO && Math.abs(c) <= ZERO;
    let type, values, vectors;
    if (disc > eps) {
      type = 'real-distinct';
      const s = Math.sqrt(disc);
      let l1, l2;
      if (trace >= 0) { l1 = (trace + s) / 2; l2 = l1 !== 0 ? det / l1 : (trace - s) / 2; }
      else { l2 = (trace - s) / 2; l1 = det / l2; }
      values = [{ re: l1, im: 0 }, { re: l2, im: 0 }];
      if (diag) {
        const firstIsA = Math.abs(l1 - a) <= Math.abs(l1 - d);
        vectors = firstIsA ? [[1, 0], [0, 1]] : [[0, 1], [1, 0]];
      } else vectors = [eigvec(a, b, c, d, l1), eigvec(a, b, c, d, l2)];
    } else if (disc >= -eps) {
      const l = trace / 2;
      values = [{ re: l, im: 0 }, { re: l, im: 0 }];
      if (diag) { type = 'repeated-scalar'; vectors = [[1, 0], [0, 1]]; }
      else { type = 'repeated-defective'; vectors = [eigvec(a, b, c, d, l), null]; }
    } else {
      type = 'complex';
      const im = Math.sqrt(-disc) / 2;
      values = [{ re: trace / 2, im: im }, { re: trace / 2, im: -im }];
      vectors = [null, null];
    }
    return { trace: trace, det: det, disc: disc, type: type, values: values, vectors: vectors };
  }

  function apply(a, b, c, d, x, y) { return [a * x + b * y, c * x + d * y]; }

  function morph(a, b, c, d, t) { return [(1 - t) + t * a + 0, t * b + 0, t * c + 0, (1 - t) + t * d + 0]; }   // + 0: no −0

  function props(a, b, c, d) {
    const det = a * d - b * c;
    const singular = Math.abs(det) < ZERO;
    const orthogonal = Math.abs(a * a + c * c - 1) <= 1e-9 && Math.abs(b * b + d * d - 1) <= 1e-9 && Math.abs(a * b + c * d) <= 1e-9;
    const allZero = Math.abs(a) <= ZERO && Math.abs(b) <= ZERO && Math.abs(c) <= ZERO && Math.abs(d) <= ZERO;
    return {
      symmetric: Math.abs(b - c) <= ZERO,
      singular: singular,
      orthogonal: orthogonal,
      diagonalizable: eig(a, b, c, d).type !== 'repeated-defective',
      rank: allZero ? 0 : singular ? 1 : 2,
      areaScale: Math.abs(det),
      flips: det < 0 && !singular
    };
  }

  // angle between two vectors in [0, π]; NaN when either has zero length
  function angleBetween(x1, y1, x2, y2) {
    if (Math.hypot(x1, y1) < ZERO || Math.hypot(x2, y2) < ZERO) return NaN;
    return Math.atan2(Math.abs(x1 * y2 - y1 * x2), x1 * x2 + y1 * y2);
  }

  // 'λ² − 4λ + 3 = 0' (real minus sign, zero terms omitted, coefficient 1 omitted)
  function charEq(a, b, c, d) {
    const tr = Number(a) + Number(d), det = a * d - b * c;
    const MINUS = '−';
    let s = 'λ²';
    const mt = RASim.fmt(Math.abs(tr), 4);
    if (mt !== '0') s += (tr > 0 ? ' ' + MINUS + ' ' : ' + ') + (mt === '1' ? '' : mt) + 'λ';
    const md = RASim.fmt(Math.abs(det), 4);
    if (md !== '0') s += (det < 0 ? ' ' + MINUS + ' ' : ' + ') + md;
    return s + ' = 0';
  }

  const model = { eig: eig, apply: apply, morph: morph, props: props, angleBetween: angleBetween, charEq: charEq };

  const DEFAULTS = { a: 2, b: 1, c: 1, d: 2, t: 1, vx: 1, vy: 0.5, showGrid: true, showCircle: true, showEigen: true };
  const PRESETS = [
    { value: 'symmetric', label: 'Symmetric [[2,1],[1,2]]', m: [2, 1, 1, 2] },
    { value: 'shear', label: 'Shear [[1,1],[0,1]] — one eigenvector only', m: [1, 1, 0, 1] },
    { value: 'rotation', label: 'Rotation 90° [[0,−1],[1,0]] — complex λ', m: [0, -1, 1, 0] },
    { value: 'scaling', label: 'Scaling [[2,0],[0,0.5]]', m: [2, 0, 0, 0.5] },
    { value: 'singular', label: 'Singular [[1,2],[2,4]] — λ = 0', m: [1, 2, 2, 4] },
    { value: 'reflection', label: 'Reflection [[0,1],[1,0]]', m: [0, 1, 1, 0] },
    { value: 'projection', label: 'Projection [[1,0],[0,0]]', m: [1, 0, 0, 0] },
    { value: 'uniform', label: 'Uniform scaling [[2,0],[0,2]] — every vector', m: [2, 0, 0, 2] }
  ];
  const EIG_TOL = 2 * Math.PI / 180;      // x within 2° of an eigenvector's line
  const ENTRY_MAX = 10;                   // matrix entries outside the −3…3 sliders (presets, setState) are limited to ±10
  const XLIM = 4;                         // x-range ±4; test vector components clamped to ±4

  RASim.register(ID, {
    title: 'Eigenvalues and eigenvectors of a 2×2 matrix',
    model: model,
    mount: function (api) {
      const ui = RASim.ui, C = RASim.colors, D = RASim.draw;
      const fmt = RASim.fmt, fmtFixed = RASim.fmtFixed;
      const st = Object.assign({}, DEFAULTS);
      const SC = '.sim[data-sim="' + ID + '"]';

      RASim.css(ID,
        SC + ' .eg-nt { text-transform: none; letter-spacing: 0; font-size: 11.5px; }\n' +
        SC + ' .eg-case { text-transform: none; letter-spacing: 0; }\n' +
        SC + ' .eg-mat { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 2px 14px; }\n' +
        SC + ' .eg-mat .sim-slider { grid-template-columns: minmax(0, 1fr) 4.4em; grid-template-areas: "label num" "range range"; }\n' +
        SC + ' .eg-mat .sim-num { width: 4.4em; }\n' +
        SC + ' .eg-mat .sim-label { font-weight: bold; font-style: italic; }\n' +
        SC + ' .eg-stats { grid-template-columns: repeat(2, minmax(0, 1fr)); }\n' +
        SC + ' .eg-wide { grid-column: 1 / -1; }\n' +
        SC + ' .eg-msg { font-size: 13px; color: ' + C.ink + '; margin: 0; padding: 5px 8px; border-radius: 4px; border: 1px solid ' + C.grid + '; background: #ffffff; }\n' +
        SC + ' .eg-msg.eg-yes { border-color: ' + C.lightGreen + '; background: ' + RASim.alpha(C.lightGreen, 0.25) + '; color: ' + C.green + '; font-weight: bold; }\n' +
        SC + ' .eg-select select { width: 100%; }\n' +
        SC + ' .eg-select { display: flex; width: 100%; }\n');
      function lab(html) { return '<span class="eg-nt">' + html + '</span>'; }

      ui.note(api.root, 'Drag the blue test vector <b>x</b> (or move <i>a</i>, <i>b</i>, <i>c</i>, <i>d</i>) until <b>Ax</b> lies on the same line as x: that direction is an eigenvector of A, and |Ax|/|x| is |λ|.');
      const sp = ui.split(api.root, { ratio: 1.5 });

      // ---- main column: canvas, detection message, legend ----
      const cv = RASim.canvas(sp.main, { aspect: 1.08, minHeight: 240, maxHeight: 560, key: 'plane' });
      const eigMsg = ui.note(sp.main, '');
      eigMsg.setAttribute('data-key', 'eigMsg');
      eigMsg.classList.add('eg-msg');
      ui.legend(sp.main, [
        { color: C.lightBlue, label: 'grid under M(t)' },
        { color: C.lightGreen, label: 'unit square → area |det| (red if det &lt; 0)' },
        { color: C.purple, label: 'unit circle → ellipse' },
        { color: C.red, label: 'A e₁ = 1st column' },
        { color: C.green, label: 'A e₂ = 2nd column' },
        { color: C.maroon, label: 'eigen-direction of λ₁', dash: true },
        { color: C.teal, label: 'eigen-direction of λ₂', dash: true },
        { color: C.blue, label: 'x (drag the tip)' },
        { color: C.orange, label: 'Ax (always the full A)' }
      ]);

      // ---- side column ----
      const gA = ui.group(sp.side, 'Matrix <span class="eg-case">A = [[a, b], [c, d]]</span>');
      const grid = ui.el('div', { 'class': 'eg-mat' }, gA);
      function entrySlider(k) {
        return ui.slider(grid, { label: k, min: -3, max: 3, step: 0.1, value: st[k], key: k,
          onInput: function (v) { st[k] = v; matrixChanged(); } });
      }
      const sA = entrySlider('a'), sB = entrySlider('b'), sC = entrySlider('c'), sD = entrySlider('d');
      const selPreset = ui.select(gA, { label: 'Preset', key: 'preset', value: 'symmetric',
        options: PRESETS.map(function (p) { return { value: p.value, label: p.label }; })
          .concat([{ value: 'custom', label: 'Custom (set with the sliders)' }]),
        onChange: function (v) {
          const pr = PRESETS.filter(function (p) { return p.value === v; })[0];
          if (pr) setMatrix(pr.m);
          matrixChanged();
        } });
      selPreset.el.classList.add('eg-select');

      const rowA = ui.row(sp.side);
      const mat = ui.matrix(rowA, { label: 'A =', key: 'A' });
      const tag = ui.tag(rowA, { text: '', key: 'type' });
      const propsNote = ui.note(sp.side, '');
      propsNote.setAttribute('data-key', 'props');

      const stats = ui.stats(sp.side);
      stats.classList.add('eg-stats');
      const tTr = ui.stat(stats, { label: lab('trace = a + d'), key: 'trace' });
      const tSum = ui.stat(stats, { label: lab('λ₁ + λ₂'), key: 'sum' });
      const tDet = ui.stat(stats, { label: lab('det = ad − bc'), key: 'det' });
      const tProd = ui.stat(stats, { label: lab('λ₁·λ₂'), key: 'prod' });
      const tChar = ui.stat(stats, { label: lab('characteristic equation'), key: 'charEq' });
      tChar.el.classList.add('eg-wide');
      const tL1 = ui.stat(stats, { label: lab('λ₁'), key: 'lambda1' });
      const tL2 = ui.stat(stats, { label: lab('λ₂'), key: 'lambda2' });
      const tV1 = ui.stat(stats, { label: lab('v₁ (unit)'), key: 'v1' });
      const tV2 = ui.stat(stats, { label: lab('v₂ (unit)'), key: 'v2' });
      const tAng = ui.stat(stats, { label: lab('angle between x and Ax'), key: 'angle' });
      const tStr = ui.stat(stats, { label: lab('|Ax| / |x|'), key: 'stretch' });
      const sumLab = tSum.el.querySelector('.sim-stat-label');
      const prodLab = tProd.el.querySelector('.sim-stat-label');

      const gM = ui.group(sp.side, 'Morph and test vector');
      const sT = ui.slider(gM, { label: 'Morph <i>t</i>: M(t) = (1 − t)I + tA', min: 0, max: 1, step: 0.01, value: st.t, key: 't',
        onInput: function (v) { stopAnim(); st.t = v; cv.redraw(); } });
      // a running morph animation writes t into the slider every frame: stop it as soon as the user takes hold of the
      // t slider or its number box, otherwise the animation overwrites what is being typed
      sT.el.querySelectorAll('input').forEach(function (inp) {
        inp.addEventListener('focus', function () { stopAnim(); });
        inp.addEventListener('pointerdown', function () { stopAnim(); });
      });
      const rowM = ui.row(gM);
      const anim = api.animate(function (dt) {
        st.t = Math.min(1, st.t + dt / 2);
        sT.set(st.t);
        if (st.t >= 1) { st.t = 1; anim.stop(); }
        cv.drawNow();
      });
      ui.button(rowM, { label: 'Play', primary: true, key: 'play', title: 'Animate t from 0 to 1 (about 2 s)',
        onClick: function () { st.t = 0; sT.set(0); anim.start(); cv.drawNow(); } });
      const bSnap1 = ui.button(rowM, { label: 'Snap x to v₁', key: 'snapV1', onClick: function () { snap(0); } });
      const bSnap2 = ui.button(rowM, { label: 'Snap x to v₂', key: 'snapV2', onClick: function () { snap(1); } });

      const gS = ui.group(sp.side, 'Show');
      const rowS = ui.row(gS);
      const cGrid = ui.checkbox(rowS, { label: 'Transformed grid', checked: st.showGrid, key: 'showGrid',
        onChange: function (v) { st.showGrid = v; cv.redraw(); } });
      const cCirc = ui.checkbox(rowS, { label: 'Unit circle', checked: st.showCircle, key: 'showCircle',
        onChange: function (v) { st.showCircle = v; cv.redraw(); } });
      const cEig = ui.checkbox(rowS, { label: 'Eigen-directions', checked: st.showEigen, key: 'showEigen',
        onChange: function (v) { st.showEigen = v; update(); } });
      const rowR = ui.row(gS);
      ui.button(rowR, { label: 'Reset', key: 'reset', title: 'Restore every default',
        onClick: function () { setState(Object.assign({}, DEFAULTS)); } });

      ui.hints(api.root, [
        'Drag x until Ax lies along the same line as x: that direction is an eigenvector, and |Ax|/|x| is |λ|.',
        'Choose <b>Shear</b>: λ = 1 is repeated but there is only ONE eigenvector direction — the matrix is not diagonalizable.',
        'Choose <b>Rotation 90°</b>: every vector is turned, so there is no real eigenvector; λ = ±j.',
        'Choose <b>Singular</b>: the whole plane collapses onto a line (det = 0), and one eigenvalue is 0.',
        'Make b = c (symmetric): the two eigen-directions are always perpendicular.'
      ]);

      // ---- state helpers ----
      let E = null, P = null;            // cached eig / props of A (recomputed only when A changes)
      function recompute() { E = model.eig(st.a, st.b, st.c, st.d); P = model.props(st.a, st.b, st.c, st.d); }
      // The sliders run −3…3, but the Singular preset [[1,2],[2,4]] (and the check state [[4,1],[2,3]]) need 4:
      // a slider's range is widened to whole numbers around a value outside ±3 while it is there, and goes back
      // to −3…3 as soon as the entry is inside again. Entries are limited to ±ENTRY_MAX.
      const SLIDERS = { a: sA, b: sB, c: sC, d: sD };
      const wide = { a: false, b: false, c: false, d: false };
      function setEntry(k, v) {
        v = Number(v);
        if (!isFinite(v)) return;
        v = M.clamp(v, -ENTRY_MAX, ENTRY_MAX);
        const sl = SLIDERS[k];
        if (v < -3 || v > 3) { sl.setRange(Math.min(-3, Math.floor(v)), Math.max(3, Math.ceil(v))); wide[k] = true; }
        else if (wide[k]) { sl.setRange(-3, 3); wide[k] = false; }
        sl.set(v); st[k] = sl.get();
      }
      function setMatrix(m) { setEntry('a', m[0]); setEntry('b', m[1]); setEntry('c', m[2]); setEntry('d', m[3]); }
      function matrixChanged() { recompute(); update(); }
      function stopAnim() { if (anim.running) anim.stop(); }
      function clampX(v) { return M.clamp(Number(v), -XLIM, XLIM); }

      function snap(i) {
        const v = E.vectors[i];
        if (!v) return;
        const l = Math.abs(E.values[i].re);
        let L = Math.hypot(st.vx, st.vy);
        if (!(L >= 0.5)) L = 1;
        L = Math.min(L, 3);
        if (l > 1e-9) L = Math.min(L, 3 / l);   // keep Ax = λx inside the view
        st.vx = L * v[0]; st.vy = L * v[1];
        update();
      }

      function vecTxt(v) { return '(' + fmtFixed(v[0], 3) + ', ' + fmtFixed(v[1], 3) + ')'; }
      function lamTxt(z) { return Math.abs(z.im) > 0 ? RASim.fmtComplex(z, 4) : fmt(z.re, 4); }
      function normA() { return Math.hypot(st.a, st.b, st.c, st.d); }

      // eigenvector detection for the current x (shared by the read-outs and the canvas)
      function detect() {
        const nx = Math.hypot(st.vx, st.vy);
        const Ax = model.apply(st.a, st.b, st.c, st.d, st.vx, st.vy);
        const nAx = Math.hypot(Ax[0], Ax[1]);
        const scale = Math.max(1, normA());
        const out = { nx: nx, Ax: Ax, nAx: nAx, xZero: nx < 1e-9, AxZero: nAx <= 1e-9 * scale * nx, hit: null };
        if (out.xZero) return out;
        if (E.type === 'repeated-scalar') { out.hit = { i: 0, lambda: E.values[0].re, off: 0 }; return out; }
        for (let i = 0; i < 2; i++) {
          const v = E.vectors[i];
          if (!v) continue;
          const ang = model.angleBetween(st.vx, st.vy, v[0], v[1]);
          const off = Math.min(ang, Math.PI - ang);
          if (off <= EIG_TOL && (!out.hit || off < out.hit.off)) out.hit = { i: i, lambda: E.values[i].re, off: off };
        }
        return out;
      }

      // ---- read-outs ----
      function update() {
        if (!E) recompute();
        const e = E, pr = P;
        mat.set([[fmt(st.a, 4), fmt(st.b, 4)], [fmt(st.c, 4), fmt(st.d, 4)]], { colColors: [C.red, C.green] });
        const cplx = e.type === 'complex';
        const l1 = e.values[0], l2 = e.values[1];
        tTr.set(fmt(e.trace, 4));
        tDet.set(fmt(e.det, 4));
        sumLab.innerHTML = cplx ? lab('λ₁ + λ₂ = 2·Re λ') : lab('λ₁ + λ₂');
        prodLab.innerHTML = cplx ? lab('λ₁·λ₂ = |λ|²') : lab('λ₁·λ₂');
        tSum.set(fmt(l1.re + l2.re, 4));
        tProd.set(fmt(cplx ? l1.re * l1.re + l1.im * l1.im : l1.re * l2.re, 4));
        tChar.set(model.charEq(st.a, st.b, st.c, st.d));
        tL1.set(lamTxt(l1));
        tL2.set(lamTxt(l2));
        if (e.type === 'repeated-scalar') { tV1.set('any direction'); tV2.set('any direction'); }
        else {
          tV1.set(e.vectors[0] ? vecTxt(e.vectors[0]) : '—');
          tV2.set(e.vectors[1] ? vecTxt(e.vectors[1]) : '—');
        }

        if (e.type === 'real-distinct') tag.set('REAL, DISTINCT', 'ok');
        else if (e.type === 'repeated-defective') tag.set('REPEATED — DEFECTIVE (1 EIGENVECTOR)', 'warn');
        else if (e.type === 'repeated-scalar') tag.set('REPEATED — EVERY VECTOR', 'ok');
        else tag.set('COMPLEX — NO REAL EIGENVECTOR', 'warn');

        const items = [];
        if (pr.symmetric) items.push('symmetric (b = c) ⇒ real λ, perpendicular eigenvectors');
        if (pr.singular) items.push('det = 0 ⇒ λ = 0 is an eigenvalue, rank ' + pr.rank);
        if (pr.orthogonal) items.push('orthogonal (AᵀA = I) ⇒ |λ| = 1');
        if (!pr.diagonalizable) items.push('not diagonalizable');
        if (pr.flips) items.push('flips orientation (det &lt; 0)');
        propsNote.innerHTML = items.length ? '<b>Properties:</b> ' + items.join('; ') + '.'
          : '<b>Properties:</b> none of the special cases (not symmetric, not singular, not orthogonal; diagonalizable, keeps orientation).';

        const dt = detect();
        let angTxt, angState = '', msg, yes = false;
        if (dt.xZero) {
          angTxt = '—'; tStr.set('—');
          msg = 'x = 0: an eigenvector must be non-zero. Drag x away from the origin.';
        } else {
          tStr.set(fmt(dt.nAx / dt.nx, 4));
          const lamZero = dt.hit && Math.abs(dt.hit.lambda) <= 1e-9 * Math.max(1, normA());
          if (dt.AxZero || lamZero) angTxt = '—';
          else angTxt = fmtFixed(M.deg(model.angleBetween(st.vx, st.vy, dt.Ax[0], dt.Ax[1])), 1) + '°';
          if (dt.hit) {
            yes = true; angState = 'ok';
            const L = dt.hit.lambda;
            msg = 'x is an eigenvector: Ax = λx with λ = ' + fmt(L, 4);
            if (lamZero) msg += dt.AxZero ? ' (Ax = 0: A sends x to the origin)' : ' (Ax ≈ 0: A sends this line to the origin)';
            else if (L < 0) msg += ' (λ &lt; 0: Ax points opposite to x)';
            if (e.type === 'repeated-scalar') msg += ' — for A = λI every vector is one';
            msg += '.';
          } else if (cplx) {
            msg = 'No real eigenvector: A turns every vector (λ is complex), so Ax never lies along x.';
          } else {
            msg = 'x is not an eigenvector: A turns it by ' + angTxt + '. ' +
              (st.showEigen ? 'Drag x onto a dashed eigen-direction.' : 'Drag x until Ax lies along x.');
          }
        }
        tAng.set(angTxt); tAng.setState(angState);
        eigMsg.innerHTML = msg;
        eigMsg.classList.toggle('eg-yes', yes);

        bSnap1.disabled = !e.vectors[0];
        bSnap2.disabled = !e.vectors[1];
        const match = PRESETS.filter(function (p) {
          return Math.abs(p.m[0] - st.a) < 1e-9 && Math.abs(p.m[1] - st.b) < 1e-9 && Math.abs(p.m[2] - st.c) < 1e-9 && Math.abs(p.m[3] - st.d) < 1e-9;
        })[0];
        selPreset.set(match ? match.value : 'custom');
        cv.redraw();
      }

      // ---- label placement (obstacles: thick segments and already placed boxes) ----
      function segHitsRect(x1, y1, x2, y2, l, t, r, b) {   // Liang–Barsky
        const dx = x2 - x1, dy = y2 - y1;
        const Pp = [-dx, dx, -dy, dy], Q = [x1 - l, r - x1, y1 - t, b - y1];
        let u0 = 0, u1 = 1;
        for (let i = 0; i < 4; i++) {
          if (Pp[i] === 0) { if (Q[i] < 0) return false; }
          else {
            const u = Q[i] / Pp[i];
            if (Pp[i] < 0) { if (u > u1) return false; if (u > u0) u0 = u; }
            else { if (u < u0) return false; if (u < u1) u1 = u; }
          }
        }
        return true;
      }
      function segsCross(ax, ay, bx, by, cx, cy, dx, dy) {   // proper intersection of two segments
        function o(px, py, qx, qy, rx, ry) { return (qx - px) * (ry - py) - (qy - py) * (rx - px); }
        const d1 = o(cx, cy, dx, dy, ax, ay), d2 = o(cx, cy, dx, dy, bx, by);
        const d3 = o(ax, ay, bx, by, cx, cy), d4 = o(ax, ay, bx, by, dx, dy);
        return d1 * d2 < 0 && d3 * d4 < 0;
      }
      function makeLayout(ctx, frame) {
        const segs = [], boxes = [];
        const L = {
          seg: function (x1, y1, x2, y2, hw, wt) { segs.push([x1, y1, x2, y2, hw, wt == null ? 1 : wt]); },
          dot: function (x, y, rr) { segs.push([x, y, x, y, rr, 1]); },
          hits: function (bx) {
            let n = 0;
            segs.forEach(function (s) { if (segHitsRect(s[0], s[1], s[2], s[3], bx.l - s[4], bx.t - s[4], bx.r + s[4], bx.b + s[4])) n += s[5]; });
            // a label on top of another label hides it: far worse than crossing a line, even when several arrows
            // lie on the same line (singular A, where x, Ax, A e₂ and an eigen-line coincide)
            boxes.forEach(function (o) { if (bx.l < o.r + 2 && bx.r > o.l - 2 && bx.t < o.b + 2 && bx.b > o.t - 2) n += 20; });
            return n;
          },
          size: function (text, font) { ctx.font = font; return { a: ctx.measureText(text).width / 2 + 3, b: 8 }; },
          place: function (text, font, color, cands) {
            const s = L.size(text, font);
            let best = null;
            cands.forEach(function (c) {
              const bx = { l: c.x - s.a, r: c.x + s.a, t: c.y - s.b, b: c.y + s.b };
              let cost = c.cost + 1000 * L.hits(bx);
              if (c.anchor) {   // a leader from the anchor to the box must not run through placed labels
                const nx = M.clamp(c.anchor[0], bx.l, bx.r), ny = M.clamp(c.anchor[1], bx.t, bx.b);
                boxes.forEach(function (o) { if (segHitsRect(c.anchor[0], c.anchor[1], nx, ny, o.l - 3, o.t - 3, o.r + 3, o.b + 3)) cost += 600; });
                segs.forEach(function (q) { if (segsCross(c.anchor[0], c.anchor[1], nx, ny, q[0], q[1], q[2], q[3])) cost += 150 * q[5]; });
              }
              if (bx.l < frame.x + 1 || bx.r > frame.x + frame.w - 1 || bx.t < frame.y + 1 || bx.b > frame.y + frame.h - 1) cost += 100000;
              if (!best || cost < best.cost) best = { cost: cost, c: c, bx: bx };
            });
            boxes.push(best.bx);
            return { x: best.c.x, y: best.c.y, text: text, font: font, color: color, box: best.bx, cand: best.c };
          },
          // label centres around (cx, cy) in directions mid ± i·step (radians, y up), just clear of radius r
          around: function (cx, cy, r, mid, step, n, text, font, rings) {
            const s = L.size(text, font);
            const out = [];
            for (let i = 0; i <= n; i++) {
              [i, -i].forEach(function (j, k) {
                if (i === 0 && k) return;
                const u = mid + j * step, ux = Math.cos(u), uy = Math.sin(u);
                (rings || [0, 10, 24]).forEach(function (extra, ring) {
                  const d = r + 3 + s.a * Math.abs(ux) + s.b * Math.abs(uy) + extra;
                  out.push({ x: cx + d * ux, y: cy - d * uy, cost: 6 * i + 9 * ring });
                });
              });
            }
            return out;
          }
        };
        return L;
      }

      // where the ray from (x0, y0) towards (x1, y1) leaves the rectangle shrunk by `inset` (null if (x1, y1) is inside)
      function exitPoint(x0, y0, x1, y1, r, inset) {
        const l = r.x + inset, t = r.y + inset, rr = r.x + r.w - inset, b = r.y + r.h - inset;
        if (x1 >= l && x1 <= rr && y1 >= t && y1 <= b) return null;
        const dx = x1 - x0, dy = y1 - y0;
        let s = 1;
        if (dx > 0) s = Math.min(s, (rr - x0) / dx); else if (dx < 0) s = Math.min(s, (l - x0) / dx);
        if (dy > 0) s = Math.min(s, (b - y0) / dy); else if (dy < 0) s = Math.min(s, (t - y0) / dy);
        s = Math.max(0, s);
        return [x0 + s * dx, y0 + s * dy];
      }

      // ---- canvas ----
      function makePlot(ctx, w, h) {
        return RASim.plot(ctx, { box: { x: 0, y: 0, w: w, h: h }, xmin: -XLIM, xmax: XLIM, ymin: -1, ymax: 1, equal: true,
          margin: { left: 24, right: 8, top: 8, bottom: 20 } });
      }
      function intTicks(a, b) { const out = []; for (let k = Math.ceil(a - 1e-9); k <= b + 1e-9; k++) out.push(k); return out; }

      cv.draw = function (ctx, w, h) {
        if (!E) recompute();
        const p = makePlot(ctx, w, h);
        const tf = function (v) { return fmt(v, 3); };
        p.axes({ xTicks: intTicks(p.xmin, p.xmax), yTicks: intTicks(p.ymin, p.ymax), xFormat: tf, yFormat: tf });
        const r = p.rect;
        const narrow = r.w < 380;
        const Mt = model.morph(st.a, st.b, st.c, st.d, st.t);
        const detM = Mt[0] * Mt[3] - Mt[1] * Mt[2];
        const ox = p.X(0), oy = p.Y(0);
        const F12 = '12px sans-serif', B13 = 'bold 13px sans-serif';
        const LBG = RASim.alpha('#ffffff', 0.88);
        const lay = makeLayout(ctx, r);
        function P(v) { return [p.X(v[0]), p.Y(v[1])]; }
        function Mv(x, y) { return model.apply(Mt[0], Mt[1], Mt[2], Mt[3], x, y); }

        ctx.save();
        ctx.beginPath(); ctx.rect(r.x, r.y, r.w, r.h); ctx.clip();

        // 1. image of the integer grid lines −4…4 under M(t)
        if (st.showGrid) {
          ctx.lineWidth = 1; ctx.strokeStyle = C.lightBlue; ctx.setLineDash([]);
          ctx.beginPath();
          for (let k = -4; k <= 4; k++) {
            let q0 = P(Mv(k, -4)), q1 = P(Mv(k, 4));
            ctx.moveTo(q0[0], q0[1]); ctx.lineTo(q1[0], q1[1]);
            q0 = P(Mv(-4, k)); q1 = P(Mv(4, k));
            ctx.moveTo(q0[0], q0[1]); ctx.lineTo(q1[0], q1[1]);
          }
          ctx.stroke();
        }

        // 2. image of the unit square
        const e1 = Mv(1, 0), e2 = Mv(0, 1);
        const p1 = P(e1), p2 = P(e2), p12 = P([e1[0] + e2[0], e1[1] + e2[1]]);
        const sqPos = detM > 1e-12, sqNeg = detM < -1e-12;
        ctx.beginPath(); ctx.moveTo(ox, oy); ctx.lineTo(p1[0], p1[1]); ctx.lineTo(p12[0], p12[1]); ctx.lineTo(p2[0], p2[1]); ctx.closePath();
        ctx.fillStyle = sqNeg ? RASim.alpha(C.lightRed, 0.6) : RASim.alpha(C.lightGreen, 0.6);
        ctx.fill();
        ctx.strokeStyle = sqNeg ? RASim.alpha(C.red, 0.55) : sqPos ? RASim.alpha(C.green, 0.55) : RASim.alpha(C.muted, 0.7);
        ctx.lineWidth = 1.25; ctx.stroke();

        // 3. unit circle and its image
        if (st.showCircle) {
          p.circle(0, 0, 1, { color: C.faint, width: 1.25, dash: [4, 4] });
          ctx.beginPath();
          for (let i = 0; i <= 96; i++) {
            const th = 2 * Math.PI * i / 96;
            const q = P(Mv(Math.cos(th), Math.sin(th)));
            if (i) ctx.lineTo(q[0], q[1]); else ctx.moveTo(q[0], q[1]);
          }
          ctx.strokeStyle = C.purple; ctx.lineWidth = 2; ctx.setLineDash([]); ctx.stroke();
        }

        // 4. images of the basis vectors (columns of M(t))
        D.arrow(ctx, ox, oy, p1[0], p1[1], { color: C.red, width: 2.5, head: 10 });
        D.arrow(ctx, ox, oy, p2[0], p2[1], { color: C.green, width: 2.5, head: 10 });
        [p1, p2].forEach(function (q) { const ex = exitPoint(ox, oy, q[0], q[1], r, 2) || q; lay.seg(ox, oy, ex[0], ex[1], 2); });

        // 5. eigen-directions of A (not of M(t))
        const eigLines = [];
        if (st.showEigen && (E.type === 'real-distinct' || E.type === 'repeated-defective')) {
          [0, 1].forEach(function (i) {
            const v = E.vectors[i];
            if (!v) return;
            const col = i === 0 ? C.maroon : C.teal;
            const R = 3 * (r.w + r.h);
            const dx = v[0], dy = -v[1];
            ctx.strokeStyle = col; ctx.lineWidth = 1.75; ctx.setLineDash(i === 0 ? [8, 5] : [3, 4]);
            ctx.beginPath(); ctx.moveTo(ox - R * dx, oy - R * dy); ctx.lineTo(ox + R * dx, oy + R * dy); ctx.stroke();
            ctx.setLineDash([]);
            const a0 = exitPoint(ox, oy, ox + R * dx, oy + R * dy, r, 0), b0 = exitPoint(ox, oy, ox - R * dx, oy - R * dy, r, 0);
            lay.seg(b0[0], b0[1], a0[0], a0[1], 1.5);
            const txt = E.type === 'repeated-defective' ? 'λ₁ = λ₂ = ' + fmt(E.values[0].re, 4)
              : (i === 0 ? 'λ₁ = ' : 'λ₂ = ') + fmt(E.values[i].re, 4);
            eigLines.push({ i: i, v: v, col: col, txt: txt, ends: [a0, b0] });
          });
        }

        // 6. test vector x and Ax (full A); the longer arrow is drawn first so both stay visible
        const dt = detect();
        const px = P([st.vx, st.vy]), pAx = P(dt.Ax);
        const xFirst = dt.nx >= dt.nAx;
        function drawX() { D.arrow(ctx, ox, oy, px[0], px[1], { color: C.blue, width: 3, head: 11 }); }
        function drawAx() { D.arrow(ctx, ox, oy, pAx[0], pAx[1], { color: C.orange, width: 3, head: 11 }); }
        if (xFirst) { drawX(); drawAx(); } else { drawAx(); drawX(); }
        // drag handle: ring around the tip of x
        ctx.setLineDash([]); ctx.lineWidth = 2; ctx.strokeStyle = C.blue;
        ctx.beginPath(); ctx.arc(px[0], px[1], 8, 0, 2 * Math.PI); ctx.stroke();
        const exX = exitPoint(ox, oy, px[0], px[1], r, 2), exAx = exitPoint(ox, oy, pAx[0], pAx[1], r, 2);
        const tipX = exX || px, tipAx = exAx || pAx;
        lay.seg(ox, oy, tipX[0], tipX[1], 2.5); lay.seg(ox, oy, tipAx[0], tipAx[1], 2.5);
        lay.dot(px[0], px[1], 10); lay.dot(ox, oy, 4);

        // 7. labels (most important first)
        const labels = [], leaders = [];
        function leader(lb, ax, ay, gap) {
          const b = lb.box;
          const nx = M.clamp(ax, b.l, b.r), ny = M.clamp(ay, b.t, b.b);
          const d = Math.hypot(nx - ax, ny - ay);
          if (d > gap + 10) leaders.push([ax + (nx - ax) * gap / d, ay + (ny - ay) * gap / d, nx, ny, lb.color]);
        }
        function tipLabel(text, font, color, tip, gap) {
          const dir = Math.atan2(-(tip[1] - oy), tip[0] - ox);
          const lb = lay.place(text, font, color, lay.around(tip[0], tip[1], gap, isFinite(dir) ? dir : Math.PI / 4, Math.PI / 8, 8, text, font, [0, 10, 24, 40]));
          labels.push(lb); leader(lb, tip[0], tip[1], gap);
          return lb;
        }
        if (!dt.xZero) {
          tipLabel(exX ? 'x (off view)' : 'x', B13, C.blue, tipX, exX ? 2 : 10);
          if (!dt.AxZero) tipLabel(exAx ? 'Ax (off view)' : 'Ax', B13, C.orange, tipAx, 3);
          else {
            const lb = lay.place('Ax = 0', B13, C.orange, lay.around(ox, oy, 4, -Math.PI / 4, Math.PI / 8, 8, 'Ax = 0', B13));
            labels.push(lb); leader(lb, ox, oy, 4);
          }
        }
        const atT1 = st.t >= 1 - 1e-12;
        const nm = atT1 ? 'A' : 'M';
        const s1 = narrow ? nm + 'e₁' : nm + ' e₁ = (' + fmt(e1[0], 3) + ', ' + fmt(e1[1], 3) + ')';
        const s2 = narrow ? nm + 'e₂' : nm + ' e₂ = (' + fmt(e2[0], 3) + ', ' + fmt(e2[1], 3) + ')';
        [[s1, C.red, p1], [s2, C.green, p2]].forEach(function (q) {
          if (Math.hypot(q[2][0] - ox, q[2][1] - oy) <= 2) return;
          const ex = exitPoint(ox, oy, q[2][0], q[2][1], r, 2);
          tipLabel(ex ? q[0] + ' (off view)' : q[0], F12, q[1], ex || q[2], ex ? 2 : 3);
        });
        // eigen-line labels after the arrow labels: they have many candidates along their line
        eigLines.forEach(function (el) {
          const s = lay.size(el.txt, F12);
          const cands = [];
          el.ends.forEach(function (end, side) {
            const ex = end[0] - ox, ey = end[1] - oy, len = Math.hypot(ex, ey);
            if (len < 1) return;
            const ux = ex / len, uy = ey / len, nx = -uy, ny = ux;
            const off = s.a * Math.abs(nx) + s.b * Math.abs(ny) + 4;
            const along = s.a * Math.abs(ux) + s.b * Math.abs(uy) + 4;
            [0, 1, 2, 3, 4, 5].forEach(function (j) {
              const dd = Math.max(len * 0.25, len - along - j * 0.12 * len);
              [1, -1].forEach(function (sd, k) {
                cands.push({ x: ox + ux * dd + sd * nx * off, y: oy + uy * dd + sd * ny * off, cost: 4 * j + 2 * side + k });
              });
            });
          });
          if (!cands.length) return;
          labels.push(lay.place(el.txt, F12, el.col, cands));
        });
        const sArea = 'area = |det| = ' + fmt(Math.abs(detM), 4);   // 4 significant digits, as the det read-out
        // candidates: points inside the parallelogram (no leader needed), then beyond its far corner M(1, 1)
        // and around its centre (with a short leader to a point inside)
        const cx = (ox + p12[0]) / 2, cy = (oy + p12[1]) / 2;
        const ca = [];
        [0.5, 0.35, 0.65, 0.2, 0.8].forEach(function (u) {
          [0.5, 0.35, 0.65, 0.2, 0.8].forEach(function (v) {
            const qx = ox + u * (p1[0] - ox) + v * (p2[0] - ox), qy = oy + u * (p1[1] - oy) + v * (p2[1] - oy);
            ca.push({ x: qx, y: qy, cost: 0.05 * Math.hypot(qx - cx, qy - cy), inside: true });
          });
        });
        const dirC = Math.atan2(-(p12[1] - oy), p12[0] - ox);
        const cornerIn = [cx + 0.8 * (p12[0] - cx), cy + 0.8 * (p12[1] - cy)];
        lay.around(p12[0], p12[1], 4, isFinite(dirC) ? dirC : Math.PI / 4, Math.PI / 8, 8, sArea, F12)
          .forEach(function (c) { c.cost += 20; c.anchor = cornerIn; ca.push(c); });
        [p1, p2].forEach(function (q) {   // beyond the other two corners, anchored just inside them
          const dq = Math.atan2(-(q[1] - cy), q[0] - cx);
          const qin = [cx + 0.75 * (q[0] - cx), cy + 0.75 * (q[1] - cy)];
          lay.around(q[0], q[1], 4, isFinite(dq) ? dq : 0, Math.PI / 8, 8, sArea, F12, [0, 12, 28, 48])
            .forEach(function (c) { c.cost += 30; c.anchor = qin; ca.push(c); });
        });
        lay.around(cx, cy, 0, -Math.PI / 2, Math.PI / 8, 8, sArea, F12, [0, 12, 28, 48, 72])
          .forEach(function (c) { c.cost += 40; c.anchor = [cx, cy]; ca.push(c); });
        const la = lay.place(sArea, F12, sqNeg ? C.red : C.green, ca);
        labels.push(la);
        if (la.cand && la.cand.anchor) leader(la, la.cand.anchor[0], la.cand.anchor[1], 0);

        ctx.lineWidth = 1; ctx.setLineDash([]);
        leaders.forEach(function (q) {
          ctx.strokeStyle = RASim.alpha(q[4], 0.7);
          ctx.beginPath(); ctx.moveTo(q[0], q[1]); ctx.lineTo(q[2], q[3]); ctx.stroke();
        });
        labels.forEach(function (lb) { D.text(ctx, lb.text, lb.x, lb.y, { align: 'center', color: lb.color, font: lb.font, bg: LBG }); });
        ctx.restore();
      };

      // ---- drag the tip of x ----
      function hitX(x, y) {
        const p = makePlot(cv.ctx, cv.width, cv.height);
        return Math.hypot(p.X(st.vx) - x, p.Y(st.vy) - y) <= 16 ? { p: p } : null;
      }
      cv.onDrag({
        hit: hitX,
        move: function (x, y, token) {
          const p = token.p;
          stopAnim();
          st.vx = Math.round(M.clamp(p.invX(x), p.xmin, p.xmax) * 100) / 100 + 0;
          st.vy = Math.round(M.clamp(p.invY(y), Math.max(p.ymin, -XLIM), Math.min(p.ymax, XLIM)) * 100) / 100 + 0;
          update();
        },
        cursor: function (x, y) { return hitX(x, y) ? 'grab' : null; }
      });

      function getState() {
        return { a: st.a, b: st.b, c: st.c, d: st.d, t: st.t, vx: st.vx, vy: st.vy,
          showGrid: st.showGrid, showCircle: st.showCircle, showEigen: st.showEigen };
      }
      function setState(s) {
        s = s || {};
        stopAnim();
        ['a', 'b', 'c', 'd'].forEach(function (k) { if (s[k] != null) setEntry(k, s[k]); });
        if (s.t != null) { sT.set(Number(s.t)); st.t = sT.get(); }
        if (s.vx != null && isFinite(Number(s.vx))) st.vx = clampX(s.vx);
        if (s.vy != null && isFinite(Number(s.vy))) st.vy = clampX(s.vy);
        if (s.showGrid != null) { st.showGrid = !!s.showGrid; cGrid.set(st.showGrid); }
        if (s.showCircle != null) { st.showCircle = !!s.showCircle; cCirc.set(st.showCircle); }
        if (s.showEigen != null) { st.showEigen = !!s.showEigen; cEig.set(st.showEigen); }
        recompute();
        update();
      }

      recompute();
      update();
      return { getState: getState, setState: setState };
    }
  });
})();
