// two-link-arm — "2-link planar arm: forward and inverse kinematics"
// Sits beside notes/src/03_robotics.html, section "5. Forward Kinematics".
//
// SIGN CONVENTIONS
//   Base (joint 1) at the origin, x to the right, y up, all angles counter-clockwise positive.
//   θ₁: from the +x axis to link 1.  θ₂: RELATIVE angle from the direction of link 1 to link 2.
//   φ = θ₁ + θ₂ (tip orientation, wrapped to (−π, π]).
//   Elbow E = (L₁cos θ₁, L₁sin θ₁);  tip P = E + (L₂cos(θ₁+θ₂), L₂sin(θ₁+θ₂)).
//   IK branches: "Elbow down (θ₂ ≥ 0)" = elbow +1, θ₂ = +acos c₂;  "Elbow up (θ₂ ≤ 0)" = elbow −1, θ₂ = −acos c₂,
//   c₂ = (r² − L₁² − L₂²)/(2L₁L₂), θ₁ = atan2(y, x) − atan2(L₂ sin θ₂, L₁ + L₂ cos θ₂) wrapped to (−π, π].
//   Reachable iff |c₂| ≤ 1 (a 1e-12 round-off guard is applied). Unreachable targets are projected radially onto the
//   nearest workspace circle (origin → +x direction). Solved point at the origin (L₁ = L₂): θ₂ = ±π, θ₁ = t1Current.
//   J = [[−L₁s₁ − L₂s₁₂, −L₂s₁₂], [L₁c₁ + L₂c₁₂, L₂c₁₂]] (tip velocity = J·[θ̇₁, θ̇₂]), det J = L₁L₂ sin θ₂.
//   Singular when |sin θ₂| < 0.05: 'stretched' (cos θ₂ > 0) or 'folded' (cos θ₂ < 0).
//   Model angles are radians; the UI and the state use degrees for θ₁, θ₂; rates in rad/s; lengths in m.
//   Tip-velocity arrow scale: arrow = v × 0.4 s (1 m/s ↔ 0.4 m on the drawing); shortened at the frame edge.
//
// STATE KEYS (getState / setState)
//   L1, L2 (m), theta1, theta2 (degrees), elbow ('down' | 'up'), w1, w2 (rad/s),
//   showWorkspace, showOther, showVelocity (booleans).
//   setState: theta2 without elbow → elbow follows the sign of θ₂ (0 keeps it); elbow alone → acts like clicking
//   the branch control (switches to the other solution, same tip); elbow together with an angle → taken literally.
//
// STAT KEYS:   x, y, phi, r, detJ, speed, vx, vy, other
// TAG KEYS:    status   (REGULAR / SINGULAR — STRETCHED / SINGULAR — FOLDED / OUTSIDE WORKSPACE)
// MATRIX KEYS: J
// DATA-KEY:    formulaX, formulaY (live formula lines), statusNote (shown only while dragging outside)
// CANVAS KEY:  arm
// CONTROL KEYS: L1, L2, theta1, theta2, elbow, w1, w2, showWorkspace, showOther, showVelocity, reset
(function () {
  'use strict';
  const RASim = globalThis.RASim;
  const M = RASim.math;

  const SING_TOL = 0.05;     // |sin θ₂| below this counts as singular
  const REACH_EPS = 1e-12;   // round-off guard on |c₂| ≤ 1

  function workspace(L1, L2) { return { rMin: Math.abs(L1 - L2), rMax: L1 + L2 }; }

  function fk(L1, L2, t1, t2) {
    const ex = L1 * Math.cos(t1), ey = L1 * Math.sin(t1);
    return { x: ex + L2 * Math.cos(t1 + t2), y: ey + L2 * Math.sin(t1 + t2), phi: M.wrapPi(t1 + t2), ex: ex, ey: ey };
  }

  function ik(L1, L2, x, y, elbow, t1Current) {
    const e = elbow < 0 ? -1 : 1;
    const t1c = isFinite(t1Current) ? Number(t1Current) : 0;
    const ws = workspace(L1, L2);
    let r = Math.hypot(x, y);
    let c2 = (r * r - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    const reachable = Math.abs(c2) <= 1 + REACH_EPS;
    let px = x, py = y;
    if (!reachable) {
      const ux = r > 0 ? x / r : 1, uy = r > 0 ? y / r : 0;
      const rc = M.clamp(r, ws.rMin, ws.rMax);
      px = rc * ux; py = rc * uy;
      c2 = rc >= ws.rMax ? 1 : -1;      // the projected point lies exactly on a boundary circle
      r = rc;
    }
    c2 = M.clamp(c2, -1, 1);
    const s2 = e * Math.sqrt(Math.max(0, 1 - c2 * c2));
    const t2 = e * Math.acos(c2) + 0;    // + 0 turns −0 into 0
    let t1;
    if (r <= 1e-12 * (L1 + L2)) t1 = M.wrapPi(t1c);          // solved point at the origin (L₁ = L₂)
    else t1 = M.wrapPi(Math.atan2(py, px) - Math.atan2(L2 * s2, L1 + L2 * c2));
    return { t1: t1, t2: t2, reachable: reachable, x: px + 0, y: py + 0 };
  }

  function other(L1, L2, t1, t2) {
    return { t1: M.wrapPi(t1 + 2 * Math.atan2(L2 * Math.sin(t2), L1 + L2 * Math.cos(t2))), t2: -t2 + 0 };
  }

  function jacobian(L1, L2, t1, t2) {
    const s1 = Math.sin(t1), c1 = Math.cos(t1), s12 = Math.sin(t1 + t2), c12 = Math.cos(t1 + t2);
    return [[-L1 * s1 - L2 * s12, -L2 * s12], [L1 * c1 + L2 * c12, L2 * c12]];
  }

  function detJ(L1, L2, t2) { return L1 * L2 * Math.sin(t2); }

  function tipVelocity(L1, L2, t1, t2, w1, w2) {
    const J = jacobian(L1, L2, t1, t2);
    const vx = J[0][0] * w1 + J[0][1] * w2, vy = J[1][0] * w1 + J[1][1] * w2;
    return { vx: vx, vy: vy, speed: Math.hypot(vx, vy) };
  }

  function singular(t2) {
    if (Math.abs(Math.sin(t2)) >= SING_TOL) return '';
    return Math.cos(t2) > 0 ? 'stretched' : 'folded';
  }

  const model = { fk: fk, ik: ik, other: other, jacobian: jacobian, detJ: detJ, tipVelocity: tipVelocity, workspace: workspace, singular: singular };

  const DEFAULTS = { L1: 1.0, L2: 0.8, theta1: 30, theta2: 60, elbow: 'down', w1: 1.0, w2: 0.5, showWorkspace: true, showOther: true, showVelocity: true };
  const VEL_T = 0.4;   // s: velocity arrow = v·VEL_T (world units)
  const LBL_DOWN = 'Elbow down (θ₂ ≥ 0)', LBL_UP = 'Elbow up (θ₂ ≤ 0)';

  RASim.register('two-link-arm', {
    title: '2-link planar arm: forward and inverse kinematics',
    model: model,
    mount: function (api) {
      const ui = RASim.ui, C = RASim.colors, D = RASim.draw;
      const fmt = RASim.fmt, fmtFixed = RASim.fmtFixed;
      const st = Object.assign({}, DEFAULTS);
      let drag = null;   // { outside: bool, tx, ty } while dragging the tip

      ui.note(api.root, 'Drag the orange-ringed tip or move the sliders, and watch how det J, the two IK solutions and the workspace ring respond.');
      const sp = ui.split(api.root, { ratio: 1.5 });

      // ---- main column: canvas, legend, formulas ----
      const cv = RASim.canvas(sp.main, { aspect: 1, minHeight: 260, maxHeight: 560, key: 'arm' });
      ui.legend(sp.main, [
        { color: C.blue, label: 'link 1 (L₁)' },
        { color: C.maroon, label: 'link 2 (L₂)' },
        { color: C.muted, label: 'other IK solution', dash: true },
        { color: C.teal, label: 'workspace |L₁−L₂| ≤ r ≤ L₁+L₂', dash: true },
        { color: C.orange, label: 'tip velocity (arrow = v × 0.4 s: 1 m/s ↔ 0.4 m)' }
      ]);
      const fX = ui.note(sp.main, '');
      fX.setAttribute('data-key', 'formulaX');
      const fY = ui.note(sp.main, '');
      fY.setAttribute('data-key', 'formulaY');

      // ---- side column: controls, tag, stats ----
      const gArm = ui.group(sp.side, 'Arm');
      const sL1 = ui.slider(gArm, { label: 'L₁', min: 0.5, max: 2, step: 0.05, value: st.L1, unit: 'm', key: 'L1',
        onInput: function (v) { st.L1 = v; update(); } });
      const sL2 = ui.slider(gArm, { label: 'L₂', min: 0.5, max: 2, step: 0.05, value: st.L2, unit: 'm', key: 'L2',
        onInput: function (v) { st.L2 = v; update(); } });
      const sT1 = ui.slider(gArm, { label: 'θ₁', min: -180, max: 180, step: 1, value: st.theta1, unit: '°', key: 'theta1',
        onInput: function (v) { st.theta1 = v; update(); } });
      const sT2 = ui.slider(gArm, { label: 'θ₂', min: -180, max: 180, step: 1, value: st.theta2, unit: '°', key: 'theta2',
        onInput: function (v) { st.theta2 = v; followSign(); update(); } });
      const seg = ui.segmented(gArm, { label: 'IK branch (used when dragging the tip)', key: 'elbow', value: st.elbow,
        options: [{ value: 'down', label: LBL_DOWN }, { value: 'up', label: LBL_UP }],
        onChange: function (v) { st.elbow = v === 'up' ? 'up' : 'down'; switchBranchIfNeeded(); update(); } });

      const gRate = ui.group(sp.side, 'Joint rates → tip velocity');
      const sW1 = ui.slider(gRate, { label: 'θ̇₁', min: -2, max: 2, step: 0.1, value: st.w1, unit: 'rad/s', key: 'w1',
        onInput: function (v) { st.w1 = v; update(); } });
      const sW2 = ui.slider(gRate, { label: 'θ̇₂', min: -2, max: 2, step: 0.1, value: st.w2, unit: 'rad/s', key: 'w2',
        onInput: function (v) { st.w2 = v; update(); } });

      const gShow = ui.group(sp.side, 'Show');
      const rowShow = ui.row(gShow);
      const cWs = ui.checkbox(rowShow, { label: 'Workspace', checked: st.showWorkspace, key: 'showWorkspace',
        onChange: function (v) { st.showWorkspace = v; update(); } });
      const cOther = ui.checkbox(rowShow, { label: 'Other IK solution', checked: st.showOther, key: 'showOther',
        onChange: function (v) { st.showOther = v; update(); } });
      const cVel = ui.checkbox(rowShow, { label: 'Tip velocity', checked: st.showVelocity, key: 'showVelocity',
        onChange: function (v) { st.showVelocity = v; update(); } });
      const rowBtn = ui.row(gShow);
      ui.button(rowBtn, { label: 'Reset', key: 'reset', title: 'Restore every default',
        onClick: function () { drag = null; setState(Object.assign({}, DEFAULTS)); } });

      const rowTag = ui.row(sp.side);
      const tag = ui.tag(rowTag, { text: '', key: 'status' });
      const statusNote = ui.note(rowTag, 'no IK solution: |cos θ₂| &gt; 1');
      statusNote.setAttribute('data-key', 'statusNote');
      statusNote.style.display = 'none';

      // stat labels: the core upper-cases them, which would turn φ, θ into Φ, Θ — keep them as written
      RASim.css('two-link-arm', '.sim[data-sim="two-link-arm"] .tla-nt { text-transform: none; letter-spacing: 0; font-size: 11.5px; }');
      function lab(html) { return '<span class="tla-nt">' + html + '</span>'; }
      const stats = ui.stats(sp.side);
      const tX = ui.stat(stats, { label: lab('x (m)'), key: 'x' });
      const tY = ui.stat(stats, { label: lab('y (m)'), key: 'y' });
      const tPhi = ui.stat(stats, { label: lab('φ = θ₁ + θ₂'), key: 'phi' });
      const tR = ui.stat(stats, { label: lab('r = √(x² + y²) (m)'), key: 'r' });
      const tDet = ui.stat(stats, { label: lab('det J (m²)'), key: 'detJ', title: 'det J = L₁L₂ sin θ₂' });
      const tSpeed = ui.stat(stats, { label: lab('tip speed |v| (m/s)'), key: 'speed' });
      const tVx = ui.stat(stats, { label: lab('v<sub>x</sub> (m/s)'), key: 'vx' });
      const tVy = ui.stat(stats, { label: lab('v<sub>y</sub> (m/s)'), key: 'vy' });
      const tOther = ui.stat(stats, { label: lab('other IK solution (same tip)'), key: 'other' });
      tOther.el.style.gridColumn = '1 / -1';
      const mat = ui.matrix(sp.side, { label: 'J =', key: 'J' });

      ui.hints(api.root, [
        'Set θ₂ = 0°: det J = 0, the arm is stretched and the tip sits on the outer circle. Now change the joint rates — the tip velocity stays perpendicular to the arm: it cannot move radially.',
        'Drag the tip somewhere inside the ring, then switch Elbow down / Elbow up: the same (x, y), two different joint solutions, and φ changes.',
        'Set L₁ = L₂: the hole in the middle disappears (inner radius |L₁ − L₂| = 0).',
        'Drag the tip outside the outer circle: |cos θ₂| &gt; 1, so there is no solution — the arm can only point at the target.'
      ]);

      // ---- helpers ----
      const rad = M.rad, deg = M.deg;
      function fmtDeg(d) { return fmt(Math.round(d * 10) / 10, 6); }          // degrees, at most 1 decimal
      function angTxt(d) { const s = fmtDeg(d) + '°'; return d < 0 && s.charAt(0) === '−' ? '(' + s + ')' : ' ' + s; }
      function followSign() {
        if (st.theta2 > 0) st.elbow = 'down'; else if (st.theta2 < 0) st.elbow = 'up';
        seg.set(st.elbow);
      }
      function setAngles(t1r, t2r) {
        sT1.set(deg(t1r)); st.theta1 = sT1.get();
        sT2.set(deg(t2r)); st.theta2 = sT2.get();
      }
      function switchBranchIfNeeded() {
        if ((st.elbow === 'down' && st.theta2 < 0) || (st.elbow === 'up' && st.theta2 > 0)) {
          const o = model.other(st.L1, st.L2, rad(st.theta1), rad(st.theta2));
          setAngles(o.t1, o.t2);
        }
      }
      function angDist(a, b) { return Math.abs(M.wrapPi(a - b)); }
      // candidate direction (radians) farthest from every direction in `avoid`
      function pickAngle(cands, avoid) {
        let best = cands[0], bestScore = -1;
        cands.forEach(function (c) {
          let s = Infinity;
          avoid.forEach(function (a) { s = Math.min(s, angDist(c, a)); });
          if (s > bestScore + 1e-9) { bestScore = s; best = c; }
        });
        return best;
      }

      // Label of an angle arc at (cx, cy) (pixels) with radius r, from direction a0 sweeping `sweep` (radians,
      // counter-clockwise positive, y up). Tries directions around the bisector and takes the one that needs the
      // smallest distance for the label box to clear the arc and every ray in `rays` (lines leaving the vertex).
      function angleLabel(ctx, cx, cy, r, a0, sweep, text, color, rays) {
        ctx.font = '12px sans-serif';
        const a = ctx.measureText(text).width / 2 + 4, b = 9;
        const mid = a0 + sweep / 2;
        let best = null;
        for (let i = -9; i <= 9; i++) {
          const u = mid + i * Math.PI / 12, ux = Math.cos(u), uy = Math.sin(u);
          let d = r + 5 + a * Math.abs(ux) + b * Math.abs(uy);
          rays.forEach(function (e) {
            if (Math.cos(e) * ux + Math.sin(e) * uy <= 0) return;   // ray points away from the label side
            const nx = -Math.sin(e), ny = Math.cos(e);
            const sn = Math.abs(ux * nx + uy * ny);
            d = Math.max(d, (a * Math.abs(nx) + b * Math.abs(ny) + 3) / Math.max(sn, 1e-3));
          });
          const score = d + 10 * Math.abs(i);
          if (!best || score < best.score) best = { score: score, x: cx + d * ux, y: cy - d * uy };
        }
        D.text(ctx, text, best.x, best.y, { align: 'center', color: color, font: '12px sans-serif', bg: 'rgba(255,255,255,0.85)' });
      }

      // ---- geometry shared by draw and drag ----
      function makePlot(ctx, w, h) {
        const R = 1.15 * (st.L1 + st.L2);
        return RASim.plot(ctx, { box: { x: 0, y: 0, w: w, h: h }, xmin: -R, xmax: R, ymin: -R, ymax: R, equal: true,
          margin: { left: 46, right: 10, top: 10, bottom: 36 }, xlabel: 'x (m)', ylabel: 'y (m)' });
      }
      function ticksOf(a, b) { const out = []; for (let k = Math.ceil(a / 0.5 - 1e-9); k * 0.5 <= b + 1e-9; k++) out.push(k * 0.5); return out; }

      cv.draw = function (ctx, w, h) {
        const p = makePlot(ctx, w, h);
        const k = p.X(1) - p.X(0);                           // px per metre
        const labelStep = 0.5 * k >= 32 ? 0.5 : k >= 32 ? 1 : 2;
        const tickFmt = function (v) { const q = v / labelStep; return Math.abs(q - Math.round(q)) < 1e-9 ? fmt(v, 3) : ''; };
        p.axes({ xTicks: ticksOf(p.xmin, p.xmax), yTicks: ticksOf(p.ymin, p.ymax), xFormat: tickFmt, yFormat: tickFmt });

        const L1 = st.L1, L2 = st.L2, t1 = rad(st.theta1), t2 = rad(st.theta2);
        const f = model.fk(L1, L2, t1, t2);
        const ws = model.workspace(L1, L2);
        const sing = model.singular(t2);
        const oth = model.other(L1, L2, t1, t2);
        const fo = model.fk(L1, L2, oth.t1, oth.t2);
        const v = model.tipVelocity(L1, L2, t1, t2, st.w1, st.w2);
        const showGhost = st.showOther && !sing;
        const bx = p.X(0), by = p.Y(0);
        const r = p.rect;
        const tipAng = Math.atan2(f.y, f.x);
        const velAng = Math.atan2(v.vy, v.vx);
        const LBG = 'rgba(255,255,255,0.85)';
        const F12 = '12px sans-serif';

        ctx.save();
        ctx.beginPath(); ctx.rect(r.x, r.y, r.w, r.h); ctx.clip();

        // workspace annulus
        if (st.showWorkspace) {
          const Ro = ws.rMax * k, Ri = ws.rMin * k;
          ctx.beginPath();
          ctx.arc(bx, by, Ro, 0, 2 * Math.PI, false);
          if (Ri > 0.5) { ctx.moveTo(bx + Ri, by); ctx.arc(bx, by, Ri, 0, 2 * Math.PI, true); }
          ctx.fillStyle = RASim.alpha(C.teal, 0.08);
          ctx.fill();
          ctx.strokeStyle = C.teal; ctx.lineWidth = 1.5; ctx.setLineDash([6, 4]);
          ctx.beginPath(); ctx.arc(bx, by, Ro, 0, 2 * Math.PI); ctx.stroke();
          if (Ri > 0.5) { ctx.beginPath(); ctx.arc(bx, by, Ri, 0, 2 * Math.PI); ctx.stroke(); }
          ctx.setLineDash([]);
          // radius labels, placed away from the arm
          const avoidOut = [tipAng, t1, velAng];
          if (showGhost) avoidOut.push(oth.t1);
          const deg45 = Math.PI / 4;
          const cands = [-1, -3, 1, 3, -2, 2, 0, 4].map(function (i) { return i * deg45 + (i % 2 ? 0.1 * Math.sign(i) : 0); });
          const aOut = pickAngle(cands, avoidOut);
          const sOut = 'L₁+L₂ = ' + fmt(ws.rMax, 3) + ' m';
          ctx.font = F12;
          let tw = ctx.measureText(sOut).width;
          let d = 10 + Math.abs(Math.cos(aOut)) * tw / 2 + Math.abs(Math.sin(aOut)) * 8;
          D.text(ctx, sOut, bx + (Ro - d) * Math.cos(aOut), by - (Ro - d) * Math.sin(aOut), { align: 'center', color: C.teal, font: F12, bg: LBG });
          const avoidIn = [t1, t1 / 2, -Math.PI / 2, 0, Math.PI, aOut];
          if (showGhost) avoidIn.push(oth.t1);
          const aIn = pickAngle(cands, avoidIn);
          const sIn = '|L₁−L₂| = ' + fmt(ws.rMin, 3) + ' m';
          tw = ctx.measureText(sIn).width;
          d = Math.max(Ri, 34) + 6 + Math.abs(Math.cos(aIn)) * tw / 2 + Math.abs(Math.sin(aIn)) * 8;
          if (Ri > 0.5) {
            ctx.strokeStyle = C.teal; ctx.lineWidth = 1;
            ctx.beginPath(); ctx.moveTo(bx + Ri * Math.cos(aIn), by - Ri * Math.sin(aIn));
            ctx.lineTo(bx + (d - Math.abs(Math.cos(aIn)) * tw / 2 - 4) * Math.cos(aIn), by - (d - Math.abs(Math.cos(aIn)) * tw / 2 - 4) * Math.sin(aIn));
            ctx.stroke();
          }
          D.text(ctx, sIn, bx + d * Math.cos(aIn), by - d * Math.sin(aIn), { align: 'center', color: C.teal, font: F12, bg: LBG });
        }

        // other IK solution (ghost)
        if (showGhost) {
          ctx.strokeStyle = C.muted; ctx.lineWidth = 3; ctx.setLineDash([7, 5]); ctx.lineCap = 'butt';
          ctx.beginPath(); ctx.moveTo(bx, by); ctx.lineTo(p.X(fo.ex), p.Y(fo.ey)); ctx.lineTo(p.X(fo.x), p.Y(fo.y)); ctx.stroke();
          ctx.setLineDash([]);
          ctx.fillStyle = '#ffffff'; ctx.lineWidth = 1.5;
          ctx.beginPath(); ctx.arc(p.X(fo.ex), p.Y(fo.ey), 4.5, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
        }

        // target marker while dragging outside the workspace
        const ex = p.X(f.ex), ey = p.Y(f.ey), tx = p.X(f.x), ty = p.Y(f.y);
        if (drag && drag.outside) {
          const gx = p.X(drag.tx), gy = p.Y(drag.ty);
          ctx.strokeStyle = C.red; ctx.lineWidth = 1.5; ctx.setLineDash([4, 4]);
          ctx.beginPath(); ctx.moveTo(tx, ty); ctx.lineTo(gx, gy); ctx.stroke();
          ctx.setLineDash([]); ctx.lineWidth = 2;
          ctx.beginPath(); ctx.moveTo(gx - 6, gy - 6); ctx.lineTo(gx + 6, gy + 6); ctx.moveTo(gx - 6, gy + 6); ctx.lineTo(gx + 6, gy - 6); ctx.stroke();
          D.text(ctx, 'target', gx + 9, gy - 10, { color: C.red, font: F12, bg: LBG });
        }

        // links, joints, ground
        ctx.lineCap = 'round';
        ctx.strokeStyle = C.blue; ctx.lineWidth = 7;
        ctx.beginPath(); ctx.moveTo(bx, by); ctx.lineTo(ex, ey); ctx.stroke();
        ctx.strokeStyle = C.maroon; ctx.lineWidth = 6;
        ctx.beginPath(); ctx.moveTo(ex, ey); ctx.lineTo(tx, ty); ctx.stroke();
        ctx.lineCap = 'butt';
        // ground symbol under the base
        ctx.fillStyle = C.fill; ctx.strokeStyle = C.ink; ctx.lineWidth = 1.5;
        ctx.beginPath(); ctx.moveTo(bx, by); ctx.lineTo(bx - 9, by + 14); ctx.lineTo(bx + 9, by + 14); ctx.closePath(); ctx.fill(); ctx.stroke();
        D.hatch(ctx, bx - 13, by + 14, bx + 13, by + 14, { side: 1, length: 6, spacing: 6, color: C.ink });
        [[bx, by], [ex, ey]].forEach(function (q) {
          ctx.fillStyle = '#ffffff'; ctx.strokeStyle = C.ink; ctx.lineWidth = 2;
          ctx.beginPath(); ctx.arc(q[0], q[1], 5.5, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
        });

        // θ₁ arc from +x to link 1
        const L1px = L1 * k, L2px = L2 * k;
        const r1 = M.clamp(0.42 * L1px, 14, 34);
        ctx.strokeStyle = C.faint; ctx.lineWidth = 1; ctx.setLineDash([3, 3]);
        ctx.beginPath(); ctx.moveTo(bx, by); ctx.lineTo(bx + r1 + 14, by); ctx.stroke(); ctx.setLineDash([]);
        if (Math.abs(st.theta1) > 0.05) D.angleArc(ctx, bx, by, r1, 0, t1, { ccw: t1 >= 0, color: C.blue, width: 1.5 });
        const rays1 = [0, t1, -Math.PI / 2];
        if (showGhost) rays1.push(oth.t1);
        angleLabel(ctx, bx, by, r1, 0, t1, 'θ₁ = ' + fmtDeg(st.theta1) + '°', C.blue, rays1);

        // dashed extension of link 1 and θ₂ arc
        const r2 = M.clamp(0.36 * Math.min(L1px, L2px), 12, 30);
        const extLen = r2 + 16;
        ctx.strokeStyle = C.blue; ctx.globalAlpha = 0.6; ctx.lineWidth = 1.5; ctx.setLineDash([4, 3]);
        ctx.beginPath(); ctx.moveTo(ex, ey); ctx.lineTo(ex + extLen * Math.cos(t1), ey - extLen * Math.sin(t1)); ctx.stroke();
        ctx.setLineDash([]); ctx.globalAlpha = 1;
        if (Math.abs(st.theta2) > 0.05) D.angleArc(ctx, ex, ey, r2, t1, t1 + t2, { ccw: t2 >= 0, color: C.maroon, width: 1.5 });
        angleLabel(ctx, ex, ey, r2, t1, t2, 'θ₂ = ' + fmtDeg(st.theta2) + '°', C.maroon, [t1, t1 + t2, t1 + Math.PI]);

        // tip velocity arrow (drawn before the handle so the handle stays on top)
        let velEnd = null;
        if (st.showVelocity && v.speed > 1e-9) {
          let dx = v.vx * VEL_T * k, dy = -v.vy * VEL_T * k;
          // shorten at the frame edge so the head stays visible
          let s = 1;
          const pad = 4;
          if (tx + dx > r.x + r.w - pad) s = Math.min(s, (r.x + r.w - pad - tx) / dx);
          if (tx + dx < r.x + pad) s = Math.min(s, (r.x + pad - tx) / dx);
          if (ty + dy > r.y + r.h - pad) s = Math.min(s, (r.y + r.h - pad - ty) / dy);
          if (ty + dy < r.y + pad) s = Math.min(s, (r.y + pad - ty) / dy);
          s = Math.max(0, s);
          dx *= s; dy *= s;
          velEnd = [tx + dx, ty + dy];
          D.arrow(ctx, tx, ty, tx + dx, ty + dy, { color: C.orange, width: 3, head: 11 });
          const vl = Math.hypot(dx, dy);
          if (vl > 18) D.text(ctx, 'v', tx + dx + 8 * dx / vl - 5 * dy / vl, ty + dy + 8 * dy / vl + 5 * dx / vl, { align: 'center', color: C.orange, font: 'bold 13px sans-serif', bg: LBG });
        }

        // tip handle
        ctx.fillStyle = '#ffffff'; ctx.strokeStyle = C.orange; ctx.lineWidth = 2.5;
        ctx.beginPath(); ctx.arc(tx, ty, 9, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
        ctx.fillStyle = C.maroon;
        ctx.beginPath(); ctx.arc(tx, ty, 4, 0, 2 * Math.PI); ctx.fill();

        // (x, y) label at the tip, away from link 2, the velocity arrow and the ghost
        const sXY = '(' + fmtFixed(f.x, 3) + ', ' + fmtFixed(f.y, 3) + ')';
        ctx.font = F12;
        const twXY = ctx.measureText(sXY).width;
        const avoidTip = [f.phi + Math.PI];
        if (velEnd) avoidTip.push(Math.atan2(-(velEnd[1] - ty), velEnd[0] - tx));
        if (showGhost) avoidTip.push(fo.phi + Math.PI);
        const candTip = [];
        for (let i = 0; i < 16; i++) candTip.push(i * Math.PI / 8);
        // drop candidates that would put the label outside the frame
        const okTip = candTip.filter(function (a) {
          const c = labelCentre(a);
          return c[0] - twXY / 2 >= r.x + 2 && c[0] + twXY / 2 <= r.x + r.w - 2 && c[1] - 8 >= r.y + 2 && c[1] + 8 <= r.y + r.h - 2;
        });
        function labelCentre(a) {
          const dd = 16 + Math.abs(Math.cos(a)) * twXY / 2 + Math.abs(Math.sin(a)) * 4;
          return [tx + dd * Math.cos(a), ty - dd * Math.sin(a)];
        }
        const aTip = pickAngle(okTip.length ? okTip : candTip, avoidTip);
        const cXY = labelCentre(aTip);
        D.text(ctx, sXY, cXY[0], cXY[1], { align: 'center', color: C.ink, font: 'bold 12px sans-serif', bg: LBG });

        ctx.restore();
      };

      // ---- drag the tip ----
      function tipPx(p) { const f = model.fk(st.L1, st.L2, rad(st.theta1), rad(st.theta2)); return [p.X(f.x), p.Y(f.y)]; }
      cv.onDrag({
        hit: function (x, y) {
          const p = makePlot(cv.ctx, cv.width, cv.height);
          const t = tipPx(p);
          return Math.hypot(t[0] - x, t[1] - y) <= 16 ? { p: p } : null;
        },
        move: function (x, y, token) {
          const p = token.p;
          const gx = p.invX(x), gy = p.invY(y);
          const res = model.ik(st.L1, st.L2, gx, gy, st.elbow === 'up' ? -1 : 1, rad(st.theta1));
          setAngles(res.t1, res.t2);
          drag = { outside: !res.reachable, tx: gx, ty: gy };
          update();
        },
        end: function () { drag = null; update(); },
        cursor: function (x, y) {
          const p = makePlot(cv.ctx, cv.width, cv.height);
          const t = tipPx(p);
          return Math.hypot(t[0] - x, t[1] - y) <= 16 ? 'grab' : null;
        }
      });

      // ---- read-outs ----
      function update() {
        const L1 = st.L1, L2 = st.L2, t1 = rad(st.theta1), t2 = rad(st.theta2);
        const f = model.fk(L1, L2, t1, t2);
        const dj = model.detJ(L1, L2, t2);
        const sing = model.singular(t2);
        const v = model.tipVelocity(L1, L2, t1, t2, st.w1, st.w2);
        const o = model.other(L1, L2, t1, t2);
        const J = model.jacobian(L1, L2, t1, t2);

        tX.set(fmtFixed(f.x, 3));
        tY.set(fmtFixed(f.y, 3));
        tPhi.set(fmtDeg(deg(f.phi)) + '°');
        tR.set(fmtFixed(Math.hypot(f.x, f.y), 3));
        tDet.set(fmtFixed(dj, 3));
        tDet.setState(sing ? 'bad' : '');
        tSpeed.set(fmtFixed(v.speed, 3));
        tVx.set(fmtFixed(v.vx, 3));
        tVy.set(fmtFixed(v.vy, 3));
        tOther.set('θ₁′ = ' + fmtDeg(deg(o.t1)) + '°, θ₂′ = ' + fmtDeg(deg(o.t2)) + '°');
        mat.set(J, { decimals: 3 });

        if (drag && drag.outside) { tag.set('OUTSIDE WORKSPACE', 'warn'); statusNote.style.display = ''; }
        else {
          statusNote.style.display = 'none';
          if (sing === 'stretched') tag.set('SINGULAR — STRETCHED', 'bad');
          else if (sing === 'folded') tag.set('SINGULAR — FOLDED', 'bad');
          else tag.set('REGULAR', 'ok');
        }

        const sum = st.theta1 + st.theta2;
        fX.innerHTML = 'x = L₁cos θ₁ + L₂cos(θ₁+θ₂) = ' + fmt(L1, 3) + '·cos' + angTxt(st.theta1) + ' + ' + fmt(L2, 3) + '·cos' + angTxt(sum) + ' = <b>' + fmtFixed(f.x, 3) + '</b>';
        fY.innerHTML = 'y = L₁sin θ₁ + L₂sin(θ₁+θ₂) = ' + fmt(L1, 3) + '·sin' + angTxt(st.theta1) + ' + ' + fmt(L2, 3) + '·sin' + angTxt(sum) + ' = <b>' + fmtFixed(f.y, 3) + '</b>';
        cv.redraw();
      }

      function getState() {
        return { L1: st.L1, L2: st.L2, theta1: st.theta1, theta2: st.theta2, elbow: st.elbow, w1: st.w1, w2: st.w2,
          showWorkspace: st.showWorkspace, showOther: st.showOther, showVelocity: st.showVelocity };
      }
      function setState(s) {
        s = s || {};
        if (s.L1 != null) { sL1.set(Number(s.L1)); st.L1 = sL1.get(); }
        if (s.L2 != null) { sL2.set(Number(s.L2)); st.L2 = sL2.get(); }
        const hasT = s.theta1 != null || s.theta2 != null;
        if (s.theta1 != null) { sT1.set(Number(s.theta1)); st.theta1 = sT1.get(); }
        if (s.theta2 != null) { sT2.set(Number(s.theta2)); st.theta2 = sT2.get(); }
        if (s.w1 != null) { sW1.set(Number(s.w1)); st.w1 = sW1.get(); }
        if (s.w2 != null) { sW2.set(Number(s.w2)); st.w2 = sW2.get(); }
        if (s.showWorkspace != null) { st.showWorkspace = !!s.showWorkspace; cWs.set(st.showWorkspace); }
        if (s.showOther != null) { st.showOther = !!s.showOther; cOther.set(st.showOther); }
        if (s.showVelocity != null) { st.showVelocity = !!s.showVelocity; cVel.set(st.showVelocity); }
        if (s.elbow != null) {
          st.elbow = s.elbow === 'up' ? 'up' : 'down';
          seg.set(st.elbow);
          if (!hasT) switchBranchIfNeeded();
        } else if (s.theta2 != null) followSign();
        update();
      }

      update();
      return { getState: getState, setState: setState };
    }
  });
})();
