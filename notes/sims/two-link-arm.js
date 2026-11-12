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
//   Tip-velocity arrow scale: arrow = v × 0.4 s (1 m/s ↔ 0.4 m on the drawing); when it would leave the frame it is
//   shortened and labelled "v (shortened)" (it is then not to scale).
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
      const tR = ui.stat(stats, { label: lab('r (m)'), key: 'r' });
      const tDet = ui.stat(stats, { label: lab('det J (m²)'), key: 'detJ', title: 'det J = L₁L₂ sin θ₂' });
      const tSpeed = ui.stat(stats, { label: lab('tip speed (m/s)'), key: 'speed' });
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
      // formula lines: angles to at most 2 decimals and lengths in full, so that substituting the printed
      // numbers reproduces the printed result to within one unit of its last digit (after a drag or a branch
      // switch the angles are not whole degrees; 1 decimal could be off by 3 units in the 3rd decimal)
      function fmtDegF(d) { return fmt(Math.round(d * 100) / 100, 7); }
      function angTxt(d) { const s = fmtDegF(d) + '°'; return s.charAt(0) === '−' ? '(' + s + ')' : ' ' + s; }
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

      // ---- label placement: obstacles are thick segments (circles = zero-length segments) and placed boxes ----
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
      function makeLayout(ctx, frame) {
        const segs = [], boxes = [];
        const L = {
          seg: function (x1, y1, x2, y2, hw) { segs.push([x1, y1, x2, y2, hw]); },
          dot: function (x, y, rr) { segs.push([x, y, x, y, rr]); },
          arc: function (cx, cy, rr, a0, a1, hw) {   // polyline of an arc (radians, y up)
            const n = 10;
            for (let i = 0; i < n; i++) {
              const p = a0 + (a1 - a0) * i / n, q = a0 + (a1 - a0) * (i + 1) / n;
              segs.push([cx + rr * Math.cos(p), cy - rr * Math.sin(p), cx + rr * Math.cos(q), cy - rr * Math.sin(q), hw]);
            }
          },
          hits: function (bx) {
            let n = 0;
            segs.forEach(function (s) { if (segHitsRect(s[0], s[1], s[2], s[3], bx.l - s[4], bx.t - s[4], bx.r + s[4], bx.b + s[4])) n++; });
            boxes.forEach(function (o) { if (bx.l < o.r + 2 && bx.r > o.l - 2 && bx.t < o.b + 2 && bx.b > o.t - 2) n += 2; });
            return n;
          },
          // cands: [{ x, y, cost }] label centres; picks the cheapest, collisions weigh most; returns the box
          place: function (text, font, color, cands) {
            ctx.font = font;
            const a = ctx.measureText(text).width / 2 + 3, b = 8;
            let best = null;
            cands.forEach(function (c) {
              const bx = { l: c.x - a, r: c.x + a, t: c.y - b, b: c.y + b };
              let cost = c.cost + 1000 * L.hits(bx);
              if (bx.l < frame.x + 1 || bx.r > frame.x + frame.w - 1 || bx.t < frame.y + 1 || bx.b > frame.y + frame.h - 1) cost += 5000;
              if (!best || cost < best.cost) best = { cost: cost, c: c, bx: bx };
            });
            boxes.push(best.bx);
            return { x: best.c.x, y: best.c.y, text: text, font: font, color: color, box: best.bx };
          },
          // label centres around (cx, cy) at directions mid + i·step, just clear of radius r
          around: function (cx, cy, r, mid, step, n, text, font) {
            ctx.font = font;
            const a = ctx.measureText(text).width / 2 + 3, b = 8;
            const out = [];
            for (let i = 0; i <= n; i++) {
              [i, -i].forEach(function (j, s) {
                if (i === 0 && s) return;
                const u = mid + j * step, ux = Math.cos(u), uy = Math.sin(u);
                [0, 10, 22, 38].forEach(function (extra, ring) {
                  const d = r + 4 + a * Math.abs(ux) + b * Math.abs(uy) + extra;
                  out.push({ x: cx + d * ux, y: cy - d * uy, cost: 6 * i + 9 * ring });
                });
              });
            }
            return out;
          }
        };
        return L;
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
        const r = p.rect;
        const bx = p.X(0), by = p.Y(0);
        const ex = p.X(f.ex), ey = p.Y(f.ey), tx = p.X(f.x), ty = p.Y(f.y);
        const gex = p.X(fo.ex), gey = p.Y(fo.ey);
        const F12 = '12px sans-serif', B12 = 'bold 12px sans-serif';
        const LBG = 'rgba(255,255,255,0.88)';
        const lay = makeLayout(ctx, r);
        const L1px = L1 * k, L2px = L2 * k;
        const r1 = M.clamp(0.42 * L1px, 14, 34);
        const r2 = M.clamp(0.36 * Math.min(L1px, L2px), 12, 30);
        const extLen = r2 + 16;

        ctx.save();
        ctx.beginPath(); ctx.rect(r.x, r.y, r.w, r.h); ctx.clip();

        // 1. workspace annulus
        const Ro = ws.rMax * k, Ri = ws.rMin * k;
        if (st.showWorkspace) {
          ctx.beginPath();
          ctx.arc(bx, by, Ro, 0, 2 * Math.PI, false);
          if (Ri > 0.5) { ctx.moveTo(bx + Ri, by); ctx.arc(bx, by, Ri, 0, 2 * Math.PI, true); }
          ctx.fillStyle = RASim.alpha(C.teal, 0.08);
          ctx.fill();
          ctx.strokeStyle = C.teal; ctx.lineWidth = 1.5; ctx.setLineDash([6, 4]);
          ctx.beginPath(); ctx.arc(bx, by, Ro, 0, 2 * Math.PI); ctx.stroke();
          if (Ri > 0.5) { ctx.beginPath(); ctx.arc(bx, by, Ri, 0, 2 * Math.PI); ctx.stroke(); }
          ctx.setLineDash([]);
        }

        // 2. other IK solution (ghost)
        if (showGhost) {
          ctx.strokeStyle = C.muted; ctx.lineWidth = 3; ctx.setLineDash([7, 5]); ctx.lineCap = 'butt';
          ctx.beginPath(); ctx.moveTo(bx, by); ctx.lineTo(gex, gey); ctx.lineTo(tx, ty); ctx.stroke();
          ctx.setLineDash([]);
          ctx.fillStyle = '#ffffff'; ctx.lineWidth = 1.5;
          ctx.beginPath(); ctx.arc(gex, gey, 4.5, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
          lay.seg(bx, by, gex, gey, 3); lay.seg(gex, gey, tx, ty, 3);
        }

        // target marker while dragging outside the workspace
        let gx = 0, gy = 0;
        const outside = !!(drag && drag.outside);
        if (outside) {
          gx = p.X(drag.tx); gy = p.Y(drag.ty);
          ctx.strokeStyle = C.red; ctx.lineWidth = 1.5; ctx.setLineDash([4, 4]);
          ctx.beginPath(); ctx.moveTo(tx, ty); ctx.lineTo(gx, gy); ctx.stroke();
          ctx.setLineDash([]); ctx.lineWidth = 2;
          ctx.beginPath(); ctx.moveTo(gx - 6, gy - 6); ctx.lineTo(gx + 6, gy + 6); ctx.moveTo(gx - 6, gy + 6); ctx.lineTo(gx + 6, gy - 6); ctx.stroke();
          lay.seg(tx, ty, gx, gy, 2); lay.dot(gx, gy, 8);
        }

        // 3. links, ground symbol, joints
        ctx.lineCap = 'round';
        ctx.strokeStyle = C.blue; ctx.lineWidth = 7;
        ctx.beginPath(); ctx.moveTo(bx, by); ctx.lineTo(ex, ey); ctx.stroke();
        ctx.strokeStyle = C.maroon; ctx.lineWidth = 6;
        ctx.beginPath(); ctx.moveTo(ex, ey); ctx.lineTo(tx, ty); ctx.stroke();
        ctx.lineCap = 'butt';
        lay.seg(bx, by, ex, ey, 5); lay.seg(ex, ey, tx, ty, 5);
        ctx.fillStyle = C.fill; ctx.strokeStyle = C.ink; ctx.lineWidth = 1.5;
        ctx.beginPath(); ctx.moveTo(bx, by); ctx.lineTo(bx - 9, by + 14); ctx.lineTo(bx + 9, by + 14); ctx.closePath(); ctx.fill(); ctx.stroke();
        D.hatch(ctx, bx - 13, by + 14, bx + 13, by + 14, { side: 1, length: 6, spacing: 6, color: C.ink });
        lay.seg(bx - 13, by + 17, bx + 13, by + 17, 5); lay.seg(bx, by, bx, by + 12, 8);
        [[bx, by], [ex, ey]].forEach(function (q) {
          ctx.fillStyle = '#ffffff'; ctx.strokeStyle = C.ink; ctx.lineWidth = 2;
          ctx.beginPath(); ctx.arc(q[0], q[1], 5.5, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
          lay.dot(q[0], q[1], 7);
        });

        // 4. θ₁ arc from +x to link 1 (reference: dashed +x ray)
        ctx.strokeStyle = C.faint; ctx.lineWidth = 1; ctx.setLineDash([3, 3]);
        ctx.beginPath(); ctx.moveTo(bx, by); ctx.lineTo(bx + r1 + 14, by); ctx.stroke(); ctx.setLineDash([]);
        if (Math.abs(st.theta1) > 0.05) {
          D.angleArc(ctx, bx, by, r1, 0, t1, { ccw: t1 >= 0, color: C.blue, width: 1.5 });
          lay.arc(bx, by, r1, 0, t1, 2);
        }
        lay.seg(bx, by, bx + r1 + 14, by, 1);

        // 5. dashed extension of link 1 and θ₂ arc from it to link 2
        const xe = ex + extLen * Math.cos(t1), ye = ey - extLen * Math.sin(t1);
        ctx.strokeStyle = C.blue; ctx.globalAlpha = 0.6; ctx.lineWidth = 1.5; ctx.setLineDash([4, 3]);
        ctx.beginPath(); ctx.moveTo(ex, ey); ctx.lineTo(xe, ye); ctx.stroke();
        ctx.setLineDash([]); ctx.globalAlpha = 1;
        lay.seg(ex, ey, xe, ye, 2);
        if (Math.abs(st.theta2) > 0.05) {
          D.angleArc(ctx, ex, ey, r2, t1, t1 + t2, { ccw: t2 >= 0, color: C.maroon, width: 1.5 });
          lay.arc(ex, ey, r2, t1, t1 + t2, 2);
        }

        // 6. tip velocity arrow, shortened at the frame edge so the head stays visible
        let vEnd = null, vShort = false;
        if (st.showVelocity && v.speed > 1e-9) {
          let dx = v.vx * VEL_T * k, dy = -v.vy * VEL_T * k;
          let s = 1;
          const pad = 4;
          if (tx + dx > r.x + r.w - pad) s = Math.min(s, (r.x + r.w - pad - tx) / dx);
          if (tx + dx < r.x + pad) s = Math.min(s, (r.x + pad - tx) / dx);
          if (ty + dy > r.y + r.h - pad) s = Math.min(s, (r.y + r.h - pad - ty) / dy);
          if (ty + dy < r.y + pad) s = Math.min(s, (r.y + pad - ty) / dy);
          s = Math.max(0, s);
          vShort = s < 0.98;              // drawn shorter than the stated scale: say so on the arrow
          dx *= s; dy *= s;
          vEnd = [tx + dx, ty + dy];
          D.arrow(ctx, tx, ty, vEnd[0], vEnd[1], { color: C.orange, width: 3, head: 11 });
          lay.seg(tx, ty, vEnd[0], vEnd[1], 4);
        }

        // 7. tip handle
        ctx.fillStyle = '#ffffff'; ctx.strokeStyle = C.orange; ctx.lineWidth = 2.5;
        ctx.beginPath(); ctx.arc(tx, ty, 9, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
        ctx.fillStyle = C.maroon;
        ctx.beginPath(); ctx.arc(tx, ty, 4, 0, 2 * Math.PI); ctx.fill();
        lay.dot(tx, ty, 11);

        // 8. labels, placed so that they avoid the drawing and each other (most important first)
        const labels = [], leaders = [];
        // a label that ended up away from what it names gets a thin leader line to it
        function leader(lb, ax, ay, gap) {
          const b = lb.box;
          const nx = M.clamp(ax, b.l, b.r), ny = M.clamp(ay, b.t, b.b);
          const d = Math.hypot(nx - ax, ny - ay);
          if (d > gap + 10) leaders.push([ax + (nx - ax) * gap / d, ay + (ny - ay) * gap / d, nx, ny, lb.color]);
        }
        const sXY = '(' + fmtFixed(f.x, 3) + ', ' + fmtFixed(f.y, 3) + ')';
        const lXY = lay.place(sXY, B12, C.ink, lay.around(tx, ty, 9, f.phi, Math.PI / 8, 8, sXY, B12));
        labels.push(lXY); leader(lXY, tx, ty, 11);
        if (vEnd) {
          const va = Math.atan2(-(vEnd[1] - ty), vEnd[0] - tx);
          const sv = vShort ? 'v (shortened)' : 'v', fv = vShort ? 'bold 12px sans-serif' : 'bold 13px sans-serif';
          const lv = lay.place(sv, fv, C.orange, lay.around(vEnd[0], vEnd[1], 2, va, Math.PI / 6, 6, sv, fv));
          labels.push(lv); leader(lv, vEnd[0], vEnd[1], 3);
        }
        const s2 = 'θ₂ = ' + fmtDeg(st.theta2) + '°';
        const l2 = lay.place(s2, F12, C.maroon, lay.around(ex, ey, r2, t1 + t2 / 2, Math.PI / 12, 12, s2, F12));
        labels.push(l2);
        if (Math.abs(st.theta2) > 0.05) leader(l2, ex + r2 * Math.cos(t1 + t2 / 2), ey - r2 * Math.sin(t1 + t2 / 2), 0);
        else leader(l2, ex, ey, 7);
        const s1 = 'θ₁ = ' + fmtDeg(st.theta1) + '°';
        const l1 = lay.place(s1, F12, C.blue, lay.around(bx, by, r1, t1 / 2, Math.PI / 12, 12, s1, F12));
        labels.push(l1);
        if (Math.abs(st.theta1) > 0.05) leader(l1, bx + r1 * Math.cos(t1 / 2), by - r1 * Math.sin(t1 / 2), 0);
        if (outside) labels.push(lay.place('target', F12, C.red, lay.around(gx, gy, 8, Math.PI / 4, Math.PI / 4, 4, 'target', F12)));
        if (st.showWorkspace) {
          const sOut = 'L₁+L₂ = ' + fmt(ws.rMax, 3) + ' m';
          ctx.font = F12;
          let a = ctx.measureText(sOut).width / 2 + 3;
          const cOut = [];
          for (let i = 0; i < 24; i++) {
            const u = -Math.PI / 2 + i * Math.PI / 12, ux = Math.cos(u), uy = Math.sin(u);
            const d = Ro - 6 - a * Math.abs(ux) - 8 * Math.abs(uy);
            cOut.push({ x: bx + d * ux, y: by - d * uy, cost: 4 * angDist(u, -Math.PI / 3) * 12 / Math.PI });
          }
          labels.push(lay.place(sOut, F12, C.teal, cOut));
          if (Ri > 0.5) {
            const sIn = '|L₁−L₂| = ' + fmt(ws.rMin, 3) + ' m';
            a = ctx.measureText(sIn).width / 2 + 3;
            const cIn = [];
            for (let i = 0; i < 24; i++) {
              const u = -Math.PI / 2 + i * Math.PI / 12, ux = Math.cos(u), uy = Math.sin(u);
              [0, 12, 26].forEach(function (extra, ring) {
                const d = Ri + 6 + a * Math.abs(ux) + 8 * Math.abs(uy) + extra;
                cIn.push({ x: bx + d * ux, y: by - d * uy, cost: 4 * angDist(u, -2 * Math.PI / 3) * 12 / Math.PI + 10 * ring });
              });
            }
            const li = lay.place(sIn, F12, C.teal, cIn);
            // leader from the inner circle to the label
            const ux = li.x - bx, uy = li.y - by, ul = Math.hypot(ux, uy);
            if (ul > Ri + 4) {
              ctx.strokeStyle = C.teal; ctx.lineWidth = 1;
              ctx.beginPath(); ctx.moveTo(bx + Ri * ux / ul, by + Ri * uy / ul); ctx.lineTo(li.x, li.y); ctx.stroke();
            }
            labels.push(li);
          }
        }
        ctx.lineWidth = 1; ctx.setLineDash([]);
        leaders.forEach(function (q) {
          ctx.strokeStyle = RASim.alpha(q[4], 0.7);
          ctx.beginPath(); ctx.moveTo(q[0], q[1]); ctx.lineTo(q[2], q[3]); ctx.stroke();
        });
        labels.forEach(function (lb) { D.text(ctx, lb.text, lb.x, lb.y, { align: 'center', color: lb.color, font: lb.font, bg: LBG }); });

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
        const sL1 = fmt(L1, 6), sL2 = fmt(L2, 6);
        fX.innerHTML = 'x = L₁cos θ₁ + L₂cos(θ₁+θ₂) = ' + sL1 + '·cos' + angTxt(st.theta1) + ' + ' + sL2 + '·cos' + angTxt(sum) + ' = <b>' + fmtFixed(f.x, 3) + '</b>';
        fY.innerHTML = 'y = L₁sin θ₁ + L₂sin(θ₁+θ₂) = ' + sL1 + '·sin' + angTxt(st.theta1) + ' + ' + sL2 + '·sin' + angTxt(sum) + ' = <b>' + fmtFixed(f.y, 3) + '</b>';
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
