// mohr-circle — "Mohr's circle for plane stress"
// Sits beside notes/src/05_mechanical.html, section "Mohr's circle — plane stress" (B2.1 Mechanics of Materials).
//
// SIGN CONVENTIONS
//   Stresses in MPa. Normal stress: tension positive. τ_xy > 0 acts in +y on the +x face (and in +x on the +y face).
//   θ: counter-clockwise positive, from the x-axis to the x′-axis (the normal of the rotated face). Model angles are
//   radians; the UI and the state use degrees.
//   Transformation (as in the notes):
//     σ_x′ = (σ_x + σ_y)/2 + [(σ_x − σ_y)/2]·cos 2θ + τ_xy·sin 2θ
//     σ_y′ = (σ_x + σ_y)/2 − [(σ_x − σ_y)/2]·cos 2θ − τ_xy·sin 2θ
//     τ_x′y′ = −[(σ_x − σ_y)/2]·sin 2θ + τ_xy·cos 2θ
//   Mohr's circle: σ to the right, τ POSITIVE DOWNWARD (the canvas plots −τ upwards and relabels the ticks).
//   X′ = (σ_x′, τ_x′y′), Y′ = (σ_y′, −τ_x′y′); a counter-clockwise θ on the element moves X′ counter-clockwise
//   on screen by 2θ.
//   σ_avg = (σ_x + σ_y)/2, R = √{[(σ_x − σ_y)/2]² + τ_xy²}, σ₁ = σ_avg + R ≥ σ₂ = σ_avg − R.
//   θ_p = ½·atan2(τ_xy, (σ_x − σ_y)/2) ∈ (−90°, 90°]: from x to the σ₁ direction (σ₂ at θ_p + 90°); R < 1e-9 → θ_p = 0.
//   θ_s = θ_p − 45° (model value, in (−135°, 45°]): τ_x′y′ = +R and σ_x′ = σ_y′ = σ_avg there. The UI shows and uses
//   the equivalent plane in the slider range (−90°, 90°] (θ_s + 180° when θ_s ≤ −90°; the transformation has period 180°).
//   Absolute max shear (σ₃ = 0): τ_abs = max(|σ₁ − σ₂|, |σ₁|, |σ₂|)/2. von Mises √(σ₁² − σ₁σ₂ + σ₂²); Tresca = 2τ_abs.
//   Element drawing: normal arrows point away from the face for tension, into it for compression; for τ_x′y′ > 0 the
//   shear arrows point to +y′ on the +x′ face, −y′ on −x′, +x′ on +y′, −x′ on −y′. Arrows below 0.05 MPa are omitted.
//   Arrow length ∝ |value| / max(|σ₁|, |σ₂|) (the scale does not change with θ), with a visible minimum.
//
// STATE KEYS (getState / setState): sx, sy, txy (MPa), theta (degrees; setState wraps values outside [−90, 90]
//   into (−90, 90]), show3D (boolean)
// STAT KEYS:   sx1, sy1, txy1, avg, R, s1, s2, thetaP, tauMax, thetaS, tauAbs, vonMises, tresca, invariant
// TAG KEYS:    state   (PRINCIPAL PLANES / MAX SHEAR PLANES / HYDROSTATIC — NO SHEAR ON ANY PLANE / empty)
// MATRIX KEYS: (none)
// DATA-KEY:    formulaAngle, formulaSx1, formulaTxy1 (live formula lines), conventions (sign-convention note)
// CANVAS KEYS: element, mohr
// CONTROL KEYS: preset, sx, sy, txy, theta, theta0, principal, maxShear, show3D, reset
(function () {
  'use strict';
  const RASim = globalThis.RASim;
  const M = RASim.math;

  // ---------------------------------------------------------------- model (pure)
  function transform(sx, sy, txy, theta) {
    const a = (sx + sy) / 2, b = (sx - sy) / 2;
    const c = Math.cos(2 * theta), s = Math.sin(2 * theta);
    return { sx1: a + b * c + txy * s, sy1: a - b * c - txy * s, txy1: -b * s + txy * c };
  }

  function principal(sx, sy, txy) {
    const avg = (sx + sy) / 2, b = (sx - sy) / 2;
    const R = Math.hypot(b, txy);
    const isotropic = R < 1e-9;
    let thetaP = 0;
    if (!isotropic) {
      thetaP = 0.5 * Math.atan2(txy + 0, b + 0);          // + 0: no −0 (atan2(−0, −1) would give −π)
      if (thetaP <= -Math.PI / 2) thetaP += Math.PI;      // keep (−90°, 90°]
    }
    const s1 = avg + R, s2 = avg - R;
    const tauAbs = Math.max(Math.abs(s1 - s2), Math.abs(s1), Math.abs(s2)) / 2;
    return {
      avg: avg, R: R, s1: s1, s2: s2, thetaP: thetaP, thetaS: thetaP - Math.PI / 4, tauMax: R, tauAbs: tauAbs,
      vonMises: Math.sqrt(Math.max(0, s1 * s1 - s1 * s2 + s2 * s2)), tresca: 2 * tauAbs, isotropic: isotropic
    };
  }

  function circlePoints(sx, sy, txy, theta) {
    const t = transform(sx, sy, txy, theta);
    return { C: [(sx + sy) / 2, 0], X: [sx, txy], Y: [sy, 0 - txy], X1: [t.sx1, t.txy1], Y1: [t.sy1, 0 - t.txy1] };
  }

  const model = { transform: transform, principal: principal, circlePoints: circlePoints };

  // ---------------------------------------------------------------- constants
  const ID = 'mohr-circle';
  const SC = '.sim[data-sim="' + ID + '"]';
  const DEFAULTS = { sx: 50, sy: -10, txy: 40, theta: 0, show3D: false };
  const PRESETS = [
    { value: 'general', label: 'General state (50, −10, 40)', s: [50, -10, 40] },
    { value: 'pureShear', label: 'Pure shear τ = 50', s: [0, 0, 50] },
    { value: 'uniaxial', label: 'Uniaxial tension σ = 80', s: [80, 0, 0] },
    { value: 'equalBiaxial', label: 'Equal biaxial 60, 60', s: [60, 60, 0] },
    { value: 'thinCylinder', label: 'Thin cylinder: hoop 80, axial 40', s: [80, 40, 0] },
    { value: 'tensionCompression', label: 'Tension–compression (60, −60, 0)', s: [60, -60, 0] },
    { value: 'biaxialCompression', label: 'Biaxial compression (−40, −80, 0)', s: [-40, -80, 0] }
  ];
  const ARROW_MIN = 0.05;    // MPa: arrows below this are omitted
  const TAG_TOL = 0.05;      // MPa: tolerance of the PRINCIPAL / MAX SHEAR tags
  const LBG = RASim.alpha('#ffffff', 0.88);

  // ---------------------------------------------------------------- rich canvas text: "σ_{x′} = 70.0"
  const richCache = {};
  function parseRich(str) {
    if (richCache[str]) return richCache[str];
    const out = [];
    const re = /_\{([^}]*)\}/g;
    let last = 0, m;
    while ((m = re.exec(str)) !== null) {
      if (m.index > last) out.push({ t: str.slice(last, m.index), sub: false });
      out.push({ t: m[1], sub: true });
      last = re.lastIndex;
    }
    if (last < str.length) out.push({ t: str.slice(last), sub: false });
    richCache[str] = out;
    return out;
  }
  function fonts(size, bold) {
    const b = bold ? 'bold ' : '';
    return { base: b + size + 'px sans-serif', sub: b + '11px sans-serif' };
  }
  function richWidth(ctx, str, size, bold) {
    const f = fonts(size, bold);
    let w = 0;
    parseRich(str).forEach(function (s) { ctx.font = s.sub ? f.sub : f.base; w += ctx.measureText(s.t).width; });
    return w;
  }
  // centre-aligned at (x, y), vertically middle; optional rounded background
  function drawRich(ctx, str, x, y, o) {
    const size = o.size || 12, f = fonts(size, o.bold);
    const segs = parseRich(str);
    const w = richWidth(ctx, str, size, o.bold);
    let left = o.align === 'left' ? x : o.align === 'right' ? x - w : x - w / 2;
    ctx.save();
    if (o.bg) {
      const hh = size * 0.62 + 4;
      ctx.fillStyle = o.bg;
      ctx.beginPath();
      if (ctx.roundRect) ctx.roundRect(left - 3, y - hh, w + 6, 2 * hh + 2, 3); else ctx.rect(left - 3, y - hh, w + 6, 2 * hh + 2);
      ctx.fill();
    }
    ctx.fillStyle = o.color || RASim.colors.ink;
    ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
    segs.forEach(function (s) {
      ctx.font = s.sub ? f.sub : f.base;
      ctx.fillText(s.t, left, s.sub ? y + size * 0.3 : y);
      left += ctx.measureText(s.t).width;
    });
    ctx.restore();
  }

  // ---------------------------------------------------------------- label placement (obstacle-avoiding)
  function segHitsRect(x1, y1, x2, y2, l, t, r, b) {   // Liang–Barsky (allocation-free: it runs ~10⁵ times per draw)
    if ((x1 < l && x2 < l) || (x1 > r && x2 > r) || (y1 < t && y2 < t) || (y1 > b && y2 > b)) return false;
    const dx = x2 - x1, dy = y2 - y1;
    let u0 = 0, u1 = 1;
    for (let i = 0; i < 4; i++) {
      const P = i === 0 ? -dx : i === 1 ? dx : i === 2 ? -dy : dy;
      const Q = i === 0 ? x1 - l : i === 1 ? r - x1 : i === 2 ? y1 - t : b - y1;
      if (P === 0) { if (Q < 0) return false; }
      else {
        const u = Q / P;
        if (P < 0) { if (u > u1) return false; if (u > u0) u0 = u; }
        else { if (u < u0) return false; if (u < u1) u1 = u; }
      }
    }
    return true;
  }
  const RINGS = [0, 8, 18, 32, 50, 72], RING_COST = [0, 8, 18, 40, 110, 220];
  // distance between the segments p1p2 and p3p4
  function ptSeg(px, py, ax, ay, bx, by) {
    const dx = bx - ax, dy = by - ay, l2 = dx * dx + dy * dy;
    const t = l2 > 0 ? M.clamp(((px - ax) * dx + (py - ay) * dy) / l2, 0, 1) : 0;
    return Math.hypot(px - ax - t * dx, py - ay - t * dy);
  }
  function segDist(x1, y1, x2, y2, x3, y3, x4, y4) {
    const d1 = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1), d2 = (x2 - x1) * (y4 - y1) - (y2 - y1) * (x4 - x1);
    const d3 = (x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3), d4 = (x4 - x3) * (y2 - y3) - (y4 - y3) * (x2 - x3);
    if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) return 0;
    return Math.min(ptSeg(x1, y1, x3, y3, x4, y4), ptSeg(x2, y2, x3, y3, x4, y4), ptSeg(x3, y3, x1, y1, x2, y2), ptSeg(x4, y4, x1, y1, x2, y2));
  }
  // leader from the anchor (starting `gap` px out) to the nearest point of the box, or null when the box is close enough
  function leaderOf(anchor, gap, b) {
    const nx = M.clamp(anchor[0], b.l, b.r), ny = M.clamp(anchor[1], b.t, b.b);
    const d = Math.hypot(nx - anchor[0], ny - anchor[1]);
    if (d <= gap + 12) return null;
    return [anchor[0] + (nx - anchor[0]) * gap / d, anchor[1] + (ny - anchor[1]) * gap / d, nx, ny, d];
  }
  function makeLayout(ctx, frame) {
    const segs = [], boxes = [];
    const L = {
      seg: function (x1, y1, x2, y2, hw, wt) { segs.push([x1, y1, x2, y2, hw, wt == null ? 1000 : wt]); },
      dot: function (x, y, r, wt) { segs.push([x, y, x, y, r, wt == null ? 1000 : wt]); },
      // arc in screen pixels, angles y-up radians from a0 to a1 (signed sweep)
      arc: function (cx, cy, r, a0, a1, hw, wt) {
        const n = Math.max(4, Math.ceil(Math.abs(a1 - a0) / 0.2));
        for (let i = 0; i < n; i++) {
          const p = a0 + (a1 - a0) * i / n, q = a0 + (a1 - a0) * (i + 1) / n;
          L.seg(cx + r * Math.cos(p), cy - r * Math.sin(p), cx + r * Math.cos(q), cy - r * Math.sin(q), hw, wt);
        }
      },
      circle: function (cx, cy, r, hw, wt) { L.arc(cx, cy, r, 0, 2 * Math.PI, hw, wt); },
      lineCost: function (ld) {
        // weight of the heaviest obstacle the leader crosses
        let n = 0;
        for (let i = 0; i < segs.length; i++) {
          const q = segs[i];
          if (q[5] > n && segDist(ld[0], ld[1], ld[2], ld[3], q[0], q[1], q[2], q[3]) < q[4] + 1) n = q[5];
        }
        for (let j = 0; j < boxes.length; j++) {
          const o = boxes[j];
          if (o.wt > n && segHitsRect(ld[0], ld[1], ld[2], ld[3], o.l, o.t, o.r, o.b)) n = o.wt;
        }
        return n;                                  // the worst obstacle crossed (a polyline counts once)
      },
      box: function (l, t, r, b, wt) { boxes.push({ l: l, t: t, r: r, b: b, wt: wt == null ? 2000 : wt }); },
      // total weight of the obstacles the box overlaps (label boxes × boxMul); stops early once it reaches
      // `bound` (the caller only needs to know that it is not better than its current best)
      cost: function (bx, bound, boxMul) {
        const lim = bound == null ? Infinity : bound, bm = boxMul || 1;
        let n = 0;
        for (let j = 0; j < boxes.length && n < lim; j++) {
          const o = boxes[j];
          if (bx.l < o.r + 2 && bx.r > o.l - 2 && bx.t < o.b + 2 && bx.b > o.t - 2) n += bm * o.wt;
        }
        for (let i = 0; i < segs.length && n < lim; i++) {
          const s = segs[i];
          if (segHitsRect(s[0], s[1], s[2], s[3], bx.l - s[4], bx.t - s[4], bx.r + s[4], bx.b + s[4])) n += s[5];
        }
        return n;
      },
      // candidate centres around (ax, ay) in directions mid ± i·step (y-up radians), just clear of radius r
      around: function (ax, ay, r, mid, step, n, rings) {
        return function (a, b) {
          const out = [];
          for (let i = 0; i <= n; i++) {
            for (let sgn = 0; sgn < 2; sgn++) {
              if (i === 0 && sgn) continue;
              const u = mid + (sgn ? -i : i) * step, ux = Math.cos(u), uy = Math.sin(u);
              (rings || RINGS).forEach(function (extra, k) {
                const d = r + 3 + a * Math.abs(ux) + b * Math.abs(uy) + extra;
                out.push({ x: ax + d * ux, y: ay - d * uy, cost: 6 * i + RING_COST[k] });
              });
            }
          }
          return out;
        };
      },
      // alts: texts in order of preference; returns { x, y, text, ... } or null (optional label that does not fit)
      place: function (alts, o, candFn) {
        const size = o.size || 12;
        let best = null;
        alts.forEach(function (text, ai) {
          const a = richWidth(ctx, text, size, o.bold) / 2 + 4, b = size * 0.62 + 3;
          candFn(a, b).forEach(function (c) {
            const bx = { l: c.x - a, r: c.x + a, t: c.y - b, b: c.y + b };
            let cost = c.cost + 900 * ai;
            if (bx.l < frame.x + 1 || bx.r > frame.x + frame.w - 1 || bx.t < frame.y + 1 || bx.b > frame.y + frame.h - 1) cost += 5000;
            if (best && cost >= best.cost) return;
            cost += L.cost(bx, best ? best.cost - cost : null, o.boxMul);
            if (best && cost >= best.cost) return;
            if (o.anchor) {                        // a leader line costs its length plus half of what it crosses
              const ld = leaderOf(o.anchor, o.gap || 0, bx);
              if (ld) cost += 1.5 * ld[4] + 0.5 * L.lineCost(ld);
            }
            if (!best || cost < best.cost) best = { cost: cost, x: c.x, y: c.y, text: text, bx: bx };
          });
        });
        if (!best || (o.optional && best.cost >= 1000) || (o.maxCost != null && best.cost > o.maxCost)) return null;
        boxes.push({ l: best.bx.l, t: best.bx.t, r: best.bx.r, b: best.bx.b, wt: 2000 });
        return { x: best.x, y: best.y, text: best.text, size: size, bold: !!o.bold, color: o.color, box: best.bx };
      }
    };
    return L;
  }

  // labels + leader lines: put(alts, style, candidates, anchor, gap); a label placed away from its anchor gets a leader
  function makeLabeler(lay) {
    const labels = [], leaders = [];
    return {
      put: function (alts, o, cands, anchor, gap) {
        const l = lay.place(alts, Object.assign({ anchor: anchor, gap: gap }, o), cands);
        if (!l) return null;
        labels.push(l);
        const ld = anchor ? leaderOf(anchor, gap, l.box) : null;
        if (ld) { leaders.push([ld[0], ld[1], ld[2], ld[3], l.color]); lay.seg(ld[0], ld[1], ld[2], ld[3], 1, 800); }
        return l;
      },
      draw: function (ctx) {
        ctx.save();
        ctx.lineWidth = 1; ctx.setLineDash([]);
        leaders.forEach(function (q) {
          ctx.strokeStyle = RASim.alpha(q[4], 0.75);
          ctx.beginPath(); ctx.moveTo(q[0], q[1]); ctx.lineTo(q[2], q[3]); ctx.stroke();
        });
        ctx.restore();
        labels.forEach(function (l) { drawRich(ctx, l.text, l.x, l.y, { size: l.size, bold: l.bold, color: l.color, bg: LBG }); });
      }
    };
  }

  RASim.register(ID, {
    title: "Mohr's circle for plane stress",
    model: model,
    mount: function (api) {
      const ui = RASim.ui, C = RASim.colors, D = RASim.draw;
      const fmt = RASim.fmt, fmtFixed = RASim.fmtFixed;
      const rad = M.rad, deg = M.deg;
      const st = Object.assign({}, DEFAULTS);

      RASim.css(ID,
        SC + ' .mc-nt { text-transform: none; letter-spacing: 0; font-size: 11.5px; }\n' +
        SC + ' .mc-case { text-transform: none; letter-spacing: 0; }\n' +
        SC + ' .mc-wide { grid-column: 1 / -1; }\n' +
        SC + ' .sim-stats { grid-auto-flow: row dense; }\n' +
        SC + ' .mc-views { display: flex; flex-wrap: wrap; gap: 10px; min-width: 0; }\n' +
        SC + ' .mc-view { flex: 1 1 250px; min-width: 0; display: flex; flex-direction: column; gap: 3px; }\n' +
        SC + ' .mc-cap { font-size: 12px; color: ' + C.muted + '; margin: 0; text-align: center; }\n' +
        SC + ' .mc-formula { font-size: 12.5px; color: ' + C.ink + '; margin: 0; line-height: 1.5; }\n' +
        SC + ' .mc-btns .sim-btn { flex: 1 1 auto; }\n' +
        SC + ' .mc-preset { display: flex; width: 100%; }\n' +
        SC + ' .mc-preset select { flex: 1 1 auto; min-width: 0; width: 100%; }\n');
      function lab(html, color) {
        return '<span class="mc-nt"' + (color ? ' style="color:' + color + '"' : '') + '>' + html + '</span>';
      }

      ui.note(api.root, 'Rotate the element with θ (or drag the red point X′ around the circle) and watch the face stresses and the point on Mohr\'s circle move together, θ on the element being 2θ on the circle.');
      const sp = ui.split(api.root, { ratio: 2 });

      // ---- main column: the two views, legend, conventions, formula lines ----
      const views = ui.el('div', { 'class': 'mc-views' }, sp.main);
      const vE = ui.el('div', { 'class': 'mc-view' }, views);
      const vM = ui.el('div', { 'class': 'mc-view' }, views);
      const cvE = RASim.canvas(vE, { aspect: 0.86, minHeight: 260, maxHeight: 460, key: 'element' });
      ui.el('p', { 'class': 'mc-cap', html: 'Element rotated by θ (faint inset: θ = 0)' }, vE);
      const cvM = RASim.canvas(vM, { aspect: 0.86, minHeight: 260, maxHeight: 460, key: 'mohr' });
      ui.el('p', { 'class': 'mc-cap', html: 'Mohr\'s circle (τ positive downward): drag X′' }, vM);
      const legendItems = [
        { color: C.red, label: 'x′ face, point X′ = (σ<sub>x′</sub>, τ<sub>x′y′</sub>)' },
        { color: C.green, label: 'y′ face, point Y′ = (σ<sub>y′</sub>, −τ<sub>x′y′</sub>)' },
        { color: C.purple, label: 'shear τ<sub>x′y′</sub>, points ±τ<sub>max</sub>' },
        { color: C.teal, label: 'principal σ₁, σ₂' },
        { color: C.orange, label: 'θ on the element = 2θ on the circle' },
        { color: C.gold, label: '2θ<sub>p</sub> (CX → σ₁)' },
        { color: C.faint, label: 'reference X–Y (θ = 0)', dash: true }
      ];
      const legend = ui.legend(sp.main, legendItems);
      const conv = ui.note(sp.main, '<b>Sign conventions.</b> Tension positive. τ<sub>xy</sub> &gt; 0 acts in +y on the +x face (and in +x on the +y face). ' +
        'θ is counter-clockwise positive, from the x-axis to x′ (the normal of the rotated face). Mohr\'s circle: σ to the right, ' +
        'τ <b>positive downward</b>; X′ = (σ<sub>x′</sub>, τ<sub>x′y′</sub>), Y′ = (σ<sub>y′</sub>, −τ<sub>x′y′</sub>). Plane stress: σ₃ = 0.');
      conv.setAttribute('data-key', 'conventions');
      const fA = ui.el('p', { 'class': 'mc-formula' }, sp.main); fA.setAttribute('data-key', 'formulaAngle');
      const fS = ui.el('p', { 'class': 'mc-formula' }, sp.main); fS.setAttribute('data-key', 'formulaSx1');
      const fT = ui.el('p', { 'class': 'mc-formula' }, sp.main); fT.setAttribute('data-key', 'formulaTxy1');

      // ---- side column: controls, tag, stats ----
      const gS = ui.group(sp.side, 'Stress at the point <span class="mc-case">(MPa)</span>');
      const opts = PRESETS.map(function (p) { return { value: p.value, label: p.label }; }).concat([{ value: 'custom', label: 'Custom (set by the sliders)' }]);
      const selP = ui.select(gS, { label: 'Preset', options: opts, value: 'general', key: 'preset',
        onChange: function (v) {
          const p = PRESETS.filter(function (q) { return q.value === v; })[0];
          if (p) setState({ sx: p.s[0], sy: p.s[1], txy: p.s[2], theta: 0 });
          else update();
        } });
      selP.el.classList.add('mc-preset');
      const sSx = ui.slider(gS, { label: 'σ<sub>x</sub>', min: -100, max: 100, step: 1, value: st.sx, unit: 'MPa', key: 'sx',
        onInput: function (v) { st.sx = v; update(); } });
      const sSy = ui.slider(gS, { label: 'σ<sub>y</sub>', min: -100, max: 100, step: 1, value: st.sy, unit: 'MPa', key: 'sy',
        onInput: function (v) { st.sy = v; update(); } });
      const sTxy = ui.slider(gS, { label: 'τ<sub>xy</sub>', min: -100, max: 100, step: 1, value: st.txy, unit: 'MPa', key: 'txy',
        onInput: function (v) { st.txy = v; update(); } });

      const gR = ui.group(sp.side, 'Rotated element');
      const sTh = ui.slider(gR, { label: 'θ (counter-clockwise, x → x′)', min: -90, max: 90, step: 0.5, value: st.theta, unit: '°', key: 'theta',
        onInput: function (v) { st.theta = v; update(); } });
      const rowB = ui.row(gR); rowB.classList.add('mc-btns');
      ui.button(rowB, { label: 'θ = 0', key: 'theta0', onClick: function () { setState({ theta: 0 }); } });
      ui.button(rowB, { label: 'Principal planes (θ = θ<sub>p</sub>)', key: 'principal', title: 'Rotate to the σ₁ direction',
        onClick: function () { setState({ theta: deg(principal(st.sx, st.sy, st.txy).thetaP) }); } });
      ui.button(rowB, { label: 'Max shear planes (θ = θ<sub>s</sub>)', key: 'maxShear', title: 'Rotate to θ_s = θ_p − 45° (τ = +τ_max)',
        onClick: function () { setState({ theta: thetaSDeg(principal(st.sx, st.sy, st.txy)) }); } });

      const gV = ui.group(sp.side, 'Show');
      const rowV = ui.row(gV);
      const c3 = ui.checkbox(rowV, { label: '3D circles (σ₃ = 0)', checked: st.show3D, key: 'show3D',
        onChange: function (v) { st.show3D = v; update(); } });
      ui.button(rowV, { label: 'Reset', key: 'reset', title: 'Restore every default',
        onClick: function () { setState(Object.assign({}, DEFAULTS)); } });

      const rowTag = ui.row(sp.side);
      const tag = ui.tag(rowTag, { text: '', key: 'state' });

      const stats = ui.stats(sp.side);
      const tSx1 = ui.stat(stats, { label: lab('σ<sub>x′</sub> (MPa)', C.red), key: 'sx1' });
      const tSy1 = ui.stat(stats, { label: lab('σ<sub>y′</sub> (MPa)', C.green), key: 'sy1' });
      const tTxy1 = ui.stat(stats, { label: lab('τ<sub>x′y′</sub> (MPa)', C.purple), key: 'txy1' });
      const tAvg = ui.stat(stats, { label: lab('σ<sub>avg</sub> (MPa)'), key: 'avg' });
      const tR = ui.stat(stats, { label: lab('R (MPa)'), key: 'R', title: 'Radius of Mohr\'s circle' });
      const tS1 = ui.stat(stats, { label: lab('σ₁ (MPa)', C.teal), key: 's1' });
      const tS2 = ui.stat(stats, { label: lab('σ₂ (MPa)', C.teal), key: 's2' });
      const tTm = ui.stat(stats, { label: lab('in-plane τ<sub>max</sub> (MPa)'), key: 'tauMax' });
      const tThP = ui.stat(stats, { label: lab('θ<sub>p</sub> (to σ₁) and θ<sub>p</sub> + 90° (to σ₂)'), key: 'thetaP' });
      tThP.el.classList.add('mc-wide');
      const tThS = ui.stat(stats, { label: lab('θ<sub>s</sub> = θ<sub>p</sub> − 45°'), key: 'thetaS', title: 'Plane of maximum in-plane shear, where τ = +τ_max' });
      const tTa = ui.stat(stats, { label: lab('absolute τ<sub>max</sub>, σ₃ = 0 (MPa)'), key: 'tauAbs' });
      const tVm = ui.stat(stats, { label: lab('von Mises σ<sub>vm</sub> (MPa)'), key: 'vonMises', title: '√(σ₁² − σ₁σ₂ + σ₂²)' });
      const tTr = ui.stat(stats, { label: lab('Tresca 2τ<sub>abs</sub> (MPa)'), key: 'tresca' });
      const tInv = ui.stat(stats, { label: lab('invariant'), key: 'invariant' });
      tInv.el.classList.add('mc-wide');

      ui.hints(api.root, [
        'Press <b>Principal planes</b>: the shear vanishes and the normal stresses become σ₁ and σ₂ — the two ends of the circle\'s horizontal diameter.',
        'Press <b>Max shear planes</b>: they are 45° from the principal planes (90° on the circle), and the normal stress there is σ<sub>avg</sub>, not zero.',
        'Preset <b>Pure shear</b>: it is the same as equal tension and compression at 45°.',
        'Preset <b>Thin cylinder</b> with 3D circles on: the in-plane τ<sub>max</sub> is (σ₁ − σ₂)/2 = 20, but the true maximum is σ₁/2 = 40 because σ₃ = 0.',
        'An angle θ on the element is 2θ on the circle, in the same sense when τ is drawn positive downward.'
      ]);

      // ---- helpers ----
      function r1(v) { return fmt(Math.round(v * 10) / 10, 7); }                 // at most 1 decimal, zeros stripped
      function degLbl(d) { return fmt(Math.round(d * 10) / 10, 7) + '°'; }
      function wrap90(d) { let x = d; while (x > 90) x -= 180; while (x <= -90) x += 180; return x; }
      function thetaSDeg(P) { return wrap90(deg(P.thetaS)); }                      // the θ_s plane inside the slider range
      function num(v) { return fmt(v, 8); }
      function paren(s) { return s.charAt(0) === '−' ? '(' + s + ')' : s; }
      // "k1 × f1 + k2 × f2" with real minus signs and no "+ −"
      function sumOfProducts(terms) {
        let out = '';
        terms.forEach(function (t, i) {
          const k = t[0], f = paren(fmtFixed(t[1], 4));
          if (i === 0) out += num(k) + ' × ' + f;
          else out += (k < 0 ? ' − ' : ' + ') + num(Math.abs(k)) + ' × ' + f;
        });
        return out;
      }

      // ================================================================ element canvas
      const STRIP = 62;   // px reserved at the top for the reference inset
      function elemGeom(w, h) {
        const rh = h - STRIP;
        const Rv = Math.max(70, Math.min(w, rh) / 2 - 4);
        const half = Math.round(0.32 * Rv);
        const gap = 12, Lmin = 10, Lmax = Math.max(Lmin + 6, 0.44 * Rv);
        return { cx: w / 2, cy: STRIP + rh / 2, half: half, gap: gap, Lmin: Lmin, Lmax: Lmax,
          reach: half + gap + Lmax + 6, Ls0: 12, Ls1: 1.6 * half, ra: half * Math.SQRT2 + 9 };
      }

      cvE.draw = function (ctx, w, h) {
        const g = elemGeom(w, h);
        const th = rad(st.theta);
        const T = transform(st.sx, st.sy, st.txy, th);
        const P = principal(st.sx, st.sy, st.txy);
        const scale = Math.max(Math.abs(P.s1), Math.abs(P.s2), 1e-9);
        const cx = g.cx, cy = g.cy, hf = g.half;
        const ux = [Math.cos(th), -Math.sin(th)], uy = [-Math.sin(th), -Math.cos(th)];   // x′, y′ on screen (y down)
        const lay = makeLayout(ctx, { x: 0, y: 0, w: w, h: h });

        // reference inset (faint), top-left
        drawInset(ctx, lay, w);

        // fixed axes x, y (thin) and rotated axes x′, y′
        const ext = g.reach;
        ctx.lineWidth = 1; ctx.strokeStyle = C.axis; ctx.setLineDash([]);
        ctx.beginPath(); ctx.moveTo(cx - ext, cy); ctx.lineTo(cx + ext, cy); ctx.moveTo(cx, cy + ext); ctx.lineTo(cx, cy - ext); ctx.stroke();
        lay.seg(cx - ext, cy, cx + ext, cy, 1, 150); lay.seg(cx, cy + ext, cx, cy - ext, 1, 150);
        const xp = [cx + ext * ux[0], cy + ext * ux[1]], yp = [cx + ext * uy[0], cy + ext * uy[1]];
        ctx.setLineDash([5, 4]); ctx.lineWidth = 1.2;
        ctx.strokeStyle = RASim.alpha(C.red, 0.8);
        ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(xp[0], xp[1]); ctx.stroke();
        ctx.strokeStyle = RASim.alpha(C.green, 0.7);
        ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(yp[0], yp[1]); ctx.stroke();
        ctx.setLineDash([]);
        lay.seg(cx, cy, xp[0], xp[1], 1, 150); lay.seg(cx, cy, yp[0], yp[1], 1, 150);

        // the element
        const corner = function (a, b) { return [cx + hf * (a * ux[0] + b * uy[0]), cy + hf * (a * ux[1] + b * uy[1])]; };
        const q = [corner(1, 1), corner(-1, 1), corner(-1, -1), corner(1, -1)];
        ctx.beginPath(); ctx.moveTo(q[0][0], q[0][1]); for (let i = 1; i < 4; i++) ctx.lineTo(q[i][0], q[i][1]); ctx.closePath();
        ctx.fillStyle = C.fill; ctx.fill();
        ctx.strokeStyle = C.maroon; ctx.lineWidth = 2; ctx.stroke();
        for (let i = 0; i < 4; i++) lay.seg(q[i][0], q[i][1], q[(i + 1) % 4][0], q[(i + 1) % 4][1], 2);

        // θ arc from x to x′ (outside the element's corners)
        if (Math.abs(st.theta) > 0.05) {
          D.angleArc(ctx, cx, cy, g.ra, 0, th, { ccw: th >= 0, color: C.orange, width: 2 });
          lay.arc(cx, cy, g.ra, 0, th, 2, 600);
        }

        // stress arrows
        const faces = [
          { n: ux, t: uy, s: 1, v: T.sx1, color: C.red },     // +x′: shear towards +y′ for τ > 0
          { n: [-ux[0], -ux[1]], t: uy, s: -1, v: T.sx1, color: C.red },
          { n: uy, t: ux, s: 1, v: T.sy1, color: C.green },   // +y′: shear towards +x′ for τ > 0
          { n: [-uy[0], -uy[1]], t: ux, s: -1, v: T.sy1, color: C.green }
        ];
        const tips = {};
        faces.forEach(function (f, i) {
          const Fx = cx + hf * f.n[0], Fy = cy + hf * f.n[1];
          // normal stress
          if (Math.abs(f.v) >= ARROW_MIN) {
            const L = g.Lmin + (g.Lmax - g.Lmin) * Math.min(1, Math.abs(f.v) / scale);
            const a = [Fx + g.gap * f.n[0], Fy + g.gap * f.n[1]], b = [Fx + (g.gap + L) * f.n[0], Fy + (g.gap + L) * f.n[1]];
            if (f.v > 0) D.arrow(ctx, a[0], a[1], b[0], b[1], { color: f.color, width: 2.5, head: 9 });
            else D.arrow(ctx, b[0], b[1], a[0], a[1], { color: f.color, width: 2.5, head: 9 });
            lay.seg(a[0], a[1], b[0], b[1], 5);
            const mid = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
            if (i === 0) tips.x = mid; if (i === 2) tips.y = mid;
          } else {
            if (i === 0) tips.x = [Fx + g.gap * f.n[0], Fy + g.gap * f.n[1]];
            if (i === 2) tips.y = [Fx + g.gap * f.n[0], Fy + g.gap * f.n[1]];
          }
          // shear along the face, just outside it
          const tau = T.txy1;
          const ox = Fx + 5 * f.n[0], oy = Fy + 5 * f.n[1];
          if (Math.abs(tau) >= ARROW_MIN) {
            const Ls = g.Ls0 + (g.Ls1 - g.Ls0) * Math.min(1, Math.abs(tau) / scale);
            const sgn = f.s * (tau > 0 ? 1 : -1);
            const d = [sgn * f.t[0], sgn * f.t[1]];
            const a = [ox - d[0] * Ls / 2, oy - d[1] * Ls / 2], b = [ox + d[0] * Ls / 2, oy + d[1] * Ls / 2];
            D.arrow(ctx, a[0], a[1], b[0], b[1], { color: C.purple, width: 2.5, head: 8 });
            lay.seg(a[0], a[1], b[0], b[1], 4);
            if (i === 0) tips.t = b;
          } else if (i === 0) tips.t = [ox + hf * f.t[0], oy + hf * f.t[1]];
        });
        lay.dot(cx, cy, 0.85 * hf, 400);                 // keep labels off the element if there is room outside

        // labels: axis names first (small, at the axis ends), then the values
        const lbl = makeLabeler(lay), put = lbl.put;
        function axisLabel(text, end, dirAng, color) {
          put([text], { size: 12, color: color, optional: true }, lay.around(end[0], end[1], 1, dirAng, Math.PI / 6, 4, [0, 6]));
        }
        axisLabel('x', [cx + ext, cy], 0, C.muted);
        axisLabel('y', [cx, cy - ext], Math.PI / 2, C.muted);
        axisLabel('x′', xp, th, C.red);
        axisLabel('y′', yp, th + Math.PI / 2, C.green);
        if (Math.abs(st.theta) > 0.05) {
          const mid = th / 2, am = [cx + g.ra * Math.cos(mid), cy - g.ra * Math.sin(mid)];
          put(['θ = ' + degLbl(st.theta), 'θ'], { size: 12, color: C.orange }, lay.around(am[0], am[1], 1, mid, Math.PI / 12, 10), am, 3);
        }
        const sxTxt = 'σ_{x′} = ' + fmtFixed(T.sx1, 1), syTxt = 'σ_{y′} = ' + fmtFixed(T.sy1, 1), tTxt = 'τ_{x′y′} = ' + fmtFixed(T.txy1, 1);
        put([sxTxt], { size: 12, bold: true, color: C.red }, lay.around(tips.x[0], tips.x[1], 3, th - Math.PI / 2, Math.PI / 12, 12), tips.x, 4);
        put([syTxt], { size: 12, bold: true, color: C.green }, lay.around(tips.y[0], tips.y[1], 3, th + Math.PI, Math.PI / 12, 12), tips.y, 4);
        put([tTxt], { size: 12, bold: true, color: C.purple }, lay.around(tips.t[0], tips.t[1], 2, th + Math.PI / 4, Math.PI / 12, 12), tips.t, 3);
        lbl.draw(ctx);
      };

      // faint θ = 0 reference element with σ_x, σ_y, τ_xy
      function drawInset(ctx, lay, w) {
        const bx = 6, by = 6, bs = 50;
        const icx = bx + bs / 2, icy = by + bs / 2, hf = 9;
        ctx.save();
        ctx.fillStyle = RASim.alpha(C.panel, 0.9); ctx.strokeStyle = C.grid; ctx.lineWidth = 1;
        ctx.fillRect(bx, by, bs, bs); ctx.strokeRect(bx + 0.5, by + 0.5, bs - 1, bs - 1);
        ctx.globalAlpha = 0.75;
        ctx.fillStyle = RASim.alpha(C.fill, 0.8); ctx.strokeStyle = C.muted; ctx.lineWidth = 1.2;
        ctx.fillRect(icx - hf, icy - hf, 2 * hf, 2 * hf); ctx.strokeRect(icx - hf, icy - hf, 2 * hf, 2 * hf);
        const col = C.muted;
        // normal arrows (fixed small length, direction by sign)
        [[1, 0, st.sx], [-1, 0, st.sx], [0, -1, st.sy], [0, 1, st.sy]].forEach(function (f) {
          if (Math.abs(f[2]) < ARROW_MIN) return;
          const a = [icx + (hf + 3) * f[0], icy + (hf + 3) * f[1]], b = [icx + (hf + 13) * f[0], icy + (hf + 13) * f[1]];
          if (f[2] > 0) D.arrow(ctx, a[0], a[1], b[0], b[1], { color: col, width: 1.5, head: 5 });
          else D.arrow(ctx, b[0], b[1], a[0], a[1], { color: col, width: 1.5, head: 5 });
        });
        // shear: +x face → +y (up on screen) for τ > 0, etc.
        if (Math.abs(st.txy) >= ARROW_MIN) {
          const s = st.txy > 0 ? 1 : -1, o = hf + 3, l = 7;
          D.arrow(ctx, icx + o, icy + s * l, icx + o, icy - s * l, { color: C.purple, width: 1.5, head: 5 });
          D.arrow(ctx, icx - o, icy - s * l, icx - o, icy + s * l, { color: C.purple, width: 1.5, head: 5 });
          D.arrow(ctx, icx - s * l, icy - o, icx + s * l, icy - o, { color: C.purple, width: 1.5, head: 5 });
          D.arrow(ctx, icx + s * l, icy + o, icx - s * l, icy + o, { color: C.purple, width: 1.5, head: 5 });
        }
        ctx.restore();
        const tx = bx + bs + 7;
        drawRich(ctx, 'θ = 0 (given), MPa:', tx, by + 8, { size: 12, color: C.muted, align: 'left' });
        const ref = circlePoints(st.sx, st.sy, st.txy, 0);                 // X = (σ_x, τ_xy), Y = (σ_y, −τ_xy)
        drawRich(ctx, 'σ_{x} = ' + r1(ref.X[0]) + ',  σ_{y} = ' + r1(ref.Y[0]), tx, by + 25, { size: 12, color: C.muted, align: 'left' });
        drawRich(ctx, 'τ_{xy} = ' + r1(ref.X[1]), tx, by + 42, { size: 12, color: C.muted, align: 'left' });
        lay.box(0, 0, Math.max(tx + 150, w * 0.6), STRIP - 2, 3000);
        ctx.strokeStyle = C.grid; ctx.lineWidth = 1;
        ctx.beginPath(); ctx.moveTo(4, STRIP - 0.5); ctx.lineTo(w - 4, STRIP - 0.5); ctx.stroke();
      }

      // ================================================================ Mohr's-circle canvas
      // window: centred on the circle, includes the origin (and the 3D circles); independent of θ
      function mohrPlot(ctx, w, h) {
        const P = principal(st.sx, st.sy, st.txy);
        const W = Math.max(P.R, Math.abs(P.avg), 5) * 1.3;
        const H = Math.max(P.R, st.show3D ? P.tauAbs : 0, 5) * 1.3;
        return RASim.plot(ctx, { box: { x: 0, y: 0, w: w, h: h }, xmin: P.avg - W, xmax: P.avg + W, ymin: -H, ymax: H, equal: true,
          margin: { left: 36, right: 10, top: 22, bottom: 34 }, xlabel: 'σ (MPa)' });
      }
      function ticks(a, b, step) { const out = []; for (let k = Math.ceil(a / step - 1e-9); k * step <= b + 1e-9; k++) out.push(k * step); return out; }

      cvM.draw = function (ctx, w, h) {
        const p = mohrPlot(ctx, w, h);
        const step = M.niceCeil((p.xmax - p.xmin) / Math.max(3, Math.floor(p.rect.w / 55)));
        const tickFmt = function (v) { return fmt(Math.round(v / step) * step, 6); };
        p.axes({ xTicks: ticks(p.xmin, p.xmax, step), yTicks: ticks(p.ymin, p.ymax, step), xFormat: tickFmt,
          yFormat: function (v) { return tickFmt(-v); } });                  // plotted y = −τ: label positive below the axis
        D.text(ctx, 'τ (positive ↓)', 4, 11, { color: C.ink, font: '12px sans-serif' });

        const th = rad(st.theta);
        const P = principal(st.sx, st.sy, st.txy);
        const cp = circlePoints(st.sx, st.sy, st.txy, th);
        const k = p.X(1) - p.X(0);                                         // px per MPa (equal axes)
        const S = function (pt) { return [p.X(pt[0]), p.Y(-pt[1])]; };      // [σ, τ] → screen
        const c = S(cp.C), X = S(cp.X), Y = S(cp.Y), X1 = S(cp.X1), Y1 = S(cp.Y1);
        const Rpx = P.R * k;
        const r = p.rect;
        const lay = makeLayout(ctx, r);
        const big = Rpx >= 3;
        lay.seg(r.x, c[1], r.x + r.w, c[1], 1, 60);
        const zx = p.X(0);
        if (zx > r.x && zx < r.x + r.w) lay.seg(zx, r.y, zx, r.y + r.h, 1, 60);

        ctx.save();
        ctx.beginPath(); ctx.rect(r.x, r.y, r.w, r.h); ctx.clip();

        // 3D circles (σ₃ = 0) and the absolute maximum shear
        let absPt = null;
        if (st.show3D) {
          ctx.strokeStyle = C.muted; ctx.lineWidth = 1.5; ctx.setLineDash([6, 4]);
          [P.s1, P.s2].forEach(function (s) {
            if (Math.abs(s) * k < 1) return;
            ctx.beginPath(); ctx.arc(p.X(s / 2), c[1], Math.abs(s) / 2 * k, 0, 2 * Math.PI); ctx.stroke();
            lay.circle(p.X(s / 2), c[1], Math.abs(s) / 2 * k, 1, 300);
          });
          ctx.setLineDash([]);
          // governing circle: in-plane (σ₁, σ₂), (σ₁, 0) or (σ₂, 0)
          const d12 = Math.abs(P.s1 - P.s2), d1 = Math.abs(P.s1), d2 = Math.abs(P.s2);
          const cAbs = d12 >= d1 && d12 >= d2 ? P.avg : d1 >= d2 ? P.s1 / 2 : P.s2 / 2;
          absPt = [p.X(cAbs), p.Y(-P.tauAbs)];
        }

        // Mohr's circle
        ctx.beginPath(); ctx.arc(c[0], c[1], Math.max(Rpx, 0.5), 0, 2 * Math.PI);
        ctx.fillStyle = RASim.alpha(C.lightBlue, 0.14); ctx.fill();
        ctx.strokeStyle = C.blue; ctx.lineWidth = 2.2; ctx.stroke();
        if (big) lay.circle(c[0], c[1], Rpx, 1.5);

        // reference diameter X–Y (faint)
        if (big) {
          ctx.strokeStyle = C.faint; ctx.lineWidth = 1.5; ctx.setLineDash([5, 4]);
          ctx.beginPath(); ctx.moveTo(X[0], X[1]); ctx.lineTo(Y[0], Y[1]); ctx.stroke(); ctx.setLineDash([]);
          lay.seg(X[0], X[1], Y[0], Y[1], 1.5, 400);
          ctx.fillStyle = C.faint;
          [X, Y].forEach(function (q) { ctx.beginPath(); ctx.arc(q[0], q[1], 3.5, 0, 2 * Math.PI); ctx.fill(); });
        }

        // arcs: 2θ (CX → CX′, orange) and 2θ_p (CX → σ₁, thin gold)
        const a0 = Math.atan2(-st.txy, (st.sx - st.sy) / 2);              // y-up angle of CX (τ positive downward)
        const ra = M.clamp(0.3 * Rpx, 14, 34), rp = ra + 9;
        let arc2 = null, arcP = null;
        if (big && Rpx > 22) {
          const sweepP = 2 * P.thetaP;
          if (Math.abs(sweepP) > 1e-3) {
            D.angleArc(ctx, c[0], c[1], rp, a0, a0 + sweepP, { ccw: sweepP >= 0, color: C.gold, width: 1.5 });
            lay.arc(c[0], c[1], rp, a0, a0 + sweepP, 1.5, 400);
            arcP = { mid: a0 + sweepP / 2 };
          }
          if (Math.abs(st.theta) > 0.05) {
            D.angleArc(ctx, c[0], c[1], ra, a0, a0 + 2 * th, { ccw: th >= 0, color: C.orange, width: 2.2 });
            lay.arc(c[0], c[1], ra, a0, a0 + 2 * th, 2, 600);
            arc2 = { mid: a0 + th };
          }
        }

        // current diameter X′–Y′ (bold)
        if (big) {
          ctx.strokeStyle = C.maroon; ctx.lineWidth = 3; ctx.lineCap = 'round';
          ctx.beginPath(); ctx.moveTo(X1[0], X1[1]); ctx.lineTo(Y1[0], Y1[1]); ctx.stroke(); ctx.lineCap = 'butt';
          lay.seg(X1[0], X1[1], Y1[0], Y1[1], 2.5);
        }

        // principal points, max-shear points, centre
        const pS1 = [p.X(P.s1), c[1]], pS2 = [p.X(P.s2), c[1]];
        const pTb = [c[0], p.Y(-P.R)], pTt = [c[0], p.Y(P.R)];               // +τ_max (below), −τ_max (above)
        function dot(q, color, rr) { ctx.beginPath(); ctx.arc(q[0], q[1], rr, 0, 2 * Math.PI); ctx.fillStyle = color; ctx.fill(); ctx.strokeStyle = '#ffffff'; ctx.lineWidth = 1.5; ctx.stroke(); lay.dot(q[0], q[1], rr + 1); }
        if (big) { dot(pS1, C.teal, 4.5); dot(pS2, C.teal, 4.5); dot(pTb, C.purple, 4); dot(pTt, C.purple, 4); }
        dot(c, C.ink, 3.5);
        if (absPt) {
          ctx.save(); ctx.translate(absPt[0], absPt[1]); ctx.rotate(Math.PI / 4);
          ctx.fillStyle = C.maroon; ctx.fillRect(-4.5, -4.5, 9, 9); ctx.strokeStyle = '#ffffff'; ctx.lineWidth = 1.2; ctx.strokeRect(-4.5, -4.5, 9, 9);
          ctx.restore(); lay.dot(absPt[0], absPt[1], 7);
          const o = [p.X(0), c[1]];
          ctx.beginPath(); ctx.arc(o[0], o[1], 3.5, 0, 2 * Math.PI); ctx.fillStyle = C.muted; ctx.fill(); lay.dot(o[0], o[1], 4);
        }
        // Y′ and the X′ handle
        if (big) {
          dot(Y1, C.green, 5);
          ctx.beginPath(); ctx.arc(X1[0], X1[1], 8, 0, 2 * Math.PI); ctx.fillStyle = '#ffffff'; ctx.fill();
          ctx.strokeStyle = C.red; ctx.lineWidth = 2.5; ctx.stroke();
          ctx.beginPath(); ctx.arc(X1[0], X1[1], 4, 0, 2 * Math.PI); ctx.fillStyle = C.red; ctx.fill();
          lay.dot(X1[0], X1[1], 10);
        }
        ctx.restore();

        // ---- labels (most important first) ----
        const PI = Math.PI;
        const lbl = makeLabeler(lay), put = lbl.put;
        const xy1 = '(' + fmtFixed(cp.X1[0], 1) + ', ' + fmtFixed(cp.X1[1], 1) + ')';
        const same = !!absPt && Math.abs(P.tauAbs - P.R) < 1e-9;       // τ_abs is the in-plane τ_max
        if (big) {
          const ang = Math.atan2(c[1] - X1[1], X1[0] - c[0]);
          // X′ with its coordinates beside the point when that is cheap; otherwise "X′" there and the coordinates in a free top corner
          if (!put(['X′ ' + xy1], { size: 12, bold: true, color: C.red, maxCost: 150 }, lay.around(X1[0], X1[1], 9, ang, PI / 12, 12), X1, 10)) {
            const tw = richWidth(ctx, 'X′ ' + xy1, 12, true) / 2 + 6;
            put(['X′ ' + xy1], { size: 12, bold: true, color: C.red }, function () {
              return [{ x: r.x + r.w - tw, y: r.y + 12, cost: 0 }, { x: r.x + tw, y: r.y + 12, cost: 5 },
                { x: r.x + r.w - tw, y: r.y + r.h - 12, cost: 10 }, { x: r.x + tw, y: r.y + r.h - 12, cost: 15 }];
            });
            put(['X′'], { size: 12, bold: true, color: C.red }, lay.around(X1[0], X1[1], 9, ang, PI / 12, 12), X1, 10);
          }
          put(['Y′'], { size: 12, bold: true, color: C.green }, lay.around(Y1[0], Y1[1], 6, ang + PI, PI / 12, 12), Y1, 7);
        } else if (P.isotropic) {
          put(['X′ = Y′ ' + xy1], { size: 12, bold: true, color: C.red }, lay.around(c[0], c[1], 5, PI / 2, PI / 12, 12), c, 5);
        } else {
          // circle too small to draw its points: X′ and Y′ are distinct, so give both coordinates
          const yy1 = '(' + fmtFixed(cp.Y1[0], 1) + ', ' + fmtFixed(cp.Y1[1], 1) + ')';
          put(['X′ ' + xy1, 'X′'], { size: 12, bold: true, color: C.red }, lay.around(c[0], c[1], 5, PI / 2, PI / 12, 12), c, 5);
          put(['Y′ ' + yy1, 'Y′'], { size: 12, bold: true, color: C.green }, lay.around(c[0], c[1], 5, -PI / 2, PI / 12, 12), c, 5);
        }
        // a value label that has no room beside its point keeps its name there and moves its value to a free
        // corner of the plot (placed below, before the optional labels), so every required value stays on the canvas
        const deferred = [];
        function putVal(full, short, cornerText, o, cands, anchor, gap) {
          const l = put([full, short], Object.assign({ maxCost: 1999 }, o), cands, anchor, gap);   // ≥ 2000: on another label
          if (!l || (l.text !== full && cornerText)) deferred.push({ text: cornerText || full, o: o });   // cornerText null: short has the value
        }
        if (arc2) {
          const m = arc2.mid;
          const am = [c[0] + ra * Math.cos(m), c[1] - ra * Math.sin(m)];
          putVal('2θ = ' + degLbl(2 * st.theta), '2θ', '2θ = ' + degLbl(2 * st.theta), { size: 12, bold: true, color: C.orange }, lay.around(am[0], am[1], 1, m, PI / 12, 12), am, 3);
        } else if (big && Math.abs(st.theta) > 0.05) {
          deferred.push({ text: '2θ = ' + degLbl(2 * st.theta), o: { size: 12, bold: true, color: C.orange } });   // circle too small for the arc
        }
        if (absPt && !same) putVal('τ_{abs} = ' + r1(P.tauAbs), 'τ_{abs}', 'τ_{abs} = ' + r1(P.tauAbs), { size: 12, bold: true, color: C.maroon },
          lay.around(absPt[0], absPt[1], 6, -PI / 2, PI / 12, 12), absPt, 7);
        if (big) {
          putVal('σ_{avg} = ' + r1(P.avg), 'C', 'C: σ_{avg} = ' + r1(P.avg), { size: 12, color: C.ink }, lay.around(c[0], c[1], 4, -PI / 2, PI / 12, 12), c, 5);
          putVal('σ₁ = ' + r1(P.s1), 'σ₁', 'σ₁ = ' + r1(P.s1), { size: 12, color: C.teal }, lay.around(pS1[0], pS1[1], 5, -PI / 4, PI / 12, 12), pS1, 6);
          putVal('σ₂ = ' + r1(P.s2), 'σ₂', 'σ₂ = ' + r1(P.s2), { size: 12, color: C.teal }, lay.around(pS2[0], pS2[1], 5, -3 * PI / 4, PI / 12, 12), pS2, 6);
          if (same) putVal('τ_{max} = τ_{abs} = ' + r1(P.R), 'τ_{abs} = ' + r1(P.R), null,
            { size: 12, bold: true, color: C.maroon }, lay.around(pTb[0], pTb[1], 5, -PI / 2, PI / 12, 12), pTb, 6);
          else putVal('τ_{max} = ' + r1(P.R), '+τ_{max}', 'τ_{max} = ' + r1(P.R), { size: 12, color: C.purple }, lay.around(pTb[0], pTb[1], 5, -PI / 2, PI / 12, 12), pTb, 6);
          put(['−τ_{max}'], { size: 12, color: C.purple, optional: true }, lay.around(pTt[0], pTt[1], 5, PI / 2, PI / 12, 12), pTt, 6);
        } else if (P.isotropic) {
          put(['σ₁ = σ₂ = ' + r1(P.s1)], { size: 12, color: C.teal }, lay.around(c[0], c[1], 5, -PI / 2, PI / 12, 12), c, 5);
          put(['σ_{avg} = ' + r1(P.avg)], { size: 12, color: C.ink, optional: true }, lay.around(c[0], c[1], 4, PI, PI / 12, 12), c, 5);
        } else {
          // σ₁ ≠ σ₂ although the circle is too small to separate them on screen
          putVal('σ₁ = ' + r1(P.s1) + ', σ₂ = ' + r1(P.s2), 'σ₁, σ₂', 'σ₁ = ' + r1(P.s1) + ', σ₂ = ' + r1(P.s2), { size: 12, color: C.teal }, lay.around(c[0], c[1], 5, -PI / 2, PI / 12, 12), c, 5);
          put(['σ_{avg} = ' + r1(P.avg)], { size: 12, color: C.ink, optional: true }, lay.around(c[0], c[1], 4, PI, PI / 12, 12), c, 5);
        }
        deferred.forEach(function (d) {
          // never on top of another label (it may cross lines: labels have a white background)
          put([d.text], Object.assign({ boxMul: 10 }, d.o), function (a, b) {
            const out = [];
            // a grid over the plot, cheapest at the bottom and top edges and at the sides
            const xs = [r.x + r.w - a - 2, r.x + a + 2, r.x + r.w / 2, r.x + r.w / 4 + a / 2, r.x + 3 * r.w / 4 - a / 2];
            const rows = Math.max(1, Math.floor((r.h / 2 - b - 2) / (2 * b + 2)) + 1);
            for (let k = 0; k < rows; k++) {
              const yb = r.y + r.h - b - 2 - k * (2 * b + 2), yt = r.y + b + 2 + k * (2 * b + 2);
              xs.forEach(function (x, j) { out.push({ x: x, y: yb, cost: 20 * k + 4 * j }, { x: x, y: yt, cost: 20 * k + 4 * j + 2 }); });
            }
            return out;
          });
        });
        if (absPt) put(['σ₃ = 0', 'σ₃'], { size: 12, color: C.muted, optional: true }, lay.around(p.X(0), c[1], 4, 3 * PI / 4, PI / 12, 12), [p.X(0), c[1]], 5);
        if (big && Math.abs(st.theta) > 0.05 && Math.hypot(X[0] - X1[0], X[1] - X1[1]) > 12) {
          const ax = Math.atan2(c[1] - X[1], X[0] - c[0]);
          put(['X'], { size: 12, color: C.muted, optional: true }, lay.around(X[0], X[1], 4, ax, PI / 12, 8));
          put(['Y'], { size: 12, color: C.muted, optional: true }, lay.around(Y[0], Y[1], 4, ax + PI, PI / 12, 8));
        }
        if (arcP) {
          const m = arcP.mid;
          const am = [c[0] + rp * Math.cos(m), c[1] - rp * Math.sin(m)];
          put(['2θ_{p}'], { size: 12, color: C.gold, optional: true }, lay.around(am[0], am[1], 1, m, PI / 12, 8, [0, 8, 18, 32]), am, 3);
        }
        lbl.draw(ctx);
      };

      // ---- drag X′ around the circle: θ = ½·(angle of CX′ − angle of CX), wrapped into (−90°, 90°] ----
      function handlePx(p) { const q = circlePoints(st.sx, st.sy, st.txy, rad(st.theta)).X1; return [p.X(q[0]), p.Y(-q[1])]; }
      // the handle exists only when X′ is drawn (circle at least 3 px in radius, as in cvM.draw)
      function onHandle(x, y) {
        const P = principal(st.sx, st.sy, st.txy);
        if (P.isotropic) return null;
        const p = mohrPlot(cvM.ctx, cvM.width, cvM.height);
        if (P.R * (p.X(1) - p.X(0)) < 3) return null;
        const hp = handlePx(p);
        return Math.hypot(hp[0] - x, hp[1] - y) <= 16 ? p : null;
      }
      function pointerAngle(p, x, y) { return Math.atan2(p.invY(y), p.invX(x) - (st.sx + st.sy) / 2); }  // y-up, seen from C
      cvM.onDrag({
        // the token keeps the angle between the pointer and X′ at the grab, so θ does not jump when X′ is
        // grabbed off its centre (no side effects here: hit can run twice for one touch)
        hit: function (x, y) {
          const p = onHandle(x, y);
          if (!p) return null;
          const hp = handlePx(p);
          return { p: p, off: M.wrapPi(pointerAngle(p, x, y) - pointerAngle(p, hp[0], hp[1])) };
        },
        move: function (x, y, tok) {
          const p = tok.p;
          const phi = pointerAngle(p, x, y) - tok.off;                    // y-up angle of CX′
          const phi0 = Math.atan2(-st.txy, (st.sx - st.sy) / 2);          // y-up angle of CX (τ positive downward)
          const t = deg(M.wrapPi(phi - phi0)) / 2;                        // (−90°, 90°]
          st.theta = t; sTh.set(t); st.theta = sTh.get();
          update();
        },
        cursor: function (x, y) { return onHandle(x, y) ? 'grab' : null; }
      });

      // ---- read-outs ----
      function presetOf() {
        for (let i = 0; i < PRESETS.length; i++) {
          const s = PRESETS[i].s;
          if (s[0] === st.sx && s[1] === st.sy && s[2] === st.txy) return PRESETS[i].value;
        }
        return 'custom';
      }
      function update() {
        const th = rad(st.theta);
        const T = transform(st.sx, st.sy, st.txy, th);
        const P = principal(st.sx, st.sy, st.txy);

        tSx1.set(fmtFixed(T.sx1, 1)); tSy1.set(fmtFixed(T.sy1, 1)); tTxy1.set(fmtFixed(T.txy1, 1));
        tAvg.set(fmtFixed(P.avg, 2)); tR.set(fmtFixed(P.R, 2));
        tS1.set(fmtFixed(P.s1, 2)); tS2.set(fmtFixed(P.s2, 2));
        if (P.isotropic) tThP.set('0° — R = 0: every direction is principal');
        else tThP.set(fmtFixed(deg(P.thetaP), 2) + '° and ' + fmtFixed(deg(P.thetaP) + 90, 2) + '°');
        tTm.set(fmtFixed(P.tauMax, 2));
        const ts = thetaSDeg(P), tsModel = deg(P.thetaS);
        if (P.isotropic) tThS.set(fmtFixed(ts, 2) + '° (τ = 0 on every plane)');
        else tThS.set(fmtFixed(ts, 2) + '°' + (Math.abs(ts - tsModel) > 1e-9 ? ' <span class="mc-nt">(≡ ' + fmtFixed(tsModel, 2) + '°)</span>' : ''));
        tTa.set(fmtFixed(P.tauAbs, 2)); tTa.setState(P.tauAbs > P.tauMax + 1e-9 ? 'warn' : '');
        tVm.set(fmtFixed(P.vonMises, 2)); tTr.set(fmtFixed(P.tresca, 2));
        tInv.set('σ<sub>x′</sub> + σ<sub>y′</sub> = σ<sub>x</sub> + σ<sub>y</sub> = ' + fmt(T.sx1 + T.sy1, 6));

        if (P.isotropic) tag.set('HYDROSTATIC — NO SHEAR ON ANY PLANE', 'ok');
        else if (Math.abs(T.txy1) < TAG_TOL) tag.set('PRINCIPAL PLANES', 'ok');
        else if (Math.abs(Math.abs(T.txy1) - P.R) < TAG_TOL && P.R > TAG_TOL) tag.set('MAX SHEAR PLANES', 'warn');
        else tag.set('', '');

        selP.set(presetOf());
        legend.set(st.show3D ? legendItems.concat([{ color: C.muted, label: '3D circles (σ₃ = 0)', dash: true }, { color: C.maroon, label: 'τ<sub>abs</sub>' }]) : legendItems);

        // live formula lines (printed numbers reproduce the printed result)
        const a = P.avg, b = (st.sx - st.sy) / 2, cc = Math.cos(2 * th), ss = Math.sin(2 * th);
        fA.innerHTML = '2θ = ' + fmtFixed(2 * st.theta, 2) + '°: cos 2θ = ' + fmtFixed(cc, 4) + ', sin 2θ = ' + fmtFixed(ss, 4);
        fS.innerHTML = 'σ<sub>x′</sub> = σ<sub>avg</sub> + ½(σ<sub>x</sub> − σ<sub>y</sub>) cos 2θ + τ<sub>xy</sub> sin 2θ = ' +
          num(a) + (b < 0 ? ' − ' : ' + ') + num(Math.abs(b)) + ' × ' + paren(fmtFixed(cc, 4)) +
          (st.txy < 0 ? ' − ' : ' + ') + num(Math.abs(st.txy)) + ' × ' + paren(fmtFixed(ss, 4)) + ' = <b>' + fmtFixed(T.sx1, 1) + '</b>';
        fT.innerHTML = 'τ<sub>x′y′</sub> = −½(σ<sub>x</sub> − σ<sub>y</sub>) sin 2θ + τ<sub>xy</sub> cos 2θ = ' +
          sumOfProducts([[-b, ss], [st.txy, cc]]) + ' = <b>' + fmtFixed(T.txy1, 1) + '</b>';

        cvE.redraw(); cvM.redraw();
      }

      function getState() { return { sx: st.sx, sy: st.sy, txy: st.txy, theta: st.theta, show3D: st.show3D }; }
      function setState(s) {
        s = s || {};
        if (s.sx != null && isFinite(Number(s.sx))) { sSx.set(Number(s.sx)); st.sx = sSx.get(); }
        if (s.sy != null && isFinite(Number(s.sy))) { sSy.set(Number(s.sy)); st.sy = sSy.get(); }
        if (s.txy != null && isFinite(Number(s.txy))) { sTxy.set(Number(s.txy)); st.txy = sTxy.get(); }
        if (s.theta != null && isFinite(Number(s.theta))) {
          let t = Number(s.theta);
          if (t < -90 || t > 90) t = wrap90(t);
          sTh.set(t); st.theta = sTh.get();
        }
        if (s.show3D != null) { st.show3D = !!s.show3D; c3.set(st.show3D); }
        update();
      }

      update();
      return { getState: getState, setState: setState };
    }
  });
})();
