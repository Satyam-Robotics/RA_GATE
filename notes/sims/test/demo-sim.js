// Core demo: exercises every RASim.ui builder, canvas drag + hover, RASim.plot (linear, log-x, equal axes),
// RASim.draw helpers, api.animate and tfStep. Lives in test/ so the build never includes it.
// Run: node notes/sims/test/harness.js demo --file notes/sims/test/demo-sim.js --out <dir>
(function () {
  'use strict';
  const RASim = globalThis.RASim;
  const M = RASim.math;

  const model = {
    secondOrder: function (zeta, wn) { return { num: [wn * wn], den: [1, 2 * zeta * wn, wn * wn] }; },
    poles: function (zeta, wn) { return M.polyRoots([1, 2 * zeta * wn, wn * wn]); },
    specs: function (zeta, wn) {
      if (zeta >= 1 || zeta <= 0) return { overshoot: zeta >= 1 ? 0 : NaN, tp: NaN, ts: zeta > 0 ? 4 / (zeta * wn) : Infinity };
      const wd = wn * Math.sqrt(1 - zeta * zeta);
      return { overshoot: 100 * Math.exp(-Math.PI * zeta / Math.sqrt(1 - zeta * zeta)), tp: Math.PI / wd, ts: 4 / (zeta * wn) };
    },
    damping: function (zeta) { return zeta <= 0 ? 'undamped' : zeta < 1 ? 'underdamped' : zeta === 1 ? 'critically damped' : 'overdamped'; },
    // pole drag: a point (sigma, omega) in the upper half plane -> (zeta, wn)
    fromPole: function (sigma, omega) {
      const wn = Math.hypot(sigma, omega);
      return { wn: wn, zeta: wn > 0 ? M.clamp(-sigma / wn, 0, 1) : 1 };
    }
  };

  RASim.register('demo', {
    title: 'Core demo',
    model: model,
    mount: function (api) {
      const ui = RASim.ui, C = RASim.colors;
      RASim.css('demo', '.sim[data-sim="demo"] .demo-caption { font-size: 12px; color: #666; margin: 0; }');
      const st = { zeta: 0.4, wn: 2, tEnd: 10, input: 'step', view: 'both', envelope: true, damping: 1, show: 'all' };

      const sp = ui.split(api.root, { ratio: 1.6 });

      // ---- main column: plots ----
      const cvStep = RASim.canvas(sp.main, { aspect: 2, key: 'step' });
      const legend = ui.legend(sp.main, [
        { color: C.blue, label: 'y(t) from tfStep' },
        { color: C.orange, label: 'envelope 1 ± e<sup>−ζω<sub>n</sub>t</sup>/√(1−ζ²)', dash: true },
        { color: C.green, label: '2 % band' }
      ]);
      const cvBode = RASim.canvas(sp.main, { aspect: 2.2, key: 'bode' });
      const twin = ui.row(sp.main);
      const planeBox = ui.el('div', { style: { flex: '1 1 220px', minWidth: '0' } }, twin);
      const massBox = ui.el('div', { style: { flex: '1 1 220px', minWidth: '0' } }, twin);
      const cvPlane = RASim.canvas(planeBox, { aspect: 1.1, maxWidth: 360, key: 'splane' });
      ui.el('p', { 'class': 'demo-caption', html: 's-plane (equal axes): drag the pole' }, planeBox);
      const cvMass = RASim.canvas(massBox, { aspect: 1.5, maxWidth: 360, key: 'mass' });
      ui.el('p', { 'class': 'demo-caption', html: 'Free-form canvas with RASim.draw helpers' }, massBox);

      // ---- side column: controls ----
      const g1 = ui.group(sp.side, 'System');
      const sZeta = ui.slider(g1, { label: 'Damping ratio ζ', min: 0, max: 2, step: 0.01, value: st.zeta, key: 'zeta', onInput: v => { st.zeta = v; update(); } });
      const sWn = ui.slider(g1, { label: 'Natural frequency ω<sub>n</sub>', min: 0.1, max: 100, value: st.wn, scale: 'log', unit: 'rad/s', key: 'wn', onInput: v => { st.wn = v; update(); } });
      const nEnd = ui.number(g1, { label: 'Plot to t =', value: st.tEnd, min: 0.5, max: 100, step: 0.5, unit: 's', key: 'tend', onInput: v => { st.tEnd = v; update(); } });
      const g2 = ui.group(sp.side, 'Display');
      const selInput = ui.select(g2, { label: 'Input', options: [{ value: 'step', label: 'Unit step' }, { value: 'impulse', label: 'Unit impulse' }], value: 'step', key: 'input', onChange: v => { st.input = v; update(); } });
      const seg = ui.segmented(g2, { label: 'Plots', options: [{ value: 'both', label: 'Both' }, { value: 'time', label: 'Time' }, { value: 'freq', label: 'Bode' }], value: 'both', key: 'view', onChange: v => { st.view = v; update(); } });
      const chk = ui.checkbox(g2, { label: 'Show envelope', checked: true, key: 'envelope', onChange: v => { st.envelope = v; update(); } });
      const row = ui.row(g2);
      const anim = api.animate(function (dt, t) { animT = (animT + dt) % st.tEnd; cvMass.drawNow(); });
      let animT = 0;
      const bPlay = ui.button(row, { label: 'Play', primary: true, key: 'play', onClick: () => { anim.toggle(); bPlay.textContent = anim.running ? 'Pause' : 'Play'; } });
      ui.button(row, { label: 'Reset', key: 'reset', title: 'Back to defaults', onClick: () => { setState({ zeta: 0.4, wn: 2, tEnd: 10, input: 'step', view: 'both', envelope: true }); } });
      const tag = ui.tag(row, { text: '', key: 'damping' });

      const stats = ui.stats(sp.side);
      const tOS = ui.stat(stats, { label: 'Overshoot', key: 'overshoot', title: 'Percent overshoot' });
      const tTp = ui.stat(stats, { label: 'Peak time', key: 'tp' });
      const tTs = ui.stat(stats, { label: 'Settling (2 %)', key: 'ts' });
      const tPole = ui.stat(stats, { label: 'Poles', key: 'poles' });
      const mat = ui.matrix(sp.side, { label: 'A =', key: 'A' });
      ui.note(sp.side, 'Controllable canonical form: ẋ = Ax + Bu. The read-out under the cursor uses <b>onHover</b>.');
      const readout = ui.note(sp.side, 'Hover over the step plot.');

      // ---- drawing ----
      let hoverX = null, stepData = null;
      function response() {
        // impulse response of G(s) = step response of s·G(s)
        const tf = model.secondOrder(st.zeta, st.wn);
        const num = st.input === 'impulse' ? M.polyMul(tf.num, [1, 0]) : tf.num;
        return M.tfStep(num, tf.den, st.tEnd, 400);
      }
      cvStep.draw = function (ctx, w, h) {
        stepData = response();
        const ys = stepData.y;
        let ymax = Math.max(1.2, Math.max.apply(null, ys.filter(isFinite)) * 1.1);
        let ymin = Math.min(0, Math.min.apply(null, ys.filter(isFinite)) * 1.1);
        const p = RASim.plot(ctx, { box: { x: 0, y: 0, w: w, h: h }, xmin: 0, xmax: st.tEnd, ymin: ymin, ymax: ymax, xlabel: 'time t (s)', ylabel: st.input === 'impulse' ? 'impulse response' : 'step response y(t)', title: 'Linear plot' });
        p.axes();
        if (st.input === 'step') p.band(0.98, 1.02, { fill: RASim.alpha(C.green, 0.15) });
        p.area(stepData.t, ys, { fill: RASim.alpha(C.lightBlue, 0.35) });
        p.hline(st.input === 'step' ? 1 : 0, { color: C.faint, width: 1, dash: [4, 3] });
        if (st.envelope && st.zeta > 0 && st.zeta < 1) {
          const k = 1 / Math.sqrt(1 - st.zeta * st.zeta);
          p.fn(t => 1 + k * Math.exp(-st.zeta * st.wn * t), { color: C.orange, width: 1.5, dash: [6, 4] });
          p.fn(t => 1 - k * Math.exp(-st.zeta * st.wn * t), { color: C.orange, width: 1.5, dash: [6, 4] });
        }
        p.line(stepData.t, ys, { color: C.blue, width: 2.5 });
        const sp2 = model.specs(st.zeta, st.wn);
        if (st.input === 'step' && isFinite(sp2.tp) && sp2.tp < st.tEnd) {
          const yp = 1 + sp2.overshoot / 100;
          p.vline(sp2.tp, { color: C.maroon, width: 1, dash: [3, 3] });
          p.point(sp2.tp, yp, { color: C.maroon });
          p.text(sp2.tp, yp, 'peak ' + RASim.fmt(yp, 3), { dx: 8, dy: -10, color: C.maroon, bg: 'rgba(255,255,255,0.85)' });
        }
        if (hoverX !== null) {
          const i = Math.round(hoverX / st.tEnd * 400);
          if (i >= 0 && i <= 400) {
            p.vline(stepData.t[i], { color: C.purple, width: 1 });
            p.point(stepData.t[i], ys[i], { color: C.purple, r: 3.5 });
          }
        }
      };
      cvStep.onHover({
        move: function (x, y) {
          const p = RASim.plot(cvStep.ctx, { box: { x: 0, y: 0, w: cvStep.width, h: cvStep.height }, xmin: 0, xmax: st.tEnd, ymin: 0, ymax: 1, title: 'x' });
          const t = p.invX(x);
          if (!p.contains(x, p.rect.y + 1) || t < 0 || t > st.tEnd) { hoverX = null; readout.innerHTML = 'Hover over the step plot.'; cvStep.redraw(); return; }
          hoverX = t;
          const i = Math.round(t / st.tEnd * 400);
          readout.innerHTML = 't = ' + RASim.fmt(stepData.t[i], 3) + ' s, y = <b>' + RASim.fmt(stepData.y[i], 4) + '</b>';
          cvStep.redraw();
        },
        leave: function () { hoverX = null; readout.innerHTML = 'Hover over the step plot.'; cvStep.redraw(); }
      });

      cvBode.draw = function (ctx, w, h) {
        const tf = model.secondOrder(st.zeta, st.wn);
        const wmin = st.wn / 100, wmax = st.wn * 100;
        const mag = w2 => 20 * Math.log10(M.cabs(M.tfEval(tf.num, tf.den, M.cx(0, w2))));
        const p = RASim.plot(ctx, { box: { x: 0, y: 0, w: w, h: h }, xmin: wmin, xmax: wmax, ymin: -80, ymax: 20, xlog: true, xlabel: 'ω (rad/s)', ylabel: '|G(jω)| (dB)', title: 'Log-x plot (Bode magnitude)' });
        p.axes({ yTicks: [-80, -60, -40, -20, 0, 20] });
        p.vband(st.wn / Math.SQRT2, st.wn * Math.SQRT2, { fill: RASim.alpha(C.gold, 0.12) });
        p.fn(mag, { color: C.blue, width: 2.5 });
        p.segment(wmin, 0, st.wn, 0, { color: C.faint, width: 1.5, dash: [5, 4] });
        p.segment(st.wn, 0, wmax, -80, { color: C.faint, width: 1.5, dash: [5, 4] });
        p.vline(st.wn, { color: C.maroon, width: 1 });
        p.text(st.wn, 12, 'ω = ω' + 'ₙ', { dx: 6, color: C.maroon });
      };

      function planeRange() { const r = Math.max(1, st.wn * 1.35); return r; }
      function planePlot(ctx, w, h) {
        const r = planeRange();
        return RASim.plot(ctx, { box: { x: 0, y: 0, w: w, h: h }, xmin: -r, xmax: r * 0.4, ymin: -r, ymax: r, equal: true, margin: { left: 40, bottom: 30 }, xlabel: 'σ', ylabel: 'jω' });
      }
      cvPlane.draw = function (ctx, w, h) {
        const p = planePlot(ctx, w, h);
        p.axes();
        p.circle(0, 0, st.wn, { color: C.faint, width: 1, dash: [4, 4] });
        const poles = model.poles(st.zeta, st.wn);
        const up = poles.reduce((a, b) => (b.im > a.im ? b : a), poles[0]);
        if (up.im > 0) {
          p.segment(0, 0, up.re, up.im, { color: C.gold, width: 1.5 });
          p.arc(0, 0, 26, Math.PI - Math.acos(Math.min(1, st.zeta)), Math.PI, { color: C.gold, width: 1.5 });
          p.text(0, 0, 'θ = cos⁻¹ζ', { dx: -30, dy: 12, align: 'right', color: C.gold, font: '11px sans-serif' });
        }
        p.arrow(p.xmax * 0.9, p.ymin * 0.88, p.xmax * 0.9 - planeRange() * 0.4, p.ymin * 0.88, { color: C.teal, width: 1.5, head: 7 });
        p.text(p.xmax * 0.9, p.ymin * 0.88, 'more stable', { dy: -11, align: 'right', color: C.teal, font: '11px sans-serif' });
        p.ring(-st.wn * 1.2, 0, { color: C.blue });
        poles.forEach(q => p.cross(q.re, q.im, { color: C.red, size: 6, width: 2.5 }));
        p.text(up.re, up.im, RASim.fmtComplex(up, 3), { dx: 10, dy: -12, color: C.red, bg: 'rgba(255,255,255,0.85)', font: '11px sans-serif' });
      };
      cvPlane.onDrag({
        hit: function (x, y) {
          const p = planePlot(cvPlane.ctx, cvPlane.width, cvPlane.height);
          const poles = model.poles(st.zeta, st.wn);
          const up = poles.reduce((a, b) => (b.im > a.im ? b : a), poles[0]);
          const d = Math.hypot(p.X(up.re) - x, p.Y(up.im) - y);
          return d < 16 ? { p: p } : null;
        },
        move: function (x, y, token) {
          const p = token.p;
          const s = Math.min(-1e-3, p.invX(x)), w = Math.max(0, p.invY(y));
          const r = model.fromPole(s, w);
          setState({ zeta: Number(r.zeta.toFixed(2)), wn: M.clamp(Number(r.wn.toPrecision(3)), 0.1, 100) }, true);
        },
        end: function () { cvPlane.redraw(); },
        cursor: function (x, y) {
          const p = planePlot(cvPlane.ctx, cvPlane.width, cvPlane.height);
          const poles = model.poles(st.zeta, st.wn);
          const up = poles.reduce((a, b) => (b.im > a.im ? b : a), poles[0]);
          return Math.hypot(p.X(up.re) - x, p.Y(up.im) - y) < 16 ? 'grab' : null;
        }
      });

      cvMass.draw = function (ctx, w, h) {
        const D = RASim.draw;
        const data = stepData || response();
        const i = Math.min(400, Math.max(0, Math.round(animT / st.tEnd * 400)));
        const yv = data.y[i] || 0;
        const wallX = 22, baseY = h - 34;
        D.hatch(ctx, wallX, 20, wallX, baseY, { side: 1, color: C.ink });
        D.hatch(ctx, wallX, baseY, w - 12, baseY, { side: 1, color: C.ink });
        const rest = w * 0.45, scale = w * 0.18;
        const mx = M.clamp(rest + scale * yv, wallX + 40, w - 70);
        const my = baseY - 52;
        D.spring(ctx, wallX, my + 12, mx, my + 12, { coils: 7, amplitude: 8, color: C.blue });
        ctx.fillStyle = C.fill; ctx.strokeStyle = C.maroon; ctx.lineWidth = 2;
        ctx.fillRect(mx, my - 8, 52, 50); ctx.strokeRect(mx, my - 8, 52, 50);
        D.text(ctx, 'm', mx + 26, my + 17, { align: 'center', font: 'bold 14px sans-serif', color: C.maroon });
        D.arrow(ctx, mx + 52, my + 17, mx + 52 + 36, my + 17, { color: C.red, width: 2 });
        D.text(ctx, 'F', mx + 52 + 40, my + 17, { color: C.red });
        D.arrow(ctx, rest + 26, 28, mx + 26, 28, { color: C.green, width: 1.5, head: 6, dash: [4, 3] });
        D.text(ctx, 'x = ' + RASim.fmt(yv, 3), Math.max(rest, mx) + 30, 28, { color: C.green, bg: 'rgba(255,255,255,0.9)' });
        const cx = 60, cy = Math.max(52, baseY - 100);
        const ang = M.rad(20 + 40 * yv);
        D.arrow(ctx, cx, cy, cx + 50 * Math.cos(ang), cy - 50 * Math.sin(ang), { color: C.purple, width: 2 });
        ctx.strokeStyle = C.faint; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx + 60, cy); ctx.stroke();
        D.angleArc(ctx, cx, cy, 24, 0, ang, { color: C.purple, label: 'θ' });
        D.text(ctx, 't = ' + RASim.fmt(animT, 3) + ' s', w - 10, 14, { align: 'right', color: C.muted, font: '11px sans-serif' });
      };

      function update() {
        const sp2 = model.specs(st.zeta, st.wn);
        tOS.set(isFinite(sp2.overshoot) ? RASim.fmt(sp2.overshoot, 3) + ' %' : '—');
        tOS.setState(sp2.overshoot > 20 ? 'bad' : sp2.overshoot > 5 ? 'warn' : 'ok');
        tTp.set(RASim.fmt(sp2.tp, 3) + (isFinite(sp2.tp) ? ' s' : ''));
        tTs.set(RASim.fmt(sp2.ts, 3) + ' s');
        const poles = model.poles(st.zeta, st.wn);
        tPole.set(poles[0].im > 0 ? RASim.fmt(poles[0].re, 3) + ' ± j' + RASim.fmt(poles[0].im, 3) : poles.map(q => RASim.fmt(q.re, 3)).join(', '));
        const d = model.damping(st.zeta);
        tag.set(d, st.zeta < 1 ? (st.zeta < 0.3 ? 'bad' : 'warn') : 'ok');
        mat.set([[0, 1], [-st.wn * st.wn, -2 * st.zeta * st.wn]], { decimals: 2, colColors: [C.blue, C.red] });
        cvBode.canvas.style.display = st.view === 'time' ? 'none' : '';
        cvStep.canvas.style.display = st.view === 'freq' ? 'none' : '';
        legend.el.style.display = st.view === 'freq' ? 'none' : '';
        legend.set([
          { color: C.blue, label: st.input === 'step' ? 'y(t) from tfStep' : 'g(t) impulse response' },
          { color: C.orange, label: 'envelope', dash: true },
          { color: C.green, label: '2 % band' }
        ]);
        stepData = null;
        cvStep.redraw(); cvBode.redraw(); cvPlane.redraw(); cvMass.redraw();
      }

      function getState() {
        return { zeta: st.zeta, wn: st.wn, tEnd: st.tEnd, input: st.input, view: st.view, envelope: st.envelope, running: anim.running, animT: animT };
      }
      function setState(s, fromDrag) {
        s = s || {};
        if (s.zeta != null) { st.zeta = M.clamp(Number(s.zeta), 0, 2); sZeta.set(st.zeta); }
        if (s.wn != null) { st.wn = M.clamp(Number(s.wn), 0.1, 100); sWn.set(st.wn); }
        if (s.tEnd != null) { st.tEnd = M.clamp(Number(s.tEnd), 0.5, 100); nEnd.set(st.tEnd); }
        if (s.input != null) { st.input = String(s.input); selInput.set(st.input); }
        if (s.view != null) { st.view = String(s.view); seg.set(st.view); }
        if (s.envelope != null) { st.envelope = !!s.envelope; chk.set(st.envelope); }
        if (s.running != null && !!s.running !== anim.running) { if (s.running) anim.start(); else anim.stop(); bPlay.textContent = anim.running ? 'Pause' : 'Play'; }
        update();
        if (fromDrag) cvPlane.drawNow();
      }
      ui.hints(api.root, [
        'Drag the <b>×</b> pole in the s-plane: ζ and ω<sub>n</sub> follow.',
        'Set ζ above 1 and watch the tag switch to <b>overdamped</b>.',
        'Press <b>Play</b> to animate the mass; scroll away and it pauses.'
      ]);
      update();
      return { getState: getState, setState: setState };
    }
  });
})();
