// second-order-step — "Second-order step response and pole location"
// Sits beside notes/src/04_electrical.html, section "Transient response" (B1.3 Control Systems).
//
// SIGN CONVENTIONS AND MODEL
//   G(s) = ωₙ²/(s² + 2ζωₙs + ωₙ²), unit step input, zero initial conditions, t in s, ωₙ in rad/s.
//   s = σ + jω: σ (real part) to the right, jω (imaginary part) up; the left half-plane (σ < 0) is stable.
//   kind: 'undamped' (ζ = 0), 'critical' (|ζ − 1| < 1e-6), 'under' (0 < ζ < 1), 'over' (ζ > 1).
//   Poles: under −ζωₙ ± jω_d with ω_d = ωₙ√(1 − ζ²) (upper pole first); undamped ±jωₙ; critical −ωₙ twice;
//   over s₁ = −ωₙ(ζ − √(ζ² − 1)) (slow pole, evaluated as −ωₙ/(ζ + √(ζ² − 1))), s₂ = −ωₙ(ζ + √(ζ² − 1)).
//   θ = cos⁻¹ζ is the angle between the NEGATIVE real axis and the radius to the upper pole (shown in degrees).
//   step(): y = 1 − e^(−ζωₙt)·[C + ζωₙt·S] with, for ζ < 1, C = cos ω_d t, S = sin(ω_d t)/(ω_d t)
//   (identical to 1 − e^(−ζωₙt)·sin(ω_d t + θ)/√(1 − ζ²)), and for ζ > 1, C = cosh βt, S = sinh(βt)/(βt),
//   β = ωₙ√(ζ² − 1) (identical to 1 − (s₂e^(s₁t) − s₁e^(s₂t))/(s₂ − s₁), which is used once βt ≥ 1).
//   At ζ = 1 both forms reduce exactly to 1 − (1 + ωₙt)e^(−ωₙt). This exact form is used for EVERY ζ (also inside
//   the critical band |ζ − 1| < 1e-6), so step() is continuous in ζ and agrees with the exact response everywhere.
//   Under-damped specs: t_r = (π − θ)/ω_d (0 → 100 %), t_p = π/ω_d, M_p = e^(−πζ/√(1 − ζ²)) (a fraction; shown in %),
//   t_s ≈ 4/(ζωₙ) (2 %) or 3/(ζωₙ) (5 %), N ≈ t_s·ω_d/(2π). ζ = 0: M_p = 1, t_s = ∞.
//   For ζ ≥ 1 there is no overshoot (M_p = 0) and no peak (t_p, t_r, t_s formula: NaN, shown as "—").
//   M_r = 1/(2ζ√(1 − ζ²)) at ω_r = ωₙ√(1 − 2ζ²) only for ζ < 1/√2 (else NaN, "—");
//   ω_b = ωₙ√[(1 − 2ζ²) + √(4ζ⁴ − 4ζ² + 2)] for every ζ.
//   settleActual(): the last time |y − 1| exceeds the band (exact: the extrema of y − 1 are ±M_p^k at t = kπ/ω_d).
//   tEnd() = niceCeil(clamp(6/σ, 8/ωₙ, 40/ωₙ)), σ = ζωₙ (ζ ≤ 1) or |s₁| (ζ > 1); σ = 0 → 40/ωₙ.
//
// STATE KEYS (getState / setState)
//   zeta, wn (rad/s), band ('2' | '5'), showEnvelope, showMarkers (booleans),
//   held (boolean; setState({ held: true }) holds the current curve if none is held, false clears it).
//   setState applies held LAST, so setState({ zeta, wn, held: true }) holds the new curve.
//
// STAT KEYS:   poles, wd, theta, tr, tp, Mp, ts, tsActual, N, tr1090, Mr, wr, wb
// TAG KEYS:    kind   (UNDAMPED / UNDERDAMPED / CRITICALLY DAMPED / OVERDAMPED)
// MATRIX KEYS: (none)
// DATA-KEY:    tf     (transfer-function line "G(s) = 4 / (s² + 2s + 4)")
// CANVAS KEYS: step, splane
// CONTROL KEYS: zeta, wn, band, showEnvelope, showMarkers,
//               preset0, preset0.2, preset0.5, preset0.707, preset1, preset2, hold, clearHeld, reset
(function () {
  'use strict';
  const RASim = globalThis.RASim;
  const M = RASim.math;
  const ID = 'second-order-step';
  const PI = Math.PI;
  const KIND_TOL = 1e-6;

  // ---------------------------------------------------------------- model (pure)
  function classify(zeta) {
    if (!(zeta > 0)) return 'undamped';
    if (Math.abs(zeta - 1) < KIND_TOL) return 'critical';
    return zeta < 1 ? 'under' : 'over';
  }

  function poles(zeta, wn) {
    const k = classify(zeta);
    if (k === 'undamped') return [{ re: 0, im: wn }, { re: 0, im: -wn }];
    if (k === 'under') {
      const wd = wn * Math.sqrt((1 - zeta) * (1 + zeta));
      return [{ re: -zeta * wn, im: wd }, { re: -zeta * wn, im: -wd }];
    }
    if (k === 'critical') return [{ re: -wn, im: 0 }, { re: -wn, im: 0 }];
    const r = Math.sqrt((zeta - 1) * (zeta + 1));
    return [{ re: -wn / (zeta + r), im: 0 }, { re: -wn * (zeta + r), im: 0 }];
  }

  // step response for ωₙ = 1 at dimensionless time x = ωₙt (exact for every ζ ≥ 0)
  function stepN(z, x) {
    if (!(x > 0)) return 0;
    const sx = z * x;
    if (z < 1) {
      const a = Math.sqrt((1 - z) * (1 + z)) * x;
      const sinc = a === 0 ? 1 : Math.sin(a) / a;
      return 1 - Math.exp(-sx) * (Math.cos(a) + sx * sinc);
    }
    const b = Math.sqrt((z - 1) * (z + 1)), a = b * x;
    if (a < 1) {
      const sinhc = a === 0 ? 1 : Math.sinh(a) / a;
      return 1 - Math.exp(-sx) * (Math.cosh(a) + sx * sinhc);
    }
    const s1 = -1 / (z + b), s2 = -(z + b);
    return 1 - (s2 * Math.exp(s1 * x) - s1 * Math.exp(s2 * x)) / (s2 - s1);
  }
  function step(zeta, wn, t) { return stepN(zeta, wn * t); }

  function bisect(f, lo, hi) {
    let flo = f(lo);
    if (flo === 0) return lo;
    const fhi = f(hi);
    if (fhi === 0) return hi;
    if (!(flo * fhi < 0)) return NaN;
    for (let i = 0; i < 200; i++) {
      const mid = 0.5 * (lo + hi);
      if (mid <= lo || mid >= hi) break;
      const fm = f(mid);
      if (fm === 0) return mid;
      if ((fm < 0) === (flo < 0)) { lo = mid; flo = fm; } else hi = mid;
    }
    return 0.5 * (lo + hi);
  }
  // first x with stepN(z, x) = level for a monotonic (ζ ≥ 1) response, 0 < level < 1
  function riseTo(z, level) {
    let hi = 1;
    while (stepN(z, hi) < level && hi < 1e9) hi *= 2;
    return bisect(function (x) { return stepN(z, x) - level; }, 0, hi);
  }
  // first x with y = level on the initial monotonic rise of an oscillatory response (0 ≤ ζ < 1), 0 < level ≤ 1
  function riseOsc(z, level) {
    const w = Math.sqrt((1 - z) * (1 + z)), xr = (PI - Math.acos(z)) / w;   // y(xr) = 1, y increasing on [0, π/w]
    if (level === 1) return xr;
    return bisect(function (x) { return stepN(z, x) - level; }, 0, xr);
  }

  function settleN(z, band) {
    const k = classify(z);
    if (k === 'undamped') return Infinity;
    if (k !== 'under') return riseTo(z, 1 - band);
    const w = Math.sqrt((1 - z) * (1 + z)), th = Math.acos(z), lnMp = -PI * z / w;
    // extrema of y − 1 at x_n = nπ/w have |y − 1| = M_p^n; find the last n with M_p^n > band
    let n = Math.max(0, Math.ceil(Math.log(band) / lnMp) - 1);
    while (Math.exp((n + 1) * lnMp) > band) n++;
    while (n > 0 && !(Math.exp(n * lnMp) > band)) n--;
    if (n === 0) return riseOsc(z, 1 - band);
    // |y − 1| decreases monotonically from M_p^n at x_n to 0 at the next zero of y − 1, x = ((n + 1)π − θ)/w
    return bisect(function (x) { return Math.abs(stepN(z, x) - 1) - band; }, n * PI / w, ((n + 1) * PI - th) / w);
  }
  function settleActual(zeta, wn, band) {
    const x = settleN(zeta, band);
    return x === Infinity ? Infinity : x / wn;
  }

  function riseTimes(zeta, wn) {
    const osc = classify(zeta) === 'under' || classify(zeta) === 'undamped';
    const f = osc ? riseOsc : riseTo;
    return { t10: f(zeta, 0.1) / wn, t90: f(zeta, 0.9) / wn };
  }
  function rise1090(zeta, wn) { const r = riseTimes(zeta, wn); return r.t90 - r.t10; }

  function bandwidth(zeta, wn) {
    const a = 1 - 2 * zeta * zeta, rb = Math.sqrt(4 * zeta * zeta * zeta * zeta - 4 * zeta * zeta + 2);
    // a + √b; for a < 0 use the identity a + √b = 1/(√b − a) (b − a² = 1) to avoid cancellation
    return wn * Math.sqrt(a >= 0 ? a + rb : 1 / (rb - a));
  }

  function specs(zeta, wn) {
    const kind = classify(zeta);
    const out = { kind: kind, wd: NaN, thetaDeg: NaN, tr: NaN, tp: NaN, Mp: NaN, ts2: NaN, ts5: NaN, N2: NaN,
      Mr: NaN, wr: NaN, wb: bandwidth(zeta, wn) };
    if (kind === 'under' || kind === 'undamped') {
      const s = Math.sqrt((1 - zeta) * (1 + zeta)), wd = wn * s, th = Math.acos(zeta);
      out.wd = wd;
      out.thetaDeg = th * 180 / PI;
      out.tr = (PI - th) / wd;
      out.tp = PI / wd;
      if (kind === 'undamped') {
        out.Mp = 1; out.ts2 = Infinity; out.ts5 = Infinity; out.N2 = Infinity;
      } else {
        out.Mp = Math.exp(-PI * zeta / s);
        out.ts2 = 4 / (zeta * wn);
        out.ts5 = 3 / (zeta * wn);
        out.N2 = out.ts2 * wd / (2 * PI);
      }
      if (zeta < Math.SQRT1_2) {
        out.Mr = 1 / (2 * zeta * s);
        out.wr = wn * Math.sqrt(1 - 2 * zeta * zeta);
      }
    } else {
      out.Mp = 0;
      if (kind === 'critical') out.thetaDeg = 0;
    }
    return out;
  }
  // N ≈ t_s·ω_d/(2π) for the chosen band (band = 0.02 or 0.05)
  function oscillations(zeta, wn, band) {
    const s = specs(zeta, wn);
    const ts = band === 0.05 ? s.ts5 : s.ts2;
    return ts * s.wd / (2 * PI);
  }

  function tEnd(zeta, wn) {
    const k = classify(zeta);
    const sigma = k === 'over' ? Math.abs(poles(zeta, wn)[0].re) : zeta * wn;
    const raw = sigma > 0 ? M.clamp(6 / sigma, 8 / wn, 40 / wn) : 40 / wn;
    return M.niceCeil(raw);
  }

  function fromPole(re, im) {
    const wn = Math.hypot(re, im);
    return { zeta: wn > 0 ? -re / wn + 0 : 1, wn: wn };
  }

  const model = { classify: classify, poles: poles, step: step, specs: specs, settleActual: settleActual,
    rise1090: rise1090, tEnd: tEnd, fromPole: fromPole, riseTimes: riseTimes, oscillations: oscillations };

  // ---------------------------------------------------------------- UI constants
  const DEFAULTS = { zeta: 0.5, wn: 2, band: '2', showEnvelope: true, showMarkers: true };
  const ZMIN = 0, ZMAX = 2.5, WMIN = 0.5, WMAX = 10;
  const PRESETS = [0, 0.2, 0.5, 0.707, 1, 2];
  const NS = 600;           // samples over [0, tEnd]
  const YTOPS = [1.3, 1.8, 2.1];

  RASim.register(ID, {
    title: 'Second-order step response and pole location',
    model: model,
    mount: function (api) {
      const ui = RASim.ui, C = RASim.colors, D = RASim.draw;
      const fmt = RASim.fmt;
      const SC = '.sim[data-sim="' + ID + '"]';
      const st = Object.assign({}, DEFAULTS);
      let held = null;          // { zeta, wn } of the held curve
      let dragW = null;         // s-plane window frozen while a drag is in progress
      const F11 = '11px sans-serif', F12 = '12px sans-serif', B12 = 'bold 12px sans-serif';
      const LBG = RASim.alpha('#ffffff', 0.88);
      const COL = { y: C.blue, held: C.faint, env: C.orange, band: RASim.alpha(C.green, 0.14), peak: C.maroon,
        tr: C.purple, ts: C.teal, tsA: C.green, pole: C.red, wn: C.purple, zr: C.gold };

      RASim.css(ID,
        SC + ' .sos-nt { text-transform: none; letter-spacing: 0; font-size: 11.5px; }\n' +
        SC + ' .sos-wide { grid-column: 1 / -1; }\n' +
        SC + ' .sos-stats { grid-template-columns: repeat(2, minmax(0, 1fr)); }\n' +
        SC + ' .sos-tf { font-size: 13.5px; color: ' + C.ink + '; margin: 2px 0 8px; font-variant-numeric: tabular-nums; }\n' +
        SC + ' .sos-pre { gap: 6px 6px; }\n' +
        SC + ' .sos-pre .sim-btn { min-width: 0; padding-left: 9px; padding-right: 9px; }\n' +
        SC + ' .sos-prelab { font-size: 13px; color: ' + C.muted + '; }\n' +
        SC + ' .sos-cap { margin-top: 2px; }\n' +
        SC + ' .sim-legend { align-content: flex-start; }\n');
      function lab(html) { return '<span class="sos-nt">' + html + '</span>'; }
      const WN = 'ω<sub>n</sub>';

      ui.note(api.root, 'Change ζ and ω<sub>n</sub> (or drag the upper pole × in the s-plane) and watch how the pole position sets the overshoot, the oscillation and the settling of the step response.');
      const sp = ui.split(api.root, { ratio: 1.5 });

      // ---- main column ----
      const cvS = RASim.canvas(sp.main, { aspect: 1.6, minHeight: 250, maxHeight: 380, key: 'step' });
      const legend = ui.legend(sp.main, []);
      const cvZ = RASim.canvas(sp.main, { aspect: 0.85, minHeight: 260, maxHeight: 500, maxWidth: 420, key: 'splane' });
      const cap = ui.note(sp.main, 's-plane (equal axes): drag the upper pole ×. Dashed: the circle of constant ω<sub>n</sub> and the ray of constant ζ.');
      cap.classList.add('sos-cap');

      // ---- side column ----
      const gS = ui.group(sp.side, 'System');
      const sZ = ui.slider(gS, { label: 'Damping ratio ζ', min: ZMIN, max: ZMAX, step: 0.01, decimals: 3, value: st.zeta, key: 'zeta',
        onInput: function (v) { st.zeta = v; update(); } });
      const sW = ui.slider(gS, { label: 'Natural frequency ' + WN, min: WMIN, max: WMAX, step: 0.1, decimals: 2, value: st.wn, unit: 'rad/s', key: 'wn',
        onInput: function (v) { st.wn = v; update(); } });
      const rowP = ui.row(gS);
      rowP.classList.add('sos-pre');
      ui.el('span', { 'class': 'sos-prelab', html: 'ζ =' }, rowP);
      PRESETS.forEach(function (z) {
        ui.button(rowP, { label: fmt(z, 3), key: 'preset' + z, title: 'Set ζ = ' + fmt(z, 3) + ' (ω' + 'ₙ unchanged)',
          onClick: function () { sZ.set(z); st.zeta = sZ.get(); update(); } });
      });

      const gD = ui.group(sp.side, 'Display');
      const segBand = ui.segmented(gD, { label: 'Settling band', key: 'band', value: st.band,
        options: [{ value: '2', label: '2 %' }, { value: '5', label: '5 %' }],
        onChange: function (v) { st.band = v === '5' ? '5' : '2'; update(); } });
      const rowC = ui.row(gD);
      const cEnv = ui.checkbox(rowC, { label: 'Envelope', checked: st.showEnvelope, key: 'showEnvelope',
        onChange: function (v) { st.showEnvelope = v; update(); } });
      const cMk = ui.checkbox(rowC, { label: 'Spec markers', checked: st.showMarkers, key: 'showMarkers',
        onChange: function (v) { st.showMarkers = v; update(); } });
      const rowB = ui.row(gD);
      ui.button(rowB, { label: 'Hold curve', key: 'hold', primary: true, title: 'Keep the current response as a grey reference curve',
        onClick: function () { hold(); update(); } });
      const bClear = ui.button(rowB, { label: 'Clear held', key: 'clearHeld', title: 'Remove the grey reference curve',
        onClick: function () { held = null; update(); } });
      ui.button(rowB, { label: 'Reset', key: 'reset', title: 'Restore every default',
        onClick: function () { setState(Object.assign({ held: false }, DEFAULTS)); } });

      const rowTag = ui.row(sp.side);
      const tag = ui.tag(rowTag, { text: '', key: 'kind' });
      const tfLine = ui.note(sp.side, '');
      tfLine.setAttribute('data-key', 'tf');
      tfLine.classList.add('sos-tf');

      const stats = ui.stats(sp.side);
      stats.classList.add('sos-stats');
      const tPo = ui.stat(stats, { label: lab('poles s₁, s₂'), key: 'poles' });
      const tWd = ui.stat(stats, { label: lab('ω<sub>d</sub> = ' + WN + '√(1 − ζ²)'), key: 'wd' });
      const tTh = ui.stat(stats, { label: lab('θ = cos⁻¹ζ'), key: 'theta' });
      const tTr = ui.stat(stats, { label: lab('t<sub>r</sub> = (π − θ)/ω<sub>d</sub>'), key: 'tr', title: 'Rise time 0 → 100 % (under-damped formula)' });
      const tTp = ui.stat(stats, { label: lab('t<sub>p</sub> = π/ω<sub>d</sub>'), key: 'tp', title: 'Peak time' });
      const tMp = ui.stat(stats, { label: lab('overshoot M<sub>p</sub>'), key: 'Mp' });
      const tN = ui.stat(stats, { label: lab('N ≈ t<sub>s</sub>ω<sub>d</sub>/2π'), key: 'N', title: 'Oscillations before settling (formula t_s)' });
      const tTs = ui.stat(stats, { label: lab('t<sub>s</sub>'), key: 'ts', title: 'Settling time from the formula' });
      const tTsA = ui.stat(stats, { label: lab('t<sub>s</sub> actual'), key: 'tsActual', title: 'Last time |y − 1| leaves the band' });
      const tR19 = ui.stat(stats, { label: lab('t<sub>r</sub> 10–90 %'), key: 'tr1090', title: 'Rise time from 10 % to 90 % (numerical)' });
      const tWb = ui.stat(stats, { label: lab('bandwidth ω<sub>b</sub>'), key: 'wb', title: '−3 dB bandwidth' });
      const tMr = ui.stat(stats, { label: lab('M<sub>r</sub> = 1/(2ζ√(1 − ζ²))'), key: 'Mr', title: 'Resonant peak (ζ < 0.7071)' });
      const tWr = ui.stat(stats, { label: lab('ω<sub>r</sub> = ' + WN + '√(1 − 2ζ²)'), key: 'wr', title: 'Resonant frequency (ζ < 0.7071)' });
      tPo.el.classList.add('sos-wide');
      const tsLabel = tTs.el.querySelector('.sim-stat-label');
      const tsALabel = tTsA.el.querySelector('.sim-stat-label');

      ui.hints(api.root, [
        'Keep ζ fixed and change ω<sub>n</sub>: the overshoot does not change — only the time scale does. In the s-plane the poles slide along a ray through the origin.',
        'Keep ω<sub>n</sub> fixed and change ζ: the poles move on a circle of radius ω<sub>n</sub>, and the overshoot depends only on ζ.',
        'ζ ≈ 0.7 gives about 4.3 % overshoot — the usual compromise between speed and overshoot.',
        'ζ = 0 puts the poles on the jω-axis (sustained oscillation); for ζ ≥ 1 both poles are real and there is no overshoot.',
        'Press <b>Hold curve</b>, then drag the pole: compare the old and new responses.'
      ]);

      // ---------------------------------------------------------------- helpers
      function bandFrac() { return st.band === '5' ? 0.05 : 0.02; }
      function hold() { held = { zeta: st.zeta, wn: st.wn }; }

      // cached samples
      let cacheY = null, cacheH = null;
      function samples(zeta, wn, T, cache) {
        const key = zeta + '|' + wn + '|' + T;
        if (cache && cache.key === key) return cache;
        const ts = new Array(NS + 1), ys = new Array(NS + 1);
        let ymax = 0;
        for (let i = 0; i <= NS; i++) {
          const t = T * i / NS;
          ts[i] = t; ys[i] = model.step(zeta, wn, t);
          if (ys[i] > ymax) ymax = ys[i];
        }
        return { key: key, ts: ts, ys: ys, ymax: ymax };
      }
      function curves() {
        const T = model.tEnd(st.zeta, st.wn);
        cacheY = samples(st.zeta, st.wn, T, cacheY);
        if (held) cacheH = samples(held.zeta, held.wn, T, cacheH); else cacheH = null;
        return { T: T, y: cacheY, h: cacheH };
      }

      // ---------------------------------------------------------------- rich canvas labels (subscripts)
      // a label is a string or an array of parts: ['t', ['p']] (an inner array is a subscript)
      function partsOf(lbl) { return typeof lbl === 'string' ? [lbl] : lbl; }
      function subFont(font) { return font.replace(/(\d+(?:\.\d+)?)px/, function (m0, n) { return Math.round(Number(n) * 0.8) + 'px'; }); }
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
      // label placement: candidates are scored by overlaps with blocked boxes and by leaving the frame
      // The placed label boxes are kept on the canvas element (el.sosLabels) for the browser regression tests.
      // frame: labels never leave it; inner (optional, inside frame): leaving it costs 60 (e.g. the plot's top margin).
      function placer(ctx, frame, el, inner) {
        const boxes = [];
        function ov(a, b) { return a.l < b.r && a.r > b.l && a.t < b.b && a.b > b.t; }
        const out = [];
        return {
          // w: cost of covering this box (100: never cover; small: a thin line a label may cross on its white background)
          block: function (x, y, rx, ry, w) { boxes.push({ l: x - rx, r: x + rx, t: y - ry, b: y + ry, w: w || 100 }); },
          blockBox: function (b) { boxes.push(b); },
          // cands: [{ x, y, align, lbl? }] (y = middle); the first candidate without conflicts wins
          put: function (lbl, font, color, cands) {
            let best = null;
            cands.forEach(function (c, n) {
              const L = c.lbl || lbl;
              const tw = richWidth(ctx, L, font);
              const l = c.align === 'left' ? c.x : c.align === 'right' ? c.x - tw : c.x - tw / 2;
              const b = { l: l - 3, r: l + tw + 3, t: c.y - 9, b: c.y + 9 };   // = the drawn background
              let cost = n + (c.cost || 0);
              boxes.forEach(function (o) { if (ov(b, o)) cost += o.w || 100; });
              if (b.l < frame.x || b.r > frame.x + frame.w || b.t < frame.y || b.b > frame.y + frame.h) cost += 1000;
              else if (inner && (b.l < inner.x || b.r > inner.x + inner.w || b.t < inner.y || b.b > inner.y + inner.h)) cost += 60;
              if (!best || cost < best.cost) best = { cost: cost, c: c, b: b, L: L };
            });
            best.b.w = 1000;          // a placed label: later labels must not cover it unless nothing else is possible
            boxes.push(best.b);
            out.push({ L: best.L, c: best.c, font: font, color: color, b: best.b });
            return best.c;
          },
          flush: function () {
            out.forEach(function (o) { richText(ctx, o.L, o.c.x, o.c.y, { align: o.c.align, color: o.color, font: o.font, bg: LBG }); });
            if (el) el.sosLabels = out.map(function (o) {
              return { text: partsOf(o.L).map(function (q) { return Array.isArray(q) ? q[0] : q; }).join(''), l: o.b.l, r: o.b.r, t: o.b.t, b: o.b.b };
            });
          }
        };
      }

      // ---------------------------------------------------------------- step-response plot
      function yTop(cv) {
        const s = model.specs(st.zeta, st.wn);
        let peak = isFinite(s.Mp) ? 1 + s.Mp : 1;
        if (cv.h) peak = Math.max(peak, cv.h.ymax);
        for (let i = 0; i < YTOPS.length; i++) if (YTOPS[i] >= peak + 0.07) return YTOPS[i];
        return YTOPS[YTOPS.length - 1];
      }
      cvS.draw = function (ctx, w, h) {
        reserveLegend(w);
        const cv = curves(), T = cv.T;
        const s = model.specs(st.zeta, st.wn);
        const band = bandFrac();
        const p = RASim.plot(ctx, { box: { x: 0, y: 0, w: w, h: h }, margin: { left: 46, right: 12, top: 12, bottom: 36 },
          xmin: 0, xmax: T, ymin: 0, ymax: yTop(cv), xlabel: 't (s)', ylabel: 'y(t)' });
        p.axes();
        const rc = p.rect, yb = rc.y + rc.h;
        p.band(1 - band, 1 + band, { fill: COL.band });
        p.hline(1, { color: C.axis, width: 1.2 });
        if (cv.h) p.line(cv.h.ts, cv.h.ys, { color: COL.held, width: 2.2 });
        const under = s.kind === 'under';
        if (under && st.showEnvelope) {
          const sg = st.zeta * st.wn, k = 1 / Math.sqrt((1 - st.zeta) * (1 + st.zeta));
          const es = { color: COL.env, width: 1.5, dash: [6, 4], n: 240 };
          p.fn(function (t) { return 1 + k * Math.exp(-sg * t); }, es);
          p.fn(function (t) { return 1 - k * Math.exp(-sg * t); }, es);
        }
        p.line(cv.y.ts, cv.y.ys, { color: COL.y, width: 2.5 });

        // labels may use the top margin (e.g. "M_p = 100 %" when the peak is close to the top of the plot)
        const lay = placer(ctx, { x: rc.x, y: Math.max(0, rc.y - 10), w: rc.w, h: rc.h + rc.y - Math.max(0, rc.y - 10) }, cvS.canvas, rc);
        // keep labels off the response curve (blocks at most 4 px apart, also on steep segments)
        let bx = -1e9, by = -1e9;
        for (let i = 0; i <= NS; i++) {
          const x = p.X(cv.y.ts[i]), y = p.Y(cv.y.ys[i]);
          if (y < rc.y - 3 || y > yb + 3) { bx = -1e9; continue; }
          const d = Math.hypot(x - bx, y - by);
          if (d < 4) continue;
          if (bx > -1e8) {                    // fill the gap from the previous block
            const k = Math.ceil(d / 4);
            for (let j = 1; j < k; j++) lay.block(bx + (x - bx) * j / k, by + (y - by) * j / k, 3, 3);
          }
          lay.block(x, y, 3, 3); bx = x; by = y;
        }
        if (!st.showMarkers) { lay.flush(); return; }
        const tsA = model.settleActual(st.zeta, st.wn, band);
        const oscill = under || s.kind === 'undamped';
        let placeTs = null;
        if (oscill) {
          // formula settling time
          const tsF = band === 0.05 ? s.ts5 : s.ts2;
          if (isFinite(tsF) && tsF <= T) {
            p.vline(tsF, { color: COL.ts, width: 1.5, dash: [5, 4] });
            const xs = p.X(tsF);
            lay.block(xs, rc.y + rc.h / 2, 2, rc.h / 2, 10);
            const full = ['t', ['s'], ' ≈ ' + (band === 0.05 ? '3' : '4') + '/(ζω', ['n'], ')'];
            const short = ['t', ['s']];
            // placed after the peak labels (they are tied to a point; this one can move or shorten)
            placeTs = function () {
              const sc = rc.w < 300 ? 0 : 20;   // narrow plot: prefer the short label
              lay.put(full, F12, COL.ts, [
                { x: xs + 5, y: rc.y + 11, align: 'left', cost: 20 - sc }, { x: xs - 5, y: rc.y + 11, align: 'right', cost: 20 - sc },
                { x: xs + 5, y: rc.y + 29, align: 'left', cost: 20 - sc }, { x: xs - 5, y: rc.y + 29, align: 'right', cost: 20 - sc },
                { x: xs + 5, y: rc.y + 11, align: 'left', lbl: short, cost: sc }, { x: xs - 5, y: rc.y + 11, align: 'right', lbl: short, cost: sc },
                { x: xs + 5, y: rc.y + 29, align: 'left', lbl: short, cost: sc }, { x: xs - 5, y: rc.y + 29, align: 'right', lbl: short, cost: sc },
                { x: xs + 5, y: rc.y + 0.55 * rc.h, align: 'left', cost: 30 }, { x: xs + 5, y: rc.y + 0.7 * rc.h, align: 'left', cost: 30 },
                { x: xs - 5, y: rc.y + 0.55 * rc.h, align: 'right', cost: 40 }, { x: xs + 5, y: rc.y + 0.55 * rc.h, align: 'left', lbl: short, cost: 40 }]);
            };
          }
          // rise time 0 → 100 %
          if (s.tr <= T) {
            p.segment(s.tr, 0, s.tr, 1, { color: COL.tr, width: 1.5, dash: [4, 3] });
            p.point(s.tr, 1, { color: COL.tr, r: 3.5 });
            lay.block(p.X(s.tr), (p.Y(1) + yb) / 2, 2, (yb - p.Y(1)) / 2);
          }
          // peak
          const yp = 1 + s.Mp;
          if (s.tp <= T) {
            const xp = p.X(s.tp), ypx = p.Y(yp);
            p.segment(s.tp, 0, s.tp, yp, { color: COL.peak, width: 1, dash: [3, 3] });
            p.segment(0, yp, s.tp, yp, { color: COL.peak, width: 1, dash: [3, 3] });
            p.point(s.tp, yp, { color: COL.peak, r: 4.5 });
            lay.block(xp, ypx, 6, 6);
            lay.block(xp, (ypx + yb) / 2, 1, (yb - ypx) / 2);
            lay.block((rc.x + xp) / 2, ypx, (xp - rc.x) / 2, 1);
          }
          if (s.tr <= T) {
            const xr = p.X(s.tr);
            lay.put(['t', ['r']], F12, COL.tr, [
              { x: xr, y: yb - 10, align: 'center' }, { x: xr - 4, y: yb - 10, align: 'right' }, { x: xr + 4, y: yb - 10, align: 'left' },
              { x: xr - 4, y: yb - 28, align: 'right' }, { x: xr + 4, y: yb - 28, align: 'left' },
              { x: xr - 4, y: yb - 46, align: 'right' }, { x: xr + 4, y: yb - 46, align: 'left' }]);
          }
          if (s.tp <= T) {
            const xp = p.X(s.tp), ypx = p.Y(yp);
            lay.put(['t', ['p']], F12, COL.peak, [
              { x: xp, y: yb - 10, align: 'center' }, { x: xp + 4, y: yb - 10, align: 'left' }, { x: xp - 4, y: yb - 10, align: 'right' },
              { x: xp + 4, y: yb - 28, align: 'left' }, { x: xp - 4, y: yb - 28, align: 'right' },
              { x: xp + 4, y: yb - 46, align: 'left' }, { x: xp - 4, y: yb - 46, align: 'right' }]);
            const mpl = ['M', ['p'], ' = ' + fmt(100 * s.Mp, 3) + ' %'];
            lay.put(mpl, B12, COL.peak, [
              { x: xp + 9, y: ypx - 2, align: 'left' }, { x: xp + 9, y: ypx - 6, align: 'left' }, { x: xp + 7, y: ypx - 14, align: 'left' },
              { x: xp - 9, y: ypx - 14, align: 'right' }, { x: xp + 9, y: ypx + 14, align: 'left' }, { x: xp, y: ypx - 16, align: 'center' }]);
          }
          if (placeTs) placeTs();
        } else {
          // ζ ≥ 1: 10 % and 90 % rise markers
          const r = model.riseTimes(st.zeta, st.wn);
          [[r.t10, 0.1, '10 %'], [r.t90, 0.9, '90 %']].forEach(function (q) {
            if (!(q[0] <= T)) return;
            p.segment(q[0], 0, q[0], q[1], { color: COL.tr, width: 1, dash: [3, 3] });
            p.segment(0, q[1], q[0], q[1], { color: COL.tr, width: 1, dash: [3, 3] });
            p.point(q[0], q[1], { color: COL.tr, r: 4 });
            lay.block(p.X(q[0]), p.Y(q[1]), 6, 6);
          });
          [[r.t10, 0.1, '10 %'], [r.t90, 0.9, '90 %']].forEach(function (q) {
            if (!(q[0] <= T)) return;
            const x = p.X(q[0]), y = p.Y(q[1]);
            lay.put(q[2], B12, COL.tr, [{ x: x + 8, y: y + 9, align: 'left' }, { x: x - 8, y: y - 10, align: 'right' },
              { x: x + 8, y: y - 10, align: 'left' }, { x: x - 8, y: y + 9, align: 'right' }]);
          });
        }
        // actual settling time: triangle on the time axis
        if (isFinite(tsA) && tsA <= T) {
          const x = p.X(tsA);
          ctx.save();
          ctx.fillStyle = COL.tsA; ctx.strokeStyle = '#ffffff'; ctx.lineWidth = 1;
          ctx.beginPath(); ctx.moveTo(x, yb - 11); ctx.lineTo(x - 6, yb); ctx.lineTo(x + 6, yb); ctx.closePath();
          ctx.fill(); ctx.stroke();
          ctx.restore();
          lay.block(x, yb - 6, 7, 6);
        }
        lay.flush();
      };

      // ---------------------------------------------------------------- s-plane
      function planeW() {
        if (dragW !== null) return dragW;
        const ps = model.poles(st.zeta, st.wn);
        let r = st.wn;
        if (model.classify(st.zeta) === 'over') r = Math.max(r, Math.abs(ps[1].re));
        return M.niceCeil(1.25 * r);
      }
      function planePlot(ctx, w, h, W) {
        const mg = { left: 44, right: 10, top: 10, bottom: 34 };
        const rw = Math.max(1, w - mg.left - mg.right), rh = Math.max(1, h - mg.top - mg.bottom);
        const xs0 = 1.3 * W, ys0 = 1.84 * W;
        const u = Math.max(xs0 / rw, ys0 / rh);
        const ex = u * rw - xs0, ey = u * rh;
        return RASim.plot(ctx, { box: { x: 0, y: 0, w: w, h: h }, margin: mg, equal: true,
          xmin: -W - 0.72 * ex, xmax: 0.3 * W + 0.28 * ex, ymin: -ey / 2, ymax: ey / 2, xlabel: 'σ', ylabel: 'jω' });
      }
      cvZ.draw = function (ctx, w, h) {
        const W = planeW();
        const p = planePlot(ctx, w, h, W);
        p.axes();
        const rc = p.rect, ox = p.X(0), oy = p.Y(0);
        // right half-plane tint
        ctx.save();
        ctx.fillStyle = RASim.alpha(C.red, 0.07);
        ctx.fillRect(ox, rc.y, rc.x + rc.w - ox, rc.h);
        ctx.restore();
        const lay = placer(ctx, rc, cvZ.canvas);
        const crosses = [];
        const kind = model.classify(st.zeta);
        const ps = model.poles(st.zeta, st.wn), up = ps[0];
        const Rpx = p.X(st.wn) - ox;
        // constant ωₙ: dashed semicircle in the left half-plane
        p.arc(0, 0, Rpx, PI / 2, 3 * PI / 2, { color: RASim.alpha(COL.wn, 0.6), width: 1.3, dash: [5, 4] });
        const oscill = kind === 'under' || kind === 'undamped';
        if (oscill) {
          const th = Math.acos(st.zeta);
          const ux = -Math.cos(th), uy = Math.sin(th);                 // unit direction to the upper pole (data)
          // constant ζ: dashed rays through the poles to the edge of the plot
          const far = 4 * W;
          p.segment(0, 0, ux * far, uy * far, { color: COL.zr, width: 1.3, dash: [6, 4] });
          p.segment(0, 0, ux * far, -uy * far, { color: RASim.alpha(COL.zr, 0.5), width: 1.2, dash: [6, 4] });
          // radius ωₙ to the upper pole
          p.segment(0, 0, up.re, up.im, { color: COL.wn, width: 2 });
          const xu = p.X(up.re), yu = p.Y(up.im);
          // dashed projections
          if (kind === 'under') {
            p.segment(up.re, up.im, up.re, 0, { color: C.muted, width: 1, dash: [3, 3] });
            p.segment(up.re, up.im, 0, up.im, { color: C.muted, width: 1, dash: [3, 3] });
            lay.block(xu, (yu + oy) / 2, 1, (oy - yu) / 2, 10);
            lay.block((xu + ox) / 2, yu, (ox - xu) / 2, 1, 10);
          }
          // θ arc between the negative real axis and the radius
          const ra = M.clamp(0.32 * Rpx, 16, 34);
          p.arc(0, 0, ra, PI - th, PI, { color: COL.zr, width: 1.8 });
          // block the radius and the arc region for labels
          for (let k = 1; k <= 8; k++) lay.block(ox + (xu - ox) * k / 8, oy + (yu - oy) * k / 8, 4, 4);
          for (let k = 0; k <= 6; k++) { const a = PI - th + th * k / 6; lay.block(ox + ra * Math.cos(a), oy - ra * Math.sin(a), 3, 3); }
          crossAt(up.re, up.im, true); crossAt(ps[1].re, ps[1].im, false);
          // labels: the ones tied to a point first, then θ (which may need a leader line to its arc)
          if (kind === 'under') {
            lay.put(['−ζω', ['n']], F12, C.ink, [
              { x: xu, y: oy + 12, align: 'center' }, { x: xu - 3, y: oy + 12, align: 'right' }, { x: xu + 3, y: oy + 12, align: 'left' },
              { x: xu - 12, y: oy + 12, align: 'right', cost: 2 }, { x: xu + 12, y: oy + 12, align: 'left', cost: 2 },
              { x: xu - 12, y: oy - 12, align: 'right', cost: 3 }, { x: xu, y: oy + 30, align: 'center', cost: 4 }]);
            lay.put(['ω', ['d']], F12, C.ink, [
              { x: ox + 5, y: yu, align: 'left' }, { x: ox + 5, y: yu - 12, align: 'left' }, { x: ox + 5, y: yu + 12, align: 'left' }]);
          }
          // ωₙ on the radius, on the side away from the θ wedge
          const mx = (ox + xu) / 2, my = (oy + yu) / 2;
          let nx = -(yu - oy), ny = (xu - ox);
          const nl = Math.hypot(nx, ny) || 1; nx /= nl; ny /= nl;
          if (nx < 0) { nx = -nx; ny = -ny; }
          lay.put(['ω', ['n']], B12, COL.wn, [
            { x: mx + 9 * nx, y: my + 9 * ny, align: 'left' }, { x: mx + 9 * nx, y: my + 9 * ny - 12, align: 'left' },
            { x: mx - 9 * nx, y: my - 9 * ny, align: 'right' }]);
          const thl = 'θ = cos⁻¹ζ = ' + fmt(model.specs(st.zeta, st.wn).thetaDeg, 3) + '°';
          const mA = PI - th / 2;
          const amx = ox + ra * Math.cos(mA), amy = oy - ra * Math.sin(mA);     // middle of the arc
          const ax = ox + (ra + 6) * Math.cos(mA), ay = oy - (ra + 6) * Math.sin(mA);
          const cands = [{ x: ax, y: Math.min(ay, oy - 12), align: 'right' }];
          if (kind === 'under') cands.push({ x: Math.min(xu, ox - ra) - 8, y: oy - 13, align: 'right', leader: true, cost: 1 });
          cands.push({ x: ox - 4, y: oy + 13, align: 'right', cost: 2 });
          cands.push({ x: ox - 4, y: oy + 31, align: 'right', leader: true, cost: 2.5 });
          if (kind === 'under') cands.push({ x: Math.min(xu, ox - ra) - 22, y: oy + 13, align: 'right', leader: true, cost: 2 });
          cands.push({ x: ox - 6, y: oy - ra - 22, align: 'right', leader: true, cost: 3 });
          const ct = lay.put(thl, B12, COL.zr, cands);
          if (ct.leader) {
            ctx.save(); ctx.strokeStyle = COL.zr; ctx.lineWidth = 1;
            ctx.beginPath(); ctx.moveTo(ct.x + 2, ct.y); ctx.lineTo(amx, amy); ctx.stroke();
            ctx.restore();
          }
        } else {
          crossAt(ps[0].re, 0, true); crossAt(ps[1].re, 0, false);
          const x1 = p.X(ps[0].re), x2 = p.X(ps[1].re);
          if (kind === 'critical') {
            lay.put('double pole', B12, COL.pole, [{ x: x1, y: oy - 21, align: 'center' }, { x: x1, y: oy + 21, align: 'center' }]);
            lay.put(['−ω', ['n']], F12, C.ink, [{ x: x1, y: oy + 21, align: 'center' }, { x: x1, y: oy - 38, align: 'center' }]);
          } else {
            lay.put('s₁', B12, COL.pole, [{ x: x1, y: oy - 21, align: 'center' }, { x: x1, y: oy + 21, align: 'center' }]);
            lay.put('s₂', B12, COL.pole, [{ x: x2, y: oy - 18, align: 'center' }, { x: x2, y: oy + 18, align: 'center' }]);
          }
          lay.put(['|s| = ω', ['n']], F12, COL.wn, [
            { x: ox - Rpx * 0.71 - 4, y: oy - Rpx * 0.71 - 8, align: 'right' }, { x: ox - Rpx * 0.71 - 4, y: oy + Rpx * 0.71 + 8, align: 'right' },
            { x: ox + 4, y: oy - Rpx - 9, align: 'left' }]);
        }
        // "unstable" in the right half-plane
        const rhpW = rc.x + rc.w - ox;
        lay.put('unstable', F11, C.red, [
          { x: ox + rhpW / 2, y: rc.y + rc.h - 11, align: 'center' }, { x: ox + rhpW / 2, y: rc.y + 11, align: 'center' }]);
        lay.flush();
        // the poles are drawn again on top, so no label background can ever hide a pole
        crosses.forEach(function (c) { p.cross(c[0], c[1], { color: COL.pole, size: 6.5, width: 2.6 }); });
        cvZ.canvas.sosPoles = crosses.map(function (c) { return { x: p.X(c[0]), y: p.Y(c[1]) }; });   // for the browser tests

        function crossAt(re, im, grab) {
          if (grab) p.point(re, im, { r: 11, color: RASim.alpha(COL.pole, 0.13), stroke: null });
          p.cross(re, im, { color: COL.pole, size: 6.5, width: 2.6 });
          crosses.push([re, im]);
          lay.block(p.X(re), p.Y(im), grab ? 11 : 8, grab ? 11 : 8);
        }
      };
      function upperPolePx(p) {
        const up = model.poles(st.zeta, st.wn)[0];
        return { x: p.X(up.re), y: p.Y(up.im) };
      }
      cvZ.onDrag({
        hit: function (x, y) {
          const p = planePlot(cvZ.ctx, cvZ.width, cvZ.height, planeW());
          const q = upperPolePx(p);
          return Math.hypot(q.x - x, q.y - y) <= 16 ? { W: planeW() } : null;
        },
        move: function (x, y, tok) {
          dragW = tok.W;
          const p = planePlot(cvZ.ctx, cvZ.width, cvZ.height, tok.W);
          const re = Math.min(0, p.invX(x)), im = Math.max(0, p.invY(y));
          const r = model.fromPole(re, im);
          sZ.set(M.clamp(r.zeta, ZMIN, ZMAX)); st.zeta = sZ.get();
          sW.set(M.clamp(r.wn, WMIN, WMAX)); st.wn = sW.get();
          update();
        },
        end: function () { dragW = null; cvZ.redraw(); },
        cursor: function (x, y) {
          const p = planePlot(cvZ.ctx, cvZ.width, cvZ.height, planeW());
          const q = upperPolePx(p);
          return Math.hypot(q.x - x, q.y - y) <= 16 ? 'grab' : null;
        }
      });

      // ---------------------------------------------------------------- read-outs
      const KIND_TAG = { undamped: ['UNDAMPED', 'bad'], under: ['UNDERDAMPED', 'warn'], critical: ['CRITICALLY DAMPED', 'ok'], over: ['OVERDAMPED', ''] };
      function sec(v) { return isFinite(v) ? fmt(v, 4) + ' s' : fmt(v, 4); }
      function rad(v) { return isFinite(v) ? fmt(v, 4) + ' rad/s' : fmt(v, 4); }
      let legendKey = '';
      function update() {
        const z = st.zeta, wn = st.wn, band = bandFrac();
        const s = model.specs(z, wn);
        const kt = KIND_TAG[s.kind];
        tag.set(kt[0], kt[1]);
        const den = RASim.fmtPoly([1, 2 * z * wn, wn * wn], 's', 4).replace('s<sup>2</sup>', 's²');
        tfLine.innerHTML = 'G(s) = ' + fmt(wn * wn, 4) + ' / (' + den + ')';

        const ps = model.poles(z, wn);
        if (s.kind === 'under' || s.kind === 'undamped') {
          tPo.set((s.kind === 'undamped' ? '' : fmt(ps[0].re, 4) + ' ') + '± j' + fmt(ps[0].im, 4));
        } else if (s.kind === 'critical') tPo.set(fmt(ps[0].re, 4) + ' (double)');
        else tPo.set(fmt(ps[0].re, 4) + ', ' + fmt(ps[1].re, 4));
        tWd.set(rad(s.wd));
        tTh.set(isFinite(s.thetaDeg) ? fmt(s.thetaDeg, 4) + '°' : '—');
        tTr.set(sec(s.tr));
        tTp.set(sec(s.tp));
        tMp.set(fmt(100 * s.Mp, 4) + ' %');
        tMp.setState(s.Mp > 0.25 ? 'bad' : s.Mp > 0.05 ? 'warn' : 'ok');
        const tsF = band === 0.05 ? s.ts5 : s.ts2;
        tsLabel.innerHTML = lab('t<sub>s</sub> ≈ ' + (band === 0.05 ? '3' : '4') + '/(ζω<sub>n</sub>), ' + fmt(100 * band, 2) + ' %');
        tsALabel.innerHTML = lab('t<sub>s</sub> actual, ' + fmt(100 * band, 2) + ' %');
        tTs.set(sec(tsF));
        tTsA.set(sec(model.settleActual(z, wn, band)));
        tN.set(fmt(model.oscillations(z, wn, band), 3));
        tR19.set(sec(model.rise1090(z, wn)));
        tMr.set(fmt(s.Mr, 4));
        tWr.set(rad(s.wr));
        tWb.set(rad(s.wb));

        bClear.disabled = !held;
        const items = legendItems(z, wn, held, s.kind, st.showEnvelope, st.showMarkers, band);
        const lk = JSON.stringify(items);
        if (lk !== legendKey) { legend.set(items); legendKey = lk; legendNow = items; }
        cvS.redraw(); cvZ.redraw();
      }
      function legendItems(z, wn, hd, kind, env, markers, band) {
        const items = [{ color: COL.y, label: 'y(t): ζ = ' + fmt(z, 3) + ', ' + WN + ' = ' + fmt(wn, 3) + ' rad/s' }];
        if (hd) items.push({ color: COL.held, label: 'held: ζ = ' + fmt(hd.zeta, 3) + ', ' + WN + ' = ' + fmt(hd.wn, 3) + ' rad/s' });
        if (kind === 'under' && env) items.push({ color: COL.env, dash: true, label: 'envelope 1 ± e<sup>−ζω<sub>n</sub>t</sup>/√(1 − ζ²)' });
        items.push({ color: RASim.alpha(C.green, 0.45), label: '±' + fmt(100 * band, 2) + ' % band' });
        if (markers) {
          if (kind === 'under' || kind === 'undamped') {
            items.push({ color: COL.tr, dash: true, label: 't<sub>r</sub> (0 → 100 %)' });
            items.push({ color: COL.peak, dash: true, label: 'peak (t<sub>p</sub>, 1 + M<sub>p</sub>)' });
            if (kind === 'under') items.push({ color: COL.ts, dash: true, label: 't<sub>s</sub> formula' });
          } else items.push({ color: COL.tr, dash: true, label: '10 % and 90 % points' });
          if (kind !== 'undamped') items.push({ color: COL.tsA, label: '▲ actual settling time' });
        }
        return items;
      }
      // The legend sits between the two canvases. Its height is reserved for the longest legend at the current
      // width, so the s-plane does not jump up or down (under the pointer) when the legend gains or loses a row.
      let legendNow = [], legendW = -1;
      function reserveLegend(w) {
        if (w === legendW || !(w > 0)) return;
        legendW = w;
        legend.el.style.minHeight = '';
        legend.set(legendItems(0.555, 5.55, { zeta: 0.555, wn: 5.55 }, 'under', true, true, 0.02));
        const hgt = legend.el.offsetHeight;
        legend.set(legendNow);
        legend.el.style.minHeight = hgt + 'px';
      }

      function getState() {
        return { zeta: st.zeta, wn: st.wn, band: st.band, showEnvelope: st.showEnvelope, showMarkers: st.showMarkers, held: !!held };
      }
      function setState(s) {
        s = s || {};
        if (s.zeta != null) { sZ.set(Number(s.zeta)); st.zeta = sZ.get(); }
        if (s.wn != null) { sW.set(Number(s.wn)); st.wn = sW.get(); }
        if (s.band != null) { st.band = String(s.band) === '5' ? '5' : '2'; segBand.set(st.band); }
        if (s.showEnvelope != null) { st.showEnvelope = !!s.showEnvelope; cEnv.set(st.showEnvelope); }
        if (s.showMarkers != null) { st.showMarkers = !!s.showMarkers; cMk.set(st.showMarkers); }
        if (s.held != null) { if (!s.held) held = null; else if (!held) hold(); }
        update();
      }

      update();
      return { getState: getState, setState: setState };
    }
  });
})();
