// pid-tuning — "PID tuning lab"
// Sits beside notes/src/04_electrical.html, section "P, PI, PID controllers" (end of "B1.3 Control Systems");
// it also uses the steady-state error table of the same part.
//
// SIGN CONVENTIONS AND MODEL
//   Unity NEGATIVE feedback: e = r − y. r is a unit step at t = 0. Polynomials: highest power of s first.
//   Controller C(s) = K_p + K_i/s + K_d·s/(τ_f·s + 1), τ_f = TAU_F = 0.02 s (filtered derivative, so C is proper).
//   C = N_c/D_c in its simplest exact form (no artificial pole–zero pairs):
//     K_i = 0, K_d = 0: N_c = [K_p], D_c = [1]          K_i = 0, K_d ≠ 0: N_c = [K_p·τ_f + K_d, K_p], D_c = [τ_f, 1]
//     K_i ≠ 0, K_d = 0: N_c = [K_p, K_i], D_c = [1, 0]   otherwise: N_c = [K_p·τ_f + K_d, K_p + K_i·τ_f, K_i], D_c = [τ_f, 1, 0]
//   Plant G = N_p/D_p: first 1/(s + 1), second 2/((s + 1)(s + 2)), third 1/(s + 1)³ (K_u = 8, ω_u = √3 rad/s,
//   P_u = 2π/√3 s), motor 1/(s(s + 1)).
//   Δ = D_c·D_p + N_c·N_p.  T = N_c·N_p/Δ (r → y), U = N_c·D_p/Δ (r → u), Y_d = N_p·D_c/Δ (d → y),
//   U_d = −N_c·N_p/Δ (d → u). The load disturbance d is a step of size d₀ ADDED AT THE PLANT INPUT
//   (plant input = u + d) at t_d = tEnd/2; y = y_r + d₀·y_d(t − t_d), u = u_r + d₀·u_d(t − t_d) (superposition).
//   Stability from the roots of Δ: max Re < −1e-6 stable, |max Re| ≤ 1e-6 marginal, otherwise unstable.
//   e_ss = 1 − T(0) = D_c(0)·D_p(0)/Δ(0) for a unit step (stable loops only, else NaN); y_∞ = T(0).
//   Loop type = (1 if K_i ≠ 0) + (1 for the motor plant).
//   Metrics of the REFERENCE response y_r (no disturbance) relative to y_∞ (stable, y_∞ > 0): overshoot % =
//   (max y − y_∞)/y_∞·100 (0 when negative); peak time (NaN when there is no overshoot); 10–90 % rise time;
//   2 % settling time = last time |y − y_∞| > 0.02·y_∞; Infinity (shown "> window") when not settled / the 90 % level
//   not reached by tEnd. metrics(t, y, yFinal) works on samples (linear interpolation; the peak is refined by a
//   cross-checked parabola that never invents overshoot). The panel shows refMetrics(), i.e. metrics() on samples
//   that resolve the response (fine sub-steps where fast poles act, exact local refinement of the peak, the
//   10 %/90 % crossings and the last band exit), because a fast filtered-derivative spike can fall between samples.
//   loop().poles are sorted closest to the jω-axis first.
//   Ziegler–Nichols (ultimate gain): P: K_p = 0.5K_u; PI: K_p = 0.45K_u, T_i = P_u/1.2; PID: K_p = 0.6K_u,
//   T_i = P_u/2, T_d = P_u/8; K_i = K_p/T_i, K_d = K_p·T_d.
//
// STATE KEYS (getState / setState)
//   plant ('first' | 'second' | 'third' | 'motor'), Kp, Ki, Kd, tEnd (s), d0, showU (boolean),
//   held (boolean; setState({ held: true }) holds the current curve if none is held, false clears it;
//   applied LAST, so setState({ Kp: 4, held: true }) holds the new curve).
//
// STAT KEYS:   ess, type, overshoot, tPeak, tRise, tSettle, poles, Ku, Pu, distOffset
// TAG KEYS:    stability   (STABLE / MARGINALLY STABLE / UNSTABLE)
// MATRIX KEYS: (none)
// DATA-KEY:    ctrl (controller line "C(s) = 4.8 + 2.646/s + 2.177 s"), zn (Ziegler–Nichols note)
// CANVAS KEYS: diagram, output, control
// CONTROL KEYS: plant, Kp, Ki, Kd, quickP, quickPI, quickPD, quickPID, setKu, znP, znPI, znPID,
//               tEnd, d0, showU, hold, clearHeld, reset
(function () {
  'use strict';
  const RASim = globalThis.RASim;
  const M = RASim.math;
  const ID = 'pid-tuning';
  const TAU_F = 0.02;
  const STAB_TOL = 1e-6;

  // ---------------------------------------------------------------- model (pure)
  const plants = {
    first: { label: '1/(s + 1)', num: [1], den: [1, 1], Ku: null, Pu: null },
    second: { label: '2/((s + 1)(s + 2))', num: [2], den: [1, 3, 2], Ku: null, Pu: null },
    third: { label: '1/(s + 1)³', num: [1], den: [1, 3, 3, 1], Ku: 8, Pu: 2 * Math.PI / Math.sqrt(3) },
    motor: { label: '1/(s(s + 1))', num: [1], den: [1, 1, 0], Ku: null, Pu: null }
  };

  function controller(Kp, Ki, Kd) {
    if (Ki === 0 && Kd === 0) return { num: [Kp], den: [1] };
    if (Ki === 0) return { num: [Kp * TAU_F + Kd, Kp], den: [TAU_F, 1] };
    if (Kd === 0) return { num: [Kp, Ki], den: [1, 0] };
    return { num: [Kp * TAU_F + Kd, Kp + Ki * TAU_F, Ki], den: [TAU_F, 1, 0] };
  }

  function at0(p) { return p[p.length - 1]; }

  function loop(plantId, Kp, Ki, Kd) {
    const P = plants[plantId];
    if (!P) throw new Error('pid-tuning: unknown plant ' + plantId);
    const c = controller(Kp, Ki, Kd);
    const NcNp = M.polyMul(c.num, P.num), DcDp = M.polyMul(c.den, P.den);
    const den = M.polyAdd(DcDp, NcNp);
    // closest to the jω-axis (smallest |Re|) first, as the panel lists them; conjugate pairs stay adjacent
    const poles = M.polyRoots(den).sort(function (p, q) { return Math.abs(p.re) - Math.abs(q.re) || q.im - p.im; });
    let maxRe = -Infinity;
    poles.forEach(function (q) { if (q.re > maxRe) maxRe = q.re; });
    const stability = maxRe < -STAB_TOL ? 'stable' : maxRe <= STAB_TOL ? 'marginal' : 'unstable';
    const stable = stability === 'stable';
    const d0 = at0(den);
    return {
      T: { num: M.polyTrim(NcNp), den: den },
      U: { num: M.polyTrim(M.polyMul(c.num, P.den)), den: den },
      Yd: { num: M.polyTrim(M.polyMul(P.num, c.den)), den: den },
      Ud: { num: M.polyTrim(M.polyScale(NcNp, -1)), den: den },
      charPoly: den,
      poles: poles,
      stability: stability,
      type: (Ki !== 0 ? 1 : 0) + (plantId === 'motor' ? 1 : 0),
      ess: stable ? at0(DcDp) / d0 : NaN,
      yFinal: stable ? at0(NcNp) / d0 : NaN,
      distOffsetPerUnit: stable ? at0(P.num) * at0(c.den) / d0 : NaN
    };
  }

  // response of tf to a unit step applied at t_d = tEnd/2, sampled on the grid t_k = k·tEnd/n (k = 0…n)
  function delayedStep(tf, tEnd, n) {
    const out = new Array(n + 1).fill(0);
    const half = tEnd / 2;
    if (n % 2 === 0) {
      const k0 = n / 2, r = M.tfStep(tf.num, tf.den, half, k0);
      for (let j = 0; j <= k0; j++) out[k0 + j] = r.y[j];
    } else {
      // t_d lies half-way between two samples: a grid of spacing dt/2 hits every later sample at an odd index
      const k0 = (n + 1) / 2, r = M.tfStep(tf.num, tf.den, half, n);
      for (let j = 0; k0 + j <= n; j++) out[k0 + j] = r.y[2 * j + 1];
    }
    return out;
  }

  function simulate(plantId, Kp, Ki, Kd, opts) {
    opts = opts || {};
    const tEnd = opts.tEnd != null ? opts.tEnd : 15;
    const n = opts.n != null ? Math.max(1, Math.round(opts.n)) : 600;
    const d0 = opts.d0 != null ? opts.d0 : 0;
    const L = loop(plantId, Kp, Ki, Kd);
    const yr = M.tfStep(L.T.num, L.T.den, tEnd, n);
    const ur = M.tfStep(L.U.num, L.U.den, tEnd, n);
    const y = yr.y.slice(), u = ur.y.slice();
    if (d0 !== 0) {
      const yd = delayedStep(L.Yd, tEnd, n), ud = delayedStep(L.Ud, tEnd, n);
      for (let k = 0; k <= n; k++) { y[k] += d0 * yd[k]; u[k] += d0 * ud[k]; }
    }
    return { t: yr.t, y: y, u: u, yRef: yr.y };
  }

  // maximum over [lo, hi] of the parabola through samples i, i + 1, i + 2 (any spacing); null when it is not concave
  function parabolaMax(t, y, i, lo, hi) {
    const t0 = t[i], t1 = t[i + 1], t2 = t[i + 2];
    const f01 = (y[i + 1] - y[i]) / (t1 - t0), f12 = (y[i + 2] - y[i + 1]) / (t2 - t1), f2 = (f12 - f01) / (t2 - t0);
    if (!(f2 < 0)) return null;
    const ts = Math.min(hi, Math.max(lo, 0.5 * (t0 + t1) - f01 / (2 * f2)));
    return { t: ts, y: y[i] + f01 * (ts - t0) + f2 * (ts - t0) * (ts - t1) };
  }

  // metrics of SAMPLED data (any increasing t). The peak is refined by the parabola through the 3 samples around
  // the largest one, cross-checked with the parabola one sample further out on the side of the vertex: a smooth
  // peak gives nearly the same value, a kink (e.g. a fast derivative-kick spike right after t = 0) does not, and the
  // smaller value is kept, so the refinement never invents overshoot. refMetrics() below supplies samples that
  // resolve the response; that is what the panel shows.
  function metrics(t, y, yFinal) {
    const out = { overshootPct: NaN, tPeak: NaN, tRise: NaN, tSettle: NaN };
    const n = y.length - 1;
    if (!(yFinal > 0) || !isFinite(yFinal) || n < 1) return out;
    // peak
    let im = 0;
    for (let k = 1; k <= n; k++) if (y[k] > y[im]) im = k;
    let yPk = y[im], tPk = t[im];
    if (im > 0 && im < n) {
      const v = parabolaMax(t, y, im - 1, t[im - 1], t[im + 1]);
      if (v && v.y > yPk) {
        let best = v;
        const w = v.t >= t[im] ? (im + 2 <= n ? parabolaMax(t, y, im, t[im], t[im + 1]) : null)
          : (im >= 2 ? parabolaMax(t, y, im - 2, t[im - 1], t[im]) : null);
        if (v.t >= t[im] ? im + 2 <= n : im >= 2) best = w && w.y < v.y ? w : (w ? v : { t: tPk, y: yPk });
        if (best.y > yPk) { yPk = best.y; tPk = best.t; }
      }
    }
    const os = (yPk - yFinal) / yFinal * 100;
    if (os > 1e-6) { out.overshootPct = os; out.tPeak = tPk; } else out.overshootPct = 0;
    // 10–90 % rise time (first crossings, linear interpolation)
    function firstCross(level) {
      if (y[0] >= level) return t[0];
      for (let k = 1; k <= n; k++) {
        if (y[k] >= level) return t[k - 1] + (level - y[k - 1]) / (y[k] - y[k - 1]) * (t[k] - t[k - 1]);
      }
      return Infinity;
    }
    const t10 = firstCross(0.1 * yFinal), t90 = firstCross(0.9 * yFinal);
    out.tRise = isFinite(t90) ? t90 - t10 : Infinity;
    // 2 % settling time
    const band = 0.02 * yFinal;
    let last = -1;
    for (let k = n; k >= 0; k--) if (Math.abs(y[k] - yFinal) > band) { last = k; break; }
    if (last < 0) out.tSettle = t[0];
    else if (last === n) out.tSettle = Infinity;
    else {
      const e0 = Math.abs(y[last] - yFinal), e1 = Math.abs(y[last + 1] - yFinal);
      out.tSettle = t[last] + (e0 - band) / (e0 - e1) * (t[last + 1] - t[last]);
    }
    return out;
  }

  // ---- exact evaluation of a step response between grid points: controllable canonical state space,
  //      augmented with the constant input (z = [x; 1]) so that z(t + h) = expm([[A, B], [0, 0]]·h)·z(t)
  function ssOf(tf) {
    const nb = M.polyTrim(tf.num), na = M.polyTrim(tf.den), N = na.length - 1, a0 = na[0];
    const a = na.map(function (v) { return v / a0; });
    const b = new Array(N + 1).fill(0);
    nb.forEach(function (v, i) { b[N + 1 - nb.length + i] = v / a0; });
    const Dff = b[0], Mx = [];
    for (let i = 0; i <= N; i++) Mx.push(new Array(N + 1).fill(0));
    for (let i = 0; i < N - 1; i++) Mx[i][i + 1] = 1;
    for (let j = 0; j < N; j++) Mx[N - 1][j] = -a[N - j];
    Mx[N - 1][N] = 1;
    const Cv = [];
    for (let j = 0; j < N; j++) Cv.push(b[N - j] - Dff * a[N - j]);
    return { N: N, Mx: Mx, Cv: Cv, D: Dff };
  }
  function ssProp(ss, h) { return M.expm(ss.Mx.map(function (r) { return r.map(function (v) { return v * h; }); })); }
  function ssAdv(E, x) {
    const N = x.length, nx = new Array(N);
    for (let r = 0; r < N; r++) { let s = E[r][N]; for (let c = 0; c < N; c++) s += E[r][c] * x[c]; nx[r] = s; }
    return nx;
  }
  function ssY(ss, x) { let s = ss.D; for (let c = 0; c < ss.N; c++) s += ss.Cv[c] * x[c]; return s; }

  // metrics of the reference response y_r over [0, tEnd], computed by metrics() on samples that resolve it:
  // the n-interval grid of simulate(), sub-sampled where fast closed-loop poles act (|p|·h ≤ 0.25), plus two levels
  // of 64× local refinement (exact state propagation) around the largest local maxima, the 10 % and 90 % crossings
  // and the last exit from the 2 % band. Sampling alone can miss a fast peak or a crossing completely.
  function refMetrics(plantId, Kp, Ki, Kd, tEnd, n) {
    tEnd = tEnd != null ? tEnd : 15;
    n = n != null ? Math.max(1, Math.round(n)) : 600;
    const L = loop(plantId, Kp, Ki, Kd), yF = L.yFinal;
    if (L.stability !== 'stable' || !(yF > 0)) return metrics([0, tEnd], [0, 0], NaN);
    const ss = ssOf(L.T), N = ss.N, dt = tEnd / n;
    // each pole with |p|·dt > 0.25 asks for 2^j sub-steps per interval (|p|·h ≤ 0.25) while its mode lasts (15/|Re p|)
    const need = [];
    L.poles.forEach(function (p) {
      const a = Math.hypot(p.re, p.im);
      if (a * dt <= 0.25) return;
      let mp = 2; while (a * dt / mp > 0.25 && mp < 1024) mp *= 2;
      need.push({ m: mp, K: Math.min(n, Math.ceil(15 / -p.re / dt)) });
    });
    function subOf(k) { let mk = 1; need.forEach(function (q) { if (k < q.K && q.m > mk) mk = q.m; }); return mk; }
    let total = 0;
    for (let k = 0; k < n; k++) total += subOf(k) - 1;
    while (total > 40000) {               // keep the cost bounded (never reached for the slider ranges)
      need.forEach(function (q) { q.m = Math.max(1, q.m / 2); });
      total = 0; for (let k = 0; k < n; k++) total += subOf(k) - 1;
    }
    const E = ssProp(ss, dt), Es = {};
    const T = [0], X = [new Array(N).fill(0)], Y = [ssY(ss, X[0])];
    let x = X[0];
    for (let k = 0; k < n; k++) {
      const mk = subOf(k);
      if (mk > 1) {
        const Ef = Es[mk] || (Es[mk] = ssProp(ss, dt / mk));
        let xf = x;
        for (let j = 1; j < mk; j++) { xf = ssAdv(Ef, xf); T.push(k * dt + j * dt / mk); X.push(xf); Y.push(ssY(ss, xf)); }
      }
      x = ssAdv(E, x);
      T.push(k + 1 === n ? tEnd : (k + 1) * dt); X.push(x); Y.push(ssY(ss, x));
    }
    let extra = [];
    // two levels of 64 sub-steps between samples i and j; pick(ts, ys) returns the sub-bracket for level 2
    function refine(i, j, pick) {
      let ta = T[i], xa = X[i], span = T[j] - T[i];
      for (let lev = 0; lev < 2 && span > 0; lev++) {
        const h = span / 64, Eh = ssProp(ss, h), ts = [ta], ys = [ssY(ss, xa)], xs = [xa];
        let xx = xa;
        for (let q = 1; q <= 64; q++) { xx = ssAdv(Eh, xx); ts.push(ta + q * h); xs.push(xx); ys.push(ssY(ss, xx)); }
        for (let q = 1; q < 64; q++) extra.push([ts[q], ys[q], xs[q]]);
        const br = pick(ts, ys) || [0, 64];
        ta = ts[br[0]]; xa = xs[br[0]]; span = ts[br[1]] - ts[br[0]];
      }
    }
    function peakOf(g) {
      return function (ts, ys) {
        let q = 0; for (let r = 1; r <= 64; r++) if (g(ys[r]) > g(ys[q])) q = r;
        return [Math.max(0, q - 1), Math.min(64, q + 1)];
      };
    }
    function merge() {
      const all = [];
      for (let i = 0; i < T.length; i++) all.push([T[i], Y[i], X[i]]);
      extra.forEach(function (e) { if (e[0] > 0 && e[0] < tEnd) all.push(e); });
      all.sort(function (p, q) { return p[0] - q[0]; });
      T.length = 0; Y.length = 0; X.length = 0;
      all.forEach(function (e) { T.push(e[0]); Y.push(e[1]); X.push(e[2]); });
      extra = [];
    }
    // local maxima of g(y) on samples lo..hi whose value lies in [0.9·thr, thr): the curve may cross thr
    // between the samples (a near touch); refine at most `max` of them, latest first when `fromEnd`
    function nearTouch(g, thr, lo, hi, max, fromEnd) {
      const idx = [];
      for (let i = Math.max(1, lo); i <= Math.min(T.length - 2, hi); i++) {
        const v = g(Y[i]);
        if (v >= g(Y[i - 1]) && v >= g(Y[i + 1]) && v >= 0.9 * thr && v < thr) idx.push(i);
      }
      (fromEnd ? idx.reverse() : idx).slice(0, max).forEach(function (i) { refine(i - 1, i + 1, peakOf(g)); });
    }
    const id = function (v) { return v; }, dev = function (v) { return Math.abs(v - yF); };
    const band = 0.02 * yF;
    function firstAt(lv) { for (let k = 1; k < T.length; k++) if (Y[k] >= lv) return k; return T.length; }
    function lastOut() { for (let k = T.length - 1; k >= 0; k--) if (dev(Y[k]) > band) return k; return -1; }
    // pass 1: the largest local maxima (the global one may hide between samples next to a slightly smaller
    // one) and near touches of the 10 % / 90 % levels and of the 2 % band
    const peaks = [];
    for (let i = 1; i < T.length - 1; i++) if (Y[i] >= Y[i - 1] && Y[i] >= Y[i + 1]) peaks.push(i);
    peaks.sort(function (p, q) { return Y[q] - Y[p]; });
    peaks.slice(0, 3).forEach(function (i) { refine(i - 1, i + 1, peakOf(id)); });
    nearTouch(id, 0.1 * yF, 1, firstAt(0.1 * yF) - 1, 3, false);
    nearTouch(id, 0.9 * yF, 1, firstAt(0.9 * yF) - 1, 3, false);
    nearTouch(dev, band, lastOut() + 1, T.length - 2, 3, true);
    merge();
    // pass 2: the first crossings of 10 % and 90 % and the last exit from the band
    [0.1, 0.9].forEach(function (f) {
      const lv = f * yF, k = firstAt(lv);
      if (k < T.length && Y[k - 1] < lv) refine(k - 1, k, function (ts, ys) { for (let q = 1; q <= 64; q++) if (ys[q] >= lv) return [q - 1, q]; return null; });
    });
    const ko = lastOut();
    if (ko >= 0 && ko < T.length - 1) refine(ko, ko + 1, function (ts, ys) { for (let q = 63; q >= 0; q--) if (dev(ys[q]) > band) return [q, q + 1]; return null; });
    merge();
    return metrics(T, Y, yF);
  }

  function znDetail(Ku, Pu, kind) {
    if (kind === 'P') return { Kp: 0.5 * Ku, Ti: Infinity, Td: 0, Ki: 0, Kd: 0 };
    if (kind === 'PI') { const Kp = 0.45 * Ku, Ti = Pu / 1.2; return { Kp: Kp, Ti: Ti, Td: 0, Ki: Kp / Ti, Kd: 0 }; }
    if (kind === 'PID') { const Kp = 0.6 * Ku, Ti = Pu / 2, Td = Pu / 8; return { Kp: Kp, Ti: Ti, Td: Td, Ki: Kp / Ti, Kd: Kp * Td }; }
    throw new Error('pid-tuning: unknown Z–N kind ' + kind);
  }
  function zn(Ku, Pu, kind) { const d = znDetail(Ku, Pu, kind); return { Kp: d.Kp, Ki: d.Ki, Kd: d.Kd }; }

  const model = { TAU_F: TAU_F, plants: plants, controller: controller, loop: loop, simulate: simulate,
    metrics: metrics, refMetrics: refMetrics, zn: zn, znDetail: znDetail };

  // ---------------------------------------------------------------- UI constants
  const DEFAULTS = { plant: 'third', Kp: 1, Ki: 0, Kd: 0, tEnd: 15, d0: 0, showU: false };
  const PLANT_IDS = ['first', 'second', 'third', 'motor'];
  const PLANT_OPTS = [
    { value: 'first', label: 'G(s) = 1/(s + 1)  — first order' },
    { value: 'second', label: 'G(s) = 2/((s + 1)(s + 2))  — second order' },
    { value: 'third', label: 'G(s) = 1/(s + 1)³  — third order' },
    { value: 'motor', label: 'G(s) = 1/(s(s + 1))  — motor (position)' }
  ];
  const NS = 600;
  const UMAX = 20;

  RASim.register(ID, {
    title: 'PID tuning lab',
    model: model,
    mount: function (api) {
      const ui = RASim.ui, C = RASim.colors, D = RASim.draw;
      const fmt = RASim.fmt;
      const SC = '.sim[data-sim="' + ID + '"]';
      const st = Object.assign({}, DEFAULTS);
      let held = null;                   // { plant, Kp, Ki, Kd, d0 } of the held curve
      const F11 = '11px sans-serif', F12 = '12px sans-serif', B12 = 'bold 12px sans-serif';
      const LBG = RASim.alpha('#ffffff', 0.88);
      const COL = { y: C.blue, u: C.orange, held: C.faint, ref: C.ink, band: RASim.alpha(C.green, 0.15),
        peak: C.maroon, ts: C.teal, dist: C.purple };

      RASim.css(ID,
        SC + ' .pid-nt { text-transform: none; letter-spacing: 0; font-size: 11.5px; }\n' +
        SC + ' .pid-wide { grid-column: 1 / -1; }\n' +
        SC + ' .pid-stats { grid-template-columns: repeat(2, minmax(0, 1fr)); }\n' +
        SC + ' .pid-ctrl { font-size: 13.5px; color: ' + C.ink + '; margin: 2px 0 8px; font-variant-numeric: tabular-nums; overflow-wrap: anywhere; }\n' +
        SC + ' .pid-btns { gap: 6px 6px; }\n' +
        SC + ' .pid-btns .sim-btn { min-width: 0; padding-left: 9px; padding-right: 9px; }\n' +
        SC + ' .pid-zn { margin: 6px 0 0; line-height: 1.45; }\n' +
        SC + ' .pid-cap { margin-top: 2px; }\n');
      function lab(html) { return '<span class="pid-nt">' + html + '</span>'; }

      ui.note(api.root, 'Change K<sub>p</sub>, K<sub>i</sub> and K<sub>d</sub> (or press a preset) and watch how the speed, the overshoot, the steady-state error and the stability of the closed-loop step response change.');
      const sp = ui.split(api.root, { ratio: 1.5 });

      // ---- main column ----
      const cvB = RASim.canvas(sp.main, { aspect: 5, minHeight: 100, maxHeight: 104, key: 'diagram' });
      const cvY = RASim.canvas(sp.main, { aspect: 1.45, minHeight: 240, maxHeight: 380, key: 'output' });
      const cvU = RASim.canvas(sp.main, { aspect: 3, minHeight: 140, maxHeight: 190, key: 'control' });
      const legend = ui.legend(sp.main, []);

      // ---- side column ----
      const gP = ui.group(sp.side, 'Plant');
      const selPlant = ui.select(gP, { label: 'Plant G(s)', options: PLANT_OPTS, value: st.plant, key: 'plant',
        onChange: function (v) { if (plants[v]) st.plant = v; update(); } });

      const gG = ui.group(sp.side, 'Controller gains');
      const sKp = ui.slider(gG, { label: 'Proportional K<sub>p</sub>', min: 0, max: 20, step: 0.05, value: st.Kp, key: 'Kp',
        onInput: function (v) { st.Kp = v; update(); } });
      const sKi = ui.slider(gG, { label: 'Integral K<sub>i</sub>', min: 0, max: 20, step: 0.05, value: st.Ki, unit: 's⁻¹', key: 'Ki',
        onInput: function (v) { st.Ki = v; update(); } });
      const sKd = ui.slider(gG, { label: 'Derivative K<sub>d</sub>', min: 0, max: 10, step: 0.05, value: st.Kd, unit: 's', key: 'Kd',
        onInput: function (v) { st.Kd = v; update(); } });
      const rowQ = ui.row(gG);
      rowQ.classList.add('pid-btns');
      ui.button(rowQ, { label: 'P only', key: 'quickP', title: 'K_i = K_d = 0',
        onClick: function () { setGains(st.Kp, 0, 0); } });
      ui.button(rowQ, { label: 'PI', key: 'quickPI', title: 'K_d = 0; K_i becomes 1 if it was 0',
        onClick: function () { setGains(st.Kp, st.Ki === 0 ? 1 : st.Ki, 0); } });
      ui.button(rowQ, { label: 'PD', key: 'quickPD', title: 'K_i = 0; K_d becomes 1 if it was 0',
        onClick: function () { setGains(st.Kp, 0, st.Kd === 0 ? 1 : st.Kd); } });
      ui.button(rowQ, { label: 'PID', key: 'quickPID', title: 'A zero K_i or K_d becomes 1',
        onClick: function () { setGains(st.Kp, st.Ki === 0 ? 1 : st.Ki, st.Kd === 0 ? 1 : st.Kd); } });
      ui.note(gG, 'The D term is filtered, K<sub>d</sub>s/(τ<sub>f</sub>s + 1) with τ<sub>f</sub> = ' + fmt(TAU_F, 3) +
        ' s, so the controller is realisable (proper).');

      const gZ = ui.group(sp.side, 'Ziegler–Nichols (ultimate gain)');
      const rowZ = ui.row(gZ);
      rowZ.classList.add('pid-btns');
      const bKu = ui.button(rowZ, { label: 'Set K<sub>p</sub> = K<sub>u</sub>', key: 'setKu',
        onClick: function () { const P = plants[st.plant]; if (P.Ku) setGains(P.Ku, 0, 0); } });
      const bZ = ['P', 'PI', 'PID'].map(function (kind) {
        return ui.button(rowZ, { label: 'Z–N ' + kind, key: 'zn' + kind,
          onClick: function () {
            const P = plants[st.plant];
            if (!P.Ku) return;
            const g = model.zn(P.Ku, P.Pu, kind);
            setGains(g.Kp, g.Ki, g.Kd);
          } });
      });
      const znNote = ui.note(gZ, '');
      znNote.classList.add('pid-zn');
      znNote.setAttribute('data-key', 'zn');

      const gS = ui.group(sp.side, 'Simulation');
      const sT = ui.slider(gS, { label: 'Time span', min: 2, max: 60, step: 1, value: st.tEnd, unit: 's', key: 'tEnd',
        onInput: function (v) { st.tEnd = v; update(); } });
      const sD = ui.slider(gS, { label: 'Load disturbance d<sub>0</sub> (at t = span/2)', min: 0, max: 1, step: 0.05, value: st.d0, key: 'd0',
        onInput: function (v) { st.d0 = v; update(); } });
      const cU = ui.checkbox(gS, { label: 'Control signal u(t)', checked: st.showU, key: 'showU',
        onChange: function (v) { st.showU = v; update(); } });
      const rowB = ui.row(gS);
      rowB.classList.add('pid-btns');
      ui.button(rowB, { label: 'Hold curve', key: 'hold', primary: true, title: 'Keep the current output as a grey comparison curve',
        onClick: function () { hold(); update(); } });
      const bClear = ui.button(rowB, { label: 'Clear held', key: 'clearHeld', title: 'Remove the grey comparison curve',
        onClick: function () { held = null; update(); } });
      ui.button(rowB, { label: 'Reset', key: 'reset', title: 'Restore every default',
        onClick: function () { setState(Object.assign({ held: false }, DEFAULTS)); } });

      const rowTag = ui.row(sp.side);
      const tag = ui.tag(rowTag, { text: '', key: 'stability' });
      const ctrlLine = ui.note(sp.side, '');
      ctrlLine.setAttribute('data-key', 'ctrl');
      ctrlLine.classList.add('pid-ctrl');

      const stats = ui.stats(sp.side);
      stats.classList.add('pid-stats');
      const tEss = ui.stat(stats, { label: lab('step error e<sub>ss</sub>'), key: 'ess', title: 'Steady-state error to a unit step, e_ss = 1 − T(0)' });
      const tType = ui.stat(stats, { label: lab('loop type'), key: 'type', title: 'Number of poles of C(s)G(s) at the origin' });
      const tOs = ui.stat(stats, { label: lab('overshoot M<sub>p</sub>'), key: 'overshoot', title: 'Of the reference response (without the disturbance)' });
      const tTp = ui.stat(stats, { label: lab('peak time t<sub>p</sub>'), key: 'tPeak', title: 'Of the reference response (without the disturbance)' });
      const tTr = ui.stat(stats, { label: lab('rise time 10–90 %'), key: 'tRise', title: 'Of the reference response (without the disturbance)' });
      const tTs = ui.stat(stats, { label: lab('settling t<sub>s</sub> (2 %)'), key: 'tSettle', title: 'Last time |y − y_∞| > 2 % of y_∞ (reference response)' });
      const tPo = ui.stat(stats, { label: lab('closed-loop poles (roots of Δ)'), key: 'poles', title: 'Closest to the jω-axis first' });
      const tKu = ui.stat(stats, { label: lab('ultimate gain K<sub>u</sub>'), key: 'Ku' });
      const tPu = ui.stat(stats, { label: lab('ultimate period P<sub>u</sub>'), key: 'Pu' });
      const tDo = ui.stat(stats, { label: lab('offset due to d: d<sub>0</sub>·Y<sub>d</sub>(0)'), key: 'distOffset', title: 'Steady change of y caused by the load disturbance' });
      tPo.el.classList.add('pid-wide');
      tDo.el.classList.add('pid-wide');

      ui.hints(api.root, [
        'Plant 1/(s+1), <b>P only</b>: raise K<sub>p</sub> — the error shrinks as 1/(1 + K<sub>p</sub>) but never reaches zero. Now add K<sub>i</sub>: the error goes to zero (the integrator raises the loop type).',
        'Plant 1/(s+1)³: press <b>Set K<sub>p</sub> = K<sub>u</sub></b> — sustained oscillation with period P<sub>u</sub> ≈ 3.63 s. Then press <b>Z–N PID</b> and compare it with <b>Z–N P</b> and <b>Z–N PI</b> using <b>Hold curve</b>.',
        'Add K<sub>d</sub> to an oscillatory response: overshoot and settling time drop (extra damping), but the steady-state error does not change.',
        'Apply a load disturbance: only a controller with integral action brings the output back to the set-point — even for the motor plant, whose own integrator sits after the disturbance.',
        'Too much K<sub>i</sub>: more overshoot, and eventually instability.'
      ]);

      // ---------------------------------------------------------------- helpers
      function setGains(kp, ki, kd) {
        sKp.set(kp); sKi.set(ki); sKd.set(kd);
        st.Kp = sKp.get(); st.Ki = sKi.get(); st.Kd = sKd.get();
        update();
      }
      function hold() { held = { plant: st.plant, Kp: st.Kp, Ki: st.Ki, Kd: st.Kd, d0: st.d0 }; }

      function ctrlTerms(Kp, Ki, Kd) {
        const t = [];
        if (Kp !== 0) t.push(fmt(Kp, 4));
        if (Ki !== 0) t.push(fmt(Ki, 4) + '/s');
        if (Kd !== 0) t.push(fmt(Kd, 4) + ' s');
        return t.length ? t.join(' + ') : '0';
      }
      function ctrlKind(Kp, Ki, Kd) {
        const k = (Kp !== 0 ? 'P' : '') + (Ki !== 0 ? 'I' : '') + (Kd !== 0 ? 'D' : '');
        return k || 'none';
      }

      // cached computations
      let cLoop = null, cSim = null, cHeld = null, cMt = null;
      function getLoop() {
        const key = st.plant + '|' + st.Kp + '|' + st.Ki + '|' + st.Kd;
        if (!cLoop || cLoop.key !== key) {
          const L = model.loop(st.plant, st.Kp, st.Ki, st.Kd);
          const mt = L.stability === 'stable' && L.yFinal > 0 ? null : { overshootPct: NaN, tPeak: NaN, tRise: NaN, tSettle: NaN };
          cLoop = { key: key, L: L, mt: mt };
        }
        return cLoop;
      }
      function getSim() {
        const key = st.plant + '|' + st.Kp + '|' + st.Ki + '|' + st.Kd + '|' + st.tEnd + '|' + st.d0;
        if (!cSim || cSim.key !== key) {
          const s = model.simulate(st.plant, st.Kp, st.Ki, st.Kd, { tEnd: st.tEnd, n: NS, d0: st.d0 });
          const lc = getLoop();
          const mk = st.plant + '|' + st.Kp + '|' + st.Ki + '|' + st.Kd + '|' + st.tEnd;
          if (!cMt || cMt.key !== mk) cMt = { key: mk, mt: lc.mt || model.refMetrics(st.plant, st.Kp, st.Ki, st.Kd, st.tEnd, NS) };
          const mt = cMt.mt;
          cSim = { key: key, s: s, mt: mt };
        }
        return cSim;
      }
      function getHeld() {
        if (!held) { cHeld = null; return null; }
        const key = held.plant + '|' + held.Kp + '|' + held.Ki + '|' + held.Kd + '|' + st.tEnd + '|' + held.d0;
        if (!cHeld || cHeld.key !== key) {
          cHeld = { key: key, s: model.simulate(held.plant, held.Kp, held.Ki, held.Kd, { tEnd: st.tEnd, n: NS, d0: held.d0 }) };
        }
        return cHeld;
      }

      // ---------------------------------------------------------------- rich canvas labels (subscripts)
      // a label is a string or an array of parts: ['M', ['p'], ' = 3 %'] (an inner array is a subscript)
      function partsOf(l) { return typeof l === 'string' ? [l] : l; }
      function subFont(font) { return font.replace(/(\d+(?:\.\d+)?)px/, function (m0, n) { return Math.max(11, Math.round(Number(n) * 0.85)) + 'px'; }); }
      function richWidth(ctx, l, font) {
        let w = 0;
        partsOf(l).forEach(function (q) {
          ctx.font = Array.isArray(q) ? subFont(font) : font;
          w += ctx.measureText(Array.isArray(q) ? q[0] : q).width;
        });
        return w;
      }
      function richText(ctx, l, x, y, o) {
        const font = o.font || F12, tw = richWidth(ctx, l, font);
        let left = o.align === 'center' ? x - tw / 2 : o.align === 'right' ? x - tw : x;
        if (o.bg) { ctx.save(); ctx.fillStyle = o.bg; ctx.fillRect(left - 3, y - 9, tw + 6, 18); ctx.restore(); }
        ctx.save(); ctx.fillStyle = o.color || C.ink; ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
        partsOf(l).forEach(function (q) {
          const sub = Array.isArray(q), s = sub ? q[0] : q;
          ctx.font = sub ? subFont(font) : font;
          ctx.fillText(s, left, sub ? y + 4 : y);
          left += ctx.measureText(s).width;
        });
        ctx.restore();
      }
      // label placement: the cheapest candidate wins (cost = candidate order + covered boxes + leaving the frame)
      // placed label boxes are stored on the canvas element (el.pidLabels) for the browser tests
      function placer(ctx, frame, el) {
        const boxes = [], out = [];
        function ov(a, b) { return a.l < b.r && a.r > b.l && a.t < b.b && a.b > b.t; }
        return {
          block: function (x, y, rx, ry, w) { boxes.push({ l: x - rx, r: x + rx, t: y - ry, b: y + ry, w: w || 100 }); },
          put: function (l, font, color, cands) {
            let best = null;
            cands.forEach(function (c, n) {
              const tw = richWidth(ctx, c.lbl || l, font);
              const left = c.align === 'left' ? c.x : c.align === 'right' ? c.x - tw : c.x - tw / 2;
              const b = { l: left - 3, r: left + tw + 3, t: c.y - 9, b: c.y + 9 };
              let cost = n + (c.cost || 0);
              boxes.forEach(function (o) { if (ov(b, o)) cost += o.w; });
              if (b.l < frame.x || b.r > frame.x + frame.w || b.t < frame.y || b.b > frame.y + frame.h) cost += 1e7;
              if (!best || cost < best.cost) best = { cost: cost, c: c, b: b };
            });
            best.b.w = 1e5;
            boxes.push(best.b);
            out.push({ l: best.c.lbl || l, c: best.c, font: font, color: color, b: best.b });
            return best.c;
          },
          flush: function () {
            out.forEach(function (o) { richText(ctx, o.l, o.c.x, o.c.y, { align: o.c.align, color: o.color, font: o.font, bg: LBG }); });
            if (el) el.pidLabels = out.map(function (o) {
              return { text: partsOf(o.l).map(function (q) { return Array.isArray(q) ? q[0] : q; }).join(''), l: o.b.l, r: o.b.r, t: o.b.t, b: o.b.b };
            });
          }
        };
      }

      // ---------------------------------------------------------------- block diagram
      function splitTf(label) {
        const i = label.indexOf('/');
        let num = label.slice(0, i), den = label.slice(i + 1);
        if (den.charAt(0) === '(' && den.charAt(den.length - 1) === ')') {
          let depth = 0, outer = true;
          for (let k = 0; k < den.length; k++) {
            if (den[k] === '(') depth++;
            else if (den[k] === ')') { depth--; if (depth === 0 && k < den.length - 1) { outer = false; break; } }
          }
          if (outer) den = den.slice(1, -1);
        }
        return { num: num, den: den };
      }
      cvB.draw = function (ctx, w, h) {
        const narrow = w < 420;
        const font = narrow ? F11 : F12, bold = narrow ? 'bold 11px sans-serif' : B12;
        const R = 9, yM = 52, yFb = h - 12, bh = 36;
        const gtf = splitTf(plants[st.plant].label);
        ctx.font = font;
        const gW = Math.max(ctx.measureText(gtf.num).width, ctx.measureText(gtf.den).width) + 18;
        const full = ctrlTerms(st.Kp, st.Ki, st.Kd), kind = ctrlKind(st.Kp, st.Ki, st.Kd);
        const fixed = 16 + 4 * R + gW + 14 + 8;          // r label, two Σ, G block, y label, margins
        const wFull = ctx.measureText(full).width + 18;
        const wKind = Math.max(ctx.measureText(kind).width + 16, 44);
        let cTxt = full, cW = Math.max(wFull, 52);
        if ((w - fixed - cW) / 6 < 20) { cTxt = kind; cW = wKind; }
        const a = Math.max(10, (w - fixed - cW) / 6);     // arrow length (the last one is 2 a)
        let x = 6;
        const xr = x; x += 12;
        const x1 = x + a + R; x = x1 + R;                  // Σ1 centre
        const xc0 = x + a; x = xc0 + cW;                   // C block
        const x2 = x + a + R; x = x2 + R;                  // Σ2 centre
        const xg0 = x + a; x = xg0 + gW;                   // G block
        const xT = x + 0.9 * a, xEnd = x + 2 * a;          // take-off point, end of the y arrow
        const line = { color: C.ink, width: 1.5, head: 7 };
        D.text(ctx, 'r', xr, yM, { font: bold, color: C.ink });
        D.arrow(ctx, xr + 10, yM, x1 - R, yM, line);
        // Σ1
        ctx.save(); ctx.strokeStyle = C.ink; ctx.lineWidth = 1.5; ctx.fillStyle = '#ffffff';
        ctx.beginPath(); ctx.arc(x1, yM, R, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
        ctx.beginPath(); ctx.arc(x2, yM, R, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
        ctx.restore();
        D.text(ctx, 'Σ', x1, yM + 0.5, { font: F11, align: 'center', color: C.ink });
        D.text(ctx, 'Σ', x2, yM + 0.5, { font: F11, align: 'center', color: C.ink });
        D.text(ctx, '+', x1 - R - 3, yM - R - 4, { font: B12, align: 'right', color: C.ink });
        D.text(ctx, '−', x1 - R - 2, yM + R + 6, { font: B12, align: 'right', color: C.ink });
        D.text(ctx, '+', x2 - R - 3, yM - R - 4, { font: B12, align: 'right', color: C.ink });
        D.text(ctx, '+', x2 + R + 3, yM - R - 6, { font: B12, align: 'left', color: C.ink });
        // e, C, u
        D.arrow(ctx, x1 + R, yM, xc0, yM, line);
        if (a >= 14) D.text(ctx, 'e', (x1 + R + xc0) / 2, yM + 10, { font: font, align: 'center', color: C.muted });
        ctx.save();
        ctx.fillStyle = RASim.alpha(C.orange, 0.08); ctx.strokeStyle = C.orange; ctx.lineWidth = 1.8;
        ctx.fillRect(xc0, yM - bh / 2, cW, bh); ctx.strokeRect(xc0, yM - bh / 2, cW, bh);
        ctx.fillStyle = RASim.alpha(C.blue, 0.06); ctx.strokeStyle = C.blue;
        ctx.fillRect(xg0, yM - bh / 2, gW, bh); ctx.strokeRect(xg0, yM - bh / 2, gW, bh);
        ctx.restore();
        D.text(ctx, 'C(s)', xc0 + cW / 2, yM - bh / 2 - 9, { font: bold, align: 'center', color: C.orange });
        D.text(ctx, 'G(s)', xg0 + gW / 2, yM - bh / 2 - 9, { font: bold, align: 'center', color: C.blue });
        D.text(ctx, cTxt, xc0 + cW / 2, yM, { font: font, align: 'center', color: C.ink });
        // plant as a stacked fraction
        D.text(ctx, gtf.num, xg0 + gW / 2, yM - 8, { font: font, align: 'center', color: C.ink });
        ctx.save(); ctx.strokeStyle = C.ink; ctx.lineWidth = 1;
        ctx.beginPath(); ctx.moveTo(xg0 + 6, yM + 0.5); ctx.lineTo(xg0 + gW - 6, yM + 0.5); ctx.stroke(); ctx.restore();
        D.text(ctx, gtf.den, xg0 + gW / 2, yM + 9, { font: font, align: 'center', color: C.ink });
        D.arrow(ctx, xc0 + cW, yM, x2 - R, yM, line);
        if (a >= 14) D.text(ctx, 'u', (xc0 + cW + x2 - R) / 2, yM + 10, { font: font, align: 'center', color: C.muted });
        // disturbance d enters Σ2 from above (at the plant input)
        D.arrow(ctx, x2, 10, x2, yM - R, { color: COL.dist, width: 1.5, head: 7 });
        D.text(ctx, 'd', x2 - 6, 14, { font: bold, align: 'right', color: COL.dist });
        D.arrow(ctx, x2 + R, yM, xg0, yM, line);
        // output and feedback
        D.arrow(ctx, xg0 + gW, yM, xEnd, yM, line);
        D.text(ctx, 'y', xEnd + 3, yM, { font: bold, color: C.ink });
        ctx.save(); ctx.fillStyle = C.ink; ctx.beginPath(); ctx.arc(xT, yM, 3, 0, 2 * Math.PI); ctx.fill();
        ctx.strokeStyle = C.ink; ctx.lineWidth = 1.5;
        ctx.beginPath(); ctx.moveTo(xT, yM); ctx.lineTo(xT, yFb); ctx.lineTo(x1, yFb); ctx.stroke(); ctx.restore();
        D.arrow(ctx, x1, yFb, x1, yM + R, line);
      };

      // ---------------------------------------------------------------- output plot
      function yRange(arrs) {
        let lo = 0, hi = 1.5;
        arrs.forEach(function (a) {
          for (let k = 0; k < a.length; k++) {
            const v = a[k];
            if (!isFinite(v)) continue;
            if (v < lo) lo = v;
            if (v > hi) hi = v;
          }
        });
        lo = lo > -1e-3 ? 0 : Math.floor(lo / 0.25) * 0.25;
        hi = Math.ceil((hi - 1e-3) / 0.25) * 0.25;
        return { lo: Math.max(-2, lo), hi: Math.min(4, hi) };
      }
      function clampArr(a, lo, hi) {
        const span = hi - lo, l2 = lo - 1e4 * span, h2 = hi + 1e4 * span;
        return a.map(function (v) { return isFinite(v) ? Math.min(h2, Math.max(l2, v)) : NaN; });
      }
      const MARGIN = { left: 46, right: 12, top: 10 };
      cvY.draw = function (ctx, w, h) {
        const lc = getLoop(), L = lc.L, cs = getSim(), s = cs.s, mt = cs.mt, hd = getHeld();
        const T = st.tEnd, td = T / 2;
        const yr = yRange(hd ? [s.y, hd.s.y] : [s.y]);
        const p = RASim.plot(ctx, { box: { x: 0, y: 0, w: w, h: h }, margin: Object.assign({ bottom: st.showU ? 24 : 36 }, MARGIN),
          xmin: 0, xmax: T, ymin: yr.lo, ymax: yr.hi, xlabel: st.showU ? '' : 't (s)', ylabel: 'output y(t)' });
        p.axes();
        const rc = p.rect, yb = rc.y + rc.h;
        const stable = L.stability === 'stable' && L.yFinal > 0;
        if (stable) p.band(0.98 * L.yFinal, 1.02 * L.yFinal, { fill: COL.band });
        p.hline(1, { color: COL.ref, width: 1.3, dash: [6, 4] });
        if (hd) p.line(hd.s.t, clampArr(hd.s.y, yr.lo, yr.hi), { color: COL.held, width: 2.2 });
        const lay = placer(ctx, rc, cvY.canvas);
        if (st.d0 > 0) {
          p.vline(td, { color: COL.dist, width: 1.3, dash: [4, 4] });
          lay.block(p.X(td), rc.y + rc.h / 2, 1, rc.h / 2, 8);
        }
        p.line(s.t, clampArr(s.y, yr.lo, yr.hi), { color: COL.y, width: 2.5 });
        // keep labels off the curves
        function blockCurve(ts, ys, wgt) {
          let bx = -1e9, by = -1e9;
          for (let i = 0; i < ys.length; i++) {
            const x = p.X(ts[i]), y = p.Y(ys[i]);
            if (!isFinite(y) || y < rc.y - 3 || y > yb + 3) { bx = -1e9; continue; }
            const dd = Math.hypot(x - bx, y - by);
            if (dd < 4) continue;
            if (bx > -1e8 && dd < 400) {
              const k = Math.ceil(dd / 4);
              for (let j = 1; j < k; j++) lay.block(bx + (x - bx) * j / k, by + (y - by) * j / k, 3, 3, wgt);
            }
            lay.block(x, y, 3, 3, wgt); bx = x; by = y;
          }
        }
        blockCurve(s.t, s.y, 100);
        if (hd) blockCurve(hd.s.t, hd.s.y, 30);
        lay.block(rc.x + rc.w / 2, p.Y(1), rc.w / 2, 1, 5);
        // markers
        const showPk = stable && mt.overshootPct > 0 && isFinite(mt.tPeak) && (st.d0 === 0 || mt.tPeak < td);
        const showTs = stable && isFinite(mt.tSettle) && mt.tSettle > 0 && mt.tSettle <= T && (st.d0 === 0 || mt.tSettle <= td);
        let pk = null;
        if (showPk) {
          const ypk = L.yFinal * (1 + mt.overshootPct / 100);
          if (ypk <= yr.hi) {
            p.point(mt.tPeak, ypk, { color: COL.peak, r: 4.5 });
            pk = { x: p.X(mt.tPeak), y: p.Y(ypk) };
            lay.block(pk.x, pk.y, 6, 6);
          }
        }
        let xs = null;
        if (showTs) {
          xs = p.X(mt.tSettle);
          p.vline(mt.tSettle, { color: COL.ts, width: 1.4, dash: [5, 4] });
          lay.block(xs, rc.y + rc.h / 2, 1.5, rc.h / 2, 8);
        }
        if (pk) {
          const l = ['M', ['p'], ' = ' + fmt(mt.overshootPct, 3) + ' %'];
          lay.put(l, B12, COL.peak, [
            { x: pk.x + 9, y: pk.y - 4, align: 'left' }, { x: pk.x + 9, y: pk.y - 13, align: 'left' },
            { x: pk.x, y: pk.y - 16, align: 'center' }, { x: pk.x - 9, y: pk.y - 13, align: 'right' },
            { x: pk.x + 9, y: pk.y + 13, align: 'left' }, { x: pk.x - 9, y: pk.y + 13, align: 'right' },
            { x: pk.x + 9, y: pk.y + 26, align: 'left' }, { x: pk.x - 9, y: pk.y + 26, align: 'right' },
            { x: pk.x + 24, y: pk.y + 40, align: 'left', cost: 4 }, { x: pk.x - 24, y: pk.y + 40, align: 'right', cost: 4 },
            { x: pk.x + 30, y: pk.y + 56, align: 'left', cost: 6 }, { x: pk.x - 30, y: pk.y + 56, align: 'right', cost: 6 }]);
        }
        if (xs !== null) {
          const l = ['t', ['s'], ' = ' + fmt(mt.tSettle, 3) + ' s'];
          const ls = ['t', ['s']];
          const cands = [];
          [yb - 11, yb - 29, rc.y + 11, rc.y + 29, yb - 47].forEach(function (yy, i) {
            cands.push({ x: xs + 5, y: yy, align: 'left', cost: i });
            cands.push({ x: xs - 5, y: yy, align: 'right', cost: i + 0.5 });
          });
          lay.put(l, F12, COL.ts, cands.concat(cands.map(function (q) { return Object.assign({}, q, { cost: q.cost + 12, lbl: ls }); })));
        }
        if (st.d0 > 0) {
          const xd = p.X(td);
          lay.put('disturbance', F11, COL.dist, [
            { x: xd + 5, y: rc.y + 10, align: 'left' }, { x: xd - 5, y: rc.y + 10, align: 'right' },
            { x: xd + 5, y: rc.y + 28, align: 'left' }, { x: xd - 5, y: rc.y + 28, align: 'right' },
            { x: xd + 5, y: yb - 10, align: 'left' }, { x: xd - 5, y: yb - 10, align: 'right' }]);
        }
        lay.flush();
      };

      // ---------------------------------------------------------------- control-signal plot
      cvU.draw = function (ctx, w, h) {
        const s = getSim().s;
        let lo = 0, hi = 0;
        s.u.forEach(function (v) { if (isFinite(v)) { if (v < lo) lo = v; if (v > hi) hi = v; } });
        let ulo = Math.max(-UMAX, lo), uhi = Math.min(UMAX, hi);
        if (uhi - ulo < 1e-9) { ulo = -1; uhi = 1; }
        const stp = M.niceCeil((uhi - ulo) / 4);
        ulo = Math.max(-UMAX, Math.floor(ulo / stp - 1e-9) * stp);
        uhi = Math.min(UMAX, Math.ceil(uhi / stp + 1e-9) * stp);
        if (uhi === ulo) uhi = ulo + stp;
        const clipped = s.u.some(function (v) { return !isFinite(v) || v > uhi + 1e-9 || v < ulo - 1e-9; });
        const p = RASim.plot(ctx, { box: { x: 0, y: 0, w: w, h: h }, margin: Object.assign({ bottom: 36 }, MARGIN),
          xmin: 0, xmax: st.tEnd, ymin: ulo, ymax: uhi, xlabel: 't (s)', ylabel: 'u(t)' });
        p.axes();
        if (st.d0 > 0) p.vline(st.tEnd / 2, { color: COL.dist, width: 1.3, dash: [4, 4] });
        p.line(s.t, clampArr(s.u, ulo, uhi), { color: COL.u, width: 2 });
        cvU.canvas.pidClipped = clipped;
        if (clipped) {
          const rc = p.rect;
          richText(ctx, 'clipped (|u| > ' + fmt(UMAX, 3) + ')', rc.x + rc.w - 6, rc.y + 11, { align: 'right', font: F11, color: C.red, bg: LBG });
        }
      };

      // ---------------------------------------------------------------- read-outs
      const TAG = { stable: ['STABLE', 'ok'], marginal: ['MARGINALLY STABLE', 'warn'], unstable: ['UNSTABLE', 'bad'] };
      function sec(v) {
        if (v === Infinity) return '&gt; window';
        return isFinite(v) ? fmt(v, 4) + ' s' : '—';
      }
      // the pole closest to the jω-axis (smallest |Re|) first; conjugate pairs written once as a ± jb
      function polesText(ps) {
        const out = [];
        ps.slice().sort(function (p, q) { return Math.abs(p.re) - Math.abs(q.re) || q.im - p.im; }).forEach(function (q) {
          const mag = Math.max(1, Math.hypot(q.re, q.im));
          if (Math.abs(q.im) <= 1e-4 * mag) out.push(fmt(q.re, 4));
          else if (q.im > 0) {
            const z = RASim.fmtComplex({ re: q.re, im: q.im }, 4);
            out.push(z.indexOf(' + j') >= 0 ? z.replace(' + j', ' ± j') : '±' + z);
          }
        });
        return out.join(', ');
      }
      let legendKey = '';
      function update() {
        const lc = getLoop(), L = lc.L, cs = getSim(), mt = cs.mt;
        const P = plants[st.plant];
        const stable = L.stability === 'stable';
        const tg = TAG[L.stability];
        tag.set(tg[0], tg[1]);
        ctrlLine.innerHTML = 'C(s) = ' + ctrlTerms(st.Kp, st.Ki, st.Kd);
        if (stable) {
          tEss.set(fmt(L.ess, 4) + ' (' + fmt(100 * L.ess, 4) + ' %)');
          tEss.setState(Math.abs(L.ess) < 1e-12 ? 'ok' : 'warn');
        } else { tEss.set('—'); tEss.setState(''); }
        tType.set(fmt(L.type, 1));
        tOs.set(isFinite(mt.overshootPct) ? fmt(mt.overshootPct, 4) + ' %' : '—');
        tOs.setState(!isFinite(mt.overshootPct) ? '' : mt.overshootPct > 25 ? 'bad' : mt.overshootPct > 5 ? 'warn' : 'ok');
        tTp.set(sec(mt.tPeak));
        tTr.set(sec(mt.tRise));
        tTs.set(sec(mt.tSettle));
        tPo.set(polesText(L.poles));
        tKu.set(P.Ku ? fmt(P.Ku, 4) : '—');
        tPu.set(P.Pu ? fmt(P.Pu, 4) + ' s' : '—');
        tDo.set(stable ? fmt(st.d0 * L.distOffsetPerUnit, 4) : '—');

        // Ziegler–Nichols group
        const hasKu = !!P.Ku;
        [bKu].concat(bZ).forEach(function (b) {
          b.disabled = !hasKu;
          b.title = hasKu ? '' : 'no ultimate gain: this loop is stable for every K_p';
        });
        if (hasKu) {
          const zP = model.znDetail(P.Ku, P.Pu, 'P'), zI = model.znDetail(P.Ku, P.Pu, 'PI'), zD = model.znDetail(P.Ku, P.Pu, 'PID');
          znNote.innerHTML = 'K<sub>u</sub> = ' + fmt(P.Ku, 4) + ', P<sub>u</sub> = 2π/√3 = ' + fmt(P.Pu, 5) + ' s.<br>' +
            '<b>P:</b> K<sub>p</sub> = 0.5K<sub>u</sub> = ' + fmt(zP.Kp, 4) + '<br>' +
            '<b>PI:</b> K<sub>p</sub> = 0.45K<sub>u</sub> = ' + fmt(zI.Kp, 4) + ', T<sub>i</sub> = P<sub>u</sub>/1.2 = ' + fmt(zI.Ti, 5) +
            ' s → K<sub>i</sub> = K<sub>p</sub>/T<sub>i</sub> = ' + fmt(zI.Ki, 4) + '<br>' +
            '<b>PID:</b> K<sub>p</sub> = 0.6K<sub>u</sub> = ' + fmt(zD.Kp, 4) + ', T<sub>i</sub> = P<sub>u</sub>/2 = ' + fmt(zD.Ti, 5) +
            ' s, T<sub>d</sub> = P<sub>u</sub>/8 = ' + fmt(zD.Td, 5) + ' s → K<sub>i</sub> = K<sub>p</sub>/T<sub>i</sub> = ' + fmt(zD.Ki, 4) +
            ', K<sub>d</sub> = K<sub>p</sub>T<sub>d</sub> = ' + fmt(zD.Kd, 4);
        } else {
          znNote.innerHTML = 'No ultimate gain: this loop is stable for every K<sub>p</sub>, so the Z–N rules do not apply.';
        }

        bClear.disabled = !held;
        cvU.canvas.style.display = st.showU ? '' : 'none';
        const items = [{ color: COL.y, label: 'output y(t)' }, { color: COL.ref, dash: true, label: 'reference r = 1' }];
        if (stable && L.yFinal > 0) items.push({ color: RASim.alpha(C.green, 0.45), label: '±2 % band around y<sub>∞</sub>' });
        if (held) items.push({ color: COL.held, label: 'held: C(s) = ' + ctrlTerms(held.Kp, held.Ki, held.Kd) +
          (held.plant !== st.plant ? ', G(s) = ' + plants[held.plant].label : '') + (held.d0 !== st.d0 ? ', d<sub>0</sub> = ' + fmt(held.d0, 3) : '') });
        if (st.d0 > 0) items.push({ color: COL.dist, dash: true, label: 'load step d<sub>0</sub> = ' + fmt(st.d0, 3) + ' at t = ' + fmt(st.tEnd / 2, 3) + ' s' });
        if (st.showU) items.push({ color: COL.u, label: 'control signal u(t)' });
        const lk = JSON.stringify(items);
        if (lk !== legendKey) { legend.set(items); legendKey = lk; }
        cvB.redraw(); cvY.redraw(); if (st.showU) cvU.redraw();
      }

      function getState() {
        return { plant: st.plant, Kp: st.Kp, Ki: st.Ki, Kd: st.Kd, tEnd: st.tEnd, d0: st.d0, showU: st.showU, held: !!held };
      }
      function num(v) { const x = Number(v); return isFinite(x) ? x : null; }
      function setState(s) {
        s = s || {};
        if (s.plant != null && plants[String(s.plant)]) { st.plant = String(s.plant); selPlant.set(st.plant); }
        [['Kp', sKp, 0, 20], ['Ki', sKi, 0, 20], ['Kd', sKd, 0, 10], ['tEnd', sT, 2, 60], ['d0', sD, 0, 1]].forEach(function (q) {
          const v = s[q[0]] != null ? num(s[q[0]]) : null;
          if (v === null) return;
          q[1].set(M.clamp(v, q[2], q[3])); st[q[0]] = q[1].get();
        });
        if (s.showU != null) { st.showU = !!s.showU; cU.set(st.showU); }
        if (s.held != null) { if (!s.held) held = null; else if (!held) hold(); }
        update();
      }

      update();
      return { getState: getState, setState: setState };
    }
  });
})();
