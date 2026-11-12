// bode-plot — "Bode plot builder with gain and phase margins"
// Sits beside notes/src/04_electrical.html, section "Bode plots" (and the gain-margin / phase-margin
// definitions in "Nyquist criterion" just above it).
//
// SIGN CONVENTIONS AND MODEL
//   Open loop in Bode (time-constant) form: G(s) = K · s^(−N) · Π(factors) · e^(−sT), K > 0, T ≥ 0.
//   N ∈ {−1, 0, 1, 2}: N = 1 one integrator 1/s, N = 2 is 1/s², N = −1 one differentiator s, N = 0 none.
//   Factors (w = corner frequency in rad/s):
//     'pole'    1/(1 + s/w)                       real pole at s = −w
//     'zero'    (1 + s/w)                         real zero at s = −w
//     'rhpzero' (1 − s/w)                         right-half-plane zero at s = +w (non-minimum phase)
//     'quad'    1/(1 + 2ζ(s/ωₙ) + (s/ωₙ)²), w = ωₙ  complex pole pair, 0 < ζ ≤ 1
//     'off'     ignored
//   sys = { K, N, factors: [{ type, w, zeta }, …], delay }.
//   Exact response at s = jω, factor by factor (continuous phase, never atan2 of the whole product):
//     |G| in dB = 20 log₁₀K − 20N log₁₀ω + Σ: pole −10 log₁₀(1 + (ω/w)²); zero, rhpzero +10 log₁₀(1 + (ω/w)²);
//                 quad −10 log₁₀[(1 − u²)² + (2ζu)²], u = ω/ωₙ; delay 0.
//     ∠G in DEGREES = −90N + Σ: pole −atan(ω/w); zero +atan(ω/w); rhpzero −atan(ω/w);
//                 quad −atan2(2ζu, 1 − u²) (0 → −180°); delay −ωT·180/π.
//   Asymptotes: magnitude corners at w (±20 dB/dec per real factor, −40 dB/dec per quad above ωₙ);
//     phase 0 → ∓90° linear in log ω from w/10 to 10w (rhpzero like a pole), quad 0 → −180° from
//     ωₙ·10^(−ζ) to ωₙ·10^(ζ) (−90/ζ °/dec), the delay keeps its exact −ωT.
//   Margins: gain crossover ω_gc where |G| = 0 dB, PM = 180° + ∠G(jω_gc) using the CONTINUOUS phase (not wrapped);
//     phase crossover ω_pc where ∠G = −180° − 360°·k (k ≥ 0), GM = −|G(jω_pc)| in dB; ratio = 10^(GM/20).
//     Search 1e-4 … 1e6 rad/s on 4001 log-spaced samples, each crossing refined by bisection in log ω.
//     Reported PM = smallest over all gain crossovers (Infinity when |G| < 0 dB everywhere, NaN when > 0 dB
//     everywhere or when |G| ≡ 0 dB, then `unity` is true); GM = smallest over all phase crossovers (Infinity if none).
//     With a delay the phase crossovers are unbounded in number: the first 200 are listed (`truncated` true,
//     `phaseCrossoverCount` = total in the search range) and the smallest GM is still found by branch and bound.
//   Closed loop: unity NEGATIVE feedback. polys(sys) = { num, den } (highest power first, Bode-form coefficients,
//     K inside num, G = num/den without the delay). Delay-free: closed-loop poles = roots of den + num;
//     closedLoopStable = every pole has Re < −1e-6·max(1, |p|) (a pole that close to the jω-axis = MARGINAL,
//     which returns false). With a delay closedLoopStable returns null and the panel decides "from margins"
//     (stable when GM > 0 dB and PM > 0°).
//   Units: ω in rad/s, T in s, magnitudes in dB, phases in degrees, slopes in dB/decade.
//
// STATE KEYS (getState / setState)
//   K (0.01 … 1000), N (−1 | 0 | 1 | 2), factors: exactly 4 × { type: 'off'|'pole'|'zero'|'rhpzero'|'quad',
//   w (0.01 … 1000), zeta (0.05 … 1) }, delay (0 … 2 s), showExact, showAsym, showFactors, showMargins (booleans)
//
// STAT KEYS:   type, slopeLow, slopeHigh, wgc, pm, wpc, gm, errConst, Mr
// TAG KEYS:    stability   (CLOSED LOOP STABLE / CLOSED LOOP UNSTABLE / MARGINAL, + "(from margins)" with a delay)
// MATRIX KEYS: (none)
// DATA-KEY:    tf (G(s) in Bode form), cursor (hover read-out: ω, |G| in dB, ∠G)
// CANVAS KEYS: bode
// CONTROL KEYS: preset, K, N, slot1type … slot4type, slot1w … slot4w, slot1zeta … slot4zeta, delay,
//               showExact, showAsym, showFactors, showMargins, reset
(function () {
  'use strict';
  const RASim = globalThis.RASim;
  const M = RASim.math;
  const ID = 'bode-plot';
  const R2D = 180 / Math.PI;
  const LN10 = Math.LN10;
  const TYPES = ['pole', 'zero', 'rhpzero', 'quad'];
  const SEARCH = { lw0: -4, lw1: 6, n: 4001 };
  const LIST_MAX = 200;

  // ---------------------------------------------------------------- model helpers (pure)
  function isActive(f) {
    return !!f && TYPES.indexOf(f.type) >= 0 && Number(f.w) > 0 && (f.type !== 'quad' || Number(f.zeta) > 0);
  }
  function activeFactors(sys) {
    return ((sys && sys.factors) || []).filter(isActive).map(function (f) {
      return { type: f.type, w: Number(f.w), zeta: f.type === 'quad' ? Number(f.zeta) : (f.zeta != null ? Number(f.zeta) : 0.5) };
    });
  }
  // normalised system: K, N, active factors, delay, 20 log K
  function norm(sys) {
    sys = sys || {};
    const K = sys.K != null ? Number(sys.K) : 1;
    const N = sys.N != null ? Number(sys.N) : 0;
    const T = Number(sys.delay) > 0 ? Number(sys.delay) : 0;
    return { K: K, N: N, f: activeFactors(sys), T: T, gDb: 20 * Math.log10(K) };
  }
  function log10p(x2) { return Math.log1p(x2) / LN10; }       // log10(1 + x²) given x²

  function magOf(ns, w) {
    let m = ns.gDb - 20 * ns.N * Math.log10(w);
    for (let i = 0; i < ns.f.length; i++) {
      const f = ns.f[i];
      if (f.type === 'quad') {
        const u = w / f.w, re = 1 - u * u, im = 2 * f.zeta * u;
        m -= 10 * Math.log10(re * re + im * im);
      } else {
        const x = w / f.w, L = 10 * log10p(x * x);
        m += f.type === 'pole' ? -L : L;
      }
    }
    return m;
  }
  function phaseOf(ns, w) {
    let p = -90 * ns.N;
    for (let i = 0; i < ns.f.length; i++) {
      const f = ns.f[i];
      if (f.type === 'quad') {
        const u = w / f.w;
        p -= Math.atan2(2 * f.zeta * u, 1 - u * u) * R2D;
      } else {
        const a = Math.atan(w / f.w) * R2D;
        p += f.type === 'zero' ? a : -a;
      }
    }
    return p - w * ns.T * R2D;
  }
  function asymMagOf(ns, w) {
    let m = ns.gDb - 20 * ns.N * Math.log10(w);
    for (let i = 0; i < ns.f.length; i++) {
      const f = ns.f[i];
      if (w <= f.w) continue;
      const d = Math.log10(w / f.w);
      if (f.type === 'quad') m -= 40 * d;
      else if (f.type === 'pole') m -= 20 * d;
      else m += 20 * d;
    }
    return m;
  }
  function asymPhaseOf(ns, w) {
    let p = -90 * ns.N;
    for (let i = 0; i < ns.f.length; i++) {
      const f = ns.f[i];
      if (f.type === 'quad') {
        const lo = f.w * Math.pow(10, -f.zeta), hi = f.w * Math.pow(10, f.zeta);
        if (w >= hi) p -= 180;
        else if (w > lo) p += -90 - (90 / f.zeta) * Math.log10(w / f.w);
      } else {
        let a;
        if (w <= f.w / 10) a = 0;
        else if (w >= 10 * f.w) a = 90;
        else a = 45 * Math.log10(w / (f.w / 10));
        p += f.type === 'zero' ? a : -a;
      }
    }
    return p - w * ns.T * R2D;
  }

  // sign crossings of a sampled function (samples ys at abscissae xs), refined by bisection of f.
  // |y| <= eps counts as zero; a run of zeros between opposite signs gives one crossing (the run's middle).
  // root of f in the bracket [a, b] (f(a), f(b) of opposite signs): Illinois false position, which keeps the bracket
  // and converges superlinearly on these smooth functions (about 6–10 evaluations instead of ~45 bisection steps).
  // It stops when the new point moves by less than 1e-13 (in log10 ω), or after 60 steps, when plain bisection
  // takes over until the bracket is 1e-14 wide.
  function bisectSign(f, a, b) {
    let fa = f(a);
    if (fa === 0) return a;
    let fb = f(b);
    if (fb === 0) return b;
    if ((fa > 0) === (fb > 0)) return 0.5 * (a + b);
    let lo = a, hi = b, flo = fa, fhi = fb, side = 0, x = NaN;
    for (let it = 0; it < 60; it++) {
      let xn = (lo * fhi - hi * flo) / (fhi - flo);
      if (!(xn > lo && xn < hi)) xn = 0.5 * (lo + hi);
      const fx = f(xn);
      if (fx === 0) return xn;
      const step = Math.abs(xn - x);
      x = xn;
      if ((fx > 0) === (flo > 0)) { lo = x; flo = fx; if (side === -1) fhi *= 0.5; side = -1; }
      else { hi = x; fhi = fx; if (side === 1) flo *= 0.5; side = 1; }
      if (step < 1e-13 || hi - lo <= 1e-14) return x;
    }
    for (let it = 0; it < 80 && hi - lo > 1e-14; it++) {
      const mid = 0.5 * (lo + hi), fm = f(mid);
      if (fm === 0) return mid;
      if ((fm > 0) === (flo > 0)) { lo = mid; flo = fm; } else hi = mid;
    }
    return 0.5 * (lo + hi);
  }
  function crossings(xs, ys, f, eps) {
    const out = [];
    let lastSign = 0, lastIdx = -1;
    for (let i = 0; i < ys.length; i++) {
      const s = ys[i] > eps ? 1 : ys[i] < -eps ? -1 : 0;
      if (s === 0) continue;
      if (lastSign !== 0 && s !== lastSign) {
        if (i === lastIdx + 1) out.push(bisectSign(f, xs[lastIdx], xs[i]));
        else out.push(0.5 * (xs[lastIdx + 1] + xs[i - 1]));
      }
      lastSign = s; lastIdx = i;
    }
    return { roots: out, lastSign: lastSign };
  }

  function marginsOf(ns) {
    const n = SEARCH.n, lws = new Array(n), mag = new Array(n), ph = new Array(n);
    for (let i = 0; i < n; i++) {
      const lw = SEARCH.lw0 + (SEARCH.lw1 - SEARCH.lw0) * i / (n - 1);
      const w = Math.pow(10, lw);
      lws[i] = lw; mag[i] = magOf(ns, w); ph[i] = phaseOf(ns, w);
    }
    // gain crossovers
    const gc = crossings(lws, mag, function (lw) { return magOf(ns, Math.pow(10, lw)); }, 1e-9);
    const gainCrossovers = gc.roots.map(function (lw) { return Math.pow(10, lw); });
    let pmDeg, wgc = NaN, unity = false;
    if (gainCrossovers.length) {
      pmDeg = Infinity;
      gainCrossovers.forEach(function (w) {
        const pm = 180 + phaseOf(ns, w);
        if (pm < pmDeg) { pmDeg = pm; wgc = w; }
      });
    } else if (gc.lastSign === 0) { pmDeg = NaN; unity = true; }       // |G| = 0 dB at every sample
    else pmDeg = gc.lastSign < 0 ? Infinity : NaN;

    // phase crossovers: h = (∠G + 180)/360 crosses an integer m ≤ 0 (∠G = −180° − 360°·k, k = −m)
    const h = ph.map(function (p) { return (p + 180) / 360; });
    const ivs = [];
    let total = 0;
    for (let i = 0; i + 1 < n; i++) {
      const lo = Math.min(h[i], h[i + 1]), hi = Math.max(h[i], h[i + 1]);
      const mA = Math.floor(lo) + 1, mB = Math.min(0, Math.floor(hi));
      if (mB >= mA) { ivs.push({ i: i, mA: mA, mB: mB, down: h[i + 1] < h[i] }); total += mB - mA + 1; }
    }
    function solve(iv, m) {
      return Math.pow(10, bisectSign(function (lw) { return (phaseOf(ns, Math.pow(10, lw)) + 180) / 360 - m; }, lws[iv.i], lws[iv.i + 1]));
    }
    const phaseCrossovers = [];
    let bestMag = -Infinity, wpc = NaN;
    function consider(w) {
      const md = magOf(ns, w);
      if (md > bestMag) { bestMag = md; wpc = w; }
    }
    // the k-th crossing of an interval in ascending ω order
    function mAt(iv, k) { return iv.down ? iv.mB - k : iv.mA + k; }
    const rest = [];
    for (let j = 0; j < ivs.length; j++) {
      const iv = ivs[j], cnt = iv.mB - iv.mA + 1;
      let k = 0;
      for (; k < cnt && phaseCrossovers.length < LIST_MAX; k++) {
        const w = solve(iv, mAt(iv, k));
        if (!phaseCrossovers.length || w > phaseCrossovers[phaseCrossovers.length - 1] * (1 + 1e-12)) {
          phaseCrossovers.push(w); consider(w);
        }
      }
      if (k < cnt) rest.push({ iv: iv, k0: k, cnt: cnt, bound: Math.max(mag[iv.i], mag[iv.i + 1]) + 10 });
    }
    // branch and bound over the crossings beyond the listed ones (delay: unbounded number of crossings);
    // inside an interval holding more than 8 crossings only the first and the last are refined
    rest.sort(function (a, b) { return b.bound - a.bound; });
    for (let j = 0; j < rest.length; j++) {
      const r = rest[j];
      if (r.bound <= bestMag) break;
      const left = r.cnt - r.k0;
      if (left <= 8) { for (let k = r.k0; k < r.cnt; k++) consider(solve(r.iv, mAt(r.iv, k))); }
      else { consider(solve(r.iv, mAt(r.iv, r.k0))); consider(solve(r.iv, mAt(r.iv, r.cnt - 1))); }
    }
    const gmDb = isFinite(bestMag) ? -bestMag : Infinity;
    return {
      gainCrossovers: gainCrossovers, phaseCrossovers: phaseCrossovers,
      pmDeg: pmDeg, wgc: wgc, gmDb: gmDb, wpc: isFinite(bestMag) ? wpc : NaN,
      unity: unity, phaseCrossoverCount: total, truncated: total > phaseCrossovers.length
    };
  }

  function polysOf(ns) {
    let num = [ns.K], den = [1];
    if (ns.N > 0) for (let i = 0; i < ns.N; i++) den = M.polyMul(den, [1, 0]);
    if (ns.N < 0) for (let i = 0; i < -ns.N; i++) num = M.polyMul(num, [1, 0]);
    ns.f.forEach(function (f) {
      if (f.type === 'pole') den = M.polyMul(den, [1 / f.w, 1]);
      else if (f.type === 'zero') num = M.polyMul(num, [1 / f.w, 1]);
      else if (f.type === 'rhpzero') num = M.polyMul(num, [-1 / f.w, 1]);
      else den = M.polyMul(den, [1 / (f.w * f.w), 2 * f.zeta / f.w, 1]);
    });
    return { num: num, den: den };
  }
  // roots of den + num (delay-free part). Leading terms that cancel (relative to the two summands) are dropped,
  // then s = c·z balances the polynomial (c = (|a₀|/|aₙ|)^(1/n)) so the core's root finder sees moderate coefficients.
  function clPolesOf(ns) {
    const P = polysOf(ns);
    const len = Math.max(P.num.length, P.den.length);
    const pad = function (p) { const o = []; for (let i = 0; i < len - p.length; i++) o.push(0); return o.concat(p); };
    const d = pad(P.den), nm = pad(P.num);
    let a = d.map(function (v, i) { return v + nm[i]; });
    let sc = d.map(function (v, i) { return Math.abs(v) + Math.abs(nm[i]); });
    while (a.length > 1 && Math.abs(a[0]) <= 1e-12 * sc[0]) { a = a.slice(1); sc = sc.slice(1); }
    const deg = a.length - 1;
    if (deg < 1) return [];
    const c0 = Math.abs(a[deg]), cn = Math.abs(a[0]);
    const c = c0 > 0 ? Math.pow(c0 / cn, 1 / deg) : 1;
    const b = a.map(function (v, i) { return v * Math.pow(c, deg - i); });
    const bmax = b.reduce(function (s, v) { return Math.max(s, Math.abs(v)); }, 0);
    return M.polyRoots(b.map(function (v) { return v / bmax; })).map(function (z) { return { re: z.re * c, im: z.im * c }; });
  }
  function marginalTol(p) { return 1e-6 * Math.max(1, Math.hypot(p.re, p.im)); }

  const model = {
    response: function (sys, w) { const ns = norm(sys); return { magDb: magOf(ns, w), phaseDeg: phaseOf(ns, w) }; },
    asymMagDb: function (sys, w) { return asymMagOf(norm(sys), w); },
    asymPhaseDeg: function (sys, w) { return asymPhaseOf(norm(sys), w); },
    factorResponse: function (f, w) {
      f = f || {};
      if (f.type === 'gain') return { magDb: 20 * Math.log10(Number(f.K)), phaseDeg: 0 };
      if (f.type === 'integrator') return { magDb: -20 * Number(f.N) * Math.log10(w), phaseDeg: -90 * Number(f.N) };
      if (f.type === 'delay') return { magDb: 0, phaseDeg: -w * Number(f.T) * R2D };
      if (!isActive(f)) return { magDb: 0, phaseDeg: 0 };
      const ns = { K: 1, N: 0, f: activeFactors({ factors: [f] }), T: 0, gDb: 0 };
      return { magDb: magOf(ns, w), phaseDeg: phaseOf(ns, w) };
    },
    corners: function (sys) { return norm(sys).f.map(function (f) { return f.w; }).sort(function (a, b) { return a - b; }); },
    margins: function (sys) { return marginsOf(norm(sys)); },
    polys: function (sys) { return polysOf(norm(sys)); },
    // extra: closed-loop poles of the delay-free loop (roots of den + num)
    closedLoopPoles: function (sys) { return clPolesOf(norm(sys)); },
    closedLoopStable: function (sys) {
      const ns = norm(sys);
      if (ns.T > 0) return null;
      return clPolesOf(ns).every(function (p) { return p.re < -marginalTol(p); });
    },
    // extra: true when a closed-loop pole lies within 1e-6·max(1, |p|) of the jω-axis (delay-free only)
    closedLoopMarginal: function (sys) {
      const ns = norm(sys);
      if (ns.T > 0) return null;
      const ps = clPolesOf(ns);
      return !ps.some(function (p) { return p.re > marginalTol(p); }) && ps.some(function (p) { return Math.abs(p.re) <= marginalTol(p); });
    },
    slopes: function (sys) {
      const ns = norm(sys);
      let n = ns.N;
      ns.f.forEach(function (f) { n += f.type === 'pole' ? 1 : f.type === 'quad' ? 2 : -1; });
      return { low: -20 * ns.N + 0, high: -20 * n + 0 };
    },
    resonance: function (sys) {
      return norm(sys).f.filter(function (f) { return f.type === 'quad' && f.zeta < Math.SQRT1_2; }).map(function (f) {
        const z = f.zeta, Mr = 1 / (2 * z * Math.sqrt(1 - z * z));
        return { Mr: Mr, MrDb: 20 * Math.log10(Mr), wr: f.w * Math.sqrt(1 - 2 * z * z), wn: f.w, zeta: z };
      });
    },
    // optional second argument: margins(sys) already computed
    freqRange: function (sys, mg) {
      const cs = model.corners(sys);
      let lo, hi;
      if (!cs.length) { lo = -2; hi = 2; }
      else { lo = Math.floor(Math.log10(cs[0]) + 1e-9) - 1; hi = Math.ceil(Math.log10(cs[cs.length - 1]) - 1e-9) + 1; }
      mg = mg || model.margins(sys);
      [mg.wgc, mg.wpc].forEach(function (w) {
        if (!(isFinite(w) && w > 0)) return;
        const lw = Math.log10(w);
        while (lw < lo + 1) lo--;
        while (lw > hi - 1) hi++;
      });
      lo = Math.max(-3, lo); hi = Math.min(5, hi);
      return { wmin: Math.pow(10, lo), wmax: Math.pow(10, hi) };
    }
  };

  // ---------------------------------------------------------------- presets and defaults
  const DEF_SLOT = { type: 'off', w: 10, zeta: 0.5 };
  const PRESETS = [
    { value: 'A', label: '1 / [s(1+s)(1+s/2)] — type 1, three poles', K: 1, N: 1, delay: 0,
      factors: [{ type: 'pole', w: 1 }, { type: 'pole', w: 2 }] },
    { value: 'B', label: '100 / [(1+s)(1+s/10)(1+s/100)] — type 0', K: 100, N: 0, delay: 0,
      factors: [{ type: 'pole', w: 1 }, { type: 'pole', w: 10 }, { type: 'pole', w: 100 }] },
    { value: 'C', label: 'Resonant pair: ζ = 0.2, ωₙ = 10', K: 1, N: 0, delay: 0,
      factors: [{ type: 'quad', w: 10, zeta: 0.2 }] },
    { value: 'D', label: 'Lead compensator (1+s)/(1+s/10)', K: 1, N: 0, delay: 0,
      factors: [{ type: 'zero', w: 1 }, { type: 'pole', w: 10 }] },
    { value: 'E', label: 'Lag compensator (1+s/10)/(1+s)', K: 1, N: 0, delay: 0,
      factors: [{ type: 'zero', w: 10 }, { type: 'pole', w: 1 }] },
    { value: 'F', label: 'Non-minimum phase (1−s/10)/(1+s/10)', K: 1, N: 0, delay: 0,
      factors: [{ type: 'rhpzero', w: 10 }, { type: 'pole', w: 10 }] },
    { value: 'G', label: 'Delay e^(−0.5s)/(1+s)', K: 1, N: 0, delay: 0.5,
      factors: [{ type: 'pole', w: 1 }] }
  ];
  function presetState(v) {
    const q = PRESETS.find(function (x) { return x.value === v; }) || PRESETS[0];
    const fs = [];
    for (let i = 0; i < 4; i++) {
      const f = q.factors[i];
      fs.push(f ? { type: f.type, w: f.w, zeta: f.zeta != null ? f.zeta : DEF_SLOT.zeta } : Object.assign({}, DEF_SLOT));
    }
    return { K: q.K, N: q.N, factors: fs, delay: q.delay };
  }
  const DEFAULTS = Object.assign(presetState('A'), { showExact: true, showAsym: true, showFactors: false, showMargins: true });

  RASim.register(ID, {
    title: 'Bode plot builder with gain and phase margins',
    model: model,
    mount: function (api) {
      const ui = RASim.ui, C = RASim.colors;
      const fmt = RASim.fmt, fmtFixed = RASim.fmtFixed;
      const SC = '.sim[data-sim="' + ID + '"]';
      const LBG = RASim.alpha('#ffffff', 0.88);
      const F11 = '11px sans-serif', B12 = 'bold 12px sans-serif';
      const FCOL = [C.orange, C.purple, C.teal, C.gold];

      RASim.css(ID,
        SC + ' .bp-nt { text-transform: none; letter-spacing: 0; font-size: 11.5px; }\n' +
        SC + ' .bp-eq { font-size: 13.5px; color: ' + C.ink + '; margin: 0 0 6px; line-height: 1.5; overflow-wrap: anywhere; }\n' +
        SC + ' .bp-line { min-height: 1.45em; margin-top: 4px; overflow-wrap: anywhere; }\n' +
        SC + ' .bp-slot { border-top: 1px dashed ' + C.grid + '; padding-top: 6px; display: flex; flex-direction: column; gap: 4px; }\n' +
        SC + ' .bp-slot:first-child { border-top: none; padding-top: 0; }\n' +
        SC + ' .bp-wide { grid-column: 1 / -1; }\n' +
        SC + ' .bp-gap { margin-top: 6px; }\n' +
        SC + ' .bp-top { margin-bottom: 8px; }\n');
      function lab(html) { return '<span class="bp-nt">' + html + '</span>'; }

      const st = JSON.parse(JSON.stringify(DEFAULTS));
      // (A local work-around for the number box ignoring a re-typed value after a track move was removed once
      // the core fixed ui.slider; the core's regression test covers it.)
      function slider(parent, opts) { return ui.slider(parent, opts); }
      let D = null;            // cached curves and results for the current state
      let hoverW = null;       // ω under the mouse

      // ---- layout ----
      const intro = ui.note(api.root, 'Pick a preset or build G(s) from the factors, then raise K: the magnitude slides up, the gain crossover moves right and the phase margin shrinks.');
      intro.classList.add('bp-top');
      const sp = ui.split(api.root, { ratio: 1.6 });
      const tfLine = ui.el('p', { 'class': 'bp-eq' }, sp.main);
      tfLine.setAttribute('data-key', 'tf');
      const cv = RASim.canvas(sp.main, { aspect: 1.0, minHeight: 420, maxHeight: 620, key: 'bode' });
      const cursorLine = ui.note(sp.main, '');
      cursorLine.setAttribute('data-key', 'cursor');
      cursorLine.classList.add('bp-line');
      const legend = ui.legend(sp.main, []);
      legend.el.classList.add('bp-gap');

      const gG = ui.group(sp.side, 'Open loop ' + lab('G(s)'));
      const selPre = ui.select(gG, { label: 'Preset', key: 'preset', value: 'A',
        options: PRESETS.map(function (q) { return { value: q.value, label: q.label }; }).concat([{ value: 'custom', label: 'Custom (edited)' }]),
        onChange: function (v) {
          if (v === 'custom') { selPre.set(matchPreset()); return; }
          applyState(presetState(v));
          update();
        } });
      const sK = slider(gG, { label: 'Gain K', min: 0.01, max: 1000, scale: 'log', value: st.K, key: 'K',
        onInput: function (v) { st.K = v; update(); } });
      const segN = ui.segmented(gG, { label: 'Type', key: 'N', value: String(st.N),
        options: [{ value: '-1', label: 's' }, { value: '0', label: 'none' }, { value: '1', label: '1/s' }, { value: '2', label: '1/s²' }],
        onChange: function (v) { st.N = Number(v); update(); } });
      const sT = slider(gG, { label: 'Delay T (e<sup>−sT</sup>)', min: 0, max: 2, step: 0.01, value: st.delay, unit: 's', key: 'delay',
        onInput: function (v) { st.delay = v; update(); } });

      const gF = ui.group(sp.side, 'Factors');
      const SLOT_OPTS = [{ value: 'off', label: 'Off' }, { value: 'pole', label: 'Real pole 1/(1 + s/ω)' },
        { value: 'zero', label: 'Real zero (1 + s/ω)' }, { value: 'rhpzero', label: 'RHP zero (1 − s/ω)' },
        { value: 'quad', label: 'Complex pole pair' }];
      const slots = [];
      for (let i = 0; i < 4; i++) {
        const box = ui.el('div', { 'class': 'bp-slot' }, gF);
        const k = i + 1;
        const sel = ui.select(box, { label: 'Factor ' + k, key: 'slot' + k + 'type', value: st.factors[i].type, options: SLOT_OPTS,
          onChange: function (v) { st.factors[i].type = v; update(); } });
        const sw = slider(box, { label: 'Corner ω', min: 0.01, max: 1000, scale: 'log', value: st.factors[i].w, unit: 'rad/s', key: 'slot' + k + 'w',
          onInput: function (v) { st.factors[i].w = v; update(); } });
        const sz = slider(box, { label: 'Damping ratio ζ', min: 0.05, max: 1, step: 0.01, value: st.factors[i].zeta, key: 'slot' + k + 'zeta',
          onInput: function (v) { st.factors[i].zeta = v; update(); } });
        slots.push({ sel: sel, sw: sw, sz: sz, lastType: null });
      }

      const gS = ui.group(sp.side, 'Show');
      const rowC = ui.row(gS);
      const cE = ui.checkbox(rowC, { label: 'Exact', checked: st.showExact, key: 'showExact', onChange: function (v) { st.showExact = v; update(); } });
      const cA = ui.checkbox(rowC, { label: 'Asymptotes', checked: st.showAsym, key: 'showAsym', onChange: function (v) { st.showAsym = v; update(); } });
      const cF = ui.checkbox(rowC, { label: 'Individual factors', checked: st.showFactors, key: 'showFactors', onChange: function (v) { st.showFactors = v; update(); } });
      const cM = ui.checkbox(rowC, { label: 'Margins', checked: st.showMargins, key: 'showMargins', onChange: function (v) { st.showMargins = v; update(); } });

      const rowR = ui.row(sp.side);
      ui.button(rowR, { label: 'Reset', key: 'reset', title: 'Preset A, K = 1, no delay, default display',
        onClick: function () { hoverW = null; setState(JSON.parse(JSON.stringify(DEFAULTS))); } });
      const tag = ui.tag(rowR, { text: '', key: 'stability' });

      const stats = ui.stats(sp.side);
      const tType = ui.stat(stats, { label: 'system type', key: 'type', title: 'Number of integrators: max(N, 0)' });
      const tErr = ui.stat(stats, { label: 'error constant', key: 'errConst', title: 'Kp = G(0) (type 0), Kv = lim sG(s) (type 1), Ka = lim s²G(s) (type 2)' });
      const tSL = ui.stat(stats, { label: 'low-' + lab('ω') + ' slope', key: 'slopeLow', title: '−20N dB/decade' });
      const tSH = ui.stat(stats, { label: 'high-' + lab('ω') + ' slope', key: 'slopeHigh', title: '−20 × (N + poles + 2·pairs − zeros) dB/decade' });
      const tWgc = ui.stat(stats, { label: 'gain crossover ' + lab('ω<sub>gc</sub>'), key: 'wgc', title: '|G(jω)| = 1 (0 dB)' });
      const tPM = ui.stat(stats, { label: 'phase margin', key: 'pm', title: 'PM = 180° + ∠G(jω_gc)' });
      const tWpc = ui.stat(stats, { label: 'phase crossover ' + lab('ω<sub>pc</sub>'), key: 'wpc', title: '∠G(jω) = −180° (− k·360°)' });
      const tGM = ui.stat(stats, { label: 'gain margin', key: 'gm', title: 'GM = −20 log|G(jω_pc)| dB; the factor by which K may grow' });
      const tMr = ui.stat(stats, { label: 'resonant peak ' + lab('M<sub>r</sub>'), key: 'Mr', title: 'Mr = 1/(2ζ√(1 − ζ²)) at ωr = ωn√(1 − 2ζ²), each complex pair with ζ < 0.707' });
      tMr.el.classList.add('bp-wide');

      ui.hints(api.root, [
        'Each real pole bends the magnitude asymptote down by 20 dB/decade at its corner and adds −90° of phase in the end; each zero does the opposite.',
        'Raise K: the magnitude curve slides up while the phase curve does not move — so ω<sub>gc</sub> moves right and the phase margin shrinks. In preset A the loop goes unstable when K passes 3 (GM = 9.54 dB = 20 log 3).',
        'At a real corner the exact curve is 3 dB away from the asymptote and the phase is exactly ∓45°.',
        'Change a zero to “RHP zero”: the magnitude is unchanged but the phase now falls instead of rising — that is non-minimum phase.',
        'Add delay: no change in magnitude, but the phase falls without limit and eats the phase margin.'
      ]);

      // ---------------------------------------------------------------- state helpers
      function sys() { return { K: st.K, N: st.N, factors: st.factors, delay: st.delay }; }
      function same(a, b) { return Math.abs(a - b) <= 1e-9 * Math.max(1, Math.abs(a), Math.abs(b)); }
      function matchPreset() {
        const q = PRESETS.find(function (p) {
          const ps = presetState(p.value);
          if (!same(ps.K, st.K) || ps.N !== st.N || !same(ps.delay, st.delay)) return false;
          for (let i = 0; i < 4; i++) {
            const a = ps.factors[i], b = st.factors[i];
            if (a.type !== b.type) return false;
            if (a.type === 'off') continue;
            if (!same(a.w, b.w)) return false;
            if (a.type === 'quad' && !same(a.zeta, b.zeta)) return false;
          }
          return true;
        });
        return q ? q.value : 'custom';
      }
      // copy a (partial) state into st and the controls, silently
      function applyState(s) {
        if (s.K != null && isFinite(Number(s.K))) { sK.set(M.clamp(Number(s.K), 0.01, 1000)); st.K = sK.get(); }
        if (s.N != null && isFinite(Number(s.N))) {
          const n = Math.round(Number(s.N));
          if (n >= -1 && n <= 2) st.N = n;
          segN.set(String(st.N));
        }
        if (s.delay != null && isFinite(Number(s.delay))) { sT.set(M.clamp(Number(s.delay), 0, 2)); st.delay = sT.get(); }
        if (Array.isArray(s.factors)) {
          for (let i = 0; i < 4; i++) {
            const f = s.factors[i], slot = st.factors[i];
            if (!f) { slot.type = 'off'; continue; }
            if (f.type != null) slot.type = (TYPES.indexOf(f.type) >= 0) ? f.type : 'off';   // no type: keep the slot's type
            if (f.w != null && isFinite(Number(f.w))) slot.w = M.clamp(Number(f.w), 0.01, 1000);
            if (f.zeta != null && isFinite(Number(f.zeta))) slot.zeta = M.clamp(Number(f.zeta), 0.05, 1);
          }
        }
        ['showExact', 'showAsym', 'showFactors', 'showMargins'].forEach(function (k) { if (s[k] != null) st[k] = !!s[k]; });
        syncControls();
      }
      function syncControls() {
        sK.set(st.K); segN.set(String(st.N)); sT.set(st.delay);
        slots.forEach(function (sl, i) {
          const f = st.factors[i];
          sl.sel.set(f.type); sl.sw.set(f.w); sl.sz.set(f.zeta);
        });
        cE.set(st.showExact); cA.set(st.showAsym); cF.set(st.showFactors); cM.set(st.showMargins);
      }
      function syncSlots() {
        slots.forEach(function (sl, i) {
          const t = st.factors[i].type;
          if (t === sl.lastType) return;
          sl.lastType = t;
          sl.sw.el.style.display = t === 'off' ? 'none' : '';
          sl.sz.el.style.display = t === 'quad' ? '' : 'none';
          sl.sw.setLabel(t === 'quad' ? 'Natural frequency ω<sub>n</sub>' : 'Corner frequency ω');
        });
      }

      // ---------------------------------------------------------------- computation (cached per state)
      function key() { return JSON.stringify([st.K, st.N, st.delay, st.factors]); }
      function compute() {
        const S = sys();
        const mg = model.margins(S);
        const fr = model.freqRange(S, mg);
        const fs = activeFactors(S);
        const lo = Math.log10(fr.wmin), hi = Math.log10(fr.wmax);
        const NP = 720;
        const set = [];
        for (let i = 0; i <= NP; i++) set.push(Math.pow(10, lo + (hi - lo) * i / NP));
        const extra = [];
        fs.forEach(function (f) {
          if (f.type === 'quad') extra.push(f.w, f.w * Math.pow(10, -f.zeta), f.w * Math.pow(10, f.zeta));
          else extra.push(f.w, f.w / 10, f.w * 10);
        });
        model.resonance(S).forEach(function (r) { extra.push(r.wr); });
        [mg.wgc, mg.wpc].forEach(function (w) { if (isFinite(w)) extra.push(w); });
        extra.forEach(function (w) { if (w > fr.wmin && w < fr.wmax) set.push(w); });
        set.sort(function (a, b) { return a - b; });
        const ws = [];
        set.forEach(function (w) { if (!ws.length || w > ws[ws.length - 1] * (1 + 1e-12)) ws.push(w); });
        const ns = norm(S);
        const em = ws.map(function (w) { return magOf(ns, w); });
        const ep = ws.map(function (w) { return phaseOf(ns, w); });
        const am = ws.map(function (w) { return asymMagOf(ns, w); });
        const ap = ws.map(function (w) { return asymPhaseOf(ns, w); });
        // magnitude range: exact and asymptote, 0 dB included, multiples of 20 dB, at least 80 dB tall
        let mlo = 0, mhi = 0;
        em.concat(am).forEach(function (v) { if (isFinite(v)) { if (v < mlo) mlo = v; if (v > mhi) mhi = v; } });
        mlo = Math.floor(mlo / 20 - 1e-9) * 20; mhi = Math.ceil(mhi / 20 + 1e-9) * 20;
        let alt = 0;
        while (mhi - mlo < 80) { if (alt++ % 2 === 0) mhi += 20; else mlo -= 20; }
        // phase range: exact and asymptote (−180° included when a margin exists), multiples of 45° (90° above 360°)
        let plo = Infinity, phi = -Infinity;
        ep.concat(ap).forEach(function (v) { if (isFinite(v)) { if (v < plo) plo = Math.max(v, -720); if (v > phi) phi = v; } });
        if (isFinite(mg.pmDeg) || isFinite(mg.gmDb)) { plo = Math.min(plo, -180); phi = Math.max(phi, -180); }
        phi = Math.max(phi, plo);
        let unit = 45;
        let pl = Math.floor(plo / unit + 1e-9) * unit, ph = Math.ceil(phi / unit - 1e-9) * unit;
        if (ph - pl > 360) { unit = 90; pl = Math.floor(plo / unit + 1e-9) * unit; ph = Math.ceil(phi / unit - 1e-9) * unit; }
        alt = 0;
        while (ph - pl < 90) { if (alt++ % 2 === 0) ph += 45; else pl -= 45; }
        pl = Math.max(pl, -720);
        // straight-line magnitude segments between distinct corners (for the slope labels)
        const cs = [];
        fs.forEach(function (f) { if (!cs.some(function (c) { return same(c, f.w); })) cs.push(f.w); });
        cs.sort(function (a, b) { return a - b; });
        const bounds = [fr.wmin].concat(cs.filter(function (c) { return c > fr.wmin && c < fr.wmax; })).concat([fr.wmax]);
        const segs = [];
        for (let i = 0; i + 1 < bounds.length; i++) {
          const a = bounds[i], b = bounds[i + 1], mid = Math.sqrt(a * b);
          let n = ns.N;
          fs.forEach(function (f) { if (f.w < mid) n += f.type === 'pole' ? 1 : f.type === 'quad' ? 2 : -1; });
          segs.push({ a: a, b: b, slope: -20 * n + 0, ya: asymMagOf(ns, a), yb: asymMagOf(ns, b), mid: mid,
            ym: asymMagOf(ns, mid), em: magOf(ns, mid) });
        }
        return {
          S: S, ns: ns, mg: mg, fr: fr, fs: fs, ws: ws, em: em, ep: ep, am: am, ap: ap,
          mag: { lo: mlo, hi: mhi }, ph: { lo: pl, hi: ph, unit: unit }, segs: segs,
          cornersIn: cs.filter(function (c) { return c >= fr.wmin && c <= fr.wmax; }),
          atGc: isFinite(mg.wgc) ? phaseOf(ns, mg.wgc) : NaN,
          atPc: isFinite(mg.wpc) ? magOf(ns, mg.wpc) : NaN,
          parts: null, key: key()
        };
      }
      // the individual factors as curves (computed only when shown)
      function parts() {
        if (D.parts) return D.parts;
        const out = [];
        const add = function (f, color, label) {
          const mm = [], pp = [];
          D.ws.forEach(function (w) { const r = model.factorResponse(f, w); mm.push(r.magDb); pp.push(r.phaseDeg); });
          out.push({ color: color, label: label, m: mm, p: pp });
        };
        if (!same(st.K, 1)) add({ type: 'gain', K: st.K }, C.muted, 'K = ' + fmt(st.K, 4));
        if (st.N !== 0) add({ type: 'integrator', N: st.N }, C.ink, st.N === -1 ? 's' : st.N === 1 ? '1/s' : '1/s²');
        st.factors.forEach(function (f, i) { if (isActive(f)) add(f, FCOL[i], factorText(f, true)); });
        if (st.delay > 0) add({ type: 'delay', T: st.delay }, C.red, delayText());
        D.parts = out;
        return out;
      }
      const MINUS = '−';
      // e^(−Ts), written e^(−s) for T = 1
      function delayText() { const t = fmt(st.delay, 3); return 'e<sup>' + MINUS + (t === '1' ? '' : t) + 's</sup>'; }
      function factorText(f, standalone) {
        const w = fmt(f.w, 4);
        if (f.type === 'pole') return (standalone ? '1/' : '') + '(1 + s/' + w + ')';
        if (f.type === 'zero') return '(1 + s/' + w + ')';
        if (f.type === 'rhpzero') return '(1 ' + MINUS + ' s/' + w + ')';
        return (standalone ? '1/' : '') + '(1 + ' + fmt(2 * f.zeta, 4) + '(s/' + w + ') + (s/' + w + ')²)';
      }
      function tfText() {
        const num = [], den = [];
        if (!same(st.K, 1)) num.push(fmt(st.K, 4));
        if (st.N === -1) num.push('s');
        if (st.N === 1) den.push('s');
        if (st.N === 2) den.push('s²');
        st.factors.forEach(function (f) {
          if (!isActive(f)) return;
          if (f.type === 'pole' || f.type === 'quad') den.push(factorText(f, false));
          else num.push(factorText(f, false));
        });
        if (st.delay > 0) num.push(delayText());
        const ns = num.length ? num.join(' ') : '1';
        if (!den.length) return 'G(s) = ' + ns;
        return 'G(s) = ' + ns + ' / ' + (den.length > 1 ? '[ ' + den.join(' ') + ' ]' : den[0]);
      }

      // ---------------------------------------------------------------- read-outs
      function slopeTxt(v) { return (v > 0 ? '+' : '') + fmt(v) + ' dB/dec'; }
      function marginColor(v) { return v > 1e-6 ? C.green : v < -1e-6 ? C.red : C.gold; }
      function marginState(v) { return v > 1e-6 ? 'ok' : v < -1e-6 ? 'bad' : 'warn'; }
      function snap(v) { return Math.abs(v) < 1e-9 ? 0 : v; }
      function update() {
        syncSlots();
        if (!D || D.key !== key()) D = compute();
        const S = D.S, mg = D.mg;
        tfLine.innerHTML = tfText();
        selPre.set(matchPreset());

        tType.set(String(Math.max(st.N, 0)));
        const errName = st.N === 2 ? 'a' : st.N === 1 ? 'v' : 'p';
        tErr.set('K<sub>' + errName + '</sub> = ' + (st.N === -1 ? '0' : fmt(st.K, 4)));
        const sl = model.slopes(S);
        tSL.set(slopeTxt(sl.low));
        tSH.set(slopeTxt(sl.high));

        if (isFinite(mg.wgc)) tWgc.set(fmt(mg.wgc, 4) + ' rad/s');
        else tWgc.set(mg.unity ? 'every ω (|G| ≡ 1)' : mg.pmDeg === Infinity ? 'none (|G| &lt; 1)' : 'none (|G| &gt; 1)');
        if (isFinite(mg.pmDeg)) { const pm = snap(mg.pmDeg); tPM.set(fmt(pm, 4) + '°'); tPM.setState(marginState(pm)); }
        else if (mg.pmDeg === Infinity) { tPM.set('∞'); tPM.setState('ok'); }
        else { tPM.set('—'); tPM.setState(''); }
        if (isFinite(mg.wpc)) tWpc.set(fmt(mg.wpc, 4) + ' rad/s');
        else tWpc.set('none');
        if (isFinite(mg.gmDb)) {
          const gm = snap(mg.gmDb);
          const ratio = fmt(Math.pow(10, gm / 20), 4);
          tGM.set(fmt(gm, 4) + ' dB (' + (ratio.indexOf('×') >= 0 ? 'ratio ' : '×') + ratio + ')');
          tGM.setState(marginState(gm));
        } else { tGM.set('∞'); tGM.setState('ok'); }
        const res = model.resonance(S);
        tMr.set(res.length ? res.map(function (r) { return fmt(r.MrDb, 3) + ' dB at ' + fmt(r.wr, 3) + ' rad/s'; }).join('; ') : '—');

        // closed-loop stability
        if (st.delay > 0) {
          const gm = snap(mg.gmDb), pm = snap(mg.pmDeg);
          if (gm === 0 || pm === 0) tag.set('MARGINAL (from margins)', 'warn');
          else if (gm > 0 && pm > 0) tag.set('CLOSED LOOP STABLE (from margins)', 'ok');
          else tag.set('CLOSED LOOP UNSTABLE (from margins)', 'bad');
        } else if (model.closedLoopStable(S)) tag.set('CLOSED LOOP STABLE', 'ok');
        else if (model.closedLoopMarginal(S)) tag.set('MARGINAL', 'warn');
        else tag.set('CLOSED LOOP UNSTABLE', 'bad');

        // legend
        const items = [];
        if (st.showExact) items.push({ color: C.blue, label: 'exact' });
        if (st.showAsym) items.push({ color: C.maroon, label: 'asymptotes (slopes in dB/dec)', dash: true });
        if (st.showMargins && (isFinite(mg.wgc) || isFinite(mg.wpc))) items.push({ color: C.green, label: 'margin &gt; 0' }, { color: C.red, label: 'margin &lt; 0' });
        if (st.showFactors) parts().forEach(function (p) { items.push({ color: p.color, label: p.label, dash: true }); });
        const lk = JSON.stringify(items);
        if (lk !== legendKey) { legend.set(items); legendKey = lk; }
        updateCursor();
        cv.redraw();
      }
      let legendKey = '';
      function updateCursor() {
        if (hoverW == null) { cursorLine.innerHTML = 'Hover over the plots to read ω, |G| and ∠G.'; return; }
        const r = model.response(D.S, hoverW);
        cursorLine.innerHTML = 'ω = <b>' + fmt(hoverW, 3) + '</b> rad/s, |G| = <b>' + fmtFixed(r.magDb, 2) + ' dB</b>, ∠G = <b>' + fmtFixed(r.phaseDeg, 1) + '°</b>';
      }

      // ---------------------------------------------------------------- drawing
      function tickList(lo, hi, steps, px, minPx) {
        let step = steps[steps.length - 1];
        for (let i = 0; i < steps.length; i++) if (steps[i] / (hi - lo) * px >= minPx) { step = steps[i]; break; }
        const out = [];
        for (let v = Math.ceil(lo / step - 1e-9) * step; v <= hi + 1e-9; v += step) out.push(v + 0);
        return out;
      }
      function makePlots(ctx, w, h) {
        const mL = 54, mR = 14;
        const hTop = Math.round((h - 30) / 2);
        const pm = RASim.plot(ctx, { box: { x: 0, y: 0, w: w, h: hTop }, margin: { left: mL, right: mR, top: 10, bottom: 6 },
          xmin: D.fr.wmin, xmax: D.fr.wmax, ymin: D.mag.lo, ymax: D.mag.hi, xlog: true, ylabel: '|G(jω)|  (dB)' });
        const pp = RASim.plot(ctx, { box: { x: 0, y: hTop, w: w, h: h - hTop }, margin: { left: mL, right: mR, top: 10, bottom: 36 },
          xmin: D.fr.wmin, xmax: D.fr.wmax, ymin: D.ph.lo, ymax: D.ph.hi, xlog: true, ylabel: '∠G(jω)  (°)', xlabel: 'ω  (rad/s)' });
        return { pm: pm, pp: pp };
      }
      function clampArr(arr, lo, hi) {
        const span = hi - lo, a = lo - 2 * span, b = hi + 2 * span;
        return arr.map(function (v) { return isFinite(v) ? M.clamp(v, a, b) : NaN; });
      }
      // label "ω" + subscript drawn at (x, y), align 'left' | 'right'; returns its box. dry: only measure.
      function omegaLabel(ctx, r, x, y, sub, align, color, dry) {
        ctx.save();
        ctx.font = '12px sans-serif';
        const w1 = ctx.measureText('ω').width;
        ctx.font = F11;
        const w2 = ctx.measureText(sub).width;
        const tw = w1 + w2 + 1;
        let x0 = align === 'right' ? x - tw : x;
        if (x0 < r.x + 3) x0 = x + 8;                       // no room: flip to the other side of the line
        else if (x0 + tw > r.x + r.w - 3) x0 = x - 8 - tw;
        const box = { x0: x0 - 3, x1: x0 + tw + 3, y0: y - 9, y1: y + 9 };
        if (dry) { ctx.restore(); return box; }
        ctx.fillStyle = LBG;
        ctx.fillRect(x0 - 3, y - 9, tw + 6, 18);
        ctx.fillStyle = color; ctx.textBaseline = 'middle'; ctx.textAlign = 'left';
        ctx.font = '12px sans-serif'; ctx.fillText('ω', x0, y - 1);
        ctx.font = F11; ctx.fillText(sub, x0 + w1 + 1, y + 3);
        ctx.restore();
        return box;
      }
      // Margin arrow, in two steps. placeMargin: a double-headed vertical arrow at pixel x from y1 to y2 and the place
      // of its label: the candidate (left / right × above the upper end, below the lower end, middle) that crosses the
      // fewest curve samples, no other margin line and no other label (boxes); ties go to the earlier candidate.
      // drawMargin draws what placeMargin chose.
      function placeMargin(ctx, p, x, y1, y2, text, color, avoidX, curves, boxes) {
        const r = p.rect;
        const c1 = M.clamp(y1, r.y + 1, r.y + r.h - 1), c2 = M.clamp(y2, r.y + 1, r.y + r.h - 1);
        ctx.save();
        ctx.font = B12;
        const tw = ctx.measureText(text).width;
        ctx.restore();
        const top = Math.min(c1, c2), bot = Math.max(c1, c2);
        const cands = [];
        [top - 12, bot + 12, 0.5 * (top + bot)].forEach(function (ty) {
          [-1, 1].forEach(function (sd) {
            const yy = M.clamp(ty, r.y + 10, r.y + r.h - 10);
            const x0 = sd > 0 ? x + 6 : x - 6 - tw;
            cands.push({ x0: x0, x1: x0 + tw, y0: yy - 9, y1: yy + 9, ty: yy, sd: sd });
          });
        });
        let best = null;
        cands.forEach(function (c) {
          let score = 0;
          if (c.x0 < r.x + 2 || c.x1 > r.x + r.w - 2) score += 1e6;
          (avoidX || []).forEach(function (ax) { if (ax > c.x0 - 4 && ax < c.x1 + 4) score += 1000; });
          if (c.y1 > top && c.y0 < bot && c.x0 < x + 3 && c.x1 > x - 3) score += 1000;
          (boxes || []).forEach(function (b) { if (overlap(c, b, 4)) score += 1000; });
          (curves || []).forEach(function (cu) {
            for (let i = 0; i < cu.x.length; i++) {
              const px = cu.x[i], py = cu.y[i];
              if (px >= c.x0 - 3 && px <= c.x1 + 3 && py >= c.y0 - 2 && py <= c.y1 + 2) score += cu.weight;
            }
          });
          if (!best || score < best.score) { best = c; best.score = score; }
        });
        return { x: x, c1: c1, c2: c2, text: text, color: color, label: best, box: { x0: best.x0 - 3, x1: best.x1 + 3, y0: best.y0, y1: best.y1 } };
      }
      function drawMargin(ctx, q) {
        if (Math.abs(q.c2 - q.c1) > 3) {
          const ym = 0.5 * (q.c1 + q.c2);
          RASim.draw.arrow(ctx, q.x, ym, q.x, q.c1, { color: q.color, width: 1.8, head: 7 });
          RASim.draw.arrow(ctx, q.x, ym, q.x, q.c2, { color: q.color, width: 1.8, head: 7 });
        }
        const b = q.label;
        RASim.draw.text(ctx, q.text, b.sd > 0 ? b.x0 : b.x1, b.ty, { color: q.color, font: B12, align: b.sd > 0 ? 'left' : 'right', bg: LBG });
      }
      function overlap(a, b, pad) { return a.x0 - pad < b.x1 && a.x1 + pad > b.x0 && a.y0 - pad < b.y1 && a.y1 + pad > b.y0; }
      function curvesOf(ex, as) {
        const out = [];
        if (st.showExact) out.push({ v: ex, weight: 3 });
        if (st.showAsym) out.push({ v: as, weight: 1 });
        return out;
      }
      // pixel samples of the drawn curves of one plot (for label placement)
      function curvePx(p, arrs) {
        return arrs.map(function (a) {
          const xs = [], ys = [];
          for (let i = 0; i < D.ws.length; i += 2) { xs.push(p.X(D.ws[i])); ys.push(p.Y(a.v[i])); }
          return { x: xs, y: ys, weight: a.weight };
        });
      }

      cv.draw = function (ctx, w, h) {
        if (!D) return;
        const P = makePlots(ctx, w, h), pm = P.pm, pp = P.pp;
        pm.axes({ xFormat: function () { return ''; }, yTicks: tickList(D.mag.lo, D.mag.hi, [20, 40, 60, 100, 200, 500], pm.rect.h, 22) });
        pp.axes({ yTicks: tickList(D.ph.lo, D.ph.hi, D.ph.unit === 45 ? [45, 90, 180, 360] : [90, 180, 360], pp.rect.h, 20) });
        pm.hline(0, { color: C.ink, width: 1.4 });
        pp.hline(-180, { color: C.ink, width: 1.4 });
        const ws = D.ws;

        if (st.showFactors) {
          parts().forEach(function (q) {
            pm.line(ws, clampArr(q.m, D.mag.lo, D.mag.hi), { color: q.color, width: 1.3, dash: [4, 3] });
            pp.line(ws, clampArr(q.p, D.ph.lo, D.ph.hi), { color: q.color, width: 1.3, dash: [4, 3] });
          });
        }
        if (st.showAsym) {
          pm.line(ws, clampArr(D.am, D.mag.lo, D.mag.hi), { color: C.maroon, width: 1.8, dash: [7, 4] });
          pp.line(ws, clampArr(D.ap, D.ph.lo, D.ph.hi), { color: C.maroon, width: 1.8, dash: [7, 4] });
        }
        if (st.showExact) {
          pm.line(ws, clampArr(D.em, D.mag.lo, D.mag.hi), { color: C.blue, width: 2.5 });
          pp.line(ws, clampArr(D.ep, D.ph.lo, D.ph.hi), { color: C.blue, width: 2.5 });
        }
        // slope labels on asymptote segments at least ~60 px long (placed now, drawn after the margin labels are
        // placed: a slope label that would sit under a margin or ω label is left out)
        const slopeLabels = [];
        if (st.showAsym) {
          D.cornersIn.forEach(function (c) { pm.point(c, asymMagOf(D.ns, c), { color: C.maroon, r: 3.5 }); });
          ctx.save();
          ctx.font = F11;
          D.segs.forEach(function (sg) {
            const x1 = pm.X(sg.a), y1 = pm.Y(sg.ya), x2 = pm.X(sg.b), y2 = pm.Y(sg.yb);
            if (Math.hypot(x2 - x1, y2 - y1) < 60) return;
            const xm = pm.X(sg.mid), ym = pm.Y(sg.ym);
            const s = (sg.slope > 0 ? '+' : '') + fmt(sg.slope);
            // perpendicular offset, on the side away from the exact curve
            const L = Math.hypot(x2 - x1, y2 - y1), nx = (y2 - y1) / L, ny = -(x2 - x1) / L;   // points "up" on screen
            const side = sg.em > sg.ym + 0.05 ? -1 : 1;
            const tw = ctx.measureText(s).width;
            const off = 12 + Math.abs(nx) * tw / 2;
            const tx = xm + nx * off * side, ty = ym + ny * off * side;
            if (ty < pm.rect.y + 7 || ty > pm.rect.y + pm.rect.h - 7 || tx < pm.rect.x + tw / 2 + 2 || tx > pm.rect.x + pm.rect.w - tw / 2 - 2) return;
            slopeLabels.push({ s: s, tx: tx, ty: ty, box: { x0: tx - tw / 2 - 3, x1: tx + tw / 2 + 3, y0: ty - 8, y1: ty + 8 } });
          });
          ctx.restore();
        }
        const mg = D.mg;
        const gcLeft = isFinite(mg.wgc) && (!isFinite(mg.wpc) || mg.wgc <= mg.wpc);
        const inView = function (w) { return isFinite(w) && w >= D.fr.wmin * (1 - 1e-9) && w <= D.fr.wmax * (1 + 1e-9); };
        const showGc = st.showMargins && inView(mg.wgc), showPc = st.showMargins && inView(mg.wpc);
        const xgc = showGc ? pm.X(mg.wgc) : NaN, xpc = showPc ? pm.X(mg.wpc) : NaN;
        // the ω_gc / ω_pc labels at the top of the magnitude plot
        const omGc = showGc ? omegaLabel(ctx, pm.rect, gcLeft ? xgc - 4 : xgc + 4, pm.rect.y + 10, 'gc', gcLeft ? 'right' : 'left', '', true) : null;
        const omPc = showPc ? omegaLabel(ctx, pm.rect, gcLeft ? xpc + 4 : xpc - 4, pm.rect.y + 10, 'pc', gcLeft ? 'left' : 'right', '', true) : null;
        const qPM = showGc ? placeMargin(ctx, pp, xgc, pp.Y(-180), pp.Y(D.atGc), 'PM = ' + fmtFixed(snap(mg.pmDeg), 1) + '°',
          marginColor(mg.pmDeg), showPc ? [xpc] : [], curvePx(pp, curvesOf(D.ep, D.ap))) : null;
        const qGM = showPc ? placeMargin(ctx, pm, xpc, pm.Y(D.atPc), pm.Y(0), 'GM = ' + fmtFixed(snap(mg.gmDb), 1) + ' dB',
          marginColor(mg.gmDb), showGc ? [xgc] : [], curvePx(pm, curvesOf(D.em, D.am)), [omGc, omPc].filter(Boolean)) : null;
        const taken = [omGc, omPc, qGM && qGM.box].filter(Boolean);
        slopeLabels.forEach(function (l) {
          if (taken.some(function (b) { return overlap(l.box, b, 2); })) return;
          RASim.draw.text(ctx, l.s, l.tx, l.ty, { color: C.maroon, font: F11, align: 'center', bg: LBG });
        });
        if (showGc) {
          const col = qPM.color;
          pm.vline(mg.wgc, { color: col, width: 1.2, dash: [5, 4] });
          pp.vline(mg.wgc, { color: col, width: 1.2, dash: [5, 4] });
          pm.point(mg.wgc, 0, { color: col, r: 3.5 });
          drawMargin(ctx, qPM);
          omegaLabel(ctx, pm.rect, gcLeft ? xgc - 4 : xgc + 4, pm.rect.y + 10, 'gc', gcLeft ? 'right' : 'left', col);
        }
        if (showPc) {
          const col = qGM.color;
          pm.vline(mg.wpc, { color: col, width: 1.2, dash: [5, 4] });
          pp.vline(mg.wpc, { color: col, width: 1.2, dash: [5, 4] });
          pp.point(mg.wpc, phaseOf(D.ns, mg.wpc), { color: col, r: 3.5 });
          drawMargin(ctx, qGM);
          omegaLabel(ctx, pm.rect, gcLeft ? xpc + 4 : xpc - 4, pm.rect.y + 10, 'pc', gcLeft ? 'left' : 'right', col);
        }
        if (hoverW != null && hoverW >= D.fr.wmin && hoverW <= D.fr.wmax) {
          const hc = RASim.alpha(C.ink, 0.5);
          pm.vline(hoverW, { color: hc, width: 1 });
          pp.vline(hoverW, { color: hc, width: 1 });
          if (st.showExact) {
            pm.point(hoverW, magOf(D.ns, hoverW), { color: C.blue, r: 3.5 });
            pp.point(hoverW, phaseOf(D.ns, hoverW), { color: C.blue, r: 3.5 });
          }
        }
      };

      cv.onHover({
        move: function (x, y) {
          if (!D) return;
          const P = makePlots(cv.ctx, cv.width, cv.height);
          const inside = P.pm.contains(x, y) || P.pp.contains(x, y);
          // ω rounded to the 3 significant digits the read-out prints, so the printed |G| and ∠G belong to the printed ω
          const nw = inside ? Number(P.pm.invX(x).toPrecision(3)) : null;
          if (nw === hoverW) return;
          hoverW = nw;
          updateCursor();
          cv.redraw();
        },
        leave: function () { if (hoverW == null) return; hoverW = null; updateCursor(); cv.redraw(); }
      });

      function getState() {
        return {
          K: st.K, N: st.N, delay: st.delay,
          factors: st.factors.map(function (f) { return { type: f.type, w: f.w, zeta: f.zeta }; }),
          showExact: st.showExact, showAsym: st.showAsym, showFactors: st.showFactors, showMargins: st.showMargins
        };
      }
      function setState(s) {
        applyState(s || {});
        update();
      }
      update();
      return { getState: getState, setState: setState };
    }
  });
})();
