// root-locus — "Root locus explorer"
// Sits beside notes/src/04_electrical.html, section "Root locus" (B1.3 Control Systems).
//
// SIGN CONVENTIONS AND MODEL
//   Unity NEGATIVE feedback around the open loop K·G(s), G(s) = N(s)/D(s) = Π(s − zᵢ)/Π(s − pⱼ), K ≥ 0.
//   N and D are monic (RASim.math.polyFromRoots). n = number of poles (1…6), m = number of zeros (m ≤ n).
//   Closed-loop poles: roots of the characteristic polynomial D(s) + K·N(s) (RASim.math.polyRoots).
//   Closed loop T(s) = K·N/(D + K·N); its unit-step response comes from RASim.math.tfStep.
//   s = σ + jω: σ to the right, jω up; the left half-plane (σ < 0) is stable. Angles in DEGREES,
//   counter-clockwise from the positive real axis, wrapped to (−180°, 180°].
//   Asymptotes (n > m): n − m rays at (2q + 1)·180°/(n − m), q = 0 … n − m − 1 (reported wrapped and sorted
//   ascending) from the centroid σ_a = (Σ Re p − Σ Re z)/(n − m). n = m: count 0, angles [], centroid NaN.
//   Break points: roots of N·D′ − N′·D = 0 at which K = −D(s)/N(s) is real (|Im K| < 1e-6·|K|) and > 1e-9
//   (stationary points with negative K are not on the locus and are not reported).
//   jω crossings: K(ω) = −D(jω)/N(jω) real and > 0 for ω > 0 (sign changes of Im K, i.e. of
//   Im[D(jω)·conj N(jω)], on log-spaced samples refined by bisection), plus ω = 0 when D(0) + K·N(0) = 0 has K > 0.
//   Departure from a complex pole p_k: θ_d = 180° − Σ_(j≠k) ∠(p_k − p_j) + Σ_i ∠(p_k − z_i);
//   arrival at a complex zero z_k: θ_a = 180° + Σ_j ∠(z_k − p_j) − Σ_(i≠k) ∠(z_k − z_i).
//   θ_d is the direction in which the locus LEAVES the pole; θ_a is the direction FROM the zero towards the
//   incoming locus (the arrow drawn on the canvas points into the zero). A root repeated q times splits the
//   angle: (180° + 360°·l − …)/q, l = 0 … q − 1 (one per copy).
//   Stability (closed loop): unstable when the largest real part > 1e-6·max|pole|, marginally stable when it is
//   within ±1e-6·max|pole| of zero (a repeated pole on the jω-axis counts as unstable), else stable.
//   dominant(roots): the closed-loop pole (pair) in the open left half-plane closest to the jω-axis,
//   ζ = −σ/|s|, ωₙ = |s|; null when no pole lies in the open left half-plane. The panel shows "—" for ζ and ωₙ
//   unless the closed loop is stable (a dominant pole is only meaningful then).
//   Gains are dimensionless, s, σ and ω in rad/s, t in s.
//
// STATE KEYS (getState / setState)
//   poles: [{ re, im } …], zeros: [{ re, im } …]   (every root, both members of each conjugate pair;
//          setState with poles and/or zeros replaces the configuration; an invalid set is refused),
//   K (0.01 … 1000), tool ('move' | 'addPole' | 'addZero' | 'delete'),
//   showAsymptotes, showBreaks, showAngles, showStep (booleans)
//
// STAT KEYS:   n, m, asym, centroid, breaks, crossings, stable, clPoles, zeta, wn
// TAG KEYS:    stability   (STABLE / MARGINALLY STABLE / UNSTABLE)
// MATRIX KEYS: (none)
// DATA-KEY:    tf (G(s) line), charEq (characteristic equation at the current K), status (tool messages),
//              hover (s and K of the locus point under the mouse)
// CANVAS KEYS: splane, step
// CONTROL KEYS: preset, tool, K, showAsymptotes, showBreaks, showAngles, showStep, reset
(function () {
  'use strict';
  const RASim = globalThis.RASim;
  const M = RASim.math;
  const ID = 'root-locus';

  // ---------------------------------------------------------------- model helpers (pure)
  function C(re, im) { return { re: re, im: im || 0 }; }
  function cabs(z) { return Math.hypot(z.re, z.im); }
  function maxAbs(list) { let r = 0; list.forEach(function (z) { r = Math.max(r, cabs(z)); }); return r; }
  function padTo(p, len) { const out = []; for (let i = 0; i < len - p.length; i++) out.push(0); return out.concat(p); }

  function polys(poles, zeros) {
    return { num: M.polyFromRoots(zeros || []), den: M.polyFromRoots(poles || []) };
  }
  // characteristic polynomial D + K·N (length n + 1, not trimmed)
  function charPoly(P, K) {
    const len = P.den.length, num = padTo(P.num, len);
    return P.den.map(function (d, i) { return d + K * num[i]; });
  }
  function closedLoopPoles(poles, zeros, K) {
    return M.polyRoots(charPoly(polys(poles, zeros), K));
  }

  function asymptotes(poles, zeros) {
    const n = poles.length, m = zeros.length, q = n - m;
    if (q <= 0) return { count: 0, anglesDeg: [], centroid: NaN };
    const a = [];
    for (let k = 0; k < q; k++) a.push(M.wrapDeg((2 * k + 1) * 180 / q));
    a.sort(function (x, y) { return x - y; });
    let s = 0;
    poles.forEach(function (p) { s += p.re; });
    zeros.forEach(function (z) { s -= z.re; });
    return { count: q, anglesDeg: a, centroid: s / q };
  }

  function breakPoints(poles, zeros) {
    const P = polys(poles, zeros);
    const Q = M.polyTrim(M.polySub(M.polyMul(P.num, M.polyDeriv(P.den)), M.polyMul(M.polyDeriv(P.num), P.den)));
    if (Q.length < 2) return [];
    const scale = Math.max(1, maxAbs(poles), maxAbs(zeros));
    const found = [];
    M.polyRoots(Q).forEach(function (r) {
      let s = C(r.re, r.im);
      if (Math.abs(s.im) < 1e-7 * Math.max(1, cabs(s))) s.im = 0;
      const Ns = M.polyEval(P.num, s), Ds = M.polyEval(P.den, s);
      if (!(cabs(Ns) > 1e-12 * Math.pow(scale, zeros.length))) return;     // at a (multiple) zero: K = ∞
      const K = M.cdiv(Ds, Ns);
      K.re = -K.re; K.im = -K.im;
      const kabs = cabs(K);
      if (!isFinite(kabs) || !(K.re > 1e-9) || !(Math.abs(K.im) < 1e-6 * kabs)) return;
      found.push({ s: s, K: K.re });
    });
    // merge the members of a cluster (a repeated root of N·D′ − N′·D comes back as a near-real pair)
    const out = [];
    found.forEach(function (b) {
      const tol = 1e-4 * Math.max(1, cabs(b.s));
      const o = out.find(function (c) { return Math.hypot(c.s.re - b.s.re, c.s.im - b.s.im) < tol; });
      if (o) { o.n++; o.sr += b.s.re; o.si += b.s.im; o.k += b.K; } else out.push({ s: b.s, n: 1, sr: b.s.re, si: b.s.im, k: b.K });
    });
    const res = out.map(function (o) {
      const s = C(o.sr / o.n, o.si / o.n);
      if (Math.abs(s.im) < 1e-4 * Math.max(1, cabs(s))) s.im = 0;
      return { s: s, K: o.k / o.n };
    });
    res.sort(function (a, b) { return (b.s.re - a.s.re) || (b.s.im - a.s.im); });
    return res;
  }

  function jwCrossings(poles, zeros) {
    const P = polys(poles, zeros);
    const R = Math.max(1, maxAbs(poles), maxAbs(zeros));
    const out = [];
    // Im K(ω) has the sign of −Im[D(jω)·conj N(jω)]: a polynomial in ω (continuous, no poles)
    function f(w) {
      const d = M.polyEval(P.den, C(0, w)), nn = M.polyEval(P.num, C(0, w));
      return d.im * nn.re - d.re * nn.im;
    }
    function kOf(w) {
      const d = M.polyEval(P.den, C(0, w)), nn = M.polyEval(P.num, C(0, w));
      const K = M.cdiv(d, nn);
      return C(-K.re, -K.im);
    }
    const ws = M.logspace(1e-6 * R, 1e3 * R, 3001);
    // f identically zero (the whole jω-axis maps to real K): no isolated crossings
    let degenerate = true;
    for (let i = 0; i < ws.length && degenerate; i += 50) {
      const d = M.polyEval(P.den, C(0, ws[i])), nn = M.polyEval(P.num, C(0, ws[i]));
      if (Math.abs(f(ws[i])) > 1e-12 * cabs(d) * cabs(nn)) degenerate = false;
    }
    if (!degenerate) {
      let f0 = f(ws[0]);
      for (let i = 1; i < ws.length; i++) {
        const f1 = f(ws[i]);
        if (f0 !== 0 && f1 !== 0 && (f0 < 0) !== (f1 < 0)) {
          const w = M.bisect(f, ws[i - 1], ws[i], 1e-15 * ws[i]);
          if (isFinite(w)) {
            const K = kOf(w), ka = cabs(K);
            if (isFinite(ka) && K.re > 1e-9 && Math.abs(K.im) <= 1e-6 * ka && K.re < 1e12) out.push({ K: K.re, w: w });
          }
        } else if (f1 === 0) {
          const K = kOf(ws[i]), ka = cabs(K);
          if (isFinite(ka) && K.re > 1e-9 && Math.abs(K.im) <= 1e-6 * ka && K.re < 1e12) out.push({ K: K.re, w: ws[i] });
        }
        f0 = f1;
      }
    }
    // ω = 0: a real closed-loop pole passing through the origin
    const D0 = P.den[P.den.length - 1], N0 = P.num[P.num.length - 1];
    if (N0 !== 0) {
      const K0 = -D0 / N0;
      if (K0 > 1e-9) out.push({ K: K0, w: 0 });
    }
    out.sort(function (a, b) { return a.K - b.K || a.w - b.w; });
    return out;
  }

  // 'stable' | 'marginal' | 'unstable' for a list of closed-loop poles
  function stability(roots) {
    if (!roots.length) return 'stable';
    const tol = 1e-6 * Math.max(maxAbs(roots), 1e-9);
    let mx = -Infinity;
    roots.forEach(function (r) { if (r.re > mx) mx = r.re; });
    if (mx > tol) return 'unstable';
    if (mx < -tol) return 'stable';
    // poles on the axis: a repeated one grows like t·sin or t
    const onAxis = roots.filter(function (r) { return Math.abs(r.re) <= tol; });
    for (let i = 0; i < onAxis.length; i++) {
      for (let j = i + 1; j < onAxis.length; j++) {
        if (Math.abs(onAxis[i].im - onAxis[j].im) <= 1e-5 * Math.max(1, Math.abs(onAxis[i].im))) return 'unstable';
      }
    }
    return 'marginal';
  }

  function stableRanges(poles, zeros) {
    const ks = [];
    jwCrossings(poles, zeros).forEach(function (c) {
      if (!ks.length || Math.abs(c.K - ks[ks.length - 1]) > 1e-9 * c.K) ks.push(c.K);
    });
    const bounds = [0].concat(ks, [Infinity]);
    const out = [];
    for (let i = 0; i + 1 < bounds.length; i++) {
      const a = bounds[i], b = bounds[i + 1];
      const Kt = b === Infinity ? (a > 0 ? 2 * a : 1) : (a === 0 ? b / 2 : Math.sqrt(a * b));
      if (stability(closedLoopPoles(poles, zeros, Kt)) === 'stable') out.push([a, b]);
    }
    return out;
  }

  // angle condition at a complex root; own: the list holding the root, other: the other list;
  // sgnOwn / sgnOther: −1/+1 (departure: −Σ other poles + Σ zeros; arrival: +Σ poles − Σ other zeros)
  function rootAngles(own, other, sgnOwn, sgnOther) {
    const out = [];
    const done = own.map(function () { return false; });
    own.forEach(function (r, k) {
      if (!(r.im > 0) || done[k]) return;
      const same = [];
      let sum = 180, bad = false;
      own.forEach(function (q, j) {
        if (q.re === r.re && q.im === r.im) { same.push(j); return; }
        sum += sgnOwn * M.deg(Math.atan2(r.im - q.im, r.re - q.re));
      });
      other.forEach(function (q) {
        if (q.re === r.re && q.im === r.im) bad = true;              // cancelled by an equal root: no motion
        sum += sgnOther * M.deg(Math.atan2(r.im - q.im, r.re - q.re));
      });
      const mult = same.length;
      same.forEach(function (j, l) {
        done[j] = true;
        out.push({ at: C(r.re, r.im), deg: bad ? NaN : M.wrapDeg((sum + 360 * l) / mult) });
      });
    });
    return out;
  }
  function departureAngles(poles, zeros) { return rootAngles(poles, zeros, -1, +1); }
  function arrivalAngles(poles, zeros) { return rootAngles(zeros, poles, -1, +1); }

  function kAt(poles, zeros, s) {
    const P = polys(poles, zeros);
    return cabs(M.polyEval(P.den, s)) / cabs(M.polyEval(P.num, s));
  }

  function dominant(roots) {
    if (!roots || !roots.length) return null;
    const tol = 1e-6 * Math.max(maxAbs(roots), 1e-9);
    let best = null;
    roots.forEach(function (r) { if (r.re < -tol && (!best || r.re > best.re)) best = r; });
    if (!best) return null;
    const wn = cabs(best);
    return { zeta: -best.re / wn, wn: wn };
  }

  // plotting window of the s-plane (fitted to poles, zeros, origin, centroid, break points and jω crossings)
  function view(poles, zeros) {
    const pts = [C(0, 0)].concat(poles, zeros);
    const as = asymptotes(poles, zeros);
    if (isFinite(as.centroid)) pts.push(C(as.centroid, 0));
    breakPoints(poles, zeros).forEach(function (b) { pts.push(b.s); });
    jwCrossings(poles, zeros).forEach(function (c) { pts.push(C(0, c.w)); });
    let x0 = Infinity, x1 = -Infinity, Y = 0;
    pts.forEach(function (p) { x0 = Math.min(x0, p.re); x1 = Math.max(x1, p.re); Y = Math.max(Y, Math.abs(p.im)); });
    const S = Math.max(x1 - x0, 2 * Y);
    const mg = Math.max(0.35 * S, 2);
    return { xmin: x0 - mg, xmax: x1 + mg, ymin: -(Y + mg), ymax: Y + mg };
  }

  // ---- locus tracing: warm-started Aberth iteration (core polyRoots as the fallback) ----
  function evalPD(a, zr, zi) {
    let pr = a[0], pi = 0, dr = 0, di = 0;
    for (let i = 1; i < a.length; i++) {
      const ndr = dr * zr - di * zi + pr;
      di = dr * zi + di * zr + pi;
      dr = ndr;
      const npr = pr * zr - pi * zi + a[i];
      pi = pr * zi + pi * zr;
      pr = npr;
    }
    return [pr, pi, dr, di];
  }
  // conjugate-symmetric clean-up, the same rules as the core's polyRoots
  function symmetrize(roots) {
    roots.forEach(function (r) {
      if (Math.abs(r.im) < 1e-9 * Math.max(1, cabs(r))) r.im = 0;
      if (r.re === 0) r.re = 0;
    });
    const used = roots.map(function () { return false; });
    for (;;) {
      let u = -1, bi = 0;
      for (let q = 0; q < roots.length; q++) if (!used[q] && roots[q].im !== 0 && Math.abs(roots[q].im) > bi) { bi = Math.abs(roots[q].im); u = q; }
      if (u < 0) break;
      used[u] = true;
      let best = -1, bd = Infinity;
      for (let v = 0; v < roots.length; v++) {
        if (used[v]) continue;
        const d = Math.hypot(roots[u].re - roots[v].re, roots[u].im + roots[v].im);
        if (d < bd) { bd = d; best = v; }
      }
      if (best >= 0 && bd < bi) {
        const sg = roots[u].im > 0 ? 1 : -1;
        const re = (roots[u].re + roots[best].re) / 2;
        let im = (Math.abs(roots[u].im) + Math.abs(roots[best].im)) / 2;
        if (im < 1e-9 * Math.max(1, Math.hypot(re, im))) im = 0;
        roots[u] = C(re, sg * im); roots[best] = C(re, -sg * im);
        used[best] = true;
      } else roots[u] = C(roots[u].re, 0);
    }
    return roots;
  }
  function solveWarm(c, guess) {
    const n = c.length - 1;
    const a = c.map(function (v) { return v / c[0]; });
    const scale = Math.max(1, maxAbs(guess));
    const zr = [], zi = [];
    guess.forEach(function (g, m) {
      zr.push(g.re); zi.push(g.im + scale * 1e-7 * (m + 1) * (m % 2 ? -1 : 1));   // break the real-axis symmetry
    });
    let conv = false;
    for (let it = 0; it < 60; it++) {
      let maxStep = 0;
      for (let m = 0; m < n; m++) {
        const e = evalPD(a, zr[m], zi[m]);
        if (e[0] === 0 && e[1] === 0) continue;
        const dd0 = e[2] * e[2] + e[3] * e[3];
        if (!(dd0 > 0)) { zr[m] += 1e-6 * scale; continue; }
        const wr = (e[0] * e[2] + e[1] * e[3]) / dd0, wi = (e[1] * e[2] - e[0] * e[3]) / dd0;   // p/p′
        let sr = 0, si = 0;
        for (let l = 0; l < n; l++) {
          if (l === m) continue;
          let dr = zr[m] - zr[l], di = zi[m] - zi[l], dd = dr * dr + di * di;
          if (dd === 0) { dr = 1e-12; di = 0; dd = 1e-24; }
          sr += dr / dd; si -= di / dd;
        }
        const denr = 1 - (wr * sr - wi * si), deni = -(wr * si + wi * sr);
        const dn = denr * denr + deni * deni;
        let cr = wr, ci = wi;
        if (dn > 0) { cr = (wr * denr + wi * deni) / dn; ci = (wi * denr - wr * deni) / dn; }
        if (!isFinite(cr) || !isFinite(ci)) return null;
        zr[m] -= cr; zi[m] -= ci;
        const st = Math.hypot(cr, ci) / Math.max(1, Math.hypot(zr[m], zi[m]));
        if (st > maxStep) maxStep = st;
      }
      if (maxStep < 1e-14) { conv = true; break; }
    }
    if (!conv) {
      // stalled at round-off (a near-multiple root): accept when the backward residual is tiny
      for (let m = 0; m < n; m++) {
        const e = evalPD(a, zr[m], zi[m]);
        const r = Math.hypot(zr[m], zi[m]);
        let mag = 0, pw = 1;
        for (let i = n; i >= 0; i--) { mag += Math.abs(a[i]) * pw; pw *= r; }
        if (!(Math.hypot(e[0], e[1]) <= 1e-12 * mag)) return null;
      }
    }
    // Vieta check: the sum of the roots is −a₁ (catches two iterates on one root)
    let sre = 0, sab = 0;
    for (let m = 0; m < n; m++) { sre += zr[m]; sab += Math.hypot(zr[m], zi[m]); }
    if (Math.abs(sre + a[1]) > 1e-8 * (sab + Math.abs(a[1]) + 1)) return null;
    const out = [];
    for (let m = 0; m < n; m++) out.push(C(zr[m], zi[m]));
    return symmetrize(out);
  }

  const PERMS = {};
  function perms(n) {
    if (PERMS[n]) return PERMS[n];
    const out = [];
    (function rec(cur, used) {
      if (cur.length === n) { out.push(cur.slice()); return; }
      for (let i = 0; i < n; i++) if (!used[i]) { used[i] = true; cur.push(i); rec(cur, used); cur.pop(); used[i] = false; }
    })([], []);
    PERMS[n] = out;
    return out;
  }
  // assignment of the new roots to the previous ones: nearest neighbour when that is a bijection (then it is
  // optimal), otherwise the permutation with the least sum of squared distances
  function matchRoots(prev, cur) {
    const n = prev.length;
    const nearest = [], taken = [];
    let ok = true;
    for (let i = 0; i < n; i++) {
      let bj = 0, bd = Infinity;
      for (let j = 0; j < n; j++) {
        const d = (prev[i].re - cur[j].re) * (prev[i].re - cur[j].re) + (prev[i].im - cur[j].im) * (prev[i].im - cur[j].im);
        if (d < bd) { bd = d; bj = j; }
      }
      if (taken[bj]) ok = false;
      taken[bj] = true; nearest.push(bj);
    }
    if (ok) return nearest;
    const D = [];
    for (let i = 0; i < n; i++) {
      D.push([]);
      for (let j = 0; j < n; j++) D[i].push((prev[i].re - cur[j].re) * (prev[i].re - cur[j].re) + (prev[i].im - cur[j].im) * (prev[i].im - cur[j].im));
    }
    let best = null, bs = Infinity;
    perms(n).forEach(function (p) {
      let s = 0;
      for (let i = 0; i < n && s < bs; i++) s += D[i][p[i]];
      if (s < bs) { bs = s; best = p; }
    });
    return best;
  }

  // opts: { view, maxMove (default: 1/200 of the larger view span), kMax (1e10), maxSolves (20000) }
  function locus(poles, zeros, opts) {
    opts = opts || {};
    const P = polys(poles, zeros);
    const n = poles.length, m = zeros.length;
    const v = opts.view || view(poles, zeros);
    const half = Math.max(v.xmax - v.xmin, v.ymax - v.ymin) / 2;
    const cx = (v.xmin + v.xmax) / 2;
    const maxMove = opts.maxMove || half / 100;
    const rOut = 2.5 * half;
    const kMax = opts.kMax || 1e10;
    const maxSolves = opts.maxSolves || 20000;
    const tolZ = 0.2 * maxMove;
    const branches = poles.map(function (p) { return [{ re: p.re, im: p.im, K: 0 }]; });
    let prev = poles.map(function (p) { return C(p.re, p.im); });
    let solves = 0;

    function solve(K) {
      solves++;
      const c = charPoly(P, K);
      return solveWarm(c, prev) || M.polyRoots(c);
    }
    function inside(z) { return Math.hypot(z.re - cx, z.im) < rOut; }
    function step(K) {
      const r = solve(K);
      const pm = matchRoots(prev, r);
      let d = 0;
      for (let i = 0; i < n; i++) {
        const q = r[pm[i]];
        if (inside(prev[i]) || inside(q)) d = Math.max(d, Math.hypot(q.re - prev[i].re, q.im - prev[i].im));
      }
      return { roots: pm.map(function (j) { return r[j]; }), d: d, K: K };
    }
    function accept(s) {
      s.roots.forEach(function (q, i) { branches[i].push({ re: q.re, im: q.im, K: s.K }); });
      prev = s.roots;
    }
    function done() {
      let nz = 0, no = 0;
      prev.forEach(function (q) {
        if (zeros.some(function (z) { return Math.hypot(q.re - z.re, q.im - z.im) < tolZ; })) nz++;
        else if (!inside(q)) no++;
      });
      return nz + no === n && no <= n - m;
    }

    // first step from K = 0
    let K = Math.max(1e-12, Math.pow(half, n - m)) * 1e-3;
    let s = step(K);
    while (s.d > maxMove && K > 1e-14) { K *= 0.1; s = step(K); }
    accept(s);
    let lg = Math.log(1.5);
    while (!done() && K < kMax && solves < maxSolves) {
      const Kn = K * Math.exp(lg);
      s = step(Kn);
      if (s.d > maxMove && lg > 1e-10) {
        lg *= M.clamp(0.8 * maxMove / s.d, 0.05, 0.5);
        continue;
      }
      accept(s);
      K = Kn;
      if (s.d < 0.4 * maxMove) lg = Math.min(lg * 1.6, Math.log(2));
    }
    Object.defineProperty(branches, 'solves', { value: solves, enumerable: false });
    return branches;
  }

  // closed-loop unit-step response T(s) = K·N/(D + K·N) with the time window of the specification
  function stepWindow(roots) {
    const kind = stability(roots);
    if (kind === 'stable') {
      let smin = Infinity;
      roots.forEach(function (r) { smin = Math.min(smin, Math.abs(r.re)); });
      return M.clamp(6 / smin, 2, 60);
    }
    const tol = 1e-6 * Math.max(maxAbs(roots), 1e-9);
    let wmax = 0;
    roots.forEach(function (r) { if (r.re >= -tol) wmax = Math.max(wmax, Math.abs(r.im)); });
    return wmax > 0 ? M.clamp(5 * 2 * Math.PI / wmax, 2, 30) : 10;
  }
  function stepResponse(poles, zeros, K, nPts) {
    const P = polys(poles, zeros);
    const cl = charPoly(P, K);
    const roots = M.polyRoots(cl);
    const tEnd = stepWindow(roots);
    const r = M.tfStep(M.polyScale(P.num, K), cl, tEnd, nPts || 400);
    const kind = stability(roots);
    const fin = kind === 'stable' ? K * P.num[P.num.length - 1] / cl[cl.length - 1] : NaN;
    return { t: r.t, y: r.y, tEnd: tEnd, final: fin, kind: kind };
  }

  const model = {
    polys: polys, closedLoopPoles: closedLoopPoles, asymptotes: asymptotes, breakPoints: breakPoints,
    jwCrossings: jwCrossings, stableRanges: stableRanges, departureAngles: departureAngles, arrivalAngles: arrivalAngles,
    locus: locus, kAt: kAt, dominant: dominant,
    // extras used by the panel
    charPoly: function (poles, zeros, K) { return charPoly(polys(poles, zeros), K); },
    stability: stability, view: view, stepWindow: stepWindow, stepResponse: stepResponse
  };

  // ---------------------------------------------------------------- UI constants
  // a root item is { re, im } with im ≥ 0; im > 0 stands for the conjugate pair re ± j·im
  const PRESETS = [
    { value: 'p2', label: '1 / [s(s+2)]', poles: [[0, 0], [-2, 0]], zeros: [] },
    { value: 'p3', label: '1 / [s(s+1)(s+2)]', poles: [[0, 0], [-1, 0], [-2, 0]], zeros: [] },
    { value: 'p2z', label: '(s+3) / [s(s+2)]', poles: [[0, 0], [-2, 0]], zeros: [[-3, 0]] },
    { value: 'p3c', label: '1 / [s(s²+4s+8)]', poles: [[0, 0], [-2, 2]], zeros: [] },
    { value: 'p00z', label: '(s+1) / s²', poles: [[0, 0], [0, 0]], zeros: [[-1, 0]] },
    { value: 'p4', label: '1 / [(s+1)(s+2)(s+3)(s+4)]', poles: [[-1, 0], [-2, 0], [-3, 0], [-4, 0]], zeros: [] },
    { value: 'p3zz', label: '(s²+2s+2) / [s(s+1)(s+2)]', poles: [[0, 0], [-1, 0], [-2, 0]], zeros: [[-1, 1]] }
  ];
  const DEFAULT_PRESET = 'p3';
  const DEFAULTS = { K: 1, tool: 'move', showAsymptotes: true, showBreaks: true, showAngles: true, showStep: true };
  const KMIN = 0.01, KMAX = 1000, MAXP = 6, IMMIN = 0.1;
  const TOOLS = ['move', 'addPole', 'addZero', 'delete'];
  const HIT_ROOT = 12, HIT_LOCUS = 10, REAL_SNAP = 8;
  const SUP = { '0': '⁰', '1': '¹', '2': '²', '3': '³', '4': '⁴', '5': '⁵', '6': '⁶', '7': '⁷', '8': '⁸', '9': '⁹' };

  function snap(v) { const s = Math.round(v * 10) / 10; return s === 0 ? 0 : s; }
  function itemsOf(list) { return list.map(function (q) { return { re: q[0], im: q[1] }; }); }
  function expand(items) {
    const out = [];
    items.forEach(function (q) { out.push(C(q.re, q.im)); if (q.im > 0) out.push(C(q.re, -q.im)); });
    return out;
  }
  function countOf(items) { let c = 0; items.forEach(function (q) { c += q.im > 0 ? 2 : 1; }); return c; }
  function itemKey(items) {
    return items.map(function (q) { return q.re + ',' + q.im; }).sort().join(';');
  }
  // full root list (both members of each pair) -> items, or null when it is not conjugate-symmetric
  function toItems(list) {
    if (!Array.isArray(list)) return null;
    const pos = [], neg = [], out = [];
    for (let i = 0; i < list.length; i++) {
      const q = list[i];
      if (!q || typeof q !== 'object') return null;
      const re = Number(q.re), im = Number(q.im || 0);
      if (!isFinite(re) || !isFinite(im)) return null;
      if (Math.abs(im) <= 1e-12) out.push({ re: re === 0 ? 0 : re, im: 0 });
      else if (im > 0) pos.push({ re: re, im: im }); else neg.push({ re: re, im: -im });
    }
    if (pos.length !== neg.length) return null;
    const used = neg.map(function () { return false; });
    for (let i = 0; i < pos.length; i++) {
      const j = neg.findIndex(function (q, k) { return !used[k] && Math.abs(q.re - pos[i].re) <= 1e-9 * Math.max(1, Math.abs(q.re)) && Math.abs(q.im - pos[i].im) <= 1e-9 * Math.max(1, q.im); });
      if (j < 0) return null;
      used[j] = true;
      out.push(pos[i]);
    }
    return out;
  }
  function supify(html) {
    return html.replace(/<sup>(\d+)<\/sup>/g, function (m0, d) { return d.split('').map(function (ch) { return SUP[ch]; }).join(''); });
  }
  function polyText(c) { return supify(RASim.fmtPoly(c, 's', 4)); }
  function nTerms(c) { return M.polyTrim(c).filter(function (v) { return v !== 0; }).length; }

  RASim.register(ID, {
    title: 'Root locus explorer',
    model: model,
    mount: function (api) {
      const ui = RASim.ui, Cl = RASim.colors;
      const fmt = RASim.fmt;
      const SC = '.sim[data-sim="' + ID + '"]';
      const F11 = '11px sans-serif', F12 = '12px sans-serif', B12 = 'bold 12px sans-serif';
      const LBG = RASim.alpha('#ffffff', 0.88);
      const COL = { locus: Cl.blue, pole: Cl.red, zero: Cl.blue, cl: Cl.orange, asym: Cl.muted, brk: Cl.purple,
        cross: Cl.gold, ang: Cl.green, hover: Cl.maroon, rhp: RASim.alpha(Cl.red, 0.07), step: Cl.orange };

      RASim.css(ID,
        SC + ' .rl-nt { text-transform: none; letter-spacing: 0; font-size: 11.5px; }\n' +
        SC + ' .rl-wide { grid-column: 1 / -1; }\n' +
        SC + ' .rl-stats { grid-template-columns: repeat(2, minmax(0, 1fr)); }\n' +
        SC + ' .rl-stats .sim-stat-value { font-size: 14px; }\n' +
        SC + ' .rl-eq { font-size: 13.5px; color: ' + Cl.ink + '; margin: 0; line-height: 1.5; overflow-wrap: anywhere; font-variant-numeric: tabular-nums; }\n' +
        SC + ' .rl-lab { color: ' + Cl.muted + '; font-size: 12px; }\n' +
        SC + ' .rl-line { min-height: 1.45em; overflow-wrap: anywhere; }\n' +
        SC + ' .rl-refused { color: ' + Cl.red + '; }\n' +
        SC + ' .rl-mk { font-weight: bold; font-size: 14px; line-height: 1; }\n' +
        SC + ' .rl-nosw .sim-swatch { display: none; }\n' +
        SC + ' .rl-top { margin-bottom: 8px; }\n' +
        SC + ' .rl-gap { margin-top: 8px; }\n');
      function lab(html) { return '<span class="rl-nt">' + html + '</span>'; }

      // ---- state ----
      const st = Object.assign({}, DEFAULTS);
      const pre0 = PRESETS.find(function (q) { return q.value === DEFAULT_PRESET; });
      let cfg = { poles: itemsOf(pre0.poles), zeros: itemsOf(pre0.zeros) };
      let G = null;            // everything that depends on the pole/zero set only
      let dyn = null;          // everything that depends on K as well
      let dragView = null;     // s-plane window frozen during a drag
      let dragMoved = false;
      let hover = null;        // { pt } nearest locus point under the mouse
      let lastPlane = null;    // { p, px: [[x, y, branch, index]] } of the last s-plane draw
      let status = { text: '', refused: false };

      // ---- layout ----
      const intro = ui.note(api.root, 'Pick a preset or edit the poles × and zeros ○ in the s-plane, then change K: the orange squares (closed-loop poles) slide along the blue locus.');
      intro.classList.add('rl-top');
      const sp = ui.split(api.root, { ratio: 1.55 });

      const cvS = RASim.canvas(sp.main, { aspect: 1.3, minHeight: 290, maxHeight: 520, key: 'splane' });
      const hoverLine = ui.note(sp.main, '');
      hoverLine.setAttribute('data-key', 'hover');
      hoverLine.classList.add('rl-line');
      const statusLine = ui.note(sp.main, '');
      statusLine.setAttribute('data-key', 'status');
      statusLine.classList.add('rl-line');
      const legend = ui.legend(sp.main, []);
      legend.el.classList.add('rl-gap');
      const stepBox = ui.el('div', { 'class': 'rl-gap' }, sp.main);
      const cvT = RASim.canvas(stepBox, { aspect: 2.4, minHeight: 180, maxHeight: 250, key: 'step' });

      const gG = ui.group(sp.side, 'Open-loop transfer function');
      const selPre = ui.select(gG, { label: 'Preset', key: 'preset', value: DEFAULT_PRESET,
        options: PRESETS.map(function (q) { return { value: q.value, label: q.label }; }).concat([{ value: 'custom', label: 'Custom (edited)' }]),
        onChange: function (v) {
          const q = PRESETS.find(function (x) { return x.value === v; });
          if (!q) { selPre.set(matchPreset()); return; }
          setConfig(itemsOf(q.poles), itemsOf(q.zeros));
          setStatus('Preset G(s) = ' + q.label + '.');
          update();
        } });
      const segTool = ui.segmented(gG, { label: 'Tool', key: 'tool', value: st.tool,
        options: [{ value: 'move', label: 'Move' }, { value: 'addPole', label: 'Add pole ×' },
          { value: 'addZero', label: 'Add zero ○' }, { value: 'delete', label: 'Delete' }],
        onChange: function (v) { st.tool = v; setStatus(''); update(); } });

      const gK = ui.group(sp.side, 'Gain');
      const sK = ui.slider(gK, { label: 'Gain K', min: KMIN, max: KMAX, scale: 'log', value: st.K, key: 'K',
        onInput: function (v) { st.K = v; update(); } });

      const gD = ui.group(sp.side, 'Show');
      const rowC = ui.row(gD);
      const cA = ui.checkbox(rowC, { label: 'Asymptotes', checked: st.showAsymptotes, key: 'showAsymptotes',
        onChange: function (v) { st.showAsymptotes = v; update(); } });
      const cB = ui.checkbox(rowC, { label: 'Break points', checked: st.showBreaks, key: 'showBreaks',
        onChange: function (v) { st.showBreaks = v; update(); } });
      const cG = ui.checkbox(rowC, { label: 'Departure / arrival angles', checked: st.showAngles, key: 'showAngles',
        onChange: function (v) { st.showAngles = v; update(); } });
      const cS = ui.checkbox(rowC, { label: 'Step response', checked: st.showStep, key: 'showStep',
        onChange: function (v) { st.showStep = v; update(); } });

      const rowR = ui.row(sp.side);
      ui.button(rowR, { label: 'Reset', key: 'reset', title: 'Default preset 1/[s(s+1)(s+2)], K = 1, Move tool, all layers on',
        onClick: function () { reset(); } });
      const tag = ui.tag(rowR, { text: '', key: 'stability' });

      const tfLine = ui.el('p', { 'class': 'rl-eq' }, sp.side);
      tfLine.setAttribute('data-key', 'tf');
      const ceLine = ui.el('p', { 'class': 'rl-eq' }, sp.side);
      ceLine.setAttribute('data-key', 'charEq');

      const stats = ui.stats(sp.side);
      stats.classList.add('rl-stats');
      const tN = ui.stat(stats, { label: 'poles ' + lab('n'), key: 'n' });
      const tM = ui.stat(stats, { label: 'zeros ' + lab('m'), key: 'm' });
      const tAs = ui.stat(stats, { label: 'asymptotes ' + lab('(2q + 1)·180°/(n − m)'), key: 'asym' });
      const tCe = ui.stat(stats, { label: 'centroid ' + lab('σ<sub>a</sub>'), key: 'centroid', title: 'σa = (Σ Re p − Σ Re z)/(n − m)' });
      const tBr = ui.stat(stats, { label: 'break points ' + lab('(dK/ds = 0, K > 0)'), key: 'breaks' });
      const tCr = ui.stat(stats, { label: lab('jω') + '-axis crossings', key: 'crossings', title: 'The marginal gain the Routh array gives' });
      const tSt = ui.stat(stats, { label: 'stable for', key: 'stable' });
      const tCl = ui.stat(stats, { label: 'closed-loop poles at ' + lab('K'), key: 'clPoles' });
      const tZ = ui.stat(stats, { label: 'dominant ' + lab('ζ'), key: 'zeta', title: 'Closed-loop pole (pair) nearest the jω-axis; shown only when the loop is stable' });
      const tW = ui.stat(stats, { label: 'dominant ' + lab('ω<sub>n</sub>'), key: 'wn', title: 'Closed-loop pole (pair) nearest the jω-axis; shown only when the loop is stable' });
      [tAs, tBr, tCr, tSt, tCl].forEach(function (t) { t.el.classList.add('rl-wide'); });
      const clLabel = tCl.el.querySelector('.sim-stat-label');

      ui.hints(api.root, [
        'Preset 1/[s(s+1)(s+2)]: raise K past 6 — two poles cross the jω-axis at ±j1.414 and the step response grows without bound. The Routh array gives the same K.',
        'Start from 1/[s(s+2)] and add a zero at −3: the locus bends to the left into a circle — a zero attracts the locus and improves damping.',
        'Add a third pole instead: the branches bend to the right — a pole pushes the locus towards instability.',
        'Count: the number of branches equals the number of poles; n − m of them leave along asymptotes from the centroid.',
        'Click on the locus to place the closed-loop poles there and read the gain K from the magnitude condition.'
      ]);

      // ---------------------------------------------------------------- configuration
      function fullPoles() { return expand(cfg.poles); }
      function fullZeros() { return expand(cfg.zeros); }
      function matchPreset() {
        const kp = itemKey(cfg.poles), kz = itemKey(cfg.zeros);
        const q = PRESETS.find(function (x) { return itemKey(itemsOf(x.poles)) === kp && itemKey(itemsOf(x.zeros)) === kz; });
        return q ? q.value : 'custom';
      }
      function setConfig(poles, zeros) {
        cfg = { poles: poles.map(function (q) { return { re: q.re, im: q.im }; }), zeros: zeros.map(function (q) { return { re: q.re, im: q.im }; }) };
        hover = null;
        recompute(false);
        selPre.set(matchPreset());
      }
      function viewHalf(v) { return Math.max(v.xmax - v.xmin, v.ymax - v.ymin) / 2; }
      // coarse: during a drag (window frozen, coarser locus)
      function recompute(coarse) {
        const P = fullPoles(), Z = fullZeros();
        const g = { P: P, Z: Z, poly: model.polys(P, Z) };
        g.asym = model.asymptotes(P, Z);
        g.breaks = model.breakPoints(P, Z);
        g.cross = model.jwCrossings(P, Z);
        g.ranges = model.stableRanges(P, Z);
        g.dep = model.departureAngles(P, Z);
        g.arr = model.arrivalAngles(P, Z);
        g.view = dragView || model.view(P, Z);
        g.locus = model.locus(P, Z, coarse ? { view: g.view, maxMove: viewHalf(g.view) / 30 } : { view: g.view });
        g.coarse = !!coarse;
        // read-outs that depend on the configuration only
        const num = g.poly.num, den = g.poly.den;
        const nt = polyText(num), dt = polyText(den);
        g.tf = 'G(s) = ' + (nTerms(num) > 1 ? '(' + nt + ')' : nt) + ' / ' + (nTerms(den) > 1 ? '(' + dt + ')' : dt);
        g.sAsym = g.asym.count ? g.asym.count + ' at ' + g.asym.anglesDeg.map(function (a) { return fmt(a, 4) + '°'; }).join(', ') : 'none';
        g.sCent = g.asym.count ? fmt(g.asym.centroid, 4) : '—';
        g.sBreaks = g.breaks.length ? g.breaks.map(function (b) { return RASim.fmtComplex(b.s, 4) + ' (K = ' + fmt(b.K, 4) + ')'; }).join(', ') : 'none';
        g.sCross = g.cross.length ? g.cross.map(function (c) { return (c.w > 0 ? '±j' + fmt(c.w, 4) : '0') + ' at K = ' + fmt(c.K, 4); }).join(', ') : 'none';
        g.sStable = rangesText(g.ranges);
        G = g;
      }
      function rangesText(rs) {
        if (!rs.length) return 'never';
        if (rs.length === 1 && rs[0][0] === 0 && rs[0][1] === Infinity) return 'all K > 0';
        return rs.map(function (iv) {
          if (iv[1] === Infinity) return 'K > ' + fmt(iv[0], 4);
          return fmt(iv[0], 4) + ' < K < ' + fmt(iv[1], 4);
        }).join(' and ');
      }
      function rootsText(roots) {
        const out = [];
        roots.forEach(function (q) {
          const a = cabs(q);
          const re = Math.abs(q.re) < 1e-7 * a ? 0 : q.re;
          if (Math.abs(q.im) < 1e-9 * Math.max(1, a)) out.push(fmt(re, 4));
          else if (q.im > 0) out.push((re === 0 ? '' : fmt(re, 4) + ' ') + '± j' + fmt(q.im, 4));
        });
        return out.join(', ').replace(/(^|, )± j/g, '$1±j');
      }

      // ---------------------------------------------------------------- status line
      const TOOL_HELP = {
        move: 'Move: drag a pole × or zero ○ (a conjugate pair moves together); click the blue locus to set K there.',
        addPole: 'Add pole: click in the s-plane. Within 8 px of the real axis you get a real pole, elsewhere a conjugate pair.',
        addZero: 'Add zero: click in the s-plane. Within 8 px of the real axis you get a real zero, elsewhere a conjugate pair.',
        'delete': 'Delete: click a pole or zero to remove it (a conjugate pair goes together).'
      };
      function setStatus(text, refused) { status = { text: text, refused: !!refused }; }
      function renderStatus() {
        const t = status.text || TOOL_HELP[st.tool];
        statusLine.innerHTML = status.refused ? '<span class="rl-refused">' + t + '</span>' : t;
      }
      const HOVER_HELP = 'Hover over the locus to read s and K = |D(s)|/|N(s)| (the magnitude condition).';
      function renderHover() {
        if (!hover) { hoverLine.innerHTML = HOVER_HELP; return; }
        const s = hover.pt;
        hoverLine.innerHTML = 'Locus point s = <b>' + RASim.fmtComplex(s, 4) + '</b>: K = |D(s)|/|N(s)| = <b>' + fmt(model.kAt(G.P, G.Z, s), 4) + '</b>';
      }

      // ---------------------------------------------------------------- legend
      let legendKey = '';
      function mk(sym, color) { return '<span class="rl-mk" style="color:' + color + '">' + sym + '</span> '; }
      function updateLegend() {
        const items = [{ color: COL.locus, label: 'root locus (arrows: K increasing)' },
          { marker: true, label: mk('×', COL.pole) + 'open-loop pole' },
          { marker: true, label: mk('○', COL.zero) + 'zero' },
          { marker: true, label: mk('■', COL.cl) + 'closed-loop poles at K' }];
        if (st.showAsymptotes && G.asym.count) items.push({ color: COL.asym, dash: true, label: 'asymptotes, centroid +' });
        if (st.showBreaks && G.breaks.length) items.push({ marker: true, label: mk('◆', COL.brk) + 'break point' });
        if (G.cross.length) items.push({ marker: true, label: mk('●', COL.cross) + 'jω-axis crossing' });
        if (st.showAngles && (G.dep.length || G.arr.length)) items.push({ color: COL.ang, label: 'departure / arrival angle' });
        const key = JSON.stringify(items);
        if (key === legendKey) return;
        legendKey = key;
        legend.set(items);
        const els = legend.el.querySelectorAll('.sim-legend-item');
        items.forEach(function (it, i) { if (it.marker && els[i]) els[i].classList.add('rl-nosw'); });
      }

      // ---------------------------------------------------------------- update
      function update() {
        const P = G.P, Z = G.Z;
        const cl = model.closedLoopPoles(P, Z, st.K);
        const kind = model.stability(cl);
        const dom = kind === 'stable' ? model.dominant(cl) : null;
        dyn = { cl: cl, kind: kind, step: st.showStep ? model.stepResponse(P, Z, st.K) : null };
        tag.set(kind === 'stable' ? 'STABLE' : kind === 'marginal' ? 'MARGINALLY STABLE' : 'UNSTABLE',
          kind === 'stable' ? 'ok' : kind === 'marginal' ? 'warn' : 'bad');
        tfLine.innerHTML = '<span class="rl-lab">Open loop:</span> ' + G.tf;
        ceLine.innerHTML = '<span class="rl-lab">1 + K·G(s) = 0 at K = ' + fmt(st.K, 4) + ':</span> ' +
          polyText(model.charPoly(P, Z, st.K)) + ' = 0';
        tN.set(String(P.length)); tM.set(String(Z.length));
        tAs.set(G.sAsym); tCe.set(G.sCent);
        tBr.set(G.sBreaks); tCr.set(G.sCross); tSt.set(G.sStable);
        clLabel.innerHTML = 'closed-loop poles at ' + lab('K = ' + fmt(st.K, 4));
        tCl.set(rootsText(cl));
        tCl.setState(kind === 'stable' ? '' : kind === 'marginal' ? 'warn' : 'bad');
        tZ.set(dom ? fmt(dom.zeta, 4) : '—');
        tW.set(dom ? fmt(dom.wn, 4) + ' rad/s' : '—');
        renderStatus(); renderHover(); updateLegend();
        stepBox.style.display = st.showStep ? '' : 'none';
        segTool.set(st.tool);
        cvS.redraw();
        if (st.showStep) cvT.redraw();
      }

      // ---------------------------------------------------------------- label placement
      function placer(ctx, frame) {
        const boxes = [], out = [];
        function ov(a, b) { return a.l < b.r && a.r > b.l && a.t < b.b && a.b > b.t; }
        return {
          block: function (x, y, rx, ry, w) { boxes.push({ l: x - rx, r: x + rx, t: y - ry, b: y + ry, w: w || 100 }); },
          // cands: [{ x, y, align, text?, cost? }] (y = middle of the text); returns the chosen candidate or null
          put: function (text, font, color, cands, optional) {
            let best = null;
            ctx.font = font;
            cands.forEach(function (c, n) {
              const T = c.text || text;
              const tw = ctx.measureText(T).width;
              const l = c.align === 'left' ? c.x : c.align === 'right' ? c.x - tw : c.x - tw / 2;
              const b = { l: l - 3, r: l + tw + 3, t: c.y - 8, b: c.y + 8 };
              let cost = n * 0.5 + (c.cost || 0);
              boxes.forEach(function (o) { if (ov(b, o)) cost += o.w; });
              if (b.l < frame.x || b.r > frame.x + frame.w || b.t < frame.y || b.b > frame.y + frame.h) cost += 5000;
              if (!best || cost < best.cost) best = { cost: cost, c: c, b: b, T: T };
            });
            if (!best || (optional && best.cost >= 900)) return null;
            best.b.w = 1000;
            boxes.push(best.b);
            out.push({ T: best.T, c: best.c, font: font, color: color, b: best.b });
            return best.c;
          },
          flush: function () {
            out.forEach(function (o) {
              ctx.save();
              ctx.fillStyle = LBG;
              ctx.fillRect(o.b.l, o.b.t, o.b.r - o.b.l, o.b.b - o.b.t);
              ctx.font = o.font; ctx.fillStyle = o.color; ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
              ctx.fillText(o.T, o.b.l + 3, o.c.y + 0.5);
              ctx.restore();
            });
            return out;
          }
        };
      }

      // ---------------------------------------------------------------- s-plane
      function planePlot(ctx, w, h) {
        const v = dragView || G.view;
        return RASim.plot(ctx, { box: { x: 0, y: 0, w: w, h: h }, margin: { left: 40, right: 10, top: 10, bottom: 32 },
          equal: true, xmin: v.xmin, xmax: v.xmax, ymin: v.ymin, ymax: v.ymax, xlabel: 'σ', ylabel: 'jω' });
      }
      function rootMarkers() {
        // every drawn marker of the open-loop roots: { kind, idx, lower, re, im }
        const out = [];
        ['pole', 'zero'].forEach(function (kind) {
          (kind === 'pole' ? cfg.poles : cfg.zeros).forEach(function (q, i) {
            out.push({ kind: kind, idx: i, lower: false, re: q.re, im: q.im });
            if (q.im > 0) out.push({ kind: kind, idx: i, lower: true, re: q.re, im: -q.im });
          });
        });
        return out;
      }
      function nearestRoot(p, x, y) {
        let best = null, bd = HIT_ROOT;
        rootMarkers().forEach(function (q) {
          const d = Math.hypot(p.X(q.re) - x, p.Y(q.im) - y);
          if (d <= bd) { bd = d; best = q; }
        });
        return best;
      }
      function nearestLocus(x, y) {
        if (!lastPlane) return null;
        let best = null, bd = HIT_LOCUS * HIT_LOCUS;
        const px = lastPlane.px;
        for (let i = 0; i < px.length; i += 4) {
          const dx = px[i] - x, dy = px[i + 1] - y, d = dx * dx + dy * dy;
          if (d <= bd) { bd = d; best = i; }
        }
        if (best === null) return null;
        const br = lastPlane.locus[px[best + 2]];
        return br ? br[px[best + 3]] || null : null;
      }

      cvS.draw = function (ctx, w, h) {
        const p = planePlot(ctx, w, h);
        p.axes();
        const rc = p.rect, ox = p.X(0), oy = p.Y(0);
        const narrow = rc.w < 360;
        // right half-plane
        if (ox < rc.x + rc.w) {
          ctx.save(); ctx.fillStyle = COL.rhp;
          const x0 = Math.max(rc.x, ox);
          ctx.fillRect(x0, rc.y, rc.x + rc.w - x0, rc.h);
          ctx.restore();
        }
        const lay = placer(ctx, rc);
        // asymptotes
        const as = G.asym;
        if (st.showAsymptotes && as.count) {
          const far = 4 * viewHalf(dragView || G.view) + Math.abs(as.centroid);
          as.anglesDeg.forEach(function (a) {
            const t = M.rad(a);
            p.segment(as.centroid, 0, as.centroid + far * Math.cos(t), far * Math.sin(t), { color: COL.asym, width: 1.2, dash: [6, 5] });
          });
        }
        // locus
        const px = [];
        const half = viewHalf(dragView || G.view);
        G.locus.forEach(function (br, bi) {
          const xs = [], ys = [];
          let lx = -1e9, ly = -1e9;
          for (let i = 0; i < br.length; i++) {
            const q = br[i];
            const X = p.X(q.re), Y = p.Y(q.im);
            const inR = X > rc.x - 50 && X < rc.x + rc.w + 50 && Y > rc.y - 50 && Y < rc.y + rc.h + 50;
            if (inR || i === 0 || i === br.length - 1) {
              xs.push(q.re); ys.push(q.im);
              px.push(X, Y, bi, i);
              if (Math.hypot(X - lx, Y - ly) > 5 && inR) { lay.block(X, Y, 3, 3, 30); lx = X; ly = Y; }
            } else if (xs.length && isFinite(xs[xs.length - 1])) {
              xs.push(q.re); ys.push(q.im); xs.push(NaN); ys.push(NaN);
            }
          }
          p.line(xs, ys, { color: COL.locus, width: 2 });
        });
        lastPlane = { p: p, px: px, locus: G.locus };
        // geometry for the browser tests (data → pixel)
        cvS.canvas.rlGeom = { x: rc.x, y: rc.y, w: rc.w, h: rc.h, xmin: p.xmin, xmax: p.xmax, ymin: p.ymin, ymax: p.ymax };
        drawArrowHeads(ctx, p, rc);
        // asymptote centroid
        if (st.showAsymptotes && as.count) {
          const cxp = p.X(as.centroid);
          if (cxp > rc.x && cxp < rc.x + rc.w) {
            ctx.save(); ctx.strokeStyle = Cl.ink; ctx.lineWidth = 1.8;
            ctx.beginPath(); ctx.moveTo(cxp - 6, oy); ctx.lineTo(cxp + 6, oy); ctx.moveTo(cxp, oy - 6); ctx.lineTo(cxp, oy + 6); ctx.stroke();
            ctx.restore();
            lay.block(cxp, oy, 7, 7);
          }
        }
        // markers: break points, jω crossings, open-loop roots, closed-loop poles
        const brk = st.showBreaks ? G.breaks : [];
        brk.forEach(function (b) { lay.block(p.X(b.s.re), p.Y(b.s.im), 8, 8); });
        G.cross.forEach(function (c) {
          [c.w, -c.w].forEach(function (wv, k) { if (!(k === 1 && c.w === 0)) lay.block(ox, p.Y(wv), 7, 7); });
        });
        const markers = rootMarkers();
        markers.forEach(function (q) { lay.block(p.X(q.re), p.Y(q.im), 9, 9); });
        dyn.cl.forEach(function (q) { lay.block(p.X(q.re), p.Y(q.im), 7, 7, 200); });
        // departure / arrival arrows
        const angLabels = [];
        if (st.showAngles) {
          G.dep.forEach(function (d) { angleArrow(ctx, p, lay, d, false, angLabels); });
          G.arr.forEach(function (d) { angleArrow(ctx, p, lay, d, true, angLabels); });
        }
        // closed-loop poles at the current K (under the open-loop markers so both stay visible)
        dyn.cl.forEach(function (q) {
          const X = p.X(q.re), Y = p.Y(q.im);
          if (X < rc.x - 6 || X > rc.x + rc.w + 6 || Y < rc.y - 6 || Y > rc.y + rc.h + 6) return;
          ctx.save(); ctx.fillStyle = COL.cl; ctx.strokeStyle = '#ffffff'; ctx.lineWidth = 1.5;
          ctx.fillRect(X - 5.5, Y - 5.5, 11, 11); ctx.strokeRect(X - 5.5, Y - 5.5, 11, 11);
          ctx.restore();
        });
        // break points and jω crossings above the closed-loop squares: when K sits exactly at a break point or a
        // crossing (e.g. (s+1)/s² at K = 4, the four-pole preset at K = 1) both markers stay visible
        brk.forEach(function (b) {
          const X = p.X(b.s.re), Y = p.Y(b.s.im);
          ctx.save(); ctx.fillStyle = COL.brk; ctx.strokeStyle = '#ffffff'; ctx.lineWidth = 1.2;
          ctx.beginPath(); ctx.moveTo(X, Y - 6.5); ctx.lineTo(X + 6.5, Y); ctx.lineTo(X, Y + 6.5); ctx.lineTo(X - 6.5, Y); ctx.closePath();
          ctx.fill(); ctx.stroke(); ctx.restore();
        });
        G.cross.forEach(function (c) {
          [c.w, -c.w].forEach(function (wv, k) {
            if (k === 1 && c.w === 0) return;
            p.point(0, wv, { color: COL.cross, r: 4.5, stroke: '#ffffff' });
          });
        });
        // labels
        angLabels.forEach(function (f) { f(); });
        const lb = rc.x + rc.w;
        if (ox < lb - 30) {
          const xm = (Math.max(ox, rc.x) + lb) / 2;
          lay.put('unstable', F11, Cl.red, [{ x: xm, y: rc.y + 11, align: 'center' }, { x: xm, y: rc.y + rc.h - 11, align: 'center' },
            { x: lb - 4, y: rc.y + 11, align: 'right' }], true);
        }
        // the centroid label before the crossing and break-point labels, which have shorter fall-backs
        if (st.showAsymptotes && as.count) {
          const cxp = p.X(as.centroid);
          if (cxp > rc.x && cxp < rc.x + rc.w) {
            const t = 'σₐ = ' + fmt(as.centroid, 4);
            lay.put(t, F12, Cl.ink, [{ x: cxp, y: oy + 17, align: 'center' }, { x: cxp, y: oy - 17, align: 'center' },
              { x: cxp + 8, y: oy + 17, align: 'left' }, { x: cxp - 8, y: oy + 17, align: 'right' },
              { x: cxp + 8, y: oy - 17, align: 'left' }, { x: cxp - 8, y: oy - 17, align: 'right' },
              { x: cxp, y: oy + 32, align: 'center' }, { x: cxp, y: oy - 32, align: 'center' }], true);
          }
        }
        G.cross.forEach(function (c) {
          const Y = p.Y(c.w);
          const full = 'K = ' + fmt(c.K, 4) + ', ω = ' + fmt(c.w, 4), short = 'K = ' + fmt(c.K, 4);
          const fc = narrow ? 12 : 0, sc = narrow ? 0 : 12;
          lay.put(full, B12, COL.cross, [
            { x: ox + 9, y: Y, align: 'left', cost: fc }, { x: ox - 9, y: Y, align: 'right', cost: fc },
            { x: ox + 9, y: Y - 13, align: 'left', cost: fc + 1 }, { x: ox - 9, y: Y - 13, align: 'right', cost: fc + 1 },
            { x: ox + 9, y: Y, align: 'left', text: short, cost: sc }, { x: ox - 9, y: Y, align: 'right', text: short, cost: sc },
            { x: ox + 9, y: Y - 14, align: 'left', text: short, cost: sc + 1 }, { x: ox - 9, y: Y - 14, align: 'right', text: short, cost: sc + 1 },
            { x: ox, y: Y - 16, align: 'center', text: short, cost: sc + 2 }]);
        });
        brk.forEach(function (b, i) {
          const X = p.X(b.s.re), Y = p.Y(b.s.im);
          if (b.s.im < 0) return;
          const full = 's = ' + RASim.fmtComplex(b.s, 3) + ', K = ' + fmt(b.K, 3), short = 'K = ' + fmt(b.K, 3);
          const fc = narrow ? 12 : 0, sc = narrow ? 0 : 12;
          const dy = i % 2 ? -1 : 1;        // alternate below / above for neighbouring break points
          const cands = [];
          [[0, 17 * dy], [0, -17 * dy], [0, 32 * dy], [0, -32 * dy]].forEach(function (o, k) {
            cands.push({ x: X + o[0], y: Y + o[1], align: 'center', cost: fc + k });
            cands.push({ x: X + 8, y: Y + o[1], align: 'left', cost: fc + k + 0.5 });
            cands.push({ x: X - 8, y: Y + o[1], align: 'right', cost: fc + k + 0.5 });
          });
          [[0, 17 * dy], [0, -17 * dy], [0, 32 * dy], [0, -32 * dy]].forEach(function (o, k) {
            cands.push({ x: X + o[0], y: Y + o[1], align: 'center', text: short, cost: sc + k });
          });
          lay.put(full, F12, COL.brk, cands);
        });
        const placed = lay.flush();
        cvS.canvas.rlLabels = placed.map(function (o) { return { text: o.T, l: o.b.l, r: o.b.r, t: o.b.t, b: o.b.b }; });
        // open-loop poles and zeros on top
        markers.forEach(function (q) {
          if (q.kind === 'pole') p.cross(q.re, q.im, { color: COL.pole, size: 6.5, width: 2.6 });
          else {
            ctx.save(); ctx.strokeStyle = COL.zero; ctx.lineWidth = 2.4;
            ctx.beginPath(); ctx.arc(p.X(q.re), p.Y(q.im), 6, 0, 2 * Math.PI); ctx.stroke(); ctx.restore();
          }
        });
        // hover marker
        if (hover) {
          ctx.save(); ctx.strokeStyle = COL.hover; ctx.lineWidth = 2;
          ctx.beginPath(); ctx.arc(p.X(hover.pt.re), p.Y(hover.pt.im), 7.5, 0, 2 * Math.PI); ctx.stroke(); ctx.restore();
        }
      };

      // small arrow heads along each branch, pointing towards increasing K
      function drawArrowHeads(ctx, p, rc) {
        const roots = rootMarkers().map(function (q) { return [p.X(q.re), p.Y(q.im)]; });
        G.locus.forEach(function (br) {
          // in-view polyline length
          const pts = [];
          for (let i = 0; i < br.length; i++) {
            const X = p.X(br[i].re), Y = p.Y(br[i].im);
            if (X >= rc.x && X <= rc.x + rc.w && Y >= rc.y && Y <= rc.y + rc.h) pts.push([X, Y]);
            else if (pts.length) break;          // only the first stretch inside the window
          }
          let L = 0;
          for (let i = 1; i < pts.length; i++) L += Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]);
          if (L < 30) return;
          const fr = L > 420 ? [0.2, 0.5, 0.82] : L > 160 ? [0.3, 0.72] : [0.5];
          fr.forEach(function (f) {
            const target = f * L;
            let acc = 0;
            for (let i = 1; i < pts.length; i++) {
              const seg = Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]);
              if (acc + seg >= target && seg > 0) {
                const u = (target - acc) / seg;
                const X = pts[i - 1][0] + u * (pts[i][0] - pts[i - 1][0]), Y = pts[i - 1][1] + u * (pts[i][1] - pts[i - 1][1]);
                // direction from a stretch of about 8 px around the point
                let j = i;
                while (j < pts.length - 1 && Math.hypot(pts[j][0] - X, pts[j][1] - Y) < 6) j++;
                let k = i - 1;
                while (k > 0 && Math.hypot(pts[k][0] - X, pts[k][1] - Y) < 6) k--;
                const dx = pts[j][0] - pts[k][0], dy = pts[j][1] - pts[k][1], dl = Math.hypot(dx, dy);
                if (dl < 1e-6) return;
                if (roots.some(function (r0) { return Math.hypot(r0[0] - X, r0[1] - Y) < 14; })) return;
                const ux = dx / dl, uy = dy / dl, hl = 9, hw = 4.5;
                const tx = X + ux * hl / 2, ty = Y + uy * hl / 2;
                ctx.save(); ctx.fillStyle = COL.locus;
                ctx.beginPath(); ctx.moveTo(tx, ty);
                ctx.lineTo(tx - ux * hl - uy * hw, ty - uy * hl + ux * hw);
                ctx.lineTo(tx - ux * hl + uy * hw, ty - uy * hl - ux * hw);
                ctx.closePath(); ctx.fill(); ctx.restore();
                return;
              }
              acc += seg;
            }
          });
        });
      }

      // departure (from a pole) or arrival (into a zero) arrow with its angle arc and label
      function angleArrow(ctx, p, lay, d, arrival, labels) {
        if (!isFinite(d.deg)) return;
        [1, -1].forEach(function (sg) {
          const X = p.X(d.at.re), Y = p.Y(sg * d.at.im);
          const t = M.rad(sg * d.deg), ux = Math.cos(t), uy = -Math.sin(t);
          const r0 = 10, r1 = p.rect.w < 360 ? 34 : 42;
          // 0° reference
          ctx.save(); ctx.strokeStyle = RASim.alpha(COL.ang, 0.7); ctx.lineWidth = 1; ctx.setLineDash([3, 3]);
          ctx.beginPath(); ctx.moveTo(X + 8, Y); ctx.lineTo(X + 36, Y); ctx.stroke(); ctx.restore();
          // arc from 0° to the angle
          if (Math.abs(d.deg) > 3) {
            ctx.save(); ctx.strokeStyle = COL.ang; ctx.lineWidth = 1.4;
            ctx.beginPath(); ctx.arc(X, Y, 20, 0, -t, t > 0); ctx.stroke(); ctx.restore();
          }
          if (arrival) RASim.draw.arrow(ctx, X + r1 * ux, Y + r1 * uy, X + r0 * ux, Y + r0 * uy, { color: COL.ang, width: 2, head: 8 });
          else RASim.draw.arrow(ctx, X + r0 * ux, Y + r0 * uy, X + r1 * ux, Y + r1 * uy, { color: COL.ang, width: 2, head: 8 });
          for (let k = 1; k <= 5; k++) lay.block(X + (r0 + (r1 - r0) * k / 5) * ux, Y + (r0 + (r1 - r0) * k / 5) * uy, 4, 4, 200);
          for (let k = 0; k <= 4; k++) { const a = -t * k / 4; lay.block(X + 20 * Math.cos(a), Y + 20 * Math.sin(a), 3, 3, 60); }
          if (sg < 0) return;
          const txt = fmt(d.deg, 4) + '°';
          // label beyond the arrow tip, or beside the arc
          const ex = X + (r1 + 8) * ux, ey = Y + (r1 + 8) * uy;
          const al = ux > 0.3 ? 'left' : ux < -0.3 ? 'right' : 'center';
          const mid = -t / 2, ax = X + 30 * Math.cos(mid), ay = Y + 30 * Math.sin(mid);
          labels.push(function () {
            lay.put(txt, B12, COL.ang, [{ x: ex, y: ey, align: al }, { x: ax, y: ay, align: Math.cos(mid) >= 0 ? 'left' : 'right' },
              { x: ex, y: ey + (uy >= 0 ? 12 : -12), align: al }, { x: X + 40, y: Y + (t > 0 ? 12 : -12), align: 'left' },
              { x: X - 12, y: Y, align: 'right' }, { x: ex + (ux >= 0 ? 10 : -10), y: ey, align: al },
              { x: X - 12, y: Y - 16, align: 'right' }, { x: X - 12, y: Y + 16, align: 'right' }]);
          });
        });
      }

      // ---- pointer interaction ----
      function canvasPos(e) {
        const r = cvS.canvas.getBoundingClientRect();
        return { x: e.clientX - r.left - (cvS.canvas.clientLeft || 0), y: e.clientY - r.top - (cvS.canvas.clientTop || 0) };
      }
      function curPlot() { return planePlot(cvS.ctx, cvS.width, cvS.height); }
      cvS.onDrag({
        hit: function (x, y) {
          if (st.tool !== 'move' || !G) return null;
          const q = nearestRoot(curPlot(), x, y);
          return q ? { q: q, view: dragView || G.view } : null;
        },
        move: function (x, y, tok) {
          dragMoved = true;
          dragView = tok.view;
          const p = curPlot();
          const list = tok.q.kind === 'pole' ? cfg.poles : cfg.zeros;
          const it = list[tok.q.idx];
          const re = snap(M.clamp(p.invX(x), p.xmin, p.xmax));
          let im = 0;
          if (it.im > 0) im = Math.max(IMMIN, snap(M.clamp(tok.q.lower ? -p.invY(y) : p.invY(y), IMMIN, Math.max(p.ymax, -p.ymin))));
          if (re === it.re && im === it.im) return;
          it.re = re; it.im = im;
          hover = null;
          recompute(true);
          selPre.set(matchPreset());
          setStatus('Moving ' + (tok.q.kind === 'pole' ? 'a pole' : 'a zero') + (im > 0 ? ' pair' : '') + ' to s = ' +
            (im > 0 ? fmt(re, 3) + ' ± j' + fmt(im, 3) : fmt(re, 3)) + '.');
          update();
        },
        end: function () {
          const was = dragView;
          dragView = null;
          if (was) { recompute(false); update(); }
        },
        cursor: function (x, y) {
          if (!G) return null;
          const p = curPlot();
          if (!p.contains(x, y)) return null;
          if (st.tool === 'move') return nearestRoot(p, x, y) ? 'grab' : nearestLocus(x, y) ? 'pointer' : null;
          if (st.tool === 'delete') return nearestRoot(p, x, y) ? 'pointer' : null;
          return 'crosshair';
        }
      });
      cvS.canvas.addEventListener('pointerdown', function () { dragMoved = false; });
      cvS.canvas.addEventListener('click', function (e) {
        if (dragMoved) { dragMoved = false; return; }
        if (!G || !cvS.width) return;
        const pos = canvasPos(e), p = curPlot();
        if (!p.contains(pos.x, pos.y)) return;
        if (st.tool === 'move') pickK(p, pos.x, pos.y);
        else if (st.tool === 'delete') deleteAt(p, pos.x, pos.y);
        else addAt(p, pos.x, pos.y, st.tool === 'addPole' ? 'pole' : 'zero');
      });
      cvS.onHover({
        move: function (x, y) {
          if (!G || dragView) return;
          const q = nearestLocus(x, y);
          const was = hover;
          hover = q && !nearestRoot(curPlot(), x, y) ? { pt: q } : null;
          if ((was && was.pt) !== (hover && hover.pt)) { renderHover(); cvS.redraw(); }
        },
        leave: function () { if (hover) { hover = null; renderHover(); cvS.redraw(); } }
      });

      function pickK(p, x, y) {
        if (nearestRoot(p, x, y)) return;
        const q = nearestLocus(x, y);
        if (!q) return;
        const k = q.K;
        sK.set(k); st.K = sK.get();
        if (k < KMIN || k > KMAX) setStatus('That locus point has K = ' + fmt(k, 4) + ', outside the slider range ' + fmt(KMIN, 2) + ' … ' + fmt(KMAX, 4) + '; K is set to ' + fmt(st.K, 4) + '.', true);
        else setStatus('Picked the locus point s = ' + RASim.fmtComplex(q, 4) + ': K = |D(s)|/|N(s)| = ' + fmt(model.kAt(G.P, G.Z, q), 4) + '.');
        update();
      }
      function addAt(p, x, y, kind) {
        const re = snap(p.invX(x));
        const real = Math.abs(y - p.Y(0)) <= REAL_SNAP;
        const im = real ? 0 : Math.max(IMMIN, snap(Math.abs(p.invY(y))));
        const add = real ? 1 : 2;
        const nP = countOf(cfg.poles), nZ = countOf(cfg.zeros);
        const where = real ? fmt(re, 3) : fmt(re, 3) + ' ± j' + fmt(im, 3);
        if (kind === 'pole' && nP + add > MAXP) {
          setStatus('Refused: at most ' + MAXP + ' poles (there are ' + nP + '; ' + (real ? 'a real pole adds 1' : 'a pair adds 2') + ').', true);
        } else if (kind === 'zero' && nZ + add > nP) {
          setStatus('Refused: zeros may not outnumber poles (m ≤ n; now m = ' + nZ + ', n = ' + nP + ', ' + (real ? 'a real zero adds 1' : 'a pair adds 2') + ').', true);
        } else {
          (kind === 'pole' ? cfg.poles : cfg.zeros).push({ re: re, im: im });
          setConfig(cfg.poles, cfg.zeros);
          setStatus('Added a ' + kind + (real ? '' : ' pair') + ' at s = ' + where + '.');
        }
        update();
      }
      function deleteAt(p, x, y) {
        const q = nearestRoot(p, x, y);
        if (!q) { setStatus('Delete: click exactly on a pole × or zero ○.'); update(); return; }
        const list = q.kind === 'pole' ? cfg.poles : cfg.zeros;
        const it = list[q.idx], c = it.im > 0 ? 2 : 1;
        const nP = countOf(cfg.poles), nZ = countOf(cfg.zeros);
        if (q.kind === 'pole' && nP - c < 1) setStatus('Refused: G(s) needs at least one pole.', true);
        else if (q.kind === 'pole' && nP - c < nZ) setStatus('Refused: removing ' + (c === 2 ? 'this pole pair' : 'this pole') + ' would leave more zeros than poles (m = ' + nZ + ', n would be ' + (nP - c) + '). Delete a zero first.', true);
        else {
          list.splice(q.idx, 1);
          setConfig(cfg.poles, cfg.zeros);
          setStatus('Removed the ' + q.kind + (c === 2 ? ' pair' : '') + ' at s = ' + (c === 2 ? fmt(it.re, 3) + ' ± j' + fmt(it.im, 3) : fmt(it.re, 3)) + '.');
        }
        update();
      }

      // ---------------------------------------------------------------- step response
      cvT.draw = function (ctx, w, h) {
        if (!st.showStep || !dyn || !dyn.step) return;
        const s = dyn.step;
        let lo = 0, hi = 0;
        s.y.forEach(function (v) { if (isFinite(v)) { lo = Math.min(lo, v); hi = Math.max(hi, v); } });
        if (isFinite(s.final)) hi = Math.max(hi, s.final);
        if (!(hi - lo > 1e-9)) hi = lo + 1;
        const pad = 0.1 * (hi - lo);
        lo = Math.max(-1, lo - (lo < 0 ? pad : 0)); hi = Math.min(3, hi + pad);
        const p = RASim.plot(ctx, { box: { x: 0, y: 0, w: w, h: h }, margin: { left: 46, right: 12, top: 26, bottom: 34 },
          xmin: 0, xmax: s.tEnd, ymin: lo, ymax: hi, xlabel: 't (s)', ylabel: 'y(t)',
          title: 'Closed-loop unit-step response, K = ' + fmt(st.K, 4) });
        p.axes();
        const rc = p.rect;
        if (isFinite(s.final)) {
          p.hline(s.final, { color: Cl.muted, width: 1.2, dash: [5, 4] });
        }
        const ys = s.y.map(function (v) { return isFinite(v) ? M.clamp(v, lo - 10, hi + 10) : NaN; });
        p.line(s.t, ys, { color: COL.step, width: 2.2 });
        ctx.save();
        if (isFinite(s.final)) {
          const t = 'final value T(0) = ' + fmt(s.final, 4);
          cvT.canvas.rlCaption = t;          // test hook
          ctx.font = F12;
          const tw = ctx.measureText(t).width;
          const yy = p.Y(s.final);
          // right end above / below the dashed line, else the left end: the first box the curve does not cross
          const cands = [[rc.x + rc.w - tw - 6, yy - 11], [rc.x + rc.w - tw - 6, yy + 11], [rc.x + 6, yy - 11], [rc.x + 6, yy + 11]];
          let pick = null;
          cands.forEach(function (c) {
            if (pick || c[1] - 8 < rc.y || c[1] + 8 > rc.y + rc.h) return;
            const hitC = s.t.some(function (tv, i) {
              const X = p.X(tv), Y = p.Y(ys[i]);
              return X >= c[0] - 6 && X <= c[0] + tw + 6 && Y >= c[1] - 11 && Y <= c[1] + 11;
            });
            if (!hitC) pick = c;
          });
          if (!pick) pick = cands[0][1] - 8 >= rc.y ? cands[0] : cands[1];
          ctx.fillStyle = LBG; ctx.fillRect(pick[0] - 3, pick[1] - 8, tw + 6, 16);
          ctx.fillStyle = Cl.muted; ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
          ctx.fillText(t, pick[0], pick[1] + 0.5);
        } else {
          // marginal: poles on the jω-axis away from the origin oscillate; a simple closed-loop pole at s = 0 makes y(t) ramp
          let osc = false, ramp = false;
          if (s.kind === 'marginal') {
            const tol = 1e-6 * Math.max(maxAbs(dyn.cl), 1e-9);
            dyn.cl.forEach(function (q) { if (Math.abs(q.re) <= tol) { if (Math.abs(q.im) > tol) osc = true; else ramp = true; } });
          }
          const t = s.kind !== 'marginal' ? 'unstable: y(t) grows without bound' :
            'marginally stable: ' + (osc ? 'sustained oscillation' : '') + (osc && ramp ? ' + ' : '') + (ramp ? 'y(t) ramps (pole at s = 0)' : '');
          cvT.canvas.rlCaption = t;          // test hook
          ctx.font = B12;
          const tw = ctx.measureText(t).width;
          ctx.fillStyle = LBG; ctx.fillRect(rc.x + 5, rc.y + 4, tw + 6, 16);
          ctx.fillStyle = s.kind === 'marginal' ? Cl.gold : Cl.red; ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
          ctx.fillText(t, rc.x + 8, rc.y + 12.5);
        }
        ctx.restore();
      };

      // ---------------------------------------------------------------- state
      function reset() {
        dragView = null;
        setConfig(itemsOf(pre0.poles), itemsOf(pre0.zeros));
        setStatus('');
        applyPlain(DEFAULTS);
        update();
      }
      function applyPlain(s) {
        if (s.K != null && isFinite(Number(s.K))) { sK.set(Number(s.K)); st.K = sK.get(); }
        if (s.tool != null && TOOLS.indexOf(String(s.tool)) >= 0) { st.tool = String(s.tool); segTool.set(st.tool); }
        if (s.showAsymptotes != null) { st.showAsymptotes = !!s.showAsymptotes; cA.set(st.showAsymptotes); }
        if (s.showBreaks != null) { st.showBreaks = !!s.showBreaks; cB.set(st.showBreaks); }
        if (s.showAngles != null) { st.showAngles = !!s.showAngles; cG.set(st.showAngles); }
        if (s.showStep != null) { st.showStep = !!s.showStep; cS.set(st.showStep); }
      }
      function getState() {
        return { poles: fullPoles(), zeros: fullZeros(), K: st.K, tool: st.tool, showAsymptotes: st.showAsymptotes,
          showBreaks: st.showBreaks, showAngles: st.showAngles, showStep: st.showStep };
      }
      function setState(s) {
        s = s || {};
        if (s.poles != null || s.zeros != null) {
          const P = s.poles != null ? toItems(s.poles) : cfg.poles;
          const Z = s.zeros != null ? toItems(s.zeros) : cfg.zeros;
          const nP = P ? countOf(P) : 0, nZ = Z ? countOf(Z) : 0;
          if (P && Z && nP >= 1 && nP <= MAXP && nZ <= nP) {
            dragView = null;
            setConfig(P, Z);
            setStatus('');
          } else setStatus('setState: refused an invalid pole/zero set (1 ≤ n ≤ 6, m ≤ n, conjugate pairs); the configuration is unchanged.', true);
        }
        if (s.tool != null && String(s.tool) !== st.tool) setStatus('');
        applyPlain(s);
        update();
      }

      recompute(false);
      selPre.set(matchPreset());
      update();
      return { getState: getState, setState: setState };
    }
  });
})();
