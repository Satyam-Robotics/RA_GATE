/* RASim core: shared foundation for the interactive study simulations.
 * Classic script (no modules). Loads in browsers and in Node (model tests).
 * At load time it never touches window or document.
 * API reference: notes/sims/README.md
 */
(function () {
  'use strict';

  var HAS_DOM = typeof document !== 'undefined' && typeof window !== 'undefined';
  var MINUS = '−';
  var TAU = 2 * Math.PI;

  var RASim = {};
  RASim.sims = {};
  RASim.instances = {};

  /* ------------------------------------------------------------------ *
   * 1.4 Colours
   * ------------------------------------------------------------------ */
  RASim.colors = {
    ink: '#1a1a1a', muted: '#666666', faint: '#9a9a9a', grid: '#e4e0d6', axis: '#8a8a8a',
    panel: '#fbf8f0', fill: '#f3ead2', maroon: '#6b1d1d', blue: '#1f4e79', green: '#2e6b30',
    orange: '#c2571a', purple: '#6a3d9a', red: '#c0392b', teal: '#0f766e', gold: '#b8860b',
    lightBlue: '#9cc3e6', lightRed: '#f2c1bb', lightGreen: '#bfe0c0'
  };
  var C = RASim.colors;

  RASim.alpha = function (hex, a) {
    var h = String(hex).replace('#', '');
    if (h.length === 3) h = h[0] + h[0] + h[1] + h[1] + h[2] + h[2];
    var r = parseInt(h.slice(0, 2), 16), g = parseInt(h.slice(2, 4), 16), b = parseInt(h.slice(4, 6), 16);
    return 'rgba(' + r + ', ' + g + ', ' + b + ', ' + a + ')';
  };

  /* ------------------------------------------------------------------ *
   * 1.3 Formatting
   * ------------------------------------------------------------------ */
  var SUP = { '0': '⁰', '1': '¹', '2': '²', '3': '³', '4': '⁴', '5': '⁵', '6': '⁶', '7': '⁷', '8': '⁸', '9': '⁹', '-': '⁻' };
  function superscript(n) {
    return String(n).split('').map(function (ch) { return SUP[ch] || ch; }).join('');
  }
  function stripZeros(s) {
    if (s.indexOf('.') < 0) return s;
    s = s.replace(/0+$/, '');
    if (s[s.length - 1] === '.') s = s.slice(0, -1);
    return s;
  }
  function special(v) {
    if (Number.isNaN(v)) return '—';
    if (v === Infinity) return '∞';
    if (v === -Infinity) return MINUS + '∞';
    return null;
  }

  function fmt(v, sig) {
    if (sig == null) sig = 4;
    sig = Math.max(1, Math.min(21, Math.round(sig)));
    v = Number(v);
    var sp = special(v);
    if (sp !== null) return sp;
    if (Math.abs(v) < 1e-12) return '0';
    var a = Math.abs(v);
    var e = a.toExponential(sig - 1);            // rounded to sig significant digits
    var parts = e.split('e');
    var exp = parseInt(parts[1], 10);
    var r = Number(e);
    var s;
    if (r >= 1e-4 && r < 1e7) {
      s = stripZeros(r.toFixed(Math.max(0, sig - 1 - exp)));
    } else {
      s = stripZeros(parts[0]) + '×10' + superscript(exp);
    }
    if (s === '0') return '0';
    return v < 0 ? MINUS + s : s;
  }
  RASim.fmt = fmt;

  RASim.fmtFixed = function (v, decimals) {
    if (decimals == null) decimals = 2;
    v = Number(v);
    var sp = special(v);
    if (sp !== null) return sp;
    if (Math.abs(v) < 1e-12) return (0).toFixed(decimals);
    if (Math.abs(v) >= 1e21) return fmt(v, 6);   // toFixed would switch to '1e+21' notation
    var s = Math.abs(v).toFixed(decimals);
    if (Number(s) === 0) return s;
    return v < 0 ? MINUS + s : s;
  };

  function toC(a) {
    if (typeof a === 'number') return { re: a, im: 0 };
    return { re: +a.re || 0, im: +a.im || 0 };
  }

  RASim.fmtComplex = function (z, sig) {
    if (sig == null) sig = 4;
    z = toC(z);
    if (Number.isNaN(z.re) || Number.isNaN(z.im)) return '—';
    var mag = Math.max(1, isFinite(z.re) ? Math.abs(z.re) : 0, isFinite(z.im) ? Math.abs(z.im) : 0);
    var imZero = Math.abs(z.im) < 1e-12 * mag || fmt(z.im, sig) === '0';
    var reZero = Math.abs(z.re) < 1e-12 * mag || fmt(z.re, sig) === '0';
    if (imZero) return fmt(z.re, sig);
    var im = 'j' + fmt(Math.abs(z.im), sig);
    if (reZero) return z.im < 0 ? MINUS + im : im;
    return fmt(z.re, sig) + (z.im < 0 ? ' ' + MINUS + ' ' : ' + ') + im;
  };

  RASim.fmtPoly = function (coeffs, variable, sig) {
    if (variable == null) variable = 's';
    if (sig == null) sig = 4;
    var n = coeffs.length;
    var out = '';
    for (var i = 0; i < n; i++) {
      var c = Number(coeffs[i]);
      var pw = n - 1 - i;
      var mag = fmt(Math.abs(c), sig);
      if (mag === '0') continue;
      var v = pw === 0 ? '' : pw === 1 ? variable : variable + '<sup>' + pw + '</sup>';
      var term = (pw > 0 && mag === '1') ? v : mag + v;
      if (out === '') out = (c < 0 ? MINUS : '') + term;
      else out += (c < 0 ? ' ' + MINUS + ' ' : ' + ') + term;
    }
    return out === '' ? '0' : out;
  };

  /* ------------------------------------------------------------------ *
   * 1.5 Math
   * ------------------------------------------------------------------ */
  var M = {};
  RASim.math = M;

  M.cx = function (re, im) { return { re: re, im: im == null ? 0 : im }; };
  M.cadd = function (a, b) { a = toC(a); b = toC(b); return { re: a.re + b.re, im: a.im + b.im }; };
  M.csub = function (a, b) { a = toC(a); b = toC(b); return { re: a.re - b.re, im: a.im - b.im }; };
  M.cmul = function (a, b) { a = toC(a); b = toC(b); return { re: a.re * b.re - a.im * b.im, im: a.re * b.im + a.im * b.re }; };
  M.cdiv = function (a, b) {
    a = toC(a); b = toC(b);
    // Smith's algorithm
    if (Math.abs(b.re) >= Math.abs(b.im)) {
      if (b.re === 0 && b.im === 0) return { re: a.re / 0, im: a.im / 0 };
      var r = b.im / b.re, d = b.re + b.im * r;
      return { re: (a.re + a.im * r) / d, im: (a.im - a.re * r) / d };
    }
    var r2 = b.re / b.im, d2 = b.re * r2 + b.im;
    return { re: (a.re * r2 + a.im) / d2, im: (a.im * r2 - a.re) / d2 };
  };
  M.cscale = function (a, k) { a = toC(a); return { re: a.re * k, im: a.im * k }; };
  M.cconj = function (a) { a = toC(a); return { re: a.re, im: -a.im }; };
  M.cabs = function (a) { a = toC(a); return Math.hypot(a.re, a.im); };
  M.carg = function (a) { a = toC(a); return Math.atan2(a.im, a.re); };
  M.cexp = function (a) { a = toC(a); var e = Math.exp(a.re); return { re: e * Math.cos(a.im), im: e * Math.sin(a.im) }; };
  M.csqrt = function (a) {
    a = toC(a);
    var r = Math.hypot(a.re, a.im);
    if (r === 0) return { re: 0, im: 0 };
    var re = Math.sqrt((r + a.re) / 2);
    var im = Math.sqrt(Math.max(0, (r - a.re) / 2));
    if (a.im < 0) im = -im;
    return { re: re, im: im };
  };

  M.clamp = function (v, lo, hi) { return v < lo ? lo : v > hi ? hi : v; };
  M.rad = function (d) { return d * Math.PI / 180; };
  M.deg = function (r) { return r * 180 / Math.PI; };
  M.wrapPi = function (a) { return a - TAU * Math.ceil((a - Math.PI) / TAU); };
  M.wrapDeg = function (a) { return a - 360 * Math.ceil((a - 180) / 360); };

  M.linspace = function (a, b, n) {
    n = Math.max(1, Math.round(n));
    if (n === 1) return [a];
    var out = new Array(n);
    for (var i = 0; i < n; i++) out[i] = a + (b - a) * i / (n - 1);
    out[n - 1] = b;
    return out;
  };
  M.logspace = function (a, b, n) {
    n = Math.max(1, Math.round(n));
    if (n === 1) return [a];
    var la = Math.log10(a), lb = Math.log10(b);
    var out = new Array(n);
    for (var i = 0; i < n; i++) out[i] = Math.pow(10, la + (lb - la) * i / (n - 1));
    out[0] = a; out[n - 1] = b;
    return out;
  };

  function clean(v) { return Number(v.toPrecision(12)); }

  M.niceCeil = function (v) {
    if (!(v > 0) || !isFinite(v)) return v;
    var k = Math.floor(Math.log10(v));
    var mults = [1, 2, 2.5, 5, 10];
    for (var kk = k - 1; kk <= k + 1; kk++) {
      var base = Math.pow(10, kk);
      for (var i = 0; i < mults.length; i++) {
        var c = clean(mults[i] * base);
        if (c >= v * (1 - 1e-12)) return c;
      }
    }
    return clean(Math.pow(10, k + 2));
  };

  M.niceTicks = function (min, max, target) {
    if (target == null) target = 6;
    if (!isFinite(min) || !isFinite(max)) return [];
    if (min > max) { var tmp = min; min = max; max = tmp; }
    if (max === min) return [min];
    var step = M.niceCeil((max - min) / Math.max(1, target));
    var i0 = Math.ceil(min / step - 1e-9), i1 = Math.floor(max / step + 1e-9);
    var out = [];
    for (var i = i0; i <= i1 && out.length < 1000; i++) {
      var v = clean(i * step);
      if (Math.abs(v) < step * 1e-9) v = 0;
      out.push(v);
    }
    return out;
  };

  // ---- polynomials (highest power first) ----
  M.polyTrim = function (p) {
    p = Array.prototype.slice.call(p).map(Number);
    var mx = 0;
    for (var i = 0; i < p.length; i++) mx = Math.max(mx, Math.abs(p[i]));
    var tol = mx * 1e-13;
    var k = 0;
    while (k < p.length - 1 && Math.abs(p[k]) <= tol) k++;
    p = p.slice(k);
    if (p.length === 0) return [0];
    if (p.length === 1 && Math.abs(p[0]) <= tol) return [0];
    return p;
  };
  function padTo(p, n) { var out = []; for (var i = 0; i < n - p.length; i++) out.push(0); return out.concat(p); }
  M.polyAdd = function (a, b) {
    var n = Math.max(a.length, b.length); a = padTo(a, n); b = padTo(b, n);
    var out = [];
    for (var i = 0; i < n; i++) out.push(a[i] + b[i]);
    return M.polyTrim(out);
  };
  M.polySub = function (a, b) {
    var n = Math.max(a.length, b.length); a = padTo(a, n); b = padTo(b, n);
    var out = [];
    for (var i = 0; i < n; i++) out.push(a[i] - b[i]);
    return M.polyTrim(out);
  };
  M.polyMul = function (a, b) {
    if (!a.length || !b.length) return [0];
    var out = [];
    for (var k = 0; k < a.length + b.length - 1; k++) out.push(0);
    for (var i = 0; i < a.length; i++) for (var j = 0; j < b.length; j++) out[i + j] += a[i] * b[j];
    return out;
  };
  M.polyScale = function (p, k) { return p.map(function (c) { return c * k; }); };
  M.polyDeriv = function (p) {
    var n = p.length - 1;
    if (n < 1) return [0];
    var out = [];
    for (var i = 0; i < n; i++) out.push(p[i] * (n - i));
    return out;
  };
  M.polyEval = function (p, s) {
    s = toC(s);
    var re = 0, im = 0;
    for (var i = 0; i < p.length; i++) {
      var nr = re * s.re - im * s.im + p[i];
      im = re * s.im + im * s.re;
      re = nr;
    }
    return { re: re, im: im };
  };
  M.polyFromRoots = function (roots) {
    var p = [1];
    var pos = 0, neg = 0;
    for (var i = 0; i < roots.length; i++) {
      var r = toC(roots[i]);
      if (r.im === 0) p = M.polyMul(p, [1, -r.re]);
      else if (r.im > 0) { pos++; p = M.polyMul(p, [1, -2 * r.re, r.re * r.re + r.im * r.im]); }
      else neg++;
    }
    if (pos !== neg) throw new Error('polyFromRoots: complex roots must come in conjugate pairs');
    return p;
  };

  function quadRoots(b, c) {   // roots of s^2 + b s + c (real b, c)
    var disc = b * b - 4 * c;
    if (disc >= 0) {
      var sq = Math.sqrt(disc);
      var q = -(b + (b >= 0 ? sq : -sq)) / 2;
      if (q === 0) return [{ re: 0, im: 0 }, { re: 0, im: 0 }];
      return [{ re: q, im: 0 }, { re: c / q, im: 0 }];
    }
    var im = Math.sqrt(-disc) / 2;
    return [{ re: -b / 2, im: im }, { re: -b / 2, im: -im }];
  }

  // Evaluate p and p' at complex z (Horner), returns [p(z), p'(z)]
  function evalPD(a, zr, zi) {
    var pr = a[0], pi = 0, dr = 0, di = 0;
    for (var i = 1; i < a.length; i++) {
      var ndr = dr * zr - di * zi + pr;
      di = dr * zi + di * zr + pi;
      dr = ndr;
      var npr = pr * zr - pi * zi + a[i];
      pi = pr * zi + pi * zr;
      pr = npr;
    }
    return [pr, pi, dr, di];
  }

  M.polyRoots = function (p) {
    p = M.polyTrim(p);
    if (p.length < 2 || p[0] === 0) return [];
    var a = p.map(function (c) { return c / p[0]; });
    var roots = [];
    while (a.length > 1 && a[a.length - 1] === 0) { a.pop(); roots.push({ re: 0, im: 0 }); }
    var n = a.length - 1;
    if (n === 1) roots.push({ re: -a[1], im: 0 });
    else if (n === 2) roots = roots.concat(quadRoots(a[1], a[2]));
    else if (n > 2) {
      // scale variable: z = R w, so the roots of q(w) have magnitude ~1
      var R = 0;
      for (var k = 1; k <= n; k++) R = Math.max(R, Math.pow(Math.abs(a[k]), 1 / k));
      if (!(R > 0) || !isFinite(R)) R = 1;
      var q = a.map(function (c, i) { return c / Math.pow(R, i); });
      var zr = [], zi = [];
      for (var j = 0; j < n; j++) {
        var ang = TAU * j / n + 0.4;
        var rad = 0.9 + 0.1 * j / n;   // slightly varying radius: non-symmetric start
        zr.push(rad * Math.cos(ang)); zi.push(rad * Math.sin(ang));
      }
      for (var it = 0; it < 800; it++) {
        var maxStep = 0;
        for (var m = 0; m < n; m++) {
          var e = evalPD(q, zr[m], zi[m]);
          if (e[0] === 0 && e[1] === 0) continue;
          var wv = M.cdiv({ re: e[0], im: e[1] }, { re: e[2], im: e[3] });   // Newton correction p/p'
          var sr = 0, si = 0;
          for (var l = 0; l < n; l++) {
            if (l === m) continue;
            var dr = zr[m] - zr[l], di = zi[m] - zi[l];
            var dd = dr * dr + di * di;
            if (dd === 0) { dr = 1e-12; di = 0; dd = 1e-24; }
            sr += dr / dd; si += -di / dd;
          }
          var corr;
          if (!isFinite(wv.re) || !isFinite(wv.im)) {
            corr = { re: 1e-3 * (m + 1), im: 1e-3 };   // p'(z) = 0: nudge
          } else {
            var den = { re: 1 - (wv.re * sr - wv.im * si), im: -(wv.re * si + wv.im * sr) };
            corr = M.cdiv(wv, den);
            if (!isFinite(corr.re) || !isFinite(corr.im)) corr = wv;
          }
          zr[m] -= corr.re; zi[m] -= corr.im;
          var st = Math.hypot(corr.re, corr.im) / Math.max(1, Math.hypot(zr[m], zi[m]));
          if (st > maxStep) maxStep = st;
        }
        if (maxStep < 1e-15) break;
      }
      // Newton polishing on the scaled monic polynomial, accept only if residual decreases
      for (var m2 = 0; m2 < n; m2++) {
        for (var pi = 0; pi < 4; pi++) {
          var e2 = evalPD(q, zr[m2], zi[m2]);
          var res0 = Math.hypot(e2[0], e2[1]);
          if (res0 === 0) break;
          var w2 = M.cdiv({ re: e2[0], im: e2[1] }, { re: e2[2], im: e2[3] });
          if (!isFinite(w2.re) || !isFinite(w2.im)) break;
          var nr = zr[m2] - w2.re, ni = zi[m2] - w2.im;
          var e3 = evalPD(q, nr, ni);
          if (Math.hypot(e3[0], e3[1]) < res0) { zr[m2] = nr; zi[m2] = ni; } else break;
        }
      }
      for (var t = 0; t < n; t++) roots.push({ re: zr[t] * R, im: zi[t] * R });
    }
    // (a) snap tiny imaginary parts
    roots.forEach(function (r) {
      if (Math.abs(r.im) < 1e-9 * Math.max(1, Math.hypot(r.re, r.im))) r.im = 0;
      if (r.re === 0) r.re = 0;   // no -0
    });
    // (b) exact conjugate pairs. The polynomial is real, so the result must be conjugate-symmetric.
    // Greedy, largest |im| first: a root u is paired with the unassigned root v whose conjugate is closest
    // to u, but only when that conjugate is closer to u than u is to the real axis; otherwise u is real
    // (this happens for the members of a repeated real root, which Aberth returns as a small cluster).
    var used = roots.map(function () { return false; });
    for (;;) {
      var u = -1, bi = 0;
      for (var q = 0; q < roots.length; q++) {
        if (!used[q] && roots[q].im !== 0 && Math.abs(roots[q].im) > bi) { bi = Math.abs(roots[q].im); u = q; }
      }
      if (u < 0) break;
      used[u] = true;
      var best = -1, bd = Infinity;
      for (var v = 0; v < roots.length; v++) {
        if (used[v]) continue;
        var d = Math.hypot(roots[u].re - roots[v].re, roots[u].im + roots[v].im);
        if (d < bd) { bd = d; best = v; }
      }
      if (best >= 0 && bd < bi) {
        var re = (roots[u].re + roots[best].re) / 2, im = (Math.abs(roots[u].im) + Math.abs(roots[best].im)) / 2;
        if (im < 1e-9 * Math.max(1, Math.hypot(re, im))) im = 0;
        roots[u] = { re: re, im: im }; roots[best] = { re: re, im: -im };
        if (im === 0) roots[best].im = 0;
        used[best] = true;
      } else {
        roots[u] = { re: roots[u].re, im: 0 };
      }
    }
    // (c) sort: descending real part, then descending imaginary part
    roots.sort(function (x, y) {
      var d = y.re - x.re;
      if (Math.abs(d) > 1e-9 * Math.max(1, Math.abs(x.re), Math.abs(y.re))) return d;
      return y.im - x.im;
    });
    return roots;
  };

  // ---- small dense matrices ----
  function zeros(n, m) { var A = []; for (var i = 0; i < n; i++) { var r = []; for (var j = 0; j < m; j++) r.push(0); A.push(r); } return A; }
  function eye(n) { var A = zeros(n, n); for (var i = 0; i < n; i++) A[i][i] = 1; return A; }
  function matMul(A, B) {
    var n = A.length, m = B[0].length, k = B.length, out = zeros(n, m);
    for (var i = 0; i < n; i++) for (var l = 0; l < k; l++) { var a = A[i][l]; if (a === 0) continue; for (var j = 0; j < m; j++) out[i][j] += a * B[l][j]; }
    return out;
  }
  function matVec(A, x) { return A.map(function (row) { var s = 0; for (var j = 0; j < row.length; j++) s += row[j] * x[j]; return s; }); }
  function solve(A, B) {   // solves A X = B (B matrix), Gaussian elimination with partial pivoting
    var n = A.length, m = B[0].length;
    var a = A.map(function (r) { return r.slice(); }), b = B.map(function (r) { return r.slice(); });
    for (var c = 0; c < n; c++) {
      var piv = c;
      for (var r = c + 1; r < n; r++) if (Math.abs(a[r][c]) > Math.abs(a[piv][c])) piv = r;
      if (piv !== c) { var t = a[c]; a[c] = a[piv]; a[piv] = t; t = b[c]; b[c] = b[piv]; b[piv] = t; }
      var d = a[c][c];
      if (d === 0) throw new Error('solve: singular matrix');
      for (var r2 = c + 1; r2 < n; r2++) {
        var f = a[r2][c] / d;
        if (f === 0) continue;
        for (var j = c; j < n; j++) a[r2][j] -= f * a[c][j];
        for (var j2 = 0; j2 < m; j2++) b[r2][j2] -= f * b[c][j2];
      }
    }
    var X = zeros(n, m);
    for (var i = n - 1; i >= 0; i--) {
      for (var jj = 0; jj < m; jj++) {
        var s = b[i][jj];
        for (var k = i + 1; k < n; k++) s -= a[i][k] * X[k][jj];
        X[i][jj] = s / a[i][i];
      }
    }
    return X;
  }
  M.matMul = matMul;
  M.matVec = matVec;
  M.solve = function (A, b) {
    if (Array.isArray(b[0])) return solve(A, b);
    return solve(A, b.map(function (v) { return [v]; })).map(function (r) { return r[0]; });
  };

  M.expm = function (A) {
    var n = A.length;
    if (n === 0) return [];
    var norm = 0;
    for (var i = 0; i < n; i++) { var s = 0; for (var j = 0; j < n; j++) s += Math.abs(A[i][j]); norm = Math.max(norm, s); }
    var sq = 0;
    if (norm > 0.5) sq = Math.max(0, Math.ceil(Math.log2(norm / 0.5)));
    var scale = Math.pow(2, -sq);
    var X = A.map(function (r) { return r.map(function (v) { return v * scale; }); });
    // Pade (6,6)
    var q = 6, c = 1;
    var N = eye(n), D = eye(n), P = eye(n);
    for (var k = 1; k <= q; k++) {
      c = c * (q - k + 1) / (k * (2 * q - k + 1));
      P = matMul(P, X);
      var sg = (k % 2 === 0) ? 1 : -1;
      for (var a = 0; a < n; a++) for (var b = 0; b < n; b++) { N[a][b] += c * P[a][b]; D[a][b] += sg * c * P[a][b]; }
    }
    var E = solve(D, N);
    for (var t = 0; t < sq; t++) E = matMul(E, E);
    return E;
  };

  M.tfEval = function (num, den, s) { return M.cdiv(M.polyEval(num, s), M.polyEval(den, s)); };

  M.tfStep = function (num, den, tEnd, n) {
    if (n == null) n = 400;
    n = Math.max(1, Math.round(n));
    var nb = M.polyTrim(num), na = M.polyTrim(den);
    if (na.length === 1 && na[0] === 0) throw new Error('tfStep: denominator is zero');
    var degA = na.length - 1, degB = (nb.length === 1 && nb[0] === 0) ? 0 : nb.length - 1;
    if (degB > degA) throw new Error('tfStep: deg(num) = ' + degB + ' > deg(den) = ' + degA + ' (improper transfer function)');
    var t = [], y = [], dt = tEnd / n, k;
    for (k = 0; k <= n; k++) t.push(k === n ? tEnd : k * dt);
    if (degA === 0) {
      var K = nb[0] / na[0];
      for (k = 0; k <= n; k++) y.push(K);
      return { t: t, y: y };
    }
    var a0 = na[0];
    var a = na.map(function (v) { return v / a0; });
    var b = padTo(nb.map(function (v) { return v / a0; }), degA + 1);
    var Dff = b[0];
    var bp = b.map(function (v, i) { return v - Dff * a[i]; });   // bp[0] = 0
    var N = degA;
    // controllable canonical form
    var Mx = zeros(N + 1, N + 1);
    for (var i = 0; i < N - 1; i++) Mx[i][i + 1] = 1;
    for (var j = 0; j < N; j++) Mx[N - 1][j] = -a[N - j];
    Mx[N - 1][N] = 1;   // B = e_N
    var Cv = [];
    for (var j2 = 0; j2 < N; j2++) Cv.push(bp[N - j2]);
    var E = M.expm(Mx.map(function (r) { return r.map(function (v) { return v * dt; }); }));
    var x = []; for (var z = 0; z < N; z++) x.push(0);
    for (k = 0; k <= n; k++) {
      var yk = Dff;
      for (var c = 0; c < N; c++) yk += Cv[c] * x[c];
      y.push(yk);
      if (k === n) break;
      var nx = [];
      for (var r = 0; r < N; r++) {
        var s = E[r][N];
        for (var c2 = 0; c2 < N; c2++) s += E[r][c2] * x[c2];
        nx.push(s);
      }
      x = nx;
    }
    return { t: t, y: y };
  };

  M.rk4 = function (f, t, y, h) {
    var n = y.length, i, tmp = new Array(n);
    var k1 = f(t, y);
    for (i = 0; i < n; i++) tmp[i] = y[i] + h / 2 * k1[i];
    var k2 = f(t + h / 2, tmp.slice());
    for (i = 0; i < n; i++) tmp[i] = y[i] + h / 2 * k2[i];
    var k3 = f(t + h / 2, tmp.slice());
    for (i = 0; i < n; i++) tmp[i] = y[i] + h * k3[i];
    var k4 = f(t + h, tmp.slice());
    var out = new Array(n);
    for (i = 0; i < n; i++) out[i] = y[i] + h / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
    return out;
  };

  M.bisect = function (f, lo, hi, tol, maxIter) {
    if (tol == null) tol = 1e-12;
    if (maxIter == null) maxIter = 200;
    var flo = f(lo);
    if (flo === 0) return lo;
    var fhi = f(hi);
    if (fhi === 0) return hi;
    if (!(flo * fhi < 0)) return NaN;
    for (var i = 0; i < maxIter; i++) {
      var mid = (lo + hi) / 2;
      if (mid <= Math.min(lo, hi) || mid >= Math.max(lo, hi)) break;
      var fm = f(mid);
      if (fm === 0) return mid;
      if ((fm < 0) === (flo < 0)) { lo = mid; flo = fm; } else { hi = mid; fhi = fm; }
      if (Math.abs(hi - lo) <= tol) break;
    }
    return (lo + hi) / 2;
  };

  M.zeroCrossings = function (f, a, b, n, opts) {
    if (n == null) n = 400;
    var log = !!(opts && opts.log);
    var xs = log ? M.logspace(a, b, n + 1) : M.linspace(a, b, n + 1);
    var fs = xs.map(function (x) { try { return f(x); } catch (e) { return NaN; } });
    var out = [];
    for (var i = 0; i < xs.length; i++) {
      if (!isFinite(fs[i])) continue;
      if (fs[i] === 0) { out.push(xs[i]); continue; }
      if (i + 1 < xs.length && isFinite(fs[i + 1]) && fs[i + 1] !== 0 && (fs[i] < 0) !== (fs[i + 1] < 0)) {
        var r = M.bisect(f, xs[i], xs[i + 1]);
        if (isFinite(r)) out.push(r);
      }
    }
    if (a > b) out.reverse();   // always ascending, even when a > b
    return out;
  };

  M.unwrapDeg = function (arr) {
    var out = [], offset = 0, prev = null;
    for (var i = 0; i < arr.length; i++) {
      var v = arr[i];
      if (!isFinite(v)) { out.push(v); continue; }
      if (prev !== null) offset -= 360 * Math.round((v - prev) / 360);
      prev = v;
      out.push(v + offset);
    }
    return out;
  };

  /* ------------------------------------------------------------------ *
   * 1.1 Registry and lifecycle
   * ------------------------------------------------------------------ */
  var booted = false;
  var allAnimators = [];
  var allCanvases = [];
  var globalListeners = false;

  RASim.register = function (id, def) {
    if (typeof id !== 'string' || !id) throw new Error('RASim.register: id must be a non-empty string');
    if (Object.prototype.hasOwnProperty.call(RASim.sims, id)) throw new Error('RASim.register: duplicate simulation id "' + id + '"');
    if (!def || typeof def.mount !== 'function') throw new Error('RASim.register("' + id + '"): def.mount must be a function');
    RASim.sims[id] = def;
    if (booted && HAS_DOM) RASim.mountAll();
    return def;
  };

  RASim.css = function (key, cssText) {
    if (!HAS_DOM) return;
    var sel = 'style[data-rasim-css="' + String(key).replace(/"/g, '\\"') + '"]';
    if (document.head.querySelector(sel)) return;
    var st = document.createElement('style');
    st.setAttribute('data-rasim-css', key);
    st.textContent = cssText;
    document.head.appendChild(st);
  };

  function ensureGlobalListeners() {
    if (globalListeners) return;
    globalListeners = true;
    document.addEventListener('visibilitychange', function () { allAnimators.forEach(function (a) { a._wake(); }); });
    window.addEventListener('resize', function () { allCanvases.forEach(function (c) { c._resize(); }); });
    watchDpr();
  }
  // devicePixelRatio can change without a resize event (window moved to another screen): watch it with matchMedia.
  function watchDpr() {
    if (typeof window.matchMedia !== 'function') return;
    var mq = window.matchMedia('(resolution: ' + (window.devicePixelRatio || 1) + 'dppx)');
    var fired = false;
    var onChange = function () {
      if (fired) return;
      fired = true;
      if (mq.removeEventListener) mq.removeEventListener('change', onChange); else if (mq.removeListener) mq.removeListener(onChange);
      allCanvases.forEach(function (c) { c._resize(); });
      watchDpr();
    };
    if (mq.addEventListener) mq.addEventListener('change', onChange); else if (mq.addListener) mq.addListener(onChange);
  }

  function isInViewport(elm) {
    var r = elm.getBoundingClientRect();
    var vh = window.innerHeight || document.documentElement.clientHeight;
    var vw = window.innerWidth || document.documentElement.clientWidth;
    return r.bottom > 0 && r.top < vh && r.right > 0 && r.left < vw && (r.width > 0 || r.height > 0);
  }

  function setupPanel(node, id, def) {
    var st = { id: id, def: def, panel: node, body: null, collapsed: false, visible: true, mounted: false, animators: [], canvases: [] };
    node.__rasim = st;
    node.id = 'sim-' + id;
    node.innerHTML = '';
    var head = document.createElement('div');
    head.className = 'sim-head';
    var badge = document.createElement('span'); badge.className = 'sim-badge'; badge.textContent = 'Interactive';
    var title = document.createElement('span'); title.className = 'sim-title'; title.textContent = def.title || id;
    var btn = document.createElement('button'); btn.type = 'button'; btn.className = 'sim-toggle';
    btn.setAttribute('aria-expanded', 'true'); btn.textContent = 'Hide';
    head.appendChild(badge); head.appendChild(title); head.appendChild(btn);
    var body = document.createElement('div');
    body.className = 'sim-body sim-pending';
    body.id = 'sim-' + id + '-body';
    btn.setAttribute('aria-controls', body.id);
    node.appendChild(head); node.appendChild(body);
    st.body = body;

    btn.addEventListener('click', function () {
      st.collapsed = !st.collapsed;
      node.classList.toggle('sim-collapsed', st.collapsed);
      btn.textContent = st.collapsed ? 'Show' : 'Hide';
      btn.setAttribute('aria-expanded', st.collapsed ? 'false' : 'true');
      if (!st.collapsed) st.canvases.forEach(function (c) { c._resize(); c.redraw(); });
      st.animators.forEach(function (a) { a._wake(); });
    });

    // narrow-panel class (fallback for container queries)
    if (typeof ResizeObserver !== 'undefined') {
      var ro = new ResizeObserver(function () {
        var w = node.getBoundingClientRect().width;
        if (w > 0) node.classList.toggle('sim-narrow', w < 720);
      });
      ro.observe(node);
    }

    // viewport visibility for animation (rootMargin 0)
    if (typeof IntersectionObserver !== 'undefined') {
      st.visible = isInViewport(node);
      var vio = new IntersectionObserver(function (entries) {
        var e = entries[entries.length - 1];
        st.visible = e.isIntersecting;
        st.animators.forEach(function (a) { a._wake(); });
      }, { rootMargin: '0px' });
      vio.observe(node);
    } else {
      st.visible = true;
    }
    return st;
  }

  function makeApi(st) {
    var api = {
      id: st.id,
      root: st.body,
      panel: st.panel,
      animate: function (fn) { return makeAnimator(st, fn); }
    };
    return api;
  }

  function mountPanel(st) {
    if (st.mounted) return;
    st.mounted = true;
    st.body.classList.remove('sim-pending');
    var api = makeApi(st);
    try {
      var ret = st.def.mount(api);
      RASim.instances[st.id] = (ret && typeof ret === 'object') ? ret : {};
    } catch (err) {
      console.error('RASim: simulation "' + st.id + '" failed to mount:', err);
      st.animators.forEach(function (a) { a.stop(); });
      st.body.innerHTML = '<p class="sim-error">This simulation failed to load.</p>';
    }
  }

  var lazyIO = null;
  function getLazyIO() {
    if (lazyIO) return lazyIO;
    lazyIO = new IntersectionObserver(function (entries) {
      entries.forEach(function (e) {
        if (!e.isIntersecting) return;
        lazyIO.unobserve(e.target);
        if (e.target.__rasim) mountPanel(e.target.__rasim);
      });
    }, { rootMargin: '400px' });
    return lazyIO;
  }

  function buildIndex() {
    var idx = document.querySelector('[data-sim-index]');
    if (!idx) return;
    var items = [];
    var nodes = document.querySelectorAll('.sim[data-sim]');
    for (var i = 0; i < nodes.length; i++) {
      var st = nodes[i].__rasim;
      if (st) items.push(st);
    }
    if (!items.length) { idx.innerHTML = ''; return; }
    var html = '<h2 class="sim-index-title">Interactive simulations</h2><ol>';
    items.forEach(function (st) {
      html += '<li><a href="#sim-' + escapeHtml(st.id) + '">' + escapeHtml(st.def.title || st.id) + '</a></li>';
    });
    idx.innerHTML = html + '</ol>';
  }

  function escapeHtml(s) {
    return String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/"/g, '&quot;');
  }

  RASim.mountAll = function () {
    if (!HAS_DOM) return;
    booted = true;
    ensureGlobalListeners();
    var eager = window.RASIM_EAGER === true || typeof IntersectionObserver === 'undefined';
    var nodes = document.querySelectorAll('.sim[data-sim]');
    var seen = {};
    for (var i = 0; i < nodes.length; i++) {
      var node = nodes[i];
      var id = node.getAttribute('data-sim');
      if (node.__rasim) { seen[id] = true; continue; }
      var def = Object.prototype.hasOwnProperty.call(RASim.sims, id) ? RASim.sims[id] : null;
      if (!def) {
        if (!node.__rasimWarned) {
          node.__rasimWarned = true;
          console.warn('RASim: no simulation registered for placeholder data-sim="' + id + '"');
        }
        continue;
      }
      if (seen[id]) {
        if (!node.__rasimWarned) {
          node.__rasimWarned = true;
          console.warn('RASim: duplicate placeholder for "' + id + '" ignored');
        }
        continue;
      }
      seen[id] = true;
      var st = setupPanel(node, id, def);
      if (eager) mountPanel(st);
      else getLazyIO().observe(node);
    }
    buildIndex();
  };

  /* ------------------------------------------------------------------ *
   * 1.2 api.animate
   * ------------------------------------------------------------------ */
  function makeAnimator(st, fn) {
    var started = false, rafId = 0, last = null, total = 0;
    function canRun() {
      return started && st.visible && !st.collapsed &&
        (typeof document === 'undefined' || document.visibilityState === undefined || document.visibilityState === 'visible');
    }
    function frame(ts) {
      rafId = 0;
      if (!canRun()) { last = null; return; }   // pause: no loop kept alive
      var dt;
      if (last === null) dt = 1 / 60;
      else dt = Math.min(0.05, Math.max(0, (ts - last) / 1000));
      last = ts;
      total += dt;
      try {
        fn(dt, total);
      } catch (err) {
        console.error('RASim: animation callback of "' + st.id + '" threw; animation stopped.', err);
        ctl.stop();
        return;
      }
      if (canRun() && !rafId) rafId = requestAnimationFrame(frame);
    }
    var ctl = {
      start: function () {
        started = true;
        ctl._wake();
      },
      stop: function () {
        started = false;
        if (rafId) { cancelAnimationFrame(rafId); rafId = 0; }
        last = null;
      },
      toggle: function () { if (started) ctl.stop(); else ctl.start(); },
      _wake: function () {
        if (canRun()) {
          if (!rafId) { last = null; rafId = requestAnimationFrame(frame); }
        } else if (rafId) {
          cancelAnimationFrame(rafId); rafId = 0; last = null;
        }
      }
    };
    Object.defineProperty(ctl, 'running', { get: function () { return started; }, enumerable: true });
    st.animators.push(ctl);
    allAnimators.push(ctl);
    return ctl;
  }

  /* ------------------------------------------------------------------ *
   * 1.6 UI builders
   * ------------------------------------------------------------------ */
  var ui = {};
  RASim.ui = ui;

  function el(tag, props, parent) {
    var e = document.createElement(tag);
    props = props || {};
    if (props['class'] != null) e.className = props['class'];
    if (props.text != null) e.textContent = props.text;
    if (props.html != null) e.innerHTML = props.html;
    if (props.title != null) e.title = props.title;
    if (props.style) Object.keys(props.style).forEach(function (k) {
      if (k.indexOf('-') >= 0) e.style.setProperty(k, props.style[k]); else e.style[k] = props.style[k];
    });
    if (props.attrs) Object.keys(props.attrs).forEach(function (k) { e.setAttribute(k, props.attrs[k]); });
    if (props.on) Object.keys(props.on).forEach(function (k) { e.addEventListener(k, props.on[k]); });
    if (parent) parent.appendChild(e);
    return e;
  }
  ui.el = el;

  function slug(html) {
    var t = String(html == null ? '' : html).replace(/<[^>]*>/g, '').replace(/&[a-z]+;/gi, ' ').toLowerCase();
    t = t.replace(/[^\p{L}\p{N}]+/gu, '-').replace(/^-+|-+$/g, '');
    return t || 'control';
  }
  function keyOf(opts) { return opts.key != null ? String(opts.key) : slug(opts.label); }
  var uidCounter = 0;
  function uid(prefix) { return 'rasim-' + prefix + '-' + (++uidCounter); }

  function decimalsOf(step) {
    var s = String(step);
    var m = s.match(/e-(\d+)$/i);
    if (m) {
      var mant = s.split(/e/i)[0];
      var md = mant.indexOf('.') >= 0 ? mant.split('.')[1].length : 0;
      return parseInt(m[1], 10) + md;
    }
    return s.indexOf('.') >= 0 ? s.split('.')[1].length : 0;
  }

  ui.split = function (parent, opts) {
    var ratio = (opts && opts.ratio != null) ? opts.ratio : 1.5;
    var root = el('div', { 'class': 'sim-split' }, parent);
    root.style.gridTemplateColumns = 'minmax(0, ' + ratio + 'fr) minmax(230px, 1fr)';
    var main = el('div', { 'class': 'sim-split-main' }, root);
    var side = el('div', { 'class': 'sim-split-side' }, root);
    return { el: root, main: main, side: side };
  };

  ui.row = function (parent) { return el('div', { 'class': 'sim-row' }, parent); };

  ui.group = function (parent, title) {
    var g = el('div', { 'class': 'sim-group' }, parent);
    if (title != null && title !== '') el('div', { 'class': 'sim-group-title', html: title }, g);
    return el('div', { 'class': 'sim-group-body' }, g);
  };

  ui.slider = function (parent, opts) {
    var key = keyOf(opts);
    var min = Number(opts.min), max = Number(opts.max);
    var step = opts.step != null ? Number(opts.step) : 1;
    var log = opts.scale === 'log';
    var decimals = opts.decimals;
    var onInput = opts.onInput;
    var POS = 399;
    var wrap = el('div', { 'class': 'sim-ctl sim-slider' + (log ? ' sim-log' : '') }, parent);
    wrap.setAttribute('data-slider', key);
    var rid = uid('r');
    var lab = el('label', { 'class': 'sim-label', html: opts.label || '' }, wrap);
    lab.setAttribute('for', rid);
    var range = el('input', { attrs: { type: 'range', id: rid, 'data-control': key } }, wrap);
    var num = el('input', { 'class': 'sim-num', attrs: { type: 'number', step: 'any', inputmode: 'decimal', 'data-control-num': key } }, wrap);
    num.setAttribute('aria-label', lab.textContent || key);
    var unit = el('span', { 'class': 'sim-unit', html: opts.unit || '' }, wrap);
    var cur;

    function l0() { return Math.log10(min); }
    function l1() { return Math.log10(max); }
    function toPos(v) {
      if (max <= min) return 0;
      return Math.round(M.clamp((Math.log10(v) - l0()) / (l1() - l0()), 0, 1) * POS);
    }
    function fromPos(p) {
      if (p <= 0) return min;
      if (p >= POS) return max;
      var v = Math.pow(10, l0() + (l1() - l0()) * p / POS);
      return M.clamp(Number(v.toPrecision(3)), min, max);
    }
    function fmtNum(v) {
      if (decimals != null) return Number(v).toFixed(decimals);
      if (log) return String(Number(Number(v).toPrecision(3)));
      return Number(v).toFixed(decimalsOf(step));
    }
    function applyRangeAttrs() {
      if (log) { range.min = 0; range.max = POS; range.step = 1; }
      else { range.min = min; range.max = max; range.step = step; }
      num.min = min; num.max = max;
    }
    function show(v, numText) {
      range.value = log ? toPos(v) : v;
      num.value = numText != null ? numText : fmtNum(v);
    }
    applyRangeAttrs();
    cur = M.clamp(opts.value != null ? Number(opts.value) : min, min, max);
    show(cur);

    range.addEventListener('input', function () {
      var p = Number(range.value);
      cur = log ? fromPos(p) : M.clamp(p, min, max);
      num.value = fmtNum(cur);
      lastText = num.value;   // the box now shows the track value: a later typed value is compared with this text
      if (onInput) onInput(cur);
    });
    var lastText = num.value;
    function commit() {
      if (num.value === lastText) return;
      var v = parseFloat(num.value);
      if (!isFinite(v)) { num.value = fmtNum(cur); lastText = num.value; return; }
      var c = M.clamp(v, min, max);
      cur = c;
      range.value = log ? toPos(c) : c;
      if (c !== v) num.value = fmtNum(c);
      lastText = num.value;
      if (onInput) onInput(cur);
    }
    num.addEventListener('change', commit);
    num.addEventListener('keydown', function (e) { if (e.key === 'Enter') commit(); });

    var api = {
      el: wrap,
      get: function () { return cur; },
      set: function (v) {
        v = Number(v);
        if (!isFinite(v)) return;
        cur = M.clamp(v, min, max);
        show(cur);
        lastText = num.value;
      },
      setRange: function (a, b) {
        min = Number(a); max = Number(b);
        applyRangeAttrs();
        cur = M.clamp(cur, min, max);
        show(cur);
        lastText = num.value;
      },
      setDisabled: function (flag) {
        range.disabled = !!flag; num.disabled = !!flag;
        wrap.classList.toggle('sim-disabled', !!flag);
      },
      setLabel: function (html) { lab.innerHTML = html; num.setAttribute('aria-label', lab.textContent || key); }
    };
    return api;
  };

  ui.number = function (parent, opts) {
    var key = keyOf(opts);
    var wrap = el('div', { 'class': 'sim-ctl sim-number' }, parent);
    var nid = uid('n');
    var lab = el('label', { 'class': 'sim-label', html: opts.label || '' }, wrap);
    lab.setAttribute('for', nid);
    var num = el('input', { 'class': 'sim-num', attrs: { type: 'number', id: nid, step: 'any', inputmode: 'decimal', 'data-control': key } }, wrap);
    if (opts.unit) el('span', { 'class': 'sim-unit', html: opts.unit }, wrap);
    var min = opts.min != null ? Number(opts.min) : -Infinity;
    var max = opts.max != null ? Number(opts.max) : Infinity;
    if (isFinite(min)) num.min = min;
    if (isFinite(max)) num.max = max;
    var decimals = opts.decimals != null ? opts.decimals : (opts.step != null ? decimalsOf(opts.step) : null);
    function fmtNum(v) { return decimals != null ? Number(v).toFixed(decimals) : String(Number(Number(v).toPrecision(12))); }
    var cur = M.clamp(opts.value != null ? Number(opts.value) : (isFinite(min) ? min : 0), min, max);
    num.value = fmtNum(cur);
    var lastText = num.value;
    function commit() {
      if (num.value === lastText) return;
      var v = parseFloat(num.value);
      if (!isFinite(v)) { num.value = fmtNum(cur); lastText = num.value; return; }
      var c = M.clamp(v, min, max);
      cur = c;
      if (c !== v) num.value = fmtNum(c);
      lastText = num.value;
      if (opts.onInput) opts.onInput(cur);
    }
    num.addEventListener('change', commit);
    num.addEventListener('keydown', function (e) { if (e.key === 'Enter') commit(); });
    return {
      el: wrap,
      get: function () { return cur; },
      set: function (v) { v = Number(v); if (!isFinite(v)) return; cur = M.clamp(v, min, max); num.value = fmtNum(cur); lastText = num.value; },
      setDisabled: function (flag) { num.disabled = !!flag; wrap.classList.toggle('sim-disabled', !!flag); }
    };
  };

  ui.select = function (parent, opts) {
    var key = keyOf(opts);
    var wrap = el('div', { 'class': 'sim-ctl sim-select' }, parent);
    var sid = uid('s');
    if (opts.label) { var lab = el('label', { 'class': 'sim-label', html: opts.label }, wrap); lab.setAttribute('for', sid); }
    var sel = el('select', { attrs: { id: sid, 'data-control': key } }, wrap);
    function setOptions(options) {
      var prev = sel.value;
      sel.innerHTML = '';
      (options || []).forEach(function (o) {
        var op = document.createElement('option');
        op.value = String(o.value); op.innerHTML = o.label != null ? o.label : String(o.value);
        sel.appendChild(op);
      });
      var vals = (options || []).map(function (o) { return String(o.value); });
      if (vals.indexOf(prev) >= 0) sel.value = prev;
      else if (vals.length) sel.value = vals[0];
    }
    setOptions(opts.options);
    if (opts.value != null) sel.value = String(opts.value);
    sel.addEventListener('change', function () { if (opts.onChange) opts.onChange(sel.value); });
    return {
      el: wrap,
      get: function () { return sel.value; },
      set: function (v) { sel.value = String(v); },
      setOptions: setOptions
    };
  };

  ui.segmented = function (parent, opts) {
    var key = keyOf(opts);
    var wrap = el('div', { 'class': 'sim-ctl sim-segmented' }, parent);
    if (opts.label) el('span', { 'class': 'sim-label', html: opts.label }, wrap);
    var group = el('div', { 'class': 'sim-seg', attrs: { role: 'group', 'data-control': key } }, wrap);
    var cur = opts.value != null ? String(opts.value) : (opts.options.length ? String(opts.options[0].value) : '');
    var buttons = opts.options.map(function (o) {
      var b = el('button', { html: o.label != null ? o.label : String(o.value), attrs: { type: 'button', 'data-value': String(o.value) } }, group);
      b.addEventListener('click', function () {
        var v = String(o.value);
        if (v === cur) return;
        cur = v; sync();
        if (opts.onChange) opts.onChange(v);
      });
      return b;
    });
    function sync() {
      buttons.forEach(function (b) {
        var on = b.getAttribute('data-value') === cur;
        b.classList.toggle('active', on);
        b.setAttribute('aria-pressed', on ? 'true' : 'false');
      });
    }
    sync();
    return {
      el: wrap,
      get: function () { return cur; },
      set: function (v) { cur = String(v); sync(); }
    };
  };

  ui.checkbox = function (parent, opts) {
    var key = keyOf(opts);
    var wrap = el('label', { 'class': 'sim-ctl sim-check' }, parent);
    var box = el('input', { attrs: { type: 'checkbox', 'data-control': key } }, wrap);
    box.checked = !!opts.checked;
    el('span', { html: opts.label || '' }, wrap);
    box.addEventListener('change', function () { if (opts.onChange) opts.onChange(box.checked); });
    return {
      el: wrap,
      get: function () { return box.checked; },
      set: function (flag) { box.checked = !!flag; }
    };
  };

  ui.button = function (parent, opts) {
    var key = keyOf(opts);
    var b = el('button', { 'class': 'sim-btn' + (opts.primary ? ' primary' : ''), html: opts.label || '', attrs: { type: 'button', 'data-control': key } }, parent);
    if (opts.title) b.title = opts.title;
    if (opts.onClick) b.addEventListener('click', function (e) { opts.onClick(e); });
    return b;
  };

  ui.stats = function (parent) { return el('div', { 'class': 'sim-stats' }, parent); };

  ui.stat = function (container, opts) {
    var key = keyOf(opts);
    var tile = el('div', { 'class': 'sim-stat', attrs: { 'data-stat': key } }, container);
    if (opts.title) tile.title = opts.title;
    el('div', { 'class': 'sim-stat-label', html: opts.label || '' }, tile);
    var val = el('div', { 'class': 'sim-stat-value', html: '—' }, tile);
    var lastHtml = '—', lastState = '';
    return {
      el: tile,
      set: function (html) { html = String(html); if (html !== lastHtml) { val.innerHTML = html; lastHtml = html; } },
      setState: function (state) {
        state = state || '';
        if (state === lastState) return;
        tile.classList.remove('sim-ok', 'sim-warn', 'sim-bad');
        if (state) tile.classList.add('sim-' + state);
        lastState = state;
      }
    };
  };

  ui.tag = function (parent, opts) {
    opts = opts || {};
    var t = el('span', { 'class': 'sim-tag' }, parent);
    t.setAttribute('data-tag', opts.key != null ? String(opts.key) : slug(opts.text || 'tag'));
    function set(text, state) {
      t.textContent = text == null ? '' : String(text);
      t.classList.remove('sim-ok', 'sim-warn', 'sim-bad');
      if (state) t.classList.add('sim-' + state);
    }
    set(opts.text || '', opts.state || '');
    return { el: t, set: set };
  };

  ui.matrix = function (parent, opts) {
    opts = opts || {};
    var wrap = el('div', { 'class': 'sim-matrix-wrap' }, parent);
    wrap.setAttribute('data-matrix', opts.key != null ? String(opts.key) : slug(opts.label || 'matrix'));
    if (opts.label) el('span', { 'class': 'sim-matrix-label', html: opts.label }, wrap);
    var table = el('table', { 'class': 'sim-matrix' }, wrap);
    var tbody = el('tbody', {}, table);
    var cells = [], shape = '';
    return {
      el: wrap,
      set: function (rows, o) {
        o = o || {};
        var dec = o.decimals != null ? o.decimals : 3;
        var nr = rows.length, nc = nr ? rows[0].length : 0;
        var sh = nr + 'x' + nc;
        if (sh !== shape) {
          tbody.innerHTML = ''; cells = [];
          for (var i = 0; i < nr; i++) {
            var tr = el('tr', {}, tbody), rc = [];
            for (var j = 0; j < nc; j++) rc.push({ td: el('td', {}, tr), text: null, color: null });
            cells.push(rc);
          }
          shape = sh;
        }
        for (var r = 0; r < nr; r++) for (var c = 0; c < nc; c++) {
          var v = rows[r][c];
          var txt = typeof v === 'number' ? RASim.fmtFixed(v, dec) : String(v);
          var cell = cells[r][c];
          if (cell.text !== txt) { cell.td.textContent = txt; cell.text = txt; }
          var col = (o.colColors && o.colColors[c]) || (o.rowColors && o.rowColors[r]) || '';
          if (cell.color !== col) { cell.td.style.color = col; cell.color = col; }
        }
      }
    };
  };

  ui.legend = function (parent, items) {
    var wrap = el('div', { 'class': 'sim-legend' }, parent);
    function set(list) {
      wrap.innerHTML = '';
      (list || []).forEach(function (it) {
        var item = el('span', { 'class': 'sim-legend-item' }, wrap);
        var sw = el('span', { 'class': 'sim-swatch' }, item);
        sw.style.borderTopColor = it.color || C.ink;
        sw.style.borderTopStyle = it.dash ? 'dashed' : 'solid';
        el('span', { html: it.label || '' }, item);
      });
    }
    set(items);
    return { el: wrap, set: set };
  };

  ui.note = function (parent, html) { return el('p', { 'class': 'sim-note', html: html }, parent); };

  ui.hints = function (parent, items) {
    var d = el('details', { 'class': 'sim-hints', attrs: { open: '' } }, parent);
    el('summary', { text: 'Try this' }, d);
    var ol = el('ol', {}, d);
    (items || []).forEach(function (h) { el('li', { html: h }, ol); });
    return d;
  };

  /* ------------------------------------------------------------------ *
   * 1.7 Canvas
   * ------------------------------------------------------------------ */
  function panelStateOf(node) {
    var p = node && node.closest ? node.closest('.sim') : null;
    return p && p.__rasim ? p.__rasim : null;
  }

  RASim.canvas = function (parent, opts) {
    opts = opts || {};
    var aspect = opts.aspect != null ? opts.aspect : 1.6;
    var minH = opts.minHeight != null ? opts.minHeight : 140;
    var maxH = opts.maxHeight != null ? opts.maxHeight : 560;
    var canvas = el('canvas', { 'class': 'sim-canvas' }, parent);
    canvas.setAttribute('data-canvas', opts.key != null ? String(opts.key) : 'canvas');
    if (opts.maxWidth) canvas.style.maxWidth = opts.maxWidth + 'px';
    var ctx = canvas.getContext('2d');
    var cw = 0, ch = 0, dpr = 0, sx = 1, sy = 1;
    var rafPending = 0, needDraw = false, lastErr = false;
    var dragHandlers = [], hoverHandlers = [];

    var cv = {
      canvas: canvas,
      ctx: ctx,
      draw: null,
      redraw: function () {
        if (rafPending) return;
        rafPending = requestAnimationFrame(function () { rafPending = 0; cv.drawNow(); });
      },
      drawNow: function () {
        if (rafPending) { cancelAnimationFrame(rafPending); rafPending = 0; }
        if (cw <= 0 || ch <= 0) resize(true);
        if (cw <= 0 || ch <= 0) { needDraw = true; return; }
        if ((window.devicePixelRatio || 1) !== dpr) resize(true);
        needDraw = false;
        ctx.setTransform(sx, 0, 0, sy, 0, 0);
        ctx.clearRect(0, 0, cw, ch);
        if (typeof cv.draw !== 'function') return;
        ctx.save();
        try {
          cv.draw(ctx, cw, ch);
          lastErr = false;
        } catch (err) {
          if (!lastErr) console.error('RASim: canvas draw failed:', err);
          lastErr = true;
        }
        ctx.restore();
      },
      setAspect: function (a) { aspect = a; resize(false); cv.redraw(); },
      onDrag: function (h) { dragHandlers.push(h); setupPointer(); },
      onHover: function (h) { hoverHandlers.push(h); setupPointer(); },
      _resize: function () { resize(false); }
    };
    Object.defineProperty(cv, 'width', { get: function () { return cw; }, enumerable: true });
    Object.defineProperty(cv, 'height', { get: function () { return ch; }, enumerable: true });

    function resize(silent) {
      var r = canvas.getBoundingClientRect();
      var bw = canvas.clientLeft || 0;
      var w = Math.max(0, r.width - 2 * bw);
      var d = window.devicePixelRatio || 1;
      if (w <= 0) { cw = 0; ch = 0; needDraw = true; return false; }
      var h = Math.round(M.clamp(w / aspect, minH, maxH));
      var styleH = (h + 2 * bw) + 'px';
      if (canvas.style.height !== styleH) canvas.style.height = styleH;
      var bwPx = Math.max(1, Math.round(w * d)), bhPx = Math.max(1, Math.round(h * d));
      var changed = Math.abs(w - cw) > 0.01 || h !== ch || d !== dpr || canvas.width !== bwPx || canvas.height !== bhPx;
      if (!changed) {
        if (needDraw && !silent) cv.drawNow();
        return false;
      }
      cw = w; ch = h; dpr = d;
      canvas.width = bwPx; canvas.height = bhPx;
      sx = bwPx / w; sy = bhPx / h;
      if (!silent) cv.drawNow();   // redraw immediately: no blank frame after a resize
      return true;
    }

    var pst = panelStateOf(parent);
    if (pst) pst.canvases.push(cv);
    allCanvases.push(cv);
    ensureGlobalListeners();
    if (typeof ResizeObserver !== 'undefined') {
      new ResizeObserver(function () { resize(false); }).observe(canvas);
    }
    resize(true);
    cv.redraw();

    function pos(e) {
      var r = canvas.getBoundingClientRect();
      return { x: e.clientX - r.left - (canvas.clientLeft || 0), y: e.clientY - r.top - (canvas.clientTop || 0) };
    }
    var pointerReady = false, drag = null, touchHit = false;
    // info = { pointerType: 'mouse' | 'pen' | 'touch' }: lets a handler refuse touches (so they scroll the page)
    function tryHit(x, y, info) {
      for (var i = 0; i < dragHandlers.length; i++) {
        var tok = dragHandlers[i].hit ? dragHandlers[i].hit(x, y, info) : null;
        if (tok) return { h: dragHandlers[i], token: tok };
      }
      return null;
    }
    function setupPointer() {
      if (pointerReady) return;
      pointerReady = true;
      canvas.addEventListener('pointerdown', function (e) {
        if (e.pointerType === 'mouse' && e.button !== 0) return;
        if (!dragHandlers.length) return;
        var p = pos(e);
        var hit = tryHit(p.x, p.y, { pointerType: e.pointerType || 'mouse' });
        touchHit = !!hit;
        if (!hit) return;
        e.preventDefault();
        try { canvas.setPointerCapture(e.pointerId); } catch (err) { /* ignore */ }
        drag = { id: e.pointerId, h: hit.h, token: hit.token };
        canvas.classList.add('sim-dragging');
      });
      canvas.addEventListener('pointermove', function (e) {
        var p = pos(e);
        if (drag && e.pointerId === drag.id) {
          drag.h.move && drag.h.move(p.x, p.y, drag.token);
          return;
        }
        if (e.pointerType !== 'touch') {
          var cur = null;
          for (var i = 0; i < dragHandlers.length && !cur; i++) if (dragHandlers[i].cursor) cur = dragHandlers[i].cursor(p.x, p.y);
          canvas.style.cursor = cur || '';
        }
      });
      function finish(e) {
        if (!drag || e.pointerId !== drag.id) return;
        var d = drag; drag = null;
        canvas.classList.remove('sim-dragging');
        try { if (canvas.hasPointerCapture(e.pointerId)) canvas.releasePointerCapture(e.pointerId); } catch (err) { /* ignore */ }
        if (d.h.end) d.h.end(d.token);
      }
      canvas.addEventListener('pointerup', finish);
      canvas.addEventListener('pointercancel', finish);
      canvas.addEventListener('lostpointercapture', finish);
      // touch: block page scrolling only when the touch starts on a handle
      canvas.addEventListener('touchstart', function (e) {
        if (touchHit || drag) { e.preventDefault(); return; }
        if (!dragHandlers.length || !e.touches.length) return;
        var t = e.touches[0];
        var p = pos(t);
        if (tryHit(p.x, p.y, { pointerType: 'touch' })) e.preventDefault();
      }, { passive: false });
      canvas.addEventListener('touchmove', function (e) { if (drag) e.preventDefault(); }, { passive: false });
      canvas.addEventListener('pointermove', function (e) {
        if (e.pointerType === 'touch') return;
        var p = pos(e);
        hoverHandlers.forEach(function (h) { if (h.move) h.move(p.x, p.y); });
      });
      canvas.addEventListener('pointerleave', function (e) {
        if (e.pointerType === 'touch') return;
        hoverHandlers.forEach(function (h) { if (h.leave) h.leave(); });
      });
    }
    return cv;
  };

  /* ------------------------------------------------------------------ *
   * 1.9 Free-form drawing helpers
   * ------------------------------------------------------------------ */
  var draw = {};
  RASim.draw = draw;

  function arrowHead(ctx, x1, y1, x2, y2, head, color) {
    var a = Math.atan2(y2 - y1, x2 - x1);
    var w = head * 0.45;
    ctx.save();
    ctx.setLineDash([]);
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.moveTo(x2, y2);
    ctx.lineTo(x2 - head * Math.cos(a) + w * Math.sin(a), y2 - head * Math.sin(a) - w * Math.cos(a));
    ctx.lineTo(x2 - head * Math.cos(a) - w * Math.sin(a), y2 - head * Math.sin(a) + w * Math.cos(a));
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  }

  draw.arrow = function (ctx, x1, y1, x2, y2, o) {
    o = o || {};
    var color = o.color || C.ink, width = o.width != null ? o.width : 2, head = o.head != null ? o.head : 8;
    var L = Math.hypot(x2 - x1, y2 - y1);
    if (!(L > 1e-9)) return;
    var hd = Math.min(head, L * 0.6);
    var ex = x2 - (x2 - x1) / L * hd * 0.8, ey = y2 - (y2 - y1) / L * hd * 0.8;
    ctx.save();
    ctx.strokeStyle = color; ctx.lineWidth = width; ctx.lineCap = 'round';
    ctx.setLineDash(o.dash || []);
    ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(ex, ey); ctx.stroke();
    ctx.restore();
    arrowHead(ctx, x1, y1, x2, y2, hd, color);
  };

  function textBg(ctx, str, x, y, align, baseline, bg) {
    var m = ctx.measureText(str);
    var w = m.width;
    var fontPx = parseFloat((ctx.font.match(/(\d+(?:\.\d+)?)px/) || [0, 12])[1]);
    var h = fontPx * 1.25;
    var left = align === 'center' ? x - w / 2 : (align === 'right' || align === 'end') ? x - w : x;
    var top = baseline === 'top' ? y - fontPx * 0.12 : baseline === 'bottom' ? y - h + fontPx * 0.12 : baseline === 'alphabetic' ? y - fontPx * 0.95 : y - h / 2;
    var pad = 3;
    roundRect(ctx, left - pad, top - 1, w + 2 * pad, h + 2, 3);
    ctx.fillStyle = bg;
    ctx.fill();
  }
  function roundRect(ctx, x, y, w, h, r) {
    r = Math.min(r, w / 2, h / 2);
    ctx.beginPath();
    ctx.moveTo(x + r, y);
    ctx.arcTo(x + w, y, x + w, y + h, r);
    ctx.arcTo(x + w, y + h, x, y + h, r);
    ctx.arcTo(x, y + h, x, y, r);
    ctx.arcTo(x, y, x + w, y, r);
    ctx.closePath();
  }

  draw.text = function (ctx, str, x, y, o) {
    o = o || {};
    ctx.save();
    ctx.font = o.font || '12px sans-serif';
    ctx.textAlign = o.align || 'left';
    ctx.textBaseline = o.baseline || 'middle';
    str = String(str);
    if (o.bg) textBg(ctx, str, x, y, ctx.textAlign, ctx.textBaseline, o.bg);
    ctx.fillStyle = o.color || C.ink;
    ctx.fillText(str, x, y);
    ctx.restore();
  };

  draw.spring = function (ctx, x1, y1, x2, y2, o) {
    o = o || {};
    var coils = o.coils != null ? o.coils : 8, amp = o.amplitude != null ? o.amplitude : 8;
    var L = Math.hypot(x2 - x1, y2 - y1);
    if (!(L > 1e-9)) return;
    var ux = (x2 - x1) / L, uy = (y2 - y1) / L, nx = -uy, ny = ux;
    var lead = Math.min(10, L * 0.15);
    var body = L - 2 * lead;
    var nseg = Math.max(2, Math.round(coils * 2));
    ctx.save();
    ctx.strokeStyle = o.color || C.ink; ctx.lineWidth = o.width != null ? o.width : 2;
    ctx.lineJoin = 'round'; ctx.lineCap = 'round';
    ctx.beginPath();
    ctx.moveTo(x1, y1);
    ctx.lineTo(x1 + ux * lead, y1 + uy * lead);
    for (var i = 0; i < nseg; i++) {
      var s = lead + body * (i + 0.5) / nseg;
      var side = (i % 2 === 0) ? 1 : -1;
      ctx.lineTo(x1 + ux * s + nx * amp * side, y1 + uy * s + ny * amp * side);
    }
    ctx.lineTo(x1 + ux * (L - lead), y1 + uy * (L - lead));
    ctx.lineTo(x2, y2);
    ctx.stroke();
    ctx.restore();
  };

  draw.hatch = function (ctx, x1, y1, x2, y2, o) {
    o = o || {};
    var side = o.side != null ? o.side : 1, len = o.length != null ? o.length : 8, sp = o.spacing != null ? o.spacing : 8;
    var L = Math.hypot(x2 - x1, y2 - y1);
    if (!(L > 1e-9)) return;
    var ux = (x2 - x1) / L, uy = (y2 - y1) / L;
    var nx = -uy * side, ny = ux * side;   // side 1: right-hand side of the direction on screen (below a left-to-right line)
    var dx = (nx - ux) * Math.SQRT1_2 * len, dy = (ny - uy) * Math.SQRT1_2 * len;
    ctx.save();
    ctx.strokeStyle = o.color || C.ink;
    ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (var s = sp / 2 + len * 0.7; s <= L + 1e-6; s += sp) {
      var px = x1 + ux * s, py = y1 + uy * s;
      ctx.moveTo(px, py); ctx.lineTo(px + dx, py + dy);
    }
    ctx.stroke();
    ctx.restore();
  };

  draw.angleArc = function (ctx, cx, cy, r, a0, a1, o) {
    o = o || {};
    var ccw = o.ccw !== false;
    var color = o.color || C.ink;
    ctx.save();
    ctx.strokeStyle = color; ctx.lineWidth = o.width != null ? o.width : 1.5;
    ctx.beginPath();
    ctx.arc(cx, cy, r, -a0, -a1, ccw);
    ctx.stroke();
    if (o.label != null && o.label !== '') {
      var sweep = ccw ? (a1 - a0) : (a0 - a1);
      sweep = ((sweep % TAU) + TAU) % TAU;
      var mid = ccw ? a0 + sweep / 2 : a0 - sweep / 2;
      ctx.font = o.font || '12px sans-serif';
      var tw = ctx.measureText(String(o.label)).width;
      var rr = r + 6 + Math.abs(Math.cos(mid)) * tw / 2 + 4;
      ctx.fillStyle = color;
      ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.fillText(String(o.label), cx + rr * Math.cos(mid), cy - rr * Math.sin(mid));
    }
    ctx.restore();
  };

  /* ------------------------------------------------------------------ *
   * 1.8 Plot helper
   * ------------------------------------------------------------------ */
  var TICK_FONT = '11px sans-serif';
  var LABEL_FONT = '12px sans-serif';
  var TITLE_FONT = 'bold 12px sans-serif';

  function tickFormatter(step) {
    return function (v) {
      if (Math.abs(v) < Math.abs(step) * 1e-9) return '0';
      var a = Math.abs(v);
      if (a >= 1e6 || step < 1e-4) {
        // enough significant digits to tell neighbouring ticks apart (e.g. 1000000, 1000001, …)
        var sig = Math.floor(Math.log10(a)) - Math.floor(Math.log10(step) + 1e-9) + 2;
        return fmt(v, Math.max(3, Math.min(15, sig)));
      }
      var d = Math.max(0, -Math.floor(Math.log10(step) + 1e-9));
      var mant = step / Math.pow(10, Math.floor(Math.log10(step) + 1e-9));
      if (Math.abs(mant - 2.5) < 1e-6) d += 1;
      var s = a.toFixed(d);
      if (d > 0) s = stripZeros(s);
      return v < 0 ? MINUS + s : s;
    };
  }
  // nice ticks for an axis: at least 3 labelled ticks whenever the axis is long enough
  function linTicks(min, max, target) {
    var t = M.niceTicks(min, max, target);
    if (t.length < 3) { var t2 = M.niceTicks(min, max, target * 2); if (t2.length > t.length) t = t2; }
    return t;
  }
  function logLabel(v) {
    var k = Math.round(Math.log10(v));
    if (Math.abs(v - Math.pow(10, k)) < 1e-9 * v && (k < -3 || k > 4)) return '10' + superscript(k);
    return fmt(v, 3);
  }
  function logTicks(min, max) {
    var lo = Math.log10(min), hi = Math.log10(max);
    var majors = [], minors = [];
    for (var k = Math.floor(lo) - 1; k <= Math.ceil(hi) + 1; k++) {
      var base = Math.pow(10, k);
      if (base >= min * (1 - 1e-9) && base <= max * (1 + 1e-9)) majors.push(base);
      for (var m = 2; m <= 9; m++) {
        var v = m * base;
        if (v >= min * (1 - 1e-9) && v <= max * (1 + 1e-9)) minors.push(v);
      }
    }
    return { majors: majors, minors: minors };
  }

  RASim.plot = function (ctx, o) {
    o = o || {};
    var box = o.box || { x: 0, y: 0, w: 300, h: 200 };
    var m = { left: 50, right: 14, top: o.title ? 28 : 12, bottom: 38 };
    if (o.margin) Object.keys(o.margin).forEach(function (k) { if (o.margin[k] != null) m[k] = o.margin[k]; });
    var rect = { x: box.x + m.left, y: box.y + m.top, w: Math.max(1, box.w - m.left - m.right), h: Math.max(1, box.h - m.top - m.bottom) };
    var xlog = !!o.xlog, ylog = !!o.ylog;
    function range(a, b, lg) {
      a = Number(a); b = Number(b);
      if (!isFinite(a) || !isFinite(b)) { a = lg ? 1 : 0; b = lg ? 10 : 1; }
      if (lg) { if (!(a > 0)) a = 1e-3; if (!(b > 0)) b = a * 10; }
      if (a > b) { var t = a; a = b; b = t; }
      if (a === b) {
        if (lg) { a = a / 1.25; b = b * 1.25; }
        else { var d = Math.abs(a) * 0.05 || 1; a -= d; b += d; }
      }
      return [a, b];
    }
    var xr = range(o.xmin != null ? o.xmin : 0, o.xmax != null ? o.xmax : 1, xlog);
    var yr = range(o.ymin != null ? o.ymin : 0, o.ymax != null ? o.ymax : 1, ylog);
    var xmin = xr[0], xmax = xr[1], ymin = yr[0], ymax = yr[1];
    if (o.equal && !xlog && !ylog) {
      var ux = (xmax - xmin) / rect.w, uy = (ymax - ymin) / rect.h;
      if (ux > uy) { var cy = (ymin + ymax) / 2, hh = ux * rect.h / 2; ymin = cy - hh; ymax = cy + hh; }
      else { var cxm = (xmin + xmax) / 2, hw = uy * rect.w / 2; xmin = cxm - hw; xmax = cxm + hw; }
    }
    var fx = xlog ? Math.log10 : function (v) { return v; };
    var fy = ylog ? Math.log10 : function (v) { return v; };
    var lx0 = fx(xmin), lx1 = fx(xmax), ly0 = fy(ymin), ly1 = fy(ymax);

    var p = {
      ctx: ctx, box: box, margin: m, rect: rect,
      xmin: xmin, xmax: xmax, ymin: ymin, ymax: ymax, xlog: xlog, ylog: ylog,
      X: function (x) { return rect.x + (fx(x) - lx0) / (lx1 - lx0) * rect.w; },
      Y: function (y) { return rect.y + rect.h - (fy(y) - ly0) / (ly1 - ly0) * rect.h; },
      invX: function (px) { var l = lx0 + (px - rect.x) / rect.w * (lx1 - lx0); return xlog ? Math.pow(10, l) : l; },
      invY: function (py) { var l = ly0 + (rect.y + rect.h - py) / rect.h * (ly1 - ly0); return ylog ? Math.pow(10, l) : l; },
      contains: function (px, py) { return px >= rect.x && px <= rect.x + rect.w && py >= rect.y && py <= rect.y + rect.h; }
    };

    function begin(style) {
      ctx.save();
      if (!style || style.clip !== false) { ctx.beginPath(); ctx.rect(rect.x, rect.y, rect.w, rect.h); ctx.clip(); }
    }
    function stroke(style, defColor) {
      style = style || {};
      ctx.strokeStyle = style.color || defColor || C.blue;
      ctx.lineWidth = style.width != null ? style.width : 2;
      ctx.setLineDash(style.dash || []);
      ctx.lineJoin = 'round'; ctx.lineCap = style.dash ? 'butt' : 'round';
      ctx.stroke();
    }

    p.axes = function (a) {
      a = a || {};
      var grid = a.grid !== false, zero = a.zero !== false, frame = a.frame !== false;
      ctx.save();
      ctx.fillStyle = '#ffffff';
      ctx.fillRect(rect.x, rect.y, rect.w, rect.h);
      ctx.font = TICK_FONT;
      var xt, yt, xminor = [], yminor = [], xf, yf;
      if (xlog) {
        var lt = logTicks(xmin, xmax);
        xt = a.xTicks || lt.majors; xminor = lt.minors;
        if (!a.xTicks && xt.length < 2) xt = lt.majors.concat(lt.minors.filter(function (v) { var mm = Math.round(v / Math.pow(10, Math.floor(Math.log10(v) + 1e-9))); return mm === 2 || mm === 5; })).sort(function (u, v) { return u - v; });
        xf = a.xFormat || logLabel;
      } else {
        var nx = Math.max(3, Math.min(10, Math.floor(rect.w / 50)));
        xt = a.xTicks || linTicks(xmin, xmax, nx);
        xf = a.xFormat || tickFormatter(xt.length > 1 ? Math.abs(xt[1] - xt[0]) : M.niceCeil(Math.abs(xmax - xmin) / nx || 1));
      }
      if (ylog) {
        var ltY = logTicks(ymin, ymax);
        yt = a.yTicks || ltY.majors; yminor = ltY.minors;
        if (!a.yTicks && yt.length < 2) yt = ltY.majors.concat(ltY.minors.filter(function (v) { var mm = Math.round(v / Math.pow(10, Math.floor(Math.log10(v) + 1e-9))); return mm === 2 || mm === 5; })).sort(function (u, v) { return u - v; });
        yf = a.yFormat || logLabel;
      } else {
        var ny = Math.max(3, Math.min(10, Math.floor(rect.h / 28)));
        yt = a.yTicks || linTicks(ymin, ymax, ny);
        yf = a.yFormat || tickFormatter(yt.length > 1 ? Math.abs(yt[1] - yt[0]) : M.niceCeil(Math.abs(ymax - ymin) / ny || 1));
      }
      function vx(px) { return Math.round(px) + 0.5; }
      if (grid) {
        ctx.lineWidth = 1;
        if (xminor.length || yminor.length) {
          ctx.strokeStyle = RASim.alpha(C.grid, 0.6);
          ctx.beginPath();
          xminor.forEach(function (v) { var px = vx(p.X(v)); ctx.moveTo(px, rect.y); ctx.lineTo(px, rect.y + rect.h); });
          yminor.forEach(function (v) { var py = vx(p.Y(v)); ctx.moveTo(rect.x, py); ctx.lineTo(rect.x + rect.w, py); });
          ctx.stroke();
        }
        ctx.strokeStyle = C.grid;
        ctx.beginPath();
        xt.forEach(function (v) { var px = p.X(v); if (px < rect.x - 0.5 || px > rect.x + rect.w + 0.5) return; px = vx(px); ctx.moveTo(px, rect.y); ctx.lineTo(px, rect.y + rect.h); });
        yt.forEach(function (v) { var py = p.Y(v); if (py < rect.y - 0.5 || py > rect.y + rect.h + 0.5) return; py = vx(py); ctx.moveTo(rect.x, py); ctx.lineTo(rect.x + rect.w, py); });
        ctx.stroke();
      }
      if (zero) {
        ctx.strokeStyle = C.axis; ctx.lineWidth = 1;
        ctx.beginPath();
        if (!xlog && xmin < 0 && xmax > 0) { var zx = vx(p.X(0)); ctx.moveTo(zx, rect.y); ctx.lineTo(zx, rect.y + rect.h); }
        if (!ylog && ymin < 0 && ymax > 0) { var zy = vx(p.Y(0)); ctx.moveTo(rect.x, zy); ctx.lineTo(rect.x + rect.w, zy); }
        ctx.stroke();
      }
      if (frame) {
        ctx.strokeStyle = C.axis; ctx.lineWidth = 1;
        ctx.strokeRect(Math.round(rect.x) + 0.5, Math.round(rect.y) + 0.5, Math.round(rect.w) - 1, Math.round(rect.h) - 1);
      }
      // x tick labels (thinned to avoid overlap)
      ctx.fillStyle = C.muted;
      var xl = [];
      xt.forEach(function (v) {
        var px = p.X(v);
        if (!(px >= rect.x - 0.5 && px <= rect.x + rect.w + 0.5)) return;
        var s = xf(v);
        xl.push({ px: px, s: s, w: ctx.measureText(s).width });
      });
      var stride = 1;
      for (; stride < xl.length; stride++) {
        var ok = true;
        for (var i = stride; i < xl.length; i += stride) {
          if (xl[i].px - xl[i].w / 2 < xl[i - stride].px + xl[i - stride].w / 2 + 6) { ok = false; break; }
        }
        if (ok) break;
      }
      ctx.textBaseline = 'top';
      for (var j = 0; j < xl.length; j += stride) {
        var lab = xl[j];
        var lx = M.clamp(lab.px, box.x + lab.w / 2 + 1, box.x + box.w - lab.w / 2 - 1);
        ctx.textAlign = 'center';
        ctx.fillText(lab.s, lx, rect.y + rect.h + 4);
      }
      // y tick labels
      ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
      var yl = [];
      yt.forEach(function (v) { var py = p.Y(v); if (py >= rect.y - 0.5 && py <= rect.y + rect.h + 0.5) yl.push({ py: py, s: yf(v) }); });
      var ys = 1;
      while (ys < yl.length && yl.length > 1 && Math.abs(yl[0].py - yl[Math.min(ys, yl.length - 1)].py) < 14) ys++;
      for (var k = 0; k < yl.length; k += ys) ctx.fillText(yl[k].s, rect.x - 5, M.clamp(yl[k].py, box.y + 6, box.y + box.h - 6));
      // axis labels and title
      ctx.fillStyle = C.ink;
      ctx.font = LABEL_FONT;
      if (o.xlabel) { ctx.textAlign = 'center'; ctx.textBaseline = 'bottom'; ctx.fillText(o.xlabel, rect.x + rect.w / 2, box.y + box.h - 2); }
      if (o.ylabel) {
        ctx.save();
        ctx.translate(box.x + 3, rect.y + rect.h / 2);
        ctx.rotate(-Math.PI / 2);
        ctx.textAlign = 'center'; ctx.textBaseline = 'top';
        ctx.fillText(o.ylabel, 0, 0);
        ctx.restore();
      }
      if (o.title) { ctx.font = TITLE_FONT; ctx.textAlign = 'center'; ctx.textBaseline = 'top'; ctx.fillText(o.title, rect.x + rect.w / 2, box.y + 7); }
      ctx.restore();
      return p;
    };

    function path(xs, ys) {
      ctx.beginPath();
      var pen = false;
      for (var i = 0; i < xs.length; i++) {
        var px = p.X(xs[i]), py = p.Y(ys[i]);
        if (!isFinite(px) || !isFinite(py)) { pen = false; continue; }
        if (pen) ctx.lineTo(px, py); else { ctx.moveTo(px, py); pen = true; }
      }
    }
    p.line = function (xs, ys, style) { begin(style); path(xs, ys); stroke(style); ctx.restore(); return p; };
    p.fn = function (f, style) {
      style = style || {};
      var n = style.n != null ? style.n : 400;
      var xs = xlog ? M.logspace(xmin, xmax, n + 1) : M.linspace(xmin, xmax, n + 1);
      var ys = xs.map(function (x) { try { var y = f(x); return typeof y === 'number' ? y : NaN; } catch (e) { return NaN; } });
      return p.line(xs, ys, style);
    };
    p.area = function (xs, ys, style) {
      style = style || {};
      var base = style.base != null ? style.base : 0;
      begin(style);
      ctx.fillStyle = style.fill || RASim.alpha(C.blue, 0.15);
      var seg = [];
      function flush() {
        if (seg.length > 1) {
          ctx.beginPath();
          ctx.moveTo(seg[0][0], p.Y(base));
          seg.forEach(function (q) { ctx.lineTo(q[0], q[1]); });
          ctx.lineTo(seg[seg.length - 1][0], p.Y(base));
          ctx.closePath(); ctx.fill();
        }
        seg = [];
      }
      for (var i = 0; i < xs.length; i++) {
        var px = p.X(xs[i]), py = p.Y(ys[i]);
        if (!isFinite(px) || !isFinite(py)) { flush(); continue; }
        seg.push([px, py]);
      }
      flush();
      ctx.restore();
      return p;
    };
    p.band = function (y0, y1, style) {
      style = style || {};
      begin(style);
      ctx.fillStyle = style.fill || RASim.alpha(C.blue, 0.1);
      var a = p.Y(y0), b = p.Y(y1);
      ctx.fillRect(rect.x, Math.min(a, b), rect.w, Math.abs(b - a));
      ctx.restore();
      return p;
    };
    p.vband = function (x0, x1, style) {
      style = style || {};
      begin(style);
      ctx.fillStyle = style.fill || RASim.alpha(C.blue, 0.1);
      var a = p.X(x0), b = p.X(x1);
      ctx.fillRect(Math.min(a, b), rect.y, Math.abs(b - a), rect.h);
      ctx.restore();
      return p;
    };
    p.hline = function (y, style) {
      begin(style);
      var py = p.Y(y);
      ctx.beginPath(); ctx.moveTo(rect.x, py); ctx.lineTo(rect.x + rect.w, py);
      stroke(style, C.axis); ctx.restore(); return p;
    };
    p.vline = function (x, style) {
      begin(style);
      var px = p.X(x);
      ctx.beginPath(); ctx.moveTo(px, rect.y); ctx.lineTo(px, rect.y + rect.h);
      stroke(style, C.axis); ctx.restore(); return p;
    };
    p.segment = function (x1, y1, x2, y2, style) {
      begin(style);
      ctx.beginPath(); ctx.moveTo(p.X(x1), p.Y(y1)); ctx.lineTo(p.X(x2), p.Y(y2));
      stroke(style); ctx.restore(); return p;
    };
    p.arrow = function (x1, y1, x2, y2, style) {
      style = style || {};
      begin(style);
      draw.arrow(ctx, p.X(x1), p.Y(y1), p.X(x2), p.Y(y2), { color: style.color || C.blue, width: style.width != null ? style.width : 2, head: style.head != null ? style.head : 8, dash: style.dash });
      ctx.restore(); return p;
    };
    p.point = function (x, y, style) {
      style = style || {};
      var color = style.color || C.blue, r = style.r != null ? style.r : 4;
      begin(style);
      ctx.beginPath(); ctx.arc(p.X(x), p.Y(y), r, 0, TAU);
      ctx.fillStyle = style.fill || color; ctx.fill();
      var sk = style.stroke !== undefined ? style.stroke : '#fff';
      if (sk) { ctx.strokeStyle = sk; ctx.lineWidth = 1.5; ctx.stroke(); }
      ctx.restore(); return p;
    };
    p.cross = function (x, y, style) {
      style = style || {};
      var s = style.size != null ? style.size : 6;
      var px = p.X(x), py = p.Y(y);
      begin(style);
      ctx.beginPath();
      ctx.moveTo(px - s, py - s); ctx.lineTo(px + s, py + s);
      ctx.moveTo(px - s, py + s); ctx.lineTo(px + s, py - s);
      ctx.strokeStyle = style.color || C.red; ctx.lineWidth = style.width != null ? style.width : 2;
      ctx.setLineDash([]); ctx.lineCap = 'round'; ctx.stroke();
      ctx.restore(); return p;
    };
    p.ring = function (x, y, style) {
      style = style || {};
      begin(style);
      ctx.beginPath(); ctx.arc(p.X(x), p.Y(y), style.r != null ? style.r : 5, 0, TAU);
      ctx.fillStyle = '#fff'; ctx.fill();
      ctx.strokeStyle = style.color || C.blue; ctx.lineWidth = style.width != null ? style.width : 2;
      ctx.setLineDash([]); ctx.stroke();
      ctx.restore(); return p;
    };
    p.circle = function (cx, cy, r, style) {
      style = style || {};
      begin(style);
      var px = p.X(cx), py = p.Y(cy);
      var rx = Math.abs(p.X(cx + r) - px), ry = Math.abs(p.Y(cy + r) - py);
      ctx.beginPath(); ctx.ellipse(px, py, rx, ry, 0, 0, TAU);
      if (style.fill) { ctx.fillStyle = style.fill; ctx.fill(); }
      if (style.color !== null) stroke(style);
      ctx.restore(); return p;
    };
    p.arc = function (cx, cy, rPx, a0, a1, style) {
      style = style || {};
      begin(style);
      ctx.beginPath(); ctx.arc(p.X(cx), p.Y(cy), rPx, -a0, -a1, style.ccw !== false);
      stroke(style, C.ink);
      ctx.restore(); return p;
    };
    p.text = function (x, y, str, style) {
      style = style || {};
      ctx.save();
      if (style.clip === true) { ctx.beginPath(); ctx.rect(rect.x, rect.y, rect.w, rect.h); ctx.clip(); }
      draw.text(ctx, str, p.X(x) + (style.dx || 0), p.Y(y) + (style.dy || 0), {
        color: style.color, font: style.font, align: style.align, baseline: style.baseline, bg: style.bg
      });
      ctx.restore(); return p;
    };
    return p;
  };

  /* ------------------------------------------------------------------ *
   * Exports and boot
   * ------------------------------------------------------------------ */
  globalThis.RASim = RASim;
  if (typeof module !== 'undefined' && module.exports) module.exports = RASim;

  if (HAS_DOM) {
    if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', function () { RASim.mountAll(); });
    else RASim.mountAll();
  }
})();
