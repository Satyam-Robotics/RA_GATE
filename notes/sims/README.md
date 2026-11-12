# RASim: interactive simulations for the RA GATE 2027 notes

This folder holds the shared core (`core.js`, `core.css`) and one file per simulation (`<id>.js`).
`node notes/build.js` inlines everything into the single offline page `notes/RA_GATE2027_Notes.html`.
Simulations are **screen-only**: `@media print` hides them, so the PDF never changes.

**Status: all 12 simulations are built, independently verified and integrated** (they mount together in the real
page; see "Run all checks" below). The 12 simulations and where their placeholders sit:

| id | title | placeholder in |
|---|---|---|
| `eigen-2x2` | Eigenvalues and eigenvectors of a 2×2 matrix | `src/01_math.html` |
| `rlc-transient` | RC / RL / RLC transient response | `src/02_mechatronics.html` |
| `rotation-transforms` | 3D rotations and homogeneous transforms | `src/03_robotics.html` |
| `two-link-arm` | 2-link planar arm: forward and inverse kinematics | `src/03_robotics.html` |
| `second-order-step` | Second-order step response and pole location | `src/04_electrical.html` |
| `bode-plot` | Bode plot builder with gain and phase margins | `src/04_electrical.html` |
| `root-locus` | Root locus explorer | `src/04_electrical.html` |
| `pid-tuning` | PID tuning lab | `src/04_electrical.html` |
| `mohr-circle` | Mohr's circle for plane stress | `src/05_mechanical.html` |
| `beam-sfd-bmd` | Beam: shear force, bending moment and deflection | `src/05_mechanical.html` |
| `four-bar` | Four-bar linkage and Grashof's law | `src/05_mechanical.html` |
| `vibration-sdof` | Single-DOF vibration: free, forced and isolation | `src/05_mechanical.html` |

## Run all checks

From the repository root (every command must exit 0; the browser parts need Playwright and are skipped with
`--no-browser`):

```
node notes/sims/test/core.test.js
for id in eigen-2x2 rlc-transient rotation-transforms two-link-arm second-order-step bode-plot root-locus \
          pid-tuning mohr-circle beam-sfd-bmd four-bar vibration-sdof; do
  node notes/sims/test/$id.test.js || echo "FAILED: $id.test.js"
  node notes/sims/test/harness.js $id --out /tmp/rasim-check/$id > /tmp/rasim-check-$id.json \
    || echo "FAILED: harness $id"          # also open /tmp/rasim-check/$id/*.png and read the JSON warnings
done
node notes/build.js --html-only            # rebuild notes/RA_GATE2027_Notes.html (the PDF is not touched)
```

The tests and the harness write their temporary pages under `os.tmpdir()` (set `TMPDIR` to keep them elsewhere).
`node notes/build.js` without `--html-only` also re-prints the PDF: simulations are hidden in print, so its text and
pages must stay identical (compare `pdftotext -layout` and `pdftoppm` renders with the committed file before keeping it).

## Rules for authors

1. Touch **only your own two files**: `notes/sims/<id>.js` and `notes/sims/test/<id>.test.js`. Never edit the core, the CSS, the build or `notes/src`. If the core is missing something, work around it inside your file and report it.
2. No external resources: no CDNs, fonts, images by URL, `fetch`, or libraries. Plain ES2019, Canvas 2D and DOM.
3. Use `RASim.colors` for every colour (and `RASim.alpha(hex, a)` for translucency).
4. Use `RASim.fmt` / `fmtFixed` / `fmtComplex` / `fmtPoly` for **every number shown** (real minus sign, no `-0`, no `0.30000000000000004`).
5. Animate **only** through `api.animate` (it pauses off-screen, in hidden tabs and when the panel is collapsed). Never call `requestAnimationFrame` or `setInterval` yourself.
6. `mount` must return `{ getState(), setState(partialState) }`. `setState` updates the controls (with the silent `set` methods) and redraws; it must accept any subset of the keys `getState` returns.
7. Build all DOM inside `api.root`. If you really need CSS, use `RASim.css('<id>', css)` and prefix every selector with `.sim[data-sim="<id>"]`.
8. Keep `model` pure (no DOM): it is what the Node tests call.
9. File names starting with `_` are ignored by the build (use them for drafts).

## Simulation file template

```js
(function () {
  'use strict';
  const RASim = globalThis.RASim;
  const M = RASim.math;

  const model = {
    // pure functions, no DOM. Example:
    peak: function (zeta) { return Math.exp(-Math.PI * zeta / Math.sqrt(1 - zeta * zeta)); }
  };

  RASim.register('my-id', {
    title: 'My simulation title',
    model: model,
    mount: function (api) {
      const ui = RASim.ui, C = RASim.colors;
      const st = { zeta: 0.5 };

      const sp = ui.split(api.root);                      // canvas left, controls right (one column when narrow)
      const cv = RASim.canvas(sp.main, { aspect: 1.6, key: 'main' });
      const g = ui.group(sp.side, 'Parameters');
      const sZ = ui.slider(g, { label: 'ζ', min: 0, max: 2, step: 0.01, value: st.zeta, key: 'zeta',
        onInput: function (v) { st.zeta = v; update(); } });
      const stats = ui.stats(sp.side);
      const sPeak = ui.stat(stats, { label: 'Overshoot', key: 'overshoot' });
      ui.hints(api.root, ['Set ζ = 0.7 and read the overshoot.']);

      cv.draw = function (ctx, w, h) {
        const p = RASim.plot(ctx, { box: { x: 0, y: 0, w: w, h: h }, xmin: 0, xmax: 10, ymin: 0, ymax: 1.6,
          xlabel: 't (s)', ylabel: 'y' });
        p.axes();
        const r = M.tfStep([4], [1, 4 * st.zeta, 4], 10);
        p.line(r.t, r.y, { color: C.blue });
      };

      function update() {
        sPeak.set(RASim.fmt(100 * model.peak(st.zeta), 3) + ' %');
        cv.redraw();
      }
      function getState() { return { zeta: st.zeta }; }
      function setState(s) {
        if (s.zeta != null) { st.zeta = s.zeta; sZ.set(s.zeta); st.zeta = sZ.get(); }
        update();
      }
      update();
      return { getState: getState, setState: setState };
    }
  });
})();
```

## Testing

### Model tests (Node)

```js
// notes/sims/test/my-id.test.js
const assert = require('assert');
const RASim = require('./load.js')('my-id');          // loads core.js once, then notes/sims/my-id.js
const m = RASim.sims['my-id'].model;
assert.ok(Math.abs(m.peak(0.5) - 0.163) < 1e-3);
console.log('ok');
```

Run with `node notes/sims/test/my-id.test.js`. `load(id, file)` takes an optional path when the sim file is elsewhere.
The core's own tests: `node notes/sims/test/core.test.js`.

### Browser harness

```
node notes/sims/test/harness.js <id> --out <dir> [--widths 1100,390] [--file <path to the sim js>]
```

Writes `<dir>/<id>.html` (same styles as the real page, `window.RASIM_EAGER = true`, then `core.js`, then your file,
each in its own `<script>`), opens it at each width (height 900), waits for `RASim.instances[id]` (5 s timeout is an
error) plus 400 ms, and saves `<dir>/<id>-<width>.png` (full page). It prints
`{ id, ok, runs: [{ width, errors, warnings, screenshot }] }` and exits 0 only when there are no errors.
Errors: console errors, uncaught page errors, mount timeout. Warnings: console warnings, `horizontal overflow`
(document wider than the window) and any panel descendant sticking out more than 1px past the panel's right edge.
Treat warnings as bugs too. **Open the screenshots and look at them.**
Use an output directory outside the repository (for example your scratchpad).

### Custom interaction scripts (Playwright)

`harness.js` exports `buildPage(id, outDir, file) -> htmlPath` and
`async openSim(id, { width = 1100, height = 900, outDir, file, touch = false, deviceScaleFactor = 1 })`
`-> { browser, context, page, errors, warnings, htmlPath }` with the simulation already mounted
(`errors` and `warnings` are live arrays). Always close the browser.

```js
const { openSim } = require('/home/user/RA_GATE/notes/sims/test/harness.js');
(async () => {
  const { browser, page, errors } = await openSim('my-id', { width: 390, outDir: '/tmp/my-id' });
  await page.evaluate(() => RASim.instances['my-id'].setState({ zeta: 0.7 }));
  const os = await page.textContent('[data-stat="overshoot"] .sim-stat-value');
  console.log('overshoot', os);                               // e.g. '4.6 %'
  await page.fill('[data-control-num="zeta"]', '0.25');       // type into the slider's number box
  await page.press('[data-control-num="zeta"]', 'Enter');     // fires onInput
  console.log(await page.evaluate(() => RASim.instances['my-id'].getState()));
  await page.screenshot({ path: '/tmp/my-id/after.png', fullPage: true });
  if (errors.length) console.log(errors);
  await browser.close();
})();
```

Canvas pointer coordinates are CSS pixels inside the canvas border: page point =
`canvas.getBoundingClientRect().left + 1 + x` (1px border). To find a handle, rebuild the same `RASim.plot`
inside `page.evaluate` and use `p.X()` / `p.Y()`, then `page.mouse.move/down/move/up`.

## Layout advice

- `ui.split` for "canvas(es) + controls"; it becomes one column (main first) when the **panel** is narrower than 720px.
- Group related controls with `ui.group`; put read-outs in `ui.stats` + `ui.stat`; use `ui.tag` for a state such as UNDERDAMPED.
- `ui.hints` for 2–4 guided experiments that teach the exam point.
- Every state change: update the stats and call `cv.redraw()` (coalesced, cheap). Inside `api.animate` callbacks call `cv.drawNow()`.
- Design for a 360px phone: the panel body is then about 300px wide. Canvas text at least 11px.
- Draw with `RASim.plot` whenever there are axes; it handles ticks, log axes, equal scaling and clipping.

---

## API reference

Everything lives on the global `RASim` (in Node: `require('notes/sims/core.js')`, which also sets `globalThis.RASim`).
Complex numbers are plain objects `{ re, im }`; wherever a complex argument is expected a plain number is accepted.
Polynomials are arrays of real coefficients, **highest power first** (`[1, 3, 2]` = s² + 3s + 2).

### Registry and lifecycle

| | |
|---|---|
| `RASim.sims` | `{ id: def }` registered definitions |
| `RASim.instances` | `{ id: value returned by mount }` (`{}` if mount returned nothing; absent if mount threw) |
| `RASim.register(id, def)` | `def = { title, model?, mount(api) }`. Throws on a duplicate id or a missing `mount`. Works in Node. |
| `RASim.mountAll()` | Called automatically once at `DOMContentLoaded`. Idempotent. For every `<div class="sim" data-sim="ID">` with a registered ID: sets `id="sim-ID"`, builds the panel chrome and arranges for `mount(api)` exactly once (lazy: when the panel comes within 400px of the viewport; immediately if `window.RASIM_EAGER === true` or there is no IntersectionObserver). Unregistered placeholders stay empty and hidden and log one `console.warn`. Fills `[data-sim-index]` with a linked list of the registered sims in document order (stays empty/hidden when there are none). A sim registered after boot is mounted by an automatic `mountAll()`. |
| `RASim.css(key, cssText)` | Adds `<style data-rasim-css="key">` to `<head>` once per key. |

Panel chrome: `.sim-head` (badge "Interactive", `.sim-title`, `button.sim-toggle` Hide/Show with `aria-expanded`) and `.sim-body`.
Collapsing adds `sim-collapsed` to the `.sim` element; expanding redraws its canvases.
If `mount` throws, the error is logged with `console.error` and `.sim-body` shows
`<p class="sim-error">This simulation failed to load.</p>`; other simulations are unaffected.
Each simulation file is a separate `<script>` so a syntax error only loses that simulation.

### `api` (argument of `mount`)

| | |
|---|---|
| `api.id` | the simulation id |
| `api.root` | the empty `.sim-body` element: build all DOM here |
| `api.panel` | the outer `.sim` element |
| `api.animate(fn) -> { start(), stop(), toggle(), running }` | Not started automatically. While started, calls `fn(dt, t)` once per frame: `dt` = seconds since the previous delivered frame, at most 0.05; `t` = sum of delivered `dt`s. Frames are delivered only while started **and** the panel intersects the viewport **and** the tab is visible **and** the panel is not collapsed. After any pause (and on the first frame after `start()`) `dt` restarts at 1/60. A paused or stopped controller keeps no `requestAnimationFrame` loop alive. `running` (getter) is true between `start()` and `stop()` even while paused. If `fn` throws, the error is logged and the controller stops. Several controllers per sim are fine. |

### Formatting (all return strings; negatives use U+2212 "−")

| | |
|---|---|
| `fmt(v, sig = 4)` | NaN → `—`, ±Infinity → `∞` / `−∞`, \|v\| < 1e-12 → `0`. Rounds to `sig` significant digits, strips trailing zeros. Plain decimals for 1e-4 ≤ \|v\| < 1e7, otherwise `1.2×10⁻⁵`. Never `−0`. |
| `fmtFixed(v, decimals = 2)` | `toFixed` with the same special cases; never `−0.00`. \|v\| ≥ 1e21 (where `toFixed` gives `1e+21`) falls back to `fmt(v, 6)`. |
| `fmtComplex(z, sig = 4)` | `−1 + j1.732`, `2 − j0.5`, `j1.732`, `−j1.732`, or `fmt(re)` when purely real. |
| `fmtPoly(coeffs, variable = 's', sig = 4)` | HTML: `[1, 3, 2, 0]` → `s<sup>3</sup> + 3s<sup>2</sup> + 2s`; zero terms skipped, unit coefficients omitted (not on the constant), `' − '` between negative terms, all-zero → `0`. |

### Colours

`RASim.colors = { ink, muted, faint, grid, axis, panel, fill, maroon, blue, green, orange, purple, red, teal, gold, lightBlue, lightRed, lightGreen }`
(`#1a1a1a #666666 #9a9a9a #e4e0d6 #8a8a8a #fbf8f0 #f3ead2 #6b1d1d #1f4e79 #2e6b30 #c2571a #6a3d9a #c0392b #0f766e #b8860b #9cc3e6 #f2c1bb #bfe0c0`).
`RASim.alpha('#rrggbb', a)` → `'rgba(r, g, b, a)'`.

### `RASim.math`

| | |
|---|---|
| `cx(re, im = 0)`, `cadd(a, b)`, `csub`, `cmul`, `cdiv` (Smith), `cscale(a, k)`, `cconj`, `cabs`, `carg` (atan2), `cexp`, `csqrt` (principal) | complex arithmetic, each returns `{ re, im }` (or a number for `cabs`, `carg`) |
| `clamp(v, lo, hi)`, `rad(deg)`, `deg(rad)` | |
| `wrapPi(a)` → (−π, π];  `wrapDeg(a)` → (−180, 180] | |
| `linspace(a, b, n)` | n values including both ends |
| `logspace(a, b, n)` | n values from a to b (actual values, > 0), equally spaced in log10; ends exact |
| `niceCeil(v)` | smallest {1, 2, 2.5, 5, 10}×10^k ≥ v |
| `niceTicks(min, max, target = 6)` | ascending nice ticks inside [min, max]; step = `niceCeil((max − min)/target)` |
| `polyTrim(p)` | drops leading coefficients with \|c\| ≤ 1e-13·max\|c\|; never empty (`[0]`) |
| `polyAdd(a, b)`, `polySub(a, b)` | results are trimmed |
| `polyMul(a, b)`, `polyScale(p, k)`, `polyDeriv(p)` (`[c]` → `[0]`) | |
| `polyEval(p, s)` → `{ re, im }` | Horner |
| `polyFromRoots(roots)` | monic real polynomial; each complex root with im > 0 contributes its conjugate pair (the im < 0 partners are expected and skipped); throws if the counts of im > 0 and im < 0 roots differ; `[]` → `[1]` |
| `polyRoots(p)` → `[{ re, im }]` | length = degree after trimming. Exact zero roots are factored out first, closed form for degree ≤ 2, otherwise Aberth iteration on the scaled polynomial plus Newton polishing; then \|im\| < 1e-9·max(1,\|root\|) is snapped to 0, conjugate pairs are made exact, and roots are sorted by descending re (ties within 1e-9 relative: descending im). The output is always conjugate-symmetric: pairing goes from the largest \|im\| down, a root is paired with the root whose conjugate is nearest, and only if that conjugate is closer to it than the real axis is; otherwise the root is made real. Simple roots ~1e-9 relative (well-conditioned cases; clustered roots are limited by their conditioning); double roots ~1e-8, triple roots ~1e-5. **A repeated real root comes back as slightly different real roots or as a near-real conjugate pair (im ~1e-8 double, ~1e-5 triple)**: classify roots with a tolerance, not `im === 0`. |
| `tfEval(num, den, s)` → `{ re, im }` | |
| `tfStep(num, den, tEnd, n = 400)` → `{ t, y }` | n + 1 samples, t from 0 to tEnd exactly. Exact (to round-off) unit-step response for any pole locations via controllable canonical state space and the matrix exponential of `[[A, B], [0, 0]]·dt`. `y[0]` = feed-through D. Pure gain → constant. Throws `Error('tfStep: deg(num) = … > deg(den) = … (improper transfer function)')`. Leading zeros in num are fine. Impulse response = `tfStep(polyMul(num, [1, 0]), den, …)`. |
| `expm(M)` | matrix exponential, scaling-and-squaring with a (6,6) Padé core |
| `rk4(f, t, y, h)` | one classical RK4 step for y′ = f(t, y); arrays in, new array out |
| `bisect(f, lo, hi, tol = 1e-12, maxIter = 200)` | returns an end if f is exactly 0 there; NaN when there is no sign change |
| `zeroCrossings(f, a, b, n = 400, opts = { log: false })` | ascending sign-change locations from n + 1 samples (log-spaced with `log: true`), each refined by `bisect`; exact zeros at samples reported once; non-finite samples (or throws) break the scan. Always ascending (also when a > b). A sign change across a pole or a jump (e.g. a phase wrapped to ±180°) is reported too: use continuous functions (`unwrapDeg` the phase) or check \|f(x)\| at the result. |
| `unwrapDeg(arr)` | copy with ±360° jumps removed |
| `matMul(A, B)`, `matVec(A, x)`, `solve(A, b)` | **extras** (not in the original contract): dense matrix product, matrix·vector, and Gaussian elimination with partial pivoting (`b` a vector or a matrix; throws on a singular matrix) |

### `RASim.ui` (DOM builders)

Each appends to `parent`. Labels are trusted HTML (`θ<sub>1</sub>`). The main input carries `data-control="KEY"` where
KEY = `opts.key` or a slug of the label text (lower-case letters/digits, other runs → `-`; `'Test θ<sub>1</sub>'` → `test-θ1`).
**Always pass `key`** so tests have stable selectors. Callbacks fire synchronously on **user** changes only, with the parsed value
(number for sliders/number boxes, string for select/segmented, boolean for checkbox); `set()` never fires them.

| | |
|---|---|
| `el(tag, props = {}, parent = null)` | props: `class, text, html, title, style` (object), `attrs` (object), `on` (object of listeners). Returns the element. |
| `split(parent, { ratio = 1.5 } = {})` → `{ el, main, side }` | grid `minmax(0, ratio fr) minmax(230px, 1fr)`; one column (main first) when the panel (its border box, `.sim` element) is < 720px wide (container query on the content box, threshold 715px = 720 − the 5px of left/right borders, plus a ResizeObserver class `sim-narrow`) |
| `row(parent)` → el | wrapping flex row, gap 8px 14px, items centred (a slider inside a row takes `flex: 1 1 220px`) |
| `group(parent, title)` → inner el | titled box (small uppercase maroon heading) |
| `slider(parent, { label, min, max, step = 1, value, unit = '', scale = 'linear', decimals, key, onInput })` → `{ el, get(), set(v), setRange(min, max), setDisabled(flag), setLabel(html) }` | label, `<input type="range" data-control=KEY>`, number box `<input type="number" class="sim-num" data-control-num=KEY>`, unit. Linear: range uses min/max/step; typed values are clamped but **not** snapped. Log (`min > 0`): 400 track positions (0…399) equally spaced in log10, track values rounded to 3 significant digits (ends exact), typed values only clamped. Number box shows `decimals` digits (default: decimals of `step`; log: 3 significant digits); a typed value is shown as typed. `onInput` fires on the track `input` event and on the number box `change` event / Enter. `get()` returns the exact current value. |
| `number(parent, { label, value, min, max, step, unit = '', decimals, key, onInput })` → `{ el, get(), set(v), setDisabled(flag) }` | number box only (`data-control=KEY`), clamped to min/max when given; shows `decimals` digits (default: decimals of `step`, or the value to 12 significant digits when neither is given); `onInput` on `change` / Enter |
| `select(parent, { label, options, value, key, onChange })` → `{ el, get(), set(v), setOptions(options) }` | `options = [{ value, label }]`, string values; `setOptions` keeps the value if still present, else selects the first |
| `segmented(parent, { label, options, value, key, onChange })` → `{ el, get(), set(v) }` | joined buttons inside `div.sim-seg[data-control=KEY]`, each `button[data-value]`; active one has class `active` and `aria-pressed="true"`; clicking the active one does nothing |
| `checkbox(parent, { label, checked = false, key, onChange })` → `{ el, get(), set(flag) }` | `input[type=checkbox][data-control=KEY]` |
| `button(parent, { label, onClick, title, primary = false, key })` → `<button type="button" class="sim-btn">` | `data-control=KEY`; `primary` adds class `primary` (filled blue) |
| `stats(parent)` → el | grid of tiles, `repeat(auto-fill, minmax(112px, 1fr))` |
| `stat(container, { label, key, title })` → `{ el, set(html), setState(state) }` | `el[data-stat=KEY]`, value element `.sim-stat-value`; the label is shown as written (normal case, 11.5px: write labels in lower case, e.g. `overshoot M<sub>p</sub>`); state `''`/`'ok'`/`'warn'`/`'bad'` → class `sim-ok`/`sim-warn`/`sim-bad`. `set` only touches the DOM when the HTML changes. |
| `tag(parent, { text = '', state = '', key })` → `{ el, set(text, state) }` | pill `span.sim-tag[data-tag=KEY]` (text, upper-cased by CSS); hidden when empty |
| `matrix(parent, { label, key })` → `{ el, set(rows, { decimals = 3, colColors, rowColors } = {}) }` | `el[data-matrix=KEY]` with an optional label and a bracketed `<table>`; numbers via `fmtFixed(decimals)`, strings as is; `colColors` wins over `rowColors`; cells are reused while the shape is unchanged (safe every frame) |
| `legend(parent, items)` → `{ el, set(items) }` | `items = [{ color, label, dash = false }]` |
| `note(parent, html)` → `<p class="sim-note">` | small muted paragraph |
| `hints(parent, items)` → `<details class="sim-hints" open>` | summary "Try this" and an `<ol>` of the HTML items |

### `RASim.canvas(parent, { aspect = 1.6, maxWidth = null, minHeight = 140, maxHeight = 560, key } = {})` → `cv`

Creates `<canvas class="sim-canvas" data-canvas="KEY">` (KEY defaults to `canvas`) filling the width of `parent`
(capped at `maxWidth` px and centred). CSS height = clamp(width / aspect, minHeight, maxHeight), rounded to whole pixels.
Backing store = CSS size × devicePixelRatio; the context is pre-scaled so you draw in CSS pixels. Sizes are tracked
with a ResizeObserver, window resize and a `matchMedia('(resolution: …dppx)')` listener (devicePixelRatio changes such as moving the window to another screen); every draw also re-checks devicePixelRatio.

| | |
|---|---|
| `cv.canvas`, `cv.ctx` | the element and its 2D context |
| `cv.width`, `cv.height` | current drawing size in CSS pixels (getters; inside the 1px border) |
| `cv.draw = function (ctx, w, h) { … }` | your drawing function. The core resets the transform, clears, and wraps the call in `save/restore`; exceptions are logged once (until a draw succeeds). |
| `cv.redraw()` | schedules one draw on the next animation frame (coalesced) |
| `cv.drawNow()` | draws synchronously (use inside `api.animate`) |
| `cv.setAspect(a)` | changes the aspect ratio, resizes and redraws |
| `cv.onDrag({ hit(x, y, info), move(x, y, token), end(token), cursor(x, y) })` | pointerdown → `hit`; `info = { pointerType }` (`'mouse'`, `'pen'` or `'touch'`), so a handler can return null for touches it wants to leave to page scrolling (e.g. a drag-anywhere orbit); a truthy return is the drag token: the pointer is captured and `move` gets every pointermove (coordinates may leave the canvas) until pointerup/pointercancel, then `end` (optional). `cursor` (optional) on hover returns a CSS cursor or null. Touch: a touch starting on a handle does not scroll the page (non-passive `touchstart` with `preventDefault` only when `hit` succeeds); a touch elsewhere scrolls normally. `hit` can be called twice for one touch (pointerdown and touchstart), so keep it free of side effects. Several `onDrag` calls are allowed: the first handler whose `hit` succeeds wins. |
| `cv.onHover({ move(x, y), leave() })` | mouse/pen hover tracking for read-out cursors (not touch) |

When the canvas has zero width (collapsed or `display: none`) drawing is skipped and done when it becomes visible again.
After every resize the core redraws synchronously (no blank frame).

### `RASim.plot(ctx, opts)` → `p`

Create it inside `cv.draw` on every draw (it keeps no state).
`opts = { box: { x, y, w, h }, margin: { left = 50, right = 14, top = 12 (28 with a title), bottom = 38 }, xmin, xmax, ymin, ymax, xlog = false, ylog = false, equal = false, xlabel = '', ylabel = '', title = '' }`.
`box` is the outer rectangle; labels, ticks and the title are drawn inside the margins. Partial `margin` objects merge
with the defaults. `equal: true` (linear axes only) widens the x- or y-range about its centre so one unit has the same
pixel length on both axes. Log axes need positive limits (non-positive limits fall back to 1e-3 / 10× min).
min > max are swapped; min == max is widened slightly.

| | |
|---|---|
| `p.rect` | `{ x, y, w, h }` inner data rectangle |
| `p.xmin, p.xmax, p.ymin, p.ymax` | limits after any `equal` adjustment |
| `p.X(x)`, `p.Y(y)`, `p.invX(px)`, `p.invY(py)`, `p.contains(px, py)` | data ↔ pixel |
| `p.axes({ grid = true, zero = true, frame = true, xTicks, yTicks, xFormat, yFormat } = {})` | white background, grid, linear ticks from `niceTicks` with about one tick per 50px (x) / 28px (y) and at least 3 when they fit, 11px tick labels in `colors.muted` (thinned to avoid overlap), axis labels (y rotated), bold title; `zero` draws darker x = 0 / y = 0 lines when inside. Log axes: decade majors labelled 0.01, 0.1, 1, 10 … (10ⁿ beyond 1e-3…1e4), minor grid at 2…9 (2 and 5 also labelled when less than two decades are visible). |

Drawing methods, all clipped to `p.rect` unless the style has `clip: false`; common style fields `color`
(default `colors.blue`; `colors.axis` for hline/vline; `colors.ink` for arc), `width` (2), `dash` (array or null), `fill`. All return `p`.

| | |
|---|---|
| `p.line(xs, ys, style)` | polyline; non-finite points break the path |
| `p.fn(f, style & { n = 400 })` | y = f(x) over the x-range (log-spaced on a log x-axis); throws/non-numbers break the path |
| `p.area(xs, ys, { fill, base = 0 })` | filled area to y = base |
| `p.band(y0, y1, { fill })`, `p.vband(x0, x1, { fill })` | horizontal / vertical strips |
| `p.hline(y, style)`, `p.vline(x, style)`, `p.segment(x1, y1, x2, y2, style)` | |
| `p.arrow(x1, y1, x2, y2, style & { head = 8 })` | head length in px |
| `p.point(x, y, { color, r = 4, fill = color, stroke = '#fff' })` | dot (`stroke: null` for none) |
| `p.cross(x, y, { color = red, size = 6, width = 2 })` | pole × |
| `p.ring(x, y, { color = blue, r = 5, width = 2 })` | zero ○ (white filled) |
| `p.circle(cx, cy, rData, style)` | radius in data units (ellipse when the axes are not equal); `fill` optional; `color: null` for no stroke |
| `p.arc(cx, cy, rPx, a0, a1, style & { ccw = true })` | angle arc at a data point, pixel radius, radians counter-clockwise from +x as seen (y up) |
| `p.text(x, y, str, { color = ink, font = '12px sans-serif', align = 'left', baseline = 'middle', dx = 0, dy = 0, bg = null, clip = false })` | text at a data point + pixel offset; `bg` draws a rounded box behind; text is **not** clipped unless `clip: true` |

### `RASim.draw` (free-form canvases, CSS pixels)

| | |
|---|---|
| `arrow(ctx, x1, y1, x2, y2, { color = ink, width = 2, head = 8, dash })` | line + filled head (head shrinks for short arrows) |
| `text(ctx, str, x, y, { color = ink, font = '12px sans-serif', align = 'left', baseline = 'middle', bg = null })` | |
| `spring(ctx, x1, y1, x2, y2, { coils = 8, amplitude = 8, color = ink, width = 2 })` | zig-zag with short straight leads |
| `hatch(ctx, x1, y1, x2, y2, { side = 1, length = 8, spacing = 8, color = ink })` | ground line with 45° hatching; `side = 1` hatches the right-hand side when travelling from (x1, y1) to (x2, y2) on screen (below a left-to-right line, left of a top-to-bottom line), `side = -1` the other side |
| `angleArc(ctx, cx, cy, r, a0, a1, { color = ink, width = 1.5, label, ccw = true, font })` | arc from a0 to a1, radians counter-clockwise from +x as seen (the helper flips for y-down), optional label just outside the arc at mid-angle |

## Files

- `core.js`, `core.css`: the core (do not edit; integration changes: `ui.slider`'s number box accepts a re-typed value
  after a track move, `onDrag` `hit` receives the pointer type, stat labels are no longer upper-cased).
- `test/load.js`: Node loader. `test/harness.js`: browser harness. `test/core.test.js`: core tests.
- `test/demo-sim.js`: id `demo`, a working example of every builder, plot method and draw helper, canvas drag, hover and animation. Read it for idioms. Run it with
  `node notes/sims/test/harness.js demo --file notes/sims/test/demo-sim.js --out <dir>`.
