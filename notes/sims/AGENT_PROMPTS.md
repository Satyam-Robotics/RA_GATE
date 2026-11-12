# Agent prompts for the interactive simulations

This file records the exact instructions given to every agent that builds the interactive simulations in `notes/RA_GATE2027_Notes.html`. It is generated from the orchestration scripts, so the text below is what the agents actually receive, not a paraphrase.

## 1. Who does what

Every agent runs on Claude Opus at high reasoning effort. Agents run strictly one after another.

| Stage | Agents | Prompt | Status when this file was written |
|---|---|---|---|
| Framework | 1 builder, then 1 reviewer | Section 3 | Finished and committed |
| Simulations | For each of the 12 simulations: 1 builder, then 1 independent verifier (24 agents) | Section 2 | Simulation 1 (`two-link-arm`) running; 2 to 12 not started |
| Integration | 1 | Section 4 | Not started; the prompt is a plan and may be adjusted to what the simulation agents report |

Order of the simulations:

1. `two-link-arm` — 2-link planar arm: forward and inverse kinematics
2. `rotation-transforms` — 3D rotations and homogeneous transforms
3. `eigen-2x2` — Eigenvalues and eigenvectors of a 2×2 matrix
4. `rlc-transient` — RC / RL / RLC transient response
5. `second-order-step` — Second-order step response and pole location
6. `root-locus` — Root locus explorer
7. `bode-plot` — Bode plot builder with gain and phase margins
8. `pid-tuning` — PID tuning lab
9. `mohr-circle` — Mohr's circle for plane stress
10. `vibration-sdof` — Single-DOF vibration: free, forced and isolation
11. `beam-sfd-bmd` — Beam: shear force, bending moment and deflection
12. `four-bar` — Four-bar linkage and Grashof's law

## 2. Simulation agents

### 2.1 How each prompt is assembled

All 24 simulation prompts are built from the same parts. Only the simulation id and the specification change.

| Prompt | Parts, in order |
|---|---|
| Build | build opening (2.3) → shared rules (2.2) → the simulation's specification (2.6) → build steps (2.3) |
| Verify | verify opening (2.4) → shared rules (2.2) → the same specification (2.6) → the builder's report → verify steps (2.4) |

In the text below `<id>` stands for the simulation id, for example `two-link-arm`. The two prompts sent for simulation 1 are exactly these parts with `<id>` = `two-link-arm` and specification 1. A full build plus verify prompt pair is between 34 and 44 thousand characters.

### 2.2 Shared rules (in every build and verify prompt)

````text
WHO USES THIS
A student preparing for the GATE 2027 Robotics & Automation exam studies from a notes web page. This simulation sits beside one section of those notes. It must be CORRECT above everything else — a wrong number, sign, unit or label teaches the wrong thing — then clear, then good-looking.

GROUND RULES
- Repo: /home/user/RA_GATE. The simulation framework ("core") is finished, reviewed and committed. Other finished simulations may already exist in notes/sims/; you may read them for idioms and visual consistency, never edit them.
- The simulation consists of exactly TWO files, and those are the only repo files that may be created or changed:
      notes/sims/<id>.js
      notes/sims/test/<id>.test.js
  Never edit notes/sims/core.js, core.css, README.md, anything else under notes/sims/test/, notes/build.js, notes/page.css, notes/src/*, the built HTML or PDF, or another simulation's files. Never run 'node notes/build.js'. No git writes at all (no add, commit, push, checkout, stash, reset, restore, clean).
- All temporary scripts, harness output and screenshots go under /tmp/claude-0/-home-user-RA-GATE/9d489dee-fbb1-53d8-91e1-ed21c31b514b/scratchpad/<id>/ (create it). Leave no stray files in the repo.
- No network, no CDNs, no npm install, no external libraries, fonts or images. Plain ES2019 JavaScript, Canvas 2D and DOM through the core. Node 22 and Playwright are installed: require('playwright') works from any directory and chromium.launch() needs no options. NEVER run 'playwright install'.
- The page uses html { scroll-behavior: smooth }: in Playwright scripts scroll with behavior 'instant' or wait about a second.
- Use the Read tool on PNG screenshots to actually LOOK at what was rendered. Never claim something looks right without looking.

READ FIRST
1. notes/sims/README.md — the complete API reference of the core (authoritative) and the rules for authors.
2. notes/sims/test/demo-sim.js — working idioms for every builder, the plot helper, drag, hover and animation.
3. notes/sims/core.js wherever exact behaviour matters.
4. The notes section named in the specification (under notes/src/), so that symbols, formulas and sign conventions match the notes exactly.

FILE CONTRACT
- notes/sims/<id>.js is an IIFE in strict mode that calls RASim.register('<id>', { title, model, mount }) with the exact title from the specification.
- model holds pure functions (no DOM) with EXACTLY the names, argument order and return shapes given in the specification — independent tests call them. Every number shown anywhere in the UI must come from the model (one source of truth) and be formatted with RASim.fmt / fmtFixed / fmtComplex / fmtPoly (real minus sign).
- mount(api) builds everything inside api.root and returns { getState(), setState(partial) } with exactly the state keys of the specification. setState must update the controls silently, recompute and redraw; it must accept any subset of the keys.
- Pass an explicit key to every control, stat, tag, matrix and canvas, using the keys given in the specification (square brackets / "key …"). Where the specification says "data key xyz" for a line of text, give that element the attribute data-key="xyz".
- The file starts with a header comment listing: the sign conventions, the STATE KEYS, the STAT / TAG / MATRIX keys and the CONTROL keys.
- Colours only from RASim.colors (and RASim.alpha). Animation only through api.animate. No requestAnimationFrame, setInterval or setTimeout loops. No globals. No innerHTML with anything but your own trusted strings.
- A Reset button restores every default. The hints from the specification go in ui.hints at the bottom of the panel.
- Layout: ui.split with the canvas(es) in the main column and controls, tag and stats in the side column, unless the specification says otherwise; group related controls with ui.group; legends via ui.legend. Start the panel body with a one-sentence ui.note saying what to do / look for.
- It must work from a 360 px wide phone (panel body about 300 px) up to the full 920 px page: no horizontal overflow, canvas text at least 11 px, no overlapping or clipped labels in the default state or in the "states to screenshot". On narrow canvases reduce label density rather than font size.
- Interaction must feel instant: a slider movement should cost well under 16 ms of script time; cache anything expensive that did not change.

IF THE SPECIFICATION AND CORRECT MATHEMATICS DISAGREE: implement what is mathematically correct, never loosen a tolerance to make a check pass, and report the disagreement under specDeviations with your derivation.
IF THE CORE LACKS SOMETHING OR MISBEHAVES: work around it inside your own file and report it under coreIssues. Do not edit the core.
````

### 2.3 Build prompt

**Opening**

````text
You are a senior engineer. Build ONE interactive study simulation, "<id>", exactly to the specification below.
````

*The shared rules (2.2) and the specification (2.6) are inserted here.*

**Steps**

````text
HOW TO WORK
1. Read the four things listed under READ FIRST.
2. Write the model first. Then write notes/sims/test/<id>.test.js (plain Node, require('assert'), loads the model with require('./load.js')('<id>')): assert EVERY check value of the specification (tolerance: as stated, otherwise 5e-4 relative or half a unit of the last printed digit, whichever is larger) and every listed invariant with seeded pseudo-random inputs (write a small deterministic generator; do not use Math.random without a seed). It prints one line per group and exits non-zero on any failure. Run it: node notes/sims/test/<id>.test.js
3. Build the user interface exactly as specified (controls with the given ranges, steps, defaults and keys; views; read-outs; hints).
4. Run the harness: node notes/sims/test/harness.js <id> --out /tmp/claude-0/-home-user-RA-GATE/9d489dee-fbb1-53d8-91e1-ed21c31b514b/scratchpad/<id>/h — it must report zero errors AND zero warnings at widths 1100 and 390. Read both screenshots and fix what is wrong or ugly. Repeat until clean.
5. Write a Playwright interaction script in /tmp/claude-0/-home-user-RA-GATE/9d489dee-fbb1-53d8-91e1-ed21c31b514b/scratchpad/<id>/ using openSim from the harness. It must: (a) drive every control to its minimum, maximum and default, press every button, choose every preset, perform every drag interaction described (with real mouse events on the canvas), toggle the panel's Hide/Show, and resize the viewport from 1100 to 390 and back; (b) after each action assert no console errors or warnings and that the panel's text contains none of: NaN, undefined, Infinity, null, [object, and no ASCII hyphen-minus directly before a digit in any read-out; (c) for the specification's check values that correspond to a state, set that state with setState and compare the displayed read-outs ([data-stat], [data-tag], [data-matrix], [data-key]) with the expected numbers; (d) save a screenshot of every state listed under STATES TO SCREENSHOT (use setState and real drags as needed). Read every screenshot and fix defects: overlapping labels, clipped text, arrows or arcs pointing the wrong way, wrong colours, unreadable density, misaligned plots.
6. Measure the script time of the heaviest update (performance.now around setState / a slider input event inside page.evaluate, median of 20) and optimise if it exceeds 16 ms.
7. Re-read your own file once from top to bottom as a reviewer would: every formula against the specification, every unit, every sign, every label, degrees versus radians at each boundary, stale values after preset or mode changes, Reset restoring everything.
8. Finish by running the Node test and the harness one last time.

DEFINITION OF DONE
- The Node test passes and covers every check value and invariant.
- The harness is clean at 1100 and 390; every screenshot state was inspected and looks correct and tidy.
- Every item of the specification is implemented (controls, keys, views, read-outs, hints, state). Anything you could not do is listed honestly under openIssues.
Your final answer is the structured result.
````

### 2.4 Verify prompt

**Opening**

````text
You are an independent verifier with full authority to fix. Another engineer has just built the study simulation "<id>" from the specification below. A student will learn exam facts from it, so a wrong number, sign, unit, label, arrow direction or stale read-out is a serious defect. Your job is to FIND defects by exercising the simulation against the specification, and to FIX every defect you find (in the simulation's two files only). Assume there are bugs. Do not trust the builder's claims or the builder's test file — its expected values may come from the same wrong code.
````

*The shared rules (2.2) and the specification (2.6) are inserted here.*

**Builder's report and steps**

````text
THE BUILDER REPORTED:
<<the structured report returned by the previous agent is inserted here as JSON>>

WHAT TO DO (work in /tmp/claude-0/-home-user-RA-GATE/9d489dee-fbb1-53d8-91e1-ed21c31b514b/scratchpad/<id>/verify/)
1. Read notes/sims/<id>.js and notes/sims/test/<id>.test.js completely, the README, and the notes section the simulation sits beside.
2. Specification conformance, item by item: every model function (name, argument order, return shape, units), every control (label, range, step, default, key), every view element, every read-out and key, every hint, every state key. Write the list of mismatches down and fix them.
3. Independent mathematics. Write your OWN reference implementation from first principles in a separate file, using a DIFFERENT method from the simulation wherever one exists (numerical integration of the differential equation instead of the closed form; brute-force search instead of an analytic extreme; finite differences instead of analytic derivatives; complex arithmetic instead of magnitude/phase formulas; direct geometric construction instead of trigonometric formulas). Compare it with the simulation's model on at least 300 seeded random parameter sets spanning the full ranges of the controls, plus every edge case (zeros, extremes of every slider, degenerate and limit cases, sign changes, values straddling every case boundary). Then check every CHECK VALUE of the specification against the model. If a check value in the specification is itself wrong, prove it with two independent calculations and report it under specDeviations — the simulation must follow the correct mathematics.
4. Truth of the display. In the browser (openSim from the harness), for at least 12 states including all check-value states: set the state, read EVERY read-out from the DOM ([data-stat] .sim-stat-value, [data-tag], [data-matrix] cells, [data-key] lines), parse the numbers (mind the Unicode minus and units) and compare them with your reference values. This catches wiring mistakes: the wrong variable shown, degrees versus radians, wrong units or prefixes, stale values after a preset, tab, mode or Reset.
5. Truth of the drawing. For geometric views (arms, frames, circles, elements, linkages, beams, vectors, poles): compute where key points must appear in canvas pixels and confirm by sampling canvas pixel colours there or by comparing with positions the code exposes; and inspect screenshots of at least 8 states, including negative values and sign flips, to confirm that arrows, arcs, rotation senses and above/below-axis conventions are right and match the stated sign conventions. For plots: check a few plotted points against the model by pixel sampling (curve colour at the expected pixel), and axis labels, units and tick values.
6. Every control and interaction with real input events: sliders (drag and typed values), buttons, presets, selects, checkboxes, every canvas drag with the mouse AND with touch emulation (openSim with touch: true; the page must not scroll while dragging a handle), hover read-outs, animation start / pause / restart, Hide / Show, viewport resize 1100 → 390 → 1100, Reset from several dirty states (getState afterwards must equal the initial state). After each: no console errors or warnings; no NaN / undefined / Infinity / null / [object in the panel text; no ASCII hyphen-minus before a digit in read-outs; no horizontal overflow.
7. Check each HINT by doing what it says in the simulation: the stated outcome must actually happen and the stated numbers must be what the read-outs show.
8. Code review of the whole file: stale caches, missing redraws, listeners added repeatedly, direct requestAnimationFrame / setInterval / setTimeout loops (forbidden), colours not from RASim.colors, numbers displayed without the RASim formatters, performance of the heaviest update (measure; it should be well under 16 ms).
9. Look at the final screenshots at 1100 and 390 wide for every state under STATES TO SCREENSHOT: correct, tidy, legible, no overlapping or clipped labels.
10. Fix everything you found. For each real bug add a regression test to notes/sims/test/<id>.test.js (the test must fail on the old code). Make sure the test file asserts every check value and invariant of the specification. Re-run after fixing: node notes/sims/test/<id>.test.js, then as the very last step the harness into the final folder: node notes/sims/test/harness.js <id> --out /tmp/claude-0/-home-user-RA-GATE/9d489dee-fbb1-53d8-91e1-ed21c31b514b/scratchpad/<id>/final — zero errors and zero warnings — and read both final screenshots.

Your final answer is the structured result. checksRun must list the concrete checks with the numbers you observed. List every defect with its severity and whether it is fixed. Be exact and honest: anything not verified goes under openIssues.
````

### 2.5 The result each agent must return

Each agent has to finish by filling in a fixed form (a JSON schema), so its report cannot skip a field.

**Builder**

```json
{
  "type": "object",
  "properties": {
    "id": {
      "type": "string"
    },
    "summary": {
      "type": "string",
      "description": "What was built, in 5-10 sentences"
    },
    "modelApi": {
      "type": "array",
      "items": {
        "type": "string"
      },
      "description": "Signatures of the model functions as implemented"
    },
    "stateKeys": {
      "type": "array",
      "items": {
        "type": "string"
      }
    },
    "statKeys": {
      "type": "array",
      "items": {
        "type": "string"
      },
      "description": "data-stat / data-tag / data-matrix / data-key keys"
    },
    "testsPass": {
      "type": "boolean",
      "description": "node notes/sims/test/<id>.test.js exits 0"
    },
    "harnessClean": {
      "type": "boolean",
      "description": "harness at 1100 and 390: zero errors and zero warnings"
    },
    "worstUpdateMs": {
      "type": "number",
      "description": "Measured script time of the heaviest single update, in ms"
    },
    "screenshots": {
      "type": "array",
      "items": {
        "type": "string"
      },
      "description": "Absolute paths of the screenshots you inspected"
    },
    "specDeviations": {
      "type": "array",
      "items": {
        "type": "string"
      }
    },
    "coreIssues": {
      "type": "array",
      "items": {
        "type": "string"
      }
    },
    "openIssues": {
      "type": "array",
      "items": {
        "type": "string"
      },
      "description": "Anything not done, not verified or not as specified"
    }
  },
  "required": [
    "id",
    "summary",
    "modelApi",
    "stateKeys",
    "statKeys",
    "testsPass",
    "harnessClean",
    "worstUpdateMs",
    "screenshots",
    "specDeviations",
    "coreIssues",
    "openIssues"
  ]
}
```

**Verifier**

```json
{
  "type": "object",
  "properties": {
    "id": {
      "type": "string"
    },
    "verdict": {
      "type": "string",
      "enum": [
        "pass",
        "fixed",
        "fail"
      ],
      "description": "pass = no defects found; fixed = defects found and all fixed; fail = defects remain"
    },
    "summary": {
      "type": "string"
    },
    "checksRun": {
      "type": "array",
      "items": {
        "type": "string"
      },
      "description": "Concrete checks you executed, each with the observed numbers"
    },
    "defects": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "severity": {
            "type": "string",
            "enum": [
              "critical",
              "major",
              "minor"
            ]
          },
          "description": {
            "type": "string"
          },
          "fixed": {
            "type": "boolean"
          }
        },
        "required": [
          "severity",
          "description",
          "fixed"
        ]
      }
    },
    "testsPass": {
      "type": "boolean"
    },
    "harnessClean": {
      "type": "boolean"
    },
    "screenshots": {
      "type": "array",
      "items": {
        "type": "string"
      },
      "description": "Absolute paths of the final screenshots (after fixes)"
    },
    "specDeviations": {
      "type": "array",
      "items": {
        "type": "string"
      },
      "description": "Every remaining difference between the simulation and the specification, with the reason"
    },
    "coreIssues": {
      "type": "array",
      "items": {
        "type": "string"
      }
    },
    "openIssues": {
      "type": "array",
      "items": {
        "type": "string"
      }
    }
  },
  "required": [
    "id",
    "verdict",
    "summary",
    "checksRun",
    "defects",
    "testsPass",
    "harnessClean",
    "screenshots",
    "specDeviations",
    "coreIssues",
    "openIssues"
  ]
}
```

### 2.6 The 12 specifications

Each specification fixes the controls, ranges, defaults, formulas, sign conventions, read-outs and check values for one simulation. The check values were computed independently before any agent started.

#### 2.6.1 `two-link-arm`

````text
SIMULATION two-link-arm — title "2-link planar arm: forward and inverse kinematics"
SITS BESIDE: notes/src/03_robotics.html, section "5. Forward Kinematics" (the bullets on the 2-link planar arm, inverse kinematics and the Jacobian). Use the same symbols: L₁, L₂, θ₁, θ₂, φ, det J.

PURPOSE
The student should come away knowing: forward kinematics x = L₁cos θ₁ + L₂cos(θ₁+θ₂), y = L₁sin θ₁ + L₂sin(θ₁+θ₂); that one reachable point has two inverse-kinematics solutions (elbow up / elbow down); that the workspace is the annulus |L₁−L₂| ≤ r ≤ L₁+L₂; and that det J = L₁L₂ sin θ₂ vanishes when the arm is stretched or folded (singularity), where the tip can no longer move along the arm.

DEFINITIONS AND CONVENTIONS
- Base (joint 1) at the origin. x to the right, y up. All angles counter-clockwise positive. θ₁ is measured from the +x axis to link 1. θ₂ is the RELATIVE angle from the direction of link 1 to link 2. φ = θ₁ + θ₂ is the tip orientation.
- Elbow E = (L₁cos θ₁, L₁sin θ₁). Tip P = E + (L₂cos(θ₁+θ₂), L₂sin(θ₁+θ₂)).
- Names of the two IK branches (fixed; show them exactly like this): "Elbow down (θ₂ ≥ 0)" and "Elbow up (θ₂ ≤ 0)".
- Inverse kinematics for a target (x, y), r² = x² + y²:
    c₂ = (r² − L₁² − L₂²) / (2 L₁ L₂);  reachable iff |c₂| ≤ 1 (equivalently |L₁ − L₂| ≤ r ≤ L₁ + L₂)
    θ₂ = +acos(c₂) for elbow down, −acos(c₂) for elbow up
    θ₁ = atan2(y, x) − atan2(L₂ sin θ₂, L₁ + L₂ cos θ₂), wrapped to (−π, π]
  Unreachable target: project it radially onto the nearest workspace boundary (r clamped to [|L₁−L₂|, L₁+L₂], direction of the target kept; if the target is exactly the origin use the direction of the +x axis) and solve for that point; reachable = false.
  Degenerate case: solved point at the origin (only possible when L₁ = L₂): θ₂ = ±π and θ₁ = the caller-supplied current θ₁.
- The other solution reaching the same tip from (θ₁, θ₂): θ₂' = −θ₂, θ₁' = θ₁ + 2·atan2(L₂ sin θ₂, L₁ + L₂ cos θ₂), wrapped.
- Jacobian (tip velocity = J · joint rates): J = [[−L₁s₁ − L₂s₁₂, −L₂s₁₂], [L₁c₁ + L₂c₁₂, L₂c₁₂]], det J = L₁ L₂ sin θ₂.
- Singularity flag: |sin θ₂| < 0.05: 'stretched' when cos θ₂ > 0, 'folded' when cos θ₂ < 0.

MODEL (exact names; angles in radians)
  fk(L1, L2, t1, t2) -> { x, y, phi, ex, ey }          phi wrapped to (−π, π]
  ik(L1, L2, x, y, elbow, t1Current = 0) -> { t1, t2, reachable, x, y }     elbow = +1 (down, θ₂ ≥ 0) or −1 (up); x, y in the result are the point actually solved for (the clamped point when unreachable)
  other(L1, L2, t1, t2) -> { t1, t2 }
  jacobian(L1, L2, t1, t2) -> [[a, b], [c, d]]
  detJ(L1, L2, t2) -> number
  tipVelocity(L1, L2, t1, t2, w1, w2) -> { vx, vy, speed }
  workspace(L1, L2) -> { rMin, rMax }
  singular(t2) -> '' | 'stretched' | 'folded'

CONTROLS
  L₁ slider 0.5–2.0 m, step 0.05, default 1.0 (key L1).  L₂ slider 0.5–2.0 m, step 0.05, default 0.8 (key L2).
  θ₁ slider −180…180°, step 1, default 30 (key theta1).  θ₂ slider −180…180°, step 1, default 60 (key theta2).
  IK branch: segmented "Elbow down (θ₂ ≥ 0)" / "Elbow up (θ₂ ≤ 0)" (key elbow). It selects the branch used while dragging the tip. Clicking it also immediately switches the arm to that branch for the current tip position when the current θ₂ has the other sign — the tip stays where it is. When θ₂ is changed by its slider, the segmented control follows the sign of θ₂ (θ₂ = 0 keeps the previous choice).
  Group "Joint rates → tip velocity": θ̇₁ and θ̇₂ sliders −2…2 rad/s, step 0.1, defaults 1.0 and 0.5 (keys w1, w2).
  Checkboxes: "Workspace" (default on, key showWorkspace), "Other IK solution" (default on, key showOther), "Tip velocity" (default on, key showVelocity).
  Button Reset.
  Drag: the tip (end-effector) is a draggable handle on the canvas (hit radius at least 14 px, grab cursor). Dragging solves IK with the selected branch on every move and updates the θ₁ and θ₂ sliders. Outside the workspace the arm follows along the boundary and the status tag shows OUTSIDE WORKSPACE with the note "no IK solution: |cos θ₂| > 1"; the tag returns to normal when the drag ends or re-enters.

VIEWS
 One canvas with equal scale, the base at its centre, half-width 1.15·(L₁+L₂) in world units. Draw back to front: light grid every 0.5 m with axis lines and tick numbers; the workspace annulus when enabled (light fill between rMin and rMax, both circles outlined dashed, radii labelled "L₁+L₂" and "|L₁−L₂|"); the other IK solution as a faint dashed ghost arm when enabled and the arm is not singular; link 1 (blue, thick), link 2 (maroon, thick), joint circles, a small ground symbol at the base; the θ₁ arc from the +x axis to link 1 with its label; a dashed extension of link 1 beyond the elbow and the θ₂ arc from that extension to link 2 with its label (arcs must run the correct way for negative angles); the tip handle; a label "(x, y)" with the tip coordinates; the tip velocity arrow (orange, starting at the tip) when enabled, drawn with a fixed scale that keeps it visible — state the scale in the legend.
 An HTML legend under the canvas.

READ-OUTS (ui.stats; keys in square brackets)
  x [x] and y [y] in m, 3 decimals; φ [phi] in °; r [r]; det J [detJ] (state bad when singular); tip speed [speed] and components [vx], [vy] in m/s; the other solution [other] as "θ₁′ = …°, θ₂′ = …°".
  Matrix J, 2×2 [matrix key J], 3 decimals.
  Tag [key status]: REGULAR (ok) / SINGULAR — STRETCHED (bad) / SINGULAR — FOLDED (bad) / OUTSIDE WORKSPACE (warn, only while dragging outside).
  Two live formula lines (ui.note, keys formulaX and formulaY via data attributes of your choice): "x = L₁cos θ₁ + L₂cos(θ₁+θ₂) = 1·cos 30° + 0.8·cos 90° = 0.866" and the same for y, with the current numbers.

STATE (getState / setState): { L1, L2, theta1, theta2 (degrees), elbow ('down' | 'up'), w1, w2, showWorkspace, showOther, showVelocity }

CHECK VALUES (model)
  fk(1, 1, 0, π/2) -> x = 1, y = 1, phi = 90°; detJ = 1
  fk(1, 0.8, 30°, 60°) -> x = 0.86603, y = 1.30000; detJ = 0.69282
  ik(1, 1, 1, 1, +1) -> θ₁ = 0°, θ₂ = 90°;   ik(1, 1, 1, 1, −1) -> θ₁ = 90°, θ₂ = −90°
  ik(2, 1, 3, 0, ±1) -> θ₁ = 0, θ₂ = 0, reachable true
  ik(1, 1, 3, 0, +1) -> reachable false, solved point (2, 0), θ₁ = 0, θ₂ = 0
  ik(2, 1, 0.5, 0, +1) -> reachable false, solved point (1, 0) on the inner boundary, θ₂ = 180°
  jacobian(1, 1, 0, π/2) = [[−1, −1], [1, 0]];  tipVelocity(1, 1, 0, π/2, 1, 0.5) -> vx = −1.5, vy = 1
  other(1, 1, 0, π/2) -> (π/2, −π/2)
  Invariants (random tests, 500 samples over the slider ranges): ik(fk(θ)) with elbow = sign(θ₂) reproduces (θ₁, θ₂) modulo 2π and the point to 1e-9; fk(other(θ)) gives the same tip position; det of jacobian() equals detJ(); jacobian() equals the central finite-difference derivative of fk to 1e-6; workspace radii; at θ₂ = 0 the tip velocity is perpendicular to the arm direction for any joint rates.

HINTS (use these)
  1. Set θ₂ = 0°: det J = 0, the arm is stretched and the tip sits on the outer circle. Now change the joint rates — the tip velocity stays perpendicular to the arm: it cannot move radially.
  2. Drag the tip somewhere inside the ring, then switch Elbow down / Elbow up: the same (x, y), two different joint solutions, and φ changes.
  3. Set L₁ = L₂: the hole in the middle disappears (inner radius |L₁ − L₂| = 0).
  4. Drag the tip outside the outer circle: |cos θ₂| > 1, so there is no solution — the arm can only point at the target.

STATES TO SCREENSHOT: default; θ₂ = 0 (singular, stretched); θ₂ = −120° with the other solution visible; L₁ = L₂ = 1 with the tip dragged to (0.3, 0.2); tip dragged outside the workspace (mid-drag); 390-wide default.
````

#### 2.6.2 `rotation-transforms`

````text
SIMULATION rotation-transforms — title "3D rotations and homogeneous transforms"
SITS BESIDE: notes/src/03_robotics.html, sections "3. Rotation Matrices" and "4. Homogeneous Transformations".

PURPOSE
(1) Elementary rotations, and that the COLUMNS of the rotation matrix of frame B relative to frame A are B's unit axes written in A. (2) Order matters, with the rule from the notes: rotation about the FIXED frame → pre-multiply; about the CURRENT (moving) frame → post-multiply. (3) The homogeneous transform T = [[R, d], [0 0 0 1]], the mapping of a point p_A = R·p_B + d, and T⁻¹ = [[Rᵀ, −Rᵀd], [0 0 0 1]]. (4) Every rotation is a single rotation about one axis: trace R = 1 + 2 cos θ.

DEFINITIONS
- Right-handed frames. Frame A is fixed (world); frame B moves and starts coincident with A.
- Elementary matrices exactly as in the notes (c = cos θ, s = sin θ):
    Rx(θ) = [[1,0,0],[0,c,−s],[0,s,c]],  Ry(θ) = [[c,0,s],[0,1,0],[−s,0,c]],  Rz(θ) = [[c,−s,0],[s,c,0],[0,0,1]]
- The student defines THREE rotation steps applied in the order 1, 2, 3; each step has an axis (X, Y or Z) and an angle. With step matrices R₁, R₂, R₃:
    mode 'fixed' (every step is about the FIXED frame A's axes):      R = R₃ · R₂ · R₁   (each new rotation PRE-multiplies)
    mode 'current' (every step is about B's own CURRENT axes):        R = R₁ · R₂ · R₃   (each new rotation POST-multiplies)
- d = (dx, dy, dz) is the position of B's origin in A, independent of the steps. T = [[R, d], [0, 0, 0, 1]].
- A point P rigidly attached to B with coordinates p_B; p_A = R·p_B + d.
- T⁻¹ = [[Rᵀ, −Rᵀd], [0, 0, 0, 1]].
- Axis–angle of R: θ = acos(clamp((trace R − 1)/2, −1, 1)) ∈ [0, π]. If θ < 1e-6 the axis is undefined (null). If π − θ < 1e-6: take kᵢ = sqrt(max(0, (Rᵢᵢ + 1)/2)), make the largest component positive and give the others the sign of (R_ij + R_ji) with respect to it. Otherwise k = [R₃₂ − R₂₃, R₁₃ − R₃₁, R₂₁ − R₁₂]/(2 sin θ) (1-based row, column indices).

MODEL (angles in radians; matrices are arrays of rows)
  Rx(a), Ry(a), Rz(a) -> 3×3;  rot(axis, a) with axis 'x' | 'y' | 'z'
  mul(A, B);  transpose(A);  det3(A);  trace(A)
  compose(steps, mode) -> 3×3      steps = [{ axis, angle }, …] in order of application; mode 'fixed' | 'current'
  partial(steps, mode, k, f) -> 3×3     the rotation when steps 1 … k−1 are complete and step k (1-based) is at fraction f ∈ [0, 1] of its angle (used by the animation): fixed: R_k(f·θ_k)·R_(k−1)·…·R₁; current: R₁·…·R_(k−1)·R_k(f·θ_k)
  homogeneous(R, d) -> 4×4;  invHomogeneous(T) -> 4×4;  apply(T, p) -> [x, y, z]
  axisAngle(R) -> { angle, axis }     axis = unit [kx, ky, kz] or null
  formula(steps, mode) -> plain string such as 'R = Rz(90°) · Rx(90°)' listing only the non-zero steps in the correct multiplication order for the mode; all zero -> 'R = I'

CONTROLS
  Mode: segmented "About FIXED axes (pre-multiply)" / "About CURRENT axes (post-multiply)" (key mode; default fixed).
  Steps 1, 2, 3: each a row with an axis segmented control X / Y / Z (keys axis1, axis2, axis3) and an angle slider −180…180°, step 1 (keys angle1, angle2, angle3). Defaults: step 1 = X 90°, step 2 = Z 90°, step 3 = Y 0°.
  Presets (select, key preset): "X 90° then Z 90° — order matters" (the defaults; mode unchanged) · "Roll–pitch–yaw, fixed XYZ: Rz(60°)·Ry(45°)·Rx(30°)" (mode fixed; X 30, Y 45, Z 60) · "ZYZ Euler angles: Rz(30°)·Ry(45°)·Rz(60°)" (mode current; Z 30, Y 45, Z 60) · "Single rotation Rz(90°)" (Z 90, others 0) · "Identity".
  Translation d: sliders dx, dy, dz, −2…2, step 0.1, default 0 (keys dx, dy, dz).
  Point p_B: sliders px, py, pz, −1.5…1.5, step 0.1, default 1, 1, 0 (keys px, py, pz).
  Checkboxes: "Ghost of the other order" (default on, key showGhost): draws, faint and dashed, frame B as it would end with the same three steps in the OTHER mode; "Equivalent axis" (default on, key showAxis); "Body box" (default on, key showBox).
  Buttons: "Play steps" — animates step 1, then 2, then 3 (about 1.2 s each, skipping zero-angle steps) using model.partial; while a step animates, the axis it turns about is highlighted as a thick dashed line (a fixed axis of A in fixed mode; the body's current axis in current mode) and that step's row is highlighted; when it finishes the final R is shown. Pressing again restarts. Any control change stops the animation and shows the final state. "Reset".
  View: dragging on the canvas orbits the camera (azimuth and elevation; elevation clamped to ±85°). Default view: the textbook view with Z_A up, X_A towards the lower left and Y_A towards the lower right.

VIEWS
 One canvas with an orthographic 3D projection that you implement yourself: world Z up. Draw: a faint ground grid in A's XY plane (−2…2); frame A axes (thin, muted, labelled X_A, Y_A, Z_A, length 1.6); frame B axes at origin d (bold with arrow heads, length 1: x_B in RASim.colors.red, y_B in green, z_B in blue, labelled); a wire-frame box attached to B (1 × 0.6 × 0.3 along x_B, y_B, z_B with one corner at B's origin) so the orientation reads at a glance; the ghost frame; the vector d from A's origin to B's origin (dashed) when d ≠ 0; the point P (dot, labelled P) with a thin line from B's origin; the equivalent rotation axis through B's origin (purple dashed, labelled "k, θ = …°") when defined and enabled. Draw roughly back to front so nearer items cover farther ones. Labels must stay legible.
 Side column: the formula line (key formula), e.g. "R = Rz(90°) · Rx(90°)"; matrix R (3×3, key R) with its three COLUMNS coloured red / green / blue to match x_B, y_B, z_B and the caption "columns = x_B, y_B, z_B expressed in A"; matrix T (4×4, key T); matrix T⁻¹ (4×4, key Tinv).
 Stats: det R [detR]; trace R [trace]; equivalent angle θ [eqAngle] in °; axis k [eqAxis] as "(kx, ky, kz)" or "—"; p_A [pA] as "(x, y, z)"; orthogonality check [orth]: the largest |RᵀR − I| entry (≈ 0).

STATE: { mode, axis1, angle1, axis2, angle2, axis3, angle3 (degrees), dx, dy, dz, px, py, pz, showGhost, showAxis, showBox, az, el (degrees), animStep (0 when not animating, else 1…3), animFrac (0…1) }   setState({ animStep, animFrac }) freezes the picture at that moment of the animation (for deterministic screenshots); any other change clears it.

CHECK VALUES
  compose([X 90°, Z 90°], 'fixed')   = Rz(90°)·Rx(90°) = [[0,0,1],[1,0,0],[0,1,0]]
  compose([X 90°, Z 90°], 'current') = Rx(90°)·Rz(90°) = [[0,−1,0],[0,0,−1],[1,0,0]]
  axisAngle of the first: 120° about (1, 1, 1)/√3;  of the second: 120° about (1, −1, 1)/√3
  The RPY preset equals Rz(60°)·Ry(45°)·Rx(30°); the ZYZ preset equals Rz(30°)·Ry(45°)·Rz(60°) (compute both sides independently in the test).
  With R = Rz(90°), d = (1, 2, 3): the translation column of T⁻¹ is (−2, 1, −3); apply(T, [1, 0, 0]) = (1, 3, 3); T·T⁻¹ = I.
  axisAngle(Rx(180°)) -> 180° about (1, 0, 0) (sign-free: ±);  axisAngle(I) -> angle 0, axis null;  axisAngle(Rz(−90°)) -> 90° about (0, 0, −1).
  Invariants over 300 random step sets in both modes: det R = 1; RᵀR = I to 1e-12; Rodrigues with axisAngle (R = I + sin θ·[k]× + (1 − cos θ)·[k]×²) reproduces R to 1e-9; R·k = k; partial(steps, mode, 3, 1) = compose(steps, mode); partial(…, k, 0) = partial(…, k − 1, 1); the fixed-mode result of steps (s1, s2, s3) equals the current-mode result of the reversed list (s3, s2, s1); invHomogeneous(T)·T = I.
  Projection sanity (browser test): with the default view the screen positions of the tips of X_A, Y_A, Z_A form a right-handed picture (Z up; rotating X_A towards Y_A is counter-clockwise when seen from above).

HINTS (use these)
  1. With steps X 90° then Z 90°, switch between FIXED and CURRENT axes: the final frames differ — rotations do not commute. The ghost shows the other answer.
  2. Read the columns of R: column 1 is where x_B points in frame A, column 2 is y_B, column 3 is z_B. Compare them with the drawn axes.
  3. Set d = (1, 2, 0): T⁻¹ is not Tᵀ — its translation part is −Rᵀd.
  4. Whatever the three steps, R is one rotation by θ about a single axis k (purple): trace R = 1 + 2 cos θ.
  5. Press Play steps in each mode and watch which axis each step turns about.

STATES TO SCREENSHOT: default (fixed mode); the same in current mode; RPY preset; a frozen animation frame (animStep 2, animFrac 0.5) in each mode; d = (1, 2, 0.5) with the point; 390-wide default.
````

#### 2.6.3 `eigen-2x2`

````text
SIMULATION eigen-2x2 — title "Eigenvalues and eigenvectors of a 2×2 matrix"
SITS BESIDE: notes/src/01_math.html, section "Eigenvalues and eigenvectors" (under "1. Linear Algebra").

PURPOSE
See what A = [[a, b], [c, d]] does to the plane: the columns of A are where the basis vectors land; |det A| is the area scale factor (a negative det flips orientation); an eigenvector is a direction that A does not turn (Ax = λx) and λ is its stretch factor; Σλ = trace and Πλ = det; symmetric matrices have perpendicular eigenvectors; some matrices have complex eigenvalues (no real invariant direction) or only one eigenvector direction (defective).

DEFINITIONS
- Characteristic equation λ² − (tr)λ + det = 0 with tr = a + d, det = ad − bc, disc = tr² − 4·det.
- Classification with tolerance eps = 1e-9·max(1, tr², |det|):
    disc > eps      'real-distinct': λ₁ = (tr + √disc)/2 > λ₂ = (tr − √disc)/2
    |disc| ≤ eps    repeated λ = tr/2: 'repeated-scalar' if |b| ≤ 1e-12 and |c| ≤ 1e-12 (then A = λI and EVERY non-zero vector is an eigenvector), else 'repeated-defective' (only one eigenvector direction: algebraic multiplicity 2, geometric multiplicity 1, not diagonalizable)
    disc < −eps     'complex': λ = tr/2 ± j√(−disc)/2; no real eigenvectors
- Eigenvector for a real eigenvalue λ: if |b| > 1e-12: v = (b, λ − a); else if |c| > 1e-12: v = (λ − d, c); else (diagonal matrix): v = (1, 0) for λ = a and (0, 1) for λ = d. Normalise to unit length and fix the sign so that the first non-zero component is positive.
- Morph matrix for the animation: M(t) = (1 − t)·I + t·A, t ∈ [0, 1].

MODEL
  eig(a, b, c, d) -> { trace, det, disc, type, values: [{ re, im }, { re, im }], vectors: [v1 | null, v2 | null] }
      values: real-distinct -> [λ₁, λ₂] with λ₁ > λ₂; repeated -> both equal; complex -> [+imaginary part first, −imaginary part second].
      vectors: real-distinct -> [vector of λ₁, vector of λ₂]; repeated-defective -> [v, null]; repeated-scalar -> [[1, 0], [0, 1]]; complex -> [null, null].
  apply(a, b, c, d, x, y) -> [x', y']
  morph(a, b, c, d, t) -> [a', b', c', d']
  props(a, b, c, d) -> { symmetric (|b − c| ≤ 1e-12), singular (|det| < 1e-12), orthogonal (AᵀA = I within 1e-9), diagonalizable (false only for 'repeated-defective'), rank (0, 1 or 2), areaScale (|det|), flips (det < 0) }
  angleBetween(x1, y1, x2, y2) -> radians in [0, π]
  charEq(a, b, c, d) -> plain string such as 'λ² − 4λ + 3 = 0' (real minus sign, zero terms omitted, coefficient 1 omitted)

CONTROLS
  Four sliders a, b, c, d: −3…3, step 0.1 (keys a, b, c, d), laid out 2 × 2 like the matrix. Default A = [[2, 1], [1, 2]].
  Preset select (key preset): "Symmetric [[2,1],[1,2]]" (default) · "Shear [[1,1],[0,1]] — one eigenvector only" · "Rotation 90° [[0,−1],[1,0]] — complex λ" · "Scaling [[2,0],[0,0.5]]" · "Singular [[1,2],[2,4]] — λ = 0" · "Reflection [[0,1],[1,0]]" · "Projection [[1,0],[0,0]]" · "Uniform scaling [[2,0],[0,2]] — every vector".
  Morph t: slider 0…1, step 0.01, default 1 (key t), with a "Play" button that animates t from 0 to 1 in about 2 s (api.animate) and then stops.
  Test vector x: a draggable handle on the canvas, default (1, 0.5) (state keys vx, vy); buttons "Snap x to v₁" and "Snap x to v₂" (disabled when that eigenvector does not exist).
  Checkboxes: "Transformed grid" (default on, key showGrid), "Unit circle" (default on, key showCircle), "Eigen-directions" (default on, key showEigen).
  Reset.

VIEWS
 One canvas with equal axes, x-range ±4 (y follows from the aspect ratio). Back to front:
  - a faint untransformed integer grid and axes with tick labels;
  - the image of the integer grid lines (−4…4) under M(t) as light blue lines, when enabled;
  - the image of the unit square under M(t) as a filled parallelogram (light green when det M > 0, light red when det M < 0) with the label "area = |det| = …" (the value for M(t));
  - the unit circle (dashed, faint) and its image under M(t) (purple ellipse), when enabled;
  - the images of the basis vectors under M(t): first column (red arrow, labelled "A e₁ = (a, c)" with numbers) and second column (green arrow, labelled "A e₂ = (b, d)");
  - the eigen-directions of A (of A itself, not of M(t)), when enabled: for each real eigenvector an infinite dashed line through the origin (maroon for λ₁, teal for λ₂) labelled "λ₁ = …" / "λ₂ = …"; nothing for the complex and repeated-scalar cases;
  - the test vector x (blue arrow) and A·x (orange arrow; always the full A, independent of t), labelled x and Ax.
 Legend below.

READ-OUTS
  Matrix A [matrix key A]. Stats: trace [trace]; det [det]; characteristic equation [charEq]; λ₁ [lambda1] and λ₂ [lambda2] (fmtComplex for complex values); v₁ [v1] and v₂ [v2] as "(0.707, 0.707)" or "—" ("any direction" for repeated-scalar); λ₁ + λ₂ [sum] and λ₁·λ₂ [prod] (placed next to trace and det so the student sees they match; for a complex pair the sum is 2·Re λ and the product is |λ|²); the angle between x and Ax [angle] in °; |Ax| / |x| [stretch].
  Tag [key type]: REAL, DISTINCT / REPEATED — DEFECTIVE (1 EIGENVECTOR) / REPEATED — EVERY VECTOR / COMPLEX — NO REAL EIGENVECTOR.
  A property line (ui.note, data key props) listing the properties that hold: symmetric ⇒ "real λ, perpendicular eigenvectors"; singular ⇒ "det = 0 ⇒ λ = 0 is an eigenvalue, rank …"; orthogonal ⇒ "|λ| = 1"; not diagonalizable; flips orientation.
  Eigenvector detection (key eigMsg): when x is within 2° of the LINE of an eigenvector (compare with both v and −v; for repeated-scalar every x qualifies), show "x is an eigenvector: Ax = λx with λ = …" and mark the angle tile ok. Remember Ax points opposite to x when λ < 0, and Ax = 0 when λ = 0 (then the angle between x and Ax is undefined: show "—").

STATE: { a, b, c, d, t, vx, vy, showGrid, showCircle, showEigen }

CHECK VALUES
  eig(2, 1, 1, 2): real-distinct, λ = 3, 1; v₁ = (0.70711, 0.70711), v₂ = (0.70711, −0.70711); trace 4, det 3
  eig(1, 1, 0, 1): repeated-defective, λ = 1, v = (1, 0)
  eig(0, −1, 1, 0): complex, λ = ±j
  eig(1, 2, 2, 4): λ = 5, 0; v₁ = (0.44721, 0.89443); v₂ = (0.89443, −0.44721); det 0; rank 1
  eig(4, 1, 2, 3): λ = 5, 2; v₁ = (0.70711, 0.70711); v₂ = (0.44721, −0.89443)
  eig(2, 0, 0, 2): repeated-scalar;  eig(0, 0, 0, 0): repeated-scalar, λ = 0, rank 0
  eig(3, 0, 0, 1): λ₁ = 3 with v (1, 0); λ₂ = 1 with v (0, 1)
  eig(1, 0, 0, 3): λ₁ = 3 with v (0, 1); λ₂ = 1 with v (1, 0)
  eig(1, 0, 2, 3): λ₁ = 3 with v (0, 1); λ₂ = 1 with v (0.70711, −0.70711)
  charEq(2, 1, 1, 2) = 'λ² − 4λ + 3 = 0'; charEq(0, −1, 1, 0) = 'λ² + 1 = 0'; charEq(1, 2, 2, 4) = 'λ² − 5λ = 0'; charEq(−1, 0, 0, −2) = 'λ² + 3λ + 2 = 0'
  Invariants over 1000 random matrices (entries on the 0.1 grid in [−3, 3], and also continuous random entries): for every returned real vector A·v = λ·v to 1e-9 and |v| = 1; λ₁ + λ₂ = trace; λ₁·λ₂ = det (complex: |λ|²); symmetric matrices with distinct eigenvalues give v₁·v₂ = 0; morph(…, 0) = I and morph(…, 1) = A.

HINTS (use these)
  1. Drag x until Ax lies along the same line as x: that direction is an eigenvector, and |Ax|/|x| is |λ|.
  2. Choose Shear: λ = 1 is repeated but there is only ONE eigenvector direction — the matrix is not diagonalizable.
  3. Choose Rotation 90°: every vector is turned, so there is no real eigenvector; λ = ±j.
  4. Choose Singular: the whole plane collapses onto a line (det = 0), and one eigenvalue is 0.
  5. Make b = c (symmetric): the two eigen-directions are always perpendicular.

STATES TO SCREENSHOT: default; Shear; Rotation 90°; Singular; default at t = 0.5; [[4,1],[2,3]] with x snapped to v₂; Reflection (det < 0, red parallelogram); 390-wide default.
````

#### 2.6.4 `rlc-transient`

````text
SIMULATION rlc-transient — title "RC / RL / RLC transient response"
SITS BESIDE: notes/src/02_mechatronics.html, section "Transient response" (under "1. Network Elements and Analysis"). Same symbols: τ, ω₀, ζ, ω_d.

PURPOSE
First-order circuits: x(t) = x(∞) + [x(0⁺) − x(∞)]·e^(−t/τ) with τ = RC or L/R (63.2 % at τ, more than 99 % at 5τ). Series RLC: s² + (R/L)s + 1/LC = 0 with ω₀ = 1/√(LC) and ζ = (R/2)√(C/L): over-, critically and under-damped responses and how the poles move. The capacitor voltage and the inductor current never jump.

CIRCUITS (all series, a DC source switched at t = 0)
  'RC': source – switch – R – C.     'RL': source – switch – R – L.     'RLC': source – switch – R – L – C.
MODES
  'charge': zero initial energy (v_C(0) = 0, i(0) = 0); the source steps from 0 to V at t = 0.
  'discharge': the circuit has been at DC steady state with the source at V; at t = 0 the source is replaced by a short circuit (steps from V to 0). By superposition every quantity q obeys q_discharge(t) = q_steady − q_charge(t), with the steady values under V: RC: v_C = V, i = 0, v_R = 0. RL: i = V/R, v_R = V, v_L = 0. RLC: v_C = V, i = 0, v_R = 0, v_L = 0.
FORMULAS (charge; i is the loop current, positive out of the source's + terminal)
  RC, τ = RC:   v_C = V(1 − e^(−t/τ)),  i = (V/R)·e^(−t/τ),  v_R = V·e^(−t/τ).
  RL, τ = L/R:  i = (V/R)(1 − e^(−t/τ)),  v_R = V(1 − e^(−t/τ)),  v_L = V·e^(−t/τ).
  RLC: α = R/(2L), ω₀ = 1/√(LC), ζ = α/ω₀ = (R/2)√(C/L), R_crit = 2√(L/C), Q = 1/(2ζ) = (1/R)√(L/C).
    kind: 'critical' if |ζ − 1| < 1e-6, else 'under' if ζ < 1, else 'over'.
    under:    ω_d = ω₀√(1 − ζ²);  v_C = V[1 − e^(−αt)(cos ω_d t + (α/ω_d) sin ω_d t)];  i = (V/(ω_d L))·e^(−αt)·sin ω_d t.
    critical: v_C = V[1 − (1 + αt)e^(−αt)];  i = (V/L)·t·e^(−αt).
    over:     s₁ = −α + √(α² − ω₀²) (nearer the origin), s₂ = −α − √(α² − ω₀²);  v_C = V[1 − (s₂e^(s₁t) − s₁e^(s₂t))/(s₂ − s₁)];  i = (V/L)(e^(s₁t) − e^(s₂t))/(s₁ − s₂).
    always:   v_R = i·R and v_L = V − v_R − v_C.
    poles: under: −α ± jω_d; critical: −α twice; over: s₁, s₂.
    under-damped peak of v_C: at t_p = π/ω_d, v_C,max = V(1 + e^(−απ/ω_d)).
  Time window tEnd: RC and RL: 6τ. RLC under: min(6/α, 20·(2π/ω_d)); critical: 8/ω₀; over: 6/|s₁|.

MODEL (SI units: V, Ω, H, F, s, A)
  firstOrder(circuit, R, L, C) -> { tau }
  rlc(R, L, C) -> { alpha, w0, f0, zeta, kind, wd (NaN unless under), s1: { re, im }, s2: { re, im }, Rcrit, Q }
  response(circuit, mode, p, t) -> { vC, vR, vL, i }     p = { V, R, L, C }; a quantity whose element is absent from the circuit is undefined (RC has no vL; RL has no vC)
  tEnd(circuit, p) -> seconds
  peak(p) -> { tp, vCmax, overshootPct } for the under-damped charge case, else null

CONTROLS
  Circuit: segmented RC / RL / Series RLC (key circuit; default RLC). Selecting a circuit loads its default values: RC: R = 1 kΩ, C = 100 µF. RL: R = 100 Ω, L = 100 mH. RLC: R = 20 Ω, L = 100 mH, C = 100 µF. Sliders of elements that are not in the circuit are hidden.
  Mode: segmented "Charge (source 0 → V)" / "Discharge (source V → 0)" (key mode; default charge).
  V slider 1–20 V, step 1, default 10 (key V).
  R: log slider 1 Ω … 10 kΩ, shown in Ω (key R).  L: log slider 1 … 1000 mH (key L).  C: log slider 0.1 … 1000 µF (key C).
  Button "Set R = R_crit" (RLC only): sets R = 2√(L/C) exactly (clamped to the slider range).
  Trace checkboxes (only those that exist for the circuit): v_C, v_R, v_L, i (keys showVC, showVR, showVL, showI). Defaults: RC: v_C and i. RL: i and v_L. RLC: v_C and i.
  Reset.

VIEWS
  (1) A small schematic canvas of the selected series circuit with standard symbols (battery, switch, zig-zag resistor, coil inductor, parallel-plate capacitor), each labelled with its value in engineering units (Ω or kΩ, mH or H, µF, V), plus an arrow for i. Simple and tidy.
  (2) A plot canvas with two stacked plots sharing the time axis: top = voltages in V (the visible voltage traces; the source level V as a faint dashed line); bottom = current with an automatic unit (A or mA). Time from 0 to tEnd with an automatic unit (µs, ms or s) and nice ticks. First-order circuits: vertical dashed markers at τ, 2τ, 3τ, 4τ, 5τ (labelled) and a marker dot on the main trace at t = τ with its label (63.2 % or 36.8 % level). Under-damped RLC with v_C shown in charge mode: the envelope V(1 ± e^(−αt)) dashed, and the first-peak marker with its value. A hover cursor (cv.onHover) draws a vertical line and writes the values of all visible traces at that time in a read-out line under the plot (data key cursor).
  (3) RLC only: a small s-plane pole map (equal axes) with the two poles as ×, axes labelled σ and jω, a dashed circle of radius ω₀, and the labels −α and ±ω_d. It must show clearly how the poles travel along that circle as R grows, meet on the real axis and then split.
  Colours: v_C blue, v_R green, v_L purple, i orange (HTML legend).

READ-OUTS
  RC and RL: τ [tau] (automatic unit); 5τ [fiveTau]; the level at τ [atTau], e.g. "v_C(τ) = 6.321 V (63.2 %)"; initial value [x0] and final value [xInf] of the main quantity (v_C for RC, i for RL).
  RLC: ω₀ [w0] in rad/s and f₀ [f0] in Hz; α [alpha]; ζ [zeta]; ω_d [wd]; R_crit [Rcrit]; Q [Q]; poles [poles] ("−100 ± j300" for a pair, or the two real values); peak v_C and overshoot [peak] (under-damped charge only, otherwise "—").
  Tag [key kind]: UNDERDAMPED (warn) / CRITICALLY DAMPED (ok) / OVERDAMPED for RLC; FIRST ORDER for RC and RL.

STATE: { circuit ('RC' | 'RL' | 'RLC'), mode ('charge' | 'discharge'), V, R (Ω), LmH, CuF, showVC, showVR, showVL, showI }   (setState with a circuit and explicit R / LmH / CuF applies the circuit first, then the values)

CHECK VALUES
  RC, R = 1000, C = 100e-6, V = 10, charge: τ = 0.1 s; v_C(0.1) = 6.3212; v_C(0.5) = 9.9326; i(0) = 0.010 A. Discharge: v_C(0.1) = 3.6788; i(0⁺) = −0.010 A.
  RL, R = 100, L = 0.1, V = 10, charge: τ = 1 ms; i(0.001) = 0.063212 A; i(∞) = 0.1 A; v_L(0) = 10 V. Discharge: i(0.001) = 0.036788 A; v_L(0⁺) = −10 V.
  RLC, R = 20, L = 0.1, C = 100e-6, V = 10: α = 100, ω₀ = 316.228, ζ = 0.31623, ω_d = 300 exactly, poles −100 ± j300, R_crit = 63.2456, Q = 1.5811; peak at t_p = 0.010472 s with v_C,max = 13.5092 V (35.09 % overshoot); the current maximum is 0.20854 A at t = 0.004163 s.
  RLC, R = 200 (same L, C): over, ζ = 3.1623, s₁ = −51.317, s₂ = −1948.683.
  RLC, R = 63.2455532 (critical; α = 316.228): v_C(1/α) = 2.6424 V.
  Invariants (300 random parameter sets over the slider ranges × several times): charge: v_C(0) = 0, i(0) = 0 for RLC, v_C → V; KVL: v_R + v_L + v_C = V in charge mode and = 0 in discharge mode (missing elements count as 0) to 1e-9·V; i = C·dv_C/dt and v_L = L·di/dt by central finite differences (relative 1e-4); continuity across the damping classes (ζ = 1 ± 1e-5 give the same v_C to 1e-4·V); discharge = steady − charge.

HINTS (use these)
  1. RC: at t = τ the capacitor has reached 63.2 % of V; after 5τ it is above 99 %. Double R or C and τ doubles.
  2. Series RLC: raise R until the oscillation just disappears — that is R = 2√(L/C) (ζ = 1), the fastest response with no overshoot. Press "Set R = R_crit".
  3. Watch the pole map: a complex pair (oscillation at ω_d) moves along the circle of radius ω₀, meets on the real axis at ζ = 1, then splits into two real poles.
  4. The capacitor voltage and the inductor current start exactly where they were: they cannot jump at t = 0. v_R, v_L and (for RC) i can.

STATES TO SCREENSHOT: RLC default; RLC critical; RLC over-damped (R = 200); RC default in charge mode; RL in discharge mode; RLC with all four traces on; 390-wide RLC default.
````

#### 2.6.5 `second-order-step`

````text
SIMULATION second-order-step — title "Second-order step response and pole location"
SITS BESIDE: notes/src/04_electrical.html, section "Transient response" (under "B1.3 Control Systems"). Same symbols: ζ, ωₙ, ω_d, t_p, M_p, t_s, t_r, θ = cos⁻¹ζ.

PURPOSE
For G(s) = ωₙ²/(s² + 2ζωₙs + ωₙ²) the student links the unit-step response to the pole positions: ζ alone fixes the overshoot (and the angle of the poles); ωₙ only rescales time (distance of the poles from the origin); the real part −ζωₙ sets the decay (settling) and the imaginary part ω_d sets the oscillation (peak time).

FORMULAS (unit step, zero initial conditions)
  ζ = 0:                 y = 1 − cos ωₙt
  0 < ζ < 1:             ω_d = ωₙ√(1 − ζ²), θ = acos ζ;  y = 1 − e^(−ζωₙt)·sin(ω_d t + θ)/√(1 − ζ²)
  |ζ − 1| < 1e-6:        y = 1 − (1 + ωₙt)·e^(−ωₙt)
  ζ > 1:                 s₁ = −ωₙ(ζ − √(ζ² − 1)) (slow pole), s₂ = −ωₙ(ζ + √(ζ² − 1));  y = 1 − (s₂e^(s₁t) − s₁e^(s₂t))/(s₂ − s₁)
  Poles: −ζωₙ ± jω_d (under-damped); ±jωₙ (ζ = 0); −ωₙ twice (ζ = 1); s₁, s₂ (over-damped).
  Under-damped specifications (0 < ζ < 1): rise time (0 → 100 %) t_r = (π − θ)/ω_d; peak time t_p = π/ω_d; overshoot M_p = e^(−πζ/√(1−ζ²)); settling time t_s ≈ 4/(ζωₙ) for the 2 % band and 3/(ζωₙ) for the 5 % band; oscillations before settling N ≈ t_s·ω_d/(2π).
  Frequency-domain companions (secondary read-outs): resonant peak M_r = 1/(2ζ√(1−ζ²)) at ω_r = ωₙ√(1−2ζ²) (only for ζ < 0.7071); bandwidth ω_b = ωₙ√[(1 − 2ζ²) + √(4ζ⁴ − 4ζ² + 2)].
  Numerical quantities for every ζ: the ACTUAL settling time = the last time |y − 1| exceeds the band (∞ for ζ = 0); the 10–90 % rise time.
  Time window: tEnd = niceCeil(clamp(6/σ, 8/ωₙ, 40/ωₙ)) with σ = ζωₙ for ζ ≤ 1 and σ = |s₁| for ζ > 1 (σ = 0 means use 40/ωₙ).

MODEL
  classify(zeta) -> 'undamped' | 'under' | 'critical' | 'over'
  poles(zeta, wn) -> [{ re, im }, { re, im }]
  step(zeta, wn, t) -> y
  specs(zeta, wn) -> { kind, wd, thetaDeg, tr, tp, Mp (fraction), ts2, ts5, N2 (oscillations to settle within 2 %), Mr, wr, wb }     fields that do not exist for the case are NaN; for ζ = 0: Mp = 1 and ts2 = ts5 = Infinity
  settleActual(zeta, wn, band) -> seconds (band = 0.02 or 0.05; Infinity for ζ = 0)
  rise1090(zeta, wn) -> seconds
  tEnd(zeta, wn) -> seconds
  fromPole(re, im) -> { zeta, wn } for a pole with re ≤ 0 and im ≥ 0: wn = hypot(re, im), zeta = −re/wn

CONTROLS
  ζ slider 0…2.5, step 0.01, default 0.5 (key zeta).  ωₙ slider 0.5…10 rad/s, step 0.1, default 2 (key wn).
  Settling band: segmented 2 % / 5 % (key band; default 2).
  Checkboxes: "Envelope" (default on, key showEnvelope), "Spec markers" (default on, key showMarkers).
  A row of preset buttons for ζ: 0, 0.2, 0.5, 0.707, 1, 2 (they change ζ only).
  Buttons: "Hold curve" (freezes the current response as a grey reference curve in real time units so later curves can be compared with it; pressing again replaces it), "Clear held", "Reset".
  Drag in the s-plane: the upper pole × is draggable; dragging sets (ζ, ωₙ) = fromPole of the pointer position clamped to re ≤ 0, im ≥ 0 and to the slider ranges. On the negative real axis (im = 0) it means ζ = 1 with ωₙ = |re|.

VIEWS (step plot in the main column, the s-plane beside or below it; controls and stats in the side column)
  (1) Step-response plot: y(t) for t ∈ [0, tEnd]; the y-axis upper limit is the smallest of {1.3, 1.8, 2.1} that leaves headroom above 1 + M_p. Draw: the reference line y = 1; the settling band as a shaded strip; the held curve (grey); the response (blue, width 2.5); the envelope 1 ± e^(−ζωₙt)/√(1−ζ²) (dashed orange, under-damped only, when enabled); when markers are enabled and the case is under-damped: a dot at (t_p, 1 + M_p) with drop lines, "t_p" on the time axis and "M_p = 16.3 %" beside the dot; a vertical marker at t_r labelled "t_r"; a vertical marker at the FORMULA settling time labelled "t_s ≈ 4/(ζωₙ)" (or 3/(ζωₙ)); a small triangle on the time axis at the ACTUAL settling time. For ζ ≥ 1 show only the actual-settling marker and the 10 % / 90 % rise markers.
  (2) s-plane (equal axes): window about re ∈ [−W, 0.3W] with W = niceCeil(1.25·max(ωₙ, |s₂| when over-damped)). The right half-plane lightly tinted red and labelled "unstable"; axes labelled σ and jω; the two poles as ×; for the under-damped case: the radius from the origin to the upper pole labelled ωₙ, the angle arc θ between the NEGATIVE real axis and that radius labelled "θ = cos⁻¹ζ = 60°", dashed projections onto the axes labelled "−ζωₙ" and "ω_d"; a dashed semicircle of radius ωₙ (constant ωₙ) and a dashed ray at the current ζ (constant ζ).
  Legend under the plot.

READ-OUTS
  Tag [key kind]: UNDAMPED (bad) / UNDERDAMPED (warn) / CRITICALLY DAMPED (ok) / OVERDAMPED.
  Stats: poles [poles] (e.g. "−1 ± j1.732"); ω_d [wd] in rad/s; θ [theta] in °; rise time [tr] (the 0–100 % formula when under-damped; "—" otherwise); peak time [tp]; overshoot [Mp] in %; settling time by formula [ts] for the chosen band; actual settling time [tsActual]; oscillations [N]; 10–90 % rise time [tr1090]; M_r [Mr] and ω_r [wr] ("—" for ζ ≥ 0.7071); ω_b [wb].
  A transfer-function line (data key tf): "G(s) = 4 / (s² + 2s + 4)" with the current numbers.

STATE: { zeta, wn, band ('2' | '5'), showEnvelope, showMarkers, held (boolean: getState reports whether a curve is held; setState({ held: true }) holds the current curve, false clears it) }

CHECK VALUES
  ζ = 0.5, ωₙ = 2: poles −1 ± j1.7321; ω_d = 1.7321; θ = 60°; t_r = 1.2092; t_p = 1.8138; M_p = 16.303 %; t_s = 4 s (2 %) and 3 s (5 %); step(t_p) = 1.16303; actual settling 4.038 s (2 %) and 2.645 s (5 %); 10–90 % rise 0.8188 s; M_r = 1.1547; ω_r = 1.4142; ω_b = 2.5440; tEnd = 10.
  ζ = 0.70711, ωₙ = 1: M_p = 4.3214 %; actual 2 % settling 5.979 s (for ζ = 0.7 exactly).
  ζ = 1, ωₙ = 1: step(1) = 0.26424; actual 2 % settling 5.834 s; 10–90 % rise 3.358 s.
  ζ = 2, ωₙ = 1: poles −0.26795 and −3.73205; step(1) = 0.17774; actual 2 % settling 14.88 s; 10–90 % rise 8.229 s.
  ζ = 0, ωₙ = 2: step(π/2) = 2; M_p = 100 %; settling ∞.
  ζ = 0.2, ωₙ = 5: actual 2 % settling 3.920 s; 10–90 % rise 0.2407 s.
  fromPole(−1, 1.7320508) -> ζ = 0.5, ωₙ = 2.
  Invariants: step() agrees with RASim.math.tfStep([wn²], [1, 2ζwn, wn²], …) to 1e-8 for 200 random (ζ, ωₙ) over the slider ranges, including ζ = 0, ζ = 1 and values within 1e-7 of 1; step(0) = 0; continuity across ζ = 1; time scaling step(ζ, ωₙ, t) = step(ζ, 1, ωₙt); the maximum of the finely sampled under-damped response equals 1 + M_p and occurs at t_p; settleActual and rise1090 agree with a brute-force scan.

HINTS (use these)
  1. Keep ζ fixed and change ωₙ: the overshoot does not change — only the time scale does. In the s-plane the poles slide along a ray through the origin.
  2. Keep ωₙ fixed and change ζ: the poles move on a circle of radius ωₙ, and the overshoot depends only on ζ.
  3. ζ ≈ 0.7 gives about 4.3 % overshoot — the usual compromise between speed and overshoot.
  4. ζ = 0 puts the poles on the jω-axis (sustained oscillation); for ζ ≥ 1 both poles are real and there is no overshoot.
  5. Press Hold curve, then drag the pole: compare the old and new responses.

STATES TO SCREENSHOT: default; ζ = 0; ζ = 1; ζ = 2; ζ = 0.2 with ωₙ = 5 and a held curve from the default; 5 % band; 390-wide default.
````

#### 2.6.6 `root-locus`

````text
SIMULATION root-locus — title "Root locus explorer"
SITS BESIDE: notes/src/04_electrical.html, section "Root locus" (under "B1.3 Control Systems"). The rules listed there are exactly what this simulation must make visible.

PURPOSE
For a unity negative-feedback loop with open-loop K·G(s), G(s) = Π(s − zᵢ)/Π(s − pⱼ), the closed-loop poles are the roots of D(s) + K·N(s) = 0. The student sees the branches start at the open-loop poles (K = 0) and end at the zeros or at infinity along asymptotes, the real-axis rule, break-away and break-in points, jω-axis crossings (the marginal K that the Routh array gives), departure angles, and how adding a pole or a zero bends the locus.

DEFINITIONS
  n poles, m zeros, 1 ≤ n ≤ 6, m ≤ n. Complex poles and zeros come in conjugate pairs. N(s) and D(s) are the monic polynomials built from the zeros and poles (RASim.math.polyFromRoots). K ≥ 0.
  Closed-loop poles for a given K: the roots of D + K·N (RASim.math.polyRoots). Mind the core's caveat: a repeated real root can come back as a near-real pair — classify with a tolerance.
  Asymptotes (when n > m): n − m of them at angles (2q + 1)·180°/(n − m), q = 0 … n − m − 1 (reported wrapped to (−180°, 180°] and sorted ascending), meeting at the centroid σ_a = (Σ Re p − Σ Re z)/(n − m).
  Break-away / break-in points: real solutions s of N(s)·D'(s) − N'(s)·D(s) = 0 for which K = −D(s)/N(s) is real and > 1e-9. (Stationary points with negative K are NOT on the locus and must not be reported.) Complex solutions with a real positive K (|Im K| < 1e-6·|K|) are also valid break points.
  jω-axis crossings: gains K > 0 at which a closed-loop pole has zero real part (open-loop poles sitting on the axis at K = 0 do not count). Find them robustly: on the axis K(ω) = −D(jω)/N(jω) must be real and positive — scan ω > 0 for sign changes of Im K(ω) (log-spaced samples, bisection), keep those with Re K > 0, and treat ω = 0 separately (a real pole passing through the origin: D(0) + K·N(0) = 0 with K > 0).
  Stable K ranges: the set of K > 0 for which every closed-loop pole has Re < 0. Use the crossing gains as interval boundaries and test one K inside each interval; the last interval is open to ∞.
  Departure angle from a complex pole p_k: θ_d = 180° − Σ_(j≠k) ∠(p_k − p_j) + Σ_i ∠(p_k − z_i). Arrival angle at a complex zero z_k: θ_a = 180° + Σ_j ∠(z_k − p_j) − Σ_(i≠k) ∠(z_k − z_i). Both wrapped to (−180°, 180°].
  Locus curves: n branches as functions of K from 0 to a large K. Compute them by stepping K on a logarithmic grid (start with K = 0), solving for the roots, and matching every root to the nearest root of the previous step (resolve conflicts by a best assignment, not greedily in index order). Adapt the K step so that no root moves by more than a small distance per step (about 3 screen pixels, or 1/200 of the view) — essential near break points where roots move fast. Go far enough in K that branches heading to infinity leave the view, and branches ending at zeros visibly reach them.

MODEL (roots are { re, im }; pole and zero lists contain EVERY root, i.e. both members of each conjugate pair)
  polys(poles, zeros) -> { num, den }
  closedLoopPoles(poles, zeros, K) -> roots
  asymptotes(poles, zeros) -> { count, anglesDeg, centroid }     (count 0 -> anglesDeg [] and centroid NaN)
  breakPoints(poles, zeros) -> [{ s: { re, im }, K }] sorted by descending re
  jwCrossings(poles, zeros) -> [{ K, w }] sorted by K (w ≥ 0)
  stableRanges(poles, zeros) -> [[Klow, Khigh], …]     (Khigh may be Infinity; [] when never stable)
  departureAngles(poles, zeros) -> [{ at: { re, im }, deg }] for the poles with im > 0;  arrivalAngles(poles, zeros) likewise for the zeros with im > 0
  locus(poles, zeros, opts) -> array of n branches, each an array of { re, im, K } with non-decreasing K     (opts: fields of your choice such as kMax and maxMove)
  kAt(poles, zeros, s) -> |D(s)| / |N(s)|     (the magnitude condition)
  dominant(roots) -> { zeta, wn } of the closed-loop pole (or pair) closest to the jω-axis, or null when there is none in the open left half-plane

CONTROLS
  Preset select (key preset): "1 / [s(s+2)]" · "1 / [s(s+1)(s+2)]" (default) · "(s+3) / [s(s+2)]" · "1 / [s(s²+4s+8)]" · "(s+1) / s²" · "1 / [(s+1)(s+2)(s+3)(s+4)]" · "(s²+2s+2) / [s(s+1)(s+2)]".
  Tool: segmented "Move" / "Add pole ×" / "Add zero ○" / "Delete" (key tool; default Move).
     Add: clicking in the s-plane adds a real root when the click is within 8 px of the real axis, otherwise a conjugate pair. Limits: at most 6 poles; zeros ≤ poles. A refused action explains itself in the status line (data key status).
     Move: drag a pole or zero; the conjugate mirrors it. A real root stays real (it slides along the axis); a complex pair stays a pair with |Im| ≥ 0.1.
     Delete: click a root to remove it (a pair goes together); refuse when it would leave no pole or more zeros than poles.
     All positions snap to a 0.1 grid.
  K: log slider 0.01 … 1000, default 1 (key K).
  With the Move tool, clicking on the locus (not on a root) within 10 px of a computed locus point sets K to that point's K — "pick a point, read its gain". Hovering near the locus shows s and K = |D|/|N| for the nearest locus point in a read-out line (data key hover).
  Checkboxes: "Asymptotes" (on, key showAsymptotes), "Break points" (on, key showBreaks), "Departure / arrival angles" (on, key showAngles), "Step response" (on, key showStep).
  Reset (the default preset with K = 1).

VIEWS
  (1) s-plane canvas with equal axes. The window is fitted to all poles, zeros, the centroid, break points and jω crossings with a 35 % margin (at least ±2 around them), symmetric about the real axis, and is recomputed only when the pole/zero set changes — never when K changes. The right half-plane is tinted light red with the word "unstable". Draw: grid and axes (σ, jω); asymptotes (dashed grey rays from the centroid; the centroid as a small + labelled "σ_a = …"); the locus branches (blue, width 2, with a few small arrow heads pointing towards increasing K); open-loop poles × (red) and zeros ○ (blue); break points (small purple diamonds labelled like "s = −0.423, K = 0.385"); jω crossings (gold dots labelled like "K = 6, ω = 1.414"); departure / arrival arrows with labels like "−45°"; the closed-loop poles for the current K as orange filled squares, clearly larger than the locus line. Offset labels so they do not pile on top of each other in the presets.
  (2) Closed-loop unit-step response for the current K: T(s) = K·N/(D + K·N) via RASim.math.tfStep. Time window: when stable, clamp(6/σ_min, 2, 60) seconds with σ_min the smallest |Re| of the closed-loop poles; otherwise clamp(5·2π/ω_max, 2, 30) with ω_max the largest |Im| among the poles with Re ≥ 0 (10 s when that is 0). y-axis fitted to the data but never beyond [−1, 3] (clip the curve). A dashed line at the final value T(0) when stable.
  Legend.

READ-OUTS
  A transfer-function line (data key tf), e.g. "G(s) = (s + 3) / (s² + 2s)" via RASim.fmtPoly, and the characteristic equation with the current K (data key charEq), e.g. "s³ + 3s² + 2s + 1 = 0".
  Tag [key stability]: STABLE (ok) / MARGINALLY STABLE (warn; the largest real part is within 1e-6 of zero relative to the pole magnitudes) / UNSTABLE (bad).
  Stats: number of poles [n]; number of zeros [m]; asymptotes [asym] as "3 at −60°, 60°, 180°" or "none"; centroid [centroid]; break points [breaks] as a list like "−0.423 (K = 0.385)" or "none"; jω crossings [crossings] like "±j1.414 at K = 6" or "none"; stable range [stable] ("0 < K < 6", "all K > 0", "K > 2.5", "never", several intervals joined by "and"); closed-loop poles [clPoles] (fmtComplex list); dominant ζ [zeta] and ωₙ [wn] or "—".

STATE: { poles: [{ re, im } …], zeros: [{ re, im } …], K, tool, showAsymptotes, showBreaks, showAngles, showStep }     (setState with poles / zeros replaces the configuration)

CHECK VALUES
  G = 1/[s(s+1)(s+2)]: asymptotes 3 at −60°, 60°, 180°; centroid −1; exactly one break point: s = −0.42265 with K = 0.38490 (the other stationary point, −1.57735, has negative K and must NOT be reported); jω crossing K = 6, ω = 1.41421; stable for 0 < K < 6; closed-loop poles at K = 6: ±j1.41421 and −3.
  G = 1/[s(s+2)]: break-away s = −1 with K = 1; asymptotes −90°, 90°; centroid −1; no crossing; stable for all K > 0; K = 5 gives poles −1 ± j2 (dominant ζ = 0.44721, ωₙ = 2.23607).
  G = (s+3)/[s(s+2)]: break-away −1.26795 (K = 0.53590) and break-in −4.73205 (K = 7.46410); one asymptote at 180°; stable for all K > 0; every complex locus point lies on the circle |s + 3| = 1.73205 (test to 1e-3).
  G = 1/[s(s²+4s+8)]: asymptotes −60°, 60°, 180°; centroid −1.33333; departure angle from −2 + j2 is −45°; crossing K = 32, ω = 2.82843; stable for 0 < K < 32; no break points.
  G = (s+1)/s²: break-in at s = −2 with K = 4; stable for all K > 0; K = 2 gives −1 ± j; every complex locus point lies on the circle |s + 1| = 1.
  G = 1/[(s+1)(s+2)(s+3)(s+4)]: asymptotes −135°, −45°, 45°, 135°; centroid −2.5; derive the marginal K and ω yourself from the Routh array of s⁴ + 10s³ + 35s² + 50s + 24 + K = 0 and assert the model against your derivation.
  Invariants: every locus sample satisfies D(s) + K·N(s) ≈ 0 (relative 1e-6); branches are continuous and never swap at break points (test with 1/[s(s+1)(s+2)] and (s+3)/[s(s+2)]: after the break-away the two branches are complex conjugates of each other all the way); the number of branches equals n; each branch starts at an open-loop pole; kAt at a locus point returns its K (1e-4 relative); stableRanges agrees with a brute-force scan of the largest real part over a fine K grid for all presets; asymptote angles and centroid for random pole/zero sets.

HINTS (use these)
  1. Preset 1/[s(s+1)(s+2)]: raise K past 6 — two poles cross the jω-axis at ±j1.414 and the step response grows without bound. The Routh array gives the same K.
  2. Start from 1/[s(s+2)] and add a zero at −3: the locus bends to the left into a circle — a zero attracts the locus and improves damping.
  3. Add a third pole instead: the branches bend to the right — a pole pushes the locus towards instability.
  4. Count: the number of branches equals the number of poles; n − m of them leave along asymptotes from the centroid.
  5. Click on the locus to place the closed-loop poles there and read the gain K from the magnitude condition.

PERFORMANCE: dragging a pole must stay smooth: during a drag compute a coarser locus (and skip the step response if necessary), then compute the full-quality locus on release. Changing K must not recompute the locus at all.

STATES TO SCREENSHOT: default preset at K = 1; the same at K = 8 (unstable); (s+3)/[s(s+2)] at K = 2; 1/[s(s²+4s+8)] at K = 10 showing the departure angle; (s+1)/s² at K = 4; the four-pole preset; 390-wide default.
````

#### 2.6.7 `bode-plot`

````text
SIMULATION bode-plot — title "Bode plot builder with gain and phase margins"
SITS BESIDE: notes/src/04_electrical.html, section "Bode plots" (and the gain-margin / phase-margin definitions in "Nyquist criterion" just above it).

PURPOSE
Build an open-loop transfer function from standard factors and see how each factor shapes the magnitude (±20 dB/decade per real pole or zero, corners) and the phase; compare the straight-line (asymptotic) plot with the exact one; read the gain margin and phase margin and connect them with closed-loop stability.

DEFINITION OF THE SYSTEM (Bode / time-constant form)
  G(s) = K · s^(−N) · (product of factors) · e^(−sT)
  K > 0.  N ∈ {−1, 0, 1, 2}: N = 1 is one integrator 1/s; N = 2 is 1/s²; N = −1 is one differentiator s; N = 0 none.
  Each factor is one of:
     'pole'      1/(1 + s/ω)                    real pole at −ω
     'zero'      (1 + s/ω)                      real zero at −ω
     'rhpzero'   (1 − s/ω)                      right-half-plane zero at +ω (non-minimum phase)
     'quad'      1/(1 + 2ζ(s/ωₙ) + (s/ωₙ)²)     complex pole pair, 0 < ζ ≤ 1
  T ≥ 0 is a transport delay.
  sys = { K, N, factors: [{ type, w, zeta }, …], delay }     (entries with type 'off' are ignored)
EXACT RESPONSE at s = jω — compute magnitude and phase factor by factor so the phase is continuous (never atan2 of the whole product):
  magnitude in dB = 20·log₁₀K − 20N·log₁₀ω + Σ: pole −10·log₁₀(1 + (ω/w)²); zero and rhpzero +10·log₁₀(1 + (ω/w)²); quad −10·log₁₀[(1 − u²)² + (2ζu)²] with u = ω/ωₙ; delay 0.
  phase in degrees = −90N + Σ: pole −atan(ω/w); zero +atan(ω/w); rhpzero −atan(ω/w); quad −atan2(2ζu, 1 − u²) (running continuously from 0 to −180°); delay −ωT·180/π.
ASYMPTOTIC (straight-line) MAGNITUDE: 20·log₁₀K − 20N·log₁₀ω + Σ: pole: 0 for ω ≤ w, −20·log₁₀(ω/w) above; zero and rhpzero: 0 for ω ≤ w, +20·log₁₀(ω/w) above; quad: 0 for ω ≤ ωₙ, −40·log₁₀(ω/ωₙ) above.
ASYMPTOTIC PHASE: −90N + Σ: pole: 0 for ω ≤ w/10, −45°·log₁₀(ω/(w/10)) between w/10 and 10w, −90° above; zero: the mirror image (positive); rhpzero: the same as a pole; quad: 0 for ω ≤ ωₙ·10^(−ζ), −180° for ω ≥ ωₙ·10^(ζ), linear in log ω in between (slope −90/ζ degrees per decade); delay: the exact −ωT (it has no straight-line form).
MARGINS
  Gain crossovers ω_gc: |G(jω)| = 1 (0 dB). Phase margin at a crossover = 180° + phase(ω_gc). Phase crossovers ω_pc: the continuous phase equals −180° − 360°·k for an integer k ≥ 0. Gain margin at a phase crossover = −(magnitude in dB at ω_pc).
  Search ω from 1e-4 to 1e6 rad/s (log-spaced, at least 2000 samples, each crossing refined by bisection).
  Reported PM (pmDeg, at wgc) = the smallest phase margin over all gain crossovers. With no gain crossover: Infinity when the magnitude is below 0 dB everywhere, NaN when it is above 0 dB everywhere. Reported GM (gmDb, at wpc) = the smallest gain margin over all phase crossovers; Infinity with no phase crossover.
  Closed-loop stability for unity negative feedback: with delay = 0 decide exactly from the roots of the characteristic polynomial D(s) + N(s) (all Re < 0 -> true); with delay > 0 return null and let the UI say "from margins" (stable when GM > 0 dB and PM > 0°).
DERIVED FACTS
  low-frequency slope = −20N dB/decade; high-frequency slope = −20·(N + #pole + 2·#quad − #zero − #rhpzero) dB/decade; system type = max(N, 0); static error constant: N = 0: K_p = K; N = 1: K_v = K; N = 2: K_a = K; N = −1: K_p = 0.
  For each quad with ζ < 0.7071: resonant peak M_r = 1/(2ζ√(1 − ζ²)) (also in dB) at ω_r = ωₙ√(1 − 2ζ²).

MODEL
  response(sys, w) -> { magDb, phaseDeg }
  asymMagDb(sys, w) -> dB;   asymPhaseDeg(sys, w) -> degrees
  factorResponse(factor, w) -> { magDb, phaseDeg } for one factor alone; also accepts { type: 'gain', K }, { type: 'integrator', N } and { type: 'delay', T }
  corners(sys) -> ascending array of the corner frequencies of the active factors
  margins(sys) -> { gainCrossovers: [w …], phaseCrossovers: [w …], pmDeg, wgc, gmDb, wpc }
  polys(sys) -> { num, den } polynomial arrays in s for the delay-free part (expand the Bode-form factors; N < 0 puts s in the numerator)
  closedLoopStable(sys) -> true | false | null
  slopes(sys) -> { low, high } in dB/decade
  resonance(sys) -> [{ Mr, MrDb, wr }] for the quads with ζ < 0.7071
  freqRange(sys) -> { wmin, wmax }: wmin = 10^(floor(log₁₀(smallest corner)) − 1), wmax = 10^(ceil(log₁₀(largest corner)) + 1); with no corners 0.01 … 100; then extend by whole decades so that wgc and wpc (when finite) lie at least one decade inside; finally clamp to [1e-3, 1e5].

CONTROLS
  Preset select (key preset):
    A. "1 / [s(1+s)(1+s/2)] — type 1, three poles" (default): K = 1, N = 1, poles at 1 and 2
    B. "100 / [(1+s)(1+s/10)(1+s/100)] — type 0": K = 100, N = 0, poles at 1, 10, 100
    C. "Resonant pair: ζ = 0.2, ωₙ = 10": K = 1, N = 0, one quad
    D. "Lead compensator (1+s)/(1+s/10)": zero at 1, pole at 10
    E. "Lag compensator (1+s/10)/(1+s)": zero at 10, pole at 1
    F. "Non-minimum phase (1−s/10)/(1+s/10)": rhpzero at 10, pole at 10
    G. "Delay e^(−0.5s)/(1+s)": pole at 1, delay 0.5
  K: log slider 0.01 … 1000 (key K).
  Type: segmented "s" / "none" / "1/s" / "1/s²" (key N; values '-1', '0', '1', '2').
  Four factor slots, each a row: a type select (Off / Real pole / Real zero / RHP zero / Complex pole pair; keys slot1type … slot4type), a corner-frequency log slider 0.01 … 1000 rad/s (keys slot1w … slot4w), and — only for a complex pair — a ζ slider 0.05 … 1, step 0.01 (keys slot1zeta … slot4zeta). "Off" hides that slot's sliders.
  Delay T: slider 0 … 2 s, step 0.01 (key delay).
  Checkboxes: "Exact" (on, key showExact), "Asymptotes" (on, key showAsym), "Individual factors" (off, key showFactors: thin dashed lines for each factor's own magnitude and phase), "Margins" (on, key showMargins).
  Reset (preset A).

VIEWS
  One canvas with two stacked semi-log plots sharing the ω axis (freqRange):
   Top: magnitude in dB. y-range fitted to the exact and asymptotic curves, rounded outward to multiples of 20 dB, at least 80 dB tall. Exact curve solid blue (width 2.5); asymptote dashed maroon with a small dot at every corner and the slope written on each straight segment ("−20", "−40", … in dB/dec; skip the label on segments shorter than about 60 px); the 0 dB line emphasised.
   Bottom: phase in degrees. y-range rounded outward to multiples of 45° (90° steps when the span exceeds 360°), lower limit never below −720° (clip). Exact solid blue; asymptotic dashed maroon; the −180° line emphasised.
   Margins when enabled: a vertical dashed line at ω_gc through both plots and, on the phase plot, a double-headed arrow between −180° and the phase curve at ω_gc labelled "PM = 32.6°" (green when positive, red when negative); a vertical dashed line at ω_pc and, on the magnitude plot, a double-headed arrow between the curve and 0 dB at ω_pc labelled "GM = 9.5 dB" (green / red).
   A hover cursor (cv.onHover): a vertical line and a read-out line (data key cursor) with ω, |G| in dB and the phase.
  Legend.

READ-OUTS
  A transfer-function line (data key tf) in Bode form built from the active parts, e.g. "G(s) = 1 / [ s (1 + s/1) (1 + s/2) ]"; include K when it is not 1 and e^(−Ts) when T > 0.
  Tag [key stability]: CLOSED LOOP STABLE (ok) / CLOSED LOOP UNSTABLE (bad) / MARGINAL (warn, when a closed-loop pole lies within 1e-6 of the axis), with "(from margins)" appended when a delay is present.
  Stats: type [type]; low-frequency slope [slopeLow] and high-frequency slope [slopeHigh] in dB/dec; ω_gc [wgc]; PM [pm] (ok when > 0, bad when < 0); ω_pc [wpc]; GM [gm] in dB with the ratio in brackets (ok / bad); error constant [errConst] such as "K_v = 1"; resonant peak [Mr] such as "8.14 dB at 9.59 rad/s" or "—".

STATE: { K, N (number −1 … 2), factors: [{ type: 'off' | 'pole' | 'zero' | 'rhpzero' | 'quad', w, zeta } — exactly 4 entries], delay, showExact, showAsym, showFactors, showMargins }

CHECK VALUES
  One pole at w = 10 (K = 1, N = 0): at ω = 10: −3.0103 dB and −45°; at ω = 100: −20.043 dB and −84.289°; asymptotic magnitude at 100: −20 dB; asymptotic phase: 0° at ω = 1, −45° at 10, −90° at 100.
  Preset A: ω_pc = 1.41421, GM = 9.542 dB (ratio 3); ω_gc = 0.7494, PM = 32.61°; closed loop stable; slopes −20 and −60 dB/dec. With K = 3: GM = 0 dB (marginal). With K = 10: unstable, GM = −10.46 dB.
  Preset D (lead): the maximum phase is +54.903° at ω = 3.1623, where the magnitude is exactly +10.000 dB; the high-frequency gain is +20 dB.
  Preset C: at ω = 10: 7.959 dB and −90°; the peak is 8.136 dB at ω = 9.5917.
  Preset F: 0 dB at every ω; phase −90° at ω = 10, tending to −180°.
  Preset G: at ω = 2: −6.990 dB and −120.731°.
  Preset B: derive ω_gc, PM, ω_pc and GM independently in the test (brute force on a dense grid) and assert the model against them.
  Invariants: multiplying K by 10 raises the magnitude by exactly 20 dB at every ω and leaves the phase unchanged; response() equals 20·log₁₀|G(jω)| and matches the phase of G(jω) modulo 360° when G(jω) is computed with complex arithmetic from polys() and the delay (200 random systems × 20 frequencies); at an isolated real corner the exact magnitude differs from the asymptote by 3.0103 dB; for delay-free systems the asymptotes equal the exact curves in the limits ω → 0 and ω → ∞; margins() agrees with a brute-force dense scan (1e-3 relative) for all presets; for presets A and B at several K, closedLoopStable agrees with (GM > 0 and PM > 0).

HINTS (use these)
  1. Each real pole bends the magnitude asymptote down by 20 dB/decade at its corner and adds −90° of phase in the end; each zero does the opposite.
  2. Raise K: the magnitude curve slides up while the phase curve does not move — so ω_gc moves right and the phase margin shrinks. In preset A the loop goes unstable when K passes 3 (GM = 9.54 dB = 20 log 3).
  3. At a real corner the exact curve is 3 dB away from the asymptote and the phase is exactly ∓45°.
  4. Change a zero to "RHP zero": the magnitude is unchanged but the phase now falls instead of rising — that is non-minimum phase.
  5. Add delay: no change in magnitude, but the phase falls without limit and eats the phase margin.

STATES TO SCREENSHOT: preset A; preset A with K = 10 (negative margins); preset C; preset D; preset G; preset B with "Individual factors" on; 390-wide preset A.
````

#### 2.6.8 `pid-tuning`

````text
SIMULATION pid-tuning — title "PID tuning lab"
SITS BESIDE: notes/src/04_electrical.html, section "P, PI, PID controllers" (the end of "B1.3 Control Systems"); it also uses the steady-state error table of the same part.

PURPOSE
Feel what each of K_p, K_i and K_d does to a closed-loop step response (speed, overshoot, steady-state error, disturbance rejection, stability), and try the Ziegler–Nichols ultimate-gain rules given in the notes.

SYSTEM
  Unity negative feedback: e = r − y. Controller C(s) = K_p + K_i/s + K_d·s/(τ_f·s + 1) with a fixed derivative filter τ_f = 0.02 s (so the controller is realisable; say so in a note). Plant G(s) = N_p/D_p, one of:
     'first'  : 1/(s + 1)                type 0; never unstable with P control
     'second' : 2/((s + 1)(s + 2))       type 0; DC gain 1
     'third'  : 1/(s + 1)³               type 0; ultimate gain K_u = 8 at ω_u = √3 rad/s, period P_u = 2π/√3 = 3.6276 s
     'motor'  : 1/(s(s + 1))             type 1 (a position servo)
  The controller as a ratio N_c/D_c, in the simplest exact form (to avoid artificial pole–zero pairs):
     K_i = 0 and K_d = 0:   N_c = [K_p],                                   D_c = [1]
     K_i = 0, K_d ≠ 0:      N_c = [K_p·τ_f + K_d, K_p],                    D_c = [τ_f, 1]
     K_i ≠ 0, K_d = 0:      N_c = [K_p, K_i],                              D_c = [1, 0]
     otherwise:             N_c = [K_p·τ_f + K_d, K_p + K_i·τ_f, K_i],     D_c = [τ_f, 1, 0]
  Characteristic polynomial Δ = D_c·D_p + N_c·N_p. Reference → output: T = N_c·N_p/Δ. Reference → control: U = N_c·D_p/Δ. Load disturbance (a step added at the plant INPUT) → output: Y_d = N_p·D_c/Δ; → control: U_d = −N_c·N_p/Δ.
  All gains zero: the output stays at 0 (T = 0).
  Simulation: a unit-step reference at t = 0, plus an optional load disturbance of size d₀ applied at t_d = tEnd/2: y(t) = y_r(t) + d₀·y_d(t − t_d) for t ≥ t_d by superposition, where y_r and y_d are unit-step responses from RASim.math.tfStep; u(t) likewise from U and U_d.
  Stability from the roots of Δ: largest real part < −1e-6 stable; within ±1e-6 marginal; otherwise unstable.
  Steady-state error for the unit step (stable loops): e_ss = 1 − T(0) = D_c(0)·D_p(0)/Δ(0).
  Loop type = number of poles of C·G at the origin = (1 if K_i ≠ 0) + (1 for the motor plant).
  Metrics of the reference response y_r over the window, relative to the final value y_∞ = T(0) (stable loops with y_∞ > 0): overshoot % = (max y − y_∞)/y_∞·100 (0 when negative); peak time; 10–90 % rise time (of y_∞); 2 % settling time (the last time |y − y_∞| > 0.02·y_∞); when the response has not settled by tEnd report Infinity (shown as "> window").
  Ziegler–Nichols, ultimate-gain method, as in the notes: P: K_p = 0.5·K_u. PI: K_p = 0.45·K_u, T_i = P_u/1.2. PID: K_p = 0.6·K_u, T_i = P_u/2, T_d = P_u/8. K_i = K_p/T_i, K_d = K_p·T_d. Only the 'third' plant has a finite K_u; for the other plants the Z–N buttons are disabled with the explanation "no ultimate gain: this loop is stable for every K_p".

MODEL
  TAU_F = 0.02
  plants -> { first: { label, num, den, Ku: null, Pu: null }, second: { … }, third: { …, Ku: 8, Pu: 3.62760 }, motor: { … } }
  controller(Kp, Ki, Kd) -> { num, den }
  loop(plantId, Kp, Ki, Kd) -> { T, U, Yd, Ud (each { num, den }), charPoly, poles, stability: 'stable' | 'marginal' | 'unstable', type, ess (NaN unless stable), yFinal (NaN unless stable), distOffsetPerUnit (Y_d(0), NaN unless stable) }
  simulate(plantId, Kp, Ki, Kd, opts) -> { t, y, u, yRef }     opts = { tEnd = 15, n = 600, d0 = 0 }; yRef is the reference-only response used for the metrics
  metrics(t, y, yFinal) -> { overshootPct, tPeak, tRise, tSettle }
  zn(Ku, Pu, kind) -> { Kp, Ki, Kd } with kind 'P' | 'PI' | 'PID'

CONTROLS
  Plant select (key plant; default 'third') with the transfer function written out in each option.
  K_p slider 0 … 20, step 0.05, default 1 (key Kp).  K_i slider 0 … 20, step 0.05, default 0 (key Ki).  K_d slider 0 … 10, step 0.05, default 0 (key Kd).
  Quick buttons: "P only" (K_i = K_d = 0); "PI" (K_d = 0; K_i becomes 1 if it was 0); "PD" (K_i = 0; K_d becomes 1 if it was 0); "PID" (a zero K_i or K_d becomes 1).
  Ziegler–Nichols group: "Set K_p = K_u" (K_p = 8, K_i = K_d = 0: sustained oscillation with period P_u) and the three buttons "Z–N P", "Z–N PI", "Z–N PID" (disabled unless the plant has a K_u).
  Time span slider 2 … 60 s, step 1, default 15 (key tEnd).
  Load disturbance d₀ slider 0 … 1, step 0.05, default 0 (key d0), applied at half the time span.
  Checkbox "Control signal u(t)" (default off, key showU).  Buttons "Hold curve" / "Clear held" (one grey comparison curve) and "Reset".

VIEWS
  (1) A compact block diagram (canvas or HTML/CSS boxes) above the plots: r → Σ (+ and −) → [C(s)] → Σ with the disturbance d entering at the plant input → [G(s)] → y, with the feedback line back to the first Σ. Label the blocks with the current controller and plant.
  (2) Output plot: y(t) in blue (width 2.5); the reference r = 1 as a dashed dark line; the ±2 % band around the final value lightly shaded when stable; the held curve in grey; a vertical dashed marker at t_d labelled "disturbance" when d₀ > 0; a peak marker (dot plus "M_p = …%") and a settling-time marker when they exist. y-range: [min(0, data), max(1.5, data)] rounded outward but never beyond [−2, 4] (clip the curve when it grows beyond that).
  (3) Control-signal plot when enabled, below and sharing the time axis: u(t) in orange; y-range fitted but never beyond ±20, with the note "clipped" when the derivative kick at t = 0 exceeds it.
  Legend.

READ-OUTS
  Tag [key stability]: STABLE (ok) / MARGINALLY STABLE (warn) / UNSTABLE (bad).
  A controller line (data key ctrl), e.g. "C(s) = 4.8 + 2.646/s + 2.177 s" (zero terms omitted; "C(s) = 0" when all gains are zero).
  Stats: steady-state error to a step [ess] as a fraction and in % ("—" when not stable); loop type [type]; overshoot [overshoot] in %; peak time [tPeak]; rise time 10–90 % [tRise]; settling time 2 % [tSettle] ("> window" when not settled); closed-loop poles [poles] (fmtComplex list, the one closest to the jω-axis first); K_u [Ku] and P_u [Pu] of the plant ("—" when none); steady offset caused by the disturbance [distOffset] = d₀·Y_d(0) when stable.

STATE: { plant, Kp, Ki, Kd, tEnd, d0, showU, held }

CHECK VALUES
  first, K_p = 4: T = 4/(s + 5); y(∞) = 0.8; e_ss = 0.2; y(0.2) = 0.50570; type 0; stable.
  first, K_p = 1, K_i = 1: y(1) = 0.63212 (the response is exactly 1 − e^(−t)); e_ss = 0; type 1.
  second, K_p = 5: y(∞) = 0.83333; overshoot 22.11 %; poles −1.5 ± j3.1225.
  third, K_p = 8: marginal with poles ±j1.73205 and −3.  third, K_p = 1: e_ss = 0.5.  third, K_p = 9: unstable.
  zn(8, 3.62760, 'PID') = (4.8, 2.6464, 2.1766); 'PI' = (3.6, 1.1909, 0); 'P' = (4, 0, 0).
  third with Z–N PID: stable, e_ss = 0, overshoot 41.5 % (± 0.5), peak time 2.19 s (± 0.02), 2 % settling 9.3 s (± 0.2).  third with Z–N PI: overshoot 56.1 % (± 0.5).  third with Z–N P: final value 0.8, overshoot 54.3 % (± 0.5).
  motor, K_p = 1: T = 1/(s² + s + 1): overshoot 16.30 %, peak time 3.628 s, e_ss = 0, type 1.  motor, K_p = 4: overshoot 44.43 %.  motor, K_p = 4, K_d = 3: overshoot 2.4 % (± 0.3), poles near −1.601, −2.673 and −46.73.
  Disturbance: third, K_p = 4, d₀ = 1: steady offset 0.2; with any K_i > 0 (stable loop): 0.  motor with P only, K_p = 2, d₀ = 1: offset 0.5 — the plant's own integrator does not remove an input disturbance.
  Invariants: simulate() agrees with an independent RK4 time-domain simulation of the plant plus the filtered PID in state-space form (small step) to 1e-4, for 50 random stable gain sets per plant, with and without the disturbance; y(0) = 0; y_∞ = 1 − e_ss; with K_i > 0 and a stable loop e_ss = 0; the stability class from the poles agrees with the simulated response (bounded or growing).

HINTS (use these)
  1. Plant 1/(s+1), P only: raise K_p — the error shrinks as 1/(1 + K_p) but never reaches zero. Now add K_i: the error goes to zero (the integrator raises the loop type).
  2. Plant 1/(s+1)³: press "Set K_p = K_u" — sustained oscillation with period P_u ≈ 3.63 s. Then press Z–N PID and compare it with Z–N P and Z–N PI using Hold curve.
  3. Add K_d to an oscillatory response: overshoot and settling time drop (extra damping), but the steady-state error does not change.
  4. Apply a load disturbance: only a controller with integral action brings the output back to the set-point — even for the motor plant, whose own integrator sits after the disturbance.
  5. Too much K_i: more overshoot, and eventually instability.

STATES TO SCREENSHOT: default (third, K_p = 1); third at K_p = 8; third with Z–N PID plus a held Z–N P curve; first with K_p = 4, then with K_i added; motor with K_p = 4, K_d = 3 and u(t) shown; third with Z–N PID and d₀ = 0.5; 390-wide default.
````

#### 2.6.9 `mohr-circle`

````text
SIMULATION mohr-circle — title "Mohr's circle for plane stress"
SITS BESIDE: notes/src/05_mechanical.html, section "Mohr's circle — plane stress" (under "B2.1 Mechanics of Materials"). Use exactly the notes' transformation equations.

PURPOSE
Connect the stress element (σ_x, σ_y, τ_xy), its rotated version at angle θ, and the corresponding point on Mohr's circle (angle 2θ); find the principal stresses and planes, the maximum in-plane shear and the absolute maximum shear (σ₃ = 0).

SIGN CONVENTIONS (state them in a note inside the panel)
  Normal stress: tension positive. τ_xy is positive when it acts in the +y direction on the +x face (and in the +x direction on the +y face). θ is counter-clockwise positive, measured from the x-axis to the x′-axis (the normal of the rotated face).
  Transformation (as in the notes):
     σ_x′ = (σ_x + σ_y)/2 + [(σ_x − σ_y)/2]·cos 2θ + τ_xy·sin 2θ
     σ_y′ = (σ_x + σ_y)/2 − [(σ_x − σ_y)/2]·cos 2θ − τ_xy·sin 2θ
     τ_x′y′ = −[(σ_x − σ_y)/2]·sin 2θ + τ_xy·cos 2θ
  Mohr's circle is drawn with σ to the right and τ POSITIVE DOWNWARD. The point of the x′ face is X′ = (σ_x′, τ_x′y′) and of the y′ face Y′ = (σ_y′, −τ_x′y′). With this convention a counter-clockwise rotation θ of the element moves X′ counter-clockwise around the circle (as seen on screen) by 2θ — the same sense. Label the vertical axis "τ (positive ↓)".
  σ_avg = (σ_x + σ_y)/2;  R = √{[(σ_x − σ_y)/2]² + τ_xy²};  σ₁ = σ_avg + R ≥ σ₂ = σ_avg − R.
  θ_p = ½·atan2(τ_xy, (σ_x − σ_y)/2): the counter-clockwise angle from the x-axis to the direction of σ₁, in (−90°, 90°]; the σ₂ direction is θ_p + 90°. When R = 0 every direction is principal: report θ_p = 0 and say so.
  Maximum in-plane shear τ_max = R; it acts as +R on the plane θ_s = θ_p − 45°, where both normal stresses equal σ_avg.
  Absolute maximum shear in plane stress (σ₃ = 0): τ_abs = max(|σ₁ − σ₂|, |σ₁|, |σ₂|)/2.
  Equivalent stresses (they link to the failure theories in the notes): von Mises σ_vm = √(σ₁² − σ₁σ₂ + σ₂²); Tresca equivalent = 2·τ_abs.

MODEL (stresses in MPa, angles in radians)
  transform(sx, sy, txy, theta) -> { sx1, sy1, txy1 }
  principal(sx, sy, txy) -> { avg, R, s1, s2, thetaP, thetaS, tauMax, tauAbs, vonMises, tresca, isotropic (true when R < 1e-9) }
  circlePoints(sx, sy, txy, theta) -> { C: [avg, 0], X: [sx, txy], Y: [sy, −txy], X1: [sx1, txy1], Y1: [sy1, −txy1] }     (each point is [σ, τ] with τ as defined above)

CONTROLS
  σ_x slider −100 … 100 MPa, step 1, default 50 (key sx).  σ_y −100 … 100, step 1, default −10 (key sy).  τ_xy −100 … 100, step 1, default 40 (key txy).
  θ slider −90 … 90°, step 0.5, default 0 (key theta).
  Buttons: "θ = 0", "Principal planes (θ = θ_p)", "Max shear planes (θ = θ_s)" (they set θ to the exact value, not snapped to the slider step), "Reset".
  Presets (select, key preset): "General state (50, −10, 40)" · "Pure shear τ = 50" (0, 0, 50) · "Uniaxial tension σ = 80" (80, 0, 0) · "Equal biaxial 60, 60" (60, 60, 0) · "Thin cylinder: hoop 80, axial 40" (80, 40, 0) · "Tension–compression (60, −60, 0)" · "Biaxial compression (−40, −80, 0)".
  Checkbox "3D circles (σ₃ = 0)" (default off, key show3D): adds the circles between σ₁ and 0 and between σ₂ and 0 and marks the absolute maximum shear.
  Drag: the point X′ on the circle can be dragged around the circle; θ follows as half of the angle of CX′ measured from CX (kept continuous during the drag and wrapped into −90 … 90°); the rotated element follows.

VIEWS (two canvases side by side in the main column, stacked when the panel is narrow; controls and stats in the side column)
  (1) Stress-element canvas: a square element rotated counter-clockwise by θ about its centre, with the fixed x and y axes (thin), the x′ axis along the element's rotated normal, and the angle arc θ (labelled; drawn the correct way for negative θ). Arrows on all four faces: normal-stress arrows perpendicular to each face, pointing AWAY from the face for tension (positive) and INTO the face for compression (negative); shear arrows along each face following the sign of τ_x′y′: for positive τ_x′y′ the shear arrow on the +x′ face points towards +y′, on the −x′ face towards −y′, on the +y′ face towards +x′ and on the −y′ face towards −x′ (so the arrow heads meet at the (+x′, +y′) corner and at the (−x′, −y′) corner); all four reverse for negative τ_x′y′. Arrow lengths grow with magnitude (with a visible minimum); an arrow is omitted when its magnitude is below 0.05 MPa. Value labels in MPa: "σ_x′ = 70.0", "σ_y′ = −30.0", "τ_x′y′ = 0.0". A small faint inset shows the original element (θ = 0) with σ_x, σ_y and τ_xy for reference.
  (2) Mohr's-circle canvas with EQUAL axes, τ positive downward (for example plot −τ with RASim.plot and relabel the ticks, or use your own transform; the tick labels must read positive values below the σ-axis). The window is centred on the circle and always includes the origin and, when enabled, the 3D circles. Draw: the circle; the centre C labelled "σ_avg = 20"; the reference diameter X–Y (faint, with labels X and Y); the current diameter X′–Y′ (bold, with dots, the labels X′ and Y′ and the coordinates of X′); the arc for 2θ at the centre from CX to CX′ labelled "2θ = 53.1°" (counter-clockwise on screen for positive θ); the principal points on the σ-axis labelled σ₁ and σ₂ with values; the two points of maximum shear (σ_avg, ±τ_max) labelled; a thin arc for 2θ_p from CX to the σ₁ point. With "3D circles" on: the two extra circles and τ_abs marked.
  Legend.

READ-OUTS
  Stats: σ_x′ [sx1], σ_y′ [sy1], τ_x′y′ [txy1] (1 decimal, MPa); σ_avg [avg]; R [R]; σ₁ [s1]; σ₂ [s2]; θ_p [thetaP] in ° (with "and θ_p + 90°"); in-plane τ_max [tauMax]; θ_s [thetaS]; absolute τ_max [tauAbs] (state warn when it exceeds the in-plane value); von Mises [vonMises]; Tresca [tresca]; invariant check [invariant] like "σ_x′ + σ_y′ = σ_x + σ_y = 40".
  Tag [key state]: PRINCIPAL PLANES (ok) when |τ_x′y′| < 0.05 and R > 0; MAX SHEAR PLANES (warn) when ||τ_x′y′| − R| < 0.05 and R > 0.05; HYDROSTATIC — NO SHEAR ON ANY PLANE when R < 1e-9; otherwise empty.

STATE: { sx, sy, txy, theta (degrees), show3D }

CHECK VALUES
  (50, −10, 40): σ_avg = 20, R = 50, σ₁ = 70, σ₂ = −30, θ_p = 26.565°, θ_s = −18.435°, τ_max = 50, τ_abs = 50, von Mises = 88.882, Tresca = 100. At θ = θ_p: (70, −30, 0). At θ = 45°: σ_x′ = 60, σ_y′ = −20, τ_x′y′ = −30. At θ = θ_s: σ_x′ = σ_y′ = 20 and τ_x′y′ = +50.
  Pure shear (0, 0, 50): σ₁ = 50, σ₂ = −50, θ_p = 45°, θ_s = 0°.
  Uniaxial (80, 0, 0): σ₁ = 80, σ₂ = 0, θ_p = 0°; τ_max = 40 at θ_s = −45°, where σ_x′ = σ_y′ = 40 and τ_x′y′ = +40.
  Thin cylinder (80, 40, 0): in-plane τ_max = 20 but τ_abs = 40; Tresca = 80; von Mises = 69.282.
  Equal biaxial (60, 60, 0): R = 0, isotropic, τ_abs = 30.
  (−40, −80, 0): σ₁ = −40, σ₂ = −80, τ_max = 20, τ_abs = 40.
  (20, 80, −30): σ_avg = 50, R = 42.426, σ₁ = 92.426, σ₂ = 7.574, θ_p = −67.5°.
  Invariants (1000 random states and angles): σ_x′ + σ_y′ = σ_x + σ_y; σ_x′·σ_y′ − τ_x′y′² = σ_x·σ_y − τ_xy²; transform(θ + 180°) = transform(θ); transform(θ + 90°) swaps the normal stresses and reverses the shear sign; transform at θ_p gives (σ₁, σ₂, 0); at θ_s gives (σ_avg, σ_avg, +R); the transformation equals the tensor rotation Q·σ·Qᵀ computed with a 2×2 rotation matrix Q = [[cos θ, sin θ], [−sin θ, cos θ]]; in the screen frame (σ to the right, −τ up) X1 is at distance R from C and at the angle of CX plus 2θ counter-clockwise.
  Browser check of the drawing: at θ = 0 with (50, −10, 40) the X point is drawn BELOW the σ-axis (τ positive downward); increasing θ moves X′ counter-clockwise on screen.

HINTS (use these)
  1. Press "Principal planes": the shear vanishes and the normal stresses become σ₁ and σ₂ — the two ends of the circle's horizontal diameter.
  2. Press "Max shear planes": they are 45° from the principal planes (90° on the circle), and the normal stress there is σ_avg, not zero.
  3. Preset "Pure shear": it is the same as equal tension and compression at 45°.
  4. Preset "Thin cylinder" with 3D circles on: the in-plane τ_max is (σ₁ − σ₂)/2 = 20, but the true maximum is σ₁/2 = 40 because σ₃ = 0.
  5. An angle θ on the element is 2θ on the circle, in the same sense when τ is drawn positive downward.

STATES TO SCREENSHOT: default at θ = 0; default at θ_p; default at θ_s; default at θ = −60°; pure shear at θ = 45°; thin cylinder with 3D circles; biaxial compression (arrows pointing inward); 390-wide default.
````

#### 2.6.10 `vibration-sdof`

````text
SIMULATION vibration-sdof — title "Single-DOF vibration: free, forced and isolation"
SITS BESIDE: notes/src/05_mechanical.html, sections "Vibrations of single DOF systems" and "Effect of damping; vibration isolation" (under "B2.2 Kinematics and Dynamics of Machines").

PURPOSE
(a) Free vibration m·x″ + c·x′ + k·x = 0: natural frequency, damping ratio, the damping cases, logarithmic decrement. (b) Forced vibration by F₀ sin ωt: magnification factor and phase versus the frequency ratio r = ω/ωₙ. (c) Transmissibility and the isolation rule r > √2.

FORMULAS
  ωₙ = √(k/m); fₙ = ωₙ/2π; Tₙ = 2π/ωₙ; c_c = 2√(km) = 2mωₙ; ζ = c/c_c; static deflection δ_st = m·g/k with g = 9.81 m/s² (so ωₙ = √(g/δ_st)).
  Free response from x(0) = x₀, x′(0) = v₀:
     ζ = 0:              x = x₀ cos ωₙt + (v₀/ωₙ) sin ωₙt
     0 < ζ < 1:          ω_d = ωₙ√(1 − ζ²);  x = e^(−ζωₙt)[x₀ cos ω_d t + ((v₀ + ζωₙx₀)/ω_d) sin ω_d t]
     |ζ − 1| < 1e-6:     x = [x₀ + (v₀ + ωₙx₀)t]·e^(−ωₙt)
     ζ > 1:              s₁ = −ωₙ(ζ − √(ζ² − 1)), s₂ = −ωₙ(ζ + √(ζ² − 1));  x = A·e^(s₁t) + B·e^(s₂t) with A = (v₀ − s₂x₀)/(s₁ − s₂), B = (s₁x₀ − v₀)/(s₁ − s₂)
  Logarithmic decrement (0 < ζ < 1): δ = ln(xₙ/xₙ₊₁) = 2πζ/√(1 − ζ²); ratio of successive peaks = e^δ; damped period T_d = 2π/ω_d; number of cycles for the amplitude to halve = ln 2/δ.
  Forced response in steady state (r = ω/ωₙ):
     magnification factor MF = X/(F₀/k) = 1/√[(1 − r²)² + (2ζr)²]
     phase lag of the displacement behind the force: φ = atan2(2ζr, 1 − r²) ∈ [0°, 180°] (90° at r = 1 for any ζ > 0)
     transmissibility TR = F_T/F₀ = √[1 + (2ζr)²]/√[(1 − r²)² + (2ζr)²]     (the same expression is X/Y for base excitation)
     rotating unbalance: M·X/(m₀e) = r²·MF
     the peak of MF is at r = √(1 − 2ζ²) (only for ζ < 0.7071): MF_max = 1/(2ζ√(1 − ζ²)); at r = 1: MF = 1/(2ζ)
     TR = 1 at r = √2 for every ζ; isolation (TR < 1) only for r > √2; isolation efficiency = (1 − TR)·100 %.
     ζ = 0 with r = 1: MF and TR are infinite (show "∞ (resonance)").

MODEL (SI units; x in metres)
  natural(m, k) -> { wn, fn, Tn, cc, dst }
  damping(m, k, zeta) -> { c, kind: 'undamped' | 'under' | 'critical' | 'over', wd, Td, delta, ratio, halfCycles }     (NaN where not applicable)
  free(m, k, zeta, x0, v0, t) -> x;   freeVel(m, k, zeta, x0, v0, t) -> velocity
  peaks(m, k, zeta, x0, v0, tEnd) -> [{ t, x }]: the successive positive local maxima of the under-damped free response in [0, tEnd] (a maximum at t = 0 counts when v₀ = 0 and x₀ > 0)
  envelope(m, k, zeta, x0, v0) -> X, the amplitude such that |x| ≤ X·e^(−ζωₙt) for the under-damped case: X = √(x₀² + ((v₀ + ζωₙx₀)/ω_d)²)
  MF(r, zeta);  phase(r, zeta) (radians);  TR(r, zeta);  unbalance(r, zeta)
  peakMF(zeta) -> { r, MF } or null
  freeTEnd(m, k, zeta) -> seconds: under-damped: min(6/(ζωₙ), 12·T_d) but at least 3·T_d; undamped: 6·Tₙ; critical: 8/ωₙ; over-damped: 6/|s₁|

CONTROLS
  Tab: segmented "Free vibration" / "Forced response & isolation" (key tab; default free).
  Shared: m slider 0.5 … 10 kg, step 0.1, default 1 (key m); k slider 10 … 2000 N/m, step 10, default 100 (key k); ζ slider 0 … 2, step 0.01, default 0.1 (key zeta).
  Free tab: x₀ slider −100 … 100 mm, step 1, default 50 (key x0); v₀ slider −1 … 1 m/s, step 0.05, default 0 (key v0); a Play / Pause toggle and a Restart button; speed segmented 1× / 0.5× / 0.25× (key speed; default 0.5).
  Forced tab: r slider 0 … 4, step 0.01, default 1 (key r); F₀ slider 1 … 100 N, step 1, default 10 (key F0); a select for the first plot (key curve): "Constant force amplitude: X/(F₀/k)" / "Rotating unbalance: MX/(m₀e)"; preset buttons "Resonance r = 1", "r = √2", "Isolation r = 3"; a Play / Pause toggle for the steady-state animation.
  Reset.

VIEWS
  Free tab:
   (1) Animation canvas: a wall on the left, a spring (RASim.draw.spring) and a dash-pot in parallel, a mass block on a floor line with small rollers, an equilibrium marker, and the current displacement as an arrow with its value in mm. Horizontal motion, scaled so that the largest |x| of the response uses about 30 % of the canvas width. Animation time advances at the chosen speed and loops back to 0 at freeTEnd.
   (2) Plot of x(t) in mm over [0, freeTEnd]: the curve (blue); for the under-damped case the envelope ±X·e^(−ζωₙt) (dashed orange) and the successive positive peaks as dots, the first three labelled x₁, x₂, x₃, with the label "ln(x₁/x₂) = δ = 0.631"; a vertical time cursor synchronised with the animation.
  Forced tab:
   (1) Plot A: the selected amplitude ratio versus r ∈ [0, 4], y from 0 to 6 (curves clipped above): a family of faint grey curves for ζ = 0.05, 0.1, 0.2, 0.3, 0.5, 0.707, 1 (each labelled), the current ζ in bold blue, a vertical line at r = 1 labelled "resonance", and the operating point as a dot with drop lines.
   (2) Plot B: the phase φ in degrees versus r (0 … 180°, ticks every 45°): the faint family, the current ζ in bold, the operating point, a horizontal line at 90°.
   (3) Plot C: transmissibility TR versus r, y from 0 to 4 (clipped): the family and the current ζ, the horizontal line TR = 1, a vertical line at r = √2 labelled "r = √2", the region r > √2 lightly shaded green and labelled "isolation (TR < 1)", and the operating point.
   (4) A small animation canvas: the spring–mass–damper oscillating in steady state (displayed amplitude normalised for visibility, the true X written out), a force arrow on the mass proportional to F₀ sin ωt, and beside it a phasor diagram with the rotating phasors F₀ (reference) and X lagging by φ with the angle labelled — so the student sees in-phase motion for r ≪ 1, quadrature at r = 1 and anti-phase for r ≫ 1. The displayed cycle takes about 2 s regardless of ω.
   All three plots must be readable at 390 px (stack them vertically on narrow panels).
  Legend.

READ-OUTS
  Shared: ωₙ [wn] in rad/s; fₙ [fn] in Hz; c_c [cc] and c [c] in N·s/m; δ_st [dst] in mm; tag [key kind]: UNDAMPED / UNDERDAMPED / CRITICALLY DAMPED / OVERDAMPED.
  Free tab: ω_d [wd]; T_d [Td]; δ [delta]; x₁/x₂ [ratio]; cycles to halve [half].
  Forced tab: ω [w] in rad/s and N [rpm] = 60ω/2π; MF [MF]; X [X] in mm = MF·F₀/k (or "∞"); phase φ [phase] in °; TR [TR]; transmitted force F_T [FT] = TR·F₀ in N; isolation [isolation]: "85.5 % isolated" (ok) when r > √2, otherwise "no isolation — TR ≥ 1" (warn); peak [peak]: "MF_max = 5.03 at r = 0.990" or "no peak (ζ ≥ 0.707)".

STATE: { tab ('free' | 'forced'), m, k, zeta, x0 (mm), v0 (m/s), speed ('1' | '0.5' | '0.25'), r, F0, curve ('force' | 'unbalance'), playing (boolean), time (seconds of animation time; setState({ time }) places the cursor and the mass there — needed for deterministic screenshots) }

CHECK VALUES
  m = 1, k = 100: ωₙ = 10, fₙ = 1.59155, c_c = 20, δ_st = 98.1 mm.
  ζ = 0.1: c = 2; ω_d = 9.94987; T_d = 0.631484; δ = 0.631484; ratio 1.88040; halving in 1.0976 cycles.
  free(1, 100, 0.1, 0.05, 0, 0.631484) = 0.026590; free(1, 100, 1, 0.05, 0, 0.1) = 0.036788; free(1, 100, 2, 0.05, 0, 0.1) = 0.041113; free(1, 100, 0, 0.05, 0.5, π/20) = 0.05.
  MF(1, 0.1) = 5; MF(0, ζ) = 1; MF(√2, 0.1) = 0.96225; phase(1, ζ) = 90°; phase(2, 0.1) = 172.405°; TR(√2, ζ) = 1 for ζ = 0.05, 0.3 and 1; TR(3, 0.1) = 0.14537; TR(3, 0.5) = 0.37012; peakMF(0.1) = { r: 0.98995, MF: 5.0252 }; peakMF(0.8) = null; unbalance(1, 0.1) = 5; unbalance(r, ζ) → 1 as r grows.
  Invariants: free() satisfies the differential equation (central finite differences, relative 1e-5) and the initial conditions for 300 random parameter sets across all four cases, and is continuous across ζ = 1; freeVel is the derivative of free; the ratios of successive peaks() equal e^δ and they are T_d apart; MF, phase and TR agree with the complex frequency response computed directly: X/F₀ = 1/(k − mω² + jcω) and F_T/F₀ = (k + jcω)/(k − mω² + jcω); TR < 1 exactly when r > √2 (random tests); in the isolation region TR increases with ζ.

HINTS (use these)
  1. Free vibration: read two successive peaks — ln(x₁/x₂) is the logarithmic decrement δ, about 2πζ for light damping.
  2. Raise ζ to 1: the fastest return to rest with no oscillation (critical damping, c = c_c = 2√(km)).
  3. Forced: at r = 1 the magnification is 1/(2ζ) and the displacement lags the force by exactly 90°, whatever the damping.
  4. Every transmissibility curve passes through TR = 1 at r = √2. Beyond that the mount isolates — and there LESS damping gives better isolation.
  5. To isolate a machine running at a fixed speed, soften the mounts: a lower k lowers ωₙ and raises r.

STATES TO SCREENSHOT: free default at time 0.4 s; free with ζ = 1; free with ζ = 2; free with ζ = 0; forced default (r = 1); forced at r = 3; forced with ζ = 0 at r = 1 (the ∞ case); forced with the unbalance curve; 390-wide in both tabs.
````

#### 2.6.11 `beam-sfd-bmd`

````text
SIMULATION beam-sfd-bmd — title "Beam: shear force, bending moment and deflection"
SITS BESIDE: notes/src/05_mechanical.html, sections "Shear force and bending moment diagrams" and "Deflection of beams" (with its table of standard cases), under "B2.1 Mechanics of Materials".

PURPOSE
Place loads on a beam and watch the reactions, the shear-force diagram, the bending-moment diagram and the deflected shape update together. Learn the signature of each load type (point load: a step in V and a kink in M; UDL: sloping V and parabolic M; couple: a jump in M only), that M is extreme where V crosses zero, points of contraflexure, and the standard results in the notes' table.

CONVENTIONS (state them in a note inside the panel)
  x from the left end, 0 ≤ x ≤ L. Units: kN, m, kN·m; EI in kN·m²; deflection displayed in mm.
  Loads: point load P (kN) positive DOWNWARD; distributed load w (kN/m) positive DOWNWARD over [x1, x2]; applied couple M₀ (kN·m) positive CLOCKWISE.
  Shear force V(x) = the sum of the UPWARD forces to the left of the section. Bending moment M(x) positive when SAGGING (compression on top), computed from everything to the left of the section. Consequently dM/dx = V and dV/dx = −w (as in the notes), a downward point load makes V drop by P, and a clockwise couple makes M jump UP by M₀ when moving from left to right.
  Deflection y positive UPWARD (a downward sag is negative); slope θ = dy/dx; EI·y″ = M.
BEAM TYPES
  'ss': simply supported, pin at x = a and roller at x = b (0 ≤ a < b ≤ L; defaults a = 0 and b = L; overhangs appear when a > 0 or b < L). Reactions, upward positive: R_b = [Σ P_i(x_i − a) + Σ W_j(x̄_j − a) + Σ M₀_k]/(b − a) with W_j = w_j(x2_j − x1_j) and x̄_j = (x1_j + x2_j)/2; R_a = Σ P_i + Σ W_j − R_b.
  'cantilever': fixed at x = 0, free at x = L. Wall force R = Σ P_i + Σ W_j (upward); bending moment at the wall M(0⁺) = −[Σ P_i·x_i + Σ W_j·x̄_j + Σ M₀_k] (negative = hogging for downward loads).
  Macaulay form (⟨u⟩ⁿ = uⁿ for u > 0, else 0; ⟨u⟩⁰ is the unit step):
     V(x) = R_a⟨x − a⟩⁰ + R_b⟨x − b⟩⁰ − Σ P_i⟨x − x_i⟩⁰ − Σ w_j(⟨x − x1_j⟩¹ − ⟨x − x2_j⟩¹)          [cantilever: R instead of the two support terms]
     M(x) = R_a⟨x − a⟩¹ + R_b⟨x − b⟩¹ − Σ P_i⟨x − x_i⟩¹ − Σ (w_j/2)(⟨x − x1_j⟩² − ⟨x − x2_j⟩²) + Σ M₀_k⟨x − x_k⟩⁰     [cantilever: M(0⁺) + R·x instead of the two support terms]
     EI·θ(x) = C₁ + ∫M and EI·y(x) = C₂ + C₁·x + ∬M, integrating each Macaulay term analytically (⟨u⟩ⁿ → ⟨u⟩ⁿ⁺¹/(n + 1)). Boundary conditions: ss: y(a) = 0 and y(b) = 0; cantilever: y(0) = 0 and θ(0) = 0. The integration must be analytical (exact), not numerical.
  At a discontinuity V (point loads, supports) and M (couples) have two values: the model functions take a side argument.
RESULTS TO REPORT
  Reactions; the maximum |V| with its location; the maximum sagging moment (largest positive M) and the maximum hogging moment (most negative M) with locations — evaluate at every critical point: both sides of every load position, UDL end and support, the beam ends, and every interior zero of V (inside a UDL span V is linear: solve exactly); points of contraflexure = interior points where M changes sign, including a sign change through the jump at a couple (flagged atCouple); the maximum |deflection| with its location (a grid of 2000 intervals plus refinement where the slope changes sign, and the beam ends); slopes at the supports (ss) or at the free end (cantilever).

MODEL
  solve(beam) -> result, with beam = { type: 'ss' | 'cantilever', L, a, b, EI, loads: [ { kind: 'point', P, x } | { kind: 'udl', w, x1, x2 } | { kind: 'moment', M, x } ] } (a and b are ignored for the cantilever) and
  result = { reactions: { Ra, Rb } for ss or { R, Mwall } for the cantilever (Mwall = M(0⁺)), V(x, side), M(x, side), slope(x), y(x) (functions; side = −1 for the left limit and +1 for the right limit; the default is +1 except at x = L where it is −1), criticalX (sorted array), maxV: { value, x }, maxSag: { value, x } | null, maxHog: { value, x } | null, contraflexure: [{ x, atCouple }], maxDeflection: { value (signed, metres), x } }
  sample(result, n) -> { x, V, M, y } arrays for plotting that contain both sides of every discontinuity, so steps are drawn vertical
  equilibrium(beam, result) -> { sumF, sumM } residuals (both ≈ 0)

CONTROLS
  Beam: segmented "Simply supported" / "Cantilever" (key type; default ss).
  L slider 2 … 10 m, step 0.5, default 6 (key L).  EI log slider 1000 … 100000 kN·m², default 10000 (key EI).
  Supports (ss only): a slider 0 … L and b slider 0 … L, step 0.1 (keys a, b), keeping b − a ≥ 0.5 by pushing the other one; defaults 0 and L. When L changes, a support that was at the end stays at the end; everything else is clamped into [0, L].
  Loads list (at most 5), each row: a type select (Point / UDL / Couple), a magnitude slider (P: −100 … 100 kN; w: −50 … 50 kN/m; M₀: −100 … 100 kN·m; step 1), position slider(s) (x for a point load or couple; x1 and x2 for a UDL with x2 − x1 ≥ 0.1; step 0.1) and a remove button. Row keys load1type, load1mag, load1x, load1x2, load1remove, … load5…. Buttons "+ Point load" (P = 10 at L/2), "+ UDL" (w = 5 over the whole beam), "+ Couple" (M₀ = 10 at L/2), disabled at 5 loads.
  Presets (select, key preset): "Simply supported, central point load" (default: L = 6, P = 20 at 3) · "Simply supported, full UDL" (L = 6, w = 5) · "Cantilever, end point load" (L = 4, P = 10 at 4) · "Cantilever, full UDL" (L = 4, w = 5) · "Cantilever, end couple" (L = 4, M₀ = 10 at 4) · "Overhang with contraflexure" (L = 8, a = 0, b = 6, w = 4 over 0 … 8) · "Couple at mid-span" (L = 6, M₀ = 12 at 3) · "Two point loads — pure bending between them" (L = 6, P = 10 at 2 and at 4). Presets use EI = 10000.
  Drag on the beam picture: point loads and couples horizontally; either end of a UDL; the supports (ss). Positions snap to 0.1 m and the sliders follow.
  Checkbox "Deflection diagram" (default on, key showDeflection). Reset.

VIEWS (one tall canvas, or stacked canvases, sharing the same x scale and margins so that everything lines up vertically)
  (1) Beam picture: the beam as a thick bar from 0 to L with a length scale; supports (a triangle pin at a, a triangle on rollers at b; a hatched wall at x = 0 for the cantilever); point loads as arrows (down for positive P, up for negative) with lengths growing with magnitude within limits, and value labels; UDLs as a row of small arrows under a line with a label like "5 kN/m" (arrows up for negative w); couples as a curved arrow (clockwise for positive) with a label; reactions as green arrows with values (and the wall moment for the cantilever); the deflected shape as a dashed curve passing through the supports, vertically exaggerated (largest deflection about 18 px), with the note "deflection not to scale".
  (2) SFD: V(x) filled, positive above the axis, steps vertical; values labelled at the ends and at local extremes (avoid clutter); zero crossings marked.
  (3) BMD: M(x) filled, sagging (positive) plotted ABOVE the axis, with the axis label "M (kN·m), sagging +"; the maximum sagging and hogging values labelled with their x; contraflexure points marked with a small circle and "M = 0".
  (4) Deflection diagram when enabled: y(x) in mm with the maximum labelled.
  A hover cursor: a vertical line through all the diagrams and a read-out line (data key cursor) with x, V, M and y.
  Legend and the convention note.

READ-OUTS
  Stats: reactions [reactions], e.g. "R_A = 10 kN ↑, R_B = 10 kN ↑" (↓ with the magnitude for negative values; cantilever: "R = 10 kN ↑, M_wall = −40 kN·m"); maximum |V| [maxV] with x; maximum sagging M [maxSag] with x ("—" when there is none); maximum hogging M [maxHog] with x ("—" when none); contraflexure [contra] (a list like "x = 5.33 m", or "none"); maximum deflection [maxDefl] in mm with x and direction (↓ or ↑); slopes [slopes] in rad (ss: at A and at B; cantilever: at the free end); equilibrium check [check] "ΣF = 0, ΣM = 0 ✓" from model.equilibrium (bad when either residual exceeds 1e-6).

STATE: { type, L, a, b, EI, loads: [ { kind, P or w or M, x or x1 and x2 } ], showDeflection }

CHECK VALUES (EI = 10000 kN·m²)
  ss, L = 6, P = 20 at 3: R_A = R_B = 10; V = +10 on (0, 3) and −10 on (3, 6); M(3) = 30 (PL/4); M(1.5) = 15; y(3) = −0.009 m (PL³/48EI); slope at A = −0.0045 rad and at B = +0.0045 (PL²/16EI); no contraflexure.
  ss, L = 6, w = 5 over 0 … 6: reactions 15 each; M(3) = 22.5 (wL²/8); y(3) = −0.0084375 m (5wL⁴/384EI); end slopes ∓0.0045 (wL³/24EI).
  cantilever, L = 4, P = 10 at 4: R = 10; M_wall = −40; V = 10 for 0 < x < 4; M(x) = −10(4 − x); y(4) = −0.0213333 m (PL³/3EI); slope(4) = −0.008 (PL²/2EI).
  cantilever, L = 4, w = 5 over 0 … 4: R = 20; M_wall = −40 (wL²/2); y(4) = −0.016 m (wL⁴/8EI); slope(4) = −0.0053333 (wL³/6EI).
  cantilever, L = 4, M₀ = 10 (clockwise) at 4: R = 0; M = −10 for 0 < x < 4; y(4) = −0.008 m (ML²/2EI); slope(4) = −0.004 (ML/EI).
  ss, L = 6, M₀ = 12 (clockwise) at 3: R_A = −2 (downward), R_B = +2; M(3⁻) = −6, M(3⁺) = +6; contraflexure at x = 3 flagged atCouple; y(1.5) = +0.3375 mm, y(3) = 0, y(4.5) = −0.3375 mm; slope at A = +0.0003 rad.
  overhang: ss, L = 8, a = 0, b = 6, w = 4 over 0 … 8: R_A = 10.6667, R_B = 21.3333; V = 0 at x = 2.6667 where M = 14.2222 (maximum sagging); M(6) = −8 (maximum hogging); contraflexure at x = 5.3333; M(8) = 0; y(3) = −4.95 mm; tip y(8) = +3.2 mm (upward).
  two point loads: ss, L = 6, P = 10 at 2 and at 4: between the loads V = 0 and M = 20.
  Invariants (300 random beams of both types with 1–5 random loads of every kind and random supports): V and M are zero just beyond the right end (closure) and M is zero at free and simply supported ends; dM/dx = V and dV/dx = −w by finite differences away from discontinuities; EI·y″ = M by finite differences; the boundary conditions hold to 1e-9; superposition (the result for loads A plus B equals the sum of the two results); mirror symmetry (mirroring a simply supported beam with its loads mirrors M and y and reverses the sign of V); the deflection agrees with an independent numerical double integration of M/EI (relative 1e-5); equilibrium residuals are zero.

HINTS (use these)
  1. The bending moment is largest where the shear force crosses zero — check it with the full UDL and with the overhang preset.
  2. A point load makes a step in the SFD and a kink in the BMD; a UDL gives a sloping SFD and a parabolic BMD; a couple makes a jump in the BMD and nothing in the SFD.
  3. Overhang preset: slide support B inward — the overhang produces a hogging moment over the support and a point of contraflexure where M = 0.
  4. Compare with the table in the notes: central load PL/4 and PL³/48EI; full UDL wL²/8 and 5wL⁴/384EI; cantilever end load PL and PL³/3EI.
  5. Two equal point loads: between them V = 0 and M is constant — pure bending.

STATES TO SCREENSHOT: default; full UDL; cantilever with end load; cantilever with UDL; overhang; couple at mid-span; a mixed case (ss, L = 8, P = 20 at 2, w = 5 over 4 … 8, M₀ = −15 at 6); 390-wide default and overhang.
````

#### 2.6.12 `four-bar`

````text
SIMULATION four-bar — title "Four-bar linkage and Grashof's law"
SITS BESIDE: notes/src/05_mechanical.html, sections "Mechanisms basics" (Grashof's law, inversions, transmission angle, toggle, quick return) and "Displacement, velocity and acceleration analysis" (instantaneous centres, Kennedy's theorem), under "B2.2 Kinematics and Dynamics of Machines".

PURPOSE
Choose four link lengths and see at once (a) whether Grashof's condition s + l ≤ p + q holds and which mechanism results (crank-rocker, double-crank, double-rocker, triple-rocker), (b) the motion, the coupler curve and the limits of the rocker, (c) the transmission angle and the toggle (dead-centre) positions, (d) the angular-velocity ratio and the instantaneous centre I₂₄.

GEOMETRY AND NOTATION
  Fixed pivots O₂ = (0, 0) and O₄ = (d, 0). Links: a = O₂A (the input, "crank"), b = AB (coupler), c = O₄B (the output, "rocker"), d = O₂O₄ (frame). θ₂, θ₃, θ₄ are the angles of O₂→A, A→B and O₄→B measured counter-clockwise from the +x axis (the direction O₂→O₄).
  A = (a cos θ₂, a sin θ₂). Let f = |O₄ − A| = √(a² + d² − 2ad·cos θ₂). The linkage closes at this θ₂ iff |b − c| ≤ f ≤ b + c and f > 0.
  B by circle intersection: e = (O₄ − A)/f, n = (−e_y, e_x) (the left normal of A→O₄), α = (b² − c² + f²)/(2f), h = √(max(0, b² − α²)); B = A + α·e + mode·h·n with mode = +1 for the OPEN configuration (B on the left of the directed line A→O₄) and −1 for the CROSSED configuration.
  θ₃ = atan2(B_y − A_y, B_x − A_x); θ₄ = atan2(B_y, B_x − d).
  Transmission angle μ = the angle ABO₄ between the coupler and the output link: cos μ = (b² + c² − f²)/(2bc), μ ∈ [0°, 180°]. Quality is judged by its deviation from 90°: "poor" when μ < 40° or μ > 140°.
  Angular-velocity ratios from the loop-closure equations: ω₄/ω₂ = a·sin(θ₂ − θ₃)/(c·sin(θ₄ − θ₃)); ω₃/ω₂ = a·sin(θ₄ − θ₂)/(b·sin(θ₃ − θ₄)). At a toggle (coupler and output collinear) they are unbounded. Ideal mechanical advantage (torque ratio T₄/T₂) = ω₂/ω₄.
  Instantaneous centres (Kennedy's theorem): I₂₄ = the intersection of the line AB with the frame line O₂O₄ (the x-axis); I₁₃ = the intersection of the lines O₂A and O₄B. At I₂₄: ω₄/ω₂ = (x_I − 0)/(x_I − d). Four links have 6 instantaneous centres.
  Coupler point P fixed to the coupler: P = A + (u·b)·ê + v·n̂ with ê = (B − A)/b and n̂ = (−ê_y, ê_x); u is a FRACTION of b along AB from A, v a perpendicular offset in length units (left positive).
CLASSIFICATION (compare lengths exactly: convert to integer tenths with Math.round(10·x) first, because the slider values are multiples of 0.1)
  s = shortest, l = longest, p and q the other two.
  Not assemblable when l ≥ s + p + q.
  s + l < p + q (Grashof) — the shortest link is then unique and decides:
       shortest = d (frame)   -> 'double-crank' (drag link): both a and c rotate fully
       shortest = a (input)   -> 'crank-rocker': a rotates fully, c rocks
       shortest = c (output)  -> 'rocker-crank': c could rotate fully while the input a only rocks (a crank-rocker driven from the rocker side)
       shortest = b (coupler) -> 'double-rocker' (Grashof double-rocker: the coupler can rotate fully, a and c rock)
  s + l = p + q -> 'change-point' (parallelogram, kite …): the limit case; the links can become collinear and the motion is indeterminate there.
  s + l > p + q -> 'triple-rocker' (non-Grashof): no link can rotate fully.
INPUT RANGE (the valid θ₂): k_max = (a² + d² − (b − c)²)/(2ad) and k_min = (a² + d² − (b + c)²)/(2ad); the linkage closes iff k_min ≤ cos θ₂ ≤ k_max. With θ_lo = acos(min(1, k_max)) and θ_hi = acos(max(−1, k_min)), in degrees:
       θ_lo = 0 and θ_hi = 180: full rotation, ranges [[0, 360]]
       θ_lo = 0 and θ_hi < 180: one range [−θ_hi, +θ_hi] (through θ₂ = 0)
       θ_lo > 0 and θ_hi = 180: one range [θ_lo, 360 − θ_lo] (through θ₂ = 180°)
       both restricted: two ranges [θ_lo, θ_hi] and [−θ_hi, −θ_lo]
  Use a small tolerance so that exact limit cases (change-point linkages) count as full rotation.
ROCKER LIMITS of the crank-rocker (a shortest): the output is at an extreme when crank and coupler are collinear:
       extended (O₂B = b + a): interior angle at O₄: β₁ = acos[(c² + d² − (b + a)²)/(2cd)]; crank angle ψ₁ = acos[((a + b)² + d² − c²)/(2(a + b)d)]
       folded (O₂B = b − a): β₂ = acos[(c² + d² − (b − a)²)/(2cd)]; crank angle 180° + ψ₂ with ψ₂ = acos[((b − a)² + d² − c²)/(2(b − a)d)]
       in the open configuration θ₄ = 180° − β; rocker swing = |β₁ − β₂|; the crank turns 180° + (ψ₂ − ψ₁) between the limits one way and 180° − (ψ₂ − ψ₁) the other way; time ratio (quick-return ratio) = larger/smaller.
  Over a full crank rotation the transmission-angle extremes occur at θ₂ = 0 (f = |d − a|) and θ₂ = 180° (f = d + a).

MODEL (angles in radians unless the name ends in Deg)
  classify(a, b, c, d) -> { assemblable, grashof: 'grashof' | 'change-point' | 'non-grashof' | null, type: 'crank-rocker' | 'double-crank' | 'double-rocker' | 'rocker-crank' | 'change-point' | 'triple-rocker' | 'none', shortest: 'a' | 'b' | 'c' | 'd' | null (null when not unique), s, l, sumSL, sumPQ, label (readable, e.g. 'Crank–rocker (Grashof: s + l < p + q)') }
  inputRange(a, b, c, d) -> { full, rangesDeg: [[lo, hi], …] }
  solve(a, b, c, d, theta2, mode) -> null when the linkage cannot close, else { A: [x, y], B: [x, y], theta3, theta4, mu, w3, w4 (ratios to ω₂; may be ±Infinity or NaN at a toggle), I24: [x, 0] or null (AB parallel to the frame), I13: [x, y] or null (O₂A parallel to O₄B) }
  couplerPoint(A, B, u, v) -> [x, y]     (u as a fraction of |AB|)
  rockerLimits(a, b, c, d) -> null unless the type is 'crank-rocker', else { theta4MinDeg, theta4MaxDeg, swingDeg, theta2ExtendedDeg, theta2FoldedDeg, forwardDeg, returnDeg, timeRatio } for the open configuration (forwardDeg ≥ returnDeg)
  transmissionRange(a, b, c, d) -> { minDeg, maxDeg } over all valid θ₂
  couplerCurve(a, b, c, d, mode, u, v, n = 360) -> an array of polylines (one per valid input range), each an array of [x, y]

CONTROLS
  Link sliders 1 … 10, step 0.1: "crank a (O₂A)" default 2 (key a); "coupler b (AB)" default 6 (key b); "rocker c (O₄B)" default 5 (key c); "frame d (O₂O₄)" default 6 (key d).
  Presets (select, key preset): "Crank–rocker (2, 6, 5, 6)" (default) · "Crank–rocker with poor transmission (2, 7, 5, 6)" · "Double-crank / drag link (4, 5, 6, 2)" · "Double-rocker, Grashof (5, 2, 6, 4)" · "Triple-rocker, non-Grashof (4, 6, 5, 8)" · "Parallelogram, change-point (3, 6, 3, 6)" · "Rocker–crank (5, 7, 2, 6)".
  θ₂ slider 0 … 360°, step 0.5, default 60 (key theta2). When the requested angle is not in a valid range, snap to the nearest valid angle and show the allowed range in the status line (data key status).
  Configuration: segmented "Open" / "Crossed" (key mode; default open).
  Play / Pause (key play) with a speed slider 10 … 180 °/s, default 60 (key speed): θ₂ advances continuously when the input can rotate fully; otherwise it sweeps back and forth inside the valid range that contains it, keeping the same configuration.
  Coupler point: u slider 0 … 1.5 (fraction of b), step 0.05, default 0.5 (key u); v slider −5 … 5, step 0.1, default 2 (key v).
  Checkboxes: "Coupler curve" (on, key showCoupler), "Paths of A and B" (on, key showPaths), "Instantaneous centres" (off, key showIC), "Transmission angle" (on, key showMu).
  Reset.

VIEWS
  (1) Mechanism canvas with equal scale. Fit the view once per geometry change (not per frame) to the bounding box of the whole motion: both pivots, the path of A, the path of B and the coupler curve, with a 12 % margin. Draw: the frame as a dashed grey line between two ground pivots (triangles with hatching, labelled O₂ and O₄); the path of A (circle or arc) and of B (arc), faint, when enabled; the coupler curve (orange) when enabled; the coupler as a faint filled triangle A–B–P so that P is visibly part of it, P as a dot labelled P; the links a (maroon), b (blue), c (green) as thick rounded bars with pin circles at A and B, labelled with their letters; the angle arc θ₂ at O₂; the transmission-angle arc μ at B with its value (red when poor) when enabled; the instantaneous centres I₂₄ and I₁₃ with thin construction lines when enabled (skip one that lies more than 3 view-widths away and say so in the status line). When the four lengths cannot be assembled show a clear message on the canvas instead: "These lengths cannot form a closed chain: the longest link must be shorter than the sum of the other three."
  (2) A plot below: θ₄ (green) and μ (red) in degrees versus θ₂ in degrees over the valid input range(s) for the current configuration, with a moving marker at the current θ₂, the band 40° … 140° lightly shaded and labelled "good transmission", and for the crank-rocker dashed markers at the two rocker limits.
  Legend.

READ-OUTS
  A Grashof line (data key grashof), e.g. "s + l = 2 + 6 = 8  <  p + q = 5 + 6 = 11  ⇒ Grashof" (with <, = or > as appropriate, the actual numbers, and which links are s and l).
  Tag [key type]: CRANK–ROCKER / DOUBLE-CRANK (DRAG LINK) / DOUBLE-ROCKER / ROCKER–CRANK / CHANGE-POINT / TRIPLE-ROCKER (NON-GRASHOF) / CANNOT BE ASSEMBLED.
  Stats: input range [range] ("full 360°" or e.g. "51.3° … 125.1° and −125.1° … −51.3°"); θ₃ [theta3]; θ₄ [theta4]; μ [mu] (bad when poor); μ minimum / maximum over the motion [muRange]; ω₄/ω₂ [w4]; ω₃/ω₂ [w3]; mechanical advantage T₄/T₂ [MA] ("∞ (toggle)" when |ω₄/ω₂| < 1e-6); rocker swing [swing] and time ratio [timeRatio] (crank-rocker only, otherwise "—"); the x-coordinate of I₂₄ [I24].

STATE: { a, b, c, d, theta2 (degrees), mode ('open' | 'crossed'), playing, speed, u, v, showCoupler, showPaths, showIC, showMu }

CHECK VALUES
  classify(2, 7, 5, 6): Grashof, crank-rocker, shortest a.  classify(2, 6, 5, 6): crank-rocker.  classify(4, 5, 6, 2): double-crank (shortest d).  classify(5, 2, 6, 4): double-rocker (shortest b).  classify(5, 7, 2, 6): rocker-crank (shortest c).  classify(4, 6, 5, 8): non-Grashof triple-rocker.  classify(3, 6, 3, 6): change-point.  classify(1, 1, 1, 5) and classify(1, 2, 3, 6): not assemblable.  classify(2.1, 4.2, 3.3, 3): 2.1 + 4.2 = 6.3 = 3.3 + 3 -> change-point (a floating-point trap: it must not be mis-classified).
  inputRange(2, 7, 5, 6): full.  inputRange(5, 2, 6, 4): two ranges [51.318°, 125.100°] and [−125.100°, −51.318°].  inputRange(4, 6, 5, 8): one range [−129.838°, 129.838°].
  solve(2, 7, 5, 6, 90°, +1): A = (0, 2); B = (6.3297, 4.9891); θ₃ = 25.278°; θ₄ = 86.219°; μ = 60.941°; ω₄/ω₂ = 0.41379; ω₃/ω₂ = 0.02155.
  rockerLimits(2, 7, 5, 6): θ₄ from 70.529° to 126.870°; swing 56.341°; crank at 31.586° (extended) and 233.130° (folded); forward 201.544°, return 158.456°; time ratio 1.2719.
  transmissionRange(2, 7, 5, 6) = 34.048° … 81.787° (poor);  transmissionRange(2, 6, 5, 6) = 41.410° … 92.866°.
  Parallelogram (3, 6, 3, 6), open: θ₄ = θ₂ and θ₃ = 0 for every θ₂ in (0°, 180°); ω₄/ω₂ = 1.
  Invariants (500 random assemblable linkages, random valid θ₂, both configurations): |A| = a, |B − A| = b, |B − O₄| = c to 1e-9; loop closure a·e^(jθ₂) + b·e^(jθ₃) = d + c·e^(jθ₄); w3 and w4 equal central finite differences of θ₃(θ₂) and θ₄(θ₂) (1e-5 relative, away from toggles); μ equals the angle between the vectors B→A and B→O₄; I₂₄ satisfies ω₄/ω₂ = x_I/(x_I − d); inputRange agrees with a brute-force scan of the solvable θ₂ (0.01° steps); classify agrees with brute force (the input "rotates fully" iff the linkage is solvable at every θ₂; test the output likewise by swapping a and c); rockerLimits agrees with the numerical minimum and maximum of θ₄ over a full crank turn.

HINTS (use these)
  1. Grashof test: s + l ≤ p + q. With the default lengths the shortest link is the crank → crank-rocker. Choose the drag-link preset (the shortest link is the frame) → both cranks rotate. Make the coupler the shortest → double-rocker.
  2. Triple-rocker preset: s + l > p + q — nothing can rotate fully; press Play and watch the input swing between its two toggle positions.
  3. Watch μ: the "poor transmission" preset dips to 34° — below about 40° the coupler pushes the rocker badly.
  4. Crank-rocker: at the rocker's two extreme positions the crank and the coupler are collinear; the two strokes take different crank angles, so one stroke is quicker (time ratio > 1).
  5. Turn on Instantaneous centres: I₂₄ lies where the coupler line meets the frame line, and ω₄/ω₂ = I₂₄O₂ / I₂₄O₄ (Kennedy's theorem).

STATES TO SCREENSHOT: default at θ₂ = 60°; the poor-transmission preset at θ₂ = 0°; drag link at 120°; double-rocker; triple-rocker at a toggle limit; parallelogram; the crossed configuration of the default; instantaneous centres on; the not-assemblable message (lengths 1, 1, 1, 5 via setState); 390-wide default.
````

## 3. Framework agents (already run)

The framework is the shared code every simulation stands on: `notes/sims/core.js`, `core.css`, the test harness and the page build.

### 3.1 The contract (in both framework prompts)

````text
================================================================
THE CONTRACT
================================================================
Twelve simulations will be written against this API by twelve engineers working in parallel who cannot ask questions. Implement it EXACTLY: same names, same argument shapes, same return shapes, same behaviour. If something in the contract is impossible or clearly wrong, implement the closest correct thing, document it in the README, and list it under contractDeviations in your final answer.

The 12 simulation ids and titles (fixed):
  eigen-2x2            Eigenvalues and eigenvectors of a 2×2 matrix
  rlc-transient        RC / RL / RLC transient response
  rotation-transforms  3D rotations and homogeneous transforms
  two-link-arm         2-link planar arm: forward and inverse kinematics
  second-order-step    Second-order step response and pole location
  bode-plot            Bode plot builder with gain and phase margins
  root-locus           Root locus explorer
  pid-tuning           PID tuning lab
  mohr-circle          Mohr's circle for plane stress
  beam-sfd-bmd         Beam: shear force, bending moment and deflection
  four-bar             Four-bar linkage and Grashof's law
  vibration-sdof       Single-DOF vibration: free, forced and isolation

Each simulation will be one file notes/sims/<id>.js of this shape:
  (function () {
    'use strict';
    const RASim = globalThis.RASim;
    const model = { /* pure functions, no DOM */ };
    RASim.register('<id>', { title: '…', model: model, mount: function (api) { /* build DOM in api.root */ return { getState: …, setState: … }; } });
  })();

----------------------------------------------------------------
PART 1 — notes/sims/core.js  (defines the global object RASim)
----------------------------------------------------------------
core.js is a classic script (no ES modules, no imports). It must ALSO load in Node for model tests: at load time it must not touch window or document. It ends with globalThis.RASim = RASim; and, when 'module' is defined, module.exports = RASim. In a browser it arranges for RASim.mountAll() to run once at DOMContentLoaded (or immediately if the document has already loaded).

1.1 Registry and lifecycle
- RASim.sims = {}        id -> definition
- RASim.instances = {}   id -> whatever mount() returned (or {} if it returned nothing)
- RASim.register(id, def): def = { title: string, model?: object, mount: function (api) }. Stores def in RASim.sims[id]. Throws on a duplicate id. Works in Node.
- A placeholder in the page looks like: <div class="sim" data-sim="ID"></div>
- RASim.mountAll(): for every placeholder whose ID is registered: set its id attribute to 'sim-' + ID, build the panel chrome inside it, and arrange for def.mount(api) to be called exactly once. A placeholder whose ID is NOT registered stays empty and hidden (CSS .sim:empty { display: none }) and produces one console.warn (never console.error). Calling mountAll() twice must not double-mount.
- Panel chrome built by the core inside the placeholder:
    <div class="sim-head"><span class="sim-badge">Interactive</span><span class="sim-title">TITLE</span><button type="button" class="sim-toggle" aria-expanded="true">Hide</button></div>
    <div class="sim-body"></div>
  The toggle collapses / expands .sim-body (button text switches Hide / Show; class 'sim-collapsed' on the .sim element; aria-expanded kept in sync). Expanding again must make canvases redraw correctly.
- Lazy mounting: def.mount is called the first time the placeholder comes within 400px of the viewport (IntersectionObserver, rootMargin '400px'). If window.RASIM_EAGER === true, or IntersectionObserver does not exist, every sim is mounted immediately inside mountAll().
- Error isolation: mount is wrapped in try/catch. On an exception: console.error the error, and .sim-body shows <p class="sim-error">This simulation failed to load.</p>. Other simulations are unaffected.
- Simulation index: if the page has an element with attribute data-sim-index, mountAll() fills it with <h2 class="sim-index-title">Interactive simulations</h2> and an <ol> of links (href '#sim-ID', text = title) for every placeholder that has a registered sim, in document order. With no registered sims the element stays empty and hidden.
- RASim.css(key, cssText): injects <style data-rasim-css="key"> into <head> once per key (a second call with the same key is a no-op). For the rare sim that needs custom CSS (authors must prefix every selector with .sim[data-sim="ID"]).

1.2 The api object passed to mount(api)
- api.id      the simulation id
- api.root    the .sim-body element (empty). The sim builds ALL of its DOM inside it.
- api.panel   the outer .sim element
- api.animate(fn) -> controller { start(), stop(), toggle(), running } (running is a boolean getter). While running, fn(dt, t) is called once per animation frame: dt = seconds since the previous delivered frame, clamped to at most 0.05; t = total seconds delivered so far (sum of the dts). The loop is NOT started automatically. Frames are delivered only while ALL hold: the controller is started, the panel intersects the viewport, document.visibilityState is 'visible', and the panel is not collapsed. When delivery resumes after any pause, dt restarts small (no time jump). A sim may create several controllers. A stopped controller must not keep a requestAnimationFrame loop alive.
- The value returned by mount(api) is stored in RASim.instances[id]. Authors return { getState(), setState(partialState) } so tests can drive the simulation.

1.3 Formatting helpers (all return strings; negative numbers use the real minus sign U+2212, never the ASCII hyphen)
- RASim.fmt(v, sig = 4): NaN -> '—'; +Infinity -> '∞'; −Infinity -> '−∞'; |v| < 1e-12 -> '0'. Otherwise round to sig significant digits and strip trailing zeros (2.500 -> '2.5', 3.000 -> '3', 1234.5 with sig 4 -> '1235' hint: 1234.5 rounds to 1235 or 1234 depending on binary representation, either is acceptable). Plain decimals when 1e-4 <= |v| < 1e7; otherwise scientific as '1.2×10⁻⁵' (Unicode superscript digits and superscript minus). Never returns '−0'.
- RASim.fmtFixed(v, decimals = 2): fixed decimals with the same special cases; never '−0.00' (returns '0.00').
- RASim.fmtComplex(z, sig = 4): z = { re, im }. Purely real -> fmt(re). Otherwise 're + j im' or 're − j im' (spaces around the sign, engineering j directly before the magnitude), e.g. '−1 + j1.732'. Purely imaginary -> 'j1.732' or '−j1.732'.
- RASim.fmtPoly(coeffs, variable = 's', sig = 4): HTML string for a polynomial with real coefficients, highest power first: [1, 3, 2, 0] -> 's<sup>3</sup> + 3s<sup>2</sup> + 2s'. Skips zero terms; a coefficient of 1 is omitted except for the constant term; negative terms are joined with ' − ' (a leading negative term starts with '−'); all-zero -> '0'.

1.4 Colours
- RASim.colors = { ink:'#1a1a1a', muted:'#666666', faint:'#9a9a9a', grid:'#e4e0d6', axis:'#8a8a8a', panel:'#fbf8f0', fill:'#f3ead2', maroon:'#6b1d1d', blue:'#1f4e79', green:'#2e6b30', orange:'#c2571a', purple:'#6a3d9a', red:'#c0392b', teal:'#0f766e', gold:'#b8860b', lightBlue:'#9cc3e6', lightRed:'#f2c1bb', lightGreen:'#bfe0c0' }
- RASim.alpha(hex, a) -> 'rgba(r, g, b, a)' string for a '#rrggbb' colour.

1.5 RASim.math  (pure functions; complex numbers are plain objects { re, im }; wherever a complex argument is expected a plain number is also accepted and treated as real)
- cx(re, im = 0); cadd(a, b); csub(a, b); cmul(a, b); cdiv(a, b); cscale(a, k); cconj(a); cabs(a); carg(a) (radians, atan2); cexp(a); csqrt(a) (principal root).
- clamp(v, lo, hi); rad(deg); deg(rad); wrapPi(a) -> (−π, π]; wrapDeg(a) -> (−180, 180].
- linspace(a, b, n) -> n values including both ends (n >= 2). logspace(a, b, n) -> n values from a to b (actual values, both > 0), equally spaced in log10.
- niceCeil(v): the smallest number of the form {1, 2, 2.5, 5, 10} × 10^k that is >= v (v > 0). niceTicks(min, max, target = 6) -> ascending array of 'nice' tick values lying inside [min, max].
- Polynomials are arrays of real coefficients, highest power first.
    polyTrim(p): removes leading (near-)zero coefficients; never returns an empty array ([0] at minimum).
    polyAdd(a, b); polySub(a, b); polyMul(a, b); polyScale(p, k); polyDeriv(p).
    polyEval(p, s) -> { re, im } (s number or { re, im }).
    polyFromRoots(roots): roots are numbers or { re, im }; complex roots must come in conjugate pairs; returns the MONIC real polynomial; [] -> [1].
    polyRoots(p) -> array of { re, im }; length = degree after trimming (degree 0 -> []). Must be robust for degree <= 10 including repeated roots and roots at the origin: Durand–Kerner or Aberth iteration from non-symmetric complex starting points on a circle scaled to the root bound, then a few Newton polishing steps per root, then (a) snap |im| < 1e-9 × max(1, |root|) to exactly 0, (b) make conjugate pairs exactly conjugate, (c) sort by descending real part, then descending imaginary part. Accuracy targets: simple roots to 1e-9 relative, a triple root to 1e-3.
- Transfer functions (num and den are polynomial arrays):
    tfEval(num, den, s) -> { re, im } (s number or { re, im }).
    tfStep(num, den, tEnd, n = 400) -> { t: number[], y: number[] } with n + 1 samples from t = 0 to tEnd: the unit-step response from zero initial conditions. Requires deg(num) <= deg(den) (throw a clear Error otherwise). Must be exact up to floating point for ANY pole locations (stable, unstable, at the origin, repeated, stiff): build the controllable-canonical state space (A, B, C, D), discretise the step input exactly using the matrix exponential of the augmented matrix [[A, B], [0, 0]] × dt, then iterate. y(0) equals the direct feed-through D, e.g. (s + 2)/(s + 1) starts at 1 and ends at 2. A pure gain (degree 0 denominator) returns the constant num/den.
    expm(M): matrix exponential of a small dense square matrix (array of arrays), scaling-and-squaring with a Padé or Taylor core; accurate to about 1e-12 relative for norms up to 1e3.
- rk4(f, t, y, h): one classical Runge–Kutta step for y' = f(t, y); y and the result are arrays.
- bisect(f, lo, hi, tol = 1e-12, maxIter = 200): root of f in [lo, hi] given a sign change; if f(lo) or f(hi) is exactly 0 return that end.
- zeroCrossings(f, a, b, n = 400, opts = { log: false }): sample f at n + 1 points (log-spaced when opts.log) and return the ascending array of x where f changes sign, each refined with bisect; an exact zero at a sample point is reported once; non-finite samples break the scan (no crossing reported across them).
- unwrapDeg(arr): returns a copy of an angle sequence in degrees with ±360° jumps removed.

1.6 RASim.ui  (DOM builders. Each appends to 'parent' and returns what is stated. Labels are trusted HTML strings so authors can write θ<sub>1</sub>. Every control gets attribute data-control="KEY" on its main input element, where KEY = opts.key if given, otherwise a slug of the label text.)
- el(tag, props = {}, parent = null): creates an element; props may contain class, text, html, title, style (object), attrs (object), on (object of event listeners). Returns the element.
- split(parent, { ratio = 1.5 } = {}) -> { el, main, side }: two-column responsive layout (grid: main minmax(0, <ratio>fr), side minmax(230px, 1fr)) that collapses to ONE column (main first) when the PANEL is narrower than 720px. It must depend on the panel width, not the window width (CSS container query or ResizeObserver).
- row(parent) -> el: a flex row that wraps (gap 8px 14px, items centred).
- group(parent, title) -> el: a titled box for related controls (small uppercase heading, thin border); returns the inner container.
- slider(parent, { label, min, max, step = 1, value, unit = '', scale = 'linear', decimals, key, onInput }) -> { el, get(), set(v), setRange(min, max), setDisabled(flag), setLabel(html) }
    Renders: label, <input type="range">, a number box <input type="number">, and the unit. The two inputs stay in sync.
    scale 'linear': the range input uses min / max / step. A value typed into the number box is clamped to [min, max] but NOT snapped to step (exact values can be entered).
    scale 'log' (min > 0): the track has 400 positions equally spaced in log10 between min and max; values coming from the track are rounded to 3 significant digits; typed values are only clamped.
    decimals: digits after the decimal point shown in the number box; default = the number of decimals in step for linear sliders, and 3 significant digits for log sliders.
    onInput(value) fires on every USER change (track: 'input' event; number box: 'change' event, which includes pressing Enter). set(v) clamps, updates both inputs and does NOT fire onInput. get() returns the current numeric value (the exact typed value when one was typed). setRange keeps the current value if it is still inside, otherwise clamps it (without firing onInput).
- number(parent, { label, value, min, max, step, unit = '', decimals, key, onInput }) -> { el, get(), set(v), setDisabled(flag) }: a number box only.
- select(parent, { label, options, value, key, onChange }) -> { el, get(), set(v), setOptions(options) }; options = [{ value, label }] with string values.
- segmented(parent, { label, options, value, key, onChange }) -> { el, get(), set(v) }: a row of joined toggle buttons, exactly one active (class 'active', aria-pressed kept in sync); each button carries data-value.
- checkbox(parent, { label, checked = false, key, onChange }) -> { el, get(), set(flag) }.
- button(parent, { label, onClick, title, primary = false, key }) -> the <button type="button"> element (attribute data-control="KEY").
- stats(parent) -> el: grid container for stat tiles (auto-fill, minimum tile width 112px).
- stat(container, { label, key, title }) -> { el, set(html), setState(state) }: a tile with a small label and a prominent value. el has attribute data-stat="KEY"; the value element has class 'sim-stat-value'. state is '', 'ok', 'warn' or 'bad' (classes sim-ok / sim-warn / sim-bad).
- tag(parent, { text = '', state = '', key }) -> { el, set(text, state) }: a pill such as UNDERDAMPED; el has data-tag="KEY".
- matrix(parent, { label, key }) -> { el, set(rows, { decimals = 3, colColors, rowColors } = {}) }: renders a bracketed matrix (a <table> with a bracket look); rows is a 2D array whose entries are numbers (formatted with fmtFixed(decimals)) or strings (used as is); colColors / rowColors are optional arrays of CSS colours for the text of each column / row; el has data-matrix="KEY". set() may be called every animation frame: reuse the cells when the shape is unchanged.
- legend(parent, items) -> { el, set(items) }: items = [{ color, label, dash = false }], small line swatches with text in a wrapping row.
- note(parent, html) -> el: a small muted paragraph.
- hints(parent, items) -> el: <details class="sim-hints" open><summary>Try this</summary><ol><li>…</li></ol></details>; items are HTML strings.
Callbacks are invoked synchronously with the parsed value (number for sliders, string for select / segmented, boolean for checkbox).

1.7 RASim.canvas(parent, { aspect = 1.6, maxWidth = null, minHeight = 140, maxHeight = 560, key } = {}) -> cv
- Creates <canvas class="sim-canvas" data-canvas="KEY"> that fills the width of parent (capped at maxWidth and centred when capped). CSS height = clamp(width / aspect, minHeight, maxHeight). Backing store = CSS size × devicePixelRatio, with the context pre-scaled so authors draw in CSS pixels. Resizes are handled with a ResizeObserver (also re-check devicePixelRatio).
- cv.canvas, cv.ctx, cv.width, cv.height (current CSS-pixel size, getters).
- cv.draw = function (ctx, w, h) { … }   assigned by the sim.
- cv.redraw(): schedules ONE call of cv.draw on the next animation frame (coalesced). Before calling cv.draw the core resets the transform to the DPR scale and clears the canvas. The core calls redraw() itself after every resize. If the canvas currently has zero width (hidden or collapsed) the draw is skipped and happens when it becomes visible again.
- cv.drawNow(): the same, synchronously (for use inside api.animate callbacks).
- cv.setAspect(a): change the aspect ratio later (triggers a resize and redraw).
- cv.onDrag({ hit(x, y), move(x, y, token), end(token), cursor(x, y) }): pointer interaction in canvas CSS-pixel coordinates. On pointerdown, hit(x, y) is called; a truthy return value is the drag token: the pointer is captured (setPointerCapture) and move(x, y, token) is called for every pointermove (coordinates may leave the canvas) until pointerup / pointercancel, then end(token) (optional). cursor(x, y) (optional) is called on hover and returns a CSS cursor string or null. Touch: dragging a handle with a finger must NOT scroll the page, and touching the canvas where hit() returns falsy should still let the page scroll. Preferred implementation: a non-passive touchstart listener that calls preventDefault only when hit() succeeds. If you cannot make that reliable in Chromium touch emulation, fall back to touch-action: none on canvases that registered onDrag, and say so in the README and in contractDeviations.
- cv.onHover({ move(x, y), leave() }): optional mouse hover tracking for read-out cursors.

1.8 RASim.plot(ctx, opts) -> p   (a light 2D plotting helper. Authors create one inside cv.draw on every frame; it keeps no state between frames.)
  opts = { box: { x, y, w, h }, margin: { left = 50, right = 14, top = 12, bottom = 38 }, xmin, xmax, ymin, ymax, xlog = false, ylog = false, equal = false, xlabel = '', ylabel = '', title = '' }
  box is the OUTER rectangle in canvas CSS pixels; tick labels, axis labels and the title are drawn inside the margins; when title is set the default top margin is 28. Partial margin objects are merged with the defaults. p.rect = { x, y, w, h } is the inner data rectangle. equal: true expands the x-range OR the y-range symmetrically about its centre so that one data unit has the same pixel length on both axes (circles stay circles). Log axes need positive limits. A degenerate range (min == max) is widened slightly instead of dividing by zero.
  - p.xmin, p.xmax, p.ymin, p.ymax (after any 'equal' adjustment); p.X(x), p.Y(y) data -> pixel; p.invX(px), p.invY(py) pixel -> data; p.contains(px, py).
  - p.axes({ grid = true, zero = true, frame = true, xTicks, yTicks, xFormat, yFormat } = {}): white background, grid, tick labels (11px sans-serif in RASim.colors.muted), axis labels (ylabel rotated), title; zero: true draws darker lines at x = 0 and y = 0 when they are inside the range. Log axes: major ticks at decades labelled 0.01, 0.1, 1, 10, 100 …, minor grid lines at 2…9. xTicks / yTicks override the tick positions; xFormat / yFormat are functions value -> string.
  - Everything below is clipped to p.rect unless its style has clip: false. Common style fields: color, width (line width, default 2), dash (array or null), fill.
      p.line(xs, ys, style): polyline; non-finite points break the path.
      p.fn(f, style & { n = 400 }): plots y = f(x) over the x-range (log-spaced samples on a log x-axis).
      p.area(xs, ys, { fill, base = 0 }): filled area between the curve and y = base.
      p.band(y0, y1, { fill }); p.vband(x0, x1, { fill }): filled horizontal / vertical strips.
      p.hline(y, style); p.vline(x, style); p.segment(x1, y1, x2, y2, style); p.arrow(x1, y1, x2, y2, style & { head = 8 }).
      p.point(x, y, { color, r = 4, fill = color, stroke = '#fff' }); p.cross(x, y, { color, size = 6, width = 2 }) (the × used for poles); p.ring(x, y, { color, r = 5, width = 2 }) (the ○ used for zeros).
      p.circle(cx, cy, rData, style): a circle whose radius is in data units (an ellipse when the axes are not equal).
      p.arc(cx, cy, rPx, a0, a1, style & { ccw = true }): an angle arc centred at data point (cx, cy) with pixel radius rPx from angle a0 to a1, radians, measured counter-clockwise from +x as the viewer sees it (y up).
      p.text(x, y, str, { color = ink, font = '12px sans-serif', align = 'left', baseline = 'middle', dx = 0, dy = 0, bg = null, clip = false }): text anchored at a data point plus a pixel offset; bg draws a rounded filled rectangle behind the text.

1.9 RASim.draw  (helpers for free-form canvases; all coordinates in CSS pixels)
  arrow(ctx, x1, y1, x2, y2, { color, width = 2, head = 8, dash })
  text(ctx, str, x, y, { color, font = '12px sans-serif', align = 'left', baseline = 'middle', bg = null })
  spring(ctx, x1, y1, x2, y2, { coils = 8, amplitude = 8, color, width = 2 })   a zig-zag spring between two points with short straight leads at both ends
  hatch(ctx, x1, y1, x2, y2, { side = 1, length = 8, spacing = 8, color })   a ground line with hatching on one side
  angleArc(ctx, cx, cy, r, a0, a1, { color, width = 1.5, label, ccw = true, font })   an arc from a0 to a1, radians, measured counter-clockwise from +x as the viewer sees it (the helper flips for the canvas y-down axis), with an optional label at mid-angle just outside the arc

----------------------------------------------------------------
PART 2 — notes/sims/core.css
----------------------------------------------------------------
Every selector starts with .sim or .sim-index (no bare element selectors, nothing that could restyle the notes). Look and feel, matching the notes: UI font "DejaVu Sans", Arial, sans-serif at 13px inside panels; panel background #fbf8f0, 1px border #d9cfae, 4px maroon (#6b1d1d) left border, 6px radius, 16px vertical margin; header strip #f3ead2 with a small maroon INTERACTIVE badge (white 10px letter-spaced uppercase text), bold title in #1f4e79, and the Hide/Show toggle at the right; body padding 12px. Controls: range inputs use accent-color #1f4e79 and shrink on narrow panels; the number box is about 5.2em wide; buttons have a thin border, 4px radius and 6px 10px padding, .primary is filled #1f4e79 with white text; segmented buttons are joined and the active one is filled #1f4e79 with white text. Stat tiles: white background, 1px #e4e0d6 border, label 10.5px uppercase muted, value 15px bold; states ok (#2e6b30), warn (#b8860b), bad (#c0392b) colour the value and tint the tile. Matrix: monospace 12.5px, right-aligned cells, bracket look (left and right borders with short top / bottom ticks). Canvas: display block, white background, 1px #e4e0d6 border, 4px radius, max-width 100%. Hints: 12.5px muted text with a bold summary. Interactive targets at least 28px high. Everything must fit a 360px-wide phone with NO horizontal scrolling (controls wrap, sliders shrink, stat grid reflows).
Sim index (.sim-index): a compact boxed list in the same palette.
Print: @media print { .sim, .sim-index, .screen-only { display: none !important; } }

----------------------------------------------------------------
PART 3 — build integration
----------------------------------------------------------------
3.1 Move the CSS template string out of notes/build.js into a new file notes/page.css without changing what applies to print. Then ADD, strictly inside @media screen { … }, a comfortable reading layout for browsers: body max-width 920px, centred, padding 16px 18px 80px, font-size 15px; tables 14px; on screens up to 640px wide: tables become horizontally scrollable blocks and the table of contents becomes one column; .cover gets less top padding (12vh); h1 margin-top 56px (no effect on print); headings and .sim get scroll-margin-top 12px; html { scroll-behavior: smooth }. Do not add or change any rule that affects print.
3.2 notes/build.js must:
  - put notes/page.css followed by notes/sims/core.css into the page's single <style> element;
  - add <meta name="viewport" content="width=device-width, initial-scale=1"> to <head>;
  - insert <nav class="sim-index" data-sim-index></nav> immediately after the table-of-contents nav;
  - append at the end of <body>: one <script> element containing notes/sims/core.js, then one SEPARATE <script> element per simulation file, where the simulation files are all files matching notes/sims/*.js except core.js and names starting with '_' (sorted by name; the test/ subfolder is never included). Separate elements so a syntax error in one simulation cannot break the others. Escape any '</script' sequence inside inlined JS (replace with '<\/script') so it cannot terminate the element;
  - keep generating the table of contents exactly as before (placeholders must not appear in it);
  - accept 'node notes/build.js --html-only' to skip the PDF step; without the flag it writes the PDF as before.
3.3 Insert these 12 placeholder lines into the notes sources. Each goes on its own line IMMEDIATELY BEFORE the quoted heading line (so it sits after the list or table that ends the previous topic), except the last entry of 04_electrical.html, which goes at the very end of that file:
  notes/src/01_math.html         <div class="sim" data-sim="eigen-2x2"></div>            before   <h2>2. Calculus</h2>
  notes/src/02_mechatronics.html <div class="sim" data-sim="rlc-transient"></div>        before   <h3>Sinusoidal steady state</h3>
  notes/src/03_robotics.html     <div class="sim" data-sim="rotation-transforms"></div>  before   <h2>5. Forward Kinematics</h2>
  notes/src/03_robotics.html     <div class="sim" data-sim="two-link-arm"></div>         before   <h2>6. Robot Applications and Performance</h2>
  notes/src/04_electrical.html   <div class="sim" data-sim="second-order-step"></div>    before   <h3>Steady-state error</h3>
  notes/src/04_electrical.html   <div class="sim" data-sim="bode-plot"></div>            before   <h3>Root locus</h3>
  notes/src/04_electrical.html   <div class="sim" data-sim="root-locus"></div>           before   <h3>Compensators</h3>
  notes/src/04_electrical.html   <div class="sim" data-sim="pid-tuning"></div>           at the very end of the file
  notes/src/05_mechanical.html   <div class="sim" data-sim="mohr-circle"></div>          before   <h3>Plane strain</h3>
  notes/src/05_mechanical.html   <div class="sim" data-sim="beam-sfd-bmd"></div>         before   <h3>Bending and shear stresses in beams</h3>
  notes/src/05_mechanical.html   <div class="sim" data-sim="four-bar"></div>             before   <h3>Displacement, velocity and acceleration analysis</h3>
  notes/src/05_mechanical.html   <div class="sim" data-sim="vibration-sdof"></div>       before   <h3>Resonance and critical speed of shafts</h3>
  Change nothing else in notes/src.
3.4 Proving the PDF is unchanged (mandatory). Before touching anything: copy the committed PDF with 'git show HEAD:notes/RA_GATE2027_Notes.pdf > /tmp/claude-0/-home-user-RA-GATE/9d489dee-fbb1-53d8-91e1-ed21c31b514b/scratchpad/core/old.pdf'. After your changes run the full build 'node notes/build.js', then compare old vs new: (a) pdfinfo page count (must be 38 for both), (b) 'pdftotext -layout' output must be byte-identical, (c) render every page of both with 'pdftoppm -r 40 -png' and compare the images page by page (identical, or differences only in a handful of anti-aliasing pixels — report the numbers). If all three hold, restore the committed binary so git shows no change for the PDF: 'git checkout -- notes/RA_GATE2027_Notes.pdf' (this is the one allowed git write). If they do not hold, find and fix the cause; never leave a changed PDF behind.

----------------------------------------------------------------
PART 4 — test tooling (all under notes/sims/test/)
----------------------------------------------------------------
4.1 load.js — Node loader for model tests:
      module.exports = function load(id, file) — sets globalThis.RASim = require('../core.js') (once), requires the simulation file (default notes/sims/<id>.js, or the given path), returns RASim. Usage in a sim test: const RASim = require('./load.js')('two-link-arm'); const m = RASim.sims['two-link-arm'].model;
4.2 harness.js — browser harness for ONE simulation:
      CLI: node notes/sims/test/harness.js <id> --out <dir> [--widths 1100,390] [--file <path to the sim js>]
      It writes <dir>/<id>.html: a standalone page using the SAME styles as the real page (notes/page.css + notes/sims/core.css), body = <h2>Harness: <id></h2><div class="sim" data-sim="<id>"></div>, then a script setting window.RASIM_EAGER = true, then core.js, then the sim file (each in its own <script>).
      For each width it opens the page in Chromium (viewport width × 900), waits until RASim.instances[id] exists (5 s timeout -> error), waits 400 ms more, and records: console errors, uncaught page errors, console warnings, 'horizontal overflow' if document.documentElement.scrollWidth > window.innerWidth + 1, and any descendant of the panel whose right edge extends more than 1px beyond the panel's right edge (reported as warnings with a short selector). It saves a full-page screenshot <dir>/<id>-<width>.png, prints a JSON summary { id, ok, runs: [{ width, errors, warnings, screenshot }] } and exits 0 when there are no errors, 1 otherwise.
      It also exports helpers for custom interaction scripts: buildPage(id, outDir, file) -> html path; and async openSim(id, { width = 1100, height = 900, outDir, file, touch = false, deviceScaleFactor = 1 }) -> { browser, context, page, errors (live array of console / page errors), htmlPath } with the simulation already mounted. Guard the CLI with require.main === module.
4.3 demo-sim.js — id 'demo', title 'Core demo': a small simulation that exercises EVERY ui builder, a canvas with a draggable handle (onDrag) and hover read-out (onHover), RASim.plot with a linear plot, a log-x plot and an equal-axes plot showing every drawing method, RASim.draw helpers, api.animate with start/stop, and a tfStep curve. It returns { getState, setState }. It lives in the test folder so the build never includes it. Run it through the harness (use --file) at 1100 and 390 wide and LOOK at the screenshots; fix layout problems in the core until both look clean and professional.
4.4 core.test.js — plain Node script using require('assert'); prints a line per test group and exits non-zero on failure. Must cover at least:
      polyRoots: s³+6s²+11s+6 -> −1, −2, −3; s²+2s+5 -> −1 ± 2j; (s+1)³ (triple root, 1e-3); s(s+1)(s+2) (root at the origin); s⁴+1; a degree-8 polynomial built with polyFromRoots from known roots (round trip to 1e-7); leading zeros; ordering and exact conjugates.
      polyMul / polyAdd / polyDeriv / polyEval / polyFromRoots basic identities.
      tfStep against closed forms: 1/(s+1) (0.632121 at t = 1); ωn²/(s²+2ζωn s+ωn²) with ζ = 0.5, ωn = 2 against 1 − e^(−ζωn t)·sin(ωd t + acos ζ)/√(1−ζ²) at 50 times (1e-9); 1/s (ramp y = t); 1/s² (t²/2); (s+2)/(s+1) (starts at 1, ends at 2: y = 2 − e^(−t)); the stiff 1000/((s+1)(s+1000)); the unstable 1/(s−1) (e^t − 1); the repeated 1/(s+1)³; a pure gain; and that deg(num) > deg(den) throws.
      expm: zero matrix -> identity; [[0,1],[−1,0]]·t -> rotation (cos t, sin t); a nilpotent matrix; expm(A)·expm(−A) = I for a random 4×4.
      bisect, zeroCrossings (sin on [0.5, 10] -> π, 2π, 3π; log option), niceCeil, niceTicks, linspace, logspace, wrapPi, wrapDeg, unwrapDeg, complex helpers (cdiv, csqrt of −4 = 2j, cexp(jπ) = −1).
      fmt / fmtFixed / fmtComplex / fmtPoly: every special case listed in 1.3, including that no ASCII '-' ever appears in the output for negative inputs.
      register: duplicate id throws; a registered sim's model is reachable via RASim.sims[id].model.

----------------------------------------------------------------
PART 5 — notes/sims/README.md
----------------------------------------------------------------
The API reference the 12 authors will work from: every public function with its exact signature, argument and return shapes and one-line behaviour; the simulation file template; how to run the model test, the harness and custom Playwright interaction scripts (with a short copy-paste example using openSim that sets a slider through RASim.instances[id].setState and reads a stat tile via [data-stat]); layout advice (use ui.split for canvas + controls, ui.stats for read-outs, ui.hints for guided experiments, redraw via cv.redraw() on every state change); and the rules for authors (touch only their own two files; no external resources; use RASim.colors; use RASim.fmt for every number shown; animations only through api.animate; return getState/setState from mount). It must match the code exactly.
````

### 3.2 Build-core prompt

````text
You are a senior front-end engineer. Build the shared foundation ("core") for 12 interactive study simulations that will be embedded in a single-file web page of GATE 2027 Robotics & Automation study notes. You do NOT write any of the 12 simulations — only the core they stand on, the build integration, and the test tooling.

GROUND RULES (apply to everything you do)
- Repo: /home/user/RA_GATE. Work directly in the working tree. Do NOT run git commit, push, checkout, switch, stash, reset, restore or clean. (git status / git diff / git show are fine. The one allowed exception is stated in Part 3.4.)
- Temp files, scripts and screenshots go ONLY under /tmp/claude-0/-home-user-RA-GATE/9d489dee-fbb1-53d8-91e1-ed21c31b514b/scratchpad/core/ (create it). Do not leave stray files in the repo.
- No network: no CDNs, no npm install, no external fonts or libraries. Plain ES2019 JavaScript, Canvas 2D and DOM only. Node 22 and Playwright are installed: require('playwright') works from any directory and chromium.launch() needs no options. NEVER run 'playwright install'.
- The notes page must work as ONE self-contained offline HTML file.
- The printed PDF (notes/RA_GATE2027_Notes.pdf) is already at the print shop: its content must stay identical. Everything added here is screen-only.
- Use the Read tool on PNG screenshots to actually look at what you rendered. Do not claim something looks right without looking.

START by reading notes/build.js, skimming notes/src/*.html and opening the current notes/RA_GATE2027_Notes.html, and by saving the committed PDF as described in Part 3.4.

DELIVERABLES (create exactly these and no other repo files)
  A. notes/sims/core.js            (Part 1)
  B. notes/sims/core.css           (Part 2)
  C. notes/page.css + modified notes/build.js + the 12 placeholders in notes/src/*.html   (Part 3)
  D. notes/sims/test/load.js, harness.js, demo-sim.js, core.test.js   (Part 4)
  E. notes/sims/README.md          (Part 5)
  F. A rebuilt notes/RA_GATE2027_Notes.html (via the build), with the PDF proven unchanged and restored (Part 3.4)
````

*The contract (3.1) is inserted here.*

````text
================================================================
DEFINITION OF DONE — verify every item yourself before answering
================================================================
1. node notes/sims/test/core.test.js exits 0.
2. The demo sim runs through the harness at widths 1100 and 390 with zero errors and zero overflow warnings; you have looked at both screenshots and they are clean: no clipped text, no overlapping controls, crisp canvas, readable 11px+ labels.
3. In a Playwright script of your own: slider set/get, typed exact values, log slider, clamping, segmented, select, checkbox, stat keys, matrix updates, canvas drag with the mouse, hover, collapse/expand redraw, api.animate start/stop and pause when scrolled out of view, lazy mounting without RASIM_EAGER, error isolation (a sim whose mount throws; a sim file with a syntax error in its own script element) all behave as the contract says.
4. node notes/build.js --html-only works; the built page opens with ZERO console errors (12 console.warn lines about unregistered placeholders are expected at this stage), the table of contents is unchanged, the notes look good at 1280 and 390 wide (look at screenshots), and no placeholder is visible.
5. The PDF proof of Part 3.4 holds and 'git status --short' shows no change to notes/RA_GATE2027_Notes.pdf.
6. README matches the code.
Your final answer is the structured result. Be exact and honest: report anything not done or not verified under openIssues.
````

### 3.3 Review-core prompt

````text
You are an independent, sceptical reviewer. Another engineer has just built the shared foundation ("core") for 12 interactive study simulations embedded in a GATE 2027 Robotics & Automation notes page. Twelve engineers are about to build on it in parallel and cannot ask questions, so any defect in the core, the harness or the README multiplies by twelve. Your job is to FIND defects by actually exercising the code against the contract below, and to FIX every defect you find. Assume there are bugs. Do not trust the builder's claims — reproduce them.

GROUND RULES (apply to everything you do)
- Repo: /home/user/RA_GATE. Work directly in the working tree. Do NOT run git commit, push, checkout, switch, stash, reset, restore or clean. (git status / git diff / git show are fine. The one allowed exception is stated in Part 3.4.)
- Temp files, scripts and screenshots go ONLY under /tmp/claude-0/-home-user-RA-GATE/9d489dee-fbb1-53d8-91e1-ed21c31b514b/scratchpad/core/ (create it). Do not leave stray files in the repo.
- No network: no CDNs, no npm install, no external fonts or libraries. Plain ES2019 JavaScript, Canvas 2D and DOM only. Node 22 and Playwright are installed: require('playwright') works from any directory and chromium.launch() needs no options. NEVER run 'playwright install'.
- The notes page must work as ONE self-contained offline HTML file.
- The printed PDF (notes/RA_GATE2027_Notes.pdf) is already at the print shop: its content must stay identical. Everything added here is screen-only.
- Use the Read tool on PNG screenshots to actually look at what you rendered. Do not claim something looks right without looking.

The builder reported:
<<the structured report returned by the previous agent is inserted here as JSON>>
````

*The contract (3.1) is inserted here.*

````text
================================================================
WHAT TO DO
================================================================
1. Read notes/sims/core.js, core.css, README.md, notes/build.js, notes/page.css and everything under notes/sims/test/ in full.
2. Contract conformance, mechanically: write a script that checks that every function named in Parts 1.1–1.9 exists with the documented arity and return shape (call each one). List every mismatch.
3. Maths, adversarially — write your own tests (in /tmp/claude-0/-home-user-RA-GATE/9d489dee-fbb1-53d8-91e1-ed21c31b514b/scratchpad/core/review/), independent of core.test.js:
   - polyRoots: 300 random polynomials of degree 1–8 built from random real roots and conjugate pairs (including clustered and repeated roots, roots at 0, coefficients spanning 1e-3…1e3): residual and round-trip checks; verify ordering, snapping and exact conjugates.
   - tfStep: compare with an independent fine-step RK4 integration for 100 random stable and unstable systems of order 1–5, plus integrators, double integrators, direct feed-through, stiff (pole ratio 1e4) and pole-zero-cancelling cases; check y(0) = D and the final value for stable systems.
   - expm: against a high-order Taylor series with scaling for random matrices.
   - zeroCrossings / bisect edge cases; fmt family on a grid of magnitudes and signs (no ASCII minus, no '−0', trailing zeros stripped, scientific thresholds); fmtPoly on tricky inputs ([−1, 0, 1], [1], [0, 0], [2, −1]).
4. Browser behaviour with Playwright (use the harness openSim and the demo sim, and write extra throw-away sims in the scratch folder when needed): every ui builder (including typed values on linear and log sliders, setRange, setDisabled, data-control / data-stat / data-matrix / data-tag attributes), split collapsing at panel width 720px, canvas crispness at deviceScaleFactor 2 (backing store size), resize, collapse / expand redraw, onDrag with mouse AND with touch emulation (page must not scroll while dragging a handle; must scroll when touching elsewhere on the canvas), onHover, api.animate semantics (dt clamp, pause off-screen, pause when collapsed, no rAF loop left running when stopped), lazy mounting, error isolation (throwing mount; syntax error in one script element), the sim index, RASim.css idempotence.
5. Plot helper: render a page that uses every p.* method on linear, log-x and equal axes and LOOK at the screenshot (Read tool): ticks sensible, labels not overlapping or clipped, log minor grid, equal axes really equal (measure a circle), clipping works, arc direction is counter-clockwise for increasing angle, arrow heads correct.
6. Page integration: run 'node notes/build.js --html-only'; open the built page at 1280 and 390 wide; zero console errors; the notes look good (screenshots: top of page, contents, one table-heavy section, on the phone width no horizontal scrolling of the page itself); the placeholders are in exactly the 12 specified positions (verify with grep, and that nothing else in notes/src changed: git diff); the table of contents is unchanged versus 'git show HEAD:notes/RA_GATE2027_Notes.html'.
7. PDF: repeat the Part 3.4 proof yourself from scratch (full build, compare, restore). 'git status --short' must not list the PDF.
8. README: every signature and behaviour statement must match the code; the copy-paste examples must actually run.
9. Fix everything you find, directly in the files (keep the contract's names and shapes; if the contract itself is wrong somewhere, choose the correct behaviour, document it in the README and report it under contractDeviations). Add regression tests for real bugs to notes/sims/test/core.test.js. Re-run everything after fixing.

Your final answer is the structured result: list every defect with severity and whether it is fixed; contractDeviations must list every remaining difference between code and contract (including any the builder reported that still hold). Be exact and honest.
````

## 4. Integration agent (planned)

Runs once, after simulation 12 has been verified. It has not been sent yet, so the wording may still change.

````text
You are the integration engineer for the interactive GATE 2027 Robotics & Automation notes page. The shared simulation framework and all 12 simulations have been built and individually verified, one at a time. Nobody has yet seen all 12 running together inside the real page. Your job: build the full page, prove that everything works together at desktop and phone widths, prove that the print PDF is unchanged, and fix whatever integration problems you find.

GROUND RULES
- Repo: /home/user/RA_GATE. Work in the working tree. No git commit, push, checkout, switch, stash, reset or clean. The one allowed git write is restoring the PDF as described in step 2.
- Temp files, scripts and screenshots go ONLY under /tmp/claude-0/-home-user-RA-GATE/9d489dee-fbb1-53d8-91e1-ed21c31b514b/scratchpad/integration/ (create it). Leave no stray files in the repo.
- No network: no CDNs, no npm install, no external libraries, fonts or images. Node 22 and Playwright are installed: require('playwright') works from any directory and chromium.launch() needs no options. NEVER run 'playwright install'.
- The page uses html { scroll-behavior: smooth }: in Playwright scripts scroll with behavior 'instant' or wait about a second.
- You may edit notes/sims/core.js, core.css, README.md, notes/build.js and notes/page.css, and — for integration defects only — a simulation's own two files (notes/sims/<id>.js and notes/sims/test/<id>.test.js). After any change to a simulation re-run that simulation's Node test and harness; after any change to the core re-run everything in step 1. Do not change notes/src/*.html except to correct a misplaced placeholder.
- Use the Read tool on PNG screenshots to actually LOOK at what was rendered. Never claim something looks right without looking.
- Read notes/sims/README.md first.

THE 12 SIMULATIONS (id — title — the notes heading that must follow its placeholder)
  eigen-2x2 — Eigenvalues and eigenvectors of a 2×2 matrix — notes/src/01_math.html, before <h2>2. Calculus</h2>
  rlc-transient — RC / RL / RLC transient response — notes/src/02_mechatronics.html, before <h3>Sinusoidal steady state</h3>
  rotation-transforms — 3D rotations and homogeneous transforms — notes/src/03_robotics.html, before <h2>5. Forward Kinematics</h2>
  two-link-arm — 2-link planar arm: forward and inverse kinematics — notes/src/03_robotics.html, before <h2>6. Robot Applications and Performance</h2>
  second-order-step — Second-order step response and pole location — notes/src/04_electrical.html, before <h3>Steady-state error</h3>
  bode-plot — Bode plot builder with gain and phase margins — notes/src/04_electrical.html, before <h3>Root locus</h3>
  root-locus — Root locus explorer — notes/src/04_electrical.html, before <h3>Compensators</h3>
  pid-tuning — PID tuning lab — notes/src/04_electrical.html, at the very end of the file
  mohr-circle — Mohr's circle for plane stress — notes/src/05_mechanical.html, before <h3>Plane strain</h3>
  beam-sfd-bmd — Beam: shear force, bending moment and deflection — notes/src/05_mechanical.html, before <h3>Bending and shear stresses in beams</h3>
  four-bar — Four-bar linkage and Grashof's law — notes/src/05_mechanical.html, before <h3>Displacement, velocity and acceleration analysis</h3>
  vibration-sdof — Single-DOF vibration: free, forced and isolation — notes/src/05_mechanical.html, before <h3>Resonance and critical speed of shafts</h3>

ISSUES REPORTED BY THE SIMULATION AGENTS (core issues, specification deviations and open issues, per simulation)
<<the collected reports of the 24 simulation agents are inserted here as JSON>>

STEPS (work in the scratch folder; report numbers, not impressions)
1. Baseline. Run node notes/sims/test/core.test.js and node notes/sims/test/<id>.test.js for all 12 ids: every one must exit 0. Run the harness for each id (node notes/sims/test/harness.js <id> --out <scratch>/h/<id>): zero errors and zero warnings at 1100 and 390.
2. PDF proof — the PDF is at the print shop and must not change. Save the committed file (git show HEAD:notes/RA_GATE2027_Notes.pdf > old.pdf), run the full build (node notes/build.js), then compare old and new: pdfinfo page count (38 for both); pdftotext -layout output byte-identical; every page rendered with pdftoppm -r 40 -png identical. If all three hold, restore the committed binary with git checkout -- notes/RA_GATE2027_Notes.pdf so that git shows no change. If anything differs, find and fix the cause; never leave a changed PDF behind.
3. The real page (notes/RA_GATE2027_Notes.html) at 1280×900 and at 390×844, plus one pass at deviceScaleFactor 2, loaded WITHOUT RASIM_EAGER:
   - zero console errors and zero console warnings (all 12 placeholders now have simulations);
   - lazy mounting: simulations far below the viewport are not mounted at load, and each one mounts when scrolled near;
   - the "Interactive simulations" index lists the 12 titles in document order and every link lands on its panel;
   - for each of the 12 panels: scroll to it, wait for the mount, save an element screenshot at both widths (<scratch>/page/<id>-<width>.png), assert that nothing inside the panel sticks out of it, that the document has no horizontal scroll, and that the panel text contains no NaN, undefined, Infinity or [object. Read all 24 screenshots.
4. Placement: each placeholder is in exactly the position listed above (grep the sources and check the built page: the panel is directly followed by the listed heading), and the table of contents is unchanged compared with git show HEAD:notes/RA_GATE2027_Notes.html.
5. Living together, with all 12 mounted:
   a. no duplicate element ids; the notes' own headings, tables, lists and links have the same computed styles as in a build without any simulation (compare a sample of each); CSS injected by one simulation through RASim.css does not match elements of another;
   b. change the state of each simulation in turn through RASim.instances[id].setState and confirm that getState() of the other 11 is unchanged;
   c. animations: start every animation, scroll away and confirm that frames stop while off-screen; with every animation stopped confirm that no requestAnimationFrame loop stays alive (count callbacks over 2 seconds);
   d. performance: time to DOMContentLoaded, time until the first simulation is mounted, the size of the HTML file, and a scripted scroll from top to bottom that mounts everything — report the numbers; any single mount that blocks the main thread for more than 200 ms is a defect to fix or report;
   e. Hide / Show on every panel; resize 1280 → 390 → 1280 with everything mounted; then confirm every canvas has a non-zero size and redraws;
   f. error isolation in the built page: in a scratch COPY of the HTML break one simulation's script (a syntax error) and confirm that the other 11 still mount and work.
6. Print: emulate print media and confirm that no .sim and no .sim-index element is visible.
7. Consistency across the 12: titles equal the fixed list; every panel starts with a one-sentence note and ends with the "Try this" hints; a Reset button exists everywhere and is labelled Reset; Play / Pause wording is the same everywhere; legends are present where curves are drawn; read-outs use the real minus sign; colours come from RASim.colors. Fix small inconsistencies in the simulation files when it is clearly safe (then re-run that simulation's test and harness); list larger ones without changing them.
8. Reported issues: for every item in the list above decide and record one of: fixed now (with a regression test), not a defect (with the reason), or left open (with the reason and its impact).
9. Update notes/sims/README.md: all 12 simulations are built; add a short "Run all checks" section with the exact commands.
10. Finish: all tests pass; run node notes/build.js --html-only last so that notes/RA_GATE2027_Notes.html is current; git status --short lists no change to the PDF and no stray files.

Your final answer is the structured result: which checks ran with the numbers observed, every defect with severity and whether it is fixed, the decision on every reported issue, and anything left open. Be exact and honest.
````

