# Shoot on the Move Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Score Pollen into the up-facing Cell while the robot drives, using a precomputed (distance, radial velocity) → RPM table generated and visualised on a laptop, and a heading lead for tangential velocity computed on the robot.

**Architecture:** One JavaScript physics module (`tools/shots/shots.js`) is shared by a browser visualiser (`index.html`) and a node exporter (`export.js`) that writes the generated `ShotTable.java`. On the robot a pure `ShotSolver` interpolates that table and leads the heading, a pure `TargetTracker` holds the Cell in field coordinates from timestamped tag detections and Pinpoint poses, and `Robot` wires them into the existing aim-assist and fire path. All pure classes have JVM tests; the ballistics has a self-check that gates export.

**Tech Stack:** FTC SDK v12.0, FTCLib 2.1.1 core, Java 8, JUnit 4, goBILDA Pinpoint driver, VisionPortal + AprilTagProcessor; Node 18+ and a browser with Plotly (CDN) for the tools.

**Spec:** `docs/superpowers/specs/2026-09-20-shoot-on-the-move-design.md`

## Global Constraints

- **Precondition:** the teleop plan (`docs/superpowers/plans/2026-09-20-biobuzz-teleop.md`) is fully executed. This plan modifies `Constants`, `ShooterSubsystem`, `OdometrySubsystem`, `VisionSubsystem` and `Robot` as that plan leaves them. If any of those files is missing, stop and execute the teleop plan first.
- SDK v12.0, no new Gradle dependencies. Java 8 source level, so no `var`, records, `List.of` or text blocks in TeamCode.
- Pure classes (`ShotSolver`, `TargetTracker`) import nothing from the SDK or Android. Only `ShotLog` and subsystems touch platform APIs.
- Units everywhere on the robot: metres, seconds, radians, RPM. The only degrees are gamepad-facing telemetry and the existing `aimRotation(bearingDeg)` helper.
- Frame: Pinpoint field frame as zeroed at init (X forward, Y left, heading CCW positive). Target and robot pose share it. No field constants for the Cell.
- Radial velocity is positive when closing on the Cell. Tangential velocity is positive when the robot moves to the left of the bearing.
- No Wi-Fi dashboards (manual R704). Tuning data goes to CSV on the hub.
- Commit prefixes: `feature:`, `docs:`, `chore:`. Every hardware-touching task is verified on the robot per the spec's ladder and recorded in the PR.
- Build and test commands need JDK 17 and the Android SDK (README "Requirements"). If they fail locally for environment reasons, push a branch and read CI.

---

## File map

Create:
- `tools/shots/shots.js`: ballistics, band search, grid sweep, Java export, self-check. The only copy of the physics.
- `tools/shots/index.html`: visualiser with the three 4414-style panels, form for inputs, export buttons.
- `tools/shots/export.js`: node script, `inputs.json` → `ShotTable.java`.
- `tools/shots/inputs.json`: the measured constants that generated the committed table.
- `tools/shots/README.md`: how to run the tools, the measurement procedure, and a table to record measurements.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/ShotTable.java`: generated, committed.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/ShotSolver.java`: pure lookup and heading lead.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/TargetTracker.java`: pure pose ring buffer and target hold.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/ShotLog.java`: CSV writer.
- `TeamCode/src/test/java/org/firstinspires/ftc/teamcode/ShotSolverTest.java`
- `TeamCode/src/test/java/org/firstinspires/ftc/teamcode/TargetTrackerTest.java`

Modify:
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Constants.java`: feed delay, camera transform, trim step, target expiry, log directory.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/ShooterSubsystem.java`: variable target RPM and trim.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/OdometrySubsystem.java`: field velocity.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/VisionSubsystem.java`: metre output, target position and frame time.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Robot.java`: solver-driven aim, fire gate, trim, telemetry, logging.
- `README.md`: controls table, tools pointer, on-robot ladder.
- `docs/superpowers/specs/2026-09-20-shoot-on-the-move-design.md`: note the `shots.js` / `export.js` split.

---

### Task 1: Ballistics module with self-check

**Files:**
- Create: `tools/shots/shots.js`
- Create: `tools/shots/export.js`
- Create: `tools/shots/inputs.json`
- Modify: `docs/superpowers/specs/2026-09-20-shoot-on-the-move-design.md`

**Interfaces:**
- Produces: `Shots.DEFAULTS`, `Shots.BALLS`, `Shots.axis(min, max, step)`, `Shots.simulate(rpm, radialVelMps, distM, p)` → `{points, result, tof}` with `result` one of `'score' | 'lip' | 'far' | 'short' | 'long'`, `Shots.band(distM, radialVelMps, p)` → `{lo, hi, rpm, band, tof}`, `Shots.solveGrid(p)` → `{dist[], vel[], rpm[][], band[][], tof[][], lo[][], hi[][]}` indexed `[distIdx][velIdx]`, `Shots.toJava(grid, p)` → string, `Shots.selfCheck()` → array of failure strings (empty is pass). `export.js` writes `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/ShotTable.java` whose public statics are listed in Task 2.

- [ ] **Step 1: Write `tools/shots/shots.js`**

```js
// Ballistics, shot-table sweep and Java export for the BIOBUZZ shooter.
// Loaded by index.html in a browser and by export.js under node. This is the only copy of the physics.
(function (root, factory) {
  if (typeof module === 'object' && module.exports) module.exports = factory();
  else root.Shots = factory();
})(typeof self !== 'undefined' ? self : this, function () {
  'use strict';
  const G = 9.81;    // m/s^2
  const RHO = 1.2;   // kg/m^3, air

  const BALLS = {
    POLLEN: { ballDiameterM: 0.0711, ballMassKg: 0.0249 },  // AndyMark am-5851: 2.80 in, 24.9 g
    NECTAR: { ballDiameterM: 0.0919, ballMassKg: 0.0413 },  // AndyMark am-5852: 3.62 in, 41.3 g
  };

  const DEFAULTS = {
    ball: 'POLLEN',
    ballDiameterM: BALLS.POLLEN.ballDiameterM,
    ballMassKg: BALLS.POLLEN.ballMassKg,
    dragCd: 0.45,             // guess; calibrate against measured landing distance
    launchAngleDeg: 60,       // measure on the robot
    exitHeightM: 0.40,        // measure on the robot
    exitSpeedPerRpm: 0.0023,  // m/s of ball per flywheel RPM; measure with slow-motion video
    shooterOffsetM: 0.15,     // ball exit point forward of the robot centre
    rpmToleranceRpm: 50,      // must match Constants.SHOOTER_TOLERANCE_RPM; the fixed hood leaves a 125-175 RPM band
    lipHeightM: 1.359,        // near lip of the up-facing Cell, 53.5 in
    openingWidthM: 0.508,     // 20 in
    openingLengthM: 0.356,    // 14 in along the tilted face
    openingTiltDeg: 30,       // face rises away from the robot; confirm against manual Figure 9-10
    marginM: 0.02,            // clearance beyond the ball radius at each edge
    distMinM: 0.6, distMaxM: 3.0, distStepM: 0.1,
    velMinMps: -1.0, velMaxMps: 1.0, velStepMps: 0.1,
    rpmMin: 1500, rpmMax: 5500, rpmStep: 25,
    dtS: 0.002,
  };

  function axis(min, max, step) {
    const out = [];
    for (let i = 0; min + i * step <= max + 1e-9; i++) out.push(Math.round((min + i * step) * 1e6) / 1e6);
    return out;
  }

  // The opening as a segment in the shot plane: near lip at x = distM, far edge higher and further.
  function opening(distM, p) {
    const tilt = p.openingTiltDeg * Math.PI / 180;
    return {
      x0: distM, y0: p.lipHeightM,
      x1: distM + p.openingLengthM * Math.cos(tilt), y1: p.lipHeightM + p.openingLengthM * Math.sin(tilt),
    };
  }

  // Fraction 0..1 along the opening where segment a->b crosses it, or null if it does not.
  function crossing(ax, ay, bx, by, o) {
    const rx = bx - ax, ry = by - ay, sx = o.x1 - o.x0, sy = o.y1 - o.y0;
    const den = rx * sy - ry * sx;
    if (Math.abs(den) < 1e-12) return null;
    const t = ((o.x0 - ax) * sy - (o.y0 - ay) * sx) / den;
    const u = ((o.x0 - ax) * ry - (o.y0 - ay) * rx) / den;
    return (t >= 0 && t <= 1 && u >= 0 && u <= 1) ? u : null;
  }

  // Fly one ball from the shooter (x = 0, y = exit height) toward a Cell whose near lip is at x = distM.
  // The robot's radial velocity adds to the ball's horizontal velocity. Quadratic drag, trapezoidal
  // velocity step so the no-drag case reproduces the exact parabola.
  function simulate(rpm, radialVelMps, distM, p) {
    const r = p.ballDiameterM / 2;
    const k = 0.5 * RHO * p.dragCd * Math.PI * r * r / p.ballMassKg;
    const a = p.launchAngleDeg * Math.PI / 180, v0 = rpm * p.exitSpeedPerRpm;
    const o = opening(distM, p);
    const clear = (r + p.marginM) / p.openingLengthM;  // edge clearance as a fraction of the opening
    let x = 0, y = p.exitHeightM, vx = v0 * Math.cos(a) + radialVelMps, vy = v0 * Math.sin(a), t = 0;
    const pts = [[x, y]];
    let result = 'short';
    while (t < 5) {
      const v = Math.hypot(vx, vy);
      const vx1 = vx - k * v * vx * p.dtS, vy1 = vy - (G + k * v * vy) * p.dtS;
      const nx = x + (vx + vx1) / 2 * p.dtS, ny = y + (vy + vy1) / 2 * p.dtS;
      vx = vx1; vy = vy1; t += p.dtS;
      const u = crossing(x, y, nx, ny, o);
      x = nx; y = ny; pts.push([x, y]);
      if (u !== null && vy < 0) { result = u < clear ? 'lip' : u > 1 - clear ? 'far' : 'score'; break; }
      if (x >= o.x0 && y < o.y0) { result = 'short'; break; }   // into the front face below the lip
      if (x > o.x1 && y >= o.y1) { result = 'long'; break; }     // over the far edge
      if (y < 0) { result = x > o.x1 ? 'long' : 'short'; break; }
    }
    return { points: pts, result: result, tof: t };
  }

  // The contiguous band of RPMs that score, its centre and the time of flight at the centre.
  function band(distM, radialVelMps, p) {
    let lo = NaN, hi = NaN;
    for (const rpm of axis(p.rpmMin, p.rpmMax, p.rpmStep)) {
      if (simulate(rpm, radialVelMps, distM, p).result === 'score') { if (Number.isNaN(lo)) lo = rpm; hi = rpm; }
    }
    if (Number.isNaN(lo)) return { lo: NaN, hi: NaN, rpm: NaN, band: NaN, tof: NaN };
    const centre = (lo + hi) / 2;
    return { lo: lo, hi: hi, rpm: centre, band: hi - lo, tof: simulate(centre, radialVelMps, distM, p).tof };
  }

  // Sweep distance x radial velocity. A cell is usable when its band is at least twice the flywheel tolerance.
  function solveGrid(p) {
    const dist = axis(p.distMinM, p.distMaxM, p.distStepM), vel = axis(p.velMinMps, p.velMaxMps, p.velStepMps);
    const g = { dist: dist, vel: vel, rpm: [], band: [], tof: [], lo: [], hi: [] };
    for (let i = 0; i < dist.length; i++) {
      for (const key of ['rpm', 'band', 'tof', 'lo', 'hi']) g[key].push([]);
      for (let j = 0; j < vel.length; j++) {
        const b = band(dist[i], vel[j], p);
        const usable = b.band >= 2 * p.rpmToleranceRpm;
        g.rpm[i].push(usable ? b.rpm : NaN);
        g.band[i].push(b.band);
        g.tof[i].push(usable ? b.tof : NaN);
        g.lo[i].push(b.lo);
        g.hi[i].push(b.hi);
      }
    }
    return g;
  }

  function num(v) { return Number.isNaN(v) ? 'Double.NaN' : String(Math.round(v * 1000) / 1000); }
  function row(a) { return '{' + a.map(num).join(', ') + '}'; }
  function matrix(m) { return '{\n        ' + m.map(row).join(',\n        ') + '\n    }'; }

  function toJava(g, p) {
    return 'package org.firstinspires.ftc.teamcode;\n\n' +
      '/**\n' +
      ' * GENERATED by tools/shots (node tools/shots/export.js). Do not edit by hand: change tools/shots/inputs.json\n' +
      ' * and regenerate. Rows are distance to the near lip, columns are radial robot velocity (positive closing).\n' +
      ' * Inputs: ' + JSON.stringify(p) + '\n' +
      ' */\n' +
      'public final class ShotTable {\n' +
      '    private ShotTable() {}\n\n' +
      '    public static final String BALL = "' + p.ball + '";\n' +
      '    public static final double LAUNCH_ANGLE_DEG = ' + num(p.launchAngleDeg) + ';\n' +
      '    public static final double EXIT_HEIGHT_M = ' + num(p.exitHeightM) + ';\n' +
      '    public static final double EXIT_SPEED_PER_RPM = ' + p.exitSpeedPerRpm + ';\n' +
      '    public static final double SHOOTER_OFFSET_M = ' + num(p.shooterOffsetM) + ';\n' +
      '    public static final double OPENING_WIDTH_M = ' + num(p.openingWidthM) + ';\n' +
      '    /** Ball radius plus edge margin: how far inside the opening\'s side edges the ball centre must pass. */\n' +
      '    public static final double LATERAL_CLEARANCE_M = ' + num(p.ballDiameterM / 2 + p.marginM) + ';\n' +
      '    public static final double MIN_BAND_RPM = ' + num(2 * p.rpmToleranceRpm) + ';\n\n' +
      '    public static final double[] DIST_M = ' + row(g.dist) + ';\n' +
      '    public static final double[] VEL_MPS = ' + row(g.vel) + ';\n' +
      '    /** Band-centre flywheel RPM, [dist][vel]. NaN where no shot lands. */\n' +
      '    public static final double[][] RPM = ' + matrix(g.rpm) + ';\n' +
      '    /** Width of the valid RPM band, [dist][vel]. NaN where no shot lands. */\n' +
      '    public static final double[][] BAND_RPM = ' + matrix(g.band) + ';\n' +
      '    /** Time of flight at the band centre, seconds, [dist][vel]. */\n' +
      '    public static final double[][] TOF_S = ' + matrix(g.tof) + ';\n' +
      '}\n';
  }

  // Two checks that fail loudly if the physics is broken. Export is refused when this returns anything.
  function selfCheck() {
    const fails = [];
    // 1. With no drag the flight must match the closed-form parabola to 1 mm.
    const p = Object.assign({}, DEFAULTS, { dragCd: 0, lipHeightM: 100 });  // nothing to hit
    const s = simulate(3000, 0.3, 2, p);
    const a = p.launchAngleDeg * Math.PI / 180, v0 = 3000 * p.exitSpeedPerRpm;
    for (const frac of [0.25, 0.5, 0.75]) {
      const n = Math.floor((s.points.length - 1) * frac), t = n * p.dtS;
      const ex = (v0 * Math.cos(a) + 0.3) * t, ey = p.exitHeightM + v0 * Math.sin(a) * t - 0.5 * G * t * t;
      const err = Math.hypot(s.points[n][0] - ex, s.points[n][1] - ey);
      if (err > 0.001) fails.push('no-drag flight is ' + (err * 1000).toFixed(2) + ' mm off the parabola at t=' + t.toFixed(3) + ' s');
    }
    // 2. Standing still, a longer shot needs more RPM.
    const g = solveGrid(Object.assign({}, DEFAULTS, { velMinMps: 0, velMaxMps: 0 }));
    let last = -Infinity, any = false;
    for (let i = 0; i < g.dist.length; i++) {
      const r = g.rpm[i][0];
      if (Number.isNaN(r)) continue;
      any = true;
      if (r < last) fails.push('RPM falls with distance at ' + g.dist[i] + ' m');
      last = r;
    }
    if (!any) fails.push('no distance has a valid stationary shot with the default inputs');
    return fails;
  }

  return { DEFAULTS: DEFAULTS, BALLS: BALLS, axis: axis, opening: opening, simulate: simulate, band: band, solveGrid: solveGrid, toJava: toJava, selfCheck: selfCheck };
});
```

- [ ] **Step 2: Write `tools/shots/export.js`**

```js
#!/usr/bin/env node
// Regenerates TeamCode/.../ShotTable.java from tools/shots/inputs.json. Run from any directory:
//   node tools/shots/export.js
const fs = require('fs');
const path = require('path');
const Shots = require('./shots.js');

const inputs = JSON.parse(fs.readFileSync(path.join(__dirname, 'inputs.json'), 'utf8'));
const p = Object.assign({}, Shots.DEFAULTS, inputs);

const fails = Shots.selfCheck();
if (fails.length) {
  console.error('Self-check failed, not exporting:\n  ' + fails.join('\n  '));
  process.exit(1);
}

const out = path.join(__dirname, '..', '..', 'TeamCode', 'src', 'main', 'java', 'org', 'firstinspires', 'ftc', 'teamcode', 'ShotTable.java');
const grid = Shots.solveGrid(p);
fs.writeFileSync(out, Shots.toJava(grid, p));
const usable = grid.rpm.flat().filter(v => !Number.isNaN(v)).length;
console.log('wrote ' + path.relative(process.cwd(), out) + ': ' + grid.dist.length + ' x ' + grid.vel.length + ' cells, ' + usable + ' usable');
```

- [ ] **Step 3: Write `tools/shots/inputs.json`**

Every physical input, copied from the defaults. This file is the record of what generated the committed table, so all keys are listed even while they equal the defaults.

```json
{
  "ball": "POLLEN",
  "ballDiameterM": 0.0711,
  "ballMassKg": 0.0249,
  "dragCd": 0.45,
  "launchAngleDeg": 60,
  "exitHeightM": 0.40,
  "exitSpeedPerRpm": 0.0023,
  "shooterOffsetM": 0.15,
  "rpmToleranceRpm": 50,
  "lipHeightM": 1.359,
  "openingWidthM": 0.508,
  "openingLengthM": 0.356,
  "openingTiltDeg": 30,
  "marginM": 0.02
}
```

- [ ] **Step 4: Run the self-check and a spot simulation**

```bash
node -e "const S=require('./tools/shots/shots.js'); console.log(S.selfCheck()); const b=S.band(2.0, 0, S.DEFAULTS); console.log(b); console.log(S.simulate(b.rpm, 0, 2.0, S.DEFAULTS).result);"
```

Expected: `[]`, then an object like `{ lo: 2600, hi: 2725, rpm: 2662.5, band: 125, tof: 0.756 }`, then `score`.

The band is narrow on purpose. With a fixed launch angle the lower edge is set by the near lip and the upper edge by the far edge of the 14 in opening, and that comes out at 125 to 175 RPM for every distance and every launch angle between 45° and 75° with these inputs. That is why `rpmToleranceRpm` defaults to 50 and Task 6 tightens `Constants.SHOOTER_TOLERANCE_RPM` to match: the flywheel has to hold ±50 RPM or no cell is usable. If `selfCheck` reports "no distance has a valid stationary shot", something in `DEFAULTS` has been changed so that the band is under twice the tolerance; put it back rather than loosening the check.

- [ ] **Step 5: Note the file split in the spec**

In `docs/superpowers/specs/2026-09-20-shoot-on-the-move-design.md`, in the "Offline solver and visualiser" section, replace the sentence beginning "`tools/shots/index.html`, vanilla JavaScript with Plotly" up to "no build step." with:

```
`tools/shots/index.html`, vanilla JavaScript with Plotly from its CDN for the
plots (so it needs internet the first time; the browser caches it after).
Opened straight from the file system, no server, no build step. The physics
lives in `tools/shots/shots.js`, loaded by the page and by a node script
`tools/shots/export.js` that regenerates the Java table from
`tools/shots/inputs.json`, so the table can be rebuilt without a browser. A
```

- [ ] **Step 6: Commit**

```bash
git add tools/shots docs/superpowers/specs/2026-09-20-shoot-on-the-move-design.md
git commit -m "feature: shot ballistics module, node exporter and inputs file"
```

---

### Task 2: Generate and commit the first `ShotTable.java`

**Files:**
- Create: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/ShotTable.java` (generated)

**Interfaces:**
- Consumes: `tools/shots/export.js` (Task 1).
- Produces: `ShotTable.BALL`, `LAUNCH_ANGLE_DEG`, `EXIT_HEIGHT_M`, `EXIT_SPEED_PER_RPM`, `SHOOTER_OFFSET_M`, `OPENING_WIDTH_M`, `LATERAL_CLEARANCE_M`, `MIN_BAND_RPM` (all `double` except `BALL`), `double[] DIST_M`, `double[] VEL_MPS`, `double[][] RPM`, `BAND_RPM`, `TOF_S` indexed `[dist][vel]`. Used by Task 6.

- [ ] **Step 1: Generate**

```bash
node tools/shots/export.js
head -30 TeamCode/src/main/java/org/firstinspires/ftc/teamcode/ShotTable.java
```

Expected: `wrote TeamCode/.../ShotTable.java: 25 x 21 cells, N usable` with N above 100, and a file that starts with the package line and the GENERATED comment.

- [ ] **Step 2: Compile**

```bash
./gradlew :TeamCode:compileDebugJavaWithJavac --no-daemon
```

Expected: `BUILD SUCCESSFUL`. A failure here means a `Double.NaN` or number formatting problem in `toJava`; fix `num()` in `shots.js`, regenerate, do not hand-edit the Java.

- [ ] **Step 3: Commit**

```bash
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/ShotTable.java
git commit -m "feature: generated shot table from default inputs"
```

---

### Task 3: Visualiser page

**Files:**
- Create: `tools/shots/index.html`

**Interfaces:**
- Consumes: everything in `Shots` (Task 1).
- Produces: nothing for code. A person uses it to inspect the table and to export `ShotTable.java` and `inputs.json`.

- [ ] **Step 1: Write `tools/shots/index.html`**

```html
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>BIOBUZZ shot table</title>
<script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script>
<script src="shots.js"></script>
<style>
  body { font: 14px system-ui, sans-serif; margin: 0; display: grid; grid-template-columns: 320px 1fr; height: 100vh; }
  form { padding: 12px; overflow-y: auto; border-right: 1px solid #ccc; }
  form label { display: flex; justify-content: space-between; gap: 8px; margin: 4px 0; align-items: center; }
  form input[type=number] { width: 100px; }
  form h3 { margin: 14px 0 4px; font-size: 13px; text-transform: uppercase; color: #555; }
  form button { margin: 6px 4px 0 0; }
  main { display: grid; grid-template-rows: auto auto 1fr 1fr; grid-template-columns: 1fr 1fr; gap: 8px; padding: 8px; min-height: 0; }
  #banner { grid-column: 1 / 3; background: #b00020; color: #fff; padding: 8px; display: none; white-space: pre-wrap; }
  #controls { grid-column: 1 / 3; display: flex; gap: 24px; align-items: center; flex-wrap: wrap; font-family: monospace; }
  #controls input[type=range] { width: 240px; }
  #fan { grid-column: 1 / 3; min-height: 0; }
  #band, #heat { min-height: 0; }
</style>
</head>
<body>
<form id="inputs" onsubmit="return false">
  <h3>Ball</h3>
  <label>Ball <select id="ball"><option>POLLEN</option><option>NECTAR</option></select></label>
  <span id="fields"></span>
  <h3>Actions</h3>
  <button id="compute">Compute table</button>
  <button id="exportJava" disabled>Export ShotTable.java</button>
  <button id="exportInputs">Export inputs.json</button>
  <label>Load inputs.json <input type="file" id="loadInputs" accept=".json"></label>
  <p id="status"></p>
</form>
<main>
  <div id="banner"></div>
  <div id="controls">
    <label>Distance <input type="range" id="distIdx" min="0" max="0" value="0"> <span id="distVal"></span></label>
    <label>Radial velocity <input type="range" id="velIdx" min="0" max="0" value="0"> <span id="velVal"></span></label>
    <label>Heatmap <select id="heatKey"><option value="rpm">RPM</option><option value="band">band width</option><option value="tof">time of flight</option></select></label>
    <span id="stats"></span>
  </div>
  <div id="fan"></div>
  <div id="band"></div>
  <div id="heat"></div>
</main>
<script>
'use strict';
const LABELS = {
  ballDiameterM: 'Diameter (m)', ballMassKg: 'Mass (kg)', dragCd: 'Drag Cd',
  launchAngleDeg: 'Launch angle (deg)', exitHeightM: 'Exit height (m)', exitSpeedPerRpm: 'Exit speed per RPM (m/s)',
  shooterOffsetM: 'Shooter offset fwd (m)', rpmToleranceRpm: 'Flywheel tolerance (RPM)',
  lipHeightM: 'Near lip height (m)', openingWidthM: 'Opening width (m)', openingLengthM: 'Opening length (m)',
  openingTiltDeg: 'Opening tilt (deg)', marginM: 'Edge margin (m)',
  distMinM: 'Distance min (m)', distMaxM: 'Distance max (m)', distStepM: 'Distance step (m)',
  velMinMps: 'Radial vel min (m/s)', velMaxMps: 'Radial vel max (m/s)', velStepMps: 'Radial vel step (m/s)',
  rpmMin: 'RPM min', rpmMax: 'RPM max', rpmStep: 'RPM step', dtS: 'Time step (s)',
};
const GROUPS = [['ballDiameterM', 'ballMassKg', 'dragCd'], ['Shooter', 'launchAngleDeg', 'exitHeightM', 'exitSpeedPerRpm', 'shooterOffsetM', 'rpmToleranceRpm'],
  ['Cell', 'lipHeightM', 'openingWidthM', 'openingLengthM', 'openingTiltDeg', 'marginM'],
  ['Sweep', 'distMinM', 'distMaxM', 'distStepM', 'velMinMps', 'velMaxMps', 'velStepMps', 'rpmMin', 'rpmMax', 'rpmStep', 'dtS']];
const NUMERIC = Object.keys(LABELS);
const $ = id => document.getElementById(id);

// Build the numeric fields from LABELS so the form and DEFAULTS cannot drift apart.
const fields = $('fields');
for (const group of GROUPS) {
  for (const key of group) {
    if (!(key in LABELS)) { const h = document.createElement('h3'); h.textContent = key; fields.appendChild(h); continue; }
    const l = document.createElement('label'); l.textContent = LABELS[key];
    const i = document.createElement('input'); i.type = 'number'; i.step = 'any'; i.id = key;
    l.appendChild(i); fields.appendChild(l);
  }
}
function readInputs() {
  const p = Object.assign({}, Shots.DEFAULTS, { ball: $('ball').value });
  for (const k of NUMERIC) p[k] = parseFloat($(k).value);
  return p;
}
function writeInputs(p) { $('ball').value = p.ball; for (const k of NUMERIC) $(k).value = p[k]; }
writeInputs(Shots.DEFAULTS);
$('ball').onchange = () => { const b = Shots.BALLS[$('ball').value]; $('ballDiameterM').value = b.ballDiameterM; $('ballMassKg').value = b.ballMassKg; };

let grid = null, params = null;

$('compute').onclick = () => {
  params = readInputs();
  $('status').textContent = 'computing...';
  setTimeout(() => {
    const t0 = performance.now();
    grid = Shots.solveGrid(params);
    $('distIdx').max = grid.dist.length - 1; $('velIdx').max = grid.vel.length - 1;
    $('velIdx').value = Math.round((grid.vel.length - 1) / 2);
    $('exportJava').disabled = selfCheckFailed;
    $('status').textContent = 'computed ' + grid.dist.length + ' x ' + grid.vel.length + ' cells in ' + ((performance.now() - t0) / 1000).toFixed(1) + ' s';
    draw();
  }, 10);
};
for (const id of ['distIdx', 'velIdx', 'heatKey']) $(id).oninput = () => grid && draw();

function download(name, text) {
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([text], { type: 'text/plain' })); a.download = name; a.click();
}
$('exportJava').onclick = () => download('ShotTable.java', Shots.toJava(grid, params));
$('exportInputs').onclick = () => {
  const p = readInputs(), out = {};
  for (const k of ['ball', 'ballDiameterM', 'ballMassKg', 'dragCd', 'launchAngleDeg', 'exitHeightM', 'exitSpeedPerRpm', 'shooterOffsetM', 'rpmToleranceRpm', 'lipHeightM', 'openingWidthM', 'openingLengthM', 'openingTiltDeg', 'marginM']) out[k] = p[k];
  download('inputs.json', JSON.stringify(out, null, 2) + '\n');
};
$('loadInputs').onchange = e => {
  const f = e.target.files[0]; if (!f) return;
  f.text().then(t => writeInputs(Object.assign({}, Shots.DEFAULTS, JSON.parse(t))));
};

function draw() {
  const i = +$('distIdx').value, j = +$('velIdx').value, d = grid.dist[i], vr = grid.vel[j];
  $('distVal').textContent = d.toFixed(2) + ' m'; $('velVal').textContent = vr.toFixed(2) + ' m/s';
  drawFan(i, j, d, vr); drawBand(i, d); drawHeat(); drawStats(i, j, d);
}

// Panel 1: every RPM in the sweep at this (distance, radial velocity). Green scores, red misses, blue is the chosen shot.
function drawFan(i, j, d, vr) {
  const traces = [];
  for (const rpm of Shots.axis(params.rpmMin, params.rpmMax, params.rpmStep)) {
    const s = Shots.simulate(rpm, vr, d, params);
    traces.push({ x: s.points.map(p => p[0]), y: s.points.map(p => p[1]), mode: 'lines', hoverinfo: 'name', name: rpm + ' RPM ' + s.result,
      line: { color: s.result === 'score' ? 'rgba(0,160,60,0.35)' : 'rgba(200,0,0,0.15)', width: 1 }, showlegend: false });
  }
  const chosen = grid.rpm[i][j];
  if (!Number.isNaN(chosen)) {
    const s = Shots.simulate(chosen, vr, d, params);
    traces.push({ x: s.points.map(p => p[0]), y: s.points.map(p => p[1]), mode: 'lines', name: 'chosen ' + chosen + ' RPM', line: { color: '#1f5fbf', width: 3 } });
  }
  const o = Shots.opening(d, params);
  traces.push({ x: [o.x0, o.x1], y: [o.y0, o.y1], mode: 'lines+markers', name: 'Cell opening', line: { color: '#000', width: 4 }, marker: { size: 8 } });
  traces.push({ x: [o.x0, o.x0], y: [0, o.y0], mode: 'lines', name: 'Cell front', line: { color: '#000', width: 2, dash: 'dot' } });
  Plotly.react('fan', traces, {
    title: 'Shots at ' + d.toFixed(2) + ' m, radial ' + vr.toFixed(2) + ' m/s' + (Number.isNaN(chosen) ? '  (NO SHOT)' : ''),
    xaxis: { title: 'distance (m)', range: [0, Math.max(3.5, o.x1 + 0.5)] }, yaxis: { title: 'height (m)', scaleanchor: 'x', range: [0, 3] },
    margin: { t: 40, r: 10 }, showlegend: false,
    annotations: [{ x: 0, y: params.exitHeightM, ax: -vr * 60, ay: 0, xref: 'x', yref: 'y', axref: 'pixel', ayref: 'pixel', showarrow: true, arrowhead: 2, text: 'robot ' + vr.toFixed(1) + ' m/s' }],
  }, { responsive: true });
}

// Panel 2: the valid RPM band against radial velocity at this distance. Where lo and hi meet, the shot disappears.
function drawBand(i, d) {
  const x = grid.vel;
  Plotly.react('band', [
    { x: x, y: grid.hi[i], mode: 'lines', name: 'highest scoring RPM', line: { color: 'green' } },
    { x: x, y: grid.lo[i], mode: 'lines', name: 'lowest scoring RPM', line: { color: 'red' }, fill: 'tonexty', fillcolor: 'rgba(0,160,60,0.15)' },
    { x: x, y: grid.rpm[i], mode: 'lines+markers', name: 'table RPM', line: { color: '#1f5fbf', width: 3 } },
    { x: [grid.vel[+$('velIdx').value]], y: [grid.rpm[i][+$('velIdx').value]], mode: 'markers', name: 'selected', marker: { color: '#1f5fbf', size: 12, symbol: 'star' } },
  ], { title: 'Valid RPM band at ' + d.toFixed(2) + ' m', xaxis: { title: 'radial velocity (m/s, + closing)' }, yaxis: { title: 'flywheel RPM' }, margin: { t: 40, r: 10 }, legend: { orientation: 'h', y: -0.25 } }, { responsive: true });
}

// Panel 3: the whole table. Blank cells have no shot.
function drawHeat() {
  const key = $('heatKey').value, m = grid[key];
  const z = grid.vel.map((_, j) => grid.dist.map((_, i) => m[i][j]));
  Plotly.react('heat', [{ type: 'heatmap', x: grid.dist, y: grid.vel, z: z, colorscale: 'Viridis', colorbar: { title: key } }],
    { title: 'Table: ' + key + ' over distance and radial velocity', xaxis: { title: 'distance (m)' }, yaxis: { title: 'radial velocity (m/s)' }, margin: { t: 40, r: 10 }, plot_bgcolor: '#ddd' }, { responsive: true });
}

function drawStats(i, j, d) {
  const rpm = grid.rpm[i][j], band = grid.band[i][j], tof = grid.tof[i][j];
  const lateral = params.openingWidthM / 2 - params.ballDiameterM / 2 - params.marginM;
  const headingTolDeg = Math.atan(lateral / d) * 180 / Math.PI;
  const uh = Number.isNaN(rpm) ? NaN : rpm * params.exitSpeedPerRpm * Math.cos(params.launchAngleDeg * Math.PI / 180);
  $('stats').textContent = Number.isNaN(rpm)
    ? 'NO SHOT (band ' + (Number.isNaN(band) ? 0 : band) + ' RPM, need ' + 2 * params.rpmToleranceRpm + ')'
    : 'RPM ' + rpm + '  band ' + band + ' RPM (' + grid.lo[i][j] + '-' + grid.hi[i][j] + ')  TOF ' + tof.toFixed(2) + ' s  heading tol +/-' + headingTolDeg.toFixed(1) + ' deg  horizontal exit ' + uh.toFixed(2) + ' m/s';
}

// Self-check on load. A failure shows the banner and keeps export disabled.
const fails = Shots.selfCheck();
const selfCheckFailed = fails.length > 0;
if (selfCheckFailed) { $('banner').style.display = 'block'; $('banner').textContent = 'Self-check failed, export disabled:\n' + fails.join('\n'); }
</script>
</body>
</html>
```

- [ ] **Step 2: Check it in a browser**

Open `tools/shots/index.html` (double-click, or `xdg-open tools/shots/index.html`). Expected: no red banner; the form shows the defaults; after "Compute table" the status line shows the cell count and time, the fan shows a spread of red and green arcs with a black opening segment, the band chart shows the green fill closing toward the right, the heatmap has blank cells at high closing speed. Moving the sliders redraws. "Export ShotTable.java" downloads a file identical to the committed one when the form still holds the defaults:

```bash
diff ~/Downloads/ShotTable.java TeamCode/src/main/java/org/firstinspires/ftc/teamcode/ShotTable.java && echo SAME
```

Expected: `SAME`.

If no browser is available to the executor, record that the visual check is pending and verify the page parses:

```bash
node -e "const fs=require('fs'); const h=fs.readFileSync('tools/shots/index.html','utf8'); const js=h.split('<script>')[1].split('</script>')[0]; new Function(js.replace(/^'use strict';/, '')); console.log('script parses')"
```

Expected: `script parses`.

- [ ] **Step 3: Commit**

```bash
git add tools/shots/index.html
git commit -m "feature: shot table visualiser with trajectory fan, band chart and heatmap"
```

---

### Task 4: `ShotSolver`, tested

**Files:**
- Create: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/ShotSolver.java`
- Test: `TeamCode/src/test/java/org/firstinspires/ftc/teamcode/ShotSolverTest.java`

**Interfaces:**
- Produces:
  - `ShotSolver(double[] dist, double[] vel, double[][] rpm, double[][] band, double[][] tof, double launchAngleDeg, double exitSpeedPerRpm, double shooterOffsetM, double feedDelayS, double minBandRpm, double openingWidthM, double lateralClearanceM, double fallbackRpm)`
  - `ShotSolver.Shot solve(double x, double y, double headingRad, double vx, double vy, double targetX, double targetY)`
  - `ShotSolver.Shot` fields: `boolean valid`, `String reason` (`"OK"`, `"OUT OF RANGE"`, `"CLOSING TOO FAST"`, `"BACKING TOO FAST"`, `"NO SHOT"`), `double rpm`, `idleRpm`, `headingRad`, `headingToleranceRad`, `distanceM`, `radialVel`, `tangentialVel`, `timeOfFlightS`, `bandRpm`; method `double headingErrorRad(double currentHeadingRad)` (positive means turn left).
  - `static double interpolate(double[] xs, double[] ys, double[][] v, double x, double y)` returning `Double.NaN` outside the grid or when any of the four corners is NaN.
  - `static double wrap(double rad)` to (-pi, pi].

- [ ] **Step 1: Write the failing tests**

```java
package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class ShotSolverTest {
    private static final double EPS = 1e-9;

    // rpm = 2000 + 500 * (d - 1) - 200 * vr on a 3 x 3 grid; the (3 m, +1 m/s) cell has no shot.
    private static final double[] DIST = {1, 2, 3};
    private static final double[] VEL = {-1, 0, 1};
    private static final double[][] RPM = {
            {2200, 2000, 1800},
            {2700, 2500, 2300},
            {3200, 3000, Double.NaN}};
    private static final double[][] BAND = {
            {400, 400, 400},
            {400, 400, 100},
            {400, 400, Double.NaN}};
    private static final double[][] TOF = {
            {0.8, 0.8, 0.8},
            {1.0, 1.0, 1.0},
            {1.2, 1.2, Double.NaN}};

    private static ShotSolver solver(double shooterOffsetM, double feedDelayS) {
        // 60 deg launch, 0.002 m/s per RPM: 2500 RPM gives 5 m/s exit, 2.5 m/s horizontal.
        return new ShotSolver(DIST, VEL, RPM, BAND, TOF, 60, 0.002, shooterOffsetM, feedDelayS, 200, 0.508, 0.0555, 3500);
    }

    @Test
    public void interpolateReturnsGridValueAtGridPoint() {
        assertEquals(2500, ShotSolver.interpolate(DIST, VEL, RPM, 2, 0), EPS);
    }

    @Test
    public void interpolateAveragesNeighboursAtMidpoint() {
        // between (1,-1)=2200, (1,0)=2000, (2,-1)=2700, (2,0)=2500
        assertEquals(2350, ShotSolver.interpolate(DIST, VEL, RPM, 1.5, -0.5), EPS);
    }

    @Test
    public void interpolateOutsideGridIsNaN() {
        assertTrue(Double.isNaN(ShotSolver.interpolate(DIST, VEL, RPM, 0.5, 0)));
        assertTrue(Double.isNaN(ShotSolver.interpolate(DIST, VEL, RPM, 2, 1.5)));
    }

    @Test
    public void interpolateNextToMissingCellIsNaN() {
        assertTrue(Double.isNaN(ShotSolver.interpolate(DIST, VEL, RPM, 2.5, 0.5)));
    }

    @Test
    public void stationaryShotAimsAlongBearingAtTableRpm() {
        ShotSolver.Shot s = solver(0, 0).solve(0, 0, 0, 0, 0, 2, 0);
        assertTrue(s.valid);
        assertEquals("OK", s.reason);
        assertEquals(2, s.distanceM, EPS);
        assertEquals(0, s.headingRad, EPS);
        assertEquals(2500, s.rpm, EPS);
        assertEquals(1.0, s.timeOfFlightS, EPS);
    }

    @Test
    public void bearingFollowsTargetPosition() {
        ShotSolver.Shot s = solver(0, 0).solve(0, 0, 0, 0, 0, 0, 2);
        assertEquals(Math.PI / 2, s.headingRad, EPS);
        assertEquals(0, s.radialVel, EPS);
    }

    @Test
    public void strafingLeftLeadsRightOfBearing() {
        // Target straight ahead, robot moving left (+y) at 0.5 m/s. Horizontal exit speed 2.5 m/s.
        ShotSolver.Shot s = solver(0, 0).solve(0, 0, 0, 0, 0.5, 2, 0);
        assertEquals(0.5, s.tangentialVel, EPS);
        assertEquals(0, s.radialVel, EPS);
        assertEquals(-Math.asin(0.5 / 2.5), s.headingRad, 1e-9);
    }

    @Test
    public void closingLowersRpmAndCountsAsRadial() {
        ShotSolver.Shot s = solver(0, 0).solve(0, 0, 0, 0.5, 0, 2, 0);
        assertEquals(0.5, s.radialVel, EPS);
        assertEquals(2400, s.rpm, EPS);   // 2500 - 200 * 0.5
        assertTrue(s.valid);
    }

    @Test
    public void closingTooFastIsInvalidWithReason() {
        ShotSolver.Shot s = solver(0, 0).solve(0, 0, 0, 2.0, 0, 2, 0);
        assertFalse(s.valid);
        assertEquals("CLOSING TOO FAST", s.reason);
        assertEquals(2500, s.idleRpm, EPS);   // stationary RPM at the same distance
    }

    @Test
    public void backingTooFastIsInvalidWithReason() {
        assertEquals("BACKING TOO FAST", solver(0, 0).solve(0, 0, 0, -2.0, 0, 2, 0).reason);
    }

    @Test
    public void outOfRangeIsInvalidAndIdlesAtNearestEdge() {
        ShotSolver.Shot s = solver(0, 0).solve(0, 0, 0, 0, 0, 5, 0);
        assertFalse(s.valid);
        assertEquals("OUT OF RANGE", s.reason);
        assertEquals(3000, s.idleRpm, EPS);   // stationary RPM at the far edge of the grid
    }

    @Test
    public void idleFallsBackWhenTheStationaryCellIsMissing() {
        double[][] noStationary = {{2200, Double.NaN, 1800}, {2700, Double.NaN, 2300}, {3200, Double.NaN, Double.NaN}};
        ShotSolver s = new ShotSolver(DIST, VEL, noStationary, BAND, TOF, 60, 0.002, 0, 0, 200, 0.508, 0.0555, 3500);
        assertEquals(3500, s.solve(0, 0, 0, 0, 0, 2, 0).idleRpm, EPS);
    }

    @Test
    public void narrowBandIsInvalid() {
        // (2 m, +1 m/s) has a 100 RPM band, below the 200 RPM minimum.
        ShotSolver.Shot s = solver(0, 0).solve(0, 0, 0, 1.0, 0, 2, 0);
        assertFalse(s.valid);
        assertEquals("NO SHOT", s.reason);
    }

    @Test
    public void feedDelayAdvancesThePose() {
        ShotSolver.Shot s = solver(0, 0.5).solve(0, 0, 0, 1.0, 0, 2, 0);
        assertEquals(1.5, s.distanceM, EPS);
    }

    @Test
    public void shooterOffsetShortensDistanceAlongHeading() {
        ShotSolver.Shot s = solver(0.2, 0).solve(0, 0, 0, 0, 0, 2, 0);
        assertEquals(1.8, s.distanceM, EPS);
    }

    @Test
    public void headingToleranceNarrowsWithDistance() {
        ShotSolver.Shot near = solver(0, 0).solve(0, 0, 0, 0, 0, 1, 0);
        ShotSolver.Shot far = solver(0, 0).solve(0, 0, 0, 0, 0, 3, 0);
        assertEquals(Math.atan((0.254 - 0.0555) / 1.0), near.headingToleranceRad, EPS);
        assertTrue(far.headingToleranceRad < near.headingToleranceRad);
    }

    @Test
    public void headingErrorWrapsAndIsPositiveWhenTargetIsLeft() {
        ShotSolver.Shot s = solver(0, 0).solve(0, 0, 0, 0, 0, 0, 2);   // target heading +90 deg
        assertEquals(Math.PI / 2, s.headingErrorRad(0), EPS);
        assertEquals(-Math.PI / 2, s.headingErrorRad(Math.PI), EPS);
        assertEquals(0.1, ShotSolver.wrap(2 * Math.PI + 0.1), EPS);
    }
}
```

- [ ] **Step 2: Run the tests to see them fail**

```bash
./gradlew :TeamCode:testDebugUnitTest --tests '*ShotSolverTest' --no-daemon
```

Expected: compile failure, `cannot find symbol ShotSolver`.

- [ ] **Step 3: Write `ShotSolver.java`**

```java
package org.firstinspires.ftc.teamcode;

/**
 * Turns robot pose, robot velocity and the Cell position into a flywheel RPM and a heading.
 *
 * The table (generated by tools/shots) already contains the effect of the robot's radial velocity on
 * the ball, so the only lookahead here is the indexer feed delay. Tangential velocity becomes a heading
 * lead: the robot turns away from its sideways motion so the ball's horizontal velocity (exit plus robot)
 * points at the Cell. Pure maths, no SDK types, unit tested.
 */
public final class ShotSolver {
    public static final class Shot {
        public final boolean valid;
        public final String reason;
        /** Flywheel target when valid. */
        public final double rpm;
        /** Flywheel target to hold when not valid: the stationary RPM at this distance clamped to the grid, or the fallback if that cell is empty. */
        public final double idleRpm;
        /** Field heading the robot must point to, radians. */
        public final double headingRad;
        /** Heading error allowed by the Cell width at this distance, radians. */
        public final double headingToleranceRad;
        public final double distanceM;
        public final double radialVel;
        public final double tangentialVel;
        public final double timeOfFlightS;
        public final double bandRpm;

        Shot(boolean valid, String reason, double rpm, double idleRpm, double headingRad, double headingToleranceRad,
             double distanceM, double radialVel, double tangentialVel, double timeOfFlightS, double bandRpm) {
            this.valid = valid;
            this.reason = reason;
            this.rpm = rpm;
            this.idleRpm = idleRpm;
            this.headingRad = headingRad;
            this.headingToleranceRad = headingToleranceRad;
            this.distanceM = distanceM;
            this.radialVel = radialVel;
            this.tangentialVel = tangentialVel;
            this.timeOfFlightS = timeOfFlightS;
            this.bandRpm = bandRpm;
        }

        /** How far to turn, radians, positive to the left (anticlockwise). */
        public double headingErrorRad(double currentHeadingRad) {
            return wrap(headingRad - currentHeadingRad);
        }
    }

    private final double[] dist;
    private final double[] vel;
    private final double[][] rpm;
    private final double[][] band;
    private final double[][] tof;
    private final double launchAngleRad;
    private final double exitSpeedPerRpm;
    private final double shooterOffsetM;
    private final double feedDelayS;
    private final double minBandRpm;
    private final double halfOpeningM;
    private final double fallbackRpm;

    public ShotSolver(double[] dist, double[] vel, double[][] rpm, double[][] band, double[][] tof,
                      double launchAngleDeg, double exitSpeedPerRpm, double shooterOffsetM, double feedDelayS,
                      double minBandRpm, double openingWidthM, double lateralClearanceM, double fallbackRpm) {
        this.dist = dist;
        this.vel = vel;
        this.rpm = rpm;
        this.band = band;
        this.tof = tof;
        this.launchAngleRad = Math.toRadians(launchAngleDeg);
        this.exitSpeedPerRpm = exitSpeedPerRpm;
        this.shooterOffsetM = shooterOffsetM;
        this.feedDelayS = feedDelayS;
        this.minBandRpm = minBandRpm;
        this.halfOpeningM = openingWidthM / 2 - lateralClearanceM;
        this.fallbackRpm = fallbackRpm;
    }

    /**
     * @param x, y, headingRad robot pose, field frame, metres and radians
     * @param vx, vy           robot velocity, field frame, m/s
     * @param targetX, targetY Cell opening centre, field frame
     */
    public Shot solve(double x, double y, double headingRad, double vx, double vy, double targetX, double targetY) {
        // 1. Where the robot will be when the ball leaves.
        double px = x + vx * feedDelayS;
        double py = y + vy * feedDelayS;
        // 2. Where the ball leaves from.
        double sx = px + shooterOffsetM * Math.cos(headingRad);
        double sy = py + shooterOffsetM * Math.sin(headingRad);
        double dx = targetX - sx;
        double dy = targetY - sy;
        double d = Math.hypot(dx, dy);
        double bearing = Math.atan2(dy, dx);
        // 3. Robot velocity along and across the line to the Cell.
        double ux = dx / d;
        double uy = dy / d;
        double radial = vx * ux + vy * uy;
        double tangential = -vx * uy + vy * ux;
        // 4. Table lookup.
        double tableRpm = interpolate(dist, vel, rpm, d, radial);
        double tableBand = interpolate(dist, vel, band, d, radial);
        double tableTof = interpolate(dist, vel, tof, d, radial);
        double stationary = interpolate(dist, vel, rpm, Math.max(dist[0], Math.min(dist[dist.length - 1], d)), 0);
        double idle = Double.isNaN(stationary) ? fallbackRpm : stationary;
        double tolerance = Math.atan(halfOpeningM / d);

        String reason = "OK";
        if (d < dist[0] || d > dist[dist.length - 1]) reason = "OUT OF RANGE";
        else if (radial > vel[vel.length - 1]) reason = "CLOSING TOO FAST";
        else if (radial < vel[0]) reason = "BACKING TOO FAST";
        else if (Double.isNaN(tableRpm) || Double.isNaN(tableBand) || tableBand < minBandRpm) reason = "NO SHOT";
        if (!reason.equals("OK")) {
            return new Shot(false, reason, Double.NaN, idle, bearing, tolerance, d, radial, tangential, Double.NaN, tableBand);
        }
        // 5. Heading lead so exit velocity plus robot velocity points along the bearing.
        double horizontalExit = tableRpm * exitSpeedPerRpm * Math.cos(launchAngleRad);
        double lead = Math.asin(Math.max(-1, Math.min(1, tangential / horizontalExit)));
        return new Shot(true, reason, tableRpm, idle, wrap(bearing - lead), tolerance, d, radial, tangential, tableTof, tableBand);
    }

    /** Bilinear interpolation on a grid v[xIndex][yIndex]. NaN outside the grid or beside a NaN corner. */
    static double interpolate(double[] xs, double[] ys, double[][] v, double x, double y) {
        if (x < xs[0] || x > xs[xs.length - 1] || y < ys[0] || y > ys[ys.length - 1]) return Double.NaN;
        int i = upperIndex(xs, x);
        int j = upperIndex(ys, y);
        double tx = (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
        double ty = (y - ys[j - 1]) / (ys[j] - ys[j - 1]);
        double a = v[i - 1][j - 1], b = v[i][j - 1], c = v[i - 1][j], e = v[i][j];
        if (Double.isNaN(a) || Double.isNaN(b) || Double.isNaN(c) || Double.isNaN(e)) return Double.NaN;
        double low = a + (b - a) * tx;
        double high = c + (e - c) * tx;
        return low + (high - low) * ty;
    }

    /** Index i in [1, n-1] with axis[i-1] <= value <= axis[i]. */
    private static int upperIndex(double[] axis, double value) {
        int i = 1;
        while (i < axis.length - 1 && axis[i] < value) i++;
        return i;
    }

    /** Wraps an angle to (-pi, pi]. */
    static double wrap(double rad) {
        double r = rad % (2 * Math.PI);
        if (r <= -Math.PI) r += 2 * Math.PI;
        if (r > Math.PI) r -= 2 * Math.PI;
        return r;
    }
}
```

- [ ] **Step 4: Run the tests**

```bash
./gradlew :TeamCode:testDebugUnitTest --tests '*ShotSolverTest' --no-daemon
```

Expected: `BUILD SUCCESSFUL`, 17 tests passed.

- [ ] **Step 5: Commit**

```bash
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/ShotSolver.java TeamCode/src/test/java/org/firstinspires/ftc/teamcode/ShotSolverTest.java
git commit -m "feature: shot solver with table lookup and heading lead, tested"
```

---

### Task 5: `TargetTracker`, tested

**Files:**
- Create: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/TargetTracker.java`
- Test: `TeamCode/src/test/java/org/firstinspires/ftc/teamcode/TargetTrackerTest.java`

**Interfaces:**
- Produces:
  - `TargetTracker(int capacity, double camForwardM, double camLeftM, double camYawRad, double camPitchRad, double expiryS)`
  - `void recordPose(long nanos, double x, double y, double headingRad)`
  - `boolean update(long frameNanos, double camXRightM, double camYForwardM, double camZUpM)`: camera-frame position of the Cell centre from `ftcPose` (x right, y forward along the lens axis, z up), returns false when no pose has been recorded yet.
  - `boolean hasTarget(long nowNanos)`, `double getX()`, `double getY()`, `double ageS(long nowNanos)`.

- [ ] **Step 1: Write the failing tests**

```java
package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class TargetTrackerTest {
    private static final double EPS = 1e-9;
    private static final long S = 1_000_000_000L;

    @Test
    public void noPoseYetMeansNoUpdate() {
        TargetTracker t = new TargetTracker(8, 0, 0, 0, 0, 5);
        assertFalse(t.update(0, 0, 2, 0));
        assertFalse(t.hasTarget(0));
    }

    @Test
    public void placesTargetFromCameraFrameAheadOfRobot() {
        TargetTracker t = new TargetTracker(8, 0, 0, 0, 0, 5);
        t.recordPose(0, 1, 1, 0);
        assertTrue(t.update(0, 0, 2, 0));
        assertEquals(3, t.getX(), EPS);
        assertEquals(1, t.getY(), EPS);
    }

    @Test
    public void usesPoseAtFrameTimeNotLatest() {
        TargetTracker t = new TargetTracker(8, 0, 0, 0, 0, 5);
        t.recordPose(0, 0, 0, 0);
        t.recordPose(S, 1, 0, 0);          // robot moved 1 m forward in the second after the frame
        t.update(0, 0, 2, 0);              // frame taken at t = 0 saw the Cell 2 m ahead
        assertEquals(2, t.getX(), EPS);
    }

    @Test
    public void picksNearestPoseInTime() {
        TargetTracker t = new TargetTracker(8, 0, 0, 0, 0, 5);
        t.recordPose(0, 0, 0, 0);
        t.recordPose(S, 1, 0, 0);
        t.update(S - S / 10, 0, 2, 0);     // 0.9 s: nearer the second pose
        assertEquals(3, t.getX(), EPS);
    }

    @Test
    public void ringBufferKeepsOnlyTheLastCapacityPoses() {
        TargetTracker t = new TargetTracker(2, 0, 0, 0, 0, 5);
        t.recordPose(0, 0, 0, 0);
        t.recordPose(S, 1, 0, 0);
        t.recordPose(2 * S, 2, 0, 0);      // evicts the t = 0 pose
        t.update(0, 0, 2, 0);              // nearest surviving pose is t = 1 s
        assertEquals(3, t.getX(), EPS);
    }

    @Test
    public void appliesRobotHeading() {
        TargetTracker t = new TargetTracker(8, 0, 0, 0, 0, 5);
        t.recordPose(0, 0, 0, Math.PI / 2);   // robot facing +y
        t.update(0, 0, 2, 0);
        assertEquals(0, t.getX(), EPS);
        assertEquals(2, t.getY(), EPS);
    }

    @Test
    public void appliesCameraOffsetAndYaw() {
        // Camera 0.1 m forward of centre, turned 90 deg left. Cell 1 m ahead of the camera is 1 m to the robot's left.
        TargetTracker t = new TargetTracker(8, 0.1, 0, Math.PI / 2, 0, 5);
        t.recordPose(0, 0, 0, 0);
        t.update(0, 0, 1, 0);
        assertEquals(0.1, t.getX(), EPS);
        assertEquals(1, t.getY(), EPS);
    }

    @Test
    public void cameraXRightIsRobotNegativeY() {
        TargetTracker t = new TargetTracker(8, 0, 0, 0, 0, 5);
        t.recordPose(0, 0, 0, 0);
        t.update(0, 0.5, 2, 0);
        assertEquals(-0.5, t.getY(), EPS);
    }

    @Test
    public void pitchLevelsTheRange() {
        // Camera pitched up 30 deg. A Cell 1 m along the lens axis is cos(30) ahead on the floor plan.
        TargetTracker t = new TargetTracker(8, 0, 0, 0, Math.toRadians(30), 5);
        t.recordPose(0, 0, 0, 0);
        t.update(0, 0, 1, 0);
        assertEquals(Math.cos(Math.toRadians(30)), t.getX(), EPS);
        // A point straight up the camera's z axis is behind the lens axis on the floor plan.
        t.update(0, 0, 0, 1);
        assertEquals(-Math.sin(Math.toRadians(30)), t.getX(), EPS);
    }

    @Test
    public void expiresAfterConfiguredAge() {
        TargetTracker t = new TargetTracker(8, 0, 0, 0, 0, 5);
        t.recordPose(0, 0, 0, 0);
        t.update(0, 0, 2, 0);
        assertTrue(t.hasTarget(4 * S));
        assertEquals(4, t.ageS(4 * S), EPS);
        assertFalse(t.hasTarget(6 * S));
    }
}
```

- [ ] **Step 2: Run the tests to see them fail**

```bash
./gradlew :TeamCode:testDebugUnitTest --tests '*TargetTrackerTest' --no-daemon
```

Expected: compile failure, `cannot find symbol TargetTracker`.

- [ ] **Step 3: Write `TargetTracker.java`**

```java
package org.firstinspires.ftc.teamcode;

/**
 * Holds the up-facing Cell's opening centre in field coordinates.
 *
 * Camera frames arrive late, so each detection is placed using the robot pose recorded nearest the
 * frame's timestamp, not the current pose. Between detections the stored point stands and the Pinpoint
 * carries the aim. The Cells move when the Hive tips, so nothing here is a field constant. Pure maths.
 */
public final class TargetTracker {
    private final long[] poseNanos;
    private final double[] poseX;
    private final double[] poseY;
    private final double[] poseHeading;
    private int next;
    private int count;

    private final double camForwardM;
    private final double camLeftM;
    private final double camYawRad;
    private final double camPitchRad;
    private final long expiryNanos;

    private double targetX;
    private double targetY;
    private long seenNanos;
    private boolean seen;

    /**
     * @param capacity    poses kept; 50 covers a second at a 50 Hz loop
     * @param camForwardM camera lens forward of the robot centre
     * @param camLeftM    camera lens left of the robot centre
     * @param camYawRad   camera turned left of robot forward
     * @param camPitchRad camera tilted up from level
     * @param expiryS     how long a detection stays usable
     */
    public TargetTracker(int capacity, double camForwardM, double camLeftM, double camYawRad, double camPitchRad, double expiryS) {
        poseNanos = new long[capacity];
        poseX = new double[capacity];
        poseY = new double[capacity];
        poseHeading = new double[capacity];
        this.camForwardM = camForwardM;
        this.camLeftM = camLeftM;
        this.camYawRad = camYawRad;
        this.camPitchRad = camPitchRad;
        this.expiryNanos = (long) (expiryS * 1e9);
    }

    /** Call once per loop with the Pinpoint pose. */
    public void recordPose(long nanos, double x, double y, double headingRad) {
        poseNanos[next] = nanos;
        poseX[next] = x;
        poseY[next] = y;
        poseHeading[next] = headingRad;
        next = (next + 1) % poseNanos.length;
        if (count < poseNanos.length) count++;
    }

    /**
     * Places the Cell from a detection. Camera frame per the SDK's ftcPose: x right, y forward along the
     * lens axis, z up. Returns false if no pose has been recorded yet.
     */
    public boolean update(long frameNanos, double camXRightM, double camYForwardM, double camZUpM) {
        if (count == 0) return false;
        int p = nearestPose(frameNanos);

        // Camera frame to a level frame: undo the pitch, then x-right becomes y-left negative.
        double forward = camYForwardM * Math.cos(camPitchRad) - camZUpM * Math.sin(camPitchRad);
        double left = -camXRightM;
        // Level camera frame to robot frame: yaw, then lens offset.
        double rx = camForwardM + forward * Math.cos(camYawRad) - left * Math.sin(camYawRad);
        double ry = camLeftM + forward * Math.sin(camYawRad) + left * Math.cos(camYawRad);
        // Robot frame to field frame with the pose at the frame time.
        double h = poseHeading[p];
        targetX = poseX[p] + rx * Math.cos(h) - ry * Math.sin(h);
        targetY = poseY[p] + rx * Math.sin(h) + ry * Math.cos(h);
        seenNanos = frameNanos;
        seen = true;
        return true;
    }

    private int nearestPose(long nanos) {
        int best = 0;
        long bestGap = Long.MAX_VALUE;
        for (int i = 0; i < count; i++) {
            long gap = Math.abs(poseNanos[i] - nanos);
            if (gap < bestGap) {
                bestGap = gap;
                best = i;
            }
        }
        return best;
    }

    public boolean hasTarget(long nowNanos) {
        return seen && nowNanos - seenNanos <= expiryNanos;
    }

    public double getX() {
        return targetX;
    }

    public double getY() {
        return targetY;
    }

    public double ageS(long nowNanos) {
        return seen ? (nowNanos - seenNanos) / 1e9 : Double.POSITIVE_INFINITY;
    }
}
```

- [ ] **Step 4: Run the tests**

```bash
./gradlew :TeamCode:testDebugUnitTest --tests '*TargetTrackerTest' --no-daemon
```

Expected: `BUILD SUCCESSFUL`, 10 tests passed.

- [ ] **Step 5: Commit**

```bash
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/TargetTracker.java TeamCode/src/test/java/org/firstinspires/ftc/teamcode/TargetTrackerTest.java
git commit -m "feature: target tracker placing the cell from timestamped detections, tested"
```

---

### Task 6: Constants, shooter target RPM, odometry velocity, vision position

**Files:**
- Modify: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Constants.java`
- Modify: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/ShooterSubsystem.java`
- Modify: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/OdometrySubsystem.java`
- Modify: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/VisionSubsystem.java`

**Interfaces:**
- Consumes: the teleop plan's versions of these files.
- Produces: `Constants.FEED_DELAY_S`, `CAMERA_FORWARD_M`, `CAMERA_LEFT_M`, `CAMERA_YAW_RAD`, `CAMERA_PITCH_RAD`, `TARGET_EXPIRY_S`, `POSE_BUFFER_SIZE`, `SHOOTER_TRIM_STEP_RPM`, `SHOT_LOG_DIR`; `ShooterSubsystem.setTargetRpm(double)`, `getTargetRpm()`, `trim(double)`, `getTrimRpm()`; `OdometrySubsystem.getVelocity()` → `double[]{vx, vy}` m/s field frame; `VisionSubsystem.getTargetX()`, `getTargetY()`, `getTargetZ()` (metres, camera frame), `getTargetFrameNanos()`.

- [ ] **Step 1: Add to `Constants.java`**

Change the shooter tolerance. The generated table only has cells where the RPM band is at least twice this, and the Cell geometry leaves a 125 to 175 RPM band, so 100 RPM would reject every cell:

```java
    public static final double SHOOTER_TOLERANCE_RPM = 50.0;   // the shot table needs the wheel within this; see tools/shots/README.md
```

Then append inside the class, after the aim-assist block:

```java
    // Shoot on the move. Shooter geometry and the table live in the generated ShotTable.
    public static final double FEED_DELAY_S = 0.15;          // indexer feed command to ball exit, measure on the robot
    public static final double SHOOTER_TRIM_STEP_RPM = 50.0;  // D-pad up/down
    public static final double TARGET_EXPIRY_S = 5.0;         // how long the last tag fix is trusted
    public static final int POSE_BUFFER_SIZE = 50;            // about one second of loops

    // Camera lens relative to the robot centre. Fill in from the CAD once the mount is designed.
    public static final double CAMERA_FORWARD_M = 0.0;
    public static final double CAMERA_LEFT_M = 0.0;
    public static final double CAMERA_YAW_RAD = 0.0;          // positive turned left
    public static final double CAMERA_PITCH_RAD = 0.0;        // positive tilted up

    public static final String SHOT_LOG_DIR = "/sdcard/FIRST/shots";
```

- [ ] **Step 2: Make the shooter target adjustable in `ShooterSubsystem.java`**

Replace the field `private boolean running;` with:

```java
    private boolean running;
    private double targetRpm = Constants.SHOOTER_SETPOINT_RPM;
    private double trimRpm = 0;
```

Replace `spinUp()` and `atSpeed()` with:

```java
    public void spinUp() {
        running = true;
        double tps = ShooterMath.rpmToTicksPerSecond(getTargetRpm(), Constants.SHOOTER_TICKS_PER_REV);
        left.setVelocity(tps);
        right.setVelocity(tps);
    }

    /** Changes the target. Re-sends the velocity command only if the target moved and the wheels are running. */
    public void setTargetRpm(double rpm) {
        if (rpm == targetRpm) return;
        targetRpm = rpm;
        if (running) spinUp();
    }

    /** Target plus the driver's trim. */
    public double getTargetRpm() {
        return targetRpm + trimRpm;
    }

    /** Driver adjustment applied to every target for the rest of the run. */
    public void trim(double deltaRpm) {
        trimRpm += deltaRpm;
        if (running) spinUp();
    }

    public double getTrimRpm() {
        return trimRpm;
    }

    /** True when both wheels are within tolerance of the current target. */
    public boolean atSpeed() {
        double target = getTargetRpm();
        return running
                && Math.abs(getLeftRpm() - target) < Constants.SHOOTER_TOLERANCE_RPM
                && Math.abs(getRightRpm() - target) < Constants.SHOOTER_TOLERANCE_RPM;
    }
```

- [ ] **Step 3: Expose velocity from `OdometrySubsystem.java`**

Add after `getPose()`:

```java
    /** Field-frame velocity from the Pinpoint, metres per second, as {x, y}. */
    public double[] getVelocity() {
        return new double[]{pinpoint.getVelX(DistanceUnit.METER), pinpoint.getVelY(DistanceUnit.METER)};
    }
```

- [ ] **Step 4: Expose the target position and frame time from `VisionSubsystem.java`**

In the constructor, change the processor builder to output metres:

```java
        processor = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)
                .build();
```

Add imports:

```java
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
```

Add after `getBearingDeg()`:

```java
    /** Cell centre right of the lens, metres. Only valid when hasTarget(). */
    public double getTargetX() {
        return target.ftcPose.x;
    }

    /** Cell centre forward along the lens axis, metres. Only valid when hasTarget(). */
    public double getTargetY() {
        return target.ftcPose.y;
    }

    /** Cell centre above the lens axis, metres. Only valid when hasTarget(). */
    public double getTargetZ() {
        return target.ftcPose.z;
    }

    /** System.nanoTime() when the frame holding this detection was captured. */
    public long getTargetFrameNanos() {
        return target.frameAcquisitionNanoTime;
    }
```

- [ ] **Step 5: Build and run all tests**

```bash
./gradlew :TeamCode:assembleDebug :TeamCode:testDebugUnitTest --no-daemon
```

Expected: `BUILD SUCCESSFUL`.

- [ ] **Step 6: Commit**

```bash
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode
git commit -m "feature: adjustable shooter target with trim, odometry velocity, vision target position"
```

---

### Task 7: `ShotLog` CSV writer

**Files:**
- Create: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/ShotLog.java`

**Interfaces:**
- Produces: `ShotLog.open(String dir)` → `ShotLog` or `null` if the file cannot be created; `void row(double... values)`; `void close()`; `String fileName()`. Column order is fixed by `HEADER`.

- [ ] **Step 1: Write `ShotLog.java`**

```java
package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * One CSV per OpMode run under /sdcard/FIRST/shots, pulled off the hub with adb after practice.
 * Wi-Fi dashboards are banned at events (R704), so this is the tuning record.
 */
public final class ShotLog {
    public static final String HEADER = "t_s,x_m,y_m,heading_rad,vx_mps,vy_mps,dist_m,radial_mps,tangential_mps,"
            + "table_rpm,left_rpm,right_rpm,heading_err_rad,valid,fired";
    private static final int FLUSH_EVERY = 50;

    private final BufferedWriter out;
    private final String fileName;
    private final long startNanos = System.nanoTime();
    private int rows;

    private ShotLog(BufferedWriter out, String fileName) {
        this.out = out;
        this.fileName = fileName;
    }

    /** Creates the directory and a timestamped file. Returns null, and never throws, if that fails. */
    @SuppressLint("SimpleDateFormat")
    public static ShotLog open(String dir) {
        try {
            File folder = new File(dir);
            if (!folder.isDirectory() && !folder.mkdirs()) return null;
            String name = new SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(new Date()) + ".csv";
            BufferedWriter w = new BufferedWriter(new FileWriter(new File(folder, name)));
            w.write(HEADER);
            w.newLine();
            return new ShotLog(w, name);
        } catch (IOException e) {
            return null;
        }
    }

    /** Appends one row. The first column, elapsed seconds, is added here. */
    public void row(double... values) {
        try {
            StringBuilder sb = new StringBuilder();
            sb.append(String.format(Locale.US, "%.3f", (System.nanoTime() - startNanos) / 1e9));
            for (double v : values) sb.append(',').append(String.format(Locale.US, "%.4f", v));
            out.write(sb.toString());
            out.newLine();
            if (++rows % FLUSH_EVERY == 0) out.flush();
        } catch (IOException ignored) {
            // A failed log line must never stop the robot.
        }
    }

    public void close() {
        try {
            out.flush();
            out.close();
        } catch (IOException ignored) {
        }
    }

    public String fileName() {
        return fileName;
    }
}
```

- [ ] **Step 2: Build**

```bash
./gradlew :TeamCode:assembleDebug --no-daemon
```

Expected: `BUILD SUCCESSFUL`.

- [ ] **Step 3: Commit**

```bash
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/ShotLog.java
git commit -m "feature: CSV shot log on the hub"
```

---

### Task 8: Wire it into `Robot`

**Files:**
- Modify: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Robot.java`

**Interfaces:**
- Consumes: `ShotTable` (Task 2), `ShotSolver` (Task 4), `TargetTracker` (Task 5), `Constants`, `ShooterSubsystem`, `OdometrySubsystem`, `VisionSubsystem` (Task 6), `ShotLog` (Task 7), and the teleop plan's `Robot` with `aimRotation(double)`, `intake`, `indexer`, `battery`.
- Produces: the driver-facing behaviour in the README controls table (Task 9).

- [ ] **Step 1: Add fields and construction**

Add imports:

```java
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
```

Add fields after `public final VisionSubsystem vision;`:

```java
    private final ShotSolver solver = new ShotSolver(
            ShotTable.DIST_M, ShotTable.VEL_MPS, ShotTable.RPM, ShotTable.BAND_RPM, ShotTable.TOF_S,
            ShotTable.LAUNCH_ANGLE_DEG, ShotTable.EXIT_SPEED_PER_RPM, ShotTable.SHOOTER_OFFSET_M,
            Constants.FEED_DELAY_S, ShotTable.MIN_BAND_RPM, ShotTable.OPENING_WIDTH_M, ShotTable.LATERAL_CLEARANCE_M,
            Constants.SHOOTER_SETPOINT_RPM);
    private final TargetTracker tracker = new TargetTracker(
            Constants.POSE_BUFFER_SIZE, Constants.CAMERA_FORWARD_M, Constants.CAMERA_LEFT_M,
            Constants.CAMERA_YAW_RAD, Constants.CAMERA_PITCH_RAD, Constants.TARGET_EXPIRY_S);
    private ShotLog log;
    private ShotSolver.Shot lastShot;
```

- [ ] **Step 2: Open and close the log**

In `start()`, after `vision.stopLiveView();`:

```java
        log = ShotLog.open(Constants.SHOT_LOG_DIR);
```

In `stop()`, before `shooter.idle();`:

```java
        if (log != null) log.close();
```

- [ ] **Step 3: Replace `periodic()`**

Replace the whole method with:

```java
    /** Called repeatedly while the OpMode is running. */
    public void periodic() {
        driver.readButtons();
        CommandScheduler.getInstance().run();
        long now = System.nanoTime();

        if (driver.wasJustPressed(GamepadKeys.Button.A)) odometry.resetHeading();
        if (driver.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) shooter.toggle();
        if (driver.wasJustPressed(GamepadKeys.Button.DPAD_UP)) shooter.trim(Constants.SHOOTER_TRIM_STEP_RPM);
        if (driver.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) shooter.trim(-Constants.SHOOTER_TRIM_STEP_RPM);

        // Where we are, how fast we are going, and where the Cell is.
        Pose2D pose = odometry.getPose();
        double x = pose.getX(DistanceUnit.METER);
        double y = pose.getY(DistanceUnit.METER);
        double heading = pose.getHeading(AngleUnit.RADIANS);
        double[] vel = odometry.getVelocity();
        tracker.recordPose(now, x, y, heading);
        if (vision.hasTarget()) {
            tracker.update(vision.getTargetFrameNanos(), vision.getTargetX(), vision.getTargetY(), vision.getTargetZ());
        }

        // Aim. Y held: solver heading if we know where the Cell is, else the raw tag bearing, else the stick.
        boolean aiming = driver.isDown(GamepadKeys.Button.Y);
        double rotate = driver.getRightX();
        ShotSolver.Shot shot = null;
        if (aiming && tracker.hasTarget(now)) {
            shot = solver.solve(x, y, heading, vel[0], vel[1], tracker.getX(), tracker.getY());
            shooter.setTargetRpm(shot.valid ? shot.rpm : shot.idleRpm);
            rotate = aimRotation(Math.toDegrees(shot.headingErrorRad(heading)));
        } else if (aiming && vision.hasTarget()) {
            rotate = aimRotation(vision.getBearingDeg());
        }
        lastShot = shot;

        boolean fieldCentric = !driver.isDown(GamepadKeys.Button.LEFT_BUMPER);
        drive.drive(driver.getLeftY(), driver.getLeftX(), rotate, fieldCentric, heading);

        // Fire gate. With a solved shot: valid, at speed, and pointing inside the Cell width.
        boolean fire = driver.isDown(GamepadKeys.Button.X);
        boolean ready = shot == null
                ? shooter.atSpeed()
                : shot.valid && shooter.atSpeed() && Math.abs(shot.headingErrorRad(heading)) < shot.headingToleranceRad;
        double rightTrigger = driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        double leftTrigger = driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        boolean firing = false;
        if (leftTrigger > Constants.TRIGGER_THRESHOLD) {
            intake.reverse();
            indexer.reverse();
        } else if (rightTrigger > Constants.TRIGGER_THRESHOLD) {
            intake.run();
            indexer.feed();
        } else if (fire && ready) {
            intake.stop();
            indexer.feed();
            firing = true;
        } else {
            intake.stop();
            indexer.stop();
        }

        if (shot != null && log != null) {
            log.row(x, y, heading, vel[0], vel[1], shot.distanceM, shot.radialVel, shot.tangentialVel,
                    shot.valid ? shot.rpm : shot.idleRpm, shooter.getLeftRpm(), shooter.getRightRpm(),
                    shot.headingErrorRad(heading), shot.valid ? 1 : 0, firing ? 1 : 0);
        }

        telemetry.addData("Alliance", vision.getAlliance());
        telemetry.addData("Pose", "%.2f %.2f m  %.0f deg  v %.2f %.2f", x, y, Math.toDegrees(heading), vel[0], vel[1]);
        telemetry.addData("Shooter", "%s target %.0f (trim %+.0f)  L %.0f  R %.0f  %s",
                shooter.isRunning() ? "ON" : "off", shooter.getTargetRpm(), shooter.getTrimRpm(),
                shooter.getLeftRpm(), shooter.getRightRpm(), shooter.atSpeed() ? "AT SPEED" : "");
        telemetry.addData("Target", tracker.hasTarget(now)
                ? String.format("%.2f %.2f m  seen %.1f s ago", tracker.getX(), tracker.getY(), tracker.ageS(now))
                : vision.hasTarget() ? "tag only, no pose" : "none");
        telemetry.addData("Shot", shot == null ? "hold Y" : shot.valid
                ? String.format("OK d %.2f  radial %+.2f  tang %+.2f  rpm %.0f  err %+.1f deg  tol %.1f",
                        shot.distanceM, shot.radialVel, shot.tangentialVel, shot.rpm,
                        Math.toDegrees(shot.headingErrorRad(heading)), Math.toDegrees(shot.headingToleranceRad))
                : String.format("NO SHOT: %s  d %.2f  radial %+.2f", shot.reason, shot.distanceM, shot.radialVel));
        telemetry.addData("Log", log == null ? "not writing" : log.fileName());
        telemetry.addData("Battery", "%.1f V", battery.getVoltage());
        telemetry.update();
    }
```

`aimRotation(double bearingDeg)` from the teleop plan stays as it is: bearing positive left gives clockwise-negative rotation, which is the same sign as `headingErrorRad`.

- [ ] **Step 4: Build and run all tests**

```bash
./gradlew :TeamCode:assembleDebug :TeamCode:testDebugUnitTest --no-daemon
```

Expected: `BUILD SUCCESSFUL`.

- [ ] **Step 5: Commit**

```bash
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Robot.java
git commit -m "feature: solver-driven aim, fire gate, RPM trim and shot logging"
```

---

### Task 9: Tools README, measurement record and repo README

**Files:**
- Create: `tools/shots/README.md`
- Modify: `README.md`

**Interfaces:**
- Consumes: everything above. No code produced.

- [ ] **Step 1: Write `tools/shots/README.md`**

```markdown
# Shot table tools

Lebob Robotics, FTC 29550 · BIOBUZZ 2026/27 · v1.0, 20 Sep 2026

The robot cannot change its launch angle, so whether a shot lands while
driving depends on distance and on how fast the robot is closing on the Cell.
These tools work that out on a laptop and give the robot a table.

## Files

- `shots.js`: the physics, the sweep and the Java export. The only copy.
- `index.html`: the visualiser. Open it in a browser (it fetches Plotly from
  the internet the first time). Change inputs, press Compute, move the
  sliders, export.
- `export.js`: `node tools/shots/export.js` regenerates
  `TeamCode/.../ShotTable.java` from `inputs.json`. Run it after changing
  `inputs.json` and commit both files together.
- `inputs.json`: the measured constants behind the committed table.

## What the three panels show

1. Trajectory fan. Every flywheel speed at the chosen distance and radial
   velocity. Green arcs score, red miss, the blue one is what the robot will
   use. The black segment is the Cell opening, the dotted line its front face.
2. RPM band against radial velocity at that distance. The green fill is the
   set of speeds that score. Where it pinches shut is the closing speed the
   driver cannot exceed.
3. Heatmap of the table. Grey cells have no shot.

## Measurements

Record here, then copy into `inputs.json` and regenerate.

| Input | How to measure | Value | Date | Who |
| --- | --- | --- | --- | --- |
| `launchAngleDeg` | From the CAD, checked with a protractor on the exit guide | | | |
| `exitHeightM` | Tape from tiles to the ball centre at exit | | | |
| `exitSpeedPerRpm` | Fire Pollen at 3000, 4000, 5000 RPM from a fixed spot, slow-motion video past a metre rule, exit speed over RPM, fit a line through zero | | | |
| `dragCd` | Compare the landing distance of those shots to the page's prediction, adjust until they match. Start at 0.45 | | | |
| `shooterOffsetM` | Ball exit point forward of the robot's tracking point | | | |
| `openingTiltDeg`, `openingLengthM`, `lipHeightM` | Competition Manual Figure 9-10 and the field CAD | | | |
| `Constants.FEED_DELAY_S` | Video the indexer starting and the ball leaving, or count loops to the shooter current spike | | | |
| `Constants.CAMERA_*` | From the CAD once the mount exists | | | |

Time of flight at two distances, from the same video, checked against the
table's `TOF_S` column.

## Why the tolerance is 50 RPM

With a fixed launch angle the near lip sets the lowest speed that scores and
the far edge of the 14 in opening sets the highest. With the current inputs
that window is about 0.3 m/s of ball speed, 125 to 175 RPM, at every distance
and for any launch angle from 45° to 75°. The table only keeps cells whose band
is at least twice `rpmToleranceRpm`, so the flywheel must hold ±50 RPM
(`Constants.SHOOTER_TOLERANCE_RPM`). Tune the shooter PIDF until the logged
`left_rpm` and `right_rpm` sit inside that at a steady target before spending
time on moving shots. A bigger window needs a hood, which is a hardware
conversation.

## Reading the logs

Each run writes `/sdcard/FIRST/shots/<date-time>.csv` on the hub while Y is
held. Pull them with:

    adb pull /sdcard/FIRST/shots ./shots

Columns are in `ShotLog.HEADER`. `valid` and `fired` are 0 or 1. A shot that
missed while `valid` was 1 and `heading_err_rad` was inside tolerance means the
table is off at that `dist_m` and `radial_mps`: check `exitSpeedPerRpm` and
`dragCd` first.
```

- [ ] **Step 2: Update the repo `README.md`**

In the driver controls table added by the teleop plan, change the Y and X rows and add the D-pad row:

```markdown
| Y (hold) | Aim: rotation tracks the solved shot heading, shooter runs at the table RPM. Falls back to tag bearing if the Cell position is unknown |
| X (hold) | Fire: feeds only when at speed, and when aiming also only with a valid shot and heading inside the Cell width |
| D-pad up / down | Trim every shooter target by ±50 RPM for the rest of the run |
```

After the on-robot checklist, add:

```markdown
### Shoot on the move

Tools and the measurement procedure are in `tools/shots/README.md`. Verify in
this order, recording hits in the PR:

0. Velocity frame: strafe left with the robot facing +X and confirm the Pose
   telemetry shows `v` growing in the second component, then turn 90° and
   repeat; the same component must still grow. If it swaps, the Pinpoint
   reports robot-frame velocity and `OdometrySubsystem.getVelocity()` must
   rotate it by the heading before the solver sees it.
1. Stationary at 1.0, 1.5, 2.0 and 2.5 m: at least 8 of 10 Pollen in. Fix the
   table inputs before moving on.
2. Strafing across the Cell at a steady speed: 7 of 10.
3. Driving toward and away at a steady speed: 7 of 10, and "NO SHOT: CLOSING
   TOO FAST" appears near the speed the band chart predicts.
4. Free driving: log, video, count, fix what the log shows.
```

- [ ] **Step 3: Commit**

```bash
git add tools/shots/README.md README.md
git commit -m "docs: shot table tools guide, measurement record and on-robot ladder"
```

---

## Testing on the robot

Everything above compiles and passes JVM tests without a robot. What needs the robot, in order:

1. **Rung 0, velocity frame.** Described in the README addition. Do this before any shooting; a frame mistake makes every moving shot wrong in a way that looks like a bad table.
2. **Measurements** from `tools/shots/README.md`, then `node tools/shots/export.js`, then commit `inputs.json` and `ShotTable.java` together.
3. **Rungs 1 to 4** from the README, each recorded in the PR under "Tested on the robot".
4. **Camera view while leading.** At rung 2, note on telemetry whether "Target" flips to "tag only" or "none" during the lead. If it expires mid-shot, the fix is a wider lens or a second camera, which is outside this plan.

## After the plan

- Pedro Pathing: `Robot` reads pose and velocity through `OdometrySubsystem`; when Pedro's localiser replaces it, keep those two getters and nothing else changes.
- Nectar: the runtime uses the Pollen table. A second generated table and a ball sensor would be a new spec.
- If logs show misses on direction changes, the next step is an acceleration term in `ShotSolver.solve`, not a bigger table.
