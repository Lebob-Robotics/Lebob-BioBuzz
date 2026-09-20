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
      '    public static final double MIN_BAND_RPM = ' + num(2 * p.rpmToleranceRpm) + ';\n' +
      '    /** The table\'s distances are to the near lip; the tag cluster origin is the opening centre, this far beyond it. */\n' +
      '    public static final double LIP_TO_CENTRE_M = ' + num(p.openingLengthM * Math.cos(p.openingTiltDeg * Math.PI / 180) / 2) + ';\n\n' +
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

  // Two checks that fail loudly if the physics is broken, or if the operator's inputs leave no usable
  // shot. Export is refused when this returns anything.
  function selfCheck(p0) {
    const base = Object.assign({}, DEFAULTS, p0);
    const fails = [];
    // 1. With no drag the flight must match the closed-form parabola to 1 mm.
    const p = Object.assign({}, base, { dragCd: 0, lipHeightM: 100 });  // nothing to hit
    const s = simulate(3000, 0.3, 2, p);
    const a = p.launchAngleDeg * Math.PI / 180, v0 = 3000 * p.exitSpeedPerRpm;
    for (const frac of [0.25, 0.5, 0.75]) {
      const n = Math.floor((s.points.length - 1) * frac), t = n * p.dtS;
      const ex = (v0 * Math.cos(a) + 0.3) * t, ey = p.exitHeightM + v0 * Math.sin(a) * t - 0.5 * G * t * t;
      const err = Math.hypot(s.points[n][0] - ex, s.points[n][1] - ey);
      if (err > 0.001) fails.push('no-drag flight is ' + (err * 1000).toFixed(2) + ' mm off the parabola at t=' + t.toFixed(3) + ' s');
    }
    // 2. Standing still, at the operator's inputs, a longer shot needs more RPM and at least one exists.
    const g = solveGrid(Object.assign({}, base, { velMinMps: 0, velMaxMps: 0 }));
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
