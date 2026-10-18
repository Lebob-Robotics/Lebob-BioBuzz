#!/usr/bin/env node
// Flywheel mass optimiser: the lightest flywheel that keeps four sequential Pollen shots within
// the shooter's RPM tolerance. One rigid inertia on a linear DC motor curve; each ball takes a
// fixed share of its kinetic energy. Loaded by flywheel.html in a browser; `node tools/flywheel/flywheel.js`
// runs test.js then prints every shot case; `node tools/flywheel/flywheel.js hood` also runs the hood sweep.
//
// Model limits:
// - Both wheels are lumped at the main wheel's radius, so a heavy second wheel's inertia is overstated.
// - Friction, belt slip, battery sag during the burst, PID overshoot and hood scrape are not modelled.
//   If the hood presses hard on the ball, drop transferEfficiency to about 0.50.
// - The ff controller assumes the hub's P term is negligible at P 1.17 (tuning-guide units, output
//   counts per tick/s of error). Confirm from left_rpm and right_rpm in a logged four-ball burst.
// - Targets change between balls while the robot moves; the model holds one target for the burst.
(function (root, factory) {
  if (typeof module === 'object' && module.exports) module.exports = factory(require('../shots/hood.js'));
  else root.Flywheel = factory(root.Hood);
})(typeof self !== 'undefined' ? self : this, function (Hood) {
  'use strict';
  const TAU = 2 * Math.PI;

  // Mirrors inputs.json so the browser page has something to start from. Values marked estimate or
  // guess in the README must be measured before the result is trusted.
  const DEFAULTS = {
    flywheelMassKg: 0.200, wheelDiameterM: 0.0762, wheel2DiameterM: 0.0254, massDistributionK: 0.6, layout: 'dualhood',
    ballMassKg: 0.0249, ballDiameterM: 0.0711, ballShell: 'hollow', energySpreadPct: 20,
    exitSpeedPerRpm: 0.00227, transferEfficiency: 0.55, exitSpeedMps: 7.49, toleranceRpm: 50,
    motorFreeRpm: 6000, motorStallNm: 0.144, motorCount: 2, gearRatio: 0.7, batteryV: 12.0, drivetrainEfficiency: 0.90,
    wheel2Gear: 1, wheel2MaxRpm: 9000, shotIntervalS: 0.12, controller: 'full', controllerDelayS: 0.05, maxSpinupS: 1.5, existingWheelMassKg: 0, addRadiusM: 0.020,
  };

  // Motor presets at the motor shaft (free rpm, stall N m).
  const MOTORS = {
    'goBILDA 5203 1:1': { motorFreeRpm: 6000, motorStallNm: 0.144 },
    'goBILDA 5203 3.7:1': { motorFreeRpm: 1620, motorStallNm: 0.530 },
    'REV HD Hex bare': { motorFreeRpm: 6000, motorStallNm: 0.105 },
  };

  // Shot cases the CLI reports. Exit speeds are physical m/s, independent of exitSpeedPerRpm: the
  // fixed rows are ShotTable RPM x 0.0023, which stays the speed the trajectory needs when kpr is corrected.
  const CASES = [
    { name: 'Hood worst case, backing away 2.2 m/s at 1.45 m', vt: 7.06 },
    { name: 'Hood standing, 1.45 m', vt: 5.71 },
    { name: 'Fixed standing, 1.45 m', vt: 7.734 },
    { name: 'Fixed median table shot', vt: 6.756 },
    { name: 'Fixed backing away 0.7 m/s', vt: 10.178 },
    { name: 'Fixed backing away 0.9 m/s', vt: 11.27 },
    { name: 'Fixed table maximum, backing away 1.7 m/s', vt: 12.535 },
  ];

  // inputs.json keys -> the short names the physics uses. The one place the two meet.
  function map(i) {
    return {
      m: i.flywheelMassKg, D: i.wheelDiameterM, D2: i.wheel2DiameterM, G2: i.wheel2Gear == null ? 1 : i.wheel2Gear, w2Max: i.wheel2MaxRpm == null ? Infinity : i.wheel2MaxRpm, k: i.massDistributionK, layout: i.layout,
      mb: i.ballMassKg, ballShell: i.ballShell, varb: i.energySpreadPct, kpr: i.exitSpeedPerRpm, eta: i.transferEfficiency,
      vt: i.exitSpeedMps, tolRpm: i.toleranceRpm, free: i.motorFreeRpm, stall: i.motorStallNm, N: i.motorCount, G: i.gearRatio,
      V: i.batteryV, etaD: i.drivetrainEfficiency, dt: i.shotIntervalS, ctrl: i.controller, delay: i.controllerDelayS,
      maxSpin: i.maxSpinupS, mEx: i.existingWheelMassKg, rAdd: i.addRadiusM,
    };
  }

  function derive(p) {
    const R = p.D / 2;
    const two = p.layout !== 'hood';
    const shellK = p.ballShell === 'hollow' ? 2 / 3 : 0.4;
    const d2 = p.G2 * p.D2;                                         // second wheel's effective diameter: it turns G2 times faster
    const ds = two ? (p.D - d2) / (p.D + d2) : 1;
    const ceil = D2 => two ? Math.PI * (p.D + D2) / 2 / 60 : Math.PI * p.D / 60 * 0.5;  // no-slip ceiling on kpr
    p.lam = shellK * ds * ds;                                       // ball spin energy over linear energy
    p.ceil = ceil(d2);
    p.slip = p.kpr / ceil(p.D2);                                    // kpr is measured on the as-built 1:1 shooter
    p.kpr = p.slip * p.ceil;                                        // so it scales with the ceiling when the second wheel is geared
    p.rpmT = p.vt / p.kpr;
    p.s = p.kpr * 60 / (TAU * R);                                   // ball speed over wheel surface speed
    p.tol = p.tolRpm / p.rpmT * 100;                                // allowed error, %
    return p;
  }

  // One ball through the pinch. Exit speed uses the mean wheel speed over the contact; damped
  // fixed-point since the exit speed and the speed drop depend on each other.
  function shoot(p, I, R, wb, mb) {
    let wa = wb;
    for (let i = 0; i < 60; i++) {
      const v = p.s * R * (wb + wa) / 2;
      const dE = 0.5 * mb * v * v * (1 + p.lam) / p.eta;
      const w2 = wb * wb - 2 * dE / I;
      wa = 0.5 * (wa + (w2 > 0 ? Math.sqrt(w2) : 0));
    }
    return { wa, v: p.s * R * (wb + wa) / 2 };
  }

  // Motor curve seen at the flywheel. Battery scaling moves free speed and stall torque together.
  function motor(p) {
    const Vs = p.V / 12;
    return { wf: p.G * p.free * TAU / 60 * Vs, ts: p.N * p.etaD * p.stall * Vs / p.G };
  }

  // Four shots from a spun-up flywheel at mass m. Returns worst speed error (%), spin-up time and per-shot detail.
  function simulate(p, m) {
    const R = p.D / 2, I = p.k * m * R * R;
    const { wf, ts } = motor(p), T = I * wf / ts;
    const w0 = p.rpmT * TAU / 60;
    if (!(w0 < 0.97 * wf)) return { ok: false, I, T, wf, w0 };
    const vRef = shoot(p, I, R, w0, p.mb).v;
    const tolW = p.tolRpm * TAU / 60;
    const spin = p.ctrl === 'ff' ? T * Math.log(w0 / (tolW / 2)) : T * Math.log(wf / (wf - w0));
    const hi = 1 + p.varb / 100, lo = 1 - p.varb / 100;
    const shots = []; let wb = w0, err = 0;
    for (let i = 0; i < 4; i++) {
      const r = shoot(p, I, R, wb, p.mb);
      const rH = shoot(p, I, R, wb, p.mb * hi);
      const rL = shoot(p, I, R, wb, p.mb * lo);
      const e = Math.max(vRef - rH.v, rL.v - vRef, vRef - r.v) / vRef * 100;
      err = Math.max(err, e);
      let rec, wn;
      if (p.ctrl === 'ff') {          // holds the duty F gives at target: closes the gap at the motor's own rate
        rec = Math.max(0, T * Math.log((w0 - r.wa) / (tolW / 2)));
        wn = w0 - (w0 - r.wa) * Math.exp(-p.dt / T);
      } else {                        // coasts for the reaction delay, then full power up the motor curve
        rec = r.wa < w0 ? p.delay + T * Math.log((wf - r.wa) / (wf - w0)) : 0;
        const coast = Math.min(p.delay, p.dt), push = p.dt - coast;
        wn = push > 0 ? wf - (wf - r.wa) * Math.exp(-push / T) : r.wa;
      }
      shots.push({ wb, wa: r.wa, v: r.v, vH: rH.v, vL: rL.v, rec, e });
      wb = Math.min(wn, w0);
    }
    return { ok: true, I, T, wf, w0, spin, shots, err, vRef };
  }

  // 20 g to 2000 g in 10 g steps.
  function sweep(p) {
    const out = [];
    for (let i = 2; i <= 200; i++) {
      const m = i / 100, sim = simulate(p, m);
      out.push({ m, sim, pass: sim.ok && sim.err <= p.tol });
    }
    return out;
  }

  // Inertia is what matters: grams to bolt on at a usable radius. Existing wheels are solid discs.
  function addedWeight(p, mRec) {
    const R = p.D / 2, rimR = 0.9 * R;
    const Ireq = 1.15 * p.k * mRec * R * R, Iex = 0.5 * p.mEx * R * R;
    return { Ireq, Iex, atRadius: (Ireq - Iex) / (p.rAdd * p.rAdd), atRim: (Ireq - Iex) / (rimR * rimR), rimR };
  }

  // status: unreachable | none | existing | build. passKg is the lightest passing mass; spin and err are at
  // that mass (or at the existing wheels when they already pass). ranges lists every passing run, since
  // the ff error curve can pass at low mass (fast recovery) and again at high mass (small drop).
  function recommend(inputs) {
    const p = derive(map(inputs));
    const { wf } = motor(p);
    const out = { p, rpmT: p.rpmT, freeRpm: wf * 60 / TAU, freePct: p.rpmT / (wf * 60 / TAU) * 100, tol: p.tol };
    if (!(p.rpmT * TAU / 60 < 0.97 * wf)) return Object.assign(out, { status: 'unreachable', reason: 'motor' });
    if (p.rpmT * p.G2 > p.w2Max) return Object.assign(out, { status: 'unreachable', reason: 'wheel2' });
    const sw = sweep(p);
    out.sweep = sw;
    out.ranges = [];
    sw.forEach((r, i) => {
      if (!r.pass) return;
      if (i > 0 && sw[i - 1].pass) out.ranges[out.ranges.length - 1].hiKg = r.m; else out.ranges.push({ loKg: r.m, hiKg: r.m });
    });
    const first = sw.find(r => r.pass);
    if (!first) return Object.assign(out, { status: 'none' });
    out.passKg = first.m;
    // A pass narrower than 30 g sits on a knife edge (typically where recovery time equals the shot
    // interval) and moves with any input; robustKg is the start of the first range at least that wide.
    const robust = out.ranges.find(x => x.hiKg - x.loKg >= 0.02 - 1e-9);
    out.robustKg = robust ? robust.loKg : NaN;
    out.fragile = !robust || robust.loKg > first.m;
    const at = first.m <= p.mEx ? simulate(p, p.mEx) : first.sim;
    out.spin = at.spin; out.err = at.err; out.sim = at;
    out.slowSpin = at.spin > p.maxSpin;
    if (first.m <= p.mEx) return Object.assign(out, { status: 'existing' });
    out.buildKg = Math.ceil(1.15 * first.m / 0.005 - 1e-9) * 0.005;
    out.add = addedWeight(p, first.m);
    return Object.assign(out, { status: 'build' });
  }

  // Hood sweep with the realistic kpr and the full moving envelope; the worst chosen shot's RPM sets
  // the flywheel's design exit speed. hoodOver pins hoodMinDeg = hoodMaxDeg = 80 for the fixed comparison.
  function hoodCase(kpr, shotsInputs, hoodOver, onProgress) {
    const p = Object.assign({}, Hood.DEFAULTS, shotsInputs, {
      exitSpeedPerRpm: kpr, velMinMps: -2.2, velMaxMps: 2.2, velStepMps: 0.2,
      rpmMin: 1500, rpmMax: 9000, rpmStep: 50,
    }, hoodOver);
    const g = Hood.sweep(p, onProgress), w = Hood.windows(g);
    const shots = [];
    if (w.rec) for (const c of g.cells) {
      const b = Hood.bestIn(c, w.rec.lo, w.rec.hi);
      if (b) shots.push({ d: c.distM, v: c.velMps, angle: g.angles[b.ia], rpm: c.rpms[b.ir] });
    }
    const worst = shots.reduce((a, b) => (b.rpm > a.rpm ? b : a), shots[0] || null);
    return {
      loDeg: w.rec ? g.angles[w.rec.lo] : NaN, hiDeg: w.rec ? g.angles[w.rec.hi] : NaN,
      covered: shots.length, cells: g.cells.length, shots, worst,
      exitSpeedMps: worst ? worst.rpm * kpr : NaN,
    };
  }

  // Second-wheel gearing. Both motors still get one setVelocity; a G2:1 stage between motor 2 and the
  // small wheel lifts its surface speed, which raises the no-slip ceiling and changes the ball spin.
  // derive() holds the measured slip fraction, so kpr rises with the ceiling. Each row runs every exit speed in vts; best is the ratio that passes the most cases, then the lowest worst-case
  // mass. The mass sweep is equivalent mass at the main radius, so a geared small wheel's inertia counts
  // G2^2 times toward it; the existing-wheel floor still treats the 120 g as one disc at R.
  function gearSweep(inputs, vts, lo, hi, step) {
    lo = lo || 1; hi = hi || 3; step = step || 0.05;
    const rows = [];
    for (let G2 = lo; G2 <= hi + 1e-9; G2 += step) {
      G2 = Math.round(G2 * 1000) / 1000;
      const inp = Object.assign({}, inputs, { wheel2Gear: G2 });
      const cases = vts.map(vt => recommend(Object.assign({}, inp, { exitSpeedMps: vt })));
      const p = cases[0].p;
      const passing = cases.filter(c => Number.isFinite(c.robustKg));
      rows.push({ G2, kpr: p.kpr, lam: p.lam, ceil: p.ceil, cases, passes: passing.length,
        worstKg: passing.length ? Math.max.apply(null, passing.map(c => Math.max(c.robustKg, c.p.mEx))) : NaN,
        wheel2Rpm: Math.max.apply(null, passing.map(c => c.rpmT * G2)) });
    }
    const best = rows.reduce((a, b) => (b.passes > a.passes || (b.passes === a.passes && b.worstKg < a.worstKg) ? b : a));
    return { rows, best, slip: rows[0].cases[0].p.slip };
  }

  const g = kg => (kg * 1000).toFixed(0) + ' g';
  function report(name, r) {
    let s = name + ': ' + r.rpmT.toFixed(0) + ' rpm (' + r.freePct.toFixed(0) + '% of ' + r.freeRpm.toFixed(0) + ' free)'
      + (r.p.G2 !== 1 ? ', second wheel ' + (r.rpmT * r.p.G2).toFixed(0) + ' rpm' : '') + '\n';
    if (r.status === 'unreachable') return s + (r.reason === 'wheel2' ? '  second wheel over its ' + r.p.w2Max + ' rpm limit\n' : '  motor can\'t reach that speed\n');
    if (r.status === 'none') return s + '  no mass under 2 kg holds +-' + r.p.tolRpm + ' rpm at ' + r.p.dt + ' s spacing\n';
    s += '  passes from ' + g(r.passKg) + ', spin-up ' + r.spin.toFixed(2) + ' s, worst error ' + r.err.toFixed(1) + '% (allowed ' + r.tol.toFixed(2) + '%)\n';
    if (r.ranges.length > 1) s += '  passing ranges: ' + r.ranges.map(x => g(x.loKg) + '-' + g(x.hiKg)).join(', ') + '\n';
    if (r.fragile) s += '  knife edge: recovery time meets the shot interval here, ' + (Number.isFinite(r.robustKg) ? 'the first solid pass is ' + g(r.robustKg) : 'no pass is wider than 30 g') + '\n';
    if (r.status === 'existing') return s + '  existing ' + g(r.p.mEx) + ' wheels already pass; add nothing\n';
    s += '  build ' + g(r.buildKg) + ': inertia ' + (r.add.Ireq * 1e6).toFixed(0) + ' kg mm2, existing ' + (r.add.Iex * 1e6).toFixed(0)
      + ', add ' + g(r.add.atRadius) + ' at ' + (r.p.rAdd * 1000).toFixed(0) + ' mm or ' + g(r.add.atRim) + ' at ' + (r.add.rimR * 1000).toFixed(1) + ' mm, spread evenly and balanced\n';
    if (r.slowSpin) s += '  spin-up exceeds ' + r.p.maxSpin + ' s: ' + (r.p.ctrl === 'ff' ? 'raise the velocity P gain' : 'gear up, space the shots or lower the target') + '\n';
    return s;
  }

  function gearReport(gs, names) {
    const f = (x, d) => Number.isFinite(x) ? x.toFixed(d) : '-';
    let s = 'second-wheel gearing, slip fraction ' + gs.slip.toFixed(2) + ' held; columns are passing mass per case, u = motor cannot reach, w = second wheel over its rpm limit, x = no mass passes, ~ = only a knife-edge pass\n';
    s += ' G2    kpr     spin  w2 max ' + names.map(n => n.padStart(11)).join('') + '\n';
    for (const r of gs.rows) {
      s += (r === gs.best ? '*' : ' ') + f(r.G2, 2) + ' ' + f(r.kpr, 5) + ' ' + f(r.lam, 3).padStart(5) + ' ' + f(r.wheel2Rpm, 0).padStart(6) + ' '
        + r.cases.map(c => (c.status === 'unreachable' ? (c.reason === 'wheel2' ? 'w' : 'u') : c.status === 'none' ? 'x' : Number.isFinite(c.robustKg) ? g(Math.max(c.robustKg, c.p.mEx)) : '~' + g(c.passKg)).padStart(11)).join('') + '\n';
    }
    const b = gs.best;
    return s + 'best ' + f(b.G2, 2) + ':1 passes ' + b.passes + '/' + b.cases.length + ' cases, heaviest ' + g(b.worstKg) + ', ball spin factor ' + f(b.lam, 3) + ' (0 is no spin; ' + (b.G2 * b.cases[0].p.D2 > b.cases[0].p.D ? 'small wheel faster' : 'main wheel faster') + ')\n';
  }

  return { DEFAULTS, MOTORS, CASES, map, derive, shoot, motor, simulate, sweep, addedWeight, recommend, hoodCase, gearSweep, gearReport, report };
});

if (typeof require !== 'undefined' && require.main === module) {
  const fs = require('fs'), path = require('path');
  const Fly = module.exports;
  const inputs = Object.assign({}, Fly.DEFAULTS, JSON.parse(fs.readFileSync(path.join(__dirname, 'inputs.json'), 'utf8')));
  const hood = process.argv.includes('hood');
  if (hood) process.argv.push('--hood');
  const fails = require('./test.js');
  if (fails.length) { console.error('Reference checks failed:\n  ' + fails.join('\n  ')); process.exit(1); }
  const p = Fly.derive(Fly.map(inputs));
  let out = 'lambda ' + p.lam.toFixed(3) + ', wheel-to-ball ratio ' + p.s.toFixed(3) + ', no-slip ceiling ' + p.ceil.toFixed(5) + ' m/s per rpm'
    + (p.kpr > p.ceil ? ' (exitSpeedPerRpm ' + p.kpr + ' is impossible for these wheels)' : '') + '\n';
  if (hood) {
    const shotsInputs = JSON.parse(fs.readFileSync(path.join(__dirname, '../shots/inputs.json'), 'utf8'));
    const h = Fly.hoodCase(inputs.exitSpeedPerRpm, shotsInputs);
    out += 'hood sweep: window ' + h.loDeg + '-' + h.hiDeg + ' deg, ' + h.covered + '/' + h.cells + ' cells, worst ' + h.worst.rpm + ' rpm at '
      + h.worst.d.toFixed(2) + ' m, ' + h.worst.v.toFixed(1) + ' m/s -> design exit speed ' + h.exitSpeedMps.toFixed(2) + ' m/s\n';
    out += Fly.report('Hood sweep worst case', Fly.recommend(Object.assign({}, inputs, { exitSpeedMps: h.exitSpeedMps })));
  }
  if (process.argv.includes('gear')) out += Fly.gearReport(Fly.gearSweep(inputs, Fly.CASES.map(c => c.vt)), Fly.CASES.map(c => c.vt + ' m/s'));
  for (const c of Fly.CASES) out += Fly.report(c.name + ' (' + c.vt + ' m/s)', Fly.recommend(Object.assign({}, inputs, { exitSpeedMps: c.vt })));
  process.stdout.write(out);
}
