#!/usr/bin/env node
// Mechanical design pass for the mk2 shooter: chains the three sims so every CAD constant comes out
// of one run. For each second-wheel ratio: hood sweep (shots/hood.js) -> hood window and worst shot ->
// flywheel sizing (flywheel/flywheel.js) across main-motor gearing. No physics here; it only wires
// the existing tools together with the mk2 geometry. `node tools/design.js` (a few minutes).
'use strict';
const fs = require('fs'), path = require('path');
const Fly = require('./flywheel/flywheel.js');
const read = f => JSON.parse(fs.readFileSync(path.join(__dirname, f), 'utf8'));

// mk2 geometry. Everything else comes from shots/inputs.json and flywheel/inputs.json.
const MK2 = {
  wheelDiameterM: 0.0762,   // 3 in main wheel
  wheel2DiameterM: 0.0254,  // 1 in second wheel
  exitHeightM: 0.22,        // ball centre at exit, above the tiles
  slip: 0.853,              // exit speed over the no-slip ceiling; same fraction the mk1 estimate used, measure it
  existingWheelMassKg: 0,   // report the inertia the CAD needs, not what to bolt onto old wheels
};
const G2S = [1, 1.5, 2, 2.5, 3];                         // second wheel rpm over main wheel rpm; 3 matches surface speeds
const GS = [0.5, 0.6, 0.7, 0.8, 0.9, 1, 1.2, 1.4];      // main wheel rpm over motor rpm (belt stage)
const MAX_FREE_PCT = 80;                                // worst shot's share of free speed: headroom for battery sag
const SPEED = { minMps: 3, maxMps: 13 };                // exit speeds the hood sweep may use
// Hood can travel 0-100 deg. Drive: 6000 rpm bare 5203 / 12 to 104 mm wheels = 2.72 m/s free, about 2.4 loaded.
const HOOD = { hoodMinDeg: 0, hoodMaxDeg: 100, velMinMps: -2.4, velMaxMps: 2.4, velStepMps: 0.2 };

const shotsIn = Object.assign(read('shots/inputs.json'), { exitHeightM: MK2.exitHeightM });
const flyIn = Object.assign(read('flywheel/inputs.json'), {
  wheelDiameterM: MK2.wheelDiameterM, wheel2DiameterM: MK2.wheel2DiameterM, existingWheelMassKg: MK2.existingWheelMassKg,
  exitSpeedPerRpm: MK2.slip * Math.PI * (MK2.wheelDiameterM + MK2.wheel2DiameterM) / 2 / 60,  // as if measured at 1:1
});

const rows = [];
for (const G2 of G2S) {
  const kpr = Fly.derive(Fly.map(Object.assign({}, flyIn, { wheel2Gear: G2 }))).kpr;
  const step = 50;
  const h = Fly.hoodCase(kpr, shotsIn, Object.assign({}, HOOD, {
    rpmMin: Math.floor(SPEED.minMps / kpr / step) * step, rpmMax: Math.ceil(SPEED.maxMps / kpr / step) * step, rpmStep: step,
  }));
  // Main-motor gearing: lightest wheel that holds the worst shot, spin-up inside the limit.
  const fly = GS.map(G => ({ G, r: Fly.recommend(Object.assign({}, flyIn, { wheel2Gear: G2, gearRatio: G, exitSpeedMps: h.exitSpeedMps })) }))
    .filter(x => Number.isFinite(x.r.robustKg) && !x.r.slowSpin && x.r.freePct <= MAX_FREE_PCT && x.r.rpmT * G2 <= flyIn.wheel2MaxRpm);
  const best = fly.reduce((a, b) => (!a || b.r.robustKg < a.r.robustKg ? b : a), null);
  rows.push({ G2, kpr, h, best });
  process.stderr.write('G2 ' + G2 + ' done\n');
}

const f = (x, d) => Number.isFinite(x) ? x.toFixed(d) : '-';
let out = 'mk2: ' + MK2.wheelDiameterM * 1000 + ' mm + ' + MK2.wheel2DiameterM * 1000 + ' mm wheels, exit ' + MK2.exitHeightM + ' m, slip ' + MK2.slip
  + ', tolerance +-' + flyIn.toleranceRpm + ' rpm at the wheel, worst shot <= ' + MAX_FREE_PCT + '% of free speed\n';
out += ' G2   m/s/rpm  hood      cells    worst shot              exit m/s  motor G  wheel rpm  w2 rpm  inertia kgmm2  spin-up\n';
for (const r of rows) {
  const b = r.best, w = r.h.worst;
  out += f(r.G2, 1).padStart(4) + '  ' + f(r.kpr, 5) + '  ' + (f(r.h.loDeg, 0) + '-' + f(r.h.hiDeg, 0)).padEnd(8) + '  ' + (r.h.covered + '/' + r.h.cells).padEnd(7)
    + '  ' + (w ? f(w.d, 2) + ' m @ ' + f(w.v, 1) + ' m/s, ' + w.angle + ' deg' : '-').padEnd(22) + '  ' + f(r.h.exitSpeedMps, 2).padStart(8)
    + '  ' + (b ? f(b.G, 1).padStart(7) + '  ' + f(b.r.rpmT, 0).padStart(9) + '  ' + f(b.r.rpmT * r.G2, 0).padStart(6)
      + '  ' + f(1.15 * b.r.p.k * b.r.robustKg * (MK2.wheelDiameterM / 2) ** 2 * 1e6, 0).padStart(13) + '  ' + f(b.r.spin, 2) + ' s' : 'no gearing passes') + '\n';
}
// Pick: most cells covered, then narrowest hood travel, then least inertia.
const ok = rows.filter(r => r.best);
const pick = ok.reduce((a, b) => {
  if (!a) return b;
  if (b.h.covered !== a.h.covered) return b.h.covered > a.h.covered ? b : a;
  const wa = a.h.hiDeg - a.h.loDeg, wb = b.h.hiDeg - b.h.loDeg;
  if (wb !== wa) return wb < wa ? b : a;
  return b.best.r.robustKg < a.best.r.robustKg ? b : a;
}, null);
if (pick) {
  const b = pick.best.r, R = MK2.wheelDiameterM / 2, I = 1.15 * b.p.k * b.robustKg * R * R;
  out += '\nRecommended constants for CAD\n'
    + '  second wheel ratio      ' + pick.G2 + ':1 (1 in wheel rpm over 3 in wheel rpm)\n'
    + '  main motor belt ratio   ' + pick.best.G + ':1 (wheel over motor)\n'
    + '  hood travel             ' + pick.h.loDeg + ' to ' + pick.h.hiDeg + ' deg launch angle\n'
    + '  flywheel inertia        ' + (I * 1e6).toFixed(0) + ' kg mm2 at the main shaft, all rotating parts (e.g. '
    + (I / (0.5 * R * R) * 1000).toFixed(0) + ' g solid 3 in disc, or ' + (I / (0.9 * R) ** 2 * 1000).toFixed(0) + ' g at ' + (0.9 * R * 1000).toFixed(1) + ' mm radius)\n'
    + '  design exit speed       ' + pick.h.exitSpeedMps.toFixed(2) + ' m/s, ' + b.rpmT.toFixed(0) + ' wheel rpm (' + b.freePct.toFixed(0) + '% of free), spin-up ' + b.spin.toFixed(2) + ' s\n'
    + '  exit speed per rpm      ' + pick.kpr.toFixed(5) + ' m/s (estimate until measured; regenerate ShotTable.java after)\n';
}
process.stdout.write(out);
