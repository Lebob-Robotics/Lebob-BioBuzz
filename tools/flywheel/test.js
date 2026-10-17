#!/usr/bin/env node
// Reference results from the flywheel handoff (2 Oct 2026). Every row must hold before the CLI
// prints anything. Tolerances: mass +-10 g (sweep step), times +-0.02 s, errors +-0.1 points.
'use strict';
const assert = require('assert');
const Fly = require('./flywheel.js');
const inputs = require('./inputs.json');

function near(got, want, tol, what) {
  assert(Math.abs(got - want) <= tol, what + ': got ' + got + ', want ' + want + ' +-' + tol);
}
function rec(over) { return Fly.recommend(Object.assign({}, inputs, over)); }

const fails = [];
function check(name, fn) { try { fn(); } catch (e) { fails.push(name + ': ' + e.message); } }

check('derived', () => {
  const p = Fly.derive(Fly.map(inputs));
  near(p.lam, 0.081, 0.001, 'lambda');
  near(p.ceil, 0.00197, 0.00001, 'no-slip ceiling');
  near(p.s, 0.632, 0.001, 'wheel-to-ball ratio');
  near(p.rpmT, 4202, 1, 'target rpm');
  near(p.tol, 1.19, 0.01, 'tol %');
});

check('hood worst case', () => {
  const r = rec({});
  assert.strictEqual(r.status, 'build');
  near(r.passKg, 0.140, 0.01, 'pass mass');
  near(r.buildKg, 0.165, 0.005, 'build mass');
  near(r.rpmT, 4202, 1, 'rpm');
  near(r.freePct, 70, 1, '% free');
  near(r.spin, 0.16, 0.02, 'spin-up');
  near(r.err, 1.1, 0.1, 'error');
  near(r.add.Ireq * 1e6, 62, 1, 'target inertia kg mm2');
  near(r.add.Iex * 1e6, 39, 1, 'existing inertia kg mm2');
  near(r.add.atRadius * 1000, 59, 1.5, 'add g at 20 mm');
  near(r.add.atRim * 1000, 45, 1.5, 'add g at rim');
  near(r.add.rimR * 1000, 22.9, 0.1, 'rim radius mm');
});

check('faster feed', () => near(rec({ shotIntervalS: 0.08 }).passKg, 0.140, 0.01, 'pass mass'));
check('current gains', () => assert.strictEqual(rec({ controller: 'ff' }).status, 'none'));

check('hood standing', () => {
  const r = rec({ exitSpeedMps: 5.71 });
  assert.strictEqual(r.status, 'existing');
  near(r.passKg, 0.110, 0.01, 'pass mass');
  near(r.spin, 0.09, 0.02, 'spin-up at 120 g');
  near(r.err, 1.3, 0.1, 'error at 120 g');
});

check('fixed standing', () => {
  const r = rec({ exitSpeedMps: 7.734 });
  assert.strictEqual(r.status, 'build');
  near(r.passKg, 0.150, 0.01, 'pass mass');
  near(r.buildKg, 0.175, 0.005, 'build mass');
  near(r.rpmT, 4604, 1, 'rpm');
  near(r.freePct, 77, 1, '% free');
  near(r.spin, 0.21, 0.02, 'spin-up');
  near(r.err, 1.0, 0.1, 'error');
  near(r.add.Ireq * 1e6, 67, 1, 'target inertia');
  near(r.add.atRadius * 1000, 70, 1.5, 'add g at 20 mm');
  near(r.add.atRim * 1000, 54, 1.5, 'add g at rim');
});

check('fixed median', () => {
  const r = rec({ exitSpeedMps: 6.756 });
  near(r.passKg, 0.130, 0.01, 'pass mass');
  near(r.rpmT, 4021, 1, 'rpm');
  near(r.spin, 0.14, 0.02, 'spin-up');
  near(r.err, 1.2, 0.1, 'error');
});

check('fixed backing 0.7', () => {
  const r = rec({ exitSpeedMps: 10.178 });
  assert.strictEqual(r.status, 'unreachable');
  near(r.rpmT, 6058, 1, 'rpm');
  near(r.freeRpm, 6000, 1, 'free rpm');
});

check('fixed backing 0.7 geared', () => {
  const r = rec({ exitSpeedMps: 10.178, gearRatio: 1.5 });
  near(r.passKg, 0.200, 0.01, 'pass mass');
  near(r.spin, 0.47, 0.02, 'spin-up');
  near(r.err, 0.8, 0.1, 'error');
});

check('fixed backing 0.9 geared', () => assert.strictEqual(rec({ exitSpeedMps: 11.27, gearRatio: 1.5 }).status, 'none'));

check('gearing', () => {
  const gs = Fly.gearSweep(inputs, [7.06], 1, 2.1, 0.1);
  const r1 = gs.rows[0], rm = gs.rows.find(r => Math.abs(r.G2 - 2.1) < 1e-9);
  near(r1.cases[0].passKg, 0.140, 0.01, 'G2 1 reproduces the base case');
  near(r1.kpr, 0.00168, 1e-6, 'G2 1 keeps kpr');
  near(rm.ceil, 0.00267, 0.00001, 'matched wheels ceiling');
  assert(rm.lam < 0.001, 'matched wheels have no spin');
  assert(rm.cases[0].rpmT < r1.cases[0].rpmT, 'gearing up lowers the target rpm');
  const edge = rec({ wheel2Gear: 1.85, exitSpeedMps: 10.178 });
  assert(edge.fragile && Math.abs(edge.passKg - 0.23) < 1e-9, 'recovery meeting the shot interval is flagged as a knife edge');
  assert(!rec({ wheel2Gear: 1.85, exitSpeedMps: 10.178, shotIntervalS: 0.15 }).fragile, 'a longer interval makes it solid');
});

if (process.argv.includes('--hood')) check('hood sweep', () => {
  const h = Fly.hoodCase(0.00168);
  near(h.loDeg, 54, 0.5, 'window lo'); near(h.hiDeg, 84, 0.5, 'window hi');
  assert.strictEqual(h.covered, 328); assert.strictEqual(h.cells, 437);
  near(h.worst.rpm, 4200, 1, 'worst rpm'); near(h.worst.d, 1.45, 0.01, 'worst dist'); near(h.worst.v, -2.2, 0.01, 'worst vel');
  near(h.exitSpeedMps, 7.06, 0.01, 'design exit speed');
});

module.exports = fails;
if (require.main === module) {
  if (fails.length) { console.error('FAIL\n  ' + fails.join('\n  ')); process.exit(1); }
  console.log('flywheel test.js: all reference results hold');
}
