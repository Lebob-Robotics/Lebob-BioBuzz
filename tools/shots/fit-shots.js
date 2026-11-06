#!/usr/bin/env node
// Fits the shot model to shots that were fired and seen: which exit speed, launch offset, backspin lift and
// bounce-out limit best explain what went in, long, short or rattled out. Prints the best parameter sets and,
// for the best and for inputs.json, each shot's predicted chance of what was seen.
//   [NO_CAL=1] node tools/shots/fit-shots.js [grid_shots.csv | judged_shots.csv ...]   (NO_CAL: grid shots only)
// Shots come from front-calibration.json (fired/scored counts at a set distance, hood and RPM) and from the
// grid CSVs that sotm-tools/gridreport.py writes, once their result column is filled in. Grid results: in,
// rattle (in after hitting the walls), out (went in and bounced out), over, short, miss (way not seen);
// left/right are sideways and carry no range information, so they are left out.
'use strict';
const fs = require('fs');
const path = require('path');
const Shots = require('./shots.js');
const Hood = require('./hood.js');

const inputs = JSON.parse(fs.readFileSync(path.join(__dirname, 'inputs.json'), 'utf8'));
const base = Object.assign({}, Hood.DEFAULTS, inputs);

// Each simulated shot is spread over the errors the robot can't see: hood, wheel speed and distance
const HOOD_ERR_DEG = [-1, -0.5, 0, 0.5, 1];
const RPM_ERR = [-50, 0, 50];
const DIST_ERR_M = [-0.03, 0, 0.03];
const PRIOR = 0.5; // Added to each outcome's count, so one unlikely shot doesn't end a parameter set

// The side webcam's OUT is any ball that touched the Cell and ended outside. Where it left gives the way: off the
// top bar (picture y up to 200) is long, off the opening's lower lip (x up to 960) short, and anything else went
// in and bounced out. A verdict checked by eye says long or short itself.
function judgedResult(row) {
  const v = row.verdict.toLowerCase();
  const d = (row.detail || '').toLowerCase().replace(/^checked: /, '');
  if (v === 'miss') return d.startsWith('long') ? 'long' : d.startsWith('short') ? 'short' : 'miss';
  if (v !== 'out') return v;
  if (d.startsWith('long') || d.startsWith('short')) return d.split(':')[0];
  const at = d.match(/^left at \((\d+),(\d+)\)/);
  if (!at) return 'out';
  return +at[2] <= 200 ? 'long' : +at[1] <= 960 ? 'short' : 'out';
}

function loadShots(csvPaths) {
  const shots = [];
  const cal = JSON.parse(fs.readFileSync(path.join(__dirname, 'front-calibration.json'), 'utf8'));
  for (const pt of process.env.NO_CAL ? [] : cal.points) {
    const hood = 99 - pt.hoodLiftDeg;
    if (pt.scored > 0) shots.push({ src: 'cal', distM: pt.distM, hood, rpm: pt.rpm, seen: 'score', n: pt.scored });
    // The calibration notes didn't say which way the misses went
    if (pt.fired > pt.scored) shots.push({ src: 'cal', distM: pt.distM, hood, rpm: pt.rpm, seen: 'miss', n: pt.fired - pt.scored });
  }
  for (const file of csvPaths) {
    const lines = fs.readFileSync(file, 'utf8').trim().split(/\r?\n/); // Python's csv writes \r\n
    const head = lines[0].split(',');
    for (const line of lines.slice(1)) {
      const cells = line.match(/("([^"]|"")*"|[^,]*)(,|$)/g).map((c) => c.replace(/,$/, '').replace(/^"|"$/g, '').replace(/""/g, '"'));
      const row = Object.fromEntries(cells.map((v, i) => [head[i], v]));
      // judged_shots.csv (trackshots.py, from the side webcam): verdict IN, OUT, or MISS with long/short in detail
      const r = row.verdict !== undefined ? judgedResult(row) : (row.result || '').trim().toLowerCase();
      const seen = r.startsWith('in') || r.startsWith('rattle') ? 'score' : r.startsWith('out') ? 'out'
        : r.startsWith('over') || r.startsWith('long') ? 'long' : r.startsWith('short') ? 'short'
        : r.startsWith('miss') ? 'miss' : null; // A miss nobody saw the way of
      if (!seen) continue;
      shots.push({ src: row.log.slice(-20) + '#' + (row.ball || row.launch), distM: +row.lip_m, hood: +row.hood,
        rpm: +row.rpm, seen, n: 1 });
    }
  }
  return shots;
}

// Chance of each outcome for one shot under parameters p
function outcomes(shot, p) {
  const count = { score: 0, long: 0, short: 0, out: 0 };
  let total = 0;
  for (const dh of HOOD_ERR_DEG) {
    for (const dr of RPM_ERR) {
      const q = Object.assign({}, p, { launchAngleDeg: shot.hood + dh });
      const pts = Shots.fly(shot.rpm + dr, 0, q, Shots.opening(shot.distM + 0.1, q).x1 + 0.05);
      for (const dd of DIST_ERR_M) {
        const res = Shots.land(pts, shot.distM + dd, q).result;
        // 'far' is a ball entering past the bounce-out limit, or onto the far edge: out, or long if no limit is set
        const kind = res === 'score' ? 'score' : res === 'far' ? (p.maxEntryFraction === undefined ? 'long' : 'out')
          : res === 'long' ? 'long' : 'short'; // 'lip' is short
        count[kind]++;
        total++;
      }
    }
  }
  const prob = {};
  for (const k of Object.keys(count)) prob[k] = (count[k] + PRIOR) / (total + 4 * PRIOR);
  prob.miss = 1 - prob.score;
  return prob;
}

function logLikelihood(shots, p) {
  let ll = 0;
  for (const s of shots) ll += s.n * Math.log(outcomes(s, p)[s.seen]);
  return ll;
}

function grid(values) { return values; }

const shots = loadShots(process.argv.slice(2));
console.log(shots.length + ' shot groups, ' + shots.reduce((a, s) => a + s.n, 0) + ' balls');
const candidates = [];
const speed0 = base.exitSpeedPerRpm;
for (const speedScale of grid([0.97, 0.98, 0.99, 1.0, 1.01, 1.02, 1.03, 1.04, 1.05, 1.06, 1.08, 1.10])) {
  for (const launchOffsetDeg of grid([5.5, 6.5, 7.5, 8.5, 9.5, 10.5, 11.5])) {
    for (const liftCl of grid([0, 0.05, 0.1, 0.15, 0.2])) {
      for (const maxEntryFraction of [undefined, 0.6, 0.75, 0.9]) {
        const p = Object.assign({}, base, { exitSpeedPerRpm: speed0 * speedScale, launchOffsetDeg, liftCl, maxEntryFraction });
        candidates.push({ speedScale, launchOffsetDeg, liftCl, maxEntryFraction, ll: logLikelihood(shots, p), p });
      }
    }
  }
}
candidates.sort((a, b) => b.ll - a.ll);
const current = logLikelihood(shots, base);
console.log('inputs.json: log likelihood ' + current.toFixed(2));
console.log('best 8 (exit speed x, launch offset deg, lift Cl, bounce-out limit, log likelihood):');
for (const c of candidates.slice(0, 8)) {
  console.log('  ' + c.speedScale.toFixed(2) + '  ' + c.launchOffsetDeg.toFixed(1) + '  ' + c.liftCl.toFixed(2) + '  '
    + (c.maxEntryFraction === undefined ? '-   ' : c.maxEntryFraction.toFixed(2)) + '  ' + c.ll.toFixed(2));
}
const best = candidates[0];
console.log('\nshot              dist  hood   rpm  seen    P(seen) inputs  best');
for (const s of shots) {
  console.log('  ' + s.src.padEnd(16) + s.distM.toFixed(2).padStart(5) + s.hood.toFixed(1).padStart(6) + String(s.rpm).padStart(6)
    + '  ' + (s.seen + ' x' + s.n).padEnd(9) + outcomes(s, base)[s.seen].toFixed(2).padStart(10) + outcomes(s, best.p)[s.seen].toFixed(2).padStart(6));
}
