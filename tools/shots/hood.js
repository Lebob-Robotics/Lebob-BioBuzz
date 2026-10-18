#!/usr/bin/env node
// Hood travel sizing: how far the hood must move so every shot in the envelope has a robust
// (angle, RPM) point. Reuses the ballistics in shots.js unchanged. Loaded by hood.html in a browser;
// `node tools/shots/hood.js` prints the recommendation.
(function (root, factory) {
  if (typeof module === 'object' && module.exports) module.exports = factory(require('./shots.js'));
  else root.Hood = factory(root.Shots);
})(typeof self !== 'undefined' ? self : this, function (Shots) {
  'use strict';

  const DEFAULTS = Object.assign({}, Shots.DEFAULTS, {
    hoodMinDeg: 30, hoodMaxDeg: 85, hoodStepDeg: 1,
    hoodTolDeg: 1,               // hood angle error the chosen shot must survive
    velMinMps: -1, velMaxMps: 1, velStepMps: 0.2,
    rpmStep: 50,                 // coarser than the RPM table: 56 angles x 81 speeds per cell
  });

  // Margin of a scoring (angle, RPM) point: distance, in tolerance units, to the nearest miss or
  // grid edge. `robust` needs every point within one hood tolerance by one RPM tolerance to score
  // (a rectangle, 4414's inscribed region), `margin` is the Euclidean distance so ties break toward
  // the middle of the valid region. Capped at RADIUS tolerances.
  const RADIUS = 3;
  function grade(ok, ia, ir, p) {
    const sa = p.hoodStepDeg / p.hoodTolDeg, sr = p.rpmStep / p.rpmToleranceRpm;  // steps -> tolerances
    const ka = Math.ceil(RADIUS / sa), kr = Math.ceil(RADIUS / sr);
    let cheb = RADIUS, euc = RADIUS;
    for (let da = -ka; da <= ka; da++) for (let dr = -kr; dr <= kr; dr++) {
      const a = ia + da, r = ir + dr;
      const miss = a < 0 || a >= ok.length || r < 0 || r >= ok[0].length || !ok[a][r];
      if (!miss) continue;
      const ta = da ? Math.abs(da) * sa : 0, tr = dr ? Math.abs(dr) * sr : 0;
      cheb = Math.min(cheb, Math.max(ta, tr)); euc = Math.min(euc, Math.hypot(ta, tr));
    }
    return { robust: cheb > 1, margin: euc };
  }

  // Every flight in the sweep at one radial velocity, indexed [angle][rpm]. Distance-independent.
  function flights(velMps, p) {
    const angles = Shots.axis(p.hoodMinDeg, p.hoodMaxDeg, p.hoodStepDeg), rpms = Shots.axis(p.rpmMin, p.rpmMax, p.rpmStep);
    const maxX = Shots.opening(p.distMaxM, p).x1 + 0.01;  // nothing to hit beyond the farthest Cell's far edge
    return { angles, rpms, pts: angles.map(a => rpms.map(r => Shots.fly(r, velMps, Object.assign({}, p, { launchAngleDeg: a }), maxX))) };
  }

  // One cell of the envelope: which (angle, RPM) points score, their margins, and the best RPM at
  // each angle so the window search only has to look at angles.
  function cell(distM, velMps, f, p) {
    const ok = f.pts.map(row => row.map(pts => Shots.land(pts, distM, p).result === 'score'));
    const margin = ok.map((row, ia) => row.map((v, ir) => { if (!v) return 0; const g = grade(ok, ia, ir, p); return g.robust ? g.margin : 0; }));
    const byAngle = margin.map(row => { let best = { ir: -1, s: 0 }; row.forEach((s, ir) => { if (s > best.s) best = { ir, s }; }); return best; });
    return { distM, velMps, angles: f.angles, rpms: f.rpms, ok, margin, byAngle };
  }

  function sweep(p, onProgress) {
    const dist = Shots.axis(p.distMinM, p.distMaxM, p.distStepM), vel = Shots.axis(p.velMinMps, p.velMaxMps, p.velStepMps);
    const cells = [];
    dist.forEach(d => vel.forEach(v => cells.push(null)));
    vel.forEach((v, j) => {
      const f = flights(v, p);
      dist.forEach((d, i) => { cells[i * vel.length + j] = cell(d, v, f, p); if (onProgress) onProgress(i * vel.length + j + 1, cells.length); });
    });
    return { dist, vel, cells, angles: cells[0].angles, rpms: cells[0].rpms };
  }

  // Best shot for a cell using only angles in [lo, hi] (indices), or null.
  function bestIn(c, lo, hi) {
    let best = null;
    for (let ia = lo; ia <= hi; ia++) { const b = c.byAngle[ia]; if (b.s > 0 && (!best || b.s > best.s)) best = { ia, ir: b.ir, s: b.s }; }
    return best;
  }

  // Every [lo, hi] window. The recommendation is the narrowest that covers every coverable cell,
  // ties broken by mean margin. coverage[w] is the most cells any window of width w covers.
  function windows(g) {
    const n = g.angles.length;
    const coverable = g.cells.filter(c => bestIn(c, 0, n - 1)).length;
    const coverage = new Array(n).fill(0);
    let rec = null;
    for (let lo = 0; lo < n; lo++) for (let hi = lo; hi < n; hi++) {
      let covered = 0, sum = 0;
      for (const c of g.cells) { const b = bestIn(c, lo, hi); if (b) { covered++; sum += b.s; } }
      coverage[hi - lo] = Math.max(coverage[hi - lo], covered);
      if (covered === coverable) {
        const w = hi - lo, mean = covered ? sum / covered : 0;
        if (!rec || w < rec.hi - rec.lo || (w === rec.hi - rec.lo && mean > rec.mean)) rec = { lo, hi, mean };
      }
    }
    const uncoverable = g.cells.filter(c => !bestIn(c, 0, n - 1)).map(c => ({ distM: c.distM, velMps: c.velMps }));
    return { rec, coverable, coverage, uncoverable };
  }

  // With the sweep pinned to the fixed launch angle and no hood error, the coverable stationary
  // cells must be exactly the usable cells of the existing RPM table.
  function selfCheck(p0) {
    const p = Object.assign({}, DEFAULTS, p0, { hoodMinDeg: p0.launchAngleDeg, hoodMaxDeg: p0.launchAngleDeg, hoodTolDeg: 0, velMinMps: 0, velMaxMps: 0, rpmStep: Shots.DEFAULTS.rpmStep });
    const g = sweep(p), t = Shots.solveGrid(p), fails = [];
    g.cells.forEach((c, i) => {
      const hood = !!bestIn(c, 0, 0), table = !Number.isNaN(t.rpm[i][0]);
      if (hood !== table) fails.push('at ' + c.distM + ' m the hood sweep says ' + (hood ? 'shot' : 'no shot') + ' but the RPM table says ' + (table ? 'shot' : 'no shot'));
    });
    return fails;
  }

  function report(g, w) {
    const f = a => g.angles[a].toFixed(0);
    let s = w.rec
      ? 'Recommended hood travel: ' + f(w.rec.lo) + ' to ' + f(w.rec.hi) + ' deg (' + (g.angles[w.rec.hi] - g.angles[w.rec.lo]) + ' deg), mean margin ' + w.rec.mean.toFixed(2) + ' tolerances\n'
      : 'No window covers anything\n';
    const byDist = {};
    for (const c of w.uncoverable) (byDist[c.distM] = byDist[c.distM] || []).push(c.velMps);
    const unc = Object.keys(byDist).map(d => { const v = byDist[d]; return d + ' m @ ' + (v.length === 1 ? v[0] : v[0] + '..' + v[v.length - 1]) + ' m/s'; });
    s += 'Covers ' + w.coverable + '/' + g.cells.length + ' cells; uncoverable at any swept angle: ' + (unc.length ? unc.join(', ') : 'none') + '\n';
    if (w.rec) {
      s += 'Stationary shots inside the window:\n';
      for (const c of g.cells) if (Math.abs(c.velMps) < 1e-9) { const b = bestIn(c, w.rec.lo, w.rec.hi); s += '  ' + c.distM.toFixed(3) + ' m: ' + (b ? f(b.ia) + ' deg @ ' + c.rpms[b.ir] + ' RPM, margin ' + b.s : 'none') + '\n'; }
    }
    return s;
  }

  return { DEFAULTS, grade, flights, cell, sweep, bestIn, windows, selfCheck, report };
});

if (typeof require !== 'undefined' && require.main === module) {
  const fs = require('fs'), path = require('path');
  const Hood = module.exports;
  const p = Object.assign({}, Hood.DEFAULTS, JSON.parse(fs.readFileSync(path.join(__dirname, 'inputs.json'), 'utf8')));
  const fails = Hood.selfCheck(p);
  if (fails.length) { console.error('Self-check failed:\n  ' + fails.join('\n  ')); process.exit(1); }
  const t0 = Date.now();
  const g = Hood.sweep(p);
  process.stdout.write(Hood.report(g, Hood.windows(g)) + 'swept ' + g.cells.length + ' cells in ' + ((Date.now() - t0) / 1000).toFixed(1) + ' s\n');
}
