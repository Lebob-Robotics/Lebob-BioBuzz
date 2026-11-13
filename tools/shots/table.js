// Shot table for the hood: one hood angle and one flywheel RPM for every distance and radial speed.
// Uses the hood sweep in hood.js (which uses the physics in shots.js) and writes
// TeamCode/.../utils/ShotTable.java. Loaded by export.js under node.
(function (root, factory) {
  if (typeof module === 'object' && module.exports) module.exports = factory(require('./shots.js'), require('./hood.js'));
  else root.Table = factory(root.Shots, root.Hood);
})(typeof self !== 'undefined' ? self : this, function (Shots, Hood) {
  'use strict';

  const MARGIN_LOST_PER_DEG_FROM_NEIGHBOURS = 0.05; // In tolerances. Stops the hood twitching between neighbouring cells

  // The shot with the biggest margin, less a penalty for differing from the neighbours' hood angles.
  // Null when nothing scores with the tolerances.
  function pickShot(cell, neighbourHoodAngles) {
    let best = null;
    cell.byAngle.forEach((bestAtAngle, angleIndex) => {
      if (bestAtAngle.s <= 0) return; // s is the margin; zero means no robust shot at this angle
      const hoodDeg = cell.angles[angleIndex];
      let score = bestAtAngle.s;
      for (const neighbourDeg of neighbourHoodAngles) score -= MARGIN_LOST_PER_DEG_FROM_NEIGHBOURS * Math.abs(hoodDeg - neighbourDeg);
      if (!best || score > best.score) best = { hoodDeg, rpm: cell.rpms[bestAtAngle.ir], margin: bestAtAngle.s, score };
    });
    return best;
  }

  // Walks the grid by distance, then speed, so each cell can see the cell before it in both directions.
  function build(p, onProgress) {
    const sweep = Hood.sweep(p, onProgress);
    const speedCount = sweep.vel.length;
    const emptyGrid = () => sweep.dist.map(() => sweep.vel.map(() => NaN));
    const table = { dist: sweep.dist, vel: sweep.vel, hood: emptyGrid(), rpm: emptyGrid(), tof: emptyGrid(), margin: emptyGrid() };
    for (let dist = 0; dist < sweep.dist.length; dist++) {
      for (let speed = 0; speed < speedCount; speed++) {
        const neighbourHoodAngles = [];
        if (dist > 0 && !Number.isNaN(table.hood[dist - 1][speed])) neighbourHoodAngles.push(table.hood[dist - 1][speed]);
        if (speed > 0 && !Number.isNaN(table.hood[dist][speed - 1])) neighbourHoodAngles.push(table.hood[dist][speed - 1]);
        const shot = pickShot(sweep.cells[dist * speedCount + speed], neighbourHoodAngles);
        if (!shot) continue;
        const flight = Shots.simulate(shot.rpm, sweep.vel[speed], sweep.dist[dist], Object.assign({}, p, { launchAngleDeg: shot.hoodDeg }));
        table.hood[dist][speed] = shot.hoodDeg;
        table.rpm[dist][speed] = shot.rpm;
        table.tof[dist][speed] = flight.tof;
        table.margin[dist][speed] = shot.margin;
      }
    }
    return table;
  }

  // The robot blends the four cells around it. Checks the blend's worst points (each block's centre and edge
  // midpoints) still score within the hood and RPM tolerances, and finds the widest hood spread in a block.
  function blendCheck(table, p) {
    const scores = (rpm, vel, dist, hood) =>
      Shots.simulate(rpm, vel, dist, Object.assign({}, p, { launchAngleDeg: hood })).result === 'score';
    let maxSpreadDeg = 0;
    const misses = [];
    for (let i = 0; i + 1 < table.dist.length; i++) {
      for (let j = 0; j + 1 < table.vel.length; j++) {
        const hs = [table.hood[i][j], table.hood[i][j + 1], table.hood[i + 1][j], table.hood[i + 1][j + 1]];
        const rs = [table.rpm[i][j], table.rpm[i][j + 1], table.rpm[i + 1][j], table.rpm[i + 1][j + 1]];
        if (hs.some(Number.isNaN)) continue; // The robot reports no shot next to an empty cell
        maxSpreadDeg = Math.max(maxSpreadDeg, Math.max(...hs) - Math.min(...hs));
        for (const [fu, fv] of [[0.5, 0.5], [0.5, 0], [0.5, 1], [0, 0.5], [1, 0.5]]) {
          const mix = v => (v[0] * (1 - fv) + v[1] * fv) * (1 - fu) + (v[2] * (1 - fv) + v[3] * fv) * fu;
          const hood = mix(hs), rpm = mix(rs);
          const dist = table.dist[i] + fu * (table.dist[i + 1] - table.dist[i]);
          const vel = table.vel[j] + fv * (table.vel[j + 1] - table.vel[j]);
          let ok = true;
          for (const dh of [-p.hoodTolDeg, 0, p.hoodTolDeg]) {
            for (const dr of [-p.rpmToleranceRpm, 0, p.rpmToleranceRpm]) ok = ok && scores(rpm + dr, vel, dist, hood + dh);
          }
          if (!ok) { misses.push('dist ' + dist.toFixed(2) + ' vel ' + vel.toFixed(2)); break; }
        }
      }
    }
    return { maxSpreadDeg, misses };
  }

  function javaNumber(v) { return Number.isNaN(v) ? 'NaN' : String(Math.round(v * 1000) / 1000); }
  function javaRow(values) { return '{' + values.map(javaNumber).join(', ') + '}'; }
  function javaGrid(rows, comment) {
    return '{' + (comment ? ' // ' + comment : '') + '\n      ' + rows.map(javaRow).join(',\n      ') + '\n  }';
  }

  function toJava(table, p) {
    const cellsWithShot = table.rpm.flat().filter(v => !Number.isNaN(v)).length;
    return 'package org.firstinspires.ftc.teamcode.utils;\n\n' +
      '/**\n' +
      ' * GENERATED by tools/shots/export.js. Do not edit by hand: change tools/shots/inputs.json and run\n' +
      ' * "node tools/shots/export.js".\n' +
      ' *\n' +
      ' * <p>Rows are the distance from the shooter to the Cell\'s near lip (metres). Columns are the robot\'s\n' +
      ' * speed toward the Cell (metres per second, positive when closing). NaN means no safe shot.\n' +
      ' * ' + cellsWithShot + ' of ' + table.dist.length * table.vel.length + ' cells have a shot.\n' +
      ' *\n' +
      ' * <p>Inputs: ' + JSON.stringify(p) + '\n' +
      ' */\n' +
      'public final class ShotTable {\n' +
      '  private ShotTable() {}\n\n' +
      '  private static final double NaN = Double.NaN;\n\n' +
      '  public static final double EXIT_SPEED_PER_RPM = ' + p.exitSpeedPerRpm + '; // Ball speed out of the shooter, m/s per flywheel RPM\n\n' +
      '  public static final double[] DIST_M = ' + javaRow(table.dist) + ';\n' +
      '  public static final double[] VEL_MPS = ' + javaRow(table.vel) + ';\n\n' +
      '  public static final double[][] HOOD_DEG = ' + javaGrid(table.hood) + ';\n\n' +
      '  public static final double[][] RPM = ' + javaGrid(table.rpm) + ';\n\n' +
      '  public static final double[][] TOF_S = ' + javaGrid(table.tof, 'Time of flight') + ';\n\n' +
      '  public static final double[][] MARGIN = ' + javaGrid(table.margin, 'Distance to the nearest miss, in tolerances. Bigger is safer') + ';\n' +
      '}\n';
  }

  // Wheel speeds above each cell's RPM that the volley table covers. A volley spins the wheels this much faster than
  // the table, and the hood follows the speed they are actually at: too slow has no hood that scores, too fast does
  const RPM_ABOVE = [0, 100, 200, 300, 400, 500, 600];
  const OFFSET_STEP_DEG = 0.2;

  /** Middle of the hood angles that score at rpm, searched up from just under fromDeg; NaN when none does. */
  function windowCentre(rpm, vel, dist, fromDeg, p) {
    let lo = NaN, hi = NaN;
    for (let h = Math.max(p.hoodMinDeg, fromDeg - 4); h <= p.hoodMaxDeg; h += OFFSET_STEP_DEG) {
      const r = Shots.simulate(rpm, vel, dist, Object.assign({}, p, { launchAngleDeg: h })).result;
      if (r === 'score') { if (Number.isNaN(lo)) lo = h; hi = h; } else if (!Number.isNaN(lo)) break;
    }
    return Number.isNaN(lo) ? NaN : (lo + hi) / 2;
  }

  /**
   * Hood degrees to add to each cell's angle when the wheels run RPM_ABOVE[k] faster than its RPM:
   * offsets[k][dist][vel]. The window's middle, never falling as the speed rises, 0 at the cell's own speed.
   */
  function rpmOffsets(table, p) {
    const offsets = RPM_ABOVE.map(() => table.dist.map(() => table.vel.map(() => NaN)));
    for (let i = 0; i < table.dist.length; i++) {
      for (let j = 0; j < table.vel.length; j++) {
        const h0 = table.hood[i][j], r0 = table.rpm[i][j];
        if (Number.isNaN(h0)) continue;
        let last = 0, from = h0;
        RPM_ABOVE.forEach((above, k) => {
          const c = above === 0 ? h0 : windowCentre(r0 + above, table.vel[j], table.dist[i], from, p);
          last = Number.isNaN(c) ? last : Math.max(last, c - h0);
          from = h0 + last;
          offsets[k][i][j] = Math.round(last * 10) / 10;
        });
      }
    }
    return offsets;
  }

  function toJavaRpm(table, offsets, p) {
    return 'package org.firstinspires.ftc.teamcode.utils;\n\n' +
      '/**\n' +
      ' * GENERATED by tools/shots/export.js with ShotTable. Do not edit by hand.\n' +
      ' *\n' +
      ' * <p>Hood degrees to add to {@link ShotTable#HOOD_DEG} when the wheels run faster than {@link ShotTable#RPM}:\n' +
      ' * OFFSET_DEG[k][dist][vel] for RPM_ABOVE[k]. The middle of the hood angles that score at that speed, by the\n' +
      ' * same flight model. Below the table\'s speed nothing scores at any angle, so the volley never fires there.\n' +
      ' */\n' +
      'public final class ShotTableRpm {\n' +
      '  private ShotTableRpm() {}\n\n' +
      '  private static final double NaN = Double.NaN;\n\n' +
      '  public static final double[] RPM_ABOVE = ' + javaRow(RPM_ABOVE) + ';\n\n' +
      '  public static final double[][][] OFFSET_DEG = {\n' +
      offsets.map((g, k) => '    // +' + RPM_ABOVE[k] + ' RPM\n    ' + javaGrid(g).replace(/\n/g, '\n  ')).join(',\n') + '\n  };\n' +
      '}\n';
  }

  return { MARGIN_LOST_PER_DEG_FROM_NEIGHBOURS, RPM_ABOVE, pickShot, build, blendCheck, toJava, rpmOffsets, toJavaRpm };
});
