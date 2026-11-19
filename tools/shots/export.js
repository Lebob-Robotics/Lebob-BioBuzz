#!/usr/bin/env node
// Regenerates TeamCode/.../utils/ShotTable.java (hood angle and RPM table) from tools/shots/inputs.json.
// Run from any directory:
//   node tools/shots/export.js
const fs = require('fs');
const path = require('path');
const Shots = require('./shots.js');
const Hood = require('./hood.js');
const Table = require('./table.js');

const inputs = JSON.parse(fs.readFileSync(path.join(__dirname, 'inputs.json'), 'utf8'));
const p = Object.assign({}, Hood.DEFAULTS, inputs);

const fails = Shots.selfCheck(p).concat(Hood.selfCheck(p));
if (fails.length) {
  console.error('Self-check failed, not exporting:\n  ' + fails.join('\n  '));
  process.exit(1);
}

const out = path.join(__dirname, '..', '..', 'TeamCode', 'src', 'main', 'java', 'org', 'firstinspires', 'ftc', 'teamcode', 'utils', 'ShotTable.java');
const table = Table.build(p);
const blends = Table.blendCheck(table, p);
if (blends.misses.length) {
  console.error('Blending neighbouring cells misses, not exporting:\n  ' + blends.misses.join('\n  '));
  process.exit(1);
}
fs.writeFileSync(out, Table.toJava(table, p));
const offsets = Table.rpmOffsets(table, p);
fs.writeFileSync(out.replace('ShotTable.java', 'ShotTableRpm.java'), Table.toJavaRpm(table, offsets, p));
const usable = table.rpm.flat().filter(v => !Number.isNaN(v)).length;
console.log('wrote ' + path.relative(process.cwd(), out) + ': ' + table.dist.length + ' x ' + table.vel.length + ' cells, ' + usable + ' with a shot');
console.log('every blend scores; widest hood spread in a block ' + blends.maxSpreadDeg.toFixed(1)
    + ' deg (keep ShootOnTheMoveConstants.MAX_HOOD_DIFFERENCE_TO_BLEND_DEG above it)');
