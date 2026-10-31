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
fs.writeFileSync(out, Table.toJava(table, p));
const usable = table.rpm.flat().filter(v => !Number.isNaN(v)).length;
console.log('wrote ' + path.relative(process.cwd(), out) + ': ' + table.dist.length + ' x ' + table.vel.length + ' cells, ' + usable + ' with a shot');
