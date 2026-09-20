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
