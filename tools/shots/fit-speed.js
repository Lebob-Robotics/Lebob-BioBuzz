// Fits exitSpeedPerRpm (ball speed per flywheel RPM) from slow motion video readings.
// Run: node tools/shots/fit-speed.js [measurements file, default speed-measurements.json]
//
// Each reading is one ball, filmed side on against a grid, with two frames picked soon after it
// leaves the hood:
//   { "hoodDeg": 75.0, "rpm": 3510, "fps": 240, "frames": 12, "acrossM": 0.21, "upM": 0.55 }
// hoodDeg and rpm are the "WRITE DOWN" values from the Shooter Calibration OpMode. frames is how
// many frames apart the two pictures are. acrossM is how far the ball moved away from the robot
// between them, upM how far it rose (negative if it fell).
'use strict';
const fs = require('fs');
const path = require('path');

const G = 9.81; // m/s^2
const MAX_SPREAD_BETWEEN_HOODS = 0.05; // Above this, make ball speed depend on hood angle in shots.js

// Speed and angle at the first frame. Gravity has already slowed the rise by the second frame, so
// add back half of g times the time to get the upward speed at the first frame.
function launch(reading) {
  const timeS = reading.frames / reading.fps;
  const acrossSpeedMps = reading.acrossM / timeS;
  const upSpeedMps = reading.upM / timeS + G * timeS / 2;
  return {
    speedMps: Math.hypot(acrossSpeedMps, upSpeedMps),
    angleDeg: Math.atan2(upSpeedMps, acrossSpeedMps) * 180 / Math.PI,
  };
}

// Best straight line through zero (least squares): speed = ratio times RPM.
function fitRatio(shots) {
  let sumRpmTimesSpeed = 0, sumRpmSquared = 0;
  for (const shot of shots) {
    sumRpmTimesSpeed += shot.rpm * shot.speedMps;
    sumRpmSquared += shot.rpm * shot.rpm;
  }
  return sumRpmTimesSpeed / sumRpmSquared;
}

function percent(fraction) { return (fraction * 100).toFixed(1) + '%'; }

const file = process.argv[2] || path.join(__dirname, 'speed-measurements.json');
const readings = JSON.parse(fs.readFileSync(file, 'utf8'));
if (readings.length === 0) {
  console.log('No readings in ' + file + ' yet. See the top of fit-speed.js for the format.');
  process.exit(0);
}

const shots = readings.map(reading => Object.assign({}, reading, launch(reading)));
console.log('hoodDeg   rpm   speed m/s   per RPM    measured angle');
for (const shot of shots) {
  console.log(shot.hoodDeg.toFixed(1).padStart(7) + String(Math.round(shot.rpm)).padStart(6) +
    shot.speedMps.toFixed(2).padStart(12) + (shot.speedMps / shot.rpm).toFixed(5).padStart(10) +
    shot.angleDeg.toFixed(1).padStart(13) + '  (' + (shot.angleDeg - shot.hoodDeg).toFixed(1) + ' from hood)');
}

const overallRatio = fitRatio(shots);
const byHoodDeg = new Map();
for (const shot of shots) {
  const hoodKey = Math.round(shot.hoodDeg);
  if (!byHoodDeg.has(hoodKey)) byHoodDeg.set(hoodKey, []);
  byHoodDeg.get(hoodKey).push(shot);
}

console.log('\nPer hood angle:');
let largestDifference = 0;
for (const [hoodKey, hoodShots] of [...byHoodDeg].sort((a, b) => a[0] - b[0])) {
  const ratio = fitRatio(hoodShots);
  const difference = (ratio - overallRatio) / overallRatio;
  largestDifference = Math.max(largestDifference, Math.abs(difference));
  console.log('  ' + hoodKey + ' deg: ' + ratio.toFixed(5) + ' m/s per RPM from ' + hoodShots.length +
    ' shots (' + (difference >= 0 ? '+' : '') + percent(difference) + ' from overall)');
}

console.log('\nexitSpeedPerRpm = ' + overallRatio.toFixed(5));
console.log('Put it in tools/shots/inputs.json, then run node tools/shots/export.js.');
if (largestDifference > MAX_SPREAD_BETWEEN_HOODS) {
  console.log('WARNING: hood angles differ by up to ' + percent(largestDifference) + ', more than ' +
    percent(MAX_SPREAD_BETWEEN_HOODS) + '. One number will not do: make ball speed depend on hood angle in shots.js.');
}
