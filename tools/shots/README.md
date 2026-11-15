# Shot table tools

Lebob Robotics, FTC 29550 · BIOBUZZ 2026/27 · v1.0, 20 Sep 2026; files and
measurements updated 15 Nov 2026

Whether a shot lands depends on the distance, how fast the robot is closing
on the Cell, the hood angle and the flywheel speed. These tools work that out
on a laptop and give the robot a table, `ShotTable.java`: a hood angle and an
RPM for every distance and radial speed.

The first robot had a fixed launch angle; mark 3 (18 Oct) added the hood. The
visualiser (`index.html`) and "Why the tolerance was 50 RPM" below describe
that original fixed-angle tool.

## Files

- `shots.js`: the physics, the sweep and the Java export. The only copy.
- `index.html`: the visualiser. Open it in a browser (it fetches Plotly from
  the internet the first time). Change inputs, press Compute, move the
  sliders, export.
- `export.js`: `node tools/shots/export.js` regenerates
  `TeamCode/.../shooting/ShotTable.java` (hood angle and RPM for every distance
  and radial speed) from `inputs.json`, using `table.js`. Run it after
  changing `inputs.json` and commit both files together. It refuses to
  write a table where blending neighbouring cells would miss, and prints the
  widest hood spread between neighbours.
- `table.js`: picks one shot per cell from the hood sweep, with a small
  penalty for hood angle jumps between neighbouring cells, and writes the Java.
- `inputs.json`: the measured constants behind the committed table. With
  `axleHeightM` and `exitRadiusM` set, each ball launches from the point on
  the hood at that angle, not a fixed exit height.

- `hood.js`: sizes an adjustable hood. Sweeps hood angle and RPM at every
  distance and radial velocity in the envelope, keeps the shots that survive
  the hood and flywheel tolerances, and searches for the narrowest hood travel
  that still covers every cell. `node tools/shots/hood.js` prints it.
- `hood.html`: the same, with plots. Open it, press Compute, move the sliders.
- `fit-speed.js`: fits `exitSpeedPerRpm` from the slow-motion video readings
  in `speed-measurements.json`.
- `fit-shots.js`: fits exit speed, launch offset, backspin lift and the
  bounce-out limit to shots that were seen: `front-calibration.json` and the
  grid CSVs from `tools/analysis/gridreport.py`.
- `front-calibration.json`: hooded shots, fired and scored counts at a set
  distance, hood lift and RPM. Its `note` says how they were measured.
- `backwards-calibration.json`: the backward shot, hood on its lower stop,
  RPM only. `BackwardShotConstants` comes from it.
- `maps/volley_spots.svg` (and `.png`): Volley Tuning's spots on the red half
  of the field with each spot's IN rate, as at regionals.

## What the three panels show

1. Trajectory fan. Every flywheel speed at the chosen distance and radial
   velocity. Green arcs score, red miss, the blue one is what the robot will
   use. The black segment is the Cell opening, the dotted line its front face.
2. RPM band against radial velocity at that distance. The green fill is the
   set of speeds that score. Where it pinches shut is the closing speed the
   driver cannot exceed.
3. Heatmap of the table. Grey cells have no shot.

## Sizing the hood

`hood.html` answers one question: how far does the hood have to travel? A
shot is kept only when every angle within the hood tolerance and every speed
within the flywheel tolerance still scores, which is 4414's inscribed
tolerance region drawn as a rectangle. Each shot's margin is its distance from
the nearest miss, in tolerances, so larger is safer. The recommended window is
the narrowest one where every cell that can be covered at all has such a shot,
ties going to the higher mean margin.

- Valid region. Hood angle against RPM at the slider's cell. Dark cannot
  score, pale scores but not with the tolerances, colour is the margin. The
  star is the chosen shot, the white box its tolerance rectangle, the dashed
  lines the recommended window.
- Chosen angle against distance, one line per radial velocity. This is what
  the runtime table would hold.
- Cells covered against hood travel. Where it meets the dotted line is the
  recommended travel; the slope before it is what each extra degree buys.

- Side view. The hood's recommended wedge at the exit point, the chosen
  angle through it, the chosen arc in blue, and the four corners of its
  tolerance rectangle faintly, green where they score.
- Field map. Top-down, coloured by margin, hood angle or RPM of the shot from
  each robot position at the slider's radial velocity. The Cell's field
  position and facing are placeholders on the form until read from the
  manual. Grey means behind the Cell, outside the swept distances, or no
  robust shot. The map assumes a head-on shot at every bearing; a robot 60
  degrees off the Cell's axis is coloured the same as one dead ahead, which
  the sim does not model.

Cells listed as uncoverable have no robust shot at any swept angle. With the
placeholder inputs those are the very close shots and fast approaches at short
range; extending the sweep to 89 degrees reaches some of them.

Self-check: with the sweep pinned to the fixed launch angle and no hood error,
the coverable stationary cells must be exactly the usable cells of the RPM
table. The page shows a red banner if not; the node script refuses to run.

## Measurements

The measured values are in `inputs.json`, and the readings behind them in
`speed-measurements.json` (`fit-speed.js`) and `front-calibration.json`
(`fit-shots.js`). Change `inputs.json`, run `export.js` and commit both.

| Input | How to measure |
| --- | --- |
| `axleHeightM`, `exitRadiusM` | From the CAD: the flywheel axle's height, and how far from it the ball leaves the hood |
| `exitSpeedPerRpm` | Slow-motion video of balls past a grid at several RPMs, into `speed-measurements.json`, then `fit-speed.js` |
| `dragCd`, `liftCl` | `fit-shots.js` against shots that were seen |
| `shooterOffsetM` | Ball exit point forward of the robot's tracking point |
| `openingTiltDeg`, `openingLengthM`, `lipHeightM` | Competition Manual Figure 9-10 and the field CAD |
| `AimConstants.FEED_DELAY_S` | Video the indexer starting and the ball leaving, or count loops to the shooter current spike |

Time of flight at two distances, from the same video, checked against the
table's `TOF_S` column.

The table's distances are to the Cell's near lip, but `TargetTracker` places
the tag cluster; `CellGeometry.CLUSTER_TO_NEAR_LIP_M` moves one to the other
before the lookup.

## Why the tolerance was 50 RPM

With a fixed launch angle the near lip sets the lowest speed that scores and
the far edge of the 14 in opening sets the highest. With the current inputs
that window is about 0.3 m/s of ball speed, 150 to 225 RPM, at every distance
the table keeps. The table only
keeps cells whose band is at least twice `rpmToleranceRpm`, so the flywheel
had to hold ±50 RPM. A bigger window needed a hood, which mark 3 added. The
robot's fire gate now uses `FireConstants.RPM_TOLERANCE`.

## Reading the logs

Shot Tuning writes every ball to `/sdcard/FIRST/ShotTuning_v2.csv` on the
Hub. `tools/analysis/tuning_report.py` turns it into table rows, and
`tuning_to_fit.py` into input for `fit-shots.js`. For a grid run,
`tools/analysis/gridreport.py` and `trackshots.py` do the same from the
PsiKit log and the field webcam. See `tools/analysis/README.md`.
