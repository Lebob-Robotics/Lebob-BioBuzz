# Shot table tools

Lebob Robotics, FTC 29550 · BIOBUZZ 2026/27 · v1.0, 20 Sep 2026

The robot cannot change its launch angle, so whether a shot lands while
driving depends on distance and on how fast the robot is closing on the Cell.
These tools work that out on a laptop and give the robot a table.

## Files

- `shots.js`: the physics, the sweep and the Java export. The only copy.
- `index.html`: the visualiser. Open it in a browser (it fetches Plotly from
  the internet the first time). Change inputs, press Compute, move the
  sliders, export.
- `export.js`: `node tools/shots/export.js` regenerates
  `TeamCode/.../ShotTable.java` from `inputs.json`. Run it after changing
  `inputs.json` and commit both files together.
- `inputs.json`: the measured constants behind the committed table.

- `hood.js`: sizes an adjustable hood. Sweeps hood angle and RPM at every
  distance and radial velocity in the envelope, keeps the shots that survive
  the hood and flywheel tolerances, and searches for the narrowest hood travel
  that still covers every cell. `node tools/shots/hood.js` prints it.
- `hood.html`: the same, with plots. Open it, press Compute, move the sliders.

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

Record here, then copy into `inputs.json` and regenerate.

| Input | How to measure | Value | Date | Who |
| --- | --- | --- | --- | --- |
| `launchAngleDeg` | From the CAD, checked with a protractor on the exit guide | | | |
| `exitHeightM` | Tape from tiles to the ball centre at exit | | | |
| `exitSpeedPerRpm` | Fire Pollen at 3000, 4000, 5000 RPM from a fixed spot, slow-motion video past a metre rule, exit speed over RPM, fit a line through zero | | | |
| `dragCd` | Compare the landing distance of those shots to the page's prediction, adjust until they match. Start at 0.45 | | | |
| `shooterOffsetM` | Ball exit point forward of the robot's tracking point | | | |
| `openingTiltDeg`, `openingLengthM`, `lipHeightM` | Competition Manual Figure 9-10 and the field CAD | | | |
| `Constants.FEED_DELAY_S` | Video the indexer starting and the ball leaving, or count loops to the shooter current spike | | | |
| `Constants.CAMERA_*` | From the CAD once the mount exists | | | |

Time of flight at two distances, from the same video, checked against the
table's `TOF_S` column.

The table's distances are to the Cell's near lip, but the tag cluster's
reported position (what `TargetTracker` places) is the opening centre;
`ShotTable.LIP_TO_CENTRE_M`, generated alongside the table, converts one to
the other before the lookup.

## Why the tolerance is 50 RPM

With a fixed launch angle the near lip sets the lowest speed that scores and
the far edge of the 14 in opening sets the highest. With the current inputs
that window is about 0.3 m/s of ball speed, 150 to 225 RPM, at every distance
the table keeps. The table only
keeps cells whose band is at least twice `rpmToleranceRpm`, so the flywheel
must hold ±50 RPM (`Constants.SHOOTER_TOLERANCE_RPM`). Tune the shooter PIDF
until the logged `left_rpm` and `right_rpm` sit inside that at a steady
target before spending time on moving shots. A bigger window needs a hood,
which is a hardware conversation.

## Reading the logs

Each run writes `/sdcard/FIRST/shots/<date-time>.csv` on the hub while Y is
held and the Cell position is known. Pull them with:

    adb pull /sdcard/FIRST/shots ./shots

Columns are in `ShotLog.HEADER`. `valid` and `fired` are 0 or 1. A shot that
missed while `valid` was 1 and `heading_err_rad` was inside tolerance means the
table is off at that `dist_m` and `radial_mps`: check `exitSpeedPerRpm` and
`dragCd` first.
