# Shoot on the move, design

Lebob Robotics, FTC 29550 · BIOBUZZ 2026/27 · v1.0, 20 Sep 2026

## Purpose

This document describes how the robot scores Pollen and Nectar into the Cell
while driving. It follows the method FRC 4414 used for REBUILT: work out every
shot that scores ahead of time on a laptop, keep the one with the most room for
error, and give the robot a table to look up at match time. The robot handles
the sideways part of its own motion live by turning toward a lead point.

It builds on the teleop design (`2026-09-20-biobuzz-teleop-design.md`), which
listed distance-based shooter speed as out of scope. That is in scope here.

## What we have to work with

The shooter is two flywheels at a fixed launch angle. There is no turret and no
hood. The two things the code can change are flywheel speed and where the robot
is pointing, and a mecanum drive can turn while it translates, so the drivetrain
is the turret.

This shapes the maths. 4414 had a hood, so their table could trade launch angle
against speed. Ours cannot. Whether a shot exists at all depends on how fast the
robot is closing on or backing away from the Cell, because the robot's radial
speed adds to the ball's and a fixed angle only lands within a narrow speed
band. The table therefore has two inputs, distance and radial velocity, and the
robot must be told when no shot exists.

Sensors: goBILDA Pinpoint for field pose and field-frame velocity
(`getVelX`, `getVelY`, `getHeadingVelocity` are in the SDK driver), one webcam
seeing the AprilTag cluster on the underside of the Cell.

## How the FRC teams do it

- **4414 (REBUILT, tech binder):** for each (distance, radial velocity) pair,
  simulate every hood angle and flywheel speed, keep those that score, pick the
  middle of the valid band, fit a polynomial for fast lookup. Tangential
  velocity is corrected live by rotating the turret. Tilt compensation covers
  the FRC bump; the FTC field is flat so we skip it.
- **6328 (REBUILT, public code):** interpolated maps of distance to hood, speed
  and time of flight, then a loop that predicts where the launcher will be after
  the time of flight and aims from there. No physics at runtime. This is the
  alternative to putting velocity into the table. We use the table instead
  because a fixed hood makes the radial term change whether a shot exists, and a
  lookahead loop cannot express that.
- **BIOBUZZ simulator (community, FTC Java):** subtracts robot velocity from the
  wanted ball velocity and re-solves azimuth, elevation and speed. Its useful
  result for us is a worked case showing a fixed hood cannot match both the
  horizontal and vertical components, so closing at 0.4 m/s from 40 in leaves
  the ball short of the Cell. Its POLLEN and NECTAR constants seed ours.

## Architecture

Three parts, kept apart so each can be checked on its own.

1. **Shot solver and visualiser**, one HTML file run in a browser on a laptop.
   It holds the physics, sweeps the inputs, draws the results and exports the
   Java table. Nothing else in the repo has a copy of the ballistics.
2. **`ShotTable.java`**, a generated constants file committed to the repo, with
   the settings that generated it in a header comment.
3. **Robot code**: a pure `ShotSolver` that reads the table, a `TargetTracker`
   that holds the Cell position in field coordinates, and changes to `Robot`,
   `ShooterSubsystem` and the drive so aiming and firing use them.

### Offline solver and visualiser

`tools/shots/index.html`, vanilla JavaScript with Plotly from its CDN for the
plots (so it needs internet the first time; the browser caches it after).
Opened straight from the file system, no server, no build step. The physics
lives in `tools/shots/shots.js`, loaded by the page and by a node script
`tools/shots/export.js` that regenerates the Java table from
`tools/shots/inputs.json`, so the table can be rebuilt without a browser. A
solver in the browser is chosen over Python because the page needs the physics
anyway to draw trajectories, and one copy of the maths is better than two that
drift.

**Inputs**, editable in a form on the page, with the measured values from the
characterisation session (below) as defaults:

- Ball: diameter, mass, drag coefficient, for Pollen and Nectar (toggle).
  Start values from the AndyMark spec (2.80 in, 24.9 g; 3.62 in, 41.3 g) and a
  drag coefficient of 0.45, which is a guess until calibrated.
- Shooter: launch angle, exit height above the tiles, exit speed per RPM
  (linear, one constant), shooter offset forward of the robot centre, flywheel
  speed tolerance.
- Cell: near lip height (53.5 in from the teleop spec), opening 20 in wide by
  14 in tall, opening tilt 30° from horizontal with the far edge higher than
  the near lip, ball clearance margin at each edge. These are read from
  Competition Manual Figure 9-10 and the field CAD before the first table is
  exported, and the page shows them on the plot so an error is visible.
- Sweep ranges: distance 0.6 to 3.0 m in 0.1 m steps, radial velocity −1.0 to
  +1.0 m/s in 0.1 m/s steps (positive is closing), RPM 1500 to 5500 in 25 RPM
  steps.

**Physics**: two-dimensional flight in the vertical plane through the Cell
centre, gravity plus quadratic drag, fixed-step integration at 2 ms. The ball
leaves with the exit velocity from the flywheel plus the robot's radial
velocity added horizontally. A shot scores when the path crosses the opening
segment travelling downward, clears the near lip by the ball radius plus
margin, and lands inside the far edge by the same margin. Spin, Magnus lift and
bounce out of the Cell are not modelled.

**Selection**: for each (distance, radial velocity) the valid RPMs form a band.
The table stores the band centre, the band width, and time of flight at the
centre. A cell is marked invalid when the band is narrower than twice the
flywheel tolerance. This is 4414's "most robust to errors" rule with speed as
the only knob. No polynomial fit: the grid is 25 by 21 and the hub interpolates
it directly.

Running the sweep with the placeholder inputs shows the band is 125 to 175 RPM
at every distance, for any launch angle from 45° to 75°: the near lip sets the
floor and the far edge of the 14 in opening sets the ceiling. Radial velocity
shifts the band rather than closing it, which is the point of putting it in
the table. The consequence is that the flywheel tolerance has to be 50 RPM,
not the teleop spec's 100, and holding that is a shooter tuning requirement
before any moving shot is attempted.

**Visualiser**, three panels matching the 4414 binder image:

- Trajectory fan for the selected distance and radial velocity. Every RPM in the
  sweep is drawn, green if it scores, red if it misses, the chosen shot in blue.
  The Cell opening and near lip are drawn to scale, with an arrow for the robot
  velocity.
- Valid RPM band against radial velocity at the selected distance, with the
  chosen centre line. This is where the fixed-hood limit shows: the band
  closes at some closing speed and that is the speed the driver cannot exceed.
- Heatmap of chosen RPM over distance and radial velocity, invalid cells grey,
  with a toggle to show band width instead.

Two sliders (distance, radial velocity), a ball toggle, and a readout of RPM,
band width, time of flight and lateral tolerance at that distance.

**Export**: a button downloads `ShotTable.java` with the grid as `double[][]`
arrays plus the axis values, and the input form as a comment block at the top.
A second button downloads a CSV of the same for spreadsheets. The Java file is
committed; the page is the source of truth for regenerating it.

**Self-check on load**: with drag set to zero the integrator is compared with
the closed-form parabola at three points and must agree within 1 mm, and the
Pollen table at zero velocity must have RPM increasing with distance. A failure
shows a red banner and disables export.

### `ShotSolver` (robot, pure Java, unit tested)

A plain class with no static state, constructed with the table and the shooter
constants.
One method, `solve(pose, velocity, target)`, returns a small result object:
`rpm`, `headingRad`, `valid`, and for telemetry `distance`, `radialVel`,
`tangentialVel`, `timeOfFlight`, `bandWidth`.

Steps:

1. Advance the pose by velocity times the feed delay (time from indexer feed
   command to ball exit, measured on the robot). This is the only lookahead
   needed, because robot velocity during flight is already inside the table.
2. Shooter position is the advanced pose plus the shooter offset rotated by
   heading. Distance and bearing are measured from there to the target.
3. Split the field velocity into radial (along the bearing, positive closing)
   and tangential components.
4. Bilinear interpolation of RPM, band width and time of flight at (distance,
   radial velocity). Outside the grid or inside an invalid cell gives
   `valid = false`.
5. Heading lead: the horizontal exit speed is the RPM times the exit speed
   constant times cos(launch angle). The robot turns away from its tangential
   motion by asin(tangential / horizontal exit speed) so the ball's horizontal
   velocity, exit plus robot, points along the bearing. Radial exit speed
   changes by the cosine of that lead, under 3 percent below 15°, and is
   ignored.

### `TargetTracker` (robot)

Holds the up-facing Cell's opening centre in field coordinates. On each cluster
detection from `VisionSubsystem` it takes the robot pose at the frame's
timestamp from a ring buffer of the last 50 Pinpoint poses, adds the cluster's
robot-relative position, and stores the result with the time. Between
detections the Pinpoint carries the aim. The stored point expires after a
configurable age (start at 5 s) so a robot that has not seen a tag for a while
is told rather than left guessing. The Cell moves when the Hive tips, so no
field constant is used.

The camera's position and yaw on the robot go into `Constants` once the mount
is in the CAD.

### Changes to existing subsystems

- `ShooterSubsystem`: `setTargetRpm(double)` replaces the single setpoint.
  `atSpeed()` compares against the current target. A D-pad up/down trim adds
  or subtracts 50 RPM to every target for the rest of the run, shown on
  telemetry, for when the table is slightly off on the day.
- `OdometrySubsystem`: exposes field velocity from the Pinpoint and keeps the
  timestamped pose ring buffer.
- `MecanumDriveSubsystem`: no change. `Robot` supplies the rotation input.
- `Robot`: while Y is held the rotation input comes from a proportional
  heading controller on the solver's heading, the left stick still translates,
  and the shooter runs at the solver's RPM. Fire (X) feeds only when the solver
  is valid, the shooter is at speed, heading error is inside the tolerance
  set by the Cell width at that distance, and the target is not expired. When
  the solver is invalid the shooter idles at the stationary RPM for the current
  distance and the Driver Station shows "NO SHOT: closing too fast" or
  "NO SHOT: out of range".
- `Constants`: feed delay, shooter offset, launch angle, exit speed constant,
  heading gains, target expiry, camera transform.

### Logging

Each loop while aiming, one CSV row to `/sdcard/FIRST/shots/<opmode start
time>.csv`: time, pose, velocity, distance, radial and tangential velocity,
table RPM, measured RPM per wheel, heading error, valid flag, fired flag. Pulled
off the hub over ADB after practice. The file is the tuning tool since Wi-Fi
dashboards are banned at events.

## Measurements needed before the first table

Done once on the robot, recorded in `tools/shots/measurements.md`:

1. Launch angle, from CAD and checked with a protractor.
2. Exit height above the tiles.
3. Exit speed per RPM: fire Pollen at three RPMs from a fixed spot, slow-motion
   video against a metre rule, fit a line through the origin.
4. Drag coefficient: compare the landing distance of those shots to the page's
   prediction and adjust until they agree. Repeat with Nectar.
5. Feed delay: video the indexer command LED and the ball exit, or count loops
   from the fire command to the shooter current spike.
6. Time of flight at two distances, to check the table's TOF column.

## Testing

**JVM unit tests** (`ShotSolverTest`, `ShotTableTest`):

- Interpolation returns grid values at grid points and the midpoint between
  neighbours.
- Stationary robot: heading equals the plain bearing and RPM equals the zero
  velocity column.
- Strafing left past the Cell: heading is to the right of the bearing by
  asin(v / horizontal exit speed).
- Closing at 0.5 m/s: RPM is lower than stationary at the same distance.
- Past the band edge or outside the grid: `valid` is false.
- Feed delay moves the distance by velocity times delay.
- `TargetTracker` places the target from a pose in the buffer, not the current
  pose, and reports expiry.

**Page self-check**, described above, runs on every load.

**On-robot ladder**, each rung recorded in the PR before the next starts:

1. Stationary at 1.0, 1.5, 2.0 and 2.5 m: at least 8 of 10 Pollen in the Cell.
   Adjust exit speed constant or drag until the table matches, regenerate.
2. Strafing across the Cell at a steady speed with heading lead only: 7 of 10.
3. Driving toward and away at a steady speed: 7 of 10, and the "NO SHOT"
   message appears at the speed the band chart predicts.
4. Free driving by the driver: log and video, count hits, fix what the log
   shows.

## Out of scope

Acceleration compensation, spin and Magnus lift, bounce-out modelling, tilt
compensation, polynomial fitting, an on-robot tuning interface, Nectar-specific
runtime tables (the runtime uses the Pollen table until the intake can tell the
two apart), and any change to the shooter hardware. If the band chart shows the
usable closing speed is too low to be worth having, the answer is an adjustable
hood and that is a hardware conversation.

## Open items

- Cell opening geometry from Figure 9-10 and the CAD: tilt, and whether the
  14 in "tall" is along the tilted face or vertical.
- Camera mount position and yaw.
- Whether the camera keeps the tag in view while the robot turns to lead a shot
  at high tangential speed. The tracker's expiry handles short losses; long
  losses mean a wider lens or a second camera, which is out of scope here.
- Pinpoint velocity noise. If the logged radial velocity is noisy enough to
  flicker the valid flag, a short moving average goes in front of the solver.
