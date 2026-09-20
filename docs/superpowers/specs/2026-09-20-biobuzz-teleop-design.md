# BIOBUZZ TeleOp robot code, design

Lebob Robotics, FTC 29550 · BIOBUZZ 2026/27 · v1.0, 20 Sep 2026

## Purpose

This document describes the robot code for the WA Qualifier. It covers the
drive, intake, indexer, shooter, odometry and camera for driver control. There
is no autonomous in this pass. The code is structured so Pedro Pathing can be
added later without rewriting the mechanisms.

## The robot

- Mecanum drive, four goBILDA Yellow Jacket 435 RPM motors.
- Intake: one front roller of compliant wheels, belt driven from a 5.2:1
  (1150 RPM) Yellow Jacket.
- Indexer: an inclined ramp of compliant wheels that carries balls to the
  shooter, driven by one 5.2:1 (1150 RPM) Yellow Jacket.
- Shooter: two flywheels, one 1:1 (6000 RPM, 28 ticks per rev) Yellow Jacket
  each.
- goBILDA Pinpoint with two dead-wheel pods on I2C.
- One UVC webcam on the Control Hub.
- REV Control Hub plus one Expansion Hub. Eight motors fills both hubs.

## What the game asks of the code

Rule references are to the BIOBUZZ Competition Manual V1 with Team Update 01.

- Score by launching Pollen and Nectar into the upward-facing Cell of our
  Hive. A Hive Tip is 20 points. The Cell opening is 20 in wide by 14 in tall
  with its bottom edge 53.5 in above the tiles.
- Each Cell carries a four-tag AprilTag cluster whose origin is the centre of
  the opening. The Cells move, so tags are for aiming only (SDK v12 release
  notes).
- A robot may control at most four balls at once (G407). The mechanism
  enforces this. The code does not count balls.
- Robot Controller must be one Control Hub with at most one Expansion Hub
  (R701). One webcam is allowed (R708). FTC Dashboard and other Wi-Fi
  streaming tools are banned at events (R704), so tuning happens through
  Driver Station telemetry.

## Step 0: SDK v12.0

FTC released SDK v12.0 on 12 September 2026 as the BIOBUZZ season release. The
repo is on the offseason v11.2.1. We move to v12.0 before writing new code,
because the camera work needs the BIOBUZZ tag library and cluster API that
only v12 has, and because the season release is what inspectors expect.

The move replaces the `FtcRobotController` module, `build.gradle`,
`build.common.gradle`, `build.dependencies.gradle` and the Gradle wrapper with
the v12.0 versions, then reapplies our two changes: the FTCLib lines in
`TeamCode/build.gradle`, and any README or CI package pins that the new
`build.common.gradle` moves. FTCLib 2.1.1 targets the same SDK APIs, so it is
expected to compile unchanged. If it does not, the plan stops and we decide
whether to drop FTCLib or patch around it.

## Architecture

We keep the FTCLib command-based layout that is already in the repo. One
`OpMode` (`Main`) owns a `Robot`. `Robot` builds one subsystem per mechanism,
runs the `CommandScheduler` each loop, and maps the gamepad. Each subsystem is
one file under `subsystems/` and exposes a handful of methods. No interfaces,
no factories.

A `Constants` class holds the hardware config names, motor directions and
tuning values (shooter RPM, aim gain, tolerances). It exists because Pedro
Pathing will need the same motor names and directions, and because tuning
numbers need one home the drive team can find.

Both hubs are set to `BulkCachingMode.AUTO` in the `Robot` constructor so the
eight motor reads cost two bus transactions per loop.

### Subsystems

**MecanumDriveSubsystem** (existing). Unchanged behaviour. The mixing maths
moves into a static `MecanumKinematics.mix(forward, right, rotate)` that
returns four normalised powers, so it can be unit tested on the JVM and so a
Pedro Pathing follower can replace the subsystem later without touching the
maths.

**OdometrySubsystem** (existing). Unchanged. The pod offsets in `init()` are a
tuning task, done on the robot with the goBILDA setup procedure. Pedro Pathing
ships its own Pinpoint localiser, so when it arrives this subsystem either
stays for teleop heading or is removed.

**IntakeSubsystem.** One `DcMotorEx`, `RUN_WITHOUT_ENCODER`, zero-power
`FLOAT`. Methods: `run()`, `reverse()`, `stop()`.

**IndexerSubsystem.** One `DcMotorEx`, `RUN_WITHOUT_ENCODER`, zero-power
`BRAKE` so held balls do not roll back. Methods: `feed()`, `reverse()`,
`stop()`.

**ShooterSubsystem.** Two `DcMotorEx` in `RUN_USING_ENCODER`, one reversed so
both wheels throw forward, zero-power `FLOAT` so the wheels spin down on their
own. Velocity control uses the hub's built-in velocity PIDF through
`setVelocityPIDFCoefficients`, with the gains in `Constants`. One shared RPM
setpoint. Methods: `spinUp()`, `idle()`, `stop()`, `atSpeed()` (both wheels
within `SHOOTER_TOLERANCE_RPM` of the setpoint), `getVelocityRpm()` for
telemetry. The conversion from RPM to encoder ticks per second is a static
function so it can be unit tested. Starting values: 3500 RPM setpoint, 100 RPM
tolerance, SDK default PIDF. All three are tuned on the robot.

**VisionSubsystem.** One webcam through `VisionPortal` with the v12
`AprilTagProcessor` using `getCurrentGameTagLibrary()`. It keeps the latest
detection of our alliance's Cell cluster, chosen by tag ID from the library,
and exposes `hasTarget()` and `getBearingDeg()`. Cluster detections are used
where the SDK provides them, because their pose points at the opening centre
even when only one member tag is visible. The live stream is off during a match
loop to save CPU. Which IDs belong to which alliance is read from the v12 tag
library during implementation.

### Controls

One driver on gamepad1. Alliance colour is chosen during `init` with D-pad
left (red) or right (blue), shown on telemetry, and defaults to red.

| Input | Action |
| --- | --- |
| Left stick | Translate (field-centric by default) |
| Right stick X | Rotate |
| Left bumper (hold) | Robot-centric translate |
| A | Zero heading |
| Right trigger (hold) | Intake and indexer run |
| Left trigger (hold) | Intake and indexer reverse |
| Right bumper (toggle) | Shooter spin up / idle |
| X (hold) | Fire: indexer feeds only while the shooter is at speed |
| Y (hold) | Aim assist: rotation comes from the tag bearing, not the stick |

Aim assist is a proportional controller on bearing with a deadband and an
output clamp, gains in `Constants`. With no target visible the right stick
rotates as normal. Triggers count as pressed above 0.2.

### Telemetry

Each loop: pose, alliance, shooter target and measured RPM per wheel, at-speed
flag, tag bearing or "no target", hub battery voltage. Nothing else, so the
Driver Station stays readable.

### Hardware configuration names

| Config name | Type | Mechanism |
| --- | --- | --- |
| `front_left_drive` | goBILDA 5202/3/4 | Drive |
| `front_right_drive` | goBILDA 5202/3/4 | Drive |
| `back_left_drive` | goBILDA 5202/3/4 | Drive |
| `back_right_drive` | goBILDA 5202/3/4 | Drive |
| `intake` | goBILDA 5202/3/4 | Intake |
| `indexer` | goBILDA 5202/3/4 | Indexer |
| `shooter_left` | goBILDA 5202/3/4 | Shooter |
| `shooter_right` | goBILDA 5202/3/4 | Shooter |
| `pinpoint` | goBILDA Pinpoint (I2C) | Odometry |
| `Webcam 1` | Webcam | Vision |

This table goes into the README with a ports column, filled in by whoever
wires the hubs, because the PR template already tells contributors to keep it
current.

## Testing

Robot code cannot run off the robot, so verification is in two parts.

**JVM unit tests** for the pure maths only: mecanum mixing (straight, strafe,
rotate, saturation) and RPM to ticks-per-second. These run in CI with
`./gradlew :TeamCode:testDebugUnitTest` and need JUnit 4 as a
`testImplementation` dependency. Nothing with an SDK import gets a unit test.

**On-robot checklist**, one per subsystem, recorded in the PR under "Tested on
the robot":

1. Drive: each motor spins forward on positive power, robot drives straight,
   strafes right on stick right, field-centric holds direction after a spin.
2. Odometry: Pinpoint LED green, X grows driving forward, Y grows driving
   left, heading grows turning anticlockwise, spin in place moves X/Y under
   100 mm, return to start reads under 10 mm.
3. Intake and indexer: run, reverse, stop, indexer holds a ball when stopped.
4. Shooter: reaches setpoint, telemetry RPM within tolerance, at-speed flag
   flips, fires a Pollen into the Cell from the practice spot.
5. Vision: telemetry shows a bearing when a Cell tag is in view, aim assist
   turns the robot toward it and settles.

## Pedro Pathing later

Pedro Pathing brings its own `Follower` that owns the drive motors and a
Pinpoint localiser. When it is added: the follower's teleop drive replaces the
`drive.drive()` call in `Robot.periodic()`, `Constants` supplies the same motor
names and directions to Pedro's constants, and autonomous OpModes are new
files. Intake, indexer, shooter and vision do not change.

## Out of scope

Autonomous, distance-based shooter speed, ball counting, flower and garden
mechanisms, driver two controls, FTC Dashboard.

## Open items

- Pod offsets, shooter setpoint, PIDF and aim gain are measured on the robot.
- Hub port assignments go into the README table when the robot is wired.
- The camera mount position on the robot is not in the CAD yet. Vision code
  assumes the camera faces forward and is roughly on the robot centreline.
