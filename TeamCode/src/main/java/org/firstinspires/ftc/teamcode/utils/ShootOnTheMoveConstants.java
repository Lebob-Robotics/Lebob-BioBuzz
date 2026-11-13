package org.firstinspires.ftc.teamcode.utils;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Tuning values for shoot on the move, editable live from Panels. Metres, seconds and radians unless
 * the name says otherwise.
 *
 * <p>Positions on the robot are from the robot centre, the middle of the four mecanum wheels, the
 * same point the Pinpoint offsets in OdometrySubsystem use. Robot and field CAD values are from the
 * Onshape exports of 25 Oct 2026 (FTC2026/files).
 *
 * <p>Every value is an estimate until the robot is built and measured. The hood angle and RPM for
 * each shot are not here: they are in the generated {@link ShotTable}, made from
 * tools/shots/inputs.json.
 */
@Configurable
public final class ShootOnTheMoveConstants {
  private ShootOnTheMoveConstants() {}

  // Fire command to ball leaving the shooter. The flywheels' RPM dips 0.12-0.30 s after the command, mostly
  // about 0.2 s (nine feeds, 4-5 Nov logs). One ball per pulse, the dip came 0.20-0.35 s after it, and the pose
  // acted on is a loop (about 55 ms) old: at 0.20 two strafing shots clipped the Cell's edge on the side the robot
  // was moving toward (6 Nov, 080732)
  public static double FEED_DELAY_S = 0.30;
  public static double FEED_DELAY_MAX_S = 0.30; // Latest a ball leaves: the heading must still be on target then
  // Time constant of the low-pass on the velocity the aim uses. The ball leaves FEED_DELAY_S after the decision, so
  // it needs the mean velocity over that time, not one loop's Pinpoint sample. Unsmoothed, the sideways speed's noise
  // swung the aim heading 2.4-3.4 degrees rms during passes and the lock chased it (7 Nov, 100030). Cost: through a
  // steady 0.4 m/s^2 sideways change the aim lags about 2.5 degrees; the steady gate refuses faster changes. 0 is off
  public static double AIM_VELOCITY_SMOOTHING_S = 0.12;

  // ---- Shooter on the robot (CAD) --------------------------------------
  public static double SHOOTER_DIRECTION_RAD = Math.PI; // Relative to robot forward. PI fires out of the back
  public static double SHOOTER_FORWARD_M = -0.0066; // Flywheel axle. The sim works out the exit point from the hood angle
  public static double SHOOTER_LEFT_M = 0.0007;

  // ---- Cell (field CAD) ------------------------------------------------
  public static double CLUSTER_TO_NEAR_LIP_M = 0.157; // Near lip is this much nearer the robot than the tag cluster centre
  public static double MIN_UP_CELL_HEIGHT_M = 1.08; // Halfway between the up (1.26 m) and down (0.90 m) Cells' clusters
  public static double CELL_HALF_WIDTH_M = 0.254; // Opening is 20 in wide
  public static double BALL_RADIUS_M = 0.0356; // Pollen, 2.8 in across
  public static double SIDE_MARGIN_M = 0.02; // Kept clear of each side of the opening

  // ---- Remembering the Cell --------------------------------------------
  // Odometry corrected by the Hive tags carries the Cell's position between sightings, and a backward shot
  // turns the cameras away from it, so it is kept well past one turn-around.
  public static double CELL_EXPIRY_S = 150.0; // Forget the Cell this long after the last sighting. A whole match: odometry carries it
  public static double CELL_SWITCH_AFTER_S = 1.0; // Switch to another of our Cells once the held one is unseen this long
  // Aim at the up Cell's field position once the tags have placed the robot; the tracker's Cell before that
  public static boolean AIM_AT_FIELD_CELL = true;
  // Aim at our Cell on the robot's side of the Hive (x > 0 AUDIENCE, x < 0 SCORING), not the one the tags vote up:
  // the vote flipped 2-7 times a run on 13 Nov, and the far volleys went to the far Cell, past and right of the
  // near one the driver meant. Which Cell is right is the driver's call (14 Nov). False: the voted up Cell
  public static boolean AIM_AT_FRONT_CELL = true;
  // TeleOp's Driver Station: false shows the driver's few lines (shot, aim, power, problems); true every subsystem's
  // state, as before 14 Nov. The tuning OpModes always show it all
  public static boolean TELEMETRY_DETAIL = false;
  public static double FRONT_CELL_SWITCH_M = 0.15; // Past x = 0 by this much before the Cell changes sides
  // Drive power at full stick while RT or RB is held. Full: the hood, wheels and heading follow the shot's rates
  // now, so the driver doesn't have to slow down to shoot
  public static double AIM_DRIVE_MAX_POWER = 1.0;
  public static double CELL_SWITCH_MARGIN_M = 0.3; // ...or sooner if the other is at least this much nearer
  public static double CELL_SMOOTHING = 0.3; // Share of the way the stored Cell moves toward each sighting. 1 is no smoothing
  public static final int POSE_HISTORY_SIZE = 100; // About two seconds of loops

  // ---- Cameras (CAD lens centres: both face backward and 35 deg out, tilted up 41.1 deg) ----
  // Both cameras are mounted upside down: roll must be undone before pitch and yaw.
  // The replacement left camera's yaw and pitch are fitted to the right camera's tag positions, not CAD
  // (6 Nov calibration drive, 824 sightings both cameras made of the same tags).
  public static final CameraMount LEFT_CAMERA = new CameraMount(
      -0.1624, 0.0377, 0.2093, Math.PI - Math.toRadians(37), Math.toRadians(39.85), Math.PI);
  public static final CameraMount RIGHT_CAMERA = new CameraMount(
      -0.1457, -0.0625, 0.2093, -(Math.PI - Math.toRadians(35)), Math.toRadians(41.09), Math.PI);

  // ---- Field frame -----------------------------------------------------
  // The odometry heading is in the field frame (0 along +x, anticlockwise positive), so the camera can
  // use the tags' field positions. The drive stays driver relative: heading minus the forward below.
  // Heading the driver calls "forward", standing at their wall. The tags set the field heading, so these must
  // match the real field. Each driver stands at the wall nearest their own Cells (red tags at -y, blue at +y), with
  // both Cells off to the sides, 90 degrees from them: red faces +y, blue -y (6 Nov, from the drivers)
  public static double ALLIANCE_FORWARD_RED_RAD = Math.PI / 2;
  public static double ALLIANCE_FORWARD_BLUE_RAD = -Math.PI / 2;

  // ---- Vision pose correction (from Sean's feature/sotm) -----------------
  // The Hive tags give the robot's field position. Only each tag's ftcPose x/y/z is used, not the SDK's
  // robotPose: the tags move when a Hive tips. Hive tag ids: RED SCORING 30-33, RED AUDIENCE 34-37,
  // BLUE AUDIENCE 38-41, BLUE SCORING 42-45. AUDIENCE Cells are at +x and SCORING at -x.
  public static double CLUSTER_UP_X_M = 0.3329; // Tag cluster centre of the up Cell: |x| and height
  public static double CLUSTER_UP_Z_M = 1.2646;
  public static double CLUSTER_DOWN_X_M = 0.2949; // Same for the down Cell
  public static double CLUSTER_DOWN_Z_M = 0.9022;
  public static final double[] TAG_Y_IN = { // By tag id less 30, four per Cell
      -6.25, -10.0, -15.5, -19.25,
      -19.25, -15.5, -10.0, -6.25,
      6.25, 10.0, 15.5, 19.25,
      19.25, 15.5, 10.0, 6.25};
  public static final double IN_TO_M = 0.0254;
  // Where the SDK cluster detector puts a Cell, from the centre of its line of tags: up, and toward the
  // camera (negative is away). Measured 4 Nov against the SDK on the same frames; see ClusterFromTags.
  public static double CELL_REF_UP_UP_IN = 8.0; // Cell up
  public static double CELL_REF_UP_TOWARD_IN = 4.2;
  public static double CELL_REF_DOWN_UP_IN = 0.5; // Cell down
  public static double CELL_REF_DOWN_TOWARD_IN = -7.1;

  public static double Z_TOL_M = 0.10; // A tag's measured height must be this close to the up or down cluster height
  public static int TIP_CONFIRM_FRAMES = 3; // Frames in a row voting for a flip before a Hive's up Cell changes

  public static double MIN_DECISION_MARGIN = 20; // Placeholder, tune
  public static double MAX_TAG_RANGE_M = 3.0;
  // Faster than this the frame's pose is too uncertain. Odometry held through the field test's moves (fixes of
  // 1 mm still), while frames taken above 0.6 m/s or 35 deg/s were off by 5 cm and 2 degrees (median) and up to
  // 13 cm and 11.5 degrees, which the aim then used (5 Nov field tests, 7381 fixes)
  public static double MAX_SPEED_FOR_VISION_MPS = 0.6;
  public static double MAX_OMEGA_FOR_VISION_RAD_S = 0.6;
  // The heading from the tags only once nearly still: a frame's heading error grows with the turn rate, and the
  // odometry's own heading turned 158 degrees on the spot without a degree of error. The first fix ignores this
  public static double MAX_SPEED_FOR_VISION_HEADING_MPS = 0.3;
  public static double MAX_OMEGA_FOR_VISION_HEADING_RAD_S = Math.toRadians(20);
  public static double MAX_FRAME_AGE_S = 0.5; // Pose history covers it. One detector pass ran 0.3 s behind on average (4 Nov)
  public static double MAX_SPREAD_M = 0.10; // Most any tag's correction may differ from the median
  public static double MAX_JUMP_M = 0.5; // Largest correction once the pose is trusted. The first fix may be any size
  public static double FIELD_HALF_SIZE_M = 1.83;
  public static double ROBOT_HALF_SIZE_M = 0.15;
  public static double CORRECTION_GAIN = 0.5; // Share of each correction applied. The first fix applies all of it
  // A jump bigger than MAX_JUMP_M that this many frames agree on (within POSE_OVERRIDE_AGREE_M) means odometry
  // lost the position, so it is replaced in full. 22 rejected fixes agreed within 5 cm on a 2 m error (5 Nov, 082826)
  public static int POSE_OVERRIDE_FRAMES = 5;
  public static double POSE_OVERRIDE_AGREE_M = 0.15;
  // Heading from the tags: two tags seen at once give the robot's heading from the line between them.
  public static boolean VISION_HEADING = true;
  public static double MIN_HEADING_BASELINE_M = 0.2; // Tags closer together than this give too noisy a heading
  // The first heading fix is taken in full, so it waits for this many tag pairs: one pair put the start heading
  // 28 degrees out, 9 to 13 pairs a few frames later agreed on the truth (6 Nov, 080409)
  public static int MIN_FIRST_HEADING_PAIRS = 3;
  public static double HEADING_GAIN = 0.2; // Share of each heading correction applied. The first applies all of it
  public static double MAX_HEADING_JUMP_DEG = 20; // Larger corrections once the heading is trusted are ignored
  // ...unless this many frames in a row agree on it (within HEADING_OVERRIDE_AGREE_DEG): then the odometry
  // heading is the wrong one (4 Nov: a bad Pinpoint read left it 172 degrees out), so it is replaced in full
  public static int HEADING_OVERRIDE_FRAMES = 5;
  public static double HEADING_OVERRIDE_AGREE_DEG = 15;

  // ---- Hood travel ------------------------------------------------------
  // Launch angle on the hood's upper hard stop (Hood Tune RANGE). The hood's soft limit and the shots both come
  // from it, so a shot flatter than the hood can reach is no shot rather than one fired at the stop's angle.
  // 54.3: the second hard stop, 5 Nov (HoodTune 083632, lift 44.7); 58.5 the first; 51 before stops
  public static double HOOD_TOP_STOP_DEG = 54.3;
  public static double HOOD_STOP_MARGIN_DEG = 1.0; // Targets stay this far inside the stop

  // ---- Shot checks -----------------------------------------------------
  // Above this, the ball's sideways error grows too fast. 40 keeps a shot strafing at 1.2 m/s from 0.7 m (30 needed
  // 1.0 m); a 10% error in the ball's exit speed then misses sideways by about 6 cm of the opening's 20
  public static double MAX_TURNING_DEG = 40;
  /**
   * Neighbouring table cells are only blended when their hood angles are this close. Further apart
   * they are different kinds of shot, and a blend may miss, so the nearest cell is used. tools/shots/export.js
   * checks every blend in the table scores and prints the widest spread (6 now). Below it, noise flips the
   * nearest cell and the hood target jumps by whole degrees.
   */
  public static double MAX_HOOD_DIFFERENCE_TO_BLEND_DEG = 7;

  // ---- Fire check (match the tolerances in tools/shots/inputs.json) ----
  // Per flywheel. The hub's speed reading steps by about 61 RPM, and the wheels settle a step or so above
  // target (3605 read as 3628-3689, 4 Nov), so 75 held the shot back about 3 s. Two steps
  public static double RPM_TOLERANCE = 125;
  // The two wheels' mean, which the aimed shot also needs. The shot table allows 50 RPM, and 50 RPM is about
  // 0.1 m of range. Both misses of 5 Nov (080106) left 56 and 80-98 RPM fast, one long; balls 63 RPM slow went in,
  // and the wheels lag a target that moves with the robot's speed, so slow gets more room
  public static double RPM_MEAN_ABOVE_TARGET = 50;
  public static double RPM_MEAN_BELOW_TARGET = 100;
  // The speed this far ahead, from its trend, must not be more than this over: wheels climbing back after a ball
  // were fed at +17 and the ball left at +130, long (5 Nov, 080706). Fast side only, and looser than the speed
  // now: balls predicted +61 and +85 went in, and wheels slowing toward the target settle rather than undershoot.
  // The mean steps 30 RPM, so the slope needs 0.2 s
  public static double RPM_PREDICTED_ABOVE_TARGET = 100;
  public static double RPM_LOOKAHEAD_S = 0.08;
  public static double RPM_TREND_WINDOW_S = 0.2;
  public static boolean PRESPIN_ON = true; // Flywheels track the aimed RPM whenever the Cell is known
  public static double PRESPIN_HOLD_S = 1.5; // Keeps that speed this long after the shot goes invalid
  public static double HOOD_TOLERANCE_DEG = 1.0;
  // Otherwise it grows large at short range. The geometric clearance leaves nothing for the other errors: every
  // moving ball that scored left within 1.1 degrees, the misses 7.7 and 13.6 degrees off, all going the way the
  // robot moved (5 Nov, 083109-092330). 4 keeps 92% of moving READY time (soak 081210, median 1.6)
  public static double MAX_HEADING_TOLERANCE_DEG = 4;
  // No shot while moving at least MOVING_SHOT_SPEED_MPS this close to the lip, nor while the hood's target swings
  // faster than MAX_HOOD_RATE_TO_FIRE_DEG_PER_SEC. Strafing past at 0.7-0.8 m/s, balls fed READY at 0.62 and 0.48 m
  // both missed: during the 0.3 s feed the heading error went 7.7 -> 20.8 degrees and the hood target moved
  // 6 degrees; the standing shot from 1.5 m went in (5 Nov, 083109). Costs about a quarter of moving READY time
  public static double MOVING_SHOT_SPEED_MPS = 0.3;
  public static double MIN_MOVING_SHOT_LIP_M = 0.7;
  public static double MAX_HOOD_RATE_TO_FIRE_DEG_PER_SEC = 12;
  // How long an aimed shot must stay ready before the feed starts. Every check above already has to pass,
  // and waiting two or three loops for it to repeat cost more shots than it saved while moving (5 Nov, 084358)
  public static double AIMED_READY_S = 0.0;
  // A feed with no ball out after this long stops once the shot is no longer ready. Past FEED_DELAY_MAX_S, so a
  // ball already on its way isn't cut off
  public static double AIMED_FEED_STOP_UNREADY_S = 0.4;
  // The longest an aimed feed runs before a ball is seen leaving. The webcam saw first balls leave 0.15-0.28 s
  // after the feed began and second balls 0.13-0.4 s behind them, while the wheel speed showed a ball 0.1-0.17 s
  // late, so feeds ran on into the next ball (5 Nov, 084355-092330). After the pulse the feeder holds backward
  // up to AIMED_FEED_PAUSE_S for the dip of a ball already on its way
  public static double AIMED_FEED_PULSE_S = 0.30;
  public static double AIMED_FEED_PAUSE_S = 0.35; // At 0.25 s the third ball's dip came just after the pause (6 Nov, 080814)
  // The wheels lose 300-430 RPM per ball and need about this long to come back (5 Nov, 084355: refed 0.08 s on)
  public static double MIN_BALL_GAP_S = 0.35;
  // From each aimed feed command until this long after, the drive keeps the translation it had at the feed, so the
  // ball leaves at the speed it was aimed for. A ball fed standing still left 0.36 s later with the robot at
  // 0.73 m/s and turning 44 deg/s after the stick was pushed, and missed right (5 Nov, 084758)
  public static double FEED_COMMIT_S = 0.4;
  // Feeds back to back kept renewing the hold, so the robot drove on after the stick was let go and ran into the
  // fence (5 Nov, 090446). Letting go of the stick always stops it, and one hold lasts at most this long
  public static double FEED_COMMIT_MAX_S = 1.0;
  // The commit holds the stick, not the speed: a ball fed in the loop the stick was pushed left with the robot at
  // 0.56 m/s, the wheels 250 RPM fast for that speed, and went long (5 Nov, 085458). So a feed also waits for the
  // drive command to hold within FIRE_STICK_CHANGE for FIRE_STEADY_STICK_S, and for the robot to stop speeding up
  public static double FIRE_STEADY_STICK_S = 0.3;
  public static double FIRE_STICK_CHANGE = 0.1;
  public static double MAX_ACCEL_TO_FIRE_MPS2 = 1.0; // Over ACCEL_WINDOW_S; 8% of moving READY time in soak 081210
  // Over 0.1 s the Pinpoint's speed noise alone read past 1 m/s^2: driving at 0.6+ m/s on a stick held still 0.6 s,
  // 12% of loops over it (median 0.54), and the gate held 40-46% of RT time at 0.6-1.2 m/s. Over 0.2 s, 2% (13 Nov)
  public static double ACCEL_WINDOW_S = 0.2;
  // A feed under way also stops at once, ball out or not, if the hood falls this far off its target (twice the
  // fire check's): the target moves when the robot sets off, and the ball then leaves at the old angle
  public static double FEED_STOP_HOOD_DEG = 2.0;

  // ---- Backward shot (tools/shots/backwards-calibration.json) ---------
  // Out of the robot's front, the other way to the cameras: hood on its lower stop, so the ball leaves
  // over the top. Taped from the flywheel axle to the near lip with the robot still, so there is no
  // speed axis: it is only taken while the robot is nearly stopped.
  public static boolean BACKWARD_SHOTS_ON = true;
  public static double BACKWARD_DIRECTION_RAD = 0; // Relative to robot forward
  public static double BACKWARD_LAUNCH_DEG = 99; // Hood on its lower stop (HoodSubsystem.ZERO_ANGLE_DEG)
  // 0.40 m (Shot Tuning, 6 Nov 082517): on its stop the hood never scored from 0.4-0.6 m (0 of 7 at 3400-3600 rpm,
  // and the G1 grid 0 of 3 at 0.6 m, onto the Cell's bottom); 8 degrees up at 3200 rpm scored 4 of 5, 3150-3300
  // 8 of 13. Shot Tuning 091010: 0.42 m at 90.3 degrees 8 of 10 against 0 of 2 at 91.3; 0.60 m 3270 rpm 94.0 degrees
  // 5 of 7; 0.72-0.74 m as interpolated (3310-3320, 95.5-95.9) 14 of 25, misses unmarked. Shot Tuning 080751 (7 Nov):
  // 0.77 m as interpolated (3320-3330, 95.9-96.5) 4 of 4, 23 degrees off the Cell's axis; 47-51 degrees off,
  // 0.74-0.85 m scored 3 of 14 at 3370-3470 rpm and 94.7-97.7 degrees.
  // Shot Tuning 085918 (7 Nov), each spot's RPM search at the old table's angle, the middle of what scored:
  // 0.63 m 3000-3360 at 94.3 (7 of 7 at 3180), 0.73-0.74 m 3200-3340 and 3020-3350 at 95.8-95.9, 0.76 m 3280-3310
  // at 96.5 (the line between 0.73 and 0.81 gives 3300 there), 0.81 m 3350-3420 at 97.0. 0.40 m: 88-94 degrees all
  // scored at 3200 but none often (14 of 52 over the session), so 90 stays
  // 0.95 m out (13 Nov): the 3 Nov one-ball points went long as volleys in the wedge. Lebot Teleop's 8 far backward
  // balls (173844-225900, hood 97.6-98.2) left at 3505 rpm from 1.53 m, 3566-3597 from 1.52-1.59, 3628 from 1.85 and
  // 3782-3812 from 1.94, all long. The drag model fitted to Shot Tuning's 179 marked backward balls (0.4-0.9 m) puts
  // the opening's half-depth at about 90 rpm out there, so each point sits at least that far under the slowest long
  // ball near it: 1.53 m 3411 (was 3592), 1.85 m 3494 (3750), 1.94 m 3555 (3836). 2.3 m: no ball, the model's
  // shape scaled to 1.85 m. Recheck these with Shot Tuning's backward spots before raising SHOT_PICK_BACKWARD_MAX_LIP_M
  public static final double[] BACKWARD_DIST_M = {0.40, 0.63, 0.73, 0.81, 0.95, 1.3, 1.8, 2.3};
  public static final double[] BACKWARD_RPM = {3200, 3180, 3230, 3390, 3370, 3370, 3460, 3800};
  public static final double[] BACKWARD_HOOD_DEG = {90, 94.3, 95.9, 97.0, 99, 99, 99, 99};
  // Closer than the table's first point, the first point's shot is used
  public static double BACKWARD_MIN_DIST_M = 0.35;
  // Lebot Teleop always takes the backward shot this close (its own distance to the lip) and the hooded one beyond.
  // Standing (6 Nov): backward 0.42-0.60 m 13/17 against hooded ~50%; the drivers chose 0.72 m as the switch. Moved to
  // 0.80 (7 Nov) so the 0.77 m shots the drivers take backward (4 of 4, Shot Tuning 080751) are backward. Hooded at the
  // table's settings scored 46 of 55 at 0.75 m over the same sessions, against 18 of 29 backward
  public static double BACKWARD_PICK_MAX_DIST_M = 0.80;
  public static double BACKWARD_MAX_SPEED_MPS = 0.3; // Toward or sideways
  // A backward shot that stays blocked this long, by speed or by being too close, gives way to the hooded shot
  // if that has one. Driving around the Cell at full stick held a backward pick for 30 s with no shot (5 Nov, 094211)
  public static double BACKWARD_GIVE_UP_S = 1.0;
  // Both TeleOps take whichever shot, hooded or backward, is ready soonest (8 Nov), now that Shot Tuning and SOTM Tuning
  // have calibrated both: no more backward arc. False goes back to the arc above (BACKWARD_PICK_MAX_DIST_M).
  // The time to ready is the longest of the turn, the hood's move and the wheels' change, plus BLOCKED_S if the shot
  // can't be taken yet (too fast for it). Fitted to Lebot Teleop's logs (1-7 Nov, 40 logs): 58 heading-lock turns,
  // median 0.50 s for 10-30 degrees, 0.77-0.92 s for 30-90 and 1.15 s for 90-180; 128 hood moves, median 0.23 s for
  // 4-8 degrees, 0.41 s for 15-25 and 0.47 s for 25-40. The wheels: about 4000 rpm/s at full power
  public static boolean SHOT_PICK_QUICKEST = true;
  public static double SHOT_PICK_TURN_BASE_S = 0.35;
  public static double SHOT_PICK_TURN_DEG_PER_S = 180;
  public static double SHOT_PICK_HOOD_BASE_S = 0.15;
  public static double SHOT_PICK_HOOD_DEG_PER_S = 90;
  public static double SHOT_PICK_RPM_PER_S = 4000;
  public static double SHOT_PICK_BLOCKED_S = 1.0;
  // Beyond this the hooded shot is always taken: the backward table is calibrated standing at 0.40-0.81 m only (Shot
  // Tuning: 331 balls at 0.4-0.8 m, 5 past 0.85), and SOTM Tuning has no backward passes. Its 0.95-2.3 m points
  // were never shot. Raise once Shot Tuning has searched backward spots out there
  public static double SHOT_PICK_BACKWARD_MAX_LIP_M = 0.85;

  // Straight in front of the Hive (the Cell's axis runs along field x, out from its wall) and this many degrees to
  // either side, only the backward shot is taken (13 Nov): the hooded shot's backspin affected shots from there.
  // Ignores SHOT_PICK_QUICKEST and the distance cap above (the backward table is calibrated at 0.40-0.81 m only, so
  // a far shot in the wedge uses its uncalibrated points), and doesn't give way to the hooded shot when blocked.
  // 0 is off
  public static double BACKWARD_ONLY_WEDGE_DEG = 30;

  // Added to every hooded shot from the table. -75 (6 Nov) made up for the model's ball being too slow; the model
  // refitted to Shot Tuning's 224 hooded balls (7 Nov) fires what they found without it
  public static double HOODED_RPM_OFFSET = 0;

  // Aim turned anticlockwise on every shot outside the tuning tools, which set their own. Shot Tuning's aim search
  // settled at these (session 085918) and both tuning tools fire with them; SOTM Tuning's 16 finished hooded spots
  // (7 Nov) put the hooded one at +1.92 far, +2.01 mid, +1.85 near. TeleOp had none and aimed ~2 deg right of them
  public static double HOODED_HEADING_TRIM_DEG = 2.0;
  public static double BACKWARD_HEADING_TRIM_DEG = 1.5;

  // Moving-shot corrections, scaled for fitting them to SOTM Tuning's balls (sotm-tools/sotm_report.py). 1 is the
  // model as built: the sideways speed the shooter turns to cancel, and the speed toward the Cell the table looks up
  public static double SOTM_SIDEWAYS_GAIN = 1.0;
  public static double SOTM_TOWARD_GAIN = 1.0;
  // Straight corrections per m/s of the speed the shot was looked up with, from SOTM Tuning's spot searches: launch
  // angle added for speed toward the Cell, RPM added for it, and aim turned anticlockwise for sideways speed (left +).
  // Fitted 7 Nov to 16 finished hooded spots (0.3-0.7 m/s, 0.8-1.6 m), all left at 0 as within noise:
  //   hood -1.22 +/- 1.02 deg per m/s toward (scatter 1.35 deg; standing error by band far +0.8, mid +0.75, near -1.6)
  //   aim -0.13 +/- 0.32 deg per m/s sideways (scatter 0.41 deg), release delay 0.292 +/- 0.031 s against FEED_DELAY_S
  //   0.30: the moving model is right as built. Left out: F ARC left 0.60 at 0.98 m, whose aim (-5.25) came from 8
  //   balls left in a row at a +6 deg hood trim while the aim step was stuck at 0.25 deg
  public static double SOTM_HOOD_DEG_PER_TOWARD_MPS = 0;
  // Launch angle added to a moving hooded shot by its distance to the lip, in TeleOp only (the tuning tools measure
  // against the bare table). Straight between the points, held past the ends; full from MOVING_SHOT_SPEED_MPS, none at
  // rest. All 17 found spots' hood centres run with distance, not speed: -0.88 +/- 0.18 deg at 1.2 m, +5.53 +/- 0.76
  // deg per m (scatter 1.29 deg; adding toward -0.64 +/- 0.52, adding |sideways| -1.92 +/- 1.37: noise). Standing balls don't show it (Shot Tuning 085918 centres -2.5..+2.4 deg either side of
  // the table at every distance). With it all 16 spots' scoring windows hold the shot (worst 0.57 deg inside); without
  // it, toward-right 0.50 at 0.91 m (window -3.7..-1.5) and field -y 0.70 at 1.14 m (-4.0..-0.5) miss
  public static final double[] SOTM_HOOD_TRIM_DIST_M = {0.95, 1.48};
  public static final double[] SOTM_HOOD_TRIM_DEG = {-2.3, 0.7};
  public static double SOTM_RPM_PER_TOWARD_MPS = 0;
  public static double SOTM_HEADING_DEG_PER_SIDEWAYS_MPS = 0;
  public static double SOTM_BACKWARD_RPM_PER_TOWARD_MPS = 0;
  public static double SOTM_BACKWARD_HEADING_DEG_PER_SIDEWAYS_MPS = 0;

  // ---- Volley (Lebot Teleop RB, RT while VOLLEY_ON_RT, and Volley Tuning) ----
  // A volley is RT's aimed hooded shot fired as a burst: each ball goes as soon as the wheels are back at the matched
  // speed, instead of the one-ball pulse, pause and gap above. One flywheel can't launch four balls at the same
  // instant; the burst is limited by the wheels' recovery between balls.
  //
  // Balls in a burst must leave at the same wheel speed and hood. Modelled (tools/shots, collide.js), two balls 0.1 s
  // apart at matched speed stay 0.12-0.18 m apart in flight (ball 0.071 m), but a second ball 200-300 RPM slower
  // lands first and the paths cross within 0.01-0.05 m. So each ball waits for the matched band below, and the feed
  // power sets how tightly the queue follows the ball ahead.
  //
  // RPM added to the table's for a volley, with the hood steepened for it (ShotTableRpm). Boost doesn't widen the
  // scoring room: modelled at 32 cells it is 75-125 RPM either side at +0, 25 above at +150, 75 either side at +200
  // to +300. So it stays 0 unless Volley Tuning shows the burst needs the headroom
  public static double VOLLEY_BOOST_RPM = 0;
  // How much the hood follows the wheels' predicted speed off the volley target (0 = fixed hood for the burst). Within
  // the band below the steepening is under 1.5 deg, inside HOOD_TOLERANCE_DEG, and the reading lags 100-250 RPM at
  // full-power recovery, so following would chase noise and move the hood between balls
  public static double VOLLEY_HOOD_FOLLOW_GAIN = 0;
  // Matched band: each ball feeds only with the wheels' mean and their speed RPM_LOOKAHEAD_S ahead this close to
  // the volley target. Below is RT's 100: the hub's speed control sits 50-80 RPM under the target at rest, so 75
  // flickered in and out, and in Volley Tuning balls fed 50-100 under went in as often as balls fed on it (12 Nov)
  public static double VOLLEY_READY_BELOW_RPM = 100;
  public static double VOLLEY_READY_ABOVE_RPM = 75;
  // Backward shots burst too (13 Nov, for the Hive-front wedge). Shot Tuning's 138 standing backward balls dipped the
  // wheels as hooded ones do (median 307 RPM, p90 369 against 307/400 for 225 hooded), so the recovery and the
  // double-ball check carry over. But fed above the table's speed they go long: hood within 1.5 deg of the table,
  // fed -50..0 RPM scored 14 of 19, -100..-50 6 of 12, 0..+50 3 of 8 (3 long), +50..+100 2 of 5 (3 long). So the
  // band's top is 25 for a backward burst; its bottom stays VOLLEY_READY_BELOW_RPM (the wheels sit 50-80 under at
  // rest). No boost and no hood trims: the backward hood follows its own table
  public static boolean VOLLEY_BACKWARD_ON = true;
  public static double VOLLEY_BACKWARD_READY_ABOVE_RPM = 25;
  // Launch angle added for the first ball and for balls 2-4. 0: each ball is now fed like RT's calibrated one-ball
  // shot (full feeder from the held stack, at the matched speed), so the shot table's own angle applies. Volley
  // Tuning's 12 Nov trims (+0.9 / -1.0) came from balls fed mid-way through the recovery ringing below, at 0.6 feed
  public static double VOLLEY_FIRST_BALL_TRIM_DEG = 0;
  public static double VOLLEY_LATER_BALL_TRIM_DEG = 0;
  // Lebot Teleop's RB reads Volley Tuning's CSV for its settings. Off: the CSV's results so far were tuned with the
  // ringing recovery and slow feed, which these constants replace (12 Nov)
  public static boolean VOLLEY_USE_TUNING_CSV = false;
  // Lebot Teleop's RT shoot on the move fires its aimed hooded shots as a volley too, with RB's feed, recovery and
  // matched band; the robot keeps moving and the aim keeps leading it. False: RT one ball at a time as before
  public static boolean VOLLEY_ON_RT = true;
  // Feeder power while a volley ball feeds. Full, as RT's shot: the ball the feeder has moves away from the next
  // one, which the indexer's last roller brings up at VOLLEY_INDEXER_SCALE of it, so the gap opens 3x as fast as
  // the queue closes it. At 0.6 the feed took 0.21-0.24 s a ball, and 0.6/0.3 left the 4th ball stuck (12 Nov)
  public static double VOLLEY_FEED_POWER = 1.0;
  // Indexer power while a ball feeds, as a share of the feeder's. Between balls the indexer stages the rest of the
  // stack at full power against its stopped last roller, so the next ball is always there (it took 2.9 s to come
  // up at 0.3, 12 Nov 211957)
  public static double VOLLEY_INDEXER_SCALE = 0.3;
  // Shortest time from a ball seen leaving to the next feed. The wheels' matched band is the main wait
  public static double VOLLEY_MIN_GAP_S = 0.05;
  // As AIMED_FEED_PULSE_S and AIMED_FEED_PAUSE_S, for a volley
  public static double VOLLEY_FEED_PULSE_S = 0.30;
  public static double VOLLEY_FEED_PAUSE_S = 0.20;
  // Between balls of a volley the feeder runs backward, as before the first ball, so every ball starts from the
  // same place as RT's shot and none creeps into the wheels while they recover (was false: stopped)
  public static boolean VOLLEY_HOLD_BETWEEN_BALLS = true;
  // Hood within this of its target for balls after the first, without the steady wait (it isn't moving)
  public static double VOLLEY_HOOD_TOLERANCE_DEG = 1.0;
  // Recovery after each ball: open loop this far below target, speed control again within the handback. The speed
  // reading lags ~0.1 s, and full power gains ~5500 RPM/s, so the wheels run on ~500 RPM past the handback point.
  // At 200/100 that overshot to +600, dipped below 200 again and rang +-500 RPM for ~0.8 s every ball, which was most
  // of the volley's time, and the swings read as double balls (12 Nov, 211957). Speed control alone recovers a 300-
  // 430 RPM dip to the band in ~0.25 s (9 Nov TeleOp); only a deeper dip gets full power, handed back 450 short
  public static boolean VOLLEY_FAST_RECOVERY = true;
  public static double VOLLEY_OPEN_LOOP_BELOW_RPM = 600;
  public static double VOLLEY_HANDBACK_RPM = 450;
  // A ball's dip deeper than this (peak before it to the trough after) is taken as two balls leaving together. A
  // single ball dropped 300-430 RPM under speed control (5 Nov)
  public static double VOLLEY_DOUBLE_DIP_RPM = 600;

  // ---- Driver ----------------------------------------------------------
  // Lebot Teleop: the intake only spins while LT is held (13 Nov), so it isn't pushing at 5-6 A all match. B's eject
  // still runs it backward. The tuning tools keep it running. False: always on, as before
  public static boolean INTAKE_ONLY_ON_LT = true;
  public static double RPM_TRIM_STEP = 50; // Per D-pad press while aiming
  public static double AIM_START_RPM = 2500; // Held while aiming until the first valid shot. Middle of the table
}
