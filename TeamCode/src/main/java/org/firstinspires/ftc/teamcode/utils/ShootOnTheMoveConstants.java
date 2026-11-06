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
  public static double ACCEL_WINDOW_S = 0.1;
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
  // 0.74-0.85 m scored 3 of 14 at 3370-3470 rpm and 94.7-97.7 degrees
  public static final double[] BACKWARD_DIST_M = {0.40, 0.60, 0.95, 1.3, 1.8, 2.3};
  public static final double[] BACKWARD_RPM = {3200, 3270, 3400, 3500, 3700, 4200};
  public static final double[] BACKWARD_HOOD_DEG = {90, 94, 99, 99, 99, 99};
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

  // Added to every hooded shot from the table. The standing grid (G0-G9, 6 Nov) at -75 hit no top lips, against 7 of
  // 42 balls long at the table's RPM; short misses stayed at 3 and bounce-outs at 6 (21/30 in, against 26/42)
  public static double HOODED_RPM_OFFSET = -75;

  // ---- Driver ----------------------------------------------------------
  public static double RPM_TRIM_STEP = 50; // Per D-pad press while aiming
  public static double AIM_START_RPM = 2500; // Held while aiming until the first valid shot. Middle of the table
}
