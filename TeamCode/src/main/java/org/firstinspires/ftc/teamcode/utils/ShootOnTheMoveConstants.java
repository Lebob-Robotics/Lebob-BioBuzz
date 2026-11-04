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

  public static double FEED_DELAY_S = 0.10; // Fire command to ball leaving the shooter. Sean's estimate

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
  public static double CELL_SWITCH_MARGIN_M = 0.3; // ...or sooner if the other is at least this much nearer
  public static double CELL_SMOOTHING = 0.3; // Share of the way the stored Cell moves toward each sighting. 1 is no smoothing
  public static final int POSE_HISTORY_SIZE = 100; // About two seconds of loops

  // ---- Cameras (CAD lens centres: both face backward and 35 deg out, tilted up 41.1 deg) ----
  public static final CameraMount LEFT_CAMERA = new CameraMount(
      -0.1624, 0.0377, 0.2093, Math.PI - Math.toRadians(35), Math.toRadians(41.09));
  // Connected camera is mounted upside down: roll must be undone before pitch and yaw.
  public static final CameraMount RIGHT_CAMERA = new CameraMount(
      -0.1457, -0.0625, 0.2093, -(Math.PI - Math.toRadians(35)), Math.toRadians(41.09), Math.PI);

  // ---- Field frame -----------------------------------------------------
  // The odometry heading is in the field frame (0 along +x, anticlockwise positive), so the camera can
  // use the tags' field positions. The drive stays driver relative: heading minus the forward below.
  public static double ALLIANCE_FORWARD_RED_RAD = Math.PI / 2; // Heading the driver calls "forward", standing at their wall. TODO verify
  public static double ALLIANCE_FORWARD_BLUE_RAD = -Math.PI / 2; // TODO verify

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

  public static double Z_TOL_M = 0.10; // A tag's measured height must be this close to the up or down cluster height
  public static int TIP_CONFIRM_FRAMES = 3; // Frames in a row voting for a flip before a Hive's up Cell changes

  public static double MIN_DECISION_MARGIN = 20; // Placeholder, tune
  public static double MAX_TAG_RANGE_M = 3.0;
  public static double MAX_SPEED_FOR_VISION_MPS = 1.0; // Faster than this the frame's pose is too uncertain
  public static double MAX_OMEGA_FOR_VISION_RAD_S = 1.0;
  public static double MAX_FRAME_AGE_S = 0.3;
  public static double MAX_SPREAD_M = 0.10; // Most any tag's correction may differ from the median
  public static double MAX_JUMP_M = 0.5; // Largest correction once the pose is trusted. The first fix may be any size
  public static double FIELD_HALF_SIZE_M = 1.83;
  public static double ROBOT_HALF_SIZE_M = 0.15;
  public static double CORRECTION_GAIN = 0.5; // Share of each correction applied. The first fix applies all of it

  // ---- Shot checks -----------------------------------------------------
  public static double MAX_TURNING_DEG = 30; // Above this, the ball's sideways error grows too fast
  /**
   * Neighbouring table cells are only blended when their hood angles are this close. Further apart
   * they are different kinds of shot, and a blend may miss, so the nearest cell is used.
   */
  public static double MAX_HOOD_DIFFERENCE_TO_BLEND_DEG = 4;

  // ---- Fire check (match the tolerances in tools/shots/inputs.json) ----
  public static double RPM_TOLERANCE = 75; // Per flywheel. The hub's speed reading steps by about 60 RPM
  public static double HOOD_TOLERANCE_DEG = 1.0;
  public static double MAX_HEADING_TOLERANCE_DEG = 5; // Otherwise it grows large at short range

  // ---- Backward shot (tools/shots/backwards-calibration.json) ---------
  // Out of the robot's front, the other way to the cameras: hood on its lower stop, so the ball leaves
  // over the top. Taped from the flywheel axle to the near lip with the robot still, so there is no
  // speed axis: it is only taken while the robot is nearly stopped.
  public static boolean BACKWARD_SHOTS_ON = true;
  public static double BACKWARD_DIRECTION_RAD = 0; // Relative to robot forward
  public static double BACKWARD_LAUNCH_DEG = 99; // Hood on its lower stop (HoodSubsystem.ZERO_ANGLE_DEG)
  public static final double[] BACKWARD_DIST_M = {0.95, 1.3, 1.8, 2.3};
  public static final double[] BACKWARD_RPM = {3400, 3500, 3700, 4200};
  public static double BACKWARD_MAX_SPEED_MPS = 0.3; // Toward or sideways

  // ---- Driver ----------------------------------------------------------
  public static double RPM_TRIM_STEP = 50; // Per D-pad press while aiming
  public static double AIM_START_RPM = 2500; // Held while aiming until the first valid shot. Middle of the table
}
