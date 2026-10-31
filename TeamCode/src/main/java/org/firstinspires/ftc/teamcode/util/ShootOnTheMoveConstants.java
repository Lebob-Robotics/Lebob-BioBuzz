package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.subsystems.ShootOnTheMove;
import org.firstinspires.ftc.teamcode.subsystems.ShootOnTheMove.Goal;

/**
 * Tunable constants for {@link ShootOnTheMove}, editable live from Panels. Units are SI (m, m/s,
 * rad, s); hood angles are degrees and named *Deg.
 */
@Configurable
public final class ShootOnTheMoveConstants {
  private ShootOnTheMoveConstants() {}

  // ---- Latency ---------------------------------------------------------
  /** Time from "fire" to the ball leaving the flywheel. */
  public static double MECHANISM_DELAY_S = 0.10; // TODO measure

  // ---- Shooter geometry ------------------------------------------------
  /** Shooter firing direction relative to robot forward. PI = out of the back. */
  public static double SHOOTER_MOUNT_YAW_RAD = Math.PI;
  /** Flywheel shaft position from the Pinpoint tracking centre. */
  public static double SHOOTER_OFFSET_FORWARD_M = 0.0612;
  public static double SHOOTER_OFFSET_LEFT_M = -0.0179;
  // TODO: offsets are from CAD, assuming the robot centre is 228.6 mm behind the front edge.

  // ---- Tuning ----------------------------------------------------------
  /** Measured launch angle minus commanded hood angle (deg). */
  public static double HOOD_ANGLE_OFFSET_DEG = 0.0;
  /**
   * Flywheel RPM per m/s of exit speed. Initial value assumes no slip:
   * 1 / (0.01885 m * 2pi / 60).
   */
  public static double RPM_PER_MPS = 506.6; // TODO tune on robot
  /**
   * Exit speeds (m/s, ascending) for the RPM correction. Pre-sized because Panels can't add array
   * entries; edit the values, not the length.
   */
  public static double[] RPM_CORRECTION_SPEEDS_MPS = {3.0, 4.0, 5.0, 6.0, 7.0};
  /** RPM multiplier at each speed; linear between, held at the ends. 1.0 = no correction. */
  public static double[] RPM_CORRECTION_FACTORS = {1.0, 1.0, 1.0, 1.0, 1.0};

  // ---- Hood limits -----------------------------------------------------
  public static double LAUNCH_MIN_DEG = 51;
  public static double LAUNCH_MAX_DEG = 89;

  // ---- Polynomials -----------------------------------------------------
  /** Terms are d^i * vr^j for i in 0..deg, j in 0..deg-i. */
  public static int POLY_DEGREE = 3;
  // From tools/shot_sim.py (2026-10-18): hood 51-89 deg, exit 2-8 m/s, Cd 0.47, no Magnus,
  // 24.9 g / 2.8 in pollen, smoothing on. 99.1% of re-simulated shots score; misses are excluded
  // from the domain below.
  /** Launch angle (deg) from distance and radial speed. */
  public static double[] HOOD_ANGLE_COEFFS = {
    91.956126, 15.1719695, 0.722881618, -0.204563807, -36.9373226,
    -8.28715297, -0.149844501, 12.2149234, 1.23902449, -1.39037354
  };
  /** Exit speed (m/s) from distance and radial speed. */
  public static double[] EXIT_SPEED_COEFFS = {
    5.04002499, -0.15867623, 0.0836796616, 0.00555802958, 0.482610353,
    -0.206795616, -0.0124745612, 0.152266603, 0.0278274074, -0.0227654289
  };
  /** Time of flight (s) from distance and radial speed. */
  public static double[] TOF_COEFFS = {
    0.583766166, 0.0585118575, 0.000245235569, -0.00526082403, -0.189880617,
    -0.0951545887, 0.00407646619, 0.146615862, 0.0139065778, -0.0197028378
  };
  /** Distances (m) at which the valid radial-speed band is defined. */
  public static double[] D_GRID_M = {
    0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2.0,
    2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7, 2.8, 2.9, 3.0, 3.1, 3.2, 3.3, 3.4, 3.5
  };
  /** Min valid radial speed (m/s) per grid distance. NaN = no valid shot. */
  public static double[] VR_MIN_MPS = {
    Double.NaN, -1.5, -1.5, -1.5, -1.5, -1.5, -1.5, -1.5, -1.5, -1.5, -1.5, -1.5, -1.5,
    -1.5, -1.5, -1.5, -1.5, -1.5, -1.5, -1.5, -1.5, -1.5, -1.5, -1.5, -1.4, -1.1, -1.0,
    -1.0, -0.9, -0.6, -0.6
  };
  /** Max valid radial speed (m/s) per grid distance. NaN = no valid shot. */
  public static double[] VR_MAX_MPS = {
    Double.NaN, 1.3, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5,
    1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5
  };

  // ---- Fire gate -------------------------------------------------------
  public static double RPM_TOLERANCE = 75;
  public static double HOOD_TOLERANCE_DEG = 0.75;
  /** Cap on the heading tolerance at any distance. */
  public static double HEADING_TOLERANCE_MAX_RAD = Math.toRadians(3);
  public static double GOAL_HALF_WIDTH_M = 0.254;
  public static double BALL_RADIUS_M = 0.03556;
  /** Clearance kept from the window edge. */
  public static double AIM_MARGIN_M = 0.03;
  public static double MAX_FIRE_TURN_RATE_RAD_S = 1.5;
  /** Max angle between the horizontal shot direction and the window normal. */
  public static double MAX_APPROACH_ANGLE_DEG = 45;

  // ---- Pose history ----------------------------------------------------
  /** Capacity of a PoseHistory, read when one is built. */
  public static int POSE_HISTORY_SIZE = 100;

  // ---- Field -----------------------------------------------------------
  // Goals from field CAD v26-27.2. AUDIENCE Cells are at +x, SCORING at -x.
  public static final Goal RED_AUDIENCE_GOAL = new Goal(0.4894, -0.3239, 0.0);
  public static final Goal RED_SCORING_GOAL = new Goal(-0.4894, -0.3239, Math.PI);
  public static final Goal BLUE_AUDIENCE_GOAL = new Goal(0.4894, 0.3239, 0.0);
  public static final Goal BLUE_SCORING_GOAL = new Goal(-0.4894, 0.3239, Math.PI);

  /** Practice only: trust DEFAULT_START_POSE_* when there is no auto handoff. */
  public static boolean USE_DEFAULT_START_POSE = false;

  /** Start pose {x, y, heading} without an auto handoff. */
  public static double[] DEFAULT_START_POSE_RED = {0.0, -1.0, Math.PI / 2}; // TODO placeholder
  public static double[] DEFAULT_START_POSE_BLUE = {0.0, 1.0, -Math.PI / 2}; // TODO placeholder

  /** Heading the driver calls "forward", standing at their wall. */
  public static double ALLIANCE_FORWARD_RED_RAD = Math.PI / 2; // TODO verify
  public static double ALLIANCE_FORWARD_BLUE_RAD = -Math.PI / 2; // TODO verify

  /** Flywheel RPM held in AIM before the first valid solution. */
  public static double IDLE_AIM_RPM = 2500; // TODO placeholder

  // ---- Vision ----------------------------------------------------------
  /** Both cameras face backward like the shooter, turned out 35 deg and tilted up. */
  public static double CAM_PITCH_RAD = Math.toRadians(48.907);
  /** Camera yaw, CCW from robot forward. */
  public static double CAM_YAW_LEFT_RAD = Math.PI - Math.toRadians(35);
  public static double CAM_YAW_RIGHT_RAD = -(Math.PI - Math.toRadians(35));
  /** Camera position from the robot centre: {forward, left, up}. */
  public static double[] CAM_POS_LEFT_M = {-0.07405, 0.04313, 0.16972};
  public static double[] CAM_POS_RIGHT_M = {-0.07442, -0.04318, 0.17040};
  // TODO: from CAD, assuming the robot centre is 228.6 mm behind the front and 173.2 mm from the
  // left side.
  /** Reference only; not used in the maths. */
  public static double CAM_HFOV_DEG = 80;

  /** Tag cluster centre of the up Cell: |x| and z. */
  public static double CLUSTER_UP_X_M = 0.3329;
  public static double CLUSTER_UP_Z_M = 1.2646;
  /** Tag cluster centre of the down Cell: |x| and z. */
  public static double CLUSTER_DOWN_X_M = 0.2949;
  public static double CLUSTER_DOWN_Z_M = 0.9022;
  // Tag y (in) by ID, four per Cell: RED SCORING 30-33, RED AUDIENCE 34-37, BLUE AUDIENCE 38-41,
  // BLUE SCORING 42-45.
  public static final double[] TAG_Y_IN = {
    -6.25, -10.0, -15.5, -19.25,
    -19.25, -15.5, -10.0, -6.25,
    6.25, 10.0, 15.5, 19.25,
    19.25, 15.5, 10.0, 6.25
  };
  public static final double IN_TO_M = 0.0254;

  // Tip detection.
  /** A tag's measured z must be within this of the up or down cluster height. */
  public static double Z_TOL_M = 0.10;
  /** Consecutive frames voting for a flip before a Hive's up Cell changes. */
  public static int TIP_CONFIRM_FRAMES = 3;

  // Rejection.
  public static double MIN_DECISION_MARGIN = 20; // TODO placeholder, tune
  public static double MAX_TAG_RANGE_M = 3.0;
  public static double MAX_SPEED_FOR_VISION_MPS = 1.0;
  public static double MAX_OMEGA_FOR_VISION_RAD_S = 1.0;
  public static double MAX_FRAME_AGE_S = 0.3;
  /** Max distance of any tag's correction from the median correction. */
  public static double MAX_SPREAD_M = 0.10;
  /** Max correction, except when bootstrapping. */
  public static double MAX_JUMP_M = 0.5;
  public static double FIELD_HALF_SIZE_M = 1.83;
  public static double ROBOT_HALF_SIZE_M = 0.15;
  /** Fraction of the correction applied per update (1 when bootstrapping). */
  public static double CORRECTION_GAIN = 0.5;
}
