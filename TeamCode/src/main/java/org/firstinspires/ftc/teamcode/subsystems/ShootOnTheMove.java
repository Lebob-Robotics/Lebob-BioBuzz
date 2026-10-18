package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.util.ShootOnTheMoveUtils.compensate;
import static org.firstinspires.ftc.teamcode.util.ShootOnTheMoveUtils.evalPoly;
import static org.firstinspires.ftc.teamcode.util.ShootOnTheMoveUtils.interpolate;
import static org.firstinspires.ftc.teamcode.util.ShootOnTheMoveUtils.median;
import static org.firstinspires.ftc.teamcode.util.ShootOnTheMoveUtils.rotate;
import static org.firstinspires.ftc.teamcode.util.ShootOnTheMoveUtils.wrapAngle;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;

import org.firstinspires.ftc.teamcode.util.ShootOnTheMoveUtils.Compensation;
import org.firstinspires.ftc.teamcode.util.ShootOnTheMoveUtils.Pose;
import org.firstinspires.ftc.teamcode.util.ShootOnTheMoveUtils.PoseHistory;

import java.util.ArrayList;
import java.util.List;

/**
 * Shoot-on-the-move solver and vision pose correction. Pure maths with no hardware, so it can be
 * unit tested on the JVM. Config-free helpers live in
 * {@link org.firstinspires.ftc.teamcode.util.ShootOnTheMoveUtils}.
 *
 * <p>Units are SI (m, m/s, rad, s); hood angles are degrees and named *Deg. Field frame is FTC
 * standard: origin at the centre, heading CCW from +x. Robot frame: +x forward, +y left.
 *
 * <p>The polynomials (tools/shot_sim.py) give launch angle, exit speed and time of flight for a
 * robot moving along the line to the goal; {@code ShootOnTheMoveUtils.compensate} adds the
 * sideways part.
 */
@Configurable
public final class ShootOnTheMove {
  private ShootOnTheMove() {}

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
  /** Exit speeds (m/s, ascending) for the optional RPM correction. Empty = none. */
  public static double[] RPM_CORRECTION_SPEEDS_MPS = {};
  /** RPM multiplier at each speed; linear between, held at the ends. */
  public static double[] RPM_CORRECTION_FACTORS = {};

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
  /** Capacity passed to {@link PoseHistory} when it is built. */
  public static int POSE_HISTORY_SIZE = 100;

  // ---- Types -----------------------------------------------------------

  /** A scoring window. */
  public static final class Goal {
    /** Midpoint of the window's bottom lip. */
    public final double x;
    public final double y;
    /** Direction the window faces, toward the shooter. */
    public final double normalRad;

    public Goal(double x, double y, double normalRad) {
      this.x = x;
      this.y = y;
      this.normalRad = normalRad;
    }
  }

  /** Why a solution is or is not valid. */
  public enum Reason {
    OK,
    /** Distance is off the polynomial grid. */
    OUT_OF_RANGE,
    /** Radial speed is outside the valid band at this distance. */
    OUT_OF_DOMAIN,
    /** Launch angle is outside the hood's travel. */
    HOOD_LIMIT,
    /** Shot is too far off the window normal. */
    BAD_APPROACH
  }

  /** Result of one solve. On failure, fields not yet computed are NaN (kept for telemetry). */
  public static final class Solution {
    public final boolean valid;
    public final Reason reason;
    /** Pose after the mechanism delay. */
    public final Pose predictedPose;
    public final double shooterX;
    public final double shooterY;
    public final double distanceM;
    /** Positive = closing on the goal. */
    public final double radialVelMps;
    public final double tangentialVelMps;
    /** Shooter to goal, field frame. */
    public final double bearingRad;
    /** Polynomial output, before compensation. */
    public final double rawLaunchDeg;
    public final double rawExitSpeedMps;
    /** After compensation. */
    public final double launchDeg;
    public final double exitSpeedMps;
    /** Robot heading to command, wrapped. */
    public final double headingTargetRad;
    public final double hoodCommandDeg;
    public final double targetRpm;
    public final double tofS;
    /** Angle between the shot and the window normal. */
    public final double approachAngleRad;

    Solution(
        Reason reason,
        Pose predictedPose,
        double shooterX,
        double shooterY,
        double distanceM,
        double radialVelMps,
        double tangentialVelMps,
        double bearingRad,
        double rawLaunchDeg,
        double rawExitSpeedMps,
        double launchDeg,
        double exitSpeedMps,
        double headingTargetRad,
        double hoodCommandDeg,
        double targetRpm,
        double tofS,
        double approachAngleRad) {
      this.valid = reason == Reason.OK;
      this.reason = reason;
      this.predictedPose = predictedPose;
      this.shooterX = shooterX;
      this.shooterY = shooterY;
      this.distanceM = distanceM;
      this.radialVelMps = radialVelMps;
      this.tangentialVelMps = tangentialVelMps;
      this.bearingRad = bearingRad;
      this.rawLaunchDeg = rawLaunchDeg;
      this.rawExitSpeedMps = rawExitSpeedMps;
      this.launchDeg = launchDeg;
      this.exitSpeedMps = exitSpeedMps;
      this.headingTargetRad = headingTargetRad;
      this.hoodCommandDeg = hoodCommandDeg;
      this.targetRpm = targetRpm;
      this.tofS = tofS;
      this.approachAngleRad = approachAngleRad;
    }
  }

  /** Fire gate result, per check. */
  public static final class GateResult {
    public final boolean valid;
    public final boolean rpmOk;
    public final boolean hoodOk;
    public final boolean headingOk;
    public final boolean turnRateOk;
    /** All checks pass. */
    public final boolean allowed;
    /** Heading tolerance used. */
    public final double headingToleranceRad;

    GateResult(
        boolean valid,
        boolean rpmOk,
        boolean hoodOk,
        boolean headingOk,
        boolean turnRateOk,
        double headingToleranceRad) {
      this.valid = valid;
      this.rpmOk = rpmOk;
      this.hoodOk = hoodOk;
      this.headingOk = headingOk;
      this.turnRateOk = turnRateOk;
      this.allowed = valid && rpmOk && hoodOk && headingOk && turnRateOk;
      this.headingToleranceRad = headingToleranceRad;
    }
  }

  // ---- Field -----------------------------------------------------------
  // Goals from field CAD v26-27.2. Each Hive has an AUDIENCE (+x) and a SCORING (-x) Cell; exactly
  // one is up at a time.

  public enum Alliance { RED, BLUE }

  /** One end of a Hive. */
  public enum Cell { AUDIENCE, SCORING }

  public static final Goal RED_AUDIENCE_GOAL = new Goal(0.4894, -0.3239, 0.0);
  public static final Goal RED_SCORING_GOAL = new Goal(-0.4894, -0.3239, Math.PI);
  public static final Goal BLUE_AUDIENCE_GOAL = new Goal(0.4894, 0.3239, 0.0);
  public static final Goal BLUE_SCORING_GOAL = new Goal(-0.4894, 0.3239, Math.PI);

  /** Up Cell of each Hive. Starts red AUDIENCE, blue SCORING; vision flips them. */
  @IgnoreConfigurable public static Cell upCellRed = Cell.AUDIENCE;
  @IgnoreConfigurable public static Cell upCellBlue = Cell.SCORING;

  /** Practice only: trust DEFAULT_START_POSE_* when there is no auto handoff. */
  public static boolean USE_DEFAULT_START_POSE = false;

  /** Start pose {x, y, heading} without an auto handoff. */
  public static double[] DEFAULT_START_POSE_RED = {0.0, -1.0, Math.PI / 2}; // TODO placeholder
  public static double[] DEFAULT_START_POSE_BLUE = {0.0, 1.0, -Math.PI / 2}; // TODO placeholder

  /** Heading the driver calls "forward", standing at their wall. */
  public static double ALLIANCE_FORWARD_RED_RAD = Math.PI / 2; // TODO verify
  public static double ALLIANCE_FORWARD_BLUE_RAD = -Math.PI / 2; // TODO verify

  /** Pose left by auto for TeleOp to apply on start. Null = none. */
  @IgnoreConfigurable public static Pose handoffPose = null;
  /** Pose is trusted: set by a handoff, a default start pose or a vision bootstrap. */
  @IgnoreConfigurable public static boolean poseValid = false;

  /** Flywheel RPM held in AIM before the first valid solution. */
  public static double IDLE_AIM_RPM = 2500; // TODO placeholder

  /** The up Cell's goal on that alliance's Hive. */
  public static Goal currentGoal(Alliance alliance) {
    if (alliance == Alliance.RED) {
      return upCellRed == Cell.AUDIENCE ? RED_AUDIENCE_GOAL : RED_SCORING_GOAL;
    }
    return upCellBlue == Cell.AUDIENCE ? BLUE_AUDIENCE_GOAL : BLUE_SCORING_GOAL;
  }

  public static double allianceForwardRad(Alliance alliance) {
    return alliance == Alliance.RED ? ALLIANCE_FORWARD_RED_RAD : ALLIANCE_FORWARD_BLUE_RAD;
  }

  // ---- Solver ----------------------------------------------------------

  /**
   * True if (d, vr) is inside the band the polynomials were fitted on, interpolating band edges
   * between grid distances. False for NaN input, d off the grid, or a NaN edge.
   */
  public static boolean inDomain(double d, double vr) {
    if (Double.isNaN(d) || Double.isNaN(vr)) {
      return false;
    }
    double[] g = D_GRID_M;
    if (g.length == 0 || d < g[0] || d > g[g.length - 1]) {
      return false;
    }
    double lo = interpolate(g, VR_MIN_MPS, d);
    double hi = interpolate(g, VR_MAX_MPS, d);
    return vr >= lo && vr <= hi; // false for NaN edges
  }

  /** Flywheel RPM for an exit speed: linear model times the optional correction. */
  public static double rpmForSpeed(double exitSpeedMps) {
    double factor = interpolate(RPM_CORRECTION_SPEEDS_MPS, RPM_CORRECTION_FACTORS, exitSpeedMps);
    return exitSpeedMps * RPM_PER_MPS * (Double.isNaN(factor) ? 1.0 : factor);
  }

  /**
   * Solves one shot. The pose is advanced by the mechanism delay, then solved twice: once at the
   * predicted heading to get a heading target, then again at that target, since that is where the
   * robot will point when it fires.
   *
   * @param pose measured pose
   * @param vx field-frame x velocity
   * @param vy field-frame y velocity
   * @param omega turn rate
   */
  public static Solution solve(Pose pose, double vx, double vy, double omega, Goal goal) {
    Pose pred =
        new Pose(
            pose.x + vx * MECHANISM_DELAY_S,
            pose.y + vy * MECHANISM_DELAY_S,
            pose.heading + omega * MECHANISM_DELAY_S);
    Solution first = solvePass(pred, pred.heading, vx, vy, goal);
    double h = Double.isNaN(first.headingTargetRad) ? pred.heading : first.headingTargetRad;
    return solvePass(pred, h, vx, vy, goal);
  }

  private static Solution solvePass(Pose pred, double heading, double vx, double vy, Goal goal) {
    double[] off = rotate(heading, SHOOTER_OFFSET_FORWARD_M, SHOOTER_OFFSET_LEFT_M);
    double sx = pred.x + off[0];
    double sy = pred.y + off[1];
    double dx = goal.x - sx;
    double dy = goal.y - sy;
    double d = Math.hypot(dx, dy);
    double bearing = Math.atan2(dy, dx);
    double ux = d > 1e-9 ? dx / d : 1.0;
    double uy = d > 1e-9 ? dy / d : 0.0;
    double vr = vx * ux + vy * uy;
    double vt = vx * -uy + vy * ux;

    // Angle between (shooter - goal) and the window normal.
    double cosA = -(ux * Math.cos(goal.normalRad) + uy * Math.sin(goal.normalRad));
    double approach = Math.acos(Math.max(-1.0, Math.min(1.0, cosA)));
    boolean approachOk = approach <= Math.toRadians(MAX_APPROACH_ANGLE_DEG);

    double nan = Double.NaN;
    double rawTh = nan;
    double rawS = nan;
    double tof = nan;
    double th = nan;
    double s = nan;
    double headingTarget = nan;
    double hood = nan;
    double rpm = nan;
    Reason reason = approachOk ? Reason.OK : Reason.BAD_APPROACH;

    boolean onGrid =
        D_GRID_M.length > 0 && d >= D_GRID_M[0] && d <= D_GRID_M[D_GRID_M.length - 1];
    if (inDomain(d, vr)) {
      rawTh = evalPoly(HOOD_ANGLE_COEFFS, POLY_DEGREE, d, vr);
      rawS = evalPoly(EXIT_SPEED_COEFFS, POLY_DEGREE, d, vr);
      tof = evalPoly(TOF_COEFFS, POLY_DEGREE, d, vr);
      Compensation c = compensate(rawTh, rawS, vt);
      th = c.launchDeg;
      s = c.speedMps;
      headingTarget = wrapAngle(bearing + c.headingOffsetRad - SHOOTER_MOUNT_YAW_RAD);
      hood = th - HOOD_ANGLE_OFFSET_DEG;
      rpm = rpmForSpeed(s);
      if (reason == Reason.OK && !(th >= LAUNCH_MIN_DEG && th <= LAUNCH_MAX_DEG)) {
        reason = Reason.HOOD_LIMIT;
      }
    } else if (reason == Reason.OK) {
      reason = onGrid ? Reason.OUT_OF_DOMAIN : Reason.OUT_OF_RANGE;
    }
    return new Solution(
        reason, pred, sx, sy, d, vr, vt, bearing, rawTh, rawS, th, s,
        headingTarget, hood, rpm, tof, approach);
  }

  /**
   * Whether it is safe to fire. Heading tolerance is the angle that keeps the ball inside the
   * window at this distance, capped and never negative.
   */
  public static GateResult gate(
      Solution s, double measuredRpm, double hoodAngleDeg, double headingRad, double turnRateRadS) {
    double tol = HEADING_TOLERANCE_MAX_RAD;
    if (s.distanceM > 0) {
      // The window looks narrower by cos(approach) when seen at an angle.
      double halfWidth = Double.isNaN(s.approachAngleRad)
          ? GOAL_HALF_WIDTH_M : GOAL_HALF_WIDTH_M * Math.cos(s.approachAngleRad);
      double room = halfWidth - BALL_RADIUS_M - AIM_MARGIN_M;
      tol = Math.min(tol, Math.atan(room / s.distanceM));
    }
    tol = Math.max(0.0, tol);
    boolean rpmOk = Math.abs(measuredRpm - s.targetRpm) <= RPM_TOLERANCE;
    boolean hoodOk = Math.abs(hoodAngleDeg - s.hoodCommandDeg) <= HOOD_TOLERANCE_DEG;
    boolean headingOk = Math.abs(wrapAngle(headingRad - s.headingTargetRad)) <= tol;
    boolean turnOk = Math.abs(turnRateRadS) <= MAX_FIRE_TURN_RATE_RAD_S;
    return new GateResult(s.valid, rpmOk, hoodOk, headingOk, turnOk, tol);
  }

  // ---- Vision ----------------------------------------------------------
  // Vision corrects x/y only; heading always comes from the Pinpoint. Only each detection's ftcPose
  // x/y/z is used. The SDK's robotPose is not: the tag library's field poses are placeholders and
  // the tags move when a Hive tips. Robot frame: +x forward, +y left, +z up.

  public enum Camera { LEFT, RIGHT }

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
  private static final double[] TAG_Y_IN = {
    -6.25, -10.0, -15.5, -19.25,
    -19.25, -15.5, -10.0, -6.25,
    6.25, 10.0, 15.5, 19.25,
    19.25, 15.5, 10.0, 6.25
  };
  private static final double IN_TO_M = 0.0254;

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

  /** One AprilTag sighting in ftcPose axes (x right, y along the lens, z up). */
  public static final class TagObservation {
    public final int id;
    public final Camera camera;
    public final double x;
    public final double y;
    public final double z;
    public final double decisionMargin;
    /** Frame capture time, System.nanoTime() base. */
    public final long frameNanos;

    public TagObservation(
        int id, Camera camera, double x, double y, double z,
        double decisionMargin, long frameNanos) {
      this.id = id;
      this.camera = camera;
      this.x = x;
      this.y = y;
      this.z = z;
      this.decisionMargin = decisionMargin;
      this.frameNanos = frameNanos;
    }

    /** Distance from the camera to the tag. */
    public double rangeM() {
      return Math.sqrt(x * x + y * y + z * z);
    }
  }

  /** Why a vision update was or was not applied. */
  public enum VisionReason {
    ACCEPTED,
    NO_OBSERVATIONS,
    TOO_FAST,
    TOO_FAST_TURNING,
    /** Every observation was dropped; see {@link VisionResult#firstTagReject}. */
    NO_VALID_TAGS,
    SPREAD,
    OUT_OF_FIELD,
    JUMP
  }

  /** Why one observation was dropped. */
  public enum TagReject {
    NONE,
    NO_POSE,
    NOT_HIVE_TAG,
    LOW_MARGIN,
    TOO_FAR,
    BAD_Z,
    FRAME_TOO_OLD,
    NO_HISTORY
  }

  /** Result of {@link VisionCorrector#process}. */
  public static final class VisionResult {
    public final boolean accepted;
    public final VisionReason reason;
    /** Why the first dropped observation was dropped; NONE if none were. */
    public final TagReject firstTagReject;
    /** Median correction (vision minus odometry at frame time). Zero unless accepted. */
    public final double dx;
    public final double dy;
    /** Fraction of the correction applied. Zero unless accepted. */
    public final double gain;
    /** Current pose + gain * correction. Null unless accepted. */
    public final Pose newPose;
    public final int tagsUsed;
    public final int tagsRejected;
    /** Max distance of a used tag's correction from the median. NaN if no tags were used. */
    public final double spread;
    /** This update flipped that Hive's up Cell. */
    public final boolean redFlipped;
    public final boolean blueFlipped;
    /** Each used tag's id and correction. */
    public final int[] usedIds;
    public final double[] usedDx;
    public final double[] usedDy;

    VisionResult(
        VisionReason reason, TagReject firstTagReject, double dx, double dy, double gain,
        Pose newPose, int tagsUsed, int tagsRejected, double spread, boolean redFlipped,
        boolean blueFlipped, int[] usedIds, double[] usedDx, double[] usedDy) {
      this.accepted = reason == VisionReason.ACCEPTED;
      this.reason = reason;
      this.firstTagReject = firstTagReject;
      this.dx = dx;
      this.dy = dy;
      this.gain = gain;
      this.newPose = newPose;
      this.tagsUsed = tagsUsed;
      this.tagsRejected = tagsRejected;
      this.spread = spread;
      this.redFlipped = redFlipped;
      this.blueFlipped = blueFlipped;
      this.usedIds = usedIds;
      this.usedDx = usedDx;
      this.usedDy = usedDy;
    }
  }

  /** Alliance of a tag, or null for an unknown ID. */
  public static Alliance tagAlliance(int id) {
    if (id >= 30 && id <= 37) {
      return Alliance.RED;
    }
    if (id >= 38 && id <= 45) {
      return Alliance.BLUE;
    }
    return null;
  }

  /** Cell a tag sits on, or null for an unknown ID. */
  public static Cell tagCell(int id) {
    if ((id >= 30 && id <= 33) || (id >= 42 && id <= 45)) {
      return Cell.SCORING;
    }
    if (id >= 34 && id <= 41) {
      return Cell.AUDIENCE;
    }
    return null;
  }

  public static Cell otherCell(Cell c) {
    return c == Cell.AUDIENCE ? Cell.SCORING : Cell.AUDIENCE;
  }

  /** Current up Cell of an alliance's Hive. */
  public static Cell upCell(Alliance a) {
    return a == Alliance.RED ? upCellRed : upCellBlue;
  }

  /**
   * Field position of a tag centre.
   *
   * @param upCellOfThatHive up Cell of the tag's own Hive
   * @return {x, y, z}, or null for an unknown ID
   */
  public static double[] tagFieldPosition(int id, Cell upCellOfThatHive) {
    Alliance a = tagAlliance(id);
    Cell cell = tagCell(id);
    if (a == null || cell == null) {
      return null;
    }
    double y = TAG_Y_IN[id - 30] * IN_TO_M;
    boolean up = cell == upCellOfThatHive;
    double ax = up ? CLUSTER_UP_X_M : CLUSTER_DOWN_X_M;
    double z = up ? CLUSTER_UP_Z_M : CLUSTER_DOWN_Z_M;
    return new double[] {cell == Cell.AUDIENCE ? ax : -ax, y, z};
  }

  /**
   * Tag centre in the robot frame from an ftcPose x/y/z.
   *
   * <p>For camera yaw psi and pitch phi, the camera axes in the robot frame are
   * forward f = (cos phi cos psi, cos phi sin psi, sin phi), right r = (sin psi, -cos psi, 0) and
   * up u = r x f = (-cos psi sin phi, -sin psi sin phi, cos phi). The tag is at
   * camPos + x r + y f + z u.
   *
   * @return {forward, left, up}
   */
  public static double[] tagInRobotFrame(Camera cam, double x, double y, double z) {
    double psi = cam == Camera.LEFT ? CAM_YAW_LEFT_RAD : CAM_YAW_RIGHT_RAD;
    double[] p = cam == Camera.LEFT ? CAM_POS_LEFT_M : CAM_POS_RIGHT_M;
    double cp = Math.cos(CAM_PITCH_RAD);
    double sp = Math.sin(CAM_PITCH_RAD);
    double cy = Math.cos(psi);
    double sy = Math.sin(psi);
    return new double[] {
      p[0] + x * sy + y * cp * cy - z * cy * sp,
      p[1] - x * cy + y * cp * sy - z * sy * sp,
      p[2] + y * sp + z * cp
    };
  }

  /**
   * Turns tag observations into x/y corrections and tracks Hive tip state with hysteresis. Keep
   * one per OpMode (it holds flip votes). Writes {@link #upCellRed}, {@link #upCellBlue} and
   * {@link #poseValid}.
   */
  public static final class VisionCorrector {
    // Consecutive frames voting against the current up Cell: [0] red, [1] blue.
    private final int[] flipVotes = new int[2];

    /** Clears partial flip votes. */
    public void reset() {
      flipVotes[0] = 0;
      flipVotes[1] = 0;
    }

    /**
     * Processes one loop's new observations. Tip state uses only tag heights, so it updates
     * whenever the robot is slow enough, even if the position correction is rejected.
     *
     * @param obs new observations only; never reuse a frame
     * @param history timestamped odometry poses, with past vision shifts applied
     * @param now current odometry pose
     * @param vx field x velocity
     * @param vy field y velocity
     * @param omega turn rate
     * @param poseValid whether the pose is already trusted; false = bootstrap
     * @param nowNanos loop time, same base as frame times
     */
    public VisionResult process(
        List<TagObservation> obs, PoseHistory history, Pose now,
        double vx, double vy, double omega, boolean poseValid, long nowNanos) {
      if (obs == null || obs.isEmpty()) {
        return fail(VisionReason.NO_OBSERVATIONS, TagReject.NONE, 0, false, false);
      }
      if (Math.hypot(vx, vy) > MAX_SPEED_FOR_VISION_MPS) {
        return fail(VisionReason.TOO_FAST, TagReject.NONE, 0, false, false);
      }
      if (Math.abs(omega) > MAX_OMEGA_FOR_VISION_RAD_S) {
        return fail(VisionReason.TOO_FAST_TURNING, TagReject.NONE, 0, false, false);
      }

      // votes[hive][cell]: observations with a plausible z saying that Cell is up.
      int[][] votes = new int[2][2];
      List<Double> dxs = new ArrayList<>();
      List<Double> dys = new ArrayList<>();
      List<Integer> ids = new ArrayList<>();
      TagReject firstReject = TagReject.NONE;
      int rejected = 0;

      for (TagObservation o : obs) {
        TagReject r = TagReject.NONE;
        Cell measuredUp = null;
        double[] robotPt = null;
        double[] tag = null;
        Pose atFrame = null;

        if (Double.isNaN(o.x) || Double.isNaN(o.y) || Double.isNaN(o.z)) {
          r = TagReject.NO_POSE;
        } else if (tagAlliance(o.id) == null) {
          r = TagReject.NOT_HIVE_TAG;
        } else if (!(o.decisionMargin >= MIN_DECISION_MARGIN)) {
          r = TagReject.LOW_MARGIN;
        } else if (o.rangeM() > MAX_TAG_RANGE_M) {
          r = TagReject.TOO_FAR;
        } else {
          robotPt = tagInRobotFrame(o.camera, o.x, o.y, o.z);
          double zMeasured = robotPt[2]; // robot is on the floor, so robot z = field z
          Cell cell = tagCell(o.id);
          if (Math.abs(zMeasured - CLUSTER_UP_Z_M) < Z_TOL_M) {
            measuredUp = cell;
          } else if (Math.abs(zMeasured - CLUSTER_DOWN_Z_M) < Z_TOL_M) {
            measuredUp = otherCell(cell);
          } else {
            r = TagReject.BAD_Z;
          }
        }

        if (r == TagReject.NONE) {
          int hive = tagAlliance(o.id) == Alliance.RED ? 0 : 1;
          votes[hive][measuredUp.ordinal()]++;
          // Locate the tag using the tip state this observation measured.
          tag = tagFieldPosition(o.id, measuredUp);
          double ageS = (nowNanos - o.frameNanos) * 1e-9;
          if (ageS > MAX_FRAME_AGE_S) {
            r = TagReject.FRAME_TOO_OLD;
          } else {
            atFrame = history == null ? null : history.sample(o.frameNanos);
            if (atFrame == null) {
              r = TagReject.NO_HISTORY;
            }
          }
        }

        if (r != TagReject.NONE) {
          rejected++;
          if (firstReject == TagReject.NONE) {
            firstReject = r;
          }
          continue;
        }

        // Robot position = tag position - tag offset rotated into the field frame.
        double[] off = rotate(atFrame.heading, robotPt[0], robotPt[1]);
        dxs.add(tag[0] - off[0] - atFrame.x);
        dys.add(tag[1] - off[1] - atFrame.y);
        ids.add(o.id);
      }

      boolean redFlipped = applyVotes(0, votes[0], Alliance.RED);
      boolean blueFlipped = applyVotes(1, votes[1], Alliance.BLUE);

      int n = dxs.size();
      if (n == 0) {
        return fail(VisionReason.NO_VALID_TAGS, firstReject, rejected, redFlipped, blueFlipped);
      }
      double[] ax = new double[n];
      double[] ay = new double[n];
      int[] usedIds = new int[n];
      for (int i = 0; i < n; i++) {
        ax[i] = dxs.get(i);
        ay[i] = dys.get(i);
        usedIds[i] = ids.get(i);
      }
      double mx = median(ax);
      double my = median(ay);
      double spread = 0;
      for (int i = 0; i < n; i++) {
        spread = Math.max(spread, Math.hypot(ax[i] - mx, ay[i] - my));
      }

      // Bounds check where vision says the robot is now.
      double limit = FIELD_HALF_SIZE_M - ROBOT_HALF_SIZE_M;
      VisionReason reason = VisionReason.ACCEPTED;
      if (spread > MAX_SPREAD_M) {
        reason = VisionReason.SPREAD;
      } else if (Math.abs(now.x + mx) > limit || Math.abs(now.y + my) > limit) {
        reason = VisionReason.OUT_OF_FIELD;
      } else if (poseValid && Math.hypot(mx, my) > MAX_JUMP_M) {
        reason = VisionReason.JUMP;
      }
      if (reason != VisionReason.ACCEPTED) {
        return new VisionResult(reason, firstReject, 0, 0, 0, null, n, rejected, spread,
            redFlipped, blueFlipped, usedIds, ax, ay);
      }

      double k = poseValid ? CORRECTION_GAIN : 1.0;
      ShootOnTheMove.poseValid = true;
      Pose newPose = new Pose(now.x + k * mx, now.y + k * my, now.heading);
      return new VisionResult(reason, firstReject, mx, my, k, newPose, n, rejected, spread,
          redFlipped, blueFlipped, usedIds, ax, ay);
    }

    private static VisionResult fail(
        VisionReason reason, TagReject firstReject, int rejected, boolean red, boolean blue) {
      return new VisionResult(reason, firstReject, 0, 0, 0, null, 0, rejected, Double.NaN,
          red, blue, new int[0], new double[0], new double[0]);
    }

    /**
     * Applies one frame's votes for one Hive; no votes or a tie changes nothing. Returns true if
     * the up Cell flipped.
     */
    private boolean applyVotes(int hive, int[] v, Alliance alliance) {
      int aud = v[Cell.AUDIENCE.ordinal()];
      int sco = v[Cell.SCORING.ordinal()];
      if (aud == sco) {
        return false;
      }
      Cell winner = aud > sco ? Cell.AUDIENCE : Cell.SCORING;
      if (winner == upCell(alliance)) {
        flipVotes[hive] = 0;
        return false;
      }
      if (++flipVotes[hive] < Math.max(1, TIP_CONFIRM_FRAMES)) {
        return false;
      }
      flipVotes[hive] = 0;
      if (alliance == Alliance.RED) {
        upCellRed = winner;
      } else {
        upCellBlue = winner;
      }
      return true;
    }
  }
}
