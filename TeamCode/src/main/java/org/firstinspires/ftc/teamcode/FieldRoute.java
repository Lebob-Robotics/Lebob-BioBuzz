package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utils.Pose;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * The test spots and the drive between them that Shot Tuning's B uses: field limits, the Hive's footprint, the route
 * round it, and the go-to-point gains. Field metres, robot centre.
 */
final class FieldRoute {
  /**
   * Robot centre limits: 0.25 m inside the perimeter, and out of the Hive's footprint. At 0.15 m a pose that went
   * 0.4 m wrong took the robot off the mat (5 Nov, 081516).
   */
  static final double FIELD_LIMIT_M = 1.35;
  /** Farthest the robot centre can be from the field's middle: 1.83 m to the wall less half the 18 in robot. */
  static final double MAX_REACH_M = 1.60;
  static final double HIVE_HALF_X_M = 0.75;
  static final double HIVE_HALF_Y_M = 0.95;
  static final double PREDICT_S = 0.15; // Reaction before braking starts
  static final double BRAKE_MPS2 = 2.0; // Braking assumed, for the stopping distance
  /** Corners of the drive round the Hive: clear of its footprint, inside the field limit. */
  private static final double ROUTE_X_M = 1.10;
  private static final double ROUTE_Y_M = 1.15;
  private static final double ROUTE_CLEARANCE_M = 0.05; // Added to the Hive footprint for the drive's segments
  static final double ROUTE_WAYPOINT_TOLERANCE_M = 0.15;

  /**
   * Standing shots for the shot model's calibration, {x, y} for a Cell on the -x, -y side (red SCORING's aim point
   * is about -0.44, -0.32), mirrored to the side of the Cell in use: 0.4 to 1.6 m from the lip, head on to 63 degrees
   * off the Cell's facing, which is all of the field this side of the Hive. Distances are about, from the aim point.
   */
  static final double[][] GRID = {
      {-1.00, -0.32}, // 0.40 m, head on
      {-1.22, -0.32}, // 0.62, head on
      {-1.22, -0.75}, // 0.73, 29 degrees off
      {-1.22, 0.10},  // 0.73, 28 the other way
      {-1.00, -1.05}, // 0.76, 53
      {-1.00, 0.45},  // 0.79, 54 the other way
      {-1.22, -1.22}, // 1.03, 49
      {-1.22, 0.55},  // 1.01, 48 the other way
      {-1.22, 0.90},  // 1.29, 57
      {-1.22, 1.22},  // 1.57, 63
  };

  static final double GOTO_STICK = 0.6;
  // Into the corner at GOTO_STICK it was still at 0.6 m/s 0.15 m out, its stop fell 4 cm past the check and the
  // drive stopped short (7 Nov, 080429)
  static final double RELOAD_STICK = 0.4;
  // Only this close to the reload spot: the whole 2.3 m leg at RELOAD_STICK crept at 0.1 m/s (7 Nov, 085204)
  static final double RELOAD_SLOW_M = 0.5;
  static final double RELOAD_MARGIN_M = 0.04; // Past the reload spot the stop check allows, inside MAX_REACH_M
  static final double GOTO_GAIN = 2.0; // Stick per metre off
  static final double GOTO_TOLERANCE_M = 0.07;
  /** Least stick while still off: 0.18 shaped to 0.19 power and stalled 9 cm short. */
  static final double GOTO_MIN_STICK = 0.35;
  // Added per second without progress: 0.35 sideways (0.26 power) sat 11 cm short until skipped (5 Nov, 082021)
  static final double GOTO_STUCK_BOOST_PER_S = 0.3;
  static final double GOTO_TIMEOUT_S = 10;
  // Odometry alone took the robot off the mat: driving to the blue side the camera saw no tags for 20 s while the
  // pose barely moved (5 Nov, 080742)
  static final double GOTO_BLIND_S = 3.0;
  // Back onto the straight leg: aimed only at the end, 2.5 m down the edge drifted 0.21 m out into the fence
  // (5 Nov, 082500)
  static final double CROSS_GAIN = 2.0;
  static final double SPOT_FENCE_SLACK_M = 0.05;
  static final double TAKEOVER_STICK = 0.3;

  private FieldRoute() {}

  /** Where it would stop along one axis, from a speed along it: reaction then steady braking. */
  static double stopDistance(double v) {
    return v * PREDICT_S + v * Math.abs(v) / (2 * BRAKE_MPS2);
  }

  static boolean inBox(double x, double y, double edgeSlack, double hiveSlack) {
    return inBox(x, y, FIELD_LIMIT_M, edgeSlack, hiveSlack);
  }

  static boolean inBox(double x, double y, double limit, double edgeSlack, double hiveSlack) {
    boolean onField = Math.abs(x) <= limit + edgeSlack && Math.abs(y) <= limit + edgeSlack;
    boolean inHive = Math.abs(x) < HIVE_HALF_X_M - hiveSlack && Math.abs(y) < HIVE_HALF_Y_M - hiveSlack;
    return onField && !inHive;
  }

  /**
   * The shortest way from one point to another that keeps clear of the Hive: straight, or round one or two of its
   * corners ({@link #ROUTE_X_M}, {@link #ROUTE_Y_M}). The waypoints, the end last.
   */
  static List<double[]> route(double fromX, double fromY, double toX, double toY) {
    double[][] corners = {{-ROUTE_X_M, -ROUTE_Y_M}, {-ROUTE_X_M, ROUTE_Y_M}, {ROUTE_X_M, -ROUTE_Y_M},
        {ROUTE_X_M, ROUTE_Y_M}};
    double[] end = {toX, toY};
    List<double[]> best = null;
    double bestLength = Double.POSITIVE_INFINITY;
    List<List<double[]>> candidates = new ArrayList<>();
    candidates.add(Arrays.asList(end));
    for (double[] a : corners) {
      candidates.add(Arrays.asList(a, end));
      for (double[] b : corners) {
        if (a != b && (a[0] == b[0] || a[1] == b[1])) { // Along one side of the Hive
          candidates.add(Arrays.asList(a, b, end));
        }
      }
    }
    for (List<double[]> candidate : candidates) {
      double length = 0;
      boolean clear = true;
      double x = fromX;
      double y = fromY;
      for (double[] w : candidate) {
        clear &= segmentClear(x, y, w[0], w[1]);
        length += Math.hypot(w[0] - x, w[1] - y);
        x = w[0];
        y = w[1];
      }
      if (clear && length < bestLength) {
        best = candidate;
        bestLength = length;
      }
    }
    return best != null ? best : Arrays.asList(end);
  }

  /**
   * Field-frame stick toward (x, y) along the leg from (legStartX, legStartY), with a pull back onto the leg's line,
   * and boost added to the least stick.
   */
  static double[] goToStick(Pose pose, double x, double y, double legStartX, double legStartY, double boost) {
    return goToStick(pose, x, y, legStartX, legStartY, boost, GOTO_STICK);
  }

  /** As above, with the stick capped at maxStick. */
  static double[] goToStick(Pose pose, double x, double y, double legStartX, double legStartY, double boost,
      double maxStick) {
    double ex = x - pose.xM;
    double ey = y - pose.yM;
    double dist = Math.hypot(ex, ey);
    double lx = x - legStartX;
    double ly = y - legStartY;
    double legLen = Math.hypot(lx, ly);
    double cx = 0;
    double cy = 0;
    if (legLen > GOTO_TOLERANCE_M) {
      double ux = lx / legLen;
      double uy = ly / legLen;
      double cross = ex * -uy + ey * ux; // Off the leg's line, to its left
      cx = -uy * cross * CROSS_GAIN;
      cy = ux * cross * CROSS_GAIN;
    }
    // A stuck robot's boost lifts a cap below GOTO_STICK: at RELOAD_STICK (0.29 power) it sat still for 5 s at a time
    // on the way to the reload corner and had to be pulled (7 Nov, 082948)
    maxStick = Math.max(maxStick, Math.min(GOTO_STICK, GOTO_MIN_STICK + boost));
    double speed = Math.min(maxStick, Math.max(GOTO_MIN_STICK + boost, GOTO_GAIN * dist));
    double vx = dist > 1e-6 ? ex / dist * speed + cx : 0;
    double vy = dist > 1e-6 ? ey / dist * speed + cy : 0;
    double mag = Math.hypot(vx, vy);
    if (mag > maxStick) {
      vx *= maxStick / mag;
      vy *= maxStick / mag;
    }
    return new double[] {vx, vy};
  }

  /** Where it would stop stays off the fence and the Hive. Not checked until it is inside, so it can drive in. */
  static boolean headedSafely(Pose pose, double[] velocity) {
    return headedSafely(pose, velocity, FIELD_LIMIT_M);
  }

  /** As {@link #headedSafely(Pose, double[])}, with the robot centre allowed out to {@code limit}. */
  static boolean headedSafely(Pose pose, double[] velocity, double limit) {
    if (!inBox(pose.xM, pose.yM, limit, 0, 0)) {
      return true;
    }
    double stopX = pose.xM + stopDistance(velocity[0]);
    double stopY = pose.yM + stopDistance(velocity[1]);
    return inBox(stopX, stopY, limit, SPOT_FENCE_SLACK_M, SPOT_FENCE_SLACK_M);
  }

  /** Entering from the perimeter still respects the physical wall and Hive stopping bounds. */
  static boolean headedSafelyEntering(Pose pose, double[] velocity) {
    if (inBox(pose.xM, pose.yM, 0, 0)) { return headedSafely(pose, velocity); }
    if (!inBox(pose.xM, pose.yM, MAX_REACH_M, 0, 0)) { return false; }
    return inBox(pose.xM + stopDistance(velocity[0]), pose.yM + stopDistance(velocity[1]),
        MAX_REACH_M, 0, 0);
  }

  /** Field limit for the drive to the reload spot: wide enough to reach it, never past the robot's reach of the wall. */
  static double reloadLimit(double x, double y) {
    return Math.min(MAX_REACH_M, Math.max(FIELD_LIMIT_M, Math.max(Math.abs(x), Math.abs(y)) + RELOAD_MARGIN_M));
  }

  /** Any stick, trigger or RB past TAKEOVER_STICK hands a scripted drive back to the driver. */
  static boolean driverTookOver(Gamepad pad) {
    return Math.abs(pad.left_stick_x) > TAKEOVER_STICK || Math.abs(pad.left_stick_y) > TAKEOVER_STICK
        || Math.abs(pad.right_stick_x) > TAKEOVER_STICK || Math.abs(pad.right_stick_y) > TAKEOVER_STICK
        || pad.left_trigger > TAKEOVER_STICK || pad.right_trigger > TAKEOVER_STICK || pad.right_bumper;
  }

  /** Every point of the straight line from one point to another is inside the area and out of the Hive's footprint. */
  static boolean pathClear(double x0, double y0, double x1, double y1) {
    return pathClearWithin(x0, y0, x1, y1, FIELD_LIMIT_M);
  }

  /**
   * Permit an initial inward leg from the perimeter margin to the normal travel area.
   * The start must remain inside the physical wall reach, and the destination must be in
   * the ordinary area. Every later leg consequently uses the ordinary limit again.
   */
  static boolean pathClearEntering(double x0, double y0, double x1, double y1) {
    if (!inBox(x0, y0, MAX_REACH_M, 0, 0) || !inBox(x1, y1, 0, 0)) { return false; }
    double entryLimit = Math.max(FIELD_LIMIT_M, Math.max(Math.abs(x0), Math.abs(y0)));
    return pathClearWithin(x0, y0, x1, y1, entryLimit);
  }

  private static boolean pathClearWithin(double x0, double y0, double x1, double y1, double limit) {
    int steps = (int) Math.ceil(Math.hypot(x1 - x0, y1 - y0) / 0.02);
    for (int i = 0; i <= steps; i++) {
      double f = steps == 0 ? 0 : (double) i / steps;
      if (!inBox(x0 + f * (x1 - x0), y0 + f * (y1 - y0), limit, 0, 0)) {
        return false;
      }
    }
    return true;
  }

  private static boolean segmentClear(double x0, double y0, double x1, double y1) {
    int steps = (int) Math.ceil(Math.hypot(x1 - x0, y1 - y0) / 0.02);
    for (int i = 0; i <= steps; i++) {
      double f = steps == 0 ? 0 : (double) i / steps;
      double x = x0 + f * (x1 - x0);
      double y = y0 + f * (y1 - y0);
      if (Math.abs(x) < HIVE_HALF_X_M + ROUTE_CLEARANCE_M && Math.abs(y) < HIVE_HALF_Y_M + ROUTE_CLEARANCE_M) {
        return false;
      }
    }
    return true;
  }
}
