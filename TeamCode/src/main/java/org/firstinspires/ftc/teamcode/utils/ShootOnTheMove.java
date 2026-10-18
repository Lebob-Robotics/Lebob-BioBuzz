package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.teamcode.utils.ShootOnTheMoveConstants.*;

/**
 * Shoot on the move maths. Pure Java with no hardware, so it is unit tested on a laptop.
 *
 * <p>Which hood angle and RPM score from each distance and speed was worked out ahead of time by
 * Hazzer's simulation in tools/shots, and saved in {@link ShotTable}. This class:
 * <ol>
 *   <li>finds the distance to the Cell and the robot's speed toward it and sideways ({@link #aim}),</li>
 *   <li>looks up the hood angle and RPM for those ({@link #calculate}),</li>
 *   <li>turns the shooter a little to cancel the robot's sideways speed,</li>
 *   <li>checks everything is ready before a ball is fed ({@link #readyToFire}).</li>
 * </ol>
 */
public final class ShootOnTheMove {
  private ShootOnTheMove() {}

  /**
   * The shot for a given distance and speed.
   *
   * @param distanceToLipM     from the shooter to the Cell's near lip
   * @param speedTowardCellMps positive when closing
   * @param sidewaysSpeedMps   across the line to the Cell, positive to the left
   */
  public static Shot calculate(double distanceToLipM, double speedTowardCellMps, double sidewaysSpeedMps) {
    double[] distances = ShotTable.DIST_M;
    double[] speeds = ShotTable.VEL_MPS;
    Shot.Reason outsideTable = null;
    if (distanceToLipM < distances[0] || distanceToLipM > distances[distances.length - 1]) {
      outsideTable = Shot.Reason.OUT_OF_RANGE;
    } else if (speedTowardCellMps > speeds[speeds.length - 1]) {
      outsideTable = Shot.Reason.CLOSING_TOO_FAST;
    } else if (speedTowardCellMps < speeds[0]) {
      outsideTable = Shot.Reason.BACKING_TOO_FAST;
    }
    if (outsideTable != null) {
      return Shot.impossible(outsideTable, distanceToLipM, speedTowardCellMps, sidewaysSpeedMps);
    }

    TableLookup lookup = new TableLookup(distanceToLipM, speedTowardCellMps);
    double hoodDifferenceDeg = lookup.spread(ShotTable.HOOD_DEG);
    if (Double.isNaN(hoodDifferenceDeg)) { // A neighbouring cell has no shot, so we're at the edge of what scores
      return Shot.impossible(Shot.Reason.NO_SAFE_SHOT, distanceToLipM, speedTowardCellMps, sidewaysSpeedMps);
    }
    boolean blend = hoodDifferenceDeg <= MAX_HOOD_DIFFERENCE_TO_BLEND_DEG;
    double hoodDeg = blend ? lookup.blended(ShotTable.HOOD_DEG) : lookup.nearest(ShotTable.HOOD_DEG);
    double targetRpm = blend ? lookup.blended(ShotTable.RPM) : lookup.nearest(ShotTable.RPM);
    double timeOfFlightS = blend ? lookup.blended(ShotTable.TOF_S) : lookup.nearest(ShotTable.TOF_S);

    double ballHorizontalSpeedMps = targetRpm * ShotTable.EXIT_SPEED_PER_RPM * Math.cos(Math.toRadians(hoodDeg));
    double sinTurning = -sidewaysSpeedMps / ballHorizontalSpeedMps; // The ball's sideways speed cancels the robot's
    if (!(Math.abs(sinTurning) <= Math.sin(Math.toRadians(MAX_TURNING_DEG)))) { // Also catches NaN
      return Shot.impossible(Shot.Reason.SIDEWAYS_TOO_FAST, distanceToLipM, speedTowardCellMps, sidewaysSpeedMps);
    }
    return new Shot(Shot.Reason.OK, targetRpm, hoodDeg, timeOfFlightS, Math.asin(sinTurning),
        distanceToLipM, speedTowardCellMps, sidewaysSpeedMps);
  }

  /** Where to point the robot and what shot to take. Everything is in the field frame. */
  public static Aim aim(Pose robot, double velocityXMps, double velocityYMps, double cellX, double cellY) {
    double predictedX = robot.xM + velocityXMps * FEED_DELAY_S; // Where the robot will be when the ball leaves
    double predictedY = robot.yM + velocityYMps * FEED_DELAY_S;

    double cosHeading = Math.cos(robot.headingRad);
    double sinHeading = Math.sin(robot.headingRad);
    double shooterX = predictedX + SHOOTER_FORWARD_M * cosHeading - SHOOTER_LEFT_M * sinHeading;
    double shooterY = predictedY + SHOOTER_FORWARD_M * sinHeading + SHOOTER_LEFT_M * cosHeading;

    double toCellX = cellX - shooterX;
    double toCellY = cellY - shooterY;
    double distanceToClusterM = Math.hypot(toCellX, toCellY);
    double directionToCellRad = Math.atan2(toCellY, toCellX);

    double unitToCellX = toCellX / distanceToClusterM;
    double unitToCellY = toCellY / distanceToClusterM;
    double speedTowardCellMps = velocityXMps * unitToCellX + velocityYMps * unitToCellY;
    double sidewaysSpeedMps = -velocityXMps * unitToCellY + velocityYMps * unitToCellX;

    Shot shot = calculate(distanceToClusterM - CLUSTER_TO_NEAR_LIP_M, speedTowardCellMps, sidewaysSpeedMps);
    double shooterHeadingRad = directionToCellRad + shot.turningRad;
    double robotHeadingRad = normalizeAngle(shooterHeadingRad - SHOOTER_DIRECTION_RAD);
    return new Aim(shot, robotHeadingRad, headingTolerance(distanceToClusterM));
  }

  /** True when the shot is valid, the flywheels and hood are on target, and the heading is close enough. */
  public static boolean readyToFire(Aim aim, boolean flywheelsReady, boolean hoodReady, double headingRad) {
    return aim.shot.valid
        && flywheelsReady
        && hoodReady
        && Math.abs(aim.headingErrorRad(headingRad)) <= aim.headingToleranceRad;
  }

  /**
   * How far off the heading can be and still go in: the half width of the opening, less the ball
   * radius and margin. The cluster is a little further than the opening, so this errs on the safe side.
   */
  static double headingTolerance(double distanceToClusterM) {
    double clearanceM = CELL_HALF_WIDTH_M - BALL_RADIUS_M - SIDE_MARGIN_M;
    return Math.min(Math.atan(clearanceM / distanceToClusterM), Math.toRadians(MAX_HEADING_TOLERANCE_DEG));
  }

  /** Same angle, between minus pi and pi. */
  public static double normalizeAngle(double angleRad) {
    return Math.atan2(Math.sin(angleRad), Math.cos(angleRad));
  }

  /** Reads the table between grid points, from the four cells around a distance and speed. */
  static final class TableLookup {
    private final int distIndex; // The cell at or below the distance
    private final int speedIndex; // The cell at or below the speed
    private final double distFraction; // How far toward the next distance, 0 to 1
    private final double speedFraction;

    /** The distance and speed must be inside the table. */
    TableLookup(double distanceM, double speedMps) {
      distIndex = indexAtOrBelow(ShotTable.DIST_M, distanceM);
      speedIndex = indexAtOrBelow(ShotTable.VEL_MPS, speedMps);
      distFraction = fractionToNext(ShotTable.DIST_M, distIndex, distanceM);
      speedFraction = fractionToNext(ShotTable.VEL_MPS, speedIndex, speedMps);
    }

    /** Bilinear blend of the four cells. */
    double blended(double[][] grid) {
      double atLowerSpeed = interpolate(grid[distIndex][speedIndex], grid[distIndex + 1][speedIndex], distFraction);
      double atHigherSpeed = interpolate(grid[distIndex][speedIndex + 1], grid[distIndex + 1][speedIndex + 1], distFraction);
      return interpolate(atLowerSpeed, atHigherSpeed, speedFraction);
    }

    double nearest(double[][] grid) {
      int nearestDist = distFraction < 0.5 ? distIndex : distIndex + 1;
      int nearestSpeed = speedFraction < 0.5 ? speedIndex : speedIndex + 1;
      return grid[nearestDist][nearestSpeed];
    }

    /** Largest minus smallest of the four cells. NaN when any is NaN. */
    double spread(double[][] grid) {
      double largest = Double.NEGATIVE_INFINITY;
      double smallest = Double.POSITIVE_INFINITY;
      for (int d = distIndex; d <= distIndex + 1; d++) {
        for (int s = speedIndex; s <= speedIndex + 1; s++) {
          if (Double.isNaN(grid[d][s])) {
            return Double.NaN;
          }
          largest = Math.max(largest, grid[d][s]);
          smallest = Math.min(smallest, grid[d][s]);
        }
      }
      return largest - smallest;
    }

    private static double interpolate(double from, double to, double fraction) {
      return from + (to - from) * fraction;
    }

    /** Never the last index, so index + 1 always exists. */
    private static int indexAtOrBelow(double[] axis, double value) {
      int index = 0;
      while (index < axis.length - 2 && axis[index + 1] <= value) {
        index++;
      }
      return index;
    }

    private static double fractionToNext(double[] axis, int index, double value) {
      return (value - axis[index]) / (axis[index + 1] - axis[index]);
    }
  }
}
