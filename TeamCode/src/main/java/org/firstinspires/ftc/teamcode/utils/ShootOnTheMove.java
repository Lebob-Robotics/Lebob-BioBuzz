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
      return Shot.impossible(outsideTable, distanceToLipM, speedTowardCellMps, sidewaysSpeedMps,
          approximateTurningRad(distanceToLipM, speedTowardCellMps, sidewaysSpeedMps));
    }

    TableLookup lookup = new TableLookup(distanceToLipM, speedTowardCellMps);
    double hoodDifferenceDeg = lookup.spread(ShotTable.HOOD_DEG);
    if (Double.isNaN(hoodDifferenceDeg)) { // A neighbouring cell has no shot, so we're at the edge of what scores
      return Shot.impossible(Shot.Reason.NO_SAFE_SHOT, distanceToLipM, speedTowardCellMps, sidewaysSpeedMps,
          approximateTurningRad(distanceToLipM, speedTowardCellMps, sidewaysSpeedMps));
    }
    boolean blend = hoodDifferenceDeg <= MAX_HOOD_DIFFERENCE_TO_BLEND_DEG;
    double hoodDeg = blend ? lookup.blended(ShotTable.HOOD_DEG) : lookup.nearest(ShotTable.HOOD_DEG);
    double targetRpm = blend ? lookup.blended(ShotTable.RPM) : lookup.nearest(ShotTable.RPM);
    double timeOfFlightS = blend ? lookup.blended(ShotTable.TOF_S) : lookup.nearest(ShotTable.TOF_S);
    if (hoodDeg < HOOD_TOP_STOP_DEG + HOOD_STOP_MARGIN_DEG - 1e-9) {
      return Shot.impossible(Shot.Reason.HOOD_OUT_OF_RANGE, distanceToLipM, speedTowardCellMps, sidewaysSpeedMps,
          approximateTurningRad(distanceToLipM, speedTowardCellMps, sidewaysSpeedMps));
    }

    double ballHorizontalSpeedMps = targetRpm * ShotTable.EXIT_SPEED_PER_RPM * Math.cos(Math.toRadians(hoodDeg));
    double sinTurning = -sidewaysSpeedMps / ballHorizontalSpeedMps; // The ball's sideways speed cancels the robot's
    if (!(Math.abs(sinTurning) <= Math.sin(Math.toRadians(MAX_TURNING_DEG)))) { // Also catches NaN
      return Shot.impossible(Shot.Reason.SIDEWAYS_TOO_FAST, distanceToLipM, speedTowardCellMps, sidewaysSpeedMps,
          clampedTurningRad(sinTurning));
    }
    return new Shot(Shot.Reason.OK, targetRpm, hoodDeg, timeOfFlightS, Math.asin(sinTurning),
        distanceToLipM, speedTowardCellMps, sidewaysSpeedMps);
  }

  /**
   * The turn for a shot the table can't make, from its nearest shot, so the robot keeps lining up through a
   * moment without one. Turning 0 then jumped the aim heading 25 degrees and back while strafing (5 Nov, 094211).
   */
  static double approximateTurningRad(double distanceToLipM, double speedTowardCellMps, double sidewaysSpeedMps) {
    double[] distances = ShotTable.DIST_M;
    double[] speeds = ShotTable.VEL_MPS;
    double dist = Math.max(distances[0], Math.min(distances[distances.length - 1], distanceToLipM));
    double speed = Math.max(speeds[0], Math.min(speeds[speeds.length - 1], speedTowardCellMps));
    double[] rpmAndHood = new TableLookup(dist, speed).nearestShot();
    double ballHorizontalSpeedMps = rpmAndHood[0] * ShotTable.EXIT_SPEED_PER_RPM
        * Math.cos(Math.toRadians(rpmAndHood[1]));
    if (!(ballHorizontalSpeedMps > 0)) { // Also NaN: none of the four cells has a shot
      return 0.0;
    }
    return clampedTurningRad(-sidewaysSpeedMps / ballHorizontalSpeedMps);
  }

  private static double clampedTurningRad(double sinTurning) {
    if (Double.isNaN(sinTurning)) {
      return 0.0;
    }
    double maxSin = Math.sin(Math.toRadians(MAX_TURNING_DEG));
    return Math.asin(Math.max(-maxSin, Math.min(maxSin, sinTurning)));
  }

  /** Bounds on the shot's rates: a jump between table rows or ranges is not a rate to follow. */
  private static final double MAX_HOOD_RATE_DEG_PER_SEC = 40;
  private static final double MAX_RPM_RATE_PER_SEC = 3000;

  /** Where to point the robot and what shot to take. Everything is in the field frame. */
  public static Aim aim(Pose robot, double velocityXMps, double velocityYMps, double cellX, double cellY) {
    return aim(robot, velocityXMps, velocityYMps, cellX, cellY, false);
  }

  /**
   * As {@link #aim(Pose, double, double, double, double)}, but with {@code allowBackward} a backward shot
   * is taken whenever the backward table has one, and the hooded shot only otherwise.
   */
  public static Aim aim(Pose robot, double velocityXMps, double velocityYMps, double cellX, double cellY,
      boolean allowBackward) {
    if (allowBackward) {
      Aim backward = aimBackward(robot, velocityXMps, velocityYMps, cellX, cellY);
      if (backward.shot.valid) {
        return backward;
      }
    }
    return aimHooded(robot, velocityXMps, velocityYMps, cellX, cellY);
  }

  /** The hooded shot out of the shooter's side, whether or not the table has one. */
  public static Aim aimHooded(Pose robot, double velocityXMps, double velocityYMps, double cellX, double cellY) {
    Geometry g = new Geometry(robot, velocityXMps, velocityYMps, cellX, cellY, FEED_DELAY_S);
    Geometry now = new Geometry(robot, velocityXMps, velocityYMps, cellX, cellY, 0);
    Shot shot = calculate(g.distanceToLipM, g.speedTowardCellMps, g.sidewaysSpeedMps);
    // Calculated rather than taken back off the shot by its rate: rate noise times the feed delay swung the
    // hood target 6 degrees loop to loop (5 Nov, 082111)
    Shot shotNow = calculate(now.distanceToLipM, now.speedTowardCellMps, now.sidewaysSpeedMps);
    boolean both = shot.valid && shotNow.valid;
    double robotHeadingRad = normalizeAngle(g.directionToCellRad + shot.turningRad - SHOOTER_DIRECTION_RAD);
    double lockHeadingRad = normalizeAngle(now.directionToCellRad + shot.turningRad - SHOOTER_DIRECTION_RAD);
    return new Aim(shot, robotHeadingRad, lockHeadingRad, now.directionRateRadPerSec(),
        headingTolerance(g.distanceToClusterM), false,
        both ? shotNow.hoodDeg : shot.hoodDeg, both ? shotNow.targetRpm : shot.targetRpm,
        both ? rate(shotNow.hoodDeg, shot.hoodDeg, MAX_HOOD_RATE_DEG_PER_SEC) : 0,
        both ? rate(shotNow.targetRpm, shot.targetRpm, MAX_RPM_RATE_PER_SEC) : 0);
  }

  /**
   * The backward shot, whether or not the backward table has one. Its heading is 180 degrees from the
   * hooded shot's, so a caller that has picked it keeps it rather than falling back each loop.
   */
  public static Aim aimBackward(Pose robot, double velocityXMps, double velocityYMps, double cellX, double cellY) {
    Geometry g = new Geometry(robot, velocityXMps, velocityYMps, cellX, cellY, FEED_DELAY_S);
    Geometry now = new Geometry(robot, velocityXMps, velocityYMps, cellX, cellY, 0);
    Shot shot = calculateBackward(g.distanceToLipM, g.speedTowardCellMps, g.sidewaysSpeedMps);
    Shot shotNow = calculateBackward(now.distanceToLipM, now.speedTowardCellMps, now.sidewaysSpeedMps);
    // Standing-still RPMs: the wheels are held at those while a backward shot waits for the robot to slow
    Shot still = calculateBackward(g.distanceToLipM, 0, 0);
    Shot stillNow = calculateBackward(now.distanceToLipM, 0, 0);
    double rpmNow = shot.valid && shotNow.valid ? shotNow.targetRpm
        : shot.valid ? shot.targetRpm : stillNow.valid ? stillNow.targetRpm : still.targetRpm;
    double robotHeadingRad = normalizeAngle(g.directionToCellRad - BACKWARD_DIRECTION_RAD);
    double lockHeadingRad = normalizeAngle(now.directionToCellRad - BACKWARD_DIRECTION_RAD);
    return new Aim(shot, robotHeadingRad, lockHeadingRad, now.directionRateRadPerSec(),
        headingTolerance(g.distanceToClusterM), true, shot.hoodDeg, rpmNow, 0,
        still.valid && stillNow.valid ? rate(stillNow.targetRpm, still.targetRpm, MAX_RPM_RATE_PER_SEC) : 0);
  }

  /** From the shot for a ball leaving now to the one leaving FEED_DELAY_S on. */
  private static double rate(double now, double atRelease, double max) {
    if (!(FEED_DELAY_S > 0)) {
      return 0;
    }
    double r = (atRelease - now) / FEED_DELAY_S;
    return Math.abs(r) <= max ? r : 0; // Also catches NaN
  }

  /** Where the Cell is from the shooter leadS from now, and how the robot moves relative to it. */
  private static final class Geometry {
    final double distanceToClusterM;
    final double distanceToLipM;
    final double directionToCellRad;
    final double speedTowardCellMps;
    final double sidewaysSpeedMps;

    Geometry(Pose robot, double velocityXMps, double velocityYMps, double cellX, double cellY, double leadS) {
      double predictedX = robot.xM + velocityXMps * leadS;
      double predictedY = robot.yM + velocityYMps * leadS;

      double cosHeading = Math.cos(robot.headingRad);
      double sinHeading = Math.sin(robot.headingRad);
      double shooterX = predictedX + SHOOTER_FORWARD_M * cosHeading - SHOOTER_LEFT_M * sinHeading;
      double shooterY = predictedY + SHOOTER_FORWARD_M * sinHeading + SHOOTER_LEFT_M * cosHeading;

      double toCellX = cellX - shooterX;
      double toCellY = cellY - shooterY;
      distanceToClusterM = Math.hypot(toCellX, toCellY);
      directionToCellRad = Math.atan2(toCellY, toCellX);

      double unitToCellX = toCellX / distanceToClusterM;
      double unitToCellY = toCellY / distanceToClusterM;
      speedTowardCellMps = velocityXMps * unitToCellX + velocityYMps * unitToCellY;
      sidewaysSpeedMps = -velocityXMps * unitToCellY + velocityYMps * unitToCellX;
      distanceToLipM = distanceToClusterM - CLUSTER_TO_NEAR_LIP_M;
    }

    /** How fast the direction to the Cell turns, anticlockwise positive: moving left swings it clockwise. */
    double directionRateRadPerSec() {
      return -sidewaysSpeedMps / distanceToClusterM;
    }
  }

  /**
   * The backward shot: hood on its lower stop, RPM read straight between the calibrated distances.
   * Calibrated standing still, so it is refused while the robot is moving.
   */
  public static Shot calculateBackward(double distanceToLipM, double speedTowardCellMps, double sidewaysSpeedMps) {
    double[] distances = BACKWARD_DIST_M;
    Shot.Reason reason = null;
    if (!(distanceToLipM >= Math.min(BACKWARD_MIN_DIST_M, distances[0])
        && distanceToLipM <= distances[distances.length - 1])) {
      reason = Shot.Reason.OUT_OF_RANGE;
    } else if (speedTowardCellMps > BACKWARD_MAX_SPEED_MPS) {
      reason = Shot.Reason.CLOSING_TOO_FAST;
    } else if (speedTowardCellMps < -BACKWARD_MAX_SPEED_MPS) {
      reason = Shot.Reason.BACKING_TOO_FAST;
    } else if (Math.abs(sidewaysSpeedMps) > BACKWARD_MAX_SPEED_MPS) {
      reason = Shot.Reason.SIDEWAYS_TOO_FAST;
    }
    if (reason != null) {
      return Shot.impossible(reason, distanceToLipM, speedTowardCellMps, sidewaysSpeedMps);
    }
    double tableDistM = Math.max(distanceToLipM, distances[0]);
    int i = Math.min(TableLookup.indexAtOrBelow(distances, tableDistM), distances.length - 2);
    double fraction = (tableDistM - distances[i]) / (distances[i + 1] - distances[i]);
    double rpm = BACKWARD_RPM[i] + fraction * (BACKWARD_RPM[i + 1] - BACKWARD_RPM[i]);
    return new Shot(Shot.Reason.OK, rpm, BACKWARD_LAUNCH_DEG, Double.NaN, 0.0,
        distanceToLipM, speedTowardCellMps, sidewaysSpeedMps);
  }

  /**
   * True when the shot is valid, the flywheels and hood are on target, and the heading is close enough to
   * the lock heading both now and FEED_DELAY_MAX_S from now, turning at their current rates. The error
   * changes steadily in between, so the ball leaves on target whenever in that window it goes. The later
   * check stops a shot fired while the robot swings through the target; the first one that only lines up
   * by still turning fast toward it.
   */
  public static boolean readyToFire(Aim aim, boolean flywheelsReady, boolean hoodReady, double headingRad,
      double turnRateRadPerSec) {
    return readyToFire(aim, flywheelsReady, hoodReady, headingRad, turnRateRadPerSec, 0.0);
  }

  /**
   * As above, with the rate the shooter's sideways turn ({@link Shot#turningRad}) is changing: reversing a
   * strafe at 1 m/s swings it 50 degrees in half a second, and a ball fed then leaves aimed for the old speed.
   */
  public static boolean readyToFire(Aim aim, boolean flywheelsReady, boolean hoodReady, double headingRad,
      double turnRateRadPerSec, double turningRateRadPerSec) {
    double errorNowRad = normalizeAngle(aim.lockHeadingRad - headingRad);
    double targetRateRadPerSec = aim.lockRateRadPerSec + turningRateRadPerSec;
    double errorAtLatestExitRad = errorNowRad + (targetRateRadPerSec - turnRateRadPerSec) * FEED_DELAY_MAX_S;
    boolean movingClose = Math.hypot(aim.shot.speedTowardCellMps, aim.shot.sidewaysSpeedMps) >= MOVING_SHOT_SPEED_MPS
        && aim.shot.distanceToLipM < MIN_MOVING_SHOT_LIP_M;
    return aim.shot.valid
        && !movingClose
        && Math.abs(aim.hoodRateDegPerSec) <= MAX_HOOD_RATE_TO_FIRE_DEG_PER_SEC
        && flywheelsReady
        && hoodReady
        && Math.abs(errorNowRad) <= aim.headingToleranceRad
        && Math.abs(errorAtLatestExitRad) <= aim.headingToleranceRad;
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

    /** {RPM, hood degrees} of the nearest of the four cells that has a shot, or NaNs when none has. */
    double[] nearestShot() {
      int nearestDist = distFraction < 0.5 ? distIndex : distIndex + 1;
      int nearestSpeed = speedFraction < 0.5 ? speedIndex : speedIndex + 1;
      if (!Double.isNaN(ShotTable.RPM[nearestDist][nearestSpeed])) {
        return new double[] {ShotTable.RPM[nearestDist][nearestSpeed], ShotTable.HOOD_DEG[nearestDist][nearestSpeed]};
      }
      for (int d = distIndex; d <= distIndex + 1; d++) {
        for (int s = speedIndex; s <= speedIndex + 1; s++) {
          if (!Double.isNaN(ShotTable.RPM[d][s])) {
            return new double[] {ShotTable.RPM[d][s], ShotTable.HOOD_DEG[d][s]};
          }
        }
      }
      return new double[] {Double.NaN, Double.NaN};
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
