package org.firstinspires.ftc.teamcode.utils;

/** Where the robot should point, and the shot to take from there: the output of {@link ShootOnTheMove#aim}. */
public final class Aim {
  public final Shot shot;
  public final double targetHeadingRad; // Field heading the robot must have when the ball leaves
  /**
   * Field heading to hold now. The ball leaves FEED_DELAY_S after the fire command, so a robot turning
   * with the target and holding this heading reaches {@link #targetHeadingRad} as the ball leaves.
   * Holding the target heading itself would lead the shot by the target's turn over the feed delay.
   */
  public final double lockHeadingRad;
  /**
   * How fast the lock heading turns, anticlockwise positive, from the robot's speed across the line to the
   * Cell. Worked out from the velocity rather than by differencing the lock heading, which lagged a loop and
   * jumped with every vision correction.
   */
  public final double lockRateRadPerSec;
  public final double headingToleranceRad; // How far off the target heading still scores
  public final boolean backward; // Out of the robot's front with the hood down, from the backward table
  /**
   * Hood angle and flywheel RPM to hold now: the shot for a ball leaving now. As with the lock heading, the
   * hood and wheels following these are at {@link #shot}'s as the ball leaves, FEED_DELAY_S after the fire
   * command. The shot's own values if there is no shot from here.
   */
  public final double hoodNowDeg;
  public final double rpmNow;
  /**
   * How fast those change as the robot keeps its present velocity, so the hood and wheels can follow a
   * moving target instead of chasing it. 0 when either shot isn't valid.
   */
  public final double hoodRateDegPerSec;
  public final double rpmRatePerSec;

  Aim(Shot shot, double targetHeadingRad, double lockHeadingRad, double lockRateRadPerSec,
      double headingToleranceRad, boolean backward, double hoodNowDeg, double rpmNow,
      double hoodRateDegPerSec, double rpmRatePerSec) {
    this.backward = backward;
    this.hoodNowDeg = hoodNowDeg;
    this.rpmNow = rpmNow;
    this.hoodRateDegPerSec = hoodRateDegPerSec;
    this.rpmRatePerSec = rpmRatePerSec;
    this.shot = shot;
    this.targetHeadingRad = targetHeadingRad;
    this.lockHeadingRad = lockHeadingRad;
    this.lockRateRadPerSec = lockRateRadPerSec;
    this.headingToleranceRad = headingToleranceRad;
  }

  /** How far the robot still has to turn, anticlockwise positive. */
  public double headingErrorRad(double currentHeadingRad) {
    return ShootOnTheMove.normalizeAngle(targetHeadingRad - currentHeadingRad);
  }
}
