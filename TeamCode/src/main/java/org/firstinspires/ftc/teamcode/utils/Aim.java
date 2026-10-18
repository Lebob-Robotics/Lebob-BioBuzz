package org.firstinspires.ftc.teamcode.utils;

/** Where the robot should point, and the shot to take from there: the output of {@link ShootOnTheMove#aim}. */
public final class Aim {
  public final Shot shot;
  public final double targetHeadingRad; // Field heading for the robot
  public final double headingToleranceRad; // How far off the target heading still scores

  Aim(Shot shot, double targetHeadingRad, double headingToleranceRad) {
    this.shot = shot;
    this.targetHeadingRad = targetHeadingRad;
    this.headingToleranceRad = headingToleranceRad;
  }

  /** How far the robot still has to turn, anticlockwise positive. */
  public double headingErrorRad(double currentHeadingRad) {
    return ShootOnTheMove.normalizeAngle(targetHeadingRad - currentHeadingRad);
  }
}
