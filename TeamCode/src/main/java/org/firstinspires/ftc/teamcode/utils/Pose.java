package org.firstinspires.ftc.teamcode.utils;

/** A robot position on the field. */
public final class Pose {
  public final double xM;
  public final double yM;
  public final double headingRad; // Anticlockwise positive

  public Pose(double xM, double yM, double headingRad) {
    this.xM = xM;
    this.yM = yM;
    this.headingRad = headingRad;
  }
}
