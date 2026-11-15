package org.firstinspires.ftc.teamcode.field;

/** A robot position on the field. */
public final class FieldPose {
  public final double xM;
  public final double yM;
  public final double headingRad; // Anticlockwise positive

  public FieldPose(double xM, double yM, double headingRad) {
    this.xM = xM;
    this.yM = yM;
    this.headingRad = headingRad;
  }
}
