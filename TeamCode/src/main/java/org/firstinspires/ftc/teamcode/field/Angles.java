package org.firstinspires.ftc.teamcode.field;

/** Angle helpers shared by the whole robot. */
public final class Angles {
  private Angles() {}

  /** Same angle, between minus pi and pi. */
  public static double normalize(double angleRad) {
    return Math.atan2(Math.sin(angleRad), Math.cos(angleRad));
  }
}
