package org.firstinspires.ftc.teamcode.control;

/**
 * The share of the driver's power to keep so the robot drives at a speed limit: the power the limit needs (what the
 * wheels need to roll, plus the limit by the drive's speed per power), corrected by how far the measured speed is
 * off it, now and over time. Scaling the power by the speed's share alone left too little to roll the robot, and it
 * stopped; following the speed by the error's sum alone was slow to brake and swung the speed 0 to 0.6 m/s.
 */
public final class SpeedLimitGovernor {
  private static final double MAX_INTEGRAL_POWER = 0.3;

  private double scale = 1.0;
  private double integral = 0.0;
  private double lastS = Double.NaN; // NaN while not limiting

  /** Not limiting: full power, and the next limit starts afresh. */
  public double release() {
    scale = 1.0;
    integral = 0.0;
    lastS = Double.NaN;
    return scale;
  }

  /**
   * @param powerAsked  the driver's power, 0 to 1
   * @param askedMps    the speed that power gives by the drive's top speeds
   * @param limitMps    the speed limit
   * @param speedMps    the robot's measured speed
   * @param startPower  what the wheels need to roll at all
   * @param kp          power per m/s the speed is over or under the limit
   * @param ki          power per second per m/s
   */
  public double update(double nowS, double powerAsked, double askedMps, double limitMps, double speedMps,
      double startPower, double kp, double ki) {
    if (!(powerAsked > 1e-6) || !(askedMps > 1e-6)) {
      return release();
    }
    double errorMps = limitMps - speedMps;
    double feedforward = startPower + limitMps * powerAsked / askedMps;
    if (!Double.isNaN(lastS)) {
      double dtS = Math.min(0.1, Math.max(0.0, nowS - lastS));
      double unclamped = feedforward + kp * errorMps + integral;
      // Not while the power is already at 0 or full and the error pushes it further: braking hard from fast wound it
      // to -0.3, and the robot then sat still under its limit
      boolean saturated = unclamped <= 0 && errorMps < 0 || unclamped >= powerAsked && errorMps > 0;
      if (!saturated) {
        integral = Math.max(-MAX_INTEGRAL_POWER, Math.min(MAX_INTEGRAL_POWER, integral + ki * dtS * errorMps));
      }
    }
    lastS = nowS;
    double power = feedforward + kp * errorMps + integral;
    scale = Math.max(0.0, Math.min(1.0, power / powerAsked));
    return scale;
  }

  public double scale() {
    return scale;
  }
}
