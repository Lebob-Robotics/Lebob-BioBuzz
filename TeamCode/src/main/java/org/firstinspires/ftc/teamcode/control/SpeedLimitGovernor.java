package org.firstinspires.ftc.teamcode.control;

/**
 * The share of the driver's power to keep so the robot drives at a speed limit: starts from the power the limit
 * needs by the drive's top speeds plus a start power, then follows the measured speed to the limit. Scaling the power
 * by the speed's share alone left too little to roll the robot, and it stopped.
 */
public final class SpeedLimitGovernor {
  private double scale = 1.0;
  private double lastS = Double.NaN; // NaN while not limiting

  /** Not limiting: full power next time, and the next limit starts afresh. */
  public double release() {
    scale = 1.0;
    lastS = Double.NaN;
    return scale;
  }

  /**
   * @param share       the limit as a share of the speed the power asks for, by the top speeds
   * @param powerAsked  the driver's power, 0 to 1
   * @param limitMps    the speed limit
   * @param speedMps    the robot's measured speed
   * @param startPower  added to the first loop's share of the power: about what the wheels need to roll
   * @param gain        power share per second per m/s the speed is off the limit
   */
  public double update(double nowS, double share, double powerAsked, double limitMps, double speedMps,
      double startPower, double gain) {
    if (Double.isNaN(lastS)) {
      scale = share + (powerAsked > 1e-6 ? startPower / powerAsked : 0.0);
    } else {
      double dtS = Math.min(0.1, Math.max(0.0, nowS - lastS));
      scale += gain * dtS * (limitMps - speedMps);
    }
    scale = Math.max(0.0, Math.min(1.0, scale));
    lastS = nowS;
    return scale;
  }

  public double scale() {
    return scale;
  }
}
