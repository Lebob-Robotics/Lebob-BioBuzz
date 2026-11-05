package org.firstinspires.ftc.teamcode.utils;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Picks the heading to hold while the driver translates without turning. Open loop, the base swings up to
 * 10-15 deg/s on its own while strafing at full stick (5 Nov, 093520), so a straight drive curved.
 * The heading is taken once a turn the driver let go of has slowed down; it is dropped while turning and
 * while the drive stick is centred, so starting to drive again holds wherever the robot then points.
 */
@Configurable
public final class HeadingHold {
  public static boolean ENABLED = true;
  // A released turn coasts on for about 0.3 s (100 deg/s to 0, 5 Nov, 093520). Taking the heading mid-coast
  // would pull the robot back against it, so it waits for the turn to slow below this, or CAPTURE_AFTER_S.
  public static double CAPTURE_BELOW_RATE_RAD_S = Math.toRadians(15);
  public static double CAPTURE_AFTER_S = 0.5;

  private double targetRad = Double.NaN;
  private double lastTurnS = Double.NEGATIVE_INFINITY;

  /**
   * @param turning     the turn stick is past its deadband
   * @param translating the drive stick is past its deadband
   * @return the heading to hold, CCW positive, or NaN for none (drive with the turn stick)
   */
  public double update(boolean turning, boolean translating, double headingRad, double turnRateRadPerSec,
      double nowS) {
    if (turning) {
      lastTurnS = nowS;
    }
    if (!ENABLED || turning || !translating) {
      targetRad = Double.NaN;
      return targetRad;
    }
    if (Double.isNaN(targetRad)
        && (Math.abs(turnRateRadPerSec) < CAPTURE_BELOW_RATE_RAD_S || nowS - lastTurnS > CAPTURE_AFTER_S)) {
      targetRad = headingRad;
    }
    return targetRad;
  }

  /** The heading estimate was corrected by this much: the held heading moves with it, so the robot doesn't turn. */
  public void shift(double dHeadingRad) {
    if (!Double.isNaN(targetRad)) {
      targetRad = ShootOnTheMove.normalizeAngle(targetRad + dHeadingRad);
    }
  }

  /** Drops the held heading, e.g. after the heading was reset by hand. */
  public void clear() {
    targetRad = Double.NaN;
  }

  /** NaN when none. */
  public double getTargetRad() {
    return targetRad;
  }
}
