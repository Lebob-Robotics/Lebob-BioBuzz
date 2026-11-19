package org.firstinspires.ftc.teamcode.shooting;

import static org.firstinspires.ftc.teamcode.shooting.SotmConstants.*;

/**
 * How much of the driver's speed to keep while aiming so the shot table still has a shot: the robot drives no
 * faster toward, away from or across the line to the Cell than the table can score from at this distance, nor
 * faster than the fire gate's moving-shot limit close to the lip.
 */
public final class SotmSpeedLimit {
  private SotmSpeedLimit() {}

  private static final int SCAN_STEPS = 20;
  private static final int REFINE_STEPS = 5;

  /**
   * The share of {towardMps, sidewaysMps} in [0, 1] to drive at: 1 when all of it has a shot, else the largest share
   * that has one at it and at marginScale times it, scanned up from standing so a gap further out doesn't count.
   * 1 when standing still has no shot either (out of range): the driver must be free to drive into range.
   *
   * @param closeMovingAllowed as {@link ShootOnTheMove#readyToFire}'s: no cap close to the lip
   * @param minSpeedMps never slower than this (or the asked-for speed, if slower): where no speed has a shot, such
   *                    as backing out of the table's far end, the driver can still drive out
   */
  public static double share(boolean backward, double distanceToLipM, double towardMps, double sidewaysMps,
      double marginScale, boolean closeMovingAllowed, double minSpeedMps) {
    double speed = Math.hypot(towardMps, sidewaysMps);
    double k = share(backward, distanceToLipM, towardMps, sidewaysMps, marginScale, closeMovingAllowed);
    return speed > 1e-6 ? Math.max(k, Math.min(1.0, minSpeedMps / speed)) : k;
  }

  private static double share(boolean backward, double distanceToLipM, double towardMps, double sidewaysMps,
      double marginScale, boolean closeMovingAllowed) {
    if (!(Math.hypot(towardMps, sidewaysMps) > 1e-6) || !hasShot(backward, distanceToLipM, 0, 0, closeMovingAllowed)) {
      return 1.0;
    }
    double scale = Math.max(1.0, marginScale);
    double good = 0;
    double bad = Double.NaN;
    for (int i = 1; i <= SCAN_STEPS; i++) {
      double k = (double) i / SCAN_STEPS;
      if (fits(backward, distanceToLipM, k * towardMps, k * sidewaysMps, scale, closeMovingAllowed)) {
        good = k;
      } else {
        bad = k;
        break;
      }
    }
    if (Double.isNaN(bad)) {
      return 1.0;
    }
    for (int i = 0; i < REFINE_STEPS; i++) {
      double k = (good + bad) / 2;
      if (fits(backward, distanceToLipM, k * towardMps, k * sidewaysMps, scale, closeMovingAllowed)) {
        good = k;
      } else {
        bad = k;
      }
    }
    return good;
  }

  /**
   * {@link #share} for the stick's velocity {toward, sideways}, checked here and where the robot will be lookaheadS on
   * at its measured speed toward the Cell (driving in at 0.7 m/s it crossed into the slow zone close to the lip and
   * took 0.2 s to get down to its speed), and also in the direction the robot really moves, which drifts off the
   * stick's (at 1.14 m/s it had no safe shot while the stick's direction did). That direction is checked at the
   * stick's speed: limited to its own measured speed, a slow robot could never speed up again and crept to a stop.
   */
  public static double shareForStick(boolean backward, double distanceToLipM, double towardMps, double sidewaysMps,
      double measuredTowardMps, double measuredSidewaysMps, double lookaheadS, double marginScale,
      boolean closeMovingAllowed, double minSpeedMps) {
    double asked = Math.hypot(towardMps, sidewaysMps);
    double dAhead = distanceToLipM - Math.max(0.0, measuredTowardMps) * lookaheadS;
    double k = Math.min(share(backward, distanceToLipM, towardMps, sidewaysMps, marginScale, closeMovingAllowed,
            minSpeedMps),
        share(backward, dAhead, towardMps, sidewaysMps, marginScale, closeMovingAllowed, minSpeedMps));
    double measured = Math.hypot(measuredTowardMps, measuredSidewaysMps);
    if (measured > 0.05 && asked > 1e-6) {
      double s = asked / measured;
      k = Math.min(k, share(backward, distanceToLipM, measuredTowardMps * s, measuredSidewaysMps * s, marginScale,
          closeMovingAllowed, minSpeedMps));
    }
    return k;
  }

  private static boolean fits(boolean backward, double d, double toward, double sideways, double scale,
      boolean closeMovingAllowed) {
    return hasShot(backward, d, toward, sideways, closeMovingAllowed)
        && hasShot(backward, d, toward * scale, sideways * scale, closeMovingAllowed);
  }

  private static boolean hasShot(boolean backward, double d, double toward, double sideways,
      boolean closeMovingAllowed) {
    Shot shot = backward ? ShootOnTheMove.calculateBackward(d, toward, sideways)
        : ShootOnTheMove.calculate(d, toward * SOTM_TOWARD_GAIN, sideways * SOTM_SIDEWAYS_GAIN);
    return shot.valid && (backward || closeMovingAllowed || !ShootOnTheMove.tooCloseToShootMoving(shot));
  }
}
