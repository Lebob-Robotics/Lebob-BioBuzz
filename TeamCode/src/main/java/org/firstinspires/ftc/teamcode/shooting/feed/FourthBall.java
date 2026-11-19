package org.firstinspires.ftc.teamcode.shooting.feed;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Tells when the robot holds 4 balls, from the beam breaks at the 4th ball's spot and the shots seen leaving.
 * A ball arriving in the beam means 4. A shot means 3 or fewer, even with a ball still in the beam: the stack only
 * moves up when the indexer runs, so the old 4th ball can sit there after a shot. The next ball to arrive, a
 * blocked beam after a clear one, means 4 again.
 */
@Configurable
public class FourthBall {
  /** Loops the beam must stay blocked to count as a ball arriving; one clear loop is enough to see a gap. */
  public static int BLOCKED_LOOPS = 2;
  /**
   * After a shot, a ball that stays in the beam this long while loading means the stack didn't move up: it is full
   * again without a gap ever showing.
   */
  public static double STILL_BLOCKED_LOADING_S = 1.0;

  private boolean four = false;
  private boolean seenBlocked = false; // Debounced: the beam's last settled reading
  private int blockedLoops = 0;
  private double blockedLoadingS = 0;
  private double lastS = Double.NaN;

  /**
   * @param blocked  either beam sees a ball this loop
   * @param ballLeft a shot was seen leaving this loop
   * @param ejecting B is running everything backward: the beam alone says what is left
   * @param loading  the indexer is loading (LT)
   * @return true while 4 balls are in
   */
  public boolean update(double nowS, boolean blocked, boolean ballLeft, boolean ejecting, boolean loading) {
    double dt = Double.isNaN(lastS) ? 0 : Math.max(0, nowS - lastS);
    lastS = nowS;
    blockedLoops = blocked ? blockedLoops + 1 : 0;
    boolean arrived = false;
    if (!blocked) {
      seenBlocked = false;
    } else if (!seenBlocked && blockedLoops >= BLOCKED_LOOPS) {
      seenBlocked = true;
      arrived = true;
    }
    if (ejecting) {
      four = seenBlocked;
    } else if (ballLeft) {
      four = false;
    } else if (arrived) {
      four = true;
    }
    blockedLoadingS = !four && seenBlocked && loading ? blockedLoadingS + dt : 0;
    if (blockedLoadingS >= STILL_BLOCKED_LOADING_S) {
      four = true;
      blockedLoadingS = 0;
    }
    return four;
  }

  public boolean isFour() {
    return four;
  }
}
