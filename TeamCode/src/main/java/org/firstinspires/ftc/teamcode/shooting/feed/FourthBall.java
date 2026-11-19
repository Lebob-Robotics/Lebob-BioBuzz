package org.firstinspires.ftc.teamcode.shooting.feed;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Tells when the robot holds 4 balls, from the beam breaks at the 4th ball's spot and the shots seen leaving. The
 * path is one ball wide, so every ball loaded passes through the beam: a ball only counts as the 4th once it stays
 * in the beam while LT loads, because the stack ahead of it can't move. A shot means 3 or fewer, even with that ball
 * still in the beam. A ball in the beam at the start is the 4th preload.
 */
@Configurable
public class FourthBall {
  /**
   * A ball in the beam this long while loading is the 4th, stopped by a full stack. A ball passing at full indexer
   * power blocked it about 0.08 s.
   */
  public static double ARRIVE_S = 0.4;
  /** Loops the beam must read blocked at the start to count a preloaded 4th ball. */
  public static int PRELOAD_LOOPS = 2;

  private boolean four = false;
  private boolean starting = true;
  private int preloadLoops = 0;
  private double blockedLoadingS = 0;
  private double lastS = Double.NaN;

  /**
   * @param blocked  either beam sees a ball this loop
   * @param ballLeft a shot was seen leaving this loop
   * @param ejecting B is running everything backward: a ball sitting in the beam is the 4th
   * @param loading  the indexer is loading (LT)
   * @return true while 4 balls are in
   */
  public boolean update(double nowS, boolean blocked, boolean ballLeft, boolean ejecting, boolean loading) {
    double dt = Double.isNaN(lastS) ? 0 : Math.max(0, nowS - lastS);
    lastS = nowS;
    if (starting) {
      preloadLoops = blocked ? preloadLoops + 1 : 0;
      if (preloadLoops >= PRELOAD_LOOPS) {
        four = true;
      }
      starting = blocked && !four && !loading && !ballLeft && !ejecting;
    }
    if (ejecting) {
      four = blocked;
    } else if (ballLeft) {
      four = false;
      blockedLoadingS = 0;
    }
    blockedLoadingS = blocked ? blockedLoadingS + (loading && !four ? dt : 0) : 0;
    if (!four && blockedLoadingS >= ARRIVE_S) {
      four = true;
    }
    return four;
  }

  /** How long the ball now in the beam has been pushed there by loading, s: a passing ball's stays short. */
  public double blockedLoadingS() {
    return blockedLoadingS;
  }
}
