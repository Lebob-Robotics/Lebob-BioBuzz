package org.firstinspires.ftc.teamcode.shooting.feed;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Counts the balls in the robot, from the beam breaks at the 4th ball's spot and the shots seen leaving, and tells
 * when it holds 4. The path is one ball wide, so every ball loaded passes through the beam: each one entering it
 * while LT loads adds one, each shot takes one off, and the 4th to enter stops loading where it sits in the beam.
 * Entries count only while loading: between shots the backward-held feeder rolls the last ball back into the beam.
 */
@Configurable
public class FourthBall {
  public static int CAPACITY = 4;
  /** Loops the beam must read blocked for a ball to enter, so a one-loop glitch isn't a ball. */
  public static int BLOCKED_LOOPS = 2;
  /**
   * The beam must read clear this long before the next ball can enter. It shines through a ball's holes: one ball
   * read clear for 0.065 s in the middle (19 Nov 172621).
   */
  public static double CLEAR_S = 0.15;
  /**
   * Backup for a miscount: a ball held in the beam this long while loading is the 4th, stopped by a full stack. A
   * slow ball passing took 0.48 s.
   */
  public static double STUCK_S = 1.0;
  /** Loops the beam must read blocked at the start to count a full load of preloads. */
  public static int PRELOAD_LOOPS = 2;

  private int count = 0;
  private boolean inBeam = false; // A ball is in the beam: blocked, or clear for less than CLEAR_S
  private int blockedLoops = 0;
  private double clearSinceS = Double.NaN;
  private boolean starting = true;
  private double blockedLoadingS = 0;
  private double lastS = Double.NaN;

  /**
   * @param blocked  either beam sees a ball this loop
   * @param ballLeft a shot was seen leaving this loop
   * @param ejecting B is running everything backward: it empties the robot
   * @param loading  the indexer is loading (LT)
   * @return true while the robot holds {@link #CAPACITY} balls
   */
  public boolean update(double nowS, boolean blocked, boolean ballLeft, boolean ejecting, boolean loading) {
    double dt = Double.isNaN(lastS) ? 0 : Math.max(0, nowS - lastS);
    lastS = nowS;
    blockedLoops = blocked ? blockedLoops + 1 : 0;
    boolean entered = false;
    if (blocked) {
      clearSinceS = Double.NaN;
      if (!inBeam && blockedLoops >= BLOCKED_LOOPS) {
        inBeam = true;
        entered = true;
      }
    } else if (inBeam) {
      if (Double.isNaN(clearSinceS)) {
        clearSinceS = nowS;
      }
      if (nowS - clearSinceS >= CLEAR_S) {
        inBeam = false;
      }
    }
    if (starting) {
      if (entered) {
        count = CAPACITY; // In the beam from the start: the stack ahead is full
      }
      starting = blocked && !entered && !loading && !ballLeft && !ejecting;
    } else if (entered && loading && !ejecting) {
      count = Math.min(CAPACITY, count + 1);
    }
    if (ejecting) {
      count = 0;
    } else if (ballLeft) {
      count = Math.max(0, count - 1);
    }
    blockedLoadingS = blocked && inBeam ? blockedLoadingS + (loading && count < CAPACITY ? dt : 0) : 0;
    if (blockedLoadingS >= STUCK_S) {
      count = CAPACITY;
    }
    return count >= CAPACITY;
  }

  /** Balls the robot holds, as counted. */
  public int count() {
    return count;
  }

  /** How long the ball now in the beam has been pushed there by loading, s: a passing ball's stays short. */
  public double blockedLoadingS() {
    return blockedLoadingS;
  }
}
