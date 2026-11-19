package org.firstinspires.ftc.teamcode.shooting.feed;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Counts the balls in the robot, from the beam breaks at the 4th ball's spot and the shots seen leaving, and tells
 * when it holds 4. The path is one ball wide, so every ball loaded passes through the beam: each one entering it
 * while LT loads adds one, each shot takes one off. Entries count only while loading: between shots the
 * backward-held feeder rolls the last ball back into the beam.
 *
 * <p>Full needs the beam to agree: the 4th ball stays in the beam, a ball with room ahead passes through. So the
 * stack is full once a ball has been held in the beam {@link #HOLD_S} of loading with 3 or more counted, and a ball
 * that passes with 4 counted shows the count was one high. A held ball with fewer counted is a slow ball, not full
 * (19 Nov 175919: the 2nd ball sat in the beam 1 s and loading stopped at 2).
 */
@Configurable
public class FourthBall {
  public static int CAPACITY = 4;
  /** Loops the beam must read blocked for a ball to enter, so a one-loop glitch isn't a ball. */
  public static int BLOCKED_LOOPS = 2;
  /**
   * The beam must read clear this long before the next ball can enter. It shines through a ball's holes: one ball
   * read clear for 0.065-0.145 s, 0.1-0.27 s after it blocked (19 Nov 172621, 173943).
   */
  public static double CLEAR_S = 0.15;
  /**
   * A ball blocked this long has passed, so a shorter gap after it is the next ball touching it: 2 balls back to back
   * read blocked 0.46 s, clear 0.075 s (19 Nov 175919), and 0.33 s, clear 0.13 s (191845). Light through a ball's
   * holes came at most 0.27 s in.
   */
  public static double PASS_S = 0.3;
  /** The shortest gap between touching balls; shorter is a glitch (19 Nov 175122: 0.034 s). */
  public static double GAP_S = 0.05;
  /** Loading time a ball stays in the beam before it counts as the 4th, held by a full stack. Passes take 0.61 s. */
  public static double HOLD_S = 0.8;

  private int count = 0;
  private boolean confirmed = false; // A ball held in the beam showed the stack really is full
  private boolean inBeam = false; // A ball is in the beam: blocked, or clear for less than CLEAR_S
  private int blockedLoops = 0;
  private double firstBlockedS = Double.NaN;
  private double ballStartS = Double.NaN; // When the ball now in the beam first blocked it
  private double clearSinceS = Double.NaN;
  private double heldLoadingS = 0;
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
    if (blockedLoops == 1) {
      firstBlockedS = nowS;
    }
    boolean entered = false;
    boolean passed = false;
    if (blocked) {
      if (inBeam && !Double.isNaN(clearSinceS) && clearSinceS - ballStartS >= PASS_S && nowS - clearSinceS >= GAP_S) {
        inBeam = false; // The next ball, right behind
        passed = true;
      }
      clearSinceS = Double.NaN;
      if (!inBeam && blockedLoops >= BLOCKED_LOOPS) {
        inBeam = true;
        entered = true;
        ballStartS = firstBlockedS;
      }
    } else if (inBeam) {
      if (Double.isNaN(clearSinceS)) {
        clearSinceS = nowS;
      }
      if (nowS - clearSinceS >= CLEAR_S) {
        inBeam = false;
        passed = true;
      }
    }
    if (passed && loading && count >= CAPACITY && !confirmed) {
      count = CAPACITY - 1; // Room ahead: it wasn't the 4th
    }
    if (entered && loading && !ejecting) {
      count = Math.min(CAPACITY, count + 1);
      confirmed = false;
      heldLoadingS = 0;
    }
    if (ejecting) {
      count = 0;
      confirmed = false;
    } else if (ballLeft) {
      count = Math.max(0, count - 1);
      confirmed = false;
      heldLoadingS = 0;
    }
    heldLoadingS = blocked && inBeam ? heldLoadingS + (loading ? dt : 0) : 0;
    if (!confirmed && heldLoadingS >= HOLD_S && count >= CAPACITY - 1) {
      count = CAPACITY;
      confirmed = true;
    }
    return count >= CAPACITY && confirmed;
  }

  /** The driver says the robot is empty (Share): the count was wrong. */
  public void reset() {
    count = 0;
    confirmed = false;
    heldLoadingS = 0;
  }

  /**
   * 4 counted, full or not yet shown full: the intake stops so it doesn't push the stack while the beam checks. Kept
   * running, the 4th ball's push sent the front ball into the wheels (19 Nov 191845).
   */
  public boolean fourCounted() {
    return count >= CAPACITY;
  }

  /** Balls the robot holds, as counted. */
  public int count() {
    return count;
  }

  /** How long the ball now in the beam has been pushed there by loading, s: a passing ball's stays short. */
  public double blockedLoadingS() {
    return heldLoadingS;
  }
}
