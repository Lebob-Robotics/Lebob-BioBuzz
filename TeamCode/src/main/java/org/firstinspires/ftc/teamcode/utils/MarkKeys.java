package org.firstinspires.ftc.teamcode.utils;

/**
 * Shot Tuning's marking keys. Tap X: the last ball went in. Hold X and tap the D-pad for where a miss went (up long,
 * down short, left, right; two can be combined), then let go of X to save it. A: it went in and bounced out.
 */
public final class MarkKeys {
  private boolean wasX;
  private boolean wasA;
  private boolean shortMiss;
  private boolean longMiss;
  private boolean left;
  private boolean right;

  /** The mark made this loop, or null. Call once a loop. */
  public BallMark update(boolean x, boolean a, boolean up, boolean down, boolean dpadLeft, boolean dpadRight) {
    BallMark mark = null;
    if (x) {
      longMiss |= up;
      shortMiss |= down;
      left |= dpadLeft;
      right |= dpadRight;
    } else if (wasX) {
      boolean any = shortMiss || longMiss || left || right;
      mark = any ? BallMark.miss(shortMiss, longMiss, left, right) : BallMark.IN;
      shortMiss = false;
      longMiss = false;
      left = false;
      right = false;
    }
    if (a && !wasA && !x) {
      mark = BallMark.BOUNCE;
    }
    wasX = x;
    wasA = a;
    return mark;
  }

  /** X is held, so the D-pad is marking rather than changing the shot. */
  public boolean isMarking() {
    return wasX;
  }

  /** What letting go of X would save now. */
  public String pending() {
    boolean any = shortMiss || longMiss || left || right;
    return any ? BallMark.miss(shortMiss, longMiss, left, right).toString() : "IN";
  }
}
