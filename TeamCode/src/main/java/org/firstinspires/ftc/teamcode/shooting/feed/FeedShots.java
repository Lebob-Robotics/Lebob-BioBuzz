package org.firstinspires.ftc.teamcode.shooting.feed;

/**
 * Counts the balls shot, one per feed: its dip, or the feed running its whole pulse without one. The dip check
 * missed 2 of 4 balls in a volley (19 Nov 173943): those wheels fell only about 120 RPM between readings 50-100 ms
 * apart, and the count left 2 balls in an empty robot. A dip seen after its feed's pulse ran out is the same ball.
 */
public final class FeedShots {
  private boolean counted = true; // This feed's ball is already counted, or no feed has started
  private boolean wasPaused = false;

  /**
   * @param feedStarted a feed started this loop
   * @param ballLeft    the wheels dipped this loop
   * @param paused      the feed ran its pulse without a dip and is waiting
   * @return true on the loop a ball counts as shot
   */
  public boolean update(boolean feedStarted, boolean ballLeft, boolean paused) {
    if (feedStarted) {
      counted = false;
    }
    boolean out = false;
    if (ballLeft || (paused && !wasPaused)) {
      out = !counted;
      counted = true;
    }
    wasPaused = paused;
    return out;
  }
}
