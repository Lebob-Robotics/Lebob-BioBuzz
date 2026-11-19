package org.firstinspires.ftc.teamcode.shooting.feed;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Counts the balls shot, one per feed: its dip, the feed running its whole pulse without one, or a feed stopped
 * (RT let go) after running long enough to get its ball out. The dip check missed 2 of 4 balls in a volley (19 Nov
 * 173943): those wheels fell only about 120 RPM between readings 50-100 ms apart. A dip seen after its feed's pulse
 * ran out is the same ball.
 */
@Configurable
public final class FeedShots {
  /**
   * A feed stopped after running this long had its ball out: a ball takes about 0.26 s from its held-back spot to
   * the wheels, and one let go at 0.27 s left (19 Nov 174626).
   */
  public static double STOPPED_FEED_S = 0.2;

  private boolean counted = true; // This feed's ball is already counted, or no feed has started
  private boolean wasPaused = false;
  private boolean wasFeeding = false;
  private double feedStartS = Double.NaN;

  /**
   * @param feedStarted a feed started this loop
   * @param ballLeft    the wheels dipped this loop
   * @param paused      the feed ran its pulse without a dip and is waiting
   * @param feeding     the feed is running
   * @return true on the loop a ball counts as shot
   */
  public boolean update(double nowS, boolean feedStarted, boolean ballLeft, boolean paused, boolean feeding) {
    if (feedStarted) {
      counted = false;
      feedStartS = nowS;
    }
    boolean stopped = wasFeeding && !feeding && !paused && !ballLeft && nowS - feedStartS >= STOPPED_FEED_S;
    boolean out = false;
    if (ballLeft || (paused && !wasPaused) || stopped) {
      out = !counted;
      counted = true;
    }
    wasPaused = paused;
    wasFeeding = feeding;
    return out;
  }
}
