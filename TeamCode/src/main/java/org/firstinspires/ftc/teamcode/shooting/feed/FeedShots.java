package org.firstinspires.ftc.teamcode.shooting.feed;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Counts the balls shot, at most one per feed: its dip, or the wheels falling {@link #DROP_RPM} below where the feed
 * started (or the target, if lower) during the feed or {@link #AFTER_S} after it. A feed whose wheels don't fall had
 * no ball at the wheels and counts nothing: counting those took 9 balls off 2 (19 Nov 175919). Every ball out in 7
 * logged runs fell 215-490 RPM, every empty feed 0-123 (19 Nov).
 */
@Configurable
public final class FeedShots {
  public static double DROP_RPM = 200;
  /** A ball's lowest wheel speed came up to 0.3 s after its feed stopped. */
  public static double AFTER_S = 0.4;

  private boolean counted = true; // This feed's ball is already counted, or no feed has started
  private boolean wasFeeding = false;
  private double refRpm = 0;
  private double feedEndS = Double.NaN;

  /**
   * @param feedStarted a feed started this loop
   * @param ballLeft    the wheels dipped this loop
   * @param feeding     the feed is running
   * @param rpm         the wheels' speed now
   * @param targetRpm   the wheels' target now
   * @return true on the loop a ball counts as shot
   */
  public boolean update(double nowS, boolean feedStarted, boolean ballLeft, boolean feeding, double rpm,
      double targetRpm) {
    if (feedStarted) {
      counted = false;
      refRpm = Math.min(rpm, targetRpm);
      feedEndS = Double.NaN;
    }
    if (wasFeeding && !feeding) {
      feedEndS = nowS;
    }
    wasFeeding = feeding;
    boolean live = feeding || feedStarted || (!Double.isNaN(feedEndS) && nowS - feedEndS <= AFTER_S);
    if (!counted && live && (ballLeft || refRpm - rpm >= DROP_RPM)) {
      counted = true;
      return true;
    }
    return false;
  }
}
