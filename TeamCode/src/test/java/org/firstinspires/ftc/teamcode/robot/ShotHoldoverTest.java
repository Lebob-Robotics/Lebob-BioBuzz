package org.firstinspires.ftc.teamcode.robot;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class ShotHoldoverTest {
  private final ShotHoldover h = new ShotHoldover();

  @Test public void letGoMidFeedHoldsUntilTheFeedEnds() {
    assertTrue(h.update(0.0, true, true));
    assertTrue(h.update(0.05, false, true));
    assertTrue(h.update(0.10, false, true));
    assertFalse(h.update(0.15, false, false));
    assertFalse(h.update(0.20, false, true)); // No new feed from a let-go button
  }

  @Test public void letGoMidFeedHoldsAtMostTheLimit() {
    h.update(0.0, true, true);
    assertTrue(h.update(0.05, false, true));
    assertFalse(h.update(0.05 + DriverConstants.SHOT_RELEASE_HOLD_S, false, true));
  }

  @Test public void letGoBetweenFeedsStopsAtOnce() {
    h.update(0.0, true, false);
    assertFalse(h.update(0.05, false, false));
  }

  @Test public void neverHeldIsNeverHeld() {
    assertFalse(h.update(0.0, false, true));
  }
}
