package org.firstinspires.ftc.teamcode.shooting.feed;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class FeedShotsTest {
  private static final double TARGET = 3500;
  private final FeedShots s = new FeedShots();
  private double t = 0;

  private boolean step(boolean feedStarted, boolean ballLeft, boolean feeding, double rpm) {
    t += 0.05;
    return s.update(t, feedStarted, ballLeft, feeding, rpm, TARGET);
  }

  @Test public void aDipIsOneBall() {
    assertFalse(step(true, false, true, TARGET));
    assertTrue(step(false, true, false, 3200));
    assertFalse(step(false, true, false, 3150));
  }

  @Test public void aWheelDropTheDipCheckMissedIsOneBall() {
    step(true, false, true, 3536); // 19 Nov 174626: fell 400 RPM, no dip seen
    step(false, false, true, 3450);
    assertTrue(step(false, false, false, 3136));
    assertFalse(step(false, false, false, 3100));
  }

  @Test public void aFeedWithoutAFallIsNoBall() {
    // 19 Nov 175919: 9 feeds while the balls sat below, wheels within 82 RPM
    for (int i = 0; i < 9; i++) {
      step(true, false, true, 3505);
      for (int j = 0; j < 5; j++) {
        assertFalse(step(false, false, true, 3443));
      }
      for (int j = 0; j < 10; j++) {
        assertFalse(step(false, false, false, 3474));
      }
    }
  }

  @Test public void aFallLongAfterItsFeedIsNotCounted() {
    step(true, false, true, TARGET);
    step(false, false, false, TARGET);
    for (int i = 0; i < 10; i++) {
      step(false, false, false, TARGET);
    }
    assertFalse(step(false, false, false, 3000));
  }

  @Test public void aFeedStartedWhileTheWheelsRecoverMeasuresFromThere() {
    step(true, false, true, 3320); // 19 Nov 175919 20.00: still 135 under after the last ball
    assertFalse(step(false, false, true, 3320));
    assertFalse(step(false, false, false, 3400));
  }

  @Test public void eachFeedCountsOnce() {
    int n = 0;
    for (int i = 0; i < 4; i++) {
      step(true, false, true, TARGET);
      if (step(false, true, false, 3200)) { n++; }
      if (step(false, false, false, 3100)) { n++; }
    }
    assertEquals(4, n);
  }

  @Test public void nothingWithoutAFeed() {
    assertFalse(step(false, true, false, 3000));
  }
}
