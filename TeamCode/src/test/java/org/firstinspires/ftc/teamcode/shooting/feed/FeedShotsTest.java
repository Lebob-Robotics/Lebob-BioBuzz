package org.firstinspires.ftc.teamcode.shooting.feed;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class FeedShotsTest {
  private final FeedShots s = new FeedShots();
  private double t = 0;

  private boolean step(boolean feedStarted, boolean ballLeft, boolean paused, boolean feeding) {
    t += 0.05;
    return s.update(t, feedStarted, ballLeft, paused, feeding);
  }

  @Test public void aDipIsOneBall() {
    assertFalse(step(true, false, false, true));
    assertTrue(step(false, true, false, false));
    assertFalse(step(false, false, false, false));
  }

  @Test public void aPulseRunningOutIsOneBall() {
    step(true, false, false, true);
    assertTrue(step(false, false, true, false));
    assertFalse(step(false, false, true, false));
    assertFalse(step(false, true, false, false)); // Its dip, late: the same ball
  }

  @Test public void eachFeedCountsOnce() {
    int n = 0;
    // 19 Nov 173943: dip, pulse out, dip, pulse out
    boolean[][] feeds = {{true, false}, {false, true}, {true, false}, {false, true}};
    for (boolean[] f : feeds) {
      step(true, false, false, true);
      if (step(false, f[0], f[1], false)) { n++; }
      step(false, false, false, false);
    }
    assertEquals(4, n);
  }

  @Test public void nothingWithoutAFeed() {
    assertFalse(step(false, true, false, false));
    assertFalse(step(false, false, true, false));
  }

  @Test public void aFeedLetGoAfterItsBallReachedTheWheelsCounts() {
    step(true, false, false, true);
    for (int i = 0; i < 4; i++) {
      assertFalse(step(false, false, false, true));
    }
    assertTrue(step(false, false, false, false)); // 19 Nov 174626: let go 0.27 s in, as the ball left
  }

  @Test public void aFeedLetGoAtOnceIsNotCounted() {
    step(true, false, false, true);
    assertFalse(step(false, false, false, false));
  }
}
