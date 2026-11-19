package org.firstinspires.ftc.teamcode.shooting.feed;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class FeedShotsTest {
  private final FeedShots s = new FeedShots();

  @Test public void aDipIsOneBall() {
    assertFalse(s.update(true, false, false));
    assertTrue(s.update(false, true, false));
    assertFalse(s.update(false, false, false));
  }

  @Test public void aPulseRunningOutIsOneBall() {
    s.update(true, false, false);
    assertTrue(s.update(false, false, true));
    assertFalse(s.update(false, false, true));
    assertFalse(s.update(false, true, false)); // Its dip, late: the same ball
  }

  @Test public void eachFeedCountsOnce() {
    int n = 0;
    // 19 Nov 173943: dip, pulse out, dip, pulse out
    boolean[][] feeds = {{true, false}, {false, true}, {true, false}, {false, true}};
    for (boolean[] f : feeds) {
      s.update(true, false, false);
      if (s.update(false, f[0], f[1])) { n++; }
      s.update(false, false, false);
    }
    assertTrue("counted " + n, n == 4);
  }

  @Test public void nothingWithoutAFeed() {
    assertFalse(s.update(false, true, false));
    assertFalse(s.update(false, false, true));
  }

  @Test public void releasedMidFeedIsNotCounted() {
    s.update(true, false, false);
    assertFalse(s.update(false, false, false));
  }
}
