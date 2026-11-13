package org.firstinspires.ftc.teamcode.utils;

import static org.junit.Assert.*;
import org.junit.Test;

public class IndexerRetryTest {
  @Test public void pausesAfterOneAndAHalfSecondsThenRetriesAtFullPower() {
    IndexerRetry retry = new IndexerRetry();
    retry.update(0, true, false);
    retry.update(1.49, true, false);
    assertFalse(retry.paused()); assertFalse(retry.fullPower());
    retry.update(1.5, true, false);
    assertTrue(retry.paused()); assertFalse(retry.fullPower());
    retry.update(1.64, true, false);
    assertTrue(retry.paused());
    retry.update(1.66, true, false);
    assertFalse(retry.paused()); assertTrue(retry.fullPower());
  }

  @Test public void repeatsIfTheBallStillDoesNotMove() {
    IndexerRetry retry = new IndexerRetry();
    retry.update(0, true, false); retry.update(1.5, true, false);
    retry.update(1.66, true, false);
    retry.update(3.17, true, false);
    assertTrue(retry.paused()); assertFalse(retry.fullPower());
    retry.update(3.33, true, false);
    assertFalse(retry.paused()); assertTrue(retry.fullPower());
  }

  @Test public void eachDetectedBallRestartsTheWait() {
    IndexerRetry retry = new IndexerRetry();
    retry.update(0, true, false); retry.update(1.4, true, true);
    retry.update(1.5, true, false); retry.update(2.9, true, false);
    assertFalse(retry.paused()); assertFalse(retry.fullPower());
    retry.update(3.01, true, false); assertTrue(retry.paused());
    retry.update(3.02, true, true);
    assertFalse(retry.paused()); assertFalse(retry.fullPower());
  }

  @Test public void releaseOrLossOfPermissionImmediatelyCancelsPauseAndBoost() {
    for (double cancelAt : new double[] {1.51, 1.67}) {
      IndexerRetry retry = new IndexerRetry();
      retry.update(0, true, false); retry.update(1.5, true, false);
      if (cancelAt > 1.6) { retry.update(1.66, true, false); }
      retry.update(cancelAt, false, false);
      assertFalse(retry.paused()); assertFalse(retry.fullPower());
      retry.update(10, true, false);
      assertFalse(retry.paused()); assertFalse(retry.fullPower());
    }
  }

  @Test public void loadingOrWaitingBeforeFirstFeedCannotArmRetry() {
    IndexerRetry retry = new IndexerRetry();
    retry.update(0, false, false); retry.update(100, false, false);
    assertFalse(retry.paused()); assertFalse(retry.fullPower());
    retry.update(101, true, false); retry.update(102, true, false);
    assertFalse(retry.paused());
  }
}
