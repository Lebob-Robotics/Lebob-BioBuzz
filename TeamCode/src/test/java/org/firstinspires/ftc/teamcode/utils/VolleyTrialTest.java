package org.firstinspires.ftc.teamcode.utils;

import static org.junit.Assert.*;
import org.junit.Test;

public class VolleyTrialTest {
  private VolleyTrial armed() {
    VolleyTrial t = new VolleyTrial();
    t.input(0, false, true, false);
    t.input(0.1, true, true, false);
    return t;
  }

  @Test public void exactlyFourDetectedBallsEndOnePress() {
    VolleyTrial t = armed();
    assertTrue(t.active());
    assertTrue(t.observe(true, false));
    assertFalse(t.observe(true, false));
    for (int i = 0; i < 4; i++) { t.observe(false, true); }
    assertFalse(t.active());
    assertEquals(4, t.balls());
    assertTrue(t.takeFinished());
    assertFalse(t.takeFinished());
    t.observe(true, true);
    assertEquals(4, t.balls());
    t.input(2, true, true, false); // Still holding RB cannot fire a fifth or a new trial.
    assertFalse(t.active());
  }

  @Test public void heldRbWhileMovingCannotFireOnArrival() {
    VolleyTrial t = new VolleyTrial();
    t.input(0, false, false, false);
    t.input(1, true, false, false);
    t.input(2, true, true, false);
    assertFalse(t.active());
    t.input(3, false, true, false);
    t.input(4, true, true, false);
    assertTrue(t.active());
  }

  @Test public void unmarkedResultsRequireReleaseAndNewPressAfterMarking() {
    VolleyTrial t = armed(); t.observe(true, true); t.cancel();
    assertTrue(t.takeFinished());
    t.input(1, false, true, true);
    t.input(2, true, true, true);
    assertFalse(t.active());
    t.input(3, true, true, false);
    assertFalse(t.active());
    t.input(4, false, true, false); t.input(5, true, true, false);
    assertTrue(t.active());
  }

  @Test public void releasePositionLossAndTimeoutFinishPartialTrials() {
    VolleyTrial t = armed(); t.observe(true, true);
    t.input(1, false, true, false);
    assertTrue(t.takeFinished()); assertEquals("released", t.reason());
    t = armed(); t.observe(true, true);
    t.input(1, true, false, false);
    assertTrue(t.takeFinished()); assertEquals("position lost", t.reason());
    t = armed(); t.observe(true, false);
    t.input(12.2, true, true, false);
    assertTrue(t.takeFinished()); assertEquals("timeout", t.reason());
  }

  @Test public void noFeedProducesNoPhantomVolleyToMark() {
    VolleyTrial t = armed(); t.input(1, false, true, false);
    assertFalse(t.takeFinished());
    assertFalse(t.active());
  }
}
