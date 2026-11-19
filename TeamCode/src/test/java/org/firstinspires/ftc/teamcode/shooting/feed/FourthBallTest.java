package org.firstinspires.ftc.teamcode.shooting.feed;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class FourthBallTest {
  private static final double DT = 0.02;
  private final FourthBall b = new FourthBall();
  private double t = 0;

  private boolean step(boolean blocked, boolean ballLeft, boolean ejecting, boolean loading) {
    t += DT;
    return b.update(t, blocked, ballLeft, ejecting, loading);
  }

  private boolean load(boolean blocked) { return step(blocked, false, false, true); }

  private boolean loadFor(boolean blocked, double s) {
    boolean four = false;
    for (int i = 0; i < Math.round(s / DT); i++) {
      four = load(blocked);
    }
    return four;
  }

  @Test public void aBallPassingThroughTheBeamIsNotTheFourth() {
    assertFalse(loadFor(false, 0.5));
    assertFalse(loadFor(true, 0.2)); // 20 Nov 172041: a ball crossed in 0.08 s at full power
    assertFalse(loadFor(false, 0.5));
    assertFalse(loadFor(true, 0.2));
    assertFalse(loadFor(false, 0.1));
  }

  @Test public void aBallStoppedInTheBeamWhileLoadingIsTheFourth() {
    load(false);
    assertTrue(loadFor(true, FourthBall.ARRIVE_S + 0.05));
    assertTrue(step(true, false, false, false)); // Stays four with LT let go
    assertTrue(step(false, false, false, false)); // And with the beam clear: nothing was shot
  }

  @Test public void aBallSittingInTheBeamWithoutLoadingIsNotCounted() {
    load(false); // A ball stopped mid-pass when LT was let go
    for (int i = 0; i < 100; i++) {
      assertFalse(step(true, false, false, false));
    }
    assertFalse(loadFor(true, 0.1)); // LT again: it moves on
    assertFalse(loadFor(false, 0.2));
  }

  @Test public void aShotAllowsLoadingWithTheOldFourthBallStillInTheBeam() {
    load(false);
    loadFor(true, FourthBall.ARRIVE_S + 0.05);
    assertFalse(step(true, true, false, false));
    assertFalse(loadFor(true, 0.1)); // The old 4th ball moving up
    assertFalse(loadFor(false, 0.3));
    assertTrue(loadFor(true, FourthBall.ARRIVE_S + 0.05)); // A new 4th ball
  }

  @Test public void aBallThatStaysInTheBeamAfterAShotIsTheFourthAgain() {
    load(false);
    loadFor(true, FourthBall.ARRIVE_S + 0.05);
    step(true, true, false, false);
    assertTrue(loadFor(true, FourthBall.ARRIVE_S + 0.05));
  }

  @Test public void startingWithABallInTheBeamIsTheFourthPreload() {
    step(true, false, false, false);
    assertTrue(step(true, false, false, false));
    assertTrue(loadFor(true, 0.1));
  }

  @Test public void startingClearThenBlockedIsNotAPreload() {
    step(false, false, false, false);
    step(true, false, false, false);
    assertFalse(step(true, false, false, false));
  }

  @Test public void ejectFollowsTheBeam() {
    load(false);
    loadFor(true, FourthBall.ARRIVE_S + 0.05);
    assertFalse(step(false, false, true, false));
    assertFalse(step(false, false, false, false));
  }
}
