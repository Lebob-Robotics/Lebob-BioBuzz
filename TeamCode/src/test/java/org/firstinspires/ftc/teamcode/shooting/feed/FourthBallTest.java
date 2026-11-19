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

  @Test public void aBallArrivingInTheBeamIsFour() {
    assertFalse(load(false));
    assertFalse(load(true)); // One loop could be flicker
    assertTrue(load(true));
    assertTrue(step(false, false, false, false)); // Stays four with the beam clear and nothing shot
  }

  @Test public void aShotAllowsLoadingWithTheOldFourthBallStillInTheBeam() {
    load(true);
    assertTrue(load(true));
    assertFalse(step(true, true, false, false));
    assertFalse(load(true)); // Still the old ball: no gap yet
    assertFalse(load(false)); // It moved up
    assertFalse(load(true));
    assertTrue(load(true)); // A new 4th ball
  }

  @Test public void aBallStuckInTheBeamWhileLoadingIsFourAgain() {
    load(true);
    load(true);
    step(true, true, false, false);
    boolean four = false;
    for (int i = 0; i < (int) Math.ceil(FourthBall.STILL_BLOCKED_LOADING_S / DT) + 1; i++) {
      four = load(true);
    }
    assertTrue(four);
  }

  @Test public void notLoadingDoesNotTimeOut() {
    load(true);
    load(true);
    step(true, true, false, false);
    for (int i = 0; i < 200; i++) {
      assertFalse(step(true, false, false, false));
    }
  }

  @Test public void ejectFollowsTheBeam() {
    load(true);
    load(true);
    assertTrue(step(true, false, true, false));
    assertFalse(step(false, false, true, false));
    assertFalse(step(false, false, false, false));
  }

  @Test public void startingWithABallInTheBeamIsFour() {
    step(true, false, false, false);
    assertTrue(step(true, false, false, false));
  }
}
