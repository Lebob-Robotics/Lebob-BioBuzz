package org.firstinspires.ftc.teamcode.shooting.feed;

import static org.junit.Assert.assertEquals;
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

  private boolean run(boolean blocked, boolean loading, double s) {
    boolean four = false;
    for (int i = 0; i < Math.round(s / DT); i++) {
      four = step(blocked, false, false, loading);
    }
    return four;
  }

  private boolean loadFor(boolean blocked, double s) { return run(blocked, true, s); }

  /** A ball passing through the beam while loading. */
  private boolean pass() {
    loadFor(true, 0.45);
    return loadFor(false, 0.4);
  }

  private void shot() { step(false, true, false, false); }

  /** Loads 3 and holds the 4th in the beam until it counts. */
  private void loadFour() {
    loadFor(false, 0.2);
    pass();
    pass();
    pass();
    assertTrue(loadFor(true, FourthBall.HOLD_S + 0.1));
  }

  @Test public void theFourthBallHeldInTheBeamStopsLoading() {
    loadFor(false, 0.2);
    assertFalse(pass());
    assertFalse(pass());
    assertFalse(pass());
    assertEquals(3, b.count());
    assertFalse(loadFor(true, 0.6)); // Could still be passing
    assertEquals(4, b.count());
    assertTrue(loadFor(true, 0.3));
  }

  @Test public void aBallPassingWithFourCountedShowsTheCountWasHigh() {
    loadFor(false, 0.2);
    pass();
    pass();
    pass();
    loadFor(true, 0.45); // A ball with room ahead, but a missed shot left the count 1 high

    assertEquals(4, b.count());
    assertFalse(loadFor(false, 0.4)); // It moved on: there was room
    assertEquals(3, b.count());
  }

  @Test public void aSlowSecondBallIsNotFull() {
    loadFor(false, 0.2);
    pass();
    assertFalse(loadFor(true, 2.0)); // 19 Nov 175919: sat in the beam with 1 ahead
    assertEquals(2, b.count());
  }

  @Test public void twoBallsBackToBackCountTwo() {
    loadFor(false, 0.2);
    loadFor(true, 0.46); // 19 Nov 175919: blocked 0.46 s, clear 0.075 s, blocked
    loadFor(false, 0.08);
    loadFor(true, 0.5);
    assertEquals(2, b.count());
  }

  @Test public void aSlowBallIsCountedOnce() {
    loadFor(false, 0.2);
    loadFor(true, 0.6);
    assertFalse(loadFor(false, 0.4));
    assertEquals(1, b.count());
  }

  @Test public void lightThroughABallsHolesIsNotASecondBall() {
    loadFor(false, 0.2);
    loadFor(true, 0.1); // 19 Nov 172621: blocked 0.1 s, clear 0.065 s, blocked 0.24 s
    loadFor(false, 0.06);
    loadFor(true, 0.24);
    loadFor(false, 0.4);
    loadFor(true, 0.28); // 19 Nov 173943: blocked 0.27 s, clear 0.145 s, blocked 0.15 s
    loadFor(false, 0.14);
    loadFor(true, 0.16);
    loadFor(false, 0.4);
    assertEquals(2, b.count());
  }

  @Test public void aGlitchAfterABallIsNotAnother() {
    loadFor(false, 0.2);
    loadFor(true, 0.48); // 19 Nov 175122: clear 0.034 s, blocked 0.036 s
    loadFor(false, 0.04);
    loadFor(true, 0.04);
    loadFor(false, 0.4);
    assertEquals(1, b.count());
  }

  @Test public void shotsTakeBallsOffAndLoadingRefillsToFour() {
    loadFour();
    shot();
    assertEquals(3, b.count());
    run(true, false, 1.0); // The old 4th ball still in the beam, not loading
    assertFalse(loadFor(true, 0.4)); // Loading moves it up
    loadFor(false, 0.4);
    assertEquals(3, b.count());
    loadFor(true, 0.45);
    assertTrue(loadFor(true, FourthBall.HOLD_S)); // The new 4th
  }

  @Test public void ballsRollingBackIntoTheBeamBetweenShotsAreNotCounted() {
    loadFor(false, 0.2);
    pass();
    pass();
    run(true, false, 0.5); // Feeder held backward between shots
    run(false, false, 0.5);
    assertEquals(2, b.count());
  }

  @Test public void aBallInTheBeamAtTheStartIsNotFour() {
    run(true, false, 0.5); // 19 Nov 174626: one of only 2 balls
    assertEquals(0, b.count());
    assertFalse(loadFor(true, 2.0));
  }

  @Test public void ejectEmptiesTheCount() {
    loadFor(false, 0.2);
    pass();
    step(true, false, true, false);
    assertEquals(0, b.count());
    assertFalse(step(false, false, false, false));
  }

  @Test public void resetSetsTheCountToZero() {
    loadFour();
    b.reset();
    assertFalse(loadFor(true, 1.0)); // The ball still in the beam doesn't count again
    assertEquals(0, b.count());
  }

  @Test public void shotsNeverCountBelowZero() {
    shot();
    shot();
    assertEquals(0, b.count());
  }
}
