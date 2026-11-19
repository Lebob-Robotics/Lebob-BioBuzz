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
    loadFor(true, 0.2);
    return loadFor(false, 0.4);
  }

  private void shot() { step(false, true, false, false); }

  @Test public void theFourthBallLoadedStopsLoadingInTheBeam() {
    loadFor(false, 0.2);
    assertFalse(pass());
    assertFalse(pass());
    assertFalse(pass());
    assertEquals(3, b.count());
    loadFor(true, 0.02);
    assertTrue(loadFor(true, 0.02)); // As soon as it is in the beam
    assertEquals(4, b.count());
  }

  @Test public void aSlowBallIsCountedOnce() {
    loadFor(false, 0.2);
    loadFor(true, 0.6); // 19 Nov 172621: the second ball sat in the beam 0.48 s
    assertFalse(loadFor(false, 0.4));
    assertEquals(1, b.count());
  }

  @Test public void lightThroughABallsHolesIsNotASecondBall() {
    loadFor(false, 0.2);
    loadFor(true, 0.1); // 19 Nov 172621: blocked 0.1 s, clear 0.065 s, blocked 0.24 s
    loadFor(false, 0.06);
    loadFor(true, 0.24);
    loadFor(false, 0.4);
    assertEquals(1, b.count());
  }

  @Test public void shotsTakeBallsOffAndLoadingRefillsToFour() {
    loadFor(false, 0.2);
    pass();
    pass();
    pass();
    loadFor(true, 0.04);
    assertEquals(4, b.count());
    shot();
    assertEquals(3, b.count());
    run(true, false, 1.0); // The old 4th ball still in the beam, not loading
    assertFalse(loadFor(true, 0.1)); // Loading moves it up
    loadFor(false, 0.4);
    loadFor(true, 0.02);
    assertTrue(loadFor(true, 0.02)); // The new 4th
  }

  @Test public void ballsRollingBackIntoTheBeamBetweenShotsAreNotCounted() {
    loadFor(false, 0.2);
    pass();
    pass();
    run(true, false, 0.5); // Feeder held backward between shots
    run(false, false, 0.5);
    assertEquals(2, b.count());
  }

  @Test public void aBallHeldInTheBeamWhileLoadingIsTheFourth() {
    loadFor(false, 0.2);
    assertTrue(loadFor(true, FourthBall.STUCK_S + 0.1)); // Preloads the count never saw
  }

  @Test public void aBallInTheBeamAtTheStartIsNotFour() {
    run(true, false, 0.5); // 19 Nov 174626: one of only 2 balls
    assertEquals(0, b.count());
    assertTrue(loadFor(true, FourthBall.STUCK_S + 0.1)); // 4 preloads: LT can't move the 4th
  }

  @Test public void ejectEmptiesTheCount() {
    loadFor(false, 0.2);
    pass();
    step(true, false, true, false);
    assertEquals(0, b.count());
    assertFalse(step(false, false, false, false));
  }

  @Test public void resetSetsTheCountToZero() {
    loadFor(false, 0.2);
    pass();
    pass();
    pass();
    loadFor(true, 0.04);
    assertEquals(4, b.count());
    b.reset();
    assertFalse(loadFor(true, 0.1)); // The ball still in the beam doesn't count again
    assertEquals(0, b.count());
  }

  @Test public void shotsNeverCountBelowZero() {
    shot();
    shot();
    assertEquals(0, b.count());
  }
}
