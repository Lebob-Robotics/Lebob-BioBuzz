package org.firstinspires.ftc.teamcode.utils;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Test;

public class HeadingHoldTest {
  private final HeadingHold hold = new HeadingHold();

  @After
  public void restore() {
    HeadingHold.ENABLED = true;
  }

  @Test
  public void holdsTheHeadingDrivingStartedAt() {
    assertEquals(0.3, hold.update(false, true, 0.3, 0.0, 1.0), 1e-9);
    // The base swings, the target doesn't
    assertEquals(0.3, hold.update(false, true, 0.45, 0.2, 1.1), 1e-9);
  }

  @Test
  public void noHoldWhileTurningOrStandingStill() {
    assertTrue(Double.isNaN(hold.update(true, true, 0.3, 0.0, 1.0)));
    assertTrue(Double.isNaN(hold.update(false, false, 0.3, 0.0, 1.1)));
  }

  @Test
  public void waitsForAReleasedTurnToCoastOut() {
    hold.update(true, true, 0.0, 1.5, 1.0);
    // Let go, still coasting at 1.5 rad/s: no target yet
    assertTrue(Double.isNaN(hold.update(false, true, 0.1, 1.5, 1.1)));
    // Slowed down: holds where it ended up
    assertEquals(0.4, hold.update(false, true, 0.4, 0.1, 1.3), 1e-9);
  }

  @Test
  public void takesTheHeadingAfterAWhileEvenIfStillTurning() {
    hold.update(true, true, 0.0, 1.5, 1.0);
    assertEquals(0.9, hold.update(false, true, 0.9, 1.0, 1.0 + HeadingHold.CAPTURE_AFTER_S + 0.01), 1e-9);
  }

  @Test
  public void aCorrectedHeadingEstimateMovesTheTarget() {
    hold.update(false, true, 3.0, 0.0, 1.0);
    hold.shift(0.5);
    assertEquals(3.5 - 2 * Math.PI, hold.getTargetRad(), 1e-9);
  }

  @Test
  public void disabledNeverHolds() {
    HeadingHold.ENABLED = false;
    assertTrue(Double.isNaN(hold.update(false, true, 0.3, 0.0, 1.0)));
  }
}
