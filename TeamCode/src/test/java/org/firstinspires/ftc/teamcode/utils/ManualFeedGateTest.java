package org.firstinspires.ftc.teamcode.utils;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import org.junit.Test;

public class ManualFeedGateTest {
  @Test
  public void neverFeedsWhileStillSpinningUp() {
    ManualFeedGate gate = new ManualFeedGate();
    // Includes the logged 3812 RPM release and arbitrarily long slow spin-ups.
    for (int i = 0; i < 1000; i++) {
      assertFalse(gate.update(true, false));
    }
    assertTrue(gate.update(true, true));
  }

  @Test
  public void keepsFeedingThroughBallSpeedDips() {
    ManualFeedGate gate = new ManualFeedGate();
    assertTrue(gate.update(true, true));
    assertTrue(gate.update(true, false));
  }

  @Test
  public void releaseStallOrAimedModeResetsTheGate() {
    ManualFeedGate gate = new ManualFeedGate();
    assertTrue(gate.update(true, true));
    assertFalse(gate.update(false, true));
    assertFalse(gate.update(true, false));
  }
}
