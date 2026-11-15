package org.firstinspires.ftc.teamcode.robot;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class RampToggleTest {
  @Test public void dpadUpTogglesTheRampOncePerPress() {
    assertTrue(DriverPad.rampToggled(true, false, false, true));
    assertFalse("Held, not pressed again", DriverPad.rampToggled(true, true, false, true));
    assertFalse(DriverPad.rampToggled(false, true, false, true));
  }

  @Test public void whileAimingDpadUpTrimsTheRpmInstead() {
    assertFalse(DriverPad.rampToggled(true, false, true, true));
  }

  @Test public void aTuningToolsButtonsLeaveTheRampAlone() {
    assertFalse(DriverPad.rampToggled(true, false, false, false));
  }
}
