package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class RampToggleTest {
  @Test public void dpadUpTogglesTheRampOncePerPress() {
    assertTrue(Robot.rampToggled(true, false, false, true));
    assertFalse("Held, not pressed again", Robot.rampToggled(true, true, false, true));
    assertFalse(Robot.rampToggled(false, true, false, true));
  }

  @Test public void whileAimingDpadUpTrimsTheRpmInstead() {
    assertFalse(Robot.rampToggled(true, false, true, true));
  }

  @Test public void aTuningToolsButtonsLeaveTheRampAlone() {
    assertFalse(Robot.rampToggled(true, false, false, false));
  }
}
