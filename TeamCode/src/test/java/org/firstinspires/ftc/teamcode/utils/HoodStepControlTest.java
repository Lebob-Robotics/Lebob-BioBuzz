package org.firstinspires.ftc.teamcode.utils;

import static org.junit.Assert.assertEquals;
import org.junit.Test;

public class HoodStepControlTest {
  @Test public void eachRightPressAddsTwoDegreesWithoutRepeatingWhileHeld() {
    HoodStepControl control = new HoodStepControl();
    assertEquals(2, control.update(0, true, false, 48), 0);
    assertEquals(2, control.update(2, true, false, 48), 0);
    assertEquals(2, control.update(2, false, false, 48), 0);
    assertEquals(4, control.update(2, true, false, 48), 0);
  }

  @Test public void eachLeftPressSubtractsTwoDegrees() {
    HoodStepControl control = new HoodStepControl();
    assertEquals(2, control.update(4, false, true, 48), 0);
    assertEquals(2, control.update(2, false, true, 48), 0);
    control.update(2, false, false, 48);
    assertEquals(0, control.update(2, false, true, 48), 0);
  }

  @Test public void stepsRespectBothTravelLimits() {
    HoodStepControl control = new HoodStepControl();
    assertEquals(48, control.update(48, true, false, 48), 0);
    control.update(0, false, false, 48);
    assertEquals(0, control.update(0, false, true, 48), 0);
  }
}
