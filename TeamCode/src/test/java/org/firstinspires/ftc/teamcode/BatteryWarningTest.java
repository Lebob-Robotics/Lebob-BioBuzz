package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class BatteryWarningTest {
  @Test public void aBatteryLowAtRestIsFlagged() {
    assertTrue(Robot.swapBattery(11.9)); // 9 Nov 080620 started here and fell to 6.8 V
    assertFalse(Robot.swapBattery(12.6));
    assertFalse("No sensor, no warning", Robot.swapBattery(Double.NaN));
  }
}
