package org.firstinspires.ftc.teamcode.robot;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class BatteryWarningTest {
  @Test public void aBatteryLowAtRestIsFlagged() {
    assertTrue(BatteryMonitor.swapBattery(11.9)); // 9 Nov 080620 started here and fell to 6.8 V
    assertFalse(BatteryMonitor.swapBattery(12.6));
    assertFalse("No sensor, no warning", BatteryMonitor.swapBattery(Double.NaN));
  }
}
