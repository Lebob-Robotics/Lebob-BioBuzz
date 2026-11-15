package org.firstinspires.ftc.teamcode.opmodes.check;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class VisionCalibrationSweepTest {
  @Test public void stepsEveryStepSecondsAndCycles() {
    assertEquals(0, VisionCalibration.stepAt(0));
    assertEquals(0, VisionCalibration.stepAt(VisionCalibration.STEP_S - 0.01));
    assertEquals(1, VisionCalibration.stepAt(VisionCalibration.STEP_S));
    assertEquals(0, VisionCalibration.stepAt(VisionCalibration.STEP_S * VisionCalibration.SWEEP.length));
  }

  @Test public void startsAtTheCurrentCompSetting() {
    assertEquals(2, VisionCalibration.SWEEP[0][0]);
    assertEquals(20, VisionCalibration.SWEEP[0][1]);
  }
}
