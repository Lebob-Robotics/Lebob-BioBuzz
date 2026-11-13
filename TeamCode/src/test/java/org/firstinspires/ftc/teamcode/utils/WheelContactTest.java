package org.firstinspires.ftc.teamcode.utils;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class WheelContactTest {
  @Test
  public void aSharpFallAtSteadyTargetIsATouch() {
    WheelContact c = new WheelContact();
    assertFalse(c.update(0.00, true, 3400, 3400));
    assertFalse(c.update(0.05, true, 3390, 3400));
    assertTrue(c.update(0.10, true, 2900, 3400)); // 191420 at 14.6 s fell 1900 in 0.1 s
  }

  @Test
  public void speedControlNoiseAndSpinUpAreNot() {
    WheelContact c = new WheelContact();
    assertFalse(c.update(0.00, true, 3400, 3400));
    assertFalse(c.update(0.05, true, 3250, 3400)); // Ordinary wobble
    WheelContact spinUp = new WheelContact();
    assertFalse(spinUp.update(0.00, true, 2000, 3400)); // Far below the target: still coming up
    assertFalse(spinUp.update(0.05, true, 1600, 3400));
    WheelContact overshoot = new WheelContact();
    assertFalse(overshoot.update(0.00, true, 3874, 3563));
    assertFalse(overshoot.update(0.10, true, 3413, 3563)); // Settling back after recovering
    WheelContact retarget = new WheelContact();
    assertFalse(retarget.update(0.00, true, 3600, 3600));
    assertFalse(retarget.update(0.10, true, 3200, 3200)); // The target moved down with it
  }

  @Test
  public void notWatchingForgetsTheWindow() {
    WheelContact c = new WheelContact();
    assertFalse(c.update(0.00, true, 3400, 3400));
    assertFalse(c.update(0.05, false, 3400, 3400));
    assertFalse(c.update(0.10, true, 2900, 3400));
  }
}
