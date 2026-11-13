package org.firstinspires.ftc.teamcode.utils;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/** The backward table past 0.95 m, against Lebot Teleop's far backward balls that went long (13 Nov). */
public class BackwardFarTableTest {
  // Distance to the lip and the RPM each long ball left at
  private static final double[][] LONG_BALLS = {
    {1.53, 3505}, {1.52, 3566}, {1.58, 3566}, {1.59, 3597}, {1.85, 3628}, {1.94, 3782}, {1.94, 3812},
  };
  // Half the opening's depth in RPM out there, from the model fitted to Shot Tuning's backward balls
  private static final double HALF_OPENING_RPM = 90;

  @Test
  public void farShotsSitUnderEveryBallThatWentLong() {
    for (double[] ball : LONG_BALLS) {
      Shot shot = ShootOnTheMove.calculateBackward(ball[0], 0, 0);
      assertTrue("no shot at " + ball[0] + " m", shot.valid);
      assertTrue(ball[0] + " m: " + shot.targetRpm + " rpm against a long ball at " + ball[1],
          shot.targetRpm <= ball[1] - HALF_OPENING_RPM);
    }
  }

  @Test
  public void calibratedNearShotsAreUnchanged() {
    assertEquals(3180, ShootOnTheMove.calculateBackward(0.63, 0, 0).targetRpm, 1e-9);
    assertEquals(3230, ShootOnTheMove.calculateBackward(0.73, 0, 0).targetRpm, 1e-9);
    assertEquals(3390, ShootOnTheMove.calculateBackward(0.81, 0, 0).targetRpm, 1e-9);
  }

  @Test
  public void rpmNeverDropsGoingFurtherOnTheHoodStop() {
    double last = 0;
    for (double d = 0.95; d <= 2.3; d += 0.01) {
      double rpm = ShootOnTheMove.calculateBackward(d, 0, 0).targetRpm;
      assertTrue(d + " m: " + rpm + " after " + last, rpm >= last - 1e-9);
      last = rpm;
    }
  }
}
