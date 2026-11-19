package org.firstinspires.ftc.teamcode.control;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class SpeedLimitGovernorTest {
  /**
   * Full stick (1.22 m/s by the top speeds) limited to limitMps, on the drive log 194737 shows: 1.5 m/s per power over
   * 0.15, lagging 0.15 s. Returns {speed at the end, slowest after 0.5 s, fastest after 0.5 s, first time within 0.05}.
   */
  private static double[] run(double startSpeed, double limitMps, double seconds) {
    SpeedLimitGovernor g = new SpeedLimitGovernor();
    double speed = startSpeed;
    double lo = Double.POSITIVE_INFINITY;
    double hi = 0;
    double settledS = Double.NaN;
    for (double t = 0; t < seconds; t += 0.02) {
      double power = g.update(t, 1.0, 1.22, limitMps, speed, 0.15, 0.5, 1.5);
      double steady = Math.max(0, power - 0.15) * 1.5;
      speed += (steady - speed) * 0.02 / 0.15;
      if (t > 0.5) {
        lo = Math.min(lo, speed);
        hi = Math.max(hi, speed);
      }
      if (Double.isNaN(settledS) && Math.abs(speed - limitMps) < 0.05) {
        settledS = t;
      }
    }
    return new double[] {speed, lo, hi, settledS};
  }

  @Test
  public void aSlowLimitStillMovesTheRobotSteadily() {
    double[] r = run(0, 0.25, 3.0);
    assertEquals(0.25, r[0], 0.02);
    assertTrue("no swing: " + r[1] + "-" + r[2], r[1] > 0.18 && r[2] < 0.32);
  }

  @Test
  public void fromFastItBrakesToTheLimitQuickly() {
    double[] r = run(0.8, 0.25, 3.0);
    assertEquals(0.25, r[0], 0.02);
    assertTrue("settled at " + r[3], r[3] < 0.4);
    assertTrue(r[2] < 0.32);
  }

  @Test
  public void aFastLimitIsHeld() {
    assertEquals(0.9, run(0, 0.9, 3.0)[0], 0.03);
  }

  @Test
  public void releaseGivesFullPower() {
    SpeedLimitGovernor g = new SpeedLimitGovernor();
    g.update(0, 1.0, 1.22, 0.25, 1.0, 0.15, 0.5, 1.5);
    assertTrue(g.scale() < 1);
    assertEquals(1.0, g.release(), 0);
  }
}
