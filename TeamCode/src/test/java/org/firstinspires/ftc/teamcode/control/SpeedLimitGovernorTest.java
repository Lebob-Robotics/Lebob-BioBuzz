package org.firstinspires.ftc.teamcode.control;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class SpeedLimitGovernorTest {
  /** A drive that needs 0.2 power to roll and reaches 1.2 m/s at full power, lagging 0.15 s. */
  private static double run(double startSpeed, double share, double limitMps, double seconds) {
    SpeedLimitGovernor g = new SpeedLimitGovernor();
    double speed = startSpeed;
    double maxAfterHalf = 0;
    for (double t = 0; t < seconds; t += 0.02) {
      double power = g.update(t, share, 1.0, limitMps, speed, 0.15, 3.0);
      double steady = Math.max(0, power - 0.2) / 0.8 * 1.2;
      speed += (steady - speed) * 0.02 / 0.15;
    }
    return speed;
  }

  @Test
  public void aSlowLimitStillMovesTheRobot() {
    // Power 0.2 by the top speeds alone (0.25 m/s of 1.22) never rolled it
    double speed = run(0, 0.2, 0.25, 3.0);
    assertEquals(0.25, speed, 0.03);
  }

  @Test
  public void fromFastItSlowsToTheLimit() {
    assertEquals(0.6, run(1.2, 0.5, 0.6, 3.0), 0.03);
  }

  @Test
  public void releaseGivesFullPower() {
    SpeedLimitGovernor g = new SpeedLimitGovernor();
    g.update(0, 0.2, 1.0, 0.25, 1.0, 0.15, 3.0);
    assertTrue(g.scale() < 1);
    assertEquals(1.0, g.release(), 0);
  }
}
