package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.List;
import org.junit.Test;

public class FieldRouteTest {
  private static void assertClearOfTheHive(double x, double y, List<double[]> route) {
    for (double[] w : route) {
      for (int i = 0; i <= 100; i++) {
        double px = x + (w[0] - x) * i / 100.0;
        double py = y + (w[1] - y) * i / 100.0;
        assertTrue("crosses the Hive at " + px + ", " + py,
            Math.abs(px) >= FieldRoute.HIVE_HALF_X_M || Math.abs(py) >= FieldRoute.HIVE_HALF_Y_M);
        assertTrue(Math.abs(px) <= FieldRoute.FIELD_LIMIT_M && Math.abs(py) <= FieldRoute.FIELD_LIMIT_M);
      }
      x = w[0];
      y = w[1];
    }
  }

  @Test
  public void onOneSideItDrivesStraight() {
    List<double[]> route = FieldRoute.route(-1.25, -0.32, -1.25, 1.30);
    assertEquals(1, route.size());
    assertEquals(1.30, route.get(0)[1], 1e-9);
  }

  @Test
  public void toTheOtherSideItGoesRoundTheNearerEnd() {
    List<double[]> route = FieldRoute.route(-1.25, -0.32, 1.25, -0.32);
    assertEquals(3, route.size());
    assertTrue(route.get(0)[1] < 0); // Round the -y end, the nearer one
    assertClearOfTheHive(-1.25, -0.32, route);
    assertEquals(1.25, route.get(2)[0], 1e-9);
  }

  @Test
  public void everyGridPointIsInsideTheAreaAndClearOfTheHive() {
    for (double[] p : FieldRoute.GRID) {
      // The trials' own margin: 0.1 m inside the field limit, a robot half width clear of the Hive
      assertTrue(Math.abs(p[0]) <= FieldRoute.FIELD_LIMIT_M - 0.1 && Math.abs(p[1]) <= FieldRoute.FIELD_LIMIT_M - 0.1);
      assertTrue(Math.abs(p[0]) >= FieldRoute.HIVE_HALF_X_M + 0.15 || Math.abs(p[1]) >= FieldRoute.HIVE_HALF_Y_M + 0.15);
    }
  }

  @Test
  public void fromAnEndToASideItTakesOneCorner() {
    List<double[]> route = FieldRoute.route(0.0, -1.30, -1.25, 0.5);
    assertEquals(2, route.size());
    assertClearOfTheHive(0.0, -1.30, route);
  }
}
