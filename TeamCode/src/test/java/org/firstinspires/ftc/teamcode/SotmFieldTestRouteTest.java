package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.List;
import org.junit.Test;

public class SotmFieldTestRouteTest {
  private static void assertClearOfTheHive(double x, double y, List<double[]> route) {
    for (double[] w : route) {
      for (int i = 0; i <= 100; i++) {
        double px = x + (w[0] - x) * i / 100.0;
        double py = y + (w[1] - y) * i / 100.0;
        assertTrue("crosses the Hive at " + px + ", " + py,
            Math.abs(px) >= SotmFieldTest.HIVE_HALF_X_M || Math.abs(py) >= SotmFieldTest.HIVE_HALF_Y_M);
        assertTrue(Math.abs(px) <= SotmFieldTest.FIELD_LIMIT_M && Math.abs(py) <= SotmFieldTest.FIELD_LIMIT_M);
      }
      x = w[0];
      y = w[1];
    }
  }

  @Test
  public void onOneSideItDrivesStraight() {
    List<double[]> route = SotmFieldTest.route(-1.25, -0.32, -1.25, 1.30);
    assertEquals(1, route.size());
    assertEquals(1.30, route.get(0)[1], 1e-9);
  }

  @Test
  public void toTheOtherSideItGoesRoundTheNearerEnd() {
    List<double[]> route = SotmFieldTest.route(-1.25, -0.32, 1.25, -0.32);
    assertEquals(3, route.size());
    assertTrue(route.get(0)[1] < 0); // Round the -y end, the nearer one
    assertClearOfTheHive(-1.25, -0.32, route);
    assertEquals(1.25, route.get(2)[0], 1e-9);
  }

  @Test
  public void fromAnEndToASideItTakesOneCorner() {
    List<double[]> route = SotmFieldTest.route(0.0, -1.30, -1.25, 0.5);
    assertEquals(2, route.size());
    assertClearOfTheHive(0.0, -1.30, route);
  }
}
