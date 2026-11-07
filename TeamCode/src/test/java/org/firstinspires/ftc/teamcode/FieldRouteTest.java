package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.List;

import org.firstinspires.ftc.teamcode.utils.Pose;
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

  @Test
  public void reloadCornerIsReachedRoundTheHiveAndTheFenceCheckLetsItIn() {
    double limit = FieldRoute.reloadLimit(-1.50, -1.50);
    assertEquals(1.50 + FieldRoute.RELOAD_MARGIN_M, limit, 1e-12);
    // From where the 7 Nov pass ended, the far side of the Hive from the corner
    List<double[]> route = FieldRoute.route(-1.16, 0.44, -1.50, -1.50);
    double x = -1.16;
    double y = 0.44;
    for (double[] w : route) {
      for (int i = 0; i <= 100; i++) {
        double px = x + (w[0] - x) * i / 100.0;
        double py = y + (w[1] - y) * i / 100.0;
        assertTrue(Math.abs(px) >= FieldRoute.HIVE_HALF_X_M || Math.abs(py) >= FieldRoute.HIVE_HALF_Y_M);
        assertTrue(Math.abs(px) <= limit && Math.abs(py) <= limit);
      }
      x = w[0];
      y = w[1];
    }
    // Slowing in to the spot it is safe; the match's limit would have stopped it
    Pose near = new Pose(-1.42, -1.42, 0);
    double[] slow = {-0.3, -0.3, 0};
    assertTrue(FieldRoute.headedSafely(near, slow, limit));
    double[] cruising = {-0.5, -0.5, 0};
    assertTrue(FieldRoute.headedSafely(new Pose(-1.30, -1.30, 0), cruising, limit));
    assertTrue(!FieldRoute.headedSafely(new Pose(-1.30, -1.30, 0), cruising, FieldRoute.FIELD_LIMIT_M));
    // Fast at the corner it still stops
    assertTrue(!FieldRoute.headedSafely(near, new double[] {-1.0, -1.0, 0}, limit));
  }

  @Test
  public void reloadSpotIsNeverPastTheWall() {
    assertEquals(FieldRoute.MAX_REACH_M, FieldRoute.reloadLimit(-1.66, -1.67), 1e-12); // Panels' corner itself
    assertEquals(FieldRoute.FIELD_LIMIT_M, FieldRoute.reloadLimit(-1.0, -0.3), 1e-12);
  }

  @Test
  public void stuckBoostLiftsTheReloadCap() {
    // 7 Nov, 082948: capped at RELOAD_STICK it sat still on the way to the reload corner and had to be pulled
    Pose at = new Pose(-1.30, -0.65, 0);
    double[] calm = FieldRoute.goToStick(at, -1.5, -1.5, -1.30, -0.60, 0, FieldRoute.RELOAD_STICK);
    assertEquals(FieldRoute.RELOAD_STICK, Math.hypot(calm[0], calm[1]), 1e-9);
    double[] stuck = FieldRoute.goToStick(at, -1.5, -1.5, -1.30, -0.60, 1.0, FieldRoute.RELOAD_STICK);
    assertEquals(FieldRoute.GOTO_STICK, Math.hypot(stuck[0], stuck[1]), 1e-9);
  }
}
