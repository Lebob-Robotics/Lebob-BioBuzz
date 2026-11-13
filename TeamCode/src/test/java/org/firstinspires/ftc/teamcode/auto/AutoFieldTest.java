package org.firstinspires.ftc.teamcode.auto;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.Alliance;
import org.junit.Test;

public class AutoFieldTest {
  // Robot centre limits round the Hive, as FieldRoute's, plus its route clearance
  private static final double HIVE_X = 0.75 + 0.05;
  private static final double HIVE_Y = 0.95 + 0.05;
  private static final double EDGE = AutoField.AT_WALL_M + 1e-9;

  private static void assertClear(double x, double y, String where) {
    assertTrue(where + " off the field: " + x + ", " + y, Math.abs(x) <= EDGE && Math.abs(y) <= EDGE);
    assertFalse(where + " in the Hive: " + x + ", " + y, Math.abs(x) < HIVE_X && Math.abs(y) < HIVE_Y);
  }

  private static void assertLineClear(double[] a, double[] b, String where) {
    for (int i = 0; i <= 100; i++) {
      double t = i / 100.0;
      assertClear(a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), where);
    }
  }

  @Test
  public void everyDriveKeepsOnTheFieldAndOutOfTheHive() {
    assertLineClear(AutoField.START, AutoField.GARDEN_IN, "to the Garden");
    assertLineClear(AutoField.GARDEN_IN, AutoField.GARDEN_OUT, "sweep");
    assertLineClear(AutoField.GARDEN_OUT, AutoField.WALL_FLOWER, "to the wall Flower");
    assertLineClear(AutoField.WALL_FLOWER, AutoField.WALL_FLOWER_INTAKE, "into the wall Flower");
    assertLineClear(AutoField.WALL_FLOWER_INTAKE, AutoField.PARK, "skipping to park");
    assertLineClear(AutoField.REAR_FLOWER, AutoField.REAR_FLOWER_INTAKE, "into the rear Flower");
    assertLineClear(AutoField.REAR_FLOWER_INTAKE, AutoField.PARK, "to park");
    double[] p0 = AutoField.WALL_FLOWER_INTAKE;
    double[] p1 = AutoField.REAR_CONTROL;
    double[] p2 = AutoField.REAR_FLOWER;
    for (int i = 0; i <= 100; i++) {
      double t = i / 100.0;
      double a = (1 - t) * (1 - t);
      double b = 2 * t * (1 - t);
      double c = t * t;
      assertClear(a * p0[0] + b * p1[0] + c * p2[0], a * p0[1] + b * p1[1] + c * p2[1], "to the rear Flower");
    }
  }

  @Test
  public void startsOutsideTheLoadingZoneAndParksWhollyInIt() {
    double[] zone = AutoField.LOADING_ZONE;
    double half = AutoField.ROBOT_HALF_M;
    double[] s = AutoField.START;
    assertTrue("start overlaps the loading zone", s[0] - half > zone[1] || s[0] + half < zone[0]);
    assertEquals("start not touching the wall", -AutoField.FIELD_HALF_M, s[1] - half, 1e-9);
    double[] p = AutoField.PARK;
    assertTrue(p[0] - half >= zone[0] && p[0] + half <= zone[1]);
    assertTrue(p[1] - half >= zone[2] && p[1] + half <= zone[3]);
  }

  @Test
  public void blueIsRedTurnedHalfATurn() {
    double[] blue = AutoField.forAlliance(AutoField.START, Alliance.BLUE);
    assertEquals(0.30, blue[0], 1e-9);
    assertEquals(AutoField.AT_WALL_M, blue[1], 1e-9);
    assertEquals(Math.PI / 2, blue[2], 1e-9); // Back toward the Hive from the blue wall
    double[] red = AutoField.forAlliance(AutoField.START, Alliance.RED);
    assertEquals(AutoField.START[2], red[2], 1e-9);
  }
}
