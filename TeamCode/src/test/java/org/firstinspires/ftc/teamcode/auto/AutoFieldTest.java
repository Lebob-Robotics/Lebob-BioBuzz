package org.firstinspires.ftc.teamcode.auto;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.utils.VisionCorrector;
import org.firstinspires.ftc.teamcode.utils.VisionCorrector.Cell;
import org.junit.Test;

public class AutoFieldTest {
  // Robot centre limits round the Hive, as FieldRoute's, plus its route clearance
  private static final double HIVE_X = 0.75 + 0.05;
  private static final double HIVE_Y = 0.95 + 0.05;
  private static final double EDGE = AutoField.AT_WALL_M + 1e-9;

  private static void assertClear(double x, double y, String where) {
    assertTrue(where + " off the field: " + x + ", " + y, Math.abs(x) <= EDGE && Math.abs(y) <= EDGE);
    assertFalse(where + " in the Hive: " + x + ", " + y, Math.abs(x) < HIVE_X && Math.abs(y) < HIVE_Y);
    double[] flower = AutoField.WALL_FLOWER_KEEP_OUT;
    assertFalse(where + " into our wall's Flower: " + x + ", " + y,
        x > flower[0] && x < flower[1] && y < flower[2] - 1e-9);
  }

  private static void assertCurveClear(double[] p0, double[] p1, double[] p2, String where) {
    for (int i = 0; i <= 100; i++) {
      double t = i / 100.0;
      double a = (1 - t) * (1 - t);
      double b = 2 * t * (1 - t);
      double c = t * t;
      assertClear(a * p0[0] + b * p1[0] + c * p2[0], a * p0[1] + b * p1[1] + c * p2[1], where);
    }
  }

  /** The drive to a shooting spot as the auto takes it, straight or bent round the Flower. */
  private static void assertShotDriveClear(double[] from, AutoSequence.Step step, double[] spot, String where) {
    double[] control = AutoField.shotSpotControl(step, spot);
    if (control == null) {
      assertLineClear(from, spot, where);
    } else {
      assertCurveClear(from, control, spot, where);
    }
  }

  private static void assertLineClear(double[] a, double[] b, String where) {
    for (int i = 0; i <= 100; i++) {
      double t = i / 100.0;
      assertClear(a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), where);
    }
  }

  @Test
  public void everyDriveKeepsOnTheFieldAndOutOfTheHive() {
    for (Cell cell : Cell.values()) {
      double[] spot = AutoField.shotSpot(Alliance.RED, cell);
      assertClear(spot[0], spot[1], cell + " shooting spot");
      assertShotDriveClear(AutoField.START, AutoSequence.Step.SHOOT_PRELOAD, spot, "to shoot the preloads at " + cell);
      double[] bend = AutoField.gardenEntryControl(spot);
      if (bend == null) {
        assertLineClear(spot, AutoField.GARDEN_IN, "from " + cell + " to the Garden");
      } else {
        assertCurveClear(spot, bend, AutoField.GARDEN_IN, "from " + cell + " to the Garden, below the Hive");
      }
      assertShotDriveClear(AutoField.GARDEN_OUT, AutoSequence.Step.SHOOT_GARDEN, spot, "from the Garden to " + cell);
      assertLineClear(spot, AutoField.WALL_FLOWER, "from " + cell + " to the wall Flower");
      assertShotDriveClear(AutoField.WALL_FLOWER, AutoSequence.Step.SHOOT_WALL_FLOWER, spot,
          "from the wall Flower to " + cell);
      assertLineClear(spot, AutoField.PARK, "from " + cell + " to park, skipping the rear Flower");
      assertCurveClear(spot, AutoField.REAR_CONTROL, AutoField.REAR_FLOWER, "from " + cell + " to the rear Flower");
    }
    assertLineClear(AutoField.GARDEN_IN, AutoField.GARDEN_WALL, "onto the Garden wall");
    assertLineClear(AutoField.GARDEN_WALL, AutoField.GARDEN_OUT, "sweep");
    assertLineClear(AutoField.GARDEN_OUT, AutoField.WALL_FLOWER, "to the wall Flower");
    assertLineClear(AutoField.REAR_FLOWER, AutoField.REAR_FLOWER_INTAKE, "into the rear Flower");
    assertLineClear(AutoField.REAR_FLOWER, AutoField.PARK, "to park, backed out");
  }

  @Test
  public void theDriveBackFromTheGardenPassesTheFlowerWellClear() {
    for (Cell cell : Cell.values()) {
      double[] spot = AutoField.shotSpot(Alliance.RED, cell);
      double[] c = AutoField.shotSpotControl(AutoSequence.Step.SHOOT_GARDEN, spot);
      for (int i = 0; i <= 100; i++) {
        double t = i / 100.0;
        double x = (1 - t) * (1 - t) * AutoField.GARDEN_OUT[0] + 2 * t * (1 - t) * c[0] + t * t * spot[0];
        double y = (1 - t) * (1 - t) * AutoField.GARDEN_OUT[1] + 2 * t * (1 - t) * c[1] + t * t * spot[1];
        if (Math.abs(x - AutoField.WALL_FLOWER[0]) < 0.1) {
          assertTrue("passes the Flower at " + y + " (hit it at -1.45 on 14 Nov)", y > -1.33);
        }
      }
    }
  }

  @Test
  public void eachShootingSpotIsALittleOffStraightOutBackToItsCell() {
    for (Alliance alliance : Alliance.values()) {
      for (Cell cell : Cell.values()) {
        double[] spot = AutoField.shotSpot(alliance, cell);
        double[] aim = org.firstinspires.ftc.teamcode.utils.VisionCorrector.upCellAimPoint(alliance, cell);
        assertEquals("Volley Tuning's spot 4", 0.92, Math.hypot(aim[0] - spot[0], aim[1] - spot[1]), 0.01);
        assertEquals(1.00, Math.abs(spot[0]), 1e-9);
        assertEquals(1.05, Math.abs(spot[1]), 1e-9);
        assertTrue("away from the middle: toward it the Hive's wall blocks the shot",
            Math.abs(spot[0]) > Math.abs(aim[0]));
        assertTrue("on our side", alliance == Alliance.RED ? spot[1] < aim[1] : spot[1] > aim[1]);
        double back = spot[2] + Math.PI; // The shooter fires out the back
        double toCell = Math.atan2(aim[1] - spot[1], aim[0] - spot[0]);
        assertEquals(alliance + " " + cell + " back to the Cell", 0, AutoField.normalize(back - toCell), 1e-9);
      }
    }
  }


  @Test
  public void theGardenSweepRunsAlongTheAudienceWallAgainstIt() {
    assertEquals(AutoField.FIELD_HALF_M, AutoField.GARDEN_WALL[0] + AutoField.ROBOT_HALF_M, 1e-9);
    assertEquals("parallel to the wall", AutoField.GARDEN_WALL[0], AutoField.GARDEN_OUT[0], 1e-9);
    assertTrue("into the corner", AutoField.GARDEN_OUT[1] < AutoField.GARDEN_WALL[1]);
    for (double[] p : new double[][] {AutoField.GARDEN_IN, AutoField.GARDEN_WALL, AutoField.GARDEN_OUT}) {
      assertEquals("intake first along the line", -Math.PI / 2, p[2], 1e-9);
    }
    // The turn to the sweep's heading happens clear of both walls: the corners reach out the half-diagonal
    double reach = AutoField.ROBOT_HALF_M * Math.sqrt(2);
    assertTrue(AutoField.GARDEN_IN[0] + reach < AutoField.FIELD_HALF_M);
    assertTrue(AutoField.GARDEN_IN[1] - reach > -AutoField.FIELD_HALF_M);
  }

  @Test
  public void eachFlowerIsApproachedStraightOnFromItsStop() {
    assertFlowerApproach(AutoField.WALL_FLOWER, AutoField.WALL_FLOWER_INTAKE);
    assertFlowerApproach(AutoField.REAR_FLOWER, AutoField.REAR_FLOWER_INTAKE);
  }

  /** In along the facing, not sideways, from far enough out for the arm to come down short of the Flower. */
  private static void assertFlowerApproach(double[] stop, double[] in) {
    assertEquals("same facing in and out", stop[2], in[2], 1e-9);
    double dx = in[0] - stop[0];
    double dy = in[1] - stop[1];
    double along = dx * Math.cos(stop[2]) + dy * Math.sin(stop[2]);
    assertEquals("straight in", Math.hypot(dx, dy), along, 1e-9);
    assertTrue("stop too close to the Flower: " + along, along >= 0.2);
  }

  @Test
  public void startsBesideTheGardensPollenNotOnIt() {
    double pollenInnerEdge = AutoField.FIELD_HALF_M - 0.076; // 3 in. balls against the audience wall
    double gap = pollenInnerEdge - (AutoField.START[0] + AutoField.ROBOT_HALF_M);
    assertTrue("gap " + gap, gap > 0 && gap < 0.05);
    assertTrue("clear of our wall's Flower", AutoField.START[0] > AutoField.WALL_FLOWER_KEEP_OUT[1]);
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
    assertEquals(-AutoField.START[0], blue[0], 1e-9);
    assertEquals(AutoField.AT_WALL_M, blue[1], 1e-9);
    assertEquals(Math.PI / 2, blue[2], 1e-9); // Back toward the Hive from the blue wall
    double[] red = AutoField.forAlliance(AutoField.START, Alliance.RED);
    assertEquals(AutoField.START[2], red[2], 1e-9);
  }

  @Test
  public void theSweepDrivesIntoTheCornerAlongTheWall() {
    double[] end = AutoField.gardenSweepEnd();
    assertEquals("along the wall", AutoField.GARDEN_OUT[0], end[0], 1e-9);
    assertTrue("past our wall, so the wall stops it", end[1] < -AutoField.AT_WALL_M);
    assertEquals(AutoField.GARDEN_HEADING, end[2], 1e-9);
  }

  @Test
  public void theAutoAimsALittleTowardTheMiddleAtEitherCell() {
    for (Alliance alliance : Alliance.values()) {
      for (VisionCorrector.Cell cell : VisionCorrector.Cell.values()) {
        double[] cellAim = VisionCorrector.upCellAimPoint(alliance, cell);
        double[] aim = AutoField.autoAimPoint(alliance, cell);
        assertEquals(Math.abs(cellAim[0]) - AutoField.AIM_INWARD_M, Math.abs(aim[0]), 1e-9);
        assertEquals(cellAim[1], aim[1], 1e-9);
      }
    }
  }

  @Test
  public void parksInTheLoadingZoneTouchingOurWallBackToTheCell() {
    for (Alliance alliance : Alliance.values()) {
      for (Cell cell : Cell.values()) {
        double[] p = AutoField.parkSpot(alliance, cell);
        double[] red = alliance == Alliance.RED ? p : new double[] {-p[0], -p[1]};
        double[] zone = AutoField.LOADING_ZONE;
        assertTrue(alliance + " " + cell + " in the loading zone", red[0] > zone[0] && red[0] < zone[1]
            && red[1] > zone[2] && red[1] < zone[3]);
        double reach = AutoField.ROBOT_HALF_M * (Math.abs(Math.sin(p[2])) + Math.abs(Math.cos(p[2])));
        assertEquals(alliance + " " + cell + " corner on the wall", AutoField.FIELD_HALF_M, Math.abs(p[1]) + reach, 1e-3);
        double[] aim = AutoField.autoAimPoint(alliance, cell);
        assertEquals("back to the Cell", Math.atan2(p[1] - aim[1], p[0] - aim[0]), p[2], 1e-3);
      }
    }
  }

  @Test
  public void thePushIsTowardOurWall() {
    for (Alliance alliance : Alliance.values()) {
      for (double h = -Math.PI; h < Math.PI; h += 0.3) {
        double[] push = AutoField.wallPush(alliance, h, 1.0);
        // Robot frame {forward, left} back to the field
        double fieldY = push[0] * Math.sin(h) + push[1] * Math.cos(h);
        double fieldX = push[0] * Math.cos(h) - push[1] * Math.sin(h);
        assertEquals(alliance == Alliance.RED ? -1 : 1, fieldY, 1e-9);
        assertEquals(0, fieldX, 1e-9);
      }
    }
  }
}
