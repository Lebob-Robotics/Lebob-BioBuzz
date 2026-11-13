package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.utils.ShootOnTheMoveConstants;
import org.junit.Test;

public class ShootAutoTest {
  @Test
  public void parksWhollyInTheManualsLoadingZone() {
    double half = 0.23; // Robot centre to bumper
    double[] red = ShootAuto.parkPoint(ShootOnTheMoveConstants.ALLIANCE_FORWARD_RED_RAD);
    assertEquals(-0.915, red[0], 1e-9); // Tile A5: x -1.22 to -0.61
    assertEquals(-1.52, red[1], 1e-9);
    assertTrue(red[0] - half >= -1.22 && red[0] + half <= -0.61);
    assertTrue(red[1] - half >= -ShootOnTheMoveConstants.FIELD_HALF_SIZE_M && red[1] + half <= -1.22);
    double[] blue = ShootAuto.parkPoint(ShootOnTheMoveConstants.ALLIANCE_FORWARD_BLUE_RAD);
    assertEquals(0.915, blue[0], 1e-9); // Red's turned half a turn
    assertEquals(1.52, blue[1], 1e-9);
  }

  @Test
  public void parkRouteKeepsClearOfTheHiveAndTheWallFlower() {
    // 13 Nov 193927: blue shot from about (-1.45, 0.95)
    double[][] route = ShootAuto.parkRoute(ShootOnTheMoveConstants.ALLIANCE_FORWARD_BLUE_RAD, -1.45, 0.95);
    assertEquals(3, route.length);
    double[] from = {-1.45, 0.95};
    for (double[] to : route) {
      for (int i = 0; i <= 100; i++) {
        double t = i / 100.0;
        double x = from[0] + t * (to[0] - from[0]);
        double y = from[1] + t * (to[1] - from[1]);
        assertFalse("in the Hive at " + x + ", " + y, Math.abs(x) < 0.80 && Math.abs(y) < 1.00);
        // Our wall's Flower: red's at x +0.61 by its wall (feature/auto's AutoField), so blue's at -0.61
        assertFalse("into the Flower at " + x + ", " + y, Math.abs(x + 0.61) < 0.35 && y > 1.30);
      }
      from = to;
    }
    assertEquals(0.915, route[2][0], 1e-9);
    assertEquals(1.52, route[2][1], 1e-9);
  }

  @Test
  public void startsInTheCornerOnTheDriversRight() {
    double[] blue = ShootAuto.cornerPoint(ShootOnTheMoveConstants.ALLIANCE_FORWARD_BLUE_RAD, 0.23, 0.23);
    assertEquals(-1.60, blue[0], 1e-9); // 13 Nov 191356: the tags placed a blue start near (-1.46, 1.38)
    assertEquals(1.60, blue[1], 1e-9);
  }

  @Test
  public void parkKeepsEnoughPowerToMoveWhenClose() {
    assertEquals(ShootAuto.PARK_MIN_POWER, ShootAuto.parkStick(0.15), 1e-9); // 193927 stalled here at 0.22
    assertEquals(ShootAuto.PARK_POWER, ShootAuto.parkStick(2.0), 1e-9);
  }

  @Test
  public void driveDistanceLeavesOutTheVisionJump() {
    double d = ShootAuto.drivenStep(Double.NaN, Double.NaN, 0, 0);
    d += ShootAuto.drivenStep(0, 0, -1.46, 1.38); // The tags place the robot
    d += ShootAuto.drivenStep(-1.46, 1.38, -1.46, 1.33);
    d += ShootAuto.drivenStep(-1.46, 1.33, -1.46, 1.28);
    assertEquals(0.10, d, 1e-9);
  }
}
