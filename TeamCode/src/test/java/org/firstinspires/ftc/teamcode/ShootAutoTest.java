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
  public void parkPushesWhenStuckCloseAndGoesFullFromFar() {
    double[] stuck = ShootAuto.parkStick(0.15, 0, 0, 0, 0);
    assertTrue(Math.hypot(stuck[0], stuck[1]) >= 0.45); // 193927 sat 0.15 m short at stick 0.22
    double[] far = ShootAuto.parkStick(2.0, 0, 0, 0, 0);
    assertEquals(1.0, far[0], 1e-9);
  }

  @Test
  public void parkBrakesBeforeTheGoal() {
    double[] eases = ShootAuto.parkStick(0.3, 0, 0, 1.6, 0); // Full speed 0.3 m out: off the throttle
    assertTrue("eases off, got " + eases[0], eases[0] < 0.25);
    double[] brakes = ShootAuto.parkStick(0.2, 0, 0, 1.6, 0); // 0.2 m out: reverses
    assertTrue("brakes, got " + brakes[0], brakes[0] < 0);
  }

  @Test
  public void parkStopsInTheZoneInsteadOfRunningPastIt() {
    // 13 Nov 215133: red, shot from (1.56, -0.74); full stick ran to x -1.354, 0.44 m past the zone's middle.
    // Robot model from that log: 1.6 m/s top speed, 0.3 s lag, 3.4 m/s^2 grip, 0.1 s loop, velocity a loop old
    double f = ShootOnTheMoveConstants.ALLIANCE_FORWARD_RED_RAD;
    double[][] route = ShootAuto.parkRoute(f, 1.56, -0.74);
    double x = 1.56, y = -0.74, vx = 0, vy = 0, seenVx = 0, seenVy = 0, minX = x;
    int leg = 0;
    double t = 0;
    for (; t < ShootAuto.PARK_TIMEOUT_S; t += 0.1) {
      double[] goal = route[leg];
      boolean last = leg == route.length - 1;
      if (!last && Math.hypot(goal[0] - x, goal[1] - y) <= ShootAuto.PARK_WAYPOINT_M) {
        goal = route[++leg];
        last = leg == route.length - 1;
      }
      if (last && Math.hypot(goal[0] - x, goal[1] - y) <= ShootAuto.PARK_DONE_M) {
        break;
      }
      double[] s = ShootAuto.parkStick(goal[0] - x, goal[1] - y, last ? 0 : ShootAuto.PARK_CORNER_MPS, seenVx, seenVy);
      seenVx = vx;
      seenVy = vy;
      for (int i = 0; i < 10; i++) {
        double ax = (s[0] * 1.6 - vx) / 0.3, ay = (s[1] * 1.6 - vy) / 0.3, a = Math.hypot(ax, ay);
        if (a > 3.4) {
          ax *= 3.4 / a;
          ay *= 3.4 / a;
        }
        vx += ax * 0.01;
        vy += ay * 0.01;
        x += vx * 0.01;
        y += vy * 0.01;
      }
      minX = Math.min(minX, x);
    }
    assertTrue("timed out", t < ShootAuto.PARK_TIMEOUT_S);
    assertTrue("ran to x " + minX, minX > -1.22 + 0.10); // Robot centre: never more than half off the zone's far edge
    assertEquals(-0.915, x, ShootAuto.PARK_DONE_M);
    assertEquals(-1.52, y, ShootAuto.PARK_DONE_M);
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
