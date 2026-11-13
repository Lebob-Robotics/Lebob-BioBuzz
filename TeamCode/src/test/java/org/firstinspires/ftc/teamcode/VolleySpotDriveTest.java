package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.*;
import org.firstinspires.ftc.teamcode.utils.Pose;
import org.junit.Test;

public class VolleySpotDriveTest {
  @Test public void pointsMirrorLikeShotTuning() {
    assertArrayEquals(new double[]{-1, -0.32}, VolleySpotDrive.point(0, -0.44, -0.32), 1e-9);
    assertArrayEquals(new double[]{1, 0.32}, VolleySpotDrive.point(0, 0.44, 0.32), 1e-9);
    assertEquals(0, VolleySpotDrive.nextSpot(-1, new boolean[10]));
    assertEquals(2, VolleySpotDrive.nextSpot(0, new boolean[]{false, true, false}));
    assertEquals(-1, VolleySpotDrive.nextSpot(0, new boolean[]{true, true}));
  }

  @Test public void arrivalSettlesThenPersistentHoldReturnsAfterNudging() {
    VolleySpotDrive d = new VolleySpotDrive();
    Pose p = new Pose(-1, -0.32, 0);
    assertTrue(d.start(0, -0.44, -0.32, 0, p, true, 1));
    d.update(0, p, new double[3], true, 1);
    assertFalse(d.ready()); assertNull(d.drive());
    d.update(1.1, p, new double[3], true, 2);
    assertTrue(d.ready()); assertArrayEquals(new double[]{-1, -0.32}, d.holdPoint(), 0);
    d.update(1.2, new Pose(-1.15, -0.32, 0), new double[3], true, 3);
    assertFalse(d.ready()); assertNotNull(d.holdPoint()); assertNull(d.drive());
    d.update(1.3, p, new double[3], true, 4);
    d.update(2.4, p, new double[3], true, 5);
    assertTrue(d.ready());
  }

  @Test public void unknownPoseAndStaleTagsCannotMoveOrFire() {
    VolleySpotDrive d = new VolleySpotDrive(); Pose p = new Pose(-1.22, -0.32, 0);
    assertTrue(d.start(0, -0.44, -0.32, 0, p, false, 0));
    assertFalse(d.active());
    assertTrue(d.start(0, -0.44, -0.32, 0, p, true, 1));
    d.update(3.1, p, new double[3], true, 1);
    assertFalse(d.active()); assertFalse(d.ready()); assertNull(d.holdPoint());
    assertArrayEquals(new double[]{0, 0}, d.drive(), 0);
  }

  @Test public void cancelStopsRouteAndClearsHold() {
    VolleySpotDrive d = new VolleySpotDrive(); Pose p = new Pose(-1.22, -0.32, 0);
    d.start(9, -0.44, -0.32, 0, p, true, 1);
    d.update(0.1, p, new double[3], true, 1);
    assertNotNull(d.drive());
    d.stop("Circle cancelled");
    assertFalse(d.shouldAim());
    assertNull(d.holdPoint()); assertFalse(d.active());
    assertArrayEquals(new double[]{0, 0}, d.drive(), 0);
  }
  @Test public void testSpotsHaveAUsableHoodedTableShot() {
    for (int i = 0; i < FieldRoute.GRID.length; i++) {
      double[] p = VolleySpotDrive.point(i, -.44, -.324);
      org.firstinspires.ftc.teamcode.utils.Aim aim = org.firstinspires.ftc.teamcode.utils.ShootOnTheMove.aimHooded(
          new Pose(p[0], p[1], 0), 0, 0, -.44, -.324);
      assertTrue("No hooded shot at spot " + i + ": " + aim.shot.reason, aim.shot.valid);
    }
  }

  @Test public void circleWaitsForVisionThenRoutesWithoutAnotherPress() {
    VolleySpotDrive d = new VolleySpotDrive(); Pose origin = new Pose(0, 0, 0);
    assertTrue(d.start(0, Double.NaN, Double.NaN, 0, origin, false, 0));
    assertEquals(VolleySpotDrive.Stage.ACQUIRING, d.stage());
    d.acquire(0.9, origin, false, 0, -.44, -.324, 1);
    d.update(0.9, origin, new double[3], false, 0);
    assertTrue(d.turn() > 0); assertFalse(d.ready()); assertFalse(d.active());
    assertTrue(d.shouldAim()); // Keep tracking any known Cell while the global pose is still pending.
    assertArrayEquals(new double[]{0, 0}, d.drive(), 0); assertNull(d.holdPoint());
    d.acquire(1.2, origin, false, 0, -.44, -.324, 0);
    assertEquals(0, d.turn(), 0); // Pause with tags visible while vision settles.
    Pose fixed = new Pose(-1.22, -.32, 0);
    d.acquire(2, fixed, true, 1, -.44, -.324, 0);
    assertEquals(VolleySpotDrive.Stage.MOVING, d.stage()); assertEquals(0, d.turn(), 0);
    d.update(2, fixed, new double[3], true, 1);
    assertNotNull(d.drive()); assertFalse(d.ready());
  }

  @Test public void tagSearchTimesOutAndCancelCannotRestartIt() {
    VolleySpotDrive d = new VolleySpotDrive(); Pose p = new Pose(0, 0, 0);
    d.start(0, -.44, -.324, 0, p, false, 0);
    d.acquire(31, p, false, 0, -.44, -.324, 1);
    assertEquals(VolleySpotDrive.Stage.IDLE, d.stage()); assertEquals(0, d.turn(), 0);
    d.start(0, -.44, -.324, 32, p, false, 0);
    d.acquire(32.9, p, false, 0, -.44, -.324, 1);
    assertTrue(d.turn() > 0);
    d.stop("Circle cancelled");
    d.acquire(33, new Pose(-1.22, -.32, 0), true, 1, -.44, -.324, 0);
    assertEquals(VolleySpotDrive.Stage.IDLE, d.stage()); assertEquals(0, d.turn(), 0);
    assertNull(d.holdPoint()); assertFalse(d.ready());
  }

  @Test public void holdingSpotSurvivesTagGapAndRbCanStartTheTrial() {
    VolleySpotDrive d = new VolleySpotDrive(); Pose p = new Pose(1, .32, 0);
    d.start(0, .44, .324, 0, p, true, 1);
    d.update(0, p, new double[3], true, 1);
    d.update(1.1, p, new double[3], true, 1);
    assertTrue(d.ready());
    d.update(73, p, new double[3], true, 1);
    assertTrue(d.ready()); assertNotNull(d.holdPoint());
    org.firstinspires.ftc.teamcode.utils.VolleyTrial trial = new org.firstinspires.ftc.teamcode.utils.VolleyTrial();
    trial.input(84, false, d.ready(), false);
    trial.input(85, true, d.ready(), false);
    assertTrue(trial.active());
  }

  @Test public void blindHoldReturnsFromSmallLoadingNudgeButStopsIfPushedFar() {
    VolleySpotDrive d = new VolleySpotDrive(); Pose p = new Pose(1, .32, 0);
    d.start(0, .44, .324, 0, p, true, 1);
    d.update(0, p, new double[3], true, 1);
    d.update(1.1, p, new double[3], true, 1);
    d.update(10, new Pose(1, .42, 0), new double[3], true, 1);
    assertEquals(VolleySpotDrive.Stage.SETTLING, d.stage());
    assertNotNull(d.holdPoint()); assertNull(d.drive()); assertFalse(d.ready());
    d.update(11, p, new double[3], true, 1);
    d.update(12.1, p, new double[3], true, 1);
    assertTrue(d.ready());
    d.update(13, new Pose(1, .62, 0), new double[3], true, 1);
    assertFalse(d.active()); assertFalse(d.ready()); assertNull(d.holdPoint());
  }

  @Test public void invalidPoseStillStopsEvenAParkedRobot() {
    VolleySpotDrive d = new VolleySpotDrive(); Pose p = new Pose(1, .32, 0);
    d.start(0, .44, .324, 0, p, true, 1);
    d.update(0, p, new double[3], true, 1);
    d.update(1.1, p, new double[3], true, 1);
    d.update(2, p, new double[3], false, 1);
    assertFalse(d.active()); assertNull(d.holdPoint()); assertFalse(d.ready());
  }

  @Test public void recordedEdgePosesCanDriveInwardToTheFirstSpot() {
    for (double[] start : new double[][] {{-1.5280515, -.372644}, {-1.578137, -.361713}}) {
      VolleySpotDrive d = new VolleySpotDrive(); Pose p = new Pose(start[0], start[1], 0);
      assertTrue(d.status(), d.start(0, -.44, .32385, 0, p, true, 1));
      d.update(.1, p, new double[3], true, 1);
      assertEquals(VolleySpotDrive.Stage.MOVING, d.stage());
      assertTrue("Entry must move inward", d.drive()[0] > 0);
      assertFalse(d.ready());
    }
  }

}
