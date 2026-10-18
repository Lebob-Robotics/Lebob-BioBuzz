package org.firstinspires.ftc.teamcode.utils;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class TargetTrackerTest {
  private static final double TOLERANCE = 1e-9;
  private static final long ONE_SECOND_NANOS = 1_000_000_000L;
  private static final CameraMount LEVEL_FORWARD_AT_CELL_HEIGHT = new CameraMount(0, 0, 1.26, 0, 0);

  private static CellSighting sighting(CameraMount camera, long frameTimeNanos, double rightM, double forwardM, double upM) {
    return new CellSighting("RED SCORING", camera, frameTimeNanos, rightM, forwardM, upM);
  }

  private static CellSighting twoMetresAhead(long frameTimeNanos) {
    return sighting(LEVEL_FORWARD_AT_CELL_HEIGHT, frameTimeNanos, 0, 2, 0);
  }

  @Test
  public void nothingHappensBeforeAPoseIsRecorded() {
    TargetTracker tracker = new TargetTracker(10);
    assertFalse(tracker.update(twoMetresAhead(0)));
    assertFalse(tracker.knowsCell(0));
  }

  @Test
  public void placesTheCellAheadOfTheRobot() {
    TargetTracker tracker = new TargetTracker(10);
    tracker.recordPose(0, new Pose(1, 1, 0));
    assertTrue(tracker.update(twoMetresAhead(0)));
    assertEquals(3, tracker.getCellX(), TOLERANCE);
    assertEquals(1, tracker.getCellY(), TOLERANCE);
  }

  @Test
  public void cameraRightIsRobotRight() {
    TargetTracker tracker = new TargetTracker(10);
    tracker.recordPose(0, new Pose(0, 0, 0));
    tracker.update(sighting(LEVEL_FORWARD_AT_CELL_HEIGHT, 0, 0.5, 2, 0));
    assertEquals(-0.5, tracker.getCellY(), TOLERANCE);
  }

  @Test
  public void usesThePoseFromWhenTheFrameWasTaken() {
    TargetTracker tracker = new TargetTracker(10);
    tracker.recordPose(0, new Pose(0, 0, 0));
    tracker.recordPose(ONE_SECOND_NANOS / 10, new Pose(1, 0, 0));
    tracker.recordPose(ONE_SECOND_NANOS / 5, new Pose(2, 0, 0));
    tracker.update(twoMetresAhead(ONE_SECOND_NANOS / 10 + 1000));
    assertEquals(3, tracker.getCellX(), TOLERANCE);
  }

  @Test
  public void appliesTheRobotHeading() {
    TargetTracker tracker = new TargetTracker(10);
    tracker.recordPose(0, new Pose(0, 0, Math.PI / 2));
    tracker.update(twoMetresAhead(0));
    assertEquals(0, tracker.getCellX(), TOLERANCE);
    assertEquals(2, tracker.getCellY(), TOLERANCE);
  }

  @Test
  public void appliesTheCameraPositionAndYaw() {
    CameraMount backward = new CameraMount(-0.1, 0, 1.26, Math.PI, 0); // 0.1 m behind centre, so a Cell 2 m out is 2.1 m behind
    TargetTracker tracker = new TargetTracker(10);
    tracker.recordPose(0, new Pose(0, 0, 0));
    tracker.update(sighting(backward, 0, 0, 2, 0));
    assertEquals(-2.1, tracker.getCellX(), TOLERANCE);
    assertEquals(0, tracker.getCellY(), TOLERANCE);
  }

  @Test
  public void undoesTheCameraTilt() {
    CameraMount tiltedUp30 = new CameraMount(0, 0, 0.2, 0, Math.toRadians(30));
    TargetTracker tracker = new TargetTracker(10);
    tracker.recordPose(0, new Pose(0, 0, 0));
    assertTrue(tracker.update(sighting(tiltedUp30, 0, 0, 2, 0)));
    assertEquals(2 * Math.cos(Math.toRadians(30)), tracker.getCellX(), TOLERANCE); // 2 m along the tilted lens
  }

  @Test
  public void ignoresTheDownFacingCell() {
    CameraMount atDownCellHeight = new CameraMount(0, 0, 0.90, 0, 0);
    TargetTracker tracker = new TargetTracker(10);
    tracker.recordPose(0, new Pose(0, 0, 0));
    assertFalse(tracker.update(sighting(atDownCellHeight, 0, 0, 2, 0)));
    assertFalse(tracker.knowsCell(0));
  }

  @Test
  public void laterSightingsAreSmoothed() {
    TargetTracker tracker = new TargetTracker(10);
    tracker.recordPose(0, new Pose(0, 0, 0));
    tracker.update(twoMetresAhead(0));
    tracker.update(sighting(LEVEL_FORWARD_AT_CELL_HEIGHT, 1, 0, 3, 0));
    assertEquals(2 + ShootOnTheMoveConstants.CELL_SMOOTHING, tracker.getCellX(), TOLERANCE);
  }

  @Test
  public void forgetsTheCellAfterTheExpiryTime() {
    TargetTracker tracker = new TargetTracker(10);
    tracker.recordPose(0, new Pose(0, 0, 0));
    tracker.update(twoMetresAhead(0));
    long expiryNanos = (long) (ShootOnTheMoveConstants.CELL_EXPIRY_S * ONE_SECOND_NANOS);
    assertTrue(tracker.knowsCell(expiryNanos));
    assertFalse(tracker.knowsCell(expiryNanos + 1000));
  }

  @Test
  public void clearForgetsTheCellAndThePoses() {
    TargetTracker tracker = new TargetTracker(10);
    tracker.recordPose(0, new Pose(0, 0, 0));
    tracker.update(twoMetresAhead(0));
    tracker.clear();
    assertFalse(tracker.knowsCell(0));
    assertFalse(tracker.update(twoMetresAhead(0)));
  }

  @Test
  public void keepsOnlyTheNewestPoses() {
    TargetTracker tracker = new TargetTracker(2);
    tracker.recordPose(0, new Pose(5, 0, 0));
    tracker.recordPose(10, new Pose(1, 0, 0));
    tracker.recordPose(20, new Pose(2, 0, 0));
    tracker.update(twoMetresAhead(0)); // The time 0 pose was dropped, so the time 10 one is nearest
    assertEquals(3, tracker.getCellX(), TOLERANCE);
  }
}
