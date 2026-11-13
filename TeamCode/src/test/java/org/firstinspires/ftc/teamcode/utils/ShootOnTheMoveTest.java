package org.firstinspires.ftc.teamcode.utils;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

public class ShootOnTheMoveTest {
  private static final double TOLERANCE = 1e-9;

  private double savedShooterForwardM; // The constants are live-tunable statics, so tests put them back
  private double savedShooterLeftM;
  private double savedFeedDelayS;
  private double savedHoodTopStopDeg;

  @Before
  public void zeroShooterOffsetAndDelay() {
    savedHoodTopStopDeg = ShootOnTheMoveConstants.HOOD_TOP_STOP_DEG;
    ShootOnTheMoveConstants.HOOD_TOP_STOP_DEG = 51; // The table's whole range, whatever the robot's stop is now
    savedShooterForwardM = ShootOnTheMoveConstants.SHOOTER_FORWARD_M;
    savedShooterLeftM = ShootOnTheMoveConstants.SHOOTER_LEFT_M;
    savedFeedDelayS = ShootOnTheMoveConstants.FEED_DELAY_S;
    ShootOnTheMoveConstants.SHOOTER_FORWARD_M = 0;
    ShootOnTheMoveConstants.SHOOTER_LEFT_M = 0;
    ShootOnTheMoveConstants.FEED_DELAY_S = 0;
  }

  @After
  public void restoreConstants() {
    ShootOnTheMoveConstants.SHOOTER_FORWARD_M = savedShooterForwardM;
    ShootOnTheMoveConstants.SHOOTER_LEFT_M = savedShooterLeftM;
    ShootOnTheMoveConstants.FEED_DELAY_S = savedFeedDelayS;
    ShootOnTheMoveConstants.HOOD_TOP_STOP_DEG = savedHoodTopStopDeg;
  }

  @Test
  public void aimVelocityAveragesOutLoopNoiseButFollowsASteadyChange() {
    // 7 Nov, 100030: sideways noise of a few cm/s swung the aim heading 2.4-3.4 degrees rms
    double[] v = ShootOnTheMove.smoothAimVelocity(null, new double[] {1.0, 0.0}, 0.07, 0.12);
    assertEquals(1.0, v[0], 1e-12); // First sample taken whole
    double maxDev = 0;
    for (int i = 0; i < 40; i++) {
      double noise = (i % 2 == 0 ? 0.1 : -0.1);
      v = ShootOnTheMove.smoothAimVelocity(v, new double[] {1.0, noise}, 0.07, 0.12);
      maxDev = Math.max(maxDev, Math.abs(v[1]));
    }
    assertTrue("alternating 0.1 m/s noise smoothed to " + maxDev, maxDev < 0.05);
    for (int i = 0; i < 20; i++) { // 1.4 s at the new speed: settled
      v = ShootOnTheMove.smoothAimVelocity(v, new double[] {0.0, 0.5}, 0.07, 0.12);
    }
    assertEquals(0.5, v[1], 0.01);
    assertEquals(0.0, v[0], 0.01);
    // Off, or after a gap, it is the raw velocity
    assertEquals(0.3, ShootOnTheMove.smoothAimVelocity(v, new double[] {0.3, 0}, 0.07, 0)[0], 1e-12);
    assertEquals(0.3, ShootOnTheMove.smoothAimVelocity(v, new double[] {0.3, 0}, 2.0, 0.12)[0], 1e-12);
  }

  @Test
  public void aShotFlatterThanTheHoodCanReachIsNoShot() {
    Shot far = ShootOnTheMove.calculate(1.0, 0, 0);
    Shot near = ShootOnTheMove.calculate(0.4, 0, 0);
    assertTrue(far.valid);
    assertTrue(near.hoodDeg > far.hoodDeg + 2); // Nearer is steeper
    ShootOnTheMoveConstants.HOOD_TOP_STOP_DEG = far.hoodDeg; // Now the far shot is inside the stop's margin
    Shot blocked = ShootOnTheMove.calculate(1.0, 0, 0);
    assertFalse(blocked.valid);
    assertEquals(Shot.Reason.HOOD_OUT_OF_RANGE, blocked.reason);
    assertTrue(ShootOnTheMove.calculate(0.4, 0, 0).valid);
  }

  private static int indexOf(double[] axis, double value) {
    for (int i = 0; i < axis.length; i++) {
      if (Math.abs(axis[i] - value) < 1e-6) {
        return i;
      }
    }
    throw new IllegalArgumentException(value + " is not on the table's axis");
  }

  // ---- calculate ---------------------------------------------------------

  @Test
  public void standingStillAtAGridPointGivesTheTableValues() {
    int distIndex = indexOf(ShotTable.DIST_M, 1.0);
    int speedIndex = indexOf(ShotTable.VEL_MPS, 0.0);
    Shot shot = ShootOnTheMove.calculate(1.0, 0, 0);
    assertTrue(shot.valid);
    assertEquals(ShotTable.HOOD_DEG[distIndex][speedIndex], shot.hoodDeg, TOLERANCE);
    assertEquals(ShotTable.RPM[distIndex][speedIndex], shot.targetRpm, TOLERANCE);
    assertEquals(ShotTable.TOF_S[distIndex][speedIndex], shot.timeOfFlightS, TOLERANCE);
    assertEquals(0, shot.turningRad, TOLERANCE);
  }

  @Test
  public void halfwayBetweenSimilarCellsBlendsThem() {
    int still = indexOf(ShotTable.VEL_MPS, 0.0);
    for (int dist = 0; dist + 1 < ShotTable.DIST_M.length; dist++) {
      double nearHood = ShotTable.HOOD_DEG[dist][still];
      double farHood = ShotTable.HOOD_DEG[dist + 1][still];
      boolean blendable = Math.abs(nearHood - farHood) <= ShootOnTheMoveConstants.MAX_HOOD_DIFFERENCE_TO_BLEND_DEG;
      if (!blendable) { // Also false when either is NaN
        continue;
      }
      double halfway = (ShotTable.DIST_M[dist] + ShotTable.DIST_M[dist + 1]) / 2;
      Shot shot = ShootOnTheMove.calculate(halfway, 0, 0);
      assertEquals((nearHood + farHood) / 2, shot.hoodDeg, 1e-6);
      assertEquals((ShotTable.RPM[dist][still] + ShotTable.RPM[dist + 1][still]) / 2, shot.targetRpm, 1e-6);
      return;
    }
    throw new AssertionError("no blendable neighbours in the table");
  }

  @Test
  public void tooCloseOrTooFarIsOutOfRange() {
    assertEquals(Shot.Reason.OUT_OF_RANGE, ShootOnTheMove.calculate(0.05, 0, 0).reason);
    assertEquals(Shot.Reason.OUT_OF_RANGE, ShootOnTheMove.calculate(5.0, 0, 0).reason);
  }

  @Test
  public void tooFastTowardOrAwayHasItsOwnReason() {
    assertEquals(Shot.Reason.CLOSING_TOO_FAST, ShootOnTheMove.calculate(1.0, 3.0, 0).reason);
    assertEquals(Shot.Reason.BACKING_TOO_FAST, ShootOnTheMove.calculate(1.0, -3.0, 0).reason);
  }

  @Test
  public void anEmptyCellNearbyMeansNoSafeShot() {
    for (int dist = 0; dist < ShotTable.DIST_M.length; dist++) {
      for (int speed = 0; speed < ShotTable.VEL_MPS.length; speed++) {
        if (Double.isNaN(ShotTable.RPM[dist][speed])) {
          Shot shot = ShootOnTheMove.calculate(ShotTable.DIST_M[dist], ShotTable.VEL_MPS[speed], 0);
          assertFalse(shot.valid);
          assertEquals(Shot.Reason.NO_SAFE_SHOT, shot.reason);
          assertTrue(Double.isNaN(shot.targetRpm));
          return;
        }
      }
    }
    throw new AssertionError("the table has no empty cells to test with");
  }

  @Test
  public void movingLeftTurnsTheShooterRight() {
    Shot shot = ShootOnTheMove.calculate(1.0, 0, 0.5);
    assertTrue(shot.valid);
    assertTrue(shot.hoodDeg < 90);
    assertTrue("should turn clockwise", shot.turningRad < 0);
    double ballHorizontalSpeed = shot.targetRpm * ShotTable.EXIT_SPEED_PER_RPM * Math.cos(Math.toRadians(shot.hoodDeg));
    assertEquals(-0.5, ballHorizontalSpeed * Math.sin(shot.turningRad), TOLERANCE); // Cancels the robot's sideways speed
  }

  @Test
  public void tooMuchSidewaysSpeedIsRejected() {
    assertEquals(Shot.Reason.SIDEWAYS_TOO_FAST, ShootOnTheMove.calculate(1.0, 0, 10).reason);
  }

  @Test
  public void withoutAShotTheTurnStaysAtItsLimitInsteadOfDroppingToZero() {
    double max = Math.toRadians(ShootOnTheMoveConstants.MAX_TURNING_DEG);
    assertEquals(-max, ShootOnTheMove.calculate(1.0, 0, 10).turningRad, 1e-9); // Moving left: turn clockwise
    Shot tooClose = ShootOnTheMove.calculate(0.05, 0, 0.5);
    assertEquals(Shot.Reason.OUT_OF_RANGE, tooClose.reason);
    assertTrue(tooClose.turningRad < 0);
    assertEquals(ShootOnTheMove.calculate(ShotTable.DIST_M[0], 0, 0.5).turningRad, tooClose.turningRad, 1e-9);
  }

  @Test
  public void notReadyWhileTheSidewaysTurnIsSwinging() {
    Aim aim = ShootOnTheMove.aim(new Pose(0, 0, 0), 0, 0, 1.5, 0);
    double swing = (aim.headingToleranceRad + Math.toRadians(1)) / ShootOnTheMoveConstants.FEED_DELAY_MAX_S;
    assertTrue(ShootOnTheMove.readyToFire(aim, true, true, aim.lockHeadingRad, 0, 0));
    assertFalse(ShootOnTheMove.readyToFire(aim, true, true, aim.lockHeadingRad, 0, swing));
    assertTrue(ShootOnTheMove.readyToFire(aim, true, true, aim.lockHeadingRad, swing, swing)); // Turning with it
  }

  // ---- aim ---------------------------------------------------------------

  @Test
  public void standingStillTheBackOfTheRobotFacesTheCell() {
    Aim aim = ShootOnTheMove.aim(new Pose(0, 0, 0), 0, 0, 1.5, 0); // Cell along +x, shooter fires out the back
    assertTrue(aim.shot.valid);
    assertEquals(0, ShootOnTheMove.normalizeAngle(aim.targetHeadingRad - Math.PI), TOLERANCE);
  }

  @Test
  public void distanceIsMeasuredToTheNearLip() {
    Aim aim = ShootOnTheMove.aim(new Pose(0, 0, 0), 0, 0, 1.5, 0);
    assertEquals(1.5 - ShootOnTheMoveConstants.CLUSTER_TO_NEAR_LIP_M, aim.shot.distanceToLipM, TOLERANCE);
  }

  @Test
  public void shooterOffsetMovesWhereTheBallLeavesFrom() {
    ShootOnTheMoveConstants.SHOOTER_FORWARD_M = 0.1; // Robot faces -x, so this is 0.1 m further from a Cell at +x
    Aim aim = ShootOnTheMove.aim(new Pose(0, 0, Math.PI), 0, 0, 1.5, 0);
    assertEquals(1.6 - ShootOnTheMoveConstants.CLUSTER_TO_NEAR_LIP_M, aim.shot.distanceToLipM, TOLERANCE);
  }

  @Test
  public void feedDelayMovesTheRobotForward() {
    ShootOnTheMoveConstants.FEED_DELAY_S = 0.2;
    Aim aim = ShootOnTheMove.aim(new Pose(0, 0, 0), 0.5, 0, 1.5, 0);
    assertEquals(1.4 - ShootOnTheMoveConstants.CLUSTER_TO_NEAR_LIP_M, aim.shot.distanceToLipM, TOLERANCE);
  }

  @Test
  public void velocityIsSplitIntoTowardAndSideways() {
    Aim aim = ShootOnTheMove.aim(new Pose(0, 0, 0), -0.3, 0.4, 0, 1.5); // Cell along +y, so -x is to the left
    assertEquals(0.4, aim.shot.speedTowardCellMps, TOLERANCE);
    assertEquals(0.3, aim.shot.sidewaysSpeedMps, TOLERANCE);
  }

  @Test
  public void turningIsAddedToTheHeading() {
    Aim standing = ShootOnTheMove.aim(new Pose(0, 0, 0), 0, 0, 1.5, 0);
    Aim strafing = ShootOnTheMove.aim(new Pose(0, 0, 0), 0, 0.5, 1.5, 0);
    assertTrue(strafing.shot.valid);
    double headingChange = ShootOnTheMove.normalizeAngle(strafing.targetHeadingRad - standing.targetHeadingRad);
    assertEquals(strafing.shot.turningRad, headingChange, TOLERANCE);
  }

  @Test
  public void holdsTheShotForNowAndItsRateToTheRelease() {
    ShootOnTheMoveConstants.FEED_DELAY_S = 0.2;
    Aim aim = ShootOnTheMove.aim(new Pose(0, 0, 0), 0.4, 0, 2.0, 0);
    // The robot at release, 0.2 s on, standing at where it is then with the same velocity
    Aim atRelease = ShootOnTheMove.aim(new Pose(0.08, 0, 0), 0.4, 0, 2.0, 0);
    assertTrue(aim.shot.valid && atRelease.shot.valid);
    assertEquals(atRelease.hoodNowDeg, aim.shot.hoodDeg, 1e-6); // Following the now values arrives at the shot
    assertEquals(atRelease.rpmNow, aim.shot.targetRpm, 1e-6);
    assertEquals((aim.shot.hoodDeg - aim.hoodNowDeg) / 0.2, aim.hoodRateDegPerSec, 1e-6);
    assertEquals((aim.shot.targetRpm - aim.rpmNow) / 0.2, aim.rpmRatePerSec, 1e-6);
    assertTrue(aim.rpmRatePerSec < 0); // Closer needs less speed

    Aim still = ShootOnTheMove.aim(new Pose(0, 0, 0), 0, 0, 2.0, 0);
    assertEquals(0, still.hoodRateDegPerSec, 0);
    assertEquals(0, still.rpmRatePerSec, 0);
  }

  // ---- readyToFire and helpers -------------------------------------------

  @Test
  public void readyNeedsEverythingOnTarget() {
    Aim aim = ShootOnTheMove.aim(new Pose(0, 0, 0), 0, 0, 1.5, 0);
    double onHeading = aim.targetHeadingRad;
    double offHeading = aim.targetHeadingRad + aim.headingToleranceRad * 2;
    assertTrue(ShootOnTheMove.readyToFire(aim, true, true, onHeading, 0));
    assertFalse(ShootOnTheMove.readyToFire(aim, false, true, onHeading, 0));
    assertFalse(ShootOnTheMove.readyToFire(aim, true, false, onHeading, 0));
    assertFalse(ShootOnTheMove.readyToFire(aim, true, true, offHeading, 0));
  }

  @Test
  public void noMovingShotCloseToTheLip() {
    double cellX = ShootOnTheMoveConstants.MIN_MOVING_SHOT_LIP_M - 0.1 + ShootOnTheMoveConstants.CLUSTER_TO_NEAR_LIP_M;
    Aim standing = ShootOnTheMove.aim(new Pose(0, 0, 0), 0, 0, cellX, 0);
    Aim strafing = ShootOnTheMove.aim(new Pose(0, 0, 0), 0, 0.5, cellX, 0);
    assertTrue(standing.shot.valid && strafing.shot.valid);
    assertTrue(ShootOnTheMove.readyToFire(standing, true, true, standing.lockHeadingRad, 0));
    assertFalse(ShootOnTheMove.readyToFire(strafing, true, true, strafing.lockHeadingRad, strafing.lockRateRadPerSec));
    // SOTM Tuning fires it, to see whether it scores
    assertTrue(ShootOnTheMove.readyToFire(strafing, true, true, strafing.lockHeadingRad, strafing.lockRateRadPerSec,
        0, true));
  }

  @Test
  public void notReadyWhileTheHoodTargetSwingsFast() {
    ShootOnTheMoveConstants.FEED_DELAY_S = 0.2;
    Aim aim = ShootOnTheMove.aim(new Pose(0, 0, 0), 0.6, 0, 1.6, 0); // Driving straight at it
    assertTrue(aim.shot.valid);
    assertTrue(Math.abs(aim.hoodRateDegPerSec) > 0);
    double saved = ShootOnTheMoveConstants.MAX_HOOD_RATE_TO_FIRE_DEG_PER_SEC;
    try {
      ShootOnTheMoveConstants.MAX_HOOD_RATE_TO_FIRE_DEG_PER_SEC = Math.abs(aim.hoodRateDegPerSec) + 1;
      assertTrue(ShootOnTheMove.readyToFire(aim, true, true, aim.lockHeadingRad, aim.lockRateRadPerSec));
      ShootOnTheMoveConstants.MAX_HOOD_RATE_TO_FIRE_DEG_PER_SEC = Math.abs(aim.hoodRateDegPerSec) - 1;
      assertFalse(ShootOnTheMove.readyToFire(aim, true, true, aim.lockHeadingRad, aim.lockRateRadPerSec));
    } finally {
      ShootOnTheMoveConstants.MAX_HOOD_RATE_TO_FIRE_DEG_PER_SEC = saved;
    }
  }

  @Test
  public void notReadyWhileSwingingThroughTheTarget() {
    ShootOnTheMoveConstants.FEED_DELAY_S = 0.2;
    Aim aim = ShootOnTheMove.aim(new Pose(0, 0, 0), 0, 0, 1.5, 0);
    // On the heading now, but turning 75 deg/s carries it 15 degrees past by the time the ball leaves
    assertFalse(ShootOnTheMove.readyToFire(aim, true, true, aim.targetHeadingRad, Math.toRadians(-75)));
    assertTrue(ShootOnTheMove.readyToFire(aim, true, true, aim.targetHeadingRad, Math.toRadians(-5)));
  }

  @Test
  public void notReadyWhenOnlyTurningFastLinesUpTheExit() {
    ShootOnTheMoveConstants.FEED_DELAY_S = 0.2;
    Aim aim = ShootOnTheMove.aim(new Pose(0, 0, 0), 0, 0, 1.5, 0);
    double rate = Math.toRadians(50);
    double heading = aim.targetHeadingRad - rate * 0.2; // 10 degrees short, arriving as the ball leaves
    assertFalse(ShootOnTheMove.readyToFire(aim, true, true, heading, rate));
  }

  @Test
  public void standingStillTheLockHeadingIsTheTarget() {
    ShootOnTheMoveConstants.FEED_DELAY_S = 0.2;
    Aim aim = ShootOnTheMove.aim(new Pose(0, 0, 0), 0, 0, 1.5, 0);
    assertEquals(aim.targetHeadingRad, aim.lockHeadingRad, TOLERANCE);
  }

  @Test
  public void strafingTheLockHeadingTrailsTheTargetByTheFeedDelay() {
    ShootOnTheMoveConstants.FEED_DELAY_S = 0.2;
    // Moving +y past a Cell at +x: by the time the ball leaves the Cell is further clockwise
    Aim aim = ShootOnTheMove.aim(new Pose(0, 0, 0), 0, 0.5, 1.5, 0);
    assertTrue(aim.shot.valid);
    double targetAheadOfLock = ShootOnTheMove.normalizeAngle(aim.targetHeadingRad - aim.lockHeadingRad);
    assertEquals(-Math.atan2(0.5 * 0.2, 1.5), targetAheadOfLock, 1e-9);
  }

  @Test
  public void holdingTheLockHeadingWhileTurningWithTheTargetIsReady() {
    ShootOnTheMoveConstants.FEED_DELAY_S = 0.2;
    Aim aim = ShootOnTheMove.aim(new Pose(0, 0, 0), 0, 0.5, 1.5, 0);
    double rate = ShootOnTheMove.normalizeAngle(aim.targetHeadingRad - aim.lockHeadingRad) / 0.2;
    assertTrue(ShootOnTheMove.readyToFire(aim, true, true, aim.lockHeadingRad, rate));
  }

  @Test
  public void strafingTheLockRateFollowsTheSidewaysSpeed() {
    // Moving left at 0.5 m/s, 1.5 m from the Cell: it swings clockwise at a third of a radian a second
    Aim aim = ShootOnTheMove.aim(new Pose(0, 0, 0), 0, 0.5, 1.5, 0);
    assertEquals(-0.5 / 1.5, aim.lockRateRadPerSec, 1e-9);
    assertEquals(0.0, ShootOnTheMove.aim(new Pose(0, 0, 0), 0.5, 0, 1.5, 0).lockRateRadPerSec, 1e-9);
  }

  @Test
  public void notReadyWhenTheLatestExitWouldMiss() {
    Aim aim = ShootOnTheMove.aim(new Pose(0, 0, 0), 0, 0, 1.5, 0);
    double driftRate = (aim.headingToleranceRad + Math.toRadians(1)) / ShootOnTheMoveConstants.FEED_DELAY_MAX_S;
    assertFalse(ShootOnTheMove.readyToFire(aim, true, true, aim.lockHeadingRad, driftRate));
    assertTrue(ShootOnTheMove.readyToFire(aim, true, true, aim.lockHeadingRad, driftRate / 2));
  }

  @Test
  public void neverReadyWithoutAValidShot() {
    Aim aim = ShootOnTheMove.aim(new Pose(0, 0, 0), 0, 0, 10, 0);
    assertFalse(aim.shot.valid);
    assertFalse(ShootOnTheMove.readyToFire(aim, true, true, aim.targetHeadingRad, 0));
  }

  @Test
  public void headingToleranceShrinksWithDistanceAndIsCapped() {
    double cap = Math.toRadians(ShootOnTheMoveConstants.MAX_HEADING_TOLERANCE_DEG);
    assertEquals(cap, ShootOnTheMove.headingTolerance(0.3), 1e-12);
    assertTrue(ShootOnTheMove.headingTolerance(3.0) < ShootOnTheMove.headingTolerance(2.0));
    assertTrue(ShootOnTheMove.headingTolerance(3.0) < cap);
  }

  @Test
  public void normalizeAngleKeepsAnglesBetweenMinusPiAndPi() {
    assertEquals(0.5, ShootOnTheMove.normalizeAngle(0.5 + 2 * Math.PI), 1e-12);
    assertEquals(-0.5, ShootOnTheMove.normalizeAngle(-0.5 - 4 * Math.PI), 1e-12);
    assertEquals(Math.PI - 0.1, ShootOnTheMove.normalizeAngle(-Math.PI - 0.1), 1e-12);
  }

  // ---- Backward shot ----

  @Test public void backwardRpmIsTheCalibratedPointAndStraightBetween() {
    assertEquals(3370, ShootOnTheMove.calculateBackward(0.95, 0, 0).targetRpm, 1e-6);
    assertEquals(3460, ShootOnTheMove.calculateBackward(1.8, 0, 0).targetRpm, 1e-6);
    assertEquals(3800, ShootOnTheMove.calculateBackward(2.3, 0, 0).targetRpm, 1e-6);
    assertEquals(3630, ShootOnTheMove.calculateBackward(2.05, 0, 0).targetRpm, 1e-6);
    assertEquals(ShootOnTheMoveConstants.BACKWARD_LAUNCH_DEG, ShootOnTheMove.calculateBackward(1.3, 0, 0).hoodDeg, 0);
  }

  @Test public void backwardCloseInLiftsTheHoodOffItsStop() {
    Shot close = ShootOnTheMove.calculateBackward(0.40, 0, 0);
    assertEquals(3200, close.targetRpm, 1e-6);
    assertEquals(90, close.hoodDeg, 1e-6);
    Shot between = ShootOnTheMove.calculateBackward(0.77, 0, 0); // Halfway from 0.73 to 0.81 m
    assertEquals(3310, between.targetRpm, 1e-6);
    assertEquals(96.45, between.hoodDeg, 1e-6);
  }

  @Test public void backwardJustInsideTheTableUsesItsFirstPoint() {
    Shot shot = ShootOnTheMove.calculateBackward(ShootOnTheMoveConstants.BACKWARD_MIN_DIST_M, 0, 0);
    assertTrue(shot.valid);
    assertEquals(ShootOnTheMoveConstants.BACKWARD_RPM[0], shot.targetRpm, 1e-6);
    assertEquals(ShootOnTheMoveConstants.BACKWARD_HOOD_DEG[0], shot.hoodDeg, 1e-6);
    assertEquals(ShootOnTheMoveConstants.BACKWARD_MIN_DIST_M, shot.distanceToLipM, 1e-9);
  }

  @Test public void backwardRefusesOutsideTheCalibratedDistances() {
    assertEquals(Shot.Reason.OUT_OF_RANGE,
        ShootOnTheMove.calculateBackward(ShootOnTheMoveConstants.BACKWARD_MIN_DIST_M - 0.01, 0, 0).reason);
    assertEquals(Shot.Reason.OUT_OF_RANGE, ShootOnTheMove.calculateBackward(2.4, 0, 0).reason);
    assertEquals(Shot.Reason.OUT_OF_RANGE, ShootOnTheMove.calculateBackward(Double.NaN, 0, 0).reason);
  }

  @Test public void backwardRefusesWhileMoving() {
    assertEquals(Shot.Reason.CLOSING_TOO_FAST, ShootOnTheMove.calculateBackward(1.5, 0.5, 0).reason);
    assertEquals(Shot.Reason.BACKING_TOO_FAST, ShootOnTheMove.calculateBackward(1.5, -0.5, 0).reason);
    assertEquals(Shot.Reason.SIDEWAYS_TOO_FAST, ShootOnTheMove.calculateBackward(1.5, 0, 0.5).reason);
  }

  @Test public void backwardInRangePointsTheRobotsFrontAtTheCell() {
    double clusterX = 1.8 + ShootOnTheMoveConstants.CLUSTER_TO_NEAR_LIP_M;
    Aim aim = ShootOnTheMove.aim(new Pose(0, 0, 0), 0, 0, clusterX, 0, true);
    assertTrue(aim.backward);
    assertTrue(aim.shot.valid);
    assertEquals(0, aim.targetHeadingRad, 1e-9); // Front toward +x, where the Cell is
    assertEquals(3460, aim.shot.targetRpm, 1e-6);
  }

  @Test public void beyondTheBackwardTableUsesTheHoodedShotOutOfTheBack() {
    double clusterX = 2.6 + ShootOnTheMoveConstants.CLUSTER_TO_NEAR_LIP_M;
    Aim aim = ShootOnTheMove.aim(new Pose(0, 0, 0), 0, 0, clusterX, 0, true);
    assertFalse(aim.backward);
    assertEquals(Math.PI, Math.abs(aim.targetHeadingRad), 1e-9);
  }

  @Test public void withoutBackwardAllowedItIsAlwaysTheHoodedShot() {
    double clusterX = 1.8 + ShootOnTheMoveConstants.CLUSTER_TO_NEAR_LIP_M;
    assertFalse(ShootOnTheMove.aim(new Pose(0, 0, 0), 0, 0, clusterX, 0).backward);
  }

  private static boolean backwardQuicker(double lipM, double headingRad, double towardMps, boolean hoodAndWheelsAtBackward) {
    double clusterX = lipM + ShootOnTheMoveConstants.CLUSTER_TO_NEAR_LIP_M;
    Pose pose = new Pose(0, 0, headingRad);
    Aim hooded = ShootOnTheMove.aimHooded(pose, towardMps, 0, clusterX, 0);
    Aim backward = ShootOnTheMove.aimBackward(pose, towardMps, 0, clusterX, 0);
    Aim at = hoodAndWheelsAtBackward ? backward : hooded;
    return ShootOnTheMove.backwardQuicker(hooded, backward, headingRad, at.hoodNowDeg, at.rpmNow);
  }

  @Test
  public void closeInFacingTheHoodedWayTakesTheHoodedShot() {
    // 0.6 m was always backward on the old 0.80 m arc; facing the hooded way it is now the hooded shot
    assertFalse(backwardQuicker(0.6, Math.PI, 0, false));
    assertTrue(backwardQuicker(0.6, 0, 0, true));
  }

  @Test
  public void facingTheBackwardWayTakesTheBackwardShot() {
    assertTrue(backwardQuicker(0.7, 0, 0, true));
    assertTrue(backwardQuicker(0.7, 0, 0, false)); // Hood and wheels elsewhere, but no 180 degree turn
  }

  @Test
  public void pastTheCalibratedBackwardSpotsIsNeverBackward() {
    // The backward table reaches 2.3 m, but only 0.40-0.81 m was ever shot
    assertFalse(backwardQuicker(1.5, 0, 0, true));
    assertFalse(backwardQuicker(3.0, 0, 0, true));
  }

  @Test
  public void aBackwardShotBlockedBySpeedCountsLater() {
    double clusterX = 0.7 + ShootOnTheMoveConstants.CLUSTER_TO_NEAR_LIP_M;
    Pose sideways = new Pose(0, 0, Math.PI / 2);
    Aim fast = ShootOnTheMove.aimBackward(sideways, 1.0, 0, clusterX, 0);
    Aim slow = ShootOnTheMove.aimBackward(sideways, 0, 0, clusterX, 0);
    assertFalse(fast.shot.valid);
    assertEquals(ShootOnTheMoveConstants.SHOT_PICK_BLOCKED_S,
        ShootOnTheMove.secondsToReady(fast, Math.PI / 2, Double.NaN, Double.NaN)
            - ShootOnTheMove.secondsToReady(slow, Math.PI / 2, Double.NaN, Double.NaN), 1e-9);
    // Side on and driving at it: both need 90 degrees, so the hooded shot, which it can take moving, wins
    assertFalse(backwardQuicker(0.7, Math.PI / 2, 1.0, true));
  }

  @Test
  public void aimBackwardKeepsItsHeadingWhenMovingTooFast() {
    // Cell 1.5 m along +x; the robot drives toward it faster than the backward table allows
    Aim aim = ShootOnTheMove.aimBackward(new Pose(0, 0, 0), 1.0, 0, 1.5, 0);
    assertTrue(aim.backward);
    assertFalse(aim.shot.valid);
    assertEquals(ShootOnTheMoveConstants.BACKWARD_DIRECTION_RAD, aim.targetHeadingRad, 1e-6);
  }

  @Test
  public void aimHoodedNeverPicksBackward() {
    Aim aim = ShootOnTheMove.aimHooded(new Pose(0, 0, 0), 0, 0, 1.5, 0);
    assertFalse(aim.backward);
  }

  @Test
  public void hoodSteepensForWheelsAboveTheTableRpm() {
    assertEquals(0, ShootOnTheMove.hoodOffsetForRpmAbove(1.2, 0, 0), TOLERANCE);
    assertEquals(0, ShootOnTheMove.hoodOffsetForRpmAbove(1.2, 0, -100), TOLERANCE); // Below the table: none
    assertEquals(0, ShootOnTheMove.hoodOffsetForRpmAbove(Double.NaN, 0, 200), TOLERANCE);
    assertEquals(0, ShootOnTheMove.hoodOffsetForRpmAbove(9.0, 0, 200), TOLERANCE); // Off the table
    double previous = 0;
    for (int rpm = 50; rpm <= 600; rpm += 50) {
      double offset = ShootOnTheMove.hoodOffsetForRpmAbove(1.2, 0, rpm);
      assertTrue("never falls as the wheels speed up: " + rpm, offset >= previous - 1e-9);
      previous = offset;
    }
    assertTrue("+300 RPM at 1.2 m needs several degrees more", ShootOnTheMove.hoodOffsetForRpmAbove(1.2, 0, 300) > 3);
    assertEquals(ShootOnTheMove.hoodOffsetForRpmAbove(1.2, 0, 600),
        ShootOnTheMove.hoodOffsetForRpmAbove(1.2, 0, 900), TOLERANCE); // Held past the last step
  }

  @Test
  public void inFrontOfCellIsAWedgeOutFromTheCellsWall() {
    // Cell on the +x wall: out from it is -x
    assertTrue(ShootOnTheMove.inFrontOfCell(new Pose(1.0, 0.1, 0), 1.5, 0, 30));
    assertTrue(ShootOnTheMove.inFrontOfCell(new Pose(1.0, 0.25, 0), 1.5, 0, 30)); // 26.6 deg
    assertFalse(ShootOnTheMove.inFrontOfCell(new Pose(1.0, 0.4, 0), 1.5, 0, 30)); // 38.7 deg
    assertFalse(ShootOnTheMove.inFrontOfCell(new Pose(2.0, 0, 0), 1.5, 0, 30)); // Behind the wall
    // Cell on the -x wall: out from it is +x
    assertTrue(ShootOnTheMove.inFrontOfCell(new Pose(-1.0, -0.1, 0), -1.5, 0, 30));
    assertFalse(ShootOnTheMove.inFrontOfCell(new Pose(-2.0, 0, 0), -1.5, 0, 30));
    assertFalse(ShootOnTheMove.inFrontOfCell(new Pose(1.0, 0, 0), 1.5, 0, 0)); // Off
  }
}
