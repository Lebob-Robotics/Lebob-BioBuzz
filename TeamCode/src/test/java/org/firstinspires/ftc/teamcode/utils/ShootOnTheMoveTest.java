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

  @Before
  public void zeroShooterOffsetAndDelay() {
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

  // ---- readyToFire and helpers -------------------------------------------

  @Test
  public void readyNeedsEverythingOnTarget() {
    Aim aim = ShootOnTheMove.aim(new Pose(0, 0, 0), 0, 0, 1.5, 0);
    double onHeading = aim.targetHeadingRad;
    double offHeading = aim.targetHeadingRad + aim.headingToleranceRad * 2;
    assertTrue(ShootOnTheMove.readyToFire(aim, true, true, onHeading));
    assertFalse(ShootOnTheMove.readyToFire(aim, false, true, onHeading));
    assertFalse(ShootOnTheMove.readyToFire(aim, true, false, onHeading));
    assertFalse(ShootOnTheMove.readyToFire(aim, true, true, offHeading));
  }

  @Test
  public void neverReadyWithoutAValidShot() {
    Aim aim = ShootOnTheMove.aim(new Pose(0, 0, 0), 0, 0, 10, 0);
    assertFalse(aim.shot.valid);
    assertFalse(ShootOnTheMove.readyToFire(aim, true, true, aim.targetHeadingRad));
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
}
