package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertArrayEquals;

import org.junit.Test;

/** Wheel powers are {FL, FR, BL, BR}, checked against gm0's mecanum mix. */
public class MecanumDriveMixTest {
  private static final double EPS = 1e-9;

  private static double[] mix(double forward, double right, double rotate) {
    double[] out = new double[4];
    MecanumDriveSubsystem.mix(forward, right, rotate, out);
    return out;
  }

  @Test
  public void forwardRunsEveryWheelForward() {
    assertArrayEquals(new double[] {0.5, 0.5, 0.5, 0.5}, mix(0.5, 0, 0), EPS);
  }

  @Test
  public void strafeRightRunsTheDiagonalsOpposite() {
    assertArrayEquals(new double[] {0.5, -0.5, -0.5, 0.5}, mix(0, 0.5, 0), EPS);
  }

  @Test
  public void clockwiseRunsTheLeftSideForward() {
    assertArrayEquals(new double[] {0.5, -0.5, 0.5, -0.5}, mix(0, 0, 0.5), EPS);
  }

  @Test
  public void scalesDownTogetherPastFullPower() {
    // Forward plus strafe right gives FL and BR 2, FR and BL 0.
    assertArrayEquals(new double[] {1, 0, 0, 1}, mix(1, 1, 0), EPS);
  }

  @Test
  public void stickCurveHasASmallDeadbandThenStartsWhereTheWheelsMove() {
    double minPower = MecanumDriveSubsystem.STICK_MIN_POWER;
    org.junit.Assert.assertEquals(0.0, MecanumDriveSubsystem.shapeStick(0.04), 1e-9);
    // Just past the deadband the wheels already get the power that turns them, not a near-zero value
    org.junit.Assert.assertEquals(minPower, MecanumDriveSubsystem.shapeStick(0.0501), 1e-3);
    org.junit.Assert.assertEquals(-minPower, MecanumDriveSubsystem.shapeStick(-0.0501), 1e-3);
    org.junit.Assert.assertEquals(1.0, MecanumDriveSubsystem.shapeStick(1.0), 1e-9);
    // Rises smoothly: no step bigger than a small one between neighbouring stick positions
    double previous = MecanumDriveSubsystem.shapeStick(0.06);
    for (double stick = 0.07; stick <= 1.0; stick += 0.01) {
      double now = MecanumDriveSubsystem.shapeStick(stick);
      org.junit.Assert.assertTrue(now > previous);
      org.junit.Assert.assertTrue(now - previous < 0.03);
      previous = now;
    }
  }

  @Test
  public void turnStickIgnoresTheSmallReadingsSeenWhileDriving() {
    // 0.06-0.11 untouched while the drive stick was pushed (5 Nov, 093520)
    org.junit.Assert.assertEquals(0.0, MecanumDriveSubsystem.shapeTurnStick(0.11), 1e-9);
    org.junit.Assert.assertEquals(MecanumDriveSubsystem.STICK_MIN_POWER,
        MecanumDriveSubsystem.shapeTurnStick(MecanumDriveSubsystem.TURN_STICK_DEADBAND + 1e-4), 1e-3);
    org.junit.Assert.assertEquals(-1.0, MecanumDriveSubsystem.shapeTurnStick(-1.0), 1e-9);
  }

  @Test
  public void headingLockTurnsWithAMovingTargetBeforeFallingBehind() {
    // On target but the target swings at 1 rad/s: the robot must already be turning with it
    double ccw = MecanumDriveSubsystem.headingLockTurn(0.0, 1.0, 0.0);
    org.junit.Assert.assertEquals(MecanumDriveSubsystem.HEADING_KV + MecanumDriveSubsystem.HEADING_KD
        + MecanumDriveSubsystem.HEADING_KP * MecanumDriveSubsystem.HEADING_LATENCY_S, ccw, 1e-9);
    // Already turning at the target's rate: no damping against the turn
    org.junit.Assert.assertEquals(MecanumDriveSubsystem.HEADING_KV,
        MecanumDriveSubsystem.headingLockTurn(0.0, 1.0, 1.0), 1e-9);
  }

  @Test
  public void headingLockPushesPastStictionWhenAFewDegreesOff() {
    // 5 Nov: standing 7-9 degrees off, P alone gave less than the robot needs to start turning
    double offRad = -0.9 * MecanumDriveSubsystem.HEADING_KS / MecanumDriveSubsystem.HEADING_KP;
    org.junit.Assert.assertTrue(Math.abs(offRad) > MecanumDriveSubsystem.HEADING_KS_ABOVE_RAD);
    double ccw = MecanumDriveSubsystem.headingLockTurn(offRad, 0.0, 0.0);
    org.junit.Assert.assertEquals(-MecanumDriveSubsystem.HEADING_KS, ccw, 1e-9);
    // Inside the deadband it is left to P, so it doesn't hunt around the target
    double small = Math.toRadians(1);
    org.junit.Assert.assertEquals(MecanumDriveSubsystem.HEADING_KP * small,
        MecanumDriveSubsystem.headingLockTurn(small, 0.0, 0.0), 1e-9);
  }

  @Test
  public void stuckOffTargetTheLeastPushGrowsUntilTheRobotTurns() {
    // 5 Nov (083302): 8.6 degrees off, 0.16 at the wheels, and the robot didn't turn for 4.5 s
    double off = Math.toRadians(8.6);
    double boost = 0;
    for (int loop = 0; loop < 25; loop++) {
      boost = MecanumDriveSubsystem.stuckBoost(boost, off, 0.0, 0.02);
    }
    org.junit.Assert.assertEquals(0.5 * MecanumDriveSubsystem.HEADING_KS_BOOST_PER_S, boost, 1e-9);
    double ccw = MecanumDriveSubsystem.headingLockTurn(off, 0.0, 0.0, MecanumDriveSubsystem.HEADING_KS + boost);
    org.junit.Assert.assertTrue(ccw > MecanumDriveSubsystem.HEADING_KS);
    // Capped however long it is stuck
    for (int loop = 0; loop < 500; loop++) {
      boost = MecanumDriveSubsystem.stuckBoost(boost, off, 0.0, 0.02);
    }
    org.junit.Assert.assertEquals(MecanumDriveSubsystem.HEADING_KS_BOOST_MAX, boost, 1e-9);
    // Once it turns, or is close enough, back to the plain least push
    org.junit.Assert.assertEquals(0, MecanumDriveSubsystem.stuckBoost(boost, off, 0.5, 0.02), 1e-9);
    org.junit.Assert.assertEquals(0, MecanumDriveSubsystem.stuckBoost(boost, Math.toRadians(1), 0.0, 0.02), 1e-9);
    // A long pause between loops adds no more than 0.1 s worth
    org.junit.Assert.assertEquals(0.1 * MecanumDriveSubsystem.HEADING_KS_BOOST_PER_S,
        MecanumDriveSubsystem.stuckBoost(0, off, 0.0, 2.0), 1e-9);
  }

  @Test
  public void aLowerTopSpeedKeepsTheStartPowerAndCurve() {
    double minPower = MecanumDriveSubsystem.STICK_MIN_POWER;
    org.junit.Assert.assertEquals(0.5, MecanumDriveSubsystem.shapeStick(1.0, 0.5), 1e-9);
    org.junit.Assert.assertEquals(-0.5, MecanumDriveSubsystem.shapeStick(-1.0, 0.5), 1e-9);
    org.junit.Assert.assertEquals(minPower, MecanumDriveSubsystem.shapeStick(0.0501, 0.5), 1e-3);
    org.junit.Assert.assertEquals(0.0, MecanumDriveSubsystem.shapeStick(0.04, 0.5), 1e-9);
    double[] out = MecanumDriveSubsystem.shapeTranslation(0.6, 0.8, 0.5); // Full stick, any direction
    org.junit.Assert.assertEquals(0.5, Math.hypot(out[0], out[1]), 1e-9);
    org.junit.Assert.assertEquals(0.6 / 0.8, out[0] / out[1], 1e-9);
  }

  @Test
  public void headingLockBrakesBeforeArrivingWhenTurningFast() {
    // 10 degrees to go but closing at 3 rad/s: within the power delay it would be past, so it brakes
    double ccw = MecanumDriveSubsystem.headingLockTurn(Math.toRadians(10), 0.0, 3.0);
    org.junit.Assert.assertTrue(ccw < 0);
  }

  @Test
  public void turnFirstKeepsTheTurnAndGivesWayOnTheDrive() {
    double[] out = new double[4];
    // Full strafe plus a 0.5 turn: the shared scale-down would leave the turn 0.33
    MecanumDriveSubsystem.mixTurnFirst(0, 1, 0.5, 0.4, out);
    org.junit.Assert.assertEquals(0.5, (out[0] - out[1] + out[2] - out[3]) / 4, 1e-9);
    for (double p : out) {
      org.junit.Assert.assertTrue(Math.abs(p) <= 1 + 1e-9);
    }
    // Room for both: unchanged from the plain mix
    double[] plain = new double[4];
    MecanumDriveSubsystem.mixTurnFirst(0.3, 0.2, 0.2, 0.4, out);
    MecanumDriveSubsystem.mix(0.3, 0.2, 0.2, plain);
    assertArrayEquals(plain, out, EPS);
    // The drive keeps at least its minimum share: then all is scaled down together
    MecanumDriveSubsystem.mixTurnFirst(0, 1, 1, 0.4, out);
    assertArrayEquals(mixed(0, 0.4, 1), out, EPS);
  }

  private static double[] mixed(double forward, double right, double rotate) {
    return mix(forward, right, rotate);
  }

  @Test
  public void nearlyBehindItKeepsTurningTheSameWay() {
    double justPast = Math.toRadians(-179); // The shorter way flipped since last loop's +179
    org.junit.Assert.assertEquals(Math.toRadians(181),
        MecanumDriveSubsystem.keepTurnDirection(justPast, Math.toRadians(179)), 1e-9);
    // No last loop, or small errors: the shorter way
    org.junit.Assert.assertEquals(justPast, MecanumDriveSubsystem.keepTurnDirection(justPast, Double.NaN), 1e-9);
    org.junit.Assert.assertEquals(Math.toRadians(-20),
        MecanumDriveSubsystem.keepTurnDirection(Math.toRadians(-20), Math.toRadians(20)), 1e-9);
    // Once turned well past, it takes the shorter way again
    org.junit.Assert.assertEquals(Math.toRadians(-100),
        MecanumDriveSubsystem.keepTurnDirection(Math.toRadians(-100), Math.toRadians(170)), 1e-9);
  }

  @Test
  public void translationCurveKeepsTheStickDirection() {
    double[] out = MecanumDriveSubsystem.shapeTranslation(0.3, 0.4); // Half stick at 53 degrees
    org.junit.Assert.assertEquals(0.3 / 0.4, out[0] / out[1], 1e-9);
    org.junit.Assert.assertEquals(MecanumDriveSubsystem.shapeStick(0.5), Math.hypot(out[0], out[1]), 1e-9);
  }
}
