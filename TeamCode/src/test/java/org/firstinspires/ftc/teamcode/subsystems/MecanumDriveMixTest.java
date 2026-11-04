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
  public void translationCurveKeepsTheStickDirection() {
    double[] out = MecanumDriveSubsystem.shapeTranslation(0.3, 0.4); // Half stick at 53 degrees
    org.junit.Assert.assertEquals(0.3 / 0.4, out[0] / out[1], 1e-9);
    org.junit.Assert.assertEquals(MecanumDriveSubsystem.shapeStick(0.5), Math.hypot(out[0], out[1]), 1e-9);
  }
}
