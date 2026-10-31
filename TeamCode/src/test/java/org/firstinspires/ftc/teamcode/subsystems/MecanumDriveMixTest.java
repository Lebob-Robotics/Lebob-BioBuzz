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
}
