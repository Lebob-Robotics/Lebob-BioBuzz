package org.firstinspires.ftc.teamcode.pedro;

import static org.junit.Assert.assertArrayEquals;

import com.pedropathing.drivetrain.DrivePowers;

import org.junit.Test;

public class CapturedDrivetrainTest {
  @Test
  public void theWallPushIsAddedAfterTheSlowPassScale() {
    CapturedDrivetrain d = new CapturedDrivetrain();
    d.setScale(0.5);
    d.setStrafeBias(0.25);
    d.drive(new DrivePowers(0.6, 0.1, -0.2), false);
    assertArrayEquals(new double[] {0.3, 0.3, -0.1}, d.powers(), 1e-9);
  }

  @Test
  public void thePushedStrafeStaysWithinFullPower() {
    CapturedDrivetrain d = new CapturedDrivetrain();
    d.setStrafeBias(0.25);
    d.drive(new DrivePowers(0, 0.9, 0), false);
    assertArrayEquals(new double[] {0, 1.0, 0}, d.powers(), 1e-9);
    d.setStrafeBias(0.0);
    d.drive(new DrivePowers(0, 0.9, 0), false);
    assertArrayEquals(new double[] {0, 0.9, 0}, d.powers(), 1e-9);
  }
}
