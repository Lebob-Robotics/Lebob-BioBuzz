package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class HeadingLockTwistTest {
  @Test
  public void twistCompensationTurnsAgainstTheMeasuredTwist() {
    // 7 Nov: forward twists the robot CCW, strafing right twists it CW
    assertTrue(MecanumDriveSubsystem.twistCompensation(0.5, 0) < 0); // So the cancel is CW
    assertTrue(MecanumDriveSubsystem.twistCompensation(0, 0.5) > 0); // And CCW
    assertEquals(0.0, MecanumDriveSubsystem.twistCompensation(0, 0), 0.0);
    // 080429's stuck pass: robot-frame forward 0.46, right -0.25 needed about 0.15 CW more than the lock gave
    double ccw = MecanumDriveSubsystem.twistCompensation(0.46, -0.25);
    assertTrue("cancel " + ccw, ccw < -0.04 && ccw > -0.16);
  }

  @Test
  public void integralBuildsWhileDrivingOffTargetAndIsCapped() {
    double i = 0;
    for (int k = 0; k < 10; k++) { // Half a second at 8 degrees behind CCW, driving
      i = MecanumDriveSubsystem.lockIntegral(i, Math.toRadians(8), 0.05, 1.0);
    }
    assertTrue("built " + i, i > 0.08 && i <= MecanumDriveSubsystem.HEADING_I_MAX);
    for (int k = 0; k < 100; k++) {
      i = MecanumDriveSubsystem.lockIntegral(i, Math.toRadians(8), 0.05, 1.0);
    }
    assertEquals(MecanumDriveSubsystem.HEADING_I_MAX, i, 1e-12);
    i = MecanumDriveSubsystem.lockIntegral(-0.05, Math.toRadians(-8), 0.05, 1.0);
    assertTrue(i < -0.05); // The other way too
  }

  @Test
  public void integralIgnoresBigTurnsAndBleedsAwayStanding() {
    // Swinging round 40 degrees at the start of a lock, it doesn't wind up
    assertEquals(0.0, MecanumDriveSubsystem.lockIntegral(0, Math.toRadians(40), 0.05, 1.0), 0.0);
    double i = 0.1;
    for (int k = 0; k < 20; k++) { // A second standing
      i = MecanumDriveSubsystem.lockIntegral(i, Math.toRadians(5), 0.05, 0.0);
    }
    assertTrue("left " + i, Math.abs(i) < 0.01);
  }
}
