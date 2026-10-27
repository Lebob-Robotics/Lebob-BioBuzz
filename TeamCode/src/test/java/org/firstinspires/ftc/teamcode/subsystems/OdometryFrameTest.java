package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.junit.Test;

/** Raw Pinpoint readings are from the Drive Check OpMode on Mark 3, 27 Oct. */
public class OdometryFrameTest {
  private static final double EPS = 1e-6;

  private static Pose2D robot(double rawX, double rawY, double rawHeadingDeg) {
    return OdometrySubsystem.toRobotFrame(
        new Pose2D(DistanceUnit.MM, rawX, rawY, AngleUnit.DEGREES, rawHeadingDeg));
  }

  @Test
  public void pushedForwardReadsPositiveX() {
    Pose2D pose = robot(25, 509, 0);
    assertEquals(509, pose.getX(DistanceUnit.MM), EPS);
    assertEquals(25, pose.getY(DistanceUnit.MM), EPS);
  }

  @Test
  public void pushedLeftReadsPositiveY() {
    Pose2D pose = robot(497, 16, 0);
    assertEquals(16, pose.getX(DistanceUnit.MM), EPS);
    assertEquals(497, pose.getY(DistanceUnit.MM), EPS);
  }

  @Test
  public void turnedCounterclockwiseReadsPositiveHeading() {
    assertEquals(88.8, robot(0, 0, -88.8).getHeading(AngleUnit.DEGREES), EPS);
  }

  @Test
  public void isItsOwnInverse() {
    Pose2D pose = OdometrySubsystem.toRobotFrame(robot(120, -340, 30));
    assertEquals(120, pose.getX(DistanceUnit.MM), EPS);
    assertEquals(-340, pose.getY(DistanceUnit.MM), EPS);
    assertEquals(30, pose.getHeading(AngleUnit.DEGREES), EPS);
  }
}
