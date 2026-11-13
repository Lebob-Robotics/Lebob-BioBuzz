package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class RobotAutoDriveFrameTest {
  /** MecanumDriveSubsystem's field-centric turn of {forward, right} into the robot's frame. */
  private static double[] driveToRobot(double forward, double right, double driverHeadingRad) {
    double theta = Math.atan2(forward, right) - driverHeadingRad;
    double r = Math.hypot(right, forward);
    return new double[] {r * Math.sin(theta), r * Math.cos(theta)};
  }

  @Test
  public void pedroPowersReachTheWheelsInTheRobotFrame() {
    double[] driverForwards = {Math.PI / 2, -Math.PI / 2, Math.PI / 2 + Math.PI};
    double[] headings = {0, 0.7, -2.1, Math.PI};
    double[][] powers = {{1, 0}, {0, 1}, {0.4, -0.3}, {-0.6, 0.2}};
    for (double f : driverForwards) {
      for (double h : headings) {
        for (double[] p : powers) {
          double[] driver = Robot.robotToDriverFrame(p[0], p[1], h, f);
          double[] robot = driveToRobot(driver[0], driver[1], h - f);
          assertEquals("forward", p[0], robot[0], 1e-9);
          assertEquals("right is minus left", -p[1], robot[1], 1e-9);
        }
      }
    }
  }
}
