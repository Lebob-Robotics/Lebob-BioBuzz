package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class RampSubsystemTest {
  private static double sentUs(double position) {
    return RampSubsystem.PWM_LOWER_US + position * (RampSubsystem.PWM_UPPER_US - RampSubsystem.PWM_LOWER_US);
  }

  @Test public void eachPositionSendsThePulseFoundOnTheRobot() {
    assertEquals(1430, sentUs(RampSubsystem.positionFor(false)), 0.5);
    assertEquals(490, sentUs(RampSubsystem.positionFor(true)), 0.5);
  }
}
