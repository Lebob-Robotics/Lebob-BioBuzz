package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import org.junit.Test;

public class HoodSubsystemTest {
  @Test public void seventyFiveDegreesNeedsTwoThirdsOfARevEncoderTurn() {
    assertEquals(5461, HoodSubsystem.angleToTicks(75));
    assertEquals(256, Math.round(HoodSubsystem.angleToTicks(75)
        / HoodEncoderGuard.REV_COUNTS_PER_TURN * HoodEncoderGuard.MOTOR_COUNTS_PER_TURN));
  }

  @Test public void quarterPinionTurnMovesHoodByNineDegrees() {
    assertEquals(90, HoodSubsystem.ticksToAngle(2048), 0.001);
  }

  @Test public void targetsStayOneDegreeInsideBothStops() {
    assertEquals(0, HoodSubsystem.clampTarget(0)); // STOW and LEFT at the bottom rest on the stop unpowered
    assertEquals(0, HoodSubsystem.clampTarget(-500));
    assertEquals(228, HoodSubsystem.clampTarget(1)); // Just above the stop: kept a margin off it
    assertEquals(HoodSubsystem.angleToTicks(52), HoodSubsystem.clampTarget(1_000_000));
    assertEquals(3000, HoodSubsystem.clampTarget(3000));
  }

  @Test public void tenDegreeMoveIsAboutOneHundredSevenMotorTicks() {
    HoodMotorTargeting targeting = new HoodMotorTargeting();
    assertEquals(107, targeting.motorTargetFor((int) Math.round(10 * HoodSubsystem.TICKS_PER_DEG), 0, 45));
  }
}
