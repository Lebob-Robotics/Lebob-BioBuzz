package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
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

  @Test public void twoDegreeMoveRequestsControlledSpeedRatherThanNinetyPercentPower() {
    assertEquals(0.0364, HoodSubsystem.positionVelocityRequest(455), 1e-9);
    assertEquals(-0.0364, HoodSubsystem.positionVelocityRequest(-455), 1e-9);
    assertEquals(1, HoodSubsystem.positionVelocityRequest(20000), 0);
    assertEquals(-1, HoodSubsystem.positionVelocityRequest(-20000), 0);
  }

  @Test public void sampledNativeVelocityModelSettlesWithoutRepeatedFullScaleReversals() {
    // Simple 40 ms inner-speed-loop model at a 20 ms outer-loop period.
    // This catches the previous position gain driving the mechanism far past a 2-degree target.
    double position = 0, nativeVelocity = 0;
    for (int step = 0; step < 100; step++) {
      double request = HoodSubsystem.positionVelocityRequest((int) Math.round(455 - position));
      assertTrue(Math.abs(request) < 0.1);
      nativeVelocity += (request * 2800 - nativeVelocity) * 0.5;
      position += nativeVelocity * 8192 / 384.5 * 0.02;
      assertTrue(position >= 0 && position <= 485);
    }
    assertEquals(455, position, 30);
  }
}
