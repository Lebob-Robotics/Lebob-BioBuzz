package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import org.junit.Test;

public class HoodEncoderGuardTest {
  @Test public void acceptsMatchedEncodersAcrossFullTravel() {
    HoodEncoderGuard guard = new HoodEncoderGuard();
    for (int i = 0; i < 100; i++) {
      double turns = i / 75.0;
      assertFalse(guard.update(i * 0.02, (int) Math.round(turns * 8192),
          (int) Math.round(turns * 384.5), true));
    }
  }

  @Test public void cutsWhenRevIsMissingButMotorMoves() {
    HoodEncoderGuard guard = new HoodEncoderGuard();
    assertFalse(guard.update(0, 0, 0, true));
    assertFalse(guard.update(0.1, 0, 96, true));
    assertTrue(guard.update(0.21, 0, 96, true));
    assertTrue(guard.update(0.3, 2048, 96, false)); // Latched until re-zeroed
  }

  @Test public void cutsWrongDirectionAndSwappedScales() {
    HoodEncoderGuard reversed = new HoodEncoderGuard();
    assertFalse(reversed.update(0, -2048, 96, true));
    assertTrue(reversed.update(0.11, -2048, 96, true));
    HoodEncoderGuard swapped = new HoodEncoderGuard();
    assertFalse(swapped.update(0, 96, 2048, true));
    assertTrue(swapped.update(0.11, 96, 2048, true));
  }

  @Test public void cutsWhenBothEncodersStopWhilePowered() {
    HoodEncoderGuard guard = new HoodEncoderGuard();
    assertFalse(guard.update(0, 0, 0, true));
    assertTrue(guard.update(0.51, 0, 0, true));
    guard.reset();
    assertFalse(guard.update(1, 0, 0, false));
    assertFalse(guard.update(10, 0, 0, false));
  }
  @Test public void encoderDisagreementIsCaughtBeforePowerIsEnabled() {
    HoodEncoderGuard guard = new HoodEncoderGuard();
    assertFalse(guard.update(0, 2048, 0, false));
    assertTrue(guard.update(0.11, 2048, 0, false));
  }
  @Test public void acceptsMeasuredReversalBacklashWhilePinionTakesUpDrive() {
    HoodEncoderGuard guard = new HoodEncoderGuard();
    assertFalse(guard.update(0, 0, 0, true));
    assertFalse(guard.update(0.05, 0, 16, true));
    assertFalse(guard.update(0.20, 64, 20, true));
    assertFalse(guard.update(0.25, 256, 24, true));
  }

  @Test public void stillCutsMissingRevBeyondBacklashAllowance() {
    HoodEncoderGuard guard = new HoodEncoderGuard();
    assertFalse(guard.update(0, 0, 0, true));
    assertFalse(guard.update(0.1, 0, 36, true));
    assertTrue(guard.update(0.21, 0, 36, true));
  }

  @Test public void fastReversalWithBacklashAndCrossHubSkewDoesNotTrip() {
    // Logged 13->3 degree move: 4-tick stop offset + 16 backlash + read skew = 19 to 21 ticks mid-move.
    HoodEncoderGuard guard = new HoodEncoderGuard();
    assertFalse(guard.update(1.82, 2693, 107, true, false));
    assertFalse(guard.update(1.92, 2515, 97, true, false));
    assertFalse(guard.update(1.98, 2053, 75, true, false));
    assertFalse(guard.update(2.10, 2009, 76, true, true)); // At rest: 20 ticks, inside backlash allowance
  }

  @Test public void grossDisagreementIsCaughtEvenMidMove() {
    HoodEncoderGuard guard = new HoodEncoderGuard();
    assertFalse(guard.update(0, 0, 0, true, false));
    assertFalse(guard.update(0.08, 0, 70, true, false)); // REV unplugged while the motor runs
    assertTrue(guard.update(0.19, 0, 107, true, false));
  }
}
