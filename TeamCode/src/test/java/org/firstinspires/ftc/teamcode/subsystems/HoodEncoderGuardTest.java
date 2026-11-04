package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.function.DoublePredicate;
import java.util.function.DoubleUnaryOperator;
import org.junit.Test;

public class HoodEncoderGuardTest {
  /**
   * Runs the guard every 20 ms as HoodSubsystem does, moving the motor's zero to match the REV on each
   * rebase. Returns the time it tripped, or NaN if it never did.
   */
  private static double run(HoodEncoderGuard guard, double seconds, DoubleUnaryOperator rev,
      DoubleUnaryOperator motor, DoublePredicate powered, boolean atRest) {
    int motorZero = 0;
    for (double t = 0; t <= seconds; t += 0.02) {
      int revTicks = (int) Math.round(rev.applyAsDouble(t));
      int motorTicks = (int) Math.round(motor.applyAsDouble(t)) - motorZero;
      if (guard.update(t, revTicks, motorTicks, powered.test(t), atRest)) {
        return t;
      }
      if (guard.takeRebase()) {
        motorZero += motorTicks - (int) Math.round(revTicks / 8192.0 * 384.5);
      }
    }
    return Double.NaN;
  }

  @Test public void acceptsMatchedEncodersAcrossFullTravel() {
    HoodEncoderGuard guard = new HoodEncoderGuard();
    for (int i = 0; i < 100; i++) {
      double turns = i / 75.0;
      assertFalse(guard.update(i * 0.02, (int) Math.round(turns * 8192),
          (int) Math.round(turns * 384.5), true));
    }
    assertEquals(0, guard.getSlipCount());
  }

  @Test public void beltSlipIsRezeroedNotTripped() {
    // Logged 4 Nov: the motor reached full lift (~490 ticks) but the REV stopped 2100 ticks short
    HoodEncoderGuard guard = new HoodEncoderGuard();
    // Encoders agree until the belt skips 2100 REV ticks at 0.2 s, then move together again
    double tripped = run(guard, 2.0,
        t -> Math.min(t, 0.4) / 0.4 * 10400 - (t >= 0.2 ? 2100 : 0),
        t -> Math.min(t, 0.4) / 0.4 * 10400 / 8192.0 * 384.5, t -> t < 0.4, true);
    assertTrue(Double.isNaN(tripped));
    assertEquals(1, guard.getSlipCount());
  }

  @Test public void cutsWhenRevIsMissingButMotorMoves() {
    HoodEncoderGuard guard = new HoodEncoderGuard();
    double tripped = run(guard, 2.0, t -> 0, t -> 300 * t, t -> true, false); // REV unplugged
    assertFalse(Double.isNaN(tripped));
    assertTrue(tripped < 1.5);
    assertTrue(guard.update(3, 2048, 96, false)); // Latched until re-zeroed
  }

  @Test public void cutsWrongDirectionAndSwappedScales() {
    double reversed = run(new HoodEncoderGuard(), 2.0, t -> -6400 * t, t -> 300 * t, t -> true, false);
    assertFalse(Double.isNaN(reversed));
    double swapped = run(new HoodEncoderGuard(), 2.0, t -> 300 * t, t -> 6400 * t, t -> true, false);
    assertFalse(Double.isNaN(swapped));
  }

  @Test public void cutsWhenBothEncodersStopWhilePowered() {
    HoodEncoderGuard guard = new HoodEncoderGuard();
    assertFalse(guard.update(0, 0, 0, true));
    assertTrue(guard.update(0.51, 0, 0, true));
    guard.reset();
    assertFalse(guard.update(1, 0, 0, false));
    assertFalse(guard.update(10, 0, 0, false));
  }

  @Test public void steadyDisagreementAtRestIsRezeroedOnce() {
    HoodEncoderGuard guard = new HoodEncoderGuard();
    assertTrue(Double.isNaN(run(guard, 1.0, t -> 2048, t -> 0, t -> false, true)));
    assertEquals(1, guard.getSlipCount());
  }

  @Test public void acceptsMeasuredReversalBacklashWhilePinionTakesUpDrive() {
    HoodEncoderGuard guard = new HoodEncoderGuard();
    assertFalse(guard.update(0, 0, 0, true));
    assertFalse(guard.update(0.05, 0, 16, true));
    assertFalse(guard.update(0.20, 64, 20, true));
    assertFalse(guard.update(0.25, 256, 24, true));
    assertFalse(guard.takeRebase());
  }

  @Test public void fastReversalWithBacklashAndCrossHubSkewDoesNotTrip() {
    // Logged 13->3 degree move: 4-tick stop offset + 16 backlash + read skew = 19 to 21 ticks mid-move.
    HoodEncoderGuard guard = new HoodEncoderGuard();
    assertFalse(guard.update(1.82, 2693, 107, true, false));
    assertFalse(guard.update(1.92, 2515, 97, true, false));
    assertFalse(guard.update(1.98, 2053, 75, true, false));
    assertFalse(guard.update(2.10, 2009, 76, true, true)); // At rest: 20 ticks, inside backlash allowance
    assertFalse(guard.takeRebase());
  }

  @Test public void grossDisagreementIsCaughtEvenMidMove() {
    HoodEncoderGuard guard = new HoodEncoderGuard();
    assertFalse(guard.update(0, 0, 0, true, false));
    assertFalse(guard.update(0.08, 0, 70, true, false)); // REV unplugged while the motor runs
    assertFalse(guard.update(0.19, 0, 107, true, false));
    assertTrue(guard.takeRebase()); // Caught mid-move; keeps coming back, so run() trips it
  }
}
