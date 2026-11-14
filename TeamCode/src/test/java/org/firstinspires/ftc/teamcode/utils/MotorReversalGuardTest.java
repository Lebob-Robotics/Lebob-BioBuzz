package org.firstinspires.ftc.teamcode.utils;

import static org.junit.Assert.assertEquals;
import org.junit.Test;

public class MotorReversalGuardTest {
  @Test public void intakeCoastsThenRampsWithoutBlockingTheLoop() {
    MotorReversalGuard g = new MotorReversalGuard(0.75, 0.75, 0);
    assertEquals(1, g.update(1, 0, 0), 0);
    assertEquals(0, g.update(-1, 1, 0), 0);
    assertEquals(0, g.update(-1, 1.74, 0), 0);
    assertEquals(0, g.update(-1, 1.75, 0), 0);
    assertEquals(0.5, g.update(-1, 2.125, 0), 1e-9);
    assertEquals(1, g.update(-1, 2.5, 0), 0);
    assertEquals(0, g.update(1, 3, 0), 0);
    assertEquals(0, g.update(1, 3.75, 0), 0);
    assertEquals(1, g.update(1, 4.5, 0), 0);
  }

  @Test public void flywheelsMustSlowDownEvenAfterTheMinimumCoastTime() {
    MotorReversalGuard g = new MotorReversalGuard(0.5, 0.75, 100);
    g.update(1, 0, 0);
    assertEquals(0, g.update(-1, 1, 4000), 0);
    assertEquals(0, g.update(-1, 3, 101), 0);
    assertEquals(0, g.update(-1, 4, 99), 0);
    assertEquals(0.5, g.update(-1, 4.375, 50), 1e-9);
    assertEquals(1, g.update(-1, 4.75, 2000), 0);
    // Releasing eject also coasts before driving forward.
    assertEquals(0, g.update(1, 5, 2000), 0);
    assertEquals(0, g.update(1, 6, 500), 0);
    assertEquals(0, g.update(1, 7, 90), 0);
    assertEquals(1, g.update(1, 7.75, 2000), 0);
  }

  @Test public void stopDoesNotForgetThePreviousDirection() {
    MotorReversalGuard g = new MotorReversalGuard(0.5, 0.75, 100);
    g.update(1, 0, 0);
    assertEquals(0, g.update(0, 1, 4000), 0);
    assertEquals(0, g.update(-1, 1.01, 4000), 0);
    assertEquals(0, g.update(-1, 2, 4000), 0);
  }

  @Test public void cancellingOrChangingDirectionDuringARampIsSafe() {
    MotorReversalGuard g = new MotorReversalGuard(0.5, 0.75, 100);
    g.update(1, 0, 0);
    g.update(-1, 1, 4000);
    assertEquals(1, g.update(1, 1.1, 3900), 0);
    g.update(-1, 2, 0);
    g.update(-1, 2.5, 0);
    assertEquals(0.5, g.update(-1, 2.875, 1000), 1e-9);
    assertEquals(0, g.update(1, 3, 1000), 0);
    assertEquals(0, g.update(1, 3.5, 500), 0);
  }

  @Test public void startupCannotReverseFlywheelsStillCoastingFromThePreviousOpMode() {
    MotorReversalGuard g = new MotorReversalGuard(0.5, 0.75, 100);
    assertEquals(0, g.update(-1, 0, 4000), 0);
    assertEquals(0, g.update(-1, 1, 500), 0);
    assertEquals(0, g.update(-1, 2, 90), 0);
    assertEquals(0.5, g.update(-1, 2.375, 500), 1e-9);
  }

  @Test public void changingRequestsCannotBypassTheUnknownStartupDirection() {
    MotorReversalGuard g = new MotorReversalGuard(0.5, 0.75, 100);
    assertEquals(0, g.update(1, 0, 4000), 0);
    assertEquals(0, g.update(-1, 0.02, 3900), 0);
    assertEquals(0, g.update(1, 0.03, 3800), 0);
    assertEquals(0, g.update(-1, 2, 90), 0);
    assertEquals(0.5, g.update(-1, 2.375, 500), 1e-9);
  }

  @Test public void stopDuringRampDoesNotAllowAnImmediateFullPowerRestart() {
    MotorReversalGuard g = new MotorReversalGuard(0.5, 0.75, 100);
    g.update(1, 0, 0);
    g.update(-1, 1, 1000);
    g.update(-1, 2, 0);
    g.update(0, 2.1, 20);
    assertEquals(0.5, g.update(-1, 2.375, 50), 1e-9);
  }

  @Test public void anIntakeStoppedLongEnoughEjectsAtOnce() {
    MotorReversalGuard g = new MotorReversalGuard(0.3, 0.2, 0);
    g.update(1, 0, 0);
    g.update(0, 1, 0); // LT let go
    assertEquals(0, g.update(-1, 5, 0), 0); // B: the ramp starts this loop, no coast
    assertEquals(0.5, g.update(-1, 5.1, 0), 1e-9);
    assertEquals(1, g.update(-1, 5.2, 0), 0);
  }

  @Test public void aBriefStopOnlyCountsForTheTimeItWasStopped() {
    MotorReversalGuard g = new MotorReversalGuard(0.3, 0.2, 0);
    g.update(1, 0, 0);
    g.update(0, 1, 0);
    assertEquals(0, g.update(-1, 1.1, 0), 0); // Stopped 0.1 s of the 0.3
    assertEquals(0, g.update(-1, 1.29, 0), 0);
    assertEquals(0, g.update(-1, 1.3, 0), 0); // Coasted 0.3 s from the stop, not from B
    assertEquals(0.5, g.update(-1, 1.4, 0), 1e-9);
  }

  @Test public void intakingStraightToEjectIsFullyReversedInHalfASecond() {
    MotorReversalGuard g = new MotorReversalGuard(0.3, 0.2, 0);
    g.update(1, 0, 0);
    assertEquals(0, g.update(-1, 1, 0), 0);
    assertEquals(0, g.update(-1, 1.29, 0), 0);
    assertEquals(0, g.update(-1, 1.3, 0), 0);
    assertEquals(1, g.update(-1, 1.5, 0), 1e-9);
  }

  @Test public void stoppedTimeStillWaitsForFlywheelsToSlow() {
    MotorReversalGuard g = new MotorReversalGuard(0.5, 0.75, 100);
    g.update(1, 0, 0);
    g.update(0, 1, 3000);
    assertEquals(0, g.update(-1, 5, 3000), 0);
  }

  @Test public void invalidSpeedCannotPermitAReversal() {
    MotorReversalGuard g = new MotorReversalGuard(0.5, 0.75, 100);
    g.update(1, 0, 0);
    g.update(-1, 1, 0);
    assertEquals(0, g.update(-1, 2, Double.NaN), 0);
    assertEquals(0, g.update(-1, 3, Double.POSITIVE_INFINITY), 0);
  }
}
