package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import org.junit.Test;

public class HoodMotorTargetingTest {
  private static final int DEADBAND = 45;
  private static final int TEN_DEG = 2276; // REV ticks
  private static final int TWENTY_DEG = 4552;

  private static int revFor(int motorTicks) {
    return (int) Math.round(motorTicks / HoodMotorTargeting.MOTOR_PER_REV);
  }

  @Test public void reAnchorsOnTheRevAtRestToTakeUpBacklash() {
    HoodMotorTargeting t = new HoodMotorTargeting();
    assertEquals(107, t.motorTargetFor(TEN_DEG, 0, DEADBAND));
    // The motor arrived at 107 but 16 ticks of belt lost motion left the pinion short.
    t.observe(revFor(91), 107, true, TEN_DEG, DEADBAND);
    assertEquals(16, t.offsetUp());
    assertEquals(123, t.motorTargetFor(TEN_DEG, revFor(91), DEADBAND)); // Drive the extra 16
  }

  @Test public void ignoresReadingsWhileMoving() {
    HoodMotorTargeting t = new HoodMotorTargeting();
    t.motorTargetFor(TEN_DEG, 0, DEADBAND);
    t.observe(0, 40, false, TEN_DEG, DEADBAND); // Cross-hub read skew mid-move must not move the target
    assertEquals(0, t.offset());
    assertEquals(107, t.motorTargetFor(TEN_DEG, 0, DEADBAND));
  }

  @Test public void settledInsideDeadbandDoesNotHuntByOneTick() {
    HoodMotorTargeting t = new HoodMotorTargeting();
    t.motorTargetFor(TEN_DEG, 0, DEADBAND);
    t.observe(TEN_DEG - 30, 106, true, TEN_DEG, DEADBAND);
    assertEquals(0, t.offset());
    assertEquals(107, t.motorTargetFor(TEN_DEG, TEN_DEG - 30, DEADBAND));
  }

  @Test public void reversalUsesTheOtherDirectionsOffsetAtOnce() {
    HoodMotorTargeting t = new HoodMotorTargeting();
    // Up to 20 degrees: belt loaded on its up side, motor 18 ticks ahead of the pinion.
    t.motorTargetFor(TWENTY_DEG, 0, DEADBAND);
    t.observe(TWENTY_DEG - 400, 214 - 19 + 18, true, TWENTY_DEG, DEADBAND);
    assertEquals(18, t.offsetUp());
    // Down to 10: first time down, so no down offset learned yet; it stops 1.5 degrees high.
    assertEquals(107, t.motorTargetFor(TEN_DEG, TWENTY_DEG, DEADBAND));
    t.observe(TEN_DEG + 341, 107, true, TEN_DEG, DEADBAND);
    assertEquals(-16, t.offsetDown());
    // Every later move picks its own direction's offset immediately, with no second correction.
    assertEquals(214 + 18, t.motorTargetFor(TWENTY_DEG, TEN_DEG, DEADBAND));
    assertEquals(107 - 16, t.motorTargetFor(TEN_DEG, TWENTY_DEG, DEADBAND));
  }

  @Test public void reversalNotYetStartedDoesNotRelearnTheNewDirection() {
    HoodMotorTargeting t = new HoodMotorTargeting();
    // Settled after a downward move: belt loaded down, motor 11 ticks behind the pinion.
    t.motorTargetFor(TEN_DEG, TWENTY_DEG, DEADBAND);
    t.observe(revFor(118), 107, true, TEN_DEG, DEADBAND);
    assertEquals(-11, t.offsetDown());
    // New target above: the up move is chosen, but the motor has not moved yet (moveDone false),
    // so the down-loaded reading must not become the up offset.
    t.motorTargetFor(TWENTY_DEG, revFor(118), DEADBAND);
    t.observe(revFor(118), 107, false, TWENTY_DEG, DEADBAND);
    assertEquals(0, t.offsetUp());
  }

  @Test public void overshootWhileStillMovingKeepsTheDirection() {
    HoodMotorTargeting t = new HoodMotorTargeting();
    t.motorTargetFor(TWENTY_DEG, 0, DEADBAND);
    t.observe(TWENTY_DEG - 400, 214 - 19 + 18, true, TWENTY_DEG, DEADBAND); // Up offset 18
    t.motorTargetFor(TEN_DEG, TWENTY_DEG, DEADBAND);
    t.observe(TEN_DEG + 341, 107, true, TEN_DEG, DEADBAND); // Down offset -16
    // Up to 20 again from rest, then the hood swings past it while the motor still moves: no switch to the down offset
    assertEquals(214 + 18, t.motorTargetFor(TWENTY_DEG, TEN_DEG, DEADBAND, false));
    assertEquals(214 + 18, t.motorTargetFor(TWENTY_DEG, TWENTY_DEG - 1000, DEADBAND, true));
    assertEquals(214 + 18, t.motorTargetFor(TWENTY_DEG, TWENTY_DEG + 300, DEADBAND, true));
    // Once it stops past the target, the correction back down uses the down offset
    assertEquals(214 - 16, t.motorTargetFor(TWENTY_DEG, TWENTY_DEG + 300, DEADBAND, false));
  }

  @Test public void insideDeadbandKeepsTheLastDirection() {
    HoodMotorTargeting t = new HoodMotorTargeting();
    t.motorTargetFor(TEN_DEG, 0, DEADBAND);
    t.observe(revFor(91), 107, true, TEN_DEG, DEADBAND);
    // A target 20 ticks below the hood is inside the deadband: still the up offset, no flip.
    assertEquals(16, t.motorTargetFor(TEN_DEG - 20, TEN_DEG, DEADBAND)
        - (int) Math.round((TEN_DEG - 20) * HoodMotorTargeting.MOTOR_PER_REV));
  }
}
