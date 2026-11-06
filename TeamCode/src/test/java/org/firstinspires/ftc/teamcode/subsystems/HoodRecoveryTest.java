package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class HoodRecoveryTest {
  @Test public void fullSpeedUntilSomethingGoesWrong() {
    assertEquals(1.0, new HoodRecovery(3).speed(1.0, 0.6, 0.3), 1e-9);
  }

  @Test public void eachSlipSlowsTheNextMoveDownToTheFloor() {
    HoodRecovery recovery = new HoodRecovery(3);
    recovery.onSlip();
    assertEquals(0.6, recovery.speed(1.0, 0.6, 0.3), 1e-9);
    recovery.onSlip();
    assertEquals(0.36, recovery.speed(1.0, 0.6, 0.3), 1e-9);
    recovery.onSlip();
    assertEquals(0.3, recovery.speed(1.0, 0.6, 0.3), 1e-9);
  }

  @Test public void arrivingRestoresFullSpeedAndTheRehomes() {
    HoodRecovery recovery = new HoodRecovery(3);
    recovery.onSlip();
    assertTrue(recovery.onTrip());
    recovery.onArrived();
    assertEquals(1.0, recovery.speed(1.0, 0.6, 0.3), 1e-9);
    assertEquals(0, recovery.rehomes());
  }

  @Test public void aTripRehomesThenStaysFaultedAfterTheLimit() {
    HoodRecovery recovery = new HoodRecovery(2);
    assertTrue(recovery.onTrip());
    assertTrue(recovery.onTrip());
    assertFalse(recovery.onTrip());
    assertFalse(recovery.onTrip());
  }

  @Test public void aRehomeStartsTheSlipCountAfresh() {
    HoodRecovery recovery = new HoodRecovery(3);
    recovery.onSlip();
    recovery.onSlip();
    recovery.onTrip();
    assertEquals(0, recovery.slipsSinceArrival());
    assertEquals(0.6, recovery.speed(1.0, 0.6, 0.3), 1e-9); // Still gentle: one re-home so far
  }

  @Test public void floorNeverExceedsTheConfiguredMaximum() {
    HoodRecovery recovery = new HoodRecovery(3);
    recovery.onSlip();
    assertEquals(0.2, recovery.speed(0.2, 0.6, 0.3), 1e-9);
  }
}
