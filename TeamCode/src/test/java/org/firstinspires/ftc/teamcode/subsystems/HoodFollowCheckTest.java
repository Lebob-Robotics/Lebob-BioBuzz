package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class HoodFollowCheckTest {
  private static int rev(double motorTicks) {
    return (int) Math.round(motorTicks / HoodMotorTargeting.MOTOR_PER_REV);
  }

  @Test public void deadRevIsCaughtWithinOneWindow() {
    // 085313: the REV read 0 to 3 throughout while the motor ran up
    HoodFollowCheck check = new HoodFollowCheck();
    assertFalse(check.update(0, 0));
    assertFalse(check.update(3, 59));
    assertTrue(check.update(3, 75));
  }

  @Test public void matchedTravelWithBacklashNeverFires() {
    HoodFollowCheck check = new HoodFollowCheck();
    check.update(0, 0);
    // Up 500 ticks, then down with 16 ticks of backlash before the REV turns, in 5-tick steps
    for (int m = 0; m <= 500; m += 5) {
      assertFalse(check.update(rev(m), m));
    }
    for (int m = 500; m >= 0; m -= 5) {
      assertFalse(check.update(rev(Math.min(500, m + 16)), m));
    }
  }

  @Test public void partialSlipAboveTheShareIsLeftToTheGuard() {
    HoodFollowCheck check = new HoodFollowCheck();
    check.update(0, 0);
    for (int m = 0; m <= 300; m += 10) {
      assertFalse(check.update(rev(m * 0.6), m));
    }
  }

  @Test public void revMovingAgainstTheMotorFires() {
    HoodFollowCheck check = new HoodFollowCheck();
    check.update(0, 0);
    assertTrue(check.update(rev(-20), 60));
  }

  @Test public void standingStillNeverFires() {
    HoodFollowCheck check = new HoodFollowCheck();
    for (int i = 0; i < 100; i++) {
      assertFalse(check.update(1234, 56));
    }
  }
}
