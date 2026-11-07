package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem.SpinMode;
import org.junit.Test;

public class ShooterSpinModeTest {
  @Test
  public void spinsUpOpenLoopFromFarBelowAndHandsBackNearTheTarget() {
    // 7 Nov, 094905: spun up under speed control, the wheels then ran 100-340 RPM fast for 20 s
    SpinMode mode = SpinMode.SPEED;
    mode = ShooterSubsystem.nextSpinMode(mode, -3300);
    assertEquals(SpinMode.SPIN_UP, mode);
    mode = ShooterSubsystem.nextSpinMode(mode, -300); // Inside OPEN_LOOP_BELOW_RPM but not yet near
    assertEquals(SpinMode.SPIN_UP, mode);
    mode = ShooterSubsystem.nextSpinMode(mode, -100);
    assertEquals(SpinMode.SPEED, mode);
  }

  @Test
  public void smallErrorsStayUnderSpeedControl() {
    assertEquals(SpinMode.SPEED, ShooterSubsystem.nextSpinMode(SpinMode.SPEED, -350)); // A ball's dip
    assertEquals(SpinMode.SPEED, ShooterSubsystem.nextSpinMode(SpinMode.SPEED, 350));
  }

  @Test
  public void farAboveBrakesUnderSpeedControlThenRestartsNearTheTarget() {
    SpinMode mode = ShooterSubsystem.nextSpinMode(SpinMode.SPEED, 3900); // Cell found after top speed
    assertEquals(SpinMode.FAR_ABOVE, mode);
    mode = ShooterSubsystem.nextSpinMode(mode, 200);
    assertEquals(SpinMode.FAR_ABOVE, mode);
    mode = ShooterSubsystem.nextSpinMode(mode, 120);
    assertEquals(SpinMode.SPEED, mode);
  }

  @Test
  public void spinUpThatOvershootsBrakes() {
    assertEquals(SpinMode.FAR_ABOVE, ShooterSubsystem.nextSpinMode(SpinMode.SPIN_UP, 200));
  }
}
