package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem.SpinMode;
import org.firstinspires.ftc.teamcode.utils.ShootOnTheMoveConstants;
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
  public void volleyRecoveryLeavesABallsDipToSpeedControlAndHandsBackEarly() {
    // 12 Nov, 211957: at 200/100 every 300-500 RPM dip went full power, overshot ~600 and rang for 0.8 s
    double open = ShootOnTheMoveConstants.VOLLEY_OPEN_LOOP_BELOW_RPM;
    double back = ShootOnTheMoveConstants.VOLLEY_HANDBACK_RPM;
    assertEquals(SpinMode.SPEED, ShooterSubsystem.nextSpinMode(SpinMode.SPEED, -500, open, back));
    SpinMode mode = ShooterSubsystem.nextSpinMode(SpinMode.SPEED, -900, open, back); // Two balls' dip
    assertEquals(SpinMode.SPIN_UP, mode);
    mode = ShooterSubsystem.nextSpinMode(mode, -500, open, back);
    assertEquals(SpinMode.SPIN_UP, mode);
    // Handed back ~450 short: the ~0.1 s reading lag at ~5500 RPM/s carries the wheels the rest of the way
    assertEquals(SpinMode.SPEED, ShooterSubsystem.nextSpinMode(mode, -400, open, back));
  }

  @Test
  public void spinUpThatOvershootsBrakes() {
    assertEquals(SpinMode.FAR_ABOVE, ShooterSubsystem.nextSpinMode(SpinMode.SPIN_UP, 200));
  }

  @Test
  public void volleyThresholdsRecoverABallsDipAtFullPower() {
    // A 300 RPM dip stays under speed control with the defaults, but a volley spins it up at full power
    assertEquals(SpinMode.SPEED, ShooterSubsystem.nextSpinMode(SpinMode.SPEED, -300));
    SpinMode mode = ShooterSubsystem.nextSpinMode(SpinMode.SPEED, -300, 200, 100);
    assertEquals(SpinMode.SPIN_UP, mode);
    mode = ShooterSubsystem.nextSpinMode(mode, -150, 200, 100);
    assertEquals(SpinMode.SPIN_UP, mode);
    mode = ShooterSubsystem.nextSpinMode(mode, -60, 200, 100);
    assertEquals(SpinMode.SPEED, mode);
  }

  @Test
  public void volleyThresholdsBrakeAnOvershootUnderSpeedControl() {
    SpinMode mode = ShooterSubsystem.nextSpinMode(SpinMode.SPIN_UP, 150, 200, 100);
    assertEquals(SpinMode.FAR_ABOVE, mode);
    mode = ShooterSubsystem.nextSpinMode(mode, 90, 200, 100);
    assertEquals(SpinMode.SPEED, mode);
  }
}
