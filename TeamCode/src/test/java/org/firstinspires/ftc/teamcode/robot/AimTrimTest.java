package org.firstinspires.ftc.teamcode.robot;

import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.shooting.AimConstants;
import org.firstinspires.ftc.teamcode.shooting.BackwardShotConstants;
import org.junit.Test;

public class AimTrimTest {
  @Test
  public void teleOpAimsWithTheStandingTrimsTheToolsFound() {
    // 7 Nov: both tuning tools fired with +2.0 hooded and +1.5 backward; TeleOp fired with none
    assertEquals(AimConstants.HOODED_HEADING_TRIM_DEG, ShooterControl.aimTrimDeg(false, 0, false), 1e-9);
    assertEquals(BackwardShotConstants.BACKWARD_HEADING_TRIM_DEG, ShooterControl.aimTrimDeg(false, 0, true), 1e-9);
  }

  @Test
  public void tuningToolsKeepTheirOwnTrim() {
    assertEquals(-0.75, ShooterControl.aimTrimDeg(true, -0.75, false), 1e-9);
    assertEquals(0, ShooterControl.aimTrimDeg(true, 0, true), 1e-9);
  }

  @Test
  public void movingHoodTrimFollowsDistanceAndFadesAtRest() {
    // 7 Nov spots: near shots needed the hood lower, far ones higher; toward-right 0.50 at 0.91 m scored only -3.7..-1.5
    assertEquals(-2.3, ShooterControl.movingHoodTrimDeg(0.91, 0.5), 1e-9);
    assertEquals(-0.8, ShooterControl.movingHoodTrimDeg(1.215, 0.5), 1e-6);
    assertEquals(0.7, ShooterControl.movingHoodTrimDeg(1.6, 0.3), 1e-9);
    assertEquals(-1.15, ShooterControl.movingHoodTrimDeg(0.9, 0.15), 1e-9); // Half the moving shot speed, half the trim
    assertEquals(0, ShooterControl.movingHoodTrimDeg(1.0, 0), 1e-9); // Standing shots keep the table's
    assertEquals(0, ShooterControl.movingHoodTrimDeg(Double.NaN, 0.5), 1e-9);
  }
}
