package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.utils.ShootOnTheMoveConstants;
import org.junit.Test;

public class AimTrimTest {
  @Test
  public void teleOpAimsWithTheStandingTrimsTheToolsFound() {
    // 7 Nov: both tuning tools fired with +2.0 hooded and +1.5 backward; TeleOp fired with none
    assertEquals(ShootOnTheMoveConstants.HOODED_HEADING_TRIM_DEG, Robot.aimTrimDeg(false, 0, false), 1e-9);
    assertEquals(ShootOnTheMoveConstants.BACKWARD_HEADING_TRIM_DEG, Robot.aimTrimDeg(false, 0, true), 1e-9);
  }

  @Test
  public void tuningToolsKeepTheirOwnTrim() {
    assertEquals(-0.75, Robot.aimTrimDeg(true, -0.75, false), 1e-9);
    assertEquals(0, Robot.aimTrimDeg(true, 0, true), 1e-9);
  }

  @Test
  public void movingHoodTrimFollowsDistanceAndFadesAtRest() {
    // 7 Nov spots: near shots needed the hood lower, far ones higher; toward-right 0.50 at 0.91 m scored only -3.7..-1.5
    assertEquals(-2.3, Robot.movingHoodTrimDeg(0.91, 0.5), 1e-9);
    assertEquals(-0.8, Robot.movingHoodTrimDeg(1.215, 0.5), 1e-6);
    assertEquals(0.7, Robot.movingHoodTrimDeg(1.6, 0.3), 1e-9);
    assertEquals(-1.15, Robot.movingHoodTrimDeg(0.9, 0.15), 1e-9); // Half the moving shot speed, half the trim
    assertEquals(0, Robot.movingHoodTrimDeg(1.0, 0), 1e-9); // Standing shots keep the table's
    assertEquals(0, Robot.movingHoodTrimDeg(Double.NaN, 0.5), 1e-9);
  }
}
