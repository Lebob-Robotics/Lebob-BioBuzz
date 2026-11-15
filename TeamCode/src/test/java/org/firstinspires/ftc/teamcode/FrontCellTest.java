package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.utils.VisionCorrector.Cell;
import org.junit.Test;

public class FrontCellTest {
  @Test
  public void aimsAtTheCellOnTheRobotsSideOfTheHive() {
    // 13 Nov 175517 at 64.2 s: from (1.18, 0.93) the volley went to the SCORING Cell at x -0.44, past the near one
    assertEquals(Cell.AUDIENCE, Robot.frontCell(null, 1.18));
    // 225900 at 76.4 s: from (-1.48, -1.20) it went to AUDIENCE at x +0.44
    assertEquals(Cell.SCORING, Robot.frontCell(null, -1.48));
  }

  @Test
  public void changesSidesOnlyWellPastTheMiddle() {
    assertEquals(Cell.AUDIENCE, Robot.frontCell(Cell.AUDIENCE, -0.10));
    assertEquals(Cell.SCORING, Robot.frontCell(Cell.AUDIENCE, -0.20));
    assertEquals(Cell.SCORING, Robot.frontCell(Cell.SCORING, 0.10));
    assertEquals(Cell.AUDIENCE, Robot.frontCell(Cell.SCORING, 0.20));
  }
}
