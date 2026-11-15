package org.firstinspires.ftc.teamcode.field;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class AnglesTest {
  @Test
  public void normalizeKeepsAnglesBetweenMinusPiAndPi() {
    assertEquals(0.5, Angles.normalize(0.5 + 2 * Math.PI), 1e-12);
    assertEquals(-0.5, Angles.normalize(-0.5 - 4 * Math.PI), 1e-12);
    assertEquals(Math.PI - 0.1, Angles.normalize(-Math.PI - 0.1), 1e-12);
  }
}
