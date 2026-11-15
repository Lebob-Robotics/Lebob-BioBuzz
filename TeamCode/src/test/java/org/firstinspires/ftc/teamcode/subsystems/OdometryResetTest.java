package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class OdometryResetTest {
  private static double[] pose(double x, double y, double headingDeg) {
    return new double[] {x, y, Math.toRadians(headingDeg)};
  }

  @Test
  public void catches223434sLateReset() {
    // The read the start pose was set against was 22:32's end (turned -35 degrees); the first read after the
    // reset was already 3 degrees turned
    assertTrue(OdometrySubsystem.cameBackReset(pose(-0.28, -1.52, -35), pose(0.0, 0.001, -3.1)));
  }

  @Test
  public void aResetFromZeroNeedsNothing() {
    // 22:32: no good read before START, so the start pose was set against zero and the reset changed nothing
    assertFalse(OdometrySubsystem.cameBackReset(pose(0, 0, 0), pose(0.0, 0.002, -1.5)));
  }

  @Test
  public void ordinaryBadReadsAreNotAReset() {
    // A bad read or two while driving: the pose carries on from near where it was
    assertFalse(OdometrySubsystem.cameBackReset(pose(1.20, 0.40, 30), pose(1.24, 0.41, 31)));
    // Driving back over the start spot: near zero, but no jump
    assertFalse(OdometrySubsystem.cameBackReset(pose(0.08, 0.0, 2), pose(0.02, 0.0, 1)));
  }
}
