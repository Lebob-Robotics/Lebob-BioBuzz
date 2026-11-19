package org.firstinspires.ftc.teamcode.shooting;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class SotmSpeedLimitTest {
  private static boolean hooded(double d, double toward, double sideways) {
    Shot s = ShootOnTheMove.calculate(d, toward, sideways);
    return s.valid && !ShootOnTheMove.tooCloseToShootMoving(s);
  }

  @Test
  public void aSpeedWithAShotIsLeftAlone() {
    assertTrue(hooded(1.0, 0, 0.3) && hooded(1.0, 0, 0.36));
    assertEquals(1.0, SotmSpeedLimit.share(false, 1.0, 0, 0.3, 1.2, false, 0), 0);
  }

  @Test
  public void strafingTooFastIsCutToTheFastestWithAShot() {
    double k = SotmSpeedLimit.share(false, 1.0, 0, 2.0, 1.2, false, 0);
    assertTrue(k > 0.2 && k < 1);
    assertTrue(hooded(1.0, 0, 2.0 * k * 1.2));
    assertFalse(hooded(1.0, 0, 2.0 * (k + 0.03) * 1.2)); // And no slower than it need be
    assertEquals(Shot.Reason.SIDEWAYS_TOO_FAST, ShootOnTheMove.calculate(1.0, 0, 2.0).reason);
  }

  @Test
  public void drivingAtTheCellTooFastIsCut() {
    double k = SotmSpeedLimit.share(false, 1.5, 3.0, 0, 1.2, false, 0);
    assertTrue(k < 1);
    assertTrue(hooded(1.5, 3.0 * k * 1.2, 0));
  }

  @Test
  public void closeToTheLipItCrawlsUnlessThatGateIsBypassed() {
    double k = SotmSpeedLimit.share(false, 0.5, 0, 1.0, 1.2, false, 0);
    assertTrue(1.0 * k * 1.2 < FireConstants.MOVING_SHOT_SPEED_MPS);
    assertTrue(SotmSpeedLimit.share(false, 0.5, 0, 1.0, 1.2, true, 0) > k);
  }

  @Test
  public void theBackwardShotKeepsToItsSpeed() {
    double k = SotmSpeedLimit.share(true, 0.6, 0.5, 0.5, 1.2, false, 0);
    assertTrue(0.5 * k * 1.2 <= BackwardShotConstants.BACKWARD_MAX_SPEED_MPS + 1e-9);
    assertTrue(k > 0);
  }

  @Test
  public void neverSlowerThanTheLeastSpeed() {
    assertEquals(0.0, SotmSpeedLimit.share(false, 2.0, -3, 0, 1.2, false, 0), 0); // No speed backing out has a shot
    assertEquals(0.25 / 3, SotmSpeedLimit.share(false, 2.0, -3, 0, 1.2, false, 0.25), 1e-9);
    assertEquals(1.0, SotmSpeedLimit.share(false, 2.0, -0.1, 0, 1.2, false, 0.25), 0);
  }

  @Test
  public void outOfRangeTheDriverCanStillDriveIn() {
    assertEquals(1.0, SotmSpeedLimit.share(false, 10.0, 2.0, 0, 1.2, false, 0), 0);
    assertEquals(1.0, SotmSpeedLimit.share(false, 1.0, 0, 0, 1.2, false, 0), 0);
  }
}
