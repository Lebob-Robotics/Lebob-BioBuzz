package org.firstinspires.ftc.teamcode.utils;

import static org.junit.Assert.assertEquals;
import org.junit.Test;

public class RpmTrendTest {
  @Test public void steadySpeedPredictsItself() {
    RpmTrend trend = new RpmTrend(0.2);
    trend.update(0.00, 3443);
    trend.update(0.08, 3443);
    trend.update(0.16, 3443);
    assertEquals(3443, trend.predicted(0.08), 1e-9);
  }

  @Test public void recoveringWheelsArePredictedPastTheTarget() {
    // 080706, ball 2: back up from the last ball's dip, at target and still climbing
    RpmTrend trend = new RpmTrend(0.2);
    trend.update(458.86, 3198);
    trend.update(458.97, 3412);
    trend.update(459.05, 3443);
    assertEquals(3443 + 245 / 0.19 * 0.08, trend.predicted(0.08), 1e-6);
  }

  @Test public void oldSamplesLeaveTheWindow() {
    RpmTrend trend = new RpmTrend(0.2);
    trend.update(0.00, 3000);
    trend.update(0.30, 3400);
    trend.update(0.40, 3400);
    assertEquals(3400, trend.predicted(0.08), 1e-9);
  }

  @Test public void oneSampleHasNoSlope() {
    RpmTrend trend = new RpmTrend(0.2);
    trend.update(1.0, 3500);
    assertEquals(3500, trend.predicted(0.08), 1e-9);
  }
}
