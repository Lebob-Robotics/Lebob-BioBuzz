package org.firstinspires.ftc.teamcode.utils;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class StackPressureTest {
  private static final double DT = 0.05;

  @Test
  public void catches225900sPushBeforeTheBallWentIn() {
    // 13 Nov 225900: from 72.81 s the intake held 4.4-5.8 A; the first ball dipped the wheels at about 73.40
    double[] amps = {5.83, 4.58, 4.45, 4.35, 4.61, 4.82, 4.66, 4.49, 4.60, 4.80};
    StackPressure p = new StackPressure();
    double t = 72.81;
    double packedAt = Double.NaN;
    for (double a : amps) {
      if (p.update(t, true, a) && Double.isNaN(packedAt)) {
        packedAt = t;
      }
      t += DT;
    }
    assertTrue("packed at " + packedAt, packedAt < 73.35);
  }

  @Test
  public void takingABallInIsNotPacked() {
    StackPressure p = new StackPressure();
    double t = 0;
    for (int i = 0; i < 40; i++, t += DT) {
      double a = i % 8 < 6 ? 6.0 : 2.0; // 0.3 s over the line, then free, again and again
      assertFalse(p.update(t, true, a));
    }
  }

  @Test
  public void onlyWatchesWhileLoading() {
    StackPressure p = new StackPressure();
    double t = 0;
    for (int i = 0; i < 20; i++, t += DT) {
      assertFalse(p.update(t, i % 5 != 0, 5.0)); // Each pause starts it over
    }
    assertFalse(new StackPressure().update(0, true, Double.NaN));
  }
}
