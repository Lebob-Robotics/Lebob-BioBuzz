package org.firstinspires.ftc.teamcode.tuning;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import org.junit.Test;

public class HeadingLockTestTest {
  private static final double DT = 0.02;

  /** Runs the test on a heading that follows the target by a first-order lag tau plus lagRad along its rate. */
  private static HeadingLockTest run(double tau, double lagPerRadPerSec, double overshootGain) {
    HeadingLockTest t = new HeadingLockTest();
    t.start(0, 1.0);
    double heading = 1.0;
    double rate = 0;
    double now = 0;
    while (t.running() && now < 60) {
      double[] target = t.target(now);
      double goal = target[0] - lagPerRadPerSec * target[1];
      // Second-order when overshootGain > 0: under-damped
      double accel = (goal - heading) / (tau * tau) + (2 * (1 - overshootGain) / tau) * (target[1] - rate);
      rate += accel * DT;
      heading += rate * DT;
      now += DT;
      t.record(now, heading);
    }
    assertTrue(t.finished());
    return t;
  }

  @Test
  public void aQuickWellDampedLockIsGood() {
    HeadingLockTest t = run(0.08, 0, 0);
    assertEquals(HeadingLockTest.PLAN.length, t.results().size());
    double[] s = new double[4];
    List<String> advice = HeadingLockTest.advice(t.results(), 1.0, 0.06, 0.16, 0.16, s);
    assertTrue(advice.toString(), advice.get(0).startsWith("GOOD"));
    HeadingLockTest.Result first = t.results().get(0);
    assertTrue(first.reachS > 0 && first.reachS < 0.6);
    assertEquals(0, first.endErrorDeg, 0.2);
  }

  @Test
  public void overshootLowersKp() {
    HeadingLockTest t = run(0.1, 0, 0.8);
    double[] s = new double[4];
    List<String> advice = HeadingLockTest.advice(t.results(), 1.0, 0.06, 0.16, 0.16, s);
    assertTrue(advice.toString(), advice.get(0).startsWith("CHANGE"));
    assertTrue(s[0] < 1.0);
    assertTrue(s[1] > 0.06);
  }

  @Test
  public void trailingAMovingTargetRaisesKvByTheLag() {
    // 3 degrees behind per rad/s: KP 1 * lag / rate = 0.052 more KV
    HeadingLockTest t = run(0.05, Math.toRadians(3), 0);
    HeadingLockTest.Result fast = t.results().get(t.results().size() - 1);
    assertTrue(fast.ramp);
    assertEquals(6, fast.lagDeg, 0.8);
    double[] s = new double[4];
    HeadingLockTest.advice(t.results(), 1.0, 0.06, 0.16, 0.16, s);
    assertEquals(0.16 + Math.toRadians(3), s[2], 0.02);
  }

  @Test
  public void runningAheadOfAMovingTargetLowersKv() {
    HeadingLockTest t = run(0.05, -Math.toRadians(3), 0);
    double[] s = new double[4];
    List<String> advice = HeadingLockTest.advice(t.results(), 1.0, 0.06, 0.16, 0.16, s);
    assertTrue(advice.toString(), advice.get(1).startsWith("Runs ahead of"));
    assertEquals(0.16 - Math.toRadians(3), s[2], 0.02);
  }

  @Test
  public void stoppingShortRaisesKs() {
    List<HeadingLockTest.Result> results = new ArrayList<>();
    results.add(new HeadingLockTest.Result("step to +30 deg", false, 0, 0.4, Double.NaN, 0, -2.0, Double.NaN,
        Double.NaN));
    double[] s = new double[4];
    List<String> advice = HeadingLockTest.advice(results, 1.0, 0.06, 0.16, 0.16, s);
    assertTrue(advice.toString(), advice.get(0).startsWith("CHANGE"));
    assertEquals(0.18, s[3], 1e-9);
  }
}
