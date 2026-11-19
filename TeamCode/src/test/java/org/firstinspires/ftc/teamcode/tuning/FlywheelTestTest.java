package org.firstinspires.ftc.teamcode.tuning;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import org.junit.Test;

public class FlywheelTestTest {
  private static FlywheelTest run(double tau, double offsetRpm) {
    FlywheelTest t = new FlywheelTest();
    double rpm = 0;
    double now = 0;
    t.start(now, rpm);
    while (t.running()) {
      double goal = t.targetRpm() + offsetRpm;
      rpm += (goal - rpm) * (1 - Math.exp(-0.02 / tau));
      now += 0.02;
      t.record(now, Math.round(rpm / 61.0) * 61.0); // The encoder's steps
    }
    return t;
  }

  @Test
  public void aQuickWheelOnTargetIsGood() {
    FlywheelTest t = run(0.15, 0);
    assertEquals(FlywheelTest.PLAN_RPM.length, t.steps().size());
    double[] s = new double[3];
    List<String> advice = FlywheelTest.advice(t.steps(), new ArrayList<>(), 150, 12.9, 150, s);
    assertTrue(advice.toString(), advice.get(0).startsWith("GOOD"));
  }

  @Test
  public void holdingFastLowersF() {
    FlywheelTest t = run(0.15, 80);
    double[] s = new double[3];
    List<String> advice = FlywheelTest.advice(t.steps(), new ArrayList<>(), 150, 12.9, 150, s);
    assertTrue(advice.toString(), advice.get(0).startsWith("CHANGE"));
    assertTrue(s[1] < 12.9 && s[1] > 12.4);
  }

  @Test
  public void aSlowWheelRaisesP() {
    FlywheelTest t = run(0.9, 0);
    double[] s = new double[3];
    FlywheelTest.advice(t.steps(), new ArrayList<>(), 150, 12.9, 150, s);
    assertEquals(180, s[0], 1e-9);
  }

  @Test
  public void aFullPowerSpinUpThatOvershootsRaisesTheHandback() {
    List<FlywheelTest.StepResult> steps = new ArrayList<>();
    FlywheelTest t = run(0.15, 0);
    for (FlywheelTest.StepResult s : t.steps()) {
      steps.add(s);
    }
    double[] s = new double[3];
    StepResponse r = new StepResponse(0, 3000, 4000, 50);
    r.add(0.2, 4300);
    r.add(1.0, 4000);
    steps.add(new FlywheelTest.StepResult(3000, 4000, r));
    List<String> advice = FlywheelTest.advice(steps, new ArrayList<>(), 150, 12.9, 150, s);
    assertTrue(advice.toString(), advice.get(1).contains("HANDBACK_RPM"));
    assertEquals(200, s[2], 1e-9);
    assertEquals(150, s[0], 1e-9); // Not the speed control's fault
  }

  @Test
  public void ballsAreReported() {
    List<FlywheelTest.BallResult> balls = new ArrayList<>();
    balls.add(new FlywheelTest.BallResult(420, 0.6));
    double[] s = new double[3];
    List<String> advice = FlywheelTest.advice(new ArrayList<>(), balls, 150, 12.9, 150, s);
    assertTrue(advice.toString(), advice.get(advice.size() - 1).contains("420"));
  }
}
