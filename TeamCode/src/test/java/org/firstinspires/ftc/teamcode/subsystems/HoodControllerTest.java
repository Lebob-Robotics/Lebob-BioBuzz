package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class HoodControllerTest {
  private static final double DT = 0.005;

  /**
   * Hood driven by a DC motor through a slack belt: friction and back-EMF as fitted on the robot, and 1.5 degrees of slack the motor winds through before the hood moves. Read one step late.
   */
  private static final class Plant {
    double pos;
    double vel;
    double readPos;
    double readVel;
    double slack; // Where the motor sits in the belt's slack, -SLACK/2..SLACK/2 (hood degrees)
    static final double KS = 0.065, KV = 0.0037, KG = 0.002, SLACK = 1.5;
    static final double KA = 0.0001;

    Plant(double pos) { this.pos = pos; readPos = pos; slack = -SLACK / 2; }

    void step(double power) {
      readPos = pos;
      readVel = vel;
      double dir = Math.signum(power);
      if (vel == 0 && dir != 0 && slack * dir < SLACK / 2) {
        // Unloaded, the motor spins through the slack at its free speed for this power
        slack += dir * Math.max(0, Math.abs(power) - 0.02) / KV * DT;
        slack = Math.max(-SLACK / 2, Math.min(SLACK / 2, slack));
        return;
      }
      double drive = power - KG - KV * vel;
      if (vel == 0 && Math.abs(drive) <= KS) {
        return; // Static friction holds it
      }
      double accel = (drive - KS * Math.signum(vel != 0 ? vel : drive)) / KA;
      double newVel = vel + accel * DT;
      if (vel != 0 && Math.signum(newVel) != Math.signum(vel)) {
        newVel = 0; // Friction stops it rather than reversing it
      }
      pos += 0.5 * (vel + newVel) * DT;
      vel = newVel;
    }
  }

  private static HoodController.Gains gains() {
    HoodController.Gains g = new HoodController.Gains();
    // Feedforward a little off the plant's, as measured gains are; top speed below its 250 deg/s free speed
    g.maxVelDegPerSec = 200;
    g.accelDegPerSec2 = 4000;
    g.decelDegPerSec2 = 3000;
    g.kS = 0.06;
    g.kV = 0.0037;
    g.kA = 0.00002;
    g.kADecel = 0.00015;
    g.kG = 0;
    g.kP = 0.3;
    g.kD = 0.01;
    g.takeUpPower = 0.4;
    g.takeUpPerDeg = 0.05;
    g.takeUpVelDegPerSec = 15;
    g.takeUpDeg = 0.1;
    g.takeUpMaxS = 0.12;
    g.takeUpRampPerSec = 3;
    g.stuckS = 0.03;
    g.latencyS = DT;
    g.deadbandDeg = 0.1;
    g.reengageDeg = 0.2;
    g.resyncDeg = 4;
    g.maxPower = 1;
    return g;
  }

  /** Runs to the goal; returns {seconds to stay within 0.2, overshoot, final error, final power}. */
  private static double[] run(Plant p, HoodController c, double goal, double seconds) {
    double start = p.pos;
    double dir = Math.signum(goal - start);
    double within = Double.NaN;
    double overshoot = 0;
    double power = 0;
    for (int i = 0; i * DT < seconds; i++) {
      power = c.update(DT, p.readPos, p.readVel, goal, gains());
      p.step(power);
      overshoot = Math.max(overshoot, dir * (p.pos - goal));
      boolean in = Math.abs(p.pos - goal) <= 0.2;
      if (in && Double.isNaN(within)) { within = i * DT; }
      if (!in) { within = Double.NaN; }
    }
    return new double[] {within, overshoot, p.pos - goal, power};
  }

  @Test public void bigMoveArrivesFastWithoutOvershoot() {
    Plant p = new Plant(10);
    double[] r = run(p, new HoodController(), 30, 0.8);
    assertTrue("settled at " + r[0], r[0] < 0.3);
    assertTrue("overshoot " + r[1], r[1] < 0.2);
    assertTrue("final " + r[2], Math.abs(r[2]) <= 0.2);
    assertEquals(0, r[3], 0); // Settled: unpowered, the brake holds it
  }

  @Test public void bigMoveDownArrivesToo() {
    Plant p = new Plant(40);
    double[] r = run(p, new HoodController(), 20, 0.8);
    assertTrue("settled at " + r[0], r[0] < 0.3);
    assertTrue("overshoot " + r[1], r[1] < 0.2);
  }

  @Test public void smallCorrectionIsQuick() {
    Plant p = new Plant(30);
    double[] r = run(p, new HoodController(), 31, 0.4);
    assertTrue("settled at " + r[0], r[0] < 0.08);
    assertTrue("overshoot " + r[1], r[1] < 0.25); // Inside the deadband
  }

  @Test public void reversalThroughTheSlackDoesNotOvershoot() {
    Plant p = new Plant(30);
    p.slack = Plant.SLACK / 2; // Belt loaded upward by the last move
    double[] r = run(p, new HoodController(), 15, 0.8);
    assertTrue("settled at " + r[0], r[0] < 0.3);
    assertTrue("overshoot " + r[1], r[1] < 0.3);
  }

  @Test public void stuckShortItStartsAgain() {
    // Too little kS and kP to beat the plant's friction a quarter degree out: only a fresh start moves it
    Plant p = new Plant(30);
    p.slack = Plant.SLACK / 2;
    HoodController c = new HoodController();
    HoodController.Gains g = gains();
    g.kS = 0.02;
    g.kP = 0.05;
    for (int i = 0; i * DT < 0.6; i++) {
      p.step(c.update(DT, p.readPos, p.readVel, 30.25, g));
    }
    assertTrue("stuck at " + p.pos, Math.abs(p.pos - 30.25) <= 0.1);
    assertTrue(c.isSettled());
  }

  @Test public void insideTheDeadbandItStaysUnpowered() {
    Plant p = new Plant(30);
    HoodController c = new HoodController();
    for (int i = 0; i < 100; i++) {
      assertEquals(0, c.update(DT, p.readPos, 0, 30.08, gains()), 0);
    }
  }

  @Test public void followsATargetMovingEveryLoop() {
    Plant p = new Plant(20);
    HoodController c = new HoodController();
    double worst = 0;
    for (int i = 0; i * DT < 1.0; i++) {
      double goal = 20 + Math.min(30 * i * DT, 15); // 30 deg/s for half a second, then holds
      p.step(c.update(DT, p.readPos, p.readVel, goal, gains()));
      if (i * DT > 0.15) { worst = Math.max(worst, Math.abs(p.pos - goal)); }
    }
    assertTrue("worst lag " + worst, worst < 0.6);
    assertTrue("final " + (p.pos - 35), Math.abs(p.pos - 35) <= 0.2);
  }

  @Test public void profileKeepsToItsLimits() {
    HoodController c = new HoodController();
    HoodController.Gains g = gains();
    g.kP = 0; g.kD = 0; g.kS = 0; g.kV = 0; g.kA = 0; g.kG = 0;
    g.takeUpMaxS = 0; // Profile from the first step
    c.update(DT, 0, 0, 40, g);
    c.update(DT, 0, 0, 40, g);
    double lastVel = c.setpointVelDegPerSec();
    for (int i = 0; i < 400; i++) {
      c.update(DT, c.setpointDeg(), c.setpointVelDegPerSec(), 40, g);
      double v = c.setpointVelDegPerSec();
      assertTrue(Math.abs(v) <= g.maxVelDegPerSec + 1e-9);
      // Braking may run a little past the set rate, enough to stop exactly on the goal
      assertTrue("accel step " + (v - lastVel), Math.abs(v - lastVel) <= 1.25 * g.accelDegPerSec2 * DT
          || v == 0); // Landing on the goal ends at zero speed
      lastVel = v;
    }
    assertEquals(40, c.setpointDeg(), 1e-9);
  }

  /**
   * Follows a target ramping at rateDegPerSec from 15 degrees, set every 80 ms (TeleOp's loop) and carried
   * forward between as HoodLoop does. Returns {max error, RMS error, power reversals}, all after the first 0.3 s.
   */
  private static double[] track(double rateDegPerSec, boolean withRate) {
    Plant p = new Plant(15);
    HoodController c = new HoodController();
    HoodController.Gains g = gains();
    g.trackMinRateDegPerSec = 1.0;
    double setDeg = 15;
    double setS = 0;
    double maxErr = 0;
    double sumSq = 0;
    int n = 0;
    int reversals = 0;
    double lastSign = 0;
    for (int i = 0; i * DT < 2.0; i++) {
      double t = i * DT;
      if (i % 16 == 0) {
        setDeg = 15 + rateDegPerSec * t;
        setS = t;
      }
      double rate = withRate ? rateDegPerSec : 0;
      double goal = setDeg + rate * Math.min(0.15, t - setS);
      double power = c.update(DT, p.readPos, p.readVel, goal, rate, g);
      p.step(power);
      if (power != 0) {
        if (t >= 0.3 && lastSign != 0 && Math.signum(power) != lastSign) {
          reversals++;
        }
        lastSign = Math.signum(power);
      }
      if (t >= 0.3) {
        double err = Math.abs(p.pos - (15 + rateDegPerSec * t));
        maxErr = Math.max(maxErr, err);
        sumSq += err * err;
        n++;
      }
    }
    return new double[] {maxErr, Math.sqrt(sumSq / n), reversals};
  }

  @Test public void followsAMovingTargetWithItsRate() {
    for (double rate : new double[] {4, 10, -10, 20}) {
      double[] r = track(rate, true);
      double[] steps = track(rate, false);
      String msg = String.format("rate %.0f: with rate max %.2f rms %.2f reversals %.0f; steps max %.2f rms %.2f reversals %.0f",
          rate, r[0], r[1], r[2], steps[0], steps[1], steps[2]);
      System.out.println(msg);
      assertTrue(msg, r[0] < 0.5);
      assertTrue(msg, r[1] < steps[1]);
      assertEquals(msg, 0, r[2], 0); // Driven one way all along: no hunting
    }
  }
}
