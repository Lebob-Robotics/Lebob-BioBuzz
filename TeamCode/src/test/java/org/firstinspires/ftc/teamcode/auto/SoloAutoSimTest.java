package org.firstinspires.ftc.teamcode.auto;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.follower.Follower;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

import org.firstinspires.ftc.teamcode.auto.SoloSequence.Step;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Angles;
import org.firstinspires.ftc.teamcode.field.FieldPose;
import org.firstinspires.ftc.teamcode.field.Units;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.junit.Test;

/**
 * The whole solo auto on PedroFollowSimTest's simulated robot: SoloAuto's own drives and arrivals with SoloSequence,
 * the heading lock, the walls, and modelled volleys and pickups. Each branch must stay out of the Hive and our wall's
 * Flower outside its pickup, shoot every ball, and be parked before the end.
 */
public class SoloAutoSimTest {
  private static final double DT = 0.02;
  private static final double MAX_FORWARD_MPS = 55 * Units.M_PER_IN;
  private static final double MAX_STRAFE_MPS = 45 * Units.M_PER_IN;
  private static final double MAX_TURN_RAD_S = 5.0;
  private static final double LAG_S = 0.12;
  private static final double LOCK_GAIN = 2.5;
  /** Stopped at a spot to the first ball, then between balls (MIN_BALL_GAP_S): from the regionals logs. */
  private static final double FIRST_BALL_S = 0.8;
  private static final double BALL_GAP_S = 0.35;
  /** In a Flower to its 4 balls in the stack. */
  private static final double FLOWER_FILL_S = 0.6;
  /** When the tags show the Hive after the check starts, if they do. */
  private static final double TAGS_AFTER_S = 0.4;

  private static final class Probe extends SoloAuto {
    FieldPose at;
    double locked = Double.NaN;

    Probe(Alliance alliance) {
      super(alliance);
      follower = new Follower(localizer, drivetrain, Constants.algorithm());
      plan = new SoloSequence(0);
    }

    @Override
    protected FieldPose pose() {
      return at;
    }

    @Override
    protected void lockHeading(double headingRad) {
      locked = headingRad;
    }
  }

  private static final class Run {
    final List<Step> steps = new ArrayList<>();
    final StringBuilder timeline = new StringBuilder();
    double parkedS = Double.NaN;
    double decidedS = Double.NaN;
    int shot;
    int picked;
    Boolean tipped;
  }

  private double x;
  private double y;
  private double h;
  private double vForward;
  private double vLeft;
  private double omega;

  /**
   * One auto from the start to 30 s.
   *
   * @param tags      the near Cell is down by the tags, null for no tags at all
   * @param predicted the ball count's prediction
   */
  private Run run(Alliance alliance, Boolean tags, boolean predicted) {
    Probe auto = new Probe(alliance);
    double[] start = AutoField.forAlliance(SoloSequence.START, alliance);
    x = start[0];
    y = start[1];
    h = start[2];
    vForward = vLeft = omega = 0;
    Run run = new Run();
    int held = 4;
    double stepStartS = 0;
    double shootingS = 0;
    int volleyBalls = 0;
    boolean filled = false;
    Step step = auto.plan.step();
    run.steps.add(step);
    sense(auto);
    auto.plan.takeNewStep(); // As BaseAuto.onPsiKitStart
    auto.startStep(step);
    for (double t = DT; t <= 30.0; t += DT) {
      sense(auto);
      double inStepS = t - stepStartS;
      Boolean verdict = step == Step.CHECK_TIP && inStepS >= TAGS_AFTER_S ? tags : null;
      auto.plan.tipCheck(verdict, predicted);

      double px = x;
      double py = y;
      auto.follower.update(DT);
      double[] p = auto.drivetrain.powers().clone();
      if (!Double.isNaN(auto.locked)) {
        p[2] = Math.max(-1, Math.min(1, LOCK_GAIN * Angles.normalize(auto.locked - h)));
      }
      move(p);
      double speed = Math.hypot(x - px, y - py) / DT;

      boolean ballLeft = false;
      if (auto.plan.shoot(t)) {
        shootingS += DT;
        if (held > 0 && shootingS >= FIRST_BALL_S + volleyBalls * BALL_GAP_S) {
          held--;
          volleyBalls++;
          run.shot++;
          ballLeft = true;
        }
      }
      boolean intaking = step == Step.INTAKE_WALL_FLOWER || step == Step.INTAKE_REAR_FLOWER;
      if (intaking && !filled && inStepS >= FLOWER_FILL_S) {
        filled = true;
        run.picked += 4 - held;
        held = 4;
      }
      boolean arrived = auto.stepArrived(step, auto.plan.isFacingStep(), inStepS, speed);
      auto.plan.update(t, arrived, ballLeft, held == 0, intaking && held >= 4);
      if (auto.plan.takeNewStep()) {
        if (step == Step.SWEEP_GARDEN) { // The Garden's 4 in the intake by the end of the sweep
          run.picked += 4 - held;
          held = 4;
        }
        if (step == Step.CHECK_TIP) {
          run.decidedS = t;
        }
        step = auto.plan.step();
        stepStartS = t;
        shootingS = 0;
        volleyBalls = 0;
        filled = false;
        run.steps.add(step);
        run.timeline.append(String.format(Locale.US, "%5.2f %s%n", t, step));
        if (step == Step.PARKED) {
          run.parkedS = t;
        }
        auto.startStep(step);
      }
      check(alliance, step, t);
    }
    run.tipped = auto.plan.tipped();
    return run;
  }

  private void sense(Probe auto) {
    double vx = vForward * Math.cos(h) - vLeft * Math.sin(h);
    double vy = vForward * Math.sin(h) + vLeft * Math.cos(h);
    auto.localizer.set(x, y, h, vx, vy, omega);
    auto.at = new FieldPose(x, y, h);
  }

  /** The powers move the robot, each speed lagging its command; the walls stop it. */
  private void move(double[] p) {
    double a = DT / LAG_S;
    vForward += (p[0] * MAX_FORWARD_MPS - vForward) * a;
    vLeft += (p[1] * MAX_STRAFE_MPS - vLeft) * a;
    omega += (p[2] * MAX_TURN_RAD_S - omega) * a;
    x += (vForward * Math.cos(h) - vLeft * Math.sin(h)) * DT;
    y += (vForward * Math.sin(h) + vLeft * Math.cos(h)) * DT;
    x = Math.max(-AutoField.AT_WALL_M, Math.min(AutoField.AT_WALL_M, x));
    y = Math.max(-AutoField.AT_WALL_M, Math.min(AutoField.AT_WALL_M, y));
    h = Angles.normalize(h + omega * DT);
  }

  /** Out of the Hive always, and out of our wall's Flower except taking it. */
  private void check(Alliance alliance, Step step, double t) {
    String where = String.format(Locale.US, "%s %s at %.2f s: %.2f, %.2f", alliance, step, t, x, y);
    assertFalse("in the Hive, " + where, Math.abs(x) < 0.75 && Math.abs(y) < 0.95);
    double[] red = AutoField.forAlliance(new double[] {x, y, h}, alliance);
    double[] keepOut = AutoField.WALL_FLOWER_KEEP_OUT;
    boolean takingIt = Arrays.asList(Step.LOWER_ARM_WALL, Step.INTAKE_WALL_FLOWER, Step.BACK_FROM_WALL_FLOWER,
        Step.RAISE_ARM_WALL).contains(step);
    boolean inKeepOut = red[0] > keepOut[0] && red[0] < keepOut[1] && red[1] < keepOut[2];
    assertFalse("in our wall's Flower, " + where, inKeepOut && !takingIt);
  }

  private static List<Step> shooting(List<Step> steps) {
    List<Step> s = new ArrayList<>();
    for (Step step : steps) {
      if (step.shooting) {
        s.add(step);
      }
    }
    return s;
  }

  private void assertFullRun(String name, Run run, List<Step> volleys) {
    String detail = name + "\n" + run.timeline;
    System.out.println(detail);
    assertEquals(detail, volleys, shooting(run.steps));
    assertTrue(detail + "parked at " + run.parkedS, run.parkedS <= 28.5);
    assertEquals(detail + "picked", 12, run.picked);
    assertEquals(detail + "shot", 16, run.shot);
  }

  @Test
  public void notTippedShootsEveryBallAndParks() {
    for (Alliance a : Alliance.values()) {
      Run run = run(a, false, false);
      assertEquals(Boolean.FALSE, run.tipped);
      assertFullRun(a + " not tipped", run,
          Arrays.asList(Step.SHOOT_PRELOAD, Step.SHOOT_NEAR, Step.SHOOT_PARK, Step.PARKED));
    }
  }

  @Test
  public void tippedShootsEveryBallAndParks() {
    for (Alliance a : Alliance.values()) {
      Run run = run(a, true, false);
      assertEquals(Boolean.TRUE, run.tipped);
      assertFullRun(a + " tipped", run,
          Arrays.asList(Step.SHOOT_PRELOAD, Step.SHOOT_PARK, Step.SHOOT_FAR, Step.SHOOT_NEAR, Step.PARKED));
    }
  }

  @Test
  public void withNoTagsThePredictionChoosesTheBranch() {
    for (Alliance a : Alliance.values()) {
      for (boolean predicted : new boolean[] {false, true}) {
        Run run = run(a, null, predicted);
        assertEquals(predicted, run.tipped);
        assertTrue(a + " parked at " + run.parkedS, run.parkedS <= 28.5);
      }
    }
  }
}
