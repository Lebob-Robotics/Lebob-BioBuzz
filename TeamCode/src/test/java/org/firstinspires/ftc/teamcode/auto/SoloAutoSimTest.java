package org.firstinspires.ftc.teamcode.auto;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.follower.Follower;

import java.io.File;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

import org.firstinspires.ftc.teamcode.auto.SoloSequence.Step;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Angles;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.field.FieldPose;
import org.firstinspires.ftc.teamcode.field.Units;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.shooting.volley.VolleySettings;
import org.firstinspires.ftc.teamcode.shooting.volley.VolleySettingsAccess;
import org.firstinspires.ftc.teamcode.shooting.volley.VolleySpotDrive;
import org.junit.Test;

/**
 * The whole solo auto on a simulated robot: SoloAuto's own drives and arrivals with SoloSequence, the heading lock,
 * the walls, and modelled volleys and pickups. Each branch must stay out of the Hive and our wall's Flower outside its
 * pickup, shoot every ball, and be parked before the end.
 *
 * <p>The drive is the real one's: Pedro's powers, scaled down together past full power on a wheel as
 * MecanumDriveSubsystem.mix does, so a diagonal or a turn on the way is slower than straight ahead. Replayed on the 11
 * drives of the regionals autos, from each one's start to where it stopped, it came to rest 0.05 s sooner than the
 * real robot on average, and within 0.35 s on 10 of them.
 *
 * <p>WRITE_SOLO_TRACE=1 writes the red runs on one timeline to tools/solo/solo-trace.js for the viewer there.
 */
public class SoloAutoSimTest {
  private static final double DT = 0.02;
  /**
   * Full power, each way. Strafe is Pedro's tuned 48 in/s. Forward is under its tuned 65: the regionals logs reached
   * 1.49 m/s (59 in/s) in 9 of 10 full-power samples, 1.67 at most; diagonally about 1.1, strafing 1.3.
   */
  private static final double MAX_FORWARD_MPS = 60 * Units.M_PER_IN;
  private static final double MAX_STRAFE_MPS = 48 * Units.M_PER_IN;
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
  private static final int TRACE_EVERY = 2;

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
    /** Every TRACE_EVERY loops: t, x, y, heading, balls shot, balls held, the step's ordinal. */
    final List<double[]> trace = new ArrayList<>();
    /** Each ball out: t, x, y, and the aim point it went to. */
    final List<double[]> balls = new ArrayList<>();
    final StringBuilder timeline = new StringBuilder();
    double parkedS = Double.NaN;
    double decidedS = Double.NaN;
    double recheckedS = Double.NaN;
    double worstAimDeg;
    int shot;
    int picked;
    Boolean tipped;
    Boolean farTipped;
  }

  /** Volley times (FIRST_BALL_S, BALL_GAP_S) multiplied by this. */
  private double volleySlowdown = 1;
  private double x;
  private double y;
  private double h;
  private double vForward;
  private double vLeft;
  private double omega;

  /** One auto, the far Cell tipping back as the recheck at our wall's Flower looks (the tags show it). */
  private Run run(Alliance alliance, Boolean tags, boolean predicted) {
    return run(alliance, tags, predicted, Boolean.TRUE);
  }

  /**
   * One auto from the start to 30 s.
   *
   * @param tags      the near Cell is down by the tags at the check, null for no tags at all
   * @param predicted the ball count's prediction: the near Cell is down
   * @param farTipped tipped, the far Cell has tipped back by the tags at the recheck, null for no tags then
   */
  private Run run(Alliance alliance, Boolean tags, boolean predicted, Boolean farTipped) {
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
    run.trace.add(new double[] {0, x, y, h, 0, held, step.ordinal()});
    int loops = 0;
    for (double t = DT; t <= 30.0; t += DT) {
      sense(auto);
      double inStepS = t - stepStartS;
      Boolean verdict = null;
      if (step == Step.CHECK_TIP) {
        verdict = inStepS >= TAGS_AFTER_S ? tags : null;
      } else if (auto.plan.watchingHive()) { // The recheck: the near Cell is down while the far one is still up
        verdict = farTipped == null ? null : !farTipped;
      }
      auto.plan.tipCheck(verdict, predicted);

      double px = x;
      double py = y;
      boolean intaking = step == Step.INTAKE_WALL_FLOWER || step == Step.INTAKE_REAR_FLOWER;
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
        if (held > 0 && shootingS >= (FIRST_BALL_S + volleyBalls * BALL_GAP_S) * volleySlowdown) {
          held--;
          volleyBalls++;
          run.shot++;
          ballLeft = true;
          double[] aim = AutoField.autoAimPoint(alliance, auto.aimCell());
          run.balls.add(new double[] {t, x, y, aim[0], aim[1]});
          double off = Angles.normalize(h + Math.PI - Math.atan2(aim[1] - y, aim[0] - x));
          run.worstAimDeg = Math.max(run.worstAimDeg, Math.toDegrees(Math.abs(off)));
        }
      }
      if (intaking && !filled && inStepS >= FLOWER_FILL_S) {
        filled = true;
        run.picked += 4 - held;
        held = 4;
      }
      boolean arrived = auto.stepArrived(step, auto.plan.isFacingStep(), inStepS, speed);
      // Never full: at regionals every Flower ran its dwell, 1.86 to 2.04 s in it (here about 2.15 s)
      auto.plan.update(t, arrived, ballLeft, held == 0, false);
      if (auto.plan.takeNewStep()) {
        if (step == Step.SWEEP_GARDEN) { // The Garden's 4 in the intake by the end of the sweep
          run.picked += 4 - held;
          held = 4;
        }
        if (step == Step.CHECK_TIP) {
          run.decidedS = t;
        }
        if (step == Step.BACK_FROM_WALL_FLOWER && auto.plan.farTipped() != null && Double.isNaN(run.recheckedS)) {
          run.recheckedS = t;
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
      if (++loops % TRACE_EVERY == 0) {
        run.trace.add(new double[] {t, x, y, h, run.shot, held, step.ordinal()});
      }
    }
    run.tipped = auto.plan.tipped();
    run.farTipped = auto.plan.farTipped();
    return run;
  }

  private void sense(Probe auto) {
    double vx = vForward * Math.cos(h) - vLeft * Math.sin(h);
    double vy = vForward * Math.sin(h) + vLeft * Math.cos(h);
    auto.localizer.set(x, y, h, vx, vy, omega);
    auto.at = new FieldPose(x, y, h);
  }

  /**
   * The powers move the robot, each speed lagging its command; the walls stop it. A wheel's power is forward,
   * strafe and turn added (MecanumDriveSubsystem.mix), so past 1 in all they are scaled down together.
   */
  private void move(double[] p) {
    double sum = Math.abs(p[0]) + Math.abs(p[1]) + Math.abs(p[2]);
    double k = sum > 1 ? 1 / sum : 1;
    double a = DT / LAG_S;
    vForward += (p[0] * k * MAX_FORWARD_MPS - vForward) * a;
    vLeft += (p[1] * k * MAX_STRAFE_MPS - vLeft) * a;
    omega += (p[2] * k * MAX_TURN_RAD_S - omega) * a;
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
    boolean takingIt = Arrays.asList(Step.LOWER_ARM_WALL, Step.INTAKE_WALL_FLOWER, Step.BACK_FROM_WALL_FLOWER)
        .contains(step);
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
    assertTrue(detail + "a ball left " + run.worstAimDeg + " deg off its Cell", run.worstAimDeg <= 3);
  }

  @Test
  public void notTippedShootsEveryBallAndParks() {
    for (Alliance a : Alliance.values()) {
      Run run = run(a, false, false);
      assertEquals(Boolean.FALSE, run.tipped);
      assertFullRun(a + " not tipped", run,
          Arrays.asList(Step.SHOOT_PRELOAD, Step.SHOOT_NEAR, Step.SHOOT_FAR, Step.SHOOT_FAR));
    }
  }

  @Test
  public void tippedShootsEveryBallAndParks() {
    for (Alliance a : Alliance.values()) {
      Run run = run(a, true, false);
      assertEquals(Boolean.TRUE, run.tipped);
      assertEquals(Boolean.TRUE, run.farTipped);
      assertFullRun(a + " tipped", run,
          Arrays.asList(Step.SHOOT_PRELOAD, Step.SHOOT_FAR, Step.SHOOT_FAR, Step.SHOOT_NEAR));
    }
  }

  @Test
  public void tippedWithTheFarCellStillUpShootsItAgainThenParks() {
    for (Alliance a : Alliance.values()) {
      Run run = run(a, true, false, false);
      assertEquals(Boolean.FALSE, run.farTipped);
      assertFullRun(a + " tipped, far Cell still up", run,
          Arrays.asList(Step.SHOOT_PRELOAD, Step.SHOOT_FAR, Step.SHOOT_FAR, Step.SHOOT_FAR));
    }
  }

  @Test
  public void withNoTagsAtTheWallFlowerThePredictionDecidesTheRecheck() {
    for (Alliance a : Alliance.values()) {
      for (boolean nearDown : new boolean[] {false, true}) {
        Run run = run(a, true, nearDown, null);
        assertEquals(a + " near Cell down " + nearDown, !nearDown, run.farTipped);
        assertTrue(a + " parked at " + run.parkedS, run.parkedS <= 28.5);
        assertEquals(16, run.shot);
      }
    }
  }

  @Test
  public void writesTheTraceForTheViewer() throws Exception {
    if (!"1".equals(System.getenv("WRITE_SOLO_TRACE"))) {
      return;
    }
    Alliance a = Alliance.RED;
    Run notTipped = run(a, false, false);
    Run tipped = run(a, true, false);
    Run farStillUp = run(a, true, false, false);
    StringBuilder j = new StringBuilder();
    j.append("// Written by SoloAutoSimTest with WRITE_SOLO_TRACE=1: the red runs of the solo auto, simulated.\n");
    j.append("var SOLO_TRACE = {\n");
    j.append("\"alliance\": \"").append(a).append("\",\n");
    j.append("\"splitS\": ").append(num(notTipped.decidedS)).append(",\n");
    j.append("\"steps\": [");
    for (Step s : Step.values()) {
      j.append(s.ordinal() == 0 ? "" : ", ").append('"').append(s).append('"');
    }
    j.append("],\n\"field\": {\n");
    j.append("\"half\": ").append(num(AutoField.FIELD_HALF_M)).append(", \"robotHalf\": ")
        .append(num(AutoField.ROBOT_HALF_M)).append(",\n");
    j.append("\"hive\": [0.75, 0.95],\n");
    j.append("\"cells\": {");
    for (Cell c : Cell.values()) {
      double[] p = AutoField.autoAimPoint(a, c);
      j.append(c.ordinal() == 0 ? "" : ", ").append('"').append(c).append("\": ").append(pair(p[0], p[1]));
    }
    j.append("},\n\"nearCell\": \"").append(SoloSequence.nearCell(a)).append("\",\n");
    j.append("\"garden\": [1.22, ").append(num(AutoField.FIELD_HALF_M)).append(", ")
        .append(num(-AutoField.FIELD_HALF_M)).append(", -1.22],\n");
    double[] k = AutoField.WALL_FLOWER_KEEP_OUT;
    j.append("\"wallFlowerKeepOut\": [").append(num(k[0])).append(", ").append(num(k[1])).append(", ")
        .append(num(k[2])).append("],\n");
    double[] wf = AutoField.WALL_FLOWER_INTAKE;
    double[] rf = AutoField.REAR_FLOWER_INTAKE;
    j.append("\"flowers\": [").append(pair(wf[0], wf[1] - AutoField.ROBOT_HALF_M)).append(", ")
        .append(pair(rf[0] - AutoField.ROBOT_HALF_M, rf[1])).append("],\n");
    double[] lz = AutoField.LOADING_ZONE;
    j.append("\"loadingZone\": [").append(num(lz[0])).append(", ").append(num(lz[1])).append(", ")
        .append(num(lz[2])).append(", ").append(num(lz[3])).append("],\n");
    // Each volley's spot by its Volley Tuning name and IN rate at the 15 Nov competition
    VolleySettings tuned = AutoVolleySettingsTest.comp();
    Cell near = SoloSequence.nearCell(a);
    Cell far = SoloSequence.parkCell(a);
    j.append("\"spots\": [").append(spot("start", AutoField.forAlliance(SoloSequence.START, a))).append(", ")
        .append(spot(tunedName(tuned, a, near, AutoField.shotSpot(a, near)), AutoField.shotSpot(a, near)))
        .append(", ").append(spot(tunedName(tuned, a, far, AutoField.shotSpot(a, far)), AutoField.shotSpot(a, far)))
        .append(", ").append(spot("park", SoloSequence.parkSpot(a)))
        .append("]\n},\n");
    j.append("\"runs\": [\n");
    j.append(runJson("Not tipped", notTipped, -1, notTipped.decidedS, "")).append(",\n")
        .append(runJson("Tipped", tipped, -1, tipped.decidedS, "")).append(",\n")
        .append(runJson("Far still up", farStillUp, 1, farStillUp.recheckedS, "the far Cell had not tipped back"))
        .append("\n]\n};\n");
    File module = new File("src/test").isDirectory() ? new File(".") : new File("TeamCode");
    File out = new File(module, "../tools/solo/solo-trace.js");
    Files.write(out.toPath(), j.toString().getBytes(StandardCharsets.UTF_8));
  }

  private static String tunedName(VolleySettings tuned, Alliance a, Cell cell, double[] p) {
    double[] aim = AutoField.autoAimPoint(a, cell);
    int spot = tuned.spotFor(new FieldPose(p[0], p[1], p[2]), aim[0], aim[1]);
    String name = VolleySpotDrive.name(spot);
    return String.format(Locale.US, "%s%s · %.0f%% IN", name.equals("PARK") ? "" : "spot ", name,
        100 * VolleySettingsAccess.rate(tuned, spot));
  }

  /** A run, the same as run parent (-1: the shared start) until branchS, where why sent it its own way. */
  private static String runJson(String name, Run run, int parent, double branchS, String why) {
    StringBuilder j = new StringBuilder("{\"name\": \"").append(name).append("\", \"parkedS\": ")
        .append(num(run.parkedS)).append(", \"shot\": ").append(run.shot).append(", \"parent\": ").append(parent)
        .append(", \"branchS\": ").append(num(branchS)).append(", \"why\": \"").append(why)
        .append("\",\n\"trace\": [");
    for (int i = 0; i < run.trace.size(); i++) {
      double[] r = run.trace.get(i);
      j.append(i == 0 ? "" : ",").append(i % 8 == 0 ? "\n" : "").append(String.format(Locale.US,
          "[%.2f,%.3f,%.3f,%.3f,%d,%d,%d]", r[0], r[1], r[2], r[3], (int) r[4], (int) r[5], (int) r[6]));
    }
    j.append("],\n\"balls\": [");
    for (int i = 0; i < run.balls.size(); i++) {
      double[] b = run.balls.get(i);
      j.append(i == 0 ? "" : ",").append(String.format(Locale.US, "[%.2f,%.3f,%.3f,%.3f,%.3f]", b[0], b[1], b[2],
          b[3], b[4]));
    }
    return j.append("]}").toString();
  }

  private static String spot(String name, double[] p) {
    return String.format(Locale.US, "{\"name\": \"%s\", \"x\": %.3f, \"y\": %.3f, \"h\": %.3f}", name, p[0], p[1],
        p[2]);
  }

  private static String pair(double a, double b) {
    return "[" + num(a) + ", " + num(b) + "]";
  }

  private static String num(double v) {
    return String.format(Locale.US, "%.3f", v);
  }

  @Test
  public void slowerVolleysStillParkInTime() {
    // Each slower run meets the park's deadline at a later step: in a volley, a Flower, backing out, on a drive
    for (Alliance a : Alliance.values()) {
      for (boolean tipped : new boolean[] {false, true}) {
        for (Boolean farTipped : new Boolean[] {true, false}) {
          for (int i = 0; i <= 30; i++) {
            volleySlowdown = 1 + i * 0.05;
            Run run = run(a, tipped, tipped, farTipped);
            String what = String.format(Locale.US, "%s tipped %b far %b volleys x%.2f%n%s", a, tipped, farTipped,
                volleySlowdown, run.timeline);
            assertTrue(what + "parked at " + run.parkedS, run.parkedS <= 28.5);
          }
        }
      }
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
