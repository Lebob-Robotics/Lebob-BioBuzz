package org.firstinspires.ftc.teamcode.robot;

import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.field.PoseHandoff;
import org.firstinspires.ftc.teamcode.vision.VisionCorrector;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BiConsumer;
import java.util.function.Consumer;

/**
 * Runs the whole Robot loop on simulated hardware ({@link RobotSim}) the ways Lebot Teleop, the tuning tools and the
 * Scoring Auto drive it, and checks every loop against {@code src/test/resources/robot/loop-trace.txt}: one hash a
 * loop of every logged value, hardware write, telemetry line and rumble. A change to what the Robot does is meant to
 * fail this: set WRITE_ROBOT_TRACE=1 to rewrite the file. Each scenario's full text is written to
 * {@code build/robot-trace/}, so two versions can be compared line by line.
 */
public class RobotLoopTraceTest {
  /** Timed with System.nanoTime, so different every run. */
  private static final String[] UNREPEATABLE = {"RealOutputs/Loop/", "RealOutputs/Vision/Left/LastFrameAgeS",
      "RealOutputs/Vision/Right/LastFrameAgeS"};
  private static final BiConsumer<Gamepad, Integer> NOTHING = (g, i) -> {};

  /** One run of a scenario: what each loop recorded. */
  static final class Run {
    final String name;
    RobotSim sim;
    /** The OpMode whose hooks the loops call, if any. */
    Object opMode;
    boolean powerTelemetry;
    /** Each INIT loop, running loop and STOP: the Robot's own calls, unless an OpMode's hooks replace them. */
    Runnable initCode = () -> sim.robot.initLoop();
    Runnable loopCode = () -> {
      if (powerTelemetry) {
        sim.robot.addPowerTelemetry();
      }
      sim.robot.periodic();
    };
    Runnable stopCode = () -> sim.robot.stop();
    final StringBuilder text = new StringBuilder();
    final List<String> hashes = new ArrayList<>();
    final Map<String, Integer> counts = new HashMap<>();
    private Map<String, String> last = new HashMap<>();
    private String phase = "construct";
    private int loop;

    Run(String name, boolean simBuildsRobot) {
      this.name = name;
      sim = new RobotSim(simBuildsRobot);
    }

    Robot robot() {
      return sim.robot;
    }

    void init(String phase, int loops, BiConsumer<Gamepad, Integer> buttons) {
      this.phase = phase;
      for (int i = 0; i < loops; i++) {
        sim.release();
        buttons.accept(sim.gamepad, i);
        sim.frame(initCode);
        record();
      }
    }

    void run(String phase, int loops, BiConsumer<Gamepad, Integer> buttons) {
      this.phase = phase;
      for (int i = 0; i < loops; i++) {
        sim.release();
        buttons.accept(sim.gamepad, i);
        sim.frame(loopCode);
        record();
      }
    }

    void stop() {
      phase = "stop";
      sim.events.clear();
      stopCode.run();
      record();
    }

    /** The next OpMode, on a new Robot. */
    void next() {
      double[] truth = sim.truth;
      sim = new RobotSim();
      sim.truth = truth;
      powerTelemetry = false;
    }

    /** A file the run wrote, such as a tuning CSV, checked whole. */
    void recordFile(String label, String content) {
      text.append("== file ").append(label).append('\n').append(content);
      hashes.add(name + " file " + label + " " + sha(content));
    }

    private void record() {
      Map<String, String> values = sim.values();
      StringBuilder full = new StringBuilder();
      text.append("== ").append(loop).append(' ').append(phase).append('\n');
      for (Map.Entry<String, String> e : values.entrySet()) {
        String key = e.getKey();
        if (unrepeatable(key)) {
          continue;
        }
        full.append(key).append('=').append(e.getValue()).append('\n');
        if (!e.getValue().equals(last.get(key))) {
          text.append("  ").append(key).append(" = ").append(e.getValue()).append('\n');
        }
      }
      for (String event : sim.events) {
        full.append("> ").append(event).append('\n');
        text.append("  > ").append(event).append('\n');
      }
      hashes.add(name + " " + loop + " " + phase + " " + sha(full.toString()));
      count(values, "SOTM/State", "READY");
      count(values, "Feeder/BallLeft", "true");
      count(values, "Volley/Shot", "true");
      count(values, "SOTM/Backward", "true");
      count(values, "Intake/StackPacked", "true");
      count(values, "Robot/PositionHold", "true");
      count(values, "Drive/FeedCommit", "true");
      count(values, "Vision/Correction/Accepted", "true");
      last = values;
      loop++;
    }

    private void count(Map<String, String> values, String key, String value) {
      for (Map.Entry<String, String> e : values.entrySet()) {
        if (e.getKey().endsWith("RealOutputs/" + key) && e.getValue().equals(value)) {
          counts.merge(key + "=" + value, 1, Integer::sum);
          return;
        }
      }
    }
  }

  @Test
  public void everyLoopMatchesTheRecordedTrace() throws IOException {
    Map<String, Consumer<Run>> scenarios = new LinkedHashMap<>();
    scenarios.put("teleop", RobotLoopTraceTest::teleop);
    scenarios.put("teleop-unplaced", RobotLoopTraceTest::teleopUnplaced);
    scenarios.put("shot-tuning", RobotLoopTraceTest::shotTuning);
    scenarios.put("volley-tuning", RobotLoopTraceTest::volleyTuning);
    scenarios.put("auto-then-teleop", RobotLoopTraceTest::autoThenTeleop);
    scenarios.put("shoot-auto", RobotLoopTraceTest::shootAuto);
    scenarios.put("hold-correcting", RobotLoopTraceTest::holdCorrecting);
    checkTrace("robot/loop-trace.txt", true, scenarios, "SOTM/State=READY", "Feeder/BallLeft=true",
        "Volley/Shot=true", "SOTM/Backward=true", "Robot/PositionHold=true", "Vision/Correction/Accepted=true");
  }

  /**
   * Runs each scenario twice, which must come out the same, and checks every loop against the golden file under
   * src/test/resources, or rewrites it with WRITE_ROBOT_TRACE=1. Without {@code simBuildsRobot} each scenario's
   * OpMode builds the Robot. Each of {@code mustReach} (key=value) has to be logged in some loop.
   */
  static void checkTrace(String golden, boolean simBuildsRobot, Map<String, Consumer<Run>> scenarios,
      String... mustReach) throws IOException {
    File out = new File(moduleDir(), "build/robot-trace");
    out.mkdirs();
    List<String> now = new ArrayList<>();
    Map<String, Integer> coverage = new HashMap<>();
    for (Map.Entry<String, Consumer<Run>> scenario : scenarios.entrySet()) {
      Run first = run(scenario.getKey(), simBuildsRobot, scenario.getValue());
      Run second = run(scenario.getKey(), simBuildsRobot, scenario.getValue());
      File text = new File(out, scenario.getKey() + ".txt");
      Files.write(text.toPath(), first.text.toString().getBytes(StandardCharsets.UTF_8));
      if (!first.hashes.equals(second.hashes)) {
        File again = new File(out, scenario.getKey() + ".again.txt");
        Files.write(again.toPath(), second.text.toString().getBytes(StandardCharsets.UTF_8));
        fail(scenario.getKey() + " ran differently twice, first at " + firstDifference(first.text, second.text)
            + "\n(" + text + " vs " + again + ")");
      }
      now.addAll(first.hashes);
      first.counts.forEach((k, v) -> coverage.merge(k, v, Integer::sum));
      System.out.println(scenario.getKey() + ": " + first.hashes.size() + " loops, " + first.counts);
    }
    for (String reached : mustReach) {
      assertTrue("the simulation never reached " + reached + ": " + coverage, coverage.getOrDefault(reached, 0) > 0);
    }

    File file = new File(moduleDir(), "src/test/resources/" + golden);
    if ("1".equals(System.getenv("WRITE_ROBOT_TRACE"))) {
      file.getParentFile().mkdirs();
      Files.write(file.toPath(), now, StandardCharsets.UTF_8);
      return;
    }
    List<String> expected = Files.readAllLines(file.toPath(), StandardCharsets.UTF_8);
    List<String> problems = new ArrayList<>();
    int changed = 0;
    for (int i = 0; i < Math.max(expected.size(), now.size()); i++) {
      String want = i < expected.size() ? expected.get(i) : "(none)";
      String got = i < now.size() ? now.get(i) : "(none)";
      if (!want.equals(got)) {
        changed++;
        if (problems.size() < 10) {
          problems.add("  was " + want + "\n  now " + got);
        }
      }
    }
    if (changed > 0) {
      fail(changed + " of " + now.size() + " loops changed (full text in " + out + "), the first:\n"
          + String.join("\n", problems));
    }
  }

  private static Run run(String name, boolean simBuildsRobot, Consumer<Run> script) {
    RobotSim.startLogger();
    try {
      Run run = new Run(name, simBuildsRobot);
      script.accept(run);
      return run;
    } finally {
      RobotSim.stopLogger();
      PoseHandoff.take(System.nanoTime()); // A stopped Robot leaves its pose for the next OpMode
    }
  }

  /** A pose this far from our audience Cell's aim point, toward the field centre, turned partly away from it. */
  static double[] nearCell(Alliance alliance, double distanceM) {
    double[] cell = VisionCorrector.upCellAimPoint(alliance, Cell.AUDIENCE);
    double away = Math.hypot(cell[0], cell[1]);
    double x = cell[0] - distanceM * cell[0] / away;
    double y = cell[1] - distanceM * cell[1] / away;
    return new double[] {x, y, Math.atan2(-cell[1], -cell[0]) + 0.6};
  }

  /** Lebot Teleop, placed where the auto left it. */
  private static void teleop(Run r) {
    Robot robot = r.robot();
    r.powerTelemetry = true;
    robot.init(true);
    robot.setAlliance(Alliance.BLUE);
    robot.setVolleyMode(true);
    r.init("init-pick-red", 15, (g, i) -> g.b = i >= 3 && i < 6);
    double[] start = nearCell(Alliance.RED, 1.9);
    robot.startAt(start[0], start[1], start[2]);
    r.sim.truth = new double[] {start[0] + 0.08, start[1] - 0.05, start[2] + Math.toRadians(2)};
    r.run("idle", 25, NOTHING);
    r.run("drive", 50, (g, i) -> {
      g.left_stick_y = -0.6f;
      g.left_stick_x = 0.3f;
      g.right_stick_x = i < 20 ? 0.4f : 0f;
    });
    r.run("coast", 30, NOTHING);
    r.run("load", 80, (g, i) -> {
      g.left_trigger = 1f;
      r.sim.intakeAmps = i >= 30 && i < 60 ? 9.0 : 3.0;
    });
    r.sim.intakeAmps = 2.0;
    r.run("load-tap", 40, (g, i) -> g.left_trigger = i >= 15 && i < 20 ? 1f : 0f);
    r.run("rt-standing", 150, (g, i) -> g.right_trigger = 1f);
    r.run("rt-moving", 120, (g, i) -> {
      g.right_trigger = 1f;
      g.left_stick_x = 0.5f;
    });
    r.run("release", 20, NOTHING);
    r.run("drive-in", 30, (g, i) -> g.left_stick_y = -0.8f);
    r.run("rb-volley", 150, (g, i) -> g.right_bumper = true);
    r.run("rt-trims", 60, (g, i) -> {
      g.right_trigger = 1f;
      g.dpad_up = i % 15 < 2;
      g.dpad_down = i % 15 == 7;
      g.dpad_right = i % 20 == 10;
      g.dpad_left = i % 20 == 15;
    });
    r.run("y-full", 40, (g, i) -> g.y = true);
    r.run("b-eject", 20, (g, i) -> g.b = true);
    r.run("x-stop", 25, (g, i) -> g.x = i < 3);
    r.run("a-reset", 20, (g, i) -> g.a = i < 3);
    r.run("lb", 10, (g, i) -> g.left_bumper = i < 2);
    r.run("ramp", 20, (g, i) -> g.dpad_up = i < 2 || (i >= 10 && i < 12));
    r.run("presets", 20, (g, i) -> {
      g.dpad_down = i < 2;
      g.dpad_right = i == 10;
    });
    r.run("rt-backing-away", 100, (g, i) -> {
      g.right_trigger = 1f;
      g.left_stick_y = 0.9f;
      g.right_stick_x = i > 60 ? 0.5f : 0f;
    });
    r.sim.redUp = Cell.SCORING;
    r.run("up-cell-flips", 40, NOTHING);
    r.run("drive-far", 60, (g, i) -> {
      g.left_stick_y = -1f;
      g.left_stick_x = -0.5f;
    });
    r.run("rt-far", 80, (g, i) -> g.right_trigger = 1f);
    r.stop();
  }

  /** RB's position hold while the tags still move the pose to where the robot really is: the held spot moves too. */
  private static void holdCorrecting(Run r) {
    Robot robot = r.robot();
    robot.init(true);
    robot.setAlliance(Alliance.BLUE);
    robot.setVolleyMode(true);
    r.init("init", 5, NOTHING);
    double[] start = nearCell(Alliance.BLUE, 2.0);
    robot.startAt(start[0], start[1], start[2]);
    r.sim.truth = new double[] {start[0] + 0.12, start[1] - 0.08, start[2] + Math.toRadians(2.5)};
    r.run("rb-hold", 120, (g, i) -> g.right_bumper = true);
    r.run("release", 10, NOTHING);
    r.stop();
  }
  /** Shoot Auto: placed square by the driver, seeded, then volleys from a held spot and parks. */
  private static void shootAuto(Run r) {
    Robot robot = r.robot();
    robot.init();
    robot.setVolleyMode(true);
    robot.setHandsOffPose(true);
    r.init("init", 10, NOTHING);
    robot.start();
    double f = robot.driverForwardHeadingRad();
    double[] start = nearCell(Alliance.BLUE, 2.4);
    robot.seedPose(start[0], start[1], f + Math.PI);
    r.sim.truth = new double[] {start[0] + 0.05, start[1] - 0.04, f + Math.PI + Math.toRadians(1.5)};
    robot.setSticksLocked(true);
    robot.setDriveBrake(true);
    robot.setDriveOverride(new double[] {0.4 * Math.cos(f), 0.4 * Math.sin(f)});
    r.run("drive-in", 40, NOTHING);
    robot.setDriveOverride(null);
    robot.setHoldPoint(new double[] {robot.getPose().xM, robot.getPose().yM});
    r.run("volleys", 220, (g, i) -> {
      boolean hiveKnown = robot.getLastAim() != null;
      robot.setScriptedVolley(hiveKnown);
      robot.setHoldFire(!robot.isPoseValid() && i < 25);
      robot.setTurnOverride(hiveKnown ? 0 : 0.3);
    });
    robot.setScriptedVolley(false);
    robot.setHoldFire(false);
    robot.setTurnOverride(0);
    robot.setHoldPoint(null);
    robot.setDriveOverride(new double[] {-0.3 * Math.cos(f), -0.3 * Math.sin(f)});
    r.run("park", 40, NOTHING);
    robot.setDriveOverride(null);
    r.run("parked", 10, NOTHING);
    r.stop();
  }

  /** Lebot Teleop before the tags have placed the robot: blind shots and the far-side driver, then the first fix. */
  private static void teleopUnplaced(Run r) {
    Robot robot = r.robot();
    r.powerTelemetry = true;
    robot.init(false);
    robot.setVolleyMode(true);
    r.init("init", 20, NOTHING);
    robot.start();
    r.run("idle", 20, NOTHING);
    r.run("rt-blind", 80, (g, i) -> g.right_trigger = 1f);
    r.run("y-full", 30, (g, i) -> g.y = true);
    r.run("rb-blind", 40, (g, i) -> g.right_bumper = true);
    r.run("lt-and-rt", 40, (g, i) -> {
      g.left_trigger = 1f;
      g.right_trigger = 1f;
    });
    robot.setDriverOnFarSide(true);
    r.run("far-side-drive", 30, (g, i) -> {
      g.left_stick_y = -0.5f;
      g.right_stick_x = -0.3f;
    });
    r.run("a-reset", 10, (g, i) -> g.a = i < 2);
    robot.setDriverOnFarSide(false);
    r.sim.truth = nearCell(Alliance.BLUE, 2.2);
    r.run("tags-in-view", 60, NOTHING);
    r.run("rt-placed", 100, (g, i) -> g.right_trigger = 1f);
    robot.setSpinOnlyOnFire(true);
    r.run("lt-spin-only", 40, (g, i) -> g.left_trigger = 1f);
    r.run("rt-spin-only", 60, (g, i) -> g.right_trigger = 1f);
    r.stop();
  }

  /** Shot Tuning: fixed shots, one ball per press, then its moving passes. */
  private static void shotTuning(Run r) {
    Robot robot = r.robot();
    robot.init();
    robot.setTuningMode(true);
    robot.setTuningShot(false, 3400, 50);
    r.init("init", 5, NOTHING);
    double[] start = nearCell(Alliance.BLUE, 1.6);
    robot.startAt(start[0], start[1], start[2]);
    r.sim.truth = new double[] {start[0] - 0.04, start[1] + 0.06, start[2] - Math.toRadians(1)};
    robot.setRpmTrim(40);
    robot.setHeadingTrimDeg(1.5);
    robot.setHoodAngleTrimDeg(1.0);
    r.run("rb-taps", 135, (g, i) -> g.right_bumper = i % 45 < 5);
    robot.setScriptedOneBall(true);
    r.run("scripted-one-ball", 40, NOTHING);
    robot.setScriptedOneBall(false);
    r.run("rest", 20, NOTHING);
    robot.setTuningShot(true, 3000, 45);
    r.run("rt-backward", 80, (g, i) -> g.right_trigger = 1f);
    robot.setTuningLive(true);
    robot.setDriveOverride(new double[] {0.3, 0});
    robot.setSteadyStickReference(new double[] {0.3, 0});
    r.run("rt-live", 80, (g, i) -> g.right_trigger = 1f);
    robot.setSteadyGateBypassed(true);
    robot.setCloseMovingGateBypassed(true);
    r.run("rt-bypassed", 40, (g, i) -> g.right_trigger = 1f);
    robot.setDriveOverride(null);
    robot.setSteadyStickReference(null);
    robot.setTuningLive(false);
    robot.setHoldFire(true);
    r.run("rt-held-fire", 40, (g, i) -> g.right_trigger = 1f);
    robot.setHoldFire(false);
    robot.setSpinOnlyOnFire(true);
    r.run("lt", 30, (g, i) -> g.left_trigger = 1f);
    r.run("lt-and-rt", 40, (g, i) -> {
      g.left_trigger = 1f;
      g.right_trigger = 1f;
    });
    r.stop();
  }

  /** Volley Tuning: the aim forced, its own volley settings, held spots and scripted drives. */
  private static void volleyTuning(Run r) {
    Robot robot = r.robot();
    robot.init();
    robot.setVolleyMode(true);
    robot.setButtonsReserved(true);
    robot.setHoodedOnly(true);
    robot.setVolleyTrimsDeg(1.0, -0.5);
    robot.setVolleyFeedPower(0.8);
    robot.setVolleyIndexerPower(0.6);
    robot.setVolleyBoostRpm(120);
    robot.setVolleyFastRecovery(false);
    robot.setVolleyHeadingTrimDeg(0.7);
    r.init("init", 5, NOTHING);
    double[] start = nearCell(Alliance.BLUE, 1.5);
    robot.startAt(start[0], start[1], start[2]);
    r.sim.truth = new double[] {start[0] + 0.03, start[1] + 0.03, start[2]};
    robot.setHoldPoint(new double[] {start[0], start[1]});
    robot.setForceAim(true);
    r.run("force-aim", 60, NOTHING);
    r.run("marks", 30, (g, i) -> {
      g.a = i % 6 == 0;
      g.b = i % 6 == 1;
      g.x = i % 6 == 2;
      g.y = i % 6 == 3;
      g.dpad_up = i % 6 == 4;
      g.dpad_left = i % 6 == 5;
    });
    r.run("rb-volley", 140, (g, i) -> g.right_bumper = true);
    robot.setForceAim(false);
    robot.setHoodedOnly(false);
    robot.setBackwardOnly(true);
    robot.setVolleyTrimsDeg(2.0, 1.0, true);
    robot.setVolleyFastRecovery(true);
    robot.setVolleyFeedPower(Double.NaN);
    robot.setVolleyIndexerPower(Double.NaN);
    robot.setVolleyBoostRpm(Double.NaN);
    r.run("rb-backward", 140, (g, i) -> g.right_bumper = true);
    robot.setBackwardOnly(false);
    robot.setVolleyFastRecovery(null);
    robot.setSticksLocked(true);
    robot.setTurnOverride(0.3);
    r.run("sticks-locked", 40, (g, i) -> g.left_stick_y = -1f);
    robot.setTurnOverride(0);
    robot.setDriveOverride(new double[] {0.0, 0.4});
    r.run("scripted-drive", 40, NOTHING);
    robot.setDriveOverride(null);
    r.run("hold-return", 40, NOTHING);
    robot.setHoldPoint(null);
    robot.setSticksLocked(false);
    robot.setAimPointOverride(new double[] {0.5, 0.5});
    robot.setForceAim(true);
    r.run("aim-point", 40, NOTHING);
    robot.setAimPointOverride(null);
    robot.setForceAim(false);
    robot.setAllianceFromTags(true);
    r.run("alliance-from-tags", 10, NOTHING);
    robot.setDriveBrake(true);
    r.run("brake", 10, NOTHING);
    r.stop();
  }

  /** The Scoring Auto's preload volley, pickup and shots, then Lebot Teleop starting from its pose. */
  private static void autoThenTeleop(Run r) {
    PoseHandoff.take(System.nanoTime()); // Nothing left by another test
    Robot robot = r.robot();
    robot.setAutonomous(true);
    robot.setHandsOffPose(true);
    robot.setVolleyMode(true);
    robot.setAutoIndexerReverse(true);
    robot.setAlliance(Alliance.RED);
    robot.init();
    r.init("init", 10, (g, i) -> g.x = true); // No gamepad in the auto: stays RED
    double[] start = nearCell(Alliance.RED, 1.7);
    robot.startAt(start[0], start[1], start[2]);
    r.sim.truth = new double[] {start[0] + 0.06, start[1] + 0.02, start[2] - Math.toRadians(1.5)};
    double[][] powers = {{0, 0, 0}};
    robot.setAutoDriver((pose, velocity) -> powers[0]);
    robot.setDriveBrake(true);
    robot.setScriptedVolley(true);
    robot.setAutoShoot(true);
    r.run("preload-volley", 120, NOTHING);
    robot.setScriptedVolley(false);
    robot.setAutoShoot(false);
    robot.setAutoIntake(true);
    powers[0] = new double[] {0.5, 0.1, 0.0};
    r.run("drive-intake", 60, (g, i) -> robot.setAutoIndexerPaused(i % 20 < 10));
    robot.setAutoHeading(start[2] + 1.0, 0.5);
    powers[0] = new double[] {0.2, 0, 0};
    r.run("wiggle", 40, NOTHING);
    robot.setAutoHeading(Double.NaN);
    powers[0] = null;
    r.run("hold-still", 20, NOTHING);
    robot.setAutoIntake(false);
    robot.setAutoIndexerPaused(false);
    powers[0] = new double[] {0, 0, 0};
    robot.setAutoShoot(true);
    r.run("shoot", 80, NOTHING);
    robot.setAutoShoot(false);
    robot.setAutoDriver(null);
    r.run("done", 10, NOTHING);
    r.stop();

    r.next();
    Robot teleop = r.robot();
    r.powerTelemetry = true;
    teleop.init(true);
    teleop.setVolleyMode(true);
    r.init("teleop-init", 5, NOTHING);
    teleop.startFromAuto();
    r.run("teleop-idle", 20, NOTHING);
    r.run("teleop-rt", 80, (g, i) -> g.right_trigger = 1f);
    r.stop();
  }

  private static boolean unrepeatable(String key) {
    for (String prefix : UNREPEATABLE) {
      if (key.contains(prefix)) {
        return true;
      }
    }
    return false;
  }

  private static String firstDifference(CharSequence a, CharSequence b) {
    String[] x = a.toString().split("\n");
    String[] y = b.toString().split("\n");
    String header = "";
    for (int i = 0; i < Math.min(x.length, y.length); i++) {
      if (x[i].startsWith("==")) {
        header = x[i];
      }
      if (!x[i].equals(y[i])) {
        return header + "\n  " + x[i] + "\n  " + y[i];
      }
    }
    return "the end (" + x.length + " vs " + y.length + " lines)";
  }

  static String sha(String text) {
    try {
      byte[] digest = MessageDigest.getInstance("SHA-256").digest(text.getBytes(StandardCharsets.UTF_8));
      StringBuilder hex = new StringBuilder();
      for (int i = 0; i < 8; i++) {
        hex.append(String.format("%02x", digest[i]));
      }
      return hex.toString();
    } catch (NoSuchAlgorithmException e) {
      throw new IllegalStateException(e);
    }
  }

  static File moduleDir() {
    return new File("src/test").isDirectory() ? new File(".") : new File("TeamCode");
  }
}
