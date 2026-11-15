package org.firstinspires.ftc.teamcode.robot;

import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.opmodes.tuning.ShotTuning;
import org.firstinspires.ftc.teamcode.opmodes.tuning.SotmTuning;
import org.firstinspires.ftc.teamcode.opmodes.tuning.SpecialShotCalibration;
import org.firstinspires.ftc.teamcode.robot.RobotLoopTraceTest.Run;
import org.firstinspires.ftc.teamcode.tuning.TuningCsv;
import org.junit.Test;
import org.psilynx.psikit.ftc.PsiKitIterativeOpMode;

import java.io.File;
import java.io.IOException;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BiConsumer;
import java.util.function.Consumer;

/**
 * Runs Shot Tuning, SOTM Tuning and Special Shot Calibration through their INIT, START, loop and STOP hooks on
 * simulated hardware ({@link RobotSim}), and checks every loop and the CSVs they write against
 * {@code src/test/resources/robot/tuning-trace.txt}, as {@link RobotLoopTraceTest} does for the Robot. Set
 * WRITE_ROBOT_TRACE=1 to rewrite it; each run's full text and CSVs are under {@code build/robot-trace/}.
 */
public class TuningOpModeTraceTest {
  private static final String SESSION = "20261116_090000";
  private static final BiConsumer<Gamepad, Integer> NOTHING = (g, i) -> {};

  @Test
  public void everyLoopAndCsvMatchesTheRecordedTrace() throws IOException {
    Map<String, Consumer<Run>> scenarios = new LinkedHashMap<>();
    scenarios.put("shot-tuning-opmode", csv(TuningOpModeTraceTest::shotTuning));
    scenarios.put("sotm-tuning-opmode", csv(TuningOpModeTraceTest::sotmTuning));
    scenarios.put("special-shot-opmode", csv(TuningOpModeTraceTest::specialShot));
    RobotLoopTraceTest.checkTrace("robot/tuning-trace.txt", false, scenarios, "Feeder/BallLeft=true");
  }

  /** Shot Tuning: the search standing, marks, a test spot, a moving pass, then carrying the session on. */
  private static void shotTuning(Run r) {
    shotSession(r);
    r.run("b-next-spot", 300, (g, i) -> g.b = i < 2);
    r.run("rb-at-spot", 150, (g, i) -> g.right_bumper = true);
    r.run("mark-short", 12, (g, i) -> {
      g.x = i < 6;
      g.dpad_down = i == 2;
    });
    r.run("rb-at-spot-2", 150, (g, i) -> g.right_bumper = true);
    r.run("mark-long-right", 12, (g, i) -> {
      g.x = i < 6;
      g.dpad_up = i == 2;
      g.dpad_right = i == 3;
    });
    r.run("start-pass", 400, (g, i) -> g.start = i < 2);
    r.run("mark-pass-in", 10, (g, i) -> g.x = i < 2);
    r.run("b-drive", 40, (g, i) -> g.b = i < 2);
    r.run("b-stops", 20, (g, i) -> g.b = i < 2);
    r.run("knocked-off", 80, (g, i) -> {
      if (i == 0) {
        r.sim.truth[0] += 0.15;
        r.sim.pinpoint.x += 0.15;
      }
    });
    r.sim.balls = 0;
    r.run("rb-empty", 150, (g, i) -> g.right_bumper = true);
    r.sim.balls = 3;
    r.run("lb-backward", 20, (g, i) -> g.left_bumper = i < 2);
    r.run("y-backward", 80, (g, i) -> g.y = i < 2);
    r.run("rb-backward", 150, (g, i) -> g.right_bumper = true);
    r.run("mark-left", 12, (g, i) -> {
      g.x = i < 6;
      g.dpad_left = i == 2;
    });
    r.run("lb-front", 20, (g, i) -> g.left_bumper = i < 2);
    r.stop();

    nextOpMode(r, new ShotTuning());
    r.init("init-carry-on", 30, (g, i) -> g.y = (i >= 5 && i < 8) || (i >= 15 && i < 18));
    call(r, "onPsiKitStart");
    r.run("carried-on", 120, NOTHING);
    r.run("rb-carried-on", 150, (g, i) -> g.right_bumper = true);
    r.run("mark-carried-on", 10, (g, i) -> g.x = i < 2);
    r.stop();
  }

  /** A short Shot Tuning session standing near the Cell: turn to it, shoot and mark. */
  private static void shotSession(Run r) {
    r.sim.truth = RobotLoopTraceTest.nearCell(Alliance.BLUE, 2.0);
    open(r, new ShotTuning());
    r.init("init", 15, NOTHING);
    call(r, "onPsiKitStart");
    r.run("idle", 40, NOTHING);
    r.run("y-orient", 80, (g, i) -> g.y = i < 2);
    r.run("rb-fire", 150, (g, i) -> g.right_bumper = true);
    r.run("mark-in", 10, (g, i) -> g.x = i < 2);
    r.run("rb-fire-2", 150, (g, i) -> g.right_bumper = true);
    r.run("mark-long", 12, (g, i) -> {
      g.x = i < 6;
      g.dpad_up = i == 2;
    });
    r.run("rb-fire-3", 150, (g, i) -> g.right_bumper = true);
    r.run("mark-bounced", 10, (g, i) -> g.a = i < 2);
    r.run("rb-fire-4", 150, (g, i) -> g.right_bumper = true);
    r.run("mark-left", 12, (g, i) -> {
      g.x = i < 6;
      g.dpad_left = i == 2;
    });
  }

  /** SOTM Tuning after a short Shot Tuning session (its standing aim trims): passes, marks, skips, then again. */
  private static void sotmTuning(Run r) {
    shotSession(r);
    r.stop();

    nextOpMode(r, new SotmTuning());
    r.sim.balls = 3;
    r.init("init", 15, NOTHING);
    call(r, "onPsiKitStart");
    r.run("idle", 40, NOTHING);
    r.run("b-pass", 400, (g, i) -> g.b = i < 2);
    r.run("mark-in", 10, (g, i) -> g.x = i < 2);
    r.run("b-pass-2", 400, (g, i) -> g.b = i < 2);
    r.run("mark-long-left", 12, (g, i) -> {
      g.x = i < 6;
      g.dpad_up = i == 2;
      g.dpad_left = i == 3;
    });
    r.run("start-again", 400, (g, i) -> g.start = i < 2);
    r.run("y-unseen", 10, (g, i) -> g.y = i < 2);
    r.run("y-skip", 10, (g, i) -> g.y = i < 2);
    r.run("back-gates", 10, (g, i) -> g.back = i < 2 || (i >= 5 && i < 7));
    r.run("b-pass-3", 60, (g, i) -> g.b = i < 2);
    r.run("stick-stops", 20, (g, i) -> g.left_stick_y = i < 3 ? -0.5f : 0f);
    r.run("back-gates-2", 10, (g, i) -> g.back = i < 2);
    r.run("lb-backward", 10, (g, i) -> g.left_bumper = i < 2);
    r.run("b-backward-pass", 400, (g, i) -> g.b = i < 2);
    r.run("mark-bounced", 10, (g, i) -> g.a = i < 2);
    r.run("rb-standing", 150, (g, i) -> {
      g.right_bumper = true;
      g.left_trigger = i < 40 ? 1f : 0f;
    });
    r.stop();

    r.sim.truth = RobotLoopTraceTest.nearCell(Alliance.BLUE, 2.0); // Facing the Hive tags again
    nextOpMode(r, new SotmTuning());
    r.init("init-carry-on", 20, (g, i) -> g.y = (i >= 5 && i < 8) || (i >= 12 && i < 15));
    call(r, "onPsiKitStart");
    r.run("idle-carried-on", 40, NOTHING);
    r.run("b-carried-on", 400, (g, i) -> g.b = i < 2);
    r.run("mark-carried-on", 10, (g, i) -> g.x = i < 2);
    r.stop();
  }

  /** Special Shot Calibration: drive to the spot, shoot, mark, change the shot by hand, and the far-side sticks. */
  private static void specialShot(Run r) {
    r.sim.truth = RobotLoopTraceTest.nearCell(Alliance.BLUE, 2.0);
    open(r, new SpecialShotCalibration());
    r.init("init", 20, (g, i) -> g.back = (i >= 5 && i < 8) || (i >= 12 && i < 15));
    call(r, "onPsiKitStart");
    r.run("idle", 40, NOTHING);
    r.run("b-drive", 40, (g, i) -> g.b = i < 2);
    r.run("stick-stops", 20, (g, i) -> g.left_stick_x = i < 3 ? 0.4f : 0f);
    r.run("b-drive-2", 300, (g, i) -> g.b = i < 2);
    r.run("y-aim", 60, (g, i) -> g.y = i < 2);
    r.run("rb-fire", 150, (g, i) -> g.right_bumper = true);
    r.run("mark-in", 10, (g, i) -> g.x = i < 2);
    r.run("rb-fire-2", 150, (g, i) -> g.right_bumper = true);
    r.run("mark-rattled", 10, (g, i) -> g.a = i < 2);
    r.run("rb-fire-unmarked", 150, (g, i) -> g.right_bumper = true);
    r.run("dpad-shot", 40, (g, i) -> {
      g.dpad_up = i == 0 || i == 4;
      g.dpad_down = i == 8;
      g.dpad_right = i == 12;
      g.dpad_left = i == 16 || i == 20;
    });
    r.run("stick-press-aim", 20, (g, i) -> {
      g.left_stick_button = i == 0;
      g.right_stick_button = i == 4 || i == 8;
    });
    r.sim.balls = 0;
    r.run("rb-empty", 150, (g, i) -> g.right_bumper = true);
    r.sim.balls = 3;
    r.run("lb-estimate", 20, (g, i) -> g.left_bumper = i < 2);
    r.run("back-wall-side", 10, (g, i) -> g.back = i < 2);
    r.run("drive", 40, (g, i) -> {
      g.left_stick_y = -0.5f;
      g.right_stick_x = i < 10 ? 0.3f : 0f;
    });
    r.run("lt-intake", 40, (g, i) -> g.left_trigger = 1f);
    r.stop();
  }

  /** The next OpMode on a new simulated Robot, where the last one left it, with the same CSV folder. */
  private static void nextOpMode(Run r, PsiKitIterativeOpMode op) {
    double[] truth = r.sim.truth;
    double[] odometry = {r.sim.pinpoint.x, r.sim.pinpoint.y, r.sim.pinpoint.h};
    r.sim = new RobotSim(false);
    r.sim.truth = truth;
    r.sim.pinpoint.x = odometry[0];
    r.sim.pinpoint.y = odometry[1];
    r.sim.pinpoint.h = odometry[2];
    open(r, op);
  }

  /** The scenario with its OpMode's CSVs in a folder of its own, empty at the start and checked at the end. */
  private static Consumer<Run> csv(Consumer<Run> scenario) {
    return r -> {
      File folder = new File(RobotLoopTraceTest.moduleDir(), "build/robot-trace/csv/" + r.name);
      folder.mkdirs();
      for (File f : listed(folder)) {
        f.delete();
      }
      setStatic(TuningCsv.class, "directory", folder);
      try {
        scenario.accept(r);
      } finally {
        setStatic(TuningCsv.class, "directory", null);
      }
      for (File f : listed(folder)) {
        try {
          r.recordFile(f.getName(), new String(Files.readAllBytes(f.toPath()), StandardCharsets.UTF_8));
        } catch (IOException e) {
          throw new IllegalStateException(e);
        }
      }
    };
  }

  private static List<File> listed(File folder) {
    File[] files = folder.listFiles();
    List<File> list = files == null ? Arrays.asList() : Arrays.asList(files);
    list.sort(null);
    return list;
  }

  /** The OpMode on the run's simulated hardware, through INIT; the run's loops call its hooks from then on. */
  private static void open(Run r, PsiKitIterativeOpMode op) {
    op.hardwareMap = r.sim.hardwareMap;
    op.telemetry = r.sim.telemetry;
    op.gamepad1 = r.sim.gamepad;
    op.gamepad2 = new Gamepad();
    set(op, "session", SESSION);
    r.opMode = op;
    call(r, "onPsiKitInit");
    waitForPlanner();
    r.sim.attach((Robot) get(op, "robot"), op);
    r.initCode = () -> call(r, "onPsiKitInitLoop");
    r.loopCode = () -> call(r, "onPsiKitLoop");
    r.stopCode = () -> call(r, "onPsiKitStop");
  }

  /** SOTM Tuning plans its passes on a thread; on the robot that takes a while in INIT, here it is done first. */
  private static void waitForPlanner() {
    for (Thread t : Thread.getAllStackTraces().keySet()) {
      if (t.getName().equals("SotmPlanner")) {
        try {
          t.join();
        } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
          throw new IllegalStateException(e);
        }
      }
    }
    for (String line : AndroidLogLines.drain()) {
      if (!line.contains("Planned in")) { // How long it took: different every run
        AndroidLogLines.add(line);
      }
    }
  }

  private static void call(Run r, String hook) {
    try {
      Method m = PsiKitIterativeOpMode.class.getDeclaredMethod(hook);
      m.setAccessible(true);
      m.invoke(r.opMode);
    } catch (InvocationTargetException e) {
      if (e.getCause() instanceof RuntimeException) {
        throw (RuntimeException) e.getCause();
      }
      throw new IllegalStateException(e.getCause());
    } catch (ReflectiveOperationException e) {
      throw new IllegalStateException(e);
    }
  }

  private static Object get(Object o, String name) {
    try {
      Field f = o.getClass().getDeclaredField(name);
      f.setAccessible(true);
      return f.get(o);
    } catch (ReflectiveOperationException e) {
      throw new IllegalStateException(e);
    }
  }

  private static void set(Object o, String name, Object value) {
    try {
      Field f = o.getClass().getDeclaredField(name);
      f.setAccessible(true);
      f.set(o, value);
    } catch (ReflectiveOperationException e) {
      throw new IllegalStateException(e);
    }
  }

  private static void setStatic(Class<?> c, String name, Object value) {
    try {
      Field f = c.getDeclaredField(name);
      f.setAccessible(true);
      f.set(null, value);
    } catch (ReflectiveOperationException e) {
      throw new IllegalStateException(e);
    }
  }
}
