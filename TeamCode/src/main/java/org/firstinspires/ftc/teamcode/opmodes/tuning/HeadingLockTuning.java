package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.tuning.Buttons;
import org.firstinspires.ftc.teamcode.tuning.Buttons.Button;
import org.firstinspires.ftc.teamcode.tuning.HeadingLockTest;
import org.firstinspires.ftc.teamcode.tuning.TuningCsv;
import org.psilynx.psikit.core.Logger;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Tunes the heading lock, the turn that aims every shot, with one button. The robot turns on the spot: steps to
 * 30 and 90 degrees and back (standing shots), then follows a target sweeping at 0.5, 1 and 2 rad/s (the turn
 * shooting on the move asks for). It then says GOOD, or what to change and to what.
 *
 * <p>Gamepad 1:
 * <ul>
 *   <li>A: run the test (about 25 s). Needs a metre clear all round; the robot only turns.</li>
 *   <li>B: stop.</li>
 *   <li>X: try the suggested values (only until this OpMode stops; nothing is saved), then A to run again. Copy them into
 *       MecanumDriveSubsystem once a run says GOOD.</li>
 *   <li>Sticks, while no test runs: drive slowly to a clear spot.</li>
 * </ul>
 * Every run's results are added to {@link #CSV_PATH}.
 */
@TeleOp(name = "Heading Lock Tuning", group = "Tuning")
public class HeadingLockTuning extends TuningOpMode {
  static final String CSV_PATH = "/sdcard/FIRST/HeadingLockTuning.csv";
  private static final String CSV_HEADER = "session,run,segment,kp,kd,kv,ks,latency_s,reach_s,settle_s,"
      + "overshoot_deg,end_error_deg,lag_deg,max_lag_deg,battery_v";
  private static final double POSITION_POWER = 0.4;

  private final HeadingLockTest test = new HeadingLockTest();
  private final Buttons buttons = new Buttons();
  private List<String> advice = new ArrayList<>();
  private final double[] suggested = new double[4];
  private boolean hasSuggestion = false;
  private int runs = 0;
  private String fileError = null;
  // Statics outlive the OpMode on the robot: put back on stop, so a tried value never reaches a match
  private final double[] original = {MecanumDriveSubsystem.HEADING_KP, MecanumDriveSubsystem.HEADING_KD,
      MecanumDriveSubsystem.HEADING_KV, MecanumDriveSubsystem.HEADING_KS};

  public HeadingLockTuning() {
    super("Heading Lock Tuning");
  }

  @Override
  protected void onPsiKitInit() {
    initTuningRobot();
    robot.setSpinOnlyOnFire(true);
    robot.setAutoDriver((pose, velocity) -> {
      if (test.running()) {
        return new double[] {0, 0, 0};
      }
      Gamepad pad = robot.getDriver();
      return new double[] {-pad.left_stick_y * POSITION_POWER, -pad.left_stick_x * POSITION_POWER,
          -pad.right_stick_x * POSITION_POWER};
    });
  }

  @Override
  protected void onPsiKitInitLoop() {
    telemetry.addLine("HEADING LOCK TUNING: press START, then A. The robot turns on the spot; keep 1 m clear.");
    robot.initLoop();
  }

  @Override
  protected void onPsiKitStart() {
    robot.start();
    clock.reset();
  }

  @Override
  protected void onPsiKitLoop() {
    double nowS = clock.seconds();
    buttons.update(robot.getDriver());
    if (buttons.pressed(Button.A)) {
      test.start(nowS, robot.getPose().headingRad);
      advice = new ArrayList<>();
      runs++;
    }
    if (buttons.pressed(Button.B)) {
      test.stop();
    }
    if (buttons.pressed(Button.X) && hasSuggestion && !test.running()) {
      MecanumDriveSubsystem.HEADING_KP = suggested[0];
      MecanumDriveSubsystem.HEADING_KD = suggested[1];
      MecanumDriveSubsystem.HEADING_KV = suggested[2];
      MecanumDriveSubsystem.HEADING_KS = suggested[3];
      hasSuggestion = false;
      advice = new ArrayList<>();
      advice.add("Suggested values in use. Press A to run again.");
    }
    double[] target = test.target(nowS);
    robot.setAutoHeading(target == null ? Double.NaN : target[0], target == null ? 0 : target[1]);
    boolean wasRunning = test.running();
    int before = test.results().size();

    robot.periodic();

    test.record(nowS, robot.getPose().headingRad);
    for (int i = before; i < test.results().size(); i++) {
      writeRow(test.results().get(i));
    }
    if (wasRunning && test.finished()) {
      advice = HeadingLockTest.advice(test.results(), MecanumDriveSubsystem.HEADING_KP,
          MecanumDriveSubsystem.HEADING_KD, MecanumDriveSubsystem.HEADING_KV, MecanumDriveSubsystem.HEADING_KS,
          suggested);
      hasSuggestion = advice.get(0).startsWith("CHANGE");
    }
    Logger.recordOutput("HeadingTuning/TargetRad", target == null ? Double.NaN : target[0]);
    Logger.recordOutput("HeadingTuning/TargetRateRadPerSec", target == null ? 0 : target[1]);
    Logger.recordOutput("HeadingTuning/Segment", test.progress());
    showTelemetry();
  }

  private void showTelemetry() {
    line("Heading Lock Tuning", test.running() ? "RUNNING " + test.progress() + "  (B stops)"
        : "A: run the test   X: try suggested values   sticks: move");
    line("Gains", String.format(Locale.US, "KP %.2f  KD %.2f  KV %.2f  KS %.2f",
        MecanumDriveSubsystem.HEADING_KP, MecanumDriveSubsystem.HEADING_KD, MecanumDriveSubsystem.HEADING_KV,
        MecanumDriveSubsystem.HEADING_KS));
    for (int i = 0; i < advice.size(); i++) {
      line(i == 0 ? "RESULT" : "Fix " + i, advice.get(i));
    }
    for (HeadingLockTest.Result r : test.results()) {
      telemetry.addLine(r.line());
    }
    if (fileError != null) {
      line("CSV", fileError);
    }
    telemetry.update();
  }

  private void writeRow(HeadingLockTest.Result r) {
    String row = String.format(Locale.US, "%s,%d,%s,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f",
        session, runs, r.name, MecanumDriveSubsystem.HEADING_KP, MecanumDriveSubsystem.HEADING_KD,
        MecanumDriveSubsystem.HEADING_KV, MecanumDriveSubsystem.HEADING_KS, MecanumDriveSubsystem.HEADING_LATENCY_S,
        r.reachS, r.settleS, r.overshootDeg, r.endErrorDeg, r.lagDeg, r.maxLagDeg, robot.getBatteryVolts());
    try {
      TuningCsv.append(CSV_PATH, CSV_HEADER, row);
    } catch (IOException e) {
      fileError = "could not write " + CSV_PATH + ": " + e.getMessage();
    }
  }

  @Override
  protected void onPsiKitStop() {
    MecanumDriveSubsystem.HEADING_KP = original[0];
    MecanumDriveSubsystem.HEADING_KD = original[1];
    MecanumDriveSubsystem.HEADING_KV = original[2];
    MecanumDriveSubsystem.HEADING_KS = original[3];
    if (robot != null) {
      robot.stop();
    }
  }
}
