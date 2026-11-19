package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmodes.LebotOpMode;
import org.firstinspires.ftc.teamcode.opmodes.Main;
import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem.FeederState;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem.IndexerState;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem.ShooterState;
import org.firstinspires.ftc.teamcode.tuning.BallDipDetector;
import org.firstinspires.ftc.teamcode.tuning.Buttons;
import org.firstinspires.ftc.teamcode.tuning.Buttons.Button;
import org.firstinspires.ftc.teamcode.tuning.FlywheelTest;
import org.firstinspires.ftc.teamcode.tuning.TuningCsv;
import org.psilynx.psikit.core.Logger;

import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Locale;

/**
 * Tunes the flywheels' speed control with one button. The robot doesn't drive.
 *
 * <p>Gamepad 1:
 * <ul>
 *   <li>A: speed test (about 25 s): the wheels step between 2500 and 4000 rpm; it shows how fast they get there,
 *       how far past they go and where they settle.</li>
 *   <li>Y: ball test, with balls already in the robot: it fires them one at a time at 3500 rpm and shows how far the
 *       wheels dip and how long they take to recover. POINT THE SHOOTER AT A NET OR THE HIVE.</li>
 *   <li>B: stop everything.</li>
 *   <li>X: try the suggested values (only until this OpMode stops; nothing is saved), then run again. Copy them into ShooterSubsystem
 *       once a run says GOOD.</li>
 * </ul>
 * Results are added to {@link #CSV_PATH}.
 */
@TeleOp(name = "Flywheel Tuning", group = "Tuning")
public class FlywheelTuning extends LebotOpMode {
  static final String CSV_PATH = "/sdcard/FIRST/FlywheelTuning.csv";
  private static final String CSV_HEADER = "session,run,kind,from_rpm,to_rpm,p,i,f,reach_s,settle_s,overshoot_rpm,"
      + "end_error_rpm,end_spread_rpm,dip_rpm,recover_s,battery_v";
  private static final double FEED_TIMEOUT_S = 0.8;
  private static final int MAX_BALLS = 4;

  private final String session = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
  private final ElapsedTime clock = new ElapsedTime();
  private final Buttons buttons = new Buttons();
  private final FlywheelTest test = new FlywheelTest();
  private final BallDipDetector dip = new BallDipDetector();
  private final List<FlywheelTest.BallResult> balls = new ArrayList<>();
  private final double[] suggested = new double[3];
  private List<String> advice = new ArrayList<>();
  private boolean hasSuggestion = false;
  private ShooterSubsystem shooter;
  private FeederSubsystem feeder;
  private IndexerSubsystem indexer;
  private int runs = 0;
  private String fileError = null;
  // Statics outlive the OpMode on the robot: put back on stop, so a tried value never reaches a match
  private double originalP = Double.NaN;
  private double originalF = Double.NaN;
  private final double originalHandback = ShooterSubsystem.HANDBACK_RPM;

  // Ball test: waiting for the wheels, feeding, or watching the recovery
  private enum BallStage { OFF, SETTLE, FEED, RECOVER }
  private BallStage ballStage = BallStage.OFF;
  private double stageStartS;
  private double settledSinceS = Double.NaN;
  private int emptyFeeds;

  @Override
  protected void onPsiKitConfigureLogging() {
    Main.configureLogging();
    Logger.recordMetadata("OpMode", "Flywheel Tuning");
    Logger.recordMetadata("TuningSession", session);
  }

  @Override
  protected void onPsiKitInit() {
    CommandScheduler.getInstance().reset();
    shooter = new ShooterSubsystem(hardwareMap); // Sets the PIDF defaults
    originalP = ShooterSubsystem.VELOCITY_P;
    originalF = ShooterSubsystem.VELOCITY_F;
    feeder = new FeederSubsystem(hardwareMap);
    indexer = new IndexerSubsystem(hardwareMap);
    shooter.setShooterState(ShooterState.STOP);
    feeder.setFeederState(FeederState.STOP);
    indexer.setIndexerState(IndexerState.STOP);
  }

  @Override
  protected void onPsiKitInitLoop() {
    telemetry.addLine("FLYWHEEL TUNING: press START, then A for the speed test, Y for the ball test.");
    telemetry.update();
  }

  @Override
  protected void onPsiKitStart() {
    clock.reset();
    feeder.setFeederState(FeederState.HOLD); // Keeps any balls back from the wheels
  }

  @Override
  protected void onPsiKitLoop() {
    double nowS = clock.seconds();
    buttons.update(gamepad1);
    double rpm = shooter.getRpm();
    if (buttons.pressed(Button.A) && ballStage == BallStage.OFF) {
      test.start(nowS, rpm);
      stepWritten = 0;
      advice = new ArrayList<>();
      runs++;
    }
    if (buttons.pressed(Button.Y) && !test.running()) {
      balls.clear();
      emptyFeeds = 0;
      startBallStage(BallStage.SETTLE, nowS);
      runs++;
    }
    if (buttons.pressed(Button.B)) {
      test.stop();
      ballStage = BallStage.OFF;
    }
    if (buttons.pressed(Button.X) && hasSuggestion && !test.running() && ballStage == BallStage.OFF) {
      ShooterSubsystem.VELOCITY_P = suggested[0];
      ShooterSubsystem.VELOCITY_F = suggested[1];
      ShooterSubsystem.HANDBACK_RPM = suggested[2];
      hasSuggestion = false;
      advice = new ArrayList<>();
      advice.add("Suggested values in use. Press A to run again.");
    }

    boolean wasRunning = test.running();
    test.record(nowS, rpm);
    while (stepWritten < test.steps().size()) {
      writeStep(test.steps().get(stepWritten++));
    }
    if (wasRunning && test.finished()) {
      finishAdvice();
    }
    updateBalls(nowS, rpm);

    double target = test.running() ? test.targetRpm() : ballStage != BallStage.OFF ? FlywheelTest.BALL_RPM : 0;
    if (target > 0) {
      shooter.setShooterState(ShooterState.AIM);
      shooter.setTargetRpm(target);
    } else {
      shooter.setShooterState(ShooterState.STOP);
    }
    CommandScheduler.getInstance().run();
    Logger.recordOutput("FlywheelTuning/TargetRpm", target);
    Logger.recordOutput("FlywheelTuning/Stage", test.running() ? test.progress() : ballStage.name());
    showTelemetry(rpm, target);
  }

  private int stepWritten = 0;

  private void finishAdvice() {
    advice = FlywheelTest.advice(test.steps(), balls, ShooterSubsystem.VELOCITY_P, ShooterSubsystem.VELOCITY_F,
        ShooterSubsystem.HANDBACK_RPM, suggested);
    hasSuggestion = advice.get(0).startsWith("CHANGE");
  }

  private void startBallStage(BallStage stage, double nowS) {
    ballStage = stage;
    stageStartS = nowS;
    settledSinceS = Double.NaN;
    boolean feeding = stage == BallStage.FEED;
    feeder.setFeederState(feeding ? FeederState.FEED : FeederState.HOLD);
    indexer.setIndexerState(feeding ? IndexerState.FEED : IndexerState.STOP);
  }

  /** Ball test: wheels settled at BALL_RPM, feed one, watch the dip and recovery, again until empty. */
  private void updateBalls(double nowS, double rpm) {
    boolean within = Math.abs(rpm - FlywheelTest.BALL_RPM) <= FlywheelTest.TOLERANCE_RPM;
    switch (ballStage) {
      case SETTLE:
        if (!within) {
          settledSinceS = Double.NaN;
        } else if (Double.isNaN(settledSinceS)) {
          settledSinceS = nowS;
        } else if (nowS - settledSinceS >= FlywheelTest.BALL_SETTLE_S) {
          dip.start(nowS, rpm);
          startBallStage(BallStage.FEED, nowS);
        }
        break;
      case FEED:
        BallDipDetector.Result r = dip.update(nowS, rpm);
        if (r == BallDipDetector.Result.BALL) {
          startBallStage(BallStage.RECOVER, nowS); // Recovery is timed from the dip showing
        } else if (nowS - stageStartS > FEED_TIMEOUT_S) {
          emptyFeeds++;
          if (emptyFeeds >= 2) {
            endBalls();
          } else {
            startBallStage(BallStage.SETTLE, nowS);
          }
        }
        break;
      case RECOVER:
        dip.update(nowS, rpm);
        boolean back = rpm >= FlywheelTest.BALL_RPM - FlywheelTest.TOLERANCE_RPM;
        if (back || nowS - stageStartS > 2.0) {
          FlywheelTest.BallResult ball = new FlywheelTest.BallResult(dip.dipRpm(),
              back ? nowS - stageStartS : Double.NaN);
          balls.add(ball);
          writeBall(ball);
          emptyFeeds = 0;
          if (balls.size() >= MAX_BALLS) {
            endBalls();
          } else {
            startBallStage(BallStage.SETTLE, nowS);
          }
        }
        break;
      default:
        break;
    }
  }

  private void endBalls() {
    startBallStage(BallStage.OFF, 0);
    feeder.setFeederState(FeederState.HOLD);
    advice = FlywheelTest.advice(test.steps(), balls, ShooterSubsystem.VELOCITY_P, ShooterSubsystem.VELOCITY_F,
        ShooterSubsystem.HANDBACK_RPM, suggested);
    hasSuggestion = advice.get(0).startsWith("CHANGE");
    if (balls.isEmpty()) {
      advice.add("No ball left the robot: load balls first (Lebot Teleop LT), then Y");
    }
  }

  private void showTelemetry(double rpm, double target) {
    telemetry.addData("Flywheel Tuning", test.running() ? "SPEED TEST " + test.progress() + "  (B stops)"
        : ballStage != BallStage.OFF ? "BALL TEST " + balls.size() + " balls so far  (B stops)"
        : "A: speed test   Y: ball test (aim at a net!)   X: try suggested values");
    telemetry.addData("Wheels", String.format(Locale.US, "%.0f rpm, target %.0f, battery %.1f V", rpm, target,
        batteryVolts()));
    telemetry.addData("PIDF", String.format(Locale.US, "P %.0f  I %.1f  F %.2f  handback %.0f",
        ShooterSubsystem.VELOCITY_P, ShooterSubsystem.VELOCITY_I, ShooterSubsystem.VELOCITY_F,
        ShooterSubsystem.HANDBACK_RPM));
    for (int i = 0; i < advice.size(); i++) {
      telemetry.addData(i == 0 ? "RESULT" : "Fix " + i, advice.get(i));
    }
    for (FlywheelTest.StepResult s : test.steps()) {
      telemetry.addLine(s.line());
    }
    for (FlywheelTest.BallResult b : balls) {
      telemetry.addLine(b.line());
    }
    if (fileError != null) {
      telemetry.addData("CSV", fileError);
    }
    telemetry.update();
  }

  private double batteryVolts() {
    double lowest = Double.NaN;
    for (VoltageSensor s : hardwareMap.voltageSensor) {
      double v = s.getVoltage();
      if (v > 0 && !(v >= lowest)) {
        lowest = v;
      }
    }
    return lowest;
  }

  private void writeStep(FlywheelTest.StepResult s) {
    write(String.format(Locale.US, "%s,%d,step,%.0f,%.0f,%.1f,%.2f,%.3f,%.3f,%.3f,%.0f,%.0f,%.0f,,,%.2f", session,
        runs, s.fromRpm, s.toRpm, ShooterSubsystem.VELOCITY_P, ShooterSubsystem.VELOCITY_I,
        ShooterSubsystem.VELOCITY_F, s.reachS, s.settleS, s.overshootRpm, s.endErrorRpm, s.endSpreadRpm,
        batteryVolts()));
  }

  private void writeBall(FlywheelTest.BallResult b) {
    write(String.format(Locale.US, "%s,%d,ball,%.0f,%.0f,%.1f,%.2f,%.3f,,,,,,%.0f,%.3f,%.2f", session, runs,
        FlywheelTest.BALL_RPM, FlywheelTest.BALL_RPM, ShooterSubsystem.VELOCITY_P, ShooterSubsystem.VELOCITY_I,
        ShooterSubsystem.VELOCITY_F, b.dipRpm, b.recoverS, batteryVolts()));
  }

  private void write(String row) {
    try {
      TuningCsv.append(CSV_PATH, CSV_HEADER, row);
    } catch (IOException e) {
      fileError = "could not write " + CSV_PATH + ": " + e.getMessage();
    }
  }

  @Override
  protected void onPsiKitStop() {
    if (!Double.isNaN(originalP)) {
      ShooterSubsystem.VELOCITY_P = originalP;
      ShooterSubsystem.VELOCITY_F = originalF;
    }
    ShooterSubsystem.HANDBACK_RPM = originalHandback;
    if (shooter != null) {
      shooter.setShooterState(ShooterState.STOP);
      feeder.setFeederState(FeederState.STOP);
      indexer.setIndexerState(IndexerState.STOP);
    }
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().reset();
  }
}
