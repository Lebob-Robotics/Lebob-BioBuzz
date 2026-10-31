package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem.FeederState;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem.IndexerState;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem.ShooterState;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.PsiKitIterativeOpMode;

/**
 * Holds one flywheel RPM and one hood angle and fires single balls, for measuring ball speed per
 * RPM with slow motion video (tools/shots/fit-speed.js turns the readings into exitSpeedPerRpm).
 * The robot does not drive.
 *
 * <p>The hood must rest on its lower stop at INIT, as in the TeleOp. Controls:
 * <ul>
 * <li>A: flywheels on or off, held at the target RPM.</li>
 * <li>D-pad up and down: target RPM. Left and right: hood angle.</li>
 * <li>Right bumper, held: feed. Only feeds while both wheels are within RPM_TOLERANCE.</li>
 * <li>B, held: run the feeder and indexer backward to clear a jam.</li>
 * </ul>
 *
 * <p>Telemetry shows the RPM and hood angle each shot was fired at. Write those down, not the
 * targets: the real speed is what the ball got.
 */
@TeleOp(name = "Shooter Calibration", group = "Tuning")
public class ShooterCalibration extends PsiKitIterativeOpMode {
  private static final double START_RPM = 3500;
  private static final double RPM_STEP = 250;
  private static final double START_HOOD_DEG = 75;
  private static final double HOOD_STEP_DEG = 5;
  private static final double RPM_TOLERANCE = 50;
  private static final double TRIGGER_PRESSED = 0.5;

  private ShooterSubsystem shooter;
  private HoodSubsystem hood;
  private FeederSubsystem feeder;
  private IndexerSubsystem indexer;

  private double targetRpm = START_RPM;
  private double hoodDeg = START_HOOD_DEG;
  private boolean flywheelsOn = false;
  private boolean wasFeeding = false;
  private int shotCount = 0;
  private double lastShotRpm = Double.NaN;
  private double lastShotHoodDeg = Double.NaN;

  private boolean wasA;
  private boolean wasDpadUp;
  private boolean wasDpadDown;
  private boolean wasDpadLeft;
  private boolean wasDpadRight;

  @Override
  protected int getRlogPort() {
    return Main.LIVE_LOGGING ? 5800 : 0;
  }

  @Override
  protected void onPsiKitConfigureLogging() {
    Logger.recordMetadata("Robot", "BioBuzz");
    Logger.recordMetadata("OpMode", "Shooter Calibration");
  }

  @Override
  protected void onPsiKitInit() {
    shooter = new ShooterSubsystem(hardwareMap);
    hood = new HoodSubsystem(hardwareMap);
    feeder = new FeederSubsystem(hardwareMap);
    indexer = new IndexerSubsystem(hardwareMap);
    hood.init();
    shooter.setShooterState(ShooterState.STOP);
  }

  @Override
  protected void onPsiKitInitLoop() {
    telemetry.addLine("Hood must rest on its lower stop now. Robot does not drive.");
    telemetry.update();
  }

  @Override
  protected void onPsiKitStart() {}

  @Override
  protected void onPsiKitLoop() {
    updateTargets();

    shooter.setShooterState(flywheelsOn ? ShooterState.AIM : ShooterState.STOP);
    shooter.setTargetRpm(targetRpm);
    hood.setTargetTicks(HoodSubsystem.angleToTicks(hoodDeg));

    boolean feedHeld = gamepad1.right_bumper;
    boolean readyToFeed = shooter.atSpeed(RPM_TOLERANCE) && hood.atTarget();
    boolean feeding = feedHeld && readyToFeed;
    if (feeding && !wasFeeding) {
      recordShot();
    }
    wasFeeding = feeding;
    updateFeederAndIndexer(feeding);

    CommandScheduler.getInstance().run(); // After the states are set, so subsystems act on and log this loop's

    Logger.recordOutput("Calibration/ShotCount", shotCount);
    Logger.recordOutput("Calibration/LastShotRpm", lastShotRpm);
    Logger.recordOutput("Calibration/LastShotHoodDeg", lastShotHoodDeg);
    addTelemetry(readyToFeed);
  }

  @Override
  protected void onPsiKitStop() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().reset();
  }

  private void updateTargets() {
    if (justPressed(gamepad1.a, wasA)) {
      flywheelsOn = !flywheelsOn;
    }
    if (justPressed(gamepad1.dpad_up, wasDpadUp)) {
      targetRpm = Math.min(shooter.getMaxRpm(), targetRpm + RPM_STEP);
    }
    if (justPressed(gamepad1.dpad_down, wasDpadDown)) {
      targetRpm = Math.max(RPM_STEP, targetRpm - RPM_STEP);
    }
    if (justPressed(gamepad1.dpad_left, wasDpadLeft)) {
      hoodDeg = Math.max(HoodSubsystem.MIN_ANGLE_DEG, hoodDeg - HOOD_STEP_DEG);
    }
    if (justPressed(gamepad1.dpad_right, wasDpadRight)) {
      hoodDeg = Math.min(HoodSubsystem.ZERO_ANGLE_DEG, hoodDeg + HOOD_STEP_DEG);
    }
    wasA = gamepad1.a;
    wasDpadUp = gamepad1.dpad_up;
    wasDpadDown = gamepad1.dpad_down;
    wasDpadLeft = gamepad1.dpad_left;
    wasDpadRight = gamepad1.dpad_right;
  }

  /** Takes the speed and angle the moment feeding starts, before the ball slows the wheel. */
  private void recordShot() {
    shotCount++;
    lastShotRpm = shooter.getRpm();
    lastShotHoodDeg = HoodSubsystem.ticksToAngle(hood.getPosition());
  }

  private void updateFeederAndIndexer(boolean feeding) {
    if (gamepad1.b) {
      feeder.setFeederState(FeederState.EJECT);
      indexer.setIndexerState(IndexerState.EJECT);
    } else if (feeding) {
      feeder.setFeederState(FeederState.FEED);
      indexer.setIndexerState(IndexerState.FEED);
    } else {
      feeder.setFeederState(FeederState.STOP);
      indexer.setIndexerState(IndexerState.STOP);
    }
  }

  private void addTelemetry(boolean readyToFeed) {
    telemetry.addData("Flywheels", flywheelsOn ? "ON" : "off  (A)");
    telemetry.addData("RPM", "target %.0f, actual %.0f", targetRpm, shooter.getRpm());
    telemetry.addData("Hood (deg)", "target %.1f, actual %.1f",
        hoodDeg, HoodSubsystem.ticksToAngle(hood.getPosition()));
    telemetry.addData("Ready to feed", readyToFeed ? "YES (right bumper)" : "no");
    telemetry.addLine();
    telemetry.addData("Shot", shotCount);
    telemetry.addData("WRITE DOWN", "rpm %.0f, hoodDeg %.1f", lastShotRpm, lastShotHoodDeg);
    telemetry.update();
  }

  private static boolean justPressed(boolean pressedNow, boolean pressedLastLoop) {
    return pressedNow && !pressedLastLoop;
  }
}
