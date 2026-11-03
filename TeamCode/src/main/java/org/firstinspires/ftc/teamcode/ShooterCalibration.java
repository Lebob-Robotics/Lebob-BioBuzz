package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem.FeederState;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.utils.HoodStepControl;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem.IndexerState;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem.ShooterState;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.utils.CellSighting;
import org.firstinspires.ftc.teamcode.utils.OneBallFeedGate;
import org.firstinspires.ftc.teamcode.utils.ShootOnTheMove;
import org.firstinspires.ftc.teamcode.utils.ShootOnTheMoveConstants;
import org.firstinspires.ftc.teamcode.utils.Shot;
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
 * <li>D-pad up/down: target RPM. Right raises the hood 2 degrees per press; left lowers it 2.</li>
 * <li>Right bumper, held: fire one ball at a time, each once the wheels are back at speed.</li>
 * <li>B, held: run the feeder and indexer backward to clear a jam.</li>
 * <li>Left bumper, held: index. The indexer moves balls up; the feeder holds them out of the wheels.</li>
 * <li>Y: load the shot table's RPM and hood angle for the distance the cameras see.</li>
 * <li>X: mark the last shot as scored (logged with its distance, RPM and hood angle).</li>
 * </ul>
 *
 * <p>The cameras give the distance from the shooter to the Cell's tag cluster and near lip, so a
 * shot that scores can be logged against where it was taken from.
 *
 * <p>Telemetry shows the RPM and hood angle each shot was fired at. Write those down, not the
 * targets: the real speed is what the ball got.
 */
@TeleOp(name = "Shooter Calibration", group = "Tuning")
public class ShooterCalibration extends PsiKitIterativeOpMode {
  private static final double START_RPM = 3500;
  private static final double RPM_STEP = 100;
  private static final double START_HOOD_DEG = HoodSubsystem.ZERO_ANGLE_DEG;
  private static final double RPM_TOLERANCE = 75; // The speed reading steps by about 60 RPM
  private static final double TRIGGER_PRESSED = 0.5;
  private static final double BATTERY_SAMPLE_PERIOD_S = 0.1; // Each read is a hub round trip

  private ShooterSubsystem shooter;
  private HoodSubsystem hood;
  private FeederSubsystem feeder;
  private IndexerSubsystem indexer;

  private double targetRpm = START_RPM;
  private double hoodDeg = START_HOOD_DEG;
  private boolean flywheelsOn = false;
  private final OneBallFeedGate feedGate = new OneBallFeedGate();
  private final com.qualcomm.robotcore.util.ElapsedTime clock = new com.qualcomm.robotcore.util.ElapsedTime();
  private int shotCount = 0;
  private double lastShotRpm = Double.NaN;
  private double lastShotHoodDeg = Double.NaN;
  private double lastShotBatteryVolts = Double.NaN;
  private VoltageSensor battery;
  private double batteryVolts = Double.NaN;
  private double lastBatterySampleS = Double.NEGATIVE_INFINITY;

  private VisionSubsystem vision;
  private double lipDistM = Double.NaN;
  private double clusterDistM = Double.NaN;
  private String clusterName = "";
  private long lastSightingNanos;
  private double lastShotLipDistM = Double.NaN;
  private int scoredCount = 0;
  private boolean lastShotScored;
  private boolean wasX;
  private boolean wasY;

  private boolean wasA;
  private boolean wasDpadUp;
  private boolean wasDpadDown;
  private final HoodStepControl hoodSteps = new HoodStepControl();

  @Override
  protected int getRlogPort() {
    return Main.LIVE_LOGGING ? Main.RLOG_PORT : 0;
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
    vision = new VisionSubsystem(hardwareMap);
    // Both hubs read the same battery, so one sensor is enough.
    battery = hardwareMap.voltageSensor.iterator().hasNext()
        ? hardwareMap.voltageSensor.iterator().next() : null;
  }

  @Override
  protected void onPsiKitInitLoop() {
    hood.periodic(); // STOW target: log both zeroed encoders without requesting hood travel.
    updateDistance();
    telemetry.addData("Distance", distanceText());
    telemetry.addData("Hood encoders", hood.hasEncoderFault() ? "FAULT: check wiring and re-INIT" : "ok");
    telemetry.addLine("Hood must rest on its lower stop now. Robot does not drive.");
    telemetry.update();
  }

  @Override
  protected void onPsiKitStart() {}

  @Override
  protected void onPsiKitLoop() {
    updateBattery();
    updateDistance();
    updateTargets();

    shooter.setShooterState(flywheelsOn ? ShooterState.AIM : ShooterState.STOP);
    shooter.setTargetRpm(targetRpm);
    hood.setTargetTicks(HoodSubsystem.angleToTicks(hoodDeg));

    boolean readyToFeed = shooter.atSpeed(RPM_TOLERANCE) && hood.atTarget();
    boolean feeding = feedGate.update(clock.seconds(), gamepad1.right_bumper, readyToFeed,
        shooter.getRpm(), targetRpm);
    if (feedGate.feedStarted()) {
      recordShot(); // Speed and angle as the ball is fed, before it slows the wheels
    }
    if (feedGate.ballLeft()) {
      shotCount++;
      lastShotScored = false;
    }
    updateFeederAndIndexer(feeding);

    CommandScheduler.getInstance().run(); // After the states are set, so subsystems act on and log this loop's

    Logger.recordOutput("Calibration/ShotCount", shotCount);
    Logger.recordOutput("Calibration/LastShotRpm", lastShotRpm);
    Logger.recordOutput("Calibration/LastShotHoodDeg", lastShotHoodDeg);
    Logger.recordOutput("Calibration/LastShotLipDistM", lastShotLipDistM);
    Logger.recordOutput("Calibration/LastShotScored", lastShotScored);
    Logger.recordOutput("Calibration/ScoredCount", scoredCount);
    Logger.recordOutput("Calibration/LipDistM", lipDistM);
    Logger.recordOutput("Calibration/ClusterDistM", clusterDistM);
    Logger.recordOutput("Calibration/ClusterName", clusterName);
    Logger.recordOutput("Calibration/BatteryVolts", batteryVolts);
    Logger.recordOutput("Calibration/LastShotBatteryVolts", lastShotBatteryVolts);
    addTelemetry(readyToFeed);
  }

  @Override
  protected void onPsiKitStop() {
    if (vision != null) { vision.close(); }
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
    double lift = hoodSteps.update(HoodSubsystem.ZERO_ANGLE_DEG - hoodDeg,
        gamepad1.dpad_right, gamepad1.dpad_left,
        HoodSubsystem.ZERO_ANGLE_DEG - HoodSubsystem.MIN_ANGLE_DEG);
    hoodDeg = HoodSubsystem.ZERO_ANGLE_DEG - lift;
    if (justPressed(gamepad1.y, wasY) && !Double.isNaN(lipDistM)) {
      Shot shot = ShootOnTheMove.calculate(lipDistM, 0, 0);
      if (shot.valid) {
        targetRpm = Math.min(shooter.getMaxRpm(), Math.round(shot.targetRpm / 10.0) * 10.0);
        hoodDeg = shot.hoodDeg;
      }
    }
    if (justPressed(gamepad1.x, wasX) && shotCount > 0 && !lastShotScored) {
      lastShotScored = true;
      scoredCount++;
    }
    wasX = gamepad1.x;
    wasY = gamepad1.y;
    wasA = gamepad1.a;
    wasDpadUp = gamepad1.dpad_up;
    wasDpadDown = gamepad1.dpad_down;
  }

  /** Takes the speed and angle the moment feeding starts, before the ball slows the wheel. */
  private void recordShot() {
    lastShotRpm = shooter.getRpm();
    lastShotHoodDeg = HoodSubsystem.ticksToAngle(hood.getPosition());
    lastShotLipDistM = lipDistM;
    lastShotBatteryVolts = batteryVolts;
  }

  /** Samples the battery every {@link #BATTERY_SAMPLE_PERIOD_S}; it sags as the wheels spin up and with each ball. */
  private void updateBattery() {
    double now = clock.seconds();
    if (battery != null && now - lastBatterySampleS >= BATTERY_SAMPLE_PERIOD_S) {
      lastBatterySampleS = now;
      batteryVolts = battery.getVoltage();
    }
  }

  /** Nearest Cell cluster in this loop's new frames, from the shooter, along the floor. */
  private void updateDistance() {
    vision.periodic();
    double nearestM = Double.NaN;
    for (CellSighting sighting : vision.takeNewCellSightings()) {
      double[] inRobot = sighting.camera.toRobotFrame(sighting.rightM, sighting.forwardM, sighting.upM);
      double distM = Math.hypot(inRobot[0] - ShootOnTheMoveConstants.SHOOTER_FORWARD_M,
          inRobot[1] - ShootOnTheMoveConstants.SHOOTER_LEFT_M);
      if (Double.isNaN(nearestM) || distM < nearestM) {
        nearestM = distM;
        clusterName = sighting.clusterName;
        lastSightingNanos = sighting.frameTimeNanos;
      }
    }
    if (!Double.isNaN(nearestM)) {
      clusterDistM = nearestM;
      lipDistM = nearestM - ShootOnTheMoveConstants.CLUSTER_TO_NEAR_LIP_M;
    }
    if (System.nanoTime() - lastSightingNanos > 1_000_000_000L) { // Seen over a second ago: unknown
      lipDistM = Double.NaN;
      clusterDistM = Double.NaN;
    }
  }

  private String distanceText() {
    if (Double.isNaN(lipDistM)) {
      return "no Cell tags seen";
    }
    return String.format("%.2f m to near lip (%.2f m to tags, %s)", lipDistM, clusterDistM, clusterName);
  }

  private void updateFeederAndIndexer(boolean feeding) {
    if (gamepad1.b) {
      feeder.setFeederState(FeederState.EJECT);
      indexer.setIndexerState(IndexerState.EJECT);
    } else if (feeding) {
      feeder.setFeederState(FeederState.FEED);
      indexer.setIndexerState(IndexerState.FEED);
    } else if (gamepad1.left_bumper) { // Index: queue balls up to the feeder, which holds them back
      feeder.setFeederState(FeederState.HOLD);
      indexer.setIndexerState(IndexerState.FEED);
    } else {
      feeder.setFeederState(FeederState.STOP);
      indexer.setIndexerState(IndexerState.STOP);
    }
  }

  private void addTelemetry(boolean readyToFeed) {
    telemetry.addData("Flywheels", flywheelsOn ? "ON" : "off  (A)");
    telemetry.addData("Battery", "%.2f V", batteryVolts);
    telemetry.addData("RPM", "target %.0f, actual %.0f", targetRpm, shooter.getRpm());
    telemetry.addData("Hood lift (deg)", "target %.1f, actual %.1f",
        HoodSubsystem.ZERO_ANGLE_DEG - hoodDeg, hood.getLiftDegrees());
    telemetry.addData("Launch angle (deg)", "target %.1f, actual %.1f",
        hoodDeg, HoodSubsystem.ticksToAngle(hood.getPosition()));
    telemetry.addData("Hood encoders", hood.hasEncoderFault() ? "FAULT: stop and check wiring, then re-INIT at lower stop" : "ok");
    telemetry.addData("Ready to feed", readyToFeed ? "YES (right bumper)" : "no");
    telemetry.addData("Index", "hold left bumper");
    telemetry.addData("Shooter stall", shooter.getStallStatus());
    telemetry.addLine();
    telemetry.addData("Distance", distanceText());
    if (!Double.isNaN(lipDistM)) {
      Shot shot = ShootOnTheMove.calculate(lipDistM, 0, 0);
      telemetry.addData("Table says (Y)", shot.valid
          ? String.format("rpm %.0f, hood %.1f deg (lift %.1f)", shot.targetRpm, shot.hoodDeg,
              HoodSubsystem.ZERO_ANGLE_DEG - shot.hoodDeg)
          : "no shot: " + shot.reason);
    }
    telemetry.addData("Shot", "%d fired, %d scored (X marks last as in)", shotCount, scoredCount);
    telemetry.addData("Last shot", "rpm %.0f, hoodDeg %.1f, lip %.2f m, %.2f V%s",
        lastShotRpm, lastShotHoodDeg, lastShotLipDistM, lastShotBatteryVolts, lastShotScored ? "  IN" : "");
    telemetry.update();
  }

  private static boolean justPressed(boolean pressedNow, boolean pressedLastLoop) {
    return pressedNow && !pressedLastLoop;
  }
}
