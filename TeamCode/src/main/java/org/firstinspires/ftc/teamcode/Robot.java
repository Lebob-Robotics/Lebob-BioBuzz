package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem.FeederState;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem.IndexerState;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem.IntakeState;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem.ShooterState;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.psilynx.psikit.core.Logger;

public class Robot {
  private final Telemetry telemetry;
  private final Gamepad driver;

  public final IntakeSubsystem intake;
  public final MecanumDriveSubsystem drive;
  public final OdometrySubsystem odometry;
  public final ShooterSubsystem shooter;
  public final FeederSubsystem feeder;
  public final IndexerSubsystem indexer;
  public final HoodSubsystem hood;
  public final VisionSubsystem vision;

  public static final double triggerLimit = 0.5;
  /** Hood power while d-pad up/down is held. Low, since nothing stops it at the ends. */
  private static final double HOOD_MANUAL_POWER = 0.3;
  /** Rumble when the shooter stall guard trips: drivers do not watch telemetry. */
  private static final int STALL_RUMBLE_MS = 1000;

  /** Battery voltage is a separate hub read, so sample it this often. */
  private static final double BATTERY_SAMPLE_PERIOD_S = 0.1;

  private boolean shooterWasStalled;

  /** Null if the config has no voltage sensor. */
  private final VoltageSensor battery;
  private final ElapsedTime clock = new ElapsedTime();
  private double lastBatterySampleS = Double.NEGATIVE_INFINITY;
  private double batteryVolts = Double.NaN;
  private double batteryMinVolts = Double.NaN;

  public Robot(
      HardwareMap hardwareMap,
      Telemetry telemetry,
      Gamepad driver) {
    this.telemetry = telemetry;
    this.driver = driver;

    CommandScheduler.getInstance().reset();

    // Construct subsystems.
    intake = new IntakeSubsystem(hardwareMap);
    drive = new MecanumDriveSubsystem(hardwareMap);
    odometry = new OdometrySubsystem(hardwareMap);
    shooter = new ShooterSubsystem(hardwareMap);
    feeder = new FeederSubsystem(hardwareMap);
    indexer = new IndexerSubsystem(hardwareMap);
    hood = new HoodSubsystem(hardwareMap);
    vision = new VisionSubsystem(hardwareMap);

    // Both hubs read the same battery, so one sensor is enough.
    battery = hardwareMap.voltageSensor.iterator().hasNext()
        ? hardwareMap.voltageSensor.iterator().next() : null;
  }

  /**
   * Samples the battery voltage, keeping the lowest seen this run (it sags
   * when the shooter spins up), then logs it and adds it to telemetry.
   */
  private void updateBattery() {
    double now = clock.seconds();
    if (battery != null && now - lastBatterySampleS >= BATTERY_SAMPLE_PERIOD_S) {
      lastBatterySampleS = now;
      batteryVolts = battery.getVoltage();
      batteryMinVolts = Double.isNaN(batteryMinVolts)
          ? batteryVolts : Math.min(batteryMinVolts, batteryVolts);
    }
    Logger.recordOutput("Robot/BatteryVolts", batteryVolts);
    Logger.recordOutput("Robot/BatteryMinVolts", batteryMinVolts);
    telemetry.addData("Battery", "%.2f V (lowest %.2f V)", batteryVolts, batteryMinVolts);
  }

  /** Called once when the OpMode enters INIT. */
  public void init() {
    odometry.init();
    hood.init();
    shooter.setShooterState(ShooterState.IDLE);
  }

  /**
   * Called repeatedly during INIT. The shooter already runs at IDLE here, so
   * its stall guard must too.
   */
  public void initLoop() {
    shooter.periodic();
    updateBattery();
    telemetry.addData("Shooter stall", shooter.getStallStatus());
    telemetry.update();
  }

  /** Called repeatedly while the OpMode is running. */
  public void periodic() {
    // The Pinpoint is already updated for this loop: PsiKit does it before
    // user code runs, so the pose read below is the one that gets logged.

    // Driver input: set every subsystem's state for this loop.
    // Field-centric needs the Pinpoint's heading.
    boolean fieldCentric = !driver.left_bumper && odometry.isAvailable();
    drive.drive(-driver.left_stick_y, driver.left_stick_x, driver.right_stick_x,
      fieldCentric, odometry.getPose().getHeading(AngleUnit.RADIANS));

    if (driver.a) {
      odometry.resetHeading();
    }

    boolean rightTriggerPressed = driver.right_trigger > triggerLimit;
    boolean leftTriggerPressed = driver.left_trigger > triggerLimit;
    boolean eject = driver.b;

    Logger.recordOutput("Robot/Driver/Shoot", rightTriggerPressed);
    Logger.recordOutput("Robot/Driver/Intake", leftTriggerPressed);
    Logger.recordOutput("Robot/Driver/Eject", eject);
    Logger.recordOutput("Robot/Driver/StopShooter", driver.x);
    Logger.recordOutput("Robot/Driver/FieldCentric", fieldCentric);
    Logger.recordOutput("Robot/Driver/ResetHeading", driver.a);

    ShooterState newShooterState = ShooterState.IDLE;
    if (rightTriggerPressed) {
      newShooterState = ShooterState.SHOOT;
    } else if (eject) {
      newShooterState = ShooterState.EJECT;
    } else if (driver.x) {
      newShooterState = ShooterState.STOP;
    }
    shooter.setShooterState(newShooterState);

    // LT loads: the intake and indexer bring balls up to the feeder, which
    // runs backward to hold them there. RT shoots: the indexer and feeder push
    // the queued balls into the shooter, and the intake stays off. RT wins if
    // both are held.
    boolean loading = leftTriggerPressed && !rightTriggerPressed;

    IntakeState newIntakeState = IntakeState.STOP;
    if (eject) {
      newIntakeState = IntakeState.EJECT;
    } else if (loading) {
      newIntakeState = IntakeState.INTAKE;
    }
    intake.setIntakeState(newIntakeState);

    FeederState newFeederState = FeederState.STOP;
    if (eject) {
      newFeederState = FeederState.EJECT;
    } else if (rightTriggerPressed) {
      newFeederState = FeederState.FEED;
    } else if (loading) {
      newFeederState = FeederState.HOLD;
    }
    feeder.setFeederState(newFeederState);

    IndexerState newIndexerState = IndexerState.STOP;
    if (eject) {
      newIndexerState = IndexerState.EJECT;
    } else if (rightTriggerPressed || loading) {
      newIndexerState = IndexerState.FEED;
    }
    indexer.setIndexerState(newIndexerState);

    // Manual hood while its encoder is not on the HoodMotor port: hold d-pad
    // up/down to move it. Presets come back once the encoder reads.
    if (driver.dpad_up) {
      hood.setManualPower(HOOD_MANUAL_POWER);
    } else if (driver.dpad_down) {
      hood.setManualPower(-HOOD_MANUAL_POWER);
    } else {
      hood.setManualPower(0.0);
    }

    // Run after the states above are set, so the hood P loop and every
    // subsystem's logging see this loop's commands rather than last loop's.
    CommandScheduler.getInstance().run();

    if (shooter.isStalled() && !shooterWasStalled) {
      driver.rumble(STALL_RUMBLE_MS);
    }
    shooterWasStalled = shooter.isStalled();

    updateBattery();
    telemetry.addData("Pose", odometry.isAvailable()
        ? odometry.getPose() : "no Pinpoint, robot-centric drive");
    telemetry.addData("Shooter", shooter.getShooterState()
        + (shooter.isAtSpeed() ? " at speed" : ""));
    telemetry.addData("Shooter stall", shooter.getStallStatus());
    telemetry.addData("Feeder", feeder.getFeederState());
    telemetry.addData("Indexer", indexer.getIndexerState());
    telemetry.addData("Hood ticks", hood.getPosition());
    telemetry.addData("Camera L", vision.describe(VisionSubsystem.Side.LEFT));
    telemetry.addData("Camera R", vision.describe(VisionSubsystem.Side.RIGHT));

    telemetry.update();
  }

  /** Called once when the OpMode stops. */
  public void stop() {
    vision.close();
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().reset();
  }
}
