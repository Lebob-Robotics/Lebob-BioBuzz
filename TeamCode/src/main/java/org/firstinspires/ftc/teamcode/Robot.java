package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem.FeederState;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem.HoodPreset;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem.IndexerState;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem.IntakeState;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem.ShooterState;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.utils.Aim;
import org.firstinspires.ftc.teamcode.utils.CellSighting;
import org.firstinspires.ftc.teamcode.utils.Pose;
import org.firstinspires.ftc.teamcode.utils.ShootOnTheMove;
import org.firstinspires.ftc.teamcode.utils.ShootOnTheMoveConstants;
import org.firstinspires.ftc.teamcode.utils.TargetTracker;
import org.psilynx.psikit.core.Logger;

/**
 * Turns gamepad input into subsystem states each loop, and runs shoot on the move.
 *
 * <p>Controls (gamepad 1):
 * <ul>
 *   <li>Left stick: drive (field centric). Right stick X: turn.</li>
 *   <li>Right trigger held: aim and shoot. The robot turns to the shot heading and sets the hood and
 *       flywheels from the shot table, then feeds each ball once the shot is ready. The left stick
 *       still drives.</li>
 *   <li>Right bumper: toggle flywheel spin-up. Keeps the flywheels at the table's RPM so the first
 *       aimed shot is ready sooner.</li>
 *   <li>Y held: manual shot at full power, without aiming or waiting. A fallback for when the camera
 *       can't see the Cell.</li>
 *   <li>Left trigger held, or left bumper to toggle: intake. Both stop at 4 balls.</li>
 *   <li>B: eject. X: stop the shooter (also ends spin-up). A: reset heading.</li>
 *   <li>D-pad: hood presets, or while aiming, up and down trim the RPM.</li>
 *   <li>During INIT, X picks blue and B picks red.</li>
 * </ul>
 */
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

  public static final double TRIGGER_PRESSED = 0.5;

  /** Where shoot on the move is up to. Shown to the driver and logged. */
  public enum AimState {
    OFF, // Right trigger not held
    CELL_NOT_SEEN,
    NO_SAFE_SHOT,
    LINING_UP, // Valid shot, waiting for the flywheels, hood or heading
    READY // A ball fed now should score
  }

  private Alliance alliance = Alliance.RED;
  private final TargetTracker tracker = new TargetTracker(ShootOnTheMoveConstants.POSE_HISTORY_SIZE);
  private AimState aimState = AimState.OFF;
  private double aimRpm = ShootOnTheMoveConstants.AIM_START_RPM; // Last valid shot's, so a brief invalid shot doesn't drop it
  private double rpmTrim = 0; // Driver's, for the rest of the match
  private boolean flywheelSpinUpOn = false;
  private boolean intakeOn = false;
  private boolean wasFull = false;
  private boolean wasDpadUp = false;
  private boolean wasDpadDown = false;
  private boolean wasLeftBumper = false;
  private boolean wasRightBumper = false;

  public Robot(HardwareMap hardwareMap, Telemetry telemetry, Gamepad driver) {
    this.telemetry = telemetry;
    this.driver = driver;

    CommandScheduler.getInstance().reset();

    intake = new IntakeSubsystem(hardwareMap);
    drive = new MecanumDriveSubsystem(hardwareMap);
    odometry = new OdometrySubsystem(hardwareMap);
    shooter = new ShooterSubsystem(hardwareMap);
    feeder = new FeederSubsystem(hardwareMap);
    indexer = new IndexerSubsystem(hardwareMap);
    hood = new HoodSubsystem(hardwareMap);
    vision = new VisionSubsystem(hardwareMap);
  }

  /** Called once when the OpMode enters INIT. */
  public void init() {
    odometry.init();
    hood.init();
    shooter.setShooterState(ShooterState.IDLE);
  }

  /** Called repeatedly during INIT: the driver picks the alliance. */
  public void initLoop() {
    pollAlliance();
    telemetry.addData("Alliance", alliance + "  (X = BLUE, B = RED)");
    telemetry.update();
  }

  /** Called once when the OpMode starts. */
  public void start() {
    pollAlliance();
    tracker.clear();
  }

  private void pollAlliance() {
    if (driver.x) {
      alliance = Alliance.BLUE;
    } else if (driver.b) {
      alliance = Alliance.RED;
    }
  }

  /** Called repeatedly while the OpMode is running. */
  public void periodic() {
    Pose2D pinpointPose = odometry.getPose(); // PsiKit has already updated the Pinpoint this loop
    Pose pose = new Pose(pinpointPose.getX(DistanceUnit.METER), pinpointPose.getY(DistanceUnit.METER),
        pinpointPose.getHeading(AngleUnit.RADIANS));
    double[] velocity = odometry.getVelocityMetersPerSec(); // {x, y, turn rate}
    long nowNanos = vision.getCaptureNanos(); // Logged, so replay gives the same result

    if (driver.a) {
      odometry.resetHeading();
      tracker.clear(); // The field frame just turned, so the stored Cell and poses no longer apply
      pose = new Pose(pose.xM, pose.yM, 0);
    }

    Aim aim = aimAtCell(pose, velocity, nowNanos);
    boolean aiming = driver.right_trigger > TRIGGER_PRESSED;
    boolean ready = aim != null && ShootOnTheMove.readyToFire(aim,
        shooter.atSpeed(ShootOnTheMoveConstants.RPM_TOLERANCE),
        hood.atTarget(hoodToleranceTicks()),
        pose.headingRad);
    updateAimState(aiming, aim, ready);

    updateTrim(aiming);
    updateToggles();
    updateDrive(aiming, aim, pose, velocity[2]);
    updateShooterAndHood(aiming, aim);
    updateIntakeFeederIndexer(aiming, ready);

    CommandScheduler.getInstance().run(); // After the states are set, so subsystems act on and log this loop's

    logAim(aim);
    addTelemetry(aim, pose);
    telemetry.update();
  }

  /** Null when the Cell hasn't been seen recently. */
  private Aim aimAtCell(Pose pose, double[] velocity, long nowNanos) {
    tracker.recordPose(nowNanos, pose);
    for (CellSighting sighting : vision.takeNewCellSightings()) {
      if (alliance.ownsCluster(sighting.clusterName)) {
        tracker.update(sighting);
      }
    }
    if (!tracker.knowsCell(nowNanos)) {
      return null;
    }
    return ShootOnTheMove.aim(pose, velocity[0], velocity[1], tracker.getCellX(), tracker.getCellY());
  }

  /** Also rumbles once each time the shot becomes ready. */
  private void updateAimState(boolean aiming, Aim aim, boolean ready) {
    AimState newState;
    if (!aiming) {
      newState = AimState.OFF;
    } else if (aim == null) {
      newState = AimState.CELL_NOT_SEEN;
    } else if (!aim.shot.valid) {
      newState = AimState.NO_SAFE_SHOT;
    } else if (ready) {
      newState = AimState.READY;
    } else {
      newState = AimState.LINING_UP;
    }
    if (newState == AimState.READY && aimState != AimState.READY) {
      driver.rumble(150);
    }
    aimState = newState;
  }

  /** While aiming, D-pad up and down nudge the RPM, for when the table is a little off on the day. */
  private void updateTrim(boolean aiming) {
    if (aiming && justPressed(driver.dpad_up, wasDpadUp)) {
      rpmTrim += ShootOnTheMoveConstants.RPM_TRIM_STEP;
    }
    if (aiming && justPressed(driver.dpad_down, wasDpadDown)) {
      rpmTrim -= ShootOnTheMoveConstants.RPM_TRIM_STEP;
    }
    wasDpadUp = driver.dpad_up;
    wasDpadDown = driver.dpad_down;
  }

  /** Right bumper toggles flywheel spin-up, left bumper toggles the intake. X ends spin-up. */
  private void updateToggles() {
    if (justPressed(driver.right_bumper, wasRightBumper)) {
      flywheelSpinUpOn = !flywheelSpinUpOn;
    }
    if (driver.x) {
      flywheelSpinUpOn = false;
    }
    if (justPressed(driver.left_bumper, wasLeftBumper)) {
      intakeOn = !intakeOn;
    }
    wasRightBumper = driver.right_bumper;
    wasLeftBumper = driver.left_bumper;
  }

  private static boolean justPressed(boolean pressedNow, boolean pressedLastLoop) {
    return pressedNow && !pressedLastLoop;
  }

  private void updateDrive(boolean aiming, Aim aim, Pose pose, double turnRateRadPerSec) {
    double forward = -driver.left_stick_y;
    double right = driver.left_stick_x;
    if (aiming && aim != null) { // Lines up even without a valid shot, so it's ready the moment one appears
      drive.driveHeadingLock(forward, right, aim.targetHeadingRad, pose.headingRad, turnRateRadPerSec);
    } else {
      drive.drive(forward, right, driver.right_stick_x, true, pose.headingRad);
    }
  }

  private void updateShooterAndHood(boolean aiming, Aim aim) {
    boolean validShot = aim != null && aim.shot.valid;
    if (validShot) {
      aimRpm = aim.shot.targetRpm;
    }

    if (aiming) {
      shooter.setShooterState(ShooterState.AIM);
      shooter.setTargetRpm(aimRpm + rpmTrim);
    } else if (driver.y) {
      shooter.setShooterState(ShooterState.SHOOT);
    } else if (driver.b) {
      shooter.setShooterState(ShooterState.EJECT);
    } else if (flywheelSpinUpOn) {
      shooter.setShooterState(ShooterState.AIM);
      shooter.setTargetRpm(aimRpm + rpmTrim);
    } else if (driver.x) {
      shooter.setShooterState(ShooterState.STOP);
    } else {
      shooter.setShooterState(ShooterState.IDLE);
    }

    if (aiming) {
      if (validShot) { // Otherwise the hood holds its last angle
        hood.setTargetTicks(HoodSubsystem.angleToTicks(aim.shot.hoodDeg));
      }
    } else if (driver.dpad_up) {
      hood.setPreset(HoodPreset.FAR);
    } else if (driver.dpad_right) {
      hood.setPreset(HoodPreset.MID);
    } else if (driver.dpad_down) {
      hood.setPreset(HoodPreset.CLOSE);
    } else if (driver.dpad_left) {
      hood.setPreset(HoodPreset.STOW);
    }
  }

  private void updateIntakeFeederIndexer(boolean aiming, boolean ready) {
    boolean shooting = aiming || driver.y; // Aimed shots wait for READY at the indexer
    boolean intaking = driver.left_trigger > TRIGGER_PRESSED || intakeOn;
    boolean ejecting = driver.b;
    boolean full = intake.isFull();
    boolean loading = intaking && !full; // Shooting keeps everything running, so balls can leave
    if (intaking && full && !wasFull) {
      driver.rumble(300);
    }
    if (full) {
      intakeOn = false; // So it doesn't start again by itself once balls are shot
    }
    wasFull = full;

    if (ejecting) {
      intake.setIntakeState(IntakeState.EJECT);
      feeder.setFeederState(FeederState.EJECT);
    } else if (loading || shooting) {
      intake.setIntakeState(IntakeState.INTAKE);
      feeder.setFeederState(FeederState.FEED);
    } else {
      intake.setIntakeState(IntakeState.STOP);
      feeder.setFeederState(FeederState.STOP);
    }

    indexer.setFeedAllowed(!aiming || ready); // Before the state, so FEED never writes full power to a closed gate
    if (ejecting) {
      indexer.setIndexerState(IndexerState.EJECT);
    } else if (shooting) {
      indexer.setIndexerState(IndexerState.FEED);
    } else if (loading) {
      indexer.setIndexerState(IndexerState.HOLD);
    } else {
      indexer.setIndexerState(IndexerState.STOP);
    }
  }

  private static int hoodToleranceTicks() {
    return (int) Math.round(ShootOnTheMoveConstants.HOOD_TOLERANCE_DEG * HoodSubsystem.TICKS_PER_DEG);
  }

  /** Everything shoot on the move worked out this loop, for AdvantageScope. */
  private void logAim(Aim aim) {
    Logger.recordOutput("SOTM/State", aimState.name());
    Logger.recordOutput("SOTM/Alliance", alliance.name());
    Logger.recordOutput("SOTM/KnowsCell", aim != null);
    Logger.recordOutput("SOTM/CellX", tracker.getCellX());
    Logger.recordOutput("SOTM/CellY", tracker.getCellY());
    Logger.recordOutput("SOTM/RpmTrim", rpmTrim);
    Logger.recordOutput("Robot/FlywheelSpinUpOn", flywheelSpinUpOn);
    Logger.recordOutput("Robot/IntakeOn", intakeOn);
    if (aim == null) {
      return;
    }
    Logger.recordOutput("SOTM/Valid", aim.shot.valid);
    Logger.recordOutput("SOTM/Reason", aim.shot.reason.name());
    Logger.recordOutput("SOTM/DistanceToLipM", aim.shot.distanceToLipM);
    Logger.recordOutput("SOTM/SpeedTowardCellMps", aim.shot.speedTowardCellMps);
    Logger.recordOutput("SOTM/SidewaysSpeedMps", aim.shot.sidewaysSpeedMps);
    Logger.recordOutput("SOTM/TargetRpm", aim.shot.targetRpm);
    Logger.recordOutput("SOTM/HoodDeg", aim.shot.hoodDeg);
    Logger.recordOutput("SOTM/TimeOfFlightS", aim.shot.timeOfFlightS);
    Logger.recordOutput("SOTM/TurningRad", aim.shot.turningRad);
    Logger.recordOutput("SOTM/TargetHeadingRad", aim.targetHeadingRad);
    Logger.recordOutput("SOTM/HeadingToleranceRad", aim.headingToleranceRad);
  }

  private void addTelemetry(Aim aim, Pose pose) {
    telemetry.addData("Alliance", alliance);
    telemetry.addData("Aim", aimState);
    if (aim != null) {
      telemetry.addData("Shot", aim.shot.valid ? "OK" : aim.shot.reason);
      telemetry.addData("Dist", String.format("%.2f m  toward %.2f  sideways %.2f m/s",
          aim.shot.distanceToLipM, aim.shot.speedTowardCellMps, aim.shot.sidewaysSpeedMps));
      telemetry.addData("RPM", String.format("%.0f target / %.0f actual  (trim %+.0f)",
          shooter.getTargetRpm(), shooter.getRpm(), rpmTrim));
      telemetry.addData("Hood", String.format("%.1f target / %.1f actual deg",
          HoodSubsystem.ticksToAngle(hood.getTarget()), HoodSubsystem.ticksToAngle(hood.getPosition())));
      telemetry.addData("Heading err", String.format("%.1f deg (allowed %.1f)",
          Math.toDegrees(aim.headingErrorRad(pose.headingRad)), Math.toDegrees(aim.headingToleranceRad)));
    }
    telemetry.addData("Pose", String.format("%.2f %.2f m  %.0f deg",
        pose.xM, pose.yM, Math.toDegrees(pose.headingRad)));
    telemetry.addData("Shooter", shooter.getShooterState() + (flywheelSpinUpOn ? "  (spin-up on)" : ""));
    telemetry.addData("Intake", intakeOn ? "ON (toggle)" : intake.getIntakeState());
    telemetry.addData("Indexer", indexer.getIndexerState()
        + (indexer.isFeedAllowed() ? "" : " (waiting)"));
    telemetry.addData("Beam T/B", intake.isTopBroken() + " / " + intake.isBottomBroken()
        + (intake.isFull() ? "  FULL" : ""));
    telemetry.addData("Camera L", vision.describe(VisionSubsystem.Side.LEFT));
    telemetry.addData("Camera R", vision.describe(VisionSubsystem.Side.RIGHT));
  }

  /** Called once when the OpMode stops. */
  public void stop() {
    vision.close();
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().reset();
  }
}
