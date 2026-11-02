package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

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
import com.bylazar.gamepad.PanelsGamepad;
import org.firstinspires.ftc.teamcode.utils.Aim;
import org.firstinspires.ftc.teamcode.utils.ManualFeedGate;
import org.firstinspires.ftc.teamcode.utils.CellSighting;
import org.firstinspires.ftc.teamcode.utils.Pose;
import org.firstinspires.ftc.teamcode.utils.PoseHistory;
import org.firstinspires.ftc.teamcode.utils.ShootOnTheMove;
import org.firstinspires.ftc.teamcode.utils.ShootOnTheMoveConstants;
import org.firstinspires.ftc.teamcode.utils.ShooterTuning;
import org.firstinspires.ftc.teamcode.utils.TagObservation;
import org.firstinspires.ftc.teamcode.utils.TargetTracker;
import org.firstinspires.ftc.teamcode.utils.VisionCorrector;
import org.firstinspires.ftc.teamcode.utils.VisionResult;
import org.psilynx.psikit.core.Logger;

import java.util.List;

/**
 * Turns gamepad input into subsystem states each loop, and runs shoot on the move.
 *
 * <p>Controls (gamepad 1):
 * <ul>
 *   <li>Left stick: drive (field centric, forward is away from the driver's wall). Right stick X: turn.</li>
 *   <li>Right trigger held: aim and shoot. The robot turns to the shot heading and sets the hood and
 *       flywheels from the shot table, then feeds each ball once the shot is ready. The left stick
 *       still drives.</li>
 *   <li>Right bumper: toggle flywheel spin-up. Keeps the flywheels at the table's RPM so the first
 *       aimed shot is ready sooner.</li>
 *   <li>Y held: manual shot at full power, without aiming or waiting. A fallback for when the camera
 *       can't see the Cell.</li>
 *   <li>Left trigger held, or left bumper to toggle: intake. The feeder runs backward to queue the
 *       balls short of the shooter. Both stop at 4 balls once the beam breaks are wired.</li>
 *   <li>B: eject. X: stop the shooter (also ends spin-up, and resets a stall trip). A: tell the robot
 *       it is facing away from the driver's wall (use it if the heading drifts).</li>
 *   <li>D-pad: hood presets, or while aiming, up and down trim the RPM.</li>
 *   <li>During INIT, X picks blue and B picks red.</li>
 * </ul>
 *
 * <p>The odometry heading is in the field frame (0 along +x), so the Hive tags can correct the pose.
 * Driving uses the driver's heading, which is that heading less the alliance's forward heading.
 *
 * <p>Balls go from the intake to the indexer to the feeder to the shooter. The feeder is the last
 * stage, so it is what waits for READY on an aimed shot.
 */
public class Robot {
  private final Telemetry telemetry;
  /** The real gamepad 1, kept for rumble. */
  private final Gamepad physicalDriver;
  /** What the code reads: gamepad 1, plus the browser gamepad from Panels when Panels is on. Rebuilt each loop. */
  private Gamepad driver;

  public final IntakeSubsystem intake;
  public final MecanumDriveSubsystem drive;
  public final OdometrySubsystem odometry;
  public final ShooterSubsystem shooter;
  public final FeederSubsystem feeder;
  public final IndexerSubsystem indexer;
  public final HoodSubsystem hood;
  public final VisionSubsystem vision;

  public static final double TRIGGER_PRESSED = 0.5;
  /** Rumble when the shooter stall guard trips: drivers do not watch telemetry. */
  private static final int STALL_RUMBLE_MS = 1000;
  /** Battery voltage is a separate hub read, so sample it this often. */
  private static final double BATTERY_SAMPLE_PERIOD_S = 0.1;

  /** Null if the config has no voltage sensor. */
  private final VoltageSensor battery;
  private final ElapsedTime clock = new ElapsedTime();
  private double lastBatterySampleS = Double.NEGATIVE_INFINITY;
  private double batteryVolts = Double.NaN;
  private double batteryMinVolts = Double.NaN;

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
  // Vision pose correction. The history holds each loop's odometry pose with the time of that loop's
  // camera capture, so a frame can be matched to where the robot was. It is separate from the tracker's.
  private final PoseHistory poseHistory = new PoseHistory(ShootOnTheMoveConstants.POSE_HISTORY_SIZE);
  private final VisionCorrector visionCorrector = new VisionCorrector();
  private long lastHistoryNanos = Long.MIN_VALUE;
  private boolean poseValid = false; // True once a tag fix has placed the robot on the field. Nothing is gated on it yet
  private VisionResult lastVision = null; // Last update that looked at tags, for telemetry
  private long lastVisionNanos = 0;

  private AimState aimState = AimState.OFF;
  private double aimRpm = ShootOnTheMoveConstants.AIM_START_RPM; // Last valid shot's, so a brief invalid shot doesn't drop it
  private double rpmTrim = 0; // Driver's, for the rest of the match
  private boolean flywheelSpinUpOn = false;
  private final ManualFeedGate manualFeedGate = new ManualFeedGate();
  private boolean intakeOn = false;
  private boolean wasFull = false;
  private boolean wasDpadUp = false;
  private boolean wasDpadDown = false;
  private boolean wasLeftBumper = false;
  private boolean wasRightBumper = false;
  private boolean shooterWasStalled = false;

  public Robot(HardwareMap hardwareMap, Telemetry telemetry, Gamepad driver) {
    this.telemetry = telemetry;
    this.physicalDriver = driver;
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

    // Both hubs read the same battery, so one sensor is enough.
    battery = hardwareMap.voltageSensor.iterator().hasNext()
        ? hardwareMap.voltageSensor.iterator().next() : null;
  }

  /** Called once when the OpMode enters INIT. */
  public void init() {
    odometry.init();
    hood.init();
    // Off during INIT; the teleop loop idles the flywheels once the OpMode starts.
    shooter.setShooterState(ShooterState.STOP);
  }

  /**
   * Merges the Panels browser gamepad into gamepad 1 so either can drive. Panels builds a new
   * combined Gamepad on every call, so this runs once per loop. Off with Main.PANELS_ENABLED.
   */
  private void refreshDriver() {
    driver = Main.PANELS_ENABLED
        ? PanelsGamepad.INSTANCE.getFirstManager().asCombinedFTCGamepad(physicalDriver)
        : physicalDriver;
  }

  /**
   * Called repeatedly during INIT: the driver picks the alliance. The shooter is stopped here, but
   * its stall guard still runs so the shooter is logged from the start.
   */
  public void initLoop() {
    refreshDriver();
    pollAlliance();
    shooter.periodic();
    updateBattery();
    telemetry.addData("Alliance", alliance + "  (X = BLUE, B = RED)");
    telemetry.addData("Shooter stall", shooter.getStallStatus());
    telemetry.update();
  }

  /**
   * Called once when the OpMode starts. Sets the heading to the alliance's forward heading, which
   * assumes the robot is placed square to its wall, facing away from it. A resets it later.
   */
  public void start() {
    pollAlliance();
    odometry.setHeading(alliance.forwardHeadingRad());
    tracker.clear();
    poseHistory.clear();
    visionCorrector.reset();
    lastHistoryNanos = Long.MIN_VALUE;
    poseValid = false;
    lastVision = null;
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
    refreshDriver();
    Pose2D pinpointPose = odometry.getPose(); // PsiKit has already updated the Pinpoint this loop
    Pose pose = new Pose(pinpointPose.getX(DistanceUnit.METER), pinpointPose.getY(DistanceUnit.METER),
        pinpointPose.getHeading(AngleUnit.RADIANS));
    double[] velocity = odometry.getVelocityMetersPerSec(); // {x, y, turn rate}
    long nowNanos = vision.getCaptureNanos(); // Logged, so replay gives the same result

    if (driver.a) {
      odometry.setHeading(alliance.forwardHeadingRad());
      tracker.clear(); // The frame just turned, so the stored Cell and poses no longer apply
      poseHistory.clear();
      pose = new Pose(pose.xM, pose.yM, alliance.forwardHeadingRad());
    }

    pose = correctPoseFromVision(pose, velocity, nowNanos); // Before aiming, so this loop aims from the corrected pose

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
    // Without the Cell in view there is nothing to aim at, so RT shoots at full flywheel speed and the
    // held hood once the wheels are up to speed, instead of holding the feeder backward forever. It then keeps
    // feeding until RT is released: each ball drops the wheels' RPM, and reversing the feeder every
    // time they dip would pull the next ball back out.
    boolean manualReady = manualFeedGate.update(aiming && aim == null && !shooter.isStalled(),
        shooter.atSpeed(ShooterTuning.MANUAL_RPM_TOLERANCE));
    updateIntakeFeederIndexer(aiming, ready || manualReady);

    CommandScheduler.getInstance().run(); // After the states are set, so subsystems act on and log this loop's
    stampPoseHistory(pose); // After the run, which is when vision takes this loop's capture

    if (shooter.isStalled() && !shooterWasStalled) {
      physicalDriver.rumble(STALL_RUMBLE_MS);
    }
    shooterWasStalled = shooter.isStalled();

    updateBattery();
    logAim(aim);
    addTelemetry(aim, pose);
    telemetry.update();
  }

  /**
   * Corrects the pose from the Hive tags, and moves the tracker's stored Cell and poses by the same
   * amount so the aim does not jump. Returns the pose to use this loop.
   */
  private Pose correctPoseFromVision(Pose pose, double[] velocity, long nowNanos) {
    List<TagObservation> observations = vision.takeNewTagObservations();
    VisionResult result = visionCorrector.process(observations, poseHistory, pose,
        velocity[0], velocity[1], velocity[2], poseValid, nowNanos);
    if (result.accepted) {
      odometry.setPose(result.newPose.xM, result.newPose.yM, result.newPose.headingRad);
      poseHistory.shift(result.appliedDxM(), result.appliedDyM());
      tracker.shift(result.appliedDxM(), result.appliedDyM());
      poseValid = true; // The first fix is applied in full, so after it the pose is on the field
      pose = result.newPose; // The Pinpoint may not report the new pose until its next update
    }
    if (result.reason != VisionResult.Reason.NO_OBSERVATIONS) {
      lastVision = result;
      lastVisionNanos = nowNanos;
    }
    double frameAgeS = Double.NaN; // Of the newest frame handed in
    for (TagObservation observation : observations) {
      double ageS = (nowNanos - observation.frameTimeNanos) * 1e-9;
      frameAgeS = Double.isNaN(frameAgeS) ? ageS : Math.min(frameAgeS, ageS);
    }
    logVision(result, frameAgeS, observations.size());
    return pose;
  }

  /** Records this loop's pose against this loop's capture time, so later frames can be matched to it. */
  private void stampPoseHistory(Pose pose) {
    long stampNanos = vision.getCaptureNanos();
    if (stampNanos > lastHistoryNanos && stampNanos > 0) { // Not before the first capture, or twice for one
      poseHistory.add(stampNanos, pose);
      lastHistoryNanos = stampNanos;
    }
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
      physicalDriver.rumble(150);
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

  /** The stick directions are from the driver's wall, so the drive gets the heading less the alliance's forward. */
  private void updateDrive(boolean aiming, Aim aim, Pose pose, double turnRateRadPerSec) {
    double forward = -driver.left_stick_y;
    double right = driver.left_stick_x;
    double forwardOffsetRad = alliance.forwardHeadingRad();
    double driverHeadingRad = pose.headingRad - forwardOffsetRad;
    if (aiming && aim != null) { // Lines up even without a valid shot, so it's ready the moment one appears
      // Target and heading both less the offset, so the error is the same and the driving is driver relative
      drive.driveHeadingLock(forward, right, aim.targetHeadingRad - forwardOffsetRad, driverHeadingRad,
          turnRateRadPerSec);
    } else {
      drive.drive(forward, right, driver.right_stick_x, true, driverHeadingRad);
    }
  }

  private void updateShooterAndHood(boolean aiming, Aim aim) {
    boolean validShot = aim != null && aim.shot.valid;
    if (validShot) {
      aimRpm = aim.shot.targetRpm;
    }

    if (aiming) {
      shooter.setShooterState(ShooterState.AIM);
      // No Cell in view means no shot table to read, so go for the top speed
      shooter.setTargetRpm(ShooterTuning.RPM_OVERRIDE > 0
          ? ShooterTuning.RPM_OVERRIDE
          : aim == null ? shooter.getMaxRpm() : aimRpm + rpmTrim);
    } else if (driver.y) {
      if (ShooterTuning.RPM_OVERRIDE > 0) {
        shooter.setShooterState(ShooterState.AIM);
        shooter.setTargetRpm(ShooterTuning.RPM_OVERRIDE);
      } else {
        shooter.setShooterState(ShooterState.SHOOT);
      }
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

  /**
   * Shooting runs the indexer and feeder into the shooter with the intake off. An aimed shot holds the
   * feeder backward until READY, so the indexer queues balls against it. Loading runs the intake and
   * indexer with the feeder backward, so balls stop short of the shooter. Shooting wins over loading.
   */
  private void updateIntakeFeederIndexer(boolean aiming, boolean ready) {
    boolean shooting = aiming || driver.y;
    boolean intaking = driver.left_trigger > TRIGGER_PRESSED || intakeOn;
    boolean ejecting = driver.b;
    boolean full = intake.isFull();
    boolean loading = intaking && !full && !shooting;
    if (intaking && full && !wasFull) {
      physicalDriver.rumble(300);
    }
    if (full) {
      intakeOn = false; // So it doesn't start again by itself once balls are shot
    }
    wasFull = full;

    if (ejecting) {
      intake.setIntakeState(IntakeState.EJECT);
      indexer.setIndexerState(IndexerState.EJECT);
      feeder.setFeederState(FeederState.EJECT);
    } else if (shooting) {
      intake.setIntakeState(IntakeState.STOP);
      indexer.setIndexerState(IndexerState.FEED);
      feeder.setFeederState(!aiming || ready ? FeederState.FEED : FeederState.HOLD);
    } else if (loading) {
      intake.setIntakeState(IntakeState.INTAKE);
      indexer.setIndexerState(IndexerState.FEED);
      feeder.setFeederState(FeederState.HOLD);
    } else {
      intake.setIntakeState(IntakeState.STOP);
      indexer.setIndexerState(IndexerState.STOP);
      feeder.setFeederState(FeederState.STOP);
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

  /** Everything the pose correction worked out this loop. */
  private void logVision(VisionResult result, double frameAgeS, int newTags) {
    Logger.recordOutput("Vision/Correction/Accepted", result.accepted);
    Logger.recordOutput("Vision/Correction/Reason", result.reason.name());
    Logger.recordOutput("Vision/Correction/TagReject", result.firstTagReject.name());
    Logger.recordOutput("Vision/Correction/NewTags", newTags);
    Logger.recordOutput("Vision/Correction/DxM", result.dxM);
    Logger.recordOutput("Vision/Correction/DyM", result.dyM);
    Logger.recordOutput("Vision/Correction/Gain", result.gain);
    Logger.recordOutput("Vision/Correction/TagsUsed", result.tagsUsed);
    Logger.recordOutput("Vision/Correction/TagsRejected", result.tagsRejected);
    Logger.recordOutput("Vision/Correction/SpreadM", result.spreadM);
    Logger.recordOutput("Vision/Correction/UsedIds", result.usedIds);
    Logger.recordOutput("Vision/Correction/UsedDxM", result.usedDxM);
    Logger.recordOutput("Vision/Correction/UsedDyM", result.usedDyM);
    Logger.recordOutput("Vision/Correction/FrameAgeS", frameAgeS);
    Logger.recordOutput("Vision/Correction/UpCellRed", visionCorrector.upCell(Alliance.RED).name());
    Logger.recordOutput("Vision/Correction/UpCellBlue", visionCorrector.upCell(Alliance.BLUE).name());
    Logger.recordOutput("Vision/Correction/RedFlipped", result.redFlipped);
    Logger.recordOutput("Vision/Correction/BlueFlipped", result.blueFlipped);
    Logger.recordOutput("Vision/Correction/PoseValid", poseValid);
  }

  /** For example "✓ 6 tags Δ3cm (0.1s ago) | up AUDIENCE", or the reason the last tags were not used. */
  private String visionLine() {
    String upCell = "up " + visionCorrector.upCell(alliance);
    VisionResult result = lastVision;
    if (result == null) {
      return "no tags yet | " + upCell;
    }
    String text;
    if (result.accepted) {
      text = String.format("✓ %d tags Δ%.0fcm", result.tagsUsed, Math.hypot(result.dxM, result.dyM) * 100);
    } else if (result.reason == VisionResult.Reason.NO_VALID_TAGS) {
      text = "✗ " + result.firstTagReject;
    } else {
      text = "✗ " + result.reason;
    }
    double sinceS = (vision.getCaptureNanos() - lastVisionNanos) * 1e-9;
    return text + String.format(" (%.1fs ago) | ", sinceS) + upCell;
  }

  private void addTelemetry(Aim aim, Pose pose) {
    telemetry.addData("Alliance", alliance + (poseValid ? "  pose on field" : "  pose not placed yet"));
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
    telemetry.addData("Pose", String.format("%.2f %.2f m  %.0f deg  (driver %.0f)",
        pose.xM, pose.yM, Math.toDegrees(pose.headingRad),
        Math.toDegrees(pose.headingRad - alliance.forwardHeadingRad())));
    telemetry.addData("Vision", visionLine());
    telemetry.addData("Shooter", shooter.getShooterState() + (flywheelSpinUpOn ? "  (spin-up on)" : ""));
    telemetry.addData("Shooter stall", shooter.getStallStatus());
    telemetry.addData("Intake", intakeOn ? "ON (toggle)" : intake.getIntakeState());
    telemetry.addData("Indexer", indexer.getIndexerState());
    telemetry.addData("Feeder", feeder.getFeederState());
    telemetry.addData("Beam T/B", intake.isTopBroken() + " / " + intake.isBottomBroken()
        + (intake.isFull() ? "  FULL" : ""));
    telemetry.addData("Camera L", vision.describe(VisionSubsystem.Side.LEFT));
    telemetry.addData("Camera R", vision.describe(VisionSubsystem.Side.RIGHT));
  }

  /**
   * Samples the battery voltage, keeping the lowest seen this run (it sags when the shooter spins up),
   * then logs it and adds it to telemetry.
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

  /** Called once when the OpMode stops. */
  public void stop() {
    vision.close();
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().reset();
  }
}
