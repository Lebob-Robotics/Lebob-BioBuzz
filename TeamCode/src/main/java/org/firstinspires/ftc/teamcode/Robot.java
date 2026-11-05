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
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.gamepad.PanelsGamepad;
import org.firstinspires.ftc.teamcode.utils.Aim;
import org.firstinspires.ftc.teamcode.utils.ManualFeedGate;
import org.firstinspires.ftc.teamcode.utils.HoodStepControl;
import org.firstinspires.ftc.teamcode.utils.HeadingHold;
import org.firstinspires.ftc.teamcode.utils.CellSighting;
import org.firstinspires.ftc.teamcode.utils.Pose;
import org.firstinspires.ftc.teamcode.utils.PoseHistory;
import org.firstinspires.ftc.teamcode.utils.RpmTrend;
import org.firstinspires.ftc.teamcode.utils.OneBallFeedGate;
import org.firstinspires.ftc.teamcode.utils.Shot;
import org.firstinspires.ftc.teamcode.utils.ShootOnTheMove;
import org.firstinspires.ftc.teamcode.utils.ShootOnTheMoveConstants;
import org.firstinspires.ftc.teamcode.utils.ShooterTuning;
import org.firstinspires.ftc.teamcode.utils.TagObservation;
import org.firstinspires.ftc.teamcode.utils.TargetTracker;
import org.firstinspires.ftc.teamcode.utils.VisionCorrector;
import org.firstinspires.ftc.teamcode.utils.VisionResult;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.wpi.math.Pose2d;
import org.psilynx.psikit.core.wpi.math.Rotation2d;

import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Turns gamepad input into subsystem states each loop, and runs shoot on the move.
 *
 * <p>Controls (gamepad 1):
 * <ul>
 *   <li>Left stick: drive (field centric, forward is away from the driver's wall). Right stick X: turn.
 *       Driving without turning holds the heading the robot had.</li>
 *   <li>Right trigger held: aim and shoot. The robot turns to the shot heading and sets the hood and
 *       flywheels from the shot table, then feeds each ball once the shot is ready. The left stick
 *       still drives. Without RT, once the Cell is known the hood already follows the shot RT would
 *       take, so it is there when RT is pressed (hood presets then do nothing).</li>
 *   <li>Right bumper held: as RT, but shoots one ball per press. Release and press again for the next.
 *       The flywheels already track the shot's RPM whenever the Cell is known, so there's no spin-up toggle.</li>
 *   <li>Y held: manual shot at full power, without aiming or waiting. A fallback for when the camera
 *       can't see the Cell.</li>
 *   <li>Left trigger held, or left bumper to toggle: intake. The feeder runs backward to queue the
 *       balls short of the shooter. Both stop at 4 balls once the beam breaks are wired.</li>
 *   <li>B: eject. X: stop the shooter (also ends spin-up, and resets a stall trip). A: tell the robot
 *       it is facing away from the driver's wall (use it if the heading drifts).</li>
 *   <li>D-pad right/left: raise/lower hood 2 degrees per press (hood trim while aiming).
 *       Up/down: hood presets, or RPM trim while aiming.</li>
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
  /** Feedforward cap: very close to the Cell the lock heading can swing faster than the robot can turn. */
  private static final double MAX_LOCK_RATE_RAD_S = 3.0;
  /** Share of each loop's new sideways-turn rate taken in: it comes from differenced odometry velocity. */
  private static final double TURNING_RATE_SMOOTHING = 0.5;

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
  // True once two tags have set the heading, or the driver reset it with A. Until then a heading fix may be any size
  private boolean headingValid = false;
  private VisionResult lastVision = null; // Last update that looked at tags, for telemetry
  private long lastVisionNanos = 0;

  private AimState aimState = AimState.OFF;
  private double aimRpm = ShootOnTheMoveConstants.AIM_START_RPM; // Last valid shot's, so a brief invalid shot doesn't drop it
  private final HoodStepControl hoodSteps = new HoodStepControl();
  private double hoodLiftTrimDeg = 0;
  private double rpmTrim = 0; // Driver's, for the rest of the match
  private final ManualFeedGate manualFeedGate = new ManualFeedGate();
  // An aimed shot feeds one ball at a time, each once the wheels are back up to speed
  private final OneBallFeedGate aimedFeedGate = new OneBallFeedGate(
      ShootOnTheMoveConstants.AIMED_READY_S, ShootOnTheMoveConstants.AIMED_FEED_STOP_UNREADY_S);
  private double cellSeenAgoS = Double.POSITIVE_INFINITY;
  private boolean aimingAtFieldCell = false; // At the up Cell's field position, not the tracker's sighted one
  private double aimCellX = 0; // The point aimed at this loop, field metres
  private double aimCellY = 0;
  private boolean intakeOn = false;
  /** Shot direction picked when RT was pressed: true backward, false hooded, null not aiming. */
  private Boolean aimBackwardLatched = null;
  private double backwardBlockedSinceS = Double.NaN;
  // Rate the aimed shot's sideways turn changes, smoothed, for the fire check and the heading lock
  private double turningRateRadPerSec = 0.0;
  private final RpmTrend rpmTrend = new RpmTrend(ShootOnTheMoveConstants.RPM_TREND_WINDOW_S);
  private double lastTurningRad = Double.NaN;
  private double lastTurningS = Double.NaN;
  private boolean lastTurningBackward = false;
  private static final double PREPOSITION_SETTLE_S = 0.5;
  private boolean prepositionBackward = false;
  private double prepositionSwitchSinceS = Double.NaN;
  private boolean wasFull = false;
  private boolean wasDpadUp = false;
  private boolean wasDpadDown = false;
  private boolean wasLeftBumper = false;
  private boolean shooterWasStalled = false;
  private final HeadingHold headingHold = new HeadingHold();

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
    // What the code acts on, Driver Hub and Panels merged. PsiKit only logs the Driver Hub gamepad, so
    // without this a run driven from Panels shows no input
    Logger.recordOutput("Driver/LeftStick", new double[] {driver.left_stick_x, driver.left_stick_y});
    Logger.recordOutput("Driver/RightStickX", driver.right_stick_x);
    Logger.recordOutput("Driver/Triggers", new double[] {driver.left_trigger, driver.right_trigger});
    if (Main.PANELS_ENABLED) {
      com.bylazar.gamepad.GamepadManager panels = PanelsGamepad.INSTANCE.getFirstManager();
      Logger.recordOutput("Driver/PanelsLeftStick", new double[] {panels.getLeftStickX(), panels.getLeftStickY()});
    }
  }

  /**
   * Called repeatedly during INIT: the driver picks the alliance. The shooter is stopped here, but
   * its stall guard still runs so the shooter is logged from the start.
   */
  public void initLoop() {
    refreshDriver();
    pollAlliance();
    shooter.periodic();
    hood.periodic(); // Drives the hood down onto its stop and zeroes it there; nothing else moves
    vision.periodic(); // Read and log tags in INIT without running any mechanism commands.
    updateBattery();
    telemetry.addData("Alliance", alliance + "  (X = BLUE, B = RED)");
    telemetry.addData("Shooter stall", shooter.getStallStatus());
    telemetry.addData("Hood", hood.getHomeResult());
    telemetry.addData("Hood encoders", hood.getEncoderFaultText());
    telemetry.addData("Camera L", vision.describe(VisionSubsystem.Side.LEFT));
    telemetry.addData("Camera R", vision.describe(VisionSubsystem.Side.RIGHT));
    telemetry.update();
  }

  /**
   * Called once when the OpMode starts. Sets the heading to the alliance's forward heading, which
   * assumes the robot is placed square to its wall, facing away from it. From then on two Hive tags
   * seen together correct it automatically; A resets it by hand.
   */
  public void start() {
    pollAlliance();
    odometry.setHeading(alliance.forwardHeadingRad());
    tracker.clear();
    poseHistory.clear();
    visionCorrector.reset();
    lastHistoryNanos = Long.MIN_VALUE;
    poseValid = false;
    headingValid = false;
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
    long t0 = System.nanoTime(); // Section times, logged under Loop/, to find what slows the loop
    refreshDriver();
    Pose2D pinpointPose = odometry.getPose(); // PsiKit has already updated the Pinpoint this loop
    Pose pose = new Pose(pinpointPose.getX(DistanceUnit.METER), pinpointPose.getY(DistanceUnit.METER),
        pinpointPose.getHeading(AngleUnit.RADIANS));
    double[] velocity = odometry.getVelocityMetersPerSec(); // {x, y, turn rate}
    long nowNanos = vision.getCaptureNanos(); // Logged, so replay gives the same result

    if (driver.a) {
      odometry.setHeading(alliance.forwardHeadingRad());
      tracker.clear(); // The frame just turned, so the stored Cell and poses no longer apply
      headingValid = true; // The driver says this is right, so vision only trims it from here
      headingHold.clear();
      poseHistory.clear();
      pose = new Pose(pose.xM, pose.yM, alliance.forwardHeadingRad());
    }

    long tVision = System.nanoTime();
    if (odometry.isReadGood()) { // A correction from a bad read would write that read back into the Pinpoint
      pose = correctPoseFromVision(pose, velocity, nowNanos); // Before aiming, so this loop aims from the corrected pose
    }

    boolean shootAll = driver.right_trigger > TRIGGER_PRESSED;
    boolean oneBall = driver.right_bumper && !shootAll; // RT wins when both are held
    boolean aiming = shootAll || driver.right_bumper;
    Aim aim = aimAtCell(pose, velocity, nowNanos, aiming);
    double lockRateRadPerSec = updateLockRate(aiming, aim);
    updateTrim(aiming);
    updateToggles();
    // Before the ready check, so it compares the hood and wheels with this loop's shot. After it, a hood target
    // that had just moved 3 degrees (the robot stopped) was judged against the old one, and the ball missed (5 Nov, 095605)
    updateShooterAndHood(aiming, aim);
    rpmTrend.update(clock.seconds(), shooter.getRpm());
    double predictedRpmError = rpmTrend.predicted(ShootOnTheMoveConstants.RPM_LOOKAHEAD_S) - shooter.getTargetRpm();
    Logger.recordOutput("Shooter/PredictedRpmError", predictedRpmError);
    boolean ready = aim != null && ShootOnTheMove.readyToFire(aim,
        shooter.atSpeed(ShootOnTheMoveConstants.RPM_TOLERANCE)
            && shooter.meanRpmWithin(ShootOnTheMoveConstants.RPM_MEAN_BELOW_TARGET,
                ShootOnTheMoveConstants.RPM_MEAN_ABOVE_TARGET)
            && predictedRpmError <= ShootOnTheMoveConstants.RPM_PREDICTED_ABOVE_TARGET,
        hood.atTarget(hoodToleranceTicks()),
        pose.headingRad, velocity[2], turningRateRadPerSec);
    // A feed under way stops if the shot goes bad before the ball is out: the driver set off or turned back
    // mid-feed, and the ball left 3 degrees of hood or 37 degrees of heading off (5 Nov, 095605)
    boolean shotLost = aim != null && (!aim.shot.valid
        || Math.abs(ShootOnTheMove.normalizeAngle(aim.lockHeadingRad - pose.headingRad)) > aim.headingToleranceRad
        || !hood.atTarget(feedStopHoodTicks()));
    updateAimState(aiming, aim, ready);

    updateDrive(aiming, aim, lockRateRadPerSec, pose, velocity[2]);
    // Without the Cell in view there is nothing to aim at, so RT shoots at full flywheel speed and the
    // held hood once the wheels are up to speed, instead of holding the feeder backward forever. It then keeps
    // feeding until RT is released: each ball drops the wheels' RPM, and reversing the feeder every
    // time they dip would pull the next ball back out.
    boolean manualReady = manualFeedGate.update(shootAll && aim == null && !shooter.isStalled(),
        shooter.atSpeed(ShooterTuning.MANUAL_RPM_TOLERANCE));
    // One ball also without the Cell in view, at full speed, so a press never empties the robot
    boolean oneBallReady = aim != null ? ready
        : shooter.atSpeed(ShooterTuning.MANUAL_RPM_TOLERANCE) && !shooter.isStalled();
    boolean aimedFeed = aimedFeedGate.update(clock.seconds(), (shootAll && aim != null) || oneBall,
        oneBall ? oneBallReady : ready, shooter.getRpm(), shooter.getTargetRpm(), oneBall, shotLost);
    updateIntakeFeederIndexer(aiming, aimedFeed || manualReady);

    long tRun = System.nanoTime();
    CommandScheduler.getInstance().run(); // After the states are set, so subsystems act on and log this loop's
    stampPoseHistory(pose); // After the run, which is when vision takes this loop's capture

    if (shooter.isStalled() && !shooterWasStalled) {
      physicalDriver.rumble(STALL_RUMBLE_MS);
    }
    shooterWasStalled = shooter.isStalled();

    long tLog = System.nanoTime();
    updateBattery();
    logAim(aim);
    drawField(pose, aim);
    addTelemetry(aim, pose);
    telemetry.update();
    long tEnd = System.nanoTime();
    Logger.recordOutput("Loop/StartMs", (tVision - t0) * 1e-6);
    Logger.recordOutput("Loop/VisionAndControlMs", (tRun - tVision) * 1e-6);
    Logger.recordOutput("Loop/SubsystemsMs", (tLog - tRun) * 1e-6);
    Logger.recordOutput("Loop/TelemetryMs", (tEnd - tLog) * 1e-6);
  }

  /**
   * Corrects the pose from the Hive tags, and moves the tracker's stored Cell and poses by the same
   * amount so the aim does not jump. Returns the pose to use this loop.
   */
  private Pose correctPoseFromVision(Pose pose, double[] velocity, long nowNanos) {
    List<TagObservation> observations = vision.takeNewTagObservations();
    VisionResult result = visionCorrector.process(observations, poseHistory, pose,
        velocity[0], velocity[1], velocity[2], poseValid, headingValid, nowNanos);
    if (result.accepted) {
      odometry.setPose(result.newPose.xM, result.newPose.yM, result.newPose.headingRad);
      // Turn about the robot by the heading fix, then shift, so stored poses and the Cell stay put relative to it
      double dHeading = result.appliedDHeadingRad();
      headingHold.shift(dHeading);
      poseHistory.transform(pose.xM, pose.yM, dHeading, result.appliedDxM(), result.appliedDyM());
      tracker.transform(pose.xM, pose.yM, dHeading, result.appliedDxM(), result.appliedDyM());
      poseValid |= result.gain > 0; // The first position fix is applied in full, so after it the pose is on the field
      headingValid |= result.headingGain > 0; // Likewise the first heading fix
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

  /**
   * Null when the Cell hasn't been seen recently. While aiming, the shot direction (hooded or backward)
   * is picked once, as the one needing less turn, and kept until RT is let go: their headings are 180
   * degrees apart, so switching between them each loop spins the robot back and forth. Backward is picked
   * whenever the distance suits it, even while moving too fast for it: the robot then lines up and the hood
   * goes down, and it fires once slowed. Picking only a ready backward shot turned the robot 180 degrees to
   * the hooded one whenever the driver was still moving at the press (5 Nov, 092433).
   */
  private Aim aimAtCell(Pose pose, double[] velocity, long nowNanos, boolean aiming) {
    tracker.recordPose(nowNanos, pose);
    // Spinning fast, the frame's pose is too uncertain and moved the Cell 0.17 m during one shot (5 Nov)
    boolean steady = Math.abs(velocity[2]) <= ShootOnTheMoveConstants.MAX_OMEGA_FOR_VISION_RAD_S;
    for (CellSighting sighting : vision.takeNewCellSightings()) {
      if (alliance.ownsCluster(sighting.clusterName)) {
        if (steady) {
          tracker.update(sighting);
        }
      }
    }
    cellSeenAgoS = tracker.secondsSinceSeen(nowNanos);
    // Once the tags have placed the robot and shown which of our Cells is up, aim at that Cell's place on the
    // field. The tracker's Cell, last seen 6-20 s before, had been dragged 0.38 m toward the down Cell by the
    // pose corrections it follows, and the balls went into the down Cell (5 Nov, 081558)
    aimingAtFieldCell = ShootOnTheMoveConstants.AIM_AT_FIELD_CELL && poseValid
        && visionCorrector.upCellConfirmed(alliance);
    double cellX;
    double cellY;
    if (aimingAtFieldCell) {
      double[] point = VisionCorrector.upCellAimPoint(alliance, visionCorrector.upCell(alliance));
      cellX = point[0];
      cellY = point[1];
    } else if (tracker.knowsCell(nowNanos)) {
      cellX = tracker.getCellX();
      cellY = tracker.getCellY();
    } else {
      return null;
    }
    aimCellX = cellX;
    aimCellY = cellY;
    Aim hooded = ShootOnTheMove.aimHooded(pose, velocity[0], velocity[1], cellX, cellY);
    Aim backward = ShootOnTheMove.aimBackward(pose, velocity[0], velocity[1], cellX, cellY);
    boolean pickBackward = ShootOnTheMoveConstants.BACKWARD_SHOTS_ON
        && backward.shot.reason != Shot.Reason.OUT_OF_RANGE
        && (turnTo(backward, pose) < turnTo(hooded, pose) || hooded.shot.reason == Shot.Reason.OUT_OF_RANGE);
    if (!aiming) {
      aimBackwardLatched = null;
      // The shot RT would pick, so the hood can get there first. Only switched once the other one has
      // been the pick for PREPOSITION_SETTLE_S, so turning past the switch point doesn't swing the hood
      // through its whole travel each time (the belt skipped doing that, 4 Nov).
      double nowS = nowNanos * 1e-9;
      if (pickBackward == prepositionBackward) {
        prepositionSwitchSinceS = Double.NaN;
      } else if (Double.isNaN(prepositionSwitchSinceS)) {
        prepositionSwitchSinceS = nowS;
      } else if (nowS - prepositionSwitchSinceS >= PREPOSITION_SETTLE_S) {
        prepositionBackward = pickBackward;
        prepositionSwitchSinceS = Double.NaN;
      }
      return prepositionBackward ? backward : hooded;
    }
    if (aimBackwardLatched == null) {
      aimBackwardLatched = pickBackward;
      backwardBlockedSinceS = Double.NaN;
    }
    if (aimBackwardLatched) {
      double nowS = nowNanos * 1e-9;
      if (backward.shot.valid) {
        backwardBlockedSinceS = Double.NaN;
      } else if (Double.isNaN(backwardBlockedSinceS)) {
        backwardBlockedSinceS = nowS;
      } else if (nowS - backwardBlockedSinceS >= ShootOnTheMoveConstants.BACKWARD_GIVE_UP_S && hooded.shot.valid) {
        aimBackwardLatched = false; // For the rest of this press, so it doesn't turn back and forth
      }
    }
    return aimBackwardLatched ? backward : hooded;
  }

  private static double turnTo(Aim aim, Pose pose) {
    return Math.abs(AngleUnit.normalizeRadians(aim.targetHeadingRad - pose.headingRad));
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

  /** Left bumper toggles the intake. */
  private void updateToggles() {
    if (justPressed(driver.left_bumper, wasLeftBumper)) {
      intakeOn = !intakeOn;
    }
    wasLeftBumper = driver.left_bumper;
  }

  private static boolean justPressed(boolean pressedNow, boolean pressedLastLoop) {
    return pressedNow && !pressedLastLoop;
  }

  /** The stick directions are from the driver's wall, so the drive gets the heading less the alliance's forward. */
  /**
   * Rate the aimed lock heading moves, CCW positive, for the heading lock's feedforward. 0 when not aiming.
   * The direction to the Cell's rate comes from the velocity; the shooter's sideways turn changes with the
   * acceleration, so its rate is taken loop to loop and smoothed.
   */
  private double updateLockRate(boolean aiming, Aim aim) {
    double nowS = clock.seconds();
    if (!aiming || aim == null || aim.backward != lastTurningBackward) {
      turningRateRadPerSec = 0.0;
      lastTurningRad = Double.NaN;
    } else if (!Double.isNaN(lastTurningRad) && nowS - lastTurningS > 1e-3) {
      double rate = (aim.shot.turningRad - lastTurningRad) / (nowS - lastTurningS);
      rate = Math.max(-MAX_LOCK_RATE_RAD_S, Math.min(MAX_LOCK_RATE_RAD_S, rate));
      turningRateRadPerSec += TURNING_RATE_SMOOTHING * (rate - turningRateRadPerSec);
    }
    if (aiming && aim != null) {
      lastTurningRad = aim.shot.turningRad;
      lastTurningS = nowS;
      lastTurningBackward = aim.backward;
    }
    double lockRateRadPerSec = !aiming || aim == null ? 0.0
        : Math.max(-MAX_LOCK_RATE_RAD_S, Math.min(MAX_LOCK_RATE_RAD_S, aim.lockRateRadPerSec + turningRateRadPerSec));
    Logger.recordOutput("Drive/HeadingLock/TargetRateRadPerSec", lockRateRadPerSec);
    Logger.recordOutput("SOTM/TurningRateRadPerSec", turningRateRadPerSec);
    return lockRateRadPerSec;
  }

  private void updateDrive(boolean aiming, Aim aim, double lockRateRadPerSec, Pose pose, double turnRateRadPerSec) {
    // Slower while shooting, so the shot's lead for the robot's speed stays small and the aim keeps up
    double maxPower = aiming ? ShootOnTheMoveConstants.AIM_DRIVE_MAX_POWER : 1.0;
    double[] translation = MecanumDriveSubsystem.shapeTranslation(-driver.left_stick_y, driver.left_stick_x, maxPower);
    double forward = translation[0];
    double right = translation[1];
    double forwardOffsetRad = alliance.forwardHeadingRad();
    double driverHeadingRad = pose.headingRad - forwardOffsetRad;
    if (aiming && aim != null) { // Lines up even without a valid shot, so it's ready the moment one appears
      // Target and heading both less the offset, so the error is the same and the driving is driver relative
      drive.driveHeadingLock(forward, right, aim.lockHeadingRad - forwardOffsetRad, lockRateRadPerSec,
          driverHeadingRad, turnRateRadPerSec);
      headingHold.clear();
    } else {
      double turn = MecanumDriveSubsystem.shapeTurnStick(driver.right_stick_x);
      double holdRad = headingHold.update(turn != 0.0, forward != 0.0 || right != 0.0, pose.headingRad,
          turnRateRadPerSec, clock.seconds());
      if (Double.isNaN(holdRad)) {
        drive.drive(forward, right, turn, true, driverHeadingRad);
      } else {
        drive.driveHeadingHold(forward, right, holdRad - forwardOffsetRad, driverHeadingRad, turnRateRadPerSec);
      }
    }
    Logger.recordOutput("Drive/HeadingHold/TargetRad", headingHold.getTargetRad());
  }

  private void updateShooterAndHood(boolean aiming, Aim aim) {
    boolean validShot = aim != null && aim.shot.valid;
    // A backward shot waiting only for the robot to slow down: hood down and wheels at its speed already
    boolean backwardWaiting = aim != null && aim.backward && !validShot
        && aim.shot.reason != Shot.Reason.OUT_OF_RANGE;
    boolean shotKnown = validShot || backwardWaiting;
    if (validShot) {
      aimRpm = aim.shot.targetRpm;
    } else if (backwardWaiting) {
      aimRpm = ShootOnTheMove.calculateBackward(aim.shot.distanceToLipM, 0, 0).targetRpm;
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
    } else if (driver.x) {
      shooter.setShooterState(ShooterState.STOP);
    } else if (shotKnown && ShootOnTheMoveConstants.PRESPIN_ON) {
      // Already at the speed RT would ask for, so a shot doesn't wait on spin-up (0.8 s from idle, 4 Nov)
      shooter.setShooterState(ShooterState.AIM);
      shooter.setTargetRpm(ShooterTuning.RPM_OVERRIDE > 0 ? ShooterTuning.RPM_OVERRIDE : aimRpm + rpmTrim);
    } else {
      shooter.setShooterState(ShooterState.IDLE);
    }

    // Not aiming but the Cell is known: move the hood to the shot RT would take now, while driving,
    // so it is already there when RT is pressed. Presets are for when the Cell isn't known.
    boolean prepositioning = !aiming && shotKnown && !driver.y && !driver.b;
    if (aiming || prepositioning) {
      if (shotKnown && aim.backward) {
        // Driven down to just above the lower stop and held there. Target 0 cuts the motor to rest on the
        // stop, but the hood doesn't fall by itself: it stayed ~36 degrees up and the shot never got ready.
        hood.setTargetTicks(HoodSubsystem.minTargetTicks()); // Trim doesn't apply
      } else if (validShot) { // Otherwise the hood holds its last angle
        hood.setTargetTicks(HoodSubsystem.angleToTicks(aim.shot.hoodDeg - hoodLiftTrimDeg));
      }
    } else if (driver.dpad_up) {
      hood.setPreset(HoodPreset.FAR);
    } else if (driver.dpad_down) {
      hood.setPreset(HoodPreset.CLOSE);
    }
    double lift = hood.getTargetLiftDegrees();
    double steppedLift = hoodSteps.update(lift, driver.dpad_right, driver.dpad_left,
        HoodSubsystem.ZERO_ANGLE_DEG - HoodSubsystem.MIN_ANGLE_DEG);
    if (steppedLift != lift) {
      if (aiming || prepositioning) { hoodLiftTrimDeg += steppedLift - lift; }
      hood.setTargetLiftDegrees(steppedLift);
    }
  }

  /**
   * Shooting runs the indexer and feeder into the shooter with the intake off. An aimed shot holds the
   * feeder backward and stops the indexer until READY, so no ball is pushed through early. Loading runs the intake and
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
      // While an aimed shot waits, the indexer stops too: pushing the stack against the held feeder forced
      // a ball through before the aim was ready (4 Nov, 084004: 18 deg off, overshot)
      boolean feeding = !aiming || ready;
      intake.setIntakeState(IntakeState.STOP);
      indexer.setIndexerState(feeding ? IndexerState.FEED : IndexerState.STOP);
      feeder.setFeederState(feeding ? FeederState.FEED : FeederState.HOLD);
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

  private static int feedStopHoodTicks() {
    return (int) Math.round(ShootOnTheMoveConstants.FEED_STOP_HOOD_DEG * HoodSubsystem.TICKS_PER_DEG);
  }

  private static int hoodToleranceTicks() {
    return (int) Math.round(ShootOnTheMoveConstants.HOOD_TOLERANCE_DEG * HoodSubsystem.TICKS_PER_DEG);
  }

  /**
   * Panels writes to every connected browser on the calling thread, and that write blocks for good once a
   * client stops reading (5 Nov: the OpMode thread hung in field.update(), the robot stopped responding
   * and the app restarted from "stuck in stop()"). So the field is sent from this thread instead.
   */
  private final ExecutorService fieldSender = Executors.newSingleThreadExecutor(r -> {
    Thread t = new Thread(r, "PanelsField");
    t.setDaemon(true);
    return t;
  });
  private final AtomicBoolean fieldBusy = new AtomicBoolean();
  private int fieldSkips;

  /**
   * The robot, where it points, and the Cell it is aiming at, on the Panels Field tab. Inches, field
   * centre at the origin, x toward the audience Cells. Panels sends it at most every 100 ms. Off with
   * Main.PANELS_ENABLED.
   */
  private void drawField(Pose pose, Aim aim) {
    if (!Main.PANELS_ENABLED) {
      return;
    }
    Logger.recordOutput("Panels/FieldSkips", fieldSkips);
    // A send still blocked on a client that stopped reading: skip this frame rather than queue behind it
    if (!fieldBusy.compareAndSet(false, true)) {
      fieldSkips++;
      return;
    }
    double x = pose.xM / ShootOnTheMoveConstants.IN_TO_M;
    double y = pose.yM / ShootOnTheMoveConstants.IN_TO_M;
    double heading = pose.headingRad;
    String colour = alliance == Alliance.RED ? "red" : "blue";
    boolean aiming = aim != null;
    double cellX = aimCellX / ShootOnTheMoveConstants.IN_TO_M;
    double cellY = aimCellY / ShootOnTheMoveConstants.IN_TO_M;
    fieldSender.execute(() -> {
      try {
        FieldManager field = PanelsField.INSTANCE.getField();
        field.setOffsets(PanelsField.INSTANCE.getPresets().getDEFAULT_FTC());
        if (aiming) {
          field.setStyle("transparent", "yellow", 0.5);
          field.moveCursor(x, y);
          field.line(cellX, cellY);
          field.setStyle("yellow", "black", 0.5);
          field.moveCursor(cellX, cellY);
          field.circle(3);
        }
        field.setStyle("transparent", colour, 1.5); // Robot: 18 in square, drawn as its 9 in circle
        field.moveCursor(x, y);
        field.circle(9);
        field.line(x + 12 * Math.cos(heading), y + 12 * Math.sin(heading)); // Front
        field.update();
      } finally {
        fieldBusy.set(false);
      }
    });
  }

  /** Everything shoot on the move worked out this loop, for AdvantageScope. */
  private void logAim(Aim aim) {
    Logger.recordOutput("SOTM/State", aimState.name());
    Logger.recordOutput("SOTM/Alliance", alliance.name());
    Logger.recordOutput("SOTM/KnowsCell", aim != null);
    Logger.recordOutput("SOTM/CellX", aimCellX);
    Logger.recordOutput("SOTM/CellY", aimCellY);
    Logger.recordOutput("SOTM/AimingAtFieldCell", aimingAtFieldCell);
    Logger.recordOutput("SOTM/UpCell", visionCorrector.upCell(alliance).name());
    Logger.recordOutput("SOTM/TrackerCellX", tracker.getCellX()); // To compare with the field Cell
    Logger.recordOutput("SOTM/TrackerCellY", tracker.getCellY());
    // For AdvantageScope's field views: a line from the robot to the Cell it is aiming at, none when unknown
    Logger.recordOutput("SOTM/Cell", aim == null ? new Pose2d[0]
        : new Pose2d[] {new Pose2d(aimCellX, aimCellY, new Rotation2d())});
    Logger.recordOutput("SOTM/CellSeenAgoS", cellSeenAgoS);
    Logger.recordOutput("SOTM/RpmTrim", rpmTrim);
    Logger.recordOutput("SOTM/HoodLiftTrimDeg", hoodLiftTrimDeg);
    Logger.recordOutput("Robot/OneBallShot", driver.right_bumper && driver.right_trigger <= TRIGGER_PRESSED);
    Logger.recordOutput("Robot/IntakeOn", intakeOn);
    if (aim == null) {
      return;
    }
    Logger.recordOutput("SOTM/Valid", aim.shot.valid);
    Logger.recordOutput("SOTM/Backward", aim.backward);
    Logger.recordOutput("SOTM/Reason", aim.shot.reason.name());
    Logger.recordOutput("SOTM/DistanceToLipM", aim.shot.distanceToLipM);
    Logger.recordOutput("SOTM/SpeedTowardCellMps", aim.shot.speedTowardCellMps);
    Logger.recordOutput("SOTM/SidewaysSpeedMps", aim.shot.sidewaysSpeedMps);
    Logger.recordOutput("SOTM/TargetRpm", aim.shot.targetRpm);
    Logger.recordOutput("SOTM/HoodDeg", aim.shot.hoodDeg);
    Logger.recordOutput("SOTM/TimeOfFlightS", aim.shot.timeOfFlightS);
    Logger.recordOutput("SOTM/TurningRad", aim.shot.turningRad);
    Logger.recordOutput("SOTM/TargetHeadingRad", aim.targetHeadingRad);
    Logger.recordOutput("SOTM/LockHeadingRad", aim.lockHeadingRad);
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
    Logger.recordOutput("Vision/Correction/HeadingValid", headingValid);
    Logger.recordOutput("Vision/Correction/DHeadingDeg", Math.toDegrees(result.dHeadingRad));
    Logger.recordOutput("Vision/Correction/HeadingGain", result.headingGain);
    Logger.recordOutput("Vision/Correction/HeadingPairs", result.headingPairs);
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
    telemetry.addData("Alliance", alliance + (poseValid ? "  pose on field" : "  pose not placed yet")
        + (headingValid ? ", heading set" : ", heading from start"));
    telemetry.addData("Aim", aimState);
    if (aim == null) {
      telemetry.addData("Distance", Double.isInfinite(cellSeenAgoS)
          ? "Cell not seen yet: point the cameras at it"
          : String.format("Cell forgotten (seen %.0f s ago): point the cameras at it", cellSeenAgoS));
    } else {
      telemetry.addData("Distance", String.format("%.2f m to lip  %s  (%s)", aim.shot.distanceToLipM,
          aim.backward ? "BACKWARD, hood down" : "FRONT, hooded",
          aimingAtFieldCell ? alliance + " " + visionCorrector.upCell(alliance) + " up, from tags"
              : cellSeenAgoS < 0.5 ? "camera sees Cell" : String.format("odometry, seen %.1f s ago", cellSeenAgoS)));
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
    telemetry.addData("Hood lift (deg)", "target %.1f, actual %.1f", hood.getTargetLiftDegrees(), hood.getLiftDegrees());
    telemetry.addData("Shooter", shooter.getShooterState());
    telemetry.addData("Shooter stall", shooter.getStallStatus());
    telemetry.addData("Hood encoders", hood.getEncoderFaultText());
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
      hood.setBatteryVolts(batteryVolts);
      batteryMinVolts = Double.isNaN(batteryMinVolts)
          ? batteryVolts : Math.min(batteryMinVolts, batteryVolts);
    }
    Logger.recordOutput("Robot/BatteryVolts", batteryVolts);
    Logger.recordOutput("Robot/BatteryMinVolts", batteryMinVolts);
    telemetry.addData("Battery", "%.2f V (lowest %.2f V)", batteryVolts, batteryMinVolts);
  }

  /** Called once when the OpMode stops. */
  public void stop() {
    fieldSender.shutdownNow();
    hood.disable(); // Its loop thread would otherwise hold the hood until it misses the heartbeat
    vision.close();
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().reset();
  }
}
