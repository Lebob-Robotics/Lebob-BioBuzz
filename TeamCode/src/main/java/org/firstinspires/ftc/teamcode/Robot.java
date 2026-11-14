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
import org.firstinspires.ftc.teamcode.subsystems.RampSubsystem;
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
import org.firstinspires.ftc.teamcode.utils.PoseHandoff;
import org.firstinspires.ftc.teamcode.utils.StackPressure;
import org.firstinspires.ftc.teamcode.utils.WheelContact;
import org.firstinspires.ftc.teamcode.utils.LoadingShooterInterlock;
import org.firstinspires.ftc.teamcode.utils.HoodStepControl;
import org.firstinspires.ftc.teamcode.utils.HeadingHold;
import org.firstinspires.ftc.teamcode.utils.LoopWatchdog;
import org.firstinspires.ftc.teamcode.utils.CellSighting;
import org.firstinspires.ftc.teamcode.utils.Pose;
import org.firstinspires.ftc.teamcode.utils.PoseHistory;
import org.firstinspires.ftc.teamcode.utils.RpmTrend;
import org.firstinspires.ftc.teamcode.utils.OneBallFeedGate;
import org.firstinspires.ftc.teamcode.utils.IndexerRetry;
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
 *   <li>Right bumper held: as RT, but the robot holds the spot it was on when RB was pressed: the left stick
 *       is ignored, and a push is driven back. It still turns to the Cell. (Shot Tuning: one ball per press.)
 *       The flywheels already track the shot's RPM whenever the Cell is known, so there's no spin-up toggle.</li>
 *   <li>Y held: manual shot at full power, without aiming or waiting. A fallback for when the camera
 *       can't see the Cell.</li>
 *   <li>Intake runs continuously, reversing when full. The feeder runs backward to queue the
 *       balls short of the shooter. At 4 balls the indexer stops and the intake rejects extras.</li>
 *   <li>B: eject. X: stop the flywheels, and keep them stopped even with the Cell known until RT, RB, Y or
 *       B (also resets a stall trip). A: tell the robot
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
  public final RampSubsystem ramp;
  public final IndexerSubsystem indexer;
  public final HoodSubsystem hood;
  public final VisionSubsystem vision;

  public static final double TRIGGER_PRESSED = 0.5;
  private boolean holdFire;
  private boolean spinOnlyOnFire;
  private final LoadingShooterInterlock loadingShooter = new LoadingShooterInterlock();
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

  private Alliance alliance = Alliance.BLUE;
  private boolean driverOnFarSide = false;
  private final TargetTracker tracker = new TargetTracker(ShootOnTheMoveConstants.POSE_HISTORY_SIZE);
  // Vision pose correction. The history holds each loop's odometry pose with the time of that loop's
  // camera capture, so a frame can be matched to where the robot was. It is separate from the tracker's.
  private final PoseHistory poseHistory = new PoseHistory(ShootOnTheMoveConstants.POSE_HISTORY_SIZE);
  private final VisionCorrector visionCorrector = new VisionCorrector();
  private long lastHistoryNanos = Long.MIN_VALUE;
  private boolean poseValid = false; // True once a tag fix has placed the robot on the field. Nothing is gated on it yet
  // True once two tags have set the heading, or the driver reset it with A. Until then a heading fix may be any size
  private boolean headingValid = false;
  private VisionCorrector.Cell frontCell = null; // Our Cell on the robot's side of the Hive
  private boolean poseSeeded = false; // Shoot Auto's assumed start pose: aimed from until vision places the robot
  private VisionResult lastVision = null; // Last update that looked at tags, for telemetry
  private long lastVisionNanos = 0;
  private int outOfFieldFrames; // Tag fixes in a row that put the robot off the field: the pose has gone wrong
  private int headingOffFrames; // Tag-pair headings in a row this far from the trusted heading
  private int positionFixes; // Tag fixes applied to the position so far
  private double lastHeadingOffRad;
  private static final double HEADING_OFF_RAD = Math.toRadians(15);

  private AimState aimState = AimState.OFF;
  private double aimRpm = ShootOnTheMoveConstants.AIM_START_RPM; // Last valid shot's, so a brief invalid shot doesn't drop it
  private final HoodStepControl hoodSteps = new HoodStepControl();
  private double hoodLiftTrimDeg = 0;
  private double rpmTrim = 0; // Driver's, for the rest of the match
  private double headingTrimDeg = 0; // Anticlockwise, the tuning tools' aim trim (TeleOp uses aimTrimDeg's)
  // Shot Tuning: the driver's RPM, hood and direction replace the tables', and the buttons they use do nothing else
  private boolean tuningMode = false;
  private static final double TUNING_HEADING_TOLERANCE_DEG = 1.5;
  private boolean tuningBackward = false;
  private double tuningRpm = 3400;
  private double tuningHoodDeg = HoodSubsystem.ZERO_ANGLE_DEG;
  private boolean tuningLive = false; // Shot Tuning's moving pass: the tables' shot, moving corrections and all
  private boolean steadyGateBypassed = false; // SOTM Tuning's passes that fire mid-change
  private boolean closeMovingGateBypassed = false; // SOTM Tuning's passes that fire moving close to the lip
  private double[] steadyStickReference; // SOTM Tuning: the plan's stick size, without its tracking corrections
  private boolean scriptedOneBall = false; // As RB held in Shot Tuning, from a script
  private boolean scriptedVolley = false; // As RB held in Lebot Teleop, from a script (autonomous)
  private Aim lastAim = null;
  private double[] driveOverride = null; // Field-frame stick {+x, +y} for a scripted drive; null for the driver's
  private boolean forceAim = false; // Aims as RB does, without firing
  private double[] aimPointOverride = null; // Field {x, y} aimed at instead of the Cell; needs the tags' pose
  private double holdAnchorX = Double.NaN; // Where RB holds the robot; NaN while not holding
  private double holdAnchorY = Double.NaN;
  private double[] holdPoint = null; // A scripted field spot held while no drive is scripted; null for none
  private boolean sticksLocked = false; // The drive and turn sticks do nothing
  private double turnOverride = 0; // Turn stick for a scripted turn while the sticks are locked
  private boolean allianceFromTags = false; // Takes the alliance of the Cell seen while its own is unknown
  // RB's position hold: field-frame power toward the anchor, less a share of the speed to damp it
  private static final double HOLD_DEADBAND_M = 0.02;
  private static final double HOLD_GAIN_PER_M = 2.5;
  private static final double HOLD_MIN_POWER = 0.08; // Gets the wheels past their friction
  private static final double HOLD_MAX_POWER = 0.4;
  private static final double HOLD_DAMPING_S = 0.3;
  // A scripted hold point rests the wheels near it: holding there, too weak to move the robot, stalled them with
  // a whine. Pushed past RETURN it drives back to DONE, its least power rising until the robot moves
  private static final double HOLD_POINT_RETURN_M = 0.04;
  private static final double HOLD_POINT_DONE_M = 0.015;
  private static final double HOLD_POINT_BOOST_PER_S = 0.3;
  private static final double HOLD_POINT_MOVING_MPS = 0.05;
  private boolean holdPointReturning = false;
  private double holdPointStuckSinceS;
  private final java.util.List<String[]> panelsLines = new java.util.ArrayList<>();
  private final AtomicBoolean panelsBusy = new AtomicBoolean();
  private double lastPanelsSendS = Double.NEGATIVE_INFINITY;
  private static final double PANELS_TELEMETRY_PERIOD_S = 0.1;
  private static final double DS_TELEMETRY_PERIOD_S = 0.1;
  private double lastTelemetryS = Double.NEGATIVE_INFINITY;
  private final IndexerRetry indexerRetry = new IndexerRetry();
  private final ManualFeedGate manualFeedGate = new ManualFeedGate();
  private final LoopWatchdog watchdog = new LoopWatchdog();
  // An aimed shot feeds one ball at a time, each once the wheels are back up to speed
  private final OneBallFeedGate aimedFeedGate = new OneBallFeedGate(
      ShootOnTheMoveConstants.AIMED_READY_S, ShootOnTheMoveConstants.AIMED_FEED_STOP_UNREADY_S,
      ShootOnTheMoveConstants.AIMED_FEED_PULSE_S, ShootOnTheMoveConstants.AIMED_FEED_PAUSE_S,
      ShootOnTheMoveConstants.MIN_BALL_GAP_S);
  // Volley mode (Lebot Teleop, Volley Tuning): RB's hooded aimed shot, held still, fires as a burst, each ball at
  // the matched wheel speed (ShootOnTheMoveConstants VOLLEY_*). RT stays the one-at-a-time shoot on the move. Off,
  // RB is a plain held shot as RT's
  private boolean volleyMode = false;
  private boolean volleyRequested = false; // This loop asks for a volley: RB, Volley Tuning's aim, or RT (VOLLEY_ON_RT)
  private boolean hoodedOnly = false;
  private double volleyHeadingTrimDeg = Double.NaN;
  private final OneBallFeedGate volleyFeedGate = new OneBallFeedGate(0.0,
      ShootOnTheMoveConstants.AIMED_FEED_STOP_UNREADY_S, ShootOnTheMoveConstants.VOLLEY_FEED_PULSE_S,
      ShootOnTheMoveConstants.VOLLEY_FEED_PAUSE_S,
      ShootOnTheMoveConstants.VOLLEY_MIN_GAP_S, true);
  private OneBallFeedGate feedGate = aimedFeedGate; // The one feeding this loop
  private boolean volleyShot = false; // This loop's shot is a volley
  // Volley Tuning's settings in place of the constants; NaN (null) for the constant
  private double volleyFirstTrimDeg = Double.NaN;
  private double volleyLaterTrimDeg = Double.NaN;
  private double volleyIndexerPower = Double.NaN;
  private double volleyFeedPower = Double.NaN;
  private double volleyBoostRpm = Double.NaN;
  private Boolean volleyFastRecovery = null;
  private double volleyHoodDeg = 0; // Launch angle the volley added this loop, for the log
  // Volley Tuning marks balls with A, B, X, Y and the D-pad, so they do nothing else (heading reset, eject, stop,
  // full power, trims, presets). The intake and the triggers work as in Lebot Teleop
  private boolean buttonsReserved = false;
  private double cellSeenAgoS = Double.POSITIVE_INFINITY;
  private boolean aimingAtFieldCell = false; // At the up Cell's field position, not the tracker's sighted one
  private double aimCellX = 0; // The point aimed at this loop, field metres
  private double aimCellY = 0;
  private boolean intakeOn = true;
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
  private boolean shooterStopLatched = false; // X: flywheels off until the driver next asks for a shot
  private boolean wasFull = false;
  private final WheelContact wheelContact = new WheelContact();
  private boolean wheelContactFull = false; // A loading ball touched the wheels: the stack is full
  private final StackPressure stackPressure = new StackPressure();
  private boolean stackPacked = false; // The intake pushed a packed stack: it eases off and the last roller stops
  private boolean intakeEased = false; // The intake at PACKED_INTAKE_POWER; clears sooner than stackPacked
  private boolean wasLtPressed = false;
  private double ltReleasedAtS = Double.NEGATIVE_INFINITY;
  private boolean handsOffPose = false;
  private double initMaxVolts = Double.NaN; // Highest in INIT: at rest, before the mechanisms load it
  private boolean swapBatteryRumbled = false;
  private String stopHint; // What the driver must change for the shot to fire; null when it's not how they drive
  private double stopHintSinceS = Double.NaN;
  private boolean stopHintRumbled = false;
  private static final double STOP_HINT_RUMBLE_AFTER_S = 0.4;
  private static final int STOP_HINT_RUMBLE_MS = 400;
  private boolean startedFromAuto = false;
  private double lastShotKnownS = Double.NEGATIVE_INFINITY;
  private final double[] hiveTagSeenS = {Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY}; // By Alliance ordinal
  private boolean wasDpadUp = false;
  private boolean wasDpadDown = false;
  private boolean wasRampButton = false;
  private boolean wasLeftBumper = false;
  private boolean shooterWasStalled = false;
  private double intakeLockRumbleS = Double.NaN; // Last rumble for a tripped intake; NaN while it runs
  static final int INTAKE_LOCK_RUMBLE_MS = 1000;
  static final int INTAKE_LOCK_REMIND_MS = 400;
  static final double INTAKE_LOCK_REMIND_S = 2.0;
  private final HeadingHold headingHold = new HeadingHold();
  // This loop's results, for an OpMode that drives the robot itself (Shot Tuning)
  private Pose lastPose = new Pose(0, 0, 0);
  private double[] lastVelocity = {0, 0, 0};
  private double[] aimVelocity; // Low-passed {x, y} the aim uses; null until the first loop
  private double aimVelocityS;
  private boolean lastFeeding = false;
  private double feedCommitUntilS = Double.NEGATIVE_INFINITY;
  private double[] committedTranslation; // Held while a fed ball is still in the shooter; null otherwise
  private double committedSinceS;
  private double[] lastTranslation = {0.0, 0.0};
  // Scoring Auto (6 Nov, feature/auto): no gamepad is read (G401); it drives, aims and intakes through these
  private boolean autonomous = false;
  private final Gamepad noGamepad = new Gamepad();
  private boolean autoShoot = false;
  private boolean autoIntake = false;
  private AutoDriver autoDriver = null;
  /** A heading the auto wants held with the heading lock instead of Pedro's turn, or NaN. */
  private double autoHeadingRad = Double.NaN;
  private double autoHeadingRateRadPerSec = 0.0;
  private double[] steadyStick = {0.0, 0.0};
  private double steadyStickSinceS = Double.NEGATIVE_INFINITY;
  private final java.util.ArrayDeque<double[]> velocityHistory = new java.util.ArrayDeque<>(); // {t, x, y}

  private boolean mechanismsWaitForStart;

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
    ramp = new RampSubsystem(hardwareMap);
    hood = new HoodSubsystem(hardwareMap);
    vision = new VisionSubsystem(hardwareMap);

    // Both hubs read the same battery, so one sensor is enough.
    battery = hardwareMap.voltageSensor.iterator().hasNext()
        ? hardwareMap.voltageSensor.iterator().next() : null;
  }

  /** Called once when the OpMode enters INIT. */
  public void init() {
    init(false);
  }

  /**
   * With {@code stillInInit}, nothing moves until START: the hood homes and the ramp servo is powered then, not
   * in INIT. G403 bans any powered movement, a twitch included, in the 8 s between AUTO and TELEOP, and Lebot
   * Teleop is INITed then: its INIT drove the hood down from the auto's ~61 degrees.
   */
  public void init(boolean stillInInit) {
    if (!Logger.isReplay()) {
      watchdog.start();
    }
    odometry.init();
    mechanismsWaitForStart = stillInInit;
    if (!stillInInit) {
      startMechanisms();
    }
    // Off during INIT; the teleop loop idles the flywheels once the OpMode starts.
    shooter.setShooterState(ShooterState.STOP);
  }

  /** Homes the hood and powers the ramp stowed (an unpowered servo lets the ramp flop). */
  private void startMechanisms() {
    mechanismsWaitForStart = false;
    hood.init();
    ramp.setDeployed(false);
    ramp.periodic();
  }

  /** True while INIT keeps the hood and ramp still for START. */
  public boolean mechanismsWaitForStart() {
    return mechanismsWaitForStart;
  }

  /**
   * Merges the Panels browser gamepad into gamepad 1 so either can drive. Panels builds a new
   * combined Gamepad on every call, so this runs once per loop. Off with Main.PANELS_ENABLED.
   */
  private void refreshDriver() {
    driver = autonomous ? noGamepad
        : Main.PANELS_ENABLED
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
    watchdog.feed();
    refreshDriver();
    pollAlliance();
    shooter.periodic();
    if (!mechanismsWaitForStart) {
      hood.periodic(); // Drives the hood down onto its stop and zeroes it there; nothing else moves
    }
    vision.periodic(); // Read and log tags in INIT without running any mechanism commands.
    updateBattery(true);
    if (!Double.isNaN(batteryVolts)) {
      initMaxVolts = Double.isNaN(initMaxVolts) ? batteryVolts : Math.max(initMaxVolts, batteryVolts);
    }
    if (swapBattery(initMaxVolts)) {
      telemetry.addLine(String.format("!! SWAP BATTERY: %.1f V at rest, want %.1f or more",
          initMaxVolts, ShootOnTheMoveConstants.SWAP_BATTERY_V));
      if (!swapBatteryRumbled) {
        physicalDriver.rumble(600);
        swapBatteryRumbled = true;
      }
    }
    noteHiveTags(vision.takeNewTagObservations());
    String allianceWarning = allianceWarning();
    if (allianceWarning != null) {
      telemetry.addLine(allianceWarning);
    }
    telemetry.addData("Alliance", alliance + "  (X = BLUE, B = RED)");
    telemetry.addData("Shooter stall", shooter.getStallStatus());
    telemetry.addData("Hood", mechanismsWaitForStart ? "homes at START (nothing moves in INIT)" : hood.getHomeResult());
    telemetry.addData("Hood encoders", hood.getEncoderFaultText());
    telemetry.addData("Camera L", vision.describe(VisionSubsystem.Side.LEFT));
    telemetry.addData("Camera R", vision.describe(VisionSubsystem.Side.RIGHT));
    telemetry.update();
  }

  /**
   * Called once when the OpMode starts. Sets the heading from {@link #placedHeadingRad}: the robot placed square,
   * its camera side toward the middle of the field. From then on two Hive tags seen together correct it
   * automatically; A resets it by hand the same way.
   */
  public void start() {
    if (mechanismsWaitForStart) {
      startMechanisms();
    }
    pollAlliance();
    odometry.setHeading(placedHeadingRad());
    tracker.clear();
    poseHistory.clear();
    visionCorrector.reset();
    lastHistoryNanos = Long.MIN_VALUE;
    poseValid = false;
    headingValid = false;
    poseSeeded = false;
    lastVision = null;
    ramp.setDeployed(false);
  }

  /**
   * Shoot Auto: the robot is where it was placed, so it can aim at its Hive at once, before any tag is seen. The
   * first tag fix still replaces this in full (poseValid stays false until then). Until the tags show which Cell
   * is up, it aims at the one the corrector starts with.
   */
  public void seedPose(double xM, double yM, double headingRad) {
    odometry.setPose(xM, yM, headingRad);
    poseHistory.clear();
    tracker.clear();
    poseSeeded = true; // headingValid stays false too, so the first heading fix is also taken in full
  }

  /** Placed by the tags, or seeded by {@link #seedPose} with the odometry still good. */
  public boolean isPoseKnown() { return poseValid || poseSeeded; }

  /** For scripted tests: the alliance without the X and B buttons, which also stop and eject the shooter. */
  public void setAlliance(Alliance alliance) {
    this.alliance = alliance;
  }

  public Alliance getAlliance() {
    return alliance;
  }

  /**
   * Field heading of the driver's forward: away from the alliance's wall, or toward it with the driver on the far
   * side. The sticks, the start heading and A all use it.
   */
  public double driverForwardHeadingRad() {
    return alliance.forwardHeadingRad() + (driverOnFarSide ? Math.PI : 0);
  }

  /**
   * Heading of a robot placed square with its camera side (the back) toward the middle, away from the driver, so
   * its front faces the driver.
   */
  private double placedHeadingRad() {
    return AngleUnit.normalizeRadians(driverForwardHeadingRad() + Math.PI);
  }

  /** The driver stands at the far wall from the alliance's, so the sticks turn round with them. */
  public void setDriverOnFarSide(boolean farSide) { driverOnFarSide = farSide; }

  public boolean isDriverOnFarSide() { return driverOnFarSide; }

  private void pollAlliance() {
    if (driver.x) {
      alliance = Alliance.BLUE;
    } else if (driver.b) {
      alliance = Alliance.RED;
    }
  }

  private void noteHiveTags(List<TagObservation> observations) {
    for (TagObservation o : observations) {
      Alliance a = VisionCorrector.tagAlliance(o.id);
      if (a != null) {
        hiveTagSeenS[a.ordinal()] = clock.seconds();
      }
    }
  }

  /** A warning while the cameras see only the other alliance's Hive, else null. */
  public String allianceWarning() {
    Alliance other = alliance == Alliance.RED ? Alliance.BLUE : Alliance.RED;
    double nowS = clock.seconds();
    return allianceWarning(alliance, nowS - hiveTagSeenS[alliance.ordinal()], nowS - hiveTagSeenS[other.ordinal()]);
  }

  /**
   * Shoot Auto set to BLUE saw only RED's Hive tags and never fired: it ignores the other alliance's Hive (13 Nov,
   * 193125). Warns while the other Hive was seen in the last second and ours not in the last three.
   */
  static String allianceWarning(Alliance alliance, double ownSeenAgoS, double otherSeenAgoS) {
    Alliance other = alliance == Alliance.RED ? Alliance.BLUE : Alliance.RED;
    if (otherSeenAgoS <= 1.0 && ownSeenAgoS > 3.0) {
      return "!! CAMERAS SEE " + other + "'S HIVE, NOT " + alliance + "'S. If you are " + other + ", press "
          + (other == Alliance.RED ? "B" : "X");
    }
    return null;
  }

  /** Called repeatedly while the OpMode is running. */
  public void periodic() {
    long t0 = System.nanoTime(); // Section times, logged under Loop/, to find what slows the loop
    watchdog.feed();
    refreshDriver();
    Pose2D pinpointPose = odometry.getPose(); // PsiKit has already updated the Pinpoint this loop
    Pose pose = new Pose(pinpointPose.getX(DistanceUnit.METER), pinpointPose.getY(DistanceUnit.METER),
        pinpointPose.getHeading(AngleUnit.RADIANS));
    double[] velocity = odometry.getVelocityMetersPerSec(); // {x, y, turn rate}
    long nowNanos = vision.getCaptureNanos(); // Logged, so replay gives the same result
    if (odometry.isFaulted()) {
      // The pose is frozen while the robot moves: nothing may aim or drive to it until vision places it again
      poseValid = false;
      headingValid = false;
      poseSeeded = false;
    }

    if (driver.a && buttonsFree()) { // Shot Tuning's A marks a ball
      odometry.setHeading(placedHeadingRad());
      tracker.clear(); // The frame just turned, so the stored Cell and poses no longer apply
      headingValid = true; // The driver says this is right, so vision only trims it from here
      headingHold.clear();
      poseHistory.clear();
      pose = new Pose(pose.xM, pose.yM, placedHeadingRad());
    }

    long tVision = System.nanoTime();
    if (odometry.isReadGood()) { // A correction from a bad read would write that read back into the Pinpoint
      Pose uncorrected = pose;
      pose = correctPoseFromVision(pose, velocity, nowNanos); // Before aiming, so this loop aims from the corrected pose
      // The robot didn't move: only where it thinks it is did, so the held spot moves with it
      holdAnchorX += pose.xM - uncorrected.xM;
      holdAnchorY += pose.yM - uncorrected.yM;
    }

    double fireTrigger = fireTrigger();
    boolean holdShot = driver.right_bumper && !tuningMode;
    volleyRequested = volleyRequested(driver.right_bumper || scriptedVolley, fireTrigger, forceAim,
        tuningMode, ShootOnTheMoveConstants.VOLLEY_ON_RT);
    boolean shootAll = fireTrigger > TRIGGER_PRESSED || holdShot || scriptedVolley;
    boolean oneBallHeld = driver.right_bumper || scriptedOneBall;
    boolean oneBall = tuningMode && oneBallHeld && fireTrigger <= TRIGGER_PRESSED; // RT wins
    boolean aiming = shootAll || oneBallHeld || forceAim;
    if (!holdShot) {
      holdAnchorX = Double.NaN;
      holdAnchorY = Double.NaN;
    } else if (Double.isNaN(holdAnchorX)) {
      holdAnchorX = pose.xM;
      holdAnchorY = pose.yM;
    }
    if (holdPoint != null && driveOverride == null) {
      holdAnchorX = holdPoint[0];
      holdAnchorY = holdPoint[1];
    }
    long tAim = System.nanoTime();
    Aim aim = aimAtCell(pose, velocity, nowNanos, aiming);
    long tAimed = System.nanoTime();
    double turnDeg = aimTrimDeg(tuningMode, headingTrimDeg, aim != null && aim.backward) + movingHeadingDeg(aim);
    if (volleyAim(aim) && !Double.isNaN(volleyHeadingTrimDeg)) { turnDeg += volleyHeadingTrimDeg; }
    if (aim != null && turnDeg != 0) {
      aim = aim.turnedBy(Math.toRadians(turnDeg));
    }
    lastAim = aim;
    double lockRateRadPerSec = updateLockRate(aiming, aim);
    updateTrim(aiming);
    updateToggles(aiming);
    // Before the ready check, so it compares the hood and wheels with this loop's shot. After it, a hood target
    // that had just moved 3 degrees (the robot stopped) was judged against the old one, and the ball missed (5 Nov, 095605)
    long tShooter = System.nanoTime();
    volleyShot = volleyAim(aim);
    feedGate = volleyShot ? volleyFeedGate : aimedFeedGate;
    loadingShooter.update(clock.seconds(), spinOnlyOnFire, shotRequested() && !holdFire,
        shooter.getMaxMeasuredRpm());
    Logger.recordOutput("Shooter/Loading/State", loadingShooter.state());
    Logger.recordOutput("Shooter/Loading/SpinBlocked", loadingShooter.blocksSpin());
    Logger.recordOutput("Shooter/Loading/CanLoad", loadingShooter.canLoad());
    updateShooterAndHood(aiming, aim);
    long tShooterSet = System.nanoTime();
    rpmTrend.update(clock.seconds(), shooter.getRpm());
    double predictedRpmError = rpmTrend.predicted(ShootOnTheMoveConstants.RPM_LOOKAHEAD_S) - shooter.getTargetRpm();
    Logger.recordOutput("Shooter/PredictedRpmError", predictedRpmError);
    boolean flywheelsReady = shooter.atSpeed(ShootOnTheMoveConstants.RPM_TOLERANCE)
        && shooter.meanRpmWithin(ShootOnTheMoveConstants.RPM_MEAN_BELOW_TARGET,
            ShootOnTheMoveConstants.RPM_MEAN_ABOVE_TARGET)
        && predictedRpmError <= ShootOnTheMoveConstants.RPM_PREDICTED_ABOVE_TARGET;
    if (volleyShot) {
      // Every ball at the same speed, so their flights match and none catches the one ahead
      flywheelsReady = shooter.atSpeed(ShootOnTheMoveConstants.RPM_TOLERANCE)
          && shooter.meanRpmWithin(ShootOnTheMoveConstants.VOLLEY_READY_BELOW_RPM, volleyReadyAboveRpm(aim))
          && predictedRpmError >= -ShootOnTheMoveConstants.VOLLEY_READY_BELOW_RPM
          && predictedRpmError <= volleyReadyAboveRpm(aim);
    }
    // A volley's hood stays put between balls, so after the first it needn't wait to settle
    boolean hoodReady = volleyShot && volleyFeedGate.ballsLeft() > 0
        ? hood.atTarget(volleyHoodToleranceTicks()) : hood.atTargetSteady(hoodToleranceTicks());
    boolean motionSteady = updateMotionSteady(aiming, velocity);
    boolean ready = aim != null && motionSteady && ShootOnTheMove.readyToFire(aim, flywheelsReady, hoodReady,
        pose.headingRad, velocity[2], turningRateRadPerSec, closeMovingGateBypassed);
    // What READY waits on, one by one: the shot and heading alone, then the wheels and the hood
    Logger.recordOutput("SOTM/Gate/AimReady", aim != null && ShootOnTheMove.readyToFire(aim, true, true,
        pose.headingRad, velocity[2], turningRateRadPerSec, closeMovingGateBypassed));
    Logger.recordOutput("SOTM/Gate/CloseMoving", aim != null && ShootOnTheMove.tooCloseToShootMoving(aim.shot));
    Logger.recordOutput("SOTM/Gate/CloseMovingBypassed", closeMovingGateBypassed);
    Logger.recordOutput("SOTM/Gate/Flywheels", flywheelsReady);
    Logger.recordOutput("SOTM/Gate/Hood", hoodReady);
    // A feed under way stops if the shot goes bad before the ball is out: the driver set off or turned back
    // mid-feed, and the ball left 3 degrees of hood or 37 degrees of heading off (5 Nov, 095605)
    boolean shotLost = aim != null && (!aim.shot.valid
        || Math.abs(ShootOnTheMove.normalizeAngle(aim.lockHeadingRad - pose.headingRad)) > aim.headingToleranceRad
        || !hood.atTarget(feedStopHoodTicks()));
    updateAimState(aiming, aim, ready);

    long tDrive = System.nanoTime();
    if (autoDriver != null) {
      driveAuto(aiming, aim, lockRateRadPerSec, pose, velocity);
    } else {
      updateDrive(aiming, aim, lockRateRadPerSec, pose, velocity[2]);
    }
    long tDriveSet = System.nanoTime();
    // Without the Cell in view there is nothing to aim at, so RT shoots at full flywheel speed and the
    // held hood once the wheels are up to speed, instead of holding the feeder backward forever. It then keeps
    // feeding until RT is released: each ball drops the wheels' RPM, and reversing the feeder every
    // time they dip would pull the next ball back out.
    // Never in the Scoring Auto: with no Cell there is nothing to aim at, and a blind shot is a wasted ball
    boolean manualReady = manualFeedGate.update(shootAll && aim == null && !tuningMode && !autonomous && !shooter.isStalled()
        && !holdFire && !loadingShooter.blocksSpin(),
        shooter.atSpeed(ShooterTuning.MANUAL_RPM_TOLERANCE));
    // One ball also without the Cell in view, at full speed, so a press never empties the robot
    boolean oneBallReady = aim != null ? ready
        : !tuningMode && shooter.atSpeed(ShooterTuning.MANUAL_RPM_TOLERANCE) && !shooter.isStalled();
    // Held fire counts as released: the gate cycled through its pulses with the feeder held, so the first ball
    // got the last 0.06 s of a pulse and fell short (6 Nov, 081518)
    volleyFeedGate.setLeadBelowRpm(ShootOnTheMoveConstants.VOLLEY_LEAD_BELOW_RPM);
    volleyFeedGate.setMinFeedSpacingS(ShootOnTheMoveConstants.VOLLEY_MIN_FEED_SPACING_S);
    boolean readyButWheels = aim != null && motionSteady && hoodReady && ShootOnTheMove.readyToFire(aim, true,
        hoodReady, pose.headingRad, velocity[2], turningRateRadPerSec, closeMovingGateBypassed);
    boolean aimedFeed = feedGate.update(clock.seconds(), ((shootAll && aim != null) || oneBall) && !holdFire && !loadingShooter.blocksSpin(),
        oneBall ? oneBallReady : ready, shooter.getRpm(), shooter.getTargetRpm(), oneBall, shotLost,
        readyButWheels && !shooter.isStalled());
    if (volleyMode) { // The other gate starts afresh when the shot switches between volley and one ball
      (volleyShot ? aimedFeedGate : volleyFeedGate).update(clock.seconds(), false, false, shooter.getRpm(),
          shooter.getTargetRpm());
    }
    Logger.recordOutput("Feeder/Gate", feedGate.stateName());
    Logger.recordOutput("Feeder/BallLeft", feedGate.ballLeft());
    long tFeed = System.nanoTime();
    updateIntakeFeederIndexer(aiming, aimedFeed || manualReady);
    long tFeedSet = System.nanoTime();
    if (aimedFeed) {
      feedCommitUntilS = clock.seconds() + ShootOnTheMoveConstants.FEED_COMMIT_S;
    }
    lastPose = pose;
    lastVelocity = velocity;
    lastFeeding = aimedFeed || manualReady;

    long tRun = System.nanoTime();
    CommandScheduler.getInstance().run(); // After the states are set, so subsystems act on and log this loop's
    stampPoseHistory(pose); // After the run, which is when vision takes this loop's capture

    if (shooter.isStalled() && !shooterWasStalled) {
      physicalDriver.rumble(STALL_RUMBLE_MS);
    }
    shooterWasStalled = shooter.isStalled();
    int lockRumbleMs = intakeLockRumbleMs(intake.isOverloaded(), clock.seconds(), intakeLockRumbleS);
    if (lockRumbleMs > 0) {
      physicalDriver.rumble(lockRumbleMs);
      intakeLockRumbleS = clock.seconds();
    } else if (!intake.isOverloaded()) {
      intakeLockRumbleS = Double.NaN;
    }

    long tLog = System.nanoTime();
    // The Driver Station shows it at 4 Hz at most, and its formatting cost 10-18 ms a loop (7 Nov, 080123)
    boolean telemetryDue = clock.seconds() - lastTelemetryS >= DS_TELEMETRY_PERIOD_S;
    updateBattery(telemetryDue);
    logAim(aim);
    if (volleyMode) {
      logVolley();
    }
    drawField(pose, aim);
    if (telemetryDue) {
      lastTelemetryS = clock.seconds();
      addTelemetry(aim, pose);
      telemetry.update();
    } else {
      telemetry.clear(); // The OpMode's lines for this loop, so they don't pile up until the next update
    }
    sendPanelsTelemetry(aim, pose);
    long tEnd = System.nanoTime();
    Logger.recordOutput("Loop/StartMs", (tVision - t0) * 1e-6);
    Logger.recordOutput("Loop/VisionAndControlMs", (tRun - tVision) * 1e-6);
    Logger.recordOutput("Loop/PoseFromVisionMs", (tAim - tVision) * 1e-6);
    Logger.recordOutput("Loop/AimMs", (tAimed - tAim) * 1e-6);
    Logger.recordOutput("Loop/ShooterHoodMs", (tShooterSet - tShooter) * 1e-6);
    Logger.recordOutput("Loop/DriveMs", (tDriveSet - tDrive) * 1e-6);
    Logger.recordOutput("Loop/IntakeFeederMs", (tFeedSet - tFeed) * 1e-6);
    Logger.recordOutput("Loop/SubsystemsMs", (tLog - tRun) * 1e-6);
    Logger.recordOutput("Loop/TelemetryMs", (tEnd - tLog) * 1e-6);
  }

  /**
   * Corrects the pose from the Hive tags, and moves the tracker's stored Cell and poses by the same
   * amount so the aim does not jump. Returns the pose to use this loop.
   */
  private Pose correctPoseFromVision(Pose pose, double[] velocity, long nowNanos) {
    List<TagObservation> observations = vision.takeNewTagObservations();
    noteHiveTags(observations);
    visionCorrector.setHeadingSeeded(poseSeeded);
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
      if (result.gain > 0) { positionFixes++; }
      headingValid |= result.headingGain > 0; // Likewise the first heading fix
      pose = result.newPose; // The Pinpoint may not report the new pose until its next update
    }
    if (result.reason != VisionResult.Reason.NO_OBSERVATIONS) {
      lastVision = result;
      lastVisionNanos = nowNanos;
      outOfFieldFrames = result.reason == VisionResult.Reason.OUT_OF_FIELD ? outOfFieldFrames + 1 : 0;
      if (result.headingPairs > 0 && headingValid) {
        boolean off = Math.abs(result.dHeadingRad) > HEADING_OFF_RAD;
        headingOffFrames = off ? headingOffFrames + 1 : 0;
        if (off) { lastHeadingOffRad = result.dHeadingRad; }
      }
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
   * is picked once, as the one ready soonest (ShootOnTheMove.secondsToReady: turn, hood and wheels), and kept
   * until RT is let go: their headings are 180 degrees apart, so switching between them each loop spins the
   * robot back and forth. A backward shot blocked only by the robot's speed still counts, SHOT_PICK_BLOCKED_S
   * later: the robot then lines up and the hood goes down, and it fires once slowed. Picking only a ready
   * backward shot turned the robot 180 degrees to the hooded one whenever the driver was still moving at the
   * press (5 Nov, 092433). SHOT_PICK_QUICKEST false picks by distance instead (BACKWARD_PICK_MAX_DIST_M).
   */
  /** The Cell on the robot's side of the Hive, changing only once the robot is FRONT_CELL_SWITCH_M past the middle. */
  static VisionCorrector.Cell frontCell(VisionCorrector.Cell current, double xM) {
    double margin = ShootOnTheMoveConstants.FRONT_CELL_SWITCH_M;
    if (current == null) {
      return xM >= 0 ? VisionCorrector.Cell.AUDIENCE : VisionCorrector.Cell.SCORING;
    }
    if (current == VisionCorrector.Cell.AUDIENCE && xM < -margin) {
      return VisionCorrector.Cell.SCORING;
    }
    if (current == VisionCorrector.Cell.SCORING && xM > margin) {
      return VisionCorrector.Cell.AUDIENCE;
    }
    return current;
  }

  /** The Cell aimed at: the one in front ({@link #frontCell}), or the tags' up Cell with AIM_AT_FRONT_CELL off. */
  private VisionCorrector.Cell aimCell() {
    return ShootOnTheMoveConstants.AIM_AT_FRONT_CELL && frontCell != null ? frontCell : visionCorrector.upCell(alliance);
  }

  private Aim aimAtCell(Pose pose, double[] velocity, long nowNanos, boolean aiming) {
    tracker.recordPose(nowNanos, pose);
    // Every loop, aiming or not, so it has settled by the time RT is pressed. Only the aim uses it
    // Loop clock, not the frame's capture time: that only moves when a camera frame arrives
    double velocityNowS = clock.seconds();
    aimVelocity = ShootOnTheMove.smoothAimVelocity(aimVelocity, velocity, velocityNowS - aimVelocityS,
        ShootOnTheMoveConstants.AIM_VELOCITY_SMOOTHING_S);
    aimVelocityS = velocityNowS;
    Logger.recordOutput("SOTM/AimVelocity", aimVelocity);
    // Spinning fast, the frame's pose is too uncertain and moved the Cell 0.17 m during one shot (5 Nov)
    boolean steady = Math.abs(velocity[2]) <= ShootOnTheMoveConstants.MAX_OMEGA_FOR_VISION_RAD_S;
    for (CellSighting sighting : vision.takeNewCellSightings()) {
      Alliance other = alliance == Alliance.RED ? Alliance.BLUE : Alliance.RED;
      if (allianceFromTags && other.ownsCluster(sighting.clusterName) && !tracker.knowsCell(nowNanos)
          && !visionCorrector.upCellConfirmed(alliance)) {
        alliance = other;
      }
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
    if (!aiming || frontCell == null) { // Held for the whole shot
      frontCell = frontCell(frontCell, pose.xM);
    }
    aimingAtFieldCell = ShootOnTheMoveConstants.AIM_AT_FIELD_CELL
        && (poseValid && (ShootOnTheMoveConstants.AIM_AT_FRONT_CELL || visionCorrector.upCellConfirmed(alliance))
            || poseSeeded);
    double cellX;
    double cellY;
    if (aimPointOverride != null) {
      if (!poseValid) {
        return null;
      }
      cellX = aimPointOverride[0];
      cellY = aimPointOverride[1];
    } else if (aimingAtFieldCell) {
      double[] point = VisionCorrector.upCellAimPoint(alliance, aimCell());
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
    Aim hooded = ShootOnTheMove.aimHooded(pose, aimVelocity[0], aimVelocity[1], cellX, cellY);
    if (hoodedOnly) {
      aimBackwardLatched = false;
      prepositionBackward = false;
      return hooded;
    }
    Aim backward = ShootOnTheMove.aimBackward(pose, aimVelocity[0], aimVelocity[1], cellX, cellY);
    if (tuningMode && tuningLive) {
      return tuningBackward ? backward : hooded;
    }
    if (tuningMode) {
      // Tight, so a tuning miss is down to the RPM and hood rather than where in the gate the ball left
      return (tuningBackward ? backward : hooded).withShot(tuningRpm, tuningHoodDeg,
          Math.toRadians(TUNING_HEADING_TOLERANCE_DEG));
    }
    boolean pickBackward;
    boolean frontOfCell = ShootOnTheMoveConstants.BACKWARD_SHOTS_ON && wedgeForcesBackward(
        ShootOnTheMove.inFrontOfCell(pose, cellX, cellY, ShootOnTheMoveConstants.BACKWARD_ONLY_WEDGE_DEG),
        backward.shot.distanceToLipM);
    if (!ShootOnTheMoveConstants.BACKWARD_SHOTS_ON || backward.shot.reason == Shot.Reason.OUT_OF_RANGE) {
      pickBackward = false;
    } else if (frontOfCell) {
      pickBackward = true;
    } else if (ShootOnTheMoveConstants.SHOT_PICK_QUICKEST) {
      // Whichever is ready soonest from the heading, hood and wheels now (8 Nov): no backward arc
      pickBackward = ShootOnTheMove.backwardQuicker(hooded, backward, pose.headingRad,
          HoodSubsystem.ticksToAngle(hood.getPosition()), shooter.getRpm());
    } else {
      pickBackward = hooded.shot.reason == Shot.Reason.OUT_OF_RANGE
          || backward.shot.distanceToLipM <= ShootOnTheMoveConstants.BACKWARD_PICK_MAX_DIST_M;
    }
    Logger.recordOutput("SOTM/PickBackward", pickBackward);
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
      } else if (!frontOfCell && nowS - backwardBlockedSinceS >= ShootOnTheMoveConstants.BACKWARD_GIVE_UP_S
          && hooded.shot.valid) {
        aimBackwardLatched = false; // For the rest of this press, so it doesn't turn back and forth
      }
    }
    return aimBackwardLatched ? backward : hooded;
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
    stopHint = aiming ? stopHint(aim, newState, closeMovingGateBypassed) : null;
    double nowS = clock.seconds();
    if (stopHint == null) {
      stopHintSinceS = Double.NaN;
    } else if (Double.isNaN(stopHintSinceS)) {
      stopHintSinceS = nowS;
    }
    if (!aiming) {
      stopHintRumbled = false;
    } else if (!stopHintRumbled && !Double.isNaN(stopHintSinceS) && nowS - stopHintSinceS >= STOP_HINT_RUMBLE_AFTER_S) {
      physicalDriver.rumble(STOP_HINT_RUMBLE_MS); // Once a press: the driver is moving in a way that can't shoot
      stopHintRumbled = true;
    }
    Logger.recordOutput("SOTM/StopHint", stopHint == null ? "" : stopHint);
  }

  /** What the driver must change for the shot to fire ({@link ShootOnTheMove#stopHint}), else null. */
  static String stopHint(Aim aim, AimState state, boolean closeMovingBypassed) {
    if (aim == null || (state != AimState.LINING_UP && state != AimState.NO_SAFE_SHOT)) {
      return null;
    }
    return ShootOnTheMove.stopHint(aim.shot, closeMovingBypassed);
  }

  /** While aiming, D-pad up and down nudge the RPM, for when the table is a little off on the day. */
  private void updateTrim(boolean aiming) {
    if (!buttonsFree()) {
      return;
    }
    if (aiming && justPressed(driver.dpad_up, wasDpadUp)) {
      rpmTrim += ShootOnTheMoveConstants.RPM_TRIM_STEP;
    }
    if (aiming && justPressed(driver.dpad_down, wasDpadDown)) {
      rpmTrim -= ShootOnTheMoveConstants.RPM_TRIM_STEP;
    }
    wasDpadUp = driver.dpad_up;
    wasDpadDown = driver.dpad_down;
  }

  /**
   * Left bumper deliberately rearms a cooled intake overload trip. D-pad up toggles the Pollen ramp when not aiming
   * (aiming, it trims the RPM).
   */
  private void updateToggles(boolean aiming) {
    if (tuningMode) {
      return;
    }
    if (justPressed(driver.left_bumper, wasLeftBumper)) {
      intake.resetOverload();
    }
    wasLeftBumper = driver.left_bumper;
    if (rampToggled(driver.dpad_up, wasRampButton, aiming, buttonsFree())) {
      ramp.toggle();
    }
    wasRampButton = driver.dpad_up;
  }

  /** D-pad up just pressed, not aiming (where it trims the RPM) and with the buttons not taken by a tuning tool. */
  static boolean rampToggled(boolean pressed, boolean wasPressed, boolean aiming, boolean buttonsFree) {
    return buttonsFree && !aiming && justPressed(pressed, wasPressed);
  }

  /**
   * How long to rumble this loop for a tripped intake, which stays off until LB: long when it trips, then a short
   * reminder every {@link #INTAKE_LOCK_REMIND_S} while it is still off. 0 for no rumble.
   */
  static int intakeLockRumbleMs(boolean tripped, double nowS, double lastRumbleS) {
    if (!tripped) {
      return 0;
    }
    if (Double.isNaN(lastRumbleS)) {
      return INTAKE_LOCK_RUMBLE_MS;
    }
    return nowS - lastRumbleS >= INTAKE_LOCK_REMIND_S ? INTAKE_LOCK_REMIND_MS : 0;
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

  /**
   * True once the drive command has held still long enough for the robot to reach its speed, and the robot is no
   * longer speeding up or slowing down: a shot is aimed for the speed now, so it must still be the speed at release.
   */
  private boolean updateMotionSteady(boolean aiming, double[] velocity) {
    double nowS = clock.seconds();
    double maxPower = aiming ? ShootOnTheMoveConstants.AIM_DRIVE_MAX_POWER : 1.0;
    // Held on the spot the driver asks for no motion; the hold's own corrections show up as acceleration
    double[] stick = isHolding() ? new double[] {0, 0}
        : driveOverride != null && steadyStickReference != null ? steadyStickReference
        : stickTranslation(maxPower);
    if (Math.hypot(stick[0] - steadyStick[0], stick[1] - steadyStick[1]) > ShootOnTheMoveConstants.FIRE_STICK_CHANGE) {
      steadyStick = stick;
      steadyStickSinceS = nowS;
    }
    velocityHistory.addLast(new double[] {nowS, velocity[0], velocity[1]});
    while (velocityHistory.size() > 2 && nowS - velocityHistory.peekFirst()[0] > ShootOnTheMoveConstants.ACCEL_WINDOW_S) {
      velocityHistory.removeFirst();
    }
    double[] oldest = velocityHistory.peekFirst();
    double spanS = nowS - oldest[0];
    double accelMps2 = spanS > 0.02 ? Math.hypot(velocity[0] - oldest[1], velocity[1] - oldest[2]) / spanS : 0.0;
    boolean steady = nowS - steadyStickSinceS >= ShootOnTheMoveConstants.FIRE_STEADY_STICK_S
        && accelMps2 <= ShootOnTheMoveConstants.MAX_ACCEL_TO_FIRE_MPS2;
    Logger.recordOutput("SOTM/AccelMps2", accelMps2);
    Logger.recordOutput("SOTM/Gate/Steady", steady);
    Logger.recordOutput("SOTM/Gate/SteadyBypassed", steadyGateBypassed);
    return steady || steadyGateBypassed;
  }

  /** {forward, right} for the drive, from the left stick or, while one is set, {@link #setDriveOverride}. */
  private double[] stickTranslation(double maxPower) {
    if (driveOverride == null && sticksLocked) {
      return new double[] {0, 0};
    }
    if (driveOverride == null) {
      return MecanumDriveSubsystem.shapeTranslation(-driver.left_stick_y, driver.left_stick_x, maxPower);
    }
    double f = driverForwardHeadingRad();
    double forward = driveOverride[0] * Math.cos(f) + driveOverride[1] * Math.sin(f);
    double right = driveOverride[0] * Math.sin(f) - driveOverride[1] * Math.cos(f);
    return MecanumDriveSubsystem.shapeTranslation(forward, right, maxPower);
  }

  private boolean isHolding() { return !Double.isNaN(holdAnchorX); }

  /** {forward, right} power back to RB's held spot, damped by the speed. Not curved as a stick is. */
  private double[] holdTranslation(Pose pose, double minPower) {
    double ex = holdAnchorX - pose.xM;
    double ey = holdAnchorY - pose.yM;
    double away = Math.hypot(ex, ey);
    double vx = -HOLD_DAMPING_S * HOLD_GAIN_PER_M * lastVelocity[0];
    double vy = -HOLD_DAMPING_S * HOLD_GAIN_PER_M * lastVelocity[1];
    if (away > HOLD_DEADBAND_M) {
      double power = Math.min(HOLD_MAX_POWER, minPower + HOLD_GAIN_PER_M * (away - HOLD_DEADBAND_M));
      vx += ex / away * power;
      vy += ey / away * power;
    }
    double magnitude = Math.hypot(vx, vy);
    if (magnitude > HOLD_MAX_POWER) {
      vx *= HOLD_MAX_POWER / magnitude;
      vy *= HOLD_MAX_POWER / magnitude;
    }
    Logger.recordOutput("Drive/PositionHold/ErrorM", away);
    double f = driverForwardHeadingRad();
    return new double[] {vx * Math.cos(f) + vy * Math.sin(f), vx * Math.sin(f) - vy * Math.cos(f)};
  }

  /** {@link #holdTranslation} for a scripted hold point: nothing near it, and back to it once pushed off. */
  private double[] holdPointTranslation(Pose pose) {
    double nowS = clock.seconds();
    double away = Math.hypot(holdAnchorX - pose.xM, holdAnchorY - pose.yM);
    if (!holdPointReturning && away > HOLD_POINT_RETURN_M) {
      holdPointReturning = true;
      holdPointStuckSinceS = nowS;
    } else if (holdPointReturning && away < HOLD_POINT_DONE_M) {
      holdPointReturning = false;
    }
    Logger.recordOutput("Drive/PositionHold/Returning", holdPointReturning);
    if (!holdPointReturning) {
      Logger.recordOutput("Drive/PositionHold/ErrorM", away);
      return new double[] {0, 0};
    }
    if (Math.hypot(lastVelocity[0], lastVelocity[1]) > HOLD_POINT_MOVING_MPS) {
      holdPointStuckSinceS = nowS;
    }
    return holdTranslation(pose, HOLD_MIN_POWER + HOLD_POINT_BOOST_PER_S * (nowS - holdPointStuckSinceS));
  }

  private void updateDrive(boolean aiming, Aim aim, double lockRateRadPerSec, Pose pose, double turnRateRadPerSec) {
    // Slower while shooting, so the shot's lead for the robot's speed stays small and the aim keeps up
    double maxPower = aiming ? ShootOnTheMoveConstants.AIM_DRIVE_MAX_POWER : 1.0;
    double[] translation = !isHolding() ? stickTranslation(maxPower)
        : holdPoint != null && driveOverride == null ? holdPointTranslation(pose) : holdTranslation(pose, HOLD_MIN_POWER);
    // The drive runs before this loop's feed check, so lastTranslation is what was commanded as the feed started
    double nowS = clock.seconds();
    boolean stickReleased = translation[0] == 0.0 && translation[1] == 0.0;
    if (isHolding()) {
      feedCommitUntilS = Double.NEGATIVE_INFINITY; // The hold keeps it still through the feed already
    }
    if (committedTranslation != null
        && (stickReleased || nowS - committedSinceS > ShootOnTheMoveConstants.FEED_COMMIT_MAX_S)) {
      feedCommitUntilS = Double.NEGATIVE_INFINITY;
    }
    if (aiming && nowS < feedCommitUntilS) {
      if (committedTranslation == null) {
        committedTranslation = lastTranslation;
        committedSinceS = nowS;
      }
      translation = committedTranslation;
    } else {
      committedTranslation = null;
    }
    lastTranslation = translation;
    Logger.recordOutput("Drive/FeedCommit", committedTranslation != null);
    double forward = translation[0];
    double right = translation[1];
    double forwardOffsetRad = driverForwardHeadingRad();
    double driverHeadingRad = pose.headingRad - forwardOffsetRad;
    if (aiming && aim != null) { // Lines up even without a valid shot, so it's ready the moment one appears
      // Target and heading both less the offset, so the error is the same and the driving is driver relative
      drive.driveHeadingLock(forward, right, aim.lockHeadingRad - forwardOffsetRad, lockRateRadPerSec,
          driverHeadingRad, turnRateRadPerSec);
      headingHold.clear();
    } else {
      double turn = sticksLocked ? turnOverride : MecanumDriveSubsystem.shapeTurnStick(driver.right_stick_x);
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

  /**
   * Drives with the Scoring Auto's powers. They are robot-frame (Pedro's), so they go to the field frame and then to
   * the driver frame the drive takes. While aiming the heading lock does the turning, as it does for the driver.
   * Straight to the drive: no stick curve, so Pedro's powers are the motors' powers.
   */
  private void driveAuto(boolean aiming, Aim aim, double lockRateRadPerSec, Pose pose, double[] velocity) {
    double[] powers = autoDriver.drive(pose, velocity); // {forward, left, counterclockwise}
    if (powers == null) { // Hold still: no aim turn either
      drive.drive(0, 0, 0, true, pose.headingRad - driverForwardHeadingRad());
      headingHold.clear();
      lastTranslation = new double[] {0, 0};
      Logger.recordOutput("Auto/DrivePowers", new double[] {0, 0, 0});
      return;
    }
    double f = driverForwardHeadingRad();
    double[] translation = robotToDriverFrame(powers[0], powers[1], pose.headingRad, f);
    double magnitude = Math.hypot(translation[0], translation[1]);
    if (aiming && magnitude > ShootOnTheMoveConstants.AIM_DRIVE_MAX_POWER) { // Slower while shooting, as for the driver
      translation[0] *= ShootOnTheMoveConstants.AIM_DRIVE_MAX_POWER / magnitude;
      translation[1] *= ShootOnTheMoveConstants.AIM_DRIVE_MAX_POWER / magnitude;
    }
    double forward = translation[0];
    double right = translation[1];
    double driverHeadingRad = pose.headingRad - f;
    if (aiming && aim != null) {
      drive.driveHeadingLock(forward, right, aim.lockHeadingRad - f, lockRateRadPerSec, driverHeadingRad, velocity[2]);
    } else if (!Double.isNaN(autoHeadingRad)) {
      // Pedro's turn fades under the friction near its heading: 14 Nov it stalled 8 degrees off at 0.14 power, where
      // the lock's least power and stuck boost turn the robot
      drive.driveHeadingLock(forward, right, autoHeadingRad - f, autoHeadingRateRadPerSec, driverHeadingRad,
          velocity[2]);
    } else {
      drive.drive(forward, right, -powers[2], true, driverHeadingRad);
    }
    headingHold.clear();
    lastTranslation = new double[] {forward, right};
    Logger.recordOutput("Auto/DrivePowers", powers);
  }

  /**
   * A robot-frame translation {forward, left} as the driver-frame {forward, right} the field-centric drive takes,
   * for a robot at headingRad and a driver whose forward is driverForwardRad.
   */
  static double[] robotToDriverFrame(double forward, double left, double headingRad, double driverForwardRad) {
    double fieldX = forward * Math.cos(headingRad) - left * Math.sin(headingRad);
    double fieldY = forward * Math.sin(headingRad) + left * Math.cos(headingRad);
    return new double[] {fieldX * Math.cos(driverForwardRad) + fieldY * Math.sin(driverForwardRad),
        fieldX * Math.sin(driverForwardRad) - fieldY * Math.cos(driverForwardRad)};
  }

  private void updateShooterAndHood(boolean aiming, Aim aim) {
    volleyHoodDeg = 0;
    boolean validShot = aim != null && aim.shot.valid;
    // A backward shot waiting only for the robot to slow down: hood down and wheels at its speed already
    boolean backwardWaiting = aim != null && aim.backward && !validShot
        && aim.shot.reason != Shot.Reason.OUT_OF_RANGE;
    boolean shotKnown = validShot || backwardWaiting;
    double nowS = clock.seconds();
    if (shotKnown) { lastShotKnownS = nowS; }
    // A shot gone for a moment (NO_SAFE_SHOT flickers, a median 0.42 s, p90 1.2 s, 13 Nov) keeps its speed:
    // dropping to IDLE braked the wheels a median 1350 RPM, 478 times that day, and they then had to come back
    boolean shotJustLost = !shotKnown && nowS - lastShotKnownS <= ShootOnTheMoveConstants.PRESPIN_HOLD_S;
    double rpmRate = shotKnown ? aim.rpmRatePerSec : 0.0;
    // The shot is for the ball leaving FEED_DELAY_S from now. The wheels and hood only have to be there by
    // then, so they follow the shot for now and arrive at this one as the ball leaves. Holding this one
    // already put them a feed delay early: 4 degrees of hood at 20 degrees/s (5 Nov, 080236).
    if (shotKnown && !Double.isNaN(aim.rpmNow)) {
      aimRpm = aim.rpmNow;
    }

    boolean fullPower = driver.y && buttonsFree();
    boolean eject = driver.b && buttonsFree();
    if (aiming || fullPower || eject) {
      shooterStopLatched = false;
    } else if (driver.x && buttonsFree()) {
      shooterStopLatched = true;
    }
    if (volleyMode) { // Full power back up to speed after each ball of a volley
      boolean fast = volleyShot && (volleyFastRecovery != null ? volleyFastRecovery
          : ShootOnTheMoveConstants.VOLLEY_FAST_RECOVERY);
      shooter.setSpinThresholds(fast ? ShootOnTheMoveConstants.VOLLEY_OPEN_LOOP_BELOW_RPM : Double.NaN,
          fast ? ShootOnTheMoveConstants.VOLLEY_HANDBACK_RPM : Double.NaN);
    }

    if (eject) {
      shooter.setShooterState(ShooterState.EJECT);
    } else if (loadingShooter.blocksSpin()) {
      shooter.setShooterState(ShooterState.STOP); // Heading/hood still track below; no powered idle while filling.
    } else if (aiming) {
      shooter.setShooterState(ShooterState.AIM);
      // No Cell in view means no shot table to read, so go for the top speed
      shooter.setTargetRpm(ShooterTuning.RPM_OVERRIDE > 0
          ? ShooterTuning.RPM_OVERRIDE
          : aim == null ? shooter.getMaxRpm() : aimRpm + rpmTrim + tableRpmOffset(aim) + volleyBoostRpm(aim),
          ShooterTuning.RPM_OVERRIDE > 0 ? 0.0 : rpmRate);
    } else if (fullPower) {
      if (ShooterTuning.RPM_OVERRIDE > 0) {
        shooter.setShooterState(ShooterState.AIM);
        shooter.setTargetRpm(ShooterTuning.RPM_OVERRIDE);
      } else {
        shooter.setShooterState(ShooterState.SHOOT);
      }
    } else if (shooterStopLatched || tuningMode) { // The tuning OpModes rest the wheels between shots
      shooter.setShooterState(ShooterState.STOP);
    } else if (shotKnown && ShootOnTheMoveConstants.PRESPIN_ON) {
      // Already at the speed RT would ask for, so a shot doesn't wait on spin-up (0.8 s from idle, 4 Nov)
      shooter.setShooterState(ShooterState.AIM);
      shooter.setTargetRpm(ShooterTuning.RPM_OVERRIDE > 0 ? ShooterTuning.RPM_OVERRIDE
          : aimRpm + rpmTrim + tableRpmOffset(aim) + volleyBoostRpm(aim),
          ShooterTuning.RPM_OVERRIDE > 0 ? 0.0 : rpmRate);
    } else if (shotJustLost && ShootOnTheMoveConstants.PRESPIN_ON
        && shooter.getShooterState() == ShooterState.AIM) {
      shooter.setShooterState(ShooterState.AIM); // At the last target, which it keeps
    } else {
      shooter.setShooterState(ShooterState.IDLE);
    }

    // Not aiming but the Cell is known: move the hood to the shot RT would take now, while driving,
    // so it is already there when RT is pressed. Presets are for when the Cell isn't known.
    boolean prepositioning = !aiming && shotKnown && !fullPower && !eject;
    if (aiming || prepositioning) {
      if (tuningMode && !tuningLive && validShot) {
        // Lift 0 is the lower stop; held just above it, as a backward shot's is, so the hood can't sag
        hood.setTargetTicks(Math.max(HoodSubsystem.minTargetTicks(), HoodSubsystem.angleToTicks(aim.hoodNowDeg)));
      } else if (shotKnown && aim.backward) {
        // Never under just above the lower stop. Target 0 cuts the motor to rest on the stop, but the hood
        // doesn't fall by itself: it stayed ~36 degrees up and the shot never got ready. Trim doesn't apply
        hood.setTargetTicks(Double.isNaN(aim.hoodNowDeg) ? HoodSubsystem.minTargetTicks()
            : Math.max(HoodSubsystem.minTargetTicks(), HoodSubsystem.angleToTicks(aim.hoodNowDeg)));
      } else if (validShot) { // Otherwise the hood holds its last angle
        // Lift rises as the launch angle falls
        volleyHoodDeg = volleyHoodDeg(aim);
        hood.setTargetTicks(HoodSubsystem.angleToTicks(aim.hoodNowDeg - hoodLiftTrimDeg + movingHoodDeg(aim)
            + volleyHoodDeg), -aim.hoodRateDegPerSec);
      }
    } else if (!buttonsFree()) {
      return; // The D-pad sets the tuning shot, or marks balls, instead
    } else if (driver.dpad_up) {
      hood.setPreset(HoodPreset.FAR);
    } else if (driver.dpad_down) {
      hood.setPreset(HoodPreset.CLOSE);
    }
    if (!buttonsFree()) {
      return;
    }
    double lift = hood.getTargetLiftDegrees();
    double steppedLift = hoodSteps.update(lift, driver.dpad_right, driver.dpad_left,
        HoodSubsystem.ZERO_ANGLE_DEG - HoodSubsystem.MIN_ANGLE_DEG);
    if (steppedLift != lift) {
      if (aiming || prepositioning) { hoodLiftTrimDeg += steppedLift - lift; }
      hood.setTargetLiftDegrees(steppedLift);
    }
  }

  private boolean shotRequested() {
    double fireTrigger = fireTrigger();
    boolean requested = fireTrigger > TRIGGER_PRESSED || driver.right_bumper
        || scriptedOneBall || scriptedVolley || (driver.y && buttonsFree());
    return requested && !(tuningMode && fireTrigger <= TRIGGER_PRESSED
        && feedGate != null && feedGate.ballsLeft() > 0);
  }

  /**
   * Intake collects while LT is held (always, with INTAKE_ONLY_ON_LT off) and reverses at full capacity. An aimed shot holds the
   * feeder backward and stops the last roller until READY. LT advances that roller gently against
   * the held feeder until a loading ball touches the spinning flywheels (WheelContact), the sign of a full stack. Between shot feeds,
   * the upstream stage keeps the second/third balls moving toward the stopped last roller.
   */
  private void updateIntakeFeederIndexer(boolean aiming, boolean ready) {
    boolean shooting = aiming || (driver.y && buttonsFree());
    boolean fireRequested = shotRequested();
    boolean shotAllowed = fireRequested && !holdFire;
    boolean enabledShot = shotAllowed && !loadingShooter.blocksSpin();
    boolean ejecting = driver.b && buttonsFree();
    double feederPower = volleyMode && volleyShot && enabledShot ? volleyFeedPower() : 1.0;
    double loadTrigger = loadTrigger(driver.left_trigger, autoIntake);
    boolean ltPressed = loadTrigger > TRIGGER_PRESSED;
    boolean loading = ltPressed && !shotAllowed;
    // Full until a ball leaves, B ejects or LT is pressed again after a real let-go (the way out if a bump looked
    // like a touch). A tap doesn't count, or tapping LT pushed the packed stack at full power on every press
    double nowS = clock.seconds();
    boolean wheelsSpinning = Math.abs(shooter.getMaxMeasuredRpm()) > WHEELS_STOPPED_RPM;
    if (loadingLatchesClear(ltPressed && !wasLtPressed, nowS - ltReleasedAtS, ejecting, feedGate.ballLeft(),
        wheelsSpinning)) {
      wheelContactFull = false;
      stackPacked = false;
      intakeEased = false;
    }
    // The intake's ease clears on a real LT re-press even with the wheels spinning: the last roller stays stopped
    // (stackPacked) until a ball leaves, so the front ball still can't creep into the wheels (14 Nov 222230)
    if (intakeEaseClears(ltPressed && !wasLtPressed, nowS - ltReleasedAtS,
        stackPressure.runningFree(nowS, intakeEased && intake.getIntakeState() == IntakeState.INTAKE,
            intake.getAmps()))) {
      intakeEased = false;
    }
    if (!ltPressed && wasLtPressed) {
      ltReleasedAtS = nowS;
    }
    wasLtPressed = ltPressed;
    boolean watchingWheels = loading && !wheelContactFull && !tuningMode
        && indexer.getIndexerState() == IndexerState.LOAD && shooter.getShooterState() == ShooterState.AIM;
    if (wheelContact.update(clock.seconds(), watchingWheels, shooter.getRpm(), shooter.getTargetRpm())) {
      wheelContactFull = true;
    }
    Logger.recordOutput("Intake/WheelContactFull", wheelContactFull);
    // Not in the auto, whose intake pushes into the Flower's balls the whole time and has no LT to start over.
    // Still packed (last roller stopped), a full intake pushing hard again eases again
    boolean watchingIntake = loading && !intakeEased && !autoIntake && !tuningMode
        && (indexer.getIndexerState() == IndexerState.LOAD || stackPacked)
        && intake.getIntakeState() == IntakeState.INTAKE;
    if (stackPressure.update(clock.seconds(), watchingIntake, intake.getAmps())) {
      stackPacked = true;
      intakeEased = true;
    }
    Logger.recordOutput("Intake/StackPacked", stackPacked);
    Logger.recordOutput("Intake/Eased", intakeEased);
    intake.setIntakePower(intakeEased ? StackPressure.PACKED_INTAKE_POWER : 1.0);
    boolean full = intake.isFull() || wheelContactFull;
    if (full && !wasFull) {
      physicalDriver.rumble(300);
    }
    wasFull = full;

    boolean intakeWanted = tuningMode || !ShootOnTheMoveConstants.INTAKE_ONLY_ON_LT
        || loadTrigger > TRIGGER_PRESSED;
    intakeOn = intakeWanted;
    intake.setIntakeState(intakeCommand(full, ejecting, intakeWanted, autoIntake));
    boolean feeding = enabledShot && (!aiming || ready);
    // A volley slows the indexer only while a ball feeds, so the next can't follow it in; between balls it stages
    // the stack at full power against its stopped last roller, or the 4th ball took 2.9 s to come up (12 Nov)
    // The auto feeds at full power: its slowed last roller brought the next ball up too little, 14 Nov 220201
    double indexerPower = volleyMode && volleyShot && feeding && !autonomous ? volleyIndexerPower() : 1.0;
    IndexerState indexerCommand = indexerCommand(ejecting, fireRequested, feeding, loadTrigger,
        holdFire || loadingShooter.blocksSpin(), full);
    indexerCommand = packedLoad(indexerCommand, stackPacked);
    indexerCommand = autoStage(indexerCommand, autonomous);
    if (!ejecting && (loadingShooter.clearing() || (loading && !loadingShooter.canLoad())
        || (spinOnlyOnFire && enabledShot && !feeding && feedGate.feedStarts() == 0))) {
      indexerCommand = IndexerState.STOP;
    }
    // Retry only after a volley has actually started. All release, clearance and shot gates still apply.
    boolean retryAllowed = volleyShot && enabledShot && !ejecting && feedGate.feedStarts() > 0
        && (indexerCommand == IndexerState.STAGE || indexerCommand == IndexerState.FEED
            || (autonomous && indexerCommand == IndexerState.LOAD));
    indexerRetry.update(clock.seconds(), retryAllowed, feedGate.ballLeft());
    // The retry's pause: in the auto a short reverse, to back a stuck ball off before full power forward
    if (indexerRetry.paused()) { indexerCommand = autonomous ? IndexerState.EJECT : IndexerState.STOP; }
    if (indexerRetry.fullPower()) { indexerPower = 1.0; }
    indexer.setIndexerState(indexerCommand, indexerPower);
    Logger.recordOutput("Indexer/Retry/Paused", indexerRetry.paused());
    Logger.recordOutput("Indexer/Retry/FullPower", indexerRetry.fullPower());
    if (spinOnlyOnFire && loading && !loadingShooter.canLoad()) {
      telemetry.addLine("LT: waiting for flywheels to stop");
    }
    boolean volleyBetweenBalls = volleyShot && volleyFeedGate.ballsLeft() > 0
        && !ShootOnTheMoveConstants.VOLLEY_HOLD_BETWEEN_BALLS;
    FeederState feederCommand = feederCommand(ejecting, loading || loadingShooter.clearing(), shooting, feeding,
        fireRequested, feedGate.feederEmpty(), volleyBetweenBalls);
    if (indexerRetry.paused()) { feederCommand = FeederState.STOP; }
    feeder.setFeederState(feederCommand, feederPower);
  }

  /**
   * The feeder's command. Held back whenever nothing shoots, as Volley Tuning's always-aimed robot does: stopped,
   * the spinning wheels and the intake's push took balls out of the stack between shots in Lebot Teleop (12 Nov),
   * and 14 Nov 211610 the auto, aiming all the time, left it stopped after an empty volley while the indexer staged
   * the next pickup at full power: balls went out on the way to the Flower.
   */
  static FeederState feederCommand(boolean ejecting, boolean loadingOrClearing, boolean shooting, boolean feeding,
      boolean fireRequested, boolean feederEmpty, boolean volleyBetweenBalls) {
    if (ejecting) {
      return FeederState.EJECT;
    }
    if (loadingOrClearing) {
      return FeederState.HOLD; // Even if the last trial's feed gate says empty
    }
    if (shooting && feeding) {
      return FeederState.FEED;
    }
    if (shooting && fireRequested && (feederEmpty || volleyBetweenBalls)) {
      // Between a volley's balls the feeder stops, so the next ball isn't pulled back from the wheels
      return FeederState.STOP;
    }
    return FeederState.HOLD; // The last roller isolates the moving upstream queue while an aimed shot waits
  }

  /** Below this the flywheels count as stopped: a ball pushed into them doesn't fly. */
  static final double WHEELS_STOPPED_RPM = 100;

  /**
   * "Packed" and "full" clear on a ball leaving, B, or an LT press after LT was let go REARM_RELEASE_S or more, the
   * last only with the flywheels stopped. 14 Nov 222230 a press 5.7 s after a packed stack was let go cleared it with
   * the wheels at 3500 RPM: loading pushed the stack at its last roller's power and shot the front ball out.
   */
  static boolean loadingLatchesClear(boolean newLtPress, double ltReleasedForS, boolean ejecting, boolean ballLeft,
      boolean wheelsSpinning) {
    return (newLtPress && ltReleasedForS >= StackPressure.REARM_RELEASE_S && !wheelsSpinning) || ejecting || ballLeft;
  }

  /** In front of the Cell the backward shot is forced, within the range its table was shot at. */
  static boolean wedgeForcesBackward(boolean inFrontOfCell, double backwardLipM) {
    return inFrontOfCell && backwardLipM <= ShootOnTheMoveConstants.BACKWARD_WEDGE_MAX_LIP_M;
  }

  /** The intake's ease ends on an LT press after a real let-go, or once it runs free (nothing pushing back). */
  static boolean intakeEaseClears(boolean newLtPress, double ltReleasedForS, boolean runningFree) {
    return (newLtPress && ltReleasedForS >= StackPressure.REARM_RELEASE_S) || runningFree;
  }

  /**
   * The intake as LT, B and a full stack ask; in the auto a full stack stops it instead of ejecting, so the Pollen
   * already taken stays in for the next volley (13 Nov 224556 ejected at the rear Flower for 3.2 s).
   */
  static IntakeState intakeCommand(boolean full, boolean ejecting, boolean wanted, boolean auto) {
    if (auto && full && !ejecting) {
      return IntakeState.STOP;
    }
    return IntakeSubsystem.automaticState(full, ejecting, wanted);
  }

  /**
   * In the auto the last roller keeps bringing balls up between shots, against the held-back feeder, as loading does:
   * stopped (STAGE), the next ball never reached the feeder in the 0.2-0.4 s feed pulses. 14 Nov 220201 the preload
   * shot 2 of 4, the Garden and Flower volleys none, the last roller still for 2 s before each.
   */
  static IndexerState autoStage(IndexerState command, boolean autonomous) {
    return autonomous && command == IndexerState.STAGE ? IndexerState.LOAD : command;
  }

  /** A packed stack loads with its last roller stopped, so nothing creeps the first ball into the wheels. */
  static IndexerState packedLoad(IndexerState command, boolean packed) {
    return packed && command == IndexerState.LOAD ? IndexerState.STAGE : command;
  }

  /** Intake protection only controls its motor; LT, eject and shot gates still control the indexer. */
  static IndexerState indexerCommand(boolean ejecting, boolean fireRequested, boolean feeding,
      double leftTrigger, boolean holdFire, boolean full) {
    if (ejecting) {
      return IndexerState.EJECT;
    }
    // Aim alone is not a request to run the indexer. An armed shot stages the upstream
    // queue through recovery gaps, with its independent last roller stopped until feeding.
    if (fireRequested && !holdFire) {
      return feeding ? IndexerState.FEED : IndexerState.STAGE;
    }
    return leftTrigger > TRIGGER_PRESSED && !full ? IndexerState.LOAD : IndexerState.STOP;
  }

  private static int feedStopHoodTicks() {
    return (int) Math.round(ShootOnTheMoveConstants.FEED_STOP_HOOD_DEG * HoodSubsystem.TICKS_PER_DEG);
  }

  private static int volleyHoodToleranceTicks() {
    return (int) Math.round(ShootOnTheMoveConstants.VOLLEY_HOOD_TOLERANCE_DEG * HoodSubsystem.TICKS_PER_DEG);
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

  /**
   * The flywheels, hood and aim on the Panels Telemetry tab, after any lines added with {@link #addPanelsLine}.
   * Sent from the field's thread for the same reason as the field, and skipped while a send is still blocked.
   */
  private void sendPanelsTelemetry(Aim aim, Pose pose) {
    double nowS = clock.seconds();
    if (!Main.PANELS_ENABLED || nowS - lastPanelsSendS < PANELS_TELEMETRY_PERIOD_S) {
      panelsLines.clear();
      return;
    }
    final java.util.List<String[]> lines = new java.util.ArrayList<>(panelsLines);
    panelsLines.clear();
    lines.add(new String[] {"Flywheel rpm", String.format("%.0f target / %.0f actual", shooter.getTargetRpm(),
        shooter.getRpm())});
    lines.add(new String[] {"Hood launch deg", String.format("%.1f target / %.1f actual  (lift %.1f / %.1f)",
        HoodSubsystem.ticksToAngle(hood.getTarget()), HoodSubsystem.ticksToAngle(hood.getPosition()),
        hood.getTargetLiftDegrees(), hood.getLiftDegrees())});
    lines.add(new String[] {"Aim", aim == null ? aimState + ", Cell not known"
        : String.format("%s  %s  %.2f m to lip  heading err %.1f deg", aimState,
            aim.backward ? "BACKWARD" : "HOODED", aim.shot.distanceToLipM,
            Math.toDegrees(aim.headingErrorRad(pose.headingRad)))});
    lines.add(new String[] {"Pose", String.format("%.2f, %.2f m  %.0f deg%s", pose.xM, pose.yM,
        Math.toDegrees(pose.headingRad), poseValid ? "" : "  (not placed by tags yet)")});
    lines.add(new String[] {"Battery", String.format("%.2f V", batteryVolts)});
    if (!panelsBusy.compareAndSet(false, true)) {
      return;
    }
    lastPanelsSendS = nowS;
    fieldSender.execute(() -> {
      try {
        com.bylazar.telemetry.TelemetryManager panels = com.bylazar.telemetry.PanelsTelemetry.INSTANCE.getTelemetry();
        for (String[] line : lines) {
          panels.addData(line[0], line[1]);
        }
        panels.update();
      } catch (RuntimeException e) {
        // Telemetry only: a failed send must not reach the loop
      } finally {
        panelsBusy.set(false);
      }
    });
  }

  /** Shown on the Panels Telemetry tab this loop, above the robot's own lines. */
  public void addPanelsLine(String key, String value) { panelsLines.add(new String[] {key, value}); }

  /** Everything shoot on the move worked out this loop, for AdvantageScope. */
  private void logAim(Aim aim) {
    Logger.recordOutput("SOTM/State", aimState.name());
    Logger.recordOutput("SOTM/Alliance", alliance.name());
    Logger.recordOutput("SOTM/KnowsCell", aim != null);
    Logger.recordOutput("SOTM/CellX", aimCellX);
    Logger.recordOutput("SOTM/CellY", aimCellY);
    Logger.recordOutput("SOTM/AimingAtFieldCell", aimingAtFieldCell);
    Logger.recordOutput("SOTM/UpCell", visionCorrector.upCell(alliance).name());
    Logger.recordOutput("SOTM/AimCell", aimCell().name());
    Logger.recordOutput("SOTM/TrackerCellX", tracker.getCellX()); // To compare with the field Cell
    Logger.recordOutput("SOTM/TrackerCellY", tracker.getCellY());
    // For AdvantageScope's field views: a line from the robot to the Cell it is aiming at, none when unknown
    Logger.recordOutput("SOTM/Cell", aim == null ? new Pose2d[0]
        : new Pose2d[] {new Pose2d(aimCellX, aimCellY, new Rotation2d())});
    Logger.recordOutput("SOTM/CellSeenAgoS", cellSeenAgoS);
    Logger.recordOutput("SOTM/RpmTrim", rpmTrim);
    Logger.recordOutput("SOTM/HeadingTrimDeg", headingTrimDeg);
    Logger.recordOutput("SOTM/TableRpmOffset", tableRpmOffset(aim));
    Logger.recordOutput("SOTM/HoodLiftTrimDeg", hoodLiftTrimDeg);
    Logger.recordOutput("Robot/OneBallShot", tuningMode && (driver.right_bumper || scriptedOneBall)
        && driver.right_trigger <= TRIGGER_PRESSED);
    Logger.recordOutput("Robot/TuningLive", tuningLive);
    Logger.recordOutput("Robot/PositionHold", isHolding());
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

  /** Every subsystem's state on the Driver Station: the tuning OpModes, or TELEMETRY_DETAIL from Panels. */
  public boolean isTelemetryDetail() {
    return tuningMode || ShootOnTheMoveConstants.TELEMETRY_DETAIL;
  }

  private void addTelemetry(Aim aim, Pose pose) {
    if (isTelemetryDetail()) {
      addDetailTelemetry(aim, pose);
    } else {
      addDriverTelemetry(aim, pose);
    }
  }

  /** The driver's lines: which shot, whether it's ready and what it waits for, and only the problems to act on. */
  private void addDriverTelemetry(Aim aim, Pose pose) {
    if (aim == null) {
      telemetry.addData("Shot", Double.isInfinite(cellSeenAgoS) ? "no Cell yet: point the cameras at the tags"
          : "Cell lost: point the cameras at the tags");
    } else {
      telemetry.addData("Shot", String.format("%s  %.2f m  %s", aimingAtFieldCell ? alliance + " " + aimCell()
          : "camera's Cell", aim.shot.distanceToLipM, aim.backward ? "BACKWARD" : "HOODED"));
      String state;
      if (stopHint != null) {
        state = stopHint;
      } else if (aimState == AimState.LINING_UP) {
        state = String.format("lining up  wheels %.0f/%.0f  hood %.1f/%.1f  turn %.0f deg", shooter.getRpm(),
            shooter.getTargetRpm(), HoodSubsystem.ticksToAngle(hood.getPosition()),
            HoodSubsystem.ticksToAngle(hood.getTarget()), Math.toDegrees(aim.headingErrorRad(pose.headingRad)));
      } else if (aimState == AimState.NO_SAFE_SHOT) {
        state = "no safe shot: " + aim.shot.reason;
      } else {
        state = aimState == AimState.OFF ? "RT to aim" : aimState.toString();
      }
      telemetry.addData("Aim", state + (rpmTrim != 0 ? String.format("  (trim %+.0f)", rpmTrim) : ""));
    }
    if (volleyMode && volleyShot) {
      telemetry.addData("Volley", "balls out %d", volleyFeedGate.ballsLeft());
    }
    if (!poseValid) {
      telemetry.addLine("! Pose not placed: point the cameras at the tags");
    }
    if (shooter.isStalled()) {
      telemetry.addLine("! Flywheels " + shooter.getStallStatus());
    }
    if (hood.hasEncoderFault()) {
      telemetry.addLine("! Hood " + hood.getEncoderFaultText());
    }
    String deadCameras = vision.deadCameras();
    if (deadCameras != null) {
      telemetry.addLine("! CAM " + deadCameras + " DEAD: fewer tag fixes");
    }
    telemetry.addData("Ramp", !ramp.isPresent() ? "! no " + RampSubsystem.SERVO_NAME + " in the configuration"
        : ramp.isDeployed() ? "DEPLOYED  (D-pad up stows)" : "stowed  (D-pad up deploys)");
  }

  private void addDetailTelemetry(Aim aim, Pose pose) {
    telemetry.addData("Alliance", alliance + (poseValid ? "  pose on field" : "  pose not placed yet")
        + (headingValid ? ", heading set" : ", heading from start"));
    telemetry.addData("Aim", aimState);
    if (volleyMode) {
      telemetry.addData("Volley", volleyShot ? String.format("balls out %d  (fed %d)  feed %.2f  boost %+.0f",
          volleyFeedGate.ballsLeft(), volleyFeedGate.feedStarts(), volleyFeedPower(), volleyBoostRpm())
          : "RB for a standing volley (RT shoots on the move; none without the Cell)");
    }
    if (aim == null) {
      telemetry.addData("Distance", Double.isInfinite(cellSeenAgoS)
          ? "Cell not seen yet: point the cameras at it"
          : String.format("Cell forgotten (seen %.0f s ago): point the cameras at it", cellSeenAgoS));
    } else {
      telemetry.addData("Distance", String.format("%.2f m to lip  %s  (%s)", aim.shot.distanceToLipM,
          aim.backward ? "BACKWARD, hood down" : "FRONT, hooded",
          aimingAtFieldCell ? alliance + " " + aimCell() + (ShootOnTheMoveConstants.AIM_AT_FRONT_CELL ? " (in front)" : " up, from tags")
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
        Math.toDegrees(pose.headingRad - driverForwardHeadingRad())));
    telemetry.addData("Vision", visionLine());
    telemetry.addData("Hood lift (deg)", "target %.1f, actual %.1f", hood.getTargetLiftDegrees(), hood.getLiftDegrees());
    telemetry.addData("Shooter", shooter.getShooterState());
    telemetry.addData("Shooter stall", shooter.getStallStatus());
    telemetry.addData("Hood encoders", hood.getEncoderFaultText());
    telemetry.addData("Intake", intake.getIntakeState());
    telemetry.addData("Intake overload", intake.getOverloadStatus());
    telemetry.addData("Indexer", indexer.getIndexerState());
    telemetry.addData("Feeder", feeder.getFeederState());
    telemetry.addData("Beam T/B", intake.isTopBroken() + " / " + intake.isBottomBroken()
        + (intake.isFull() ? "  FULL" : ""));
    telemetry.addData("Camera L", vision.describe(VisionSubsystem.Side.LEFT));
    telemetry.addData("Camera R", vision.describe(VisionSubsystem.Side.RIGHT));
  }

  /**
   * Power summary for the co-driver, first on the Driver Station (Main adds it before anything else): intake current
   * and how much of its overload budget is used, the flywheels' current, and the battery. The intake is the one that
   * overheats (13 Nov: 5-6 A for most of a match; ~9 A is a stall).
   */
  public void addPowerTelemetry() {
    String allianceWarning = allianceWarning();
    if (allianceWarning != null) {
      telemetry.addLine(allianceWarning);
    }
    double a = intake.getAmps();
    double load = intake.getLoadFraction();
    String intakeFlag = intake.isOverloaded() ? "TRIPPED - LB to restart"
        : a >= 7 || load >= 0.7 ? "!! HOT - let it rest"
        : a >= 5 || load >= 0.35 ? "! warm"
        : "ok";
    telemetry.addData("INTAKE", String.format("%.1f A  load %.0f%%  %s", a, 100 * load, intakeFlag));
    if (!isTelemetryDetail()) { // Battery on one line, its flag only when it needs doing something about
      String batteryFlag = batteryMinVolts < 11.0 ? "  !! LOW - swap" : batteryVolts < 12.0 ? "  ! low" : "";
      telemetry.addData("Battery", String.format("%.1f V%s%s", batteryVolts, batteryFlag,
          shooter.isStalled() ? "   FLYWHEELS STALLED" : ""));
      return;
    }
    String shooterFlag = shooter.isStalled() ? "STALLED" : "ok";
    telemetry.addData("FLYWHEELS", String.format("L %.1f  R %.1f A  %s",
        shooter.getLeftAmps(), shooter.getRightAmps(), shooterFlag));
    String batteryFlag = batteryMinVolts < 11.0 ? "!! LOW - swap" : batteryVolts < 12.0 ? "! low" : "ok";
    telemetry.addData("BATTERY", String.format("%.2f V  (lowest %.2f)  %s", batteryVolts, batteryMinVolts,
        batteryFlag));
  }

  /**
   * Samples the battery voltage, keeping the lowest seen this run (it sags when the shooter spins up),
   * then logs it and adds it to telemetry.
   */
  /**
   * A battery this low at rest sags toward brownout in a match: two 9 Nov runs that started at 11.1-11.9 V fell to
   * 6.8-6.9 V, and every auto sags to 9.6 V at spin-up even from 12.6 V.
   */
  static boolean swapBattery(double restingVolts) {
    return restingVolts < ShootOnTheMoveConstants.SWAP_BATTERY_V;
  }

  private void updateBattery(boolean telemetryDue) {
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
    if (telemetryDue && isTelemetryDetail()) {
      telemetry.addData("Battery", "%.2f V (lowest %.2f V)", batteryVolts, batteryMinVolts);
    }
  }

  /** Called once when the OpMode stops. */
  /** The pose this loop aimed from, after the vision correction. */
  public Pose getPose() { return lastPose; }
  /** Field velocity {x, y, turn rate} this loop, m/s and rad/s. */
  public double[] getVelocity() { return lastVelocity.clone(); }
  public boolean isPoseValid() { return poseValid; }
  public double getBatteryVolts() { return batteryVolts; }
  public double getBatteryMinVolts() { return batteryMinVolts; }
  public int getOutOfFieldFrames() { return outOfFieldFrames; }
  public int getHeadingOffFrames() { return headingOffFrames; }
  public int getPositionFixes() { return positionFixes; }
  public double getLastHeadingOffRad() { return lastHeadingOffRad; }
  public boolean isHeadingValid() { return headingValid; }
  public boolean isAimingAtFieldCell() { return aimingAtFieldCell; }
  public double getAimCellX() { return aimCellX; }
  public double getAimCellY() { return aimCellY; }
  public AimState getAimState() { return aimState; }
  public boolean isFeeding() { return lastFeeding; }
  /** Aims as RT asks but feeds nothing: a scripted test lining up before the shot it means to take. */
  public void setHoldFire(boolean hold) { holdFire = hold; }

  /** Heading/hood may track while filling, but flywheels start only for an enabled shot. */
  public void setSpinOnlyOnFire(boolean enabled) { spinOnlyOnFire = enabled; }

  /** The shortest time from one aimed feed starting to the next. */
  public void setMinFeedSpacingS(double s) { aimedFeedGate.setMinFeedSpacingS(s); }

  private double tableRpmOffset(Aim aim) {
    if (aim == null || (tuningMode && !tuningLive)) {
      return 0;
    }
    double toward = finiteOr0(aim.shot.speedTowardCellMps);
    return aim.backward ? ShootOnTheMoveConstants.SOTM_BACKWARD_RPM_PER_TOWARD_MPS * toward
        : ShootOnTheMoveConstants.HOODED_RPM_OFFSET + ShootOnTheMoveConstants.SOTM_RPM_PER_TOWARD_MPS * toward;
  }

  /** Launch angle added to a hooded shot for its speed toward the Cell. */
  private double movingHoodDeg(Aim aim) {
    if (aim == null || aim.backward || (tuningMode && !tuningLive)) {
      return 0;
    }
    double towardMps = finiteOr0(aim.shot.speedTowardCellMps);
    return ShootOnTheMoveConstants.SOTM_HOOD_DEG_PER_TOWARD_MPS * towardMps + (tuningMode ? 0
        : movingHoodTrimDeg(aim.shot.distanceToLipM, Math.hypot(towardMps, finiteOr0(aim.shot.sidewaysSpeedMps))));
  }

  /**
   * TeleOp's launch angle added to a moving hooded shot for its distance to the lip
   * ({@link ShootOnTheMoveConstants#SOTM_HOOD_TRIM_DEG}): full from the moving shot speed, scaled down to none at rest.
   */
  static double movingHoodTrimDeg(double lipM, double speedMps) {
    double[] d = ShootOnTheMoveConstants.SOTM_HOOD_TRIM_DIST_M;
    double[] t = ShootOnTheMoveConstants.SOTM_HOOD_TRIM_DEG;
    if (Double.isNaN(lipM) || Double.isNaN(speedMps)) {
      return 0;
    }
    double trim;
    if (lipM <= d[0]) {
      trim = t[0];
    } else if (lipM >= d[d.length - 1]) {
      trim = t[t.length - 1];
    } else {
      int i = 0;
      while (lipM > d[i + 1]) { i++; }
      trim = t[i] + (t[i + 1] - t[i]) * (lipM - d[i]) / (d[i + 1] - d[i]);
    }
    return trim * Math.min(1, Math.abs(speedMps) / ShootOnTheMoveConstants.MOVING_SHOT_SPEED_MPS);
  }

  /**
   * Aim trim, degrees anticlockwise: the tuning tools' own (toolTrimDeg), else the standing trim they found for the
   * hooded or backward shot.
   */
  static double aimTrimDeg(boolean tuningMode, double toolTrimDeg, boolean backward) {
    if (tuningMode) {
      return toolTrimDeg;
    }
    return backward ? ShootOnTheMoveConstants.BACKWARD_HEADING_TRIM_DEG
        : ShootOnTheMoveConstants.HOODED_HEADING_TRIM_DEG;
  }

  /** Aim turned anticlockwise for the shot's sideways speed. */
  private double movingHeadingDeg(Aim aim) {
    if (aim == null || (tuningMode && !tuningLive)) {
      return 0;
    }
    return (aim.backward ? ShootOnTheMoveConstants.SOTM_BACKWARD_HEADING_DEG_PER_SIDEWAYS_MPS
        : ShootOnTheMoveConstants.SOTM_HEADING_DEG_PER_SIDEWAYS_MPS) * finiteOr0(aim.shot.sidewaysSpeedMps);
  }

  private static double finiteOr0(double v) {
    return Double.isNaN(v) || Double.isInfinite(v) ? 0 : v;
  }

  /**
   * Shot Tuning: every shot is the one set by {@link #setTuningShot}, from anywhere, and D-pad, left bumper, X and
   * Y are left to the OpMode.
   */
  public void setTuningMode(boolean on) { tuningMode = on; }

  /** @param hoodDeg launch angle; HoodSubsystem.ZERO_ANGLE_DEG is the hood on its lower stop */
  public void setTuningShot(boolean backward, double rpm, double hoodDeg) {
    tuningBackward = backward;
    tuningRpm = rpm;
    tuningHoodDeg = hoodDeg;
  }

  /**
   * Shot Tuning: the tables' shot for the direction set by {@link #setTuningShot}, with the moving corrections and
   * the usual gate, instead of the fixed RPM and hood. For shots taken on the move.
   */
  public void setTuningLive(boolean on) { tuningLive = on; }

  /** Tuning only: fire without waiting for a steady stick and low acceleration, to measure shots fired mid-change. */
  public void setSteadyGateBypassed(boolean on) { steadyGateBypassed = on; }

  /** Tuning only: fire moving close to the lip, which the match's gate refuses, to measure whether those shots score. */
  public void setCloseMovingGateBypassed(boolean on) { closeMovingGateBypassed = on; }

  /**
   * Tuning only: the steady gate judges the stick by this field stick (a scripted pass's planned speed) instead of
   * the scripted drive, whose tracking corrections change every loop and so never read as held; null for the drive.
   */
  public void setSteadyStickReference(double[] fieldStick) {
    steadyStickReference = fieldStick == null ? null : fieldStick.clone();
  }

  /** Shot Tuning: fires one ball once ready, as holding RB does. Release before asking for the next. */
  public void setScriptedOneBall(boolean on) { scriptedOneBall = on; }

  /** Autonomous: aims and fires the whole load as a volley once ready, as holding RB does. */
  public void setScriptedVolley(boolean on) { scriptedVolley = on; }

  /** This loop's aim, null with the Cell unknown. */
  public Aim getLastAim() { return lastAim; }

  /**
   * Drives as if the left stick were pushed this far along the field's +x and +y (each -1..1), instead of reading
   * it; null hands it back. The turn still comes from the right stick, or the aim.
   */
  public void setDriveOverride(double[] fieldStick) { driveOverride = fieldStick; }

  /** Brakes the wheels at zero power (on) instead of letting them roll; teleop keeps them rolling. */
  public void setDriveBrake(boolean on) { drive.setBrake(on); }

  /** Turns to the shot and keeps the wheels and hood on it, as RB does, but fires nothing. */
  public void setForceAim(boolean on) { forceAim = on; }

  /**
   * Holds the robot on this field point {x, y}, as RB holds it in Lebot Teleop, while no drive is scripted;
   * null lets it go. Unlike RB's spot it stays put on the field when the tags move the pose.
   */
  public void setHoldPoint(double[] point) { holdPoint = point == null ? null : point.clone(); }

  /** The driver's drive and turn sticks do nothing while on; scripted drives and the aim still move the robot. */
  public void setSticksLocked(boolean on) { sticksLocked = on; }

  /** With the sticks locked and nothing aiming, turns as the turn stick pushed this far (-1..1) would. */
  public void setTurnOverride(double turn) { turnOverride = turn; }

  /** Seconds since the cameras last saw a tag; infinite if never. */
  public double getTagsSeenAgoS() {
    return lastVisionNanos == 0 ? Double.POSITIVE_INFINITY : (System.nanoTime() - lastVisionNanos) * 1e-9;
  }

  /** Switches to the other alliance when the cameras see its Cell while this alliance's Cell isn't known. */
  public void setAllianceFromTags(boolean on) { allianceFromTags = on; }

  /** Aims at this field point {x, y} instead of the Cell, once the tags have placed the robot; null for the Cell. */
  public void setAimPointOverride(double[] point) { aimPointOverride = point == null ? null : point.clone(); }

  /** Which end of our alliance's Hive is up, as the tags last showed it (the starting guess until they have). */
  public VisionCorrector.Cell getUpCell() { return visionCorrector.upCell(alliance); }

  public boolean isUpCellConfirmed() { return visionCorrector.upCellConfirmed(alliance); }

  /** True on the loop an aimed feed starts a ball. */
  public boolean feedStartedThisLoop() { return feedGate.feedStarted(); }

  /** True on the loop the wheels showed a ball leaving (aimed or volley). */
  public boolean ballLeftThisLoop() { return feedGate.ballLeft(); }

  /** True once a feed pulse and its pause passed with no ball out, until one leaves. */
  /** The Pinpoint's last read was good: false while it calibrates after the reset INIT asks for. */
  public boolean isOdometryReady() { return odometry.isReadGood(); }

  public boolean isFeederEmpty() { return feedGate.feederEmpty(); }

  /** The stack read full this loop (a loading ball touched the wheels): the auto moves on from a Flower. */
  public boolean isStackFull() { return wasFull; }

  /** Frames so far whose tags voted for our up Cell as it stands. */
  public int getUpCellAgreeingFrames() { return visionCorrector.agreeingFrames(alliance); }

  /** Powers for the Scoring Auto's drive, from this loop's pose (vision corrected) and field-frame velocity. */
  /** LT, or held fully down while the autonomous is loading: in auto the driver gamepad is a blank one. */
  static double loadTrigger(double leftTrigger, boolean autoIntake) {
    return autoIntake ? 1.0 : leftTrigger;
  }

  /**
   * RT, or held fully down while the autonomous shoots. 13 Nov 224556: with only the aim taking autoShoot, the
   * indexer never staged or fed (shotRequested read the blank gamepad's RT), and the auto shot nothing in 29 s.
   */
  private double fireTrigger() {
    return loadTrigger(driver.right_trigger, autoShoot);
  }

  public interface AutoDriver {
    /** @return robot-frame {forward, left, counterclockwise}, each -1..1; null to stand still, not even aiming */
    double[] drive(Pose pose, double[] velocity);
  }

  /**
   * Scoring Auto: from now on no gamepad is read (G401), and nothing fires without an aim. Call before INIT's loop,
   * so the alliance can't be changed from a gamepad either.
   */
  public void setAutonomous(boolean on) { autonomous = on; }

  /** Aims and shoots as RT does, while true. */
  public void setAutoShoot(boolean on) { autoShoot = on; }

  /** An autonomous: on stop, its pose and alliance are left for the TeleOp after it ({@link PoseHandoff}). */
  public void setHandsOffPose(boolean on) { handsOffPose = on; }

  /**
   * TeleOp: starts from the auto's pose if it just stopped, else as {@link #start}. Returns whether it did.
   * The pose counts as placed, so field-centric driving and the aim are right from the first loop.
   */
  public boolean startFromAuto() {
    Pose handoff = PoseHandoff.take(System.nanoTime());
    if (handoff == null) {
      start();
      startedFromAuto = false;
    } else {
      startAt(handoff.xM, handoff.yM, handoff.headingRad);
      startedFromAuto = true;
    }
    Logger.recordOutput("Start/FromAuto", startedFromAuto);
    return startedFromAuto;
  }

  /** Intakes as LT does, while true. Shooting wins. */
  public void setAutoIntake(boolean on) { autoIntake = on; }

  /** Drives from these powers instead of the sticks; null hands the drive back. */
  public void setAutoDriver(AutoDriver driver) { autoDriver = driver; }

  /** The auto's drive turns with the heading lock to this field heading, not Pedro's turn; NaN gives it back. */
  public void setAutoHeading(double headingRad) { setAutoHeading(headingRad, 0.0); }

  /** As above, for a heading that moves at this rate (CCW positive), as the Flower wiggle. */
  public void setAutoHeading(double headingRad, double rateRadPerSec) {
    autoHeadingRad = headingRad;
    autoHeadingRateRadPerSec = rateRadPerSec;
  }

  /**
   * As {@link #start}, but from a known pose instead of the placed heading: the Scoring Auto's measured start spot.
   * The pose counts as placed, so the robot aims before the tags are seen; they then only trim it.
   */
  public void startAt(double xM, double yM, double headingRad) {
    start();
    odometry.setPose(xM, yM, headingRad);
    poseValid = true;
    headingValid = true;
    lastPose = new Pose(xM, yM, headingRad);
  }

  /** Gamepad 1 merged with the Panels gamepad, as this loop's controls read it. */
  public Gamepad getDriver() { return driver; }

  /** Adds this to every aimed shot's RPM, as the driver's trim does. */
  public void setRpmTrim(double rpm) { rpmTrim = rpm; }

  public void setHeadingTrimDeg(double deg) { headingTrimDeg = deg; }


  /** Raises every hooded shot's launch angle this much, as the driver's hood trim does (lift falls). */
  public void setHoodAngleTrimDeg(double deg) { hoodLiftTrimDeg = -deg; }

  /** The longest an aimed feed runs before pausing for its ball's dip. */
  public void setFeedPulseS(double s) { aimedFeedGate.setPulseS(s); }

  // ---- Volley (Lebot Teleop RB, Volley Tuning) ----

  /** RB's hooded aimed shot, held still, fires as a burst; RT's on the move too while VOLLEY_ON_RT. */
  public void setVolleyMode(boolean on) { volleyMode = on; }

  /**
   * A volley is RB's (it wins when both are held: the robot is holding still), the aim a tool forces without RT
   * (Volley Tuning, so the hood and wheels are already at the volley's before RB), or RT's on the move when
   * {@code rtVolley}. Volley Tuning's RT stays a one-at-a-time shot.
   */
  static boolean volleyRequested(boolean rightBumper, double rightTrigger, boolean forceAim, boolean tuningMode,
      boolean rtVolley) {
    if (tuningMode) { return false; }
    if (rightBumper) { return true; }
    return forceAim ? rightTrigger <= TRIGGER_PRESSED : rtVolley && rightTrigger > TRIGGER_PRESSED;
  }

  public boolean isVolleyMode() { return volleyMode; }

  /** Volley calibration measures the hooded shot at every test spot. */
  public void setHoodedOnly(boolean on) { hoodedOnly = on; }

  /** A, B, X, Y and the D-pad do nothing in the robot, so an OpMode can use them (Volley Tuning's marks). */
  public void setButtonsReserved(boolean on) { buttonsReserved = on; }

  /** Launch angle added to a volley's first ball and to balls 2-4; NaN for the constants'. */
  public void setVolleyTrimsDeg(double firstDeg, double laterDeg) {
    volleyFirstTrimDeg = firstDeg;
    volleyLaterTrimDeg = laterDeg;
  }

  /** Feeder and indexer power during a volley; NaN for VOLLEY_FEED_POWER. */
  public void setVolleyFeedPower(double power) { volleyFeedPower = power; }

  /** Separate queue pressure for calibration; NaN follows the feeder as normal Volley TeleOp does. */
  public void setVolleyIndexerPower(double power) { volleyIndexerPower = power; }

  /** Automatic calibration's aim correction, in addition to the standing shot trim. */
  public void setVolleyHeadingTrimDeg(double deg) { volleyHeadingTrimDeg = deg; }

  public double volleyIndexerPower() {
    return Double.isNaN(volleyIndexerPower)
        ? volleyFeedPower() * ShootOnTheMoveConstants.VOLLEY_INDEXER_SCALE : volleyIndexerPower;
  }

  /** RPM over the table's for a volley; NaN for VOLLEY_BOOST_RPM. */
  public void setVolleyBoostRpm(double rpm) { volleyBoostRpm = rpm; }

  /** Full-power recovery between a volley's balls; null for VOLLEY_FAST_RECOVERY. */
  public void setVolleyFastRecovery(Boolean on) { volleyFastRecovery = on; }

  /** This loop's shot is a volley: volley mode, RB or RT, an aimed shot (backward too), not a tuning tool's. */
  public boolean isVolleyShot() { return volleyShot; }

  /** Feeds started in this press of the volley trigger. */
  public int getVolleyFeedStarts() { return volleyFeedGate.feedStarts(); }

  /** Balls seen leaving (wheel dips) in this press of the volley trigger. */
  public int getVolleyBallsLeft() { return volleyFeedGate.ballsLeft(); }

  /** About the wheel speed the last volley ball left at; NaN if none. */
  public double getVolleyLeftAtRpm() { return volleyFeedGate.peakAtLeaveRpm(); }

  /** How far the wheels have dropped since the last volley ball left; NaN if none. */
  public double getVolleyDipDepthRpm() { return volleyFeedGate.lastDipDepthRpm(); }

  /** Launch angle the volley added to the hood this loop. */
  public double getVolleyHoodDeg() { return volleyHoodDeg; }

  public double volleyFeedPower() {
    return Double.isNaN(volleyFeedPower) ? ShootOnTheMoveConstants.VOLLEY_FEED_POWER : volleyFeedPower;
  }

  public double volleyBoostRpm() {
    return Double.isNaN(volleyBoostRpm) ? ShootOnTheMoveConstants.VOLLEY_BOOST_RPM : volleyBoostRpm;
  }

  public boolean volleyFastRecovery() {
    return volleyFastRecovery != null ? volleyFastRecovery : ShootOnTheMoveConstants.VOLLEY_FAST_RECOVERY;
  }

  private boolean buttonsFree() {
    return !tuningMode && !buttonsReserved;
  }

  private boolean volleyAim(Aim aim) {
    return volleyMode && volleyRequested && aim != null
        && (!aim.backward || ShootOnTheMoveConstants.VOLLEY_BACKWARD_ON);
  }

  private double volleyBoostRpm(Aim aim) {
    return volleyAim(aim) && !aim.backward ? volleyBoostRpm() : 0;
  }

  /** Top of a volley's matched band: tighter for a backward burst, which goes long when fed fast. */
  private static double volleyReadyAboveRpm(Aim aim) {
    return aim != null && aim.backward ? ShootOnTheMoveConstants.VOLLEY_BACKWARD_READY_ABOVE_RPM
        : ShootOnTheMoveConstants.VOLLEY_READY_ABOVE_RPM;
  }

  /**
   * Launch angle a volley adds: the steepening for its boost (ShotTableRpm), a share of the steepening for the
   * wheels' predicted speed off the target (VOLLEY_HOOD_FOLLOW_GAIN), and the trim for the ball about to go.
   */
  private double volleyHoodDeg(Aim aim) {
    if (!volleyAim(aim)) {
      return 0;
    }
    double boost = volleyBoostRpm();
    double lipM = aim.shot.distanceToLipM;
    double towardMps = finiteOr0(aim.shot.speedTowardCellMps);
    double boostDeg = ShootOnTheMove.hoodOffsetForRpmAbove(lipM, towardMps, boost);
    double followDeg = 0;
    double gain = ShootOnTheMoveConstants.VOLLEY_HOOD_FOLLOW_GAIN;
    if (gain != 0) {
      double above = rpmTrend.predicted(ShootOnTheMoveConstants.RPM_LOOKAHEAD_S) - shooter.getTargetRpm() + boost;
      above = Math.max(boost - ShootOnTheMoveConstants.VOLLEY_READY_BELOW_RPM,
          Math.min(boost + ShootOnTheMoveConstants.VOLLEY_READY_ABOVE_RPM, above));
      followDeg = gain * (ShootOnTheMove.hoodOffsetForRpmAbove(lipM, towardMps, above) - boostDeg);
    }
    boolean first = volleyFeedGate.ballsLeft() == 0;
    double trim = first
        ? (Double.isNaN(volleyFirstTrimDeg) ? ShootOnTheMoveConstants.VOLLEY_FIRST_BALL_TRIM_DEG : volleyFirstTrimDeg)
        : (Double.isNaN(volleyLaterTrimDeg) ? ShootOnTheMoveConstants.VOLLEY_LATER_BALL_TRIM_DEG : volleyLaterTrimDeg);
    return finiteOr0(boostDeg + followDeg) + trim;
  }

  private void logVolley() {
    Logger.recordOutput("Volley/Shot", volleyShot);
    Logger.recordOutput("Volley/FeedStarts", volleyFeedGate.feedStarts());
    Logger.recordOutput("Volley/BallsLeft", volleyFeedGate.ballsLeft());
    Logger.recordOutput("Volley/LeftAtRpm", volleyFeedGate.peakAtLeaveRpm());
    Logger.recordOutput("Volley/DipDepthRpm", volleyFeedGate.lastDipDepthRpm());
    Logger.recordOutput("Volley/DoubleBall",
        volleyFeedGate.lastDipDepthRpm() > ShootOnTheMoveConstants.VOLLEY_DOUBLE_DIP_RPM);
    Logger.recordOutput("Volley/HoodDeg", volleyHoodDeg);
    Logger.recordOutput("Volley/BoostRpm", volleyBoostRpm());
    Logger.recordOutput("Volley/FeedPower", volleyFeedPower());
    Logger.recordOutput("Volley/IndexerPower", volleyIndexerPower());
    Logger.recordOutput("Volley/FastRecovery", volleyFastRecovery());
  }

  public void stop() {
    watchdog.feed(); // A stop that hangs (closing a camera) is reported too
    if (handsOffPose) {
      PoseHandoff.save(isPoseKnown() && !odometry.isFaulted() ? lastPose : null, alliance, System.nanoTime());
    }
    fieldSender.shutdownNow();
    hood.disable(); // Its loop thread would otherwise hold the hood until it misses the heartbeat
    vision.close();
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().reset();
    watchdog.stop();
  }
}
