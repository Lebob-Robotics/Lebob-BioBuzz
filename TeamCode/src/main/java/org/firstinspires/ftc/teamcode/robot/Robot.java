package org.firstinspires.ftc.teamcode.robot;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.field.Angles;
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
import com.bylazar.gamepad.PanelsGamepad;
import org.firstinspires.ftc.teamcode.shooting.Aim;
import org.firstinspires.ftc.teamcode.shooting.feed.ManualFeedGate;
import org.firstinspires.ftc.teamcode.field.PoseHandoff;
import org.firstinspires.ftc.teamcode.shooting.feed.StackPressure;
import org.firstinspires.ftc.teamcode.shooting.feed.WheelContact;
import org.firstinspires.ftc.teamcode.shooting.feed.LoadingShooterInterlock;
import org.firstinspires.ftc.teamcode.control.HoodStepControl;
import org.firstinspires.ftc.teamcode.control.HeadingHold;
import org.firstinspires.ftc.teamcode.control.LoopWatchdog;
import org.firstinspires.ftc.teamcode.vision.CellSighting;
import org.firstinspires.ftc.teamcode.field.FieldPose;
import org.firstinspires.ftc.teamcode.field.PoseHistory;
import org.firstinspires.ftc.teamcode.shooting.feed.RpmTrend;
import org.firstinspires.ftc.teamcode.shooting.feed.OneBallFeedGate;
import org.firstinspires.ftc.teamcode.shooting.feed.IndexerRetry;
import org.firstinspires.ftc.teamcode.shooting.Shot;
import org.firstinspires.ftc.teamcode.shooting.ShootOnTheMove;
import org.firstinspires.ftc.teamcode.shooting.AimConstants;
import org.firstinspires.ftc.teamcode.shooting.BackwardShotConstants;
import org.firstinspires.ftc.teamcode.shooting.FireConstants;
import org.firstinspires.ftc.teamcode.shooting.ShotPickConstants;
import org.firstinspires.ftc.teamcode.shooting.SotmConstants;
import org.firstinspires.ftc.teamcode.shooting.volley.VolleyConstants;
import org.firstinspires.ftc.teamcode.vision.VisionConstants;
import org.firstinspires.ftc.teamcode.shooting.ShooterTuning;
import org.firstinspires.ftc.teamcode.vision.TagObservation;
import org.firstinspires.ftc.teamcode.vision.TargetTracker;
import org.firstinspires.ftc.teamcode.vision.VisionCorrector;
import org.firstinspires.ftc.teamcode.vision.VisionResult;
import org.psilynx.psikit.core.Logger;

import java.util.List;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.opmodes.Main;

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
  private final RobotTelemetry display;
  private final RobotLogger log = new RobotLogger(this);
  private final RobotLogger.LoopTimes times = new RobotLogger.LoopTimes();
  /** The real gamepad 1, kept for rumble. */
  private final Gamepad physicalDriver;
  /** What the code reads: gamepad 1, plus the browser gamepad from Panels when Panels is on. Rebuilt each loop. */
  Gamepad driver;

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
  final ElapsedTime clock = new ElapsedTime();
  private double lastBatterySampleS = Double.NEGATIVE_INFINITY;
  double batteryVolts = Double.NaN;
  double batteryMinVolts = Double.NaN;

  /** Where shoot on the move is up to. Shown to the driver and logged. */
  public enum AimState {
    OFF, // Right trigger not held
    CELL_NOT_SEEN,
    NO_SAFE_SHOT,
    LINING_UP, // Valid shot, waiting for the flywheels, hood or heading
    READY // A ball fed now should score
  }

  Alliance alliance = Alliance.BLUE;
  private boolean driverOnFarSide = false;
  final TargetTracker tracker = new TargetTracker(VisionConstants.POSE_HISTORY_SIZE);
  // Vision pose correction. The history holds each loop's odometry pose with the time of that loop's
  // camera capture, so a frame can be matched to where the robot was. It is separate from the tracker's.
  private final PoseHistory poseHistory = new PoseHistory(VisionConstants.POSE_HISTORY_SIZE);
  final VisionCorrector visionCorrector = new VisionCorrector();
  private long lastHistoryNanos = Long.MIN_VALUE;
  boolean poseValid = false; // True once a tag fix has placed the robot on the field
  // True once two tags have set the heading, or the driver reset it with A. Until then a heading fix may be any size
  boolean headingValid = false;
  private Cell frontCell = null; // Our Cell on the robot's side of the Hive
  private boolean poseSeeded = false; // Shoot Auto's assumed start pose: aimed from until vision places the robot
  VisionResult lastVision = null; // Last update that looked at tags, for telemetry
  long lastVisionNanos = 0;
  private int positionFixes; // Tag fixes applied to the position so far

  AimState aimState = AimState.OFF;
  private double aimRpm = DriverConstants.AIM_START_RPM; // Last valid shot's, so a brief invalid shot doesn't drop it
  private final HoodStepControl hoodSteps = new HoodStepControl();
  double hoodLiftTrimDeg = 0;
  double rpmTrim = 0; // Driver's, for the rest of the match
  double headingTrimDeg = 0; // Anticlockwise, the tuning tools' aim trim (TeleOp uses aimTrimDeg's)
  // Shot Tuning: the driver's RPM, hood and direction replace the tables', and the buttons they use do nothing else
  boolean tuningMode = false;
  private static final double TUNING_HEADING_TOLERANCE_DEG = 1.5;
  private boolean tuningBackward = false;
  private double tuningRpm = 3400;
  private double tuningHoodDeg = HoodSubsystem.ZERO_ANGLE_DEG;
  boolean tuningLive = false; // Shot Tuning's moving pass: the tables' shot, moving corrections and all
  private boolean steadyGateBypassed = false; // SOTM Tuning's passes that fire mid-change
  private boolean closeMovingGateBypassed = false; // SOTM Tuning's passes that fire moving close to the lip
  private double[] steadyStickReference; // SOTM Tuning: the plan's stick size, without its tracking corrections
  boolean scriptedOneBall = false; // As RB held in Shot Tuning, from a script
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
  private final IndexerRetry indexerRetry = new IndexerRetry();
  private final ManualFeedGate manualFeedGate = new ManualFeedGate();
  private final LoopWatchdog watchdog = new LoopWatchdog();
  // An aimed shot feeds one ball at a time, each once the wheels are back up to speed
  private final OneBallFeedGate aimedFeedGate = new OneBallFeedGate(
      FireConstants.AIMED_READY_S, FireConstants.AIMED_FEED_STOP_UNREADY_S,
      FireConstants.AIMED_FEED_PULSE_S, FireConstants.AIMED_FEED_PAUSE_S,
      FireConstants.MIN_BALL_GAP_S);
  // Volley mode (Lebot Teleop, Volley Tuning): RB's hooded aimed shot, held still, fires as a burst, each ball at
  // the matched wheel speed (VolleyConstants). RT stays the one-at-a-time shoot on the move. Off,
  // RB is a plain held shot as RT's
  boolean volleyMode = false;
  private boolean volleyRequested = false; // This loop asks for a volley: RB, Volley Tuning's aim, or RT (VOLLEY_ON_RT)
  private boolean hoodedOnly = false;
  private boolean backwardOnly = false;
  final OneBallFeedGate volleyFeedGate
 = new OneBallFeedGate(0.0,
      FireConstants.AIMED_FEED_STOP_UNREADY_S, VolleyConstants.VOLLEY_FEED_PULSE_S,
      VolleyConstants.VOLLEY_FEED_PAUSE_S,
      VolleyConstants.VOLLEY_MIN_GAP_S, true);
  private OneBallFeedGate feedGate = aimedFeedGate; // The one feeding this loop
  boolean volleyShot = false; // This loop's shot is a volley
  // Volley Tuning's settings in place of the constants
  private final VolleyConfig volley = new VolleyConfig();

  double volleyHoodDeg = 0; // Launch angle the volley added this loop, for the log
  // Volley Tuning marks balls with A, B, X, Y and the D-pad, so they do nothing else (heading reset, eject, stop,
  // full power, trims, presets). The intake and the triggers work as in Lebot Teleop
  private boolean buttonsReserved = false;
  double cellSeenAgoS = Double.POSITIVE_INFINITY;
  boolean aimingAtFieldCell = false; // At the up Cell's field position, not the tracker's sighted one
  double aimCellX = 0; // The point aimed at this loop, field metres
  double aimCellY = 0;
  boolean intakeOn = true;
  /** Shot direction picked when RT was pressed: true backward, false hooded, null not aiming. */
  private Boolean aimBackwardLatched = null;
  private double backwardBlockedSinceS = Double.NaN;
  // Rate the aimed shot's sideways turn changes, smoothed, for the fire check and the heading lock
  private double turningRateRadPerSec = 0.0;
  private final RpmTrend rpmTrend = new RpmTrend(FireConstants.RPM_TREND_WINDOW_S);
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
  String stopHint; // What the driver must change for the shot to fire; null when it's not how they drive
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
  private FieldPose lastPose = new FieldPose(0, 0, 0);
  private double[] lastVelocity = {0, 0, 0};
  private double[] aimVelocity; // Low-passed {x, y} the aim uses; null until the first loop
  private double aimVelocityS;
  private double feedCommitUntilS = Double.NEGATIVE_INFINITY;
  private double[] committedTranslation; // Held while a fed ball is still in the shooter; null otherwise
  private double committedSinceS;
  private double[] lastTranslation = {0.0, 0.0};
  // Scoring Auto (6 Nov, feature/auto): no gamepad is read (G401); it drives, aims and intakes through these
  private boolean autonomous = false;
  private boolean autoIndexerPaused = false; // The auto's agitation: a loading indexer stops while true
  private boolean autoIndexerReverse = false; // ...or runs backward, a jiggle like the volley's retry pulse
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

  boolean mechanismsWaitForStart;

  public Robot(HardwareMap hardwareMap, Telemetry telemetry, Gamepad driver) {
    this.telemetry = telemetry;
    this.display = new RobotTelemetry(this, telemetry);
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
          initMaxVolts, DriverConstants.SWAP_BATTERY_V));
      if (!swapBatteryRumbled) {
        physicalDriver.rumble(600);
        swapBatteryRumbled = true;
      }
    }
    noteHiveTags(vision.takeNewTagObservations());
    display.addInitTelemetry();
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
    times.start = System.nanoTime();
    watchdog.feed();
    refreshDriver();
    Pose2D pinpointPose = odometry.getPose(); // PsiKit has already updated the Pinpoint this loop
    FieldPose pose = new FieldPose(pinpointPose.getX(DistanceUnit.METER), pinpointPose.getY(DistanceUnit.METER),
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
      pose = new FieldPose(pose.xM, pose.yM, placedHeadingRad());
    }

    times.vision = System.nanoTime();
    if (odometry.isReadGood()) { // A correction from a bad read would write that read back into the Pinpoint
      FieldPose uncorrected = pose;
      pose = correctPoseFromVision(pose, velocity, nowNanos); // Before aiming, so this loop aims from the corrected pose
      // The robot didn't move: only where it thinks it is did, so the held spot moves with it
      holdAnchorX += pose.xM - uncorrected.xM;
      holdAnchorY += pose.yM - uncorrected.yM;
    }

    double fireTrigger = fireTrigger();
    boolean holdShot = driver.right_bumper && !tuningMode;
    volleyRequested = volleyRequested(driver.right_bumper || scriptedVolley, fireTrigger, forceAim,
        tuningMode, VolleyConstants.VOLLEY_ON_RT);
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
    times.aim = System.nanoTime();
    Aim aim = aimAtCell(pose, velocity, nowNanos, aiming);
    times.aimed = System.nanoTime();
    double turnDeg = aimTrimDeg(tuningMode, headingTrimDeg, aim != null && aim.backward) + movingHeadingDeg(aim);
    if (volleyAim(aim) && !Double.isNaN(volley.headingTrimDeg())) { turnDeg += volley.headingTrimDeg(); }
    if (aim != null && turnDeg != 0) {
      aim = aim.turnedBy(Math.toRadians(turnDeg));
    }
    lastAim = aim;
    double lockRateRadPerSec = updateLockRate(aiming, aim);
    updateTrim(aiming);
    updateToggles(aiming);
    // Before the ready check, so it compares the hood and wheels with this loop's shot. After it, a hood target
    // that had just moved 3 degrees (the robot stopped) was judged against the old one, and the ball missed (5 Nov, 095605)
    times.shooter = System.nanoTime();
    volleyShot = volleyAim(aim);
    feedGate = volleyShot ? volleyFeedGate : aimedFeedGate;
    loadingShooter.update(clock.seconds(), spinOnlyOnFire, shotRequested() && !holdFire,
        shooter.getMaxMeasuredRpm());
    Logger.recordOutput("Shooter/Loading/State", loadingShooter.state());
    Logger.recordOutput("Shooter/Loading/SpinBlocked", loadingShooter.blocksSpin());
    Logger.recordOutput("Shooter/Loading/CanLoad", loadingShooter.canLoad());
    updateShooterAndHood(aiming, aim);
    times.shooterSet = System.nanoTime();
    rpmTrend.update(clock.seconds(), shooter.getRpm());
    double predictedRpmError = rpmTrend.predicted(FireConstants.RPM_LOOKAHEAD_S) - shooter.getTargetRpm();
    Logger.recordOutput("Shooter/PredictedRpmError", predictedRpmError);
    boolean flywheelsReady = shooter.atSpeed(FireConstants.RPM_TOLERANCE)
        && shooter.meanRpmWithin(FireConstants.RPM_MEAN_BELOW_TARGET,
            FireConstants.RPM_MEAN_ABOVE_TARGET)
        && predictedRpmError <= FireConstants.RPM_PREDICTED_ABOVE_TARGET;
    if (volleyShot) {
      // Every ball at the same speed, so their flights match and none catches the one ahead
      flywheelsReady = shooter.atSpeed(FireConstants.RPM_TOLERANCE)
          && shooter.meanRpmWithin(VolleyConstants.VOLLEY_READY_BELOW_RPM, volleyReadyAboveRpm(aim))
          && predictedRpmError >= -VolleyConstants.VOLLEY_READY_BELOW_RPM
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
        || Math.abs(Angles.normalize(aim.lockHeadingRad - pose.headingRad)) > aim.headingToleranceRad
        || !hood.atTarget(feedStopHoodTicks()));
    updateAimState(aiming, aim, ready);

    times.drive = System.nanoTime();
    if (autoDriver != null) {
      driveAuto(aiming, aim, lockRateRadPerSec, pose, velocity);
    } else {
      updateDrive(aiming, aim, lockRateRadPerSec, pose, velocity[2]);
    }
    times.driveSet = System.nanoTime();
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
    volleyFeedGate.setLeadBelowRpm(autonomous ? VolleyConstants.AUTO_VOLLEY_LEAD_BELOW_RPM
        : VolleyConstants.VOLLEY_LEAD_BELOW_RPM);
    volleyFeedGate.setMinFeedSpacingS(VolleyConstants.VOLLEY_MIN_FEED_SPACING_S);
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
    times.feed = System.nanoTime();
    updateIntakeFeederIndexer(aiming, aimedFeed || manualReady);
    times.feedSet = System.nanoTime();
    if (aimedFeed) {
      feedCommitUntilS = clock.seconds() + FireConstants.FEED_COMMIT_S;
    }
    lastPose = pose;
    lastVelocity = velocity;

    times.run = System.nanoTime();
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

    times.log = System.nanoTime();
    boolean telemetryDue = display.due();
    updateBattery(telemetryDue);
    log.logAim(aim);
    if (volleyMode) {
      log.logVolley();
    }
    display.drawField(pose, aim);
    display.update(telemetryDue, aim, pose);
    display.sendPanelsTelemetry(aim, pose);
    times.end = System.nanoTime();
    times.record();
  }

  /**
   * Corrects the pose from the Hive tags, and moves the tracker's stored Cell and poses by the same
   * amount so the aim does not jump. Returns the pose to use this loop.
   */
  private FieldPose correctPoseFromVision(FieldPose pose, double[] velocity, long nowNanos) {
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
    }
    double frameAgeS = Double.NaN; // Of the newest frame handed in
    for (TagObservation observation : observations) {
      double ageS = (nowNanos - observation.frameTimeNanos) * 1e-9;
      frameAgeS = Double.isNaN(frameAgeS) ? ageS : Math.min(frameAgeS, ageS);
    }
    log.logVision(result, frameAgeS, observations.size());
    return pose;
  }

  /** Records this loop's pose against this loop's capture time, so later frames can be matched to it. */
  private void stampPoseHistory(FieldPose pose) {
    long stampNanos = vision.getCaptureNanos();
    if (stampNanos > lastHistoryNanos && stampNanos > 0) { // Not before the first capture, or twice for one
      poseHistory.add(stampNanos, pose);
      lastHistoryNanos = stampNanos;
    }
  }

  /** The Cell on the robot's side of the Hive, changing only once the robot is FRONT_CELL_SWITCH_M past the middle. */
  static Cell frontCell(Cell current, double xM) {
    double margin = AimConstants.FRONT_CELL_SWITCH_M;
    if (current == null) {
      return xM >= 0 ? Cell.AUDIENCE : Cell.SCORING;
    }
    if (current == Cell.AUDIENCE && xM < -margin) {
      return Cell.SCORING;
    }
    if (current == Cell.SCORING && xM > margin) {
      return Cell.AUDIENCE;
    }
    return current;
  }

  /** The Cell aimed at: the one in front ({@link #frontCell}), or the tags' up Cell with AIM_AT_FRONT_CELL off. */
  Cell aimCell() {
    return AimConstants.AIM_AT_FRONT_CELL && frontCell != null ? frontCell : visionCorrector.upCell(alliance);
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
  private Aim aimAtCell(FieldPose pose, double[] velocity, long nowNanos, boolean aiming) {
    tracker.recordPose(nowNanos, pose);
    // Every loop, aiming or not, so it has settled by the time RT is pressed. Only the aim uses it
    // Loop clock, not the frame's capture time: that only moves when a camera frame arrives
    double velocityNowS = clock.seconds();
    aimVelocity = ShootOnTheMove.smoothAimVelocity(aimVelocity, velocity, velocityNowS - aimVelocityS,
        AimConstants.AIM_VELOCITY_SMOOTHING_S);
    aimVelocityS = velocityNowS;
    Logger.recordOutput("SOTM/AimVelocity", aimVelocity);
    // Spinning fast, the frame's pose is too uncertain and moved the Cell 0.17 m during one shot (5 Nov)
    boolean steady = Math.abs(velocity[2]) <= VisionConstants.MAX_OMEGA_FOR_VISION_RAD_S;
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
    aimingAtFieldCell = AimConstants.AIM_AT_FIELD_CELL
        && (poseValid && (AimConstants.AIM_AT_FRONT_CELL || visionCorrector.upCellConfirmed(alliance))
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
    boolean frontOfCell = BackwardShotConstants.BACKWARD_SHOTS_ON && wedgeForcesBackward(
        ShootOnTheMove.inFrontOfCell(pose, cellX, cellY, ShotPickConstants.BACKWARD_ONLY_WEDGE_DEG),
        backward.shot.distanceToLipM);
    if (!BackwardShotConstants.BACKWARD_SHOTS_ON || backward.shot.reason == Shot.Reason.OUT_OF_RANGE) {
      pickBackward = false;
    } else if (frontOfCell || backwardOnly) {
      pickBackward = true;
    } else if (ShotPickConstants.SHOT_PICK_QUICKEST) {
      // Whichever is ready soonest from the heading, hood and wheels now (8 Nov): no backward arc
      pickBackward = ShootOnTheMove.backwardQuicker(hooded, backward, pose.headingRad,
          HoodSubsystem.ticksToAngle(hood.getPosition()), shooter.getRpm());
    } else {
      pickBackward = hooded.shot.reason == Shot.Reason.OUT_OF_RANGE
          || backward.shot.distanceToLipM <= ShotPickConstants.BACKWARD_PICK_MAX_DIST_M;
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
      } else if (!frontOfCell && nowS - backwardBlockedSinceS >= ShotPickConstants.BACKWARD_GIVE_UP_S
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
      rpmTrim += DriverConstants.RPM_TRIM_STEP;
    }
    if (aiming && justPressed(driver.dpad_down, wasDpadDown)) {
      rpmTrim -= DriverConstants.RPM_TRIM_STEP;
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
    double maxPower = aiming ? DriverConstants.AIM_DRIVE_MAX_POWER : 1.0;
    // Held on the spot the driver asks for no motion; the hold's own corrections show up as acceleration
    double[] stick = isHolding() ? new double[] {0, 0}
        : driveOverride != null && steadyStickReference != null ? steadyStickReference
        : stickTranslation(maxPower);
    if (Math.hypot(stick[0] - steadyStick[0], stick[1] - steadyStick[1]) > FireConstants.FIRE_STICK_CHANGE) {
      steadyStick = stick;
      steadyStickSinceS = nowS;
    }
    velocityHistory.addLast(new double[] {nowS, velocity[0], velocity[1]});
    while (velocityHistory.size() > 2 && nowS - velocityHistory.peekFirst()[0] > FireConstants.ACCEL_WINDOW_S) {
      velocityHistory.removeFirst();
    }
    double[] oldest = velocityHistory.peekFirst();
    double spanS = nowS - oldest[0];
    double accelMps2 = spanS > 0.02 ? Math.hypot(velocity[0] - oldest[1], velocity[1] - oldest[2]) / spanS : 0.0;
    boolean steady = nowS - steadyStickSinceS >= FireConstants.FIRE_STEADY_STICK_S
        && accelMps2 <= FireConstants.MAX_ACCEL_TO_FIRE_MPS2;
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

  boolean isHolding() { return !Double.isNaN(holdAnchorX); }

  /** {forward, right} power back to RB's held spot, damped by the speed. Not curved as a stick is. */
  private double[] holdTranslation(FieldPose pose, double minPower) {
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
  private double[] holdPointTranslation(FieldPose pose) {
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

  private void updateDrive(boolean aiming, Aim aim, double lockRateRadPerSec, FieldPose pose, double turnRateRadPerSec) {
    // Slower while shooting, so the shot's lead for the robot's speed stays small and the aim keeps up
    double maxPower = aiming ? DriverConstants.AIM_DRIVE_MAX_POWER : 1.0;
    double[] translation = !isHolding() ? stickTranslation(maxPower)
        : holdPoint != null && driveOverride == null ? holdPointTranslation(pose) : holdTranslation(pose, HOLD_MIN_POWER);
    // The drive runs before this loop's feed check, so lastTranslation is what was commanded as the feed started
    double nowS = clock.seconds();
    boolean stickReleased = translation[0] == 0.0 && translation[1] == 0.0;
    if (isHolding()) {
      feedCommitUntilS = Double.NEGATIVE_INFINITY; // The hold keeps it still through the feed already
    }
    if (committedTranslation != null
        && (stickReleased || nowS - committedSinceS > FireConstants.FEED_COMMIT_MAX_S)) {
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
  private void driveAuto(boolean aiming, Aim aim, double lockRateRadPerSec, FieldPose pose, double[] velocity) {
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
    if (aiming && magnitude > DriverConstants.AIM_DRIVE_MAX_POWER) { // Slower while shooting, as for the driver
      translation[0] *= DriverConstants.AIM_DRIVE_MAX_POWER / magnitude;
      translation[1] *= DriverConstants.AIM_DRIVE_MAX_POWER / magnitude;
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
    boolean shotJustLost = !shotKnown && nowS - lastShotKnownS <= FireConstants.PRESPIN_HOLD_S;
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
      boolean fast = volleyShot && volley.fastRecovery();

      shooter.setSpinThresholds(fast ? VolleyConstants.VOLLEY_OPEN_LOOP_BELOW_RPM : Double.NaN,
          fast ? VolleyConstants.VOLLEY_HANDBACK_RPM : Double.NaN);
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
    } else if (shotKnown && FireConstants.PRESPIN_ON) {
      // Already at the speed RT would ask for, so a shot doesn't wait on spin-up (0.8 s from idle, 4 Nov)
      shooter.setShooterState(ShooterState.AIM);
      shooter.setTargetRpm(ShooterTuning.RPM_OVERRIDE > 0 ? ShooterTuning.RPM_OVERRIDE
          : aimRpm + rpmTrim + tableRpmOffset(aim) + volleyBoostRpm(aim) + volleyBackwardRpm(aim),
          ShooterTuning.RPM_OVERRIDE > 0 ? 0.0 : rpmRate);
    } else if (shotJustLost && FireConstants.PRESPIN_ON
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
    boolean watchingIntake = StackPressure.ON && loading && !intakeEased && !autoIntake && !tuningMode
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

    boolean intakeWanted = tuningMode || !DriverConstants.INTAKE_ONLY_ON_LT
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
    indexerCommand = agitated(indexerCommand, autonomous && autoIndexerPaused, autoIndexerReverse);
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
        && !VolleyConstants.VOLLEY_HOLD_BETWEEN_BALLS;
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
    return inFrontOfCell && backwardLipM <= ShotPickConstants.BACKWARD_WEDGE_MAX_LIP_M;
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

  /** The auto's agitation pauses only loading: a feed, an eject or a stop goes on as commanded. */
  static IndexerState agitated(IndexerState command, boolean paused) {
    return agitated(command, paused, false);
  }

  /** As above; with reverse the paused part runs the indexer backward, jiggling the stack, instead of stopping it. */
  static IndexerState agitated(IndexerState command, boolean paused, boolean reverse) {
    if (!paused || command != IndexerState.LOAD) {
      return command;
    }
    return reverse ? IndexerState.EJECT : IndexerState.STOP;
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
    return (int) Math.round(FireConstants.FEED_STOP_HOOD_DEG * HoodSubsystem.TICKS_PER_DEG);
  }

  private static int volleyHoodToleranceTicks() {
    return (int) Math.round(VolleyConstants.VOLLEY_HOOD_TOLERANCE_DEG * HoodSubsystem.TICKS_PER_DEG);
  }

  private static int hoodToleranceTicks() {
    return (int) Math.round(FireConstants.HOOD_TOLERANCE_DEG * HoodSubsystem.TICKS_PER_DEG);
  }

  /** Shown on the Panels Telemetry tab this loop, above the robot's own lines. */
  public void addPanelsLine(String key, String value) { display.addPanelsLine(key, value); }

  /** Every subsystem's state on the Driver Station: the tuning OpModes, or TELEMETRY_DETAIL from Panels. */
  public boolean isTelemetryDetail() {
    return tuningMode || DriverConstants.TELEMETRY_DETAIL;
  }

  /** Power summary for the co-driver, first on the Driver Station (Main adds it before anything else). */
  public void addPowerTelemetry() {
    display.addPowerTelemetry();
  }

  /**
   * A battery this low at rest sags toward brownout in a match: two 9 Nov runs that started at 11.1-11.9 V fell to
   * 6.8-6.9 V, and every auto sags to 9.6 V at spin-up even from 12.6 V.
   */
  static boolean swapBattery(double restingVolts) {
    return restingVolts < DriverConstants.SWAP_BATTERY_V;
  }

  /**
   * Samples the battery voltage, keeping the lowest seen this run (it sags when the shooter spins up),
   * then logs it and adds it to telemetry.
   */
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

  /** The pose this loop aimed from, after the vision correction. */
  public FieldPose getPose() { return lastPose; }
  /** Field velocity {x, y, turn rate} this loop, m/s and rad/s. */
  public double[] getVelocity() { return lastVelocity.clone(); }
  public boolean isPoseValid() { return poseValid; }
  public double getBatteryVolts() { return batteryVolts; }
  public int getPositionFixes() { return positionFixes; }
  public double getAimCellX() { return aimCellX; }
  public double getAimCellY() { return aimCellY; }
  /** Aims as RT asks but feeds nothing: a scripted test lining up before the shot it means to take. */
  public void setHoldFire(boolean hold) { holdFire = hold; }

  /** Heading/hood may track while filling, but flywheels start only for an enabled shot. */
  public void setSpinOnlyOnFire(boolean enabled) { spinOnlyOnFire = enabled; }

  double tableRpmOffset(Aim aim) {
    if (aim == null || (tuningMode && !tuningLive)) {
      return 0;
    }
    double toward = finiteOr0(aim.shot.speedTowardCellMps);
    return aim.backward ? SotmConstants.SOTM_BACKWARD_RPM_PER_TOWARD_MPS * toward
        : AimConstants.HOODED_RPM_OFFSET + SotmConstants.SOTM_RPM_PER_TOWARD_MPS * toward;
  }

  /** Launch angle added to a hooded shot for its speed toward the Cell. */
  private double movingHoodDeg(Aim aim) {
    if (aim == null || aim.backward || (tuningMode && !tuningLive)) {
      return 0;
    }
    double towardMps = finiteOr0(aim.shot.speedTowardCellMps);
    return SotmConstants.SOTM_HOOD_DEG_PER_TOWARD_MPS * towardMps + (tuningMode ? 0
        : movingHoodTrimDeg(aim.shot.distanceToLipM, Math.hypot(towardMps, finiteOr0(aim.shot.sidewaysSpeedMps))));
  }

  /**
   * TeleOp's launch angle added to a moving hooded shot for its distance to the lip
   * ({@link SotmConstants#SOTM_HOOD_TRIM_DEG}): full from the moving shot speed, scaled down to none at rest.
   */
  static double movingHoodTrimDeg(double lipM, double speedMps) {
    double[] d = SotmConstants.SOTM_HOOD_TRIM_DIST_M;
    double[] t = SotmConstants.SOTM_HOOD_TRIM_DEG;
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
    return trim * Math.min(1, Math.abs(speedMps) / FireConstants.MOVING_SHOT_SPEED_MPS);
  }

  /**
   * Aim trim, degrees anticlockwise: the tuning tools' own (toolTrimDeg), else the standing trim they found for the
   * hooded or backward shot.
   */
  static double aimTrimDeg(boolean tuningMode, double toolTrimDeg, boolean backward) {
    if (tuningMode) {
      return toolTrimDeg;
    }
    return backward ? BackwardShotConstants.BACKWARD_HEADING_TRIM_DEG
        : AimConstants.HOODED_HEADING_TRIM_DEG;
  }

  /** Aim turned anticlockwise for the shot's sideways speed. */
  private double movingHeadingDeg(Aim aim) {
    if (aim == null || (tuningMode && !tuningLive)) {
      return 0;
    }
    return (aim.backward ? SotmConstants.SOTM_BACKWARD_HEADING_DEG_PER_SIDEWAYS_MPS
        : SotmConstants.SOTM_HEADING_DEG_PER_SIDEWAYS_MPS) * finiteOr0(aim.shot.sidewaysSpeedMps);
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
  public Cell getUpCell() { return visionCorrector.upCell(alliance); }

  public boolean isUpCellConfirmed() { return visionCorrector.upCellConfirmed(alliance); }

  /** True on the loop an aimed feed starts a ball. */
  public boolean feedStartedThisLoop() { return feedGate.feedStarted(); }

  /** True on the loop the wheels showed a ball leaving (aimed or volley). */
  public boolean ballLeftThisLoop() { return feedGate.ballLeft(); }

  /** The Pinpoint's last read was good: false while it calibrates after the reset INIT asks for. */
  public boolean isOdometryReady() { return odometry.isReadGood(); }

  /** True once a feed pulse and its pause passed with no ball out, until one leaves. */
  public boolean isFeederEmpty() { return feedGate.feederEmpty(); }

  /** The stack read full this loop (a loading ball touched the wheels): the auto moves on from a Flower. */
  public boolean isStackFull() { return wasFull; }

  /** Frames so far whose tags voted for our up Cell as it stands. */
  public int getUpCellAgreeingFrames() { return visionCorrector.agreeingFrames(alliance); }

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

  /** Powers for the Scoring Auto's drive, from this loop's pose (vision corrected) and field-frame velocity. */
  public interface AutoDriver {
    /** @return robot-frame {forward, left, counterclockwise}, each -1..1; null to stand still, not even aiming */
    double[] drive(FieldPose pose, double[] velocity);
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
    FieldPose handoff = PoseHandoff.take(System.nanoTime());
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

  /** In the auto only: a loading indexer stops while true (the auto's agitation). */
  public void setAutoIndexerPaused(boolean paused) { autoIndexerPaused = paused; }

  /** In the auto only: the agitation's pause runs the indexer backward instead of stopping it. */
  public void setAutoIndexerReverse(boolean reverse) { autoIndexerReverse = reverse; }

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
    lastPose = new FieldPose(xM, yM, headingRad);
  }

  /** Gamepad 1 merged with the Panels gamepad, as this loop's controls read it. */
  public Gamepad getDriver() { return driver; }

  /** Adds this to every aimed shot's RPM, as the driver's trim does. */
  public void setRpmTrim(double rpm) { rpmTrim = rpm; }

  public void setHeadingTrimDeg(double deg) { headingTrimDeg = deg; }


  /** Raises every hooded shot's launch angle this much, as the driver's hood trim does (lift falls). */
  public void setHoodAngleTrimDeg(double deg) { hoodLiftTrimDeg = -deg; }

  // ---- Volley (Lebot Teleop RB, Volley Tuning) ----

  /** RB's hooded aimed shot, held still, fires as a burst; RT's on the move too while VOLLEY_ON_RT. */
  public void setVolleyMode(boolean on) { volleyMode = on; }

  /**
   * A volley is RB's (it wins when both are held: the robot is holding still), the aim a tool forces without RT
   * (Volley Tuning, so the hood and wheels are already at the volley's before RB), or RT's on the move when
   * {@code rtVolley}. Volley Tuning's RT stays a one-at-a-time shot.
   */
  public static boolean volleyRequested(boolean rightBumper, double rightTrigger, boolean forceAim, boolean tuningMode,
      boolean rtVolley) {
    if (tuningMode) { return false; }
    if (rightBumper) { return true; }
    return forceAim ? rightTrigger <= TRIGGER_PRESSED : rtVolley && rightTrigger > TRIGGER_PRESSED;
  }

  /** Volley calibration measures the hooded shot at every test spot. */
  public void setHoodedOnly(boolean on) { hoodedOnly = on; }

  /** Volley Tuning's backward spots: the backward shot whenever the table has one. */
  public void setBackwardOnly(boolean on) { backwardOnly = on; }

  /** A, B, X, Y and the D-pad do nothing in the robot, so an OpMode can use them (Volley Tuning's marks). */
  public void setButtonsReserved(boolean on) { buttonsReserved = on; }

  /** Launch angle added to a volley's first ball and to balls 2-4; NaN for the constants'. */
  public void setVolleyTrimsDeg(double firstDeg, double laterDeg) {
    setVolleyTrimsDeg(firstDeg, laterDeg, false);
  }

  /** As above; {@code backward} when they were tuned on backward volleys, which use them as RPM. */
  public void setVolleyTrimsDeg(double firstDeg, double laterDeg, boolean backward) {
    volley.setTrimsDeg(firstDeg, laterDeg, backward);
  }

  /** Feeder and indexer power during a volley; NaN for VOLLEY_FEED_POWER. */
  public void setVolleyFeedPower(double power) { volley.setFeedPower(power); }

  /** Separate queue pressure for calibration; NaN follows the feeder as normal Volley TeleOp does. */
  public void setVolleyIndexerPower(double power) { volley.setIndexerPower(power); }

  /** Automatic calibration's aim correction, in addition to the standing shot trim. */
  public void setVolleyHeadingTrimDeg(double deg) { volley.setHeadingTrimDeg(deg); }

  public double volleyIndexerPower() { return volley.indexerPower(); }

  /** RPM over the table's for a volley; NaN for VOLLEY_BOOST_RPM. */
  public void setVolleyBoostRpm(double rpm) { volley.setBoostRpm(rpm); }

  /** Full-power recovery between a volley's balls; null for VOLLEY_FAST_RECOVERY. */
  public void setVolleyFastRecovery(Boolean on) { volley.setFastRecovery(on); }

  /** This loop's shot is a volley: volley mode, RB or RT, an aimed shot (backward too), not a tuning tool's. */
  public boolean isVolleyShot() { return volleyShot; }

  /** About the wheel speed the last volley ball left at; NaN if none. */
  public double getVolleyLeftAtRpm() { return volleyFeedGate.peakAtLeaveRpm(); }

  /** How far the wheels have dropped since the last volley ball left; NaN if none. */
  public double getVolleyDipDepthRpm() { return volleyFeedGate.lastDipDepthRpm(); }

  public double volleyFeedPower() { return volley.feedPower(); }

  public double volleyBoostRpm() { return volley.boostRpm(); }

  public boolean volleyFastRecovery() { return volley.fastRecovery(); }

  private boolean buttonsFree() {
    return !tuningMode && !buttonsReserved;
  }

  private boolean volleyAim(Aim aim) {
    return volleyMode && volleyRequested && aim != null
        && (!aim.backward || VolleyConstants.VOLLEY_BACKWARD_ON);
  }

  private double volleyBoostRpm(Aim aim) {
    return volleyAim(aim) && !aim.backward ? volleyBoostRpm() : 0;
  }

  /** Top of a volley's matched band: tighter for a backward burst, which goes long when fed fast. */
  private static double volleyReadyAboveRpm(Aim aim) {
    return aim != null && aim.backward ? VolleyConstants.VOLLEY_BACKWARD_READY_ABOVE_RPM
        : VolleyConstants.VOLLEY_READY_ABOVE_RPM;
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
    double gain = VolleyConstants.VOLLEY_HOOD_FOLLOW_GAIN;
    if (gain != 0) {
      double above = rpmTrend.predicted(FireConstants.RPM_LOOKAHEAD_S) - shooter.getTargetRpm() + boost;
      above = Math.max(boost - VolleyConstants.VOLLEY_READY_BELOW_RPM,
          Math.min(boost + VolleyConstants.VOLLEY_READY_ABOVE_RPM, above));
      followDeg = gain * (ShootOnTheMove.hoodOffsetForRpmAbove(lipM, towardMps, above) - boostDeg);
    }
    return finiteOr0(boostDeg + followDeg) + volleyTrimDeg(false);
  }

  /** The trim for the ball about to go (VolleyConfig.trimDeg). */
  private double volleyTrimDeg(boolean backward) {
    return volley.trimDeg(volleyFeedGate.ballsLeft() == 0, backward);
  }

  public static double volleyTrimDeg(double trim, boolean volleyTrimsBackward, boolean backward, boolean first) {
    return VolleyConfig.trimDeg(trim, volleyTrimsBackward, backward, first);
  }

  /** A backward volley's trim, as wheel speed: its hood follows the table, and its range comes from the RPM. */
  private double volleyBackwardRpm(Aim aim) {
    return volleyAim(aim) && aim.backward
        ? -VolleyConstants.VOLLEY_BACKWARD_RPM_PER_TRIM_DEG * volleyTrimDeg(true) : 0;
  }

  /** Called once when the OpMode stops. */
  public void stop() {
    watchdog.feed(); // A stop that hangs (closing a camera) is reported too
    if (handsOffPose) {
      PoseHandoff.save(isPoseKnown() && !odometry.isFaulted() ? lastPose : null, alliance, System.nanoTime());
    }
    display.stop();
    hood.disable(); // Its loop thread would otherwise hold the hood until it misses the heartbeat
    vision.close();
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().reset();
    watchdog.stop();
  }
}
