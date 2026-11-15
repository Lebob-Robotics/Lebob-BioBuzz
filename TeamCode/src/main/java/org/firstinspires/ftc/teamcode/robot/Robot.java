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
import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RampSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem.ShooterState;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import com.bylazar.gamepad.PanelsGamepad;
import org.firstinspires.ftc.teamcode.shooting.Aim;
import org.firstinspires.ftc.teamcode.field.PoseHandoff;
import org.firstinspires.ftc.teamcode.control.LoopWatchdog;
import org.firstinspires.ftc.teamcode.field.FieldPose;
import org.firstinspires.ftc.teamcode.shooting.volley.VolleyConstants;
import org.firstinspires.ftc.teamcode.vision.VisionConstants;
import org.firstinspires.ftc.teamcode.vision.TagObservation;
import org.firstinspires.ftc.teamcode.vision.TargetTracker;
import org.firstinspires.ftc.teamcode.vision.VisionCorrector;
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
  final Telemetry telemetry;
  private final RobotTelemetry display;
  final RobotLogger log = new RobotLogger(this);
  final BallPath ballPath = new BallPath(this);
  final DriveControl driveControl = new DriveControl(this);
  final ShooterControl shooterControl = new ShooterControl(this);
  final AimSelector aimSelector = new AimSelector(this);
  final PoseFusion poseFusion = new PoseFusion(this);
  final FireControl fireControl = new FireControl(this);
  final DriverPad driverPad = new DriverPad(this);
  private final RobotLogger.LoopTimes times = new RobotLogger.LoopTimes();
  /** The real gamepad 1, kept for rumble. */
  final Gamepad physicalDriver;
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
  boolean holdFire;
  boolean spinOnlyOnFire;
  /** Battery voltage is a separate hub read, so sample it this often. */
  private static final double BATTERY_SAMPLE_PERIOD_S = 0.1;

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
  final VisionCorrector visionCorrector = new VisionCorrector();

  double hoodLiftTrimDeg = 0;
  double rpmTrim = 0; // Driver's, for the rest of the match
  double headingTrimDeg = 0; // Anticlockwise, the tuning tools' aim trim (TeleOp uses aimTrimDeg's)
  // Shot Tuning: the driver's RPM, hood and direction replace the tables', and the buttons they use do nothing else
  boolean tuningMode = false;
  boolean tuningLive = false; // Shot Tuning's moving pass: the tables' shot, moving corrections and all
  boolean closeMovingGateBypassed = false; // SOTM Tuning's passes that fire moving close to the lip
  boolean scriptedOneBall = false; // As RB held in Shot Tuning, from a script
  private boolean scriptedVolley = false; // As RB held in Lebot Teleop, from a script (autonomous)
  private Aim lastAim = null;
  private boolean forceAim = false; // Aims as RB does, without firing
  private final LoopWatchdog watchdog = new LoopWatchdog();
  // Volley mode (Lebot Teleop, Volley Tuning): RB's hooded aimed shot, held still, fires as a burst, each ball at
  // the matched wheel speed (VolleyConstants). RT stays the one-at-a-time shoot on the move. Off,
  // RB is a plain held shot as RT's
  boolean volleyMode = false;
  boolean volleyRequested = false; // This loop asks for a volley: RB, Volley Tuning's aim, or RT (VOLLEY_ON_RT)
  // Volley Tuning's settings in place of the constants
  final VolleyConfig volley = new VolleyConfig();

  // Volley Tuning marks balls with A, B, X, Y and the D-pad, so they do nothing else (heading reset, eject, stop,
  // full power, trims, presets). The intake and the triggers work as in Lebot Teleop
  private boolean buttonsReserved = false;
  private boolean handsOffPose = false;
  private double initMaxVolts = Double.NaN; // Highest in INIT: at rest, before the mechanisms load it
  private boolean swapBatteryRumbled = false;
  private boolean startedFromAuto = false;
  private final double[] hiveTagSeenS = {Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY}; // By Alliance ordinal
  // This loop's results, for an OpMode that drives the robot itself (Shot Tuning)
  private FieldPose lastPose = new FieldPose(0, 0, 0);
  double[] lastVelocity = {0, 0, 0};
  // Scoring Auto (6 Nov, feature/auto): no gamepad is read (G401); it drives, aims and intakes through these
  boolean autonomous = false;
  boolean autoIndexerPaused = false; // The auto's agitation: a loading indexer stops while true
  boolean autoIndexerReverse = false; // ...or runs backward, a jiggle like the volley's retry pulse
  private final Gamepad noGamepad = new Gamepad();
  private boolean autoShoot = false;
  boolean autoIntake = false;

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
    poseFusion.reset();
    visionCorrector.reset();
    ramp.setDeployed(false);
  }

  /**
   * Shoot Auto: the robot is where it was placed, so it can aim at its Hive at once, before any tag is seen. The
   * first tag fix still replaces this in full (poseValid stays false until then). Until the tags show which Cell
   * is up, it aims at the one the corrector starts with.
   */
  public void seedPose(double xM, double yM, double headingRad) {
    odometry.setPose(xM, yM, headingRad);
    poseFusion.poseHistory.clear();
    tracker.clear();
    poseFusion.poseSeeded = true; // headingValid stays false too, so the first heading fix is also taken in full
  }

  /** Placed by the tags, or seeded by {@link #seedPose} with the odometry still good. */
  public boolean isPoseKnown() { return poseFusion.poseValid || poseFusion.poseSeeded; }

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

  void noteHiveTags(List<TagObservation> observations) {
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
      poseFusion.poseValid = false;
      poseFusion.headingValid = false;
      poseFusion.poseSeeded = false;
    }

    if (driver.a && buttonsFree()) { // Shot Tuning's A marks a ball
      odometry.setHeading(placedHeadingRad());
      tracker.clear(); // The frame just turned, so the stored Cell and poses no longer apply
      poseFusion.headingValid = true; // The driver says this is right, so vision only trims it from here
      driveControl.headingHold.clear();
      poseFusion.poseHistory.clear();
      pose = new FieldPose(pose.xM, pose.yM, placedHeadingRad());
    }

    times.vision = System.nanoTime();
    if (odometry.isReadGood()) { // A correction from a bad read would write that read back into the Pinpoint
      FieldPose uncorrected = pose;
      // Before aiming, so this loop aims from the corrected pose
      pose = poseFusion.correctPoseFromVision(pose, velocity, nowNanos);
      // The robot didn't move: only where it thinks it is did, so the held spot moves with it
      driveControl.shiftHoldAnchor(pose.xM - uncorrected.xM, pose.yM - uncorrected.yM);
    }

    double fireTrigger = fireTrigger();
    boolean holdShot = driver.right_bumper && !tuningMode;
    volleyRequested = volleyRequested(driver.right_bumper || scriptedVolley, fireTrigger, forceAim,
        tuningMode, VolleyConstants.VOLLEY_ON_RT);
    boolean shootAll = fireTrigger > TRIGGER_PRESSED || holdShot || scriptedVolley;
    boolean oneBallHeld = driver.right_bumper || scriptedOneBall;
    boolean oneBall = tuningMode && oneBallHeld && fireTrigger <= TRIGGER_PRESSED; // RT wins
    boolean aiming = shootAll || oneBallHeld || forceAim;
    driveControl.updateHoldAnchor(holdShot, pose);
    times.aim = System.nanoTime();
    Aim aim = aimSelector.aimAtCell(pose, velocity, nowNanos, aiming);
    times.aimed = System.nanoTime();
    double turnDeg = ShooterControl.aimTrimDeg(tuningMode, headingTrimDeg, aim != null && aim.backward)
        + shooterControl.movingHeadingDeg(aim);
    if (shooterControl.volleyAim(aim) && !Double.isNaN(volley.headingTrimDeg())) { turnDeg += volley.headingTrimDeg(); }
    if (aim != null && turnDeg != 0) {
      aim = aim.turnedBy(Math.toRadians(turnDeg));
    }
    lastAim = aim;
    double lockRateRadPerSec = driveControl.updateLockRate(aiming, aim);
    driverPad.updateTrim(aiming);
    driverPad.updateToggles(aiming);
    // Before the ready check, so it compares the hood and wheels with this loop's shot. After it, a hood target
    // that had just moved 3 degrees (the robot stopped) was judged against the old one, and the ball missed (5 Nov, 095605)
    times.shooter = System.nanoTime();
    fireControl.selectGate(aim);
    shooterControl.update(aiming, aim);
    times.shooterSet = System.nanoTime();
    boolean ready = fireControl.checkReady(aiming, aim, pose, velocity);
    aimSelector.updateAimState(aiming, aim, ready);

    times.drive = System.nanoTime();
    driveControl.update(aiming, aim, lockRateRadPerSec, pose, velocity);
    times.driveSet = System.nanoTime();
    fireControl.updateFeed(shootAll, oneBall, aim, pose, velocity);
    times.feed = System.nanoTime();
    ballPath.update(aiming, fireControl.aimedFeed || fireControl.manualReady);
    times.feedSet = System.nanoTime();
    if (fireControl.aimedFeed) {
      driveControl.commitFeed();
    }
    lastPose = pose;
    lastVelocity = velocity;

    times.run = System.nanoTime();
    CommandScheduler.getInstance().run(); // After the states are set, so subsystems act on and log this loop's
    poseFusion.stampPoseHistory(pose); // After the run, which is when vision takes this loop's capture

    driverPad.updateRumbles();

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

  boolean shotRequested() {
    double fireTrigger = fireTrigger();
    boolean requested = fireTrigger > TRIGGER_PRESSED || driver.right_bumper
        || scriptedOneBall || scriptedVolley || (driver.y && buttonsFree());
    return requested && !(tuningMode && fireTrigger <= TRIGGER_PRESSED
        && fireControl.feedGate != null && fireControl.feedGate.ballsLeft() > 0);
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
  public boolean isPoseValid() { return poseFusion.poseValid; }
  public double getBatteryVolts() { return batteryVolts; }
  public int getPositionFixes() { return poseFusion.positionFixes; }
  public double getAimCellX() { return aimSelector.aimCellX; }
  public double getAimCellY() { return aimSelector.aimCellY; }
  /** Aims as RT asks but feeds nothing: a scripted test lining up before the shot it means to take. */
  public void setHoldFire(boolean hold) { holdFire = hold; }

  /** Heading/hood may track while filling, but flywheels start only for an enabled shot. */
  public void setSpinOnlyOnFire(boolean enabled) { spinOnlyOnFire = enabled; }

  /**
   * Shot Tuning: every shot is the one set by {@link #setTuningShot}, from anywhere, and D-pad, left bumper, X and
   * Y are left to the OpMode.
   */
  public void setTuningMode(boolean on) { tuningMode = on; }

  /** @param hoodDeg launch angle; HoodSubsystem.ZERO_ANGLE_DEG is the hood on its lower stop */
  public void setTuningShot(boolean backward, double rpm, double hoodDeg) {
    aimSelector.tuningBackward = backward;
    aimSelector.tuningRpm = rpm;
    aimSelector.tuningHoodDeg = hoodDeg;
  }

  /**
   * Shot Tuning: the tables' shot for the direction set by {@link #setTuningShot}, with the moving corrections and
   * the usual gate, instead of the fixed RPM and hood. For shots taken on the move.
   */
  public void setTuningLive(boolean on) { tuningLive = on; }

  /** Tuning only: fire without waiting for a steady stick and low acceleration, to measure shots fired mid-change. */
  public void setSteadyGateBypassed(boolean on) { driveControl.steadyGateBypassed = on; }

  /** Tuning only: fire moving close to the lip, which the match's gate refuses, to measure whether those shots score. */
  public void setCloseMovingGateBypassed(boolean on) { closeMovingGateBypassed = on; }

  /**
   * Tuning only: the steady gate judges the stick by this field stick (a scripted pass's planned speed) instead of
   * the scripted drive, whose tracking corrections change every loop and so never read as held; null for the drive.
   */
  public void setSteadyStickReference(double[] fieldStick) {
    driveControl.steadyStickReference = fieldStick == null ? null : fieldStick.clone();
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
  public void setDriveOverride(double[] fieldStick) { driveControl.driveOverride = fieldStick; }

  /** Brakes the wheels at zero power (on) instead of letting them roll; teleop keeps them rolling. */
  public void setDriveBrake(boolean on) { drive.setBrake(on); }

  /** Turns to the shot and keeps the wheels and hood on it, as RB does, but fires nothing. */
  public void setForceAim(boolean on) { forceAim = on; }

  /**
   * Holds the robot on this field point {x, y}, as RB holds it in Lebot Teleop, while no drive is scripted;
   * null lets it go. Unlike RB's spot it stays put on the field when the tags move the pose.
   */
  public void setHoldPoint(double[] point) {
    driveControl.holdPoint = point == null ? null : point.clone();
  }

  /** The driver's drive and turn sticks do nothing while on; scripted drives and the aim still move the robot. */
  public void setSticksLocked(boolean on) { driveControl.sticksLocked = on; }

  /** With the sticks locked and nothing aiming, turns as the turn stick pushed this far (-1..1) would. */
  public void setTurnOverride(double turn) { driveControl.turnOverride = turn; }

  /** Seconds since the cameras last saw a tag; infinite if never. */
  public double getTagsSeenAgoS() {
    return poseFusion.lastVisionNanos == 0 ? Double.POSITIVE_INFINITY : (System.nanoTime() - poseFusion.lastVisionNanos) * 1e-9;
  }

  /** Switches to the other alliance when the cameras see its Cell while this alliance's Cell isn't known. */
  public void setAllianceFromTags(boolean on) { aimSelector.allianceFromTags = on; }

  /** Aims at this field point {x, y} instead of the Cell, once the tags have placed the robot; null for the Cell. */
  public void setAimPointOverride(double[] point) {
    aimSelector.aimPointOverride = point == null ? null : point.clone();
  }

  /** Which end of our alliance's Hive is up, as the tags last showed it (the starting guess until they have). */
  public Cell getUpCell() { return visionCorrector.upCell(alliance); }

  public boolean isUpCellConfirmed() { return visionCorrector.upCellConfirmed(alliance); }

  /** True on the loop an aimed feed starts a ball. */
  public boolean feedStartedThisLoop() { return fireControl.feedGate.feedStarted(); }

  /** True on the loop the wheels showed a ball leaving (aimed or volley). */
  public boolean ballLeftThisLoop() { return fireControl.feedGate.ballLeft(); }

  /** The Pinpoint's last read was good: false while it calibrates after the reset INIT asks for. */
  public boolean isOdometryReady() { return odometry.isReadGood(); }

  /** True once a feed pulse and its pause passed with no ball out, until one leaves. */
  public boolean isFeederEmpty() { return fireControl.feedGate.feederEmpty(); }

  /** The stack read full this loop (a loading ball touched the wheels): the auto moves on from a Flower. */
  public boolean isStackFull() { return ballPath.stackFull(); }

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
  public void setAutoDriver(AutoDriver driver) { driveControl.autoDriver = driver; }

  /** The auto's drive turns with the heading lock to this field heading, not Pedro's turn; NaN gives it back. */
  public void setAutoHeading(double headingRad) { setAutoHeading(headingRad, 0.0); }

  /** As above, for a heading that moves at this rate (CCW positive), as the Flower wiggle. */
  public void setAutoHeading(double headingRad, double rateRadPerSec) {
    driveControl.autoHeadingRad = headingRad;
    driveControl.autoHeadingRateRadPerSec = rateRadPerSec;
  }

  /**
   * As {@link #start}, but from a known pose instead of the placed heading: the Scoring Auto's measured start spot.
   * The pose counts as placed, so the robot aims before the tags are seen; they then only trim it.
   */
  public void startAt(double xM, double yM, double headingRad) {
    start();
    odometry.setPose(xM, yM, headingRad);
    poseFusion.poseValid = true;
    poseFusion.headingValid = true;
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
  public void setHoodedOnly(boolean on) { aimSelector.hoodedOnly = on; }

  /** Volley Tuning's backward spots: the backward shot whenever the table has one. */
  public void setBackwardOnly(boolean on) { aimSelector.backwardOnly = on; }

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
  public boolean isVolleyShot() { return fireControl.volleyShot; }

  /** About the wheel speed the last volley ball left at; NaN if none. */
  public double getVolleyLeftAtRpm() { return fireControl.volleyFeedGate.peakAtLeaveRpm(); }

  /** How far the wheels have dropped since the last volley ball left; NaN if none. */
  public double getVolleyDipDepthRpm() { return fireControl.volleyFeedGate.lastDipDepthRpm(); }

  public double volleyFeedPower() { return volley.feedPower(); }

  public double volleyBoostRpm() { return volley.boostRpm(); }

  public boolean volleyFastRecovery() { return volley.fastRecovery(); }

  boolean buttonsFree() {
    return !tuningMode && !buttonsReserved;
  }

  public static double volleyTrimDeg(double trim, boolean volleyTrimsBackward, boolean backward, boolean first) {
    return VolleyConfig.trimDeg(trim, volleyTrimsBackward, backward, first);
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
