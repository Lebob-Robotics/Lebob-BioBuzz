package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.panels.Panels;
import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.field.FieldPose;
import org.firstinspires.ftc.teamcode.opmodes.LebotOpMode;
import org.firstinspires.ftc.teamcode.opmodes.Main;
import org.firstinspires.ftc.teamcode.pedro.CapturedDrivetrain;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.RobotLocalizer;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.shooting.volley.VolleyConstants;
import org.firstinspires.ftc.teamcode.shooting.volley.VolleySettings;
import org.psilynx.psikit.core.Logger;

/**
 * An autonomous that runs a step plan ({@link AutoPlan}), for one alliance: {@link ScoringAuto} and
 * {@link CornerAuto}. Pedro follows each step's path from the Robot's own pose (odometry corrected by the Hive tags)
 * and the Robot drives with Pedro's powers. Each volley is shot stopped at a spot in front of a Cell, where the
 * heading lock turns the robot instead of Pedro. No gamepad is read (G401).
 *
 * <p>Select, INIT and start it from the Driver Station; the Driver Station's timer ends it at 30 s, and it stops
 * itself at {@link #STOP_ALL_S} in case. INIT homes the hood, then nothing moves until start (G304).
 *
 * <p>Each auto gives its start, its plan, and for each step its drive ({@link #startStep}), when that drive is done
 * ({@link #stepArrived}), the Cell it aims at ({@link #aimCell}) and the heading it wiggles about in a Flower.
 */
public abstract class BaseAuto<S extends Enum<S>, P extends AutoPlan<S>> extends LebotOpMode {
  /** Everything stops and the OpMode ends here, before the 30 s AUTO ends (G403). */
  public static double STOP_ALL_S = 29.5;
  /** Pedro's powers are scaled by this during the Garden sweep and into a Flower, so the intake takes the Pollen in. */
  public static double SWEEP_POWER_SCALE = 0.5;
  /** Turning to the Pollen in place (Garden, Flower) is done this close to the spot and to its heading. */
  public static double FACE_FLOWER_M = 0.10; // Pedro's hold can stall 6 cm off the Flower stop
  public static double FACE_FLOWER_RAD = Math.toRadians(8);
  /** Driving into a Flower, the robot is in it once this slow, after FLOWER_STALL_AFTER_S of the drive in. */
  public static double FLOWER_STALLED_MPS = 0.05;
  public static double FLOWER_STALL_AFTER_S = 0.5;
  /** Shorter than this, a drive is held at its end instead: Pedro can't follow a path of no length. */
  protected static final double MIN_PATH_IN = 1.0;

  protected final Alliance alliance;
  protected final RobotLocalizer localizer = new RobotLocalizer();
  protected final CapturedDrivetrain drivetrain = new CapturedDrivetrain();
  private final ElapsedTime clock = new ElapsedTime();
  protected Robot robot;
  protected Follower follower;
  protected P plan;
  private boolean odometryReady;
  private boolean stopped;
  /** Volley Tuning's confirmed spots, as Lebot Teleop shoots them; null for the constants. */
  private VolleySettings volley;
  private boolean shotLastLoop; // The volley's settings are picked only before it starts, never mid-burst
  private double speedMps;
  private double stepStartS;

  protected BaseAuto(Alliance alliance) {
    this.alliance = alliance;
  }

  /** Where it starts, red field coordinates {x, y, heading}. */
  protected abstract double[] redStart();

  /** The plan, made at start. */
  protected abstract P newPlan();

  /** INIT's line on where to place the robot. */
  protected abstract String placement();

  /** The Driver Station's line each loop. */
  protected abstract String status(double t);

  /** Starts the drive for a step, from where the robot is now ({@link #pose}). */
  protected abstract void startStep(S step);

  /** The step's drive is done, this far into the step, the robot moving this fast. */
  protected abstract boolean stepArrived(S step, boolean facingStep, double inStepS, double robotSpeedMps);

  /** The Cell to aim at now. */
  protected abstract Cell aimCell();

  /** In a step that takes a Flower in, the heading it wiggles about; NaN in any other step. */
  protected abstract double flowerHeading(S step);

  /** The agitation's pause runs the indexer backward instead of stopping it. */
  protected boolean indexerPauseRunsBackward() {
    return false;
  }

  /** First in each loop, before the aim. */
  protected void onLoopStart(double t) {
  }

  /** A ball left the shooter this loop. */
  protected void onBallLeft(double t) {
  }

  /** The heading the wiggle turns to this loop. */
  protected void onWiggle(double headingRad) {
  }

  /** The auto's own outputs, logged last in each loop. */
  protected void logExtras(double t) {
  }

  @Override
  protected void onPsiKitConfigureLogging() {
    Main.configureLogging();
    Logger.recordMetadata("Auto", getClass().getSimpleName());
    Logger.recordMetadata("PedroTuned", String.valueOf(Constants.TUNED));
  }

  @Override
  protected void onPsiKitInit() {
    if (!Main.PANELS_ENABLED) {
      Panels.INSTANCE.disable();
    }
    robot = new Robot(hardwareMap, telemetry, gamepad1);
    robot.setAutonomous(true);
    // Volleys as Lebot Teleop's RT shoots them: the indexer stages the stack between balls, its last roller slowed
    // while one feeds, and pulses a stuck ball free. Off, each ball was a one-ball shot and the last of every volley
    // stayed in
    robot.setVolleyMode(true);
    if (indexerPauseRunsBackward()) {
      robot.setAutoIndexerReverse(true);
    }
    robot.setAlliance(alliance);
    robot.setHandsOffPose(true);
    robot.init();
    if (VolleyConstants.VOLLEY_USE_TUNING_CSV) {
      volley = VolleySettings.loadTuned(); // Reads a file only: INIT moves nothing
    }
    follower = new Follower(localizer, drivetrain, Constants.algorithm());
  }

  @Override
  protected void onPsiKitInitLoop() {
    if (robot == null) {
      return;
    }
    telemetry.addData("Auto", placement());
    if (!Constants.TUNED) {
      telemetry.addData("WARNING", "Pedro not tuned: placeholder constants (run Foresight Tuner)");
    }
    if (Main.PANELS_ENABLED || Main.LIVE_LOGGING) {
      telemetry.addData("WARNING", "Panels or live logging on: both must be off at competition (R704)");
    }
    String allianceWarning = robot.allianceWarning();
    if (allianceWarning != null) {
      telemetry.addLine(allianceWarning);
    }
    robot.initLoop();
  }

  @Override
  protected void onPsiKitStart() {
    if (robot == null) {
      return;
    }
    clock.reset();
    double[] start = AutoField.forAlliance(redStart(), alliance);
    robot.startAt(start[0], start[1], start[2]);
    localizer.set(start[0], start[1], start[2], 0, 0, 0);
    robot.setAutoDriver(this::pedroPowers);
    robot.setDriveBrake(true); // Pedro's constants are measured on BRAKE
    plan = newPlan();
    plan.takeNewStep();
    startStep(plan.step());
    robot.setAutoShoot(plan.shoot(0));
    robot.setAutoIntake(plan.intake());
  }

  /** Called by the Robot each loop with the corrected pose, before it drives. */
  private double[] pedroPowers(FieldPose pose, double[] velocity) {
    // Still while the Pinpoint finishes the reset from INIT: it can still be calibrating after START, and the IMU
    // calibrates only standing still
    if (!odometryReady) {
      odometryReady = robot.isOdometryReady();
      Logger.recordOutput("Auto/WaitingForOdometry", !odometryReady);
      if (!odometryReady) {
        return null; // DriveControl.driveAuto: still, without the aim's turn
      }
    }
    localizer.set(pose.xM, pose.yM, pose.headingRad, velocity[0], velocity[1], velocity[2]);
    speedMps = Math.hypot(velocity[0], velocity[1]);
    follower.update();
    return drivetrain.powers();
  }

  @Override
  protected void onPsiKitLoop() {
    if (robot == null || stopped) {
      return;
    }
    double t = clock.seconds();
    if (t >= STOP_ALL_S) {
      stopAll();
      return;
    }
    onLoopStart(t);
    double[] aimPoint = AutoField.autoAimPoint(alliance, aimCell());
    robot.setAimPointOverride(aimPoint);
    if (volley != null && !shotLastLoop) {
      // The tuned spot nearest where it is about to shoot: spot 4 (VOLLEY_SPOT) at each Cell, the park spot at the end
      int spot = volley.spotFor(robot.getPose(), aimPoint[0], aimPoint[1]);
      VolleySettings.apply(robot, volley.get(spot));
      Logger.recordOutput("Auto/VolleySpot", spot);
    }
    addTelemetry(t);
    robot.periodic();

    boolean ballLeft = robot.ballLeftThisLoop();
    if (ballLeft) {
      onBallLeft(t);
    }
    plan.update(t, stepArrived(), ballLeft, robot.isFeederEmpty(), robot.isStackFull());
    if (plan.takeNewStep()) {
      stepStartS = t;
      startStep(plan.step());
    }
    shotLastLoop = plan.shoot(t);
    robot.setAutoShoot(shotLastLoop);
    robot.setAutoIntake(plan.intake());
    robot.setAutoIndexerPaused(plan.indexerPaused(t));
    robot.ramp.setDeployed(plan.armDown());
    double flowerHeading = flowerHeading(plan.step());
    double inFlowerS = plan.arrivedForS(t);
    if (!Double.isNaN(flowerHeading) && !Double.isNaN(inFlowerS)) {
      // In the Flower, wiggling left and right for the intake to take every ball
      double[] wiggle = AutoSequence.flowerWiggle(inFlowerS);
      robot.setAutoHeading(flowerHeading + wiggle[0], wiggle[1]);
      onWiggle(flowerHeading + wiggle[0]);
    }

    Logger.recordOutput("Auto/TimeS", t);
    Logger.recordOutput("Auto/Step", plan.step().name());
    Logger.recordOutput("Auto/BallsShot", plan.ballsShot());
    Logger.recordOutput("Auto/AimCell", aimCell().name());
    Logger.recordOutput("Auto/IndexerPaused", plan.indexerPaused(t));
    Logger.recordOutput("Auto/FollowerMode", follower.mode().name());
    Logger.recordOutput("Auto/ArmDown", plan.armDown());
    logExtras(t);
  }

  private boolean stepArrived() {
    return stepArrived(plan.step(), plan.isFacingStep(), clock.seconds() - stepStartS, speedMps);
  }

  /** The Robot's corrected pose. */
  protected FieldPose pose() {
    return robot.getPose();
  }

  /** The heading lock's target, NaN to leave the turn to Pedro. */
  protected void lockHeading(double headingRad) {
    robot.setAutoHeading(headingRad);
    Logger.recordOutput("Auto/HeadingTarget", headingRad);
  }

  /** A straight drive to the end's heading: turning on the way, or facing it from the start. */
  protected void line(Pose here, Pose end, boolean turnOnTheWay) {
    if (here.distance(end) < MIN_PATH_IN) {
      follower.hold(end);
      return;
    }
    Path line = Paths.line(here, end);
    follower.follow(turnOnTheWay ? line.linear(here.heading(), end.heading()) : line.constant(end.heading()));
  }

  private void addTelemetry(double t) {
    telemetry.addData("Auto", status(t));
    // The Driver Station loads the preselected TeleOp only when the auto ends itself or its timer does
    telemetry.addData("STOP", "pressed early? Select Lebot Teleop by hand");
    if (!Constants.TUNED) {
      telemetry.addData("WARNING", "Pedro not tuned");
    }
  }

  /** Stops every mechanism and ends the OpMode. */
  private void stopAll() {
    stopped = true;
    robot.setAutoShoot(false);
    robot.setAutoIntake(false);
    robot.setAutoIndexerPaused(false);
    if (indexerPauseRunsBackward()) {
      robot.setAutoIndexerReverse(false);
    }
    robot.ramp.setDeployed(false);
    robot.setAutoHeading(Double.NaN);
    drivetrain.setStrafeBias(0.0);
    follower.stop();
    robot.setAutoDriver(null);
    robot.drive.stop();
    requestOpModeStop();
  }

  @Override
  protected void onPsiKitStop() {
    if (robot != null) {
      robot.stop();
    }
  }
}
