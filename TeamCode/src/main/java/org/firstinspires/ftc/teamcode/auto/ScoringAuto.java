package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.auto.AutoField.pedro;

import com.bylazar.panels.Panels;
import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Main;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedro.CapturedDrivetrain;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.RobotLocalizer;
import org.firstinspires.ftc.teamcode.utils.VisionCorrector;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.PsiKitIterativeOpMode;

/**
 * The shooting autonomous ({@link AutoSequence}), for one alliance. Pedro follows the paths from the Robot's own
 * pose (odometry corrected by the Hive tags) and the Robot drives with Pedro's powers. Each volley is shot stopped
 * at a spot in front of the up Cell, where the heading lock turns the robot instead of Pedro. No gamepad is read
 * (G401).
 *
 * <p>Select, INIT and start it from the Driver Station; the Driver Station's timer ends it at 30 s, and it stops
 * itself at {@link #STOP_ALL_S} in case. INIT homes the hood, then nothing moves until start (G304).
 */
public abstract class ScoringAuto extends PsiKitIterativeOpMode {
  /** Everything stops and the OpMode ends here, before the 30 s AUTO ends (G403). */
  public static double STOP_ALL_S = 29.5;
  /** Pedro's powers are scaled by this during the Garden sweep, so the intake takes the Pollen in. */
  public static double SWEEP_POWER_SCALE = 0.5;
  /** Turning to the Pollen in place (Garden, Flower) is done this close to the spot and to its heading. */
  public static double FACE_FLOWER_M = 0.10; // 14 Nov Pedro's hold stalled 6 cm off the Flower stop for 2 s
  public static double FACE_FLOWER_RAD = Math.toRadians(8);
  /** Onto the Garden's wall is closer: the sweep runs against it. */
  public static double ONTO_WALL_M = 0.02;
  /**
   * Strafe power added toward the audience wall (the robot's left, both alliances) while sliding onto it and sweeping
   * along it. 14 Nov Pedro alone stalled 5.8 cm off at 0.14 strafe, and swept 2.5-3 cm off.
   */
  public static double WALL_PUSH = 0.25;
  /** Sliding onto the wall, the robot counts as against it once this slow, after WALL_SETTLE_S of the slide. */
  public static double WALL_STOPPED_MPS = 0.03;
  public static double WALL_SETTLE_S = 0.3;
  /** Driving into a Flower, the robot is in it once this slow, after FLOWER_STALL_AFTER_S of the drive in. */
  public static double FLOWER_STALLED_MPS = 0.05;
  public static double FLOWER_STALL_AFTER_S = 0.5;
  /** Shorter than this, a drive is held at its end instead: Pedro can't follow a path of no length. */
  private static final double MIN_PATH_IN = 1.0;

  private final Alliance alliance;
  private final RobotLocalizer localizer = new RobotLocalizer();
  private final CapturedDrivetrain drivetrain = new CapturedDrivetrain();
  private final ElapsedTime clock = new ElapsedTime();
  private Robot robot;
  private boolean odometryReady;
  private Follower follower;
  private AutoSequence sequence;
  private TipPredictor tips;
  private boolean stopped;
  private double speedMps;
  private double stepStartS;
  /** The Cell the current shooting spot is in front of, picked as its drive started. */
  private VisionCorrector.Cell shotSpotCell;
  /** The Cell up at the start, the first volley's: the plan's later volleys alternate from it. */
  private VisionCorrector.Cell firstCell;

  protected ScoringAuto(Alliance alliance) {
    this.alliance = alliance;
  }

  @Override
  protected int getRlogPort() {
    return Main.LIVE_LOGGING ? Main.RLOG_PORT : 0;
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
    // stayed in (14 Nov 214945)
    robot.setVolleyMode(true);
    robot.setAlliance(alliance);
    robot.setHandsOffPose(true);
    robot.init();
    follower = new Follower(localizer, drivetrain, Constants.algorithm());
  }

  @Override
  protected void onPsiKitInitLoop() {
    if (robot == null) {
      return;
    }
    telemetry.addData("Auto", alliance + ": place the robot touching our wall at the start mark, camera to the Hive");
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
    double[] start = AutoField.forAlliance(AutoField.START, alliance);
    robot.startAt(start[0], start[1], start[2]);
    localizer.set(start[0], start[1], start[2], 0, 0, 0);
    robot.setAutoDriver(this::pedroPowers);
    robot.setDriveBrake(true); // Pedro's constants are measured on BRAKE (13 Nov)
    sequence = new AutoSequence(0);
    tips = new TipPredictor(robot.getUpCell());
    firstCell = tips.cell();
    sequence.takeNewStep();
    startStep(sequence.step());
    robot.setAutoShoot(sequence.shoot(0));
    robot.setAutoIntake(sequence.intake());
  }

  /** Called by the Robot each loop with the corrected pose, before it drives. */
  private double[] pedroPowers(org.firstinspires.ftc.teamcode.utils.Pose pose, double[] velocity) {
    // Still while the Pinpoint finishes the reset from INIT: 13 Nov it calibrated for 0.2 s after START both runs,
    // and the IMU calibrates only standing still
    if (!odometryReady) {
      odometryReady = robot.isOdometryReady();
      Logger.recordOutput("Auto/WaitingForOdometry", !odometryReady);
      if (!odometryReady) {
        return null; // Robot.driveAuto: still, without the aim's turn
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
    tips.onVision(robot.getUpCell(), robot.getUpCellAgreeingFrames(), t);
    robot.setAimPointOverride(AutoField.autoAimPoint(alliance, aimCell()));
    addTelemetry(t);
    robot.periodic();

    boolean ballLeft = robot.ballLeftThisLoop();
    if (ballLeft) {
      tips.onBall(t);
    }
    sequence.update(t, stepArrived(), ballLeft, robot.isFeederEmpty(), robot.isStackFull());
    if (sequence.takeNewStep()) {
      stepStartS = t;
      startStep(sequence.step());
    }
    robot.setAutoShoot(sequence.shoot(t));
    robot.setAutoIntake(sequence.intake());
    robot.ramp.setDeployed(sequence.armDown());
    AutoSequence.Step step = sequence.step();
    if (step == AutoSequence.Step.PARKED) { // Touching the wall, whatever the turn to the Cell
      double[] push = AutoField.wallPush(alliance, robot.getPose().headingRad, AutoField.PARK_PUSH);
      drivetrain.setForwardBias(push[0]);
      drivetrain.setStrafeBias(push[1]);
    }
    double inFlowerS = sequence.arrivedForS(t);
    if ((step == AutoSequence.Step.INTAKE_WALL_FLOWER || step == AutoSequence.Step.INTAKE_REAR_FLOWER)
        && !Double.isNaN(inFlowerS)) {
      // In the Flower, wiggling left and right for the intake to take every ball
      double[] wiggle = AutoSequence.flowerWiggle(inFlowerS);
      robot.setAutoHeading(lockedHeading(step) + wiggle[0], wiggle[1]);
      Logger.recordOutput("Auto/HeadingTarget", lockedHeading(step) + wiggle[0]);
    }

    Logger.recordOutput("Auto/TimeS", t);
    Logger.recordOutput("Auto/Step", sequence.step().name());
    Logger.recordOutput("Auto/BallsShot", sequence.ballsShot());
    Logger.recordOutput("Auto/AimCell", aimCell().name());
    Logger.recordOutput("Auto/PredictedCell", tips.cell().name());
    Logger.recordOutput("Auto/SkipSecondFlower", sequence.isSkippingSecondFlower());
    Logger.recordOutput("Auto/FollowerMode", follower.mode().name());
    Logger.recordOutput("Auto/ArmDown", sequence.armDown());
    Logger.recordOutput("Auto/ShotSpotCell", shotSpotCell == null ? "" : shotSpotCell.name());
  }

  /**
   * The step's drive is done. Turning in place to the Pollen Pedro only holds there, so that is done once the robot
   * is on the spot and facing it: the sweep then starts facing the Pollen, and the arm doesn't swing down mid-turn.
   */
  private boolean stepArrived() {
    AutoSequence.Step step0 = sequence.step();
    if ((step0 == AutoSequence.Step.INTAKE_WALL_FLOWER || step0 == AutoSequence.Step.INTAKE_REAR_FLOWER)
        && clock.seconds() - stepStartS >= FLOWER_STALL_AFTER_S && speedMps <= FLOWER_STALLED_MPS) {
      return true; // Pushed up against the Flower short of the intake point: in it (14 Nov 212840 stopped 12 cm short)
    }
    if (step0 == AutoSequence.Step.SWEEP_GARDEN && clock.seconds() - stepStartS >= FLOWER_STALL_AFTER_S
        && speedMps <= WALL_STOPPED_MPS) {
      return true; // In the corner: the sweep drives past our wall, so the wall ends it
    }
    if (!sequence.isFacingStep()) {
      return !follower.following();
    }
    AutoSequence.Step step = sequence.step();
    double[] stop = AutoField.forAlliance(holdSpot(step), alliance);
    org.firstinspires.ftc.teamcode.utils.Pose pose = robot.getPose();
    if (step == AutoSequence.Step.ONTO_GARDEN_WALL && clock.seconds() - stepStartS >= WALL_SETTLE_S
        && speedMps <= WALL_STOPPED_MPS) {
      return true; // Stopped by the wall, wherever odometry puts it
    }
    double within = step == AutoSequence.Step.ONTO_GARDEN_WALL ? ONTO_WALL_M : FACE_FLOWER_M;
    return Math.hypot(pose.xM - stop[0], pose.yM - stop[1]) <= within
        && Math.abs(AutoField.normalize(pose.headingRad - stop[2])) <= FACE_FLOWER_RAD;
  }

  /** The Cell to aim at: the plan's for the last shooting step started, the start's up Cell before the first. */
  private VisionCorrector.Cell aimCell() {
    return shotSpotCell != null ? shotSpotCell : firstCell;
  }

  /** Starts the drive for a step, from where the robot is now. */
  private void startStep(AutoSequence.Step step) {
    org.firstinspires.ftc.teamcode.utils.Pose now = robot.getPose();
    Pose here = new Pose(now.xM / AutoField.M_PER_IN, now.yM / AutoField.M_PER_IN, now.headingRad);
    boolean slow = step == AutoSequence.Step.SWEEP_GARDEN || step == AutoSequence.Step.INTAKE_WALL_FLOWER
        || step == AutoSequence.Step.INTAKE_REAR_FLOWER;
    drivetrain.setScale(slow ? SWEEP_POWER_SCALE : 1.0);
    boolean onWall = step == AutoSequence.Step.ONTO_GARDEN_WALL || step == AutoSequence.Step.SWEEP_GARDEN;
    drivetrain.setStrafeBias(onWall ? WALL_PUSH : 0.0);
    drivetrain.setForwardBias(0.0);
    robot.setAutoHeading(lockedHeading(step));
    Logger.recordOutput("Auto/HeadingTarget", lockedHeading(step));
    switch (step) {
      case SHOOT_PRELOAD: // To the spot in front of whichever Cell is up now, back (shooter) to it
      case SHOOT_GARDEN:
      case SHOOT_WALL_FLOWER:
        shotSpotCell = AutoSequence.plannedCell(firstCell, step);
        Pose spot = AutoField.pedroShotSpot(alliance, shotSpotCell);
        double[] control = AutoField.shotSpotControl(step, AutoField.forAlliance(AutoField.shotSpot(alliance, shotSpotCell),
            alliance));
        if (control == null || here.distance(spot) < MIN_PATH_IN) {
          turningLine(here, spot, true);
        } else { // Round our wall's Flower
          follower.follow(Paths.curve(here, pedro(control, alliance), spot).linear(here.heading(), spot.heading()));
        }
        break;
      case DRIVE_TO_GARDEN: { // Turning on the way to face along the Pollen line
        Pose garden = pedro(AutoField.GARDEN_IN, alliance);
        double[] bend = AutoField.gardenEntryControl(AutoField.forAlliance(new double[] {now.xM, now.yM, now.headingRad},
            alliance)); // forAlliance turns blue to red as well as red to blue
        if (bend == null || here.distance(garden) < MIN_PATH_IN) {
          turningLine(here, garden, true);
        } else { // From the far Cell's spot, below the Hive
          follower.follow(Paths.curve(here, pedro(bend, alliance), garden).linear(here.heading(), garden.heading()));
        }
        break;
      }
      case FACE_GARDEN: // Intake first along the Pollen line, clear of the wall to turn
        follower.hold(pedro(AutoField.GARDEN_IN, alliance));
        break;
      case ONTO_GARDEN_WALL: // Sideways onto the audience wall, not turning
        follower.hold(pedro(AutoField.GARDEN_WALL, alliance));
        break;
      case SWEEP_GARDEN: { // Along the wall, against it, from the line's start: Pedro closes any gap left
        Pose end = pedro(AutoField.gardenSweepEnd(), alliance);
        follower.follow(Paths.line(pedro(AutoField.GARDEN_WALL, alliance), end).constant(end.heading()));
        break;
      }
      case DRIVE_TO_WALL_FLOWER:
        turningLine(here, pedro(AutoField.WALL_FLOWER, alliance), true);
        break;
      case LOWER_ARM_WALL: // Faces the Flower where the drive stopped; the arm goes down once it does
        follower.hold(pedro(AutoField.WALL_FLOWER, alliance));
        break;
      case INTAKE_WALL_FLOWER: // Arm down, straight in still facing it
        turningLine(here, pedro(AutoField.WALL_FLOWER_INTAKE, alliance), false);
        break;
      case BACK_FROM_WALL_FLOWER: // Arm still down, straight back out to the stop
        turningLine(here, pedro(AutoField.WALL_FLOWER, alliance), false);
        break;
      case DRIVE_TO_REAR_FLOWER: {
        Pose end = pedro(AutoField.REAR_FLOWER, alliance);
        if (here.distance(end) < MIN_PATH_IN) {
          follower.hold(end.withHeading(here.heading()));
        } else {
          follower.follow(Paths.curve(here, pedro(AutoField.REAR_CONTROL, alliance), end)
              .linear(here.heading(), end.heading()));
        }
        break;
      }
      case LOWER_ARM_REAR:
        follower.hold(pedro(AutoField.REAR_FLOWER, alliance));
        break;
      case INTAKE_REAR_FLOWER:
        turningLine(here, pedro(AutoField.REAR_FLOWER_INTAKE, alliance), false);
        break;
      case BACK_FROM_REAR_FLOWER:
        turningLine(here, pedro(AutoField.REAR_FLOWER, alliance), false);
        break;
      case RAISE_ARM_WALL: // Still while the arm comes up
      case RAISE_ARM_REAR:
        follower.hold(here);
        break;
      case DRIVE_TO_PARK: // Back (shooter) to the Hive, to shoot what it holds once parked
        shotSpotCell = AutoSequence.plannedCell(firstCell, AutoSequence.Step.PARKED);
        turningLine(here, AutoField.pedroParkSpot(alliance, shotSpotCell), true);
        break;
      case PARKED: // Against our wall, turned to the Cell, shooting until the end
        shotSpotCell = AutoSequence.plannedCell(firstCell, step);
        follower.hold(AutoField.pedroParkSpot(alliance, shotSpotCell));
        break;
    }
  }

  /**
   * The heading the heading lock holds for a step taken at a fixed heading (the Garden, a Flower), or NaN to leave
   * the turn to Pedro on the drives between places.
   */
  private double lockedHeading(AutoSequence.Step step) {
    switch (step) {
      case FACE_GARDEN:
      case ONTO_GARDEN_WALL:
      case SWEEP_GARDEN:
        return AutoField.forAlliance(AutoField.GARDEN_IN, alliance)[2];
      case LOWER_ARM_WALL:
      case INTAKE_WALL_FLOWER:
      case BACK_FROM_WALL_FLOWER:
      case RAISE_ARM_WALL:
        return AutoField.forAlliance(AutoField.WALL_FLOWER, alliance)[2];
      case LOWER_ARM_REAR:
      case INTAKE_REAR_FLOWER:
      case BACK_FROM_REAR_FLOWER:
      case RAISE_ARM_REAR:
        return AutoField.forAlliance(AutoField.REAR_FLOWER, alliance)[2];
      default:
        return Double.NaN;
    }
  }

  /** Where a step that holds a spot (red) holds it. */
  private static double[] holdSpot(AutoSequence.Step step) {
    switch (step) {
      case FACE_GARDEN:
        return AutoField.GARDEN_IN;
      case ONTO_GARDEN_WALL:
        return AutoField.GARDEN_WALL;
      case LOWER_ARM_WALL:
        return AutoField.WALL_FLOWER;
      default:
        return AutoField.REAR_FLOWER;
    }
  }

  /** A straight drive to the end's heading: turning on the way, or facing it from the start. */
  private void turningLine(Pose here, Pose end, boolean turnOnTheWay) {
    if (here.distance(end) < MIN_PATH_IN) {
      follower.hold(end);
      return;
    }
    Path line = Paths.line(here, end);
    follower.follow(turnOnTheWay ? line.linear(here.heading(), end.heading()) : line.constant(end.heading()));
  }

  private void addTelemetry(double t) {
    telemetry.addData("Auto", String.format(java.util.Locale.US, "%s  %.1f s  %d shot  aim %s%s",
        sequence.step(), t, sequence.ballsShot(), aimCell(),
        sequence.isSkippingSecondFlower() ? "  (skipping 2nd Flower)" : ""));
    if (!Constants.TUNED) {
      telemetry.addData("WARNING", "Pedro not tuned");
    }
  }

  /** Stops every mechanism and ends the OpMode. */
  private void stopAll() {
    stopped = true;
    robot.setAutoShoot(false);
    robot.setAutoIntake(false);
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
