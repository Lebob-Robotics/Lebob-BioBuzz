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
import org.firstinspires.ftc.teamcode.VolleySettings;
import org.firstinspires.ftc.teamcode.field.Angles;
import org.firstinspires.ftc.teamcode.field.Units;
import org.firstinspires.ftc.teamcode.pedro.CapturedDrivetrain;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.RobotLocalizer;
import org.firstinspires.ftc.teamcode.utils.ShootOnTheMoveConstants;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.PsiKitIterativeOpMode;
import org.firstinspires.ftc.teamcode.field.Cell;

/**
 * The corner autonomous ({@link CornerSequence}), for one alliance: preloads from the rear corner, a wait there, then
 * our wall's Flower, a volley into the AUDIENCE Cell from in front of it, and park, shooting what's left. Driven as {@link ScoringAuto} drives: Pedro on the
 * Robot's pose, each volley shot stopped with the heading lock turning the robot. No gamepad is read (G401).
 */
public abstract class CornerAuto extends PsiKitIterativeOpMode {
  /** Shorter than this, a drive is held at its end instead: Pedro can't follow a path of no length. */
  private static final double MIN_PATH_IN = 1.0;

  private final Alliance alliance;
  private final RobotLocalizer localizer = new RobotLocalizer();
  private final CapturedDrivetrain drivetrain = new CapturedDrivetrain();
  private final ElapsedTime clock = new ElapsedTime();
  private Robot robot;
  private boolean odometryReady;
  private Follower follower;
  private CornerSequence sequence;
  private boolean stopped;
  private VolleySettings volley;
  private boolean shotLastLoop; // The volley's settings are picked only before it starts, never mid-burst
  private double speedMps;
  private double stepStartS;

  protected CornerAuto(Alliance alliance) {
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
    robot.setVolleyMode(true);
    robot.setAutoIndexerReverse(true); // CornerSequence's jiggle
    robot.setAlliance(alliance);
    robot.setHandsOffPose(true);
    robot.init();
    if (ShootOnTheMoveConstants.VOLLEY_USE_TUNING_CSV) {
      volley = VolleySettings.loadTuned(); // Reads a file only: INIT moves nothing
    }
    follower = new Follower(localizer, drivetrain, Constants.algorithm());
  }

  @Override
  protected void onPsiKitInitLoop() {
    if (robot == null) {
      return;
    }
    telemetry.addData("Auto", alliance + " CORNER: robot in the rear corner, touching our wall and the rear wall,"
        + " intake to our wall, outside the loading zone");
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
    double[] start = AutoField.forAlliance(CornerSequence.CORNER, alliance);
    robot.startAt(start[0], start[1], start[2]);
    localizer.set(start[0], start[1], start[2], 0, 0, 0);
    robot.setAutoDriver(this::pedroPowers);
    robot.setDriveBrake(true); // Pedro's constants are measured on BRAKE (13 Nov)
    sequence = new CornerSequence(0);
    sequence.takeNewStep();
    startStep(sequence.step());
    robot.setAutoShoot(sequence.shoot(0));
    robot.setAutoIntake(sequence.intake());
  }

  /** Called by the Robot each loop with the corrected pose, before it drives. */
  private double[] pedroPowers(org.firstinspires.ftc.teamcode.utils.Pose pose, double[] velocity) {
    if (!odometryReady) { // Still while the Pinpoint finishes the reset from INIT (13 Nov)
      odometryReady = robot.isOdometryReady();
      Logger.recordOutput("Auto/WaitingForOdometry", !odometryReady);
      if (!odometryReady) {
        return null;
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
    if (t >= ScoringAuto.STOP_ALL_S) {
      stopAll();
      return;
    }
    double[] aimPoint = AutoField.autoAimPoint(alliance, aimCell());
    robot.setAimPointOverride(aimPoint);
    if (volley != null && !shotLastLoop) {
      int spot = volley.spotFor(robot.getPose(), aimPoint[0], aimPoint[1]);
      VolleySettings.apply(robot, volley.get(spot));
      Logger.recordOutput("Auto/VolleySpot", spot);
    }
    addTelemetry(t);
    robot.periodic();

    boolean ballLeft = robot.ballLeftThisLoop();
    sequence.update(t, stepArrived(), ballLeft, robot.isFeederEmpty(), robot.isStackFull());
    if (sequence.takeNewStep()) {
      stepStartS = t;
      startStep(sequence.step());
    }
    shotLastLoop = sequence.shoot(t);
    robot.setAutoShoot(shotLastLoop);
    robot.setAutoIntake(sequence.intake());
    robot.setAutoIndexerPaused(sequence.indexerPaused(t));
    robot.ramp.setDeployed(sequence.armDown());
    double inFlowerS = sequence.arrivedForS(t);
    if (sequence.step() == CornerSequence.Step.INTAKE_FLOWER && !Double.isNaN(inFlowerS)) {
      double[] wiggle = AutoSequence.flowerWiggle(inFlowerS); // For the intake to take every ball
      robot.setAutoHeading(wallHeading() + wiggle[0], wiggle[1]);
    }

    Logger.recordOutput("Auto/TimeS", t);
    Logger.recordOutput("Auto/Step", sequence.step().name());
    Logger.recordOutput("Auto/BallsShot", sequence.ballsShot());
    Logger.recordOutput("Auto/AimCell", aimCell().name());
    Logger.recordOutput("Auto/IndexerPaused", sequence.indexerPaused(t));
    Logger.recordOutput("Auto/FollowerMode", follower.mode().name());
    Logger.recordOutput("Auto/ArmDown", sequence.armDown());
  }

  /** The step's drive is done: for a facing step, on its spot and at its heading. */
  private boolean stepArrived() {
    CornerSequence.Step step = sequence.step();
    if (step == CornerSequence.Step.INTAKE_FLOWER && clock.seconds() - stepStartS >= ScoringAuto.FLOWER_STALL_AFTER_S
        && speedMps <= ScoringAuto.FLOWER_STALLED_MPS) {
      return true; // Pushed up against the Flower short of the intake point: in it
    }
    if (!sequence.isFacingStep()) {
      return !follower.following();
    }
    double[] stop = step == CornerSequence.Step.FACE_WALL ? CornerSequence.preloadSpot(alliance)
        : AutoField.forAlliance(AutoField.WALL_FLOWER, alliance);
    org.firstinspires.ftc.teamcode.utils.Pose pose = robot.getPose();
    return Math.hypot(pose.xM - stop[0], pose.yM - stop[1]) <= ScoringAuto.FACE_FLOWER_M
        && Math.abs(Angles.normalize(pose.headingRad - wallHeading())) <= ScoringAuto.FACE_FLOWER_RAD;
  }

  private Cell aimCell() {
    if (sequence == null) {
      return CornerSequence.nearCell(alliance);
    }
    int step = sequence.step().ordinal();
    if (step >= CornerSequence.Step.DRIVE_TO_PARK.ordinal()) {
      return CornerSequence.parkCell(alliance);
    }
    return step >= CornerSequence.Step.DRIVE_TO_FLOWER.ordinal()
        ? CornerSequence.gardenCell(alliance) : CornerSequence.nearCell(alliance);
  }

  /** Intake to our wall: the start's heading, the Flower's, and the one it waits at. */
  private double wallHeading() {
    return AutoField.forAlliance(CornerSequence.CORNER, alliance)[2];
  }

  /** Starts the drive for a step, from where the robot is now. */
  private void startStep(CornerSequence.Step step) {
    org.firstinspires.ftc.teamcode.utils.Pose now = robot.getPose();
    Pose here = new Pose(now.xM / Units.M_PER_IN, now.yM / Units.M_PER_IN, now.headingRad);
    boolean intoFlower = step == CornerSequence.Step.INTAKE_FLOWER;
    drivetrain.setScale(intoFlower ? ScoringAuto.SWEEP_POWER_SCALE : 1.0);
    drivetrain.setStrafeBias(0.0);
    drivetrain.setForwardBias(0.0);
    // Square to our wall everywhere but the AUDIENCE volley and the park, which Pedro turns on the way.
    // Shooting, the aim's lock turns the robot instead
    boolean turning = step == CornerSequence.Step.SHOOT_GARDEN_SIDE || step == CornerSequence.Step.DRIVE_TO_PARK
        || step == CornerSequence.Step.PARKED;
    robot.setAutoHeading(turning ? Double.NaN : wallHeading());
    Pose preload = poseOf(CornerSequence.preloadSpot(alliance));
    Pose flowerStop = pedro(AutoField.WALL_FLOWER, alliance);
    switch (step) {
      case SHOOT_PRELOAD: // Straight out of the corner without turning: turned in it, a corner would hit a wall
        line(here, preload, false);
        break;
      case FACE_WALL:
        follower.hold(preload);
        break;
      case BACK_TO_CORNER:
        line(here, pedro(CornerSequence.CORNER_WAIT, alliance), false);
        break;
      case WAIT_IN_CORNER:
        follower.hold(pedro(CornerSequence.CORNER_WAIT, alliance));
        break;
      case DRIVE_TO_FLOWER: { // Along our wall just off it, then up round the Flower to the stop short of it
        Pose edgeEnd = pedro(CornerSequence.EDGE_END, alliance);
        if (here.distance(edgeEnd) < MIN_PATH_IN) {
          follower.follow(Paths.curve(here, pedro(CornerSequence.FLOWER_BEND, alliance), flowerStop)
              .constant(flowerStop.heading()));
        } else {
          follower.follow(Paths.path(Paths.line(here, edgeEnd).constant(edgeEnd.heading()),
              Paths.curve(edgeEnd, pedro(CornerSequence.FLOWER_BEND, alliance), flowerStop)
                  .constant(flowerStop.heading())));
        }
        break;
      }
      case LOWER_ARM:
        follower.hold(flowerStop);
        break;
      case INTAKE_FLOWER: // Arm down, straight in still facing it
        line(here, pedro(AutoField.WALL_FLOWER_INTAKE, alliance), false);
        break;
      case BACK_FROM_FLOWER:
        line(here, flowerStop, false);
        break;
      case RAISE_ARM:
        follower.hold(here);
        break;
      case SHOOT_GARDEN_SIDE: // Back (shooter) to the AUDIENCE Cell, from the spot in front of it
        line(here, AutoField.pedroShotSpot(alliance, CornerSequence.gardenCell(alliance)), true);
        break;
      case DRIVE_TO_PARK: // Turning on the way, back to the park's Cell
        line(here, poseOf(CornerSequence.parkSpot(alliance)), true);
        break;
      case PARKED:
        follower.hold(poseOf(CornerSequence.parkSpot(alliance)));
        break;
    }
    Logger.recordOutput("Auto/HeadingTarget", turning ? Double.NaN : wallHeading());
  }

  private static Pose poseOf(double[] field) {
    return new Pose(field[0] / Units.M_PER_IN, field[1] / Units.M_PER_IN, field[2]);
  }

  /** A straight drive to the end's heading: turning on the way, or facing it from the start. */
  private void line(Pose here, Pose end, boolean turnOnTheWay) {
    if (here.distance(end) < MIN_PATH_IN) {
      follower.hold(end);
      return;
    }
    Path line = Paths.line(here, end);
    follower.follow(turnOnTheWay ? line.linear(here.heading(), end.heading()) : line.constant(end.heading()));
  }

  private void addTelemetry(double t) {
    telemetry.addData("Auto", String.format(java.util.Locale.US, "CORNER %s  %.1f s  %d shot  aim %s",
        sequence.step(), t, sequence.ballsShot(), aimCell()));
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
    robot.setAutoIndexerReverse(false);
    robot.ramp.setDeployed(false);
    robot.setAutoHeading(Double.NaN);
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
