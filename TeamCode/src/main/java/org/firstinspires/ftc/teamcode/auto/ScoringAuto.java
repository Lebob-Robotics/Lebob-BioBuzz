package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.auto.AutoField.pedro;

import com.bylazar.panels.Panels;
import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
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
 * pose (odometry corrected by the Hive tags) and the Robot drives with Pedro's powers; while it shoots, its heading
 * lock turns the robot instead of Pedro. No gamepad is read (G401).
 *
 * <p>Select, INIT and start it from the Driver Station; the Driver Station's timer ends it at 30 s, and it stops
 * itself at {@link #STOP_ALL_S} in case. INIT homes the hood, then nothing moves until start (G304).
 */
public abstract class ScoringAuto extends PsiKitIterativeOpMode {
  /** Everything stops and the OpMode ends here, before the 30 s AUTO ends (G403). */
  public static double STOP_ALL_S = 29.5;
  /** Pedro's powers are scaled by this during the Garden sweep, so the intake takes the Pollen in. */
  public static double SWEEP_POWER_SCALE = 0.5;
  /** Shorter than this, a drive is held at its end instead: Pedro can't follow a path of no length. */
  private static final double MIN_PATH_IN = 1.0;

  private final Alliance alliance;
  private final RobotLocalizer localizer = new RobotLocalizer();
  private final CapturedDrivetrain drivetrain = new CapturedDrivetrain();
  private final ElapsedTime clock = new ElapsedTime();
  private Robot robot;
  private Follower follower;
  private AutoSequence sequence;
  private TipPredictor tips;
  /** The robot's heading this loop. While shooting, the paths "want" whatever heading the robot has. */
  private double headingNowRad;
  private final Interpolator aimedHeading = (curve, t) -> headingNowRad;
  private boolean stopped;

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
    robot.setAlliance(alliance);
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
    headingNowRad = start[2];
    localizer.set(start[0], start[1], start[2], 0, 0, 0);
    robot.setAutoDriver(this::pedroPowers);
    robot.setDriveBrake(true); // Pedro's constants are measured on BRAKE (13 Nov)
    sequence = new AutoSequence(0);
    tips = new TipPredictor(robot.getUpCell());
    sequence.takeNewStep();
    startStep(sequence.step());
    robot.setAutoShoot(sequence.shoot(0));
    robot.setAutoIntake(sequence.intake());
  }

  /** Called by the Robot each loop with the corrected pose, before it drives. */
  private double[] pedroPowers(org.firstinspires.ftc.teamcode.utils.Pose pose, double[] velocity) {
    headingNowRad = pose.headingRad;
    localizer.set(pose.xM, pose.yM, pose.headingRad, velocity[0], velocity[1], velocity[2]);
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
    robot.setAimPointOverride(VisionCorrector.upCellAimPoint(alliance, tips.cell()));
    addTelemetry(t);
    robot.periodic();

    boolean ballLeft = robot.ballLeftThisLoop();
    if (ballLeft) {
      tips.onBall(t);
    }
    sequence.update(t, !follower.following(), ballLeft, robot.isFeederEmpty());
    if (sequence.takeNewStep()) {
      startStep(sequence.step());
    }
    robot.setAutoShoot(sequence.shoot(t));
    robot.setAutoIntake(sequence.intake());

    Logger.recordOutput("Auto/TimeS", t);
    Logger.recordOutput("Auto/Step", sequence.step().name());
    Logger.recordOutput("Auto/BallsShot", sequence.ballsShot());
    Logger.recordOutput("Auto/AimCell", tips.cell().name());
    Logger.recordOutput("Auto/SkipSecondFlower", sequence.isSkippingSecondFlower());
    Logger.recordOutput("Auto/FollowerMode", follower.mode().name());
  }

  /** Starts the drive for a step, from where the robot is now. */
  private void startStep(AutoSequence.Step step) {
    org.firstinspires.ftc.teamcode.utils.Pose now = robot.getPose();
    Pose here = new Pose(now.xM / AutoField.M_PER_IN, now.yM / AutoField.M_PER_IN, now.headingRad);
    drivetrain.setScale(step == AutoSequence.Step.SWEEP_GARDEN ? SWEEP_POWER_SCALE : 1.0);
    switch (step) {
      case DRIVE_TO_GARDEN:
        aimedLine(here, pedro(AutoField.GARDEN_IN, alliance));
        break;
      case SWEEP_GARDEN:
        turningLine(here, pedro(AutoField.GARDEN_OUT, alliance), false);
        break;
      case DRIVE_TO_WALL_FLOWER:
        aimedLine(here, pedro(AutoField.WALL_FLOWER, alliance));
        break;
      case INTAKE_WALL_FLOWER:
        turningLine(here, pedro(AutoField.WALL_FLOWER_INTAKE, alliance), true);
        break;
      case DRIVE_TO_REAR_FLOWER: {
        Pose end = pedro(AutoField.REAR_FLOWER, alliance);
        if (here.distance(end) < MIN_PATH_IN) {
          follower.hold(end.withHeading(here.heading()));
        } else {
          follower.follow(Paths.curve(here, pedro(AutoField.REAR_CONTROL, alliance), end).heading(aimedHeading));
        }
        break;
      }
      case INTAKE_REAR_FLOWER:
        turningLine(here, pedro(AutoField.REAR_FLOWER_INTAKE, alliance), true);
        break;
      case DRIVE_TO_PARK:
        aimedLine(here, pedro(AutoField.PARK, alliance));
        break;
      case PARKED:
        follower.hold(pedro(AutoField.PARK, alliance));
        break;
    }
  }

  /** A straight drive on which the heading is left to the aim. */
  private void aimedLine(Pose here, Pose end) {
    if (here.distance(end) < MIN_PATH_IN) {
      follower.hold(end.withHeading(here.heading()));
      return;
    }
    follower.follow(Paths.line(here, end).heading(aimedHeading));
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
        sequence.step(), t, sequence.ballsShot(), tips.cell(),
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
