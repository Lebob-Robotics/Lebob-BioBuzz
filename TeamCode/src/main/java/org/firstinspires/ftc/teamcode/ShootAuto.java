package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.pedro.CapturedDrivetrain;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.RobotLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.Pose;
import org.firstinspires.ftc.teamcode.utils.ShootOnTheMoveConstants;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.PsiKitIterativeOpMode;

/**
 * Drives forward (as the left stick pushed forward: away from the alliance's wall), aims at the Hive and fires the
 * whole load as one volley, as RB does in Lebot Teleop, then parks wholly in the LOADING ZONE: tile A5 against the
 * alliance's own wall, 0.915 m to the driver's left of its middle (red {-0.915, -1.52}, blue {+0.915, +1.52}), by a
 * lane clear of the Hive and the wall's Flower. It starts in the corner of its own wall and the side wall on the driver's right, square, the camera side toward the middle, and takes
 * that as its pose: it turns straight to its Hive from there, and the tags correct it once they come into view.
 * The park is Pedro Pathing's: one curve through the lane's corners, followed from the Robot's tag-corrected pose.
 * With the odometry faulted it backs straight to its own wall instead of parking by pose.
 * X = BLUE, B = RED in INIT.
 */
@Configurable
@Autonomous(name = "Shoot Auto", group = "Robot", preselectTeleOp = "Lebot Teleop")
public class ShootAuto extends PsiKitIterativeOpMode {
  // Placed square in the corner of its own wall and the side wall on the driver's right (the loading zone), the
  // camera side toward the middle, centre this far off each wall. It aims from there at once; the tags then correct it
  public static double START_OFF_WALL_M = 0.23;
  public static double START_OFF_SIDE_M = 0.23;
  public static double VISION_WAIT_S = 1.0; // Holds the first ball this long for the tags to correct the start pose
  public static double DRIVE_M = 0.6; // How far forward before shooting; 0 shoots from the start
  public static double DRIVE_POWER = 0.8;
  // Drives no longer than this: 0.6 m takes under a second. With the odometry faulted (FAULT_BAD_READ,
  // 13 Nov 191050 and 191103) the pose never moved, and the old 3 s let it drive blind
  public static double DRIVE_TIMEOUT_S = 1.5;
  public static double DRIVE_MAX_STEP_M = 0.12; // A bigger pose step in one loop is vision placing the robot, not driving
  public static double SETTLE_S = 0.3; // Stopped this long before aiming, so the pose and the steady gate settle
  public static double SEARCH_TURN = 0.25; // Turn power while looking for the Hive
  // Turns in steps, still in between: turning on at 0.25 reached 1 rad/s, past the 0.6 rad/s the cameras are used
  // up to, so it saw nothing once it started searching (13 Nov, 193125)
  public static double SEARCH_TURN_S = 0.3;
  public static double SEARCH_PAUSE_S = 0.6;
  public static int BALLS = 4;
  public static double PARK_BY_S = 22; // Gives up on the rest of the balls and parks this long after the start
  // The LOADING ZONE, from the game manual's field (feature/auto's AutoField): red tile A5, x -1.22 to -0.61 on the
  // -y wall, so its middle is 0.915 m to the driver's left of the wall's middle. Blue's is red's turned half a turn
  public static double PARK_LEFT_M = 0.915;
  public static double PARK_OFF_WALL_M = 0.31; // Robot centre this far off its own wall: wholly in the zone (1.52 m)
  // Out of the way to it: this far off the wall, clear of the Hive (robot centre |y| > 1.0) and of the Flower that
  // stands by our wall, then along to the zone and straight in
  public static double PARK_LANE_OFF_WALL_M = 0.73;
  // Pedro follows the park (Constants.foresightConfig: run the Foresight Tuner for real constants). Off: the
  // speed-planned drive below, leg by leg
  public static boolean PARK_WITH_PEDRO = true;
  public static double PARK_WAYPOINT_M = 0.12; // Close enough to a turning point to head for the next
  // Drives to a planned speed, not a stick: full stick reached 1.6 m/s along the lane and nothing slowed it in time,
  // so it ran 0.45 m past the zone, swung back past it again and parked 0.19 m off (13 Nov 200130 and 215133).
  // Planned speed = what still stops at the goal braking at PARK_BRAKE_MPS2 (full reverse stopped it at about 3.4)
  public static double PARK_MAX_SPEED_MPS = 1.6; // Full stick's top speed (13 Nov logs)
  public static double PARK_BRAKE_MPS2 = 2.0;
  public static double PARK_CORNER_MPS = 0.6; // Through the turning points
  // Extra stick per m/s off the planned speed: brakes when too fast, damps sideways slide, and pushes when stuck
  // (stick 0.2 didn't move the turned robot and it sat 0.15 m short for 5 s, 13 Nov 193927)
  public static double PARK_VEL_GAIN = 1.0;
  public static double PARK_TIMEOUT_S = 7.0;
  public static double PARK_DONE_M = 0.06;
  // And this slow: 220620 was "there" at 0.43 m/s along the wall, and the push carried it 0.08 m on, over the zone's edge
  public static double PARK_DONE_MPS = 0.15;
  public static double PARK_PUSH_POWER = 0.5; // Then into the wall this hard for PARK_PUSH_S (0.2 and 0.35 didn't move it, 13 Nov)
  public static double PARK_PUSH_S = 0.8;

  private enum Phase { DRIVE, SETTLE, SHOOT, PARK, PUSH, DONE }

  private Robot robot;
  private Phase phase = Phase.DRIVE;
  private double phaseStartS;
  private double lastX;
  private double lastY;
  private double drivenM;
  private int ballsOut;
  private double[][] parkRoute;
  private int parkLeg;
  private final RobotLocalizer localizer = new RobotLocalizer();
  private final CapturedDrivetrain drivetrain = new CapturedDrivetrain();
  private Follower follower;
  private boolean parkByPedro;
  private double autoStartS;

  @Override
  protected int getRlogPort() {
    return Main.LIVE_LOGGING ? Main.RLOG_PORT : 0;
  }

  @Override
  protected void onPsiKitConfigureLogging() {
    Main.configureLogging();
    Logger.recordMetadata("PedroTuned", String.valueOf(Constants.TUNED));
  }

  @Override
  protected void onPsiKitInit() {
    robot = new Robot(hardwareMap, telemetry, gamepad1);
    robot.init();
    robot.setVolleyMode(true);
    follower = new Follower(localizer, drivetrain, Constants.algorithm());
  }

  @Override
  protected void onPsiKitInitLoop() {
    if (PARK_WITH_PEDRO && !Constants.TUNED) {
      telemetry.addLine("Pedro park on guessed constants");
    }
    robot.initLoop();
  }

  @Override
  protected void onPsiKitStart() {
    robot.start();
    double f = robot.driverForwardHeadingRad();
    double[] start = cornerPoint(f, START_OFF_WALL_M, START_OFF_SIDE_M);
    robot.seedPose(start[0], start[1], f + Math.PI);
    robot.setSticksLocked(true);
    robot.setDriveBrake(true);
    phase = DRIVE_M > 0 ? Phase.DRIVE : Phase.SETTLE;
    phaseStartS = getRuntime();
    autoStartS = phaseStartS;
    lastX = Double.NaN;
    drivenM = 0;
  }

  @Override
  protected void onPsiKitLoop() {
    double nowS = getRuntime();
    double inPhaseS = nowS - phaseStartS;
    Pose pose = robot.getPose();
    switch (phase) {
      case DRIVE: {
        // Summed loop by loop, leaving out the jump when the tags first place the robot (13 Nov 191356: from
        // (0, 0) to (-1.46, 1.38) on one loop, which counted as the whole drive)
        if (pose != null) {
          drivenM += drivenStep(lastX, lastY, pose.xM, pose.yM);
          lastX = pose.xM;
          lastY = pose.yM;
        }
        Logger.recordOutput("Auto/DrivenM", drivenM);
        if (drivenM >= DRIVE_M || inPhaseS >= DRIVE_TIMEOUT_S) {
          robot.setDriveOverride(null);
          next(Phase.SETTLE, nowS);
        } else {
          double f = robot.driverForwardHeadingRad();
          robot.setDriveOverride(new double[] {DRIVE_POWER * Math.cos(f), DRIVE_POWER * Math.sin(f)});
        }
        break;
      }
      case SETTLE:
        robot.setDriveOverride(null);
        if (pose != null && inPhaseS < 0.05) {
          robot.setHoldPoint(new double[] {pose.xM, pose.yM});
        }
        if (inPhaseS >= SETTLE_S) {
          next(Phase.SHOOT, nowS);
        }
        break;
      case SHOOT: {
        boolean hiveKnown = robot.getLastAim() != null;
        // Turns to the Hive straight away from the start pose; fires once the tags have placed it, or after a wait
        robot.setScriptedVolley(hiveKnown && (robot.isPoseValid() || inPhaseS >= VISION_WAIT_S));
        boolean searchTurning = inPhaseS % (SEARCH_PAUSE_S + SEARCH_TURN_S) >= SEARCH_PAUSE_S;
        robot.setTurnOverride(hiveKnown || !searchTurning ? 0 : SEARCH_TURN);
        if (robot.ballLeftThisLoop()) {
          ballsOut++;
        }
        if (ballsOut >= BALLS || nowS - autoStartS >= PARK_BY_S) {
          robot.setScriptedVolley(false);
          robot.setTurnOverride(0);
          robot.setHoldPoint(null);
          parkRoute = pose == null ? null : parkRoute(robot.driverForwardHeadingRad(), pose.xM, pose.yM);
          parkLeg = 0;
          parkByPedro = PARK_WITH_PEDRO && parkRoute != null;
          if (parkByPedro) {
            double[] v = robot.getVelocity();
            localizer.set(pose.xM, pose.yM, pose.headingRad, v[0], v[1], v[2]);
            follower.follow(pedroParkPath(pose.xM, pose.yM, pose.headingRad, parkRoute));
          }
          next(Phase.PARK, nowS);
        }
        break;
      }
      case PARK: {
        double f = robot.driverForwardHeadingRad();
        double[] stick;
        boolean there;
        if (robot.isPoseKnown() && pose != null && parkRoute != null) {
          double[] v = robot.getVelocity();
          double[] park = parkRoute[parkRoute.length - 1];
          there = Math.hypot(park[0] - pose.xM, park[1] - pose.yM) <= PARK_DONE_M
              && Math.hypot(v[0], v[1]) <= PARK_DONE_MPS;
          if (parkByPedro) {
            localizer.set(pose.xM, pose.yM, pose.headingRad, v[0], v[1], v[2]);
            follower.update();
            double[] powers = drivetrain.powers();
            Logger.recordOutput("Auto/Pedro/Powers", powers);
            Logger.recordOutput("Auto/Pedro/Mode", follower.mode().name());
            stick = there ? new double[] {0, 0} : pedroStick(powers, pose.headingRad);
          } else {
            double[] goal = parkRoute[parkLeg];
            double d = Math.hypot(goal[0] - pose.xM, goal[1] - pose.yM);
            boolean last = parkLeg == parkRoute.length - 1;
            if (!last && d <= PARK_WAYPOINT_M) {
              parkLeg++;
              goal = parkRoute[parkLeg];
              last = parkLeg == parkRoute.length - 1;
            }
            stick = there ? new double[] {0, 0} : stickFor(
                parkStick(goal[0] - pose.xM, goal[1] - pose.yM, last ? 0 : PARK_CORNER_MPS, v[0], v[1]));
          }
        } else { // Not placed: straight back to the wall
          stick = new double[] {-Math.cos(f), -Math.sin(f)};
          there = inPhaseS >= DRIVE_TIMEOUT_S + 1.0;
        }
        robot.setDriveOverride(stick);
        Logger.recordOutput("Auto/ParkLeg", parkLeg);
        if (there || inPhaseS >= PARK_TIMEOUT_S) {
          follower.stop();
          next(Phase.PUSH, nowS);
        }
        break;
      }
      case PUSH: {
        double f = robot.driverForwardHeadingRad();
        robot.setDriveOverride(new double[] {-PARK_PUSH_POWER * Math.cos(f), -PARK_PUSH_POWER * Math.sin(f)});
        if (inPhaseS >= PARK_PUSH_S) {
          robot.setDriveOverride(null);
          next(Phase.DONE, nowS);
        }
        break;
      }
      case DONE:
      default:
        robot.setScriptedVolley(false);
        robot.setTurnOverride(0);
        robot.setDriveOverride(null);
        break;
    }
    Logger.recordOutput("Auto/Phase", phase.name());
    Logger.recordOutput("Auto/BallsOut", ballsOut);
    String allianceWarning = robot.allianceWarning();
    if (allianceWarning != null) {
      telemetry.addLine(allianceWarning);
    }
    telemetry.addData("Auto", "%s  balls out %d/%d", phase, ballsOut, BALLS);
    robot.periodic();
  }

  /**
   * Field stick toward a goal {dx, dy} away, arriving at endSpeedMps, from field velocity {vx, vy}: the planned
   * velocity as stick, plus PARK_VEL_GAIN times how far the actual velocity is off it. At most 1 long.
   */
  static double[] parkStick(double dx, double dy, double endSpeedMps, double vx, double vy) {
    double d = Math.hypot(dx, dy);
    if (d < 1e-6) {
      return new double[] {0, 0};
    }
    double speed = Math.min(PARK_MAX_SPEED_MPS, Math.sqrt(endSpeedMps * endSpeedMps + 2 * PARK_BRAKE_MPS2 * d));
    double wantX = speed * dx / d;
    double wantY = speed * dy / d;
    double sx = wantX / PARK_MAX_SPEED_MPS + PARK_VEL_GAIN * (wantX - vx);
    double sy = wantY / PARK_MAX_SPEED_MPS + PARK_VEL_GAIN * (wantY - vy);
    double mag = Math.hypot(sx, sy);
    return mag > 1 ? new double[] {sx / mag, sy / mag} : new double[] {sx, sy};
  }

  /**
   * Pedro's park: one curve from {x, y} with the route's turning points as its control points, so it rounds the
   * corners instead of stopping at them, and stays inside them (a Bezier curve stays within its control points),
   * clear of the Hive and the Flower. Each corner counts twice, so the curve rounds it tightly instead of cutting
   * toward the Hive's corner. The heading stays as it is. Pedro's units: inches.
   */
  static Path pedroParkPath(double xM, double yM, double headingRad, double[][] route) {
    java.util.List<com.pedropathing.math.Pose> points = new java.util.ArrayList<>();
    points.add(inches(xM, yM, headingRad));
    for (int i = 0; i < route.length; i++) {
      points.add(inches(route[i][0], route[i][1], headingRad));
      if (i < route.length - 1) {
        points.add(inches(route[i][0], route[i][1], headingRad));
      }
    }
    return Paths.curve(points.toArray(new com.pedropathing.math.Pose[0])).constant(headingRad);
  }

  private static com.pedropathing.math.Pose inches(double xM, double yM, double headingRad) {
    return new com.pedropathing.math.Pose(xM / RobotLocalizer.M_PER_IN, yM / RobotLocalizer.M_PER_IN, headingRad);
  }

  /**
   * The field stick for Pedro's robot-frame {forward, left, turn} on a robot at headingRad. The turn is left out:
   * the Robot's heading hold keeps the heading, as the path asks.
   */
  static double[] pedroStick(double[] powers, double headingRad) {
    double c = Math.cos(headingRad);
    double s = Math.sin(headingRad);
    return stickFor(new double[] {powers[0] * c - powers[1] * s, powers[0] * s + powers[1] * c});
  }

  /**
   * The field stick that gives this field power. The Robot curves a scripted stick as it does the driver's
   * (STICK_EXPONENT 2), so without this half power came out as about a quarter: the park's braking was mostly lost.
   */
  static double[] stickFor(double[] power) {
    double m = Math.hypot(power[0], power[1]);
    if (m < 1e-9) {
      return new double[] {0, 0};
    }
    double scale = MecanumDriveSubsystem.stickForPower(Math.min(1.0, m)) / m;
    return new double[] {power[0] * scale, power[1] * scale};
  }

  /** One loop's pose step, or 0 for the first pose or a jump bigger than DRIVE_MAX_STEP_M. */
  static double drivenStep(double lastX, double lastY, double x, double y) {
    if (Double.isNaN(lastX)) {
      return 0;
    }
    double step = Math.hypot(x - lastX, y - lastY);
    return step <= DRIVE_MAX_STEP_M ? step : 0;
  }

  /**
   * The loading zone's park point {x, y}: the corner of the alliance's own wall (behind the driver's forward f) and the
   * side wall on the driver's right. Red (f = +y): {+1.53, -1.53}; blue (f = -y): {-1.53, +1.53}.
   */
  static double[] parkPoint(double f) {
    return cornerPoint(f, PARK_OFF_WALL_M, ShootOnTheMoveConstants.FIELD_HALF_SIZE_M + PARK_LEFT_M);
  }

  /**
   * From {x, y} to the park point: out to the lane PARK_LANE_OFF_WALL_M off our wall, along it to the zone, then in.
   * Leaves out the first turning point when already at the lane or nearer the wall.
   */
  static double[][] parkRoute(double f, double x, double y) {
    double[] park = parkPoint(f);
    double[] laneAtZone = cornerPoint(f, PARK_LANE_OFF_WALL_M, ShootOnTheMoveConstants.FIELD_HALF_SIZE_M + PARK_LEFT_M);
    // Our position along the wall (driver's right) and off it, in the same terms as cornerPoint
    double fx = Math.cos(f);
    double fy = Math.sin(f);
    double offWall = ShootOnTheMoveConstants.FIELD_HALF_SIZE_M + (x * fx + y * fy);
    double right = x * fy - y * fx;
    if (offWall <= PARK_LANE_OFF_WALL_M + PARK_WAYPOINT_M) {
      return new double[][] {laneAtZone, park};
    }
    double[] laneHere = cornerPoint(f, PARK_LANE_OFF_WALL_M, ShootOnTheMoveConstants.FIELD_HALF_SIZE_M - right);
    return new double[][] {laneHere, laneAtZone, park};
  }

  /** {x, y} offWallM off the own wall (behind forward f) and offSideM off the side wall on the driver's right. */
  static double[] cornerPoint(double f, double offWallM, double offSideM) {
    double fx = Math.cos(f);
    double fy = Math.sin(f);
    double back = ShootOnTheMoveConstants.FIELD_HALF_SIZE_M - offWallM;
    double right = ShootOnTheMoveConstants.FIELD_HALF_SIZE_M - offSideM;
    // Right of forward is forward turned clockwise: (fy, -fx)
    return new double[] {right * fy - back * fx, -right * fx - back * fy};
  }

  private void next(Phase p, double nowS) {
    phase = p;
    phaseStartS = nowS;
  }

  @Override
  protected void onPsiKitStop() {
    if (robot != null) {
      robot.setScriptedVolley(false);
      robot.stop();
    }
  }
}
