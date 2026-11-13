package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
  public static double PARK_WAYPOINT_M = 0.12; // Close enough to a turning point to head for the next
  public static double PARK_POWER = 1.0;
  public static double PARK_GAIN = 2.5; // Stick per metre to go, so it slows only in the last 0.4 m
  public static double PARK_TIMEOUT_S = 7.0;
  // Never less than this until there: 1.5 x 0.15 m to go gave 0.2, which didn't move the turned robot, and it sat
  // 0.15 m short for 5 s (13 Nov, 193927)
  public static double PARK_MIN_POWER = 0.45;
  public static double PARK_DONE_M = 0.06;
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
  private double autoStartS;

  @Override
  protected int getRlogPort() {
    return Main.LIVE_LOGGING ? Main.RLOG_PORT : 0;
  }

  @Override
  protected void onPsiKitConfigureLogging() {
    Main.configureLogging();
  }

  @Override
  protected void onPsiKitInit() {
    robot = new Robot(hardwareMap, telemetry, gamepad1);
    robot.init();
    robot.setVolleyMode(true);
  }

  @Override
  protected void onPsiKitInitLoop() {
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
          next(Phase.PARK, nowS);
        }
        break;
      }
      case PARK: {
        double f = robot.driverForwardHeadingRad();
        double[] stick;
        boolean there;
        if (robot.isPoseKnown() && pose != null && parkRoute != null) {
          double[] goal = parkRoute[parkLeg];
          double d = Math.hypot(goal[0] - pose.xM, goal[1] - pose.yM);
          boolean last = parkLeg == parkRoute.length - 1;
          if (!last && d <= PARK_WAYPOINT_M) {
            parkLeg++;
            goal = parkRoute[parkLeg];
            d = Math.hypot(goal[0] - pose.xM, goal[1] - pose.yM);
            last = parkLeg == parkRoute.length - 1;
          }
          double dx = goal[0] - pose.xM;
          double dy = goal[1] - pose.yM;
          there = last && d <= PARK_DONE_M;
          double mag = last ? parkStick(d) : PARK_POWER; // Full speed through the turning points
          stick = there || d < 1e-6 ? new double[] {0, 0} : new double[] {mag * dx / d, mag * dy / d};
        } else { // Not placed: straight back to the wall
          stick = new double[] {-PARK_POWER * Math.cos(f), -PARK_POWER * Math.sin(f)};
          there = inPhaseS >= DRIVE_TIMEOUT_S + 1.0;
        }
        robot.setDriveOverride(stick);
        Logger.recordOutput("Auto/ParkLeg", parkLeg);
        if (there || inPhaseS >= PARK_TIMEOUT_S) {
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

  /** Stick toward the park point d metres away: slowing in, but never below PARK_MIN_POWER. */
  static double parkStick(double d) {
    return Math.max(PARK_MIN_POWER, Math.min(PARK_POWER, PARK_GAIN * d));
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
