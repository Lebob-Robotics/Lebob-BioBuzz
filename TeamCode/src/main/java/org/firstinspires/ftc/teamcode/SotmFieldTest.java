package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.utils.Pose;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.PsiKitIterativeOpMode;

/**
 * Shoot-on-the-move field test: the whole TeleOp ({@link Robot}) driven by a scripted driver instead of a
 * gamepad. It runs laps until stopped, red then blue then red: each lap runs every one of {@link #TRIALS} against
 * that alliance's up Cell, the paths mirrored to that Cell's side of the Hive, and drives around the Hive between
 * them. For each trial it drives to the start, lines up on the Cell
 * standing still, lets go of RT, then presses RT again as it sets off along the path, and holds it a moment
 * after stopping. Read the log for how soon each moving shot is ready and how well the hood, wheels and heading
 * keep up. No balls are needed: the feeder runs as it would to fire.
 *
 * <p>The robot centre stays within {@link #FIELD_LIMIT_M} of the field centre and out of the Hive's footprint
 * ({@link #HIVE_HALF_X_M}, {@link #HIVE_HALF_Y_M}); no trial path crosses it, and the drives between them go round
 * its corners ({@link #route}). Stops for good if the pose, or where it would stop ({@link #stopDistance}), leaves that
 * area, or if the pose isn't placed by the tags. Waits for the tags first,
 * turning on the spot if they aren't in view. Any stick or button on the Driver Hub's gamepad also stops it, and
 * so does the robot moving against the direction it is driven (a sign error would otherwise run it to the fence).
 */
@TeleOp(name = "SOTM Field Test", group = "Tuning")
public class SotmFieldTest extends PsiKitIterativeOpMode {
  /**
   * Robot centre limits, field metres: 0.25 m inside the perimeter, and out of the Hive's footprint. At 0.15 m a
   * pose that went 0.4 m wrong took the robot off the mat (5 Nov, 081516).
   */
  static final double FIELD_LIMIT_M = 1.35;
  static final double HIVE_HALF_X_M = 0.75;
  static final double HIVE_HALF_Y_M = 0.95;
  static final double PREDICT_S = 0.15; // Reaction before braking starts
  static final double BRAKE_MPS2 = 2.0; // Braking assumed, for the stopping distance
  private static final double FENCE_SLACK_M = 0.05; // Allowed past the box now; the prediction gets twice this
  // Extra allowed past the field edge only: the trials keep 0.1 m more margin, not a fence 0.1 m nearer. A 0.1 m
  // tag fix while standing aiming 0.05 inside ended the run (5 Nov, 080149)
  private static final double EDGE_FENCE_EXTRA_M = 0.10;
  /** Corners of the drive round the Hive, robot centre: clear of its footprint, inside the field limit. */
  private static final double ROUTE_X_M = 1.10;
  private static final double ROUTE_Y_M = 1.15;
  private static final double ROUTE_CLEARANCE_M = 0.05; // Added to the Hive footprint for the drive's segments
  private static final double ROUTE_WAYPOINT_TOLERANCE_M = 0.15;
  private static final int MAX_LAPS = 1000; // Runs until stopped

  /**
   * {from x, from y, to x, to y, stick 0..1}, for a Cell on the -x, -y side (red SCORING's aim point is about
   * -0.44, -0.32). Each lap mirrors x and y to the side its Cell is on.
   */
  private static final double[][] TRIALS = {
      {-1.15, -0.32, -1.15, -0.32, 0}, // Standing still
      // Full stick is full speed while aiming. Ends at least 1.0 from the middle: 0.85 let the robot coast into the
      // Hive's margin. Full-speed runs end 0.3 short of the fence: ending at 1.30 the robot was still doing
      // 0.70 m/s at 1.34 and its predicted stop left the fence
      {-1.15, 1.20, -1.15, -1.05, 1.0}, // Across the whole field
      {-1.05, -1.20, -1.05, 1.05, 1.0}, // Back across, nearer
      // Outermost x 0.13 inside the fence: starts 0.05 inside overshot it by 0.06 driving there (5 Nov, 084933)
      {-1.22, 1.20, -1.22, -1.05, 0.6}, // Across again, slower and further out
      {-1.22, 1.20, -1.00, -0.60, 1.0}, // Diagonally in from the far corner
      {-1.00, -0.60, -1.22, 1.05, 1.0}, // And back out
      {-1.22, -1.20, -1.00, 0.20, 1.0}, // In from the near corner
      {-1.22, -0.32, -1.00, -0.32, 1.0}, // Straight at the Cell
      {-1.00, -0.32, -1.20, -0.32, 1.0}, // Straight away: ending 0.05 inside the fence it coasted into it
      {-1.15, -1.00, -1.15, 0.40, 0.4}, // Slow strafe: at 0.3 it never moved (5 Nov, 083552)
      {-1.22, 0.90, -1.22, 0.90, 0}, // Standing far
      {-1.00, 1.05, -1.22, -1.05, 1.0}, // Corner to corner, past the Cell
      {-1.22, -1.05, -1.00, 1.05, 1.0}, // And back
  };
  private static final double GOTO_STICK = 0.6;
  private static final double GOTO_GAIN = 2.0; // Stick per metre off
  private static final double GOTO_TOLERANCE_M = 0.07;
  /** Least stick while still off: 0.18 shaped to 0.19 power and stalled 9 cm short. */
  private static final double GOTO_MIN_STICK = 0.35;
  // Added per second without progress: 0.35 sideways (0.26 power) sat 11 cm short until skipped (5 Nov, 082021)
  private static final double GOTO_STUCK_BOOST_PER_S = 0.3;
  private static final double GOTO_TIMEOUT_S = 10;
  // Odometry alone took the robot off the mat: driving to the blue side the camera saw no tags for 20 s while the
  // pose barely moved (5 Nov, 080742)
  private static final double GOTO_BLIND_S = 3.0;
  private static final double CAMERA_LOST_S = 6.0; // Long enough for a restart to bring it back
  private int lastFixCount = -1;
  private double lastFixS = Double.NaN;
  private boolean needFreshFix; // Searching after a blind drive: goes on only once the tags fix the position again
  private int fixCountAtSearch;
  private static final double DONE_STOP_S = 1.0; // Logs the finish, then stops the OpMode
  private static final double POSE_LOST_WAIT_S = 1;
  private static final int OUT_OF_FIELD_STOP_FRAMES = 5;
  private static final int HEADING_OFF_STOP_FRAMES = 3;
  private double poseLostSinceS = Double.NaN;
  private double doneS;
  private static final double CROSS_GAIN = 2.0;
  private static final double END_SLOW_MIN = 0.35;
  // Stick per metre left. At 1.2 the robot still did 0.72 m/s 4 cm from the end and its prediction left the fence;
  // at 0.8, uncapped, 0.99 m/s 0.25 m from it, and it coasted 0.35 m past (5 Nov, 083228)
  private static final double END_SLOW_GAIN = 0.5;
  private static final double SETTLE_S = 0.5;
  private static final double PRE_AIM_S = 2.0;
  private static final double RELEASE_S = 0.4;
  private static final double STILL_AIM_S = 2.5;
  private static final double AFTER_S = 1.0;
  private static final double BALL_AFTER_S = 3.0; // A single ball trial holds RT longer, so every ball loaded is fired
  private static final double POSE_WAIT_S = 4;
  // 0.35 for 0.35 s never broke the mat's static friction: 0.6 degrees in 30 s of searching (5 Nov, 081514)
  private static final double SEARCH_TURN = 0.6;
  private static final double SEARCH_TIMEOUT_S = 30;
  private static final double SEARCH_PERIOD_S = 1.0; // Turns for SEARCH_TURN_S of each, then holds still to look
  private static final double SEARCH_TURN_S = 0.3;

  // 0.3 sideways gave the wheels 0.12-0.31 and the robot didn't move (5 Nov, 084013)
  private static final double BACK_OUT_STICK = 0.45;
  private static final double BACK_OUT_TIMEOUT_S = 5;
  // Starting past FIELD_LIMIT_M but this far in from the field's middle at most, it drives straight back in
  // first: a fence stop coasts the robot out of the area, and the next run then had to be moved by hand
  private static final double BACK_IN_FROM_M = 1.70;
  private static final double BACK_IN_MARGIN_M = 0.05; // Backing in ends this far inside the field limit
  private double backOutX; // Stick direction while backing out, field frame
  private double backOutY;
  private double backOutLimitX; // Fence while backing out: no further out than where it started
  private double backOutLimitY;

  private enum Phase { WAIT_POSE, BACK_OUT, GOTO, SETTLE, PRE_AIM, RELEASE, MOVE, AFTER, DONE }

  private final Gamepad script = new Gamepad();
  private final ElapsedTime clock = new ElapsedTime();
  private Robot robot;
  private Phase phase = Phase.WAIT_POSE;
  private double phaseStartS;
  private int trial;
  private boolean finished;
  private String finishReason = "";
  private double cmdX; // Field direction asked for this loop, unit or 0
  private double cmdY;
  private double wrongWaySinceS = Double.NaN;
  // Stall: driven but not getting on. Skips the trial rather than pushing on whatever is in the way
  private static final double STALL_S = 1.5;
  private static final double STALL_PROGRESS_M = 0.03;
  private double stallRefS;
  private double stallRefX;
  private double stallRefY;
  private int skipped;
  // Skips in a row that stop the test. A frozen pose looks like a stall on every drive, and the robot, pushed a
  // new way for STALL_S each time, wandered off the field (5 Nov, 085633: 25 skips)
  private static final int MAX_SKIPS_IN_A_ROW = 3;
  private int skipsInARow;
  private int lap;
  protected Alliance testAlliance = Alliance.RED;
  /** Set by {@link #configureTest}: runs only this trial, once, then stops. -1 runs every trial, lap after lap. */
  protected int onlyTrial = -1;
  private double mirrorX = 1; // This lap's Cell side: -1 flips the trials to +x
  private double mirrorY = 1;
  private final java.util.List<double[]> route = new java.util.ArrayList<>(); // GOTO's waypoints, the start last
  private int routeIndex;
  private double routeTimeoutS;
  private double legStartX; // Where the leg to route.get(routeIndex) began, so GOTO holds the straight line
  private double legStartY;

  @Override protected int getRlogPort() { return Main.LIVE_LOGGING ? Main.RLOG_PORT : 0; }
  @Override protected void onPsiKitConfigureLogging() {
    Main.configureLogging();
    Logger.recordMetadata("OpMode", "SOTM Field Test");
  }
  /** Before init: a subclass may pick one trial ({@link #onlyTrial}) and the alliance. */
  protected void configureTest() {}

  @Override protected void onPsiKitInit() {
    configureTest();
    if (onlyTrial >= TRIALS.length) {
      onlyTrial = TRIALS.length - 1;
    }
    trial = Math.max(onlyTrial, 0);
    robot = new Robot(hardwareMap, telemetry, script);
    robot.init();
  }
  @Override protected void onPsiKitInitLoop() {
    robot.initLoop();
  }
  @Override protected void onPsiKitStart() {
    robot.start();
    robot.setAlliance(testAlliance);
    clock.reset();
    phaseStartS = 0;
  }

  @Override protected void onPsiKitLoop() {
    double nowS = clock.seconds();
    neutral();
    Pose pose = robot.getPose();
    double[] v = robot.getVelocity();
    boolean placed = robot.isPoseValid() && robot.isHeadingValid();
    if (!finished && driverTouched()) {
      finish("driver took over");
    }
    // Last loop's command against this loop's motion
    double along = v[0] * cmdX + v[1] * cmdY;
    if (along < -0.1 && Math.hypot(v[0], v[1]) > 0.15) {
      if (Double.isNaN(wrongWaySinceS)) { wrongWaySinceS = nowS; }
    } else {
      wrongWaySinceS = Double.NaN;
    }
    if (!finished && !Double.isNaN(wrongWaySinceS) && nowS - wrongWaySinceS >= 0.3) {
      finish(String.format("moving against the command: %.2f, %.2f for %.2f, %.2f", v[0], v[1], cmdX, cmdY));
    }
    cmdX = 0;
    cmdY = 0;
    boolean regained = false;
    if (robot.getPositionFixes() != lastFixCount) {
      lastFixCount = robot.getPositionFixes();
      lastFixS = nowS;
    }
    if (!finished && robot.vision.newestFrameAgeS() > CAMERA_LOST_S) {
      // The camera dropped off USB and the run waited on tags that could never come (5 Nov, 080143)
      finish(String.format("camera gave no frames for %.0f s: check its USB cable", robot.vision.newestFrameAgeS()));
    } else if (!finished && phase != Phase.WAIT_POSE) {
      if (!placed) {
        // Holds still while the odometry recovers, then looks for the tags again as at the start, and goes on
        // with this trial. Ending the run lost it to a Pinpoint dropout that was back in 0.9 s (5 Nov, 083136);
        // standing still, the tags in view gave no heading for 5 s (083552)
        if (Double.isNaN(poseLostSinceS)) { poseLostSinceS = nowS; }
        if (nowS - poseLostSinceS >= POSE_LOST_WAIT_S) {
          poseLostSinceS = Double.NaN;
          next(Phase.WAIT_POSE, nowS);
        }
      } else if (robot.getOutOfFieldFrames() >= OUT_OF_FIELD_STOP_FRAMES) {
        // The tags agree the robot is off the field while odometry has it inside: it drove off the mat for 18 s,
        // the fence trusting a pose 0.9 m out (5 Nov, 080057)
        finish("tags put the robot off the field: odometry is wrong");
      } else if (robot.getHeadingOffFrames() >= HEADING_OFF_STOP_FRAMES) {
        // Single tags move the pose using the heading, so a heading 35 degrees out put it 0.9 m out; tag pairs
        // were that far off for 4 s before it left the mat (5 Nov, 080057)
        finish(String.format("tag pairs put the heading %.0f degrees off", Math.toDegrees(robot.getLastHeadingOffRad())));
      } else if (!Double.isNaN(poseLostSinceS)) {
        poseLostSinceS = Double.NaN;
        regained = true;
      } else if (phase == Phase.GOTO && !(nowS - lastFixS <= GOTO_BLIND_S)) {
        needFreshFix = true;
        fixCountAtSearch = robot.getPositionFixes();
        next(Phase.WAIT_POSE, nowS); // Stops, looks for the tags, then redoes this trial from wherever it is
      } else if (phase == Phase.BACK_OUT) {
        // Starts inside the Hive's margin or past the edge, so only the field edge, or where it started past it,
        // applies; moving the wrong way still stops it
        if (Math.abs(pose.xM) > backOutLimitX || Math.abs(pose.yM) > backOutLimitY) {
          finish(String.format("fence while backing out at %.2f, %.2f", pose.xM, pose.yM));
        }
      } else if (!inBox(pose.xM, pose.yM, FENCE_SLACK_M + EDGE_FENCE_EXTRA_M, FENCE_SLACK_M)
          || !inBox(pose.xM + stopDistance(v[0]), pose.yM + stopDistance(v[1]),
              2 * FENCE_SLACK_M + EDGE_FENCE_EXTRA_M, 2 * FENCE_SLACK_M)) {
        finish(String.format("fence at %.2f, %.2f moving %.2f, %.2f", pose.xM, pose.yM, v[0], v[1]));
      }
    }
    if (!finished && regained) {
      startGoto(nowS, pose); // This trial again, from its start
    } else if (!finished && Double.isNaN(poseLostSinceS)) {
      step(nowS, pose, placed);
    }
    if (finished) {
      neutral();
      if (phase != Phase.DONE) {
        doneS = nowS;
      }
      phase = Phase.DONE;
      if (nowS - doneS >= DONE_STOP_S) {
        requestOpModeStop(); // Left running, the flywheel kept spinning with nothing to do
      }
    }
    // A moving ball trial lines up standing with RT held but keeps its balls for the move
    robot.setHoldFire(phase == Phase.GOTO || phase == Phase.SETTLE
        || phase == Phase.PRE_AIM && onlyTrial >= 0 && trialPath()[4] != 0);
    robot.periodic(); // Acts on this loop's script; its pose is read at the top of the next
    Logger.recordOutput("FieldTest/Phase", phase.name());
    Logger.recordOutput("FieldTest/Trial", trial);
    Logger.recordOutput("FieldTest/Lap", lap);
    Logger.recordOutput("FieldTest/OnlyTrial", onlyTrial);
    Logger.recordOutput("FieldTest/Alliance", testAlliance.name());
    Logger.recordOutput("FieldTest/Mirror", new double[] {mirrorX, mirrorY});
    Logger.recordOutput("FieldTest/Finished", finished);
    Logger.recordOutput("FieldTest/FinishReason", finishReason);
    Logger.recordOutput("FieldTest/Stick", new double[] {script.left_stick_x, script.left_stick_y,
        script.right_stick_x, script.right_trigger});
  }

  private void step(double nowS, Pose pose, boolean placed) {
    double inPhase = nowS - phaseStartS;
    double[] t = trialPath();
    switch (phase) {
      case WAIT_POSE: {
        // Placed, and the tags have shown which of this alliance's Cells is up
        boolean ready = placed && robot.isAimingAtFieldCell() && robot.getAlliance() == testAlliance
            && (!needFreshFix || robot.getPositionFixes() > fixCountAtSearch);
        if (ready && inPhase >= 1.0) {
          needFreshFix = false;
          boolean nearEdge = !inBox(pose.xM, pose.yM, 0)
              && Math.abs(pose.xM) <= BACK_IN_FROM_M && Math.abs(pose.yM) <= BACK_IN_FROM_M;
          if (Math.abs(pose.xM) < HIVE_HALF_X_M && Math.abs(pose.yM) < HIVE_HALF_Y_M) {
            // Stopped close to the Hive last time: straight back out the nearer way
            boolean outInX = Math.abs(pose.xM) / HIVE_HALF_X_M > Math.abs(pose.yM) / HIVE_HALF_Y_M;
            backOut(nowS, pose, outInX ? Math.signum(pose.xM) : 0, outInX ? 0 : Math.signum(pose.yM));
          } else if (nearEdge) {
            // Inward on every axis short of BACK_IN_MARGIN_M, the margin it must reach: at x 1.32 it backed in
            // along y only and drove 1.2 m down the edge until the timeout (5 Nov, 091240)
            double in = FIELD_LIMIT_M - BACK_IN_MARGIN_M;
            backOut(nowS, pose, Math.abs(pose.xM) > in ? -Math.signum(pose.xM) : 0,
                Math.abs(pose.yM) > in ? -Math.signum(pose.yM) : 0);
          } else if (!inBox(pose.xM, pose.yM, 0)) {
            finish(String.format("starts outside the test area, at %.2f, %.2f", pose.xM, pose.yM));
          } else {
            mirrorX = robot.getAimCellX() < 0 ? 1 : -1;
            mirrorY = robot.getAimCellY() < 0 ? 1 : -1;
            startGoto(nowS, pose);
          }
        } else if (inPhase >= POSE_WAIT_S + SEARCH_TIMEOUT_S) {
          if (needFreshFix) {
            finish("no tag fix since driving blind: is the camera giving frames?");
          } else if (lap > 0 && placed && onlyTrial < 0) {
            nextLap(nowS); // This alliance's up Cell never showed: on with the other one
          } else {
            finish(placed ? "tags never showed which Cell is up" : "tags never placed the robot");
          }
        } else if (inPhase >= POSE_WAIT_S && !ready
            && (inPhase - POSE_WAIT_S) % SEARCH_PERIOD_S < SEARCH_TURN_S) {
          // On the spot: look around for the Hive, a step at a time. Turning all the time, every frame was too
          // fast for vision and it never placed the robot (5 Nov, 090321)
          script.right_stick_x = (float) SEARCH_TURN;
        }
        break;
      }
      case BACK_OUT:
        if (inBox(pose.xM, pose.yM, -BACK_IN_MARGIN_M)) {
          next(Phase.WAIT_POSE, nowS); // Then picks the Cell side and the route from where it is now
        } else if (inPhase >= BACK_OUT_TIMEOUT_S) {
          finish(String.format("could not back into the test area, at %.2f, %.2f", pose.xM, pose.yM));
        } else {
          setStick(backOutX * BACK_OUT_STICK, backOutY * BACK_OUT_STICK, BACK_OUT_STICK);
        }
        break;
      case GOTO: {
        double[] w = route.get(routeIndex);
        boolean last = routeIndex == route.size() - 1;
        if (!last && Math.hypot(w[0] - pose.xM, w[1] - pose.yM) <= ROUTE_WAYPOINT_TOLERANCE_M) {
          routeIndex++;
          legStartX = pose.xM;
          legStartY = pose.yM;
          stallRefS = nowS;
        } else if (aimOnTheWay()
            && goTo(pose, w[0], w[1], GOTO_STICK, GOTO_STUCK_BOOST_PER_S * (nowS - stallRefS)) && last) {
          skipsInARow = 0;
          next(Phase.SETTLE, nowS);
        } else if (inPhase >= routeTimeoutS || stalled(nowS, pose)) {
          skipTrial(nowS, pose);
        }
        break;
      }
      case SETTLE:
        script.right_trigger = 1;
        if (inPhase >= SETTLE_S) { next(Phase.PRE_AIM, nowS); }
        break;
      case PRE_AIM: // Lined up standing still, so the moving shot starts from the right heading
        script.right_trigger = 1;
        if (inPhase >= (t[4] == 0 ? STILL_AIM_S : PRE_AIM_S)) {
          next(t[4] == 0 ? Phase.AFTER : Phase.RELEASE, nowS);
        }
        break;
      case RELEASE:
        if (inPhase >= RELEASE_S) { next(Phase.MOVE, nowS); }
        break;
      case MOVE: {
        script.right_trigger = 1;
        double dx = t[2] - t[0];
        double dy = t[3] - t[1];
        double length = Math.hypot(dx, dy);
        double ux = dx / length;
        double uy = dy / length;
        double along = (pose.xM - t[0]) * ux + (pose.yM - t[1]) * uy;
        double cross = -(pose.xM - t[0]) * uy + (pose.yM - t[1]) * ux; // Left of the path positive
        if (along >= length - 0.03 || inPhase >= length / 0.2 + 2 || stalled(nowS, pose)) {
          next(Phase.AFTER, nowS);
          script.right_trigger = 1;
          break;
        }
        // Slows for the end, as a driver would, rather than running past it
        double speed = Math.min(t[4], END_SLOW_MIN + END_SLOW_GAIN * (length - along));
        double vx = speed * ux + CROSS_GAIN * cross * uy; // Back toward the line: right of travel
        double vy = speed * uy - CROSS_GAIN * cross * ux;
        setStick(vx, vy, t[4]);
        break;
      }
      case AFTER:
        script.right_trigger = 1;
        if (onlyTrial >= 0 && inPhase >= BALL_AFTER_S) {
          finish("ball trial done");
        } else if (onlyTrial < 0 && inPhase >= AFTER_S) {
          trial++;
          if (trial >= TRIALS.length) {
            nextLap(nowS);
          } else {
            startGoto(nowS, pose);
          }
        }
        break;
      default:
        break;
    }
  }

  /**
   * Aims (fire held) all the way to the start. PRE_AIM then starts facing the Cell: turning there from the way
   * GOTO left it took up to 1.4 s of the 2 s, 127 degrees off (5 Nov, 082500). Facing the Cell also keeps the
   * camera on the Hive: holding the red side's heading on the blue side, it saw no tags (5 Nov, 080742).
   * Always true.
   */
  private boolean aimOnTheWay() {
    script.right_trigger = 1;
    return true;
  }

  /** Drives toward (x, y); true once within GOTO_TOLERANCE_M, sticks left centred. */
  private boolean goTo(Pose pose, double x, double y, double maxStick, double stuckBoost) {
    double ex = x - pose.xM;
    double ey = y - pose.yM;
    if (Math.hypot(ex, ey) <= GOTO_TOLERANCE_M) {
      return true;
    }
    double r = Math.hypot(ex, ey);
    double stick = Math.min(maxStick, Math.max(GOTO_MIN_STICK + stuckBoost, GOTO_GAIN * r));
    double vx = ex / r * stick;
    double vy = ey / r * stick;
    // Back onto the straight leg: aimed only at the end, 2.5 m down the edge drifted 0.21 m out into the
    // fence (5 Nov, 082500)
    double lx = x - legStartX;
    double ly = y - legStartY;
    double legLength = Math.hypot(lx, ly);
    if (legLength > GOTO_TOLERANCE_M) {
      double ux = lx / legLength;
      double uy = ly / legLength;
      double cross = -(pose.xM - legStartX) * uy + (pose.yM - legStartY) * ux; // Left of the leg positive
      vx += CROSS_GAIN * cross * uy;
      vy -= CROSS_GAIN * cross * ux;
    }
    setStick(vx, vy, maxStick);
    return false;
  }

  /**
   * Field-frame stick: the drive is field-centric with forward along +x (both alliances' forward is 0), so
   * forward is +x and right is -y.
   */
  private void setStick(double vx, double vy, double maxStick) {
    double r = Math.hypot(vx, vy);
    if (r > maxStick) {
      vx *= maxStick / r;
      vy *= maxStick / r;
    }
    if (r > 0) {
      cmdX = vx / Math.hypot(vx, vy);
      cmdY = vy / Math.hypot(vx, vy);
    }
    script.left_stick_y = (float) -vx;
    script.left_stick_x = (float) -vy;
  }

  private boolean driverTouched() {
    Gamepad g = gamepad1;
    return Math.abs(g.left_stick_x) > 0.3 || Math.abs(g.left_stick_y) > 0.3 || Math.abs(g.right_stick_x) > 0.3
        || Math.abs(g.right_stick_y) > 0.3 || g.left_trigger > 0.3 || g.right_trigger > 0.3
        || g.a || g.b || g.x || g.y || g.left_bumper || g.right_bumper;
  }

  /** Where it would stop along one axis, from a speed along it: reaction then steady braking. */
  static double stopDistance(double v) {
    return v * PREDICT_S + v * Math.abs(v) / (2 * BRAKE_MPS2);
  }

  private static boolean inBox(double x, double y, double slack) {
    return inBox(x, y, slack, slack);
  }

  private static boolean inBox(double x, double y, double edgeSlack, double hiveSlack) {
    boolean onField = Math.abs(x) <= FIELD_LIMIT_M + edgeSlack && Math.abs(y) <= FIELD_LIMIT_M + edgeSlack;
    boolean inHive = Math.abs(x) < HIVE_HALF_X_M - hiveSlack && Math.abs(y) < HIVE_HALF_Y_M - hiveSlack;
    return onField && !inHive;
  }

  private void backOut(double nowS, Pose pose, double dirX, double dirY) {
    double r = Math.hypot(dirX, dirY);
    backOutX = dirX / r;
    backOutY = dirY / r;
    backOutLimitX = Math.max(FIELD_LIMIT_M, Math.abs(pose.xM) + FENCE_SLACK_M);
    backOutLimitY = Math.max(FIELD_LIMIT_M, Math.abs(pose.yM) + FENCE_SLACK_M);
    next(Phase.BACK_OUT, nowS);
  }

  private void neutral() {
    script.left_stick_x = 0;
    script.left_stick_y = 0;
    script.right_stick_x = 0;
    script.right_trigger = 0;
  }

  private void next(Phase p, double nowS) {
    phase = p;
    phaseStartS = nowS;
    stallRefS = nowS;
    stallRefX = robot.getPose().xM;
    stallRefY = robot.getPose().yM;
  }

  /** True once the robot has moved less than STALL_PROGRESS_M in STALL_S of being driven. */
  private boolean stalled(double nowS, Pose pose) {
    if (Math.hypot(pose.xM - stallRefX, pose.yM - stallRefY) >= STALL_PROGRESS_M) {
      stallRefS = nowS;
      stallRefX = pose.xM;
      stallRefY = pose.yM;
      return false;
    }
    return nowS - stallRefS >= STALL_S;
  }

  private void skipTrial(double nowS, Pose pose) {
    skipped++;
    skipsInARow++;
    Logger.recordOutput("FieldTest/Skipped", skipped);
    if (skipsInARow >= MAX_SKIPS_IN_A_ROW) {
      finish(skipsInARow + " trials in a row could not reach their start: pose stuck or the way blocked");
      return;
    }
    if (onlyTrial >= 0) {
      finish("ball trial skipped: could not reach its start");
      return;
    }
    trial++;
    if (trial >= TRIALS.length) {
      nextLap(nowS);
    } else {
      startGoto(nowS, pose);
    }
  }

  /** The other alliance next, from its first trial, once the tags show its up Cell. */
  private void nextLap(double nowS) {
    lap++;
    trial = 0;
    if (lap >= MAX_LAPS) {
      finish("complete, " + skipped + " skipped");
      return;
    }
    testAlliance = testAlliance == Alliance.RED ? Alliance.BLUE : Alliance.RED;
    robot.setAlliance(testAlliance);
    next(Phase.WAIT_POSE, nowS);
  }

  /** This trial's path, mirrored to this lap's Cell side. */
  private double[] trialPath() {
    double[] t = TRIALS[Math.min(trial, TRIALS.length - 1)];
    return new double[] {mirrorX * t[0], mirrorY * t[1], mirrorX * t[2], mirrorY * t[3], t[4]};
  }

  private void startGoto(double nowS, Pose pose) {
    double[] t = trialPath();
    route.clear();
    route.addAll(route(pose.xM, pose.yM, t[0], t[1]));
    routeIndex = 0;
    legStartX = pose.xM;
    legStartY = pose.yM;
    double length = 0;
    double x = pose.xM;
    double y = pose.yM;
    for (double[] w : route) {
      length += Math.hypot(w[0] - x, w[1] - y);
      x = w[0];
      y = w[1];
    }
    routeTimeoutS = GOTO_TIMEOUT_S + length / 0.3;
    next(Phase.GOTO, nowS);
  }

  /**
   * The shortest way from one point to another that keeps clear of the Hive: straight, or round one or two of its
   * corners ({@link #ROUTE_X_M}, {@link #ROUTE_Y_M}). The waypoints, the end last.
   */
  static java.util.List<double[]> route(double fromX, double fromY, double toX, double toY) {
    double[][] corners = {{-ROUTE_X_M, -ROUTE_Y_M}, {-ROUTE_X_M, ROUTE_Y_M}, {ROUTE_X_M, -ROUTE_Y_M},
        {ROUTE_X_M, ROUTE_Y_M}};
    double[] end = {toX, toY};
    java.util.List<double[]> best = null;
    double bestLength = Double.POSITIVE_INFINITY;
    java.util.List<java.util.List<double[]>> candidates = new java.util.ArrayList<>();
    candidates.add(java.util.Arrays.asList(end));
    for (double[] a : corners) {
      candidates.add(java.util.Arrays.asList(a, end));
      for (double[] b : corners) {
        if (a != b && (a[0] == b[0] || a[1] == b[1])) { // Along one side of the Hive
          candidates.add(java.util.Arrays.asList(a, b, end));
        }
      }
    }
    for (java.util.List<double[]> candidate : candidates) {
      double length = 0;
      boolean clear = true;
      double x = fromX;
      double y = fromY;
      for (double[] w : candidate) {
        clear &= segmentClear(x, y, w[0], w[1]);
        length += Math.hypot(w[0] - x, w[1] - y);
        x = w[0];
        y = w[1];
      }
      if (clear && length < bestLength) {
        best = candidate;
        bestLength = length;
      }
    }
    return best != null ? best : java.util.Arrays.asList(end);
  }

  private static boolean segmentClear(double x0, double y0, double x1, double y1) {
    int steps = (int) Math.ceil(Math.hypot(x1 - x0, y1 - y0) / 0.02);
    for (int i = 0; i <= steps; i++) {
      double f = steps == 0 ? 0 : (double) i / steps;
      double x = x0 + f * (x1 - x0);
      double y = y0 + f * (y1 - y0);
      if (Math.abs(x) < HIVE_HALF_X_M + ROUTE_CLEARANCE_M && Math.abs(y) < HIVE_HALF_Y_M + ROUTE_CLEARANCE_M) {
        return false;
      }
    }
    return true;
  }

  private void finish(String reason) {
    if (!finished) {
      finished = true;
      finishReason = reason;
    }
  }

  @Override protected void onPsiKitStop() {
    if (robot != null) {
      robot.stop();
    }
  }
}
