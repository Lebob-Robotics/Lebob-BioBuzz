package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.utils.Aim;
import org.firstinspires.ftc.teamcode.utils.Pose;
import org.firstinspires.ftc.teamcode.utils.ShootOnTheMoveConstants;
import org.firstinspires.ftc.teamcode.utils.VisionCorrector;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.PsiKitIterativeOpMode;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;
import java.util.Locale;

/**
 * Lob over our Hive into its far Cell: the robot stands {@link #LIP_GAP_M} out from the near Cell's lip, on the
 * Hive's line, and fires the hooded shot straight along it, over the near end, to drop through the far Cell's
 * opening. That opening faces away and rises toward the middle at about 60 degrees, so the ball only goes in falling
 * steeper than that; seen along its path the gap is a few centimetres. Every ball that leaves is written to
 * {@link #CSV_PATH}; X marks the last one as in, A as rattled out, unmarked is a miss.
 *
 * <p>Controls (gamepad 1):
 * <ul>
 *   <li>Sticks: drive and turn.</li>
 *   <li>Left trigger held: intake and indexer.</li>
 *   <li>B: drive to the spot in front of the nearer end of our Hive and turn to the far Cell. Any stick, trigger or B
 *       again stops it.</li>
 *   <li>Y: turn to the far Cell.</li>
 *   <li>Right bumper: turn to the far Cell and fire one ball once ready. Press again for the next.</li>
 *   <li>D-pad up/down: RPM by {@link #RPM_STEP}. Right/left: raise/lower the hood by {@link #HOOD_STEP_DEG} (launch
 *       angle falls/rises).</li>
 *   <li>Left bumper: back to the starting estimate for the far Cell as the tags last showed it (up or down).</li>
 *   <li>X: the last ball went in. A: it went in and rattled out. Unmarked: a miss.</li>
 *   <li>Left/right stick press: aim 1 degree further left/right.</li>
 *   <li>Share (back): driving from the far side; turns the sticks round. On by default.</li>
 * </ul>
 */
@TeleOp(name = "Special Shot Calibration", group = "Tuning")
public class SpecialShotCalibration extends PsiKitIterativeOpMode {
  private static final String CSV_PATH = "/sdcard/FIRST/SpecialShot.csv";
  private static final String CSV_HEADER = "session,shot,kind,x_m,y_m,heading_deg,target_x_m,target_y_m,far_cell,"
      + "far_cell_up,up_confirmed,head_err_deg,target_rpm,rpm_at_feed,launch_deg,hood_at_feed_deg,battery_v,"
      + "head_trim_deg,rpm_dip";

  /** From the near Cell's lip to the robot centre, along the Hive's line. */
  static final double LIP_GAP_M = 0.5;

  /*
   * Starting shots, from tools/shots' ball model with inputs.json (drag 0.1, 0.00156 m/s per rpm, the ball 8.5 deg
   * steeper than the hood) flown from 1.10 m out: clear of the near Cell's top edge (1.67 m) by a ball and 8 cm,
   * crossing the far opening inward away from its edges. Launch angle as Shot Tuning shows it (hood angle).
   *
   * Far Cell up (mirror of the near one, lip 1.36 m, face at 60 deg): 72 deg scores 4910-5010 rpm, the middle
   * falling at 78 deg after a 3.1 m apex, 1.3 s up. 70 deg gives 4550-4620; 74 deg 5430-5570. Steeper is wider but
   * needs more rpm. The model was fitted to shots under 4200 rpm and leaves out backspin lift, which carries the ball
   * further, so it should need less than this: walk the rpm down first.
   *
   * Far Cell down: guessed lip 1.00 m and face at 70 deg. Only 74-75 deg at 5390-5800 rpm got in, in a 50-70 rpm
   * band; with the face at 80 deg nothing does. Try it last.
   */
  static final double START_RPM_FAR_UP = 4960;
  static final double START_LAUNCH_DEG_FAR_UP = 72;
  static final double START_RPM_FAR_DOWN = 5580;
  static final double START_LAUNCH_DEG_FAR_DOWN = 74.5;

  private static final double RPM_STEP = 25; // The modelled band is 100 rpm wide
  private static final double HOOD_STEP_DEG = 0.5;
  private static final double MIN_RPM = 1000;
  private static final double HEADING_TRIM_STEP_DEG = 1;
  private static final double BALL_DIP_RPM = 200; // As Shot Tuning: a feed with no ball doesn't dip this far
  private static final double BALL_DIP_WINDOW_S = 0.6;
  private static final double ORIENT_TIMEOUT_S = 3;
  private static final double ORIENT_DONE_DEG = 1.0;
  private static final double SPOT_SETTLE_S = 1.5;

  private Robot robot;
  private final ElapsedTime clock = new ElapsedTime();
  private final String session = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
  private double rpm = START_RPM_FAR_UP;
  private double launchDeg = START_LAUNCH_DEG_FAR_UP;
  private double trimDeg = 0;
  private VisionCorrector.Cell farCell = null; // Fixed by B; until then the end away from the robot
  private int shots = 0;
  private int scored = 0;
  private int rattled = 0;
  private int emptyFeeds = 0;
  private String lastShotRow = null;
  private String lastMark = null;
  private String fileError = null;

  private String pendingRow = null;
  private double pendingStartS;
  private double pendingStartRpm;
  private double pendingMinRpm;

  private boolean driving = false;
  private List<double[]> route;
  private int routeIndex;
  private double legStartX;
  private double legStartY;
  private double driveStartS;
  private double driveTimeoutS;
  private int lastFixCount;
  private double lastFixS;
  private double progressX;
  private double progressY;
  private double progressS;
  private double settleUntilS = Double.NaN;
  private double orientUntilS = Double.NaN;
  private String driveStatus = "";

  private boolean wasLeftBumper;
  private boolean wasLeftStick;
  private boolean wasRightStick;
  private boolean wasBack;
  private boolean wasB;
  private boolean wasX;
  private boolean wasA;
  private boolean wasY;
  private boolean wasDpadUp;
  private boolean wasDpadDown;
  private boolean wasDpadLeft;
  private boolean wasDpadRight;

  @Override
  protected int getRlogPort() {
    return Main.LIVE_LOGGING ? Main.RLOG_PORT : 0;
  }

  @Override
  protected void onPsiKitConfigureLogging() {
    Main.configureLogging();
    Logger.recordMetadata("OpMode", "Special Shot Calibration");
    Logger.recordMetadata("TuningSession", session);
  }

  @Override
  protected void onPsiKitInit() {
    robot = new Robot(hardwareMap, telemetry, gamepad1);
    robot.init();
    robot.setTuningMode(true);
    robot.setAlliance(Alliance.BLUE);
    robot.setDriverOnFarSide(true);
    pushShot(null);
  }

  @Override
  protected void onPsiKitInitLoop() {
    if (robot != null) {
      telemetry.addData("Driving from", robot.isDriverOnFarSide() ? "FAR side of the field (Share to change)"
          : "alliance wall (Share to change)");
      Gamepad pad = robot.getDriver();
      if (justPressed(pad.back, wasBack)) {
        robot.setDriverOnFarSide(!robot.isDriverOnFarSide());
      }
      wasBack = pad.back;
      robot.initLoop();
    }
  }

  @Override
  protected void onPsiKitStart() {
    if (robot != null) {
      robot.start();
    }
    clock.reset();
  }

  @Override
  protected void onPsiKitLoop() {
    double nowS = clock.seconds();
    Gamepad pad = robot.getDriver();
    Pose pose = robot.getPose();
    VisionCorrector.Cell far = farCell != null ? farCell : farEndFrom(pose);
    if (justPressed(pad.back, wasBack)) {
      robot.setDriverOnFarSide(!robot.isDriverOnFarSide());
    }
    if (justPressed(pad.left_bumper, wasLeftBumper)) {
      boolean down = robot.getUpCell() != far;
      rpm = down ? START_RPM_FAR_DOWN : START_RPM_FAR_UP;
      launchDeg = down ? START_LAUNCH_DEG_FAR_DOWN : START_LAUNCH_DEG_FAR_UP;
    }
    if (justPressed(pad.y, wasY)) {
      orientUntilS = nowS + ORIENT_TIMEOUT_S;
    }
    if (justPressed(pad.b, wasB)) {
      if (driving) {
        stopDriving("stopped (B)");
      } else {
        startDriving(nowS, pose);
        far = farCell != null ? farCell : far;
      }
    }
    if (justPressed(pad.dpad_up, wasDpadUp)) {
      rpm = Math.min(robot.shooter.getMaxRpm(), rpm + RPM_STEP);
    }
    if (justPressed(pad.dpad_down, wasDpadDown)) {
      rpm = Math.max(MIN_RPM, rpm - RPM_STEP);
    }
    if (justPressed(pad.dpad_right, wasDpadRight)) {
      launchDeg = Math.max(HoodSubsystem.MIN_ANGLE_DEG, launchDeg - HOOD_STEP_DEG);
    }
    if (justPressed(pad.dpad_left, wasDpadLeft)) {
      launchDeg = Math.min(HoodSubsystem.ZERO_ANGLE_DEG, launchDeg + HOOD_STEP_DEG);
    }
    trimDeg += (justPressed(pad.left_stick_button, wasLeftStick) ? HEADING_TRIM_STEP_DEG : 0)
        - (justPressed(pad.right_stick_button, wasRightStick) ? HEADING_TRIM_STEP_DEG : 0);
    String mark = justPressed(pad.x, wasX) ? "IN" : justPressed(pad.a, wasA) ? "RATTLED" : null;
    wasBack = pad.back;
    wasLeftStick = pad.left_stick_button;
    wasRightStick = pad.right_stick_button;
    wasLeftBumper = pad.left_bumper;
    wasB = pad.b;
    wasX = pad.x;
    wasA = pad.a;
    wasY = pad.y;
    wasDpadUp = pad.dpad_up;
    wasDpadDown = pad.dpad_down;
    wasDpadLeft = pad.dpad_left;
    wasDpadRight = pad.dpad_right;
    autoDrive(nowS, pose, pad);
    pushShot(far);
    addTelemetry(far);
    robot.periodic();

    if (robot.feedStartedThisLoop()) {
      startPending(nowS, far);
    }
    checkPending(nowS);
    if (mark != null && lastShotRow != null && !mark.equals(lastMark)) {
      scored += ("IN".equals(mark) ? 1 : 0) - ("IN".equals(lastMark) ? 1 : 0);
      rattled += ("RATTLED".equals(mark) ? 1 : 0) - ("RATTLED".equals(lastMark) ? 1 : 0);
      lastMark = mark;
      append(String.format(Locale.US, "%s,%d,%s,%s", session, shots, mark, lastShotRow));
    }
    Logger.recordOutput("Special/Rpm", rpm);
    Logger.recordOutput("Special/LaunchDeg", launchDeg);
    Logger.recordOutput("Special/HeadingTrimDeg", trimDeg);
    Logger.recordOutput("Special/FarCell", far.name());
    Logger.recordOutput("Special/FarCellUp", robot.getUpCell() == far);
    Logger.recordOutput("Special/Shots", shots);
    Logger.recordOutput("Special/Scored", scored);
    Logger.recordOutput("Special/Rattled", rattled);
    Logger.recordOutput("Special/EmptyFeeds", emptyFeeds);
    Logger.recordOutput("Special/LastMark", lastMark == null ? "" : lastMark);
    Logger.recordOutput("Special/Driving", driving);
    Logger.recordOutput("Special/DriveStatus", driveStatus);
  }

  @Override
  protected void onPsiKitStop() {
    if (robot != null) {
      robot.setDriveOverride(null);
      robot.setForceAim(false);
      robot.setAimPointOverride(null);
      robot.stop();
    }
  }

  /** The end of our Hive away from the robot: SCORING is at -x, AUDIENCE at +x. */
  private static VisionCorrector.Cell farEndFrom(Pose pose) {
    return pose != null && pose.xM < 0 ? VisionCorrector.Cell.AUDIENCE : VisionCorrector.Cell.SCORING;
  }

  /** {x, y} where the robot centre stands for the far Cell: on the Hive's line, LIP_GAP_M out from the near lip. */
  static double[] spotFor(Alliance alliance, VisionCorrector.Cell far) {
    double[] farPoint = VisionCorrector.upCellAimPoint(alliance, far);
    double outM = Math.abs(farPoint[0]) + ShootOnTheMoveConstants.CLUSTER_TO_NEAR_LIP_M + LIP_GAP_M;
    return new double[] {farPoint[0] > 0 ? -outM : outM, farPoint[1]};
  }

  private void pushShot(VisionCorrector.Cell far) {
    robot.setTuningShot(false, rpm, launchDeg);
    robot.setHeadingTrimDeg(trimDeg);
    robot.setAimPointOverride(far == null ? null : VisionCorrector.upCellAimPoint(robot.getAlliance(), far));
  }

  private void addTelemetry(VisionCorrector.Cell far) {
    boolean up = robot.getUpCell() == far;
    line("SPECIAL", String.format("%.0f rpm  launch %.1f deg  aim %s", rpm, launchDeg,
        trimDeg == 0 ? "straight" : String.format("%.0f deg %s", Math.abs(trimDeg), trimDeg > 0 ? "left" : "right")));
    line("Far Cell", String.format("%s end, %s%s  (LB: start at %.0f rpm, %.1f deg)", far, up ? "UP" : "DOWN",
        robot.isUpCellConfirmed() ? "" : " (tags not sure yet)",
        up ? START_RPM_FAR_UP : START_RPM_FAR_DOWN, up ? START_LAUNCH_DEG_FAR_UP : START_LAUNCH_DEG_FAR_DOWN));
    line("Balls", String.format("%d fired, %d in, %d rattled out%s%s", shots, scored, rattled,
        lastShotRow == null ? "" : lastMark != null ? "  (last " + lastMark + ")" : "  (last a miss: X in, A rattled)",
        emptyFeeds == 0 ? "" : String.format("  [%d feeds without a ball]", emptyFeeds)));
    line("Spot (B)", (driving ? "driving " : "") + driveStatus);
    line("Keys", "B spot  Y aim  RB fire  Dpad rpm/hood  LB estimate  X in  A rattled  Share "
        + (robot.isDriverOnFarSide() ? "(far side)" : "(alliance wall)"));
    if (fileError != null) {
      line("CSV", fileError);
    }
  }

  private void line(String key, String value) {
    telemetry.addData(key, value);
    robot.addPanelsLine(key, value);
  }

  // ---- Driving to the spot, and turning to the shot

  private void startDriving(double nowS, Pose pose) {
    if (!robot.isPoseValid()) {
      driveStatus = "B needs the tags to have placed the robot: point the cameras at the Hive";
      return;
    }
    farCell = farEndFrom(pose);
    double[] spot = spotFor(robot.getAlliance(), farCell);
    route = FieldRoute.route(pose.xM, pose.yM, spot[0], spot[1]);
    routeIndex = 0;
    legStartX = pose.xM;
    legStartY = pose.yM;
    driveStartS = nowS;
    double pathM = 0;
    double fromX = pose.xM;
    double fromY = pose.yM;
    for (double[] w : route) {
      pathM += Math.hypot(w[0] - fromX, w[1] - fromY);
      fromX = w[0];
      fromY = w[1];
    }
    driveTimeoutS = FieldRoute.GOTO_TIMEOUT_S + pathM / 0.4;
    lastFixCount = robot.getPositionFixes();
    lastFixS = nowS;
    progressX = pose.xM;
    progressY = pose.yM;
    progressS = nowS;
    settleUntilS = Double.NaN;
    orientUntilS = Double.NaN;
    driving = true;
    driveStatus = String.format("to %.2f, %.2f for the %s Cell", spot[0], spot[1], farCell);
  }

  private void stopDriving(String why) {
    driving = false;
    driveStatus = why;
  }

  private void autoDrive(double nowS, Pose pose, Gamepad pad) {
    boolean took = FieldRoute.driverTookOver(pad);
    double[] stick = null;
    boolean aim = false;
    if (driving) {
      if (robot.getPositionFixes() != lastFixCount) {
        lastFixCount = robot.getPositionFixes();
        lastFixS = nowS;
      }
      if (Math.hypot(pose.xM - progressX, pose.yM - progressY) >= 0.05) {
        progressX = pose.xM;
        progressY = pose.yM;
        progressS = nowS;
      }
      if (took) {
        stopDriving("stopped: you took over");
      } else if (!robot.isPoseValid()) {
        stopDriving("stopped: pose not placed by the tags");
      } else if (nowS - lastFixS > FieldRoute.GOTO_BLIND_S) {
        stopDriving("stopped: no tags seen for a while");
      } else if (nowS - driveStartS > driveTimeoutS) {
        stopDriving("stopped: took too long");
      } else if (!FieldRoute.headedSafely(pose, robot.getVelocity())) {
        stopDriving("stopped: heading for the fence or the Hive");
      } else {
        double[] w = route.get(routeIndex);
        boolean last = routeIndex == route.size() - 1;
        double away = Math.hypot(w[0] - pose.xM, w[1] - pose.yM);
        if (last && away <= FieldRoute.GOTO_TOLERANCE_M) {
          driving = false;
          driveStatus = "at the spot";
          settleUntilS = nowS + SPOT_SETTLE_S;
        } else {
          if (!last && away <= FieldRoute.ROUTE_WAYPOINT_TOLERANCE_M) {
            routeIndex++;
            legStartX = pose.xM;
            legStartY = pose.yM;
            w = route.get(routeIndex);
          }
          stick = FieldRoute.goToStick(pose, w[0], w[1], legStartX, legStartY,
              FieldRoute.GOTO_STUCK_BOOST_PER_S * (nowS - progressS));
          aim = true; // Keeps the cameras on the Hive tags on the way
        }
      }
    }
    if (!Double.isNaN(settleUntilS)) {
      if (took) {
        settleUntilS = Double.NaN;
      } else if (nowS >= settleUntilS) {
        settleUntilS = Double.NaN;
        orientUntilS = nowS + ORIENT_TIMEOUT_S;
      } else {
        aim = true;
      }
    }
    if (!Double.isNaN(orientUntilS)) {
      Aim last = robot.getLastAim();
      boolean lined = last != null
          && Math.abs(Math.toDegrees(last.headingErrorRad(pose.headingRad))) <= ORIENT_DONE_DEG;
      if (took || lined || nowS >= orientUntilS) {
        orientUntilS = Double.NaN;
      } else {
        aim = true;
      }
    }
    robot.setDriveOverride(stick);
    robot.setForceAim(aim);
  }

  // ---- The CSV

  private void startPending(double nowS, VisionCorrector.Cell far) {
    Aim aim = robot.getLastAim();
    if (aim == null) {
      return;
    }
    Pose pose = robot.getPose();
    pendingRow = String.format(Locale.US, "%.3f,%.3f,%.2f,%.3f,%.3f,%s,%d,%d,%.2f,%.0f,%.0f,%.2f,%.2f,%.2f,%.1f",
        pose.xM, pose.yM, Math.toDegrees(pose.headingRad), robot.getAimCellX(), robot.getAimCellY(), far,
        robot.getUpCell() == far ? 1 : 0, robot.isUpCellConfirmed() ? 1 : 0,
        Math.toDegrees(aim.headingErrorRad(pose.headingRad)), rpm, robot.shooter.getRpm(), launchDeg,
        HoodSubsystem.ticksToAngle(robot.hood.getPosition()), robot.getBatteryVolts(), trimDeg);
    pendingStartS = nowS;
    pendingStartRpm = robot.shooter.getRpm();
    pendingMinRpm = pendingStartRpm;
  }

  private void checkPending(double nowS) {
    if (pendingRow == null) {
      return;
    }
    pendingMinRpm = Math.min(pendingMinRpm, robot.shooter.getRpm());
    double dip = pendingStartRpm - pendingMinRpm;
    if (dip >= BALL_DIP_RPM) {
      shots++;
      lastMark = null;
      lastShotRow = String.format(Locale.US, "%s,%.0f", pendingRow, dip);
      append(String.format(Locale.US, "%s,%d,SHOT,%s", session, shots, lastShotRow));
      Logger.recordOutput("Special/ShotRow", lastShotRow);
      pendingRow = null;
    } else if (nowS - pendingStartS > BALL_DIP_WINDOW_S) {
      emptyFeeds++;
      pendingRow = null;
    }
  }

  private void append(String row) {
    if (Logger.isReplay()) {
      return;
    }
    File file = new File(CSV_PATH);
    boolean isNew = !file.exists();
    try (FileWriter out = new FileWriter(file, true)) {
      if (isNew) {
        out.write(CSV_HEADER + "\n");
      }
      out.write(row + "\n");
      fileError = null;
    } catch (IOException e) {
      fileError = "not saved: " + e.getMessage();
      RobotLog.ee("SpecialShot", e, "Can't write %s", CSV_PATH);
    }
  }

  private static boolean justPressed(boolean pressedNow, boolean pressedLastLoop) {
    return pressedNow && !pressedLastLoop;
  }
}
