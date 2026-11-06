package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.utils.Aim;
import org.firstinspires.ftc.teamcode.utils.Pose;
import org.firstinspires.ftc.teamcode.utils.ShootOnTheMove;
import org.firstinspires.ftc.teamcode.utils.ShootOnTheMoveConstants;
import org.firstinspires.ftc.teamcode.utils.Shot;
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
 * Drive anywhere and try shots by hand, to find the RPM and hood angle that score from each spot. Every ball that
 * leaves is written to {@link #CSV_PATH} with where the robot was and the shot it took. X marks the last one as in
 * and A as bounced out (backspin); unmarked is a plain miss. The shot tables can be refitted from the file.
 *
 * <p>Controls (gamepad 1), otherwise as Lebot Teleop:
 * <ul>
 *   <li>Sticks: drive and turn. The tags keep the heading right; A doesn't reset it here.</li>
 *   <li>Left trigger held: intake and indexer.</li>
 *   <li>Right bumper: turn to the Cell and fire one ball once ready. Press again for the next.</li>
 *   <li>Left bumper: switch between the hooded shot (front) and the backward shot.</li>
 *   <li>D-pad up/down: RPM. Right/left: raise/lower the hood (launch angle falls/rises).</li>
 *   <li>Y: turn to the shot and load the table's RPM and hood for this distance and direction.</li>
 *   <li>B: drive to the next test spot ({@link FieldRoute#GRID}, on the side of the Cell in use), turn to the
 *       shot and load the table. Any stick, trigger or B again stops it.</li>
 *   <li>X: the last ball went in. A: it bounced out (backspin). Unmarked: a plain miss.</li>
 *   <li>Share (back): driving from the far side; turns the sticks round. On by default.</li>
 *   <li>Left/right stick press: aim 1 degree further left/right. Kept apart for the two directions.</li>
 *   <li>During INIT, X picks blue and B picks red. Blue unless changed.</li>
 * </ul>
 *
 * <p>Nothing aims until the cameras have seen the Hive tags: point the camera side at the Hive first.
 */
@TeleOp(name = "Shot Tuning", group = "Tuning")
public class ShotTuning extends PsiKitIterativeOpMode {
  private static final String CSV_PATH = "/sdcard/FIRST/ShotTuning.csv";
  private static final String CSV_HEADER = "session,shot,kind,mode,lip_m,x_m,y_m,heading_deg,cell_x_m,cell_y_m,"
      + "head_err_deg,toward_mps,sideways_mps,target_rpm,rpm_at_feed,launch_deg,hood_at_feed_deg,battery_v,"
      + "head_trim_deg,spot,rpm_dip";
  private static final double RPM_STEP = 50;
  private static final double HOOD_STEP_DEG = 1;
  private static final double MIN_RPM = 1000;
  private static final double HEADING_TRIM_STEP_DEG = 1;

  // A feed counts as a ball once the flywheels dip this far within the window. The one-ball gate pulses the feeder
  // again ~0.8 s after a pulse that delivered nothing; those dipped under 150 rpm (6 Nov 082517)
  private static final double BALL_DIP_RPM = 200;
  private static final double BALL_DIP_WINDOW_S = 0.6;

  private static final double ORIENT_TIMEOUT_S = 3;
  private static final double ORIENT_DONE_DEG = 1.0;
  private static final double SPOT_SETTLE_S = 1.5; // Aims at the spot while the tags pull the pose in

  private Robot robot;
  private final ElapsedTime clock = new ElapsedTime();
  private final String session = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
  private boolean backward = false;
  private double rpm = 3400;
  private double hoodLiftDeg = 0; // Above the lower stop; the launch angle is ZERO_ANGLE_DEG less this
  private boolean tableLoaded = false;
  private double hoodedTrimDeg = 0; // Anticlockwise (left), per direction: the ball leaves opposite sides
  private double backwardTrimDeg = 0;
  private int shots = 0;
  private int scored = 0;
  private int bounced = 0;
  private int emptyFeeds = 0;
  private String lastShotRow = null; // Without its kind, so X can write it again as IN
  private String lastMark = null; // IN or BOUNCE once X or A marks the last ball; unmarked is a plain miss
  private String fileError = null;

  // The feed waiting for its dip: the row as the feed started
  private String pendingRow = null;
  private double pendingStartS;
  private double pendingStartRpm;
  private double pendingMinRpm;

  // Driving to a test spot
  private int spot = -1;
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
    Logger.recordMetadata("OpMode", "Shot Tuning");
    Logger.recordMetadata("TuningSession", session);
  }

  @Override
  protected void onPsiKitInit() {
    robot = new Robot(hardwareMap, telemetry, gamepad1);
    robot.init();
    robot.setTuningMode(true);
    robot.setAlliance(Alliance.BLUE);
    robot.setDriverOnFarSide(true);
    pushShot();
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
    if (justPressed(pad.back, wasBack)) {
      robot.setDriverOnFarSide(!robot.isDriverOnFarSide());
    }
    if (justPressed(pad.left_bumper, wasLeftBumper)) {
      backward = !backward;
      tableLoaded = false; // The other direction's table, as soon as the distance is known
    }
    if (justPressed(pad.y, wasY)) {
      tableLoaded = false;
      orientUntilS = nowS + ORIENT_TIMEOUT_S;
    }
    if (!tableLoaded) {
      loadTable();
    }
    if (justPressed(pad.b, wasB)) {
      if (driving) {
        stopDriving("stopped (B)");
      } else {
        startDriving(nowS, pose);
      }
    }
    if (justPressed(pad.dpad_up, wasDpadUp)) {
      rpm = Math.min(robot.shooter.getMaxRpm(), rpm + RPM_STEP);
    }
    if (justPressed(pad.dpad_down, wasDpadDown)) {
      rpm = Math.max(MIN_RPM, rpm - RPM_STEP);
    }
    double maxLiftDeg = HoodSubsystem.ZERO_ANGLE_DEG - HoodSubsystem.MIN_ANGLE_DEG;
    if (justPressed(pad.dpad_right, wasDpadRight)) {
      hoodLiftDeg = Math.min(maxLiftDeg, hoodLiftDeg + HOOD_STEP_DEG);
    }
    if (justPressed(pad.dpad_left, wasDpadLeft)) {
      hoodLiftDeg = Math.max(0, hoodLiftDeg - HOOD_STEP_DEG);
    }
    double trimStep = (justPressed(pad.left_stick_button, wasLeftStick) ? HEADING_TRIM_STEP_DEG : 0)
        - (justPressed(pad.right_stick_button, wasRightStick) ? HEADING_TRIM_STEP_DEG : 0);
    if (backward) {
      backwardTrimDeg += trimStep;
    } else {
      hoodedTrimDeg += trimStep;
    }
    String mark = justPressed(pad.x, wasX) ? "IN" : justPressed(pad.a, wasA) ? "BOUNCE" : null;
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
    pushShot();
    addTelemetry();
    robot.periodic();

    if (robot.feedStartedThisLoop()) {
      startPending(nowS); // A retry pulse replaces a feed that hasn't dipped: only the last can carry the ball
    }
    checkPending(nowS);
    // The other key changes a mark; the report takes each ball's last one
    if (mark != null && lastShotRow != null && !mark.equals(lastMark)) {
      scored += ("IN".equals(mark) ? 1 : 0) - ("IN".equals(lastMark) ? 1 : 0);
      bounced += ("BOUNCE".equals(mark) ? 1 : 0) - ("BOUNCE".equals(lastMark) ? 1 : 0);
      lastMark = mark;
      append(String.format(Locale.US, "%s,%d,%s,%s", session, shots, mark, lastShotRow));
    }
    Logger.recordOutput("Tuning/Backward", backward);
    Logger.recordOutput("Tuning/Rpm", rpm);
    Logger.recordOutput("Tuning/LaunchDeg", launchDeg());
    Logger.recordOutput("Tuning/HeadingTrimDeg", headingTrimDeg());
    Logger.recordOutput("Tuning/Shots", shots);
    Logger.recordOutput("Tuning/Scored", scored);
    Logger.recordOutput("Tuning/EmptyFeeds", emptyFeeds);
    Logger.recordOutput("Tuning/Bounced", bounced);
    Logger.recordOutput("Tuning/LastMark", lastMark == null ? "" : lastMark);
    Logger.recordOutput("Tuning/Spot", spot);
    Logger.recordOutput("Tuning/Driving", driving);
    Logger.recordOutput("Tuning/DriveStatus", driveStatus);
  }

  @Override
  protected void onPsiKitStop() {
    if (robot != null) {
      robot.setDriveOverride(null);
      robot.setForceAim(false);
      robot.stop();
    }
  }

  // Lines go to the Driver Station and the Panels Telemetry tab, before the robot's own (which end with the update)
  private void addTelemetry() {
    line("TUNING", String.format("%s  %.0f rpm  launch %.1f deg (lift %.1f)  aim %s",
        backward ? "BACKWARD" : "FRONT hooded", rpm, launchDeg(), hoodLiftDeg,
        headingTrimDeg() == 0 ? "straight" : String.format("%.0f deg %s", Math.abs(headingTrimDeg()),
            headingTrimDeg() > 0 ? "left" : "right")));
    line("Balls", String.format("%d fired, %d in, %d bounced out%s%s", shots, scored, bounced,
        lastShotRow == null ? "" : lastMark != null ? "  (last " + lastMark + ")"
            : "  (last a miss: X in, A bounced out)",
        emptyFeeds == 0 ? "" : String.format("  [%d feeds without a ball]", emptyFeeds)));
    double[] table = tableShot();
    Aim aim = robot.getLastAim();
    line("Table here (Y)", aim == null ? "Cell not known: point the cameras at the Hive tags"
        : table == null ? String.format("no %s shot from %.2f m", backward ? "backward" : "hooded",
            aim.shot.distanceToLipM)
        : String.format("%.0f rpm  launch %.1f deg (lift %.1f)  at %.2f m", table[0],
            HoodSubsystem.ZERO_ANGLE_DEG - table[1], table[1], aim.shot.distanceToLipM));
    line("Spot (B: next)", (spot < 0 ? "none yet" : "G" + spot + (driving ? " driving" : ""))
        + (driveStatus.isEmpty() ? "" : "  " + driveStatus));
    line("Keys", "LB front/back  Dpad rpm/hood  Y aim+table  B next spot  RB fire  X in  A bounced  Share "
        + (robot.isDriverOnFarSide() ? "(far side)" : "(alliance wall)"));
    if (fileError != null) {
      line("CSV", fileError);
    }
  }

  private void line(String key, String value) {
    telemetry.addData(key, value);
    robot.addPanelsLine(key, value);
  }

  // ---- Driving to a spot, and turning to the shot

  private void startDriving(double nowS, Pose pose) {
    Aim aim = robot.getLastAim();
    if (aim == null || !robot.isPoseValid()) {
      driveStatus = "B needs the Cell known: point the cameras at the Hive";
      return;
    }
    spot = (spot + 1) % FieldRoute.GRID.length;
    // GRID is for the Cell at -x, -y; mirrored onto the Cell in use
    double mirrorX = robot.getAimCellX() < 0 ? 1 : -1;
    double mirrorY = robot.getAimCellY() < 0 ? 1 : -1;
    double targetX = FieldRoute.GRID[spot][0] * mirrorX;
    double targetY = FieldRoute.GRID[spot][1] * mirrorY;
    route = FieldRoute.route(pose.xM, pose.yM, targetX, targetY);
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
    driveStatus = String.format("to %.2f, %.2f", targetX, targetY);
  }

  private void stopDriving(String why) {
    driving = false;
    driveStatus = why;
  }

  /** Sets the robot's scripted drive and aim for this loop: driving to a spot, settling there, or turning (Y). */
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
        tableLoaded = false; // This spot's table, from the settled pose
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

  // ---- The shot

  private double launchDeg() {
    return HoodSubsystem.ZERO_ANGLE_DEG - hoodLiftDeg;
  }

  private void pushShot() {
    robot.setTuningShot(backward, rpm, launchDeg());
    robot.setHeadingTrimDeg(headingTrimDeg());
  }

  private double headingTrimDeg() {
    return backward ? backwardTrimDeg : hoodedTrimDeg;
  }

  /** Loads {@link #tableShot}. Waits for a known Cell; keeps the current values where the table has no shot. */
  private void loadTable() {
    if (robot.getLastAim() == null) {
      return;
    }
    double[] table = tableShot();
    if (table != null) {
      rpm = table[0];
      hoodLiftDeg = table[1];
    }
    tableLoaded = true;
  }

  /**
   * {rpm, hood lift} the table gives for the robot's distance standing still, as Lebot Teleop would take it; null
   * with the Cell unknown or no shot. Backward shots outside the table use its nearest row.
   */
  private double[] tableShot() {
    Aim aim = robot.getLastAim();
    if (aim == null) {
      return null;
    }
    double distM = aim.shot.distanceToLipM;
    if (backward) {
      double[] table = ShootOnTheMoveConstants.BACKWARD_DIST_M;
      Shot shot = ShootOnTheMove.calculateBackward(
          Math.max(table[0], Math.min(table[table.length - 1], distM)), 0, 0);
      return shot.valid ? new double[] {Math.round(shot.targetRpm / 10.0) * 10.0,
          Math.round((HoodSubsystem.ZERO_ANGLE_DEG - shot.hoodDeg) * 10.0) / 10.0} : null;
    }
    Shot shot = ShootOnTheMove.calculate(distM, 0, 0);
    return shot.valid ? new double[] {
        Math.round((shot.targetRpm + ShootOnTheMoveConstants.HOODED_RPM_OFFSET) / 10.0) * 10.0,
        Math.round((HoodSubsystem.ZERO_ANGLE_DEG - shot.hoodDeg) * 10.0) / 10.0} : null;
  }

  // ---- The CSV

  /** As the feed starts, before the ball slows the wheels; written once the dip shows a ball left. */
  private void startPending(double nowS) {
    Aim aim = robot.getLastAim();
    if (aim == null) {
      return;
    }
    Pose pose = robot.getPose();
    pendingRow = String.format(Locale.US,
        "%s,%.3f,%.3f,%.3f,%.1f,%.3f,%.3f,%.2f,%.3f,%.3f,%.0f,%.0f,%.2f,%.2f,%.2f,%.1f,%s",
        backward ? "BACKWARD" : "HOODED", aim.shot.distanceToLipM, pose.xM, pose.yM,
        Math.toDegrees(pose.headingRad), robot.getAimCellX(), robot.getAimCellY(),
        Math.toDegrees(aim.headingErrorRad(pose.headingRad)), aim.shot.speedTowardCellMps,
        aim.shot.sidewaysSpeedMps, rpm, robot.shooter.getRpm(), launchDeg(),
        HoodSubsystem.ticksToAngle(robot.hood.getPosition()), robot.getBatteryVolts(), headingTrimDeg(),
        spot < 0 ? "" : "G" + spot);
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
      Logger.recordOutput("Tuning/ShotRow", lastShotRow);
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
      RobotLog.ee("ShotTuning", e, "Can't write %s", CSV_PATH);
    }
  }

  private static boolean justPressed(boolean pressedNow, boolean pressedLastLoop) {
    return pressedNow && !pressedLastLoop;
  }
}
