package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.utils.Aim;
import org.firstinspires.ftc.teamcode.utils.BallMark;
import org.firstinspires.ftc.teamcode.utils.MarkKeys;
import org.firstinspires.ftc.teamcode.utils.Pose;
import org.firstinspires.ftc.teamcode.utils.ShootOnTheMoveConstants;
import org.firstinspires.ftc.teamcode.utils.VolleyCalibration;
import org.firstinspires.ftc.teamcode.utils.VolleyCalibration.Ball;
import org.firstinspires.ftc.teamcode.utils.VolleyTrial;
import org.firstinspires.ftc.teamcode.utils.VolleyAutoTuner;
import org.psilynx.psikit.core.Logger;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Four-ball calibration at Shot Tuning's field spots. Circle/B drives to the next unfinished spot,
 * then holds it while loading with LT. Hold RB to aim and fire up to four balls; release to abort.
 * The volley ends at four detected balls or a timeout, and all four physical balls must be marked.
 *
 * <p>Marks match Shot Tuning: X tap IN; X+D-pad miss; A BOUNCE; Y unseen/not fired.
 * Hold X+Circle and release X for a collision with another ball (Circle alone always moves/cancels).
 * All numerical settings are searched automatically; the D-pad only marks miss direction.
 * INIT Y chooses carry-on/new searches; X/B select alliance. Sticks do not drive the robot.
 * Searches and feed power are saved separately for every spot in a new CSV; old data is untouched.
 */
@TeleOp(name = "Volley Tuning", group = "Tuning")
public class VolleyTuning extends Main {
  static final String CSV_PATH = "/sdcard/FIRST/VolleyTuning_v3.csv";
  static final String CSV_HEADER = VolleyCalibration.CSV_HEADER
      + ",spot,xM,yM,detected,elapsedS,releaseReason,indexerScale,profile,trialSeconds,headingTrimDeg";
  private final MarkKeys keys = new MarkKeys();
  private final VolleySpotDrive position = new VolleySpotDrive();
  private final VolleyTrial trial = new VolleyTrial();
  private final ElapsedTime clock = new ElapsedTime();
  private final Map<Integer, VolleyAutoTuner> spots = new HashMap<>();
  private final String session = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
  private List<String> history = new ArrayList<>();
  private VolleyCalibration cal;
  private int selectedSpot;
  private boolean carryOn = true;
  private boolean wasInitY, wasCircle, wasY;
  private boolean collisionMark;
  private VolleyAutoTuner auto;
  private VolleyAutoTuner.Settings settings;
  private VolleyAutoTuner.Settings trialSettings;
  private double firstFeedS;
  private double trialSeconds;
  private double[] fedSnapshot; // rpm, target, distance, toward, sideways, hood, battery, elapsed
  private String fileError;
  private String lastMarkText = "";
  private String trialReason = "";

  @Override protected void configureRobot(Robot robot) {
    robot.setVolleyMode(true);
    robot.setHoodedOnly(true);
    robot.setButtonsReserved(true);
    robot.setSticksLocked(true);
    robot.setAlliance(Alliance.BLUE);
    robot.setDriverOnFarSide(true); // Same placement/driver convention as Shot and SOTM Tuning.
    robot.setDriveBrake(true);
    robot.setAllianceFromTags(true);
    robot.setHoldFire(true);
    robot.setSpinOnlyOnFire(true);
    history = readCsv();
    selectedSpot = lastSpot(history);
    auto = tuner(selectedSpot); cal = auto.calibration();
  }

  @Override protected boolean usesTunedVolley() { return false; } // It sets each trial's settings itself

  @Override protected void onPsiKitInitLoop() {
    boolean y = robot().getDriver().y;
    if (y && !wasInitY) { carryOn = !carryOn; }
    wasInitY = y;
    telemetry.addData("Volley Tuning", carryOn ? "CARRY ON saved spots (Y: new searches)"
        : "NEW searches (Y: carry on)");
    telemetry.addData("Controls", "Circle moves/holds; LT loads FOUR; hold RB fires; X/D-pad/A mark");
    super.onPsiKitInitLoop();
  }

  @Override protected void onPsiKitStart() {
    if (!carryOn) {
      history.clear(); spots.clear(); selectedSpot = 0;
      auto = tuner(selectedSpot); cal = auto.calibration();
      append(resetCsvRow(session));
    }
    wasCircle = robot().getDriver().b;
    wasY = robot().getDriver().y;
    clock.reset();
    super.onPsiKitStart();
  }

  @Override protected void onPsiKitLoop() {
    Robot robot = robot();
    double now = clock.seconds();
    Gamepad g = robot.getDriver(); // Same previous-loop merged pad used by Shot/SOTM Tuning.
    readMarks(g);
    if (g.b && !wasCircle && !g.x && !keys.isMarking()) { circle(now); }
    wasCircle = g.b;
    Pose pose = robot.getPose();
    position.acquire(now, pose, robot.isPoseValid() && robot.getLastAim() != null,
        robot.getPositionFixes(), robot.getAimCellX(), robot.getAimCellY(), robot.getTagsSeenAgoS());
    position.update(now, pose, robot.getVelocity(), robot.isPoseValid(), robot.getPositionFixes());
    if (!trial.active()) {
      settings = auto.current();
      robot.setVolleyTrimsDeg(settings.firstTrim, settings.laterTrim);
      robot.setVolleyHeadingTrimDeg(settings.headingTrim);
      robot.setVolleyFeedPower(settings.feeder);
      robot.setVolleyIndexerPower(settings.feeder * settings.indexerScale);
      robot.setVolleyBoostRpm(settings.boostRpm);
      robot.setVolleyFastRecovery(settings.fastRecovery);
    }
    trial.input(now, g.right_bumper, position.ready(), cal.marksPending());
    finishTrial();
    robot.setTurnOverride(position.turn());
    robot.setDriveOverride(position.drive());
    robot.setHoldPoint(position.holdPoint());
    robot.setForceAim(position.shouldAim());
    robot.setHoldFire(!trial.active()); // RT alone never starts a calibration trial.
    addTelemetry();
    super.onPsiKitLoop();
    track(now);
    Logger.recordOutput("VolleyTuning/Spot", selectedSpot);
    Logger.recordOutput("VolleyTuning/PositionStage", position.stage().name());
    Logger.recordOutput("VolleyTuning/PositionStatus", position.status());
    Logger.recordOutput("VolleyTuning/SearchTurn", position.turn());
    Logger.recordOutput("VolleyTuning/Circle", g.b);
    Logger.recordOutput("VolleyTuning/TrialActive", trial.active());
    Logger.recordOutput("VolleyTuning/DetectedBalls", trial.balls());
    Logger.recordOutput("VolleyTuning/MarkedBalls", cal.marked());
    Logger.recordOutput("VolleyTuning/TrialReason", trialReason);
  }

  private void circle(double now) {
    if (trial.active()) { trial.cancel(); finishTrial(); return; }
    if (position.stage() == VolleySpotDrive.Stage.ACQUIRING
        || position.stage() == VolleySpotDrive.Stage.MOVING
        || position.stage() == VolleySpotDrive.Stage.SETTLING) {
      position.stop("Move cancelled; Circle retries"); return;
    }
    if (cal.marksPending()) { lastMarkText = "Mark all FOUR balls before moving"; return; }
    int next = selectedSpot;
    if (position.ready()) {
      boolean[] done = new boolean[FieldRoute.GRID.length];
      for (int i = 0; i < done.length; i++) {
        done[i] = tuner(i).done();
      }
      next = VolleySpotDrive.nextSpot(selectedSpot, done);
      if (next < 0) { lastMarkText = "All spots confirmed"; return; }
    }
    Robot robot = robot();
    if (position.start(next, robot.getAimCellX(), robot.getAimCellY(), now, robot.getPose(),
        robot.isPoseValid() && robot.getLastAim() != null, robot.getPositionFixes())) {
      selectedSpot = next; auto = tuner(next); cal = auto.calibration();
    }
  }

  private void readMarks(Gamepad g) {
    collisionMark |= g.x && g.b;
    BallMark mark = keys.update(g.x, g.a, g.dpad_up, g.dpad_down, g.dpad_left, g.dpad_right);
    boolean unseen = g.y && !wasY && !g.x;
    wasY = g.y;
    String text = mark != null ? collisionMark ? VolleyCalibration.HIT : mark.toString()
        : unseen ? VolleyCalibration.UNSEEN : null;
    if (!g.x) { collisionMark = false; }
    if (trial.active() || !cal.marksPending() || text == null) { return; }
    Ball ball = cal.mark(text);
    lastMarkText = String.format(Locale.US, "spot %d volley %d ball %d/4: %s",
        selectedSpot + 1, ball.volley, ball.index + 1, text);
    write(ball);
    if (!cal.marksPending()) {
      auto.completed(cal.balls(), trialSeconds, trialReason);
      cal = auto.calibration();
    }
  }

  private void track(double now) {
    Robot robot = robot();
    boolean feed = robot.isVolleyShot() && robot.feedStartedThisLoop();
    boolean left = robot.isVolleyShot() && robot.ballLeftThisLoop();
    boolean wasActive = trial.active();
    if (feed && wasActive) {
      Aim aim = robot.getLastAim();
      fedSnapshot = new double[] {robot.shooter.getRpm(), robot.shooter.getTargetRpm(),
          aim == null ? Double.NaN : aim.shot.distanceToLipM,
          aim == null ? Double.NaN : aim.shot.speedTowardCellMps,
          aim == null ? Double.NaN : aim.shot.sidewaysSpeedMps,
          HoodSubsystem.ticksToAngle(robot.hood.getTarget()), robot.getBatteryVolts(), now};
    }
    if (trial.observe(feed, left)) {
      trialSettings = settings;
      firstFeedS = now;
      for (Ball unseen : cal.startVolley(settings.firstTrim, settings.laterTrim)) { write(unseen); }
    }
    if (wasActive) {
      while (cal.balls().size() < trial.balls()) {
        Ball ball = cal.fed();
        if (fedSnapshot != null) {
          ball.fedRpm = fedSnapshot[0]; ball.targetRpm = fedSnapshot[1]; ball.lipM = fedSnapshot[2];
          ball.towardMps = fedSnapshot[3]; ball.sidewaysMps = fedSnapshot[4]; ball.hoodDeg = fedSnapshot[5];
          ball.batteryVolts = fedSnapshot[6]; ball.elapsedS = fedSnapshot[7];
        }
      }
      if (trial.balls() > 0 && Double.isFinite(robot.getVolleyDipDepthRpm())) {
        cal.dip(trial.balls() - 1, robot.getVolleyLeftAtRpm(), robot.getVolleyDipDepthRpm(),
            ShootOnTheMoveConstants.VOLLEY_DOUBLE_DIP_RPM);
      }
    }
    finishTrial();
  }

  private void finishTrial() {
    if (trial.takeFinished()) {
      cal.finishVolley();
      for (Ball b : cal.balls()) { b.headingTrimDeg = trialSettings.headingTrim; }
      trialReason = trial.reason();
      trialSeconds = Math.max(0.001, clock.seconds() - firstFeedS);
    }
  }

  @Override protected void onPsiKitStop() {
    trial.cancel(); finishTrial();
    if (cal != null) { for (Ball ball : cal.markRestUnseen()) { write(ball); } }
    Robot robot = robot();
    if (robot != null) {
      robot.setHoldFire(true); robot.setForceAim(false); robot.setHoldPoint(null);
      robot.setTurnOverride(0); robot.setDriveOverride(null); robot.setSticksLocked(false); robot.setHoodedOnly(false);
    }
    super.onPsiKitStop();
  }

  private VolleyAutoTuner tuner(int spot) {
    VolleyAutoTuner t = spots.get(spot);
    if (t == null) {
      t = new VolleyAutoTuner();
      if (carryOn) { t.replay(rowsForSpot(history, spot)); }
      spots.put(spot, t);
    }
    return t;
  }

  static String resetCsvRow(String session) {
    String[] names = CSV_HEADER.split(",", -1);
    StringBuilder row = new StringBuilder();
    for (int i = 0; i < names.length; i++) {
      if (i > 0) { row.append(','); }
      if (names[i].equals("session")) { row.append(session); }
      else if (names[i].equals("mark")) { row.append(VolleyCalibration.RESET); }
      else if (names[i].equals("spot")) { row.append(-1); }
    }
    return row.toString();
  }

  /** Keeps independent searches per spot; a global RESET applies to all of them. */
  static List<String> rowsForSpot(List<String> lines, int spot) {
    List<String> out = new ArrayList<>();
    if (lines.isEmpty()) { return out; }
    out.add(lines.get(0));
    int col = column(lines.get(0), "spot");
    int mark = column(lines.get(0), "mark");
    if (col < 0 || mark < 0) { return out; }
    for (String line : lines.subList(1, lines.size())) {
      String[] cells = line.split(",", -1);
      if (cells.length > Math.max(col, mark) && (Integer.toString(spot).equals(cells[col])
          || VolleyCalibration.RESET.equals(cells[mark]))) { out.add(line); }
    }
    return out;
  }

  static int lastSpot(List<String> lines) {
    if (lines.isEmpty()) { return 0; }
    int col = column(lines.get(0), "spot");
    for (int i = lines.size() - 1; col >= 0 && i > 0; i--) {
      String[] c = lines.get(i).split(",", -1);
      if (c.length > col) {
        try {
          int spot = Integer.parseInt(c[col]);
          if (spot == -1) { return 0; }
          if (spot >= 0 && spot < FieldRoute.GRID.length) { return spot; }
        } catch (NumberFormatException ignored) { }
      }
    }
    return 0;
  }

  private static int column(String header, String name) {
    String[] cols = header.split(",", -1);
    for (int i = 0; i < cols.length; i++) { if (cols[i].equals(name)) { return i; } }
    return -1;
  }

  private void addTelemetry() {
    telemetry.addData("POSITION", position.status());
    telemetry.addData("Spot", "%d/%d; intake %s; full %s", selectedSpot + 1, FieldRoute.GRID.length,
        robot().getDriver().left_trigger > Robot.TRIGGER_PRESSED ? "loading (LT)" : "automatic", robot().intake.isFull());
    if (trial.active()) {
      telemetry.addData("VOLLEY", "Hold RB: %d/4 detected; release or Circle cancels", trial.balls());
    } else if (cal.marksPending()) {
      telemetry.addData("MARK", "Ball %d/4: X in; X+D-pad miss; A bounce; X+Circle collision; Y unseen/not fired",
          cal.marked() + 1);
    } else {
      telemetry.addData("READY", "Load FOUR with LT. Hold RB fires at the held spot. Circle moves to next spot.");
    }
    telemetry.addData("Last", lastMarkText);
    telemetry.addData("First ball", VolleyCalibration.describe(cal.firstSearch()));
    telemetry.addData("Balls 2-4", VolleyCalibration.describe(cal.laterSearch()));
    VolleyAutoTuner.Settings next = auto.current();
    telemetry.addData("AUTOMATIC", auto.status());
    telemetry.addData("Trial settings", "feeder %.2f; indexer %.2f; boost %.0f; recovery %s; trims %+.1f/%+.1f",
        next.feeder, next.feeder * next.indexerScale, next.boostRpm,
        next.fastRecovery ? "FULL" : "PID", next.firstTrim, next.laterTrim);
    for (Ball b : cal.balls()) {
      telemetry.addData("Ball " + (b.index + 1), "%s; fed %+.0f RPM; dip %.0f; trim %+.1f; %s",
          b.detected ? "detected" : "dip missed/not fired", b.fedRpm - b.targetRpm, b.dipRpm, b.trimDeg,
          b.mark == null ? "unmarked" : b.mark);
    }
    telemetry.addData("Saved automatically", "Lebot Teleop RB volleys use the best settings so far");
    if (fileError != null) { telemetry.addData("CSV", fileError); }
  }

  private void write(Ball ball) {
    double[] p = VolleySpotDrive.point(selectedSpot, robot().getAimCellX(), robot().getAimCellY());
    append(resultRow(session, cal, ball, trialSettings, selectedSpot, p, trialSeconds, trialReason));
  }

  static String resultRow(String session, VolleyCalibration cal, Ball ball, VolleyAutoTuner.Settings settings,
      int spot, double[] point, double trialSeconds, String reason) {
    return cal.row(session, ball, settings.boostRpm, settings.fastRecovery,
        ShootOnTheMoveConstants.VOLLEY_DOUBLE_DIP_RPM, ball.batteryVolts)
        + String.format(Locale.US, ",%d,%.3f,%.3f,%b,%.3f,%s,%.2f,%d,%.3f", spot, point[0], point[1],
            ball.detected, ball.elapsedS, reason, settings.indexerScale, settings.profile, trialSeconds) + String.format(Locale.US, ",%.2f", settings.headingTrim);
  }

  private List<String> readCsv() {
    List<String> lines = new ArrayList<>();
    File file = new File(CSV_PATH);
    if (!file.exists()) { return lines; }
    try (BufferedReader in = new BufferedReader(new FileReader(file))) {
      for (String line = in.readLine(); line != null; line = in.readLine()) { lines.add(line); }
    } catch (IOException e) {
      fileError = "not read: " + e.getMessage(); RobotLog.ee("VolleyTuning", e, "Can't read CSV");
    }
    return lines;
  }

  private void append(String row) {
    if (Logger.isReplay()) { return; }
    File file = new File(CSV_PATH);
    boolean empty = !file.exists() || file.length() == 0;
    try (FileWriter out = new FileWriter(file, true)) {
      if (empty) { out.write(CSV_HEADER + "\n"); }
      out.write(row + "\n");
      fileError = null;
    } catch (IOException e) {
      fileError = "not saved: " + e.getMessage(); RobotLog.ee("VolleyTuning", e, "Can't write CSV");
    }
  }
}
