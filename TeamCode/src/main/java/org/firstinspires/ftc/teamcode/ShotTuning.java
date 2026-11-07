package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.utils.Aim;
import org.firstinspires.ftc.teamcode.utils.AimTrimSearch;
import org.firstinspires.ftc.teamcode.utils.BackwardShotSearch;
import org.firstinspires.ftc.teamcode.utils.BallMark;
import org.firstinspires.ftc.teamcode.utils.MarkKeys;
import org.firstinspires.ftc.teamcode.utils.Pose;
import org.firstinspires.ftc.teamcode.utils.ShootOnTheMove;
import org.firstinspires.ftc.teamcode.utils.ShootOnTheMoveConstants;
import org.firstinspires.ftc.teamcode.utils.Shot;
import org.firstinspires.ftc.teamcode.utils.ShotSearch;
import org.firstinspires.ftc.teamcode.utils.ShotTuningSettings;
import org.firstinspires.ftc.teamcode.utils.TuningResume;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.PsiKitIterativeOpMode;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Drive to test spots and shoot, to find the hood angle (or RPM) range that scores from each, and how moving shots
 * miss. Every ball that leaves is written to {@link #CSV_PATH} with where the robot was and the shot it took, and
 * every mark the driver gives it.
 *
 * <p>At each spot a search ({@link ShotSearch}) starts from the table's shot. Mark every ball and it moves the hood
 * for the next one: out from the first scoring angle until a ball goes long, then short, halving the gap until both
 * edges are known, then {@link ShotTuningSettings#CONFIRM_BALLS} at the middle. Where the table's hood is on a stop,
 * and for every backward shot, it searches the RPM instead at the table's launch angle. Left and right misses move the aim trim ({@link AimTrimSearch}). The report
 * (sotm-tools/tuning_report.py) turns the ranges into table rows.
 *
 * <p>Controls (gamepad 1), otherwise as Lebot Teleop:
 * <ul>
 *   <li>Sticks: nothing. The robot drives itself, and holds itself on the spot: knocked off it while being
 *       loaded, it drives back.</li>
 *   <li>Left trigger held: intake and indexer.</li>
 *   <li>Right bumper: turn to the Cell and fire one ball once ready. Press again for the next.</li>
 *   <li>X tapped: the last ball went in. X held, D-pad tapped, X let go: where it missed. Up long, down short,
 *       left, right, as seen from behind the shot; two can be combined. A: it went in and bounced out.</li>
 *   <li>B: drive to the next test spot not yet done for this shot ({@link FieldRoute#GRID}, on the side of the
 *       Cell in use; the backward shot's first {@link #BACKWARD_SPOTS}) and start its search. B again stops it.</li>
 *   <li>Y: turn to the shot, load the table and restart the search from where the robot is.</li>
 *   <li>Start: a moving pass through the spot, firing one ball as it passes: toward the Cell, away, then across
 *       each way, skipping any that would leave the area. Start again stops it.</li>
 *   <li>Left bumper: switch between the hooded shot (front) and the backward shot.</li>
 *   <li>Nothing sets the shot by hand: the search sets the hood (or RPM) and the marks the aim.</li>
 *   <li>During INIT, Y switches between carrying on the CSV's last session (at its spot, with its search) and
 *       starting a new one. It carries on unless changed.</li>
 *   <li>During INIT, X picks blue and B picks red. Blue unless changed; while that alliance's Cell isn't known
 *       it switches to the Cell the cameras see.</li>
 * </ul>
 *
 * <p>Nothing aims until the cameras have seen the Hive tags: point the camera side at the Hive first.
 */
@TeleOp(name = "Shot Tuning", group = "Tuning")
public class ShotTuning extends PsiKitIterativeOpMode {
  // A new file: the old one's header has fewer columns
  private static final String CSV_PATH = "/sdcard/FIRST/ShotTuning_v2.csv";
  private static final String CSV_HEADER = "session,shot,kind,mode,lip_m,x_m,y_m,heading_deg,cell_x_m,cell_y_m,"
      + "head_err_deg,toward_mps,sideways_mps,target_rpm,rpm_at_feed,launch_deg,hood_at_feed_deg,battery_v,"
      + "head_trim_deg,spot,rpm_dip,pass,search";
  private static final double MIN_RPM = 1000;
  private static final double RESUME_ROUTE_M = 0.3; // Further than this from the resumed spot, it drives the route
  private static final int BACKWARD_SPOTS = 6; // GRID's first six are within the backward shot's reach

  // A feed counts as a ball once the flywheels dip this far within the window. The one-ball gate pulses the feeder
  // again ~0.8 s after a pulse that delivered nothing; those dipped under 150 rpm (6 Nov 082517)
  private static final double BALL_DIP_RPM = 200;
  private static final double BALL_DIP_WINDOW_S = 0.6;

  private static final double ORIENT_TIMEOUT_S = 3;
  private static final double ORIENT_DONE_DEG = 1.0;
  private static final double SPOT_SETTLE_S = 1.5; // Aims at the spot while the tags pull the pose in

  private enum Arrive { SPOT, RESUMED_SPOT, PASS_START, PASS_BACK }
  private enum PassStage { NONE, TO_START, READY, RUN, BACK }
  private static final String[] PASS_NAMES = {"TOWARD", "AWAY", "ACROSS_LEFT", "ACROSS_RIGHT"};

  private Robot robot;
  private final ElapsedTime clock = new ElapsedTime();
  private String session = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
  private boolean backward = false;
  private double rpm = 3400;
  private double hoodLiftDeg = 0; // Above the lower stop; the launch angle is ZERO_ANGLE_DEG less this
  private boolean tableLoaded = false;
  private boolean restartSearch = false; // With the next table load
  private AimTrimSearch hoodedTrim;
  private AimTrimSearch backwardTrim;
  private final MarkKeys keys = new MarkKeys();
  private int shots = 0;
  private int scored = 0;
  private int bounced = 0;
  private int emptyFeeds = 0;
  private String lastShotRow = null; // Without its kind, so a mark can write it again
  private String lastMark = null;
  private String fileError = null;
  private String lastAdjust = "";

  // The search at this spot
  private ShotSearch search;
  private int searchId = 0;
  private boolean searchRpm; // Searching the RPM, the table's hood being on a stop, or the backward shot
  private BackwardShotSearch backSearch; // The backward shot's: steps the hood, its RPM search is search
  private double searchStartLaunchDeg;
  private String searchKey = "";
  private double searchX;
  private double searchY;
  private double searchDistM;
  private double searchStart;
  // The search's balls, so a changed mark can replay it
  private final List<Double> searchValues = new ArrayList<>();
  private final List<BallMark> searchMarks = new ArrayList<>();
  private final Map<String, String> found = new LinkedHashMap<>(); // Finished searches, by mode and spot
  private final Map<String, String> inProgress = new LinkedHashMap<>();

  // The feed waiting for its dip: the row as the feed started, and what it was fired with
  private String pendingRow = null;
  private double pendingStartS;
  private double pendingStartRpm;
  private double pendingMinRpm;
  private String pendingPass = "";
  private String pendingSearch = "";
  private int pendingSearchId = -1; // -1: not a search ball
  private double pendingValue;
  // The last ball's, for its mark
  private int lastShotSearchId = -1;
  private double lastShotValue;
  private boolean lastShotMarked;
  private boolean lastShotFromPass;

  // Driving to a point
  private int spot = -1;
  private boolean driving = false;
  private Arrive arrive = Arrive.SPOT;
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
  private double settleDeadlineS = Double.NaN;
  private int settleFixCount;
  private boolean settleNewSearch;
  private double arriveFixAgeS = Double.POSITIVE_INFINITY; // Since the drive's last tag fix, on arriving
  private double orientUntilS = Double.NaN;
  // Backward: turning the cameras to the Hive before driving
  private boolean looking = false;
  private double lookUntilS;
  private int lookFixCount = -1; // Fixes once turned; -1 until then
  private double lookTurnedS;
  private double lookX;
  private double lookY;
  private Arrive lookArrive;
  private int lookSpot;
  private double findTagsSinceS = Double.NaN; // Turning to find the tags since; NaN while not
  private double findTagsStepS; // This turn step or pause began
  private String driveStatus = "";
  private double driveTargetX;
  private double driveTargetY;
  private double[] holdPoint = null; // The spot, once reached

  // Moving pass
  private PassStage passStage = PassStage.NONE;
  private int passCycle = 0;
  private int passCycleSpot = Integer.MIN_VALUE;
  private int passKind;
  private double passCentreX;
  private double passCentreY;
  private double passStartX;
  private double passStartY;
  private double passEndX;
  private double passEndY;
  private double passStageS;
  private boolean passBallLeft;
  private String passStatus = "";

  // Carrying on the CSV's last session
  private TuningResume resumeData;
  private boolean resume = true;
  private boolean resumeSpotPending = false; // Holds the resumed spot once the Cell and pose are known
  private String resumeNote = "";

  private boolean wasLeftBumper;
  private boolean wasB;
  private boolean wasY;
  private boolean wasStart;

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
    hoodedTrim = new AimTrimSearch(ShotTuningSettings.TRIM_STEP_DEG, ShotTuningSettings.TRIM_MIN_STEP_DEG);
    backwardTrim = new AimTrimSearch(ShotTuningSettings.TRIM_STEP_DEG, ShotTuningSettings.TRIM_MIN_STEP_DEG);
    robot = new Robot(hardwareMap, telemetry, gamepad1);
    robot.init();
    robot.setTuningMode(true);
    robot.setAlliance(Alliance.BLUE);
    robot.setDriverOnFarSide(true);
    robot.setSticksLocked(true);
    robot.setAllianceFromTags(true); // Tunes on whichever Cell the cameras face
    resumeData = readLastSession();
    pushShot();
  }

  @Override
  protected void onPsiKitInitLoop() {
    if (robot != null) {
      Gamepad pad = robot.getDriver();
      if (justPressed(pad.y, wasY)) {
        resume = !resume;
      }
      wasY = pad.y;
      telemetry.addData("Session", resumeData == null ? "new (no earlier balls in the CSV)"
          : resume ? String.format(Locale.US, "CARRY ON %s at %s, %d balls so far (Y: start a new one)",
              resumeData.session, TuningResume.key(resumeData.backward, resumeData.spot), resumeData.balls.size())
          : "NEW (Y: carry on " + resumeData.session + ")");
      robot.initLoop();
    }
  }

  @Override
  protected void onPsiKitStart() {
    if (robot != null) {
      robot.start();
    }
    if (resume && resumeData != null) {
      applyResume(resumeData);
    }
    clock.reset();
  }

  private static TuningResume readLastSession() {
    File file = new File(CSV_PATH);
    if (!file.exists()) {
      return null;
    }
    List<String> lines = new ArrayList<>();
    try (BufferedReader in = new BufferedReader(new FileReader(file))) {
      for (String line = in.readLine(); line != null; line = in.readLine()) {
        lines.add(line);
      }
    } catch (IOException e) {
      RobotLog.ee("ShotTuning", e, "Can't read %s", CSV_PATH);
      return null;
    }
    return TuningResume.fromCsv(lines);
  }

  /** Carries on the session: its numbers, spot, trims, and the search at its spot replayed from its marks. */
  private void applyResume(TuningResume r) {
    session = r.session;
    shots = r.lastShot;
    scored = r.scored;
    bounced = r.bounced;
    backward = r.backward;
    spot = r.spot;
    hoodedTrim.set(r.hoodedTrimDeg);
    backwardTrim.set(r.backwardTrimDeg);
    String current = TuningResume.key(r.backward, r.spot);
    for (Map.Entry<String, List<TuningResume.Ball>> e : r.searchBallsBySpot().entrySet()) {
      if (e.getKey().equals(current)) {
        continue;
      }
      String phase = e.getValue().get(e.getValue().size() - 1).phase();
      ("CONFIRM".equals(phase) || "DONE".equals(phase) ? found : inProgress)
          .put(e.getKey(), TuningResume.summary(e.getValue()));
    }
    List<TuningResume.Ball> run = r.currentSearchRun();
    if (run.isEmpty() || !ShotTuningSettings.AUTO_SEARCH || (r.backward && !fromBackwardTable(run.get(0)))) {
      restartSearch = true; // A new search at this spot once the Cell is known
    } else {
      TuningResume.Ball first = run.get(0);
      TuningResume.Ball last = run.get(run.size() - 1);
      searchRpm = first.search.startsWith("rpm:");
      searchId++;
      searchStartLaunchDeg = first.launchDeg;
      beginSearch(searchRpm ? first.rpm : first.launchDeg);
      for (TuningResume.Ball b : run) {
        if (b.mark != null) {
          recordSearchBall(searchRpm ? b.rpm : b.launchDeg, BallMark.parse(b.mark));
        }
      }
      rpm = first.rpm;
      hoodLiftDeg = Math.max(0, Math.min(maxLiftDeg(), HoodSubsystem.ZERO_ANGLE_DEG - first.launchDeg));
      applySearchValue();
      searchKey = current;
      searchX = last.xM;
      searchY = last.yM;
      searchDistM = last.lipM;
      tableLoaded = true;
      noteSearch();
    }
    resumeSpotPending = spot >= 0;
    resumeNote = String.format(Locale.US, "carried on %s: %d balls, at %s", session, r.balls.size(),
        search == null ? "a new search here" : searchKey);
    lastAdjust = resumeNote;
  }

  /**
   * Whether a backward run began as a search starts now: on the RPM, from the table's launch angle for where it
   * stood. Runs that searched the launch angle went the wrong way; Shot Tuning 085918 (7 Nov) began one at 0.90 m's
   * angle from 0.64 m, the table loaded for a Cell guessed before the tags were seen.
   */
  private static boolean fromBackwardTable(TuningResume.Ball first) {
    double[] table = ShootOnTheMoveConstants.BACKWARD_DIST_M;
    Shot shot = ShootOnTheMove.calculateBackward(
        Math.max(table[0], Math.min(table[table.length - 1], first.lipM)), 0, 0);
    return first.search.startsWith("rpm:") && shot.valid && Math.abs(first.launchDeg - shot.hoodDeg) <= 0.6;
  }

  /** GRID's spot, mirrored onto the Cell in use; GRID is for the Cell at -x, -y. */
  private double[] spotPoint(int index) {
    double mirrorX = robot.getAimCellX() < 0 ? 1 : -1;
    double mirrorY = robot.getAimCellY() < 0 ? 1 : -1;
    return new double[] {FieldRoute.GRID[index][0] * mirrorX, FieldRoute.GRID[index][1] * mirrorY};
  }

  /** Once the Cell and pose are known, holds the resumed spot, driving the route there if it's far. */
  private void holdResumedSpot(double nowS, Pose pose) {
    if (!resumeSpotPending || driving || robot.getLastAim() == null || !robot.isPoseValid()) {
      return;
    }
    resumeSpotPending = false;
    double[] p = spotPoint(spot);
    if (search != null) {
      // The CSV's position is on the Cell used then; the tags may now place the robot by the other one
      searchX = p[0];
      searchY = p[1];
    }
    if (Math.hypot(p[0] - pose.xM, p[1] - pose.yM) > RESUME_ROUTE_M) {
      startRoute(nowS, pose, p[0], p[1], Arrive.RESUMED_SPOT, spot);
    } else {
      holdPoint = p;
      if (backward) {
        arriveFixAgeS = Double.POSITIVE_INFINITY;
        settle(nowS, false);
      } else {
        orientUntilS = nowS + ORIENT_TIMEOUT_S;
      }
    }
  }

  @Override
  protected void onPsiKitLoop() {
    double nowS = clock.seconds();
    Gamepad pad = robot.getDriver();
    Pose pose = robot.getPose();
    BallMark mark = keys.update(pad.x, pad.a, pad.dpad_up, pad.dpad_down, pad.dpad_left, pad.dpad_right);
    if (justPressed(pad.left_bumper, wasLeftBumper)) {
      backward = !backward;
      tableLoaded = false; // The other direction's table, as soon as the distance is known
      restartSearch = true;
    }
    if (justPressed(pad.y, wasY)) {
      findTagsSinceS = Double.NaN;
      tableLoaded = false;
      restartSearch = true;
      orientUntilS = nowS + ORIENT_TIMEOUT_S;
    }
    if (!tableLoaded) {
      loadTable();
    }
    if (justPressed(pad.b, wasB)) {
      if (driving || looking || passStage != PassStage.NONE) {
        stopDriving("stopped (B)");
      } else {
        startDrivingToNextSpot(nowS, pose);
      }
    }
    if (justPressed(pad.start, wasStart)) {
      if (passStage != PassStage.NONE) {
        stopDriving("pass stopped (Start)");
      } else if (!driving) {
        startPass(nowS, pose);
      }
    }
    holdResumedSpot(nowS, pose);
    wasLeftBumper = pad.left_bumper;
    wasB = pad.b;
    wasY = pad.y;
    wasStart = pad.start;
    autoDrive(nowS, pose);
    pushShot();
    addTelemetry(nowS, pose);
    robot.periodic();

    if (robot.feedStartedThisLoop()) {
      startPending(nowS); // A retry pulse replaces a feed that hasn't dipped: only the last can carry the ball
      if (passStage == PassStage.RUN) {
        passBallLeft = true;
      }
    }
    checkPending(nowS);
    if (mark != null) {
      applyMark(mark);
    }
    logState();
  }

  @Override
  protected void onPsiKitStop() {
    if (robot != null) {
      robot.setDriveOverride(null);
      robot.setForceAim(false);
      robot.setTuningLive(false);
      robot.setScriptedOneBall(false);
      robot.setHoldPoint(null);
      robot.setSticksLocked(false);
      robot.setTurnOverride(0);
      robot.stop();
    }
  }

  private void logState() {
    Logger.recordOutput("Tuning/Backward", backward);
    Logger.recordOutput("Tuning/Rpm", rpm);
    Logger.recordOutput("Tuning/LaunchDeg", launchDeg());
    Logger.recordOutput("Tuning/HeadingTrimDeg", trim().trimDeg());
    Logger.recordOutput("Tuning/Shots", shots);
    Logger.recordOutput("Tuning/Scored", scored);
    Logger.recordOutput("Tuning/EmptyFeeds", emptyFeeds);
    Logger.recordOutput("Tuning/Bounced", bounced);
    Logger.recordOutput("Tuning/LastMark", lastMark == null ? "" : lastMark);
    Logger.recordOutput("Tuning/Spot", spot);
    Logger.recordOutput("Tuning/Driving", driving);
    Logger.recordOutput("Tuning/Looking", looking);
    Logger.recordOutput("Tuning/CamerasToHive", camerasToHive());
    Logger.recordOutput("Tuning/DriveStatus", driveStatus);
    Logger.recordOutput("Tuning/Search/Phase", search == null ? "" : search.phase().name());
    Logger.recordOutput("Tuning/Session", session);
    Logger.recordOutput("Tuning/Search/Rpm", searchRpm);
    Logger.recordOutput("Tuning/Search/Next", search == null ? Double.NaN : search.next());
    Logger.recordOutput("Tuning/Search/Range", search == null ? new double[] {Double.NaN, Double.NaN}
        : search.range());
    Logger.recordOutput("Tuning/Pass/Stage", passStage.name());
    Logger.recordOutput("Tuning/Pass/Kind", passStage == PassStage.NONE ? "" : PASS_NAMES[passKind]);
    Logger.recordOutput("Tuning/Pass/Status", passStatus);
  }

  // ---- Marks, and the search they feed

  private void applyMark(BallMark mark) {
    if (lastShotRow == null) {
      lastAdjust = "no ball to mark yet";
      return;
    }
    String name = mark.toString();
    if (name.equals(lastMark)) {
      return;
    }
    scored += (mark.in ? 1 : 0) - ("IN".equals(lastMark) ? 1 : 0);
    bounced += (mark.bounced ? 1 : 0) - ("BOUNCE".equals(lastMark) ? 1 : 0);
    lastMark = name;
    append(String.format(Locale.US, "%s,%d,%s,%s", session, shots, name, lastShotRow));
    if (lastShotMarked) {
      // The trim keeps the first mark; the search replays with the new one
      if (search != null && lastShotSearchId == searchId && !lastShotFromPass && !searchMarks.isEmpty()) {
        searchMarks.set(searchMarks.size() - 1, mark);
        replaySearch();
        applySearchValue();
        noteSearch();
        lastAdjust = name + " (changed): " + nextText();
      } else {
        lastAdjust = "mark changed to " + name;
      }
      return;
    }
    lastShotMarked = true;
    if (lastShotFromPass) {
      lastAdjust = "moving ball marked " + name;
      return;
    }
    StringBuilder adjust = new StringBuilder();
    if (ShotTuningSettings.AUTO_TRIM) {
      double change = trim().record(mark);
      if (change != 0) {
        adjust.append(String.format(Locale.US, "aim %+.2f deg  ", change));
      }
    }
    if (search != null && lastShotSearchId == searchId) {
      recordSearchBall(lastShotValue, mark);
      applySearchValue();
      adjust.append(nextText());
      noteSearch();
    } else if (search != null) {
      adjust.append(lastShotSearchId < 0 ? "(fired off the spot: not counted in the search)"
          : "(ball from before this search)");
    }
    lastAdjust = name + ": " + adjust.toString().trim();
  }

  /** Starts this spot's search from the table's shot, if searching is on. */
  private void startSearch(Pose pose, double distM) {
    search = null;
    if (!ShotTuningSettings.AUTO_SEARCH) {
      return;
    }
    searchId++;
    searchX = pose.xM;
    searchY = pose.yM;
    searchDistM = distM;
    searchKey = (backward ? "BACK " : "FRONT ") + (spot < 0 ? String.format(Locale.US, "%.2fm", distM) : "G" + spot);
    double launch = launchDeg();
    // On a stop the hood can't move one way, so the RPM moves the ball instead. The backward shot always searches
    // the RPM: flatter reaches further but comes in under the rim, steeper falls short, and both get marked short
    // (Shot Tuning 085918, 7 Nov: back G1 lowered 94 -> 88 degrees, 12 of 12 short). More RPM raises and lengthens it.
    searchRpm = backward
        || launch >= HoodSubsystem.ZERO_ANGLE_DEG - 0.5 || launch <= HoodSubsystem.MIN_ANGLE_DEG + 0.5;
    searchStartLaunchDeg = launch;
    beginSearch(searchRpm ? rpm : launch);
    applySearchValue();
    lastAdjust = "search started at " + (searchRpm ? String.format(Locale.US, "%.0f rpm", rpm)
        : String.format(Locale.US, "launch %.1f deg", launchDeg()));
  }

  private void beginSearch(double start) {
    searchStart = start;
    searchValues.clear();
    searchMarks.clear();
    search = newSearch(start);
  }

  private void recordSearchBall(double value, BallMark mark) {
    searchValues.add(value);
    searchMarks.add(mark);
    record(value, mark);
  }

  private void record(double value, BallMark mark) {
    if (backSearch != null) {
      backSearch.record(value, mark);
      search = backSearch.rpm();
    } else {
      search.record(value, mark);
    }
  }

  private void replaySearch() {
    search = newSearch(searchStart);
    for (int i = 0; i < searchValues.size(); i++) {
      record(searchValues.get(i), searchMarks.get(i));
    }
  }

  /** What the search fires next, for the driver. */
  private String nextText() {
    if (backSearch != null) {
      return String.format(Locale.US, "next %.0f rpm at launch %.1f deg%s", search.next(), backSearch.launchDeg(),
          backSearch.steepened() > 0 ? " (hood steeper: no window flatter)" : "");
    }
    return searchRpm ? String.format(Locale.US, "next %.0f rpm", search.next())
        : String.format(Locale.US, "next launch %.1f deg", search.next());
  }

  /** A search of the RPM or the launch angle, by {@link #searchRpm}, from this value. */
  private ShotSearch newSearch(double start) {
    backSearch = null;
    if (searchRpm && backward) {
      backSearch = new BackwardShotSearch(searchStartLaunchDeg, start, ShotTuningSettings.BACKWARD_STEEPEN_DEG,
          ShotTuningSettings.BACKWARD_STEEPEST_DEG, ShotTuningSettings.SEARCH_RPM_STEP,
          ShotTuningSettings.SEARCH_RPM_RESOLUTION, MIN_RPM, robot.shooter.getMaxRpm(), ShotTuningSettings.CONFIRM_BALLS);
      return backSearch.rpm();
    }
    if (searchRpm) {
      return new ShotSearch(start, 1, ShotTuningSettings.SEARCH_RPM_STEP, ShotTuningSettings.SEARCH_RPM_RESOLUTION,
          10, MIN_RPM, robot.shooter.getMaxRpm(), ShotTuningSettings.CONFIRM_BALLS);
    }
    // A flatter hooded shot goes further
    return new ShotSearch(start, -1, ShotTuningSettings.SEARCH_HOOD_STEP_DEG,
        ShotTuningSettings.SEARCH_HOOD_RESOLUTION_DEG, 0.1, HoodSubsystem.MIN_ANGLE_DEG,
        HoodSubsystem.ZERO_ANGLE_DEG, ShotTuningSettings.CONFIRM_BALLS);
  }

  private void applySearchValue() {
    if (search == null) {
      return;
    }
    if (searchRpm) {
      rpm = search.next();
      if (backSearch != null) {
        hoodLiftDeg = Math.max(0, Math.min(maxLiftDeg(), HoodSubsystem.ZERO_ANGLE_DEG - backSearch.launchDeg()));
      }
    } else {
      hoodLiftDeg = Math.max(0, Math.min(maxLiftDeg(), HoodSubsystem.ZERO_ANGLE_DEG - search.next()));
    }
  }

  private void noteSearch() {
    if (search == null) {
      return;
    }
    double[] r = search.range();
    String unit = searchRpm ? "rpm" : "deg";
    String text = Double.isNaN(r[0]) ? String.format(Locale.US, "%d balls, none in", search.balls())
        : String.format(Locale.US, searchRpm ? "%.0f (%.0f-%.0f) %s, %d/%d in" : "%.1f (%.1f-%.1f) %s, %d/%d in",
            search.centre(), r[0], r[1], unit, search.scored(), search.balls());
    if (backSearch != null) {
      text += String.format(Locale.US, " at launch %.1f deg", backSearch.launchDeg());
    }
    if (search.phase() == ShotSearch.Phase.DONE) {
      found.put(searchKey, text);
      inProgress.remove(searchKey);
    } else {
      inProgress.put(searchKey, text);
    }
  }

  private String searchLine(Pose pose) {
    if (!ShotTuningSettings.AUTO_SEARCH) {
      return "off (Panels: ShotTuningSettings.AUTO_SEARCH)";
    }
    if (search == null) {
      return "starts at a spot (B) or with Y";
    }
    String what = backSearch != null ? "rpm, " + nextText()
        : searchRpm ? String.format(Locale.US, "rpm, next %.0f", search.next())
        : String.format(Locale.US, "hood, next launch %.1f deg", search.next());
    String phase;
    switch (search.phase()) {
      case FIND: phase = "looking for a scoring ball"; break;
      case LONG_EDGE: phase = "finding the long edge"; break;
      case SHORT_EDGE: phase = "finding the short edge"; break;
      case CONFIRM: phase = String.format(Locale.US, "confirming the middle %d/%d", search.confirmFired(),
          ShotTuningSettings.CONFIRM_BALLS); break;
      default: phase = "done: B for the next spot"; break;
    }
    double moved = Math.hypot(pose.xM - searchX, pose.yM - searchY);
    String warn = moved > ShotTuningSettings.SEARCH_MOVED_M ? String.format(Locale.US, "  MOVED %.2f m: Y restarts here", moved)
        : "";
    String contrary = search.contrary() >= 2
        ? String.format(Locale.US, "  %d misses marked the other way than the setting moves", search.contrary()) : "";
    return String.format(Locale.US, "%s %.2f m: %s, %s, %d/%d in%s%s", searchKey, searchDistM, phase, what,
        search.scored(), search.balls(), warn, contrary);
  }

  private String spotsLeft() {
    StringBuilder front = new StringBuilder();
    for (int i = 0; i < FieldRoute.GRID.length; i++) {
      if (!found.containsKey("FRONT G" + i)) { front.append(" G").append(i); }
    }
    StringBuilder back = new StringBuilder();
    for (int i = 0; i < BACKWARD_SPOTS; i++) {
      if (!found.containsKey("BACK G" + i)) { back.append(" G").append(i); }
    }
    return "front" + (front.length() == 0 ? " all done" : front) + "  |  back" + (back.length() == 0 ? " all done" : back);
  }

  // Lines go to the Driver Station and the Panels Telemetry tab, before the robot's own (which end with the update)
  private void addTelemetry(double nowS, Pose pose) {
    double trimDeg = trim().trimDeg();
    line("TUNING", String.format(Locale.US, "%s %s  %.0f rpm  launch %.1f deg (lift %.1f)  aim %s",
        robot.getAlliance(), backward ? "BACKWARD" : "FRONT hooded", rpm, launchDeg(), hoodLiftDeg,
        Math.abs(trimDeg) < 0.01 ? "straight" : String.format(Locale.US, "%.2f deg %s", Math.abs(trimDeg),
            trimDeg > 0 ? "left" : "right")));
    String markHint = keys.isMarking() ? "MARKING " + keys.pending() + " (let go of X to save)"
        : lastShotRow != null && !lastShotMarked ? "MARK THE LAST BALL: tap X in, hold X + D-pad for the miss, A bounced"
        : lastAdjust;
    line("Mark", markHint);
    line("Balls", String.format(Locale.US, "%d fired, %d in, %d bounced out%s", shots, scored, bounced,
        emptyFeeds == 0 ? "" : String.format(Locale.US, "  [%d feeds without a ball]", emptyFeeds)));
    line("Search", searchLine(pose));
    line("Found", found.isEmpty() ? "none yet" : joined(found));
    if (!inProgress.isEmpty()) {
      line("Unfinished", joined(inProgress));
    }
    line("Spots left", spotsLeft());
    double[] table = tableShot();
    Aim aim = robot.getLastAim();
    line("Table here", aim == null ? "Cell not known: point the cameras at the Hive tags"
        : table == null ? String.format(Locale.US, "no %s shot from %.2f m", backward ? "backward" : "hooded",
            aim.shot.distanceToLipM)
        : String.format(Locale.US, "%.0f rpm  launch %.1f deg  at %.2f m", table[0],
            HoodSubsystem.ZERO_ANGLE_DEG - table[1], aim.shot.distanceToLipM));
    line("Spot (B: next)", (spot < 0 ? "none yet: B drives to G0" : "G" + spot + (driving ? " driving" : ""))
        + (driveStatus.isEmpty() ? "" : "  " + driveStatus)
        + (holdPoint == null ? "" : String.format(Locale.US, "  held, %.1f cm off",
            100 * Math.hypot(pose.xM - holdPoint[0], pose.yM - holdPoint[1]))));
    line("Pass (Start)", passStage == PassStage.NONE ? (passStatus.isEmpty() ? "none" : passStatus)
        : PASS_NAMES[passKind] + " " + passStage.name().toLowerCase(Locale.US).replace('_', ' ')
            + (passStatus.isEmpty() ? "" : "  " + passStatus));
    line("Keys", "X in / hold X+Dpad miss  A in then rolled out  RB fire  B next spot  Start pass  Y restart  LB front/back");
    if (fileError != null) {
      line("CSV", fileError);
    }
  }

  private static String joined(Map<String, String> map) {
    StringBuilder s = new StringBuilder();
    for (Map.Entry<String, String> e : map.entrySet()) {
      if (s.length() > 0) { s.append("  |  "); }
      s.append(e.getKey()).append(' ').append(e.getValue());
    }
    return s.toString();
  }

  private void line(String key, String value) {
    telemetry.addData(key, value);
    robot.addPanelsLine(key, value);
  }

  // ---- Driving to a spot or a pass's start, the pass itself, and turning to the shot

  private void startDrivingToNextSpot(double nowS, Pose pose) {
    Aim aim = robot.getLastAim();
    if (aim == null || !robot.isPoseValid()) {
      driveStatus = "B needs the Cell known: point the cameras at the Hive";
      return;
    }
    boolean[] done = new boolean[backward ? BACKWARD_SPOTS : FieldRoute.GRID.length];
    for (int i = 0; i < done.length; i++) {
      done[i] = found.containsKey(TuningResume.key(backward, i));
    }
    int next = TuningResume.nextSpot(spot, done);
    if (next < 0) {
      driveStatus = backward ? "every backward spot is done" : "every front spot is done: LB for the backward shot";
      return;
    }
    resumeSpotPending = false;
    double[] p = spotPoint(next);
    startRoute(nowS, pose, p[0], p[1], Arrive.SPOT, next);
  }

  /** Drives the route to a point; for the backward shot, first turns the cameras to the Hive and waits for fixes. */
  private void startRoute(double nowS, Pose pose, double targetX, double targetY, Arrive why, int toSpot) {
    settleUntilS = Double.NaN;
    orientUntilS = Double.NaN;
    if (!backward) {
      spot = toSpot;
      holdPoint = null;
      startDrivingTo(nowS, pose, targetX, targetY, why);
      return;
    }
    lookSpot = toSpot;
    looking = true; // Holds where it is meanwhile
    lookUntilS = nowS + ShotTuningSettings.LOOK_TIMEOUT_S;
    lookFixCount = -1;
    lookX = targetX;
    lookY = targetY;
    lookArrive = why;
    driveStatus = "turning the cameras to the Hive";
  }

  /** At the spot: aims (the cameras to the Hive for the backward shot) while the tags pull the pose in. */
  private void settle(double nowS, boolean newSearch) {
    settleUntilS = nowS + SPOT_SETTLE_S;
    settleDeadlineS = nowS + ShotTuningSettings.SETTLE_MAX_S;
    settleFixCount = robot.getPositionFixes();
    settleNewSearch = newSearch;
  }

  /** Backward spot changes are made facing the Hive, where the cameras see its tags. */
  private boolean camerasToHive() {
    return backward && passStage == PassStage.NONE && (looking || driving || !Double.isNaN(settleUntilS));
  }

  private void startDrivingTo(double nowS, Pose pose, double targetX, double targetY, Arrive why) {
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
    arrive = why;
    driveTargetX = targetX;
    driveTargetY = targetY;
    driveStatus = String.format(Locale.US, "to %.2f, %.2f", targetX, targetY);
  }

  private void stopDriving(String why) {
    driving = false;
    looking = false;
    if (passStage != PassStage.NONE) {
      passStage = PassStage.NONE;
      passStatus = why;
    }
    driveStatus = why;
  }

  /** A moving pass through where the robot is: the next of toward, away and across that stays in the area. */
  private void startPass(double nowS, Pose pose) {
    if (robot.getLastAim() == null || !robot.isPoseValid()) {
      passStatus = "Start needs the Cell known: point the cameras at the Hive";
      return;
    }
    if (spot != passCycleSpot) {
      passCycle = 0;
      passCycleSpot = spot;
    }
    double ux = robot.getAimCellX() - pose.xM;
    double uy = robot.getAimCellY() - pose.yM;
    double norm = Math.hypot(ux, uy);
    if (norm < 1e-6) {
      return;
    }
    ux /= norm;
    uy /= norm;
    double h = ShotTuningSettings.PASS_HALF_LENGTH_M;
    // Toward the Cell, away, then across to the left and right as seen from behind the shot
    double[][] dirs = {{ux, uy}, {-ux, -uy}, {-uy, ux}, {uy, -ux}};
    for (int i = 0; i < dirs.length; i++) {
      int k = (passCycle + i) % dirs.length;
      double cx = holdPoint != null ? holdPoint[0] : pose.xM;
      double cy = holdPoint != null ? holdPoint[1] : pose.yM;
      double sx = cx - dirs[k][0] * h;
      double sy = cy - dirs[k][1] * h;
      double ex = cx + dirs[k][0] * h;
      double ey = cy + dirs[k][1] * h;
      if (FieldRoute.pathClear(sx, sy, ex, ey)) {
        passCycle = k + 1;
        passKind = k;
        passCentreX = holdPoint != null ? holdPoint[0] : pose.xM;
        passCentreY = holdPoint != null ? holdPoint[1] : pose.yM;
        passStartX = sx;
        passStartY = sy;
        passEndX = ex;
        passEndY = ey;
        passStatus = "";
        passStage = PassStage.TO_START;
        startDrivingTo(nowS, pose, sx, sy, Arrive.PASS_START);
        return;
      }
    }
    passStatus = "no pass from here stays in the area";
  }

  /** Sets the robot's scripted drive, aim and fire for this loop. */
  private void autoDrive(double nowS, Pose pose) {
    double[] stick = null;
    boolean aim = false;
    boolean live = false;
    boolean fire = false;
    if (driving || passStage == PassStage.RUN) {
      if (robot.getPositionFixes() != lastFixCount) {
        lastFixCount = robot.getPositionFixes();
        lastFixS = nowS;
      }
      if (Math.hypot(pose.xM - progressX, pose.yM - progressY) >= 0.05) {
        progressX = pose.xM;
        progressY = pose.yM;
        progressS = nowS;
      }
      String stop = !robot.isPoseValid() ? "stopped: pose not placed by the tags"
          : nowS - lastFixS > FieldRoute.GOTO_BLIND_S ? "stopped: no tags seen for a while"
          : !FieldRoute.headedSafely(pose, robot.getVelocity()) ? "stopped: heading for the fence or the Hive"
          : driving && nowS - driveStartS > driveTimeoutS ? "stopped: took too long"
          : null;
      if (stop != null) {
        stopDriving(stop);
      }
    }
    if (looking) {
      aim = true;
      Aim last = robot.getLastAim();
      boolean turned = last != null && !last.backward
          && Math.abs(Math.toDegrees(last.headingErrorRad(pose.headingRad))) <= ShotTuningSettings.LOOK_DONE_DEG;
      if (turned && lookFixCount < 0) {
        lookFixCount = robot.getPositionFixes();
        lookTurnedS = nowS;
      }
      if (lookFixCount >= 0 && (robot.getPositionFixes() - lookFixCount >= ShotTuningSettings.SPOT_TAG_FIXES
          || nowS - lookTurnedS >= ShotTuningSettings.LOOK_FIX_WAIT_S)) {
        looking = false;
        spot = lookSpot;
        holdPoint = null;
        startDrivingTo(nowS, pose, lookX, lookY, lookArrive);
      } else if (lookFixCount < 0 && nowS >= lookUntilS) {
        looking = false;
        driveStatus = "stopped: couldn't turn the cameras to the Hive";
      }
    }
    if (driving) {
      double[] w = route.get(routeIndex);
      boolean last = routeIndex == route.size() - 1;
      double away = Math.hypot(w[0] - pose.xM, w[1] - pose.yM);
      if (last && away <= FieldRoute.GOTO_TOLERANCE_M) {
        driving = false;
        arriveFixAgeS = nowS - lastFixS;
        if (arrive == Arrive.SPOT) {
          driveStatus = backward ? "at the spot: checking it on the tags" : "at the spot";
          holdPoint = new double[] {driveTargetX, driveTargetY};
          settle(nowS, true);
        } else if (arrive == Arrive.RESUMED_SPOT) {
          driveStatus = backward ? "back at the spot: checking it on the tags" : "back at the spot";
          holdPoint = new double[] {driveTargetX, driveTargetY};
          if (backward) {
            settle(nowS, false); // The resumed search carries on
          } else {
            orientUntilS = nowS + ORIENT_TIMEOUT_S;
          }
        } else if (arrive == Arrive.PASS_START) {
          driveStatus = "at the pass start";
          passStage = PassStage.READY;
          passStageS = nowS;
        } else {
          driveStatus = "back at the spot";
          passStage = PassStage.NONE;
          orientUntilS = nowS + ORIENT_TIMEOUT_S; // Turns back to the standing shot; the search goes on
        }
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
    if (passStage == PassStage.READY) {
      live = true;
      aim = true;
      if (nowS - passStageS >= ShotTuningSettings.PASS_READY_S) {
        passStage = PassStage.RUN;
        passStageS = nowS;
        passBallLeft = false;
        progressS = nowS;
        lastFixS = nowS;
      }
    } else if (passStage == PassStage.RUN) {
      live = true;
      aim = true;
      double lx = passEndX - passStartX;
      double ly = passEndY - passStartY;
      double length = Math.hypot(lx, ly);
      double ux = lx / length;
      double uy = ly / length;
      double along = (pose.xM - passStartX) * ux + (pose.yM - passStartY) * uy;
      double cross = -(pose.xM - passStartX) * uy + (pose.yM - passStartY) * ux; // Left of the path positive
      boolean timedOut = nowS - passStageS > length / 0.1 + 3;
      if (along >= length - 0.03 || timedOut) {
        passStatus = passBallLeft ? "ball fired: mark it"
            : timedOut ? "no ball: the pass took too long" : "no ball: the shot never got ready at speed";
        passStage = PassStage.BACK;
        startDrivingTo(nowS, pose, passCentreX, passCentreY, Arrive.PASS_BACK);
      } else {
        double speed = backward ? ShotTuningSettings.PASS_STICK_BACKWARD : ShotTuningSettings.PASS_STICK_HOODED;
        stick = new double[] {speed * ux + FieldRoute.CROSS_GAIN * cross * uy,
            speed * uy - FieldRoute.CROSS_GAIN * cross * ux};
        fire = !passBallLeft && along >= length / 2 - ShotTuningSettings.PASS_FIRE_LEAD_M;
      }
    }
    if (!Double.isNaN(settleUntilS)) {
      boolean fixedHere = robot.getPositionFixes() > settleFixCount && holdPoint != null
          && Math.hypot(pose.xM - holdPoint[0], pose.yM - holdPoint[1]) <= ShotTuningSettings.SPOT_FIX_M;
      boolean fixedOnArrival = arriveFixAgeS <= ShotTuningSettings.SPOT_RECENT_FIX_S;
      if (nowS >= settleUntilS && (!backward || fixedHere || fixedOnArrival || nowS >= settleDeadlineS)) {
        settleUntilS = Double.NaN;
        if (backward) {
          driveStatus = (fixedHere ? "spot checked on the tags here"
              : fixedOnArrival ? String.format(Locale.US, "spot checked on the tags %.1f s before arriving", arriveFixAgeS)
              : "spot NOT checked: no tags seen here or arriving") + ", turning around";
        }
        if (settleNewSearch) {
          tableLoaded = false; // This spot's table and a new search, from the settled pose
          restartSearch = true;
        }
        orientUntilS = nowS + ORIENT_TIMEOUT_S;
      } else {
        aim = true;
      }
    }
    if (!Double.isNaN(orientUntilS)) {
      Aim last = robot.getLastAim();
      boolean lined = last != null && last.backward == backward
          && Math.abs(Math.toDegrees(last.headingErrorRad(pose.headingRad))) <= ORIENT_DONE_DEG;
      if (lined || nowS >= orientUntilS) {
        orientUntilS = Double.NaN;
      } else {
        aim = true;
      }
    }
    boolean lost = robot.getLastAim() == null || !robot.isPoseValid();
    double turn = 0;
    if (lost && !driving && !looking && passStage == PassStage.NONE) {
      if (Double.isNaN(findTagsSinceS)) {
        findTagsSinceS = nowS;
        findTagsStepS = nowS;
      }
      if (nowS - findTagsSinceS < ShotTuningSettings.FIND_TAGS_TIMEOUT_S) {
        boolean seen = robot.getTagsSeenAgoS() < 0.3;
        double pause = seen ? ShotTuningSettings.FIND_TAGS_SEEN_PAUSE_S : ShotTuningSettings.FIND_TAGS_PAUSE_S;
        double inCycle = nowS - findTagsStepS;
        if (inCycle >= pause + ShotTuningSettings.FIND_TAGS_STEP_S) {
          findTagsStepS = nowS;
          inCycle = 0;
        }
        turn = inCycle < pause ? 0 : ShotTuningSettings.FIND_TAGS_TURN;
        driveStatus = seen ? "turning to find the Hive tags: tags in view, waiting for a fix"
            : "turning to find the Hive tags";
      } else {
        driveStatus = "no Hive tags found turning: turn the cameras to the Hive (Y tries again)";
      }
    } else if (!lost && !Double.isNaN(findTagsSinceS)) {
      findTagsSinceS = Double.NaN;
      driveStatus = "found the Hive tags";
    }
    robot.setTurnOverride(turn);
    robot.setDriveOverride(stick);
    // Not during a pass: from its start it would pull the robot back to the spot
    robot.setHoldPoint(passStage == PassStage.NONE ? holdPoint : null);
    robot.setForceAim(aim);
    robot.setTuningLive(live);
    robot.setScriptedOneBall(fire);
  }

  // ---- The shot

  private double launchDeg() {
    return HoodSubsystem.ZERO_ANGLE_DEG - hoodLiftDeg;
  }

  private static double maxLiftDeg() {
    return HoodSubsystem.ZERO_ANGLE_DEG - HoodSubsystem.MIN_ANGLE_DEG;
  }

  private AimTrimSearch trim() {
    return backward ? backwardTrim : hoodedTrim;
  }

  private void pushShot() {
    robot.setTuningShot(backward && !camerasToHive(), rpm, launchDeg());
    robot.setHeadingTrimDeg(trim().trimDeg());
  }

  /** Loads {@link #tableShot}, and starts a search if one was asked for. Waits for a known Cell. */
  private void loadTable() {
    Aim aim = robot.getLastAim();
    if (aim == null) {
      return;
    }
    // A new search waits for the robot to be placed at its spot: before the tags are seen the Cell may be a guess
    if (restartSearch && (resumeSpotPending || camerasToHive() || !robot.isPoseValid())) {
      return;
    }
    double[] table = tableShot();
    if (table != null) {
      rpm = table[0];
      hoodLiftDeg = table[1];
    }
    tableLoaded = true;
    if (restartSearch) {
      restartSearch = false;
      startSearch(robot.getPose(), aim.shot.distanceToLipM);
    }
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
    boolean fromPass = passStage == PassStage.READY || passStage == PassStage.RUN;
    double moved = Math.hypot(pose.xM - searchX, pose.yM - searchY);
    boolean searchable = !fromPass && search != null && moved <= ShotTuningSettings.SEARCH_MOVED_M;
    pendingPass = fromPass ? PASS_NAMES[passKind] : "";
    pendingSearch = fromPass ? "PASS"
        : search == null ? (ShotTuningSettings.AUTO_SEARCH ? "none" : "off")
        : !searchable ? "moved"
        : (searchRpm ? "rpm:" : "hood:") + search.phase().name();
    pendingSearchId = searchable ? searchId : -1;
    pendingValue = searchRpm ? rpm : launchDeg();
    // A moving pass fires the tables' shot, so that is what it records
    double targetRpm = fromPass ? robot.shooter.getTargetRpm() : rpm;
    double launch = fromPass ? aim.shot.hoodDeg : launchDeg();
    pendingRow = String.format(Locale.US,
        "%s,%.3f,%.3f,%.3f,%.1f,%.3f,%.3f,%.2f,%.3f,%.3f,%.0f,%.0f,%.2f,%.2f,%.2f,%.2f,%s",
        backward ? "BACKWARD" : "HOODED", aim.shot.distanceToLipM, pose.xM, pose.yM,
        Math.toDegrees(pose.headingRad), robot.getAimCellX(), robot.getAimCellY(),
        Math.toDegrees(aim.headingErrorRad(pose.headingRad)), aim.shot.speedTowardCellMps,
        aim.shot.sidewaysSpeedMps, targetRpm, robot.shooter.getRpm(), launch,
        HoodSubsystem.ticksToAngle(robot.hood.getPosition()), robot.getBatteryVolts(), trim().trimDeg(),
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
      lastShotRow = String.format(Locale.US, "%s,%.0f,%s,%s", pendingRow, dip, pendingPass, pendingSearch);
      lastShotSearchId = pendingSearchId;
      lastShotValue = pendingValue;
      lastShotMarked = false;
      lastShotFromPass = !pendingPass.isEmpty();
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
