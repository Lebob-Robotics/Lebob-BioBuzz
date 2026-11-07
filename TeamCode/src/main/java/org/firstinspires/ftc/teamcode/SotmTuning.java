package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.Aim;
import org.firstinspires.ftc.teamcode.utils.BallMark;
import org.firstinspires.ftc.teamcode.utils.MarkKeys;
import org.firstinspires.ftc.teamcode.utils.Pose;
import org.firstinspires.ftc.teamcode.utils.ShootOnTheMove;
import org.firstinspires.ftc.teamcode.utils.ShootOnTheMoveConstants;
import org.firstinspires.ftc.teamcode.utils.SotmSpotSearch;
import org.firstinspires.ftc.teamcode.utils.SotmTuningSettings;
import org.firstinspires.ftc.teamcode.utils.TuningResume;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.PsiKitIterativeOpMode;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Date;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;

/**
 * Shoot-on-the-move tuning: drives planned passes ({@link SotmPath#plan}) past the Cell and fires one ball during
 * each, with the tables' shot and the moving corrections, exactly as Lebot Teleop would.
 *
 * <p>The steady passes (straight lines and arcs at one speed) are moving spots, tuned as Shot Tuning tunes a standing
 * spot: the same pass is run again and again, one ball each, and the marks move a {@link SotmSpotSearch}, the range
 * as a trim on the launch angle (or the RPM, for a hood near a stop and the backward shot) until both edges are found
 * and confirmed, and the aim as a heading trim from the left and right misses. The passes that change speed or
 * direction come after them, one ball each, as checks. Every ball is written to {@link #CSV_PATH} with the robot's
 * state as the feed started and as the ball left (the flywheels' dip), every mark, and each spot's result (FOUND);
 * sotm-tools/sotm_report.py fits ShootOnTheMoveConstants' moving corrections from them.
 *
 * <p>Controls (gamepad 1):
 * <ul>
 *   <li>B: drive to the start of the next spot or pass not yet done, aim, run it and fire. B again stops. A spot
 *       is run again until its search is done; its last ball has to be marked first.</li>
 *   <li>Start: run the last pass again.</li>
 *   <li>Y: skip the next pass (or the one running): it is written down as skipped. With the last ball still to
 *       mark, Y instead writes it down as not seen.</li>
 *   <li>X tapped: the last ball went in. X held, D-pad tapped, X let go: where it missed. Up long, down short,
 *       left, right, as seen from behind the shot; two can be combined. A: it went in and bounced out.</li>
 *   <li>Left trigger held: intake. Right bumper: fire one ball where it stands.</li>
 *   <li>Left bumper (between passes): hooded or backward shot.</li>
 *   <li>After each pass it drives to the reload spot (SotmTuningSettings.RELOAD_X_M, RELOAD_Y_M) and holds there
 *       for a ball to be loaded; B starts the next pass from there, or on the way.</li>
 *   <li>Back: the fire gates: each pass's own, the match's, or none (Panels: SotmTuningSettings.GATES).</li>
 *   <li>During INIT, Y switches between carrying on (passes marked in the CSV are done) and starting the plan over.</li>
 * </ul>
 *
 * <p>Any stick, either trigger or RB pushed during a drive stops it. Point the cameras at the Hive tags first.
 */
@TeleOp(name = "SOTM Tuning", group = "Tuning")
public class SotmTuning extends PsiKitIterativeOpMode {
  private static final String CSV_PATH = "/sdcard/FIRST/SotmTuning.csv";
  private static final String STATE_COLUMNS = "x_m,y_m,heading_deg,vx_mps,vy_mps,omega_dps,ax_mps2,ay_mps2,lip_m,"
      + "toward_mps,sideways_mps,head_err_deg,turning_deg,target_rpm,rpm,hood_target_deg,hood_deg";
  private static final String CSV_HEADER = "session,ball,kind,mode,plan_key,path,speed_mps,dir_deg,field_frame,"
      + "aim_dist_m,gates,plan_fire_s,window_s,feed_s,dip_after_feed_s,track_err_m,"
      + prefixed("f_") + "," + prefixed("d_")
      + ",cell_x_m,cell_y_m,battery_v,rpm_dip,feed_delay_s,sideways_gain,toward_gain"
      + ",search,range_kind,range_trim,head_trim_deg,hood_per_toward,rpm_per_toward,head_per_sideways";
  // FOUND rows: session,ball,FOUND,mode,plan_key,range_kind,centre,lo,hi,head_trim_deg,balls,scored
  private static final String STANDING_CSV_PATH = "/sdcard/FIRST/ShotTuning_v2.csv";
  private static final double MIN_RPM = 1500;
  private static final double MAX_HOOD_TRIM_DEG = 10;
  private static final double MAX_RPM_TRIM = 800;
  private static final double HOOD_STOP_MARGIN_DEG = 2; // The range is searched by RPM this close to a stop
  private static final double BALL_DIP_RPM = 200; // As Shot Tuning
  private static final double BALL_DIP_WINDOW_S = 0.6;
  private static final double ACCEL_WINDOW_S = 0.1;

  private enum Stage { IDLE, TO_START, READY, RUN, TO_RELOAD, END }

  private Robot robot;
  private final ElapsedTime clock = new ElapsedTime();
  private final String session = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
  private boolean backward = false;
  // Planned on their own thread: on the hub it takes seconds, longer than INIT may block
  private volatile List<SotmPath.Entry> hoodedPlan;
  private volatile List<SotmPath.Entry> backwardPlan;
  private volatile String planError;
  private volatile List<SotmPath.Entry> hoodedOrder; // Spots first, then the checks
  private volatile List<SotmPath.Entry> backwardOrder;
  private final Set<String> done = new HashSet<>(); // Searched, marked or skipped, from the CSV and this run
  private final Set<String> doneInCsv = new HashSet<>();
  // Each spot's balls in the CSV, in order: {range_kind, range_trim, head_trim_deg, last mark or null}
  private final Map<String, List<String[]>> csvSpotBalls = new LinkedHashMap<>();
  private final Map<String, SotmSpotSearch> searches = new HashMap<>();
  private boolean resumed;
  private double standingHoodedTrimDeg;
  private double standingBackwardTrimDeg;
  private boolean carryOn = true;
  private final MarkKeys keys = new MarkKeys();

  private Stage stage = Stage.IDLE;
  private SotmPath.Entry entry; // The pass running, or last run
  private SotmPath path; // Its path for the Cell in use
  private SotmSpotSearch spot; // Its search, when it is a spot
  private boolean relaxed;
  private double stageS;
  private double runS;
  private double pathS; // The plan's clock: waits while the robot is behind along the line
  private static final double STOP_BRAKE_S = 1.5;
  private double stoppedS = Double.NEGATIVE_INFINITY;
  private double lastRunS;
  private boolean fed;
  private double feedPathS = Double.NaN;
  private double trackErrM;
  private String status = "";
  private String reloadStatus = ""; // Shown once it reaches the reload spot

  // Driving to the start
  private List<double[]> route;
  private int routeIndex;
  private double legStartX;
  private double legStartY;
  private double progressX;
  private double progressY;
  private double progressS;
  private double driveStartS;
  private int lastFixCount;
  private double lastFixS;
  private double[] holdPoint;

  // The ball
  private String pendingRow; // Up to and including the feed's state
  private double pendingStartS;
  private double pendingStartRpm;
  private double pendingMinRpm;
  private String pendingKey = "";
  private String pendingSearchKey = "";
  private double pendingRangeTrim;
  private String pendingSearchColumns = "";
  private String lastShotRow; // Without session, ball and kind
  private String lastShotKey = "";
  private String lastShotSearchKey = ""; // The spot the last ball searched, if it did
  private double lastShotRangeTrim;
  private boolean lastShotMarked;
  private String lastMark;
  private String lastBallNote = "";
  private int shots;
  private int scored;
  private int emptyFeeds;
  private String fileError;

  private final ArrayDeque<double[]> velocityHistory = new ArrayDeque<>(); // {t, vx, vy}
  private double[] accel = {0, 0};

  private boolean wasB;
  private boolean wasY;
  private boolean wasStart;
  private boolean wasBack;
  private boolean wasLeftBumper;

  private static String prefixed(String p) {
    StringBuilder s = new StringBuilder();
    for (String c : STATE_COLUMNS.split(",")) {
      if (s.length() > 0) {
        s.append(',');
      }
      s.append(p).append(c);
    }
    return s.toString();
  }

  @Override
  protected int getRlogPort() {
    return Main.LIVE_LOGGING ? Main.RLOG_PORT : 0;
  }

  @Override
  protected void onPsiKitConfigureLogging() {
    Main.configureLogging();
    Logger.recordMetadata("OpMode", "SOTM Tuning");
    Logger.recordMetadata("TuningSession", session);
  }

  @Override
  protected void onPsiKitInit() {
    robot = new Robot(hardwareMap, telemetry, gamepad1);
    robot.init();
    robot.setTuningMode(true);
    robot.setTuningLive(true);
    robot.setAlliance(Alliance.BLUE);
    robot.setDriverOnFarSide(true);
    robot.setSticksLocked(true);
    robot.setAllianceFromTags(true);
    readCsv();
    readStandingTrims();
    Thread planner = new Thread(() -> {
      try {
        long t0 = System.nanoTime();
        List<SotmPath.Entry> b = SotmPath.plan(true);
        List<SotmPath.Entry> h = SotmPath.plan(false);
        backwardOrder = SotmPath.spotsFirst(b);
        hoodedOrder = SotmPath.spotsFirst(h);
        backwardPlan = b;
        hoodedPlan = h;
        RobotLog.ii("SotmTuning", "Planned in %d ms", (System.nanoTime() - t0) / 1000000);
      } catch (RuntimeException e) {
        planError = e.toString();
        RobotLog.ee("SotmTuning", e, "Planning failed");
      }
    }, "SotmPlanner");
    planner.setPriority(Thread.MIN_PRIORITY);
    planner.start();
  }

  @Override
  protected void onPsiKitInitLoop() {
    if (robot == null) {
      return;
    }
    Gamepad pad = robot.getDriver();
    if (pad.y && !wasY) {
      carryOn = !carryOn;
    }
    wasY = pad.y;
    telemetry.addData("Plan", planError != null ? "FAILED: " + planError : !planned() ? "planning the passes..."
        : String.format(Locale.US, "hooded: %d spots + %d checks, backward: %d spots + %d checks",
            spotCount(hoodedPlan), hoodedPlan.size() - spotCount(hoodedPlan), spotCount(backwardPlan),
            backwardPlan.size() - spotCount(backwardPlan)));
    telemetry.addData("Session", carryOn
        ? String.format(Locale.US, "CARRY ON: %d checks done or skipped, %d spots with balls in the CSV "
            + "(Y: start the plan over)", doneInCsv.size(), csvSpotBalls.size())
        : "START OVER: every spot and pass again (Y: carry on)");
    telemetry.addData("Standing aim trims", String.format(Locale.US, "hooded %+.2f deg, backward %+.2f deg "
        + "(spots start from these)", standingHoodedTrimDeg, standingBackwardTrimDeg));
    robot.initLoop();
  }

  @Override
  protected void onPsiKitStart() {
    robot.start();
    if (carryOn) {
      done.addAll(doneInCsv);
    } else {
      csvSpotBalls.clear();
    }
    clock.reset();
  }

  @Override
  protected void onPsiKitStop() {
    if (robot != null) {
      robot.setDriveOverride(null);
      robot.setForceAim(false);
      robot.setTuningLive(false);
      robot.setScriptedOneBall(false);
      robot.setSteadyGateBypassed(false);
      robot.setCloseMovingGateBypassed(false);
      robot.setHeadingTrimDeg(0);
      robot.setHoodAngleTrimDeg(0);
      robot.setRpmTrim(0);
      robot.setHoldPoint(null);
      robot.setSticksLocked(false);
      robot.stop();
    }
  }

  /**
   * The CSV so far: passes skipped, checks marked, and each spot's balls with their trims and last marks, for
   * carrying on its search.
   */
  private void readCsv() {
    File file = new File(CSV_PATH);
    if (!file.exists()) {
      return;
    }
    try (BufferedReader in = new BufferedReader(new FileReader(file))) {
      String header = in.readLine();
      if (header == null) {
        return;
      }
      Map<String, Integer> col = new HashMap<>();
      String[] names = header.split(",");
      for (int i = 0; i < names.length; i++) {
        col.put(names[i], i);
      }
      Map<String, String[]> balls = new LinkedHashMap<>(); // session/ball -> {key, search, kind, trim, head, mark}
      for (String line = in.readLine(); line != null; line = in.readLine()) {
        String[] f = line.split(",", -1);
        String kind = field(f, col, "kind");
        String key = field(f, col, "plan_key");
        String id = field(f, col, "session") + "/" + field(f, col, "ball");
        if (kind.equals("SKIP")) {
          if (!key.isEmpty()) {
            doneInCsv.add(key);
          }
        } else if (kind.equals("SHOT")) {
          balls.put(id, new String[] {key, field(f, col, "search"), field(f, col, "range_kind"),
              field(f, col, "range_trim"), field(f, col, "head_trim_deg"), null});
        } else if (!kind.equals("FOUND") && balls.containsKey(id)) {
          // Later marks replace earlier ones. Not seen is no mark: the search leaves it out and the pass is not done
          balls.get(id)[5] = kind.equals("UNSEEN") ? null : kind;
        }
      }
      for (String[] b : balls.values()) {
        if (b[0].isEmpty()) {
          continue;
        }
        if (b[1].startsWith("SEARCH")) {
          List<String[]> list = csvSpotBalls.get(b[0]);
          if (list == null) {
            list = new ArrayList<>();
            csvSpotBalls.put(b[0], list);
          }
          list.add(new String[] {b[2], b[3], b[4], b[5]});
        } else if (b[5] != null) {
          doneInCsv.add(b[0]);
        }
      }
    } catch (IOException e) {
      RobotLog.ee("SotmTuning", e, "Can't read %s", CSV_PATH);
    }
  }

  private static String field(String[] f, Map<String, Integer> col, String name) {
    Integer i = col.get(name);
    return i == null || i >= f.length ? "" : f[i];
  }

  /** The aim trims Shot Tuning found standing, where each spot's aim search starts. */
  private void readStandingTrims() {
    File file = new File(STANDING_CSV_PATH);
    if (!file.exists()) {
      return;
    }
    List<String> lines = new ArrayList<>();
    try (BufferedReader in = new BufferedReader(new FileReader(file))) {
      for (String line = in.readLine(); line != null; line = in.readLine()) {
        lines.add(line);
      }
      TuningResume r = TuningResume.fromCsv(lines);
      if (r != null) {
        standingHoodedTrimDeg = r.hoodedTrimDeg;
        standingBackwardTrimDeg = r.backwardTrimDeg;
      }
    } catch (IOException | RuntimeException e) {
      RobotLog.ee("SotmTuning", e, "Can't read %s", STANDING_CSV_PATH);
    }
  }

  private static double parseOr(String s, double fallback) {
    try {
      double v = Double.parseDouble(s);
      return Double.isNaN(v) ? fallback : v;
    } catch (NumberFormatException e) {
      return fallback;
    }
  }

  private static boolean isSpot(SotmPath.Entry e) {
    return SotmPath.isSpot(e);
  }

  private static int spotCount(List<SotmPath.Entry> plan) {
    int n = 0;
    for (SotmPath.Entry e : plan) {
      n += isSpot(e) ? 1 : 0;
    }
    return n;
  }

  private boolean planned() {
    return hoodedOrder != null && backwardOrder != null && hoodedPlan != null && backwardPlan != null;
  }

  private List<SotmPath.Entry> plan() {
    List<SotmPath.Entry> p = backward ? backwardOrder : hoodedOrder;
    return p != null ? p : new ArrayList<SotmPath.Entry>();
  }

  /**
   * The search at a spot, from its balls in the CSV if it has any. The range trim's limits keep the shot inside
   * the hood's travel (or the wheels' speed) at the pass's fire point.
   */
  private SotmSpotSearch spotSearch(SotmPath.Entry e, SotmPath p) {
    SotmSpotSearch s = searches.get(e.key);
    if (s != null) {
      return s;
    }
    boolean back = e.key.startsWith("BACKWARD");
    double[] f = p.sample(p.fireS);
    Pose at = new Pose(f[0], f[1], 0);
    double cx = robot.getAimCellX();
    double cy = robot.getAimCellY();
    Aim a = back ? ShootOnTheMove.aimBackward(at, f[2], f[3], cx, cy) : ShootOnTheMove.aimHooded(at, f[2], f[3], cx, cy);
    boolean valid = a.shot.valid && !Double.isNaN(a.hoodNowDeg) && !Double.isNaN(a.rpmNow);
    List<String[]> old = csvSpotBalls.get(e.key);
    boolean rpm = old != null && !old.isEmpty() ? "RPM".equals(old.get(0)[0])
        : back || !valid || a.hoodNowDeg >= HoodSubsystem.ZERO_ANGLE_DEG - HOOD_STOP_MARGIN_DEG
            || a.hoodNowDeg <= HoodSubsystem.MIN_ANGLE_DEG + HOOD_STOP_MARGIN_DEG;
    double min;
    double max;
    if (rpm) {
      double base = valid ? a.rpmNow + (back ? 0 : ShootOnTheMoveConstants.HOODED_RPM_OFFSET) : 3000;
      min = Math.max(-MAX_RPM_TRIM, MIN_RPM - base);
      max = Math.min(MAX_RPM_TRIM, robot.shooter.getMaxRpm() - base);
    } else {
      min = Math.max(-MAX_HOOD_TRIM_DEG, HoodSubsystem.MIN_ANGLE_DEG - a.hoodNowDeg);
      max = Math.min(MAX_HOOD_TRIM_DEG, HoodSubsystem.ZERO_ANGLE_DEG - a.hoodNowDeg);
    }
    double startTrim = old != null && !old.isEmpty() ? parseOr(old.get(0)[2], 0)
        : back ? standingBackwardTrimDeg : standingHoodedTrimDeg;
    s = SotmSpotSearch.withTuningSettings(rpm, min, max, startTrim);
    if (old != null) {
      for (String[] b : old) {
        if (b[3] != null) {
          s.record(parseOr(b[1], 0), parseOr(b[2], Double.NaN), BallMark.parse(b[3]));
        }
      }
    }
    searches.put(e.key, s);
    return s;
  }

  /** Once planned: the spots with balls in the CSV carry on their searches, and those finished are done. */
  private void resumeSpots() {
    if (resumed || !planned()) {
      return;
    }
    resumed = true;
    List<SotmPath.Entry> all = new ArrayList<>(hoodedPlan);
    all.addAll(backwardPlan);
    for (SotmPath.Entry e : all) {
      if (isSpot(e) && csvSpotBalls.containsKey(e.key)) {
        SotmSpotSearch s = spotSearch(e, e.path.mirroredFor(robot.getAimCellX(), robot.getAimCellY()));
        if (s.done()) {
          done.add(e.key);
        }
      }
    }
  }
  private SotmPath.Entry nextEntry() {
    for (SotmPath.Entry e : plan()) {
      if (!done.contains(e.key)) {
        return e;
      }
    }
    return null;
  }

  /** {spots done, spots, checks done, checks} in the plan in use. */
  private int[] doneCounts() {
    int[] n = new int[4];
    for (SotmPath.Entry e : plan()) {
      int i = isSpot(e) ? 0 : 2;
      n[i + 1]++;
      if (done.contains(e.key)) {
        n[i]++;
      }
    }
    return n;
  }

  @Override
  protected void onPsiKitLoop() {
    double nowS = clock.seconds();
    resumeSpots();
    Gamepad pad = robot.getDriver();
    Pose pose = robot.getPose();
    BallMark mark = keys.update(pad.x, pad.a, pad.dpad_up, pad.dpad_down, pad.dpad_left, pad.dpad_right);
    updateAccel(nowS);
    boolean between = stage == Stage.IDLE || stage == Stage.END || stage == Stage.TO_RELOAD;
    boolean pressB = pad.b && !wasB || SotmTuningSettings.PRESS_B;
    SotmTuningSettings.PRESS_B = false;
    if (pressB) {
      if (between) {
        start(nowS, pose, nextEntry());
      } else {
        stop("stopped (B)");
      }
    }
    if (pad.start && !wasStart && between) {
      start(nowS, pose, entry);
    }
    if (pad.y && !wasY && between && lastShotRow != null && !lastShotMarked) {
      // Nobody saw where it went: written down as unseen and left out of the spot's search, which runs the spot again.
      // Before, a spot's unmarked ball held B up and Y skipped a pass instead, so there was no way on
      lastShotMarked = true;
      lastMark = "UNSEEN";
      append(String.format(Locale.US, "%s,%d,UNSEEN,%s", session, shots, lastShotRow));
      status = "last ball written down as not seen: B for the next";
    } else if (pad.y && !wasY) {
      SotmPath.Entry skip = between ? nextEntry() : entry;
      if (!between) {
        stop("skipped");
      }
      if (skip != null) {
        done.add(skip.key);
        append(String.format(Locale.US, "%s,%d,SKIP,%s,%s", session, shots, backward ? "BACKWARD" : "HOODED",
            skip.key));
        status = "skipped " + skip.key;
      }
    }
    if (pad.back && !wasBack) {
      SotmTuningSettings.GATES = (SotmTuningSettings.GATES + 1) % 3;
    }
    if (pad.left_bumper && !wasLeftBumper && between) {
      backward = !backward;
      stage = Stage.IDLE;
      entry = null;
      spot = null;
    }
    wasB = pad.b;
    wasY = pad.y;
    wasStart = pad.start;
    wasBack = pad.back;
    wasLeftBumper = pad.left_bumper;
    if (stage != Stage.IDLE && FieldRoute.driverTookOver(pad)) {
      stop("stopped: the driver took over");
    }
    drive(nowS, pose);
    addTelemetry(pose);
    robot.periodic();

    if (robot.feedStartedThisLoop()) {
      startPending(nowS);
    }
    checkPending(nowS);
    if (mark != null) {
      applyMark(mark);
    }
    logState();
  }

  private void start(double nowS, Pose pose, SotmPath.Entry e) {
    if (!planned()) {
      status = planError != null ? "planning failed: " + planError : "still planning the passes";
      return;
    }
    if (e == null) {
      status = entry == null && nextEntry() == null ? "every pass is done: LB for the other shot" : "no pass to run";
      return;
    }
    if (robot.getLastAim() == null || !robot.isPoseValid()) {
      status = "needs the Cell known: point the cameras at the Hive tags";
      return;
    }
    if (lastShotRow != null && !lastShotMarked && !lastShotSearchKey.isEmpty()) {
      status = "mark the last ball first: its spot's search needs it (Y if nobody saw it)";
      return;
    }
    entry = e;
    path = e.path.mirroredFor(robot.getAimCellX(), robot.getAimCellY());
    spot = isSpot(e) ? spotSearch(e, path) : null;
    relaxed = SotmTuningSettings.GATES == 0 ? e.relaxedGates : SotmTuningSettings.GATES == 2;
    fed = false;
    feedPathS = Double.NaN;
    double[] s = path.sample(0);
    driveTo(nowS, pose, s[0], s[1]);
    stage = Stage.TO_START;
    status = "driving to the start";
  }

  /** Sets up the drive round the Hive to (x, y). */
  private void driveTo(double nowS, Pose pose, double x, double y) {
    route = FieldRoute.route(pose.xM, pose.yM, x, y);
    routeIndex = 0;
    legStartX = pose.xM;
    legStartY = pose.yM;
    progressX = pose.xM;
    progressY = pose.yM;
    progressS = nowS;
    driveStartS = nowS;
    lastFixCount = robot.getPositionFixes();
    lastFixS = nowS;
    holdPoint = null;
    stageS = nowS;
  }

  private void stop(String why) {
    if (stage != Stage.IDLE) {
      stoppedS = clock.seconds();
    }
    stage = Stage.IDLE;
    status = why;
    holdPoint = null;
  }

  private void updateAccel(double nowS) {
    double[] v = robot.getVelocity();
    velocityHistory.addLast(new double[] {nowS, v[0], v[1]});
    while (velocityHistory.size() > 2 && nowS - velocityHistory.peekFirst()[0] > ACCEL_WINDOW_S) {
      velocityHistory.removeFirst();
    }
    double[] old = velocityHistory.peekFirst();
    double span = nowS - old[0];
    accel = span > 0.02 ? new double[] {(v[0] - old[1]) / span, (v[1] - old[2]) / span} : new double[] {0, 0};
  }

  /** Sets this loop's scripted drive, aim and fire. */
  private void drive(double nowS, Pose pose) {
    double[] stick = null;
    double[] steadyReference = null;
    boolean fire = false;
    boolean aim = true; // Keeps the shot (and on the way, the cameras) on the Cell
    boolean shotBackward = false; // Hooded on the way: the cameras face the Hive
    if (robot.getPositionFixes() != lastFixCount) {
      lastFixCount = robot.getPositionFixes();
      lastFixS = nowS;
    }
    boolean toReload = stage == Stage.TO_RELOAD;
    if (stage == Stage.TO_START || stage == Stage.RUN || toReload) {
      boolean going = stage == Stage.TO_START || toReload;
      double limit = toReload ? FieldRoute.reloadLimit(SotmTuningSettings.RELOAD_X_M, SotmTuningSettings.RELOAD_Y_M)
          : FieldRoute.FIELD_LIMIT_M;
      String why = !robot.isPoseValid() ? "stopped: pose not placed by the tags"
          : going && nowS - lastFixS > FieldRoute.GOTO_BLIND_S ? "stopped: no tags seen for a while"
          : !FieldRoute.headedSafely(pose, robot.getVelocity(), limit) ? "stopped: heading for the fence or the Hive"
          : going && nowS - driveStartS > FieldRoute.GOTO_TIMEOUT_S + 10 ? "stopped: took too long"
          : null;
      if (why != null) {
        stop(toReload ? why + " (going to the reload spot)" : why);
      }
    }
    if (stage == Stage.TO_START || stage == Stage.TO_RELOAD) {
      if (Math.hypot(pose.xM - progressX, pose.yM - progressY) >= 0.05) {
        progressX = pose.xM;
        progressY = pose.yM;
        progressS = nowS;
      }
      double[] w = route.get(routeIndex);
      boolean last = routeIndex == route.size() - 1;
      double away = Math.hypot(w[0] - pose.xM, w[1] - pose.yM);
      if (last && away <= FieldRoute.GOTO_TOLERANCE_M && stage == Stage.TO_RELOAD) {
        stage = Stage.END;
        stageS = nowS;
        holdPoint = new double[] {w[0], w[1]};
        status = reloadStatus;
      } else if (last && away <= FieldRoute.GOTO_TOLERANCE_M) {
        stage = Stage.READY;
        stageS = nowS;
        holdPoint = new double[] {w[0], w[1]};
        status = "at the start, lining up";
      } else {
        if (!last && away <= FieldRoute.ROUTE_WAYPOINT_TOLERANCE_M) {
          routeIndex++;
          legStartX = pose.xM;
          legStartY = pose.yM;
          w = route.get(routeIndex);
        }
        // The last leg into the reload corner slower, so its stop stays inside the check
        double maxStick = stage == Stage.TO_RELOAD && last && away <= FieldRoute.RELOAD_SLOW_M
            ? FieldRoute.RELOAD_STICK : FieldRoute.GOTO_STICK;
        stick = FieldRoute.goToStick(pose, w[0], w[1], legStartX, legStartY,
            FieldRoute.GOTO_STUCK_BOOST_PER_S * (nowS - progressS), maxStick);
      }
    }
    if (stage == Stage.READY) {
      shotBackward = backward;
      Aim a = robot.getLastAim();
      double err = a == null || a.backward != backward ? 180
          : Math.abs(Math.toDegrees(a.headingErrorRad(pose.headingRad)));
      double t = nowS - stageS;
      if (t >= SotmTuningSettings.READY_MIN_S && (err <= SotmTuningSettings.READY_HEADING_DEG
          || t >= SotmTuningSettings.READY_MAX_S)) {
        stage = Stage.RUN;
        runS = nowS;
        pathS = 0;
        lastRunS = nowS;
        holdPoint = null;
        status = "running";
      }
    }
    if (stage == Stage.RUN) {
      shotBackward = backward;
      pathS += (nowS - lastRunS) * Math.max(0, Math.min(1, 1 - lagM(path.sample(pathS), pose)
          / SotmTuningSettings.LAG_HOLD_M));
      lastRunS = nowS;
      double t = pathS;
      double[] want = path.sample(t);
      double ex = want[0] - pose.xM;
      double ey = want[1] - pose.yM;
      trackErrM = Math.hypot(ex, ey);
      if (trackErrM > SotmTuningSettings.ABORT_TRACK_M) {
        stop(String.format(Locale.US, "stopped: %.2f m off the plan", trackErrM));
      } else if (nowS - runS > path.durationS() + SotmTuningSettings.MAX_EXTRA_S) {
        stop("stopped: fell too far behind the plan");
      } else if (t > path.durationS()) {
        stage = Stage.END;
        holdPoint = new double[] {pose.xM, pose.yM};
        status = fed ? "done: mark the ball, B for the next" : String.format(Locale.US,
            "done, NO BALL: never ready in its window (gates %s). B tries again, Y skips", relaxed ? "off" : "match");
        if (SotmTuningSettings.RELOAD_BETWEEN) {
          // To the corner, so a ball can be loaded by hand; marks and B work on the way
          reloadStatus = "at the reload spot: load a ball. " + status;
          driveTo(nowS, pose, SotmTuningSettings.RELOAD_X_M, SotmTuningSettings.RELOAD_Y_M);
          stage = Stage.TO_RELOAD;
          status = "going to the reload spot. " + status;
        }
      } else {
        stick = trackStick(want, path.sample(t + SotmTuningSettings.TRACK_LEAD_S), ex, ey);
        // Only the plan's speed: on an arc its direction turns all the time, so a field-frame stick never held still
        // and the Steady gate never let an arc fire (7 Nov, 085204)
        double[] planStick = stickFor(want[2], want[3]);
        steadyReference = new double[] {Math.hypot(planStick[0], planStick[1]), 0};
        // Held until the ball is out (or the dip window ends): released the loop the feed started, the feeder
        // stopped after one loop and the ball mostly stayed in (7 Nov, 080123: 5 of 16 passes with no ball)
        boolean feeding = pendingRow != null && pendingKey.equals(entry.key);
        fire = feeding || (!fed && t >= path.fireS && t <= path.fireS + path.windowS);
      }
    }
    robot.setDriveOverride(stick);
    // Braked while it drives itself and just after a stop, so a cut stick stops it instead of letting it roll on
    // toward the fence; rolling free while idle, so it can be pushed by hand
    robot.setDriveBrake(stage != Stage.IDLE || nowS - stoppedS < STOP_BRAKE_S);
    robot.setSteadyStickReference(stage == Stage.RUN ? steadyReference : null);
    robot.setHoldPoint(stage == Stage.READY || stage == Stage.END || stage == Stage.IDLE ? holdPoint : null);
    robot.setForceAim(aim);
    robot.setTuningLive(true);
    robot.setTuningShot(shotBackward, 3400, HoodSubsystem.ZERO_ANGLE_DEG);
    robot.setScriptedOneBall(fire);
    robot.setSteadyGateBypassed(stage == Stage.RUN && relaxed);
    robot.setCloseMovingGateBypassed(stage == Stage.RUN && SotmTuningSettings.GATES != 1
        && (relaxed || SotmTuningSettings.FIRE_CLOSE_MOVING));
    // A spot's trims from its search; the checks fire with the standing aim trim and the table's range
    robot.setHeadingTrimDeg(spot != null ? spot.headingTrimDeg() : standingTrimDeg());
    robot.setHoodAngleTrimDeg(spot != null && !spot.rpm ? spot.rangeTrim() : 0);
    robot.setRpmTrim(spot != null && spot.rpm ? spot.rangeTrim() : 0);
  }

  private double standingTrimDeg() {
    return backward ? standingBackwardTrimDeg : standingHoodedTrimDeg;
  }

  /** How far the robot is behind the plan's point {x, y, vx, vy} along its direction of travel, metres. */
  private static double lagM(double[] want, Pose pose) {
    double speed = Math.hypot(want[2], want[3]);
    return speed < SotmTuningSettings.MIN_SPEED_MPS ? 0
        : ((want[0] - pose.xM) * want[2] + (want[1] - pose.yM) * want[3]) / speed;
  }

  /** Field stick for the plan's speed a little ahead, with its lag made up and any excess speed braked off. */
  private double[] trackStick(double[] want, double[] ahead, double ex, double ey) {
    double[] v = robot.getVelocity();
    double speed = Math.hypot(want[2], want[3]);
    double dvx = SotmTuningSettings.TRACK_SPEED_GAIN * (want[2] - v[0]);
    double dvy = SotmTuningSettings.TRACK_SPEED_GAIN * (want[3] - v[1]);
    double cx = SotmTuningSettings.TRACK_GAIN_PER_S * ex;
    double cy = SotmTuningSettings.TRACK_GAIN_PER_S * ey;
    if (speed >= SotmTuningSettings.MIN_SPEED_MPS) {
      double ux = want[2] / speed;
      double uy = want[3] / speed;
      double along = ex * ux + ey * uy;
      double alongCorrection =
          (SotmTuningSettings.TRACK_ALONG_GAIN_PER_S - SotmTuningSettings.TRACK_GAIN_PER_S) * along;
      cx += alongCorrection * ux;
      cy += alongCorrection * uy;
      double excess = (v[0] - want[2]) * ux + (v[1] - want[3]) * uy; // Faster than the plan along its line
      if (excess > 0) {
        double extra = (SotmTuningSettings.TRACK_BRAKE_GAIN - SotmTuningSettings.TRACK_SPEED_GAIN) * excess;
        dvx -= extra * ux;
        dvy -= extra * uy;
      }
    }
    double c = Math.hypot(cx, cy);
    if (c > SotmTuningSettings.MAX_CORRECTION_MPS) {
      cx *= SotmTuningSettings.MAX_CORRECTION_MPS / c;
      cy *= SotmTuningSettings.MAX_CORRECTION_MPS / c;
    }
    return stickFor(ahead[2] + cx + dvx, ahead[3] + cy + dvy);
  }

  /** Field stick for a field velocity, m/s. */
  private static double[] stickFor(double vx, double vy) {
    double speed = Math.hypot(vx, vy);
    if (speed < SotmTuningSettings.MIN_SPEED_MPS) {
      return new double[] {0, 0};
    }
    double power = SotmTuningSettings.POWER_AT_REST + speed / SotmTuningSettings.SPEED_PER_POWER_MPS;
    double s = MecanumDriveSubsystem.stickForPower(Math.min(1.0, power));
    return new double[] {vx / speed * s, vy / speed * s};
  }

  // ---- The CSV

  /** The robot's state now: {@link #STATE_COLUMNS}. */
  private String state() {
    Aim aim = robot.getLastAim();
    Pose pose = robot.getPose();
    double[] v = robot.getVelocity();
    double cx = robot.getAimCellX();
    double cy = robot.getAimCellY();
    double ux = cx - pose.xM;
    double uy = cy - pose.yM;
    double d = Math.max(1e-6, Math.hypot(ux, uy));
    ux /= d;
    uy /= d;
    return String.format(Locale.US, "%.3f,%.3f,%.2f,%.3f,%.3f,%.1f,%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.0f,%.0f,%.2f,%.2f",
        pose.xM, pose.yM, Math.toDegrees(pose.headingRad), v[0], v[1], Math.toDegrees(v[2]), accel[0], accel[1],
        d - ShootOnTheMoveConstants.CLUSTER_TO_NEAR_LIP_M, v[0] * ux + v[1] * uy, -v[0] * uy + v[1] * ux,
        aim == null ? Double.NaN : Math.toDegrees(aim.headingErrorRad(pose.headingRad)),
        aim == null ? Double.NaN : Math.toDegrees(aim.shot.turningRad), robot.shooter.getTargetRpm(),
        robot.shooter.getRpm(), aim == null ? Double.NaN : aim.shot.hoodDeg,
        HoodSubsystem.ticksToAngle(robot.hood.getPosition()));
  }

  private void startPending(double nowS) {
    boolean inPass = stage == Stage.RUN && path != null;
    double t = pathS;
    if (inPass) {
      fed = true;
      feedPathS = t;
    }
    SotmPath p = inPass ? path : null;
    pendingKey = inPass ? entry.key : "";
    pendingRow = String.format(Locale.US, "%s,%s,%s,%.2f,%.0f,%s,%.2f,%s,%.2f,%.2f,%.3f",
        backward ? "BACKWARD" : "HOODED", pendingKey, p == null ? "" : p.kind.name(),
        p == null ? 0 : p.speedMps, p == null ? 0 : p.dirDeg, p == null ? "" : String.valueOf(p.fieldFrame),
        p == null ? 0 : p.aimDistM, relaxed && inPass ? "OFF" : "MATCH", p == null ? Double.NaN : p.fireS,
        p == null ? Double.NaN : p.windowS, inPass ? t : Double.NaN);
    pendingRow += String.format(Locale.US, ",%%DIP%%,%.3f,%s", inPass ? trackErrM : Double.NaN, state());
    SotmSpotSearch s = inPass ? spot : null;
    pendingSearchKey = s != null ? entry.key : "";
    pendingRangeTrim = s != null ? s.rangeTrim() : Double.NaN;
    pendingSearchColumns = String.format(Locale.US, ",%s,%s,%.2f,%.2f,%.3f,%.1f,%.3f",
        s != null ? "SEARCH_" + s.phase().name() : inPass ? "CHECK" : "STANDING",
        s == null ? "" : s.rpm ? "RPM" : "HOOD", pendingRangeTrim,
        s != null ? s.headingTrimDeg() : standingTrimDeg(),
        backward ? 0 : ShootOnTheMoveConstants.SOTM_HOOD_DEG_PER_TOWARD_MPS,
        backward ? ShootOnTheMoveConstants.SOTM_BACKWARD_RPM_PER_TOWARD_MPS
            : ShootOnTheMoveConstants.SOTM_RPM_PER_TOWARD_MPS,
        backward ? ShootOnTheMoveConstants.SOTM_BACKWARD_HEADING_DEG_PER_SIDEWAYS_MPS
            : ShootOnTheMoveConstants.SOTM_HEADING_DEG_PER_SIDEWAYS_MPS);
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
      lastShotRow = pendingRow.replace("%DIP%", String.format(Locale.US, "%.3f", nowS - pendingStartS))
          + "," + state()
          + String.format(Locale.US, ",%.3f,%.3f,%.2f,%.0f,%.3f,%.3f,%.3f", robot.getAimCellX(), robot.getAimCellY(),
              robot.getBatteryVolts(), dip, ShootOnTheMoveConstants.FEED_DELAY_S,
              ShootOnTheMoveConstants.SOTM_SIDEWAYS_GAIN, ShootOnTheMoveConstants.SOTM_TOWARD_GAIN)
          + pendingSearchColumns;
      lastShotKey = pendingKey;
      lastShotSearchKey = pendingSearchKey;
      lastShotRangeTrim = pendingRangeTrim;
      lastShotMarked = false;
      lastMark = null;
      double[] v = robot.getVelocity();
      Aim aim = robot.getLastAim();
      lastBallNote = String.format(Locale.US, "fed %s, out %.2f s later at %.2f m/s, heading %.1f deg off",
          Double.isNaN(feedPathS) || pendingKey.isEmpty() ? "standing (RB)"
              : String.format(Locale.US, "%+.2f s from the plan's time", feedPathS - path.fireS),
          nowS - pendingStartS, Math.hypot(v[0], v[1]),
          aim == null ? Double.NaN : Math.toDegrees(aim.headingErrorRad(robot.getPose().headingRad)));
      append(String.format(Locale.US, "%s,%d,SHOT,%s", session, shots, lastShotRow));
      Logger.recordOutput("SotmTuning/ShotRow", lastShotRow);
      pendingRow = null;
    } else if (nowS - pendingStartS > BALL_DIP_WINDOW_S) {
      emptyFeeds++;
      pendingRow = null;
      if (pendingKey.equals(entry == null ? null : entry.key)) {
        fed = false; // No ball left: the window may still fire one
      }
    }
  }

  private void applyMark(BallMark mark) {
    if (lastShotRow == null) {
      status = "no ball to mark yet";
      return;
    }
    String name = mark.toString();
    if (name.equals(lastMark)) {
      return;
    }
    scored += (mark.in ? 1 : 0) - ("IN".equals(lastMark) ? 1 : 0);
    lastMark = name;
    append(String.format(Locale.US, "%s,%d,%s,%s", session, shots, name, lastShotRow));
    SotmSpotSearch s = lastShotSearchKey.isEmpty() ? null : searches.get(lastShotSearchKey);
    String next = ": B for the next pass";
    if (s != null) {
      if (lastShotMarked) {
        s.remark(mark);
      } else {
        s.record(lastShotRangeTrim, mark);
      }
      if (s.done()) {
        done.add(lastShotSearchKey);
        double[] r = s.range();
        append(String.format(Locale.US, "%s,%d,FOUND,%s,%s,%s,%.2f,%.2f,%.2f,%.2f,%d,%d", session, shots,
            lastShotSearchKey.startsWith("BACKWARD") ? "BACKWARD" : "HOODED", lastShotSearchKey,
            s.rpm ? "RPM" : "HOOD", s.centre(), r[0], r[1], s.headingTrimDeg(), s.balls(), s.scored()));
        next = ": SPOT DONE, " + spotNote(s) + ". B for the next";
      } else {
        done.remove(lastShotSearchKey);
        next = ": " + spotNote(s) + ". B runs the spot again";
      }
    } else if (!lastShotKey.isEmpty()) {
      done.add(lastShotKey);
    }
    status = (lastShotMarked ? "mark changed to " : "marked ") + name
        + (stage == Stage.END || stage == Stage.IDLE || stage == Stage.TO_RELOAD ? next : "");
    lastShotMarked = true;
  }

  /** Where a spot's search is: its phase, the trims next, and the range once found. */
  private static String spotNote(SotmSpotSearch s) {
    String unit = s.rpm ? "rpm" : "deg";
    String fmt = s.rpm ? "%+.0f" : "%+.1f";
    double[] r = s.range();
    String range = Double.isNaN(r[0]) ? "none in yet"
        : String.format(Locale.US, "in " + fmt + " to " + fmt + " %s", r[0], r[1], unit);
    String phase;
    switch (s.phase()) {
      case FIND: phase = "finding a ball in"; break;
      case LONG_EDGE: phase = "finding the long edge"; break;
      case SHORT_EDGE: phase = "finding the short edge"; break;
      case CONFIRM: phase = "confirming the middle"; break;
      default: phase = "found"; break;
    }
    return String.format(Locale.US, "%s %s: %s, next " + fmt + " %s, aim %+.2f deg, %d/%d in",
        s.rpm ? "RPM" : "hood", phase, range, s.done() ? s.centre() : s.rangeTrim(), unit, s.headingTrimDeg(),
        s.scored(), s.balls());
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
      RobotLog.ee("SotmTuning", e, "Can't write %s", CSV_PATH);
    }
  }

  // ---- Telemetry and logs

  private void addTelemetry(Pose pose) {
    SotmPath.Entry next = nextEntry();
    int[] n = doneCounts();
    line("SOTM", String.format(Locale.US, "%s %s  gates: %s  spots %d/%d done, then checks %d/%d",
        robot.getAlliance(), backward ? "BACKWARD" : "FRONT hooded", gateName(), n[0], n[1], n[2], n[3]));
    line("Pass", entry == null ? "none yet" : (spot != null ? "SPOT " : "CHECK ") + entry.key + "  ["
        + stage.name().toLowerCase(Locale.US) + "]");
    if (spot != null) {
      line("Search", spotNote(spot));
    }
    line("Status", status);
    String markHint = keys.isMarking() ? "MARKING " + keys.pending() + " (let go of X to save)"
        : lastShotRow != null && !lastShotMarked ? "MARK THE LAST BALL: tap X in, hold X + D-pad for the miss, A bounced, Y not seen"
        : lastMark == null ? "" : "last ball " + lastMark;
    line("Mark", markHint);
    line("Last ball", lastBallNote.isEmpty() ? "none yet" : lastBallNote);
    line("Balls", String.format(Locale.US, "%d fired, %d in%s", shots, scored,
        emptyFeeds == 0 ? "" : String.format(Locale.US, "  [%d feeds without a ball]", emptyFeeds)));
    line("Next (B)", next == null ? "every spot and pass done"
        : (isSpot(next) ? "SPOT " : "CHECK ") + next.key + (next == entry ? " (again)" : ""));
    if (stage == Stage.RUN) {
      line("Tracking", String.format(Locale.US, "%.2f m off the plan, %.1f / %.1f s", trackErrM,
          pathTimeS(), path.durationS()));
    }
    line("Keys", "B run/stop  Start again  Y skip  X/A/D-pad mark  LT intake  RB fire  LB front/back  Back gates");
    if (fileError != null) {
      line("CSV", fileError);
    }
  }

  private double pathTimeS() {
    return pathS;
  }

  private String gateName() {
    switch (SotmTuningSettings.GATES) {
      case 1: return "match's always";
      case 2: return "none always";
      default: return "per pass";
    }
  }

  private void line(String key, String value) {
    telemetry.addData(key, value);
    robot.addPanelsLine(key, value);
  }

  private void logState() {
    Logger.recordOutput("SotmTuning/Stage", stage.name());
    Logger.recordOutput("SotmTuning/Pass", entry == null ? "" : entry.key);
    Logger.recordOutput("SotmTuning/Status", status);
    Logger.recordOutput("SotmTuning/GatesOff", relaxed && stage == Stage.RUN);
    Logger.recordOutput("SotmTuning/TrackErrM", stage == Stage.RUN ? trackErrM : Double.NaN);
    Logger.recordOutput("SotmTuning/Shots", shots);
    Logger.recordOutput("SotmTuning/Scored", scored);
    Logger.recordOutput("SotmTuning/AccelMps2", accel);
    if (stage == Stage.RUN && path != null) {
      double[] want = path.sample(pathS);
      Logger.recordOutput("SotmTuning/PlanPose", want);
      Logger.recordOutput("SotmTuning/PathS", pathS);
      Logger.recordOutput("SotmTuning/RunS", clock.seconds() - runS);
    }
  }
}
