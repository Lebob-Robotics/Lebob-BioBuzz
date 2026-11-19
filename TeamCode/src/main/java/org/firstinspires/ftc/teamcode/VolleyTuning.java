package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.utils.Aim;
import org.firstinspires.ftc.teamcode.utils.BallMark;
import org.firstinspires.ftc.teamcode.utils.MarkKeys;
import org.firstinspires.ftc.teamcode.utils.ShootOnTheMoveConstants;
import org.firstinspires.ftc.teamcode.utils.VolleyCalibration;
import org.firstinspires.ftc.teamcode.utils.VolleyCalibration.Ball;
import org.psilynx.psikit.core.Logger;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Locale;

/**
 * Volley calibration, after SOTM Tuning: Lebot Teleop Volley with a mark for every ball, to make the four-ball volley
 * score and keep its balls apart. Drive and shoot exactly as in the volley TeleOp (its SOTM trims and all); after each
 * volley mark its balls in the order they left.
 *
 * <p>The marks run two hood searches, one for the first ball of a volley and one for balls 2-4 (the first leaves
 * from wheels that have held speed, the rest from wheels just recovered), each found as Shot Tuning finds a spot's.
 * A ball that hit another, or a wheel dip deep enough to be two balls at once, lowers the feed power a step, so the
 * queue follows the ball ahead less tightly. Every mark goes to {@link #CSV_PATH}; a restart carries on from it.
 * Telemetry shows the constants to copy into ShootOnTheMoveConstants once found.
 *
 * <p>Controls (gamepad 1), besides Lebot Teleop's sticks, triggers, bumpers and Share:
 * <ul>
 *   <li>After a volley, for each ball in order: X tapped, it went in. X held, D-pad tapped, X let go: where it
 *       missed (up long, down short, left, right). A: in and bounced out. B: it hit another ball. Y: not seen.
 *       The next volley waits until every ball is marked.</li>
 *   <li>D-pad without X: up/down feed power +/-0.05, right full-power recovery on/off, left RPM boost
 *       0/100/200/300.</li>
 *   <li>A, B, X, Y and the D-pad do nothing else here: no heading reset, eject, stop, full power or trims.</li>
 *   <li>During INIT, Y switches between carrying on from the CSV and starting the searches over.</li>
 * </ul>
 */
@TeleOp(name = "Volley Tuning", group = "Tuning")
public class VolleyTuning extends Main {
  static final String CSV_PATH = "/sdcard/FIRST/VolleyTuning.csv";
  private static final double FEED_POWER_STEP = 0.05;
  private static final double[] BOOSTS_RPM = {0, 100, 200, 300};

  private final MarkKeys keys = new MarkKeys();
  private final String session = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
  private VolleyCalibration resumed;
  private VolleyCalibration cal;
  private boolean carryOn = true;
  private boolean wasInitY;
  private boolean wasB;
  private boolean wasY;
  private boolean wasUp;
  private boolean wasDown;
  private boolean wasLeft;
  private boolean wasRight;
  private boolean fastRecovery = ShootOnTheMoveConstants.VOLLEY_FAST_RECOVERY;
  private double boostRpm = ShootOnTheMoveConstants.VOLLEY_BOOST_RPM;
  private boolean inVolley;
  private double[] fedSnapshot; // At the last feed start: {rpm, target, lip, toward, sideways, hood}
  private String fileError;
  private String lastMarkText = "";

  @Override
  protected void configureRobot(Robot robot) {
    robot.setVolleyMode(true);
    robot.setButtonsReserved(true);
    resumed = new VolleyCalibration(ShootOnTheMoveConstants.VOLLEY_FEED_POWER);
    resumed.replay(readCsv());
  }

  @Override
  protected void onPsiKitInitLoop() {
    Robot robot = robot();
    if (robot != null) {
      boolean y = robot.getDriver().y;
      if (y && !wasInitY) {
        carryOn = !carryOn;
      }
      wasInitY = y;
      telemetry.addData("Volley Tuning", carryOn
          ? "CARRY ON from " + CSV_PATH + " (Y: start over)" : "START OVER: new searches (Y: carry on)");
      if (carryOn) {
        telemetry.addData("First ball", VolleyCalibration.describe(resumed.firstSearch()));
        telemetry.addData("Balls 2-4", VolleyCalibration.describe(resumed.laterSearch()));
        telemetry.addData("Feed power", String.format(Locale.US, "%.2f", resumed.feedPower()));
      }
    }
    super.onPsiKitInitLoop();
  }

  @Override
  protected void onPsiKitStart() {
    if (carryOn) {
      cal = resumed;
    } else {
      cal = new VolleyCalibration(ShootOnTheMoveConstants.VOLLEY_FEED_POWER);
      append(cal.resetRow(session));
    }
    super.onPsiKitStart();
  }

  @Override
  protected void onPsiKitLoop() {
    Robot robot = robot();
    if (cal == null) {
      cal = resumed != null ? resumed : new VolleyCalibration(ShootOnTheMoveConstants.VOLLEY_FEED_POWER);
    }
    Gamepad g = robot.getDriver(); // Last loop's merged gamepad; the robot reads this loop's in periodic()
    readMarks(g);
    readSettings(g);
    if (!inVolley) { // Fixed through a volley, so every ball of it is fired with what it is recorded with
      robot.setVolleyTrimsDeg(cal.nextFirstTrimDeg(), cal.nextLaterTrimDeg());
      robot.setVolleyFeedPower(cal.feedPower());
      robot.setVolleyBoostRpm(boostRpm);
      robot.setVolleyFastRecovery(fastRecovery);
    }
    robot.setHoldFire(cal.marksPending() && !inVolley);
    addTelemetry();
    super.onPsiKitLoop();
    track(robot);
  }

  @Override
  protected void onPsiKitStop() {
    Robot robot = robot();
    if (robot != null) {
      robot.setHoldFire(false);
      robot.setButtonsReserved(false);
    }
    super.onPsiKitStop();
  }

  /** The marks for the last volley's balls, one at a time in the order they left. */
  private void readMarks(Gamepad g) {
    BallMark mark = keys.update(g.x, g.a, g.dpad_up, g.dpad_down, g.dpad_left, g.dpad_right);
    boolean b = g.b && !wasB && !g.x;
    boolean y = g.y && !wasY && !g.x;
    wasB = g.b;
    wasY = g.y;
    // Past the balls counted, up to four: a ball that left with another in one dip
    if (inVolley || cal.volleys() == 0 || (!cal.marksPending() && cal.balls().size() >= 4)) {
      return;
    }
    String text = mark != null ? mark.toString() : b ? VolleyCalibration.HIT : y ? VolleyCalibration.UNSEEN : null;
    if (text != null) {
      Ball ball = cal.mark(text);
      lastMarkText = String.format(Locale.US, "volley %d ball %d: %s", ball.volley, ball.index + 1, text);
      write(ball);
    }
  }

  /** D-pad without X: the volley's feed power, recovery and boost. */
  private void readSettings(Gamepad g) {
    boolean free = !g.x && !keys.isMarking();
    if (free && g.dpad_up && !wasUp) {
      cal.setFeedPower(cal.feedPower() + FEED_POWER_STEP);
    }
    if (free && g.dpad_down && !wasDown) {
      cal.setFeedPower(cal.feedPower() - FEED_POWER_STEP);
    }
    if (free && g.dpad_right && !wasRight) {
      fastRecovery = !fastRecovery;
    }
    if (free && g.dpad_left && !wasLeft) {
      int i = 0;
      while (i < BOOSTS_RPM.length && BOOSTS_RPM[i] <= boostRpm) {
        i++;
      }
      boostRpm = i < BOOSTS_RPM.length ? BOOSTS_RPM[i] : BOOSTS_RPM[0];
    }
    wasUp = g.dpad_up;
    wasDown = g.dpad_down;
    wasLeft = g.dpad_left;
    wasRight = g.dpad_right;
  }

  /**
   * After the robot's loop: each feed start's state, kept for the ball it turns out to push, and each ball the wheels
   * showed leaving. A feed that pushes nothing (the robot empty, RT still held) shows no dip, so counts no ball.
   */
  private void track(Robot robot) {
    if (robot.isVolleyShot() && robot.feedStartedThisLoop()) {
      Aim aim = robot.getLastAim();
      fedSnapshot = new double[] {robot.shooter.getRpm(), robot.shooter.getTargetRpm(),
          aim != null ? aim.shot.distanceToLipM : Double.NaN, aim != null ? aim.shot.speedTowardCellMps : Double.NaN,
          aim != null ? aim.shot.sidewaysSpeedMps : Double.NaN, HoodSubsystem.ticksToAngle(robot.hood.getTarget())};
    }
    // The volley is the press: from its first feed, so the trims are fixed and no mark lands while a ball is in
    // the feeder, until the trigger is let go (the gate's count goes back to 0)
    int feedStarts = robot.getVolleyFeedStarts();
    if (feedStarts > 0 && !inVolley) {
      for (Ball unseen : cal.startVolley()) {
        write(unseen);
      }
      inVolley = true;
    }
    int ballsLeft = robot.getVolleyBallsLeft();
    if (inVolley) {
      while (cal.balls().size() < ballsLeft) {
        Ball ball = cal.fed();
        if (fedSnapshot != null) {
          ball.fedRpm = fedSnapshot[0];
          ball.targetRpm = fedSnapshot[1];
          ball.lipM = fedSnapshot[2];
          ball.towardMps = fedSnapshot[3];
          ball.sidewaysMps = fedSnapshot[4];
          ball.hoodDeg = fedSnapshot[5];
        }
      }
      if (ballsLeft > 0) {
        cal.dip(ballsLeft - 1, robot.getVolleyLeftAtRpm(), robot.getVolleyDipDepthRpm(),
            ShootOnTheMoveConstants.VOLLEY_DOUBLE_DIP_RPM);
      }
    }
    if (feedStarts == 0 && inVolley) {
      inVolley = false; // Trigger let go: now the marks (none if no ball left)
    }
  }

  private void addTelemetry() {
    if (inVolley) {
      telemetry.addData("VOLLEY", String.format(Locale.US, "%d  firing: %d out", cal.volleys(), cal.balls().size()));
    } else if (cal.marksPending()) {
      telemetry.addData("MARK", String.format(Locale.US, "volley %d ball %d of %d: %s  (X in, X+D-pad miss, A bounce,"
          + " B hit a ball, Y not seen)", cal.volleys(), cal.marked() + 1, cal.balls().size(),
          keys.isMarking() ? "saving " + keys.pending() : "?"));
    } else {
      telemetry.addData("READY", "RT fires a volley. D-pad: up/down feed power, right recovery, left boost"
          + (cal.volleys() > 0 && cal.balls().size() < 4 ? ". A ball not counted? Mark it now" : ""));
    }
    if (!lastMarkText.isEmpty()) {
      telemetry.addData("Last mark", lastMarkText);
    }
    for (Ball ball : cal.balls()) {
      telemetry.addData("Ball " + (ball.index + 1), String.format(Locale.US,
          "fed %+.0f  left %+.0f RPM  dip %.0f%s  trim %+.1f  %s", ball.fedRpm - ball.targetRpm,
          ball.leftRpm - ball.targetRpm, ball.dipRpm,
          ball.doubleBall(ShootOnTheMoveConstants.VOLLEY_DOUBLE_DIP_RPM) ? " TWO BALLS?" : "", ball.trimDeg,
          ball.mark == null ? "" : ball.mark));
    }
    telemetry.addData("First ball", VolleyCalibration.describe(cal.firstSearch()));
    telemetry.addData("Balls 2-4", VolleyCalibration.describe(cal.laterSearch()));
    telemetry.addData("Feed", String.format(Locale.US, "power %.2f  recovery %s  boost %+.0f RPM  (hits %d, double"
        + " dips %d)", cal.feedPower(), fastRecovery ? "FULL POWER" : "PID", boostRpm, cal.hits(), cal.doubles()));
    telemetry.addData("Paste", String.format(Locale.US, "VOLLEY_FIRST_BALL_TRIM_DEG = %s; VOLLEY_LATER_BALL_TRIM_DEG"
        + " = %s; VOLLEY_FEED_POWER = %.2f; VOLLEY_BOOST_RPM = %.0f; VOLLEY_FAST_RECOVERY = %b",
        found(cal.firstSearch()), found(cal.laterSearch()), cal.feedPower(), boostRpm, fastRecovery));
    if (fileError != null) {
      telemetry.addData("CSV", fileError);
    }
  }

  private static String found(org.firstinspires.ftc.teamcode.utils.ShotSearch search) {
    return search.hasRange() ? String.format(Locale.US, "%.1f", search.centre()) : "(not found yet)";
  }

  private void write(Ball ball) {
    append(cal.row(session, ball, boostRpm, fastRecovery, ShootOnTheMoveConstants.VOLLEY_DOUBLE_DIP_RPM,
        robot().getBatteryVolts()));
  }

  private List<String> readCsv() {
    List<String> lines = new ArrayList<>();
    File file = new File(CSV_PATH);
    if (!file.exists()) {
      return lines;
    }
    try (BufferedReader in = new BufferedReader(new FileReader(file))) {
      for (String line = in.readLine(); line != null; line = in.readLine()) {
        lines.add(line);
      }
    } catch (IOException e) {
      fileError = "not read: " + e.getMessage();
      RobotLog.ee("VolleyTuning", e, "Can't read %s", CSV_PATH);
    }
    return lines;
  }

  private void append(String row) {
    if (Logger.isReplay()) {
      return;
    }
    File file = new File(CSV_PATH);
    boolean isNew = !file.exists();
    try (FileWriter out = new FileWriter(file, true)) {
      if (isNew) {
        out.write(VolleyCalibration.CSV_HEADER + "\n");
      }
      out.write(row + "\n");
      fileError = null;
    } catch (IOException e) {
      fileError = "not saved: " + e.getMessage();
      RobotLog.ee("VolleyTuning", e, "Can't write %s", CSV_PATH);
    }
  }
}
