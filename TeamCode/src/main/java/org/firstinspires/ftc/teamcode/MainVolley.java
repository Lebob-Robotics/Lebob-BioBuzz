package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.utils.Pose;
import org.firstinspires.ftc.teamcode.utils.VolleyAutoTuner;
import org.firstinspires.ftc.teamcode.utils.VolleyAutoTuner.Settings;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/** Volley TeleOp automatically loads only confirmed four-ball calibration near the tested field spots. */
@TeleOp(name = "Lebot Teleop Volley", group = "Robot")
public class MainVolley extends Main {
  private static final double MAX_CALIBRATION_DISTANCE_M = 0.5;
  private final Settings[] saved = new Settings[FieldRoute.GRID.length];
  private boolean wasShooting;
  private int applied = -1;

  @Override protected void configureRobot(Robot robot) {
    robot.setVolleyMode(true);
    File file = new File(VolleyTuning.CSV_PATH);
    if (!file.exists()) { return; }
    List<String> lines = new ArrayList<>();
    try (BufferedReader in = new BufferedReader(new FileReader(file))) {
      for (String line = in.readLine(); line != null; line = in.readLine()) { lines.add(line); }
      for (int i = 0; i < saved.length; i++) {
        VolleyAutoTuner tuner = new VolleyAutoTuner();
        tuner.replay(VolleyTuning.rowsForSpot(lines, i));
        if (tuner.done()) { saved[i] = tuner.current(); }
      }
    } catch (IOException e) {
      RobotLog.ee("Volley", e, "Can't load calibration; using default settings");
    }
  }

  @Override protected void onPsiKitLoop() {
    Robot robot = robot();
    if (!wasShooting) {
      boolean[] known = new boolean[saved.length];
      for (int i = 0; i < saved.length; i++) { known[i] = saved[i] != null; }
      applied = robot.isPoseValid() && robot.getLastAim() != null
          ? nearestSpot(robot.getPose(), robot.getAimCellX(), robot.getAimCellY(), known) : -1;
      Settings s = applied >= 0 ? saved[applied] : null;
      robot.setVolleyTrimsDeg(s == null ? Double.NaN : s.firstTrim, s == null ? Double.NaN : s.laterTrim);
      robot.setVolleyHeadingTrimDeg(s == null ? Double.NaN : s.headingTrim);
      robot.setVolleyFeedPower(s == null ? Double.NaN : s.feeder);
      robot.setVolleyIndexerPower(s == null ? Double.NaN : s.feeder * s.indexerScale);
      robot.setVolleyBoostRpm(s == null ? Double.NaN : s.boostRpm);
      robot.setVolleyFastRecovery(s == null ? null : s.fastRecovery);
    }
    telemetry.addData("Volley calibration", applied < 0 ? "Default (no confirmed nearby spot)"
        : "Confirmed four-ball settings from spot " + (applied + 1));
    super.onPsiKitLoop();
    Gamepad g = robot.getDriver();
    wasShooting = g.right_trigger > Robot.TRIGGER_PRESSED || g.right_bumper;
  }

  static int nearestSpot(Pose pose, double cellX, double cellY, boolean[] known) {
    int best = -1;
    double distance = MAX_CALIBRATION_DISTANCE_M;
    for (int i = 0; i < known.length; i++) {
      if (!known[i]) { continue; }
      double[] p = VolleySpotDrive.point(i, cellX, cellY);
      double d = Math.hypot(p[0] - pose.xM, p[1] - pose.yM);
      if (d <= distance) { best = i; distance = d; }
    }
    return best;
  }
}
