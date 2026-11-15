package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.opmodes.LebotOpMode;
import org.firstinspires.ftc.teamcode.opmodes.Main;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.psilynx.psikit.core.Logger;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * Shot Tuning, SOTM Tuning and Special Shot Calibration: the log, the session their CSV rows carry, and a Robot in
 * tuning mode. Volley Tuning runs on {@link Main} instead.
 */
abstract class TuningOpMode extends LebotOpMode {
  /** When the OpMode was opened, unless a session from the CSV is carried on. */
  String session = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
  Robot robot;
  final ElapsedTime clock = new ElapsedTime();
  private final String logName;

  /** @param logName the OpMode's name in the log's metadata */
  TuningOpMode(String logName) {
    this.logName = logName;
  }

  @Override
  protected void onPsiKitConfigureLogging() {
    Main.configureLogging();
    Logger.recordMetadata("OpMode", logName);
    Logger.recordMetadata("TuningSession", session);
  }

  /** The Robot in tuning mode, for BLUE, driven from the far side. */
  void initTuningRobot() {
    robot = new Robot(hardwareMap, telemetry, gamepad1);
    robot.init();
    robot.setTuningMode(true);
    robot.setAlliance(Alliance.BLUE);
    robot.setDriverOnFarSide(true);
  }

  /** A telemetry line on the Driver Hub and Panels. */
  void line(String key, String value) {
    telemetry.addData(key, value);
    robot.addPanelsLine(key, value);
  }
}
