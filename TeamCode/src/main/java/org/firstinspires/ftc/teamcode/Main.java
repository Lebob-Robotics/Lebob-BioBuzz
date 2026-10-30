package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.FtcLogTuning;
import org.psilynx.psikit.ftc.PsiKitIterativeOpMode;

@TeleOp(name = "test12342574839", group = "Robot")
public class Main extends PsiKitIterativeOpMode {
  /**
   * Serves the live log to AdvantageScope over the network. MUST be false at
   * competition. The log file is written either way.
   */
  public static final boolean LIVE_LOGGING = true;

  /**
   * Logs each motor's current. Adds a non-bulk hub read per motor every
   * FtcLogTuning.motorCurrentReadPeriodSec.
   */
  public static final boolean LOG_MOTOR_CURRENT = true;

  private Robot robot;

  @Override
  protected int getRlogPort() {
    return LIVE_LOGGING ? 5800 : 0;
  }

  @Override
  protected void onPsiKitConfigureLogging() {
    Logger.recordMetadata("Robot", "BioBuzz");
    Logger.recordMetadata("Mark", "3");
    Logger.recordMetadata("LiveLogging", String.valueOf(LIVE_LOGGING));
    Logger.recordMetadata("LogMotorCurrent", String.valueOf(LOG_MOTOR_CURRENT));

    // Bulk-only mode skips the Pinpoint and the non-bulk signals (motor
    // current), and without the Pinpoint in the log its pose cannot be replayed.
    FtcLogTuning.bulkOnlyLogging = false;
    FtcLogTuning.logMotorCurrent = LOG_MOTOR_CURRENT;
    // PsiKit does the one Pinpoint update per loop, before user code. Our code
    // must not call update() itself (see OdometrySubsystem).
    FtcLogTuning.pinpointLoggerCallsUpdate = true;
  }

  @Override
  protected void onPsiKitInit() {
    robot = new Robot(hardwareMap, telemetry, gamepad1);
    robot.init();
  }

  @Override
  protected void onPsiKitInitLoop() {
    if (robot != null) {
      robot.initLoop();
    }
  }

  @Override
  protected void onPsiKitLoop() {
    robot.periodic();
  }

  @Override
  protected void onPsiKitStop() {
    // robot is null if init threw (e.g. a device name missing from the config).
    if (robot != null) {
      robot.stop();
    }
  }
}
