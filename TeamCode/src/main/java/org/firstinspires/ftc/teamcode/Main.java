package org.firstinspires.ftc.teamcode;

import com.bylazar.panels.Panels;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.FtcLogTuning;
import org.psilynx.psikit.ftc.PsiKitIterativeOpMode;

@TeleOp(name = "BioBuzz TeleOp", group = "Robot")
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

  /**
   * Allows Panels (live tuning web server). MUST be false at events: R704 D
   * bans it in matches. False stops the Panels server and socket when this
   * OpMode inits, and Panels remembers that across OpModes. Panels then stays
   * off until the "Enable/Disable Panels" OpMode is run (or this is true and
   * Panels was never disabled). To remove it for good, delete the
   * com.bylazar:fullpanels dependency and the @Configurable imports in
   * ShootOnTheMoveConstants, ShooterSubsystem, HoodSubsystem and
   * MecanumDriveSubsystem.
   */
  public static final boolean PANELS_ENABLED = true;

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
    Logger.recordMetadata("PanelsEnabled", String.valueOf(PANELS_ENABLED));

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
    if (!PANELS_ENABLED) {
      Panels.INSTANCE.disable();
    }
    robot = new Robot(hardwareMap, telemetry, gamepad1);
    robot.init();
  }

  @Override
  protected void onPsiKitInitLoop() {
    robot.initLoop();
  }

  @Override
  protected void onPsiKitStart() {
    robot.start();
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
