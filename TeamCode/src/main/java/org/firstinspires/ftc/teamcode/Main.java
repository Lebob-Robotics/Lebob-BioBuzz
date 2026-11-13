package org.firstinspires.ftc.teamcode;

import com.bylazar.panels.Panels;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utils.ShootOnTheMoveConstants;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.FtcLogTuning;
import org.psilynx.psikit.ftc.PsiKitIterativeOpMode;
import org.psilynx.psikit.ftc.wrappers.MotorWrapper;

@TeleOp(name = "Lebot Teleop", group = "Robot")
public class Main extends PsiKitIterativeOpMode {
  /**
   * Serves the live log to AdvantageScope over the network. MUST be false at
   * competition. The log file is written either way.
   */
  public static final boolean LIVE_LOGGING = true;

  /**
   * Port of the live log server. Not 5800, the PsiKit default: the Panels Limelight proxy takes
   * 5800, 5801, 5805 and 5807 on the hub, so PsiKit could not bind it. Set the same port in
   * AdvantageScope (Preferences, RLOG port).
   */
  public static final int RLOG_PORT = 5810;

  /**
   * Logs each motor's current. Adds a non-bulk hub read per motor every
   * FtcLogTuning.motorCurrentReadPeriodSec: all 8 motors in one loop, about 20 ms. Off for loop speed
   * (4 Nov). The shooter's stall guard still reads its own two.
   */
  public static final boolean LOG_MOTOR_CURRENT = false;

  /**
   * How often PsiKit re-reads the values the bulk read doesn't cover: each motor's power and target
   * position, and each servo's settings. Each is its own hub call, about 2-3 ms, so reading them every
   * loop made the motor logging 62 ms of a 138 ms loop (4 Nov). Our code doesn't read them back, so they
   * are only for the log.
   */
  public static final double NON_BULK_READ_PERIOD_S = 2.0;

  /**
   * How often PsiKit re-reads each motor's settings (direction, mode, zero power behaviour), each its own
   * hub call. Its default of 0.25 s gave 20 ms spikes in the motor logging. Our code sets them once.
   */
  public static final double MOTOR_CONFIG_READ_PERIOD_S = 5.0;

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
  private boolean wasBack;
  private VolleySettings volley; // Null in a variant that sets its own (Volley Tuning)
  private boolean wasShooting;
  private int volleySpot = -1;

  @Override
  protected int getRlogPort() {
    return LIVE_LOGGING ? RLOG_PORT : 0;
  }

  @Override
  protected void onPsiKitConfigureLogging() {
    configureLogging();
  }

  /** Shared with OpModes that run the whole Robot, so their logs replay the same way. */
  public static void configureLogging() {
    Logger.recordMetadata("Robot", "BioBuzz");
    Logger.recordMetadata("Mark", "3");
    Logger.recordMetadata("LiveLogging", String.valueOf(LIVE_LOGGING));
    Logger.recordMetadata("LogMotorCurrent", String.valueOf(LOG_MOTOR_CURRENT));
    Logger.recordMetadata("PanelsEnabled", String.valueOf(PANELS_ENABLED));

    // Bulk-only mode skips the Pinpoint and the non-bulk signals (motor
    // current), and without the Pinpoint in the log its pose cannot be replayed.
    FtcLogTuning.bulkOnlyLogging = false;
    FtcLogTuning.logMotorCurrent = LOG_MOTOR_CURRENT;
    FtcLogTuning.nonBulkReadPeriodSec = NON_BULK_READ_PERIOD_S;
    MotorWrapper.Companion.setConfigRefreshPeriodSec(MOTOR_CONFIG_READ_PERIOD_S);
    MotorWrapper.Companion.setMetadataRefreshPeriodSec(10.0);
    // Our code doesn't use them, and the hood and shooter log their own state
    MotorWrapper.Companion.setLogOverCurrent(false);
    MotorWrapper.Companion.setLogBusy(false);
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
    // RB is a standing volley, RT the one-at-a-time shoot on the move
    robot.setVolleyMode(true);
    if (usesTunedVolley()) {
      volley = VolleySettings.load(VolleyTuning.CSV_PATH);
    }
    configureRobot(robot);
  }

  /** For a variant of this TeleOp (Volley Tuning): changes the robot once it is made. Nothing here. */
  protected void configureRobot(Robot robot) {
  }

  /**
   * False for a variant that sets the volley's settings itself; true reads Volley Tuning's results. Off while
   * VOLLEY_USE_TUNING_CSV is, so RB runs the ShootOnTheMoveConstants volley.
   */
  protected boolean usesTunedVolley() {
    return ShootOnTheMoveConstants.VOLLEY_USE_TUNING_CSV;
  }

  /** The robot, for a variant of this TeleOp; null before INIT or if it threw. */
  protected Robot robot() {
    return robot;
  }

  @Override
  protected void onPsiKitInitLoop() {
    if (robot != null) {
      pollDriverSide();
      telemetry.addData("Driving from", robot.isDriverOnFarSide() ? "FAR side of the field (Share to change)"
          : "alliance wall (Share to change)");
      robot.initLoop();
    }
  }

  /** Share turns the sticks round for a driver at the far wall, as when practising from the other side. */
  private void pollDriverSide() {
    boolean back = robot.getDriver().back;
    if (back && !wasBack) {
      robot.setDriverOnFarSide(!robot.isDriverOnFarSide());
    }
    wasBack = back;
  }

  @Override
  protected void onPsiKitStart() {
    if (robot != null) {
      robot.start();
    }
  }

  @Override
  protected void onPsiKitLoop() {
    robot.addPowerTelemetry(); // First on the Driver Station, for the co-driver
    pollDriverSide();
    if (robot.isDriverOnFarSide()) {
      telemetry.addData("Driving from", "FAR side (Share to change)");
    }
    if (volley != null) {
      applyVolleySettings();
    } else if (usesTunedVolley() || getClass() == Main.class) {
      telemetry.addData(ShootOnTheMoveConstants.VOLLEY_ON_RT ? "Volley (RB still, RT moving)" : "Volley (RB)",
          "constants (full feed, matched speed each ball)");
    }
    robot.periodic();
    Gamepad g = robot.getDriver();
    wasShooting = g.right_trigger > Robot.TRIGGER_PRESSED || g.right_bumper;
  }

  /** Picks the volley's settings for where the robot is, between shots only, so a volley never changes mid-burst. */
  private void applyVolleySettings() {
    if (!wasShooting) {
      boolean placed = robot.isPoseValid() && robot.getLastAim() != null;
      volleySpot = placed ? volley.spotFor(robot.getPose(), robot.getAimCellX(), robot.getAimCellY()) : -1;
      VolleySettings.apply(robot, volley.get(volleySpot));
    }
    String source;
    if (volleySpot < 0) {
      source = "constants (no tuned spot, or pose not placed)";
    } else {
      boolean near = volley.isNear(robot.getPose(), robot.getAimCellX(), robot.getAimCellY(), volleySpot);
      source = (volley.isConfirmed(volleySpot) ? "confirmed" : "best so far") + " at spot " + (volleySpot + 1)
          + (near ? "" : ", guessed for here");
    }
    telemetry.addData("Volley (RB)", source);
  }

  @Override
  protected void onPsiKitStop() {
    // robot is null if init threw (e.g. a device name missing from the config).
    if (robot != null) {
      robot.stop();
    }
  }
}
