package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;

/** Lebot Teleop's driver settings. Editable live from Panels. */
@Configurable
public final class DriverConstants {
  private DriverConstants() {}

  // TeleOp's Driver Station: false shows the driver's few lines (shot, aim, power, problems); true every subsystem's
  // state. The tuning OpModes always show it all
  public static boolean TELEMETRY_DETAIL = false;
  // Drive power at full stick while RT or RB is held. Full: the hood, wheels and heading follow the shot's rates
  // now, so the driver doesn't have to slow down to shoot
  public static double AIM_DRIVE_MAX_POWER = 1.0;
  // While aiming, the sticks drive no faster than the shot table has a shot for at this distance (toward, away and
  // sideways), with the shot checked at AIM_SPEED_LIMIT_MARGIN times the speed too. Off: any speed, and the shot waits
  public static boolean AIM_SPEED_LIMIT_ON = true;
  public static double AIM_SPEED_LIMIT_MARGIN = 1.2;
  // But never slower than this: backing out past the table's far end has no shot at any speed
  public static double AIM_SPEED_LIMIT_MIN_MPS = 0.25;
  // The limit's power: AIM_SPEED_LIMIT_START_POWER (what the wheels need to roll; log 194737 fits speed = 1.5 m/s per
  // power over 0.15) plus the limit's power by the top speeds, plus KP power per m/s the speed is off the limit and KI
  // per second of it
  public static double AIM_SPEED_LIMIT_START_POWER = 0.15;
  public static double AIM_SPEED_LIMIT_KP = 0.5;
  public static double AIM_SPEED_LIMIT_KI = 1.5;
  // Speed at full power, for the limit: Pedro's measured top speeds (65 and 48 in/s)
  public static double DRIVE_FORWARD_MPS_AT_FULL_POWER = 1.65;
  public static double DRIVE_STRAFE_MPS_AT_FULL_POWER = 1.22;

  // INIT tells the driver to swap a battery under this at rest (it rumbles once)
  public static double SWAP_BATTERY_V = 12.3;
  // Lebot Teleop: the intake only spins while LT is held, so it isn't pushing at 5-6 A all match. B's eject still
  // runs it backward. The tuning tools keep it running. False: always on
  public static boolean INTAKE_ONLY_ON_LT = true;
  public static double RPM_TRIM_STEP = 50; // Per D-pad press while aiming
  public static double AIM_START_RPM = 2500; // Held while aiming until the first valid shot. Middle of the table
  // The upstream indexer rollers' power while LT loads. At full power they pushed the front ball past the held-back
  // feeder into the spinning wheels. The last roller stays at its own gentle load power
  public static double LOAD_UPSTREAM_POWER = 0.5;
  // LT loading stops the last roller (before the feeder): at 0.3 it pushed the front ball past the held-back feeder
  // into the wheels with 3 balls in. The auto keeps it running to bring its next ball up. Shots run it as before
  public static boolean LOAD_LAST_ROLLER_COASTS = true;
  // RT or RB let go mid-feed keeps feeding until that feed ends, at most this long, so a ball halfway through the
  // feeder goes out and is counted. Let go between feeds, they stop at once
  public static double SHOT_RELEASE_HOLD_S = 0.25;
}
