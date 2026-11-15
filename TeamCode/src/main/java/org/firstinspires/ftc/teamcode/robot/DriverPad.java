package org.firstinspires.ftc.teamcode.robot;

/**
 * The driver's buttons that act once per press (the RPM trim, the ramp, rearming the intake), and the rumbles that
 * tell the driver what they would otherwise have to read: a stalled shooter or a tripped intake.
 */
final class DriverPad {
  /** Rumble when the shooter stall guard trips: drivers do not watch telemetry. */
  private static final int STALL_RUMBLE_MS = 1000;
  static final int INTAKE_LOCK_RUMBLE_MS = 1000;
  static final int INTAKE_LOCK_REMIND_MS = 400;
  static final double INTAKE_LOCK_REMIND_S = 2.0;

  private final Robot robot;
  private boolean wasDpadUp = false;
  private boolean wasDpadDown = false;
  private boolean wasRampButton = false;
  private boolean wasLeftBumper = false;
  private boolean shooterWasStalled = false;
  private double intakeLockRumbleS = Double.NaN; // Last rumble for a tripped intake; NaN while it runs

  DriverPad(Robot robot) {
    this.robot = robot;
  }

  /** While aiming, D-pad up and down nudge the RPM, for when the table is a little off on the day. */
  void updateTrim(boolean aiming) {
    if (!robot.buttonsFree()) {
      return;
    }
    if (aiming && justPressed(robot.driver.dpad_up, wasDpadUp)) {
      robot.rpmTrim += DriverConstants.RPM_TRIM_STEP;
    }
    if (aiming && justPressed(robot.driver.dpad_down, wasDpadDown)) {
      robot.rpmTrim -= DriverConstants.RPM_TRIM_STEP;
    }
    wasDpadUp = robot.driver.dpad_up;
    wasDpadDown = robot.driver.dpad_down;
  }

  /**
   * Left bumper deliberately rearms a cooled intake overload trip. D-pad up toggles the Pollen ramp when not aiming
   * (aiming, it trims the RPM).
   */
  void updateToggles(boolean aiming) {
    if (robot.tuningMode) {
      return;
    }
    if (justPressed(robot.driver.left_bumper, wasLeftBumper)) {
      robot.intake.resetOverload();
    }
    wasLeftBumper = robot.driver.left_bumper;
    if (rampToggled(robot.driver.dpad_up, wasRampButton, aiming, robot.buttonsFree())) {
      robot.ramp.toggle();
    }
    wasRampButton = robot.driver.dpad_up;
  }

  /** D-pad up just pressed, not aiming (where it trims the RPM) and with the buttons not taken by a tuning tool. */
  static boolean rampToggled(boolean pressed, boolean wasPressed, boolean aiming, boolean buttonsFree) {
    return buttonsFree && !aiming && justPressed(pressed, wasPressed);
  }

  /**
   * How long to rumble this loop for a tripped intake, which stays off until LB: long when it trips, then a short
   * reminder every {@link #INTAKE_LOCK_REMIND_S} while it is still off. 0 for no rumble.
   */
  static int intakeLockRumbleMs(boolean tripped, double nowS, double lastRumbleS) {
    if (!tripped) {
      return 0;
    }
    if (Double.isNaN(lastRumbleS)) {
      return INTAKE_LOCK_RUMBLE_MS;
    }
    return nowS - lastRumbleS >= INTAKE_LOCK_REMIND_S ? INTAKE_LOCK_REMIND_MS : 0;
  }

  private static boolean justPressed(boolean pressedNow, boolean pressedLastLoop) {
    return pressedNow && !pressedLastLoop;
  }

  /** Rumbles once when the shooter's stall guard trips, and for a tripped intake until LB rearms it. */
  void updateRumbles() {
    if (robot.shooter.isStalled() && !shooterWasStalled) {
      robot.physicalDriver.rumble(STALL_RUMBLE_MS);
    }
    shooterWasStalled = robot.shooter.isStalled();
    int lockRumbleMs = intakeLockRumbleMs(robot.intake.isOverloaded(), robot.clock.seconds(), intakeLockRumbleS);
    if (lockRumbleMs > 0) {
      robot.physicalDriver.rumble(lockRumbleMs);
      intakeLockRumbleS = robot.clock.seconds();
    } else if (!robot.intake.isOverloaded()) {
      intakeLockRumbleS = Double.NaN;
    }
  }
}
