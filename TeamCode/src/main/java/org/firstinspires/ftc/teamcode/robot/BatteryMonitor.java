package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.psilynx.psikit.core.Logger;

/** Samples the battery voltage for the log, telemetry and the hood's power, and warns of a low battery in INIT. */
final class BatteryMonitor {
  /** Battery voltage is a separate hub read, so sample it this often. */
  private static final double BATTERY_SAMPLE_PERIOD_S = 0.1;

  private final Robot robot;
  /** Null if the config has no voltage sensor. */
  private final VoltageSensor battery;
  private double lastBatterySampleS = Double.NEGATIVE_INFINITY;
  double batteryVolts = Double.NaN;
  double batteryMinVolts = Double.NaN;
  private double initMaxVolts = Double.NaN; // Highest in INIT: at rest, before the mechanisms load it
  private boolean swapBatteryRumbled = false;

  BatteryMonitor(Robot robot, VoltageSensor battery) {
    this.robot = robot;
    this.battery = battery;
  }

  /** In INIT: tells the driver to swap a battery that is low at rest, with one rumble. */
  void warnIfLowAtRest() {
    if (!Double.isNaN(batteryVolts)) {
      initMaxVolts = Double.isNaN(initMaxVolts) ? batteryVolts : Math.max(initMaxVolts, batteryVolts);
    }
    if (swapBattery(initMaxVolts)) {
      robot.telemetry.addLine(String.format("!! SWAP BATTERY: %.1f V at rest, want %.1f or more",
          initMaxVolts, DriverConstants.SWAP_BATTERY_V));
      if (!swapBatteryRumbled) {
        robot.physicalDriver.rumble(600);
        swapBatteryRumbled = true;
      }
    }
  }

  /** A battery this low at rest sags toward brownout in a match. */
  static boolean swapBattery(double restingVolts) {
    return restingVolts < DriverConstants.SWAP_BATTERY_V;
  }

  /**
   * Samples the battery voltage, keeping the lowest seen this run (it sags when the shooter spins up),
   * then logs it and adds it to telemetry.
   */
  void update(boolean telemetryDue) {
    double now = robot.clock.seconds();
    if (battery != null && now - lastBatterySampleS >= BATTERY_SAMPLE_PERIOD_S) {
      lastBatterySampleS = now;
      batteryVolts = battery.getVoltage();
      robot.hood.setBatteryVolts(batteryVolts);
      batteryMinVolts = Double.isNaN(batteryMinVolts)
          ? batteryVolts : Math.min(batteryMinVolts, batteryVolts);
    }
    Logger.recordOutput("Robot/BatteryVolts", batteryVolts);
    Logger.recordOutput("Robot/BatteryMinVolts", batteryMinVolts);
    if (telemetryDue && robot.isTelemetryDetail()) {
      robot.telemetry.addData("Battery", "%.2f V (lowest %.2f V)", batteryVolts, batteryMinVolts);
    }
  }
}
