package org.firstinspires.ftc.teamcode.subsystems;

/** Cross-checks the independent REV and motor encoders on the same 1:1 pinion shaft. */
public final class HoodEncoderGuard {
  public static final double REV_COUNTS_PER_TURN = 8192;
  public static final double MOTOR_COUNTS_PER_TURN = 384.5; // 435 RPM 5203 output shaft
  // Direction reversal measured about 15 degrees of motor-side lost motion before the REV moved.
  private static final double MAX_DIFFERENCE_TURNS = 0.05; // 18 pinion degrees; primary position still uses REV
  private static final double DISAGREEMENT_TIME_S = 0.10;
  private static final double NO_MOTION_TIME_S = 0.50;

  private boolean tripped;
  private boolean initialized;
  private double mismatchSince = Double.NaN;
  private double lastMotionS;
  private int lastRevTicks;
  private int lastMotorTicks;

  /** Counts are relative to the same lower-stop zero, with matching positive directions. */
  public boolean update(double nowS, int revTicks, int motorTicks, boolean powered) {
    if (tripped) { return true; }
    if (!initialized) {
      initialized = true;
      lastMotionS = nowS;
      lastRevTicks = revTicks;
      lastMotorTicks = motorTicks;
    }
    if (!powered || Math.abs(revTicks - lastRevTicks) >= 16
        || Math.abs(motorTicks - lastMotorTicks) >= 2) {
      lastMotionS = nowS;
      lastRevTicks = revTicks;
      lastMotorTicks = motorTicks;
    }
    double difference = Math.abs(revTicks / REV_COUNTS_PER_TURN - motorTicks / MOTOR_COUNTS_PER_TURN);
    if (difference > MAX_DIFFERENCE_TURNS) {
      if (Double.isNaN(mismatchSince)) { mismatchSince = nowS; }
      if (nowS - mismatchSince >= DISAGREEMENT_TIME_S) { tripped = true; }
    } else {
      mismatchSince = Double.NaN;
    }
    if (powered && nowS - lastMotionS >= NO_MOTION_TIME_S) { tripped = true; }
    return tripped;
  }

  public boolean isTripped() { return tripped; }

  public void reset() {
    tripped = false;
    initialized = false;
    mismatchSince = Double.NaN;
  }
}
