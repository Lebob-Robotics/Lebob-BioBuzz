package org.firstinspires.ftc.teamcode.subsystems;

/**
 * Cross-checks the independent REV and motor encoders on the same 1:1 pinion shaft.
 *
 * <p>The belt between them can skip a tooth (TeleOp 4 Nov: the motor reached full lift, the REV stopped
 * 9 degrees short). The REV is on the pinion, so it is the hood's true position: a disagreement asks the
 * caller to re-zero the motor to the REV ({@link #takeRebase()}) instead of cutting power. Only one that
 * keeps coming back (MAX_REBASES within REBASE_WINDOW_S: an unplugged, reversed or wrongly scaled
 * encoder) trips the guard, as does power with no motion.
 */
public final class HoodEncoderGuard {
  public static final double REV_COUNTS_PER_TURN = 8192;
  public static final double MOTOR_COUNTS_PER_TURN = 384.5; // 435 RPM 5203 output shaft
  // Measured at rest: about 16 motor ticks (15 pinion degrees) of belt lost motion on reversal, plus a
  // few ticks of offset at the stop. Mid-move the two hubs are read at different instants, which adds
  // several more ticks at speed, so small disagreements are only judged at rest.
  private static final double MAX_DIFFERENCE_TURNS = 0.08; // 31 motor ticks; primary position still uses REV
  private static final double GROSS_DIFFERENCE_TURNS = 0.15; // Judged even mid-move: no skew is this large
  private static final double DISAGREEMENT_TIME_S = 0.10;
  private static final double NO_MOTION_TIME_S = 0.50;
  private static final int MAX_REBASES = 3;
  private static final double REBASE_WINDOW_S = 3.0;
  private final double[] rebaseTimes = {Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY};
  private int rebaseSlot;
  private boolean rebasePending;
  private int slipCount;

  private boolean tripped;
  private boolean initialized;
  private double mismatchSince = Double.NaN;
  private double lastMotionS;
  private int lastRevTicks;
  private int lastMotorTicks;

  /** Counts are relative to the same lower-stop zero, with matching positive directions. */
  public boolean update(double nowS, int revTicks, int motorTicks, boolean powered) {
    return update(nowS, revTicks, motorTicks, powered, true);
  }

  /** As above; atRest says the motor is stationary, so the two hubs' readings are comparable. */
  public boolean update(double nowS, int revTicks, int motorTicks, boolean powered, boolean atRest) {
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
    if (difference > MAX_DIFFERENCE_TURNS && (atRest || difference > GROSS_DIFFERENCE_TURNS)) {
      if (Double.isNaN(mismatchSince)) { mismatchSince = nowS; }
      if (nowS - mismatchSince >= DISAGREEMENT_TIME_S) {
        if (nowS - rebaseTimes[rebaseSlot] > REBASE_WINDOW_S) { // Oldest of the last MAX_REBASES
          rebaseTimes[rebaseSlot] = nowS;
          rebaseSlot = (rebaseSlot + 1) % MAX_REBASES;
          rebasePending = true;
          slipCount++;
          mismatchSince = Double.NaN;
        } else {
          tripped = true;
        }
      }
    } else if (difference <= MAX_DIFFERENCE_TURNS) {
      mismatchSince = Double.NaN;
    } // A moderate disagreement mid-move neither confirms nor clears a mismatch

    if (powered && nowS - lastMotionS >= NO_MOTION_TIME_S) { tripped = true; }
    return tripped;
  }

  public boolean isTripped() { return tripped; }
  /** True once after a disagreement: the caller re-zeroes the motor count to match the REV. */
  public boolean takeRebase() {
    boolean pending = rebasePending;
    rebasePending = false;
    return pending;
  }
  /** Belt slips re-zeroed since the last reset. */
  public int getSlipCount() { return slipCount; }

  public void reset() {
    tripped = false;
    initialized = false;
    mismatchSince = Double.NaN;
    java.util.Arrays.fill(rebaseTimes, Double.NEGATIVE_INFINITY);
    rebaseSlot = 0;
    rebasePending = false;
    slipCount = 0;
  }
}
