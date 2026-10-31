package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.GobildaMotor;
import org.psilynx.psikit.core.Logger;

/**
 * Subsystem to control the shooter hood.
 *
 * <p>The hood is driven by a goBILDA 435 RPM Yellow Jacket. A REV Through Bore
 * encoder (8192 ticks/rev) sits on the motor-side pinion of the 10:1 hood
 * reduction and is plugged into the
 * HoodMotor's encoder port, so {@link GobildaMotor#getPosition()} reads the
 * hood directly.
 *
 * <p>We run our own P loop in {@link #periodic()} instead of RUN_TO_POSITION.
 * The built-in position PID is tuned for the motor's own encoder, not an
 * 8192 CPR one, so it would behave badly with this feedback.
 *
 * <p>The gains and geometry are live in Panels. Ticks are always derived from the current geometry,
 * so editing it takes effect immediately.
 */
@Configurable
public class HoodSubsystem extends SubsystemBase {
  /** Proportional gain, in motor power per encoder tick of error. */
  public static double KP = 0.002; // TODO tune on robot
  /** Largest power the P loop may command. */
  public static double MAX_POWER = 0.6;
  /** Errors smaller than this (ticks) are treated as on target. */
  public static int DEADBAND_TICKS = 30;
  /** Soft limit at the lower hard stop, where the encoder is zeroed. */
  private static final int MIN_TICKS = 0;

  // ---- Geometry --------------------------------------------------------
  /** Launch angle at the lower hard stop, where the encoder is zeroed. */
  public static double ZERO_ANGLE_DEG = 99.0;
  /** Launch angle at the top of the hood's travel; sets the upper soft limit. */
  public static double MIN_ANGLE_DEG = 51.0; // TODO tune on robot
  /** Encoder ticks per degree of launch angle: 8192 ticks/rev through the 10:1. */
  public static double TICKS_PER_DEG = 8192.0 * 10.0 / 360.0;

  private final GobildaMotor hoodMotor;

  /** Hood positions, each given as a launch angle and held in Through Bore ticks. */
  public enum HoodPreset {
    /** The lower hard stop, wherever ZERO_ANGLE_DEG puts it. */
    STOW(Double.NaN),
    CLOSE(75.0), // TODO tune on robot
    MID(65.0), // TODO tune on robot
    FAR(55.0); // TODO tune on robot

    private final double launchAngleDeg;

    HoodPreset(double launchAngleDeg) {
      this.launchAngleDeg = launchAngleDeg;
    }

    public int getTicks() {
      return Double.isNaN(launchAngleDeg) ? MIN_TICKS : angleToTicks(launchAngleDeg);
    }
  }

  private HoodPreset preset = HoodPreset.STOW;
  // When on, directTicks overrides the preset (used for continuous aiming).
  private boolean directMode = false;
  private int directTicks = 0;
  // Open-loop power that overrides the P loop. NaN = follow the preset.
  private double manualPower = Double.NaN;

  // What the last P loop step saw, kept for logging.
  private int lastPosition;
  private int lastError;
  private boolean lastAtTarget;
  private boolean lastAtSoftLimit;

  public HoodSubsystem(HardwareMap hardwareMap) {
    // reversed=false is a guess: verify on robot that positive power raises the hood.
    hoodMotor = new GobildaMotor(hardwareMap, "HoodMotor", false, DcMotor.RunMode.RUN_WITHOUT_ENCODER, true);
  }

  /**
   * Called once from the OpMode's init(). Zeroes the encoder, so the hood must
   * be resting at its lower hard stop when this runs.
   */
  public void init() {
    hoodMotor.resetEncoder();
  }

  @Override
  public void periodic() {
    if (Double.isNaN(manualPower)) {
      runPositionLoop();
    } else {
      // No soft limits: they need the encoder, which manual mode does not trust.
      lastPosition = hoodMotor.getPosition();
      hoodMotor.setPower(manualPower);
    }

    Logger.recordOutput("Hood/Preset", preset);
    Logger.recordOutput("Hood/DirectMode", directMode);
    Logger.recordOutput("Hood/ManualPower", manualPower);
    Logger.recordOutput("Hood/TargetTicks", getTarget());
    Logger.recordOutput("Hood/PositionTicks", lastPosition);
    Logger.recordOutput("Hood/TargetAngleDeg", ticksToAngle(getTarget()));
    Logger.recordOutput("Hood/AngleDeg", ticksToAngle(lastPosition));
    Logger.recordOutput("Hood/ErrorTicks", lastError);
    Logger.recordOutput("Hood/CommandedPower", hoodMotor.getCommandedPower());
    Logger.recordOutput("Hood/AtTarget", lastAtTarget);
    Logger.recordOutput("Hood/AtSoftLimit", lastAtSoftLimit);
  }

  /** One step of the P loop. Also stores what it saw so periodic() can log it. */
  private void runPositionLoop() {
    int position = hoodMotor.getPosition();
    int error = getTarget() - position;
    lastPosition = position;
    lastError = error;
    lastAtTarget = Math.abs(error) < DEADBAND_TICKS;
    lastAtSoftLimit = false;

    if (lastAtTarget) {
      hoodMotor.stop();
      return;
    }

    double power = Math.max(-MAX_POWER, Math.min(MAX_POWER, KP * error));

    // Soft limits: never drive further down below MIN_TICKS or up above maxTicks().
    if ((power < 0 && position <= MIN_TICKS) || (power > 0 && position >= maxTicks())) {
      lastAtSoftLimit = true;
      hoodMotor.stop();
      return;
    }

    hoodMotor.setPower(power);
  }

  public void setPreset(HoodPreset newPreset) {
    preset = newPreset;
    directMode = false;
    manualPower = Double.NaN;
  }

  /**
   * Targets an exact position, overriding the preset until the next
   * {@link #setPreset(HoodPreset)}. Clamped to the soft limits.
   */
  public void setTargetTicks(int ticks) {
    directTicks = Math.max(MIN_TICKS, Math.min(maxTicks(), ticks));
    directMode = true;
    manualPower = Double.NaN;
  }

  /** True while a direct-ticks target is overriding the preset. */
  public boolean isDirectMode() {
    return directMode;
  }

  /**
   * Drives the hood open loop at this power (0 holds it with brake), ignoring
   * the preset and soft limits until the next {@link #setPreset(HoodPreset)}.
   */
  public void setManualPower(double power) {
    manualPower = power;
  }

  public HoodPreset getPreset() {
    return preset;
  }

  /**
   * Converts a launch angle to encoder ticks. Ticks count up as the hood
   * raises and the launch angle drops from {@link #ZERO_ANGLE_DEG}.
   */
  public static int angleToTicks(double launchAngleDeg) {
    return (int) Math.round((ZERO_ANGLE_DEG - launchAngleDeg) * TICKS_PER_DEG);
  }

  /** Upper soft limit, in ticks. */
  private static int maxTicks() {
    return angleToTicks(MIN_ANGLE_DEG);
  }

  /** Converts encoder ticks to a launch angle in degrees. */
  public static double ticksToAngle(int ticks) {
    return ZERO_ANGLE_DEG - ticks / TICKS_PER_DEG;
  }

  /** Hood position in Through Bore ticks. */
  public int getPosition() {
    return hoodMotor.getPosition();
  }

  /** Target position in ticks: the direct override if set, else the preset's. */
  public int getTarget() {
    return directMode ? directTicks : preset.getTicks();
  }

  /** True when the hood is within the deadband of its target. */
  public boolean atTarget() {
    return Math.abs(getTarget() - getPosition()) < DEADBAND_TICKS;
  }

  /** True when the hood is within toleranceTicks of its target. */
  public boolean atTarget(int toleranceTicks) {
    return Math.abs(getTarget() - getPosition()) <= toleranceTicks;
  }
}
