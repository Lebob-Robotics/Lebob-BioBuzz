package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.GobildaMotor;
import org.psilynx.psikit.core.Logger;

/**
 * Subsystem to control the shooter hood.
 *
 * <p>The 435 RPM Yellow Jacket output drives the pinion 1:1. Its own encoder remains on
 * Expansion Hub encoder port 0 (HoodMotor). The REV Through Bore on the pinion is connected
 * to Control Hub encoder port 0 (IntakeMotor); the intake is deliberately power-controlled.
 * Hood position comes from the REV's 8192 counts/rev through the 10:1 hood reduction.
 * The motor's approximately 384.5 counts/rev are an independent movement cross-check.
 *
 * <p>The Hub's velocity loop uses the native motor encoder. Our slower outer position loop
 * uses the REV to approach the target without driving the mechanism at full power past it.
 * The Hub can apply full electrical power as needed to maintain the requested motor speed.
 *
 * <p>The gains and geometry are live in Panels. Ticks are always derived from the current geometry,
 * so editing it takes effect immediately.
 */
@Configurable
public class HoodSubsystem extends SubsystemBase {
  /** Outer position gain: native motor velocity request fraction per REV tick of error. */
  public static double KP = 0.00008;
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
  private final DcMotorEx revEncoderInput;
  private final HoodEncoderGuard encoderGuard = new HoodEncoderGuard();
  private final ElapsedTime clock = new ElapsedTime();
  private int revZero;
  private int motorZero;
  private boolean zeroed;
  private int lastMotorPosition;

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

  // What the last P loop step saw, kept for logging.
  private int lastPosition;
  private int lastError;
  private boolean lastAtTarget;
  private boolean lastAtSoftLimit;

  public HoodSubsystem(HardwareMap hardwareMap) {
    // The motor port now carries its native encoder; the independent REV is on IntakeMotor.
    // Native encoder speed regulation supplies starting torque without an open-loop power kick.
    hoodMotor = GobildaMotor.speedControlled(hardwareMap, "HoodMotor", false, true);
    // Read only: never change IntakeMotor's power, mode, direction or encoder zero here.
    revEncoderInput = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
  }

  /**
   * Called once from the OpMode's init(). Zeroes the encoder, so the hood must
   * be resting at its lower hard stop when this runs.
   */
  public void init() {
    hoodMotor.stop();
    revZero = revEncoderInput.getCurrentPosition();
    motorZero = hoodMotor.getPosition();
    encoderGuard.reset();
    clock.reset();
    directMode = false;
    preset = HoodPreset.STOW;
    zeroed = true;
  }

  @Override
  public void periodic() {
    runPositionLoop();

    Logger.recordOutput("Hood/Preset", preset);
    Logger.recordOutput("Hood/DirectMode", directMode);
    Logger.recordOutput("Hood/TargetTicks", getTarget());
    Logger.recordOutput("Hood/PositionTicks", lastPosition);
    Logger.recordOutput("Hood/TargetAngleDeg", ticksToAngle(getTarget()));
    Logger.recordOutput("Hood/AngleDeg", ticksToAngle(lastPosition));
    Logger.recordOutput("Hood/ErrorTicks", lastError);
    Logger.recordOutput("Hood/CommandedPower", hoodMotor.getCommandedPower());
    Logger.recordOutput("Hood/ControlMode", "NATIVE_VELOCITY_WITH_REV_POSITION");
    Logger.recordOutput("Hood/VelocityTargetTicksPerSec", hoodMotor.getCommandedPower() * hoodMotor.getMaxVelocity());
    Logger.recordOutput("Hood/VelocityActualTicksPerSec", hoodMotor.getVelocity());
    Logger.recordOutput("Hood/AtTarget", lastAtTarget);
    Logger.recordOutput("Hood/AtSoftLimit", lastAtSoftLimit);
    Logger.recordOutput("Hood/EncoderGuard/Tripped", encoderGuard.isTripped());
    Logger.recordOutput("Hood/EncoderGuard/MotorTicks", lastMotorPosition);
    Logger.recordOutput("Hood/EncoderGuard/RevTicks", lastPosition);
    Logger.recordOutput("Hood/EncoderGuard/DifferencePinionTurns", Math.abs(
        lastPosition / HoodEncoderGuard.REV_COUNTS_PER_TURN
        - lastMotorPosition / HoodEncoderGuard.MOTOR_COUNTS_PER_TURN));
  }

  /** One step of the P loop. Also stores what it saw so periodic() can log it. */
  private void runPositionLoop() {
    int position = getPosition();
    lastMotorPosition = hoodMotor.getPosition() - motorZero;
    int error = getTarget() - position;
    lastPosition = position;
    lastError = error;
    lastAtTarget = zeroed && !encoderGuard.isTripped() && Math.abs(error) < DEADBAND_TICKS;
    lastAtSoftLimit = false;

    if (!zeroed || encoderGuard.isTripped() || encoderGuard.update(clock.seconds(), position,
        lastMotorPosition, Math.abs(hoodMotor.getCommandedPower()) > 0.0001)) {
      lastAtTarget = false;
      hoodMotor.cut(); // A fault must remove drive power, not hold zero speed with the Hub PID.
      return;
    }

    if (lastAtTarget) {
      hoodMotor.stop();
      return;
    }

    double velocityRequest = positionVelocityRequest(error);

    // Soft limits: never drive further down below MIN_TICKS or up above maxTicks().
    if ((velocityRequest < 0 && position <= MIN_TICKS) || (velocityRequest > 0 && position >= maxTicks())) {
      lastAtSoftLimit = true;
      hoodMotor.cut();
      return;
    }

    hoodMotor.setPower(velocityRequest); // RUN_USING_ENCODER: this requests speed, not raw electrical power.
  }

  static double positionVelocityRequest(int errorTicks) {
    return Math.max(-1, Math.min(1, KP * errorTicks));
  }

  public void setPreset(HoodPreset newPreset) {
    preset = newPreset;
    directMode = false;
  }

  /**
   * Targets an exact position, overriding the preset until the next
   * {@link #setPreset(HoodPreset)}. Clamped to the soft limits.
   */
  public void setTargetTicks(int ticks) {
    directTicks = Math.max(MIN_TICKS, Math.min(maxTicks(), ticks));
    directMode = true;
  }

  /** True while a direct-ticks target is overriding the preset. */
  public boolean isDirectMode() {
    return directMode;
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

  /** Hood position from the REV encoder input, relative to the lower-stop zero. */
  public int getPosition() {
    return revEncoderInput.getCurrentPosition() - revZero;
  }

  /** Physical hood lift above the lower stop, independent of the launch-angle convention. */
  public double getLiftDegrees() {
    return getPosition() / TICKS_PER_DEG;
  }

  public double getTargetLiftDegrees() {
    return getTarget() / TICKS_PER_DEG;
  }

  public void setTargetLiftDegrees(double liftDegrees) {
    setTargetTicks((int) Math.round(liftDegrees * TICKS_PER_DEG));
  }

  public boolean hasEncoderFault() {
    return encoderGuard.isTripped();
  }

  /** Target position in ticks: the direct override if set, else the preset's. */
  public int getTarget() {
    return directMode ? directTicks : preset.getTicks();
  }

  /** True when the hood is within the deadband of its target. */
  public boolean atTarget() {
    return zeroed && !hasEncoderFault() && Math.abs(getTarget() - getPosition()) < DEADBAND_TICKS;
  }

  /** True when the hood is within toleranceTicks of its target. */
  public boolean atTarget(int toleranceTicks) {
    return zeroed && !hasEncoderFault() && Math.abs(getTarget() - getPosition()) <= toleranceTicks;
  }
}
