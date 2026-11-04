package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.HubMotors;
import org.psilynx.psikit.core.Logger;

/**
 * Subsystem to control the shooter hood.
 *
 * <p>The 435 RPM Yellow Jacket output drives the pinion 1:1. Its own encoder remains on
 * Expansion Hub encoder port 0 (HoodMotor). The REV Through Bore on the pinion is connected
 * to Control Hub encoder port 0 (IntakeMotor); the intake is deliberately power-controlled.
 * Hood position comes from the REV's 8192 counts/rev through the 10:1 hood reduction.
 *
 * <p>The hub runs the position loop itself (RUN_TO_POSITION on the motor's own encoder, at full
 * speed). The TeleOp loop runs at about 12 Hz, far too slow to close a loop on a hood that moves
 * over 100 degrees a second; the hub's loop does not wait for it. Each loop the REV target is
 * mapped to a motor target through {@link HoodMotorTargeting}, which re-anchors on the REV at rest.
 *
 * <p>Targets stay {@link #STOP_MARGIN_DEG} inside both stops, so the hub never drives into one at
 * full power. The hood is not driven at all until something sets a target after init(), so INIT
 * never moves it.
 *
 * <p>Gains go to the hub's motor behind PsiKit's wrapper ({@link HubMotors}); the wrapper drops them.
 * POSITION_P, the VELOCITY_* gains and the margins are live in Panels. All gains are written at
 * every INIT: the hub keeps whatever the last OpMode wrote until it is power-cycled.
 */
@Configurable
public class HoodSubsystem extends SubsystemBase {
  /** Hub RUN_TO_POSITION gain. Higher is faster until it overshoots. */
  public static double POSITION_P = 20.0;
  /**
   * Hub speed loop under RUN_TO_POSITION, in SDK units. These are the SDK's own goBILDA defaults,
   * the only set seen well damped on this hood; a feedforward drives the motor through the belt
   * slack and overshoots.
   */
  public static double VELOCITY_P = 10.0;
  public static double VELOCITY_I = 3.0;
  public static double VELOCITY_D = 0.0;
  public static double VELOCITY_F = 0.0;
  /** Share of full motor speed RUN_TO_POSITION may use. */
  public static double MAX_SPEED = 1.0;
  /** The hub counts a move done within this many motor ticks (one tick is about 0.09 degrees). */
  public static int HUB_TOLERANCE_TICKS = 1;
  /** REV errors smaller than this (ticks) are on target: 45 ticks is 0.2 degrees. */
  public static int DEADBAND_TICKS = 45;
  /** Targets are kept this far inside both stops. */
  public static double STOP_MARGIN_DEG = 1.0;
  /** Motor speeds below this (ticks/s) count as at rest; the hub reports speed in 20 tick/s steps. */
  public static double AT_REST_TICKS_PER_SEC = 30;
  /** Motor error the guard treats as "trying to move" for its stall check. */
  private static final int GUARD_BUSY_MOTOR_TICKS = 5;
  /** Backstop on the motor target beyond the REV-mapped range: backlash (16) plus drift. */
  private static final int MOTOR_TARGET_SLACK_TICKS = 20;

  // ---- Geometry --------------------------------------------------------
  /** Launch angle at the lower hard stop, where the encoder is zeroed. */
  public static double ZERO_ANGLE_DEG = 99.0;
  /** Launch angle at the top of the hood's travel; sets the upper soft limit. */
  public static double MIN_ANGLE_DEG = 51.0; // TODO tune on robot
  /** Encoder ticks per degree of launch angle: 8192 ticks/rev through the 10:1. */
  public static double TICKS_PER_DEG = 8192.0 * 10.0 / 360.0;

  private final DcMotorEx hoodMotor;
  private final DcMotorEx hubMotor; // Same motor, past the logging wrapper, for gains and tolerance
  private final DcMotorEx revEncoderInput;
  private final HoodEncoderGuard encoderGuard = new HoodEncoderGuard();
  private final HoodMotorTargeting targeting = new HoodMotorTargeting();
  private int hubTolerance = -1;
  private final ElapsedTime clock = new ElapsedTime();
  private int revZero;
  private int motorZero;
  private boolean zeroed;
  private boolean enabled; // Set by the first target after init()
  private boolean driving; // In RUN_TO_POSITION
  private int lastMotorTarget = Integer.MIN_VALUE;
  private double appliedPositionP = Double.NaN;
  private final double[] appliedVelocityPidf = new double[4];

  /** Hood positions, each given as a launch angle and held in Through Bore ticks. */
  public enum HoodPreset {
    /** As low as the stop margin allows. */
    STOW(Double.NaN),
    CLOSE(75.0), // TODO tune on robot
    MID(65.0), // TODO tune on robot
    FAR(55.0); // TODO tune on robot

    private final double launchAngleDeg;

    HoodPreset(double launchAngleDeg) {
      this.launchAngleDeg = launchAngleDeg;
    }

    public int getTicks() {
      return Double.isNaN(launchAngleDeg) ? 0 : angleToTicks(launchAngleDeg);
    }
  }

  private HoodPreset preset = HoodPreset.STOW;
  // When on, directTicks overrides the preset (used for continuous aiming).
  private boolean directMode = false;
  private int directTicks = 0;

  // What the last loop step saw, kept for logging.
  private int lastPosition;
  private int lastError;
  private int lastMotorPosition;
  private double lastMotorVelocity;
  private boolean lastAtTarget;

  public HoodSubsystem(HardwareMap hardwareMap) {
    hoodMotor = hardwareMap.get(DcMotorEx.class, "HoodMotor");
    hubMotor = HubMotors.unwrap(hoodMotor);
    hoodMotor.setDirection(DcMotor.Direction.FORWARD); // Positive raises the hood (verified on robot)
    hoodMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    cut();
    // Read only: never change IntakeMotor's power, mode, direction or encoder zero here.
    revEncoderInput = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
  }

  /**
   * Called once from the OpMode's init(). Zeroes the encoders, so the hood must
   * be resting at its lower hard stop when this runs. The hood stays undriven until a target is set.
   */
  public void init() {
    cut();
    revZero = revEncoderInput.getCurrentPosition();
    motorZero = hoodMotor.getCurrentPosition();
    encoderGuard.reset();
    targeting.reset();
    clock.reset();
    directMode = false;
    preset = HoodPreset.STOW;
    enabled = false;
    zeroed = true;
    // Log what the hub held before this OpMode wrote anything (the previous OpMode's gains).
    PIDFCoefficients position = hubMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
    PIDFCoefficients velocity = hubMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
    Logger.recordOutput("Hood/HubAtInit/PositionP", position.p);
    Logger.recordOutput("Hood/HubAtInit/VelocityPIDF",
        new double[] {velocity.p, velocity.i, velocity.d, velocity.f});
    appliedPositionP = Double.NaN; // Force a full write on the first drive
    java.util.Arrays.fill(appliedVelocityPidf, Double.NaN);
    Logger.recordOutput("Hood/GainsReachHub", hubMotor != hoodMotor);
  }

  @Override
  public void periodic() {
    runPositionLoop();

    Logger.recordOutput("Hood/Preset", preset);
    Logger.recordOutput("Hood/DirectMode", directMode);
    Logger.recordOutput("Hood/Enabled", enabled);
    Logger.recordOutput("Hood/Driving", driving);
    Logger.recordOutput("Hood/TargetTicks", getTarget());
    Logger.recordOutput("Hood/PositionTicks", lastPosition);
    Logger.recordOutput("Hood/TargetAngleDeg", ticksToAngle(getTarget()));
    Logger.recordOutput("Hood/AngleDeg", ticksToAngle(lastPosition));
    Logger.recordOutput("Hood/ErrorTicks", lastError);
    Logger.recordOutput("Hood/MotorTargetTicks", lastMotorTarget);
    Logger.recordOutput("Hood/MotorTicks", lastMotorPosition);
    Logger.recordOutput("Hood/MotorVelocityTicksPerSec", lastMotorVelocity);
    Logger.recordOutput("Hood/OffsetMotorTicks", targeting.offset());
    Logger.recordOutput("Hood/OffsetUpMotorTicks", targeting.offsetUp());
    Logger.recordOutput("Hood/OffsetDownMotorTicks", targeting.offsetDown());
    Logger.recordOutput("Hood/HubToleranceTicks", hubTolerance);
    Logger.recordOutput("Hood/PositionP", appliedPositionP);
    Logger.recordOutput("Hood/VelocityPIDF", appliedVelocityPidf);
    Logger.recordOutput("Hood/AtTarget", lastAtTarget);
    Logger.recordOutput("Hood/EncoderGuard/Tripped", encoderGuard.isTripped());
    Logger.recordOutput("Hood/EncoderGuard/Slips", encoderGuard.getSlipCount());
    Logger.recordOutput("Hood/EncoderGuard/DifferencePinionTurns", Math.abs(
        lastPosition / HoodEncoderGuard.REV_COUNTS_PER_TURN
        - lastMotorPosition / HoodEncoderGuard.MOTOR_COUNTS_PER_TURN));
  }

  /** One loop step. Also stores what it saw so periodic() can log it. */
  private void runPositionLoop() {
    int position = getPosition();
    int motorPosition = hoodMotor.getCurrentPosition() - motorZero;
    double motorVelocity = hoodMotor.getVelocity();
    int target = getTarget();
    lastPosition = position;
    lastMotorPosition = motorPosition;
    lastMotorVelocity = motorVelocity;
    lastError = target - position;
    lastAtTarget = zeroed && !encoderGuard.isTripped() && Math.abs(lastError) < DEADBAND_TICKS;

    boolean trying = driving && Math.abs(lastMotorTarget - motorPosition) > GUARD_BUSY_MOTOR_TICKS;
    boolean atRest = Math.abs(motorVelocity) < AT_REST_TICKS_PER_SEC;
    if (!zeroed || encoderGuard.update(clock.seconds(), position, motorPosition, trying, atRest)) {
      lastAtTarget = false;
      cut(); // A fault must remove drive power; RUN_TO_POSITION would keep pushing.
      return;
    }
    if (encoderGuard.takeRebase()) { // Belt skipped: the REV is the hood, so move the motor's zero to match it
      int motorFromRev = (int) Math.round(position / HoodEncoderGuard.REV_COUNTS_PER_TURN
          * HoodEncoderGuard.MOTOR_COUNTS_PER_TURN);
      motorZero += motorPosition - motorFromRev;
      motorPosition = motorFromRev;
      lastMotorPosition = motorPosition;
      lastMotorTarget = Integer.MIN_VALUE; // Resend the target against the new zero
      targeting.reset();
    }
    if (!enabled) {
      return;
    }
    if (target == 0) { // Rest on the lower stop unpowered; holding just above it only stalls the motor
      if (driving) {
        cut();
      }
      return;
    }

    boolean moveDone = driving && atRest
        && Math.abs(lastMotorTarget - motorPosition) <= GUARD_BUSY_MOTOR_TICKS;
    targeting.observe(position, motorPosition, moveDone, target, DEADBAND_TICKS);
    // A wrong offset (say a REV fault the guard has not yet caught) must not aim past either stop.
    int motorTarget = Math.max(
        (int) Math.round(minTargetTicks() * HoodMotorTargeting.MOTOR_PER_REV) - MOTOR_TARGET_SLACK_TICKS,
        Math.min((int) Math.round(maxTargetTicks() * HoodMotorTargeting.MOTOR_PER_REV) + MOTOR_TARGET_SLACK_TICKS,
            targeting.motorTargetFor(target, position, DEADBAND_TICKS)));
    if (motorTarget != lastMotorTarget || !driving) {
      hoodMotor.setTargetPosition(motorZero + motorTarget); // Must be set before entering RUN_TO_POSITION
      lastMotorTarget = motorTarget;
    }
    applyGains();
    if (!driving) {
      hubMotor.setTargetPositionTolerance(HUB_TOLERANCE_TICKS);
      hubTolerance = hubMotor.getTargetPositionTolerance(); // Logged: confirms the write reached the hub
      hoodMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      hoodMotor.setPower(MAX_SPEED);
      driving = true;
    }
  }

  /** Sends gains changed in Panels (or not yet written this OpMode) to the hub. */
  private void applyGains() {
    if (POSITION_P != appliedPositionP) {
      hubMotor.setPositionPIDFCoefficients(POSITION_P);
      appliedPositionP = POSITION_P;
    }
    double[] pidf = {
        VELOCITY_P, VELOCITY_I, VELOCITY_D, VELOCITY_F};
    if (!java.util.Arrays.equals(pidf, appliedVelocityPidf)) {
      hubMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
          new PIDFCoefficients(pidf[0], pidf[1], pidf[2], pidf[3]));
      System.arraycopy(pidf, 0, appliedVelocityPidf, 0, 4);
    }
  }

  /** Removes drive power until the next target. Open loop, so nothing holds a setpoint. */
  public void disable() {
    enabled = false;
    cut();
  }

  private void cut() {
    hoodMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    hoodMotor.setPower(0);
    driving = false;
  }

  public void setPreset(HoodPreset newPreset) {
    preset = newPreset;
    directMode = false;
    enabled = true;
  }

  /**
   * Targets an exact position, overriding the preset until the next
   * {@link #setPreset(HoodPreset)}. Clamped to the soft limits.
   */
  public void setTargetTicks(int ticks) {
    directTicks = clampTarget(ticks);
    directMode = true;
    enabled = true;
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

  /** Lowest target, in ticks: the stop margin above the lower stop. */
  public static int minTargetTicks() {
    return (int) Math.round(STOP_MARGIN_DEG * TICKS_PER_DEG);
  }

  /** Highest target, in ticks: the stop margin below the top of travel. */
  static int maxTargetTicks() {
    return angleToTicks(MIN_ANGLE_DEG + STOP_MARGIN_DEG);
  }

  /** Kept inside the margins, except that a target at or below the lower stop rests on it (0). */
  static int clampTarget(int ticks) {
    if (ticks <= 0) {
      return 0;
    }
    return Math.max(minTargetTicks(), Math.min(maxTargetTicks(), ticks));
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

  /** Target position in ticks: the direct override if set, else the preset's, kept inside the margins. */
  public int getTarget() {
    return clampTarget(directMode ? directTicks : preset.getTicks());
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
