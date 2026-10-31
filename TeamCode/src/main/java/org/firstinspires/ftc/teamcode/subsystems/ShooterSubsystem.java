package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.GobildaMotor;
import org.psilynx.psikit.core.Logger;

import java.util.Arrays;

/**
 * Controls the two side-by-side shooter flywheels. Feeding balls into them is
 * done by {@link FeederSubsystem} and {@link IndexerSubsystem}.
 *
 * <ul>
 * <li>{@link ShooterState#IDLE} spins the flywheels a little slower (the
 * initial state), so they are close to shooting speed.</li>
 * <li>{@link ShooterState#SHOOT} spins them at full open-loop power.</li>
 * <li>{@link ShooterState#EJECT} runs them backward.</li>
 * <li>{@link ShooterState#AIM} holds {@link #setTargetRpm(double)} in closed loop.</li>
 * <li>{@link ShooterState#STOP} stops them.</li>
 * </ul>
 *
 * {@link ShooterStallGuard} cuts both motors if either stalls, in any running
 * state. Once tripped the shooter stays off until the driver selects
 * {@link ShooterState#STOP}.
 */
@Configurable
public class ShooterSubsystem extends SubsystemBase {
  // ---- Tuning ----------------------------------------------------------
  // Flywheel velocity PIDF for AIM, in SDK units. NaN takes the SDK default at startup, so Panels
  // shows the values actually in use. Edits apply on the next loop.
  public static double VELOCITY_P = Double.NaN;
  public static double VELOCITY_I = Double.NaN;
  public static double VELOCITY_D = Double.NaN;
  public static double VELOCITY_F = Double.NaN;
  private static final double SHOOTER_VELOCITY = 4000;
  /** Motor encoder ticks per motor revolution (goBILDA 5203 6000 RPM). */
  private static final double MOTOR_TICKS_PER_REV = 28.0; // TODO verify
  /** Motor revolutions per flywheel revolution (gear ratio). */
  private static final double MOTOR_REVS_PER_WHEEL_REV = 1.0 / 0.7;
  /** Target changes smaller than this (RPM) are not re-sent to the motors. */
  private static final double TARGET_RESEND_RPM = 1.0;
  private static final double IDLE_POWER = 0.8;
  private static final double SHOOT_POWER = 1.0;
  /** Current and velocity are separate hub reads, so sample them this often. */
  private static final double STALL_SAMPLE_PERIOD_S = 0.1;

  // ---- Hardware --------------------------------------------------------
  private final GobildaMotor shooterRight;
  private final GobildaMotor shooterLeft;

  public enum ShooterState {
    STOP, IDLE, SHOOT, EJECT, AIM
  }

  private ShooterState shooterState;
  // Flywheel RPM that AIM holds.
  private double targetRpm = 0.0;
  // Last target actually sent, so slow drifts below TARGET_RESEND_RPM still go out.
  private double sentRpm = Double.NaN;
  private ShooterState previousShooterState = ShooterState.STOP;
  // Tracked here (not read back from the motors) so the logged mode replays identically.
  // Matches the mode the motors are built with.
  private DcMotor.RunMode shooterMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
  // Last PIDF sent to the motors, {p, i, d, f}.
  private final double[] sentPidf = {Double.NaN, Double.NaN, Double.NaN, Double.NaN};

  // ---- Stall protection ------------------------------------------------
  private final ShooterStallGuard stallGuard = new ShooterStallGuard();
  private final ElapsedTime clock = new ElapsedTime();
  private double lastStallSampleS = Double.NEGATIVE_INFINITY;
  private double leftAmps;
  private double rightAmps;

  public ShooterSubsystem(HardwareMap hardwareMap) {
    shooterRight = new GobildaMotor(hardwareMap, "ShooterR", false, DcMotor.RunMode.RUN_WITHOUT_ENCODER, false);
    // Side-by-side wheels spin opposite ways, so one is reversed. Direction is
    // to be verified on the robot.
    shooterLeft = new GobildaMotor(hardwareMap, "ShooterL", true, DcMotor.RunMode.RUN_WITHOUT_ENCODER, false);

    PIDFCoefficients sdk = shooterRight.getVelocityPidf();
    if (Double.isNaN(VELOCITY_P)) {
      VELOCITY_P = sdk.p;
    }
    if (Double.isNaN(VELOCITY_I)) {
      VELOCITY_I = sdk.i;
    }
    if (Double.isNaN(VELOCITY_D)) {
      VELOCITY_D = sdk.d;
    }
    if (Double.isNaN(VELOCITY_F)) {
      VELOCITY_F = sdk.f;
    }
    applyVelocityPidf();

    setShooterState(ShooterState.IDLE);
  }

  private void doShooterState() {
    if (previousShooterState == shooterState) {
      return;
    }

    switch (shooterState) {
      // Everything but AIM is open loop: RUN_USING_ENCODER would cap power 1.0
      // at 85% of the motor's rated speed (about 5100 RPM on the 6000 RPM motors).
      case STOP:
        setShooterMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setShooterPower(0.0);
        break;
      case IDLE:
        setShooterMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setShooterPower(IDLE_POWER);
        break;
      case SHOOT:
        setShooterMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setShooterPower(SHOOT_POWER);
        break;
      case EJECT:
        setShooterMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setShooterPower(-1.0);
        break;
      case AIM:
        setShooterMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sendTargetVelocity();
        break;
    }
  }

  public void setShooterState(ShooterState newShooterState) {
    if (newShooterState == ShooterState.STOP && stallGuard.isTripped()) {
      stallGuard.rearm();
    }
    previousShooterState = shooterState;
    shooterState = newShooterState;
    doShooterState();
  }

  public ShooterState getShooterState() {
    return shooterState;
  }

  /**
   * Sets the flywheel RPM held in {@link ShooterState#AIM}. Applies immediately
   * if already aiming; otherwise it takes effect on entering AIM.
   */
  public void setTargetRpm(double rpm) {
    rpm = Math.max(0.0, rpm);
    targetRpm = rpm;
    // doShooterState ignores unchanged states, so re-send the velocity here.
    if (shooterState == ShooterState.AIM
        && !(Math.abs(rpm - sentRpm) <= TARGET_RESEND_RPM)) {
      sendTargetVelocity();
    }
  }

  public double getTargetRpm() {
    return targetRpm;
  }

  /** Average flywheel speed of both wheels, in RPM. */
  public double getRpm() {
    return (getRpmLeft() + getRpmRight()) / 2.0;
  }

  /** True only in AIM, when each wheel is within toleranceRpm of the target. */
  public boolean atSpeed(double toleranceRpm) {
    return shooterState == ShooterState.AIM
        && Math.abs(getRpmLeft() - targetRpm) <= toleranceRpm
        && Math.abs(getRpmRight() - targetRpm) <= toleranceRpm;
  }

  private double getRpmLeft() {
    return ticksPerSecToRpm(Math.abs(shooterLeft.getVelocity()));
  }

  private double getRpmRight() {
    return ticksPerSecToRpm(Math.abs(shooterRight.getVelocity()));
  }

  private static double ticksPerSecToRpm(double ticksPerSec) {
    return ticksPerSec / MOTOR_TICKS_PER_REV / MOTOR_REVS_PER_WHEEL_REV * 60.0;
  }

  private static double rpmToTicksPerSec(double rpm) {
    return rpm * MOTOR_REVS_PER_WHEEL_REV * MOTOR_TICKS_PER_REV / 60.0;
  }

  private void sendTargetVelocity() {
    if (stallGuard.isTripped()) {
      // The SDK velocity loop would keep driving a stalled motor, so hold it off.
      setShooterPower(0.0);
      return;
    }
    sentRpm = targetRpm;
    double ticksPerSec = rpmToTicksPerSec(targetRpm);
    shooterRight.setVelocity(ticksPerSec);
    shooterLeft.setVelocity(ticksPerSec);
  }

  public boolean isAtSpeed() {
    double minSpeed = SHOOTER_VELOCITY;
    return Math.abs(shooterRight.getVelocity()) >= minSpeed
        && Math.abs(shooterLeft.getVelocity()) >= minSpeed;
  }

  /** Sends the PIDF to both motors if Panels changed it. */
  private void applyVelocityPidf() {
    double[] pidf = {VELOCITY_P, VELOCITY_I, VELOCITY_D, VELOCITY_F};
    if (Arrays.equals(pidf, sentPidf)) {
      return;
    }
    shooterRight.setVelocityPidf(pidf[0], pidf[1], pidf[2], pidf[3]);
    shooterLeft.setVelocityPidf(pidf[0], pidf[1], pidf[2], pidf[3]);
    System.arraycopy(pidf, 0, sentPidf, 0, 4);
  }

  /** True while the stall guard is holding the shooter off. */
  public boolean isStalled() {
    return stallGuard.isTripped();
  }

  /** Stall guard state for telemetry. */
  public String getStallStatus() {
    if (stallGuard.isLockedOut()) {
      return "STALLED, off until restart: check belts and pulleys";
    }
    if (stallGuard.isTripped()) {
      return String.format("STALLED (%d/%d), check belts, X to reset",
          stallGuard.getTrips(), ShooterStallGuard.MAX_TRIPS);
    }
    return "ok";
  }

  /**
   * Samples both motors and cuts them if either stalls. See
   * {@link ShooterStallGuard}.
   */
  private void checkStall() {
    double now = clock.seconds();
    if (now - lastStallSampleS < STALL_SAMPLE_PERIOD_S) {
      return;
    }
    lastStallSampleS = now;

    boolean powered = shooterState != ShooterState.STOP;
    double leftVelocity = shooterLeft.getVelocity();
    double rightVelocity = shooterRight.getVelocity();
    leftAmps = powered ? shooterLeft.getCurrentAmps() : 0.0;
    rightAmps = powered ? shooterRight.getCurrentAmps() : 0.0;
    boolean wasTripped = stallGuard.isTripped();
    if (stallGuard.update(now, powered, leftAmps, leftVelocity, rightAmps, rightVelocity) && !wasTripped) {
      // Leave velocity mode so the SDK loop stops driving the motors.
      setShooterMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      setShooterPower(0.0);
      RobotLog.ee("BioBuzz", "shooter stalled, trip %d/%d: L %.1f A %.0f ticks/s, R %.1f A %.0f ticks/s",
          stallGuard.getTrips(), ShooterStallGuard.MAX_TRIPS,
          leftAmps, leftVelocity, rightAmps, rightVelocity);
    }
  }

  @Override
  public void periodic() {
    applyVelocityPidf();
    checkStall();
    Logger.recordOutput("Shooter/State", shooterState);
    Logger.recordOutput("Shooter/VelocityLeft", shooterLeft.getVelocity());
    Logger.recordOutput("Shooter/VelocityRight", shooterRight.getVelocity());
    Logger.recordOutput("Shooter/AtSpeed", isAtSpeed());
    Logger.recordOutput("Shooter/AtSpeedThreshold", SHOOTER_VELOCITY);
    Logger.recordOutput("Shooter/TargetRpm", targetRpm);
    Logger.recordOutput("Shooter/Rpm", getRpm());
    Logger.recordOutput("Shooter/RpmLeft", getRpmLeft());
    Logger.recordOutput("Shooter/RpmRight", getRpmRight());
    Logger.recordOutput("Shooter/Mode", shooterMode);
    Logger.recordOutput("Shooter/CommandedPower", shooterRight.getCommandedPower());
    Logger.recordOutput("Shooter/VelocityPidf", sentPidf.clone());
    Logger.recordOutput("Shooter/StallGuard/Tripped", stallGuard.isTripped());
    Logger.recordOutput("Shooter/StallGuard/Trips", stallGuard.getTrips());
    Logger.recordOutput("Shooter/StallGuard/LeftAmps", leftAmps);
    Logger.recordOutput("Shooter/StallGuard/RightAmps", rightAmps);
  }

  private void setShooterMode(DcMotor.RunMode mode) {
    if (mode == shooterMode) {
      return;
    }
    shooterMode = mode;
    shooterRight.setMode(mode);
    shooterLeft.setMode(mode);
  }

  /** Every shooter power write goes through here, so a stall trip cannot be overridden. */
  private void setShooterPower(double power) {
    if (stallGuard.isTripped()) {
      power = 0.0;
    }
    shooterRight.setPower(power);
    shooterLeft.setPower(power);
  }
}
