package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.servo.GobildaMotor;
import org.psilynx.psikit.core.Logger;

import java.util.Arrays;

/**
 * Controls the two side-by-side shooter flywheels. Feeding balls into them is
 * done by {@link FeederSubsystem} and {@link IndexerSubsystem}.
 *
 * <ul>
 * <li>{@link ShooterState#IDLE} holds IDLE_RPM (the initial state), so the wheels are part way to
 * shooting speed.</li>
 * <li>{@link ShooterState#SHOOT} spins them at the fastest allowed speed.</li>
 * <li>{@link ShooterState#EJECT} runs them backward.</li>
 * <li>{@link ShooterState#AIM} holds {@link #setTargetRpm(double)} in closed loop.</li>
 * <li>{@link ShooterState#STOP} stops them.</li>
 * </ul>
 *
 * <p>Both motors always use the hub's speed control, and {@link GobildaMotor} caps them at
 * MAX_OUTPUT of full speed, so every state above is capped too.
 *
 * <p>{@link ShooterStallGuard} cuts both motors if either stalls, in any running state. Under speed
 * control a stalled wheel gets full power just as it would open loop, so the guard is still needed.
 * Once tripped the shooter stays off until the driver selects {@link ShooterState#STOP}.
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
  public static double IDLE_RPM = 2500; // Middle of the shot table, so any aimed shot is a short spin-up away
  private static final double SHOOTER_VELOCITY = 4000;
  /** Motor encoder ticks per motor revolution (goBILDA 5203 6000 RPM). */
  private static final double MOTOR_TICKS_PER_REV = 28.0; // TODO verify
  /** Motor revolutions per flywheel revolution: 23 tooth wheel pulley over 33 tooth motor pulley (CAD). */
  private static final double MOTOR_REVS_PER_WHEEL_REV = 23.0 / 33.0;
  /** Target changes smaller than this (RPM) are not re-sent to the motors. */
  private static final double TARGET_RESEND_RPM = 1.0;
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
  // Last PIDF sent to the motors, {p, i, d, f}.
  private final double[] sentPidf = {Double.NaN, Double.NaN, Double.NaN, Double.NaN};

  // ---- Stall protection ------------------------------------------------
  private final ShooterStallGuard stallGuard = new ShooterStallGuard();
  private final ElapsedTime clock = new ElapsedTime();
  private double lastStallSampleS = Double.NEGATIVE_INFINITY;
  private double leftAmps;
  private double rightAmps;

  public ShooterSubsystem(HardwareMap hardwareMap) {
    shooterRight = GobildaMotor.speedControlled(hardwareMap, "ShooterR", false, false);
    // Side-by-side wheels spin opposite ways, so one is reversed. Direction is
    // to be verified on the robot.
    shooterLeft = GobildaMotor.speedControlled(hardwareMap, "ShooterL", true, false);

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
      case STOP:
        setShooterPower(0.0);
        break;
      case IDLE:
        setShooterRpm(IDLE_RPM);
        break;
      case SHOOT:
        setShooterPower(1.0); // Capped to MAX_OUTPUT of full speed by GobildaMotor
        break;
      case EJECT:
        setShooterPower(-1.0);
        break;
      case AIM:
        setShooterRpm(targetRpm);
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
    rpm = Math.max(0.0, Math.min(getMaxRpm(), rpm)); // So atSpeed() compares against a speed the motors can reach
    targetRpm = rpm;
    // doShooterState ignores unchanged states, so re-send the velocity here.
    if (shooterState == ShooterState.AIM
        && !(Math.abs(rpm - sentRpm) <= TARGET_RESEND_RPM)) {
      setShooterRpm(targetRpm);
    }
  }

  public double getTargetRpm() {
    return targetRpm;
  }

  /** Fastest flywheel RPM allowed: MAX_OUTPUT of full speed. */
  public double getMaxRpm() {
    return ticksPerSecToRpm(shooterRight.getMaxVelocity());
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

  public boolean isAtSpeed() {
    double minSpeed = SHOOTER_VELOCITY;
    return Math.abs(shooterRight.getVelocity()) >= minSpeed
        && Math.abs(shooterLeft.getVelocity()) >= minSpeed;
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

  /** Samples both motors and cuts them if either stalls. See {@link ShooterStallGuard}. */
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
      setShooterPower(0.0);
      RobotLog.ee("BioBuzz", "shooter stalled, trip %d/%d: L %.1f A %.0f ticks/s, R %.1f A %.0f ticks/s",
          stallGuard.getTrips(), ShooterStallGuard.MAX_TRIPS,
          leftAmps, leftVelocity, rightAmps, rightVelocity);
    }
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
    Logger.recordOutput("Shooter/MaxRpm", getMaxRpm());
    Logger.recordOutput("Shooter/CommandedPower", shooterRight.getCommandedPower());
    Logger.recordOutput("Shooter/VelocityPidf", sentPidf.clone());
    Logger.recordOutput("Shooter/StallGuard/Tripped", stallGuard.isTripped());
    Logger.recordOutput("Shooter/StallGuard/Trips", stallGuard.getTrips());
    Logger.recordOutput("Shooter/StallGuard/LeftAmps", leftAmps);
    Logger.recordOutput("Shooter/StallGuard/RightAmps", rightAmps);
  }

  // Every shooter motor write goes through these two, so a stall trip cannot be overridden.
  private void setShooterPower(double power) {
    if (stallGuard.isTripped()) {
      power = 0.0;
    }
    sentRpm = Double.NaN;
    shooterRight.setPower(power);
    shooterLeft.setPower(power);
  }

  private void setShooterRpm(double rpm) {
    if (stallGuard.isTripped()) {
      setShooterPower(0.0);
      return;
    }
    sentRpm = rpm;
    double ticksPerSec = rpmToTicksPerSec(rpm);
    shooterRight.setVelocity(ticksPerSec);
    shooterLeft.setVelocity(ticksPerSec);
  }
}
