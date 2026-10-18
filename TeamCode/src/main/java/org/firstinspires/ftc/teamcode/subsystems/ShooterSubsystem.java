package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.servo.GobildaMotor;
import org.psilynx.psikit.core.Logger;

import java.util.Arrays;

/**
 * Controls the two side-by-side shooter flywheels. Feeding balls into them is
 * done by {@link FeederSubsystem} and {@link IndexerSubsystem}.
 *
 * <ul>
 * <li>{@link ShooterState#IDLE} spins the flywheels (the initial state).</li>
 * <li>{@link ShooterState#SHOOT} spins them at full open-loop power.</li>
 * <li>{@link ShooterState#EJECT} runs them backward.</li>
 * <li>{@link ShooterState#AIM} holds {@link #setTargetRpm(double)} in closed loop.</li>
 * <li>{@link ShooterState#STOP} stops them.</li>
 * </ul>
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
  private DcMotor.RunMode shooterMode = DcMotor.RunMode.RUN_USING_ENCODER;
  // Last PIDF sent to the motors, {p, i, d, f}.
  private final double[] sentPidf = {Double.NaN, Double.NaN, Double.NaN, Double.NaN};

  public ShooterSubsystem(HardwareMap hardwareMap) {
    shooterRight = new GobildaMotor(hardwareMap, "ShooterR", false, DcMotor.RunMode.RUN_USING_ENCODER, false);
    // Side-by-side wheels spin opposite ways, so one is reversed. Direction is
    // to be verified on the robot.
    shooterLeft = new GobildaMotor(hardwareMap, "ShooterL", true, DcMotor.RunMode.RUN_USING_ENCODER, false);

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
        setShooterMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setShooterPower(1.0);
        break;
      case SHOOT:
        // RUN_USING_ENCODER holds power 1.0 to 85% of the motor's rated speed
        // (about 5100 RPM on the 6000 RPM motors); open loop gives full voltage.
        setShooterMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setShooterPower(1.0);
        break;
      case EJECT:
        setShooterMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setShooterPower(-1.0);
        break;
      case AIM:
        setShooterMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sendTargetVelocity();
        break;
    }
  }

  public void setShooterState(ShooterState newShooterState) {
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

  @Override
  public void periodic() {
    applyVelocityPidf();

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
  }

  private void setShooterMode(DcMotor.RunMode mode) {
    shooterMode = mode;
    shooterRight.setMode(mode);
    shooterLeft.setMode(mode);
  }

  private void setShooterPower(double power) {
    shooterRight.setPower(power);
    shooterLeft.setPower(power);
  }
}
