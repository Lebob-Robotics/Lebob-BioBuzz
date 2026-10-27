package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.GobildaMotor;
import org.psilynx.psikit.core.Logger;

/**
 * Controls the two side-by-side shooter flywheels. Feeding balls into them is
 * done by {@link FeederSubsystem} and {@link IndexerSubsystem}.
 *
 * <ul>
 * <li>{@link ShooterState#IDLE} spins the flywheels a little slower (the
 * initial state), so they are close to shooting speed.</li>
 * <li>{@link ShooterState#SHOOT} spins them at full open-loop power.</li>
 * <li>{@link ShooterState#EJECT} runs them backward.</li>
 * <li>{@link ShooterState#STOP} stops them.</li>
 * </ul>
 *
 * {@link ShooterStallGuard} cuts both motors if either stalls, in any running
 * state. Once tripped the shooter stays off until the driver selects
 * {@link ShooterState#STOP}.
 */
public class ShooterSubsystem extends SubsystemBase {
  // ---- Tuning ----------------------------------------------------------
  private static final double SHOOTER_VELOCITY = 4000;
  private static final double IDLE_POWER = 0.8;
  private static final double SHOOT_POWER = 1.0;
  /** Current and velocity are separate hub reads, so sample them this often. */
  private static final double STALL_SAMPLE_PERIOD_S = 0.1;

  // ---- Hardware --------------------------------------------------------
  private final GobildaMotor shooterRight;
  private final GobildaMotor shooterLeft;

  public enum ShooterState {
    STOP, IDLE, SHOOT, EJECT
  }

  private ShooterState shooterState;
  private ShooterState previousShooterState = ShooterState.STOP;

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
      // Always open loop: RUN_USING_ENCODER would cap power 1.0 at 85% of the
      // motor's rated speed (about 5100 RPM on the 6000 RPM motors).
      case IDLE:
        setShooterPower(IDLE_POWER);
        break;
      case SHOOT:
        setShooterPower(SHOOT_POWER);
        break;
      case EJECT:
        setShooterPower(-1.0);
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
      setShooterPower(0.0);
      RobotLog.ee("BioBuzz", "shooter stalled, trip %d/%d: L %.1f A %.0f ticks/s, R %.1f A %.0f ticks/s",
          stallGuard.getTrips(), ShooterStallGuard.MAX_TRIPS,
          leftAmps, leftVelocity, rightAmps, rightVelocity);
    }
  }

  @Override
  public void periodic() {
    checkStall();
    Logger.recordOutput("Shooter/State", shooterState);
    Logger.recordOutput("Shooter/VelocityLeft", shooterLeft.getVelocity());
    Logger.recordOutput("Shooter/VelocityRight", shooterRight.getVelocity());
    Logger.recordOutput("Shooter/AtSpeed", isAtSpeed());
    Logger.recordOutput("Shooter/AtSpeedThreshold", SHOOTER_VELOCITY);
    Logger.recordOutput("Shooter/CommandedPower", shooterRight.getCommandedPower());
    Logger.recordOutput("Shooter/StallGuard/Tripped", stallGuard.isTripped());
    Logger.recordOutput("Shooter/StallGuard/Trips", stallGuard.getTrips());
    Logger.recordOutput("Shooter/StallGuard/LeftAmps", leftAmps);
    Logger.recordOutput("Shooter/StallGuard/RightAmps", rightAmps);
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
