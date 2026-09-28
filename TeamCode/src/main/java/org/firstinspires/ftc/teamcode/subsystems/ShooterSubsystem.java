package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.GobildaMotor;

/**
 * Controls the two-wheel shooter and the indexer that feeds balls into it.
 *
 * <ul>
 * <li>{@link #init()} spins the shooter until {@link #stop()} or
 * {@link #eject()}.</li>
 * <li>{@link #shoot()} runs the indexer once the shooter is at speed
 * (spinning it up first if needed).</li>
 * <li>{@link #eject()} runs shooter and indexer backward.</li>
 * <li>{@link #stop()} stops everything.</li>
 * </ul>
 *
 * The "wait until at speed" step is non-blocking: it is handled in
 * {@link #periodic()}, which the CommandScheduler calls every loop, so it never
 * stalls the OpMode loop.
 */
public class ShooterSubsystem extends SubsystemBase {
  // ---- Tuning ----------------------------------------------------------
  private static final double SHOOTER_VELOCITY = 4000;
  private static final double INDEXER_POWER = 1.0;
  private static final double EJECT_INDEXER_POWER = 0.6;

  // ---- Hardware --------------------------------------------------------
  private final GobildaMotor indexerMotor;
  private final GobildaMotor shooterFrontMotor;
  private final GobildaMotor shooterBackMotor;

  public enum ShooterState {
    STOP, IDLE, SHOOT, EJECT
  }

  private ShooterState shooterState;
  private ShooterState previousShooterState = ShooterState.STOP;

  public ShooterSubsystem(HardwareMap hardwareMap) {
    indexerMotor = new GobildaMotor(hardwareMap, "Indexer", false, DcMotor.RunMode.RUN_USING_ENCODER, true);
    shooterFrontMotor = new GobildaMotor(hardwareMap, "ShooterF", false, DcMotor.RunMode.RUN_USING_ENCODER, false);
    shooterBackMotor = new GobildaMotor(hardwareMap, "ShooterB", false, DcMotor.RunMode.RUN_USING_ENCODER, false);

    setShooterState(ShooterState.IDLE);
  }

  private void doShooterState() {
    if (previousShooterState == shooterState) {
      return;
    }

    switch (shooterState) {
      case STOP:
        indexerMotor.stop();
        setShooterPower(0.0);
        break;
      case IDLE:
        indexerMotor.stop();
        setShooterPower(1.0);
        break;
      case SHOOT:
        indexerMotor.setPower(1.0);
        setShooterPower(1.0);
        break;
      case EJECT:
        indexerMotor.setPower(-1.0);
        setShooterPower(-1.0);
        break;
    }
  }

  public void setShooterState(ShooterState newShooterState) {
    previousShooterState = shooterState;
    shooterState = newShooterState;
    doShooterState();
  }

  //
  // @Override
  // public void periodic() {
  // doShooterState();
  // }
  //
  public boolean isAtSpeed() {
    double minSpeed = SHOOTER_VELOCITY;
    return Math.abs(shooterFrontMotor.getVelocity()) >= minSpeed
        && Math.abs(shooterBackMotor.getVelocity()) >= minSpeed;
  }

  private void setShooterPower(double ticksPerSecond) {
    shooterFrontMotor.setPower(ticksPerSecond);
    shooterBackMotor.setPower(ticksPerSecond);
  }
}
