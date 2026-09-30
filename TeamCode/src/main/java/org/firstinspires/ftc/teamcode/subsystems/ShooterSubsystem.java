package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.hardware.GobildaMotor;
import org.firstinspires.ftc.teamcode.subsystems.IndexerStallGuard.Phase;

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
  private static final double UNJAM_INDEXER_POWER = 0.6;
  /** Current and velocity are separate hub reads, so sample them this often. */
  private static final double STALL_SAMPLE_PERIOD_S = 0.05;

  // ---- Hardware --------------------------------------------------------
  private final GobildaMotor indexerMotor;
  private final GobildaMotor shooterFrontMotor;
  private final GobildaMotor shooterBackMotor;

  public enum ShooterState {
    STOP, IDLE, SHOOT, EJECT
  }

  private ShooterState shooterState;
  private ShooterState previousShooterState = ShooterState.STOP;

  // ---- Stall protection ------------------------------------------------
  private final IndexerStallGuard indexerStallGuard = new IndexerStallGuard(INDEXER_POWER, -UNJAM_INDEXER_POWER);
  private final ElapsedTime clock = new ElapsedTime();
  private double lastStallSampleS = Double.NEGATIVE_INFINITY;
  private double indexerCurrentAmps;
  private double indexerVelocity;

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
        setShooterMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setShooterPower(1.0);
        break;
      case SHOOT:
        indexerStallGuard.start(clock.seconds());
        indexerMotor.setPower(indexerStallGuard.getPower());
        // RUN_USING_ENCODER holds power 1.0 to 85% of the motor's rated speed
        // (about 5100 RPM on the 6000 RPM motors); open loop gives full voltage.
        setShooterMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setShooterPower(1.0);
        break;
      case EJECT:
        indexerMotor.setPower(-1.0);
        setShooterMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setShooterPower(-1.0);
        break;
    }
  }

  public void setShooterState(ShooterState newShooterState) {
    previousShooterState = shooterState;
    shooterState = newShooterState;
    doShooterState();
  }

  /**
   * While shooting, watches the indexer for a jam and backs it off. See
   * {@link IndexerStallGuard}.
   */
  @Override
  public void periodic() {
    if (shooterState != ShooterState.SHOOT) {
      return;
    }
    double now = clock.seconds();
    if (now - lastStallSampleS < STALL_SAMPLE_PERIOD_S) {
      return;
    }
    lastStallSampleS = now;

    indexerCurrentAmps = indexerMotor.getCurrentAmps();
    indexerVelocity = indexerMotor.getVelocity();
    Phase before = indexerStallGuard.getPhase();
    indexerMotor.setPower(indexerStallGuard.update(now, indexerCurrentAmps, indexerVelocity));
    if (indexerStallGuard.getPhase() != before) {
      RobotLog.dd("BioBuzz", "indexer %s -> %s at %.1f A, %.0f ticks/s, unjam %d/%d",
          before, indexerStallGuard.getPhase(), indexerCurrentAmps, indexerVelocity,
          indexerStallGuard.getUnjamAttempts(), IndexerStallGuard.MAX_UNJAM_ATTEMPTS);
    }
  }

  /** True while shooting with the indexer feeding, not backing off a jam. */
  public boolean isIndexerFeeding() {
    return shooterState == ShooterState.SHOOT && indexerStallGuard.getPhase() == Phase.FEEDING;
  }

  /** True once the indexer has given up on a jam; clears when shooting restarts. */
  public boolean isIndexerJammed() {
    return shooterState == ShooterState.SHOOT && indexerStallGuard.getPhase() == Phase.JAMMED;
  }

  /** Indexer feed state and the last current and velocity reading, for telemetry. */
  public String getIndexerStatus() {
    if (shooterState != ShooterState.SHOOT) {
      return shooterState.toString();
    }
    return String.format("%s  %.1f A  %.0f ticks/s  unjam %d/%d",
        indexerStallGuard.getPhase(), indexerCurrentAmps, indexerVelocity,
        indexerStallGuard.getUnjamAttempts(), IndexerStallGuard.MAX_UNJAM_ATTEMPTS);
  }

  public boolean isAtSpeed() {
    double minSpeed = SHOOTER_VELOCITY;
    return Math.abs(shooterFrontMotor.getVelocity()) >= minSpeed
        && Math.abs(shooterBackMotor.getVelocity()) >= minSpeed;
  }

  private void setShooterMode(DcMotor.RunMode mode) {
    shooterFrontMotor.setMode(mode);
    shooterBackMotor.setMode(mode);
  }

  private void setShooterPower(double ticksPerSecond) {
    shooterFrontMotor.setPower(ticksPerSecond);
    shooterBackMotor.setPower(ticksPerSecond);
  }
}
