package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.CrServo;
import org.psilynx.psikit.core.Logger;

/**
 * Controls two independent indexer stages. IndexerServoL on Expansion Hub port 0 drives
 * the upstream rollers; IndexerServoR on Control Hub port 0 drives the last roller.
 * Loading advances the last roller gently against the backward-held feeder until capacity.
 * Between shot feeds the upstream stage keeps indexing while the last roller holds still.
 */
public class IndexerSubsystem extends SubsystemBase {
  // ---- Tuning ----------------------------------------------------------
  private static final double FEED_POWER = 1.0;
  private double feedPower = FEED_POWER; // A volley's (Lebot Teleop RB) can be lower
  private static final double EJECT_POWER = 1.0;
  private static final double LAST_ROLLER_LOAD_POWER = 0.30;

  // ---- Hardware --------------------------------------------------------
  private final CrServo indexerServoL;
  private final CrServo indexerServoR;

  public enum IndexerState {
    STOP, LOAD, STAGE, FEED, EJECT
  }

  private IndexerState indexerState;

  public IndexerSubsystem(HardwareMap hardwareMap) {
    // Independent stages, retaining their verified forward directions.
    indexerServoL = new CrServo(hardwareMap, "IndexerServoL", true);
    indexerServoR = new CrServo(hardwareMap, "IndexerServoR", false);

    setIndexerState(IndexerState.STOP);
  }

  /** Logical {upstream, last roller} powers, separating loading from shot staging and feeding. */
  static double[] powersFor(IndexerState state, double power) {
    double feed = Math.max(0, Math.min(1, power));
    switch (state) {
      case LOAD:
        return new double[] {feed, Math.min(feed, LAST_ROLLER_LOAD_POWER)};
      case STAGE:
        return new double[] {feed, 0.0};
      case STOP:
        return new double[] {0.0, 0.0};
      case FEED:
        return new double[] {feed, feed};
      case EJECT:
        return new double[] {-EJECT_POWER, -EJECT_POWER};
      default:
        throw new IllegalArgumentException("Unknown indexer state " + state);
    }
  }

  public void setIndexerState(IndexerState newIndexerState) {
    setIndexerState(newIndexerState, feedPower);
  }

  /** Apply state and power together: no brief forward pulse on release when power changes. */
  public void setIndexerState(IndexerState newIndexerState, double power) {
    double p = Math.max(0, Math.min(1, power));
    if (indexerState == newIndexerState && feedPower == p) { return; }
    indexerState = newIndexerState;
    feedPower = p;
    double[] powers = powersFor(indexerState, feedPower);
    indexerServoL.setPower(powers[0]);
    indexerServoR.setPower(powers[1]);
  }

  /** Forward power, 0-1. Shot staging keeps the last roller stopped; loading caps its power. */
  public void setFeedPower(double power) {
    setIndexerState(indexerState, power);
  }

  public IndexerState getIndexerState() {
    return indexerState;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Indexer/State", indexerState);
    Logger.recordOutput("Indexer/CommandedPower", indexerServoL.getCommandedPower());
    Logger.recordOutput("Indexer/UpstreamPower", indexerServoL.getCommandedPower());
    Logger.recordOutput("Indexer/LastRollerPower", indexerServoR.getCommandedPower());
  }
}
