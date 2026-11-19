package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.CrServo;
import org.psilynx.psikit.core.Logger;

/**
 * Controls the indexer, two Melonbotics Super Servos in continuous mode that
 * carry balls from the intake up to the feeder. It only runs on driver input;
 * there are no beam breaks.
 */
public class IndexerSubsystem extends SubsystemBase {
  // ---- Tuning ----------------------------------------------------------
  private static final double FEED_POWER = 1.0;
  private double feedPower = FEED_POWER; // Lebot Teleop Volley's can be lower
  private static final double EJECT_POWER = 1.0;

  // ---- Hardware --------------------------------------------------------
  private final CrServo indexerServoL;
  private final CrServo indexerServoR;

  public enum IndexerState {
    STOP, FEED, EJECT
  }

  private IndexerState indexerState;
  private IndexerState previousIndexerState = IndexerState.STOP;

  public IndexerSubsystem(HardwareMap hardwareMap) {
    // The servos face each other, so one is reversed. Direction is to be
    // verified on the robot.
    indexerServoL = new CrServo(hardwareMap, "IndexerServoL", true);
    indexerServoR = new CrServo(hardwareMap, "IndexerServoR", false);

    setIndexerState(IndexerState.STOP);
  }

  private void doIndexerState() {
    if (previousIndexerState == indexerState) {
      return;
    }

    switch (indexerState) {
      case STOP:
        setIndexerPower(0.0);
        break;
      case FEED:
        setIndexerPower(feedPower);
        break;
      case EJECT:
        setIndexerPower(-EJECT_POWER);
        break;
    }
  }

  public void setIndexerState(IndexerState newIndexerState) {
    previousIndexerState = indexerState;
    indexerState = newIndexerState;
    doIndexerState();
  }

  /** Power while feeding, 0-1; the default is full. Applies at once if feeding now. */
  public void setFeedPower(double power) {
    double p = Math.max(0, Math.min(1, power));
    if (p == feedPower) {
      return;
    }
    feedPower = p;
    if (indexerState == IndexerState.FEED) {
      setIndexerPower(feedPower);
    }
  }

  public IndexerState getIndexerState() {
    return indexerState;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Indexer/State", indexerState);
    Logger.recordOutput("Indexer/CommandedPower", indexerServoL.getCommandedPower());
  }

  private void setIndexerPower(double power) {
    indexerServoL.setPower(power);
    indexerServoR.setPower(power);
  }
}
