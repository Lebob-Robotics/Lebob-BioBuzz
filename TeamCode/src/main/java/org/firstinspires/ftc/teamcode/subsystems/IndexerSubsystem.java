package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.CrServo;

/**
 * Controls the indexer, a single Melonbotics Super Servo in continuous mode
 * that sends balls from the feeder tunnel into the shooter.
 */
public class IndexerSubsystem extends SubsystemBase {
  // ---- Tuning ----------------------------------------------------------
  private static final double FEED_POWER = 1.0;
  private static final double EJECT_POWER = 1.0;

  // ---- Hardware --------------------------------------------------------
  private final CrServo indexerServo;

  public enum IndexerState {
    STOP, FEED, EJECT
  }

  private IndexerState indexerState;
  private IndexerState previousIndexerState = IndexerState.STOP;

  public IndexerSubsystem(HardwareMap hardwareMap) {
    indexerServo = new CrServo(hardwareMap, "IndexerServo", false);

    setIndexerState(IndexerState.STOP);
  }

  private void doIndexerState() {
    if (previousIndexerState == indexerState) {
      return;
    }

    switch (indexerState) {
      case STOP:
        indexerServo.stop();
        break;
      case FEED:
        indexerServo.setPower(FEED_POWER);
        break;
      case EJECT:
        indexerServo.setPower(-EJECT_POWER);
        break;
    }
  }

  public void setIndexerState(IndexerState newIndexerState) {
    previousIndexerState = indexerState;
    indexerState = newIndexerState;
    doIndexerState();
  }

  public IndexerState getIndexerState() {
    return indexerState;
  }
}
