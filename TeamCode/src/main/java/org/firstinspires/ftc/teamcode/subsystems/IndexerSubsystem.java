package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.CrServo;
import org.psilynx.psikit.core.Logger;

/**
 * Controls the indexer, two Melonbotics Super Servos in continuous mode that
 * carry balls from the intake up to the feeder. It only runs on driver input;
 * there are no beam breaks.
 */
public class IndexerSubsystem extends SubsystemBase {
  // ---- Tuning ----------------------------------------------------------
  private static final double FEED_POWER = 1.0;
  private static final double EJECT_POWER = 1.0;
  // Small reverse power while intaking, so balls cannot creep into the shooter.
  private static final double HOLD_POWER = 0.15; // TODO tune

  // ---- Hardware --------------------------------------------------------
  private final CrServo indexerServoL;
  private final CrServo indexerServoR;

  public enum IndexerState {
    STOP, FEED, EJECT, HOLD
  }

  private IndexerState indexerState;
  private IndexerState previousIndexerState = IndexerState.STOP;
  // Gates FEED only (e.g. until the shooter is up to speed). EJECT is never gated.
  private boolean feedAllowed = true;

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
        setIndexerPower(feedAllowed ? FEED_POWER : 0.0);
        break;
      case EJECT:
        setIndexerPower(-EJECT_POWER);
        break;
      case HOLD:
        setIndexerPower(-HOLD_POWER);
        break;
    }
  }

  public void setIndexerState(IndexerState newIndexerState) {
    previousIndexerState = indexerState;
    indexerState = newIndexerState;
    doIndexerState();
  }

  /**
   * Allows or blocks feeding. Takes effect immediately even if already in FEED.
   */
  public void setFeedAllowed(boolean allowed) {
    boolean changed = feedAllowed != allowed;
    feedAllowed = allowed;
    // doIndexerState ignores unchanged states, so write the servo directly.
    if (changed && indexerState == IndexerState.FEED) {
      setIndexerPower(allowed ? FEED_POWER : 0.0);
    }
  }

  public boolean isFeedAllowed() {
    return feedAllowed;
  }

  public IndexerState getIndexerState() {
    return indexerState;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Indexer/State", indexerState);
    Logger.recordOutput("Indexer/FeedAllowed", feedAllowed);
    Logger.recordOutput("Indexer/CommandedPower", indexerServoL.getCommandedPower());
  }

  private void setIndexerPower(double power) {
    indexerServoL.setPower(power);
    indexerServoR.setPower(power);
  }
}
