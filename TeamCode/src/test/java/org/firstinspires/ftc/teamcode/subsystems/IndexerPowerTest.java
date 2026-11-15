package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.*;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem.IndexerState;
import org.junit.Test;

public class IndexerPowerTest {
  @Test public void loadingAdvancesLastRollerGentlyRatherThanLockingBallsAway() {
    assertArrayEquals(new double[] {1, .3}, IndexerSubsystem.powersFor(IndexerState.LOAD, 1), 0);
    assertArrayEquals(new double[] {.15, .15}, IndexerSubsystem.powersFor(IndexerState.LOAD, .15), 0);
    assertArrayEquals(new double[] {0, 0}, IndexerSubsystem.powersFor(IndexerState.LOAD, 0), 0);
  }

  @Test public void waitingShotCanStageUpstreamWithoutDrivingIntoTheWheels() {
    assertArrayEquals(new double[] {1, 0}, IndexerSubsystem.powersFor(IndexerState.STAGE, 1), 0);
  }

  @Test public void authorizedFeedRunsBothStagesAtTheVolleyPower() {
    assertArrayEquals(new double[] {.4, .4}, IndexerSubsystem.powersFor(IndexerState.FEED, .4), 0);
  }

  @Test public void stoppingAndEjectingStillApplyToBothStages() {
    assertArrayEquals(new double[] {0, 0}, IndexerSubsystem.powersFor(IndexerState.STOP, 1), 0);
    assertArrayEquals(new double[] {-1, -1}, IndexerSubsystem.powersFor(IndexerState.EJECT, .4), 0);
  }

}
