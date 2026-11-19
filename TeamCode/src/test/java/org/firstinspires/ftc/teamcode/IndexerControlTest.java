package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.*;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem.IndexerState;
import org.firstinspires.ftc.teamcode.subsystems.IntakeOverloadGuard;
import org.junit.Test;

public class IndexerControlTest {
  @Test public void ltControlsLoadingEvenWhenIntakeProtectionIsLatched() {
    IntakeOverloadGuard intakeGuard = new IntakeOverloadGuard();
    intakeGuard.update(0, true, 9);
    assertTrue(intakeGuard.update(0.3, true, 9));
    assertEquals(IndexerState.LOAD, Robot.indexerCommand(false, false, false, 1, false, false));
    assertEquals(IndexerState.STOP, Robot.indexerCommand(false, false, false, 0, false, false));
    assertTrue(intakeGuard.isTripped());
  }

  @Test public void automaticIntakeDoesNotRunIndexerWithoutLt() {
    assertEquals(IndexerState.STOP, Robot.indexerCommand(false, false, false, 0, false, false));
    assertEquals(IndexerState.STOP, Robot.indexerCommand(false, false, false, Robot.TRIGGER_PRESSED, false, false));
    assertEquals(IndexerState.LOAD, Robot.indexerCommand(false, false, false, Robot.TRIGGER_PRESSED + 0.01, false, false));
  }

  @Test public void waitingShotStagesUpstreamWithoutFeedingLastRoller() {
    assertEquals(IndexerState.STAGE, Robot.indexerCommand(false, true, false, 1, false, false));
    assertEquals(IndexerState.FEED, Robot.indexerCommand(false, true, true, 0, false, false));
  }

  @Test public void ejectRemainsAvailableAndWinsOverShooting() {
    assertEquals(IndexerState.EJECT, Robot.indexerCommand(true, true, false, 1, false, false));
    assertEquals(IndexerState.EJECT, Robot.indexerCommand(true, false, false, 0, false, false));
  }
  @Test public void ltCanLoadWhileScriptedAimInhibitsFire() {
    assertEquals(IndexerState.LOAD, Robot.indexerCommand(false, true, false, 1, true, false));
    assertEquals(IndexerState.STOP, Robot.indexerCommand(false, true, false, 0, true, false));
    assertEquals(IndexerState.STAGE, Robot.indexerCommand(false, true, false, 1, false, false));
    assertEquals(IndexerState.FEED, Robot.indexerCommand(false, true, true, 1, false, false));
    assertEquals(IndexerState.EJECT, Robot.indexerCommand(true, true, false, 1, true, false));
  }

  @Test public void fullCapacityStopsLoadingButAllowsShootingAndEject() {
    assertEquals(IndexerState.STOP, Robot.indexerCommand(false, false, false, 1, true, true));
    assertEquals(IndexerState.FEED, Robot.indexerCommand(false, true, true, 0, false, true));
    assertEquals(IndexerState.EJECT, Robot.indexerCommand(true, false, false, 1, false, true));
  }

  @Test public void volleyRecoveryAdvancesTheUpstreamQueueWithoutLt() {
    assertEquals(IndexerState.STAGE, Robot.indexerCommand(false, true, false, 0, false, false));
    assertEquals(IndexerState.STOP, Robot.indexerCommand(false, true, false, 0, true, false));
    assertEquals(IndexerState.STOP, Robot.indexerCommand(false, false, false, 0, false, false));
  }

}
