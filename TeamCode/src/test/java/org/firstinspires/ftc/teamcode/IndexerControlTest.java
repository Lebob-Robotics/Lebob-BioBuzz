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
    assertEquals(IndexerState.FEED, Robot.indexerCommand(false, false, false, 1));
    assertEquals(IndexerState.STOP, Robot.indexerCommand(false, false, false, 0));
    assertTrue(intakeGuard.isTripped());
  }

  @Test public void automaticIntakeDoesNotRunIndexerWithoutLt() {
    assertEquals(IndexerState.STOP, Robot.indexerCommand(false, false, false, 0));
    assertEquals(IndexerState.STOP, Robot.indexerCommand(false, false, false, Robot.TRIGGER_PRESSED));
    assertEquals(IndexerState.FEED, Robot.indexerCommand(false, false, false, Robot.TRIGGER_PRESSED + 0.01));
  }

  @Test public void ltCannotPushThroughAnUnreadyAimedShot() {
    assertEquals(IndexerState.STOP, Robot.indexerCommand(false, true, false, 1));
    assertEquals(IndexerState.FEED, Robot.indexerCommand(false, true, true, 0));
  }

  @Test public void ejectRemainsAvailableAndWinsOverShooting() {
    assertEquals(IndexerState.EJECT, Robot.indexerCommand(true, true, false, 1));
    assertEquals(IndexerState.EJECT, Robot.indexerCommand(true, false, false, 0));
  }
}
