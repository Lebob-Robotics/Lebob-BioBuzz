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


  @Test public void packedStackLoadsWithTheLastRollerStopped() {
    assertEquals(IndexerState.STAGE, Robot.packedLoad(IndexerState.LOAD, true));
    assertEquals(IndexerState.LOAD, Robot.packedLoad(IndexerState.LOAD, false));
    assertEquals(IndexerState.FEED, Robot.packedLoad(IndexerState.FEED, true)); // Shots and eject unchanged
    assertEquals(IndexerState.EJECT, Robot.packedLoad(IndexerState.EJECT, true));
  }

  @Test public void trippedIntakeRumblesUntilReset() {
    assertEquals(0, Robot.intakeLockRumbleMs(false, 5, Double.NaN));
    assertEquals(Robot.INTAKE_LOCK_RUMBLE_MS, Robot.intakeLockRumbleMs(true, 5, Double.NaN)); // Trips
    assertEquals(0, Robot.intakeLockRumbleMs(true, 6, 5));
    assertEquals(Robot.INTAKE_LOCK_REMIND_MS, Robot.intakeLockRumbleMs(true, 5 + Robot.INTAKE_LOCK_REMIND_S, 5));
    assertEquals(0, Robot.intakeLockRumbleMs(false, 9, 5)); // LB reset it
  }
}
