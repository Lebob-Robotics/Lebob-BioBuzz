package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.*;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem.IndexerState;
import org.firstinspires.ftc.teamcode.subsystems.IntakeOverloadGuard;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem.IntakeState;
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

  @Test public void tappingLtKeepsAPackedStackEased() {
    double rearm = org.firstinspires.ftc.teamcode.utils.StackPressure.REARM_RELEASE_S;
    assertFalse("A 0.1 s tap", Robot.loadingLatchesClear(true, 0.1, false, false, false));
    assertTrue("A real let-go", Robot.loadingLatchesClear(true, rearm, false, false, false));
    assertFalse("Not with the wheels spinning: the stack would be pushed into them",
        Robot.loadingLatchesClear(true, rearm + 5, false, false, true));
    assertFalse("Still held", Robot.loadingLatchesClear(false, 5, false, false, false));
    assertTrue("B", Robot.loadingLatchesClear(false, 0, true, false, true));
    assertTrue("A ball out", Robot.loadingLatchesClear(false, 0, false, true, true));
  }

  @Test public void theWedgeForcesBackwardOnlyWhereTheBackwardTableWasShot() {
    assertTrue("230333's 0.90 m volley", Robot.wedgeForcesBackward(true, 0.90));
    assertTrue(Robot.wedgeForcesBackward(true, 0.41));
    assertFalse("against the rear wall, 1.05 m: as before", Robot.wedgeForcesBackward(true, 1.05));
    assertFalse(Robot.wedgeForcesBackward(true, Double.NaN));
    assertFalse(Robot.wedgeForcesBackward(false, 0.60));
  }

  @Test public void theIntakesEaseEndsOnARealLtPressEvenWithTheWheelsSpinning() {
    double rearm = org.firstinspires.ftc.teamcode.utils.StackPressure.REARM_RELEASE_S;
    assertTrue("A real let-go", Robot.intakeEaseClears(true, rearm, false));
    assertFalse("A 0.1 s tap", Robot.intakeEaseClears(true, 0.1, false));
    assertFalse("Still held, pushing", Robot.intakeEaseClears(false, 5, false));
    assertTrue("Running free", Robot.intakeEaseClears(false, 0, true));
  }

  @Test public void theAutoStopsItsIntakeWhenFullInsteadOfEjecting() {
    IntakeState auto = Robot.intakeCommand(true, false, true, true);
    assertEquals(IntakeState.STOP, auto);
    assertEquals(IntakeState.EJECT, Robot.intakeCommand(true, false, true, false));
    assertEquals(IntakeState.EJECT, Robot.intakeCommand(true, true, true, true));
    assertEquals(IntakeState.INTAKE, Robot.intakeCommand(false, false, true, true));
  }

  @Test
  public void theAutoKeepsBringingBallsToTheFeederBetweenShots() {
    assertEquals(IndexerState.LOAD, Robot.autoStage(IndexerState.STAGE, true));
    assertEquals(IndexerState.FEED, Robot.autoStage(IndexerState.FEED, true));
    assertEquals(IndexerState.STOP, Robot.autoStage(IndexerState.STOP, true));
    assertEquals("TeleOp stages as before", IndexerState.STAGE, Robot.autoStage(IndexerState.STAGE, false));
  }
}
