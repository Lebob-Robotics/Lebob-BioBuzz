package org.firstinspires.ftc.teamcode.robot;

import static org.junit.Assert.*;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem.IndexerState;
import org.firstinspires.ftc.teamcode.subsystems.IntakeOverloadGuard;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem.IntakeState;
import org.junit.Test;
import org.firstinspires.ftc.teamcode.shooting.feed.StackPressure;

public class IndexerControlTest {
  @Test public void ltControlsLoadingEvenWhenIntakeProtectionIsLatched() {
    IntakeOverloadGuard intakeGuard = new IntakeOverloadGuard();
    intakeGuard.update(0, true, 9);
    assertTrue(intakeGuard.update(0.3, true, 9));
    assertEquals(IndexerState.LOAD, BallPath.indexerCommand(false, false, false, 1, false, false));
    assertEquals(IndexerState.STOP, BallPath.indexerCommand(false, false, false, 0, false, false));
    assertTrue(intakeGuard.isTripped());
  }

  @Test public void automaticIntakeDoesNotRunIndexerWithoutLt() {
    assertEquals(IndexerState.STOP, BallPath.indexerCommand(false, false, false, 0, false, false));
    assertEquals(IndexerState.STOP, BallPath.indexerCommand(false, false, false, Robot.TRIGGER_PRESSED, false, false));
    assertEquals(IndexerState.LOAD, BallPath.indexerCommand(false, false, false, Robot.TRIGGER_PRESSED + 0.01, false, false));
  }

  @Test public void waitingShotStagesUpstreamWithoutFeedingLastRoller() {
    assertEquals(IndexerState.STAGE, BallPath.indexerCommand(false, true, false, 1, false, false));
    assertEquals(IndexerState.FEED, BallPath.indexerCommand(false, true, true, 0, false, false));
  }

  @Test public void ejectRemainsAvailableAndWinsOverShooting() {
    assertEquals(IndexerState.EJECT, BallPath.indexerCommand(true, true, false, 1, false, false));
    assertEquals(IndexerState.EJECT, BallPath.indexerCommand(true, false, false, 0, false, false));
  }
  @Test public void ltCanLoadWhileScriptedAimInhibitsFire() {
    assertEquals(IndexerState.LOAD, BallPath.indexerCommand(false, true, false, 1, true, false));
    assertEquals(IndexerState.STOP, BallPath.indexerCommand(false, true, false, 0, true, false));
    assertEquals(IndexerState.STAGE, BallPath.indexerCommand(false, true, false, 1, false, false));
    assertEquals(IndexerState.FEED, BallPath.indexerCommand(false, true, true, 1, false, false));
    assertEquals(IndexerState.EJECT, BallPath.indexerCommand(true, true, false, 1, true, false));
  }

  @Test public void fullCapacityStopsLoadingButAllowsShootingAndEject() {
    assertEquals(IndexerState.STOP, BallPath.indexerCommand(false, false, false, 1, true, true));
    assertEquals(IndexerState.FEED, BallPath.indexerCommand(false, true, true, 0, false, true));
    assertEquals(IndexerState.EJECT, BallPath.indexerCommand(true, false, false, 1, false, true));
  }

  @Test public void volleyRecoveryAdvancesTheUpstreamQueueWithoutLt() {
    assertEquals(IndexerState.STAGE, BallPath.indexerCommand(false, true, false, 0, false, false));
    assertEquals(IndexerState.STOP, BallPath.indexerCommand(false, true, false, 0, true, false));
    assertEquals(IndexerState.STOP, BallPath.indexerCommand(false, false, false, 0, false, false));
  }


  @Test public void ltLoadingEasesTheUpstreamRollers() {
    double eased = DriverConstants.LOAD_UPSTREAM_POWER;
    assertEquals(eased, BallPath.loadingIndexerPower(IndexerState.LOAD, true, false, 1.0), 1e-9);
    assertEquals(eased, BallPath.loadingIndexerPower(IndexerState.STAGE, true, false, 1.0), 1e-9); // Packed
    assertEquals(1.0, BallPath.loadingIndexerPower(IndexerState.LOAD, true, true, 1.0), 1e-9); // Auto
    assertEquals(1.0, BallPath.loadingIndexerPower(IndexerState.STAGE, false, false, 1.0), 1e-9); // Shot staging
    assertEquals(1.0, BallPath.loadingIndexerPower(IndexerState.EJECT, true, false, 1.0), 1e-9);
  }

  @Test public void packedStackLoadsWithTheLastRollerStopped() {
    assertEquals(IndexerState.STAGE, BallPath.packedLoad(IndexerState.LOAD, true));
    assertEquals(IndexerState.LOAD, BallPath.packedLoad(IndexerState.LOAD, false));
    assertEquals(IndexerState.FEED, BallPath.packedLoad(IndexerState.FEED, true)); // Shots and eject unchanged
    assertEquals(IndexerState.EJECT, BallPath.packedLoad(IndexerState.EJECT, true));
  }

  @Test public void trippedIntakeRumblesUntilReset() {
    assertEquals(0, DriverPad.intakeLockRumbleMs(false, 5, Double.NaN));
    assertEquals(DriverPad.INTAKE_LOCK_RUMBLE_MS, DriverPad.intakeLockRumbleMs(true, 5, Double.NaN)); // Trips
    assertEquals(0, DriverPad.intakeLockRumbleMs(true, 6, 5));
    assertEquals(DriverPad.INTAKE_LOCK_REMIND_MS, DriverPad.intakeLockRumbleMs(true, 5 + DriverPad.INTAKE_LOCK_REMIND_S, 5));
    assertEquals(0, DriverPad.intakeLockRumbleMs(false, 9, 5)); // LB reset it
  }

  @Test public void tappingLtKeepsAPackedStackEased() {
    double rearm = StackPressure.REARM_RELEASE_S;
    assertFalse("A 0.1 s tap", BallPath.loadingLatchesClear(true, 0.1, false, false, false));
    assertTrue("A real let-go", BallPath.loadingLatchesClear(true, rearm, false, false, false));
    assertFalse("Not with the wheels spinning: the stack would be pushed into them",
        BallPath.loadingLatchesClear(true, rearm + 5, false, false, true));
    assertFalse("Still held", BallPath.loadingLatchesClear(false, 5, false, false, false));
    assertTrue("B", BallPath.loadingLatchesClear(false, 0, true, false, true));
    assertTrue("A ball out", BallPath.loadingLatchesClear(false, 0, false, true, true));
  }

  @Test public void theWedgeForcesBackwardOnlyWhereTheBackwardTableWasShot() {
    assertTrue("230333's 0.90 m volley", AimSelector.wedgeForcesBackward(true, 0.90));
    assertTrue(AimSelector.wedgeForcesBackward(true, 0.41));
    assertFalse("against the rear wall, 1.05 m: as before", AimSelector.wedgeForcesBackward(true, 1.05));
    assertFalse(AimSelector.wedgeForcesBackward(true, Double.NaN));
    assertFalse(AimSelector.wedgeForcesBackward(false, 0.60));
  }

  @Test public void theIntakesEaseEndsOnARealLtPressEvenWithTheWheelsSpinning() {
    double rearm = StackPressure.REARM_RELEASE_S;
    assertTrue("A real let-go", BallPath.intakeEaseClears(true, rearm, false));
    assertFalse("A 0.1 s tap", BallPath.intakeEaseClears(true, 0.1, false));
    assertFalse("Still held, pushing", BallPath.intakeEaseClears(false, 5, false));
    assertTrue("Running free", BallPath.intakeEaseClears(false, 0, true));
  }

  @Test public void theAutoStopsItsIntakeWhenFullInsteadOfEjecting() {
    IntakeState auto = BallPath.intakeCommand(true, false, true, true);
    assertEquals(IntakeState.STOP, auto);
    assertEquals(IntakeState.EJECT, BallPath.intakeCommand(true, false, true, false));
    assertEquals(IntakeState.EJECT, BallPath.intakeCommand(true, true, true, true));
    assertEquals(IntakeState.INTAKE, BallPath.intakeCommand(false, false, true, true));
  }

  @Test
  public void theAutoKeepsBringingBallsToTheFeederBetweenShots() {
    assertEquals(IndexerState.LOAD, BallPath.autoStage(IndexerState.STAGE, true));
    assertEquals(IndexerState.FEED, BallPath.autoStage(IndexerState.FEED, true));
    assertEquals(IndexerState.STOP, BallPath.autoStage(IndexerState.STOP, true));
    assertEquals("TeleOp stages as before", IndexerState.STAGE, BallPath.autoStage(IndexerState.STAGE, false));
  }

  @Test
  public void agitationPausesOnlyLoading() {
    assertEquals(IndexerState.STOP, BallPath.agitated(IndexerState.LOAD, true));
    assertEquals(IndexerState.LOAD, BallPath.agitated(IndexerState.LOAD, false));
    assertEquals(IndexerState.FEED, BallPath.agitated(IndexerState.FEED, true));
    assertEquals(IndexerState.EJECT, BallPath.agitated(IndexerState.EJECT, true));
    assertEquals("the jiggle backs the stack off", IndexerState.EJECT, BallPath.agitated(IndexerState.LOAD, true, true));
    assertEquals(IndexerState.LOAD, BallPath.agitated(IndexerState.LOAD, false, true));
    assertEquals(IndexerState.FEED, BallPath.agitated(IndexerState.FEED, true, true));
    assertEquals(IndexerState.STOP, BallPath.agitated(IndexerState.STOP, true, true));
  }
}
