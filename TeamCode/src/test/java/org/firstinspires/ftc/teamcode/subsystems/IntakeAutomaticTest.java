package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem.IntakeState;
import org.junit.Test;

public class IntakeAutomaticTest {
  @Test public void collectsWhenNotFullAndImmediatelyRequestsCollectionAgainAfterFullClears() {
    assertEquals(IntakeState.INTAKE, IntakeSubsystem.automaticState(false, false));
    assertEquals(IntakeState.EJECT, IntakeSubsystem.automaticState(true, false));
    assertEquals(IntakeState.INTAKE, IntakeSubsystem.automaticState(false, false));
  }
  @Test public void manualEjectOverridesCollectionAtAnyCapacity() {
    assertEquals(IntakeState.EJECT, IntakeSubsystem.automaticState(false, true));
    assertEquals(IntakeState.EJECT, IntakeSubsystem.automaticState(true, true));
  }
}
