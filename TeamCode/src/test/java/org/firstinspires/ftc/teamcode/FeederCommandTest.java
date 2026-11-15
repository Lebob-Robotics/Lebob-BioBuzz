package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem.FeederState;
import org.junit.Test;

public class FeederCommandTest {
  @Test
  public void anEmptyFeederAimedButNotFiringHoldsTheStackBack() {
    // The auto aims all the time; after an empty volley the indexer stages the next pickup at full power
    assertEquals(FeederState.HOLD, Robot.feederCommand(false, false, true, false, false, true, false));
  }

  @Test
  public void firingStillStopsWhenEmptyOrBetweenAVolleysBalls() {
    assertEquals(FeederState.STOP, Robot.feederCommand(false, false, true, false, true, true, false));
    assertEquals(FeederState.STOP, Robot.feederCommand(false, false, true, false, true, false, true));
    assertEquals(FeederState.HOLD, Robot.feederCommand(false, false, true, false, true, false, false));
    assertEquals(FeederState.FEED, Robot.feederCommand(false, false, true, true, true, true, false));
  }

  @Test
  public void loadingHoldsAndEjectingEjects() {
    assertEquals(FeederState.HOLD, Robot.feederCommand(false, true, true, true, true, true, true));
    assertEquals(FeederState.EJECT, Robot.feederCommand(true, true, true, true, true, true, true));
    assertEquals(FeederState.HOLD, Robot.feederCommand(false, false, false, false, false, true, false));
  }
}
