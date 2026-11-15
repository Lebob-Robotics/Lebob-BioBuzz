package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.*;
import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem.FeederState;
import org.junit.Test;

public class FeederPowerTest {
  @Test public void loadingHoldRemainsBackwardAtEveryVolleyPower() {
    for (double feed : new double[] {0, .3, .8, 1}) {
      assertEquals(-1, FeederSubsystem.powerFor(FeederState.HOLD, feed), 0);
    }
  }

  @Test public void authorizedFeedUsesTheRequestedPower() {
    assertEquals(.4, FeederSubsystem.powerFor(FeederState.FEED, .4), 0);
  }

  @Test public void stopAndEjectIgnoreForwardPower() {
    assertEquals(0, FeederSubsystem.powerFor(FeederState.STOP, 1), 0);
    assertEquals(-1, FeederSubsystem.powerFor(FeederState.EJECT, .4), 0);
  }
}
