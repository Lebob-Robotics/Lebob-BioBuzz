package org.firstinspires.ftc.teamcode.subsystems.hood;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/** 15 Nov, 074849: the main loop stalled 0.59 s, the fast loop stopped itself and the hood never moved again. */
public class HoodFastLoopRestartTest {
  @Test public void aFastLoopThatStoppedWhileDrivingIsStartedAgain() {
    assertTrue(HoodSubsystem.fastLoopStoppedItself(true, false));
  }

  @Test public void aRunningOrIdleLoopIsLeftAlone() {
    assertFalse(HoodSubsystem.fastLoopStoppedItself(true, true));
    assertFalse("not driving: driveFast starts it anyway", HoodSubsystem.fastLoopStoppedItself(false, false));
  }
}
