package org.firstinspires.ftc.teamcode.utils;

import static org.junit.Assert.*;
import org.junit.Test;

public class LoadingShooterInterlockTest {
  @Test public void volleyStandbyNeverPowersIdleWheelsWithoutARequest() {
    LoadingShooterInterlock g = new LoadingShooterInterlock();
    g.update(0, true, false, 2500);
    assertTrue(g.blocksSpin()); assertFalse(g.canLoad());
    g.update(10, true, false, 2500);
    assertTrue(g.blocksSpin()); assertFalse(g.canLoad());
  }

  @Test public void loadingWaitsForBothWheelsToCoastAndStayStopped() {
    LoadingShooterInterlock g = new LoadingShooterInterlock();
    g.update(0, true, false, 3000);
    g.update(1, true, false, 90);
    assertFalse(g.canLoad());
    g.update(1.1, true, false, 90);
    assertFalse(g.canLoad());
    g.update(1.2, true, false, 90);
    assertTrue(g.canLoad()); assertTrue(g.blocksSpin());
    g.update(1.3, true, false, 200);
    assertFalse(g.canLoad());
  }

  @Test public void releaseOfLtDoesNotAutomaticallyRestartStandbySpin() {
    LoadingShooterInterlock g = new LoadingShooterInterlock();
    g.update(0, true, false, 0);
    g.update(1, true, false, 0);
    g.update(30, true, false, 0);
    assertTrue(g.blocksSpin()); assertTrue(g.canLoad());
  }

  @Test public void shotBeginsWithAStoppedFeederClearanceInterval() {
    LoadingShooterInterlock g = new LoadingShooterInterlock();
    g.update(0, true, true, 0);
    assertTrue(g.blocksSpin()); assertTrue(g.clearing()); assertFalse(g.canLoad());
    g.update(.2, true, true, 0);
    assertTrue(g.blocksSpin());
    g.update(.31, true, true, 0);
    assertFalse(g.blocksSpin()); assertFalse(g.clearing()); assertFalse(g.canLoad());
    g.update(2, true, true, 3400);
    assertFalse(g.blocksSpin());
  }

  @Test public void releaseCancelsClearanceAndAnotherPressMustClearAgain() {
    LoadingShooterInterlock g = new LoadingShooterInterlock();
    g.update(0, true, true, 0);
    g.update(.1, true, false, 0);
    assertTrue(g.blocksSpin()); assertFalse(g.clearing());
    g.update(.2, true, true, 0);
    g.update(.4, true, true, 0);
    assertTrue(g.blocksSpin());
    g.update(.51, true, true, 0);
    assertFalse(g.blocksSpin());
  }

  @Test public void invalidSpeedNeverPermitsLoading() {
    LoadingShooterInterlock g = new LoadingShooterInterlock();
    g.update(0, true, false, 0);
    g.update(1, true, false, Double.NaN);
    assertFalse(g.canLoad());
    g.update(2, true, false, 0);
    assertFalse(g.canLoad());
  }

  @Test public void otherTuningModesRetainTheirExistingSpinPolicy() {
    LoadingShooterInterlock g = new LoadingShooterInterlock();
    g.update(0, false, false, 2500);
    assertFalse(g.blocksSpin()); assertTrue(g.canLoad()); assertFalse(g.clearing());
  }
}
