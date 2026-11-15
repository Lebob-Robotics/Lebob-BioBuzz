package org.firstinspires.ftc.teamcode.tuning;

import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.tuning.BallDipDetector.Result;
import org.junit.Test;

public class BallDipDetectorTest {
  @Test
  public void aBallDipsTheWheels200RpmWithinTheWindow() {
    BallDipDetector d = new BallDipDetector();
    d.start(10.0, 3500);
    assertEquals(Result.WAITING, d.update(10.1, 3400));
    assertEquals(Result.WAITING, d.update(10.2, 3301));
    assertEquals(Result.BALL, d.update(10.3, 3300));
    assertEquals(200, d.dipRpm(), 1e-9);
    assertEquals(0.3, d.sinceStartS(10.3), 1e-9);
  }

  @Test
  public void theDeepestDipCountsThoughTheWheelsRecover() {
    BallDipDetector d = new BallDipDetector();
    d.start(0, 3500);
    d.update(0.1, 3350);
    assertEquals(Result.WAITING, d.update(0.2, 3480));
    assertEquals(150, d.dipRpm(), 1e-9);
  }

  @Test
  public void aFeedWithoutABallIsEmptyOnceTheWindowHasPassed() {
    BallDipDetector d = new BallDipDetector();
    d.start(0, 3500);
    assertEquals(Result.WAITING, d.update(0.6, 3400));
    assertEquals(Result.EMPTY, d.update(0.62, 3400));
  }

  @Test
  public void aRetryPulseStartsAgainFromTheWheelsSpeedThen() {
    BallDipDetector d = new BallDipDetector();
    d.start(0, 3500);
    d.update(0.1, 3400);
    d.start(0.5, 3450);
    assertEquals(Result.WAITING, d.update(0.6, 3300));
    assertEquals(Result.BALL, d.update(0.7, 3250));
  }
}
