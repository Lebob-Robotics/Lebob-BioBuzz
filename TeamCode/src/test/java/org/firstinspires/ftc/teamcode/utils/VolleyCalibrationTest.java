package org.firstinspires.ftc.teamcode.utils;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.utils.VolleyCalibration.Ball;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

public class VolleyCalibrationTest {
  private static final double DOUBLE_DIP = 600;

  private static void volley(VolleyCalibration cal, int balls) {
    cal.startVolley();
    for (int i = 0; i < balls; i++) {
      cal.fed();
    }
  }

  @Test public void firstBallAndLaterBallsSearchApart() {
    VolleyCalibration cal = new VolleyCalibration(1.0);
    volley(cal, 4);
    assertTrue(cal.marksPending());
    cal.mark("LONG"); // First ball long: the first ball's hood steepens (longer sign -1)
    cal.mark("IN");
    cal.mark("IN");
    cal.mark("IN");
    assertFalse(cal.marksPending());
    assertEquals(1.0, cal.nextFirstTrimDeg(), 1e-9);
    assertEquals(0, cal.firstSearch().scored());
    assertEquals(3, cal.laterSearch().scored());
    assertEquals(3, cal.laterSearch().balls());
  }

  @Test public void aVolleyFiresTheTrimsItStartedWith() {
    VolleyCalibration cal = new VolleyCalibration(1.0);
    volley(cal, 2);
    cal.mark("SHORT");
    cal.mark("SHORT");
    double first = cal.nextFirstTrimDeg();
    double later = cal.nextLaterTrimDeg();
    volley(cal, 3);
    assertEquals(first, cal.balls().get(0).trimDeg, 1e-9);
    assertEquals(later, cal.balls().get(1).trimDeg, 1e-9);
    cal.mark("SHORT");
    cal.mark("SHORT"); // The later search moves after this ball...
    cal.mark("IN");
    assertEquals(later, cal.balls().get(2).trimDeg, 1e-9); // ...but the volley's third ball was fired at its start
  }

  @Test public void aHitLowersTheFeedPowerOnceAVolley() {
    VolleyCalibration cal = new VolleyCalibration(0.8);
    volley(cal, 4);
    cal.mark(VolleyCalibration.HIT);
    cal.mark(VolleyCalibration.HIT);
    cal.mark("IN");
    cal.mark("IN");
    assertEquals(0.7, cal.feedPower(), 1e-9);
    assertEquals(2, cal.hits());
    assertEquals(0, cal.firstSearch().balls()); // A collision says nothing about range
    assertEquals(0.8, cal.balls().get(0).feedPower, 1e-9); // Recorded as fired
  }

  @Test public void aDoubleDipLowersTheFeedPower() {
    VolleyCalibration cal = new VolleyCalibration(1.0);
    volley(cal, 2);
    cal.dip(0, 3600, 350, DOUBLE_DIP);
    assertEquals(1.0, cal.feedPower(), 1e-9);
    cal.dip(1, 3550, 420, DOUBLE_DIP);
    cal.dip(1, 3550, 700, DOUBLE_DIP); // Deepening: two balls went together
    assertEquals(0.9, cal.feedPower(), 1e-9);
    assertEquals(1, cal.doubles());
    assertTrue(cal.balls().get(1).doubleBall(DOUBLE_DIP));
  }

  @Test public void feedPowerStaysInItsRange() {
    VolleyCalibration cal = new VolleyCalibration(0.3);
    volley(cal, 1);
    cal.mark(VolleyCalibration.HIT);
    assertEquals(VolleyCalibration.MIN_FEED_POWER, cal.feedPower(), 1e-9);
    cal.setFeedPower(1.4);
    assertEquals(1.0, cal.feedPower(), 1e-9);
  }

  @Test public void aMarkPastTheCountedBallsIsALaterBall() {
    VolleyCalibration cal = new VolleyCalibration(1.0);
    volley(cal, 1); // Two balls in one dip: the feed counted one
    cal.mark("IN");
    assertFalse(cal.marksPending());
    Ball extra = cal.mark("IN");
    assertEquals(1, extra.index);
    assertEquals(1, cal.laterSearch().balls());
  }

  @Test public void startingANewVolleyMarksTheRestUnseen() {
    VolleyCalibration cal = new VolleyCalibration(1.0);
    volley(cal, 3);
    cal.mark("IN");
    List<Ball> unseen = cal.startVolley();
    assertEquals(2, unseen.size());
    assertEquals(VolleyCalibration.UNSEEN, unseen.get(0).mark);
    assertEquals(1, cal.laterSearch().balls() + cal.firstSearch().balls());
  }

  @Test public void replayCarriesTheSearchesAndFeedPowerOn() {
    VolleyCalibration cal = new VolleyCalibration(1.0);
    List<String> lines = new ArrayList<>();
    lines.add(VolleyCalibration.CSV_HEADER);
    volley(cal, 4);
    String[] marks = {"LONG", VolleyCalibration.HIT, "SHORT", "IN"};
    for (String m : marks) {
      lines.add(cal.row("s1", cal.mark(m), 0, true, DOUBLE_DIP, 12.5));
    }
    volley(cal, 2);
    lines.add(cal.row("s1", cal.mark("IN"), 0, true, DOUBLE_DIP, 12.5));
    lines.add(cal.row("s1", cal.mark("LONG"), 0, true, DOUBLE_DIP, 12.5));

    VolleyCalibration resumed = new VolleyCalibration(1.0);
    resumed.replay(lines);
    assertEquals(cal.nextFirstTrimDeg(), resumed.nextFirstTrimDeg(), 1e-9);
    assertEquals(cal.nextLaterTrimDeg(), resumed.nextLaterTrimDeg(), 1e-9);
    assertEquals(cal.firstSearch().balls(), resumed.firstSearch().balls());
    assertEquals(cal.laterSearch().balls(), resumed.laterSearch().balls());
    assertEquals(cal.feedPower(), resumed.feedPower(), 1e-9);
    assertEquals(1, resumed.hits());

    lines.add(resumed.resetRow("s2"));
    VolleyCalibration over = new VolleyCalibration(1.0);
    over.replay(lines);
    assertEquals(0, over.firstSearch().balls());
    assertEquals(0, over.laterSearch().balls());
    assertEquals(0.9, over.feedPower(), 1e-9); // The power carries on; only the searches start over
  }
  @Test public void everyTrialRequiresFourMarksEvenWhenDipsAreMissed() {
    VolleyCalibration cal = new VolleyCalibration(1);
    volley(cal, 2);
    cal.finishVolley();
    assertEquals(4, cal.balls().size());
    assertTrue(cal.balls().get(0).detected);
    assertFalse(cal.balls().get(3).detected);
    for (int i = 0; i < 3; i++) { cal.mark("IN"); assertTrue(cal.marksPending()); }
    cal.mark(VolleyCalibration.UNSEEN);
    assertFalse(cal.marksPending());
    assertEquals(3, cal.firstSearch().balls() + cal.laterSearch().balls());
  }

  @Test(expected = IllegalStateException.class) public void cannotCountAFifthBall() {
    VolleyCalibration cal = new VolleyCalibration(1); volley(cal, 4); cal.fed();
  }

  @Test(expected = IllegalStateException.class) public void cannotMarkAFifthBall() {
    VolleyCalibration cal = new VolleyCalibration(1); volley(cal, 4);
    for (int i = 0; i < 5; i++) { cal.mark("IN"); }
  }

}
