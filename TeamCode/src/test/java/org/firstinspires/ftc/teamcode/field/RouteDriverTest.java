package org.firstinspires.ftc.teamcode.field;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Arrays;
import java.util.List;
import org.junit.Test;

public class RouteDriverTest {
  private static final List<double[]> ROUTE = Arrays.asList(new double[] {0, 1}, new double[] {1, 1});

  @Test
  public void followsEachLegToTheEnd() {
    RouteDriver d = new RouteDriver();
    FieldPose start = new FieldPose(0, 0, 0);
    d.start(0, start, ROUTE, 0);
    assertArrayEquals(FieldRoute.goToStick(start, 0, 1, 0, 0, 0), d.stick(0, start), 1e-12);
    FieldPose corner = new FieldPose(0, 0.9, 0);
    assertArrayEquals(FieldRoute.goToStick(corner, 1, 1, 0, 0.9, 0), d.stick(0, corner), 1e-12);
    assertFalse(d.arrived(new FieldPose(0, 1, 0)));
    assertTrue(d.arrived(new FieldPose(0.95, 1, 0)));
  }

  @Test
  public void boostsTheStickUntilItMovesFiveCentimetres() {
    RouteDriver d = new RouteDriver();
    FieldPose start = new FieldPose(0, 0, 0);
    d.start(0, start, ROUTE, 0);
    FieldPose crept = new FieldPose(0, 0.04, 0);
    d.track(2, crept, 0);
    assertArrayEquals(FieldRoute.goToStick(crept, 0, 1, 0, 0, 2 * FieldRoute.GOTO_STUCK_BOOST_PER_S),
        d.stick(2, crept), 1e-12);
    FieldPose moved = new FieldPose(0, 0.05, 0);
    d.track(2, moved, 0);
    assertArrayEquals(FieldRoute.goToStick(moved, 0, 1, 0, 0, 0), d.stick(2, moved), 1e-12);
  }

  @Test
  public void blindWithoutANewFixAndTimedOutOverTheRoutesLength() {
    RouteDriver d = new RouteDriver();
    FieldPose start = new FieldPose(0, 0, 0);
    d.start(0, start, ROUTE, 3);
    d.track(FieldRoute.GOTO_BLIND_S, start, 3);
    assertFalse(d.blind(FieldRoute.GOTO_BLIND_S));
    assertTrue(d.blind(FieldRoute.GOTO_BLIND_S + 0.01));
    d.track(FieldRoute.GOTO_BLIND_S + 0.01, start, 4);
    assertFalse(d.blind(2 * FieldRoute.GOTO_BLIND_S));
    d.restartClocks(10);
    assertFalse(d.blind(10 + FieldRoute.GOTO_BLIND_S));
    double timeoutS = FieldRoute.GOTO_TIMEOUT_S + 2 / 0.4;
    assertFalse(d.timedOut(timeoutS));
    assertTrue(d.timedOut(timeoutS + 0.01));
  }
}
