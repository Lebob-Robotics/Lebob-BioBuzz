package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import java.util.EnumSet;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.junit.Test;

public class SotmPathTest {
  @Test
  public void hoodedPlanCoversEveryKindAndFits() {
    long t0 = System.nanoTime();
    SotmPath.plan(true);
    List<SotmPath.Entry> plan = SotmPath.plan(false);
    System.out.println("Both plans built in " + (System.nanoTime() - t0) / 1000000 + " ms");
    Set<String> keys = new HashSet<>();
    EnumSet<SotmPath.Kind> kinds = EnumSet.noneOf(SotmPath.Kind.class);
    for (SotmPath.Entry e : plan) {
      assertTrue("duplicate " + e.key, keys.add(e.key));
      assertTrue(e.key, e.path.fits());
      kinds.add(e.path.kind);
    }
    System.out.println("Hooded plan: " + plan.size() + " passes");
    for (String s : SotmPath.LEFT_OUT) {
      System.out.println("  LEFT OUT " + s);
    }
    for (SotmPath.Entry e : plan) {
      System.out.println("  " + e.key + String.format(" fire at %.2f,%.2f  %.1f s", e.path.sample(e.path.fireS)[0],
          e.path.sample(e.path.fireS)[1], e.path.durationS()));
    }
    assertEquals(EnumSet.allOf(SotmPath.Kind.class), kinds);
    assertTrue(plan.size() >= 80);
  }

  @Test
  public void bothPlansHaveSteadySpotsToSearch() {
    for (boolean backward : new boolean[] {false, true}) {
      int spots = 0;
      for (SotmPath.Entry e : SotmPath.plan(backward)) {
        if (e.path.kind == SotmPath.Kind.STRAIGHT || e.path.kind == SotmPath.Kind.ARC) {
          spots++;
          assertTrue(e.key, !e.relaxedGates);
        }
      }
      System.out.println((backward ? "backward" : "hooded") + " spots: " + spots);
      assertTrue(spots >= 6);
    }
  }

  @Test
  public void firstSpotsSpanTowardSidewaysAndDistance() {
    List<SotmPath.Entry> order = SotmPath.spotsFirst(SotmPath.plan(false));
    boolean toward = false;
    boolean away = false;
    boolean left = false;
    boolean right = false;
    double near = Double.POSITIVE_INFINITY;
    double far = 0;
    for (int i = 0; i < 12; i++) {
      assertTrue(SotmPath.isSpot(order.get(i)));
      double[] s = SotmPath.fireState(order.get(i).path);
      System.out.printf("%2d %-40s toward %+.2f sideways %+.2f at %.2f m%n", i, order.get(i).key, s[0], s[1], s[2]);
      toward |= s[0] > 0.25;
      away |= s[0] < -0.25;
      left |= s[1] > 0.25;
      right |= s[1] < -0.25;
      near = Math.min(near, s[2]);
      far = Math.max(far, s[2]);
    }
    assertTrue(toward && away && left && right);
    assertTrue(far - near > 0.3);
  }

  @Test
  public void backwardPlanStaysSlow() {
    List<SotmPath.Entry> plan = SotmPath.plan(true);
    System.out.println("Backward plan: " + plan.size() + " passes");
    assertTrue(plan.size() >= 10);
    for (SotmPath.Entry e : plan) {
      for (int i = 0; i < e.path.vx.length; i++) {
        assertTrue(e.key, Math.hypot(e.path.vx[i], e.path.vy[i]) <= 0.3);
      }
    }
  }

  @Test
  public void straightPassFiresAtTheFirePointAtSpeedAndEndsStopped() {
    SotmPath p = SotmPath.place(SotmPath.Kind.STRAIGHT, 0.3, 90, 0, false, 0.9, 1.1);
    assertNotNull(p);
    double[] atFire = p.sample(p.fireS);
    double ux = SotmPath.CELL_X - atFire[0];
    double uy = SotmPath.CELL_Y - atFire[1];
    double n = Math.hypot(ux, uy);
    assertEquals(p.aimDistM, n, 1e-4); // Between two of the plan's 20 ms points
    // Left: ShootOnTheMove's sideways speed, positive to the left of the line to the Cell
    assertEquals(0.3, (-atFire[2] * uy + atFire[3] * ux) / n, 0.01);
    assertEquals(0.0, (atFire[2] * ux + atFire[3] * uy) / n, 0.01);
    double[] end = p.sample(p.durationS());
    assertEquals(0, Math.hypot(end[2], end[3]), 1e-9);
  }

  @Test
  public void arcKeepsItsDistanceFromTheCell() {
    SotmPath p = SotmPath.place(SotmPath.Kind.ARC, 0.3, -90, 0, false, 1.1, 1.35);
    assertNotNull(p);
    double[] atFire = p.sample(p.fireS);
    double r = Math.hypot(atFire[0] - SotmPath.CELL_X, atFire[1] - SotmPath.CELL_Y);
    assertEquals(p.aimDistM, r, 0.01);
    for (double t = p.fireS - 0.3; t <= p.fireS + 0.3; t += 0.05) {
      double[] s = p.sample(t);
      assertEquals(r, Math.hypot(s[0] - SotmPath.CELL_X, s[1] - SotmPath.CELL_Y), 0.03);
    }
  }

  @Test
  public void mirroredPathStaysInTheArea() {
    for (SotmPath.Entry e : SotmPath.plan(false)) {
      assertTrue(e.key, e.path.mirroredFor(0.44, 0.32).fits());
      assertTrue(e.key, e.path.mirroredFor(-0.44, 0.32).fits());
    }
  }

  @Test
  public void pathsTooBigForTheAreaAreLeftOut() {
    assertEquals(null, SotmPath.build(SotmPath.Kind.STRAIGHT, 0.9, 0, 0, false, -1.2, -0.32, SotmPath.CELL_X,
        SotmPath.CELL_Y));
  }

  @Test
  public void fieldFrameLineRunsAlongTheStrip() {
    SotmPath p = SotmPath.place(SotmPath.Kind.STRAIGHT, 1.0, -90, 0, true, 1.1, 1.35);
    assertNotNull(p);
    double[] atFire = p.sample(p.fireS);
    assertEquals(0, atFire[2], 1e-6);
    assertEquals(-1.0, atFire[3], 0.01);
  }

  @Test
  public void everyPassHasAShotFromItsFireTime() {
    for (boolean backward : new boolean[] {false, true}) {
      for (SotmPath.Entry e : SotmPath.plan(backward)) {
        double needS = Math.min(e.path.windowS, SotmPath.SHOT_CHECK_MIN_S);
        assertTrue(e.key + " has a shot for " + e.path.shotS(backward) + " s",
            e.path.shotS(backward) >= needS - 1e-9);
        // Its window ends where the shot does, so it never waits to fire where there is none
        assertEquals(e.key, e.path.windowS, e.path.shotS(backward), 1e-9);
      }
    }
  }

  @Test
  public void lineBackingOutOfTheTableIsNotPlanned() {
    // 7 Nov, 083925: fired from at 0.98 m, it backed out of the table 0.25 s into its window, three runs no ball
    for (SotmPath.Entry e : SotmPath.plan(false)) {
      assertTrue(e.key, !e.key.equals("F STRAIGHT field +y 1.00 m/s at 0.98 m"));
    }
  }
}
