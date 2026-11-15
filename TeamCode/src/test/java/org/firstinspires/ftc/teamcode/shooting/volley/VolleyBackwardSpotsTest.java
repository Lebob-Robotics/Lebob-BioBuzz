package org.firstinspires.ftc.teamcode.shooting.volley;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.shooting.Aim;
import org.firstinspires.ftc.teamcode.field.FieldPose;
import org.firstinspires.ftc.teamcode.shooting.ShootOnTheMove;
import org.firstinspires.ftc.teamcode.shooting.ShotPickConstants;
import org.junit.Test;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class VolleyBackwardSpotsTest {
  private static final double AX = -.44, AY = -.324;

  @Test public void backwardSpotsAreGridOneToFourAfterTheExtraOnes() {
    assertEquals(VolleySpotDrive.PARK + 3, VolleySpotDrive.BACKWARD_FIRST);
    assertEquals(VolleySpotDrive.BACKWARD_FIRST + 4, VolleySpotDrive.SPOTS);
    for (int k = 0; k < 4; k++) {
      int s = VolleySpotDrive.BACKWARD_FIRST + k;
      assertTrue(VolleySpotDrive.isBackward(s));
      assertEquals("B" + (k + 1), VolleySpotDrive.name(s));
      assertArrayEquals(VolleySpotDrive.point(k, AX, AY), VolleySpotDrive.point(s, AX, AY), 1e-12);
    }
    assertFalse(VolleySpotDrive.isBackward(VolleySpotDrive.PARK + 2));
    assertFalse(VolleySpotDrive.isBackward(-1));
  }

  @Test public void everyBackwardSpotHasABackwardTableShotInTheWedge() {
    for (int s : VolleySpotDrive.BACKWARD_ORDER) {
      double[] p = VolleySpotDrive.point(s, AX, AY);
      FieldPose pose = new FieldPose(p[0], p[1], 0);
      Aim aim = ShootOnTheMove.aimBackward(pose, 0, 0, AX, AY);
      assertTrue("spot " + VolleySpotDrive.name(s) + ": " + aim.shot.reason, aim.shot.valid);
      assertTrue(ShootOnTheMove.inFrontOfCell(pose, AX, AY, ShotPickConstants.BACKWARD_ONLY_WEDGE_DEG));
      assertTrue(aim.shot.distanceToLipM <= ShotPickConstants.BACKWARD_WEDGE_MAX_LIP_M);
    }
  }

  @Test public void backwardSpotsAreReachable() {
    for (int s : VolleySpotDrive.BACKWARD_ORDER) {
      VolleySpotDrive d = new VolleySpotDrive();
      assertTrue(VolleySpotDrive.name(s), d.start(s, AX, AY, 0, new FieldPose(-1.0, -1.0, 0), true, 1));
      assertEquals(VolleySpotDrive.Stage.MOVING, d.stage());
    }
  }

  @Test public void startsAndMovesOnAtTheFirstUntunedSpotOfTheSetInUse() {
    boolean[] done = new boolean[VolleySpotDrive.SPOTS];
    assertEquals(VolleySpotDrive.PARK, VolleySpotDrive.firstUntuned(done, VolleySpotDrive.order(false)));
    done[VolleySpotDrive.PARK] = done[4] = true;
    assertEquals(VolleySpotDrive.PARK + 1, VolleySpotDrive.firstUntuned(done, VolleySpotDrive.order(false)));
    assertEquals(VolleySpotDrive.BACKWARD_FIRST + 2, VolleySpotDrive.firstUntuned(done, VolleySpotDrive.order(true)));
    done[VolleySpotDrive.BACKWARD_FIRST + 2] = true;
    assertEquals(VolleySpotDrive.BACKWARD_FIRST + 3,
        VolleySpotDrive.nextVolleySpot(VolleySpotDrive.BACKWARD_FIRST + 2, done, VolleySpotDrive.order(true)));
    for (int s : VolleySpotDrive.VOLLEY_ORDER) { done[s] = true; }
    assertEquals(VolleySpotDrive.PARK, VolleySpotDrive.firstUntuned(done, VolleySpotDrive.order(false)));
  }

  @Test public void trimsOnlyApplyToTheShotTheyWereTunedOn() {
    double first = VolleyConstants.VOLLEY_FIRST_BALL_TRIM_DEG;
    double later = VolleyConstants.VOLLEY_LATER_BALL_TRIM_DEG;
    double backFirst = VolleyConstants.VOLLEY_BACKWARD_FIRST_BALL_TRIM_DEG;
    double backLater = VolleyConstants.VOLLEY_BACKWARD_LATER_BALL_TRIM_DEG;
    // Hooded trims: the hooded volley uses them, the backward one its own constants
    assertEquals(1.5, Robot.volleyTrimDeg(1.5, false, false, true), 0);
    assertEquals(backFirst, Robot.volleyTrimDeg(1.5, false, true, true), 0);
    assertEquals(backLater, Robot.volleyTrimDeg(1.5, false, true, false), 0);
    // Backward trims: the backward volley uses them, the hooded one the constants
    assertEquals(-2, Robot.volleyTrimDeg(-2, true, true, false), 0);
    assertEquals(first, Robot.volleyTrimDeg(-2, true, false, true), 0);
    assertEquals(later, Robot.volleyTrimDeg(-2, true, false, false), 0);
    // None set
    assertEquals(backFirst, Robot.volleyTrimDeg(Double.NaN, true, true, true), 0);
    assertEquals(185, -VolleyConstants.VOLLEY_BACKWARD_RPM_PER_TRIM_DEG * backFirst, 1e-9); // B3's first balls
    assertEquals(later, Robot.volleyTrimDeg(Double.NaN, false, false, false), 0);
  }
}
