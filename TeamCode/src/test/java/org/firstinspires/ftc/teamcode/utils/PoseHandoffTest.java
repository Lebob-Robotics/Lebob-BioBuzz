package org.firstinspires.ftc.teamcode.utils;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;

import org.firstinspires.ftc.teamcode.Alliance;
import org.junit.After;
import org.junit.Test;

public class PoseHandoffTest {
  private static final long S = 1_000_000_000L;

  @After public void forget() { PoseHandoff.clear(); }

  @Test public void teleOpRightAfterTheAutoStartsWhereItEnded() {
    PoseHandoff.save(new Pose(-0.91, -1.54, Math.toRadians(-128.9)), Alliance.RED, 100 * S);
    Pose p = PoseHandoff.take(108 * S); // A match's transition
    assertNotNull(p);
    assertEquals(-128.9, Math.toDegrees(p.headingRad), 1e-9);
    assertNull("Used once", PoseHandoff.take(109 * S));
    assertEquals("The alliance stays", Alliance.RED, PoseHandoff.alliance());
  }

  @Test public void anOldPoseIsNotUsed() {
    PoseHandoff.save(new Pose(0, 0, 0), Alliance.BLUE, 0);
    assertNull(PoseHandoff.take((long) ((PoseHandoff.MAX_AGE_S + 1) * S)));
    assertEquals(Alliance.BLUE, PoseHandoff.alliance());
  }

  @Test public void anAutoThatNeverKnewItsPoseHandsOnOnlyItsAlliance() {
    PoseHandoff.save(null, Alliance.RED, 0);
    assertNull(PoseHandoff.take(S));
    assertEquals(Alliance.RED, PoseHandoff.alliance());
  }
}
