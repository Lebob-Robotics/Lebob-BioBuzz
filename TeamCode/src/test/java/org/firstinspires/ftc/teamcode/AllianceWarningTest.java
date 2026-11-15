package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class AllianceWarningTest {
  @Test
  public void warnsWhileOnlyTheOtherHiveIsSeen() {
    String w = Robot.allianceWarning(Alliance.BLUE, Double.POSITIVE_INFINITY, 0.2); // 193125: only red tags
    assertNotNull(w);
    assertTrue(w.contains("press B"));
    assertNull(Robot.allianceWarning(Alliance.BLUE, 0.5, 0.2)); // Both Hives in view: ours is there
    assertNull(Robot.allianceWarning(Alliance.BLUE, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY));
    assertTrue(Robot.allianceWarning(Alliance.RED, 10, 0).contains("press X"));
  }
}
