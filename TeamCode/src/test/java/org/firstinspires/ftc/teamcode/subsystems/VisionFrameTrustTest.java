package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/** 15 Nov, 080649: camera R's first frame after a restart (4.4 fps) put the Cell 0.7 m off and TeleOp fell short. */
public class VisionFrameTrustTest {
  @Test public void aStreamStillStartingIsNotUsed() {
    assertFalse(VisionSubsystem.frameTimesTrusted(4.38));
    assertFalse(VisionSubsystem.frameTimesTrusted(0));
  }

  @Test public void normalStreamsAreUsed() {
    assertTrue(VisionSubsystem.frameTimesTrusted(5.0));
    assertTrue(VisionSubsystem.frameTimesTrusted(13.0));
  }
}
