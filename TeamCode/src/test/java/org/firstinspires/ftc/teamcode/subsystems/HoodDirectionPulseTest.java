package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import org.junit.Test;

public class HoodDirectionPulseTest {
  @Test public void onlyPowersDuringTwoShortWindows() {
    HoodDirectionPulse pulse = new HoodDirectionPulse();
    assertEquals(0.25, pulse.update(0, 0, 0, 0), 0);
    assertEquals(0, pulse.update(0.15, 0, 0, 0), 0);
    assertEquals(-0.25, pulse.update(0.60, 0, 0, 0), 0);
    assertEquals(0, pulse.update(0.75, 0, 0, 0), 0);
    assertEquals(0, pulse.update(100, 0, 0, 0), 0);
  }

  @Test public void eitherEncoderLimitsTravelInEitherDirectionAndCutStaysLatched() {
    HoodDirectionPulse rev = new HoodDirectionPulse();
    rev.update(0, 1000, 0, 0);
    assertEquals(0, rev.update(0.02, 744, 0, 0), 0);
    assertEquals(0, rev.update(0.03, 1000, 0, 0), 0);
    HoodDirectionPulse motor = new HoodDirectionPulse();
    motor.update(0, 0, 100, 0);
    assertEquals(0, motor.update(0.02, 0, 120, 0), 0);
  }

  @Test public void highOrInvalidCurrentImmediatelyCutsThePulse() {
    HoodDirectionPulse pulse = new HoodDirectionPulse();
    assertEquals(0, pulse.update(0, 0, 0, 3), 0);
    HoodDirectionPulse invalid = new HoodDirectionPulse();
    assertEquals(0, invalid.update(0, 0, 0, Double.NaN), 0);
  }
}
