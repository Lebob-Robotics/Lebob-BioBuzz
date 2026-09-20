package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class ShooterMathTest {
    @Test
    public void freeSpeedOfOneToOneMotorIs2800TicksPerSecond() {
        assertEquals(2800.0, ShooterMath.rpmToTicksPerSecond(6000, 28), 1e-9);
    }

    @Test
    public void conversionsRoundTrip() {
        double rpm = 3500;
        double tps = ShooterMath.rpmToTicksPerSecond(rpm, 28);
        assertEquals(rpm, ShooterMath.ticksPerSecondToRpm(tps, 28), 1e-9);
    }
}
