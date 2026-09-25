package org.firstinspires.ftc.teamcode;

/** Unit conversions for the flywheel encoders. */
public final class ShooterMath {
    private ShooterMath() {}

    public static double rpmToTicksPerSecond(double rpm, double ticksPerRev) {
        return rpm * ticksPerRev / 60.0;
    }

    public static double ticksPerSecondToRpm(double ticksPerSecond, double ticksPerRev) {
        return ticksPerSecond * 60.0 / ticksPerRev;
    }
}
