package org.firstinspires.ftc.teamcode;

/** Pure maths for a four-wheel mecanum base. No hardware, so it runs in JVM unit tests. */
public final class MecanumKinematics {
    private MecanumKinematics() {}

    /**
     * Rotates a field-relative (forward, right) command into the robot frame.
     *
     * @param headingRadians robot heading, anticlockwise positive, 0 = facing field-forward
     * @return {forward, right} in the robot frame
     */
    public static double[] toRobotRelative(double forward, double right, double headingRadians) {
        double cos = Math.cos(headingRadians);
        double sin = Math.sin(headingRadians);
        // Standard rotation by -heading with x = right, y = forward.
        return new double[]{
                forward * cos - right * sin,
                right * cos + forward * sin,
        };
    }

    /**
     * Mixes forward, strafe-right and clockwise rotation into wheel powers.
     *
     * @return {frontLeft, frontRight, backLeft, backRight}, scaled so the largest magnitude is at most 1
     */
    public static double[] mix(double forward, double right, double rotate) {
        double[] p = {
                forward + right + rotate,
                forward - right - rotate,
                forward - right + rotate,
                forward + right - rotate,
        };
        double max = 1.0;
        for (double v : p) max = Math.max(max, Math.abs(v));
        for (int i = 0; i < p.length; i++) p[i] /= max;
        return p;
    }
}
