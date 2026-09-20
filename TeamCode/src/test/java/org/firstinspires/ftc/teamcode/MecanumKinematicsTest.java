package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertArrayEquals;

import org.junit.Test;

public class MecanumKinematicsTest {
    private static final double EPS = 1e-9;

    @Test
    public void forwardDrivesAllWheelsForward() {
        assertArrayEquals(new double[]{1, 1, 1, 1}, MecanumKinematics.mix(1, 0, 0), EPS);
    }

    @Test
    public void strafeRightUsesDiagonalPattern() {
        assertArrayEquals(new double[]{1, -1, -1, 1}, MecanumKinematics.mix(0, 1, 0), EPS);
    }

    @Test
    public void rotateClockwiseDrivesLeftForwardRightBack() {
        assertArrayEquals(new double[]{1, -1, 1, -1}, MecanumKinematics.mix(0, 0, 1), EPS);
    }

    @Test
    public void saturationScalesAllWheelsTogether() {
        // raw = {3, -1, 1, 1}; divide by 3
        assertArrayEquals(new double[]{1, -1.0 / 3, 1.0 / 3, 1.0 / 3}, MecanumKinematics.mix(1, 1, 1), EPS);
    }

    @Test
    public void smallInputsAreNotScaledUp() {
        assertArrayEquals(new double[]{0.5, 0.5, 0.5, 0.5}, MecanumKinematics.mix(0.5, 0, 0), EPS);
    }

    @Test
    public void zeroHeadingLeavesInputUnchanged() {
        assertArrayEquals(new double[]{0.3, -0.4}, MecanumKinematics.toRobotRelative(0.3, -0.4, 0), EPS);
    }

    @Test
    public void robotFacingLeftMovesRightToGoFieldForward() {
        // Robot rotated 90 degrees anticlockwise. A field-forward command becomes robot-right.
        assertArrayEquals(new double[]{0, 1}, MecanumKinematics.toRobotRelative(1, 0, Math.PI / 2), EPS);
    }
}
