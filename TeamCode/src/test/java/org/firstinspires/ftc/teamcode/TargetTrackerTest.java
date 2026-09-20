package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class TargetTrackerTest {
    private static final double EPS = 1e-9;
    private static final long S = 1_000_000_000L;

    @Test
    public void noPoseYetMeansNoUpdate() {
        TargetTracker t = new TargetTracker(8, 0, 0, 0, 0, 5);
        assertFalse(t.update(0, 0, 2, 0));
        assertFalse(t.hasTarget(0));
    }

    @Test
    public void placesTargetFromCameraFrameAheadOfRobot() {
        TargetTracker t = new TargetTracker(8, 0, 0, 0, 0, 5);
        t.recordPose(0, 1, 1, 0);
        assertTrue(t.update(0, 0, 2, 0));
        assertEquals(3, t.getX(), EPS);
        assertEquals(1, t.getY(), EPS);
    }

    @Test
    public void usesPoseAtFrameTimeNotLatest() {
        TargetTracker t = new TargetTracker(8, 0, 0, 0, 0, 5);
        t.recordPose(0, 0, 0, 0);
        t.recordPose(S, 1, 0, 0);          // robot moved 1 m forward in the second after the frame
        t.update(0, 0, 2, 0);              // frame taken at t = 0 saw the Cell 2 m ahead
        assertEquals(2, t.getX(), EPS);
    }

    @Test
    public void picksNearestPoseInTime() {
        TargetTracker t = new TargetTracker(8, 0, 0, 0, 0, 5);
        t.recordPose(0, 0, 0, 0);
        t.recordPose(S, 1, 0, 0);
        t.update(S - S / 10, 0, 2, 0);     // 0.9 s: nearer the second pose
        assertEquals(3, t.getX(), EPS);
    }

    @Test
    public void ringBufferKeepsOnlyTheLastCapacityPoses() {
        TargetTracker t = new TargetTracker(2, 0, 0, 0, 0, 5);
        t.recordPose(0, 0, 0, 0);
        t.recordPose(S, 1, 0, 0);
        t.recordPose(2 * S, 2, 0, 0);      // evicts the t = 0 pose
        t.update(0, 0, 2, 0);              // nearest surviving pose is t = 1 s
        assertEquals(3, t.getX(), EPS);
    }

    @Test
    public void appliesRobotHeading() {
        TargetTracker t = new TargetTracker(8, 0, 0, 0, 0, 5);
        t.recordPose(0, 0, 0, Math.PI / 2);   // robot facing +y
        t.update(0, 0, 2, 0);
        assertEquals(0, t.getX(), EPS);
        assertEquals(2, t.getY(), EPS);
    }

    @Test
    public void appliesCameraOffsetAndYaw() {
        // Camera 0.1 m forward of centre, turned 90 deg left. Cell 1 m ahead of the camera is 1 m to the robot's left.
        TargetTracker t = new TargetTracker(8, 0.1, 0, Math.PI / 2, 0, 5);
        t.recordPose(0, 0, 0, 0);
        t.update(0, 0, 1, 0);
        assertEquals(0.1, t.getX(), EPS);
        assertEquals(1, t.getY(), EPS);
    }

    @Test
    public void cameraXRightIsRobotNegativeY() {
        TargetTracker t = new TargetTracker(8, 0, 0, 0, 0, 5);
        t.recordPose(0, 0, 0, 0);
        t.update(0, 0.5, 2, 0);
        assertEquals(-0.5, t.getY(), EPS);
    }

    @Test
    public void pitchLevelsTheRange() {
        // Camera pitched up 30 deg. A Cell 1 m along the lens axis is cos(30) ahead on the floor plan.
        TargetTracker t = new TargetTracker(8, 0, 0, 0, Math.toRadians(30), 5);
        t.recordPose(0, 0, 0, 0);
        t.update(0, 0, 1, 0);
        assertEquals(Math.cos(Math.toRadians(30)), t.getX(), EPS);
        // A point straight up the camera's z axis is behind the lens axis on the floor plan.
        t.update(0, 0, 0, 1);
        assertEquals(-Math.sin(Math.toRadians(30)), t.getX(), EPS);
    }

    @Test
    public void expiresAfterConfiguredAge() {
        TargetTracker t = new TargetTracker(8, 0, 0, 0, 0, 5);
        t.recordPose(0, 0, 0, 0);
        t.update(0, 0, 2, 0);
        assertTrue(t.hasTarget(4 * S));
        assertEquals(4, t.ageS(4 * S), EPS);
        assertFalse(t.hasTarget(6 * S));
    }
}
