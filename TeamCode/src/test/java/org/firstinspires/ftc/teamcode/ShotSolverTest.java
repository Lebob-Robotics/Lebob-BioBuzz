package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class ShotSolverTest {
    private static final double EPS = 1e-9;

    // rpm = 2000 + 500 * (d - 1) - 200 * vr on a 3 x 3 grid; the (3 m, +1 m/s) cell has no shot.
    private static final double[] DIST = {1, 2, 3};
    private static final double[] VEL = {-1, 0, 1};
    private static final double[][] RPM = {
            {2200, 2000, 1800},
            {2700, 2500, 2300},
            {3200, 3000, Double.NaN}};
    private static final double[][] BAND = {
            {400, 400, 400},
            {400, 400, 100},
            {400, 400, Double.NaN}};
    private static final double[][] TOF = {
            {0.8, 0.8, 0.8},
            {1.0, 1.0, 1.0},
            {1.2, 1.2, Double.NaN}};

    private static ShotSolver solver(double shooterOffsetM, double feedDelayS) {
        // 60 deg launch, 0.002 m/s per RPM: 2500 RPM gives 5 m/s exit, 2.5 m/s horizontal.
        return new ShotSolver(DIST, VEL, RPM, BAND, TOF, 60, 0.002, shooterOffsetM, 0, feedDelayS, 200, 0.508, 0.0555, 3500);
    }

    @Test
    public void interpolateReturnsGridValueAtGridPoint() {
        assertEquals(2500, ShotSolver.interpolate(DIST, VEL, RPM, 2, 0), EPS);
    }

    @Test
    public void interpolateAveragesNeighboursAtMidpoint() {
        // between (1,-1)=2200, (1,0)=2000, (2,-1)=2700, (2,0)=2500
        assertEquals(2350, ShotSolver.interpolate(DIST, VEL, RPM, 1.5, -0.5), EPS);
    }

    @Test
    public void interpolateOutsideGridIsNaN() {
        assertTrue(Double.isNaN(ShotSolver.interpolate(DIST, VEL, RPM, 0.5, 0)));
        assertTrue(Double.isNaN(ShotSolver.interpolate(DIST, VEL, RPM, 2, 1.5)));
    }

    @Test
    public void interpolateNextToMissingCellIsNaN() {
        assertTrue(Double.isNaN(ShotSolver.interpolate(DIST, VEL, RPM, 2.5, 0.5)));
    }

    @Test
    public void stationaryShotAimsAlongBearingAtTableRpm() {
        ShotSolver.Shot s = solver(0, 0).solve(0, 0, 0, 0, 0, 2, 0);
        assertTrue(s.valid);
        assertEquals("OK", s.reason);
        assertEquals(2, s.distanceM, EPS);
        assertEquals(0, s.headingRad, EPS);
        assertEquals(2500, s.rpm, EPS);
        assertEquals(1.0, s.timeOfFlightS, EPS);
    }

    @Test
    public void bearingFollowsTargetPosition() {
        ShotSolver.Shot s = solver(0, 0).solve(0, 0, 0, 0, 0, 0, 2);
        assertEquals(Math.PI / 2, s.headingRad, EPS);
        assertEquals(0, s.radialVel, EPS);
    }

    @Test
    public void strafingLeftLeadsRightOfBearing() {
        // Target straight ahead, robot moving left (+y) at 0.5 m/s. Horizontal exit speed 2.5 m/s.
        ShotSolver.Shot s = solver(0, 0).solve(0, 0, 0, 0, 0.5, 2, 0);
        assertEquals(0.5, s.tangentialVel, EPS);
        assertEquals(0, s.radialVel, EPS);
        assertEquals(-Math.asin(0.5 / 2.5), s.headingRad, 1e-9);
    }

    @Test
    public void closingLowersRpmAndCountsAsRadial() {
        ShotSolver.Shot s = solver(0, 0).solve(0, 0, 0, 0.5, 0, 2, 0);
        assertEquals(0.5, s.radialVel, EPS);
        assertEquals(2400, s.rpm, EPS);   // 2500 - 200 * 0.5
        assertTrue(s.valid);
    }

    @Test
    public void closingTooFastIsInvalidWithReason() {
        ShotSolver.Shot s = solver(0, 0).solve(0, 0, 0, 2.0, 0, 2, 0);
        assertFalse(s.valid);
        assertEquals("CLOSING TOO FAST", s.reason);
        assertEquals(2500, s.idleRpm, EPS);   // stationary RPM at the same distance
    }

    @Test
    public void backingTooFastIsInvalidWithReason() {
        assertEquals("BACKING TOO FAST", solver(0, 0).solve(0, 0, 0, -2.0, 0, 2, 0).reason);
    }

    @Test
    public void outOfRangeIsInvalidAndIdlesAtNearestEdge() {
        ShotSolver.Shot s = solver(0, 0).solve(0, 0, 0, 0, 0, 5, 0);
        assertFalse(s.valid);
        assertEquals("OUT OF RANGE", s.reason);
        assertEquals(3000, s.idleRpm, EPS);   // stationary RPM at the far edge of the grid
    }

    @Test
    public void idleFallsBackWhenTheStationaryCellIsMissing() {
        double[][] noStationary = {{2200, Double.NaN, 1800}, {2700, Double.NaN, 2300}, {3200, Double.NaN, Double.NaN}};
        ShotSolver s = new ShotSolver(DIST, VEL, noStationary, BAND, TOF, 60, 0.002, 0, 0, 0, 200, 0.508, 0.0555, 3500);
        assertEquals(3500, s.solve(0, 0, 0, 0, 0, 2, 0).idleRpm, EPS);
    }

    @Test
    public void narrowBandIsInvalid() {
        // (2 m, +1 m/s) has a 100 RPM band, below the 200 RPM minimum.
        ShotSolver.Shot s = solver(0, 0).solve(0, 0, 0, 1.0, 0, 2, 0);
        assertFalse(s.valid);
        assertEquals("NO SHOT", s.reason);
    }

    @Test
    public void missingTimeOfFlightIsInvalid() {
        // Same RPM/BAND fixture, but the TOF cells around (2 m, 0 m/s) are missing.
        double[][] tofMissing = {
                {0.8, 0.8, 0.8},
                {Double.NaN, Double.NaN, Double.NaN},
                {1.2, 1.2, 1.2}};
        ShotSolver s = new ShotSolver(DIST, VEL, RPM, BAND, tofMissing, 60, 0.002, 0, 0, 0, 200, 0.508, 0.0555, 3500);
        ShotSolver.Shot shot = s.solve(0, 0, 0, 0, 0, 2, 0);
        assertFalse(shot.valid);
        assertEquals("NO SHOT", shot.reason);
    }

    @Test
    public void zeroHorizontalExitSpeedIsInvalid() {
        // 90 deg launch: horizontal exit speed is ~0, so no tangential lead is possible.
        ShotSolver s = new ShotSolver(DIST, VEL, RPM, BAND, TOF, 90, 0.002, 0, 0, 0, 200, 0.508, 0.0555, 3500);
        ShotSolver.Shot shot = s.solve(0, 0, 0, 0, 0.5, 2, 0);
        assertFalse(shot.valid);
        assertEquals("NO SHOT", shot.reason);
        assertFalse(Double.isNaN(shot.headingRad));
    }

    @Test
    public void feedDelayAdvancesThePose() {
        ShotSolver.Shot s = solver(0, 0.5).solve(0, 0, 0, 1.0, 0, 2, 0);
        assertEquals(1.5, s.distanceM, EPS);
    }

    @Test
    public void shooterOffsetShortensDistanceAlongHeading() {
        ShotSolver.Shot s = solver(0.2, 0).solve(0, 0, 0, 0, 0, 2, 0);
        assertEquals(1.8, s.distanceM, EPS);
    }

    @Test
    public void headingToleranceNarrowsWithDistance() {
        ShotSolver.Shot near = solver(0, 0).solve(0, 0, 0, 0, 0, 1, 0);
        ShotSolver.Shot far = solver(0, 0).solve(0, 0, 0, 0, 0, 3, 0);
        assertEquals(Math.atan((0.254 - 0.0555) / 1.0), near.headingToleranceRad, EPS);
        assertTrue(far.headingToleranceRad < near.headingToleranceRad);
    }

    @Test
    public void lipToCentreShortensTheTableDistance() {
        // Table distance is to the near lip; the tracked target is the opening centre, 0.15 m further out.
        ShotSolver s = new ShotSolver(DIST, VEL, RPM, BAND, TOF, 60, 0.002, 0, 0.15, 0, 200, 0.508, 0.0555, 3500);
        ShotSolver.Shot shot = s.solve(0, 0, 0, 0, 0, 2, 0);
        assertEquals(1.85, shot.distanceM, EPS);
        assertEquals(0, shot.headingRad, EPS);
    }

    @Test
    public void headingErrorWrapsAndIsPositiveWhenTargetIsLeft() {
        ShotSolver.Shot s = solver(0, 0).solve(0, 0, 0, 0, 0, 0, 2);   // target heading +90 deg
        assertEquals(Math.PI / 2, s.headingErrorRad(0), EPS);
        assertEquals(-Math.PI / 2, s.headingErrorRad(Math.PI), EPS);
        assertEquals(0.1, ShotSolver.wrap(2 * Math.PI + 0.1), EPS);
    }
}
