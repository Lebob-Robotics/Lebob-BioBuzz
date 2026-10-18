package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.subsystems.ShootOnTheMove.Cell;
import org.firstinspires.ftc.teamcode.subsystems.ShootOnTheMove.Compensation;
import org.firstinspires.ftc.teamcode.subsystems.ShootOnTheMove.GateResult;
import org.firstinspires.ftc.teamcode.subsystems.ShootOnTheMove.Goal;
import org.firstinspires.ftc.teamcode.subsystems.ShootOnTheMove.Pose;
import org.firstinspires.ftc.teamcode.subsystems.ShootOnTheMove.PoseHistory;
import org.firstinspires.ftc.teamcode.subsystems.ShootOnTheMove.Reason;
import org.firstinspires.ftc.teamcode.subsystems.ShootOnTheMove.TagObservation;
import org.firstinspires.ftc.teamcode.subsystems.ShootOnTheMove.Solution;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ShootOnTheMoveTest {
  private static final double EPS = 1e-9;

  // Saved constants, restored after each test.
  private double delay, mount, offF, offL, hoodOff, rpmPer, lmin, lmax;
  private int degree, histSize;
  private double[] hoodC, speedC, tofC, grid, vrMin, vrMax, corrX, corrY;
  private double rpmTol, hoodTol, hdgCap, halfW, ballR, margin, maxTurn, maxApproach;

  @Before
  public void setUp() {
    delay = ShootOnTheMove.MECHANISM_DELAY_S;
    mount = ShootOnTheMove.SHOOTER_MOUNT_YAW_RAD;
    offF = ShootOnTheMove.SHOOTER_OFFSET_FORWARD_M;
    offL = ShootOnTheMove.SHOOTER_OFFSET_LEFT_M;
    hoodOff = ShootOnTheMove.HOOD_ANGLE_OFFSET_DEG;
    rpmPer = ShootOnTheMove.RPM_PER_MPS;
    lmin = ShootOnTheMove.LAUNCH_MIN_DEG;
    lmax = ShootOnTheMove.LAUNCH_MAX_DEG;
    degree = ShootOnTheMove.POLY_DEGREE;
    histSize = ShootOnTheMove.POSE_HISTORY_SIZE;
    hoodC = ShootOnTheMove.HOOD_ANGLE_COEFFS;
    speedC = ShootOnTheMove.EXIT_SPEED_COEFFS;
    tofC = ShootOnTheMove.TOF_COEFFS;
    grid = ShootOnTheMove.D_GRID_M;
    vrMin = ShootOnTheMove.VR_MIN_MPS;
    vrMax = ShootOnTheMove.VR_MAX_MPS;
    corrX = ShootOnTheMove.RPM_CORRECTION_SPEEDS_MPS;
    corrY = ShootOnTheMove.RPM_CORRECTION_FACTORS;
    rpmTol = ShootOnTheMove.RPM_TOLERANCE;
    hoodTol = ShootOnTheMove.HOOD_TOLERANCE_DEG;
    hdgCap = ShootOnTheMove.HEADING_TOLERANCE_MAX_RAD;
    halfW = ShootOnTheMove.GOAL_HALF_WIDTH_M;
    ballR = ShootOnTheMove.BALL_RADIUS_M;
    margin = ShootOnTheMove.AIM_MARGIN_M;
    maxTurn = ShootOnTheMove.MAX_FIRE_TURN_RATE_RAD_S;
    maxApproach = ShootOnTheMove.MAX_APPROACH_ANGLE_DEG;

    // Simple analytic setup. Degree 1 terms are ordered 1, vr, d.
    // theta = 60 - 2d, s = 4 + d - 0.5 vr, tof = 0.5 + 0.1 d.
    ShootOnTheMove.MECHANISM_DELAY_S = 0;
    ShootOnTheMove.SHOOTER_MOUNT_YAW_RAD = Math.PI;
    ShootOnTheMove.SHOOTER_OFFSET_FORWARD_M = 0;
    ShootOnTheMove.SHOOTER_OFFSET_LEFT_M = 0;
    ShootOnTheMove.HOOD_ANGLE_OFFSET_DEG = 0;
    ShootOnTheMove.RPM_PER_MPS = 500;
    ShootOnTheMove.LAUNCH_MIN_DEG = 51;
    ShootOnTheMove.LAUNCH_MAX_DEG = 89;
    ShootOnTheMove.POLY_DEGREE = 1;
    ShootOnTheMove.HOOD_ANGLE_COEFFS = new double[] {60, 0, -2};
    ShootOnTheMove.EXIT_SPEED_COEFFS = new double[] {4, -0.5, 1};
    ShootOnTheMove.TOF_COEFFS = new double[] {0.5, 0, 0.1};
    ShootOnTheMove.D_GRID_M = new double[] {0, 10};
    ShootOnTheMove.VR_MIN_MPS = new double[] {-10, -10};
    ShootOnTheMove.VR_MAX_MPS = new double[] {10, 10};
    ShootOnTheMove.RPM_CORRECTION_SPEEDS_MPS = new double[] {};
    ShootOnTheMove.RPM_CORRECTION_FACTORS = new double[] {};
  }

  @After
  public void tearDown() {
    ShootOnTheMove.MECHANISM_DELAY_S = delay;
    ShootOnTheMove.SHOOTER_MOUNT_YAW_RAD = mount;
    ShootOnTheMove.SHOOTER_OFFSET_FORWARD_M = offF;
    ShootOnTheMove.SHOOTER_OFFSET_LEFT_M = offL;
    ShootOnTheMove.HOOD_ANGLE_OFFSET_DEG = hoodOff;
    ShootOnTheMove.RPM_PER_MPS = rpmPer;
    ShootOnTheMove.LAUNCH_MIN_DEG = lmin;
    ShootOnTheMove.LAUNCH_MAX_DEG = lmax;
    ShootOnTheMove.POLY_DEGREE = degree;
    ShootOnTheMove.POSE_HISTORY_SIZE = histSize;
    ShootOnTheMove.HOOD_ANGLE_COEFFS = hoodC;
    ShootOnTheMove.EXIT_SPEED_COEFFS = speedC;
    ShootOnTheMove.TOF_COEFFS = tofC;
    ShootOnTheMove.D_GRID_M = grid;
    ShootOnTheMove.VR_MIN_MPS = vrMin;
    ShootOnTheMove.VR_MAX_MPS = vrMax;
    ShootOnTheMove.RPM_CORRECTION_SPEEDS_MPS = corrX;
    ShootOnTheMove.RPM_CORRECTION_FACTORS = corrY;
    ShootOnTheMove.RPM_TOLERANCE = rpmTol;
    ShootOnTheMove.HOOD_TOLERANCE_DEG = hoodTol;
    ShootOnTheMove.HEADING_TOLERANCE_MAX_RAD = hdgCap;
    ShootOnTheMove.GOAL_HALF_WIDTH_M = halfW;
    ShootOnTheMove.BALL_RADIUS_M = ballR;
    ShootOnTheMove.AIM_MARGIN_M = margin;
    ShootOnTheMove.MAX_FIRE_TURN_RATE_RAD_S = maxTurn;
    ShootOnTheMove.MAX_APPROACH_ANGLE_DEG = maxApproach;
  }

  // Goal 2 m ahead (+x) of the origin, facing back toward -x (toward the shooter).
  private static Goal goal() {
    return new Goal(2, 0, Math.PI);
  }

  private static Pose origin() {
    return new Pose(0, 0, 0);
  }

  private static double angleDiff(double a, double b) {
    return ShootOnTheMove.wrapAngle(a - b);
  }

  // ---- Polynomials and domain -----------------------------------------

  @Test
  public void evalPolyTermOrder() {
    // Degree 2 terms: 1, vr, vr^2, d, d*vr, d^2.
    double[] c = {1, 2, 3, 4, 5, 6};
    double d = 2;
    double vr = 3;
    double expected = 1 + 2 * vr + 3 * vr * vr + 4 * d + 5 * d * vr + 6 * d * d;
    assertEquals(expected, ShootOnTheMove.evalPoly(c, 2, d, vr), EPS);
  }

  @Test
  public void evalPolyDegree1() {
    assertEquals(60 - 2 * 3.0, ShootOnTheMove.evalPoly(ShootOnTheMove.HOOD_ANGLE_COEFFS, 1, 3, 7), EPS);
    assertEquals(4 + 3 - 0.5 * 2, ShootOnTheMove.evalPoly(ShootOnTheMove.EXIT_SPEED_COEFFS, 1, 3, 2), EPS);
  }

  @Test
  public void inDomainEdgesAndNaN() {
    ShootOnTheMove.D_GRID_M = new double[] {1, 2, 3};
    ShootOnTheMove.VR_MIN_MPS = new double[] {-1, -2, Double.NaN};
    ShootOnTheMove.VR_MAX_MPS = new double[] {1, 2, Double.NaN};
    assertTrue(ShootOnTheMove.inDomain(1, 0));
    assertTrue(ShootOnTheMove.inDomain(1, 1));
    assertTrue(ShootOnTheMove.inDomain(1, -1));
    assertFalse(ShootOnTheMove.inDomain(1, 1.01));
    // Midway the band is +-1.5.
    assertTrue(ShootOnTheMove.inDomain(1.5, 1.5));
    assertFalse(ShootOnTheMove.inDomain(1.5, 1.51));
    assertTrue(ShootOnTheMove.inDomain(2, -2));
    // d off the grid.
    assertFalse(ShootOnTheMove.inDomain(0.99, 0));
    assertFalse(ShootOnTheMove.inDomain(3.01, 0));
    // NaN band edge (at d=3 and in the segment leading to it).
    assertFalse(ShootOnTheMove.inDomain(3, 0));
    assertFalse(ShootOnTheMove.inDomain(2.5, 0));
    // NaN input.
    assertFalse(ShootOnTheMove.inDomain(Double.NaN, 0));
    assertFalse(ShootOnTheMove.inDomain(1.5, Double.NaN));
  }

  @Test
  public void defaultPlaceholderGridHasNaNAtFirstPoint() {
    // Restore the shipped defaults for this check only.
    ShootOnTheMove.D_GRID_M = grid;
    ShootOnTheMove.VR_MIN_MPS = vrMin;
    ShootOnTheMove.VR_MAX_MPS = vrMax;
    assertEquals(31, grid.length);
    assertEquals(0.5, grid[0], EPS);
    assertEquals(3.5, grid[30], EPS);
    assertFalse(ShootOnTheMove.inDomain(0.5, 0));
    assertTrue(ShootOnTheMove.inDomain(1.0, 0));
    assertTrue(ShootOnTheMove.inDomain(3.5, 1.5));
  }

  // ---- RPM ------------------------------------------------------------

  @Test
  public void rpmLinearWithEmptyCorrection() {
    assertEquals(2000, ShootOnTheMove.rpmForSpeed(4), EPS);
  }

  @Test
  public void rpmCorrectionMapInterpolatesAndClamps() {
    ShootOnTheMove.RPM_CORRECTION_SPEEDS_MPS = new double[] {4, 8};
    ShootOnTheMove.RPM_CORRECTION_FACTORS = new double[] {1.0, 1.2};
    assertEquals(2 * 500 * 1.0, ShootOnTheMove.rpmForSpeed(2), EPS); // clamped low
    assertEquals(4 * 500 * 1.0, ShootOnTheMove.rpmForSpeed(4), EPS);
    assertEquals(6 * 500 * 1.1, ShootOnTheMove.rpmForSpeed(6), EPS); // midpoint
    assertEquals(10 * 500 * 1.2, ShootOnTheMove.rpmForSpeed(10), EPS); // clamped high
  }

  // ---- Solve: velocity cases ------------------------------------------

  @Test
  public void stationary() {
    Solution s = ShootOnTheMove.solve(origin(), 0, 0, 0, goal());
    assertTrue(s.valid);
    assertEquals(Reason.OK, s.reason);
    assertEquals(2, s.distanceM, EPS);
    assertEquals(0, s.bearingRad, EPS);
    assertEquals(0, s.radialVelMps, EPS);
    assertEquals(0, s.tangentialVelMps, EPS);
    assertEquals(56, s.rawLaunchDeg, EPS);
    assertEquals(6, s.rawExitSpeedMps, EPS);
    assertEquals(56, s.launchDeg, EPS);
    assertEquals(6, s.exitSpeedMps, EPS);
    assertEquals(0.7, s.tofS, EPS);
    assertEquals(0, angleDiff(s.headingTargetRad, s.bearingRad - Math.PI), EPS);
    assertEquals(56, s.hoodCommandDeg, EPS);
    assertEquals(6 * 500, s.targetRpm, EPS);
    assertEquals(0, s.approachAngleRad, EPS);
  }

  @Test
  public void pureRadialClosingIsPositive() {
    // Moving +x toward the goal at 1 m/s: v_r = +1, so s = 4 + 2 - 0.5 = 5.5.
    Solution s = ShootOnTheMove.solve(origin(), 1, 0, 0, goal());
    assertEquals(1, s.radialVelMps, EPS);
    assertEquals(0, s.tangentialVelMps, EPS);
    assertEquals(5.5, s.rawExitSpeedMps, EPS);
    assertEquals(5.5, s.exitSpeedMps, EPS);
    assertEquals(56, s.launchDeg, EPS);
    assertEquals(0, angleDiff(s.headingTargetRad, -Math.PI), EPS); // unchanged

    // Moving away: v_r = -1, s = 6.5.
    Solution away = ShootOnTheMove.solve(origin(), -1, 0, 0, goal());
    assertEquals(-1, away.radialVelMps, EPS);
    assertEquals(6.5, away.rawExitSpeedMps, EPS);
  }

  @Test
  public void pureTangential() {
    double vt = 0.5;
    Solution s = ShootOnTheMove.solve(origin(), 0, vt, 0, goal());
    assertEquals(vt, s.tangentialVelMps, EPS);
    assertEquals(0, s.radialVelMps, EPS);
    double th = Math.toRadians(56);
    double sp = 6;
    double h0 = sp * Math.cos(th);
    double offset = Math.atan2(-vt, h0);
    assertEquals(0, angleDiff(s.headingTargetRad, 0 + offset - Math.PI), EPS);
    assertEquals(Math.hypot(sp, vt), s.exitSpeedMps, EPS);
    // Vertical component preserved.
    assertEquals(sp * Math.sin(th), s.exitSpeedMps * Math.sin(Math.toRadians(s.launchDeg)), EPS);
    // Compensated horizontal magnitude is hypot(h0, vt).
    assertEquals(
        Math.hypot(h0, vt), s.exitSpeedMps * Math.cos(Math.toRadians(s.launchDeg)), EPS);
  }

  @Test
  public void compensationVectorIdentity() {
    // relative horizontal velocity (rotated into the field) + robot velocity
    // must point along u with magnitude s*cos(theta).
    double[][] cases = {{0.5, 3.0}, {-1.0, 0.7}, {0.0, -2.0}, {2.0, 0.0}};
    for (double[] v : cases) {
      double launch = 57;
      double speed = 6.3;
      double vr = v[0];
      double vt = v[1];
      Compensation c = ShootOnTheMove.compensate(launch, speed, vt);
      double hMag = c.speedMps * Math.cos(Math.toRadians(c.launchDeg));
      // Frame with u along +x and t along +y; shooter yaw offset is c.headingOffsetRad.
      double rx = hMag * Math.cos(c.headingOffsetRad);
      double ry = hMag * Math.sin(c.headingOffsetRad);
      double gx = rx + vr;
      double gy = ry + vt;
      double h0 = speed * Math.cos(Math.toRadians(launch));
      assertEquals(0, gy, 1e-9);
      // The radial part of the ground velocity is h0 + vr (the polynomials account for vr).
      assertEquals(h0 + vr, gx, 1e-9);
      assertEquals(
          speed * Math.sin(Math.toRadians(launch)),
          c.speedMps * Math.sin(Math.toRadians(c.launchDeg)),
          1e-9);
    }
  }

  @Test
  public void combinedRadialAndTangential() {
    // Goal 2 m ahead; robot moving +x at 0.4 (closing) and +y at 0.6 (CCW side).
    Solution s = ShootOnTheMove.solve(origin(), 0.4, 0.6, 0, goal());
    assertEquals(0.4, s.radialVelMps, EPS);
    assertEquals(0.6, s.tangentialVelMps, EPS);
    double sp = 4 + 2 - 0.5 * 0.4;
    double th = 60 - 4;
    assertEquals(sp, s.rawExitSpeedMps, EPS);
    assertEquals(th, s.rawLaunchDeg, EPS);
    Compensation c = ShootOnTheMove.compensate(th, sp, 0.6);
    assertEquals(c.launchDeg, s.launchDeg, EPS);
    assertEquals(c.speedMps, s.exitSpeedMps, EPS);
    assertEquals(0, angleDiff(s.headingTargetRad, c.headingOffsetRad - Math.PI), EPS);
  }

  @Test
  public void worksForRotatedGeometry() {
    // Goal along +y, robot moving along +x: that is purely tangential, v_t = -1.
    Goal g = new Goal(0, 2, -Math.PI / 2);
    Solution s = ShootOnTheMove.solve(origin(), 1, 0, 0, g);
    assertEquals(Math.PI / 2, s.bearingRad, EPS);
    assertEquals(0, s.radialVelMps, EPS);
    assertEquals(-1, s.tangentialVelMps, EPS);
  }

  @Test
  public void mountYawPiVsZero() {
    Solution back = ShootOnTheMove.solve(origin(), 0, 0, 0, goal());
    assertEquals(0, angleDiff(back.headingTargetRad, -Math.PI), EPS);
    ShootOnTheMove.SHOOTER_MOUNT_YAW_RAD = 0;
    Solution front = ShootOnTheMove.solve(origin(), 0, 0, 0, goal());
    assertEquals(0, angleDiff(front.headingTargetRad, 0), EPS);
  }

  @Test
  public void shooterOffsetChangesDistance() {
    ShootOnTheMove.SHOOTER_OFFSET_FORWARD_M = 0.1;
    // Fires from the back, so the robot faces away from the goal and the shooter
    // (0.1 m forward of centre) sits 0.1 m farther from the goal.
    Solution back = ShootOnTheMove.solve(origin(), 0, 0, 0, goal());
    assertEquals(2.1, back.distanceM, EPS);
    assertEquals(-0.1, back.shooterX, EPS);
    // Fires from the front: shooter is 0.1 m closer.
    ShootOnTheMove.SHOOTER_MOUNT_YAW_RAD = 0;
    Solution front = ShootOnTheMove.solve(origin(), 0, 0, 0, goal());
    assertEquals(1.9, front.distanceM, EPS);
    assertEquals(0.1, front.shooterX, EPS);
  }

  @Test
  public void shooterLeftOffsetRotatesWithHeading() {
    ShootOnTheMove.SHOOTER_OFFSET_LEFT_M = 0.1;
    ShootOnTheMove.SHOOTER_MOUNT_YAW_RAD = 0;
    // Heading ends up near 0 (aim shifts slightly with the offset), so left is ~+y.
    Solution s = ShootOnTheMove.solve(origin(), 0, 0, 0, goal());
    assertEquals(0.1, s.shooterY, 1e-3);
  }

  @Test
  public void mechanismDelayMovesPredictedPose() {
    ShootOnTheMove.MECHANISM_DELAY_S = 0.2;
    Solution s = ShootOnTheMove.solve(new Pose(0, 0, 0.1), 1, -0.5, 2, goal());
    assertEquals(0.2, s.predictedPose.x, EPS);
    assertEquals(-0.1, s.predictedPose.y, EPS);
    assertEquals(0.1 + 0.4, s.predictedPose.heading, EPS);
    // Shooter offsets are zero, so the shooter sits at the predicted position.
    assertEquals(0.2, s.shooterX, EPS);
    assertEquals(-0.1, s.shooterY, EPS);
  }

  // ---- Invalid reasons ------------------------------------------------

  @Test
  public void outOfRange() {
    Solution s = ShootOnTheMove.solve(new Pose(-20, 0, 0), 0, 0, 0, goal());
    assertFalse(s.valid);
    assertEquals(Reason.OUT_OF_RANGE, s.reason);
    assertEquals(22, s.distanceM, EPS);
    assertTrue(Double.isNaN(s.rawLaunchDeg));
  }

  @Test
  public void outOfDomain() {
    Solution s = ShootOnTheMove.solve(origin(), 20, 0, 0, goal());
    assertFalse(s.valid);
    assertEquals(Reason.OUT_OF_DOMAIN, s.reason);
    assertEquals(20, s.radialVelMps, EPS);
  }

  @Test
  public void hoodLimit() {
    // d = 5 gives theta = 50 < 51.
    Solution s = ShootOnTheMove.solve(new Pose(-3, 0, 0), 0, 0, 0, goal());
    assertFalse(s.valid);
    assertEquals(Reason.HOOD_LIMIT, s.reason);
    assertEquals(50, s.launchDeg, EPS); // still filled for telemetry
    assertEquals(50, s.hoodCommandDeg, EPS);
  }

  @Test
  public void badApproach() {
    Goal facingAway = new Goal(2, 0, 0);
    Solution s = ShootOnTheMove.solve(origin(), 0, 0, 0, facingAway);
    assertFalse(s.valid);
    assertEquals(Reason.BAD_APPROACH, s.reason);
    assertEquals(Math.PI, s.approachAngleRad, EPS);
    assertEquals(56, s.launchDeg, EPS); // still computed

    // 60 degrees off the normal is beyond the 45 degree limit; 30 is fine.
    Goal off60 = new Goal(2, 0, Math.PI + Math.toRadians(60));
    assertEquals(Reason.BAD_APPROACH, ShootOnTheMove.solve(origin(), 0, 0, 0, off60).reason);
    Goal off30 = new Goal(2, 0, Math.PI + Math.toRadians(30));
    assertEquals(Reason.OK, ShootOnTheMove.solve(origin(), 0, 0, 0, off30).reason);
  }

  @Test
  public void hoodAngleOffsetShiftsCommand() {
    ShootOnTheMove.HOOD_ANGLE_OFFSET_DEG = 2;
    Solution s = ShootOnTheMove.solve(origin(), 0, 0, 0, goal());
    assertEquals(56, s.launchDeg, EPS);
    assertEquals(54, s.hoodCommandDeg, EPS);
  }

  // ---- Gate -----------------------------------------------------------

  private static GateResult gateOn(Solution s, double rpm, double hood, double heading, double turn) {
    return ShootOnTheMove.gate(s, rpm, hood, heading, turn);
  }

  @Test
  public void gateAllGood() {
    Solution s = ShootOnTheMove.solve(origin(), 0, 0, 0, goal());
    GateResult g = gateOn(s, s.targetRpm, s.hoodCommandDeg, s.headingTargetRad, 0);
    assertTrue(g.allowed);
    assertTrue(g.valid && g.rpmOk && g.hoodOk && g.headingOk && g.turnRateOk);
  }

  @Test
  public void gateEachTermClosesIt() {
    Solution s = ShootOnTheMove.solve(origin(), 0, 0, 0, goal());
    GateResult rpm = gateOn(s, s.targetRpm + 100, s.hoodCommandDeg, s.headingTargetRad, 0);
    assertFalse(rpm.rpmOk);
    assertFalse(rpm.allowed);
    assertTrue(rpm.hoodOk && rpm.headingOk && rpm.turnRateOk);

    GateResult hood = gateOn(s, s.targetRpm, s.hoodCommandDeg + 1, s.headingTargetRad, 0);
    assertFalse(hood.hoodOk);
    assertFalse(hood.allowed);
    assertTrue(hood.rpmOk && hood.headingOk && hood.turnRateOk);

    GateResult hdg = gateOn(s, s.targetRpm, s.hoodCommandDeg, s.headingTargetRad + 0.2, 0);
    assertFalse(hdg.headingOk);
    assertFalse(hdg.allowed);
    assertTrue(hdg.rpmOk && hdg.hoodOk && hdg.turnRateOk);

    GateResult turn = gateOn(s, s.targetRpm, s.hoodCommandDeg, s.headingTargetRad, 2.0);
    assertFalse(turn.turnRateOk);
    assertFalse(turn.allowed);
    assertTrue(turn.rpmOk && turn.hoodOk && turn.headingOk);

    Solution bad = ShootOnTheMove.solve(origin(), 0, 0, 0, new Goal(2, 0, 0));
    GateResult inv = gateOn(bad, bad.targetRpm, bad.hoodCommandDeg, bad.headingTargetRad, 0);
    assertFalse(inv.valid);
    assertFalse(inv.allowed);
  }

  @Test
  public void gateHeadingWrapsAcrossPi() {
    Solution s = ShootOnTheMove.solve(origin(), 0, 0, 0, goal());
    // Target is -pi; +pi is the same direction.
    GateResult g = gateOn(s, s.targetRpm, s.hoodCommandDeg, Math.PI, 0);
    assertTrue(g.headingOk);
  }

  @Test
  public void headingToleranceFormula() {
    ShootOnTheMove.HEADING_TOLERANCE_MAX_RAD = Math.toRadians(3);
    // Far away the window geometry limits it.
    Solution far = ShootOnTheMove.solve(new Pose(-8, 0, 0), 0, 0, 0, goal());
    double expected = Math.atan((0.254 - 0.03556 - 0.03) / far.distanceM);
    assertTrue(expected < Math.toRadians(3));
    assertEquals(expected, gateOn(far, 0, 0, 0, 0).headingToleranceRad, EPS);
    // Close up the cap limits it.
    Solution near = ShootOnTheMove.solve(origin(), 0, 0, 0, goal());
    assertEquals(Math.toRadians(3), gateOn(near, 0, 0, 0, 0).headingToleranceRad, EPS);
    // Never negative.
    ShootOnTheMove.AIM_MARGIN_M = 1.0;
    assertEquals(0.0, gateOn(far, 0, 0, 0, 0).headingToleranceRad, EPS);
  }

  // ---- PoseHistory ----------------------------------------------------

  @Test
  public void historyInterpolates() {
    PoseHistory h = new PoseHistory();
    h.add(1000, new Pose(0, 0, 0));
    h.add(2000, new Pose(2, 4, 1));
    Pose p = h.sample(1500);
    assertNotNull(p);
    assertEquals(1, p.x, EPS);
    assertEquals(2, p.y, EPS);
    assertEquals(0.5, p.heading, EPS);
    Pose exact = h.sample(2000);
    assertEquals(2, exact.x, EPS);
    assertEquals(2, h.size());
  }

  @Test
  public void historyHeadingWrapsAcrossPi() {
    PoseHistory h = new PoseHistory();
    h.add(0, new Pose(0, 0, Math.PI - 0.1));
    h.add(100, new Pose(0, 0, -Math.PI + 0.1));
    Pose p = h.sample(50);
    // Halfway through the short way round is +-pi, not 0.
    assertEquals(Math.PI, Math.abs(p.heading), 1e-9);
    Pose q = h.sample(25);
    assertEquals(0, angleDiff(q.heading, Math.PI - 0.05), 1e-9);
  }

  @Test
  public void historyOutOfRange() {
    PoseHistory h = new PoseHistory();
    assertNull(h.sample(0));
    for (int i = 0; i < 5; i++) {
      h.add(1000 + i * 100L, new Pose(i, 0, 0));
    }
    assertNull(h.sample(500)); // too early
    assertNull(h.sample(5000)); // too late
    // Within one sample period (100) of the ends: nearest end sample.
    assertEquals(0, h.sample(950).x, EPS);
    assertEquals(4, h.sample(1450).x, EPS);
    assertNull(h.sample(899));
  }

  @Test
  public void historyRingOverwritesOldest() {
    ShootOnTheMove.POSE_HISTORY_SIZE = 3;
    PoseHistory h = new PoseHistory();
    for (int i = 0; i < 5; i++) {
      h.add(i * 10L, new Pose(i, 0, 0));
    }
    assertEquals(3, h.size());
    assertEquals(2, h.sample(20).x, EPS); // oldest retained
    assertEquals(3, h.sample(30).x, EPS);
    assertEquals(3.5, h.sample(35).x, EPS);
    assertNull(h.sample(0));
  }

  @Test
  public void historyShiftAndClear() {
    PoseHistory h = new PoseHistory();
    h.add(0, new Pose(0, 0, 0));
    h.add(10, new Pose(1, 1, 0));
    h.shift(0.5, -0.25);
    assertEquals(0.5, h.sample(0).x, EPS);
    assertEquals(-0.25, h.sample(0).y, EPS);
    assertEquals(1.5, h.sample(10).x, EPS);
    assertEquals(0.75, h.sample(10).y, EPS);
    h.clear();
    assertEquals(0, h.size());
    assertNull(h.sample(0));
  }

  // ---- Vision ---------------------------------------------------------

  private static final long T0 = 1_000_000_000L;
  private static final long MS = 1_000_000L;

  // Robot used for most vision tests: between the Hives, back toward the red Hive.
  private static final Pose TRUE_POSE = new Pose(0.1, -0.9, -Math.PI / 2);

  // Saved vision statics, restored in tearDownVision.
  private Cell savedRed, savedBlue;
  private boolean savedPoseValid, savedUseDefault;
  private Pose savedHandoff;
  private double[] savedVision;
  private int savedConfirm;
  private ShootOnTheMove.VisionCorrector corrector;

  @Before
  public void setUpVision() {
    savedRed = ShootOnTheMove.upCellRed;
    savedBlue = ShootOnTheMove.upCellBlue;
    savedPoseValid = ShootOnTheMove.poseValid;
    savedUseDefault = ShootOnTheMove.USE_DEFAULT_START_POSE;
    savedHandoff = ShootOnTheMove.handoffPose;
    savedConfirm = ShootOnTheMove.TIP_CONFIRM_FRAMES;
    savedVision = new double[] {
      ShootOnTheMove.MIN_DECISION_MARGIN, ShootOnTheMove.MAX_TAG_RANGE_M,
      ShootOnTheMove.MAX_SPEED_FOR_VISION_MPS, ShootOnTheMove.MAX_OMEGA_FOR_VISION_RAD_S,
      ShootOnTheMove.MAX_FRAME_AGE_S, ShootOnTheMove.MAX_SPREAD_M, ShootOnTheMove.MAX_JUMP_M,
      ShootOnTheMove.CORRECTION_GAIN, ShootOnTheMove.Z_TOL_M, ShootOnTheMove.FIELD_HALF_SIZE_M,
      ShootOnTheMove.ROBOT_HALF_SIZE_M
    };
    ShootOnTheMove.upCellRed = Cell.AUDIENCE;
    ShootOnTheMove.upCellBlue = Cell.SCORING;
    ShootOnTheMove.poseValid = true;
    corrector = new ShootOnTheMove.VisionCorrector();
  }

  @After
  public void tearDownVision() {
    ShootOnTheMove.upCellRed = savedRed;
    ShootOnTheMove.upCellBlue = savedBlue;
    ShootOnTheMove.poseValid = savedPoseValid;
    ShootOnTheMove.USE_DEFAULT_START_POSE = savedUseDefault;
    ShootOnTheMove.handoffPose = savedHandoff;
    ShootOnTheMove.TIP_CONFIRM_FRAMES = savedConfirm;
    ShootOnTheMove.MIN_DECISION_MARGIN = savedVision[0];
    ShootOnTheMove.MAX_TAG_RANGE_M = savedVision[1];
    ShootOnTheMove.MAX_SPEED_FOR_VISION_MPS = savedVision[2];
    ShootOnTheMove.MAX_OMEGA_FOR_VISION_RAD_S = savedVision[3];
    ShootOnTheMove.MAX_FRAME_AGE_S = savedVision[4];
    ShootOnTheMove.MAX_SPREAD_M = savedVision[5];
    ShootOnTheMove.MAX_JUMP_M = savedVision[6];
    ShootOnTheMove.CORRECTION_GAIN = savedVision[7];
    ShootOnTheMove.Z_TOL_M = savedVision[8];
    ShootOnTheMove.FIELD_HALF_SIZE_M = savedVision[9];
    ShootOnTheMove.ROBOT_HALF_SIZE_M = savedVision[10];
  }

  /**
   * What a camera would report for a tag, from first principles: the tag's
   * offset from the camera, projected on the camera's right, forward and up axes
   * (up built as right x forward numerically).
   */
  private static TagObservation observe(
      ShootOnTheMove.Camera cam, Pose robot, int id, Cell up,
      double tagZOverride, long frameNanos) {
    double[] t = ShootOnTheMove.tagFieldPosition(id, up);
    double tz = Double.isNaN(tagZOverride) ? t[2] : tagZOverride;
    double dx = t[0] - robot.x;
    double dy = t[1] - robot.y;
    double c = Math.cos(robot.heading);
    double s = Math.sin(robot.heading);
    double[] p = cam == ShootOnTheMove.Camera.LEFT
        ? ShootOnTheMove.CAM_POS_LEFT_M : ShootOnTheMove.CAM_POS_RIGHT_M;
    double psi = cam == ShootOnTheMove.Camera.LEFT
        ? ShootOnTheMove.CAM_YAW_LEFT_RAD : ShootOnTheMove.CAM_YAW_RIGHT_RAD;
    double phi = ShootOnTheMove.CAM_PITCH_RAD;
    double[] v = {c * dx + s * dy - p[0], -s * dx + c * dy - p[1], tz - p[2]};
    double[] f = {Math.cos(phi) * Math.cos(psi), Math.cos(phi) * Math.sin(psi), Math.sin(phi)};
    double[] r = {Math.sin(psi), -Math.cos(psi), 0};
    double[] u = {
      r[1] * f[2] - r[2] * f[1], r[2] * f[0] - r[0] * f[2], r[0] * f[1] - r[1] * f[0]
    };
    return new TagObservation(id, cam, dot(v, r), dot(v, f), dot(v, u), 100, frameNanos);
  }

  private static double dot(double[] a, double[] b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  }

  /** Seen by the left camera at T0 + 50 ms. */
  private static TagObservation observe(int id, Cell up, Pose robot) {
    return observe(ShootOnTheMove.Camera.LEFT, robot, id, up, Double.NaN, T0 + 50 * MS);
  }

  /** Odometry sat still at the given pose from T0 to T0 + 100 ms. */
  private static PoseHistory stillHistory(Pose odom) {
    PoseHistory h = new PoseHistory();
    h.add(T0, odom);
    h.add(T0 + 100 * MS, odom);
    return h;
  }

  private static Pose offsetBy(Pose p, double dx, double dy) {
    return new Pose(p.x + dx, p.y + dy, p.heading);
  }

  private ShootOnTheMove.VisionResult run(List<TagObservation> obs, Pose odom, boolean valid) {
    return corrector.process(obs, stillHistory(odom), odom, 0, 0, 0, valid, T0 + 100 * MS);
  }

  private static List<TagObservation> list(TagObservation... o) {
    return new ArrayList<>(Arrays.asList(o));
  }

  @Test
  public void tagFieldPositions() {
    Cell aud = Cell.AUDIENCE;
    Cell sco = Cell.SCORING;
    // RED SCORING tag 30 with AUDIENCE up: down cluster, -x side.
    double[] a = ShootOnTheMove.tagFieldPosition(30, aud);
    assertEquals(-0.2949, a[0], EPS);
    assertEquals(-6.25 * 0.0254, a[1], EPS);
    assertEquals(0.9022, a[2], EPS);
    // Same tag with SCORING up: up cluster, -x side.
    double[] b = ShootOnTheMove.tagFieldPosition(30, sco);
    assertEquals(-0.3329, b[0], EPS);
    assertEquals(1.2646, b[2], EPS);
    // RED AUDIENCE tag 34, AUDIENCE up: up cluster on +x.
    double[] c = ShootOnTheMove.tagFieldPosition(34, aud);
    assertEquals(0.3329, c[0], EPS);
    assertEquals(-19.25 * 0.0254, c[1], EPS);
    assertEquals(1.2646, c[2], EPS);
    // BLUE AUDIENCE tag 41, SCORING up: down cluster on +x.
    double[] d = ShootOnTheMove.tagFieldPosition(41, sco);
    assertEquals(0.2949, d[0], EPS);
    assertEquals(19.25 * 0.0254, d[1], EPS);
    assertEquals(0.9022, d[2], EPS);
    // BLUE SCORING tag 45.
    assertEquals(6.25 * 0.0254, ShootOnTheMove.tagFieldPosition(45, sco)[1], EPS);
    assertNull(ShootOnTheMove.tagFieldPosition(29, aud));
    assertNull(ShootOnTheMove.tagFieldPosition(46, aud));
    assertNull(ShootOnTheMove.tagFieldPosition(-1, aud));

    assertEquals(ShootOnTheMove.Alliance.RED, ShootOnTheMove.tagAlliance(37));
    assertEquals(ShootOnTheMove.Alliance.BLUE, ShootOnTheMove.tagAlliance(38));
    assertNull(ShootOnTheMove.tagAlliance(20));
    assertEquals(Cell.SCORING, ShootOnTheMove.tagCell(33));
    assertEquals(Cell.AUDIENCE, ShootOnTheMove.tagCell(34));
    assertEquals(Cell.AUDIENCE, ShootOnTheMove.tagCell(41));
    assertEquals(Cell.SCORING, ShootOnTheMove.tagCell(42));
    assertNull(ShootOnTheMove.tagCell(0));
  }

  @Test
  public void cameraAxesPointTheRightWay() {
    // CameraL looks backward and to the left; its image-right points forward-left.
    double[] p = ShootOnTheMove.CAM_POS_LEFT_M;
    double[] ahead = ShootOnTheMove.tagInRobotFrame(ShootOnTheMove.Camera.LEFT, 0, 1, 0);
    assertTrue(ahead[0] < p[0] && ahead[1] > p[1] && ahead[2] > p[2]);
    double[] right = ShootOnTheMove.tagInRobotFrame(ShootOnTheMove.Camera.LEFT, 1, 0, 0);
    assertTrue(right[0] > p[0] && right[1] > p[1]);
    assertEquals(p[2], right[2], EPS);
    // CameraR is the mirror image.
    double[] q = ShootOnTheMove.CAM_POS_RIGHT_M;
    double[] aheadR = ShootOnTheMove.tagInRobotFrame(ShootOnTheMove.Camera.RIGHT, 0, 1, 0);
    assertTrue(aheadR[0] < q[0] && aheadR[1] < q[1] && aheadR[2] > q[2]);
    // Image-up raises the point by cos(pitch) per metre.
    double[] up = ShootOnTheMove.tagInRobotFrame(ShootOnTheMove.Camera.LEFT, 0, 0, 1);
    assertEquals(p[2] + Math.cos(ShootOnTheMove.CAM_PITCH_RAD), up[2], EPS);
  }

  @Test
  public void extrinsicsRoundTripRecoversPose() {
    double[] headings = {-Math.PI / 2, 0.0, 2.0, -2.7};
    Cell[] reds = {Cell.AUDIENCE, Cell.SCORING};
    for (double h : headings) {
      for (Cell redUp : reds) {
        ShootOnTheMove.upCellRed = redUp;
        ShootOnTheMove.upCellBlue = ShootOnTheMove.otherCell(redUp);
        corrector.reset();
        Pose truth = new Pose(0.1, -0.9, h);
        // The odometry is wrong by (+0.04, -0.03).
        Pose odom = offsetBy(truth, 0.04, -0.03);
        List<TagObservation> obs = new ArrayList<>();
        for (ShootOnTheMove.Camera cam : ShootOnTheMove.Camera.values()) {
          for (int id = 30; id <= 45; id++) {
            Cell up = ShootOnTheMove.tagAlliance(id) == ShootOnTheMove.Alliance.RED
                ? ShootOnTheMove.upCellRed : ShootOnTheMove.upCellBlue;
            obs.add(observe(cam, truth, id, up, Double.NaN, T0 + 50 * MS));
          }
        }
        ShootOnTheMove.VisionResult r = run(obs, odom, true);
        assertTrue("heading " + h + " " + r.reason + " " + r.firstTagReject, r.accepted);
        assertEquals(32, r.tagsUsed);
        assertEquals(-0.04, r.dx, 1e-3);
        assertEquals(0.03, r.dy, 1e-3);
        assertEquals(0.0, r.spread, 1e-6);
      }
    }
  }

  @Test
  public void measuredZDoesNotDependOnHeadingOrPosition() {
    for (double h : new double[] {0, 1, -2}) {
      TagObservation o = observe(
          ShootOnTheMove.Camera.RIGHT, new Pose(-0.3, 0.2, h), 36, Cell.AUDIENCE, Double.NaN, 0);
      double[] robotPt = ShootOnTheMove.tagInRobotFrame(o.camera, o.x, o.y, o.z);
      assertEquals(1.2646, robotPt[2], 1e-9);
    }
  }

  @Test
  public void tipDetectionNeedsThreeFrames() {
    // Red Hive says AUDIENCE up; the tags actually show SCORING up.
    assertEquals(Cell.AUDIENCE, ShootOnTheMove.upCellRed);
    for (int frame = 1; frame <= 3; frame++) {
      ShootOnTheMove.VisionResult r =
          run(list(observe(31, Cell.SCORING, TRUE_POSE)), TRUE_POSE, true);
      if (frame < 3) {
        assertEquals(Cell.AUDIENCE, ShootOnTheMove.upCellRed);
        assertFalse(r.redFlipped);
      } else {
        assertEquals(Cell.SCORING, ShootOnTheMove.upCellRed);
        assertTrue(r.redFlipped);
        assertFalse(r.blueFlipped);
      }
    }
    // The goal follows the up Cell.
    assertEquals(ShootOnTheMove.RED_SCORING_GOAL,
        ShootOnTheMove.currentGoal(ShootOnTheMove.Alliance.RED));
    assertEquals(Cell.SCORING, ShootOnTheMove.upCellBlue); // untouched
  }

  @Test
  public void tipVotesMustBeConsecutive() {
    TagObservation flip = observe(31, Cell.SCORING, TRUE_POSE);
    TagObservation same = observe(31, Cell.AUDIENCE, TRUE_POSE);
    run(list(flip), TRUE_POSE, true);
    run(list(flip), TRUE_POSE, true);
    run(list(same), TRUE_POSE, true); // back to the current state: votes reset
    run(list(flip), TRUE_POSE, true);
    run(list(flip), TRUE_POSE, true);
    assertEquals(Cell.AUDIENCE, ShootOnTheMove.upCellRed);
    run(list(flip), TRUE_POSE, true);
    assertEquals(Cell.SCORING, ShootOnTheMove.upCellRed);
  }

  @Test
  public void tipDetectionReadsDownHeightAsOtherCellUp() {
    // Blue says SCORING up. A BLUE SCORING tag (42) at the down height means AUDIENCE is up.
    ShootOnTheMove.TIP_CONFIRM_FRAMES = 1;
    ShootOnTheMove.VisionResult r =
        run(list(observe(42, Cell.AUDIENCE, TRUE_POSE)), TRUE_POSE, true);
    assertTrue(r.blueFlipped);
    assertEquals(Cell.AUDIENCE, ShootOnTheMove.upCellBlue);
    assertEquals(Cell.AUDIENCE, ShootOnTheMove.upCellRed); // untouched
  }

  @Test
  public void tipConfirmedStateKeepsRunning() {
    // Flipped once, then the tag keeps agreeing: nothing changes back.
    ShootOnTheMove.TIP_CONFIRM_FRAMES = 1;
    run(list(observe(31, Cell.SCORING, TRUE_POSE)), TRUE_POSE, true);
    assertEquals(Cell.SCORING, ShootOnTheMove.upCellRed);
    ShootOnTheMove.VisionResult r =
        run(list(observe(31, Cell.SCORING, TRUE_POSE)), TRUE_POSE, true);
    assertFalse(r.redFlipped);
    assertEquals(Cell.SCORING, ShootOnTheMove.upCellRed);
  }

  private void assertTagRejected(TagObservation o, ShootOnTheMove.TagReject why) {
    ShootOnTheMove.VisionResult r = run(list(o), TRUE_POSE, true);
    assertFalse(r.accepted);
    assertEquals(ShootOnTheMove.VisionReason.NO_VALID_TAGS, r.reason);
    assertEquals(why, r.firstTagReject);
    assertEquals(1, r.tagsRejected);
    assertEquals(0, r.tagsUsed);
    assertNull(r.newPose);
  }

  @Test
  public void rejectsUnknownAndClusterIds() {
    TagObservation good = observe(31, Cell.AUDIENCE, TRUE_POSE);
    assertTagRejected(
        new TagObservation(20, good.camera, good.x, good.y, good.z, 100, good.frameNanos),
        ShootOnTheMove.TagReject.NOT_HIVE_TAG);
    assertTagRejected(
        new TagObservation(-1, good.camera, good.x, good.y, good.z, Double.NaN, good.frameNanos),
        ShootOnTheMove.TagReject.NOT_HIVE_TAG);
    assertTagRejected(
        new TagObservation(31, good.camera, Double.NaN, Double.NaN, Double.NaN, 100,
            good.frameNanos),
        ShootOnTheMove.TagReject.NO_POSE);
  }

  @Test
  public void rejectsLowDecisionMargin() {
    TagObservation g = observe(31, Cell.AUDIENCE, TRUE_POSE);
    assertTagRejected(
        new TagObservation(31, g.camera, g.x, g.y, g.z, 19.9, g.frameNanos),
        ShootOnTheMove.TagReject.LOW_MARGIN);
    assertTrue(run(list(
        new TagObservation(31, g.camera, g.x, g.y, g.z, 20, g.frameNanos)),
        TRUE_POSE, true).accepted);
  }

  @Test
  public void rejectsTooFar() {
    TagObservation g = observe(31, Cell.AUDIENCE, TRUE_POSE);
    assertTagRejected(
        new TagObservation(31, g.camera, 0, 3.5, 0, 100, g.frameNanos),
        ShootOnTheMove.TagReject.TOO_FAR);
    ShootOnTheMove.MAX_TAG_RANGE_M = 0.1;
    assertTagRejected(g, ShootOnTheMove.TagReject.TOO_FAR);
  }

  @Test
  public void rejectsImplausibleZ() {
    // Tag centre 0.5 m up: neither 0.9022 nor 1.2646.
    assertTagRejected(
        observe(ShootOnTheMove.Camera.LEFT, TRUE_POSE, 31, Cell.AUDIENCE, 0.5, T0 + 50 * MS),
        ShootOnTheMove.TagReject.BAD_Z);
    // Just outside the tolerance of the up height.
    assertTagRejected(
        observe(ShootOnTheMove.Camera.LEFT, TRUE_POSE, 34, Cell.AUDIENCE, 1.2646 + 0.11,
            T0 + 50 * MS),
        ShootOnTheMove.TagReject.BAD_Z);
    // Just inside it is fine.
    assertEquals(1, run(list(observe(ShootOnTheMove.Camera.LEFT, TRUE_POSE, 34, Cell.AUDIENCE,
        1.2646 + 0.05, T0 + 50 * MS)), TRUE_POSE, true).tagsUsed);
  }

  @Test
  public void badZObservationCastsNoTipVote() {
    ShootOnTheMove.TIP_CONFIRM_FRAMES = 1;
    run(list(observe(ShootOnTheMove.Camera.LEFT, TRUE_POSE, 31, Cell.SCORING, 0.5,
        T0 + 50 * MS)), TRUE_POSE, true);
    assertEquals(Cell.AUDIENCE, ShootOnTheMove.upCellRed);
  }

  @Test
  public void rejectsWhenMovingTooFastOrTurning() {
    List<TagObservation> obs = list(observe(31, Cell.AUDIENCE, TRUE_POSE));
    PoseHistory h = stillHistory(TRUE_POSE);
    ShootOnTheMove.VisionResult fast =
        corrector.process(obs, h, TRUE_POSE, 0.8, 0.8, 0, true, T0 + 100 * MS);
    assertEquals(ShootOnTheMove.VisionReason.TOO_FAST, fast.reason);
    assertFalse(fast.accepted);
    ShootOnTheMove.VisionResult ok =
        corrector.process(obs, h, TRUE_POSE, 1.0, 0, 0, true, T0 + 100 * MS);
    assertTrue(ok.accepted);
    ShootOnTheMove.VisionResult spin =
        corrector.process(obs, h, TRUE_POSE, 0, 0, 1.1, true, T0 + 100 * MS);
    assertEquals(ShootOnTheMove.VisionReason.TOO_FAST_TURNING, spin.reason);
    ShootOnTheMove.VisionResult spinNeg =
        corrector.process(obs, h, TRUE_POSE, 0, 0, -1.1, true, T0 + 100 * MS);
    assertEquals(ShootOnTheMove.VisionReason.TOO_FAST_TURNING, spinNeg.reason);
  }

  @Test
  public void rejectsNoObservations() {
    ShootOnTheMove.VisionResult r = run(new ArrayList<TagObservation>(), TRUE_POSE, true);
    assertEquals(ShootOnTheMove.VisionReason.NO_OBSERVATIONS, r.reason);
    assertFalse(r.accepted);
  }

  @Test
  public void rejectsStaleFrame() {
    TagObservation o = observe(31, Cell.AUDIENCE, TRUE_POSE); // frame at T0 + 50 ms
    // 0.35 s later than the frame.
    ShootOnTheMove.VisionResult r = corrector.process(
        list(o), stillHistory(TRUE_POSE), TRUE_POSE, 0, 0, 0, true, T0 + 50 * MS + 350 * MS);
    assertEquals(ShootOnTheMove.VisionReason.NO_VALID_TAGS, r.reason);
    assertEquals(ShootOnTheMove.TagReject.FRAME_TOO_OLD, r.firstTagReject);
    // 0.25 s later is fine.
    assertTrue(corrector.process(
        list(o), stillHistory(TRUE_POSE), TRUE_POSE, 0, 0, 0, true, T0 + 50 * MS + 250 * MS)
        .accepted);
  }

  @Test
  public void rejectsFrameOlderThanHistory() {
    TagObservation o = observe(
        ShootOnTheMove.Camera.LEFT, TRUE_POSE, 31, Cell.AUDIENCE, Double.NaN, T0 - 250 * MS);
    ShootOnTheMove.VisionResult r = corrector.process(
        list(o), stillHistory(TRUE_POSE), TRUE_POSE, 0, 0, 0, true, T0);
    assertEquals(ShootOnTheMove.TagReject.NO_HISTORY, r.firstTagReject);
    assertFalse(r.accepted);
    // No history at all.
    ShootOnTheMove.VisionResult none = corrector.process(
        list(observe(31, Cell.AUDIENCE, TRUE_POSE)), new PoseHistory(), TRUE_POSE,
        0, 0, 0, true, T0 + 100 * MS);
    assertEquals(ShootOnTheMove.TagReject.NO_HISTORY, none.firstTagReject);
  }

  @Test
  public void medianOfOddAndEvenCounts() {
    // Each observation is made from a robot a little off the true pose, so its own
    // correction equals that offset. Three tags: median of {0, 0.02, 0.08} = 0.02.
    ShootOnTheMove.VisionResult r = run(list(
        observe(31, Cell.AUDIENCE, offsetBy(TRUE_POSE, 0.00, 0.0)),
        observe(32, Cell.AUDIENCE, offsetBy(TRUE_POSE, 0.02, 0.01)),
        observe(33, Cell.AUDIENCE, offsetBy(TRUE_POSE, 0.08, -0.01))), TRUE_POSE, true);
    assertTrue(r.accepted);
    assertEquals(0.02, r.dx, 1e-3);
    assertEquals(0.0, r.dy, 1e-3);
    assertEquals(3, r.tagsUsed);
    assertEquals(Math.hypot(0.06, 0.01), r.spread, 1e-3);
    assertEquals(3, r.usedIds.length);

    // Four tags: mean of the middle two.
    ShootOnTheMove.VisionResult e = run(list(
        observe(31, Cell.AUDIENCE, offsetBy(TRUE_POSE, 0.00, 0.0)),
        observe(32, Cell.AUDIENCE, offsetBy(TRUE_POSE, 0.02, 0.0)),
        observe(33, Cell.AUDIENCE, offsetBy(TRUE_POSE, 0.04, 0.0)),
        observe(35, Cell.AUDIENCE, offsetBy(TRUE_POSE, 0.06, 0.0))), TRUE_POSE, true);
    assertTrue(e.accepted);
    assertEquals(0.03, e.dx, 1e-3);
  }

  @Test
  public void rejectsWhenTagsDisagree() {
    ShootOnTheMove.VisionResult r = run(list(
        observe(31, Cell.AUDIENCE, offsetBy(TRUE_POSE, 0.0, 0.0)),
        observe(32, Cell.AUDIENCE, offsetBy(TRUE_POSE, 0.0, 0.0)),
        observe(33, Cell.AUDIENCE, offsetBy(TRUE_POSE, 0.30, 0.0))), TRUE_POSE, true);
    assertEquals(ShootOnTheMove.VisionReason.SPREAD, r.reason);
    assertFalse(r.accepted);
    assertEquals(0.30, r.spread, 1e-3);
    assertNull(r.newPose);
    // Loosening the limit lets the same data through, at the median.
    ShootOnTheMove.MAX_SPREAD_M = 0.35;
    ShootOnTheMove.VisionResult ok = run(list(
        observe(31, Cell.AUDIENCE, offsetBy(TRUE_POSE, 0.0, 0.0)),
        observe(32, Cell.AUDIENCE, offsetBy(TRUE_POSE, 0.0, 0.0)),
        observe(33, Cell.AUDIENCE, offsetBy(TRUE_POSE, 0.30, 0.0))), TRUE_POSE, true);
    assertTrue(ok.accepted);
    assertEquals(0.0, ok.dx, 1e-3);
  }

  @Test
  public void correctionIsMeasuredFromThePastPose() {
    // Odometry went from x=0 to x=1 (heading -pi/2 to -pi/2+0.2) over one second.
    Pose start = new Pose(0.0, -0.9, -Math.PI / 2);
    Pose end = new Pose(1.0, -0.9, -Math.PI / 2 + 0.2);
    PoseHistory h = new PoseHistory();
    h.add(T0, start);
    h.add(T0 + 1000 * MS, end);
    // At the frame (0.5 s) the odometry said (0.5, -0.9, -pi/2+0.1); the real robot was
    // 0.10 m ahead in x and 0.05 m ahead in y of that.
    Pose truthAtFrame = new Pose(0.6, -0.85, -Math.PI / 2 + 0.1);
    TagObservation o = observe(
        ShootOnTheMove.Camera.RIGHT, truthAtFrame, 35, Cell.AUDIENCE, Double.NaN,
        T0 + 500 * MS);
    ShootOnTheMove.VisionResult r = corrector.process(
        list(o), h, end, 0.5, 0, 0, true, T0 + 600 * MS);
    assertTrue(r.accepted);
    assertEquals(0.10, r.dx, 1e-3);
    assertEquals(0.05, r.dy, 1e-3);
    // Applied to the CURRENT pose, half of it.
    assertEquals(1.05, r.newPose.x, 1e-3);
    assertEquals(-0.875, r.newPose.y, 1e-3);
    assertEquals(end.heading, r.newPose.heading, EPS);
  }

  @Test
  public void bootstrapAcceptsLargeJumpAtFullGain() {
    ShootOnTheMove.poseValid = false;
    Pose odom = offsetBy(TRUE_POSE, 0.9, 0.7); // far beyond MAX_JUMP_M
    ShootOnTheMove.VisionResult r = run(list(observe(31, Cell.AUDIENCE, TRUE_POSE)), odom, false);
    assertTrue(r.accepted);
    assertEquals(1.0, r.gain, EPS);
    assertEquals(-0.9, r.dx, 1e-3);
    assertEquals(-0.7, r.dy, 1e-3);
    assertEquals(TRUE_POSE.x, r.newPose.x, 1e-3);
    assertEquals(TRUE_POSE.y, r.newPose.y, 1e-3);
    assertTrue(ShootOnTheMove.poseValid);
  }

  @Test
  public void bootstrapStillNeedsAGoodFix() {
    ShootOnTheMove.poseValid = false;
    ShootOnTheMove.VisionResult r = run(list(
        observe(31, Cell.AUDIENCE, offsetBy(TRUE_POSE, 0.0, 0.0)),
        observe(32, Cell.AUDIENCE, offsetBy(TRUE_POSE, 0.5, 0.0))), TRUE_POSE, false);
    assertEquals(ShootOnTheMove.VisionReason.SPREAD, r.reason);
    assertFalse(ShootOnTheMove.poseValid);
  }

  @Test
  public void rejectsJumpOnceThePoseIsValid() {
    Pose odom = offsetBy(TRUE_POSE, 0.6, 0.0);
    ShootOnTheMove.VisionResult r = run(list(observe(31, Cell.AUDIENCE, TRUE_POSE)), odom, true);
    assertEquals(ShootOnTheMove.VisionReason.JUMP, r.reason);
    assertFalse(r.accepted);
    // 0.4 m is fine.
    Pose near = offsetBy(TRUE_POSE, 0.4, 0.0);
    assertTrue(run(list(observe(31, Cell.AUDIENCE, TRUE_POSE)), near, true).accepted);
  }

  @Test
  public void appliesTheGain() {
    Pose odom = offsetBy(TRUE_POSE, 0.2, -0.1);
    ShootOnTheMove.VisionResult r = run(list(observe(31, Cell.AUDIENCE, TRUE_POSE)), odom, true);
    assertTrue(r.accepted);
    assertEquals(0.5, r.gain, EPS);
    assertEquals(-0.2, r.dx, 1e-3);
    assertEquals(0.1, r.dy, 1e-3);
    assertEquals(odom.x - 0.1, r.newPose.x, 1e-3);
    assertEquals(odom.y + 0.05, r.newPose.y, 1e-3);
    assertEquals(odom.heading, r.newPose.heading, EPS);

    ShootOnTheMove.CORRECTION_GAIN = 0.25;
    ShootOnTheMove.VisionResult q = run(list(observe(31, Cell.AUDIENCE, TRUE_POSE)), odom, true);
    assertEquals(odom.x - 0.05, q.newPose.x, 1e-3);
  }

  @Test
  public void rejectsPositionOutsideTheField() {
    // |x| limit is 1.83 - 0.15 = 1.68.
    Pose out = new Pose(1.75, -0.9, -Math.PI / 2);
    ShootOnTheMove.VisionResult r = run(list(observe(31, Cell.AUDIENCE, out)), out, true);
    assertEquals(ShootOnTheMove.VisionReason.OUT_OF_FIELD, r.reason);
    assertFalse(r.accepted);
    Pose outY = new Pose(0.1, 1.70, -Math.PI / 2);
    assertEquals(ShootOnTheMove.VisionReason.OUT_OF_FIELD,
        run(list(observe(31, Cell.AUDIENCE, outY)), outY, true).reason);
    Pose inside = new Pose(1.6, -0.9, -Math.PI / 2);
    assertTrue(run(list(observe(31, Cell.AUDIENCE, inside)), inside, true).accepted);
    // Still checked while bootstrapping.
    ShootOnTheMove.poseValid = false;
    assertEquals(ShootOnTheMove.VisionReason.OUT_OF_FIELD,
        run(list(observe(31, Cell.AUDIENCE, out)), out, false).reason);
    assertFalse(ShootOnTheMove.poseValid);
  }
}
