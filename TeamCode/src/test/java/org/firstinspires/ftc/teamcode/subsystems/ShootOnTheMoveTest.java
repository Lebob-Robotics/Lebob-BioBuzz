package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.subsystems.ShootOnTheMove.Compensation;
import org.firstinspires.ftc.teamcode.subsystems.ShootOnTheMove.GateResult;
import org.firstinspires.ftc.teamcode.subsystems.ShootOnTheMove.Goal;
import org.firstinspires.ftc.teamcode.subsystems.ShootOnTheMove.Pose;
import org.firstinspires.ftc.teamcode.subsystems.ShootOnTheMove.PoseHistory;
import org.firstinspires.ftc.teamcode.subsystems.ShootOnTheMove.Reason;
import org.firstinspires.ftc.teamcode.subsystems.ShootOnTheMove.Solution;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

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
}
