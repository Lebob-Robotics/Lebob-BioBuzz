package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import org.firstinspires.ftc.teamcode.pedro.CapturedDrivetrain;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.RobotLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.ShootOnTheMoveConstants;
import org.junit.Test;

/**
 * Shoot Auto's Pedro park on a robot modelled on 13 Nov 220620's park: 1.75 m/s top speed forward (1.4 sideways),
 * 0.3 s to reach a new speed, at most 4.3 m/s^2, a 0.068 s loop, and the pose and speed a loop old when used.
 * The powers go through the Robot's stick curve, as on the robot.
 */
public class ShootAutoPedroTest {
  private static final double DT = 0.068;
  private static final double TOP_FORWARD_MPS = 1.75;
  private static final double TOP_STRAFE_MPS = 1.4;
  private static final double LAG_S = 0.3;
  private static final double MAX_ACCEL = 4.3;

  @Test
  public void stickForUndoesTheStickCurve() {
    for (double p : new double[] {0.2, 0.3, 0.5, 0.9}) { // Below STICK_MIN_POWER (0.18) the wheels don't turn
      double[] stick = ShootAuto.stickFor(new double[] {p * 0.6, -p * 0.8});
      double[] power = MecanumDriveSubsystem.shapeTranslation(stick[0], stick[1]);
      assertEquals(p, Math.hypot(power[0], power[1]), 1e-3);
    }
  }

  @Test
  public void parkPathKeepsClearOfTheHiveAndTheFlower() {
    double f = ShootOnTheMoveConstants.ALLIANCE_FORWARD_RED_RAD;
    Path path = ShootAuto.pedroParkPath(1.583, -0.556, -0.17, ShootAuto.parkRoute(f, 1.583, -0.556));
    for (int i = 0; i <= 200; i++) {
      Pose p = path.get(i / 200.0);
      double x = p.x() * RobotLocalizer.M_PER_IN;
      double y = p.y() * RobotLocalizer.M_PER_IN;
      assertFalse("in the Hive at " + x + ", " + y, Math.abs(x) < 0.80 && Math.abs(y) < 1.00);
      assertFalse("into the Flower at " + x + ", " + y, Math.abs(x - 0.61) < 0.35 && y < -1.30); // Red's
    }
  }

  @Test
  public void redParksFrom220620sShotSpotWithoutRunningPast() {
    park(ShootOnTheMoveConstants.ALLIANCE_FORWARD_RED_RAD, 1.583, -0.556, -0.17);
  }

  @Test
  public void blueParksFrom200130sShotSpotWithoutRunningPast() {
    park(ShootOnTheMoveConstants.ALLIANCE_FORWARD_BLUE_RAD, -1.477, 0.468, Math.PI - 0.2);
  }

  private static void park(double f, double x, double y, double h) {
    double[][] route = ShootAuto.parkRoute(f, x, y);
    double[] goal = route[route.length - 1];
    RobotLocalizer localizer = new RobotLocalizer();
    CapturedDrivetrain drivetrain = new CapturedDrivetrain();
    Follower follower = new Follower(localizer, drivetrain, Constants.algorithm());
    double vx = 0, vy = 0;
    double seenX = x, seenY = y, seenVx = 0, seenVy = 0;
    localizer.set(x, y, h, 0, 0, 0);
    follower.follow(ShootAuto.pedroParkPath(x, y, h, route));
    // Along the wall toward the goal: how far past it the robot ever gets
    double ax = goal[0] - x;
    double ay = goal[1] - y;
    double len = Math.hypot(ax, ay);
    double worstPast = 0;
    double t = 0;
    boolean there = false;
    for (; t < ShootAuto.PARK_TIMEOUT_S && !there; t += DT) {
      localizer.set(seenX, seenY, h, seenVx, seenVy, 0);
      follower.update(DT);
      double[] stick = ShootAuto.pedroStick(drivetrain.powers(), h);
      double[] power = MecanumDriveSubsystem.shapeTranslation(stick[0], stick[1]);
      // Field power to robot frame, each axis to its own top speed
      double c = Math.cos(h), s = Math.sin(h);
      double fwd = power[0] * c + power[1] * s;
      double left = -power[0] * s + power[1] * c;
      double wantX = (fwd * TOP_FORWARD_MPS) * c - (left * TOP_STRAFE_MPS) * s;
      double wantY = (fwd * TOP_FORWARD_MPS) * s + (left * TOP_STRAFE_MPS) * c;
      seenX = x;
      seenY = y;
      seenVx = vx;
      seenVy = vy;
      for (int i = 0; i < 10; i++) {
        double accX = (wantX - vx) / LAG_S, accY = (wantY - vy) / LAG_S, acc = Math.hypot(accX, accY);
        if (acc > MAX_ACCEL) {
          accX *= MAX_ACCEL / acc;
          accY *= MAX_ACCEL / acc;
        }
        vx += accX * DT / 10;
        vy += accY * DT / 10;
        x += vx * DT / 10;
        y += vy * DT / 10;
      }
      // Past the goal, along the wall (the route's last leg runs into the wall, so that's x)
      worstPast = Math.max(worstPast, Math.signum(goal[0] - route[0][0]) * (x - goal[0]));
      there = Math.hypot(goal[0] - seenX, goal[1] - seenY) <= ShootAuto.PARK_DONE_M
          && Math.hypot(seenVx, seenVy) <= ShootAuto.PARK_DONE_MPS;
    }
    assertTrue("not there after " + t + " s, at " + x + ", " + y, there);
    assertTrue("ran " + worstPast + " m past the zone's middle", worstPast < 0.10);
    assertEquals(goal[0], x, 0.08);
    assertEquals(goal[1], y, 0.08);
  }
}
