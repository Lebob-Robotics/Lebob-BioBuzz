package org.firstinspires.ftc.teamcode.auto;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.pedro.CapturedDrivetrain;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.RobotLocalizer;
import org.junit.Test;

/**
 * Pedro, with the placeholder constants and our adapters, on a simple simulated robot: the powers' signs and frames
 * must take it to the end of a path, and the aim's heading must be left alone.
 */
public class PedroFollowSimTest {
  private static final double DT = 0.02;
  private static final double MAX_FORWARD_MPS = 55 * AutoField.M_PER_IN;
  private static final double MAX_STRAFE_MPS = 45 * AutoField.M_PER_IN;
  private static final double MAX_TURN_RAD_S = 5.0;
  private static final double LAG_S = 0.12;

  private final RobotLocalizer localizer = new RobotLocalizer();
  private final CapturedDrivetrain drivetrain = new CapturedDrivetrain();
  private final Follower follower = new Follower(localizer, drivetrain, Constants.algorithm());
  private double x;
  private double y;
  private double h;
  private double vForward;
  private double vLeft;
  private double omega;
  private double heading; // Read by the aimed interpolator

  private void place(double[] pose) {
    x = pose[0];
    y = pose[1];
    h = pose[2];
    heading = h;
    sense();
  }

  private void sense() {
    double vx = vForward * Math.cos(h) - vLeft * Math.sin(h);
    double vy = vForward * Math.sin(h) + vLeft * Math.cos(h);
    localizer.set(x, y, h, vx, vy, omega);
  }

  /** One loop: Pedro's powers move the robot, each speed lagging its command. */
  private void step() {
    sense();
    heading = h;
    follower.update(DT);
    double[] p = drivetrain.powers();
    for (double v : p) {
      assertFalse("NaN power", Double.isNaN(v));
    }
    double a = DT / LAG_S;
    vForward += (p[0] * MAX_FORWARD_MPS - vForward) * a;
    vLeft += (p[1] * MAX_STRAFE_MPS - vLeft) * a;
    omega += (p[2] * MAX_TURN_RAD_S - omega) * a;
    x += (vForward * Math.cos(h) - vLeft * Math.sin(h)) * DT;
    y += (vForward * Math.sin(h) + vLeft * Math.cos(h)) * DT;
    h = AutoField.normalize(h + omega * DT);
  }

  private double runToEnd(double maxS) {
    double t = 0;
    while (follower.following() && t < maxS) {
      step();
      t += DT;
    }
    for (int i = 0; i < 50; i++) { // Settle on the hold
      step();
    }
    return t;
  }

  @Test
  public void drivesToTheGardenLeavingTheHeadingToTheAim() {
    for (Alliance alliance : Alliance.values()) {
      place(AutoField.forAlliance(AutoField.START, alliance));
      double startHeading = h;
      Interpolator aimed = (curve, t) -> heading;
      Pose here = new Pose(x / AutoField.M_PER_IN, y / AutoField.M_PER_IN, h);
      follower.follow(Paths.line(here, AutoField.pedro(AutoField.GARDEN_IN, alliance)).heading(aimed));
      double t = runToEnd(5.0);
      double[] end = AutoField.forAlliance(AutoField.GARDEN_IN, alliance);
      assertTrue(alliance + " took " + t + " s", t < 4.0);
      assertEquals(alliance + " x", end[0], x, 0.05);
      assertEquals(alliance + " y", end[1], y, 0.05);
      assertEquals(alliance + " heading kept", startHeading, h, Math.toRadians(10));
    }
  }

  @Test
  public void turnsToFaceTheFlowerOnTheWayIn() {
    place(AutoField.WALL_FLOWER);
    h = AutoField.normalize(AutoField.WALL_FLOWER[2] + Math.toRadians(25)); // As the last shot left it
    Pose here = new Pose(x / AutoField.M_PER_IN, y / AutoField.M_PER_IN, h);
    Pose end = AutoField.pedro(AutoField.WALL_FLOWER_INTAKE, Alliance.RED);
    follower.follow(Paths.line(here, end).linear(here.heading(), end.heading()));
    runToEnd(3.0);
    assertEquals(AutoField.WALL_FLOWER_INTAKE[0], x, 0.04);
    assertEquals(AutoField.WALL_FLOWER_INTAKE[1], y, 0.04);
    assertEquals(AutoField.WALL_FLOWER_INTAKE[2], h, Math.toRadians(5));
  }

  @Test
  public void curvesRoundTheHiveToTheRearFlower() {
    place(AutoField.WALL_FLOWER_INTAKE);
    Interpolator aimed = (curve, t) -> heading;
    Pose here = new Pose(x / AutoField.M_PER_IN, y / AutoField.M_PER_IN, h);
    follower.follow(Paths.curve(here, AutoField.pedro(AutoField.REAR_CONTROL, Alliance.RED),
        AutoField.pedro(AutoField.REAR_FLOWER, Alliance.RED)).heading(aimed));
    double t = 0;
    while (follower.following() && t < 6.0) {
      step();
      t += DT;
      assertFalse("in the Hive at " + x + ", " + y, Math.abs(x) < 0.75 && Math.abs(y) < 0.95);
    }
    assertEquals(AutoField.REAR_FLOWER[0], x, 0.08);
    assertEquals(AutoField.REAR_FLOWER[1], y, 0.08);
  }
}
