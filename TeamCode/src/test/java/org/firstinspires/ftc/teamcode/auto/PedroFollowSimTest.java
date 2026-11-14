package org.firstinspires.ftc.teamcode.auto;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.pedro.CapturedDrivetrain;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.RobotLocalizer;
import org.firstinspires.ftc.teamcode.utils.VisionCorrector.Cell;
import org.junit.Test;

/**
 * Pedro, with the placeholder constants and our adapters, on a simple simulated robot: the powers' signs and frames
 * must take it to the end of each of the auto's drives, at the end's heading.
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

  private void place(double[] pose) {
    x = pose[0];
    y = pose[1];
    h = pose[2];
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

  /** One of the auto's turning drives, from where the robot is to end, as ScoringAuto.turningLine. */
  private double turningDrive(double[] end, double maxS) {
    Pose here = new Pose(x / AutoField.M_PER_IN, y / AutoField.M_PER_IN, h);
    Pose to = new Pose(end[0] / AutoField.M_PER_IN, end[1] / AutoField.M_PER_IN, end[2]);
    follower.follow(Paths.line(here, to).linear(here.heading(), to.heading()));
    return runToEnd(maxS);
  }

  @Test
  public void reachesEachShootingSpotBackToTheCell() {
    for (Alliance alliance : Alliance.values()) {
      for (Cell cell : Cell.values()) {
        place(AutoField.forAlliance(AutoField.START, alliance));
        double[] spot = AutoField.shotSpot(alliance, cell);
        double t = turningDrive(spot, 5.0);
        assertTrue(alliance + " " + cell + " took " + t + " s", t < 3.0);
        assertEquals(alliance + " " + cell + " x", spot[0], x, 0.05);
        assertEquals(alliance + " " + cell + " y", spot[1], y, 0.05);
        assertEquals(alliance + " " + cell + " heading", 0, AutoField.normalize(spot[2] - h), Math.toRadians(5));
      }
    }
  }

  @Test
  public void drivesFromTheHiveToTheGardenFacingAlongTheLine() {
    for (Alliance alliance : Alliance.values()) {
      for (Cell cell : Cell.values()) {
        place(AutoField.shotSpot(alliance, cell));
        double[] end = AutoField.forAlliance(AutoField.GARDEN_IN, alliance);
        double t = turningDrive(end, 5.0);
        assertTrue(alliance + " took " + t + " s", t < 4.0);
        assertEquals(alliance + " x", end[0], x, 0.05);
        assertEquals(alliance + " y", end[1], y, 0.05);
        assertEquals(alliance + " heading", 0, AutoField.normalize(end[2] - h), Math.toRadians(5));
      }
    }
  }

  @Test
  public void sweepsTheGardenAlongTheWallFacingTheCorner() {
    for (Alliance alliance : Alliance.values()) {
      place(AutoField.forAlliance(AutoField.GARDEN_IN, alliance));
      double wantHeading = h;
      double[] wall = AutoField.forAlliance(AutoField.GARDEN_WALL, alliance);
      double[] end = AutoField.forAlliance(AutoField.GARDEN_OUT, alliance);
      follower.hold(AutoField.pedro(AutoField.GARDEN_WALL, alliance)); // ONTO_GARDEN_WALL
      double onto = 0;
      while (Math.hypot(x - wall[0], y - wall[1]) > ScoringAuto.ONTO_WALL_M && onto < 1.5) {
        step();
        onto += DT;
        assertEquals(alliance + " turned onto the wall", wantHeading, h, Math.toRadians(8));
      }
      assertTrue(alliance + " took " + onto + " s onto the wall", onto < AutoSequence.ONTO_WALL_MAX_S);
      follower.follow(Paths.line(AutoField.pedro(AutoField.GARDEN_WALL, alliance),
          AutoField.pedro(AutoField.GARDEN_OUT, alliance)).constant(wantHeading));
      double t = 0;
      double worstOffWall = 0;
      while (follower.following() && t < 6.0) {
        step();
        t += DT;
        assertEquals(alliance + " turned at " + t, wantHeading, h, Math.toRadians(8));
        worstOffWall = Math.max(worstOffWall, Math.abs(x - wall[0]));
      }
      for (int i = 0; i < 50; i++) {
        step();
      }
      assertTrue(alliance + " off the wall by " + worstOffWall, worstOffWall < 0.03);
      assertEquals(alliance + " x", end[0], x, 0.04);
      assertEquals(alliance + " y", end[1], y, 0.04);
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
    for (Cell cell : Cell.values()) {
      place(AutoField.shotSpot(Alliance.RED, cell)); // After shooting the first Flower
      Pose here = new Pose(x / AutoField.M_PER_IN, y / AutoField.M_PER_IN, h);
      Pose end = AutoField.pedro(AutoField.REAR_FLOWER, Alliance.RED);
      follower.follow(Paths.curve(here, AutoField.pedro(AutoField.REAR_CONTROL, Alliance.RED), end)
          .linear(here.heading(), end.heading()));
      double t = 0;
      while (follower.following() && t < 6.0) {
        step();
        t += DT;
        assertFalse(cell + ": in the Hive at " + x + ", " + y, Math.abs(x) < 0.75 && Math.abs(y) < 0.95);
      }
      assertEquals(cell + " x", AutoField.REAR_FLOWER[0], x, 0.08);
      assertEquals(cell + " y", AutoField.REAR_FLOWER[1], y, 0.08);
    }
  }
}
