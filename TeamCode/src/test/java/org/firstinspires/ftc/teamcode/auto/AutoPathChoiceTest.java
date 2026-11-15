package org.firstinspires.ftc.teamcode.auto;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import com.pedropathing.api.Paths;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import java.io.File;
import java.lang.reflect.Field;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.field.FieldPose;
import org.firstinspires.ftc.teamcode.pedro.CapturedDrivetrain;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.junit.Test;

/**
 * What each auto does at each step, kept as it was at the regionals: the drive a step starts from a set of poses
 * (Pedro's mode, the path or the held pose, the power scale and push, the heading lock), the Cell it aims at, and
 * when a step counts as arrived, over a grid of times, speeds and poses either side of each threshold.
 *
 * <p>The expected lines are in {@code src/test/resources/auto/path-choice.txt}. A change to an auto's drives is
 * meant to fail this: set WRITE_PATH_CHOICE=1 to rewrite the file from the current code, and check its diff.
 */
public class AutoPathChoiceTest {
  private static final double[] OFFSETS_M = {0, 0.015, 0.05, 0.09, 0.15, 0, 0};
  private static final double[] OFFSETS_DEG = {0, 0, 0, 0, 0, 5, 10};
  private static final double[] IN_STEP_S = {0.2, 0.4, 0.6};
  private static final double[] SPEEDS_MPS = {0.02, 0.04, 0.2};
  private static final double UNSET = Double.NEGATIVE_INFINITY;

  private static final class ScoringProbe extends ScoringAuto {
    FieldPose at;
    double locked = UNSET;

    ScoringProbe(Alliance alliance, Cell first) {
      super(alliance);
      follower = new Follower(localizer, drivetrain, Constants.algorithm());
      firstCell = first;
    }

    @Override
    protected FieldPose pose() {
      return at;
    }

    @Override
    protected void lockHeading(double headingRad) {
      locked = headingRad;
    }
  }

  private static final class CornerProbe extends CornerAuto {
    FieldPose at;
    double locked = UNSET;

    CornerProbe(Alliance alliance) {
      super(alliance);
      follower = new Follower(localizer, drivetrain, Constants.algorithm());
    }

    @Override
    protected FieldPose pose() {
      return at;
    }

    @Override
    protected void lockHeading(double headingRad) {
      locked = headingRad;
    }
  }

  @Test
  public void eachAutoChoosesTheSameDrivesAimsAndArrivals() throws Exception {
    List<String> now = new ArrayList<>();
    scoringStarts(now);
    scoringAimThroughThePlan(now);
    scoringArrivals(now);
    cornerStarts(now);
    cornerAims(now);
    cornerArrivals(now);

    File file = new File(resourceDir(), "path-choice.txt");
    if ("1".equals(System.getenv("WRITE_PATH_CHOICE"))) {
      Files.write(file.toPath(), now, StandardCharsets.UTF_8);
      return;
    }
    List<String> expected = Files.readAllLines(file.toPath(), StandardCharsets.UTF_8);
    List<String> problems = new ArrayList<>();
    int changed = 0;
    for (int i = 0; i < Math.max(expected.size(), now.size()); i++) {
      String want = i < expected.size() ? expected.get(i) : "(none)";
      String got = i < now.size() ? now.get(i) : "(none)";
      if (!want.equals(got)) {
        changed++;
        if (problems.size() < 20) {
          problems.add("line " + (i + 1) + "\n  was " + want + "\n  now " + got);
        }
      }
    }
    if (changed > 0) {
      fail(changed + " auto choices changed, the first " + problems.size() + ":\n" + String.join("\n", problems));
    }
    assertTrue("nothing recorded", now.size() > 100);
  }

  private static void scoringStarts(List<String> out) {
    for (Alliance alliance : Alliance.values()) {
      for (Cell first : Cell.values()) {
        for (AutoSequence.Step step : AutoSequence.Step.values()) {
          for (Map.Entry<String, double[]> from : scoringPoses(alliance).entrySet()) {
            ScoringProbe auto = new ScoringProbe(alliance, first);
            auto.at = fieldPose(from.getValue());
            auto.startStep(step);
            out.add("scoring " + alliance + " first " + first + " " + step + " from " + from.getKey() + ": "
                + drive(auto.follower, auto.drivetrain) + " lock " + num(auto.locked) + " aim " + auto.aimCell());
          }
        }
      }
    }
  }

  /** The aim follows the last shooting spot started, from the start's up Cell before the first. */
  private static void scoringAimThroughThePlan(List<String> out) {
    for (Alliance alliance : Alliance.values()) {
      for (Cell first : Cell.values()) {
        ScoringProbe auto = new ScoringProbe(alliance, first);
        auto.at = fieldPose(AutoField.forAlliance(AutoField.START, alliance));
        StringBuilder aims = new StringBuilder("before " + auto.aimCell());
        for (AutoSequence.Step step : AutoSequence.Step.values()) {
          auto.startStep(step);
          aims.append(", ").append(step).append(' ').append(auto.aimCell());
        }
        out.add("scoring " + alliance + " first " + first + " aims: " + aims);
      }
    }
  }

  private static void scoringArrivals(List<String> out) {
    for (Alliance alliance : Alliance.values()) {
      ScoringProbe auto = new ScoringProbe(alliance, Cell.SCORING);
      Map<String, double[]> spots = new LinkedHashMap<>();
      spots.put("gardenIn", AutoField.forAlliance(AutoField.GARDEN_IN, alliance));
      spots.put("gardenWall", AutoField.forAlliance(AutoField.GARDEN_WALL, alliance));
      spots.put("wallFlower", AutoField.forAlliance(AutoField.WALL_FLOWER, alliance));
      spots.put("rearFlower", AutoField.forAlliance(AutoField.REAR_FLOWER, alliance));
      for (AutoSequence.Step step : AutoSequence.Step.values()) {
        for (boolean following : new boolean[] {false, true}) {
          setFollowing(auto.follower, following);
          auto.at = fieldPose(AutoField.forAlliance(AutoField.START, alliance));
          out.add("scoring " + alliance + " arrived " + step + " not facing, following " + following + ": "
              + arrivals((inStepS, speed) -> auto.stepArrived(step, false, inStepS, speed)));
          for (Map.Entry<String, double[]> spot : spots.entrySet()) {
            StringBuilder bits = new StringBuilder();
            for (int i = 0; i < OFFSETS_M.length; i++) {
              auto.at = offset(spot.getValue(), i);
              bits.append(arrivals((inStepS, speed) -> auto.stepArrived(step, true, inStepS, speed))).append(' ');
            }
            out.add("scoring " + alliance + " arrived " + step + " facing, following " + following + ", near "
                + spot.getKey() + ": " + bits.toString().trim());
          }
        }
      }
    }
  }

  private static void cornerStarts(List<String> out) {
    for (Alliance alliance : Alliance.values()) {
      for (CornerSequence.Step step : CornerSequence.Step.values()) {
        for (Map.Entry<String, double[]> from : cornerPoses(alliance).entrySet()) {
          CornerProbe auto = new CornerProbe(alliance);
          auto.at = fieldPose(from.getValue());
          auto.startStep(step);
          out.add("corner " + alliance + " " + step + " from " + from.getKey() + ": "
              + drive(auto.follower, auto.drivetrain) + " lock " + num(auto.locked));
        }
      }
    }
  }

  private static void cornerAims(List<String> out) {
    for (Alliance alliance : Alliance.values()) {
      CornerProbe auto = new CornerProbe(alliance);
      StringBuilder aims = new StringBuilder("before " + auto.aimCell(null));
      for (CornerSequence.Step step : CornerSequence.Step.values()) {
        aims.append(", ").append(step).append(' ').append(auto.aimCell(step));
      }
      out.add("corner " + alliance + " aims: " + aims);
    }
  }

  private static void cornerArrivals(List<String> out) {
    for (Alliance alliance : Alliance.values()) {
      CornerProbe auto = new CornerProbe(alliance);
      Map<String, double[]> spots = new LinkedHashMap<>();
      spots.put("preload", CornerSequence.preloadSpot(alliance));
      spots.put("wallFlower", AutoField.forAlliance(AutoField.WALL_FLOWER, alliance));
      for (CornerSequence.Step step : CornerSequence.Step.values()) {
        for (boolean following : new boolean[] {false, true}) {
          setFollowing(auto.follower, following);
          auto.at = fieldPose(AutoField.forAlliance(CornerSequence.CORNER, alliance));
          out.add("corner " + alliance + " arrived " + step + " not facing, following " + following + ": "
              + arrivals((inStepS, speed) -> auto.stepArrived(step, false, inStepS, speed)));
          for (Map.Entry<String, double[]> spot : spots.entrySet()) {
            StringBuilder bits = new StringBuilder();
            for (int i = 0; i < OFFSETS_M.length; i++) {
              auto.at = offset(spot.getValue(), i);
              bits.append(arrivals((inStepS, speed) -> auto.stepArrived(step, true, inStepS, speed))).append(' ');
            }
            out.add("corner " + alliance + " arrived " + step + " facing, following " + following + ", near "
                + spot.getKey() + ": " + bits.toString().trim());
          }
        }
      }
    }
  }

  private static Map<String, double[]> scoringPoses(Alliance alliance) {
    Map<String, double[]> poses = new LinkedHashMap<>();
    poses.put("start", AutoField.forAlliance(AutoField.START, alliance));
    poses.put("middle", AutoField.forAlliance(new double[] {0.3, -0.4, 1.0}, alliance));
    poses.put("scoringSpot", AutoField.shotSpot(alliance, Cell.SCORING));
    poses.put("audienceSpot", AutoField.shotSpot(alliance, Cell.AUDIENCE));
    poses.put("gardenIn", AutoField.forAlliance(AutoField.GARDEN_IN, alliance));
    poses.put("rearFlower", AutoField.forAlliance(AutoField.REAR_FLOWER, alliance));
    return poses;
  }

  private static Map<String, double[]> cornerPoses(Alliance alliance) {
    Map<String, double[]> poses = new LinkedHashMap<>();
    poses.put("corner", AutoField.forAlliance(CornerSequence.CORNER, alliance));
    poses.put("middle", AutoField.forAlliance(new double[] {0.3, -0.4, 1.0}, alliance));
    poses.put("preload", CornerSequence.preloadSpot(alliance));
    poses.put("edgeEnd", AutoField.forAlliance(CornerSequence.EDGE_END, alliance));
    poses.put("wallFlower", AutoField.forAlliance(AutoField.WALL_FLOWER, alliance));
    poses.put("park", CornerSequence.parkSpot(alliance));
    return poses;
  }

  private interface Arrival {
    boolean arrived(double inStepS, double speedMps);
  }

  /** One 0/1 per (time in step, speed), times outer. */
  private static String arrivals(Arrival arrival) {
    StringBuilder bits = new StringBuilder();
    for (double inStepS : IN_STEP_S) {
      for (double speed : SPEEDS_MPS) {
        bits.append(arrival.arrived(inStepS, speed) ? '1' : '0');
      }
    }
    return bits.toString();
  }

  private static FieldPose offset(double[] spot, int i) {
    return new FieldPose(spot[0] + OFFSETS_M[i], spot[1], spot[2] + Math.toRadians(OFFSETS_DEG[i]));
  }

  private static void setFollowing(Follower follower, boolean following) {
    if (following) {
      follower.follow(Paths.line(new Pose(0, 0, 0), new Pose(20, 0, 0)).constant(0));
    } else {
      follower.hold(new Pose(0, 0, 0));
    }
    assertEquals(following, follower.following());
  }

  /** Pedro's mode, its path's start, middle and end or its held pose, and the power scale and push it drives with. */
  private static String drive(Follower follower, CapturedDrivetrain drivetrain) {
    StringBuilder s = new StringBuilder(follower.mode().name());
    if (follower.mode() == Follower.Mode.FOLLOW) {
      Path path = follower.currentPath();
      s.append(" segments ").append(path.getSegments().size());
      for (double t : new double[] {0, 0.5, 1}) {
        Pose p = path.get(t);
        s.append(" (").append(num(p.x())).append(',').append(num(p.y())).append(',').append(num(path.heading(t)))
            .append(')');
      }
    } else if (follower.mode() == Follower.Mode.HOLD) {
      Pose p = holdPose(follower);
      s.append(" (").append(num(p.x())).append(',').append(num(p.y())).append(',').append(num(p.heading()))
          .append(')');
    }
    drivetrain.drive(new DrivePowers(0.1, 0.1, 0.1), false); // Shows the scale and the forward and strafe push
    double[] p = drivetrain.powers();
    s.append(" powers ").append(num(p[0])).append(',').append(num(p[1])).append(',').append(num(p[2]));
    return s.toString();
  }

  private static Pose holdPose(Follower follower) {
    try {
      Field held = Follower.class.getDeclaredField("holdPose");
      held.setAccessible(true);
      return (Pose) held.get(follower);
    } catch (ReflectiveOperationException e) {
      throw new AssertionError("Pedro's Follower has no holdPose field", e);
    }
  }

  private static FieldPose fieldPose(double[] pose) {
    return new FieldPose(pose[0], pose[1], pose[2]);
  }

  private static String num(double v) {
    return v == UNSET ? "unset" : String.format(Locale.US, "%.4f", v);
  }

  private static File resourceDir() {
    for (String base : new String[] {"src/test/resources/auto", "TeamCode/src/test/resources/auto"}) {
      File dir = new File(base);
      if (dir.isDirectory()) {
        return dir;
      }
    }
    throw new IllegalStateException("no auto resource dir from " + new File(".").getAbsolutePath());
  }
}
