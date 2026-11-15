package org.firstinspires.ftc.teamcode.vision;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Pose correction from the Hive tags (from Sean's feature/sotm) and the remembered Cell. Only each tag's ftcPose x/y/z
 * is used, not the SDK's robotPose: the tags move when a Hive tips. Editable live from Panels.
 */
@Configurable
public final class VisionConstants {
  private VisionConstants() {}

  // Odometry corrected by the Hive tags carries the Cell's position between sightings, and a backward shot
  // turns the cameras away from it, so it is kept well past one turn-around.
  public static double CELL_EXPIRY_S = 150.0; // Forget the Cell this long after the last sighting. A whole match: odometry carries it
  public static double CELL_SWITCH_AFTER_S = 1.0; // Switch to another of our Cells once the held one is unseen this long
  public static double CELL_SWITCH_MARGIN_M = 0.3; // ...or sooner if the other is at least this much nearer
  public static double CELL_SMOOTHING = 0.3; // Share of the way the stored Cell moves toward each sighting. 1 is no smoothing
  public static final int POSE_HISTORY_SIZE = 100; // About two seconds of loops

  public static double Z_TOL_M = 0.10; // A tag's measured height must be this close to the up or down cluster height
  public static int TIP_CONFIRM_FRAMES = 3; // Frames in a row voting for a flip before a Hive's up Cell changes

  public static double MIN_DECISION_MARGIN = 20; // Placeholder, tune
  public static double MAX_TAG_RANGE_M = 3.0;
  // Faster than this the frame's pose is too uncertain: odometry holds through the moves, while faster frames were
  // off by centimetres and degrees, which the aim then used
  public static double MAX_SPEED_FOR_VISION_MPS = 0.6;
  public static double MAX_OMEGA_FOR_VISION_RAD_S = 0.6;
  // The heading from the tags only once nearly still: a frame's heading error grows with the turn rate, and the
  // odometry's own heading stays true through turns on the spot. The first fix ignores this
  public static double MAX_SPEED_FOR_VISION_HEADING_MPS = 0.3;
  public static double MAX_OMEGA_FOR_VISION_HEADING_RAD_S = Math.toRadians(20);
  public static double MAX_FRAME_AGE_S = 0.5; // Pose history covers it. One detector pass ran 0.3 s behind on average
  public static double MAX_SPREAD_M = 0.10; // Most any tag's correction may differ from the median
  public static double MAX_JUMP_M = 0.5; // Largest correction once the pose is trusted. The first fix may be any size
  // A fix must put the robot's centre this far inside the walls. Not the robot's half width (AutoField.ROBOT_HALF_M)
  public static double FIX_WALL_MARGIN_M = 0.15;
  public static double CORRECTION_GAIN = 0.5; // Share of each correction applied. The first fix applies all of it
  // A jump bigger than MAX_JUMP_M that this many frames agree on (within POSE_OVERRIDE_AGREE_M) means odometry
  // lost the position, so it is replaced in full
  public static int POSE_OVERRIDE_FRAMES = 5;
  public static double POSE_OVERRIDE_AGREE_M = 0.15;
  // Heading from the tags: two tags seen at once give the robot's heading from the line between them.
  public static boolean VISION_HEADING = true;
  public static double MIN_HEADING_BASELINE_M = 0.2; // Tags closer together than this give too noisy a heading
  // The first heading fix is taken in full, so it waits for this many tag pairs: one pair put the start heading
  // far out, while more pairs a few frames later agreed on the truth
  public static int MIN_FIRST_HEADING_PAIRS = 3;
  public static double HEADING_GAIN = 0.2; // Share of each heading correction applied. The first applies all of it
  public static double MAX_HEADING_JUMP_DEG = 20; // Larger corrections once the heading is trusted are ignored
  // ...unless this many frames in a row agree on it (within HEADING_OVERRIDE_AGREE_DEG): then the odometry
  // heading is the wrong one (a bad Pinpoint read can leave it far out), so it is replaced in full
  public static int HEADING_OVERRIDE_FRAMES = 5;
  public static double HEADING_OVERRIDE_AGREE_DEG = 15;
}
