package org.firstinspires.ftc.teamcode.field;

/** The field frame and size. Not on Panels: these must match the real field. Tests change the size, so it isn't final. */
public final class FieldGeometry {
  private FieldGeometry() {}

  // The odometry heading is in the field frame (0 along +x, anticlockwise positive), so the camera can
  // use the tags' field positions. The drive stays driver relative: heading minus the forward below.
  // Heading the driver calls "forward", standing at their wall. The tags set the field heading, so these must
  // match the real field. Each driver stands at the wall nearest their own Cells (red tags at -y, blue at +y), with
  // both Cells off to the sides, 90 degrees from them: red faces +y, blue -y (6 Nov, from the drivers)
  public static final double ALLIANCE_FORWARD_RED_RAD = Math.PI / 2;
  public static final double ALLIANCE_FORWARD_BLUE_RAD = -Math.PI / 2;

  public static double FIELD_HALF_SIZE_M = 1.83;
}
