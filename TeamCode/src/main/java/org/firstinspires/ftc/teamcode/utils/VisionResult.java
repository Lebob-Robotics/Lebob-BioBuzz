package org.firstinspires.ftc.teamcode.utils;

/** What {@link VisionCorrector#process} made of one loop's tags. */
public final class VisionResult {
  /** Why the pose was or was not corrected. */
  public enum Reason {
    ACCEPTED,
    HEADING_ONLY, // The position was rejected (off the field or too big a jump), but the tag pairs fixed the heading
    NO_OBSERVATIONS,
    TOO_FAST,
    TOO_FAST_TURNING,
    NO_VALID_TAGS, // Every tag was dropped, see firstTagReject
    SPREAD,
    OUT_OF_FIELD,
    JUMP,
    NO_HEADING, // A first position fix, but the heading isn't trusted yet and no tag pair fixed it
    RELOCATED // Too big a jump, but the tags kept agreeing on it, so odometry had lost the position: taken in full
  }

  /** Why one tag was dropped. */
  public enum TagReject {
    NONE,
    NO_POSE,
    NOT_HIVE_TAG,
    LOW_MARGIN,
    TOO_FAR,
    BAD_Z,
    FRAME_TOO_OLD,
    NO_HISTORY
  }

  public final boolean accepted;
  public final Reason reason;
  public final TagReject firstTagReject; // Why the first dropped tag was dropped. NONE if none were
  public final double dxM; // Median correction: where the tags say the robot was, less where odometry said. Zero unless accepted
  public final double dyM;
  public final double gain; // Share of the correction to apply. Zero unless accepted
  public final Pose newPose; // Current pose plus gain times the correction. Null unless accepted
  public final int tagsUsed;
  public final int tagsRejected;
  public final double spreadM; // Furthest any used tag's correction is from the median. NaN if none were used
  public final boolean redFlipped; // This update changed that Hive's up Cell
  public final boolean blueFlipped;
  public final int[] usedIds; // Each used tag's id and correction
  public final double[] usedDxM;
  public final double[] usedDyM;
  public double dHeadingRad; // Median heading correction from tag pairs, before the gain. Zero if none
  public double headingGain; // Share of it to apply. Zero when no heading correction is applied
  public int headingPairs; // Tag pairs that gave a heading

  VisionResult(Reason reason, TagReject firstTagReject, double dxM, double dyM, double gain,
      Pose newPose, int tagsUsed, int tagsRejected, double spreadM, boolean redFlipped,
      boolean blueFlipped, int[] usedIds, double[] usedDxM, double[] usedDyM) {
    this.accepted = reason == Reason.ACCEPTED || reason == Reason.HEADING_ONLY || reason == Reason.RELOCATED;
    this.reason = reason;
    this.firstTagReject = firstTagReject;
    this.dxM = dxM;
    this.dyM = dyM;
    this.gain = gain;
    this.newPose = newPose;
    this.tagsUsed = tagsUsed;
    this.tagsRejected = tagsRejected;
    this.spreadM = spreadM;
    this.redFlipped = redFlipped;
    this.blueFlipped = blueFlipped;
    this.usedIds = usedIds;
    this.usedDxM = usedDxM;
    this.usedDyM = usedDyM;
  }

  /** The turn to apply to the heading, and to anything stored in the same frame (about the robot). */
  public double appliedDHeadingRad() {
    return headingGain * dHeadingRad;
  }

  /** The shift to apply to the pose, and to anything stored in the same frame. */
  public double appliedDxM() {
    return gain * dxM;
  }

  public double appliedDyM() {
    return gain * dyM;
  }
}
