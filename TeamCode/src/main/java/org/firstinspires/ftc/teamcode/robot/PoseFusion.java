package org.firstinspires.ftc.teamcode.robot;

import java.util.List;
import org.firstinspires.ftc.teamcode.field.FieldPose;
import org.firstinspires.ftc.teamcode.field.PoseHistory;
import org.firstinspires.ftc.teamcode.vision.TagObservation;
import org.firstinspires.ftc.teamcode.vision.VisionConstants;
import org.firstinspires.ftc.teamcode.vision.VisionResult;

/** Corrects the odometry pose from the Hive tags, and keeps whether the tags have placed the robot yet. */
final class PoseFusion {
  private final Robot robot;
  // Vision pose correction. The history holds each loop's odometry pose with the time of that loop's
  // camera capture, so a frame can be matched to where the robot was. It is separate from the tracker's.
  final PoseHistory poseHistory = new PoseHistory(VisionConstants.POSE_HISTORY_SIZE);
  private long lastHistoryNanos = Long.MIN_VALUE;
  boolean poseValid = false; // True once a tag fix has placed the robot on the field
  // True once two tags have set the heading, or the driver reset it with A. Until then a heading fix may be any size
  boolean headingValid = false;
  boolean poseSeeded = false; // Shoot Auto's assumed start pose: aimed from until vision places the robot
  VisionResult lastVision = null; // Last update that looked at tags, for telemetry
  long lastVisionNanos = 0;
  int positionFixes; // Tag fixes applied to the position so far

  PoseFusion(Robot robot) {
    this.robot = robot;
  }

  /** Forgets the pose history and the tags' placement, for the start of a run. */
  void reset() {
    poseHistory.clear();
    lastHistoryNanos = Long.MIN_VALUE;
    poseValid = false;
    headingValid = false;
    poseSeeded = false;
    lastVision = null;
  }
  /**
   * Corrects the pose from the Hive tags, and moves the tracker's stored Cell and poses by the same
   * amount so the aim does not jump. Returns the pose to use this loop.
   */
  FieldPose correctPoseFromVision(FieldPose pose, double[] velocity, long nowNanos) {
    List<TagObservation> observations = robot.vision.takeNewTagObservations();
    robot.noteHiveTags(observations);
    robot.visionCorrector.setHeadingSeeded(poseSeeded);
    VisionResult result = robot.visionCorrector.process(observations, poseHistory, pose,
        velocity[0], velocity[1], velocity[2], poseValid, headingValid, nowNanos);
    if (result.accepted) {
      robot.odometry.setPose(result.newPose.xM, result.newPose.yM, result.newPose.headingRad);
      // Turn about the robot by the heading fix, then shift, so stored poses and the Cell stay put relative to it
      double dHeading = result.appliedDHeadingRad();
      robot.driveControl.headingHold.shift(dHeading);
      poseHistory.transform(pose.xM, pose.yM, dHeading, result.appliedDxM(), result.appliedDyM());
      robot.tracker.transform(pose.xM, pose.yM, dHeading, result.appliedDxM(), result.appliedDyM());
      poseValid |= result.gain > 0; // The first position fix is applied in full, so after it the pose is on the field
      if (result.gain > 0) { positionFixes++; }
      headingValid |= result.headingGain > 0; // Likewise the first heading fix
      pose = result.newPose; // The Pinpoint may not report the new pose until its next update
    }
    if (result.reason != VisionResult.Reason.NO_OBSERVATIONS) {
      lastVision = result;
      lastVisionNanos = nowNanos;
    }
    double frameAgeS = Double.NaN; // Of the newest frame handed in
    for (TagObservation observation : observations) {
      double ageS = (nowNanos - observation.frameTimeNanos) * 1e-9;
      frameAgeS = Double.isNaN(frameAgeS) ? ageS : Math.min(frameAgeS, ageS);
    }
    robot.log.logVision(result, frameAgeS, observations.size());
    return pose;
  }

  /** Records this loop's pose against this loop's capture time, so later frames can be matched to it. */
  void stampPoseHistory(FieldPose pose) {
    long stampNanos = robot.vision.getCaptureNanos();
    if (stampNanos > lastHistoryNanos && stampNanos > 0) { // Not before the first capture, or twice for one
      poseHistory.add(stampNanos, pose);
      lastHistoryNanos = stampNanos;
    }
  }
}
