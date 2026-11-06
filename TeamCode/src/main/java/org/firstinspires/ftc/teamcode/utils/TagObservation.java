package org.firstinspires.ftc.teamcode.utils;

/** One AprilTag seen by one camera, in the SDK's ftcPose axes: x right, y along the lens, z up. */
public final class TagObservation {
  public final int id;
  public final CameraMount camera;
  public final double rightM;
  public final double forwardM; // Along the lens
  public final double upM;
  public final double decisionMargin;
  public final long frameTimeNanos; // System.nanoTime() when the frame was taken

  public TagObservation(int id, CameraMount camera, double rightM, double forwardM, double upM,
      double decisionMargin, long frameTimeNanos) {
    this.id = id;
    this.camera = camera;
    this.rightM = rightM;
    this.forwardM = forwardM;
    this.upM = upM;
    this.decisionMargin = decisionMargin;
    this.frameTimeNanos = frameTimeNanos;
  }

  /** Distance from the camera to the tag. */
  public double rangeM() {
    return Math.sqrt(rightM * rightM + forwardM * forwardM + upM * upM);
  }
}
