package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.psilynx.psikit.core.LogTable;
import org.psilynx.psikit.core.LoggableInputs;

import java.util.Collections;
import java.util.List;

/**
 * Everything one camera tells the robot, in a form PsiKit can log and replay.
 * Webcams are not covered by PsiKit's hardware map wrapper, so this is logged
 * by hand. The detection arrays are parallel: index i is the same tag in all
 * of them. Distances are in inches and angles in degrees, matching the
 * processor's output units. A value is NaN when the SDK gave no pose for it.
 */
public class VisionInputs implements LoggableInputs {
  /** Tag id reported for a detection that is a cluster rather than a single tag. */
  public static final int CLUSTER_ID = -1;

  public boolean available = false;
  public double fps = 0.0;

  public int[] tagIds = new int[0];
  public double[] range = new double[0];
  public double[] bearing = new double[0];
  public double[] elevation = new double[0];
  public double[] yaw = new double[0];
  public double[] x = new double[0];
  public double[] y = new double[0];
  public double[] z = new double[0];
  public double[] decisionMargin = new double[0];
  /** Per detection: frame acquisition time (detection.frameAcquisitionNanoTime). */
  public long[] frameNanos = new long[0];
  /** System.nanoTime() when this capture was taken. Zero before the first capture. */
  public long captureNanos = 0;

  /** Copies the processor's current detections into this object. Live only. */
  public void capture(AprilTagProcessor processor, VisionPortal portal) {
    available = processor != null;
    fps = portal == null ? 0.0 : portal.getFps();
    captureNanos = System.nanoTime();

    List<AprilTagDetection> detections = processor == null
        ? Collections.<AprilTagDetection>emptyList()
        : processor.getDetections();
    int n = detections.size();
    tagIds = new int[n];
    range = new double[n];
    bearing = new double[n];
    elevation = new double[n];
    yaw = new double[n];
    x = new double[n];
    y = new double[n];
    z = new double[n];
    decisionMargin = new double[n];
    frameNanos = new long[n];

    for (int i = 0; i < n; i++) {
      AprilTagDetection detection = detections.get(i);
      frameNanos[i] = detection.frameAcquisitionNanoTime;
      if (detection instanceof AprilTagSingleDetection) {
        AprilTagSingleDetection single = (AprilTagSingleDetection) detection;
        tagIds[i] = single.id;
        decisionMargin[i] = single.decisionMargin;
      } else {
        tagIds[i] = CLUSTER_ID;
        decisionMargin[i] = Double.NaN;
      }

      if (detection.ftcPose != null) {
        range[i] = detection.ftcPose.range;
        bearing[i] = detection.ftcPose.bearing;
        elevation[i] = detection.ftcPose.elevation;
        yaw[i] = detection.ftcPose.yaw;
        x[i] = detection.ftcPose.x;
        y[i] = detection.ftcPose.y;
        z[i] = detection.ftcPose.z;
      } else {
        range[i] = Double.NaN;
        bearing[i] = Double.NaN;
        elevation[i] = Double.NaN;
        yaw[i] = Double.NaN;
        x[i] = Double.NaN;
        y[i] = Double.NaN;
        z[i] = Double.NaN;
      }
    }
  }

  /** Short telemetry string, e.g. "id 20 @ 42.1 in, id 24 @ 60.0 in". */
  public String describe() {
    if (!available) {
      return "unavailable";
    }
    if (tagIds.length == 0) {
      return "none";
    }

    StringBuilder sb = new StringBuilder();
    for (int i = 0; i < tagIds.length; i++) {
      if (sb.length() > 0) {
        sb.append(", ");
      }
      if (tagIds[i] == CLUSTER_ID) {
        sb.append("cluster");
      } else {
        sb.append("id ").append(tagIds[i]);
      }
      if (!Double.isNaN(range[i])) {
        sb.append(String.format(" @ %.1f in", range[i]));
      }
    }
    return sb.toString();
  }

  @Override
  public void toLog(LogTable table) {
    table.put("Available", available);
    table.put("Fps", fps);
    table.put("TagIds", tagIds);
    table.put("RangeIn", range);
    table.put("BearingDeg", bearing);
    table.put("ElevationDeg", elevation);
    table.put("YawDeg", yaw);
    table.put("XIn", x);
    table.put("YIn", y);
    table.put("ZIn", z);
    table.put("DecisionMargin", decisionMargin);
    table.put("FrameNanos", frameNanos);
    table.put("CaptureNanos", captureNanos);
  }

  @Override
  public void fromLog(LogTable table) {
    available = table.get("Available", available);
    fps = table.get("Fps", fps);
    tagIds = table.get("TagIds", tagIds);
    range = table.get("RangeIn", range);
    bearing = table.get("BearingDeg", bearing);
    elevation = table.get("ElevationDeg", elevation);
    yaw = table.get("YawDeg", yaw);
    x = table.get("XIn", x);
    y = table.get("YIn", y);
    z = table.get("ZIn", z);
    decisionMargin = table.get("DecisionMargin", decisionMargin);
    frameNanos = table.get("FrameNanos", frameNanos);
    captureNanos = table.get("CaptureNanos", captureNanos);
  }
}
