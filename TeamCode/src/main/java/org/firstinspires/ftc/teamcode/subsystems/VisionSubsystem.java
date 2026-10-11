package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.ArrayList;
import java.util.List;

/**
 * Wraps two webcams, each running an AprilTag processor, and reports their
 * detections. The two live views share the Robot Controller preview.
 *
 * <p>A camera that is missing or fails to start is left unavailable instead of
 * crashing init.
 */
public class VisionSubsystem extends SubsystemBase {
  public enum Side {
    LEFT, RIGHT
  }

  private final AprilTagProcessor[] processors = new AprilTagProcessor[2];
  private final VisionPortal[] portals = new VisionPortal[2];

  public VisionSubsystem(HardwareMap hardwareMap) {
    int[] viewIds = null;
    try {
      viewIds = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);
    } catch (RuntimeException e) {
      // Fall back to portals without a shared live view.
    }

    setUpCamera(hardwareMap, Side.LEFT, "CameraL", viewIds == null ? -1 : viewIds[0]);
    setUpCamera(hardwareMap, Side.RIGHT, "CameraR", viewIds == null ? -1 : viewIds[1]);
  }

  private void setUpCamera(HardwareMap hardwareMap, Side side, String name, int viewId) {
    try {
      AprilTagProcessor processor = new AprilTagProcessor.Builder()
          .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
          .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
          .build();

      VisionPortal.Builder builder = new VisionPortal.Builder()
          .setCamera(hardwareMap.get(WebcamName.class, name))
          .addProcessor(processor);
      if (viewId >= 0) {
        builder.setLiveViewContainerId(viewId);
      }

      portals[side.ordinal()] = builder.build();
      processors[side.ordinal()] = processor;
    } catch (RuntimeException e) {
      portals[side.ordinal()] = null;
      processors[side.ordinal()] = null;
    }
  }

  public boolean isAvailable(Side side) {
    return processors[side.ordinal()] != null;
  }

  /** Current AprilTag detections for one camera, empty if it is unavailable. */
  public List<AprilTagDetection> getDetections(Side side) {
    AprilTagProcessor processor = processors[side.ordinal()];
    if (processor == null) {
      return new ArrayList<>();
    }
    return processor.getDetections();
  }

  /** Short telemetry string, e.g. "id 20 @ 42.1 in, id 24 @ 60.0 in". */
  public String describe(Side side) {
    if (!isAvailable(side)) {
      return "unavailable";
    }

    List<AprilTagDetection> detections = getDetections(side);
    if (detections.isEmpty()) {
      return "none";
    }

    StringBuilder sb = new StringBuilder();
    for (AprilTagDetection detection : detections) {
      if (sb.length() > 0) {
        sb.append(", ");
      }
      if (detection instanceof AprilTagSingleDetection) {
        sb.append("id ").append(((AprilTagSingleDetection) detection).id);
      } else {
        sb.append("cluster");
      }
      if (detection.ftcPose != null) {
        sb.append(String.format(" @ %.1f in", detection.ftcPose.range));
      }
    }
    return sb.toString();
  }

  /** Closes both portals. Call from the OpMode's stop(). */
  public void close() {
    for (int i = 0; i < portals.length; i++) {
      if (portals[i] != null) {
        portals[i].close();
        portals[i] = null;
        processors[i] = null;
      }
    }
  }
}
