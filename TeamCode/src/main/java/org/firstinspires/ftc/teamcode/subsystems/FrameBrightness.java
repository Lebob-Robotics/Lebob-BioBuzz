package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Canvas;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

/**
 * Counts a camera's frames and measures how bright they are, for setting exposure from a match log: the mean grey
 * level and the share of pixels clipped white or crushed black. Sampled with nearest-neighbour on a 160x90 grid, so
 * clipped pixels stay clipped instead of being averaged away, and at most STATS_HZ times a second; all native calls,
 * well under a millisecond. Any failure turns the measurement off rather than stopping the camera.
 */
@Configurable
public class FrameBrightness implements VisionProcessor {
  public static double STATS_HZ = 5;
  /** Grey levels (0-255) at or above this count as clipped white, at or below DARK_LEVEL as crushed black. */
  public static int CLIPPED_LEVEL = 250;
  public static int DARK_LEVEL = 20;

  private static final Size SAMPLE = new Size(160, 90);
  private final Mat small = new Mat();
  private final Mat grey = new Mat();
  private final Mat mask = new Mat();
  private long lastStatsNanos = Long.MIN_VALUE;
  private volatile boolean failed;

  private volatile long frames;
  private volatile double mean = Double.NaN;
  private volatile double clippedPct = Double.NaN;
  private volatile double darkPct = Double.NaN;

  @Override public void init(int width, int height, CameraCalibration calibration) {}

  @Override public Object processFrame(Mat frame, long captureTimeNanos) {
    frames++;
    double hz = STATS_HZ;
    if (hz <= 0 || failed) {
      return null;
    }
    if (lastStatsNanos != Long.MIN_VALUE && captureTimeNanos - lastStatsNanos < 1e9 / hz) {
      return null;
    }
    lastStatsNanos = captureTimeNanos;
    try {
      Imgproc.resize(frame, small, SAMPLE, 0, 0, Imgproc.INTER_NEAREST);
      Imgproc.cvtColor(small, grey, small.channels() == 4 ? Imgproc.COLOR_RGBA2GRAY : Imgproc.COLOR_RGB2GRAY);
      double pixels = grey.total();
      mean = Core.mean(grey).val[0];
      Core.compare(grey, new Scalar(CLIPPED_LEVEL), mask, Core.CMP_GE);
      clippedPct = 100.0 * Core.countNonZero(mask) / pixels;
      Core.compare(grey, new Scalar(DARK_LEVEL), mask, Core.CMP_LE);
      darkPct = 100.0 * Core.countNonZero(mask) / pixels;
    } catch (Throwable t) {
      failed = true;
      RobotLog.ee("BioBuzzVision", t, "Frame brightness measurement stopped");
    }
    return null;
  }

  /** Frames this camera has delivered since it opened. */
  public long frames() {
    return frames;
  }

  /** Mean grey level 0-255 of the latest measured frame; NaN before the first. */
  public double mean() {
    return mean;
  }

  public double clippedPct() {
    return clippedPct;
  }

  public double darkPct() {
    return darkPct;
  }

  @Override public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
      float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {}
}
