package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Arrays;
import java.util.Comparator;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Saves a camera's frames as small JPEGs, so a PC can line up what the robot saw with its log and the field
 * webcam. Each file is named by the frame's capture time in milliseconds of System.nanoTime(), the clock the log
 * runs on, under /sdcard/FIRST/robotcam/RUN/. The camera thread only shrinks the frame; a writer thread encodes and
 * saves it, and a frame that arrives while the writer is busy is skipped, so neither the robot loop nor the tag
 * detection waits on the disk. Any failure turns recording off rather than stopping the camera.
 */
@Configurable
public class FrameRecorder implements VisionProcessor {
  /** Frames saved per second at most; 0 records nothing. Read every frame. */
  public static double FPS = 5; // 10 fps at 640 wide slowed the robot loop from 67 to 88 ms (6 Nov)
  public static int WIDTH = 480;
  public static int JPEG_QUALITY = 70;
  /** Runs kept on the hub; older ones are deleted when a new one starts. About 25 MB a minute at 10 FPS. */
  public static int KEEP_RUNS = 12;
  /** Stops a run after this many frames (20 minutes at 10 FPS), so a forgotten OpMode can't fill the hub. */
  public static int MAX_FRAMES_PER_RUN = 12000;

  public static final File ROOT = new File("/sdcard/FIRST/robotcam");

  private final File dir;
  private final ExecutorService writer = Executors.newSingleThreadExecutor(r -> {
    Thread t = new Thread(r, "BioBuzzFrameRecorder");
    t.setDaemon(true);
    t.setPriority(Thread.MIN_PRIORITY);
    return t;
  });
  private final AtomicBoolean busy = new AtomicBoolean();
  private volatile long lastFrameNanos = Long.MIN_VALUE;
  private volatile boolean failed;
  private volatile boolean useBitmap;
  private volatile int saved;
  private volatile int skipped;
  private volatile String status = "starting";

  /** @param camera short camera name, part of the run folder's name */
  public FrameRecorder(String camera) {
    dir = new File(ROOT, String.format("%d_%s", System.nanoTime() / 1_000_000_000L, camera));
    writer.execute(() -> {
      if (!dir.mkdirs() && !dir.isDirectory()) {
        fail("can't make " + dir, null);
        return;
      }
      pruneOldRuns();
      status = "recording to " + dir.getName();
    });
  }

  @Override public void init(int width, int height, CameraCalibration calibration) {}

  @Override public Object processFrame(Mat frame, long captureTimeNanos) {
    double fps = FPS;
    if (fps <= 0 || failed || saved >= MAX_FRAMES_PER_RUN) {
      return null;
    }
    // The first frame is always taken: now minus Long.MIN_VALUE overflows to a negative gap
    if (lastFrameNanos != Long.MIN_VALUE && captureTimeNanos - lastFrameNanos < 1e9 / fps) {
      return null;
    }
    if (!busy.compareAndSet(false, true)) {
      skipped++;
      return null;
    }
    lastFrameNanos = captureTimeNanos;
    Mat small = new Mat();
    try {
      int width = Math.min(WIDTH, frame.cols());
      Imgproc.resize(frame, small, new Size(width, Math.round(frame.rows() * (double) width / frame.cols())));
    } catch (Throwable t) {
      small.release();
      busy.set(false);
      fail("resize", t);
      return null;
    }
    writer.execute(() -> {
      try {
        write(small, captureTimeNanos);
      } catch (Throwable t) {
        fail("write", t);
      } finally {
        small.release();
        busy.set(false);
      }
    });
    return null;
  }

  private void write(Mat rgba, long captureTimeNanos) throws IOException {
    File file = new File(dir, (captureTimeNanos / 1_000_000L) + ".jpg");
    if (!useBitmap) {
      Mat bgr = new Mat();
      try {
        Imgproc.cvtColor(rgba, bgr, rgba.channels() == 4 ? Imgproc.COLOR_RGBA2BGR : Imgproc.COLOR_RGB2BGR);
        if (Imgcodecs.imwrite(file.getPath(), bgr, new MatOfInt(Imgcodecs.IMWRITE_JPEG_QUALITY, JPEG_QUALITY))) {
          saved++;
          return;
        }
      } catch (Throwable t) {
        RobotLog.ww("BioBuzzFrameRecorder", "imwrite failed (%s); using Bitmap", t);
      } finally {
        bgr.release();
      }
      useBitmap = true;
    }
    Bitmap bitmap = Bitmap.createBitmap(rgba.cols(), rgba.rows(), Bitmap.Config.ARGB_8888);
    try (FileOutputStream out = new FileOutputStream(file)) {
      Utils.matToBitmap(rgba, bitmap);
      if (!bitmap.compress(Bitmap.CompressFormat.JPEG, JPEG_QUALITY, out)) {
        throw new IOException("compress failed");
      }
      saved++;
    } finally {
      bitmap.recycle();
    }
  }

  private void pruneOldRuns() {
    File[] runs = ROOT.listFiles(File::isDirectory);
    if (runs == null || runs.length <= KEEP_RUNS) {
      return;
    }
    Arrays.sort(runs, Comparator.comparingLong(File::lastModified));
    for (int i = 0; i < runs.length - KEEP_RUNS; i++) {
      if (runs[i].equals(dir)) {
        continue;
      }
      File[] files = runs[i].listFiles();
      if (files != null) {
        for (File f : files) {
          //noinspection ResultOfMethodCallIgnored
          f.delete();
        }
      }
      //noinspection ResultOfMethodCallIgnored
      runs[i].delete();
    }
  }

  private void fail(String what, Throwable t) {
    failed = true;
    status = "off: " + what + (t == null ? "" : " " + t);
    RobotLog.ee("BioBuzzFrameRecorder", t, "Frame recording stopped: %s", what);
  }

  public int saved() {
    return saved;
  }

  public int skipped() {
    return skipped;
  }

  public String status() {
    return status;
  }

  /** Lets the last frame finish writing, then stops the writer. */
  public void close() {
    writer.shutdown();
  }

  @Override public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
      float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {}
}
