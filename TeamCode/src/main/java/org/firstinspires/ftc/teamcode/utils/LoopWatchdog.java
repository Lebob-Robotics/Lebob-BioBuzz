package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.RobotLog;

import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * Writes where the robot loop is stuck once it has gone {@link #STALL_S} without a feed: the loop thread's stack,
 * to the robot log and to {@link #FILE}, which survives the app restart that follows a hung OpMode.
 *
 * <p>Twice on 6 Nov (091421, 093333) the loop stopped in INIT with the right camera opening, and the SDK restarted
 * the app when the OpMode would not stop; the log ended at that loop and gave no clue to the call that hung.
 */
public final class LoopWatchdog {
  public static double STALL_S = 2.0;
  public static final String FILE = "/sdcard/FIRST/loop_stalls.txt";

  private volatile Thread loopThread;
  private volatile long lastFeedNanos;
  private volatile boolean running;
  private Thread watcher;

  /** Starts watching the calling thread. */
  public synchronized void start() {
    feed();
    if (watcher != null) {
      return;
    }
    running = true;
    watcher = new Thread(this::watch, "LoopWatchdog");
    watcher.setDaemon(true);
    watcher.start();
  }

  /** Once per loop, from the loop thread. */
  public void feed() {
    loopThread = Thread.currentThread();
    lastFeedNanos = System.nanoTime();
  }

  public synchronized void stop() {
    running = false;
    if (watcher != null) {
      watcher.interrupt();
      watcher = null;
    }
  }

  private void watch() {
    long reportedFeed = 0;
    while (running) {
      try {
        Thread.sleep(250);
      } catch (InterruptedException e) {
        return;
      }
      long fed = lastFeedNanos;
      double stalledS = (System.nanoTime() - fed) * 1e-9;
      Thread t = loopThread;
      if (stalledS >= STALL_S && fed != reportedFeed && t != null) {
        reportedFeed = fed;
        report(t, stalledS);
      }
    }
  }

  private static void report(Thread t, double stalledS) {
    StringBuilder sb = new StringBuilder();
    sb.append(new SimpleDateFormat("HH:mm:ss.SSS", Locale.US).format(new Date()))
        .append(String.format(Locale.US, " loop stalled %.1f s in %s (%s)%n", stalledS, t.getName(), t.getState()));
    for (StackTraceElement e : t.getStackTrace()) {
      sb.append("    at ").append(e).append('\n');
    }
    RobotLog.ww("LoopWatchdog", "%s", sb);
    try (FileWriter w = new FileWriter(FILE, true)) {
      w.write(sb.append('\n').toString());
    } catch (IOException e) {
      RobotLog.ee("LoopWatchdog", e, "Could not write %s", FILE);
    }
  }
}
