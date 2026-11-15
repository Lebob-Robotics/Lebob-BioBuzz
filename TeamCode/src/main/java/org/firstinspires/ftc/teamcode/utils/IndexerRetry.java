package org.firstinspires.ftc.teamcode.utils;

/** Relieve queue pressure briefly before retrying a volley indexer at full forward power. */
public final class IndexerRetry {
  public static final double WAIT_SECONDS = 1.5;
  public static final double PAUSE_SECONDS = 0.15;
  private double since = Double.NaN;
  private boolean paused;
  private boolean fullPower;

  public void update(double now, boolean indexingShot, boolean ballLeft) {
    if (!indexingShot || ballLeft) {
      since = Double.NaN;
      paused = false;
      fullPower = false;
      return;
    }
    if (Double.isNaN(since)) { since = now; }
    if (paused) {
      if (now - since >= PAUSE_SECONDS) {
        paused = false;
        fullPower = true;
        since = now;
      }
    } else if (now - since >= WAIT_SECONDS) {
      paused = true;
      fullPower = false;
      since = now;
    }
  }

  public boolean paused() { return paused; }
  public boolean fullPower() { return fullPower; }
}
