package org.firstinspires.ftc.teamcode.utils;

/** One RB press is one trial of at most four detected balls, followed by four marks. */
public final class VolleyTrial {
  public static final int BALLS = 4;
  public static final double TIMEOUT_SECONDS = 12;
  private boolean active;
  private boolean fed;
  private boolean armed;
  private boolean finishing;
  private int balls;
  private double start;
  private String reason = "";

  /** Release before arming: RB held while moving or marking cannot start a volley on arrival. */
  public void input(double now, boolean held, boolean positioned, boolean marksPending) {
    if (!held) { armed = true; }
    if (active && (!held || !positioned || now - start >= TIMEOUT_SECONDS)) {
      end(!held ? "released" : !positioned ? "position lost" : "timeout");
    }
    if (!active && !finishing && held && armed && positioned && !marksPending) {
      active = true; fed = false; balls = 0; start = now; armed = false; reason = "firing";
    } else if (held && !active) {
      armed = false;
    }
  }

  /** True only for the first feed of the trial, so its settings/bookkeeping are frozen once. */
  public boolean observe(boolean feedStarted, boolean ballLeft) {
    boolean firstFeed = active && feedStarted && !fed;
    if (active && feedStarted) { fed = true; }
    if (active && ballLeft) {
      balls++;
      if (balls >= BALLS) { end("four detected balls"); }
    }
    return firstFeed;
  }

  public void cancel() { if (active) { end("Circle cancelled"); } }
  private void end(String why) { active = false; finishing = fed; armed = false; reason = why; }
  public boolean takeFinished() { boolean done = finishing; finishing = false; return done; }
  public boolean active() { return active; }
  public int balls() { return balls; }
  public String reason() { return reason; }
}
