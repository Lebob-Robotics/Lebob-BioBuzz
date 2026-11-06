package org.firstinspires.ftc.teamcode.utils;

/**
 * Where one Shot Tuning ball went, as the driver marked it: in, bounced out, or a miss short, long, left and/or right.
 * Left and right are as seen from behind the shot, looking at the Cell.
 */
public final class BallMark {
  public final boolean in;
  public final boolean bounced;
  public final boolean shortMiss;
  public final boolean longMiss;
  public final boolean left;
  public final boolean right;

  public static final BallMark IN = new BallMark(true, false, false, false, false, false);
  public static final BallMark BOUNCE = new BallMark(false, true, false, false, false, false);

  public BallMark(boolean in, boolean bounced, boolean shortMiss, boolean longMiss, boolean left, boolean right) {
    this.in = in;
    this.bounced = bounced;
    this.shortMiss = shortMiss;
    this.longMiss = longMiss;
    this.left = left;
    this.right = right;
  }

  public static BallMark miss(boolean shortMiss, boolean longMiss, boolean left, boolean right) {
    return new BallMark(false, false, shortMiss, longMiss, left, right);
  }

  /** The mark {@link #toString} wrote. */
  public static BallMark parse(String text) {
    if ("IN".equals(text)) {
      return IN;
    }
    if ("BOUNCE".equals(text)) {
      return BOUNCE;
    }
    return miss(text.contains("SHORT"), text.contains("LONG"), text.contains("LEFT"), text.contains("RIGHT"));
  }

  /** Short or long, and not both. */
  public boolean hasRange() {
    return shortMiss != longMiss;
  }

  /** Left or right, and not both. */
  public boolean hasSide() {
    return left != right;
  }

  /** IN, BOUNCE, or the miss's directions joined by +, e.g. SHORT+LEFT. MISS when none was given. */
  @Override
  public String toString() {
    if (in) {
      return "IN";
    }
    if (bounced) {
      return "BOUNCE";
    }
    // Not String.join: the Control Hub's Android 7 doesn't have it
    StringBuilder parts = new StringBuilder();
    if (shortMiss) { parts.append("+SHORT"); }
    if (longMiss) { parts.append("+LONG"); }
    if (left) { parts.append("+LEFT"); }
    if (right) { parts.append("+RIGHT"); }
    return parts.length() == 0 ? "MISS" : parts.substring(1);
  }

  @Override
  public boolean equals(Object o) {
    return o instanceof BallMark && o.toString().equals(toString());
  }

  @Override
  public int hashCode() {
    return toString().hashCode();
  }
}
