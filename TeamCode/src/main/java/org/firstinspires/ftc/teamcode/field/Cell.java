package org.firstinspires.ftc.teamcode.field;

/** One end of a Hive. Exactly one of the two Cells is up at a time. */
public enum Cell {
  AUDIENCE,
  SCORING;

  /** The Hive's other end. */
  public Cell other() {
    return this == AUDIENCE ? SCORING : AUDIENCE;
  }
}
