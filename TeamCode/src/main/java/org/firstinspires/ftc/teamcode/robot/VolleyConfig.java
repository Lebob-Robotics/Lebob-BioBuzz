package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.shooting.volley.VolleyConstants;

/** Volley Tuning's settings in place of VolleyConstants' for the Robot's volleys; NaN (null) for the constant. */
final class VolleyConfig {
  private double firstTrimDeg = Double.NaN;
  private double laterTrimDeg = Double.NaN;
  private boolean trimsBackward = false; // The trims were tuned on backward volleys, not hooded ones
  private double indexerPower = Double.NaN;
  private double feedPower = Double.NaN;
  private double boostRpm = Double.NaN;
  private Boolean fastRecovery = null;
  private double headingTrimDeg = Double.NaN;

  void setTrimsDeg(double firstDeg, double laterDeg, boolean backward) {
    firstTrimDeg = firstDeg;
    laterTrimDeg = laterDeg;
    trimsBackward = backward;
  }

  void setFeedPower(double power) { feedPower = power; }

  void setIndexerPower(double power) { indexerPower = power; }

  void setBoostRpm(double rpm) { boostRpm = rpm; }

  void setFastRecovery(Boolean on) { fastRecovery = on; }

  void setHeadingTrimDeg(double deg) { headingTrimDeg = deg; }

  /** Automatic calibration's aim correction; NaN for none. */
  double headingTrimDeg() { return headingTrimDeg; }

  double feedPower() {
    return Double.isNaN(feedPower) ? VolleyConstants.VOLLEY_FEED_POWER : feedPower;
  }

  double indexerPower() {
    return Double.isNaN(indexerPower)
        ? feedPower() * VolleyConstants.VOLLEY_INDEXER_SCALE : indexerPower;
  }

  double boostRpm() {
    return Double.isNaN(boostRpm) ? VolleyConstants.VOLLEY_BOOST_RPM : boostRpm;
  }

  boolean fastRecovery() {
    return fastRecovery != null ? fastRecovery : VolleyConstants.VOLLEY_FAST_RECOVERY;
  }

  /** The trim for the volley's first ball or a later one. */
  double trimDeg(boolean first, boolean backward) {
    return trimDeg(first ? firstTrimDeg : laterTrimDeg, trimsBackward, backward, first);
  }

  /**
   * A backward volley uses only trims tuned on backward volleys, else the backward constants; a hooded one only
   * hooded trims, else the hooded constants.
   */
  static double trimDeg(double trim, boolean trimsBackward, boolean backward, boolean first) {
    if (backward) {
      if (trimsBackward && !Double.isNaN(trim)) { return trim; }
      return first ? VolleyConstants.VOLLEY_BACKWARD_FIRST_BALL_TRIM_DEG
          : VolleyConstants.VOLLEY_BACKWARD_LATER_BALL_TRIM_DEG;
    }
    if (trimsBackward || Double.isNaN(trim)) {
      return first ? VolleyConstants.VOLLEY_FIRST_BALL_TRIM_DEG
          : VolleyConstants.VOLLEY_LATER_BALL_TRIM_DEG;
    }
    return trim;
  }
}
