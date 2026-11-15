package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.utils.VisionCorrector.Cell;

/**
 * Which Cell of our Hive to aim at, a step ahead of the tags. The Hive tips once the up Cell holds enough balls,
 * and the tags only show it after the tip, so the ball after the one that tips it would go to the Cell now down.
 * This counts our balls into the up Cell and calls the tip as the ball that should tip it leaves.
 *
 * <p>The tags win: a tip they see first (partner balls) is taken as it is, and a predicted tip they keep
 * disagreeing with, once the ball has had time to land, is undone.
 */
public final class TipPredictor {
  /** Our balls into the up Cell before it tips: the first tip, then the next. Partner balls are not counted. */
  public static int[] TIP_AFTER_BALLS = {3, 8};
  /** Time for the ball to land and the Hive to tip before the tags may undo a predicted tip. */
  public static double LAND_S = 1.5;
  /** Frames agreeing with the old up Cell, after LAND_S, that undo a predicted tip. */
  public static int UNDO_FRAMES = 3;

  private Cell visionCell;
  private Cell predicted;
  private int tips = 0;
  private int balls = 0;
  private double predictedAtS = Double.NaN;
  private int agreeingAtLand = -1;

  public TipPredictor(Cell initial) {
    visionCell = initial;
    predicted = initial;
  }

  /**
   * Each loop, before any ball of the loop.
   *
   * @param visionUp        the up Cell as the tags show it
   * @param agreeingFrames  frames so far voting for visionUp (VisionCorrector.agreeingFrames)
   */
  public void onVision(Cell visionUp, int agreeingFrames, double nowS) {
    if (visionUp != visionCell) {
      visionCell = visionUp;
      if (predicted != visionUp) { // A tip we didn't call
        predicted = visionUp;
        tips++;
        balls = 0;
      }
      predictedAtS = Double.NaN;
      return;
    }
    if (predicted == visionCell || Double.isNaN(predictedAtS) || nowS - predictedAtS < LAND_S) {
      return;
    }
    if (agreeingAtLand < 0) {
      agreeingAtLand = agreeingFrames;
    } else if (agreeingFrames - agreeingAtLand >= UNDO_FRAMES) { // Still up after the ball landed: no tip
      predicted = visionCell;
      tips--;
      balls = 0;
      predictedAtS = Double.NaN;
    }
  }

  /** A ball of ours left for the predicted Cell. */
  public void onBall(double nowS) {
    balls++;
    if (tips < TIP_AFTER_BALLS.length && balls >= TIP_AFTER_BALLS[tips]) {
      predicted = predicted == Cell.AUDIENCE ? Cell.SCORING : Cell.AUDIENCE;
      tips++;
      balls = 0;
      predictedAtS = nowS;
      agreeingAtLand = -1;
    }
  }

  /** The Cell to aim the next ball at. */
  public Cell cell() {
    return predicted;
  }

  public int ballsSinceTip() {
    return balls;
  }
}
