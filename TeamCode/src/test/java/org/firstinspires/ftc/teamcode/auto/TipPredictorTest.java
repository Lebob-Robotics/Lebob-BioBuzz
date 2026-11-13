package org.firstinspires.ftc.teamcode.auto;

import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.utils.VisionCorrector.Cell;
import org.junit.Test;

public class TipPredictorTest {
  @Test
  public void callsTheTipOnTheThirdBallThenTheEighth() {
    TipPredictor tips = new TipPredictor(Cell.AUDIENCE);
    tips.onBall(1.0);
    tips.onBall(1.4);
    assertEquals(Cell.AUDIENCE, tips.cell());
    tips.onBall(1.8);
    assertEquals("the fourth ball goes to the Cell now up", Cell.SCORING, tips.cell());
    tips.onVision(Cell.SCORING, 0, 2.6); // The tags then agree: not a second tip
    assertEquals(Cell.SCORING, tips.cell());
    for (int i = 0; i < 7; i++) {
      tips.onBall(3 + i);
    }
    assertEquals(Cell.SCORING, tips.cell());
    tips.onBall(10);
    assertEquals(Cell.AUDIENCE, tips.cell());
  }

  @Test
  public void aTipTheTagsSeeFirstIsTaken() {
    TipPredictor tips = new TipPredictor(Cell.AUDIENCE);
    tips.onBall(1.0);
    tips.onVision(Cell.SCORING, 0, 1.5); // Partner balls tipped it
    assertEquals(Cell.SCORING, tips.cell());
    assertEquals(0, tips.ballsSinceTip());
  }

  @Test
  public void aCalledTipTheTagsKeepDenyingIsUndone() {
    TipPredictor tips = new TipPredictor(Cell.AUDIENCE);
    tips.onBall(1.0);
    tips.onBall(1.4);
    tips.onBall(1.8); // Called: SCORING
    tips.onVision(Cell.AUDIENCE, 10, 2.0);
    tips.onVision(Cell.AUDIENCE, 20, 2.5); // Ball still in the air: no say yet
    assertEquals(Cell.SCORING, tips.cell());
    tips.onVision(Cell.AUDIENCE, 20, 1.8 + TipPredictor.LAND_S + 0.01);
    tips.onVision(Cell.AUDIENCE, 22, 3.5);
    assertEquals(Cell.SCORING, tips.cell());
    tips.onVision(Cell.AUDIENCE, 20 + TipPredictor.UNDO_FRAMES, 3.6);
    assertEquals(Cell.AUDIENCE, tips.cell());
  }
}
