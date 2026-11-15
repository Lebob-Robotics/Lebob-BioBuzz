package org.firstinspires.ftc.teamcode.opmodes;

import org.psilynx.psikit.ftc.PsiKitIterativeOpMode;

/** A PsiKit OpMode that also serves its log live to AdvantageScope while {@link Main#LIVE_LOGGING} is on. */
public abstract class LebotOpMode extends PsiKitIterativeOpMode {
  @Override
  protected int getRlogPort() {
    return Main.LIVE_LOGGING ? Main.RLOG_PORT : 0;
  }
}
