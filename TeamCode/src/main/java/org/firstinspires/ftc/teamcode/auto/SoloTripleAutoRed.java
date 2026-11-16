package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.Alliance;

/** The solo auto with no partner: it also takes the partner's preloads from the loading zone. */
@Autonomous(name = "Lets Solo Triple Red", group = "Robot", preselectTeleOp = "Lebot Teleop")
public class SoloTripleAutoRed extends SoloAuto {
  public SoloTripleAutoRed() {
    super(Alliance.RED, true);
  }
}
