package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.Alliance;

/** The solo auto with no partner: it also takes the partner's preloads from the loading zone. */
@Autonomous(name = "Lets Solo Triple Blue", group = "Robot", preselectTeleOp = "Lebot Teleop")
public class SoloTripleAutoBlue extends SoloAuto {
  public SoloTripleAutoBlue() {
    super(Alliance.BLUE, true);
  }
}
