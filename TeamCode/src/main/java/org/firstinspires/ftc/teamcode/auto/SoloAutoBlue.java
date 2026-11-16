package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.Alliance;

@Autonomous(name = "Lets Solo Double Blue", group = "Robot", preselectTeleOp = "Lebot Teleop")
public class SoloAutoBlue extends SoloAuto {
  public SoloAutoBlue() {
    super(Alliance.BLUE);
  }
}
