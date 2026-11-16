package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.Alliance;

@Autonomous(name = "Lets Solo Double Red", group = "Robot", preselectTeleOp = "Lebot Teleop")
public class SoloAutoRed extends SoloAuto {
  public SoloAutoRed() {
    super(Alliance.RED);
  }
}
