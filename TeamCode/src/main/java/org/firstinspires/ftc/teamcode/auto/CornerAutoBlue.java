package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Alliance;

@Autonomous(name = "Lebot Corner Blue", group = "Robot", preselectTeleOp = "Lebot Teleop")
public class CornerAutoBlue extends CornerAuto {
  public CornerAutoBlue() {
    super(Alliance.BLUE);
  }
}
