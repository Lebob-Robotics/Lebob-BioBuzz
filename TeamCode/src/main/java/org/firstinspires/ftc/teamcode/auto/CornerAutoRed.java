package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Alliance;

@Autonomous(name = "Lebot Corner Red", group = "Robot", preselectTeleOp = "Lebot Teleop")
public class CornerAutoRed extends CornerAuto {
  public CornerAutoRed() {
    super(Alliance.RED);
  }
}
