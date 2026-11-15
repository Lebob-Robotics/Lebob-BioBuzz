package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Alliance;

@Autonomous(name = "Lebot Auto Blue", group = "Robot", preselectTeleOp = "Lebot Teleop")
public class ScoringAutoBlue extends ScoringAuto {
  public ScoringAutoBlue() {
    super(Alliance.BLUE);
  }
}
