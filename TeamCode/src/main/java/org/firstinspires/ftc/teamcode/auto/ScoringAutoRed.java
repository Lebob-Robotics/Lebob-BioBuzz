package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Alliance;

@Autonomous(name = "Lebot Auto Red", group = "Robot", preselectTeleOp = "Lebot Teleop")
public class ScoringAutoRed extends ScoringAuto {
  public ScoringAutoRed() {
    super(Alliance.RED);
  }
}
