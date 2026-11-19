package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Lebot Teleop with RT's (and RB's) hooded aimed shot fired as a volley: each ball goes as soon as the wheels are
 * back at the matched speed after the one before, instead of one at a time with a pause between. Everything else,
 * controls included, is Lebot Teleop's. Backward shots, Y's full-power shot and shots without the Cell in view are
 * as in Lebot Teleop. The volley's settings are ShootOnTheMoveConstants VOLLEY_*; Volley Tuning finds them.
 */
@TeleOp(name = "Lebot Teleop Volley", group = "Robot")
public class MainVolley extends Main {
  @Override
  protected void configureRobot(Robot robot) {
    robot.setVolleyMode(true);
  }
}
