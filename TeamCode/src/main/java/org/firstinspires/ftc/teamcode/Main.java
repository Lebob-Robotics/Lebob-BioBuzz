package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "BioBuzz TeleOp", group = "Robot")
public class Main extends OpMode {
    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, gamepad1);
        robot.init();
    }

    @Override
    public void loop() {
        robot.periodic();
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
