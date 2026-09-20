package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

/** Front compliant-wheel roller. Open loop. */
public class IntakeSubsystem extends SubsystemBase {
    private final DcMotorEx motor;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, Constants.INTAKE);
        motor.setDirection(Constants.INTAKE_DIRECTION);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void run() {
        motor.setPower(Constants.INTAKE_POWER);
    }

    public void reverse() {
        motor.setPower(-Constants.INTAKE_POWER);
    }

    public void stop() {
        motor.setPower(0);
    }
}
