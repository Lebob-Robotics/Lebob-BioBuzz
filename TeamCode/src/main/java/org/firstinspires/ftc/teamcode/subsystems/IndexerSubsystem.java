package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

/** Inclined compliant-wheel ramp that carries balls to the shooter. Brakes so held balls stay put. */
public class IndexerSubsystem extends SubsystemBase {
    private final DcMotorEx motor;

    public IndexerSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, Constants.INDEXER);
        motor.setDirection(Constants.INDEXER_DIRECTION);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void feed() {
        motor.setPower(Constants.INDEXER_POWER);
    }

    public void reverse() {
        motor.setPower(-Constants.INDEXER_POWER);
    }

    public void stop() {
        motor.setPower(0);
    }
}
