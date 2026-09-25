package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.MecanumKinematics;

/**
 * Drivetrain subsystem for a four-motor mecanum base, with optional field-centric
 * driving using the Pinpoint's heading.
 */
public class MecanumDriveSubsystem extends SubsystemBase {
    private final DcMotor frontLeftDrive;
    private final DcMotor frontRightDrive;
    private final DcMotor backLeftDrive;
    private final DcMotor backRightDrive;

    public MecanumDriveSubsystem(HardwareMap hardwareMap) {
        frontLeftDrive = hardwareMap.get(DcMotor.class, Constants.FRONT_LEFT_DRIVE);
        frontRightDrive = hardwareMap.get(DcMotor.class, Constants.FRONT_RIGHT_DRIVE);
        backLeftDrive = hardwareMap.get(DcMotor.class, Constants.BACK_LEFT_DRIVE);
        backRightDrive = hardwareMap.get(DcMotor.class, Constants.BACK_RIGHT_DRIVE);

        frontLeftDrive.setDirection(Constants.FRONT_LEFT_DIRECTION);
        frontRightDrive.setDirection(Constants.FRONT_RIGHT_DIRECTION);
        backLeftDrive.setDirection(Constants.BACK_LEFT_DIRECTION);
        backRightDrive.setDirection(Constants.BACK_RIGHT_DIRECTION);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Drives the robot given forward/right/rotate joystick components.
     *
     * @param forward        forward power, [-1, 1]
     * @param right          strafe-right power, [-1, 1]
     * @param rotate         clockwise rotation power, [-1, 1]
     * @param fieldCentric   if true, forward/right are relative to the field instead of the robot
     * @param headingRadians the robot's current field heading; only used when fieldCentric is true
     */
    public void drive(double forward, double right, double rotate, boolean fieldCentric, double headingRadians) {
        if (fieldCentric) {
            double[] rr = MecanumKinematics.toRobotRelative(forward, right, headingRadians);
            forward = rr[0];
            right = rr[1];
        }
        double[] p = MecanumKinematics.mix(forward, right, rotate);
        frontLeftDrive.setPower(p[0]);
        frontRightDrive.setPower(p[1]);
        backLeftDrive.setPower(p[2]);
        backRightDrive.setPower(p[3]);
    }
}
