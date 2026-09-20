package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        // Left motors are flipped so that a positive power on every motor drives straight.
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

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
