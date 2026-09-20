package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants;

/**
 * Wraps a goBILDA Pinpoint Odometry Computer reading two dead-wheel pods, and
 * reports the robot's field pose.
 */
public class OdometrySubsystem extends SubsystemBase {
    private final GoBildaPinpointDriver pinpoint;

    public OdometrySubsystem(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.PINPOINT);
    }

    /** Called once from the OpMode's init(). Configures the Pinpoint and zeroes its pose. */
    public void init() {
        pinpoint.setOffsets(Constants.PINPOINT_X_OFFSET_MM, Constants.PINPOINT_Y_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Robot must be stationary during this call.
        pinpoint.resetPosAndIMU();
    }

    @Override
    public void periodic() {
        pinpoint.update();
    }

    public Pose2D getPose() {
        return pinpoint.getPosition();
    }

    /** Zeroes the reported heading in place, keeping the current position. */
    public void resetHeading() {
        Pose2D current = pinpoint.getPosition();
        pinpoint.setPosition(new Pose2D(
                DistanceUnit.MM, current.getX(DistanceUnit.MM), current.getY(DistanceUnit.MM),
                AngleUnit.DEGREES, 0));
    }
}
