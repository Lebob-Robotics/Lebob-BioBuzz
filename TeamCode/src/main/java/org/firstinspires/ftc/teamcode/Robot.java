package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;

public class Robot {
    private final Telemetry telemetry;
    private final GamepadEx driver;

    public final MecanumDriveSubsystem drive;
    public final OdometrySubsystem odometry;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, Gamepad driverGamepad) {
        this.telemetry = telemetry;
        this.driver = new GamepadEx(driverGamepad);

        // One bulk read per hub per loop instead of one bus transaction per motor read.
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        CommandScheduler.getInstance().reset();

        drive = new MecanumDriveSubsystem(hardwareMap);
        odometry = new OdometrySubsystem(hardwareMap);
    }

    /** Called once when the OpMode enters INIT. */
    public void init() {
        odometry.init();
    }

    /** Called repeatedly while the OpMode is running. */
    public void periodic() {
        driver.readButtons();
        CommandScheduler.getInstance().run();

        if (driver.wasJustPressed(GamepadKeys.Button.A)) {
            odometry.resetHeading();
        }

        // Hold left bumper to drive robot-relative; otherwise drive field-relative.
        boolean fieldCentric = !driver.isDown(GamepadKeys.Button.LEFT_BUMPER);
        drive.drive(driver.getLeftY(), driver.getLeftX(), driver.getRightX(),
                fieldCentric, odometry.getPose().getHeading(AngleUnit.RADIANS));

        telemetry.addData("Pose", odometry.getPose());
        telemetry.update();
    }

    /** Called once when the OpMode stops. */
    public void stop() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();
    }
}
