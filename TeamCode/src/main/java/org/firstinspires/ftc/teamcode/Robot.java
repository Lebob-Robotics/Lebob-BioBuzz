package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class Robot {
    private final Telemetry telemetry;
    private final GamepadEx driver;
    private Alliance alliance = Alliance.RED;

    public final MecanumDriveSubsystem drive;
    public final OdometrySubsystem odometry;
    public final IntakeSubsystem intake;
    public final IndexerSubsystem indexer;
    public final ShooterSubsystem shooter;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, Gamepad driverGamepad) {
        this.telemetry = telemetry;
        this.driver = new GamepadEx(driverGamepad);

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        CommandScheduler.getInstance().reset();

        drive = new MecanumDriveSubsystem(hardwareMap);
        odometry = new OdometrySubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        indexer = new IndexerSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
    }

    /** Called once when the OpMode enters INIT. */
    public void init() {
        odometry.init();
    }

    /** Select the alliance before START with D-pad left or right. */
    public void initLoop() {
        driver.readButtons();
        CommandScheduler.getInstance().run();

        if (driver.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            alliance = Alliance.RED;
        } else if (driver.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            alliance = Alliance.BLUE;
        }

        telemetry.addData("Alliance (D-pad left/right)", alliance);
        telemetry.update();
    }

    /** Called repeatedly while the OpMode is running. */
    public void periodic() {
        driver.readButtons();
        CommandScheduler.getInstance().run();

        if (driver.wasJustPressed(GamepadKeys.Button.A)) {
            odometry.resetHeading();
        }

        // Hold left bumper to drive robot-relative; otherwise drive field-relative.
        drive.drive(driver.getLeftY(), driver.getLeftX(), driver.getRightX(),
                !driver.isDown(GamepadKeys.Button.LEFT_BUMPER),
                odometry.getPose().getHeading(AngleUnit.RADIANS));

        if (driver.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            shooter.toggle();
        }

        double leftTrigger = driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        double rightTrigger = driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        if (leftTrigger > Constants.TRIGGER_THRESHOLD) {
            intake.reverse();
            indexer.reverse();
        } else if (rightTrigger > Constants.TRIGGER_THRESHOLD) {
            intake.run();
            indexer.feed();
        } else if (driver.isDown(GamepadKeys.Button.X) && shooter.atSpeed()) {
            intake.stop();
            indexer.feed();
        } else {
            intake.stop();
            indexer.stop();
        }

        telemetry.addData("Alliance", alliance);
        telemetry.addData("Pose", odometry.getPose());
        telemetry.addData("Shooter", "%s target %.0f RPM  L %.0f  R %.0f  %s",
                shooter.isRunning() ? "ON" : "off", Constants.SHOOTER_SETPOINT_RPM,
                shooter.getLeftRpm(), shooter.getRightRpm(), shooter.atSpeed() ? "AT SPEED" : "");
        telemetry.update();
    }

    /** Called once when the OpMode stops. */
    public void stop() {
        drive.stop();
        intake.stop();
        indexer.stop();
        shooter.idle();
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();
    }
}
