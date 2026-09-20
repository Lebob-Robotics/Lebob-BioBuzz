package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class Robot {
    private final Telemetry telemetry;
    private final GamepadEx driver;
    private final VoltageSensor battery;

    public final MecanumDriveSubsystem drive;
    public final OdometrySubsystem odometry;
    public final IntakeSubsystem intake;
    public final IndexerSubsystem indexer;
    public final ShooterSubsystem shooter;
    public final VisionSubsystem vision;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, Gamepad driverGamepad) {
        this.telemetry = telemetry;
        this.driver = new GamepadEx(driverGamepad);

        // One bulk read per hub per loop instead of one bus transaction per motor read.
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        battery = hardwareMap.voltageSensor.iterator().next();

        CommandScheduler.getInstance().reset();

        drive = new MecanumDriveSubsystem(hardwareMap);
        odometry = new OdometrySubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        indexer = new IndexerSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap);
    }

    /** Called once when the OpMode enters INIT. */
    public void init() {
        odometry.init();
    }

    /** Called repeatedly while the OpMode sits in INIT. D-pad left = red, right = blue. */
    public void initLoop() {
        driver.readButtons();
        CommandScheduler.getInstance().run();
        if (driver.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) vision.setAlliance(Alliance.RED);
        if (driver.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) vision.setAlliance(Alliance.BLUE);
        telemetry.addData("Alliance (dpad L/R)", vision.getAlliance());
        telemetry.addData("Camera", vision.isAvailable() ? "ok" : "NOT FOUND");
        telemetry.addData("Camera sees", vision.getTargetName());
        telemetry.update();
    }

    /** Called once when the driver presses START. */
    public void start() {
        vision.stopLiveView();
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
        double rotate = driver.getRightX();
        if (driver.isDown(GamepadKeys.Button.Y) && vision.hasTarget()) {
            rotate = aimRotation(vision.getBearingDeg());
        }
        drive.drive(driver.getLeftY(), driver.getLeftX(), rotate,
                fieldCentric, odometry.getPose().getHeading(AngleUnit.RADIANS));

        if (driver.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            shooter.toggle();
        }
        boolean fire = driver.isDown(GamepadKeys.Button.X);

        double rightTrigger = driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        double leftTrigger = driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        if (leftTrigger > Constants.TRIGGER_THRESHOLD) {
            intake.reverse();
            indexer.reverse();
        } else if (rightTrigger > Constants.TRIGGER_THRESHOLD) {
            intake.run();
            indexer.feed();
        } else if (fire && shooter.atSpeed()) {
            // Fire: feed only while the flywheels are at speed so every shot leaves at the same velocity.
            intake.stop();
            indexer.feed();
        } else {
            intake.stop();
            indexer.stop();
        }

        telemetry.addData("Alliance", vision.getAlliance());
        telemetry.addData("Pose", odometry.getPose());
        telemetry.addData("Shooter", "%s target %.0f  L %.0f  R %.0f  %s",
                shooter.isRunning() ? "ON" : "off", Constants.SHOOTER_SETPOINT_RPM,
                shooter.getLeftRpm(), shooter.getRightRpm(), shooter.atSpeed() ? "AT SPEED" : "");
        telemetry.addData("Tag bearing", vision.hasTarget() ? String.format("%.1f deg", vision.getBearingDeg()) : "no target");
        telemetry.addData("Battery", "%.1f V", battery.getVoltage());
        telemetry.update();
    }

    /** Called once when the OpMode stops. */
    public void stop() {
        shooter.idle();
        vision.close();
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();
    }

    /** P-controller from tag bearing (deg, +left) to clockwise rotation power. */
    private static double aimRotation(double bearingDeg) {
        if (Math.abs(bearingDeg) < Constants.AIM_DEADBAND_DEG) return 0;
        double rotate = -Constants.AIM_KP * bearingDeg;
        return Math.max(-Constants.AIM_MAX_ROTATE, Math.min(Constants.AIM_MAX_ROTATE, rotate));
    }
}
