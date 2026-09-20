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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
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

    private final ShotSolver solver = new ShotSolver(
            ShotTable.DIST_M, ShotTable.VEL_MPS, ShotTable.RPM, ShotTable.BAND_RPM, ShotTable.TOF_S,
            ShotTable.LAUNCH_ANGLE_DEG, ShotTable.EXIT_SPEED_PER_RPM, ShotTable.SHOOTER_OFFSET_M,
            Constants.FEED_DELAY_S, ShotTable.MIN_BAND_RPM, ShotTable.OPENING_WIDTH_M, ShotTable.LATERAL_CLEARANCE_M,
            Constants.SHOOTER_SETPOINT_RPM);
    private final TargetTracker tracker = new TargetTracker(
            Constants.POSE_BUFFER_SIZE, Constants.CAMERA_FORWARD_M, Constants.CAMERA_LEFT_M,
            Constants.CAMERA_YAW_RAD, Constants.CAMERA_PITCH_RAD, Constants.TARGET_EXPIRY_S);
    private ShotLog log;

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
        log = ShotLog.open(Constants.SHOT_LOG_DIR);
    }

    /** Called repeatedly while the OpMode is running. */
    public void periodic() {
        driver.readButtons();
        CommandScheduler.getInstance().run();
        long now = System.nanoTime();

        if (driver.wasJustPressed(GamepadKeys.Button.A)) {
            odometry.resetHeading();
            tracker.clear();   // the reset rotates the field frame; the old target and poses no longer apply
        }
        if (driver.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) shooter.toggle();
        if (driver.wasJustPressed(GamepadKeys.Button.DPAD_UP)) shooter.trim(Constants.SHOOTER_TRIM_STEP_RPM);
        if (driver.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) shooter.trim(-Constants.SHOOTER_TRIM_STEP_RPM);

        // Where we are, how fast we are going, and where the Cell is.
        Pose2D pose = odometry.getPose();
        double x = pose.getX(DistanceUnit.METER);
        double y = pose.getY(DistanceUnit.METER);
        double heading = pose.getHeading(AngleUnit.RADIANS);
        double[] vel = odometry.getVelocity();
        tracker.recordPose(now, x, y, heading);
        if (vision.hasTarget()) {
            tracker.update(vision.getTargetFrameNanos(), vision.getTargetX(), vision.getTargetY(), vision.getTargetZ());
        }

        // Aim. Y held: solver heading if we know where the Cell is, else the raw tag bearing, else the stick.
        boolean aiming = driver.isDown(GamepadKeys.Button.Y);
        double rotate = driver.getRightX();
        ShotSolver.Shot shot = null;
        if (aiming && tracker.hasTarget(now)) {
            shot = solver.solve(x, y, heading, vel[0], vel[1], tracker.getX(), tracker.getY());
            shooter.setTargetRpm(shot.valid ? shot.rpm : shot.idleRpm);
            rotate = aimRotation(Math.toDegrees(shot.headingErrorRad(heading)));
        } else if (aiming && vision.hasTarget()) {
            rotate = aimRotation(vision.getBearingDeg());
        }

        boolean fieldCentric = !driver.isDown(GamepadKeys.Button.LEFT_BUMPER);
        drive.drive(driver.getLeftY(), driver.getLeftX(), rotate, fieldCentric, heading);

        // Fire gate. With a solved shot: valid, at speed, and pointing inside the Cell width.
        boolean fire = driver.isDown(GamepadKeys.Button.X);
        boolean ready = shot == null
                ? shooter.atSpeed()
                : shot.valid && shooter.atSpeed() && Math.abs(shot.headingErrorRad(heading)) < shot.headingToleranceRad;
        double rightTrigger = driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        double leftTrigger = driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        boolean firing = false;
        if (leftTrigger > Constants.TRIGGER_THRESHOLD) {
            intake.reverse();
            indexer.reverse();
        } else if (rightTrigger > Constants.TRIGGER_THRESHOLD) {
            intake.run();
            indexer.feed();
        } else if (fire && ready) {
            intake.stop();
            indexer.feed();
            firing = true;
        } else {
            intake.stop();
            indexer.stop();
        }

        if (shot != null && log != null) {
            log.row(x, y, heading, vel[0], vel[1], shot.distanceM, shot.radialVel, shot.tangentialVel,
                    shot.valid ? shot.rpm : shot.idleRpm, shooter.getLeftRpm(), shooter.getRightRpm(),
                    shot.headingErrorRad(heading), shot.valid ? 1 : 0, firing ? 1 : 0);
        }

        telemetry.addData("Alliance", vision.getAlliance());
        telemetry.addData("Pose", "%.2f %.2f m  %.0f deg  v %.2f %.2f", x, y, Math.toDegrees(heading), vel[0], vel[1]);
        telemetry.addData("Shooter", "%s target %.0f (trim %+.0f)  L %.0f  R %.0f  %s",
                shooter.isRunning() ? "ON" : "off", shooter.getTargetRpm(), shooter.getTrimRpm(),
                shooter.getLeftRpm(), shooter.getRightRpm(), shooter.atSpeed() ? "AT SPEED" : "");
        telemetry.addData("Target", tracker.hasTarget(now)
                ? String.format("%.2f %.2f m  seen %.1f s ago", tracker.getX(), tracker.getY(), tracker.ageS(now))
                : vision.hasTarget() ? "tag only, no pose" : "none");
        telemetry.addData("Shot", shot == null ? "hold Y" : shot.valid
                ? String.format("OK d %.2f  radial %+.2f  tang %+.2f  rpm %.0f  err %+.1f deg  tol %.1f",
                        shot.distanceM, shot.radialVel, shot.tangentialVel, shot.rpm,
                        Math.toDegrees(shot.headingErrorRad(heading)), Math.toDegrees(shot.headingToleranceRad))
                : String.format("NO SHOT: %s  d %.2f  radial %+.2f", shot.reason, shot.distanceM, shot.radialVel));
        telemetry.addData("Log", log == null ? "not writing" : log.fileName());
        telemetry.addData("Battery", "%.1f V", battery.getVoltage());
        telemetry.update();
    }

    /** Called once when the OpMode stops. */
    public void stop() {
        if (log != null) log.close();
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
