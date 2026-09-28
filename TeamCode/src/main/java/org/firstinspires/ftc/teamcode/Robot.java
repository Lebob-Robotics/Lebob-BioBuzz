package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem.IntakeState;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem.ShooterState;

public class Robot {
  private final Telemetry telemetry;
  private final Gamepad driver;

  public final IntakeSubsystem intake;
  public final MecanumDriveSubsystem drive;
  public final OdometrySubsystem odometry;
  public final ShooterSubsystem shooter;

  public static final double triggerLimit = 0.5;

  public Robot(
      HardwareMap hardwareMap,
      Telemetry telemetry,
      Gamepad driver) {
    this.telemetry = telemetry;
    this.driver = driver;

    CommandScheduler.getInstance().reset();

    // Construct subsystems.
    intake = new IntakeSubsystem(hardwareMap);
    drive = new MecanumDriveSubsystem(hardwareMap);
    odometry = new OdometrySubsystem(hardwareMap);
    shooter = new ShooterSubsystem(hardwareMap);

  }

  /** Called once when the OpMode enters INIT. */
  public void init() {
    odometry.init();
    shooter.setShooterState(ShooterState.IDLE);
  }

  /** Called repeatedly while the OpMode is running. */
  public void periodic() {
    CommandScheduler.getInstance().run();

    if (driver.a) {
      odometry.resetHeading();
    }

    if (driver.right_trigger > triggerLimit) {
      shooter.setShooterState(ShooterState.SHOOT);
    } else if (driver.b) {
      shooter.setShooterState(ShooterState.EJECT);
    } else {
      shooter.setShooterState(ShooterState.IDLE);
    }

    if (driver.b) {
      intake.setIntakeState(IntakeState.EJECT);
    } else if (driver.left_trigger > triggerLimit) {
      intake.setIntakeState(IntakeState.INTAKE);
    } else {
      intake.setIntakeState(IntakeState.STOP);
    }

    // Hold left bumper to drive robot-relative; otherwise drive field-relative.
    drive.drive(-driver.left_stick_y, driver.left_stick_x, driver.right_stick_x,
        !driver.left_bumper, odometry.getPose().getHeading(AngleUnit.RADIANS));

    telemetry.addData("Pose", odometry.getPose());
    telemetry.update();
  }

  /** Called once when the OpMode stops. */
  public void stop() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().reset();
  }
}
