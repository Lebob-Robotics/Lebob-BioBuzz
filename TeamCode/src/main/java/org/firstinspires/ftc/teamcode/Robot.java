package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem.FeederState;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem.HoodPreset;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem.IndexerState;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem.IntakeState;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem.ShooterState;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class Robot {
  private final Telemetry telemetry;
  private final Gamepad driver;

  public final IntakeSubsystem intake;
  public final MecanumDriveSubsystem drive;
  public final OdometrySubsystem odometry;
  public final ShooterSubsystem shooter;
  public final FeederSubsystem feeder;
  public final IndexerSubsystem indexer;
  public final HoodSubsystem hood;
  public final VisionSubsystem vision;

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
    feeder = new FeederSubsystem(hardwareMap);
    indexer = new IndexerSubsystem(hardwareMap);
    hood = new HoodSubsystem(hardwareMap);
    vision = new VisionSubsystem(hardwareMap);
  }

  /** Called once when the OpMode enters INIT. */
  public void init() {
    odometry.init();
    hood.init();
    shooter.setShooterState(ShooterState.IDLE);
  }

  /** Called repeatedly while the OpMode is running. */
  public void periodic() {
    CommandScheduler.getInstance().run();
    drive.drive(-driver.left_stick_y, driver.left_stick_x, driver.right_stick_x,
      !driver.left_bumper, odometry.getPose().getHeading(AngleUnit.RADIANS));

    if (driver.a) {
      odometry.resetHeading();
    }

    boolean rightTriggerPressed = driver.right_trigger > triggerLimit;
    boolean leftTriggerPressed = driver.left_trigger > triggerLimit;
    boolean eject = driver.b;

    ShooterState newShooterState = ShooterState.IDLE;
    if (rightTriggerPressed) {
      newShooterState = ShooterState.SHOOT;
    } else if (eject) {
      newShooterState = ShooterState.EJECT;
    } else if (driver.x) {
      newShooterState = ShooterState.STOP;
    }
    shooter.setShooterState(newShooterState);

    IntakeState newIntakeState = IntakeState.STOP;
    if (eject) {
      newIntakeState = IntakeState.EJECT;
    } else if (leftTriggerPressed || rightTriggerPressed) {
      newIntakeState = IntakeState.INTAKE;
    }
    intake.setIntakeState(newIntakeState);

    FeederState newFeederState = FeederState.STOP;
    if (eject) {
      newFeederState = FeederState.EJECT;
    } else if (leftTriggerPressed || rightTriggerPressed) {
      newFeederState = FeederState.FEED;
    }
    feeder.setFeederState(newFeederState);

    // Intaking loads the tunnel without pushing balls into the shooter.
    IndexerState newIndexerState = IndexerState.STOP;
    if (eject) {
      newIndexerState = IndexerState.EJECT;
    } else if (rightTriggerPressed) {
      newIndexerState = IndexerState.FEED;
    }
    indexer.setIndexerState(newIndexerState);

    if (driver.dpad_up) {
      hood.setPreset(HoodPreset.FAR);
    } else if (driver.dpad_right) {
      hood.setPreset(HoodPreset.MID);
    } else if (driver.dpad_down) {
      hood.setPreset(HoodPreset.CLOSE);
    } else if (driver.dpad_left) {
      hood.setPreset(HoodPreset.STOW);
    }

    telemetry.addData("Pose", odometry.getPose());
    telemetry.addData("Shooter", shooter.getShooterState()
        + (shooter.isAtSpeed() ? " at speed" : ""));
    telemetry.addData("Feeder", feeder.getFeederState());
    telemetry.addData("Indexer", indexer.getIndexerState());
    telemetry.addData("Hood", String.format("%s  %d / %d",
        hood.getPreset(), hood.getPosition(), hood.getTarget()));
    telemetry.addData("Beam T/B", intake.isTopBroken() + " / " + intake.isBottomBroken());
    telemetry.addData("Camera L", vision.describe(VisionSubsystem.Side.LEFT));
    telemetry.addData("Camera R", vision.describe(VisionSubsystem.Side.RIGHT));

    telemetry.update();
  }

  /** Called once when the OpMode stops. */
  public void stop() {
    vision.close();
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().reset();
  }
}
