package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.drivetrain.DrivePowers;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

/** Drives the motors straight from Pedro's powers. For the tuners and tests, which run without the Robot. */
public final class DirectDrivetrain extends BioBuzzMecanum {
  private final MecanumDriveSubsystem drive;

  public DirectDrivetrain(MecanumDriveSubsystem drive) {
    this.drive = drive;
  }

  @Override
  public void drive(DrivePowers powers, boolean manual) {
    last = powers;
    drive.drive(powers.forward(), -powers.strafe(), -powers.turn(), false, 0);
  }

  @Override
  public void stop() {
    last = DrivePowers.zero();
    drive.stop();
  }
}
