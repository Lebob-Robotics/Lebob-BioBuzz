package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.drivetrain.DrivePowers;

/**
 * Keeps Pedro's powers for the Robot to drive with, instead of driving the motors. The Robot turns them into a
 * field-centric drive and, while it aims, replaces the turn with its own heading lock.
 */
public final class CapturedDrivetrain extends BioBuzzMecanum {
  private double scale = 1.0;

  /** Every power Pedro asks for is multiplied by this, for slow passes such as sweeping the Garden. */
  public void setScale(double scale) {
    this.scale = scale;
  }

  @Override
  public void drive(DrivePowers powers, boolean manual) {
    last = new DrivePowers(powers.forward() * scale, powers.strafe() * scale, powers.turn() * scale);
  }

  @Override
  public void stop() {
    last = DrivePowers.zero();
  }

  /** {forward, strafe left, turn counterclockwise}, each -1..1, as Pedro last asked. */
  public double[] powers() {
    return new double[] {last.forward(), last.strafe(), last.turn()};
  }
}
