package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.drivetrain.DrivePowers;

/**
 * Keeps Pedro's powers for the Robot to drive with, instead of driving the motors. The Robot turns them into a
 * field-centric drive and, while it aims, replaces the turn with its own heading lock.
 */
public final class CapturedDrivetrain extends BioBuzzMecanum {
  private double scale = 1.0;
  private double strafeBias = 0.0;
  private double forwardBias = 0.0;

  /** Every power Pedro asks for is multiplied by this, for slow passes such as sweeping the Garden. */
  public void setScale(double scale) {
    this.scale = scale;
  }

  /** Added to Pedro's strafe (left positive), after the scale: a steady push, such as against a wall. */
  public void setStrafeBias(double bias) {
    this.strafeBias = bias;
  }

  /** Added to Pedro's forward power, after the scale: with the strafe bias, a push in any direction. */
  public void setForwardBias(double bias) {
    this.forwardBias = bias;
  }

  @Override
  public void drive(DrivePowers powers, boolean manual) {
    double strafe = Math.max(-1, Math.min(1, powers.strafe() * scale + strafeBias));
    double forward = Math.max(-1, Math.min(1, powers.forward() * scale + forwardBias));
    last = new DrivePowers(forward, strafe, powers.turn() * scale);
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
