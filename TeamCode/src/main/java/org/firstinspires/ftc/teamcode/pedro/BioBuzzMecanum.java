package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;

import java.util.HashMap;
import java.util.Map;

/**
 * Pedro's view of the BioBuzz mecanum drive. Pedro's powers are forward, strafe left and turn counterclockwise;
 * {@code MecanumDriveSubsystem} takes forward, right and clockwise, so strafe and turn change sign on the way.
 * The wheel mix is the same either way, so {@link #maxScaling} is Pedro's own Mecanum one: the tuned constants
 * then mean the same here as they would on Pedro's drivetrain.
 */
public abstract class BioBuzzMecanum implements Drivetrain {
  protected DrivePowers last = DrivePowers.zero();

  /** Wheel powers {FL, FR, BL, BR} before scaling, from Pedro's powers. Same as MecanumDriveSubsystem.mix. */
  static double[] wheels(DrivePowers p) {
    double f = p.forward();
    double s = p.strafe();
    double t = p.turn();
    return new double[] {f - s - t, f + s + t, f + s - t, f - s + t};
  }

  @Override
  public double maxScaling(DrivePowers current, DrivePowers delta) {
    double lambda = 1.0;
    double[] a = wheels(current);
    double[] b = wheels(delta);
    for (int i = 0; i < 4; i++) {
      if (Math.abs(b[i]) < 1e-9) {
        continue;
      }
      double t1 = (1.0 - a[i]) / b[i];
      double t2 = (-1.0 - a[i]) / b[i];
      if (t1 >= 0.0 && t1 < lambda) {
        lambda = t1;
      }
      if (t2 >= 0.0 && t2 < lambda) {
        lambda = t2;
      }
    }
    return Math.max(0.0, Math.min(1.0, lambda));
  }

  @Override
  public double interpolateVelocity(double xRadius, double yRadius, double theta) {
    return 1.0 / (Math.abs(Math.cos(theta)) / xRadius + Math.abs(Math.sin(theta)) / yRadius);
  }

  @Override
  public void stop(boolean brake) {
    stop(); // The drive motors always float
  }

  @Override
  public Map<String, Object> debug() {
    Map<String, Object> map = new HashMap<>();
    map.put("forward", last.forward());
    map.put("strafe", last.strafe());
    map.put("turn", last.turn());
    return map;
  }
}
