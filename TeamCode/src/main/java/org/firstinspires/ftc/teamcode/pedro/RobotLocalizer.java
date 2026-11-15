package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import org.firstinspires.ftc.teamcode.field.Units;

/**
 * Hands Pedro the Robot's pose, already corrected from the Hive tags, in Pedro's units (inches). The Robot owns
 * the pose: this never writes the Pinpoint, and {@link #setPose} only changes what Pedro sees until the next
 * {@link #set}.
 */
public final class RobotLocalizer implements Localizer {
  private MotionState state = MotionState.zero();

  /** Field pose in metres and radians, and field-frame velocity in m/s and rad/s. */
  public void set(double xM, double yM, double headingRad, double vxMps, double vyMps, double omegaRadPerS) {
    state = MotionState.ofVelocity(new Pose(xM / Units.M_PER_IN, yM / Units.M_PER_IN, headingRad),
        new Velocity(vxMps / Units.M_PER_IN, vyMps / Units.M_PER_IN, omegaRadPerS));
  }

  @Override
  public void setPose(Pose pose) {
    state = state.withPose(pose);
  }

  @Override
  public MotionState state() {
    return state;
  }

  @Override
  public void update() {}

  @Override
  public void reset() {}
}
