package org.firstinspires.ftc.teamcode.utils;

/** What the shooter should do for one shot: the output of {@link ShootOnTheMove#calculate}. */
public final class Shot {
  public enum Reason {
    OK,
    OUT_OF_RANGE,
    CLOSING_TOO_FAST,
    BACKING_TOO_FAST,
    NO_SAFE_SHOT,
    SIDEWAYS_TOO_FAST,
    HOOD_OUT_OF_RANGE // Flatter than the hood's top stop allows
  }

  public final boolean valid;
  public final Reason reason;
  public final double targetRpm; // For the flywheels
  public final double hoodDeg;
  public final double timeOfFlightS; // Of the ball
  public final double turningRad; // Shooter turn that cancels the robot's sideways speed, so the ball still scores. Anticlockwise positive

  public final double distanceToLipM;
  public final double speedTowardCellMps;
  public final double sidewaysSpeedMps; // Left positive

  Shot(Reason reason, double targetRpm, double hoodDeg, double timeOfFlightS, double turningRad,
      double distanceToLipM, double speedTowardCellMps, double sidewaysSpeedMps) {
    this.valid = reason == Reason.OK;
    this.reason = reason;
    this.targetRpm = targetRpm;
    this.hoodDeg = hoodDeg;
    this.timeOfFlightS = timeOfFlightS;
    this.turningRad = turningRad;
    this.distanceToLipM = distanceToLipM;
    this.speedTowardCellMps = speedTowardCellMps;
    this.sidewaysSpeedMps = sidewaysSpeedMps;
  }

  static Shot impossible(Reason reason, double distanceToLipM, double speedTowardCellMps, double sidewaysSpeedMps) {
    return impossible(reason, distanceToLipM, speedTowardCellMps, sidewaysSpeedMps, 0.0);
  }

  /** @param turningRad the nearest shot's turn, so the robot keeps lining up while there is no shot */
  static Shot impossible(Reason reason, double distanceToLipM, double speedTowardCellMps, double sidewaysSpeedMps,
      double turningRad) {
    return new Shot(reason, Double.NaN, Double.NaN, Double.NaN, turningRad,
        distanceToLipM, speedTowardCellMps, sidewaysSpeedMps);
  }
}
