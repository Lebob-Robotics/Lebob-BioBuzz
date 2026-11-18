package org.firstinspires.ftc.teamcode.hardware;

/** Device names in the robot configuration on the Control Hub. Each must match the configuration exactly. */
public final class HardwareNames {
  private HardwareNames() {}

  public static final String DRIVE_FRONT_LEFT = "DriveFL";
  public static final String DRIVE_FRONT_RIGHT = "DriveFR";
  public static final String DRIVE_BACK_LEFT = "DriveBL";
  public static final String DRIVE_BACK_RIGHT = "DriveBR";

  public static final String SHOOTER_LEFT = "ShooterL";
  public static final String SHOOTER_RIGHT = "ShooterR";

  public static final String INTAKE_MOTOR = "IntakeMotor";
  /** The hood's REV encoder is read through IntakeMotor's encoder port; the hood never drives that motor. */
  public static final String HOOD_ENCODER = INTAKE_MOTOR;
  public static final String HOOD_MOTOR = "HoodMotor";

  /** Upstream indexer rollers, Expansion Hub servo port 0. */
  public static final String INDEXER_SERVO_LEFT = "IndexerServoL";
  /** Last indexer roller, Control Hub servo port 0. */
  public static final String INDEXER_SERVO_RIGHT = "IndexerServoR";
  /** More upstream indexer rollers, run with IndexerServoL, Control Hub servo port 5. */
  public static final String INDEXER_SERVO_3 = "IndexerServo3";
  /** The feeder on the current robot, Control Hub servo port 1. */
  public static final String FEEDER_SERVO = "FeederServo";
  /** A paired feeder's names; FeederServoR is used ahead of FeederServo when the configuration has it. */
  public static final String FEEDER_SERVO_RIGHT = "FeederServoR";
  public static final String FEEDER_SERVO_LEFT = "FeederServoL";
  /** The Pollen ramp, Expansion Hub servo port 1. */
  public static final String RAMP_SERVO = "RampServo";

  /** Unused while IntakeSubsystem.BEAM_BREAKS_WIRED is false, so not needed in the configuration. */
  public static final String BEAM_BREAK_TOP = "BeamBreakT";
  public static final String BEAM_BREAK_BOTTOM = "BeamBreakB";

  public static final String PINPOINT = "pinpoint";
  public static final String CAMERA_LEFT = "CameraL";
  public static final String CAMERA_RIGHT = "CameraR";
}
