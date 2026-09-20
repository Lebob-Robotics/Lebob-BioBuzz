package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

/** Hardware names, directions and tuning values. One home so drivers and Pedro Pathing can find them. */
public final class Constants {
    private Constants() {}

    // Robot Controller configuration names.
    public static final String FRONT_LEFT_DRIVE = "front_left_drive";
    public static final String FRONT_RIGHT_DRIVE = "front_right_drive";
    public static final String BACK_LEFT_DRIVE = "back_left_drive";
    public static final String BACK_RIGHT_DRIVE = "back_right_drive";
    public static final String INTAKE = "intake";
    public static final String INDEXER = "indexer";
    public static final String SHOOTER_LEFT = "shooter_left";
    public static final String SHOOTER_RIGHT = "shooter_right";
    public static final String PINPOINT = "pinpoint";
    public static final String WEBCAM = "Webcam 1";

    // Motor directions. Left side reversed so positive power on every drive motor goes forward.
    public static final DcMotorSimple.Direction FRONT_LEFT_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction FRONT_RIGHT_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction BACK_LEFT_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction BACK_RIGHT_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction INTAKE_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction INDEXER_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction SHOOTER_LEFT_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction SHOOTER_RIGHT_DIRECTION = DcMotorSimple.Direction.REVERSE;

    // Pinpoint pod offsets from the tracking point, mm. X pod: left of centre positive.
    // Y pod: forward of centre positive. Measure on the robot per the goBILDA setup guide.
    public static final double PINPOINT_X_OFFSET_MM = 0.0;
    public static final double PINPOINT_Y_OFFSET_MM = 0.0;

    // Intake and indexer open-loop powers.
    public static final double INTAKE_POWER = 1.0;
    public static final double INDEXER_POWER = 1.0;

    // Shooter. 1:1 Yellow Jacket, 28 ticks per rev, 6000 RPM free speed.
    public static final double SHOOTER_TICKS_PER_REV = 28.0;
    public static final double SHOOTER_SETPOINT_RPM = 3500.0;   // tune on the robot
    public static final double SHOOTER_TOLERANCE_RPM = 50.0;   // the shot table needs the wheel within this; see tools/shots/README.md
    // Velocity PIDF starting point: F = 32767 / max ticks per second, P = 0.1 F, I = 0.1 P, D = 0.
    public static final double SHOOTER_P = 1.17;
    public static final double SHOOTER_I = 0.117;
    public static final double SHOOTER_D = 0.0;
    public static final double SHOOTER_F = 11.7;

    // Driver controls.
    public static final double TRIGGER_THRESHOLD = 0.2;

    // Aim assist: rotation = -AIM_KP * bearingDeg, clamped. Bearing is positive to the left.
    public static final double AIM_KP = 0.02;
    public static final double AIM_MAX_ROTATE = 0.5;
    public static final double AIM_DEADBAND_DEG = 1.0;

    // Shoot on the move. Shooter geometry and the table live in the generated ShotTable.
    public static final double FEED_DELAY_S = 0.15;          // indexer feed command to ball exit, measure on the robot
    public static final double SHOOTER_TRIM_STEP_RPM = 50.0;  // D-pad up/down
    public static final double TARGET_EXPIRY_S = 5.0;         // how long the last tag fix is trusted
    public static final int POSE_BUFFER_SIZE = 50;            // about one second of loops

    // Camera lens relative to the robot centre. Fill in from the CAD once the mount is designed.
    public static final double CAMERA_FORWARD_M = 0.0;
    public static final double CAMERA_LEFT_M = 0.0;
    public static final double CAMERA_YAW_RAD = 0.0;          // positive turned left
    public static final double CAMERA_PITCH_RAD = 0.0;        // positive tilted up

    public static final String SHOT_LOG_DIR = "/sdcard/FIRST/shots";
}
