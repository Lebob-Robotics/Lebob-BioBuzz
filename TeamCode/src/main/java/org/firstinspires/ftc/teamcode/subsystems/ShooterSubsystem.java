package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.ShooterMath;

/** Two flywheels, one motor each, held at a shared RPM by the hub's velocity controller. */
public class ShooterSubsystem extends SubsystemBase {
    private final DcMotorEx left;
    private final DcMotorEx right;
    private boolean running;
    private double leftRpm;
    private double rightRpm;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        left = hardwareMap.get(DcMotorEx.class, Constants.SHOOTER_LEFT);
        right = hardwareMap.get(DcMotorEx.class, Constants.SHOOTER_RIGHT);
        left.setDirection(Constants.SHOOTER_LEFT_DIRECTION);
        right.setDirection(Constants.SHOOTER_RIGHT_DIRECTION);
        for (DcMotorEx m : new DcMotorEx[]{left, right}) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            m.setVelocityPIDFCoefficients(Constants.SHOOTER_P, Constants.SHOOTER_I, Constants.SHOOTER_D, Constants.SHOOTER_F);
        }
    }

    /** Back into RUN_USING_ENCODER so the hub PID holds the setpoint. */
    public void spinUp() {
        running = true;
        double tps = ShooterMath.rpmToTicksPerSecond(Constants.SHOOTER_SETPOINT_RPM, Constants.SHOOTER_TICKS_PER_REV);
        for (DcMotorEx m : new DcMotorEx[]{left, right}) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        left.setVelocity(tps);
        right.setVelocity(tps);
    }

    /**
     * Drops out of RUN_USING_ENCODER before cutting power, so the wheels coast down (FLOAT)
     * instead of the hub PID braking them to zero velocity. spinUp() restores velocity control;
     * the controller keeps PIDF coefficients per mode, so the constructor's gains survive.
     */
    public void idle() {
        running = false;
        for (DcMotorEx m : new DcMotorEx[]{left, right}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setPower(0);
        }
    }

    public void toggle() {
        if (running) idle(); else spinUp();
    }

    public boolean isRunning() {
        return running;
    }

    /** One velocity read per motor per loop; everything else reads the cache (bulk read stays at one). */
    @Override
    public void periodic() {
        leftRpm = ShooterMath.ticksPerSecondToRpm(left.getVelocity(), Constants.SHOOTER_TICKS_PER_REV);
        rightRpm = ShooterMath.ticksPerSecondToRpm(right.getVelocity(), Constants.SHOOTER_TICKS_PER_REV);
    }

    /** True when both wheels are within tolerance of the setpoint. */
    public boolean atSpeed() {
        return running
                && Math.abs(leftRpm - Constants.SHOOTER_SETPOINT_RPM) < Constants.SHOOTER_TOLERANCE_RPM
                && Math.abs(rightRpm - Constants.SHOOTER_SETPOINT_RPM) < Constants.SHOOTER_TOLERANCE_RPM;
    }

    public double getLeftRpm() {
        return leftRpm;
    }

    public double getRightRpm() {
        return rightRpm;
    }
}
