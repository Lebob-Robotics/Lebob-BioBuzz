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

    public void spinUp() {
        running = true;
        double tps = ShooterMath.rpmToTicksPerSecond(Constants.SHOOTER_SETPOINT_RPM, Constants.SHOOTER_TICKS_PER_REV);
        left.setVelocity(tps);
        right.setVelocity(tps);
    }

    /** Cuts power and lets the flywheels coast down (FLOAT), rather than braking to zero velocity. */
    public void idle() {
        running = false;
        left.setPower(0);
        right.setPower(0);
    }

    public void toggle() {
        if (running) idle(); else spinUp();
    }

    public boolean isRunning() {
        return running;
    }

    /** True when both wheels are within tolerance of the setpoint. */
    public boolean atSpeed() {
        return running
                && Math.abs(getLeftRpm() - Constants.SHOOTER_SETPOINT_RPM) < Constants.SHOOTER_TOLERANCE_RPM
                && Math.abs(getRightRpm() - Constants.SHOOTER_SETPOINT_RPM) < Constants.SHOOTER_TOLERANCE_RPM;
    }

    public double getLeftRpm() {
        return ShooterMath.ticksPerSecondToRpm(left.getVelocity(), Constants.SHOOTER_TICKS_PER_REV);
    }

    public double getRightRpm() {
        return ShooterMath.ticksPerSecondToRpm(right.getVelocity(), Constants.SHOOTER_TICKS_PER_REV);
    }
}
