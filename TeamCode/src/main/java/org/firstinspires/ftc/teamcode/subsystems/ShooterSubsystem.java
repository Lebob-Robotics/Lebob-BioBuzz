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
    private double targetRpm = Constants.SHOOTER_SETPOINT_RPM;
    private double trimRpm = 0;

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
        for (DcMotorEx m : new DcMotorEx[]{left, right}) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        sendVelocity();
    }

    /** Sends the current target to both motors without touching the run mode. */
    private void sendVelocity() {
        double tps = ShooterMath.rpmToTicksPerSecond(getTargetRpm(), Constants.SHOOTER_TICKS_PER_REV);
        left.setVelocity(tps);
        right.setVelocity(tps);
    }

    /**
     * Changes the target. Ignores moves smaller than Constants.SHOOTER_RETARGET_RPM (noise next to the
     * scoring band) and, when running, only re-sends the velocity: re-asserting RUN_USING_ENCODER every
     * loop would restart the hub PID's settling instead of letting it hold.
     */
    public void setTargetRpm(double rpm) {
        if (Math.abs(rpm - targetRpm) < Constants.SHOOTER_RETARGET_RPM) return;
        targetRpm = rpm;
        if (running) sendVelocity();
    }

    /** Target plus the driver's trim. */
    public double getTargetRpm() {
        return targetRpm + trimRpm;
    }

    /** Driver adjustment applied to every target for the rest of the run. */
    public void trim(double deltaRpm) {
        trimRpm += deltaRpm;
        if (running) sendVelocity();
    }

    public double getTrimRpm() {
        return trimRpm;
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

    /** True when both wheels are within tolerance of the current target. */
    public boolean atSpeed() {
        double target = getTargetRpm();
        return running
                && Math.abs(leftRpm - target) < Constants.SHOOTER_TOLERANCE_RPM
                && Math.abs(rightRpm - target) < Constants.SHOOTER_TOLERANCE_RPM;
    }

    public double getLeftRpm() {
        return leftRpm;
    }

    public double getRightRpm() {
        return rightRpm;
    }
}
