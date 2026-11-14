package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.psilynx.psikit.core.Logger;

/**
 * The Pollen ramp: one REV servo, RampServo on Expansion Hub servo port 1, stowed or deployed. It is stowed when an
 * OpMode INITs, so the servo holds it there instead of letting it flop, and TeleOp's D-pad up toggles it. Find the
 * two positions with the Ramp Servo Check OpMode.
 */
public class RampSubsystem extends SubsystemBase {
  public static final String SERVO_NAME = "RampServo";
  /** The pulse the Hub sends for each, found with Ramp Servo Check (14 Nov, arm refitted on the spline). */
  public static double STOWED_US = 1430;
  public static double DEPLOYED_US = 490; // Just past the REV servo's rated 500-2500 us
  /**
   * The pulse that setPosition's 0 and 1 send. Wider than the SDK's default 600-2400, which can't reach deployed, and
   * than the servo's rated 500-2500.
   */
  public static final double PWM_LOWER_US = 400;
  public static final double PWM_UPPER_US = 2600;

  private final Servo servo; // Null until the robot configuration has RampServo
  private boolean commanded = false;
  private boolean deployed = false;
  private double written = Double.NaN;

  public RampSubsystem(HardwareMap hardwareMap) {
    servo = hardwareMap.tryGet(Servo.class, SERVO_NAME);
    widenRange(servo);
  }

  /** Sets the pulse range the positions are in, before anything is sent to servo. */
  public static void widenRange(Servo servo) {
    if (servo instanceof PwmControl) {
      ((PwmControl) servo).setPwmRange(new PwmControl.PwmRange(PWM_LOWER_US, PWM_UPPER_US));
    }
  }

  public void setDeployed(boolean deployed) {
    this.deployed = deployed;
    commanded = true;
  }

  public void toggle() {
    setDeployed(!deployed);
  }

  public boolean isDeployed() {
    return deployed;
  }

  /** False when the robot configuration has no RampServo: the ramp then does nothing. */
  public boolean isPresent() {
    return servo != null;
  }

  static double positionFor(boolean deployed) {
    return toPosition(deployed ? DEPLOYED_US : STOWED_US);
  }

  /** The setPosition value that sends this pulse, in {@link #PWM_LOWER_US}-{@link #PWM_UPPER_US}. */
  static double toPosition(double us) {
    return Math.max(0, Math.min(1, (us - PWM_LOWER_US) / (PWM_UPPER_US - PWM_LOWER_US)));
  }

  @Override
  public void periodic() {
    double position = positionFor(deployed);
    // Only on a change: every servo write is a trip over the hub link the loop waits on
    if (servo != null && commanded && position != written) {
      servo.setPosition(position);
      written = position;
    }
    Logger.recordOutput("Ramp/Present", servo != null);
    Logger.recordOutput("Ramp/Deployed", deployed);
    Logger.recordOutput("Ramp/Position", Double.isNaN(written) ? -1 : written);
  }
}
