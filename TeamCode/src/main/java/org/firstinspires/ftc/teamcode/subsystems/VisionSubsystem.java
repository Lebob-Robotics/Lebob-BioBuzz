package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/**
 * One webcam looking for our alliance's Hive cell. BIOBUZZ cells move, so this is for aiming only.
 * Cluster names in the SDK library are "RED SCORING", "RED AUDIENCE", "BLUE AUDIENCE", "BLUE SCORING".
 * A missing or misconfigured webcam leaves the subsystem inert rather than killing the OpMode:
 * isAvailable() is false, hasTarget() stays false, and aim assist simply does nothing.
 */
public class VisionSubsystem extends SubsystemBase {
    private AprilTagProcessor processor;
    private VisionPortal portal;
    private Alliance alliance = Alliance.RED;
    private AprilTagClusterDetection target;

    public VisionSubsystem(HardwareMap hardwareMap) {
        try {
            processor = new AprilTagProcessor.Builder()
                    .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                    .build();
            portal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, Constants.WEBCAM))
                    .addProcessor(processor)
                    .build();
        } catch (RuntimeException e) {
            processor = null;
            portal = null;
        }
    }

    /** False when the webcam was missing or failed to open at construction. */
    public boolean isAvailable() {
        return portal != null;
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    public Alliance getAlliance() {
        return alliance;
    }

    /** Picks the best-seen cluster belonging to our alliance from the latest frame. */
    @Override
    public void periodic() {
        if (portal == null) return;
        AprilTagClusterDetection best = null;
        for (AprilTagDetection d : processor.getDetections()) {
            if (!(d instanceof AprilTagClusterDetection)) continue;
            AprilTagClusterDetection c = (AprilTagClusterDetection) d;
            if (!c.metadata.name.startsWith(alliance.name())) continue;
            if (best == null || c.percentClusterFound > best.percentClusterFound) best = c;
        }
        target = best;
    }

    public boolean hasTarget() {
        return target != null;
    }

    /** Horizontal angle to the cell opening, degrees, positive to the left. Only valid when hasTarget(). */
    public double getBearingDeg() {
        return target.ftcPose.bearing;
    }

    public String getTargetName() {
        return target == null ? "none" : target.metadata.name;
    }

    /** Turn off the Driver Station preview once the match starts to save CPU. */
    public void stopLiveView() {
        if (portal != null) portal.stopLiveView();
    }

    public void close() {
        if (portal != null) portal.close();
    }
}
