package org.firstinspires.ftc.teamcode;

/**
 * Holds the up-facing Cell's opening centre in field coordinates.
 *
 * Camera frames arrive late, so each detection is placed using the robot pose recorded nearest the
 * frame's timestamp, not the current pose. Between detections the stored point stands and the Pinpoint
 * carries the aim. The Cells move when the Hive tips, so nothing here is a field constant. Pure maths.
 */
public final class TargetTracker {
    private final long[] poseNanos;
    private final double[] poseX;
    private final double[] poseY;
    private final double[] poseHeading;
    private int next;
    private int count;

    private final double camForwardM;
    private final double camLeftM;
    private final double camYawRad;
    private final double camPitchRad;
    private final long expiryNanos;

    private double targetX;
    private double targetY;
    private long seenNanos;
    private boolean seen;

    /**
     * @param capacity    poses kept; 50 covers a second at a 50 Hz loop
     * @param camForwardM camera lens forward of the robot centre
     * @param camLeftM    camera lens left of the robot centre
     * @param camYawRad   camera turned left of robot forward
     * @param camPitchRad camera tilted up from level
     * @param expiryS     how long a detection stays usable
     */
    public TargetTracker(int capacity, double camForwardM, double camLeftM, double camYawRad, double camPitchRad, double expiryS) {
        poseNanos = new long[capacity];
        poseX = new double[capacity];
        poseY = new double[capacity];
        poseHeading = new double[capacity];
        this.camForwardM = camForwardM;
        this.camLeftM = camLeftM;
        this.camYawRad = camYawRad;
        this.camPitchRad = camPitchRad;
        this.expiryNanos = (long) (expiryS * 1e9);
    }

    /** Call once per loop with the Pinpoint pose. */
    public void recordPose(long nanos, double x, double y, double headingRad) {
        poseNanos[next] = nanos;
        poseX[next] = x;
        poseY[next] = y;
        poseHeading[next] = headingRad;
        next = (next + 1) % poseNanos.length;
        if (count < poseNanos.length) count++;
    }

    /**
     * Places the Cell from a detection. Camera frame per the SDK's ftcPose: x right, y forward along the
     * lens axis, z up. Returns false if no pose has been recorded yet.
     */
    public boolean update(long frameNanos, double camXRightM, double camYForwardM, double camZUpM) {
        if (count == 0) return false;
        int p = nearestPose(frameNanos);

        // Camera frame to a level frame: undo the pitch, then x-right becomes y-left negative.
        double forward = camYForwardM * Math.cos(camPitchRad) - camZUpM * Math.sin(camPitchRad);
        double left = -camXRightM;
        // Level camera frame to robot frame: yaw, then lens offset.
        double rx = camForwardM + forward * Math.cos(camYawRad) - left * Math.sin(camYawRad);
        double ry = camLeftM + forward * Math.sin(camYawRad) + left * Math.cos(camYawRad);
        // Robot frame to field frame with the pose at the frame time.
        double h = poseHeading[p];
        targetX = poseX[p] + rx * Math.cos(h) - ry * Math.sin(h);
        targetY = poseY[p] + rx * Math.sin(h) + ry * Math.cos(h);
        seenNanos = frameNanos;
        seen = true;
        return true;
    }

    private int nearestPose(long nanos) {
        int best = 0;
        long bestGap = Long.MAX_VALUE;
        for (int i = 0; i < count; i++) {
            long gap = Math.abs(poseNanos[i] - nanos);
            if (gap < bestGap) {
                bestGap = gap;
                best = i;
            }
        }
        return best;
    }

    public boolean hasTarget(long nowNanos) {
        return seen && nowNanos - seenNanos <= expiryNanos;
    }

    public double getX() {
        return targetX;
    }

    public double getY() {
        return targetY;
    }

    public double ageS(long nowNanos) {
        return seen ? (nowNanos - seenNanos) / 1e9 : Double.POSITIVE_INFINITY;
    }

    /** Forgets the target and every recorded pose. Call whenever the field frame moves, e.g. a heading reset. */
    public void clear() {
        seen = false;
        count = 0;
        next = 0;
    }
}
