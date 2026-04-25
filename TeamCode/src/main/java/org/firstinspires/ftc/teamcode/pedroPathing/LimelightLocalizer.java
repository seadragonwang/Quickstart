package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.CONSTANTS;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * LimelightLocalizer — uses a Limelight 3A camera with MegaTag2 AprilTag localization
 * to periodically correct the robot's pose (X, Y) in the Pedro Pathing system.
 *
 * Uses the follower's heading (from Pinpoint IMU + odometry) as the heading prior
 * fed to MegaTag2, instead of a separate Control Hub IMU.
 *
 * Coordinate conversion:
 *   Limelight returns Pose3D in meters with FTC field origin at center.
 *   Pedro Pathing uses inches with origin at a field corner.
 *   Conversion: pedro = (limelight_meters * 39.3701) + 72
 *
 * Usage:
 *   1. Call init() during hardware initialization
 *   2. Call update(follower) every loop iteration
 *   3. The localizer will correct the follower's pose when a valid Limelight result is available
 */
public class LimelightLocalizer {

    private Limelight3A limelight;
    private boolean initialized = false;

    // Conversion: meters to inches
    private static final double METERS_TO_INCHES = 39.3701;
    // FTC field center offset: Pedro origin is at corner, FTC origin is at center
    // Field is 144 inches (3.6576m) per side, so offset is 72 inches
    private static final double FIELD_OFFSET_INCHES = 72.0;

    // Pipeline index for AprilTag localization (configure in Limelight web UI)
    private int aprilTagPipeline = 0;

    // Minimum number of AprilTags required for a valid pose update
    private int minTagCount = 1;

    // Maximum allowed latency in milliseconds before discarding result
    private double maxLatencyMs = 200;

    // Valid AprilTag IDs — if non-empty, only results containing exclusively these tags are accepted
    private Set<Integer> validTagIds = new HashSet<>();

    // Last valid pose for telemetry
    private Pose lastLimelightPose = null;
    private boolean lastResultValid = false;
    private double lastLatencyMs = 0;
    private int lastTagCount = 0;
    private Servo pivot;
    private double lastHeadingDeg = 0;
    private String lastRejectReason = "none";
    private String initError = "";
    private double lastRawX = 0;
    private double lastRawY = 0;
    private double lastYawDegrees = 0;
    private double lastPedroX = 0;
    private double lastPedroY = 0;
    // Tunable offsets (inches) applied after coordinate conversion — set these to correct systematic bias
    private double xOffsetInches = 0;
    private double yOffsetInches = 0;

    /**
     * Initialize the Limelight 3A.
     * @param hardwareMap the hardware map
     * @param limelightName the hardware name of the Limelight (e.g., "limelight")
     * @param pipeline the pipeline index for AprilTag detection
     */
    public void init(HardwareMap hardwareMap, String limelightName, int pipeline) {
        try {
            pivot = hardwareMap.get(Servo.class, "pivot");
            pivot.setPosition(CONSTANTS.LIMELIGHT_APRIL_TAG_POS);
            limelight = hardwareMap.get(Limelight3A.class, limelightName);
            aprilTagPipeline = pipeline;
            limelight.pipelineSwitch(aprilTagPipeline);
            limelight.start();

            initialized = true;
            initError = "";
        } catch (Exception e) {
            initialized = false;
            initError = e.getClass().getSimpleName() + ": " + e.getMessage();
        }
    }

    /**
     * Initialize with default device name and pipeline 0.
     * The startingHeadingDeg parameter is kept for API compatibility but is no longer used —
     * heading is now taken directly from the follower (Pinpoint).
     * @param hardwareMap the hardware map
     * @param startingHeadingDeg ignored (kept for backward compatibility)
     */
    public void init(HardwareMap hardwareMap, double startingHeadingDeg) {
        init(hardwareMap, "limelight", 0);
    }

    /**
     * Poll the Limelight and correct the follower's pose if a valid result is available.
     * Uses the follower's heading (from Pinpoint IMU + odometry) as the MegaTag2 prior.
     *
     * @param follower the Pedro Pathing follower to correct
     */
    public void update(Follower follower) {
        if (!initialized || limelight == null) {
            lastResultValid = false;
            lastRejectReason = "not initialized: " + initError;
            return;
        }

        // Track follower heading for telemetry
        double followerHeadingDeg = Math.toDegrees(follower.getPose().getHeading());
        if (followerHeadingDeg > 180) {
            followerHeadingDeg -= 360;
        }
        lastHeadingDeg = followerHeadingDeg;

        LLResult result = limelight.getLatestResult();

        if (result == null) {
            lastResultValid = false;
            lastRejectReason = "result is null";
            return;
        }

        if (!result.isValid()) {
            lastResultValid = false;
            lastRejectReason = "result not valid";
            return;
        }

        // Check latency
        double totalLatency = result.getCaptureLatency() + result.getTargetingLatency();
        lastLatencyMs = totalLatency;

        if (totalLatency > maxLatencyMs) {
            lastResultValid = false;
            lastRejectReason = "latency too high: " + totalLatency;
            return;
        }

        // Filter by valid tag IDs — reject if any detected tag is not in the valid set
        if (!validTagIds.isEmpty()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials.isEmpty()) {
                lastResultValid = false;
                lastRejectReason = "no fiducial results to validate tag IDs";
                return;
            }
            for (LLResultTypes.FiducialResult fr : fiducials) {
                if (!validTagIds.contains(fr.getFiducialId())) {
                    lastResultValid = false;
                    lastRejectReason = "invalid tag ID: " + fr.getFiducialId();
                    return;
                }
            }
        }

        // Use regular botpose (MegaTag1) — works reliably without IMU fusion.
        // MT2 requires a precise heading prior and returns (0,0,0) when it can't solve,
        // and with only 1 visible tag it doesn't add meaningful accuracy.
        Pose3D botpose = result.getBotpose();
        if (botpose == null) {
            lastResultValid = false;
            lastRejectReason = "botpose is null";
            return;
        }

        // Track tag count for telemetry
        lastTagCount = result.getFiducialResults().size();

        // Convert Limelight coordinates (meters, FTC field center origin)
        // to Pedro Pathing coordinates (inches, corner origin)
        Position pos = botpose.getPosition();
        double limelightXMeters = pos.x;
        double limelightYMeters = pos.y;

        // Store raw values for telemetry debugging
        lastRawX = limelightXMeters;
        lastRawY = limelightYMeters;

        // Reject (0,0) botpose — Limelight returns this when it can't solve
        if (Math.abs(limelightXMeters) < 0.01 && Math.abs(limelightYMeters) < 0.01) {
            lastResultValid = false;
            lastRejectReason = "botpose is (0,0) — unsolved";
            return;
        }

        // LL +X = Pedro -Y, LL +Y = Pedro +X
        double pedroX = (limelightYMeters * METERS_TO_INCHES) + FIELD_OFFSET_INCHES + xOffsetInches;
        double pedroY = (-limelightXMeters * METERS_TO_INCHES) + FIELD_OFFSET_INCHES + yOffsetInches;

        // Store computed Pedro coordinates for telemetry/debugging
        lastPedroX = pedroX;
        lastPedroY = pedroY;

        // Sanity check: reject if position is outside the field
        if (pedroX < -12 || pedroX > 156 || pedroY < -12 || pedroY > 156) {
            lastResultValid = false;
            lastRejectReason = "position out of bounds: " + pedroX + ", " + pedroY;
            return;
        }

        // Convert Limelight heading to Pedro heading
        // LL yaw 0° = facing -Y in Pedro = 270° in Pedro. Both CCW-positive.
        // Pedro heading = yawDegrees + 270
        YawPitchRollAngles angles = botpose.getOrientation();
        double yawDegrees = angles.getYaw(AngleUnit.DEGREES);
        lastYawDegrees = yawDegrees;
        double headingRadians = Math.toRadians(yawDegrees + 270);

        // Store for telemetry
        lastLimelightPose = new Pose(pedroX, pedroY, headingRadians);
        lastResultValid = true;
        lastRejectReason = "accepted";

        // Correct the follower's pose using offsets so the correction persists
        // when the Limelight loses sight of the AprilTag.
        // setPose() resets the localizer directly, but the Pinpoint hardware may
        // overwrite it on the next update. setCurrentPoseWithOffset() stores the
        // difference as a persistent offset that is applied on every future read.
        follower.poseTracker.setCurrentPoseWithOffset(lastLimelightPose);
    }

    /** Set the minimum number of AprilTags required for a valid pose update. */
    public void setMinTagCount(int count) {
        this.minTagCount = count;
    }

    /** Set the maximum allowed latency before discarding a result. */
    public void setMaxLatencyMs(double ms) {
        this.maxLatencyMs = ms;
    }

    /** Set the valid AprilTag IDs. Only results containing exclusively these tags are accepted.
     *  Pass no arguments or an empty array to accept all tags. */
    public void setValidTagIds(int... ids) {
        validTagIds.clear();
        for (int id : ids) {
            validTagIds.add(id);
        }
    }

    /** Switch to a different pipeline. */
    public void setPipeline(int pipeline) {
        if (initialized && limelight != null) {
            aprilTagPipeline = pipeline;
            limelight.pipelineSwitch(pipeline);
        }
    }

    /** @return true if the Limelight was initialized successfully */
    public boolean isInitialized() {
        return initialized;
    }

    /** @return true if the last update produced a valid pose correction */
    public boolean isLastResultValid() {
        return lastResultValid;
    }

    /** @return the last valid pose from the Limelight (in Pedro coordinates), or null */
    public Pose getLastPose() {
        return lastLimelightPose;
    }

    /** @return the total latency of the last result in milliseconds */
    public double getLastLatencyMs() {
        return lastLatencyMs;
    }

    /** @return the number of tags detected in the last result */
    public int getLastTagCount() {
        return lastTagCount;
    }

    /** @return the heading (degrees) sent to Limelight as MegaTag2 prior */
    public double getLastImuHeadingDeg() {
        return lastHeadingDeg;
    }

    /** @return the reason the last result was rejected, or "accepted" */
    public String getLastRejectReason() {
        return lastRejectReason;
    }

    /** @return raw Limelight X in meters (for debugging) */
    public double getLastRawX() {
        return lastRawX;
    }

    /** @return raw Limelight yaw in degrees (for debugging) */
    public double getLastYawDegrees() {
        return lastYawDegrees;
    }

    /** @return raw Limelight Y in meters (for debugging) */
    public double getLastRawY() {
        return lastRawY;
    }

    /**
     * Set a fixed X/Y offset (inches) applied after coordinate conversion.
     * Use this to correct systematic position bias: measure where the robot actually
     * is, compare to what the LL reports, and pass the difference here.
     */
    public void setPoseOffsetInches(double xOffset, double yOffset) {
        this.xOffsetInches = xOffset;
        this.yOffsetInches = yOffset;
    }

    /** @return last computed Pedro X (inches) before pose update — useful for debugging */
    public double getLastPedroX() { return lastPedroX; }

    /** @return last computed Pedro Y (inches) before pose update — useful for debugging */
    public double getLastPedroY() { return lastPedroY; }

    /** Stop the Limelight polling. Call when OpMode ends. */
    public void stop() {
        if (initialized && limelight != null) {
            limelight.stop();
        }
    }
}

