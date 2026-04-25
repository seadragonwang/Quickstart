package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CONSTANTS;

import java.util.List;

/**
 * LimelightBallDetector — uses a Limelight 3A neural detector pipeline
 * to detect balls and return their horizontal (tx) and vertical (ty) angles.
 *
 * Usage:
 *   1. Call init() during hardware initialization
 *   2. Call update() each loop to poll the Limelight
 *   3. Use hasBallTarget(), getTx(), getTy(), getArea() to read results
 *   4. Call switchToAprilTagPipeline() / switchToBallPipeline() to share
 *      the Limelight with LimelightLocalizer
 *
 * Pipeline convention (configure in Limelight web UI):
 *   Pipeline 0 — AprilTag localization (used by LimelightLocalizer)
 *   Pipeline 1 — Neural ball detector (default for this class)
 */
public class LimelightBallDetector {

    private Limelight3A limelight;
    private Servo pivot;
    private boolean initialized = false;
    private String initError = "";

    /** Pipeline index for ball detection (neural network or color). */
    private int ballPipeline = 1;
    /** Pipeline index for AprilTag localization — restored when done. */
    private int aprilTagPipeline = 0;

    // Latest detection results
    private boolean hasTarget = false;
    private double tx = 0;   // horizontal angle to best ball target (degrees, + = right)
    private double ty = 0;   // vertical angle to best ball target (degrees, + = up)
    private double area = 0; // target area as % of image (larger = closer)
    private String lastStatus = "not updated";

    /**
     * Initialize the ball detector.
     * @param hardwareMap    the hardware map
     * @param ballPipeline   Limelight pipeline index configured for ball detection
     * @param aprilTagPipeline Limelight pipeline index for AprilTag (to restore when done)
     */
    public void init(HardwareMap hardwareMap, int ballPipeline, int aprilTagPipeline) {
        this.ballPipeline = ballPipeline;
        this.aprilTagPipeline = aprilTagPipeline;
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            pivot = hardwareMap.get(Servo.class, "pivot");
            // Start in AprilTag mode; switch to ball mode on demand
            limelight.pipelineSwitch(aprilTagPipeline);
            limelight.start();
            initialized = true;
            initError = "";
        } catch (Exception e) {
            initialized = false;
            initError = e.getClass().getSimpleName() + ": " + e.getMessage();
        }
    }

    /** Initialize with default pipelines: ball = 1, AprilTag = 0. */
    public void init(HardwareMap hardwareMap) {
        init(hardwareMap, 1, 0);
    }

    /**
     * Switch the Limelight to ball detection pipeline and tilt pivot to ball position.
     * Call this at the start of a pickup phase.
     */
    public void switchToBallPipeline() {
        if (!initialized || limelight == null) return;
        pivot.setPosition(CONSTANTS.LIMELIGHT_BALL_POS);
        limelight.pipelineSwitch(ballPipeline);
    }

    /**
     * Switch the Limelight back to AprilTag localization pipeline and restore pivot.
     * Call this when done with pickup.
     */
    public void switchToAprilTagPipeline() {
        if (!initialized || limelight == null) return;
        pivot.setPosition(CONSTANTS.LIMELIGHT_APRIL_TAG_POS);
        limelight.pipelineSwitch(aprilTagPipeline);
        // Clear stale ball results
        hasTarget = false;
        tx = 0;
        ty = 0;
        area = 0;
        lastStatus = "switched to AprilTag pipeline";
    }

    /**
     * Poll the Limelight for ball detections. Call each loop while in ball pipeline.
     * Picks the largest (closest) detected ball as the primary target.
     */
    public void update() {
        if (!initialized || limelight == null) {
            hasTarget = false;
            lastStatus = "not initialized: " + initError;
            return;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            hasTarget = false;
            lastStatus = result == null ? "result null" : "result invalid";
            return;
        }

        // Neural detector results
        List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
        if (detections == null || detections.isEmpty()) {
            hasTarget = false;
            lastStatus = "no detections";
            return;
        }

        // Pick the detection with the largest area (closest ball)
        LLResultTypes.DetectorResult best = null;
        double bestArea = -1;
        for (LLResultTypes.DetectorResult d : detections) {
            if (d.getTargetArea() > bestArea) {
                bestArea = d.getTargetArea();
                best = d;
            }
        }

        if (best == null) {
            hasTarget = false;
            lastStatus = "no valid detection";
            return;
        }

        hasTarget = true;
        tx = best.getTargetXDegrees();
        ty = best.getTargetYDegrees();
        area = best.getTargetArea();
        lastStatus = String.format("target tx=%.1f ty=%.1f area=%.2f", tx, ty, area);
    }

    /** @return true if a ball is currently detected */
    public boolean hasBallTarget() { return hasTarget; }

    /**
     * Horizontal angle to the best ball target in degrees.
     * Positive = ball is to the RIGHT of camera center.
     * Negative = ball is to the LEFT of camera center.
     */
    public double getTx() { return tx; }

    /**
     * Vertical angle to the best ball target in degrees.
     * Positive = ball is ABOVE camera center.
     */
    public double getTy() { return ty; }

    /** Target area as % of image (0–100). Larger = ball is closer. */
    public double getArea() { return area; }

    /** @return status string for telemetry */
    public String getStatus() { return lastStatus; }

    /** @return true if initialized successfully */
    public boolean isInitialized() { return initialized; }
}

