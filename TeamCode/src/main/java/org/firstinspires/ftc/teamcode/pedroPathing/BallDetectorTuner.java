package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Ball Detector Tuner
 *
 * Use this OpMode to find the right distance threshold for ball detection.
 *
 * HOW TO USE:
 * 1. Run this OpMode
 * 2. With NO ball in front of the sensor, note the distance reading (should be high, e.g., 80+ mm)
 * 3. Place a ball in front of the sensor, note the distance reading (should be low, e.g., 10-30 mm)
 * 4. Set the detection threshold in BallDetector to a value between these two readings
 * 5. Press A to reset the ball counter
 * 6. Feed balls through and verify the counter increments correctly
 *
 * Use D-pad up/down to adjust the detection threshold live.
 */
@TeleOp(name = "Ball Detector Tuner", group = "Tuning")
public class BallDetectorTuner extends OpMode {
    private BallDetector ballDetector;

    private double detectThreshold = 35.0; // ball detected below this
    private double clearThreshold  = 50.0; // ball cleared above this
    private boolean dpadUpPrev = false;
    private boolean dpadDownPrev = false;
    private boolean dpadLeftPrev = false;
    private boolean dpadRightPrev = false;
    private boolean aPrev = false;

    // Track min/max distance for calibration
    private double minDistance = Double.MAX_VALUE;
    private double maxDistance = 0;

    @Override
    public void init() {
        ballDetector = new BallDetector(hardwareMap);
        ballDetector.setDetectionDistance(detectThreshold, clearThreshold);

        telemetry.addLine("=== BALL DETECTOR TUNER ===");
        telemetry.addLine("D-pad Up/Down: adjust detect threshold");
        telemetry.addLine("D-pad Left/Right: adjust clear threshold");
        telemetry.addLine("A: reset counter & min/max");
        telemetry.addLine("Pass balls in front of sensor to test");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Adjust detect threshold with D-pad up/down
        if (gamepad1.dpad_up && !dpadUpPrev) {
            detectThreshold += 5.0;
            ballDetector.setDetectionDistance(detectThreshold, clearThreshold);
        }
        if (gamepad1.dpad_down && !dpadDownPrev) {
            detectThreshold -= 5.0;
            if (detectThreshold < 5.0) detectThreshold = 5.0;
            ballDetector.setDetectionDistance(detectThreshold, clearThreshold);
        }
        // Adjust clear threshold with D-pad left/right
        if (gamepad1.dpad_right && !dpadRightPrev) {
            clearThreshold += 5.0;
            ballDetector.setDetectionDistance(detectThreshold, clearThreshold);
        }
        if (gamepad1.dpad_left && !dpadLeftPrev) {
            clearThreshold -= 5.0;
            if (clearThreshold < detectThreshold + 5.0) clearThreshold = detectThreshold + 5.0;
            ballDetector.setDetectionDistance(detectThreshold, clearThreshold);
        }
        dpadUpPrev = gamepad1.dpad_up;
        dpadDownPrev = gamepad1.dpad_down;
        dpadLeftPrev = gamepad1.dpad_left;
        dpadRightPrev = gamepad1.dpad_right;

        // Reset on A press
        if (gamepad1.a && !aPrev) {
            ballDetector.reset();
            minDistance = Double.MAX_VALUE;
            maxDistance = 0;
        }
        aPrev = gamepad1.a;

        // Update detector
        ballDetector.update();

        // Get raw distance
        double distanceMM = ballDetector.getDistanceMM();

        // Track min/max
        if (distanceMM < 2000) { // ignore out-of-range readings
            if (distanceMM < minDistance) minDistance = distanceMM;
            if (distanceMM > maxDistance) maxDistance = distanceMM;
        }

        // Display
        telemetry.addLine("=== BALL DETECTOR TUNER (REV 2M) ===");
        telemetry.addLine("");

        telemetry.addLine("--- DISTANCE ---");
        telemetry.addData("Distance (mm)", String.format("%.1f", distanceMM));
        telemetry.addData("Min Distance (mm)", String.format("%.1f", minDistance));
        telemetry.addData("Max Distance (mm)", String.format("%.1f", maxDistance));
        telemetry.addLine("");

        telemetry.addLine("--- THRESHOLDS (hysteresis) ---");
        telemetry.addData("Detect below (mm)", String.format("%.1f", detectThreshold));
        telemetry.addData("Clear above (mm)", String.format("%.1f", clearThreshold));
        telemetry.addLine("");

        telemetry.addLine("--- DETECTION ---");
        telemetry.addData("Ball Present?", ballDetector.isBallPresent() ? "YES <<<" : "no");
        telemetry.addData("Ball Count", ballDetector.getBallCount());
        telemetry.addData("Balls Loaded", ballDetector.getBallsLoaded());
        telemetry.addLine("");

        telemetry.addLine("--- CONTROLS ---");
        telemetry.addLine("D-pad Up/Down: detect threshold (+/-5mm)");
        telemetry.addLine("D-pad Right/Left: clear threshold (+/-5mm)");
        telemetry.addLine("A: reset counter & min/max");

        telemetry.update();
    }
}
