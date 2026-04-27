package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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

    // Right sensor (REV 2M) thresholds
    private double detectThreshold = 35.0;
    private double clearThreshold  = 50.0;

    // Left sensor (Color Sensor V3 proximity) thresholds
    private double colorDetectThreshold = 60.0;
    private double colorClearThreshold  = 75.0;

    // Raw left sensor reference for live telemetry
    private DistanceSensor leftSensorRaw = null;
    private boolean leftSensorAvailable = false;

    private boolean dpadUpPrev = false, dpadDownPrev = false;
    private boolean dpadLeftPrev = false, dpadRightPrev = false;
    private boolean aPrev = false;
    // gamepad2 for color sensor thresholds
    private boolean g2dpadUpPrev = false, g2dpadDownPrev = false;
    private boolean g2dpadLeftPrev = false, g2dpadRightPrev = false;

    private double minDistance = Double.MAX_VALUE, maxDistance = 0;
    private double colorMinDist = Double.MAX_VALUE, colorMaxDist = 0;

    @Override
    public void init() {
        ballDetector = new BallDetector(hardwareMap);
        ballDetector.setDetectionDistance(detectThreshold, clearThreshold);
        ballDetector.setColorDetectionDistance(colorDetectThreshold, colorClearThreshold);

        // Get direct reference to left sensor for raw telemetry
        try {
            leftSensorRaw = hardwareMap.get(DistanceSensor.class, "ball_detector");
            leftSensorAvailable = true;
        } catch (Exception e) {
            leftSensorAvailable = false;
        }

        telemetry.addLine("=== BALL DETECTOR TUNER ===");
        telemetry.addLine("Gamepad1: tune RIGHT sensor  Gamepad2: tune LEFT sensor");
        telemetry.addLine("D-pad Up/Down: detect threshold  D-pad Right/Left: clear threshold");
        telemetry.addLine("A (gp1): reset counters & min/max");
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- Gamepad1: right sensor thresholds ---
        if (gamepad1.dpad_up    && !dpadUpPrev)   { detectThreshold += 5; ballDetector.setDetectionDistance(detectThreshold, clearThreshold); }
        if (gamepad1.dpad_down  && !dpadDownPrev)  { detectThreshold = Math.max(5, detectThreshold - 5); ballDetector.setDetectionDistance(detectThreshold, clearThreshold); }
        if (gamepad1.dpad_right && !dpadRightPrev) { clearThreshold += 5; ballDetector.setDetectionDistance(detectThreshold, clearThreshold); }
        if (gamepad1.dpad_left  && !dpadLeftPrev)  { clearThreshold = Math.max(detectThreshold + 5, clearThreshold - 5); ballDetector.setDetectionDistance(detectThreshold, clearThreshold); }
        dpadUpPrev = gamepad1.dpad_up; dpadDownPrev = gamepad1.dpad_down;
        dpadLeftPrev = gamepad1.dpad_left; dpadRightPrev = gamepad1.dpad_right;

        // --- Gamepad2: left color sensor thresholds ---
        if (gamepad2.dpad_up    && !g2dpadUpPrev)   { colorDetectThreshold += 5; ballDetector.setColorDetectionDistance(colorDetectThreshold, colorClearThreshold); }
        if (gamepad2.dpad_down  && !g2dpadDownPrev)  { colorDetectThreshold = Math.max(5, colorDetectThreshold - 5); ballDetector.setColorDetectionDistance(colorDetectThreshold, colorClearThreshold); }
        if (gamepad2.dpad_right && !g2dpadRightPrev) { colorClearThreshold += 5; ballDetector.setColorDetectionDistance(colorDetectThreshold, colorClearThreshold); }
        if (gamepad2.dpad_left  && !g2dpadLeftPrev)  { colorClearThreshold = Math.max(colorDetectThreshold + 5, colorClearThreshold - 5); ballDetector.setColorDetectionDistance(colorDetectThreshold, colorClearThreshold); }
        g2dpadUpPrev = gamepad2.dpad_up; g2dpadDownPrev = gamepad2.dpad_down;
        g2dpadLeftPrev = gamepad2.dpad_left; g2dpadRightPrev = gamepad2.dpad_right;

        // Reset
        if (gamepad1.a && !aPrev) {
            ballDetector.reset();
            minDistance = Double.MAX_VALUE; maxDistance = 0;
            colorMinDist = Double.MAX_VALUE; colorMaxDist = 0;
        }
        aPrev = gamepad1.a;

        ballDetector.update();

        double distMM = ballDetector.getDistanceMM();
        if (distMM < 2000) { minDistance = Math.min(minDistance, distMM); maxDistance = Math.max(maxDistance, distMM); }

        double colorDistMM = leftSensorAvailable ? leftSensorRaw.getDistance(DistanceUnit.MM) : -1;
        if (leftSensorAvailable && colorDistMM < 2000) { colorMinDist = Math.min(colorMinDist, colorDistMM); colorMaxDist = Math.max(colorMaxDist, colorDistMM); }

        // --- Telemetry ---
        telemetry.addLine("=== RIGHT SENSOR (REV 2M) — Gamepad1 ===");
        telemetry.addData("Distance (mm)",      String.format("%.1f", distMM));
        telemetry.addData("Min / Max (mm)",     String.format("%.1f / %.1f", minDistance == Double.MAX_VALUE ? 0 : minDistance, maxDistance));
        telemetry.addData("Detect below (mm)",  String.format("%.1f", detectThreshold));
        telemetry.addData("Clear above  (mm)",  String.format("%.1f", clearThreshold));
        telemetry.addData("rightBall?",          ballDetector.isRightBallPresent() ? "YES" : "no");
        telemetry.addLine("");

        telemetry.addLine("=== LEFT SENSOR (REV 2M) — Gamepad2 ===");
        telemetry.addData("Distance (mm)",      String.format("%.1f", colorDistMM));
        telemetry.addData("Min / Max (mm)",     String.format("%.1f / %.1f", colorMinDist == Double.MAX_VALUE ? 0 : colorMinDist, colorMaxDist));
        telemetry.addData("Detect below (mm)",  String.format("%.1f", colorDetectThreshold));
        telemetry.addData("Clear above  (mm)",  String.format("%.1f", colorClearThreshold));
        telemetry.addData("leftBall?",           ballDetector.isLeftBallPresent() ? "YES" : "no");
        telemetry.addLine("");

        telemetry.addLine("=== COMBINED ===");
        telemetry.addData("Ball Present?",  ballDetector.isBallPresent() ? "YES <<<" : "no");
        telemetry.addData("Ball Count",     ballDetector.getBallCount());
        telemetry.addData("Balls Loaded",   ballDetector.getBallsLoaded());
        telemetry.addLine("A(gp1)=reset  DU/DD=detect  DR/DL=clear");
        telemetry.update();
    }
}
