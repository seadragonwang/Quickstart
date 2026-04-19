package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Prism.Color;
import org.firstinspires.ftc.teamcode.pedroPathing.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.pedroPathing.Prism.PrismAnimations;

/**
 * BallDetector — uses a REV 2M Distance Sensor to count balls passing through the intake/outtake,
 * and a goBILDA Prism LED driver to indicate how many balls the robot has.
 *
 * LED colors:
 *   0 balls → WHITE
 *   1 ball  → YELLOW
 *   2 balls → BLUE
 *   3 balls → GREEN (max)
 *
 * Usage:
 *   BallDetector detector = new BallDetector(hardwareMap);
 *   // In your loop:
 *   detector.update();
 *   int count = detector.getBallsLoaded();
 */
public class BallDetector {

    private final DistanceSensor distanceSensor; // REV 2M Distance Sensor
    private GoBildaPrismDriver prism;

    // Tune these thresholds using BallDetectorTuner
    // Uses hysteresis to prevent flickering: two thresholds instead of one
    private double detectDistanceMM = 105.0;   // Ball detected when distance drops BELOW this
    private double clearDistanceMM  = 120.0;   // Ball cleared when distance rises ABOVE this
    private long debounceTimeMs = 200;         // Minimum time between state changes (prevents noise)

    private boolean ballPresent = false;
    private int ballCount = 0;
    private final ElapsedTime debounceTimer = new ElapsedTime();
    private double lastDistanceMM = 999.0; // cached reading from last update()

    // For tracking balls loaded vs launched
    private int ballsLoaded = 0;
    private int ballsLaunched = 0;

    // Prism LED state
    private int lastDisplayedBallCount = -1; // tracks what's currently shown on LEDs

    // Colors for each ball count
    private static final Color COLOR_0_BALLS = Color.WHITE;
    private static final Color COLOR_1_BALL  = Color.YELLOW;
    private static final Color COLOR_2_BALLS = Color.BLUE;
    private static final Color COLOR_3_BALLS = Color.GREEN;

    /**
     * Constructor with color sensor only (no Prism LEDs)
     */
    public BallDetector(HardwareMap hardwareMap) {
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");
    }

    private void initPrism() {
        if (prism != null) {
            // Start with LEDs off (0 balls)
            PrismAnimations.Solid off = new PrismAnimations.Solid(COLOR_0_BALLS);
            prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, off);
        }
    }

    /**
     * Call this every loop iteration. Detects balls passing through the sensor.
     * A ball is only counted when it fully passes (appears then disappears).
     * A ball that stays in front of the sensor will NOT keep incrementing the count.
     */
    public void update() {
        double distance = distanceSensor.getDistance(DistanceUnit.MM);
        lastDistanceMM = distance;

        if (debounceTimer.milliseconds() < debounceTimeMs) return;

        // Rising edge: ball appears (distance drops below lower threshold)
        if (!ballPresent && distance < detectDistanceMM) {
            ballPresent = true;
            debounceTimer.reset();
        }

        // Falling edge: ball leaves (distance rises above higher threshold)
        if (ballPresent && distance > clearDistanceMM) {
            ballPresent = false;
            // Only count if we haven't reached max capacity (3 balls)
            if (getBallsLoaded() < 3) {
                ballCount++;
                ballsLoaded++;
                debounceTimer.reset();
                updatePrismLEDs();
            }
        }
    }

    /** Total balls detected since last reset */
    public int getBallCount() {
        return ballCount;
    }

    /** Number of balls currently loaded (loaded - launched), max 3 */
    public int getBallsLoaded() {
        return Math.min(3, Math.max(0, ballsLoaded - ballsLaunched));
    }

    /** Call this each time a ball is launched */
    public void ballLaunched() {
        ballsLaunched++;
        updatePrismLEDs();
    }

    /** Call this when a set of balls is launched (e.g., 3 balls) */
    public void ballsLaunched(int count) {
        ballsLaunched += count;
        updatePrismLEDs();
    }

    /** Returns true if a ball is currently in front of the sensor */
    public boolean isBallPresent() {
        return ballPresent;
    }

    /** Get current distance reading in mm (cached from last update() call — no extra I2C read) */
    public double getDistanceMM() {
        return lastDistanceMM;
    }

    /** Reset all counters */
    public void reset() {
        ballCount = 0;
        ballsLoaded = 0;
        ballsLaunched = 0;
        ballPresent = false;
        lastDisplayedBallCount = -1;
        updatePrismLEDs();
    }

    /** Reset just the loaded/launched counters */
    public void resetLoadedCount() {
        ballsLoaded = 0;
        ballsLaunched = 0;
        lastDisplayedBallCount = -1;
        updatePrismLEDs();
    }

    /** Set detection thresholds with hysteresis (in mm). Tune with BallDetectorTuner.
     *  detectMM: ball detected when distance drops below this (e.g., 35mm)
     *  clearMM:  ball cleared when distance rises above this (e.g., 50mm)
     */
    public void setDetectionDistance(double detectMM, double clearMM) {
        this.detectDistanceMM = detectMM;
        this.clearDistanceMM = clearMM;
    }

    /** Set detection threshold (single value, auto-sets clear = detect + 15mm) */
    public void setDetectionDistance(double distanceMM) {
        this.detectDistanceMM = distanceMM;
        this.clearDistanceMM = distanceMM + 15.0;
    }

    /** Set debounce time (in ms). Prevents double-counting fast-moving balls. */
    public void setDebounceTime(long timeMs) {
        this.debounceTimeMs = timeMs;
    }

    /**
     * Update Prism LEDs based on current ball count.
     * Only writes to I2C if the count has changed (to avoid spamming the bus).
     */
    private void updatePrismLEDs() {
        if (prism == null) return;

        int currentBalls = getBallsLoaded();
        if (currentBalls == lastDisplayedBallCount) return;

        lastDisplayedBallCount = currentBalls;

        Color color;
        switch (currentBalls) {
            case 1:
                color = COLOR_1_BALL;
                break;
            case 2:
                color = COLOR_2_BALLS;
                break;
            case 3:
                color = COLOR_3_BALLS;
                break;
            default:
                color = COLOR_0_BALLS;
                break;
        }

        PrismAnimations.Solid solid = new PrismAnimations.Solid(color);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, solid);
    }
}

