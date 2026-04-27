package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Prism.Color;
import org.firstinspires.ftc.teamcode.pedroPathing.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.pedroPathing.Prism.PrismAnimations;

/**
 * BallDetector — uses two REV 2M Distance Sensors (right + left) to count balls,
 * and a goBILDA Prism LED driver to indicate how many balls the robot has.
 *
 * LED colors:
 *   0 balls → WHITE  1 ball → YELLOW  2 balls → BLUE  3 balls → GREEN
 *
 * Ball is counted on ARRIVAL (rising edge) so the 3rd parked ball is always counted.
 * Both sensors must clear before a new ball can be detected (AND-NOT clear logic).
 */
public class BallDetector {

    private final DistanceSensor rightSensor;  // hardware name: "distanceSensor"
    private final DistanceSensor leftSensor;   // hardware name: "ball_detector"
    private GoBildaPrismDriver prism;

    // Right sensor thresholds (mm)
    private double rightDetectMM = 100.0;
    private double rightClearMM  = 125.0;

    // Left sensor thresholds (mm) — tune separately with BallDetectorTuner
    private double leftDetectMM  = 100.0;
    private double leftClearMM   = 12.0;

    private long debounceTimeMs = 200;

    private boolean rightBallPresent = false;
    private boolean leftBallPresent  = false;
    private boolean ballPresent      = false;

    private int ballCount    = 0;
    private int ballsLoaded  = 0;
    private int ballsLaunched = 0;

    private final ElapsedTime debounceTimer = new ElapsedTime();
    private double lastRightDistanceMM = 999.0;
    private double lastLeftDistanceMM  = 999.0;
    private int lastDisplayedBallCount = -1;

    private static final Color COLOR_0_BALLS = Color.WHITE;
    private static final Color COLOR_1_BALL  = Color.YELLOW;
    private static final Color COLOR_2_BALLS = Color.BLUE;
    private static final Color COLOR_3_BALLS = Color.GREEN;

    public BallDetector(HardwareMap hardwareMap) {
        rightSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        leftSensor  = hardwareMap.get(DistanceSensor.class, "ball_detector");
        prism       = hardwareMap.get(GoBildaPrismDriver.class, "prism");
    }

    /**
     * Call every loop. Ball counted on arrival (rising edge).
     * Cleared only when BOTH sensors see nothing (AND-NOT logic).
     */
    public void update() {
        if (getBallsLoaded() >= 3) return;

        double rightDist = rightSensor.getDistance(DistanceUnit.MM);
        double leftDist  = leftSensor.getDistance(DistanceUnit.MM);
        lastRightDistanceMM = rightDist;
        lastLeftDistanceMM  = leftDist;

        // Hysteresis per sensor
        if (!rightBallPresent && rightDist < rightDetectMM)       rightBallPresent = true;
        else if (rightBallPresent && rightDist > rightClearMM)    rightBallPresent = false;

        if (!leftBallPresent && leftDist < leftDetectMM)          leftBallPresent = true;
        else if (leftBallPresent && leftDist > leftClearMM)       leftBallPresent = false;

        if (debounceTimer.milliseconds() < debounceTimeMs) return;

        boolean newBallPresent = rightBallPresent || leftBallPresent;

        if (!ballPresent && newBallPresent) {
            ballPresent = true;
            if (getBallsLoaded() < 3) {
                ballCount++;
                ballsLoaded++;
                updatePrismLEDs();
            }
            debounceTimer.reset();
        } else if (ballPresent && !newBallPresent) {
            ballPresent = false;
            debounceTimer.reset();
        }
    }

    public int getBallCount()   { return ballCount; }
    public int getBallsLoaded() { return Math.min(3, Math.max(0, ballsLoaded - ballsLaunched)); }

    public void ballLaunched()           { ballsLaunched++;      updatePrismLEDs(); }
    public void ballsLaunched(int count) { ballsLaunched += count; updatePrismLEDs(); }

    public boolean isBallPresent()       { return ballPresent; }
    public boolean isRightBallPresent()  { return rightBallPresent; }
    public boolean isLeftBallPresent()   { return leftBallPresent; }

    public double getDistanceMM()      { return lastRightDistanceMM; }
    public double getRightDistanceMM() { return lastRightDistanceMM; }
    public double getLeftDistanceMM()  { return lastLeftDistanceMM; }

    public void reset() {
        ballCount = 0; ballsLoaded = 0; ballsLaunched = 0;
        ballPresent = false; rightBallPresent = false; leftBallPresent = false;
        lastDisplayedBallCount = -1;
        updatePrismLEDs();
    }

    public void resetLoadedCount() {
        ballsLoaded = 0; ballsLaunched = 0;
        ballPresent = false; rightBallPresent = false; leftBallPresent = false;
        debounceTimer.reset();
        lastDisplayedBallCount = -1;
        updatePrismLEDs();
    }

    /** Set right sensor thresholds (mm). */
    public void setDetectionDistance(double detectMM, double clearMM) {
        this.rightDetectMM = detectMM;
        this.rightClearMM  = clearMM;
    }

    /** Set left sensor thresholds (mm). */
    public void setLeftDetectionDistance(double detectMM, double clearMM) {
        this.leftDetectMM = detectMM;
        this.leftClearMM  = clearMM;
    }

    // Keep old name as alias for BallDetectorTuner compatibility
    public void setColorDetectionDistance(double detectMM, double clearMM) {
        setLeftDetectionDistance(detectMM, clearMM);
    }

    public void setDebounceTime(long timeMs) { this.debounceTimeMs = timeMs; }

    private void updatePrismLEDs() {
        if (prism == null) return;
        int n = getBallsLoaded();
        if (n == lastDisplayedBallCount) return;
        lastDisplayedBallCount = n;
        Color color;
        switch (n) {
            case 1:  color = COLOR_1_BALL;  break;
            case 2:  color = COLOR_2_BALLS; break;
            case 3:  color = COLOR_3_BALLS; break;
            default: color = COLOR_0_BALLS; break;
        }
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0,
                new PrismAnimations.Solid(color));
    }
}
