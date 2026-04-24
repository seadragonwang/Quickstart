package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Runs BallDetector.update() on a dedicated background thread so that
 * slow I2C distance-sensor reads don't stall the main teleop loop.
 *
 * Usage:
 *   BallDetectorThread thread = new BallDetectorThread(hardwareMap);
 *   thread.start();
 *   // ... in loop: thread.getBallDetector().getBallsLoaded()
 *   thread.stopThread();
 */
public class BallDetectorThread extends Thread {

    private final BallDetector ballDetector;
    private volatile boolean running = false;

    /** Poll interval in milliseconds — tune as needed (50 ms = ~20 Hz) */
    private static final long POLL_INTERVAL_MS = 50;

    public BallDetectorThread(HardwareMap hardwareMap) {
        this.ballDetector = new BallDetector(hardwareMap);
        setDaemon(true); // auto-stops if the main thread dies
        setName("BallDetectorThread");
    }

    @Override
    public void run() {
        running = true;
        while (running && !isInterrupted()) {
            ballDetector.update();
            try {
                Thread.sleep(POLL_INTERVAL_MS);
            } catch (InterruptedException e) {
                interrupt(); // restore interrupt flag
                break;
            }
        }
    }

    /** Gracefully stop the background thread. Call this at the end of the OpMode. */
    public void stopThread() {
        running = false;
        interrupt();
    }

    /** Access the underlying detector to read ball count, distance, etc. */
    public BallDetector getBallDetector() {
        return ballDetector;
    }
}

