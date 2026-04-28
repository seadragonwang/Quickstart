package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Base class for all Autonomous OpModes. Subclasses only need to provide:
 *   - getStartPose()          — field starting position
 *   - getPidfCoefficients()   — launcher PIDF tuning (close vs far)
 *   - buildPaths()            — construct all PathChains using {@code follower}
 *   - autonomousPathUpdate()  — state-machine that sequences paths & actions
 */
public abstract class AutoBase extends OpMode {

    // --- Fields shared by every auto ---
    protected Follower follower;
    protected Launcher launcher;
    protected Timer pathTimer, actionTimer, opmodeTimer, waitTimer;
    protected int pathState;

    // --- Tuning knobs (subclasses may override in constructor or field initialiser) ---
    protected double launchTime = 1000;
    protected double grabTime   = 1700;
    protected double pickupSpeed = 0.9;
    protected double grabSpeed   = 0.6;

    // --- Abstract methods for subclass configuration ---

    /** Field starting pose (inches / radians). */
    protected abstract Pose getStartPose();

    /** PIDF coefficients for the launcher outtake motors. */
    protected abstract PIDFCoefficients getPidfCoefficients();

    /** Build all PathChains. Called once during {@code init()} after follower is created. */
    protected abstract void buildPaths();

    /** State-machine that sequences paths and launcher actions. Called every loop. */
    protected abstract void autonomousPathUpdate();

    // --- Shared implementation ---

    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
        actionTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer   = new Timer();
        waitTimer   = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        launcher = new Launcher(hardwareMap, getPidfCoefficients());

        buildPaths();
        follower.setStartingPose(getStartPose());
    }

    @Override
    public void init_loop() {
        // no-op — override in subclass if needed
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        launcher.update();

        // Telemetry shared across all autos
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("flywheel vel", launcher.getOuttakeVelocity());
        telemetry.addData("flywheel ready", launcher.isFlywheelReady());
        telemetry.addData("launcher state", launcher.getState());
        telemetry.update();
    }
}

