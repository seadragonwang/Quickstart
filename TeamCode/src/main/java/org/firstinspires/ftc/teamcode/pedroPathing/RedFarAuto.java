package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//@Disabled
@Autonomous(name = "Red Far Auto", group = "OrcaRobotics")
public class RedFarAuto extends AutoBase {

    { grabTime = 1400; pickupSpeed = 0.9; }

    private LimelightBallDetector ballDetector;

    /**
     * Inches-per-degree horizontal conversion for pickup path adjustment.
     * Robot heading is 0° (facing +X), so:
     *   tx > 0 (ball right of camera) → +Y field direction → increase Y
     *   tx < 0 (ball left of camera)  → -Y field direction → decrease Y
     *   yAdjust = +tx * BALL_TX_TO_INCHES  (opposite sign from BlueFarAuto)
     */
    private static final double BALL_TX_TO_INCHES = 0.6;
    private static final double BALL_TX_THRESHOLD = 3.0;
    private static final double PICKUP_Y_MIN = 4.0;
    private static final double PICKUP_Y_MAX = 44.0;

    // Mirrored from BlueFarAuto: X -> 144-X, Y same, heading -> 180°-heading
    private final Pose startPose = new Pose(88.2, 7.5, Math.toRadians(0));
    private final Pose pickup1PoseStart = new Pose(102, 32, Math.toRadians(0));
    private final Pose pickup1PoseEnd = new Pose(130, 32, Math.toRadians(0));
    private final Pose scorePose = new Pose(84, 20, Math.toRadians(0)); // Mirrored: 144-64=80, inside far launch zone
    private final Pose pickup2PoseStart = new Pose(128, 10, Math.toRadians(0));
    private final Pose pickup2PoseEnd = new Pose(131, 10, Math.toRadians(0));
    private final Pose leavePose = new Pose(100, 25, Math.toRadians(0));

    private PathChain
            score1,
            pickup1,
            pickup1Path,
            score2,
            pickup2,
            pickup2Path,
            score3,
            pickup3,
            pickup3Path,
            score4,
            pickup4,
            pickup4Path,
            score5,
            pickup5,
            pickup5Path,
            score6,
            leave;

    @Override
    protected Pose getStartPose() {
        return startPose;
    }

    @Override
    public void start() {
        // Start flywheel early so it's at full speed before first shot
        launcher.setState(Launcher.LauncherState.START_LAUNCHING_RED_FAR);
        super.start();
    }

    @Override
    protected PIDFCoefficients getPidfCoefficients() {
        return Constants.farPidfCoefficients;
    }

    @Override
    public void init() {
        super.init();
        ballDetector = new LimelightBallDetector();
        ballDetector.init(hardwareMap);
        ballDetector.switchToBallPipeline();
    }

    @Override
    protected void buildPaths() {
        // score1: Move from start to score position, spin up flywheel mid-path
        score1 = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, new Pose(88, 14), scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.0, () -> launcher.setState(Launcher.LauncherState.START_LAUNCHING_RED_FAR))
                .build();

        // pickup1: From score position to pickup, switch to pickup mode mid-path
        pickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(88, 26), pickup1PoseStart))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1PoseStart.getHeading())
                .addParametricCallback(0.2, () -> launcher.setState(Launcher.LauncherState.PICKUP))
                .build();

        pickup1Path = follower.pathBuilder()
                .addPath(new BezierLine(pickup1PoseStart, pickup1PoseEnd))
                .build();

        // score2: Prep launcher from start of path
        score2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup1PoseEnd, new Pose(96, 38), scorePose))
                .setLinearHeadingInterpolation(pickup1PoseEnd.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.0, () -> launcher.setState(Launcher.LauncherState.START_LAUNCHING_RED_FAR))
                .build();

        // pickup2: Switch to pickup mode mid-path
        pickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(110, 20), new Pose(128, 16), pickup2PoseStart))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2PoseStart.getHeading())
                .addParametricCallback(0.2, () -> launcher.setState(Launcher.LauncherState.PICKUP))
                .build();

        pickup2Path = follower.pathBuilder()
                .addPath(new BezierLine(pickup2PoseStart, pickup2PoseEnd))
                .build();

        // score3
        score3 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2PoseEnd, new Pose(115, 21), scorePose))
                .setLinearHeadingInterpolation(pickup2PoseEnd.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.0, () -> launcher.setState(Launcher.LauncherState.START_LAUNCHING_RED_FAR))
                .build();

        pickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(110, 20), new Pose(128, 16), pickup2PoseStart))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2PoseStart.getHeading())
                .addParametricCallback(0.2, () -> launcher.setState(Launcher.LauncherState.PICKUP))
                .build();

        pickup3Path = follower.pathBuilder()
                .addPath(new BezierLine(pickup2PoseStart, pickup2PoseEnd))
                .build();

        score4 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2PoseEnd, new Pose(115, 21), scorePose))
                .setLinearHeadingInterpolation(pickup2PoseEnd.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.0, () -> launcher.setState(Launcher.LauncherState.START_LAUNCHING_RED_FAR))
                .build();

        pickup4 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(110, 20), new Pose(128, 16), pickup2PoseStart))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2PoseStart.getHeading())
                .addParametricCallback(0.2, () -> launcher.setState(Launcher.LauncherState.PICKUP))
                .build();

        pickup4Path = follower.pathBuilder()
                .addPath(new BezierLine(pickup2PoseStart, pickup2PoseEnd))
                .build();

        score5 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2PoseEnd, new Pose(115, 21), scorePose))
                .setLinearHeadingInterpolation(pickup2PoseEnd.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.0, () -> launcher.setState(Launcher.LauncherState.START_LAUNCHING_RED_FAR))
                .build();

        pickup5 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(110, 20), new Pose(128, 16), pickup2PoseStart))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2PoseStart.getHeading())
                .addParametricCallback(0.2, () -> launcher.setState(Launcher.LauncherState.PICKUP))
                .build();

        pickup5Path = follower.pathBuilder()
                .addPath(new BezierLine(pickup2PoseStart, pickup2PoseEnd))
                .build();

        score6 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2PoseEnd, new Pose(115, 21), scorePose))
                .setLinearHeadingInterpolation(pickup2PoseEnd.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.0, () -> launcher.setState(Launcher.LauncherState.START_LAUNCHING_RED_FAR))
                .build();

        leave = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(90, 21), leavePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), leavePose.getHeading())
                .build();
    }

    @Override
    public void loop() {
        super.loop();
        telemetry.addData("Ball target", ballDetector.hasBallTarget());
        telemetry.addData("Ball tx", ballDetector.getTx());
        telemetry.addData("Ball ty", ballDetector.getTy());
        telemetry.addData("Ball area", ballDetector.getArea());
        telemetry.addData("Ball status", ballDetector.getStatus());
    }

    /**
     * Compute a pickup end pose shifted toward the detected ball.
     * Robot heading is 0° (facing +X), so:
     *   tx > 0 → ball to robot's right → +Y field direction → increase Y
     *   tx < 0 → ball to robot's left  → -Y field direction → decrease Y
     */
    private Pose getAdjustedPickupEnd(Pose defaultEnd) {
        if (!ballDetector.hasBallTarget()) return defaultEnd;
        double tx = ballDetector.getTx();
        if (Math.abs(tx) < BALL_TX_THRESHOLD) return defaultEnd;
        // +tx → +Y for heading 0° (opposite of BlueFarAuto which uses heading 180°)
        double yAdjust = tx * BALL_TX_TO_INCHES;
        double newY = com.qualcomm.robotcore.util.Range.clip(
                defaultEnd.getY() + yAdjust, PICKUP_Y_MIN, PICKUP_Y_MAX);
        return new Pose(defaultEnd.getX(), newY, defaultEnd.getHeading());
    }

    /**
     * Build a pickup path from the given start aimed at the detected ball (or defaultEnd
     * if no ball is visible).
     */
    private PathChain buildAimedPickupPath(Pose start, Pose defaultEnd) {
        ballDetector.update();
        Pose aimed = getAdjustedPickupEnd(defaultEnd);
        return follower.pathBuilder()
                .addPath(new BezierLine(start, aimed))
                .build();
    }

    @Override
    protected void autonomousPathUpdate() {
        switch (pathState) {
            // === SCORE 1 (move to score position, then shoot preloaded balls) ===
            case 0:
                launcher.setState(Launcher.LauncherState.START_LAUNCHING_RED_FAR);
                follower.followPath(score1, 1, true);
                setPathState(1);
                break;
            case 1: // Arrived at score position → wait for flywheel ready, then launch
                if (!follower.isBusy() && launcher.isFlywheelReady()) {
                    launcher.setState(Launcher.LauncherState.LAUNCH);
                    setPathState(2);
                }
                break;
            case 2: // Wait for all balls to launch
                if (actionTimer.getElapsedTime() > launchTime) {
                    follower.followPath(pickup1, 1, true);
                    setPathState(3);
                }
                break;

            // === PICKUP 1 ===
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(pickup1Path, pickupSpeed, true);
                    setPathState(4);
                }
                break;
            case 4:
                // Spike-mark row — no aiming needed, balls are in a fixed row
                if (!follower.isBusy()) {
                    follower.followPath(score2, 1, true);
                    setPathState(5);
                }
                break;

            // === SCORE 2 ===
            case 5:
                if (!follower.isBusy() && launcher.isFlywheelReady()) {
                    launcher.setState(Launcher.LauncherState.LAUNCH);
                    setPathState(6);
                }
                break;
            case 6:
                if (actionTimer.getElapsedTime() > launchTime) {
                    follower.followPath(pickup2, 1, true);
                    setPathState(7);
                }
                break;

            // === PICKUP 2 ===
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(buildAimedPickupPath(pickup2PoseStart, pickup2PoseEnd), pickupSpeed, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy() && actionTimer.getElapsedTime() > grabTime) {
                    follower.followPath(score3, 1, true);
                    setPathState(9);
                }
                break;

            // === SCORE 3 ===
            case 9:
                if (!follower.isBusy() && launcher.isFlywheelReady()) {
                    launcher.setState(Launcher.LauncherState.LAUNCH);
                    setPathState(10);
                }
                break;
            case 10:
                if (actionTimer.getElapsedTime() > launchTime) {
                    follower.followPath(pickup3, 1, true);
                    setPathState(11);
                }
                break;

            // === PICKUP 3 ===
            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(buildAimedPickupPath(pickup2PoseStart, pickup2PoseEnd), pickupSpeed, true);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy() && actionTimer.getElapsedTime() > grabTime) {
                    follower.followPath(score4, 1, true);
                    setPathState(13);
                }
                break;

            // === SCORE 4 ===
            case 13:
                if (!follower.isBusy() && launcher.isFlywheelReady()) {
                    launcher.setState(Launcher.LauncherState.LAUNCH);
                    setPathState(14);
                }
                break;
            case 14:
                if (actionTimer.getElapsedTime() > launchTime) {
                    follower.followPath(pickup4, 1, true);
                    setPathState(15);
                }
                break;

            // === PICKUP 4 ===
            case 15:
                if (!follower.isBusy()) {
                    follower.followPath(buildAimedPickupPath(pickup2PoseStart, pickup2PoseEnd), pickupSpeed, true);
                    setPathState(16);
                }
                break;
            case 16:
                if (!follower.isBusy() && actionTimer.getElapsedTime() > grabTime) {
                    follower.followPath(score5, 1, true);
                    setPathState(17);
                }
                break;

            // === SCORE 5 ===
            case 17:
                if (!follower.isBusy() && launcher.isFlywheelReady()) {
                    launcher.setState(Launcher.LauncherState.LAUNCH);
                    setPathState(18);
                }
                break;
            case 18:
                if (actionTimer.getElapsedTime() > launchTime) {
                    follower.followPath(pickup5, 1, true);
                    setPathState(19);
                }
                break;

            // === PICKUP 5 ===
            case 19:
                if (!follower.isBusy()) {
                    follower.followPath(buildAimedPickupPath(pickup2PoseStart, pickup2PoseEnd), pickupSpeed, true);
                    setPathState(20);
                }
                break;
            case 20:
                if (!follower.isBusy() && actionTimer.getElapsedTime() > grabTime) {
                    follower.followPath(score6, 1, true);
                    setPathState(21);
                }
                break;

            // === SCORE 6 ===
            case 21:
                if (!follower.isBusy() && launcher.isFlywheelReady()) {
                    launcher.setState(Launcher.LauncherState.LAUNCH);
                    setPathState(22);
                }
                break;
            case 22:
                if (actionTimer.getElapsedTime() > launchTime) {
                    follower.followPath(leave, 1, true);
                    setPathState(23);
                }
                break;

            // === LEAVE ===
            case 23:
                if (!follower.isBusy()) {
                    setPathState(-1);
                    requestOpModeStop();
                }
                break;
        }
    }
}
