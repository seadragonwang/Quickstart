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

        leave = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(90, 21), leavePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), leavePose.getHeading())
                .build();
    }

    @Override
    public void loop() {
        super.loop();
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

            // === PICKUP 1 (spike mark) ===
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(pickup1Path, pickupSpeed, true);
                    setPathState(4);
                }
                break;
            case 4:
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

            // === PICKUP 2 (1st wall cycle) ===
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(pickup2Path, pickupSpeed, true);
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

            // === PICKUP 3 (2nd wall cycle) ===
            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(pickup3Path, pickupSpeed, true);
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

            // === PICKUP 4 (3rd wall cycle) ===
            case 15:
                if (!follower.isBusy()) {
                    follower.followPath(pickup4Path, pickupSpeed, true);
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
                    follower.followPath(leave, 1, true);
                    setPathState(19);
                }
                break;

            // === LEAVE ===
            case 19:
                if (!follower.isBusy()) {
                    setPathState(-1);
                    requestOpModeStop();
                }
                break;
        }
    }
}
