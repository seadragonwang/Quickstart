package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//@Disabled
@Autonomous(name = "Blue Far Auto", group = "OrcaRobotics")
public class BlueFarAuto extends AutoBase {

    { grabTime = 1400; pickupSpeed = 0.9; }

    private final Pose startPose = new Pose(55.8, 7.5, Math.toRadians(180));
    private final Pose pickup1PoseStart = new Pose(42, 36, Math.toRadians(180));
    private final Pose pickup1PoseEnd = new Pose(14, 36, Math.toRadians(180));
    private final Pose scorePose = new Pose(52, 16.5, Math.toRadians(180));
    private final Pose pickup2PoseStart = new Pose(16, 9, Math.toRadians(180));
    private final Pose pickup2PoseEnd = new Pose(13, 9, Math.toRadians(180));
    private final Pose leavePose = new Pose(44, 25, Math.toRadians(180));

    private PathChain
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
    protected PIDFCoefficients getPidfCoefficients() {
        return Constants.farPidfCoefficients;
    }

    @Override
    protected void buildPaths() {
        // pickup1: Switch to pickup mode mid-path
        pickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, new Pose(56, 26), pickup1PoseStart))
                .setLinearHeadingInterpolation(startPose.getHeading(), pickup1PoseStart.getHeading())
                .addParametricCallback(0.2, () -> launcher.setState(Launcher.LauncherState.PICKUP))
                .build();

        pickup1Path = follower.pathBuilder()
                .addPath(new BezierLine(pickup1PoseStart, pickup1PoseEnd))
                .build();

        // score2: Prep launcher mid-path
        score2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup1PoseEnd, new Pose(48, 38), scorePose))
                .setLinearHeadingInterpolation(pickup1PoseEnd.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.1, () -> launcher.setState(Launcher.LauncherState.START_LAUNCHING_BLUE_FAR))
                .build();

        // pickup2: Switch to pickup mode mid-path
        pickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(34, 20), new Pose(16, 16), pickup2PoseStart))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2PoseStart.getHeading())
                .addParametricCallback(0.2, () -> launcher.setState(Launcher.LauncherState.PICKUP))
                .build();

        pickup2Path = follower.pathBuilder()
                .addPath(new BezierLine(pickup2PoseStart, pickup2PoseEnd))
                .build();

        // score3: Prep launcher mid-path
        score3 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2PoseEnd, new Pose(29, 21), scorePose))
                .setLinearHeadingInterpolation(pickup2PoseEnd.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.1, () -> launcher.setState(Launcher.LauncherState.START_LAUNCHING_BLUE_FAR))
                .build();

        // pickup3: Switch to pickup mode mid-path
        pickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(34, 20), new Pose(16, 16), pickup2PoseStart))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2PoseStart.getHeading())
                .addParametricCallback(0.2, () -> launcher.setState(Launcher.LauncherState.PICKUP))
                .build();

        pickup3Path = follower.pathBuilder()
                .addPath(new BezierLine(pickup2PoseStart, pickup2PoseEnd))
                .build();

        // score4: Prep launcher mid-path
        score4 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2PoseEnd, new Pose(29, 21), scorePose))
                .setLinearHeadingInterpolation(pickup2PoseEnd.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.1, () -> launcher.setState(Launcher.LauncherState.START_LAUNCHING_BLUE_FAR))
                .build();

        // pickup4: Switch to pickup mode mid-path
        pickup4 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(34, 20), new Pose(16, 16), pickup2PoseStart))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2PoseStart.getHeading())
                .addParametricCallback(0.2, () -> launcher.setState(Launcher.LauncherState.PICKUP))
                .build();

        pickup4Path = follower.pathBuilder()
                .addPath(new BezierLine(pickup2PoseStart, pickup2PoseEnd))
                .build();

        // score5: Prep launcher mid-path
        score5 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2PoseEnd, new Pose(29, 21), scorePose))
                .setLinearHeadingInterpolation(pickup2PoseEnd.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.1, () -> launcher.setState(Launcher.LauncherState.START_LAUNCHING_BLUE_FAR))
                .build();

        // pickup5: Switch to pickup mode mid-path
        pickup5 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(34, 20), new Pose(16, 16), pickup2PoseStart))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2PoseStart.getHeading())
                .addParametricCallback(0.2, () -> launcher.setState(Launcher.LauncherState.PICKUP))
                .build();

        pickup5Path = follower.pathBuilder()
                .addPath(new BezierLine(pickup2PoseStart, pickup2PoseEnd))
                .build();

        // score6: Prep launcher mid-path
        score6 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2PoseEnd, new Pose(29, 21), scorePose))
                .setLinearHeadingInterpolation(pickup2PoseEnd.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.1, () -> launcher.setState(Launcher.LauncherState.START_LAUNCHING_BLUE_FAR))
                .build();

        leave = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(54, 21), leavePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), leavePose.getHeading())
                .build();
    }

    @Override
    protected void autonomousPathUpdate() {
        switch (pathState) {
            // === SCORE 1 (preloaded balls — launch from start position) ===
            case 0:
                launcher.setState(Launcher.LauncherState.START_LAUNCHING_BLUE_FAR);
                setPathState(1);
                break;
            case 1: // Wait for flywheel to reach target velocity
                if (launcher.isFlywheelReady()) {
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
                if (!follower.isBusy()) {
                    follower.followPath(score2, 1, true);
                    setPathState(5);
                }
                break;

            // === SCORE 2 (callback preps launcher at 30%) ===
            case 5: // Arrived → launch
                if (!follower.isBusy()) {
                    launcher.setState(Launcher.LauncherState.LAUNCH);
                    setPathState(6);
                }
                break;
            case 6: // Wait for all balls
                if (actionTimer.getElapsedTime() > launchTime) {
                    follower.followPath(pickup2, 1, true);
                    setPathState(7);
                }
                break;

            // === PICKUP 2 ===
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(pickup2Path, pickupSpeed, true);
                    setPathState(8);
                }
                break;
            case 8: // Wait for pickup
                if (!follower.isBusy() && actionTimer.getElapsedTime() > grabTime) {
                    follower.followPath(score3, 1, true);
                    setPathState(9);
                }
                break;

            // === SCORE 3 (callback preps launcher at 30%) ===
            case 9: // Arrived → launch
                if (!follower.isBusy()) {
                    launcher.setState(Launcher.LauncherState.LAUNCH);
                    setPathState(10);
                }
                break;
            case 10: // Wait for all balls
                if (actionTimer.getElapsedTime() > launchTime) {
                    follower.followPath(pickup3, 1, true);
                    setPathState(11);
                }
                break;

            // === PICKUP 3 ===
            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(pickup3Path, pickupSpeed, true);
                    setPathState(12);
                }
                break;
            case 12: // Wait for pickup
                if (!follower.isBusy() && actionTimer.getElapsedTime() > grabTime) {
                    follower.followPath(score4, 1, true);
                    setPathState(13);
                }
                break;

            // === SCORE 4 (callback preps launcher at 30%) ===
            case 13: // Arrived → launch
                if (!follower.isBusy()) {
                    launcher.setState(Launcher.LauncherState.LAUNCH);
                    setPathState(14);
                }
                break;
            case 14: // Wait for all balls
                if (actionTimer.getElapsedTime() > launchTime) {
                    follower.followPath(pickup4, 1, true);
                    setPathState(15);
                }
                break;

            // === PICKUP 4 ===
            case 15:
                if (!follower.isBusy()) {
                    follower.followPath(pickup4Path, pickupSpeed, true);
                    setPathState(16);
                }
                break;
            case 16: // Wait for pickup
                if (!follower.isBusy() && actionTimer.getElapsedTime() > grabTime) {
                    follower.followPath(score5, 1, true);
                    setPathState(17);
                }
                break;

            // === SCORE 5 (callback preps launcher at 30%) ===
            case 17: // Arrived → launch
                if (!follower.isBusy()) {
                    launcher.setState(Launcher.LauncherState.LAUNCH);
                    setPathState(18);
                }
                break;
            case 18: // Wait for all balls
                if (actionTimer.getElapsedTime() > launchTime) {
                    follower.followPath(pickup5, 1, true);
                    setPathState(19);
                }
                break;

            // === PICKUP 5 ===
            case 19:
                if (!follower.isBusy()) {
                    follower.followPath(pickup5Path, pickupSpeed, true);
                    setPathState(20);
                }
                break;
            case 20: // Wait for pickup
                if (!follower.isBusy() && actionTimer.getElapsedTime() > grabTime) {
                    follower.followPath(score6, 1, true);
                    setPathState(21);
                }
                break;

            // === SCORE 6 (callback preps launcher at 30%) ===
            case 21: // Arrived → launch
                if (!follower.isBusy()) {
                    launcher.setState(Launcher.LauncherState.LAUNCH);
                    setPathState(22);
                }
                break;
            case 22: // Wait for all balls, then leave
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
