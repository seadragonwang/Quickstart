package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//@Disabled
@Autonomous(name = "Blue Close Auto", group = "OrcaRobotics")
public class BlueNearAuto extends AutoBase {

    private final Pose startPose = new Pose(15.5, 112.5, Math.toRadians(180));
    private final Pose scorePose = new Pose(50, 81, Math.toRadians(180)); // Wheel touches launch triangle
    private final Pose pickup1PoseStart = new Pose(24, 52, Math.toRadians(180));
    private final Pose pickup1PoseEnd = new Pose(16, 52, Math.toRadians(180));
    private final Pose openGateGrabStartPose = new Pose(17.5, 60, Math.toRadians(161));
    private final Pose openGateGrabEndPose = new Pose(11.5, 60, Math.toRadians(161));
    private final Pose pickup2PoseStart = new Pose(40, 82, Math.toRadians(180));
    private final Pose pickup2PoseEnd = new Pose(15, 82, Math.toRadians(180));
    private final Pose leavePose = new Pose(36, 70, Math.toRadians(180));

    private PathChain
            score1,
            pickup1,
            pickup1Path,
            score2,
            openGateStartGrab,
            openGateEndGrab,
            score3,
            openGateStartGrab2,
            openGateEndGrab2,
            score4,
            pickup2,
            pickup2Path,
            score5,
            leave;

    @Override
    protected Pose getStartPose() {
        return startPose;
    }

    @Override
    protected PIDFCoefficients getPidfCoefficients() {
        return Constants.closePidfCoefficients;
    }

    @Override
    protected void buildPaths() {
        // score1: Start launcher mid-path so it's ready by arrival
        // Use BezierCurve to route above spike marks before curving down to score
        score1 = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, new Pose(50, 112), scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.0, () -> launcher.setState(Launcher.LauncherState.START_LAUNCHING_BLUE_NEAR))
                .build();

        // pickup1: Switch to pickup mode mid-path
        pickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(51, 66), pickup1PoseStart))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1PoseStart.getHeading())
                .addParametricCallback(0.2, () -> launcher.setState(Launcher.LauncherState.PICKUP))
                .build();

        pickup1Path = follower.pathBuilder()
                .addPath(new BezierLine(pickup1PoseStart, pickup1PoseEnd))
                .build();

        // score2
        score2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup1PoseEnd, new Pose(49, 54), scorePose))
                .setLinearHeadingInterpolation(pickup1PoseEnd.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.1, () -> launcher.setState(Launcher.LauncherState.START_LAUNCHING_BLUE_NEAR))
                .build();

        // openGateStartGrab: Fast approach to gate area
        openGateStartGrab = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(43, 64), openGateGrabStartPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), openGateGrabStartPose.getHeading())
                .addParametricCallback(0.1, () -> launcher.setState(Launcher.LauncherState.PICKUP))
                .build();
        // openGateEndGrab: Slow grab into gate
        openGateEndGrab = follower.pathBuilder()
                .addPath(new BezierLine(openGateGrabStartPose, openGateGrabEndPose))
                .setConstantHeadingInterpolation(openGateGrabStartPose.getHeading())
                .build();

        // score3: from gate grab end to score3 — route BELOW spike marks to avoid hitting balls
        score3 = follower.pathBuilder()
                .addPath(new BezierCurve(openGateGrabEndPose, new Pose(30, 52), new Pose(50, 72), scorePose))
                .setLinearHeadingInterpolation(openGateGrabEndPose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.1, () -> launcher.setState(Launcher.LauncherState.START_LAUNCHING_BLUE_NEAR))
                .build();

        // openGateStartGrab2: Fast approach to gate area (second time)
        openGateStartGrab2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(42, 70), openGateGrabStartPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), openGateGrabStartPose.getHeading())
                .addParametricCallback(0.1, () -> launcher.setState(Launcher.LauncherState.PICKUP))
                .build();
        // openGateEndGrab2: Slow grab into gate (second time)
        openGateEndGrab2 = follower.pathBuilder()
                .addPath(new BezierLine(openGateGrabStartPose, openGateGrabEndPose))
                .setConstantHeadingInterpolation(openGateGrabStartPose.getHeading())
                .build();

        // score4: from gate grab end to score4 — route BELOW spike marks to avoid hitting balls
        score4 = follower.pathBuilder()
                .addPath(new BezierCurve(openGateGrabEndPose, new Pose(30, 52), new Pose(50, 72), scorePose))
                .setLinearHeadingInterpolation(openGateGrabEndPose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.1, () -> launcher.setState(Launcher.LauncherState.START_LAUNCHING_BLUE_NEAR))
                .build();

        // pickup2: Switch to pickup mode mid-path (from score4)
        pickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(42, 83), pickup2PoseStart))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2PoseStart.getHeading())
                .addParametricCallback(0.2, () -> launcher.setState(Launcher.LauncherState.PICKUP))
                .build();

        pickup2Path = follower.pathBuilder()
                .addPath(new BezierLine(pickup2PoseStart, pickup2PoseEnd))
                .build();

        // score5
        score5 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2PoseEnd, new Pose(38, 82), scorePose))
                .setLinearHeadingInterpolation(pickup2PoseEnd.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.1, () -> launcher.setState(Launcher.LauncherState.START_LAUNCHING_BLUE_NEAR))
                .build();

        leave = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(37, 87), leavePose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    @Override
    protected void autonomousPathUpdate() {
        switch (pathState) {
            // === SCORE 1 ===
            case 0:
                follower.followPath(score1, 1, true);
                setPathState(1);
                break;
            case 1: // Arrived → wait for flywheel then launch immediately
                if (!follower.isBusy() && launcher.isFlywheelReady()) {
                    launcher.setState(Launcher.LauncherState.LAUNCH);
                    setPathState(3);
                }
                break;
            case 3: // Wait for all balls
                if (actionTimer.getElapsedTime() > launchTime) {
                    follower.followPath(pickup1, 1, true);
                    setPathState(4);
                }
                break;

            // === PICKUP 1 ===
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(pickup1Path, pickupSpeed, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(score2, 1, true);
                    setPathState(6);
                }
                break;

            // === SCORE 2 (callback preps launcher at 10%) ===
            case 6: // Arrived → launch
                if (!follower.isBusy()) {
                    launcher.setState(Launcher.LauncherState.LAUNCH);
                    setPathState(7);
                }
                break;
            case 7: // Wait for all balls
                if (actionTimer.getElapsedTime() > launchTime) {
                    follower.followPath(openGateStartGrab, 1.0, true);
                    setPathState(8);
                }
                break;

            // === OPEN GATE GRAB 1 ===
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(openGateEndGrab, grabSpeed, true);
                    setPathState(9);
                }
                break;
            case 9: // Wait for pickup
                if (!follower.isBusy() && actionTimer.getElapsedTime() > grabTime) {
                    follower.followPath(score3, 1.0, true);
                    setPathState(10);
                }
                break;

            // === SCORE 3 (callback preps launcher at 10%) ===
            case 10: // Arrived → launch
                if (!follower.isBusy()) {
                    launcher.setState(Launcher.LauncherState.LAUNCH);
                    setPathState(11);
                }
                break;
            case 11: // Wait for all balls
                if (actionTimer.getElapsedTime() > launchTime) {
                    follower.followPath(openGateStartGrab2, 1.0, true);
                    setPathState(12);
                }
                break;

            // === OPEN GATE GRAB 2 ===
            case 12:
                if (!follower.isBusy()) {
                    follower.followPath(openGateEndGrab2, grabSpeed, true);
                    setPathState(13);
                }
                break;
            case 13: // Wait for pickup
                if (!follower.isBusy() && actionTimer.getElapsedTime() > grabTime) {
                    follower.followPath(score4, 1.0, true);
                    setPathState(14);
                }
                break;

            // === SCORE 4 (callback preps launcher at 10%) ===
            case 14: // Arrived → launch
                if (!follower.isBusy()) {
                    launcher.setState(Launcher.LauncherState.LAUNCH);
                    setPathState(15);
                }
                break;
            case 15: // Wait for all balls, then go to pickup 2
                if (actionTimer.getElapsedTime() > launchTime) {
                    follower.followPath(pickup2, 1, false);
                    setPathState(16);
                }
                break;

            // === PICKUP 2 ===
            case 16:
                if (!follower.isBusy()) {
                    follower.followPath(pickup2Path, pickupSpeed, true);
                    setPathState(17);
                }
                break;
            case 17:
                if (!follower.isBusy()) {
                    follower.followPath(score5, 1, true);
                    setPathState(18);
                }
                break;

            // === SCORE 5 (callback preps launcher at 10%) ===
            case 18: // Arrived → launch
                if (!follower.isBusy()) {
                    launcher.setState(Launcher.LauncherState.LAUNCH);
                    setPathState(19);
                }
                break;
            case 19: // Wait for all balls, then leave
                if (actionTimer.getElapsedTime() > launchTime) {
                    follower.followPath(leave, 1, true);
                    setPathState(20);
                }
                break;

            // === LEAVE ===
            case 20:
                if (!follower.isBusy()) {
                    setPathState(-1);
                    requestOpModeStop();
                }
                break;
        }
    }
}
