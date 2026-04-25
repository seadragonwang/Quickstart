package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.CONSTANTS.CLOSE_INTAKE_POWER;
import static org.firstinspires.ftc.teamcode.CONSTANTS.CLOSE_OUTTAKE_VELOCITY;
import static org.firstinspires.ftc.teamcode.CONSTANTS.FAR_OUTTAKE_VELOCITY;
import static org.firstinspires.ftc.teamcode.CONSTANTS.HOOD_MAX_POS;
import static org.firstinspires.ftc.teamcode.CONSTANTS.HOOD_MIN_POS;
import static org.firstinspires.ftc.teamcode.CONSTANTS.MAX_TURRET_ANGLE;
import static org.firstinspires.ftc.teamcode.CONSTANTS.kD;
import static org.firstinspires.ftc.teamcode.CONSTANTS.kI;
import static org.firstinspires.ftc.teamcode.CONSTANTS.kP;

import org.firstinspires.ftc.teamcode.CONSTANTS;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Base class for all TeleOp modes. Subclasses only need to provide:
 *   - getStartingPose()
 *   - getGoalX() / getGoalY()
 *   - getLimelightStartingHeadingDeg()
 *   - getValidTagIds()
 */
public abstract class TeleBase extends LinearOpMode {

    // --- Abstract methods for subclass configuration ---
    protected abstract Pose getStartingPose();
    protected abstract double getGoalX();
    protected abstract double getGoalY();
    protected abstract double getLimelightStartingHeadingDeg();
    protected abstract int[] getValidTagIds();
    // class fields
    private double batteryVoltage = 12.5;
    private static final double V_NOM = 12.5;
    private static final double KVOLT = 35.0; // ticks/s per volt (tune)
    private ShooterModel shooterModel;
    private double filteredDistance = 70.0;
    private double lastFComp = -1;
    /** How far outside the launch zone (inches) to begin pre-spinning the flywheel */
    private static final double LAUNCH_ZONE_PRE_SPIN_MARGIN = 10.0;
    // --- Hardware & state (all shared) ---
    protected Follower follower;
    protected double angleToGoal;
    protected double turretError;
    public static Pose startingPose;

    protected ElapsedTime runtime = new ElapsedTime();
    protected DcMotor rightFront = null;
    protected DcMotor leftFront = null;
    protected DcMotor rightBack = null;
    protected DcMotor leftBack = null;
    protected Servo turretServo = null;
    /**
     * max position: 0.93
     * min position: 0.23
     */
    protected Servo hoodServo = null;
    protected DcMotorEx outtake1 = null;
    protected DcMotorEx outtake2 = null;
    protected DcMotorEx intake1 = null;
    protected DcMotor intake2 = null;
    protected double intake1Power = 0.0;
    protected double intake2Power = 0.0;
    protected double targetv = 0;
    protected double UPPER_LIMIT_VELOCITY = FAR_OUTTAKE_VELOCITY;
    protected double lastTargetV = 0;
    protected boolean autoUpdate = false;
    protected ElapsedTime velocityTimer = new ElapsedTime();


    protected enum INTAKE_STATUS {
        INTAKE_STOPPED,
        INTAKE_STARTED,
        INTAKE_NEED_TO_BE_MONITORED,
        INTAKE_MONITORING,
        INTAKE_JAMMED
    }

    protected double targetOuttakeVelocity = 0.0;

    // pidf
    protected double outtakeZeroPower = 0.0;
    protected double motorOneCurrentVelocity = 0.0;
    protected double motorOneMaxVelocity = 2800;
    protected double F = 14.1/*32767 / motorOneMaxVelocity*/;

    protected double position = 5.0;
    protected double turretPos = 0.5;
    protected double robotHeading = Math.PI / 2;
    protected double hoodPos = 0.5;

    protected boolean intake1On = false;
    protected double intake1Vel = 0.0;
    protected INTAKE_STATUS intakeStatus = INTAKE_STATUS.INTAKE_STOPPED;
    protected Servo pivot;
    protected BallDetectorThread ballDetectorThread;
    protected LimelightLocalizer limelightLocalizer;

    protected ElapsedTime intakeTimer = new ElapsedTime();

    protected static final boolean USE_WEBCAM = false;
    protected AprilTagProcessor aprilTag;
    protected VisionPortal visionPortal;
    protected ElapsedTime loopTimer = new ElapsedTime();

    private double getBatteryVoltage() {
        double v = Double.POSITIVE_INFINITY;
        for (com.qualcomm.robotcore.hardware.VoltageSensor sensor : hardwareMap.voltageSensor) {
            double sv = sensor.getVoltage();
            if (sv > 0) v = Math.min(v, sv); // lowest valid is safest
        }
        return Double.isInfinite(v) ? 12.0 : v;
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initHardware();
        follower = Constants.createFollower(hardwareMap);
        startingPose = getStartingPose();
        boolean aiming = true;
        follower.setStartingPose(startingPose);
        follower.startTeleopDrive();

        // Reduce telemetry transmission frequency to free up loop time
        telemetry.setMsTransmissionInterval(100);

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            follower.update();

            // Limelight pose correction
            limelightLocalizer.update(follower);

            Pose pose = follower.getPose();

            // Drive — process input FIRST for minimum latency
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;
            follower.setTeleOpDrive(y, -x, rx);

            // Read intake velocity once per loop (avoid redundant I2C calls)
            intake1Vel = intake1.getVelocity();

//            ballDetector.update();
            // OUTTAKE
            if (gamepad2.left_bumper) {
                targetOuttakeVelocity = FAR_OUTTAKE_VELOCITY+50;
                autoUpdate = false;
            } else if (gamepad2.right_bumper) {
                targetOuttakeVelocity = CLOSE_OUTTAKE_VELOCITY;
                autoUpdate = false;
            } else if (gamepad2.dpad_right) {
                targetOuttakeVelocity = 0;
                autoUpdate = false;
            } else if (gamepad2.dpad_up) {
                autoUpdate = true;
                velocityTimer.reset();
            }

            // Update filtered distance every loop so it's always current when autoUpdate fires

            // TOGGLE
            if (gamepad1.x && gamepad1.b) {
                UPPER_LIMIT_VELOCITY = FAR_OUTTAKE_VELOCITY;
            } else if (gamepad1.a && gamepad1.y) {
                UPPER_LIMIT_VELOCITY = CLOSE_OUTTAKE_VELOCITY + 100;
            }

            if (autoUpdate && LaunchZoneChecker.isNearLaunchZone(pose, LAUNCH_ZONE_PRE_SPIN_MARGIN)) {
                // Pre-spin flywheel based on distance when within 10 in of launch zone,
                // and continue updating once inside
                filteredDistance = 0.5 * filteredDistance + 0.5 * getRobotToGoalDistance();
                batteryVoltage = getBatteryVoltage();

                ShooterModel.ShotSolution sol = shooterModel.solve(filteredDistance, batteryVoltage);
                targetOuttakeVelocity = Range.clip(sol.velocityTicksPerSec, 1100, UPPER_LIMIT_VELOCITY);

                if (Math.abs(sol.hoodPos - hoodPos) > 0.006) {
                    hoodPos = sol.hoodPos;
                }
            }
            outtake1.setVelocity(targetOuttakeVelocity);
            outtake2.setVelocity(targetOuttakeVelocity);
            hoodServo.setPosition(Range.clip(hoodPos, HOOD_MIN_POS, HOOD_MAX_POS));

            // AIMING
            if (gamepad1.right_trigger > 0.3) {
                aiming = true;
            }
            if (gamepad1.left_trigger > 0.3) {
                aiming = false;
                turretPos = 0.5;
            }

            if (aiming) {
                double robotX = pose.getX();
                double robotY = pose.getY();
                double rh = pose.getHeading();

                double turretX = robotX
                        + CONSTANTS.TURRET_OFFSET_FORWARD * Math.cos(rh)
                        - CONSTANTS.TURRET_OFFSET_LEFT * Math.sin(rh);
                double turretY = robotY
                        + CONSTANTS.TURRET_OFFSET_FORWARD * Math.sin(rh)
                        + CONSTANTS.TURRET_OFFSET_LEFT * Math.cos(rh);

                angleToGoal = Math.atan2(getGoalY() - turretY, getGoalX() - turretX);
                turretError = angleToGoal - rh;

                while (turretError > Math.PI) turretError -= 2 * Math.PI;
                while (turretError < -Math.PI) turretError += 2 * Math.PI;

                double errorDegrees = Math.toDegrees(turretError);
                errorDegrees = Range.clip(errorDegrees, -MAX_TURRET_ANGLE, MAX_TURRET_ANGLE);

                turretPos = TurretMapper.degreesToServoPos(errorDegrees);
                turretServo.setPosition(Range.clip(turretPos, 0.28, 0.694));
            }

            // INTAKE
            if (gamepad1.right_bumper) {
                intake1Power = CLOSE_INTAKE_POWER;
                intakeStatus = INTAKE_STATUS.INTAKE_STARTED;
                intake2Power = 0;
            } else if (gamepad1.left_bumper) {
                intake1Power = 0;
                intake2Power = 0;
                intakeStatus = INTAKE_STATUS.INTAKE_STOPPED;
            } else if (gamepad1.dpad_left) {
                intake1Power = -CLOSE_INTAKE_POWER;
            } else if (gamepad2.a) {
                intake2Power = 1.0;
                intake1Power = CLOSE_INTAKE_POWER;
                intakeStatus = INTAKE_STATUS.INTAKE_STARTED;
                if (targetOuttakeVelocity > 0) {
                    ballDetectorThread.getBallDetector().resetLoadedCount();
                }
            } else if (gamepad2.b) {
                intake2Power = 0.0;
                intake1Power = 0;
                intakeStatus = INTAKE_STATUS.INTAKE_STOPPED;
            }

            // Hood: interpolate linearly based on distance to goal
            // Near (~45 in) -> 0.93, Far (~130 in) -> 0.75
            // Only update when change is significant to prevent jitter from LL corrections
//            {
//                double d = getRobotToGoalDistance();
//                double newHoodPos = Range.clip((0.93 - (0.45 / 85.0) * (d - 45)), 0.35, 0.93); //
//                if (Math.abs(newHoodPos - hoodPos) > 0.006) {
//                    hoodPos = newHoodPos;
//                }
//            }

            // Intake jam detection (intake1Vel already read above)
            if (intakeStatus == INTAKE_STATUS.INTAKE_STARTED && intake1Vel > 800) {
                intakeStatus = INTAKE_STATUS.INTAKE_NEED_TO_BE_MONITORED;
            } else if (intakeStatus == INTAKE_STATUS.INTAKE_NEED_TO_BE_MONITORED && intake1Vel < 200) {
                intakeStatus = INTAKE_STATUS.INTAKE_MONITORING;
                intakeTimer.reset();
            } else if (intakeStatus == INTAKE_STATUS.INTAKE_MONITORING && intake1Vel < 400 && intakeTimer.milliseconds() > 50) {
                intakeStatus = INTAKE_STATUS.INTAKE_JAMMED;
            } else if (intakeStatus == INTAKE_STATUS.INTAKE_JAMMED) {
                intake1Power = 0;
                intakeTimer.reset();
                intakeStatus = INTAKE_STATUS.INTAKE_STOPPED;
            }

            intake1.setPower(intake1Power);
            intake2.setPower(intake2Power);
            showTelemetry();
            telemetry.update();
        }
        ballDetectorThread.stopThread();
    }

    // --- Hardware init ---
    public void initHardware() {
        initMotorOne(kP, kI, kD, F, position);
        initMotorTwo(kP, kI, kD, F, position);
        initDriveMotors();
        initAprilTag();
//        ballDetectorThread = new BallDetectorThread();
//        ballDetector = new BallDetector(hardwareMap);
        initIntake();
        initTurret();

        ballDetectorThread = new BallDetectorThread(hardwareMap);
        ballDetectorThread.start();

        limelightLocalizer = new LimelightLocalizer();
        limelightLocalizer.init(hardwareMap, getLimelightStartingHeadingDeg());
        limelightLocalizer.setValidTagIds(getValidTagIds());
        shooterModel = new ShooterModel(HOOD_MIN_POS, HOOD_MAX_POS, 1100, 2500);
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        builder.enableLiveView(false);
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    public void initDriveMotors() {
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        leftFront = hardwareMap.get(DcMotor.class, "lf");
        rightBack = hardwareMap.get(DcMotor.class, "rr");
        leftBack = hardwareMap.get(DcMotor.class, "lr");

        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void initIntake() {
        intake1 = hardwareMap.get(DcMotorEx.class, "intake1");
        intake1.setDirection(DcMotorSimple.Direction.REVERSE);
        intake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        intake2.setDirection(DcMotorSimple.Direction.FORWARD);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void initTurret() {
        turretServo = hardwareMap.get(Servo.class, "turretServo");
        turretServo.setPosition(Range.clip(turretPos, 0, 1));

        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        hoodServo.setPosition(Range.clip(hoodPos, 0, 1));
    }

    private void initMotorOne(double kP, double kI, double kD, double F, double position) {
        outtake1 = hardwareMap.get(DcMotorEx.class, "outtake1");
        outtake1.setDirection(DcMotorEx.Direction.REVERSE);
        outtake1.setVelocityPIDFCoefficients(kP, kI, kD, F);
        outtake1.setPower(outtakeZeroPower);
        outtake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake1.setVelocity(targetOuttakeVelocity);
    }

    private void initMotorTwo(double kP, double kI, double kD, double F, double position) {
        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");
        outtake2.setDirection(DcMotorEx.Direction.FORWARD);
        outtake2.setVelocityPIDFCoefficients(kP, kI, kD, F);
        outtake2.setPower(outtakeZeroPower);
        outtake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake2.setVelocity(targetOuttakeVelocity);
    }

    // --- Utility ---
    public double getRobotToGoalDistance() {
        Pose pose = follower.getPose();
        double dx = getGoalX() - pose.getX();
        double dy = getGoalY() - pose.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    protected boolean getGoal() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (detection.id == 20 || detection.id == 24) {
                    return true;
                }
            }
        }
        return false;
    }

    // --- Telemetry ---
    public void showTelemetry() {
        telemetry.addData("Loop ms", loopTimer.milliseconds());
        loopTimer.reset();
        telemetry.addData("distance to goal", getRobotToGoalDistance());
        telemetry.addData("autoUpdate", autoUpdate);
        telemetry.addData("in launch zone", LaunchZoneChecker.isAnyWheelInLaunchZone(follower.getPose()));
        telemetry.addData("near launch zone", LaunchZoneChecker.isNearLaunchZone(follower.getPose(), LAUNCH_ZONE_PRE_SPIN_MARGIN));
        telemetry.addData("ShooterModel active", autoUpdate && LaunchZoneChecker.isNearLaunchZone(follower.getPose(), LAUNCH_ZONE_PRE_SPIN_MARGIN));
        telemetry.addData("turretpos", turretPos);
        telemetry.addData("turretError", Math.toDegrees(turretError));
        telemetry.addData("x pos", follower.getPose().getX());
        telemetry.addData("y pos", follower.getPose().getY());
        double displayHeading = Math.toDegrees(follower.getHeading()) % 360;
        if (displayHeading < 0) displayHeading += 360;
        telemetry.addData("robot heading", displayHeading);
        telemetry.addData("hood", hoodPos);
        telemetry.addData("LL valid", limelightLocalizer.isLastResultValid());
        telemetry.addData("LL reject", limelightLocalizer.getLastRejectReason());
        telemetry.addData("Intake state", intakeStatus);
        telemetry.addData("intake velocity", intake1Vel);
        telemetry.addData("Target Velocity", targetOuttakeVelocity);
        telemetry.addData("Flywheel1 Velocity", outtake1.getVelocity());
        telemetry.addData("Flywheel2 Velocity", outtake2.getVelocity());
        telemetry.addData("Balls loaded", ballDetectorThread.getBallDetector().getBallsLoaded());
        telemetry.addData("Ball distance mm", ballDetectorThread.getBallDetector().getDistanceMM());
        telemetry.addData("Battery V", batteryVoltage);
        telemetry.addData("Distance filt", filteredDistance);
        telemetry.addData("Vel err 1", targetOuttakeVelocity - outtake1.getVelocity());
        telemetry.addData("Vel err 2", targetOuttakeVelocity - outtake2.getVelocity());

    }
}

