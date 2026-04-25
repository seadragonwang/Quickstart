/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.CONSTANTS.BLUE_GOAL_POSITION_X;
import static org.firstinspires.ftc.teamcode.CONSTANTS.BLUE_GOAL_POSITION_Y;
import static org.firstinspires.ftc.teamcode.CONSTANTS.BLUE_NEAR_TELE_START;
import static org.firstinspires.ftc.teamcode.CONSTANTS.CLOSE_INTAKE_POWER;
import static org.firstinspires.ftc.teamcode.CONSTANTS.CLOSE_OUTTAKE_VELOCITY;
import static org.firstinspires.ftc.teamcode.CONSTANTS.FAR_OUTTAKE_VELOCITY;
import static org.firstinspires.ftc.teamcode.CONSTANTS.HOOD_MAX_POS;
import static org.firstinspires.ftc.teamcode.CONSTANTS.HOOD_MIN_POS;
import static org.firstinspires.ftc.teamcode.CONSTANTS.MAX_TURRET_ANGLE;
import static org.firstinspires.ftc.teamcode.CONSTANTS.RED_GOAL_POSITION_X;
import static org.firstinspires.ftc.teamcode.CONSTANTS.RED_GOAL_POSITION_Y;
import static org.firstinspires.ftc.teamcode.CONSTANTS.TURRET_POSITION_PER_DEGREE;
import static org.firstinspires.ftc.teamcode.CONSTANTS.kD;
import static org.firstinspires.ftc.teamcode.CONSTANTS.kI;
import static org.firstinspires.ftc.teamcode.CONSTANTS.kP;

import org.firstinspires.ftc.teamcode.CONSTANTS;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Blue CLOSE Tele", group="OrcaRobotics")
public class BlueCloseTele extends LinearOpMode {
//    Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));

    // Declare OpMode members.
//    public MecanumDrive drive =  new MecanumDrive(hardwareMap, initialPose);
    private Follower follower;
    double angleToGoal;
    double turretError;
    public static Pose startingPose;

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rightFront = null;
    private DcMotor leftFront = null;
    private DcMotor rightBack = null;
    private DcMotor leftBack = null;
    private Servo turretServo = null;
    private Servo hoodServo = null;
    private DcMotorEx outtake1 = null;
    private DcMotorEx outtake2 = null;
    private DcMotorEx intake1 = null;
    private DcMotor intake2 = null;
    double intake1Power = 0.0;
    double intake2Power = 0.0;
    double targetv = 0;
    double lastTargetV = 0;
    boolean autoUpdate = false;
    ElapsedTime velocityTimer = new ElapsedTime();
    enum INTAKE_STATUS {
        INTAKE_STOPPED,
        INTAKE_STARTED,
        INTAKE_NEED_TO_BE_MONITORED,
        INTAKE_MONITORING,
        INTAKE_JAMMED
    }
    double targetOuttakeVelocity = 0.0;

    // sensors
//    private NormalizedColorSensor r_frontColorSensor;
//    private NormalizedColorSensor l_frontColorSensor;
//    private NormalizedColorSensor r_backColorSensor;
//    private NormalizedColorSensor l_backColorSensor;

    // pidf
    private double outtakeZeroPower = 0.0;
    private double motorOneCurrentVelocity = 0.0;
    private double motorOneMaxVelocity = 2800;
    private double F = 32767/motorOneMaxVelocity;

    private double position = 5.0;
    private double turretPos = 0.5;
    private double robotHeading = Math.PI/2;
    private double hoodPos = 0.5;

    boolean intake1On = false;
    double intake1Vel = 0.0;
    INTAKE_STATUS intakeStatus = INTAKE_STATUS.INTAKE_STOPPED;
    private Servo pivot;
    private BallDetector ballDetector;
    private LimelightLocalizer limelightLocalizer;

    ElapsedTime intakeTimer = new ElapsedTime();

    private static final boolean USE_WEBCAM = false;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initHardware();
        follower = Constants.createFollower(hardwareMap);
        startingPose = BLUE_NEAR_TELE_START;
//        startingPose = new Pose(15.5, 112.5, Math.toRadians(180));
        boolean aiming = false;
        follower.setStartingPose(startingPose); // Or your last Auto pose
        follower.startTeleopDrive();

        float step = 0.001f;
        double outtakeStep = 7.0;


        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            follower.update();

            // Limelight pose correction — corrects X, Y, heading when AprilTags visible
            limelightLocalizer.update(follower);

            Pose pose = follower.getPose();
            double heading = pose.getHeading(); // Radians

            //drive
            double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
            double x = gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;
            follower.setTeleOpDrive(y,-x,rx);
//            double leftFrontPower = Range.clip((y + x + rx),-DRIVE_POWER,DRIVE_POWER);
//            double leftBackPower = Range.clip((y - x + rx),-DRIVE_POWER,DRIVE_POWER);
//            double rightFrontPower = Range.clip((y - x - rx),-DRIVE_POWER,DRIVE_POWER);
//            double rightBackPower = Range.clip((y + x - rx),-DRIVE_POWER,DRIVE_POWER);
            intake1Vel = intake1.getVelocity();

//            leftFront.setPower(leftFrontPower);
//            leftBack.setPower(leftBackPower);
//            rightFront.setPower(rightFrontPower);
//            rightBack.setPower(rightBackPower);

            ballDetector.update();

            // Get raw distance
            double distanceMM = ballDetector.getDistanceMM();
            // OUTTAKE//99 100 0.57
            if(gamepad2.left_bumper) {
                targetOuttakeVelocity = FAR_OUTTAKE_VELOCITY; // 2200 tip of far triangle 2300 for back then
                autoUpdate = false;
            } else if(gamepad2.right_bumper) {
                targetOuttakeVelocity = CLOSE_OUTTAKE_VELOCITY;
                autoUpdate = false;
            } else if (gamepad2.dpad_right) {
                targetOuttakeVelocity = 0;
                autoUpdate = false;
            } else if (gamepad2.dpad_up){
                autoUpdate = true;
                velocityTimer.reset();
            }
            outtake1.setVelocity(targetOuttakeVelocity);
            outtake2.setVelocity(targetOuttakeVelocity);

            if (autoUpdate) {

                double d = getRobotToGoalDistance();

//                targetv = Range.clip(
//                        1600
//                                + 2.0 * (d - 35)
//                                + 0.06 * Math.pow(d - 35, 2),
//                        1000,
//                        FAR_OUTTAKE_VELOCITY
//                );
                targetv = Range.clip(
                        (500.0 / (130 - 45)) * (getRobotToGoalDistance() - 45) + 1350,
                        1000,
                        FAR_OUTTAKE_VELOCITY
                );
                targetOuttakeVelocity = targetv;
            }



            if (gamepad1.right_trigger > 0.3) {
                aiming = true;
            } if (gamepad1.left_trigger > 0.3) {
                aiming = false;
            }

            if (aiming){
                // 1. Compute turret world position (turret is offset from robot center)
                double robotX = pose.getX();
                double robotY = pose.getY();
                double robotHeading = pose.getHeading();

                // Transform turret offset from robot-local frame to field frame
                double turretX = robotX
                        + CONSTANTS.TURRET_OFFSET_FORWARD * Math.cos(robotHeading)
                        - CONSTANTS.TURRET_OFFSET_LEFT * Math.sin(robotHeading);
                double turretY = robotY
                        + CONSTANTS.TURRET_OFFSET_FORWARD * Math.sin(robotHeading)
                        + CONSTANTS.TURRET_OFFSET_LEFT * Math.cos(robotHeading);

                // 2. Angle from turret position to goal
                angleToGoal = Math.atan2(BLUE_GOAL_POSITION_Y - turretY, BLUE_GOAL_POSITION_X - turretX);
                turretError = angleToGoal - robotHeading;

                while (turretError > Math.PI) turretError -= 2 * Math.PI;
                while (turretError < -Math.PI) turretError += 2 * Math.PI;

                double errorDegrees = Math.toDegrees(turretError);

// Clamp turret angle to ±135° to prevent over-rotation
                errorDegrees = Range.clip(errorDegrees, -MAX_TURRET_ANGLE, MAX_TURRET_ANGLE);

                turretPos = TurretMapper.degreesToServoPos(errorDegrees);
                turretServo.setPosition(Range.clip(turretPos, 0.28, 0.694));
            }

            // INTAKE
            if (gamepad1.right_bumper) {
                intake1Power = CLOSE_INTAKE_POWER;
                intakeStatus = INTAKE_STATUS.INTAKE_STARTED;
                intake2Power = 0;
//                blockerPosition = JR_OUTTAKE_BLOCK;
            } else if (gamepad1.left_bumper) {
                intake1Power = 0;
                intake2Power = 0;
                intakeStatus = INTAKE_STATUS.INTAKE_STOPPED;
            } else if (gamepad1.dpad_left) {
                intake1Power = -CLOSE_INTAKE_POWER;
            } else if (gamepad2.a){
                intake2Power = 1.0;
                intake1Power = CLOSE_INTAKE_POWER;
                intakeStatus = INTAKE_STATUS.INTAKE_STARTED;
                // Launching balls — reset ball count since all balls are being shot out
                if (targetOuttakeVelocity > 0) {
                    ballDetector.resetLoadedCount();
                }
            } else if (gamepad2.b){
                intake2Power = 0.0;
                intake1Power = 0;
                intakeStatus = INTAKE_STATUS.INTAKE_STOPPED;
            }
//
//            if (gamepad1.dpad_up){
//                hoodPos = 0.7;
//            } else if (gamepad1.dpad_down){
//                hoodPos = 0.2;
//            }



            // Hood: interpolate linearly based on distance to goal
            // Near (~45 in) → 0.62, Far (~130 in) → 0.35
            // Only update when change is significant to prevent jitter from LL corrections
            {
                double d = getRobotToGoalDistance();
                double newHoodPos = Range.clip(0.62 - (0.27 / 85.0) * (d - 45), 0.35, 0.62);
                if (Math.abs(newHoodPos - hoodPos) > 0.005) {
                    hoodPos = newHoodPos;
                }
            }

            hoodServo.setPosition(Range.clip(hoodPos,HOOD_MIN_POS,HOOD_MAX_POS));



            intake1Vel = intake1.getVelocity();

            if (intakeStatus == INTAKE_STATUS.INTAKE_STARTED && intake1Vel > 800) {
                intakeStatus = INTAKE_STATUS.INTAKE_NEED_TO_BE_MONITORED;
            }
            else if (intakeStatus == INTAKE_STATUS.INTAKE_NEED_TO_BE_MONITORED && intake1Vel < 200) {
                intakeStatus = INTAKE_STATUS.INTAKE_MONITORING;
                intakeTimer.reset();
            }
            else if (intakeStatus == INTAKE_STATUS.INTAKE_MONITORING && intake1Vel < 400 && intakeTimer.milliseconds() > 50) {
                intakeStatus = INTAKE_STATUS.INTAKE_JAMMED;
            }
            else if (intakeStatus == INTAKE_STATUS.INTAKE_JAMMED) {
                intake1Power = 0;
                intakeTimer.reset();
                intakeStatus = INTAKE_STATUS.INTAKE_STOPPED;
            }


            intake1Vel = intake1.getVelocity();

            intake1.setPower(intake1Power);
            intake2.setPower(intake2Power);
            telemetry();

            telemetry.update();
        }

    }

    public void initHardware() {
        initMotorOne(kP, kI, kD, F, position);
        initMotorTwo(kP, kI, kD, F, position);
        initDriveMotors();
        initAprilTag();
        ballDetector = new BallDetector(hardwareMap);
//        initCamera();
        initIntake();
        initTurret();

        // Initialize Limelight for AprilTag pose correction
        limelightLocalizer = new LimelightLocalizer();
        limelightLocalizer.init(hardwareMap, 180); // Blue Near starts facing 180°
        limelightLocalizer.setValidTagIds(20); // Only accept tag 20 for blue alliance
    }
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()
    public void initDriveMotors(){
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

    private void initIntake(){
        intake1 = hardwareMap.get(DcMotorEx.class,"intake1");
        intake1.setDirection(DcMotorSimple.Direction.REVERSE);
        intake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake2 = hardwareMap.get(DcMotor.class,"intake2");
        intake2.setDirection(DcMotorSimple.Direction.FORWARD);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void initTurret(){
        turretServo = hardwareMap.get(Servo.class,"turretServo");
        turretServo.setPosition(Range.clip(turretPos,0,1));

        hoodServo = hardwareMap.get(Servo.class,"hoodServo");
        hoodServo.setPosition(Range.clip(hoodPos,0,1));
    }
    private void initCamera(){
//        limelight3A = hardwareMap.get(Limelight3A.class,"limelight3A");
//        limelight3A.start();
    }

    /*private double getRobotToGoalDistance(){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        double distance = -1.0;

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (detection.id == 20 || detection.id == 24){
                    distance = detection.ftcPose.y;
                    break;
                }
            }
        }
        return distance;
    } // end method
     */

    public double getRobotToGoalDistance() {
        Pose pose = follower.getPose();

        double dx = BLUE_GOAL_POSITION_X - pose.getX();
        double dy = BLUE_GOAL_POSITION_Y - pose.getY();

        // Pythagorean theorem: distance = sqrt(dx^2 + dy^2)
        return Math.sqrt(dx * dx + dy * dy);
    }

    private boolean getGoal(){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
//        double id = -1.0;
        boolean goal = false;

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (detection.id == 20 || detection.id == 24){
                    goal = true;
                    break;
                }
            }
        }
        return goal;
    }

    private void initMotorOne(double kP, double kI, double kD, double F, double position) {
        outtake1 = hardwareMap.get(DcMotorEx.class, "outtake1");
        outtake1.setDirection(DcMotorEx.Direction.FORWARD);
        outtake1.setVelocityPIDFCoefficients(kP, kI, kD, F);
        outtake1.setPower(outtakeZeroPower);
        outtake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake1.setVelocity(targetOuttakeVelocity);
    }

    private void initMotorTwo(double kP, double kI, double kD, double F, double position) {
        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");
        outtake2.setDirection(DcMotorEx.Direction.REVERSE);
        outtake2.setVelocityPIDFCoefficients(kP, kI, kD, F);
        outtake2.setPower(outtakeZeroPower);
        outtake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake2.setVelocity(targetOuttakeVelocity);
    }

    public void telemetry() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("distance to goal", getRobotToGoalDistance());
        telemetry.addData("autoUpdate", autoUpdate);
        telemetry.addData("turretpos",turretPos);
        telemetry.addData("turretError", Math.toDegrees(turretError));
        telemetry.addData("angle to goal", Math.toDegrees(angleToGoal));
        telemetry.addData("x pos",follower.getPose().getX());
        telemetry.addData("y pos",follower.getPose().getY());
        double displayHeading = Math.toDegrees(follower.getHeading()) % 360;
        if (displayHeading < 0) displayHeading += 360;
        telemetry.addData("robot heading", displayHeading);
        telemetry.addData("hood",hoodPos);
        // Limelight telemetry
        telemetry.addData("LL valid", limelightLocalizer.isLastResultValid());
        telemetry.addData("LL reject", limelightLocalizer.getLastRejectReason());
        telemetry.addData("LL tags", limelightLocalizer.getLastTagCount());
        telemetry.addData("LL latency ms", limelightLocalizer.getLastLatencyMs());
        telemetry.addData("LL heading sent", limelightLocalizer.getLastImuHeadingDeg());
        telemetry.addData("LL raw yaw deg", limelightLocalizer.getLastYawDegrees());
        telemetry.addData("LL raw meters", "x=%.3f y=%.3f",
                limelightLocalizer.getLastRawX(), limelightLocalizer.getLastRawY());
        if (limelightLocalizer.getLastPose() != null) {
            Pose llPose = limelightLocalizer.getLastPose();
            telemetry.addData("LL x", llPose.getX());
            telemetry.addData("LL y", llPose.getY());
            telemetry.addData("LL heading", Math.toDegrees(llPose.getHeading()));
        }
        telemetry.addData("Intake Power", "Intake Power: " + intake1Power);
        telemetry.addData("Target Velocity", targetOuttakeVelocity);
        telemetry.addData("Intake state", intakeStatus);
        telemetry.addData("intake velocity", intake1Vel);
        telemetry.addData("Intake TIMER",intakeTimer.milliseconds());
        telemetry.addData("Outtake 1 power", outtake1.getPower());
        telemetry.addData("Outtake 2 power", outtake2.getPower());
        telemetry.addData("Outtake 1 Velocity", outtake1.getVelocity());
        telemetry.addData("Outtake 2 Velocity", outtake2.getVelocity());
    }
}