package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.CONSTANTS.BLUE_GOAL_POSITION_X;
import static org.firstinspires.ftc.teamcode.CONSTANTS.BLUE_GOAL_POSITION_Y;
import static org.firstinspires.ftc.teamcode.CONSTANTS.HOOD_MAX_POS;
import static org.firstinspires.ftc.teamcode.CONSTANTS.HOOD_MIN_POS;
import static org.firstinspires.ftc.teamcode.CONSTANTS.MAX_TURRET_ANGLE;

import org.firstinspires.ftc.teamcode.CONSTANTS;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Launcher {
    public enum LauncherState {
        START_LAUNCHING_BLUE_NEAR,
        START_LAUNCHING_BLUE_FAR,
        START_LAUNCHING_RED_NEAR,

        START_LAUNCHING_RED_FAR,

        LAUNCH,
        IDLE,
        PICKUP
    }

    private LauncherState launcherState = LauncherState.IDLE;
    private DcMotorEx outtake1;
    private DcMotorEx outtake2;
    private DcMotor intake1;
    private DcMotor intake2;
    private Servo turretServo = null;
    private Servo hoodServo = null;
    private Timer stateTimer = new Timer();
    private Timer launchTimer = new Timer();
    public final static double JR_OUTTAKE_BLOCK = 0.78;
    public final static double JR_OUTTAKE_OPEN = 0.0;
    double FAR_OUTTAKE_VEL = 2200;
    double NEAR_OUTTAKE_VEL = 1620;
    double BLUE_NEAR_TURRET_POS = 0.42;
    double BLUE_FAR_TURRET_POS = 0.383;
    double RED_NEAR_TURRET_POS = 0.582;
    double RED_FAR_TURRET_POS = 0.617;
    double lastOuttakeVel = 0; // remember flywheel speed for LAUNCH state


    public Launcher(HardwareMap hardwareMap, PIDFCoefficients pidfCoefficients) {
        outtake1 = hardwareMap.get(DcMotorEx.class, "outtake1");
        outtake1.setDirection(DcMotorEx.Direction.FORWARD);
        outtake1.setVelocityPIDFCoefficients(pidfCoefficients.P, pidfCoefficients.I, pidfCoefficients.D, pidfCoefficients.F);

        outtake1.setVelocity(0);
        outtake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");
        outtake2.setDirection(DcMotorEx.Direction.REVERSE);
        outtake2.setVelocityPIDFCoefficients(pidfCoefficients.P, pidfCoefficients.I, pidfCoefficients.D, pidfCoefficients.F);
        outtake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake2.setVelocity(0);

        intake1 = hardwareMap.get(DcMotorEx.class,"intake1");
        intake1.setDirection(DcMotorSimple.Direction.REVERSE);
        intake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake2 = hardwareMap.get(DcMotor.class,"intake2");
        intake2.setDirection(DcMotorSimple.Direction.FORWARD);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretServo = hardwareMap.get(Servo.class,"turretServo");

        hoodServo = hardwareMap.get(Servo.class,"hoodServo");
    }

    public void setState(LauncherState state) {
        launcherState = state;
        stateTimer.resetTimer();
    }

    public void update(){
        switch (launcherState){
            case START_LAUNCHING_BLUE_NEAR:
                lastOuttakeVel = NEAR_OUTTAKE_VEL;
                outtake1.setVelocity(NEAR_OUTTAKE_VEL);
                outtake2.setVelocity(NEAR_OUTTAKE_VEL);
                intake1.setPower(0);
                intake2.setPower(0);
                turretServo.setPosition(Range.clip(BLUE_NEAR_TURRET_POS, 0.28, 0.694)); // 0.422
                hoodServo.setPosition(Range.clip(0.62,HOOD_MIN_POS,HOOD_MAX_POS));
                break;
            case START_LAUNCHING_BLUE_FAR:
                lastOuttakeVel = FAR_OUTTAKE_VEL;
                outtake1.setVelocity(FAR_OUTTAKE_VEL);
                outtake2.setVelocity(FAR_OUTTAKE_VEL);
                intake1.setPower(0);
                intake2.setPower(0);
                turretServo.setPosition(Range.clip(BLUE_FAR_TURRET_POS, 0.28, 0.694)); // 0.422
                hoodServo.setPosition(Range.clip(0.383,HOOD_MIN_POS,HOOD_MAX_POS));
                break;
            case START_LAUNCHING_RED_NEAR:
                lastOuttakeVel = NEAR_OUTTAKE_VEL;
                outtake1.setVelocity(NEAR_OUTTAKE_VEL);
                outtake2.setVelocity(NEAR_OUTTAKE_VEL);
                intake1.setPower(0);
                intake2.setPower(0);
                turretServo.setPosition(Range.clip(RED_NEAR_TURRET_POS, 0.28, 0.694));
                hoodServo.setPosition(Range.clip(0.62,HOOD_MIN_POS,HOOD_MAX_POS));
                break;
            case START_LAUNCHING_RED_FAR:
                lastOuttakeVel = FAR_OUTTAKE_VEL;
                outtake1.setVelocity(FAR_OUTTAKE_VEL);
                outtake2.setVelocity(FAR_OUTTAKE_VEL);
                intake1.setPower(0);
                intake2.setPower(0);
                turretServo.setPosition(Range.clip(RED_FAR_TURRET_POS, 0.28, 0.694));
                hoodServo.setPosition(Range.clip(0.39,HOOD_MIN_POS,HOOD_MAX_POS));
                break;
            case LAUNCH:
                outtake1.setVelocity(lastOuttakeVel);
                outtake2.setVelocity(lastOuttakeVel);
                intake1.setPower(1);
                intake2.setPower(1);
                break;
            case IDLE:
                intake1.setPower(0);
                intake2.setPower(0);
                break;
            case PICKUP:
//                blocker.setPosition(JR_OUTTAKE_BLOCK);
                intake1.setPower(1);
                intake2.setPower(0);
        }
    }
    public LauncherState getState() {
        return launcherState;
    }

    public double getOuttakeVelocity() {
        return outtake1.getVelocity();
    }

    public boolean isFlywheelReady() {
        return lastOuttakeVel > 0 && Math.abs(outtake1.getVelocity()) >= lastOuttakeVel * 0.95;
    }
    public void updateTurret(Pose robotPose){
        double turretPos = 0.5;
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double robotHeading = robotPose.getHeading();

        // Transform turret offset from robot-local frame to field frame
        double turretX = robotX
                + CONSTANTS.TURRET_OFFSET_FORWARD * Math.cos(robotHeading)
                - CONSTANTS.TURRET_OFFSET_LEFT * Math.sin(robotHeading);
        double turretY = robotY
                + CONSTANTS.TURRET_OFFSET_FORWARD * Math.sin(robotHeading)
                + CONSTANTS.TURRET_OFFSET_LEFT * Math.cos(robotHeading);

        // Angle from turret position to goal
        double angleToGoal = Math.atan2(BLUE_GOAL_POSITION_Y - turretY, BLUE_GOAL_POSITION_X - turretX);

        double turretError = angleToGoal - robotHeading;
        while (turretError > Math.PI) turretError -= 2 * Math.PI;
        while (turretError < -Math.PI) turretError += 2 * Math.PI;

        double errorDegrees = Math.toDegrees(turretError);

// Clamp turret angle to ±135° to prevent over-rotation
        errorDegrees = Range.clip(errorDegrees, -MAX_TURRET_ANGLE, MAX_TURRET_ANGLE);

        turretPos = TurretMapper.degreesToServoPos(errorDegrees);
        turretServo.setPosition(Range.clip(turretPos, 0.28, 0.694));
    }

}
