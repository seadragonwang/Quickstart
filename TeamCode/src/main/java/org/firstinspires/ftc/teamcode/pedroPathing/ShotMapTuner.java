package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.CONSTANTS.BLUE_GOAL_POSITION_X;
import static org.firstinspires.ftc.teamcode.CONSTANTS.BLUE_GOAL_POSITION_Y;
import static org.firstinspires.ftc.teamcode.CONSTANTS.RED_GOAL_POSITION_X;
import static org.firstinspires.ftc.teamcode.CONSTANTS.RED_GOAL_POSITION_Y;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "ShotMapTuner", group = "Tuning")
public class ShotMapTuner extends LinearOpMode {

    private Follower follower;

    private DcMotorEx outtake1, outtake2;
    private DcMotorEx intake1;
    private DcMotor intake2;
    private Servo hoodServo;

    private double hood = 0.80;
    private double vel = 1450;

    // step sizes
    private static final double HOOD_STEP_FINE = 0.001;
    private static final double HOOD_STEP_COARSE = 0.005;
    private static final double VEL_STEP_FINE = 10;
    private static final double VEL_STEP_COARSE = 50;

    // debounce
    private boolean lastA, lastB, lastX, lastY, lastDU, lastDD, lastDL, lastDR, lastLB, lastRB;
    private boolean lastStart1;
    private boolean flywheelOn = false;
    private boolean redAlliance = false; // toggle with gamepad1.start

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, Math.PI/2));
        follower.startTeleopDrive();

        outtake1 = hardwareMap.get(DcMotorEx.class, "outtake1");
        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        outtake1.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake1.setVelocityPIDFCoefficients(
                org.firstinspires.ftc.teamcode.CONSTANTS.kP,
                org.firstinspires.ftc.teamcode.CONSTANTS.kI,
                org.firstinspires.ftc.teamcode.CONSTANTS.kD,
                14.1);
        outtake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake1.setVelocity(0);

        outtake2.setDirection(DcMotorSimple.Direction.FORWARD);
        outtake2.setVelocityPIDFCoefficients(
                org.firstinspires.ftc.teamcode.CONSTANTS.kP,
                org.firstinspires.ftc.teamcode.CONSTANTS.kI,
                org.firstinspires.ftc.teamcode.CONSTANTS.kD,
                14.1);
        outtake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake2.setVelocity(0);

        intake1 = hardwareMap.get(DcMotorEx.class, "intake1");
        intake1.setDirection(DcMotorSimple.Direction.REVERSE);
        intake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        intake2.setDirection(DcMotorSimple.Direction.FORWARD);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            follower.update();

            // drive for positioning
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;
            follower.setTeleOpDrive(y, -x, rx);

            // controls:
            // D-pad up/down: velocity fine +/-10  (hold to repeat)
            // D-pad right/left: velocity coarse +/-50  (hold to repeat)
            // A/B: hood fine +/-  (hold to repeat)
            // Y/X: hood coarse +/-  (hold to repeat)
            // RB: toggle flywheel on/off
            if (gamepad2.dpad_up)    vel += VEL_STEP_FINE;
            if (gamepad2.dpad_down)  vel -= VEL_STEP_FINE;
            if (gamepad2.dpad_right) vel += VEL_STEP_COARSE;
            if (gamepad2.dpad_left)  vel -= VEL_STEP_COARSE;

            if (gamepad2.a) hood += HOOD_STEP_FINE;
            if (gamepad2.b) hood -= HOOD_STEP_FINE;
            if (gamepad2.y) hood += HOOD_STEP_COARSE;
            if (gamepad2.x) hood -= HOOD_STEP_COARSE;

            if (edge(gamepad2.right_bumper, lastRB)) flywheelOn = !flywheelOn;
            if (edge(gamepad2.left_bumper,  lastLB)) {
                flywheelOn = false;
                outtake1.setVelocity(0);
                outtake2.setVelocity(0);
            }

            // Apply velocity every loop when on — so d-pad adjustments take effect immediately
            vel = Range.clip(vel, 900, 2200);
            hood = Range.clip(hood, 0.23, 0.93);

            if (flywheelOn) {
                outtake1.setVelocity(vel);
                outtake2.setVelocity(vel);
            }

            // Intake controls (gamepad1):
            // RB = intake in, LB = intake reverse, neither = stop
            double intakePower = 0.0;
            if (gamepad1.right_bumper) {
                intakePower = 1.0;
            } else if (gamepad1.left_bumper) {
                intakePower = -1.0;
            }
            intake1.setPower(intakePower);
            intake2.setPower(intakePower);

            hoodServo.setPosition(hood);
            Pose p = follower.getPose();
            double battery = getBatteryVoltage();

            // Toggle alliance with gamepad1 START
            if (edge(gamepad1.start, lastStart1)) redAlliance = !redAlliance;
            lastStart1 = gamepad1.start;

            double goalX = redAlliance ? RED_GOAL_POSITION_X : BLUE_GOAL_POSITION_X;
            double goalY = redAlliance ? RED_GOAL_POSITION_Y : BLUE_GOAL_POSITION_Y;
            double dx = goalX - p.getX();
            double dy = goalY - p.getY();
            double distToGoal = Math.sqrt(dx * dx + dy * dy);

            telemetry.addLine("Tune shot, then write down: distance, hood, vel, battery");
            telemetry.addData("alliance", redAlliance ? "RED" : "BLUE");
            telemetry.addData("distance to goal (in)", distToGoal);
            telemetry.addData("x", p.getX());
            telemetry.addData("y", p.getY());
            telemetry.addData("heading deg", Math.toDegrees(p.getHeading()));
            telemetry.addData("hood cmd", hood);
            telemetry.addData("flywheel ON", flywheelOn);
            telemetry.addData("vel cmd", vel);
            telemetry.addData("vel1 actual", outtake1.getVelocity());
            telemetry.addData("vel2 actual", outtake2.getVelocity());
            telemetry.addData("battery V", battery);
            telemetry.addData("intake power", intakePower);
            telemetry.addData("intake1 vel", intake1.getVelocity());

            // Press START to "mark" current point (just visual cue for driver)
            if (gamepad2.start) {
                telemetry.addLine("MARK THIS POINT NOW");
            }

            telemetry.update();

            // Rate-limit to ~20 Hz so hold-to-repeat adjustments don't change too fast
            sleep(50);

            // update last states
            lastA = gamepad2.a; lastB = gamepad2.b; lastX = gamepad2.x; lastY = gamepad2.y;
            lastDU = gamepad2.dpad_up; lastDD = gamepad2.dpad_down; lastDL = gamepad2.dpad_left; lastDR = gamepad2.dpad_right;
            lastLB = gamepad2.left_bumper; lastRB = gamepad2.right_bumper;
            // lastStart1 updated above where it's used
        }
    }

    private boolean edge(boolean now, boolean prev) { return now && !prev; }

    private double getBatteryVoltage() {
        double min = Double.POSITIVE_INFINITY;
        for (VoltageSensor vs : hardwareMap.voltageSensor) {
            double v = vs.getVoltage();
            if (v > 0) min = Math.min(min, v);
        }
        return Double.isInfinite(min) ? 12.0 : min;
    }
}
