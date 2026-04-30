package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Outtake Tuner", group="OrcaRobotics")
public class outtakeTuner extends LinearOpMode {
    private DcMotorEx ts;
    private double velocity = 0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        ts = hardwareMap.get(DcMotorEx.class,"outtake2");
        ts.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ts.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ts.setDirection(DcMotorEx.Direction.FORWARD);
        ts.setVelocity(velocity);
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.x) {
                velocity = 1600;
            }
            ts.setVelocity(velocity);
            telemetry.update();
        }
    }
}
