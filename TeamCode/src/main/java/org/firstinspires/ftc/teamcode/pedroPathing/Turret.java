package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Turret Positions", group="OrcaRobotics")
public class Turret extends LinearOpMode {
    private Servo ts;
    private double turretPos = 0.5;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        ts = hardwareMap.get(Servo.class,"turretServo");
        ts.setPosition(turretPos);
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.x) {
                turretPos += 0.0001;
                ts.setPosition(turretPos);
            } else if (gamepad1.b) {
                turretPos -= 0.0001;
                ts.setPosition(turretPos);
            }
            telemetry.addData("turretpos",turretPos);
            telemetry.update();
        }
    }
}
