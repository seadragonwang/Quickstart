package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="CheckServo", group="OrcaRobotics")
@Disabled
public class CheckServo extends LinearOpMode {
    protected double hoodPos = 0.5;
    private Servo hoodServo;

    @Override
    public void runOpMode() throws InterruptedException {
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        hoodServo.setPosition(Range.clip(hoodPos, 0, 1));
        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.a){
                hoodPos += 0.001;
            } else if (gamepad1.b) {
                hoodPos -= 0.001;
            }
            hoodServo.setPosition(hoodPos);
            telemetry.addData("Hood Position", hoodPos);
            telemetry.update();
        }
    }
}

