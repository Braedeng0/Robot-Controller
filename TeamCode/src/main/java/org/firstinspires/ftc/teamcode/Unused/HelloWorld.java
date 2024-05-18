package org.firstinspires.ftc.teamcode.Unused;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name = "hello world")
public class HelloWorld extends LinearOpMode{
    public void runOpMode(){

        telemetry.addData("Hello, World!", "");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("X: ", gamepad1.left_stick_x);
        }
    }
}
