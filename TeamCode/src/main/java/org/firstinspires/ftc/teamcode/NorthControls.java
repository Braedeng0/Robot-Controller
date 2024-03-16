package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.MecanumBase;

@TeleOp(name = "NorthControls", group = "MecanumBot")
public class NorthControls extends LinearOpMode{
    public void runOpMode(){
        MecanumBase base = new MecanumBase(hardwareMap);

        waitForStart();

        while (opModeIsActive()){
            //Get move angle
            double px = gamepad1.left_stick_x;
            double py = -gamepad1.left_stick_y;

            double stick_angle = Math.atan2(py, px);
            double angle = ((stick_angle + 3*Math.PI/2) % (2*Math.PI)) * -1;

            //Speed
            double speed_multiplier = Math.sqrt(Math.pow(px, 2) + Math.pow(py, 2));

            //For turning
            double turn = gamepad1.right_stick_x;

            base.move(angle, speed_multiplier, turn);

            if(gamepad1.a) {
                base.reset();
            } else {
                base.resetStop();
            }

            telemetry.addData("Stick Angle: ", stick_angle);
            telemetry.addData("X: ", px);
            telemetry.addData("Y: ", py);
            telemetry.addData("ResetRunning: ", base.getResetRunning());
            telemetry.addData("Turn: ", base.getTurn());
            telemetry.addData("A Button: ", gamepad1.a);
            telemetry.addData("Temperature: ", base.getTemp());
            telemetry.update();
        }
    }
}