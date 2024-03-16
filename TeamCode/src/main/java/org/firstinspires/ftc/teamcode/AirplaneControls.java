package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.checker.units.qual.A;


@TeleOp(name = "Airplane Controls")
public class AirplaneControls extends LinearOpMode{
    public void runOpMode(){
        MecanumBase base = new MecanumBase(hardwareMap);
        AirplaneLauncher launcher = new AirplaneLauncher(hardwareMap);

        waitForStart();

        while (opModeIsActive()){
            double px = gamepad1.left_stick_x;
            double py = -gamepad1.left_stick_y;

            double stick_angle = Math.atan2(py, px);
            double angle = ((stick_angle + 3*Math.PI/2) % (2*Math.PI)) * -1;

            //Speed
            double speed_multiplier = Math.sqrt(Math.pow(px, 2) + Math.pow(py, 2));

            //For turning
            double turn = gamepad1.right_stick_x;

            base.move(angle, speed_multiplier, turn);

            if (gamepad1.a) launcher.launch(); else launcher.ready();

        }
    }
}