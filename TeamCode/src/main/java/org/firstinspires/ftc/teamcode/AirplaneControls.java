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
            //Get stick positions
            double px = gamepad1.left_stick_x;
            double py = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            base.move(px, py, turn, 1);

            if (gamepad1.a) launcher.launch(); else launcher.ready();

        }
    }
}