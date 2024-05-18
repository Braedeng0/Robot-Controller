package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot.MecanumBase;

@TeleOp(name = "North Controls", group = "MecanumBot")
public class NorthControls extends LinearOpMode{
    public void runOpMode(){
        MecanumBase base = new MecanumBase(hardwareMap, telemetry);

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
        }
    }
}
