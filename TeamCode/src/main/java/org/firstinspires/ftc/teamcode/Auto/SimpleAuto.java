package org.firstinspires.ftc.teamcode.Auto;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.MecanumBase;

@Autonomous(name="SimpleAuto")
public class SimpleAuto extends LinearOpMode {
    DcMotorEx m1, m2, m3, m4;

    public void runOpMode() {
        ElapsedTime timer = new ElapsedTime();
        MecanumBase base = new MecanumBase(hardwareMap, telemetry);

        waitForStart();
        timer.reset();

        while(opModeIsActive()) {
            if (timer.seconds() <= 1) {
                base.move(0,1, 0.5);
            } else if (1 < timer.seconds() && timer.seconds() <= 2) {
                base.move(1,0, 0.5);
            } else if (2 < timer.seconds() && timer.seconds() <= 3) {
                base.move(0,-1, 0.5);
            } else if (3 < timer.seconds() && timer.seconds() <= 4) {
                base.move(-1,1, 0.5);
            } else {
                base.move(0,0);
                terminateOpModeNow();
            }
        }
    }
}
