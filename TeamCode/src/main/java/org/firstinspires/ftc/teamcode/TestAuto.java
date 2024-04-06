package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SelfDriving;


@Autonomous(name = "TestAuto", group = "MecanumBot")
public class TestAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        SelfDriving base = new SelfDriving(hardwareMap, telemetry);

        waitForStart();

        base.move(270, 59.3725, 1);
    }
}
