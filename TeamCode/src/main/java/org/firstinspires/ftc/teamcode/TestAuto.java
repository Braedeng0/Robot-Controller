package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name = "TestAuto", group = "MecanumBot")
public class TestAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        PIDController base = new PIDController(hardwareMap, telemetry);
        SlideArm arm = new SlideArm(hardwareMap, telemetry);

        waitForStart();

        arm.breakOnStop(true);
        arm.setPosition(100, 0.5);

        while (true) {}
    }
}