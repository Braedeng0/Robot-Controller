package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.PID.PIDController;

@Autonomous(name = "TestAuto", group = "MecanumBot")
public class TestAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        PIDController base = new PIDController(hardwareMap, telemetry);

        waitForStart();

        base.move(50, 30, 0);
    }
}
