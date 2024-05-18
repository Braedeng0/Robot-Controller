package org.firstinspires.ftc.teamcode.Unused;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@TeleOp(name = "Camera Test")
public class CameraTest extends LinearOpMode {
    public void runOpMode() {
        Camera camera = new Camera(hardwareMap, telemetry);

        camera.start();

        waitForStart();
        while (opModeIsActive()) {

        }
    }
}
