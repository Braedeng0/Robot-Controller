package org.firstinspires.ftc.teamcode.Unused;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.AprilTag.AprilTagProcessor;


@TeleOp(name = "AprilTagOpMode", group = "AprilTag")
public class AprilTagOpMode extends LinearOpMode {
    public void runOpMode() {

        AprilTagProcessor processor = new AprilTagProcessor(hardwareMap, telemetry);

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Get the tag id
                int[] tagId = processor.getTagID();

                // Get position data
                double[] XYZ = processor.getXYZ();

                // Get orientation data
                double[] pry = processor.getPRY();

                // Telemetry
                telemetry.addData("Tag ID: ", tagId[0]);
                telemetry.addData("X: ", XYZ[0]);
                telemetry.addData("Y: ", XYZ[1]);
                telemetry.addData("Z: ", XYZ[2]);
                telemetry.addData("Pitch: ", pry[0]);
                telemetry.addData("Roll: ", pry[1]);
                telemetry.addData("Yaw: ", pry[2]);
                telemetry.update();

                // Share the CPU.
                sleep(20);
            }
        }
    }
}