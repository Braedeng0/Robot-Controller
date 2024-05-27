package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.PID.PID;
import org.firstinspires.ftc.teamcode.Robot.MecanumBase;
import org.firstinspires.ftc.teamcode.AprilTag.AprilTagProcessor;

@Autonomous(name = "FollowAprilTag", group = "AprilTag")
public class FollowAprilTag extends LinearOpMode {
    public void runOpMode() {
        AprilTagProcessor processor = new AprilTagProcessor(hardwareMap, telemetry);
        MecanumBase base = new MecanumBase(hardwareMap, telemetry);

        PID xPID = new PID(0.5, 0.05, 0, 0);
        PID yPID = new PID(0.5, 0.05, 0, 0);

        waitForStart();
        while (opModeIsActive()) {
            double[] XYZ = processor.getXYZ();

            double x = -xPID.calculate(-XYZ[0], 0); // about -10 to 10 (negative is left)
            double y = yPID.calculate(XYZ[1], 20); // about 15 to 40 (negative is forward)

            // Limit the x and y values to -1 to 1
            x = Math.max(-1, Math.min(1, x));
            y = Math.max(-1, Math.min(1, y));

            telemetry.addData("X: ", XYZ[0]);
            telemetry.addData("Y: ", XYZ[1]);
            telemetry.addData("PID X: ", x);
            telemetry.addData("PID Y: ", y);
            telemetry.update();
        }
    }
}
