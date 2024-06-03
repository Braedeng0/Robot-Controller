package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PID.PID;
import org.firstinspires.ftc.teamcode.Robot.MecanumBase;
import org.firstinspires.ftc.teamcode.AprilTag.AprilTagProcessor;

@Autonomous(name = "FollowAprilTag", group = "AprilTag")
public class FollowAprilTag extends LinearOpMode {

    public void runOpMode() {
        AprilTagProcessor processor = new AprilTagProcessor(hardwareMap, telemetry);
        MecanumBase base = new MecanumBase(hardwareMap, telemetry);

        PID xPID = new PID(0.1, 0.0005, 0, 0);
        PID yPID = new PID(0.1, 0.0005, 0, 0);

        double[] prevMove = {0, 0};
        double x = 0;
        double y = 0;

        ElapsedTime timer = new ElapsedTime();
        boolean noTags = false;

        waitForStart();
        while (opModeIsActive()) {
            double[] XYZ = processor.getXYZ();
            int[] tagID = processor.getTagID();

            if (tagID.length > 0) {
                x = xPID.calculate(-XYZ[0], 0); // about -10 to 10 (negative is left)
                y = -yPID.calculate(XYZ[1], 20); // about 15 to 40 (negative is forward)

                prevMove[0] = x;
                prevMove[1] = y;

                noTags = false;
            } else {
                // Check if there were no tags in the last second
                if (noTags) {
                    if (timer.seconds() < 0.2) {
                        // Use the previous move if there are no tags for less than a second
                        x = prevMove[0];
                        y = prevMove[1];
                    } else {
                        // Reset the x and y values to 0 if there are no tags for more than a second
                        x = 0;
                        y = 0;
                    }
                } else {
                    // Start the timer if there are no tags
                    timer.reset();
                    noTags = true;
                }
            }

            // Limit the x and y values to -1 to 1
            x = Math.max(-1, Math.min(1, x));
            y = Math.max(-1, Math.min(1, y));

            base.move(x, y, 1);

            telemetry.addData("X: ", XYZ[0]);
            telemetry.addData("Y: ", XYZ[1]);
            telemetry.addData("PID X: ", x);
            telemetry.addData("PID Y: ", y);
            telemetry.addData("Number of tags: ", tagID.length);
            telemetry.update();
        }
    }
}
