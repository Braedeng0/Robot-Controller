package org.firstinspires.ftc.teamcode.PID;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.PID.PID;

import org.firstinspires.ftc.robotcore.external.Telemetry;
//import java.util.ArrayList;
//import java.util.List;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import java.io.File;
//import java.io.FileWriter;
//import java.io.IOException;

public class PIDController {
    private final Telemetry telemetry;

    // Motors
    private final DcMotorEx m1, m2, m3, m4;

    // Odometers
    private  final DcMotorEx odoLeft, odoCenter;

    // IMU
    private final BNO055IMU imu;

    // Odometer variables
    private final double ticsToCM;

    // Robot coordinates
    private double robotX = 0;
    private double robotY = 0;
    private double robotRotation = 0;

    public PIDController(HardwareMap hardwareMap, Telemetry telemetry) {
        // Hardware map and telemetry
        this.telemetry = telemetry;

        // Initialize hardware
        // Motors
        m1 = hardwareMap.get(DcMotorEx.class, "LF");
        m2 = hardwareMap.get(DcMotorEx.class, "RF");
        m3 = hardwareMap.get(DcMotorEx.class, "LB");
        m4 = hardwareMap.get(DcMotorEx.class, "RB");

        m1.setDirection(DcMotor.Direction.REVERSE);
        m3.setDirection(DcMotor.Direction.REVERSE);

        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Odometers
        odoLeft = hardwareMap.get(DcMotorEx.class, "OdoLeft");
        odoCenter = hardwareMap.get(DcMotorEx.class, "collector");

        odoLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoCenter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        odoLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        odoCenter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        imu.initialize(params);

        // Odometer variables
        double odoDiameterCM = 4.8;
        double odoCircumferenceCM = odoDiameterCM * Math.PI;
        ticsToCM = odoCircumferenceCM / 2000;

        // Other variables
        robotX = 0;
        robotY = 0;
        robotRotation = 0;

        // Telemetry
        telemetry.addData("PID Controller", "Initialized");
        telemetry.update();
    }

    public void move(double x, double y, double rotation) {
        // Convert rotation to radians
        rotation = Math.toRadians(rotation);

        // Get robot coordinates
        robotX = odoCenter.getCurrentPosition() * ticsToCM;
        robotY = -odoLeft.getCurrentPosition() * ticsToCM;
        robotRotation = imu.getAngularOrientation().firstAngle;

        double kI = 0.000000005;
        double kP = 1;
        double kD = 10000;

        if (x == 0 || y == 0) {
            kI = 0;
        }

        // Initialize PID based on the direction
        PID xPID = new PID(kP, kI, kD, x-robotX, 1000);
        PID yPID = new PID(kP, kI, kD, y-robotY, 1000);
        PID rotationPID = new PID(1, 0.000001, 10000, rotation-robotRotation, 1000);

        // Initialize error
        double[] error = {xPID.getError(), yPID.getError(), rotationPID.getError()};

//        // Test code to plot terms over time
//        List<Double> testTerm = new ArrayList<>();
//        List<Double> time = new ArrayList<>();
//
//        // Start time
//        ElapsedTime timer = new ElapsedTime();
//        double startTime = timer.milliseconds();
//

        // While the robot is not at the target position
        while (Math.abs(error[0]) > 0.1 || Math.abs(error[1]) > 0.1 || Math.abs(error[2]) > 0.1) {
            // Calculate PID for each axis
            double[] move = {xPID.calculate(robotX, x), yPID.calculate(robotY, y), rotationPID.calculate(robotRotation, rotation)};
            error[0] = xPID.getError();
            error[1] = yPID.getError();
            error[2] = rotationPID.getError();

            // Calculate motor powers based on the move for each axis
            double p1 = move[1] + move[0]; //+ move[2];
            double p2 = move[1] - move[0]; //- move[2];
            double p3 = move[1] - move[0]; //+ move[2];
            double p4 = move[1] + move[0]; //- move[2];

            // Set the motor powers
            m1.setPower(p1);
            m2.setPower(p2);
            m3.setPower(p3);
            m4.setPower(p4);

            // Update robot coordinates
            robotX = -odoCenter.getCurrentPosition() * ticsToCM;
            robotY = -odoLeft.getCurrentPosition() * ticsToCM;
            robotRotation = imu.getAngularOrientation().firstAngle;

            // Update telemetry
            telemetry.addData("kI", kI);
            telemetry.addData("Robot X", robotX);
            telemetry.addData("Robot Y", robotY);
            telemetry.addData("Robot Rotation", robotRotation);
            telemetry.addData("Error X", error[0]);
            telemetry.addData("Error Y", error[1]);
            telemetry.addData("Error Rotation", error[2]);
            telemetry.addData("Move X", move[0]);
            telemetry.addData("Move Y", move[1]);
            telemetry.addData("Move Rotation", move[2]);
            telemetry.update();

//            // Test code to plot terms over time
//            testTerm.add(error[0]);
//            time.add(timer.milliseconds() - startTime);
        }

        // Stop motors
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);

//        // Test code to plot terms over time
//        File file = new File("Output/plot.csv");
//        try {
//            FileWriter writer = new FileWriter(file);
//            for (int i = 0; i < testTerm.size(); i++) {
//                writer.write(time.get(i) + "," + testTerm.get(i) + "\n");
//            }
//            writer.close();
//        } catch (IOException e) {
//            telemetry.addData("Error", e.getMessage());
//            telemetry.update();
//        }
    }
}
