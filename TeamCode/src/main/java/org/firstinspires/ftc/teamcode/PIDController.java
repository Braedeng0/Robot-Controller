package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;

public class PIDController {
    private final Telemetry telemetry;

    // Motors
    private final DcMotorEx m1, m2, m3, m4;

    // Odometers
    private  final DcMotorEx odoLeft, odoCenter;

    // IMU
    BNO055IMU imu;

    // Odometer variables
    private final double ticsToCM;

    // PID constants for each axis
    private final double[] kP = new double[3];
    private final double[] kI = new double[3];
    private final double[] kD = new double[3];

    // PID variables
    private final double[] error = new double[3];
    private final double[] integral = new double[3];
    private final double[] derivative = new double[3];
    private final double[] prevError = new double[3];
    private final double[] move = new double[3];

    // Robot coordinates
    private double robotX = 0;
    private double robotY = 0;
    private double robotRotation = 0;

    public PIDController(HardwareMap hardwareMap, Telemetry telemetry) {
        // Hardware map and telemetry
        this.telemetry = telemetry;

        // Initialize PID constants
        // X-axis
        kP[0] = 0.1;
        kI[0] = 1;
        kD[0] = 0.1;

        // Y-axis
        kP[1] = 0.1;
        kI[1] = 1;
        kD[1] = 0.1;

        // Rotation
        kP[2] = 0.1;
        kI[2] = 1;
        kD[2] = 0.1;

        // Initialize PID variables
        for (int i = 0; i < 3; i++) {
            error[i] = 0;
            integral[i] = 0;
            derivative[i] = 0;
            prevError[i] = 0;
            move[i] = 0;
        }

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
        odoCenter = hardwareMap.get(DcMotorEx.class, "hang");

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

    public void move(double x, double y, double rotation, double speed) {
        // Convert rotation to radians
        rotation = Math.toRadians(rotation);

        // Calculate initial error for each axis so the loop runs at least once
        error[0] = x - robotX;
        error[1] = y - robotY;
        error[2] = rotation - robotRotation;

        // Initialize previous error for each axis
        for (int i = 0; i < 3; i++) {
            prevError[i] = error[i];
        }

        // Reset integral and derivative for each axis
        for (int i = 0; i < 3; i++) {
            integral[i] = 0;
            derivative[i] = 0;
        }

        // While the robot is not at the target position
        while (Math.abs(error[0]) > 5 || Math.abs(error[1]) > 5 || Math.abs(error[2]) > 1) {
            // Calculate error for each axis
            error[0] = x - robotX;
            error[1] = y - robotY;
            error[2] = rotation - robotRotation;

            // Calculate integral for each axis
            for (int i = 0; i < 3; i++) {
                integral[i] += error[i];
            }

            // Calculate derivative for each axis
            for (int i = 0; i < 3; i++) {
                derivative[i] = error[i] - prevError[i];
            }

            // Calculate move for each axis
            for (int i = 0; i < 3; i++) {
                move[i] = kP[i] * error[i] + kI[i] * integral[i] + kD[i] * derivative[i];
            }

            // Use the ratio of each move axis to scale the move values between -1 and 1
            double maxMove = Math.max(Math.abs(move[0]), Math.max(Math.abs(move[1]), Math.abs(move[2])));

            if (maxMove > 1) {
                for (int i = 0; i < 3; i++) {
                    move[i] /= maxMove;
                }
            }

            //Set Motor Power
            double targetHeading = Math.atan2(move[1], move[0]);

            double p1 = Math.sin(targetHeading + Math.PI / 4);
            double p2 = -Math.cos(targetHeading + Math.PI / 4);

            // Update motor powers
            m1.setPower(p1 * speed);
            m2.setPower(p2 * speed);
            m3.setPower(p2 * speed);
            m4.setPower(p1 * speed);

            // Update previous error for each axis
            for (int i = 0; i < 3; i++) {
                prevError[i] = error[i];
            }

            // Update robot coordinates
            robotX = odoCenter.getCurrentPosition() * ticsToCM;
            robotY = -odoLeft.getCurrentPosition() * ticsToCM;
            robotRotation = imu.getAngularOrientation().firstAngle;

            // Update telemetry
            telemetry.addData("Robot X", robotX);
            telemetry.addData("Robot Y", robotY);
            telemetry.addData("Robot Rotation", robotRotation);
            telemetry.addData("Error X", error[0]);
            telemetry.addData("Error Y", error[1]);
            telemetry.addData("Error Rotation", error[2]);
            telemetry.addData("Move X", move[0]);
            telemetry.addData("Move Y", move[1]);
            telemetry.addData("Move Rotation", move[2]);
            telemetry.addData("Target Heading", targetHeading);
            telemetry.addData("P1", p1);
            telemetry.addData("P2", p2);
            telemetry.update();
        }

        // Stop motors
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
    }
}