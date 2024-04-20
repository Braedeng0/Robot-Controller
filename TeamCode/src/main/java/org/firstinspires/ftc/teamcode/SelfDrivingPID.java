package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class SelfDrivingPID {
    DcMotorEx m1, m2, m3, m4, odoLeft, odoRight, odoCenter;
    private double odoDiameterCM, odoCircumferenceCM, ticsToCM;
    private double odoLeftCM, odoRightCM, odoCenterCM;
    private double currentDistance;
    private Telemetry telemetry;

    public SelfDrivingPID(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        m1 = hardwareMap.get(DcMotorEx.class, "LF");
        m2 = hardwareMap.get(DcMotorEx.class, "RF");
        m3 = hardwareMap.get(DcMotorEx.class, "LB");
        m4 = hardwareMap.get(DcMotorEx.class, "RB");
        odoLeft = hardwareMap.get(DcMotorEx.class, "OdoLeft");
        odoCenter = hardwareMap.get(DcMotorEx.class, "collector");
        odoRight = hardwareMap.get(DcMotorEx.class, "hang");

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

        odoLeft.setDirection(DcMotor.Direction.REVERSE);

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        imu.initialize(params);

        odoDiameterCM = 4.8;
        odoCircumferenceCM = odoDiameterCM * Math.PI;
        ticsToCM = odoCircumferenceCM / 2000;
    }

    public void move(double x, double y, double speed) {
        // Initialize PID variables for x and y
        double kPx = 0.1;
        double kIx = 0.1;
        double kDx = 0.1;
        double kPy = 0.1;
        double kIy = 0.1;
        double kDy = 0.1;

        // Initialize previous error variables for x and y
        double prevErrorX = 0;
        double prevErrorY = 0;

        // Initialize integral and derivative variables for x and y
        double integralX = 0;
        double integralY = 0;
        double derivativeX = 0;
        double derivativeY = 0;

        // Define coordinate system with robot at (0, 0)
        double x0 = 0;
        double y0 = 0;

        // Reset Odometers
        odoLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoCenter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Initialize error for x and y
        double errorX = x - x0;
        double errorY = y - y0;

        // While error is greater than 1, continue moving
        while (Math.abs(errorX) > 1 || Math.abs(errorY) > 1) {
            // Set x and y
            x0 = odoCenter.getCurrentPosition() * ticsToCM;
            y0 = odoLeft.getCurrentPosition() * ticsToCM;

            // Calculate error for x and y
            errorX = x - x0;
            errorY = y - y0;

            // Calculate derivative for x and y
            derivativeX = errorX - prevErrorX;
            derivativeY = errorY - prevErrorY;

            // Calculate integral for x and y
            // Anti-windup logic
            double epsilon = 0.00001;
            double maxIntegralX = Math.abs(kPx / (kIx + epsilon)); // Maximum allowed integral based on gains
            double maxIntegralY = Math.abs(kPy / (kIy + epsilon));

            integralX = Math.max(-maxIntegralX, Math.min(integralX, maxIntegralX));
            integralY = Math.max(-maxIntegralY, Math.min(integralY, maxIntegralY));

            // Calculate movement in x and y
            double moveX = kPx * errorX + kIx * integralX + kDx * derivativeX;
            double moveY = kPy * errorY + kIy * integralY + kDy * derivativeY;

            // Calculate power for motors based on movement in x and y
            double heading = Math.atan2(moveY, moveX) + Math.PI / 4;
            double p1 = Math.sin(heading) * speed;
            double p2 = Math.cos(heading) * speed;

            // Run motors
            m1.setPower(p1);
            m2.setPower(p2);
            m3.setPower(p2);
            m4.setPower(p1);

            // Update previous error variables
            prevErrorX = errorX;
            prevErrorY = errorY;

            // Update telemetry
            telemetry.addData("Current X", x0);
            telemetry.addData("Current Y", y0);
            telemetry.addData("Error X", errorX);
            telemetry.addData("Error Y", errorY);
            telemetry.addData("MoveX", moveX);
            telemetry.addData("MoveY", moveY);
            telemetry.addData("Heading", heading);
            telemetry.addData("odoLeft", odoLeft.getCurrentPosition());
            telemetry.addData("odoCenter", odoCenter.getCurrentPosition());
            telemetry.update();
        }

        // Stop motors
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
    }
}
