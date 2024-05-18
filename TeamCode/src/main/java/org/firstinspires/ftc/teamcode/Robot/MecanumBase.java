package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

// Your motor control class
public class MecanumBase {
    private DcMotor m1;
    private DcMotor m2;
    private DcMotor m3;
    private DcMotor m4;
    private BNO055IMU imu;
    private double turn;
    private boolean resetRunning;
    private Telemetry telemetry;
    public MecanumBase(HardwareMap hardwareMap, Telemetry telemetry) {
        m1 = hardwareMap.dcMotor.get("LF");
        m2 = hardwareMap.dcMotor.get("RF");
        m3 = hardwareMap.dcMotor.get("LB");
        m4 = hardwareMap.dcMotor.get("RB");

        m2.setDirection(DcMotor.Direction.REVERSE);
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelerationIntegrationAlgorithm = null;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;

        imu.initialize(parameters);

        resetRunning = false;

        this.telemetry = telemetry;
    }

    public void move(double x, double y) {
        // args: x (-1 to 1), y (-1 to 1)
        double r = Math.hypot(x, y);
        double angle = Math.atan2(y, x) - Math.PI / 4;

        double p1 = Math.sin(angle + Math.PI / 4) * r;
        double p2 = Math.cos(angle + Math.PI / 4) * r;

        m1.setPower(p1);
        m2.setPower(p2);
        m3.setPower(p2);
        m4.setPower(p1);
    }
}