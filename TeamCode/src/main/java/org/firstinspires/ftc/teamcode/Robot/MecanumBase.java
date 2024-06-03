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

        m1.setDirection(DcMotor.Direction.REVERSE); // For new robot
        m3.setDirection(DcMotor.Direction.REVERSE); // For old robot
        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        m3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        m4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        move(x, y, 1);
    }

    public void move(double x, double y, double speedMultiplier) {
        // args: x (-1 to 1), y (-1 to 1)

        x = -x;

        double r = Math.hypot(x, y);
        double angle = Math.atan2(y, x);
        telemetry.addData("Angle: ", angle);

        double p1 = Math.sin(angle - Math.PI/4) * r;
        double p2 = Math.cos(angle - Math.PI/4) * r;
        telemetry.addData("p1: ", p1);
        telemetry.addData("p2: ", p2);

//        telemetry.update();

        m1.setPower(p1 * speedMultiplier);
        m2.setPower(p2 * speedMultiplier);
        m3.setPower(p2 * speedMultiplier);
        m4.setPower(p1 * speedMultiplier);
    }
}