package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Temperature;

// Your motor control class
public class MecanumBase {
    private DcMotor m1;
    private DcMotor m2;
    private DcMotor m3;
    private DcMotor m4;
    private BNO055IMU imu;
    private double turn;
    private boolean resetRunning;
    public MecanumBase(HardwareMap hardwareMap) {
        m1 = hardwareMap.dcMotor.get("LF");
        m2 = hardwareMap.dcMotor.get("RF");
        m3 = hardwareMap.dcMotor.get("LB");
        m4 = hardwareMap.dcMotor.get("RB");

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

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelerationIntegrationAlgorithm = null;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;

        imu.initialize(parameters);

        resetRunning = false;

    }

    public void move(double stick_angle, double speed_multiplier, double pxr) {
        //Get orientation
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double heading = orientation.firstAngle * -1;
        double targetHeading = stick_angle - heading;

        //Set Motor Power
        double p1 = Math.sin(targetHeading + Math.PI / 4) * speed_multiplier;
        double p2 = Math.cos(targetHeading + Math.PI / 4) * speed_multiplier;

        turn = pxr;


        m1.setPower(p1 + turn);
        m2.setPower(p2 - turn);
        m3.setPower(p2 + turn);
        m4.setPower(p1 - turn);
    }
    //test
    public void reset() {
        //Get orientation
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double heading = orientation.firstAngle * -1;
        resetRunning = true;

        if (heading > 0) {
            this.turn = -1;
            while (heading > 1) {
                orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
                heading = orientation.firstAngle * -1;
            }
        } else {
            this.turn = 1;
            while (heading < -1) {
                orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
                heading = orientation.firstAngle * -1;
            }
        }
    }

    public void resetStop() {
        resetRunning = false;
    }

    public double getTurn(){
        return turn;
    }

    public boolean getResetRunning(){
        return resetRunning;
    }

    public double getTemp(){
        Temperature temp = imu.getTemperature();
        return temp.temperature;
    }
}
