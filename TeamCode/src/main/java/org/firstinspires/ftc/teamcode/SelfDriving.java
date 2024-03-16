package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class SelfDriving {
    DcMotorEx m1, m2, m3, m4, odoLeft, odoRight, odoCenter;
    private double odoDiameterCM, odoCircumferenceCM, ticsToCM;
    private double odoLeftCM, odoRightCM, odoCenterCM;
    private double currentDistance;
    private Telemetry telemetry;

    public SelfDriving(HardwareMap hardwareMap, Telemetry telemetry) {
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

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        imu.initialize(params);

        odoDiameterCM = 4.6;
        odoCircumferenceCM = odoDiameterCM * Math.PI;
        ticsToCM = odoCircumferenceCM / 2000;
    }

    public void move(double heading, double distance, double speed) {
        //Absolute Value of Distance
        distance = Math.abs(distance);

        //Reset Odometers
        odoLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoCenter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set Motor Power
        double p1 = Math.sin(heading + Math.PI / 4) * speed;
        double p2 = Math.cos(heading + Math.PI / 4) * speed;

        //Calculate current distance
        odoLeftCM = -odoLeft.getCurrentPosition() * ticsToCM;
        odoCenterCM = odoCenter.getCurrentPosition() * ticsToCM;

        currentDistance = Math.sqrt(Math.pow(odoLeftCM, 2) + Math.pow(odoCenterCM, 2));

        while (currentDistance < distance) {
            //Run Motors
            m1.setPower(p1);
            m2.setPower(p2);
            m3.setPower(p2);
            m4.setPower(p1);

            telemetry.addData("Current Distance", currentDistance);
            telemetry.update();
        }

        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
    }
}
