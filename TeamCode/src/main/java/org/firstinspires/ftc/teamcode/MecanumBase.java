package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


// Your motor control class
public class MecanumBase {
    private final DcMotor m1;
    private final DcMotor m2;
    private final DcMotor m3;
    private final DcMotor m4;

    public MecanumBase(HardwareMap hardwareMap) {
        m1 =
        m2 = hardwareMap.dcMotor.get("front_right_motor");
        m3 = hardwareMap.dcMotor.get("back_left_motor");
        m4 = hardwareMap.dcMotor.get("back_right_motor");

        m1.setDirection(DcMotor.Direction.REVERSE);
        m3.setDirection(DcMotor.Direction.REVERSE);
        m4.setDirection(DcMotor.Direction.REVERSE);
        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void move(double px, double py, double turn, double speed_multiplier) {
        double stick_angle = Math.atan2(py, px);
//        stick_angle = ((stick_angle + 3*Math.PI/2) % (2*Math.PI)) * -1;

        //Speed
        double speed = Math.sqrt(Math.pow(px, 2) + Math.pow(py, 2));
        //Speed acceleration
        speed = Math.pow(speed, 2);

        //Set Motor Power
        double p1 = Math.sin(stick_angle + Math.PI / 4) * speed * speed_multiplier;
        double p2 = Math.cos(stick_angle + Math.PI / 4) * speed * speed_multiplier;

        //Adjust turn for speed_multiplier
        turn = turn * speed_multiplier;

        m1.setPower(p1 + turn);
        m2.setPower(p2 - turn);
        m3.setPower(p2 + turn);
        m4.setPower(p1 - turn);
    }
}
